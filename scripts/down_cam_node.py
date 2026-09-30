#!/usr/bin/env python3
"""Pull the DJI down-camera MJPEG stream from the MavDrone app and republish it
as ROS image topics.

Topics:
    <topic>/image_raw        sensor_msgs/Image          (decoded frames)
    <topic>/image_compressed sensor_msgs/CompressedImage (original JPEG)

Parameters:
    ~host    phone IP on the GCS WiFi link (same host as the MAVLink TCP link)
    ~port    MavDrone MJPEG port (default 8090)
    ~topic   base topic name (default dji/down_camera)
    ~fps     target republish rate (default 15)
"""

import os
import threading
import traceback

import cv2
import rospy
from sensor_msgs.msg import CompressedImage, Image

# 拉流是局域网直连, 忽略系统代理: FFmpeg 会读取 http_proxy 等环境变量,
# 代理(通常不在机载网段)会导致打开流失败
for _proxy_var in (
    "http_proxy",
    "https_proxy",
    "all_proxy",
    "HTTP_PROXY",
    "HTTPS_PROXY",
    "ALL_PROXY",
):
    os.environ.pop(_proxy_var, None)


class DownCamNode:
    def __init__(self):
        rospy.init_node("down_cam_node")

        self.host = rospy.get_param("~host", "")
        self.port = rospy.get_param("~port", 8090)
        self.topic = rospy.get_param("~topic", "dji/down_camera/image_raw").strip("/")
        self.rate = rospy.Rate(rospy.get_param("~fps", 15.0))

        self.pub_image = rospy.Publisher(self.topic, Image, queue_size=1)

        if not self.host:
            rospy.logerr(
                "[down_cam] ~host is not set! Point it at the phone IP on the "
                "GCS WiFi link (the same host used for the MAVLink TCP link)."
            )

        self.url = "http://{}:{}/stream".format(self.host, self.port)
        self.stamp = rospy.Time.now()
        self.stamp_lock = threading.Lock()
        # Grab frames on a worker thread so that slow reads do not stall publishing
        self.frame = None
        # Bind the condition to stamp_lock: wait()/notify_all() must run while the
        # caller holds exactly this lock
        self.frame_condition = threading.Condition(self.stamp_lock)
        self.conversion_traceback_logged = False

    def grab_loop(self):
        cap = cv2.VideoCapture(self.url, cv2.CAP_FFMPEG)
        if not cap.isOpened():
            rospy.logerr("[down_cam] cannot open stream: %s", self.url)
            return

        while not rospy.is_shutdown():
            ret, cv_image = cap.read()
            if not ret:
                rospy.logwarn_throttle(5.0, "[down_cam] stream read failed, reopening")
                cap.release()
                rospy.sleep(1.0)
                cap = cv2.VideoCapture(self.url, cv2.CAP_FFMPEG)
                if not cap.isOpened():
                    continue
                continue

            with self.stamp_lock:
                self.stamp = rospy.Time.now()
                self.frame = cv_image
                self.frame_condition.notify_all()

        cap.release()

    def publish_loop(self):
        while not rospy.is_shutdown():
            with self.stamp_lock:
                while self.frame is None and not rospy.is_shutdown():
                    self.frame_condition.wait(timeout=0.5)
                cv_image = self.frame
                stamp = self.stamp
                self.frame = None

            if cv_image is None:
                continue

            try:
                # FFMPEG 解码偶发返回 4 通道帧, 统一转成 BGR
                if cv_image.ndim == 3 and cv_image.shape[2] == 4:
                    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGRA2BGR)
                # 手工填充 Image (绕开系统 cv_bridge 与当前 numpy/OpenCV
                # 组合的 KeyError: 16 兼容性问题)
                msg = Image()
                msg.height, msg.width = cv_image.shape[0], cv_image.shape[1]
                msg.encoding = "bgr8"
                msg.is_bigendian = 0
                msg.step = int(cv_image.shape[1] * 3)
                msg.data = cv_image.tobytes()
                msg.header.stamp = stamp
                self.pub_image.publish(msg)
            except (
                Exception
            ) as exc:  # cv_bridge conversion errors must not kill the node
                rospy.logwarn_throttle(
                    5.0,
                    "[down_cam] conversion failed: %r frame=%s",
                    exc,
                    (
                        (cv_image.shape, str(cv_image.dtype))
                        if cv_image is not None
                        else None
                    ),
                )
                if not self.conversion_traceback_logged:
                    self.conversion_traceback_logged = True
                    rospy.logerr(
                        "[down_cam] first conversion failure traceback:\n%s",
                        traceback.format_exc(),
                    )
                continue
            self.rate.sleep()

    def run(self):
        grab_thread = threading.Thread(target=self.grab_loop, daemon=True)
        grab_thread.start()
        self.publish_loop()


if __name__ == "__main__":
    try:
        DownCamNode().run()
    except rospy.ROSInterruptException:
        pass
