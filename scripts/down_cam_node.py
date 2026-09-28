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

import threading

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image


class DownCamNode:
    def __init__(self):
        rospy.init_node("down_cam_node")

        self.host = rospy.get_param("~host", "")
        self.port = rospy.get_param("~port", 8090)
        self.topic = rospy.get_param("~topic", "dji/down_camera/image_raw").strip("/")
        self.rate = rospy.Rate(rospy.get_param("~fps", 15.0))

        self.bridge = CvBridge()
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
                msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
                msg.header.stamp = stamp
                self.pub_image.publish(msg)
            except (
                Exception
            ) as exc:  # cv_bridge conversion errors must not kill the node
                rospy.logwarn_throttle(
                    5.0, "[down_cam] conversion failed: %s", str(exc)
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
