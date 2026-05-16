#!/usr/bin/env python
# coding=utf-8
import math
import random
import time
import unittest
from test.test_helper import Robot, http_post, sitl_env
from test.utils import gps_distance

import rospy
import rostest
from rsos_msgs.srv import (
    SetGimbalAngle,
    SetGimbalAngleResponse,
    StartBagRecord,
    StartBagRecordResponse,
)
from std_srvs.srv import Trigger, TriggerResponse

XY_THRESHOLD = 3
Z_THRESHOLD = 2.0

test_waypoint = [
    [120.14099425554586, 30.111834585498475, 10],
    [120.14089461794015, 30.111973425562567, 3],
    [120.1409098335351, 30.112108444153343, 15],
    [120.14101732435114, 30.112202702305442, 5],
]

test_event_node_list = [
    [{"eventType": "video", "eventStatus": "on", "eventParam": "test_bag"}],
    [
        {"eventType": "video", "eventStatus": "off"},
        {"eventType": "hat", "eventStatus": "on"},
    ],
    [{"eventType": "gimbal", "eventStatus": "", "eventParam": "45"}],
]


class TestWaypointEvent(unittest.TestCase):
    def setUp(self):
        rospy.init_node("auto_test_event", anonymous=True)
        self.robot = Robot()

        self.start_record_called = False
        self.stop_record_called = False
        self.set_gimbal_called = False
        self.start_record_bag_name = ""
        self.set_gimbal_angle = 0.0
        self.set_gimbal_mode = ""

        rospy.Service(
            "/data_recorder/start_recording", StartBagRecord, self._cb_start_record
        )
        rospy.Service("/data_recorder/stop_recording", Trigger, self._cb_stop_record)
        rospy.Service(
            "/UAV0/sensor/serial_gimbal/set_gimbal_angle",
            SetGimbalAngle,
            self._cb_set_gimbal,
        )

    def _cb_start_record(self, req):
        self.start_record_called = True
        self.start_record_bag_name = req.prefix
        return StartBagRecord.Response(success=True, message="OK")

    def _cb_stop_record(self, req):
        self.stop_record_called = True
        return TriggerResponse(message="OK")

    def _cb_set_gimbal(self, req):
        self.set_gimbal_called = True
        self.set_gimbal_angle = req.angle
        self.set_gimbal_mode = req.mode
        return SetGimbalAngleResponse(success=True, message="OK")

    def tearDown(self):
        pass

    def _set_waypoint_with_events(self, waypoint, event_node_list):
        data = dict(waypoint=waypoint)
        if event_node_list is not None:
            data["nodeEventList"] = event_node_list

        for i in range(10):
            res = http_post("/set_waypoint", data, check=True)
            if res.get("status") == "success":
                break
            time.sleep(1)

        for i, wp in enumerate(waypoint[1:]):
            for _ in range(100):
                try:
                    res = self.robot.ws_event_queue.get(timeout=50)
                except:
                    continue
                if res.get("event") == "progress":
                    break
            else:
                raise RuntimeError("没有出现进度信息!")

            rospy.logerr(f"{i}, 收到消息!: {res}")
            assert res.get("total") == len(waypoint) - 1, res
            assert res.get("cur") == i + 1, f"{res}, {i}"
            lat = self.robot.state["lat"]
            lon = self.robot.state["lon"]
            dist = gps_distance(lon, lat, wp[0], wp[1])
            assert dist < XY_THRESHOLD, f"dist: {dist}, wp:[{wp[0]}, {wp[1]}]"
            if rospy.get_param("~robot_type") == "drone":
                dist_z = math.fabs(self.robot.state["rel_alt"] - wp[2])
                assert dist_z < Z_THRESHOLD, f"z: {dist_z}"
        self.robot.wait_for_state("state", "悬停状态")

    def test_waypoint_event(self):
        IRIS_X = random.randint(-3, 3)
        IRIS_Y = random.randint(-3, 3)
        self.robot.set_state(x=IRIS_X, y=IRIS_Y, z=0.02)

        with sitl_env():
            self.robot.init()
            self.robot.takeoff()
            http_post("/stop_pland", check=True)
            http_post("/stop_planner", check=True)

            self._set_waypoint_with_events(test_waypoint, test_event_node_list)

            assert (
                self.start_record_called
            ), "video:on 未触发 /data_recorder/start_recording"
            assert (
                self.stop_record_called
            ), "video:off 未触发 /data_recorder/stop_recording"
            assert (
                self.set_gimbal_called
            ), "gimbal: 未触发 /UAV0/sensor/serial_gimbal/set_gimbal_angle"
            assert (
                self.start_record_bag_name == "test_bag"
            ), f"bag_name 错误: {self.start_record_bag_name}"
            assert (
                abs(self.set_gimbal_angle - 45.0) < 0.01
            ), f"gimbal angle 错误: {self.set_gimbal_angle}"

            # 验证 noh硬hat 检测未在 detect_map 中设置（hat:on 会触发但被 reject）
            # 但 smoke 和 noh硬hat 相关的检测区域不会被启用（因为 hat 类型不在 detect_map 中）
            rospy.logerr("所有事件服务调用验证通过!")

            http_post("/land", check=True)
            self.robot.wait_for_state("state", "地面状态", 120)


if __name__ == "__main__":
    rostest.rosrun("dankong", "test_waypoint_event", TestWaypointEvent)
