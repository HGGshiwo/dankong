#!/usr/bin/env python
# coding=utf-8
import random
import time
import unittest

import rospy
import rostest
from mavproxy_ros.test.test_helper import Robot, http_post, sitl_env
from mavproxy_ros.test.utils import gps_distance
from rsos_msgs.msg import PointObj

MODEL_NAME = "iris_demo"
XY_THRESHOLD = 2.5  # 目标的距离阈值
Z_THRESHOLD = 1.0  # 高度的距离阈值


class TestForever(unittest.TestCase):
    def setUp(self):
        # 初始化节点（对于rostest，必须用匿名节点）
        rospy.init_node("auto_test_director", anonymous=True)
        self.robot = Robot()

    def tearDown(self):
        pass

    def test_case1(self):
        IRIS_X = random.randint(-3, 3)
        IRIS_Y = random.randint(-3, 3)
        self.robot.set_state(x=IRIS_X, y=IRIS_Y, z=0.5)

        with sitl_env():
            time.sleep(10000000)


if __name__ == "__main__":
    # 将 unittest 挂载到 rostest 框架上
    rostest.rosrun("dankong", "test_forever", TestForever)
