#!/usr/bin/env python
# coding=utf-8
import random
import time
import unittest
from test.test_helper import Apriltag, Robot, sitl_env

import rospy
import rostest
from nav_msgs.msg import Odometry

XY_THRESHOLD = 2.5  # 目标的距离阈值
Z_THRESHOLD = 1.0  # 高度的距离阈值


class Debug(unittest.TestCase):
    def setUp(self):
        # 初始化节点（对于rostest，必须用匿名节点）
        rospy.init_node("auto_test_director", anonymous=True)
        self.robot = Robot()

    def tearDown(self):
        pass

    def board_pub_worker(self):
        pub = rospy.Publisher("/pland/board_gt", Odometry, queue_size=1, latch=True)

        while True:
            xyz, rpy = self.robot.get_state()
            b_xyz, b_rpy = self.board.get_state()
            pos_enu = self.robot.state["pos_enu"]
            b_enu_x = pos_enu[0] + b_xyz[0] - xyz[0]
            b_enu_y = pos_enu[1] + b_xyz[1] - xyz[1]
            b_enu_z = pos_enu[2] + b_xyz[2] - xyz[2]
            msg = Odometry()
            msg.header.frame_id = "map"
            msg.pose.pose.position.x = b_enu_x
            msg.pose.pose.position.y = b_enu_y
            msg.pose.pose.position.z = b_enu_z
            pub.publish(msg)

    def test_case1(self):
        while True:
            try:
                IRIS_X = random.randint(-1, 1)
                IRIS_Y = random.randint(-1, 1)
                self.robot.set_state(x=IRIS_X, y=IRIS_Y, z=0.5)
                self.board = Apriltag("apriltag")
                self.board.spawn()
                self.board.set_state(
                    x=1.5,
                    y=1.5,
                    z=0.015,
                )

                with sitl_env():
                    # This prompt stays at the bottom and waits for user input

                    user_input = input("Enter anything to restart (or 'q' to quit) >")

                if user_input.strip().lower() == "q":
                    print("Exiting...")
                    break

            except KeyboardInterrupt:
                # Handle Ctrl+C gracefully
                break
            except EOFError:
                # Handle Ctrl+D gracefully
                break


if __name__ == "__main__":
    # 将 unittest 挂载到 rostest 框架上
    rostest.rosrun("dankong", "debug", Debug)
