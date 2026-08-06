#!/usr/bin/env python
# coding=utf-8
import math
import random
import threading
import time
import unittest
from test.test_helper import Apriltag, Robot, sitl_env

import requests
import rospy
import rostest
from nav_msgs.msg import Odometry

XY_THRESHOLD = 2.5  # 目标的距离阈值
Z_THRESHOLD = 1.0  # 高度的距离阈值


class BoardController:
    def __init__(self, board, robot):
        self.board = board
        self.robot = robot
        self.moving = False
        self.thread = None
        self.lock = threading.Lock()
        self.move_thread = None

    def start(self):
        with self.lock:
            if not self.moving:
                self.moving = True

                # 设置步进回调 (前进速度 enu.x=1.0)
                self.board.step_cb = lambda: (0.0, -1.0, 0.0)
                self.board.before_cb = lambda writer: None
                self.board.is_moving.set()

                # 启动 test_helper 中的控制线程，向 /apriltag/cmd_vel 发布速度
                self.move_thread = threading.Thread(
                    target=self.board.move_worker, daemon=True
                )
                self.move_thread.start()

                # 启动 GPS 目标上报线程
                self.thread = threading.Thread(target=self.run, daemon=True)
                self.thread.start()

    def stop(self):
        with self.lock:
            if self.moving:
                self.moving = False
                self.board.is_moving.clear()
                if self.move_thread:
                    self.move_thread = None
                if self.thread:
                    self.thread = None

    def run(self):
        while True:
            with self.lock:
                if not self.moving:
                    break

            # 直接从 Gazebo 仿真中获取板子的实际位姿和速度，不再手动累加位置
            res = self.board.get_state()
            if not res:
                time.sleep(0.05)
                continue
            b_pos, b_vel, b_rpy = res
            b_x, b_y, b_z = b_pos
            vx, vy, vz = b_vel

            # 获取无人机的 GPS
            try:
                resp = requests.get("http://localhost:8000/get_gps", timeout=0.1)
                if resp.status_code == 200:
                    data = resp.json()
                    if data.get("status") == "success":
                        drone_lat, drone_lon, drone_alt = data["msg"]

                        # 获取无人机的 ENU 状态
                        r_res = self.robot.get_state()
                        if r_res:
                            r_pos, r_vel, r_rpy = r_res
                            r_x, r_y, r_z = r_pos

                            # 计算板子的 GPS (根据相对于无人机的位置偏差)
                            d_north = b_x - r_x
                            d_east = r_y - b_y
                            d_up = b_z - r_z

                            R_earth = 6378137.0
                            d_lat = d_north / R_earth
                            b_lat = drone_lat + math.degrees(d_lat)
                            d_lon = d_east / (
                                R_earth * math.cos(math.radians(drone_lat))
                            )
                            b_lon = drone_lon + math.degrees(d_lon)
                            b_alt = drone_alt + d_up

                            # 发送目标信息到服务器 (ENU 坐标系下的速度)
                            vel_east = -vy
                            vel_north = vx
                            vel_up = vz

                            post_data = {
                                "position": [b_lon, b_lat, b_alt],
                                "velocity": [vel_east, vel_north, vel_up],
                            }
                            requests.post(
                                "http://localhost:8000/pland_target/set",
                                json=post_data,
                                timeout=0.1,
                            )
            except Exception as e:
                # 忽略连接或请求错误
                pass

            time.sleep(0.05)


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
            res_r = self.robot.get_state()
            res_b = self.board.get_state()
            if not res_r or not res_b:
                time.sleep(0.1)
                continue
            xyz, _, rpy = res_r
            b_xyz, _, b_rpy = res_b
            pos_enu = self.robot.state.get("pos_enu", [0.0, 0.0, 0.0])
            # Gazebo coordinates: X=North, Y=-East, Z=Up. ENU coordinates: X=East, Y=North, Z=Up.
            b_enu_x = pos_enu[0] + (xyz[1] - b_xyz[1])
            b_enu_y = pos_enu[1] + (b_xyz[0] - xyz[0])
            b_enu_z = pos_enu[2] + (b_xyz[2] - xyz[2])
            msg = Odometry()
            msg.header.frame_id = "map"
            msg.pose.pose.position.x = b_enu_x
            msg.pose.pose.position.y = b_enu_y
            msg.pose.pose.position.z = b_enu_z
            pub.publish(msg)
            time.sleep(0.02)

    def test_case1(self):
        while True:
            try:
                # IRIS_X = random.randint(-1, 1)
                # IRIS_Y = random.randint(-1, 1)
                IRIS_X = 0
                IRIS_Y = 0
                self.robot.set_state(x=IRIS_X, y=IRIS_Y, z=0.5)
                self.board = Apriltag("apriltag")
                self.board.spawn()
                self.board.set_state(
                    x=1.5,
                    y=1.5,
                    z=0.015,
                )

                controller = BoardController(self.board, self.robot)

                with sitl_env():
                    while True:
                        user_input = input(
                            "Press [Space] then [Enter] to start/stop, or 'q' to quit > "
                        )
                        if user_input.strip().lower() == "q":
                            print("Exiting...")
                            controller.stop()
                            return
                        elif user_input == " ":
                            if controller.moving:
                                controller.stop()
                                print("Stopped moving board.")
                            else:
                                controller.start()
                                print("Started moving board.")
                        else:
                            print(
                                "Invalid input. Press [Space] then [Enter] to start/stop."
                            )

            except KeyboardInterrupt:
                controller.stop()
                break
            except EOFError:
                controller.stop()
                break


if __name__ == "__main__":
    # 将 unittest 挂载到 rostest 框架上
    rostest.rosrun("dankong", "debug", Debug)
