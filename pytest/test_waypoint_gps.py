#!/usr/bin/env python
# coding=utf-8
"""GPS 航线用例: 起飞后以 GPS (lat/lon/alt) 下发 3 个航点,
校验逐点到达与终点精度 (区别于 test_waypoint.py 的 ENU 米制模式)。"""
from steps import EnsureGround, FlyWaypoint, Land, PrearmCheck, Sleep, Step, Takeoff

# 偏移量 (度): ~0.0003° ≈ 33m, 三个点构成一个小三角形
DLAT = 0.0003
DLON = 0.0003


class CaptureGpsWaypoints(Step):
    """起飞悬停后读取当前 GPS, 生成围绕当前位置的 3 个 GPS 航点。"""

    name = "生成GPS航点"

    def run(self, robot):
        state = robot.snapshot()
        lat, lon = state.get("lat"), state.get("lon")
        if not lat or not lon:
            raise RuntimeError(f"WS state 中无 GPS: keys={sorted(state.keys())}")
        # dankong 坐标约定 (gps_to_enu/enu_to_gps): [经度lon, 纬度lat, 高度alt]
        robot.gps_waypoints = [
            [lon + DLON, lat, 10.0],
            [lon + DLON, lat + DLAT, 12.0],
            [lon, lat + DLAT, 10.0],
        ]
        print(f"GPS 航点(lon,lat,alt): {robot.gps_waypoints}")


def test_waypoint_gps(check, robot):
    check(
        [
            EnsureGround(),
            PrearmCheck(),
            Takeoff(alt=10),
            CaptureGpsWaypoints(),
        ]
    )
    check([FlyWaypoint(robot.gps_waypoints, speed=3, local=False)])
    # 清理: 降落
    check([Sleep(2), Land()])
