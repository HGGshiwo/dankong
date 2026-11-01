#!/usr/bin/env python
# coding=utf-8
"""GPS 航线用例: 起飞后以 GPS (lon/lat/alt) 下发 3 个航点,
逐点校验真实航点的到达与精度 (区别于 test_waypoint.py 的 ENU 米制模式)。

首位为起飞占位 [0, 0, 起飞高度] (仅 z 生效, 空中整点抛弃),
逐点校验只针对真实航点。
"""
from steps import (
    EnsureGround,
    FlyWaypoint,
    Land,
    PrearmCheck,
    Sleep,
    Takeoff,
    gps_offset,
)

# 偏移量 (度): ~0.0003° ≈ 33m, 三个点构成一个小三角形
DLAT = 0.0003
DLON = 0.0003


def test_waypoint_gps(check, robot):
    robot.wait_state_fresh()
    state = robot.snapshot()
    lat, lon = state.get("lat"), state.get("lon")
    if not lat or not lon:
        raise RuntimeError(f"WS state 中无 GPS: keys={sorted(state.keys())}")

    def _fly():
        # dankong 坐标约定: [经度lon, 纬度lat, 高度alt]
        wps = [
            [0, 0, 10.0],
            [lon + DLON, lat, 10.0],
            [lon + DLON, lat + DLAT, 12.0],
            [lon, lat + DLAT, 10.0],
        ]
        print(f"GPS 航点(lon,lat,alt): {wps}")
        return FlyWaypoint(wps, speed=3, expected=wps[1:])

    check(
        [
            EnsureGround(),
            PrearmCheck(),
            Takeoff(alt=10),
        ]
    )
    flyer = _fly()
    check([flyer])
    # 清理: 降落
    check([Sleep(2), Land()])
