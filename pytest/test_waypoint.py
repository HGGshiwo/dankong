#!/usr/bin/env python
# coding=utf-8
"""航点用例: 起飞后飞 3 个 ENU 航点, 校验逐点到达与终点精度。"""
from steps import EnsureGround, FlyWaypoint, PrearmCheck, Takeoff

WAYPOINTS = [
    [10.0, 0.0, 10.0],
    [10.0, 10.0, 10.0],
    [0.0, 10.0, 12.0],
]


def test_waypoint(check):
    check(
        [
            EnsureGround(),
            PrearmCheck(),
            Takeoff(alt=10),
            FlyWaypoint(WAYPOINTS, speed=3),
        ]
    )
