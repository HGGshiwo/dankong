#!/usr/bin/env python
# coding=utf-8
"""航点用例: 起飞后飞 3 个航点, 校验逐点到达与终点精度。

航点统一用 GPS [lon, lat, alt] (dankong 约定顺序), 首位为起飞占位
[0, 0, 起飞高度] (仅 z 生效, 空中整点抛弃), 真实航点从第二个开始;
逐点校验只针对真实航点。
"""
from steps import EnsureGround, FlyWaypoint, PrearmCheck, Step, Takeoff, mk_gps_points

# 相对锚点 (起飞悬停点) 的 ENU 偏移 [东, 北, 高度]
WAYPOINT_OFFSETS = [
    [10.0, 0.0, 10.0],
    [10.0, 10.0, 10.0],
    [0.0, 10.0, 12.0],
]


class FlyWaypointGpsFromAnchor(Step):
    """以当前位置为锚把 ENU 偏移转成 GPS 航点并下发, 逐点校验真实航点。"""

    name = "飞GPS航点(锚定当前位置)"

    def run(self, robot):
        pos = robot.get_pos()
        ax, ay = pos[0], pos[1]
        specs = [(x - ax, y - ay, z) for x, y, z in WAYPOINT_OFFSETS]
        wps = [[0, 0, 10.0]] + mk_gps_points(robot, specs)
        flyer = FlyWaypoint(wps, speed=3, expected=wps[1:])
        flyer.run(robot)
        flyer.check(robot)


def test_waypoint(check):
    check(
        [
            EnsureGround(),
            PrearmCheck(),
            Takeoff(alt=10),
            FlyWaypointGpsFromAnchor(),
        ]
    )
