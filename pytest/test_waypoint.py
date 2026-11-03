#!/usr/bin/env python
# coding=utf-8
"""航点用例: 起飞后飞 3 个航点, 校验逐点到达与终点精度。

航点统一用 GPS [lon, lat, alt] (dankong 约定顺序), 首位为起飞占位
[0, 0, 起飞高度] (仅 z 生效, 空中整点抛弃), 真实航点从第二个开始;
逐点校验只针对真实航点。
"""
from steps import (
    EnsureGround,
    FlyWaypoint,
    Land,
    PrearmCheck,
    Step,
    Takeoff,
    http_post,
    mk_gps_points,
)

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


# ============================ 检测限速用例 ============================

# 开启检测期间航点巡航速度必须压到 1 m/s (限速), 校验阈值放宽到 1.1 m/s
DETECT_SPEED_LIMIT = 1.1


class StartDetect(Step):
    """POST /start_detect 开启检测 (SITL 下仅设置 ROS 参数, 无感知数据)。

    type 必须与 config 的 detect_map 键一致: nohardhat / smoke。
    """

    name = "开启检测"

    def run(self, robot):
        res = http_post("/start_detect", {"type": "nohardhat"})
        if res.get("status") != "success":
            raise RuntimeError(f"/start_detect 被拒: {res}")


class StopDetect(Step):
    """POST /stop_detect 关闭检测, 巡航速度应恢复。"""

    name = "关闭检测"

    def run(self, robot):
        res = http_post("/stop_detect", {})
        if res.get("status") != "success":
            raise RuntimeError(f"/stop_detect 被拒: {res}")


class FlyWaypointGpsFromAnchorOffset(Step):
    """以当前位置为锚把 ENU 偏移转成 GPS 航点并下发, 支持速度监控参数。"""

    def __init__(self, offsets, speed=3, max_speed_limit=None, min_speed=None):
        self.name = "飞GPS航点(锚定当前位置)"
        self.offsets = offsets
        self.speed = speed
        self.max_speed_limit = max_speed_limit
        self.min_speed = min_speed  # 断言观测到的最大速度至少达到该值
        self._flyer = None

    def run(self, robot):
        pos = robot.get_pos()
        ax, ay = pos[0], pos[1]
        specs = [(x - ax, y - ay, z) for x, y, z in self.offsets]
        wps = [[0, 0, 10.0]] + mk_gps_points(robot, specs)
        self._flyer = FlyWaypoint(
            wps,
            speed=self.speed,
            expected=wps[1:],
            max_speed_limit=self.max_speed_limit,
        )
        self._flyer.run(robot)
        self._flyer.check(robot)
        if self.min_speed is not None:
            observed = self._flyer.max_observed_speed
            if observed < self.min_speed:
                raise RuntimeError(
                    f"巡航速度未恢复: 观测最大水平速度 {observed:.2f} m/s "
                    f"< 期望下限 {self.min_speed} m/s"
                )
            print(
                f"巡航速度恢复校验通过: 最大 {observed:.2f} m/s >= {self.min_speed} m/s"
            )


def test_waypoint_detect_speed(check):
    """开启检测时巡航速度限到 1 m/s (<=1.1 校验), 关闭后恢复 task speed=3。"""
    check(
        [
            EnsureGround(),
            PrearmCheck(),
            Takeoff(alt=10),
            StartDetect(),
            FlyWaypointGpsFromAnchorOffset(
                WAYPOINT_OFFSETS, speed=3, max_speed_limit=DETECT_SPEED_LIMIT
            ),
            StopDetect(),
            FlyWaypointGpsFromAnchorOffset(WAYPOINT_OFFSETS, speed=3, min_speed=1.5),
            Land(),
        ]
    )
