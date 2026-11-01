#!/usr/bin/env python
# coding=utf-8
"""航点数量 x 地面/空中 x 航线/返航 —— 12 格矩阵测试。

测试只发表格中的原始输入 (0/1/2 个 GPS 点), 不做任何补全/加工;
dankong 侧语义 (单点自动补起飞占位、多点首点仅取高度、返航追加返航点)
由后端实现, expected 为该格实际会飞的航点。

|                  | 地面=0 | 地面=1           | 地面>1                | 空中=0 | 空中=1 | 空中>1         |
| 航线(set_waypoint) | 报错   | 飞到该点高度后飞到该点 | 正常飞(第一点高度作起飞高度) | 悬停   | 飞向该点 | 正常飞(忽略第一个点) |
| 返航(return)       | 报错   | 飞到该点高度后飞到该点+返航 | 同上+返航          | 返航   | 飞向该点，返航 | 飞航点(忽略第一个点)+返航 |
"""
import time

from steps import (
    STATE_GROUND,
    STATE_HOVER,
    CheckHoverAtAlt,
    EnsureGround,
    FlyWaypoint,
    PrearmCheck,
    Step,
    Takeoff,
    gps_distance,
    http_get,
    http_post,
    mk_gps_points,
)

XY_TOL = 3.0
LAND_TOL = 4.0  # 返航落点容差
MOVE = 20.0  # 相邻航点间距 (m)


class AnchorTakeoffPos(Step):
    """记录当前 GPS 为返航校验锚点 (dankong 起飞点)。"""

    name = "记录返航锚点"

    def run(self, robot):
        robot.wait_state_fresh()
        s = robot.snapshot()
        robot.return_target_gps = (s["lon"], s["lat"])


class LandNearAnchor(Step):
    """断言已回到地面, 且落点接近 robot.return_target_gps。"""

    name = "校验返航落点"

    def __init__(self, xy_tol=LAND_TOL, timeout=600):
        self.xy_tol = xy_tol
        self.timeout = timeout

    def run(self, robot):
        robot.wait_for_state(STATE_GROUND, self.timeout)

    def check(self, robot):
        s = robot.snapshot()
        d = gps_distance(s["lon"], s["lat"], *robot.return_target_gps)
        if d > self.xy_tol:
            raise RuntimeError(f"落点偏离返航锚点 {d:.2f}m (> {self.xy_tol}m)")
        print(f"落点偏差 {d:.2f}m")


class ExpectWpRejected(Step):
    """发送航点任务, 断言被拒绝。"""

    name = "航点任务应被拒绝"

    def __init__(self, points, finish_return=False):
        self.points = points
        self.finish_return = finish_return

    def run(self, robot):
        path = "/return" if self.finish_return else "/set_waypoint"
        res = http_post(path, data=dict(waypoint=self.points, local=False))
        if res.get("status") == "success":
            raise RuntimeError(f"{path}({len(self.points)}点) 未被拒绝: {res}")
        print(f"{path}({len(self.points)}点) 按预期被拒: {res}")


class ExpectStillHover(Step):
    """空中 0 航点任务应原地悬停: 校验状态保持悬停且位置漂移小。"""

    name = "应保持悬停"

    def __init__(self, seconds=5, drift_tol=2.0):
        self.seconds = seconds
        self.drift_tol = drift_tol

    def run(self, robot):
        robot.wait_state_fresh()
        time.sleep(1)

    def check(self, robot):
        deadline = time.time() + self.seconds
        p0 = robot.get_pos()
        while time.time() < deadline:
            if robot.get_state_name() != STATE_HOVER:
                raise RuntimeError(f"悬停被破坏, 当前状态: {robot.get_state_name()}")
            time.sleep(0.5)
        p1 = robot.get_pos()
        drift = (
            (p1[0] - p0[0]) ** 2 + (p1[1] - p0[1]) ** 2 + (p1[2] - p0[2]) ** 2
        ) ** 0.5
        if drift > self.drift_tol:
            raise RuntimeError(f"悬停位置漂移 {drift:.2f}m (> {self.drift_tol}m)")


# ---------------- 航线 (set_waypoint) ----------------


def test_sw_ground_0_rejected(check):
    """地面 0 点航线: 直接报错。"""
    check([EnsureGround(), ExpectWpRejected(points=[])])


def test_sw_ground_1_fly_to_point(check):
    """地面 1 点航线: 飞到该点高度后飞向该点, 悬停。"""
    check([EnsureGround(), _fly_ground([(MOVE, 0, 10)]), CheckHoverAtAlt(alt=10)])


def test_sw_ground_2_first_alt_is_takeoff(check):
    """地面 2 点航线: 先起飞 (悬停后观察起飞高度), 再发 2 点航线
    (空中忽略第一个点, 飞第二个点)。"""
    check(
        [
            EnsureGround(),
            PrearmCheck(),
            Takeoff(alt=12),
            CheckHoverAtAlt(alt=12),
            _fly_air([(MOVE, 0, 12), (2 * MOVE, 0, 14)], speed=2),
        ]
    )


def test_sw_air_0_hover(check):
    """空中 0 点航线: success 且原地悬停 (不拒绝、不崩溃)。"""

    class _Send0(Step):
        name = "空中发0点航线"

        def run(self, robot):
            res = http_post("/set_waypoint", data=dict(waypoint=[], local=False))
            if res.get("status") != "success":
                raise RuntimeError(f"空中0点航线被拒: {res}")
            print(f"空中0点航线响应: {res}")

    check(
        [
            EnsureGround(),
            PrearmCheck(),
            Takeoff(alt=10),
            _Send0(),
            ExpectStillHover(),
        ]
    )


def test_sw_air_1_flies_to_point(check):
    """空中 1 点航线: 飞向该点。"""
    check([EnsureGround(), PrearmCheck(), Takeoff(alt=10), _fly_air([(MOVE, 0, 12)])])


def test_sw_air_2_ordered(check):
    """空中 2 点航线: 忽略第一个点, 飞第二个点。"""
    check(
        [
            EnsureGround(),
            PrearmCheck(),
            Takeoff(alt=10),
            _fly_air([(MOVE, 0, 12), (2 * MOVE, 0, 14)], speed=2),
        ]
    )


# ---------------- 返航 (return) ----------------


def test_ret_ground_0_rejected(check):
    """地面 0 点返航: 直接报错。"""
    check([EnsureGround(), ExpectWpRejected(points=[], finish_return=True)])


def test_ret_ground_1_fly_then_land(check):
    """地面 1 点返航: 飞到该点高度后飞向该点, 返航降落在锚点。"""
    check(
        [
            EnsureGround(),
            AnchorTakeoffPos(),
            _fly_ground_return([(MOVE, 0, 10)]),
            LandNearAnchor(),
        ]
    )


def test_ret_ground_2_fly_then_land(check):
    """地面 2 点返航: 第一点高度作起飞高度, 飞第二点后返航降落。"""
    check(
        [
            EnsureGround(),
            AnchorTakeoffPos(),
            _fly_ground_return([(MOVE, 0, 10), (2 * MOVE, 0, 12)]),
            LandNearAnchor(),
        ]
    )


def test_ret_air_0_return(check):
    """空中 0 点返航: 直接返航降落 (dankong 起飞点)。"""
    check(
        [
            EnsureGround(),
            PrearmCheck(),
            AnchorTakeoffPos(),
            Takeoff(alt=10),
            _return_empty(),
            LandNearAnchor(),
        ]
    )


def test_ret_air_1_fly_then_return(check):
    """空中 1 点返航: 飞向该点后返航降落。"""
    check(
        [
            EnsureGround(),
            PrearmCheck(),
            AnchorTakeoffPos(),
            Takeoff(alt=10),
            _fly_air_return([(MOVE, 0, 12)]),
            LandNearAnchor(),
        ]
    )


def test_ret_air_2_fly_then_return(check):
    """空中 2 点返航: 忽略第一个点, 飞第二点后返航降落。"""
    check(
        [
            EnsureGround(),
            PrearmCheck(),
            AnchorTakeoffPos(),
            Takeoff(alt=10),
            _fly_air_return([(MOVE, 0, 12), (2 * MOVE, 0, 14)], speed=2),
            LandNearAnchor(),
        ]
    )


# ---------------- 构造辅助 (纯坐标锚定, 不做语义加工) ----------------


def _fly_ground(specs):
    """地面航线: 原样发送 specs, 校验第二个及之后的点。"""

    class _FlyGround(Step):
        name = f"地面航线({len(specs)}点)"

        def run(self, robot):
            robot.wait_state_fresh()
            pts = mk_gps_points(robot, specs)
            flyer = FlyWaypoint(pts, speed=2, expected=pts[1:], xy_tol=XY_TOL)
            flyer.run(robot)
            flyer.check(robot)

    return _FlyGround()


def _fly_ground_return(specs):
    """地面返航: 原样发送 specs + /return 收尾, 校验第二个及之后的点 +
    返航落点 (锚点, 地面高度)。"""

    class _Fly(Step):
        name = f"地面返航({len(specs)}点)"

        def run(self, robot):
            robot.wait_state_fresh()
            pts = mk_gps_points(robot, specs)
            anchor = [robot.return_target_gps[0], robot.return_target_gps[1], 0]
            flyer = FlyWaypoint(
                pts, finish_return=True, expected=pts[1:] + [anchor], xy_tol=XY_TOL
            )
            flyer.run(robot)
            flyer.check(robot)

    return _Fly()


def _fly_air(specs, speed=None):
    """空中航线: 原样发送 specs, 校验第二个及之后的点。"""

    class _Fly(Step):
        name = f"空中航线({len(specs)}点)"

        def run(self, robot):
            robot.wait_state_fresh()
            pts = mk_gps_points(robot, specs)
            flyer = FlyWaypoint(pts, speed=speed, expected=pts[1:], xy_tol=XY_TOL)
            flyer.run(robot)
            flyer.check(robot)

    return _Fly()


def _fly_air_return(specs, speed=None):
    """空中返航: 原样发送 specs + /return 收尾, 校验第二个及之后的点 +
    返航落点 (锚点, 地面高度)。"""

    class _Fly(Step):
        name = f"空中返航({len(specs)}点)"

        def run(self, robot):
            robot.wait_state_fresh()
            pts = mk_gps_points(robot, specs)
            anchor = [robot.return_target_gps[0], robot.return_target_gps[1], 0]
            flyer = FlyWaypoint(
                pts,
                speed=speed,
                finish_return=True,
                expected=pts[1:] + [anchor],
                xy_tol=XY_TOL,
            )
            flyer.run(robot)
            flyer.check(robot)

    return _Fly()


def _return_empty():
    class _Ret(Step):
        name = "空中返航(无点)"

        def run(self, robot):
            res = http_post("/return", data=dict(waypoint=[], local=False))
            if res.get("status") != "success":
                raise RuntimeError(f"return 被拒: {res}")

    return _Ret()
