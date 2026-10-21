#!/usr/bin/env python
# coding=utf-8
"""Follow(跟随)专项用例: 模拟发送 location_vel 目标, 验证进入跟随、
跟随指令真实执行、obj_lla 目标点发布、停止跟随后的状态恢复与冷却。

运行方式(与其他 pytest 用例一致):
    ./pytest/run_test.sh pytest/test_follow.py -v
"""
import os
import re
import subprocess
import time

from steps import (
    WAYPOINT_STATES,
    EnsureGround,
    Land,
    PrearmCheck,
    Sleep,
    Step,
    Takeoff,
    http_post,
)

STATE_FOLLOW = "跟随模式"
STATE_HOVER = "悬停状态"

# 跨用例共享: 最近一次 /stop_follow 的时刻。dankong 的 100s 跟随冷却
# 记在节点进程里, pytest 多个用例共用同一个节点, 上一个用例的
# stop_follow 会吞掉下一个用例的目标, 发目标前必须把剩余冷却等完
last_stop_follow_time = [0.0]
# 与 C++ 侧同一个环境变量 (algo/event_listener.cpp), 测试时可用
# DK_FOLLOW_COOL_TIME=2 ./pytest/run_test.sh ... 秒级验证冷却逻辑
COOLDOWN_BASE = float(os.environ.get("DK_FOLLOW_COOL_TIME", 100.0))
FOLLOW_COOL_TIME = COOLDOWN_BASE + 5.0  # 裕量
# 注入是否生效一目了然; C++ 侧生效时 dankong.log 里也会有 override warn
print(
    f"[test_follow] DK_FOLLOW_COOL_TIME={os.environ.get('DK_FOLLOW_COOL_TIME', '(未设置, 默认100s)')}"
    f", pytest 侧冷却等待按 {FOLLOW_COOL_TIME:.0f}s 计算"
)


def wait_cooldown_clear(extra=2.0):
    """阻塞直到距离上次 stop_follow 超过冷却期。"""
    remain = last_stop_follow_time[0] + FOLLOW_COOL_TIME + extra - time.time()
    if remain > 0:
        print(f"等待跟随冷却结束 {remain:.0f}s (上次 stop_follow 遗留)")
        time.sleep(remain)


TOPIC_LOC = "/UAV0/perception/object_location/location_vel"
TOPIC_LLA = "/UAV0/perception/object_location/obj_lla"


def _pointobj_yaml(score, vx=0.0, vy=0.0, vz=0.0, dx=0.0, dy=0.0, dz=0.0):
    """构造 rsos_msgs/PointObj 的 rostopic pub YAML (header auto 填充)。"""
    return (
        "{header: auto, score: %s, "
        "pos: {x: %s, y: %s, z: %s}, "
        "velocity: {x: %s, y: %s, z: %s}}" % (score, dx, dy, dz, vx, vy, vz)
    )


class SendFollowTarget(Step):
    """后台启动 rostopic pub 以 10Hz 持续发布 PointObj 到 location_vel。

    类级注册表跟踪所有存活发布者: run() 自动停掉之前的实例 (避免双发布者
    交替发指令), stop_all() 供用例收尾统一清理, 杜绝孤儿发布进程跨用例
    污染 (它会把后续用例的飞机重新拽进跟随模式)。
    """

    name = "发送跟随目标"
    _live = []

    def __init__(self, score=1, vx=0.0, vy=0.0, vz=0.0, dx=0.0, dy=0.0, dz=0.0):
        self.args = dict(score=score, vx=vx, vy=vy, vz=vz, dx=dx, dy=dy, dz=dz)
        self.proc = None

    def run(self, robot):
        self.stop_all()
        self.proc = subprocess.Popen(
            [
                "rostopic",
                "pub",
                "-r",
                "10",
                TOPIC_LOC,
                "rsos_msgs/PointObj",
                _pointobj_yaml(**self.args),
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        type(self)._live.append(self)
        time.sleep(1)  # 等 rostopic 建立连接

    @classmethod
    def stop_all(cls):
        for s in list(cls._live):
            s.stop()

    def stop(self):
        if self.proc is not None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.proc.kill()
            self.proc = None
        if self in type(self)._live:
            type(self)._live.remove(self)

    def check(self, robot):
        pass


class WaitFollow(Step):
    """等待进入跟随模式。"""

    def __init__(self, timeout=30):
        self.name = "等待进入跟随"
        self.timeout = timeout

    def run(self, robot):
        robot.wait_for_state(STATE_FOLLOW, self.timeout)


class VerifyFollowClimb(Step):
    """跟随指令执行正确性: velocity.z=+1 下发后, 采样 climb 秒,
    高度上升量必须超过 expect 米 (速度指令真实被执行)。"""

    def __init__(self, climb=10, expect=1.5):
        self.name = f"校验跟随爬升({climb}s >{expect}m)"
        self.climb = climb
        self.expect = expect

    def run(self, robot):
        z0 = robot.get_pos()[2]
        time.sleep(self.climb)
        z1 = robot.get_pos()[2]
        rise = z1 - z0
        print(f"跟随爬升校验: z {z0:.2f} -> {z1:.2f} (rise={rise:.2f}m)")
        if rise < self.expect:
            raise RuntimeError(
                f"跟随速度指令未执行/不足: {self.climb}s 仅上升 {rise:.2f}m"
            )


class VerifyLostTargetHold(Step):
    """score<0 (目标丢失) 时 FollowState 应清零速度: 悬停不动。
    发送 score=-1 且 velocity 非零, hold 秒内漂移必须在容差内。"""

    def __init__(self, hold=8, xy_tol=1.0, z_tol=1.0):
        self.name = f"校验目标丢失保持({hold}s 漂移<{xy_tol}m)"
        self.hold = hold
        self.xy_tol = xy_tol
        self.z_tol = z_tol

    def run(self, robot):
        sender = SendFollowTarget(score=-1, vx=1.0, vy=1.0, vz=1.0)
        pos0 = robot.get_pos()
        sender.run(robot)
        try:
            time.sleep(self.hold)
        finally:
            sender.stop()
        pos1 = robot.get_pos()
        dxy = ((pos1[0] - pos0[0]) ** 2 + (pos1[1] - pos0[1]) ** 2) ** 0.5
        dz = abs(pos1[2] - pos0[2])
        print(f"丢失目标保持校验: dxy={dxy:.2f}m dz={dz:.2f}m")
        if dxy > self.xy_tol or dz > self.z_tol:
            raise RuntimeError(f"score<0 未清零速度: 漂移 dxy={dxy:.2f}m dz={dz:.2f}m")


class VerifyObjLla(Step):
    """跟随中应发布 obj_lla (PointStamped), 且经纬度非零;
    若 ws state 提供 lon/lat, 进一步断言目标点距飞机 <15m。"""

    def __init__(self, timeout=15, dist_tol=15.0):
        self.name = "校验 obj_lla 发布"
        self.timeout = timeout
        self.dist_tol = dist_tol

    def run(self, robot):
        try:
            out = subprocess.run(
                ["rostopic", "echo", "-n", "1", TOPIC_LLA],
                capture_output=True,
                text=True,
                timeout=self.timeout,
            ).stdout
        except subprocess.TimeoutExpired:
            raise RuntimeError(f"{self.timeout}s 内未收到 {TOPIC_LLA} 消息")
        m = re.search(
            r"point:\s*x:\s*([-0-9.eE]+)\s*y:\s*([-0-9.eE]+)\s*z:\s*([-0-9.eE]+)",
            out,
        )
        if not m:
            raise RuntimeError(f"obj_lla 格式异常: {out[:200]!r}")
        lon, lat, alt = (float(g) for g in m.groups())
        print(f"obj_lla: lon={lon:.7f} lat={lat:.7f} alt={alt:.2f}")
        if abs(lon) < 1e-6 or abs(lat) < 1e-6:
            raise RuntimeError("obj_lla 经纬度为零")

        state = robot.snapshot()
        if state.get("lon") and state.get("lat"):
            d = self._gps_dist(lon, lat, state["lon"], state["lat"])
            print(f"obj_lla 距飞机: {d:.1f}m")
            if d > self.dist_tol:
                raise RuntimeError(f"obj_lla 目标点距飞机 {d:.1f}m > {self.dist_tol}m")

    @staticmethod
    def _gps_dist(lon1, lat1, lon2, lat2):
        from math import asin, cos, radians, sin, sqrt

        dlat = radians(lat2 - lat1)
        dlon = radians(lon2 - lon1)
        a = (
            sin(dlat / 2) ** 2
            + cos(radians(lat1)) * cos(radians(lat2)) * sin(dlon / 2) ** 2
        )
        return 2 * 6371000 * asin(sqrt(a))


class StopFollow(Step):
    """POST /stop_follow, 断言退出跟随, 并校验恢复到期望状态。"""

    def __init__(self, recover_states=(STATE_HOVER,), timeout=60):
        self.name = f"停止跟随(恢复到{list(recover_states)})"
        self.recover = tuple(recover_states)
        self.timeout = timeout

    def run(self, robot):
        res = http_post("/stop_follow")
        if res.get("status") != "success":
            raise RuntimeError(f"stop_follow 被拒: {res}")
        robot.wait_for_state_any(self.recover, self.timeout)
        last_stop_follow_time[0] = time.time()
        cur = robot.get_state_name()
        if cur == STATE_FOLLOW:
            raise RuntimeError("停止跟随后仍在跟随模式")


class AssertNotFollow(Step):
    """冷却期内持续发送目标, 断言不进入跟随模式。

    暴露窗口以创建发布者之前为起点计时 (sender.run() 内部含 1s 连接
    sleep, 不能计入窗口外), 且总时长不超过冷却期的 40%, 保证整个窗口
    都落在保护期内。
    """

    def __init__(self, seconds=None):
        if seconds is None:
            seconds = min(max(0.6, COOLDOWN_BASE * 0.4), 10.0)
        self.name = f"冷却期不响应({seconds:.1f}s)"
        self.seconds = seconds

    def run(self, robot):
        t0 = time.time()
        sender = SendFollowTarget(score=1, vz=0.5)
        sender.run(robot)
        try:
            while time.time() - t0 < self.seconds:
                if robot.get_state_name() == STATE_FOLLOW:
                    raise RuntimeError("冷却期内进入了跟随模式")
                time.sleep(0.1)
        finally:
            sender.stop()


class StartWaypoint(Step):
    """下发远处航点并进入航点模式 (不等飞完, 供跟随打断用)。"""

    def __init__(self, waypoints, timeout=60):
        self.name = f"下发航点({len(waypoints)}个, 不等完成)"
        self.waypoints = [list(w) for w in waypoints]
        self.timeout = timeout

    def run(self, robot):
        res = http_post("/set_waypoint", data=dict(waypoint=self.waypoints, local=True))
        if res.get("status") != "success":
            raise RuntimeError(f"set_waypoint 被拒: {res}")
        robot.wait_for_state_any(WAYPOINT_STATES, self.timeout)


class WaitPosNear(Step):
    """等待飞机到达指定 ENU 位置附近 (用于确认已飞过前几个航点)。"""

    def __init__(self, target, xy_tol=3.0, z_tol=1.5, timeout=300):
        self.name = f"等待到达{[round(v, 1) for v in target]}"
        self.target = list(target)
        self.xy_tol = xy_tol
        self.z_tol = z_tol
        self.timeout = timeout

    def run(self, robot):
        deadline = time.time() + self.timeout
        pos = None
        while time.time() < deadline:
            pos = robot.get_pos()
            d = ((pos[0] - self.target[0]) ** 2 + (pos[1] - self.target[1]) ** 2) ** 0.5
            if d < self.xy_tol and abs(pos[2] - self.target[2]) < self.z_tol:
                print(f"已到达附近: dxy={d:.1f}m")
                return
            time.sleep(0.5)
        raise RuntimeError(f"等待到达 {self.target} 超时, 当前: {pos}")


def test_follow_hover(check, robot):
    """悬停 -> 跟随(校验速度执行+obj_lla) -> 停止跟随恢复悬停 -> 冷却不响应。"""
    sender = None
    try:
        # 防其他用例遗留的 stop_follow 冷却
        wait_cooldown_clear()
        check(
            [
                EnsureGround(),
                PrearmCheck(),
                Takeoff(alt=8),
            ]
        )

        # 进入跟随
        sender = SendFollowTarget(score=1, vz=1.0)
        check([sender, WaitFollow()])

        # 跟随指令执行正确性: 速度真实执行
        check([VerifyFollowClimb(climb=10, expect=1.5)])

        # 丢失保持校验前必须停掉 vz=1 的旧发布者, 否则两个 pub 交替
        # 发送 score=1/score=-1, FollowState 仍会收到爬升指令
        sender.stop()
        time.sleep(1)
        check([VerifyLostTargetHold(hold=8)])

        # obj_lla 校验需要有效的跟随目标 (零速悬停跟随)
        sender = SendFollowTarget(score=1, vz=0.0)
        check([sender])
        time.sleep(1)
        check([VerifyObjLla()])

        # 停止跟随 -> 恢复悬停; 冷却期内不响应 (时长自动随冷却值缩放)
        check([StopFollow(recover_states=(STATE_HOVER,))])
        check([AssertNotFollow()])

        # 冷却期过后应恢复响应: 等满保护期再发目标, 应重新进入跟随
        wait_cooldown_clear()
        sender = SendFollowTarget(
            score=1, vz=0.5
        )  # 必须交给 sender, 否则 finally 停不掉
        check([sender, WaitFollow(timeout=30)])

        # 收尾: 再次退出跟随, 避免跟随中降落被拒
        check([StopFollow(recover_states=(STATE_HOVER,))])
    finally:
        SendFollowTarget.stop_all()  # 统一清理, 防孤儿发布进程泄漏
        # 兜底: 用例中途失败时飞机可能仍在跟随, 先退出再降落,
        # 避免把跟随状态泄漏给后续用例
        if robot.get_state_name() == STATE_FOLLOW:
            try:
                http_post("/stop_follow")
                robot.wait_for_state(STATE_HOVER, 30)
            except Exception as e:  # noqa: BLE001
                print(f"兜底退出跟随失败: {e}")
    # 清理: 降落
    check([Sleep(2), Land()])


def test_follow_waypoint(check, robot):
    """航点 -> 跟随 -> 停止跟随恢复航点执行。"""
    sender = None
    try:
        # 必须在进入航点前把 hover 用例遗留的冷却等完: 若在航点中等待,
        # 等待期间航点会飞完回退悬停, "打断航线"的场景就不成立了
        wait_cooldown_clear()
        check(
            [
                EnsureGround(),
                PrearmCheck(),
                Takeoff(alt=8),
            ]
        )
        check(
            [
                StartWaypoint(
                    # 4 个航点: 先正常飞完前两个, 在飞往第 3 个途中打断
                    waypoints=[[0, 0, 8], [20, 0, 8], [60, 0, 8], [120, 0, 8]],
                    timeout=60,
                ),
                # 前两个航点已飞过 (当前在 [20,0,8] 附近)
                WaitPosNear(target=[20, 0, 8]),
            ]
        )

        sender = SendFollowTarget(score=1, vz=0.5)
        check([sender, WaitFollow()])

        check([StopFollow(recover_states=WAYPOINT_STATES)])

        # 恢复航线: 停掉目标发布后, 飞机应继续朝第 3 个航点 [60,0,8] 飞
        sender.stop()
        time.sleep(1)
        check([WaitPosNear(target=[60, 0, 8], timeout=180)])
    finally:
        SendFollowTarget.stop_all()  # 统一清理, 防孤儿发布进程泄漏
        # 兜底: 用例中途失败时飞机可能仍在跟随, 先退出避免状态泄漏
        # (回航点或悬停均可, 后续 EnsureGround/Land 能处理)
        if robot.get_state_name() == STATE_FOLLOW:
            try:
                http_post("/stop_follow")
                robot.wait_for_state_any(WAYPOINT_STATES + (STATE_HOVER,), 30)
            except Exception as e:  # noqa: BLE001
                print(f"兜底退出跟随失败: {e}")
    # 清理: 降落
    check([Sleep(2), Land()])
