#!/usr/bin/env python
# coding=utf-8
"""
dankong 任务流测试框架与步骤库 (满足开闭原则)

设计:
  - Step 基类 + MissionRunner 是"封闭"部分: 只负责调度/判定/报告, 新增功能不改这里
  - 具体步骤 (Takeoff/Land/...) 是"开放"部分: 继承 Step 即可扩展
  - 测试场景是"数据": Step 有序列表, 排列组合即可生成新用例

仅依赖 requests / websockets, 不依赖 rospy / gazebo。
状态与位置来自 dankong WebSocket (ws://127.0.0.1:8000/ws) 的 state 帧。
"""
import asyncio
import json
import threading
import time
from dataclasses import dataclass, field

import requests
import websockets

HOST = "127.0.0.1"
PORT = 8000

# 状态名 (与各 State 的 static_name() 对应)
STATE_GROUND = "地面状态"
STATE_TAKEOFF = "起飞状态"
STATE_HOVER = "悬停状态"
STATE_WAYPOINT = "航点模式"
STATE_WP_LIFT = "调整高度"  # WaypointState::LiftingState 叶子名
STATE_WP_EXEC = "执行航点"  # WaypointState::ExcuteState 叶子名
STATE_LAND = "降落状态"
# 航点任务的任一子状态 (WS state 帧上报的是叶子状态名)
WAYPOINT_STATES = (STATE_WAYPOINT, STATE_WP_LIFT, STATE_WP_EXEC)


def http_post(path, data=None):
    r = requests.post(f"http://{HOST}:{PORT}{path}", json=data, timeout=10)
    return r.json()


def http_get(path):
    r = requests.get(f"http://{HOST}:{PORT}{path}", timeout=10)
    return r.json()


def wait_prearms_ready(timeout=180):
    """等待 dankong web 服务就绪 (GET /prearms 返回 status==success)。"""
    deadline = time.time() + timeout
    last_err = None
    while time.time() < deadline:
        try:
            if http_get("/prearms").get("status") == "success":
                return
        except Exception as e:  # noqa: BLE001
            last_err = e
        time.sleep(2)
    raise RuntimeError(f"dankong web 服务未就绪: {last_err}")


class RobotClient:
    """轻量 WS 客户端: 后台线程维护 state 帧快照 (state/pos_enu/arm 等)。"""

    def __init__(self):
        self.state = {}
        self.lock = threading.Lock()
        self.connected = threading.Event()
        self._stop = threading.Event()
        self.state_ts = 0.0  # 最近一次收到 state 帧的时刻 (判断快照新鲜度)
        self.takeoff_pos = (0.0, 0.0)  # 由 RecordTakeoffPos 步骤刷新
        self._thread = threading.Thread(
            target=asyncio.run, args=(self._loop(),), daemon=True
        )
        self._thread.start()

    async def _loop(self):
        uri = f"ws://{HOST}:{PORT}/ws"
        while not self._stop.is_set():
            try:
                async with websockets.connect(uri, max_size=None) as ws:
                    self.connected.set()
                    async for raw in ws:
                        data = json.loads(raw)
                        if data.get("type") == "state":
                            with self.lock:
                                self.state.update(data)
                            self.state_ts = time.time()
            except Exception:  # noqa: BLE001
                self.connected.clear()
            time.sleep(1)

    def wait_state_fresh(self, max_age=10, timeout=30):
        """等待快照来自当前存活节点 (节点重启后/未连上时的旧快照不可信)。"""
        deadline = time.time() + timeout
        while time.time() < deadline:
            if time.time() - self.state_ts < max_age:
                return
            time.sleep(0.2)
        with self.lock:
            snapshot = dict(self.state)
        raise TimeoutError(
            f"等待新鲜状态帧超时 (最近一帧 {time.time() - self.state_ts:.0f}s 前, "
            f"快照: {snapshot})"
        )

    def snapshot(self):
        with self.lock:
            return dict(self.state)

    def get_state_name(self):
        return self.snapshot().get("state")

    def get_pos(self):
        """返回 [x, y, z] (ENU 米制), 无数据时返回 None。"""
        return self.snapshot().get("pos_enu")

    def wait_for_state(self, name, timeout=120):
        deadline = time.time() + timeout
        last = None
        while time.time() < deadline:
            last = self.get_state_name()
            if last == name:
                return
            time.sleep(0.2)
        raise TimeoutError(f"等待状态[{name}]超时, 当前: {last}")

    def wait_for_state_any(self, names, timeout=120):
        deadline = time.time() + timeout
        last = None
        while time.time() < deadline:
            last = self.get_state_name()
            if last in names:
                return
            time.sleep(0.2)
        raise TimeoutError(f"等待状态{list(names)}超时, 当前: {last}")

    def wait_pos_ready(self, timeout=60):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.get_pos() is not None:
                return
            time.sleep(0.2)
        raise TimeoutError("等待 pos_enu 数据超时")

    def close(self):
        self._stop.set()


@dataclass
class StepResult:
    name: str
    ok: bool
    detail: str = ""
    elapsed: float = 0.0


@dataclass
class MissionReport:
    results: list = field(default_factory=list)

    def to_markdown(self):
        lines = [
            "# dankong SITL 测试报告",
            "",
            "| 用例步骤 | 结果 | 耗时(s) | 详情 |",
            "|---|---|---|---|",
        ]
        for r in self.results:
            flag = "PASS" if r.ok else "FAIL"
            lines.append(f"| {r.name} | {flag} | {r.elapsed:.1f} | {r.detail} |")
        n_ok = sum(1 for r in self.results if r.ok)
        lines.append("")
        lines.append(f"**总计: {n_ok}/{len(self.results)} 通过**")
        return "\n".join(lines)


class Step:
    """测试步骤基类。子类实现 run(), 需要后置校验时覆盖 check()。"""

    name = "base-step"

    def run(self, robot):
        raise NotImplementedError

    def check(self, robot):
        return


class MissionRunner:
    """按顺序执行 Step 列表并汇总报告。对扩展封闭: 只认 Step 接口。"""

    def __init__(self, robot, report=None):
        self.robot = robot
        self.report = report if report is not None else MissionReport()

    def run(self, steps):
        failed = []
        for step in steps:
            print(f"===== 步骤开始: {step.name} =====")
            t0 = time.time()
            try:
                step.run(self.robot)
                step.check(self.robot)
                self.report.results.append(
                    StepResult(step.name, True, elapsed=time.time() - t0)
                )
            except Exception as e:  # noqa: BLE001
                self.report.results.append(
                    StepResult(step.name, False, str(e), time.time() - t0)
                )
                failed.append(f"{step.name}: {e}")
        return failed


# ---------------- 基础等待步骤 ----------------


class WaitState(Step):
    def __init__(self, state, timeout=120):
        self.name = f"等待状态[{state}]"
        self.state = state
        self.timeout = timeout

    def run(self, robot):
        robot.wait_for_state(self.state, self.timeout)


class Sleep(Step):
    def __init__(self, seconds):
        self.name = f"等待{seconds}s"
        self.seconds = seconds

    def run(self, robot):
        time.sleep(self.seconds)


# ---------------- 指令步骤 ----------------


class EnsureGround(Step):
    """用例前置: 确保回到地面状态, 空中则先降落。

    落地/上锁后 APM 地面检测需要几秒稳定, 否则紧随其后的 NAV_TAKEOFF
    会被 ACK 但不执行 (电机不转, DISARM_DELAY 后自动上锁), 故固定等待。
    """

    name = "确保地面状态"

    def __init__(self, settle=5):
        self.settle = settle

    def run(self, robot):
        robot.wait_state_fresh()
        robot.wait_pos_ready(60)
        if robot.get_state_name() != STATE_GROUND:
            with robot.lock:
                state_name = robot.state.get("state")
                age = time.time() - robot.state_ts
            print(
                f"当前不在地面 (state={state_name!r}, 快照 {age:.1f}s 前, "
                f"keys={sorted(robot.state.keys())}), 先降落"
            )
            res = http_post("/land", data=dict(do_pland=False))
            if res.get("status") != "success":
                raise RuntimeError(f"land 被拒: {res}")
            robot.wait_for_state(STATE_GROUND, 240)
        time.sleep(self.settle)


class PrearmCheck(Step):
    def __init__(self, retry=30, interval=3):
        self.name = "Prearm检查"
        self.retry = retry
        self.interval = interval

    def run(self, robot):
        for _ in range(self.retry):
            time.sleep(self.interval)
            res = http_get("/prearms")
            msg = res.get("msg")
            if isinstance(msg, dict) and msg.get("arm") is True:
                return
            print(f"prearm 未通过: {res}")
        raise RuntimeError("prearm 检查超时")


def _wait_prearm_ok(timeout=120):
    """起飞前置: 轮询 GET /prearms 直到通过 (msg.arm==true)。"""
    deadline = time.time() + timeout
    while time.time() < deadline:
        res = http_get("/prearms")
        msg = res.get("msg")
        if isinstance(msg, dict) and msg.get("arm") is True:
            return
        print(f"prearm 未通过: {res}")
        time.sleep(3)
    raise RuntimeError("prearm 检查超时")


def _ensure_disarmed(robot, timeout=10):
    """确保飞控已上锁。

    上个用例刚降落时, APM 处于落地检测/待自动上锁的收尾窗口, 此时
    arm + NAV_TAKEOFF 会被 ACK 但电机不转 (DISARM_DELAY 后自动上锁)。
    起飞前显式上锁并等待确认, 保证从干净的解锁状态起飞。
    """
    if robot.snapshot().get("arm", False) is not True:
        return
    print("起飞前飞控仍在解锁状态, 先上锁")
    http_post("/disarm")
    deadline = time.time() + timeout
    while time.time() < deadline:
        if robot.snapshot().get("arm", True) is not True:
            time.sleep(2)  # 避开 APM 落地检测收尾窗口
            return
        time.sleep(0.5)
    raise RuntimeError("等待飞控上锁超时")


def _takeoff_until_climbing(
    robot, alt, retries=3, z_tol=1.5, stale_check=15, total_timeout=60
):
    """下发起飞, success 即视为已接受, 轮询高度直到到达预定值。

    唯一成功判据是高度 |z - alt| <= z_tol; 不再用"是否开始爬升 +
    状态名快照"做二次确认 (起飞初期 地面->解锁->起飞 切换期间 WS
    快照滞后, 曾导致正常起飞被误判假 ACK 而同秒重复下发, 触发引擎
    'Broken Promise: Event was dropped')。

    仅当下发后 stale_check 秒高度仍 <0.5m (指令确实未被执行) 或响应
    非 success (含事件被丢弃) 时才重试, 最多 retries 次。
    """
    deadline = time.time() + total_timeout
    for attempt in range(1, retries + 1):
        _ensure_disarmed(robot)
        _wait_prearm_ok()
        res = http_post("/takeoff", data=dict(alt=alt))
        if res.get("status") != "success":
            print(f"第{attempt}次起飞被拒 (重试): {res}")
            time.sleep(3)
            continue
        # success: 轮询高度到位
        stale_deadline = time.time() + stale_check
        while time.time() < deadline:
            pos = robot.get_pos()
            z = pos[2] if pos else 0.0
            if abs(z - alt) <= z_tol:
                return
            if time.time() > stale_deadline:
                if z < 0.5:
                    break  # 指令确实未执行, 重新下发
                # 在爬但未到位: 继续轮询到 total_timeout
                stale_deadline = time.time() + stale_check
            time.sleep(0.5)
        if time.time() >= deadline:
            break
        print(f"第{attempt}次起飞 {stale_check}s 高度仍<0.5m (假 ACK), 重试")
    raise RuntimeError(
        f"起飞重试{retries}次仍未达到高度 {alt}m (当前: {robot.get_pos()})"
    )


class Takeoff(Step):
    """POST /takeoff, 等待到达目标高度后进入悬停。

    带验证重试: 落地后立即解锁起飞时, APM 可能假 ACK (电机不转,
    10s 后 DISARM_DELAY 自动上锁), 检测到未爬升则直接重试起飞。
    """

    def __init__(self, alt=8, timeout=180, z_tolerance=1.5, retries=3):
        self.name = f"起飞(alt={alt})"
        self.alt = alt
        self.timeout = timeout
        self.z_tol = z_tolerance
        self.retries = retries

    def run(self, robot):
        _takeoff_until_climbing(robot, self.alt, self.retries)
        robot.wait_for_state(STATE_HOVER, self.timeout)

    def check(self, robot):
        pos = robot.get_pos()
        if abs(pos[2] - self.alt) > self.z_tol:
            raise RuntimeError(f"高度未到位: {pos[2]:.2f} != {self.alt}")


class TakeoffReenter(Step):
    """起飞中再次触发起飞: 完全重入状态机并重新下发 NAV_TAKEOFF。

    APM 对空中 NAV_TAKEOFF 的接受/拒绝与时机相关 (非确定), 因此不
    断言具体响应, 只断言系统安全: 无论 success/error, 机体最终稳定
    回到悬停状态。
    """

    def __init__(self, first_alt=5, reenter_alt=12, wait_before_reenter=3, timeout=240):
        self.name = f"起飞中重入({first_alt}m 时再起飞)"
        self.first_alt = first_alt
        self.reenter_alt = reenter_alt
        self.wait = wait_before_reenter
        self.timeout = timeout

    def run(self, robot):
        _takeoff_until_climbing(robot, self.first_alt)
        cur = robot.get_state_name()
        if cur != STATE_TAKEOFF:
            raise RuntimeError(f"未处于起飞状态: {cur}")
        res2 = http_post("/takeoff", data=dict(alt=self.reenter_alt))
        print(f"空中重入起飞响应(接受任一): {res2}")
        # 安全兜底: 最终必须稳定悬停 (若第二次假 ACK 导致掉回地面, 重新起飞)
        if robot.get_state_name() == STATE_GROUND:
            _takeoff_until_climbing(robot, self.first_alt)
        robot.wait_for_state(STATE_HOVER, self.timeout)


class Land(Step):
    """POST /land, 等待回到地面状态并落地停稳。"""

    def __init__(self, timeout=240, z_tol=0.3, vz_tol=0.15):
        self.name = "降落"
        self.timeout = timeout
        self.z_tol = z_tol
        self.vz_tol = vz_tol

    def run(self, robot):
        res = http_post("/land", data=dict(do_pland=False))
        if res.get("status") != "success":
            raise RuntimeError(f"land 被拒: {res}")
        robot.wait_for_state(STATE_GROUND, self.timeout)

    def check(self, robot):
        deadline = time.time() + 20
        pos, vz = None, None
        while time.time() < deadline:
            pos = robot.get_pos()
            vz = self._estimate_vz(robot)
            if pos[2] < self.z_tol and abs(vz) < self.vz_tol:
                return
            time.sleep(0.5)
        raise RuntimeError(f"落地停稳校验失败: z={pos[2]:.2f} vz={vz:.2f}")

    @staticmethod
    def _estimate_vz(robot):
        pos = robot.get_pos()
        time.sleep(0.3)
        pos2 = robot.get_pos()
        return (pos2[2] - pos[2]) / 0.3


def gps_distance(lon1, lat1, lon2, lat2):
    """两点 GPS 水平距离 (米), haversine。"""
    from math import asin, cos, radians, sin, sqrt

    dlat = radians(lat2 - lat1)
    dlon = radians(lon2 - lon1)
    a = (
        sin(dlat / 2) ** 2
        + cos(radians(lat1)) * cos(radians(lat2)) * sin(dlon / 2) ** 2
    )
    return 2 * 6371000 * asin(sqrt(a))


class FlyWaypoint(Step):
    """POST /set_waypoint, 等待逐点到达后回到悬停。

    local=True (默认): ENU 米制 [x, y, z];
    local=False: GPS [lat, lon, alt], 到达判定用 haversine 距离。
    """

    def __init__(
        self, waypoints, speed=None, xy_tol=3.0, z_tol=1.5, timeout=600, local=True
    ):
        self.name = f"飞航点({len(waypoints)}个, {'ENU' if local else 'GPS'})"
        self.waypoints = [list(w) for w in waypoints]
        self.speed = speed
        self.xy_tol = xy_tol
        self.z_tol = z_tol
        self.timeout = timeout
        self.local = local

    def run(self, robot):
        data = dict(waypoint=self.waypoints, local=self.local)
        if self.speed is not None:
            data["speed"] = self.speed
        res = http_post("/set_waypoint", data=data)
        if res.get("status") != "success":
            raise RuntimeError(f"set_waypoint 被拒: {res}")
        robot.wait_for_state_any(WAYPOINT_STATES, 60)
        self._wait_all_reached(robot)

    def _not_reached(self, robot):
        """返回尚未到达的航点列表。"""
        pos = robot.get_pos()
        if not pos:
            return self.waypoints
        if self.local:
            return [
                w
                for w in self.waypoints
                if self._dist2d(pos, w) > self.xy_tol or abs(pos[2] - w[2]) > self.z_tol
            ]
        state = robot.snapshot()
        if not state.get("lat") or not state.get("lon"):
            return self.waypoints
        # GPS 航点顺序为 [lon, lat, alt] (dankong 约定)
        return [
            w
            for w in self.waypoints
            if gps_distance(state["lon"], state["lat"], w[0], w[1]) > self.xy_tol
            or abs(pos[2] - w[2]) > self.z_tol
        ]

    def _wait_all_reached(self, robot):
        deadline = time.time() + self.timeout
        while time.time() < deadline:
            if robot.get_state_name() == STATE_HOVER:
                return  # 航点完成 fallback 回悬停
            remaining = self._not_reached(robot)
            print(f"剩余未达航点: {len(remaining)}")
            time.sleep(1)
        raise RuntimeError("航点飞行超时")

    def check(self, robot):
        last = self.waypoints[-1]
        if self.local:
            pos = robot.get_pos()
            d = self._dist2d(pos, last)
            dz = pos[2] - last[2]
        else:
            state = robot.snapshot()
            pos = robot.get_pos()
            # GPS 航点顺序为 [lon, lat, alt] (dankong 约定)
            d = gps_distance(state["lon"], state["lat"], last[0], last[1])
            dz = pos[2] - last[2]
        if d > self.xy_tol or abs(dz) > self.z_tol:
            raise RuntimeError(f"终点偏差过大: xy={d:.2f}m dz={dz:.2f}m")

    @staticmethod
    def _dist2d(a, b):
        return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5


class RecordTakeoffPos(Step):
    """记录当前位置 (地面) 为起飞点, 供 Return 校验使用。"""

    name = "记录起飞点"

    def run(self, robot):
        pos = robot.get_pos()
        robot.takeoff_pos = (pos[0], pos[1])


class Return(Step):
    """POST /return: 飞回起飞点并自动降落, 等待回到地面状态。"""

    def __init__(self, timeout=600, xy_tol=3.0):
        self.name = "返航"
        self.timeout = timeout
        self.xy_tol = xy_tol

    def run(self, robot):
        res = http_post("/return", data=dict(waypoint=[], local=True))
        if res.get("status") != "success":
            raise RuntimeError(f"return 被拒: {res}")
        robot.wait_for_state(STATE_GROUND, self.timeout)

    def check(self, robot):
        pos = robot.get_pos()
        d = (
            (pos[0] - robot.takeoff_pos[0]) ** 2 + (pos[1] - robot.takeoff_pos[1]) ** 2
        ) ** 0.5
        if d > self.xy_tol:
            raise RuntimeError(f"未降落在起飞点附近: 偏差 {d:.2f}m")
