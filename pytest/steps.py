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
            except Exception:  # noqa: BLE001
                self.connected.clear()
            time.sleep(1)

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

    def __init__(self, robot):
        self.robot = robot
        self.report = MissionReport()

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

    def __init__(self, settle=3):
        self.settle = settle

    def run(self, robot):
        if robot.get_state_name() != STATE_GROUND:
            print("当前不在地面, 先降落")
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


def _takeoff_until_climbing(robot, alt, retries=3, climb_check=15):
    """下发起飞并确认真正爬升。

    每次尝试先做 prearm 检查再调 /takeoff; 若假 ACK (APM ACK 成功但
    不执行) 则直接再次调用起飞 (重入机制允许重复下发), 不做 land。
    """
    for attempt in range(1, retries + 1):
        _wait_prearm_ok()
        res = http_post("/takeoff", data=dict(alt=alt))
        if res.get("status") != "success":
            raise RuntimeError(f"takeoff 被拒 (第{attempt}次): {res}")
        if _is_climbing(robot, climb_check):
            return
        print(f"第{attempt}次起飞未爬升 (假 ACK), 直接重试起飞")
    raise RuntimeError(f"起飞重试{retries}次仍未爬升")


def _is_climbing(robot, climb_check=15):
    """climb_check 秒内高度超过 0.5m 即认为起飞真正执行。"""
    deadline = time.time() + climb_check
    while time.time() < deadline:
        pos = robot.get_pos()
        if pos and pos[2] > 0.5:
            return True
        if robot.get_state_name() == STATE_GROUND:
            return False
        time.sleep(0.5)
    return False


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


class FlyWaypoint(Step):
    """POST /set_waypoint (local=true, ENU 米制), 等待逐点到达后回到悬停。"""

    def __init__(self, waypoints, speed=None, xy_tol=3.0, z_tol=1.5, timeout=600):
        self.name = f"飞航点({len(waypoints)}个)"
        self.waypoints = [list(w) for w in waypoints]
        self.speed = speed
        self.xy_tol = xy_tol
        self.z_tol = z_tol
        self.timeout = timeout

    def run(self, robot):
        data = dict(waypoint=self.waypoints, local=True)
        if self.speed is not None:
            data["speed"] = self.speed
        res = http_post("/set_waypoint", data=data)
        if res.get("status") != "success":
            raise RuntimeError(f"set_waypoint 被拒: {res}")
        robot.wait_for_state_any(WAYPOINT_STATES, 60)
        self._wait_all_reached(robot)

    def _wait_all_reached(self, robot):
        deadline = time.time() + self.timeout
        while time.time() < deadline:
            if robot.get_state_name() == STATE_HOVER:
                return  # 航点完成 fallback 回悬停
            pos = robot.get_pos()
            if pos:
                remaining = [
                    w
                    for w in self.waypoints
                    if self._dist2d(pos, w) > self.xy_tol
                    or abs(pos[2] - w[2]) > self.z_tol
                ]
                print(f"剩余未达航点: {len(remaining)}")
            time.sleep(1)
        raise RuntimeError("航点飞行超时")

    def check(self, robot):
        pos = robot.get_pos()
        last = self.waypoints[-1]
        d = self._dist2d(pos, last)
        if d > self.xy_tol or abs(pos[2] - last[2]) > self.z_tol:
            raise RuntimeError(f"终点偏差过大: xy={d:.2f}m dz={pos[2] - last[2]:.2f}m")

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
