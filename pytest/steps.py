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
import math
import threading
import time
from dataclasses import dataclass, field

import requests
import websockets

HOST = "127.0.0.1"
PORT = 8000

# 状态名 (与各 State 的 static_name() 对应)
STATE_GROUND = "地面状态"
STATE_INIT = "初始状态"  # 节点刚 respawn 时先处于初始状态, 连上飞控后才切地面
STATE_TAKEOFF = "起飞状态"
STATE_HOVER = "悬停状态"
STATE_WAYPOINT = "航点模式"
STATE_WP_LIFT = "调整高度"  # WaypointState::LiftingState 叶子名
STATE_WP_EXEC = "执行航点"  # WaypointState::ExcuteState 叶子名
STATE_LAND = "降落状态"
# 航点任务的任一子状态 (WS state 帧上报的是叶子状态名)
WAYPOINT_STATES = (STATE_WAYPOINT, STATE_WP_LIFT, STATE_WP_EXEC)
# 在地面的状态集合 (初始状态同样"在地面", 不可降落)
GROUND_STATES = (STATE_GROUND, STATE_INIT)


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
        if robot.get_state_name() not in GROUND_STATES:
            with robot.lock:
                state_name = robot.state.get("state")
                age = time.time() - robot.state_ts
            print(
                f"当前不在地面 (state={state_name!r}, 快照 {age:.1f}s 前, "
                f"keys={sorted(robot.state.keys())}), 先降落"
            )
            res = http_post("/land", data=dict(do_pland=False))
            msg = str(res.get("msg", ""))
            if res.get("status") != "success":
                if "当前在地面" in msg:
                    # 后端认定在地面而 WS 快照仍是其他状态名 (快照滞后),
                    # 视为已在地面, 等状态机切到地面状态即可
                    print(f"land 被拒但后端认定在地面 ({msg}), 继续等待")
                else:
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


# ---------------- 指点控制 (/set_posvel) ----------------

# PosVelState 及其子状态叶子名 (WS state 帧上报叶子)
POSVEL_STATES = ("指点控制", "指点移动", "指点转向")


def gps_offset(lon, lat, dx_m, dy_m):
    """从 (lon, lat) 出发按米制偏移 (dx=东, dy=北) 求目标 GPS [lon, lat]。"""
    from math import cos, radians

    return [
        lon + dx_m / (111320.0 * cos(radians(lat))),
        lat + dy_m / 111320.0,
    ]


class _PosVelTargetStreamer:
    """后台线程: 以固定频率向 /set_posvel 下发按恒速移动的目标点。

    模拟 mavproxy 式用法 —— 目标持续刷新 (每次请求都会刷新 dankong 内部
    超时计时); 移动结束后对终点再连发 finish_repeat 次然后停止下发。
    """

    def __init__(
        self,
        start_lon,
        start_lat,
        alt,
        dir_e,
        dir_n,
        speed,
        duration,
        rate=1.0,
        cmd_timeout=3.0,
        vel=2,
        finish_repeat=3,
    ):
        norm = (dir_e**2 + dir_n**2) ** 0.5
        self._unit = (dir_e / norm, dir_n / norm)
        self._start = (start_lon, start_lat)
        self._alt = alt
        self._speed = speed
        self._duration = duration
        self._interval = 1.0 / rate
        self._cmd_timeout = cmd_timeout
        self._vel = vel
        self._finish_repeat = finish_repeat
        self.max_lag = 0.0  # 跟随过程中观测到的最大滞后 (m)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _target(self, t):
        e, n = self._unit[0] * self._speed * t, self._unit[1] * self._speed * t
        return gps_offset(self._start[0], self._start[1], e, n) + [self._alt]

    def _post(self, pos):
        res = http_post(
            "/set_posvel", data=dict(pos=pos, vel=self._vel, timeout=self._cmd_timeout)
        )
        return res.get("status") == "success"

    def _run(self):
        t0 = time.time()
        while not self._stop.is_set():
            t = time.time() - t0
            if t >= self._duration:
                break
            self._post(self._target(t))
            time.sleep(self._interval)
        # 目标已停: 对终点补发数次, 让飞机明确"停在这里"
        final = self._target(self._duration)
        for _ in range(self._finish_repeat):
            if self._stop.is_set():
                return
            self._post(final)
            time.sleep(self._interval)

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=10)


class FollowMovingTarget(Step):
    """1Hz 持续下发恒速移动目标, 验证跟随精度; 目标停止后飞机应停在
    终点 (≤1m), 不能冲过头继续飞。

    需要从悬停状态进入 (waypoint 场景传 back_states=WAYPOINT_STATES)。
    """

    def __init__(
        self,
        alt,
        dir_e=1.0,
        dir_n=0.0,
        speed=2.0,
        duration=20.0,
        rate=1.0,
        follow_tol=5.0,
        arrive_tol=1.0,
        drift_tol=1.0,
        stop_sample=2.0,
        check_stop=True,
        back_states=(STATE_HOVER,),
        settle_timeout=60,
        enter_timeout=30,
    ):
        self.name = f"跟随移动目标({speed}m/s x {duration}s @ {rate}Hz)"
        self.alt = alt
        self.dir = (dir_e, dir_n)
        self.speed = speed
        self.duration = duration
        self.rate = rate
        self.follow_tol = follow_tol
        self.arrive_tol = arrive_tol
        self.drift_tol = drift_tol
        self.stop_sample = stop_sample
        # 悬停场景应停稳; 航点场景指点结束后续飞剩余航点, 停止校验不适用
        self.check_stop = check_stop
        self.back_states = back_states
        self.settle_timeout = settle_timeout
        self.enter_timeout = enter_timeout
        self.lags = []  # (t, lag_m) 采样

    def run(self, robot):
        robot.wait_state_fresh()
        state = robot.snapshot()
        if not state.get("lon") or not state.get("lat"):
            raise RuntimeError(f"WS 快照无 GPS: {sorted(state.keys())}")
        self._stream_start = (state["lon"], state["lat"])
        streamer = _PosVelTargetStreamer(
            state["lon"],
            state["lat"],
            self.alt,
            self.dir[0],
            self.dir[1],
            self.speed,
            self.duration,
            self.rate,
        )
        try:
            robot.wait_for_state_any(POSVEL_STATES, self.enter_timeout)
            # 跟随过程: 抽样飞机与"当前理论目标点"的滞后
            deadline = time.time() + self.duration + 5
            start = time.time()
            while time.time() < deadline:
                if robot.get_state_name() not in POSVEL_STATES:
                    break
                t = time.time() - start
                if t < self.duration:
                    tgt = self._target_of(streamer, t, state)
                    lag = self._lag(robot, tgt)
                    if lag is not None:
                        self.lags.append((t, lag))
                time.sleep(1.0)
            # 目标已停止, 等状态机回退 (自动停止)
            robot.wait_for_state_any(self.back_states, self.settle_timeout)
        finally:
            streamer.stop()
        self.max_lag = max((lag for _, lag in self.lags), default=0.0)
        if self.max_lag > self.follow_tol:
            raise RuntimeError(
                f"跟随滞后超限: max={self.max_lag:.1f}m > {self.follow_tol}m "
                f"(采样 {len(self.lags)} 次)"
            )

    def _target_of(self, streamer, t, state):
        return streamer._target(t)

    def _lag(self, robot, tgt):
        state = robot.snapshot()
        if not state.get("lon") or not state.get("lat"):
            return None
        return gps_distance(state["lon"], state["lat"], tgt[0], tgt[1])

    def check(self, robot):
        if not self.check_stop:
            # 航点场景: 指点结束后任务续飞剩余航点, 终点精度/不过冲/停止
            # 校验均不适用 (续飞正确性由用例最后的悬停等待覆盖)
            print("航点场景, 跳过终点精度/停止校验")
            return
        stream_start = self._stream_start
        state = robot.snapshot()
        pos = robot.get_pos()
        # ① 终点精度 ≤1m
        final = gps_offset(
            stream_start[0],
            stream_start[1],
            self.dir[0] * self.speed * self.duration,
            self.dir[1] * self.speed * self.duration,
        )
        d = gps_distance(state["lon"], state["lat"], final[0], final[1])
        dz = pos[2] - self.alt
        if d > self.arrive_tol or abs(dz) > self.arrive_tol:
            raise RuntimeError(f"终点精度不足: xy={d:.2f}m dz={dz:.2f}m")
        # ② 不越过终点: 航向方向投影超出终点 ≤ arrive_tol
        e = (
            (state["lon"] - stream_start[0])
            * 111320.0
            * math.cos(math.radians(state["lat"]))
        )
        n = (state["lat"] - stream_start[1]) * 111320.0
        proj = e * self.dir[0] + n * self.dir[1]  # 航向方向投影 (m)
        track_len = self.speed * self.duration
        overshoot = proj - track_len
        if overshoot > self.arrive_tol:
            raise RuntimeError(
                f"冲过终点 {overshoot:.1f}m (投影 {proj:.1f}m / 航迹 {track_len:.1f}m)"
            )
        # ③ 停止: 采样位移
        p1 = robot.get_pos()
        time.sleep(self.stop_sample)
        p2 = robot.get_pos()
        drift = (
            (p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2 + (p2[2] - p1[2]) ** 2
        ) ** 0.5
        if drift > self.drift_tol:
            raise RuntimeError(f"到达后未停止: {self.stop_sample}s 内位移 {drift:.2f}m")


class PosVelGo(Step):
    """静态指点: POST /set_posvel, 等待到达目标点后回退并停稳。

    pos 支持 [lon,lat] 或 [lon,lat,alt] —— 后端忽略第三值, 目标高度
    始终为当前高度; expected_alt 显式传入期望保持的高度用于校验。
    """

    def __init__(
        self,
        pos,
        vel=2,
        timeout=60,
        expected_alt=None,
        xy_tol=1.5,
        z_tol=1.5,
        drift_tol=1.0,
        stop_sample=2.0,
        back_states=(STATE_HOVER,),
        enter_timeout=30,
    ):
        self.name = f"静态指点({pos}, vel={vel})"
        self.pos = list(pos)
        self.vel = vel
        self.timeout = timeout
        self.expected_alt = expected_alt
        self.xy_tol = xy_tol
        self.z_tol = z_tol
        self.drift_tol = drift_tol
        self.stop_sample = stop_sample
        self.back_states = back_states
        self.enter_timeout = enter_timeout

    def run(self, robot):
        robot.wait_state_fresh()
        res = http_post(
            "/set_posvel", data=dict(pos=self.pos, vel=self.vel, timeout=self.timeout)
        )
        if res.get("status") != "success":
            raise RuntimeError(f"set_posvel 被拒: {res}")
        robot.wait_for_state_any(POSVEL_STATES, self.enter_timeout)
        robot.wait_for_state_any(self.back_states, self.timeout + 60)

    def check(self, robot):
        state = robot.snapshot()
        pos = robot.get_pos()
        d = gps_distance(state["lon"], state["lat"], self.pos[0], self.pos[1])
        if d > self.xy_tol:
            raise RuntimeError(f"到达精度不足: xy={d:.2f}m")
        if (
            self.expected_alt is not None
            and abs(pos[2] - self.expected_alt) > self.z_tol
        ):
            raise RuntimeError(
                f"高度偏差过大: z={pos[2]:.2f} != {self.expected_alt:.2f}"
            )
        # 停稳: 采样位移
        p1 = robot.get_pos()
        time.sleep(self.stop_sample)
        p2 = robot.get_pos()
        drift = (
            (p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2 + (p2[2] - p1[2]) ** 2
        ) ** 0.5
        if drift > self.drift_tol:
            raise RuntimeError(f"到达后未停稳: {self.stop_sample}s 内漂移 {drift:.2f}m")


class PosVelTimeout(Step):
    """只发一次远目标 (不刷新): 超时后应自动回退且远未到达目标。"""

    def __init__(
        self,
        pos,
        timeout=5,
        vel=2,
        back_states=(STATE_HOVER,),
        enter_timeout=30,
        far_tol=50.0,
    ):
        self.name = f"指点超时(timeout={timeout}s)"
        self.pos = list(pos)
        self.timeout = timeout
        self.vel = vel
        self.back_states = back_states
        self.enter_timeout = enter_timeout
        self.far_tol = far_tol

    def run(self, robot):
        robot.wait_state_fresh()
        res = http_post(
            "/set_posvel", data=dict(pos=self.pos, vel=self.vel, timeout=self.timeout)
        )
        if res.get("status") != "success":
            raise RuntimeError(f"set_posvel 被拒: {res}")
        robot.wait_for_state_any(POSVEL_STATES, self.enter_timeout)
        # 超时后状态机回退 (留裕量覆盖网络/状态帧延迟)
        robot.wait_for_state_any(self.back_states, self.timeout + 20)

    def check(self, robot):
        state = robot.snapshot()
        d = gps_distance(state["lon"], state["lat"], self.pos[0], self.pos[1])
        if d < self.far_tol:
            raise RuntimeError(
                f"超时回退后不应已到达目标: 距目标仅 {d:.1f}m (<{self.far_tol}m)"
            )


class ExpectPosVelRejected(Step):
    """非 waypoint/hover 状态下 /set_posvel 应被拒绝。"""

    name = "指点指令应被拒绝"

    def __init__(self, pos=None):
        self.pos = pos or [116.0, 39.0, 10.0]

    def run(self, robot):
        res = http_post("/set_posvel", data=dict(pos=self.pos, vel=2, timeout=10))
        if res.get("status") == "success":
            raise RuntimeError(f"set_posvel 未被拒绝: {res}")
        print(f"set_posvel 按预期被拒: {res}")


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
