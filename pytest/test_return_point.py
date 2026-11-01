#!/usr/bin/env python
# coding=utf-8
"""返航点选择语义测试。

新语义 (RETURN 点三级 fallback):
  1. 有航线任务 -> 返回任务开始的地方 (任务受理时的位置, 用一次即清空);
  2. 无任务、dankong 起飞过 -> 返回 dankong 记录的起飞点;
  3. 都没有 (外部起飞) -> 返回飞控 HOME_POSITION 记录的起飞点;
  4. 全无 -> /return 被拒绝。

用 set_posvel 把飞机挪离起飞点 (30m), 使得"起飞点 / 任务起点 / 飞控 home"
三个候选点两两可区分, 校验落点即能确定返航语义。

外部起飞用例通过 dronekit 连 SITL 的额外 TCP 端口 5762
(dankong/MAVSDK 占用 5760, SITL 顺序开放 5760/5761/5762, 互不干扰)。
"""
import atexit
import subprocess
import time

from steps import (
    GROUND_STATES,
    STATE_GROUND,
    STATE_HOVER,
    EnsureGround,
    FlyWaypoint,
    PrearmCheck,
    Sleep,
    Step,
    Takeoff,
    gps_distance,
    gps_offset,
    http_get,
    http_post,
    mk_gps_points,
    wait_prearms_ready,
)

import pytest

RETURN_TOL = 4.0  # 落点校验容差 (m); 候选点间距 30m, 足以区分
MOVE_DIST = 30.0  # set_posvel 挪机距离 (m)

_DANKONG_CLEANUP_REGISTERED = False


def _stop_dankong_roslaunch():
    """SIGINT 停掉 roslaunch (含 dankong 节点, node 是 respawn 的,
    只杀节点会被拉回来)。"""
    subprocess.run(["pkill", "-INT", "-f", "dankong.launch"], check=False)


def _dankong_available(timeout=2):
    """dankong web 服务是否在线。"""
    import requests

    try:
        requests.get(f"http://127.0.0.1:8000/prearms", timeout=timeout)
        return True
    except Exception:  # noqa: BLE001
        return False


class StopDankong(Step):
    """停掉 dankong (整个 roslaunch): 外部起飞期间避免其悬停零速
    burst 作废 NAV_TAKEOFF。起飞成功后用 StartDankong 拉起。"""

    name = "停止dankong"

    def run(self, robot):
        if not _dankong_available():
            print("dankong 已离线, 跳过停止")
            return
        _stop_dankong_roslaunch()
        deadline = time.time() + 30
        while time.time() < deadline:
            if not _dankong_available(timeout=1):
                print("dankong 已停止")
                return
            time.sleep(1)
        raise RuntimeError("dankong 30s 内未停止")


class StartDankong(Step):
    """重新拉起 dankong (roslaunch), 等待 web 服务与 prearm 就绪。"""

    name = "启动dankong"

    def run(self, robot):
        global _DANKONG_CLEANUP_REGISTERED
        # 追加写 run_test.sh 同一份日志 (绝对路径, 不依赖 pytest 工作目录)
        from pathlib import Path

        dankong_log = Path(__file__).parent / "logs" / "dankong.log"
        dankong_log.parent.mkdir(parents=True, exist_ok=True)
        log = open(dankong_log, "ab")
        robot.dankong_proc = subprocess.Popen(
            ["roslaunch", "dankong", "dankong.launch"],
            stdout=log,
            stderr=subprocess.STDOUT,
        )
        if not _DANKONG_CLEANUP_REGISTERED:
            atexit.register(_stop_dankong_roslaunch)
            _DANKONG_CLEANUP_REGISTERED = True
        wait_prearms_ready(180)


class AnchorReturnTarget(Step):
    """记录当前位置为返航校验目标 (只在第一条航线前调用)。"""

    name = "记录返航目标点(当前位置)"

    def run(self, robot):
        robot.wait_state_fresh()
        s = robot.snapshot()
        robot.return_target_gps = (s["lon"], s["lat"])


class MoveBy(Step):
    """悬停中用 /set_posvel 指点移动 (de 东向 / dn 北向, 米)。

    dankong 的指点状态在"最后一次下发后 timeout 秒无刷新"才回退悬停,
    因此用短超时(3s)+约1Hz持续刷新, 到达后补发终点数次再停,
    避免到达后空等整个指令超时。
    """

    def __init__(self, de, dn, alt=10, timeout=120, cmd_timeout=3.0, rate=1.0):
        self.name = f"指点移动(东{de}m 北{dn}m)"
        self.de = de
        self.dn = dn
        self.alt = alt
        self.timeout = timeout  # 整个移动过程的总等待上限
        self.cmd_timeout = cmd_timeout  # 单次 /set_posvel 的指令超时
        self.rate = rate

    def run(self, robot):
        robot.wait_state_fresh()
        # 起飞检查通过并确认 dankong 进入悬停后才指点 (外部起飞场景
        # dankong 靠 check_hover 自动切悬停, 正常几秒内完成)
        robot.wait_for_state(STATE_HOVER, 90)
        s = robot.snapshot()
        target = gps_offset(s["lon"], s["lat"], self.de, self.dn) + [self.alt]
        self._target_lon, self._target_lat = target[0], target[1]
        interval = 1.0 / self.rate
        deadline = time.time() + self.timeout
        # 流式刷新直到到达 (xy 1.5m)
        while time.time() < deadline:
            cur = robot.snapshot()
            if gps_distance(cur["lon"], cur["lat"], target[0], target[1]) <= 1.5:
                break
            http_post(
                "/set_posvel",
                data=dict(pos=target, vel=2, timeout=self.cmd_timeout),
            )
            time.sleep(interval)
        else:
            raise RuntimeError(f"指点移动超时未到达 (>{self.timeout}s)")
        # 对终点补发数次, 让 dankong 明确"停在这里"
        for _ in range(3):
            http_post(
                "/set_posvel",
                data=dict(pos=target, vel=2, timeout=self.cmd_timeout),
            )
            time.sleep(interval)
        # 指点超时(3s)后状态机回退悬停
        robot.wait_for_state_any((STATE_HOVER,), self.cmd_timeout + 30)

    def check(self, robot):
        s = robot.snapshot()
        d = gps_distance(s["lon"], s["lat"], self._target_lon, self._target_lat)
        if d > 1.5:
            raise RuntimeError(f"指点到达精度不足: xy={d:.2f}m")


class FlyMission(Step):
    """以当前实时位置为参考发一条航线。

    首点 [0,0,高度] 为起飞占位 (仅 z 生效, x,y 被后端抛弃, 空中则整点
    抛弃), 真实航点从第二个开始, 逐点按序校验。
    """

    def __init__(self, offsets, alt=10, timeout=600):
        self.name = f"飞航线({len(offsets)}个航点)"
        self.offsets = list(offsets)
        self.alt = alt
        self.timeout = timeout

    def run(self, robot):
        robot.wait_state_fresh()
        wps = [[0, 0, self.alt]] + mk_gps_points(
            robot, [(de, dn, self.alt) for de, dn in self.offsets]
        )
        flyer = FlyWaypoint(wps, expected=wps[1:], timeout=self.timeout)
        flyer.run(robot)
        flyer.check(robot)


class ReturnToTarget(Step):
    """POST /return (不带航点), 等待降落并校验落点接近 robot.return_target_gps。"""

    def __init__(self, timeout=600, xy_tol=RETURN_TOL):
        self.name = "返航并校验落点"
        self.timeout = timeout
        self.xy_tol = xy_tol

    def run(self, robot):
        res = http_post("/return", data=dict(waypoint=[]))
        if res.get("status") != "success":
            raise RuntimeError(f"return 被拒: {res}")
        robot.wait_for_state(STATE_GROUND, self.timeout)

    def check(self, robot):
        s = robot.snapshot()
        target = robot.return_target_gps
        d = gps_distance(s["lon"], s["lat"], target[0], target[1])
        if d > self.xy_tol:
            raise RuntimeError(f"落点偏离返航目标 {d:.2f}m (> {self.xy_tol}m)")
        print(f"落点偏差 {d:.2f}m")


# ---------------- 外部起飞 (dronekit, 不经过 dankong) ----------------


class ExternalTakeoff(Step):
    """dronekit 连 SITL (tcp:127.0.0.1:5762), GUIDED + arm + simple_takeoff。

    dankong 检测到 armed && 油门高于阈值 (check_hover) 后会自动从
    地面状态切到悬停状态, 但不记录 takeoff_enu。
    起飞完成判据: 相对高度达到阈值且连续 5 拍稳定 (地面待机也可能被
    dankong 判为 hover, 不能用 hover 状态当起飞完成依据)。
    """

    def __init__(self, alt=10, conn_str="tcp:127.0.0.1:5762", timeout=120):
        self.name = f"外部起飞(alt={alt}, {conn_str})"
        self.alt = alt
        self.conn_str = conn_str
        self.timeout = timeout

    def run(self, robot):
        from dronekit import APIException, VehicleMode, connect

        vehicle = connect(self.conn_str, wait_ready=True, timeout=60)
        robot.dronekit_vehicle = vehicle
        # dronekit 预检: is_armable = 指南针/EKF/电压等解锁条件就绪
        deadline = time.time() + 120
        while not vehicle.is_armable:
            if time.time() > deadline:
                raise RuntimeError("120s 内飞控解锁条件 (is_armable) 未就绪")
            time.sleep(1)
        # 切 GUIDED 并确认
        vehicle.mode = VehicleMode("GUIDED")
        deadline = time.time() + 15
        while vehicle.mode.name != "GUIDED":
            if time.time() > deadline:
                raise RuntimeError(f"切 GUIDED 失败, 当前={vehicle.mode.name}")
            time.sleep(0.2)
        # 解锁: 一直重试直到成功
        for attempt in range(1, 11):
            try:
                vehicle.arm(wait=True)
                break
            except APIException as e:
                print(f"第{attempt}次解锁被拒: {e}")
                time.sleep(2)
        else:
            raise RuntimeError("重试10次解锁均未通过")
        # dankong 在油门爬过 throttle_thresh(0.1) 时切悬停, 并发一次
        # 0.5s 零速度 burst (GUIDED 下会作废进行中的 NAV_TAKEOFF),
        # 因此 takeoff 后检测爬升, 未爬升则等 burst 结束后重发
        for attempt in range(1, 4):
            vehicle.simple_takeoff(self.alt)
            t0 = time.time()
            base = None
            climbed = False
            while time.time() - t0 < 5:
                alt = vehicle.location.global_relative_frame.alt
                if base is None:
                    base = alt
                if alt - base > 0.5:  # 5s 内爬升超过 0.5m 即起飞成功
                    climbed = True
                    break
                time.sleep(0.2)
            if climbed:
                break
            print(
                f"第{attempt}次外部起飞未爬升 (疑似被悬停零速 burst 覆盖), "
                "等待后重发"
            )
            time.sleep(2)
        else:
            raise RuntimeError("重试3次外部起飞均未爬升")
        # 起飞完成判据: 相对高度达到阈值且连续 5 拍稳定
        near = 0
        deadline = time.time() + self.timeout
        while time.time() < deadline:
            alt = vehicle.location.global_relative_frame.alt
            if abs(alt - self.alt) <= 1.5:
                near += 1
                if near >= 5:  # 连续 5 拍在阈值附近, 视为到位并稳定
                    break
            else:
                near = 0
            time.sleep(0.5)
        else:
            raise RuntimeError(f"外部起飞超时, 相对高度未稳定在 {self.alt}m")


class RecordFcuHome(Step):
    """通过 dronekit 读取飞控 home_location 记录为返航校验目标。"""

    name = "记录飞控home点"

    def __init__(self, conn_str="tcp:127.0.0.1:5762", timeout=30):
        self.name = "记录飞控home点"
        self.conn_str = conn_str
        self.timeout = timeout

    def run(self, robot):
        # 复用外部起飞已建立的连接 (SITL 对第二条 TCP 链路可能不回心跳)
        vehicle = getattr(robot, "dronekit_vehicle", None)
        if vehicle is None:
            vehicle = connect(self.conn_str, wait_ready=True, timeout=60)
            robot.dronekit_vehicle = vehicle
        vehicle.commands.download()
        vehicle.commands.wait_ready()
        home = vehicle.home_location
        if home is None:
            raise RuntimeError("未获取到飞控 home_location")
        robot.return_target_gps = (home.lon, home.lat)
        print(f"飞控 home: {robot.return_target_gps}")


class CloseDronekit(Step):
    name = "关闭dronekit连接"

    def run(self, robot):
        vehicle = getattr(robot, "dronekit_vehicle", None)
        if vehicle is not None:
            try:
                vehicle.close()
            except Exception:  # noqa: BLE001
                pass


class ExpectReturnRejectedOnGround(Step):
    """地面阶段 POST /return(空航点) 应被直接拒绝, 且不进入起飞/航点状态。"""

    name = "地面返航应被拒绝"

    def run(self, robot):
        robot.wait_state_fresh()
        res = http_post("/return", data=dict(waypoint=[]))
        if res.get("status") == "success":
            raise RuntimeError(f"地面 return 未被拒绝: {res}")
        print(f"地面 return 按预期被拒: {res}")

    def check(self, robot):
        deadline = time.time() + 5
        while time.time() < deadline:
            state = robot.get_state_name()
            if state not in GROUND_STATES:
                raise RuntimeError(f"地面 return 被拒后仍发生状态迁移: {state}")
            time.sleep(0.5)


# ---------------- 用例 ----------------


def test_return_mission_start_single_wp(check):
    """set_posvel 挪离起飞点 -> 单航点任务 -> /return 应回到任务开始处。"""
    check(
        [
            EnsureGround(),
            PrearmCheck(),
            Takeoff(alt=10),
            Sleep(2),
            MoveBy(MOVE_DIST, 0),
            AnchorReturnTarget(),  # 任务起点 = 挪机后的位置
            FlyMission([(15, 0)]),
            Sleep(2),
            ReturnToTarget(),
        ]
    )


def test_return_mission_start_multi_wp(check):
    """连发多条航线后 /return: 应回到第一条航线的开始位置。

    任务起点"首条优先": 任务1 受理时记录 A 点; 任务2 受理不覆盖;
    RETURN 消耗后清空。
    """
    check(
        [
            EnsureGround(),
            PrearmCheck(),
            Takeoff(alt=10),
            Sleep(2),
            MoveBy(MOVE_DIST, 0),
            AnchorReturnTarget(),  # 任务1 起点 = A (返航校验目标)
            FlyMission([(15, 0)]),
            Sleep(2),
            MoveBy(-MOVE_DIST / 2, MOVE_DIST / 2),
            # 任务2 以当前位置 (B) 为参考, 不覆盖任务1 的记录
            FlyMission([(0, 15), (-15, 15)]),
            Sleep(2),
            ReturnToTarget(),  # 应回到 A (任务1 开始处), 而非 B
        ]
    )


def test_return_takeoff_point_when_no_mission(check):
    """无航线任务: set_posvel 挪离后 /return 应回到 dankong 记录的起飞点。"""
    check(
        [
            EnsureGround(),
            PrearmCheck(),
            AnchorReturnTarget(),  # 地面上记录 = dankong 起飞点
            Takeoff(alt=10),
            Sleep(2),
            MoveBy(MOVE_DIST, 0),
            ReturnToTarget(),
        ]
    )


def test_return_ground_empty_rejected(check):
    """地面调用航点为空的返航: 直接拒绝, 不允许起飞。"""
    check(
        [
            EnsureGround(),
            PrearmCheck(),
            ExpectReturnRejectedOnGround(),
        ]
    )


@pytest.mark.parametrize("conn_str", ["tcp:127.0.0.1:5762"])
def test_return_fcu_home_when_external_takeoff(check, conn_str):
    """外部 dronekit 起飞 (dankong 无起飞点/无任务): /return 应回飞控 home。"""
    pytest.importorskip("dronekit")  # 外部起飞用例, 见 pytest/requirements.txt
    check(
        [
            EnsureGround(),
            StopDankong(),  # 先停 dankong, 外部起飞不受其悬停零速 burst 干扰
            ExternalTakeoff(alt=10, conn_str=conn_str),  # dronekit 自带预检+解锁重试
            RecordFcuHome(),
            StartDankong(),  # 起飞成功后拉回, 继续指点/返航
            Sleep(2),
            MoveBy(MOVE_DIST, 0),
            ReturnToTarget(),
            CloseDronekit(),
        ]
    )
