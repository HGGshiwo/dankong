#!/usr/bin/env python
# coding=utf-8
"""pytest 公共 fixture: robot 客户端 + 报告输出 + 每用例重启飞控与 dankong。"""
import atexit
import os
import signal
import socket
import subprocess
import sys
import time
import uuid
from pathlib import Path

import psutil

import pytest

sys.path.insert(0, str(Path(__file__).parent))
from steps import MissionReport, MissionRunner, RobotClient, wait_prearms_ready

REPORT_PATH = Path(__file__).parent / "report.md"
LOG_DIR = Path(__file__).parent / "logs"
SITL_LOG = Path(__file__).parent / "logs" / "sitl.log"

# 报告跨用例累积 (每用例的 robot/WS 客户端全新, 但报告汇总是一份)
_SHARED_REPORT = MissionReport()

# 本 fixture 拉起的 SITL 进程与会话狗牌 (清理时按狗牌精确击杀)
_sitl_proc = None
_sitl_session_id = None

# 本 fixture 拉起的 dankong roslaunch 进程与当前参数
_dankong_proc = None
_dankong_args = None
_restore_registered = False


def _dankong_roslaunch_alive():
    """dankong roslaunch / 节点 / 8000 监听任一存活即真。"""
    for pattern in ("dankong.launch", "dankong_.*_node"):
        if (
            subprocess.run(["pgrep", "-f", pattern], capture_output=True).returncode
            == 0
        ):
            return True
    try:
        socket.create_connection(("127.0.0.1", 8000), timeout=0.5).close()
        return True
    except OSError:
        return False


def _stop_dankong_roslaunch():
    """确定性停止: SIGINT -> 轮询确认 -> 不灭升级 SIGKILL -> 等 8000 释放。

    只杀 dankong 节点会被 roslaunch respawn 拉回来, 必须连 roslaunch
    一起停; 残留实例会与新实例争抢 8000/FCU 连接。
    """
    subprocess.run(["pkill", "-INT", "-f", "dankong.launch"], check=False)
    deadline = time.time() + 10
    while time.time() < deadline:
        if not _dankong_roslaunch_alive():
            return
        time.sleep(0.5)
    print("[conftest] SIGINT 未停干净, 升级 SIGKILL")
    subprocess.run(["pkill", "-9", "-f", "dankong.launch"], check=False)
    subprocess.run(["pkill", "-9", "-f", "dankong_.*_node"], check=False)
    deadline = time.time() + 10
    while time.time() < deadline:
        if not _dankong_roslaunch_alive():
            return
        time.sleep(0.5)
    raise RuntimeError("dankong roslaunch 无法停止 (端口 8000 仍被占用)")


def _start_dankong_roslaunch(extra_args=()):
    global _dankong_proc, _dankong_args
    log = LOG_DIR / "dankong.log"
    log.parent.mkdir(parents=True, exist_ok=True)
    # 继承 pytest 环境 (DK_FOLLOW_COOL_TIME 等经此传给 dankong 节点)
    _dankong_proc = subprocess.Popen(
        ["roslaunch", "dankong", "dankong.launch", *extra_args],
        stdout=open(log, "ab"),
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    _dankong_args = tuple(extra_args)


def _ensure_dankong_roslaunch(extra_args):
    """按需启动: 已按相同参数运行则复用, 否则确定性停旧后按参数拉起。"""
    global _restore_registered
    extra_args = tuple(extra_args)
    if (
        _dankong_proc is not None
        and _dankong_proc.poll() is None
        and _dankong_args == extra_args
    ):
        return  # 复用本会话已拉起的实例
    if _dankong_roslaunch_alive():
        print("\n[fresh_dankong] 停止已存在的 dankong roslaunch...")
        _stop_dankong_roslaunch()
    if not _restore_registered:
        atexit.register(_stop_dankong_roslaunch)
        _restore_registered = True
    print(f"[fresh_dankong] 启动 dankong roslaunch (args={list(extra_args)})...")
    _start_dankong_roslaunch(extra_args)


def _wait_fcu_tcp(port=5760, timeout=120):
    """等待 SITL 的 MAVLink TCP 端口就绪 (与 run_test.sh 的判定一致)。"""
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=1):
                return
        except OSError:
            time.sleep(1)
    raise RuntimeError(f"SITL 端口 {port} 启动超时, sitl.log 尾部:\n{_read_log_tail()}")


def _read_log_tail(n=15):
    """读 sitl.log 尾部, 用于启动失败时直接暴露原因。"""
    try:
        lines = SITL_LOG.read_text(errors="replace").splitlines()
        return "\n".join(lines[-n:])
    except OSError:
        return "(sitl.log 不存在)"


def session_cleanup(session_id):
    """按环境变量狗牌精确清理: sim_vehicle 与 arducopter/xterm 等子进程
    全部继承 MY_ROS_SITL_SESSION, 逐个 SIGKILL; 最后进程组再补一刀。
    (与 test/test/test_helper.py::sitl_env 的清理逻辑一致)
    """
    for p in psutil.process_iter(["pid", "name", "environ"]):
        try:
            env = p.info.get("environ")
            if env and env.get("MY_ROS_SITL_SESSION") == session_id:
                p.kill()
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
    try:
        os.killpg(os.getpgid(_sitl_proc.pid), 9)
    except OSError:
        pass
    time.sleep(2.0)


def _kill_sitl_binaries():
    """按进程查询并击杀所有 SITL 仿真二进制 (含 xterm 包装进程)。

    匹配: cmdline 含 ardupilot/build/sitl/bin/arducopter, 或进程名
    为 arducopter/ArduCopter。
    """
    for p in psutil.process_iter(["pid", "name", "cmdline"]):
        try:
            cmd = " ".join(p.info.get("cmdline") or [])
            name = p.info.get("name") or ""
            if "ardupilot/build/sitl/bin" in cmd or name in (
                "arducopter",
                "ArduCopter",
            ):
                p.kill()
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue


def _stop_sitl():
    """停掉 SITL 并确认 5760 端口已释放。"""
    global _sitl_proc, _sitl_session_id
    if _sitl_proc is not None:
        try:
            os.killpg(os.getpgid(_sitl_proc.pid), signal.SIGTERM)
        except (ProcessLookupError, PermissionError):
            pass
    if _sitl_session_id is not None:
        session_cleanup(_sitl_session_id)
    _sitl_proc = None
    _sitl_session_id = None
    # 按仿真二进制路径兜底击杀 (覆盖无狗牌的遗留实例), 连杀两轮
    _kill_sitl_binaries()
    time.sleep(1)
    _kill_sitl_binaries()
    # 等端口真正释放, 避免新实例 bind 失败或连上旧飞控
    deadline = time.time() + 15
    while time.time() < deadline:
        try:
            with socket.create_connection(("127.0.0.1", 5760), timeout=1):
                time.sleep(1)  # 仍有连接, 旧实例还活着
        except OSError:
            return
    print(
        "[警告] SITL 端口 5760 在 15s 内未释放, 可能存在残留实例, "
        "请手动检查: pgrep -af arducopter"
    )


@pytest.fixture(autouse=True)
def fresh_dankong(request):
    """每个用例开始前整组重启 SITL 飞控和 dankong 节点 (所有用例同一路径)。

    顺序保证 dankong 总在飞控就绪后才被 respawn (其 web 服务依赖飞控):
      1) 停 SITL (进程组 + 按狗牌击杀, 等 5760 释放);
      2) 启动 SITL, 等 tcp:5760 (飞控参数/航点/EKF home 全部复位);
      3) 飞控就绪后 kill dankong, roslaunch respawn 拉起全新节点
         (状态机/跟随冷却/参数缓存复位), 睁眼即有活飞控;
      4) 等 GET /prearms success (MAVSDK 连接 + EKF 收敛完成)。

    标记 @pytest.mark.no_restart 可跳过 (用于不依赖节点状态的纯计算用例)。

    dankong roslaunch 由本 fixture 按需管理 (run_test.sh 不再启动):
      - 测试模块声明 DANKONG_LAUNCH_ARGS (如代理测试的
        ["mavsdk_url:=tcp://127.0.0.1:20000"]) 时按这些参数拉起;
      - 未声明用默认参数 (mavsdk_url 走 yaml);
      - 已按相同参数运行则复用; teardown 时确定性停止, 不跨用例残留。
    """
    if request.node.get_closest_marker("no_restart"):
        yield
        return
    global _sitl_proc, _sitl_session_id
    t0 = time.time()
    print("\n[fresh_dankong] 停旧 SITL...")
    _stop_sitl()
    SITL_LOG.parent.mkdir(parents=True, exist_ok=True)
    # 唯一会话狗牌: 注入环境, 子进程 (xterm/arducopter/mavproxy) 全部继承,
    # 清理时按狗牌精确击杀, 见 test/test/test_helper.py::sitl_env 同款实现
    _sitl_session_id = f"sitl_test_{uuid.uuid4().hex[:8]}"
    custom_env = os.environ.copy()
    custom_env["MY_ROS_SITL_SESSION"] = _sitl_session_id
    with open(SITL_LOG, "ab") as log:
        _sitl_proc = subprocess.Popen(
            ["sim_vehicle.py", "-v", "Copter", "--no-mavproxy", "--no-rebuild"],
            stdout=log,
            stderr=subprocess.STDOUT,
            env=custom_env,
            preexec_fn=os.setsid,
            cwd=str(Path(__file__).parents[1]),
        )
    print(f"[fresh_dankong] SITL pid={_sitl_proc.pid}, 等待存活...")
    # 启动即退出的检测: sim_vehicle 秒退时直接把日志尾部抛出来
    for _ in range(10):
        if _sitl_proc.poll() is not None:
            raise RuntimeError(
                f"SITL 启动即退出 (exit={_sitl_proc.returncode}), "
                f"sitl.log 尾部:\n{_read_log_tail()}"
            )
        time.sleep(1)
    print("[fresh_dankong] 等 tcp:5760...")
    _wait_fcu_tcp()
    # 飞控就绪后按需启动 dankong roslaunch (带模块声明的参数, 默认无),
    # 再重启 dankong 节点, 保证 respawn 出的节点直接连上活飞控
    _ensure_dankong_roslaunch(getattr(request.module, "DANKONG_LAUNCH_ARGS", ()))
    print("[fresh_dankong] 重启 dankong 节点...")
    subprocess.run(["pkill", "-f", "dankong_.*_node"], check=False)
    print("[fresh_dankong] 等 /prearms 就绪...")
    wait_prearms_ready(180)
    print(f"\n[飞控+dankong 已重启并就绪, 耗时 {time.time() - t0:.0f}s]")
    yield
    # 用例结束即清理本用例的 SITL 与 dankong roslaunch (下个用例开头还会
    # 再清一次, 幂等双保险); 否则进程退出后会残留占端口
    print("[fresh_dankong] 用例结束, 停 SITL 与 dankong...")
    _stop_sitl()
    _stop_dankong_roslaunch()


@pytest.fixture
def robot(fresh_dankong):
    """每个用例全新的 WS 客户端。

    显式依赖 fresh_dankong: 创建时机保证在飞控+dankong 重启就绪之后,
    状态快照不跨用例残留。客户端自身不做任何就绪等待 (那是
    fresh_dankong 的职责), 就绪后的 pos/状态等待由用例步骤负责。
    """
    client = RobotClient()
    yield client
    client.close()


@pytest.fixture
def report(robot):
    runner = MissionRunner(robot, report=_SHARED_REPORT)
    yield runner
    # 每个用例结束都落盘累积结果, 最终 report.md 包含全部用例
    REPORT_PATH.write_text(_SHARED_REPORT.to_markdown(), encoding="utf-8")
    print(f"\n测试报告已写入: {REPORT_PATH}")


@pytest.fixture
def check(report):
    """用例内执行步骤列表, 全部通过才让用例通过。"""

    def _run(steps):
        failed = report.run(steps)
        assert not failed, "; ".join(failed)

    return _run
