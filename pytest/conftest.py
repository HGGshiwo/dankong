#!/usr/bin/env python
# coding=utf-8
"""pytest 公共 fixture: robot 客户端 + 报告输出 + 每用例重启飞控与 dankong。"""
import os
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent))
from steps import MissionReport, MissionRunner, RobotClient, wait_prearms_ready

REPORT_PATH = Path(__file__).parent / "report.md"
SITL_LOG = Path(__file__).parent / "logs" / "sitl.log"

# 报告跨用例累积 (每用例的 robot/WS 客户端全新, 但报告汇总是一份)
_SHARED_REPORT = MissionReport()

# 本 fixture 拉起的 SITL 进程 (按进程组整组杀, 连同 sim 二进制/MAVProxy
# 子进程; 只 pkill sim_vehicle.py 会留下孤儿仿真进程继续占用 5760)
_sitl_proc = None


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


def _stop_sitl():
    """停掉 SITL 并确认 5760 端口已释放。

    先按进程组杀自己拉起的实例; 再用模式兜底清理外部启动的实例
    (如 run_test.sh 起的), 覆盖包装脚本与仿真二进制两类进程名。
    """
    global _sitl_proc
    if _sitl_proc is not None:
        try:
            os.killpg(os.getpgid(_sitl_proc.pid), signal.SIGTERM)
        except (ProcessLookupError, PermissionError):
            pass
        _sitl_proc = None
    for pattern in (
        "sim_vehicle.py",
        "mavproxy",
        "arducopter",
        "ArduCopter",
        "autotest/sim.py",
    ):
        subprocess.run(["pkill", "-f", pattern], check=False)
    # 等端口真正释放, 避免新实例 bind 失败或连上旧飞控
    deadline = time.time() + 15
    while time.time() < deadline:
        try:
            with socket.create_connection(("127.0.0.1", 5760), timeout=1):
                time.sleep(1)  # 仍有连接, 旧实例还活着
        except OSError:
            return
    print("[警告] SITL 端口 5760 在 15s 内未释放, 可能存在残留实例")


@pytest.fixture(autouse=True)
def fresh_dankong(request):
    """每个用例开始前整组重启 SITL 飞控和 dankong 节点 (所有用例同一路径)。

    顺序保证 dankong 总在飞控就绪后才被 respawn (其 web 服务依赖飞控):
      1) 停 SITL (进程组杀 + pkill 兜底, 等 5760 释放);
      2) 启动 SITL, 等 tcp:5760 (飞控参数/航点/EKF home 全部复位);
      3) 飞控就绪后 kill dankong, roslaunch respawn 拉起全新节点
         (状态机/跟随冷却/参数缓存复位), 睁眼即有活飞控;
      4) 等 GET /prearms success (MAVSDK 连接 + EKF 收敛完成)。

    标记 @pytest.mark.no_restart 可跳过 (用于不依赖节点状态的纯计算用例)。
    """
    if request.node.get_closest_marker("no_restart"):
        yield
        return
    global _sitl_proc
    t0 = time.time()
    print("\n[fresh_dankong] 停旧 SITL...")
    _stop_sitl()
    SITL_LOG.parent.mkdir(parents=True, exist_ok=True)
    with open(SITL_LOG, "ab") as log:
        _sitl_proc = subprocess.Popen(
            ["sim_vehicle.py", "-v", "Copter", "--no-mavproxy", "--no-rebuild"],
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
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
    # 飞控就绪后再重启 dankong, 保证 respawn 出的节点直接连上活飞控
    print("[fresh_dankong] 重启 dankong 节点...")
    subprocess.run(["pkill", "-f", "dankong_.*_node"], check=False)
    print("[fresh_dankong] 等 /prearms 就绪...")
    wait_prearms_ready(180)
    print(f"\n[飞控+dankong 已重启并就绪, 耗时 {time.time() - t0:.0f}s]")
    yield


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
