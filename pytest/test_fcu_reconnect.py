#!/usr/bin/env python
# coding=utf-8
"""FCU 断联重连用例: 通过 TCP 代理间接连接飞控, 杀代理模拟断联,
重启代理模拟重连, 验证航点任务能否自动恢复。

架构:
  dankong --(mavsdk_url=tcp://127.0.0.1:20000)--> 本测试的 TCP 代理
        --> 127.0.0.1:5760 (SITL serial0 mavlink 主端口)

通过模块属性 DANKONG_LAUNCH_ARGS 声明 dankong 启动参数, 由 conftest 的
dankong_lifecycle 机制 (fresh_dankong) 负责以该参数拉起/停止 dankong
roslaunch —— 本文件不自管 dankong 进程。

杀代理时 dankong 的 MAVSDK 连接立刻断开 (FCU Connection Changed: false);
重启代理后 MAVSDK 的 TcpClientConnection 每秒自动重试重连。本用例断言:
重连后指令流恢复、飞机继续执行剩余航点并最终回悬停。

结果解读:
  - 通过  = MAVSDK 指令流自愈成立, 任务自动恢复;
  - 失败  = 复现"重连后不自愈"的冻结, pos 快照即为证据
            (实机上对应 FCU 失控保护切 LAND/掉 GUIDED, 指令被飞控丢弃)。

用法:
  ./pytest/run_test.sh pytest/test_fcu_reconnect.py -v
"""
import socket
import time

from steps import EnsureGround, PrearmCheck, Takeoff, http_post
from tcp_proxy import TcpProxy

import pytest

# 声明后由 conftest 的 dankong_lifecycle 机制按此参数启动 dankong
DANKONG_LAUNCH_ARGS = ["mavsdk_url:=tcp://127.0.0.1:20000"]

PROXY_PORT = 20000  # 代理监听端口, 避开 ArduPilot SITL 的 serial 端口段 (5760~5765)
# 转发目标用 SITL serial0 主端口 5760: --no-mavproxy 下无人消费, 代理可独占。
# SITL 的额外端口 5762/5763 默认协议未启用, 只 listen 不 accept, 不可作转发目标
SITL_PORT = 5760

_proxy = None


# module 级: 保证代理在 fresh_dankong 拉起 dankong 之前就监听就绪,
# 否则 dankong 首次连接 20000 被拒会 5s 超时 abort 并陷入 respawn 循环
@pytest.fixture(scope="module", autouse=True)
def fcu_proxy():
    """启动代理; 杀/重启代理由用例自行控制。"""
    global _proxy
    _proxy = TcpProxy(PROXY_PORT, SITL_PORT)
    _proxy.start()
    yield
    _proxy.stop()


def _wait_moving(robot, timeout=10, min_disp=0.5):
    """确认飞机在移动: 采样窗口内位移超过 min_disp 米。"""
    p1 = robot.get_pos()
    deadline = time.time() + timeout
    while time.time() < deadline:
        time.sleep(1.0)
        p2 = robot.get_pos()
        if p1 and p2:
            disp = ((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2) ** 0.5
            if disp > min_disp:
                return disp
        p1 = p2 or p1
    raise TimeoutError(f"飞机未移动 ({min_disp}m/{timeout}s 内)")


def test_fcu_reconnect_resumes_mission(check, robot):
    """断联 3s 后重连: 航点任务应自动恢复执行并最终回悬停。"""
    check([EnsureGround(), PrearmCheck(), Takeoff(alt=10)])

    res = http_post(
        "/set_waypoint",
        data=dict(waypoint=[[30.0, 0.0, 10.0], [60.0, 0.0, 10.0]], local=True),
    )
    assert res.get("status") == "success", f"set_waypoint 被拒: {res}"

    # 进入执行航点并确认在移动
    deadline = time.time() + 60
    while time.time() < deadline:
        if robot.get_state_name() == "执行航点":
            break
        time.sleep(0.5)
    else:
        raise TimeoutError("等待进入执行航点超时")
    _wait_moving(robot, timeout=10, min_disp=0.5)

    # --- 杀代理模拟 FCU 断联, 再重启模拟重连 ---
    print("杀代理, 模拟 FCU 断联...")
    _proxy.stop()
    time.sleep(3.0)
    print("重启代理, 等待 MAVSDK 自动重连...")
    _proxy.restart()
    time.sleep(2.0)

    # --- 恢复断言: 继续移动 + 完成任务回悬停 ---
    deadline = time.time() + 180
    p_ref = robot.get_pos()
    resumed = False
    while time.time() < deadline:
        time.sleep(2.0)
        p = robot.get_pos()
        if p and p_ref:
            disp = ((p[0] - p_ref[0]) ** 2 + (p[1] - p_ref[1]) ** 2) ** 0.5
            if disp > 2.0:
                resumed = True
                print(f"任务已恢复执行 (距断联时位置 {disp:.1f}m)")
                break
        p_ref = p or p_ref
    assert resumed, (
        f"重连后 180s 内任务未恢复执行, 飞机冻结在 {robot.get_pos()}, "
        f"状态: {robot.get_state_name()}"
    )
    robot.wait_for_state("悬停状态", 300)
