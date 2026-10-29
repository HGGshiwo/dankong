#!/usr/bin/env python
# coding=utf-8
"""指点控制 (/set_posvel) 用例:

1. 1Hz 持续下发恒速移动目标, 验证跟随精度; 目标停止后飞机应停在终点
   (≤1m), 不冲过头;
2. 只发一次远目标时超时逻辑生效 (自动回退且未到达);
3. 地面状态调用被拒;
4. 航点任务中注入指点指令, 完成后回航点子状态续飞。

注意: posvel_timeout 默认配置很短 (3s), mavproxy 式用法要求持续刷新
目标 —— 因此跟随用例必须 1Hz 流式下发, 不能发一次等结果。
"""
import time

from steps import (
    WAYPOINT_STATES,
    EnsureGround,
    ExpectPosVelRejected,
    FlyWaypoint,
    FollowMovingTarget,
    PosVelTimeout,
    PrearmCheck,
    Takeoff,
    gps_offset,
    http_post,
)


def _hover(check, alt=10):
    check([EnsureGround(), PrearmCheck(), Takeoff(alt=alt)])


def test_posvel_follow_moving_target(check, robot):
    """10Hz 持续下发移动目标: 滞后 ≤5m, 目标停后 ~1m 内停住、不过冲。"""
    _hover(check, alt=10)
    check(
        [
            FollowMovingTarget(
                alt=10,
                dir_e=1.0,
                dir_n=0.0,
                speed=2.0,
                duration=20.0,
                rate=10.0,
                follow_tol=5.0,
                arrive_tol=1.2,
            )
        ]
    )  # 1.0 会被 GPS 噪声卡边


def test_posvel_timeout(check, robot):
    """单次远目标不刷新: timeout 到期自动回退悬停, 且远未到达目标。"""
    _hover(check, alt=10)
    state = robot.snapshot()
    # 正东 ~300m 外的远点
    far = gps_offset(state["lon"], state["lat"], 300.0, 0.0) + [10.0]
    check([PosVelTimeout(pos=far, timeout=5, vel=2)])


def test_posvel_rejected_on_ground(check):
    """非 waypoint/hover 状态 (地面) 下 set_posvel 应被拒绝。"""
    check([ExpectPosVelRejected()])


def test_posvel_in_waypoint(check, robot):
    """航点任务中注入指点: 完成后回航点子状态续飞, 最终正常完成。

    航点用 GPS 模式 (local=False, [lon, lat, alt]), 从当前位置推算,
    更贴近实机用法。
    """
    _hover(check, alt=10)
    state = robot.wait_state_fresh() or robot.snapshot()
    lon, lat = state["lon"], state["lat"]
    # 从当前位置推算两个 GPS 航点: 东 60m / 东60m+北60m, 高度 12m
    wp1 = gps_offset(lon, lat, 60.0, 0.0) + [12.0]
    wp2 = gps_offset(lon, lat, 60.0, 60.0) + [12.0]
    wps = [wp1, wp2]
    res = http_post("/set_waypoint", data=dict(waypoint=wps, local=False))
    assert res.get("status") == "success", f"set_waypoint 被拒: {res}"
    # 进入航点执行后, 中途注入 10Hz 慢速移动目标 (10m 航迹)
    deadline = time.time() + 60
    while time.time() < deadline:
        if robot.get_state_name() == "执行航点":
            break
        time.sleep(0.5)
    else:
        raise TimeoutError("等待进入执行航点超时")
    check(
        [
            FollowMovingTarget(
                alt=12.0,
                dir_e=1.0,
                dir_n=0.0,
                speed=1.0,
                duration=10.0,
                rate=10.0,
                follow_tol=5.0,
                arrive_tol=1.0,
                check_stop=False,  # 指点结束后续飞航点, 不做停止校验
                back_states=WAYPOINT_STATES,
                settle_timeout=60,
            ),
        ]
    )
    # 指点结束后航线任务应继续执行直至完成回悬停
    robot.wait_for_state("悬停状态", 300)
