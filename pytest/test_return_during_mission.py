#!/usr/bin/env python
# coding=utf-8
"""回归用例: 航线执行中途带航点的 /return。

语义: /return 带航点时, 应先飞向该航点 (含高度调整), 再回返航点
(任务起点) 降落。

事故背景 (2026-11-09 实机): 航线执行中 (/stop_follow 续飞剩余航点,
检测限速 1m/s) 调用带航点的 /return, 状态机复用了旧 WaypointState
(wp_idx 沿用旧进度), /return 自带的航点被丢弃; 恰逢旧航线末点在
容差内, 1.5s 后误判"任务完成"而原地降落。

修复: 空中带航点的 return 构造全新 WaypointState (wp_list_/wp_idx 重建)。

断言:
  1. /return 后 15s 内不得落地 (复现旧 bug 会原地快速降落);
  2. 90s 内接近 /return 指定的航点 (xy<=6m) 且完成爬升 (z 高于航线高度);
  3. 最终降落在任务起点附近 (<=6m)。
"""
import time

from steps import (
    EnsureGround,
    PrearmCheck,
    Takeoff,
    gps_distance,
    gps_offset,
    http_post,
)

MISSION_ALT = 8.0
RETURN_WP_D_E = 20.0  # /return 航点: 当前位置再向东 20m
RETURN_WP_ALT = 15.0  # /return 航点高度 (高于航线, 验证爬升)
LAND_TOL = 6.0


def _pos_lon_lat_alt(robot):
    robot.wait_state_fresh()
    s = robot.snapshot()
    g = s.get("gps") or [s.get("lon"), s.get("lat"), s.get("rel_alt")]
    return g[0], g[1], g[2], s.get("pos_enu")


def test_return_with_waypoint_during_mission(check, robot):
    check([EnsureGround(), PrearmCheck(), Takeoff(alt=MISSION_ALT)])
    robot.wait_state_fresh()
    home = robot.snapshot()
    home_lon, home_lat = home["lon"], home["lat"]

    # 发一条向东的航线 (首点 [0,0,z] 为起飞占位, 空中整点抛弃)
    wps = [[0, 0, MISSION_ALT]] + [
        gps_offset(home_lon, home_lat, de, dn) + [MISSION_ALT]
        for de, dn in ((40.0, 0.0), (80.0, 0.0))
    ]
    res = http_post("/set_waypoint", data=dict(waypoint=wps, do_pland=False, speed=5))
    assert res.get("status") == "success", f"set_waypoint 被拒: {res}"

    # 等进入执行航点且离开起飞点 10m 以上
    deadline = time.time() + 90
    while time.time() < deadline:
        s = robot.snapshot()
        if (
            s.get("state") == "执行航点"
            and gps_distance(s["lon"], s["lat"], home_lon, home_lat) > 10.0
        ):
            break
        time.sleep(0.5)
    else:
        raise RuntimeError("等待进入执行航点(离home>10m)超时")

    # 构造 /return 航点: 当前位置再向东 20m, 高度 15 (高于航线, 需爬升)
    cur_lon, cur_lat, _, _ = _pos_lon_lat_alt(robot)
    rwp = gps_offset(cur_lon, cur_lat, RETURN_WP_D_E, 0.0) + [RETURN_WP_ALT]
    res = http_post("/return", data=dict(waypoint=[rwp], do_pland=False))
    assert res.get("status") == "success", f"return 被拒: {res}"
    t_return = time.time()

    # 断言1: 15s 内不得落地 (旧 bug: 复用旧任务, 数秒内原地降落)
    deadline = t_return + 15
    while time.time() < deadline:
        s = robot.snapshot()
        if s.get("state") == "地面状态":
            raise AssertionError(
                f"/return 后 {time.time()-t_return:.1f}s 即落地 "
                "(复现旧 bug: return 航点被丢弃, 复用旧任务误判完成)"
            )
        time.sleep(0.5)

    # 断言2: 90s 内接近 return 航点且完成爬升
    deadline = t_return + 90
    reached = False
    while time.time() < deadline:
        s = robot.snapshot()
        d = gps_distance(s["lon"], s["lat"], rwp[0], rwp[1])
        z = (s.get("pos_enu") or [0, 0, 0])[2]
        if d <= 6.0 and z > MISSION_ALT + 3.0:
            reached = True
            print(
                f"/return 航点已到达: xy偏差 {d:.1f}m, "
                f"z={z:.1f} (目标 {RETURN_WP_ALT}m, 航线 {MISSION_ALT}m)"
            )
            break
        if s.get("state") == "地面状态":
            raise AssertionError(
                f"未到达 return 航点即落地 (xy偏差 {d:.1f}m, z={z:.1f})"
            )
        time.sleep(0.5)
    assert reached, (
        f"90s 内未接近 /return 指定航点 (return 航点被忽略?): "
        f"最后 xy偏差 {d:.1f}m, z={z:.1f}"
    )

    # 断言3: 最终降落在任务起点附近 (RETURN 收尾)
    robot.wait_for_state("地面状态", 300)
    time.sleep(1)
    s = robot.snapshot()
    d_home = gps_distance(s["lon"], s["lat"], home_lon, home_lat)
    assert d_home <= LAND_TOL, f"未降落在任务起点附近: 偏差 {d_home:.1f}m"
    print(f"已降落在任务起点附近: 偏差 {d_home:.1f}m")
