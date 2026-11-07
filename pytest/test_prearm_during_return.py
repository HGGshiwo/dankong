#!/usr/bin/env python
# coding=utf-8
"""回归用例: 返航降落期间轮询 /prearms 不得打断降落。

背景: run_prearm_checks 曾无条件先切 GUIDED (保证"GUIDED 下能否解锁"
判定准确), 上位机飞行中轮询 /prearms 会把 LAND 中的飞机拽出降落,
叠加 tracker 残留 POSITION setpoint 使其重新爬回返航高度悬停。
修复后: 空中 (armed/in_air) 跳过切模式, LandState 进场清空 tracker 指令流。

本用例在返航+降落全程以 1Hz 轮询 /prearms, 断言:
  - 核心不变量: WS mode 一旦进入 LAND, 回到地面状态前不得再出现 GUIDED;
  - 降落期间 /prearms 至少成功响应 3 次 (轮询窗口确实覆盖降落);
  - 轮询全程无超时/异常 (顺带覆盖 HTTP 响应性);
  - 落点精度校验沿用 Return.check。
"""
import threading
import time

from steps import (
    EnsureGround,
    PrearmCheck,
    RecordTakeoffPos,
    Return,
    Sleep,
    Takeoff,
    http_get,
)


class PrearmPoller:
    """后台线程: 1Hz 轮询 /prearms, 同时采样 WS mode 时间线。"""

    def __init__(self, robot, interval=1.0):
        self.robot = robot
        self.interval = interval
        self._stop = threading.Event()
        self._thread = None
        self.modes = []  # [(t, mode)]
        self.prearm_ok = 0
        self.prearm_fail = 0
        self.errors = []

    def start(self):
        self._thread = threading.Thread(target=self._poll, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=5)

    def _poll(self):
        while not self._stop.is_set():
            try:
                res = http_get("/prearms")
                if res.get("status") == "success":
                    self.prearm_ok += 1
                else:
                    self.prearm_fail += 1
                    self.errors.append(f"/prearms 响应异常: {res}")
            except Exception as e:  # noqa: BLE001
                self.errors.append(f"/prearms 请求失败: {e}")
                self.prearm_fail += 1
            self.modes.append((time.time(), self.robot.snapshot().get("mode")))
            time.sleep(self.interval)

    # ---- 断言数据 ----
    def land_then_guided_again(self):
        """mode 时间线中, 进入 LAND 之后、(测试结束前) 是否重现 GUIDED。"""
        seen_land = False
        for _, mode in self.modes:
            if mode == "LAND":
                seen_land = True
            elif seen_land and mode == "GUIDED":
                return True
        return False


def test_prearm_during_return(check, robot):
    poller = PrearmPoller(robot)
    try:
        check(
            [
                EnsureGround(),
                PrearmCheck(),
                RecordTakeoffPos(),
                Takeoff(alt=10),
                Sleep(3),
            ]
        )
        # 轮询窗口覆盖整个返航+降落过程
        poller.start()
        check([Return()])
    finally:
        poller.stop()

    # --- 核心不变量: 进入 LAND 后不得再出现 GUIDED ---
    assert not poller.land_then_guided_again(), (
        f"降落中 mode 被切回 GUIDED (复现 /prearms 打断降落的 bug), "
        f"mode 时间线: {poller.modes}"
    )

    # --- 轮询窗口确实覆盖了降落, 且接口全程可响应 ---
    assert poller.prearm_ok >= 3, (
        f"降落期间 /prearms 成功响应仅 {poller.prearm_ok} 次 (<3), "
        f"轮询窗口未覆盖或接口异常: {poller.errors}"
    )
    assert poller.prearm_fail == 0, f"/prearms 轮询存在失败: {poller.errors}"
