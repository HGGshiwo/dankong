#!/usr/bin/env python
# coding=utf-8
"""起飞相关用例: 普通起飞 + 起飞中重入起飞(优雅拒绝)。"""
from steps import EnsureGround, Land, PrearmCheck, Sleep, Takeoff, TakeoffReenter


def test_takeoff(check):
    """普通起飞到 8m 并在目标高度悬停。"""
    check(
        [
            EnsureGround(),
            PrearmCheck(),
            Takeoff(alt=8),
        ]
    )
    # 清理: 降落到地面, 不影响后续用例
    check([Sleep(2), Land()])


def test_takeoff_reenter(check):
    """起飞中再次触发起飞: 空中 NAV_TAKEOFF 被飞控拒绝(合理行为),
    验证系统优雅拒绝且飞机安全过渡到悬停。"""
    check(
        [
            EnsureGround(),
            PrearmCheck(),
            TakeoffReenter(first_alt=5, reenter_alt=12),
        ]
    )
    # 清理: 降落到地面
    check([Sleep(2), Land()])
