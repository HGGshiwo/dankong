#!/usr/bin/env python
# coding=utf-8
"""降落用例: 起飞后降落, 校验落地停稳。"""
from steps import EnsureGround, Land, PrearmCheck, Sleep, Takeoff


def test_land(check):
    check(
        [
            EnsureGround(),
            PrearmCheck(),
            Takeoff(alt=8),
            Sleep(3),
            Land(),
        ]
    )
