#!/usr/bin/env python
# coding=utf-8
"""返航用例: 起飞后 /return, 校验回到起飞点并降落。"""
from steps import EnsureGround, PrearmCheck, RecordTakeoffPos, Return, Sleep, Takeoff


def test_return(check):
    check(
        [
            EnsureGround(),
            PrearmCheck(),
            RecordTakeoffPos(),
            Takeoff(alt=10),
            Sleep(3),
            Return(),
        ]
    )
