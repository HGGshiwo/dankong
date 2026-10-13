#!/usr/bin/env python
# coding=utf-8
"""pytest 公共 fixture: robot 客户端 + 报告输出。"""
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent))
from steps import MissionRunner, RobotClient, wait_prearms_ready

REPORT_PATH = Path(__file__).parent / "report.md"


@pytest.fixture(scope="session")
def robot():
    wait_prearms_ready()
    client = RobotClient()
    client.wait_pos_ready(120)
    client.wait_for_state("地面状态", 120)
    client.takeoff_pos = (0.0, 0.0)  # 由 RecordTakeoffPos 步骤刷新
    yield client
    client.close()


@pytest.fixture(scope="session")
def report(robot):
    runner = MissionRunner(robot)
    yield runner
    REPORT_PATH.write_text(runner.report.to_markdown(), encoding="utf-8")
    print(f"\n测试报告已写入: {REPORT_PATH}")


@pytest.fixture
def check(report):
    """用例内执行步骤列表, 全部通过才让用例通过。"""

    def _run(steps):
        failed = report.run(steps)
        assert not failed, "; ".join(failed)

    return _run
