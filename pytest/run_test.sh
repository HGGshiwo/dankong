#!/usr/bin/env bash
# 一键测试脚本: 启动 dankong (roslaunch, 供 respawn), 运行 pytest, 结束后清理。
# SITL 飞控的启动/每用例重启由 pytest 的 fresh_dankong fixture 负责
# (conftest.py), 这里不再启动 SITL, 避免会话开头"起一次又杀一次"。
# 用法:
#   ./pytest/run_test.sh                              # 跑全部用例
#   ./pytest/run_test.sh pytest/test_takeoff.py -v    # 参数透传给 pytest
set -uo pipefail

PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="$PKG_DIR/pytest/logs"
mkdir -p "$LOG_DIR"

LAUNCH_PID=""

cleanup() {
    echo "[run_test] 清理进程..."
    if [[ -n "$LAUNCH_PID" ]]; then
        # roslaunch 需要 SIGINT 才能优雅关闭其子节点
        kill -INT "$LAUNCH_PID" 2>/dev/null
    fi
    sleep 3
    # 兜底: 残留进程强杀 (SITL 由 conftest 按进程组管理, 这里只兜底)
    [[ -n "$LAUNCH_PID" ]] && kill -9 "$LAUNCH_PID" 2>/dev/null
    pkill -f "dankong_.*_node" 2>/dev/null
    pkill -f "sim_vehicle.py" 2>/dev/null
    # 兜底: 杀掉测试用例可能遗留的 rostopic pub (孤儿发布者会在下次
    # 测试起飞时把飞机重新拽进跟随模式)
    pkill -f "rostopic pub.*location_vel" 2>/dev/null
    echo "[run_test] 清理完成"
}
trap cleanup EXIT

# 启动 dankong (roslaunch, respawn 供 pytest 每用例重启)。
# SITL 飞控由 conftest.py 的 fresh_dankong fixture 在每个用例前重启,
# dankong 总在飞控就绪后才被 (re)spawn。
echo "[run_test] 启动 dankong (日志: pytest/logs/dankong.log)"

# 测试用跟随冷却时间: 脚本内就地取值, 外层未设置时默认 2s (缩短冷却等待);
# 同时给到 dankong 节点 (C++ 侧读环境变量) 和 pytest (计算冷却等待时长)。
# 非法值 (非数字或 <1, 会使冷却失效导致冷却断言不可能通过) 回退为 2
DK_FOLLOW_COOL_TIME="${DK_FOLLOW_COOL_TIME:-2}"
if ! python3 -c "import sys; sys.exit(0 if float('${DK_FOLLOW_COOL_TIME}') >= 1 else 1)" 2>/dev/null; then
    echo "[run_test] 警告: DK_FOLLOW_COOL_TIME='${DK_FOLLOW_COOL_TIME}' 非法(<1), 回退为 2"
    DK_FOLLOW_COOL_TIME=2
fi
export DK_FOLLOW_COOL_TIME
echo "[run_test] dankong DK_FOLLOW_COOL_TIME=${DK_FOLLOW_COOL_TIME}s"

roslaunch dankong dankong.launch > "$LOG_DIR/dankong.log" 2>&1 &
LAUNCH_PID=$!

# 不在这里等 dankong web 就绪: 飞控 (SITL) 尚未启动, web 服务不会起。
# 每个用例开始前 fresh_dankong fixture 会先启动 SITL 并等
# GET /prearms success (见 conftest.py), 那是唯一的就绪判定点。

echo "[run_test] 开始测试"
cd "$PKG_DIR"
python3 -m pytest pytest/ -v "$@"
RC=$?

echo "[run_test] 测试结束 (exit=$RC), 报告: pytest/report.md, 日志: $LOG_DIR/"
exit $RC
