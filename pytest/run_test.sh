#!/usr/bin/env bash
# 一键测试脚本: 设置环境变量, 运行 pytest, 结束后清理残留。
# SITL 飞控与 dankong roslaunch 的启动/每用例重启/停止全部由 pytest 的
# fresh_dankong fixture 负责 (conftest.py, 按需启动、用完即停)。
# 用法:
#   ./pytest/run_test.sh                              # 跑全部用例
#   ./pytest/run_test.sh pytest/test_takeoff.py -v    # 参数透传给 pytest
#   DK_MAVSDK_URL 等模块级参数由测试文件的 DANKONG_LAUNCH_ARGS 声明
set -uo pipefail

PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="$PKG_DIR/pytest/logs"
mkdir -p "$LOG_DIR"

cleanup() {
    echo "[run_test] 清理进程..."
    sleep 3
    # 兜底: 残留进程强杀 (SITL/dankong 由 conftest 按狗牌与确定性停止
    # 管理, 这里只清理意外残留)
    pkill -f "dankong_.*_node" 2>/dev/null
    pkill -f "dankong.launch" 2>/dev/null
    pkill -f "sim_vehicle.py" 2>/dev/null
    # 兜底: 杀掉测试用例可能遗留的 rostopic pub (孤儿发布者会在下次
    # 测试起飞时把飞机重新拽进跟随模式)
    pkill -f "rostopic pub.*location_vel" 2>/dev/null
    echo "[run_test] 清理完成"
}
trap cleanup EXIT

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

# dankong roslaunch 由 conftest 按需启动 (见 fresh_dankong fixture)

echo "[run_test] 开始测试"
cd "$PKG_DIR"
python3 -m pytest pytest/ -v "$@"
RC=$?

echo "[run_test] 测试结束 (exit=$RC), 报告: pytest/report.md, 日志: $LOG_DIR/"
exit $RC
