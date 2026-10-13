#!/usr/bin/env bash
# 一键测试脚本: 自动启动 SITL + dankong, 运行 pytest, 结束后清理所有子进程
# 用法:
#   ./pytest/run_test.sh                              # 跑全部用例
#   ./pytest/run_test.sh pytest/test_takeoff.py -v    # 参数透传给 pytest
set -uo pipefail

PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="$PKG_DIR/pytest/logs"
mkdir -p "$LOG_DIR"

SITL_PID=""
LAUNCH_PID=""

cleanup() {
    echo "[run_test] 清理进程..."
    if [[ -n "$LAUNCH_PID" ]]; then
        # roslaunch 需要 SIGINT 才能优雅关闭其子节点
        kill -INT "$LAUNCH_PID" 2>/dev/null
    fi
    if [[ -n "$SITL_PID" ]]; then
        # sim_vehicle.py 建立了进程组, 按组杀干净
        kill -TERM -- -"$SITL_PID" 2>/dev/null
    fi
    sleep 3
    # 兜底: 残留进程强杀
    [[ -n "$LAUNCH_PID" ]] && kill -9 "$LAUNCH_PID" 2>/dev/null
    [[ -n "$SITL_PID" ]] && kill -9 -- -"$SITL_PID" 2>/dev/null
    pkill -f "dankong_.*_node" 2>/dev/null
    echo "[run_test] 清理完成"
}
trap cleanup EXIT

echo "[run_test] 启动 SITL (日志: pytest/logs/sitl.log)"
setsid sim_vehicle.py -v Copter --no-mavproxy --no-rebuild \
    > "$LOG_DIR/sitl.log" 2>&1 &
SITL_PID=$!

echo "[run_test] 等待 SITL 就绪 (tcp:5760)..."
for i in $(seq 1 60); do
    if timeout 1 bash -c "</dev/tcp/127.0.0.1/5760" 2>/dev/null; then
        echo "[run_test] SITL 就绪"
        break
    fi
    if [[ $i -eq 60 ]]; then
        echo "[run_test] SITL 启动超时, 请查看 $LOG_DIR/sitl.log"
        exit 1
    fi
    sleep 2
done

echo "[run_test] 启动 dankong (日志: pytest/logs/dankong.log)"
roslaunch dankong dankong.launch > "$LOG_DIR/dankong.log" 2>&1 &
LAUNCH_PID=$!

# 就绪判定: GET /prearms 返回 status=="success"
echo "[run_test] 等待 dankong web 服务就绪 (127.0.0.1:8000, GET /prearms)..."
READY=0
for i in $(seq 1 90); do
    RES=$(curl -sf http://127.0.0.1:8000/prearms 2>/dev/null)
    if echo "$RES" | grep -q '"status": *"success"'; then
        READY=1
        break
    fi
    if [[ $i -eq 90 ]]; then
        echo "[run_test] dankong 启动超时, 请查看 $LOG_DIR/dankong.log"
        exit 1
    fi
    sleep 2
done
echo "[run_test] dankong 就绪 (GPS/EKF 收敛由用例内 PrearmCheck 步骤保证)"

echo "[run_test] 开始测试"
cd "$PKG_DIR"
python3 -m pytest pytest/ -v "$@"
RC=$?

echo "[run_test] 测试结束 (exit=$RC), 报告: pytest/report.md, 日志: $LOG_DIR/"
exit $RC
