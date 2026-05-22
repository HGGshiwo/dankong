import os
import subprocess
import sys

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# ==========================================
# 0. 配置参数
# ==========================================
# C++ 编译生成的可执行文件路径 (根据你的系统修改)
CPP_EXEC = "/home/hgg/catkin_ws/devel/lib/dankong/test_tracker_sim"

is_omni = 0  # 0 为非全向, 1 为全向
auto_heading_dist = 1.0  # 盲区距离

# ==========================================
# 1. 终端输入 / 交互式获取目标点
# ==========================================
print("==========================================")
print("请提供目标点坐标 (X, Y) :")
print("  -> 方式 1: 直接在此输入坐标，用空格或逗号隔开 (如: 1.5 2.0 或 1.5,2.0)")
print("  -> 方式 2: 直接按【回车键】，弹出绘图窗口使用鼠标点击选择")
print("==========================================")

user_input = input("请输入坐标 (或直接回车): ").strip()

if user_input == "":
    # ---------------- 鼠标选点模式 ----------------
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_xlim(-3.0, 3.0)
    ax.set_ylim(-3.0, 3.0)
    ax.set_aspect("equal")
    ax.grid(True, linestyle="--")
    ax.set_title("Click anywhere to set Target Position (Direction)")

    # 画出原点
    ax.plot(0, 0, "go", markersize=10, label="Start (0,0)")
    ax.legend()

    print("\n等待输入... 请在弹出的窗口中鼠标点击一个位置作为目标点！")
    # 阻塞等待用户点击1次
    coords = plt.ginput(1, timeout=-1)
    plt.close(fig)  # 获取完关闭交互窗口

    if not coords:
        print("未选择点，程序退出。")
        sys.exit(0)

    target_x, target_y = coords[0]

else:
    # ---------------- 终端输入模式 ----------------
    try:
        # 兼容逗号或空格分隔 (将逗号替换为空格)
        clean_input = user_input.replace(",", " ")
        parts = clean_input.split()
        if len(parts) != 2:
            raise ValueError("必须输入两个数值")

        target_x = float(parts[0])
        target_y = float(parts[1])
    except ValueError as e:
        print(f"\n❌ 输入格式错误: {e}")
        print("正确示例: 1.5 2.0")
        sys.exit(1)

print(f"\n✅ 捕获目标点: X = {target_x:.2f}, Y = {target_y:.2f}")

# ==========================================
# 2. 调用 C++ 后端计算
# ==========================================
print("正在调用 C++ Tracker 引擎计算轨迹...")
# 构造输入字符串，对应 C++ 需要的 4 个参数
input_str = f"{is_omni}\n{auto_heading_dist}\n{target_x}\n{target_y}\n"

try:
    # 瞬间运行 C++，传入参数，捕获输出
    result = subprocess.run(
        [CPP_EXEC], input=input_str, text=True, capture_output=True, check=True
    )
    print("C++ 计算完成！")
except FileNotFoundError:
    print(f"找不到可执行文件 {CPP_EXEC}，请确认是否已编译且路径正确。")
    sys.exit(1)
except subprocess.CalledProcessError as e:
    print("C++ 程序运行报错:\n", e.stderr)
    sys.exit(1)

# ==========================================
# 3. 读取数据并生成动画
# ==========================================
if not os.path.exists("trajectory.csv"):
    print("未找到 trajectory.csv，仿真可能失败。")
    sys.exit(1)
df = pd.read_csv("trajectory.csv")
target_pos = (target_x, target_y)
# --- 数据自检，帮你排查 C++ 是否算对了 ---
print("\n--- 轨迹数据检查 ---")
print(f"总数据帧数: {len(df)}")
print(f"起点坐标: ({df['x'].iloc[0]:.2f}, {df['y'].iloc[0]:.2f})")
print(f"终点坐标: ({df['x'].iloc[-1]:.2f}, {df['y'].iloc[-1]:.2f})")
if np.isnan(df["x"].iloc[-1]):
    print("⚠️ 警告：轨迹中出现了 NaN，C++ 控制器可能除以 0 算崩了！")
elif abs(df["x"].iloc[-1]) < 0.01 and abs(df["y"].iloc[-1]) < 0.01:
    print("⚠️ 警告：无人机几乎没有移动！(是不是触发了控制器的停滞 Bug？)")
print("--------------------\n")

# 准备画布
fig, ax = plt.subplots(figsize=(8, 8))
max_bound = max(3.0, abs(target_x) + 1.5, abs(target_y) + 1.5)
ax.set_xlim(-max_bound, max_bound)
ax.set_ylim(-max_bound, max_bound)
ax.set_aspect("equal")
ax.grid(True, linestyle="--")
ax.set_title(f"Tracker Simulation\nTarget: ({target_x:.2f}, {target_y:.2f})")

# 画目标点和盲区
ax.plot(target_pos[0], target_pos[1], "r*", markersize=12, label="Target")
circle = plt.Circle(
    target_pos,
    auto_heading_dist,
    color="r",
    fill=False,
    linestyle=":",
    alpha=0.5,
    label=f"{auto_heading_dist}m Blind Zone",
)
ax.add_patch(circle)
ax.legend(loc="lower right")

# 初始化动态元素
(line,) = ax.plot([], [], "k-", alpha=0.5)
time_text = ax.text(
    0.05,
    0.90,
    "",
    transform=ax.transAxes,
    fontsize=12,
    bbox=dict(facecolor="white", alpha=0.8),
)

# 提前创建一个多边形并添加到画布，后续只更新它的坐标
dummy_points = [[0, 0], [0, 0], [0, 0]]
robot_patch = plt.Polygon(dummy_points, color="blue", alpha=0.8)
ax.add_patch(robot_patch)


def init():
    line.set_data([], [])
    robot_patch.set_xy(dummy_points)
    time_text.set_text("")
    return line, time_text, robot_patch


def update(frame):
    # 更新轨迹线
    line.set_data(df["x"][:frame], df["y"][:frame])

    # 获取当前状态
    x = df["x"][frame]
    y = df["y"][frame]
    yaw = df["yaw"][frame]

    # 计算新顶点，使用 set_xy 更新多边形
    L = max_bound * 0.05
    p1 = [x + L * np.cos(yaw), y + L * np.sin(yaw)]
    p2 = [x + 0.5 * L * np.cos(yaw + 2.5), y + 0.5 * L * np.sin(yaw + 2.5)]
    p3 = [x + 0.5 * L * np.cos(yaw - 2.5), y + 0.5 * L * np.sin(yaw - 2.5)]
    robot_patch.set_xy([p1, p2, p3])

    # 更新文字
    vx = df["vx"][frame]
    vw = df["vw"][frame]
    yaw_deg = np.rad2deg(yaw)
    time_text.set_text(
        f"Time: {df['t'][frame]:.2f}s\nSpeed X: {vx:.2f} m/s\nYaw rate: {vw:.2f} rad/s\nYaw: {yaw_deg:.1f}°"
    )

    return line, time_text, robot_patch


print("Generating Animation...")
# 跳帧播放：如果 50Hz 播放太慢，可以加上 step，比如 frames=range(0, len(df), 2)
ani = animation.FuncAnimation(
    fig, update, frames=len(df), init_func=init, blit=False, interval=20
)
plt.show()
