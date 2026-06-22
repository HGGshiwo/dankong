#!/bin/bash

# ==============================================================================
# Dankong 项目独立打包部署脚本
# 功能: 编译项目 -> 提取依赖库 -> 拷贝配置文件 -> 生成启动脚本
# ==============================================================================

# 出现错误时立即停止脚本
set -e

# --- 1. 配置参数 ---
PROJECT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
BUILD_DIR="$PROJECT_DIR/build_standalone" # 使用独立的 build 文件夹避免污染 ROS 缓存
DEPLOY_DIR="$PROJECT_DIR/deploy_pkg"      # 最终的打包输出目录
EXEC_NAME="dankong_drone_node"            # 你的节点名字

echo "🚀 [1/5] 开始准备打包环境..."
rm -rf "$DEPLOY_DIR"
mkdir -p "$DEPLOY_DIR/bin"
mkdir -p "$DEPLOY_DIR/libs"

echo "⚙️  [2/5] 正在编译项目 (无 ROS 模式)..."
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"
# 强制关闭 ROS 并编译
cmake .. -DBUILD_WITH_ROS=OFF
make -j$(nproc)

echo "📦 [3/5] 提取可执行文件与静态资源..."
cd "$PROJECT_DIR"

# 查找刚编译好的可执行文件
EXEC_PATH=$(find "$BUILD_DIR" -name "$EXEC_NAME" -type f | head -n 1)
if [ -z "$EXEC_PATH" ]; then
    echo "❌ 错误：找不到可执行文件 $EXEC_NAME，编译可能失败。"
    exit 1
fi

# 拷贝二进制文件
cp "$EXEC_PATH" "$DEPLOY_DIR/bin/"

# 拷贝运行时依赖的资源目录（根据你 CMake 里的 target_copy_post_build）
if [ -d "$PROJECT_DIR/config" ]; then
    cp -r "$PROJECT_DIR/config" "$DEPLOY_DIR/bin/"
    echo "  -> 已拷贝 config 目录"
fi

if [ -d "$PROJECT_DIR/dk/frontend/dist" ]; then
    cp -r "$PROJECT_DIR/dk/frontend/dist" "$DEPLOY_DIR/bin/"
    echo "  -> 已拷贝前端 dist 目录"
fi

echo "🔍 [4/5] 扫描并拷贝动态依赖库 (.so)..."

# 使用 ldd 列出依赖，并设置黑名单，剔除操作系统核心底层库
ldd "$EXEC_PATH" | awk '{print $3}' | grep -v "^(" | grep "^/" | \
grep -vE "/libc\.so|/libm\.so|/libdl\.so|/libpthread\.so|/libresolv\.so|/librt\.so|/libgcc_s\.so|/libstdc\+\+\.so|/ld-linux" | \
while read -r lib_path; do
    cp "$lib_path" "$DEPLOY_DIR/libs/"
done

echo "  -> 动态依赖库提取完毕！(已安全跳过底层系统库)"

echo "📜 [5/5] 生成绿色版启动脚本..."
# 创建一个名为 run.sh 的启动脚本
cat << 'EOF' > "$DEPLOY_DIR/run.sh"
#!/bin/bash
# 获取当前脚本所在目录的绝对路径
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# 核心魔法：告诉系统优先从我们自带的 libs 文件夹里加载动态库（如 OpenCV, MAVSDK）
export LD_LIBRARY_PATH="$DIR/libs:$LD_LIBRARY_PATH"

# 进入 bin 目录，确保程序能正确读取到同级目录下的 config 和 dist 文件夹
cd "$DIR/bin"

echo "启动 Dankong 无 ROS 独立版..."
# 运行程序，并将后方传入的参数原样传递给程序
./dankong_drone_node "$@"
EOF

# 赋予启动脚本执行权限
chmod +x "$DEPLOY_DIR/run.sh"

echo "✅ 打包大功告成！"
echo "================================================================================"
echo "你的独立运行包已生成在: $DEPLOY_DIR"
echo "你可以直接把整个 'deploy_pkg' 文件夹拷贝到任何一台干净的机器上。"
echo "运行方式: 进入文件夹，执行 ./run.sh"
echo "================================================================================"