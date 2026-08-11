#!/bin/bash

# ==============================================================================
# Dankong 项目独立打包部署脚本
# 功能: 编译项目 -> 提取依赖库 -> 拷贝配置文件 -> 生成启动脚本 -> 配置环境变量
# ==============================================================================

# 出现错误时立即停止脚本
set -e

# --- 0. 默认参数与命令行解析 ---
BUILD_TYPE="Release"
ROBOT_TYPE="DEFAULT"
ADD_TO_BASHRC=0

# 解析命令行参数
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --debug) 
            BUILD_TYPE="Debug"
            shift 
            ;;
        --robot|-r) 
            if [ -z "$2" ]; then
                echo "❌ 错误: --robot 参数需要指定一个类型 (例如: --robot Iris)"
                exit 1
            fi
            ROBOT_TYPE="$2"
            shift 2 
            ;;
        --add-bashrc) 
            ADD_TO_BASHRC=1
            shift 
            ;;
        -h|--help)
            echo "用法: ./deploy.sh [选项]"
            echo "选项:"
            echo "  --debug          使用 Debug 模式编译 (默认 Release)"
            echo "  --robot <type>   设置机器人类型，将传入 CMake 的 -DROBOT 选项"
            echo "  --add-bashrc     将生成的打包目录加入 ~/.bashrc 的 PATH 中"
            exit 0
            ;;
        *) 
            echo "❌ 未知参数: $1"
            exit 1 
            ;;
    esac
done

# --- 1. 配置基础路径 ---
PROJECT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
BUILD_DIR="$PROJECT_DIR/build_standalone" # 使用独立的 build 文件夹避免污染 ROS 缓存
DEPLOY_DIR="$PROJECT_DIR/deploy_pkg"      # 最终的打包输出目录
EXEC_NAME="dankong_drone_node"            # 你的节点名字

echo "🚀 [1/6] 开始准备打包环境..."
echo "   -> 编译模式: $BUILD_TYPE"
echo "   -> 机器人类型: $ROBOT_TYPE"
rm -rf "$DEPLOY_DIR"
mkdir -p "$DEPLOY_DIR/bin"
mkdir -p "$DEPLOY_DIR/libs"

echo "⚙️  [2/6] 正在编译项目 (无 ROS 模式)..."
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# 将解析出的参数传递给 CMake
cmake .. \
    -DBUILD_WITH_ROS=OFF \
    -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
    -DROBOT="$ROBOT_TYPE"

make -j4

echo "📦 [3/6] 提取可执行文件与静态资源..."
cd "$PROJECT_DIR"

# 查找刚编译好的可执行文件
EXEC_PATH=$(find "$BUILD_DIR" -name "$EXEC_NAME" -type f | head -n 1)
if [ -z "$EXEC_PATH" ]; then
    echo "❌ 错误：找不到可执行文件 $EXEC_NAME，编译可能失败。"
    exit 1
fi

# 拷贝二进制文件
cp "$EXEC_PATH" "$DEPLOY_DIR/bin/"

# 拷贝运行时依赖的资源目录
if [ -d "$PROJECT_DIR/config" ]; then
    cp -r "$PROJECT_DIR/config" "$DEPLOY_DIR/bin/"
    echo "  -> 已拷贝 config 目录"
fi

if [ -d "$PROJECT_DIR/dk/frontend/dist" ]; then
    cp -r "$PROJECT_DIR/dk/frontend/dist" "$DEPLOY_DIR/bin/"
    echo "  -> 已拷贝前端 dist 目录"
fi

echo "🔍 [4/6] 扫描并拷贝动态依赖库 (.so)..."
# 使用 ldd 列出依赖，并设置黑名单，剔除操作系统核心底层库
ldd "$EXEC_PATH" | awk '{print $3}' | grep -v "^(" | grep "^/" | \
grep -vE "/libc\.so|/libm\.so|/libdl\.so|/libpthread\.so|/libresolv\.so|/librt\.so|/libgcc_s\.so|/libstdc\+\+\.so|/ld-linux" | \
while read -r lib_path; do
    cp "$lib_path" "$DEPLOY_DIR/libs/"
done
echo "  -> 动态依赖库提取完毕！(已安全跳过底层系统库)"

echo "📜 [5/6] 生成绿色版启动脚本..."
# 将 run.sh 改名为 dankong，防止加入 PATH 后与其他脚本冲突
RUN_SCRIPT="$DEPLOY_DIR/dankong"
cat << 'EOF' > "$RUN_SCRIPT"
#!/bin/bash
# 获取当前脚本所在目录的绝对路径
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# 核心魔法：告诉系统优先从我们自带的 libs 文件夹里加载动态库
export LD_LIBRARY_PATH="$DIR/libs:$LD_LIBRARY_PATH"

# 进入 bin 目录，确保程序能正确读取到同级目录下的 config 和 dist 文件夹
cd "$DIR/bin"

echo "启动 Dankong 无 ROS 独立版..."
# 运行程序，并将后方传入的参数原样传递给程序
./dankong_drone_node "$@"
EOF

# 赋予启动脚本执行权限
chmod +x "$RUN_SCRIPT"

# --- 6. 环境变量注入 ---
if [ "$ADD_TO_BASHRC" -eq 1 ]; then
    echo "🔧 [6/6] 正在配置环境变量 (~/.bashrc)..."
    BASHRC_FILE="$HOME/.bashrc"
    # 使用精确的字符串防止重复追加
    EXPORT_LINE="export PATH=\"\$PATH:$DEPLOY_DIR\""
    
    if ! grep -Fq "$EXPORT_LINE" "$BASHRC_FILE"; then
        echo "" >> "$BASHRC_FILE"
        echo "# Dankong Standalone Environment" >> "$BASHRC_FILE"
        echo "$EXPORT_LINE" >> "$BASHRC_FILE"
        echo "  -> ✅ 已成功将 $DEPLOY_DIR 添加到 PATH"
        echo "  -> 💡 请在终端执行 'source ~/.bashrc' 使其立即生效！"
    else
        echo "  -> ℹ️ 环境变量已存在，跳过添加。"
    fi
else
    echo "🔧 [6/6] 跳过配置环境变量 (如需配置请加上 --add-bashrc 参数)"
fi

echo "================================================================================"
echo "🎉 打包大功告成！"
echo "你的独立运行包已生成在: $DEPLOY_DIR"
if [ "$ADD_TO_BASHRC" -eq 1 ]; then
    echo "运行方式: 刷新终端后，在任意位置输入 'dankong' 即可启动！"
else
    echo "运行方式: 进入文件夹，执行 ./dankong"
fi
echo "================================================================================"