#!/bin/bash
# ==============================================================================
# Dankong 发布脚本 (由 deploy.sh 改造)
# 功能: 编译项目 -> 提取依赖库 -> 拷贝配置 -> 生成启动器 -> 压成 release 产物
# 产物: dist/dankong-<robot>-linux-<arch>.tar.gz
#
# 用法:
#   ./sh/release.sh                     # 默认 drone
#   ./sh/release.sh --robot dog         # 指定机型 (drone/dog/car/go2)
#
# 多机型/多架构发布: 在对应的目标架构机器上分别执行, 然后把所有
# .tar.gz 挂到同一个 GitHub Release 下。
# ==============================================================================
set -e

# --- 0. 参数解析 ---
ROBOT_TYPE="drone"
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --robot|-r)
            if [ -z "$2" ]; then
                echo "❌ 错误: --robot 参数需要指定一个类型 (drone/dog/car/go2)"
                exit 1
            fi
            ROBOT_TYPE="$2"
            shift 2
            ;;
        -h|--help)
            echo "用法: ./sh/release.sh [--robot <type>]"
            echo "  --robot <type>  机器人类型, 传入 CMake -DROBOT (默认 drone)"
            exit 0
            ;;
        *)
            echo "❌ 未知参数: $1"
            exit 1
            ;;
    esac
done

# --- 1. 路径与架构 ---
PROJECT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
BUILD_DIR="$PROJECT_DIR/build_standalone"  # 独立 build 目录, 不污染 ROS 缓存
DEPLOY_DIR="$PROJECT_DIR/deploy_pkg"
OUT_DIR="$PROJECT_DIR/dist"
EXEC_NAME="dankong_${ROBOT_TYPE}_node"

case "$(uname -m)" in
    x86_64)  ARCH="amd64" ;;
    aarch64) ARCH="arm64" ;;
    *) echo "❌ 不支持的架构: $(uname -m)"; exit 1 ;;
esac
TAR_NAME="dankong-${ROBOT_TYPE}-linux-${ARCH}.tar.gz"

echo "🚀 [1/6] 准备打包环境..."
echo "   -> 机器人类型: $ROBOT_TYPE"
echo "   -> 架构: $ARCH"
rm -rf "$DEPLOY_DIR"
mkdir -p "$DEPLOY_DIR/bin"
mkdir -p "$DEPLOY_DIR/libs"
mkdir -p "$OUT_DIR"

echo "⚙️  [2/6] 编译项目 (无 ROS 模式, Release)..."
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"
cmake .. \
    -DROS_SUPPORT=NONE \
    -DCMAKE_BUILD_TYPE=Release \
    -DROBOT="$ROBOT_TYPE" \
    -DFETCHCONTENT_UPDATES_DISCONNECTED=ON
make -j4
cd "$PROJECT_DIR"

echo "📦 [3/6] 提取可执行文件与静态资源..."
EXEC_PATH=$(find "$BUILD_DIR" -name "$EXEC_NAME" -type f | head -n 1)
if [ -z "$EXEC_PATH" ]; then
    echo "❌ 错误: 找不到可执行文件 $EXEC_NAME, 编译可能失败。"
    exit 1
fi
cp "$EXEC_PATH" "$DEPLOY_DIR/bin/"

if [ -d "$PROJECT_DIR/config" ]; then
    cp -r "$PROJECT_DIR/config" "$DEPLOY_DIR/bin/"
    echo "  -> 已拷贝 config 目录"
fi
if [ -d "$PROJECT_DIR/dk/frontend/dist" ]; then
    cp -r "$PROJECT_DIR/dk/frontend/dist" "$DEPLOY_DIR/bin/"
    echo "  -> 已拷贝前端 dist 目录"
fi

echo "🔍 [4/6] 扫描并拷贝动态依赖库 (.so)..."
# ldd 提取依赖, 剔除操作系统核心底层库
ldd "$EXEC_PATH" | awk '{print $3}' | grep -v "^(" | grep "^/" | \
grep -vE "/libc\.so|/libm\.so|/libdl\.so|/libpthread\.so|/libresolv\.so|/librt\.so|/libgcc_s\.so|/libstdc\+\+\.so|/ld-linux" | \
while read -r lib_path; do
    cp "$lib_path" "$DEPLOY_DIR/libs/"
done
echo "  -> 动态依赖库提取完毕!"

echo "📜 [5/6] 生成启动器 dankong..."
RUN_SCRIPT="$DEPLOY_DIR/dankong"
cat << EOF > "$RUN_SCRIPT"
#!/bin/bash
DIR="\$( cd "\$( dirname "\${BASH_SOURCE[0]}" )" && pwd )"
export LD_LIBRARY_PATH="\$DIR/libs:\$LD_LIBRARY_PATH"
cd "\$DIR/bin"
echo "启动 Dankong 无 ROS 独立版..."
./${EXEC_NAME} "\$@"
EOF
chmod +x "$RUN_SCRIPT"

echo "📦 [6/6] 打压 release 产物..."
TAR_PATH="$OUT_DIR/$TAR_NAME"
tar -czf "$TAR_PATH" -C "$DEPLOY_DIR" .

echo "================================================================================"
echo "🎉 发布产物构建完成:"
echo "   $(cd "$OUT_DIR" && pwd)/$TAR_NAME"
echo ""
echo "请手动创建 GitHub Release 并上传该文件:"
echo "  1. 打开 https://github.com/HGGshiwo/dankong/releases/new"
echo "  2. tag 建议使用: v\$(date +%Y.%m.%d) 格式"
echo "  3. 上传上述 .tar.gz 后发布 (其他机型/架构请在对应机器上重复执行本脚本,"
echo "     并把所有产物挂到同一个 release 下)"
echo ""
echo "目标机器一键安装: curl -fsSL <install.sh 的 raw 地址> | sh"
echo "================================================================================"
