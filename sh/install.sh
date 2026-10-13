#!/bin/bash
# ==============================================================================
# Dankong 一键安装脚本
# 用法 (兼容 curl | sh):
#   curl -fsSL <install.sh 的 raw 地址> | sh
#   ROBOT=dog curl -fsSL <url> | sh          # 指定机型
#   GH_PROXY=https://ghproxy.com curl ... | sh  # 走镜像加速
#
# 流程: 下载最新 GitHub Release 产物 -> 解压到当前目录 -> 加执行权限
#       -> 生成 <robot>-real-config.yaml -> 询问串口写入 mavsdk_url
#       -> 打印启动命令
# ==============================================================================
set -e

REPO="HGGshiwo/dankong"

log() { echo "[install] $*"; }
die() { echo "[install] ❌ $*" >&2; exit 1; }

# 交互询问 (curl|sh 时 stdin 是脚本本身, 必须从 /dev/tty 读写;
# 兼容 dash: 提示用 printf 输出, read 不用 -p)
ask() {
    printf "%s" "$1" > /dev/tty
    read -r "$2" < /dev/tty
}

# --- 架构探测 ---
case "$(uname -m)" in
    x86_64)  ARCH="amd64" ;;
    aarch64) ARCH="arm64" ;;
    *) die "不支持的架构: $(uname -m)" ;;
esac

# --- 机型: 环境变量优先, 否则交互询问 (curl|sh 时 stdin 被占用, 从 /dev/tty 读) ---
ROBOT="${ROBOT:-}"
if [ -z "$ROBOT" ]; then
    ask "机器人类型 [drone/dog/car/go2] (默认 drone): " ROBOT || ROBOT=""
    ROBOT="${ROBOT:-drone}"
fi

ASSET="dankong-${ROBOT}-linux-${ARCH}.tar.gz"
log "目标: $ROBOT / linux-$ARCH -> $ASSET"

# --- 下载并解压到当前目录 ---
# 固定地址, 自动指向最新 release 的同名资产, 无需查询 API
URL="https://github.com/$REPO/releases/latest/download/$ASSET"
# 支持镜像前缀 (国内加速)
if [ -n "$GH_PROXY" ]; then
    URL="${GH_PROXY%/}/$URL"
fi

log "下载: $URL"
curl -fL --progress-bar -o "$ASSET" "$URL"
tar -xzf "$ASSET" -C .
rm -f "$ASSET"

# --- 可执行文件加权限 ---
chmod +x ./dankong
chmod +x ./bin/dankong_*_node 2>/dev/null || true

# --- 生成 real 配置并写入 mavsdk_url ---
if [ ! -f "./bin/config/${ROBOT}-config.yaml" ]; then
    die "找不到 bin/config/${ROBOT}-config.yaml"
fi
REAL_CONFIG="./bin/config/${ROBOT}-real-config.yaml"
cp "./bin/config/${ROBOT}-config.yaml" "$REAL_CONFIG"

# 串口等连接参数: curl|sh 时必须从 /dev/tty 读取
TTY_DEV=""
TTY_BAUD=""
ask "飞控串口设备 (如 /dev/ttyS5, 直接回车跳过): " TTY_DEV || TTY_DEV=""
if [ -n "$TTY_DEV" ]; then
    ask "波特率 (默认 115200): " TTY_BAUD || TTY_BAUD=""
    TTY_BAUD="${TTY_BAUD:-115200}"
    if echo "$TTY_DEV" | grep -q "://"; then
        MAVSDK_URL="$TTY_DEV"  # 已是完整 url (serial:///... tcp://...)
    else
        MAVSDK_URL="serial://${TTY_DEV}:${TTY_BAUD}"
    fi
else
    MAVSDK_URL="serial:///dev/ttyS5:115200"  # 默认值, 事后可改配置
fi
sed -i "s|^mavsdk_url:.*|mavsdk_url: \"$MAVSDK_URL\"|" "$REAL_CONFIG"
log "已写入 $REAL_CONFIG -> mavsdk_url: \"$MAVSDK_URL\""

INSTALL_DIR="$(pwd)"
echo ""
echo "================================================================================"
echo "🎉 部署完成! 目录: $INSTALL_DIR"
echo ""
echo "启动命令:"
echo "  cd $INSTALL_DIR && ./dankong --config config/${ROBOT}-real-config.yaml"
echo "================================================================================"
