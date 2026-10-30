#!/bin/bash
# ==============================================================================
# Dankong 一键安装脚本
# 用法 (兼容 curl | sh):
#   curl -fsSL <install.sh 的 raw 地址> | sh
#   ROBOT=dog curl -fsSL <url> | sh          # 指定机型
#   GH_PROXY=https://ghproxy.com curl ... | sh  # 走镜像加速
#
# 安装本地已下载的压缩包 (文件名需为 dankong-<robot>-linux-<arch>.tar.gz):
#   sh install.sh /path/to/dankong-dog-linux-arm64.tar.gz
#   PKG=/path/to/xxx.tar.gz sh install.sh
#
# 流程: 下载 (或使用本地包) 最新 GitHub Release 产物 -> 解压到 ./dankong/
#       -> 加执行权限 -> 生成 <robot>-real-config.yaml -> 询问串口写入 mavsdk_url
#       -> 打印启动命令
# 若 ./dankong/ 已存在, 会询问是否重装 (全部删除后重装), 否则跳过。
# ==============================================================================
set -e

REPO="HGGshiwo/dankong"
INSTALL_DIR="$PWD/dankong"

log() { echo "[install] $*"; }
die() { echo "[install] ❌ $*" >&2; exit 1; }

# 交互询问 (curl|sh 时 stdin 是脚本本身, 必须从 /dev/tty 读写;
# 兼容 dash: 提示用 printf 输出, read 不用 -p)
ask() {
    printf "%s" "$1" > /dev/tty
    read -r "$2" < /dev/tty
}

# --- 本地压缩包: 第一个参数优先, 其次 $PKG ---
PKG="${1:-${PKG:-}}"

# --- 架构探测 (本地包场景下文件名里已含架构, 不强制要求) ---
if [ -z "$PKG" ]; then
    case "$(uname -m)" in
        x86_64)  ARCH="amd64" ;;
        aarch64) ARCH="arm64" ;;
        *) die "不支持的架构: $(uname -m)" ;;
    esac
fi

if [ -n "$PKG" ]; then
    # --- 本地包: 从文件名解析机型 ---
    [ -f "$PKG" ] || die "找不到本地压缩包: $PKG"
    PKG="$(cd "$(dirname "$PKG")" && pwd)/$(basename "$PKG")"  # 转绝对路径
    case "$(basename "$PKG")" in
        dankong-*-linux-*.tar.gz) ;;
        *) die "文件名需为 dankong-<robot>-linux-<arch>.tar.gz: $(basename "$PKG")" ;;
    esac
    _base="$(basename "$PKG")"
    ROBOT="${_base#dankong-}"; ROBOT="${ROBOT%%-linux-*}"
    [ -n "$ROBOT" ] || die "无法从文件名解析机型: $_base"
    log "使用本地压缩包: $PKG (机型: $ROBOT)"
else
    # --- 机型: 环境变量优先, 否则交互询问 (curl|sh 时 stdin 被占用, 从 /dev/tty 读) ---
    ROBOT="${ROBOT:-}"
    if [ -z "$ROBOT" ]; then
        ask "机器人类型 [drone/dog/car/go2] (默认 drone): " ROBOT || ROBOT=""
        ROBOT="${ROBOT:-drone}"
    fi
fi

# --- 已安装检查: 询问重装或跳过 ---
if [ -x "$INSTALL_DIR/dankong" ]; then
    log "检测到已存在安装: $INSTALL_DIR"
    ans=""
    ask "是否重装? (会删除该目录全部内容) [y/N]: " ans || ans=""
    case "$ans" in
        y|Y|yes|YES)
            log "删除旧安装..."
            rm -rf "$INSTALL_DIR"
            ;;
        *)
            log "跳过安装, 保留现有安装: $INSTALL_DIR"
            log "启动命令: cd $INSTALL_DIR && ./dankong --config config/<robot>-real-config.yaml"
            exit 0
            ;;
    esac
fi
mkdir -p "$INSTALL_DIR"

# --- 下载或复制本地包, 解压到 ./dankong/ ---
if [ -z "$PKG" ]; then
    ASSET="dankong-${ROBOT}-linux-${ARCH}.tar.gz"
    log "目标: $ROBOT / linux-$ARCH -> $ASSET"
    # 固定地址, 自动指向最新 release 的同名资产, 无需查询 API
    URL="https://github.com/$REPO/releases/latest/download/$ASSET"
    # 支持镜像前缀 (国内加速)
    if [ -n "$GH_PROXY" ]; then
        URL="${GH_PROXY%/}/$URL"
    fi
    log "下载: $URL"
    curl -fL --progress-bar -o "$INSTALL_DIR/$ASSET" "$URL"
    PKG="$INSTALL_DIR/$ASSET"
fi
tar -xzf "$PKG" -C "$INSTALL_DIR"
[ -n "${ASSET:-}" ] && rm -f "$PKG"

# --- 可执行文件加权限 ---
chmod +x "$INSTALL_DIR/dankong"
chmod +x "$INSTALL_DIR"/bin/dankong_*_node 2>/dev/null || true

# --- 生成 real 配置并写入 mavsdk_url ---
if [ ! -f "$INSTALL_DIR/bin/config/${ROBOT}-config.yaml" ]; then
    die "找不到 bin/config/${ROBOT}-config.yaml"
fi
REAL_CONFIG="$INSTALL_DIR/bin/config/${ROBOT}-real-config.yaml"
cp "$INSTALL_DIR/bin/config/${ROBOT}-config.yaml" "$REAL_CONFIG"

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

echo ""
echo "================================================================================"
echo "🎉 部署完成! 目录: $INSTALL_DIR"
echo ""
echo "启动命令:"
echo "  cd $INSTALL_DIR && ./dankong --config config/${ROBOT}-real-config.yaml"
echo "================================================================================"
