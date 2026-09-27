#!/data/data/com.termux/files/usr/bin/bash
# ============================================================================
# Android 手机一键安装 (在 Termux 里执行，不要在 proot 里)
#
#   curl -fsSL https://raw.githubusercontent.com/doghelWang/agv_sim/main/deploy/android/install.sh -o install.sh
#   bash install.sh --repo https://github.com/doghelWang/agv_sim.git [选项]
#
# 选项:
#   --repo URL         git 仓库 (默认 https://github.com/doghelWang/agv_sim.git)
#   --branch main      分支
#   --name phone1      节点名 (平台上显示)
#   --distro ubuntu    proot 容器名 (已存在且是 Ubuntu 22.04 时直接复用)
#   --mirror tuna      tuna=清华镜像 (默认) | official
#   --hub URL          接入已有主平台 (例如 http://192.168.1.10:8082)；不填则手机自己当平台
#   --token TOKEN      主平台集群令牌 (不填且指定了 --hub 时会提示输入，不回显)
#   --no-start         装完不启动
# 步骤: Termux 软件包 → proot Ubuntu 22.04 → ROS 2 Humble/Nav2/MuJoCo (proot_setup.sh) → 拉取代码到 /opt/agv
#       → Termux 侧脚本与开机自启 → 写 ~/.agv.env → 启动
# ============================================================================
set -e
REPO=""; BRANCH=""; NAME=""; DISTRO=""; MIRROR="tuna"; HUB=""; TOKEN=""; START=1
while [ $# -gt 0 ]; do
    case "$1" in
        --repo) REPO="$2"; shift 2;;   --branch) BRANCH="$2"; shift 2;;  --name) NAME="$2"; shift 2;;
        --distro) DISTRO="$2"; shift 2;; --mirror) MIRROR="$2"; shift 2;; --hub) HUB="$2"; shift 2;;
        --token) TOKEN="$2"; shift 2;;   --no-start) START=0; shift;;
        -h|--help) sed -n 2,20p "$0"; exit 0;;
        *) echo "未知参数 $1"; exit 1;;
    esac
done
[ -n "$PREFIX" ] && [ -d /data/data/com.termux ] || { echo "[错误] 请在 Termux 里运行"; exit 1; }
[ -f ~/.agv.env ] && . ~/.agv.env
REPO="${REPO:-${AGV_GIT_REMOTE:-https://github.com/doghelWang/agv_sim.git}}"; BRANCH="${BRANCH:-${AGV_GIT_BRANCH:-main}}"
NAME="${NAME:-${AGENT_NAME:-$(getprop ro.product.device 2>/dev/null || echo phone)}}"
DISTRO="${DISTRO:-${AGV_DISTRO:-ubuntu}}"
[ -n "$REPO" ] || { echo "[错误] 需要 --repo https://github.com/doghelWang/agv_sim.git"; exit 1; }
if [ -n "$HUB" ] && [ -z "$TOKEN" ]; then
    read -r -s -p "主平台集群令牌 (主平台上 ~/.agv-hub/cluster_token): " TOKEN; echo
fi
say() { echo -e "\033[1;36m[install]\033[0m $*"; }

# ---- 1. Termux 软件包
say "1/6 Termux 软件包"
yes | pkg update -y >/dev/null 2>&1 || true
pkg install -y proot-distro python git curl openssh termux-api android-tools net-tools >/dev/null
PDV=$(proot-distro help 2>&1 | grep -oE "version '[0-9.]+'" | grep -oE "[0-9.]+" | head -1)
say "   proot-distro ${PDV:-?}"

# ---- 2. proot Ubuntu 22.04
say "2/6 proot 容器 $DISTRO (Ubuntu 22.04)"
ROOT5="$PREFIX/var/lib/proot-distro/containers/$DISTRO/rootfs"; ROOT4="$PREFIX/var/lib/proot-distro/installed-rootfs/$DISTRO"
if [ ! -d "$ROOT5" ] && [ ! -d "$ROOT4" ]; then
    if proot-distro install --help 2>&1 | grep -q "IMAGE:TAG"; then       # proot-distro ≥ 5: 按 Docker 镜像安装
        proot-distro install -n "$DISTRO" ubuntu:22.04
    else                                                                  # 旧版: 按发行版别名，装完检查版本
        proot-distro install --override-alias "$DISTRO" ubuntu
    fi
fi
ROOTFS="$ROOT5"; [ -d "$ROOTFS" ] || ROOTFS="$ROOT4"
grep -q 'VERSION_ID="22.04"' "$ROOTFS/etc/os-release" || {
    echo "[错误] 容器 $DISTRO 不是 Ubuntu 22.04 ($(grep PRETTY "$ROOTFS/etc/os-release"))。"
    echo "       升级 proot-distro (pkg upgrade proot-distro) 后用 --distro 换个名字重装"; exit 1; }

# ---- 3. 拉取代码 (先放一份到容器内 /opt/agv-git，里面有 proot_setup.sh)
say "3/6 拉取代码 $REPO ($BRANCH)"
proot-distro login "$DISTRO" -- bash -c "command -v git >/dev/null || (apt-get update -q && apt-get install -y -q git ca-certificates >/dev/null)"
if [ ! -d "$ROOTFS/opt/agv-git/.git" ]; then
    proot-distro login "$DISTRO" -- git clone -q -b "$BRANCH" "$REPO" /opt/agv-git
fi

# ---- 4. 运行环境 (ROS 2 Humble / Nav2 / MuJoCo)
say "4/6 安装运行环境 (ROS 2 Humble + Nav2 + MuJoCo，首次约 20~40 分钟)"
proot-distro login "$DISTRO" -- env AGV_MIRROR="$MIRROR" bash /opt/agv-git/deploy/android/proot_setup.sh

# ---- 5. 配置 + 同步代码 + Termux 侧脚本
say "5/6 写配置 ~/.agv.env 并同步代码"
{
    echo "AGV_DISTRO=$DISTRO"
    echo "AGV_GIT_REMOTE=$REPO"
    echo "AGV_GIT_BRANCH=$BRANCH"
    echo "AGENT_NAME=$NAME"
    echo "AGV_HUB_PORT=${AGV_HUB_PORT:-8082}"
    if [ -n "$HUB" ]; then echo "HUB_API=$HUB"; echo "JOIN_TOKEN=$TOKEN"; fi
} > ~/.agv.env
chmod 600 ~/.agv.env
cp "$ROOTFS/opt/agv-git/deploy/android/agv_common.sh" ~/agv_common.sh
bash "$ROOTFS/opt/agv-git/deploy/android/update_from_git.sh"

# ---- 6. 启动
say "6/6 完成"
cat <<EOF
  代码: 容器 $DISTRO 内 /opt/agv (Termux 路径 $ROOTFS/opt/agv)
  脚本: ~/start_agv.sh ~/stop_agv.sh ~/status_agv.sh ~/update_from_git.sh   配置: ~/.agv.env
  建议: 1) 安装 Termux:Boot 并打开一次 (开机自启)  2) 系统设置里关闭 Termux 的电池优化
        3) Android 12+ 关闭幽灵进程限制 (见 docs/DEPLOY_ANDROID.md「幽灵进程」)
EOF
if [ "$START" = 1 ]; then
    bash ~/start_agv.sh
fi
