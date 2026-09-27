#!/bin/bash
# ============================================================================
# Linux 板卡 / 电脑一键部署 (RK3588、树莓派 5、x86；Ubuntu 22.04/24.04、Debian 11/12、Armbian …)
#
#   git clone https://github.com/doghelWang/agv_sim.git ~/agv_sim && cd ~/agv_sim
#   bash deploy/linux/setup.sh                      # 本机当主平台 (资源平台 + 节点代理)
#   bash deploy/linux/setup.sh --hub http://<主平台IP>:8080 --token <集群令牌> [--name 名称]   # 接入已有主平台
#
# 网络: 需要能访问 GitHub (克隆/更新)、Docker Hub 或其镜像站 (基础镜像)、Ubuntu/ROS/PyPI 源 (默认清华镜像)。
#       Docker Hub 不通时自动改用 docker.m.daocloud.io；GitHub 不通时先 git config --global http.proxy http://<代理>
# 选项: --swap 4G  内存 <6 GB 时建议: 建 swap 文件 (编译/运行更稳)
#       --no-build  镜像已存在时跳过构建   --official  不用清华镜像 (海外网络)
# 步骤: 检查硬件/系统 → 安装 Docker → (可选) swap → 构建 agv-sim / agv-nav / agv-platform 镜像 → 启动平台或节点代理
# 首次构建需要下载约 2 GB (ROS 2 Humble 基础镜像 + Nav2)，树莓派 5 约 30~60 分钟，RK3588 约 20~40 分钟。
# ============================================================================
set -e
cd "$(dirname "$0")/../.."
HUB=""; TOKEN=""; NODE_NAME=""; SWAP=""; BUILD=1; OFFICIAL=0
while [ $# -gt 0 ]; do
    case "$1" in
        --hub) HUB="$2"; shift 2;; --token) TOKEN="$2"; shift 2;; --name) NODE_NAME="$2"; shift 2;;
        --swap) SWAP="$2"; shift 2;; --no-build) BUILD=0; shift;; --official) OFFICIAL=1; shift;;
        -h|--help) sed -n 2,14p "$0"; exit 0;;
        *) echo "未知参数 $1"; exit 1;;
    esac
done
say() { echo -e "\033[1;36m[setup]\033[0m $*"; }
warn() { echo -e "\033[1;33m[setup]\033[0m $*"; }
SUDO=""; [ "$(id -u)" = 0 ] || SUDO="sudo"

# ---- 1. 硬件与系统
ARCH=$(uname -m); MEM_MB=$(awk '/MemTotal/{print int($2/1024)}' /proc/meminfo)
SWAP_MB=$(awk '/SwapTotal/{print int($2/1024)}' /proc/meminfo); DISK_GB=$(df -BG --output=avail . | tail -1 | tr -dc 0-9)
MODEL=$(tr -d '\0' < /proc/device-tree/model 2>/dev/null || grep -m1 "model name" /proc/cpuinfo | cut -d: -f2)
. /etc/os-release 2>/dev/null || true
say "设备: ${MODEL:-未知}  架构 $ARCH  内存 ${MEM_MB} MB  swap ${SWAP_MB} MB  可用磁盘 ${DISK_GB} GB  系统 ${PRETTY_NAME:-?}"
case "$ARCH" in aarch64|arm64|x86_64) ;; *) echo "[错误] 只支持 64 位 ARM (aarch64) 或 x86_64"; exit 1;; esac
[ "${DISK_GB:-0}" -ge 12 ] || warn "可用磁盘不足 12 GB，镜像构建可能失败"
if [ "$MEM_MB" -lt 6000 ] && [ "$SWAP_MB" -lt 3000 ] && [ -z "$SWAP" ]; then
    warn "内存 < 6 GB 且 swap 很小: 建议加 --swap 4G (否则构建 Nav2 镜像时可能内存耗尽卡死)"
fi

# ---- 2. Docker
if ! command -v docker >/dev/null 2>&1; then
    say "安装 Docker"
    if command -v apt-get >/dev/null; then
        $SUDO apt-get update -q && $SUDO apt-get install -y -q docker.io curl git
    else
        curl -fsSL https://get.docker.com | $SUDO sh
    fi
fi
$SUDO systemctl enable --now docker >/dev/null 2>&1 || $SUDO service docker start || true
if ! docker info >/dev/null 2>&1; then
    if [ -n "$SUDO" ] && $SUDO docker info >/dev/null 2>&1; then
        $SUDO usermod -aG docker "$USER"
        warn "已把 $USER 加入 docker 组: 请重新登录 (或执行 newgrp docker) 后再运行本脚本"
    else
        warn "Docker 服务没有运行: sudo systemctl start docker (或 sudo service docker start) 后再运行本脚本"
    fi
    exit 1
fi
say "Docker $(docker version --format '{{.Server.Version}}' 2>/dev/null)"

# ---- 3. swap (可选)
if [ -n "$SWAP" ] && [ ! -f /swapfile.agv ]; then
    say "创建 swap 文件 /swapfile.agv ($SWAP)"
    $SUDO fallocate -l "$SWAP" /swapfile.agv || $SUDO dd if=/dev/zero of=/swapfile.agv bs=1M count=$(( ${SWAP%G} * 1024 ))
    $SUDO chmod 600 /swapfile.agv && $SUDO mkswap /swapfile.agv >/dev/null && $SUDO swapon /swapfile.agv
    grep -q swapfile.agv /etc/fstab || echo "/swapfile.agv none swap sw 0 0" | $SUDO tee -a /etc/fstab >/dev/null
fi

# ---- 4. 构建镜像
if [ "$OFFICIAL" = 1 ]; then export APT_MIRROR="" PIP_INDEX=""; fi
# Docker Hub 直连不通 (国内常见) 且 Docker 没配 registry-mirrors 时，基础镜像改从镜像站拉
if [ -z "$BASE_REGISTRY" ] && ! docker info 2>/dev/null | grep -qi "Registry Mirrors"; then
    code=$(curl -s -m 8 -o /dev/null -w "%{http_code}" https://registry-1.docker.io/v2/ || true)
    if [ "$code" != 401 ] && [ "$code" != 200 ]; then
        export BASE_REGISTRY="${DOCKER_MIRROR:-docker.m.daocloud.io}"
        warn "Docker Hub 连不上，基础镜像改从 $BASE_REGISTRY 拉取 (可用 DOCKER_MIRROR=... 指定其它镜像站)"
    fi
fi
if [ "$BUILD" = 1 ]; then
    say "构建镜像 (agv-sim / agv-nav / agv-platform)，首次较慢"
    bash deploy.sh build all
fi

# ---- 5. 启动
if [ -n "$HUB" ]; then
    [ -n "$TOKEN" ] || { read -r -s -p "主平台集群令牌: " TOKEN; echo; }
    bash deploy.sh agent --hub "$HUB" --token "$TOKEN" ${NODE_NAME:+--name "$NODE_NAME"}
    say "已作为计算节点接入 $HUB，在主平台「部署仿真」里选择本节点"
else
    bash deploy.sh hub ${NODE_NAME:+--name "$NODE_NAME"}
    . ./.hub.env 2>/dev/null || true
    IP=$(hostname -I 2>/dev/null | awk '{print $1}')
    say "主平台: http://$IP:${HUB_PORT_ENV:-8080}   集群令牌: ~/.agv-hub/cluster_token (其它节点接入时用)"
fi
bash deploy.sh status || true
