#!/bin/bash
# ============================================================================
# 在能联网的 arm64 机器 (如 Apple 芯片 Mac + Docker Desktop) 上打包 RK3588 离线部署包
#   bash deploy/rk3588_offline/pack.sh [输出目录]      → agv-rk-rootfs.tar.xz + agv_ros.sh
# 镜像 agv-rk = agv-nav (ROS 2/Nav2) + 仿真依赖 + 完整代码 (同目录 Dockerfile)
# 然后 (dropbear 没有 sftp-server，scp 不可用，用管道):
#   cat <输出目录>/agv_ros.sh | ssh root@<板卡> 'mkdir -p /mnt/agv_ros && cat > /mnt/agv_ros/agv_ros.sh'
#   cat <输出目录>/agv-rk-rootfs.tar.xz | ssh root@<板卡> 'cat > /mnt/agv_ros/agv-rk-rootfs.tar.xz'
#   ssh root@<板卡> 'sh /mnt/agv_ros/agv_ros.sh install'
# ============================================================================
set -e
cd "$(dirname "$0")/../.."
OUT="${1:-dist/rk3588}"
mkdir -p "$OUT"
docker image inspect agv-nav:latest >/dev/null 2>&1 || ./deploy.sh build nav
docker build -f deploy/rk3588_offline/Dockerfile \
    --build-arg AGV_VERSION="$(git rev-parse --short HEAD 2>/dev/null || echo local) rk3588" -t agv-rk:latest .
arch=$(docker image inspect -f '{{.Architecture}}' agv-rk:latest)
[ "$arch" = arm64 ] || { echo "agv-rk:latest 是 $arch，RK3588 需要 arm64"; exit 1; }
cid=$(docker create agv-rk:latest)
trap 'docker rm -f "$cid" >/dev/null' EXIT
echo "导出 agv-rk:latest → $OUT/agv-rk-rootfs.tar.xz ..."
docker export "$cid" | xz -T0 -6 > "$OUT/agv-rk-rootfs.tar.xz"
cp deploy/rk3588_offline/agv_ros.sh "$OUT/"
ls -lh "$OUT"
