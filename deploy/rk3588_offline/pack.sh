#!/bin/bash
# ============================================================================
# 在能联网的 arm64 机器 (如 Apple 芯片 Mac + Docker Desktop) 上打包 RK3588 离线部署包
#   bash deploy/rk3588_offline/pack.sh [输出目录]      → agv-nav-rootfs.tar.xz + agv_ros.sh
# 然后:
#   scp <输出目录>/agv-nav-rootfs.tar.xz deploy/rk3588_offline/agv_ros.sh root@<板卡>:/mnt/agv_ros/
#   ssh root@<板卡> 'cd /mnt/agv_ros && mkdir -p rootfs && xz -dc agv-nav-rootfs.tar.xz | tar -x -C rootfs'
# ============================================================================
set -e
cd "$(dirname "$0")/../.."
OUT="${1:-dist/rk3588}"
mkdir -p "$OUT"
docker image inspect agv-nav:latest >/dev/null 2>&1 || ./deploy.sh build nav
arch=$(docker image inspect -f '{{.Architecture}}' agv-nav:latest)
[ "$arch" = arm64 ] || { echo "agv-nav:latest 是 $arch，RK3588 需要 arm64"; exit 1; }
cid=$(docker create agv-nav:latest)
trap 'docker rm -f "$cid" >/dev/null' EXIT
echo "导出 agv-nav:latest → $OUT/agv-nav-rootfs.tar.xz ..."
docker export "$cid" | xz -T0 -6 > "$OUT/agv-nav-rootfs.tar.xz"
cp deploy/rk3588_offline/agv_ros.sh "$OUT/"
ls -lh "$OUT"
