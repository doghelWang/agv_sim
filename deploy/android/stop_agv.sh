#!/data/data/com.termux/files/usr/bin/bash
# 停止手机上的平台 / 节点代理 / 派生服务 / 全部仿真与 ROS 进程
#   做法: 结束派生服务，再结束该 proot 容器的全部 proot 追踪进程 (带 --kill-on-exit，里面的程序随之结束)。
#   注意: 同一容器里你自己开的 proot-distro login 交互终端也会被关闭。
. ~/agv_common.sh
echo "=== 停止 AMR 仿真节点 ($AGV_DISTRO) ==="
pkill -f "^([^ ]*/)?python3? [^ ]*proot_spawner.py" 2>/dev/null
pkill -f "^([^ ]*/)?gpucastd 8068" 2>/dev/null
sleep 0.5
pkill -9 -f "^([^ ]*/)?proot .*(containers/$AGV_DISTRO/|installed-rootfs/$AGV_DISTRO)" 2>/dev/null
sleep 1
# 追踪进程被强杀时个别程序可能脱离 (父进程变成 1)：按工作目录在容器 rootfs 下的进程补杀
for d in /proc/[0-9]*; do
    c=$(readlink "$d/cwd" 2>/dev/null) || continue
    case "$c" in "$AGV_ROOTFS"/*|"$AGV_ROOTFS") kill -9 "${d#/proc/}" 2>/dev/null;; esac
done
sleep 0.5
left=$(ps -A -o args | grep -cE "^python3 -m (hub|agent|sim_server|nav_runtime)|^/opt/ros/humble/lib/")
echo "[OK] 已停止 (剩余相关进程 $left)"
