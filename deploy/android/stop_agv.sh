#!/data/data/com.termux/files/usr/bin/bash
# 停止手机上的平台 / 节点代理 / 派生服务 / 全部仿真与 ROS 进程
#   做法: 结束派生服务，再结束该 proot 容器的全部 proot 追踪进程 (带 --kill-on-exit，里面的程序随之结束)。
#   注意: 同一容器里你自己开的 proot-distro login 交互终端也会被关闭。
. ~/agv_common.sh
echo "=== 停止 AMR 仿真节点 ($AGV_DISTRO) ==="
pkill -f "^([^ ]*/)?python3? [^ ]*proot_spawner.py" 2>/dev/null
pkill -f "^([^ ]*/)?python3? [^ ]*android_helper.py" 2>/dev/null
pkill -f "^([^ ]*/)?gpucastd 8068" 2>/dev/null
kill "$(cat ~/.agv_front.pid 2>/dev/null)" 2>/dev/null; rm -f ~/.agv_front.pid
sleep 0.5
# 先结束 proot 里的程序，再结束追踪进程: 反过来 (直接强杀追踪进程) 程序不会跟着退出，会留下一批残留进程
T=$(agv_tracees); [ -n "$T" ] && kill -9 $T 2>/dev/null
sleep 0.3
pkill -9 -f "^([^ ]*/)?proot .*(containers/$AGV_DISTRO/|installed-rootfs/$AGV_DISTRO)" 2>/dev/null
sleep 1
O=$(agv_orphans); [ -n "$O" ] && { kill -9 $O 2>/dev/null; echo "[info] 清理残留进程 $(echo $O | wc -w) 个"; }
sleep 0.5
left=$(( $(agv_orphans | wc -l) + $(agv_tracees | wc -l) ))
echo "[OK] 已停止 (剩余相关进程 $left)"
