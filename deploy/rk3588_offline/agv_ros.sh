#!/bin/sh
# ============================================================================
# RK3588 离线部署: 执行进程 (ROS 2 Humble + Nav2 + slam_toolbox) 跑在 chroot 里
#
# 板卡情况 (192.168.1.64): 根文件系统在内存里 (rootfs)，重启后恢复出厂；只有 /mnt (ext4) 持久。
# 没有 Docker / gcc / bash，只有 busybox + chroot → 把 agv-nav 镜像 docker export 成目录树放在 /mnt/agv_ros/rootfs。
# 板上原有的业务进程 (carServer / cmodel_daemon / S99app …) 一概不碰；删掉 /mnt/agv_ros 即完全卸载。
#
#   sh /mnt/agv_ros/agv_ros.sh boot       # 重启后执行: 恢复 SSH 公钥 + 挂载 + 启动执行进程
#   sh /mnt/agv_ros/agv_ros.sh key|mount|start|stop|status|shell
#
# 环境变量: SIM_API (默认 http://127.0.0.1:8090，经 ssh -R 反向隧道接到仿真主机)、NAV_API_PORT (8091)、
#           ROS_DOMAIN_ID (0)；执行进程与 DDS 都只走 127.0.0.1，不向局域网发包
# 打包见同目录 pack.sh (在能联网的 arm64 机器上执行)
# ============================================================================
BASE=/mnt/agv_ros
R=$BASE/rootfs
LOG=$BASE/logs
PIDF=$BASE/nav.pid

key() {   # 根文件系统在内存里，/root/.ssh 每次重启都会丢
    [ -f $BASE/authorized_keys ] || { echo "缺少 $BASE/authorized_keys"; return 1; }
    mkdir -p /root/.ssh
    cp $BASE/authorized_keys /root/.ssh/authorized_keys
    chmod 755 /root; chmod 700 /root/.ssh; chmod 600 /root/.ssh/authorized_keys   # dropbear 拒绝组/其他可写的家目录
    echo "SSH 公钥已恢复"
}

mnt() {   # $1 = 类型或 bind, $2 = 源, $3 = 目标
    grep -q " $3 " /proc/mounts && return 0
    mkdir -p "$3"
    if [ "$1" = bind ]; then mount -o bind "$2" "$3"; else mount -t "$1" "$2" "$3"; fi
}

do_mount() {
    [ -x $R/bin/bash ] || { echo "未找到 $R (先解包 agv-nav-rootfs.tar.xz)"; return 1; }
    mnt proc proc $R/proc
    mnt sysfs sysfs $R/sys
    mnt bind /dev $R/dev
    mnt devpts devpts $R/dev/pts
    mnt tmpfs tmpfs $R/dev/shm          # Fast DDS 共享内存传输
    mnt tmpfs tmpfs $R/tmp
    printf '127.0.0.1 localhost\n::1 localhost\n' > $R/etc/hosts
    echo "chroot 挂载就绪"
}

running() { [ -f $PIDF ] && kill -0 "$(cat $PIDF)" 2>/dev/null; }

start() {
    running && { echo "执行进程已在运行 (pid $(cat $PIDF))"; return 0; }
    do_mount || return 1
    mkdir -p $LOG
    # docker export 不带镜像的 ENV，这里补齐 (对应 docker/Dockerfile.nav)
    chroot $R /usr/bin/env -i HOME=/root PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
        LANG=C.UTF-8 PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1 AGV_HOME=/opt/agv AGV_BIND=127.0.0.1 \
        NAV_API_PORT=${NAV_API_PORT:-8091} SIM_API=${SIM_API:-http://127.0.0.1:8090} NAV_USE_ROS=1 \
        ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/roslog \
        NAV_LOCALIZATION=slam LOC_ENGINE=auto NAV2_AUTOSTART=1 SIM_LOCALIZATION=slam \
        bash /usr/local/bin/agv-nav-entrypoint >> $LOG/nav.log 2>&1 &
    echo $! > $PIDF
    echo "执行进程已启动 (pid $!)，日志 $LOG/nav.log"
}

stop() {
    # 只杀 chroot 里的进程: /proc/<pid>/root 指向 $R
    for p in /proc/[0-9]*; do
        [ "$(readlink $p/root 2>/dev/null)" = "$R" ] && kill "${p#/proc/}" 2>/dev/null
    done
    sleep 2
    for p in /proc/[0-9]*; do
        [ "$(readlink $p/root 2>/dev/null)" = "$R" ] && kill -9 "${p#/proc/}" 2>/dev/null
    done
    rm -f $PIDF
    echo "执行进程已停止"
}

status() {
    n=0
    for p in /proc/[0-9]*; do [ "$(readlink $p/root 2>/dev/null)" = "$R" ] && n=$((n + 1)); done
    running && echo "执行进程运行中 (pid $(cat $PIDF))，chroot 内进程 $n 个" || echo "执行进程未运行 (chroot 内进程 $n 个)"
    wget -q -O - -T 2 http://127.0.0.1:${NAV_API_PORT:-8091}/api/v1/health 2>/dev/null && echo
}

case "$1" in
    boot) key; start ;;
    key) key ;;
    mount) do_mount ;;
    start) start ;;
    stop) stop ;;
    status) status ;;
    shell) do_mount && chroot $R /bin/bash -l ;;
    *) sed -n 2,15p "$0" ;;
esac
