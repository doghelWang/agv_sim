#!/bin/sh
# ============================================================================
# RK3588 离线单机全套 (对应手机单机模式): 资源平台 :8082 + 节点代理 :8070 + 实例 (仿真 / Web 网关 / 执行 ROS 2+Nav2)
# 全部跑在 chroot (Ubuntu 22.04，agv-rk 镜像 docker export 而来) 里。
#
# 板卡情况 (192.168.1.64): 根文件系统在内存里 (rootfs)，重启后恢复出厂；/mnt (ext4) 持久，但厂商 initrun.sh
# 在部分硬件/条件下会把 /mnt 换挂到别的分区 (U 盘 / 另一块 eMMC)，所以放在 /mnt/misc/agv_ros (厂商约定的持久目录)。
# 没有 Docker / gcc / bash，只有 busybox + chroot。板上原有的业务进程 (carServer / cmodel_daemon / S99app …)
# 一概不碰；删掉 /mnt/misc/agv_ros 即完全卸载。本脚本按自身所在目录定位，整个目录可以整体搬走。
#
#   sh /mnt/misc/agv_ros/agv_ros.sh boot       # 重启后执行: 恢复 SSH 公钥 + 挂载 + 启动平台/代理并自动拉起实例
#   sh /mnt/misc/agv_ros/agv_ros.sh install    # 解包 agv-rk-rootfs.tar.xz (保留平台数据 /root/.agv-hub)
#   sh /mnt/misc/agv_ros/agv_ros.sh key|mount|umount|start|stop|status|shell
#
# 平台 :8082 与节点代理 :8070 监听局域网 (与手机单机模式相同)；实例进程由平台设为只监听 127.0.0.1，经平台 /inst/<id>/ 访问；
# ROS 2 只走 127.0.0.1 (ROS_LOCALHOST_ONLY=1)。环境变量: AGV_HUB_PORT (8082)、AGENT_NAME (rk3588)、AGV_PLANNER (nav2)
# 打包见同目录 pack.sh (在能联网的 arm64 机器上执行)
# ============================================================================
BASE=$(cd "$(dirname "$0")" && pwd)
R=$BASE/rootfs
LOG=$BASE/logs
HUB_PORT=${AGV_HUB_PORT:-8082}
NODE=${AGENT_NAME:-rk3588}

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
    [ -x $R/bin/bash ] || { echo "未找到 $R (先 install)"; return 1; }
    mnt proc proc $R/proc
    mnt sysfs sysfs $R/sys
    mnt bind /dev $R/dev
    mnt devpts devpts $R/dev/pts
    mnt tmpfs tmpfs $R/dev/shm          # Fast DDS 共享内存传输
    mnt tmpfs tmpfs $R/tmp
    printf '127.0.0.1 localhost\n::1 localhost\n' > $R/etc/hosts
}

umount_all() {
    for m in dev/shm dev/pts dev tmp sys proc; do grep -q " $R/$m " /proc/mounts && umount $R/$m; done
}

# 在 chroot 里后台运行 (脱离 ssh 会话)；docker export 不带镜像 ENV，这里给干净的环境
bg() {   # $1 = 日志名, $2 = 命令
    chroot $R /usr/bin/setsid /usr/bin/env -i HOME=/root PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
        LANG=C.UTF-8 PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 ROS_LOCALHOST_ONLY=1 \
        /bin/bash -c "cd /opt/agv && $2" >> $LOG/$1.log 2>&1 < /dev/null &
}

up() { wget -q -O /dev/null -T 2 "$1" 2>/dev/null; }

start() {
    do_mount || return 1
    mkdir -p $LOG
    if ! up http://127.0.0.1:$HUB_PORT/api/hub/health; then
        bg hub "HUB_PORT=$HUB_PORT exec python3 -m hub.server"
        i=0; until up http://127.0.0.1:$HUB_PORT/api/hub/health; do i=$((i + 1)); [ $i -gt 60 ] && { echo "平台没有起来，见 $LOG/hub.log"; return 1; }; sleep 1; done
        echo "资源平台 :$HUB_PORT 已启动"
    fi
    if ! up http://127.0.0.1:8070/api/v1/health; then
        tok=$(cat $R/root/.agv-hub/cluster_token 2>/dev/null)
        bg agent "HUB_API=http://127.0.0.1:$HUB_PORT JOIN_TOKEN='$tok' AGENT_RUNTIME=process AGENT_NAME=$NODE AGENT_PORT=8070 \
            AGV_DEVICE_MODEL='RK3588' exec python3 -m agent.server"
        # 自动拉起要等代理可达 (否则实例部署报"节点不可达"就放弃了)
        i=0; until up http://127.0.0.1:8070/api/v1/health; do i=$((i + 1)); [ $i -gt 60 ] && { echo "节点代理没有起来，见 $LOG/agent.log"; return 1; }; sleep 1; done
        echo "节点代理 :8070 已启动"
    fi
    # 自动拉起实例: 已在运行就不动 / 重启最近的实例 (保留 SLAM 地图) / 没有就新部署 (仿真 + 执行都在本机)
    bg autostart "exec python3 deploy/android/autostart.py --hub http://127.0.0.1:$HUB_PORT --node $NODE --planner ${AGV_PLANNER:-nav2}"
    echo "实例自动拉起中 (约 2~3 分钟)，进度: tail -f $LOG/autostart.log"
    echo "资源平台: http://$(ifconfig eth0 2>/dev/null | sed -n 's/.*inet addr:\([0-9.]*\).*/\1/p'):$HUB_PORT"
}

stop() {
    # 只杀 chroot 里的进程: /proc/<pid>/root 指向 $R
    for sig in TERM KILL; do
        for p in /proc/[0-9]*; do
            [ "$(readlink $p/root 2>/dev/null)" = "$R" ] && kill -$sig "${p#/proc/}" 2>/dev/null
        done
        [ $sig = TERM ] && sleep 3
    done
    echo "chroot 内进程已全部停止"
}

status() {
    n=0
    for p in /proc/[0-9]*; do [ "$(readlink $p/root 2>/dev/null)" = "$R" ] && n=$((n + 1)); done
    echo "chroot 内进程 $n 个"
    up http://127.0.0.1:$HUB_PORT/api/hub/health && echo "资源平台 :$HUB_PORT 正常" || echo "资源平台 :$HUB_PORT 未运行"
    up http://127.0.0.1:8070/api/v1/health && echo "节点代理 :8070 正常" || echo "节点代理 :8070 未运行"
    tail -4 $LOG/autostart.log 2>/dev/null
}

install() {   # 解包新版本；保留平台数据 (实例、模型、SLAM 地图) 与代理数据
    f=$BASE/agv-rk-rootfs.tar.xz
    [ -f $f ] || { echo "缺少 $f"; return 1; }
    stop; umount_all
    rm -rf $BASE/rootfs.new && mkdir -p $BASE/rootfs.new
    xz -dc $f | tar -x -C $BASE/rootfs.new || { echo "解包失败"; return 1; }
    for d in root/.agv-hub root/.agv-agent data; do
        [ -d $R/$d ] && { rm -rf $BASE/rootfs.new/$d; mkdir -p $(dirname $BASE/rootfs.new/$d); mv $R/$d $BASE/rootfs.new/$d; }
    done
    rm -rf $BASE/rootfs.old; [ -d $R ] && mv $R $BASE/rootfs.old
    mv $BASE/rootfs.new $R && rm -rf $BASE/rootfs.old
    echo "已安装 $(cat $R/opt/agv/.agv_version 2>/dev/null)"
}

case "$1" in
    boot) key; start ;;
    key) key ;;
    install) install ;;
    mount) do_mount ;;
    umount) umount_all ;;
    start) start ;;
    stop) stop ;;
    status) status ;;
    shell) do_mount && chroot $R /bin/bash -l ;;
    *) sed -n 2,19p "$0" ;;
esac
