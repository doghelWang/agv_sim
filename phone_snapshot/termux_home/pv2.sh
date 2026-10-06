R=$PREFIX/var/lib/proot-distro/containers/ubuntu/rootfs
unset LD_PRELOAD
K='--kernel-release=\Linux\localhost\6.17.0-PRoot-Distro\#1 SMP PREEMPT_DYNAMIC Fri, 10 Oct 2025 00:00:00 +0000\aarch64\localdomain\-1\'
run() { echo "== $1"; shift; "$@" -r $R -b /dev -b /proc -b /sys -w /tmp /usr/bin/env -i PATH=/usr/bin:/bin HOME=/root /usr/bin/python3 /tmp/sysb.py 2>&1 | grep -E "sendto|epoll|stat 路径|open\+read|pipe"; }
run "只加 --kernel-release" proot "$K"
run "只加 --change-id=0:0" proot --change-id=0:0
run "全套去掉 --kernel-release" proot --kill-on-exit --link2symlink --sysvipc -L --change-id=0:0
run "精简: kill-on-exit + sysvipc" proot --kill-on-exit --sysvipc
