R=$PREFIX/var/lib/proot-distro/containers/ubuntu/rootfs
unset LD_PRELOAD
run() { echo "== $1"; shift; "$@" -r $R -b /dev -b /proc -b /sys -w /tmp /usr/bin/env -i PATH=/usr/bin:/bin HOME=/root /usr/bin/python3 /tmp/sysb.py 2>&1 | grep -E "sendto|sendmsg|epoll|stat 路径|open\+read|TCP send|listdir"; }
run "最小 (无任何扩展)" proot
run "+ -0 (伪 root)" proot -0
run "+ --link2symlink" proot --link2symlink
run "+ --sysvipc" proot --sysvipc
run "+ --kill-on-exit" proot --kill-on-exit
run "+ -L" proot -L
run "PROOT_NO_SECCOMP=1 最小" env PROOT_NO_SECCOMP=1 proot
