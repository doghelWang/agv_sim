# 用不同的 proot 参数组合运行同一个微基准
R=$PREFIX/var/lib/proot-distro/containers/ubuntu/rootfs
P=$(pgrep -f "bin/proot --kill-on-exit" | head -1)
mapfile -d '' ARGS < /proc/$P/cmdline
run() { # $1 = 要去掉的参数前缀的正则
  local a=(); local skip=0
  for x in "${ARGS[@]}"; do
    [ "$x" = /bin/bash ] && break
    if [ -n "$1" ] && echo "$x" | grep -qE -- "$1"; then continue; fi
    a+=("$x")
  done
  (cd $R && env -i HOME=/root PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin TERM=xterm LANG=C.UTF-8 "${a[@]}" /usr/bin/python3 /tmp/sysb2.py 2>&1 | tail -2)
}
cp ~/sysb2.py $R/tmp/sysb2.py
echo "cpuset $(cat /proc/self/cpuset)"
echo "[原生]        $(python3 ~/sysb2.py)"
echo "[现状]        $(run '')"
echo "[去掉 kernel-release] $(run '^--kernel-release')"
echo "[再去掉 change-id]    $(run '^--kernel-release|^--change-id')"
echo "[再去掉 link2symlink/sysvipc/-L] $(run '^--kernel-release|^--change-id|^--link2symlink|^--sysvipc|^-L$')"
