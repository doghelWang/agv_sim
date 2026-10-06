#!/data/data/com.termux/files/usr/bin/bash
# 前台看守 (start_agv.sh 启动，stop_agv.sh 结束): 手机上切到别的应用、或面板被关掉后，Termux 会被系统限制到小核
# (cpuset 不再是 /top-app)，Nav2 应答超时、任务一下发就失败。每 10 秒看一次，连续 3 次 (约 30 秒) 不在前台，
# 就让面板应用把自己和 Termux 重新调到前台。记录写在 ~/agv_front.log。
n=0
while sleep 10; do
    if [ "$(cat /proc/self/cpuset 2>/dev/null)" = /top-app ]; then n=0; else n=$((n + 1)); fi
    if [ $n -ge 3 ]; then
        am broadcast -n com.agvsim.cover/.StartReceiver >/dev/null 2>&1
        echo "$(date '+%F %T') 不在前台 (cpuset $(cat /proc/self/cpuset 2>/dev/null))，重新调到前台" >> ~/agv_front.log
        n=0
    fi
done
