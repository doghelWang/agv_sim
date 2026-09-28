#!/bin/bash
# ============================================================================
# 导航快速回归 (树莓派 i12，几分钟内出结果) —— 改导航/参数后先跑这个，性能对比再用 ab_bench_i12.sh
#   bash tools/quick_nav_check.sh [--restart] [--planner nav2|dijkstra] [--goals JSON] [--tmax 90]
# grid_9_square 的工位关于两条轴镜像对称，不必遍历 13 个；默认 3 个目标连续走，每个覆盖一种"贴墙"情形:
#   S5 (-7.5,7.5)  角点工位: 经离墙 1.5 m 的节点转向 + 车头朝墙进站 (前向防护区按剩余行程缩短)
#   S10 (4.3,7.5)  从角点出发 (车头朝墙，段起点转向)，沿北排到装卸位后终点对位转向 (车角离墙约 5 cm)
#   P0 (0,0)       从贴墙装卸位离开 (段起点车头朝墙转向) 回中心
# 不重启实例 (除非 --restart): 省掉 Nav2/slam_toolbox 启动约 1 分钟；只统计本次检查期间的事件与控制器日志
# 非树莓派部署用环境变量改地址/日志来源，如 RK3588 离线部署 (deploy/rk3588_offline，--restart 不适用):
#   GW=http://127.0.0.1:8088 NAV=http://127.0.0.1:8091 NAV_LOG="ssh root@192.168.1.64 cat /mnt/agv_ros/logs/nav.log" bash tools/quick_nav_check.sh
# ============================================================================
cd "$(dirname "$0")/.." || exit 1
GW=${GW:-http://127.0.0.1:8101}; NAV=${NAV:-http://127.0.0.1:8102}; HUB=${HUB:-http://127.0.0.1:8082}; INST=${INST:-i12}
NAV_LOG=${NAV_LOG:-}   # 执行进程日志命令 (不设则 docker logs agv-nav-$INST)
PLANNER=nav2; TMAX=90; RESTART=0
GOALS='[[-7.5,7.5,3.14159],[4.3,7.5,1.5708],[0,0,0]]'
while [ $# -gt 0 ]; do
    case "$1" in
        --restart) RESTART=1 ;;
        --planner) PLANNER=$2; shift ;;
        --goals) GOALS=$2; shift ;;
        --tmax) TMAX=$2; shift ;;
    esac; shift
done
if [ $RESTART = 1 ]; then
    curl -s -X POST $HUB/api/hub/instances/$INST/restart >/dev/null; sleep 15
fi
t=0
until curl -s -m 2 $NAV/api/v1/nav | python3 -c "import json,sys;sys.exit(0 if (json.load(sys.stdin).get('nav2') or {}).get('server_ready') else 1)" 2>/dev/null; do
    sleep 2; t=$((t+2)); [ $t -gt 180 ] && { echo "Nav2 未就绪"; exit 1; }
done
E0=$(curl -s "$NAV/api/v1/events?since=0&limit=5000" | python3 -c "import json,sys;d=json.load(sys.stdin);ev=d.get('events',d) if isinstance(d,dict) else d;print(max([e['id'] for e in ev] or [0]))")
T0=$(date +%s)
L0=0; [ -n "$NAV_LOG" ] && L0=$($NAV_LOG | wc -l)
navlog() { if [ -n "$NAV_LOG" ]; then $NAV_LOG | tail -n +$((L0 + 1)); else docker logs --since "$T0" agv-nav-$INST 2>&1; fi; }
python3 tools/precision_test.py --gw $GW --planner $PLANNER --goals "$GOALS" --tmax $TMAX --out /tmp/quick_nav.json 2>&1 | grep -E "目标|汇总"
echo "-- 导航事件 (本次)"
curl -s "$NAV/api/v1/events?since=$E0&limit=5000" | python3 -c "
import json,sys,collections; d=json.load(sys.stdin); ev=d.get('events',d) if isinstance(d,dict) else d
print(dict(collections.Counter(e.get('type') for e in ev if e.get('category')=='navigation')))
for e in ev:
    if e.get('type') in ('NAV2_RETRY','NAV2_GIVEUP','NAV2_ABORTED','NAV2_ALIGN'): print('  ', e.get('time_str'), e.get('type'), e.get('title'), '|', (e.get('message') or '')[:140])"
echo "-- controller_server (本次，去掉控制频率告警)"
navlog | grep controller_server | grep -v "missed its desired" \
    | sed -E 's/\[[0-9]{10}\.[0-9]+\]//; s/Requested time.*frame \[map\]/(外推)/' | sort | uniq -c | sort -rn | head -8
echo "用时 $(( $(date +%s) - T0 )) s"
