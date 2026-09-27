#!/bin/bash
# 用法 (树莓派，平台 :8082，实例 i12 端口 8100/8101/8102): bash tools/ab_bench_i12.sh <标签>；结果在 ~/bench/<标签>/
# 对比某个开关时先给镜像加一层 ENV，如: printf "FROM agv-nav:latest\nENV NAV_ROS_BRIDGE=py\n" | docker build -t agv-nav:latest -
# 树莓派 i12 对比测量: bash tools/ab_bench_i12.sh <label>
# 重启 i12 → 等 Nav2 就绪 → 采样 CPU/step_ms/状态频率 + py-spy (sim/nav 各 60 s) → 精度回归 (dijkstra, nav2)
L=${1:?label}; OUT=~/bench/$L; mkdir -p $OUT; cd ~/ros2_cmodel_agv
GOALS='[[0,5,1.5708],[5,0,0],[0,0,0]]'
curl -s -X POST localhost:8082/api/hub/instances/i12/restart >/dev/null
for i in $(seq 1 60); do sleep 3; s=$(curl -s localhost:8102/api/v1/nav | python3 -c "import json,sys;d=json.load(sys.stdin);print((d.get('nav2') or {}).get('server_ready') or (d.get('nav2') or {}).get('active'))" 2>/dev/null); [ "$s" = True ] && break; done
echo "nav2 ready after ~$((i*3))s"; sleep 10
curl -s localhost:8100/api/v1/sim | python3 -c "import json,sys;d=json.load(sys.stdin);print('native:',d.get('native'),'step_ms',d['step_ms'])"
T0=$(date +%s); rm -f $OUT/stop; python3 tools/perf_sample.py --secs 1800 --interval 2 --sim http://127.0.0.1:8100 --nav http://127.0.0.1:8102 --label $L --out $OUT/perf.json --stop-file $OUT/stop > $OUT/perf.log 2>&1 &
SP=$(docker top agv-sim-i12 -eo pid,args | awk '/sim_server.api/{print $1; exit}')
NP=$(docker top agv-nav-i12 -eo pid,args | awk '/nav_runtime.main/{print $1; exit}')
( sleep 15; sudo ~/.local/bin/py-spy record -p $SP -d 60 -r 100 -f raw --gil -o $OUT/spy_sim.txt --nonblocking >/dev/null 2>&1 ) &
( sleep 15; sudo ~/.local/bin/py-spy record -p $NP -d 60 -r 100 -f raw --gil -o $OUT/spy_nav.txt --nonblocking >/dev/null 2>&1 ) &
python3 tools/precision_test.py --gw http://127.0.0.1:8101 --goals "$GOALS" --planner dijkstra --out $OUT/prec_dijkstra.json > $OUT/prec_dijkstra.log 2>&1
python3 tools/precision_test.py --gw http://127.0.0.1:8101 --goals "$GOALS" --planner nav2 --out $OUT/prec_nav2.json > $OUT/prec_nav2.log 2>&1
# 长路线: 场景全部工位 (Nav2)，用于统计线路跟随中断
python3 tools/precision_test.py --gw http://127.0.0.1:8101 --planner nav2 --out $OUT/prec_nav2_all.json > $OUT/prec_nav2_all.log 2>&1
touch $OUT/stop; wait
# Nav2 线路跟随中断统计: NAV2_RETRY 事件 + controller_server 的 TF 外推异常 / 放弃
{ curl -s "localhost:8102/api/v1/events?since=0&limit=5000" | python3 -c "
import json,sys; d=json.load(sys.stdin); ev=d.get('events', d) if isinstance(d, dict) else d
r=[e for e in ev if e.get('type')=='NAV2_RETRY']; print('NAV2_RETRY', len(r)); [print('  ', e.get('time_str'), e.get('title'), e.get('message')) for e in r[:20]]"
  L=$(docker logs --since "$T0" agv-nav-i12 2>&1)
  echo "transformPose 外推异常 $(echo "$L" | grep -c 'Exception in transformPose')"
  echo "Controller patience exceeded $(echo "$L" | grep -c 'Controller patience exceeded')"
  echo "follow_path Aborting $(echo "$L" | grep -c 'follow_path.*Aborting')"; } > $OUT/abort.txt 2>&1
cat $OUT/abort.txt
grep 汇总 $OUT/prec_*.log
python3 tools/perf_summary.py $OUT/perf.json
