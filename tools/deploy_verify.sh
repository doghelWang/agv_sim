#!/bin/bash
# ============================================================================
# 单机 (树莓派/RK3588/x86) 一键部署 + 验证 开源定位/导航栈 (slam_toolbox + robot_localization + Nav2)
#
#   cd ~/agv_sim && bash tools/deploy_verify.sh            # 构建镜像 + 单机启动 + 验证
#   bash tools/deploy_verify.sh --no-build                         # 镜像已构建，只重启与验证
# 输出同时写入 verify_<时间>.log，把这个文件发给开发者。
# 流程: 构建 agv-sim / agv-nav → deploy.sh up (单机: 网关 :8088，执行 :8091)
#       → 定位栈检查 → 第 1 轮 (slam 建图) 跑一圈工位 → 保存地图 → 第 2 轮 (slam_toolbox 定位) 再跑一圈
# ============================================================================
cd "$(dirname "$0")/.." || exit 1
LOG="verify_$(date +%Y%m%d_%H%M%S).log"
exec > >(tee "$LOG") 2>&1
# 端口: deploy.sh 在端口被占用时会顺延，所以从容器环境变量里取实际端口
envp() { docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "$1" 2>/dev/null | sed -n "s/^$2=//p" | head -1; }
SC="${SCENARIO:-grid_9_square}"
echo "== $(date)  $(hostname)  $(uname -m)"
git log --oneline -1 2>/dev/null
if [ "$1" != "--no-build" ]; then
    bash deploy.sh down || true
    bash deploy.sh build || { echo "构建失败"; exit 1; }
fi
bash deploy.sh up || { echo "启动失败"; exit 1; }
WP=$(envp agv-sim WEB_PORT); NP=$(envp agv-nav NAV_API_PORT)
GW="${GW:-http://127.0.0.1:${WP:-8088}}"; NAV="${NAV:-http://127.0.0.1:${NP:-8091}}"
echo "== 网关 $GW  执行 $NAV"
# 其它仿真实例会抢 CPU，结果不可信：提示
docker ps --format '{{.Names}}' | grep -E '^agv-(sim|nav)-i' && echo "[注意] 平台实例正在运行，会与本次测试争用 CPU"
echo "== 等待执行进程与 Nav2 就绪 (最多 240 s)"
for i in $(seq 1 120); do
    s=$(curl -s "$NAV/api/v1/nav" 2>/dev/null)
    echo "$s" | grep -q '"server_ready": *true' && break
    sleep 2
done
curl -s "$NAV/api/v1/nav" | python3 -c "import json,sys; d=json.load(sys.stdin); print('planner', d.get('planner'), 'nav2', d.get('nav2'))"
bash tools/check_loc_stack.sh agv-nav "${NP:-8091}"
GOALS="${GOALS:-[[0,5,1.5708],[5,0,0],[0,-5,-1.5708],[-5,0,3.1416],[0,0,0]]}"
echo "== 第 1 轮: slam 建图 ($SC)"
python3 tools/precision_test.py --gw "$GW" --scenario "$SC" --goals "$GOALS" --planner nav2 --out "precision_mapping.json"
echo "== 保存地图"
curl -s -XPOST "$NAV/api/v1/slam/save" -H 'Content-Type: application/json' -d '{}' | head -c 600; echo
curl -s -XPOST "$NAV/api/v1/slam/mode" -H 'Content-Type: application/json' -d '{"mode":"localization"}' | head -c 300; echo
sleep 8
echo "== 第 2 轮: slam_toolbox 定位"
python3 tools/precision_test.py --gw "$GW" --goals "$GOALS" --planner nav2 --out "precision_localization.json"
echo "== 对照: 自研导引 (dijkstra) + 同一定位"
python3 tools/precision_test.py --gw "$GW" --goals "$GOALS" --planner dijkstra --out "precision_dijkstra.json"
echo "== 执行进程日志 (末 80 行)"
docker logs --tail 80 agv-nav 2>&1
echo "== 完成，日志: $LOG"
