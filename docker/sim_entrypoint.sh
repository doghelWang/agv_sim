#!/bin/bash
# agv-sim 入口: 准备数据 (默认模型 / 平台下发的模型与场景) → 仿真进程 (:SIM_API_PORT) → Web 网关 (:WEB_PORT)
# 容器内 AGV_HOME=/opt/agv；节点代理的 process 运行时会把 AGV_HOME 指向源码目录
set -e
cd "${AGV_HOME:-/opt/agv}"
PY="${PYTHON:-python3}"
DATA="${AGV_DATA:-/data}"
export AGV_DATA="$DATA"
mkdir -p "$DATA"
# 1) 数据初始化 + 平台下发 (HUB_API + MODEL_ID / SCENE_ID)
eval "$($PY -m sim_server.bootstrap)"
# 2) 挂载目录里放了新 cmodel 时可设 CMODEL_FILE 重新解析 (人工补全自动叠加)
if [ -n "$CMODEL_FILE" ] && [ -f "$CMODEL_FILE" ]; then
    echo "[agv-sim] 解析 $CMODEL_FILE"
    $PY cmodel_parser.py "$CMODEL_FILE" "$DATA" --load "${CMODEL_LOAD:-full}" --no-nav2
fi
export ROBOT_CONFIG="${ROBOT_CONFIG:-$DATA/robot_config.json}"
export SIM_API="http://127.0.0.1:${SIM_API_PORT:-8090}"
export NAV_API="${NAV_API:-http://127.0.0.1:8091}"

echo "=========================================================="
echo " agv-sim ${INSTANCE_ID:+实例 $INSTANCE_ID }仿真进程 :${SIM_API_PORT:-8090}   Web 网关 :${WEB_PORT:-8088}"
echo " 执行进程 NAV_API=$NAV_API   数据 $DATA${HUB_API:+   平台 $HUB_API}"
[ -n "$SIM_SCENE_FILE" ] && echo " 场景 $SIM_SCENE_FILE"
$PY -c "import mujoco; print(' 引擎 MuJoCo', mujoco.__version__)" 2>/dev/null || echo " 引擎 kinematic (未安装 mujoco)"
echo "=========================================================="

PIDS=""
cleanup() { kill $PIDS 2>/dev/null; wait 2>/dev/null; exit 1; }
trap cleanup SIGINT SIGTERM

$PY -m sim_server.api --port "${SIM_API_PORT:-8090}" --config "$ROBOT_CONFIG" & SIM_PID=$!
PIDS="$SIM_PID"
for i in $(seq 1 120); do
    kill -0 $SIM_PID 2>/dev/null || { echo "[agv-sim] 仿真进程启动失败"; cleanup; }
    $PY -c "import urllib.request,sys; sys.exit(0 if b'\"service\":\"sim\"' in urllib.request.urlopen('$SIM_API/api/v1/health',timeout=2).read() else 1)" 2>/dev/null && break
    sleep 0.5
done
$PY -m common.spawn exec web -- $PY web_gateway.py & WEB_PID=$!   # Android proot: 网关放独立 proot 会话 (其它环境等同直接运行)
PIDS="$PIDS $WEB_PID"
wait -n $SIM_PID $WEB_PID || true
echo "[agv-sim] 有进程退出，容器重启"
cleanup
