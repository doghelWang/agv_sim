#!/bin/bash
# agv-sim 入口 (Rust 版): sim-server bootstrap (模型/场景拉取与落盘) → sim-server (:SIM_API_PORT) → web-gateway (:WEB_PORT)
# 节点代理的 process 运行时在 sim_entrypoint_rs2.sh 存在时优先使用它 (AGENT_SIM_PYTHON=1 退回 Python 版 sim_entrypoint.sh)
cd "${AGV_HOME:-/opt/agv}"
export MUJOCO_LIB="${MUJOCO_LIB:-/usr/local/lib/python3.10/dist-packages/mujoco/libmujoco.so.3.16.0}"
BOOT="$(/opt/agv/bin/sim-server bootstrap)" || { echo "[agv-sim] bootstrap failed"; exit 1; }
eval "$BOOT"
export ROBOT_CONFIG="${ROBOT_CONFIG:-$AGV_DATA/robot_config.json}"
export SIM_API="http://127.0.0.1:${SIM_API_PORT:-8100}"
PIDS=""
cleanup() { kill $PIDS 2>/dev/null; wait 2>/dev/null; exit 1; }
trap cleanup SIGINT SIGTERM
/opt/agv/bin/sim-server --port "${SIM_API_PORT:-8100}" --config "$ROBOT_CONFIG" & SIM_PID=$!
PIDS="$SIM_PID"
sleep 2
/opt/agv/bin/web-gateway & WEB_PID=$!
PIDS="$PIDS $WEB_PID"
wait -n $SIM_PID $WEB_PID || true
echo "[agv-sim] process exited"
cleanup
