#!/bin/bash
# ============================================================================
# AMR Studio V4 · CModel AGV 仿真 + Nav2 一键启动 (三进程 RESTful 架构)
#
#   仿真进程 sim_server   :8090  模型解析/构建 + 物理 + 激光/光电/触边/IO 数据产生 (不依赖 ROS)
#   执行进程 nav_runtime  :8091  ROS 2 + Nav2 导航执行，经 REST 拉取仿真数据、回馈控制量
#   Web 网关 web_gateway  :8088  浏览器调度台，聚合两进程 REST
#   进程间只有 HTTP REST，无 ROS 话题跨进程 (详见 docs/ARCHITECTURE.md, docs/API.md)
#
#   NAV_USE_ROS=1|0               执行进程是否启用 ROS 2/Nav2 (0 = 纯内置导引)
#   SIM_PHYSICS=mujoco|kinematic  仿真引擎 (默认 mujoco；未安装时自动退回 kinematic)
#   SIM_CAMERA_RENDER=ray|gl      相机成像: 光线投射 (默认，无需 GPU) / OpenGL 光栅 (需 EGL，MUJOCO_GL=egl)
#   SIM_CAMERA_MAX_HZ=10          相机帧率上限；SIM_RAY_THREADS 射线并行线程数
#   SIM_API_PORT / NAV_API_PORT / WEB_PORT  端口 (默认 8090 / 8091 / 8088)
#
# 可选环境变量:
#   CMODEL_FILE=/path/xxx.cmodel  启动前重新解析 cmodel → robot_config.json / robot.urdf / nav2 参数
#   CMODEL_LOAD=full|idle         使用满载(默认)或空载运动参数
#   SIM_NOISE=1|0                 传感器/里程计噪声
#   SIM_LOCALIZATION=ground_truth|amcl
#   SIM_USE_SIM_TIME=0|1          仿真时钟 (/clock)
#   NAV2_AUTOSTART=1|0            由 Web 服务按车型自动拉起 Nav2
# ============================================================================
set -e
if [ ! -f /opt/ros/humble/setup.bash ]; then
    # 宿主机(如树莓派 OS)未安装 ROS 2 → 自动转到 Docker 容器中运行
    if command -v docker >/dev/null 2>&1; then
        cd "$(dirname "$0")"
        echo "[info] 本机未安装 ROS 2 Humble，改用 Docker 运行"
        # 两个镜像分离部署 (agv-sim 仿真+Web / agv-nav ROS2+Nav2)，见 docs/DEPLOY.md
        exec bash ./deploy.sh up "$@"
    fi
    echo "[error] 未找到 /opt/ros/humble/setup.bash，且未安装 docker。请按 README 安装 Docker 后重试。"
    exit 1
fi
source /opt/ros/humble/setup.bash

ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT"
export PYTHONUNBUFFERED=1
export SIM_LOCALIZATION="${SIM_LOCALIZATION:-ground_truth}"
export SIM_USE_SIM_TIME="${SIM_USE_SIM_TIME:-0}"
export SIM_API_PORT="${SIM_API_PORT:-8090}"
export NAV_API_PORT="${NAV_API_PORT:-8091}"
export WEB_PORT="${WEB_PORT:-8088}"
export SIM_API="${SIM_API:-http://127.0.0.1:$SIM_API_PORT}"
export NAV_API="${NAV_API:-http://127.0.0.1:$NAV_API_PORT}"
ARCH=rest

echo "=========================================================="
echo " AMR Studio V4 - CModel AGV Simulation & Nav2  [架构: $ARCH]"
echo " Web 调度台 : http://0.0.0.0:$WEB_PORT"
[ "$ARCH" = "rest" ] && echo " 仿真 REST  : http://0.0.0.0:$SIM_API_PORT/api/v1" && echo " 执行 REST  : http://0.0.0.0:$NAV_API_PORT/api/v1"
echo "=========================================================="

# 1. CModel → robot_config.json + robot.urdf (也可运行时 POST /api/v1/model/reload)
if [ -n "$CMODEL_FILE" ]; then
    python3 cmodel_parser.py "$CMODEL_FILE" "$ROOT" --load "${CMODEL_LOAD:-full}" --no-nav2
fi

# 2. 场景栅格地图 + 按车型生成 Nav2 参数
python3 tools/scenario_to_map.py "$ROOT/maps" >/dev/null
python3 tools/gen_nav2_params.py "$ROOT/robot_config.json" "$ROOT/nav2" $([ "$SIM_USE_SIM_TIME" = "1" ] && echo --use-sim-time) >/dev/null
echo "[init] maps/*.yaml 与 nav2/nav2_params_*.yaml 已生成"

PIDS=""
cleanup() { kill $PIDS 2>/dev/null; wait 2>/dev/null; exit 1; }
trap cleanup SIGINT SIGTERM

# 2b. 仿真引擎 MuJoCo (旧镜像未内置时首次启动自动安装；容器 restart 后保留)
if [ "$ARCH" = "rest" ] && ! python3 -c "import mujoco" 2>/dev/null; then
    echo "[init] 安装 MuJoCo 仿真引擎 (首次，约 1~2 分钟)…"
    PIP_PROXY="${PIP_PROXY:-}"
    pip3 install --no-cache-dir -q -i "${PIP_INDEX:-https://pypi.tuna.tsinghua.edu.cn/simple}" "mujoco>=3.3" "numpy<2" pillow \
      || { [ -n "$PIP_PROXY" ] && pip3 install --no-cache-dir -q --proxy "$PIP_PROXY" "mujoco>=3.3" "numpy<2" pillow; } \
      || echo "[warn] MuJoCo 安装失败 → 使用 kinematic 兜底后端 (无相机着色)"
fi
python3 -c "import mujoco; print('[init] MuJoCo', mujoco.__version__)" 2>/dev/null || true

# 3. 端口预检: 清理本架构残留进程；端口仍被其它程序占用 → 自动顺延到空闲端口
port_busy() { (exec 3<>/dev/tcp/127.0.0.1/$1) 2>/dev/null; }
who_uses() { (ss -ltnp 2>/dev/null || netstat -ltnp 2>/dev/null) | grep -E "[:.]$1[[:space:]]" | head -1; }
for pat in "sim_server.api" "nav_runtime.main" "web_gateway.py"; do
    for p in $(pgrep -f "python3 .*$pat" 2>/dev/null); do
        if [ "$p" != "$$" ] && kill "$p" 2>/dev/null; then echo "[init] 结束残留进程 $pat (pid $p)"; fi
    done
done
sleep 0.5
TAKEN=" "
pick_port() {   # $1=变量名 $2=首选端口；跳过被占用及已分配给其它进程的端口
    local p=$2 n=0
    while port_busy "$p" || [[ "$TAKEN" == *" $p "* ]]; do
        port_busy "$p" && echo "[warn] 端口 $p 已被其它程序占用: $(who_uses $p)" >&2
        p=$((p + 1)); n=$((n + 1))
        [ $n -gt 30 ] && { echo "[error] 找不到空闲端口" >&2; exit 1; }
    done
    if [ "$p" != "$2" ]; then echo "[warn] $1: $2 → $p" >&2; fi
    TAKEN="$TAKEN$p "
    eval "$1=$p"
}
pick_port SIM_API_PORT "$SIM_API_PORT"
pick_port NAV_API_PORT "$NAV_API_PORT"
pick_port WEB_PORT "$WEB_PORT"
export SIM_API_PORT NAV_API_PORT WEB_PORT
export SIM_API="http://127.0.0.1:$SIM_API_PORT"
export NAV_API="http://127.0.0.1:$NAV_API_PORT"
echo "[init] 端口: 仿真 $SIM_API_PORT · 执行 $NAV_API_PORT · Web $WEB_PORT"

# 4. 仿真进程 (纯 Python，不加载 ROS)
env -u AMENT_PREFIX_PATH python3 -m sim_server.api --port "$SIM_API_PORT" & SIM_PID=$!
PIDS="$PIDS $SIM_PID"
READY=0
for i in $(seq 1 90); do
    kill -0 $SIM_PID 2>/dev/null || { echo "[error] 仿真进程启动失败"; cleanup; }
    if curl -s "$SIM_API/api/v1/health" 2>/dev/null | grep -q '"service":"sim"'; then READY=1; break; fi
    sleep 0.5
done
[ $READY = 1 ] || { echo "[error] 仿真进程 45 s 内未就绪"; cleanup; }
echo "[init] 仿真进程就绪 (pid $SIM_PID)"

# 5. 执行进程 (ROS 2 + Nav2；robot_state_publisher / Nav2 由其内部监管)
python3 -m nav_runtime.main & NAV_PID=$!
PIDS="$PIDS $NAV_PID"

# 6. Web 网关 (纯 REST 聚合)
python3 "$ROOT/web_gateway.py" & WEB_PID=$!
PIDS="$PIDS $WEB_PID"

# 任一进程退出 → 全部退出 (容器 restart 策略负责拉起)
wait -n $SIM_PID $NAV_PID $WEB_PID || true
echo "[error] 有进程退出，停止全部进程"
cleanup
