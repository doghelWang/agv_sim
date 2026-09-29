#!/bin/bash
# ============================================================================
# AMR Studio V4 · 双镜像部署脚本 (不依赖 docker compose 插件)
#
#   ./deploy.sh build [sim|nav|all]     构建镜像 agv-sim:latest / agv-nav:latest
#   ./deploy.sh up    [sim|nav|all]     启动 (镜像不存在时自动构建；up --build 强制重建)
#   ./deploy.sh down  [sim|nav|all]     停止并删除容器
#   ./deploy.sh restart [sim|nav|all]
#   ./deploy.sh status                  容器状态 + 两进程 REST 健康检查
#   ./deploy.sh logs  sim|nav           跟踪日志
#   ./deploy.sh save  [sim|nav|all]     导出镜像 (agv-sim.tar.gz / agv-nav.tar.gz)，拷到别的机器后 ./deploy.sh load
#   ./deploy.sh load  [文件...]
#
# 资源管理平台 (多机部署/实例管理/工作台，见 docs/PLATFORM.md):
#   ./deploy.sh hub   [--port 8080] [--no-agent]        本机启动平台 agv-hub，并以集群令牌接入本机节点代理
#   ./deploy.sh agent --hub http://<平台IP>:8080 --token <令牌> [--kind hybrid|controller|sim] [--name 名称]
#   ./deploy.sh build platform                          构建 agv-platform 镜像 (hub/agent 共用)
#
# 分机部署:
#   仿真机: NAV_API=http://<导航机IP>:8091 ./deploy.sh up sim
#   导航机: SIM_API=http://<仿真机IP>:8090 ./deploy.sh up nav
# 环境变量: SIM_API_PORT(8090) NAV_API_PORT(8091) WEB_PORT(8088) ROS_DOMAIN_ID(0)
#          BUILD_PROXY(构建用 HTTP 代理，缺省不用) APT_MIRROR(清华，设为空=官方源) PIP_INDEX(清华，设为空=官方源)
#          BASE_REGISTRY(基础镜像镜像站，如 docker.m.daocloud.io；Docker Hub 拉取失败/限流时用)
#          BUILD_JOBS(nav 镜像 C++ 编译并行度，缺省按构建时可用内存 ≈1.8 GB/路)
#          SIM_PHYSICS SIM_CAMERA_RENDER SIM_CAMERA_MAX_HZ CMODEL_FILE NAV_USE_ROS NAV2_AUTOSTART
# ============================================================================
set -e
cd "$(dirname "$0")"
CMD="${1:-up}"; shift || true
FORCE_BUILD=0
ARGS=()
for a in "$@"; do [ "$a" = "--build" ] && FORCE_BUILD=1 || ARGS+=("$a"); done
WHAT="${ARGS[0]:-all}"
SIM_IMG=agv-sim:latest; NAV_IMG=agv-nav:latest; PLAT_IMG=agv-platform:latest
# 上次自动选定的端口 (环境变量优先)
if [ -f .deploy.env ]; then
    while IFS='=' read -r k v; do [ -n "$k" ] && [ -z "${!k}" ] && export "$k=$v"; done < .deploy.env
fi
SIM_API_PORT="${SIM_API_PORT:-8090}"; NAV_API_PORT="${NAV_API_PORT:-8091}"; WEB_PORT="${WEB_PORT:-8088}"
has() { [ "$WHAT" = "all" ] || [ "$WHAT" = "$1" ]; }
say() { echo -e "\033[1;36m[deploy]\033[0m $*"; }
warn() { echo -e "\033[1;33m[deploy]\033[0m $*"; }

build_args() {
    BUILD_PROXY="${BUILD_PROXY:-}"
    BA=""
    if [ -n "$BUILD_PROXY" ] && curl -s -o /dev/null --max-time 5 -x "$BUILD_PROXY" http://packages.ros.org/ 2>/dev/null; then
        say "构建代理: $BUILD_PROXY"
        BA="--build-arg http_proxy=$BUILD_PROXY --build-arg https_proxy=$BUILD_PROXY --build-arg HTTP_PROXY=$BUILD_PROXY --build-arg HTTPS_PROXY=$BUILD_PROXY"
    fi
    APT_MIRROR="${APT_MIRROR-http://mirrors.tuna.tsinghua.edu.cn}"
    if [ -n "$APT_MIRROR" ] && curl -s -o /dev/null --max-time 5 "$APT_MIRROR/ubuntu-ports/dists/jammy/Release"; then
        BA="$BA --build-arg APT_MIRROR=$APT_MIRROR"
    fi
    BA="$BA --build-arg PIP_INDEX=${PIP_INDEX-https://pypi.tuna.tsinghua.edu.cn/simple}"
    [ -n "$BUILD_JOBS" ] && BA="$BA --build-arg BUILD_JOBS=$BUILD_JOBS"
    # Docker Hub 拉不动 (国内/限流 429) 时用镜像站: BASE_REGISTRY=docker.m.daocloud.io 或 mirror.gcr.io
    if [ -n "$BASE_REGISTRY" ]; then
        say "基础镜像从 $BASE_REGISTRY 拉取"
        BA="$BA --build-arg PY_BASE=$BASE_REGISTRY/library/python:3.10-slim-bookworm --build-arg ROS_BASE=$BASE_REGISTRY/library/ros:humble-ros-base"
    fi
}

build_one() {   # $1 = sim|nav|platform
    build_args
    local img=$SIM_IMG; [ "$1" = nav ] && img=$NAV_IMG; [ "$1" = platform ] && img=$PLAT_IMG
    BA="$BA --build-arg AGV_VERSION=${AGV_VERSION:-$(date +%Y%m%d)-$(git rev-parse --short HEAD 2>/dev/null || echo local)}"
    say "构建 $img (docker/Dockerfile.$1) ..."
    docker build --network host $BA -f "docker/Dockerfile.$1" -t "$img" .
}

img_exists() { docker image inspect "$1" >/dev/null 2>&1; }
port_busy() { (exec 3<>/dev/tcp/127.0.0.1/$1) 2>/dev/null; }

stop_legacy() {
    if docker ps -a --format '{{.Names}}' | grep -qx ros2_agv_sim; then
        warn "停止旧的单容器部署 ros2_agv_sim (与新容器端口冲突)"
        docker rm -f ros2_agv_sim >/dev/null
    fi
}

# ---- 端口占用诊断: 找出占用者；本项目的残留实例自动清理，其它程序则自动顺延到空闲端口 (记录到 .deploy.env)
port_owner() {  # 输出: "pid|进程名|命令行|容器名"
    local port=$1 line pid="" cmd="" ctr=""
    line=$( (sudo -n ss -ltnpH "sport = :$port" 2>/dev/null || ss -ltnpH "sport = :$port" 2>/dev/null) | head -1)
    pid=$(echo "$line" | grep -oE 'pid=[0-9]+' | head -1 | cut -d= -f2)
    if [ -n "$pid" ]; then
        cmd=$(tr '\0' ' ' < /proc/$pid/cmdline 2>/dev/null | cut -c1-160)
        local cid; cid=$(grep -oE '[0-9a-f]{64}' /proc/$pid/cgroup 2>/dev/null | head -1)
        [ -n "$cid" ] && ctr=$(docker ps --no-trunc --format '{{.ID}} {{.Names}}' | awk -v c="$cid" '$1==c{print $2}')
    fi
    if [ -z "$ctr" ]; then   # 无 root 时: 在 host 网络容器里找监听该端口的进程
        for c in $(docker ps --filter network=host --format '{{.Names}}'); do
            if docker exec "$c" sh -c "cat /proc/net/tcp /proc/net/tcp6 2>/dev/null" 2>/dev/null | awk '{print $2, $4}' | grep -qiE ":$(printf '%04X' "$port") 0A"; then
                if docker top "$c" -eo args 2>/dev/null | grep -qE "sim_server|web_gateway|web_teleop|agv_simulation|nav_runtime|http.server|--port $port"; then ctr=$c; break; fi
            fi
        done
    fi
    echo "${pid}|${cmd}|${ctr}"
}

is_ours() {  # 本项目进程/容器特征
    echo "$1" | grep -qE "sim_server|web_gateway|web_teleop_server|agv_simulation|nav_runtime|ros2_agv|agv-sim|agv-nav|ros2_cmodel_agv"
}

free_port() {  # 下一个空闲且未分配给其它服务的端口 ($1 起始 $2 本变量名)
    local p=$1 self=$2
    while port_busy "$p" || { [ "$self" != SIM_API_PORT ] && [ "$p" = "$SIM_API_PORT" ]; } \
          || { [ "$self" != NAV_API_PORT ] && [ "$p" = "$NAV_API_PORT" ]; } || { [ "$self" != WEB_PORT ] && [ "$p" = "$WEB_PORT" ]; }; do
        p=$((p + 1))
    done
    echo "$p"
}

check_port() {  # $1 端口变量名 $2 本容器名 ；可能修改该变量
    local var=$1 self=$2 port=${!1}
    port_busy "$port" || return 0
    docker ps --format '{{.Names}}' | grep -qx "$self" && return 0
    local info pid cmd ctr health
    info=$(port_owner "$port"); pid=${info%%|*}; ctr=${info##*|}; cmd=${info#*|}; cmd=${cmd%|*}
    health=$(curl -s --max-time 2 "http://127.0.0.1:$port/api/v1/health" 2>/dev/null | tr -d '\n' | grep -oE '^\{.{0,110}' || true)
    warn "端口 $port ($var) 被占用: ${ctr:+容器 $ctr }${pid:+PID $pid }${cmd:-（无权限查看进程，可用 sudo ss -ltnp 查看）}"
    [ -n "$health" ] && warn "  该端口响应: $health"
    if [ -n "$ctr" ] && is_ours "$ctr $cmd"; then
        warn "  → 本项目的旧容器 $ctr，自动停止"; docker rm -f "$ctr" >/dev/null; sleep 1
    elif [ -n "$pid" ] && is_ours "$cmd" && kill "$pid" 2>/dev/null; then
        warn "  → 本项目的残留进程 PID $pid，已结束"; sleep 1
    elif echo "$health" | grep -q '"service":"\(sim\|nav\)"' && [ -z "$pid" ]; then
        warn "  → 疑似本项目的残留实例 (无权限结束)，请执行: sudo ss -ltnp | grep :$port  然后 sudo kill <PID>"
    fi
    if port_busy "$port"; then
        local np; np=$(free_port $((port + 1)) "$var")
        warn "  → 该端口由其它程序使用，$var 改用 $np (已记录到 .deploy.env，后续命令沿用)"
        printf -v "$var" '%s' "$np"
        save_env
    fi
}

save_env() {
    printf 'SIM_API_PORT=%s\nNAV_API_PORT=%s\nWEB_PORT=%s\n' "$SIM_API_PORT" "$NAV_API_PORT" "$WEB_PORT" > .deploy.env
}

run_sim() {
    # 前端页面直接挂载仓库文件 (改页面只需刷新浏览器，无需重建镜像)；AGV_WEB_MOUNT=0 则使用镜像内置页面
    WEB_MOUNTS=""
    if [ "${AGV_WEB_MOUNT:-1}" = "1" ]; then
        for f in index.html model_editor.html vendor; do
            [ -e "$f" ] && WEB_MOUNTS="$WEB_MOUNTS -v $(pwd)/$f:/opt/agv/$f:ro"
        done
    fi
    docker rm -f agv-sim >/dev/null 2>&1 || true
    check_port SIM_API_PORT agv-sim; check_port WEB_PORT agv-sim
    mkdir -p data
    docker run -d --name agv-sim --restart unless-stopped --network host \
        -v "$(pwd)/data:/data" $WEB_MOUNTS \
        -e SIM_API_PORT="$SIM_API_PORT" -e WEB_PORT="$WEB_PORT" \
        -e NAV_API="${NAV_API:-http://127.0.0.1:$NAV_API_PORT}" \
        -e SIM_PHYSICS="${SIM_PHYSICS:-mujoco}" -e SIM_CAMERA_RENDER="${SIM_CAMERA_RENDER:-ray}" \
        -e SIM_CAMERA_MAX_HZ="${SIM_CAMERA_MAX_HZ:-10}" -e CMODEL_FILE="${CMODEL_FILE:-}" \
        "$SIM_IMG" >/dev/null
    say "agv-sim 已启动  仿真 REST :$SIM_API_PORT   Web :$WEB_PORT   数据目录 $(pwd)/data"
}

run_nav() {
    docker rm -f agv-nav >/dev/null 2>&1 || true
    check_port NAV_API_PORT agv-nav
    mkdir -p data
    docker run -d --name agv-nav --restart unless-stopped --network host -v "$(pwd)/data:/data" \
        -e NAV_LOCALIZATION="${NAV_LOCALIZATION:-slam}" -e LOC_ENGINE="${LOC_ENGINE:-auto}" \
        -e ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}" \
        -e SIM_API="${SIM_API:-http://127.0.0.1:$SIM_API_PORT}" -e NAV_API_PORT="$NAV_API_PORT" \
        -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" -e NAV_USE_ROS="${NAV_USE_ROS:-1}" \
        -e NAV2_AUTOSTART="${NAV2_AUTOSTART:-1}" -e SIM_LOCALIZATION="${SIM_LOCALIZATION:-slam}" \
        "$NAV_IMG" >/dev/null
    say "agv-nav 已启动  执行 REST :$NAV_API_PORT   → 仿真 ${SIM_API:-http://127.0.0.1:$SIM_API_PORT}"
}

ctr_env() {  # $1 容器 $2 变量名 → 值
    docker inspect --type container -f '{{range .Config.Env}}{{println .}}{{end}}' "$1" 2>/dev/null | sed -n "s/^$2=//p" | head -1
}

sync_peers() {
    local running; running=$(docker ps --format '{{.Names}}')
    if echo "$running" | grep -qx agv-sim && [ -z "$NAV_API" ]; then
        local want="http://127.0.0.1:$NAV_API_PORT" have; have=$(ctr_env agv-sim NAV_API)
        if echo "$running" | grep -qx agv-nav && [ "$have" != "$want" ]; then
            warn "agv-sim 中的执行进程地址 $have ≠ $want，重建 agv-sim"; run_sim
        fi
    fi
    if echo "$running" | grep -qx agv-nav && [ -z "$SIM_API" ]; then
        local want="http://127.0.0.1:$SIM_API_PORT" have; have=$(ctr_env agv-nav SIM_API)
        if echo "$running" | grep -qx agv-sim && [ "$have" != "$want" ]; then
            warn "agv-nav 中的仿真进程地址 $have ≠ $want，重建 agv-nav"; run_nav
        fi
    fi
}

# ---------------------------------------------------------------- 平台 (hub) 与节点代理 (agent)
opt() {  # $1 选项名 → 值 (从 ARGS 中取 --name value)
    local i
    for ((i = 0; i < ${#ARGS[@]}; i++)); do [ "${ARGS[$i]}" = "$1" ] && { echo "${ARGS[$((i + 1))]}"; return; }; done
}
flag() { local a; for a in "${ARGS[@]}"; do [ "$a" = "$1" ] && return 0; done; return 1; }
free_simple() { local p=$1; while port_busy "$p"; do p=$((p + 1)); done; echo "$p"; }

run_hub() {
    img_exists $PLAT_IMG && [ $FORCE_BUILD = 0 ] || build_one platform
    HUB_PORT="$(opt --port)"; HUB_PORT="${HUB_PORT:-${HUB_PORT_ENV:-8080}}"
    docker rm -f agv-hub >/dev/null 2>&1 || true
    if port_busy "$HUB_PORT"; then local np; np=$(free_simple "$HUB_PORT"); warn "端口 $HUB_PORT 被占用，平台改用 $np"; HUB_PORT=$np; fi
    local data="${AGV_HUB_DATA:-$HOME/.agv-hub}"; mkdir -p "$data"
    local mounts=""
    [ "${AGV_WEB_MOUNT:-1}" = "1" ] && for f in web vendor model_editor.html; do [ -e "$f" ] && mounts="$mounts -v $(pwd)/$f:/opt/agv/$f:ro"; done
    docker run -d --name agv-hub --restart unless-stopped --network host -v "$data:/data/hub" $mounts \
        -e HUB_PORT="$HUB_PORT" "$PLAT_IMG" hub >/dev/null
    for i in $(seq 1 40); do curl -fsS --max-time 2 "http://127.0.0.1:$HUB_PORT/api/hub/health" >/dev/null 2>&1 && break; sleep 0.5; done
    say "资源管理平台已启动: http://$(hostname -I 2>/dev/null | awk '{print $1}'):$HUB_PORT   数据 $data"
    printf 'HUB_PORT_ENV=%s\n' "$HUB_PORT" > .hub.env
    if ! flag --no-agent; then
        local tok; tok=$(cat "$data/cluster_token" 2>/dev/null)
        [ -n "$tok" ] && run_agent "http://127.0.0.1:$HUB_PORT" "$tok" "${AGENT_KIND:-hybrid}" "$(opt --name)"
    fi
}

run_agent() {   # $1 平台地址 $2 令牌 $3 类型 $4 名称
    img_exists $PLAT_IMG || build_one platform
    local data="${AGV_AGENT_DATA:-$HOME/.agv-agent}"; mkdir -p "$data"
    local port="${AGENT_PORT:-8070}"
    docker rm -f agv-agent >/dev/null 2>&1 || true
    port_busy "$port" && { port=$(free_simple "$port"); warn "代理端口改用 $port"; }
    docker run -d --name agv-agent --restart unless-stopped --network host \
        -v /var/run/docker.sock:/var/run/docker.sock -v "$data:$data" \
        -e AGENT_DATA="$data" -e HUB_API="$1" -e JOIN_TOKEN="$2" -e AGENT_KIND="${3:-hybrid}" \
        -e AGENT_NAME="${4:-$(hostname)}" -e AGENT_PORT="$port" -e AGENT_RUNTIME=docker \
        -e AGENT_PORT_RANGE="${AGENT_PORT_RANGE:-8100-8199}" ${AGENT_HOST:+-e AGENT_HOST=$AGENT_HOST} "$PLAT_IMG" agent >/dev/null
    say "节点代理已启动 (:$port) → 平台 $1"
}

health() {  # $1 名称 $2 url
    if curl -fsS --max-time 3 "$2" >/dev/null 2>&1; then echo "  ✔ $1  $2"; else echo "  ✘ $1  $2"; fi
}

case "$CMD" in
    build)
        if [ "$WHAT" = platform ]; then build_one platform; else
        has sim && build_one sim
        has nav && build_one nav
        [ "$WHAT" = all ] && build_one platform; fi
        ;;
    hub|platform)
        [ -f .hub.env ] && . ./.hub.env
        run_hub
        ;;
    agent)
        HUB="$(opt --hub)"; TOK="$(opt --token)"
        [ -z "$HUB" ] || [ -z "$TOK" ] && { warn "用法: bash deploy.sh agent --hub http://<平台IP>:8080 --token <令牌> [--kind hybrid|controller|sim] [--name 名称]"; exit 1; }
        run_agent "$HUB" "$TOK" "$(opt --kind)" "$(opt --name)"
        ;;
    up)
        stop_legacy
        # 1) 先构建 (构建可能耗时几十分钟，期间端口占用会变化，所以端口必须在构建之后再确定)
        if has sim; then { [ $FORCE_BUILD = 1 ] || ! img_exists $SIM_IMG; } && build_one sim; fi
        if has nav; then { [ $FORCE_BUILD = 1 ] || ! img_exists $NAV_IMG; } && build_one nav; fi
        # 2) 统一确定端口 (同机部署时两个容器互相引用对方地址)
        has sim && docker rm -f agv-sim >/dev/null 2>&1; has nav && docker rm -f agv-nav >/dev/null 2>&1
        has sim && { check_port SIM_API_PORT agv-sim; check_port WEB_PORT agv-sim; }
        has nav && check_port NAV_API_PORT agv-nav
        # 3) 启动
        has sim && run_sim
        has nav && run_nav
        # 4) 同机时校验两容器互相引用的地址，不一致则重建对方 (例如只重启了其中一个且端口有变)
        sync_peers
        say "查看状态: bash deploy.sh status    日志: bash deploy.sh logs sim|nav"
        ;;
    down)
        has sim && docker rm -f agv-sim >/dev/null 2>&1 && say "agv-sim 已停止" || true
        has nav && docker rm -f agv-nav >/dev/null 2>&1 && say "agv-nav 已停止" || true
        ;;
    restart)
        has sim && docker restart agv-sim
        has nav && docker restart agv-nav
        ;;
    status)
        docker ps -a --filter name=agv- --format 'table {{.Names}}\t{{.Image}}\t{{.Status}}'
        [ -f .hub.env ] && . ./.hub.env && health "资源管理平台" "http://127.0.0.1:${HUB_PORT_ENV:-8080}/api/hub/health"
        docker ps --format '{{.Names}}' | grep -qx agv-agent && health "节点代理" "http://127.0.0.1:${AGENT_PORT:-8070}/api/v1/health"
        if docker ps -a --format '{{.Names}}' | grep -qx agv-sim; then     # 单机方式 (deploy.sh up) 的两容器
            health "仿真进程" "http://127.0.0.1:$SIM_API_PORT/api/v1/health"
            health "Web 网关" "http://127.0.0.1:$WEB_PORT/"
        fi
        docker ps -a --format '{{.Names}}' | grep -qx agv-nav && health "执行进程" "http://127.0.0.1:$NAV_API_PORT/api/v1/health"
        n=$(docker ps --format '{{.Names}}' | grep -cE '^agv-(sim|nav)-i') || true
        [ "${n:-0}" -gt 0 ] && echo "  平台实例容器: $(docker ps --format '{{.Names}}' | grep -E '^agv-(sim|nav)-i' | tr '\n' ' ')"
        a=$(ctr_env agv-sim NAV_API); b=$(ctr_env agv-nav SIM_API)
        [ -n "$a" ] && health "网关→执行进程 (agv-sim 的 NAV_API)" "$a/api/v1/health"
        [ -n "$b" ] && health "执行→仿真进程 (agv-nav 的 SIM_API)" "$b/api/v1/health"
        ;;
    logs)
        docker logs -f --tail 200 "agv-${WHAT/all/sim}"
        ;;
    save)
        [ "$WHAT" = platform ] && { docker save $PLAT_IMG | gzip > agv-platform.tar.gz; ls -lh agv-platform.tar.gz; exit 0; }
        has sim && { say "导出 agv-sim.tar.gz"; docker save $SIM_IMG | gzip > agv-sim.tar.gz; }
        has nav && { say "导出 agv-nav.tar.gz"; docker save $NAV_IMG | gzip > agv-nav.tar.gz; }
        ls -lh agv-*.tar.gz
        ;;
    load)
        files=("${ARGS[@]}"); [ ${#files[@]} -eq 0 ] && files=(agv-*.tar.gz)
        for f in "${files[@]}"; do say "导入 $f"; gunzip -c "$f" | docker load; done
        ;;
    *)
        sed -n "2,26p" "$0"; exit 1
        ;;
esac
exit 0
