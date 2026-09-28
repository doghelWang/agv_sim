#!/bin/bash
# ============================================================================
# 只刷新 agv-sim / agv-nav 镜像里的代码层 (基于已构建的镜像，不重装 ROS/依赖)
#   树莓派 4 GB 上完整重建会把内存吃光；这个脚本几十秒完成。
#   bash tools/refresh_images.sh            # 两个都刷新
#   bash tools/refresh_images.sh nav        # 只刷新执行镜像
# 每次刷新叠一层，偶尔 (依赖有变化时) 仍需 deploy.sh build 完整重建。
# ============================================================================
cd "$(dirname "$0")/.." || exit 1
KINDS="${*:-sim nav}"
# sim 的 C 内核编译阶段用固定的 builder 镜像 (agv-sim + gcc，按 MuJoCo 版本打标签)：apt 层与编译层都能命中缓存，
# C 源码不变时不重新编译 (否则每次以新的 agv-sim:latest 为基础都要重装 gcc，约 1 分钟)
ensure_builder() {
    local mj
    mj=$(docker run --rm --entrypoint python agv-sim:latest -c "import mujoco; print(mujoco.__version__)" 2>/dev/null)
    if [ "$(docker image inspect -f '{{index .Config.Labels "agv.mujoco"}}' agv-simcore-builder:latest 2>/dev/null)" != "$mj" ]; then
        echo "== 构建 C 内核编译镜像 agv-simcore-builder (mujoco $mj)"
        printf 'FROM agv-sim:latest\nARG APT_MIRROR=""\nRUN if [ -n "$APT_MIRROR" ]; then sed -i "s|http://deb.debian.org|$APT_MIRROR|g" /etc/apt/sources.list.d/*.sources /etc/apt/sources.list 2>/dev/null; fi; apt-get update && apt-get install -y --no-install-recommends gcc libc6-dev && rm -rf /var/lib/apt/lists/*\nLABEL agv.mujoco=%s\n' "$mj" \
            | docker build -q --network host --build-arg APT_MIRROR="${APT_MIRROR-http://mirrors.tuna.tsinghua.edu.cn}" -t agv-simcore-builder:latest - >/dev/null
    fi
}
for k in $KINDS; do
    docker image inspect "agv-$k:latest" >/dev/null 2>&1 || { echo "没有 agv-$k:latest，请先 deploy.sh build"; exit 1; }
    # 每次刷新叠一层，超过 100 层先压平 (Docker 上限约 125 层，否则报 max depth exceeded)
    [ "$(docker history -q "agv-$k:latest" | wc -l)" -gt 100 ] && bash tools/flatten_image.sh "agv-$k:latest"
    f=$(mktemp)
    # 多阶段构建的辅助阶段 (# @stage-begin … # @stage-end，如 sim 的 C 内核编译) + 基于现有镜像的代码层
    { awk '/^FROM/{exit} /^ARG /' "docker/Dockerfile.$k"; awk '/^# @stage-begin/{f=1} /^# @stage-end/{f=0} f' "docker/Dockerfile.$k"
      echo "FROM agv-$k:latest"; awk '/^WORKDIR/{f=1} f' "docker/Dockerfile.$k"; } > "$f"
    echo "== 刷新 agv-$k:latest ($(git log --oneline -1 2>/dev/null || cat .agv_version 2>/dev/null))"
    B="agv-$k:latest"
    [ "$k" = sim ] && ensure_builder && B=agv-simcore-builder:latest
    docker build -q --network host --build-arg SIMCORE_BASE="$B" --build-arg APT_MIRROR="${APT_MIRROR-http://mirrors.tuna.tsinghua.edu.cn}" ${BUILD_PROXY:+--build-arg http_proxy=$BUILD_PROXY --build-arg https_proxy=$BUILD_PROXY} -f "$f" --label "agv.code=$(git rev-parse --short HEAD 2>/dev/null)" -t "agv-$k:latest" . || { rm -f "$f"; exit 1; }
    rm -f "$f"
done
docker image prune -f >/dev/null 2>&1
