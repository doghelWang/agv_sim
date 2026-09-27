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
for k in $KINDS; do
    docker image inspect "agv-$k:latest" >/dev/null 2>&1 || { echo "没有 agv-$k:latest，请先 deploy.sh build"; exit 1; }
    f=$(mktemp)
    # 多阶段构建的辅助阶段 (# @stage-begin … # @stage-end，如 sim 的 C 内核编译) + 基于现有镜像的代码层
    { grep -m1 '^ARG PY_BASE' "docker/Dockerfile.$k"; awk '/^# @stage-begin/{f=1} /^# @stage-end/{f=0} f' "docker/Dockerfile.$k"
      echo "FROM agv-$k:latest"; awk '/^WORKDIR/{f=1} f' "docker/Dockerfile.$k"; } > "$f"
    echo "== 刷新 agv-$k:latest ($(git log --oneline -1 2>/dev/null || cat .agv_version 2>/dev/null))"
    docker build -q --network host --build-arg APT_MIRROR="${APT_MIRROR-http://mirrors.tuna.tsinghua.edu.cn}" ${BUILD_PROXY:+--build-arg http_proxy=$BUILD_PROXY --build-arg https_proxy=$BUILD_PROXY} -f "$f" --label "agv.code=$(git rev-parse --short HEAD 2>/dev/null)" -t "agv-$k:latest" . || { rm -f "$f"; exit 1; }
    rm -f "$f"
done
docker image prune -f >/dev/null 2>&1
