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
    { echo "FROM agv-$k:latest"; awk '/^WORKDIR/{f=1} f' "docker/Dockerfile.$k"; } > "$f"
    echo "== 刷新 agv-$k:latest ($(git log --oneline -1 2>/dev/null || cat .agv_version 2>/dev/null))"
    docker build -q -f "$f" --label "agv.code=$(git rev-parse --short HEAD 2>/dev/null)" -t "agv-$k:latest" . || { rm -f "$f"; exit 1; }
    rm -f "$f"
done
docker image prune -f >/dev/null 2>&1
