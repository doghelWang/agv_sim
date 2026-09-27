#!/bin/bash
# ============================================================================
# 把镜像压成单层 (保留 ENV / WORKDIR / ENTRYPOINT / CMD / EXPOSE / VOLUME / LABEL / HEALTHCHECK)
#   tools/refresh_images.sh 每次在现有镜像上叠一层代码，叠到 ~120 层后 Docker 报 "max depth exceeded"；
#   树莓派 4 GB 上完整重建 nav 镜像 (ROS/Nav2) 很吃力，压平只需几分钟。
#   bash tools/flatten_image.sh agv-nav:latest [agv-sim:latest ...]
# ============================================================================
set -e
for img in "$@"; do
    n=$(docker history -q "$img" | wc -l)
    echo "== 压平 $img ($n 层)"
    df=$(mktemp -d)
    docker image inspect "$img" | python3 -c '
import json, sys
c = json.load(sys.stdin)[0]["Config"]
out = []
for e in c.get("Env") or []:
    k, _, v = e.partition("=")
    out.append("ENV %s=%s" % (k, json.dumps(v)))
if c.get("WorkingDir"):
    out.append("WORKDIR " + c["WorkingDir"])
for p in (c.get("ExposedPorts") or {}):
    out.append("EXPOSE " + p)
for v in (c.get("Volumes") or {}):
    out.append("VOLUME " + json.dumps([v]))
for k, v in (c.get("Labels") or {}).items():
    out.append("LABEL %s=%s" % (json.dumps(k), json.dumps(v)))
h = c.get("Healthcheck")
if h and h.get("Test") and h["Test"][0] != "NONE":
    ns = lambda x: "%ds" % (int(x) // 1000000000)
    opt = " ".join("--%s=%s" % (k, ns(h[f])) for k, f in (("interval", "Interval"), ("timeout", "Timeout"), ("start-period", "StartPeriod")) if h.get(f))
    if h.get("Retries"):
        opt += " --retries=%d" % h["Retries"]
    t = h["Test"]
    out.append("HEALTHCHECK %s CMD %s" % (opt, t[1] if t[0] == "CMD-SHELL" else json.dumps(t[1:])))
if c.get("Entrypoint"):
    out.append("ENTRYPOINT " + json.dumps(c["Entrypoint"]))
if c.get("Cmd"):
    out.append("CMD " + json.dumps(c["Cmd"]))
print("\n".join(out))
' > "$df/config"
    cid=$(docker create "$img")
    docker export "$cid" | docker import - "${img%%:*}:flat-tmp" >/dev/null
    docker rm "$cid" >/dev/null
    { echo "FROM ${img%%:*}:flat-tmp"; cat "$df/config"; } > "$df/Dockerfile"
    docker build -q -t "$img" "$df" >/dev/null
    docker rmi "${img%%:*}:flat-tmp" >/dev/null 2>&1 || true
    rm -rf "$df"
    echo "   → $(docker history -q "$img" | wc -l) 层"
done
