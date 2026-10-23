#!/bin/bash
# agv-nav 入口 (Rust 版, 纯导引模式): nav-runtime (:NAV_API_PORT)，等待仿真进程 REST 就绪由 nav-runtime 自己完成。
# 节点代理的 process 运行时在 nav_entrypoint_rs.sh 存在时优先使用它 (AGENT_NAV_PYTHON=1 退回 Python 版 nav_entrypoint.sh，含 Nav2/SLAM)
cd "${AGV_HOME:-/opt/agv}"
export SIM_API="${SIM_API:-http://127.0.0.1:8090}"
if [ -f "$HOME/.agv-agent/nav.env" ]; then set -a; . "$HOME/.agv-agent/nav.env"; set +a; fi
exec /opt/agv/bin/nav-runtime
