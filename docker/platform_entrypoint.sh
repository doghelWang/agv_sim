#!/bin/bash
# agv-platform 入口: hub (资源管理平台) | agent (计算节点代理)
cd /opt/agv
case "${1:-hub}" in
    hub)   exec python3 -m hub.server ;;
    agent) exec python3 -m agent.server ;;
    *)     exec "$@" ;;
esac
