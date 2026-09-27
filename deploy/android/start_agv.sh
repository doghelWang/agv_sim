#!/data/data/com.termux/files/usr/bin/bash
# ============================================================================
# 启动手机上的 AMR 仿真节点 (Termux 里执行: bash ~/start_agv.sh)
#   1. proot 派生服务 :8069 (Termux 原生 Python；每个仿真/执行/ROS 进程独立 proot 会话)
#   2. 资源平台 agv-hub :AGV_HUB_PORT (只有未配置 HUB_API，即手机自己当平台时)
#   3. 节点代理 agv-agent :8070 (process 运行时)，接入本机平台或 HUB_API 指定的主平台
# 实例 (仿真/执行进程) 由平台部署，不在这里启动。配置见 ~/.agv.env (agv_common.sh)
# ============================================================================
. ~/agv_common.sh
echo "=== AMR 仿真节点启动 ($(getprop ro.product.model 2>/dev/null)) ==="
termux-wake-lock 2>/dev/null || true
[ -x ~/.agv_prestart.sh ] && ~/.agv_prestart.sh          # 可选: 自定义前置 (代理等)

[ -d "$AGV_CODE" ] || { echo "[错误] 没有找到 $AGV_CODE，请先运行 install.sh"; exit 1; }

# ---- 1. proot 派生服务
if [ "${AGV_SPAWNER_OFF:-0}" != 1 ] && ! http_ok http://127.0.0.1:8069/health; then
    SPAWNER_DISTRO="$AGV_DISTRO" nohup python3 ~/proot_spawner.py > ~/spawner.log 2>&1 < /dev/null &
    echo "[info] proot 派生服务 :8069 已启动"
    sleep 0.5
fi
SPAWNER_ENV=""
[ "${AGV_SPAWNER_OFF:-0}" != 1 ] && SPAWNER_ENV="AGV_SPAWNER=http://127.0.0.1:8069"

# ---- 2. 资源平台 (本机当平台时)
if remote_hub; then
    echo "[info] 作为计算节点接入主平台 $HUB_API (不启动本机平台)"
    HUB="$HUB_API"; TOKEN="$JOIN_TOKEN"; ADV="AGENT_HOST=${AGENT_HOST:-$(wlan_ip)}"
else
    if ! http_ok "http://127.0.0.1:$AGV_HUB_PORT/api/hub/health"; then
        nohup proot-distro login "$AGV_DISTRO" -- bash -c "cd /opt/agv && HUB_PORT=$AGV_HUB_PORT python3 -m hub.server" > ~/hub.log 2>&1 < /dev/null &
        echo "[info] 资源平台 :$AGV_HUB_PORT 启动中"
        for i in $(seq 1 60); do http_ok "http://127.0.0.1:$AGV_HUB_PORT/api/hub/health" && break; sleep 1; done
    fi
    HUB="http://127.0.0.1:$AGV_HUB_PORT"; ADV=""
    TOKEN=$(cat "$AGV_ROOTFS/root/.agv-hub/cluster_token" 2>/dev/null)     # 平台首次启动时生成
fi

# ---- 3. 节点代理
if ! http_ok http://127.0.0.1:8070/api/v1/health; then
    MODEL="${AGV_DEVICE_MODEL:-$(getprop ro.product.model 2>/dev/null) ($(getprop ro.soc.model 2>/dev/null))}"
    nohup proot-distro login "$AGV_DISTRO" -- bash -c "cd /opt/agv && $SPAWNER_ENV $ADV HUB_API='$HUB' JOIN_TOKEN='$TOKEN' \
        AGENT_RUNTIME=process AGENT_NAME='${AGENT_NAME:-phone}' AGV_DEVICE_MODEL='$MODEL' AGENT_PORT=8070 python3 -m agent.server" \
        > ~/agent.log 2>&1 < /dev/null &
    echo "[info] 节点代理 :8070 启动中"
fi
for i in $(seq 1 30); do http_ok http://127.0.0.1:8070/api/v1/health && break; sleep 1; done

echo "==================================================="
if remote_hub; then
    echo " 节点 ${AGENT_NAME:-phone} ($(wlan_ip):8070) → 主平台 $HUB_API"
    echo " 在主平台「部署仿真」里选择本节点即可"
else
    echo " 资源平台: http://$(wlan_ip):$AGV_HUB_PORT"
fi
echo " 状态: bash ~/status_agv.sh    停止: bash ~/stop_agv.sh"
echo "==================================================="
