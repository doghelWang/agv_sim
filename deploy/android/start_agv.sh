#!/data/data/com.termux/files/usr/bin/bash
# ============================================================================
# 启动手机上的 AMR 仿真节点 (Termux 里执行: bash ~/start_agv.sh)
#   1. proot 派生服务 :8069 (Termux 原生 Python；每个仿真/执行/ROS 进程独立 proot 会话)
#   2. 资源平台 agv-hub :AGV_HUB_PORT (只有未配置 HUB_API，即手机自己当平台时)
#   3. 节点代理 agv-agent :8070 (process 运行时)，接入本机平台或 HUB_API 指定的主平台
#   4. AGV_AUTOSTART=1 时自动拉起一个实例 (deploy/android/autostart.py)
#   另: 有 ~/gpucastd 时先启动 GPU 射线求交服务 :8068 (sim_core/native/gpucast)
# 其它情况下实例 (仿真/执行进程) 由平台部署，不在这里启动。配置见 ~/.agv.env (agv_common.sh)
# ============================================================================
. ~/agv_common.sh
echo "=== AMR 仿真节点启动 ($(getprop ro.product.model 2>/dev/null)) ==="
termux-wake-lock 2>/dev/null || true
# 先让 Termux 成为前台应用: 三星系统把不在前台的 Termux 限制到 3 个小核 (cpuset moderate)，在小核上启动时 Nav2 的 TF 监听容易卡死。
# 装了外屏状态面板 (deploy/android/cover_app) 就先打开它: 面板点亮屏幕并保持常亮，把 Termux 调到前台，自己悬浮在上面，
# 平台起来之前显示"等待中"。三星不允许从外屏上的应用直接打开别的应用页面，所以发广播让面板应用自己处理。AGV_COVER_PANEL=0 不打开面板
if [ "${AGV_COVER_PANEL:-1}" = 1 ] && pm path com.agvsim.cover >/dev/null 2>&1; then
    am broadcast -n com.agvsim.cover/.StartReceiver >/dev/null 2>&1 && echo "[info] 外屏状态面板已打开 (长按面板让出屏幕，点右下角小按钮才回来；点过「退出」则不会打开，点应用图标重新开启)"
    for i in $(seq 1 15); do     # 等 Termux 真正到前台 (屏幕灭着时面板要先点亮屏幕)，最多 15 秒
        [ "$(cat /proc/self/cpuset 2>/dev/null)" = /top-app ] && break; sleep 1
    done
else
    am start -n com.termux/.app.TermuxActivity >/dev/null 2>&1 || true
fi
# 前台看守 (keep_front.sh): 离开前台约 30 秒后自动把面板和 Termux 调回前台。AGV_KEEP_FRONT=0 关闭 (要在手机上长时间用别的应用时)
if [ "${AGV_KEEP_FRONT:-1}" = 1 ] && [ "${AGV_COVER_PANEL:-1}" = 1 ] && [ -f ~/keep_front.sh ] && pm path com.agvsim.cover >/dev/null 2>&1 \
        && ! kill -0 "$(cat ~/.agv_front.pid 2>/dev/null)" 2>/dev/null; then
    nohup bash ~/keep_front.sh >/dev/null 2>&1 < /dev/null &
    echo $! > ~/.agv_front.pid
    echo "[info] 前台看守已启动 (离开前台约 30 秒后自动调回；AGV_KEEP_FRONT=0 关闭)"
fi
echo "[info] 可用 CPU 核: $(grep Cpus_allowed_list /proc/self/status | awk '{print $2}') (cpuset $(cat /proc/self/cpuset 2>/dev/null))"
[ -x ~/.agv_prestart.sh ] && ~/.agv_prestart.sh          # 可选: 自定义前置 (代理等)

[ -d "$AGV_CODE" ] || { echo "[错误] 没有找到 $AGV_CODE，请先运行 install.sh"; exit 1; }

# ---- 0. GPU 射线求交服务 (可选): ~/gpucastd 存在就启动 (update_from_git.sh 用 Termux 的 cc 编译)。
#         仿真进程启动后自动探测 127.0.0.1:8068，相机/深度这类大批量射线交给 GPU；没有 GPU 或启动失败时仿真照常用 CPU。
#         厂商 OpenCL 库要放在 LD_LIBRARY_PATH 里才允许加载；AGV_GPU_CAST=0 关闭
if [ "${AGV_GPU_CAST:-1}" != 0 ] && [ -x ~/gpucastd ] && ! pgrep -f "^([^ ]*/)?gpucastd 8068" >/dev/null; then
    LD_LIBRARY_PATH=/vendor/lib64 nohup ~/gpucastd 8068 > ~/gpucastd.log 2>&1 < /dev/null &
    sleep 0.5
    pgrep -f "^([^ ]*/)?gpucastd 8068" >/dev/null && echo "[info] GPU 射线求交服务 :8068 已启动 ($(sed -n 's/.*设备 \(.*\)，监听.*/\1/p' ~/gpucastd.log))" \
        || echo "[info] GPU 射线求交服务没有启动 (见 ~/gpucastd.log)，仿真使用 CPU 求交"
fi

# 上次没停干净留下的残留进程 (见 agv_common.sh: agv_orphans) 先清掉，否则它们占着的端口会让新实例的 Nav2 起不来
O=$(agv_orphans); [ -n "$O" ] && { kill -9 $O 2>/dev/null; echo "[info] 清理上次残留的进程 $(echo $O | wc -w) 个"; }

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
        AGENT_RUNTIME=process AGENT_NAME='${AGENT_NAME:-phone}' \
        AGV_CPUS_SIM='$AGV_CPUS_SIM' AGV_CPUS_NAV='$AGV_CPUS_NAV' AGV_CPUS_WEB='$AGV_CPUS_WEB' OPENBLAS_NUM_THREADS='${OPENBLAS_NUM_THREADS:-1}' AGV_DEVICE_MODEL='$MODEL' AGENT_PORT=8070 python3 -m agent.server" \
        > ~/agent.log 2>&1 < /dev/null &
    echo "[info] 节点代理 :8070 启动中${AGV_CPUS_SIM:+ (仿真绑核 $AGV_CPUS_SIM，网关 ${AGV_CPUS_WEB:-不绑定})}"
fi
for i in $(seq 1 30); do http_ok http://127.0.0.1:8070/api/v1/health && break; sleep 1; done

# ---- 4. 开机自动拉起实例 (AGV_AUTOSTART=1): 重启本机最近的实例 / 没有就新部署，仿真 + 执行都在本机
if [ "${AGV_AUTOSTART:-0}" = 1 ]; then
    setsid nohup python3 "$AGV_CODE/deploy/android/autostart.py" --hub "$HUB" --node "${AGENT_NAME:-phone}" \
        > ~/autostart.log 2>&1 < /dev/null &
    echo "[info] 自动拉起实例中 (约 2~5 分钟)，进度: tail -f ~/autostart.log，完成后地址写在 ~/agv_url.txt"
fi

echo "==================================================="
if remote_hub; then
    echo " 节点 ${AGENT_NAME:-phone} ($(wlan_ip):8070) → 主平台 $HUB_API"
    echo " 在主平台「部署仿真」里选择本节点即可"
else
    echo " 资源平台: http://$(wlan_ip):$AGV_HUB_PORT"
fi
echo " 状态: bash ~/status_agv.sh    停止: bash ~/stop_agv.sh"
echo "==================================================="
