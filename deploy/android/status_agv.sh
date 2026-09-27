#!/data/data/com.termux/files/usr/bin/bash
# 手机节点状态: 派生服务 / 平台 / 节点代理 / 运行中的实例进程 / 温度与频率
. ~/agv_common.sh
echo "==================================================="
echo " AMR 仿真节点状态  $(getprop ro.product.model 2>/dev/null)  容器 $AGV_DISTRO  代码 $(cat "$AGV_CODE/.agv_version" 2>/dev/null)"
echo "==================================================="
python3 - "$AGV_HUB_PORT" "${HUB_API:-}" <<'EOF'
import json, sys, urllib.request
port, hub = sys.argv[1], sys.argv[2]
def get(u):
    try:
        with urllib.request.urlopen(u, timeout=3) as r:
            return json.loads(r.read() or b"{}")
    except Exception:
        return None
sp = get("http://127.0.0.1:8069/list")
print("派生服务 :8069   ", "运行中" if sp is not None else "未运行")
if sp:
    for s in sp["sessions"]:
        if s["running"]:
            print(f"   {s['name']:<48} 进程 {s['tracees']}")
h = get(f"http://127.0.0.1:{port}/api/hub/health")
print(f"本机平台 :{port}    ", ("运行中" if h else "未运行") + (f" (节点接入主平台 {hub})" if hub else ""))
a = get("http://127.0.0.1:8070/api/v1/health")
print("节点代理 :8070    ", "运行中" if a else "未运行", json.dumps(a, ensure_ascii=False)[:160] if a else "")
EOF
echo "---------------------------------------------------"
echo "CPU 频率 (MHz): $(for c in /sys/devices/system/cpu/cpu[0-9]*; do echo -n "$(( $(cat $c/cpufreq/scaling_cur_freq 2>/dev/null || echo 0) / 1000 )) "; done)"
b=$(cat /sys/class/power_supply/battery/temp 2>/dev/null); [ -n "$b" ] && echo "电池温度: $((b / 10)) °C"
command -v adb >/dev/null && adb devices 2>/dev/null | grep -q "device$" && \
    echo "热状态 (adb): $(adb shell dumpsys thermalservice 2>/dev/null | grep -m1 'Thermal Status')"
