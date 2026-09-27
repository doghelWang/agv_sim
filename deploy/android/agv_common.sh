#!/data/data/com.termux/files/usr/bin/bash
# ============================================================================
# Android (Termux) 脚本公共部分: 读取配置、定位 proot 容器
#   配置文件 ~/.agv.env (install.sh 生成，可手动修改):
#     AGV_DISTRO=ubuntu                 proot-distro 容器名 (Ubuntu 22.04)
#     AGV_GIT_REMOTE=https://github.com/doghelWang/agv_sim.git   AGV_GIT_BRANCH=main
#     AGENT_NAME=phone1                 在平台上显示的节点名
#     AGV_DEVICE_MODEL="Pixel 4"        机型 (可选，缺省自动识别)
#     AGV_HUB_PORT=8082                 本机平台端口 (本机当平台时)
#     HUB_API=http://<主平台IP>:<端口>   设置后手机只作为计算节点接入主平台，不启动本机平台
#     JOIN_TOKEN=<集群令牌>              主平台 ~/.agv-hub/cluster_token (或平台「添加计算节点」生成的一次性令牌)
#     AGENT_HOST=<本机局域网IP>           可选，缺省自动取 wlan0 地址
#     AGV_PROXY=http://127.0.0.1:7890    可选，访问 GitHub 用的 HTTP 代理 (git 克隆/更新、下载 ROS 密钥)
#     AGV_CPUS_SIM=4-7                   可选，仿真进程绑定的核 (缺省自动取大核；none = 不绑定)
#     AGV_CPUS_WEB=0-3 AGV_CPUS_NAV=     可选，Web 网关 / 执行进程绑定的核 (缺省: 网关放小核，执行不绑定)
#   兼容旧文件 ~/.agv-hub.env (HUB_API/JOIN_TOKEN/AGENT_HOST)
# ============================================================================
set -a
[ -f ~/.agv.env ] && . ~/.agv.env
[ -f ~/.agv-hub.env ] && . ~/.agv-hub.env
set +a
AGV_DISTRO="${AGV_DISTRO:-ubuntu}"
AGV_HUB_PORT="${AGV_HUB_PORT:-8082}"
AGV_ROOTFS="$PREFIX/var/lib/proot-distro/containers/$AGV_DISTRO/rootfs"
[ -d "$AGV_ROOTFS" ] || AGV_ROOTFS="$PREFIX/var/lib/proot-distro/installed-rootfs/$AGV_DISTRO"   # 旧版 proot-distro (<5)
AGV_CODE="$AGV_ROOTFS/opt/agv"

pd() { proot-distro login "$AGV_DISTRO" -- "$@"; }
http_ok() { python3 -c "import urllib.request,sys; urllib.request.urlopen(sys.argv[1], timeout=2)" "$1" 2>/dev/null; }
wlan_ip() {   # 本机局域网地址 (Android 13 上 ifconfig 常拿不到)
    python3 -c "import socket; s=socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.connect(('10.255.255.255', 1)); print(s.getsockname()[0])" 2>/dev/null \
        || ifconfig wlan0 2>/dev/null | awk '/inet /{print $2; exit}'
}
remote_hub() { [ -n "$HUB_API" ] && [ "${AGV_LOCAL_HUB:-0}" != 1 ]; }
big_cores() {   # 大小核手机: 最高频率高于最低档的核 (如骁龙 855: 4-7)；同构 CPU 输出空
    python3 - <<'PY' 2>/dev/null
import glob, re
f = {}
for p in glob.glob("/sys/devices/system/cpu/cpu[0-9]*/cpufreq/cpuinfo_max_freq"):
    try:
        f[int(re.search(r"cpu(\d+)/", p).group(1))] = int(open(p).read())
    except Exception:
        pass
if f and len(set(f.values())) > 1:
    lo = min(f.values())
    print(",".join(str(c) for c in sorted(f) if f[c] > lo))
PY
}
little_cores() {
    python3 - <<'PY' 2>/dev/null
import glob, re
f = {}
for p in glob.glob("/sys/devices/system/cpu/cpu[0-9]*/cpufreq/cpuinfo_max_freq"):
    try:
        f[int(re.search(r"cpu(\d+)/", p).group(1))] = int(open(p).read())
    except Exception:
        pass
if f and len(set(f.values())) > 1:
    lo = min(f.values())
    print(",".join(str(c) for c in sorted(f) if f[c] == lo))
PY
}
# 绑核缺省值 (OPTIMIZATION M2): 仿真进程 (连同其 proot 追踪进程) 放大核，Web 网关放小核
[ -z "${AGV_CPUS_SIM+x}" ] && AGV_CPUS_SIM="$(big_cores)"
[ -z "${AGV_CPUS_WEB+x}" ] && AGV_CPUS_WEB="$(little_cores)"
[ "$AGV_CPUS_SIM" = none ] && AGV_CPUS_SIM=""
[ "$AGV_CPUS_WEB" = none ] && AGV_CPUS_WEB=""
[ "$AGV_CPUS_NAV" = none ] && AGV_CPUS_NAV=""
