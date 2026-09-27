#!/data/data/com.termux/files/usr/bin/bash
# ============================================================================
# 手机 (Android + Termux) 运行能力与传感器探测
#
#   在 Termux 里执行 (不要在 proot 里):  bash phone_probe.sh
#   需要: pkg install termux-api jq   以及安装 Termux:API 应用 (F-Droid，与 Termux 同一来源)
#   输出: ~/phone_probe_<时间>.txt  —— 发给开发者
#   (可选) 电脑上: adb connect <手机IP>:5555 && adb shell dumpsys sensorservice > sensors.txt
# ============================================================================
OUT="$HOME/phone_probe_$(date +%Y%m%d_%H%M%S).txt"
exec > >(tee "$OUT") 2>&1
sec() { echo; echo "==== $*"; }
sec "设备"
for k in ro.product.manufacturer ro.product.model ro.product.device ro.build.version.release ro.build.version.sdk ro.soc.model ro.board.platform ro.build.fingerprint; do
  printf "%-32s %s\n" "$k" "$(getprop $k 2>/dev/null)"
done
sec "CPU (核 / 最高频率 / 当前频率 / 调度器)"
for c in /sys/devices/system/cpu/cpu[0-9]*; do
  n=${c##*/}; printf "%-6s max %-8s cur %-8s gov %s\n" $n "$(cat $c/cpufreq/cpuinfo_max_freq 2>/dev/null)" "$(cat $c/cpufreq/scaling_cur_freq 2>/dev/null)" "$(cat $c/cpufreq/scaling_governor 2>/dev/null)"
done
grep -m3 -E "Hardware|model name|Features" /proc/cpuinfo
sec "内存 / 存储"
grep -E "MemTotal|MemAvailable|SwapTotal" /proc/meminfo; df -h "$HOME" | tail -1
sec "温度 (thermal zones 可读的)"
for z in /sys/class/thermal/thermal_zone*; do t=$(cat $z/temp 2>/dev/null) && echo "$(cat $z/type 2>/dev/null) $t"; done | head -40
termux-battery-status 2>/dev/null
sec "GPU / 图形"
ls /dev/kgsl-3d0 /dev/dri 2>&1 | head; getprop ro.hardware.egl; getprop ro.hardware.vulkan
command -v vulkaninfo >/dev/null && vulkaninfo --summary 2>/dev/null | head -20
sec "Android 进程限制 (Android 12+ 幽灵进程上限，需 adb 查看/关闭)"
getprop ro.build.version.sdk
echo "电脑上执行: adb -s <手机IP>:5555 shell device_config get activity_manager max_phantom_processes"
echo "            adb -s <手机IP>:5555 shell settings get global settings_enable_monitor_phantom_procs"
sec "传感器列表 (termux-sensor -l)"
if command -v termux-sensor >/dev/null; then
  timeout 15 termux-sensor -l
  sec "传感器采样 (加速度/陀螺/磁力/气压/旋转矢量，各 5 条，间隔 20 ms)"
  for s in accelerometer gyroscope magnetic pressure "rotation vector" "game rotation" "step counter" light proximity; do
    echo "-- $s"; timeout 6 termux-sensor -s "$s" -d 20 -n 5 2>/dev/null | tr -d '\n' | head -c 1500; echo
  done
  termux-sensor -c >/dev/null 2>&1
else
  echo "未安装 termux-api：pkg install termux-api，并安装 Termux:API 应用"
fi
sec "相机"
command -v termux-camera-info >/dev/null && timeout 15 termux-camera-info | head -c 6000
sec "定位 / 网络"
command -v termux-location >/dev/null && timeout 20 termux-location -p network -r once 2>/dev/null
command -v termux-wifi-connectioninfo >/dev/null && termux-wifi-connectioninfo
ip -4 addr 2>/dev/null | grep inet
sec "Termux 与 proot"
echo "TERMUX_VERSION=$TERMUX_VERSION  PREFIX=$PREFIX"
pkg list-installed 2>/dev/null | grep -E "^(proot|proot-distro|python|termux-api|openssh|git)/" | head
command -v proot-distro >/dev/null && proot-distro list 2>/dev/null | grep -iE "installed|ubuntu" | head
termux-wake-lock 2>/dev/null && echo "wake-lock 已获取"
sec "proot Ubuntu 内的运行环境"
if command -v proot-distro >/dev/null; then
  proot-distro login ubuntu -- bash -c '
    cat /etc/os-release | head -2; uname -a
    source /opt/ros/humble/setup.bash 2>/dev/null && echo "ROS_DISTRO=$ROS_DISTRO"
    for p in nav2_bt_navigator slam_toolbox robot_localization nav2_rotation_shim_controller; do ros2 pkg prefix $p >/dev/null 2>&1 && echo "  ✔ $p" || echo "  ✘ $p"; done
    python3 -c "import sys,numpy,mujoco;print(\"python\",sys.version.split()[0],\"numpy\",numpy.__version__,\"mujoco\",mujoco.__version__)" 2>&1
    python3 -c "import psutil;print(\"psutil\",psutil.__version__, \"cpu%\", psutil.cpu_percent(0.5))" 2>&1
    for d in /opt/agv ~/agv_sim ~/ros2_cmodel_agv; do [ -d $d ] && { echo "代码目录 $d"; (cd $d && git log --oneline -1 2>/dev/null; ls nav_runtime | tr "\n" " "; echo; ls -la --time-style=+%F\ %T nav_runtime/*.py | awk "{print \$6,\$7,\$8}"); }; done
    ps -eo pid,pcpu,rss,etime,args --sort=-pcpu 2>/dev/null | grep -E "sim_server|nav_runtime|web_gateway|hub.server|agent.server|ros2|nav2|slam|ekf" | grep -v grep | head -30
  '
fi
sec "MuJoCo 基准 (proot 内，4 线程射线)"
command -v proot-distro >/dev/null && proot-distro login ubuntu -- bash -c 'cd /opt/agv 2>/dev/null || cd ~/ros2_cmodel_agv; timeout 120 python3 tools/bench_host.py 2>&1 | tail -20'
echo; echo "完成: $OUT"
