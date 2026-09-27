#!/bin/bash
# ============================================================================
# 在 proot Ubuntu 22.04 容器内安装运行环境 (由 install.sh 调用，也可单独执行)
#   ROS 2 Humble (ros-base + Nav2 + slam_toolbox + robot_localization + robot_state_publisher)
#   Python: numpy / Pillow / psutil / protobuf (apt)，MuJoCo (pip)
#   AGV_MIRROR=tuna (默认，清华镜像；国内推荐) | official (ports.ubuntu.com / packages.ros.org)
# 可重复执行 (已安装的会跳过)。约需下载 700 MB，安装后占用约 2.5 GB。
# ============================================================================
set -e
export DEBIAN_FRONTEND=noninteractive
MIRROR="${AGV_MIRROR:-tuna}"
. /etc/os-release
if [ "$VERSION_ID" != "22.04" ]; then
    echo "[错误] 需要 Ubuntu 22.04 (ROS 2 Humble 只支持 jammy)，当前 $PRETTY_NAME"; exit 1
fi
ARCH=$(dpkg --print-architecture)

if [ "$MIRROR" = tuna ]; then
    UB=http://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports; [ "$ARCH" = amd64 ] && UB=http://mirrors.tuna.tsinghua.edu.cn/ubuntu
    ROS_REPO=http://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu
else
    UB=http://ports.ubuntu.com/ubuntu-ports; [ "$ARCH" = amd64 ] && UB=http://archive.ubuntu.com/ubuntu
    ROS_REPO=http://packages.ros.org/ros2/ubuntu
fi
echo "== apt 源: $UB  ROS: $ROS_REPO"
cat > /etc/apt/sources.list <<EOF
deb $UB jammy main restricted universe multiverse
deb $UB jammy-updates main restricted universe multiverse
deb $UB jammy-backports main restricted universe multiverse
deb $UB jammy-security main restricted universe multiverse
EOF
printf 'Acquire::Retries "5";\nAcquire::http::Timeout "30";\n' > /etc/apt/apt.conf.d/99agv-net
apt-get update -q
apt-get install -y -q --no-install-recommends ca-certificates curl gnupg git rsync procps util-linux iproute2 locales

# ---- ROS 2 Humble 软件源
KEY=/usr/share/keyrings/ros-archive-keyring.gpg
if [ ! -s "$KEY" ]; then
    for u in https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
             https://mirrors.tuna.tsinghua.edu.cn/rosdistro/ros.key; do
        curl -fsSL --max-time 30 ${AGV_PROXY:+-x $AGV_PROXY} "$u" -o "$KEY" && [ -s "$KEY" ] && break
    done
    [ -s "$KEY" ] || { echo "[错误] 下载 ROS 签名密钥失败，检查网络/代理"; exit 1; }
fi
echo "deb [arch=$ARCH signed-by=$KEY] $ROS_REPO jammy main" > /etc/apt/sources.list.d/ros2.list
apt-get update -q

echo "== 安装 ROS 2 Humble + Nav2 + slam_toolbox + robot_localization (耗时较长)"
apt-get install -y -q --no-install-recommends \
    ros-humble-ros-base ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-robot-state-publisher \
    ros-humble-slam-toolbox ros-humble-robot-localization \
    python3-pip python3-numpy python3-pil python3-psutil python3-protobuf libopenblas0-pthread

# ---- MuJoCo (pip)。proot 里 ANDROID_* 环境变量会让 pip 误判成 Android 而启动失败，先去掉
if ! python3 -c "import mujoco" 2>/dev/null; then
    echo "== pip 安装 MuJoCo"
    PIPI=""; [ "$MIRROR" = tuna ] && PIPI="-i https://pypi.tuna.tsinghua.edu.cn/simple"
    env -u ANDROID_DATA -u ANDROID_ROOT -u ANDROID_ART_ROOT -u ANDROID_I18N_ROOT -u ANDROID_TZDATA_ROOT \
        python3 -m pip install --no-cache-dir $PIPI "mujoco>=3.3" "numpy<2"
fi

python3 - <<'EOF'
import numpy, mujoco, PIL, psutil, google.protobuf as pb
print("numpy", numpy.__version__, "| mujoco", mujoco.__version__, "| Pillow", PIL.__version__, "| psutil", psutil.__version__, "| protobuf", pb.__version__)
EOF
bash -c 'source /opt/ros/humble/setup.bash && for p in nav2_controller nav2_bt_navigator slam_toolbox robot_localization robot_state_publisher; do
    ros2 pkg prefix $p >/dev/null 2>&1 && echo "  ✔ $p" || { echo "  ✘ $p"; exit 1; }; done'
rm -rf /var/lib/apt/lists/*
echo "== 运行环境安装完成"
