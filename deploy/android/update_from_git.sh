#!/data/data/com.termux/files/usr/bin/bash
# ============================================================================
# 从 git 仓库更新手机上的代码 (Termux 里执行)
#   bash ~/update_from_git.sh              拉取并同步到 proot 容器内 /opt/agv
#   bash ~/update_from_git.sh --restart    同步后重启节点 (平台上运行中的实例会被平台重新部署/可手动重启)
# 仓库地址: ~/.agv.env 的 AGV_GIT_REMOTE (https://github.com/doghelWang/agv_sim.git，也可以是 ssh 地址或 git bundle 文件)
# 克隆在容器内 /opt/agv-git；同步到 /opt/agv 时保留设备本地的数据与配置:
#   data/ records/ model_overrides.json robot_config.json sensor_overrides.json robot.urdf
# ============================================================================
# 整个脚本放进一个函数里: 执行过程中会覆盖 ~/update_from_git.sh 自身，bash 必须先读完再执行
main() {
. ~/agv_common.sh
REMOTE="${AGV_GIT_REMOTE:-}"
BRANCH="${AGV_GIT_BRANCH:-main}"
if [ -z "$REMOTE" ] && [ ! -d "$AGV_ROOTFS/opt/agv-git/.git" ]; then
    echo "[错误] 未设置仓库地址: 在 ~/.agv.env 写 AGV_GIT_REMOTE=https://github.com/doghelWang/agv_sim.git"; exit 1
fi
pd env ${AGV_PROXY:+https_proxy=$AGV_PROXY http_proxy=$AGV_PROXY} bash -c "
set -e
command -v rsync >/dev/null || apt-get install -y -q rsync git >/dev/null
export GIT_SSH_COMMAND='ssh -o BatchMode=yes -o StrictHostKeyChecking=accept-new'
if [ ! -d /opt/agv-git/.git ]; then
  git clone -q -b '$BRANCH' '$REMOTE' /opt/agv-git
else
  cd /opt/agv-git
  R='$REMOTE'; [ -n \"\$R\" ] || R=origin
  git fetch -q \"\$R\" '+$BRANCH:refs/remotes/origin/$BRANCH'
  git checkout -q '$BRANCH' 2>/dev/null || git checkout -q -b '$BRANCH' 'origin/$BRANCH'
  git reset -q --hard 'origin/$BRANCH'
fi
cd /opt/agv-git
echo \"仓库版本: \$(git log --oneline -1)\"
mkdir -p /opt/agv
rsync -a --exclude /.git --exclude /data/ --exclude /records/ --exclude /model_overrides.json --exclude /robot_config.json \
      --exclude /sensor_overrides.json --exclude /robot.urdf /opt/agv-git/ /opt/agv/
# 首次安装: 设备本地配置用仓库里的默认值
for f in model_overrides.json robot_config.json sensor_overrides.json robot.urdf; do
  [ -f /opt/agv/\$f ] || cp /opt/agv-git/\$f /opt/agv/\$f
done
git log --oneline -1 > /opt/agv/.agv_version
# 仿真 C 内核 (sim_core/native)：有 gcc 就编译，没有则仿真自动回退纯 Python
if command -v gcc >/dev/null || apt-get install -y -q gcc libc6-dev >/dev/null 2>&1; then
  bash /opt/agv/sim_core/native/build.sh gcc >/dev/null && echo 'libsimcore 已编译' || echo '[警告] libsimcore 编译失败，仿真使用纯 Python 实现'
  bash /opt/agv/planning/native/build.sh gcc >/dev/null && echo 'libagvnav 已编译' || echo '[警告] libagvnav 编译失败，规划使用纯 Python 实现'
fi
# 执行侧 C++ 发布端 (ros2/agv_ros_bridge)：源码有变化才重编 (colcon，手机上约几分钟)；编译失败执行进程自动用 Python 发布
if command -v colcon >/dev/null && [ -f /opt/ros/humble/setup.bash ]; then
  # ros2/tf2: 上游 geometry2 0.25.24 的 tf2 (修掉 waitForTransform 与 testTransformableRequests 的 ABBA 死锁，
  # apt 源还是 0.25.23)，装进同一个 overlay 盖住 /opt/ros 的 libtf2.so；编不过就跳过 (退回系统自带的)
  H=\$(cat /opt/agv/ros2/agv_ros_bridge/src/*.cpp /opt/agv/ros2/agv_ros_bridge/CMakeLists.txt /opt/agv/ros2/agv_ros_bridge/package.xml \
       /opt/agv/ros2/agv_nav2_plugins/src/*.cpp /opt/agv/ros2/agv_nav2_plugins/CMakeLists.txt /opt/agv/ros2/tf2/package.xml 2>/dev/null | md5sum | cut -c1-12)
  if [ \"\$H\" != \"\$(cat /opt/agv/ros2/.built 2>/dev/null)\" ] || [ ! -x /opt/agv/ros2/install/agv_ros_bridge/lib/agv_ros_bridge/agv_ros_bridge ]; then
    echo '编译 agv_ros_bridge (C++) ...'
    (cd /opt/agv/ros2 && . /opt/ros/humble/setup.bash && rm -f tf2/COLCON_IGNORE \
      && { colcon build --packages-select tf2 --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF > /tmp/agv_tf2_build.log 2>&1 \
           || { echo '[警告] tf2 0.25.24 编译失败 (见 /tmp/agv_tf2_build.log)，使用系统自带的 tf2'; touch tf2/COLCON_IGNORE; rm -rf install/tf2 build/tf2; }; } \
      && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF > /tmp/agv_ros_bridge_build.log 2>&1 \
      && echo \$H > .built && echo 'agv_ros_bridge 已编译') || echo '[警告] agv_ros_bridge 编译失败 (见 /tmp/agv_ros_bridge_build.log)，执行进程使用 Python 发布'
  fi
fi
echo '已同步到 /opt/agv'
" || exit 1
# Termux 侧脚本 (启动/停止/状态/更新/派生服务/开机自启) 随仓库更新
R="$AGV_ROOTFS/opt/agv/deploy/android"
for f in agv_common.sh start_agv.sh stop_agv.sh status_agv.sh update_from_git.sh proot_spawner.py android_helper.py keep_front.sh bench_nav.py nav_profile.py prof_stat.py wobble.py wobble_stat.py; do
    cp "$R/$f" ~/"$f" && chmod +x ~/"$f"
done
# GPU 射线求交服务: 在 Termux 里 (不是容器里) 编译，要用系统的 C 库才能加载厂商 OpenCL 驱动；没有 cc 就跳过 (pkg install clang)
if command -v cc >/dev/null && [ -f "$AGV_ROOTFS/opt/agv/sim_core/native/gpucast/gpucastd.c" ]; then
    cc -O2 -o ~/gpucastd.new "$AGV_ROOTFS/opt/agv/sim_core/native/gpucast/gpucastd.c" -ldl -lm -lpthread 2>/dev/null \
        && mv -f ~/gpucastd.new ~/gpucastd && echo 'gpucastd 已编译 (GPU 射线求交服务)' || echo '[警告] gpucastd 编译失败，仿真使用 CPU 求交'
fi
mkdir -p ~/.termux/boot && cp "$R/termux-boot-01-start-agv.sh" ~/.termux/boot/01-start-agv.sh && chmod +x ~/.termux/boot/01-start-agv.sh
if [ "$1" = "--restart" ]; then
    bash ~/stop_agv.sh
    setsid nohup bash ~/start_agv.sh > ~/start_agv.out 2>&1 < /dev/null &
    sleep 15
    tail -6 ~/start_agv.out
fi
}
main "$@"
exit $?
