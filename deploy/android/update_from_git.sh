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
fi
echo '已同步到 /opt/agv'
" || exit 1
# Termux 侧脚本 (启动/停止/状态/更新/派生服务/开机自启) 随仓库更新
R="$AGV_ROOTFS/opt/agv/deploy/android"
for f in agv_common.sh start_agv.sh stop_agv.sh status_agv.sh update_from_git.sh proot_spawner.py; do
    cp "$R/$f" ~/"$f" && chmod +x ~/"$f"
done
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
