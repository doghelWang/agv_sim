# Android 部署脚本

完整说明见 [docs/DEPLOY_ANDROID.md](../../docs/DEPLOY_ANDROID.md)。

| 文件 | 运行位置 | 说明 |
|---|---|---|
| `install.sh` | Termux | 一键安装：proot Ubuntu 22.04 → ROS 2 Humble/Nav2/MuJoCo → 代码 → 脚本 → 配置 → 启动 |
| `proot_setup.sh` | proot 容器内 | 安装运行环境 (install.sh 调用) |
| `agv_common.sh` | Termux | 公共配置 (读 `~/.agv.env`) |
| `start_agv.sh` / `stop_agv.sh` / `status_agv.sh` | Termux | 启动 / 停止 / 状态 |
| `update_from_git.sh` | Termux | 从 git 更新代码 |
| `termux-boot-01-start-agv.sh` | Termux:Boot | 开机自启 |
| `proot_spawner.py` | Termux 原生 Python | proot 派生服务 :8069 |
| `fastdds_localhost.xml` / `cyclonedds_localhost.xml` | 执行进程 | Android 回环 DDS 配置 |
