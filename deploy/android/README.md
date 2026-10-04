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
| `autostart.py` | Termux 原生 Python | `AGV_AUTOSTART=1` 时开机自动拉起实例 |
| `cover_app/` | Termux 编译，电脑 adb 安装 | 外屏状态面板 APK：全屏打开本机平台的 `/cover.html` (折叠屏外屏；`bash cover_app/build.sh`) |

GPU 射线求交服务 `~/gpucastd` (源码 `sim_core/native/gpucast/`) 由 `update_from_git.sh` 用 Termux 的 `cc` 编译，`start_agv.sh` 启动；
说明与实测见 [docs/PERFORMANCE.md](../../docs/PERFORMANCE.md) 6.8 节。`AGV_GPU_CAST=0` 关闭。

状态面板也可以直接用浏览器打开：`http://<平台>/cover.html` (`?inst=i01` 指定实例，`?period=2` 刷新周期)。
注意：三星手机上面板应用在前台时 Termux 只能用 3 个小核 (见 PERFORMANCE.md 6.8)，跑重负载时把 Termux 切回前台。
