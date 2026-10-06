# 手机现场快照 (Galaxy Z Flip 5，SM-F731B，2026-10-06)

这个分支是手机上实际在跑的那一套的原样快照，从 `main` 的 `7e0db2d` 分出：

- **仓库根目录** = 手机上 `/opt/agv` (proot Ubuntu 里) 的源码，逐文件覆盖到 `main` 之上。`git diff main..phone-flip5-snapshot -- . ':!phone_snapshot'`
  看到的就是手机与 `main` 的全部差别：
  - `docs/*.md`、`docker/Dockerfile.nav`、`deploy/android/wobble.py`：手机上是较早的版本 (这些文件不影响手机运行，没有每次同步)
  - `maps/*.pgm`、`nav2/nav2_params_*.yaml`：运行时按当前车型/场景重新生成的
  - `ros2/tf2/test/` 等：手机上是 geometry2 0.25.24 的完整 tf2 源码，`main` 里去掉了测试
  - `main` 里有而手机上没有的 `docs/FLIP5_PERF_REPORT.md`、`ros2/tf2/LICENSE` 保留未动
- **`phone_snapshot/termux_home/`** = Termux 主目录 (`~`) 里与本项目有关的文件
  - 启停/更新脚本 (`start_agv.sh`、`stop_agv.sh`、`proot_spawner.py` …，与 `deploy/android/` 同源)、开机自启 (`.termux_boot/`)
  - 排查与测量脚本：`nav_profile.py`、`wobble.py`、`tune.py` (优先级/绑核试验)、`dump.py`/`stk.py`/`sym.py` (没有 gdb 时看线程卡在哪)、
    `sysb.py`/`udpb.py`/`cb.py` (proot 系统调用、UDP、单核基准) 等
  - 测量数据：`prof_*.csv/log`、`wobble_*.csv/log`、`idle_group.log` (docs/FLIP5_PERF_REPORT.md 里的数字出自这些)
  - `cover_build/app/` = 外屏监控面板 `com.agvsim.cover` (源码与 `deploy/android/cover_app/` 相同)，`bin/agv-cover.apk` 是手机上装的那个包
  - `CoverMonitorApp/` = 更早的一版外屏监控应用 `com.cloudai.covermonitor` (源码 + 已编译的 apk)
- **`phone_snapshot/env/`** = 运行环境清单。手机上没有 Docker 镜像 (用的是进程运行时：Termux + proot-distro Ubuntu 22.04)，
  相当于"镜像"的是那个 Ubuntu rootfs (数 GB，不适合放进 git)，这里记录的是重建它所需的信息：
  `device.txt` (机型/系统/内核/proot 版本)、`termux_packages.txt`、`ubuntu_dpkg.txt`、`ubuntu_pip.txt`、`built_binaries.txt` (现场编译出的 .so / 可执行文件)。
  按 `deploy/android/install.sh` + `proot_setup.sh` 可以从零装出同样的环境；Docker 镜像的定义在 `docker/` (Dockerfile.sim / Dockerfile.nav)。

## 没有放进来的

| 内容 | 原因 |
|---|---|
| `~/.agv.env`、`/root/.agv-hub/`、各种 `*.log` 里带集群令牌的 | 凭据 |
| 签名密钥 (`.agv-cover.keystore`、`debug.keystore`) | 密钥 |
| `service_native.py`、`test_pipeline_breakdown.py`、`test_timing.py`、`qnn_yolo_server*`、`*.onnx`、`libonnxruntime.so` | 属于手机上另一个项目 (视觉识别服务)，且前三个文件里写有摄像头/机器人账号 |
| sing-box 配置 (`config.json`) | 与本项目无关，含代理账号 |
| 编译产物 (`ros2/build`、`ros2/install`、`*.so`、`gpucastd`、`*.class`、`classes.dex`) | 都能由源码重新编译，清单见 `env/built_binaries.txt` |
| 实例运行数据 (`/root/.agv-agent/instances/`：地图、录制、日志) | 运行数据，体积大 |
