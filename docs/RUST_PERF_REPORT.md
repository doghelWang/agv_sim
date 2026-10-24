# Rust 运行时性能报告 (Galaxy Z Flip5 / Ubuntu 22.04 chroot)

测量脚本 `perf.py` (手机 `/root/perf.py`)：先空闲 30 秒，再跑四工位循环任务；按进程组统计 CPU (单核 %)、常驻内存、仿真实时因子与步率、REST 延迟。
结果文件：手机 `/root/perf_python_gt.out` `perf_python_noros2.out` `perf_rust3.out`。

## 1. 资源占用 (一台仿真实例：sim + web 网关 + nav)

| 运行时 | CPU 合计 (空闲→跑任务) | 内存合计 (空闲→跑任务) | 仿真步率 | RTF | REST p50 / p95 |
|---|---|---|---|---|---|
| Python + ROS2 Nav2 (ground-truth 定位) | 174% → 83% | **894 MB** → 586 MB | 76.7/s | 1.0 | 1.13 / 4.96 ms |
| Python，不带 ROS | 81% → 82% | 106 MB → 177 MB | 72/s | 1.0 | 3.45 / 10.25 ms |
| **Rust (本次)** | **51% → 55%** | **53 MB → 65 MB** | **83/s** | 1.0 | **0.62 / 1.36 ms** |

分进程 (Rust，空闲)：sim-server 31% / 11.6 MB；web-gateway 14% / 37 MB (跑任务后 48 MB)；nav-runtime 6% / 4.3 MB。
相比 Python 无 ROS：内存约 1/2~1/3，CPU 约 63%，API 延迟约 1/5；相比 Python+Nav2：内存约 1/14，CPU 约 29%。

## 2. 任务质量 (四工位循环)

| 运行时 | 到达 | 停车位置误差 | 航向误差 | 单段用时 |
|---|---|---|---|---|
| Rust | **4/4 ARRIVED** (两次独立测量) | 0.17–0.21 cm | 0.05–0.18° | 19–39 s |
| Python 无 ROS | 0/4 (FAILED / OBSTACLE_WAIT) | 4–5 m | — | — |
| Python + Nav2 | 0 (CANCELED / FAILED) | 7–10 m | — | 120 s 超时 |

注意：Python 两组任务失败的主要原因是手机上 Python 仿真的激光雷达冻结 (之前遗留问题，未在 Python 里修)，不是路径规划算法的差别，所以只能用来比资源，不能说 Rust 的规划比 Python 好。

## 3. 本次修复与已知限制

- web-gateway 内存：从 618 MB 降到约 32–50 MB (记录文件按需加载、压缩帧字符串、跳过 8 MB 以上的记录)。
- Nav2 后端 (`NAV_NAV2=1` 或把规划器切到 nav2)：功能测试通过 (到达、取消、切换、重启后无遗留进程)，**这次没有做 Nav2 模式的资源测量**：`perf_nav2mode.out` 实际仍是 dijkstra 规划器 (脚本里的切换没有生效)，不能当作 Nav2 数据。
- 没有测的：带障碍物的 Nav2、手机重启后的自启持久性、GPU/NPU 加速 (当前计算量小，没有用到)。

## 4. 之前的 501 ("not supported") 是什么

旧 Python 平台里这些功能依赖 Docker / Termux / Python 打包，Rust 重写时先留了占位，返回 501。现在都已实现：

| 接口 | Rust 实现 |
|---|---|
| `admin/updates` (上传/详情/应用/回滚/删除) + `admin/jobs` | 解析 `agv-update/1` 的 .tgz，校验 sha256，备份后原子替换文件 (可替换正在运行的程序)，按文件类型重启 hub / agent / 实例，失败可回滚 |
| `admin/apps` (上传/下载/删除/安装) + `admin/adb/*` | 直接解析 APK 里的二进制清单 (包名/版本/名称)；安装和无线调试走本机 `adb` (没有 adb 时返回明确提示)；外屏面板需要安卓助手，仍返回 400 说明 |
| `admin/logs` | 列出并下载日志文件 (最近 8 MB) |
| `packages/upload` | 接收 tar/tar.gz：docker save 镜像包或含 `bin/` 的程序包 |
| `packages/from_node` + agent `images/export` | 节点把 `bin/` 打包 tar.gz 上传到平台入库 (手机实测 9.2 MB) |
| `nodes/add` | 用本机 `ssh` 验证登录，密码加密保存，节点显示为待接入 (本机无 ssh 时返回 502 说明) |
