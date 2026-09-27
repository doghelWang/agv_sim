# 性能对比：手机 vs 树莓派 5

测试时间 2026-09-27。工具：`tools/perf_matrix.py` (经主平台依次部署 4 种组合) + `tools/perf_sample.py` (两台设备同时每 2 s 采样) + `tools/precision_test.py` (精度) + `tools/bench_host.py` (单机基准)。

| 设备 | 配置 | 运行方式 |
|---|---|---|
| 板卡 | 树莓派 5，4 核 Cortex-A76 2.4 GHz，4 GB | Docker (agv-sim / agv-nav 镜像) |
| 手机 | Pixel 4，骁龙 855 (1×2.84 + 3×2.42 + 4×1.78 GHz)，6 GB，Android 13 | Termux + proot Ubuntu 22.04，proot 派生服务 |

同一车型 (单舵轮测试车)、同一场景 (九宫格立体仓 grid_9_square)、同一组 5 个目标、slam_toolbox 建图定位；每种组合分别用 Nav2 (RotationShim + RPP 线路跟随) 与自研导引各跑一遍。

## 1. 单机计算基准 (bench_host.py)

| 项目 | 手机 (proot 内) | 树莓派 5 |
|---|---|---|
| 纯 Python 单核循环 | 18.6 ms | 19.5 ms |
| numpy 400×400 矩阵乘 | 71.9 ms (Ubuntu 自带 numpy，无优化 BLAS) | 2.7 ms (pip numpy + OpenBLAS) |
| MuJoCo 物理一步 | 0.75 ms | 0.54 ms |
| 2D 激光一次扫描 (1638 束) | 1.37 ms | 1.16 ms |
| 3D 激光一帧 (6000 点) | 6.6 ms | 3.6 ms |
| 相机一帧 (射线渲染) | 152 ms | 179 ms |

单核算力两者相当；手机的优势是 8 核，但 proot 的系统调用转发有额外开销 (每次 stat/send 约 80~130 µs，原生约 3~4 µs)。

## 2. 组合测试

| 组合 | 仿真 | 执行 | 规划 | 到达 | 横向偏差最大 | 终点误差最大 | 定位误差最大 | 用时 |
|---|---|---|---|---|---|---|---|---|
| A | 树莓派 | 树莓派 | Nav2 | 5/5 | 50.5 mm | 15.6 mm | 29.4 mm | 265 s |
| | | | 自研 | 5/5 | 25.1 mm | 18.4 mm | 37.8 mm | 160 s |
| B | 手机 | 手机 | Nav2 | 3/5 | 315.7 mm | 30.6 mm | 44.0 mm | 502 s |
| | | | 自研 | 5/5 | 784 mm | 16.7 mm | 276 mm | 213 s |
| **C** | **手机** | **树莓派** | **Nav2** | **5/5** | **34.0 mm** | 20.1 mm | 31.1 mm | **186 s** |
| | | | 自研 | 5/5 | 36.1 mm | 20.8 mm | 35.2 mm | 167 s |
| D | 树莓派 | 手机 | Nav2 | 5/5 | 236.3 mm | 20.9 mm | 47.0 mm | 228 s |
| | | | 自研 | 5/5 | 43.5 mm | 21.8 mm | 47.2 mm | 180 s |

运行负载 (测试期间平均)：

| 组合 | 仿真 RTF | 执行进程收到状态频率 | 树莓派整机 CPU | 手机进程 CPU 合计 (8 核=800%) | 手机 proot 追踪进程 | Nav2 启动到激活 |
|---|---|---|---|---|---|---|
| A | 1.00 | 41 Hz | 94% (4 核跑满) | — | — | 4 s |
| B | 1.00 | 21 Hz | — | 317~372% | 54% | 15 s |
| C | 1.00 | 39 Hz | 36~42% | 206% | 37% | 6 s |
| D | 1.00 | 30 Hz | 95% | 200~228% | 22% | 15 s |

主要进程 CPU (单核 = 100%)：

| 进程 | A 树莓派 | B 手机 | C 手机 (仿真) / 树莓派 (执行) | D 树莓派 (仿真) / 手机 (执行) |
|---|---|---|---|---|
| sim_server | 222% | 117% | 136% | 355% |
| web_gateway | 12% | 26% | 32% | 10% |
| nav_runtime | 46% | 61% | 41% | 80% |
| slam_toolbox | 26~50% | 21~85% | 32~58% | 50~88% |
| Nav2 各服务器合计 | ~45% | ~40% | ~40% | ~45% |

## 3. 结论

1. **推荐组合 C：手机跑仿真，板卡跑执行 (ROS 2 / Nav2 / slam_toolbox)**。板卡负载最低 (约 40%)，Nav2 横向偏差 34 mm，用时最短。
2. **手机不适合跑执行进程**：proot 下执行进程拿到状态只有 20~30 Hz，控制回路延迟大，Nav2 跟线偏差 236~316 mm；手机全包 (B) 时 slam_toolbox 也跟不上，定位误差可达 276 mm。
3. **树莓派的瓶颈是仿真把 4 核吃满**：同样的仿真在手机上约 120~140% CPU，在树莓派上 220~355%。当时推测是 OpenBLAS 多线程空转；实测主要原因是相机持续渲染，P0 优化后已降到 26~36% (见第 4 节)。
4. 各组合仿真实时因子都能保持 1.0。

## 4. P0 优化后复测 (2026-09-27 下午)

改动 (见 [OPTIMIZATION.md](OPTIMIZATION.md) 第一批)：相机按需成像 (`SIM_CAMERA_IDLE_S=3`)、执行进程只在有 ROS 订阅时拉相机 (`NAV_CAMERAS=auto`)、
`OPENBLAS_NUM_THREADS=1` / `OMP_NUM_THREADS=1`、`SIM_RAY_THREADS` 默认 2、手机补装 OpenBLAS、手机仿真进程 (连同 proot 追踪进程) 绑大核 4-7、网关绑小核 0-3。
同一套 `perf_matrix.py` 重跑 A~D，每组各一次：

| 组合 | 规划 | 树莓派整机 CPU | 手机进程 CPU 合计 | sim_server CPU | 状态频率 | 到达 | 横向偏差最大 | 终点误差最大 | 定位误差最大 |
|---|---|---|---|---|---|---|---|---|---|
| A | Nav2 | 94% → **49%** | — | 222% → **35%** | 41 → **48 Hz** | 5/5 | 50.5 → 45.2 mm | 15.6 → 26.0 mm | 29.4 → 43.4 mm |
| | 自研 | 95% → **55%** | — | 204% → **36%** | 38 → **48 Hz** | 5/5 | 25.1 → 31.8 mm | 18.4 → 23.0 mm | 37.8 → 41.3 mm |
| B | Nav2 | — | 317% → 317% | 117% → **60%** | 21 → 26 Hz | 3/5 → **5/5** | 316 → 355 mm | 30.6 → 28.6 mm | 44.0 → 58.1 mm |
| | 自研 | — | 372% → 358% | 114% → **61%** | 17 → 22 Hz | 5/5 | 784 → **52.5 mm** | 16.7 → 28.3 mm | 276 → **52.9 mm** |
| **C** | **Nav2** | 36% → 35% | 206% → **156%** | 136% → **65%** | 38 → **48 Hz** | 5/5 | 34.0 → 49.1 mm | 20.1 → 17.6 mm | 31.1 → 27.8 mm |
| | 自研 | 42% → 41% | 207% → **156%** | 136% → **65%** | 39 → **47 Hz** | 5/5 | 36.1 → **27.6 mm** | 20.8 → **12.0 mm** | 35.2 → 31.1 mm |
| D | Nav2 | 95% → **12%** | 200% → 230% | 355% → **26%** | 31 → 31 Hz | 5/5 → 2/5 | 236 → 77.5 mm (已到达的) | 20.9 → 9.4 mm | 47.0 → 37.1 mm |
| | 自研 | 95% → **12%** | 228% → 194% | 355% → **27%** | 32 → 36 Hz | 5/5 → 4/5 | 43.5 → 58.9 mm | 21.8 → 13.8 mm | 47.2 → 58.2 mm |

实例 i11 (组合 C) 静置时的仿真内部指标：物理一步 3.3 → **2.0 ms**，三路激光一次扫描 12.8 → **4.9 ms**，相机不再成像 (之前 200 ms/帧持续渲染)；
手机 numpy 256×256 矩阵乘 3.5 ms (装 OpenBLAS 前约 75 ms/400×400)。

结论：
1. **CPU 大幅下降**：仿真在树莓派上从 200~355% 降到 26~36%，树莓派同时跑仿真和执行 (A) 时整机从 94% 降到约 50%；手机仿真进程从 136% 降到 65%。
   主要来自相机按需成像 (本地 A/B：同一仿真进程 190% → 11%)。
2. **状态频率接近 50 Hz 目标** (A、C 组 38~41 → 47~48 Hz)，因为仿真进程不再被相机渲染拖慢。
3. **精度没有明显变化**：A、C 组横向偏差 28~49 mm、终点误差 12~26 mm，与优化前在同一范围，仍然受定位 (slam_toolbox) 30~40 mm 的跳变限制；单次测试的波动大于这次改动的影响。
4. 执行放在手机上 (B、D) 依然不可用：D 组 Nav2 这次有 3 个目标失败并发生碰撞 (之前 5/5 但偏差 236 mm)，属于该组合本来的不稳定，不建议使用。
5. 手机 Web 网关绑到小核后 CPU 占用从 32% 升到 45~53% (小核慢，同样的工作占用时间更长)，但没有影响仿真；第二批 (P1) 会让网关空闲时不轮询。

## 5. 仿真核心 C 化 + 执行桥接 C++ 化 (2026-09-27 晚)

### 5.1 改动

| 部分 | 实现 | 开关 |
|---|---|---|
| 仿真核心 | `sim_core/native/simcore.c` → `libsimcore.so` (纯 C，ctypes 加载)：`sc_collides` 车身/触边多边形 vs 线段、`sc_raycast2d` 线段/圆柱射线 (光电，在 MuJoCo 后端下用与 MuJoCo 模型相同的 5 cm 厚墙盒体与行人圆柱)、`sc_forward` 轮系加权最小二乘 (单边 Jacobi SVD，与 `numpy.linalg.lstsq` 同解同秩判定)、`sc_se2_integrate`、`sc_kin_step` (`ChassisKinematics.step` 整体下沉) | `SIM_NATIVE=0` 回退纯 Python；加载失败自动回退；`/api/v1/sim` 的 `native` 字段显示状态 |
| 仿真 HTTP/JSON | 激光测距 JSON 向量化 (`ranges_json`)、快照里的融合扫描按帧序号缓存；执行进程取融合扫描改二进制 (`Accept: application/octet-stream`) | — |
| 执行桥接 | `ros2/agv_ros_bridge` (rclcpp)：`/odom` `/ground_truth/odom` `/imu` `/joint_states` `/scan` `/scan/<name>` 与 TF 广播，50 Hz 查询 TF map/odom→base_footprint 回传；Python ↔ C++ 走容器内 Unix 数据报 (`nav_runtime/cpp_bridge.py`)。时间戳偏移、`_costmap_stamp` 钳位 (`AGV_COSTMAP_SCAN_CLAMP` / `AGV_COSTMAP_SCAN_MARGIN`)、协方差、关节隔帧发布与 Python 版一致。Python 节点不再订阅 `/tf`、不再有 50 Hz 定时器 | `NAV_ROS_BRIDGE=auto` (默认，有可执行文件就用 C++) / `cpp` / `py`；`/api/v1/nav` 的 `ros_bridge` 字段显示计数 |
| 构建 | `Dockerfile.sim` 多阶段编译 `libsimcore.so` (`tools/refresh_images.sh` 也带上该阶段，走 `APT_MIRROR`)；`Dockerfile.nav` 用 colcon 编译 `agv_ros_bridge`；手机 `proot_setup.sh` 装 gcc、`update_from_git.sh` 同步后编译 C 内核 (C++ 桥接在手机上未编译，自动用 Python) | — |
| 其它 | `NAV_ROS_EXECUTOR=single` 可试单线程执行器 (默认仍 4 线程)；平台实例选项 `env_sim` / `env_nav` 透传环境变量；`hub/store.py write_json` 临时文件名加进程/线程号 (修复并发写同一记录时 `os.replace` 的 FileNotFoundError) | — |

一致性：`tests/test_sim_core.py` 新增 6 项 C/Python 对比 (碰撞 3000 个随机位姿、射线 200×721 束、正解 400 组随机轮系含秩亏、运动学 3 车型各 3000 步含刹车/急停、SE2)，
全部逐项一致 (误差 < 1e-9，饱和系数有 1 ulp 差)；光电 C 射线 vs `mj_multiRay` 在 300 个随机位置 × 90 束上最大偏差 < 2 mm。

### 5.2 树莓派组合 A 对比 (i12，同一套流程：重启实例 → 采样 → 3 个目标 dijkstra + Nav2)

`tools/perf_sample.py` 每 2 s 采样 (约 4.5 min/次)，`/api/v1/sim` 的 `step_ms` 为物理一步耗时 (EMA，含等 GIL)。

| 运行 | 仿真 C 内核 | 执行桥接 | 整机 CPU | sim_server | nav_runtime | C++ 桥接 | slam_toolbox | Nav2 合计 | step_ms | 状态频率 |
|---|---|---|---|---|---|---|---|---|---|---|
| 基线 67c6eac | — | Python | 46.1% | 34.1% | 50.5% | — | 29.7% | 42.3% | 1.54 | 48.5 Hz |
| 新代码全关 (对照) | 关 | Python | 45.6% | 30.2% | 50.8% | — | 28.3% | 44.2% | 1.50 | 48.5 Hz |
| 第 1 步 | 开 | Python | 43.0% | **22.4%** | 51.0% | — | 27.8% | 42.4% | **0.56** | 48.7 Hz |
| 第 2 步 | 开 | C++ | **37.4%** | 22.1% | **19.5%** | 3.0% | 32.2% | 40.7% | 0.55 | 49.3 Hz |
| 第 2 步 (重复) | 开 | C++ | **37.0%** | 22.1% | **19.4%** | 2.9% | 34.5% | 40.2% | 0.55 | 49.3 Hz |

- 仿真进程 34% → 22% (其中 JSON 向量化/缓存约 4 个点，C 内核约 8 个点)，物理一步 1.54 → 0.55 ms。
- 执行进程 50% → 19.5% (+ C++ 节点 3%)：py-spy 里 rclpy 执行器 (原 44% GIL 时间) 从热点中消失。
- 整机 46% → 37%，状态频率 48.5 → 49.3 Hz (最低值 40 → 46 Hz)，RTF 保持 1.0。

单机微基准 (`tools/bench_simcore.py`，树莓派 sim 容器，MuJoCo 后端，µs/次)：

| 项目 | Python | C 内核 |
|---|---|---|
| 物理整步 `SimCore.step` | 669 | **155** |
| 运动学 `kin.step` | 88 | 15 |
| 里程计 `odom.update` (含一次正解) | 68 | 12 |
| 正解 `forward` | 57 | 6 |
| 开关量 `update_discrete` (4 光电 + 2 触边) | 571 | 104 |
| 单个触边 / 车身碰撞 | 84 / 129 | 9 / 11 |
| 单个光电 (3 射线) | 86 | 21 |

### 5.3 精度回归 (`precision_test.py`，目标 [[0,5,90°],[5,0,0],[0,0,0]])

| 运行 | dijkstra 横偏 / 终点 / 定位 (mm) | Nav2 横偏 / 终点 / 定位 (mm) |
|---|---|---|
| 基线 | 20.1 / 22.5 / 34.2 | 40.0 / 18.8 / 31.3 |
| 新代码全关 | 24.4 / 12.9 / 32.6 | 43.3 / 20.9 / 29.5 |
| 第 1 步 | 15.4 / 14.0 / 30.8 | 28.2 / 19.3 / 27.7 |
| 第 2 步 | 30.1 / 7.7 / 31.7 | 31.1 / 18.7 / 24.7 |
| 第 2 步 (重复) | 21.2 / 14.0 / 29.5 | 38.3 / 20.7 / 25.9 |

全部 3/3 到达；各项在同一波动范围内 (仍受 slam_toolbox 30 mm 级定位跳变限制)，没有看到 C/C++ 化引入的精度变化。

复现: 树莓派上 `bash tools/ab_bench_i12.sh <标签>` (重启 i12 → 采样 + py-spy → 精度)，`python3 tools/perf_summary.py ~/bench/*/perf.json`，
`python3 tools/spy_summary.py ~/bench/<标签>/spy_sim.txt`；微基准 `python3 tools/bench_simcore.py` 与 `SIM_NATIVE=0 python3 tools/bench_simcore.py`。

### 5.4 剩余热点 (第 2 步的 py-spy)

- 仿真进程：HTTP 请求处理约 60% GIL 时间 (头解析 `parse_headers`、`json.dumps`)，来自执行进程 50 Hz 状态轮询 + 20 Hz IO/光电 + 网关快照。
- 执行进程：`sim_link` 的 HTTP 客户端 (状态轮询、激光长轮询、cmd_vel PUT) 约 50%。
- 下一步可考虑：状态改为长轮询/推送或二进制帧，`/api/v1/nav` 状态接口里的 `ros.graph()` 改缓存。

## 6. 复现

```bash
# 两台设备都接入同一主平台后，在手机 Termux 里:
python3 $PREFIX/var/lib/proot-distro/containers/ubuntu/rootfs/opt/agv/tools/perf_matrix.py \
    --hub http://<主平台IP>:<端口> --board-ssh <用户>@<板卡IP> --board-node <板卡节点名> --phone-node <手机节点名>
# 结果: ~/perf_matrix/summary.json、各组合目录下的采样与精度明细
```

注意：测试会依次停止平台上运行中的实例，结束后恢复；手机需能免密 ssh 到板卡。手机温度在 Termux 里读不到 (Android 权限)，用 `adb shell dumpsys thermalservice` 查看。
