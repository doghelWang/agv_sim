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
3. **树莓派的瓶颈是仿真把 4 核吃满**：同样的仿真在手机上约 120~140% CPU，在树莓派上 220~355%。推测是 pip numpy 自带的 OpenBLAS 多线程空转，可试 `OPENBLAS_NUM_THREADS=1` (待验证，见 BACKLOG)。
4. 各组合仿真实时因子都能保持 1.0。

## 4. 复现

```bash
# 两台设备都接入同一主平台后，在手机 Termux 里:
python3 $PREFIX/var/lib/proot-distro/containers/ubuntu/rootfs/opt/agv/tools/perf_matrix.py \
    --hub http://<主平台IP>:<端口> --board-ssh <用户>@<板卡IP> --board-node <板卡节点名> --phone-node <手机节点名>
# 结果: ~/perf_matrix/summary.json、各组合目录下的采样与精度明细
```

注意：测试会依次停止平台上运行中的实例，结束后恢复；手机需能免密 ssh 到板卡。手机温度在 Termux 里读不到 (Android 权限)，用 `adb shell dumpsys thermalservice` 查看。
