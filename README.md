# AMR 仿真平台 (CModel AGV 仿真与导航)

由 `.cmodel` 车辆模型驱动的 AMR (自主移动机器人) 仿真与导航系统，可部署在 **Linux 板卡 (RK3588、树莓派 5)、x86 电脑** 和 **Android 手机** 上，多台设备组成一个集群：

- **资源平台 agv-hub**：计算节点、车辆模型、仿真场景、软件程序包、仿真记录；网页部署向导把一次仿真的「仿真进程」和「执行进程」分配到一台或两台节点。
- **仿真进程 sim_server**：MuJoCo 物理 + 激光 / 相机 / 光电 / 触边 / IO 仿真 (不依赖 ROS)，附带设备端 Web 工作台。
- **执行进程 nav_runtime**：定位 (slam_toolbox + robot_localization；无 ROS 时内置 SLAM)、拓扑路径规划、导航 (Nav2 线路跟随 RotationShim + Regulated Pure Pursuit；自研精确导引)、任务流、保护空间。
- **节点代理 agv-agent**：在每台设备上启停进程 (Linux 用 Docker，手机用 proot 进程)。

进程之间只用 HTTP REST 通信，ROS 2 只在执行进程内部使用。

```
        浏览器 ──► 资源平台 agv-hub (主平台，一个集群一个)
                        │ 部署/监控
          ┌─────────────┴──────────────┐
    节点代理 (手机)                 节点代理 (RK3588 / 树莓派)
    仿真进程 + Web 网关  ◄── REST ──►  执行进程 (ROS 2 Humble: slam_toolbox + Nav2)
```

## 快速开始

**Linux 板卡 (RK3588 / 树莓派 5 / x86)，当主平台：**

```bash
git clone https://github.com/doghelWang/agv_sim.git ~/agv_sim && cd ~/agv_sim
bash deploy/linux/setup.sh            # 装 Docker → 构建镜像 → 启动平台；内存 < 6 GB 加 --swap 4G
# 浏览器打开终端打印的地址 (默认 http://<IP>:8080)
```

**Android 手机 (Termux)，接入主平台：**

```bash
curl -fsSL https://raw.githubusercontent.com/doghelWang/agv_sim/main/deploy/android/install.sh -o install.sh
bash install.sh --repo https://github.com/doghelWang/agv_sim.git --name phone1 --hub http://<主平台IP>:8080
```

然后在平台「部署仿真」里选：仿真节点 = 手机，执行节点 = 板卡 (推荐组合，见 [性能对比](docs/PERFORMANCE.md))。

## 文档

| 文档 | 内容 |
|---|---|
| [docs/DEPLOY.md](docs/DEPLOY.md) | **部署指南**：支持的设备、Linux 板卡一键部署、加节点、更新、端口、故障处理 |
| [docs/DEPLOY_ANDROID.md](docs/DEPLOY_ANDROID.md) | **Android 手机部署**：Termux + proot、一键安装、幽灵进程限制、proot 派生服务、散热 |
| [docs/PERFORMANCE.md](docs/PERFORMANCE.md) | 手机 vs 树莓派 5 性能对比与推荐组合 |
| [docs/PLATFORM.md](docs/PLATFORM.md) | 平台与工作台功能、路径规划与精确跟踪、保护空间 |
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | 进程架构与设计取舍 |
| [docs/CODEBASE.md](docs/CODEBASE.md) | 代码框架：每个文件的职责、依赖库、调用链 |
| [docs/API.md](docs/API.md) | REST 接口 |
| [docs/ENGINE_EVALUATION.md](docs/ENGINE_EVALUATION.md) | 仿真引擎选型 (MuJoCo vs PyBullet) |
| [docs/BACKLOG.md](docs/BACKLOG.md) | 待办与已知问题 |

## 目录

```
hub/ agent/            资源平台、节点代理
sim_server/ sim_core/  仿真进程、仿真内核 (MuJoCo、传感器)
nav_runtime/ nav2/     执行进程、Nav2/定位栈启动文件
planning/ sensors/     规划、IO 仿真
common/                REST 通信、主机信息、进程派生
web/ vendor/           前端 (平台 + 工作台)
docker/ deploy.sh      镜像与 Docker 部署
deploy/linux/          Linux 一键部署
deploy/android/        Android 安装与启停脚本
tools/ tests/          工具 (精度/性能测试、Nav2 参数生成 …)、测试
```

## 测试

```bash
python3 tests/test_sim_core.py        # 离线单测 (无需 ROS)
python3 tests/test_slam.py            # 内置 SLAM (离线)
python3 tests/test_platform_e2e.py    # 平台端到端 (无需 Docker/ROS)
python3 tools/precision_test.py --gw http://<仿真节点IP>:<Web端口> --planner nav2   # 导航与定位精度
```

依赖：Python 3.10、numpy、mujoco ≥ 3.3、Pillow、psutil、protobuf；执行进程另需 ROS 2 Humble + Nav2 + slam_toolbox + robot_localization (Docker 镜像与手机安装脚本已包含)。
