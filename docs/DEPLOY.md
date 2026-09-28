# 部署指南

本系统由 5 个进程组成，进程之间只用 HTTP REST 通信 (架构见 [ARCHITECTURE.md](ARCHITECTURE.md))：

| 进程 | 作用 | 运行位置 |
|---|---|---|
| 资源平台 agv-hub | 节点、车辆模型、场景、程序包、实例编排、网页前端 | 一个集群一个 (主平台) |
| 节点代理 agv-agent | 在本机启停仿真/执行进程，向平台汇报资源 | 每个计算节点一个 |
| 仿真进程 sim_server + Web 网关 | MuJoCo 物理、激光/相机/IO 仿真、设备端工作台 | 实例的「仿真节点」 |
| 执行进程 nav_runtime | 定位 (slam_toolbox)、规划、导航 (Nav2 / 自研导引)、任务流 | 实例的「执行节点」 |

一次部署 (实例) = 仿真进程 + 执行进程，可以在同一台节点，也可以分在两台节点。

## 1. 支持的设备

| 设备 | 系统 | 运行方式 | 适合做 | 文档 |
|---|---|---|---|---|
| RK3588 板卡 (Orange Pi 5 / Rock 5 / 香橙派等) | Ubuntu 22.04/24.04、Debian 11/12、Armbian (64 位) | Docker | 主平台、执行节点、仿真节点 | 本文第 2、3 节 |
| 树莓派 5 (4 GB 以上) | Raspberry Pi OS 64 位 / Ubuntu | Docker | 同上 (4 GB 需加 swap) | 本文第 2、3 节 |
| x86_64 电脑 / 虚拟机 | Ubuntu / Debian | Docker | 同上 | 本文第 2、3 节 |
| Android 手机 (骁龙 855 及以上，8 GB 内存推荐) | Android 10+ + Termux | proot Ubuntu 22.04 (无需 root) | **仿真节点** (推荐)；也可当平台 | [DEPLOY_ANDROID.md](DEPLOY_ANDROID.md) |

推荐组合 (实测见 [PERFORMANCE.md](PERFORMANCE.md))：**板卡当主平台 + 执行节点，手机当仿真节点**。
执行进程 (ROS 2 / Nav2) 放在手机上时，受 proot 系统调用开销影响，Nav2 控制回路只有 20~30 Hz，跟线误差会到 200 mm 以上。

硬件要求 (Linux 板卡)：64 位 ARM 或 x86_64；内存 ≥ 4 GB (构建镜像时建议 ≥ 6 GB 或加 4 GB swap)；可用磁盘 ≥ 12 GB；能访问 Docker Hub、Ubuntu/ROS 软件源 (国内默认用清华镜像)。

### 1.1 系统与网络要求 (部署前逐条确认)

| 项 | 要求 | 说明 |
|---|---|---|
| 系统 | **64 位** Ubuntu 20.04/22.04/24.04、Debian 11/12、Armbian、Raspberry Pi OS 64 位 (有 `apt`、`systemd`) | RK3588 厂商的 Buildroot/Yocto 精简系统通常没有 apt、内核不带 Docker 所需功能 (overlayfs、cgroup、veth)，请刷 Ubuntu/Debian 系镜像 (如 Joshua-Riek ubuntu-rockchip、Armbian、厂商 Debian) |
| 权限 | root 或有 sudo 的用户 | 装 Docker、建 swap |
| 基础工具 | `git`、`curl` | `sudo apt-get install -y git curl` |
| 磁盘 / 内存 | 可用 ≥ 12 GB；内存 ≥ 4 GB (构建时 < 6 GB 加 `--swap 4G`) | 三个镜像约 4.5 GB |
| 访问 GitHub | 克隆与更新代码 | 国内直连常失败：`git config --global http.proxy http://<代理IP:端口>` 后再 clone；或在能上网的电脑上下载 zip 拷过去 |
| 访问 Docker Hub | 拉基础镜像 `ros:humble-ros-base`、`python:3.10-slim` (约 1 GB) | 国内直连常失败：`setup.sh` 检测到连不上时自动改用 `docker.m.daocloud.io`；也可指定 `DOCKER_MIRROR=<镜像站>` 或 `BASE_REGISTRY=<镜像站>`，或在 `/etc/docker/daemon.json` 配 `registry-mirrors` |
| 访问 Ubuntu / ROS / PyPI 源 | 构建时装 Nav2、MuJoCo 等 | 默认清华镜像 (国内直连可用)；海外用 `--official` |

除此之外不需要预装任何东西：ROS 2、Nav2、MuJoCo、Python 依赖都在 Docker 镜像里，由 `setup.sh` 自动构建。

## 2. Linux 板卡一键部署 (RK3588 / 树莓派 / x86)

```bash
sudo apt-get update && sudo apt-get install -y git curl
# 国内网络连不上 GitHub 时先: git config --global http.proxy http://<代理IP:端口>
git clone https://github.com/doghelWang/agv_sim.git ~/agv_sim
cd ~/agv_sim

# 第一台: 当主平台 (资源平台 + 本机节点代理)
bash deploy/linux/setup.sh                 # 内存 < 6 GB 时加 --swap 4G；海外网络加 --official
```

`setup.sh` 依次完成：检查架构/内存/磁盘 → 安装 Docker (`apt install docker.io`) 并启动 → (可选) 创建 swap → 构建三个镜像 → 启动平台与本机节点代理 → 打印状态。

- 首次执行若提示「已把用户加入 docker 组」，重新登录后再执行一次。
- 构建需下载约 2 GB，RK3588 约 20~40 分钟，树莓派 5 约 30~60 分钟。
- 完成后浏览器打开终端打印的地址 (默认 `http://<板卡IP>:8080`，端口被占用时自动顺延，实际端口记录在 `.hub.env`)。
- 集群令牌在 `~/.agv-hub/cluster_token`，其它节点接入时使用 (也可以在平台「计算资源 → 添加计算节点」生成一次性令牌)。

**再加一台 Linux 节点** (接入已有主平台)：

```bash
git clone https://github.com/doghelWang/agv_sim.git ~/agv_sim && cd ~/agv_sim
bash deploy/linux/setup.sh --hub http://<主平台IP>:8080 --name board2      # 会提示输入集群令牌 (不回显)
```

**加一台 Android 手机**：见 [DEPLOY_ANDROID.md](DEPLOY_ANDROID.md)，安装时加 `--hub http://<主平台IP>:8080`。

### 2.1 部署一个仿真实例

浏览器打开主平台 →「仿真运行列表 → 部署仿真」：选车辆模型 (首次启动已自动入库示例车模)、场景、**仿真节点**、**执行节点**、程序包，提交后平台按步骤显示进度：资源校验 → 分配端口 → 准备镜像 → 启动仿真 → 仿真就绪 → 启动执行 → 执行就绪 → Nav2 就绪。
就绪后点「进入工作台」即可下发任务、查看 3D 视图、记录与回放 (功能见 [PLATFORM.md](PLATFORM.md))。

程序包：节点上已有的镜像会自动登记 (Docker 节点为 `agv-sim/agv-nav:latest`，手机为 `process:sim/nav` 源码包)。
新节点没有镜像时，先在「软件程序包」里把已有节点的镜像「入库」，部署时平台会分发 (镜像架构必须与节点一致)。

### 2.2 验证

```bash
bash deploy.sh status                                   # 容器与健康检查
python3 tools/precision_test.py --gw http://<仿真节点IP>:<实例Web端口> --planner nav2    # 导航/定位精度 (实例端口见平台实例卡片)
```

在执行节点上检查定位/导航栈：`bash tools/check_loc_stack.sh agv-nav-<实例ID> <执行端口>`。

## 3. 更新代码

| 设备 | 命令 |
|---|---|
| Linux 板卡 | `cd ~/agv_sim && git pull && bash tools/refresh_images.sh` (只刷新镜像代码层，几十秒)；依赖有变化时 `bash deploy.sh build all`。平台/代理本身有改动时 `bash deploy.sh build platform && bash deploy.sh hub` (或 `agent …`) |
| Android 手机 | `bash ~/update_from_git.sh --restart` |

更新后在平台上重启实例 (实例卡片「重启」) 生效。

## 4. 不用平台的单机方式

```bash
bash deploy.sh up          # 同机两容器: agv-sim (仿真 :8090 + Web :8088) 与 agv-nav (执行 :8091)
bash deploy.sh status
bash deploy.sh down
```

分机：仿真机 `NAV_API=http://<执行机IP>:8091 bash deploy.sh up sim`，执行机 `SIM_API=http://<仿真机IP>:8090 bash deploy.sh up nav`。
不能联网构建的机器：在同架构机器上 `bash deploy.sh save` 导出，拷过去 `bash deploy.sh load`。
本机装有 ROS 2 Humble (Ubuntu 22.04 裸机) 时也可以不用 Docker：`bash start_sim.sh` 直接起三进程。

### 4.1 无 Docker、不能联网、重启即还原的板卡 (RK3588 车载控制器，chroot)

实测板卡 (192.168.1.64)：busybox + dropbear，根文件系统在内存里 (重启恢复出厂)，只有 `/mnt` (ext4) 持久；
没有 Docker / gcc / bash，有 `chroot`、`xz`。板上跑着原有业务进程，部署不碰它们，全部放在 `/mnt/agv_ros`，删掉该目录即卸载。

```bash
# 1) 能联网的 arm64 机器 (Apple 芯片 Mac + Docker Desktop，约 10 分钟)：构建 agv-nav 并 docker export → 406 MB 的 tar.xz
bash deploy/rk3588_offline/pack.sh dist/rk3588
# 2) 拷到板卡 (dropbear 没有 sftp-server，scp 不可用，用管道) 并解包 (约 50 s，解包后 2.4 GB)
ssh root@<板卡> 'mkdir -p /mnt/agv_ros'
cat dist/rk3588/agv_ros.sh | ssh root@<板卡> 'cat > /mnt/agv_ros/agv_ros.sh'
cat dist/rk3588/agv-nav-rootfs.tar.xz | ssh root@<板卡> 'cat > /mnt/agv_ros/agv-nav-rootfs.tar.xz'
ssh root@<板卡> 'cd /mnt/agv_ros && mkdir -p rootfs && xz -dc agv-nav-rootfs.tar.xz | tar -x -C rootfs'
ssh root@<板卡> 'cp /root/.ssh/authorized_keys /mnt/agv_ros/'   # 公钥备份，重启后由 boot 恢复
# 3) 每次重启后在板卡本地执行一次：恢复 SSH 公钥 (含 chmod 755 /root，dropbear 拒绝组/其他可写的家目录) + 挂载 + 启动执行进程
sh /mnt/agv_ros/agv_ros.sh boot          # 其余: start | stop | status | shell
```

执行进程与 DDS 只监听/只走 127.0.0.1 (`AGV_BIND=127.0.0.1`、`ROS_LOCALHOST_ONLY=1`)，不向车载网络发包。
仿真放在另一台机器 (如 Mac: `docker run -p 127.0.0.1:8090:8090 -p 127.0.0.1:8088:8088 -e SIM_CMD_UDP=0 -e NAV_API=http://host.docker.internal:8091 agv-sim:latest`)，
两边用 SSH 隧道接通，不对局域网开端口 (UDP 速度指令过不了 SSH 隧道，所以 `SIM_CMD_UDP=0`，执行进程改用 REST 下发)：
`ssh -N -R 127.0.0.1:8090:127.0.0.1:8090 -L 127.0.0.1:8091:127.0.0.1:8091 root@<板卡>`。

验证 (2026-09-28，RK3588 执行 + M1 Mac 仿真)：Nav2 启动到激活约 1 分钟，执行进程收到状态 49.6 Hz、链路 0 错误，板卡整机 CPU 空闲约 70%；
`GW=http://127.0.0.1:8088 NAV=http://127.0.0.1:8091 NAV_LOG="ssh root@<板卡> cat /mnt/agv_ros/logs/nav.log" bash tools/quick_nav_check.sh`
两次均 3/3 到达，终点误差 42~74 mm (贴墙工位) / 18 mm (P0)，定位误差最大 54~85 mm，与树莓派上贴墙工位的已知问题 (NAVIGATION.md 待查项) 同一现象；
controller_server 30 Hz 控制频率告警约 175 次 / 4 分钟。

## 5. 端口

| 端口 | 进程 | 说明 |
|---|---|---|
| 8080 (Linux) / 8082 (手机) | 资源平台 | 被占用时自动顺延 |
| 8070 | 节点代理 | |
| 8100~8199 | 实例 | 每个实例 3 个: 仿真 REST / Web 网关 / 执行 REST |
| 8069 | 手机 proot 派生服务 | 只监听 127.0.0.1 |
| 8088 / 8090 / 8091 | 单机方式 (`deploy.sh up`) | Web / 仿真 / 执行 |

所有容器使用 host 网络；ROS 2 只在本机通信 (Linux: `ROS_LOCALHOST_ONLY=1`；手机: 只走 127.0.0.1 的 DDS 配置)，同一局域网的多台设备不会串话。

## 6. 数据与配置

| 位置 | 内容 |
|---|---|
| `~/.agv-hub/` (主平台) | 平台数据库 hub.db、模型/场景/程序包仓库、`cluster_token` |
| `~/.agv-agent/` (每个节点) | 节点密钥、实例数据 (`instances/<实例>/sim|nav`，SLAM 地图在 nav/slam_maps) |
| 项目目录 `data/` | 单机方式的持久化数据 |

常用环境变量：

| 变量 | 默认 | 说明 |
|---|---|---|
| `APT_MIRROR` / `PIP_INDEX` | 清华镜像 | 构建时的软件源，设为空字符串用官方源 |
| `BUILD_PROXY` | 不用 | 构建时的 HTTP 代理，例如 `http://192.168.1.2:7890` |
| `BASE_REGISTRY` | 不用 (Docker Hub) | 基础镜像 (python、ros) 的镜像站，如 `docker.m.daocloud.io`、`mirror.gcr.io` |
| `SIM_CAMERA_RENDER` | `ray` | 相机成像；`gl` 需要 EGL |
| `NAV_USE_ROS` | 1 | 0 = 执行进程不启 ROS，只用内置 SLAM + 自研导引 |
| `LOC_ENGINE` | auto | `builtin` 强制内置 SLAM |
| `ROS_DOMAIN_ID` | 平台按实例分配 | |

## 7. 故障处理

| 现象 | 原因 / 处理 |
|---|---|
| 构建 agv-nav 时板卡卡死、SSH 无响应 | 内存耗尽。加 swap (`setup.sh --swap 4G`)，或在别的同架构机器构建后 `save/load` |
| 拉取基础镜像失败 / `429 Too Many Requests` | Docker Hub 不通或限流：`BASE_REGISTRY=docker.m.daocloud.io bash deploy/linux/setup.sh` (或 `mirror.gcr.io`) |
| 构建时 apt/pip 下载失败 | 国内默认清华镜像；海外用 `--official`；需要代理时 `BUILD_PROXY=http://<代理> bash deploy.sh build all` |
| 平台打不开 8080 | 端口被占用时平台会顺延，看 `.hub.env` 或 `bash deploy.sh status` |
| 部署卡在「Nav2 就绪」 | 执行节点负载过高或 Nav2 生命周期启动失败；执行进程有看门狗会自动重启 Nav2 (最多 3 次)。看平台实例日志 (执行) |
| 跨节点实例连不上 | 平台按节点注册的局域网地址互访；多网卡时在代理上设 `AGENT_HOST=<局域网IP>` (`deploy.sh agent` 前 export) |
| 定位漂移 / 精度差 | 先看 [PERFORMANCE.md](PERFORMANCE.md) 的组合建议；执行节点 CPU 占满时 slam_toolbox 会滞后 |
| 手机相关 | 见 [DEPLOY_ANDROID.md](DEPLOY_ANDROID.md#8-故障处理) |
