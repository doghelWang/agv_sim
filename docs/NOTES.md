# 注意事项

部署、更新和日常使用中容易踩的坑。操作步骤见 [DEPLOY.md](DEPLOY.md) / [DEPLOY_ANDROID.md](DEPLOY_ANDROID.md)，这里只列需要特别留意的地方。

## 1. 网络 (国内环境最常见的问题)

| 访问目标 | 用在哪里 | 国内直连 | 处理 |
|---|---|---|---|
| GitHub (github.com、raw.githubusercontent.com) | 克隆/更新代码、手机下载 install.sh、下载 ROS 签名密钥 | 经常不通或很慢 | Linux: `git config --global http.proxy http://<代理>`；手机: `install.sh --proxy http://127.0.0.1:7890` (写入 `~/.agv.env` 的 `AGV_PROXY`，之后更新也走代理) |
| Docker Hub | 构建时拉 `ros:humble-ros-base`、`python:3.10-slim` | 不通 | `setup.sh` 检测到连不上会自动改用 `docker.m.daocloud.io`；也可 `DOCKER_MIRROR=<镜像站>` 或在 `/etc/docker/daemon.json` 配 `registry-mirrors` |
| 清华镜像 (Ubuntu/ROS/PyPI/Termux) | 构建镜像、手机装 ROS | 可用 | 默认使用；海外网络用 `setup.sh --official` / `install.sh --mirror official` |

- git 经代理出现 `HTTP2 framing layer` / `Connection reset` 时加 `-c http.version=HTTP/1.1`。
- 代理是某台设备提供的 (例如手机上的 sing-box 开放 7890 端口给局域网)，该设备关机时其它设备就拉不了 GitHub。长期使用建议板卡自己配代理，或把仓库镜像到 Gitee。
- 设备之间 (平台 ↔ 节点、仿真 ↔ 执行) 只走局域网 HTTP，不需要外网；外网只在安装和更新时需要。

## 2. 系统与硬件

- **只支持 64 位** (aarch64 / x86_64)。RK3588 请用 Ubuntu / Debian / Armbian 系统镜像；厂商 Buildroot/Yocto 精简系统通常没有 apt、内核缺 Docker 需要的功能，不支持。
- **内存 4 GB 的板子构建镜像前加 swap** (`setup.sh --swap 4G`)，否则构建 Nav2 镜像时可能内存耗尽、SSH 都连不上，只能断电重启。
- 磁盘至少留 12 GB (镜像约 4.5 GB，加构建缓存)；手机上 proot 容器约 3 GB。
- 树莓派的 8080 端口常被系统自带的管理页面占用，平台会自动顺延 (8081、8082…)，实际端口看项目目录下 `.hub.env` 或 `bash deploy.sh status`。**其它节点接入时填的是这个实际端口**。

## 3. 部署组合

- **推荐：板卡跑执行 (ROS 2 / Nav2 / slam_toolbox)，手机跑仿真**。数据见 [PERFORMANCE.md](PERFORMANCE.md)。
- **不要把执行进程放在手机上**：proot 下执行进程状态频率只有 20~30 Hz，Nav2 跟线偏差会到 200~300 mm；手机全包 (仿真 + 执行) 时定位误差可达 270 mm。
- 树莓派 4 核同时跑仿真和执行时 CPU 会跑满 (94%)，Nav2 控制循环频繁超时，精度下降。
- 平台里每个节点默认「可并行实例数」为 1：一台设备上已有实例在跑时，部署新实例会被拒绝，先在平台上停掉旧实例，或在「计算资源」里调大并行数 (注意 CPU)。
- 跨设备实例依赖节点注册的局域网地址。多网卡 (有线 + 无线、docker 网桥) 时，在节点上设置 `AGENT_HOST=<局域网IP>` 再启动代理，否则另一台设备可能连到错误地址。

## 4. 手机 (Android)

- **必须关闭幽灵进程限制** (Android 12+)，否则运行一段时间后 ROS 进程会被系统随机杀掉，表现为 Nav2/定位突然消失。步骤见 DEPLOY_ANDROID.md 第 5 节；手机重启后设置仍在，但**无线调试的连接端口每次都会变**。
- Termux 设为「电池不优化」，保持充电；装 Termux:Boot 才能开机自启。
- **散热**：持续满载时 Pixel 4 会降频 (大核 2.4~2.8 GHz → 1.6 GHz)。建议背夹风扇、去掉手机壳、不要边充电边满载。`bash ~/status_agv.sh` 可看各核频率与热状态。
- `stop_agv.sh` 会结束该 proot 容器 (默认 ubuntu) 的**全部** proot 会话，包括你自己开的 `proot-distro login` 终端。需要在容器里手动调试时，另建一个容器 (`proot-distro install -n dev ubuntu:22.04`)。
- 不要在 proot 容器里直接运行 `pip`：会报 `Cannot find path to android app folder`。要用 `env -u ANDROID_DATA -u ANDROID_ROOT python3 -m pip …`。
- 不要在 proot 里设置 `ROS_LOCALHOST_ONLY=1`：Android 回环网卡不支持组播，ROS 节点会互相发现不全。执行进程在 Android 上已自动改用 `deploy/android/fastdds_ds.xml` (发现服务器)。
- 容器里自己跑 ROS 命令 (`ros2 topic echo` 等) 时，要和实例使用相同的 `ROS_DOMAIN_ID` (平台按实例分配，见实例详情) 和同一份 DDS 配置 (`FASTRTPS_DEFAULT_PROFILES_FILE=/opt/agv/deploy/android/fastdds_ds.xml` 加 `ROS_DISCOVERY_SERVER=127.0.0.1:<11811 + ROS_DOMAIN_ID>`；执行进程日志开头会打印这几个值)，否则看不到任何话题。发现服务器模式下普通客户端只看到与自己匹配的端点，`ros2 topic list` 可能不全。
- 2026-09-27 之前安装的手机，容器里是 Ubuntu 自带的参考 BLAS (numpy 矩阵运算慢约 20 倍)，补装一次：
  `proot-distro login ubuntu -- apt-get install -y libopenblas0-pthread` (新安装的已包含)。
- 首次从旧版本升级 `update_from_git.sh` 时可能报一次 `unexpected EOF` (脚本在运行中覆盖了自己)，再执行一次即可；之后的版本已避免。

## 5. 更新代码

| 设备 | 命令 | 注意 |
|---|---|---|
| Linux 板卡 | `git pull && bash tools/refresh_images.sh` | 只刷新镜像里的代码层，几十秒；依赖 (Dockerfile、apt/pip 包) 有变化时要 `bash deploy.sh build all`。平台/代理代码 (`hub/`、`agent/`) 有变化时要 `bash deploy.sh build platform` 后重新 `deploy.sh hub` / `agent` |
| 手机 | `bash ~/update_from_git.sh --restart` | 保留本机的模型补全 (`model_overrides.json` 等) 与数据 |

- **更新后要在平台上重启实例**才会用上新代码；正在跑的实例不会自动更新。
- `refresh_images.sh` 每次叠加一层镜像，长期使用后可 `bash deploy.sh build all` 完整重建一次。
- 本机修改过的 `robot_config.json`、`model_overrides.json`、`robot.urdf` 是设备本地配置，`git pull` 冲突时先 `git stash`，拉取后 `git stash pop`。

## 6. 安全

- **集群令牌** (`~/.agv-hub/cluster_token`) 等同于接入平台的密码：不要提交到 git、不要发到群里。手机上写在 `~/.agv.env` (权限 600)。临时接入可用平台「添加计算节点」生成的一次性令牌 (24 小时有效)。
- 平台、节点代理、实例接口都在局域网内无认证开放 (除节点代理需节点密钥)，**不要把这些端口映射到公网**。
- 仓库是公开的，提交前检查不要带上内网地址、账号、令牌、代理配置。

## 7. 数据与备份

| 位置 | 内容 | 建议 |
|---|---|---|
| 主平台 `~/.agv-hub/` | 平台数据库、模型/场景/程序包仓库、仿真记录、集群令牌 | 定期备份整个目录；换主平台设备时整体拷过去 |
| 各节点 `~/.agv-agent/` | 节点密钥、实例数据 (含 SLAM 地图 `instances/<实例>/nav/slam_maps`) | 删除后节点需要重新用令牌接入 |
| 手机 | 以上两个目录在 proot 容器内 `/root/` 下 (Termux 路径 `$PREFIX/var/lib/proot-distro/containers/ubuntu/rootfs/root/`) | 停止节点后打包该目录备份 |

- SLAM 地图目前按实例保存，删除实例记录或换执行节点后需要重新建图 (见 BACKLOG T14)。

## 8. 相机与性能开关

- **相机按需成像**：仿真进程只在最近 3 s 内有人取帧时才渲染相机 (一帧射线渲染要 150~200 ms，一直渲染会占 1~2 个核)。
  工作台打开相机画面、或 ROS 里有节点订阅 `/<相机名>/image_raw` 等话题时自动恢复，首帧约 0.2~1 s。
  `/api/v1/sensors/cameras` 返回的 `active` 表示当前是否在成像。
- 相关环境变量 (都在实例环境里设置，默认值已按测试结果选定)：

| 变量 | 默认 | 说明 |
|---|---|---|
| `SIM_CAMERA_IDLE_S` | 3 | 相机多少秒无人取帧就停止成像；0 = 一直渲染 (旧行为) |
| `NAV_CAMERAS` | auto | 执行进程拉取相机：auto = 有 ROS 订阅者才拉，on = 一直拉，off = 不拉 |
| `SIM_RAY_THREADS` | min(2, 核数) | 相机/3D 激光射线并行线程数 |
| `OPENBLAS_NUM_THREADS` / `OMP_NUM_THREADS` | 1 | 数学库线程数 (入口脚本设置) |
| `AGV_CPUS_SIM` / `AGV_CPUS_WEB` / `AGV_CPUS_NAV` | 大核 / 小核 / 不绑定 | 仅手机：绑核，见 DEPLOY_ANDROID.md 第 6 节 |

## 9. 精度与测试

- 精度考核 `tools/precision_test.py` 统计的是**真值**相对参考线的偏差；导航用的是**定位结果** (slam_toolbox)，不是真值。定位误差会直接反映到跟线误差上。
- 刚部署的实例处于**建图模式**，前几圈定位误差偏大；建好地图后保存地图并切换到定位模式再考核 (执行进程接口 `POST /api/v1/slam/save`、`POST /api/v1/slam/mode {"mode":"localization"}`)。
- 测试会占用实例：跑 `perf_matrix.py` 会依次停止平台上正在运行的实例，结束后再恢复。
- 温度会影响手机上的结果，对比测试前让手机降温，并记录 `status_agv.sh` 显示的频率。
