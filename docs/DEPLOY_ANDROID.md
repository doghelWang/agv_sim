# Android 手机部署 (Termux + proot Ubuntu，无需 root)

手机上用 Termux 跑一个 proot Ubuntu 22.04 容器，里面装 ROS 2 Humble、Nav2、MuJoCo 和本项目代码；
节点代理用 process 运行时 (不用 Docker) 直接起进程。手机可以：

- **作为计算节点接入主平台 (推荐)**：主平台在 Linux 板卡上，手机跑仿真进程，执行进程放在板卡上；
- 自己当主平台 (单手机演示)：平台、仿真、执行都在手机上，Nav2 精度明显变差 (见 [PERFORMANCE.md](PERFORMANCE.md))。

已验证机型：Pixel 4 (骁龙 855，6 GB，Android 13)。其它 64 位 Android 10+ 手机同理，建议 8 核、6 GB 内存以上。

## 1. 准备

1. 安装 **Termux** 与 **Termux:Boot** (开机自启，可选)、**Termux:API** (传感器，可选)，都从 F-Droid 或 GitHub Releases 安装 (不要用 Google Play 版，已停止更新)，三者必须来自同一来源。
2. 手机设置：
   - 电池 → Termux 设为「不优化 / 无限制」；
   - 开发者选项 → 打开「无线调试」(第 5 节用于关闭幽灵进程限制)。
3. 保持充电、散热 (背夹风扇或去掉手机壳)：持续高负载时手机会降频，见第 7 节。

## 2. 一键安装

在 Termux 里执行 (不要进 proot)：

```bash
pkg update -y && pkg install -y curl
curl -fsSL https://raw.githubusercontent.com/doghelWang/agv_sim/main/deploy/android/install.sh -o install.sh

# 作为计算节点接入主平台 (推荐)
bash install.sh --repo https://github.com/doghelWang/agv_sim.git --name phone1 --hub http://<主平台IP>:8080
# 或: 手机自己当主平台
bash install.sh --repo https://github.com/doghelWang/agv_sim.git --name phone1
```

`--hub` 时会提示输入集群令牌 (主平台上 `cat ~/.agv-hub/cluster_token`，或平台「计算资源 → 添加计算节点」生成的一次性令牌)。
海外网络加 `--mirror official`。仓库是私有的时，把 `--repo` 换成带令牌的地址或先配置 git 凭据。

安装过程 (约 20~40 分钟，下载约 700 MB，占用约 3 GB)：

| 步骤 | 内容 |
|---|---|
| 1 | Termux 软件包：proot-distro、python、git、openssh、termux-api、android-tools、net-tools |
| 2 | proot 容器：`proot-distro install -n ubuntu ubuntu:22.04` (proot-distro ≥ 5；旧版本先 `pkg upgrade proot-distro`) |
| 3 | 克隆代码到容器内 `/opt/agv-git` |
| 4 | 容器内 `deploy/android/proot_setup.sh`：清华/官方源，ROS 2 Humble (ros-base、navigation2、nav2-bringup、robot-state-publisher、slam-toolbox、robot-localization)，numpy/Pillow/psutil/protobuf、OpenBLAS，pip 装 MuJoCo |
| 5 | 同步到 `/opt/agv`，Termux 侧脚本复制到 `~`，开机自启脚本到 `~/.termux/boot/`，写配置 `~/.agv.env` |
| 6 | 启动 (`~/start_agv.sh`) |

安装完成后，主平台「计算资源」里会出现该手机节点 (运行时 process)，「软件程序包」里会自动登记 `process:sim` / `process:nav` 源码包。

## 3. 目录与脚本

| 位置 | 说明 |
|---|---|
| `~/.agv.env` | 配置：容器名、仓库地址、节点名、主平台地址与令牌 (chmod 600) |
| `~/start_agv.sh` / `~/stop_agv.sh` / `~/status_agv.sh` | 启动 / 停止 / 状态 |
| `~/update_from_git.sh [--restart]` | 从 git 更新代码 (保留设备本地的模型补全与数据) |
| `~/proot_spawner.py` | proot 派生服务 :8069 (第 6 节) |
| `~/hub.log` `~/agent.log` `~/spawner.log` | 平台 / 节点代理 / 派生服务日志 |
| 容器内 `/opt/agv` | 运行中的代码 (Termux 路径 `$PREFIX/var/lib/proot-distro/containers/ubuntu/rootfs/opt/agv`) |
| 容器内 `/opt/agv-git` | git 克隆 |
| 容器内 `/root/.agv-agent/` | 节点密钥、实例数据、实例日志 (`logs/agv-sim-<实例>.log`、`agv-nav-<实例>.log`) |
| 容器内 `/root/.agv-hub/` | 手机当主平台时的平台数据 |

`~/.agv.env` 示例：

```bash
AGV_DISTRO=ubuntu
AGV_GIT_REMOTE=https://github.com/doghelWang/agv_sim.git
AGV_GIT_BRANCH=main
AGENT_NAME=phone1
AGV_HUB_PORT=8082                 # 手机当主平台时的端口
HUB_API=http://192.168.1.10:8080  # 删掉这两行 = 手机自己当主平台
JOIN_TOKEN=xxxxxxxx
# AGENT_HOST=192.168.1.20         # 可选，缺省自动取 wlan0 地址
```

修改后 `bash ~/stop_agv.sh && bash ~/start_agv.sh` 生效。

## 4. 部署实例

在主平台「部署仿真」里：**仿真节点选手机、执行节点选板卡** (推荐)，程序包分别选 `process:sim` 与板卡的 `agv-nav`。
手机当主平台时浏览器打开 `http://<手机IP>:8082`。

## 5. 关闭幽灵进程限制 (Android 12+ 必做)

Android 12 起系统限制 App 的子进程数 (默认 32)，超过会被随机杀掉。一个实例在手机上有二三十个进程 (每个 ROS 节点一个 proot 会话)，必须关闭。

**不用电脑** (Termux 自带 adb，手机自己连自己)：

1. 开发者选项 →「无线调试」→ 打开 →「使用配对码配对设备」，记下 **配对端口** 和 6 位配对码；
2. Termux 里：`adb pair 127.0.0.1:<配对端口>`，输入配对码；
3. 回到「无线调试」主页面，记下「IP 地址和端口」里的 **连接端口** (与配对端口不同)：`adb connect 127.0.0.1:<连接端口>`；
4. 执行：

```bash
adb shell device_config set_sync_disabled_for_tests persistent
adb shell device_config put activity_manager max_phantom_processes 2147483647
adb shell settings put global settings_enable_monitor_phantom_procs false
adb shell dumpsys deviceidle whitelist +com.termux
```

以上设置重启后仍有效 (第一条防止系统同步把配置改回去)。Android 14+ 也可以在开发者选项里打开「停用子进程限制」。
可选：`adb tcpip 5555` 后电脑可以 `adb connect <手机IP>:5555` 调试 (重启后失效)。

## 6. 为什么要 proot 派生服务

proot 用 ptrace 转发系统调用，**一个 proot 会话只有一个追踪线程**。所有进程挤在一个会话里时，8 核手机实际只用上约 1 核 (实测 RTF 掉到 0.07)。
`start_agv.sh` 先在 Termux 原生 Python 里启动派生服务 (`~/proot_spawner.py`, 127.0.0.1:8069)，节点代理和执行进程通过它 (`common/spawn.py`) 把仿真、网关、执行进程以及每个 ROS 节点各放进独立 proot 会话。

```bash
curl -s 127.0.0.1:8069/list          # 查看会话，如 agv-nav-i03/nav2/controller_server
AGV_SPAWNER_OFF=1 bash ~/start_agv.sh   # 关闭 (回到单会话，只用于排查)
```

同理，Android 的回环网卡不支持组播，`ROS_LOCALHOST_ONLY=1` 下十几个 ROS 进程发现不全；执行进程在 Android 上自动改用
`deploy/android/fastdds_localhost.xml` (只走 127.0.0.1，单播发现 120 个参与者)。其它 Android 专用调整 (都在代码里自动生效)：
不启 EKF (执行进程直接发布里程计 TF)、全局代价地图不加障碍层、slam_toolbox 的 map→odom 外推 0.8 s、Nav2 控制 20 Hz、关闭 bond 心跳。

## 7. 性能与散热

- 手机单核算力与树莓派 5 接近，优势是核多；proot 本身会占用 20~50% CPU。数据见 [PERFORMANCE.md](PERFORMANCE.md)。
- 持续满载时 Pixel 4 会因机身温度触发降频 (大核从 2.4~2.8 GHz 降到 1.6 GHz)。`bash ~/status_agv.sh` 显示各核频率；已配对 adb 时还显示热状态 (0 正常，3 严重)。
- 建议：背夹风扇、不要边充边满载、仿真节点只放一个实例。

## 8. 故障处理

| 现象 | 处理 |
|---|---|
| `proot-distro install` 装的不是 22.04 | 升级 proot-distro (`pkg upgrade proot-distro`，需要 ≥ 5)，`install.sh --distro 新名字` 重装 |
| 容器内 `pip` 报 `Cannot find path to android app folder` | proot 里的 ANDROID_* 环境变量让 pip 误判平台；用 `env -u ANDROID_DATA -u ANDROID_ROOT python3 -m pip install …` (proot_setup.sh 已处理) |
| 下载 ROS 密钥失败 | 网络问题；`proot_setup.sh` 会依次尝试 GitHub 与清华镜像，也可手动把 `ros.key` 放到容器内 `/usr/share/keyrings/ros-archive-keyring.gpg` |
| 进程运行一段时间后消失 | 幽灵进程限制未关闭 (第 5 节)，或 Termux 被电池优化 |
| 平台上手机节点离线 | `bash ~/status_agv.sh`；看 `~/agent.log`；检查 `~/.agv.env` 的 HUB_API / 令牌；手机与主平台需在同一局域网 |
| 实例一直「部署中」/ Nav2 未就绪 | 看实例日志；手机负载过高时 Nav2 启动慢，看门狗会自动重启 Nav2；建议执行节点放板卡 |
| `adb connect` 被拒绝 | 连接端口不是配对端口，看「无线调试」主页的「IP 地址和端口」 |
| Termux 里 `adb` 报 `CANNOT LINK EXECUTABLE` | `pkg upgrade` 更新 android-tools 与 libc++ |
| 停止脚本关掉了我自己的 proot 终端 | `stop_agv.sh` 会结束该容器的全部 proot 会话；需要交互调试时另建一个容器 |
