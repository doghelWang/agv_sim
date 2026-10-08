# AMR 仿真平台 —— 接手提示词

> 用法：把下面「提示词正文」整段发给接手的 agent。它不在原来的电脑上、看不到之前的对话，只能访问 GitHub 和手机，所以正文里写全了背景、环境、约束和待办。尖括号里的内容（手机 IP、SSH 公钥、管理员密码）由你在交接时现场提供。

---

## 提示词正文

你接手一个进行中的工程：**AMR (自主移动机器人) 仿真与导航平台**，主要部署在一台三星 Galaxy Z Flip5 手机上（Termux + proot Ubuntu）。你看不到之前的沟通记录，以下是交接时需要知道的全部背景。先通读，再按「第一步」开始。

### 1. 你能用什么、不能用什么

- **能用**：
  - GitHub 仓库 https://github.com/doghelWang/agv_sim（主分支 `main`，最新提交 `7193b30`）。
  - 手机（通过 SSH 登录 Termux）。
- **不能用**：
  - 原来那台电脑及其本地文件，包括 Mac 上的脚本、adb USB 连接、截图工具。
  - 之前的对话记录。
- **需要的东西缺了，直接问用户**，不要猜，也不要绕路获取：
  - 手机当前 IP
  - 把你的 SSH 公钥加进手机
  - 运维管理的管理员密码
  - 需要人在手机上点的操作（安装 APK、无线调试配对等）

### 2. 工作方式（用户的明确要求）

- **语言与风格**：用中文沟通，简洁，先说结论。
- **遇到困难立即说明，不要反复自己重试**：同一条命令出错两次就停下来告诉用户，用户可以替你执行。
- **常规开发操作默认已授权**：改代码、在手机上部署和重启服务、跑测试、提交都可以，不必逐项请示。
- **向 `main` 推送**：先确认测试通过，并在汇报里写清楚推了什么。禁止改写 git 历史、禁止 force push。
- **平台代码更新优先走「运维管理」页面**（见第 6 节），这是用户指定的方式；只有平台已经起不来时才直接 SSH 修复，并说明原因。
- **每次交付都要在手机上实际验证**：说明测了什么、结果如何、还有什么没验证。不确定的事情明说，不要把推测当结论。

### 3. 绝对禁止

- **不得更新手机系统**。OTA 已关闭，不要打开；当前固件 F731BXXU5FYI9（Android 16 / One UI 8）对降级方案很关键。
- **不得改动手机系统设置**，包括开发者选项、电池、网络等。确需改动时，先说明并请用户自己操作。
- **不得做以下操作**：恢复出厂、刷机、解锁 bootloader、root。之前做过刷机/root 的可行性分析，只是分析，没有用户明确指令绝不执行。
- **不得撤销**手机上已有的 adb 授权。
- **不得输出、提交、写进日志或文档的秘密**：
  - 集群令牌（会出现在 `~/hub.log` 和 `/root/.agv-hub/cluster_token` 里，查日志时过滤掉含"令牌"的行）
  - `~/service_native.py` 里的 Telegram bot token
  - RTSP 密码、sing-box 等代理凭据
  - 运维管理员密码
  - 节点的 SSH 密码（加密存放在平台数据目录，密钥是 `secret.key`）
- **不得用会匹配到自己命令行的 `pkill -f` / `pgrep -f`**。这样会把自己的 shell 杀掉，复杂操作写成脚本文件再执行。

### 4. 系统架构（先读仓库里的文档）

必读文档：
- `README.md`
- `docs/ARCHITECTURE.md`
- `docs/CODEBASE.md`（目录说明）
- `docs/DEPLOY_ANDROID.md`
- `docs/NAVIGATION.md`（尤其 §8.7：带车头朝向的规划与分步挪车）
- `docs/FLIP5_PERF_REPORT.md`（手机性能与小核限制）
- `docs/BACKLOG.md`（待办与已知问题）

进程之间只走 HTTP REST，ROS 2 只在执行进程内部使用：

| 组件 | 代码 | 端口（手机上） | 说明 |
|---|---|---|---|
| 资源平台 hub | `hub/` | 8082 | 计算节点、车辆模型、场景、程序包、实例部署、记录；网页在 `web/`；运维管理在 `hub/admin.py` + `web/js/ops.js` |
| 节点代理 agent | `agent/` | 8070 | 在设备上启停实例进程（手机是 process 运行时，Linux 板卡是 Docker） |
| 仿真进程 | `sim_server/`、`sim_core/`、`sensors/` | 每个实例从 8100 起 | MuJoCo 物理、激光、相机、IO |
| Web 网关 | `web_gateway.py`、`gateway_v2.py` | 每个实例 | 设备端工作台、遥测、任务流 |
| 执行进程 | `nav_runtime/`、`planning/`、`ros2/` | 每个实例 | 定位、拓扑规划（Dijkstra，支持倒车段）、导航（Nav2 或内置 C++ 引导） |
| 外屏面板 App | `deploy/android/cover_app/` | — | 包名 `com.agvsim.cover`，当前装的是 1.7（versionCode 8）；悬浮在 Termux 上，保证 Termux 留在前台 |

**实例接口**：
- 任务：`POST :<nav端口>/api/v1/missions {x,y,yaw}`
- 切换规划器：`PUT /api/v1/nav/planner {"type":"nav2"|"dijkstra"}`
- 注入障碍：`PUT :<sim端口>/api/v1/world/obstacles`

**平台接口**：
- 实例：`/api/hub/instances`、`/api/hub/instances/<id>/restart|stop`
- 节点：`/api/hub/nodes`
- 运维：`/api/hub/admin/*`（要登录）

完整接口见 `docs/API.md`；运行中的服务访问 `GET /api/v1` 会列出全部接口。

### 5. 手机环境

**设备与连接**
- 设备：Galaxy Z Flip5（SM-F731B，8 核，约 7 GB 内存）；外屏 748×720，底部 66 px 是摄像头缺口和系统导航键。
- 网络：Wi‑Fi，IP 会变（出现过 .136 / .137），以 `<手机IP>` 为准。
- 登录：Termux 的 SSH 在 **8022 端口**，只认密钥，例如 `ssh -p 8022 <手机IP>`。
- 代码目录：Ubuntu 22.04 容器的 rootfs 在 `$PREFIX/var/lib/proot-distro/containers/ubuntu/rootfs`，代码在容器内的 `/opt/agv`（不是 git 仓库）。
  - 当前版本写在 `/opt/agv/.agv_version`，现在是 `7193b30`。
  - 进容器：`proot-distro login ubuntu`。
  - ROS 2 Humble 在容器内的 `/opt/ros/humble`。

**Termux 主目录里的脚本**（源文件在 `deploy/android/`）
- `start_agv.sh`：启动平台、代理、派生服务、外屏面板、前台看守、安卓助手，并自动拉起实例。
- `stop_agv.sh`：全部停止。
- `status_agv.sh`：查看状态。
- `agv_common.sh`：公共变量。
- `keep_front.sh`：前台看守。
- `proot_spawner.py`：proot 派生服务，端口 8069。
- `android_helper.py`：安卓助手，只监听本机 8067，密钥在 `~/.agv-helper.key`。
- `gpucastd`：GPU 射线服务，端口 8068。
- 开机自启：Termux:Boot（`~/.termux/boot/01-start-agv.sh`）。

**日志**：`~/hub.log`、`~/agent.log`、`~/spawner.log`、`~/agv_boot.log`、`~/autostart.log`、`~/android_helper.log`。

**编译 ROS 插件**（在手机上，约 3~5 分钟，放后台跑）：

```bash
proot-distro login ubuntu -- bash -c "cd /opt/agv/ros2 && . /opt/ros/humble/setup.bash && . install/local_setup.bash; colcon build --packages-select agv_nav2_plugins agv_ros_bridge --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF"
```

**手机特有的坑**（都踩过）
- **小核限制**：Termux 不在前台时，三星会把它限制到小核（cpuset 不再是 `/top-app`），Nav2 会超时、任务失败。外屏面板和 `keep_front.sh` 就是为此存在的。息屏会进入 Doze，同样会限核。
- **不能用 `os.execv`**：多线程进程在 proot 里 exec 会段错误。在 `proot-distro login` 会话里起的子进程，也会随父进程一起被结束（`--kill-on-exit`）。
  - 要另起一个长期运行的进程，用派生服务（`common/spawn.py`）。
  - 服务要重启自己，用 `common.spawn.respawn()`。
- **proot 里不能运行系统命令**：`/system/bin/pm`、`am` 等在 proot 里不能执行（Operation not permitted）。需要安卓操作时，经安卓助手（Termux 原生）完成。
- **启动脚本打印的地址不对**：显示的是 `172.19.0.1`，不是 Wi‑Fi IP（已知问题，见第 8 节）。

### 6. 怎么改代码、怎么发布

**1）在你自己的环境里开发和测试**：克隆仓库，不需要 ROS。

```bash
pip install mujoco numpy ...                     # 按报错补依赖
python3 tests/test_platform_e2e.py               # 平台端到端：节点、部署、任务流、避障、记录
python3 tests/test_agvnav.py; python3 tests/test_cpp_safety.py
cd ros2/agv_nav2_plugins && g++ -O2 -std=c++17 -Iinclude test/test_geom.cpp && ./a.out   # 几何/规划单测
```

- **云端试跑**：起仿真和执行进程，不用 ROS。

  ```bash
  python3 -m sim_server.api --port 8090 --config robot_config.json   # 设 SIM_SCENARIO=narrow_aisle 等
  NAV_USE_ROS=0 NAV_LOCALIZATION=ground_truth SIM_API=http://127.0.0.1:8090 NAV_API_PORT=8091 python3 -m nav_runtime.main
  ```

- **改了网页样式类**：要重新生成离线 CSS。

  ```bash
  npx tailwindcss@3 -c web/css/tailwind.config.js -i web/css/src.css -o web/css/app.css --minify
  ```

**2）生成更新包**：以手机当前版本为基线。

```bash
python3 tools/make_update.py --base <手机 /opt/agv/.agv_version 里的提交号> -o agv-update-<版本>.tgz
```

**3）在运维管理页面发布**：浏览器打开 `http://<手机IP>:8082` →「运维管理」→ 登录（密码向用户要）→「平台更新」→ 上传 → 看预览 → 应用。
- 预览里有改动的文件和受影响的服务。
- 应用时会：备份原文件；改到 `ros2/` 时自动编译，编译失败自动恢复；按需重新部署实例、重启节点代理或平台自身。
- 最近一次更新可以回滚。
- 设备本地的数据和配置不会被覆盖：`data/`、`records/`、`robot_config.json`、`model_overrides.json`、`sensor_overrides.json`、`robot.urdf`。
- 你无法替用户点浏览器时：把更新包交给用户上传，或者经用户同意，用管理员登录接口 `POST /api/hub/admin/login`、`/updates/upload`、`/updates/<id>/apply` 完成。

**4）APK**：
- 在手机 Termux 里用 `bash deploy/android/cover_app/build.sh` 编译。
- 在运维管理「App 管理」上传，再选「手机确认安装」（用户点一次安装）或「静默安装」（要先完成无线调试配对）。这两种方式都还没在手机上实测过。

**5）提交**：
- 提交说明写清楚做了什么、为什么。
- 推到 `main` 后，手机的 `.agv_version` 应与推送的提交一致。
- 文档放 `docs/`，待办更新 `docs/BACKLOG.md`。

### 7. 当前状态（交接时）

- **手机上运行的服务**：
  - 平台：`7193b30`。
  - 节点代理：仍是**修复前的旧代码**。它的远程重启还用 exec，在 proot 里会崩溃，所以运维页上的「重启本机节点代理」**先不要点**。
- **正在运行的实例**：用户的仿真 i09。
- **节点代理的处理**：等 i09 不用时，先在平台停掉 i09，再在手机上把节点代理按原环境变量重启（或 `stop_agv.sh` 后 `start_agv.sh`），然后重新部署 i09，让节点代理用上新代码。
- **最近完成的功能**：
  - 规划支持倒车段，空间不足时分步挪车。
  - 首页实例卡片简化，可下载仿真引擎日志。
  - 添加计算节点 = 只验证并保存 SSH 账号，首次部署时才通过 SSH 装运行环境（目标设备要有 Docker）。
  - 节点详情只读。
  - 运维管理（平台更新、App 管理、服务与日志、设置）。
  - 外屏面板：下发任务列表第一个任务、退出、让出屏幕不自动回来、0.5 秒刷新、适配外屏可用区。

### 8. 待办与已知问题（按建议优先级）

1. **重启节点代理**：按第 7 节的步骤，让手机上的节点代理用上新代码。之后在运维页实测一次「重启本机节点代理」。
2. **App 安装实测**：在手机上实测「手机确认安装」和无线调试「静默安装」。手机重启后是否需要重新配对，还未知。
3. **SSH 节点首次部署实测**：用户有一台 192.168.11.114 待接入。实测"添加节点（验证并保存）→ 首次部署自动安装运行环境"的全流程，确认目标机有 Docker、镜像能分发。
4. **平台地址错误**：`start_agv.sh` 打印的平台地址、本机节点的 `lan_host` 都是 `172.19.0.1`，应为 Wi‑Fi IP。
5. **引擎日志被访问记录淹没**：下载到的仿真引擎日志，大部分是网关每次读状态的访问记录，而且只保留最近约 256 KB。
6. **倒车相关**：倒车靠站时车尾防护区没有缩短；`NAV2_ROUTE_MODE=follow_path` 备用模式不支持倒车。
7. **性能**：`web_gateway` 空闲时 CPU 偏高的原因未查清；`docs/FLIP5_PERF_REPORT.md` §6.2 里的模块精简建议。
8. **其余**：`docs/BACKLOG.md` 中的 T 系列待办和 B 系列已知问题。

### 9. 第一步

1. 克隆仓库，读第 4 节列的文档。
2. 向用户要：手机当前 IP、把你的 SSH 公钥加到手机 `~/.ssh/authorized_keys`（用户在 Termux 里操作）、运维管理密码。
3. SSH 登录手机，执行 `bash ~/status_agv.sh`。核对：
   - `.agv_version`
   - 平台、代理、派生服务、安卓助手是否在运行
   - 有哪些实例在运行
4. 把核对结果简要报告给用户，并提出你打算先做的事（默认从第 8 节第 1 项开始），等用户确认方向后再动手。
