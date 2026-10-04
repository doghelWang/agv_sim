# 代码框架说明 (CODEBASE)

本文档说明仓库中每个文件的职责、依赖的库及其用途，以及模块之间的调用链。与 `docs/ARCHITECTURE.md` (架构理念)、`docs/API.md` (REST 接口)、`docs/PLATFORM.md` (平台功能)、`docs/DEPLOY.md` (部署) 互补。

---

## 1. 总体结构

系统由 **5 类进程**组成，进程之间只通过 **HTTP REST** 通信 (`common/rest.py`)；ROS 2 只存在于执行进程内部。

```
浏览器 (web/ 新前端 · index.html 经典调度台 · model_editor.html 模型补全)
   │ HTTP
   ▼
agv-hub  hub/server.py  :8080 (被占用自动顺延)          ← 镜像 agv-platform (hub)
   │  资源仓库 (节点/模型/场景/程序包/记录) · 部署编排 · /inst/<id>/ 反向代理
   │ REST + X-Node-Key
   ▼
agv-agent  agent/server.py  :8070 (每台计算节点一份)      ← 镜像 agv-platform (agent)
   │ /var/run/docker.sock (Docker Engine API)
   ▼
┌──────────────── 一个仿真实例 (可同机，也可分到两台节点) ─────────────────┐
│ agv-sim 容器                                        agv-nav 容器        │
│  sim_server/api.py  :81xx  仿真进程 (MuJoCo)  ◄─REST─►  nav_runtime/main.py :81xx │
│  web_gateway.py     :81xx  Web 网关 (+gateway_v2)  ─REST─►  执行进程 (导引 / ROS 2 + Nav2) │
└──────────────────────────────────────────────────────────────────────────┘
```

| 进程 | 入口 | 镜像 / Dockerfile | 依赖的第三方库 |
|---|---|---|---|
| 资源平台 agv-hub | `python3 -m hub.server` | agv-platform / `docker/Dockerfile.platform` | numpy (场景栅格)、protobuf (cmodel 解码) |
| 节点代理 agv-agent | `python3 -m agent.server` | agv-platform | psutil (主机指标) |
| 仿真进程 | `python3 -m sim_server.api` | agv-sim / `docker/Dockerfile.sim` | mujoco、numpy、Pillow、protobuf |
| Web 网关 | `python3 web_gateway.py` | agv-sim | psutil |
| 执行进程 | `python3 -m nav_runtime.main` | agv-nav / `docker/Dockerfile.nav` | numpy、rclpy + ROS 2 消息包 + Nav2 (可选，NAV_USE_ROS=0 时不需要) |

---

## 2. 目录结构

```
.
├── common/            进程间通信层 (REST 服务端/客户端) + 事件总线
├── hub/               资源管理平台 agv-hub
├── agent/             计算节点代理 agv-agent
├── sim_server/        仿真进程 (REST 服务 + 仿真服务 + 实例启动准备)
├── sim_core/          仿真内核 (运动学、MuJoCo 物理、激光/相机/光电/触边、世界几何)
├── sensors/           工业 IO 仿真 (急停/触边/光电 DI、顶升、灯塔)
├── nav_runtime/       执行进程 (导引、任务流、仿真数据接入、ROS 2 桥)
├── planning/          规划 (拓扑 Dijkstra、A*、车体机动可行性、保护空间)
├── tools/             生成器与运维脚本 (Nav2 参数、场景栅格地图、精度/性能测试、镜像代码刷新)
├── cmodel_proto/      .cmodel 解码 (protobuf 描述符 + 解码器)
├── nav2/              Nav2 启动文件 (+ 运行时生成的各车型参数)
├── docker/            三个镜像的 Dockerfile 与入口脚本
├── deploy/linux/      Linux 板卡 (RK3588/树莓派/x86) 一键部署 setup.sh
├── deploy/android/    Android 手机 (Termux + proot Ubuntu) 安装、启停、更新脚本，proot 派生服务，DDS 配置
├── web/               资源平台 + 设备端工作台前端 (ES Modules + Tailwind)
├── vendor/            前端第三方库 (three.js、OrbitControls、lucide、tailwind) + AMR 3D 模型
├── maps/              场景栅格地图 (tools/scenario_to_map.py 生成)
├── tests/             测试 (离线单测、三进程 REST 端到端、平台端到端) + 示例 cmodel
├── docs/              文档
├── cmodel_parser.py   .cmodel → robot spec / URDF
├── model_overrides.py 模型人工补全 (覆盖、审计、传感器模板)
├── web_gateway.py     Web 网关 (经典调度台后端)
├── gateway_v2.py      网关 v2 接口 (工作台: 控制权锁、事件、注入、记录)
├── nav2_bridge.py     Nav2 action 客户端 + Nav2 进程监管 (执行进程使用)
├── index.html         经典调度台页面 (web_gateway 提供)
├── model_editor.html  模型补全与传感器安装页面
├── deploy.sh          构建/部署脚本 (build / up / hub / agent / down / save)
└── start_sim.sh       非 Docker 本机三进程启动 (装有 ROS 2 时)，无 ROS 时转 deploy.sh up
```

---

## 3. 逐文件说明

表中「依赖」只列第三方库和关键标准库；所有 Python 文件都只用标准库完成 HTTP/JSON。

### 3.1 common — 通信层

| 文件 | 功能 | 依赖 | 被调用 |
|---|---|---|---|
| `common/rest.py` | `RestServer`：基于 `http.server` 的线程化 REST 服务，路由 `{param}`、JSON/二进制/文件响应、挂载静态目录；`RestClient`：带超时与重试的 JSON 客户端；`ApiError` 统一错误；`wait_for()` 等待服务就绪 | http.server、urllib、socket、threading | 所有服务进程 |
| `common/spawn.py` | 子进程派生：本机 `Popen`，或在 Android 上经 proot 派生服务把进程放进独立 proot 会话 (`popen`/`killpg`、`python3 common/spawn.py exec 名称 -- 命令`)；未配置 `AGV_SPAWNER` 时等同 `subprocess.Popen` | 标准库 | agent/runtime.py、nav2_bridge.py、nav_runtime/ros_slam.py、ros_bridge.py、nav2/*_launch.py、docker/sim_entrypoint.sh |
| `common/hostinfo.py` | 主机信息 (树莓派 / x86 / Android)：机型、大小核分组、温度、proot 下的 CPU 估算 | 标准库 | agent/server.py、web_gateway.py、tools/bench_host.py |
| `common/events.py` | `EventHub`：环形事件缓冲 (分类/级别/标题/详情)，按 since_id 增量拉取 | threading | web_gateway、navigator |

### 3.2 hub — 资源管理平台 (agv-hub)

| 文件 | 功能 | 依赖 | 被调用 |
|---|---|---|---|
| `hub/server.py` | 平台入口：组装各仓库，注册 `/api/hub/*` 接口，提供前端静态文件与 `/model-editor`，`/inst/<id>/*` 反向代理到实例网关 | http.client、mimetypes | 容器入口 |
| `hub/store.py` | SQLite 文档存储 (每类资源一张表 id→JSON) + 文件目录 `~/.agv-hub` | sqlite3 | 各仓库 |
| `hub/nodes.py` | 节点注册表：一次性令牌/集群令牌注册、节点密钥、心跳状态；`AgentClient` 调用节点代理 | urllib | server、deployer |
| `hub/models_repo.py` | 车辆模型仓库：上传 .cmodel 即解析 (cmodel_parser) 为基线，保存人工补全，版本管理，生成实例用模型包 (bundle)，离线补全编辑器接口 | cmodel_parser、model_overrides | server |
| `hub/scenes_repo.py` | 场景仓库：场景包 (scene.json / PGM+YAML / topology.json / mesh) 解析，只有栅格时由栅格生成墙体，任务流存取 | numpy (PGM 处理)、zipfile；planning.dijkstra_planner、tools.scenario_to_map | server |
| `hub/packages.py` | 软件程序包 (Docker 镜像)：上传 `docker save` 包流式落盘并解析 RepoTags/架构/标签；节点镜像自动登记；可部署性判断 | tarfile | server、deployer |
| `hub/deployer.py` | 实例编排：校验 → 分配端口 → 确保镜像 → 启动 agv-sim → 等就绪 → 启动 agv-nav → 等就绪 → Nav2 检查；监控、重启、终止、日志；失败回滚 | urllib | server |

### 3.3 agent — 计算节点代理 (agv-agent)

| 文件 | 功能 | 依赖 | 被调用 |
|---|---|---|---|
| `agent/server.py` | 节点代理入口：向平台注册、每 5 s 心跳 (CPU/内存/温度/磁盘、镜像、容器、端口)；接口：端口分配、镜像确保/导出、容器运行/停止/日志、探测 | psutil | 容器入口 |
| `agent/runtime.py` | `DockerRuntime` (正式：经 docker.sock 起停容器，host 网络、数据目录挂载)；`ProcessRuntime` (开发/测试：直接起源码进程) | subprocess | server |
| `agent/docker_api.py` | Docker Engine API 客户端 (Unix socket 上的 HTTP，无需 docker CLI)：镜像列表/拉取/加载/导出、容器创建/启动/停止/日志 | socket、http.client | runtime |

### 3.4 sim_server — 仿真进程

| 文件 | 功能 | 依赖 | 被调用 |
|---|---|---|---|
| `sim_server/api.py` | 仿真进程 REST v1 (`:8090` 或实例端口)：模型/补全/URDF、场景/障碍物/栅格地图、仿真控制、状态、激光 (二进制长轮询)、相机帧、IO、指令、导航回馈、事件、快照 | numpy；common.rest、sim_core.imgcodec | 入口 |
| `sim_server/service.py` | `SimService`：模型构建 (cmodel/spec → 运动学、传感器、URDF)、世界构建 (场景、障碍物、移动障碍)、仿真主循环线程、激光缓冲、看门狗、事件；生成 Nav2 参数与地图 | numpy；sim_core.engine、cmodel_parser、model_overrides、tools.* | api |
| `sim_server/bootstrap.py` | 实例启动准备：初始化数据目录；从平台拉取模型包与场景 | urllib | 容器入口、service |

### 3.5 sim_core — 仿真内核

| 文件 | 功能 | 依赖 | 被调用 |
|---|---|---|---|
| `sim_core/engine.py` | `SimCore`：固定步长 10 ms + 实时累加器；组合运动学、MuJoCo 后端 (不可用时退回纯运动学)、激光/3D 激光/相机/光电/触边/IMU/里程计/读码/IO；碰撞处理与急停 | numpy | service |
| `sim_core/kinematics.py` | 通用轮式底盘模型 (差速/单舵轮/双舵轮/多舵轮)：逆解、舵角限位与最近解、先转后走、执行器惯性、最小二乘正解、SE(2) 积分 | numpy | engine、sensors、gen_nav2_params |
| `sim_core/mujoco_backend.py` | MuJoCo 后端：由 spec 生成 MJCF，车体平面刚体 + 接触；移动障碍 (mocap)；批量射线 (mj_multiRay，多线程、每线程独立 MjData)；可选 OpenGL 渲染 | **mujoco**、numpy、concurrent.futures | engine |
| `sim_core/world.py` | 世界几何：墙/货架/障碍 (带高度竖直线段)、车体轮廓碰撞检测 | numpy | engine、backend、sensors |
| `sim_core/sensors.py` | 2D/3D 激光 (安装位姿、倒装、视场、量程、噪声)、融合扫描、编码器里程计、打滑、IMU、读码相机 | numpy | engine |
| `sim_core/cameras.py` | 单目 RGB / 双目 (左右+深度) / ToF (深度+幅度+点云)：逐像素射线 + Lambert 着色；`CAMERA_LIBRARY` 型号参数 | numpy | engine、model_overrides |
| `sim_core/discrete.py` | 光电 (漫反射/对射) 与防撞触边 (碰撞条)；运动阻断判断 | numpy | engine |
| `sim_core/imgcodec.py` | 图像编码：JPEG (Pillow)、PNG (纯 zlib)、深度伪彩 | **Pillow**、numpy | sim_server.api |

### 3.6 sensors — 工业 IO

| 文件 | 功能 | 依赖 | 被调用 |
|---|---|---|---|
| `sensors/io_simulator.py` | 急停、触边、光电 DI，顶升限位与电机，抱闸、灯塔 DO | — | sim_core.engine |

### 3.7 nav_runtime — 执行进程

| 文件 | 功能 | 依赖 | 被调用 |
|---|---|---|---|
| `nav_runtime/main.py` | 执行进程入口 (`:8091` 或实例端口)：连接仿真进程、启动 Navigator/任务流、可选 ROS 2 + Nav2；REST：任务、遥控、规划器、任务流、路线预览、保护空间、事件、ROS 图 | rclpy (可选)；tools.gen_nav2_params | 入口 |
| `nav_runtime/navigator.py` | 任务执行器：规划 (Dijkstra 拓扑 / A* / 直连 / Nav2)、导引 (精确直线跟踪、拐点停车原地转向或圆弧过渡、先转后走、倒车离站、末端对正)、保护空间 (分档防护区、转向防护、进站放宽)、光电/触边/急停安全层、任务记录与回馈 | numpy；planning.*、common.events | main |
| `nav_runtime/sim_link.py` | 仿真数据接入 (纯 REST)：状态 50 Hz、IO 20 Hz、激光/融合扫描长轮询、模型/场景变化回调、下发 cmd_vel、回馈导航状态 5 Hz | numpy | main |
| `nav_runtime/ros_slam.py` | 开源定位栈监管 (ROS 2)：启动/重启 `nav2/loc_launch.py` (robot_localization EKF + slam_toolbox 建图/定位)、EKF odom 系对齐初始位姿 (/set_pose)、TF map→base_footprint → 导引位姿、/map → 界面、保存位姿图与 PGM/YAML | rclpy、tf2_ros、nav_msgs、slam_toolbox.srv | ros_bridge |
| `nav_runtime/slam.py` | 定位门面 SlamLocalizer (Navigator 只和它打交道)：外部引擎 (ros_slam) 存在时转发/换算其结果；否则内置 激光 SLAM (占据栅格 log-odds 2.5 cm + 亚栅格命中密度 → 两级匹配场；Gauss-Newton 扫描匹配) 与轮式里程计融合 (以里程计预测为先验的迭代 EKF，输出 map→odom)；模式 slam / localization / odom / ground_truth；地图保存/加载 (npz + Nav2 PGM/YAML) | numpy | navigator、main、ros_bridge |
| `nav_runtime/taskflow.py` | 任务流执行器：工步 move / lift / drop / wait，限速、进度与事件 | — | main |
| `nav_runtime/ros_bridge.py` | ROS 2 桥 (只在执行进程内)：把 REST 数据发布为 /scan /odom /tf /map 等，robot_state_publisher 子进程监管，Nav2 输出回灌 | rclpy、sensor_msgs、nav_msgs、geometry_msgs、tf2_ros | main |
| `nav2_bridge.py` (根目录) | FollowPath (线路跟随，默认) / NavigateToPose / NavigateThroughPoses action 客户端、地图热切换 (LoadMap)、Nav2 进程监管 (按车型参数重启) | rclpy、nav2_msgs、action_msgs | main、navigator |
| `nav2/nav2_launch.py` | Nav2 启动文件：map_server (map_source=topic 时不启动，用 slam_toolbox 的 /map)、planner、controller (RotationShim+RPP / DWB)、velocity_smoother、behaviors、bt_navigator | launch、launch_ros | nav2_bridge (子进程) |
| `nav2/loc_launch.py`、`nav2/ekf.yaml`、`nav2/slam_toolbox.yaml` | 定位栈启动文件与参数：robot_localization ekf_node、slam_toolbox (async 建图 / localization) | launch、launch_ros | ros_slam (子进程) |

### 3.8 planning — 规划

| 文件 | 功能 | 依赖 | 被调用 |
|---|---|---|---|
| `planning/dijkstra_planner.py` | 内置场景定义 `SCENARIO_DEFINITIONS` 与 `register_scenario()`；拓扑路网 Dijkstra：起终点投影到拓扑边、(节点,来向) 状态、转向与拐点代价、障碍阻断、拐点可行性惩罚 | heapq | navigator、sim_server、hub.scenes_repo、tools、web_gateway |
| `planning/maneuver.py` | 车体机动可行性：车体矩形在一组位姿上的净空、拐点过弯方式选择 (原地转向两个方向 / 圆弧) | numpy | dijkstra_planner、navigator |
| `planning/protection.py` | 保护空间：默认值 (按车型动力学生成防护区)、带载外形、按速度取档、机体系多边形、制动距离/带载/激光覆盖校核 | — | navigator、model_overrides、gen_nav2_params |
| `planning/a_star_planner.py` | A* 栅格规划 (障碍膨胀、路径平滑) | heapq | navigator |

### 3.9 模型与工具

| 文件 | 功能 | 依赖 | 被调用 |
|---|---|---|---|
| `cmodel_parser.py` | .cmodel → robot spec (底盘类型、轮组、电机、激光、相机、IO、电池、空载/满载运动参数) + URDF；CLI 可同时生成 Nav2 参数 | cmodel_proto、model_overrides | hub、sim_server、sim_core.engine、tests |
| `cmodel_proto/decoder.py` | .cmodel (ZIP + protobuf) 独立解码，内置描述符 `*.desc` | **protobuf** | cmodel_parser |
| `model_overrides.py` | 人工补全：载入/保存/迁移 (兼容旧 sensor_overrides.json)、`apply_overrides()` (车体、轮组、电机、顶升、IMU、传感器增删改、保护空间)、`audit()` 完整度/保护空间校核、`sensor_template()` | sim_core.cameras、planning.protection | hub、sim_server、cmodel_parser |
| `tools/gen_nav2_params.py` | robot spec → Nav2 参数 (按车型：限速/加速度/控制器/代价地图外形取保护空间) | sim_core.kinematics、planning.protection | nav_runtime.main、sim_server、cmodel_parser |
| `tools/scenario_to_map.py` | 场景几何 → PGM/YAML 栅格地图 (与仿真几何一致) | planning.dijkstra_planner | sim_server、hub.scenes_repo |
| `tools/precision_test.py` | 导航/定位精度考核：经网关逐目标下发，统计横向偏差、终点误差、定位误差 | — | 人工 |
| `tools/check_loc_stack.sh` | 检查执行容器内定位/导航栈 (软件包、节点、话题频率、TF、已保存地图) | docker、ros2 CLI | 人工 |
| `tools/refresh_images.sh` | 在已构建的 agv-sim / agv-nav 镜像上只刷新代码层 (不重装依赖，几十秒)，代码更新后用 |
| `tools/perf_sample.py` | 运行性能采样：进程 CPU/内存、CPU 频率、温度、仿真 RTF、执行进程状态频率、TF 延迟 (只用标准库) |
| `tools/perf_matrix.py` | 手机/板卡组合性能对比：经主平台依次部署 4 种组合，采样 + 精度测试，结果见 docs/PERFORMANCE.md |
| `tools/bench_host.py` | 单机计算基准 (Python/numpy/MuJoCo 步/激光/相机/内置 SLAM) |
| `tools/phone_probe.sh` | 手机能力与传感器探测 (Termux) |
| `tools/deploy_verify.sh` | 单机一键构建 + 启动 + 定位/导航栈检查 + 精度测试 |

### 3.10 Web 网关

| 文件 | 功能 | 依赖 | 被调用 |
|---|---|---|---|
| `web_gateway.py` | Web 网关 (`:8088` 或实例端口)：提供 index.html / model_editor.html / vendor；轮询仿真快照 (20 Hz)、执行状态 (5 Hz)、模型与场景；兼容旧前端接口 (`/api/telemetry`、`/api/navigate_to_pose` 等)；飞行记录器；主机性能监控 | psutil；common.rest、common.events、gateway_v2 | 容器入口 |
| `gateway_v2.py` | 工作台 v2 接口 `/api/v2/*`：控制权锁 (租约 30 s)、实例信息、带标签事件、暂停/重置、任务流、扰动注入 (目录+落点)、仿真记录 (sim_bundle 生成/打包/归档平台)、日志、保护空间/规划器/底盘/场景/模型热切换、路线预览 | — | web_gateway |

### 3.11 前端

| 文件 | 功能 | 依赖 (import) |
|---|---|---|
| `web/index.html` | 新前端外壳 (平台 + 工作台)，加载 css/app.css 与 vendor 脚本 | three.js、OrbitControls、lucide、amr_model3d |
| `web/js/app.js` | hash 路由 `#/platform/<页>`、`#/wb/<实例>/<tab>`、`#/replay` | core、platform、workbench |
| `web/js/core.js` | 公共工具：`$`、http/hub 客户端、上传下载、toast、modal、格式化、状态徽标 | — |
| `web/js/platform.js` | 资源平台页面：总览、实例列表与部署向导、计算资源、车辆模型、场景、程序包、记录归档 | core、viz2d、viz3d |
| `web/js/workbench.js` | 工作台框架：实例接入、控制权、遥测轮询、tab 切换 | core、wbcore、wb_* |
| `web/js/wbcore.js` | 工作台共享状态 `W`、实例接口 (带锁令牌) | core |
| `web/js/wb_taskflow.js` | 3-1 任务流配置 + 2D 地图动线预览 (后端 /api/v2/plan) | core、wbcore、viz2d |
| `web/js/wb_run.js` | 3-2 仿真运行：3D 视图、HUD (下一路段/保护空间)、事件流、注入库、手动控制 | core、wbcore、viz3d、viz2d |
| `web/js/wb_tabs.js` | 3-3 记录、3-4 回放、3-5 传感器与安全、3-6 系统与导航 (保护空间编辑) | core、wbcore、viz3d |
| `web/js/viz2d.js` | 2D：PGM/YAML 解析、场景缩略图、模型俯视 SVG、`MapCanvas` (地图、拓扑、路线分段、防护区)、前端拓扑路线 | core |
| `web/js/viz3d.js` | 3D：`Viewer3D` (实时车体、路径分段、下一航点、防护区)、`ReplayView`、`ModelView`、`SceneView` | viz2d、three.js |
| `web/css/src.css` → `app.css` | Tailwind 源与离线编译产物 | tailwindcss CLI (构建时) |
| `vendor/amr_model3d.js` | AMR 3D 模型 (按 spec 生成车体/轮组/传感器) | three.js |
| `index.html` (根目录) | 经典调度台 (单页) | vendor/* |
| `model_editor.html` | 模型补全：完整度检查、车体轮组、顶升、传感器安装、保护空间、覆盖文件；可接实例 (`/sim/api/v1`) 或平台仓库 (`?api=`) | — |

### 3.12 部署与测试

| 文件 | 功能 |
|---|---|
| `deploy.sh` | `build [sim|nav|platform]`、`up` (单机两容器)、`hub` (平台 + 本机代理)、`agent` (加入平台)、`down`、`save` (导出镜像) |
| `docker/Dockerfile.sim` / `sim_entrypoint.sh` | 仿真镜像：mujoco + numpy + Pillow + protobuf；入口先 bootstrap 再起仿真进程与网关 |
| `docker/Dockerfile.nav` / `nav_entrypoint.sh` | 执行镜像：ROS 2 Humble + Nav2；入口起执行进程 |
| `docker/Dockerfile.platform` / `platform_entrypoint.sh` | 平台镜像：hub / agent 二选一 |
| `docker-compose.yml` | 单机两容器的 compose 等价写法 |
| `start_sim.sh` | 本机已装 ROS 2 时直接起三进程；未装时转 `deploy.sh up` |
| `deploy/linux/setup.sh` | Linux 板卡/电脑一键部署：检查硬件 → 装 Docker → (可选 swap) → 构建镜像 → 启动主平台或接入已有主平台 |
| `deploy/android/install.sh` | Android 一键安装 (Termux)：proot Ubuntu 22.04 → proot_setup.sh (ROS 2 Humble/Nav2/MuJoCo) → 拉代码 → 脚本与开机自启 → 配置 ~/.agv.env |
| `deploy/android/proot_setup.sh` | proot 容器内安装运行环境 (清华/官方源可选，处理 proot 下 pip 的 Android 误判) |
| `deploy/android/agv_common.sh`、`start_agv.sh`、`stop_agv.sh`、`status_agv.sh`、`update_from_git.sh`、`termux-boot-01-start-agv.sh` | 手机上的启停、状态、从 git 更新、开机自启 (Termux 侧) |
| `deploy/android/proot_spawner.py` | proot 派生服务 (:8069，Termux 原生 Python)：每个重负载进程独立 proot 会话，解除单 ptrace 追踪线程瓶颈 |
| `deploy/android/fastdds_ds.xml` | Android 默认 DDS 配置：只走 127.0.0.1，发现走实例的 Fast DDS 发现服务器 |
| `deploy/android/fastdds_localhost.xml`、`cyclonedds_localhost.xml` | Android 回环不支持组播：只走 127.0.0.1、单播发现 120 个参与者 |
| `deploy/android/bench_nav.py`、`tools/dds_probe.py` | 手机上的导航通信基准：任务 + DDS 往返延迟 + 各进程 CPU |
| `tests/test_sim_core.py` | 离线单测 (解析、URDF、运动学、碰撞、激光、Nav2 参数、MuJoCo、相机、补全) |
| `tests/test_rest_e2e.py` | 三进程 REST 端到端 (需先启动三进程) |
| `tests/test_platform_e2e.py` | 平台端到端：hub + 两个进程运行时节点，分离部署、任务流、注入避障、记录归档 |

### 3.13 数据文件

| 文件 | 说明 |
|---|---|
| `robot_config.base.json` | cmodel 解析基线 (无人工补全) |
| `robot_config.json` / `robot.urdf` | 基线 + 补全后的生效模型 (运行时由仿真进程重写) |
| `model_overrides.json` | 人工补全 (含 `protection` 保护空间) |
| `sensor_overrides.json` | 旧版传感器覆盖格式，首次启动自动迁移到 model_overrides.json (保留用于迁移与测试) |
| `maps/*.pgm|yaml` | 内置场景栅格地图 (由场景几何生成，Nav2 地图服务用) |
| `data/slam_maps/<场景>.npz|pgm|yaml` | 执行进程保存的 SLAM 地图 (容器内 /data/slam_maps，SLAM_MAP_DIR 可改) |
| `nav2/nav2_params_<车型>.yaml` | 运行时按车型生成 |
| `cmodel_proto/*.desc` | protobuf 描述符 |
| `tests/data/*.cmodel` | 示例车模 (平台首次启动自动入库) |

---

## 4. 第三方库清单

| 库 | 使用位置 | 用途 |
|---|---|---|
| mujoco | sim_core/mujoco_backend.py | 物理步进 (车体刚体+接触)、移动障碍 mocap、批量射线 (激光/光电/相机) |
| numpy | sim_core/*、sim_server、nav_runtime、planning/maneuver、hub/scenes_repo | 向量化几何、射线结果处理、栅格/扫描数组 |
| Pillow | sim_core/imgcodec.py | 相机帧 JPEG 编码 |
| protobuf | cmodel_proto/decoder.py | 解码 .cmodel 中的组件描述 |
| psutil | web_gateway.py、agent/server.py | 进程与主机资源监控 |
| rclpy + ROS 2 消息 (sensor_msgs、nav_msgs、geometry_msgs、tf2_ros、nav2_msgs、action_msgs) | nav_runtime/ros_bridge.py、nav2_bridge.py、nav_runtime/main.py | 执行进程内与 Nav2 对接 (可选) |
| launch / launch_ros | nav2/nav2_launch.py、nav2/loc_launch.py | Nav2 / 定位栈启动 |
| slam_toolbox (ROS 2，apt `ros-humble-slam-toolbox`) | nav2/loc_launch.py、nav_runtime/ros_slam.py | 激光 SLAM 建图与定位 (Karto 扫描匹配 + Ceres 位姿图回环) |
| robot_localization (ROS 2，apt `ros-humble-robot-localization`) | nav2/loc_launch.py | EKF 融合轮式里程计与 IMU |
| Nav2 (controller_server / RotationShim / Regulated Pure Pursuit / costmap_2d / bt_navigator …) | nav2/nav2_launch.py、nav2_bridge.py | 路径跟随、局部避障、速度平滑 |
| PyYAML | tests/test_sim_core.py | 校验生成的 Nav2 参数 (仅测试) |
| three.js、OrbitControls、lucide、Tailwind | vendor/、web/ | 3D 视图、相机交互、图标、样式 |

---

## 5. 调用链

### 5.1 部署链

```
deploy.sh hub
  └─ docker run agv-platform hub  → hub/server.py (Store、NodeRegistry、ModelRepo、SceneRepo、PackageRepo、Deployer)
  └─ docker run agv-platform agent → agent/server.py ──POST /api/hub/nodes/register──▶ hub (集群令牌)
浏览器 部署向导 ──POST /api/hub/instances──▶ hub/deployer.Deployer.deploy()
  ├─ AgentClient ──/api/v1/ports/allocate、/images/ensure──▶ agent → runtime.DockerRuntime → docker_api (docker.sock)
  ├─ /containers/run agv-sim  → docker/sim_entrypoint.sh
  │     ├─ sim_server/bootstrap.py  ──GET /api/hub/models/{id}/versions/{v}/bundle、/scenes/{id}/definition──▶ hub
  │     ├─ python -m sim_server.api   (仿真进程)
  │     └─ python web_gateway.py      (网关，内含 gateway_v2)
  ├─ 等待 /api/v1/health
  ├─ /containers/run agv-nav  → docker/nav_entrypoint.sh → python -m nav_runtime.main
  └─ 就绪 → 浏览器 #/wb/<iid> ──/inst/<iid>/...──▶ hub 反向代理 ──▶ 该实例的 web_gateway
```

### 5.2 模型链 (cmodel → 仿真 → 执行)

```
上传 .cmodel ──▶ hub/models_repo.upload
                  └─ cmodel_parser.parse_cmodel_file → cmodel_proto.decoder (protobuf) → extract_robot_spec → base.json
模型补全 (model_editor.html ?api=/api/hub/models/..)
                  └─ PUT overrides → model_overrides.apply_overrides + audit (含 planning.protection.validate)
实例启动 bootstrap 拉取 bundle (base + overrides)
sim_server/service.SimService.load_model
  ├─ model_overrides.apply_overrides → spec (含 protection、protection_effective)
  ├─ sim_core.engine.SimCore(spec) → kinematics.build_kinematics、mujoco_backend.MuJoCoBackend、sensors、cameras、discrete
  ├─ cmodel_parser.generate_urdf → robot.urdf
  └─ tools.gen_nav2_params.write_all (Nav2 外形取 planning.protection.nav2_outline)
执行进程 sim_link ──GET /api/v1/model──▶ navigator._on_model
  └─ protection.effective(spec) → outline() → DijkstraPlanner.set_footprint(...)、A* 膨胀
```

### 5.3 仿真步进与传感器数据流

```
SimService 主循环线程 (10 ms 固定步长)
  └─ SimCore.step
       ├─ kinematics.step(cmd_vel)          先转后走、执行器惯性
       ├─ MuJoCoBackend.step                接触 → 碰撞点 → 触边 DI / 急停
       ├─ LidarSensor / Lidar3DSensor.scan  → MuJoCoBackend.cast (mj_multiRay)
       ├─ cameras / PhotoSensor / BumperStrip / Imu / WheelOdometry / CodeReader
       └─ _IO (sensors/io_simulator)        顶升/抱闸/灯塔
sim_server/api 暴露: /api/v1/state、/sensors/lidars/{name} (长轮询二进制)、/sensors/scan、/io、/snapshot、/sensors/cameras/{name}
```

### 5.4 定位链 (SLAM + 里程计)

开源引擎 (有 ROS 2 且装了 slam_toolbox / robot_localization，默认)：
```
state.odom / state.imu ──ros_bridge──▶ /odom /imu ──▶ ekf_node (robot_localization) ──▶ TF odom→base_footprint
sensors/scan (合并 360°) ──ros_bridge──▶ /scan ──▶ slam_toolbox (建图 async / 定位) ──▶ TF map→odom、/map
ros_slam.RosLocalization.poll (50 Hz) : TF map→base_footprint (带时间戳)
     └─ SlamLocalizer.set_external: 按时间戳对齐轮式里程计历史 → 修正 M → 导引位姿 = M ∘ 当前里程计 (无延迟)
保存: /slam_toolbox/serialize_map → slam_maps/<场景>.posegraph/.data；/map → <场景>.pgm/.yaml
场景切换/复位/车被搬动: 按当前 (已知) 位姿重启定位栈 (有保存的地图 → localization，否则 mapping)
```
内置引擎 (无 ROS)：
```
仿真进程: WheelOdometry (轮径误差/舵角偏置/编码器量化，真值带打滑 → 漂移)  → /api/v1/state.odom (带仿真时间 t)
          各 2D 激光原始帧 (安装位姿 + 角度，带噪声/丢点)              → /api/v1/sensors/lidars/{name}
执行进程:
  SimLink._state_loop (50 Hz) → Navigator._on_state → SlamLocalizer.on_odom(t, odom)
        融合位姿 = M ∘ odom  → telemetry x/y/yaw (导引、保护空间、到位判断都用它)
  SimLink._lidar_loop (每个激光) → Navigator._on_lidar → 同一时刻各激光合为一帧 → SlamLocalizer.on_points
        ├─ 等状态覆盖到激光时间戳 (最多 80 ms)，按时间插值里程计 → 预测位姿 + 预测协方差
        ├─ match(): 粗 (10 cm, σ15 cm) → 细 (2.5 cm, σ5 cm) Gauss-Newton，里程计先验 → 位姿 + 协方差
        ├─ 内点率 > 35% 采纳 → 更新 M (map→odom) 与协方差
        └─ slam 模式: 移动 > 8 cm / 3° 时插入地图 → 后台线程重建匹配场
  只有 3D 激光时改用合并扫描 (/sensors/scan，精度较低)
  真值 (state.truth) 只用于: 启动/复位/被搬动时的初始位姿 (相当于实车在已知工位设初始位姿)、误差统计
  ros_bridge: TF odom→base_footprint = 轮式里程计，TF map→odom = SlamLocalizer.M (SIM_LOCALIZATION=amcl 时由 AMCL 发布)
```

### 5.5 导航任务链 (浏览器 → 执行 → 仿真)

```
工作台 3-2 下发 / 任务流 move 工步
  └─ web_gateway (/api/navigate_to_pose 或 gateway_v2 /api/v2/taskflow/run)
       ──POST /api/v1/missions 或 /taskflows/run──▶ nav_runtime/main
            └─ Navigator.send_nav_goal
                 ├─ DijkstraPlanner.plan → plan_route (投影接入、转向/拐点代价、_corner_penalty → maneuver.plan_corner)
                 ├─ 拐点不可行检查 → 失败事件
                 ├─ _plan_route_corners → 每个拐点的过弯方式 + plan_curve (显示/考核参考线)
                 └─ 线程 _autonomous_guidance_loop
                      ├─ Phase 1  _rotation_dir → (_back_out) → _align_steer → _rotate_to
                      ├─ Phase 2  直线精确跟踪 (二阶临界阻尼) + 按剩余路程减速 + 防护区 _allowed_speed
                      ├─ 圆弧    _corner_arc → _corner_arc_loop (曲率前馈 + 闭环 + _arc_blocked)
                      ├─ Phase 3  末端对正 _rotate_to
                      └─ publish_cmd_vel → safety_filter (防护区/转向防护/光电/急停)
                           └─ SimLink.send_cmd ──PUT /api/v1/control/cmd_vel──▶ 仿真进程
Nav2 模式 (有 Nav2 时默认): Navigator._send_nav2_goal
          ├─ DijkstraPlanner 拓扑路线 + _plan_route_corners (拐点过弯方式)
          ├─ _nav2_segments: 按原地转向拐点切成直线段 (5 cm 稠密位姿，段终点朝向 = 下一段方向；圆弧拐点并入)
          └─ 线程 _nav2_follow_loop: 逐段 nav2_bridge.follow_path (FollowPath, precise_goal_checker ±20 mm)，中断重试
               → controller_server (RotationShim + Regulated Pure Pursuit) → velocity_smoother → /cmd_vel
               → ros_bridge._on_cmd_vel → safety_filter → SimLink.send_cmd
          (NAV2_ROUTE_MODE=through_poses 时用 NavigateThroughPoses，由 Nav2 规划器沿拓扑点规划)
状态回馈: Navigator.feedback() ──PUT /api/v1/nav/feedback (5 Hz)──▶ 仿真进程 ──/api/v1/snapshot──▶ web_gateway ──▶ 浏览器
```

### 5.6 安全链

```
仿真进程: 碰撞 → 触边 DI；光电射线 → 光电 DI；急停 DI
执行进程: SimLink /api/v1/io (20 Hz) → Navigator._safety_loop (急停挂起/恢复、触边终止任务)
          SimLink 融合扫描 → Navigator._on_merged → 各档防护区走廊距离 (bands) → safety_filter / _allowed_speed
事件:     Navigator.event_hub → web_gateway 拉取 → gateway_v2 打标签 (OBS/SAF/NAV...) → 工作台事件流
```

### 5.7 扰动注入与仿真记录

```
工作台 注入库 ──POST /api/v2/inject──▶ gateway_v2.inject → _path_point (沿当前路线/path_index 取落点)
      ──PUT /api/v1/world/obstacles──▶ 仿真进程 (人员 motion → MuJoCo mocap 往返)
任务流结束 → gateway_v2.TaskRecorder 生成 sim_bundle ──POST /api/hub/records──▶ 平台归档
```

### 5.8 前端数据链

```
app.js 路由 → platform.js (资源页, hub.get/post)  或  workbench.js
workbench.js 轮询 /api/telemetry (含 plan_path、plan_curve、path_index、next_segment、protection)
   → wb_run.js (HUD、Viewer3D) / wb_taskflow.js (MapCanvas) / wb_tabs.js
```

---

## 6. 已移除的旧文件 (2026-09-26)

| 文件 | 原因 |
|---|---|
| `agv_simulation.py`、`web_teleop_server.py`、`controllers/`、根目录 `Dockerfile`、`docker-compose.legacy.yml`、`tools/make_rsp_params.py`、`robot_active.urdf`、`tests/test_integration_mock.py`、`tests/mock_ros.py`、`tests/nav2_smoke.py` | 旧的单 ROS 图 / 单容器架构，已被 sim_server + nav_runtime + web_gateway 取代 |
| `models/` (含 `pybullet_engine.py`)、`sim_core/pybullet_backend.py` | PyBullet 后端，引擎已改用 MuJoCo，不再加载 |
| `sensors/lidar_simulator.py`、`sensors/vision_simulator.py` | 未被引用 (激光/相机由 sim_core 实现) |
| `generate_console.py` | 经典调度台的旧生成器，输出与现有 index.html 不一致，运行会覆盖页面 |
| `nav2_cmodel_params.yaml` | 旧 Nav2 参数，现按车型生成到 `nav2/` |
| `slam/` | 旧 slam_toolbox 启动文件，未被任何镜像或脚本使用 (SLAM 现由 nav_runtime/slam.py 实现) |


