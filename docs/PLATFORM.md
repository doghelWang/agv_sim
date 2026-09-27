# AMR 仿真资源管理平台 + 设备端仿真工作台

平台服务 **agv-hub** 统一管理计算节点、车辆模型、仿真场景和软件程序包；用户在网页上选设备、模型、场景和版本，一键部署后进入工作台。
仿真 (agv-sim) 和执行 (agv-nav) 可以部署在同一台树莓派，也可以分开部署到两台。

```
浏览器 ──HTTP :8080──► agv-hub (资源仓库 · 部署编排 · 实例监控 · 前端 · /inst/<id>/ 反向代理)
                         │ REST + 节点密钥                      ▲ 模型包/场景包/记录归档
                         ▼                                      │
             agv-agent (每台节点 :8070) ──docker.sock──► agv-sim-<id> ◄──REST v1──► agv-nav-<id>
```

| 组件 | 镜像 | 端口 | 说明 |
|---|---|---|---|
| 平台服务 | agv-platform (`hub`) | 8080 | SQLite + 文件仓库，数据在宿主机 `~/.agv-hub` |
| 节点代理 | agv-platform (`agent`) | 8070 | 挂载 docker.sock；实例数据在 `~/.agv-agent/instances/<id>` |
| 仿真程序包 | agv-sim | 实例分配 8100–8199 | 仿真进程 + Web 网关 (含工作台 v2 接口) |
| 运行程序包 | agv-nav | 实例分配 8100–8199 | 执行进程 + ROS 2 / Nav2 |

## 1. 部署

```bash
# 平台所在的树莓派 (同时作为第一个计算节点)
cd ~/agv_sim
bash deploy.sh build          # 构建 agv-sim / agv-nav / agv-platform (已有前两个时只构建 platform: build platform)
bash deploy.sh hub            # 启动平台 + 本机节点代理 (集群令牌自动接入)
# 浏览器打开 http://<树莓派IP>:8080
```

增加计算节点：平台「计算资源 → 添加计算节点」生成一次性令牌和命令，在目标设备的项目目录执行：

```bash
bash deploy.sh agent --hub http://<平台IP>:8080 --token jt-xxxx --kind hybrid     # hybrid=运行+仿真  controller=只运行  sim=只仿真
```

- 平台不保存 SSH 密码；节点以令牌注册后获得节点密钥，平台之后用该密钥调度节点。
- 新节点没有镜像时，部署时由平台分发：先在「软件程序包」里把节点镜像「入库」(或上传 `deploy.sh save` 导出的 tar.gz)。镜像架构必须与节点一致 (树莓派 arm64)。
- 原来的 `deploy.sh up` 单机部署仍可用；在平台「仿真运行列表 → 接入已有实例」填 `http://<IP>:8088` 即可在新工作台操作它。

## 2. 资源管理

| 页面 | 内容 |
|---|---|
| 仿真运行列表 | 实例卡片 (节点、模型、场景、程序包、使用人、当前工步)、部署进度、日志、重启、终止；历史实例 |
| 计算资源 | 按 实体控制器 / 运行+仿真 / 仅仿真 分组；CPU/内存/温度曲线、镜像、容器；类型与可并行实例数 |
| 车辆模型 | 上传 .cmodel 自动解析；卡片俯视图；详情: 3D 透视/俯/侧/前视 + 线框、轮组电机、尺寸载荷、传感器外参；「补全与传感器安装」直接编辑仓库中的模型；版本历史 (新版本继承人工补全) |
| 仿真场景 | 场景包 (scene.json / map.pgm+yaml / topology.json / mesh) 上传；只有栅格时由栅格生成墙体；详情: 3D 图层 (环境/SLAM 栅格/拓扑) 与视角预设；下载场景包 |
| 软件程序包 | 运行程序包 (agv-nav) / 仿真程序包 (agv-sim)；节点镜像自动登记；入库、上传、标签 (基准推荐/测试版…) |
| 仿真记录归档 | 各实例结束任务流时归档的 sim_bundle，可下载或离线回放 |

部署向导四步：计算设备 (可勾选「分离部署」选执行节点) → 车辆模型 → 仿真场景 → 两个程序包；实时校验节点能力/占用/架构/协议版本/模型完整度，部署后显示逐步进度并自动进入工作台。

## 3. 工作台

顶栏：当前实例 · 控制权 (获取/释放/强制获取，持锁者之外为只读) · 安全徽标 · 录制开关 · 下载日志 (RCD/EVT) · 返回平台。

| Tab | 功能 |
|---|---|
| 3-1 任务流配置 | 任务流列表；工步 移动(目标+限速)/顶升/下降/等待；2D 地图 (SLAM 栅格、拓扑、站点、动线预览)，点击站点追加移动工步；任务流保存在平台场景库 |
| 3-2 仿真运行与环境 | 下发/暂停/恢复/重置/终止；自由3D/俯瞰/跟随/车体检视；HUD；带标签事件流 (TSK/NAV/OBS/INJ/RST/DRV/SAF)；轮组状态；**暂停后**出现注入库：栈板/货架/纸箱/人员(可往返行走)，落点 车前2.5m/车前1.2m/下一工位/随机/自定义；手动抽屉 (点动遥控、单点导航) |
| 3-3 仿真记录 | 统计卡 + 记录表，下载 sim_bundle、打包全部、载入回放 |
| 3-4 仿真回放 | 导入 .json 或从 3-3 载入；播放/±2s/拖动/0.5–4x；幽灵轨迹 + 注入元素按时间出现 + 事件高亮 |
| 3-5 传感器与安全 | 急停/复位、触边、光电、激光走廊；DI/DO 开关；相机实时画面 (RGB/深度/ToF)；激光配置 |
| 3-6 系统与导航 | 规划器 (Nav2/Dijkstra/A*/直连)；避障参数；底盘/场景/模型热切换；架构与性能；模型补全、经典调度台入口 |

避障：执行进程在光电/触边/急停之外，新增**激光前向走廊**：走廊宽 = 车宽 + 2×余量，距离从车头算起；< 1.6 m 限速 0.35 m/s，< 0.6 m 停车 (3-6 可调)。

## 3.1 路径规划与精确跟踪

- **规划**：起终点投影到最近可达拓扑边，路网内以 (节点, 来向) 做 Dijkstra；代价 = 路程 + 0.8 m/rad 转向 + 每个拐点 1.5 m (停车转向)。每个拐点按车体外形 (含带载外形) 检查能否转过去，转不过去的拐点加 40 m 惩罚换路线；所有路线都转不过去时规划失败并说明是哪个拐点。
- **执行 (默认 corner_mode=auto)**：车头始终与线路方向一致。直线段用二阶临界阻尼横向控制 (收敛距离 0.6 m)；拐点精确停车 (按剩余路程与延迟补偿减速) → 舵轮先转到位 → 原地转向 (±0.3°) → 舵轮回正 → 起步；原地转不开的拐点用圆弧过渡 (圆弧属于下发路线的一部分，切点停车、先转舵再入弧、闭环跟踪、弧终点停车)。共线的中间拓扑点不停车。离站转不开时先倒车到可转向位置。
- **精度 (云端仿真，5 个场景 24 个任务)**：直线行驶最大横向偏差 ≤ 2.1 mm，圆弧 ≤ 10.4 mm，终点位置误差 ≤ 2.1 mm，航向 ≤ 0.2°，无碰撞。工位前方紧贴设备、车头确实放不下时按「进站受限到位容差」处理并报警。
- **光电保护包络**：光电触发后按检测点位置判断，检测点在包络内 (默认随速度档取停车区，可改为自定义外扩或 always) 才响应；距前方停车点 < 10 cm 时屏蔽前向光电。
- **大车版场景 `fms_workshop_xl`**：保留原 fms_workshop，新增 24 m×24 m 网格通道布局，所有路口/工位可原地转向 (车长 ≤ 2.0 m)。
- **显示**：遥测 `path_index`/`path_labels`/`next_segment`/`plan_curve`/`protection`；3D/2D 显示已走/当前/后续路段、下一航点、参考曲线 (含过渡圆弧) 与车体防护区 (按状态着色)。

## 3.2 车辆保护空间

保存在车辆模型补全 (`model_overrides.json` → `protection`)，随模型版本管理并下发：车体净空、转向防护外扩、过弯方式 (auto/rotate/arc)、按速度分档的防护区 (前/后停车距离、两侧外扩；未配置时按车型减速度 + 0.3 s 反应时间自动生成)、预警区倍数、带载外形 (顶升到位 di_lift_top 后生效)、进站余量与受限到位容差。编辑入口：模型详情「补全与传感器安装 → 保护空间」(俯视图显示各档防护区与原地转向扫掠圆，含制动距离/带载外形/激光覆盖校核)；工作台 3-6「车辆保护空间」可临时应用或保存回模型。执行层：允许速度 = 停车距离放得下的最高一档；前方停车点之外的障碍不影响本段；原地转向有扫掠防护；Nav2 代价地图外形 = 车体 ∪ 带载外形，padding = 车体净空。

## 3.3 定位与导航 (开源方案为主)

**定位来源**：导引、保护空间、到位判断用的都是执行进程的定位结果，不是仿真真值。真值 (`state.truth`) 只用于两件事：启动/复位/车被搬动时的初始位姿 (相当于实车在已知工位设初始位姿)，以及定位误差统计 (界面「定位误差」)。

| 引擎 | 何时使用 | 组成 |
|---|---|---|
| **slam_toolbox** (默认) | 执行容器有 ROS 2 且装了 slam_toolbox / robot_localization (agv-nav 镜像已包含) | robot_localization EKF：轮式里程计速度 + IMU 角速度 → TF odom→base_footprint；slam_toolbox：/scan (360° 合并扫描) → TF map→odom 与 /map (Karto 扫描匹配 + 位姿图回环)。进程由 `nav_runtime/ros_slam.py` 监管，启动文件 `nav2/loc_launch.py`，参数 `nav2/ekf.yaml`、`nav2/slam_toolbox.yaml` |
| 内置 | 无 ROS (`NAV_USE_ROS=0`)、未安装上述包、或 `LOC_ENGINE=builtin` | `nav_runtime/slam.py`：占据栅格建图 + Gauss-Newton 扫描匹配 + 以里程计为先验的 EKF 融合 (只依赖 numpy) |

**模式** (3-6「定位」卡片或 `POST /api/v2/slam/mode`)：`slam` 边建图边定位 → 把车开遍作业区域后「保存地图」→ 该场景之后自动用 `localization` 在固定地图上定位；`odom` 纯里程计 (演示漂移)；`ground_truth` 真值 (对照)。地图保存在执行进程数据目录 `slam_maps/` (slam_toolbox 位姿图 `.posegraph/.data` + Nav2 可用的 PGM/YAML)。

**坐标对齐**：EKF 的 odom 系在启动时用 `/set_pose` 对齐到车辆初始世界位姿，slam_toolbox 建图的第一帧取自 odom，所以 SLAM 地图与场景拓扑在同一坐标系。

**导航** (有 Nav2 时默认规划器为 nav2)：拓扑路线 (Dijkstra，含车体过弯可行性) 按拐点切成直线段，逐段用 Nav2 `FollowPath` 交给 controller_server：RotationShim (偏离路径方向时先原地转向) + Regulated Pure Pursuit，段终点朝向 = 下一段方向，`precise_goal_checker` ±20 mm / ±1°；转不开的拐点用过渡圆弧并入同一段。中断 (障碍物、进度超时) 后等待 2 s 重试。`NAV2_ROUTE_MODE=through_poses` 可改回 NavigateThroughPoses。自研精确导引 (规划器 dijkstra) 保留作对照与后备。Nav2 代价地图的静态层在 slam_toolbox 引擎下直接用它发布的 `/map` (不再加载场景几何地图)。

**实测 (云端仿真，内置 SLAM 引擎 + 自研导引，6 个场景 34 个任务，导引只用定位结果)**：全部到位、无碰撞；定位误差 (相对真值) 峰值 6~26 mm (原地转向时最大)；行驶中横向偏差 29/34 个任务 ≤ 20 mm，超出的 5 个为 20.4 / 20.7 / 21.0 / 28.0 / 45.5 mm (fms、narrow、fms_workshop_xl)。此前「≤ 2 mm」是真值定位下的控制精度，不含定位误差。slam_toolbox + Nav2 组合需在 117 上验证 (云端无 ROS 软件源)。

**验证**：`bash tools/check_loc_stack.sh [容器] [执行端口]` 检查软件包/节点/话题频率/TF；`python3 tools/precision_test.py --gw http://<IP>:<web端口> --planner nav2` 逐工位下发并统计横向偏差、终点误差、定位误差。

## 4. 接口

平台 `/api/hub/*`：nodes (enroll/register/heartbeat)、models (upload、versions/{v}/bundle、versions/{v}/api/v1/model/*)、scenes (upload、definition、map、package、taskflows)、packages (upload 流式、from_node、image)、deployments/check、instances (部署/attach/stop/restart/logs)、records。
实例网关 `/api/v2/*`：lock、info、brief、events、pause、reset、taskflow/run|stop、inject (catalog/add/remove)、records (bundle、bundle_all、archive)、recording、logs、safety、planner、chassis、scene、model、plan (只读路线预览)、slam (状态)、slam/map (grid|pgm|yaml)、slam/mode|save|reset|initialpose。
执行进程新增：`POST /api/v1/taskflows/run|stop`、`GET /api/v1/taskflows/current`、`GET|PUT /api/v1/safety/params` (含 `corner_radius`)、`POST /api/v1/plan` (路线预览)。
仿真进程新增：`POST /api/v1/scene/load`、`POST /api/v1/model/load`、`GET /api/v1/instance`；障碍物支持 `motion` (往返行走，MuJoCo mocap 刚体，无需重编译)。

## 5. sim_bundle

遵循需求数据契约 (metadata/taskFlow/environment/vehicleModel/injectedElements/events/trajectory)，顶层 `schema: "sim_bundle/1"`，并有扩展字段 (实例、节点、结果、避障次数、墙体/货架用于回放)。轨迹 10 Hz：`time, x, y, yaw, vx, w, obsDist, status` (+ `vy, step, safety`)。

## 6. 测试

`python3 tests/test_platform_e2e.py`：平台 + 两个节点代理 (process 运行时，无需 Docker/ROS) → 分离部署 → 任务流 (含顶升/等待/下降) → 注入人员触发减速/停车 → 移除后完成 → 记录/归档/重置/终止。
