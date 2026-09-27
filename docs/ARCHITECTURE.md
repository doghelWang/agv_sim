# AMR Studio V4 · 三进程 RESTful 架构

> 多机部署、资源管理 (节点/模型/场景/程序包) 与新工作台见 [PLATFORM.md](PLATFORM.md)：平台服务 agv-hub 在多台树莓派上编排本文的 agv-sim / agv-nav 实例。

仿真引擎和导航执行拆成两个独立进程，另加一个 Web 网关。**进程之间只走 HTTP REST，不再用 ROS 话题互相订阅。**
ROS 2 只留在执行进程内部，作为驱动 Nav2 的"局部总线"。

```
                        浏览器 (index.html · 2D/3D 调度台)
                                   │  HTTP  (/api/telemetry, /api/navigate_to_pose …，旧接口保持兼容)
                                   ▼
┌──────────────────────────── Web 网关 web_gateway.py :8088 ────────────────────────────┐
│  聚合: 20 Hz GET sim/snapshot · 5 Hz GET nav/nav · 2 Hz model/world/events              │
│  指令: 任务/规划器 → 执行进程；车型/场景/障碍/暂停/IO → 仿真进程                        │
│  飞行记录器 · 系统性能监视 · 事件中心 (合并两进程事件)                                  │
└───────────────┬───────────────────────────────────────────────────┬────────────────────┘
                │ REST                                              │ REST
                ▼                                                   ▼
┌──────── 仿真进程 sim_server :8090 (无 ROS) ────────┐   ┌──── 执行进程 nav_runtime :8091 ────────────────────┐
│ 模型: cmodel → robot spec → 运动学/传感器/URDF    │   │ SimLink (纯 REST 数据接入)                          │
│ 世界: 场景墙体/货架/工位/拓扑 + 动态障碍          │◄──┤   GET state 50 Hz · GET io 20 Hz                    │
│ 引擎: MuJoCo (物理+射线)，固定步长 10 ms          │   │   GET lidars/{name} 长轮询 (二进制) · GET scan      │
│ 数据产生:                                         │   │   PUT control/cmd_vel  ← 控制量回馈                 │
│   2D 激光 ×N · Mid-360S 3D 点云 · 融合 360° scan  │──►│   PUT nav/feedback 5 Hz ← 导航状态回馈              │
│   IMU · 编码器里程计(含漂移) · 读码相机            │
│   单目 RGB · 双目 (左右+深度) · ToF (深度/点云)    │   │ Navigator: Dijkstra / A* / 直连 导引 + 任务管理     │
│   光电 (近场避障/在位) · 防撞触边(碰撞条)          │   │ 安全层: 急停挂起 · 触边终止 · 光电方向封锁          │
│   工业 IO: 急停/触边/光电/抱闸/塔灯/顶升            │   │ RosBridge (进程内部 ROS 2):                          │
│ 安全互锁: 急停→抱闸；触边→方向封锁(可后退脱困)    │   │   REST 数据 → /odom /tf /scan /livox/lidar /joint…  │
│ 看门狗: 0.5 s 无指令 → 停车                        │   │   Nav2 /cmd_vel → safety_filter → REST cmd_vel      │
└───────────────────────────────────────────────────┘   │   robot_state_publisher (URDF 取自 REST)            │
                                                        │   Nav2Supervisor: 按车型生成参数并拉起 Nav2         │
                                                        └──────────────────────────────────────────────────────┘
```

## 1. 职责划分

| 进程 | 入口 | 依赖 | 职责 |
|---|---|---|---|
| 仿真进程 | `python3 -m sim_server.api` | numpy + MuJoCo（未安装时退回 kinematic），**不依赖 ROS** | 模型解析与构建、物理步进、全部仿真数据的产生、接收执行进程的控制量和导航状态回馈 |
| 执行进程 | `python3 -m nav_runtime.main` | ROS 2 Humble + Nav2 (`NAV_USE_ROS=0` 时不需要) | 通过 REST 拿仿真数据 → 导航规划和控制 → 把控制量和导航状态回馈给仿真进程 |
| Web 网关 | `python3 web_gateway.py` | psutil，**不依赖 ROS** | 汇总两个进程的数据，给浏览器提供调度台；前端原有 `/api/*` 接口不变 |

> 执行进程里的 ROS 话题 (`/odom`、`/scan`、`/livox/lidar`、TF…) 只给同一进程里的 Nav2 用，不再作为进程之间的通信方式。
> 换掉 Nav2、或者接入真实 AGV 控制器时，只要实现同一套 REST 客户端 (SimLink) 就行。

## 2. 数据流 (一个控制周期)

1. **仿真进程**的物理线程按 10 ms 推进一步；传感器线程按各激光的频率扫描，把结果写入带 `seq` 的缓冲区，同时唤醒长轮询请求。
2. **执行进程**的 SimLink：
   * `GET /api/v1/state` 50 Hz：拿里程计、关节、碰撞 (以及真值，仅用于初始位姿和定位误差统计)。
   * 定位：`nav_runtime/slam.py` 用各激光原始帧做 SLAM 扫描匹配，与里程计融合得到 map 位姿 (导引用这个位姿)。
   * `GET /api/v1/sensors/lidars/{name}?after_seq=N&wait=0.5` (二进制)：每个新帧**只取一次**，不会漏帧也不会重复。
   * `GET /api/v1/io` 20 Hz：拿急停、触边、光电的 DI。
3. Navigator (或 Nav2) 算出速度后，经过 `safety_filter`，再 `PUT /api/v1/control/cmd_vel {"vx","vy","wz","source"}`。
4. 仿真进程收到指令后记下指令来源和时间戳。超过 0.5 s 没有新指令，看门狗会让车停下。
5. 执行进程 5 Hz `PUT /api/v1/nav/feedback`，回写任务、路径、剩余距离和 Nav2 状态；仿真进程把这些写进快照，再由 Web 呈现。

## 3. 开关量传感器仿真 (sim_core/discrete.py)

| 传感器 | 来源 | 模型 | 输出 |
|---|---|---|---|
| 光电 | cmodel `sensor/photoelectric\|PT\|proximitySensor` 的安装位姿；没有的话在车身四角放 45° 斜向避障光电 | 窄锥形 3 条射线，看不到比安装高度更矮的物体；触发阈值带 2 cm 回差 | `di_pe_<name>`、检测距离 |
| 防撞触边 | 车身轮廓前、后边缘 (可选左、右) 外扩 3 cm 的检测带 | 和世界线段做多边形相交；释放后保持 0.5 s | `di_bumper_front/rear`、按压次数、多边形 |
| 急停 | IO `di_estop` | 断开抱闸 → 全停 | `is_emergency_stop` |

安全链分两层，互为冗余：
* **仿真进程 (硬件互锁)**：急停 → 抱闸。触边压下 → 禁止继续往该侧走，但允许反方向脱困；同时红灯亮、蜂鸣器响。
* **执行进程 (软件安全层)**：急停 → 挂起任务，复位后自动恢复。触边 → 任务终止 (`BUMPER_STOP`)，需要人工重新下发。前向光电触发 → 当作障碍物，减速停车等待 (补上激光近场的盲区)。

## 4. 仿真引擎：MuJoCo

MuJoCo 与 PyBullet 的对比和实测数据见 [ENGINE_EVALUATION.md](ENGINE_EVALUATION.md)。

- **物理**：车体为平面三自由度刚体，由速度执行器驱动，与墙体、障碍物做接触约束。轮组运动学（先转后走、打滑）仍由 `ChassisKinematics` 计算。
- **传感器**：激光、光电、ToF、双目深度、相机统一用 `mj_multiRay` 批量射线（多线程，返回法向）；地面和屋顶按解析平面计算，地面标识查纹理表。
- **相机成像**：默认用射线着色，不需要 GPU；设置 `SIM_CAMERA_RENDER=gl` 可改用 OpenGL 光栅渲染。

## 5. 模型补全与传感器安装

数据流：`cmodel` → `robot_config.base.json`（纯解析结果）→ 叠加 `model_overrides.json`（人工补全）→ `robot_config.json` + `robot.urdf` → 重建仿真。

- **完整度审计**：`GET /api/v1/model/editor` 逐项列出参数来源（cmodel / 型号库 / 推断 / 默认 / 人工）和缺失项。
- **编辑页面**：Web 地址为 `/model`。
  - 可编辑车体质量与质心、轮组、电机转矩、顶升机构。
  - 可增删改传感器：2D/3D 激光、单目、双目、ToF、光电、读码。
  - 俯视图可拖动传感器调整安装位置；侧视图显示安装高度；可预览仿真图像；可下载 URDF。
- **应用方式**：`PUT /api/v1/model/overrides` 保存后，仿真进程重建模型，`model_rev` 加 1。执行进程和网关检测到变化后，自动刷新传感器与 URDF：robot_state_publisher 重启，并新增相机话题。
- **URDF 补充内容**：相机 `_link` 与 `_optical_frame`（双目另有右目光学坐标系）、光电 link、顶升 `lift_joint`（prismatic）、质心、轮质量。

## 6. 部署（两个镜像）

| 镜像 | 包含的进程 | 依赖 |
|---|---|---|
| `agv-sim` | 仿真进程 + Web 网关 | Python、MuJoCo，不含 ROS |
| `agv-nav` | 执行进程 | ROS 2 Humble + Nav2，不含仿真代码 |

执行镜像所需的场景定义、Nav2 栅格地图、URDF 和 Nav2 限速，全部通过 REST 从仿真进程获取。

```bash
./deploy.sh up                                             # 同机部署两个容器
NAV_API=http://<导航机>:8091 ./deploy.sh up sim            # 分机部署：仿真机
SIM_API=http://<仿真机>:8090 ./deploy.sh up nav            # 分机部署：导航机
```

详细说明见 [DEPLOY.md](DEPLOY.md)。

## 7. 测试

| 测试 | 内容 |
|---|---|
| `tests/test_sim_core.py` | 仿真内核单元测试（运动学、激光、Mid-360S、触边、光电、MuJoCo 接触与相机深度、模型补全往返） |
| `tests/test_platform_e2e.py` | 平台端到端：节点注册、仓库上传、分离部署、任务流/注入/避障、记录归档、终止 |
| `tests/test_rest_e2e.py` | 三进程端到端测试：接口自描述、传感器二进制帧、网关聚合、任务闭环到位精度、车型切换传播、障碍/暂停、光电/急停/触边安全链 |
