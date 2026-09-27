# REST API v1

两个服务都遵守同一套约定 (实现见 `common/rest.py`)：

* 基础路径 `/api/v1`。`GET /api/v1` 返回本服务全部路由 (自描述)。
* 请求体、响应体默认都是 JSON (`application/json; charset=utf-8`)。
* 出错时统一返回 `{"error": {"code": "...", "message": "..."}}`，并带上对应的 HTTP 状态码：400 参数错误、404 资源不存在、405 方法不允许、503 暂无数据。
* 使用 HTTP/1.1 keep-alive，服务端和客户端都开启 `TCP_NODELAY`。局域网内一次 state 轮询约 0.3 ms。
* 已开启 CORS，浏览器可以直接调用。
* 单位：米、弧度、秒、m/s、rad/s。坐标系：`map` (仿真世界，真值在此系下)、`odom` (里程计)、`base_link` (机体)。

---

## 1. 仿真进程 `http://<host>:8090`

|方法|路径|说明|
|---|---|---|
|GET|`/api/v1`|列出全部接口|
|GET|`/api/v1/health`|健康检查|
|GET|`/api/v1/model`|机器人模型|
|GET|`/api/v1/model/urdf`|URDF|
|PUT|`/api/v1/model/chassis`|切换车型|
|POST|`/api/v1/model/reload`|重新解析 cmodel 并重建仿真|
|GET|`/api/v1/world`|场景|
|PUT|`/api/v1/world/scenario`|切换场景|
|GET|`/api/v1/world/obstacles`|动态障碍物|
|PUT|`/api/v1/world/obstacles`|设置动态障碍物|
|GET|`/api/v1/sim`|仿真状态/性能|
|PUT|`/api/v1/sim`|暂停/实时因子|
|POST|`/api/v1/sim/reset`|位姿复位|
|GET|`/api/v1/state`|真值/里程计/关节/碰撞 (执行进程定位只用里程计；真值只用于初始位姿和误差统计)|
|GET|`/api/v1/sensors`|传感器清单|
|GET|`/api/v1/sensors/lidars/{name}`|激光帧 (长轮询/二进制)|
|GET|`/api/v1/sensors/scan`|融合扫描|
|PUT|`/api/v1/sensors/lidar_config`|激光配置|
|GET|`/api/v1/sensors/imu`|IMU|
|GET|`/api/v1/sensors/codes`|读码/工位标签|
|GET|`/api/v1/sensors/bumpers`|防撞触边 (碰撞条)|
|GET|`/api/v1/sensors/photoelectric`|光电传感器|
|GET|`/api/v1/io`|IO 状态|
|PUT|`/api/v1/io`|批量写 IO|
|PUT|`/api/v1/io/{kind}/{name}`|写单个 IO|
|GET|`/api/v1/control`|当前指令/看门狗|
|PUT|`/api/v1/control/cmd_vel`|速度指令 (执行进程 → 仿真)|
|GET|`/api/v1/nav/feedback`|导航回馈|
|PUT|`/api/v1/nav/feedback`|执行进程回馈导航状态|
|GET|`/api/v1/events`|仿真事件|
|GET|`/api/v1/snapshot`|Web 汇总快照|

### 1.1 状态 `GET /api/v1/state`

```json
{"seq": 20178, "t": 218.97, "wall_time": 1790290667.96,
 "truth": {"frame": "map", "x": 0.0, "y": 0.0, "yaw": 0.05, "vx": 0.0, "vy": 0.0, "wz": 0.0},
 "odom":  {"frame": "odom", "x": 0.0, "y": 0.0, "yaw": 0.05, "vx": 0.0, "vy": 0.0, "wz": 0.0},
 "map_to_odom": {"x": 0.0, "y": 0.0, "yaw": 0.0},
 "joints": {"names": ["left_wheel_joint", "right_wheel_joint"], "position": [], "velocity": [], "effort": []},
 "collision": {"count": 0, "bumper_front": false, "bumper_rear": false, "last_contact": null},
 "paused": false, "chassis": "diff_drive", "scenario": "grid_9_square"}
```

### 1.2 激光帧 `GET /api/v1/sensors/lidars/{name}`

| 参数 | 说明 |
|---|---|
| `after_seq=N` | 长轮询：等到出现 `seq > N` 的新帧才返回 |
| `wait=0.5` | 最多等待的秒数。超时后返回当前最新帧，客户端对比 `seq` 就能判断是不是新帧 |
| `Accept: application/octet-stream` 或 `format=bin` | 按二进制返回 |

二进制响应头：`X-Seq`、`X-Stamp` (仿真时间)，以及 `X-Meta` (JSON：`type`、`frame_id`、`pose`、`angle_min`、`angle_increment`、`range_min/max`、`scan_hz`、`count`、`layout`)。

| 类型 | 消息体 |
|---|---|
| 2D | `float32 LE [N]` 距离数组，`inf` 表示没有回波 |
| 3D (Mid-360S) | `float32 LE [N,4]` (x, y, z, intensity，传感器坐标系) + `uint8 [N]` line |

不加 Accept 时返回 JSON，便于调试。`GET /api/v1/sensors/scan` 是融合后的 360° 扫描，坐标系为 `base_link`，接口形式同上。

### 1.3 光电 `GET /api/v1/sensors/photoelectric`

```json
{"t": 218.98, "sensors": [{"name": "front_left", "di": "di_pe_front_left", "kind": "diffuse",
  "detected": true, "distance_m": 0.218, "trigger_m": 0.35, "range_m": 1.0,
  "mount": {"x": 1.308, "y": 0.497, "z": 0.12, "yaw": 0.5236}, "source": "default: 角部避障光电"}]}
```

### 1.4 防撞触边 `GET /api/v1/sensors/bumpers`

```json
{"t": 12.3, "count": 1, "last_contact": [0.46, 0.0], "any_pressed": true,
 "strips": [{"name": "front", "side": "front", "di": "di_bumper_front", "pressed": true, "press_count": 1,
             "polygon": [[1.308, 0.477], [1.338, 0.477], [1.338, -0.477], [1.308, -0.477]]}],
 "bumper_front": true, "bumper_rear": false}
```

### 1.5 IO `GET /api/v1/io`、`PUT /api/v1/io/{di|do}/{name}`

```json
{"inputs":  {"di_estop": false, "di_bumper_front": false, "di_bumper_rear": false, "di_cargo_present": false,
              "di_lift_top": false, "di_lift_bottom": true, "di_pe_front_left": false},
 "outputs": {"do_brake_release": true, "do_tower_green": true, "do_tower_yellow": false, "do_tower_red": false,
              "do_buzzer": false, "do_lift_motor_up": false, "do_lift_motor_down": false},
 "lift_height_mm": 0.0, "is_emergency_stop": false, "bumper_active": false}
```

写单个 IO：`PUT /api/v1/io/di/di_estop` `{"value": true}`
批量写：`PUT /api/v1/io` `{"di": {...}, "do": {...}}`

### 1.6 控制量回馈 `PUT /api/v1/control/cmd_vel`

请求：`{"vx": 0.3, "vy": 0.0, "wz": 0.1, "source": "nav:dijkstra"}`。超过 0.5 s 没有新指令，看门狗会置零速度。
`GET /api/v1/control` 返回当前指令、来源、指令时长 (`age_s`) 和看门狗、急停状态。

### 1.7 导航状态回馈 `PUT /api/v1/nav/feedback`

由执行进程以 5 Hz 写入，内容与 `GET {nav}/api/v1/nav` 相同 (见 2.1)。仿真进程读出时会附加 `age_s` 字段；超过 3 s 没有更新就标记为 `online=false`。

### 1.8 模型与世界

* `GET /api/v1/model`：cmodel 解析结果 (底盘、轮组、激光、相机、IO) 加 `active_chassis` / `active_wheels`。
* `GET /api/v1/model/urdf`：当前车型的 URDF (`application/xml`)。
* `PUT /api/v1/model/chassis` `{"type": "diff_drive"}`：切换车型。
* `POST /api/v1/model/reload` `{"cmodel_path": "/workspace/tests/data/测试车模型.cmodel", "load": "full"}`：重新解析 cmodel，并重建仿真。
* `GET /api/v1/world`：场景 (墙体、工位、拓扑、原点、障碍物)。
* `PUT /api/v1/world/scenario` `{"id": "grid_9_square"}`：切换场景。
* `PUT /api/v1/world/obstacles`：`[{"x", "y", "w", "h", "z", "yaw", "type"}]`。

### 1.9 相机类传感器（单目 / 双目 / ToF）

`GET /api/v1/sensors/cameras` 返回相机清单：名称、类型、分辨率、视场、内参 K（行优先 3×3）、`frame_id`、可用的流，以及最新帧序号、`active` (当前是否在成像)。
相机按需成像：最近 `SIM_CAMERA_IDLE_S` 秒 (默认 3) 内没有人取帧就停止渲染；闲置后的第一次取帧会等一帧新图 (至少等 1 s)，而不是返回旧帧。

`GET /api/v1/sensors/cameras/{name}?stream=&format=&after_seq=&wait=` 取一帧（支持长轮询）。

| 相机 | stream | format | 内容 |
|---|---|---|---|
| 单目 | `rgb` | `jpeg`（默认）/ `png` / `raw` | raw 为 `uint8[H,W,3]` rgb8 |
| 双目 | `left` / `right` | 同上 | 右目 `frame_id` 为 `<name>_right_optical_frame` |
| 双目 / ToF | `depth` | `png`（默认，16 位，单位 mm）/ `jpeg`（伪彩色）/ `raw`（`float32[H,W]`，单位 m，NaN 表示无效） | 光学坐标系 z 深度 |
| ToF | `amplitude` | `png`（16 位）/ `jpeg` / `raw`（`uint16`） | 回波幅度 |
| ToF | `points` | `raw` | `float32[N,3]`，光学坐标系点云 |

响应头 `X-Meta` 包含：`width`、`height`、`K`、`frame_id`、`encoding`、`pose`。

噪声模型：
- **RGB**：σ = 2 DN。
- **双目深度**：σz = z²·σd/(f·b)。
- **ToF**：σ = 5 mm + 0.4%·z，低幅度像素判为无效。

### 1.10 模型补全

| 方法 路径 | 说明 |
|---|---|
| GET `/api/v1/model/editor` | `{spec, overrides, audit, sensors, sensor_types}`，供编辑器使用 |
| GET `/api/v1/model/audit` | 完整度审计：每项的 `source`（cmodel / library / inferred / default / manual / missing）和 `severity`（ok / info / warn / missing） |
| GET `/api/v1/model/overrides` | 当前人工补全（`model_overrides.json`） |
| PUT `/api/v1/model/overrides` | 保存并应用，重写 robot_config.json 和 robot.urdf 并重建仿真，返回同 editor |
| POST `/api/v1/model/preview` | 只试算不保存：返回 spec、audit、sensors、urdf |
| GET `/api/v1/model/sensor_template?type=` | 新增传感器时的默认参数（lidar2d / lidar3d / camera / stereo / tof / photoelectric / codeReader） |

`model_overrides.json` 结构：

```json
{"version": 1,
 "chassis": {"mass_kg": 320, "com": [0.4, 0, 0.35]},
 "wheels":  {"rear_left_load_wheel": {"radius_m": 0.09, "mass_kg": 6}},
 "motors":  {"walk-motor": {"rated_torque_nm": 5.1}},
 "lift":    {"enabled": true, "stroke_m": 0.08, "speed_mps": 0.02, "x": 0.4, "z": 0.3, "size": [1.2, 0.8, 0.04]},
 "sensors": {"laser":     {"library": "mid-360s"},
             "front_cam": {"type": "camera", "_added": true, "x": 1.3, "z": 0.9, "pitch": 0.14, "width": 320, "height": 240, "hfov_deg": 70},
             "laser0":    {"_removed": true}}}
```

位姿单位为 m 和 rad，roll、pitch、yaw 在 base_link 系下（pitch 为正时向下俯）；型号参数中带 `_deg` 后缀的字段单位为度。
应用后，`GET /api/v1/state` 中的 `model_rev` 会加 1，执行进程和网关据此自动刷新模型。

---

## 2. 执行进程 `http://<host>:8091`

|方法|路径|说明|
|---|---|---|
|GET|`/api/v1`|列出全部接口|
|GET|`/api/v1/health`|健康检查|
|GET|`/api/v1/nav`|执行状态|
|PUT|`/api/v1/nav/planner`|切换规划器|
|GET|`/api/v1/missions`|最近任务|
|POST|`/api/v1/missions`|下发任务|
|GET|`/api/v1/missions/current`|当前任务|
|GET|`/api/v1/missions/{id}`|任务详情|
|DELETE|`/api/v1/missions/current`|取消当前任务|
|DELETE|`/api/v1/missions/{id}`|取消任务|
|POST|`/api/v1/teleop`|手动遥控|
|GET|`/api/v1/events`|执行事件|
|GET|`/api/v1/ros`|ROS 图|
|GET|`/api/v1/slam`|定位状态 (模式、位姿、map→odom、标准差、匹配统计、相对真值误差)|
|GET|`/api/v1/slam/map`|SLAM 地图 `?part=grid` (默认，JSON：`res`/`origin`/`w`/`h`/`data` base64，0 未知 1 空闲 2 占据；`&step=N` 下采样) \| `pgm` \| `yaml`|
|POST|`/api/v1/slam/mode`|`{"mode": "slam" \| "localization" \| "odom" \| "ground_truth"}`|
|POST|`/api/v1/slam/save`|保存当前场景地图 (之后该场景自动进入 localization)|
|POST|`/api/v1/slam/reset`|清空地图重新建图 `{"delete_saved": false}`|
|POST|`/api/v1/slam/initialpose`|设置初始位姿 `{"x", "y", "yaw"}`|

### 2.1 执行状态 `GET /api/v1/nav`

```json
{"online": true, "mission_id": 3, "status": "NAVIGATING", "planner": "nav2",
 "goal": {"x": 3.0, "y": 0.0, "yaw": 0.0}, "path": [{"x": 0, "y": 0}], "dist_remaining": 2.26,
 "planners": ["dijkstra", "astar", "direct", "straight", "nav2"],
 "nav2": {"server_ready": true, "process": true, "chassis": "diff_drive"},
 "mission": {"id": 3, "status": "NAVIGATING", "planner": "nav2", "duration_s": 4.2},
 "safety": {"estop": false, "bumpers": [], "photo": {"front": [], "rear": [], "left": [], "right": []}},
 "link": {"sim_url": "http://127.0.0.1:8090", "online": true, "state_hz": 49.4,
          "lidar_frames": {"laser": 1433}, "cmd_sent": 322, "errors": 0},
 "ros": {"nodes": ["/nav_runtime_bridge"], "topics": ["/odom"]}}
```

任务状态 `status` 的取值：
`PLANNING` / `NAVIGATING` / `OBSTACLE_WAIT` / `ARRIVED` / `FAILED` / `NO_PATH` / `CANCELED` / `SUPERSEDED` / `SAFETY_STOP` (急停挂起) / `BUMPER_STOP` (触边终止)

### 2.2 定位 `GET /api/v1/slam`

执行进程用「激光 SLAM + 轮式里程计」定位，导引用的就是这个位姿；`state.truth` 只用于初始位姿和误差统计。

```json
{"mode": "slam", "pose": {"x": 5.0012, "y": -0.0008, "yaw": 0.00003},
 "map_to_odom": {"x": 0.0152, "y": -0.0031, "yaw": 0.0021}, "std_xy_mm": 1.5, "std_yaw_deg": 0.012,
 "err_mm": 1.4, "err_yaw_deg": 0.004, "err_rms_30s_mm": 1.9, "err_max_30s_mm": 4.1,
 "map": {"res": 0.025, "w": 1128, "h": 1129, "updates": 57, "saved": false},
 "stats": {"matches": 812, "rejects": 0, "match_ms": 2.9, "hz": 9.8,
           "last_info": {"inliers": 0.99, "score": 0.86, "n": 1200, "accepted": true}}}
```

执行回馈 (`/api/v1/nav` → 网关遥测 `localization`) 里带一份精简版 (`mode`、`err_mm`、`std_xy_mm`、`pose`)。

### 2.3 下发任务 `POST /api/v1/missions`

* 按坐标：`{"x": 3.0, "y": 0.0, "yaw": 0.0, "planner": "nav2"}`
* 按工位：`{"station": "S4"}`

返回任务视图。取消任务用 `DELETE /api/v1/missions/current`。

---

## 3. Web 网关 `http://<host>:8088`

网关另外提供透明代理：`/sim/api/v1/*` 转发到仿真进程，`/nav/api/v1/*` 转发到执行进程；模型补全页 `/model` 就通过它访问。

网关保留原前端的全部接口 (`/api/telemetry`、`/api/navigate_to_pose`、`/api/chassis_type`、`/api/map_scenario`、`/api/obstacles/*`、`/api/sim_pause|resume|reset`、`/api/set_io`、`/api/events`、`/api/replay/*` …)，在内部把它们映射到上面两个服务的 REST v1。
`/api/telemetry` 新增的字段：`arch` (两个进程的在线状态、链路统计、指令来源)、`photoelectric`、`bumpers`、`safety`。
