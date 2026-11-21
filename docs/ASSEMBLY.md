# cmodel → 装配模型 (RobotAssembly)：外观、执行机构、真实运动与仓储环境

> 这次重写的目标：**以 cmodel 作为车辆唯一描述文件**，自动生成像真车的 AMR，包括底盘、万向轮/舵轮、激光、读码相机，以及顶升 / 货叉 / 辊筒 / 牵引等上装。
> 生成的 AMR 要像真车一样运动和动作，环境也要像真实仓库：真二维码、货架、托盘、输送线、充电桩、立柱，并能导出 SLAM 点云。
> cmodel 不包含的信息会逐项列出（见第 2 节）。

## 1. 数据流

```
.cmodel ──cmodel crate──▶ spec (底盘/轮组/电机/传感器/IO/限速)
                             │  + model_overrides.json (人工补全，新增 "actuator": {...})
                             ▼
              sim_core::assembly::build(spec) ─▶ RobotAssembly JSON
                ├─ links / joints / visuals (基本体)     → URDF   (model::urdf, GET /api/v1/model/urdf)
                │                                       → MJCF   (铰接机器人体, GET /api/v1/model/mjcf)
                │                                       → Web 3D (vendor/amr_assembly3d.js，按 joint_states 驱动)
                ├─ archetype + actuator 参数           → 执行机构运行时 (actuator.rs / fork.rs)
                └─ missing[] (cmodel 缺失 / 推断 / 默认) → GET /api/v1/model/missing、URDF 头注释、平台模型详情
```

坐标约定沿用 URDF：`base_link` 原点为运动中心、位于地面 (z=0)，x 指向车头，y 指向左侧，z 朝上。

## 2. cmodel 有什么、缺什么

cmodel (`CompDesc.json`) 只按**电气 / 逻辑模块**描述车辆，内容包括：底盘外廓偏移、轮组与电机（转速、减速比、抱闸）、激光和相机的安装位姿、IO（接近开关、按钮）、电池以及限速/加速度。
它**没有机构几何和外观**。下表列出 3 个真实 cmodel 的情况：

| 项目 | 0323 / 1234 (顶升潜伏) | AOBO cmodel555 (前移式叉车) | 12345 (差速底盘) |
|---|---|---|---|
| 底盘外廓 / 车高 | 1476×1063 mm, 车高 178 mm | 1494×1607 mm, 1.94 m | 1200×800 mm, 0.4 m |
| 自重 | **缺失** (按投影面积估 235 kg) | 700 kg | **缺失** |
| 上装电机 | motor-lift (减速比 35, 抱闸) | motor_DoorFrame (门架前移, 减速比 26.25) + 液压泵 | 无 |
| 上装限位/传感 | up_sensor / down_sensor | DoorFrame_origin/limit, up/down_limit, CB_Front/Back, 拉线编码器 | — |
| **机构类型** | 未声明 → 推断 `jacking` | 未声明 → 推断 `reach_fork` | `none` |
| **行程 / 速度** | 缺失 (默认 60 mm, 30 mm/s) | 起升高度/速度缺失 (默认 1.6 m, 0.2 m/s)；前移行程缺失 (默认 0.63 m) | — |
| **托盘 / 货叉 / 辊筒 / 挂钩尺寸** | 缺失 (托盘 Ø0.77 m) | 货叉长宽厚、间距缺失 (1070×100×40, 560) | — |
| 万向轮 | 无模块 → 在四角推断 | 推断 2 个 | 推断 4 个 |
| 万向轮偏心距 | 缺失 (默认 0.35 r) | 缺失 | 缺失 |
| 质心 / 惯量 | 缺失 (按均质盒体) | 缺失 | 缺失 |
| 外观网格 / 材质 | 缺失 (用参数化基本体拼装) | 缺失 | 缺失 |
| 轮胎摩擦 | 缺失 (默认 μ=0.8) | 缺失 | 缺失 |
| 激光光学参数 | WLR-716MINI 不在型号库 → 通用 270° 25 m | 2D laser 型号缺失 → 通用；laser_350 = Mid-360 (库内) | 无激光 → 默认车头 270° |
| 读码相机内参 | 缺失 (默认 320×240, 44°) | 缺失, 安装高度 0 → 默认 0.1 m | — |

以上每一项在 `GET /api/v1/model/missing` 中都有对应条目，格式为 `{category, item, status: missing|inferred|defaulted|estimated, value, note}`。
URDF 文件开头的注释块也会逐条列出，平台「模型详情 → 执行机构 · cmodel 缺失项」卡片同样会显示。

补全方式：在 `model_overrides.json` 中加入 `actuator` 段（平台模型补全会保存到同一个文件），例如：

```json
{"version": 1, "actuator": {"type": "jacking", "stroke_m": 0.05, "lift_speed_mps": 0.025, "tray_diameter_m": 0.8}}
```

被覆盖的字段会从 missing 列表中消失，并记入 `provenance` (`actuator.stroke_m: manual`)。

## 3. 上装类型判定 (`assembly::classify`)

判定顺序（`actuator.type` 人工覆盖优先）：

1. 模块名、电机型号或 IO 名称含 `roller / conveyor / 滚筒 / 辊` → **roller**
2. 含 `tow / hitch / hook / tug / 牵引` → **tugger**
3. 含 `doorframe / reach / oil_pump / 门架 / 前移` → **reach_fork**
4. 有 `lifter.forkMast` 模块：起升行程 ≤ 0.35 m → **pallet_truck**，否则 → **stacker_fork**
5. 含 `lift / jack / 顶升`（例如 0323 的 motor-lift） → **jacking**
6. 以上都不满足 → **none**

| archetype | 关节 (joint_states 名称) | 说明 |
|---|---|---|
| jacking | `lift_joint` (prismatic z), `tray_rotate_joint` (continuous) | 圆形顶升托盘，嵌在车顶；托盘可同步旋转 |
| reach_fork | `reach_joint` (x), `mast_stage_joint` (= h/2, mimic), `fork_lift_joint` | 跨腿 + 前移门架 + 两级门架 + 叉架/货叉 |
| stacker_fork | `mast_stage_joint`, `fork_lift_joint` | 门架固定在车头 |
| pallet_truck | `fork_lift_joint` | 低位货叉 + 叉尖承载轮 |
| roller | `roller_{i}_joint` (continuous, mimic roller_0), `stopper_l/r_joint` | 辊面横向输送，两侧挡停销 |
| tugger | `hitch_pin_joint` | 车尾挂钩 + 升降销 |

所有车型都有：`{wheel}_joint` 或 `{wheel}_steer_joint`/`{wheel}_drive_joint`，以及万向轮的 `{caster}_swivel_joint`/`{caster}_wheel_joint`。

## 4. 真实运动

- **万向轮自对正**：脚轮绕摆动轴随回转中心的速度方向自对正，`dφ = v/e·sin(θv−φ)·dt`。车辆原地旋转时脚轮会转到切向，直行时回正。
- 舵轮的转向速率和速度门控、驱动轮的加减速坡道沿用原有 `kin.rs`。驱动轮和脚轮的转角都会输出到 `joint_states`，3D 视图据此实时驱动轮子转动和转向。
- 带载时线速度和角速度降到 75%（叉车按 cmodel 的 `load_speed_factor`）。
- 载荷参与碰撞：车辆运动前会预测 0.06 s 后的位姿。如果被顶起的货架、辊筒上伸出的货箱或牵引料车将与墙、货架或其他物体相交，底盘会停车并发出 `LOAD_COLLISION` 事件。

## 5. 执行机构 (`PUT /api/v1/actuator`)

| 类型 | 指令 | 行为 / 事件 / IO |
|---|---|---|
| jacking | `{action: up\|down\|stop}`、`{height}`、`{tray_deg}`、`{rotate_deg}`、`{follow: bool}` | 托盘顶到货架底部时挂接货架；偏心 > 200 mm 拒绝顶升 (`JACK_MISALIGNED`)，> 30 mm 告警 (`JACK_OFFSET`)。顶起后托盘同步反向旋转，原地转向时货架朝向保持不变；下降到地面时放下 (`JACK_PICK/JACK_PLACE`)。IO：`di_lift_top/bottom` 以及 cmodel 中的 `di_up_sensor/di_down_sensor`、`di_cargo_present`、`do_lift_motor_up/down` |
| reach_fork / stacker_fork / pallet_truck | `{action: up\|down\|top\|bottom\|stop\|reach_out\|reach_in}`、`{height}`、`{reach}` | 沿用原有货叉/托盘物理 (9 块四向进叉托盘)。新增门架前移：前移量带动货叉和已叉起的货物，接触即停 (`FORK_REACH_BLOCKED`)。IO：`di_reach_origin/limit`、`do_reach_out/in` |
| roller | `{action: load\|unload, side: left\|right}`、`forward/reverse/stop` | 接货时，输送线把对接侧、同高度的货箱送上辊面并居中 (`ROLLER_LOADED`)；送货时货箱被推出，到达输送线中心后释放 (`ROLLER_UNLOADED`)；对接侧没有输送线时货箱跌落到地面 (`ROLLER_DROP`)。挡停销在输送侧收回。IO：`di_roller_in_place/running`、`do_roller_forward/reverse` |
| tugger | `{action: lock\|release}` | 挂钩销落下，牵引环在 8 cm 内即挂接 (`TOW_HITCHED`)；料车按牵引杆做拖车运动；折叠角超过 75° 时发出 `TOW_JACKKNIFE` 并禁止继续加大折叠。IO：`di_hitch_locked/di_cart_present`、`do_hitch_lock` |

`GET /api/v1/actuator` 返回统一视图（`type/state/loaded/load_id/...`），快照和网关遥测中对应 `actuator` 字段。

货物新增 `POST /api/v1/world/cargo {kind: rack|cart|tote|pallet|box}`：

- 顶升货架默认比车体长 0.16 m、宽 0.24 m，腿下净空 = 车高 + 20 mm。
- 料车带牵引杆、牵引环和 4 个脚轮。

## 6. 仓储环境

- **物体分解** (`world::obstacle_parts`)：托盘、货架、料车、输送线、充电桩、立柱、货架立柱和横梁、护栏都会分解成多个基本体。MuJoCo (激光 / 接触)、相机光线追踪、PCD 和 Web 3D 使用同一套分解结果。
  例如车辆可以从货架腿之间钻入，激光只会打到四条腿。
- **布景** (`props.rs`，`SIM_DRESS=auto|demo|none`，`PUT /api/v1/world/dressing {mode}`)：
  - 6 m 柱网立柱，充电节点（或原点）的墙边放充电桩；
  - 辊筒车在站点旁放输送线；
  - 按车型在路网之外布置货架、托盘（部分堆纸箱）、托盘货架（含横梁上的托盘）、料车、周转箱、护栏。
  - 所有物体与路网边线的距离不小于车半宽 + 0.2 m。
  - `demo` 模式还会在每个站点放一个待取货物。
- **真二维码**：每个拓扑节点地面贴 8 cm 的 QR 码（ISO/IEC 18004，由 qrcodegen 生成，内容为 `QR_<节点>`），作为高分辨率贴花参与相机着色。
  下视读码相机 (`codeReader`) 现在输出灰度图像 `GET /api/v1/sensors/cameras/<name>`，用 OpenCV `QRCodeDetector` 可以直接解码。
  `GET /api/v1/sensors/codes` 增加 `symbology: "QR"`、`payload` 字段。
- **相机材质**：墙面有黄黑踢脚线和分格，木托盘有木纹，纸箱有封箱胶带，立柱有警示色带，钢结构和辊筒也有对应材质。
- **SLAM 点云地图**：`GET /api/v1/world/pcd?res=0.05&floor=0&ceiling=0&z_max=6&format=binary|ascii` 输出 PCD v0.7 (x y z intensity)。点在墙、货架、道具、货物的可见表面上规则采样，移动障碍和人员除外。
- **Web 3D**：
  - 车辆按装配模型绘制，关节由 `joint_states` 驱动：轮子转动、脚轮摆动、顶升托盘、货叉、门架、辊筒和挂钩都会动。
  - 环境按 `scenario_metadata.props` / `qr_codes` 绘制道具和真二维码。
  - 工作台 HUD 增加执行机构控制按钮。

## 7. 接口一览 (sim-server)

| 方法 | 路径 | 说明 |
|---|---|---|
| GET | `/api/v1/model/assembly` | 装配模型 (links/joints/visuals/actuator/missing/joint_names) |
| GET | `/api/v1/model/missing` | cmodel 缺失项 |
| GET | `/api/v1/model/urdf` | 由装配模型生成的 URDF (含 mimic 关节与缺失项注释) |
| GET | `/api/v1/model/mjcf` | 当前场景 + 铰接机器人 MJCF |
| GET/PUT | `/api/v1/actuator` | 上装执行机构状态 / 控制 |
| GET | `/api/v1/world/props` | 布景道具 |
| PUT | `/api/v1/world/dressing` | 重新布景 |
| GET | `/api/v1/world/pcd` | 场景点云地图 |

平台 hub 新增：`GET /api/hub/models/{mid}/versions/{ver}/api/v1/model/assembly`；模型编辑器和预览的返回结果中也带 `assembly`。

## 8. 新增 cmodel 与测试

- `tools/make_amr_family_cmodels.py` 生成 `models/latent_jacking_amr.cmodel`（参数取自真实 0323）、`models/roller_amr.cmodel` 和 `models/tugger_amr.cmodel`。
  与真实 cmodel 一样，这些文件只包含模块信息。
- `crates/sim-core/tests/actuators.rs` 覆盖以下场景，运动学模式和 MuJoCo 模式（设置 `MUJOCO_LIB` 时）都会运行：
  - 顶升：钻入货架、顶起、原地转向时货架朝向不变、搬运、放下；
  - 辊筒：从左侧输送线接货、向右侧输送线送货、无输送线时跌落；
  - 牵引：挂接、转弯牵引、脱钩；
  - 前移门架、万向轮摆动、所有车型的 MJCF 编译；
  - 激光穿过货架腿下方、读码相机拍到真二维码、布景无碰撞。
- `sim-core` 单元测试覆盖：类型判定、装配模型生成、QR finder pattern、布景避开路网、PCD。`model` 单元测试检查 URDF 结构完整。
- `cargo run --release -p sim-core --example assembly_dump -- spec.json` 可离线查看任意 spec 的装配模型。
