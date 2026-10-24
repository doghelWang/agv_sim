# 四舵轮叉车 AMR (forklift_4steer)

`models/forklift_4steer.cmodel` 由 `tools/make_forklift_cmodel.py` 生成（ZIP + CompDesc.json，Rust/Python 解析器均可读）。

## 车型

| 部件 | 说明 |
|---|---|
| 底盘 | 1.7 × 0.9 m，自重 1800 kg，额载 1000 kg，空载 1.5 m/s，带载 0.9 m/s |
| 四舵轮 | 4 个 verticalSteerWheel（±120° 舵角，各自 walk + steer 电机）→ `multi_steer`：前进/横移(蟹行)/斜行/原地旋转 |
| 门架 + 货叉 | `lifter/forkMast`：行程 3.0 m，起升速度 0.3 m/s，加速度 0.4 m/s²，叉齿 1.1 m × 0.12 m × 0.06 m，叉间距 0.35 m（中心距） |
| 3D 激光 | Livox Mid-360S，护顶架上方 z=2.12 m |
| 安全激光 ×2 | MR-LS-07H 270°，对角布置 (前左 / 后右) z=0.32 m |
| 双目相机 | cam_left / cam_right，y=±0.12 m，z=1.95 m，俯角 20° |
| 读码相机 | 朝下 codeReader |
| 光电 ×2 | 叉尖 pallet_tip_l / pallet_tip_r |
| 其它 | 陀螺仪、主控、IO 模块、急停按钮、48 V/200 Ah 电池 |

cmodel 中 `main_type = lifter, sub_type = forkMast` 的模块会被解析为 `spec["fork"]`，同时自动生成带载保护外形 `spec["protection"]["payload"]`（货叉 + 托盘外形）。

## 仿真模型

### 底盘仿真
- `sim-core/src/kin.rs` 多舵轮运动学：每个舵轮独立转角/转速限制，逆运动学分配，里程计/滑移噪声。
- 带载时速度/加速度自动降到 `load_speed_factor`（默认 0.6）。

### 货叉与货物仿真 (`sim-core/src/fork.rs`)
- 门架：梯形速度曲线（目标高度 / 点动 / 限位），`di_lift_top` / `di_lift_bottom` / `di_cargo_present` / `di_pallet_detect`，`do_lift_motor_up|down` 点动。
- 货物 = `world.obstacles` 中 `cargo:true` 的条目（`z0` 离地高度），MuJoCo 几何/激光/相机/防撞条都能看到。
- 托盘（欧标 1200×800×144 mm）有真实的 3 根垫块与 2 个插孔：叉齿横向错位会被垫块挡住（`FORK_COLLISION`，底盘立即停住并回退到最近的无碰撞位姿）。
- 叉齿插入 ≥ 5/8 叉长后起升到托盘底面 → 自动叉起（含托盘上堆叠的货物）；落到地面 / 平台 / 其它货物顶面时自动放下并释放。
- 带载货物跟随车体位姿，叉齿/货物与墙、货架、其它货物的碰撞会阻止底盘移动。
- 事件：`FORK_PICK` / `FORK_PLACE` / `FORK_BLOCKED` / `FORK_COLLISION` / `CARGO`。

### REST（sim-server）
```
GET  /api/v1/fork                     货叉状态 (高度/速度/载货/受阻)
PUT  /api/v1/fork                     {height: m} | {action: up|down|stop|top|bottom}
GET  /api/v1/world/cargo              货物列表
POST /api/v1/world/cargo              {kind: pallet|box, id, x, y, yaw, z0?, size?: [长,宽,高]}
DELETE /api/v1/world/cargo/{id}
```
`/api/v1/state`、`/api/v1/snapshot`、`/api/v1/sim` 中都带 `fork`；网关遥测带 `cargo` / `fork`，2D/3D 视图直接显示货叉与货物。

## 任务流工步 (nav-runtime)

| type | 参数 | 行为 |
|---|---|---|
| `pick` | `cargo`(托盘 id) 或 `target`(附近工位)，`height`(起升高度，默认 0.25)，`approach`(预备距离，默认 2.0 m) | 选入口侧 → 导航到预备位 → 货叉降到底 → 对中(横移 + 偏航闭环) → 直线插入 → 起升叉起 → 倒车退出 |
| `place` | `x,y,yaw` 或 `target`，`z`(承载面高度，默认 0 地面；叠放在托盘上用 0.144) | 起升到行走高度 → 导航到预备位 → 对中 → 送入 → 下放释放 → 退出 → 货叉回 0.1 m |
| `fork` | `height` | 货叉直接升降 |

对中/插入阶段直接下发 cmd_vel（绕过防护区，相当于对接模式）；其余阶段保护外形始终覆盖货叉 + 托盘。

## 测试
```
tests/test_forklift.py        # 仿真层: 托盘/叠放、错位被挡、插入叉起、横移带载、放下、顶限位、碰撞
tests/test_forklift_flow.py   # 导航层: pick+place 地面搬运，再叉起叠放到另一托盘
```
启动：`sim-server` 载入 `models/forklift_4steer.cmodel`（`POST /api/v1/model/reload {"cmodel_path": ...}`），`nav-runtime` 指向它。
