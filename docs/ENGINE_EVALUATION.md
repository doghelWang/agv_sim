# 仿真引擎选型：MuJoCo vs PyBullet

**结论：选 MuJoCo。** 这次只在这两个引擎里二选一，不考虑自研引擎。

决定因素有两个：

1. **树莓派能直接装。** MuJoCo 官方提供 aarch64 wheel，PyBullet 没有。
2. **传感器仿真快得多。** 批量射线快约 7 倍，这是激光、ToF、双目、光电的主要开销。

MuJoCo 也有代价：URDF 导入不如 PyBullet 原生，但本项目本来就由 robot spec 直接生成 MJCF，所以不受影响。

## 1. 基础情况（2026-09 查询 PyPI）

| 项目 | MuJoCo | PyBullet |
|---|---|---|
| 最新版本 | 3.14.0（2026-09-22），基本每月一版 | 3.2.7（2025-01-30），之后没有新版本 |
| 树莓派 / aarch64 | 有官方 wheel（cp310–cp313，manylinux_2_28），`pip install mujoco` 即可 | 没有 aarch64 wheel，需要源码编译，树莓派上要 20 分钟以上 |
| 许可证 | Apache-2.0 | zlib |
| 维护方 | Google DeepMind | 社区（原作者已转向其它项目） |
| 模型格式 | MJCF 原生；可导入 URDF，但 fixed joint 会被合并，传感器需要另写 | URDF / SDF / MJCF 原生导入 |
| 接触求解 | 软接触 + implicitfast 积分，大时间步也稳定 | 顺序冲量 / LCP，大时间步容易抖动 |
| 批量射线 | `mj_multiRay`：C 实现，会释放 GIL，可多线程；返回距离、几何体 ID 和法向 | `rayTestBatch`：结果逐条转成 Python tuple，开销大 |
| 相机渲染 | `mujoco.Renderer`（EGL / OSMesa / GLFW）；也可以用射线自己着色 | TinyRenderer（纯 CPU）/ EGL 插件 |

## 2. 同场景实测（云端 x86，2 核，无 GPU）

测试场景：grid_9_square（20 段墙体）+ 20 个障碍物 + 车体，共约 47 个 geom。

| 项目 | MuJoCo 3.14.0 | PyBullet 3.2.7 | 倍数 |
|---|---|---|---|
| 物理单步（10 ms 步长） | 0.031 ms | 0.107 ms | 3.5× |
| 2D 激光 819 束 | 0.48 ms | 1.6–2.4 ms | 3–5× |
| Mid-360S 实机点数 20000 点 | 9.1 ms（1 线程 12.7 ms / 2 线程 8.1 ms） | 64–77 ms | 7–8× |
| ToF 224×172（38528 点） | 17.4 ms | 139 ms | 8× |
| RGB 640×480 | GL（EGL + llvmpipe 软件光栅）42–50 ms；射线着色 270–300 ms | TinyRenderer RGBD 126 ms | GL 快 2.5× |
| 深度图 640×480 | GL 36 ms | 含在上一行 | — |

树莓派 4 单核一般比这台机器慢 3–4 倍，可据此估算实机耗时。下表是**估算，未在实机测量**。

| 项目 | 树莓派估算耗时 |
|---|---|
| 20000 点 3D 激光 | 30–40 ms（4 线程时更低） |
| ToF | 60–70 ms |
| 320×240 射线相机 | 200–250 ms |

## 3. 集成方式（sim_core/mujoco_backend.py）

- **物理**
  - 车体建模为平面三自由度（slide x、slide y、hinge yaw），由速度执行器驱动，推力有上限。
  - 轮组运动学（先转后走、打滑）继续由 `ChassisKinematics` 计算车体期望速度；MuJoCo 负责积分和接触约束。
  - 撞到障碍物时车体会被真实挡住，编码器继续计数，里程计因此产生打滑漂移。
- **传感器几何**
  - 墙、货架、障碍物放在 geom group 0，射线只和这一组求交。
  - 地面和屋顶按解析平面计算。
  - 工位圆标、地面二维码、车道线预先烘焙成 2 cm/px 的地面纹理，着色时查表。
- **并发**
  - 射线使用独立的 `MjData`（静态几何与物理状态无关），不需要和物理步进互斥。
  - 相机单独开线程成像，不阻塞 100 Hz 物理和激光。
- **相机**
  - 默认用射线着色（`SIM_CAMERA_RENDER=ray`，不需要 GPU/GL）。
  - 有 EGL 时可设 `SIM_CAMERA_RENDER=gl` + `MUJOCO_GL=egl`，走 MuJoCo 光栅渲染，RGB 快 3–6 倍。
- **兜底**：没有安装 mujoco 时自动退回 kinematic 后端（numpy 线段求交），相机只输出深度灰度。

## 4. 部署

- `Dockerfile` 已加入 `pip install "mujoco>=3.3" "numpy<2" pillow`（清华 PyPI 镜像）。
- 旧镜像不需要重建：`start_sim.sh` 首次启动时检测到没有 mujoco 会自动安装，容器 restart 后安装结果保留。
- 限定 `numpy<2`，是为了保持与 ROS Humble 自带的 numpy 1.x ABI 兼容。已用 numpy 1.26 + mujoco 3.14 验证可以正常运行。
