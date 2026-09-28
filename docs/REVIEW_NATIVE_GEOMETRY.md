# 原生代码审核：几何 / 碰撞判定的效率与精度 (2026-09-28)

范围：`planning/native/agvnav.c`、`planning/native/slamcore.c`、`ros2/agv_nav2_plugins` (geom / localize)、`ros2/agv_ros_bridge` (safety / guidance)，
以及对应的 Python 实现 (`planning/maneuver.py`、`nav_runtime/navigator.py`)。

## 1. 主要问题：用固定步长采样做碰撞 / 扫掠判定

同一类写法出现在多处：把车体边界或车体运动按固定步长采样，只在采样点上判断。采样间隔与要保证的安全余量同量级甚至更大，而且每个采样点都重算三角函数。

| 位置 | 原做法 | 精度问题 (实测) | 改法 |
|---|---|---|---|
| `an_clearance` (C) / `_clearance_py` | 车体周边每 8 cm 取一点，求点到线段距离 | 净空最多高估 25.8 mm；20 万次随机扫掠里，在 0.03 / 0.05 m 阈值上分别有 89 / 85 次判定与精确值相反；线段斜穿车体 (端点都在车外) 时有 5 次被误判为可以转 | 车体系下"线段 vs 矩形"精确距离：不相交时 = min(两端点到矩形, 矩形四角到线段)；相交返回 0；端点在车内仍返回 -1 |
| 过弯 / 转向的位姿序列 (`an_plan_corner`、`_plan_corner_py`、`navigator._rotation_dir`、`guidance rotation_dir`) | 原地转 17 个位姿、圆弧 13 个、转向方向判断每 0.12 rad 一个 | 转 90° 时车角每步移动约 14 cm (0.12 rad 时约 17 cm)，两个位姿之间的车身完全没检查 | 位姿数按"车体上任一点相邻两位姿之间移动 ≤ 1 cm"取 (`SWEEP_STEP` / `an_sweep_samples`)，采样间漏检 ≤ 5 mm (距离的 Lipschitz 常数就是点的移动速度) |
| `geom.hpp rotationBlocked` (Nav2 插件：控制器、位姿调整) | 每 0.05 rad 采样，每点每步 sin/cos | 车角半径 1.4 m 时每步 7 cm，2 cm 外扩带会被跳过；车角扫掠圆 ±6 cm 内的 20 万个用例漏判 5692 次 (2.8%) | 点在车体系下的轨迹是圆弧：圆与矩形四边求交，交点落在弧段上即受阻 (`sweep.hpp arcHitsBox`)；0 漏判 |
| `geom.hpp translationBlocked` | 每 3 cm 采样 | 实测未漏判 (车体够厚，穿过外扩带后必进入车体) | 线段与矩形求交 (Liang–Barsky)，与原结果一致 |
| `safety.hpp rotation_blocked` / `navigator._rotation_blocked` (执行安全层) | 只看 1/3、2/3、全程 3 个角度 | 每步约 12 cm，外扩带和车角都可能漏 | 同 `arcHitsBox` (Python 为 numpy 向量化的 `planning/sweep.py`) |
| `guidance arc_blocked` / `navigator._arc_blocked` (圆弧过弯) | 每 0.15 rad 采样 | R = 0.9 m 时每步约 13 cm，外扩 3 cm | 车体沿圆弧 ⇔ 点绕瞬心 (0, ±R) 转：同一个 `arcHitsBox`，中心换成瞬心；与 1e-4 rad 采样对照 2 万例 0 漏判 0 多判 |

## 2. 效率问题

| 位置 | 问题 | 改法 | 效果 |
|---|---|---|---|
| `rotationBlocked` 等 | 每个激光点每个采样都算 sin/cos | 精确解 + 半径剪枝 (圆比矩形最远角还远的点直接跳过) | 1440 点：转 0.25 rad 35→6 µs，0.6 rad 79→6 µs，π 488→6 µs |
| `an_clearance` | O(位姿 × 72 × 线段) | 精确距离 (每对 4 次点到线段) + 逐线段剪枝 (控制点到线段距离 − 外接圆半径 ≥ 当前最优即跳过) + 按位置缓存 | 17 位姿：2.5~8.1 µs → 1.3~1.8 µs |
| 拐点惩罚缓存 (`corner_penalty`) | 固定 4096 项开放寻址，满了以后插入静默失败，之后每次查询扫整张表 | 负载过半扩容 ×2 并重建 | — |
| 路网按节点对查边号 (`attach`、`an_router_plan`) | 在邻接遍历里线性扫描全部边：O(m²) | 建图时给每条边记节点对编号，规划时 O(1) | 1200 条随机路线结果与原来逐点一致 |
| `sl_insert` (内置 SLAM 插帧) | 每帧 `calloc` 一整张地图大小的标记数组 | 线程内复用，只清本帧碰过的格子 | Mac 上约 5%；地图越大、内存带宽越小收益越大 |

`an_plan_corner` 整体：旧 22 µs / 拐点 → 精确距离 4.1 µs → 加上 1 cm 位姿加密后 51 µs (模式 2：8 → 15 µs)。
这部分不在控制回路里 (规划器按拐点缓存，所有场景 234 个拐点合计约 12 ms；导引每条路线只算几个拐点)，为换取有保证的 5 mm 误差界接受这个开销。
如需再提速：先粗采样，只在"下界 = 相邻两点较小值 − 步长/2"可能低于当前最小值的区间里加密 (误差界不变)，C 与 Python 需同步实现。

## 3. 对实际行为的影响

- 全部场景 468 个拐点过弯决策 (拐点 × 来去方向 × 两种模式)：通过/不通过**翻转 0 个**；8 个在 fms_workshop 环线拐点上新旧都不通过 (净空 4.6 mm / 0 mm)，只是"都不通过时报告哪个半径"从 0.9 变成 0.945。
- 本机自研导引 3 个贴墙代表点位：3/3 到达，终点误差 ≤ 10.7 mm，0 碰撞。
- 一致性测试：`tests/test_agvnav.py` (C vs Python 净空 / 过弯 / 拓扑规划 1000/1000/1200)、`tests/test_cpp_safety.py` (2000/2000)、`tests/test_slamcore.py`、`tests/test_sim_core.py` (22 项)、`ros2/agv_nav2_plugins/test/test_geom.cpp` 全部通过。
- **未在真实 ROS/Nav2 环境验证**：`geom.hpp` (Nav2 控制器与位姿调整插件) 与 `guidance.hpp` 的改动只在本机编译与单测验证过，需在装有 ROS 的设备上编译桥接和插件后跑一次 `tools/quick_nav_check.sh`。

## 4. 看过但暂不改的

- `localize.hpp icp`：每次迭代对每个点遍历全部线段，O(点 × 线段 × 迭代)；1440 点 × 20 线段实测 3.2 ms (手机约 13 ms)，每次停车只做一次，不是瓶颈。线段多时可按网格建索引。
- `geom.hpp remaining()`：每次调用都重新累加剩余路径长度 O(n)；路径段只有几百个点，可按需改为预计算累积长度。
- `slamcore.c` 格子下标用 `(long)` 截断而非向下取整：地图原点左下方 1 格内的点会被算进第 0 格。Python 参考实现同样截断 (两边一致)，只在点落在地图边界外时出现。
- `slamcore.c` 光线步进按 0.8 格采样：斜向光线可能漏掉角上的格子 (空闲更新不完整)，精确做法是 DDA / Bresenham；对建图结果影响小。
