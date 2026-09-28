# 交接: 导航遗留问题排查 (接 HANDOFF_C_REFACTOR 之后)

## 0. 代码与环境
- 仓库 https://github.com/doghelWang/agv_sim ，main = `4ab9ebd`。本机克隆在 `~/agv_sim` (与 GitHub 同步)。
- 先读: `docs/NAVIGATION.md` (两种规划器、Nav2 中断根因、恢复流程)、`docs/PERFORMANCE.md` 第 5~6 节 (C/C++ 化结果)。
- 树莓派 5: `ssh rpi117` (192.168.11.117，免密 sudo)，项目 `~/ros2_cmodel_agv`，平台 :8082，基准实例 **i12** (仿真 8100 / 网关 8101 / 执行 8102，Docker 容器 agv-sim-i12 / agv-nav-i12)。
- **树莓派访问不了 GitHub**：它的 git 代理指向手机 (192.168.11.140:7890)，手机已带走。同步代码用 bundle：
  `git bundle create /tmp/x.bundle <树莓派当前提交>..main` → `scp` 到树莓派 → `git fetch /tmp/x.bundle main:refs/remotes/origin/main && git merge --ff-only origin/main`
  (或者在树莓派上 `git -c http.proxy= -c https.proxy= fetch`，但之前试过直连也不通)。
- 手机 (Pixel 4) 已被带走，暂时不可用。回来后: `ssh pixel4` (端口 8022)，单手机模式 (本机平台 :8082，实例 i04，端口同上)，更新用 `bash ~/update_from_git.sh`，测量脚本在 `~/agvbench/phone_bench.sh <标签>`。

## 1. 约束 (用户明确要求)
- MuJoCo、Nav2、slam_toolbox、ROS 2 是基础引擎，**不要自研替代品**。可以把包在引擎外的 Python 壳改成 C/C++。
- 验证要快：用 `tools/quick_nav_check.sh` (3 个镜像代表点位，约 2.5 分钟)，不要遍历 13 个工位；性能对比才用 `tools/ab_bench_i12.sh`。
- 工作台有"控制权锁"，别人持锁时测试工具下发目标会得到 HTTP 423。**不要强制抢锁**，请用户释放。

## 2. 常用操作 (树莓派)
- 改了执行侧代码: `bash tools/refresh_images.sh nav` (约 30 s)；改了仿真侧: `bash tools/refresh_images.sh sim`。超过 100 层会自动压平。
- 快速回归: `bash tools/quick_nav_check.sh --restart` (换镜像后要 --restart)；输出到达情况、导航事件 (NAV2_RETRY / NAV2_ALIGN / NAV2_GIVEUP / LOC_STALE)、controller_server 错误统计。
- 本机 (无 ROS) 回归: `python3 tests/test_sim_core.py` (22 项)；三进程端到端见 tests/test_rest_e2e.py 顶部说明 (需先起 sim_server / nav_runtime(NAV_USE_ROS=0) / web_gateway)。
- 在树莓派上 `pkill -f` 时，模式要写成 `"[p]recision_test.py"` 这种形式，否则会连同 ssh 会话自己一起杀掉。

## 3. 待排查问题 (按优先级)
1. **手机上 Nav2 任务从中心 P0 出发即被中止，没有进入恢复流程**
   - 现象 (手机 d1a5299)：NAV2_ABORTED "终点 (0.00,-0.01) 目标误差 0.6 cm / 29.2°" 与 "/ 54.7°"，事件里没有 NAV2_RETRY，也没有 NAV2_GIVEUP。
   - 怀疑方向：`nav_runtime/navigator.py` `_nav2_follow_loop` 里 follow_path 返回的不是 "ABORTED" (例如 "FollowPath 目标被拒绝"、"服务未就绪"、"TIMEOUT")，或者走了 `_nav2_follow_loop` 以外的失败分支。结果文本应在 `nav2_bridge.py follow_path` 的返回值里。
   - 手机不在时，可以在树莓派上尝试复现：从 P0 下发需要原地大角度转向的目标，看 on_result 的调用路径。
2. **手机 proot 下 slam_toolbox 的 map→odom 停更 (实测 11 s)**
   - 证据：手机实例日志 `/root/.agv-agent/logs/agv-nav-i04.log` (proot 里的路径) 出现 `Transform data too old when converting from map to odom … Transform time` 比 Data time 早 11 s。
   - 已加保护：Nav2 执行中定位超过 1 s 未更新就停车 (`LOC_STALE_S`)，事件 LOC_STALE / LOC_RESUME。
   - 可尝试的方向：降低 slam_toolbox 负载 (激光点数、`minimum_travel_*`、`map_update_interval`、`transform_publish_period`)。手机跑执行仍不推荐。
3. **贴墙工位终点误差偏大 (26~51 mm，场地中部 ≤ 20 mm)**：可能与贴墙时 slam 定位误差 (30~70 mm) 以及执行进程接手对位转向有关，未定位。
4. controller_server 大量 "Control loop missed its desired rate of 30Hz" (树莓派)：还没评估影响；Android 上用的是 20 Hz。
5. 场景层面的根治建议 (需用户决定)：grid_9_square 里离外墙 1.5 m 的拓扑节点/工位 (±7.5) 往场内移约 0.2 m，可以消除贴墙原地转向的问题。

## 4. 已完成的关键改动 (供理解代码)
- 仿真: `sim_core/native/simcore.c` + `simcore_rt.c` (C 实时循环，调用 MuJoCo C API)、`sim_core/rt.py`、推送流 `/api/v1/stream`、UDP 速度指令。开关: SIM_RT / SIM_NATIVE / NAV_SIM_STREAM / SIM_CMD_UDP。
- 执行: `ros2/agv_ros_bridge` (C++，抽象命名空间套接字) + `nav_runtime/cpp_bridge.py`，开关 NAV_ROS_BRIDGE=auto|cpp|py。
- 导航 (`nav_runtime/navigator.py`):
  - 中断恢复：有无进展判定 → BackUp 0.2 / 0.35 m → 从当前位置重新规划 → 连续 3 次无进展放弃；
  - Nav2 拒绝原地转向时，由执行进程 `_rotate_to` 接手；
  - 防护区按剩余行程缩短，同时速度封顶 √(2·a·剩余)；
  - 定位停更保护。
- Nav2 参数 (`tools/gen_nav2_params.py`、`nav2/loc_launch.py`)：RPP transform_tolerance 0.3、slam transform_timeout 0.5 (Android 0.8)、局部代价地图 footprint_padding = rotate_margin (0.02)。

## 5. 完成标准
每个问题：先找到有证据的根因 → 修改 → 本机单测 + 树莓派 quick_nav_check 通过 → 结论写进 docs/NAVIGATION.md → 推送 GitHub (本机推，树莓派用 bundle 同步)。
