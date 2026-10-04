#!/usr/bin/env python3
"""
Web 调度台 ↔ Nav2 桥接

* NavigateToPose action 客户端: 下发目标 / 反馈剩余距离 / 结果回写导航状态 / 取消
* 拓扑路线 (默认，需 ros2/agv_nav2_plugins): 路线发到 /agv/route，NavigateToPose 用 nav2/agv_route_bt.xml 行为树 ——
  AgvRoute 规划 (圆弧过弯/拐点转向) → RouteFollow 跟随 (停车精度) → 受阻时 adjust_pose 恢复，全部在 Nav2 内完成
* FollowPath (NAV2_ROUTE_MODE=follow_path，或插件不可用时): 拓扑路线按拐点切成直线段，逐段交给 controller_server (RotationShim + RPP) 精确跟随
* NavigateThroughPoses: 沿拓扑路线逐点通过 (NAV2_ROUTE_MODE=through_poses 时使用)，剩余位姿数 → 当前路段
* /map_server/load_map: 场景切换时热切换 Nav2 全局地图
* Nav2Supervisor: 按当前车型自动启动/重启 nav2_launch.py (车型切换 → 换一套 cmodel 生成的参数)
"""

import math
import os
import shutil
import signal
import subprocess
import threading
import time

from common import spawn

import importlib.util

# nav2_msgs / rclpy 按需导入: 核心模式 (C++ 桥接承担全部 ROS 通信) 下执行进程不加载 rclpy；
# 只有回退模式 (FollowPath 分段跟线 / NavigateToPose 默认行为树 / BackUp) 用到时才导入
NAV2_MSGS = importlib.util.find_spec("nav2_msgs") is not None and importlib.util.find_spec("rclpy") is not None
ActionClient = NavigateToPose = NavigateThroughPoses = FollowPath = LoadMap = GoalStatus = None


def _lazy():
    global ActionClient, NavigateToPose, NavigateThroughPoses, FollowPath, LoadMap, GoalStatus
    if NavigateToPose is not None:
        return
    from rclpy.action import ActionClient as _AC
    from nav2_msgs.action import NavigateToPose as _N
    from nav2_msgs.srv import LoadMap as _L
    from action_msgs.msg import GoalStatus as _G
    ActionClient, NavigateToPose, LoadMap, GoalStatus = _AC, _N, _L, _G
    try:
        from nav2_msgs.action import NavigateThroughPoses as _T
        NavigateThroughPoses = _T
    except Exception:  # pragma: no cover
        pass
    try:
        from nav2_msgs.action import FollowPath as _F
        FollowPath = _F
    except Exception:  # pragma: no cover
        pass

HERE = os.path.dirname(os.path.abspath(__file__))


def nav2_installed() -> bool:
    if not NAV2_MSGS or shutil.which("ros2") is None:
        return False
    try:
        r = subprocess.run(["ros2", "pkg", "prefix", "nav2_bt_navigator"], capture_output=True, timeout=10)
        return r.returncode == 0
    except Exception:
        return False


class Nav2Supervisor:
    def __init__(self, logger):
        self.log = logger
        self.proc = None
        self.chassis = None
        self.lock = threading.Lock()
        self.localization = os.environ.get("SIM_LOCALIZATION", "slam")
        self.map_source = "map_server"          # topic: 用 slam_toolbox 的 /map (main 按定位引擎设置)
        self.use_sim_time = os.environ.get("SIM_USE_SIM_TIME", "0") == "1"
        self.enabled = os.environ.get("NAV2_AUTOSTART", "1") == "1" and nav2_installed()

    def running(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def start(self, chassis: str, scenario: str, origin=(0.0, 0.0, 0.0)):
        if not self.enabled:
            return False
        chassis = chassis if chassis in ("diff_drive", "single_steer", "dual_steer") else "dual_steer"
        with self.lock:
            self._stop_locked()
            cmd = ["ros2", "launch", os.path.join(HERE, "nav2", "nav2_launch.py"),
                   f"chassis:={chassis}", f"map:={os.path.join(HERE, 'maps', scenario + '.yaml')}",
                   f"localization:={self.localization}", f"use_sim_time:={'true' if self.use_sim_time else 'false'}",
                   f"map_source:={self.map_source}", f"initial_x:={origin[0]}", f"initial_y:={origin[1]}", f"initial_yaw:={origin[2]}"]
            self.log.info("启动 Nav2: " + " ".join(cmd))
            self.proc = spawn.popen("nav2", cmd)
            self.started_at, self.last_args = time.time(), (chassis, scenario, origin)
            self.chassis = chassis
        return True

    def _stop_locked(self):
        if self.proc is not None and self.proc.poll() is None:
            try:
                spawn.killpg(self.proc, signal.SIGINT)
                self.proc.wait(timeout=8)
            except Exception:
                try:
                    spawn.killpg(self.proc, signal.SIGKILL)
                except Exception:
                    pass
        self.proc = None

    def stop(self):
        with self.lock:
            self._stop_locked()


class Nav2Bridge:
    RESULT_TEXT = {4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}

    def __init__(self, node):
        self.node = node
        self.available = NAV2_MSGS
        self.goal_handle = None
        self.mission_id = None
        self.on_result = None
        self.on_feedback = None
        self.supervisor = Nav2Supervisor(node.get_logger())
        self._clients = {}
        self._clients_lock = threading.Lock()
        self._route_io = None                 # Python 路线接口 (无 C++ 桥接时才创建)
        self._route_via_cpp = False
        self.on_stop_distance = None          # RouteController 发布的到下一停车点剩余行程 (m)
        self.on_plugin_event = None           # agv_nav2_plugins 的导航事件 (dict)
        self.on_plan = None                   # planner_server 发布的规划路径 (/plan) → [{"x","y"}, ...] (约 0.2 m 一个点)
        self.cppmode = getattr(node, "rclpy_free", False)     # CppRos: 执行进程不加载 rclpy
        self._BackUp = None
        self._fp_handle = None
        self._active = False
        self._active_cli = None
        if self.available and self.cppmode:
            threading.Thread(target=self._watchdog, daemon=True, name="nav2-watchdog").start()
        elif self.available:
            _lazy()
            # Action 客户端按需创建: 客户端一旦存在就会订阅该动作的反馈话题 —— 即使目标不是本进程下发的
            # (如 bt_navigator 每 10 ms 一条 NavigateToPose 反馈)，rclpy 也要逐条反序列化，执行进程 CPU 翻倍
            self.load_map_cli = node.create_client(LoadMap, "/map_server/load_map")
            try:
                from std_srvs.srv import Trigger
                self._active_cli = node.create_client(Trigger, "/lifecycle_manager_navigation/is_active")
                self._Trigger = Trigger
                threading.Thread(target=self._watchdog, daemon=True, name="nav2-watchdog").start()
            except Exception as e:  # noqa
                self._active_cli = None
                node.get_logger().warn(f"Nav2 看门狗不可用: {e}")

    # ------------------------------------------------------------------
    @property
    def rnode(self):
        """回退模式用的 rclpy 节点 (核心模式下按需创建)"""
        return self.node.rclpy_node() if self.cppmode else self.node

    def _action(self, key, typ, name):
        if typ is None:
            return None
        with self._clients_lock:
            c = self._clients.get(key)
            if c is None:
                c = ActionClient(self.rnode, typ, name)
                self._clients[key] = c
            return c

    @property
    def client(self):
        _lazy()
        return self._action("navigate_to_pose", NavigateToPose, "navigate_to_pose")

    @property
    def through_client(self):
        _lazy()
        return self._action("navigate_through_poses", NavigateThroughPoses, "navigate_through_poses")

    @property
    def follow_client(self):
        _lazy()
        return self._action("follow_path", FollowPath, "follow_path")

    @property
    def backup_client(self):
        if self._BackUp is None:
            try:   # Nav2 behavior_server 的 BackUp 行为 (转向空间不足时后退一点再转)
                from nav2_msgs.action import BackUp
                self._BackUp = BackUp
            except Exception:
                return None
        return self._action("backup", self._BackUp, "backup")

    def _cpp(self):
        c = getattr(self.node, "cpp", None)
        return c if c is not None and c.running() else None

    # ------------------------------------------------------------------
    def _is_active(self) -> bool:
        if self.cppmode:                      # C++ 桥接每 2 s 查询一次 lifecycle_manager
            # Nav2 刚 (重新) 启动的头 10 s 不采信: 桥接上报的还是上一套 Nav2 的"已激活" (查询 2 s 一次，无应答要 6 s 才放弃)。
            # 曾因此把卡在 "Configuring controller_server" 的新 Nav2 当成已就绪，启动看门狗不再重启它，任务永远等下去
            if time.time() - getattr(self.supervisor, "started_at", 0.0) < 10.0:
                return False
            return bool(getattr(self.node, "nav2_active", False))
        if self._active_cli is None or not self._active_cli.service_is_ready():
            return False
        fut = self._active_cli.call_async(self._Trigger.Request())
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 5.0:
            time.sleep(0.05)
        return bool(fut.done() and fut.result() is not None and fut.result().success)

    def _watchdog(self):
        """Nav2 生命周期看门狗: 启动后 NAV2_BRINGUP_TIMEOUT 秒内没有全部激活 (lifecycle_manager 放弃，
        例如负载高时某个节点 get_state 响应超时 → "Aborting bringup")，重启 Nav2，最多 NAV2_BRINGUP_RETRIES 次"""
        # Android: 正常 30~60 s 就绪；lifecycle_manager 等某个节点的 configure/activate 应答没有超时，请求丢了就永远卡住，90 s 没好就重来
        timeout = float(os.environ.get("NAV2_BRINGUP_TIMEOUT", "90" if os.path.exists("/system/build.prop") else "150"))
        max_retry = int(os.environ.get("NAV2_BRINGUP_RETRIES", "3"))
        retries, seen, was_active = 0, None, False
        while True:
            time.sleep(5.0)
            sup = self.supervisor
            try:
                if not sup.running():
                    self._active = False
                    continue
                if seen != getattr(sup, "started_at", None):     # 新一次启动
                    seen, was_active = sup.started_at, False
                if was_active:
                    # 本次启动已全部激活过: 保持就绪 (负载高时 is_active 服务偶尔超时，不能据此判定掉线或重启；
                    # 进程退出/重启时由上面的分支复位)
                    self._active = True
                    continue
                ok = self._is_active()
                self._active = ok
                if ok:
                    retries, was_active = 0, True
                    continue
                if was_active:
                    continue
                if time.time() - sup.started_at > timeout and retries < max_retry and getattr(sup, "last_args", None):
                    retries += 1
                    self.node.get_logger().warn(f"Nav2 {timeout:.0f} s 内未全部激活，重启 Nav2 ({retries}/{max_retry})")
                    sup.start(*sup.last_args)
            except Exception as e:  # noqa
                self.node.get_logger().warn(f"Nav2 看门狗: {e}")

    # ------------------------------------------------------------------
    def ready(self) -> bool:
        # 以 lifecycle_manager 报告"全部激活"为准 (看门狗每 5 s 刷新)；不为查询就绪去创建 Action 客户端
        if not self.available:
            return False
        if self.cppmode or getattr(self, "_active_cli", None) is not None:
            return self._active
        return self.client.server_is_ready()

    def restart_stack(self) -> bool:
        """Nav2 内部卡死 (控制器不出指令、连取消都不应答) 时整套重启；就绪状态清零，由看门狗重新确认激活"""
        sup = self.supervisor
        args = getattr(sup, "last_args", None)
        if not args or not sup.enabled:
            return False
        self._route_active = False
        self.goal_handle = None
        self._active = False
        try:
            self.node.nav2_active = False      # 桥接缓存的旧状态
        except Exception:
            pass
        return bool(sup.start(*args))

    def status(self) -> dict:
        return {"msgs": self.available, "server_ready": self.ready(), "process": self.supervisor.running(),
                "active": getattr(self, "_active", None), "autostart": self.supervisor.enabled, "chassis": self.supervisor.chassis,
                "localization": self.supervisor.localization}

    def send_goal(self, x, y, yaw, mission_id, on_result, on_feedback=None) -> str:
        if not self.available:
            return "nav2_msgs 未安装"
        if not self.client.wait_for_server(timeout_sec=2.0):
            return "Nav2 navigate_to_pose 服务未就绪 (Nav2 未启动或仍在激活中)"
        self.mission_id = mission_id
        self.on_result = on_result
        self.on_feedback = on_feedback
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = "map"
        g.pose.header.stamp = self.rnode.get_clock().now().to_msg()
        g.pose.pose.position.x = float(x)
        g.pose.pose.position.y = float(y)
        g.pose.pose.orientation.z = math.sin(yaw / 2.0)
        g.pose.pose.orientation.w = math.cos(yaw / 2.0)
        fut = self.client.send_goal_async(g, feedback_callback=self._feedback)
        fut.add_done_callback(lambda f, mid=mission_id: self._accepted(f, mid))
        return ""

    def _pose(self, x, y, yaw):
        from geometry_msgs.msg import PoseStamped
        p = PoseStamped()
        p.header.frame_id = "map"
        p.header.stamp = self.rnode.get_clock().now().to_msg()
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.orientation.z = math.sin(yaw / 2.0)
        p.pose.orientation.w = math.cos(yaw / 2.0)
        return p

    def send_through_poses(self, poses, mission_id, on_result, on_feedback=None) -> str:
        """poses: [(x, y, yaw), ...] 沿拓扑路线的中间点 + 终点。返回错误文本 ('' 表示已下发)"""
        if not self.available or self.through_client is None:
            return "NavigateThroughPoses 不可用"
        if not self.through_client.wait_for_server(timeout_sec=2.0):
            return "Nav2 navigate_through_poses 服务未就绪"
        self.mission_id = mission_id
        self.on_result = on_result
        self.on_feedback = on_feedback
        g = NavigateThroughPoses.Goal()
        g.poses = [self._pose(x, y, yaw) for x, y, yaw in poses]
        fut = self.through_client.send_goal_async(g, feedback_callback=self._feedback)
        fut.add_done_callback(lambda f, mid=mission_id: self._accepted(f, mid))
        return ""

    def agv_ready(self) -> bool:
        """拓扑路线模式可用: 插件已安装且本次 Nav2 参数里生成了 RouteFollow (非全向车型)，Nav2 已激活"""
        if not self.available or self.supervisor.chassis == "dual_steer":
            return False
        try:
            from tools.gen_nav2_params import agv_plugins_available
            if not agv_plugins_available():
                return False
        except Exception:
            return False
        return self.ready()

    def send_route(self, route, x, y, yaw, mission_id, on_result, on_feedback=None, segs=None) -> str:
        """拓扑路线导航: route = [(x, y), ...] (含起点与终点)，segs = 场景静态线段 (末段精定位)。
        有 C++ 桥接时整个交给 agv_ros_bridge (发布路线/线段、NavigateToPose、限频回馈)；否则由本进程 rclpy 完成"""
        # 上一个目标还在执行就来了新目标: 行为树只规划一次，Nav2 "抢占" 后仍沿旧路径走到旧终点，再把新目标报成"成功"
        # (车停在旧目标处，任务却显示到达)。所以先取消旧目标，等取消落地 (0.8 s) 再发新的；
        # 取消与下发只隔几毫秒 (任务流重新开始) 时，新目标会被接受但控制器不出指令、任务一直停在"导航中"，同样靠这段等待避开。
        if getattr(self, "_route_active", False):
            self.cancel(quiet=True)
        gap = 0.8 - (time.time() - getattr(self, "_cancel_t", 0.0))
        if gap > 0:
            time.sleep(gap)
        self._route_active = True
        self.mission_id = mission_id
        self.on_result = on_result
        self.on_feedback = on_feedback
        bt = os.path.join(HERE, "nav2", "agv_route_bt.xml")
        cpp = self._cpp()
        if cpp is not None:
            cpp.on_nav = self._on_cpp_nav
            self._route_via_cpp = True
            # 每次下发带一个新编号 (任务号 × 1000 + 序号): 同一任务重新下发后，旧目标迟到的结果/回馈按编号丢弃
            self._seq = getattr(self, "_seq", 0) + 1
            self._token = int(mission_id) * 1000 + self._seq % 1000
            cpp.send_route(self._token, route, (x, y, yaw), bt, segs)
            return ""
        self._route_via_cpp = False
        io = self._py_route_io()
        if io is None:
            return "/agv/route 接口不可用"
        if segs:
            m = io["Float32MultiArray"]()
            m.data = [float(v) for sg in segs for v in sg[:4]]
            io["segs_pub"].publish(m)
        if not self.client.wait_for_server(timeout_sec=2.0):
            return "Nav2 navigate_to_pose 服务未就绪"
        from nav_msgs.msg import Path
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.rnode.get_clock().now().to_msg()
        path.poses = [self._pose(px, py, 0.0) for px, py in route]
        io["route_pub"].publish(path)
        g = NavigateToPose.Goal()
        g.pose = self._pose(x, y, yaw)
        g.behavior_tree = bt
        fut = self.client.send_goal_async(g, feedback_callback=self._feedback)
        fut.add_done_callback(lambda f, mid=mission_id: self._accepted(f, mid))
        return ""

    def _py_route_io(self):
        """无 C++ 桥接时的路线接口 (按需创建): /agv/route、/agv/world_segments 发布，停车距离/插件事件/规划路径订阅"""
        if self._route_io is not None:
            return self._route_io
        try:
            from nav_msgs.msg import Path
            from std_msgs.msg import Float32, Float32MultiArray, String
            from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
            n = self.rnode
            self._route_io = {"route_pub": n.create_publisher(Path, "/agv/route", latched),
                              "segs_pub": n.create_publisher(Float32MultiArray, "/agv/world_segments", latched),
                              "Float32MultiArray": Float32MultiArray,
                              "subs": [n.create_subscription(Float32, "/agv/stop_distance", self._on_stop_distance, 10),
                                       n.create_subscription(String, "/agv/nav_event", self._on_plugin_event, 20),
                                       n.create_subscription(Path, "/plan", self._on_plan, 2)]}
        except Exception as e:  # noqa
            self.node.get_logger().warn(f"/agv/route 接口不可用: {e}")
        return self._route_io

    def _on_cpp_nav(self, m: dict):
        k = m.get("k")
        if k == "stop":
            cb = self.on_stop_distance
            if cb:
                cb(float(m.get("d", 0.0)))
        elif k == "fb":
            if self.on_feedback and m.get("mid") == getattr(self, "_token", None):
                self.on_feedback(self.mission_id, float(m.get("dist", 0.0)), float(m.get("t", 0)), int(m.get("rec", 0)))
        elif k == "result":
            if m.get("mid") == getattr(self, "_token", None):
                self._result(self.mission_id, m.get("result", "ABORTED"))
        elif k == "event":
            cb = self.on_plugin_event
            if cb and isinstance(m.get("e"), dict):
                cb(m["e"])
        elif k == "plan":
            cb = self.on_plan
            if cb:
                cb([{"x": p[0], "y": p[1]} for p in m.get("curve", [])])

    def _on_stop_distance(self, msg):
        cb = self.on_stop_distance
        if cb:
            cb(float(msg.data))

    def _on_plan(self, msg):
        cb = self.on_plan
        if not cb or not msg.poses:
            return
        out, last = [], None
        for ps in msg.poses:
            p = ps.pose.position
            if last is None or math.hypot(p.x - last[0], p.y - last[1]) >= 0.2:
                out.append({"x": round(p.x, 3), "y": round(p.y, 3)})
                last = (p.x, p.y)
        p = msg.poses[-1].pose.position
        out.append({"x": round(p.x, 3), "y": round(p.y, 3)})
        cb(out)

    def _on_plugin_event(self, msg):
        cb = self.on_plugin_event
        if cb:
            import json
            try:
                cb(json.loads(msg.data))
            except Exception:
                pass

    def follow_ready(self) -> bool:
        return (self.available and self.follow_client is not None and self.follow_client.server_is_ready()
                and (self._active if (self.cppmode or getattr(self, "_active_cli", None) is not None) else True))

    def follow_path(self, poses, cancelled, on_feedback=None, goal_checker="precise_goal_checker", timeout=600.0) -> str:
        """线路跟随 (阻塞): 把一段稠密路径交给 Nav2 controller_server (FollowPath)。
        poses: [(x, y, yaw), ...] map 坐标；cancelled(): 返回 True 时取消。返回 SUCCEEDED / ABORTED / CANCELED / 错误文本"""
        if not self.available or self.follow_client is None:
            return "FollowPath 不可用"
        if not self.follow_client.wait_for_server(timeout_sec=2.0):
            return "Nav2 follow_path 服务未就绪"
        from nav_msgs.msg import Path
        g = FollowPath.Goal()
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.rnode.get_clock().now().to_msg()
        path.poses = [self._pose(x, y, yaw) for x, y, yaw in poses]
        g.path = path
        g.controller_id = "FollowPath"
        g.goal_checker_id = goal_checker

        def fb(msg):
            if on_feedback:
                try:
                    on_feedback(float(msg.feedback.distance_to_goal), float(msg.feedback.speed))
                except Exception:
                    pass
        fut = self.follow_client.send_goal_async(g, feedback_callback=fb)
        t_end = time.time() + 10.0
        while not fut.done():
            if time.time() > t_end:
                return "FollowPath 目标未被接受 (超时)"
            time.sleep(0.02)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return "FollowPath 目标被拒绝"
        self._fp_handle = gh
        rf = gh.get_result_async()
        t_end = time.time() + timeout
        while not rf.done():
            if cancelled() or time.time() > t_end:
                try:
                    gh.cancel_goal_async()
                except Exception:
                    pass
                self._fp_handle = None
                return "CANCELED" if cancelled() else "TIMEOUT"
            time.sleep(0.02)
        self._fp_handle = None
        try:
            st = rf.result().status
        except Exception:
            st = 6
        return self.RESULT_TEXT.get(st, f"STATUS_{st}")

    def backup(self, dist: float, speed: float, cancelled, timeout: float = 20.0) -> str:
        """Nav2 BackUp 行为 (阻塞)：沿车头反方向后退 dist 米 (behavior_server 按局部代价地图做后方碰撞检查)"""
        if not self.available or self.backup_client is None:
            return "BackUp 不可用"
        if not self.backup_client.wait_for_server(timeout_sec=2.0):
            return "Nav2 backup 服务未就绪"
        from builtin_interfaces.msg import Duration
        g = self._BackUp.Goal()
        g.target.x = float(abs(dist))
        g.speed = float(abs(speed))
        g.time_allowance = Duration(sec=int(timeout))
        fut = self.backup_client.send_goal_async(g)
        t_end = time.time() + 5.0
        while not fut.done():
            if time.time() > t_end:
                return "BackUp 目标未被接受 (超时)"
            time.sleep(0.02)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return "BackUp 目标被拒绝"
        rf = gh.get_result_async()
        t_end = time.time() + timeout + 2.0
        while not rf.done():
            if cancelled() or time.time() > t_end:
                try:
                    gh.cancel_goal_async()
                except Exception:
                    pass
                return "CANCELED" if cancelled() else "TIMEOUT"
            time.sleep(0.02)
        try:
            st = rf.result().status
        except Exception:
            st = 6
        return self.RESULT_TEXT.get(st, f"STATUS_{st}")

    def _feedback(self, msg):
        if self.on_feedback:
            fb = msg.feedback
            rem = getattr(fb, "number_of_poses_remaining", None)
            try:
                self.on_feedback(self.mission_id, float(fb.distance_remaining), float(fb.navigation_time.sec),
                                 int(fb.number_of_recoveries), None if rem is None else int(rem))
            except TypeError:
                self.on_feedback(self.mission_id, float(fb.distance_remaining), float(fb.navigation_time.sec),
                                 int(fb.number_of_recoveries))

    def _accepted(self, fut, mid):
        gh = fut.result()
        if gh is None or not gh.accepted:
            if self.on_result:
                self.on_result(mid, "REJECTED")
            return
        self.goal_handle = gh
        gh.get_result_async().add_done_callback(lambda f, m=mid: self._done(f, m))

    def _done(self, fut, mid):
        try:
            st = fut.result().status
        except Exception:
            st = 6
        self._result(mid, self.RESULT_TEXT.get(st, f"STATUS_{st}"))

    def _result(self, mid, result):
        """目标结束。被新目标顶掉 (cancel(quiet=True)) 的旧目标的 CANCELED 不上报: 同一任务重新下发时它会把任务标成已取消"""
        q = getattr(self, "_quiet_cancel", None)
        if q and result == "CANCELED" and mid == q[0] and time.time() - q[1] < 5.0:
            self._quiet_cancel = None
            return
        if mid == self.mission_id:
            self._route_active = False
        if self.on_result:
            self.on_result(mid, result)

    def cancel(self, quiet=False):
        self._cancel_t = time.time()
        if quiet and getattr(self, "_route_active", False) and not getattr(self, "_route_via_cpp", False):
            self._quiet_cancel = (self.mission_id, self._cancel_t)
        self._route_active = False
        if quiet:
            self._token = None                 # C++ 桥接: 被顶掉的旧目标的结果/回馈不再上报
        cpp = self._cpp()
        if cpp is not None and self._route_via_cpp:
            cpp.send_cancel()
        fp = getattr(self, "_fp_handle", None)
        self._fp_handle = None
        if fp is not None:
            try:
                fp.cancel_goal_async()
            except Exception:
                pass
        gh = self.goal_handle
        self.goal_handle = None
        if gh is not None:
            try:
                gh.cancel_goal_async()
            except Exception:
                pass

    def load_map(self, scenario: str) -> bool:
        if self.cppmode:
            return self.node.roscall("load_map", url=os.path.join(HERE, "maps", scenario + ".yaml"), wait=False)
        if not self.available or not self.load_map_cli.service_is_ready():
            return False
        req = LoadMap.Request()
        req.map_url = os.path.join(HERE, "maps", scenario + ".yaml")
        self.load_map_cli.call_async(req)
        return True
