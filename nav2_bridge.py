#!/usr/bin/env python3
"""
Web 调度台 ↔ Nav2 桥接

* NavigateToPose action 客户端: 下发目标 / 反馈剩余距离 / 结果回写导航状态 / 取消
* FollowPath (默认): 拓扑路线按拐点切成直线段，逐段交给 controller_server (RotationShim + RPP) 精确跟随
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

try:
    from rclpy.action import ActionClient
    from nav2_msgs.action import NavigateToPose
    try:
        from nav2_msgs.action import NavigateThroughPoses
    except Exception:  # pragma: no cover
        NavigateThroughPoses = None
    try:
        from nav2_msgs.action import FollowPath
    except Exception:  # pragma: no cover
        FollowPath = None
    from nav2_msgs.srv import LoadMap
    from action_msgs.msg import GoalStatus
    NAV2_MSGS = True
except Exception:  # pragma: no cover
    NAV2_MSGS = False

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
        if self.available:
            self.client = ActionClient(node, NavigateToPose, "navigate_to_pose")
            self.through_client = (ActionClient(node, NavigateThroughPoses, "navigate_through_poses")
                                   if NavigateThroughPoses is not None else None)
            self.load_map_cli = node.create_client(LoadMap, "/map_server/load_map")
            self.follow_client = ActionClient(node, FollowPath, "follow_path") if FollowPath is not None else None
            self._fp_handle = None
            self._active = False
            try:
                from std_srvs.srv import Trigger
                self._active_cli = node.create_client(Trigger, "/lifecycle_manager_navigation/is_active")
                self._Trigger = Trigger
                threading.Thread(target=self._watchdog, daemon=True, name="nav2-watchdog").start()
            except Exception as e:  # noqa
                self._active_cli = None
                node.get_logger().warn(f"Nav2 看门狗不可用: {e}")

    # ------------------------------------------------------------------
    def _is_active(self) -> bool:
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
        timeout = float(os.environ.get("NAV2_BRINGUP_TIMEOUT", "150"))
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
        # 动作服务器在 configure 时就可见；以 lifecycle_manager 报告"全部激活"为准 (看门狗每 5 s 刷新)
        if not (self.available and self.client.server_is_ready()):
            return False
        return self._active if getattr(self, "_active_cli", None) is not None else True

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
        g.pose.header.stamp = self.node.get_clock().now().to_msg()
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
        p.header.stamp = self.node.get_clock().now().to_msg()
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

    def follow_ready(self) -> bool:
        return (self.available and self.follow_client is not None and self.follow_client.server_is_ready()
                and (self._active if getattr(self, "_active_cli", None) is not None else True))

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
        path.header.stamp = self.node.get_clock().now().to_msg()
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
        if self.on_result:
            self.on_result(mid, self.RESULT_TEXT.get(st, f"STATUS_{st}"))

    def cancel(self):
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
        if not self.available or not self.load_map_cli.service_is_ready():
            return False
        req = LoadMap.Request()
        req.map_url = os.path.join(HERE, "maps", scenario + ".yaml")
        self.load_map_cli.call_async(req)
        return True
