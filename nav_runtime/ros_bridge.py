#!/usr/bin/env python3
"""
ROS 2 桥 (执行进程内部) —— 把 REST 拉取的仿真数据转换成 Nav2 所需的 ROS 2 接口，并把 Nav2 输出回馈给仿真进程

  REST (SimLink)                 →  ROS 2 (仅执行进程内部，供 Nav2 使用)
  ─────────────────────────────────────────────────────────────────────
  state.odom                      →  /odom (nav_msgs/Odometry，轮式里程计)
  state.imu                       →  /imu (sensor_msgs/Imu，imu_link)
  定位 (两种引擎):
    slam_toolbox (默认，有 ROS 且已安装)  robot_localization EKF (/odom + /imu) 发布 TF odom→base_footprint，
                                          slam_toolbox (/scan) 发布 TF map→odom 与 /map；见 ros_slam.py、nav2/loc_launch.py
    builtin (无 slam_toolbox 或 odom/ground_truth 模式)  本桥发布 TF odom→base_footprint (轮式里程计)
                                          与 map→odom (nav_runtime/slam.py)；SIM_LOCALIZATION=amcl 时由 AMCL 发布 map→odom
  state.joints                    →  /joint_states
  lidars/{2d}                     →  /scan/<name> (sensor_msgs/LaserScan, 传感器坐标系)
  lidars/{3d} (Mid-360S)          →  /livox/lidar (PointCloud2, livox_ros_driver2 字段布局) + /scan/<name> 切片
  sensors/scan (融合)              →  /scan (base_link)
  model/urdf                      →  robot_state_publisher (子进程，车型变化时重启)

  /cmd_vel (Nav2 velocity_smoother 输出) → PUT /api/v1/control/cmd_vel (仅 Nav2 任务执行期间)

  C++ 核心模式 (NAV_CPP_CORE=1 默认，agv_ros_bridge --core)：状态/2D 激光/融合扫描由 C++ 直接从仿真推送流发布，
  /cmd_vel 由 C++ 安全层下发；本桥只剩 3D 点云、相机、robot_state_publisher，并把 TF 发布标志 (MODE) 交给 C++
  Nav2 NavigateToPose 反馈/结果         → Navigator → PUT /api/v1/nav/feedback
"""

import array
import math
import os
import shutil
import signal
import subprocess
import tempfile
import threading
import time

from common import spawn

# 轮子关节 TF 频率 (导航不依赖)；Android proot 下降到 10 Hz 减少 /tf 扇出
RSP_HZ = float(os.environ.get("RSP_HZ", "10.0" if os.path.exists("/system/build.prop") else "30.0"))

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, LaserScan, PointCloud2, PointField
from tf2_ros import TransformBroadcaster

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class RspSupervisor:
    """robot_state_publisher 子进程 (URDF 来自仿真进程 GET /api/v1/model/urdf)"""

    def __init__(self, log):
        self.log, self.proc, self.urdf = log, None, None

    def apply(self, urdf: str):
        if not urdf or urdf == self.urdf or shutil.which("ros2") is None:
            return
        self.urdf = urdf
        self.stop()
        body = "\n".join("      " + ln for ln in urdf.rstrip("\n").split("\n"))
        fd, path = tempfile.mkstemp(suffix=".yaml", prefix="rsp_")
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            f.write(f"robot_state_publisher:\n  ros__parameters:\n    publish_frequency: {RSP_HZ}\n    robot_description: |\n" + body + "\n")
        self.proc = spawn.popen("rsp", ["ros2", "run", "robot_state_publisher", "robot_state_publisher", "--ros-args",
                                                "--params-file", path])
        self.log(f"[ros_bridge] robot_state_publisher 已启动 (URDF {len(urdf)} 字节)")

    def stop(self):
        if self.proc and self.proc.poll() is None:
            try:
                spawn.killpg(self.proc, signal.SIGINT)
                self.proc.wait(timeout=5)
            except Exception:
                pass
        self.proc = None


class RosBridge(Node):
    LIVOX_DT = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4"),
                         ("tag", "u1"), ("line", "u1"), ("timestamp", "<f8")])

    def __init__(self, link, navigator):
        super().__init__("nav_runtime_bridge")
        self.link, self.nav = link, navigator
        self.localization = os.environ.get("SIM_LOCALIZATION", "ground_truth")
        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        # 状态/激光/TF 发布端: C++ 节点 (ros2/agv_ros_bridge，NAV_ROS_BRIDGE=cpp|auto) 或本进程 rclpy (py)
        from nav_runtime.cpp_bridge import CppBridge, wanted_mode
        self.cpp = None
        self.bridge_mode = wanted_mode()
        if self.bridge_mode == "cpp":
            try:
                self.cpp = CppBridge(log=self.get_logger().info)
            except Exception as e:
                self.get_logger().warn(f"C++ 发布端启动失败，改用 Python 发布: {e}")
                self.bridge_mode = "py"
        if self.cpp is None:
            self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
            self.gt_pub = self.create_publisher(Odometry, "/ground_truth/odom", 10)
            self.js_pub = self.create_publisher(JointState, "/joint_states", 10)
            self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
            self.imu_pub = self.create_publisher(Imu, "/imu", 20)
            self.tf = TransformBroadcaster(self)
        self.sensor_qos = sensor_qos
        self.lidar_pubs, self.cloud_pubs = {}, {}
        self.core = self.cpp is not None and self.cpp.core
        if self.core:
            self.cpp.on_relay = link.feed                    # 推送流由 C++ 接收，帧转给 SimLink
        else:
            self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.rsp = RspSupervisor(self.get_logger().info)
        self._n = 0
        link.on_state.append(self._on_state)
        link.on_lidar.append(self._on_lidar)
        link.on_merged.append(self._on_merged)
        link.on_camera.append(self._on_camera)
        self.cam_pubs = {}
        link.camera_wanted = self._camera_wanted
        link._ensure_camera_threads()
        # 开源定位栈 (robot_localization + slam_toolbox)；LOC_ENGINE=builtin 强制使用内置 SLAM
        self.loc = None
        if os.environ.get("LOC_ENGINE", "auto") != "builtin" and self.localization != "amcl":
            from nav_runtime.ros_slam import RosLocalization, stack_installed
            if stack_installed():
                self.loc = RosLocalization(self, navigator.slam, log=self.get_logger().info)
                navigator.slam.attach_external(self.loc)
            else:
                self.get_logger().warn("未安装 slam_toolbox / robot_localization，定位使用内置 SLAM (nav_runtime/slam.py)")
        link.on_model_change.append(lambda m: self.rsp.apply(self.link.urdf))
        if link.urdf:
            self.rsp.apply(link.urdf)

    def sim_stamp(self, t_sim):
        """仿真时间 → ROS 时间戳 (墙钟)。传感器数据经 REST 到达有几十毫秒延迟，用接收时刻打时间戳会让
        slam_toolbox 把激光和错误时刻的里程计配对 (0.8 m/s 下 80 ms ≈ 6 cm)。
        偏移量 = 墙钟 - 仿真时间，取状态帧上观测到的最小值 (最小延迟)，缓慢跟随仿真实时因子的变化。"""
        from builtin_interfaces.msg import Time as TimeMsg
        now = self.get_clock().now().nanoseconds * 1e-9
        off = getattr(self, "_toff", None)
        if off is None or t_sim is None:
            return self.now()
        ts = min(t_sim + off, now)
        m = TimeMsg()
        m.sec = int(ts)
        m.nanosec = int((ts - int(ts)) * 1e9)
        return m

    def _track_offset(self, t_sim):
        if t_sim is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        o = now - float(t_sim)
        prev = getattr(self, "_toff", None)
        # 取最小延迟；仿真比实时慢/快时偏移会漂，给 0.5%/帧 的向上松弛
        self._toff = o if prev is None or o < prev or abs(o - prev) > 1.0 else prev + 0.005 * (o - prev)

    def now(self):
        return self.get_clock().now().to_msg()

    # ------------------------------------------------------------------ REST → ROS
    def _on_state(self, st):
        self._track_offset(st.get("t"))
        if self.cpp is not None:
            self._on_state_cpp(st)
            return
        stamp = self.sim_stamp(st.get("t"))
        o, tr, m2o = st["odom"], st["truth"], st["map_to_odom"]
        od = Odometry()
        od.header.stamp, od.header.frame_id, od.child_frame_id = stamp, "odom", "base_footprint"
        od.pose.pose.position.x, od.pose.pose.position.y = o["x"], o["y"]
        q = quat(o["yaw"])
        od.pose.pose.orientation.x, od.pose.pose.orientation.y, od.pose.pose.orientation.z, od.pose.pose.orientation.w = q
        od.twist.twist.linear.x, od.twist.twist.linear.y, od.twist.twist.angular.z = o["vx"], o["vy"], o["wz"]
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.02 ** 2
        cov[35] = math.radians(0.5) ** 2
        cov[14] = cov[21] = cov[28] = 1e6
        od.pose.covariance = cov
        tc = [0.0] * 36                                  # 速度协方差 (EKF 融合用)
        tc[0], tc[7], tc[35] = 0.01 ** 2, 0.01 ** 2, 0.01 ** 2
        tc[14] = tc[21] = tc[28] = 1e6
        od.twist.covariance = tc
        self.odom_pub.publish(od)
        im = st.get("imu")
        if im:
            m = Imu()
            m.header.stamp, m.header.frame_id = stamp, "imu_link"
            m.angular_velocity.z = float(im.get("wz", 0.0))
            m.linear_acceleration.x, m.linear_acceleration.y = float(im.get("ax", 0.0)), float(im.get("ay", 0.0))
            m.linear_acceleration.z = float(im.get("az", 9.81))
            m.orientation_covariance[0] = -1.0          # 不提供姿态
            m.angular_velocity_covariance[0] = m.angular_velocity_covariance[4] = 1e-4
            m.angular_velocity_covariance[8] = 0.02 ** 2     # 含零偏的实际水平，避免 EKF 过度相信陀螺
            m.linear_acceleration_covariance[0] = m.linear_acceleration_covariance[4] = m.linear_acceleration_covariance[8] = 0.02 ** 2
            self.imu_pub.publish(m)
        slam = getattr(self.nav, "slam", None)
        ext_tf = slam is not None and slam.ros_active()   # EKF + slam_toolbox 接管 TF
        gt = Odometry()
        gt.header.stamp, gt.header.frame_id, gt.child_frame_id = stamp, "map", "base_footprint"
        gt.pose.pose.position.x, gt.pose.pose.position.y = tr["x"], tr["y"]
        qg = quat(tr["yaw"])
        gt.pose.pose.orientation.x, gt.pose.pose.orientation.y, gt.pose.pose.orientation.z, gt.pose.pose.orientation.w = qg
        self.gt_pub.publish(gt)
        tfs = []
        loc = getattr(self, "loc", None)
        own_odom = not ext_tf or (loc is not None and not getattr(loc, "ekf", True))
        if own_odom:
            t = TransformStamped()
            t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, "odom", "base_footprint"
            t.transform.translation.x, t.transform.translation.y = o["x"], o["y"]
            t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
            tfs.append(t)
        if self.localization != "amcl" and not ext_tf:
            # map→odom 来自执行进程定位 (SLAM+里程计融合；定位模式为 ground_truth 时即真值)
            mx, my, myaw = slam.M if slam is not None else (m2o["x"], m2o["y"], m2o["yaw"])
            mt = TransformStamped()
            mt.header.stamp, mt.header.frame_id, mt.child_frame_id = stamp, "map", "odom"
            mt.transform.translation.x, mt.transform.translation.y = mx, my
            qm = quat(myaw)
            mt.transform.rotation.x, mt.transform.rotation.y, mt.transform.rotation.z, mt.transform.rotation.w = qm
            tfs.append(mt)
        if tfs:
            self.tf.sendTransform(tfs)
        self._own_odom_tf = own_odom
        self._odom_stamp = stamp.sec + stamp.nanosec * 1e-9     # 之后发布的代价地图激光不晚于此 (TF 已发出)
        self._n += 1
        if self._n % 2 == 0:
            j = st.get("joints", {})
            js = JointState()
            js.header.stamp = stamp
            js.name, js.position, js.velocity, js.effort = j.get("names", []), j.get("position", []), j.get("velocity", []), j.get("effort", [])
            self.js_pub.publish(js)

    def _on_state_cpp(self, st):
        """C++ 发布端: 发布标志与 map→odom 仍由这里决定 (与 Python 发布逻辑同一套判断)；
        核心模式下 C++ 自己发布状态，这里只把标志 (MODE) 交给它"""
        from nav_runtime.cpp_bridge import F_MAP_ODOM, F_OWN_ODOM
        slam = getattr(self.nav, "slam", None)
        ext_tf = slam is not None and slam.ros_active()
        loc = getattr(self, "loc", None)
        own_odom = not ext_tf or (loc is not None and not getattr(loc, "ekf", True))
        flags = F_OWN_ODOM if own_odom else 0
        m2o = (0.0, 0.0, 0.0)
        if self.localization != "amcl" and not ext_tf:
            m = st["map_to_odom"]
            m2o = slam.M if slam is not None else (m["x"], m["y"], m["yaw"])
            flags |= F_MAP_ODOM
        self._own_odom_tf = own_odom
        if self.core:
            self.mode_flags = {"own_odom": own_odom, "map_odom": bool(flags & F_MAP_ODOM),
                               "m2o": [round(float(v), 5) for v in m2o], "loc_ext": bool(ext_tf), "want_raw_lidar": not ext_tf}
            return
        self.cpp.send_state(st, m2o, flags)

    def _lidar_cfg(self, name):
        for l in self.link.sensors.get("lidars", []):
            if l["name"] == name:
                return l
        return {}

    def _on_lidar(self, name, meta, payload):
        stamp = self.sim_stamp(meta.get("t"))
        if meta.get("type") == "3d":
            cfg = self._lidar_cfg(name)
            topic = cfg.get("topic_hint") or f"/points/{name}"
            if name not in self.cloud_pubs:
                self.cloud_pubs[name] = self.create_publisher(PointCloud2, topic, self.sensor_qos)
            xyzi, line = payload["xyzi"], payload["line"]
            arr = np.zeros(len(xyzi), dtype=self.LIVOX_DT)
            arr["x"], arr["y"], arr["z"], arr["intensity"] = xyzi[:, 0], xyzi[:, 1], xyzi[:, 2], xyzi[:, 3]
            arr["line"] = line
            arr["timestamp"] = stamp.sec * 1e9 + stamp.nanosec
            m = PointCloud2()
            m.header.stamp, m.header.frame_id = stamp, meta.get("frame_id", f"{name}_link")
            m.height, m.width = 1, len(arr)
            m.fields = [PointField(name=n, offset=o, datatype=d, count=1) for n, o, d in
                        (("x", 0, 7), ("y", 4, 7), ("z", 8, 7), ("intensity", 12, 7), ("tag", 16, 2), ("line", 17, 2), ("timestamp", 18, 8))]
            m.is_bigendian, m.point_step, m.row_step, m.is_dense = False, 26, 26 * len(arr), True
            m.data = array.array("B", arr.tobytes())
            self.cloud_pubs[name].publish(m)
            return
        if self.cpp is not None:
            if not self.core:                               # 核心模式: C++ 已从推送流发布
                self.cpp.send_scan(False, name, meta, payload["ranges"])
            return
        if name not in self.lidar_pubs:
            self.lidar_pubs[name] = self.create_publisher(LaserScan, f"/scan/{name}", self.sensor_qos)
        self.lidar_pubs[name].publish(self._scan(self._costmap_stamp(stamp), meta, payload["ranges"]))

    def _costmap_stamp(self, stamp):
        """/scan/<name> 只给 Nav2 代价地图 (障碍层) 用：时间戳不晚于已发布的里程计 (EKF 的 odom→base TF 已覆盖)。
        否则障碍层的 tf2 MessageFilter 要排队等 TF —— 在 Android proot 下这条等待路径会让 Nav2 进程的
        TF 监听整体卡死 (controller/planner 的 TF 停在启动几秒后，FollowPath 立即误判到达)。
        slam_toolbox 用的合并 /scan 保持真实时间戳。AGV_COSTMAP_SCAN_CLAMP=0 关闭。"""
        lim = getattr(self, "_odom_stamp", None)
        if lim is None or os.environ.get("AGV_COSTMAP_SCAN_CLAMP", "1") != "1":
            return stamp
        ts = stamp.sec + stamp.nanosec * 1e-9
        # 本进程自己发布 odom→base 时恰好覆盖；由 EKF 发布时留一点余量
        lim -= float(os.environ.get("AGV_COSTMAP_SCAN_MARGIN", "0.0" if getattr(self, "_own_odom_tf", False) else "0.06"))
        if ts <= lim:
            return stamp
        from builtin_interfaces.msg import Time as TimeMsg
        m = TimeMsg()
        m.sec, m.nanosec = int(lim), int((lim - int(lim)) * 1e9)
        return m

    # 相机: /<name>/image_raw | /<name>/left|right/image_raw | /<name>/depth/image_raw (32FC1) | /<name>/points + camera_info
    TOPICS = {"rgb": "image_raw", "left": "left/image_raw", "right": "right/image_raw", "depth": "depth/image_raw",
              "amplitude": "amplitude/image_raw", "points": "points"}

    def _pub(self, key, typ, topic):
        if key not in self.cam_pubs:
            self.cam_pubs[key] = self.create_publisher(typ, topic, self.sensor_qos)
        return self.cam_pubs[key]

    def _camera_wanted(self, name, info) -> bool:
        """相机话题当前是否有订阅者 (图像/点云/camera_info 任一)。预先建好发布者，订阅方才能匹配上"""
        n = 0
        for st in info.get("streams", []):
            topic = f"/{name}/{self.TOPICS.get(st, st)}"
            if st == "points":
                n += self._pub((name, st), PointCloud2, topic).get_subscription_count()
                continue
            n += self._pub((name, st), Image, topic).get_subscription_count()
            n += self._pub((name, st, "info"), CameraInfo, f"{topic.rsplit('/', 1)[0]}/camera_info").get_subscription_count()
        return n > 0

    def _on_camera(self, name, info, frames, meta):
        stamp = self.now()
        for st, a in frames.items():
            frame = info["frame_id"] if st != "right" else f"{name}_right_optical_frame"
            topic = f"/{name}/{self.TOPICS.get(st, st)}"
            if st == "points":
                m = PointCloud2()
                m.header.stamp, m.header.frame_id = stamp, frame
                m.height, m.width = 1, len(a)
                m.fields = [PointField(name=n, offset=o, datatype=7, count=1) for n, o in (("x", 0), ("y", 4), ("z", 8))]
                m.is_bigendian, m.point_step, m.row_step, m.is_dense = False, 12, 12 * len(a), True
                m.data = array.array("B", np.ascontiguousarray(a, "<f4").tobytes())
                self._pub((name, st), PointCloud2, topic).publish(m)
                continue
            img = Image()
            img.header.stamp, img.header.frame_id = stamp, frame
            img.height, img.width = int(a.shape[0]), int(a.shape[1])
            if a.ndim == 3:
                img.encoding, img.step = "rgb8", img.width * 3
            elif a.dtype == np.uint16:
                img.encoding, img.step = "16UC1", img.width * 2
            else:
                img.encoding, img.step = "32FC1", img.width * 4
            img.is_bigendian = 0
            img.data = array.array("B", np.ascontiguousarray(a).tobytes())
            self._pub((name, st), Image, topic).publish(img)
            ci = CameraInfo()
            ci.header = img.header
            ci.height, ci.width = img.height, img.width
            ci.distortion_model = "plumb_bob"
            ci.d = [0.0] * 5
            K = [float(k) for k in info["K"]]
            ci.k = K
            ci.r = [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
            tx = -K[0] * float(info.get("baseline_m", 0.0)) if st == "right" else 0.0
            ci.p = [K[0], 0.0, K[2], tx, 0.0, K[4], K[5], 0.0, 0.0, 0.0, 1.0, 0.0]
            base = topic.rsplit("/", 1)[0]
            self._pub((name, st, "info"), CameraInfo, f"{base}/camera_info").publish(ci)

    def _on_merged(self, d):
        if self.core:
            return
        r0 = d["ranges"]
        rs = r0.astype(np.float32) if isinstance(r0, np.ndarray) else np.array([np.inf if r is None else r for r in r0], dtype=np.float32)
        if self.cpp is not None:
            if not self.core:
                self.cpp.send_scan(True, "merged", d, rs)
            return
        self.scan_pub.publish(self._scan(self.sim_stamp(d.get("t")), d, rs))

    @staticmethod
    def _scan(stamp, meta, ranges):
        s = LaserScan()
        s.header.stamp, s.header.frame_id = stamp, meta.get("frame_id", "base_link")
        s.angle_min = float(meta["angle_min"])
        s.angle_increment = float(meta["angle_increment"])
        s.angle_max = float(meta["angle_min"] + meta["angle_increment"] * (len(ranges) - 1))
        s.range_min, s.range_max = float(meta.get("range_min", 0.05)), float(meta.get("range_max", 30.0))
        s.scan_time = 1.0 / max(1.0, float(meta.get("scan_hz", 10.0)))
        s.ranges = array.array("f", np.asarray(ranges, dtype=np.float32).tobytes())
        return s

    # ------------------------------------------------------------------ ROS → REST
    def _on_cmd_vel(self, msg: Twist):
        # 仅 Nav2 任务执行中转发 (其它规划器由 Navigator 直接下发，避免指令冲突)
        if self.nav.active_planner == "nav2" and self.nav.telemetry.get("nav_status") in ("NAVIGATING", "PLANNING", "OBSTACLE_WAIT"):
            vx, vy, wz = self.nav.safety_filter(msg.linear.x, msg.linear.y, msg.angular.z)
            self.link.send_cmd(vx, vy, wz, source="nav2")

    def graph(self):
        """ROS 图 (节点/话题)；遍历 DDS 发现数据较慢，2 s 缓存 (/api/v1/nav 被网页与采样工具频繁调用)"""
        c = getattr(self, "_graph_cache", None)
        if c and time.time() - c[0] < 2.0:
            return c[1]
        g = self._graph()
        self._graph_cache = (time.time(), g)
        return g

    def _graph(self):
        try:
            nodes = sorted({(ns.rstrip("/") + "/" + n) for n, ns in self.get_node_names_and_namespaces()})
            topics = sorted(t for t, _ in self.get_topic_names_and_types() if not t.startswith("/rosout"))
            return {"nodes": nodes, "topics": topics}
        except Exception:
            return {"nodes": [], "topics": []}

    def bridge_status(self) -> dict:
        return self.cpp.status() if self.cpp is not None else {"mode": "py"}

    def shutdown(self):
        self.rsp.stop()
        if self.cpp is not None:
            self.cpp.stop()
        if self.loc is not None:
            self.loc.stop()
