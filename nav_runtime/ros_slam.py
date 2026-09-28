#!/usr/bin/env python3
"""
开源定位栈监管 (ROS 2)：robot_localization EKF + slam_toolbox

  由 RosBridge 创建，挂到 Navigator.slam (SlamLocalizer) 的 ext 上：
    start(mode, scenario, pose)   启动/重启 nav2/loc_launch.py (mapping | localization)
    save(scenario)                slam_toolbox 位姿图序列化 (<场景>.posegraph/.data) + 当前 /map 另存 PGM/YAML
    poll()                        50 Hz：TF map→base_footprint → SlamLocalizer.set_external (导引使用的位姿)
    grid()                        最新 /map (占据栅格) → 界面叠加

  坐标系约定：EKF 的 odom 系在启动时用 /set_pose 对齐到车辆初始位姿 (世界系)，
  slam_toolbox 建图时第一帧位姿取自 odom，因此 SLAM 地图与场景拓扑在同一坐标系下。
"""
import math
import os
import shutil
import signal
import subprocess
import threading
import time

from common import spawn

import numpy as np

# ROS 消息/rclpy 按需导入: 核心模式 (nav_runtime/cpp_ros.py) 下执行进程不加载 rclpy，
# 地图 / 初始位姿 / 位姿图保存都经 C++ 桥接完成

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _pkg(name) -> bool:
    if shutil.which("ros2") is None:
        return False
    try:
        return subprocess.run(["ros2", "pkg", "prefix", name], capture_output=True, timeout=10).returncode == 0
    except Exception:
        return False


def stack_installed() -> bool:
    return _pkg("slam_toolbox") and _pkg("robot_localization")


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RosLocalization:
    name = "slam_toolbox"

    def __init__(self, node, slam, log=print):
        self.node, self.slam, self.log = node, slam, log
        # robot_localization EKF (odom→base_footprint)。里程计只融合轮速时 EKF 等同于积分，
        # Android 上默认不启动: 由执行进程直接发布轮式里程计 TF，并保证激光时间戳不晚于它
        # (EKF 偶发 0.3 s 卡顿时 Nav2 障碍层的 tf2 MessageFilter 要排队等 TF，proot 下会卡死 Nav2 的 TF 监听)。
        self.ekf = os.environ.get("LOC_EKF", "0" if os.path.exists("/system/build.prop") else "1") == "1"
        self.proc = None
        self.mode = None
        self.scenario = None
        self.lock = threading.Lock()
        self.map_msg = None
        self.map_rev = 0
        self.last_tf = 0.0
        self.last_tf_wall = 0.0      # 最近一次收到新的 map→base 定位的墙钟时刻 (定位停更保护用)
        self.started_at = 0.0
        self.err = None
        # C++ 发布端 (ros_bridge.cpp) 在时: TF 监听与 50 Hz 查询都在 C++ 里做，结果经 on_tf 回调送来；
        # 否则本进程 rclpy 订阅 /tf (每条 TF 消息都要在 Python 里反序列化) + 50 Hz 定时器
        self.cpp = getattr(node, "cpp", None)
        self.cppmode = getattr(node, "rclpy_free", False)
        self.map_file = None                          # 核心模式: C++ 写的 /map 共享内存文件
        if self.cpp is not None:
            self.buf = self.tfl = None
            self.cpp.on_tf = self._on_cpp_tf
        else:
            try:
                import tf2_ros
            except Exception:  # pragma: no cover
                tf2_ros = None
            self.buf = tf2_ros.Buffer() if tf2_ros else None
            self.tfl = tf2_ros.TransformListener(self.buf, node) if tf2_ros else None
            node.create_timer(0.02, self.poll)
        if self.cppmode:
            node.on_map = self._on_cpp_map
            self.set_pose_pub = None
        else:
            from nav_msgs.msg import OccupancyGrid
            from geometry_msgs.msg import PoseWithCovarianceStamped
            from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            node.create_subscription(OccupancyGrid, "/map", self._on_map, qos)
            self.set_pose_pub = node.create_publisher(PoseWithCovarianceStamped, "/set_pose", 10)

    # ------------------------------------------------------------------ 进程
    def running(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def map_base(self, sid):
        return os.path.join(self.slam.map_dir, sid)

    def has_saved(self, sid) -> bool:
        return bool(sid) and os.path.exists(self.map_base(sid) + ".posegraph")

    def start(self, mode, sid, pose):
        """mode: mapping | localization；pose: 车辆当前世界位姿 (初始位姿)"""
        with self.lock:
            self._stop_locked()
            cmd = ["ros2", "launch", os.path.join(HERE, "nav2", "loc_launch.py"), f"mode:={mode}",
                   f"use_sim_time:={'true' if os.environ.get('SIM_USE_SIM_TIME', '0') == '1' else 'false'}",
                   f"ekf:={'true' if self.ekf else 'false'}"]
            if mode == "localization":
                cmd += [f"map_file:={self.map_base(sid)}", f"start_x:={pose[0]:.4f}", f"start_y:={pose[1]:.4f}", f"start_yaw:={pose[2]:.5f}"]
            self.log("[ros_slam] 启动定位栈: " + " ".join(cmd))
            self.proc = spawn.popen("loc", cmd)
            self.mode, self.scenario, self.started_at = mode, sid, time.time()
            self.map_msg, self.last_tf, self.err = None, 0.0, None
            self.last_tf_wall = 0.0
            self.map_rev += 1
        if self.ekf:
            threading.Thread(target=self._align_odom, args=(pose, self.started_at), daemon=True, name="ekf-set-pose").start()

    def _align_odom(self, pose, token):
        """EKF 启动后把 odom 系对齐到车辆初始世界位姿 (重复发送直到 TF 反映出来，最多 15 s)"""
        t_end = time.time() + 15.0
        while time.time() < t_end and self.started_at == token and self.running():
            if self.cppmode:
                self.node.roscall("set_pose", x=float(pose[0]), y=float(pose[1]), yaw=float(pose[2]), wait=False)
                time.sleep(0.3)
                ob = self.cpp.odom_base if self.cpp is not None else None
                if ob is not None and math.hypot(ob[0] - pose[0], ob[1] - pose[1]) < 0.3:
                    return
                continue
            from geometry_msgs.msg import PoseWithCovarianceStamped
            m = PoseWithCovarianceStamped()
            m.header.frame_id = "odom"
            m.header.stamp = self.node.get_clock().now().to_msg()
            m.pose.pose.position.x, m.pose.pose.position.y = float(pose[0]), float(pose[1])
            m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(pose[2] / 2), math.cos(pose[2] / 2)
            c = [0.0] * 36
            c[0] = c[7] = 1e-6
            c[35] = 1e-6
            m.pose.covariance = c
            self.set_pose_pub.publish(m)
            time.sleep(0.3)
            try:
                if self.cpp is not None:
                    ob = self.cpp.odom_base
                    if ob is not None and math.hypot(ob[0] - pose[0], ob[1] - pose[1]) < 0.3:
                        return
                    continue
                from rclpy.time import Time
                tr = self.buf.lookup_transform("odom", "base_footprint", Time())
                p = tr.transform.translation
                if math.hypot(p.x - pose[0], p.y - pose[1]) < 0.3:
                    return
            except Exception:
                continue

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
        self.mode = None

    def stop(self):
        with self.lock:
            self._stop_locked()

    # ------------------------------------------------------------------ 数据
    def _on_map(self, msg):
        self.map_msg = msg
        self.map_rev += 1

    def poll(self):
        """TF map→base_footprint → 导引位姿 (TF 时间戳换算成仿真时间，对齐里程计历史，零延迟推算到当前)"""
        if not self.running() or self.buf is None:
            return
        try:
            from rclpy.time import Time
            tr = self.buf.lookup_transform("map", "base_footprint", Time())
        except Exception:
            return
        st = tr.header.stamp
        stamp = st.sec + st.nanosec * 1e-9
        if stamp <= self.last_tf:
            return
        self.last_tf = stamp
        self.last_tf_wall = time.time()
        t, q = tr.transform.translation, tr.transform.rotation
        off = getattr(self.node, "_toff", None)          # ros_bridge: 墙钟 - 仿真时间
        if off is None:
            return
        self.slam.set_external(stamp - off, (t.x, t.y, _yaw(q)))

    def _on_cpp_tf(self, stamp, toff, pose):
        """C++ 发布端回传的 map→base_footprint (stamp 为 ROS 墙钟时间，toff = 墙钟 - 仿真时间)"""
        if not self.running() or stamp <= self.last_tf:
            return
        self.last_tf = stamp
        self.last_tf_wall = time.time()
        self.slam.set_external(stamp - toff, pose)

    def _on_cpp_map(self, path, rev):
        self.map_file = path
        self.map_msg = True                           # 仅作"已收到"标志
        self.map_rev += 1

    def _grid_file(self):
        """C++ 写的 /map 文件: <u32 w><u32 h><f64 分辨率><f64 原点 x><f64 原点 y><int8 × w·h>"""
        import struct
        try:
            with open(self.map_file, "rb") as f:
                b = f.read()
        except OSError:
            return None
        w, h, res, ox, oy = struct.unpack_from("<IIddd", b, 0)
        d = np.frombuffer(b, dtype=np.int8, offset=32, count=w * h).astype(np.int16).reshape(h, w)
        c = np.zeros((h, w), np.uint8)
        c[(d >= 0) & (d < 25)] = 1
        c[d >= 65] = 2
        return c, float(res), float(ox), float(oy), self.map_rev

    def grid(self):
        if self.cppmode:
            return self._grid_file() if self.map_file else None
        m = self.map_msg
        if m is None:
            return None
        w, h = m.info.width, m.info.height
        d = np.asarray(m.data, dtype=np.int16).reshape(h, w)
        c = np.zeros((h, w), np.uint8)
        c[(d >= 0) & (d < 25)] = 1
        c[d >= 65] = 2
        o = m.info.origin.position
        return c, float(m.info.resolution), float(o.x), float(o.y), self.map_rev

    def save(self, sid):
        """位姿图 (定位模式用) + PGM/YAML (Nav2 map_server 可直接加载)"""
        base = self.map_base(sid)
        os.makedirs(os.path.dirname(base), exist_ok=True)
        out = {}
        if self.cppmode:
            r = self.node.roscall("save_map", file=base, wait=True, timeout=25.0)
            if not r or not r.get("ok"):
                raise RuntimeError((r or {}).get("err") or "位姿图序列化超时")
        else:
            from slam_toolbox.srv import SerializePoseGraph
            cli = self.node.create_client(SerializePoseGraph, "/slam_toolbox/serialize_map")
            if not cli.wait_for_service(timeout_sec=3.0):
                raise RuntimeError("slam_toolbox 服务 /slam_toolbox/serialize_map 未就绪")
            req = SerializePoseGraph.Request()
            req.filename = base
            fut = cli.call_async(req)
            t_end = time.time() + 20.0
            while not fut.done() and time.time() < t_end:
                time.sleep(0.05)
            if not fut.done():
                raise RuntimeError("位姿图序列化超时")
        out["posegraph"] = base + ".posegraph"
        g = self.grid()
        if g is not None:
            c, res, ox, oy, _ = g
            img = np.full(c.shape, 205, np.uint8)
            img[c == 1] = 254
            img[c == 2] = 0
            img = np.flipud(img)
            with open(base + ".pgm", "wb") as f:
                f.write(b"P5\n%d %d\n255\n" % (img.shape[1], img.shape[0]) + img.tobytes())
            with open(base + ".yaml", "w") as f:
                f.write(f"image: {os.path.basename(base)}.pgm\nresolution: {res}\norigin: [{ox:.3f}, {oy:.3f}, 0.0]\n"
                        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\nmode: trinary\n")
            out["pgm"], out["yaml"] = base + ".pgm", base + ".yaml"
        return out

    def delete_saved(self, sid):
        n = 0
        for ext in (".posegraph", ".data", ".pgm", ".yaml"):
            p = self.map_base(sid) + ext
            if os.path.exists(p):
                os.remove(p)
                n += 1
        return n

    def status(self):
        return {"engine": self.name, "process": self.running(), "stack_mode": self.mode,
                "tf_age_s": round(time.time() - self.last_tf, 2) if self.last_tf else None,
                "map_received": self.map_msg is not None, "since_start_s": round(time.time() - self.started_at, 1) if self.started_at else None}
