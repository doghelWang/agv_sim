#!/usr/bin/env python3
"""
C++ ROS 2 发布端 (ros2/agv_ros_bridge) 的 Python 客户端  —— NAV_ROS_BRIDGE=cpp|py|auto (默认 auto: 有可执行文件就用 C++)

  Python → C++ (Unix 数据报，容器内私有路径，多实例 host 网络下不占端口)
    STATE  每个状态帧: 里程计/真值/IMU/关节 + 由 Python 决定的 map→odom 与发布标志
    SCAN   2D 激光 (/scan/<name>，代价地图钳位) 与融合扫描 (/scan)
  C++ → Python
    TF     map→base_footprint (slam_toolbox/EKF 定位结果) + odom→base_footprint + 墙钟-仿真时间偏移
    STATS  每秒一次: 已发布的状态/激光/融合/TF 计数

  C++ 节点负责 /odom /ground_truth/odom /imu /joint_states /scan /scan/<name> 与 TF 广播/监听；
  Python 节点只剩 cmd_vel、/map、Nav2 Action 客户端、相机与 3D 点云 → rclpy 执行器不再处理 /tf 与 50 Hz 定时器。
"""
import os
import shutil
import signal
import socket
import struct
import tempfile
import threading
import time
from typing import Callable, Optional

import numpy as np

from common import spawn

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAGIC = b"AGV1"
T_STATE, T_SCAN, T_TF, T_STATS = 1, 2, 10, 11
F_OWN_ODOM, F_MAP_ODOM, F_IMU, F_HAS_T = 1, 2, 4, 8
_STATE = struct.Struct("<4sBd6d3d3d4dBH")
_SCAN_HEAD = struct.Struct("<4sBB")
_SCAN_NUM = struct.Struct("<6dI")
_TF = struct.Struct("<dd3d3dB")
_STATS = struct.Struct("<4I")


def find_binary() -> Optional[str]:
    cands = [os.environ.get("NAV_ROS_BRIDGE_BIN", ""),
             os.path.join(HERE, "ros2", "install", "agv_ros_bridge", "lib", "agv_ros_bridge", "agv_ros_bridge"),
             "/opt/agv_ros/lib/agv_ros_bridge/agv_ros_bridge"]
    for c in cands:
        if c and os.path.isfile(c) and os.access(c, os.X_OK):
            return c
    return shutil.which("agv_ros_bridge")


def wanted_mode() -> str:
    """cpp | py (auto → 有可执行文件就用 cpp)"""
    m = os.environ.get("NAV_ROS_BRIDGE", "auto").strip().lower()
    if m == "py":
        return "py"
    return "cpp" if find_binary() else "py"


def _s(b: str) -> bytes:
    e = (b or "").encode("utf-8")[:255]
    return bytes([len(e)]) + e


class CppBridge:
    def __init__(self, log=print):
        self.log = log
        self.bin = find_binary()
        self.dir = tempfile.mkdtemp(prefix="agv_bridge_")
        self.in_path = os.path.join(self.dir, "cpp.sock")     # C++ 接收
        self.out_path = os.path.join(self.dir, "py.sock")     # Python 接收
        self.rx = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.rx.bind(self.out_path)
        self.rx.settimeout(0.5)
        self.tx = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.tx.setblocking(False)
        self.proc = None
        self.on_tf: Optional[Callable] = None            # cb(stamp, toff, map_base|None, odom_base|None)
        self.stats = {"state": 0, "scan": 0, "merged": 0, "tf": 0, "restarts": 0, "send_errors": 0}
        self.odom_base = None
        self.toff = None
        self._joint_cache = (None, b"")
        self._stop = threading.Event()
        self._start_proc()
        threading.Thread(target=self._rx_loop, daemon=True, name="cpp-bridge-rx").start()
        threading.Thread(target=self._watchdog, daemon=True, name="cpp-bridge-wd").start()

    # ------------------------------------------------------------------ 进程
    def _start_proc(self):
        self.proc = spawn.popen("ros_bridge_cpp", [self.bin, "--in", self.in_path, "--out", self.out_path])
        self.log(f"[cpp_bridge] 启动 C++ 发布端 {self.bin} (pid {getattr(self.proc, 'pid', '?')})")

    def _watchdog(self):
        while not self._stop.is_set():
            time.sleep(2.0)
            if self.proc is not None and self.proc.poll() is not None:
                self.stats["restarts"] += 1
                self.log(f"[cpp_bridge] C++ 发布端退出 (code {self.proc.returncode})，重启")
                self._start_proc()

    def running(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def stop(self):
        self._stop.set()
        if self.running():
            try:
                spawn.killpg(self.proc, signal.SIGINT)
                self.proc.wait(timeout=5)
            except Exception:
                pass
        for p in (self.in_path, self.out_path):
            try:
                os.unlink(p)
            except OSError:
                pass

    # ------------------------------------------------------------------ 发送
    def _send(self, b: bytes):
        try:
            self.tx.sendto(b, self.in_path)
        except OSError:            # C++ 端未就绪/缓冲满: 丢帧 (与 best-effort 话题语义一致)
            self.stats["send_errors"] += 1

    def send_state(self, st: dict, m2o, flags: int):
        o, tr = st["odom"], st["truth"]
        im = st.get("imu") or {}
        t = st.get("t")
        if im:
            flags |= F_IMU
        if t is not None:
            flags |= F_HAS_T
        j = st.get("joints") or {}
        names = j.get("names") or []
        key = tuple(names)
        if self._joint_cache[0] != key:
            self._joint_cache = (key, b"".join(_s(n) for n in names))
        n = len(names)
        vals = np.asarray(list(j.get("position", [])) + list(j.get("velocity", [])) + list(j.get("effort", [])), dtype="<f8")
        if len(vals) != 3 * n:
            n, vals = 0, np.zeros(0, "<f8")
            names_b = b""
        else:
            names_b = self._joint_cache[1]
        head = _STATE.pack(MAGIC, T_STATE, float(t if t is not None else "nan"),
                           o["x"], o["y"], o["yaw"], o["vx"], o["vy"], o["wz"], tr["x"], tr["y"], tr["yaw"],
                           float(m2o[0]), float(m2o[1]), float(m2o[2]),
                           float(im.get("wz", 0.0)), float(im.get("ax", 0.0)), float(im.get("ay", 0.0)), float(im.get("az", 9.81)),
                           flags, n)
        self._send(head + names_b + vals.tobytes())

    def send_scan(self, merged: bool, name: str, meta: dict, ranges):
        r = np.asarray(ranges, dtype="<f4")
        t = meta.get("t")
        b = (_SCAN_HEAD.pack(MAGIC, T_SCAN, 1 if merged else 0) + _s(name) + _s(meta.get("frame_id") or "base_link")
             + _SCAN_NUM.pack(float(t if t is not None else "nan"), float(meta["angle_min"]), float(meta["angle_increment"]),
                              float(meta.get("range_min", 0.05)), float(meta.get("range_max", 30.0)),
                              float(meta.get("scan_hz", 10.0)), len(r))
             + r.tobytes())
        self._send(b)

    # ------------------------------------------------------------------ 接收
    def _rx_loop(self):
        while not self._stop.is_set():
            try:
                b = self.rx.recv(65536)
            except socket.timeout:
                continue
            except OSError:
                time.sleep(0.2)
                continue
            if len(b) < 5 or b[:4] != MAGIC:
                continue
            if b[4] == T_TF and len(b) >= 5 + _TF.size:
                stamp, toff, mx, my, myaw, ox, oy, oyaw, fl = _TF.unpack_from(b, 5)
                self.toff = toff
                if fl & 2:
                    self.odom_base = (ox, oy, oyaw)
                if fl & 1:
                    self.stats["tf"] += 1
                    cb = self.on_tf
                    if cb is not None:
                        try:
                            cb(stamp, toff, (mx, my, myaw))
                        except Exception as e:  # pragma: no cover
                            self.log(f"[cpp_bridge] TF 回调异常: {e}")
            elif b[4] == T_STATS and len(b) >= 5 + _STATS.size:
                s, sc, mg, tf = _STATS.unpack_from(b, 5)
                self.stats.update({"cpp_state": s, "cpp_scan": sc, "cpp_merged": mg, "cpp_tf": tf})

    def status(self) -> dict:
        return dict(self.stats, mode="cpp", running=self.running(), bin=self.bin)
