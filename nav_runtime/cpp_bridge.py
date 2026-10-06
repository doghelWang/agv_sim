#!/usr/bin/env python3
"""
C++ ROS 2 发布端 (ros2/agv_ros_bridge) 的 Python 客户端  —— NAV_ROS_BRIDGE=cpp|py|auto (默认 auto: 有可执行文件就用 C++)

  Python → C++ (Unix 数据报，容器内私有路径，多实例 host 网络下不占端口)
    STATE  每个状态帧: 里程计/真值/IMU/关节 + 由 Python 决定的 map→odom 与发布标志
    SCAN   2D 激光 (/scan/<name>，代价地图钳位) 与融合扫描 (/scan)
    ROUTE  拓扑路线导航 (JSON)：C++ 发布 /agv/route、/agv/world_segments 并下发 NavigateToPose (agv_nav2_plugins 行为树)
    CANCEL 取消路线导航
  C++ → Python
    TF     map→base_footprint (slam_toolbox/EKF 定位结果) + odom→base_footprint + 墙钟-仿真时间偏移
    STATS  每秒一次: 已发布的状态/激光/融合/TF 计数
    NAV    路线导航回馈 (JSON)：结果 / 反馈 (5 Hz) / 停车点剩余行程 / 插件事件 / 规划曲线 / 安全层事件 (sevent)

  核心模式 (NAV_CPP_CORE=1，默认)：C++ 直接连仿真推送流并发布 ROS，Nav2 /cmd_vel 经 C++ 安全层下发；
    自研导引 (NAV_CPP_GUIDE=1，默认) 也在 C++ 里执行 (GUIDE 下发路线/拐点/参数，回收状态与结束)；
    Python → C++  CONFIG (保护空间/外形/光电，JSON)、MODE (是否转发 Nav2 指令、TF 发布标志、限速等，JSON)
    C++ → Python  RELAY (推送流帧原样: 状态/元信息/IO/融合扫描，内置 SLAM 需要时含各激光原始帧)、SAFETY (安全层快照)

  C++ 节点负责 /odom /ground_truth/odom /imu /joint_states /scan /scan/<name> 与 TF 广播/监听；
  Python 节点只剩 cmd_vel、/map、Nav2 Action 客户端、相机与 3D 点云 → rclpy 执行器不再处理 /tf 与 50 Hz 定时器。
"""
import os
import shutil
import signal
import socket
import struct
import threading
import time
from typing import Callable, Optional

import numpy as np

from common import spawn

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAGIC = b"AGV1"
T_STATE, T_SCAN, T_ROUTE, T_CANCEL, T_CONFIG, T_MODE, T_GUIDE, T_GUIDE_CANCEL = 1, 2, 3, 4, 5, 6, 7, 8
T_TF, T_STATS, T_NAV = 10, 11, 12
T_RELAY, T_SAFETY = 20, 30
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
        # Linux 抽象命名空间套接字 ("@name"，内核里只有名字没有文件): Android proot 下文件路径会被翻译成
        # 宿主机长路径 (超过 108 字节上限)，且各 ROS 进程在不同 proot 会话里；抽象名字不受影响
        tag = f"agv_bridge_{os.getpid()}_{int(time.time() * 1000) % 100000000}"
        self.in_path = f"@{tag}_cpp"     # C++ 接收
        self.out_path = f"@{tag}_py"     # Python 接收
        self.rx = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.rx.bind(self._addr(self.out_path))
        self.rx.settimeout(0.5)
        self.tx = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.tx.setblocking(False)
        self.proc = None
        self.on_tf: Optional[Callable] = None            # cb(stamp, toff, map_base|None, odom_base|None)
        self.on_nav: Optional[Callable] = None           # cb(dict) 路线导航回馈
        self.on_relay: Optional[Callable] = None         # cb(帧类型, bytes) 核心模式: 仿真推送流帧
        self.on_safety: Optional[Callable] = None        # cb(dict) 核心模式: 安全层快照
        self.on_sevent: Optional[Callable] = None        # cb(dict) 核心模式: 安全层事件
        self.on_guide: Optional[Callable] = None         # cb(dict) 核心模式: 自研导引状态 / 结束
        self.on_ros: Optional[Callable] = None           # cb(dict) 核心模式: ROS 图/Nav2 生命周期/地图/ROS 操作回复 (cpp_ros)
        self.core = os.environ.get("NAV_CPP_CORE", "1") != "0"
        self._last_mode = None
        self.stats = {"state": 0, "scan": 0, "merged": 0, "tf": 0, "restarts": 0, "send_errors": 0}
        self.odom_base = None
        self.toff = None
        self._joint_cache = (None, b"")
        self._stop = threading.Event()
        self._start_proc()
        threading.Thread(target=self._rx_loop, daemon=True, name="cpp-bridge-rx").start()
        threading.Thread(target=self._watchdog, daemon=True, name="cpp-bridge-wd").start()

    @staticmethod
    def _addr(name: str) -> str:
        return "\0" + name[1:] if name.startswith("@") else name

    # ------------------------------------------------------------------ 进程
    def _start_proc(self):
        argv = [self.bin, "--in", self.in_path, "--out", self.out_path] + (["--core"] if self.core else [])
        self.proc = spawn.popen("ros_bridge_cpp", argv)
        self._last_mode = None                           # 重启后重新下发 MODE / CONFIG
        if getattr(self, "_config", None):
            threading.Timer(1.0, lambda: self._send(MAGIC + bytes([T_CONFIG]) + self._config)).start()
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
        try:
            self.rx.close()
        except OSError:
            pass

    # ------------------------------------------------------------------ 发送
    def _send(self, b: bytes):
        try:
            self.tx.sendto(b, self._addr(self.in_path))
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

    def send_route(self, mission_id: int, route, goal, bt: str, segs, rev=None) -> None:
        """rev: 可选，与 route 等长；rev[i] 为真表示到达 route[i] 的这一段倒车 (rev[0] 无意义)"""
        import json
        body = json.dumps({"mid": int(mission_id), "route": [[round(float(x), 4), round(float(y), 4)] for x, y in route],
                           "rev": [1 if r else 0 for r in (rev or [])],
                           "goal": [float(goal[0]), float(goal[1]), float(goal[2])], "bt": bt,
                           "segs": [[round(float(v), 4) for v in sg[:4]] for sg in (segs or [])]}, separators=(",", ":"))
        self._send(MAGIC + bytes([T_ROUTE]) + body.encode("utf-8"))

    def send_cancel(self) -> None:
        # 末尾补 1 字节: 旧版 agv_ros_bridge 把恰好 5 字节的消息当成空帧丢掉 (取消从未到达 Nav2)
        self._send(MAGIC + bytes([T_CANCEL, 0]))

    def send_config(self, cfg: dict) -> None:
        import json
        self._config = json.dumps(cfg, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
        self._send(MAGIC + bytes([T_CONFIG]) + self._config)

    def send_guide(self, msg: dict) -> None:
        import json
        self._send(MAGIC + bytes([T_GUIDE]) + json.dumps(msg, separators=(",", ":"), ensure_ascii=False).encode("utf-8"))

    def send_guide_cancel(self) -> None:
        self._send(MAGIC + bytes([T_GUIDE_CANCEL, 0]))            # 同上: 补 1 字节

    def send_mode(self, mode: dict, force: bool = False) -> None:
        import json
        b = json.dumps(mode, separators=(",", ":")).encode("utf-8")
        if force or b != self._last_mode:
            self._last_mode = b
            self._send(MAGIC + bytes([T_MODE]) + b)

    # ------------------------------------------------------------------ 接收
    def _rx_loop(self):
        while not self._stop.is_set():
            try:
                b = self.rx.recv(1 << 20)
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
            elif T_RELAY < b[4] < T_RELAY + 10:
                cb = self.on_relay
                if cb is not None:
                    try:
                        cb(b[4] - T_RELAY, b[5:])
                    except Exception as e:  # pragma: no cover
                        self.log(f"[cpp_bridge] 推送流帧处理异常: {e}")
            elif b[4] in (T_NAV, T_SAFETY):
                import json
                try:
                    m = json.loads(b[5:].decode("utf-8"))
                except Exception:
                    continue
                k = m.get("k")
                cb = self.on_safety if b[4] == T_SAFETY else (self.on_sevent if k == "sevent" else
                                                              (self.on_guide if k in ("guide", "guide_done") else
                                                               (self.on_ros if k in ("ros", "map", "roscall") else self.on_nav)))
                if cb is not None:
                    try:
                        cb(m)
                    except Exception as e:  # pragma: no cover
                        self.log(f"[cpp_bridge] 回馈处理异常: {e}")
            elif b[4] == T_STATS and len(b) >= 5 + _STATS.size:
                s, sc, mg, tf = _STATS.unpack_from(b, 5)
                self.stats.update({"cpp_state": s, "cpp_scan": sc, "cpp_merged": mg, "cpp_tf": tf})

    def status(self) -> dict:
        return dict(self.stats, mode="cpp", running=self.running(), bin=self.bin)
