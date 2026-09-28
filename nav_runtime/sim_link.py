#!/usr/bin/env python3
"""
SimLink —— 执行进程侧的仿真数据接入层 (纯 REST，替代 ROS 话题订阅)

  推送流          GET /api/v1/stream           状态 50 Hz (二进制帧) + IO/光电 20 Hz，一条长连接 (默认)
  state 轮询     GET /api/v1/state            50 Hz  (推送流不可用时回退)
  io 轮询        GET /api/v1/io + /sensors/photoelectric  20 Hz  (同上；急停/触边/光电 DI 与检测距离 → 执行进程安全层)
  激光长轮询      GET /api/v1/sensors/lidars/{name}?after_seq=N&wait=0.5  (二进制，每帧恰好取一次)
  融合扫描长轮询  GET /api/v1/sensors/scan?after_seq=N&wait=0.5  (二进制)
  模型/场景      GET /api/v1/model, /api/v1/world (车型/场景变化时刷新)
  指令回馈       UDP (仿真 /api/v1/sim 声明 cmd_udp_port 时) 或 PUT /api/v1/control/cmd_vel
  状态回馈       PUT /api/v1/nav/feedback     5 Hz

  外部馈送 (C++ 核心模式，nav_runtime/cpp_bridge.py)：推送流由 agv_ros_bridge 连接，帧原样转发过来 (feed)，
  本进程不再自己连推送流；馈送中断 1.5 s 自动恢复自己连接
"""

import json
import math
import os
import struct
import threading
import time
from http.client import HTTPConnection
from typing import Callable, Dict, List, Optional

import numpy as np

from common.rest import RestClient


# 推送流帧格式 (与 sim_server/service.py STATE_FMT 一致)
STREAM_HDR = struct.Struct("<IB")
STATE_FMT = struct.Struct("<Idd6d6d3d5dIBddIH")


def decode_state(body: bytes, meta: dict) -> dict:
    """二进制状态帧 → 与 GET /api/v1/state 相同结构的 dict"""
    v = STATE_FMT.unpack_from(body, 0)
    seq, t, wall = v[0], v[1], v[2]
    tr, od, mo, im = v[3:9], v[9:15], v[15:18], v[18:23]
    coll, fl, cx, cy, rev, nj = v[23], v[24], v[25], v[26], v[27], v[28]
    arr = struct.unpack_from(f"<{3 * nj}d", body, STATE_FMT.size) if nj else ()
    names = meta.get("joint_names") or []
    if len(names) != nj:
        names = names[:nj] + [f"j{i}" for i in range(len(names), nj)]
    return {
        "seq": seq, "t": round(t, 4), "wall_time": wall,
        "truth": {"frame": "map", "x": tr[0], "y": tr[1], "yaw": tr[2], "vx": tr[3], "vy": tr[4], "wz": tr[5]},
        "odom": {"frame": "odom", "x": od[0], "y": od[1], "yaw": od[2], "vx": od[3], "vy": od[4], "wz": od[5]},
        "map_to_odom": {"x": mo[0], "y": mo[1], "yaw": mo[2]},
        "imu": {"wz": round(im[0], 6), "ax": round(im[1], 6), "ay": round(im[2], 6), "az": round(im[3], 6), "yaw": round(im[4], 6)},
        "joints": {"names": list(names), "position": list(arr[:nj]), "velocity": list(arr[nj:2 * nj]), "effort": list(arr[2 * nj:])},
        "collision": {"count": coll, "bumper_front": bool(fl & 1), "bumper_rear": bool(fl & 2),
                      "last_contact": (cx, cy) if fl & 8 else None},
        "paused": bool(fl & 4), "chassis": meta.get("chassis"), "scenario": meta.get("scenario"), "model_rev": rev,
    }


class SimLink:
    def __init__(self, base_url: str, log=print):
        self.url = base_url
        self.log = log
        self.c_state = RestClient(base_url, timeout=1.0)
        self.c_cmd = RestClient(base_url, timeout=1.0)
        self.c_misc = RestClient(base_url, timeout=3.0)
        self.lock = threading.Lock()
        self.state: Dict = {}
        self.io: Dict = {}
        self.model: Dict = {}
        self.world: Dict = {}
        self.sensors: Dict = {}
        self.photos: Dict = {}
        self.urdf: str = ""
        self.merged: Dict = {}
        self.online = False
        self.stop_evt = threading.Event()
        self.on_state: List[Callable] = []
        self.on_lidar: List[Callable] = []       # cb(name, meta, payload(np))
        self.on_merged: List[Callable] = []
        self.on_camera: List[Callable] = []      # cb(name, info, {stream: ndarray}, meta)
        # 相机拉取策略 NAV_CAMERAS: auto = 有人订阅才拉 (camera_wanted 判定), on = 一直拉, off = 不拉
        self.camera_mode = os.environ.get("NAV_CAMERAS", "auto").strip().lower()
        self.camera_wanted: Optional[Callable] = None   # cb(name, info) -> bool
        self.on_model_change: List[Callable] = []
        self.on_world_change: List[Callable] = []
        self.feedback_fn: Optional[Callable[[], dict]] = None
        self._lidar_threads: Dict[str, threading.Thread] = {}
        self._camera_threads: Dict[str, threading.Thread] = {}
        self.stats = {"state_hz": 0.0, "lidar_frames": {}, "cmd_sent": 0, "errors": 0, "transport": "poll"}
        # 状态/IO/光电: 优先用仿真的推送流 (NAV_SIM_STREAM=0 关闭，或仿真版本不支持时自动回退轮询)
        self.use_stream = os.environ.get("NAV_SIM_STREAM", "1") != "0"
        self.stream_io = False
        self.stream_scans = False
        self.cmd_udp = None           # (host, port)：仿真声明了 UDP 指令通道时 send_cmd 走 UDP
        self.external = False         # 推送流帧由 C++ 核心转发 (feed)
        self._feed_t = 0.0
        self._feed_meta: Dict = {}
        self._feed_n, self._feed_t0 = 0, time.time()

    # ------------------------------------------------------------------
    def start(self):
        for fn, name in ((self._state_loop, "state"), (self._slow_loop, "slow"), (self._merged_loop, "merged"),
                         (self._feedback_loop, "feedback")):
            threading.Thread(target=fn, daemon=True, name=f"simlink-{name}").start()

    def stop(self):
        self.stop_evt.set()

    # ------------------------------------------------------------------ 拉取
    def _handle_state(self, st):
        self.online = True
        prev = self.state
        with self.lock:
            self.state = st
        for cb in self.on_state:
            cb(st)
        if prev and (prev.get("chassis") != st.get("chassis") or prev.get("model_rev") != st.get("model_rev")):
            self.refresh_model()
        if prev and (prev.get("scenario") != st.get("scenario")):
            self.refresh_world()

    def _stream_loop(self) -> bool:
        """推送流；返回 False 表示仿真不支持 (回退轮询)"""
        import urllib.parse
        u = urllib.parse.urlparse(self.url)
        c = HTTPConnection(u.hostname, u.port or 80, timeout=3.0)
        try:
            c.request("GET", u.path.rstrip("/") + "/api/v1/stream?hz=50&io_hz=20&scans=1")
            r = c.getresponse()
            if r.status != 200:
                return False
            self.stats["transport"] = "stream"
            self._feed_meta = {}
            while not self.stop_evt.is_set() and not self.external:
                hdr = r.read(STREAM_HDR.size)
                if len(hdr) < STREAM_HDR.size:
                    break
                ln, typ = STREAM_HDR.unpack(hdr)
                body = r.read(ln)
                if len(body) < ln:
                    break
                self._on_frame(typ, body)
            return True
        except Exception:
            self.stats["errors"] += 1
            return True
        finally:
            if not self.external:
                self.stream_io = self.stream_scans = False
                self.stats["transport"] = "poll"
            try:
                c.close()
            except Exception:
                pass

    def _on_frame(self, typ: int, body: bytes):
        """推送流帧 (自己连接的推送流，或 C++ 核心转发的)"""
        if typ == 1:
            self._handle_state(decode_state(body, self._feed_meta))
            self._feed_n += 1
            if time.time() - self._feed_t0 >= 1.0:
                self.stats["state_hz"] = round(self._feed_n / (time.time() - self._feed_t0), 1)
                self._feed_n, self._feed_t0 = 0, time.time()
        elif typ == 2:
            self._feed_meta = json.loads(body)
        elif typ == 3:
            d = json.loads(body)
            self.stream_io = True
            with self.lock:
                self.io = d.get("io") or {}
                self.photos = {p["name"]: p for p in d.get("photos") or []}
        elif typ == 4:
            self.stream_scans = True
            ml = struct.unpack_from("<H", body, 0)[0]
            m = json.loads(body[2:2 + ml])
            ranges = np.frombuffer(body, dtype="<f4", offset=2 + ml)
            name = m.pop("name")
            if name == "merged":
                m["ranges"] = ranges
                with self.lock:
                    self.merged = m
                for cb in self.on_merged:
                    cb(m)
            else:
                self.stats["lidar_frames"][name] = m.get("seq")
                for cb in self.on_lidar:
                    cb(name, m, {"ranges": ranges})

    def feed(self, typ: int, body: bytes):
        """C++ 核心转发的推送流帧 (cpp_bridge on_relay)"""
        if not self.external:
            self.external = True
            self.log("[simlink] 推送流改由 C++ 核心接收并转发")
        self._feed_t = time.time()
        if typ == 1:
            self.stats["transport"] = "cpp"
            self.stream_io = self.stream_scans = True
        self._on_frame(typ, body)

    def _state_loop(self):
        while self.use_stream and not self.stop_evt.is_set():
            if self.external:                      # C++ 核心转发中: 馈送中断 1.5 s 后自己重新连接
                if time.time() - self._feed_t < 1.5:
                    time.sleep(0.3)
                    continue
                self.external = False
                self.online = False
                self.log("[simlink] C++ 核心转发中断，恢复自己连接推送流")
            if not self._stream_loop():
                self.log("[simlink] 仿真进程不支持推送流，改用轮询")
                break
            self.online = False
            time.sleep(0.5)
        n, t0 = 0, time.time()
        while not self.stop_evt.is_set():
            ts = time.time()
            try:
                st = self.c_state.get("/api/v1/state")
                self._handle_state(st)
                n += 1
            except Exception:
                self.online = False
                self.stats["errors"] += 1
                time.sleep(0.5)
            if time.time() - t0 >= 1.0:
                self.stats["state_hz"] = round(n / (time.time() - t0), 1)
                n, t0 = 0, time.time()
            time.sleep(max(0.0, 0.02 - (time.time() - ts)))

    def _slow_loop(self):
        first = True
        while not self.stop_evt.is_set():
            try:
                if first or not self.model:
                    self.refresh_model()
                    self.refresh_world()
                    first = False
                if not self.stream_io:             # 推送流已带 IO/光电时不再轮询
                    io = self.c_misc.get("/api/v1/io")
                    pe = self.c_misc.get("/api/v1/sensors/photoelectric")     # 光电检测距离 (保护包络判断用)
                    with self.lock:
                        self.io = io
                        self.photos = {p["name"]: p for p in (pe or {}).get("sensors", [])}
            except Exception:
                time.sleep(0.5)
            time.sleep(0.05)   # IO/光电/触边 20 Hz

    def _refresh_cmd_channel(self):
        try:
            import socket
            import urllib.parse
            port = (self.c_misc.get("/api/v1/sim") or {}).get("cmd_udp_port")
            if port and os.environ.get("NAV_CMD_UDP", "1") != "0":
                host = urllib.parse.urlparse(self.url).hostname
                if getattr(self, "_udp", None) is None:
                    self._udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.cmd_udp = (socket.gethostbyname(host), int(port))
            else:
                self.cmd_udp = None
        except Exception:
            self.cmd_udp = None

    def refresh_model(self):
        self._refresh_cmd_channel()
        try:
            m = self.c_misc.get("/api/v1/model")
            s = self.c_misc.get("/api/v1/sensors")
            _, _, u = self.c_misc.request("GET", "/api/v1/model/urdf")
            with self.lock:
                self.model, self.sensors, self.urdf = m, s, u.decode("utf-8")
            self._ensure_lidar_threads()
            self._ensure_camera_threads()
            for cb in self.on_model_change:
                cb(m)
        except Exception as e:
            self.log(f"[simlink] 刷新模型失败: {e}")

    def refresh_world(self):
        try:
            w = self.c_misc.get("/api/v1/world")
            with self.lock:
                self.world = w
            for cb in self.on_world_change:
                cb(w)
        except Exception as e:
            self.log(f"[simlink] 刷新场景失败: {e}")

    def fetch_map(self, out_dir: str, scenario: Optional[str] = None) -> Optional[str]:
        """从仿真进程下载当前场景的 Nav2 栅格地图 (PGM + YAML) → out_dir/<id>.yaml"""
        import os
        try:
            os.makedirs(out_dir, exist_ok=True)
            st, h, pgm = self.c_misc.request("GET", "/api/v1/world/map?part=pgm", timeout=10.0, accept="*/*")
            st2, _, yml = self.c_misc.request("GET", "/api/v1/world/map?part=yaml", timeout=10.0, accept="*/*")
            if st != 200 or st2 != 200:
                return None
            sid = json.loads(h.get("x-meta", "{}")).get("id") or scenario or "map"
            with open(os.path.join(out_dir, f"{sid}.pgm"), "wb") as f:
                f.write(pgm)
            path = os.path.join(out_dir, f"{sid}.yaml")
            with open(path, "wb") as f:
                f.write(yml)
            return path
        except Exception as e:
            self.log(f"[simlink] 下载地图失败: {e}")
            return None

    def _ensure_lidar_threads(self):
        for l in self.sensors.get("lidars", []):
            n = l["name"]
            if n not in self._lidar_threads:
                th = threading.Thread(target=self._lidar_loop, args=(n,), daemon=True, name=f"lidar-{n}")
                self._lidar_threads[n] = th
                th.start()

    def _lidar_loop(self, name: str):
        c = RestClient(self.url, timeout=2.0)
        seq = -1
        is2d = next((l.get("type", "2d") == "2d" for l in self.sensors.get("lidars", []) if l["name"] == name), True)
        while not self.stop_evt.is_set():
            if is2d and self.stream_scans:          # 2D 激光由推送流送达
                time.sleep(0.5)
                continue
            try:
                status, h, body = c.binary(f"/api/v1/sensors/lidars/{name}?after_seq={seq}&wait=0.5")
                if status != 200:
                    time.sleep(0.2)
                    continue
                s = int(h.get("x-seq", seq))
                if s == seq:
                    continue
                seq = s
                meta = json.loads(h.get("x-meta", "{}"))
                if meta.get("type") == "3d":
                    n = int(meta["count"])
                    xyzi = np.frombuffer(body[:n * 16], dtype="<f4").reshape(n, 4)
                    line = np.frombuffer(body[n * 16:n * 17], dtype="u1")
                    payload = {"xyzi": xyzi, "line": line}
                else:
                    payload = {"ranges": np.frombuffer(body, dtype="<f4")}
                self.stats["lidar_frames"][name] = seq
                for cb in self.on_lidar:
                    cb(name, meta, payload)
            except Exception:
                time.sleep(0.5)

    def _ensure_camera_threads(self):
        if not self.on_camera or self.camera_mode in ("off", "0", "false"):
            return      # 无消费者 (未启用 ROS) 或关闭时不拉取图像，节省带宽
        for c in self.sensors.get("camera_streams", []):
            n = c["name"]
            if n not in self._camera_threads:
                th = threading.Thread(target=self._camera_loop, args=(n,), daemon=True, name=f"cam-{n}")
                self._camera_threads[n] = th
                th.start()

    def _camera_loop(self, name: str):
        """相机帧: 以首个流长轮询新帧，同序号再取其余流 (raw 格式，零解码)"""
        c = RestClient(self.url, timeout=3.0)
        seq = -1
        while not self.stop_evt.is_set():
            info = next((x for x in self.sensors.get("camera_streams", []) if x["name"] == name), None)
            if info is None:
                time.sleep(1.0)
                continue
            if self.camera_mode == "auto" and self.camera_wanted is not None:
                try:
                    wanted = self.camera_wanted(name, info)
                except Exception:
                    wanted = True
                if not wanted:      # 没有订阅者: 不拉取 (仿真端相机随之闲置、停止成像)
                    self.stats.setdefault("camera_idle", {})[name] = True
                    time.sleep(0.5)
                    continue
                self.stats.setdefault("camera_idle", {})[name] = False
            try:
                frames, meta0 = {}, None
                for i, st in enumerate(info["streams"]):
                    q = f"/api/v1/sensors/cameras/{name}?stream={st}&format=raw"
                    q += f"&after_seq={seq}&wait=1.0" if i == 0 else ""
                    status, h, body = c.binary(q)
                    if status != 200:
                        raise RuntimeError(status)
                    meta = json.loads(h.get("x-meta", "{}"))
                    if i == 0:
                        s = int(h.get("x-seq", seq))
                        if s == seq:
                            break
                        seq, meta0 = s, meta
                    W, H = meta["width"], meta["height"]
                    if st in ("rgb", "left", "right"):
                        frames[st] = np.frombuffer(body, np.uint8).reshape(H, W, 3)
                    elif st == "depth":
                        frames[st] = np.frombuffer(body, "<f4").reshape(H, W)
                    elif st == "amplitude":
                        frames[st] = np.frombuffer(body, "<u2").reshape(H, W)
                    else:
                        frames[st] = np.frombuffer(body, "<f4").reshape(-1, 3)
                if frames:
                    self.stats.setdefault("camera_frames", {})[name] = seq
                    for cb in self.on_camera:
                        cb(name, info, frames, meta0)
            except Exception:
                time.sleep(0.5)

    def _merged_loop(self):
        """融合扫描: 二进制帧 (float32 ranges，inf = 无回波)，省掉 JSON 编解码；d["ranges"] 为 np.ndarray"""
        c = RestClient(self.url, timeout=2.0)
        seq = -1
        while not self.stop_evt.is_set():
            if self.stream_scans:                   # 融合扫描由推送流送达
                time.sleep(0.5)
                continue
            try:
                status, h, body = c.binary(f"/api/v1/sensors/scan?after_seq={seq}&wait=0.5")
                if status != 200:
                    time.sleep(0.2)
                    continue
                s = int(h.get("x-seq", seq))
                if s == seq:
                    continue
                seq = s
                d = json.loads(h.get("x-meta", "{}"))
                d["ranges"] = np.frombuffer(body, dtype="<f4")
                with self.lock:
                    self.merged = d
                for cb in self.on_merged:
                    cb(d)
            except Exception:
                time.sleep(0.5)

    # ------------------------------------------------------------------ 回馈
    def send_cmd(self, vx: float, vy: float = 0.0, wz: float = 0.0, source: str = "nav") -> bool:
        if self.cmd_udp is not None:
            try:
                self._cmd_seq = getattr(self, "_cmd_seq", 0) + 1
                self._udp.sendto(struct.pack("<4sI3d16s", b"AGVC", self._cmd_seq & 0xFFFFFFFF, float(vx), float(vy), float(wz),
                                             source.encode()[:16]), self.cmd_udp)
                self.stats["cmd_sent"] += 1
                return True
            except OSError:
                self.stats["errors"] += 1
                self.cmd_udp = None           # 回退 HTTP
        try:
            self.c_cmd.put("/api/v1/control/cmd_vel", {"vx": float(vx), "vy": float(vy), "wz": float(wz), "source": source})
            self.stats["cmd_sent"] += 1
            return True
        except Exception:
            self.stats["errors"] += 1
            return False

    def _feedback_loop(self):
        c = RestClient(self.url, timeout=1.0)
        while not self.stop_evt.is_set():
            if self.feedback_fn:
                try:
                    c.put("/api/v1/nav/feedback", self.feedback_fn())
                except Exception:
                    pass
            time.sleep(0.2)
