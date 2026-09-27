#!/usr/bin/env python3
"""
SimLink —— 执行进程侧的仿真数据接入层 (纯 REST，替代 ROS 话题订阅)

  state 轮询     GET /api/v1/state            50 Hz  (真值/里程计/关节/碰撞)
  io 轮询        GET /api/v1/io + /sensors/photoelectric  20 Hz  (急停/触边/光电 DI 与检测距离 → 执行进程安全层)
  激光长轮询      GET /api/v1/sensors/lidars/{name}?after_seq=N&wait=0.5  (二进制，每帧恰好取一次)
  融合扫描长轮询  GET /api/v1/sensors/scan?after_seq=N&wait=0.5  (二进制)
  模型/场景      GET /api/v1/model, /api/v1/world (车型/场景变化时刷新)
  指令回馈       PUT /api/v1/control/cmd_vel
  状态回馈       PUT /api/v1/nav/feedback     5 Hz
"""

import json
import math
import os
import threading
import time
from typing import Callable, Dict, List, Optional

import numpy as np

from common.rest import RestClient


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
        self.stats = {"state_hz": 0.0, "lidar_frames": {}, "cmd_sent": 0, "errors": 0}

    # ------------------------------------------------------------------
    def start(self):
        for fn, name in ((self._state_loop, "state"), (self._slow_loop, "slow"), (self._merged_loop, "merged"),
                         (self._feedback_loop, "feedback")):
            threading.Thread(target=fn, daemon=True, name=f"simlink-{name}").start()

    def stop(self):
        self.stop_evt.set()

    # ------------------------------------------------------------------ 拉取
    def _state_loop(self):
        n, t0 = 0, time.time()
        while not self.stop_evt.is_set():
            ts = time.time()
            try:
                st = self.c_state.get("/api/v1/state")
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
                io = self.c_misc.get("/api/v1/io")
                pe = self.c_misc.get("/api/v1/sensors/photoelectric")     # 光电检测距离 (保护包络判断用)
                with self.lock:
                    self.io = io
                    self.photos = {p["name"]: p for p in (pe or {}).get("sensors", [])}
            except Exception:
                time.sleep(0.5)
            time.sleep(0.05)   # IO/光电/触边 20 Hz

    def refresh_model(self):
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
        while not self.stop_evt.is_set():
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
