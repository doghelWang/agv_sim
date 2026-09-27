#!/usr/bin/env python3
"""
SimService —— 仿真进程的核心服务 (不依赖 ROS)

职责: 模型解析与构建 (cmodel → robot spec → 运动学/传感器/URDF) + 世界构建 (场景/障碍物)
      + 物理步进 + 仿真数据产生 (激光 2D/3D、IMU、编码器里程计、读码、光电/IO、防撞触边/碰撞)
      + 接收执行进程回馈 (cmd_vel、导航状态)

线程模型
  physics 线程: 固定步长实时推进 (默认 10 ms)
  sensor  线程: 按各激光频率扫描，写入带序号的缓冲区；Condition 通知长轮询请求
  HTTP 线程池: 只读快照/写指令 (全部经 self.lock 串行化访问 SimCore)
"""

import json
import math
import os
import struct
import sys
import threading
import time
from collections import deque
from typing import Any, Dict, List, Optional

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from planning.dijkstra_planner import SCENARIO_DEFINITIONS, DijkstraPlanner, register_scenario  # noqa: E402
from sim_core.engine import SimCore  # noqa: E402

API_VERSION = "1.0"


def _upgrade_spec(spec: dict) -> dict:
    """兼容旧版 robot_config.json"""
    if spec.get("schema_version", 1) >= 2:
        return spec
    ch = spec.setdefault("chassis", {})
    h, t = ch.get("head_offset_m", 0.6), ch.get("tail_offset_m", 0.6)
    l, r = ch.get("left_offset_m", 0.4), ch.get("right_offset_m", 0.4)
    ch.setdefault("footprint", [[h, l], [h, -r], [-t, -r], [-t, l]])
    ch["type"] = {"diff": "diff_drive"}.get(ch.get("type", "diff_drive"), ch.get("type", "diff_drive"))
    ch.setdefault("mass_kg", 200.0)
    spec.setdefault("wheels", [])
    return spec


def ranges_json(v: np.ndarray, nd: int = 4) -> list:
    """测距数组 → JSON 列表 (无回波 = None)。向量化: round/tolist 在 C 里做，只对少量无回波点逐个替换"""
    v = np.asarray(v, dtype=np.float64)
    fin = np.isfinite(v)
    out = np.round(np.where(fin, v, 0.0), nd).tolist()
    for i in np.flatnonzero(~fin).tolist():
        out[i] = None
    return out


# ---------------------------------------------------------------------------- 状态推送流 (/api/v1/stream)
# 帧 = <u32 负载长度><u8 类型><负载>；类型: 1 状态 (二进制，见 STATE_FMT)，2 元信息 JSON (车型/场景/关节名，变化时)，
# 3 IO+光电 JSON (20 Hz)，4 2D 激光/融合扫描 (<u16 元信息长度><元信息 JSON><float32 ranges>，有新帧时)。状态字段与 GET /api/v1/state 相同 (nav_runtime/sim_link.decode_state 还原成同样的 dict)
STREAM_HDR = struct.Struct("<IB")
STATE_FMT = struct.Struct("<Idd6d6d3d5dIBddIH")
F_BUMPER_FRONT, F_BUMPER_REAR, F_PAUSED, F_CONTACT = 1, 2, 4, 8


class ScanBuffer:
    def __init__(self):
        self.seq = 0
        self.data: Dict[str, Any] = {}


class SimService:
    def __init__(self, config_path: str, scenario: str = "grid_9_square"):
        self.config_path = config_path
        self.lock = threading.RLock()
        self.cond = threading.Condition(self.lock)
        self.events: deque = deque(maxlen=500)
        self.event_id = 0
        self.nav_feedback: Dict[str, Any] = {"status": "OFFLINE", "t_wall": 0.0}
        self.cmd_meta = {"source": "none", "t_wall": 0.0, "vx": 0.0, "vy": 0.0, "wz": 0.0}
        self.lidar_bufs: Dict[str, ScanBuffer] = {}
        self.merged = ScanBuffer()
        self.state_seq = 0
        self.rtf_target = 1.0
        self._stop = threading.Event()
        self.merged_hz = 10.0
        self.lidar_max_hz = float(os.environ.get("SIM_LIDAR_MAX_HZ", "10"))
        self.camera_max_hz = float(os.environ.get("SIM_CAMERA_MAX_HZ", "10"))
        # 相机按需成像: 最近 SIM_CAMERA_IDLE_S 秒内没有人取帧就不渲染 (0 = 一直渲染，旧行为)
        self.camera_idle_s = float(os.environ.get("SIM_CAMERA_IDLE_S", "3"))
        self._cam_req: Dict[str, float] = {}
        self.cam_bufs: Dict[str, ScanBuffer] = {}
        # 模型 = 基线 (cmodel 解析, robot_config.base.json) + 人工补全 (model_overrides.json)
        from model_overrides import apply_overrides, load_overrides
        self.base_path = os.path.join(os.path.dirname(os.path.abspath(config_path)), "robot_config.base.json")
        self.overrides_path = os.path.join(os.path.dirname(os.path.abspath(config_path)), "model_overrides.json")
        if os.path.exists(self.base_path):
            with open(self.base_path, "r", encoding="utf-8") as f:
                base = _upgrade_spec(json.load(f))
        else:
            with open(config_path, "r", encoding="utf-8") as f:
                base = _upgrade_spec(json.load(f))
            with open(self.base_path, "w", encoding="utf-8") as f:
                json.dump(base, f, indent=2, ensure_ascii=False)
        self.base_spec = base
        self.overrides = load_overrides(self.overrides_path)
        spec = apply_overrides(base, self.overrides)
        try:   # 平台下发的模型: robot_config.json 带 repo {model_id, version, name}
            with open(config_path, "r", encoding="utf-8") as f:
                repo = json.load(f).get("repo")
            if repo:
                spec["repo"] = repo
        except Exception:
            pass
        self._build(spec, scenario, None)

    # ================================================================== 构建
    def _build(self, spec: dict, scenario: str, chassis: Optional[str]):
        self.model_rev = getattr(self, "model_rev", 0) + 1
        self.spec = spec
        old = getattr(self, "core", None)
        if old is not None and old.rt is not None:     # 旧的原生实时线程先停
            old.rt.stop()
        self.core = SimCore(spec, SCENARIO_DEFINITIONS, scenario,
                            backend=os.environ.get("SIM_PHYSICS", "mujoco"),
                            dt=float(os.environ.get("SIM_DT", "0.01")),
                            noise=os.environ.get("SIM_NOISE", "1") == "1", chassis_type=chassis)
        self.core.lidar_max_hz, self.core.merged_hz = self.lidar_max_hz, self.merged_hz
        if self.core.enable_rt() and getattr(self, "_started", False):
            self.core.rt.start()
            if getattr(self, "cmd_udp", None) == "native":
                self.core.rt.udp_retarget()
        self.planner = DijkstraPlanner(scenario)
        self.lidar_bufs = {l.name: ScanBuffer() for l in self.core.lidars + self.core.lidars3d}
        self._next_scan = {n: 0.0 for n in self.lidar_bufs}
        self._next_merged = 0.0
        # 相机类 (单目/双目/ToF): 独立缓冲 + 独立线程 (成像耗时长，不阻塞物理/激光)
        old = getattr(self, "cam_bufs", {})
        self.cam_bufs = {c.name: old.get(c.name, ScanBuffer()) for c in self.core.cameras}
        self._next_cam = {c.name: 0.0 for c in self.core.cameras}

    # ================================================================== UDP 速度指令通道
    def start_cmd_udp(self, port: int):
        """执行进程 → 仿真的速度指令走 UDP (同号端口)；C 实时循环在时由原生线程直接写入指令，否则 Python 线程接收。
        SIM_CMD_UDP=0 关闭 (执行进程随之继续用 PUT /api/v1/control/cmd_vel)"""
        if os.environ.get("SIM_CMD_UDP", "1") == "0":
            return
        host = os.environ.get("AGV_BIND", "")
        host = "" if host in ("", "0.0.0.0") else host
        if self.core.rt is not None and self.core.rt.udp_start(host, port) == 0:
            self.cmd_udp, self.cmd_udp_port = "native", port
            return
        import socket
        try:
            sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sk.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sk.bind((host, port))
        except OSError as e:
            print(f"[sim] UDP 指令通道未启用: {e}", flush=True)
            return
        self.cmd_udp, self.cmd_udp_port = "python", port

        def loop():
            while not self._stop.is_set():
                try:
                    b = sk.recv(128)
                except OSError:
                    continue
                if len(b) >= 48 and b[:4] == b"AGVC":
                    vx, vy, wz = struct.unpack_from("<3d", b, 8)
                    src = b[32:48].split(b"\0", 1)[0].decode("utf-8", "replace")
                    if all(math.isfinite(v) for v in (vx, vy, wz)):
                        self.set_cmd(vx, vy, wz, src or "udp")
        threading.Thread(target=loop, daemon=True, name="cmd-udp").start()

    def emit(self, kind: str, level: str, message: str, data: Optional[dict] = None):
        with self.lock:
            self.event_id += 1
            self.events.append({"id": self.event_id, "t": round(self.core.t, 3), "wall": time.time(), "type": kind,
                                "level": level, "message": message, "data": data or {}})

    # ================================================================== 线程
    def start(self):
        self._started = True
        if self.core.rt is not None:
            self.core.rt.start()
        threading.Thread(target=self._physics_loop, daemon=True, name="physics").start()
        threading.Thread(target=self._sensor_loop, daemon=True, name="sensors").start()
        threading.Thread(target=self._camera_loop, daemon=True, name="cameras").start()
        self.emit("SIM_START", "info", f"仿真进程启动: 车型 {self.core.chassis_type}, 场景 {self.core.scenario_id}, "
                                        f"激光 {[l.name for l in self.core.lidars + self.core.lidars3d]}")

    def stop(self):
        self._stop.set()
        if self.core.rt is not None:
            self.core.rt.stop()

    def _physics_loop(self):
        last = time.perf_counter()
        last_coll = 0
        self._last_discrete = self._discrete_sig()
        last_t = -1.0
        while not self._stop.is_set():
            if self.core.rt is not None:
                # 原生实时循环在 C 线程里推进物理；这里 100 Hz 做管家: 同步快照、急停/暂停/倍速、IO、行人、事件
                time.sleep(0.01)
                with self.lock:
                    core = self.core
                    if core.rt is None:
                        continue
                    self._rt_housekeeping(core)
                    if core.t != last_t:
                        last_t = core.t
                        self.state_seq += 1
                    last = time.perf_counter()
                    ds = self._discrete_sig()
                    if ds != self._last_discrete:
                        self._discrete_events(self._last_discrete, ds)
                        self._last_discrete = ds
                    if core.collisions != last_coll:
                        last_coll = core.collisions
                        self.emit("COLLISION", "danger", f"车体碰撞 (累计 {last_coll} 次)，防撞触边触发",
                                  {"contact": core.last_contact, "count": last_coll})
                continue
            dt = self.core.dt
            time.sleep(dt * 0.5)          # 累加器保证物理时间精确，睡眠只决定调度粒度
            now = time.perf_counter()
            with self.lock:
                n = self.core.advance(min(0.25, now - last), self.rtf_target)
                last = now
                if n:
                    self.state_seq += 1
                ds = self._discrete_sig()
                if ds != self._last_discrete:
                    self._discrete_events(self._last_discrete, ds)
                    self._last_discrete = ds
                if self.core.collisions != last_coll:
                    last_coll = self.core.collisions
                    self.emit("COLLISION", "danger", f"车体碰撞 (累计 {last_coll} 次)，防撞触边触发",
                              {"contact": self.core.last_contact, "count": last_coll})

    def _rt_housekeeping(self, core):
        """(持 self.lock) 拉取 C 实时循环快照 → DI；急停/暂停/倍速下发；行人 20 Hz；顶升机构"""
        rt = core.rt
        with rt.hold(sync=False):
            rt.pull()
            rt.set_flags(core.paused, core.estop, self.rtf_target)
        for b in core.bumpers:
            if core.io.di.get(b.di) != b.pressed:
                core.io.set_di(b.di, b.pressed)
        for p in core.photos:
            if core.io.di.get(p.di) != p.detected:
                core.io.set_di(p.di, p.detected)
        prev = getattr(self, "_hk_t", core.t)
        self._hk_t = core.t
        if core.t > prev:
            core.io.update_lift_physics(core.t - prev)
            core._mover_acc = getattr(core, "_mover_acc", 0.0) + (core.t - prev)
            if core._mover_acc >= 0.05:
                core._mover_acc = 0.0
                if any(o.get("motion") for o in core.world.obstacles):
                    core.update_movers()

    def _rt_sensor_poll(self, core):
        """(持 self.lock) 从 C 双缓冲取新的 2D 激光/融合帧"""
        rt = core.rt
        got = False
        last = getattr(rt, "_read_seq", None)
        if last is None:
            last = rt._read_seq = {}            # 已读的 C 侧帧序号 (缓冲序号自身保持单调递增，重建模型后不回退)
        with rt.hold(sync=False):
            for i, l in enumerate(core.lidars):
                b = self.lidar_bufs[l.name]
                r = rt.read_lidar(i, last.get(i, 0))
                if r is None:
                    continue
                cseq, t, (px, py, pth), ranges = r
                last[i] = cseq
                hz = min(l.freq_hz, self.lidar_max_hz) if self.lidar_max_hz > 0 else l.freq_hz
                b.seq += 1
                seq = b.seq
                b.data = {"type": "2d", "seq": seq, "t": round(t, 4), "frame_id": l.frame_id,
                          "pose": {"x": round(px, 4), "y": round(py, 4), "yaw": round(pth, 5)},
                          "angle_min": l.angle_min, "angle_increment": l.angle_inc, "range_min": l.range_min,
                          "range_max": l.range_max, "scan_hz": hz, "ranges": ranges}
                got = True
            r = rt.read_lidar(-1, last.get(-1, 0))
            if r is not None:
                cseq, t, (px, py, pth), merged = r
                last[-1] = cseq
                n = len(merged)
                self.merged.seq += 1
                seq = self.merged.seq
                self.merged.data = {"seq": seq, "t": round(t, 4), "frame_id": "base_link",
                                    "pose": {"x": round(px, 4), "y": round(py, 4), "yaw": round(pth, 5)},
                                    "angle_min": -math.pi + math.pi / n, "angle_increment": 2 * math.pi / n, "range_min": 0.05,
                                    "range_max": core.merged_range, "scan_hz": self.merged_hz, "ranges": merged}
                got = True
        if got:
            self.cond.notify_all()

    def _sensor_loop(self):
        while not self._stop.is_set():
            time.sleep(0.01)
            with self.lock:
                if self.core.rt is not None and self.core.rt.lidars_on:
                    self._rt_sensor_poll(self.core)
                    continue
                t = self.core.t
                due = [n for n, nt in self._next_scan.items() if t >= nt]
                mdue = t >= self._next_merged
                if not due and not mdue:
                    continue
                rs, merged = self.core.scan_all()
                pose = {"x": round(self.core.x, 4), "y": round(self.core.y, 4), "yaw": round(self.core.th, 5)}
                for l, r in zip(self.core.lidars, rs):
                    if l.name in due:
                        hz = min(l.freq_hz, self.lidar_max_hz) if self.lidar_max_hz > 0 else l.freq_hz
                        self._next_scan[l.name] = t + 1.0 / max(1.0, hz)
                        b = self.lidar_bufs[l.name]
                        b.seq += 1
                        b.data = {"type": "2d", "seq": b.seq, "t": round(t, 4), "frame_id": l.frame_id, "pose": pose,
                                  "angle_min": l.angle_min, "angle_increment": l.angle_inc, "range_min": l.range_min,
                                  "range_max": l.range_max, "scan_hz": hz, "ranges": r}
                for l3, cl, sl in self.core.last_clouds:
                    if l3.name in due:
                        self._next_scan[l3.name] = t + 1.0 / max(1.0, l3.freq_hz)
                        b = self.lidar_bufs[l3.name]
                        b.seq += 1
                        b.data = {"type": "3d", "seq": b.seq, "t": round(t, 4), "frame_id": l3.frame_id, "pose": pose,
                                  "scan_hz": l3.freq_hz, "points": cl["points"], "intensity": cl["intensity"],
                                  "line": cl["line"], "offset_time": cl["offset_time"], "slice": sl}
                if mdue:
                    self._next_merged = t + 1.0 / self.merged_hz
                    self.merged.seq += 1
                    n = len(merged)
                    self.merged.data = {"seq": self.merged.seq, "t": round(t, 4), "frame_id": "base_link", "pose": pose,
                                        "angle_min": -math.pi + math.pi / n, "angle_increment": 2 * math.pi / n, "range_min": 0.05,
                                        "range_max": self.core.merged_range, "scan_hz": self.merged_hz, "ranges": merged}
                self.cond.notify_all()

    def _camera_loop(self):
        while not self._stop.is_set():
            time.sleep(0.005)
            with self.lock:
                core = self.core
                t = core.t
                now = time.time()
                due = [c for c in core.cameras if t >= self._next_cam.get(c.name, 0.0) and self._cam_wanted(c.name, now)]
                pose = (core.x, core.y, core.th)
                for c in due:
                    hz = min(c.fps, self.camera_max_hz) if self.camera_max_hz > 0 else c.fps
                    self._next_cam[c.name] = t + 1.0 / max(0.2, hz)
            for c in due:          # 成像在锁外进行 (MuJoCo 射线用独立 MjData，不与物理步进互斥)
                try:
                    t0 = time.perf_counter()
                    out = c.capture(core.world, core.mj, pose[0], pose[1], pose[2], noise=core.noise)
                    c.last_ms = 0.8 * c.last_ms + 0.2 * (time.perf_counter() - t0) * 1000.0
                except Exception as e:  # pragma: no cover
                    self.emit("CAMERA_ERROR", "warning", f"相机 {c.name} 成像失败: {e}")
                    continue
                with self.cond:
                    b = self.cam_bufs.get(c.name)
                    if b is None:
                        continue
                    b.seq += 1
                    b.data = {"seq": b.seq, "t": round(t, 4), "pose": {"x": pose[0], "y": pose[1], "yaw": pose[2]},
                              "info": c.info(), "frames": out}
                    self.cond.notify_all()

    # ================================================================== 相机
    def _cam_wanted(self, name: str, now: float) -> bool:
        return self.camera_idle_s <= 0 or now - self._cam_req.get(name, 0.0) <= self.camera_idle_s

    def cameras(self) -> list:
        now = time.time()
        with self.lock:
            return [dict(c.info(), seq=self.cam_bufs[c.name].seq, capture_ms=round(c.last_ms, 1),
                         active=self._cam_wanted(c.name, now),
                         render=("gl" if getattr(c, "use_gl", False) else "ray")) for c in self.core.cameras]

    def wait_camera(self, name: str, after_seq: int, wait: float) -> Optional[dict]:
        if name not in self.cam_bufs:
            raise KeyError(name)
        buf = self.cam_bufs[name]
        now = time.time()
        if not self._cam_wanted(name, now):
            # 相机闲置过 (缓冲里是旧帧): 唤醒后等一帧新的，最多 wait 秒 (无 wait 时至少等 1 s)
            after_seq = max(after_seq, buf.seq)
            wait = max(wait, 1.0)
        self._cam_req[name] = now
        deadline = time.time() + max(0.0, min(wait, 5.0))
        with self.cond:
            while (buf.seq <= after_seq or not buf.data) and time.time() < deadline:
                self.cond.wait(timeout=max(0.001, deadline - time.time()))
            return dict(buf.data) if buf.data else None

    # ================================================================== 模型补全 (人工)
    def model_editor(self) -> dict:
        from model_overrides import SENSOR_TYPES, audit, sensor_list
        with self.lock:
            photos = [p.view() for p in self.core.photos]
            return {"spec": self.spec, "base_file": os.path.basename(self.base_path), "overrides": self.overrides,
                    "audit": audit(self.spec, self.overrides), "sensors": sensor_list(self.spec, photos),
                    "sensor_types": SENSOR_TYPES}

    def preview_overrides(self, ov: dict) -> dict:
        from cmodel_parser import generate_urdf
        from model_overrides import apply_overrides, audit, sensor_list
        sp = apply_overrides(self.base_spec, ov)
        return {"spec": sp, "audit": audit(sp, ov), "sensors": sensor_list(sp), "urdf": generate_urdf(sp)}

    def apply_overrides(self, ov: dict, save: bool = True) -> dict:
        """保存 model_overrides.json → 生成 robot_config.json / robot.urdf → 重建仿真 (保留场景与位姿)"""
        from cmodel_parser import generate_urdf
        from model_overrides import apply_overrides, save_overrides
        ov = dict(ov)
        ov.setdefault("version", 1)
        sp = apply_overrides(self.base_spec, ov)
        if self.spec.get("repo"):
            sp["repo"] = self.spec["repo"]
        if save:
            save_overrides(ov, self.overrides_path)
            with open(self.config_path, "w", encoding="utf-8") as f:
                json.dump(sp, f, indent=2, ensure_ascii=False)
            with open(os.path.join(os.path.dirname(os.path.abspath(self.config_path)), "robot.urdf"), "w", encoding="utf-8") as f:
                f.write(generate_urdf(sp))
        with self.lock:
            pose = (self.core.x, self.core.y, self.core.th)
            sc, ct, obs = self.core.scenario_id, self.core.chassis_type, list(self.core.world.obstacles)
            self.overrides = ov
            self._build(sp, sc, ct if ct != sp["chassis"]["type"] else None)
            self.core.set_obstacles(obs)
            self.core.reset_pose(*pose)
        self.emit("MODEL_APPLY", "success", f"模型补全已应用: {len(sp.get('provenance', {}))} 项人工参数，"
                                            f"传感器 {len(sp.get('lidars', []))} 激光 / {len(sp.get('cameras', []))} 相机")
        return self.model_editor()

    # ================================================================== 模型
    def model(self) -> dict:
        with self.lock:
            sp = dict(self.spec)
            sp["active_chassis"] = self.core.chassis_type
            sp["active_wheels"] = self.core.wheels_d
            sp["model_rev"] = self.model_rev
            # 各车型 Nav2 可用限速 (底盘设定 ∩ 轮端能力)，执行进程生成 Nav2 参数时直接使用
            try:
                from tools.gen_nav2_params import CHASSIS_TYPES, effective_limits
                sp["nav_limits"] = {ct: effective_limits(self.spec, ct) for ct in CHASSIS_TYPES}
            except Exception:
                pass
            return sp

    def urdf(self) -> str:
        with self.lock:
            return self.core.robot_urdf()

    def set_chassis(self, ctype: Optional[str]) -> str:
        with self.lock:
            ct = self.core.set_chassis(None if ctype in (None, "", "cmodel") else ctype)
        self.emit("CHASSIS", "info", f"车型切换 → {ct}", {"chassis": ct})
        return ct

    def reload_model(self, cmodel_path: Optional[str] = None, load: str = "full", save: bool = True) -> dict:
        """重新解析 cmodel (或重读 robot_config.json) 并重建仿真 (保留场景)"""
        from cmodel_parser import generate_urdf, parse_cmodel_file
        from model_overrides import apply_overrides
        if cmodel_path:
            base = parse_cmodel_file(cmodel_path, load=load)
            base["model_path"] = os.path.abspath(cmodel_path)
            if save:
                with open(self.base_path, "w", encoding="utf-8") as f:
                    json.dump(base, f, indent=2, ensure_ascii=False)
            self.base_spec = base
        spec = apply_overrides(self.base_spec, self.overrides)
        if save:
            with open(self.config_path, "w", encoding="utf-8") as f:
                json.dump(spec, f, indent=2, ensure_ascii=False)
            with open(os.path.join(os.path.dirname(os.path.abspath(self.config_path)), "robot.urdf"), "w", encoding="utf-8") as f:
                f.write(generate_urdf(spec))
        with self.lock:
            sc = self.core.scenario_id
            self._build(spec, sc, None)
        self.emit("MODEL_RELOAD", "success", f"模型重建完成: {spec.get('model_file')} ({spec['chassis']['type']})")
        return self.model()

    # ================================================================== 世界
    def world(self) -> dict:
        with self.lock:
            self.planner.set_scenario(self.core.scenario_id)
            meta = self.planner.get_scenario_metadata()
            meta["topology"] = self.planner.get_topology()
            meta["obstacles"] = self.core.world.obstacles
            meta["heights"] = {"wall": 6.0, "shelf": 2.5, "ceiling": 6.0}
            # 原始场景定义: 执行进程据此构建拓扑规划器 (不依赖本地场景库)
            sc = SCENARIO_DEFINITIONS[self.core.scenario_id]
            meta["scenario_def"] = {k: ([list(c) for c in v] if k in ("walls", "connections") else
                                        ({n: list(xy) for n, xy in v.items()} if k == "nodes" else v))
                                    for k, v in sc.items()}
            return meta

    def occupancy_map(self, res: float = 0.05) -> dict:
        """Nav2 map_server 用栅格地图 (PGM + YAML): 场景包自带栅格 (平台下发) 优先，否则按场景几何生成"""
        from tools.scenario_to_map import rasterize
        sid = self.core.scenario_id
        mf = SCENARIO_DEFINITIONS[sid].get("map_files") or {}
        if mf.get("pgm") and os.path.exists(mf["pgm"]) and os.path.exists(mf.get("yaml", "")):
            import re
            with open(mf["pgm"], "rb") as f:
                pgm = f.read()
            with open(mf["yaml"], encoding="utf-8") as f:
                yml = f.read()
            yml = re.sub(r"^image:.*$", f"image: {sid}.pgm", yml, flags=re.M)
            m_res = re.search(r"^resolution:\s*([\d.eE+-]+)", yml, re.M)
            m_org = re.search(r"^origin:\s*\[([^\]]*)\]", yml, re.M)
            try:
                hdr = [t for t in pgm[:200].split() if not t.startswith(b"#")]
                W, H = int(hdr[1]), int(hdr[2])
            except Exception:
                W = H = 0
            org = [float(x) for x in m_org.group(1).split(",")[:2]] if m_org else [0.0, 0.0]
            return {"id": sid, "pgm": pgm, "yaml": yml, "width": W, "height": H,
                    "resolution": float(m_res.group(1)) if m_res else res, "origin": org}
        img, W, H, (ox, oy) = rasterize(SCENARIO_DEFINITIONS[sid], res)
        pgm = f"P5\n# scenario {sid}\n{W} {H}\n255\n".encode() + bytes(img)
        yml = (f"image: {sid}.pgm\nmode: trinary\nresolution: {res}\norigin: [{ox:.3f}, {oy:.3f}, 0.0]\n"
               f"negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
        return {"id": sid, "pgm": pgm, "yaml": yml, "width": W, "height": H, "resolution": res, "origin": [ox, oy]}

    def load_scene(self, sc: dict) -> str:
        """登记外部场景定义 (平台场景包) 并切换过去"""
        sid = register_scenario(sc)
        self.set_scenario(sid)
        return sid

    def load_model_from_hub(self, hub: str, mid: str, ver: str = "") -> dict:
        """从平台拉取模型包 (基线+补全) → 重建仿真 (保留场景与位姿)"""
        from model_overrides import apply_overrides, load_overrides
        from sim_server.bootstrap import pull_model
        data = os.path.dirname(os.path.abspath(self.config_path))
        pull_model(hub, data, mid, ver)
        with open(self.base_path, encoding="utf-8") as f:
            self.base_spec = _upgrade_spec(json.load(f))
        self.overrides = load_overrides(self.overrides_path)
        spec = apply_overrides(self.base_spec, self.overrides)
        with open(self.config_path, encoding="utf-8") as f:
            spec["repo"] = json.load(f).get("repo")
        with self.lock:
            pose = (self.core.x, self.core.y, self.core.th)
            sc, obs = self.core.scenario_id, list(self.core.world.obstacles)
            self._build(spec, sc, None)
            self.core.set_obstacles(obs)
            self.core.reset_pose(*pose)
        self.emit("MODEL_RELOAD", "success", f"已加载平台模型 {mid} {ver or ''}")
        return self.model()

    def instance(self) -> dict:
        sc = SCENARIO_DEFINITIONS.get(self.core.scenario_id, {})
        return {"instance_id": os.environ.get("INSTANCE_ID"), "instance_name": os.environ.get("INSTANCE_NAME"),
                "hub": os.environ.get("HUB_API"), "model": dict(self.spec.get("repo") or {}, file=self.spec.get("model_file"),
                                                                   chassis=self.core.chassis_type),
                "scene": {"id": sc.get("id"), "name": sc.get("name")}, "model_rev": self.model_rev}

    def set_scenario(self, sid: str):
        if sid not in SCENARIO_DEFINITIONS:
            raise KeyError(sid)
        with self.lock:
            self.core.set_scenario(sid)
            self.core.cmd = (0.0, 0.0, 0.0)
        self.emit("SCENARIO", "info", f"场景切换 → {sid}", {"scenario": sid})

    def set_obstacles(self, obs: List[dict]):
        with self.lock:
            self.core.set_obstacles(obs)
        self.emit("OBSTACLES", "info", f"动态障碍物更新: {len(obs)} 个", {"count": len(obs)})

    # ================================================================== 状态
    def state(self) -> dict:
        with self.lock:
            c = self.core
            c.rt_pull()
            names, pos, vel, eff = c.kin.joint_state()
            mx, my, mth = c.map_to_odom()
            return {
                "seq": self.state_seq, "t": round(c.t, 4), "wall_time": time.time(),
                "truth": {"frame": "map", "x": c.x, "y": c.y, "yaw": c.th, "vx": c.vx, "vy": c.vy, "wz": c.wz},
                "odom": {"frame": "odom", "x": c.odom.x, "y": c.odom.y, "yaw": c.odom.th, "vx": c.odom.vx, "vy": c.odom.vy, "wz": c.odom.wz},
                "map_to_odom": {"x": mx, "y": my, "yaw": mth},
                "imu": {k: round(float(v), 6) for k, v in (c.imu_sample or {}).items()},
                "joints": {"names": names, "position": pos, "velocity": vel, "effort": eff},
                "collision": {"count": c.collisions, "bumper_front": c.io.di.get("di_bumper_front", False),
                              "bumper_rear": c.io.di.get("di_bumper_rear", False), "last_contact": c.last_contact},
                "paused": c.paused, "chassis": c.chassis_type, "scenario": c.scenario_id, "model_rev": self.model_rev,
            }

    def state_frames(self, last_meta):
        """推送流: (状态帧 bytes, 元信息帧 bytes|None, 当前元信息键)"""
        with self.lock:
            c = self.core
            c.rt_pull()
            names, pos, vel, eff = c.kin.joint_state()
            mx, my, mth = c.map_to_odom()
            im = c.imu_sample or {}
            fl = ((F_BUMPER_FRONT if c.io.di.get("di_bumper_front", False) else 0) | (F_BUMPER_REAR if c.io.di.get("di_bumper_rear", False) else 0)
                  | (F_PAUSED if c.paused else 0) | (F_CONTACT if c.last_contact else 0))
            lc = c.last_contact or (0.0, 0.0)
            od = c.odom
            body = STATE_FMT.pack(self.state_seq, c.t, time.time(), c.x, c.y, c.th, c.vx, c.vy, c.wz,
                                  od.x, od.y, od.th, od.vx, od.vy, od.wz, mx, my, mth,
                                  float(im.get("wz", 0.0)), float(im.get("ax", 0.0)), float(im.get("ay", 0.0)),
                                  float(im.get("az", 9.81)), float(im.get("yaw", 0.0)),
                                  c.collisions, fl, float(lc[0]), float(lc[1]), self.model_rev, len(names))
            body += struct.pack(f"<{3 * len(names)}d", *pos, *vel, *eff)
            key = (c.chassis_type, c.scenario_id, tuple(names))
        meta = None
        if key != last_meta:
            m = json.dumps({"chassis": key[0], "scenario": key[1], "joint_names": list(key[2])}, ensure_ascii=False).encode()
            meta = STREAM_HDR.pack(len(m), 2) + m
        return STREAM_HDR.pack(len(body), 1) + body, meta, key

    def scan_frames(self, sent: dict) -> bytes:
        """推送流: 自上次发送后有新帧的 2D 激光与融合扫描 (sent: 名称 → 已发送的帧序号，就地更新)"""
        out = []
        with self.lock:
            items = [(n, b) for n, b in self.lidar_bufs.items() if b.data and b.data.get("type") == "2d"] + [("merged", self.merged)]
            for name, b in items:
                d = b.data
                if not d or b.seq <= sent.get(name, 0):
                    continue
                sent[name] = b.seq
                meta = {k: v for k, v in d.items() if not isinstance(v, np.ndarray)}
                meta["name"] = name
                mj = json.dumps(meta, ensure_ascii=False, separators=(",", ":")).encode()
                body = struct.pack("<H", len(mj)) + mj + np.asarray(d["ranges"], dtype="<f4").tobytes()
                out.append(STREAM_HDR.pack(len(body), 4) + body)
        return b"".join(out)

    def io_frame(self) -> bytes:
        b = json.dumps({"io": self.io(), "photos": self.photoelectric()["sensors"]}, ensure_ascii=False, separators=(",", ":")).encode()
        return STREAM_HDR.pack(len(b), 3) + b

    def imu(self) -> dict:
        with self.lock:
            d = dict(self.core.imu_sample)
            d.update({"t": round(self.core.t, 4), "frame_id": "imu_link"})
            return d

    def sim_status(self) -> dict:
        with self.lock:
            s = self.core.status()
            s.update({"paused": self.core.paused, "rtf_target": self.rtf_target, "api_version": API_VERSION,
                      "cmd_udp_port": getattr(self, "cmd_udp_port", None), "cmd_udp": getattr(self, "cmd_udp", None)})
            return s

    # ---- 开关量传感器
    def _discrete_sig(self):
        c = self.core
        return tuple(b.pressed for b in c.bumpers) + tuple(p.detected for p in c.photos)

    def _discrete_events(self, old, new):
        c = self.core
        items = [("bumper", b.name, b.pressed) for b in c.bumpers] + [("photo", p.name, p.detected) for p in c.photos]
        for (kind, name, v), o in zip(items, old if len(old) == len(items) else [None] * len(items)):
            if o is None or o == v:
                continue
            if kind == "bumper":
                self.emit("BUMPER", "danger" if v else "success", f"防撞触边[{name}] {'压下 — 禁止向该侧运动' if v else '释放'}",
                          {"strip": name, "pressed": v})
            else:
                self.emit("PHOTO", "warning" if v else "info", f"光电[{name}] {'触发' if v else '解除'}", {"sensor": name, "detected": v})

    def photoelectric(self) -> dict:
        with self.lock:
            return {"t": round(self.core.t, 4), "sensors": [p.view() for p in self.core.photos]}

    def bumpers(self) -> dict:
        with self.lock:
            c = self.core
            return {"t": round(c.t, 4), "count": c.collisions, "last_contact": c.last_contact,
                    "any_pressed": any(b.pressed for b in c.bumpers),
                    "strips": [b.view() for b in c.bumpers],
                    "bumper_front": c.io.di.get("di_bumper_front", False), "bumper_rear": c.io.di.get("di_bumper_rear", False)}

    def io(self) -> dict:
        with self.lock:
            return self.core.io.get_io_state()

    def set_io(self, di: Optional[dict] = None, do: Optional[dict] = None):
        with self.lock:
            for k, v in (di or {}).items():
                self.core.io.set_di(k, bool(v))
            for k, v in (do or {}).items():
                self.core.io.set_do(k, bool(v))
            st = self.core.io.get_io_state()
        if di and any(k in ("di_estop",) for k in di):
            self.emit("ESTOP", "danger" if di.get("di_estop") else "success",
                      "急停按下 (抱闸)" if di.get("di_estop") else "急停复位")
        return st

    def codes(self) -> list:
        with self.lock:
            return self.core.detect_codes()

    # ================================================================== 传感器数据
    def sensors(self) -> dict:
        with self.lock:
            lid = []
            for l in self.core.lidars:
                lid.append({"name": l.name, "type": "2d", "frame_id": l.frame_id, "model": l.cfg.get("vendor_model") or l.cfg.get("model"),
                            "mount": {k: l.cfg.get(k, 0.0) for k in ("x", "y", "z", "roll", "pitch", "yaw")},
                            "fov_deg": math.degrees(l.fov), "beams": l.n, "range_max": l.range_max, "freq_hz": l.freq_hz,
                            "seq": self.lidar_bufs[l.name].seq})
            for l in self.core.lidars3d:
                lid.append({"name": l.name, "type": "3d", "frame_id": l.frame_id, "model": l.cfg.get("vendor_model"),
                            "mount": {k: l.cfg.get(k, 0.0) for k in ("x", "y", "z", "roll", "pitch", "yaw")},
                            "vfov_deg": [math.degrees(l.vmin), math.degrees(l.vmax)], "points_per_frame": l.n,
                            "real_points_per_frame": l.real_points, "range_max": l.range_max, "freq_hz": l.freq_hz,
                            "topic_hint": l.cfg.get("topic"), "imu_topic_hint": l.cfg.get("imu_topic"), "seq": self.lidar_bufs[l.name].seq})
            return {"lidars": lid, "imu": self.spec.get("imu"), "cameras": self.spec.get("cameras", []),
                    "io": self.spec.get("io", {}), "merged_scan": {"seq": self.merged.seq, "hz": self.merged_hz},
                    "photoelectric": [p.view() for p in self.core.photos],
                    "camera_streams": [dict(c.info(), seq=self.cam_bufs[c.name].seq) for c in self.core.cameras],
                    "bumpers": [{k: v for k, v in b.view().items() if k != "pressed"} for b in self.core.bumpers]}

    def wait_lidar(self, name: str, after_seq: int, wait: float) -> Optional[dict]:
        """长轮询: 等到 seq > after_seq 的新帧 (最多 wait 秒)"""
        if name not in self.lidar_bufs and name != "merged":
            raise KeyError(name)
        buf = self.merged if name == "merged" else self.lidar_bufs[name]
        deadline = time.time() + max(0.0, min(wait, 5.0))
        with self.cond:
            while (buf.seq <= after_seq or not buf.data) and time.time() < deadline:
                self.cond.wait(timeout=max(0.001, deadline - time.time()))
            return dict(buf.data) if buf.data else None

    def set_lidar_config(self, cfg: dict) -> dict:
        beams = cfg.get("beams")
        if not beams and cfg.get("angle_resolution_deg"):
            beams = int(round(360.0 / float(cfg["angle_resolution_deg"])))
        with self.lock:
            self.core.set_lidar_config(beams=beams, range_max=cfg.get("range_max"), res_deg=cfg.get("sensor_resolution_deg"))
            if cfg.get("freq_hz"):
                self.merged_hz = max(1.0, min(50.0, float(cfg["freq_hz"])))
                self.core.merged_hz = self.merged_hz
                with self.core._sync():
                    pass
            return {"beams": self.core.merged_bins, "angle_resolution_deg": round(360.0 / self.core.merged_bins, 3),
                    "freq_hz": self.merged_hz, "range_max": self.core.merged_range}

    # ================================================================== 控制 & 回馈
    def set_cmd(self, vx: float, vy: float, wz: float, source: str = "nav") -> dict:
        with self.lock:
            self.core.set_cmd(vx, vy, wz)
            self.cmd_meta = {"source": source, "t_wall": time.time(), "vx": vx, "vy": vy, "wz": wz}
            return {"accepted": True, "estop": self.core.estop, "paused": self.core.paused, "t": round(self.core.t, 4)}

    def control(self) -> dict:
        with self.lock:
            if getattr(self, "cmd_udp", None) == "native" and self.core.rt is not None:
                n, wall, (vx, vy, wz), src = self.core.rt.udp_meta()
                if n and wall > self.cmd_meta["t_wall"]:
                    self.cmd_meta = {"source": src or "udp", "t_wall": wall, "vx": vx, "vy": vy, "wz": wz}
            age = time.time() - self.cmd_meta["t_wall"]
            return dict(self.cmd_meta, age_s=round(age, 3), watchdog_timeout_s=self.core.cmd_timeout,
                        watchdog_active=age > self.core.cmd_timeout, estop=self.core.estop)

    def set_nav_feedback(self, fb: dict):
        with self.lock:
            prev = self.nav_feedback.get("status")
            self.nav_feedback = dict(fb, t_wall=time.time())
        if fb.get("status") != prev and fb.get("status") in ("ARRIVED", "FAILED", "NO_PATH", "CANCELED"):
            self.emit("NAV_RESULT", "success" if fb["status"] == "ARRIVED" else "warning",
                      f"执行进程回馈: 任务 #{fb.get('mission_id')} {fb['status']}", {"mission_id": fb.get("mission_id")})

    def get_nav_feedback(self) -> dict:
        with self.lock:
            fb = dict(self.nav_feedback)
        fb["age_s"] = round(time.time() - fb.get("t_wall", 0.0), 3)
        if fb["age_s"] > 3.0:
            fb["online"] = False
        else:
            fb.setdefault("online", True)
        return fb

    # ================================================================== 仿真控制
    def set_sim(self, paused: Optional[bool] = None, rtf: Optional[float] = None):
        with self.lock:
            with self.core._sync():
                if paused is not None:
                    self.core.paused = bool(paused)
                if rtf is not None:
                    self.rtf_target = max(0.1, min(5.0, float(rtf)))
                self.core.rtf_target = self.rtf_target
        if paused is not None:
            self.emit("SIM_PAUSE", "info", "仿真暂停" if paused else "仿真恢复")

    def reset(self, x=None, y=None, yaw=None):
        with self.lock:
            o = SCENARIO_DEFINITIONS[self.core.scenario_id].get("origin", {})
            self.core.reset_pose(float(o.get("x", 0.0) if x is None else x), float(o.get("y", 0.0) if y is None else y),
                                 float(o.get("yaw", 0.0) if yaw is None else yaw))
        self.emit("SIM_RESET", "warning", "车辆位姿复位")

    def events_since(self, since: int) -> list:
        with self.lock:
            return [e for e in self.events if e["id"] > since]

    # ================================================================== 汇总快照 (Web)
    def snapshot(self, scans: bool = False, max_3d_points: int = 3000) -> dict:
        snap = {"state": self.state(), "io": self.io(), "codes": self.codes(), "sim": self.sim_status(),
                "photoelectric": self.photoelectric()["sensors"], "bumpers": self.bumpers(), "cameras": self.cameras(),
                "control": self.control(), "nav": self.get_nav_feedback()}
        with self.lock:
            m = self.merged.data
            if m:
                # 同一帧会被多个网页/网关反复取: 按帧序号缓存 JSON 化结果
                cache = getattr(self, "_merged_json", None)
                if cache is None or cache[0] != m["seq"]:
                    cache = self._merged_json = (m["seq"], ranges_json(m["ranges"], 3))
                snap["merged_scan"] = {k: v for k, v in m.items() if k != "ranges"}
                snap["merged_scan"]["ranges"] = cache[1]
            if scans:
                out = {}
                for name, b in self.lidar_bufs.items():
                    d = b.data
                    if not d:
                        continue
                    if d["type"] == "2d":
                        step = max(1, len(d["ranges"]) // 360)
                        r = np.asarray(d["ranges"][::step], dtype=np.float64)
                        out[name] = {"type": "2d", "seq": d["seq"], "pose": d["pose"], "angle_min": d["angle_min"],
                                     "angle_inc": d["angle_increment"] * step,
                                     "ranges": np.round(np.where(np.isfinite(r), r, -1.0), 2).tolist()}
                    else:
                        l3 = next(l for l in self.core.lidars3d if l.name == name)
                        P = l3.points_base({"points": d["points"]})
                        step = max(1, len(P) // max_3d_points)
                        out[name] = {"type": "3d", "seq": d["seq"], "pose": d["pose"], "n_raw": int(len(P)),
                                     "points3d": np.round(P[::step], 2).ravel().tolist()}
                snap["lidar_scans"] = out
        return snap
