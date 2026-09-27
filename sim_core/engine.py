#!/usr/bin/env python3
"""
SimCore —— 与 ROS 解耦的仿真内核 (可离线单测/基准测试)

* 固定步长 (默认 10 ms) + 实时累加器调度: 物理时间严格按步长推进，不受定时器抖动影响；
  落后过多时丢弃并计数 (overruns)，不会出现"越跑越慢"的时间漂移。
* 后端: mujoco (默认: MuJoCo 平面刚体+接触，激光/光电/相机/ToF 全部走 mj_multiRay 批量射线)
        kinematic (仅在未安装 mujoco 时的兜底: 运动学积分 + numpy 线段求交，无相机着色)
* 真值与估计分离: truth 位姿 (地图系) vs /odom (编码器积分，含轮径误差/量化/打滑，会漂移)
"""

import contextlib
import math
import time
from typing import Dict, List, Optional, Tuple

import numpy as np

from .kinematics import ChassisKinematics, build_kinematics, se2_integrate, wrap
from .sensors import (CodeReader, GroundSlip, ImuModel, Lidar3DSensor, LidarSensor, StationTagCamera, WheelOdometry,
                      merge_scans, slice_to_scan)
import os
from .world import World
from . import native
from .discrete import NativeBatch, build_bumpers, build_photos, motion_block

from .cameras import build_camera
from .mujoco_backend import MUJOCO_AVAILABLE, MUJOCO_VERSION, MuJoCoBackend


class _IO:
    """工业 IO (与 sensors/io_simulator.IOSimulator 同接口，按 cmodel 扩展 DI/DO)"""

    def __init__(self, spec: dict):
        from sensors.io_simulator import IOSimulator  # 复用原实现
        self.sim = IOSimulator()
        io = (spec or {}).get("io", {})
        for d in io.get("inputs", []):
            self.sim.di.setdefault(f"di_{d['name']}", False)
        for d in io.get("outputs", []):
            self.sim.do.setdefault(f"do_{d['name']}", False)


class SimCore:
    def __init__(self, spec: dict, scenarios: Dict[str, dict], scenario_id: str = None,
                 backend: str = "auto", dt: float = 0.01, noise: bool = True, chassis_type: Optional[str] = None):
        self.spec = spec
        self.scenarios = scenarios
        self.dt = dt
        self.noise = noise
        self.world = World()
        self.footprint = spec["chassis"]["footprint"]
        self.io = _IO(spec).sim
        self.t = 0.0
        self.paused = False
        self.cmd = (0.0, 0.0, 0.0)
        self.cmd_time = -1e9
        self.cmd_timeout = 0.5
        self.acc = 0.0
        self.max_substeps = 10
        self.overruns = 0
        self.collisions = 0
        self.bumper_until = -1.0
        self.last_contact = None
        self.step_ms = 0.0
        self.lidar_ms = 0.0
        self.rtf = 1.0
        self._rtf_win = [time.perf_counter(), 0.0]
        # 位姿真值
        self.x = self.y = self.th = 0.0
        self.vx = self.vy = self.wz = 0.0
        self.slip = GroundSlip(enabled=noise)
        self.imu = ImuModel(enabled=noise)
        self.imu_sample = {"wz": 0.0, "ax": 0.0, "ay": 0.0, "az": 9.81, "yaw": 0.0}
        # 传感器
        self.lidars = [LidarSensor(l) for l in spec.get("lidars", []) if l.get("type") != "3d"]
        n3d = int(os.environ.get("SIM_LIDAR3D_POINTS", "6000"))   # Mid-360S 实机 20000 点/帧；树莓派默认降采样
        self.lidars3d = [Lidar3DSensor(l, n3d) for l in spec.get("lidars", []) if l.get("type") == "3d"]
        self.last_clouds = []          # [(sensor, cloud_dict, slice_ranges)]
        self.merged_bins = 720
        self.merged_range = max([l.range_max for l in self.lidars + self.lidars3d] or [12.0])
        # 3D 点云切片为 2D 障碍 (机体系高度带): 地面以上 5 cm 至车高 +0.15 m
        self.slice_zmin = 0.05
        self.slice_zmax = float(spec["chassis"].get("height_m", 2.0)) + 0.15
        # 开关量传感器: 光电 + 防撞触边
        self.photos = build_photos(spec)
        self.bumpers = build_bumpers(spec)
        for p in self.photos:
            self.io.di.setdefault(p.di, False)
        for b in self.bumpers:
            self.io.di.setdefault(b.di, False)
        self._discrete_n = 0
        self._batch = NativeBatch(self.photos, self.bumpers) if native.lib is not None else None
        self.code_reader = CodeReader(spec.get("cameras", []))
        self.station_cam = StationTagCamera(mount_x=spec["chassis"].get("head_offset_m", 0.5) * 0.8)
        # 后端
        # 相机类 (单目/双目/ToF)；读码相机仍由 CodeReader 处理
        self.cameras = [c for c in (build_camera(cfg) for cfg in spec.get("cameras", [])) if c is not None]
        # 后端
        self.backend_name = "kinematic"
        self.mj = None
        if backend in ("mujoco", "auto"):
            if MUJOCO_AVAILABLE:
                self.mj = MuJoCoBackend(dt=dt)
                self.mj.set_robot(spec)
                self.backend_name = f"mujoco {MUJOCO_VERSION}"
                self.world.engine = self.mj
                if os.environ.get("SIM_CAMERA_RENDER", "ray") == "gl" and self.cameras:
                    defs = [d for c in self.cameras if c.kind in ("camera", "stereo") for d in c.gl_defs()]
                    self.mj.set_gl_cameras(defs)
                    for c in self.cameras:
                        c.use_gl = c.kind in ("camera", "stereo")
            else:
                print("[SimCore] 未安装 mujoco，退回 kinematic 兜底后端 (pip install mujoco)")
        self.rt = None                 # 原生实时循环 (sim_core/rt.py)，由 enable_rt() 启用
        self.lidar_max_hz = 10.0
        self.merged_hz = 10.0
        self.rtf_target = 1.0
        self.set_chassis(chassis_type)
        self.set_scenario(scenario_id or next(iter(scenarios)))

    # ==================================================================
    # 原生实时循环
    # ==================================================================
    def enable_rt(self) -> bool:
        """SIM_RT=1 (默认) 且 C 内核可用时: 物理/开关量/里程计/IMU/2D 激光改由 C 线程推进 (MuJoCo 仍是物理引擎)"""
        from . import rt as _rt
        if self.rt is not None or not _rt.available() or not self.kin.native:
            return self.rt is not None
        if self.mj is not None and not native.mj_bind():
            return False
        self.rt = _rt.NativeRT(self)
        return True

    def _sync(self):
        """修改仿真状态的 Python 代码: 暂停 C 线程并同步状态 (可嵌套)"""
        return self.rt.hold() if self.rt is not None else contextlib.nullcontext()

    def rt_pull(self):
        if self.rt is not None:
            with self.rt.hold(sync=False):
                self.rt.pull()

    # ==================================================================
    # 配置
    # ==================================================================
    def set_chassis(self, chassis_type: Optional[str]):
        with self._sync():
            keep = (self.x, self.y, self.th)
            self.kin, self.wheels_d = build_kinematics(self.spec, chassis_type)
            self.chassis_type = self.kin.type
            self.odom = WheelOdometry(self.kin, enabled=self.noise)
            self.odom.reset(*keep)
            self._odom_origin = keep
            return self.chassis_type

    def robot_urdf(self) -> str:
        from cmodel_parser import generate_urdf
        sp = dict(self.spec)
        sp["wheels"] = self.wheels_d
        return generate_urdf(sp)

    def set_scenario(self, scenario_id: str):
        with self._sync():
            sc = self.scenarios[scenario_id]
            self.scenario_id = scenario_id
            self.scenario = sc
            self.world.load_scenario(sc)
            nodes = sc.get("nodes", {})
            self.ground_tags = [{"id": f"QR_{k}", "name": k, "x": v[0], "y": v[1], "yaw": 0.0} for k, v in nodes.items()]
            self.station_tags = [{"id": 100 + i, "name": s.get("name", s.get("id")), "x": s["x"], "y": s["y"],
                                  "yaw": s.get("dock_yaw", 0.0)} for i, s in enumerate(sc.get("stations", []))]
            if self.mj:
                lanes = [(nodes[a][0], nodes[a][1], nodes[b][0], nodes[b][1]) for a, b in sc.get("connections", [])
                         if a in nodes and b in nodes]
                self.mj.build(self.world, self.station_tags, self.ground_tags, lanes, keep_state=False)
            o = sc.get("origin", {"x": 0.0, "y": 0.0, "yaw": 0.0})
            self.reset_pose(o["x"], o["y"], o.get("yaw", 0.0))

    def set_obstacles(self, obstacles: List[dict]):
        with self._sync():
            self.world.set_obstacles(obstacles)
            for o in self.world.obstacles:
                if o.get("motion"):
                    o["motion"].setdefault("t0", self.t)
                    o["motion"].setdefault("ax", o["x"])
                    o["motion"].setdefault("ay", o["y"])
            if self.mj:
                self.mj.build(self.world, keep_state=True)

    def update_movers(self):
        """行走人员等动态障碍物: 在 A↔B 之间往返 (motion={type:patrol, ax, ay, bx, by, speed})"""
        with self._sync():
            poses = []
            for i, o in enumerate(self.world.obstacles):
                m = o.get("motion")
                if not m or m.get("type", "patrol") != "patrol":
                    continue
                ax, ay, bx, by = float(m["ax"]), float(m["ay"]), float(m.get("bx", m["ax"])), float(m.get("by", m["ay"]))
                L = math.hypot(bx - ax, by - ay)
                if L < 1e-3:
                    continue
                s = (self.t - float(m.get("t0", 0.0))) * float(m.get("speed", 0.8)) % (2 * L)
                k = s / L if s <= L else 2 - s / L
                x, y = ax + (bx - ax) * k, ay + (by - ay) * k
                yaw = math.atan2(by - ay, bx - ax) + (0.0 if s <= L else math.pi)
                o["x"], o["y"], o["yaw"] = round(x, 3), round(y, 3), round(yaw, 3)
                poses.append((i, x, y, yaw))
            if poses:
                self.world._rebuild_dynamic()
                if self.mj:
                    self.mj.move_obstacles(poses)

    def reset_pose(self, x: float, y: float, th: float):
        with self._sync():
            self.x, self.y, self.th = x, y, th
            for b in getattr(self, "bumpers", []):
                b.pressed = False
                self.io.set_di(b.di, False)
            self.vx = self.vy = self.wz = 0.0
            self.kin.reset()
            self.odom.reset(x, y, th)
            self.cmd = (0.0, 0.0, 0.0)
            if self.mj and self.mj.m is not None:
                self.mj.reset_pose(x, y, th)

    def set_cmd(self, vx: float, vy: float, wz: float):
        self.cmd = (float(vx), float(vy), float(wz))
        self.cmd_time = self.t
        if self.rt is not None:
            self.rt.set_cmd(*self.cmd)

    def set_lidar_config(self, beams: Optional[int] = None, res_deg: Optional[float] = None,
                         range_max: Optional[float] = None, freq_hz: Optional[float] = None):
        """beams → 融合扫描 (/scan) 的 360° 点数；res_deg → 所有物理激光的角分辨率；range_max → 量程上限"""
        with self._sync():
            if beams:
                self.merged_bins = int(max(90, min(3600, beams)))
            if res_deg:
                for l in self.lidars:
                    l.set_resolution(math.radians(float(res_deg)))
            if range_max:
                for l in self.lidars:
                    l.range_max = min(l.native_range_max, float(range_max)) if float(range_max) < l.native_range_max else float(range_max)
                self.merged_range = max(l.range_max for l in self.lidars)
            if freq_hz:
                for l in self.lidars:
                    l.freq_hz = float(freq_hz)

    # ==================================================================
    # 物理步进
    # ==================================================================
    @property
    def estop(self) -> bool:
        return not self.io.do.get("do_brake_release", True)

    def step(self):
        dt = self.dt
        t0 = time.perf_counter()
        if self.paused:
            return
        watchdog = (self.t - self.cmd_time) > self.cmd_timeout
        brake = self.estop
        cvx, cvy, cwz = (0.0, 0.0, 0.0) if (watchdog or brake) else self.cmd
        cvx, cvy, cwz = motion_block(self.bumpers, cvx, cvy, cwz)

        self.kin.step(cvx, cvy, cwz, dt, brake=brake)

        if self.mj:
            vx, vy, wz = self.slip.apply(self.kin.vx, self.kin.vy, self.kin.wz, dt)
            x, y, th, vx, vy, wz, contact = self.mj.step(vx, vy, wz)
            if contact is not None:
                self._on_collision(contact)
            self.x, self.y, self.th, self.vx, self.vy, self.wz = x, y, th, vx, vy, wz
        else:
            vx, vy, wz = self.slip.apply(self.kin.vx, self.kin.vy, self.kin.wz, dt)
            nx, ny, nth = se2_integrate(self.x, self.y, self.th, vx, vy, wz, dt)
            hit, pt = self.world.collides(self.footprint, nx, ny, nth)
            if hit:
                self._on_collision(pt)
                self.kin.stop_now()
                vx = vy = wz = 0.0
            else:
                self.x, self.y, self.th = nx, ny, nth
            self.vx, self.vy, self.wz = vx, vy, wz

        self.update_discrete()
        self.odom.update(dt)
        self.imu_sample = self.imu.sample(self.vx, self.vy, self.wz, self.th, dt)
        self.io.update_lift_physics(dt)
        self.t += dt
        self._mover_acc = getattr(self, "_mover_acc", 0.0) + dt
        if self._mover_acc >= 0.05:
            self._mover_acc = 0.0
            if any(o.get("motion") for o in self.world.obstacles):
                self.update_movers()
        self.step_ms = 0.9 * self.step_ms + 0.1 * (time.perf_counter() - t0) * 1000.0

    def _on_collision(self, pt):
        if not any(b.pressed for b in self.bumpers):
            self.collisions += 1
        front = True
        if pt is not None:
            c, s = math.cos(self.th), math.sin(self.th)
            bx = c * (pt[0] - self.x) + s * (pt[1] - self.y)
            front = bx >= (self.spec["chassis"]["head_offset_m"] - self.spec["chassis"]["tail_offset_m"]) / 2.0
            self.last_contact = (float(pt[0]), float(pt[1]))
        side = "front" if front else "rear"
        for b in self.bumpers:
            if b.side == side:
                b.update(self.world, self.x, self.y, self.th, self.t, forced=True)
                self.io.set_di(b.di, True)

    def update_discrete(self, force: bool = False):
        """触边每步检测 (安全链)；光电 50 Hz"""
        changed = False
        nb = self._batch
        hits = nb.bumper_hits(self.world, self.x, self.y, self.th) if nb is not None else None
        for i, b in enumerate(self.bumpers):
            was = b.pressed
            if hits is not None:
                b.apply_hit(hits[i], self.t)
            else:
                b.update(self.world, self.x, self.y, self.th, self.t)
            if b.pressed != was:
                if b.pressed and not was:
                    self.collisions += 1
                    self.last_contact = (self.x, self.y)
                self.io.set_di(b.di, b.pressed)
                changed = True
        self._discrete_n += 1
        if force or self._discrete_n % 2 == 0:
            ds = nb.photo_distances(self.world, self.x, self.y, self.th) if nb is not None else None
            for i, p in enumerate(self.photos):
                was = p.detected
                det = p.apply_distance(float(ds[i])) if ds is not None else p.update(self.world, self.x, self.y, self.th)
                if det != was:
                    self.io.set_di(p.di, p.detected)
                    changed = True
        return changed

    def advance(self, real_dt: float, rtf: float = 1.0) -> int:
        """按真实流逝时间推进若干固定步；返回执行步数"""
        self.acc += max(0.0, real_dt) * rtf
        n = 0
        while self.acc >= self.dt and n < self.max_substeps:
            self.step()
            self.acc -= self.dt
            n += 1
        if self.acc >= self.dt:  # 追不上 → 丢弃积压，计数
            self.overruns += 1
            self.acc = 0.0
        # 实时因子统计
        self._rtf_win[1] += n * self.dt
        now = time.perf_counter()
        if now - self._rtf_win[0] >= 1.0:
            self.rtf = self._rtf_win[1] / (now - self._rtf_win[0])
            self._rtf_win = [now, 0.0]
        return n

    # ==================================================================
    # 传感器
    # ==================================================================
    def scan_all(self) -> Tuple[List[np.ndarray], np.ndarray]:
        t0 = time.perf_counter()
        rs = [l.scan(self.world, self.x, self.y, self.th, noise=self.noise) for l in self.lidars]
        merged = merge_scans(self.lidars, rs, self.merged_bins, self.merged_range)
        clouds = []
        for l3 in self.lidars3d:
            cl = l3.scan(self.world, self.x, self.y, self.th, noise=self.noise)
            sl = slice_to_scan(l3.points_base(cl), self.slice_zmin, self.slice_zmax, self.merged_bins, self.merged_range)
            merged = np.minimum(merged, sl)
            clouds.append((l3, cl, sl))
        self.last_clouds = clouds
        self.lidar_ms = 0.8 * self.lidar_ms + 0.2 * (time.perf_counter() - t0) * 1000.0
        return rs, merged

    def detect_codes(self) -> List[dict]:
        out = self.code_reader.detect(self.x, self.y, self.th, self.ground_tags, self.station_tags)
        out += self.station_cam.detect(self.x, self.y, self.th, self.station_tags)
        return out

    def map_to_odom(self) -> Tuple[float, float, float]:
        """真值定位: T_map_odom = T_map_base · T_odom_base⁻¹"""
        ox, oy, oth = self.odom.x, self.odom.y, self.odom.th
        dth = wrap(self.th - oth)
        c, s = math.cos(dth), math.sin(dth)
        tx = self.x - (c * ox - s * oy)
        ty = self.y - (s * ox + c * oy)
        return tx, ty, dth

    def status(self) -> dict:
        return {
            "backend": self.backend_name, "chassis": self.chassis_type, "scenario": self.scenario_id,
            "sim_time": round(self.t, 3), "dt": self.dt, "rtf": round(self.rtf, 3), "overruns": self.overruns,
            "step_ms": round(self.step_ms, 3), "lidar_ms": round(self.lidar_ms, 3), "collisions": self.collisions,
            "odom_drift_m": round(math.hypot(self.x - self.odom.x, self.y - self.odom.y), 4),
            "odom_drift_deg": round(math.degrees(wrap(self.th - self.odom.th)), 3),
            "lidars": [{"name": l.name, "beams": l.n, "res_deg": round(math.degrees(l.angle_inc), 3), "fov_deg": round(math.degrees(l.fov), 1),
                        "range_max": l.range_max, "freq_hz": l.freq_hz, "z": l.mz, "inverted": l.sign < 0} for l in self.lidars]
                      + [{"name": l.name, "type": "3d", "beams": l.n, "real_points_per_frame": l.real_points, "fov_deg": 360.0,
                          "vfov_deg": [round(math.degrees(l.vmin), 1), round(math.degrees(l.vmax), 1)], "range_max": l.range_max,
                          "freq_hz": l.freq_hz, "z": l.mz, "model": l.cfg.get("vendor_model")} for l in self.lidars3d],
            "kinematics": self.kin.telemetry(),
            "engine": ({"name": "mujoco", "version": MUJOCO_VERSION, "ngeom": self.mj.m.ngeom, "compile_ms": round(self.mj.compile_ms, 2),
                        "mj_step_ms": round(self.mj.step_ms, 4), "ray_threads": self.mj.threads,
                        "rays_total": self.mj.ray_count, "ray_ms_total": round(self.mj.ray_ms, 1)}
                       if self.mj else {"name": "kinematic", "note": "未安装 mujoco，兜底模式"}),
            "cameras": [{"name": c.name, "type": c.kind, "res": f"{c.W}x{c.H}", "fps": c.fps, "ms": round(c.last_ms, 2),
                         "render": "gl" if getattr(c, "use_gl", False) and self.mj and not self.mj.gl_error else "ray"} for c in self.cameras],
            "camera_gl_error": self.mj.gl_error if self.mj else None,
            "native": native.summary(),
        }

    def capture_camera(self, cam) -> dict:
        t0 = time.perf_counter()
        out = cam.capture(self.world, self.mj, self.x, self.y, self.th, noise=self.noise)
        cam.last_ms = 0.8 * cam.last_ms + 0.2 * (time.perf_counter() - t0) * 1000.0
        return out
