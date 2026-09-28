#!/usr/bin/env python3
"""
仿真实时循环的原生实现 (sim_core/native/simcore_rt.c) —— SIM_RT=1 (默认) 时替代 SimService 的 Python 物理线程

  C 线程 (不持有 GIL) 按固定步长推进: 运动学 → 打滑 → MuJoCo mj_step (C API) → 接触/触边/光电 (mj_multiRay) →
  里程计 → IMU；2D 激光与 3D 激光 (Livox 类，高度带切片并入融合) 按频率扫描 (mj_multiRay) 并融合，双缓冲输出。
  Python 侧只做: 配置下发 (configure)、状态快照读写 (pull/push)、行人移动、顶升/IO/事件、相机。
  SIM_RT_L3D=0: 3D 激光退回 Python (此时 2D 激光也在 Python，融合要叠加 3D 切片)。

  所有改动 SimCore 的 Python 代码都在 hold() 里: 先 pull 最新状态 → 修改 → configure/push → 放开 C 线程。
"""
import ctypes
import math
import os
import random
from contextlib import contextmanager

import numpy as np

from . import native

_D, _I, _U = ctypes.c_double, ctypes.c_int32, ctypes.c_uint32
MAXB, MAXP, MAXL, MAXL3 = 8, 16, 8, 4
SIDES = {"front": 0, "rear": 1, "left": 2, "right": 3}


class RtState(ctypes.Structure):
    _fields_ = ([(n, _D) for n in ("t", "x", "y", "th", "vx", "vy", "wz")]
                + [("odom", _D * 6), ("imu", _D * 5), ("imu_st", _D * 4), ("slip_prev", _D * 3), ("cmd", _D * 3), ("cmd_time", _D),
                   ("last_contact", _D * 2)]
                + [(n, _D) for n in ("step_ms", "lidar_ms", "mj_step_ms", "rtf", "rtf_target")]
                + [("bumper_contact_t", _D * MAXB), ("photo_dist", _D * MAXP)]
                + [(n, _U) for n in ("collisions", "overruns", "steps", "discrete_n")]
                + [(n, _I) for n in ("has_contact", "paused", "brake", "_pad")]
                + [("bumper_pressed", _I * MAXB), ("bumper_count", _I * MAXB), ("photo_detected", _I * MAXP)])


class RtConfig(ctypes.Structure):
    _fields_ = ([(n, _D) for n in ("dt", "cmd_timeout", "head", "tail", "steer_sigma", "cpr", "merged_rmax", "merged_period")]
                + [("imu_par", _D * 3)]
                + [(n, _I) for n in ("max_substeps", "noise", "use_mj", "nbins", "robot_body", "lidars_on", "_pad0", "_pad1")]
                + [("group", ctypes.c_uint8 * 8)])


class RtLidar(ctypes.Structure):
    _fields_ = [(n, _D) for n in ("mx", "my", "mz", "yaw", "sign", "a0", "inc", "rmax", "rmin", "std", "prop", "dropout", "period")] \
        + [("n", _I), ("_pad", _I)]


class RtLidar3D(ctypes.Structure):
    _fields_ = [(n, _D) for n in ("mx", "my", "mz")] + [("R", _D * 9)] \
        + [(n, _D) for n in ("vmin", "vmax", "rmin", "rmax", "std", "ang_noise", "period", "ceiling", "zmin", "zmax", "frame")] \
        + [("n", _I), ("lines", _I)]


def available() -> bool:
    if native.lib is None or os.environ.get("SIM_RT", "1").strip() in ("0", "false", "off", "no"):
        return False
    lib = native.lib
    return (lib.sc_rt_sizeof_state() == ctypes.sizeof(RtState) and lib.sc_rt_sizeof_config() == ctypes.sizeof(RtConfig)
            and lib.sc_rt_sizeof_lidar() == ctypes.sizeof(RtLidar) and lib.sc_rt_sizeof_l3d() == ctypes.sizeof(RtLidar3D))


class NativeRT:
    def __init__(self, core):
        self.core = core
        self.lib = native.lib
        self.h = self.lib.sc_rt_create(random.getrandbits(64))
        self.st = RtState()
        self.cfg = RtConfig()
        self._refs = []            # 交给 C 的内存必须由 Python 持有
        self._depth = 0
        self.use_mj = core.mj is not None and native.mj_bind()
        self.lidars_on = False
        self.started = False
        self._lid_bufs = []
        self._merged_buf = None
        self._seq = (ctypes.c_uint32(), _D(), (_D * 3)())

    # ------------------------------------------------------------------ 锁
    @contextmanager
    def hold(self, sync: bool = True):
        """C 线程暂停在步与步之间；sync=True: 进入时 pull，退出时 configure + push"""
        if self._depth:
            self._depth += 1
            try:
                yield
            finally:
                self._depth -= 1
            return
        self.lib.sc_rt_lock(self.h)
        self._depth = 1
        try:
            if sync:
                self.pull()
            yield
            if sync:
                self.configure()
                self.push()
        finally:
            self._depth = 0
            self.lib.sc_rt_unlock(self.h)

    # ------------------------------------------------------------------ 配置 (须在 hold 内)
    def configure(self):
        c, lib, h, cfg = self.core, self.lib, self.h, self.cfg
        ch = c.spec["chassis"]
        self.use_mj = c.mj is not None and c.mj.m is not None and native.mj_bind()
        refs = []
        cfg.dt, cfg.cmd_timeout = c.dt, c.cmd_timeout
        cfg.head, cfg.tail = float(ch.get("head_offset_m", 0.5)), float(ch.get("tail_offset_m", 0.5))
        cfg.max_substeps, cfg.noise, cfg.use_mj = c.max_substeps, 1 if c.noise else 0, 1 if self.use_mj else 0
        # 运动学 / 里程计 (与 Python 对象共用同一块 C 内存)
        kin, odom = c.kin, c.odom
        if not kin.native:
            raise RuntimeError("运动学未启用 C 内核")
        if odom._nat is not None:
            re, sb = odom._nat[0], odom._nat[1]
        else:
            re = (_D * len(kin.wheels))(*[1.0] * len(kin.wheels))
            sb = (_D * len(kin.wheels))()
        refs += [re, sb]
        cfg.steer_sigma, cfg.cpr = odom.steer_sigma, float(odom.cpr)
        kin._sync_params()
        lib.sc_rt_set_kin(h, ctypes.addressof(kin._cc), ctypes.addressof(kin._cw), len(kin.wheels),
                          ctypes.addressof(re), ctypes.addressof(sb))
        imu = c.imu
        cfg.imu_par[0], cfg.imu_par[1], cfg.imu_par[2] = (math.radians(0.001) if imu.enabled else 0.0), imu.gyro_noise, imu.acc_noise
        # 几何 (触边/兜底后端)
        segs = np.ascontiguousarray(c.world.segments, dtype=np.float64)
        fp = np.ascontiguousarray(c.footprint, dtype=np.float64)
        refs += [segs, fp]
        lib.sc_rt_set_geom(h, native.ptr(segs), len(segs), native.ptr(fp), len(fp))
        # MuJoCo
        if self.use_mj:
            mj = c.mj
            flags = np.zeros(mj.m.ngeom, dtype=np.int32)
            for g in mj.robot_geoms:
                flags[g] = 1
            refs += [flags, mj.m, mj.d]
            lib.sc_rt_set_mj(h, mj.m._address, mj.d._address, flags.ctypes.data, len(flags))
            cfg.robot_body = mj.robot_body
            for i, v in enumerate(mj.GEOMGROUP):
                cfg.group[i] = int(v)
        # 光电 / 触边
        P = np.ascontiguousarray([[p.x, p.y, p.z, p.yaw, p.half, p.range_m, p.trigger_m, p.hyst] for p in c.photos[:MAXP]],
                                 dtype=np.float64).reshape(-1, 8)
        refs.append(P)
        lib.sc_rt_set_photos(h, native.ptr(P), len(P))
        bs = c.bumpers[:MAXB]
        polys = np.ascontiguousarray(np.concatenate([b._poly_np for b in bs]) if bs else np.zeros((0, 2)))
        counts = (ctypes.c_int * max(1, len(bs)))(*[len(b._poly_np) for b in bs])
        sides = (ctypes.c_int * max(1, len(bs)))(*[SIDES.get(b.side, 0) for b in bs])
        holds = (_D * max(1, len(bs)))(*[b.hold_s for b in bs])
        refs += [polys, counts, sides, holds]
        lib.sc_rt_set_bumpers(h, native.ptr(polys), ctypes.addressof(counts), ctypes.addressof(sides), ctypes.addressof(holds), len(bs))
        # 2D / 3D 激光 (3D 需 MuJoCo 求交；SIM_RT_L3D=0 或 3D 超过 MAXL3 台时全部激光退回 Python)
        from .sensors import LIDAR_RAY
        l3d_ok = not c.lidars3d or (self.use_mj and len(c.lidars3d) <= MAXL3 and os.environ.get("SIM_RT_L3D", "1") != "0")
        self.lidars_on = bool(c.lidars or c.lidars3d) and l3d_ok and LIDAR_RAY != "engine" and len(c.lidars) <= MAXL
        cfg.lidars_on = 1 if self.lidars_on else 0
        cfg.nbins, cfg.merged_rmax = c.merged_bins, c.merged_range
        cfg.merged_period = 1.0 / max(0.1, getattr(c, "merged_hz", 10.0))
        mx = getattr(c, "lidar_max_hz", 10.0)
        L = (RtLidar * max(1, len(c.lidars)))()
        for i, l in enumerate(c.lidars[:MAXL]):
            hz = min(l.freq_hz, mx) if mx > 0 else l.freq_hz
            L[i].mx, L[i].my, L[i].mz, L[i].yaw, L[i].sign = l.mx, l.my, l.mz, l.yaw, l.sign
            L[i].a0, L[i].inc, L[i].n = l.angle_min, l.angle_inc, l.n
            L[i].rmax, L[i].rmin, L[i].std, L[i].prop, L[i].dropout = l.range_max, l.range_min, l.noise_std, l.noise_prop, l.dropout
            L[i].period = 1.0 / max(1.0, hz)
        l3s = c.lidars3d[:MAXL3] if self.lidars_on else []
        sig = (tuple((l.n, l.angle_min, l.angle_inc, l.range_max, l.freq_hz) for l in c.lidars), c.merged_bins, c.merged_range,
               cfg.merged_period, mx, tuple((l.name, l.n, l.freq_hz, l.range_max, l.mx, l.my, l.mz) for l in l3s),
               c.slice_zmin, c.slice_zmax)
        if sig != getattr(self, "_lid_sig", None):          # 激光配置变化才重建缓冲 (会重置扫描时刻)
            self._lid_sig = sig
            lib.sc_rt_set_lidars(h, ctypes.addressof(L), len(c.lidars[:MAXL]), c.merged_bins)
            self._lid_bufs = [np.empty(l.n) for l in c.lidars[:MAXL]]
            self._merged_buf = np.empty(c.merged_bins)
            from .world import CEILING_HEIGHT
            L3 = (RtLidar3D * max(1, len(l3s)))()
            for i, l in enumerate(l3s):
                q = L3[i]
                q.mx, q.my, q.mz = l.mx, l.my, l.mz
                for k, v in enumerate(np.asarray(l.R, float).ravel()):
                    q.R[k] = v
                q.vmin, q.vmax, q.rmin, q.rmax = l.vmin, l.vmax, l.range_min, l.range_max
                q.std, q.ang_noise, q.period = l.noise_std, l.ang_noise, 1.0 / max(1.0, l.freq_hz)
                q.ceiling, q.zmin, q.zmax, q.frame = CEILING_HEIGHT, c.slice_zmin, c.slice_zmax, float(l.frame)
                q.n, q.lines = l.n, l.lines
            lib.sc_rt_set_lidars3d(h, ctypes.addressof(L3), len(l3s))
            self._l3d_bufs = [(np.empty((l.n, 3), np.float32), np.empty(l.n, np.float32), np.empty(l.n, np.uint8), np.empty(l.n),
                               np.empty(c.merged_bins)) for l in l3s]
        lib.sc_rt_set_config(h, ctypes.addressof(cfg))
        self._refs = refs

    # ------------------------------------------------------------------ 状态 (须在 hold 内)
    def pull(self):
        c, st = self.core, self.st
        self.lib.sc_rt_get_state(self.h, ctypes.addressof(st))
        c.t, c.x, c.y, c.th, c.vx, c.vy, c.wz = st.t, st.x, st.y, st.th, st.vx, st.vy, st.wz
        o = st.odom
        od = c.odom
        od.x, od.y, od.th, od.vx, od.vy, od.wz = o[0], o[1], o[2], o[3], o[4], o[5]
        im = st.imu
        c.imu_sample = {"wz": im[0], "ax": im[1], "ay": im[2], "az": im[3], "yaw": im[4]}
        c.imu.prev, c.imu.gyro_bias, c.imu.yaw = (st.imu_st[0], st.imu_st[1]), st.imu_st[2], st.imu_st[3]
        c.slip.prev = (st.slip_prev[0], st.slip_prev[1], st.slip_prev[2])
        c.cmd_time = st.cmd_time
        c.collisions, c.overruns = st.collisions, st.overruns
        c.last_contact = (st.last_contact[0], st.last_contact[1]) if st.has_contact else c.last_contact
        c.step_ms, c.lidar_ms, c.rtf = st.step_ms, st.lidar_ms, st.rtf
        if c.mj is not None:
            c.mj.step_ms = st.mj_step_ms
        c._discrete_n = st.discrete_n
        for i, b in enumerate(c.bumpers[:MAXB]):
            b.pressed, b.contact_t, b.press_count = bool(st.bumper_pressed[i]), st.bumper_contact_t[i], st.bumper_count[i]
        for i, p in enumerate(c.photos[:MAXP]):
            p.detected, p.distance = bool(st.photo_detected[i]), st.photo_dist[i]
        kin = c.kin
        for cw, w in zip(kin._cw, kin.wheels):
            w.steer, w.steer_target, w.speed, w.speed_target, w.cmd_speed = cw.steer, cw.steer_target, cw.speed, cw.speed_target, cw.cmd_speed
            w.angle, w.motor_rpm, w.current_a, w.torque_nm, w.steer_current_a = cw.angle, cw.motor_rpm, cw.current_a, cw.torque_nm, cw.steer_current_a
        cc = kin._cc
        kin.shaped = [cc.shaped[0], cc.shaped[1], cc.shaped[2]]
        kin.vx, kin.vy, kin.wz, kin.slip_residual, kin.saturation = cc.vx, cc.vy, cc.wz, cc.slip_residual, cc.saturation

    def push(self):
        c, st = self.core, self.st
        st.t, st.x, st.y, st.th, st.vx, st.vy, st.wz = c.t, c.x, c.y, c.th, c.vx, c.vy, c.wz
        od = c.odom
        for i, v in enumerate((od.x, od.y, od.th, od.vx, od.vy, od.wz)):
            st.odom[i] = v
        st.imu_st[0], st.imu_st[1], st.imu_st[2], st.imu_st[3] = c.imu.prev[0], c.imu.prev[1], c.imu.gyro_bias, c.imu.yaw
        st.slip_prev[0], st.slip_prev[1], st.slip_prev[2] = c.slip.prev
        st.cmd[0], st.cmd[1], st.cmd[2] = c.cmd
        st.cmd_time = c.cmd_time
        st.collisions = c.collisions
        st.paused = 1 if c.paused else 0
        st.brake = 1 if c.estop else 0
        st.rtf_target = getattr(c, "rtf_target", 1.0)
        for i, b in enumerate(c.bumpers[:MAXB]):
            st.bumper_pressed[i], st.bumper_contact_t[i], st.bumper_count[i] = int(b.pressed), b.contact_t, b.press_count
        for i, p in enumerate(c.photos[:MAXP]):
            st.photo_detected[i] = int(p.detected)
            st.photo_dist[i] = p.distance
        kin = c.kin
        for cw, w in zip(kin._cw, kin.wheels):
            cw.steer, cw.steer_target, cw.speed, cw.speed_target, cw.cmd_speed = w.steer, w.steer_target, w.speed, w.speed_target, w.cmd_speed
            cw.angle = w.angle
        cc = kin._cc
        cc.shaped[0], cc.shaped[1], cc.shaped[2] = kin.shaped
        cc.vx, cc.vy, cc.wz = kin.vx, kin.vy, kin.wz
        self.lib.sc_rt_set_state(self.h, ctypes.addressof(st))

    def set_flags(self, paused: bool, brake: bool, rtf_target: float):
        """只改少数标志 (housekeeping 每次调用；须在 hold(sync=False) 内且已 pull)"""
        st = self.st
        st.paused, st.brake, st.rtf_target = int(paused), int(brake), float(rtf_target)
        self.lib.sc_rt_set_state(self.h, ctypes.addressof(st))

    # ------------------------------------------------------------------ 运行
    def start(self):
        if not self.started:
            with self.hold():
                pass
            self.lib.sc_rt_start(self.h)
            self.started = True

    def stop(self):
        if self.h:
            self.lib.sc_rt_stop(self.h)
            self.started = False

    def close(self):
        if self.h:
            self.lib.sc_rt_destroy(self.h)
            self.h = None

    # 注意: 不在 __del__ 里释放 C 对象 —— UDP 指令线程可能仍持有旧实例指针 (模型重建时改指向新实例)，
    # 旧实例只停线程不释放 (每次重建几 KB)。

    def set_cmd(self, vx, vy, wz):
        if self._depth:            # 已在 hold() 内: 退出时 push 会带上 core.cmd / cmd_time
            return
        self.lib.sc_rt_set_cmd(self.h, float(vx), float(vy), float(wz))

    def udp_start(self, host: str, port: int) -> int:
        return self.lib.sc_rt_udp_start(self.h, (host or "").encode(), int(port))

    def udp_retarget(self):
        self.lib.sc_rt_udp_retarget(self.h)

    def udp_meta(self):
        out, src = (_D * 5)(), ctypes.create_string_buffer(17)
        self.lib.sc_rt_udp_meta(ctypes.addressof(out), ctypes.addressof(src))
        return int(out[0]), out[1], (out[2], out[3], out[4]), src.value.decode("utf-8", "replace")

    def read_lidar3d(self, i: int, after: int):
        """返回 (seq, t, pose, {points, intensity, line, offset_time}, slice) 或 None；须在 hold(sync=False) 内"""
        xyz, inten, line, ot, sl = self._l3d_bufs[i]
        cnt, frame = ctypes.c_int(), _D()
        seq, t, pose = self._seq
        ok = self.lib.sc_rt_read_lidar3d(self.h, i, after, xyz.ctypes.data, inten.ctypes.data, line.ctypes.data, ot.ctypes.data,
                                         sl.ctypes.data, len(xyz), len(sl), ctypes.byref(cnt), ctypes.addressof(seq),
                                         ctypes.addressof(t), ctypes.addressof(pose), ctypes.byref(frame))
        self.core.lidars3d[i].frame = int(frame.value)          # 退回 Python 扫描时序列接着走
        if not ok:
            return None
        m = cnt.value
        cloud = {"points": xyz[:m].copy(), "intensity": inten[:m].copy(), "line": line[:m].copy(), "offset_time": ot[:m].copy()}
        return seq.value, t.value, (pose[0], pose[1], pose[2]), cloud, sl.copy()

    def read_lidar(self, i: int, after: int):
        """i = -1: 融合扫描。返回 (seq, t, pose(x,y,th), ranges) 或 None；须在 hold(sync=False) 内"""
        buf = self._merged_buf if i < 0 else self._lid_bufs[i]
        seq, t, pose = self._seq
        if not self.lib.sc_rt_read_lidar(self.h, i, after, buf.ctypes.data, len(buf), ctypes.addressof(seq),
                                         ctypes.addressof(t), ctypes.addressof(pose)):
            return None
        return seq.value, t.value, (pose[0], pose[1], pose[2]), buf.copy()
