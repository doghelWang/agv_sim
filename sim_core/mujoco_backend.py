#!/usr/bin/env python3
"""
MuJoCo 仿真后端 —— 物理 (车体平面刚体 + 接触) 与全部光学类传感器 (激光 / 光电 / 相机 / 双目 / ToF) 的统一几何引擎

为什么是 MuJoCo (对比 PyBullet，详见 docs/ENGINE_EVALUATION.md)
  * 官方提供 aarch64 wheel (树莓派 / ROS Humble 的 Python 3.10 可直接 pip 安装)，月度发布；PyBullet 无 ARM wheel 且 2025-01 后停更
  * mj_multiRay (C 实现，释放 GIL，可多线程) 批量射线比 PyBullet rayTestBatch 快约 7 倍，并直接返回法向 → 可做 CPU 光线投射相机
  * 接触求解稳定 (implicitfast)，单步 0.03 ms

建模方式
  * 世界: 地面 (棋盘格/工位标记/地面二维码贴片) + 屋顶 + 墙体 / 货架 / 动态障碍物 (静态盒体，带真实高度)
  * 车体: 平面三自由度 (slide x / slide y / hinge yaw) + 速度执行器；轮组运动学 (先转后走、打滑) 由 ChassisKinematics
          计算期望车体速度，MuJoCo 负责积分与接触约束 → 撞墙时被真实阻挡、推力受限
  * 传感器全部用射线求交 (不依赖 OpenGL，树莓派容器内可用)；可选 SIM_CAMERA_RENDER=gl 走 MuJoCo 光栅渲染 (EGL/OSMesa)
"""

import math
import os
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from typing import Dict, List, Optional, Tuple

import numpy as np

try:
    import mujoco
    MUJOCO_AVAILABLE = True
    MUJOCO_VERSION = mujoco.__version__
except Exception:  # pragma: no cover
    mujoco = None
    MUJOCO_AVAILABLE = False
    MUJOCO_VERSION = None

from .world import CEILING_HEIGHT

# 几何类别 → 颜色 (相机着色)
CAT_COLORS = {
    "floor": (0.78, 0.78, 0.76), "wall": (0.82, 0.83, 0.86), "shelf": (0.20, 0.32, 0.55),
    "obstacle": (0.95, 0.55, 0.15), "ceiling": (0.92, 0.92, 0.94), "station": (0.10, 0.70, 0.35),
    "tag": (0.05, 0.05, 0.05), "robot": (0.95, 0.55, 0.10), "lane": (0.95, 0.80, 0.10),
}
CAT_IDS = {k: i for i, k in enumerate(CAT_COLORS)}
OBSTACLE_COLORS = {"pallet": (0.62, 0.45, 0.25), "person": (0.25, 0.45, 0.85), "shelf": (0.35, 0.40, 0.50)}


def _q(v):
    return f"{v:.5f}"


class MuJoCoBackend:
    def __init__(self, dt: float = 0.01, threads: Optional[int] = None):
        if not MUJOCO_AVAILABLE:
            raise RuntimeError("mujoco 未安装 (pip install mujoco)")
        self.dt = dt
        self.threads = int(threads or os.environ.get("SIM_RAY_THREADS", str(max(1, min(2, os.cpu_count() or 1)))))
        self.pool = ThreadPoolExecutor(self.threads) if self.threads > 1 else None
        self.m = self.d = None
        self.robot_body = -1
        self.robot_geoms: set = set()
        self.geom_cat = np.zeros(0, np.int32)
        self.geom_rgb = np.zeros((0, 3), np.float32)
        self.compile_ms = 0.0
        self.step_ms = 0.0
        self.ray_count = 0
        self.ray_ms = 0.0
        self._pose = (0.0, 0.0, 0.0)
        self._robot = {"footprint": [[0.5, 0.4], [0.5, -0.4], [-0.5, -0.4], [-0.5, 0.4]], "height": 1.0, "mass": 200.0, "clearance": 0.05}
        self._world = None
        self._stations: List[dict] = []
        self._tags: List[dict] = []
        self._lanes: List[Tuple[float, float, float, float]] = []
        self._cams: List[dict] = []            # GL 渲染用的 MJCF 相机 (SIM_CAMERA_RENDER=gl)
        self._renderers: Dict[Tuple[int, int], object] = {}
        self.gl_error = None
        self.lock = threading.RLock()
        self._tls = threading.local()
        self._ray_ver = 0

    # ================================================================== 构建
    def set_robot(self, spec: dict):
        ch = spec["chassis"]
        wheels = spec.get("wheels", [])
        r_min = min([w.get("radius_m", 0.1) for w in wheels] or [0.1])
        self._robot = {"footprint": ch["footprint"], "height": float(ch.get("height_m", 1.0)),
                       "mass": float(ch.get("mass_kg") or 200.0), "clearance": max(0.02, r_min * 0.5)}

    def build(self, world, stations=None, tags=None, lanes=None, keep_state: bool = True):
        """由 World (静态线段 + 障碍物) 生成 MJCF 并编译；keep_state 保留车体位姿/速度"""
        self._world = world
        if stations is not None:
            self._stations = stations
        if tags is not None:
            self._tags = tags
        if lanes is not None:
            self._lanes = lanes
        qpos = qvel = None
        if keep_state and self.d is not None:
            qpos, qvel = self.d.qpos.copy(), self.d.qvel.copy()
        self._st_arr = np.array([[s["x"], s["y"]] for s in self._stations], float).reshape(-1, 2)
        self._tag_arr = np.array([[t["x"], t["y"]] for t in self._tags], float).reshape(-1, 2)
        self._lane_arr = np.array(self._lanes, float).reshape(-1, 4)
        if stations is not None or not hasattr(self, "_floor_tex"):
            self._make_floor_texture(world)
        xml, cats = self._mjcf(world)
        t0 = time.perf_counter()
        with self.lock:
            self.m = mujoco.MjModel.from_xml_string(xml)
            self.d = mujoco.MjData(self.m)
            if qpos is not None and len(qpos) == self.m.nq:
                self.d.qpos[:] = qpos
                self.d.qvel[:] = qvel
            mujoco.mj_forward(self.m, self.d)
            # 传感器专用 MjData: 静态几何的位姿与物理无关 → 射线求交无需与物理步进互斥 (车体几何在 group 1，不参与射线)
            self.d_ray = mujoco.MjData(self.m)
            mujoco.mj_forward(self.m, self.d_ray)
            self._ray_ver = getattr(self, "_ray_ver", 0) + 1
            self.robot_body = self.m.body("agv").id
            self.robot_geoms = {g for g in range(self.m.ngeom) if self.m.geom_bodyid[g] == self.robot_body}
            self.geom_cat = np.array([CAT_IDS.get(cats.get(self.m.geom(g).name, "wall"), 1) for g in range(self.m.ngeom)], np.int32)
            self.geom_rgb = self.m.geom_rgba[:, :3].astype(np.float32).copy()
            self.floor_geom = self.m.geom("floor").id
            self.ceiling_geom = self.m.geom("ceiling").id
            self.xml = xml
        self.compile_ms = (time.perf_counter() - t0) * 1000.0

    def _make_floor_texture(self, world, res: float = 0.02):
        """地面纹理 (2 cm/px): 50 cm 棋盘格 + 车道线 + 工位圆标 + 地面二维码 → 相机着色查表 O(1)；只在换场景时生成"""
        b = world.bounds
        x0, y0 = b[0] - 1.0, b[1] - 1.0
        W = int(math.ceil((b[2] - b[0] + 2.0) / res)); H = int(math.ceil((b[3] - b[1] + 2.0) / res))
        ix = (np.floor((x0 + (np.arange(W) + 0.5) * res) / 0.5)).astype(np.int32)
        iy = (np.floor((y0 + (np.arange(H) + 0.5) * res) / 0.5)).astype(np.int32)
        chk = ((iy[:, None] + ix[None, :]) % 2).astype(np.float32)
        T = np.empty((H, W, 3), np.float32)
        T[:] = np.array(CAT_COLORS["floor"], np.float32)
        T *= (0.92 + 0.08 * chk)[..., None]

        def win(cx0, cy0, cx1, cy1):
            i0 = max(0, int((cx0 - x0) / res)); i1 = min(W, int((cx1 - x0) / res) + 1)
            j0 = max(0, int((cy0 - y0) / res)); j1 = min(H, int((cy1 - y0) / res) + 1)
            X, Y = np.meshgrid(x0 + (np.arange(i0, i1) + 0.5) * res, y0 + (np.arange(j0, j1) + 0.5) * res)
            return (slice(j0, j1), slice(i0, i1)), X, Y

        for (ax, ay, bx, by) in self._lanes:
            sl, X, Y = win(min(ax, bx) - 0.05, min(ay, by) - 0.05, max(ax, bx) + 0.05, max(ay, by) + 0.05)
            ab = np.array([bx - ax, by - ay]); l2 = max(float(ab @ ab), 1e-9)
            t = np.clip(((X - ax) * ab[0] + (Y - ay) * ab[1]) / l2, 0, 1)
            T[sl][np.hypot(X - (ax + t * ab[0]), Y - (ay + t * ab[1])) < 0.025] = CAT_COLORS["lane"]
        for st in self._stations:
            sl, X, Y = win(st["x"] - 0.4, st["y"] - 0.4, st["x"] + 0.4, st["y"] + 0.4)
            T[sl][np.hypot(X - st["x"], Y - st["y"]) < 0.35] = CAT_COLORS["station"]
        for tg in self._tags:
            sl, X, Y = win(tg["x"] - 0.05, tg["y"] - 0.05, tg["x"] + 0.05, tg["y"] + 0.05)
            T[sl][np.maximum(np.abs(X - tg["x"]), np.abs(Y - tg["y"])) < 0.04] = CAT_COLORS["tag"]
        self._floor_tex, self._floor_org = T, (x0, y0, res)

    def _mjcf(self, world) -> Tuple[str, Dict[str, str]]:
        cats: Dict[str, str] = {"floor": "floor", "ceiling": "ceiling"}
        g = []
        b = world.bounds
        cx, cy = (b[0] + b[2]) / 2, (b[1] + b[3]) / 2
        sx, sy = max(5.0, (b[2] - b[0]) / 2 + 2), max(5.0, (b[3] - b[1]) / 2 + 2)
        n_wall = 0
        for i, (x0, y0, x1, y1, h) in enumerate(world.static_segments):
            L = math.hypot(x1 - x0, y1 - y0)
            if L < 0.01:
                continue
            nm = f"s{i}"
            cat = "wall" if h >= CEILING_HEIGHT - 1e-3 else "shelf"
            cats[nm] = cat
            rgb = CAT_COLORS[cat]
            g.append(f'<geom name="{nm}" type="box" pos="{_q((x0 + x1) / 2)} {_q((y0 + y1) / 2)} {_q(h / 2)}" '
                     f'size="{_q(L / 2)} 0.025 {_q(h / 2)}" euler="0 0 {_q(math.atan2(y1 - y0, x1 - x0))}" '
                     f'rgba="{rgb[0]} {rgb[1]} {rgb[2]} 1" group="0"/>')
            n_wall += 1
        for i, o in enumerate(world.obstacles):
            # 动态障碍物 = mocap 刚体: 可逐步移动 (行走人员) 而无需重新编译模型
            nm = f"o{i}"
            cats[nm] = "obstacle"
            z = float(o.get("z", o.get("height", 1.0)))
            rgb = OBSTACLE_COLORS.get(o.get("type"), CAT_COLORS["obstacle"])
            w, h = float(o.get("w", 0.8)), float(o.get("h", 0.8))
            if o.get("type") == "person":
                shape = f'type="cylinder" size="{_q(max(w, h) / 2)} {_q(z / 2)}"'
            else:
                shape = f'type="box" size="{_q(w / 2)} {_q(h / 2)} {_q(z / 2)}"'
            g.append(f'<body name="ob{i}" mocap="true" pos="{_q(float(o["x"]))} {_q(float(o["y"]))} 0" '
                     f'euler="0 0 {_q(float(o.get("yaw", 0.0)))}"><geom name="{nm}" {shape} pos="0 0 {_q(z / 2)}" '
                     f'rgba="{rgb[0]} {rgb[1]} {rgb[2]} 1" group="0"/></body>')
        # 地面装饰 (不参与碰撞): 工位标记、地面二维码、行驶车道线
        for i, s in enumerate(self._stations):
            nm = f"st{i}"
            cats[nm] = "station"
            g.append(f'<geom name="{nm}" type="cylinder" pos="{_q(s["x"])} {_q(s["y"])} 0.0005" size="0.35 0.0005" '
                     f'rgba="0.10 0.70 0.35 1" contype="0" conaffinity="0" group="2"/>')
        for i, t in enumerate(self._tags):
            nm = f"tag{i}"
            cats[nm] = "tag"
            g.append(f'<geom name="{nm}" type="box" pos="{_q(t["x"])} {_q(t["y"])} 0.0008" size="0.04 0.04 0.0008" '
                     f'rgba="0.05 0.05 0.05 1" contype="0" conaffinity="0" group="2"/>')
        for i, (x0, y0, x1, y1) in enumerate(self._lanes):
            L = math.hypot(x1 - x0, y1 - y0)
            if L < 0.05:
                continue
            nm = f"ln{i}"
            cats[nm] = "lane"
            g.append(f'<geom name="{nm}" type="box" pos="{_q((x0 + x1) / 2)} {_q((y0 + y1) / 2)} 0.0003" size="{_q(L / 2)} 0.025 0.0003" '
                     f'euler="0 0 {_q(math.atan2(y1 - y0, x1 - x0))}" rgba="0.95 0.80 0.10 1" contype="0" conaffinity="0" group="2"/>')
        # 车体
        R = self._robot
        P = np.asarray(R["footprint"], float)
        xmax, xmin, ymax, ymin = P[:, 0].max(), P[:, 0].min(), P[:, 1].max(), P[:, 1].min()
        bx, by = (xmax + xmin) / 2, (ymax + ymin) / 2
        hx, hy = (xmax - xmin) / 2, (ymax - ymin) / 2
        H, cl, mass = R["height"], R["clearance"], R["mass"]
        base_h = min(H, 0.35) if H > 0.6 else H
        izz = mass * (4 * hx * hx + 4 * hy * hy) / 12.0
        kv_lin, kv_ang = mass * 40.0, izz * 40.0
        f_lin = mass * 4.0
        cats.update({"agv_base": "robot", "agv_upper": "robot"})
        robot = (f'<body name="agv" pos="0 0 0">'
                 f'<joint name="jx" type="slide" axis="1 0 0" damping="0"/>'
                 f'<joint name="jy" type="slide" axis="0 1 0" damping="0"/>'
                 f'<joint name="jyaw" type="hinge" axis="0 0 1" damping="0"/>'
                 f'<geom name="agv_base" type="box" pos="{_q(bx)} {_q(by)} {_q(cl + base_h / 2)}" size="{_q(hx)} {_q(hy)} {_q(base_h / 2)}" '
                 f'mass="{_q(mass)}" rgba="0.95 0.55 0.1 1" group="1" condim="3" friction="0.4"/>')
        if H - base_h - cl > 0.05:
            up = H - base_h - cl
            robot += (f'<geom name="agv_upper" type="box" pos="{_q(bx)} {_q(by)} {_q(cl + base_h + up / 2)}" '
                      f'size="{_q(hx)} {_q(hy)} {_q(up / 2)}" mass="0.001" rgba="0.85 0.85 0.88 0.6" group="1"/>')
        for cm in self._cams:
            robot += (f'<camera name="{cm["name"]}" pos="{_q(cm["pos"][0])} {_q(cm["pos"][1])} {_q(cm["pos"][2])}" '
                      f'xyaxes="{" ".join(_q(v) for v in cm["xy"])}" fovy="{_q(cm["fovy"])}"/>')
        robot += '</body>'
        xml = f"""<mujoco model="agv_world">
  <compiler angle="radian"/>
  <option timestep="{self.dt}" integrator="implicitfast" gravity="0 0 0" cone="elliptic"/>
  <visual><global offwidth="1280" offheight="960"/><quality shadowsize="0"/>
    <headlight ambient="0.45 0.45 0.45" diffuse="0.35 0.35 0.35" specular="0 0 0"/></visual>
  <asset>
    <texture name="floor_tex" type="2d" builtin="checker" rgb1="0.78 0.78 0.76" rgb2="0.72 0.72 0.70" width="256" height="256"/>
    <material name="floor_mat" texture="floor_tex" texrepeat="{_q(sx * 2)} {_q(sy * 2)}" rgba="1 1 1 1"/>
  </asset>
  <worldbody>
    <light pos="{_q(cx)} {_q(cy)} 5.5" dir="0.3 0.2 -1" directional="true" diffuse="0.6 0.6 0.6" castshadow="false"/>
    <geom name="floor" type="plane" pos="{_q(cx)} {_q(cy)} 0" size="{_q(sx)} {_q(sy)} 0.1" material="floor_mat" contype="0" conaffinity="0" group="2"/>
    <geom name="ceiling" type="box" pos="{_q(cx)} {_q(cy)} {CEILING_HEIGHT + 0.05}" size="{_q(sx)} {_q(sy)} 0.05" rgba="0.92 0.92 0.94 1" contype="0" conaffinity="0" group="2"/>
    {chr(10).join('    ' + s for s in g)}
    {robot}
  </worldbody>
  <actuator>
    <velocity name="vx" joint="jx" kv="{_q(kv_lin)}" forcelimited="true" forcerange="{_q(-f_lin)} {_q(f_lin)}"/>
    <velocity name="vy" joint="jy" kv="{_q(kv_lin)}" forcelimited="true" forcerange="{_q(-f_lin)} {_q(f_lin)}"/>
    <velocity name="wz" joint="jyaw" kv="{_q(kv_ang)}"/>
  </actuator>
</mujoco>"""
        return xml, cats

    def move_obstacles(self, poses):
        """poses: [(index, x, y, yaw)] → 更新 mocap 位姿 (物理数据与射线数据各一份)"""
        with self.lock:
            if self.m is None:
                return
            for i, x, y, yaw in poses:
                try:
                    mid = self.m.body_mocapid[self.m.body(f"ob{i}").id]
                except KeyError:
                    continue
                if mid < 0:
                    continue
                q = (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))
                for d in (self.d, self.d_ray):
                    d.mocap_pos[mid] = (x, y, 0.0)
                    d.mocap_quat[mid] = q
            mujoco.mj_kinematics(self.m, self.d_ray)
            self._ray_ver += 1

    # ================================================================== 物理
    def reset_pose(self, x, y, th):
        with self.lock:
            self.d.qpos[:3] = [x, y, th]
            self.d.qvel[:3] = 0.0
            self.d.ctrl[:] = 0.0
            mujoco.mj_forward(self.m, self.d)

    def step(self, vx_b: float, vy_b: float, wz: float) -> Tuple[float, float, float, float, float, float, Optional[tuple]]:
        """输入: 期望车体速度 (机体系)；输出: 真值位姿/速度 (机体系) + 接触点 (无接触为 None)"""
        t0 = time.perf_counter()
        with self.lock:
            th = self.d.qpos[2]
            c, s = math.cos(th), math.sin(th)
            self.d.ctrl[0] = c * vx_b - s * vy_b
            self.d.ctrl[1] = s * vx_b + c * vy_b
            self.d.ctrl[2] = wz
            mujoco.mj_step(self.m, self.d)
            x, y, th = float(self.d.qpos[0]), float(self.d.qpos[1]), float(self.d.qpos[2])
            vxw, vyw, w = float(self.d.qvel[0]), float(self.d.qvel[1]), float(self.d.qvel[2])
            contact = None
            for i in range(self.d.ncon):
                con = self.d.contact[i]
                g1, g2 = con.geom1, con.geom2
                if (g1 in self.robot_geoms) != (g2 in self.robot_geoms) and con.dist < 0.002:
                    contact = (float(con.pos[0]), float(con.pos[1]))
                    break
        c, s = math.cos(th), math.sin(th)
        self._pose = (x, y, th)
        self.step_ms = 0.9 * self.step_ms + 0.1 * (time.perf_counter() - t0) * 1000.0
        return x, y, th, c * vxw + s * vyw, -s * vxw + c * vyw, w, contact

    # ================================================================== 射线
    def _ray_data(self):
        """每个线程独立的射线 MjData: mj_multiRay 会使用 mjData 的栈，多线程共用同一 MjData 会破坏栈指针 (段错误)。
        按版本号 (换模型/障碍物移动) 从 d_ray 同步 mocap 位姿后做一次运动学。"""
        tl = self._tls
        with self.lock:
            m, ver = self.m, self._ray_ver
            if getattr(tl, "m", None) is not m or getattr(tl, "ver", -1) != ver:
                if getattr(tl, "m", None) is not m:
                    tl.d = mujoco.MjData(m)
                src = self.d_ray
                tl.d.qpos[:] = src.qpos
                if m.nmocap:
                    tl.d.mocap_pos[:] = src.mocap_pos
                    tl.d.mocap_quat[:] = src.mocap_quat
                mujoco.mj_kinematics(m, tl.d)
                tl.m, tl.ver = m, ver
        return tl.m, tl.d

    def _ray_chunk(self, md, pnt, vec, maxr, want_normal):
        m, d = self._ray_data()
        n = len(vec)
        gid = np.full(n, -1, np.int32)
        dist = np.zeros(n)
        nrm = np.zeros(n * 3) if want_normal else None
        mujoco.mj_multiRay(m, d, pnt, np.ascontiguousarray(vec, np.float64).ravel(), self.GEOMGROUP, 1, self.robot_body,
                           gid, dist, nrm, n, float(maxr))
        return gid, dist, nrm

    # 只对 group 0 (墙/货架/障碍物) 做 BVH 求交；地面/屋顶为解析平面，地面贴片 (工位/二维码/车道线) 按坐标查表着色
    GEOMGROUP = np.array([1, 0, 0, 0, 0, 0], np.uint8)

    def cast(self, origin, dirs: np.ndarray, max_range: float, want_normal: bool = False):
        """单原点多射线 → (dist[inf=无回波], geomid[-1], normal[N,3]|None)；dirs 需为单位向量"""
        t0 = time.perf_counter()
        pnt = np.asarray(origin, np.float64)
        n = len(dirs)
        if n == 0:
            return np.zeros(0), np.zeros(0, np.int32), None
        with self.lock:                 # 只在取模型引用时加锁 (障碍物变更会整体替换模型)
            md = (self.m, self.d_ray)
            floor_g, ceil_g = self.floor_geom, self.ceiling_geom
        if self.pool and n >= 2000:
            k = self.threads
            sl = [slice(i * n // k, (i + 1) * n // k) for i in range(k)]
            parts = list(self.pool.map(lambda s: self._ray_chunk(md, pnt, dirs[s], max_range, want_normal), sl))
            gid = np.concatenate([p[0] for p in parts])
            dist = np.concatenate([p[1] for p in parts])
            nrm = np.concatenate([p[2] for p in parts]) if want_normal else None
        else:
            gid, dist, nrm = self._ray_chunk(md, pnt, dirs, max_range, want_normal)
        dist = np.where((gid < 0) | (dist < 0), np.inf, dist)
        # 解析平面: 地面 z=0 / 屋顶
        dz = dirs[:, 2]
        oz = float(pnt[2])
        with np.errstate(divide="ignore", invalid="ignore"):
            tf = np.where(dz < -1e-9, -oz / dz, np.inf)
            tc = np.where(dz > 1e-9, (CEILING_HEIGHT - oz) / dz, np.inf)
        fl = tf < dist
        dist = np.where(fl, tf, dist)
        gid = np.where(fl, floor_g, gid)
        ce = tc < dist
        dist = np.where(ce, tc, dist)
        gid = np.where(ce, ceil_g, gid)
        if want_normal:
            nrm = nrm.reshape(-1, 3)
            nrm[fl] = (0.0, 0.0, 1.0)
            nrm[ce] = (0.0, 0.0, -1.0)
        dist[dist > max_range] = np.inf
        self.ray_count += n
        self.ray_ms += (time.perf_counter() - t0) * 1000.0
        return dist, gid, (nrm if want_normal else None)

    def raycast2d(self, ox, oy, oz, angles, max_range):
        d = np.stack([np.cos(angles), np.sin(angles), np.zeros_like(angles)], 1)
        return self.cast((ox, oy, oz), d, max_range)[0]

    def raycast3d(self, origin, dirs, max_range, min_range=0.0):
        r = self.cast(origin, dirs, max_range)[0]
        r[r < min_range] = np.inf
        return r

    # ================================================================== 相机 (光线投射着色)
    def shade(self, origin, dirs, dist, gid, nrm) -> np.ndarray:
        """Lambert + 环境光 + 距离雾；地面棋盘格。返回 [N,3] float 0~1"""
        n = len(dist)
        col = np.full((n, 3), 0.55, np.float32)       # 背景 (未命中)
        hit = np.isfinite(dist)
        if not hit.any():
            return col
        g = gid[hit]
        base = self.geom_rgb[g].copy()
        P = np.asarray(origin)[None, :] + dirs[hit] * dist[hit][:, None]
        fl = g == self.floor_geom
        if fl.any():
            Q = P[fl, :2]
            T, (x0, y0, res) = self._floor_tex, self._floor_org
            ix = np.clip(((Q[:, 0] - x0) / res).astype(np.int32), 0, T.shape[1] - 1)
            iy = np.clip(((Q[:, 1] - y0) / res).astype(np.int32), 0, T.shape[0] - 1)
            fc = T[iy, ix]
            base[fl] = fc
        N = nrm[hit] if nrm is not None else np.tile([0, 0, 1.0], (int(hit.sum()), 1))
        light = np.array([0.35, 0.25, 0.9]); light /= np.linalg.norm(light)
        lam = np.clip(np.abs(N @ light), 0, 1)
        view = np.clip(np.abs((N * dirs[hit]).sum(1)), 0, 1)       # 面朝相机更亮
        k = 0.35 + 0.45 * lam + 0.2 * view
        fog = np.exp(-dist[hit] / 45.0)
        col[hit] = base * (k * fog)[:, None] + 0.55 * (1 - fog)[:, None]
        return np.clip(col, 0, 1)

    # ================================================================== 可选: OpenGL 光栅渲染 (EGL / OSMesa)
    def set_gl_cameras(self, cams: List[dict]):
        """cams: [{"name","pos":[x,y,z] (机体系),"R":3x3 (link→base),"fovy":deg}]；需重新 build 生效"""
        out = []
        for c in cams:
            R = np.asarray(c["R"])
            xax = R @ np.array([0.0, -1.0, 0.0])      # MuJoCo 相机 x 右
            yax = R @ np.array([0.0, 0.0, 1.0])       # y 上，视线 -z = link +x
            out.append({"name": c["name"], "pos": list(c["pos"]), "xy": list(xax) + list(yax), "fovy": float(c["fovy"])})
        self._cams = out

    def render_gl(self, cam_name: str, width: int, height: int, depth: bool = False, pose=None) -> Optional[np.ndarray]:
        """返回 uint8 RGB (或 float 深度)；GL 不可用时返回 None (调用方退回光线投射)"""
        if self.gl_error:
            return None
        try:
            key = (height, width, depth)
            with self.lock:
                r = self._renderers.get(key)
                if r is None or getattr(r, "_model", None) is not self.m:
                    r = mujoco.Renderer(self.m, height, width)
                    r._model = self.m
                    if depth:
                        r.enable_depth_rendering()
                    self._renderers[key] = r
                if pose is not None:
                    dg = getattr(self, "_d_gl", None)
                    if dg is None or getattr(self, "_d_gl_model", None) is not self.m:
                        dg = self._d_gl = mujoco.MjData(self.m); self._d_gl_model = self.m
                    dg.qpos[:3] = pose
                    mujoco.mj_kinematics(self.m, dg)
                    mujoco.mj_camlight(self.m, dg)
                else:
                    dg = self.d
                r.update_scene(dg, camera=cam_name)
                r.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
                r.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = 0
                return r.render().copy()
        except Exception as e:  # pragma: no cover - 取决于宿主 GL 环境
            self.gl_error = repr(e)[:200]
            return None
