#!/usr/bin/env python3
"""
Navigator —— 执行进程的任务执行器

规划器: dijkstra (拓扑路网) / astar (栅格) / direct (直连) —— 本进程内闭环导引 (Stanley + 先停后转 + 末端对正)
        nav2 —— 通过 ROS 2 NavigateToPose action 交给 Nav2 (ros_bridge 提供)
输入 : SimLink 从仿真进程 REST 拉取的位姿/融合扫描/IO/场景/模型
输出 : cmd_vel → PUT /api/v1/control/cmd_vel；任务状态 → PUT /api/v1/nav/feedback；事件 → GET /api/v1/events
(导引算法源自原 web_teleop_server，迁移至执行进程)
"""

import math
import os
import threading
import time
from collections import deque
from typing import Optional

import numpy as np

from common.events import EventHub
from planning import AStarPlanner, DijkstraPlanner
from planning.dijkstra_planner import SCENARIO_DEFINITIONS, register_scenario
from planning import maneuver, protection
from nav_runtime.slam import SlamLocalizer


# 线路跟随中断且无进展时逐级后退的距离 (m)，NAV2_BACKUP_STEPS="0.2,0.35" 可调；用完仍无进展即放弃
BACKUP_STEPS = tuple(float(v) for v in os.environ.get("NAV2_BACKUP_STEPS", "0.2,0.35").split(",") if v.strip())


class _Recorder:
    """记录任务元数据 (原 flight_recorder 的位置；回放录制在 Web 网关进行)"""

    def __init__(self, nav):
        self.nav = nav

    def start_session(self, session_type="mission", metadata=None):
        self.nav._mission_meta(metadata or {})

    def end_session(self, status="COMPLETED"):
        self.nav._mission_end(status)


class _NoNav2:
    available = False

    def send_goal(self, *a, **k):
        return "Nav2 不可用: 执行进程未运行 ROS 2 / Nav2"

    def cancel(self):
        pass

    def load_map(self, *a):
        return False

    def status(self):
        return {"msgs": False, "server_ready": False, "process": False, "autostart": False, "chassis": None, "localization": None}


class Navigator:
    PLANNERS = ["dijkstra", "astar", "direct", "straight", "nav2"]

    def __init__(self, link, log=print, event_hub: Optional[EventHub] = None):
        self.link = link
        self.log = log
        self.lock = threading.Lock()
        self.event_hub = event_hub or EventHub(max_history=1000)
        self.recorder = _Recorder(self)
        self.nav2 = _NoNav2()
        self.running = True
        self.current_mission_id = 0
        self.active_planner = "dijkstra"
        self.active_scenario = "grid_9_square"
        self.active_chassis_type = "single_steer"
        self.is_paused = False
        self.dynamic_obstacles = []
        self.cfg = {"chassis": {}}
        self.head_offset = 0.6
        self.max_decel = 0.5
        self.dijkstra_planner = DijkstraPlanner(self.active_scenario)
        self.astar_planner = AStarPlanner(resolution=0.1, inflation_radius=0.5)
        self.walls = self.dijkstra_planner.get_walls()
        self.missions = deque(maxlen=50)
        self.mission = None
        self.telemetry = {"x": 0.0, "y": 0.0, "yaw": 0.0, "vx": 0.0, "vy": 0.0, "wz": 0.0, "nav_status": "IDLE",
                          "target_goal": None, "plan_path": [], "path_labels": [], "path_index": 0, "nav_dist_rem": 0.0, "io_states": {},
                          "scan_ranges": [], "scan_angle_min": -math.pi, "scan_angle_inc": 0.0}
        # 定位: 激光 SLAM + 里程计融合 (nav_runtime/slam.py)；导引用融合位姿，真值只用于误差统计
        self.slam = SlamLocalizer(mode=os.environ.get("NAV_LOCALIZATION", "slam"), log=log)
        self.slam.on_event = lambda lv, msg, det: self.event_hub.emit("localization", "SLAM", lv, msg, det)
        self._lidar_pend = {}
        self._loc_n = 0
        link.on_state.append(self._on_state)
        link.on_merged.append(self._on_merged)
        link.on_lidar.append(self._on_lidar)
        link.on_world_change.append(self._on_world)
        link.on_model_change.append(self._on_model)
        link.feedback_fn = self.feedback
        self._estop_resume = None
        # 软件安全层参数: 激光前向走廊减速/停车 (距车头距离)，与光电/触边/急停共同组成避障
        self.approach_left = None      # 末段进站时车头剩余行程 (m)，供安全层缩短停车距离
        self.in_arc = False            # 正在圆弧过弯
        # 保护空间 (planning/protection.py)：随车辆模型下发，3-6 页面可临时调整
        self.prot = protection.effective({})
        self.body = {"head_offset_m": 0.6, "tail_offset_m": 0.6, "left_offset_m": 0.4, "right_offset_m": 0.4}
        self.loaded = False            # 顶升到位 (带载外形生效)
        self.left_off = self.right_off = 0.4
        self.obs = {"front": None, "rear": None, "zone": "clear", "layer": None, "bands": [], "band": 0}
        self.speed_cap = None          # 工步限速 (任务流 move 工步)
        self.track_L = 0.6             # 横向跟踪收敛距离 (m)
        self.half_width = 0.4
        self.tail_offset = 0.6
        self.map_dir = None           # 设置后，场景变化时从仿真进程下载 Nav2 地图到该目录
        threading.Thread(target=self._safety_loop, daemon=True, name="nav-safety").start()
        # C++ 核心 (agv_ros_bridge --core): 保护空间配置/运行模式下发，安全层快照与事件回收
        self._core_cfg_ver = 0          # 配置变化计数 (外形/保护空间/车型)
        self._core_sent = (None, -1)    # (已下发的 C++ 桥实例, 配置版本)
        threading.Thread(target=self._core_loop, daemon=True, name="nav-core-sync").start()

    # ------------------------------------------------------------------ 数据输入 (REST → 本地视图)
    def _on_state(self, st):
        tr = st.get("truth", {})
        od = st.get("odom") or tr
        truth = (tr.get("x", 0.0), tr.get("y", 0.0), tr.get("yaw", 0.0))
        pose = self.slam.on_odom(float(st.get("t", 0.0)), (od.get("x", 0.0), od.get("y", 0.0), od.get("yaw", 0.0)), truth,
                                (od.get("vx", 0.0), od.get("vy", 0.0), od.get("wz", 0.0)))
        self._loc_n += 1
        loc = self.slam.brief() if self._loc_n % 5 == 0 else None
        with self.lock:
            T = self.telemetry
            T["x"], T["y"], T["yaw"] = pose
            # 速度: 机体系，来自里程计 (与实车一致)
            T["vx"], T["vy"], T["wz"] = od.get("vx", 0.0), od.get("vy", 0.0), od.get("wz", 0.0)
            T["truth"] = {"x": truth[0], "y": truth[1], "yaw": truth[2]}
            if loc is not None:
                T["localization"] = loc
            T["io_states"] = self.link.io
            self.is_paused = bool(st.get("paused"))
        js = st.get("joints") or {}
        self._steer = [p for n, p in zip(js.get("names", []), js.get("position", [])) if "steer" in n.lower()]
        di = (self.link.io or {}).get("inputs", {})
        loaded = bool(di.get("di_lift_top")) and not bool(di.get("di_lift_bottom"))
        if loaded != self.loaded:
            self.loaded = loaded
            self._apply_outline()
            h, t, l, r = self.outline()
            self.event_hub.emit("navigation", "OUTLINE", "info", "保护外形切换: " + ("带载" if loaded else "空载"),
                                f"外形 头 {h:.2f} / 尾 {t:.2f} / 左 {l:.2f} / 右 {r:.2f} m" +
                                ("" if (self.prot.get("payload") or {}).get("enabled") or not loaded else " (未配置带载外形，沿用车体)"), {"loaded": loaded})
        with self.lock:
            self.active_chassis_type = st.get("chassis", self.active_chassis_type)
            g = T.get("target_goal")
            if g and T["nav_status"] in ("NAVIGATING", "PLANNING", "OBSTACLE_WAIT") and self.active_planner != "nav2":
                T["nav_dist_rem"] = round(math.hypot(g["x"] - T["x"], g["y"] - T["y"]), 2)

    def _on_lidar(self, name, meta, payload):
        """原始 2D 激光帧 → SLAM (角度精确；同一时刻各激光合为一帧)"""
        if meta.get("type") == "3d" or "ranges" not in payload:
            return
        import numpy as np
        cfgs = {l["name"]: l for l in self.link.sensors.get("lidars", []) if l.get("type", "2d") == "2d"}
        cfg = cfgs.get(name)
        if cfg is None:
            return
        m = cfg.get("mount") or {}
        r = np.asarray(payload["ranges"], float)
        a = float(meta.get("angle_min", 0.0)) + float(meta.get("angle_increment", 0.0)) * np.arange(len(r))
        sign = 1.0 if math.cos(float(m.get("roll", 0.0))) * math.cos(float(m.get("pitch", 0.0))) >= 0 else -1.0
        ang = float(m.get("yaw", 0.0)) + sign * a
        ok = np.isfinite(r) & (r > float(meta.get("range_min", 0.05)) + 1e-3)
        mx, my = float(m.get("x", 0.0)), float(m.get("y", 0.0))
        px, py = mx + r[ok] * np.cos(ang[ok]), my + r[ok] * np.sin(ang[ok])
        t = round(float(meta.get("t", 0.0)), 3)
        with self.lock:
            pend = self._lidar_pend
            e = pend.setdefault(t, {})
            e[name] = (px, py, mx, my)
            ready = [k for k in pend if len(pend[k]) >= len(cfgs) or k < t - 0.25]
            frames = [(k, pend.pop(k)) for k in sorted(ready)]
        for k, fr in frames:
            P = list(fr.values())
            self.slam.on_points(k, np.concatenate([p[0] for p in P]), np.concatenate([p[1] for p in P]), None,
                                np.concatenate([np.full(len(p[0]), p[2]) for p in P]),
                                np.concatenate([np.full(len(p[0]), p[3]) for p in P]))

    def _slam_uses_merged(self):
        return not any(l.get("type", "2d") == "2d" for l in self.link.sensors.get("lidars", []))

    def _on_merged(self, d):
        rmax = float(d.get("range_max", 12.0))
        r0 = d.get("ranges", [])
        if isinstance(r0, np.ndarray):          # SimLink 二进制帧 (inf = 无回波)
            rs = np.where(np.isfinite(r0), r0, rmax).astype(float).tolist()
        else:
            rs = [rmax if r is None else float(r) for r in r0]
        a0, inc = float(d.get("angle_min", -math.pi)), float(d.get("angle_increment", 0.0))
        with self.lock:
            self.telemetry["scan_ranges"] = rs
            self.telemetry["scan_angle_min"] = a0
            self.telemetry["scan_angle_inc"] = inc
        self._scan_body = (rs, a0, inc, rmax)
        if self._slam_uses_merged():           # 只有 3D 激光时用合并扫描做 SLAM
            self.slam.on_scan(float(d.get("t", 0.0)), rs, a0, inc, rmax)
        if self._core() is not None:          # C++ 核心已算好各档走廊最近障碍 (安全层快照)，这里只保留点集
            r = np.asarray(rs, float)
            ok = (r > 0.02) & (r < rmax - 1e-3)
            a = a0 + inc * np.nonzero(ok)[0]
            self._pts = (r[ok] * np.cos(a), r[ok] * np.sin(a))
            return
        # 各档防护区走廊内最近障碍 (机体系；走廊 = 当前外形两侧外扩 side；距离从车头/车尾算起)
        r = np.asarray(rs, float)
        ok = (r > 0.02) & (r < rmax - 1e-3)
        a = a0 + inc * np.nonzero(ok)[0]
        px, py = r[ok] * np.cos(a), r[ok] * np.sin(a)
        self._pts = (px, py)
        h, t, lo, ro = self.outline()
        bands = []
        for f in self.prot["fields"]:
            m = (py <= lo + f["side"]) & (py >= -(ro + f["side"]))
            fr = px[m & (px > 0)] - h
            rr = -px[m & (px <= 0)] - t
            fr, rr = fr[fr > -0.05], rr[rr > -0.05]
            bands.append((round(max(0.0, float(fr.min())), 2) if fr.size else None,
                          round(max(0.0, float(rr.min())), 2) if rr.size else None))
        self.obs["bands"] = bands
        with self.lock:
            v = self.telemetry.get("vx", 0.0)
        i, _ = protection.field_for_speed(self.prot, v)
        self.obs["band"] = i
        self.obs["front"], self.obs["rear"] = bands[i] if bands else (None, None)

    def _on_world(self, w):
        sid = w.get("id", self.active_scenario)
        if w.get("scenario_def"):
            register_scenario(w["scenario_def"])          # 场景定义以仿真进程为准
        if self.map_dir:
            self.link.fetch_map(self.map_dir, sid)        # Nav2 栅格地图同样来自仿真进程
        self.slam.on_scenario(sid)
        with self.lock:
            changed = sid != self.active_scenario
            self.active_scenario = sid
            self.dijkstra_planner.set_scenario(sid)
            self.walls = self.dijkstra_planner.get_walls()
            self.dynamic_obstacles = list(w.get("obstacles", []))
        if changed:
            self.cancel_nav()
            self.nav2.load_map(sid)

    def _on_model(self, m):
        ch = m.get("chassis", {})
        with self.lock:
            self.cfg = {"chassis": ch}
            self.body = dict(ch)
            self.max_decel = float(ch.get("max_decel_mps2", ch.get("max_accel_mps2", 0.5)))
            self.prot = protection.effective(m)
            self._runtime_prot = None
        self._apply_outline()

    # ------------------------------------------------------------------ 保护空间
    def outline(self):
        """当前外形 (head, tail, left, right)：空载=车体，带载=车体∪负载"""
        return protection.outline(self.body, self.prot, self.loaded)

    def _apply_outline(self):
        self._core_cfg_ver = getattr(self, "_core_cfg_ver", 0) + 1
        h, t, l, r = self.outline()
        self.slam.set_body(h, t, max(l, r))
        with self.lock:
            self.head_offset, self.tail_offset = h, t
            self.left_off, self.right_off = l, r
            self.half_width = max(l, r)
            hw = self.half_width
            self.dijkstra_planner.robot_half_width = hw
            self.dijkstra_planner.set_footprint(h, t, hw, self.corner_radius(), self.prot["body_margin"],
                                                self.prot.get("corner_mode", "auto"))
            self.dijkstra_planner.robot_circum_radius = max(math.hypot(h, hw), math.hypot(t, hw))
            self.astar_planner = AStarPlanner(resolution=0.1, inflation_radius=hw + self.prot["fields"][0]["side"] + self.prot["body_margin"])

    # ------------------------------------------------------------------ C++ 核心同步
    def _core(self):
        node = getattr(self.nav2, "node", None)
        cpp = getattr(node, "cpp", None)
        return cpp if cpp is not None and getattr(node, "core", False) and cpp.running() else None

    def _core_config(self) -> dict:
        h, t, l, r = self.outline()
        photos = [{"name": p["name"], "di": p["di"], "x": float(p["mount"]["x"]), "y": float(p["mount"]["y"]),
                   "yaw": float(p["mount"]["yaw"])} for p in (self.link.sensors or {}).get("photoelectric", [])]
        l2d = [x for x in (self.link.sensors or {}).get("lidars", []) if x.get("type", "2d") == "2d"]
        rl = {}
        if l2d:          # 精定位用视场最宽、束数最多的 2D 激光原始帧
            L = max(l2d, key=lambda x: (float(x.get("fov_deg", 0)), int(x.get("beams", 0))))
            mt = L.get("mount") or {}
            rl = {"name": L["name"], "x": float(mt.get("x", 0)), "y": float(mt.get("y", 0)), "yaw": float(mt.get("yaw", 0)),
                  "roll": float(mt.get("roll", 0))}
        sen = self.link.sensors or {}
        media = {"lidars3d": [{"name": x["name"], "topic": x.get("topic_hint") or f"/points/{x['name']}",
                               "frame_id": x.get("frame_id") or f"{x['name']}_link"} for x in sen.get("lidars", []) if x.get("type") == "3d"],
                 "cameras": [{"name": c["name"], "frame_id": c.get("frame_id", ""), "streams": list(c.get("streams", [])),
                              "K": [float(k) for k in c.get("K", [])], "baseline_m": float(c.get("baseline_m", 0.0) or 0.0)}
                             for c in sen.get("camera_streams", [])]}
        return {"prot": self.prot, "outline": [h, t, l, r], "photos": photos, "refine_lidar": rl, "media": media,
                "max_decel": float(self.cfg.get("chassis", {}).get("max_decel_mps2", 0.5) or 0.5),
                "loc_stale_s": float(os.environ.get("LOC_STALE_S", "1.0"))}

    def _core_mode(self) -> dict:
        m = dict(getattr(self.nav2.node, "mode_flags", None) or {})
        with self.lock:
            st = self.telemetry.get("nav_status")
        m["nav2_forward"] = self.active_planner == "nav2" and st in ("NAVIGATING", "PLANNING", "OBSTACLE_WAIT")
        m["speed_cap"] = float(self.speed_cap or 0.0)
        return m

    def push_core(self, force_mode: bool = False):
        cpp = self._core()
        if cpp is None:
            return
        if self._core_sent != (cpp, self._core_cfg_ver) or cpp.on_safety is None:
            cpp.on_safety = self._on_core_safety
            cpp.on_guide = self._on_core_guide
            cpp.on_sevent = lambda e: self.event_hub.emit(e.get("cat", "sensors"), e.get("type", "SAFETY"), e.get("level", "info"),
                                                          e.get("title", ""), e.get("msg", ""), {"source": "cpp"})
            try:
                cpp.send_config(self._core_config())
                self._core_sent = (cpp, self._core_cfg_ver)
            except Exception as e:  # noqa
                self.log(f"[navigator] 下发安全层配置失败: {e}")
        cpp.send_mode(self._core_mode(), force=force_mode)

    def _core_loop(self):
        n = 0
        while self.running:
            time.sleep(0.2)
            n += 1
            try:
                self.push_core(force_mode=n % 10 == 0)      # 每 2 s 强制重发一次 MODE (C++ 重启后恢复)
            except Exception as e:
                if n % 50 == 1:                             # 最多 10 s 记一次
                    self.log(f"[navigator] C++ 核心同步失败: {e!r}")

    def _guide_cpp(self):
        """自研导引在 C++ 核心里执行 (NAV_CPP_GUIDE=0 用本进程的 _autonomous_guidance_loop)"""
        return self._core() if os.environ.get("NAV_CPP_GUIDE", "1") != "0" else None

    def _start_guide_cpp(self, cpp, mission_id, waypoints, target_yaw, replan_left, corners, labels):
        ch = self.cfg.get("chassis", {})
        h, t, l, r = self.outline()
        cs = []
        for i in range(len(waypoints)):
            c = corners[i] if corners and i < len(corners) else None
            if not c:
                cs.append(None)
                continue
            d, clr = c
            if "rotate" in d:
                cs.append({"kind": 2, "rot": float(d["rotate"]), "clr": float(clr)})
            else:
                cs.append({"kind": 1, "R": d["R"], "d": d["d"], "turn": d["turn"], "heading": d["heading"], "v": d.get("v", 0.35),
                           "cx": d["cx"], "cy": d["cy"], "clr": float(clr)})
        static = [list(map(float, w[:4])) for w in self.dijkstra_planner._static_segments()]
        with self.lock:
            planner, chassis = self.active_planner, self.active_chassis_type
        msg = {"mid": mission_id, "wps": [[float(p[0]), float(p[1])] for p in waypoints],
               "labels": [x or "" for x in (labels or [])], "corners": cs, "yaw": float(target_yaw), "replan_left": replan_left,
               "planner": planner, "chassis": chassis, "corner_mode": self.prot.get("corner_mode", "auto"),
               "max_v": min(1.2, ch.get("max_speed_mps", 1.5)), "max_w": min(1.6, ch.get("max_ang_speed_radps", 2.0)),
               "max_decel": self.max_decel, "max_ang_decel": float(ch.get("max_ang_decel_radps2", 1.0) or 1.0),
               "track_L": self.track_L, "head": h, "tail": t, "hw": max(l, r), "corner_radius": self.corner_radius(),
               "body_margin": self.prot["body_margin"], "rotate_margin": self.prot["rotate_margin"],
               "arrive_tol": self.prot["docking"]["arrive_tolerance"],
               "segs": [float(v) for sg in self._obstacle_segments_all() for v in sg[:4]],
               "refine_segs": [v for sg in static for v in sg],
               "refine": os.environ.get("NAV2_REFINE_LOC", "1") != "0", "refine_dist": 0.8}
        self._guide_targets = getattr(self, "_guide_targets", {})
        self._guide_targets[mission_id] = (waypoints[-1][0], waypoints[-1][1], target_yaw, replan_left)
        self.push_core(force_mode=True)
        cpp.send_guide(msg)

    def _on_core_guide(self, m: dict):
        """C++ 自研导引: 状态 (NAVIGATING/OBSTACLE_WAIT、当前路段) 与结束 (ARRIVED/FAILED/REPLAN/ABORT)"""
        mid = m.get("mid")
        if m.get("k") == "guide":
            with self.lock:
                if self.current_mission_id == mid and self.telemetry["nav_status"] in ("NAVIGATING", "PLANNING", "OBSTACLE_WAIT"):
                    self.telemetry["nav_status"] = m.get("st", "NAVIGATING")
                    self.telemetry["path_index"] = int(m.get("idx", 1))
            return
        res = m.get("result")
        tgt = getattr(self, "_guide_targets", {}).pop(mid, None)
        with self.lock:
            if self.current_mission_id != mid:
                return
        if res == "REPLAN" and tgt:
            threading.Thread(target=self.send_nav_goal, args=(tgt[0], tgt[1], tgt[2]), kwargs={"_replan_left": tgt[3] - 1},
                             daemon=True).start()
            return
        if res == "ARRIVED":
            with self.lock:
                self.telemetry["nav_status"] = "ARRIVED"
                self.telemetry["plan_path"] = []
                self.telemetry["nav_dist_rem"] = 0.0
                fx, fy, fyaw = self.telemetry["x"], self.telemetry["y"], self.telemetry["yaw"]
            self.recorder.end_session("ARRIVED")
            ty = tgt[2] if tgt else fyaw
            dock = round(ty / (math.pi / 2.0)) * (math.pi / 2.0)
            dev = abs(round(math.degrees(math.atan2(math.sin(dock - fyaw), math.cos(dock - fyaw))), 2))
            self.event_hub.emit("navigation", "MISSION_ARRIVED", "success", f"任务 #{mid} 停靠到位完成",
                                f"已就位停靠，位置: ({fx:.2f}, {fy:.2f}) | 航向对齐偏差: {dev}° (C++ 导引)",
                                {"mission_id": mid, "x": round(fx, 3), "y": round(fy, 3), "yaw": round(fyaw, 3), "dev_deg": dev})
        elif res == "FAILED":
            with self.lock:
                self.telemetry["nav_status"] = "FAILED"
            self.recorder.end_session("FAILED")

    def _on_core_safety(self, m: dict):
        """C++ 安全层快照 (10~20 Hz): 各档走廊最近障碍；Nav2 执行中防护区状态以 C++ 为准"""
        bands = [tuple(b) for b in m.get("bands", [])]
        self.obs["bands"] = bands
        i = int(m.get("band", 0))
        self.obs["band"] = i
        self.obs["front"], self.obs["rear"] = bands[i] if i < len(bands) else (None, None)
        if m.get("nav2"):
            self.obs["zone"] = m.get("zone") or "clear"
            self.obs["layer"] = m.get("layer") or None
            self.obs["photo_ignored"] = m.get("photo_ignored") or []
        self.link.stats["cpp_core"] = {k: m.get(k) for k in ("stream", "state_frames", "cmd_sent", "cmd_errors", "udp")}

    def set_protection(self, patch: dict, persist_note: str = "") -> dict:
        """运行时调整保护空间 (不写回模型；写回由平台模型补全保存)"""
        base = dict(self.prot)
        base.pop("fields_auto", None) if patch.get("fields") else None
        self.prot = protection.effective(protection._merge(base, patch) if not patch.get("fields") else dict(protection._merge(base, patch), fields=patch["fields"]), self.body)
        self._runtime_prot = True
        self._apply_outline()
        return self.protection_view()

    def protection_view(self) -> dict:
        v = self.telemetry.get("vx", 0.0)
        i = self.obs.get("band", 0)
        return {"config": self.prot, "loaded": self.loaded, "outline": self.outline(), "band": i,
                "band_name": self.prot["fields"][i]["name"] if self.prot["fields"] else None,
                "polygons": protection.polygons(self.body, self.prot, self.loaded, i),
                "zone": self.obs.get("zone"), "layer": self.obs.get("layer"),
                "front": self.obs.get("front"), "rear": self.obs.get("rear"), "runtime_modified": bool(getattr(self, "_runtime_prot", None)),
                "reverse": v < -0.02}

    def _allowed_speed(self, sign: int, left=None):
        """按防护区分档求允许速度: 取停车距离放得下的最高一档的 v_max；最低档都放不下 → 0。
        返回 (允许速度, 当前档距离, 当前档停车距离)"""
        fs, bands = self.prot["fields"], self.obs.get("bands") or []
        allowed, d_hit, need_hit = None, None, None
        for i, f in enumerate(fs):
            d = bands[i][0 if sign > 0 else 1] if i < len(bands) else None
            need = f["front"] if sign > 0 else f["rear"]
            if left is not None:
                need = min(need, left + self.prot["docking"]["front"])
            if d is not None and d < need:
                d_hit, need_hit = d, need
                break
            allowed = f["v_max"] if i < len(fs) - 1 else 99.0
        return (0.0 if allowed is None else allowed), d_hit, need_hit

    def _rotation_blocked(self, direction: float) -> bool:
        """原地转向防护: 外形外扩 rotate_margin，向 direction 转 rotate_lookahead 弧度的扫掠区内出现新的点
        (圆弧与矩形精确求交，见 planning/sweep.py；原来只看 3 个角度，贴边的点会在采样之间漏过)"""
        pts = getattr(self, "_pts", None)
        if pts is None or not len(pts[0]):
            return False
        from planning.sweep import arc_hits_box
        px, py = np.asarray(pts[0], float), np.asarray(pts[1], float)
        h, t, l, r = self.outline()
        m = self.prot["rotate_margin"]
        box = (-t - m, h + m, -r - m, l + m)
        now = (px < h + m) & (px > -t - m) & (py < l + m) & (py > -r - m)
        if now.all():
            return False
        # 车体转 +φ ⇔ 点在车体系下转 -φ
        return bool(np.any(arc_hits_box(px[~now], py[~now], -direction * self.prot["rotate_lookahead_rad"], box)))

    def refresh_obstacles(self):
        self.link.refresh_world()

    # ------------------------------------------------------------------ 输出
    def publish_cmd_vel(self, vx: float, vy: float = 0.0, wz: float = 0.0):
        vx, vy, wz = self.safety_filter(vx, vy, wz)
        self.link.send_cmd(vx, vy, wz, source=f"nav:{self.active_planner}")

    # ------------------------------------------------------------------ 安全层 (消费仿真进程的光电/触边/急停数据)
    def _photo_sides(self, skip_diag: bool = False, apply_envelope: bool = True):
        """触发且检测点落在光电保护包络内的光电，按安装朝向分组 → {'front': [...], 'rear': [...], 'left': [...], 'right': [...]}
        包络见 planning/protection.photo_envelope (默认随速度档取停车区)；距前方停车点很近时屏蔽前向光电 (muting)。
        skip_diag: 圆弧过弯时屏蔽斜向角部光电 (过弯已做车体净空预检)"""
        di = (self.link.io or {}).get("inputs", {})
        dist = getattr(self.link, "photos", {}) or {}
        out = {"front": [], "rear": [], "left": [], "right": []}
        env = protection.photo_envelope(self.body, self.prot, self.loaded, self.obs.get("band", 0)) if apply_envelope else None
        ph = self.prot.get("photo") or {}
        mute = self.approach_left is not None and self.approach_left < float(ph.get("mute_near_stop", 0.1))
        ignored = []
        for p in (self.link.sensors or {}).get("photoelectric", []):
            if not di.get(p["di"]):
                continue
            yaw = p["mount"]["yaw"]
            if skip_diag and 0.3 < abs(math.atan2(math.sin(yaw), math.cos(yaw))) < 2.8:
                continue
            c, s = math.cos(yaw), math.sin(yaw)
            side = "front" if c > 0.3 else ("rear" if c < -0.3 else ("left" if s > 0 else "right"))
            if side == "front" and mute:
                ignored.append(p["name"])
                continue
            if env is not None:
                d = (dist.get(p["name"]) or {}).get("distance_m")
                if d is None:                         # 无检测距离 (如 DI 被外部强制置位): 保守响应
                    out[side].append(p["name"])
                    continue
                d = float(d)
                hx, hy = p["mount"]["x"] + d * c, p["mount"]["y"] + d * s
                H, T, Lf, Rt = env
                if not (-T <= hx <= H and -Rt <= hy <= Lf):
                    ignored.append(p["name"])       # 检测点在包络外 (如斜向光电扫到侧面墙体): 不响应
                    continue
            out[side].append(p["name"])
        self.obs["photo_ignored"] = ignored
        return out

    def safety_filter(self, vx, vy, wz):
        """执行进程侧软件安全: 急停/触边 → 零速；光电触发 → 禁止朝该侧平移 (与仿真进程硬件互锁互为冗余)"""
        io = (self.link.io or {}).get("inputs", {})
        if io.get("di_estop"):
            return 0.0, 0.0, 0.0
        # 定位停更保护: Nav2 执行中 slam_toolbox 的 map→base 定位超过 LOC_STALE_S (默认 1 s) 没有更新 → 停车等待。
        # Nav2 控制器会拿过期的 map→odom 换算路径 (手机 proot 下实测停更 11 s，RPP 冲过终点撞墙)
        ext = getattr(self.slam, "ext", None)
        if self.active_planner == "nav2" and ext is not None and self.slam.ros_active() and getattr(ext, "last_tf_wall", 0.0):
            age = time.time() - ext.last_tf_wall
            if age > float(os.environ.get("LOC_STALE_S", "1.0")):
                if not self.obs.get("_loc_stale"):
                    self.obs["_loc_stale"] = True
                    self.event_hub.emit("localization", "LOC_STALE", "warning", "定位停更，停车等待",
                                        f"slam_toolbox 定位已 {age:.1f} s 未更新，Nav2 路径换算不可信", {"age_s": round(age, 2)})
                return 0.0, 0.0, 0.0
            if self.obs.get("_loc_stale"):
                self.obs["_loc_stale"] = False
                self.event_hub.emit("localization", "LOC_RESUME", "info", "定位恢复", "", {})
        cap = self.speed_cap
        if cap and cap > 0:
            v = math.hypot(vx, vy)
            if v > cap:
                vx, vy = vx * cap / v, vy * cap / v
        P = self.prot
        if P.get("enabled", True) and abs(vx) > 1e-3 and not (self.in_arc and vx > 0):
            # 行驶防护区 (圆弧过弯时改由过弯循环按弧线扫掠区检查)
            sign = 1 if vx > 0 else -1
            left = self.approach_left if sign > 0 else None      # 末段进站: 只关心车头剩余行程内的障碍
            allowed, d_hit, need = self._allowed_speed(sign, left)
            if left is not None and allowed > 0:
                # 防护区按剩余行程缩短的前提是"车会停在停车点"。外部控制器 (Nav2) 不一定停得住 (手机上 slam 的
                # map→odom 停更时 RPP 会冲过终点)：若按完整防护区本应减速/停车，则速度封顶到剩余行程内能停下的
                # v = √(2·a·left)；冲过停车点 (left = 0) 即停车
                full, _, _ = self._allowed_speed(sign, None)
                if full < allowed:
                    a = 0.5 * float(self.cfg.get("chassis", {}).get("max_decel_mps2", 0.5) or 0.5)
                    v_stop = max(0.01, math.sqrt(2.0 * a * left)) if left > 0.002 else 0.0   # 与导引末段爬行一致
                    allowed = min(allowed, max(full, v_stop))
            zone, layer = "clear", None
            if allowed <= 1e-6:
                vx, zone, layer = 0.0, "stop", "field_" + ("front" if sign > 0 else "rear")
            elif abs(vx) > allowed:
                vx, zone, layer = sign * allowed, "slow", "field_" + ("front" if sign > 0 else "rear")
            else:
                i, f = protection.field_for_speed(P, vx)
                bands = self.obs.get("bands") or []
                d = bands[i][0 if sign > 0 else 1] if i < len(bands) else None
                if d is not None and d < (f["front"] if sign > 0 else f["rear"]) * P["slow_ratio"]:
                    zone = "warn"
            self._zone(zone, layer, d_hit, need)
        elif P.get("enabled", True) and abs(vx) < 0.05 and abs(wz) > 0.05:
            # 原地转向防护
            if self._rotation_blocked(1.0 if wz > 0 else -1.0):
                wz = 0.0
                self._zone("stop", "rotate")
            elif self.obs.get("layer") == "rotate":
                self._zone("clear")
        ph = self._photo_sides(skip_diag=self.in_arc)
        if ph["front"] and vx > 0:
            vx = 0.0
        if ph["rear"] and vx < 0:
            vx = 0.0
        if ph["left"] and vy > 0:
            vy = 0.0
        if ph["right"] and vy < 0:
            vy = 0.0
        return vx, vy, wz

    def _zone(self, zone: str, layer: str = None, d=None, need=None):
        """防护区状态变化 → OBS 事件 (正常巡航 / 预警 / 减速避让 / 停车等待)；layer: field_front/field_rear/rotate"""
        prev = self.obs.get("zone", "clear")
        self.obs["layer"] = layer if zone in ("slow", "stop") else None
        if zone == "warn":
            if prev == "clear" and time.time() - self.obs.get("_tw", 0) > 3.0:
                self.obs["_tw"] = time.time()
                self.event_hub.emit("sensors", "OBS_WARN", "info", "近距避障: 预警区有障碍",
                                    f"当前档 ({self.prot['fields'][self.obs.get('band', 0)]['name']}) 预警区内探测到障碍，保持速度并准备降档", {"band": self.obs.get("band", 0)})
            self.obs["zone"] = "warn"
            return
        if zone == prev or (prev == "warn" and zone == "clear"):
            self.obs["zone"] = zone
            return
        self.obs["zone"] = zone
        self._zone_info = (layer, d, need)
        now = time.time()
        if zone == "clear" and now - self.obs.get("_t", 0) < 0.5:
            return
        self.obs["_t"] = now
        layer, dd, need = getattr(self, "_zone_info", (None, None, None))
        i = self.obs.get("band", 0)
        band = self.prot["fields"][i]["name"] if self.prot["fields"] else ""
        where = {"field_front": "前向防护区", "field_rear": "后向防护区", "rotate": "转向防护区"}.get(layer, "防护区")
        info = {"layer": layer, "dist": dd, "need": need, "band": band, "loaded": self.loaded}
        if zone == "slow":
            self.event_hub.emit("sensors", "OBS_SLOW", "warning", "近距避障: 减速避让",
                                f"{where}: 障碍 {dd} m < {need} m，按防护区分档降速" + (" (带载外形)" if self.loaded else ""), info)
        elif zone == "stop":
            if layer == "rotate":
                self.event_hub.emit("sensors", "OBS_STOP", "danger", "近距避障: 停车等待",
                                    f"{where}: 原地转向扫掠区 (外扩 {self.prot['rotate_margin']} m) 内有障碍，停止转向", info)
            else:
                self.event_hub.emit("sensors", "OBS_STOP", "danger", "近距避障: 停车等待",
                                    f"{where}: 障碍 {dd} m < 最低档停车距离 {need} m，停车等待" + (" (带载外形)" if self.loaded else ""), info)
        elif prev in ("slow", "stop"):
            self.event_hub.emit("sensors", "OBS_CLEAR", "success", "障碍解除，恢复巡航", "", {})

    def safety_state(self) -> dict:
        di = (self.link.io or {}).get("inputs", {})
        return {"estop": bool(di.get("di_estop")),
                "bumpers": sorted(k[len("di_bumper_"):] for k, v in di.items() if k.startswith("di_bumper_") and v),
                "photo_raw": self._photo_sides(apply_envelope=False), "photo": self._photo_sides(),
                "photo_ignored": list(self.obs.get("photo_ignored") or []),
                "obstacle": {"front": self.obs.get("front"), "rear": self.obs.get("rear"), "zone": self.obs.get("zone", "clear"),
                             "layer": self.obs.get("layer"), "band": self.obs.get("band", 0)},
                "params": self.safety_params, "speed_cap": self.speed_cap, "protection": self.protection_view()}

    @property
    def safety_params(self) -> dict:
        """旧接口兼容视图 (减速/停车距离取当前档)"""
        P = self.prot
        i = self.obs.get("band", 0) if P["fields"] else 0
        f = P["fields"][i]
        return {"enabled": P["enabled"], "stop_dist": f["front"], "slow_dist": round(f["front"] * P["slow_ratio"], 2),
                "slow_speed": P["slow_speed"], "corridor_margin": f["side"], "corner_radius": P.get("corner_radius"),
                "protection": P}

    def _safety_loop(self):
        """急停 → 任务挂起 (SAFETY_STOP) 并在解除后自动恢复；触边压下 → 任务终止 (BUMPER_STOP)，需人工复位"""
        prev_estop, prev_bump = False, False
        while self.running:
            time.sleep(0.05)
            st = self.safety_state()
            busy = self.telemetry.get("nav_status") in ("NAVIGATING", "PLANNING", "OBSTACLE_WAIT", "SAFETY_STOP")
            if st["estop"] and not prev_estop:
                self.event_hub.emit("safety", "ESTOP", "danger", "急停触发", "执行进程暂停当前任务，等待急停复位", st)
                if busy:
                    self.nav2.cancel() if self.active_planner == "nav2" else None
                    cpp = self._guide_cpp()
                    if cpp is not None:
                        cpp.send_guide_cancel()             # C++ 自研导引随任务挂起一并停止 (复位后按原目标重新下发)
                    with self.lock:
                        self._estop_resume = self.telemetry.get("target_goal")
                        self.current_mission_id += 1          # 结束当前导引线程
                        self.telemetry["nav_status"] = "SAFETY_STOP"
            if not st["estop"] and prev_estop:
                g = getattr(self, "_estop_resume", None)
                self._estop_resume = None
                self.event_hub.emit("safety", "ESTOP_RELEASE", "success", "急停复位", "恢复任务" if g else "", {})
                if g:
                    threading.Thread(target=self.send_nav_goal, args=(g["x"], g["y"], g.get("yaw", 0.0)), daemon=True).start()
            bump = bool(st["bumpers"])
            if bump and not prev_bump and busy and self.telemetry.get("nav_status") != "SAFETY_STOP":
                with self.lock:
                    self.current_mission_id += 1
                    self.telemetry["nav_status"] = "BUMPER_STOP"
                    self.telemetry["plan_path"] = []
                self.nav2.cancel()
                cpp = self._guide_cpp()
                if cpp is not None:
                    cpp.send_guide_cancel()
                self._mission_end("BUMPER_STOP")
                self.recorder.end_session("BUMPER_STOP")
                for _ in range(3):
                    self.link.send_cmd(0.0, 0.0, 0.0, source="nav:safety")
                self.event_hub.emit("safety", "BUMPER_STOP", "danger", "防撞触边触发，任务终止",
                                    f"触边 {','.join(st['bumpers'])} 压下；请移除障碍后重新下发任务", st)
            prev_estop, prev_bump = st["estop"], bump

    def set_planner_type(self, planner_type: str) -> bool:
        if planner_type not in self.PLANNERS:
            return False
        if planner_type == "nav2" and not getattr(self.nav2, "available", False):
            self.event_hub.emit("navigation", "PLANNER_SWITCH", "danger", "Nav2 不可用", "执行进程未连接 ROS 2 Nav2", {})
            return False
        with self.lock:
            self.active_planner = planner_type
        self.event_hub.emit("navigation", "PLANNER_SWITCH", "info", "导航规划引擎切换", f"执行进程规划器 → {planner_type}", {"planner_type": planner_type})
        return True

    # ------------------------------------------------------------------ 任务记录
    def _new_mission(self, mid, x, y, yaw, planner):
        m = {"id": mid, "planner": planner, "goal": {"x": x, "y": y, "yaw": yaw}, "status": "PLANNING",
             "created": time.time(), "ended": None, "meta": {}}
        with self.lock:
            prev = self.mission
            if prev and not prev["ended"]:
                prev["ended"] = time.time()
                prev["status"] = "SUPERSEDED"      # 重规划/新任务替代
            self.mission = m
            self.missions.append(m)
        return m

    def _mission_meta(self, meta):
        with self.lock:
            if self.mission and self.mission["id"] == meta.get("mission_id", self.mission["id"]):
                self.mission["meta"] = meta

    def _mission_end(self, status):
        with self.lock:
            if self.mission and not self.mission["ended"]:
                self.mission["ended"] = time.time()
                self.mission["status"] = status

    def mission_view(self, m=None) -> Optional[dict]:
        m = m or self.mission
        if not m:
            return None
        with self.lock:
            live = m is self.mission
            st = self.telemetry["nav_status"] if live and not m["ended"] else m["status"]
            return {"id": m["id"], "planner": m["planner"], "goal": m["goal"], "status": st,
                    "created": m["created"], "ended": m["ended"],
                    "duration_s": round((m["ended"] or time.time()) - m["created"], 2),
                    "title": m["meta"].get("title"), "path": self.telemetry["plan_path"] if live else None,
                    "dist_remaining": self.telemetry["nav_dist_rem"] if live else 0.0}

    def submit(self, x: float, y: float, yaw: float = 0.0, planner: Optional[str] = None) -> dict:
        if planner:
            if not self.set_planner_type(planner):
                raise ValueError(f"规划器不可用: {planner}")
        self._pending = (x, y, yaw)
        self.send_nav_goal(x, y, yaw)
        return self.mission_view()

    def feedback(self) -> dict:
        with self.lock:
            T = self.telemetry
            fb = {"online": True, "mission_id": self.current_mission_id, "status": T["nav_status"], "planner": self.active_planner,
                  "goal": T["target_goal"], "path": T["plan_path"], "path_labels": T.get("path_labels", []),
                  "curve": T.get("plan_curve") if T["plan_path"] else [], "protection": self.protection_view(),
                  "path_index": T.get("path_index", 0), "next_segment": self._next_segment_locked(), "dist_remaining": T["nav_dist_rem"],
                  "nav2_feedback": T.get("nav2_feedback"), "planners": self.PLANNERS,
                  "localization": dict(T.get("localization") or {}, pose={"x": round(T["x"], 4), "y": round(T["y"], 4), "yaw": round(T["yaw"], 5)})}
        fb["nav2"] = self.nav2.status()
        fb["mission"] = self.mission_view()
        fb["safety"] = self.safety_state()
        runner = getattr(self, "taskflow", None)
        if runner:
            fb["taskflow"] = runner.view()
        return fb

    def _align_steer(self, mission_id, mode: str, sign: float = 1.0, timeout: float = 2.5) -> bool:
        """先转后走: 起步/原地转向前用微小指令把舵轮转到位 (车体基本不动)，舵角稳定后再给速度，
        避免舵角过渡把车体带偏。mode = drive (直行舵角) | rotate (原地转向舵角)"""
        if self.active_chassis_type == "diff_drive" or not getattr(self, "_steer", None):
            return True
        cmd = (0.002 * sign, 0.0, 0.0) if mode == "drive" else (0.0, 0.0, 0.004 * sign)
        t0 = time.time()
        last, still, moved = None, 0, False
        while self.running and time.time() - t0 < timeout:
            if self.current_mission_id != mission_id:
                return False
            self.publish_cmd_vel(*cmd)
            cur = list(getattr(self, "_steer", []) or [])
            if last is not None and cur and len(cur) == len(last) and max(abs(a - b) for a, b in zip(cur, last)) < math.radians(0.15):
                still += 1
            else:
                if last is not None:
                    moved = True
                still = 0
            last = cur
            if (moved and still >= 5) or (not moved and time.time() - t0 > 0.7):
                break
            time.sleep(0.03)
        return True

    def _rotate_to(self, mission_id, target_yaw: float, rot_dir: float, max_w: float) -> str:
        """原地转向到 target_yaw (±0.3°)。按角减速度规划角速度 ω = √(2·α·|e|)，避免超调。返回 done/abort/blocked"""
        alpha = 0.6 * float(self.cfg.get("chassis", {}).get("max_ang_decel_radps2", 1.0) or 1.0)
        block_since = None
        settle = 0
        while self.running:
            with self.lock:
                if self.current_mission_id != mission_id:
                    return "abort"
                paused = self.is_paused or self.telemetry.get("io_states", {}).get("is_emergency_stop", False)
                cur_yaw, wnow = self.telemetry["yaw"], self.telemetry.get("wz", 0.0)
            if paused:
                self.publish_cmd_vel(0.0, 0.0, 0.0)
                time.sleep(0.04)
                continue
            e = math.atan2(math.sin(target_yaw - cur_yaw), math.cos(target_yaw - cur_yaw))
            if rot_dir and abs(e) > 0.5 and e * rot_dir < 0:
                e += rot_dir * 2 * math.pi
            e_pred = e - wnow * 0.12                        # 补偿指令/反馈延迟
            if abs(e) < 0.005 and abs(wnow) < 0.02:
                settle += 1
                if settle >= 3:
                    self.publish_cmd_vel(0.0, 0.0, 0.0)
                    return "done"
            else:
                settle = 0
            w = min(max_w, math.sqrt(2.0 * alpha * max(0.0, abs(e_pred))), 1.5 * abs(e_pred))
            w = math.copysign(max(0.01 if abs(e) >= 0.005 else 0.0, w), e_pred if abs(e_pred) > 1e-4 else e)
            self.publish_cmd_vel(0.0, 0.0, w)
            if self.obs.get("layer") == "rotate":
                block_since = block_since or time.time()
                with self.lock:
                    self.telemetry["nav_status"] = "OBSTACLE_WAIT"
                if time.time() - block_since > 8.0:
                    return "blocked"
            else:
                block_since = None
            time.sleep(0.03)
        return "abort"

    def _back_out(self, mission_id, x, y, yaw, yaw_to=None) -> bool:
        """离站倒车: 沿当前车身反方向找最近的「可原地转向」位置 (≤ 3 m，倒车路径车体净空满足)，倒车过去。
        返回 False 表示任务已被替换/取消"""
        segs = self._obstacle_segments_all()
        h, t, l, r = self.outline()
        hw = max(l, r)
        target = None
        for k in range(1, 31):
            d = 0.1 * k
            bx, by = x - d * math.cos(yaw), y - d * math.sin(yaw)
            if maneuver.clearance(segs, [(bx, by, yaw)], h, t, hw) < self.prot["body_margin"]:
                break                                     # 倒车路径被挡
            if (self._rotation_dir(bx, by, yaw, yaw_to) if yaw_to is not None else self._rotation_free(bx, by)):
                target = d
                break
        if target is None:
            self.event_hub.emit("navigation", "BACKOUT_FAIL", "warning", "离站倒车: 未找到可转向位置",
                                f"车尾方向 3 m 内没有原地转向空间 (扫掠半径 {self.sweep_radius():.2f} m)，尝试直接转向", {})
            return True
        self.event_hub.emit("navigation", "BACKOUT", "info", f"离站倒车 {target:.1f} m",
                            f"原地转向扫掠半径 {self.sweep_radius():.2f} m 内有设备/墙体，先倒车到可转向位置", {"dist": round(target, 2)})
        t_end = time.time() + 6.0 + target / 0.1
        while self.running and time.time() < t_end:
            with self.lock:
                if self.current_mission_id != mission_id:
                    return False
                cx, cy, cyaw = self.telemetry["x"], self.telemetry["y"], self.telemetry["yaw"]
                paused = self.is_paused
            if paused:
                self.publish_cmd_vel(0.0, 0.0, 0.0)
                time.sleep(0.04)
                continue
            done = -((cx - x) * math.cos(yaw) + (cy - y) * math.sin(yaw))
            if done >= target - 0.02:
                break
            v = -max(0.06, min(0.3, (target - done) * 1.2))
            wz = 2.0 * math.atan2(math.sin(yaw - cyaw), math.cos(yaw - cyaw))
            self.publish_cmd_vel(v, 0.0, wz)
            time.sleep(0.03)
        self._wait_until_stopped(mission_id)
        return self.current_mission_id == mission_id

    def _plan_route_corners(self, path):
        """下发时一次性确定每个拐点的过弯方式 (与执行一致)，并生成显示/考核用的参考曲线 (直线 + 过渡圆弧)"""
        n = len(path)
        corners = [None] * n
        curve = [{"x": round(path[0][0], 3), "y": round(path[0][1], 3)}] if n else []
        for i in range(1, n):
            a, b = path[i - 1], path[i]
            arc = None
            if i < n - 1:
                c = path[i + 1]
                h1, h2 = math.atan2(b[1] - a[1], b[0] - a[0]), math.atan2(c[1] - b[1], c[0] - b[0])
                turn = math.atan2(math.sin(h2 - h1), math.cos(h2 - h1))
                if abs(turn) > (0.35 if self.prot.get("corner_mode") == "arc" else 0.02) and self.active_chassis_type != "dual_steer":
                    corners[i] = self._plan_corner(b, h1, h2, math.hypot(b[0] - a[0], b[1] - a[1]), math.hypot(c[0] - b[0], c[1] - b[1]))
                    if "R" in corners[i][0]:
                        arc = corners[i][0]
            if arc:
                d, R = arc["d"], arc["R"]
                h1 = math.atan2(b[1] - a[1], b[0] - a[0])
                sgn = 1.0 if arc["turn"] > 0 else -1.0
                for k in range(0, 13):
                    th = h1 + arc["turn"] * k / 12.0
                    curve.append({"x": round(arc["cx"] + sgn * R * math.sin(th), 3), "y": round(arc["cy"] - sgn * R * math.cos(th), 3)})
            else:
                curve.append({"x": round(b[0], 3), "y": round(b[1], 3)})
        return corners, curve

    def _obstacle_segments_all(self):
        try:
            segs = list(self.dijkstra_planner._static_segments())
        except Exception:
            segs = list(self.walls)
        with self.lock:
            obs = list(self.dynamic_obstacles)
        return segs + maneuver.box_segments(obs)

    def _plan_corner(self, node, h1, h2, len_in, len_out):
        """拐点过弯方式 (含动态障碍物)，见 planning.maneuver.plan_corner"""
        return maneuver.plan_corner(self._obstacle_segments_all(), node, h1, h2, len_in, len_out,
                                    self.head_offset, self.tail_offset, self.half_width, self.corner_radius(),
                                    clear_min=self.prot["body_margin"], mode=self.prot.get("corner_mode", "auto"))

    def sweep_radius(self) -> float:
        return math.hypot(max(self.head_offset, self.tail_offset), self.half_width) + self.prot["rotate_margin"]

    def _rotation_free(self, x, y) -> bool:
        """原地转向扫掠圆 (半径 = 车体最远角点) 内无墙体/货架/障碍物"""
        r = self.sweep_radius()
        def seg_d(ax, ay, bx, by):
            dx, dy = bx - ax, by - ay
            L2 = dx * dx + dy * dy
            u = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / L2))
            return math.hypot(x - ax - u * dx, y - ay - u * dy)
        try:
            segs = self.dijkstra_planner._static_segments()
        except Exception:
            segs = list(self.walls)
        for w in segs:
            if seg_d(*w[:4]) < r:
                return False
        with self.lock:
            obs = list(self.dynamic_obstacles)
        for o in obs:
            if math.hypot(float(o.get("x", 0)) - x, float(o.get("y", 0)) - y) < r + 0.5 * math.hypot(float(o.get("w", .8)), float(o.get("h", .8))):
                return False
        return True

    def _rotation_dir(self, x, y, yaw_from, yaw_to) -> float:
        """原地从 yaw_from 转到 yaw_to 的可行方向: 先试最短方向，再试反方向 (绕开内侧设备)；
        车体矩形扫掠净空 ≥ rotate_margin + 1 cm 视为可行。返回 +1/-1，都不行返回 0"""
        turn = math.atan2(math.sin(yaw_to - yaw_from), math.cos(yaw_to - yaw_from))
        if abs(turn) < 1e-3:
            return 1.0
        sgn = 1.0 if turn > 0 else -1.0
        segs = self._obstacle_segments_all()
        h, t, l, r = self.outline()
        need = self.prot["rotate_margin"] + 0.01
        rmax = math.hypot(max(h, t), max(l, r))
        for tt in (turn, turn - sgn * 2 * math.pi):
            n = maneuver.sweep_samples(abs(tt) * rmax, 4)     # 车角每步 ≤ 1 cm (原来每 0.12 rad，车角每步约 17 cm)
            poses = [(x, y, yaw_from + tt * i / n) for i in range(n + 1)]
            if maneuver.clearance(segs, poses, h, t, max(l, r)) >= need:
                return 1.0 if tt > 0 else -1.0
        return 0.0

    @staticmethod
    def _snap_heading(h):
        c = round(h / (math.pi / 2.0)) * (math.pi / 2.0)
        return c if abs(math.atan2(math.sin(h - c), math.cos(h - c))) < 0.25 else h

    def corner_radius(self) -> float:
        """过弯圆弧半径: 车头越长半径越大 (减小车头外摆)，可由安全参数 corner_radius 覆盖"""
        p = getattr(self, "prot", None) or {}
        if p.get("corner_radius"):
            return float(p["corner_radius"])
        return max(0.6, min(1.4, 0.6 * self.head_offset + 0.3))

    def _corner_arc(self, mission_id, corner, max_w) -> bool:
        """以恒定线速度 + 角速度 v/R 走圆弧，直到航向对准下一路段。返回 False 表示任务已被替换/取消"""
        v, R, tgt = corner["v"], corner["R"], corner["heading"]
        sgn = 1.0 if corner["turn"] > 0 else -1.0
        t_end = time.time() + 4.0 * abs(corner["turn"]) * R / max(v, 0.05) + 2.0
        self.in_arc = True
        try:
            cxy = (corner["cx"], corner["cy"]) if "cx" in corner else None
            # 切点处已停稳: 先把舵轮转到圆弧曲率对应的舵角 (先转后走)，消除入弧滞后
            t0 = time.time()
            last, still = None, 0
            while self.running and time.time() - t0 < 2.5:
                if self.current_mission_id != mission_id:
                    return False
                self.publish_cmd_vel(0.002, 0.0, sgn * 0.002 / R)
                cur = list(getattr(self, "_steer", []) or [])
                if last is not None and cur and max(abs(a - b) for a, b in zip(cur, last)) < math.radians(0.15):
                    still += 1
                else:
                    still = 0
                last = cur
                if still >= 5 and time.time() - t0 > 0.3:
                    break
                time.sleep(0.03)
            return self._corner_arc_loop(mission_id, min(v, 0.2), R, tgt, sgn, t_end + 3.0, max_w, cxy, abs(corner["turn"]))
        finally:
            self.in_arc = False

    def _arc_blocked(self, R, sgn, remain) -> bool:
        """激光点是否落在剩余弧线的车体扫掠区内 (机体系；车体外扩 3 cm)"""
        sb = getattr(self, "_scan_body", None)
        if not sb or remain <= 0.02:
            return False
        import numpy as np
        rs, a0, inc, rmax = sb
        r = np.asarray(rs, float)
        ok = (r > 0.02) & (r < rmax - 1e-3) & (r < R + self.head_offset + 1.5)
        if not ok.any():
            return False
        a = a0 + inc * np.nonzero(ok)[0]
        px, py = r[ok] * np.cos(a), r[ok] * np.sin(a)
        m = 0.03
        # 车体沿圆弧行驶 ⇔ 点在车体系下绕瞬心 (0, sgn·R) 转 -sgn·remain：圆弧与矩形精确求交
        # (原来每 0.15 rad 采样一次，R = 0.9 m 时每步约 13 cm，比 3 cm 外扩大得多)
        from planning.sweep import arc_hits_box
        box = (-self.tail_offset - m, self.head_offset + m, -self.half_width - m, self.half_width + m)
        return bool(np.any(arc_hits_box(px, py, -sgn * remain, box, 0.0, sgn * R)))

    def _corner_arc_loop(self, mission_id, v, R, tgt, sgn, t_end, max_w, cxy=None, sweep=None) -> bool:
        stalled = None
        phi0 = None
        dec = self.max_decel * 0.55
        while self.running and time.time() < t_end:
            with self.lock:
                if self.current_mission_id != mission_id:
                    return False
                paused = self.is_paused or self.telemetry.get("io_states", {}).get("is_emergency_stop", False)
                cur_yaw = self.telemetry["yaw"]
                vnow = math.hypot(self.telemetry.get("vx", 0.0), self.telemetry.get("vy", 0.0))
            if paused:
                self.publish_cmd_vel(0.0, 0.0, 0.0)
                time.sleep(0.04)
                continue
            err = math.atan2(math.sin(tgt - cur_yaw), math.cos(tgt - cur_yaw))
            if cxy is not None and sweep:
                # 按位置在圆弧上的进度判断弧终点，并规划减速停在弧终点 (出弧后舵角回正再走直线)
                with self.lock:
                    px0, py0 = self.telemetry["x"], self.telemetry["y"]
                phi = math.atan2(py0 - cxy[1], px0 - cxy[0])
                if phi0 is None:
                    phi0 = phi
                prog = sgn * math.atan2(math.sin(phi - phi0), math.cos(phi - phi0))
                rem = R * (sweep - prog)
                if rem <= 0.003:
                    self.publish_cmd_vel(0.0, 0.0, 0.0)
                    self._wait_until_stopped(mission_id)
                    return True
                rem_eff = rem - vnow * 0.12
                vv = max(0.01, min(v, math.sqrt(2.0 * dec * max(0.0, rem_eff)), 2.0 * max(0.0, rem_eff)))
            else:
                if abs(err) < 0.06 or err * sgn < 0:
                    return True
                vv = max(0.08, min(v, vnow + 0.1))
            # 闭环跟踪圆弧: 前馈 v/R + 切线航向误差 + 径向偏差 (转向响应滞后时也保持在规划弧线上)
            vref = max(vnow, 0.03)
            wz = sgn * vref / R                                  # 曲率前馈
            if cxy is not None:
                with self.lock:
                    px, py = self.telemetry["x"], self.telemetry["y"]
                rx, ry = px - cxy[0], py - cxy[1]
                rho = math.hypot(rx, ry)
                tangent = math.atan2(ry, rx) + sgn * math.pi / 2
                e_y = sgn * (R - rho)                           # 左正 (与直线段同一约定)
                e_th = math.atan2(math.sin(cur_yaw - tangent), math.cos(cur_yaw - tangent))
                Lc = self.track_L
                wz -= vref * (1.8 / Lc * e_th + e_y / (Lc * Lc))
            wz = max(-max_w, min(max_w, wz))
            vx_s, _, _ = self.safety_filter(vv, 0.0, wz)
            if vx_s > 1e-3 and self._arc_blocked(R, sgn, min(abs(err), 1.2)):
                vx_s, vv = 0.0, 0.0
                with self.lock:
                    self.telemetry["nav_status"] = "OBSTACLE_WAIT"
            elif vx_s > 1e-3:
                with self.lock:
                    if self.telemetry["nav_status"] == "OBSTACLE_WAIT":
                        self.telemetry["nav_status"] = "NAVIGATING"
            if vx_s <= 1e-3:
                wz = 0.0                      # 前方有障碍停车等待: 不转向
                stalled = stalled or time.time()
                t_end += 0.03
                if time.time() - stalled > 20.0:
                    return True
            else:
                stalled = None
            self.publish_cmd_vel(vv, 0.0, wz)
            time.sleep(0.03)
        return True

    def _next_segment_locked(self):
        """当前路段 (车辆正驶向 path[path_index]) 与其后的下一路段，供界面显示"""
        T = self.telemetry
        path, labels, i = T.get("plan_path") or [], T.get("path_labels") or [], int(T.get("path_index") or 0)
        if len(path) < 2 or i <= 0 or i >= len(path):
            return None
        lab = lambda k: (labels[k] if k < len(labels) else None)
        tgt = path[i]
        seg = {"index": i, "to": tgt, "to_label": lab(i), "from_label": lab(i - 1),
               "dist": round(math.hypot(tgt["x"] - T["x"], tgt["y"] - T["y"]), 2),
               "remaining_waypoints": len(path) - i}
        if i + 1 < len(path):
            n = path[i + 1]
            seg["next"] = {"to": n, "to_label": lab(i + 1),
                           "dist": round(math.hypot(n["x"] - tgt["x"], n["y"] - tgt["y"]), 2)}
        return seg

    def _get_obstacle_segments(self):
        segments = []
        with self.lock:
            obstacles = list(self.dynamic_obstacles)
        for obs in obstacles:
            ox = float(obs.get("x", 0.0))
            oy = float(obs.get("y", 0.0))
            ow = float(obs.get("w", 0.8))
            oh = float(obs.get("h", 0.8))
            hw, hh = ow / 2.0, oh / 2.0
            segments.extend([
                (ox - hw, oy - hh, ox + hw, oy - hh),
                (ox + hw, oy - hh, ox + hw, oy + hh),
                (ox + hw, oy + hh, ox - hw, oy + hh),
                (ox - hw, oy + hh, ox - hw, oy - hh)
            ])
        return segments

    def send_nav_goal(self, x: float, y: float, yaw: float = 0.0, _replan_left: int = 2):
        with self.lock:
            self.current_mission_id += 1
            mission_id = self.current_mission_id
            cur_x, cur_y = self.telemetry["x"], self.telemetry["y"]
            planner = self.active_planner
            self.dynamic_obstacles = list(self.link.world.get("obstacles", self.dynamic_obstacles))
            active_obstacles = list(self.dynamic_obstacles)
            # Find station matching x, y if any to get dock_yaw
            stations = self.dijkstra_planner.get_stations()
            matched_station = next((st for st in stations if math.hypot(st["x"] - x, st["y"] - y) < 0.35), None)
            if matched_station and "dock_yaw" in matched_station:
                target_dock_yaw = matched_station["dock_yaw"]
            else:
                target_dock_yaw = yaw

            self.telemetry["target_goal"] = {"x": x, "y": y, "yaw": target_dock_yaw}
            self.telemetry["nav_status"] = "PLANNING"
        self._new_mission(mission_id, x, y, target_dock_yaw, planner)

        cpp = self._guide_cpp()
        if cpp is not None:
            cpp.send_guide_cancel()                     # 新任务: 停掉 C++ 里仍在执行的自研导引
        # Path Planning Selection
        if planner == "nav2":
            return self._send_nav2_goal(mission_id, cur_x, cur_y, x, y, target_dock_yaw, matched_station, stations)
        if planner in ("direct", "straight"):
            raw_path = [(cur_x, cur_y), (x, y)]
        elif planner == "astar":
            all_walls = self.walls + self._get_obstacle_segments()
            raw_path = self.astar_planner.plan((cur_x, cur_y), (x, y), all_walls)
            if len(raw_path) <= 2:
                raw_path = self.dijkstra_planner.plan((cur_x, cur_y), (x, y), active_obstacles)
        else:
            raw_path = self.dijkstra_planner.plan((cur_x, cur_y), (x, y), active_obstacles)

        if not raw_path or len(raw_path) < 2:
            with self.lock:
                self.telemetry["nav_status"] = "NO_PATH"
                self.telemetry["plan_path"] = []
            self._mission_end("NO_PATH")
            self.event_hub.emit("navigation", "PLAN_FAILED", "danger", f"任务 #{mission_id} 规划失败",
                                f"[{planner.upper()}] 从 ({cur_x:.2f},{cur_y:.2f}) 到 ({x:.2f},{y:.2f}) 无可行路径 (障碍物阻断全部路线)", {})
            return

        # 拐点可行性: 路线中仍含「车体无法转过去」的拐点 (所有候选路线都被挡) → 规划失败并说明原因，不下发
        if planner == "dijkstra" and len(raw_path) >= 3 and self.dijkstra_planner.footprint:
            bad = []
            for i in range(1, len(raw_path) - 1):
                a, b, c = raw_path[i - 1], raw_path[i], raw_path[i + 1]
                if self.dijkstra_planner._turn(a, b, c) > 0.02 and self.dijkstra_planner._corner_penalty(a, b, c) > 0:
                    bad.append(b)
            if bad:
                with self.lock:
                    self.telemetry["nav_status"] = "NO_PATH"
                    self.telemetry["plan_path"] = []
                self._mission_end("NO_PATH")
                h, t, l, r = self.outline()
                self.event_hub.emit("navigation", "PLAN_FAILED", "danger", f"任务 #{mission_id} 规划失败: 拐点转不过去",
                                    f"拐点 {', '.join(f'({p[0]:.1f},{p[1]:.1f})' for p in bad)} 处车体 (头 {h:.2f}/尾 {t:.2f}/半宽 {max(l, r):.2f} m) "
                                    f"原地转向净空不足 {self.prot['body_margin'] * 100:.0f} cm，且无其它可行路线。"
                                    f"可在保护空间中改用 auto/arc 过弯模式，或调整拓扑/工位", {"corners": bad})
                return
        labels = [None] * len(raw_path)
        lr = getattr(self.dijkstra_planner, "last_route", None)
        if lr and len(lr.get("points", [])) == len(raw_path) and all(
                math.hypot(a[0] - b[0], a[1] - b[1]) < 1e-6 for a, b in zip(lr["points"], raw_path)):
            labels = list(lr.get("labels") or labels)
        with self.lock:
            self.telemetry["plan_path"] = [{"x": round(pt[0], 3), "y": round(pt[1], 3)} for pt in raw_path]
            self.telemetry["path_labels"] = labels
            self.telemetry["path_index"] = 1
        corners, curve = self._plan_route_corners(raw_path)
        self._mission_corners = (mission_id, corners)
        with self.lock:
            self.telemetry["plan_curve"] = curve

        self.log(f"Mission #{mission_id} dispatched via [{planner.upper()}]: ({x}, {y}, yaw={target_dock_yaw:.2f}) with {len(raw_path)} waypoints")
        origin_st = next((st for st in stations if math.hypot(st["x"] - cur_x, st["y"] - cur_y) < 0.8), None)
        origin_name = origin_st["name"] if origin_st else f"起点 ({cur_x:.1f}, {cur_y:.1f})"
        dest_name = matched_station["name"] if matched_station else f"工位 ({x:.1f}, {y:.1f})"

        self.recorder.start_session(
            session_type="mission",
            metadata={
                "mission_id": mission_id,
                "title": f"{origin_name} ➔ {dest_name}",
                "scenario": self.active_scenario,
                "chassis": self.active_chassis_type,
                "planner": planner,
                "origin": {"x": round(cur_x, 2), "y": round(cur_y, 2), "name": origin_name},
                "destination": {"x": round(x, 2), "y": round(y, 2), "yaw": round(target_dock_yaw, 2), "name": dest_name},
                "planned_route": {
                    "planner": planner,
                    "target_goal": {"x": round(x, 2), "y": round(y, 2), "yaw": round(target_dock_yaw, 2)},
                    "waypoints": [[round(pt[0], 2), round(pt[1], 2)] for pt in raw_path]
                }
            }
        )

        self.event_hub.emit(
            "navigation", "MISSION_DISPATCH", "info",
            f"下发调度导航任务 #{mission_id}",
            f"目标工位: ({x:.2f}, {y:.2f}) | 目标航向: {round(math.degrees(target_dock_yaw), 1)}° | 规划算法: {planner.upper()} | 规划航点: {len(raw_path)} 个",
            {"mission_id": mission_id, "target_x": x, "target_y": y, "target_yaw": target_dock_yaw, "waypoints_count": len(raw_path)}
        )
        cpp = self._guide_cpp()
        if cpp is not None:
            with self.lock:
                self.telemetry["nav_status"] = "NAVIGATING"
            self._start_guide_cpp(cpp, mission_id, raw_path, target_dock_yaw, _replan_left, corners, labels)
            return
        threading.Thread(target=self._autonomous_guidance_loop, args=(mission_id, raw_path, target_dock_yaw, _replan_left), daemon=True).start()

    def _autonomous_guidance_loop(self, mission_id: int, waypoints: list, target_yaw: float, replan_left: int = 2):
        """
        High-precision Guidance Controller with Strict Corridor Tangent Locking
        and Cardinal Orthogonal Final Docking Alignment.
        """
        with self.lock:
            if self.current_mission_id != mission_id:
                return
            self.telemetry["nav_status"] = "NAVIGATING"
            chassis_type = self.active_chassis_type
            planner_kind = self.active_planner
        wait_since = None

        max_v = min(1.2, self.cfg.get('chassis', {}).get('max_speed_mps', 1.5))
        max_w = min(1.6, self.cfg.get('chassis', {}).get('max_ang_speed_radps', 2.0))

        # seg_rest[i] = 从航点 i 到终点的路程
        seg_rest = [0.0] * len(waypoints)
        for i in range(len(waypoints) - 2, -1, -1):
            a, b = waypoints[i], waypoints[i + 1]
            seg_rest[i] = seg_rest[i + 1] + math.hypot(b[0] - a[0], b[1] - a[1])
        # stop_rest[i] = 从航点 i 到下一个需要停车的航点 (拐点/终点) 的路程；共线中间点不停车
        stop_rest = [0.0] * len(waypoints)
        for i in range(len(waypoints) - 2, 0, -1):
            a, b, c = waypoints[i - 1], waypoints[i], waypoints[i + 1]
            h1, h2 = math.atan2(b[1] - a[1], b[0] - a[0]), math.atan2(c[1] - b[1], c[0] - b[0])
            straight = abs(math.atan2(math.sin(h2 - h1), math.cos(h2 - h1))) <= 0.02
            stop_rest[i] = (math.hypot(c[0] - b[0], c[1] - b[1]) + stop_rest[i + 1]) if straight else 0.0
        next_rot_dir = 0.0
        try:
            for wp_idx in range(1, len(waypoints)):
                rot_dir, next_rot_dir = next_rot_dir, 0.0
                prev_wp = waypoints[wp_idx - 1]
                target_wp = waypoints[wp_idx]
                with self.lock:
                    seg_labels = None
                    if self.current_mission_id == mission_id:
                        self.telemetry["path_index"] = wp_idx
                        labs = self.telemetry.get("path_labels") or []
                        if wp_idx < len(labs) and (labs[wp_idx] or labs[wp_idx - 1]):
                            seg_labels = (labs[wp_idx - 1], labs[wp_idx])
                if seg_labels:
                    fr = seg_labels[0] or f"({prev_wp[0]:.1f},{prev_wp[1]:.1f})"
                    to = seg_labels[1] or f"({target_wp[0]:.1f},{target_wp[1]:.1f})"
                    self.event_hub.emit("navigation", "SEGMENT", "info", f"进入路段 {wp_idx}/{len(waypoints) - 1}",
                                        f"{fr} → {to}  {math.hypot(target_wp[0] - prev_wp[0], target_wp[1] - prev_wp[1]):.2f} m",
                                        {"mission_id": mission_id, "index": wp_idx, "from": seg_labels[0], "to": seg_labels[1]})
                target_x, target_y = target_wp[0], target_wp[1]
                is_final_wp = (wp_idx == len(waypoints) - 1)

                # Desired segment heading
                seg_dx = target_x - prev_wp[0]
                seg_dy = target_y - prev_wp[1]
                seg_dist = math.hypot(seg_dx, seg_dy)
                if seg_dist > 0.02:
                    # 车头与线路方向一致: 航向严格取路段方向 (不再就近取整到 0/90/180°，否则非正交路段会产生横向漂移)
                    seg_heading = math.atan2(seg_dy, seg_dx)
                else:
                    seg_heading = target_yaw

                # Desired segment heading error relative to current vehicle yaw
                with self.lock:
                    cur_yaw = self.telemetry["yaw"]
                    cur_x = self.telemetry["x"]
                    cur_y = self.telemetry["y"]

                init_heading_err = math.atan2(math.sin(seg_heading - cur_yaw), math.cos(seg_heading - cur_yaw))

                # 倒车: 目标路段与车头方向近乎相反且原地转向会扫到周边 (窄巷道/设备旁工位) → 不转向，直接倒车沿路段行驶
                dist_to_target = math.hypot(target_x - cur_x, target_y - cur_y)
                need_rot = chassis_type != "dual_steer" and abs(init_heading_err) > 0.02 and dist_to_target > 0.1
                rd = self._rotation_dir(cur_x, cur_y, cur_yaw, seg_heading) if need_rot else 1.0
                reverse = chassis_type != "dual_steer" and abs(init_heading_err) > 2.4 and dist_to_target > 0.3 and rd == 0.0
                if reverse:
                    self.event_hub.emit("navigation", "REVERSE", "info", f"路段 {wp_idx} 倒车行驶",
                                        f"原地转向扫掠半径 {self.sweep_radius():.2f} m 内有障碍，改为倒车 {dist_to_target:.1f} m", {"index": wp_idx})
                # PHASE 1: 原地转向对准路段方向 (精度 0.3°)，随后以车头朝向路段方向行驶
                if not reverse and abs(init_heading_err) > 0.02 and dist_to_target > 0.1:
                    # 先停稳再原地转向 (按 cmodel 减速度自然制动，等待实测速度归零)
                    self._wait_until_stopped(mission_id)
                    # 原地转不开 (工位紧贴设备/墙体，车头朝里停靠): 先沿车身方向倒车到能转向的位置 (离站倒车)
                    if chassis_type != "dual_steer" and rd == 0.0:
                        if not self._back_out(mission_id, cur_x, cur_y, cur_yaw, seg_heading):
                            return
                        with self.lock:
                            bx, by, byaw = self.telemetry["x"], self.telemetry["y"], self.telemetry["yaw"]
                        rd = self._rotation_dir(bx, by, byaw, seg_heading)
                    if rd:
                        rot_dir = rd
                    if not self._align_steer(mission_id, "rotate", 1.0 if (rot_dir or init_heading_err) > 0 else -1.0):
                        return
                    res = self._rotate_to(mission_id, seg_heading, rot_dir, max_w)
                    if res == "abort":
                        return
                    if res == "blocked":
                        self.event_hub.emit("navigation", "ROTATE_BLOCKED", "danger", f"任务 #{mission_id} 原地转向受阻",
                                            "转向防护区内持续有障碍 8 s，任务终止；请清除障碍或调整工位/保护空间后重新下发", {})
                        with self.lock:
                            self.telemetry["nav_status"] = "FAILED"
                        self.recorder.end_session("FAILED")
                        return
                    with self.lock:
                        if self.telemetry["nav_status"] == "OBSTACLE_WAIT":
                            self.telemetry["nav_status"] = "NAVIGATING"
                # 起步前舵角回正 (先转后走)
                if chassis_type != "dual_steer" and not self._align_steer(mission_id, "drive", -1.0 if reverse else 1.0):
                    return

                # 拐角圆弧过弯 (非全向底盘): 在拐点前 d_c 处切入半径 R 的圆弧，车头不再在拐点处原地大角度扫掠
                # (长车头车型原地转向扫掠半径 = hypot(车头, 半宽)，在货架/设备岛旁的拓扑拐点易碰撞)
                corner = None
                corner_rot = False
                if not reverse and not is_final_wp and chassis_type != "dual_steer" and seg_dist > 0.05:
                    nxt = waypoints[wp_idx + 1]
                    n_len = math.hypot(nxt[0] - target_x, nxt[1] - target_y)
                    if n_len > 0.1:
                        next_heading = math.atan2(nxt[1] - target_y, nxt[0] - target_x)
                        turn = math.atan2(math.sin(next_heading - seg_heading), math.cos(next_heading - seg_heading))
                        if abs(turn) > (0.35 if self.prot.get("corner_mode") == "arc" else 0.02):
                            mc = getattr(self, "_mission_corners", (None, None))
                            pre = mc[1][wp_idx] if mc[0] == mission_id and mc[1] and wp_idx < len(mc[1]) else None
                            corner, clr = pre if pre else self._plan_corner((target_x, target_y), seg_heading, next_heading, seg_dist, n_len)
                            corner = dict(corner)
                            if "rotate" in corner:
                                corner_rot, next_rot_dir, corner = True, corner["rotate"], None
                            else:
                                corner["v"] = min(corner["v"], max_v)
                            if clr < 0.05:
                                self.event_hub.emit("navigation", "CORNER_TIGHT", "warning", f"拐点 {wp_idx} 净空不足",
                                                    f"{'圆弧 R=%.2f m' % corner['R'] if corner else '拐点原地转向'} 车体最小净空 {clr * 100:.0f} cm "
                                                    f"(车头 {self.head_offset:.2f} m / 半宽 {self.half_width:.2f} m)", {"index": wp_idx, "clearance": round(clr, 3)})

                # 无法走圆弧的急转 (下一路段过短): 走到拐点再原地转向，避免提前转向偏离拓扑
                sharp_next = corner_rot
                if not is_final_wp and not corner and not corner_rot:
                    nxt = waypoints[wp_idx + 1]
                    if math.hypot(nxt[0] - target_x, nxt[1] - target_y) > 0.1:
                        nh = math.atan2(nxt[1] - target_y, nxt[0] - target_x)
                        sharp_next = abs(math.atan2(math.sin(nh - seg_heading), math.cos(nh - seg_heading))) > 0.6

                # PHASE 2: Straight Corridor Tracking with Continuous Smooth Stanley Line Following
                while self.running:
                    with self.lock:
                        if self.current_mission_id != mission_id:
                            return
                        if self.is_paused or self.telemetry.get("io_states", {}).get("is_emergency_stop", False):
                            self.publish_cmd_vel(0.0, 0.0, 0.0)
                            time.sleep(0.04)
                            continue
                        cur_x = self.telemetry["x"]
                        cur_y = self.telemetry["y"]
                        cur_yaw = self.telemetry["yaw"]

                    dx = target_x - cur_x
                    dy = target_y - cur_y
                    dist = math.hypot(dx, dy)

                    along = (cur_x - target_x) * math.cos(seg_heading) + (cur_y - target_y) * math.sin(seg_heading)
                    lateral = -(cur_x - target_x) * math.sin(seg_heading) + (cur_y - target_y) * math.cos(seg_heading)
                    stop_here = is_final_wp or corner_rot
                    if corner:                                   # 圆弧过弯: 到达切点 (距拐点 d) 切入圆弧
                        if along >= -corner["d"] - 0.003 and abs(lateral) < 0.35:
                            break
                    elif stop_here:
                        # 精确停到拐点/终点: 沿路段方向到达 (剩余 ≤ 3 mm) 即停，再原地转向
                        if along > -0.003 and abs(lateral) < 0.35:
                            break
                    elif along >= 0.0 and abs(lateral) < 0.6:  # 共线的中间拓扑点: 不停车直接切换
                        break

                    # 横向偏差 (相对路段直线，左正)
                    cross_track = -(cur_x - prev_wp[0]) * math.sin(seg_heading) + (cur_y - prev_wp[1]) * math.cos(seg_heading)

                    # 速度: 按到下一个停车点 (拐点/终点) 的剩余路程规划减速 v = √(2·a·s)，末段按比例爬行，保证停准
                    dec = self.max_decel * 0.55                 # 留余量: 仿真/实车的速度环有一阶滞后
                    with self.lock:
                        v_meas = abs(self.telemetry.get("vx", 0.0))
                    rem_stop = max(0.0, -along - (corner["d"] if corner else 0.0)) + (0.0 if corner else stop_rest[wp_idx])
                    rem_eff = rem_stop - v_meas * 0.12             # 补偿指令→执行→反馈的延迟
                    # 前方停车点 (拐点/切点/终点) 之外的障碍不影响本段: 防护区缩短到剩余行程 (紧贴墙体/设备的拐点、工位仍可停到位)
                    self.approach_left = None if reverse else rem_stop
                    v_stop = min(math.sqrt(2.0 * dec * max(0.0, rem_eff - 0.002)), 2.0 * max(0.0, rem_eff))
                    vx_nom = min(max_v, max(0.01 if rem_stop > 0.002 else 0.0, v_stop))

                    # 防护区: 允许速度 = 停车距离放得下的最高一档 (末段进站只看剩余行程)；最低档也放不下 → 停车等待
                    sgn = -1 if reverse else 1
                    allowed, d_hit, need = self._allowed_speed(sgn, self.approach_left if not reverse else None)
                    if not self.prot.get("enabled", True):
                        allowed = 99.0
                    if self._photo_sides()["rear" if reverse else "front"]:
                        allowed, d_hit = 0.0, 0.0     # 光电触发 (激光近场盲区) → 视同障碍停车等待 (距停车点 < 10 cm 时不再响应)

                    if allowed <= 1e-6:
                        # 【已运动到障碍物边缘】：平稳停车等待；持续阻塞 → 带障碍物重新规划绕行
                        self.publish_cmd_vel(0.0, 0.0, 0.0)
                        if wait_since is None:
                            wait_since = time.time()
                        elif is_final_wp and -along < self.prot["docking"]["arrive_tolerance"] and time.time() - wait_since > 2.0:
                            # 工位前方紧贴设备/墙体: 距目标小于到位容差时视为到位，避免反复重规划
                            self.event_hub.emit("navigation", "DOCK_LIMITED", "warning", "工位前方受限，提前停靠",
                                                f"距目标 {max(0.0, -along) * 100:.0f} cm 处前方障碍 {d_hit} m，按到位处理", {})
                            break
                        elif time.time() - wait_since > 5.0 and planner_kind not in ("direct", "straight") and replan_left > 0:
                            self.event_hub.emit("navigation", "REPLAN", "warning", f"任务 #{mission_id} 阻塞超时，重新规划",
                                                "前方障碍物持续 5 s 未解除，结合动态障碍物重新计算绕行路线", {})
                            threading.Thread(target=self.send_nav_goal, args=(waypoints[-1][0], waypoints[-1][1], target_yaw),
                                             kwargs={"_replan_left": replan_left - 1}, daemon=True).start()
                            return
                        with self.lock:
                            self.telemetry["nav_status"] = "OBSTACLE_WAIT"
                        time.sleep(0.04)
                        continue
                    else:
                        wait_since = None
                    if allowed < vx_nom:
                        # 【防护区降档】：不必停车，按放得下的档位限速跟进
                        vx_nom = max(0.08, allowed)
                    with self.lock:
                        if self.telemetry["nav_status"] == "OBSTACLE_WAIT":
                            self.telemetry["nav_status"] = "NAVIGATING"
                    # 横向跟踪 (二阶临界阻尼，空间域): ω = -v·(2ζ/L·e_θ + e_y/L²)
                    #   e_y 横向偏差，e_θ 行驶方向与路段方向的夹角；L 为收敛距离 (与速度无关)，航向始终贴近路段方向
                    move_heading = cur_yaw + (math.pi if reverse else 0.0)
                    e_th = math.atan2(math.sin(move_heading - seg_heading), math.cos(move_heading - seg_heading))
                    L_conv, zeta = self.track_L, 0.9
                    heading_err = -e_th
                    vx = vx_nom
                    if abs(e_th) > 0.35:                       # 偏航过大 (受扰): 降速纠正
                        vx = min(vx, 0.1)
                    wz = -max(vx, 0.02) * (2.0 * zeta / L_conv * e_th + cross_track / (L_conv * L_conv))
                    if reverse:
                        wz = wz                                # 倒车时车体与行驶方向同步转动，公式不变
                    wz = max(-max_w, min(max_w, wz))

                    if chassis_type == "dual_steer":
                        # 全向底盘: 航向锁定在通道方向，横向偏差用 vy 直接修正 (避免舵轮大角度反复换向)
                        h_err = math.atan2(math.sin(seg_heading - cur_yaw), math.cos(seg_heading - cur_yaw))
                        wz = max(-min(max_w, 0.6), min(min(max_w, 0.6), h_err * 1.5))
                        vy = max(-0.3, min(0.3, -cross_track * 1.2))
                        vx = max(min(0.15, vx_nom), vx_nom * max(0.3, math.cos(min(math.pi / 2, abs(h_err)))))
                        self.publish_cmd_vel(vx, vy, wz)
                    elif reverse:
                        self.publish_cmd_vel(-min(vx, 0.5), 0.0, wz)
                    else:
                        self.publish_cmd_vel(vx, 0.0, wz)

                    time.sleep(0.03)

                if corner and not self._corner_arc(mission_id, corner, max_w):
                    return

            self.approach_left = None
            # PHASE 3: Precision Final Orientation Alignment (Parallel or Orthogonal Docking)
            cardinal_target = round(target_yaw / (math.pi / 2.0)) * (math.pi / 2.0)
            target_dock_yaw = cardinal_target

            self._wait_until_stopped(mission_id)

            self.event_hub.emit(
                "navigation", "NAV_DOCKING", "info",
                "工位末端调姿对齐",
                f"进入工位末端高精度调姿阶段，目标 Cardinal 航向: {round(math.degrees(target_dock_yaw), 1)}°",
                {"target_dock_yaw": round(target_dock_yaw, 3)}
            )

            align_start = time.time()
            with self.lock:
                fx, fy, fyaw = self.telemetry["x"], self.telemetry["y"], self.telemetry["yaw"]
            if (chassis_type != "dual_steer" and abs(math.atan2(math.sin(target_dock_yaw - fyaw), math.cos(target_dock_yaw - fyaw))) > 0.3
                    and not self._rotation_dir(fx, fy, fyaw, target_dock_yaw)):
                self.event_hub.emit("navigation", "DOCK_SKIP_ROTATE", "warning", "工位调姿跳过",
                                    f"原地转向扫掠半径 {self.sweep_radius():.2f} m 内有障碍，保持当前航向 {math.degrees(fyaw):.0f}°", {})
                align_start = -1e9
            if align_start > 0:
                self._align_steer(mission_id, "rotate", 1.0)
                if self._rotate_to(mission_id, target_dock_yaw, 0.0, min(1.0, max_w)) == "abort":
                    return

            # Mission Complete
            self.publish_cmd_vel(0.0, 0.0, 0.0)
            with self.lock:
                if self.current_mission_id == mission_id:
                    self.telemetry["nav_status"] = "ARRIVED"
                    self.telemetry["plan_path"] = []
                    self.telemetry["nav_dist_rem"] = 0.0
                    final_x = self.telemetry["x"]
                    final_y = self.telemetry["y"]
                    final_yaw = self.telemetry["yaw"]
            self.recorder.end_session("ARRIVED")
            yaw_dev_deg = abs(round(math.degrees(math.atan2(math.sin(target_dock_yaw - final_yaw), math.cos(target_dock_yaw - final_yaw))), 2))
            self.log(f"Mission #{mission_id} ARRIVED & DOCKED (final yaw={final_yaw:.2f} rad, target={target_dock_yaw:.2f}).")
            self.event_hub.emit(
                "navigation", "MISSION_ARRIVED", "success",
                f"任务 #{mission_id} 停靠到位完成",
                f"已就位停靠，位置: ({final_x:.2f}, {final_y:.2f}) | 航向对齐偏差: {yaw_dev_deg}° (严格满足正交平行)",
                {"mission_id": mission_id, "x": round(final_x, 3), "y": round(final_y, 3), "yaw": round(final_yaw, 3), "dev_deg": yaw_dev_deg}
            )

        finally:
            if self.current_mission_id == mission_id:
                self.approach_left = None
            for _ in range(3):
                self.publish_cmd_vel(0.0, 0.0, 0.0)
                time.sleep(0.02)

    def _send_nav2_goal(self, mission_id, cur_x, cur_y, x, y, yaw, matched_station, stations, _waited=False):
        origin_st = next((st for st in stations if math.hypot(st["x"] - cur_x, st["y"] - cur_y) < 0.8), None)
        self.recorder.start_session(session_type="mission", metadata={
            "mission_id": mission_id, "title": f"[Nav2] {(origin_st or {}).get('name', f'({cur_x:.1f},{cur_y:.1f})')} ➔ {(matched_station or {}).get('name', f'({x:.1f},{y:.1f})')}",
            "scenario": self.active_scenario, "chassis": self.active_chassis_type, "planner": "nav2",
            "origin": {"x": round(cur_x, 2), "y": round(cur_y, 2)}, "destination": {"x": round(x, 2), "y": round(y, 2), "yaw": round(yaw, 2)},
            "planned_route": {"planner": "nav2", "target_goal": {"x": round(x, 2), "y": round(y, 2), "yaw": round(yaw, 2)}, "waypoints": []}})

        # 拓扑路线: Nav2 沿拓扑边逐点通过 (NavigateThroughPoses)，不可用时退回 NavigateToPose
        with self.lock:
            obstacles = list(self.dynamic_obstacles)
        route = self.dijkstra_planner.plan((cur_x, cur_y), (x, y), obstacles) or []
        labels = list((getattr(self.dijkstra_planner, "last_route", None) or {}).get("labels") or [None] * len(route))
        n_pts = len(route)
        with self.lock:
            self.telemetry["plan_path"] = [{"x": round(p[0], 3), "y": round(p[1], 3)} for p in route]
            self.telemetry["path_labels"] = labels[:n_pts]
            self.telemetry["path_index"] = 1 if n_pts >= 2 else 0

        def on_fb(mid, dist, t_sec, recoveries, poses_remaining=None):
            with self.lock:
                if self.current_mission_id == mid:
                    if poses_remaining is not None and n_pts >= 2:
                        self.telemetry["path_index"] = max(1, min(n_pts - 1, n_pts - int(poses_remaining)))
                    self.telemetry["nav_dist_rem"] = round(dist, 2)
                    self.telemetry["nav2_feedback"] = {"distance_remaining": round(dist, 2), "navigation_time_s": t_sec, "recoveries": recoveries}
                    if self.telemetry["nav_status"] in ("PLANNING", "OBSTACLE_WAIT"):
                        self.telemetry["nav_status"] = "NAVIGATING"

        def on_result(mid, result):
            status = {"SUCCEEDED": "ARRIVED", "CANCELED": "CANCELED"}.get(result, "FAILED")
            with self.lock:
                if self.current_mission_id != mid:
                    return
                self.telemetry["nav_status"] = status
                if status == "ARRIVED":
                    self.telemetry["nav_dist_rem"] = 0.0
                fx, fy, fyaw = self.telemetry["x"], self.telemetry["y"], self.telemetry["yaw"]
            self.recorder.end_session(status)
            if getattr(self, "_agv_mission", None) == mid:
                self._agv_mission = None
                self.approach_left = None
            lvl = "success" if status == "ARRIVED" else ("warning" if status == "CANCELED" else "danger")
            dev = abs(math.degrees(math.atan2(math.sin(yaw - fyaw), math.cos(yaw - fyaw))))
            self.event_hub.emit("navigation", f"NAV2_{result}", lvl, f"Nav2 任务 #{mid}: {result}",
                                f"终点 ({fx:.2f}, {fy:.2f}) 目标误差 {math.hypot(fx - x, fy - y) * 100:.1f} cm / {dev:.1f}°",
                                {"mission_id": mid, "result": result})

        # 默认: 拓扑路线交给 Nav2 插件 (agv_nav2_plugins，C++) —— 规划 (圆弧过弯/拐点转向)、跟随 (停车精度)、
        # 受阻恢复 (摆头 + 后退 + 转向) 都在 Nav2 行为树里完成；NAV2_ROUTE_MODE=follow_path 或插件不可用时走下面的 Python 分段跟线
        route_mode = os.environ.get("NAV2_ROUTE_MODE", "agv")
        agv_ok = route_mode == "agv" and hasattr(self.nav2, "agv_ready") and self.nav2.agv_ready()
        if route_mode == "agv" and hasattr(self.nav2, "agv_ready") and not agv_ok and not _waited and \
                getattr(self.nav2, "available", False) and not self.nav2.ready():
            def later_agv():
                t_wait = time.time() + float(os.environ.get("NAV2_FOLLOW_WAIT", "20"))
                while time.time() < t_wait and not self.nav2.agv_ready() and self.current_mission_id == mission_id:
                    time.sleep(0.5)
                if self.current_mission_id == mission_id:
                    self._send_nav2_goal(mission_id, cur_x, cur_y, x, y, yaw, matched_station, stations, _waited=True)
            with self.lock:
                self.telemetry["nav_status"] = "PLANNING"
            self.event_hub.emit("navigation", "NAV2_WAIT", "info", f"Nav2 任务 #{mission_id}: 等待 Nav2 激活", "", {})
            threading.Thread(target=later_agv, daemon=True, name=f"nav2-wait-{mission_id}").start()
            return
        if agv_ok:
            pts = [(float(p[0]), float(p[1])) for p in route] if n_pts >= 2 else [(cur_x, cur_y), (x, y)]
            if math.hypot(pts[0][0] - cur_x, pts[0][1] - cur_y) > 0.03:
                pts.insert(0, (cur_x, cur_y))
            if math.hypot(pts[-1][0] - x, pts[-1][1] - y) > 0.01:
                pts.append((x, y))
            self._agv_mission = mission_id
            self.approach_left = None
            self.nav2.on_stop_distance = lambda d, mid=mission_id: (
                setattr(self, "approach_left", d) if getattr(self, "_agv_mission", None) == mid else None)
            self.nav2.on_plugin_event = lambda e, mid=mission_id: self.event_hub.emit(
                "navigation", e.get("type", "NAV2_EVENT"), e.get("level", "info"), e.get("title", ""), e.get("message", ""),
                {"mission_id": mid, "source": "agv_nav2_plugins"})
            try:   # 场景静态几何 (墙/货架线段) → RouteController 末段精定位
                segs = list(self.dijkstra_planner._static_segments())
            except Exception as e:  # noqa
                segs = []
                self.log(f"场景几何不可用 (末段精定位退回 slam 定位): {e}")
            def on_plan(curve, mid=mission_id):   # 界面参考曲线 = Nav2 AgvRoute 实际规划的路径 (/plan)
                with self.lock:
                    if getattr(self, "_agv_mission", None) == mid:
                        self.telemetry["plan_curve"] = curve
            self.nav2.on_plan = on_plan
            err = self.nav2.send_route(pts, x, y, yaw, mission_id, on_result, on_fb, segs=segs)
            if err:
                self._agv_mission = None
                with self.lock:
                    self.telemetry["nav_status"] = "FAILED"
                self.recorder.end_session("FAILED")
                self.event_hub.emit("navigation", "NAV2_UNAVAILABLE", "danger", "Nav2 目标下发失败", err, {})
                return
            with self.lock:
                self.telemetry["nav_status"] = "NAVIGATING"
            self.push_core(force_mode=True)
            self.event_hub.emit("navigation", "MISSION_DISPATCH", "info", f"Nav2 导航任务 #{mission_id}",
                                f"拓扑路线 {len(pts) - 1} 个路段 → ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°)，Nav2 插件: "
                                "AgvRoute 规划 (圆弧过弯/拐点转向) + RouteFollow 跟随 (停车精度) + adjust_pose 恢复", {"route_points": len(pts)})
            return

        # 线路跟随 (Python 分段) —— 拓扑路线按拐点切成直线段 (可原地转向的拐点停车转向，转不开的拐点用过渡圆弧)，
        # 逐段 FollowPath 交给 Nav2 controller_server (RotationShim + Regulated Pure Pursuit，±20 mm 到位)
        follow_mode = os.environ.get("NAV2_ROUTE_MODE", "follow_path") == "follow_path" and hasattr(self.nav2, "follow_ready")
        if follow_mode and n_pts >= 2 and not _waited and not self.nav2.follow_ready():
            # 线路跟随是默认方式：FollowPath 暂时不可用 (Nav2 重启/激活中) 时后台等一会儿再下发，不要直接退到 BT 导航
            def later():
                t_wait = time.time() + float(os.environ.get("NAV2_FOLLOW_WAIT", "20"))
                while time.time() < t_wait and not self.nav2.follow_ready() and self.current_mission_id == mission_id:
                    time.sleep(0.5)
                if self.current_mission_id == mission_id:
                    self._send_nav2_goal(mission_id, cur_x, cur_y, x, y, yaw, matched_station, stations, _waited=True)
            with self.lock:
                self.telemetry["nav_status"] = "PLANNING"
            self.event_hub.emit("navigation", "NAV2_WAIT", "info", f"Nav2 任务 #{mission_id}: 等待 FollowPath 就绪", "", {})
            threading.Thread(target=later, daemon=True, name=f"nav2-wait-{mission_id}").start()
            return
        if follow_mode and n_pts >= 2 and self.nav2.follow_ready():
            corners, curve = self._plan_route_corners(route)
            segs = self._nav2_segments(route, corners, yaw)
            with self.lock:
                self.telemetry["plan_curve"] = curve
                self.telemetry["nav_status"] = "NAVIGATING"
            self.event_hub.emit("navigation", "MISSION_DISPATCH", "info", f"Nav2 导航任务 #{mission_id}",
                                f"FollowPath 线路跟随 → ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°)，{len(segs)} 段 (拐点停车转向)，"
                                "控制器 RotationShim + Regulated Pure Pursuit", {"segments": len(segs)})
            threading.Thread(target=self._nav2_follow_loop, args=(mission_id, segs, on_result), daemon=True,
                             name=f"nav2-follow-{mission_id}").start()
            return

        err, via = "", "NavigateToPose"
        if n_pts >= 3 and hasattr(self.nav2, "send_through_poses"):
            poses = []
            for i in range(1, n_pts):
                if i == n_pts - 1:
                    poses.append((x, y, yaw))
                else:
                    a, b = route[i - 1], route[i]
                    poses.append((b[0], b[1], math.atan2(b[1] - a[1], b[0] - a[0])))
            err = self.nav2.send_through_poses(poses, mission_id, on_result, on_fb)
            via = "NavigateThroughPoses"
            if err:
                self.log(f"NavigateThroughPoses 不可用 ({err})，退回 NavigateToPose")
                err, via = "", "NavigateToPose"
        if via == "NavigateToPose":
            err = self.nav2.send_goal(x, y, yaw, mission_id, on_result, on_fb)
        if err:
            with self.lock:
                self.telemetry["nav_status"] = "FAILED"
            self.recorder.end_session("FAILED")
            self.event_hub.emit("navigation", "NAV2_UNAVAILABLE", "danger", "Nav2 目标下发失败", err, {})
            return
        with self.lock:
            self.telemetry["nav_status"] = "NAVIGATING"
        self.event_hub.emit("navigation", "MISSION_DISPATCH", "info", f"Nav2 导航任务 #{mission_id}",
                            f"{via} → ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°)" + (f"，沿拓扑 {n_pts - 1} 个路段" if via != "NavigateToPose" else "") + "，局部控制/恢复行为由 Nav2 执行", {})

    @staticmethod
    def _nav2_segments(path, corners, final_yaw, step=0.05):
        """拓扑路线 → Nav2 线路跟随段 [(poses[(x,y,yaw)...], 结束节点序号)]。
        原地转向的拐点处分段 (段终点朝向 = 下一段方向，控制器到位后原地转过去)；圆弧拐点并入同一段。"""
        def line(a, b, h):
            L = math.hypot(b[0] - a[0], b[1] - a[1])
            n = max(1, int(math.ceil(L / step)))
            return [(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n, h) for k in range(1, n + 1)]
        n = len(path)
        if n < 2:
            return []
        h0 = math.atan2(path[1][1] - path[0][1], path[1][0] - path[0][0])
        cur, segs, start = [(path[0][0], path[0][1], h0)], [], path[0]
        for i in range(1, n):
            b = path[i]
            h_in = math.atan2(b[1] - path[i - 1][1], b[0] - path[i - 1][0])
            c = corners[i] if i < len(corners) else None
            arc = c[0] if (c and isinstance(c[0], dict) and "R" in c[0]) else None
            if arc and i < n - 1:
                d, R, turn = arc["d"], arc["R"], arc["turn"]
                tin = (b[0] - d * math.cos(h_in), b[1] - d * math.sin(h_in))
                cur += line(start, tin, h_in)
                sgn = 1.0 if turn > 0 else -1.0
                N = max(6, int(abs(turn) * R / step))
                for k in range(1, N + 1):
                    th = h_in + turn * k / N
                    cur.append((arc["cx"] + sgn * R * math.sin(th), arc["cy"] - sgn * R * math.cos(th), th))
                start = cur[-1][:2]
                continue
            cur += line(start, b, h_in)
            start = b
            if i < n - 1:
                h_out = math.atan2(path[i + 1][1] - b[1], path[i + 1][0] - b[0])
                if abs(math.atan2(math.sin(h_out - h_in), math.cos(h_out - h_in))) > 0.02:
                    # 段终点保持到达方向 (不在段末原地转)；转向交给下一段开头的 RotationShim ——
                    # 转向空间不足时车停在下一段起点，可由 _nav2_follow_loop 后退一点再转 (Nav2 BackUp)
                    segs.append((cur, i))
                    cur = [(b[0], b[1], h_out)]
        cur[-1] = (cur[-1][0], cur[-1][1], final_yaw)
        segs.append((cur, n - 1))
        return [sg for sg in segs if len(sg[0]) >= 2]

    def _nav2_follow_loop(self, mission_id, segs, on_result):
        """逐段 FollowPath。中断后的恢复 (逐级)：
          · 段起点原地转向受阻 → 执行进程精确原地转向；终点只差停靠朝向 → 执行进程对位转向
          · 其余中断看本次有无进展: 有进展 (动态障碍等) 保持位姿等 2 s；无进展第 1 次 Nav2 BackUp 后退 0.2 m，
            第 2 次后退 0.35 m 并原地对准路径方向；每次都从当前位置重新规划剩余路径；第 3 次仍无进展即放弃 (NAV2_GIVEUP)"""
        cancelled = lambda: self.current_mission_id != mission_id  # noqa: E731
        seg_len = [sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(p, p[1:])) for p, _ in segs]
        retries = int(os.environ.get("NAV2_RETRIES", "8"))
        for k, (poses, end_i) in enumerate(segs):
            rest = sum(seg_len[k + 1:])
            with self.lock:
                if cancelled():
                    return
                self.telemetry["path_index"] = max(1, end_i)

            # 本段路径累计长度 (从末端算起)，用于按实时位姿求剩余行程
            tail = [0.0] * len(poses)
            for i in range(len(poses) - 2, -1, -1):
                tail[i] = tail[i + 1] + math.hypot(poses[i + 1][0] - poses[i][0], poses[i + 1][1] - poses[i][1])

            def fb(dist, speed, rest=rest, poses=poses, tail=tail):
                # 分段终点 = 停车点 (拐点原地转向 / 工位)：之外的障碍不影响本段 → 防护区缩短到剩余行程 (与自研导引
                # approach_left 同一套逻辑)。否则紧贴墙体的拓扑节点 (如 grid_9_square 的 (±7.5, 0)，车头离外墙
                # 仅 0.17 m) 会被低速档 0.35 m 防护区挡住，Nav2 "Failed to make progress" → 线路跟随中断反复重试。
                # 注意: Humble 的 FollowPath 反馈 distance_to_goal 不随车辆移动更新 (实测整段恒为段长)，这里按实时位姿自己算
                dist = self._path_remaining(poses, tail)
                self.approach_left = dist
                with self.lock:
                    if not cancelled():
                        self.telemetry["nav_dist_rem"] = round(dist + rest, 2)
                        self.telemetry["nav2_feedback"] = {"distance_remaining": round(dist + rest, 2), "speed": round(speed, 3),
                                                           "segment": k + 1, "segments": len(segs)}
            tries = no_prog = 0
            cur = poses                         # 本次下发的路径 (重新规划后从当前位置接续)
            while True:
                rem0 = self._path_remaining(poses, tail)
                self.approach_left = None
                try:
                    res = self.nav2.follow_path(cur, cancelled, on_feedback=fb)
                finally:
                    self.approach_left = None
                # 动作结束后 Nav2 不再发 cmd_vel，而仿真看门狗会把最后一条指令保持 0.5 s
                # (到位时正在原地转向 → 多转 10° 以上)：显式下发零速
                for _ in range(3):
                    self.link.send_cmd(0.0, 0.0, 0.0, source="nav2")
                    time.sleep(0.02)
                if cancelled():
                    return
                if res == "SUCCEEDED":
                    break
                # 终点对位转向受阻: 已到工位 (≤ 3 cm) 只差停靠朝向 —— Nav2 (RPP) 按 5 cm 栅格 + 激光噪声判定的碰撞预测
                # 在贴墙工位 (车角离墙约 5 cm) 不肯转；改由执行进程的精确原地转向 (实测激光点、rotate_margin 余量) 完成对位
                if res == "ABORTED" and k == len(segs) - 1 and self._near_goal_heading_only(poses[-1]):
                    r = self._rotate_to(mission_id, poses[-1][2], 0.0, 0.5)
                    if r == "abort":
                        return
                    if r == "done":
                        self.event_hub.emit("navigation", "NAV2_ALIGN", "info", "终点对位转向由执行进程完成",
                                            "Nav2 控制器拒绝贴墙原地转向 (碰撞预测)，改用执行进程原地转向", {"mission_id": mission_id})
                        break
                # 段起点原地转向受阻 (如贴墙拓扑节点): 由执行进程精确原地转向 (实测激光点判定) 后接着跟线
                if res == "ABORTED" and self._turn_blocked_at(cur):
                    p0, j = cur[0], min(len(cur) - 1, 6)
                    r = self._rotate_to(mission_id, math.atan2(cur[j][1] - p0[1], cur[j][0] - p0[0]), 0.0, 0.5)
                    if r == "abort":
                        return
                    if r == "done":
                        self.event_hub.emit("navigation", "NAV2_ALIGN", "info", "段起点转向由执行进程完成",
                                            f"第 {k + 1}/{len(segs)} 段: Nav2 控制器拒绝原地转向，改用执行进程原地转向", {"mission_id": mission_id})
                        continue
                # ---- 中断恢复: 看这次有没有进展 → 逐级调整位姿 → 从当前位置重新规划剩余路径
                rem1 = self._path_remaining(poses, tail)
                progressed = rem0 - rem1 > 0.05
                no_prog = 0 if progressed else no_prog + 1
                tries += 1
                if tries > retries or no_prog > len(BACKUP_STEPS):
                    why = (f"连续 {no_prog} 次无进展 (已尝试后退 {'/'.join(f'{d:.2f}' for d in BACKUP_STEPS)} m)" if no_prog > len(BACKUP_STEPS)
                           else f"已重试 {retries} 次")
                    self.event_hub.emit("navigation", "NAV2_GIVEUP", "danger", f"Nav2 线路跟随放弃: {why}",
                                        f"第 {k + 1}/{len(segs)} 段，剩余 {rem1:.2f} m，最后一次结果 {res}", {"mission_id": mission_id})
                    on_result(mission_id, "ABORTED")
                    return
                with self.lock:
                    self.telemetry["nav_status"] = "OBSTACLE_WAIT"
                adjust = "保持位姿 (本次有进展，按动态障碍处理，等待 2 s)"
                if no_prog:
                    d = BACKUP_STEPS[no_prog - 1]
                    r = self.nav2.backup(d, 0.1, cancelled)
                    for _ in range(3):
                        self.link.send_cmd(0.0, 0.0, 0.0, source="nav2")
                        time.sleep(0.02)
                    if cancelled():
                        return
                    adjust = f"Nav2 BackUp 后退 {d:.2f} m ({r})"
                    if no_prog >= 2:            # 第二级: 后退后再原地对准路径方向 (执行进程精确转向)
                        i = self._nearest_index(poses)
                        j = min(len(poses) - 1, i + 6)
                        if j > i:
                            h = math.atan2(poses[j][1] - poses[i][1], poses[j][0] - poses[i][0])
                            if self._rotate_to(mission_id, h, 0.0, 0.5) == "abort":
                                return
                            adjust += "，原地对准路径方向"
                cur = self._replan_from_here(poses)
                self.event_hub.emit("navigation", "NAV2_RETRY", "warning", f"Nav2 线路跟随中断 ({res})",
                                    f"第 {k + 1}/{len(segs)} 段 ({tries}/{retries})：位姿调整: {adjust}；"
                                    f"从当前位置重新规划，剩余 {self._path_remaining(poses, tail):.2f} m",
                                    {"mission_id": mission_id, "no_progress": no_prog})
                if not no_prog:
                    time.sleep(2.0)
                with self.lock:
                    if cancelled():
                        return
                    self.telemetry["nav_status"] = "NAVIGATING"
        on_result(mission_id, "SUCCEEDED")

    def _nearest_index(self, poses) -> int:
        with self.lock:
            x, y = self.telemetry.get("x", 0.0), self.telemetry.get("y", 0.0)
        return min(range(len(poses)), key=lambda k: (poses[k][0] - x) ** 2 + (poses[k][1] - y) ** 2)

    def _path_remaining(self, poses, tail) -> float:
        """按实时位姿求本段剩余行程 (最近点 + 沿路径投影)"""
        with self.lock:
            x, y = self.telemetry.get("x", 0.0), self.telemetry.get("y", 0.0)
        i = self._nearest_index(poses)
        a, b = (poses[i], poses[i + 1]) if i < len(poses) - 1 else (poses[i - 1], poses[i])
        ex, ey = b[0] - a[0], b[1] - a[1]
        L = math.hypot(ex, ey) or 1.0
        return max(0.0, tail[i] - ((x - poses[i][0]) * ex + (y - poses[i][1]) * ey) / L)

    def _replan_from_here(self, poses):
        """重新规划: 从车辆当前位置接到本段路径上最近点之后的部分 (不再退回段起点重走)"""
        with self.lock:
            x, y = self.telemetry.get("x", 0.0), self.telemetry.get("y", 0.0)
        i = self._nearest_index(poses)
        rest = list(poses[i + 1:]) or [poses[-1]]
        h = math.atan2(rest[0][1] - y, rest[0][0] - x) if math.hypot(rest[0][0] - x, rest[0][1] - y) > 1e-3 else poses[i][2]
        return [(x, y, h)] + rest

    def _near_goal_heading_only(self, goal) -> bool:
        """已到终点位置 (≤ 3 cm)，只差朝向 (> 1°)"""
        with self.lock:
            x, y, yaw = self.telemetry.get("x", 0.0), self.telemetry.get("y", 0.0), self.telemetry.get("yaw", 0.0)
        err = abs(math.atan2(math.sin(yaw - goal[2]), math.cos(yaw - goal[2])))
        return math.hypot(x - goal[0], y - goal[1]) <= 0.03 and err > math.radians(1.0)

    def _turn_blocked_at(self, poses) -> bool:
        """车在本段起点附近 (≤ 0.6 m) 且车头与路径方向相差 > 0.2 rad —— 说明卡在段起点的原地转向上"""
        if len(poses) < 2:
            return False
        with self.lock:
            x, y, yaw = self.telemetry.get("x", 0.0), self.telemetry.get("y", 0.0), self.telemetry.get("yaw", 0.0)
        p0 = poses[0]
        j = min(len(poses) - 1, 6)
        h = math.atan2(poses[j][1] - p0[1], poses[j][0] - p0[0])
        err = abs(math.atan2(math.sin(yaw - h), math.cos(yaw - h)))
        return math.hypot(x - p0[0], y - p0[1]) <= 0.6 and err > 0.2

    def _wait_until_stopped(self, mission_id, timeout=6.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.publish_cmd_vel(0.0, 0.0, 0.0)
            with self.lock:
                if self.current_mission_id != mission_id:
                    return
                v = math.hypot(self.telemetry.get("vx", 0.0), self.telemetry.get("vy", 0.0))
            if v < 0.03:
                return
            time.sleep(0.03)

    def cancel_nav(self):
        self.nav2.cancel()
        cpp = self._guide_cpp()
        if cpp is not None:
            cpp.send_guide_cancel()
        self._agv_mission = None
        self.approach_left = None
        with self.lock:
            self.telemetry["nav_status"] = "CANCELED"
        self.push_core(force_mode=True)                 # 立即停止转发 Nav2 指令
        with self.lock:
            self.current_mission_id += 1
            canceled_id = self.current_mission_id - 1
            self.telemetry["nav_status"] = "CANCELED"
            self.telemetry["target_goal"] = None
            self.telemetry["plan_path"] = []
            self.telemetry["nav_dist_rem"] = 0.0
        self.recorder.end_session("CANCELED")
        for _ in range(4):
            self.publish_cmd_vel(0.0, 0.0, 0.0)
            time.sleep(0.02)
        self.event_hub.emit(
            "navigation", "MISSION_CANCELED", "warning",
            "调度任务手动中止",
            f"操作员已中止当前导航任务 #{canceled_id}，底盘已安全急停抱闸",
            {"mission_id": canceled_id}
        )


