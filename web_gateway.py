#!/usr/bin/env python3
"""
Web 网关进程 (无 ROS) —— 端口 8088

  浏览器 ──HTTP──▶ web_gateway ──REST──▶ 仿真进程 sim_server (:8090)   状态/传感器/IO/场景/车型
                                   └─REST──▶ 执行进程 nav_runtime (:8091) 任务/规划器/Nav2/遥控

  * 保持原前端接口 (/api/telemetry、/api/navigate_to_pose ...) 不变，内部全部改为调用两进程的 REST API v1
  * 事件总线合并两进程事件；飞行记录器按执行进程回馈的任务状态自动分段录制
"""

import json
import math
import os
import random
import sys
import threading
import time
import urllib.parse
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler

import psutil

ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, ROOT)

from common.events import EventHub  # noqa: E402
from common.rest import RestClient  # noqa: E402
from planning.dijkstra_planner import SCENARIO_DEFINITIONS, DijkstraPlanner, register_scenario  # noqa: E402

SIM_API = os.environ.get("SIM_API", "http://127.0.0.1:8090")
NAV_API = os.environ.get("NAV_API", "http://127.0.0.1:8091")

event_hub = EventHub(max_history=1200)

class SystemPerformanceMonitor:
    """
    Real-time Raspberry Pi System & ROS2/Simulator Process Resource Monitor
    Tracks:
    - Host CPU % (total & per-core), RAM (used, total, %), CPU Temp (°C), Frequency (MHz), Load Avg
    - ROS & Simulator specific processes:
        * sim_server (仿真进程: MuJoCo 物理、激光/相机/IO)
        * nav_runtime (执行进程: 导引/Nav2)
        * web_gateway (本进程: Web / REST 聚合)
        * nav2_map_server (ROS2 map server)
        * nav2_lifecycle_manager (ROS2 lifecycle manager)
    - 60-second sliding performance timeline
    """
    def __init__(self, history_len=300):
        self.history_len = history_len
        self.history = []
        self.lock = threading.Lock()
        self.latest_stats = {
            "timestamp": time.time(),
            "host": {
                "model": "",
                "cpu_count": os.cpu_count() or 1,
                "cpu_total_percent": 0.0,
                "cpu_per_core": [0.0, 0.0, 0.0, 0.0],
                "cpu_temp_c": 50.0,
                "cpu_freq_mhz": 1800.0,
                "memory_total_mb": 4049.0,
                "memory_used_mb": 800.0,
                "memory_free_mb": 3249.0,
                "memory_percent": 19.8,
                "load_avg": [0.0, 0.0, 0.0]
            },
            "ros_simulation_total": {
                "combined_cpu_percent": 0.0,
                "combined_rss_mb": 0.0,
                "combined_ram_percent": 0.0
            },
            "processes": []
        }
        self.running = True
        self.tracked_procs = {}

        # Start 1Hz background monitoring thread
        self.thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.thread.start()

    def _find_processes(self):
        targets = {
            "sim_server": {"name": "仿真进程 sim_server (REST :8090)", "match": "sim_server.api", "role": "cmodel 解析/构建、物理步进、激光/IO/碰撞数据产生"},
            "nav_runtime": {"name": "执行进程 nav_runtime (ROS2+Nav2, REST :8091)", "match": "nav_runtime.main", "role": "Nav2/拓扑导引执行，cmd_vel 与任务状态回馈仿真进程"},
            "web_gateway": {"name": "Web 网关 web_gateway (:8088)", "match": "web_gateway.py", "role": "聚合两进程 REST，前端/事件/回放"},
            "map_server": {"name": "Nav2 栅格地图服务器 (map_server)", "match": "nav2_map_server/map_server", "role": "ROS2 占据栅格地图发布与代价地图广播"},
            "lifecycle_manager": {"name": "Nav2 生命周期管理 (lifecycle_manager)", "match": "nav2_lifecycle_manager/lifecycle_manager", "role": "ROS2 节点健康监控与生命周期状态切换"},
        }
        my_pid = os.getpid()
        found = {}
        for p in psutil.process_iter(["pid", "cmdline", "name"]):
            if p.pid == my_pid:
                continue
            args = p.info.get("cmdline") or []
            if "-c" in args:        # 跳过 bash -c 包装进程 (按参数匹配: 子串匹配会误伤 sim_server 的 --config，导致每 0.1 s 重扫全部进程)
                continue
            cmd = " ".join(args)
            for key, meta in targets.items():
                if key not in found and meta["match"] in cmd:
                    try:
                        p.cpu_percent(None)  # prime cpu percent
                        found[key] = (p, meta)
                    except Exception:
                        pass
                    break
        self.tracked_procs = found

    def _get_cpu_temp(self):
        try:
            from common.hostinfo import temp_c
            return temp_c()
        except Exception:
            pass
        for path in ["/sys/class/thermal/thermal_zone0/temp", "/sys/devices/virtual/thermal/thermal_zone0/temp"]:
            if os.path.exists(path):
                try:
                    with open(path, "r") as f:
                        return round(float(f.read().strip()) / 1000.0, 1)
                except Exception:
                    pass
        return None

    def _get_cpu_freq(self):
        path = "/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq"
        if os.path.exists(path):
            try:
                with open(path, "r") as f:
                    return round(float(f.read().strip()) / 1000.0, 1)
            except Exception:
                pass
        return None

    def _monitor_loop(self):
        self._find_processes()
        try:
            psutil.cpu_percent(None)
            psutil.cpu_percent(None, percpu=True)
        except Exception:
            pass
        time.sleep(0.5)

        while self.running:
            try:
                cpu_tot = round(psutil.cpu_percent(None), 1)
                per_core = [round(c, 1) for c in psutil.cpu_percent(None, percpu=True)]
                # Android (Termux/proot) 的 /proc/stat 不反映真实负载 → 按可见进程 CPU 时间估算
                if not hasattr(self, "_cpu_est"):
                    from common.hostinfo import CpuEstimator, core_groups, model as _model
                    self._cpu_est, self._model, self._cores = CpuEstimator(), _model(), core_groups()
                est = self._cpu_est.percent()
                cpu_estimated = cpu_tot <= 0.0 and est > 0.0
                if cpu_estimated:
                    cpu_tot = est
                vmem = psutil.virtual_memory()
                temp_c = self._get_cpu_temp()
                freq_mhz = self._get_cpu_freq()
                load_avg = [round(x, 2) for x in os.getloadavg()] if hasattr(os, "getloadavg") else [0.0, 0.0, 0.0]

                if len(self.tracked_procs) < 3 and time.time() - getattr(self, "_scan_t", 0) > 10:
                    self._scan_t = time.time()     # 找不全时最多 10 s 重扫一次 (板卡上 300+ 进程，psutil 全扫约 20 ms)
                    self._find_processes()

                procs_data = []
                sim_cpu = 0.0
                teleop_cpu = 0.0
                map_cpu = 0.0
                total_proc_rss = 0.0

                for key, (p, meta) in list(self.tracked_procs.items()):
                    try:
                        if not p.is_running() or p.status() == psutil.STATUS_ZOMBIE:
                            del self.tracked_procs[key]
                            continue
                        cpu_p = round(p.cpu_percent(None), 1)
                        mem_info = p.memory_info()
                        rss_mb = round(mem_info.rss / (1024 * 1024), 1)
                        total_proc_rss += rss_mb
                        threads = p.num_threads()
                        status = p.status()

                        if key == "sim_server":
                            sim_cpu = cpu_p
                        elif key == "web_gateway":
                            teleop_cpu = cpu_p
                        elif key in ("map_server", "lifecycle_manager"):
                            map_cpu += cpu_p

                        procs_data.append({
                            "key": key,
                            "name": meta["name"],
                            "pid": p.pid,
                            "cpu_percent": cpu_p,
                            "memory_mb": rss_mb,
                            "memory_percent": round((rss_mb / (vmem.total / (1024 * 1024))) * 100.0, 1),
                            "threads": threads,
                            "status": status,
                            "role": meta["role"]
                        })
                    except (psutil.NoSuchProcess, psutil.AccessDenied):
                        if key in self.tracked_procs:
                            del self.tracked_procs[key]

                now = time.time()
                time_label = time.strftime("%H:%M:%S", time.localtime(now))

                point = {
                    "time": time_label,
                    "timestamp": round(now, 2),
                    "cpu_total": cpu_tot,
                    "mem_used_mb": round(vmem.used / (1024 * 1024), 1),
                    "mem_percent": round(vmem.percent, 1),
                    "sim_cpu": sim_cpu,
                    "teleop_cpu": teleop_cpu,
                    "map_cpu": map_cpu,
                    "temp_c": temp_c or 50.0
                }

                snapshot = {
                    "timestamp": now,
                    "host": {
                        "model": self._model,
                        "cpu_count": psutil.cpu_count(),
                        "cpu_estimated": cpu_estimated,
                        "core_groups": self._cores,
                        "cpu_total_percent": cpu_tot,
                        "cpu_per_core": per_core,
                        "cpu_temp_c": temp_c,
                        "cpu_freq_mhz": freq_mhz,
                        "memory_total_mb": round(vmem.total / (1024 * 1024), 1),
                        "memory_used_mb": round(vmem.used / (1024 * 1024), 1),
                        "memory_free_mb": round(vmem.available / (1024 * 1024), 1),
                        "memory_percent": round(vmem.percent, 1),
                        "load_avg": load_avg
                    },
                    "ros_simulation_total": {
                        "combined_cpu_percent": round(sim_cpu + teleop_cpu + map_cpu, 1),
                        "combined_rss_mb": round(total_proc_rss, 1),
                        "combined_ram_percent": round((total_proc_rss / (vmem.total / (1024 * 1024))) * 100.0, 1)
                    },
                    "processes": procs_data
                }

                with self.lock:
                    if "bullet_simulation" in self.latest_stats:
                        snapshot["bullet_simulation"] = self.latest_stats["bullet_simulation"]
                    self.latest_stats = snapshot
                    self.history.append(point)
                    if len(self.history) > self.history_len:
                        self.history.pop(0)

            except Exception:
                pass

            time.sleep(0.1)

    def set_bullet_metrics(self, metrics: dict):
        with self.lock:
            self.latest_stats["bullet_simulation"] = metrics

    def get_stats(self):
        with self.lock:
            data = dict(self.latest_stats)
            data["history"] = list(self.history)
            return data



perf_monitor = SystemPerformanceMonitor()



class FlightRecorder:
    """
    Simulation Blackbox & Flight Recorder for AMR Studio V4.
    Records motion trajectories, planned routes, perception sensor frames,
    chassis command inputs, IO interlocks and hardware performance at 10Hz.
    Supports auto mission session slicing and manual recording window capture.
    """
    def __init__(self, max_sessions: int = 20, max_rolling_frames: int = 1200):
        self.lock = threading.Lock()
        self.max_sessions = max_sessions
        self.max_rolling_frames = max_rolling_frames
        self.completed_sessions = []
        self.active_session = None
        self.rolling_buffer = []
        self.last_cmd = {"vx": 0.0, "vy": 0.0, "wz": 0.0}

    def set_cmd_vel(self, vx: float, vy: float, wz: float):
        with self.lock:
            self.last_cmd = {"vx": round(vx, 3), "vy": round(vy, 3), "wz": round(wz, 3)}

    def is_active_recording(self) -> bool:
        with self.lock:
            return self.active_session is not None

    def start_session(self, session_type: str = "mission", metadata: dict = None) -> str:
        with self.lock:
            if self.active_session:
                self._finalize_active_session("AUTO_CLOSED")

            now = time.time()
            sid = f"{session_type}_{int(now)}_{random.randint(100, 999)}"
            self.active_session = {
                "id": sid,
                "type": session_type,
                "start_time": now,
                "end_time": None,
                "duration_s": 0.0,
                "total_distance_m": 0.0,
                "status": "RECORDING",
                "metadata": metadata or {},
                "planned_route": (metadata.get("planned_route", {}) if metadata else {}),
                "frames": []
            }
            return sid

    def end_session(self, status: str = "COMPLETED"):
        with self.lock:
            return self._finalize_active_session(status)

    def _finalize_active_session(self, status: str):
        if not self.active_session:
            return None
        session = self.active_session
        now = time.time()
        session["end_time"] = now
        session["status"] = status
        session["duration_s"] = round(now - session["start_time"], 2)
        session["total_frames"] = len(session["frames"])

        # Compute analytical statistics
        total_dist = 0.0
        max_speed = 0.0
        speeds = []
        min_dist_overall = 99.0
        frames = session["frames"]

        for i in range(len(frames)):
            f = frames[i]
            if i > 0:
                p0 = frames[i-1]["pose"]
                p1 = f["pose"]
                dist = math.hypot(p1["x"] - p0["x"], p1["y"] - p0["y"])
                total_dist += dist
            v_curr = math.hypot(f["vel"]["vx"], f["vel"]["vy"])
            speeds.append(v_curr)
            if v_curr > max_speed:
                max_speed = v_curr
            m_dist = f.get("perception", {}).get("min_dist", 99.0)
            if m_dist < min_dist_overall:
                min_dist_overall = m_dist

        session["total_distance_m"] = round(total_dist, 2)
        session["max_speed_mps"] = round(max_speed, 2)
        session["avg_speed_mps"] = round(sum(speeds) / max(1, len(speeds)), 2)
        session["min_obstacle_dist_m"] = round(min_dist_overall, 2) if min_dist_overall < 90 else 12.0

        self.completed_sessions.insert(0, session)
        if len(self.completed_sessions) > self.max_sessions:
            self.completed_sessions.pop()

        self.active_session = None
        return session

    def capture_rolling_as_session(self, seconds: float = 30.0, title: str = None) -> dict:
        with self.lock:
            now = time.time()
            cutoff = now - seconds
            frames = [f for f in self.rolling_buffer if f["t"] >= cutoff]
            if not frames:
                return None

            sid = f"manual_slice_{int(now)}_{random.randint(100, 999)}"
            start_t = frames[0]["t"]
            for f in frames:
                f["rel_t"] = round(f["t"] - start_t, 2)

            total_dist = 0.0
            max_speed = 0.0
            speeds = []
            min_dist_overall = 99.0
            for i in range(len(frames)):
                f = frames[i]
                if i > 0:
                    dist = math.hypot(f["pose"]["x"] - frames[i-1]["pose"]["x"], f["pose"]["y"] - frames[i-1]["pose"]["y"])
                    total_dist += dist
                v_curr = math.hypot(f["vel"]["vx"], f["vel"]["vy"])
                speeds.append(v_curr)
                if v_curr > max_speed:
                    max_speed = v_curr
                m_dist = f.get("perception", {}).get("min_dist", 99.0)
                if m_dist < min_dist_overall:
                    min_dist_overall = m_dist

            session = {
                "id": sid,
                "type": "manual_slice",
                "start_time": start_t,
                "end_time": now,
                "duration_s": round(now - start_t, 2),
                "total_frames": len(frames),
                "total_distance_m": round(total_dist, 2),
                "max_speed_mps": round(max_speed, 2),
                "avg_speed_mps": round(sum(speeds) / max(1, len(speeds)), 2),
                "min_obstacle_dist_m": round(min_dist_overall, 2) if min_dist_overall < 90 else 12.0,
                "status": "COMPLETED",
                "metadata": {
                    "title": title or f"手动切片记录 (近 {int(seconds)} 秒)",
                    "source": "rolling_buffer"
                },
                "planned_route": {},
                "frames": frames
            }
            self.completed_sessions.insert(0, session)
            if len(self.completed_sessions) > self.max_sessions:
                self.completed_sessions.pop()
            return session

    def record_frame(self, telemetry: dict, perf_summary: dict = None):
        now = time.time()
        with self.lock:
            cmd = dict(self.last_cmd)
            raw_ranges = telemetry.get("scan_ranges", [])
            ranges_sample = []
            if raw_ranges:
                step = max(1, len(raw_ranges) // 36)
                ranges_sample = [round(float(r), 2) for r in raw_ranges[::step][:36]]

            frame = {
                "t": round(now, 3),
                "pose": {
                    "x": round(float(telemetry.get("x", 0.0)), 3),
                    "y": round(float(telemetry.get("y", 0.0)), 3),
                    "yaw": round(float(telemetry.get("yaw", 0.0)), 4),
                    "yaw_deg": round(math.degrees(float(telemetry.get("yaw", 0.0))), 1)
                },
                "vel": {
                    "vx": round(float(telemetry.get("vx", 0.0)), 3),
                    "vy": round(float(telemetry.get("vy", 0.0)), 3),
                    "wz": round(float(telemetry.get("wz", 0.0)), 3)
                },
                "cmd": cmd,
                "nav": {
                    "status": telemetry.get("nav_status", "IDLE"),
                    "dist_rem": telemetry.get("nav_dist_rem", 0.0)
                },
                "io": {
                    "estop": bool(telemetry.get("io_states", {}).get("is_emergency_stop", False)),
                    "bumper": bool(telemetry.get("io_states", {}).get("inputs", {}).get("di_bumper_front", False)),
                    "cargo": bool(telemetry.get("io_states", {}).get("inputs", {}).get("di_cargo_present", False)),
                    "brake": bool(telemetry.get("io_states", {}).get("outputs", {}).get("do_brake_release", True))
                },
                "perception": {
                    "min_dist": telemetry.get("scan_min_dist", 12.0),
                    "ranges": ranges_sample,
                    "markers": list(telemetry.get("vision_markers", [])),
                    "obstacles": list(telemetry.get("dynamic_obstacles", []))
                },
                "perf": perf_summary or {}
            }

            self.rolling_buffer.append(frame)
            if len(self.rolling_buffer) > self.max_rolling_frames:
                self.rolling_buffer.pop(0)

            if self.active_session:
                frame_copy = dict(frame)
                frame_copy["rel_t"] = round(now - self.active_session["start_time"], 2)
                self.active_session["frames"].append(frame_copy)

    def get_session_list(self):
        with self.lock:
            res = []
            if self.active_session:
                s = self.active_session
                res.append({
                    "id": s["id"],
                    "type": s["type"],
                    "start_time": s["start_time"],
                    "duration_s": round(time.time() - s["start_time"], 1),
                    "total_frames": len(s["frames"]),
                    "status": "RECORDING",
                    "title": s["metadata"].get("title", "实时录制中..."),
                    "scenario": s["metadata"].get("scenario", "default"),
                    "origin": s["metadata"].get("origin", {}),
                    "destination": s["metadata"].get("destination", {})
                })
            for s in self.completed_sessions:
                res.append({
                    "id": s["id"],
                    "type": s["type"],
                    "start_time": s["start_time"],
                    "end_time": s["end_time"],
                    "duration_s": s["duration_s"],
                    "total_frames": s["total_frames"],
                    "total_distance_m": s["total_distance_m"],
                    "max_speed_mps": s.get("max_speed_mps", 0.0),
                    "avg_speed_mps": s.get("avg_speed_mps", 0.0),
                    "min_obstacle_dist_m": s.get("min_obstacle_dist_m"),
                    "status": s["status"],
                    "title": s["metadata"].get("title", f"航段 {s['id']}"),
                    "scenario": s["metadata"].get("scenario", "default"),
                    "origin": s["metadata"].get("origin", {}),
                    "destination": s["metadata"].get("destination", {})
                })
            return res

    def get_session(self, session_id: str):
        with self.lock:
            if self.active_session and self.active_session["id"] == session_id:
                return dict(self.active_session)
            for s in self.completed_sessions:
                if s["id"] == session_id:
                    return dict(s)
            # Default to latest completed session if no id provided
            if not session_id and self.completed_sessions:
                return dict(self.completed_sessions[0])
            return None


flight_recorder = FlightRecorder()




class _String:
    def __init__(self):
        self.data = ""


String = _String


class _Pub:
    def __init__(self, fn):
        self.fn = fn

    def publish(self, msg):
        try:
            self.fn(msg.data)
        except Exception as e:
            print(f"[gateway] 下发失败: {e}", flush=True)


class _Logger:
    def info(self, m): print("[gateway] " + str(m), flush=True)
    warn = error = info


LEVEL_MAP = {"danger": "danger", "warning": "warning", "success": "success", "info": "info"}


class Gateway:
    """聚合仿真进程与执行进程的 REST 数据，向前端提供与原 WebTeleopBridge 相同的接口"""

    def __init__(self):
        self.sim = RestClient(SIM_API, timeout=2.0)
        self.nav = RestClient(NAV_API, timeout=2.0)
        self.lock = threading.Lock()
        self.event_hub = event_hub
        self.is_paused = False
        self.dynamic_obstacles = []
        self.active_scenario = "grid_9_square"
        self.active_chassis_type = "single_steer"
        self.active_planner = "dijkstra"
        self.dijkstra_planner = DijkstraPlanner(self.active_scenario)
        self.walls = self.dijkstra_planner.get_walls()
        self.lidar_config = {"beams": 720, "angle_resolution_deg": 0.5, "freq_hz": 10.0, "range_max": 30.0}
        self.model, self.world, self.nav_status = {}, {}, {}
        self.sim_online = self.nav_online = False
        self.want_scans_until = 0.0
        self.last_client = time.time()   # 最近一次有网页/工具访问网关的时刻 (空闲时降低对仿真的轮询频率)
        self._sim_ev = self._nav_ev = 0
        self._last_mission = None
        self._last_status = None
        self.telemetry = {"x": 0.0, "y": 0.0, "yaw": 0.0, "vx": 0.0, "vy": 0.0, "wz": 0.0, "nav_status": "IDLE",
                          "plan_path": [], "path_labels": [], "path_index": 0, "next_segment": None, "target_goal": None, "nav_dist_rem": 0.0, "is_paused": False,
                          "dynamic_obstacles": [], "obstacles": [], "scan_ranges": [], "io_states": {},
                          "chassis_type": self.active_chassis_type, "planner_type": self.active_planner,
                          "active_scenario": self.active_scenario, "map_scenario": self.active_scenario,
                          "lidar_config": self.lidar_config, "timestamp": time.time(), "last_update": time.time()}
        self.obstacle_pub = _Pub(lambda d: self.sim.put("/api/v1/world/obstacles", json.loads(d)))
        self.pause_pub = _Pub(lambda d: self.sim.put("/api/v1/sim", {"paused": bool(json.loads(d).get("paused"))}))
        self.scenario_pub = _Pub(lambda d: self.sim.post("/api/v1/sim/reset", {}))

    def get_logger(self):
        return _Logger()

    # ------------------------------------------------------------------ 轮询
    def start(self):
        threading.Thread(target=self._sim_loop, daemon=True, name="gw-sim").start()
        threading.Thread(target=self._nav_loop, daemon=True, name="gw-nav").start()
        threading.Thread(target=self._slow_loop, daemon=True, name="gw-slow").start()
        threading.Thread(target=self._recorder_loop, daemon=True, name="gw-rec").start()

    def _sim_loop(self):
        while True:
            t0 = time.time()
            try:
                snap = self.sim.get("/api/v1/snapshot" + ("?scans=1" if time.time() < self.want_scans_until else ""))
                self.sim_online = True
                self._apply_snapshot(snap)
            except Exception:
                self.sim_online = False
                time.sleep(0.5)
            # 有客户端 (网页/工作台/测试工具) 在看或任务执行中 (录制) → 20 Hz；空闲 → 1 Hz
            # (快照是仿真进程最大的 JSON 负载；GW_IDLE_POLL_S=0.05 恢复一直 20 Hz)
            with self.lock:
                busy = self.telemetry.get("nav_status") in ("NAVIGATING", "PLANNING", "OBSTACLE_WAIT", "DOCKING")
            idle = float(os.environ.get("GW_IDLE_POLL_S", "1.0"))
            period = 0.05 if busy or time.time() - self.last_client < 3.0 else idle
            time.sleep(max(0.0, period - (time.time() - t0)))

    def _nav_loop(self):
        while True:
            try:
                st = self.nav.get("/api/v1/nav")
                self.nav_online = True
                with self.lock:
                    self.nav_status = st
                    self.active_planner = st.get("planner", self.active_planner)
                    self.telemetry["planner_type"] = self.active_planner
                    if st.get("ros"):
                        self.telemetry["ros_graph"] = {"nodes": st["ros"].get("nodes", []), "topic_count": len(st["ros"].get("topics", [])),
                                                       "topics": st["ros"].get("topics", [])[:80]}
                evs = self.nav.get(f"/api/v1/events?since={self._nav_ev}")
                for e in evs.get("events", []):
                    self._nav_ev = max(self._nav_ev, e["id"])
                    self.event_hub.emit(e.get("category", "navigation"), e.get("type", "NAV"), e.get("level", "info"),
                                        "[执行进程] " + e.get("title", ""), e.get("message", ""), e.get("payload"))
            except Exception:
                self.nav_online = False
                time.sleep(1.0)
            time.sleep(0.2)

    def _slow_loop(self):
        cnt = 0
        while True:
            try:
                if not self.model or cnt % 10 == 0 or getattr(self, "_model_dirty", False):
                    self._model_dirty = False
                    m = self.sim.get("/api/v1/model")
                    if (m.get("active_chassis") != self.model.get("active_chassis") or m.get("model_file") != self.model.get("model_file")
                            or m.get("model_rev") != self.model.get("model_rev")):
                        self._apply_model(m)
                w = self.sim.get("/api/v1/world")
                if w.get("id") != self.world.get("id") or not self.world:
                    self._apply_world(w)
                if not self.is_paused:   # 以仿真进程的障碍物为准 (其它客户端经 REST 布置的也能显示)
                    with self.lock:
                        self.dynamic_obstacles = [dict(o, active=True) for o in w.get("obstacles", [])]
                evs = self.sim.get(f"/api/v1/events?since={self._sim_ev}")
                for e in evs.get("events", []):
                    self._sim_ev = max(self._sim_ev, e["id"])
                    cat = {"COLLISION": "safety", "ESTOP": "safety"}.get(e["type"], "system")
                    self.event_hub.emit(cat, e["type"], LEVEL_MAP.get(e["level"], "info"), "[仿真进程] " + e["message"], e["message"], e.get("data"))
            except Exception:
                pass
            cnt += 1
            time.sleep(0.5)

    def _apply_model(self, spec):
        with self.lock:
            self.model = spec
            ch = spec.get("chassis", {})
            self.telemetry["config"] = spec
            self.telemetry["robot_spec"] = {
                "model_file": spec.get("model_file"), "native_chassis": ch.get("type"), "active_chassis": spec.get("active_chassis"),
                "model_rev": spec.get("model_rev"), "photoelectric": spec.get("photoelectric", []), "lift": spec.get("lift"),
                "footprint": ch.get("footprint"), "wheels": spec.get("active_wheels", spec.get("wheels", [])),
                "lidars": [{k: l.get(k) for k in ("name", "model", "x", "y", "z", "yaw", "inverted", "fov_deg", "max_range")} for l in spec.get("lidars", [])],
                "cameras": spec.get("cameras", []), "warnings": spec.get("warnings", []), "assumptions": spec.get("assumptions", []),
                "chassis": {k: ch.get(k) for k in ("type", "cmodel_type", "cmodel_name", "length_m", "width_m", "height_m", "head_offset_m",
                                                   "tail_offset_m", "left_offset_m", "right_offset_m", "max_speed_mps", "max_accel_mps2",
                                                   "max_decel_mps2", "max_ang_speed_radps", "mass_kg")},
                "imu": spec.get("imu"), "battery": spec.get("battery"), "io": spec.get("io"), "other_actuators": spec.get("other_actuators", []),
                "lidars_full": [{k: l.get(k) for k in ("name", "model", "vendor_model", "type", "x", "y", "z", "roll", "pitch", "yaw", "inverted",
                                                       "fov_deg", "min_range", "max_range", "resolution_deg", "freq_hz", "spec_source",
                                                       "vfov_min_deg", "vfov_max_deg", "point_rate", "topic", "lines", "scan_pattern")}
                                for l in spec.get("lidars", [])],
            }

    def _apply_world(self, w):
        if w.get("scenario_def"):
            try:
                register_scenario(w["scenario_def"])     # 平台下发的场景 (本地场景库没有)
            except Exception:
                pass
        with self.lock:
            self.world = w
            self.active_scenario = w.get("id", self.active_scenario)
            self.dijkstra_planner.set_scenario(self.active_scenario)
            self.walls = self.dijkstra_planner.get_walls()
            meta = {k: v for k, v in w.items() if k not in ("topology", "obstacles", "heights")}
            self.telemetry["scenario_metadata"] = meta
            self.telemetry["topo_graph"] = w.get("topology")
            self.telemetry["active_scenario"] = self.telemetry["map_scenario"] = self.active_scenario

    def _apply_snapshot(self, s):
        st, sim, nav = s["state"], s.get("sim", {}), s.get("nav", {})
        if self.model and st.get("model_rev") != self.model.get("model_rev"):
            self._model_dirty = True
        tr, od = st["truth"], st["odom"]
        with self.lock:
            T = self.telemetry
            T.update({"x": tr["x"], "y": tr["y"], "yaw": tr["yaw"], "vx": tr["vx"], "vy": tr["vy"], "wz": tr["wz"],
                      "odom_pose": {"x": round(od["x"], 3), "y": round(od["y"], 3)},
                      "chassis_type": st.get("chassis"), "is_paused": st.get("paused", False),
                      "io_states": s.get("io", {}), "vision_markers": s.get("codes", []),
                      "joint_states": {"names": st["joints"]["names"], "positions": [round(p, 4) for p in st["joints"]["position"]],
                                       "velocities": [round(v, 4) for v in st["joints"]["velocity"]]},
                      "sim_status": sim, "actuators": sim.get("kinematics"), "timestamp": time.time(), "last_update": time.time(),
                      "dynamic_obstacles": list(self.dynamic_obstacles), "obstacles": list(self.dynamic_obstacles)})
            sim_paused = bool(st.get("paused", False))
            ov = getattr(self, "_pause_override", None)
            if ov and time.time() < ov[1] and sim_paused != ov[0]:
                sim_paused = ov[0]
            else:
                self._pause_override = None
            self.is_paused = sim_paused
            T["is_paused"] = sim_paused
            self.active_chassis_type = st.get("chassis", self.active_chassis_type)
            ms = s.get("merged_scan")
            if ms:
                rmax = float(ms["range_max"])
                rs = [rmax if r is None else r for r in ms["ranges"]]
                step = max(1, len(rs) // 720)
                T["scan_ranges"] = [round(r, 2) for r in rs[::step]]
                T["scan_angle_min"], T["scan_angle_inc"] = ms["angle_min"], ms["angle_increment"] * step
                T["scan_angle_max"] = ms["angle_min"] + ms["angle_increment"] * (len(rs) - 1)
                fin = [r for r in rs if r < rmax]
                T["scan_min_dist"] = round(min(fin), 2) if fin else rmax
                T["scan_range_max"] = rmax
                T["scan_pose"] = ms.get("pose")
                self.lidar_config = {"beams": len(rs), "angle_resolution_deg": round(math.degrees(ms["angle_increment"]), 3),
                                     "freq_hz": ms.get("scan_hz", 10.0), "total_rays_per_sec": int(len(rs) * ms.get("scan_hz", 10.0)), "range_max": rmax}
                T["lidar_config"] = self.lidar_config
            if "lidar_scans" in s:
                T["lidar_scans"] = s["lidar_scans"]
            T["photoelectric"] = s.get("photoelectric", [])
            T["cameras"] = s.get("cameras", [])
            T["bumpers"] = s.get("bumpers", {})
            T["safety"] = nav.get("safety")
            T["taskflow"] = nav.get("taskflow")
            T["localization"] = nav.get("localization")      # 执行进程定位 (SLAM+里程计)：估计位姿 / 误差 / 协方差
            # 执行进程回馈 (经仿真进程中转) —— 导航状态以此为准
            if nav.get("online"):
                T["nav_status"] = nav.get("status", "IDLE")
                T["plan_path"] = nav.get("path") or []
                T["path_labels"] = nav.get("path_labels") or []
                T["plan_curve"] = nav.get("curve") or []
                T["protection"] = nav.get("protection")
                T["path_index"] = nav.get("path_index", 0)
                T["next_segment"] = nav.get("next_segment")
                T["target_goal"] = nav.get("goal")
                T["nav_dist_rem"] = nav.get("dist_remaining", 0.0)
                T["nav2"] = nav.get("nav2")
                T["nav2_feedback"] = nav.get("nav2_feedback")
                T["planner_type"] = nav.get("planner", T.get("planner_type"))
            else:
                T["nav2"] = {"msgs": False, "server_ready": False, "process": False}
            T["bullet_metrics"] = self._metrics(sim)
            T["arch"] = {"sim_api": SIM_API, "nav_api": NAV_API, "sim_online": self.sim_online, "nav_online": self.nav_online,
                         "nav_feedback_age_s": nav.get("age_s"), "control": s.get("control"),
                         "link": (self.nav_status or {}).get("link")}
            mission = nav.get("mission") or {}
        self._track_mission(mission, nav)

    def _metrics(self, s):
        if not s:
            return {}
        lid = s.get("lidars", [])
        rays = sum(l.get("beams", 0) * l.get("freq_hz", 10) for l in lid)
        dt = s.get("dt", 0.01) or 0.01
        return {"engine": f"SimCore/{s.get('backend')} (独立仿真进程)", "mode": f"固定步长 {dt * 1000:.0f} ms · RTF {s.get('rtf')}",
                "parameters": {"fixed_timestep_s": dt, "physics_rate_hz": round(1 / dt, 1), "num_solver_iterations": 0,
                               "gravity": [0, 0, -9.81], "wheel_friction": 1.2, "ground_friction": 1.0},
                "data_volume": {"total_rigid_bodies": 0, "obstacle_bodies": len(self.dynamic_obstacles), "simulated_joints": 0,
                                "active_contact_points": s.get("collisions", 0), "lidar_beams_per_scan": sum(l.get("beams", 0) for l in lid),
                                "lidar_scan_freq_hz": max([l.get("freq_hz", 0) for l in lid] or [0]), "lidar_rays_per_second": int(rays),
                                "physics_steps_total": int(s.get("sim_time", 0) / dt)},
                "performance": {"physics_step_ms": s.get("step_ms"), "raycast_step_ms": s.get("lidar_ms"),
                                "physics_capacity_hz": round(1000.0 / max(s.get("step_ms", 1) or 1, 0.01)),
                                "raycast_throughput_rays_per_ms": 0, "step_cpu_time_pct": round((s.get("step_ms") or 0) / (dt * 1000) * 100, 2),
                                "raycast_cpu_time_pct": round((s.get("lidar_ms") or 0) * 10 / 1000 * 100, 2),
                                "combined_core_load_pct": 0, "rtf": s.get("rtf"), "overruns": s.get("overruns")},
                "sim_status": s}

    def _track_mission(self, m, nav):
        """按执行进程回馈的任务状态驱动飞行记录器分段"""
        mid, status = m.get("id"), m.get("status")
        if mid and mid != self._last_mission:
            if self._last_mission is not None:
                flight_recorder.end_session("SUPERSEDED")
            self._last_mission = mid
            flight_recorder.start_session("mission", {"mission_id": mid, "title": m.get("title") or f"任务 #{mid}",
                                                      "scenario": self.active_scenario, "chassis": self.active_chassis_type,
                                                      "planner": m.get("planner"), "destination": m.get("goal"),
                                                      "planned_route": {"planner": m.get("planner"),
                                                                        "waypoints": [[p["x"], p["y"]] for p in (m.get("path") or [])]}})
        if mid and status != self._last_status and status in ("ARRIVED", "FAILED", "CANCELED", "NO_PATH", "REJECTED"):
            flight_recorder.end_session(status)
        self._last_status = status

    # ------------------------------------------------------------------ 指令 (→ 两进程 REST)
    def publish_cmd_vel(self, vx, vy=0.0, wz=0.0):
        flight_recorder.set_cmd_vel(vx, vy, wz)
        if self.nav.safe("POST", "/api/v1/teleop", {"vx": vx, "vy": vy, "wz": wz}) is None:
            self.sim.safe("PUT", "/api/v1/control/cmd_vel", {"vx": vx, "vy": vy, "wz": wz, "source": "web-teleop"})

    def send_nav_goal(self, x, y, yaw=0.0):
        try:
            m = self.nav.post("/api/v1/missions", {"x": x, "y": y, "yaw": yaw})
            with self.lock:
                self.telemetry["nav_status"] = m.get("status", "PLANNING")
        except Exception as e:
            self.event_hub.emit("navigation", "MISSION_FAILED", "danger", "任务下发失败", f"执行进程不可用: {e}", {})

    def cancel_nav(self):
        self.nav.safe("DELETE", "/api/v1/missions/current")

    def set_planner_type(self, planner_type):
        r = self.nav.safe("PUT", "/api/v1/nav/planner", {"type": planner_type})
        if r:
            with self.lock:
                self.active_planner = r["planner"]
                self.telemetry["planner_type"] = self.active_planner

    def set_chassis_type(self, chassis_type):
        if self.sim.safe("PUT", "/api/v1/model/chassis", {"type": chassis_type}) is not None:
            self.model = {}
            self.event_hub.emit("chassis", "CHASSIS_SWITCH", "info", "底盘运动学模型切换", f"仿真进程车型 → {chassis_type}", {"chassis_type": chassis_type})

    def set_map_scenario(self, scenario_id):
        if scenario_id not in SCENARIO_DEFINITIONS:
            return
        self.cancel_nav()
        with self.lock:
            self.dynamic_obstacles = []
        self.sim.safe("PUT", "/api/v1/world/obstacles", [])
        self.sim.safe("PUT", "/api/v1/world/scenario", {"id": scenario_id})
        self.world = {}
        self.event_hub.emit("system", "SCENARIO_SWITCH", "info", "仓储场景地图切换", f"仿真进程场景 → {scenario_id}", {"scenario_id": scenario_id})

    def set_lidar_config(self, beams=None, angle_resolution_deg=None, freq_hz=None, range_max=None):
        body = {k: v for k, v in (("beams", beams), ("angle_resolution_deg", angle_resolution_deg), ("freq_hz", freq_hz), ("range_max", range_max)) if v is not None}
        r = self.sim.safe("PUT", "/api/v1/sensors/lidar_config", body) or self.lidar_config
        self.lidar_config.update(r)
        return self.lidar_config

    def set_io(self, key, value):
        self.sim.safe("PUT", f"/api/v1/io/di/{key}", {"value": bool(value)})

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

    def broadcast_obstacles(self):
        s = String()
        with self.lock:
            s.data = json.dumps(self.dynamic_obstacles)
        self.obstacle_pub.publish(s)

    def clear_obstacles(self):
        with self.lock:
            self.dynamic_obstacles = []
            self.telemetry["dynamic_obstacles"] = []
            self.telemetry["obstacles"] = []
        self.broadcast_obstacles()
        self.get_logger().info("Cleared all dynamic obstacles")
        self.event_hub.emit(
            "sensors", "OBSTACLE_CHANGE", "info",
            "动态干扰路障已清空",
            "已清空移除主干道上所有动态干扰路障与物理实体，导轨路网恢复畅通",
            {"count": 0}
        )

    def add_obstacle(self, x: float, y: float, w: float = 0.8, h: float = 0.8, obs_type: str = "box"):
        with self.lock:
            if not self.is_paused:
                return False, "仅在物理仿真暂停状态下允许添加扰动模块，请先暂停仿真"
            new_id = len(self.dynamic_obstacles) + 1
            obs = {
                "id": new_id,
                "x": round(x, 2),
                "y": round(y, 2),
                "w": round(w, 2),
                "h": round(h, 2),
                "type": obs_type,
                "active": False  # 待恢复仿真后生效
            }
            self.dynamic_obstacles.append(obs)
            self.telemetry["dynamic_obstacles"] = list(self.dynamic_obstacles)
            self.telemetry["obstacles"] = list(self.dynamic_obstacles)
        self.broadcast_obstacles()
        self.get_logger().info(f"Added staged obstacle #{new_id} ({obs_type}) at ({x}, {y}), waiting for unpause to activate")
        type_names = {
            "pallet": "标准木质栈板",
            "shelf": "双层轻型货架",
            "box": "工业周转纸箱",
            "person": "车间作业人员"
        }
        type_name = type_names.get(obs_type, "工业实体障碍物")
        self.event_hub.emit(
            "sensors", "OBSTACLE_CHANGE", "warning",
            f"布置扰动物理实体 #{new_id} ({type_name})",
            f"在坐标 ({x:.2f}, {y:.2f}) 放置规格为 {w:.2f}x{h:.2f}m 的{type_name}，将在恢复仿真时正式生效",
            obs
        )
        return True, "ok"

    def generate_random_obstacles(self, count: int = 3):
        import random
        with self.lock:
            cur_x, cur_y = self.telemetry["x"], self.telemetry["y"]
            stations = [(st["x"], st["y"]) for st in self.dijkstra_planner.get_stations()]

        # Generate on valid topological nodes
        nodes = list(self.dijkstra_planner.nodes.values())
        random.shuffle(nodes)
        new_obs = []

        type_defs = {
            "pallet": (1.2, 1.0),
            "shelf": (2.0, 1.0),
            "box": (0.8, 0.8),
            "person": (0.5, 0.5)
        }
        type_keys = list(type_defs.keys())

        for nx, ny in nodes:
            if len(new_obs) >= count:
                break
            ox = round(nx + random.uniform(-0.15, 0.15), 2)
            oy = round(ny + random.uniform(-0.15, 0.15), 2)

            if math.hypot(ox - cur_x, oy - cur_y) < 1.4:
                continue
            if any(math.hypot(ox - sx, oy - sy) < 1.1 for sx, sy in stations):
                continue

            chosen_type = random.choice(type_keys)
            dw, dh = type_defs[chosen_type]
            ow = round(dw, 2)
            oh = round(dh, 2)
            new_obs.append({
                "id": len(new_obs) + 1,
                "x": ox,
                "y": oy,
                "w": ow,
                "h": oh,
                "type": chosen_type,
                "active": not self.is_paused
            })

        with self.lock:
            self.dynamic_obstacles = new_obs
            self.telemetry["dynamic_obstacles"] = list(self.dynamic_obstacles)
            self.telemetry["obstacles"] = list(self.dynamic_obstacles)
        self.broadcast_obstacles()
        self.get_logger().info(f"Generated {len(new_obs)} random obstacles")
        self.event_hub.emit(
            "sensors", "OBSTACLE_CHANGE", "warning",
            "随机布置动态干扰物理实体",
            f"在仓储拓扑主通道内随机部署了 {len(new_obs)} 处工业干扰实体(托盘/货架/纸箱/人员)，检验动态避障能力",
            {"count": len(new_obs), "obstacles": new_obs}
        )

    def set_simulation_pause(self, paused: bool):
        with self.lock:
            self.is_paused = paused
            self._pause_override = (paused, time.time() + 2.0)   # 仿真进程确认前，旧快照不得覆盖本地暂停状态
            self.telemetry["is_paused"] = self.is_paused
            if not paused:
                # 恢复仿真时物体生效：激活所有暂存的障碍物实体
                for obs in self.dynamic_obstacles:
                    obs["active"] = True
                self.telemetry["dynamic_obstacles"] = list(self.dynamic_obstacles)
                self.telemetry["obstacles"] = list(self.dynamic_obstacles)
        msg = String()
        msg.data = json.dumps({"paused": paused})
        self.pause_pub.publish(msg)
        if not paused:
            self.broadcast_obstacles()
        action_name = "暂停" if paused else "恢复"
        self.get_logger().info(f"Simulation {action_name}")
        self.event_hub.emit(
            "system", "SIM_STATE_CHANGE", "info",
            f"仿真环境已{action_name}",
            f"操作员已{action_name}仿真时钟与物理步进" + ("，所有暂存扰动模块已正式生效进入物理碰撞引擎" if not paused else "，当前可布置扰动模块"),
            {"paused": paused}
        )

    def reset_simulation(self):
        self.cancel_nav()
        with self.lock:
            sc = SCENARIO_DEFINITIONS.get(self.active_scenario, {})
            orig = sc.get("origin", {"x": 0.0, "y": 0.0, "yaw": 0.0})
            self.telemetry["x"] = float(orig["x"])
            self.telemetry["y"] = float(orig["y"])
            self.telemetry["yaw"] = float(orig.get("yaw", 0.0))
            self.telemetry["vx"] = 0.0
            self.telemetry["vy"] = 0.0
            self.telemetry["wz"] = 0.0
            self.telemetry["plan_path"] = []
            self.telemetry["target_goal"] = None
            self.telemetry["nav_status"] = "IDLE"
            self.telemetry["nav_dist_rem"] = 0.0
            self.telemetry["scan_pose"] = {
                "x": float(orig["x"]),
                "y": float(orig["y"]),
                "yaw": float(orig.get("yaw", 0.0))
            }
        self.clear_obstacles()
        msg = String()
        msg.data = self.active_scenario
        self.scenario_pub.publish(msg)
        self.set_simulation_pause(False)
        self.event_hub.emit(
            "system", "SIM_RESET", "warning",
            "仿真环境已重置",
            "已将车辆位姿重置至初始原点并重置仿真状态",
            {"origin": orig}
        )



    def _on_recorder_timer(self):
        with self.lock:
            t = {
                "x": self.telemetry["x"],
                "y": self.telemetry["y"],
                "yaw": self.telemetry["yaw"],
                "vx": self.telemetry["vx"],
                "vy": self.telemetry.get("vy", 0.0),
                "wz": self.telemetry["wz"],
                "nav_status": self.telemetry["nav_status"],
                "nav_dist_rem": self.telemetry.get("nav_dist_rem", 0.0),
                "io_states": self.telemetry.get("io_states", {}),
                "scan_ranges": self.telemetry.get("scan_ranges", []),
                "scan_min_dist": self.telemetry.get("scan_min_dist", 12.0),
                "vision_markers": self.telemetry.get("vision_markers", []),
                "dynamic_obstacles": self.telemetry.get("dynamic_obstacles", []),
                "obstacles": self.telemetry.get("dynamic_obstacles", [])
            }
        perf_summary = None
        if perf_monitor:
            h = perf_monitor.latest_stats.get("host", {})
            r = perf_monitor.latest_stats.get("ros_simulation_total", {})
            perf_summary = {
                "host_cpu": h.get("cpu_total_percent", 0.0),
                "ros_cpu": r.get("combined_cpu_percent", 0.0),
                "ram_mb": h.get("memory_used_mb", 0.0),
                "temp_c": h.get("cpu_temp_c", 0.0)
            }
        flight_recorder.record_frame(t, perf_summary)



    def _recorder_loop(self):
        while True:
            time.sleep(0.1)
            try:
                self._on_recorder_timer()
            except Exception:
                pass


bridge_node = None
v2 = None

class TeleopHTTPHandler(SimpleHTTPRequestHandler):
    protocol_version = "HTTP/1.0"

    # ---- 透明 REST 代理: 浏览器经网关直接访问两进程 v1 接口 (模型补全页/相机预览)
    #      /sim/api/v1/...  → 仿真进程    /nav/api/v1/... → 执行进程
    def _proxy(self, method) -> bool:
        path = self.path
        if path.startswith("/sim/api/v1"):
            client, rest = bridge_node.sim if bridge_node else None, path[len("/sim"):]
        elif path.startswith("/nav/api/v1"):
            client, rest = bridge_node.nav if bridge_node else None, path[len("/nav"):]
        else:
            return False
        if client is None:
            self.send_response(503); self.end_headers(); return True
        n = int(self.headers.get("Content-Length") or 0)
        body = json.loads(self.rfile.read(n).decode("utf-8")) if n > 0 else None
        try:
            status, headers, data = client.request(method, rest, body, timeout=30.0, accept=self.headers.get("Accept", "application/json"))
        except Exception as e:
            status, headers, data = 502, {"content-type": "application/json"}, json.dumps({"error": {"code": "bad_gateway", "message": str(e)}}).encode()
        self.send_response(status)
        for k in ("content-type", "x-seq", "x-stamp", "x-meta"):
            if k in headers:
                self.send_header(k, headers[k])
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-cache, no-store")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Expose-Headers", "*")
        self.end_headers()
        self.wfile.write(data)
        return True

    def _v2(self, method) -> bool:
        if urllib.parse.urlparse(self.path).path.startswith("/api/v2/") and v2 is not None:
            v2.handle(self, method)
            return True
        if method != "GET" and v2 is not None and not self.path.startswith(("/api/events", "/api/replay")):
            if v2.guard_legacy(self):
                return True
        return False

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET,POST,PUT,PATCH,DELETE,OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type, X-Lock-Token, X-User")
        self.send_header("Content-Length", "0")
        self.end_headers()

    def do_PUT(self):
        if bridge_node is not None:
            bridge_node.last_client = time.time()
        if self._v2("PUT"):
            return
        if not self._proxy("PUT"):
            self.send_response(404); self.end_headers()

    def do_DELETE(self):
        if bridge_node is not None:
            bridge_node.last_client = time.time()
        if self._v2("DELETE"):
            return
        if not self._proxy("DELETE"):
            self.send_response(404); self.end_headers()

    def do_GET(self):
        if bridge_node is not None:
            bridge_node.last_client = time.time()
        parsed = urllib.parse.urlparse(self.path)
        if self._v2("GET"):
            return
        if self._proxy("GET"):
            return
        if parsed.path in ("/model", "/model_editor.html"):
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
            self.end_headers()
            with open(os.path.join(os.path.dirname(__file__), "model_editor.html"), "rb") as f:
                self.wfile.write(f.read())
            return
        if parsed.path in ["/", "/index.html"]:
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
            self.end_headers()
            html_path = os.path.join(os.path.dirname(__file__), "index.html")
            with open(html_path, "rb") as f:
                self.wfile.write(f.read())
        elif parsed.path.startswith("/vendor/"):
            rel_path = parsed.path.lstrip("/")
            file_path = os.path.join(os.path.dirname(__file__), rel_path)
            if os.path.isfile(file_path):
                self.send_response(200)
                if file_path.endswith(".css"):
                    self.send_header("Content-Type", "text/css; charset=utf-8")
                elif file_path.endswith(".js"):
                    self.send_header("Content-Type", "application/javascript; charset=utf-8")
                else:
                    self.send_header("Content-Type", "application/octet-stream")
                self.send_header("Cache-Control", "public, max-age=86400")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                with open(file_path, "rb") as f:
                    self.wfile.write(f.read())
            else:
                self.send_response(404)
                self.end_headers()
        elif parsed.path == "/annotator.js":
            file_path = os.path.join(os.path.dirname(__file__), "annotator.js")
            if os.path.isfile(file_path):
                self.send_response(200)
                self.send_header("Content-Type", "application/javascript; charset=utf-8")
                self.send_header("Cache-Control", "no-cache, no-store")
                self.end_headers()
                with open(file_path, "rb") as f:
                    self.wfile.write(f.read())
            else:
                self.send_response(404)
                self.end_headers()
        elif parsed.path == "/api/telemetry":
            query = urllib.parse.parse_qs(parsed.query)
            want_full = query.get("full", ["0"])[0] in ("1", "true")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Cache-Control", "no-cache, no-store")
            self.end_headers()
            if bridge_node:
                with bridge_node.lock:
                    if perf_monitor:
                        h = perf_monitor.latest_stats.get("host", {})
                        r = perf_monitor.latest_stats.get("ros_simulation_total", {})
                        bridge_node.telemetry["perf_summary"] = {
                            "cpu_tot": h.get("cpu_total_percent", 0.0),
                            "mem_pct": h.get("memory_percent", 0.0),
                            "temp_c": h.get("cpu_temp_c", 0.0),
                            "ros_cpu": r.get("combined_cpu_percent", 0.0),
                            "ros_mb": r.get("combined_rss_mb", 0.0)
                        }
                    want_scans = query.get("scans", ["0"])[0] in ("1", "true")
                    if want_scans:
                        bridge_node.want_scans_until = time.time() + 2.0
                    if want_full:
                        data = json.dumps({k: v for k, v in bridge_node.telemetry.items() if want_scans or k != "lidar_scans"})
                    else:
                        fast_telemetry = {k: v for k, v in bridge_node.telemetry.items()
                                          if k not in ("scenario_metadata", "topo_graph", "config") and (want_scans or k != "lidar_scans")}
                        data = json.dumps(fast_telemetry)
                self.wfile.write(data.encode("utf-8"))
            else:
                self.wfile.write(b"{}")
        elif parsed.path == "/api/events":
            query = urllib.parse.parse_qs(parsed.query)
            since_id = int(query.get("since_id", [0])[0])
            cats_str = query.get("categories", [""])[0]
            categories = cats_str.split(",") if cats_str else None
            limit = int(query.get("limit", [100])[0])

            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Cache-Control", "no-cache, no-store")
            self.end_headers()
            data = event_hub.get_events(since_id=since_id, categories=categories, limit=limit)
            self.wfile.write(json.dumps(data).encode("utf-8"))
        elif parsed.path == "/api/system_perf":
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Cache-Control", "no-cache, no-store")
            self.end_headers()
            data = perf_monitor.get_stats()
            self.wfile.write(json.dumps(data).encode("utf-8"))
        elif parsed.path == "/api/bullet_metrics":
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Cache-Control", "no-cache, no-store")
            self.end_headers()
            with bridge_node.lock:
                bm = bridge_node.telemetry.get("bullet_metrics") or (perf_monitor.latest_stats.get("bullet_simulation") if perf_monitor else {})
            self.wfile.write(json.dumps(bm or {}).encode("utf-8"))
        elif parsed.path == "/api/lidar_config":
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Cache-Control", "no-cache, no-store")
            self.end_headers()
            cfg = bridge_node.lidar_config if bridge_node else {}
            self.wfile.write(json.dumps(cfg).encode("utf-8"))
        elif parsed.path == "/api/replay/sessions":
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Cache-Control", "no-cache, no-store")
            self.end_headers()
            data = {
                "sessions": flight_recorder.get_session_list(),
                "is_recording": flight_recorder.is_active_recording()
            }
            self.wfile.write(json.dumps(data).encode("utf-8"))
        elif parsed.path == "/api/replay/session":
            query = urllib.parse.parse_qs(parsed.query)
            sid = query.get("id", [""])[0]
            session = flight_recorder.get_session(sid)
            self.send_response(200 if session else 404)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Cache-Control", "no-cache, no-store")
            self.end_headers()
            self.wfile.write(json.dumps(session or {"error": "session not found"}).encode("utf-8"))
        elif parsed.path == "/api/replay/export":
            query = urllib.parse.parse_qs(parsed.query)
            sid = query.get("id", [""])[0]
            session = flight_recorder.get_session(sid)
            if session:
                self.send_response(200)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Disposition", f'attachment; filename="amr_flight_log_{session["id"]}.json"')
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                payload = {
                    "version": "AMR_FLIGHT_RECORDER_V4",
                    "export_time": time.time(),
                    "session": session
                }
                self.wfile.write(json.dumps(payload, indent=2).encode("utf-8"))
            else:
                self.send_response(404)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                self.wfile.write(b'{"error": "Session not found"}')
        else:
            super().do_GET()

    def do_POST(self):
        if bridge_node is not None:
            bridge_node.last_client = time.time()
        if self._v2("POST"):
            return
        if self._proxy("POST"):
            return
        parsed = urllib.parse.urlparse(self.path)
        length = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(length).decode("utf-8") if length > 0 else ""
        res = {"status": "ok"}

        try:
            req = json.loads(body) if body else {}
        except Exception:
            req = {}

        if parsed.path == "/api/cmd_vel":
            vx = float(req.get("vx", 0.0))
            vy = float(req.get("vy", 0.0))
            wz = float(req.get("wz", 0.0))
            if bridge_node:
                bridge_node.publish_cmd_vel(vx, vy, wz)
        elif parsed.path in ("/api/navigate_to_pose", "/api/navigate"):
            x = float(req.get("x", 0.0))
            y = float(req.get("y", 0.0))
            yaw = float(req.get("yaw", 0.0))
            if bridge_node:
                bridge_node.send_nav_goal(x, y, yaw)
        elif parsed.path == "/api/map_scenario":
            scenario_id = str(req.get("scenario", "grid_9_square"))
            if bridge_node:
                bridge_node.set_map_scenario(scenario_id)
        elif parsed.path == "/api/cancel_navigation":
            if bridge_node:
                bridge_node.cancel_nav()
        elif parsed.path == "/api/sim_pause":
            paused = bool(req.get("paused", True))
            if bridge_node:
                bridge_node.set_simulation_pause(paused)
            res = {"status": "ok", "paused": paused}
        elif parsed.path == "/api/sim_resume":
            if bridge_node:
                bridge_node.set_simulation_pause(False)
            res = {"status": "ok", "paused": False}
        elif parsed.path == "/api/sim_reset":
            if bridge_node:
                bridge_node.reset_simulation()
            res = {"status": "ok", "reset": True}
        elif parsed.path == "/api/chassis_type":
            ctype = str(req.get("type", "diff_drive"))
            if bridge_node:
                bridge_node.set_chassis_type(ctype)
        elif parsed.path == "/api/planner_type":
            ptype = str(req.get("type", "dijkstra"))
            if bridge_node:
                bridge_node.set_planner_type(ptype)
        elif parsed.path == "/api/lidar_config":
            beams = req.get("beams")
            angle_res = req.get("angle_resolution_deg") or req.get("resolution")
            freq = req.get("freq_hz") or req.get("freq") or req.get("rate")
            range_max = req.get("range_max")
            if bridge_node:
                cfg = bridge_node.set_lidar_config(
                    beams=int(beams) if beams is not None else None,
                    angle_resolution_deg=float(angle_res) if angle_res is not None else None,
                    freq_hz=float(freq) if freq is not None else None,
                    range_max=float(range_max) if range_max is not None else None
                )
                res = {"status": "ok", "config": cfg}
            else:
                res = {"status": "error", "message": "bridge_node not initialized"}
        elif parsed.path == "/api/set_io":
            key = str(req.get("key", ""))
            val = bool(req.get("value", False))
            if bridge_node:
                bridge_node.set_io(key, val)
        elif parsed.path == "/api/obstacles/random":
            count = int(req.get("count", 3))
            if bridge_node:
                bridge_node.generate_random_obstacles(count)
        elif parsed.path == "/api/obstacles/clear":
            if bridge_node:
                bridge_node.clear_obstacles()
        elif parsed.path == "/api/obstacles/add":
            x = float(req.get("x", 0.0))
            y = float(req.get("y", 0.0))
            w = float(req.get("w", 0.8))
            h = float(req.get("h", 0.8))
            obs_type = str(req.get("type", "box"))
            if bridge_node:
                ok, msg = bridge_node.add_obstacle(x, y, w, h, obs_type)
                if not ok:
                    self.send_response(400)
                    self.send_header("Content-Type", "application/json; charset=utf-8")
                    self.send_header("Access-Control-Allow-Origin", "*")
                    self.end_headers()
                    self.wfile.write(json.dumps({"status": "error", "message": msg}).encode("utf-8"))
                    return
                res = {"status": "ok", "message": "扰动模块已添加，将在恢复仿真时生效"}
            else:
                res = {"status": "error", "message": "bridge_node not initialized"}
        elif parsed.path == "/api/events/clear":
            res = event_hub.clear()
        elif parsed.path == "/api/events/inject":
            category = str(req.get("category", "system"))
            event_type = str(req.get("type", "MANUAL_TEST"))
            level = str(req.get("level", "info"))
            title = str(req.get("title", "测试注入事件"))
            message = str(req.get("message", "用户从控制台手动注入测试事件"))
            payload = req.get("payload", {})
            ev = event_hub.emit(category, event_type, level, title, message, payload)
            res = {"status": "ok", "event": ev}
        elif parsed.path == "/api/replay/record":
            action = str(req.get("action", "start"))
            if action == "start":
                sid = flight_recorder.start_session("manual", metadata={
                    "title": req.get("title", f"手动录制会话 {time.strftime('%H:%M:%S')}"),
                    "scenario": bridge_node.active_scenario if bridge_node else "default"
                })
                res = {"status": "ok", "session_id": sid, "action": "started"}
            elif action == "stop":
                s = flight_recorder.end_session("MANUAL_STOPPED")
                res = {"status": "ok", "session": s, "action": "stopped"}
            elif action == "capture_rolling":
                secs = float(req.get("seconds", 30.0))
                s = flight_recorder.capture_rolling_as_session(secs, title=req.get("title"))
                res = {"status": "ok", "session": s, "action": "captured"}

        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(json.dumps(res).encode("utf-8"))


def run_server(port=8088):
    # WEB_BIND=127.0.0.1: 只给同机的资源平台反向代理用 (平台 /inst/<实例>/)，不在局域网单独开放端口
    server_address = (os.environ.get("WEB_BIND", "0.0.0.0"), port)
    httpd = ThreadingHTTPServer(server_address, TeleopHTTPHandler)
    httpd.serve_forever()


def main():
    global bridge_node, v2
    port = int(os.environ.get("WEB_PORT", "8088"))
    bridge_node = Gateway()
    bridge_node.start()
    from gateway_v2 import GatewayV2
    v2 = GatewayV2(bridge_node, SIM_API, NAV_API)
    print(f"[web_gateway] http://{os.environ.get('WEB_BIND', '0.0.0.0')}:{port}  ←→ 仿真进程 {SIM_API} / 执行进程 {NAV_API}", flush=True)
    try:
        run_server(port)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
