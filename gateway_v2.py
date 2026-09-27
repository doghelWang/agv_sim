#!/usr/bin/env python3
"""
Web 网关 v2 接口 —— 设备端仿真工作台 (资源平台 /inst/<id>/ 代理到这里)

  控制权锁   POST/PUT/DELETE /api/v2/lock      持锁后所有写操作需带 X-Lock-Token (无人持锁时不限制，兼容旧调度台)
  实例信息   GET /api/v2/info   GET /api/v2/brief (平台监控用的轻量状态)
  事件流     GET /api/v2/events?after=N         带标签 TSK/NAV/OBS/INJ/RST/DRV/SAF/SYS
  运行控制   POST /api/v2/pause {paused}   POST /api/v2/reset
  任务流     POST /api/v2/taskflow/run {flow}   POST /api/v2/taskflow/stop
  扰动注入   GET /api/v2/inject/catalog   POST /api/v2/inject {type, placement, x, y, motion}   DELETE /api/v2/inject/{id}
  仿真记录   GET /api/v2/records   GET /api/v2/records/{id}/bundle   GET /api/v2/records/bundle_all   POST /api/v2/recording {on}
             POST /api/v2/records/archive (实例终止前由平台调用，归档到平台)   GET /api/v2/logs (RCD/EVT 打包)
  参数       GET/PUT /api/v2/safety     PUT /api/v2/planner  PUT /api/v2/chassis  POST /api/v2/scene  POST /api/v2/model
"""

import io
import json
import math
import os
import random
import secrets
import threading
import time
import urllib.request
import zipfile
from typing import Optional

from common.rest import ApiError, BinaryBody, RawBody, RestServer

LOCK_TTL = 30.0

INJECT_CATALOG = [
    {"type": "pallet", "name": "标准木质栈板", "icon": "🪵", "w": 1.2, "h": 1.0, "z": 0.15, "desc": "1.2 × 1.0 × 0.15m"},
    {"type": "shelf", "name": "双层轻型货架", "icon": "🏗️", "w": 2.0, "h": 1.0, "z": 2.2, "desc": "2.0 × 1.0 × 2.2m"},
    {"type": "box", "name": "工业周转纸箱", "icon": "📦", "w": 0.8, "h": 0.8, "z": 0.8, "desc": "0.8 × 0.8 × 0.8m"},
    {"type": "person", "name": "车间作业人员", "icon": "👷", "w": 0.5, "h": 0.5, "z": 1.7, "desc": "反光工装 · 可行走"},
]
PLACEMENTS = [
    {"id": "ahead_2.5", "name": "车身正前方 2.5 米 (规划路径线上)"},
    {"id": "ahead_1.2", "name": "车身正前方 1.2 米 (近距减速区)"},
    {"id": "next_station", "name": "下一工位站点附近"},
    {"id": "random", "name": "随机拓扑节点附近"},
    {"id": "custom", "name": "自定义场景坐标 (画布点选/手动输入)"},
]

TAG_BY_TYPE = {
    "TASKFLOW": "TSK", "MISSION": "NAV", "PLAN": "NAV", "NAV2": "NAV", "PLANNER": "NAV", "REPLAN": "NAV", "ARRIVED": "NAV",
    "OBS_": "OBS", "OBSTACLE_WAIT": "OBS", "INJECT": "INJ", "OBSTACLE_CHANGE": "INJ", "SIM_RESET": "RST", "RESET": "RST",
    "CHASSIS": "DRV", "WHEEL": "DRV", "SLIP": "DRV", "ESTOP": "SAF", "BUMPER": "SAF", "COLLISION": "SAF", "SAFETY": "SAF",
    "PHOTO": "SAF", "DI_": "SAF",
}
TAG_BY_CAT = {"task": "TSK", "navigation": "NAV", "safety": "SAF", "sensors": "OBS", "chassis": "DRV", "system": "SYS"}


def tag_of(ev: dict) -> str:
    t = (ev.get("type") or "").upper()
    for k, v in TAG_BY_TYPE.items():
        if t.startswith(k):
            return v
    return TAG_BY_CAT.get(ev.get("category"), "SYS")


class LockManager:
    def __init__(self):
        self.lock = threading.Lock()
        self.holder = None      # {user, token, since, renewed}

    def _expired(self):
        return self.holder is None or time.time() - self.holder["renewed"] > LOCK_TTL

    def status(self, token: str = "") -> dict:
        with self.lock:
            if self._expired():
                self.holder = None
                return {"held": False}
            h = self.holder
            return {"held": True, "user": h["user"], "since": h["since"], "mine": bool(token) and secrets.compare_digest(token, h["token"]),
                    "ttl": round(LOCK_TTL - (time.time() - h["renewed"]), 1)}

    def acquire(self, user: str, force: bool = False) -> dict:
        with self.lock:
            if not self._expired() and not force and self.holder["user"] != user:
                raise ApiError(409, f"控制权已被 {self.holder['user']} 占用", "locked")
            self.holder = {"user": user or "未命名用户", "token": secrets.token_urlsafe(16), "since": time.time(), "renewed": time.time()}
            return {"held": True, "user": self.holder["user"], "token": self.holder["token"], "mine": True, "ttl": LOCK_TTL}

    def renew(self, token: str) -> dict:
        with self.lock:
            if self._expired() or not secrets.compare_digest(token or "", self.holder["token"]):
                raise ApiError(409, "控制权已失效，请重新获取", "lock_lost")
            self.holder["renewed"] = time.time()
            return {"held": True, "user": self.holder["user"], "mine": True, "ttl": LOCK_TTL}

    def release(self, token: str) -> dict:
        with self.lock:
            if self.holder and secrets.compare_digest(token or "", self.holder["token"]):
                self.holder = None
            return {"held": False}

    def allowed(self, token: str) -> Optional[str]:
        """None=允许写；否则返回占用者"""
        with self.lock:
            if self._expired():
                return None
            if token and secrets.compare_digest(token, self.holder["token"]):
                return None
            return self.holder["user"]


class TaskRecorder:
    """按任务流切分仿真记录 → sim_bundle (需求数据契约)，10 Hz 轨迹 + 全量事件 + 注入元素"""

    def __init__(self, v2):
        self.v2 = v2
        self.lock = threading.Lock()
        self.on = True
        self.active = None
        self.records = []          # 已完成 (新→旧)
        self.dir = os.path.join(os.environ.get("AGV_DATA", os.path.dirname(os.path.abspath(__file__))), "records")
        os.makedirs(self.dir, exist_ok=True)
        self._load()
        threading.Thread(target=self._loop, daemon=True, name="task-recorder").start()

    def _load(self):
        try:
            fs = sorted((f for f in os.listdir(self.dir) if f.endswith(".json")), reverse=True)[:50]
            for f in fs:
                with open(os.path.join(self.dir, f), encoding="utf-8") as fh:
                    b = json.load(fh)
                self.records.append(b)
            self.records.sort(key=lambda b: b["metadata"].get("recordedAt", ""), reverse=True)
        except Exception as e:
            print(f"[gateway-v2] 读取历史记录失败: {e}", flush=True)

    def start(self, flow: dict):
        with self.lock:
            if self.active:
                self._finish("SUPERSEDED")
            if not self.on:
                return
            gw = self.v2.gw
            info = self.v2.info()
            now = time.time()
            rid = f"{time.strftime('%Y%m%d%H%M%S', time.localtime(now))}{random.randint(10, 99)}"
            self.active = {"id": rid, "t0": now, "flow": flow, "ev_from": self.v2.gw.event_hub.event_counter,
                           "traj": [], "inj": [], "info": info, "scene": dict(gw.telemetry.get("scenario_metadata") or {}),
                           "topo": gw.telemetry.get("topo_graph") or {}, "spec": gw.telemetry.get("robot_spec") or {}}

    def injected(self, el: dict):
        with self.lock:
            if self.active:
                self.active["inj"].append({"type": el["type"], "name": el.get("name"), "x": el["x"], "y": el["y"],
                                           "spawnTime": round(time.time() - self.active["t0"], 2), "size": [el.get("w"), el.get("h"), el.get("z")],
                                           "motion": el.get("motion"), "id": el.get("id")})

    def removed(self, oid):
        with self.lock:
            if self.active:
                for e in self.active["inj"]:
                    if e.get("id") == oid and "removedAt" not in e:
                        e["removedAt"] = round(time.time() - self.active["t0"], 2)

    def end(self, result: str):
        with self.lock:
            return self._finish(result)

    def _loop(self):
        while True:
            time.sleep(0.1)
            try:
                self._sample()
            except Exception:
                pass

    def _sample(self):
        gw = self.v2.gw
        with gw.lock:
            T = gw.telemetry
            safety = T.get("safety") or {}
            obs = (safety.get("obstacle") or {}).get("front")
            fr = {"time": 0.0, "x": round(T.get("x", 0.0), 3), "y": round(T.get("y", 0.0), 3), "yaw": round(T.get("yaw", 0.0), 4),
                  "vx": round(T.get("vx", 0.0), 3), "vy": round(T.get("vy", 0.0), 3), "w": round(T.get("wz", 0.0), 3),
                  "obsDist": round(obs, 2) if obs is not None else T.get("scan_min_dist"),
                  "status": T.get("nav_status", "IDLE"), "safety": (safety.get("obstacle") or {}).get("zone"),
                  "locErrMm": (T.get("localization") or {}).get("err_mm")}
            tf = T.get("taskflow") or {}
        fr["step"] = tf.get("step_index")
        with self.lock:
            a = self.active
            if not a:
                return
            fr["time"] = round(time.time() - a["t0"], 2)
            a["traj"].append(fr)
            st = tf.get("status")
            if st in ("done", "failed", "stopped") and time.time() - a["t0"] > 1.5 and tf.get("started", 0) >= a["t0"] - 1:
                self._finish({"done": "SUCCESS", "failed": "FAILED", "stopped": "STOPPED"}[st])

    def _finish(self, result: str):
        a = self.active
        if not a:
            return None
        self.active = None
        dur = round(time.time() - a["t0"], 2)
        evs = [e for e in self.v2.gw.event_hub.get_events(since_id=a["ev_from"], limit=2000)["events"]]
        events = [{"time": round(e["timestamp"] - a["t0"], 2), "tag": tag_of(e), "title": e.get("title"), "detail": e.get("message"),
                   "source": e.get("category"), "level": e.get("level")} for e in evs]
        avoid = len([e for e in events if e["tag"] == "OBS"])
        collided = any(e["tag"] == "SAF" and ("碰撞" in (e["title"] or "") or "触边" in (e["title"] or "")) for e in events)
        if result == "SUCCESS" and a["inj"] and avoid:
            result = "AVOIDED"
        if result in ("SUCCESS", "AVOIDED") and collided:
            result = "COLLIDED"
        flow, info, sc, spec = a["flow"], a["info"], a["scene"], a["spec"]
        b = sc.get("bounds") or {}
        ch = spec.get("chassis") or {}
        wheels = spec.get("wheels") or []
        xs = [w.get("x", 0) for w in wheels]
        topo = a["topo"] or {}
        bundle = {
            "schema": "sim_bundle/1",
            "metadata": {"taskId": f"TID-{flow.get('tid') or a['id']}", "taskName": flow.get("name"), "recordId": a["id"],
                         "recordedAt": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(a["t0"])), "durationSeconds": dur,
                         "result": result, "instanceId": info.get("instance_id"), "instanceName": info.get("instance_name"),
                         "operator": (self.v2.locks.status().get("user")), "simApi": info.get("sim_api"), "navApi": info.get("nav_api"),
                         "avoidCount": avoid, "collided": collided},
            "taskFlow": {"id": flow.get("id"), "name": flow.get("name"), "tid": flow.get("tid"), "loop": flow.get("loop", "single"),
                         "steps": flow.get("steps", [])},
            "environment": {"sceneName": sc.get("name"), "sceneId": sc.get("id"),
                            "size": [round(b.get("max_x", 0) - b.get("min_x", 0), 2), round(b.get("max_y", 0) - b.get("min_y", 0), 2)],
                            "bounds": b, "slamResolution": 0.05,
                            "topologyNodes": [{"id": k, "x": v.get("x"), "y": v.get("y")} for k, v in (topo.get("nodes") or {}).items()],
                            "topologyEdges": [[e.get("from"), e.get("to")] for e in topo.get("edges") or []],
                            "stations": [{"id": s.get("id"), "name": s.get("name"), "x": s.get("x"), "y": s.get("y")} for s in sc.get("stations", [])],
                            "walls": sc.get("walls", []), "shelves": sc.get("shelves", [])},
            "vehicleModel": {"modelName": (info.get("model") or {}).get("name") or spec.get("model_file"),
                             "materialNo": (info.get("model") or {}).get("material_no"), "type": ch.get("type"),
                             "wheelbase": round(max(xs) - min(xs), 3) if len(xs) > 1 else None, "modelId": (info.get("model") or {}).get("model_id"),
                             "version": (info.get("model") or {}).get("version"), "chassis": spec.get("active_chassis"),
                             "footprint": ch.get("footprint") or spec.get("footprint"), "length": ch.get("length_m"), "width": ch.get("width_m")},
            "injectedElements": a["inj"],
            "events": events,
            "trajectory": a["traj"],
        }
        self.records.insert(0, bundle)
        self.records = self.records[:50]
        try:
            with open(os.path.join(self.dir, f"{a['id']}.json"), "w", encoding="utf-8") as f:
                json.dump(bundle, f, ensure_ascii=False)
            fs = sorted(os.listdir(self.dir))
            for old in fs[:-50]:
                os.remove(os.path.join(self.dir, old))
        except Exception as e:
            print(f"[gateway-v2] 保存记录失败: {e}", flush=True)
        self.v2.gw.event_hub.emit("task", "RECORD_SAVED", "info", f"仿真记录已保存 {bundle['metadata']['taskId']}",
                                  f"时长 {dur}s，结果 {result}，轨迹 {len(a['traj'])} 帧", {"record": a["id"]})
        threading.Thread(target=self.v2.archive, args=(bundle,), daemon=True).start()
        return bundle

    def summary(self, b: dict) -> dict:
        m = b["metadata"]
        return {"id": m.get("recordId"), "taskId": m.get("taskId"), "taskName": m.get("taskName"), "recordedAt": m.get("recordedAt"),
                "durationSeconds": m.get("durationSeconds"), "result": m.get("result"), "scene": b["environment"].get("sceneName"),
                "model": b["vehicleModel"].get("modelName"), "injections": len(b.get("injectedElements") or []),
                "avoidCount": m.get("avoidCount", 0), "archived": m.get("archived", False)}

    def stats(self) -> dict:
        rs = self.records
        n = len(rs)
        ok = [r for r in rs if r["metadata"].get("result") in ("SUCCESS", "AVOIDED")]
        return {"count": n, "success_rate": round(100.0 * len(ok) / n, 1) if n else None,
                "avoid_count": sum(r["metadata"].get("avoidCount", 0) for r in rs),
                "avg_duration": round(sum(r["metadata"].get("durationSeconds", 0) for r in ok) / len(ok), 1) if ok else None,
                "recording": self.on, "active": {"id": self.active["id"], "elapsed": round(time.time() - self.active["t0"], 1),
                                                 "name": self.active["flow"].get("name")} if self.active else None}


class GatewayV2:
    def __init__(self, gw, sim_api: str, nav_api: str):
        self.gw, self.sim_api, self.nav_api = gw, sim_api, nav_api
        self.locks = LockManager()
        self.rec = TaskRecorder(self)
        self._info = {}
        self._info_t = 0
        self.api = self._build()

    # ------------------------------------------------------------------ 信息
    def info(self) -> dict:
        if time.time() - self._info_t > 5 or not self._info:
            inst = self.gw.sim.safe("GET", "/api/v1/instance") or {}
            self._info = {"instance_id": inst.get("instance_id") or os.environ.get("INSTANCE_ID"),
                          "instance_name": inst.get("instance_name") or os.environ.get("INSTANCE_NAME"),
                          "sim_api": self.sim_api, "nav_api": self.nav_api, "hub": inst.get("hub") or os.environ.get("HUB_API"),
                          "model": inst.get("model") or {}, "scene": inst.get("scene") or {}, "engine": (self.gw.telemetry.get("sim_status") or {}).get("backend")}
            self._info_t = time.time()
        d = dict(self._info)
        d.update({"sim_online": self.gw.sim_online, "nav_online": self.gw.nav_online})
        return d

    def brief(self) -> dict:
        with self.gw.lock:
            T = self.gw.telemetry
            tf = T.get("taskflow") or {}
            d = {"sim_online": self.gw.sim_online, "nav_online": self.gw.nav_online, "nav_status": T.get("nav_status"),
                 "paused": T.get("is_paused"), "pose": {"x": round(T.get("x", 0), 2), "y": round(T.get("y", 0), 2), "yaw": round(T.get("yaw", 0), 3)},
                 "planner": T.get("planner_type"), "nav2_ready": (T.get("nav2") or {}).get("server_ready")}
        steps = tf.get("steps") or []
        i = tf.get("step_index")
        d["taskflow"] = {"status": tf.get("status"), "name": (tf.get("flow") or {}).get("name"), "tid": (tf.get("flow") or {}).get("tid"),
                         "step_index": i, "step": steps[i] if isinstance(i, int) and i < len(steps) else None, "total": len(steps)}
        d["lock"] = self.locks.status()
        d["records"] = len(self.rec.records)
        return d

    def archive(self, bundle: dict) -> bool:
        hub = self.info().get("hub")
        if not hub:
            return False
        try:
            req = urllib.request.Request(hub.rstrip("/") + "/api/hub/records", data=json.dumps(bundle, ensure_ascii=False).encode("utf-8"),
                                         headers={"Content-Type": "application/json"}, method="POST")
            urllib.request.urlopen(req, timeout=15).read()
            bundle["metadata"]["archived"] = True
            return True
        except Exception as e:
            print(f"[gateway-v2] 记录归档到平台失败: {e}", flush=True)
            return False

    # ------------------------------------------------------------------ 注入
    def _path_point(self, dist: float):
        """沿当前规划路径 (无路径则沿车头方向) 前进 dist 米的位置与方向"""
        with self.gw.lock:
            T = self.gw.telemetry
            x, y, yaw = T.get("x", 0.0), T.get("y", 0.0), T.get("yaw", 0.0)
            path = [(p["x"], p["y"]) for p in (T.get("plan_path") or []) if isinstance(p, dict)]
            head = ((T.get("robot_spec") or {}).get("chassis") or {}).get("head_offset_m") or 0.5
            pidx = int(T.get("path_index") or 0)
        dist += head
        if len(path) >= 2:
            if 1 <= pidx < len(path):
                k = pidx - 1          # 车辆正驶向 path[pidx]
            else:
                k = min(range(len(path)), key=lambda i: math.hypot(path[i][0] - x, path[i][1] - y))
            pts = [(x, y)] + path[k + 1:]
            left = dist
            for (ax, ay), (bx, by) in zip(pts, pts[1:]):
                L = math.hypot(bx - ax, by - ay)
                if L >= left and L > 1e-6:
                    return ax + (bx - ax) * left / L, ay + (by - ay) * left / L, math.atan2(by - ay, bx - ax)
                left -= L
            if len(pts) >= 2:
                (ax, ay), (bx, by) = pts[-2], pts[-1]
                return bx, by, math.atan2(by - ay, bx - ax)
        return x + dist * math.cos(yaw), y + dist * math.sin(yaw), yaw

    def inject(self, b: dict) -> dict:
        cat = next((c for c in INJECT_CATALOG if c["type"] == b.get("type")), None)
        if not cat:
            raise ApiError(400, f"未知元素类型 {b.get('type')}")
        if not self.gw.is_paused and not b.get("force"):
            raise ApiError(409, "请先暂停仿真，再从注入库放置扰动元素", "not_paused")
        pl = b.get("placement", "ahead_2.5")
        yaw = 0.0
        if pl in ("ahead_2.5", "ahead_1.2"):
            x, y, yaw = self._path_point(2.5 if pl == "ahead_2.5" else 1.2)
        elif pl == "next_station":
            tf = self.gw.telemetry.get("taskflow") or {}
            goal = self.gw.telemetry.get("target_goal")
            st = None
            steps = tf.get("steps") or []
            i = tf.get("step_index")
            if isinstance(i, int):
                for s in steps[i:]:
                    if s.get("type") == "move":
                        st = next((q for q in (self.gw.telemetry.get("scenario_metadata") or {}).get("stations", []) if q.get("id") == s.get("target")), None)
                        if st:
                            break
            tx, ty = (st["x"], st["y"]) if st else ((goal["x"], goal["y"]) if goal else self._path_point(3.0)[:2])
            rx, ry = self.gw.telemetry.get("x", 0.0), self.gw.telemetry.get("y", 0.0)
            d = math.hypot(tx - rx, ty - ry) or 1.0
            off = min(1.4, d * 0.5)
            x, y, yaw = tx - (tx - rx) / d * off, ty - (ty - ry) / d * off, math.atan2(ty - ry, tx - rx)
        elif pl == "random":
            nodes = list(((self.gw.telemetry.get("topo_graph") or {}).get("nodes") or {}).values())
            rx, ry = self.gw.telemetry.get("x", 0.0), self.gw.telemetry.get("y", 0.0)
            nodes = [n for n in nodes if math.hypot(n["x"] - rx, n["y"] - ry) > 2.0] or nodes
            if not nodes:
                raise ApiError(400, "场景没有拓扑节点")
            n = random.choice(nodes)
            x, y = n["x"] + random.uniform(-0.2, 0.2), n["y"] + random.uniform(-0.2, 0.2)
        else:
            try:
                x, y = float(b["x"]), float(b["y"])
            except (KeyError, TypeError, ValueError):
                raise ApiError(400, "自定义位置需要 x, y")
            yaw = float(b.get("yaw", 0.0))
        el = {"type": cat["type"], "name": cat["name"], "x": round(x, 2), "y": round(y, 2), "yaw": round(yaw, 3), "w": cat["w"],
              "h": cat["h"], "z": cat["z"], "placement": pl}
        if cat["type"] == "person" and b.get("walk"):
            # 行走人员: 垂直于路径方向往返横穿 (±span)
            span = float(b.get("span", 1.5))
            px, py = -math.sin(yaw), math.cos(yaw)
            el["motion"] = {"type": "patrol", "ax": round(x - px * span, 2), "ay": round(y - py * span, 2),
                            "bx": round(x + px * span, 2), "by": round(y + py * span, 2), "speed": float(b.get("speed", 0.6))}
            el["x"], el["y"] = el["motion"]["ax"], el["motion"]["ay"]
        with self.gw.lock:
            ids = [o.get("id", 0) for o in self.gw.dynamic_obstacles if isinstance(o.get("id"), int)]
            el["id"] = max(ids + [0]) + 1
            el["active"] = not self.gw.is_paused
            self.gw.dynamic_obstacles.append(el)
            self.gw.telemetry["dynamic_obstacles"] = list(self.gw.dynamic_obstacles)
            self.gw.telemetry["obstacles"] = list(self.gw.dynamic_obstacles)
        self.gw.broadcast_obstacles()
        self.rec.injected(el)
        self.gw.event_hub.emit("sensors", "INJECT_ADD", "warning", f"注入扰动元素 #{el['id']} {cat['name']}",
                               f"{next(p['name'] for p in PLACEMENTS if p['id'] == pl) if pl != 'custom' else '自定义'} "
                               f"({el['x']:.2f}, {el['y']:.2f}){' · 往返行走' if el.get('motion') else ''}", el)
        return el

    def remove(self, oid: int) -> dict:
        with self.gw.lock:
            before = len(self.gw.dynamic_obstacles)
            self.gw.dynamic_obstacles = [o for o in self.gw.dynamic_obstacles if o.get("id") != oid]
            self.gw.telemetry["dynamic_obstacles"] = list(self.gw.dynamic_obstacles)
            self.gw.telemetry["obstacles"] = list(self.gw.dynamic_obstacles)
            n = before - len(self.gw.dynamic_obstacles)
        if not n:
            raise ApiError(404, f"没有注入元素 #{oid}")
        self.gw.broadcast_obstacles()
        self.rec.removed(oid)
        self.gw.event_hub.emit("sensors", "INJECT_REMOVE", "info", f"移除扰动元素 #{oid}", "", {"id": oid})
        return {"removed": oid}

    # ------------------------------------------------------------------ 运行控制
    def reset(self) -> dict:
        self.gw.nav.safe("POST", "/api/v1/taskflows/stop", {"reason": "环境重置"})
        n = len(self.gw.dynamic_obstacles)
        self.gw.reset_simulation()
        self.rec.end("RESET")
        self.gw.event_hub.emit("system", "RESET_ENV", "warning", "仿真环境已重置", f"车辆回到起点 P0，清除 {n} 个注入元素，任务流已终止", {})
        return {"reset": True}

    def run_flow(self, flow: dict) -> dict:
        if not flow.get("steps"):
            raise ApiError(400, "任务流没有工步")
        if self.gw.is_paused:
            self.gw.set_simulation_pause(False)
        try:
            r = self.gw.nav.post("/api/v1/taskflows/run", {"flow": flow}, timeout=8)
        except ApiError as e:
            raise ApiError(400, str(e.message).split(": ", 1)[-1])
        except Exception as e:
            raise ApiError(502, f"执行进程不可用: {e}")
        self.rec.start(flow)
        return r

    def stop_flow(self) -> dict:
        r = self.gw.nav.safe("POST", "/api/v1/taskflows/stop", {"reason": "操作员终止"}) or {}
        self.rec.end("STOPPED")
        return r

    def logs_zip(self) -> bytes:
        buf = io.BytesIO()
        with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as z:
            evs = self.gw.event_hub.get_events(since_id=0, limit=5000)["events"]
            z.writestr("events.evt.json", json.dumps([dict(e, tag=tag_of(e)) for e in evs], ensure_ascii=False, indent=1))
            for b in self.rec.records[:20]:
                z.writestr(f"records/sim_bundle_{b['metadata'].get('recordId')}.rcd.json", json.dumps(b, ensure_ascii=False))
            z.writestr("instance.json", json.dumps(self.info(), ensure_ascii=False, indent=1))
        return buf.getvalue()

    # ------------------------------------------------------------------ 路由
    def _build(self) -> RestServer:
        api = RestServer("gateway-v2")
        R = api.route
        V = "/api/v2"
        tok = lambda q: q.headers.get("X-Lock-Token", "")  # noqa: E731
        R("GET", V + "/info", lambda q: self.info(), "实例信息")
        R("GET", V + "/brief", lambda q: self.brief(), "轻量状态")
        R("GET", V + "/lock", lambda q: self.locks.status(tok(q)), "控制权状态")
        R("POST", V + "/lock", lambda q: self._lock_acq(q), "获取控制权 {user, force}")
        R("PUT", V + "/lock", lambda q: self.locks.renew(tok(q)), "续约")
        R("DELETE", V + "/lock", lambda q: self._lock_rel(q), "释放控制权")

        def events(q):
            d = self.gw.event_hub.get_events(since_id=q.q("after", 0, int), limit=q.q("limit", 200, int))
            evs = [dict(e, tag=tag_of(e)) for e in d["events"]]
            tag = q.q("tag")
            if tag:
                evs = [e for e in evs if e["tag"] in tag.split(",")]
            return {"events": evs, "latest_id": d["latest_id"]}
        R("GET", V + "/events", events, "带标签事件流")
        R("POST", V + "/pause", lambda q: self.gw.set_simulation_pause(bool(q.json.get("paused", True))) or {"paused": self.gw.is_paused},
          "暂停/恢复")
        R("POST", V + "/reset", lambda q: self.reset(), "重置环境")
        R("POST", V + "/taskflow/run", lambda q: self.run_flow(q.json.get("flow") or q.json), "执行任务流")
        R("POST", V + "/taskflow/stop", lambda q: self.stop_flow(), "终止任务流")
        R("GET", V + "/inject/catalog", lambda q: {"catalog": INJECT_CATALOG, "placements": PLACEMENTS}, "注入库")
        R("POST", V + "/inject", lambda q: self.inject(q.json), "注入元素")
        R("DELETE", V + "/inject/{oid}", lambda q: self.remove(int(q.params["oid"])), "移除注入元素")
        R("GET", V + "/records", lambda q: {"records": [self.rec.summary(b) for b in self.rec.records], "stats": self.rec.stats()}, "仿真记录")

        def bundle(q):
            b = next((b for b in self.rec.records if b["metadata"].get("recordId") == q.params["rid"]), None)
            if not b:
                raise ApiError(404, "记录不存在")
            return RawBody(json.dumps(b, ensure_ascii=False).encode("utf-8"), "application/json; charset=utf-8",
                           {"Content-Disposition": f"attachment; filename=sim_bundle_{b['metadata'].get('taskId')}_{q.params['rid']}.json"})
        R("GET", V + "/records/{rid}/bundle", bundle, "下载 sim_bundle")

        def bundle_all(q):
            buf = io.BytesIO()
            with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as z:
                for b in self.rec.records:
                    z.writestr(f"sim_bundle_{b['metadata'].get('taskId')}_{b['metadata'].get('recordId')}.json", json.dumps(b, ensure_ascii=False))
            return RawBody(buf.getvalue(), "application/zip", {"Content-Disposition": "attachment; filename=sim_bundles_all.zip"})
        R("GET", V + "/records/bundle_all", bundle_all, "打包下载全部记录")

        def archive_all(q):
            if self.rec.active:
                self.rec.end("STOPPED")
            n = sum(1 for b in self.rec.records if not b["metadata"].get("archived") and self.archive(b))
            return {"archived": n}
        R("POST", V + "/records/archive", archive_all, "归档全部记录到平台")

        def recording(q):
            self.rec.on = bool(q.json.get("on", True))
            if not self.rec.on:
                self.rec.end("STOPPED")
            return self.rec.stats()
        R("POST", V + "/recording", recording, "开关录制")
        R("GET", V + "/logs", lambda q: RawBody(self.logs_zip(), "application/zip", {
            "Content-Disposition": f"attachment; filename=sim_logs_{time.strftime('%Y%m%d_%H%M%S')}.zip"}), "下载仿真日志 RCD/EVT")

        R("GET", V + "/safety", lambda q: self.gw.nav.safe("GET", "/api/v1/safety/params") or {}, "避障参数")
        R("POST", V + "/plan", lambda q: self.gw.nav.safe("POST", "/api/v1/plan", q.json) or {"legs": []}, "路线预览 (执行进程规划器)")
        R("PUT", V + "/safety", lambda q: self.gw.nav.put("/api/v1/safety/params", q.json), "修改避障参数")
        # ---- 定位 (执行进程: 激光 SLAM + 里程计融合)
        R("GET", V + "/slam", lambda q: self.gw.nav.safe("GET", "/api/v1/slam") or {"online": False}, "定位状态")

        def slam_map(q):
            part = q.q("part", "grid")
            if part in ("pgm", "yaml"):
                st, h, body = self.gw.nav.binary(f"/api/v1/slam/map?part={part}", timeout=10)
                if st != 200:
                    raise ApiError(st, "地图为空或执行进程不可用")
                return RawBody(body, "image/x-portable-graymap" if part == "pgm" else "text/plain; charset=utf-8",
                               {"Content-Disposition": h.get("content-disposition", f'attachment; filename="slam.{part}"')})
            return self.gw.nav.safe("GET", f"/api/v1/slam/map?step={q.q('step', 1, int)}", timeout=5) or {"empty": True}
        R("GET", V + "/slam/map", slam_map, "SLAM 地图 ?part=grid|pgm|yaml")
        for act in ("mode", "save", "reset", "initialpose"):
            R("POST", V + "/slam/" + act, (lambda a: lambda q: self.gw.nav.post("/api/v1/slam/" + a, q.json, timeout=10))(act), "定位: " + act)
        R("PUT", V + "/planner", lambda q: self.gw.set_planner_type(q.json.get("type")) or {"planner": self.gw.active_planner}, "切换规划器")
        R("PUT", V + "/chassis", lambda q: self.gw.set_chassis_type(q.json.get("type")) or {"ok": True}, "切换车型 (热切换)")

        def scene(q):
            self.gw.nav.safe("POST", "/api/v1/taskflows/stop", {"reason": "场景切换"})
            self.gw.cancel_nav()
            with self.gw.lock:
                self.gw.dynamic_obstacles = []
            self.gw.sim.safe("PUT", "/api/v1/world/obstacles", [])
            r = self.gw.sim.post("/api/v1/scene/load", q.json, timeout=60)
            self.gw.world = {}
            self._info_t = 0
            self.gw.event_hub.emit("system", "SCENARIO_SWITCH", "info", "仿真场景热切换", f"→ {r.get('scenario')}", r)
            return r
        R("POST", V + "/scene", scene, "热切换场景 {hub_scene_id}")

        def model(q):
            r = self.gw.sim.post("/api/v1/model/load", q.json, timeout=60)
            self.gw.model = {}
            self._info_t = 0
            self.gw.event_hub.emit("chassis", "CHASSIS_SWITCH", "info", "车辆模型热切换", f"→ {q.json.get('model_id')}", {})
            return {"ok": True, "model_rev": r.get("model_rev")}
        R("POST", V + "/model", model, "热切换车辆模型 {model_id, version}")
        return api

    def _lock_acq(self, q):
        r = self.locks.acquire(q.json.get("user", ""), bool(q.json.get("force")))
        self.gw.event_hub.emit("system", "LOCK", "info", f"{r['user']} 获取控制权", "", {})
        return r

    def _lock_rel(self, q):
        st = self.locks.status(q.headers.get("X-Lock-Token", ""))
        r = self.locks.release(q.headers.get("X-Lock-Token", ""))
        if st.get("mine"):
            self.gw.event_hub.emit("system", "LOCK", "info", f"{st.get('user')} 释放控制权", "", {})
        return r

    # ------------------------------------------------------------------ HTTP 接入 (web_gateway 的 Handler 调用)
    def handle(self, h, method: str):
        import urllib.parse
        path = urllib.parse.urlparse(h.path).path
        n = int(h.headers.get("Content-Length") or 0)
        body = h.rfile.read(n) if n else b""
        if method != "GET" and not path.startswith("/api/v2/lock") and path != "/api/v2/plan":   # 路线预览为只读
            who = self.locks.allowed(h.headers.get("X-Lock-Token", ""))
            if who:
                return self.send(h, 423, {"error": {"code": "locked", "message": f"控制权由 {who} 持有，当前为只读"}})
        try:
            status, payload = self.api.dispatch(method, h.path, body, h.headers, h.client_address)
        except ApiError as e:
            status, payload = e.status, {"error": {"code": e.code, "message": e.message}}
        except Exception as e:
            import traceback
            traceback.print_exc()
            status, payload = 500, {"error": {"code": "internal", "message": str(e)}}
        self.send(h, status, payload if payload is not None else {"ok": True})

    def guard_legacy(self, h) -> bool:
        """旧接口写操作: 有人持锁且未带令牌 → 423。返回 True 表示已拒绝"""
        who = self.locks.allowed(h.headers.get("X-Lock-Token", ""))
        if who:
            self.send(h, 423, {"error": {"code": "locked", "message": f"控制权由 {who} 持有，当前为只读"}, "status": "error",
                               "message": f"控制权由 {who} 持有"})
            return True
        return False

    @staticmethod
    def send(h, status, payload):
        extra = {}
        if isinstance(payload, (RawBody, BinaryBody)):
            data, ctype, extra = payload.data, payload.content_type, payload.headers
            status = getattr(payload, "status", status)
        else:
            data, ctype = json.dumps(payload, ensure_ascii=False).encode("utf-8"), "application/json; charset=utf-8"
        h.send_response(status)
        h.send_header("Content-Type", ctype)
        h.send_header("Content-Length", str(len(data)))
        h.send_header("Cache-Control", "no-cache, no-store")
        h.send_header("Access-Control-Allow-Origin", "*")
        h.send_header("Access-Control-Expose-Headers", "*")
        for k, v in extra.items():
            h.send_header(k, v)
        h.end_headers()
        h.wfile.write(data)
