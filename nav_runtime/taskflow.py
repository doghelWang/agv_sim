#!/usr/bin/env python3
"""
任务流 (工步序列) 执行器 —— 运行在执行进程内

任务流: {id, name, tid, loop: single|infinite, steps: [{type, target, speed, seconds}]}
  move  前往工位/拓扑节点 (target=工位 id 或节点 id，speed=限速 m/s)，走现有导航任务 (Dijkstra/A*/Nav2)
  lift  顶升/货叉抬起: 写仿真 IO do_lift_motor_up，等待 di_lift_top
  drop  下降:         写仿真 IO do_lift_motor_down，等待 di_lift_bottom
  wait  原地等待 seconds 秒 (仿真暂停时不计时)

进度随执行进程的导航回馈 (feedback.taskflow) 上报，事件类型 TASKFLOW_* (Web 工作台标签 TSK)。
"""

import math
import threading
import time
from typing import Optional

STEP_LABEL = {"move": "导航移动", "lift": "顶升取货", "drop": "下降放货", "wait": "等待"}
FAIL_STATES = ("FAILED", "NO_PATH", "CANCELED", "REJECTED", "BUMPER_STOP", "ABORTED")


class TaskFlowRunner:
    def __init__(self, nav, link):
        self.nav, self.link = nav, link
        self.lock = threading.Lock()
        self.state = {"status": "idle"}
        self._stop = threading.Event()
        self._gen = 0

    # ------------------------------------------------------------------ 查询
    def view(self) -> dict:
        with self.lock:
            s = dict(self.state)
        if s.get("status") == "running" and s.get("started"):
            s["elapsed_s"] = round(time.time() - s["started"], 1)
        return s

    def _emit(self, typ: str, level: str, title: str, msg: str, payload: Optional[dict] = None):
        self.nav.event_hub.emit("task", typ, level, title, msg, payload or {})

    def _set(self, **kw):
        with self.lock:
            self.state.update(kw)

    # ------------------------------------------------------------------ 解析目标
    def resolve(self, target: str):
        w = self.link.world or {}
        for s in w.get("stations", []):
            if s.get("id") == target or s.get("name") == target:
                return float(s["x"]), float(s["y"]), float(s.get("dock_yaw", 0.0)), s.get("name") or target
        nodes = (w.get("scenario_def") or {}).get("nodes") or {}
        if target in nodes:
            x, y = nodes[target][:2]
            return float(x), float(y), None, target
        topo = (w.get("topology") or {}).get("nodes") or {}
        if target in topo:
            n = topo[target]
            return float(n["x"]), float(n["y"]), None, target
        raise ValueError(f"未知工位/节点 {target}")

    # ------------------------------------------------------------------ 控制
    def run(self, flow: dict) -> dict:
        steps = flow.get("steps") or []
        if not steps:
            raise ValueError("任务流没有工步")
        for i, st in enumerate(steps):
            if st.get("type") not in STEP_LABEL:
                raise ValueError(f"工步 {i + 1} 类型无效: {st.get('type')}")
            if st["type"] == "move":
                self.resolve(str(st.get("target", "")))
        self.stop(quiet=True)
        self._stop.clear()
        self._gen += 1
        gen = self._gen
        with self.lock:
            self.state = {"status": "running", "flow": {k: flow.get(k) for k in ("id", "name", "tid", "loop")}, "steps": steps,
                          "step_index": 0, "step_status": "pending", "loop_count": 0, "started": time.time(), "ended": None,
                          "message": "", "log": []}
        self._emit("TASKFLOW_START", "info", f"下发仿真任务流 {flow.get('id', '')}",
                   f"TID:{flow.get('tid', '-')} [{flow.get('name', '')}] 共 {len(steps)} 个工步，"
                   f"{'无限循环' if flow.get('loop') == 'infinite' else '单次执行'}", {"flow": flow.get("id"), "tid": flow.get("tid")})
        threading.Thread(target=self._worker, args=(gen, flow), daemon=True, name="taskflow").start()
        return self.view()

    def stop(self, quiet: bool = False, reason: str = "STOPPED") -> dict:
        was = self.view().get("status") == "running"
        self._stop.set()
        self._gen += 1
        if was:
            try:
                self.nav.cancel_nav()
            except Exception:
                pass
            self._set(status="stopped", ended=time.time(), message=reason)
            if not quiet:
                self._emit("TASKFLOW_STOP", "warning", "任务流已终止", reason)
        self.nav.speed_cap = None
        return self.view()

    # ------------------------------------------------------------------ 执行
    def _alive(self, gen) -> bool:
        return gen == self._gen and not self._stop.is_set()

    def _worker(self, gen: int, flow: dict):
        steps = flow["steps"]
        loop = flow.get("loop") == "infinite"
        n = 0
        try:
            while self._alive(gen):
                for i, st in enumerate(steps):
                    if not self._alive(gen):
                        return
                    self._set(step_index=i, step_status="running", step_started=time.time())
                    label = STEP_LABEL[st["type"]]
                    tgt = st.get("target", "")
                    self._emit("TASKFLOW_STEP", "info", f"工步 {i + 1:02d}: {label}{' ' + tgt if tgt else ''}",
                               f"TID:{flow.get('tid', '-')} 第 {n + 1} 轮", {"index": i, "step": st})
                    t0 = time.time()
                    ok, msg = getattr(self, "_do_" + st["type"])(gen, st)
                    if not self._alive(gen):
                        return
                    with self.lock:
                        self.state.setdefault("log", []).append({"index": i, "type": st["type"], "target": tgt, "t0": t0,
                                                                 "t1": time.time(), "ok": ok, "msg": msg, "loop": n})
                        self.state["log"] = self.state["log"][-200:]
                    if not ok:
                        self._set(status="failed", step_status="failed", ended=time.time(), message=msg)
                        self._emit("TASKFLOW_FAIL", "danger", f"工步 {i + 1:02d} 失败，任务流终止", msg, {"index": i})
                        return
                    self._set(step_status="done")
                n += 1
                self._set(loop_count=n)
                if not loop:
                    break
            if self._alive(gen):
                v = self.view()
                self._set(status="done", ended=time.time(), message="全部工步完成")
                self._emit("TASKFLOW_DONE", "success", "任务流圆满完成",
                           f"全部 {len(steps)} 个工步执行完毕，用时 {time.time() - v.get('started', time.time()):.1f} s", {})
        finally:
            if gen == self._gen:
                self.nav.speed_cap = None

    def _wait_sim(self, gen, seconds: float) -> bool:
        """按仿真时间等待 (暂停不计时)"""
        left = seconds
        last = time.time()
        while left > 0 and self._alive(gen):
            time.sleep(0.1)
            now = time.time()
            if not self.nav.is_paused:
                left -= now - last
            last = now
        return self._alive(gen)

    def _do_wait(self, gen, st):
        s = float(st.get("seconds") or st.get("duration") or 3.0)
        return self._wait_sim(gen, s), f"等待 {s:.1f} s"

    def _do_move(self, gen, st):
        x, y, yaw, name = self.resolve(str(st.get("target")))
        sp = st.get("speed")
        self.nav.speed_cap = float(sp) if sp else None
        cur = self.nav.telemetry
        if yaw is None:
            yaw = math.atan2(y - cur.get("y", 0.0), x - cur.get("x", 0.0))
        m = self.nav.submit(x, y, yaw)
        mid = (m or {}).get("id")
        t0 = time.time()
        seen_active = False
        while self._alive(gen):
            time.sleep(0.2)
            status = self.nav.telemetry.get("nav_status")
            cur_mid = (self.nav.mission or {}).get("id")
            if mid is not None and cur_mid is not None and cur_mid != mid and status not in ("PLANNING", "NAVIGATING", "OBSTACLE_WAIT"):
                mid = cur_mid          # 执行进程内部重规划产生的新任务
            if status in ("PLANNING", "NAVIGATING", "OBSTACLE_WAIT", "SAFETY_STOP"):
                seen_active = True
            if status == "ARRIVED" and (seen_active or time.time() - t0 > 1.0):
                return True, f"到达 {name}"
            if status in FAIL_STATES and (seen_active or time.time() - t0 > 2.0):
                return False, f"前往 {name} 失败: {status}"
            if time.time() - t0 > 900:
                return False, f"前往 {name} 超时"
        return False, "已终止"

    def _io(self, do: dict):
        self.link.c_misc.put("/api/v1/io", {"do": do})

    def _di(self, name: str) -> bool:
        return bool(((self.link.io or {}).get("inputs") or {}).get(name))

    def _lift(self, gen, up: bool):
        motor, other, limit = ("do_lift_motor_up", "do_lift_motor_down", "di_lift_top") if up else \
            ("do_lift_motor_down", "do_lift_motor_up", "di_lift_bottom")
        if self._di(limit):
            return True, "已在限位"
        try:
            self._io({motor: True, other: False})
            t0 = time.time()
            while self._alive(gen) and time.time() - t0 < 20:
                time.sleep(0.1)
                if self._di(limit):
                    return True, f"{'顶升' if up else '下降'}到位 ({time.time() - t0:.1f} s)"
            return False, f"{'顶升' if up else '下降'}超时，未到 {limit}"
        finally:
            try:
                self._io({motor: False})
            except Exception:
                pass

    def _do_lift(self, gen, st):
        return self._lift(gen, True)

    def _do_drop(self, gen, st):
        return self._lift(gen, False)
