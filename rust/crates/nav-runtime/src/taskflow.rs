//! Task-flow runner (port of nav_runtime/taskflow.py)
use crate::events::now;
use crate::navigator::{sleep, Navigator};
use serde_json::{json, Value};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, Weak};

const FAIL_STATES: [&str; 6] = ["FAILED", "NO_PATH", "CANCELED", "REJECTED", "BUMPER_STOP", "ABORTED"];
fn label(t: &str) -> Option<&'static str> {
    Some(match t { "move" => "导航移动", "lift" => "顶升取货", "drop" => "下降放货", "wait" => "等待", _ => return None })
}

pub struct TaskFlow {
    nav: Weak<Navigator>,
    state: Mutex<Value>,
    gen: AtomicU64,
}

impl TaskFlow {
    pub fn new(nav: &Arc<Navigator>) -> Arc<TaskFlow> { Arc::new(TaskFlow { nav: Arc::downgrade(nav), state: Mutex::new(json!({"status": "idle"})), gen: AtomicU64::new(0) }) }
    fn nav(&self) -> Arc<Navigator> { self.nav.upgrade().expect("navigator dropped") }
    fn st(&self) -> std::sync::MutexGuard<'_, Value> { self.state.lock().unwrap_or_else(|e| e.into_inner()) }
    fn set(&self, kv: Value) { let mut s = self.st(); for (k, v) in kv.as_object().unwrap() { s[k] = v.clone(); } }

    pub fn view(&self) -> Value {
        let mut s = self.st().clone();
        if s["status"] == "running" {
            if let Some(t) = s["started"].as_f64() { s["elapsed_s"] = json!(((now() - t) * 10.0).round_ties_even() / 10.0); }
        }
        s
    }
    fn emit(&self, typ: &str, level: &str, title: &str, msg: &str, payload: Value) { self.nav().ev("task", typ, level, title, msg, payload); }
    fn alive(&self, gen: u64) -> bool { self.gen.load(Ordering::SeqCst) == gen }

    pub fn resolve(&self, target: &str) -> Result<(f64, f64, Option<f64>, String), String> {
        let w = self.nav().link.world();
        if let Some(sts) = w["stations"].as_array() {
            for s in sts {
                if s["id"].as_str() == Some(target) || s["name"].as_str() == Some(target) {
                    return Ok((s["x"].as_f64().unwrap_or(0.0), s["y"].as_f64().unwrap_or(0.0), Some(s["dock_yaw"].as_f64().unwrap_or(0.0)),
                               s["name"].as_str().unwrap_or(target).to_string()));
                }
            }
        }
        if let Some(n) = w.pointer("/scenario_def/nodes").and_then(|n| n.get(target)) {
            return Ok((n[0].as_f64().unwrap_or(0.0), n[1].as_f64().unwrap_or(0.0), None, target.to_string()));
        }
        if let Some(n) = w.pointer("/topology/nodes").and_then(|n| n.get(target)) {
            return Ok((n["x"].as_f64().unwrap_or(0.0), n["y"].as_f64().unwrap_or(0.0), None, target.to_string()));
        }
        Err(format!("未知工位/节点 {target}"))
    }

    pub fn run(self: &Arc<Self>, flow: &Value) -> Result<Value, String> {
        let steps = flow["steps"].as_array().cloned().unwrap_or_default();
        if steps.is_empty() { return Err("任务流没有工步".into()); }
        for (i, st) in steps.iter().enumerate() {
            let t = st["type"].as_str().unwrap_or("");
            if label(t).is_none() { return Err(format!("工步 {} 类型无效: {}", i + 1, st["type"].as_str().unwrap_or("None"))); }
            if t == "move" { self.resolve(st["target"].as_str().unwrap_or(""))?; }
        }
        self.stop(true, "STOPPED");
        let gen = self.gen.fetch_add(1, Ordering::SeqCst) + 1;
        let fl = json!({"id": flow["id"], "name": flow["name"], "tid": flow["tid"], "loop": flow["loop"]});
        *self.st() = json!({"status": "running", "flow": fl, "steps": steps, "step_index": 0, "step_status": "pending", "loop_count": 0,
                            "started": now(), "ended": null, "message": "", "log": []});
        let g = |k: &str| flow.get(k).filter(|v| !v.is_null()).map(|v| v.as_str().map(|s| s.to_string()).unwrap_or(v.to_string())).unwrap_or("-".into());
        self.emit("TASKFLOW_START", "info", &format!("下发仿真任务流 {}", flow["id"].as_str().unwrap_or("")),
                  &format!("TID:{} [{}] 共 {} 个工步，{}", g("tid"), flow["name"].as_str().unwrap_or(""), steps.len(), if flow["loop"] == "infinite" { "无限循环" } else { "单次执行" }),
                  json!({"flow": flow["id"], "tid": flow["tid"]}));
        let me = self.clone();
        let flow = flow.clone();
        std::thread::Builder::new().name("taskflow".into()).spawn(move || me.worker(gen, flow)).unwrap();
        Ok(self.view())
    }

    pub fn stop(&self, quiet: bool, reason: &str) -> Value {
        let was = self.view()["status"] == "running";
        self.gen.fetch_add(1, Ordering::SeqCst);
        if was {
            self.nav().cancel_nav();
            self.set(json!({"status": "stopped", "ended": now(), "message": reason}));
            if !quiet { self.emit("TASKFLOW_STOP", "warning", "任务流已终止", reason, json!({})); }
        }
        self.nav().s().speed_cap = None;
        self.view()
    }

    fn worker(&self, gen: u64, flow: Value) {
        let steps = flow["steps"].as_array().cloned().unwrap_or_default();
        let looped = flow["loop"] == "infinite";
        let tid = flow.get("tid").filter(|v| !v.is_null()).map(|v| v.as_str().map(|s| s.to_string()).unwrap_or(v.to_string())).unwrap_or("-".into());
        let mut n = 0u64;
        'outer: while self.alive(gen) {
            for (i, st) in steps.iter().enumerate() {
                if !self.alive(gen) { break 'outer; }
                self.set(json!({"step_index": i, "step_status": "running", "step_started": now()}));
                let ty = st["type"].as_str().unwrap_or("");
                let tgt = st["target"].as_str().unwrap_or("");
                self.emit("TASKFLOW_STEP", "info", &format!("工步 {:02}: {}{}", i + 1, label(ty).unwrap(), if tgt.is_empty() { String::new() } else { format!(" {tgt}") }),
                          &format!("TID:{tid} 第 {} 轮", n + 1), json!({"index": i, "step": st}));
                let t0 = now();
                let (ok, msg) = match ty { "wait" => self.do_wait(gen, st), "move" => self.do_move(gen, st), "lift" => self.lift(gen, true), _ => self.lift(gen, false) };
                if !self.alive(gen) { break 'outer; }
                {
                    let mut s = self.st();
                    let entry = json!({"index": i, "type": ty, "target": tgt, "t0": t0, "t1": now(), "ok": ok, "msg": msg, "loop": n});
                    let log = s["log"].as_array_mut().unwrap();
                    log.push(entry);
                    if log.len() > 200 { let k = log.len() - 200; log.drain(..k); }
                }
                if !ok {
                    self.set(json!({"status": "failed", "step_status": "failed", "ended": now(), "message": msg}));
                    self.emit("TASKFLOW_FAIL", "danger", &format!("工步 {:02} 失败，任务流终止", i + 1), &msg, json!({"index": i}));
                    break 'outer;
                }
                self.set(json!({"step_status": "done"}));
            }
            n += 1;
            self.set(json!({"loop_count": n}));
            if !looped { break; }
        }
        if self.alive(gen) {
            let v = self.view();
            self.set(json!({"status": "done", "ended": now(), "message": "全部工步完成"}));
            self.emit("TASKFLOW_DONE", "success", "任务流圆满完成", &format!("全部 {} 个工步执行完毕，用时 {:.1} s", steps.len(), now() - v["started"].as_f64().unwrap_or(now())), json!({}));
        }
        if self.alive(gen) { self.nav().s().speed_cap = None; }
    }

    fn wait_sim(&self, gen: u64, seconds: f64) -> bool {
        let mut left = seconds;
        let mut last = now();
        while left > 0.0 && self.alive(gen) {
            sleep(100);
            let t = now();
            if !self.nav().s().paused { left -= t - last; }
            last = t;
        }
        self.alive(gen)
    }
    fn do_wait(&self, gen: u64, st: &Value) -> (bool, String) {
        let s = st.get("seconds").or(st.get("duration")).and_then(|v| v.as_f64()).filter(|v| *v != 0.0).unwrap_or(3.0);
        (self.wait_sim(gen, s), format!("等待 {s:.1} s"))
    }
    fn do_move(&self, gen: u64, st: &Value) -> (bool, String) {
        let nav = self.nav();
        let (x, y, yaw, name) = match self.resolve(st["target"].as_str().unwrap_or("")) { Ok(v) => v, Err(e) => return (false, e) };
        nav.s().speed_cap = st.get("speed").and_then(|v| v.as_f64()).filter(|v| *v != 0.0);
        let (cx, cy, _) = nav.pose();
        let yaw = yaw.unwrap_or_else(|| (y - cy).atan2(x - cx));
        let m = nav.submit(x, y, yaw, None).ok().flatten();
        let mut mid = m.as_ref().and_then(|m| m["id"].as_u64());
        let t0 = now();
        let mut seen = false;
        while self.alive(gen) {
            sleep(200);
            let (status, cur_mid) = { let s = nav.s(); (s.tele.nav_status.clone(), s.mission.as_ref().map(|m| m.id)) };
            if let (Some(a), Some(b)) = (mid, cur_mid) {
                if a != b && !["PLANNING", "NAVIGATING", "OBSTACLE_WAIT"].contains(&status.as_str()) { mid = Some(b); }
            }
            if ["PLANNING", "NAVIGATING", "OBSTACLE_WAIT", "SAFETY_STOP"].contains(&status.as_str()) { seen = true; }
            if status == "ARRIVED" && (seen || now() - t0 > 1.0) { return (true, format!("到达 {name}")); }
            if FAIL_STATES.contains(&status.as_str()) && (seen || now() - t0 > 2.0) { return (false, format!("前往 {name} 失败: {status}")); }
            if now() - t0 > 900.0 { return (false, format!("前往 {name} 超时")); }
        }
        (false, "已终止".into())
    }
    fn io(&self, d: Value) { let _ = self.nav().link.c_misc.put("/api/v1/io", &json!({"do": d})); }
    fn di(&self, name: &str) -> bool { self.nav().link.io().pointer("/inputs").and_then(|i| i.get(name)).map(|v| v.as_bool().unwrap_or(false) || v.as_i64().unwrap_or(0) != 0).unwrap_or(false) }
    fn lift(&self, gen: u64, up: bool) -> (bool, String) {
        let (motor, other, limit) = if up { ("do_lift_motor_up", "do_lift_motor_down", "di_lift_top") } else { ("do_lift_motor_down", "do_lift_motor_up", "di_lift_bottom") };
        if self.di(limit) { return (true, "已在限位".into()); }
        self.io(json!({motor: true, other: false}));
        let t0 = now();
        let mut res = (false, format!("{}超时，未到 {limit}", if up { "顶升" } else { "下降" }));
        while self.alive(gen) && now() - t0 < 20.0 {
            sleep(100);
            if self.di(limit) { res = (true, format!("{}到位 ({:.1} s)", if up { "顶升" } else { "下降" }, now() - t0)); break; }
        }
        self.io(json!({motor: false}));
        res
    }
}
