//! Task-flow runner (port of nav_runtime/taskflow.py)
use crate::events::now;
use crate::navigator::{sleep, Navigator};
use serde_json::{json, Value};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, Weak};

const FAIL_STATES: [&str; 6] = ["FAILED", "NO_PATH", "CANCELED", "REJECTED", "BUMPER_STOP", "ABORTED"];
fn label(t: &str) -> Option<&'static str> {
    Some(match t { "move" => "导航移动", "lift" => "顶升取货", "drop" => "下降放货", "wait" => "等待", "pick" => "叉取货物", "place" => "放置货物", "fork" => "货叉升降", _ => return None })
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
            if t == "move" || (t == "place" && st.get("x").is_none()) { self.resolve(st["target"].as_str().unwrap_or(""))?; }
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
                let (ok, msg) = match ty { "wait" => self.do_wait(gen, st), "move" => self.do_move(gen, st), "lift" => self.lift(gen, true), "pick" => self.pick(gen, st), "place" => self.place(gen, st), "fork" => self.fork_step(gen, st), _ => self.lift(gen, false) };
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
        let failed = self.st()["status"] == "failed";
        if self.alive(gen) && !failed {
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
    /// A failed move is retried (transient planner rejections, Nav2 warming up, ...) before the whole task flow fails.
    fn do_move(&self, gen: u64, st: &Value) -> (bool, String) {
        const RETRIES: u32 = 3;
        let mut last = (false, String::new());
        for attempt in 0..=RETRIES {
            last = self.move_once(gen, st);
            if last.0 || !self.alive(gen) { return last; }
            let retryable = ["FAILED", "NO_PATH", "REJECTED", "ABORTED"].iter().any(|s| last.1.ends_with(&format!(": {s}")));
            if !retryable || attempt == RETRIES { break; }
            self.emit("TASKFLOW_RETRY", "warning", &format!("工步移动失败，{} 秒后重试 ({}/{RETRIES})", 3 * (attempt + 1), attempt + 1), &last.1, json!({"attempt": attempt + 1}));
            if !self.wait_sim(gen, 3.0 * (attempt + 1) as f64) { break; }
        }
        last
    }
    fn move_once(&self, gen: u64, st: &Value) -> (bool, String) {
        let (x, y, yaw, name) = match self.resolve(st["target"].as_str().unwrap_or("")) { Ok(v) => v, Err(e) => return (false, e) };
        self.goto_xy(gen, st, x, y, yaw, name)
    }
    fn goto_xy(&self, gen: u64, st: &Value, x: f64, y: f64, yaw: Option<f64>, name: String) -> (bool, String) {
        let nav = self.nav();
        if let Some(yw) = yaw {
            let (cx, cy, cth) = nav.pose();
            if (x - cx).hypot(y - cy) < 0.12 && wrap(yw - cth).abs() < 0.08 { return (true, format!("已在 {name}")); }
        }
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

// ---------------------------------------------------------------------------------------------------- forklift steps
fn wrap(a: f64) -> f64 { let mut a = a; while a > std::f64::consts::PI { a -= 2.0 * std::f64::consts::PI; } while a < -std::f64::consts::PI { a += 2.0 * std::f64::consts::PI; } a }

impl TaskFlow {
    fn sim_get(&self, p: &str) -> Value { self.nav().link.c_misc.get(p).unwrap_or(Value::Null) }
    fn sim_put(&self, p: &str, b: Value) -> Value { self.nav().link.c_misc.put(p, &b).unwrap_or(Value::Null) }
    fn truth(&self) -> (f64, f64, f64) {
        let t = self.sim_get("/api/v1/state");
        (t["truth"]["x"].as_f64().unwrap_or(0.0), t["truth"]["y"].as_f64().unwrap_or(0.0), t["truth"]["yaw"].as_f64().unwrap_or(0.0))
    }
    fn cmd(&self, vx: f64, vy: f64, wz: f64) { self.nav().link.send_cmd(vx, vy, wz, "forklift"); }
    fn halt(&self) { for _ in 0..3 { self.cmd(0.0, 0.0, 0.0); sleep(30); } }

    /// command the mast to `h` and wait until it is there (or blocked / timeout)
    fn fork_to(&self, gen: u64, h: f64, timeout: f64) -> Result<Value, String> {
        let r = self.sim_put("/api/v1/fork", json!({"height": h}));
        if r.is_null() { return Err("货叉不可用 (车型没有门架)".into()); }
        let t0 = now();
        while self.alive(gen) && now() - t0 < timeout {
            sleep(100);
            let f = self.sim_get("/api/v1/fork");
            if (f["height_m"].as_f64().unwrap_or(-9.0) - h).abs() < 0.012 && f["velocity_mps"].as_f64().unwrap_or(1.0).abs() < 0.02 { return Ok(f); }
            if f["state"] == "BLOCKED" { return Err(format!("货叉升降受阻: {}", f["blocked"].as_str().unwrap_or(""))); }
        }
        if !self.alive(gen) { return Err("已终止".into()); }
        Err(format!("货叉升降超时 (目标 {h:.2} m)"))
    }

    fn fork_step(&self, gen: u64, st: &Value) -> (bool, String) {
        let h = st.get("height").and_then(|v| v.as_f64()).unwrap_or(0.0);
        match self.fork_to(gen, h, 60.0) { Ok(f) => (true, format!("货叉到 {:.2} m", f["height_m"].as_f64().unwrap_or(h))), Err(e) => (false, e) }
    }

    /// holonomic servo: align lateral/yaw to the pallet axis, then drive straight until the pallet centre is `d` ahead of the vehicle origin.
    fn servo_insert(&self, gen: u64, cx: f64, cy: f64, yaw_in: f64, d: f64, vmax: f64) -> Result<(), String> {
        let rel = || { let (x, y, th) = self.truth(); let (dx, dy) = (cx - x, cy - y); (th.cos() * dx + th.sin() * dy, -th.sin() * dx + th.cos() * dy, wrap(yaw_in - th)) };
        let t0 = now();
        let mut ok_n = 0;
        // phase A: pure alignment (no forward motion)
        while self.alive(gen) {
            let (_, ly, ey) = rel();
            if ly.abs() < 0.012 && ey.abs() < 0.006 { ok_n += 1; } else { ok_n = 0; }
            if ok_n >= 4 { break; }
            if now() - t0 > 40.0 { self.halt(); return Err(format!("对中超时 (横向 {:.3} m, 偏航 {:.1}°)", ly, ey.to_degrees())); }
            self.cmd(0.0, (1.6 * ly).clamp(-0.2, 0.2), (1.6 * ey).clamp(-0.25, 0.25));
            sleep(50);
        }
        // phase B: straight insertion with continuous lateral/yaw correction
        let t1 = now();
        let (mut last_lx, mut last_t) = (rel().0, now());
        while self.alive(gen) {
            let (lx, ly, ey) = rel();
            if lx <= d + 0.01 { break; }
            if now() - t1 > 60.0 { self.halt(); return Err("插入超时".into()); }
            if (last_lx - lx).abs() > 0.01 { last_lx = lx; last_t = now(); }
            if now() - last_t > 3.0 { self.halt(); return Err(format!("插入受阻 (剩余 {:.2} m)", lx - d)); }
            let vx = (0.8 * (lx - d)).clamp(0.05, vmax);
            self.cmd(vx, (1.6 * ly).clamp(-0.1, 0.1), (1.6 * ey).clamp(-0.1, 0.1));
            sleep(50);
        }
        self.halt();
        if !self.alive(gen) { return Err("已终止".into()); }
        Ok(())
    }

    fn reverse_out(&self, gen: u64, dist: f64) {
        let (x0, y0, _) = self.truth();
        let t0 = now();
        while self.alive(gen) && now() - t0 < 30.0 {
            let (x, y, _) = self.truth();
            if (x - x0).hypot(y - y0) >= dist { break; }
            self.cmd(-0.3, 0.0, 0.0);
            sleep(50);
        }
        self.halt();
    }

    fn tine_geom(&self) -> (f64, f64) {
        let f = self.sim_get("/api/v1/fork");
        let t = &f["tine"];
        (t["root_x"].as_f64().unwrap_or(1.15) + t["length"].as_f64().unwrap_or(1.1), t["length"].as_f64().unwrap_or(1.1))
    }

    fn pick(&self, gen: u64, st: &Value) -> (bool, String) {
        let f0 = self.sim_get("/api/v1/fork");
        if f0.is_null() { return (false, "货叉不可用 (车型没有门架)".into()); }
        if f0["loaded"] == true { return (false, "货叉上已有货物".into()); }
        let cargo = self.sim_get("/api/v1/world/cargo");
        let list = cargo.as_array().cloned().unwrap_or_default();
        let (vx0, vy0, _) = self.truth();
        // which pallet
        let want = st.get("cargo").and_then(|c| c.as_str()).unwrap_or("");
        let near = st.get("target").and_then(|t| t.as_str()).and_then(|t| self.resolve(t).ok());
        let mut best: Option<Value> = None;
        let mut bd = f64::MAX;
        for c in &list {
            if c["pocket_h"].as_f64().unwrap_or(0.0) <= 0.0 || c["carried"] == true { continue; }
            if !want.is_empty() { if c["id"].as_str() == Some(want) { best = Some(c.clone()); break; } continue; }
            let (rx, ry) = near.as_ref().map(|n| (n.0, n.1)).unwrap_or((vx0, vy0));
            let d = (c["x"].as_f64().unwrap_or(0.0) - rx).hypot(c["y"].as_f64().unwrap_or(0.0) - ry);
            if d < bd && (near.is_none() || d < 2.0) { bd = d; best = Some(c.clone()); }
        }
        let Some(p) = best else { return (false, if want.is_empty() { "附近没有可叉取的托盘".into() } else { format!("找不到托盘 {want}") }) };
        let id = p["id"].as_str().unwrap_or("").to_string();
        let (cx, cy, pyaw, depth) = (p["x"].as_f64().unwrap_or(0.0), p["y"].as_f64().unwrap_or(0.0), p["yaw"].as_f64().unwrap_or(0.0), p["w"].as_f64().unwrap_or(1.2));
        let (tip_x, _len) = self.tine_geom();
        let inside = (depth - 0.2).min(1.0);
        let d = tip_x + depth / 2.0 - inside; // vehicle origin -> pallet centre when fully inserted
        let approach = st.get("approach").and_then(|v| v.as_f64()).unwrap_or(2.0);
        // entry side: nearer of the two ends
        let cand = [pyaw, wrap(pyaw + std::f64::consts::PI)];
        let stage_of = |y: f64| (cx - (d + approach) * y.cos(), cy - (d + approach) * y.sin());
        let yaw_in = *cand.iter().min_by(|a, b| { let (sa, sb) = (stage_of(**a), stage_of(**b)); (sa.0 - vx0).hypot(sa.1 - vy0).partial_cmp(&(sb.0 - vx0).hypot(sb.1 - vy0)).unwrap() }).unwrap();
        let (sx, sy) = stage_of(yaw_in);
        self.emit("FORK_PICK_PLAN", "info", &format!("叉取 {id}"), &format!("托盘 ({cx:.2},{cy:.2}) 朝向 {:.0}°，预备位 ({sx:.2},{sy:.2})", yaw_in.to_degrees()), json!({"cargo": id}));
        if let Err(e) = self.fork_to(gen, 0.0, 60.0) { return (false, e); }
        let (ok, msg) = self.goto_xy(gen, st, sx, sy, Some(yaw_in), format!("托盘预备位({id})"));
        if !ok { return (false, msg); }
        if let Err(e) = self.servo_insert(gen, cx, cy, yaw_in, d, 0.25) { return (false, format!("叉入托盘 {id} 失败: {e}")); }
        let lift_h = st.get("height").and_then(|v| v.as_f64()).unwrap_or(0.25).max(0.12);
        if let Err(e) = self.fork_to(gen, lift_h, 60.0) { return (false, format!("起升失败: {e}")); }
        let f = self.sim_get("/api/v1/fork");
        if f["loaded"] != true { return (false, format!("未能叉起托盘 {id}")); }
        self.reverse_out(gen, approach - 0.1);
        (true, format!("已叉取 {id}，起升到 {lift_h:.2} m"))
    }

    fn place(&self, gen: u64, st: &Value) -> (bool, String) {
        let f0 = self.sim_get("/api/v1/fork");
        if f0.is_null() { return (false, "货叉不可用 (车型没有门架)".into()); }
        if f0["loaded"] != true { return (false, "货叉上没有货物".into()); }
        let (tx, ty, tyaw, name) = if let (Some(x), Some(y)) = (st.get("x").and_then(|v| v.as_f64()), st.get("y").and_then(|v| v.as_f64())) {
            (x, y, st.get("yaw").and_then(|v| v.as_f64()), format!("({x:.2},{y:.2})"))
        } else { match self.resolve(st["target"].as_str().unwrap_or("")) { Ok(v) => v, Err(e) => return (false, e) } };
        let z = st.get("z").and_then(|v| v.as_f64()).unwrap_or(0.0).max(0.0);
        let (vx, vy, th) = self.truth();
        let yaw_in = tyaw.unwrap_or_else(|| (ty - vy).atan2(tx - vx));
        // current pallet centre relative to vehicle origin (distance along heading)
        let root = f0["load"].as_array().and_then(|l| l.iter().find(|c| c["id"] == f0["load_id"]).cloned()).unwrap_or(json!({}));
        let (px, py) = (root["x"].as_f64().unwrap_or(vx + 2.0 * th.cos()), root["y"].as_f64().unwrap_or(vy + 2.0 * th.sin()));
        let d = (th.cos() * (px - vx) + th.sin() * (py - vy)).max(1.0);
        let approach = st.get("approach").and_then(|v| v.as_f64()).unwrap_or(2.0);
        let (sx, sy) = (tx - (d + approach) * yaw_in.cos(), ty - (d + approach) * yaw_in.sin());
        self.emit("FORK_PLACE_PLAN", "info", &format!("放置到 {name}"), &format!("目标高度 {z:.2} m，预备位 ({sx:.2},{sy:.2})"), json!({"target": name}));
        let travel = z + 0.25;
        let cur_h = f0["height_m"].as_f64().unwrap_or(0.0);
        if cur_h < travel - 0.01 { if let Err(e) = self.fork_to(gen, travel, 90.0) { return (false, e); } }
        let (ok, msg) = self.goto_xy(gen, st, sx, sy, Some(yaw_in), format!("放置预备位{name}"));
        if !ok { return (false, msg); }
        if cur_h > travel + 0.01 {
            // carrying high: lower to just above the support before entering
            if let Err(e) = self.fork_to(gen, travel, 90.0) { return (false, e); }
        }
        if let Err(e) = self.servo_insert(gen, tx, ty, yaw_in, d, 0.25) { return (false, format!("送入失败: {e}")); }
        let down = if z <= 0.001 { 0.0 } else { z + 0.01 };
        match self.fork_to(gen, down, 90.0) { Ok(_) => {}, Err(e) => { if self.sim_get("/api/v1/fork")["loaded"] == true { return (false, format!("下放失败: {e}")); } } }
        if self.sim_get("/api/v1/fork")["loaded"] == true { return (false, "货物未能放下".into()); }
        self.reverse_out(gen, approach - 0.1);
        let _ = self.fork_to(gen, 0.1, 60.0);
        (true, format!("已放置到 {name}"))
    }
}
