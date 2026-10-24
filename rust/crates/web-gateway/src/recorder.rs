use crate::util::*;
use serde_json::{json, Value};
use std::sync::Mutex;

/// A recorded session: header (no frames) + frames kept as compact JSON strings. A serde_json::Value tree costs ~15x the
/// serialized size (23 KB vs 1.5 KB per frame), so retaining 20 sessions as Values grew the gateway to 650 MB.
#[derive(Clone)]
struct Sess { head: Value, frames: Vec<String> }
impl Sess {
    fn full(&self) -> Value { let mut v = self.head.clone(); v["frames"] = Value::Array(self.frames.iter().filter_map(|f| serde_json::from_str(f).ok()).collect()); v }
}
struct Inner { completed: Vec<Sess>, active: Option<Sess>, rolling: std::collections::VecDeque<(f64, String)>, last_cmd: Value }
pub struct FlightRecorder { g: Mutex<Inner>, max_sessions: usize, max_rolling: usize }

fn stats(frames: &[Value]) -> (f64, f64, f64, f64) {
    let (mut dist, mut maxs, mut sum, mut mind) = (0.0f64, 0.0f64, 0.0f64, 99.0f64);
    for (i, fr) in frames.iter().enumerate() {
        if i > 0 { let (p0, p1) = (&frames[i - 1]["pose"], &fr["pose"]); dist += (f(p1, "x", 0.0) - f(p0, "x", 0.0)).hypot(f(p1, "y", 0.0) - f(p0, "y", 0.0)); }
        let v = f(&fr["vel"], "vx", 0.0).hypot(f(&fr["vel"], "vy", 0.0));
        sum += v; if v > maxs { maxs = v; }
        let m = fr.get("perception").map(|p| f(p, "min_dist", 99.0)).unwrap_or(99.0);
        if m < mind { mind = m; }
    }
    let n = frames.len().max(1) as f64;
    (rnd(dist, 2), rnd(maxs, 2), rnd(sum / n, 2), if mind < 90.0 { rnd(mind, 2) } else { 12.0 })
}

impl FlightRecorder {
    pub fn new() -> FlightRecorder { FlightRecorder { g: Mutex::new(Inner { completed: vec![], active: None, rolling: Default::default(), last_cmd: json!({"vx": 0.0, "vy": 0.0, "wz": 0.0}) }), max_sessions: 20, max_rolling: 1200 } }
    fn lock(&self) -> std::sync::MutexGuard<'_, Inner> { self.g.lock().unwrap_or_else(|e| e.into_inner()) }
    pub fn set_cmd_vel(&self, vx: f64, vy: f64, wz: f64) { self.lock().last_cmd = json!({"vx": rnd(vx, 3), "vy": rnd(vy, 3), "wz": rnd(wz, 3)}); }
    pub fn is_active(&self) -> bool { self.lock().active.is_some() }

    pub fn start_session(&self, stype: &str, metadata: Value) -> String {
        let mut g = self.lock();
        if g.active.is_some() { Self::finalize(&mut g, "AUTO_CLOSED", self.max_sessions); }
        let t = now();
        let sid = format!("{stype}_{}_{}", t as i64, randint(100, 999));
        let planned = metadata.get("planned_route").cloned().unwrap_or(json!({}));
        g.active = Some(Sess { head: json!({"id": sid, "type": stype, "start_time": t, "end_time": null, "duration_s": 0.0, "total_distance_m": 0.0, "status": "RECORDING",
            "metadata": metadata, "planned_route": planned}), frames: vec![] });
        sid
    }
    pub fn end_session(&self, status: &str) -> Option<Value> { let mut g = self.lock(); Self::finalize(&mut g, status, self.max_sessions) }

    fn finalize(g: &mut Inner, status: &str, max: usize) -> Option<Value> {
        let mut sess = g.active.take()?;
        let t = now();
        let frames: Vec<Value> = sess.frames.iter().filter_map(|f| serde_json::from_str(f).ok()).collect();
        let s = &mut sess.head;
        s["end_time"] = json!(t);
        s["status"] = json!(status);
        s["duration_s"] = jr(t - f(s, "start_time", t), 2);
        s["total_frames"] = json!(frames.len());
        let (d, mx, av, md) = stats(&frames);
        s["total_distance_m"] = json!(d); s["max_speed_mps"] = json!(mx); s["avg_speed_mps"] = json!(av); s["min_obstacle_dist_m"] = json!(md);
        let out = sess.full();
        g.completed.insert(0, sess);
        if g.completed.len() > max { g.completed.pop(); }
        Some(out)
    }

    pub fn capture_rolling(&self, seconds: f64, title: Option<&str>) -> Option<Value> {
        let mut g = self.lock();
        let t = now();
        let cutoff = t - seconds;
        let mut frames: Vec<Value> = g.rolling.iter().filter(|(ft, _)| *ft >= cutoff).filter_map(|(_, fr)| serde_json::from_str(fr).ok()).collect();
        if frames.is_empty() { return None; }
        let start = f(&frames[0], "t", t);
        for fr in frames.iter_mut() { fr["rel_t"] = jr(f(fr, "t", 0.0) - start, 2); }
        let (d, mx, av, md) = stats(&frames);
        let sid = format!("manual_slice_{}_{}", t as i64, randint(100, 999));
        let title = title.map(|s| s.to_string()).unwrap_or(format!("手动切片记录 (近 {} 秒)", seconds as i64));
        let head = json!({"id": sid, "type": "manual_slice", "start_time": start, "end_time": t, "duration_s": rnd(t - start, 2), "total_frames": frames.len(), "total_distance_m": d,
            "max_speed_mps": mx, "avg_speed_mps": av, "min_obstacle_dist_m": md, "status": "COMPLETED", "metadata": {"title": title, "source": "rolling_buffer"}, "planned_route": {}});
        let sess = Sess { head, frames: frames.iter().map(|f| f.to_string()).collect() };
        let out = sess.full();
        g.completed.insert(0, sess);
        if g.completed.len() > self.max_sessions { g.completed.pop(); }
        Some(out)
    }

    pub fn record_frame(&self, t: &Value, perf: Option<Value>) {
        let now_t = now();
        let mut g = self.lock();
        let cmd = g.last_cmd.clone();
        let raw = t.get("scan_ranges").and_then(|r| r.as_array()).cloned().unwrap_or_default();
        let ranges: Vec<Value> = if raw.is_empty() { vec![] } else { let step = (raw.len() / 36).max(1); raw.iter().step_by(step).take(36).map(|r| jr(r.as_f64().unwrap_or(0.0), 2)).collect() };
        let io = t.get("io_states").cloned().unwrap_or(json!({}));
        let b = |v: Option<&Value>, d: bool| v.and_then(|x| x.as_bool()).unwrap_or(d);
        let yaw = f(t, "yaw", 0.0);
        let frame = json!({"t": rnd(now_t, 3), "pose": {"x": rnd(f(t, "x", 0.0), 3), "y": rnd(f(t, "y", 0.0), 3), "yaw": rnd(yaw, 4), "yaw_deg": rnd(yaw.to_degrees(), 1)},
            "vel": {"vx": rnd(f(t, "vx", 0.0), 3), "vy": rnd(f(t, "vy", 0.0), 3), "wz": rnd(f(t, "wz", 0.0), 3)}, "cmd": cmd,
            "nav": {"status": t.get("nav_status").cloned().unwrap_or(json!("IDLE")), "dist_rem": t.get("nav_dist_rem").cloned().unwrap_or(json!(0.0))},
            "io": {"estop": b(io.get("is_emergency_stop"), false), "bumper": b(io.pointer("/inputs/di_bumper_front"), false), "cargo": b(io.pointer("/inputs/di_cargo_present"), false), "brake": b(io.pointer("/outputs/do_brake_release"), true)},
            "perception": {"min_dist": t.get("scan_min_dist").cloned().unwrap_or(json!(12.0)), "ranges": ranges, "markers": t.get("vision_markers").cloned().unwrap_or(json!([])), "obstacles": t.get("dynamic_obstacles").cloned().unwrap_or(json!([]))},
            "perf": perf.unwrap_or(json!({}))});
        g.rolling.push_back((now_t, frame.to_string()));
        if g.rolling.len() > self.max_rolling { g.rolling.pop_front(); }
        if let Some(a) = g.active.as_mut() {
            let mut fc = frame;
            fc["rel_t"] = jr(now_t - f(&a.head, "start_time", now_t), 2);
            a.frames.push(fc.to_string());
        }
    }

    pub fn session_list(&self) -> Vec<Value> {
        let g = self.lock();
        let mut res = vec![];
        let meta = |s: &Value, k: &str, d: Value| s["metadata"].get(k).cloned().unwrap_or(d);
        if let Some(sess) = &g.active {
            let (s, nfr) = (&sess.head, sess.frames.len());
            res.push(json!({"id": s["id"], "type": s["type"], "start_time": s["start_time"], "duration_s": rnd(now() - f(s, "start_time", 0.0), 1), "total_frames": nfr,
                "status": "RECORDING", "title": meta(s, "title", json!("实时录制中...")), "scenario": meta(s, "scenario", json!("default")), "origin": meta(s, "origin", json!({})), "destination": meta(s, "destination", json!({}))}));
        }
        for sess in &g.completed {
            let s = &sess.head;
            res.push(json!({"id": s["id"], "type": s["type"], "start_time": s["start_time"], "end_time": s["end_time"], "duration_s": s["duration_s"], "total_frames": s["total_frames"],
                "total_distance_m": s["total_distance_m"], "max_speed_mps": s.get("max_speed_mps").cloned().unwrap_or(json!(0.0)), "avg_speed_mps": s.get("avg_speed_mps").cloned().unwrap_or(json!(0.0)),
                "min_obstacle_dist_m": s.get("min_obstacle_dist_m").cloned().unwrap_or(Value::Null), "status": s["status"],
                "title": meta(s, "title", json!(format!("航段 {}", s["id"].as_str().unwrap_or("")))), "scenario": meta(s, "scenario", json!("default")), "origin": meta(s, "origin", json!({})), "destination": meta(s, "destination", json!({}))}));
        }
        res
    }

    pub fn get_session(&self, id: &str) -> Option<Value> {
        let g = self.lock();
        if let Some(a) = &g.active { if a.head["id"].as_str() == Some(id) { return Some(a.full()); } }
        for s in &g.completed { if s.head["id"].as_str() == Some(id) { return Some(s.full()); } }
        if id.is_empty() { return g.completed.first().map(|s| s.full()); }
        None
    }
}
