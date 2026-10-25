mod events;
mod gw;
mod perf;
mod recorder;
mod util;
mod v2;

use gw::Gateway;
use rest::{ok, ApiError, HResult, Request, Response, Server};
use serde_json::{json, Map, Value};
use std::path::{Path, PathBuf};
use std::sync::Arc;
use util::*;

fn ctype(p: &Path) -> &'static str {
    match p.extension().and_then(|e| e.to_str()).unwrap_or("").to_lowercase().as_str() {
        "html" | "htm" => "text/html; charset=utf-8", "js" | "mjs" => "application/javascript; charset=utf-8", "css" => "text/css; charset=utf-8", "json" => "application/json; charset=utf-8",
        "png" => "image/png", "jpg" | "jpeg" => "image/jpeg", "gif" => "image/gif", "svg" => "image/svg+xml", "ico" => "image/x-icon", "woff2" => "font/woff2", "woff" => "font/woff",
        "txt" | "md" | "yaml" | "yml" => "text/plain; charset=utf-8", "pgm" => "image/x-portable-graymap", "map" => "application/json", "wasm" => "application/wasm", _ => "application/octet-stream",
    }
}
fn file(root: &Path, rel: &str, cache: &str) -> Option<Response> {
    let p: PathBuf = root.join(rel.trim_start_matches('/'));
    let p = p.canonicalize().ok()?;
    if !p.starts_with(root.canonicalize().ok()?) { return None; }
    let p = if p.is_dir() { p.join("index.html") } else { p };
    let data = std::fs::read(&p).ok()?;
    Some(Response::raw(data, ctype(&p)).header("Cache-Control", cache))
}
fn jresp(v: Value) -> Response { Response::json(&v) }
fn getf(b: &Value, k: &str, d: f64) -> f64 { b.get(k).and_then(|v| v.as_f64().or_else(|| v.as_str().and_then(|s| s.parse().ok()))).unwrap_or(d) }

fn proxy(gw: &Gateway, req: &Request) -> Option<Response> {
    let (client, rest) = if req.target.starts_with("/sim/api/v1") { (&gw.sim, &req.target[4..]) } else if req.target.starts_with("/nav/api/v1") { (&gw.nav, &req.target[4..]) } else { return None };
    let body: Option<Value> = if req.body.is_empty() { None } else { serde_json::from_slice(&req.body).ok() };
    let accept = req.header("accept").unwrap_or("application/json");
    let method = if req.method == "HEAD" { "GET" } else { req.method.as_str() };
    match client.request(method, rest, body.as_ref(), Some(30.0), accept) {
        Ok(r) => {
            let mut resp = Response::raw(r.body, r.headers.get("content-type").map(|s| s.as_str()).unwrap_or("application/json")).with_status(r.status);
            for k in ["x-seq", "x-stamp", "x-meta"] { if let Some(v) = r.headers.get(k) { resp = resp.header(k, v); } }
            Some(resp)
        }
        Err(e) => Some(jresp(json!({"error": {"code": "bad_gateway", "message": e}})).with_status(502)),
    }
}

fn handle(gw: &Arc<Gateway>, v2: &Arc<v2::V2>, perf: &Arc<perf::PerfMonitor>, root: &Path, req: &Request) -> HResult {
    gw.touch();
    let method = if req.method == "HEAD" { "GET" } else { req.method.as_str() };
    let path = req.path.as_str();
    if path.starts_with("/api/v2/") || path == "/api/v2" { return Ok(v2.handle(req, method)); }
    if method != "GET" && !path.starts_with("/api/events") && !path.starts_with("/api/replay") { if let Some(r) = v2.guard_legacy(req) { return Ok(r); } }
    if let Some(r) = proxy(gw, req) { return Ok(r); }
    match method {
        "GET" => {
            let nc = "no-cache, no-store, must-revalidate";
            match path {
                "/model" | "/model_editor.html" => return Ok(file(root, "model_editor.html", nc).unwrap_or_else(|| Response::raw(b"not found".to_vec(), "text/plain").with_status(404))),
                "/" | "/index.html" => return Ok(file(root, "index.html", nc).unwrap_or_else(|| Response::raw(b"not found".to_vec(), "text/plain").with_status(404))),
                "/annotator.js" => return Ok(file(root, "annotator.js", "no-cache, no-store").unwrap_or_else(|| Response::raw(vec![], "text/plain").with_status(404))),
                "/api/telemetry" => {
                    let full = matches!(req.qs("full"), Some("1" | "true"));
                    let scans = matches!(req.qs("scans"), Some("1" | "true"));
                    return Ok(Response::raw(gw.telemetry_json(full, scans), "application/json"));
                }
                "/api/events" => {
                    let since = req.qs("since_id").and_then(|v| v.parse::<u64>().ok()).unwrap_or(0);
                    let cats = req.qs("categories").filter(|c| !c.is_empty()).map(|c| c.split(',').map(|s| s.to_string()).collect());
                    let limit = req.qs("limit").and_then(|v| v.parse::<usize>().ok()).unwrap_or(100);
                    return ok(gw.events.get_events(since, cats, limit));
                }
                "/api/system_perf" => return ok(perf.get_stats()),
                "/api/bullet_metrics" => {
                    let bm = gw.lock().telemetry.get("bullet_metrics").filter(|v| truthy(v)).cloned().or_else(|| perf.latest().get("bullet_simulation").cloned()).unwrap_or(json!({}));
                    return ok(bm);
                }
                "/api/lidar_config" => return ok(gw.lock().lidar_config.clone()),
                "/api/replay/sessions" => return ok(json!({"sessions": gw.rec.session_list(), "is_recording": gw.rec.is_active()})),
                "/api/replay/session" => {
                    let s = gw.rec.get_session(&req.q_str("id", ""));
                    return Ok(jresp(s.clone().unwrap_or(json!({"error": "session not found"}))).with_status(if s.is_some() { 200 } else { 404 }));
                }
                "/api/replay/export" => {
                    return Ok(match gw.rec.get_session(&req.q_str("id", "")) {
                        Some(s) => {
                            let id = s["id"].as_str().unwrap_or("").to_string();
                            Response::raw(serde_json::to_vec_pretty(&json!({"version": "AMR_FLIGHT_RECORDER_V4", "export_time": now(), "session": s})).unwrap_or_default(), "application/json; charset=utf-8")
                                .header("Content-Disposition", &format!("attachment; filename=\"amr_flight_log_{id}.json\""))
                        }
                        None => jresp(json!({"error": "Session not found"})).with_status(404),
                    });
                }
                _ => {}
            }
            if path.starts_with("/vendor/") { return Ok(file(root, path, "public, max-age=86400").unwrap_or_else(|| Response::raw(vec![], "text/plain").with_status(404))); }
            Ok(file(root, path, "no-cache").unwrap_or_else(|| Response::raw(b"File not found".to_vec(), "text/plain; charset=utf-8").with_status(404)))
        }
        "POST" => {
            let b = req.json().unwrap_or(json!({}));
            let b = if b.is_object() { b } else { json!({}) };
            let mut res = json!({"status": "ok"});
            match path {
                "/api/cmd_vel" => gw.publish_cmd_vel(getf(&b, "vx", 0.0), getf(&b, "vy", 0.0), getf(&b, "wz", 0.0)),
                "/api/navigate_to_pose" | "/api/navigate" => gw.send_nav_goal(getf(&b, "x", 0.0), getf(&b, "y", 0.0), getf(&b, "yaw", 0.0)),
                "/api/map_scenario" => gw.set_map_scenario(b.get("scenario").and_then(|s| s.as_str()).unwrap_or("grid_9_square")),
                "/api/cancel_navigation" => gw.cancel_nav(),
                "/api/sim_pause" => { let p = b.get("paused").map(truthy).unwrap_or(true); gw.set_simulation_pause(p); res = json!({"status": "ok", "paused": p}); }
                "/api/sim_resume" => { gw.set_simulation_pause(false); res = json!({"status": "ok", "paused": false}); }
                "/api/sim_reset" => { gw.reset_simulation(); res = json!({"status": "ok", "reset": true}); }
                "/api/chassis_type" => gw.set_chassis_type(b.get("type").and_then(|s| s.as_str()).unwrap_or("diff_drive")),
                "/api/planner_type" => gw.set_planner_type(b.get("type").and_then(|s| s.as_str()).unwrap_or("dijkstra")),
                "/api/lidar_config" => {
                    let pick = |keys: &[&str]| keys.iter().filter_map(|k| b.get(*k)).find(|v| truthy(v)).and_then(|v| v.as_f64().or_else(|| v.as_str().and_then(|s| s.parse().ok())));
                    let mut body = Map::new();
                    if let Some(v) = b.get("beams").filter(|v| !v.is_null()) { body.insert("beams".into(), json!(v.as_f64().unwrap_or(0.0) as i64)); }
                    if let Some(v) = pick(&["angle_resolution_deg", "resolution"]) { body.insert("angle_resolution_deg".into(), json!(v)); }
                    if let Some(v) = pick(&["freq_hz", "freq", "rate"]) { body.insert("freq_hz".into(), json!(v)); }
                    if let Some(v) = b.get("range_max").filter(|v| !v.is_null()) { body.insert("range_max".into(), json!(v.as_f64().unwrap_or(0.0))); }
                    res = json!({"status": "ok", "config": gw.set_lidar_config(body)});
                }
                "/api/set_io" => gw.set_io(b.get("key").and_then(|s| s.as_str()).unwrap_or(""), b.get("value").map(truthy).unwrap_or(false)),
                "/api/obstacles/random" => gw.generate_random_obstacles(getf(&b, "count", 3.0) as usize),
                "/api/obstacles/clear" => gw.clear_obstacles(),
                "/api/fork" => res = gw.fork_cmd(&b),
                "/api/cargo/add" => match gw.cargo_add(&b) { Ok(v) => res = v, Err(m) => return Ok(jresp(json!({"status": "error", "message": m})).with_status(400)) },
                "/api/fork/demo" => match gw.fork_demo(&b) { Ok(v) => res = json!({"status": "ok", "flow": v}), Err(m) => return Ok(jresp(json!({"status": "error", "message": m})).with_status(400)) },
                "/api/fork/pick" => match gw.fork_pick(&b) { Ok(v) => res = json!({"status": "ok", "flow": v}), Err(m) => return Ok(jresp(json!({"status": "error", "message": m})).with_status(400)) },
                "/api/fork/place" => match gw.fork_place(&b) { Ok(v) => res = json!({"status": "ok", "flow": v}), Err(m) => return Ok(jresp(json!({"status": "error", "message": m})).with_status(400)) },
                "/api/fork/stop_flow" => { gw.fork_stop_flow(); res = json!({"status": "ok"}); }
                "/api/cargo/clear" => { let n = gw.cargo_clear(); res = json!({"status": "ok", "removed": n}); }
                "/api/obstacles/add" => {
                    match gw.add_obstacle(getf(&b, "x", 0.0), getf(&b, "y", 0.0), getf(&b, "w", 0.8), getf(&b, "h", 0.8), b.get("type").and_then(|s| s.as_str()).unwrap_or("box")) {
                        Ok(()) => res = json!({"status": "ok", "message": "扰动模块已添加，将在恢复仿真时生效"}),
                        Err(m) => return Ok(jresp(json!({"status": "error", "message": m})).with_status(400)),
                    }
                }
                "/api/events/clear" => res = gw.events.clear(),
                "/api/events/inject" => {
                    let g = |k: &str, d: &str| b.get(k).and_then(|v| v.as_str()).unwrap_or(d).to_string();
                    let ev = gw.events.emit(&g("category", "system"), &g("type", "MANUAL_TEST"), &g("level", "info"), &g("title", "测试注入事件"), &g("message", "用户从控制台手动注入测试事件"), b.get("payload").cloned().unwrap_or(json!({})));
                    res = json!({"status": "ok", "event": ev});
                }
                "/api/replay/record" => match b.get("action").and_then(|a| a.as_str()).unwrap_or("start") {
                    "start" => {
                        let title = b.get("title").and_then(|t| t.as_str()).map(|t| t.to_string()).unwrap_or(format!("手动录制会话 {}", fmt_local(now(), "%H:%M:%S")));
                        let sc = gw.lock().active_scenario.clone();
                        let sid = gw.rec.start_session("manual", json!({"title": title, "scenario": sc}));
                        res = json!({"status": "ok", "session_id": sid, "action": "started"});
                    }
                    "stop" => res = json!({"status": "ok", "session": gw.rec.end_session("MANUAL_STOPPED"), "action": "stopped"}),
                    "capture_rolling" => res = json!({"status": "ok", "session": gw.rec.capture_rolling(getf(&b, "seconds", 30.0), b.get("title").and_then(|t| t.as_str())), "action": "captured"}),
                    _ => {}
                },
                _ => {}
            }
            Ok(jresp(res))
        }
        _ => Ok(Response::raw(vec![], "text/plain").with_status(404)),
    }
}

fn main() {
    let env = |k: &str, d: &str| std::env::var(k).ok().filter(|v| !v.is_empty()).unwrap_or(d.to_string());
    let sim_api = env("SIM_API", "http://127.0.0.1:8090");
    let nav_api = env("NAV_API", "http://127.0.0.1:8091");
    let port: u16 = env("WEB_PORT", "8088").parse().expect("WEB_PORT");
    let root = PathBuf::from(std::env::var("AGV_HOME").unwrap_or_else(|_| ".".into()));
    let events = Arc::new(events::EventHub::new(1200));
    let rec = Arc::new(recorder::FlightRecorder::new());
    let gw = Gateway::new(&sim_api, &nav_api, events, rec);
    let g2 = gw.clone();
    let perf = perf::PerfMonitor::start(Arc::new(move || g2.is_idle()));
    let _ = gw.perf.set(perf.clone());
    gw.start();
    let v2 = v2::V2::new(gw.clone());
    let mut srv = Server::new("web-gateway", port, "Web 网关", "聚合仿真进程与执行进程 REST 的前端网关");
    srv.host = env("WEB_BIND", "0.0.0.0");
    let (g3, root2) = (gw.clone(), root.clone());
    srv.catch_all = Some(Arc::new(move |q: &Request| -> Result<Response, ApiError> { handle(&g3, &v2, &perf, &root2, q) }));
    println!("[web_gateway] http://{}:{port}  ←→ 仿真进程 {sim_api} / 执行进程 {nav_api}", srv.host);
    use std::io::Write;
    let _ = std::io::stdout().flush();
    if let Err(e) = Arc::new(srv).serve_forever() { eprintln!("[web_gateway] 监听失败: {e}"); std::process::exit(1); }
}
