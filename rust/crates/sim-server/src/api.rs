use crate::service::*;
use rest::{ok, ApiError, HResult, Mount, Request, Response, Server};
use serde_json::{json, Map, Value};
use sim_core::cameras::Plane;
use sim_core::imgcodec;
use std::collections::HashMap;
use std::io::Write;
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::time::{Duration, Instant};

fn ascii_json(v: &Value) -> String {
    let s = serde_json::to_string(v).unwrap();
    let mut o = String::with_capacity(s.len());
    for c in s.chars() {
        if c.is_ascii() { o.push(c); } else {
            let mut b = [0u16; 2];
            for u in c.encode_utf16(&mut b) { o.push_str(&format!("\\u{:04x}", u)); }
        }
    }
    o
}

fn bin_headers(seq: u64, t: f64, meta: &Value) -> Vec<(String, String)> {
    vec![("X-Seq".into(), seq.to_string()), ("X-Stamp".into(), pyfloat(t)), ("X-Meta".into(), ascii_json(meta))]
}
fn pyfloat(x: f64) -> String { if x.fract() == 0.0 && x.abs() < 1e16 { format!("{:.1}", x) } else { format!("{}", x) } }

fn scan_json(s: &Scan) -> Value {
    match s {
        Scan::D2(d) => {
            let mut m = scan2d_meta(d);
            m["ranges"] = ranges_json(&d.ranges, 4);
            m
        }
        Scan::D3(d) => {
            let pts: Vec<Value> = d.f.xyz.iter().map(|v| json!(rnd(*v as f64, 4))).collect();
            json!({"type": "3d", "seq": d.seq, "t": d.t, "frame_id": d.frame_id, "pose": pose_json(&d.pose), "scan_hz": d.scan_hz, "points": pts, "point_fields": ["x", "y", "z"],
                "intensity": d.f.intensity.iter().map(|v| json!(*v as f64)).collect::<Vec<_>>(), "line": d.f.line.iter().map(|v| json!(*v)).collect::<Vec<_>>(),
                "slice": ranges_json(&d.f.slice, 4)})
        }
    }
}

fn scan_binary(s: &Scan) -> Response {
    match s {
        Scan::D2(d) => {
            let mut meta = scan2d_meta(d);
            meta["layout"] = json!("float32[ranges]");
            let mut body = Vec::with_capacity(4 * d.ranges.len());
            for r in d.ranges.iter() { body.extend_from_slice(&(*r as f32).to_le_bytes()); }
            Response::binary(body, bin_headers(d.seq, d.t, &meta), "application/octet-stream")
        }
        Scan::D3(d) => {
            let n = d.f.xyz.len() / 3;
            let mut meta = json!({"type": "3d", "seq": d.seq, "t": d.t, "frame_id": d.frame_id, "pose": pose_json(&d.pose), "scan_hz": d.scan_hz});
            meta["layout"] = json!("float32[N,4](x,y,z,intensity) + uint8[N](line)");
            meta["count"] = json!(n);
            let mut body = Vec::with_capacity(n * 16 + n);
            for k in 0..n {
                for c in 0..3 { body.extend_from_slice(&d.f.xyz[3 * k + c].to_le_bytes()); }
                body.extend_from_slice(&d.f.intensity.get(k).cloned().unwrap_or(0.0).to_le_bytes());
            }
            body.extend_from_slice(&d.f.line);
            Response::binary(body, bin_headers(d.seq, d.t, &meta), "application/octet-stream")
        }
    }
}

fn percentile99(a: &[u16]) -> f64 {
    if a.is_empty() { return 0.0; }
    let mut v: Vec<f64> = a.iter().map(|x| *x as f64).collect();
    v.sort_by(|x, y| x.partial_cmp(y).unwrap());
    let pos = 0.99 * (v.len() - 1) as f64;
    let (lo, hi) = (pos.floor() as usize, pos.ceil() as usize);
    v[lo] + (v[hi] - v[lo]) * (pos - lo as f64)
}

fn camera_body(d: &CamFrame, stream: Option<&str>, fmt: Option<&str>) -> HResult {
    let info = &d.info;
    let frames = &d.frames.streams;
    let stream = stream.map(|s| s.to_string()).unwrap_or_else(|| frames[0].0.clone());
    let Some((_, plane)) = frames.iter().find(|(n, _)| *n == stream) else {
        let names: Vec<&str> = frames.iter().map(|(n, _)| n.as_str()).collect();
        return Err(ApiError::new(400, format!("流 {stream} 不存在，可用: {names:?}")));
    };
    let (w, h) = (info["width"].as_u64().unwrap_or(0) as usize, info["height"].as_u64().unwrap_or(0) as usize);
    let mut meta = json!({"camera": info["name"], "type": info["type"], "stream": stream, "frame_id": info["frame_id"], "width": info["width"], "height": info["height"], "K": info["K"], "pose": pose_json(&d.pose)});
    if stream == "right" { meta["frame_id"] = json!(format!("{}_right_optical_frame", info["name"].as_str().unwrap_or(""))); }
    let (body, ctype): (Vec<u8>, &str);
    match (stream.as_str(), plane) {
        ("rgb" | "left" | "right", Plane::Rgb(a)) => match fmt.unwrap_or("jpeg") {
            "raw" => { meta["encoding"] = json!("rgb8"); body = a.clone(); ctype = "application/octet-stream"; }
            "png" => { body = imgcodec::png_rgb(w, h, a); ctype = "image/png"; }
            _ => { body = imgcodec::jpeg(w, h, a, 80); ctype = "image/jpeg"; }
        },
        ("depth", Plane::Depth(a)) => {
            let rng = info.get("range").and_then(|r| r.as_array()).map(|r| (r[0].as_f64().unwrap_or(0.1), r[1].as_f64().unwrap_or(10.0))).unwrap_or((0.1, 10.0));
            match fmt.unwrap_or("png") {
                "raw" => { meta["encoding"] = json!("32FC1"); body = a.iter().flat_map(|v| v.to_le_bytes()).collect(); ctype = "application/octet-stream"; }
                "jpeg" => { body = imgcodec::jpeg(w, h, &imgcodec::colorize_depth(a, rng.0, rng.1), 80); ctype = "image/jpeg"; }
                _ => {
                    let mm: Vec<u16> = a.iter().map(|v| { let x = v * 1000.0f32; if x.is_nan() { 0 } else { x.clamp(0.0, 65535.0) as u16 } }).collect();
                    meta["encoding"] = json!("16UC1 (mm)");
                    body = imgcodec::png_gray16(w, h, &mm); ctype = "image/png";
                }
            }
        }
        ("amplitude", Plane::Amp(a)) => match fmt.unwrap_or("png") {
            "raw" => { meta["encoding"] = json!("16UC1"); body = a.iter().flat_map(|v| v.to_le_bytes()).collect(); ctype = "application/octet-stream"; }
            "jpeg" => {
                let den = percentile99(a).max(1.0) as f32;
                let g: Vec<u8> = a.iter().map(|v| ((*v as f32 / den).clamp(0.0, 1.0) * 255.0) as u8).collect();
                let rgb: Vec<u8> = g.iter().flat_map(|v| [*v, *v, *v]).collect();
                body = imgcodec::jpeg(w, h, &rgb, 80); ctype = "image/jpeg";
            }
            _ => { body = imgcodec::png_gray16(w, h, a); ctype = "image/png"; }
        },
        ("points", Plane::Points(a)) => {
            meta["encoding"] = json!("float32[N,3] xyz"); meta["count"] = json!(a.len() / 3);
            body = a.iter().flat_map(|v| v.to_le_bytes()).collect(); ctype = "application/octet-stream";
        }
        _ => return Err(ApiError::new(400, format!("流 {stream} 不存在"))),
    }
    Ok(Response::binary(body, bin_headers(d.seq, d.t, &meta), ctype))
}

fn num(b: &Value, k: &str) -> Result<f64, ApiError> {
    match b.get(k) {
        None | Some(Value::Null) => Ok(0.0),
        Some(Value::Number(n)) => Ok(n.as_f64().unwrap_or(0.0)),
        Some(Value::String(s)) => s.trim().parse().map_err(|_| ApiError::new(400, "vx/vy/wz 必须为数值")),
        Some(Value::Bool(b)) => Ok(*b as i64 as f64),
        _ => Err(ApiError::new(400, "vx/vy/wz 必须为数值")),
    }
}
fn opt_f(b: &Value, k: &str) -> Option<f64> { b.get(k).and_then(|v| v.as_f64()) }
fn not_impl(what: &str) -> ApiError { ApiError::coded(501, format!("{what}: Rust 版本尚未实现 (Phase 1b)"), "not_implemented") }

pub fn build_api(svc: Arc<Service>, port: u16) -> Server {
    let mut api = Server::new("sim", port, "仿真进程 sim_server", "MuJoCo 物理仿真: 车型、场景、障碍物、传感器 (激光/相机/IMU/光电/触边/读码)、IO、速度指令、推送流");
    macro_rules! R { ($m:expr, $p:expr, $d:expr, $f:expr) => {{ let s = svc.clone(); #[allow(clippy::redundant_closure_call)] api.route($m, $p, $d, move |q: &Request| ($f)(&s, q)); }}; }

    R!("GET", "/api/v1/health", "健康检查", |s: &Arc<Service>, _q: &Request| { let g = s.lock(); ok(json!({"service": "sim", "api_version": API_VERSION, "ok": true, "sim_time": g.core.t, "rtf": g.core.rtf, "paused": g.core.paused})) });
    // ---- model
    R!("GET", "/api/v1/model", "机器人模型", |s: &Arc<Service>, _q: &Request| ok(s.model()));
    R!("GET", "/api/v1/model/urdf", "URDF", |s: &Arc<Service>, _q: &Request| -> HResult { Ok(Response::raw(s.urdf().into_bytes(), "application/xml; charset=utf-8")) });
    R!("GET", "/api/v1/model/editor", "补全编辑器数据: 完整度审计/传感器清单/当前覆盖", |s: &Arc<Service>, _q: &Request| ok(s.model_editor()));
    R!("GET", "/api/v1/model/audit", "完整度审计 (参数来源/缺失)", |s: &Arc<Service>, _q: &Request| ok(s.model_editor()["audit"].clone()));
    R!("GET", "/api/v1/model/overrides", "人工补全覆盖 (model_overrides.json)", |s: &Arc<Service>, _q: &Request| ok(s.lock().overrides.clone()));
    R!("PUT", "/api/v1/model/overrides", "保存并应用人工补全 (重建模型/URDF/仿真)", |s: &Arc<Service>, q: &Request| {
        let ov = q.json()?;
        if !ov.is_object() { return Err(ApiError::new(400, "需要 JSON 对象")); }
        ok(s.apply_overrides(&ov, true).map_err(|e| ApiError::new(400, format!("覆盖参数无效: {e}")))?)
    });
    R!("POST", "/api/v1/model/preview", "试算补全结果 (不保存): spec/审计/URDF", |s: &Arc<Service>, q: &Request| {
        let ov = q.json()?;
        if !ov.is_object() { return Err(ApiError::new(400, "覆盖参数无效: 需要 JSON 对象")); }
        ok(s.preview_overrides(&ov))
    });
    R!("GET", "/api/v1/model/sensor_template", "新增传感器默认参数 ?type=lidar2d|lidar3d|camera|stereo|tof|photoelectric|codeReader", |_s: &Arc<Service>, q: &Request| ok(model::overrides::sensor_template(&q.q_str("type", "camera")).map_err(|e| ApiError::new(400, e))?));
    R!("PUT", "/api/v1/model/chassis", "切换车型 {type: single_steer|diff_drive|dual_steer|cmodel}", |s: &Arc<Service>, q: &Request| {
        let b = q.json()?;
        let t = b.get("type").and_then(|t| t.as_str());
        if let Some(t) = t { if !["", "cmodel", "diff_drive", "single_steer", "dual_steer", "multi_steer"].contains(&t) { return Err(ApiError::new(400, format!("未知车型 {t}"))); } }
        let t = t.filter(|t| !t.is_empty() && *t != "cmodel");
        let c = s.set_chassis(t).map_err(|e| ApiError::new(400, e))?;
        ok(json!({"chassis": c}))
    });
    R!("POST", "/api/v1/model/reload", "重新解析 cmodel 并重建仿真 {cmodel_path, load: full|idle}", |s: &Arc<Service>, q: &Request| -> HResult {
        let b = q.json()?;
        if let Some(p) = b.get("cmodel_path").filter(|v| truthy(v)).and_then(|v| v.as_str()) {
            let load = b.get("load").and_then(|v| v.as_str()).unwrap_or("full");
            return ok(s.reload_model_from_cmodel(p, load, b.get("save").map(truthy).unwrap_or(true)).map_err(|e| ApiError::new(400, e))?);
        }
        let save = b.get("save").map(truthy).unwrap_or(true);
        ok(s.reload_model(save).map_err(|e| ApiError::new(400, e))?)
    });
    // ---- world
    R!("GET", "/api/v1/world", "场景", |s: &Arc<Service>, _q: &Request| ok(s.world().map_err(|e| ApiError::new(500, e))?));
    R!("PUT", "/api/v1/world/scenario", "切换场景 {id}", |s: &Arc<Service>, q: &Request| {
        let b = q.json()?;
        let sid = b.get("id").filter(|v| !v.is_null()).or_else(|| b.get("scenario")).and_then(|v| v.as_str()).map(|s| s.to_string());
        match sid { Some(id) if s.set_scenario(&id).is_ok() => ok(json!({"scenario": id})), other => Err(ApiError::new(404, format!("未知场景 {}", other.unwrap_or("None".into())))) }
    });
    R!("GET", "/api/v1/gpu", "GPU 射线求交服务 (gpucastd) 状态", |_s: &Arc<Service>, _q: &Request| ok(sim_core::gpu::info()));
    R!("GET", "/api/v1/world/obstacles", "动态障碍物", |s: &Arc<Service>, _q: &Request| ok(s.world().map_err(|e| ApiError::new(500, e))?["obstacles"].clone()));
    R!("POST", "/api/v1/scene/load", "加载场景定义 {scene} 或平台场景 {hub_scene_id}", |s: &Arc<Service>, q: &Request| {
        let b = q.json()?;
        if let Some(sc) = b.get("scene").filter(|v| truthy(v)) {
            let id = s.load_scene(sc).map_err(|e| ApiError::new(400, e))?;
            ok(json!({"scenario": id}))
        } else if let Some(sid) = b.get("hub_scene_id").filter(|v| truthy(v)).and_then(|v| v.as_str()) {
            let hub = b.get("hub").and_then(|h| h.as_str()).filter(|h| !h.is_empty()).map(|h| h.to_string()).unwrap_or_else(|| std::env::var("HUB_API").unwrap_or_default());
            if hub.is_empty() { return Err(ApiError::new(400, "未配置平台地址 HUB_API")); }
            let data = std::path::Path::new(&s.config_path).parent().map(|p| p.to_string_lossy().to_string()).unwrap_or_default();
            let p = crate::boot::pull_scene(hub.trim_end_matches('/'), &data, sid).map_err(|e| ApiError::new(502, e))?;
            let sc: Value = serde_json::from_str(&std::fs::read_to_string(p).map_err(|e| ApiError::new(500, e.to_string()))?).map_err(|e| ApiError::new(500, e.to_string()))?;
            let id = s.load_scene(&sc).map_err(|e| ApiError::new(400, e))?;
            ok(json!({"scenario": id}))
        } else { Err(ApiError::new(400, "需要 scene (场景定义) 或 hub_scene_id")) }
    });
    R!("POST", "/api/v1/model/load", "从平台加载模型 {model_id, version}", |s: &Arc<Service>, q: &Request| -> HResult {
        let b = q.json()?;
        let hub = b.get("hub").and_then(|h| h.as_str()).filter(|h| !h.is_empty()).map(|h| h.to_string()).unwrap_or_else(|| std::env::var("HUB_API").unwrap_or_default());
        let mid = b.get("model_id").and_then(|m| m.as_str()).unwrap_or("");
        if hub.is_empty() || mid.is_empty() { return Err(ApiError::new(400, "需要 model_id 且配置了平台地址 HUB_API")); }
        ok(s.load_model_from_hub(hub.trim_end_matches('/'), mid, b.get("version").and_then(|v| v.as_str()).unwrap_or("")).map_err(|e| ApiError::new(502, e))?)
    });
    R!("GET", "/api/v1/instance", "实例信息 (平台模型/场景)", |s: &Arc<Service>, _q: &Request| ok(s.instance()));
    R!("GET", "/api/v1/world/map", "Nav2 栅格地图 ?part=pgm|yaml&res=0.05", |s: &Arc<Service>, q: &Request| {
        let m = s.occupancy_map(q.q_f64("res", 0.05)?);
        if q.q_str("part", "pgm") == "yaml" { return Ok(Response::raw(m["yaml"].as_str().unwrap_or("").as_bytes().to_vec(), "text/yaml; charset=utf-8")); }
        let pgm: Vec<u8> = m["pgm"].as_array().map(|a| a.iter().map(|v| v.as_u64().unwrap_or(0) as u8).collect()).unwrap_or_default();
        let meta = json!({"id": m["id"], "width": m["width"], "height": m["height"], "resolution": m["resolution"], "origin": m["origin"]});
        Ok(Response::binary(pgm, vec![("X-Meta".into(), serde_json::to_string(&meta).unwrap())], "image/x-portable-graymap"))
    });
    R!("PUT", "/api/v1/world/obstacles", "设置动态障碍物 (请求体为数组，元素 {x, y, w, h, z, yaw, motion})", |s: &Arc<Service>, q: &Request| {
        let b = q.json()?;
        let Some(a) = b.as_array() else { return Err(ApiError::new(400, "需要障碍物数组")) };
        let obs: Vec<Map<String, Value>> = a.iter().map(|o| o.as_object().cloned().unwrap_or_default()).collect();
        let n = obs.len();
        s.set_obstacles(obs).map_err(|e| ApiError::new(400, e))?;
        ok(json!({"count": n}))
    });
    // ---- fork / cargo
    R!("GET", "/api/v1/fork", "货叉状态 (高度/速度/载货/受阻)", |s: &Arc<Service>, _q: &Request| ok(s.fork()));
    R!("PUT", "/api/v1/fork", "货叉控制 {height: m} 或 {action: up|down|stop|top|bottom}", |s: &Arc<Service>, q: &Request| {
        let b = q.json()?;
        ok(s.fork_cmd(opt_f(&b, "height"), b.get("action").and_then(|a| a.as_str())).map_err(|e| ApiError::new(400, e))?)
    });
    R!("GET", "/api/v1/world/cargo", "货物列表 (托盘/箱子)", |s: &Arc<Service>, _q: &Request| ok(s.cargo_list()));
    R!("POST", "/api/v1/world/cargo", "新增货物 {kind: pallet|box, id, x, y, yaw, z0?, size?: [长,宽,高]}", |s: &Arc<Service>, q: &Request| {
        let b = q.json()?;
        ok(s.cargo_add(&b).map_err(|e| ApiError::new(400, e))?)
    });
    R!("DELETE", "/api/v1/world/cargo/{id}", "移除货物", |s: &Arc<Service>, q: &Request| {
        s.cargo_remove(&q.params["id"]).map_err(|e| ApiError::new(404, e))?;
        ok(json!({"removed": q.params["id"]}))
    });
    // ---- sim control
    R!("GET", "/api/v1/sim", "仿真状态/性能", |s: &Arc<Service>, _q: &Request| ok(s.sim_status()));
    R!("PUT", "/api/v1/sim", "暂停/实时因子 {paused, rtf}", |s: &Arc<Service>, q: &Request| {
        let b = q.json()?;
        s.set_sim(b.get("paused").filter(|v| !v.is_null()).map(truthy), opt_f(&b, "rtf"));
        ok(s.sim_status())
    });
    R!("POST", "/api/v1/sim/reset", "位姿复位 {x, y, yaw}", |s: &Arc<Service>, q: &Request| {
        let b = q.json()?;
        s.reset(opt_f(&b, "x"), opt_f(&b, "y"), opt_f(&b, "yaw"));
        ok(s.state()["truth"].clone())
    });
    // ---- state & sensors
    R!("GET", "/api/v1/state", "真值/里程计/关节/碰撞", |s: &Arc<Service>, _q: &Request| ok(s.state()));
    R!("GET", "/api/v1/sensors", "传感器清单", |s: &Arc<Service>, _q: &Request| ok(s.sensors()));
    fn get_lidar(s: &Arc<Service>, q: &Request, name: &str) -> HResult {
        let d = s.wait_lidar(name, q.q_i64("after_seq", -1)?, q.q_f64("wait", 0.0)?).map_err(|_| ApiError::new(404, format!("无此激光 {name}")))?;
        let Some(d) = d else { return Err(ApiError::coded(503, "尚无数据", "no_data")) };
        if q.header("accept").map(|a| a.contains("octet-stream")).unwrap_or(false) || q.qs("format") == Some("bin") { return Ok(scan_binary(&d)); }
        ok(scan_json(&d))
    }
    R!("GET", "/api/v1/sensors/lidars/{name}", "激光帧 (长轮询/二进制) ?after_seq=N&wait=0.2", |s: &Arc<Service>, q: &Request| get_lidar(s, q, &q.params["name"]));
    R!("GET", "/api/v1/sensors/scan", "融合扫描", |s: &Arc<Service>, q: &Request| get_lidar(s, q, "merged"));
    R!("PUT", "/api/v1/sensors/lidar_config", "激光配置 {beams, range_max, freq_hz, sensor_resolution_deg}", |s: &Arc<Service>, q: &Request| ok(s.set_lidar_config(&q.json()?)));
    R!("GET", "/api/v1/sensors/cameras", "相机清单 (内参/流/帧序号)", |s: &Arc<Service>, _q: &Request| ok(json!({"cameras": s.cameras()})));
    R!("GET", "/api/v1/sensors/cameras/{name}", "相机帧 ?stream=rgb|left|right|depth|amplitude|points&format=jpeg|png|raw", |s: &Arc<Service>, q: &Request| {
        let name = &q.params["name"];
        let d = s.wait_camera(name, q.q_i64("after_seq", -1)?, q.q_f64("wait", 0.0)?).map_err(|_| ApiError::new(404, format!("无此相机 {name}")))?;
        let Some(d) = d else { return Err(ApiError::coded(503, "尚无图像", "no_data")) };
        camera_body(&d, q.qs("stream"), q.qs("format"))
    });
    R!("GET", "/api/v1/sensors/imu", "IMU", |s: &Arc<Service>, _q: &Request| ok(s.imu()));
    R!("GET", "/api/v1/sensors/codes", "读码/工位标签", |s: &Arc<Service>, _q: &Request| ok(s.codes()));
    R!("GET", "/api/v1/sensors/bumpers", "防撞触边 (碰撞条)", |s: &Arc<Service>, _q: &Request| ok(s.bumpers()));
    R!("GET", "/api/v1/sensors/photoelectric", "光电传感器", |s: &Arc<Service>, _q: &Request| ok(s.photoelectric()));
    // ---- io
    R!("GET", "/api/v1/io", "IO 状态", |s: &Arc<Service>, _q: &Request| ok(s.io()));
    R!("PUT", "/api/v1/io", "批量写 IO {di, do}", |s: &Arc<Service>, q: &Request| { let b = q.json()?; ok(s.set_io(b.get("di"), b.get("do"))) });
    R!("PUT", "/api/v1/io/{kind}/{name}", "写单个 IO {value}", |s: &Arc<Service>, q: &Request| {
        let (kind, name) = (&q.params["kind"], &q.params["name"]);
        let v = q.json()?.get("value").map(truthy).unwrap_or(false);
        let m = json!({ name.as_str(): v });
        match kind.as_str() { "di" => ok(s.set_io(Some(&m), None)), "do" => ok(s.set_io(None, Some(&m))), _ => Err(ApiError::new(400, "kind 必须为 di 或 do")) }
    });
    // ---- control & feedback
    R!("GET", "/api/v1/control", "当前指令/看门狗", |s: &Arc<Service>, _q: &Request| ok(s.control()));
    R!("PUT", "/api/v1/control/cmd_vel", "速度指令 (执行进程 → 仿真) {vx, vy, wz, source}", |s: &Arc<Service>, q: &Request| {
        let b = q.json()?;
        let src = b.get("source").map(|v| v.as_str().map(|s| s.to_string()).unwrap_or(v.to_string())).unwrap_or("nav".into());
        ok(s.set_cmd(num(&b, "vx")?, num(&b, "vy")?, num(&b, "wz")?, &src))
    });
    R!("GET", "/api/v1/nav/feedback", "导航回馈", |s: &Arc<Service>, _q: &Request| ok(s.get_nav_feedback()));
    R!("PUT", "/api/v1/nav/feedback", "执行进程回馈导航状态 (请求体为导航回馈对象，字段同 GET /api/v1/nav/feedback)", |s: &Arc<Service>, q: &Request| { s.set_nav_feedback(&q.json()?); ok(json!({"ok": true})) });
    R!("GET", "/api/v1/events", "仿真事件 ?since=N", |s: &Arc<Service>, q: &Request| ok(json!({"events": s.events_since(q.q_i64("since", 0)?)})));
    R!("GET", "/api/v1/snapshot", "Web 汇总快照 ?scans=1", |s: &Arc<Service>, q: &Request| ok(s.snapshot(q.q_bool("scans", false))));

    // ---- push stream
    let s = svc.clone();
    api.mount("/api/v1/stream", move |q: &Request, w: &mut std::net::TcpStream| {
        if q.path != "/api/v1/stream" { return Ok(Mount::NotHandled); }
        let fq = |k: &str, d: f64| q.qs(k).and_then(|v| v.parse::<f64>().ok()).unwrap_or(d);
        let hz = fq("hz", 50.0).clamp(1.0, 100.0);
        let io_hz = fq("io_hz", 20.0).clamp(0.0, 50.0);
        let scans = q.qs("scans") == Some("1");
        let hdr = "HTTP/1.1 200 OK\r\nContent-Type: application/x-agv-stream\r\nCache-Control: no-cache, no-store\r\nAccess-Control-Allow-Origin: *\r\nConnection: close\r\n\r\n";
        if w.write_all(hdr.as_bytes()).is_err() { return Ok(Mount::HandledClose); }
        let mut sent: HashMap<String, u64> = HashMap::new();
        let (mut meta_key, mut next_io, mut t_next) = (None::<String>, Instant::now(), Instant::now());
        while !s.stop.load(Ordering::Relaxed) {
            let (frame, meta, key) = s.state_frames(&meta_key);
            meta_key = key;
            let mut out = meta.unwrap_or_default();
            out.extend_from_slice(&frame);
            let now = Instant::now();
            if io_hz > 0.0 && now >= next_io { out.extend_from_slice(&s.io_frame()); next_io = now + Duration::from_secs_f64(1.0 / io_hz); }
            if scans { out.extend_from_slice(&s.scan_frames(&mut sent)); }
            if w.write_all(&out).is_err() { break; }
            t_next += Duration::from_secs_f64(1.0 / hz);
            let now = Instant::now();
            if t_next > now { std::thread::sleep(t_next - now); } else { t_next = now; }
        }
        Ok(Mount::HandledClose)
    });
    api
}
