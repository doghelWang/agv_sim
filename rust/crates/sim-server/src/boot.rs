//! Instance bootstrap (port of sim_server/bootstrap.py)
use model::overrides::{apply_overrides, save_overrides};
use rest::Client;
use serde_json::{json, Map, Value};
use std::path::Path;

pub fn log(m: &str) { eprintln!("[bootstrap] {m}"); }

pub fn fetch(url: &str, timeout: f64, tries: u32) -> Result<Vec<u8>, String> {
    let u = url.trim_start_matches("http://");
    let (hp, path) = u.split_once('/').map(|(a, b)| (a, format!("/{b}"))).unwrap_or((u, "/".into()));
    let c = Client::new(&format!("http://{hp}"), timeout);
    let mut last = String::new();
    for i in 0..tries {
        match c.request("GET", &path, None, Some(timeout), "*/*") {
            Ok(r) if r.status < 400 => return Ok(r.body),
            Ok(r) => last = format!("HTTP {}", r.status),
            Err(e) => last = e,
        }
        if i + 1 < tries { std::thread::sleep(std::time::Duration::from_secs(2)); }
    }
    Err(format!("下载 {url} 失败: {last}"))
}

fn write_json(p: &Path, v: &Value) -> Result<(), String> { std::fs::write(p, serde_json::to_string_pretty(v).unwrap()).map_err(|e| format!("{}: {e}", p.display())) }

pub fn init_defaults(data: &str) {
    let root = std::env::var("AGV_HOME").unwrap_or_else(|_| "/opt/agv".into());
    let d = if Path::new(&root).join("defaults").is_dir() { Path::new(&root).join("defaults") } else { Path::new(&root).to_path_buf() };
    for f in ["robot_config.json", "sensor_overrides.json", "robot.urdf"] {
        let (src, dst) = (d.join(f), Path::new(data).join(f));
        if !dst.exists() && src.exists() { let _ = std::fs::copy(&src, &dst); }
    }
    let cm = if d.join("cmodel").is_dir() { d.join("cmodel") } else { Path::new(&root).join("tests").join("data") };
    let dst = Path::new(data).join("cmodel");
    let empty = std::fs::read_dir(&dst).map(|mut r| r.next().is_none()).unwrap_or(true);
    if cm.is_dir() && empty {
        let _ = std::fs::create_dir_all(&dst);
        if let Ok(rd) = std::fs::read_dir(&cm) { for e in rd.flatten() { if e.path().is_file() { let _ = std::fs::copy(e.path(), dst.join(e.file_name())); } } }
    }
}

pub fn pull_model(hub: &str, data: &str, mid: &str, ver: &str) -> Result<(), String> {
    let ver = if ver.is_empty() { "latest" } else { ver };
    let b: Value = serde_json::from_slice(&fetch(&format!("{hub}/api/hub/models/{mid}/versions/{ver}/bundle"), 10.0, 15)?).map_err(|e| e.to_string())?;
    let d = Path::new(data);
    let ref_p = d.join(".model_ref");
    let sv = |k: &str| b.get(k).map(|v| v.as_str().map(|s| s.to_string()).unwrap_or(v.to_string())).unwrap_or_default();
    let rf = format!("{}@{}", sv("model_id"), sv("version"));
    let old = std::fs::read_to_string(&ref_p).map(|s| s.trim().to_string()).unwrap_or_default();
    let ov_p = d.join("model_overrides.json");
    write_json(&d.join("robot_config.base.json"), &b["base"])?;
    if old != rf || !ov_p.exists() {
        let ov = b.get("overrides").filter(|o| model::overrides::truthy(o)).cloned().unwrap_or(json!({"version": 1}));
        save_overrides(&ov, &ov_p.to_string_lossy())?;
        let _ = std::fs::remove_file(d.join("sensor_overrides.json"));
    }
    let ov: Value = serde_json::from_str(&std::fs::read_to_string(&ov_p).map_err(|e| e.to_string())?).map_err(|e| e.to_string())?;
    let mut spec = apply_overrides(&b["base"], &ov);
    let mut repo = Map::new();
    for k in ["model_id", "version", "name", "material_no", "vtype", "project"] { repo.insert(k.into(), b.get(k).cloned().unwrap_or(Value::Null)); }
    spec["repo"] = Value::Object(repo);
    write_json(&d.join("robot_config.json"), &spec)?;
    std::fs::write(d.join("robot.urdf"), model::urdf::generate_urdf(&spec)).map_err(|e| e.to_string())?;
    std::fs::write(&ref_p, &rf).map_err(|e| e.to_string())?;
    log(&format!("模型 {} {} 已就绪 (补全 {})", sv("name"), sv("version"), if old == rf { "沿用本地" } else { "来自平台" }));
    Ok(())
}

pub fn pull_scene(hub: &str, data: &str, sid: &str) -> Result<String, String> {
    let sd = Path::new(data).join("scene");
    std::fs::create_dir_all(&sd).map_err(|e| e.to_string())?;
    let mut sc: Value = serde_json::from_slice(&fetch(&format!("{hub}/api/hub/scenes/{sid}/definition"), 10.0, 15)?).map_err(|e| e.to_string())?;
    let maps = fetch(&format!("{hub}/api/hub/scenes/{sid}/map?part=pgm"), 60.0, 3).and_then(|pgm| {
        let yml = fetch(&format!("{hub}/api/hub/scenes/{sid}/map?part=yaml"), 10.0, 3)?;
        std::fs::write(sd.join("map.pgm"), pgm).map_err(|e| e.to_string())?;
        std::fs::write(sd.join("map.yaml"), yml).map_err(|e| e.to_string())
    });
    match maps {
        Ok(()) => { sc["map_files"] = json!({"pgm": sd.join("map.pgm").to_string_lossy(), "yaml": sd.join("map.yaml").to_string_lossy()}); }
        Err(e) => log(&format!("栅格地图下载失败 (仿真进程将按几何生成): {e}")),
    }
    let p = sd.join("scene.json");
    write_json(&p, &sc)?;
    log(&format!("场景 {} ({sid}) 已就绪", sc.get("name").and_then(|n| n.as_str()).unwrap_or("")));
    Ok(p.to_string_lossy().to_string())
}

/// `sim-server bootstrap`: prepare data dir and print shell exports; returns process exit code
pub fn run(builtin: &Map<String, Value>) -> i32 {
    let data = std::env::var("AGV_DATA").unwrap_or_else(|_| "/data".into());
    let _ = std::fs::create_dir_all(&data);
    init_defaults(&data);
    let hub = std::env::var("HUB_API").unwrap_or_default().trim_end_matches('/').to_string();
    let mut ex: Vec<(String, String)> = vec![("ROBOT_CONFIG".into(), format!("{data}/robot_config.json"))];
    let env = |k: &str| std::env::var(k).unwrap_or_default();
    if !hub.is_empty() && !env("MODEL_ID").is_empty() {
        if let Err(e) = pull_model(&hub, &data, &env("MODEL_ID"), &env("MODEL_VER")) { log(&format!("[错误] 从平台加载模型 {} 失败: {e}", env("MODEL_ID"))); return 3; }
    }
    if !hub.is_empty() && !env("SCENE_ID").is_empty() {
        let sid = env("SCENE_ID");
        match pull_scene(&hub, &data, &sid) {
            Ok(p) => ex.push(("SIM_SCENE_FILE".into(), p)),
            Err(e) => {
                if builtin.contains_key(&sid) { log(&format!("从平台加载场景失败，使用本地内置的同名场景 {sid}: {e}")); ex.push(("SIM_SCENARIO".into(), sid)); }
                else { log(&format!("[错误] 从平台加载场景 {sid} 失败: {e}")); return 3; }
            }
        }
    }
    for (k, v) in ex { println!("export {k}='{v}'"); }
    0
}
