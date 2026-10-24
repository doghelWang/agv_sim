//! Platform operations: update packages (agv-update/1), APK store/adb install, jobs, logs, package upload, SSH node add.
use crate::store::*;
use crate::{Hub, user};
use rest::{Request, ApiError};
use serde_json::{json, Map, Value};
use sha2::{Digest, Sha256};
use std::io::Read;
use std::path::{Path, PathBuf};
use std::sync::Arc;

const FORMAT: &str = "agv-update/1";
const MANIFEST: &str = "agv_update.json";
const VERSION_FILE: &str = ".agv_version";
const PROTECTED: [&str; 6] = ["data/", "records/", "robot_config.json", "model_overrides.json", ".git/", VERSION_FILE];
const NO_EFFECT: [&str; 5] = ["docs/", "tests/", "tools/", "README", ".github/"];
const MAX_UPDATE: usize = 400 << 20;
const MAX_APK: usize = 200 << 20;

fn sha256(b: &[u8]) -> String { hex(&Sha256::digest(b)) }
fn log(m: &str) { println!("[hub][admin] {m}"); }

pub fn safe_rel(p: &str) -> R<String> {
    let p = p.replace('\\', "/");
    let parts: Vec<&str> = p.trim_start_matches('/').split('/').filter(|x| !x.is_empty() && *x != ".").collect();
    if parts.is_empty() || parts.contains(&"..") { return Err(err(400, format!("更新包里有非法路径: {p:?}"))); }
    Ok(parts.join("/"))
}
fn protected(rel: &str) -> bool { PROTECTED.iter().any(|x| rel == x.trim_end_matches('/') || rel.starts_with(x)) }

/// Which services a changed file affects (Rust layout: bin/<name>, web/, ...)
fn components(rel: &str) -> Vec<&'static str> {
    if NO_EFFECT.iter().any(|x| rel.starts_with(x)) || rel.ends_with(".md") { return vec![]; }
    if rel.starts_with("web/") || rel == "index.html" || rel == "model_editor.html" { return vec!["web"]; }
    if rel == "bin/hub" { return vec!["hub"]; }
    if rel == "bin/agent" { return vec!["agent"]; }
    if rel.starts_with("deploy/") || rel.starts_with("docker/") && !rel.ends_with(".sh") { return vec![]; }
    vec!["instances"]
}
fn comp_label(c: &str) -> &'static str {
    match c { "web" => "网页 (刷新浏览器即可)", "hub" => "平台 (自动重启，约 10 秒不可用)", "agent" => "节点代理 (运行中的实例会先停止，代理重启后重新部署)", "instances" => "运行中的仿真实例 (重新部署)", _ => "" }
}

// -------------------------------------------------------------- tar helpers
fn open_tar(data: &[u8]) -> Result<Vec<u8>, String> {
    if data.len() > 2 && data[0] == 0x1f && data[1] == 0x8b {
        let mut out = Vec::new();
        flate2::read::GzDecoder::new(data).read_to_end(&mut out).map_err(|e| e.to_string())?;
        Ok(out)
    } else { Ok(data.to_vec()) }
}
/// path → (content, mode) for all regular files
fn tar_files(data: &[u8]) -> Result<Vec<(String, Vec<u8>, u32)>, String> {
    let raw = open_tar(data)?;
    let mut ar = tar::Archive::new(&raw[..]);
    let mut out = vec![];
    for e in ar.entries().map_err(|e| e.to_string())? {
        let mut e = e.map_err(|e| e.to_string())?;
        if !e.header().entry_type().is_file() { continue; }
        let p = e.path().map_err(|e| e.to_string())?.to_string_lossy().trim_start_matches("./").to_string();
        let mode = e.header().mode().unwrap_or(0o644);
        let mut b = Vec::new();
        e.read_to_end(&mut b).map_err(|e| e.to_string())?;
        out.push((p, b, mode));
    }
    Ok(out)
}

// -------------------------------------------------------------- jobs
fn persist_job(h: &Hub, j: &Value) {
    let p = h.admin.dir.join("jobs").join(format!("{}.json", sget(j, "id")));
    let _ = std::fs::write(p, serde_json::to_vec(j).unwrap());
}
fn jlog(h: &Hub, msg: &str, level: &str) {
    log(msg);
    let mut g = h.admin.job.lock().unwrap();
    if let Some(j) = g.as_mut() {
        let l = j["log"].as_array_mut().unwrap();
        l.push(json!({"t": now(), "level": level, "msg": msg}));
        if l.len() > 800 { let n = l.len() - 800; l.drain(..n); }
        persist_job(h, j);
    }
}
fn end_job(h: &Hub, ok: bool, msg: &str) {
    if !msg.is_empty() { jlog(h, msg, if ok { "ok" } else { "error" }); }
    let mut g = h.admin.job.lock().unwrap();
    if let Some(j) = g.as_mut() { j["status"] = json!(if ok { "done" } else { "failed" }); j["ended"] = json!(now()); persist_job(h, j); }
}
fn new_job(h: &Hub, kind: &str, title: &str, by: &str, rf: &str) -> R<Value> {
    let mut g = h.admin.job.lock().unwrap();
    if let Some(j) = g.as_ref() { if sget(j, "status") == "running" { return Err(err(409, format!("已有任务在执行: {}", sget(j, "title")))); } }
    let id = format!("j{}-{}", crate::unix_stamp(), &hex(&rand_bytes(2)));
    let j = json!({"id": id, "kind": kind, "title": title, "by": by, "ref": rf, "status": "running", "started": now(), "ended": null, "log": []});
    persist_job(h, &j);
    *g = Some(j.clone());
    Ok(j)
}
pub fn jobs(h: &Hub) -> Value {
    let cur = h.admin.job.lock().unwrap().clone();
    let d = h.admin.dir.join("jobs");
    let mut fs: Vec<String> = std::fs::read_dir(&d).map(|r| r.filter_map(|e| e.ok()).map(|e| e.file_name().to_string_lossy().to_string()).collect()).unwrap_or_default();
    fs.sort(); fs.reverse();
    let mut out = vec![];
    for f in fs.into_iter().take(30) {
        let Some(mut j) = Store::read_json(&d.join(&f)) else { continue };
        if sget(&j, "status") == "running" && cur.as_ref().map(|c| sget(c, "id") != sget(&j, "id")).unwrap_or(true) { j["status"] = json!("interrupted"); }
        let mut m = Map::new();
        for k in ["id", "kind", "title", "by", "status", "started", "ended"] { m.insert(k.into(), j.get(k).cloned().unwrap_or(Value::Null)); }
        out.push(Value::Object(m));
    }
    json!({"jobs": out, "current": cur.as_ref().map(|c| c["id"].clone())})
}
pub fn job_view(h: &Hub, jid: &str) -> R<Value> {
    if let Some(c) = h.admin.job.lock().unwrap().as_ref() { if sget(c, "id") == jid { return Ok(c.clone()); } }
    let mut j = Store::read_json(&h.admin.dir.join("jobs").join(format!("{}.json", safe_rel(jid)?))).ok_or_else(|| err(404, "任务不存在"))?;
    if sget(&j, "status") == "running" { j["status"] = json!("interrupted"); }
    Ok(j)
}

// -------------------------------------------------------------- update packages
fn upd_dir(h: &Hub, uid: &str) -> PathBuf { h.admin.dir.join("updates").join(uid) }
fn load_upd(h: &Hub, uid: &str) -> R<Value> {
    if uid.contains('/') || uid.contains("..") { return Err(err(404, format!("更新包 {uid} 不存在"))); }
    Store::read_json(&upd_dir(h, uid).join("meta.json")).ok_or_else(|| err(404, format!("更新包 {uid} 不存在")))
}
fn save_upd(h: &Hub, m: &Value) { h.s.write_json(&upd_dir(h, sget(m, "id")).join("meta.json"), m); }

pub fn updates(h: &Hub) -> Vec<Value> {
    let mut out = vec![];
    for e in std::fs::read_dir(h.admin.dir.join("updates")).into_iter().flatten().flatten() {
        let Ok(m) = load_upd(h, &e.file_name().to_string_lossy()) else { continue };
        let mut o = Map::new();
        for k in ["id", "version", "title", "base", "created", "uploaded", "uploaded_by", "status", "applied_at", "rolled_back_at", "counts", "components", "size"] { o.insert(k.into(), m.get(k).cloned().unwrap_or(Value::Null)); }
        out.push(Value::Object(o));
    }
    out.sort_by(|a, b| b.get("uploaded").and_then(|v| v.as_f64()).unwrap_or(0.0).partial_cmp(&a.get("uploaded").and_then(|v| v.as_f64()).unwrap_or(0.0)).unwrap());
    out
}
fn last_applied(h: &Hub) -> Option<String> {
    updates(h).into_iter().filter(|u| sget(u, "status") == "applied").max_by(|a, b| a.get("applied_at").and_then(|v| v.as_f64()).unwrap_or(0.0).partial_cmp(&b.get("applied_at").and_then(|v| v.as_f64()).unwrap_or(0.0)).unwrap()).map(|u| sget(&u, "id").to_string())
}

pub fn upload(h: &Hub, data: &[u8], by: &str) -> R<Value> {
    if data.is_empty() { return Err(err(400, "请求体为空")); }
    if data.len() > MAX_UPDATE { return Err(err(413, "更新包过大")); }
    let files = tar_files(data).map_err(|_| err(400, "不是有效的更新包 (应为 .tgz，含 agv_update.json 与 files/)"))?;
    let man_raw = files.iter().find(|f| f.0 == MANIFEST).ok_or_else(|| err(400, format!("更新包缺少 {MANIFEST}")))?;
    let man: Value = serde_json::from_slice(&man_raw.1).map_err(|_| err(400, "更新清单不是有效 JSON"))?;
    if sget(&man, "format") != FORMAT { return Err(err(400, format!("更新包格式不支持: {}", sget(&man, "format")))); }
    let mut out = vec![];
    let mut blobs: Vec<(String, &Vec<u8>)> = vec![];
    for f in man.get("files").and_then(|f| f.as_array()).cloned().unwrap_or_default() {
        let rel = safe_rel(sget(&f, "path"))?;
        let ent = files.iter().find(|e| e.0 == format!("files/{rel}")).ok_or_else(|| err(400, format!("更新包缺少文件 {rel}")))?;
        if sha256(&ent.1) != sget(&f, "sha256") { return Err(err(400, format!("文件校验失败: {rel} (更新包损坏)"))); }
        let mode = f.get("mode").and_then(|m| m.as_u64()).unwrap_or(ent.2 as u64);
        out.push(json!({"path": rel, "sha256": sget(&f, "sha256"), "size": ent.1.len(), "mode": mode}));
        blobs.push((rel, &ent.1));
    }
    let mut dels = vec![];
    for p in man.get("delete").and_then(|d| d.as_array()).cloned().unwrap_or_default() { dels.push(safe_rel(p.as_str().unwrap_or(""))?); }
    let uid = format!("u{}-{}", crate::unix_stamp(), hex(&rand_bytes(2)));
    let d = upd_dir(h, &uid);
    let _ = std::fs::create_dir_all(&d);
    std::fs::write(d.join("pkg.tgz"), data).map_err(|e| err(500, e.to_string()))?;
    let s = |k: &str| man.get(k).and_then(|v| v.as_str()).unwrap_or("").chars().take(if k == "title" { 200 } else { 40 }).collect::<String>();
    let mut m = json!({"id": uid, "version": if s("version").is_empty() { uid.clone() } else { s("version") }, "title": s("title"), "base": man.get("base").cloned().unwrap_or(Value::Null),
        "created": man.get("created").cloned().unwrap_or(Value::Null), "uploaded": now(), "uploaded_by": by, "files": out, "delete": dels, "status": "uploaded", "size": data.len()});
    let pl = plan(h, &m);
    for (k, v) in pl.as_object().unwrap() { m[k] = v.clone(); }
    save_upd(h, &m);
    log(&format!("收到更新包 {uid} 版本 {} ({} 个文件)", sget(&m, "version"), m["files"].as_array().map(|a| a.len()).unwrap_or(0)));
    view(h, &uid)
}

fn plan(h: &Hub, m: &Value) -> Value {
    let (mut rows, mut comps) = (vec![], std::collections::BTreeSet::new());
    for f in m["files"].as_array().unwrap_or(&vec![]) {
        let rel = sget(f, "path");
        let p = h.root.join(rel);
        let st = if protected(rel) { "protected" } else if !p.exists() { "new" } else if std::fs::read(&p).map(|b| sha256(&b) == sget(f, "sha256")).unwrap_or(false) { "same" } else { "changed" };
        rows.push(json!({"path": rel, "status": st, "size": f["size"]}));
        if st == "new" || st == "changed" { for c in components(rel) { comps.insert(c); } }
    }
    for r in m["delete"].as_array().unwrap_or(&vec![]) {
        let rel = r.as_str().unwrap_or("");
        let st = if protected(rel) { "protected" } else if h.root.join(rel).exists() { "delete" } else { "absent" };
        rows.push(json!({"path": rel, "status": st, "size": 0}));
        if st == "delete" { for c in components(rel) { comps.insert(c); } }
    }
    let mut counts = Map::new();
    for r in &rows { let k = sget(r, "status").to_string(); let n = counts.get(&k).and_then(|v| v.as_i64()).unwrap_or(0); counts.insert(k, json!(n + 1)); }
    json!({"plan": rows, "counts": counts, "components": comps.into_iter().collect::<Vec<_>>()})
}

pub fn view(h: &Hub, uid: &str) -> R<Value> {
    let mut m = load_upd(h, uid)?;
    if sget(&m, "status") == "uploaded" { let p = plan(h, &m); for (k, v) in p.as_object().unwrap() { m[k] = v.clone(); } }
    let cur = h.admin.version()["version"].as_str().unwrap_or("").to_string();
    let comps: Vec<String> = m["components"].as_array().unwrap_or(&vec![]).iter().filter_map(|c| c.as_str().map(|s| s.to_string())).collect();
    let mut v = m.clone();
    v.as_object_mut().unwrap().remove("files");
    v["current"] = json!(cur);
    v["base_mismatch"] = json!(truthy(m.get("base")) && m["base"].as_str() != Some(&cur) && sget(&m, "status") == "uploaded");
    v["builds"] = json!([]);
    v["effects"] = json!(comps.iter().map(|c| comp_label(c)).filter(|l| !l.is_empty()).collect::<Vec<_>>());
    v["running_instances"] = json!(h.dep.list(false).iter().filter(|i| matches!(sget(i, "status"), "running" | "degraded" | "deploying")).map(|i| i["id"].clone()).collect::<Vec<_>>());
    v["can_rollback"] = json!(last_applied(h).as_deref() == Some(uid));
    Ok(v)
}
pub fn delete_update(h: &Hub, uid: &str) -> R<Value> {
    let m = load_upd(h, uid)?;
    if matches!(sget(&m, "status"), "applying" | "rolling_back") { return Err(err(409, "更新正在进行")); }
    if sget(&m, "status") == "applied" && last_applied(h).as_deref() == Some(uid) { return Err(err(409, "这是当前生效的更新，删除后无法回滚；请先回滚或等下一次更新后再删")); }
    let _ = std::fs::remove_dir_all(upd_dir(h, uid));
    let _ = std::fs::remove_dir_all(h.admin.dir.join("backups").join(uid));
    Ok(json!({"ok": true}))
}

fn write_file(root: &Path, rel: &str, data: &[u8], mode: u32) -> std::io::Result<()> {
    let p = root.join(rel);
    if let Some(d) = p.parent() { std::fs::create_dir_all(d)?; }
    let tmp = PathBuf::from(format!("{}.agvtmp", p.display()));
    std::fs::write(&tmp, data)?;
    #[cfg(unix)] { use std::os::unix::fs::PermissionsExt; std::fs::set_permissions(&tmp, std::fs::Permissions::from_mode(if mode & 0o777 == 0 { 0o644 } else { mode & 0o777 }))?; }
    std::fs::rename(&tmp, &p)   // atomic replace works while the old binary is running
}

pub fn apply(h: &Arc<Hub>, uid: &str, opts: &Value, by: &str) -> R<Value> {
    let m = load_upd(h, uid)?;
    if sget(&m, "status") != "uploaded" { return Err(err(409, format!("该更新包状态为 {}，不能再次应用 (可重新上传)", sget(&m, "status")))); }
    let prev = last_applied(h);
    let j = new_job(h, "apply", &format!("应用更新 {}", sget(&m, "version")), by, uid)?;
    let (h2, uid2, opts2) = (h.clone(), uid.to_string(), opts.clone());
    std::thread::Builder::new().name("admin-apply".into()).spawn(move || run_apply(&h2, &uid2, prev, &opts2)).ok();
    Ok(j)
}
pub fn rollback(h: &Arc<Hub>, uid: &str, opts: &Value, by: &str) -> R<Value> {
    let m = load_upd(h, uid)?;
    if last_applied(h).as_deref() != Some(uid) { return Err(err(409, "只能回滚最近一次应用的更新")); }
    let j = new_job(h, "rollback", &format!("回滚更新 {}", sget(&m, "version")), by, uid)?;
    let (h2, uid2, opts2) = (h.clone(), uid.to_string(), opts.clone());
    std::thread::Builder::new().name("admin-rollback".into()).spawn(move || run_rollback(&h2, &uid2, &opts2)).ok();
    Ok(j)
}
fn set_status(h: &Hub, uid: &str, f: impl FnOnce(&mut Value)) { if let Ok(mut m) = load_upd(h, uid) { f(&mut m); save_upd(h, &m); } }

fn run_apply(h: &Arc<Hub>, uid: &str, prev: Option<String>, opts: &Value) {
    let r: Result<(), String> = (|| {
        let m = load_upd(h, uid).map_err(|e| e.message)?;
        set_status(h, uid, |m| m["status"] = json!("applying"));
        let pl = plan(h, &m);
        let st: std::collections::HashMap<String, String> = pl["plan"].as_array().unwrap().iter().map(|r| (sget(r, "path").to_string(), sget(r, "status").to_string())).collect();
        let todo: Vec<Value> = m["files"].as_array().unwrap().iter().filter(|f| matches!(st.get(sget(f, "path")).map(|s| s.as_str()), Some("new") | Some("changed"))).cloned().collect();
        let dels: Vec<String> = m["delete"].as_array().unwrap().iter().filter_map(|p| p.as_str()).filter(|p| st.get(*p).map(|s| s == "delete").unwrap_or(false)).map(|s| s.to_string()).collect();
        let mut comps = std::collections::BTreeSet::new();
        for rel in todo.iter().map(|f| sget(f, "path")).chain(dels.iter().map(|s| s.as_str())) { for c in components(rel) { comps.insert(c); } }
        jlog(h, &format!("更新 {} {}: 写入 {} 个文件，删除 {} 个", sget(&m, "version"), sget(&m, "title"), todo.len(), dels.len()), "info");
        let bdir = h.admin.dir.join("backups").join(uid);
        let _ = std::fs::remove_dir_all(&bdir);
        std::fs::create_dir_all(bdir.join("files")).map_err(|e| e.to_string())?;
        let mut added = vec![];
        for rel in todo.iter().map(|f| sget(f, "path").to_string()).chain(dels.iter().cloned()).chain(std::iter::once(VERSION_FILE.to_string())) {
            let src = h.root.join(&rel);
            if src.is_file() { let dst = bdir.join("files").join(&rel); if let Some(d) = dst.parent() { let _ = std::fs::create_dir_all(d); } let _ = std::fs::copy(&src, &dst); }
            else if rel != VERSION_FILE { added.push(rel); }
        }
        h.s.write_json(&bdir.join("backup.json"), &json!({"added": added, "components": comps, "prev_update": prev}));
        jlog(h, &format!("已备份 {} 个原文件", todo.len() + dels.len() - added.len()), "info");
        let pkg = std::fs::read(upd_dir(h, uid).join("pkg.tgz")).map_err(|e| e.to_string())?;
        let files = tar_files(&pkg)?;
        for f in &todo {
            let rel = sget(f, "path");
            let ent = files.iter().find(|e| e.0 == format!("files/{rel}")).ok_or(format!("更新包缺少 {rel}"))?;
            write_file(&h.root, rel, &ent.1, f.get("mode").and_then(|m| m.as_u64()).unwrap_or(0o644) as u32).map_err(|e| format!("写入 {rel} 失败: {e}"))?;
        }
        for rel in &dels { let _ = std::fs::remove_file(h.root.join(rel)); }
        write_file(&h.root, VERSION_FILE, format!("{} {}\n", sget(&m, "version"), sget(&m, "title")).as_bytes(), 0o644).map_err(|e| e.to_string())?;
        jlog(h, "文件已写入", "info");
        let comps: Vec<&str> = comps.into_iter().collect();
        finish(h, &comps, opts);
        set_status(h, uid, |m| { m["status"] = json!("applied"); m["applied_at"] = json!(now()); });
        end_job(h, true, &format!("更新 {} 已应用", sget(&m, "version")));
        if comps.contains(&"hub") { crate::reexec_later(); }
        Ok(())
    })();
    if let Err(e) = r {
        set_status(h, uid, |m| m["status"] = json!("failed"));
        end_job(h, false, &format!("更新出错: {e}"));
    }
}

fn restore(h: &Hub, uid: &str) -> Result<Vec<String>, String> {
    let bdir = h.admin.dir.join("backups").join(uid);
    let b = Store::read_json(&bdir.join("backup.json")).ok_or("没有备份，无法回滚")?;
    let fdir = bdir.join("files");
    let (mut n, mut stack) = (0, vec![fdir.clone()]);
    while let Some(d) = stack.pop() {
        for e in std::fs::read_dir(&d).map_err(|e| e.to_string())?.flatten() {
            let p = e.path();
            if p.is_dir() { stack.push(p); continue; }
            let rel = p.strip_prefix(&fdir).unwrap().to_string_lossy().to_string();
            let mode = std::fs::metadata(&p).map(|m| { use std::os::unix::fs::PermissionsExt; m.permissions().mode() }).unwrap_or(0o644);
            write_file(&h.root, &rel, &std::fs::read(&p).map_err(|e| e.to_string())?, mode).map_err(|e| e.to_string())?;
            n += 1;
        }
    }
    let added: Vec<String> = b["added"].as_array().unwrap_or(&vec![]).iter().filter_map(|x| x.as_str().map(|s| s.to_string())).collect();
    for rel in &added { let _ = std::fs::remove_file(h.root.join(rel)); }
    jlog(h, &format!("已恢复 {n} 个文件，移除 {} 个新增文件", added.len()), "info");
    Ok(b["components"].as_array().unwrap_or(&vec![]).iter().filter_map(|x| x.as_str().map(|s| s.to_string())).collect())
}
fn run_rollback(h: &Arc<Hub>, uid: &str, opts: &Value) {
    let r: Result<(), String> = (|| {
        set_status(h, uid, |m| m["status"] = json!("rolling_back"));
        let comps = match restore(h, uid) { Ok(c) => c, Err(e) => { set_status(h, uid, |m| m["status"] = json!("applied")); return Err(e); } };
        let cr: Vec<&str> = comps.iter().map(|s| s.as_str()).collect();
        finish(h, &cr, opts);
        set_status(h, uid, |m| { m["status"] = json!("rolled_back"); m["rolled_back_at"] = json!(now()); });
        end_job(h, true, &format!("已回滚到更新前的版本 ({})", h.admin.version()["version"].as_str().unwrap_or("")));
        if cr.contains(&"hub") { crate::reexec_later(); }
        Ok(())
    })();
    if let Err(e) = r { end_job(h, false, &format!("回滚出错: {e}")); }
}

fn running(h: &Hub, node: Option<&str>) -> Vec<String> {
    h.dep.list(false).iter().filter(|i| matches!(sget(i, "status"), "running" | "degraded" | "deploying") && !truthy(i.get("external")) && node.map(|n| sget(i, "sim_node") == n || sget(i, "nav_node") == n).unwrap_or(true)).map(|i| sget(i, "id").to_string()).collect()
}
fn wait_instances(h: &Hub, mut ids: Vec<String>) {
    let t0 = now();
    while !ids.is_empty() && now() - t0 < 600.0 {
        ids.retain(|iid| match h.dep.get(iid) { Ok(i) if sget(&i, "status") == "deploying" => true, Ok(i) => { jlog(h, &format!("实例 {iid}: {}", sget(&i, "status")), "ok"); false } Err(_) => false });
        std::thread::sleep(std::time::Duration::from_secs(3));
    }
    for iid in ids { jlog(h, &format!("实例 {iid} 部署超时，请在首页查看"), "warn"); }
}
fn restart_instances(h: &Hub) {
    let ids = running(h, None);
    if ids.is_empty() { jlog(h, "没有运行中的实例需要重启", "info"); return; }
    for iid in &ids { jlog(h, &format!("重新部署实例 {iid}"), "info"); if let Err(e) = h.dep.restart(iid) { jlog(h, &format!("实例 {iid} 重启失败: {}", e.message), "warn"); } }
    wait_instances(h, ids);
}
fn restart_agents(h: &Hub, redeploy: bool) {
    for n in h.nodes.list() {
        if sget(&n, "status") == "offline" { continue; }
        let rt = n.pointer("/info/runtime").cloned().unwrap_or(json!({}));
        if sget(&rt, "runtime") != "process" || std::fs::canonicalize(sget(&rt, "code_dir")).ok() != std::fs::canonicalize(&h.root).ok() { continue; }
        let name = sget(&n, "name").to_string();
        let insts = running(h, Some(sget(&n, "id")));
        for iid in &insts { jlog(h, &format!("停止实例 {iid} (节点代理 {name} 要重启)"), "info"); let _ = h.dep.stop(iid); }
        if let Err(e) = h.nodes.client_of(&n).call("POST", "/api/v1/admin/restart", Some(&json!({})), Some(10.0)) { jlog(h, &format!("节点代理 {name} 不支持远程重启 ({e})"), "warn"); continue; }
        jlog(h, &format!("节点代理 {name} 正在重启"), "info");
        let t0 = now();
        std::thread::sleep(std::time::Duration::from_secs(4));
        let mut up = false;
        while now() - t0 < 90.0 {
            if let Ok(nn) = h.nodes.get(sget(&n, "id")) { if nn.get("last_seen").and_then(|v| v.as_f64()).unwrap_or(0.0) > t0 + 2.0 && sget(&nn, "status") != "offline" { up = true; break; } }
            std::thread::sleep(std::time::Duration::from_secs(2));
        }
        if !up { jlog(h, &format!("节点代理 {name} 90 秒内没有重新上线"), "warn"); continue; }
        jlog(h, &format!("节点代理 {name} 已重新上线"), "info");
        if !insts.is_empty() && redeploy {
            for iid in &insts { jlog(h, &format!("重新部署实例 {iid}"), "info"); if let Err(e) = h.dep.restart(iid) { jlog(h, &format!("实例 {iid} 重新部署失败: {}", e.message), "warn"); } }
            wait_instances(h, insts);
        }
    }
}
fn finish(h: &Hub, comps: &[&str], opts: &Value) {
    let redeploy = opts.get("restart_instances").and_then(|v| v.as_bool()).unwrap_or(true);
    if comps.contains(&"agent") { restart_agents(h, redeploy); }
    else if comps.contains(&"instances") && redeploy { restart_instances(h); }
    else if comps.contains(&"instances") { jlog(h, "运行中的实例没有重启，新程序在实例下次启动时生效", "info"); }
    if comps.contains(&"web") { jlog(h, "网页文件已更新，浏览器刷新后生效", "info"); }
}
/// POST /admin/restart {target: agent} → job that restarts local agents
pub fn restart_agent_job(h: &Arc<Hub>, by: &str) -> R<Value> {
    let j = new_job(h, "restart", "重启节点代理", by, "")?;
    let h2 = h.clone();
    std::thread::spawn(move || { restart_agents(&h2, true); end_job(&h2, true, "完成"); });
    Ok(j)
}

// -------------------------------------------------------------- logs
fn log_dirs(h: &Hub) -> Vec<PathBuf> {
    let mut v: Vec<PathBuf> = vec![];
    if let Ok(d) = std::env::var("AGV_LOG_DIR") { if !d.is_empty() { v.push(d.into()); } }
    v.push(h.s.root.clone()); v.push(h.root.join("logs"));
    if let Some(p) = h.s.root.parent() { v.push(p.to_path_buf()); }
    v.push("/root".into());
    v.push(h.s.root.parent().map(|p| p.join(".agv-agent/logs")).unwrap_or("/root/.agv-agent/logs".into()));
    v.dedup();
    v
}
pub fn logs(h: &Hub) -> Value {
    let mut out: Vec<Value> = vec![]; let mut seen = std::collections::HashSet::new();
    for d in log_dirs(h) {
        for e in std::fs::read_dir(&d).into_iter().flatten().flatten() {
            let n = e.file_name().to_string_lossy().to_string();
            if !n.ends_with(".log") || !seen.insert(n.clone()) { continue; }
            if let Ok(m) = e.metadata() { if m.is_file() { out.push(json!({"name": n, "size": m.len(), "mtime": m.modified().ok().and_then(|t| t.duration_since(std::time::UNIX_EPOCH).ok()).map(|d| d.as_secs_f64()).unwrap_or(0.0)})); } }
        }
    }
    out.sort_by(|a, b| sget(a, "name").cmp(sget(b, "name")));
    json!({"logs": out})
}
pub fn log_file(h: &Hub, name: &str) -> R<Vec<u8>> {
    if name.contains('/') || name.contains("..") || !name.ends_with(".log") { return Err(err(404, "没有这个日志")); }
    for d in log_dirs(h) {
        let p = d.join(name);
        if let Ok(mut f) = std::fs::File::open(&p) {
            use std::io::{Seek, SeekFrom};
            let size = f.seek(SeekFrom::End(0)).unwrap_or(0);
            let _ = f.seek(SeekFrom::Start(size.saturating_sub(8 << 20)));
            let mut b = vec![]; let _ = f.read_to_end(&mut b);
            return Ok(b);
        }
    }
    Err(err(404, "日志文件不存在"))
}

// -------------------------------------------------------------- APK store + adb
fn apk_info(data: &[u8]) -> R<Value> {
    let mut z = zip::ZipArchive::new(std::io::Cursor::new(data)).map_err(|_| err(400, "不是有效的 APK 文件"))?;
    let mut ax = vec![];
    z.by_name("AndroidManifest.xml").map_err(|_| err(400, "不是有效的 APK 文件"))?.read_to_end(&mut ax).map_err(|_| err(400, "不是有效的 APK 文件"))?;
    let (u16_, u32_) = (|b: &[u8], p: usize| u16::from_le_bytes([b[p], b[p + 1]]), |b: &[u8], p: usize| u32::from_le_bytes([b[p], b[p + 1], b[p + 2], b[p + 3]]));
    let mut info = json!({"package": null, "versionCode": null, "versionName": null, "label": null});
    let (mut pos, mut strings): (usize, Vec<String>) = (8, vec![]);
    let _ = (|| -> Option<()> {
        while pos + 8 < ax.len() {
            let (typ, hsz, sz) = (u16_(&ax, pos), u16_(&ax, pos + 2) as usize, u32_(&ax, pos + 4) as usize);
            if sz == 0 { break; }
            if typ == 0x0001 {
                let (cnt, flags, sstart) = (u32_(&ax, pos + 8) as usize, u32_(&ax, pos + 16), u32_(&ax, pos + 20) as usize);
                let base = pos + sstart;
                for i in 0..cnt {
                    let mut p = base + u32_(&ax, pos + hsz + i * 4) as usize;
                    if flags & 0x100 != 0 {
                        let n = *ax.get(p)? as usize; p += if n & 0x80 != 0 { 2 } else { 1 };
                        let mut n = *ax.get(p)? as usize; p += 1;
                        if n & 0x80 != 0 { n = ((n & 0x7F) << 8) | *ax.get(p)? as usize; p += 1; }
                        strings.push(String::from_utf8_lossy(ax.get(p..p + n)?).to_string());
                    } else {
                        let mut n = u16_(&ax, p) as usize; p += 2;
                        if n & 0x8000 != 0 { n = ((n & 0x7FFF) << 16) | u16_(&ax, p) as usize; p += 2; }
                        let u: Vec<u16> = ax.get(p..p + n * 2)?.chunks(2).map(|c| u16::from_le_bytes([c[0], c[1]])).collect();
                        strings.push(String::from_utf16_lossy(&u));
                    }
                }
            } else if typ == 0x0102 {
                let name_i = u32_(&ax, pos + 20) as usize;
                let tag = strings.get(name_i).cloned().unwrap_or_default();
                let acnt = u16_(&ax, pos + 28) as usize;
                for k in 0..acnt {
                    let a = pos + 36 + k * 20;
                    let (an, raw, dtype, dval) = (u32_(&ax, a + 4) as usize, u32_(&ax, a + 8), ax[a + 15], u32_(&ax, a + 16));
                    let aname = strings.get(an).cloned().unwrap_or_default();
                    let val: Value = if raw != 0xFFFF_FFFF && (raw as usize) < strings.len() { json!(strings[raw as usize]) } else if dtype == 0x10 || dtype == 0x11 { json!(dval) } else { Value::Null };
                    if tag == "manifest" && matches!(aname.as_str(), "package" | "versionCode" | "versionName") { info[&aname] = val.clone(); }
                    if tag == "application" && aname == "label" && val.is_string() { info["label"] = val; }
                }
                if tag == "application" { break; }
            }
            pos += sz;
        }
        Some(())
    })();
    if !truthy(info.get("package")) { return Err(err(400, "无法从 APK 里读出包名")); }
    Ok(info)
}
fn adb_bin() -> Option<String> {
    for p in ["adb", "/usr/bin/adb", "/usr/local/bin/adb", "/data/local/tmp/adb"] {
        if std::process::Command::new(p).arg("version").stdout(std::process::Stdio::null()).stderr(std::process::Stdio::null()).status().map(|s| s.success()).unwrap_or(false) { return Some(p.into()); }
    }
    None
}
fn run_cmd(prog: &str, args: &[&str], timeout_s: u64) -> (i32, String) {
    use std::process::{Command, Stdio};
    let Ok(mut c) = Command::new(prog).args(args).stdin(Stdio::null()).stdout(Stdio::piped()).stderr(Stdio::piped()).spawn() else { return (127, format!("无法执行 {prog}")) };
    let t0 = std::time::Instant::now();
    loop {
        match c.try_wait() {
            Ok(Some(_)) => break,
            Ok(None) if t0.elapsed().as_secs() < timeout_s => std::thread::sleep(std::time::Duration::from_millis(50)),
            _ => { let _ = c.kill(); let _ = c.wait(); return (124, "超时".into()); }
        }
    }
    let o = c.wait_with_output().unwrap();
    (o.status.code().unwrap_or(-1), format!("{}{}", String::from_utf8_lossy(&o.stdout), String::from_utf8_lossy(&o.stderr)))
}
fn adb_devices() -> Value {
    let Some(a) = adb_bin() else { return Value::Null };
    let (_, out) = run_cmd(&a, &["devices", "-l"], 8);
    json!({"devices": out.lines().skip(1).filter(|l| !l.trim().is_empty()).map(|l| { let mut it = l.split_whitespace(); json!({"serial": it.next().unwrap_or(""), "state": it.next().unwrap_or(""), "detail": it.collect::<Vec<_>>().join(" ")}) }).collect::<Vec<_>>()})
}
fn app_dir(h: &Hub) -> PathBuf { h.admin.dir.join("apps") }
pub fn apps(h: &Hub) -> Value {
    let mut items: Vec<Value> = std::fs::read_dir(app_dir(h)).into_iter().flatten().flatten().filter(|e| e.file_name().to_string_lossy().ends_with(".json")).filter_map(|e| Store::read_json(&e.path())).collect();
    items.sort_by(|a, b| b.get("uploaded").and_then(|v| v.as_f64()).unwrap_or(0.0).partial_cmp(&a.get("uploaded").and_then(|v| v.as_f64()).unwrap_or(0.0)).unwrap());
    let adb = adb_devices();
    json!({"apps": items, "installed": {}, "adb": adb, "helper": !adb.is_null()})
}
pub fn app_upload(h: &Hub, data: &[u8], filename: &str, by: &str) -> R<Value> {
    if data.len() > MAX_APK { return Err(err(413, "APK 过大")); }
    let info = apk_info(data)?;
    let sh = sha256(data);
    let aid = format!("{}-{}-{}", sget(&info, "package"), info["versionCode"].as_u64().map(|v| v.to_string()).unwrap_or("0".into()), &sh[..8]);
    std::fs::write(app_dir(h).join(format!("{aid}.apk")), data).map_err(|e| err(500, e.to_string()))?;
    let mut doc = info; doc["id"] = json!(aid); doc["filename"] = json!(if filename.is_empty() { format!("{aid}.apk") } else { filename.to_string() });
    doc["size"] = json!(data.len()); doc["sha256"] = json!(sh); doc["uploaded"] = json!(now()); doc["uploaded_by"] = json!(by);
    h.s.write_json(&app_dir(h).join(format!("{aid}.json")), &doc);
    Ok(doc)
}
pub fn app_delete(h: &Hub, aid: &str) -> R<Value> {
    let aid = safe_rel(aid)?;
    for ext in ["apk", "json"] { let _ = std::fs::remove_file(app_dir(h).join(format!("{aid}.{ext}"))); }
    Ok(json!({"ok": true}))
}
pub fn app_download(h: &Hub, aid: &str) -> R<Vec<u8>> { std::fs::read(app_dir(h).join(format!("{}.apk", safe_rel(aid)?))).map_err(|_| err(404, "APK 不存在")) }
pub fn app_install(h: &Hub, aid: &str, mode: &str, serial: &str) -> R<Value> {
    let aid = safe_rel(aid)?;
    let src = app_dir(h).join(format!("{aid}.apk"));
    if !src.is_file() { return Err(err(404, "APK 不存在")); }
    let Some(a) = adb_bin() else { return Err(err(400, "本机没有 adb，无法安装 APK；可用 GET /api/hub/admin/apps/{id}/download 下载后自行安装")) };
    if mode == "prompt" { return Err(err(400, "弹窗安装需要安卓助手 (Termux)，请改用 mode=adb")); }
    let mut args: Vec<&str> = vec![]; if !serial.is_empty() { args.push("-s"); args.push(serial); }
    let p = src.to_string_lossy().to_string();
    args.extend(["install", "-r", &p]);
    let (rc, out) = run_cmd(&a, &args, 180);
    if rc != 0 || !out.contains("Success") { return Err(ApiError::coded(502, format!("adb 安装失败: {}", out.trim().chars().rev().take(300).collect::<String>().chars().rev().collect::<String>()), "adb")); }
    Ok(json!({"ok": true, "mode": "adb", "output": out.trim()}))
}
pub fn adb_action(action: &str, b: &Value) -> R<Value> {
    if !matches!(action, "pair" | "connect" | "disconnect" | "mdns") { return Err(err(400, "未知操作")); }
    let Some(a) = adb_bin() else { return Err(err(400, "本机没有 adb")) };
    let hostport = || { let h = sget(b, "host"); let p = b.get("port").map(|p| p.to_string().trim_matches('"').to_string()).unwrap_or_default(); if h.is_empty() { p } else { format!("{h}:{p}") } };
    let (rc, out) = match action {
        "pair" => { let t = hostport(); run_cmd(&a, &["pair", &t, sget(b, "code")], 30) }
        "connect" => { let t = hostport(); run_cmd(&a, &["connect", &t], 20) }
        "disconnect" => { let t = hostport(); if t.is_empty() { run_cmd(&a, &["disconnect"], 10) } else { run_cmd(&a, &["disconnect", &t], 10) } }
        _ => run_cmd(&a, &["mdns", "services"], 10),
    };
    Ok(json!({"ok": rc == 0, "output": out.trim()}))
}

// -------------------------------------------------------------- package upload (docker-save image or Rust program bundle)
pub fn package_upload(h: &Hub, q: &Request) -> R<Value> {
    let data = &q.body;
    if data.len() < 200 { return Err(err(400, "程序包为空或过小")); }
    let gz = data[0] == 0x1f && data[1] == 0x8b;
    let files = tar_files(data).map_err(|e| err(400, format!("不是有效的 tar / tar.gz 包: {e}")))?;
    let m = |k: &str| q.qs(k).map(|v| rest::url_decode(&v.replace('+', "%2B"))).unwrap_or_default();
    let mut refs: Vec<String> = vec![]; let mut arch = Value::Null; let (mut source, mut created) = ("bundle", Value::Null);
    if let Some(man) = files.iter().find(|f| f.0 == "manifest.json") {
        if let Ok(Value::Array(a)) = serde_json::from_slice::<Value>(&man.1) {
            for e in &a { for t in e.get("RepoTags").and_then(|t| t.as_array()).into_iter().flatten() { if let Some(t) = t.as_str() { refs.push(t.into()); } }
                if let Some(cfg) = e.get("Config").and_then(|c| c.as_str()).and_then(|c| files.iter().find(|f| f.0 == c)).and_then(|f| serde_json::from_slice::<Value>(&f.1).ok()) {
                    if let Some(a) = cfg.get("architecture").and_then(|a| a.as_str()) { arch = json!(match a { "aarch64" => "arm64", "x86_64" => "amd64", o => o }); }
                    created = cfg.get("created").cloned().unwrap_or(Value::Null); } }
            if !refs.is_empty() { source = "upload"; }
        }
    }
    let bundle_bins: Vec<String> = files.iter().filter(|f| f.0.starts_with("bin/") || f.0.starts_with("aarch64/") || f.0.starts_with("x86_64/")).map(|f| f.0.clone()).collect();
    if refs.is_empty() && bundle_bins.is_empty() { return Err(err(400, "不是有效的程序包: 既不是 docker save 镜像 (无 RepoTags)，也不含 bin/ 程序目录")); }
    let ids: Vec<String> = h.s.list("packages").iter().map(|p| sget(p, "id").to_string()).collect();
    let pid = new_id("pkg-", &|i| ids.iter().any(|e| e == i));
    let fname = format!("{pid}{}", if gz { ".tar.gz" } else { ".tar" });
    std::fs::write(h.s.path(&["packages", &fname]), data).map_err(|e| err(500, e.to_string()))?;
    let kind = { let k = m("kind"); if !k.is_empty() { k } else if refs.first().map(|r| r.contains("agv-nav")).unwrap_or(false) { "nav".into() } else { "sim".into() } };
    let rf = if !m("ref").is_empty() { m("ref") } else if let Some(r) = refs.first() { r.clone() } else { format!("bundle:{kind}") };
    if arch.is_null() && !bundle_bins.is_empty() { arch = json!(if bundle_bins.iter().any(|b| b.starts_with("x86_64/")) { "amd64" } else { "arm64" }); }
    let ver = { let v = m("version"); if !v.is_empty() { v } else { rf.rsplit(':').next().unwrap_or("").to_string() } };
    let fn_ = m("from_node");
    Ok(h.s.put("packages", json!({"id": pid, "kind": kind, "source": source, "file": fname, "size": data.len(), "sha256": sha256(data), "image_ref": rf, "refs": refs, "arch": arch,
        "version": ver, "api": "v1", "note": m("note"), "tag": m("tag"), "image_created": created, "operator": user(q), "from_node": if fn_.is_empty() { Value::Null } else { json!(fn_) },
        "bins": bundle_bins.iter().map(|b| b.rsplit('/').next().unwrap_or("").to_string()).collect::<Vec<_>>()})))
}
pub fn package_from_node(h: &Hub, q: &Request) -> R<Value> {
    let b = q.json()?;
    let n = h.nodes.get(sget(&b, "node_id"))?;
    let hub_url = if truthy(n.get("hub_url")) { sget(&n, "hub_url").to_string() } else { format!("http://127.0.0.1:{}", h.port) };
    let mut qs = vec![("ref", sget(&b, "ref").to_string()), ("from_node", sget(&n, "id").to_string()), ("operator", user(q))];
    for k in ["kind", "version", "note", "tag"] { if truthy(b.get(k)) { qs.push((k, sget(&b, k).to_string())); } }
    let qs: Vec<String> = qs.iter().map(|(k, v)| format!("{k}={}", rest::url_encode(v))).collect();
    let job = h.nodes.client_of(&n).call("POST", "/api/v1/images/export", Some(&json!({"ref": sget(&b, "ref"), "upload_url": format!("{hub_url}/api/hub/packages/upload?{}", qs.join("&"))})), Some(15.0)).map_err(|e| err(502, e))?;
    Ok(json!({"node_id": n["id"], "job": job}))
}

// -------------------------------------------------------------- add node over SSH
fn seal(h: &Hub, text: &str) -> String {
    use base64::Engine; use hmac::{Hmac, Mac};
    let kp = h.s.path(&["secret.key"]);
    if !kp.exists() { let _ = std::fs::write(&kp, rand_bytes(32)); #[cfg(unix)] { use std::os::unix::fs::PermissionsExt; let _ = std::fs::set_permissions(&kp, std::fs::Permissions::from_mode(0o600)); } }
    let k = std::fs::read(&kp).unwrap_or_default();
    let (nonce, data) = (rand_bytes(16), text.as_bytes());
    let mut stream = vec![];
    for i in 0..(data.len() / 32 + 1) as u32 { let mut m = <Hmac<Sha256> as Mac>::new_from_slice(&k).unwrap(); m.update(&nonce); m.update(&i.to_be_bytes()); stream.extend(m.finalize().into_bytes()); }
    let ct: Vec<u8> = data.iter().zip(stream.iter()).map(|(a, b)| a ^ b).collect();
    let mut m = <Hmac<Sha256> as Mac>::new_from_slice(&k).unwrap(); m.update(b"mac"); m.update(&nonce); m.update(&ct);
    let tag = m.finalize().into_bytes();
    base64::engine::general_purpose::STANDARD.encode([&nonce[..], &tag[..16], &ct[..]].concat())
}
fn ssh_run(ip: &str, port: u16, user: &str, pw: &str, script: &str) -> Result<String, ApiError> {
    use std::os::unix::fs::PermissionsExt;
    use std::process::{Command, Stdio};
    let tmp = std::env::temp_dir().join(format!("askpass-{}", hex(&rand_bytes(6))));
    std::fs::write(&tmp, "#!/bin/sh\nprintf '%s\\n' \"$AGV_SSH_PW\"\n").map_err(|e| err(500, e.to_string()))?;
    let _ = std::fs::set_permissions(&tmp, std::fs::Permissions::from_mode(0o700));
    let target = format!("{user}@{ip}");
    let c = Command::new("setsid").args(["ssh", "-p", &port.to_string(), "-o", "StrictHostKeyChecking=no", "-o", "UserKnownHostsFile=/dev/null", "-o", "ConnectTimeout=8", "-o", "NumberOfPasswordPrompts=1", "-o", "PreferredAuthentications=password,keyboard-interactive", "-o", "LogLevel=ERROR", &target, script])
        .env("SSH_ASKPASS", &tmp).env("SSH_ASKPASS_REQUIRE", "force").env("DISPLAY", ":0").env("AGV_SSH_PW", pw).stdin(Stdio::null()).stdout(Stdio::piped()).stderr(Stdio::piped()).output();
    let _ = std::fs::remove_file(&tmp);
    let o = c.map_err(|e| ApiError::coded(502, format!("本机没有 ssh 客户端，无法验证登录: {e}"), "ssh_client"))?;
    let out = format!("{}{}", String::from_utf8_lossy(&o.stdout), String::from_utf8_lossy(&o.stderr));
    if !o.status.success() && !out.contains("AGVOK") {
        let msg = if out.contains("Permission denied") { "用户名或密码错误".to_string() } else { out.trim().chars().take(200).collect() };
        return Err(ApiError::coded(502, format!("SSH 登录 {ip} 失败: {msg}"), "ssh_login"));
    }
    Ok(out)
}
pub fn nodes_add(h: &Hub, b: &Value) -> R<Value> {
    let ip = sget(b, "ip").trim().to_string();
    let (usr, pw) = (sget(b, "username").trim().to_string(), sget(b, "password").to_string());
    let port: u16 = b.get("ssh_port").map(|p| p.to_string().trim_matches('"').parse().unwrap_or(0)).filter(|p| *p != 0 || b.get("ssh_port").map(|v| v.is_null()).unwrap_or(true)).unwrap_or(22);
    let port = if b.get("ssh_port").map(|v| v.is_null() || v == &json!("")).unwrap_or(true) { 22 } else { port };
    let kind = if truthy(b.get("kind")) { sget(b, "kind").to_string() } else { "hybrid".into() };
    if ip.is_empty() || ip.chars().any(|c| " /@:;'\"$`".contains(c)) { return Err(err(400, "请填写正确的节点 IP")); }
    if port == 0 { return Err(err(400, "SSH 端口应为 1–65535")); }
    if usr.is_empty() || pw.is_empty() { return Err(err(400, "请填写 SSH 用户名和密码")); }
    if !matches!(kind.as_str(), "controller" | "hybrid" | "sim") { return Err(err(400, "计算资源类型无效")); }
    use std::net::ToSocketAddrs;
    let addr = format!("{ip}:{port}").to_socket_addrs().ok().and_then(|mut a| a.next()).ok_or_else(|| ApiError::coded(502, format!("{ip} 无法解析"), "ssh_connect"))?;
    std::net::TcpStream::connect_timeout(&addr, std::time::Duration::from_secs(6)).map_err(|e| ApiError::coded(502, format!("{ip}:{port} 端口不通: {e}"), "ssh_connect"))?;
    let script = "echo AGVOK; uname -m; (. /etc/os-release 2>/dev/null && echo \"$PRETTY_NAME\") || uname -sr; if command -v docker >/dev/null 2>&1; then echo Docker; else echo 无\\ Docker; fi";
    let out = ssh_run(&ip, port, &usr, &pw, script)?;
    let lines: Vec<&str> = out.lines().collect();
    let i = lines.iter().position(|l| *l == "AGVOK").ok_or_else(|| ApiError::coded(502, format!("登录 {ip} 成功但执行命令失败: {}", out.trim().chars().take(200).collect::<String>()), "ssh_exec"))?;
    let g = |k: usize| lines.get(i + k).map(|s| s.trim().to_string()).unwrap_or_default();
    let arch = match g(1).as_str() { "aarch64" => "arm64".to_string(), "x86_64" => "amd64".into(), "armv7l" => "arm".into(), o => o.to_string() };
    let (osn, dk) = (g(2), g(3));
    let brief = [arch.as_str(), osn.as_str(), dk.as_str()].iter().filter(|s| !s.is_empty()).cloned().collect::<Vec<_>>().join(" · ");
    let check = json!({"at": now(), "arch": arch, "os": osn, "docker": dk, "brief": brief});
    let cred = seal(h, &pw);
    let all = h.s.list("nodes");
    let ex = all.iter().find(|x| x.pointer("/ssh/ip").and_then(|v| v.as_str()) == Some(&ip) || sget(x, "host") == ip).cloned();
    let note = sget(b, "note").to_string(); let name = sget(b, "name").trim().to_string();
    let ssh = json!({"ip": ip, "port": port, "user": usr});
    let n = match ex {
        Some(mut n) if sget(&n, "status") != "pending" && truthy(n.get("key")) => { n["ssh"] = ssh; n["cred"] = json!(cred); n["ssh_check"] = check.clone(); if !note.is_empty() { n["note"] = json!(note); } h.s.put("nodes", n) }
        ex => {
            let mut n = ex.unwrap_or_else(|| { let ids: Vec<String> = all.iter().map(|x| sget(x, "id").to_string()).collect(); json!({"id": new_id("n-", &|i| ids.iter().any(|e| e == i)), "created": now(), "max_instances": 1}) });
            n["name"] = json!(if name.is_empty() { ip.clone() } else { name }); n["host"] = json!(ip); n["api_port"] = json!(8070); n["kind"] = json!(kind); n["note"] = json!(note);
            n["ssh"] = ssh; n["cred"] = json!(cred); n["status"] = json!("pending"); n["arch"] = check["arch"].clone(); n["ssh_check"] = check.clone();
            n["info"] = json!({"os": check["os"], "arch": check["arch"], "ips": [ip]});
            h.s.put("nodes", n)
        }
    };
    Ok(json!({"node": h.nodes.view(&n, &h.dep.list(false)), "check": check["brief"]}))
}
