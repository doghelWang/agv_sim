//! .cmodel (ZIP + Protobuf) decoder. Reads the serialized FileDescriptorProto (*.desc) at runtime and
//! decodes messages dynamically into JSON identical to Python protobuf's MessageToDict (proto3 rules).
use serde_json::{json, Map, Value};
use std::collections::HashMap;
use std::io::Read;

pub mod spec;

const DESCS: [(&str, &[u8], &str, &str); 3] = [
    ("CompDesc.model", include_bytes!("../desc/controller_model_comp_desc.desc"), "Message_Module_Info", "CompDesc"),
    ("AbiSet.model", include_bytes!("../desc/controller_model_abi_set.desc"), "Controller_Ability", "AbiSet"),
    ("FuncDesc.model", include_bytes!("../desc/controller_model_abi_desc.desc"), "Robot_Description", "FuncDesc"),
];

// ---------------------------------------------------------------- wire reader
#[derive(Clone, Debug)]
enum W<'a> { V(u64), F64(u64), F32(u32), L(&'a [u8]) }

fn varint(b: &[u8], i: &mut usize) -> Option<u64> {
    let (mut v, mut s) = (0u64, 0u32);
    loop {
        let x = *b.get(*i)?; *i += 1;
        if s < 64 { v |= ((x & 0x7f) as u64) << s; }
        if x & 0x80 == 0 { return Some(v); }
        s += 7;
        if s > 70 { return None; }
    }
}

fn fields(b: &[u8]) -> Option<Vec<(u32, W<'_>)>> {
    let mut i = 0; let mut out = vec![];
    while i < b.len() {
        let k = varint(b, &mut i)?;
        let (num, wt) = ((k >> 3) as u32, (k & 7) as u8);
        out.push((num, match wt {
            0 => W::V(varint(b, &mut i)?),
            1 => { let s = b.get(i..i + 8)?; i += 8; W::F64(u64::from_le_bytes(s.try_into().ok()?)) }
            2 => { let n = varint(b, &mut i)? as usize; let s = b.get(i..i.checked_add(n)?)?; i += n; W::L(s) }
            5 => { let s = b.get(i..i + 4)?; i += 4; W::F32(u32::from_le_bytes(s.try_into().ok()?)) }
            _ => return None,
        }));
    }
    Some(out)
}

// ---------------------------------------------------------------- descriptors
#[derive(Clone, Debug)]
struct Fld { name: String, json: String, num: u32, repeated: bool, ty: u32, type_name: String, present: bool }
#[derive(Clone, Debug, Default)]
struct Msg { fields: Vec<Fld> }
#[derive(Default)]
struct Pool { msgs: HashMap<String, Msg>, enums: HashMap<String, HashMap<i32, String>> }

fn s(w: &W) -> String { if let W::L(b) = w { String::from_utf8_lossy(b).into_owned() } else { String::new() } }
fn n(w: &W) -> u64 { if let W::V(v) = w { *v } else { 0 } }

fn camel(name: &str) -> String {
    let mut o = String::new(); let mut up = false;
    for c in name.chars() { if c == '_' { up = true } else if up { o.extend(c.to_uppercase()); up = false } else { o.push(c) } }
    o
}

fn add_enum(p: &mut Pool, full: String, b: &[u8]) {
    let mut m = HashMap::new();
    for (num, w) in fields(b).unwrap_or_default() {
        if num == 2 { if let W::L(vb) = w { let vf = fields(vb).unwrap_or_default();
            let name = vf.iter().find(|f| f.0 == 1).map(|f| s(&f.1)).unwrap_or_default();
            let val = vf.iter().find(|f| f.0 == 2).map(|f| n(&f.1) as i32).unwrap_or(0);
            m.entry(val).or_insert(name); } }
    }
    p.enums.insert(full, m);
}

fn add_msg(p: &mut Pool, prefix: &str, b: &[u8]) {
    let fs = fields(b).unwrap_or_default();
    let name = fs.iter().find(|f| f.0 == 1).map(|f| s(&f.1)).unwrap_or_default();
    let full = format!("{prefix}.{name}");
    let mut m = Msg::default();
    for (num, w) in &fs {
        match (num, w) {
            (2, W::L(fb)) => {
                let ff = fields(fb).unwrap_or_default();
                let g = |k: u32| ff.iter().find(|f| f.0 == k);
                let nm = g(1).map(|f| s(&f.1)).unwrap_or_default();
                let js = g(10).map(|f| s(&f.1)).filter(|x| !x.is_empty()).unwrap_or_else(|| camel(&nm));
                m.fields.push(Fld { json: js, name: nm, num: g(3).map(|f| n(&f.1) as u32).unwrap_or(0), repeated: g(4).map(|f| n(&f.1)) == Some(3),
                    ty: g(5).map(|f| n(&f.1) as u32).unwrap_or(0), type_name: g(6).map(|f| s(&f.1)).unwrap_or_default(),
                    present: g(9).is_some() || g(17).map(|f| n(&f.1)) == Some(1) });
            }
            (3, W::L(nb)) => add_msg(p, &full, nb),
            (4, W::L(eb)) => { let en = fields(eb).unwrap_or_default().iter().find(|f| f.0 == 1).map(|f| s(&f.1)).unwrap_or_default(); add_enum(p, format!("{full}.{en}"), eb); }
            _ => {}
        }
    }
    m.fields.sort_by_key(|f| f.num);
    p.msgs.insert(full, m);
}

fn load_pool(desc: &[u8]) -> (Pool, String) {
    let mut p = Pool::default();
    let fs = fields(desc).unwrap_or_default();
    let pkg = fs.iter().find(|f| f.0 == 2).map(|f| s(&f.1)).unwrap_or_default();
    for (num, w) in &fs {
        match (num, w) {
            (4, W::L(b)) => add_msg(&mut p, &format!(".{pkg}"), b),
            (5, W::L(b)) => { let en = fields(b).unwrap_or_default().iter().find(|f| f.0 == 1).map(|f| s(&f.1)).unwrap_or_default(); add_enum(&mut p, format!(".{pkg}.{en}"), b); }
            _ => {}
        }
    }
    (p, pkg)
}

// ---------------------------------------------------------------- json conversion
fn b64(b: &[u8]) -> String {
    const T: &[u8] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut o = String::new();
    for c in b.chunks(3) {
        let v = (c[0] as u32) << 16 | (*c.get(1).unwrap_or(&0) as u32) << 8 | *c.get(2).unwrap_or(&0) as u32;
        o.push(T[(v >> 18) as usize & 63] as char); o.push(T[(v >> 12) as usize & 63] as char);
        o.push(if c.len() > 1 { T[(v >> 6) as usize & 63] as char } else { '=' });
        o.push(if c.len() > 2 { T[v as usize & 63] as char } else { '=' });
    }
    o
}

fn fnum(v: f64) -> Value {
    if v.is_nan() { json!("NaN") } else if v.is_infinite() { json!(if v > 0.0 { "Infinity" } else { "-Infinity" }) } else { json!(v) }
}

fn scalar(p: &Pool, f: &Fld, w: &W) -> Option<Value> {
    Some(match (f.ty, w) {
        (1, W::F64(b)) => fnum(f64::from_bits(*b)),
        (2, W::F32(b)) => { let x = f32::from_bits(*b); if x.is_finite() { json!(format!("{x}").parse::<f64>().unwrap_or(x as f64)) } else { fnum(x as f64) } }
        (3, W::V(v)) => json!((*v as i64).to_string()),
        (4, W::V(v)) => json!(v.to_string()),
        (5, W::V(v)) => json!(*v as i32),
        (6, W::F64(b)) => json!(b.to_string()),
        (7, W::F32(b)) => json!(*b),
        (8, W::V(v)) => json!(*v != 0),
        (9, W::L(b)) => json!(String::from_utf8_lossy(b)),
        (12, W::L(b)) => json!(b64(b)),
        (13, W::V(v)) => json!(*v as u32),
        (14, W::V(v)) => { let k = *v as i32; match p.enums.get(&f.type_name).and_then(|e| e.get(&k)) { Some(nm) => json!(nm), None => json!(k) } }
        (15, W::F32(b)) => json!(*b as i32),
        (16, W::F64(b)) => json!((*b as i64).to_string()),
        (17, W::V(v)) => json!(((*v >> 1) as i64 ^ -((*v & 1) as i64)) as i32),
        (18, W::V(v)) => json!((((*v >> 1) as i64) ^ -((*v & 1) as i64)).to_string()),
        (11, W::L(b)) => decode_msg(p, &f.type_name, b),
        _ => return None,
    })
}

fn unpack(f: &Fld, b: &[u8]) -> Vec<W<'static>> {
    let mut out = vec![]; let mut i = 0;
    match f.ty {
        1 | 6 | 16 => while i + 8 <= b.len() { out.push(W::F64(u64::from_le_bytes(b[i..i + 8].try_into().unwrap()))); i += 8; },
        2 | 7 | 15 => while i + 4 <= b.len() { out.push(W::F32(u32::from_le_bytes(b[i..i + 4].try_into().unwrap()))); i += 4; },
        _ => while i < b.len() { match varint(b, &mut i) { Some(v) => out.push(W::V(v)), None => break } },
    }
    out
}

fn decode_msg(p: &Pool, name: &str, b: &[u8]) -> Value {
    let Some(m) = p.msgs.get(name) else { return json!({}) };
    let mut raw: HashMap<u32, Vec<W>> = HashMap::new();
    for (num, w) in fields(b).unwrap_or_default() { raw.entry(num).or_default().push(w); }
    let mut o = Map::new();
    for f in &m.fields {
        let Some(ws) = raw.get(&f.num) else { continue };
        if f.repeated {
            let mut vs = vec![];
            for w in ws {
                match (w, f.ty) {
                    (W::L(pb), t) if !matches!(t, 9 | 11 | 12) => { for x in unpack(f, pb) { if let Some(v) = scalar(p, f, &x) { vs.push(v); } } }
                    _ => if let Some(v) = scalar(p, f, w) { vs.push(v); },
                }
            }
            if !vs.is_empty() { o.insert(f.json.clone(), Value::Array(vs)); }
        } else if let Some(v) = ws.last().and_then(|w| scalar(p, f, w)) {
            let is_default = match &v { Value::Bool(b) => !*b, Value::String(s) => s.is_empty() && f.ty != 11 && f.ty != 14 && !matches!(f.ty, 3 | 4 | 6 | 16 | 18), Value::Number(x) => x.as_f64() == Some(0.0), _ => false };
            let is_default = is_default || (matches!(f.ty, 3 | 4 | 6 | 16 | 18) && v.as_str() == Some("0")) || (f.ty == 14 && p.enums.get(&f.type_name).and_then(|e| e.get(&0)).map(|d| v.as_str() == Some(d.as_str())).unwrap_or(false));
            if f.present || f.ty == 11 || !is_default { o.insert(f.json.clone(), v); }
        }
    }
    Value::Object(o)
}

pub fn decode_bytes(model_name: &str, data: &[u8]) -> Result<Value, String> {
    let (_, desc, msg, _) = DESCS.iter().find(|d| d.0 == model_name).ok_or_else(|| format!("未知模型文件 {model_name}"))?;
    let (pool, pkg) = load_pool(desc);
    if !pool.msgs.contains_key(&format!(".{pkg}.{msg}")) { return Err(format!("描述中无消息 {msg}")); }
    Ok(decode_msg(&pool, &format!(".{pkg}.{msg}"), data))
}

/// {"CompDesc": .., "AbiSet": .., "FuncDesc": ..} (optional ones omitted when absent)
pub fn decode_cmodel_bytes(zip_bytes: &[u8]) -> Result<Map<String, Value>, String> {
    let mut z = zip::ZipArchive::new(std::io::Cursor::new(zip_bytes)).map_err(|e| format!("不是有效的 .cmodel (ZIP): {e}"))?;
    let mut out = Map::new();
    let names: Vec<String> = (0..z.len()).filter_map(|i| z.by_index(i).ok().map(|f| f.name().to_string())).collect();
    for (mname, _, _, key) in DESCS.iter() {
        let jname = format!("{key}.json");
        let found = names.iter().find(|n| n.rsplit('/').next() == Some(mname)).cloned();
        if let Some(n) = found {
            let mut buf = vec![]; z.by_name(&n).map_err(|e| e.to_string())?.read_to_end(&mut buf).map_err(|e| e.to_string())?;
            out.insert(key.to_string(), decode_bytes(mname, &buf)?);
        } else if let Some(n) = names.iter().find(|n| n.rsplit('/').next() == Some(jname.as_str())).cloned() {
            let mut buf = vec![]; z.by_name(&n).map_err(|e| e.to_string())?.read_to_end(&mut buf).map_err(|e| e.to_string())?;
            out.insert(key.to_string(), serde_json::from_slice(&buf).map_err(|e| e.to_string())?);
        }
    }
    if !out.contains_key("CompDesc") { return Err("cmodel 中缺少 CompDesc.model / CompDesc.json".into()); }
    Ok(out)
}
