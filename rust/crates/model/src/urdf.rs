//! URDF generation from the RobotAssembly (cmodel → assembly → URDF).
//!
//! Links/joints/visuals come from `sim_core::assembly::build` (chassis body, wheels incl. caster swivel, sensors, actuator
//! mechanism with prismatic/revolute joints and mimic joints).  Frames kept for ROS: base_footprint → base_link, imu_link,
//! `{sensor}_link`, `{camera}_optical_frame`, `{photo}_pe_link`.  Every value the cmodel did not provide is listed in a comment
//! block at the top (same content as `GET /api/v1/model/assembly` → `missing`).
use crate::{fnum, pstr};
use serde_json::Value;
use std::collections::BTreeMap;
use std::f64::consts::PI;

const SMALL: &str = "<inertial><origin xyz=\"0 0 0\" rpy=\"0 0 0\"/><mass value=\"0.05\"/><inertia ixx=\"1e-4\" ixy=\"0\" ixz=\"0\" iyy=\"1e-4\" iyz=\"0\" izz=\"1e-4\"/></inertial>";

fn box_inertia(m: f64, x: f64, y: f64, z: f64) -> (f64, f64, f64) { (m * (y * y + z * z) / 12.0, m * (x * x + z * z) / 12.0, m * (x * x + y * y) / 12.0) }
fn inertial(m: f64, i: (f64, f64, f64), origin: &str) -> String {
    format!("<inertial><origin xyz=\"{origin}\" rpy=\"0 0 0\"/><mass value=\"{m:.4}\"/><inertia ixx=\"{:.5}\" ixy=\"0\" ixz=\"0\" iyy=\"{:.5}\" iyz=\"0\" izz=\"{:.5}\"/></inertial>", i.0.max(1e-5), i.1.max(1e-5), i.2.max(1e-5))
}
fn n(v: &Value, k: &str, d: f64) -> f64 { fnum(v.get(k), d) }
fn s<'a>(v: &'a Value, k: &str) -> &'a str { v.get(k).and_then(|x| x.as_str()).unwrap_or("") }
fn a3(v: &Value, k: &str) -> [f64; 3] { let a = v.get(k).and_then(|x| x.as_array()); [0, 1, 2].map(|i| a.and_then(|a| a.get(i)).and_then(|x| x.as_f64()).unwrap_or(0.0)) }
fn f3(a: [f64; 3]) -> String { format!("{:.4} {:.4} {:.4}", a[0], a[1], a[2]) }
fn xml_esc(t: &str) -> String { t.replace('&', "&amp;").replace('<', "&lt;").replace('>', "&gt;").replace("--", "- -") }
fn mat_name(c: &str) -> String { format!("c_{}", c.trim_start_matches('#')) }
fn rgba(c: &str, op: f64) -> String {
    let h = c.trim_start_matches('#');
    let v = |i: usize| u8::from_str_radix(h.get(i..i + 2).unwrap_or("80"), 16).unwrap_or(128) as f64 / 255.0;
    format!("{:.3} {:.3} {:.3} {op}", v(0), v(2), v(4))
}

fn geometry(v: &Value) -> Option<String> {
    match s(v, "shape") {
        "cylinder" => { let (r, l) = (n(v, "radius", 0.0), n(v, "length", 0.0)); (r > 1e-4 && l > 1e-4).then(|| format!("<cylinder radius=\"{r:.4}\" length=\"{l:.4}\"/>")) }
        "sphere" => { let r = n(v, "radius", 0.0); (r > 1e-4).then(|| format!("<sphere radius=\"{r:.4}\"/>")) }
        _ => { let z = a3(v, "size"); z.iter().all(|x| *x > 1e-4).then(|| format!("<box size=\"{}\"/>", f3(z))) }
    }
}

pub fn generate_urdf(spec: &Value) -> String {
    let asm = sim_core::assembly::build(spec);
    assembly_urdf(spec, &asm)
}

pub fn assembly_urdf(spec: &Value, asm: &Value) -> String {
    let ch = &spec["chassis"];
    let links = asm["links"].as_array().cloned().unwrap_or_default();
    let mass = { let m = n(ch, "mass_kg", 0.0); if m == 0.0 { 150.0 } else { m } };
    let mut colors: BTreeMap<String, String> = BTreeMap::new();
    for l in &links { for v in l["visuals"].as_array().into_iter().flatten() { let c = s(v, "color"); if !c.is_empty() { colors.insert(mat_name(c), rgba(c, n(v, "opacity", 1.0))); } } }
    let mut x: Vec<String> = vec![
        "<?xml version=\"1.0\"?>".into(),
        format!("<!-- Auto-generated from cmodel {} (schema v{}) via RobotAssembly v{} -->", spec.get("model_file").map(pstr).unwrap_or_default(), spec.get("schema_version").map(pstr).unwrap_or("None".into()), asm["version"]),
        format!("<!-- chassis={} ({}), actuator={} [{}], base_link = 运动中心(motion center), 地面高度 -->", pstr(&ch["type"]), ch.get("cmodel_type").map(pstr).unwrap_or_default(), pstr(&asm["archetype"]), xml_esc(asm["archetype_source"].as_str().unwrap_or(""))),
        "<!-- cmodel 缺失/推断项 (missing in cmodel, value used here):".into(),
    ];
    for m in asm["missing"].as_array().into_iter().flatten() {
        x.push(format!("     [{}] {}.{} = {} : {}", s(m, "status"), s(m, "category"), s(m, "item"), xml_esc(&m["value"].to_string()), xml_esc(s(m, "note"))));
    }
    x.push("-->".into());
    x.push(format!("<robot name=\"{}\">", spec.get("robot_name").map(pstr).unwrap_or("cmodel_agv".into())));
    for (k, c) in &colors { x.push(format!("  <material name=\"{k}\"><color rgba=\"{c}\"/></material>")); }
    x.push("  <link name=\"base_footprint\"/>".into());
    x.push("  <joint name=\"base_footprint_joint\" type=\"fixed\"><parent link=\"base_footprint\"/><child link=\"base_link\"/><origin xyz=\"0 0 0\" rpy=\"0 0 0\"/></joint>".into());
    let c = &asm["chassis"];
    let (l_, w_, h_, cl) = (n(c, "length_m", 1.0), n(c, "width_m", 0.8), n(c, "body_top_m", 0.3), n(c, "clearance_m", 0.03));
    let cc = c.get("center").and_then(|v| v.as_array()).map(|v| [v[0].as_f64().unwrap_or(0.0), v[1].as_f64().unwrap_or(0.0)]).unwrap_or([0.0, 0.0]);
    let n_wheels = links.iter().filter(|l| l["joint"]["source"] == "wheel" || l["joint"]["source"] == "caster").count() as f64;
    let body_mass = (10.0f64).max(mass - 5.0 * n_wheels - 30.0);
    for l in &links {
        let name = s(l, "name");
        x.push(format!("  <link name=\"{name}\">"));
        for v in l["visuals"].as_array().into_iter().flatten() {
            let Some(g) = geometry(v) else { continue };
            x.push(format!("    <visual name=\"{}\"><origin xyz=\"{}\" rpy=\"{}\"/><geometry>{g}</geometry><material name=\"{}\"/></visual>", s(v, "role"), f3(a3(v, "xyz")), f3(a3(v, "rpy")), mat_name(s(v, "color"))));
        }
        let src = s(&l["joint"], "source");
        if name == "base_link" {
            let zc = cl + (h_ - cl) / 2.0;
            x.push(format!("    <collision><origin xyz=\"{:.4} {:.4} {zc:.4}\"/><geometry><box size=\"{l_:.4} {w_:.4} {:.4}\"/></geometry></collision>", cc[0], cc[1], (h_ - cl).max(0.02)));
            let com = match ch.get("com").and_then(|c| c.as_array()) { Some(c) if c.len() == 3 => format!("{:.4} {:.4} {:.4}", fnum(c.first(), 0.0), fnum(c.get(1), 0.0), fnum(c.get(2), 0.0)), _ => format!("{:.4} {:.4} {zc:.4}", cc[0], cc[1]) };
            x.push(format!("    {}", inertial(body_mass, box_inertia(body_mass, l_, w_, h_ - cl), &com)));
        } else if src == "wheel" || src == "caster" {
            let tyre = l["visuals"].as_array().and_then(|v| v.iter().find(|v| s(v, "role") == "tyre").cloned());
            if let Some(t) = tyre {
                let (r, wd) = (n(&t, "radius", 0.05), n(&t, "length", 0.04));
                if s(&l["joint"], "type") == "continuous" && (name.ends_with("_link")) && !name.ends_with("_swivel_link") {
                    x.push(format!("    <collision><origin xyz=\"0 0 0\" rpy=\"{:.4} 0 0\"/><geometry><cylinder radius=\"{r:.4}\" length=\"{wd:.4}\"/></geometry></collision>", PI / 2.0));
                }
                let m = if src == "caster" { 1.0 } else { 5.0 };
                let ixx = m * (3.0 * r * r + wd * wd) / 12.0;
                x.push(format!("    {}", inertial(m, (ixx, m * r * r / 2.0, ixx), "0 0 0")));
            } else { x.push(format!("    {SMALL}")); }
        } else if src == "actuator" {
            let m = 10.0;
            x.push(format!("    {}", inertial(m, (0.2, 0.2, 0.2), "0 0 0")));
        } else { x.push(format!("    {SMALL}")); }
        x.push("  </link>".into());
        let j = &l["joint"];
        if j.is_object() {
            let jt = s(j, "type");
            let mut js = format!("  <joint name=\"{}\" type=\"{jt}\"><parent link=\"{}\"/><child link=\"{name}\"/><origin xyz=\"{}\" rpy=\"{}\"/>", s(j, "name"), s(l, "parent"), f3(a3(j, "xyz")), f3(a3(j, "rpy")));
            if jt != "fixed" {
                js += &format!("<axis xyz=\"{}\"/>", f3(a3(j, "axis")));
                let (eff, vel) = (n(j, "effort", 200.0), n(j, "velocity", if jt == "prismatic" { 0.5 } else { 30.0 }));
                if jt == "continuous" { js += &format!("<limit effort=\"{eff}\" velocity=\"{vel}\"/>"); }
                else { js += &format!("<limit lower=\"{:.4}\" upper=\"{:.4}\" effort=\"{eff}\" velocity=\"{vel}\"/>", n(j, "lower", 0.0), n(j, "upper", 0.0)); }
                if let Some(m) = j.get("mimic").filter(|m| m.is_object()) {
                    if s(m, "joint") != s(j, "name") { js += &format!("<mimic joint=\"{}\" multiplier=\"{}\" offset=\"0\"/>", s(m, "joint"), n(m, "multiplier", 1.0)); }
                }
            }
            js += "</joint>";
            x.push(js);
        }
    }
    // IMU, optical frames, photoelectrics
    let imu = spec.get("imu").cloned().unwrap_or(Value::Null);
    x.push(format!("  <link name=\"imu_link\">{SMALL}</link>"));
    x.push(format!("  <joint name=\"imu_joint\" type=\"fixed\"><parent link=\"base_link\"/><child link=\"imu_link\"/><origin xyz=\"{:.4} {:.4} {:.4}\" rpy=\"0 0 0\"/></joint>", n(&imu, "x", 0.0), n(&imu, "y", 0.0), n(&imu, "z", 0.2)));
    let opt = format!("{:.5} 0 {:.5}", -PI / 2.0, -PI / 2.0);
    for cam in spec.get("cameras").and_then(|c| c.as_array()).cloned().unwrap_or_default() {
        let nm = s(&cam, "name");
        x.push(format!("  <link name=\"{nm}_optical_frame\"/>"));
        x.push(format!("  <joint name=\"{nm}_optical_joint\" type=\"fixed\"><parent link=\"{nm}_link\"/><child link=\"{nm}_optical_frame\"/><origin xyz=\"0 0 0\" rpy=\"{opt}\"/></joint>"));
        if s(&cam, "type") == "stereo" {
            let b = n(&cam, "baseline_m", 0.05);
            x.push(format!("  <link name=\"{nm}_right_optical_frame\"/>"));
            x.push(format!("  <joint name=\"{nm}_right_optical_joint\" type=\"fixed\"><parent link=\"{nm}_link\"/><child link=\"{nm}_right_optical_frame\"/><origin xyz=\"0 {:.4} 0\" rpy=\"{opt}\"/></joint>", -b));
        }
    }
    for pe in spec.get("photoelectric").and_then(|p| p.as_array()).cloned().unwrap_or_default() {
        let nm = s(&pe, "name");
        x.push(format!("  <link name=\"{nm}_pe_link\"><visual><geometry><box size=\"0.02 0.02 0.02\"/></geometry></visual>{SMALL}</link>"));
        x.push(format!("  <joint name=\"{nm}_pe_joint\" type=\"fixed\"><parent link=\"base_link\"/><child link=\"{nm}_pe_link\"/><origin xyz=\"{:.4} {:.4} {:.4}\" rpy=\"0 0 {:.5}\"/></joint>", n(&pe, "x", 0.0), n(&pe, "y", 0.0), n(&pe, "z", 0.1), n(&pe, "yaw", 0.0)));
    }
    x.push("</robot>".into());
    x.join("\n") + "\n"
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;
    #[test]
    fn urdf_from_assembly_is_wellformed() {
        let spec = json!({"robot_name": "t", "chassis": {"type": "diff_drive", "head_offset_m": 0.7, "tail_offset_m": 0.7, "left_offset_m": 0.5, "right_offset_m": 0.5, "height_m": 0.18, "mass_kg": 200.0},
            "wheels": [{"name": "wl", "kind": "drive", "x": 0.0, "y": 0.45, "radius_m": 0.065}, {"name": "wr", "kind": "drive", "x": 0.0, "y": -0.45, "radius_m": 0.065}, {"name": "c1", "kind": "caster", "x": 0.5, "y": 0.3, "radius_m": 0.04}],
            "lidars": [{"name": "laser", "x": 0.7, "z": 0.08}], "cameras": [{"name": "cr", "type": "codeReader", "orientation": "LENS_DIR_DOWN", "z": 0.1}],
            "other_actuators": [{"name": "motor-lift"}], "io": {"inputs": [{"name": "up_sensor"}]}});
        let u = generate_urdf(&spec);
        for k in ["<joint name=\"lift_joint\" type=\"prismatic\"", "<joint name=\"tray_rotate_joint\" type=\"continuous\"", "c1_swivel_joint", "laser_joint", "cr_optical_frame", "[defaulted] actuator.stroke_m"] { assert!(u.contains(k), "missing {k}"); }
        // every joint's parent and child are declared links
        let links: Vec<&str> = u.match_indices("<link name=\"").map(|(i, _)| { let r = &u[i + 12..]; &r[..r.find('"').unwrap()] }).collect();
        for (i, _) in u.match_indices("<parent link=\"") { let r = &u[i + 14..]; assert!(links.contains(&&r[..r.find('"').unwrap()])); }
        for (i, _) in u.match_indices("<child link=\"") { let r = &u[i + 13..]; assert!(links.contains(&&r[..r.find('"').unwrap()])); }
        assert_eq!(u.matches("<robot ").count(), 1);
    }
}
