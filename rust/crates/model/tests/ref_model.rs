use model::*;
use serde_json::Value;
fn diff(a: &Value, b: &Value, p: &str, out: &mut Vec<String>) {
    match (a, b) {
        (Value::Object(x), Value::Object(y)) => {
            for (k, v) in x { match y.get(k) { Some(w) => diff(v, w, &format!("{p}.{k}"), out), None => out.push(format!("{p}.{k}: only in rust")) } }
            for k in y.keys() { if !x.contains_key(k) { out.push(format!("{p}.{k}: only in python")); } }
        }
        (Value::Array(x), Value::Array(y)) => { if x.len() != y.len() { out.push(format!("{p}: len {} vs {}", x.len(), y.len())); } for (i, (v, w)) in x.iter().zip(y).enumerate() { diff(v, w, &format!("{p}[{i}]"), out); } }
        (Value::Number(x), Value::Number(y)) => { let (f, g) = (x.as_f64().unwrap(), y.as_f64().unwrap()); if (f - g).abs() > 1e-9 * (1.0 + g.abs()) { out.push(format!("{p}: {f} vs {g}")); } }
        _ => if a != b { out.push(format!("{p}: {a} vs {b}")); }
    }
}
#[test]
fn model_matches_python() {
    let r: Value = serde_json::from_str(&std::fs::read_to_string("/tmp/ref_model.json").unwrap()).unwrap();
    let mut all = vec![];
    for tag in ["a", "b"] {
        let t = &r[tag];
        let sp = overrides::apply_overrides(&r["base"], &t["ov"]);
        let mut o = vec![]; diff(&sp, &t["spec"], &format!("{tag}.spec"), &mut o); all.extend(o);
        let mut o = vec![]; diff(&Value::Array(overrides::sensor_list(&t["spec"], None)), &t["sensor_list"], &format!("{tag}.sensors"), &mut o); all.extend(o);
        let mut o = vec![]; diff(&overrides::audit(&t["spec"], Some(&t["ov"])), &t["audit"], &format!("{tag}.audit"), &mut o); all.extend(o);
        let u = urdf::generate_urdf(&t["spec"]);
        if u != t["urdf"].as_str().unwrap() {
            let py = t["urdf"].as_str().unwrap();
            for (i, (a, b)) in u.lines().zip(py.lines()).enumerate() { if a != b { all.push(format!("{tag}.urdf line {i}:\n rs {a}\n py {b}")); break; } }
            all.push(format!("{tag}.urdf lines {} vs {}", u.lines().count(), py.lines().count()));
        }
        for c in limits::CHASSIS_TYPES {
            let l = limits::effective_limits(&t["spec"], c).unwrap();
            let mut o = vec![]; diff(&l, &t["limits"][c], &format!("{tag}.limits.{c}"), &mut o); all.extend(o);
        }
    }
    for (k, v) in r["tmpl"].as_object().unwrap() { let mut o = vec![]; diff(&overrides::sensor_template(k).unwrap(), v, &format!("tmpl.{k}"), &mut o); all.extend(o); }
    for l in all.iter().take(60) { eprintln!("{l}"); }
    assert!(all.is_empty(), "{} diffs", all.len());
}
