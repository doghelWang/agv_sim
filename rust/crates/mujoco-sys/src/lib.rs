//! Raw + thin safe MuJoCo bindings, loaded at runtime (dlopen) so the binary has no link-time
//! dependency on libmujoco and can verify the library version before use.
#![allow(non_camel_case_types, non_snake_case, non_upper_case_globals, dead_code, clippy::all)]
mod bindings;
pub use bindings::*;

pub const EXPECTED_VERSION: i32 = 3016000;

/// Load libmujoco from `path` (or default search) and check its version.
pub fn load(path: Option<&str>) -> Result<MuJoCo, String> {
    let cands: Vec<String> = match path {
        Some(p) => vec![p.to_string()],
        None => {
            let mut v = Vec::new();
            if let Ok(p) = std::env::var("MUJOCO_LIB") { v.push(p); }
            v.push("libmujoco.so.3.16.0".into());
            v.push("libmujoco.so".into());
            v
        }
    };
    let mut last = String::new();
    for c in cands {
        match unsafe { MuJoCo::new(&c) } {
            Ok(lib) => {
                let ver = unsafe { lib.mj_version() };
                if ver != EXPECTED_VERSION {
                    return Err(format!("libmujoco version {ver} != expected {EXPECTED_VERSION}"));
                }
                return Ok(lib);
            }
            Err(e) => last = format!("{c}: {e}"),
        }
    }
    Err(last)
}
