//! World geometry: static walls/shelves + dynamic obstacles as vertical segments; bounds; primitive list for ray casting.
use crate::geom::Seg;
use serde_json::{Map, Value};

pub const WALL_HEIGHT: f64 = 6.0;
pub const CEILING_HEIGHT: f64 = 6.0;
pub const SHELF_HEIGHT: f64 = 2.5;
pub const OBSTACLE_HEIGHT: f64 = 1.0;

#[derive(Clone, Debug)]
pub struct World {
    pub static_segments: Vec<Seg>,
    pub dynamic_segments: Vec<Seg>,
    pub segments: Vec<Seg>,
    pub obstacles: Vec<Map<String, Value>>,
    pub bounds: [f64; 4], // minx miny maxx maxy
}

fn onum(o: &Map<String, Value>, k: &str, d: f64) -> f64 { o.get(k).and_then(|x| x.as_f64()).unwrap_or(d) }

impl World {
    pub fn new() -> World {
        World { static_segments: vec![], dynamic_segments: vec![], segments: vec![], obstacles: vec![], bounds: [-10.0, -10.0, 10.0, 10.0] }
    }

    pub fn load_scenario(&mut self, sc: &Value) {
        let walls = sc.get("walls").and_then(|w| w.as_array()).cloned().unwrap_or_default();
        self.static_segments = walls.iter().enumerate().map(|(i, w)| {
            let a: Vec<f64> = w.as_array().map(|a| a.iter().map(|x| x.as_f64().unwrap_or(0.0)).collect()).unwrap_or_default();
            let h = if i < 4 { WALL_HEIGHT } else { SHELF_HEIGHT };
            [a[0], a[1], a[2], a[3], h]
        }).collect();
        if self.static_segments.len() >= 4 {
            let s = &self.static_segments[..4];
            let xs: Vec<f64> = s.iter().flat_map(|w| [w[0], w[2]]).collect();
            let ys: Vec<f64> = s.iter().flat_map(|w| [w[1], w[3]]).collect();
            self.bounds = [xs.iter().cloned().fold(f64::INFINITY, f64::min), ys.iter().cloned().fold(f64::INFINITY, f64::min),
                xs.iter().cloned().fold(f64::NEG_INFINITY, f64::max), ys.iter().cloned().fold(f64::NEG_INFINITY, f64::max)];
        }
        self.set_obstacles(vec![]);
    }

    pub fn set_obstacles(&mut self, obs: Vec<Map<String, Value>>) {
        self.obstacles = obs;
        self.rebuild_dynamic();
    }

    pub fn rebuild_dynamic(&mut self) {
        let mut segs = vec![];
        for o in &self.obstacles {
            if o.get("carried").and_then(|v| v.as_bool()).unwrap_or(false) { continue; }
            let (ox, oy) = (onum(o, "x", 0.0), onum(o, "y", 0.0));
            let (hw, hh) = (onum(o, "w", 0.8) / 2.0, onum(o, "h", 0.8) / 2.0);
            let z = o.get("z").and_then(|v| v.as_f64()).or_else(|| o.get("height").and_then(|v| v.as_f64())).unwrap_or(OBSTACLE_HEIGHT);
            let yaw = onum(o, "yaw", 0.0);
            let (s, c) = yaw.sin_cos();
            let pts: Vec<(f64, f64)> = [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)].iter().map(|(dx, dy)| (ox + c * dx - s * dy, oy + s * dx + c * dy)).collect();
            for i in 0..4 {
                let (a, b) = (pts[i], pts[(i + 1) % 4]);
                segs.push([a.0, a.1, b.0, b.1, z]);
            }
        }
        self.dynamic_segments = segs;
        self.segments = self.static_segments.iter().chain(self.dynamic_segments.iter()).cloned().collect();
    }
}

/// Ray-cast primitive (oriented box / cylinder), 18 doubles, same layout as the C kernel.
/// [0..3] center, [3..12] rotation (row-major local->world), [12..15] half size (cyl: r,r,hz), [15] kind, [16] geom id, [17] bound radius
pub type Prim = [f64; 18];

pub fn rot_z(yaw: f64) -> [f64; 9] {
    let (s, c) = yaw.sin_cos();
    [c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0]
}

/// Category colours for camera shading (floor, wall, shelf, obstacle, ceiling, station, tag, robot, lane)
pub const CAT_NAMES: [&str; 9] = ["floor", "wall", "shelf", "obstacle", "ceiling", "station", "tag", "robot", "lane"];
pub const CAT_COLORS: [[f32; 3]; 9] = [
    [0.78, 0.78, 0.76], [0.82, 0.83, 0.86], [0.20, 0.32, 0.55], [0.95, 0.55, 0.15], [0.92, 0.92, 0.94],
    [0.10, 0.70, 0.35], [0.05, 0.05, 0.05], [0.95, 0.55, 0.10], [0.95, 0.80, 0.10],
];
pub fn cat_id(name: &str) -> usize { CAT_NAMES.iter().position(|n| *n == name).unwrap_or(1) }

pub fn obstacle_color(t: Option<&str>) -> [f32; 3] {
    match t { Some("pallet") => [0.62, 0.45, 0.25], Some("person") => [0.25, 0.45, 0.85], Some("shelf") => [0.35, 0.40, 0.50], _ => CAT_COLORS[3] }
}

