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
            let z = o.get("z").and_then(|v| v.as_f64()).or_else(|| o.get("height").and_then(|v| v.as_f64())).unwrap_or(OBSTACLE_HEIGHT);
            // footprint = floor-standing parts (rack legs, pallet blocks ...); racks/carts let the robot pass between legs
            let typ = o.get("type").and_then(|t| t.as_str()).unwrap_or("");
            let kind = o.get("kind").and_then(|t| t.as_str()).unwrap_or("");
            let partial = matches!(kind, "rack") || typ == "rack";
            if partial {
                for p in obstacle_parts(o).iter().filter(|p| p.foot) {
                    let pts = part_corners(o, p);
                    for i in 0..4 { let (a, b) = (pts[i], pts[(i + 1) % 4]); segs.push([a.0, a.1, b.0, b.1, p.c[2] + p.half[2]]); }
                }
                continue;
            }
            let (ox, oy) = (onum(o, "x", 0.0), onum(o, "y", 0.0));
            let (hw, hh) = (onum(o, "w", 0.8) / 2.0, onum(o, "h", 0.8) / 2.0);
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


// ------------------------------------------------------------------ obstacle part decomposition
/// Surface material used by the camera shader for procedural texture detail
pub const MAT_PLAIN: u8 = 0;
pub const MAT_WALL: u8 = 1;
pub const MAT_WOOD: u8 = 2;
pub const MAT_CARDBOARD: u8 = 3;
pub const MAT_HAZARD: u8 = 4;
pub const MAT_STEEL: u8 = 5;
pub const MAT_ROLLER: u8 = 6;
pub const MAT_CONCRETE: u8 = 7;

/// One primitive of an obstacle, in the obstacle frame (origin at the obstacle's floor center, z up from its `z0`).
#[derive(Clone, Debug)]
pub struct Part {
    pub cyl: bool,
    pub c: [f64; 3],
    pub half: [f64; 3],
    /// yaw of the part in the obstacle frame (box only); cylinders are vertical unless `axis_y`
    pub yaw: f64,
    /// horizontal cylinder along the obstacle's local y axis (rollers, wheels)
    pub axis_y: bool,
    pub rgb: [f32; 3],
    pub mat: u8,
    /// counts as floor-standing footprint for 2D collision / bumper / kinematic lidar segments
    pub foot: bool,
}

fn pbox(c: [f64; 3], size: [f64; 3], rgb: [f32; 3], mat: u8, foot: bool) -> Part {
    Part { cyl: false, c, half: [size[0] / 2.0, size[1] / 2.0, size[2] / 2.0], yaw: 0.0, axis_y: false, rgb, mat, foot }
}

pub const WOOD: [f32; 3] = [0.62, 0.45, 0.25];
pub const WOOD_LIGHT: [f32; 3] = [0.80, 0.62, 0.40];
pub const CARDBOARD: [f32; 3] = [0.76, 0.58, 0.38];
pub const RACK_BLUE: [f32; 3] = [0.15, 0.32, 0.62];
pub const RACK_ORANGE: [f32; 3] = [0.92, 0.45, 0.10];
pub const STEEL: [f32; 3] = [0.55, 0.58, 0.62];
pub const DARK: [f32; 3] = [0.16, 0.18, 0.22];
pub const YELLOW: [f32; 3] = [0.98, 0.80, 0.08];
pub const CONCRETE: [f32; 3] = [0.70, 0.70, 0.68];
pub const GREEN: [f32; 3] = [0.12, 0.62, 0.32];
pub const TOTE_BLUE: [f32; 3] = [0.15, 0.40, 0.80];

/// Decompose an obstacle into primitives (pallet blocks + deck, rack legs + shelves, cart frame + casters, conveyor bed + rollers ...).
/// The decomposition is shared by MuJoCo (geoms → lidar / contacts), the camera ray caster, PCD export and 2D footprints.
pub fn obstacle_parts(o: &Map<String, Value>) -> Vec<Part> {
    let typ = o.get("type").and_then(|t| t.as_str()).unwrap_or("box");
    let kind = o.get("kind").and_then(|t| t.as_str()).unwrap_or(typ);
    let (w, h) = (onum(o, "w", 0.8), onum(o, "h", 0.8));
    let z = o.get("z").and_then(|v| v.as_f64()).or_else(|| o.get("height").and_then(|v| v.as_f64())).unwrap_or(OBSTACLE_HEIGHT);
    let mut v = vec![];
    match (typ, kind) {
        ("person", _) => {
            let r = w.max(h) / 2.0;
            v.push(Part { cyl: true, c: [0.0, 0.0, z / 2.0], half: [r, r, z / 2.0], yaw: 0.0, axis_y: false, rgb: obstacle_color(Some("person")), mat: MAT_PLAIN, foot: true });
        }
        ("pallet", _) => {
            let ph = onum(o, "pocket_h", 0.10);
            let b = 0.13;
            for bx in [-(w / 2.0 - b / 2.0), 0.0, w / 2.0 - b / 2.0] {
                for by in [-(h / 2.0 - b / 2.0), 0.0, h / 2.0 - b / 2.0] { v.push(pbox([bx, by, ph / 2.0], [b, b, ph], WOOD, MAT_WOOD, true)); }
            }
            v.push(pbox([0.0, 0.0, ph + (z - ph) / 2.0], [w, h, (z - ph).max(0.01)], WOOD_LIGHT, MAT_WOOD, false));
        }
        (_, "rack") | ("rack", _) => {
            // jacking shelf rack: 4 legs (the AMR drives underneath), bottom deck at `leg_h`, two upper shelves
            let lh = onum(o, "leg_h", 0.2);
            let lw = onum(o, "leg_w", 0.05);
            for (sx, sy) in [(1.0, 1.0), (1.0, -1.0), (-1.0, 1.0), (-1.0, -1.0)] {
                v.push(pbox([sx * (w / 2.0 - lw / 2.0), sy * (h / 2.0 - lw / 2.0), z / 2.0], [lw, lw, z], RACK_BLUE, MAT_STEEL, true));
            }
            v.push(pbox([0.0, 0.0, lh + 0.015], [w, h, 0.03], RACK_ORANGE, MAT_STEEL, false));
            let up = z - lh;
            for k in [0.5, 1.0] { v.push(pbox([0.0, 0.0, lh + up * k - 0.015], [w, h, 0.03], RACK_ORANGE, MAT_STEEL, false)); }
            // a tote on the middle shelf so the camera sees "goods"
            if up > 0.6 { v.push(pbox([0.0, 0.0, lh + up * 0.5 + 0.13], [w * 0.55, h * 0.6, 0.23], TOTE_BLUE, MAT_PLAIN, false)); }
        }
        (_, "cart") | ("cart", _) => {
            // tow cart: deck on four casters, frame posts, drawbar with ring at +x
            let deck = onum(o, "deck_z", 0.28);
            let dl = onum(o, "drawbar_m", 0.35);
            let hz = onum(o, "hitch_z", 0.3);
            v.push(pbox([0.0, 0.0, deck], [w, h, 0.04], STEEL, MAT_STEEL, true));
            for (sx, sy) in [(1.0, 1.0), (1.0, -1.0), (-1.0, 1.0), (-1.0, -1.0)] {
                v.push(Part { cyl: true, c: [sx * (w / 2.0 - 0.1), sy * (h / 2.0 - 0.08), 0.06], half: [0.06, 0.06, 0.025], yaw: 0.0, axis_y: true, rgb: DARK, mat: MAT_PLAIN, foot: false });
                v.push(pbox([-w / 2.0 + 0.03, sy * (h / 2.0 - 0.03), (deck + z) / 2.0], [0.04, 0.04, z - deck], STEEL, MAT_STEEL, false));
                let _ = sx;
            }
            v.push(pbox([-w / 2.0 + 0.03, 0.0, z - 0.02], [0.04, h, 0.04], STEEL, MAT_STEEL, false));
            v.push(pbox([0.0, 0.0, z - 0.02], [w, h, 0.03], STEEL, MAT_STEEL, false));
            v.push(pbox([w / 2.0 + dl / 2.0, 0.0, hz], [dl, 0.05, 0.03], YELLOW, MAT_HAZARD, false));
            v.push(Part { cyl: true, c: [w / 2.0 + dl, 0.0, hz], half: [0.045, 0.045, 0.015], yaw: 0.0, axis_y: false, rgb: DARK, mat: MAT_STEEL, foot: false });
            // goods on the deck
            v.push(pbox([0.05, 0.0, deck + 0.22], [w * 0.7, h * 0.75, 0.4], CARDBOARD, MAT_CARDBOARD, false));
        }
        ("conveyor", _) => {
            // fixed roller conveyor station: frame + bed + rollers (rollers along the local y axis transfer along x... we use y transfer: rollers axis = x)
            let bed = 0.06;
            for (sx, sy) in [(1.0, 1.0), (1.0, -1.0), (-1.0, 1.0), (-1.0, -1.0)] {
                v.push(pbox([sx * (w / 2.0 - 0.04), sy * (h / 2.0 - 0.04), (z - bed) / 2.0], [0.05, 0.05, z - bed], DARK, MAT_STEEL, true));
            }
            v.push(pbox([0.0, h / 2.0 - 0.02, z - bed / 2.0 - 0.01], [w, 0.04, bed], STEEL, MAT_STEEL, true));
            v.push(pbox([0.0, -h / 2.0 + 0.02, z - bed / 2.0 - 0.01], [w, 0.04, bed], STEEL, MAT_STEEL, true));
            let axis_x = o.get("roll_axis").and_then(|a| a.as_str()).unwrap_or("x") == "x";
            let (len_dir, n) = if axis_x { (h, ((h - 0.08) / 0.09).floor().max(2.0) as usize) } else { (w, ((w - 0.08) / 0.09).floor().max(2.0) as usize) };
            for i in 0..n {
                let t = -len_dir / 2.0 + 0.04 + 0.045 + i as f64 * 0.09;
                if axis_x { v.push(Part { cyl: true, c: [0.0, t, z - 0.03], half: [0.03, 0.03, w / 2.0 - 0.03], yaw: std::f64::consts::FRAC_PI_2, axis_y: true, rgb: [0.80, 0.82, 0.86], mat: MAT_ROLLER, foot: false }); }
                else { v.push(Part { cyl: true, c: [t, 0.0, z - 0.03], half: [0.03, 0.03, h / 2.0 - 0.03], yaw: 0.0, axis_y: true, rgb: [0.80, 0.82, 0.86], mat: MAT_ROLLER, foot: false }); }
            }
            v.push(pbox([0.0, 0.0, (z - bed) * 0.3], [w - 0.08, h - 0.08, 0.02], DARK, MAT_STEEL, false));
        }
        ("charger", _) => {
            v.push(pbox([0.0, 0.0, 0.04], [w, h, 0.08], DARK, MAT_STEEL, true));
            v.push(pbox([-w / 2.0 + 0.07, 0.0, z / 2.0], [0.14, h, z], [0.92, 0.93, 0.95], MAT_PLAIN, true));
            v.push(pbox([-w / 2.0 + 0.145, 0.0, 0.12], [0.01, h * 0.5, 0.05], [0.85, 0.65, 0.15], MAT_STEEL, false));
            v.push(pbox([-w / 2.0 + 0.145, 0.0, z - 0.12], [0.01, h * 0.6, 0.06], GREEN, MAT_PLAIN, false));
        }
        ("pillar", _) => {
            v.push(pbox([0.0, 0.0, z / 2.0 + 0.5], [w, h, z - 1.0], CONCRETE, MAT_CONCRETE, true));
            v.push(pbox([0.0, 0.0, 0.5], [w + 0.02, h + 0.02, 1.0], YELLOW, MAT_HAZARD, true));
        }
        ("rack_upright", _) => { v.push(pbox([0.0, 0.0, z / 2.0], [w, h, z], RACK_BLUE, MAT_STEEL, true)); }
        ("rack_beam", _) => { v.push(pbox([0.0, 0.0, z / 2.0], [w, h, z], RACK_ORANGE, MAT_STEEL, false)); }
        ("barrier", _) => {
            v.push(pbox([0.0, 0.0, z / 2.0], [w, h, z], YELLOW, MAT_HAZARD, true));
        }
        (_, "box") | (_, "tote") => {
            let c = if kind == "tote" { TOTE_BLUE } else { CARDBOARD };
            v.push(pbox([0.0, 0.0, z / 2.0], [w, h, z], c, if kind == "tote" { MAT_PLAIN } else { MAT_CARDBOARD }, true));
        }
        _ => {
            v.push(pbox([0.0, 0.0, z / 2.0], [w, h, z], obstacle_color(Some(typ)), MAT_PLAIN, true));
        }
    }
    v
}

/// world-frame rectangle corners of a part (for 2D footprint segments)
pub fn part_corners(o: &Map<String, Value>, p: &Part) -> [(f64, f64); 4] {
    let (ox, oy, yaw) = (onum(o, "x", 0.0), onum(o, "y", 0.0), onum(o, "yaw", 0.0));
    let (s, c) = yaw.sin_cos();
    let (ps, pc) = p.yaw.sin_cos();
    let (hx, hy) = if p.cyl && p.axis_y { (p.half[0], p.half[2]) } else { (p.half[0], p.half[1]) };
    let mut out = [(0.0, 0.0); 4];
    for (k, (dx, dy)) in [(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)].iter().enumerate() {
        let (lx, ly) = (p.c[0] + pc * dx - ps * dy, p.c[1] + ps * dx + pc * dy);
        out[k] = (ox + c * lx - s * ly, oy + s * lx + c * ly);
    }
    out
}
