//! Floor QR codes: real QR symbols (ISO/IEC 18004, via `qrcodegen`) rendered as high-resolution decals for the camera shader,
//! plus decoding for the downward code readers (a symbol is only "decoded" when the reader's field of view covers it fully).
use qrcodegen::{QrCode, QrCodeEcc};
use std::collections::HashMap;

pub const TAG_SIZE_M: f64 = 0.08;
pub const QUIET_MODULES: usize = 2;

#[derive(Clone, Debug)]
pub struct Decal {
    pub id: String,
    pub payload: String,
    pub x: f64, pub y: f64, pub yaw: f64,
    /// printed label size incl. quiet zone (m)
    pub size: f64,
    /// modules per side incl. quiet zone
    pub n: usize,
    pub bits: Vec<bool>,
}

impl Decal {
    pub fn new(id: &str, payload: &str, x: f64, y: f64, yaw: f64, size: f64) -> Decal {
        let qr = QrCode::encode_text(payload, QrCodeEcc::Medium).unwrap_or_else(|_| QrCode::encode_text("?", QrCodeEcc::Low).unwrap());
        let s = qr.size() as usize;
        let n = s + 2 * QUIET_MODULES;
        let mut bits = vec![false; n * n];
        for j in 0..s { for i in 0..s { bits[(j + QUIET_MODULES) * n + i + QUIET_MODULES] = qr.get_module(i as i32, j as i32); } }
        Decal { id: id.into(), payload: payload.into(), x, y, yaw, size, n, bits }
    }
    /// Some(true) = dark module, Some(false) = white label, None = outside the label
    pub fn sample(&self, px: f64, py: f64) -> Option<bool> {
        let (s, c) = self.yaw.sin_cos();
        let (dx, dy) = (px - self.x, py - self.y);
        let (lx, ly) = (c * dx + s * dy, -s * dx + c * dy);
        let h = self.size / 2.0;
        if lx.abs() >= h || ly.abs() >= h { return None; }
        let m = self.size / self.n as f64;
        // module (0,0) at the label's top-left as seen from above with +x right / +y up
        let i = ((lx + h) / m) as usize;
        let j = ((h - ly) / m) as usize;
        Some(self.bits[j.min(self.n - 1) * self.n + i.min(self.n - 1)])
    }
}

/// spatial hash of decals (0.25 m cells)
#[derive(Clone, Debug, Default)]
pub struct DecalGrid { pub decals: Vec<Decal>, cells: HashMap<(i32, i32), Vec<u32>> }

const CELL: f64 = 0.25;
impl DecalGrid {
    pub fn new(decals: Vec<Decal>) -> DecalGrid {
        let mut cells: HashMap<(i32, i32), Vec<u32>> = HashMap::new();
        for (k, d) in decals.iter().enumerate() {
            let r = d.size * 0.75;
            for cx in ((d.x - r) / CELL).floor() as i32..=((d.x + r) / CELL).floor() as i32 {
                for cy in ((d.y - r) / CELL).floor() as i32..=((d.y + r) / CELL).floor() as i32 { cells.entry((cx, cy)).or_default().push(k as u32); }
            }
        }
        DecalGrid { decals, cells }
    }
    pub fn sample(&self, px: f64, py: f64) -> Option<bool> {
        let c = ((px / CELL).floor() as i32, (py / CELL).floor() as i32);
        for &k in self.cells.get(&c)? { if let Some(v) = self.decals[k as usize].sample(px, py) { return Some(v); } }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn qr_has_finder_patterns() {
        let d = Decal::new("QR_A", "QR_A", 1.0, 2.0, 0.0, TAG_SIZE_M);
        assert_eq!(d.n, 21 + 2 * QUIET_MODULES);
        // finder pattern top-left: dark ring corner module
        let m = d.size / d.n as f64;
        let (x0, y0) = (1.0 - d.size / 2.0 + (QUIET_MODULES as f64 + 0.5) * m, 2.0 + d.size / 2.0 - (QUIET_MODULES as f64 + 0.5) * m);
        assert_eq!(d.sample(x0, y0), Some(true));
        assert_eq!(d.sample(1.0 - d.size / 2.0 + 0.5 * m, y0), Some(false)); // quiet zone
        assert_eq!(d.sample(5.0, 5.0), None);
        let g = DecalGrid::new(vec![d]);
        assert_eq!(g.sample(x0, y0), Some(true));
    }
}
