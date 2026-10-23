//! xoshiro256** + Box-Muller, bit-compatible with simcore.c (per-sensor state).
#[derive(Clone, Debug)]
pub struct Rng {
    s: [u64; 4],
    spare: Option<f64>,
}

#[inline]
fn rotl(x: u64, k: u32) -> u64 { x.rotate_left(k) }

impl Rng {
    pub fn new(seed: u64) -> Self {
        let mut seed = seed;
        let mut s = [0u64; 4];
        for v in s.iter_mut() {
            seed = seed.wrapping_add(0x9e3779b97f4a7c15);
            let mut z = seed;
            z = (z ^ (z >> 30)).wrapping_mul(0xbf58476d1ce4e5b9);
            z = (z ^ (z >> 27)).wrapping_mul(0x94d049bb133111eb);
            *v = z ^ (z >> 31);
        }
        Rng { s, spare: None }
    }
    #[inline]
    pub fn next_u64(&mut self) -> u64 {
        let s = &mut self.s;
        let r = rotl(s[1].wrapping_mul(5), 7).wrapping_mul(9);
        let t = s[1] << 17;
        s[2] ^= s[0]; s[3] ^= s[1]; s[1] ^= s[2]; s[0] ^= s[3];
        s[2] ^= t;
        s[3] = rotl(s[3], 45);
        r
    }
    #[inline]
    pub fn uniform(&mut self) -> f64 { (self.next_u64() >> 11) as f64 * (1.0 / 9007199254740992.0) }
    pub fn gauss(&mut self) -> f64 {
        if let Some(v) = self.spare.take() { return v; }
        let mut u1;
        loop { u1 = self.uniform(); if u1 > 1e-300 { break; } }
        let u2 = self.uniform();
        let r = (-2.0 * u1.ln()).sqrt();
        let a = 2.0 * std::f64::consts::PI * u2;
        self.spare = Some(r * a.sin());
        r * a.cos()
    }
}
