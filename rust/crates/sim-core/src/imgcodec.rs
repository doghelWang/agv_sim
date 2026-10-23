//! Image encoding: PNG (8-bit gray / 16-bit gray / RGB), JPEG, depth colorization
pub fn png_rgb(w: usize, h: usize, rgb: &[u8]) -> Vec<u8> { enc(w, h, png::ColorType::Rgb, png::BitDepth::Eight, rgb) }
pub fn png_gray8(w: usize, h: usize, g: &[u8]) -> Vec<u8> { enc(w, h, png::ColorType::Grayscale, png::BitDepth::Eight, g) }
pub fn png_gray16(w: usize, h: usize, g: &[u16]) -> Vec<u8> {
    let mut b = Vec::with_capacity(g.len() * 2);
    for v in g { b.extend_from_slice(&v.to_be_bytes()); }
    enc(w, h, png::ColorType::Grayscale, png::BitDepth::Sixteen, &b)
}
fn enc(w: usize, h: usize, ct: png::ColorType, bd: png::BitDepth, data: &[u8]) -> Vec<u8> {
    let mut out = vec![];
    {
        let mut e = png::Encoder::new(&mut out, w as u32, h as u32);
        e.set_color(ct); e.set_depth(bd);
        e.set_compression(png::Compression::Fast);
        let mut wr = e.write_header().unwrap();
        wr.write_image_data(data).unwrap();
    }
    out
}
pub fn jpeg(w: usize, h: usize, rgb: &[u8], quality: u8) -> Vec<u8> {
    let mut out = vec![];
    let e = jpeg_encoder::Encoder::new(&mut out, quality);
    e.encode(rgb, w as u16, h as u16, jpeg_encoder::ColorType::Rgb).unwrap();
    out
}
pub fn colorize_depth(z: &[f32], zmin: f64, zmax: f64) -> Vec<u8> {
    let mut o = vec![0u8; z.len() * 3];
    let span = (zmax - zmin).max(1e-6);
    for (i, &v) in z.iter().enumerate() {
        if !v.is_finite() { continue; }
        let t = ((v as f64 - zmin) / span).clamp(0.0, 1.0);
        let f = |k: f64| ((1.5 - (4.0 * t - k).abs()).clamp(0.0, 1.0) * 255.0) as u8;
        o[3 * i] = f(3.0); o[3 * i + 1] = f(2.0); o[3 * i + 2] = f(1.0);
    }
    o
}
