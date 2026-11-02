//! Minimal PNG writer (host only, no dependencies): 8-bit RGB, stored
//! (uncompressed) deflate blocks. For previews and contact sheets.

use std::path::Path;

fn crc32(data: &[u8]) -> u32 {
    let mut c = !0u32;
    for &b in data {
        c ^= b as u32;
        for _ in 0..8 {
            c = if c & 1 != 0 { 0xEDB8_8320 ^ (c >> 1) } else { c >> 1 };
        }
    }
    !c
}

fn chunk(out: &mut Vec<u8>, kind: &[u8; 4], data: &[u8]) {
    out.extend_from_slice(&(data.len() as u32).to_be_bytes());
    let start = out.len();
    out.extend_from_slice(kind);
    out.extend_from_slice(data);
    let crc = crc32(&out[start..]);
    out.extend_from_slice(&crc.to_be_bytes());
}

pub fn encode(w: u32, h: u32, rgb: &[[u8; 3]]) -> Vec<u8> {
    assert_eq!(rgb.len(), (w * h) as usize);
    let mut raw = Vec::with_capacity(((w * 3 + 1) * h) as usize);
    for row in rgb.chunks(w as usize) {
        raw.push(0);
        raw.extend(row.iter().flatten());
    }
    let mut z = vec![0x78, 0x01];
    let blocks: Vec<&[u8]> = raw.chunks(65535).collect();
    for (i, b) in blocks.iter().enumerate() {
        z.push((i + 1 == blocks.len()) as u8);
        z.extend_from_slice(&(b.len() as u16).to_le_bytes());
        z.extend_from_slice(&(!(b.len() as u16)).to_le_bytes());
        z.extend_from_slice(b);
    }
    let (mut s1, mut s2) = (1u32, 0u32);
    for &b in &raw {
        s1 = (s1 + b as u32) % 65521;
        s2 = (s2 + s1) % 65521;
    }
    z.extend_from_slice(&(s2 << 16 | s1).to_be_bytes());
    let mut out = b"\x89PNG\r\n\x1a\n".to_vec();
    let mut ihdr = Vec::new();
    ihdr.extend_from_slice(&w.to_be_bytes());
    ihdr.extend_from_slice(&h.to_be_bytes());
    ihdr.extend_from_slice(&[8, 2, 0, 0, 0]);
    chunk(&mut out, b"IHDR", &ihdr);
    chunk(&mut out, b"IDAT", &z);
    chunk(&mut out, b"IEND", &[]);
    out
}

pub fn save(path: &Path, w: u32, h: u32, rgb: &[[u8; 3]]) -> Result<(), String> {
    if let Some(dir) = path.parent() {
        std::fs::create_dir_all(dir).map_err(|e| format!("{}: {e}", dir.display()))?;
    }
    std::fs::write(path, encode(w, h, rgb)).map_err(|e| format!("{}: {e}", path.display()))
}

/// Save multicolor pixels with each pixel doubled horizontally (true aspect).
pub fn save_wide(path: &Path, w: usize, px: &[[u8; 3]]) -> Result<(), String> {
    let wide: Vec<[u8; 3]> = px.iter().flat_map(|&p| [p, p]).collect();
    save(path, 2 * w as u32, (px.len() / w) as u32, &wide)
}
