//! The REU package (host): planes from [`crate::map`], plus the per-angle line tables, the camera path,
//! the header and the fog bank; and the host's render of a path frame, as the C64 draws it.

use crate::camera::{self, View};
use crate::format::*;
use crate::geom::{self, Camera};
use crate::layout::{self, BUFFERS, Buffer};
use crate::map;
use crate::mode7::{self as m7, DmaMode, Line};
use crate::model::Machine;
use std::f32::consts::TAU;

/// Lines whose step exceeds this (texels per multicolour pixel) are drawn as haze ([`FOG_BANK`]).
pub const FOG_STEP: f32 = 5.0;

fn wrap(t: f32) -> u16 {
    ((t * 256.0).round() as i64).rem_euclid(65536) as u16
}

/// One angle's line table (`format::line_table` layout), camera at the origin.
pub fn line_table(angle: u8) -> Vec<u8> {
    let view = View::Floor {
        x: 0.0,
        y: 0.0,
        angle: angle as f32 / 256.0 * TAU,
        height: CAMERA_HEIGHT,
        horizon: HORIZON,
        focal: FOCAL,
    };
    let f = camera::frame(&view);
    let mut t = vec![0u8; LINE_BLOCK as usize];
    let mut put = |k: usize, l: usize, w: u16| {
        t[k * LINES + l] = w as u8;
        t[(k + 1) * LINES + l] = (w >> 8) as u8;
    };
    let mut banks = [SKY_BANK; LINES];
    for (l, e) in f.exact.iter().enumerate() {
        let Some((p, st)) = e else { continue };
        if (st[0] * st[0] + st[1] * st[1]).sqrt() > FOG_STEP {
            banks[l] = FOG_BANK;
            continue;
        }
        let (du, dv) = ((st[0] * 256.0).round() as i16, (st[1] * 256.0).round() as i16);
        put(field::U0, l, wrap(p[0]));
        put(field::V0, l, wrap(p[1]));
        put(field::DU4, l, (du as u16).wrapping_mul(4));
        put(field::DV4, l, (dv as u16).wrapping_mul(4));
        banks[l] = camera::variant(st[0], st[1]);
    }
    t[field::BANK * LINES..].copy_from_slice(&banks);
    t
}

/// The camera path: once round the circuit's centreline in [`PATH_LEN`] frames, looking along it.
pub fn path() -> Vec<(u16, u16, u8)> {
    (0..PATH_LEN)
        .map(|i| {
            let t = i as f32 / PATH_LEN as f32;
            let p = map::centre(t);
            let q = map::centre(t + 0.25 / PATH_LEN as f32);
            // View::Floor looks along (sin a, -cos a).
            let a = (q[0] - p[0]).atan2(-(q[1] - p[1]));
            let angle = ((a / TAU * 256.0).round() as i32).rem_euclid(256) as u8;
            (wrap(p[0]), wrap(p[1]), angle)
        })
        .collect()
}

/// The whole 16 MiB image.
pub fn bake(codes: &[u8]) -> Vec<u8> {
    let mut reu = map::bake(codes);
    for a in 0..=255u8 {
        let at = line_table(a);
        let base = line_table_addr(a);
        reu[base..base + at.len()].copy_from_slice(&at);
    }
    let (block, records) = crate::scene::bake();
    crate::scene::install(&mut reu, &block, &records);
    let fog = (FOG_BANK as usize) << 16;
    reu[fog..fog + 0x10000].fill(0x00);
    let h = HEADER as usize;
    reu[h..h + 8].copy_from_slice(MAGIC);
    reu[h + H_VERSION as usize..][..2].copy_from_slice(&PACKAGE_VERSION.to_le_bytes());
    reu[h + H_PATH_LEN as usize..][..2].copy_from_slice(&PATH_LEN.to_le_bytes());
    for (i, (x, y, a)) in path().into_iter().enumerate() {
        let e = h + H_PATH as usize + i * PATH_ENTRY as usize;
        reu[e..e + 2].copy_from_slice(&x.to_le_bytes());
        reu[e + 2..e + 4].copy_from_slice(&y.to_le_bytes());
        reu[e + 4] = a;
    }
    reu
}

fn line_table_addr(a: u8) -> usize {
    crate::format::line_table(a) as usize
}

/// Path frame `tick`'s camera, read from the image.
pub fn camera(reu: &[u8], tick: u16) -> Camera {
    let e = HEADER as usize + H_PATH as usize + (tick % PATH_LEN) as usize * PATH_ENTRY as usize;
    Camera { x: u16::from_le_bytes([reu[e], reu[e + 1]]), y: u16::from_le_bytes([reu[e + 2], reu[e + 3]]), angle: reu[e + 4], pop: geom::POP }
}

/// Path frame `tick`'s lines, read from the image exactly as the C64 does.
pub fn frame_lines(reu: &[u8], tick: u16) -> [Line; 200] {
    let c = camera(reu, tick);
    let t = line_table_addr(c.angle);
    m7::lines_at(&reu[t..t + LINE_BLOCK as usize], c.x, c.y)
}

/// The scene block as the C64 fetches it.
pub fn scene(reu: &[u8]) -> &[u8] {
    let s = SCENE as usize;
    let len = u16::from_le_bytes([reu[s], reu[s + 1]]) as usize;
    &reu[s..s + len]
}

/// The C64's screen setup on a model: colours, the sky drawn once into `buf`.
pub fn setup(m: &mut Machine, buf: Buffer) {
    m7::setup_colors(m, buf, COLORS, SKY_COLOR, FLOOR_ROW);
    m.ram[buf.bitmap as usize..(buf.bitmap + FLOOR_ROW * layout::ROW_STRIDE) as usize].fill(m7::SKY_BYTE);
}

/// The sky and floor of path frame `tick` in buffer 0 of a fresh model of `reu`.
pub fn render_floor(reu: &[u8], tick: u16, doubled: bool) -> (Machine, crate::cost::Cost) {
    let mut m = Machine::new(reu);
    setup(&mut m, BUFFERS[0]);
    let lines = frame_lines(reu, tick);
    let cost = m7::per_byte_dma(&mut m, BUFFERS[0], &lines, DmaMode { pre: true, doubled, first_row: FLOOR_ROW });
    (m, cost)
}

/// Render path frame `tick` into buffer 0 of a fresh model of `reu` (as the C64 draws any frame: sky,
/// floor, then the geometry over both).
pub fn render(reu: &[u8], tick: u16, doubled: bool) -> (Machine, crate::cost::Cost, geom::Stats) {
    let (mut m, cost) = render_floor(reu, tick, doubled);
    let mut work = Box::new(geom::Work::new());
    let mut st = geom::draw(&mut m, scene(reu), camera(reu, tick), BUFFERS[0].bitmap, &mut work, &mut Box::new(geom::Verts::new()));
    let (tris, top, dropped) = geom::raster_bands(&mut m, &work, BUFFERS[0].bitmap);
    (st.tris, st.top_row) = (tris, top);
    st.dropped = st.dropped.saturating_add(dropped);
    (m, cost, st)
}
