//! The REU package (host): the tiled ground map and its mip levels, the sea, the header; and the host's
//! render of a frame exactly as the C64 draws it (the reference `tools/emu.py` checks against).

use crate::floor::{self, COLS, FloorCam, LEVELS, MAP_TILES, MAX_SPANS, Rays, Span, VIEW_BANDS};
use crate::format::*;
use crate::layout::{self, BUFFERS, Buffer, LINEBUF, STAGE};
use crate::model::Machine;
use crate::world::{self, World};

/// The image (only up to [`IMAGE_END`]: everything after is unused for now).
pub fn bake(world: &World) -> Vec<u8> {
    let mut reu = vec![0u8; IMAGE_END as usize];
    for (k, level) in world::mips(&world.texels).iter().enumerate() {
        let tiles = (MAP_TILES >> k) as usize;
        let n = tiles * 256;
        assert_eq!(level.len(), n * n);
        for v in 0..n {
            for u in 0..n {
                let bank = floor::tile_bank(k as u8, (u >> 8) as u8, (v >> 8) as u8) as usize;
                reu[bank << 16 | (v & 255) << 8 | (u & 255)] = world::dither(level[v * n + u], u, v);
            }
        }
    }
    let sea = (SEA_BANK as usize) << 16;
    reu[sea..sea + 0x10000].fill(world::texel::WATER);
    let h = HEADER as usize;
    reu[h..h + 8].copy_from_slice(MAGIC);
    reu[h + H_VERSION as usize..][..2].copy_from_slice(&PACKAGE_VERSION.to_le_bytes());
    reu
}

/// The cockpit panel's bytes (bands `VIEW_BANDS..25`), the same on both sides: brown with a light-blue
/// rim along its top.
pub fn panel_byte(band: u16, line: u16) -> u8 {
    if band == VIEW_BANDS && line < 2 { 0x55 } else { 0xAA }
}

/// The C64's screen set-up on a model: colours, the panel, the view cleared to sky.
pub fn setup(m: &mut Machine, buf: Buffer) {
    use crate::hw::Hw;
    m.poke(0xD021, COLORS[0]);
    m.fill(buf.screen, layout::CELLS, COLORS[1] << 4 | COLORS[2]);
    m.fill(layout::COLOR_RAM, layout::CELLS, COLORS[3]);
    m.fill(buf.bitmap, VIEW_BANDS * layout::ROW_STRIDE, floor::SKY_BYTE);
    for band in VIEW_BANDS..layout::ROWS {
        for col in 0..layout::COLS {
            for line in 0..8 {
                m.poke(buf.bitmap + band * layout::ROW_STRIDE + col * 8 + line, panel_byte(band, line));
            }
        }
    }
}

/// What the host's floor did (for the frame report).
#[derive(Clone, Copy, Debug, Default)]
pub struct FloorStats {
    pub dmas: u32,
    pub io_writes: u32,
    pub spans: u32,
    pub ground_spans: u32,
}

/// Draw the view's bands into `buf` exactly as the C64 does: per band, per drawn line, the runs into the
/// line buffer, copied into the stage, the stage published to the bitmap.
pub fn render_floor(m: &mut Machine, buf: Buffer, cam: &FloorCam, doubled: bool) -> FloorStats {
    use crate::hw::Hw;
    let mut st = FloorStats::default();
    let rays = Rays::new(cam);
    let mut shadow = floor::floor_begin(m);
    let mut spans = [Span::default(); MAX_SPANS];
    for band in 0..VIEW_BANDS {
        for line in (0..8u8).step_by(if doubled { 2 } else { 1 }) {
            let a = rays.line(band * 8 + line as u16);
            let n = floor::make_line(cam, &rays, &a, floor::haze_byte(line, doubled), &mut spans);
            st.spans += n as u32;
            st.ground_spans += spans[..n].iter().filter(|s| s.fill().is_none()).count() as u32;
            let c = floor::draw_line(m, &spans[..n], LINEBUF, &mut shadow);
            st.dmas += c.dmas;
            st.io_writes += c.io_writes;
            floor::copy_line(m, LINEBUF, STAGE, line, doubled);
        }
        for i in 0..layout::ROW_STRIDE {
            let b = m.peek(STAGE + i);
            m.poke(buf.bitmap + band * layout::ROW_STRIDE + i, b);
        }
    }
    let _ = COLS;
    st
}

/// Frame `tick` of the autopilot flight in buffer 0 of a fresh model of `reu`.
pub fn render(reu: &[u8], tick: u16, doubled: bool) -> (Machine, FloorStats) {
    let mut m = Machine::new(reu);
    setup(&mut m, BUFFERS[0]);
    let (cam, _yaw) = crate::pilot::autopilot(tick);
    let st = render_floor(&mut m, BUFFERS[0], &cam, doubled);
    (m, st)
}

const _: () = assert!(LEVELS == 4);
