//! Chopper for the C64 Ultimate, milestone F1: the floor. The autopilot (`chopper::pilot`, shared with
//! the host) flies a banked circle over the 2048×2048-texel island; every view line is cut into runs by
//! `chopper::floor::make_line` (the same Rust on both sides, so the frames match the host byte for byte)
//! and drawn by `asm/floor.S`, one REU DMA per ground byte, into a single buffer one band at a time
//! (`band_publish`). Pattern: `../mode7/target-c64/src/main.rs`.
//!
//! Needs the matching `chopper.reu` at REU address 0; until it appears the border flashes, so it may be
//! uploaded after starting. Fire on joystick 2 (or `STATUS+MODE`) toggles full / line-doubled.
//! Performance is in the status block at `$CA80` (`layout::status`).

#![no_std]
#![no_main]

use core::ptr::{read_volatile, write_volatile};

use chopper::floor::{self, Rays, SpanArgs, VIEW_BANDS};
use chopper::format::{COLORS, HEADER, MAGIC, PACKAGE_VERSION};
use chopper::geom::{Obj, Verts, Work, geo_args};
use chopper::hw::{self, Hw};
use chopper::layout::{self, BOUNCE, BUFFERS, GEO_ARGS, POLY_ARGS, RAYS, SCENE_RAM, STATUS, STATUS_MAGIC, TILES, status};
use chopper::pilot::{LAP, autopilot};
use chopper::poly::{Row, args};

unsafe extern "C" {
    fn reu_execute(command: u8);
    /// `asm/floor.S`: once a frame `floor_begin`; per view line `floor_line` (the runs at `SPANS` into
    /// the line buffer) and `floor_copy` (into band line `line` of `STAGE`, and the next when doubled).
    /// `asm/spans.S`: the runs of the line whose ray is `RAYS.a` into `SPANS` (`floor::make_line`).
    fn span_line();
    fn bt_build();
    fn floor_begin();
    fn floor_end();
    fn floor_line();
    fn floor_copy(line: u8, doubled: u8);
    /// `asm/floor.S`: copy `STAGE` to the bitmap's cell row `band` once the raster is not in it.
    fn band_publish(band: u8);
    /// `asm/poly.S`: one cell row of a polygon from `POLY_ARGS`.
    fn poly_row();
    /// `asm/geom.S`: one object's vertices, one object's faces (`GEO_ARGS`); then band by band: file the
    /// sorted triangles under their first bands, activate a band's newcomers (returns how many are
    /// active), draw the active ones' rows into `STAGE`.
    fn geo_xform();
    fn geo_faces();
    /// `asm/floor.S`: CIA 2 ticks for 20,480 cycles.
    fn turbo_probe() -> u16;
}

/// NTSC: 263 lines of 65 cycles.
const FRAME_CYCLES: u16 = 263 * 65;

struct C64;

impl Hw for C64 {
    #[inline(always)]
    fn peek(&mut self, addr: u16) -> u8 {
        unsafe { read_volatile(addr as *const u8) }
    }
    #[inline(always)]
    fn poke(&mut self, addr: u16, value: u8) {
        unsafe { write_volatile(addr as *mut u8, value) }
    }
    #[inline(always)]
    fn reu_exec(&mut self, command: u8) {
        unsafe { reu_execute(command) }
    }
    fn fill(&mut self, addr: u16, len: u16, value: u8) {
        unsafe { core::ptr::write_bytes(addr as *mut u8, value, len as usize) }
    }
    #[inline(never)]
    fn poly_row(&mut self, row: &Row) {
        unsafe {
            let a = POLY_ARGS as *mut u8;
            for k in 0..8 {
                write_volatile(a.add(args::XL as usize + k), row.xl[k]);
                write_volatile(a.add(args::XR as usize + k), row.xr[k]);
            }
            write_volatile(a.add(args::PAT as usize), row.pat[0]);
            write_volatile(a.add(args::PAT as usize + 1), row.pat[1]);
            write_volatile(a.add(args::BASE as usize), row.base as u8);
            write_volatile(a.add(args::BASE as usize + 1), (row.base >> 8) as u8);
            poly_row();
        }
    }
    fn xform(&mut self, _scene: &[u8], o: &Obj, _v: &mut Verts) {
        let g = GEO_ARGS;
        poke16(g + geo_args::VPTR, SCENE_RAM + o.verts);
        poke(g + geo_args::NV, o.nv);
        poke(g + geo_args::SH, 14 - o.t);
        poke(g + geo_args::T, o.t);
        poke16(g + geo_args::X0, o.x0 as u16);
        poke16(g + geo_args::Z0, o.z0 as u16);
        poke16(g + geo_args::CC, o.cc as u16);
        poke16(g + geo_args::SS, o.ss as u16);
        unsafe { geo_xform() }
    }
    fn faces(&mut self, _scene: &[u8], o: &Obj, _v: &Verts, _w: &mut Work) {
        poke16(GEO_ARGS + geo_args::FPTR, SCENE_RAM + o.faces);
        poke(GEO_ARGS + geo_args::NF, o.nf);
        unsafe { geo_faces() }
    }
}


#[inline(always)]
fn peek(addr: u16) -> u8 {
    C64.peek(addr)
}

#[inline(always)]
fn poke(addr: u16, value: u8) {
    C64.poke(addr, value)
}

fn poke16(addr: u16, value: u16) {
    poke(addr, value as u8);
    poke(addr + 1, (value >> 8) as u8);
}

/// NTSC frames since the clock started (CIA 2 timer B counting timer A's frame-long underflows).
fn frames_elapsed() -> u16 {
    loop {
        let hi = peek(0xDD07);
        let lo = peek(0xDD06);
        if peek(0xDD07) == hi {
            return !(lo as u16 | (hi as u16) << 8);
        }
    }
}

/// CIA 1 timer A as a stopwatch (1 MHz ticks, counting down from `$FFFF`).
fn stopwatch_start() {
    poke(0xDC04, 0xFF);
    poke(0xDC05, 0xFF);
    poke(0xDC0E, 0x11);
}

fn stopwatch_stop() -> u16 {
    poke(0xDC0E, 0x00);
    !(peek(0xDC04) as u16 | (peek(0xDC05) as u16) << 8)
}

/// The running stopwatch's ticks, without stopping it (they wrap after 65 ms: read it more often).
fn stopwatch_read() -> u16 {
    loop {
        let hi = peek(0xDC05);
        let lo = peek(0xDC04);
        if peek(0xDC05) == hi {
            return !(lo as u16 | (hi as u16) << 8);
        }
    }
}

fn in_vblank() -> bool {
    peek(0xD011) & 0x80 != 0 || peek(0xD012) >= 251
}

fn wait_vblank() {
    while !in_vblank() {}
}

/// Turbo engages ~2 s after `$D031` is written: probe until a timed loop runs over 10x fast.
fn wait_turbo() -> u16 {
    poke(0xD031, 0x8F);
    let mut probes = 0u16;
    loop {
        probes += 1;
        if unsafe { turbo_probe() } < 2000 || probes >= 3000 {
            return probes;
        }
    }
}

/// Wait for the package (it may be uploaded after the program starts), flashing the border.
fn wait_image(c: &mut C64) {
    let mut header = [0u8; 10];
    loop {
        hw::read_into(c, HEADER, BOUNCE, &mut header);
        if &header[..8] == MAGIC && u16::from_le_bytes([header[8], header[9]]) == PACKAGE_VERSION {
            return;
        }
        poke(0xD020, peek(0xD020).wrapping_add(1));
        for _ in 0..8 {
            while in_vblank() {}
            wait_vblank();
        }
    }
}

fn video_init(c: &mut C64) {
    let buf = BUFFERS[0];
    c.fill(buf.screen, layout::CELLS, COLORS[1] << 4 | COLORS[2]);
    c.fill(layout::COLOR_RAM, layout::CELLS, COLORS[3]);
    c.fill(buf.bitmap, VIEW_BANDS * layout::ROW_STRIDE, floor::SKY_BYTE);
    // The cockpit panel (`package::panel_byte`), drawn once.
    for band in VIEW_BANDS..layout::ROWS {
        for col in 0..layout::COLS {
            for line in 0..8 {
                let byte = if band == VIEW_BANDS && line < 2 { 0x55 } else { 0xAA };
                poke(buf.bitmap + band * layout::ROW_STRIDE + col * 8 + line, byte);
            }
        }
    }
    poke(0xD015, 0);
    poke(0xD020, 0);
    poke(0xD021, COLORS[0]);
    poke(0xDD02, peek(0xDD02) | 3);
    poke(0xDD00, (peek(0xDD00) & !3) | buf.dd00);
    poke(0xD018, buf.d018);
    poke(0xD016, layout::D016);
    wait_vblank();
    poke(0xD011, layout::D011);
}

#[unsafe(no_mangle)]
pub extern "C" fn main() -> i16 {
    let c = &mut C64;
    poke(0xD011, 0x0B); // display off while booting
    poke(STATUS + status::MAGIC, 0);
    let probes = wait_turbo();
    hw::init(c);
    wait_image(c);
    chopper::fixed::init_tables();
    for level in 0..floor::LEVELS {
        for (i, &b) in floor::tile_table(level).iter().enumerate() {
            poke(TILES + level as u16 * 256 + i as u16, b);
        }
    }
    video_init(c);

    // The frame clock: CIA 2 timer A underflows once an NTSC frame, timer B counts the underflows.
    poke16(0xDD04, FRAME_CYCLES - 1);
    poke16(0xDD06, 0xFFFF);
    poke(0xDD0F, 0x51); // B: start, force load, count A underflows
    poke(0xDD0E, 0x11); // A: start, force load, continuous

    c.fill(STATUS, status::SIZE, 0);
    poke16(STATUS + status::PROBES, probes);
    poke(STATUS + status::MAGIC, STATUS_MAGIC);

    let mut tick = 0u16;
    let mut frames = 0u16;
    let (mut window, mut in_window) = (frames_elapsed(), 0u8);
    let mut fire_was = false;
    let args = unsafe { &mut *(RAYS as *mut SpanArgs) };
    loop {
        let now = frames_elapsed();
        if peek(STATUS + status::HOLD) == 0 {
            tick = now % LAP;
        }
        // Three stopwatches a frame (tools/emu.py finds the frames by them): set-up, geometry (none
        // yet), the bands.
        stopwatch_start();
        let (cam, _yaw) = autopilot(tick);
        let rays = Rays::new(&cam);
        *args = SpanArgs::new(&cam, &rays);
        unsafe { bt_build() };
        poke16(STATUS + status::SETUP_US, stopwatch_stop());
        stopwatch_start();
        poke16(STATUS + status::GEOM_US, stopwatch_stop());

        let doubled = peek(STATUS + status::MODE) & 1;
        stopwatch_start();
        unsafe { floor_begin() };
        // The bands can take longer than the 16-bit stopwatch: add it up band by band.
        let (mut render, mut last) = (0u32, 0u16);
        // Line y's ray at byte 0, stepped (exactly) line by line.
        let mut a = rays.a0;
        let step: [i32; 3] = if doubled != 0 { rays.ay.map(|c| c * 2) } else { rays.ay };
        for band in 0..VIEW_BANDS as u8 {
            let mut line = 0u8;
            while line < 8 {
                args.a = a;
                unsafe {
                    span_line();
                    floor_line();
                    floor_copy(line, doubled);
                }
                a = [a[0] + step[0], a[1] + step[1], a[2] + step[2]];
                line += 1 + doubled;
            }
            unsafe { band_publish(band) };
            let now = stopwatch_read();
            render += now.wrapping_sub(last) as u32;
            last = now;
        }
        unsafe { floor_end() };
        render += stopwatch_stop().wrapping_sub(last) as u32;
        poke16(STATUS + status::RENDER_US, (render >> 2) as u16);
        poke(STATUS + status::TRIS, 0);
        poke(STATUS + status::OBJECTS, 0);
        poke16(STATUS + status::TICK, tick);

        frames = frames.wrapping_add(1);
        poke16(STATUS + status::FRAMES, frames);
        in_window += 1;
        if now.wrapping_sub(window) >= 60 {
            poke(STATUS + status::FPS, in_window);
            window = now;
            in_window = 0;
        }

        // Fire on joystick 2 toggles full / line-doubled.
        let fire = peek(0xDC00) & 0x10 == 0;
        if fire && !fire_was {
            poke(STATUS + status::MODE, doubled ^ 1);
        }
        fire_was = fire;
    }
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        poke(0xD020, 2);
    }
}
