//! MODE 7 for the C64 Ultimate, milestone 3: a perspective floor flying round the test circuit, drawn by
//! `asm/floor.S` (one REU DMA per bitmap byte, the REU address stepped per byte) into a double-buffered
//! 160×200 multicolour bitmap. Pattern: `../talonhd/target-c64/src/main.rs`.
//!
//! Needs the matching `mode7.reu` at REU address 0 (16 MB REU); until it appears the border flashes, so it
//! may be uploaded after starting. Fire on joystick 2 (or `STATUS+MODE`) toggles full / line-doubled.
//! Performance is in the status block at `$CA80` (`layout::status`).
//!
//! Milestone 4: flat-shaded 3D over the floor. `chopper::geom` (shared with the host, so both compute the
//! same bits) transforms, clips, culls and depth-sorts the scene; `asm/poly.S` writes each triangle's cell
//! rows, whole bytes wherever a cell is fully covered.

#![no_std]
#![no_main]

use core::ptr::{read_volatile, write_volatile};

use chopper::format::{
    COLORS, FLOOR_ROW, H_PATH, HEADER, LINE_BLOCK, MAGIC, PACKAGE_VERSION, PATH_ENTRY, PATH_LEN, SCENE, SCENE_MAX,
    SKY_COLOR, line_table,
};
use chopper::geom::{self, BANDS, Camera, Obj, Verts, Work, geo_args};
use chopper::hw::{self, Hw};
use chopper::layout::{self, BOUNCE, BUFFERS, LINE_RAM, GEO_ARGS, PATS, POLY_ARGS, SCENE_RAM, STAGE, STATUS, STATUS_MAGIC, VERTS, WORK, status};
use chopper::poly::{Row, args};

unsafe extern "C" {
    fn reu_execute(command: u8);
    /// `asm/floor.S`: the floor a cell row at a time (`floor_row` into 320 bytes at `dest`, lines
    /// `line..line + 8` of the line table at `LINE_RAM`), between `floor_begin` and `floor_end`.
    fn floor_begin(doubled: u8);
    fn floor_row(dest: u16, line: u8);
    fn floor_end();
    /// `asm/floor.S`: copy `STAGE` to the bitmap's cell row `band` once the raster is not in it.
    fn band_publish(band: u8);
    /// `asm/poly.S`: one cell row of a polygon from `POLY_ARGS`.
    fn poly_row();
    /// `asm/geom.S`: one object's vertices, one object's faces (`GEO_ARGS`); then band by band: file the
    /// sorted triangles under their first bands, activate a band's newcomers (returns how many are
    /// active), draw the active ones' rows into `STAGE`.
    fn geo_xform();
    fn geo_faces();
    fn geo_bin();
    fn geo_activate(band: u8) -> u8;
    fn geo_draw(band: u8);
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
    let sky_bytes = FLOOR_ROW * layout::ROW_STRIDE;
    for buf in BUFFERS {
        c.fill(buf.screen, layout::CELLS, COLORS[1] << 4 | COLORS[2]);
        c.fill(buf.bitmap, sky_bytes, chopper::mode7::SKY_BYTE);
        c.fill(buf.bitmap + sky_bytes, layout::BITMAP_BYTES - sky_bytes, 0);
    }
    c.fill(layout::COLOR_RAM, FLOOR_ROW * layout::COLS, SKY_COLOR);
    c.fill(layout::COLOR_RAM + FLOOR_ROW * layout::COLS, layout::CELLS - FLOOR_ROW * layout::COLS, COLORS[3]);
    poke(0xD015, 0);
    poke(0xD020, 0);
    poke(0xD021, COLORS[0]);
    poke(0xDD02, peek(0xDD02) | 3);
    poke(0xDD00, (peek(0xDD00) & !3) | BUFFERS[0].dd00);
    poke(0xD018, BUFFERS[0].d018);
    poke(0xD016, layout::D016);
    wait_vblank();
    poke(0xD011, layout::D011);
}

/// `asm/floor.S`'s camera (x, y in 8.8 texels; `layout::CAM_RAM`), copied to its zero page each row.
const CAMERA: u16 = layout::CAM_RAM;

/// Path frame `tick`: fetch the camera and its angle's line table.
fn frame_setup(c: &mut C64, tick: u16) -> Camera {
    let mut cam = [0u8; PATH_ENTRY as usize];
    hw::read_into(c, HEADER + H_PATH + tick as u32 * PATH_ENTRY, BOUNCE, &mut cam);
    hw::fetch(c, line_table(cam[4]), LINE_RAM, LINE_BLOCK as u16);
    for (i, &b) in cam[..4].iter().enumerate() {
        poke(CAMERA + i as u16, b);
    }
    let pop = peek(STATUS + status::POP);
    Camera { x: u16::from_le_bytes([cam[0], cam[1]]), y: u16::from_le_bytes([cam[2], cam[3]]), angle: cam[4], pop }
}

/// Fetch the scene block to `SCENE_RAM`; its length.
fn scene_load(c: &mut C64) -> u16 {
    let mut len = [0u8; 2];
    hw::read_into(c, SCENE, BOUNCE, &mut len);
    let len = u16::from_le_bytes(len).min(SCENE_MAX);
    hw::fetch(c, SCENE, SCENE_RAM, len);
    len
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
    let scene_len = scene_load(c);
    let scene = unsafe { core::slice::from_raw_parts(SCENE_RAM as *const u8, scene_len as usize) };
    // The scratch lives at WORK and VERTS (outside the program image); geom::draw resets what it reads.
    let work = unsafe { &mut *(WORK as *mut Work) };
    let verts = unsafe { &mut *(VERTS as *mut Verts) };
    // Shade patterns for asm/geom.S: even bytes, then odd.
    for (i, p) in chopper::poly::PATTERNS.iter().enumerate() {
        poke(PATS + i as u16, p[0]);
        poke(PATS + 16 + i as u16, p[1]);
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
    // Sky bands geometry drew into last frame: republished (clean) even when nothing touches them now.
    let mut sky_dirty = [false; FLOOR_ROW as usize];
    loop {
        let now = frames_elapsed();
        if peek(STATUS + status::HOLD) == 0 {
            tick = now % PATH_LEN;
        }
        stopwatch_start();
        let cam = frame_setup(c, tick);
        poke16(STATUS + status::SETUP_US, stopwatch_stop());

        // Geometry: transform, cull and sort into the depth buckets, then file by first band.
        stopwatch_start();
        let st = geom::draw(c, scene, cam, layout::BUFFERS[0].bitmap, work, verts);
        unsafe { geo_bin() };
        poke16(STATUS + status::GEOM_US, stopwatch_stop());

        // Band by band: build each cell row in STAGE (floor, then the triangles crossing it, in painter's
        // order) and publish it when the raster is elsewhere. One buffer; a frame tears between bands.
        let doubled = peek(STATUS + status::MODE) & 1;
        stopwatch_start();
        unsafe { floor_begin(doubled) };
        for band in 0..BANDS as u8 {
            let active = unsafe { geo_activate(band) };
            if band < FLOOR_ROW as u8 {
                let dirty = &mut sky_dirty[band as usize];
                if active > 0 || *dirty {
                    c.fill(STAGE, layout::ROW_STRIDE, chopper::mode7::SKY_BYTE);
                    if active > 0 {
                        unsafe { geo_draw(band) };
                    }
                    unsafe { band_publish(band) };
                }
                *dirty = active > 0;
            } else {
                unsafe { floor_row(STAGE, band * 8) };
                if active > 0 {
                    unsafe { geo_draw(band) };
                }
                unsafe { band_publish(band) };
            }
        }
        unsafe { floor_end() };
        poke16(STATUS + status::RENDER_US, stopwatch_stop());
        poke(STATUS + status::TRIS, peek(GEO_ARGS + geo_args::TRIS));
        poke(STATUS + status::OBJECTS, st.objects);
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
