//! Chopper for the C64 Ultimate, milestone N1: the terrain flythrough.
//!
//! The camera flies a fixed circuit over the island. Every frame runs the
//! shared library exactly as the host does — horizon, Röttger terrain walk,
//! depth sort, band raster, cell packing — and publishes each finished band
//! straight into the single displayed bitmap (H.A.V.O.C.'s scheme: no back
//! buffer, colour RAM included). Terrain samples come from the REU through a
//! small direct-mapped cache, one 4-byte DMA per miss.
//!
//! Code paging (`chopper::paging`): GEOMETRY and PIXELS run in turn from the
//! `$4000-$BFFF` window; everything else is resident.
//!
//! Needs `chopper.reu` (from the same build) at REU address 0; until it
//! appears the border flashes, so it may be uploaded after starting. A
//! mismatched image stops with an orange border. Performance is in the
//! status block at `$0200` (`tools/c64u.py status`).

#![no_std]
#![no_main]

use core::cell::UnsafeCell;
use core::ptr::{read_volatile, write_volatile};

use chopper::camera::{Camera, QUARTER_TURN, WorldPos, cos, sin};
use chopper::render::{BAND_PIXELS, BANDS, RasterScratch, Renderer, VIEW_BANDS, draw_text, pack_band};
use chopper::terrain::{self, Governor, Lod, SIZE, Stats, TerrainData};
use chopper::{horizon, package, paging};

unsafe extern "C" {
    fn reu_execute(command: u8);
    fn turbo_probe() -> u16;
}

const SKY: u8 = 14;
const SEA: u8 = 6;
const PANEL: u8 = 11;

const SCREEN: u16 = 0xC000;
const BITMAP: u16 = 0xE000;
const COLOR_RAM: u16 = 0xD800;

/// Status block, read by `tools/c64u.py status`.
mod status {
    pub const BASE: u16 = 0x0200;
    pub const MAGIC: u16 = 0; // 'C' once running
    pub const STATE: u16 = 1; // 1 turbo, 2 waiting for REU, 3 running, 8 image mismatch
    pub const FRAMES: u16 = 2; // u16
    pub const FRAME_TICKS: u16 = 4; // u32, CIA ticks (~1 µs) for the last frame
    pub const GEOMETRY_TICKS: u16 = 8; // u32: paging + horizon + terrain walk
    pub const PAGING_TICKS: u16 = 12; // u32: both phase fetches
    pub const PIXELS_TICKS: u16 = 16; // u32: paging + sort + raster + pack + publish
    pub const COMMANDS: u16 = 20; // u16
    pub const TERRAIN_TRIS: u16 = 22; // u16
    pub const RESOLUTION: u16 = 24; // u32 (Q8)
    pub const NODES: u16 = 28; // u16
    pub const MISSES: u16 = 30; // u16 cache misses (DMAs) this frame
    pub const DROPPED: u16 = 32; // u16
    pub const PROBES: u16 = 34; // u16 turbo probes at boot
    pub const STACK_LOW: u16 = 36; // u16 lowest soft-stack address written (from $0800 down)
    pub const SIZE: u16 = 38;
}

#[inline(always)]
fn peek(addr: u16) -> u8 {
    unsafe { read_volatile(addr as *const u8) }
}

#[inline(always)]
fn poke(addr: u16, value: u8) {
    unsafe { write_volatile(addr as *mut u8, value) }
}

fn poke16(addr: u16, value: u16) {
    poke(addr, value as u8);
    poke(addr + 1, (value >> 8) as u8);
}

fn status_set16(field: u16, value: u16) {
    poke16(status::BASE + field, value);
}

fn status_set32(field: u16, value: u32) {
    poke16(status::BASE + field, value as u16);
    poke16(status::BASE + field + 2, (value >> 16) as u16);
}

/// REU → C64 in one command. The C64U reloads the whole REU address on any
/// address-register write (../mode7/docs/validation.md): write all three.
#[inline(never)]
fn reu_fetch(reu: u32, c64: u16, len: u16) {
    poke(0xDF06, (reu >> 16) as u8);
    poke(0xDF05, (reu >> 8) as u8);
    poke(0xDF04, reu as u8);
    poke16(0xDF02, c64);
    poke16(0xDF07, len);
    unsafe { reu_execute(0x91) };
}

/// A free-running 32-bit CIA 2 clock: timer A counts ~1 MHz cycles, timer B
/// counts its underflows.
fn clock_start() {
    poke16(0xDD04, 0xFFFF);
    poke16(0xDD06, 0xFFFF);
    poke(0xDD0F, 0x51); // B: start, force load, count A underflows
    poke(0xDD0E, 0x11); // A: start, force load, continuous
}

#[inline(never)]
fn clock() -> u32 {
    loop {
        let b_hi = peek(0xDD07);
        let b_lo = peek(0xDD06);
        let a_hi = peek(0xDD05);
        let a_lo = peek(0xDD04);
        if peek(0xDD07) == b_hi && peek(0xDD06) == b_lo {
            let b = !(b_lo as u16 | (b_hi as u16) << 8);
            let a = !(a_lo as u16 | (a_hi as u16) << 8);
            return (b as u32) << 16 | a as u32;
        }
    }
}

/// Turbo engages ~2 s after `$D031` is written: probe until a timed loop runs
/// over 10x fast.
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

#[inline(never)]
fn raster_line() -> u16 {
    loop {
        let lo = peek(0xD012);
        let hi = peek(0xD011) & 0x80;
        if peek(0xD012) == lo {
            return lo as u16 | (hi as u16) << 1;
        }
    }
}

/// Patched by tools/phases.py: the stamp of this build's phase images.
#[unsafe(no_mangle)]
static PHASE_STAMP: u16 = 0xFFFF;
/// Each phase's length, from the REU header.
static mut PHASE_LEN: [u16; paging::PHASES as usize] = [0; paging::PHASES as usize];
static mut CURRENT_PHASE: u8 = 0xFF;

/// Make phase `p` current: one REU fetch unless it is in the window already.
#[inline(never)]
fn page(p: u8) {
    unsafe {
        if CURRENT_PHASE == p {
            return;
        }
        CURRENT_PHASE = p;
        // At most 32 KiB in one command: slots are inside one 512 KiB REU bank.
        reu_fetch(paging::slot(p), paging::WINDOW, PHASE_LEN[p as usize]);
    }
}

/// Wait for the REU image (it may be uploaded after the program starts),
/// flashing the border. An image from another build (wrong stamp) keeps it
/// waiting with an orange border: the REU usually still holds the previous
/// build while the new one uploads (its header arrives last).
fn wait_image() {
    let mut header = [0u8; package::HEADER_LEN as usize];
    loop {
        reu_fetch(package::HEADER, header.as_mut_ptr() as u16, package::HEADER_LEN);
        let header = unsafe { read_volatile(&header) };
        let stamp = unsafe { read_volatile(&raw const PHASE_STAMP) };
        let ours = &header[..8] == package::MAGIC && u16::from_le_bytes([header[8], header[9]]) == package::VERSION;
        if ours && u16::from_le_bytes([header[10], header[11]]) != stamp {
            poke(status::BASE + status::STATE, 8);
            poke(0xD020, 8);
            continue;
        }
        if ours {
            for p in 0..paging::PHASES as usize {
                unsafe { PHASE_LEN[p] = u16::from_le_bytes([header[12 + 2 * p], header[13 + 2 * p]]) };
            }
            return;
        }
        poke(0xD020, peek(0xD020).wrapping_add(1));
        for _ in 0..20000u16 {
            core::hint::spin_loop();
        }
    }
}

/// Terrain samples from the REU through a 256-entry direct-mapped cache
/// (resident, so it survives paging).
struct ReuTerrain {
    tags: UnsafeCell<[u32; 256]>,
    records: UnsafeCell<[[u8; 4]; 256]>,
    misses: UnsafeCell<u16>,
}

impl ReuTerrain {
    const fn new() -> Self {
        Self { tags: UnsafeCell::new([u32::MAX; 256]), records: UnsafeCell::new([[0; 4]; 256]), misses: UnsafeCell::new(0) }
    }

    #[inline(never)]
    fn record(&self, i: usize, j: usize) -> [u8; 4] {
        let key = j as u32 * SIZE as u32 + i as u32;
        let slot = ((i as u8) ^ ((j as u8) << 3) ^ ((j >> 5) as u8)) as usize;
        unsafe {
            let tags = &mut *self.tags.get();
            let records = self.records.get() as *mut [u8; 4];
            if tags[slot] != key {
                tags[slot] = key;
                *self.misses.get() += 1;
                reu_fetch(package::record(i, j), records.add(slot) as u16, 4);
            }
            read_volatile(records.add(slot))
        }
    }

    fn take_misses(&self) -> u16 {
        unsafe { core::mem::replace(&mut *self.misses.get(), 0) }
    }
}

impl TerrainData for ReuTerrain {
    fn height(&self, i: usize, j: usize) -> u8 {
        self.record(i, j)[0]
    }
    fn kind(&self, i: usize, j: usize) -> u8 {
        self.record(i, j)[1]
    }
    fn d2(&self, i: usize, j: usize) -> u8 {
        self.record(i, j)[2]
    }
    fn hmax(&self, i: usize, j: usize) -> u8 {
        self.record(i, j)[3]
    }
}

/// The target is single-threaded and the cache is only touched from main.
struct ReuTerrainCell(ReuTerrain);
unsafe impl Sync for ReuTerrainCell {}

static TERRAIN: ReuTerrainCell = ReuTerrainCell(ReuTerrain::new());
static mut RENDERER: Renderer = Renderer::new();
/// PIXELS' scratch: window RAM after its code (link.ld `.pixels_bss`), only
/// valid while PIXELS is paged in and fully rewritten before each use.
#[unsafe(link_section = ".pixels_bss")]
static mut SCRATCH: RasterScratch = RasterScratch::new();
#[unsafe(link_section = ".pixels_bss")]
static mut PIXELS: [u8; BAND_PIXELS] = [0; BAND_PIXELS];

/// Pack one band and publish it into the displayed bitmap, screen matrix and
/// colour RAM, waiting while the beam is fetching that band.
#[inline(never)]
fn publish(band: usize, pixels: &[u8; BAND_PIXELS], background: u8) {
    let mut previous_screen = [0u8; 40];
    let mut previous_color = [0u8; 40];
    for cell in 0..40 {
        previous_screen[cell] = peek(SCREEN + (band * 40 + cell) as u16);
        previous_color[cell] = peek(COLOR_RAM + (band * 40 + cell) as u16) & 15;
    }
    let mut bitmap = [0u8; 320];
    let mut screen = [0u8; 40];
    let mut colors = [0u8; 40];
    pack_band(pixels, &previous_screen, &previous_color, background, &mut bitmap, &mut screen, &mut colors);
    // Visible text row `band` spans raster lines 51 + 8·band .. +8.
    let top = 51 + 8 * band as u16;
    loop {
        let line = raster_line();
        if line + 2 < top || line >= top + 8 {
            break;
        }
    }
    unsafe {
        core::ptr::copy_nonoverlapping(bitmap.as_ptr(), (BITMAP + band as u16 * 320) as *mut u8, 320);
        core::ptr::copy_nonoverlapping(screen.as_ptr(), (SCREEN + band as u16 * 40) as *mut u8, 40);
        core::ptr::copy_nonoverlapping(colors.as_ptr(), (COLOR_RAM + band as u16 * 40) as *mut u8, 40);
    }
}

fn video_init() {
    unsafe {
        core::ptr::write_bytes(BITMAP as *mut u8, 0, 8000);
        core::ptr::write_bytes(SCREEN as *mut u8, 0, 1000);
        core::ptr::write_bytes(COLOR_RAM as *mut u8, 0, 1000);
    }
    poke(0xD015, 0);
    poke(0xD020, 0);
    poke(0xD021, SKY);
    poke(0xDD02, peek(0xDD02) | 3);
    poke(0xDD00, peek(0xDD00) & !3); // VIC bank 3: $C000-$FFFF
    poke(0xD018, 0x08); // screen $C000, bitmap $E000
    poke(0xD016, 0x18); // multicolor, 40 columns
    poke(0xD011, 0x3B); // bitmap mode, display on, 25 rows
}

/// The autopilot circuit: a 6 km circle 150 m above the ground, banked 15°.
#[inline(never)]
fn autopilot(terrain: &ReuTerrain, frame: u16) -> Camera {
    let a = frame.wrapping_mul(0x0050);
    let x = sin(a) * 6000 >> 14;
    let z = (-cos(a) * 6000 >> 14) - 2000;
    let ground = terrain::height_at(terrain, x, z);
    let degree = (65536 / 360) as u16;
    Camera::new(WorldPos { x, y: ground + 600, z }, a.wrapping_add(QUARTER_TURN), 0u16.wrapping_sub(6 * degree), 15 * degree)
}

/// GEOMETRY entry: build this frame's command list.
#[inline(never)]
#[unsafe(link_section = ".phase.geometry.entry")]
#[unsafe(no_mangle)]
extern "C" fn geometry_frame(frame: u16, lod: &Lod, stats: &mut Stats) {
    let terrain = &TERRAIN.0;
    let renderer = unsafe { &mut *(&raw mut RENDERER) };
    let camera = autopilot(terrain, frame);
    renderer.begin(SKY);
    horizon::draw_ground(renderer, &camera, SEA);
    *stats = terrain::render(terrain, &camera, *lod, renderer);
}

/// PIXELS entry: sort, rasterize, pack and publish the view bands.
#[inline(never)]
#[unsafe(link_section = ".phase.pixels.entry")]
#[unsafe(no_mangle)]
extern "C" fn pixels_frame() {
    let renderer = unsafe { &mut *(&raw mut RENDERER) };
    let scratch = unsafe { &mut *(&raw mut SCRATCH) };
    let pixels = unsafe { &mut *(&raw mut PIXELS) };
    renderer.finish();
    let mut pass = renderer.raster_pass(scratch);
    while let Some(band) = pass.next_band(pixels) {
        publish(band as usize, pixels, SKY);
    }
}

/// PIXELS entry: the cockpit panel, drawn once (N1: a plain placeholder).
#[inline(never)]
#[unsafe(link_section = ".phase.pixels.entry")]
#[unsafe(no_mangle)]
extern "C" fn pixels_panel() {
    let pixels = unsafe { &mut *(&raw mut PIXELS) };
    for band in VIEW_BANDS..BANDS {
        pixels.fill(PANEL);
        draw_text(band as u8, pixels, 4, 142, b"CHOPPER N1 - TERRAIN FLYTHROUGH", 1, 1);
        draw_text(band as u8, pixels, 4, 152, b"C64 ULTIMATE 64 MHZ + 16 MB REU", 15, 1);
        publish(band, pixels, SKY);
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn main() -> i16 {
    poke(0xD011, 0x0B); // display off while booting
    unsafe { core::ptr::write_bytes(status::BASE as *mut u8, 0, status::SIZE as usize) };
    poke(status::BASE + status::STATE, 1);
    // Paint the free soft stack so its high-water mark can be measured.
    unsafe { core::ptr::write_bytes(STACK_FLOOR as *mut u8, 0xA5, (0x0700 - STACK_FLOOR) as usize) };
    let probes = wait_turbo();
    status_set16(status::PROBES, probes);
    poke(0xDF09, 0);
    poke(0xDF0A, 0);
    poke(status::BASE + status::STATE, 2);
    wait_image();
    video_init();
    page(paging::PIXELS);
    pixels_panel();
    clock_start();
    poke(status::BASE + status::STATE, 3);
    poke(status::BASE + status::MAGIC, b'C');

    let mut governor = Governor::new();
    let mut stats = Stats::default();
    let mut frame = 0u16;
    let mut previous = clock();
    loop {
        let start = clock();
        page(paging::GEOMETRY);
        let paged_geometry = clock();
        geometry_frame(frame, &governor.lod, &mut stats);
        let geometry = clock();
        page(paging::PIXELS);
        let paged_pixels = clock();
        pixels_frame();
        let end = clock();
        let renderer = unsafe { &*(&raw const RENDERER) };
        governor.update(renderer.triangle_count as u32);

        status_set32(status::FRAME_TICKS, end.wrapping_sub(previous));
        status_set32(status::GEOMETRY_TICKS, geometry.wrapping_sub(start));
        status_set32(status::PAGING_TICKS, paged_geometry.wrapping_sub(start).wrapping_add(paged_pixels.wrapping_sub(geometry)));
        status_set32(status::PIXELS_TICKS, end.wrapping_sub(geometry));
        status_set16(status::COMMANDS, renderer.triangle_count);
        status_set16(status::TERRAIN_TRIS, stats.triangles as u16);
        status_set32(status::RESOLUTION, governor.lod.resolution_q8 as u32);
        status_set16(status::NODES, stats.nodes as u16);
        status_set16(status::MISSES, TERRAIN.0.take_misses());
        status_set16(status::DROPPED, renderer.dropped);
        previous = end;
        frame = frame.wrapping_add(1);
        status_set16(status::FRAMES, frame);
        status_set16(status::STACK_LOW, stack_low());
    }
}

/// The soft stack's lowest legal address (just above the status block).
const STACK_FLOOR: u16 = 0x0240;

/// The lowest soft-stack byte that no longer holds the boot paint.
fn stack_low() -> u16 {
    let mut a = STACK_FLOOR;
    while a < 0x0700 && peek(a) == 0xA5 {
        a += 1;
    }
    a
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        poke(0xD020, 2);
    }
}
