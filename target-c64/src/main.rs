//! Chopper for the C64 Ultimate: the floor and the 3D objects. The attract-mode helicopter
//! (`chopper::agent`, shared with the host, run from the AI code overlay) waits on the carrier's deck for
//! the rotor, lifts off and roams the 2048×2048-texel island, steered by the GOAP-planned decision tree in
//! the REU (`chopper::goap`, `chopper::brain`). Each frame the objects in view
//! are streamed from the REU, transformed and sorted (`chopper::geom::draw`, `asm/geom.S`); then band by
//! band every view line is cut into runs (`asm/spans.S`) and drawn by `asm/floor.S`, one REU DMA per
//! ground byte, the band's triangles go over it (`geo_activate`, `geo_draw`), and the band is published
//! (`band_publish`). The host renders the same frames byte for byte. Pattern:
//! `../mode7/target-c64/src/main.rs`.
//!
//! Needs the matching `chopper.reu` at REU address 0; until it appears the border flashes, so it may be
//! uploaded after starting. Any control (joystick 2 or its second button, or the keys: W A S D, SPACE,
//! SHIFT, `controls`) takes over from the autopilot. The floor's lines are adaptive: a line pair is drawn
//! once only where its lines repeat a texel (`floor::double_mask`), the same rule every frame. (The host
//! can set full or line-doubled lines at `STATUS+MODE`, to compare.)
//! Performance is in the status block at `$CA80` (`layout::status`).

#![no_std]
#![no_main]

use core::ptr::{read_volatile, write_volatile};

use chopper::floor::{self, Rays, SpanArgs, VIEW_BANDS};
use chopper::cockpit;
use chopper::format::{COLORS, H_BUILD, HEADER, MAGIC, OVERLAY_ACTORS, OVERLAY_AI, OVERLAY_COCKPIT, OVERLAY_DYN, OVERLAY_GEOM, OVERLAY_HUD, OVERLAY_WORLD, OVERLAY_WORLD2, OVERLAY_WORLD3, OVERLAY_WORLD4, OVERLAY_COMBAT, OVERLAY_COMBATLIB, OVERLAY_DECK, OVERLAY_CLIP, OVERLAY_FX, OVERLAY_STREAM, PACKAGE_VERSION, SCENE, SCENE_MAX};
use chopper::geom::{self, Obj, Verts, View, Work, geo_args};
use chopper::hw::{self, Hw};
use chopper::layout::{self, AGENT, BOUNCE, CLOCK, BUFFERS, GAUGES, GEO_ARGS, PATS, POLY_ARGS, RAYS, SCENE_RAM, STATUS, STATUS_MAGIC, TILES, WALK_ARGS, WORK, status};
use chopper::agent::Agent;
use chopper::poly::{Row, args};

unsafe extern "C" {
    fn reu_execute(command: u8);
    /// `asm/stream.S` (the `.text.stream` overlay, at `WORK`): the rotor sound's decoder. `stream_init`
    /// finds the sampler (1) or not (0: nothing streams); `stream_run` decodes pages into the ring up to
    /// half a lap ahead of the playback `frames` NTSC frames after the sampler's start (0: its budget spent,
    /// 1: there, 2: the stream has ended);
    /// `stream_start` starts the sampler round the ring. `stream_state`'s first byte: the stream has ended
    /// (or never began).
    fn stream_init() -> u8;
    fn stream_run(frames_lo: u8, frames_hi: u8) -> u8;
    fn stream_start(divider_lo: u8, divider_hi: u8);
    /// `asm/stream.S`: the sampler's clock divider for 16 kHz, measured with CIA 2 (which the frame clock
    /// then takes over).
    fn stream_calibrate() -> u16;
    static stream_state: u8;
    /// `asm/floor.S`: once a frame `floor_begin`; per view line `floor_line` (the runs at `SPANS` into
    /// the line buffer) and `floor_copy` (into band line `line` of `STAGE`).
    /// `asm/spans.S`: the runs of the line whose ray is `RAYS.a` into `SPANS` (`floor::make_line`), and
    /// `fog_second`: a doubled pair's second line over them, its own fog line (`floor::second_line`).
    fn span_line();
    fn fog_second();
    fn bt_build();
    fn floor_begin();
    fn floor_end();
    fn floor_line();
    fn floor_copy(line: u8);
    /// `asm/floor.S`: `RAYS.a += RAY_STEP` (the next drawn line's ray).
    fn ray_step();
    /// `asm/floor.S`: copy `STAGE` to the bitmap's cell row `band` once the raster is not in it.
    fn band_publish(band: u8);
    /// `asm/poly.S`: one cell row of a polygon from `POLY_ARGS`.
    fn poly_row();
    /// `asm/geom.S`: one object's vertices, one object's faces (`GEO_ARGS`); then band by band: file the
    /// sorted triangles under their first bands, activate a band's newcomers (returns how many are
    /// active), draw the active ones' rows into `STAGE`.
    fn geo_xform();
    fn geo_faces();
    /// `asm/geom.S`: the frame's objects from the REU grids into the buckets (`geom::draw`), from
    /// `WALK_ARGS`.
    fn geo_walk();
    fn geo_bin();
    fn geo_activate(band: u8) -> u8;
    fn geo_draw(band: u8);
    /// `asm/floor.S`: CIA 2 ticks for 20,480 cycles.
    fn turbo_probe() -> u16;
    /// `asm/hud.S`: the HUD's raster IRQ (`chopper::hud`), and `cli`.
    fn hud_irq();
    fn hud_irq_enable();
}

/// This build's stamp (target-c64/build.rs): `chopper bake --elf` copies it into the image's header
/// with the REU overlays, and the boot waits for an image that has it.
#[unsafe(no_mangle)]
#[used]
pub static CHOPPER_BUILD: u32 = stamp(env!("CHOPPER_BUILD"));

const fn stamp(s: &str) -> u32 {
    let b = s.as_bytes();
    let (mut v, mut i) = (0u32, 0);
    while i < b.len() {
        v = v.wrapping_mul(10).wrapping_add((b[i] - b'0') as u32);
        i += 1;
    }
    v
}

unsafe extern "C" {
    /// target-c64/link.ld: the REU overlays' sizes (`.text.ai`, `.text.hud`, run at `WORK`).
    static __aitext_size: u8;
    static __hudtext_size: u8;
    static __geomtext_size: u8;
    static __actorstext_size: u8;
    static __worldtext_size: u8;
    static __world2text_size: u8;
    static __world3text_size: u8;
    static __world4text_size: u8;
    static __combattext_size: u8;
    static __fxtext_size: u8;
    static __streamtext_size: u8;
    static __squadtext_size: u8;
    static __sfxlibtext_size: u8;
    static __combatlibtext_size: u8;
    /// The second window's overlays (`.text.cockpit`, `.text.dyn`, run at `BAND_RAM`).
    static __cocktext_size: u8;
    static __dyntext_size: u8;
    static __decktext_size: u8;
    static __cliptext_size: u8;
}

fn cockpit_overlay() -> (u16, u16) {
    (chopper::layout::BAND_RAM, &raw const __cocktext_size as u16)
}

/// The agent's REU reads (the hazard map, the tree), through the bounce buffer.
struct Senses<'a>(&'a mut C64);

impl chopper::brain::Reu for Senses<'_> {
    fn read(&mut self, addr: u32, out: &mut [u8]) {
        hw::read_into(self.0, addr, BOUNCE, out);
    }
}

fn overlay_sizes() -> (u16, u16, u16, u16) {
    (&raw const __aitext_size as u16, &raw const __hudtext_size as u16, &raw const __geomtext_size as u16, &raw const __actorstext_size as u16)
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
        for j in 0..3u16 {
            poke16(g + geo_args::ORG + 2 * j, o.org[j as usize] as u16);
            for k in 0..3u16 {
                poke16(g + geo_args::M + 6 * j + 2 * k, o.m[j as usize][k as usize] as u16);
            }
        }
        unsafe { geo_xform() }
    }
    fn faces(&mut self, _scene: &[u8], o: &Obj, _v: &mut Verts, _w: &mut Work) {
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

/// `asm/geom.S` `geo_faces`'s tail call when it listed faces straddling the near plane (`WORK.clip`):
/// they are cut at the plane and pushed (`geom::clip`). In its own overlay (`format::OVERLAY_CLIP`), above
/// the triangle work, loaded before the walk.
#[unsafe(no_mangle)]
#[unsafe(link_section = ".text.clip")]
pub extern "C" fn geo_clip() {
    let fptr = peek(GEO_ARGS + geo_args::FPTR) as u16 | (peek(GEO_ARGS + geo_args::FPTR + 1) as u16) << 8;
    let nf = peek(GEO_ARGS + geo_args::NF) as usize;
    let faces = unsafe { core::slice::from_raw_parts(fptr as *const u8, nf * geom::scene::FACE) };
    let verts = unsafe { &mut *(chopper::layout::VERTS as *mut Verts) };
    let work = unsafe { &mut *(WORK as *mut Work) };
    geom::clip(faces, verts, work);
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
#[inline(always)]
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
#[inline(always)]
fn wait_image(c: &mut C64) {
    let mut header = [0u8; 10];
    let mut build = [0u8; 4];
    loop {
        hw::read_into(c, HEADER, BOUNCE, &mut header);
        hw::read_into(c, HEADER + H_BUILD, BOUNCE, &mut build);
        if &header[..8] == MAGIC && u16::from_le_bytes([header[8], header[9]]) == PACKAGE_VERSION && u32::from_le_bytes(build) == unsafe { read_volatile(&raw const CHOPPER_BUILD) } {
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
    // The cockpit's static panel (from its overlay; the gauges follow on the first frame).
    let (at, size) = cockpit_overlay();
    hw::fetch(c, OVERLAY_COCKPIT, at, size);
    cockpit::panel(c);
    poke(0xD020, 0);
    poke(0xD021, COLORS[0]);
    poke(0xDD02, peek(0xDD02) | 3);
    poke(0xDD00, (peek(0xDD00) & !3) | buf.dd00);
    poke(0xD018, buf.d018);
    poke(0xD016, layout::D016);
    wait_vblank();
    poke(0xD011, layout::D011);
}

/// While the rotor sound streams (its first ~80 s): its decoder's overlay fetched and the ring filled on
/// to half a lap ahead of the playback (`now` frames since the sampler started). Then never again.
#[inline(never)]
#[unsafe(link_section = ".text.spare.stream")]
fn stream(c: &mut C64, now: u16) {
    if unsafe { read_volatile(&raw const stream_state) } == 0 {
        hw::fetch(c, OVERLAY_STREAM, WORK, unsafe { &raw const __streamtext_size } as u16);
        unsafe { stream_run(now as u8, (now >> 8) as u8) };
    }
}

/// The player's controls as `agent::input` bits: joystick 2 (its second button on POT X), or the keys:
/// W, A, S, D for the stick, SPACE to fire, either SHIFT for a missile. (In the spare region: the
/// resident code is full.) The keys are read once a frame, and not while the stick is pushed.
///
/// The stick and POT X are read first, with CIA 1 port A as the boot left it (bits 7/6 select the
/// paddle pair); the stick pushed, it alone (its lines would select rows of their own). Else the keyboard
/// matrix: port A all outputs, each row low in turn, its keys low on port
/// B. Columns already low with no row selected are joystick 1 (or noise), not keys: they are masked out
/// (else its fire would read as SPACE). Then port A back as it was: the SID's next POT X sample, long
/// before the next frame's read, sees the port 2 pair again.
#[inline(never)]
#[unsafe(link_section = ".text.spare.keys")]
fn controls() -> u8 {
    use chopper::agent::input::{DOWN, FIRE, FIRE2, LEFT, RIGHT, UP};
    let mut bits = !peek(0xDC00) & 0x1F;
    if peek(0xD419) < 0x80 {
        bits |= FIRE2;
    }
    // The stick pushed: it alone. (Its lines pull port A low, selecting keyboard rows of their own: the
    // scan would lose keys or find ones not pressed.)
    if bits & 0x1F != 0 {
        return bits;
    }
    poke(0xDC02, 0xFF);
    poke(0xDC00, 0xFF);
    let held = !peek(0xDC01);
    // Row 1: W column 1, A 2, S 5, left SHIFT 7; row 2: D column 2; row 6: right SHIFT column 4; row 7:
    // SPACE column 4. (Each row's keys moved onto their bits.)
    poke(0xDC00, !0x02);
    let r1 = !peek(0xDC01) & !held;
    poke(0xDC00, !0x04);
    let r2 = !peek(0xDC01) & !held;
    poke(0xDC00, !0x40);
    let r6 = !peek(0xDC01) & !held;
    poke(0xDC00, !0x80);
    let r7 = !peek(0xDC01) & !held;
    port_a_paddles();
    bits |= (r1 >> 1) & UP | (r1 >> 4) & DOWN | r1 & LEFT | r2 << 1 & RIGHT | (r1 >> 2 | r6 << 1) & FIRE2 | r7 & FIRE;
    bits
}

/// CIA 1 port A for joystick 2's second button on POT X: bits 7/6 outputs, selecting the paddle pair
/// (%10: port 2). At boot, and after each keyboard scan (`controls`).
#[inline(always)]
fn port_a_paddles() {
    poke(0xDC02, 0xC0);
    poke(0xDC00, 0x80);
}

/// The near platoons' vehicles in formation (`squad::expand`), from their overlay. (In the spare region:
/// the resident code is full.)
#[inline(never)]
#[unsafe(link_section = ".text.spare.squad")]
fn squad(c: &mut C64, agent: &chopper::agent::Agent) {
    hw::fetch(c, chopper::format::OVERLAY_SQUAD, WORK, unsafe { &raw const __squadtext_size } as u16);
    let (pu, pv) = agent.at();
    chopper::squad::expand(c, agent.t as u16, pu, pv, agent.yaw);
}

/// Everything before the frame buffer is first drawn: this code runs from the frame buffer itself
/// (`.text.boot`, target-c64/link.ld), so it must not touch it. Returns the turbo probes.
#[inline(never)]
#[unsafe(link_section = ".text.boot")]
fn boot(c: &mut C64) -> u16 {
    let probes = wait_turbo();
    hw::init(c);
    wait_image(c);
    // The rotor sound: its ring filled ahead by the decoder's overlay (in the window, free until the first
    // frame), then the sampler started round it.
    hw::fetch(c, OVERLAY_STREAM, WORK, unsafe { &raw const __streamtext_size } as u16);
    if unsafe { stream_init() } != 0 {
        let divider = unsafe { stream_calibrate() };
        while unsafe { stream_run(0, 0) } == 0 {}
        unsafe { stream_start(divider as u8, (divider >> 8) as u8) };
    }
    // The frame clock starts with the sound, so the flight's time is the sound's (`agent`) and the frames
    // since are how far it has played (the decoder's pacing): CIA 2 timer A underflows once an NTSC frame,
    // timer B counts the underflows.
    poke16(0xDD04, FRAME_CYCLES - 1);
    poke16(0xDD06, 0xFFFF);
    poke(0xDD0F, 0x51); // B: start, force load, count A underflows
    poke(0xDD0E, 0x11); // A: start, force load, continuous
    chopper::fixed::init_tables();
    scene_load(c);
    // Shade patterns for asm/geom.S: even bytes, then odd.
    for (i, p) in chopper::poly::PATTERNS.iter().enumerate() {
        poke(PATS + i as u16, p[0]);
        poke(PATS + 16 + i as u16, p[1]);
    }
    for level in 0..floor::TABLE_LEVELS {
        // Entry by entry (`floor::tile_table`): a 256-byte table on the soft stack would push the boot's
        // frame into llvm-mos's static frames.
        for i in 0..=255u8 {
            poke(TILES + level as u16 * 256 + i as u16, floor::tile_bank(level, i & 15, i >> 4));
        }
    }
    // The HUD: hires sprites in light green in front of the view, multiplexed by asm/hud.S from an
    // empty table until the first frame's is ready.
    port_a_paddles();
    // The sound effects' state afresh (RAM outlives a reset: the first frame sets it up and silences the
    // SIDs), and no pitch sweeps or slews for the IRQ till then (each slew's target its value).
    poke(chopper::sfx::STATE + chopper::sfx::state::MAGIC, 0);
    for p in 0..chopper::sfx::PAIRS {
        poke(chopper::sfx::SWEEP + p * chopper::sfx::SWEEP_PAIR, 0);
    }
    for ch in 0..2 * chopper::sfx::SLEW_CH {
        poke(chopper::sfx::SLEW + ch, 0);
    }
    poke(chopper::sfx::HOLD, 0);
    poke(0xD015, 0);
    poke(0xD017, 0);
    poke(0xD01B, 0);
    poke(0xD01C, 0);
    poke(0xD01D, 0);
    for k in 0..8 {
        poke(0xD027 + k, chopper::hud::COLOUR);
    }
    chopper::hud::init(c);
    let vector = hud_irq as usize as u16;
    poke16(0xFFFE, vector);
    poke(0xD012, chopper::hud::TOP_LINE);
    poke(0xD011, peek(0xD011) & 0x7F);
    poke(0xD019, 0xFF);
    poke(0xD01A, 0x01);
    unsafe { hud_irq_enable() };
    probes
}

/// Fetch the scene block to `SCENE_RAM`; its length.
#[inline(always)]
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
    let probes = boot(c);
    // The frame's triangles live at WORK (outside the program image); asm/geom.S reads the scene at
    // SCENE_RAM and keeps the vertices at VERTS.
    let work = unsafe { &mut *(WORK as *mut Work) };
    video_init(c);


    // The helicopter on the deck, its clock the sound's (tools/emu.py --start overwrites both once MAGIC
    // is set).
    let mut seen = frames_elapsed();
    let agent = unsafe { &mut *(AGENT as *mut Agent) };
    // (On the carrier's deck, placed at the first flight.)
    *agent = Agent::boot();
    unsafe { write_volatile(CLOCK as *mut u32, seen as u32) };
    let (ai_size, hud_size, geom_size, actors_size) = overlay_sizes();
    let (cockpit_at, cockpit_size) = cockpit_overlay();
    let gauges = unsafe { &mut *(GAUGES as *mut [u8; cockpit::gauge::COUNT]) };
    *gauges = [0xFF; cockpit::gauge::COUNT];
    poke(chopper::world_sim::NEAR, 0);
    poke(chopper::world_sim::PHASE, chopper::world_sim::MOVE);
    poke(chopper::world_sim::TURN, 0);
    chopper::combat::init(c);

    c.fill(STATUS, status::SIZE, 0);
    poke16(STATUS + status::PROBES, probes);
    // Adaptive lines (`floor::double_mask`). The host can still set full or line-doubled ones.
    poke(STATUS + status::MODE, status::mode::ADAPTIVE);
    poke(STATUS + status::MAGIC, STATUS_MAGIC);

    let mut frames = 0u16;
    let (mut window, mut in_window) = (frames_elapsed(), 0u8);
    let args = unsafe { &mut *(RAYS as *mut SpanArgs) };
    loop {
        let now = frames_elapsed();
        let mut clock = unsafe { read_volatile(CLOCK as *const u32) };
        if peek(STATUS + status::HOLD) == 0 {
            clock += now.wrapping_sub(seen) as u32;
            unsafe { write_volatile(CLOCK as *mut u32, clock) };
        }
        seen = now;
        // The stick and the keys, into the agent before its frame (the snapshot holds it).
        agent.input = controls();
        if agent.input != 0 {
            // The player takes over from the autopilot: any control.
            agent.take_over();
        }
        // Three stopwatches a frame (tools/emu.py finds the frames by them): set-up, geometry, the bands.
        stopwatch_start();
        // The agent catches up with the clock, from its overlay over the (spent) triangles at WORK.
        hw::fetch(c, OVERLAY_AI, WORK, ai_size);
        // The carrier's deck with it (the second window: the band state is spent; the war's move phase
        // uses it too).
        hw::fetch(c, OVERLAY_DECK, chopper::layout::BAND_RAM, unsafe { &raw const __decktext_size } as u16);
        agent.fly(&mut Senses(c), clock);
        let (cam, _yaw) = agent.camera();
        let t = agent.t;
        // The war and the gauges take turns (`world_sim::TURN`).
        let turn = peek(chopper::world_sim::TURN);
        poke(chopper::world_sim::TURN, turn ^ 1);
        if turn == 0 {
            // The war: a page of units, its move, decide or fight phase (each its own overlay, over the AI's).
            let (ovl, size) = match peek(chopper::world_sim::PHASE) {
                chopper::world_sim::MOVE => (OVERLAY_WORLD, unsafe { &raw const __worldtext_size } as u16),
                chopper::world_sim::DECIDE => (OVERLAY_WORLD3, unsafe { &raw const __world3text_size } as u16),
                chopper::world_sim::DIRECT => (OVERLAY_WORLD4, unsafe { &raw const __world4text_size } as u16),
                _ => (OVERLAY_WORLD2, unsafe { &raw const __world2text_size } as u16),
            };
            hw::fetch(c, ovl, WORK, size);
            let (pu, pv) = agent.at();
            chopper::world_sim::step_page(c, t as u16, pu, pv);
        }
        // The near platoons' vehicles in formation (squad overlay), for the player's fight (combat overlay,
        // its library over the spent band state): down, the autopilot takes over again at home. Then the
        // effects pass (its own overlay).
        squad(c, agent);
        hw::fetch(c, OVERLAY_COMBATLIB, chopper::layout::BAND_RAM, unsafe { &raw const __combatlibtext_size } as u16);
        hw::fetch(c, OVERLAY_COMBAT, WORK, unsafe { &raw const __combattext_size } as u16);
        if chopper::combat::update(c, agent, &cam) {
            agent.reset();
        }
        hw::fetch(c, OVERLAY_FX, WORK, unsafe { &raw const __fxtext_size } as u16);
        let (cam, _) = agent.camera();
        chopper::combat::fx(c, agent, &cam);
        // Its sounds (their own overlay, and their library in the second window: the band state's spent, and
        // the combat library done with).
        hw::fetch(c, chopper::format::OVERLAY_SFXLIB, chopper::layout::BAND_RAM, unsafe { &raw const __sfxlibtext_size } as u16);
        chopper::sfx::frame(c, agent);
        poke(0xD020, if peek(chopper::combat::PLAYER + chopper::combat::player::FLASH) != 0 { 2 } else { 0 });
        // The gauges whose readings changed, from their overlay over the (spent) band state.
        hw::fetch(c, OVERLAY_COCKPIT, cockpit_at, cockpit_size);
        if turn != 0 {
            cockpit::update(c, agent, gauges);
        }
        // The HUD: the targets read (cockpit overlay), the sprites placed (HUD overlay, over the AI's), the
        // shapes and the IRQ's table written (cockpit overlay).
        chopper::hud::prepare(c);
        hw::fetch(c, OVERLAY_HUD, WORK, hud_size);
        chopper::hud::build(c, agent, &cam);
        chopper::hud::finish(c);
        // The moving objects (actors overlay, over the HUD's).
        hw::fetch(c, OVERLAY_ACTORS, WORK, actors_size);
        chopper::actors::update(c, &cam);
        floor::write_fog(&cam);
        stream(c, now);
        // The floor's rays and the grids' set-up from their overlay (then the window is the frame's
        // geometry again).
        hw::fetch(c, OVERLAY_GEOM, WORK, geom_size);
        floor::write_pairs(c, &cam);
        let rays = Rays::new(&cam);
        *args = SpanArgs::new(&cam, &rays);
        unsafe { bt_build() };
        poke16(STATUS + status::SETUP_US, stopwatch_stop());
        // Geometry: cull, transform and sort into the depth buckets, then file by first band.
        stopwatch_start();
        geom::walk_args(c, WALK_ARGS, &View { cam, pop: peek(STATUS + status::POP) });
        work.clear();
        // The near-plane clipping `geo_faces` calls (`geo_clip`), above the triangle work.
        hw::fetch(c, OVERLAY_CLIP, chopper::layout::CLIP_RAM, unsafe { &raw const __cliptext_size } as u16);
        unsafe { geo_walk() };
        let verts = unsafe { &mut *(chopper::layout::VERTS as *mut Verts) };
        // The moving objects into the buckets (their overlay over the spent band state).
        hw::fetch(c, OVERLAY_DYN, cockpit_at, unsafe { &raw const __dyntext_size } as u16);
        geom::push_dynamic(c, work, verts);
        unsafe { geo_bin() };
        poke16(STATUS + status::GEOM_US, stopwatch_stop());

        stopwatch_start();
        unsafe { floor_begin() };
        // The bands can take longer than the 16-bit stopwatch: add it up band by band.
        let (mut render, mut last) = (0u32, 0u16);
        // Of it, the floor's lines and the triangles (the status block's split; the rest is publishing).
        let (mut floor_t, mut draw_t) = (0u32, 0u32);
        // Line y's ray at byte 0, stepped (exactly) line by line in place (`ray_step`, twice for a pair).
        args.a = rays.a0;
        unsafe { core::ptr::write_volatile(chopper::layout::RAY_STEP as *mut [i32; 3], rays.ay) };
        for band in 0..VIEW_BANDS as u8 {
            // (Sprites fetch from the REU between bands: the floor starts each band afresh.)
            let active = unsafe {
                floor_begin();
                geo_activate(band)
            };
            let lines = stopwatch_read();
            draw_t += lines.wrapping_sub(last) as u32;
            // The band's pairs drawn once (`floor::write_pairs`: none, all, or the adaptive ones).
            let pairs = peek(chopper::layout::DOUBLE_MASK + band as u16);
            let mut line = 0u8;
            while line < 8 {
                let doubled = floor::doubled(pairs, line);
                args.haze = floor::haze_byte(line);
                unsafe {
                    span_line();
                    floor_line();
                    floor_copy(line);
                    ray_step();
                    if doubled {
                        line += 1;
                        fog_second();
                        floor_copy(line);
                        ray_step();
                    }
                }
                line += 1;
            }
            let drawn = stopwatch_read();
            floor_t += drawn.wrapping_sub(lines) as u32;
            if active > 0 {
                unsafe { geo_draw(band) };
            }
            draw_t += stopwatch_read().wrapping_sub(drawn) as u32;
            unsafe { band_publish(band) };
            let now = stopwatch_read();
            render += now.wrapping_sub(last) as u32;
            last = now;
        }
        unsafe { floor_end() };
        // The frame is on screen: its HUD table goes up at the next top-border IRQ.
        poke(chopper::hud::READY, 1);
        render += stopwatch_stop().wrapping_sub(last) as u32;
        poke16(STATUS + status::RENDER_US, (render >> 2) as u16);
        poke16(STATUS + status::FLOOR_US, (floor_t >> 2) as u16);
        poke16(STATUS + status::DRAW_US, (draw_t >> 2) as u16);
        poke(STATUS + status::TRIS, peek(GEO_ARGS + geo_args::TRIS));
        poke(STATUS + status::OBJECTS, peek(WALK_ARGS + geom::walk::OBJECTS));
        poke16(STATUS + status::TICK, t as u16);
        poke16(STATUS + status::TICK_HI, (t >> 16) as u16);

        frames = frames.wrapping_add(1);
        poke16(STATUS + status::FRAMES, frames);
        in_window += 1;
        if now.wrapping_sub(window) >= 60 {
            poke(STATUS + status::FPS, in_window);
            window = now;
            in_window = 0;
        }

    }
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        poke(0xD020, 2);
    }
}
