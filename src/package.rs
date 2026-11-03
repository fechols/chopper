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
    crate::scene::install(&mut reu, &crate::scene::bake(world));
    let h = HEADER as usize;
    reu[h..h + 8].copy_from_slice(MAGIC);
    reu[h + H_VERSION as usize..][..2].copy_from_slice(&PACKAGE_VERSION.to_le_bytes());
    let brain = crate::goap::compile();
    reu[AI_TREES as usize..][..brain.len()].copy_from_slice(&brain);
    crate::instruments::install(&mut reu, &crate::instruments::bake());
    for (k, b) in crate::hud::art::shapes().iter().enumerate() {
        reu[HUD_SHAPES as usize + k * 64..][..64].copy_from_slice(b);
    }
    let brains = crate::roles::compile();
    assert!(brains.len() <= crate::brain::UNIT_BYTES);
    reu[TREES as usize..][..brains.len()].copy_from_slice(&brains);
    let war = crate::war::bake(world);
    reu[WORLD as usize..][..war.len()].copy_from_slice(&war);
    let vehicles = crate::scene::vehicle_bank();
    reu[VEHICLES as usize..][..vehicles.len()].copy_from_slice(&vehicles);
    let targets = crate::hud::art::targets(world);
    reu[HUD_TARGETS as usize..][..targets.len()].copy_from_slice(&targets);
    let hazard = crate::goap::hazard_map(world);
    reu[HAZARD as usize..][..hazard.len()].copy_from_slice(&hazard);
    let dlt = crate::digital::ROTOR;
    let d = crate::digital::Dlt5::parse(dlt).expect("rotor.dlt");
    // asm/audio.S decodes whole 8-frame groups only.
    assert!(dlt.len() <= AUDIO_MAX as usize && d.frames == crate::digital::ROTOR_LOOP_END && d.frames % crate::digital::GROUP == 0);
    reu[AUDIO_DLT as usize..][..dlt.len()].copy_from_slice(dlt);
    for (at, v) in [
        (H_AUDIO_BYTES, dlt.len() as u32),
        (H_LOOP_START, crate::digital::ROTOR_LOOP_START),
        (H_LOOP_END, crate::digital::ROTOR_LOOP_END),
        (H_STREAM, AUDIO_DLT + d.stream_offset() as u32),
        (H_GROUPS, d.frames / crate::digital::GROUP),
    ] {
        reu[h + at as usize..][..4].copy_from_slice(&v.to_le_bytes());
    }
    for (at, v) in [(H_SEG_GROUPS, (d.per_segment / crate::digital::GROUP) as u16), (H_SEG_LONG, d.long)] {
        reu[h + at as usize..][..2].copy_from_slice(&v.to_le_bytes());
    }
    crate::effects_bake::install(&mut reu);
    reu
}

/// The C64's screen set-up on a model: colours, the view cleared to sky, the cockpit's static panel
/// (`cockpit::panel`, from the REU).
pub fn setup(m: &mut Machine, buf: Buffer) {
    use crate::hw::Hw;
    m.poke(0xD021, COLORS[0]);
    m.fill(buf.screen, layout::CELLS, COLORS[1] << 4 | COLORS[2]);
    m.fill(layout::COLOR_RAM, layout::CELLS, COLORS[3]);
    m.fill(buf.bitmap, VIEW_BANDS * layout::ROW_STRIDE, floor::SKY_BYTE);
    crate::cockpit::panel(m);
}

/// Copy the C64 build's REU overlays and its build stamp into an image (`chopper bake --elf`): the PRG
/// only runs with an image baked from its own ELF.
pub fn install_overlays(reu: &mut [u8], elf: &[u8]) -> Result<(), String> {
    for (name, at, max) in [
        (".aitext", OVERLAY_AI, OVERLAY_MAX),
        (".hudtext", OVERLAY_HUD, OVERLAY_MAX),
        (".geomtext", OVERLAY_GEOM, OVERLAY_MAX),
        (".actorstext", OVERLAY_ACTORS, OVERLAY_MAX),
        (".cocktext", OVERLAY_COCKPIT, OVERLAY2_MAX),
        (".dyntext", OVERLAY_DYN, OVERLAY2_MAX),
        (".cliptext", OVERLAY_CLIP, OVERLAY3_MAX),
        (".worldtext", OVERLAY_WORLD, OVERLAY_MAX),
        (".world2text", OVERLAY_WORLD2, OVERLAY_MAX),
        (".world3text", OVERLAY_WORLD3, OVERLAY_MAX),
        (".world4text", OVERLAY_WORLD4, OVERLAY_MAX),
        (".combattext", OVERLAY_COMBAT, OVERLAY_MAX),
        (".fxtext", OVERLAY_FX, OVERLAY_MAX),
        (".combatlibtext", OVERLAY_COMBATLIB, OVERLAY2_MAX),
        (".decktext", OVERLAY_DECK, OVERLAY2_MAX),
    ] {
        let code = crate::elf::section(elf, name)?;
        if code.len() as u32 > max {
            return Err(format!("{name}: {} bytes, more than its window", code.len()));
        }
        reu[at as usize..][..code.len()].copy_from_slice(code);
    }
    let stamp = crate::elf::symbol(elf, "CHOPPER_BUILD", 4)?;
    reu[(HEADER + H_BUILD) as usize..][..4].copy_from_slice(stamp);
    Ok(())
}

/// What the host's floor did (for the frame report).
#[derive(Clone, Copy, Debug, Default)]
pub struct FloorStats {
    pub dmas: u32,
    pub io_writes: u32,
    pub spans: u32,
    pub ground_spans: u32,
    /// The geometry: objects drawn, triangles (and sprites) set up, dropped.
    pub geom: crate::geom::Stats,
    pub tris: u8,
    pub dropped: u8,
}

/// The scene block of an image (`format::SCENE`, its length in its first two bytes).
pub fn scene_block(reu: &[u8]) -> Vec<u8> {
    let at = SCENE as usize;
    let len = reu[at] as usize | (reu[at + 1] as usize) << 8;
    reu[at..at + len.max(crate::geom::scene::MESHES)].to_vec()
}

/// Draw the view's bands into `buf` exactly as the C64 does: the objects sorted first; then per band,
/// per drawn line, the runs into the line buffer, copied into the stage; the band's objects over them;
/// the stage published to the bitmap.
pub fn render_floor(m: &mut Machine, buf: Buffer, scene: &[u8], cam: &FloorCam, doubled: bool) -> FloorStats {
    use crate::geom::{self, Bands, View, Verts, Work};
    use crate::hw::Hw;
    let mut st = FloorStats::default();
    let mut work = Box::new(Work::new());
    let mut verts = Box::new(Verts::new());
    let view = View { cam: *cam, pop: 0 };
    st.geom = geom::draw(m, scene, &view, &mut work, &mut verts);
    geom::walk_args(m, layout::WALK_ARGS, &view);
    geom::push_dynamic(m, &mut work, &mut verts);
    let mut bands = Bands::new(&work);
    let rays = Rays::new(cam);
    let mut shadow;
    let mut spans = [Span::default(); MAX_SPANS];
    for band in 0..VIEW_BANDS {
        // (Sprites fetch from the REU between bands: the floor starts each band afresh, as on the C64.)
        shadow = floor::floor_begin(m);
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
        bands.band(m, &work, band as usize, STAGE);
        for i in 0..layout::ROW_STRIDE {
            let b = m.peek(STAGE + i);
            m.poke(buf.bitmap + band * layout::ROW_STRIDE + i, b);
        }
    }
    let _ = COLS;
    (st.tris, st.dropped) = (bands.tris, bands.dropped);
    st
}

/// Frame `tick` of the autopilot flight in buffer 0 of a fresh model of `reu`.
pub fn render(reu: &[u8], tick: u32, doubled: bool) -> (Machine, FloorStats) {
    render_from(reu, tick, doubled, [crate::hud::NONE, 0, 0, 0, 0])
}

/// [`render`], the HUD's lock-on carried on from `lock` (`hud::LOCK_STATE` before the frame: the lock
/// builds over the frames shown, so the C64's state before a frame is what this frame's follows from).
pub fn render_from(reu: &[u8], tick: u32, doubled: bool, lock: [u8; crate::hud::LOCK_BYTES]) -> (Machine, FloorStats) {
    use crate::hw::Hw;
    let mut m = Machine::new(reu);
    setup(&mut m, BUFFERS[0]);
    let agent = crate::agent::at(reu, tick);
    // The gauges: all drawn (the C64 redraws only those that changed, to the same bytes).
    crate::cockpit::update(&mut m, &agent, &mut [0xFF; crate::cockpit::gauge::COUNT]);
    let (cam, _yaw) = agent.camera();
    crate::hud::init(&mut m);
    for (i, &b) in lock.iter().enumerate() {
        m.poke(crate::hud::LOCK_STATE + i as u16, b);
    }
    crate::hud::update(&mut m, &agent, &cam);
    crate::actors::update(&mut m, &cam);
    let scene = scene_block(reu);
    let st = render_floor(&mut m, BUFFERS[0], &scene, &cam, doubled);
    (m, st)
}

/// One frame stepped from a snapshot of the C64's state at its start (`snapshot`), as the C64's main loop
/// does: the agent flies to its clock, the gauges and the HUD follow, the frame is drawn. The model holds
/// the state after it.
pub fn render_snapshot(reu: &[u8], snap: &crate::snapshot::Snapshot, doubled: bool) -> Result<(Machine, FloorStats), String> {
    use crate::hw::Hw;
    let mut m = Machine::new(reu);
    for (at, bytes) in &snap.reu {
        m.reu[*at as usize..][..bytes.len()].copy_from_slice(bytes);
    }
    setup(&mut m, BUFFERS[0]);
    for (at, bytes) in &snap.ram {
        for (i, &b) in bytes.iter().enumerate() {
            m.poke(at + i as u16, b);
        }
    }
    let mut agent = crate::agent::Agent::from_bytes(snap.ram_at(layout::AGENT, crate::agent::Agent::BYTES).ok_or("snapshot: no agent")?);
    let clock = u32::from_le_bytes(snap.ram_at(layout::CLOCK, 4).ok_or("snapshot: no clock")?.try_into().unwrap());
    let reu_now = m.reu.clone();
    agent.fly(&mut &reu_now[..], clock);
    let (pu, pv) = agent.at();
    // The war and the gauges take turns (`world_sim::TURN`).
    let turn = m.peek(crate::world_sim::TURN);
    m.poke(crate::world_sim::TURN, turn ^ 1);
    if turn == 0 {
        crate::world_sim::step_page(&mut m, agent.t as u16, pu, pv);
    }
    let (cam0, _) = agent.camera();
    if crate::combat::update(&mut m, &mut agent, &cam0) {
        agent.reset();
    }
    crate::combat::fx(&mut m, &agent, &agent.camera().0);
    for (i, b) in agent.to_bytes().iter().enumerate() {
        m.poke(layout::AGENT + i as u16, *b);
    }
    // The gauges: all drawn (the C64 redraws only those that changed, to the same bytes): on the war's
    // turn, as they were.
    let mut gauges = [0xFF; crate::cockpit::gauge::COUNT];
    if turn == 0 {
        for (g, k) in gauges.iter_mut().enumerate() {
            *k = m.peek(layout::GAUGES + g as u16);
            if *k != 0xFF {
                crate::cockpit::draw(&mut m, g, *k);
            }
        }
    } else {
        crate::cockpit::update(&mut m, &agent, &mut gauges);
    }
    for (i, &g) in gauges.iter().enumerate() {
        m.poke(layout::GAUGES + i as u16, g);
    }
    let (cam, _yaw) = agent.camera();
    let lock: Vec<u8> = (0..crate::hud::LOCK_BYTES as u16).map(|i| m.peek(crate::hud::LOCK_STATE + i)).collect();
    crate::hud::init(&mut m);
    for (i, &b) in lock.iter().enumerate() {
        m.poke(crate::hud::LOCK_STATE + i as u16, b);
    }
    crate::hud::update(&mut m, &agent, &cam);
    crate::actors::update(&mut m, &cam);
    let scene = scene_block(reu);
    let st = render_floor(&mut m, BUFFERS[0], &scene, &cam, doubled);
    Ok((m, st))
}

const _: () = assert!(LEVELS == 4);
