//! Moving objects: the units near the player, their projectiles, drawn among the static scene each frame.
//!
//! Each frame's moving objects ([`Object`]: mesh, yaw, position) are culled and taken into camera space
//! in the set-up ([`prepare`], into [`DYN`]); after the static objects are walked, `geom::push_dynamic`
//! draws each: its mesh streamed from the REU (`format::VEHICLES`) into [`MESHBUF`], turned to any yaw,
//! into the same depth buckets.
//!
//! Until the living world is in (`world_sim`), [`demo`] fills the list: a convoy driving a loop round the
//! home pad and a gunship circling above it.

use crate::hw::Hw;

/// Unit kinds (`docs/design.md`'s enemies), and the projectiles.
pub mod kind {
    pub const TRUCK: u8 = 0;
    pub const APC: u8 = 1;
    pub const TANK: u8 = 2;
    pub const AA: u8 = 3;
    pub const SAM: u8 = 4;
    pub const RADAR: u8 = 5;
    pub const GUNBOAT: u8 = 6;
    pub const MISSILE_BOAT: u8 = 7;
    pub const GUNSHIP: u8 = 8;
    pub const DEPOT: u8 = 9;
    pub const BUNKER: u8 = 10;
    pub const CARRIER: u8 = 11;
    pub const DESTROYER: u8 = 12;
    pub const TRACER: u8 = 13;
    pub const MISSILE: u8 = 14;
    pub const COUNT: u8 = 15;
    pub const NAMES: [&str; COUNT as usize] = [
        "truck", "apc", "tank", "aa", "sam", "radar", "gunboat", "missile-boat", "gunship", "depot", "bunker", "carrier", "destroyer",
        "tracer", "missile",
    ];
}
/// The sides.
pub const ENEMY: u8 = 0;
pub const ALLIED: u8 = 1;

/// A kind's mesh in `format::VEHICLES` (each kind in both sides' colours).
pub const fn mesh_of(kind: u8, side: u8) -> u8 {
    kind * 2 + (side & 1)
}

/// The carrier's flight deck (`scene::deck`), after the wrecks.
pub const DECK: u8 = kind::COUNT * 3;

/// A kind's wreck's mesh (after both sides' meshes, `scene::vehicles`).
pub const fn wreck_of(kind: u8) -> u8 {
    kind::COUNT * 2 + kind % kind::COUNT
}

/// The mesh table's records: blob offset u16, blob length u8, the far mesh (0: none), vertices, faces,
/// shift, radius (`scene::vehicle_bank`).
pub const MESH_RECORD: usize = 8;
/// How much larger the units are drawn than they are modelled (`scene::vehicles`): at their modelled 1-2
/// texels they were a few pixels at fighting range, too small to see as targets. The sizes that follow
/// the drawn units are the modelled ones times this: the carrier's deck (`world_sim::DECK_TOP`,
/// `DECK_HALF`, `agent::DECK_AFT`), the formations' spacing (`squad::half`), the wrecks' smoke.
/// Projectiles keep their size.
pub const UNIT_SCALE: i16 = 2;
/// Units further out than this along the view (Q4: 48 texels, where they are a few pixels across) are
/// drawn with their far mesh (`scene::far`).
pub const LOD_Z: i32 = 48 * 16;

/// The frame's moving objects, ready to draw (scratch during the set-up and the geometry: the band
/// stage, after the HUD's scratch; the sprite records after it are unused): a count, then [`DYN_ENTRY`]
/// bytes each, as [`prepare`] leaves them for `geom::push_dynamic`: the mesh's blob (REU offset in
/// `format::VEHICLES` u16, length u8), its vertices, faces and shift, its centre in camera space (Q4,
/// 3 × i16), and the sine and cosine of its yaw (Q14, i16).
pub const DYN: u16 = crate::layout::STAGE + 0xA0;
pub const DYN_ENTRY: u16 = 16;
pub const DYN_MAX: u8 = 12;
pub mod dyn_entry {
    pub const BLOB: u16 = 0;
    pub const LEN: u16 = 2;
    pub const NV: u16 = 3;
    pub const NF: u16 = 4;
    pub const SHIFT: u16 = 5;
    pub const ORG: u16 = 6;
    pub const SIN: u16 = 12;
    pub const COS: u16 = 14;
}
const _: () = assert!(DYN + 1 + DYN_MAX as u16 * DYN_ENTRY <= crate::layout::JOB);
const _: () = assert!(DYN >= crate::hud::S_COUNT + 1);

/// One mesh at a time, streamed from the REU for the object being drawn (after the relocated `spantext`
/// pages, before the HUD's page).
pub const MESHBUF: u16 = 0x8A10;
pub const MESHBUF_LEN: u16 = 0xA0;
const _: () = assert!(MESHBUF + MESHBUF_LEN <= crate::combat::WRECKS);

/// An object in the world: mesh, yaw (256 a turn), position in 1/16 texel.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Object {
    pub mesh: u8,
    pub yaw: u8,
    pub u: u16,
    pub v: u16,
    pub h: u16,
}

/// Whether a mesh is the player's tracer's or missile's.
#[inline(always)]
fn projectile(mesh: u8) -> bool {
    mesh == mesh_of(kind::TRACER, ALLIED) || mesh == mesh_of(kind::MISSILE, ALLIED)
}

/// The objects in view into [`DYN`], ready to draw: each one's centre into camera space, culled like a
/// static object (its bounding sphere against the view, and out past its pop-in distance). The
/// projectiles first, then the rest in the list's order: at [`DYN_MAX`], what is cut is the last listed
/// (the farthest vehicles: `squad::expand` lists them nearest first), never the player's own fire.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.actors"))]
pub fn prepare<H: Hw + ?Sized>(hw: &mut H, cam: &crate::floor::FloorCam, list: u16) {
    use crate::hud::mul16;
    let mut n = 0u8;
    let count = hw.peek(list) as u16;
    for k in 0..2 * count {
        if n >= DYN_MAX {
            break;
        }
        // Object `k` of the list (mesh, yaw, u, v, h): read in place, not copied onto the stack. (Twice
        // round: the projectiles, then the rest.)
        let at = list + 1 + 8 * if k < count { k } else { k - count };
        if projectile(hw.peek(at)) != (k < count) {
            continue;
        }
        let rd = |hw: &mut H, a: u16| hw.peek(a) as u16 | (hw.peek(a + 1) as u16) << 8;
        let o = Object { mesh: hw.peek(at), yaw: hw.peek(at + 1), u: rd(hw, at + 2), v: rd(hw, at + 4), h: rd(hw, at + 6) };
        let mut rec = [0u8; MESH_RECORD];
        crate::hw::read_into(hw, crate::format::VEHICLES + o.mesh as u32 * MESH_RECORD as u32, crate::layout::BOUNCE, &mut rec);
        let rad = rec[7];
        let d = [
            (o.u as i32 - (cam.u >> 4)).clamp(-32767, 32767) as i16,
            (o.h as i32 - (cam.h as i32 >> 4)).clamp(-32767, 32767) as i16,
            (o.v as i32 - (cam.v >> 4)).clamp(-32767, 32767) as i16,
        ];
        let mut c = [0i32; 3];
        for (j, cj) in c.iter_mut().enumerate() {
            *cj = (mul16(cam.r[0][j] as i16, d[0]) + mul16(cam.r[1][j] as i16, d[1]) + mul16(cam.r[2][j] as i16, d[2])) >> 14;
        }
        // In front, inside `FAR` (moving objects are targets: drawn far out, a few pixels across, not popped
        // in like a tree), inside the side planes (45°) and the top and bottom ones (|Y|·2 against Z: a
        // little wider than the view's 133/68).
        let (x, y, z, r) = (c[0], c[1], c[2], (rad as i32) << 4);
        if z + r < crate::geom::NEAR || z - r > FAR || x.abs() - z > r + (r >> 1) || (y.abs() << 1) - z > r * 3 {
            continue;
        }
        // Far off, the unit's far mesh (culled by the near one's sphere: its footprint, a little wider at most).
        if z > LOD_Z && rec[3] != 0 {
            crate::hw::read_into(hw, crate::format::VEHICLES + rec[3] as u32 * MESH_RECORD as u32, crate::layout::BOUNCE, &mut rec);
        }
        let a = (o.yaw as u16) << 8;
        let at = DYN + 1 + n as u16 * DYN_ENTRY;
        let w = [u16::from_le_bytes([rec[0], rec[1]]), c[0] as u16, c[1] as u16, c[2] as u16, crate::camera::sin(a) as u16, crate::camera::cos(a) as u16];
        hw.poke(at + dyn_entry::BLOB, w[0] as u8);
        hw.poke(at + dyn_entry::BLOB + 1, (w[0] >> 8) as u8);
        hw.poke(at + dyn_entry::LEN, rec[2].min(MESHBUF_LEN as u8));
        hw.poke(at + dyn_entry::NV, rec[4]);
        hw.poke(at + dyn_entry::NF, rec[5]);
        hw.poke(at + dyn_entry::SHIFT, rec[6]);
        for (k, &x) in w[1..].iter().enumerate() {
            hw.poke(at + dyn_entry::ORG + 2 * k as u16, x as u8);
            hw.poke(at + dyn_entry::ORG + 2 * k as u16 + 1, (x >> 8) as u8);
        }
        n += 1;
    }
    hw.poke(DYN, n);
}

/// How far out moving objects are drawn (Q4: 200 texels).
pub const FAR: i32 = 200 * 16;

/// The frame's moving objects (`combat::OBJLIST`: the war's units near the player moved on from their
/// last step, and the projectiles), ready to draw.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.actors"))]
pub fn update<H: Hw + ?Sized>(hw: &mut H, cam: &crate::floor::FloorCam) {
    let n = hw.peek(crate::combat::OBJLIST).min(crate::combat::OBJLIST_MAX);
    hw.poke(crate::combat::OBJLIST, n);
    prepare(hw, cam, crate::combat::OBJLIST);
}

/// The demo's loop round the home pad: centre and radius (texels).
pub const DEMO_CENTRE: (i32, i32) = (1024, 820);
pub const DEMO_RADIUS: i32 = 40;

/// The demo for tick `t`: a convoy of five (truck, APC, tank, AA, truck) on a circle, a quarter turn of
/// it every 1024 ticks, and an allied gunship circling the other way above.
#[inline(never)]
#[cfg_attr(target_arch = "mos", unsafe(link_section = ".text.actors"))]
pub fn demo<H: Hw + ?Sized>(hw: &mut H, t: u32, cam: &crate::floor::FloorCam) {
    let mut objects = [Object::default(); 6];
    let convoy = [kind::TRUCK, kind::APC, kind::TANK, kind::AA, kind::TRUCK];
    let base = (t << 4) as u16;
    for (k, &kd) in convoy.iter().enumerate() {
        // The heading along the circle (counter-clockwise), the unit `k` × 5° behind the leader.
        let a = base.wrapping_sub(k as u16 * 910);
        let (s, c) = (crate::camera::sin(a), crate::camera::cos(a));
        let u = (DEMO_CENTRE.0 << 4) + (c * DEMO_RADIUS >> 10);
        let v = (DEMO_CENTRE.1 << 4) + (s * DEMO_RADIUS >> 10);
        // Heading (clockwise from north, as the agent's): along the circle, counter-clockwise.
        objects[k] = Object { mesh: mesh_of(kd, ENEMY), yaw: (a.wrapping_neg() >> 8) as u8, u: u as u16, v: v as u16, h: 0 };
    }
    let a = base.wrapping_mul(2).wrapping_neg();
    let (s, c) = (crate::camera::sin(a), crate::camera::cos(a));
    let u = (DEMO_CENTRE.0 << 4) + (c * (DEMO_RADIUS + 20) >> 10);
    let v = (DEMO_CENTRE.1 << 4) + (s * (DEMO_RADIUS + 20) >> 10);
    objects[5] = Object { mesh: mesh_of(kind::GUNSHIP, ALLIED), yaw: (0x8000u16.wrapping_sub(a) >> 8) as u8, u: u as u16, v: v as u16, h: 12 << 4 };
    let list = crate::combat::OBJLIST;
    hw.poke(list, objects.len() as u8);
    for (k, o) in objects.iter().enumerate() {
        let at = list + 1 + 8 * k as u16;
        for (i, x) in [o.mesh, o.yaw, o.u as u8, (o.u >> 8) as u8, o.v as u8, (o.v >> 8) as u8, o.h as u8, (o.h >> 8) as u8].into_iter().enumerate() {
            hw.poke(at + i as u16, x);
        }
    }
    prepare(hw, cam, list);
}
