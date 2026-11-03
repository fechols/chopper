//! The war's opening (host): both sides' forces and the routes they use, baked into the page file
//! (`world_sim`), and the war run on its own (`chopper war`), with no player, for tests and reports.
//!
//! The enemy has landed on the north shore and holds the west outpost (pad 2) and the north (pad 4, its
//! HQ), with SAM sites in the ridge, boats off the west coast and gunships on patrol. The allies hold home
//! (pad 0), the east outpost (pad 1) and the field hospital (pad 3), run convoys between them, push a front
//! toward pad 2, and have a carrier group offshore with its gunships. Each side's reinforcement slots wait
//! dead until the director sends a wave.

use crate::actors::{ALLIED, ENEMY, kind};
use crate::world::{PADS, World, texel};
const _: () = assert!(PADS.len() == ws::PADS as usize);
use crate::world_sim::{self as ws, ALIVE, COUNT, LOOP, PING_PONG, RECORD, rec, stats};
#[allow(unused_imports)]
use LOOP as _;

/// Route numbers.
pub mod route {
    /// Allied supply road: home, east outpost, field hospital.
    pub const SUPPLY: u8 = 0;
    /// Enemy road: west outpost to HQ.
    pub const ENEMY_ROAD: u8 = 1;
    /// Enemy reinforcements: the beachhead to HQ, the west outpost and on toward the allies' pads, as far
    /// as the front (`world_sim::front`).
    pub const BEACHHEAD: u8 = 2;
    /// Allied reinforcements: home, the east outpost, toward the west outpost, HQ and the beachhead, as far
    /// as the front.
    pub const FRONT: u8 = 3;
    const _: () = assert!(crate::world_sim::FRONT_ROUTE[crate::actors::ENEMY as usize] == BEACHHEAD);
    const _: () = assert!(crate::world_sim::FRONT_ROUTE[crate::actors::ALLIED as usize] == FRONT);
    /// Enemy boats off the west coast.
    pub const WEST_SEA: u8 = 4;
    /// The carrier group's station offshore, south-east (back and forth).
    pub const CARRIER: u8 = 5;
    /// Air patrols.
    pub const ENEMY_AIR: u8 = 6;
    pub const ALLIED_AIR: u8 = 7;
    const _: () = assert!(ALLIED_AIR == crate::world_sim::CARRIER_AIR);
    /// Stays put.
    pub const NONE: u8 = 0xFF;
}

pub(crate) fn water(w: &World, u: i32, v: i32) -> bool {
    !(0..2048).contains(&u) || !(0..2048).contains(&v) || w.texels[v as usize * 2048 + u as usize] == texel::WATER
}

/// The first land along a line from (`u`, `v`) stepping (`du`, `dv`).
fn shore(w: &World, mut u: i32, mut v: i32, du: i32, dv: i32) -> (i32, i32) {
    while water(w, u, v) {
        u += du;
        v += dv;
    }
    (u, v)
}

/// How far inside the map's edge the sea routes keep (texels).
const SEA_ROOM: i32 = 48;

/// The routes' waypoints (texels) and modes.
pub fn routes(w: &World) -> Vec<(Vec<(i32, i32)>, u8)> {
    let pad = |k: usize| (PADS[k].u as i32, PADS[k].v as i32);
    // The beachhead: where the north shore is, north of HQ, and a little inland.
    let beach = shore(w, 980, 2040, 0, -1);
    let beach = (beach.0, beach.1 - 30);
    // Sea points: 70 texels off the coast along bearings from the island's centre (degrees, from east
    // toward north): the farthest out, up to 70 off, that is at sea and keeps a sea room inside the
    // map's edge (pulling a point in from the edge could land it back on the coast).
    let room = |x: i32| (SEA_ROOM..2048 - SEA_ROOM).contains(&x);
    let along = |deg: f32| {
        let (s, c) = deg.to_radians().sin_cos();
        let at = move |r: f32| ((1024.0 + c * r) as i32, (1024.0 + s * r) as i32);
        let mut last = 0.0;
        let mut r = 0.0;
        while r < 1000.0 {
            let (u, v) = at(r);
            if !water(w, u, v) {
                last = r;
            }
            r += 4.0;
        }
        let mut r = last + 70.0;
        while r >= last + 8.0 {
            let (u, v) = at(r);
            if room(u) && room(v) && water(w, u, v) {
                return Ok((u, v));
            }
            r -= 4.0;
        }
        Err(at(last + 70.0))
    };
    // (Where the coast runs out to the edge on a bearing, the nearest bearing with sea room; with none,
    // the bearing's own point pulled inside the room.)
    let offshore = |deg: f32| {
        along(deg).unwrap_or_else(|(u, v)| {
            [5.0, -5.0, 10.0, -10.0, 15.0, -15.0, 20.0, -20.0, 25.0, -25.0, 30.0, -30.0]
                .iter()
                .find_map(|d| along(deg + d).ok())
                .unwrap_or((u.clamp(SEA_ROOM, 2047 - SEA_ROOM), v.clamp(SEA_ROOM, 2047 - SEA_ROOM)))
        })
    };
    // The enemy flotilla raids from the west coast round the south; the carrier group's station overlaps
    // the end of it, so the fleets meet.
    let west = [offshore(200.0), offshore(230.0), offshore(260.0), offshore(290.0)];
    let carrier = [offshore(270.0), offshore(290.0), offshore(310.0), offshore(330.0)];
    vec![
        (vec![pad(0), pad(1), pad(3)], PING_PONG),
        (vec![pad(2), pad(4)], PING_PONG),
        // The front roads (`world_sim::FRONT_ROUTE`), each side's from its rear to the other's.
        (vec![beach, pad(4), pad(2), (1000, 980), pad(1), pad(0)], PING_PONG),
        (vec![pad(0), pad(1), (1000, 980), pad(2), pad(4), beach], PING_PONG),
        (west.to_vec(), PING_PONG),
        (carrier.to_vec(), PING_PONG),
        (vec![pad(4), pad(2), (1100, 1200), beach], LOOP),
        (vec![carrier[1], pad(0), pad(1), pad(3)], LOOP),
    ]
}

/// One unit to place: kind, side, route (or a fixed place), its share along the route (0..=255), height.
struct Unit {
    kind: u8,
    side: u8,
    route: u8,
    at: (i32, i32),
    along: u8,
    h: u8,
    alive: bool,
    hold: bool,
}

/// Where a share along a route is (texels), and the leg it is on.
fn along(rs: &[(Vec<(i32, i32)>, u8)], r: u8, share: u8) -> ((i32, i32), u8) {
    let (pts, _) = &rs[r as usize];
    let n = pts.len().max(2) - 1;
    let x = share as usize * n;
    let (leg, f) = (x / 256, (x % 256) as i32);
    let (a, b) = (pts[leg.min(pts.len() - 1)], pts[(leg + 1).min(pts.len() - 1)]);
    ((a.0 + (b.0 - a.0) * f / 256, a.1 + (b.1 - a.1) * f / 256), (leg + 1) as u8)
}

/// The war's opening: the page file (`COUNT` records), sectors, routes and header.
pub fn bake(w: &World) -> Vec<u8> {
    let rs = routes(w);
    let pad = |k: usize| (PADS[k].u as i32, PADS[k].v as i32);
    let mut units: Vec<Unit> = Vec::new();
    #[allow(clippy::too_many_arguments)]
    fn push(units: &mut Vec<Unit>, kind: u8, side: u8, route: u8, at: (i32, i32), along: u8, h: u8, alive: bool, hold: bool) {
        units.push(Unit { kind, side, route, at, along, h, alive, hold });
    }
    // Enemy: the column from the beachhead, garrisons, SAM sites, boats, gunships.
    for k in 0..24u8 {
        let kd = [kind::TRUCK, kind::APC, kind::TANK, kind::TRUCK, kind::AA, kind::TANK][k as usize % 6];
        push(&mut units, kd, ENEMY, route::BEACHHEAD, (0, 0), k * 6, 0, true, false);
    }
    for (p, n) in [(2usize, 0u8), (4, 1)] {
        let (u, v) = pad(p);
        for (j, kd) in [kind::AA, kind::AA, kind::TANK, kind::TANK, kind::BUNKER].into_iter().enumerate() {
            let a = j as f32 * 1.26 + n as f32;
            push(&mut units, kd, ENEMY, route::NONE, (u + (12.0 * a.cos()) as i32, v + (12.0 * a.sin()) as i32), 0, 0, true, true);
        }
        if p == 4 {
            push(&mut units, kind::DEPOT, ENEMY, route::NONE, (u + 20, v + 8), 0, 0, true, true);
        }
    }
    for (j, &(mu, mv, mr, _)) in crate::goap::mountains(w).iter().step_by(9).take(3).enumerate() {
        // A SAM site in the lee of a mountain: radar, two launchers, two AA guns.
        let (u, v) = ((mu + mr + 20.0) as i32, (mv - 10.0 * j as f64) as i32);
        for (d, kd) in [(0, kind::RADAR), (8, kind::SAM), (-8, kind::SAM), (14, kind::AA), (-14, kind::AA)] {
            push(&mut units, kd, ENEMY, route::NONE, (u + d, v + d / 2), 0, 0, !water(w, u + d, v), true);
        }
    }
    for k in 0..4u8 {
        push(&mut units, if k == 3 { kind::MISSILE_BOAT } else { kind::GUNBOAT }, ENEMY, route::WEST_SEA, (0, 0), k * 60, 0, true, false);
    }
    for k in 0..4u8 {
        push(&mut units, kind::GUNSHIP, ENEMY, route::ENEMY_AIR, (0, 0), k * 64, 20, true, false);
    }
    // Allies: home's defence, convoys, the front, the carrier group, gunships.
    let (hu, hv) = pad(0);
    for (j, kd) in [kind::AA, kind::AA, kind::AA, kind::TANK, kind::TANK, kind::DEPOT].into_iter().enumerate() {
        push(&mut units, kd, ALLIED, route::NONE, (hu - 30 + 12 * j as i32, hv - 25), 0, 0, true, true);
    }
    for k in 0..8u8 {
        push(&mut units, if k % 4 == 3 { kind::APC } else { kind::TRUCK }, ALLIED, route::SUPPLY, (0, 0), k * 30, 0, true, false);
    }
    for k in 0..8u8 {
        push(&mut units, if k % 2 == 0 { kind::TANK } else { kind::APC }, ALLIED, route::FRONT, (0, 0), 60 + k * 6, 0, true, false);
    }
    push(&mut units, kind::CARRIER, ALLIED, route::CARRIER, (0, 0), 0, 0, true, false);
    push(&mut units, kind::DESTROYER, ALLIED, route::CARRIER, (0, 0), 20, 0, true, false);
    push(&mut units, kind::DESTROYER, ALLIED, route::CARRIER, (0, 0), 236, 0, true, false);
    for k in 0..3u8 {
        push(&mut units, kind::GUNSHIP, ALLIED, route::ALLIED_AIR, (0, 0), k * 85, 16, true, false);
    }
    // Reinforcement slots, dead until a wave: the rest, shared out.
    let mut k = 0u8;
    while units.len() < COUNT as usize {
        let side = k & 1;
        let kd = [kind::APC, kind::TANK, kind::AA, kind::TANK][(k as usize >> 1) % 4];
        let r = if side == ENEMY { route::BEACHHEAD } else { route::FRONT };
        push(&mut units, kd, side, r, (0, 0), 0, 0, false, false);
        k = k.wrapping_add(1);
    }

    let mut out = vec![0u8; ws::STATE_LEN as usize];
    let mut alive = [0u16; 2];
    let mut seed = 0x5Du8;
    for (id, un) in units.iter().enumerate() {
        let ((u, v), leg) = if un.route == route::NONE { (un.at, 0) } else { along(&rs, un.route, un.along) };
        let r = &mut out[id * RECORD as usize..(id + 1) * RECORD as usize];
        r[rec::KIND as usize] = un.kind;
        r[rec::FLAGS as usize] = un.side | if un.alive { ALIVE } else { 0 };
        r[rec::HP as usize] = stats::HP[un.kind as usize];
        r[rec::ACTION as usize] = if un.hold { ws::act::HOLD } else { ws::act::MOVE };
        let (u16_, v16) = ((u.clamp(0, 2047) as u16) << 4, (v.clamp(0, 2047) as u16) << 4);
        r[rec::U as usize..rec::U as usize + 2].copy_from_slice(&u16_.to_le_bytes());
        r[rec::V as usize..rec::V as usize + 2].copy_from_slice(&v16.to_le_bytes());
        r[rec::H as usize] = un.h;
        // Heading for its leg's waypoint (already turned to it).
        if un.route != route::NONE {
            let pts = &rs[un.route as usize].0;
            let (wu, wv) = pts[(leg as usize).min(pts.len() - 1)];
            let yaw = ws::heading((wu - u) as i16, (wv - v) as i16);
            (r[rec::YAW as usize], r[rec::COURSE as usize]) = (yaw, yaw);
        }
        r[rec::SPEED as usize] = stats::SPEED[un.kind as usize];
        r[rec::ROUTE as usize] = un.route;
        r[rec::LEG as usize] = leg;
        r[rec::TARGET as usize..rec::TARGET as usize + 2].copy_from_slice(&ws::NOBODY.to_le_bytes());
        // The reinforcement slots: free from the start (no wreck to wait out).
        if !un.alive {
            r[rec::LAST as usize..rec::LAST as usize + 2].copy_from_slice(&0u16.wrapping_sub(ws::WRECK_TICKS).to_le_bytes());
        }
        seed = seed.wrapping_mul(29).wrapping_add(17);
        r[rec::SEED as usize] = seed;
        let sector = ((u16_ >> 11) & 15) as u8 | (((v16 >> 11) & 15) as u8) << 4;
        r[rec::SECTOR as usize] = sector;
        if un.alive {
            alive[un.side as usize] += 1;
            out[(ws::SECTORS - ws::STATE) as usize + sector as usize * ws::SECTOR as usize + un.side as usize] += 1;
        }
    }
    for s in 0..256 {
        let at = (ws::SECTORS - ws::STATE) as usize + s * ws::SECTOR as usize + ws::sec::REP as usize;
        out[at..at + 4].copy_from_slice(&[0xFF; 4]);
    }
    // Routes and waypoints.
    let mut first = 0u8;
    for (k, (pts, mode)) in rs.iter().enumerate() {
        let at = (ws::ROUTES - ws::STATE) as usize + k * ws::ROUTE_ENTRY as usize;
        out[at..at + 4].copy_from_slice(&[first, pts.len() as u8, *mode, pts.len() as u8]);
        for (j, &(u, v)) in pts.iter().enumerate() {
            let w = (ws::WAYPOINTS - ws::STATE) as usize + (first as usize + j) * 4;
            // A pad's waypoint carries the pad.
            let tag = PADS.iter().position(|p| (p.u as i32, p.v as i32) == (u, v)).map_or(0, |p| (p as u16 + 1) << ws::PAD_SHIFT);
            out[w..w + 2].copy_from_slice(&(u.clamp(0, 2047) as u16 | tag).to_le_bytes());
            out[w + 2..w + 4].copy_from_slice(&(v.clamp(0, 2047) as u16).to_le_bytes());
        }
        first += pts.len() as u8;
    }
    let h = (ws::HEADER - ws::STATE) as usize;
    for side in 0..2 {
        out[h + ws::head::ALIVE as usize + 2 * side..][..2].copy_from_slice(&alive[side].to_le_bytes());
    }
    out[h + ws::head::OWNER as usize..][..PADS.len()].copy_from_slice(&OWNERS);
    out[h + ws::head::KNOBS as usize..][..5].copy_from_slice(&ws::Knobs::DEFAULT.to_bytes());
    let carrier = units.iter().position(|un| un.kind == kind::CARRIER).expect("a carrier") as u16;
    out[h + ws::head::CARRIER as usize..][..2].copy_from_slice(&carrier.to_le_bytes());
    // The front roads as far as the first pad the other side holds.
    for side in 0..2 {
        let at = (ws::ROUTES - ws::STATE) as usize + ws::FRONT_ROUTE[side] as usize * ws::ROUTE_ENTRY as usize;
        let pts = &rs[ws::FRONT_ROUTE[side] as usize].0;
        let reach = pts.iter().position(|&q| PADS.iter().position(|p| (p.u as i32, p.v as i32) == q).is_some_and(|p| OWNERS[p] != side as u8));
        out[at + 1] = reach.map_or(pts.len(), |k| k + 1) as u8;
    }
    out
}

/// Who holds each pad at the start: home, the east outpost and the field hospital the allies; the west
/// outpost and the north (HQ) the enemy.
pub const OWNERS: [u8; 5] = [ALLIED, ALLIED, ENEMY, ALLIED, ENEMY];

/// What the war looks like: units alive and lost per side, and who holds each sector.
pub struct Report {
    pub alive: [u16; 2],
    pub lost: [u16; 2],
    /// Each pad's owner.
    pub owner: [u8; 5],
    /// Per sector: 0 empty, 1 enemy, 2 allied, 3 contested.
    pub control: [u8; 256],
}

pub fn report(reu: &[u8]) -> Report {
    let h = ws::HEADER as usize;
    let w = |a: usize| u16::from_le_bytes([reu[a], reu[a + 1]]);
    let mut control = [0u8; 256];
    for (s, c) in control.iter_mut().enumerate() {
        let a = ws::SECTORS as usize + s * ws::SECTOR as usize;
        *c = (reu[a] > 0) as u8 | ((reu[a + 1] > 0) as u8) << 1;
    }
    Report {
        alive: [w(h + ws::head::ALIVE as usize), w(h + ws::head::ALIVE as usize + 2)],
        lost: [w(h + ws::head::LOST as usize), w(h + ws::head::LOST as usize + 2)],
        owner: core::array::from_fn(|p| reu[h + ws::head::OWNER as usize + p]),
        control,
    }
}

/// Set the director's knobs in a baked war (`reu`, the whole REU image).
pub fn set_knobs(reu: &mut [u8], k: ws::Knobs) {
    reu[ws::HEADER as usize + ws::head::KNOBS as usize..][..5].copy_from_slice(&k.to_bytes());
}

/// Run the war for `ticks` with no player near, a page every `per_frame` ticks (as the C64 at about 15
/// frames a second): the model's REU holds the war after.
pub fn run(m: &mut crate::model::Machine, from: u32, ticks: u32, per_frame: u32) {
    let mut t = from;
    while t < from + ticks {
        ws::step_page(m, t as u16, -4000, -4000);
        t += per_frame;
    }
}
