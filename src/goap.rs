//! Goal-oriented action planning, on the host, baked into a decision tree for the C64 (`brain`).
//!
//! The agent's view of the world is a set of boolean [`SYMBOLS`], each a threshold test on one of its
//! u8 features (`agent::feature`). The actions ([`OPS`]) have preconditions, effects and costs over those
//! symbols. [`plan`] searches (A*) for the cheapest action sequence from a state to the [`GOAL`]. The
//! compiler plans from every one of the 2^[`N`] states, keeps each plan's first action, and builds a
//! decision tree (ID3) that gives exactly that table. The tree is emitted in impact's node format
//! (postorder, children before parents, identical nodes shared) and walked by `brain::walk`, so the
//! C64 replays the planner's choices with a few comparisons and no search.
//!
//! The same machinery will drive the enemies: more symbols, operators and goals, more trees.

use std::collections::HashMap;

use crate::agent::{action, feature};
use crate::brain;

/// A symbol: true when `features[feature] >= threshold` (`ge`), or when it is below (`!ge`).
#[derive(Clone, Copy, Debug)]
pub struct Symbol {
    pub name: &'static str,
    pub feature: u8,
    pub threshold: u8,
    pub ge: bool,
}

const fn sym(name: &'static str, feature: usize, threshold: u8, ge: bool) -> Symbol {
    Symbol { name, feature: feature as u8, threshold, ge }
}

pub const N: usize = 17;
/// Bits of a state: the symbols, then the planning-only bits (never in a real state: a goal's).
pub mod s {
    pub const GROUNDED: u32 = 1 << 0;
    pub const ROTOR: u32 = 1 << 1;
    pub const LOW: u32 = 1 << 2;
    pub const HIGH: u32 = 1 << 3;
    pub const BLOCKED: u32 = 1 << 4;
    pub const BLOCKED_FAR: u32 = 1 << 5;
    pub const BLOCKED_L: u32 = 1 << 6;
    pub const BLOCKED_R: u32 = 1 << 7;
    /// Something within 12 texels below: no descending yet.
    pub const OVER: u32 = 1 << 8;
    pub const WP_L: u32 = 1 << 9;
    pub const WP_R: u32 = 1 << 10;
    pub const ARRIVED: u32 = 1 << 11;
    /// The fight: a prey (the waypoint is it), the seeker locked with a missile left, hurt (the waypoint
    /// is home), and the waypoint near (the gun's range; over home, time to slow).
    pub const HUNT: u32 = 1 << 12;
    pub const LOCKED: u32 = 1 << 13;
    pub const HURT: u32 = 1 << 14;
    pub const CLOSE: u32 = 1 << 15;
    /// The waypoint inside the turning circle on its side: no turn brings it ahead.
    pub const INSIDE: u32 = 1 << 16;
    /// Planning only: the prey engaged (a burst or a missile on its way).
    pub const ENGAGED: u32 = 1 << 17;
}
/// Planning states: the symbols and the planning-only bits.
const STATES: usize = 1 << (N + 1);

pub const SYMBOLS: [Symbol; N] = [
    sym("grounded", feature::GROUND, 1, true),
    sym("rotor", feature::ROTOR, 1, true),
    sym("low", feature::ALT, 126, false),
    sym("high", feature::ALT, 131, true),
    sym("blocked", feature::HAZ_NEAR, 128, true),
    sym("blocked-far", feature::HAZ_FAR, 128, true),
    sym("blocked-l", feature::HAZ_LEFT, 128, true),
    sym("blocked-r", feature::HAZ_RIGHT, 128, true),
    sym("over", feature::HAZ_MAX, 116, true),
    sym("wp-l", feature::BEARING, 96, false),
    sym("wp-r", feature::BEARING, 160, true),
    sym("arrived", feature::DIST, 4, false),
    sym("hunt", feature::HUNT, 1, true),
    sym("locked", feature::LOCK, 1, true),
    sym("hurt", feature::HURT, 1, true),
    sym("close", feature::DIST, 10, false),
    sym("inside", feature::INSIDE, 1, true),
];

/// An operator: applicable when every `need` bit is set and every `not` bit clear; sets `set`, clears
/// `clear`; flown as `leaf`.
#[derive(Clone, Copy, Debug)]
pub struct Op {
    pub name: &'static str,
    pub leaf: u8,
    pub need: u32,
    pub not: u32,
    pub set: u32,
    pub clear: u32,
    pub cost: u32,
}

const AIR: u32 = s::GROUNDED;
const ALL_BLOCKS: u32 = s::BLOCKED | s::BLOCKED_FAR | s::BLOCKED_L | s::BLOCKED_R;
/// Not dead ahead: the waypoint (prey, home) off to a side.
const ASIDE: u32 = s::WP_L | s::WP_R;

/// The helicopter's operators, in tie-break order.
pub const OPS: [Op; 21] = [
    Op { name: "wait", leaf: action::IDLE, need: s::GROUNDED, not: s::ROTOR, set: s::ROTOR, clear: 0, cost: 1 },
    Op { name: "lift", leaf: action::LIFT, need: s::GROUNDED | s::ROTOR, not: 0, set: s::LOW, clear: s::GROUNDED, cost: 1 },
    Op { name: "climb-up", leaf: action::CLIMB, need: s::LOW, not: AIR, set: 0, clear: s::LOW, cost: 1 },
    // The fight: a missile at the lock, or the gun on the run in; or round again for another pass.
    Op { name: "launch", leaf: action::LAUNCH, need: s::HUNT | s::LOCKED, not: AIR | s::LOW, set: s::ENGAGED, clear: 0, cost: 1 },
    Op { name: "strafe", leaf: action::STRAFE, need: s::HUNT | s::CLOSE, not: AIR | s::LOW | s::HIGH | ALL_BLOCKS | ASIDE, set: s::ENGAGED, clear: 0, cost: 2 },
    Op { name: "run-in", leaf: action::FORWARD, need: s::HUNT, not: AIR | s::LOW | s::HIGH | ALL_BLOCKS | ASIDE | s::CLOSE, set: s::CLOSE, clear: 0, cost: 2 },
    // The waypoint inside the turning circle (the prey just overflown, home just off a side): on, wings
    // level, out of the circle; then round onto it.
    Op { name: "extend", leaf: action::FORWARD, need: s::INSIDE, not: AIR | s::LOW | s::BLOCKED, set: 0, clear: s::INSIDE | s::CLOSE | s::ARRIVED, cost: 2 },
    // Home: slow over it (only what is dead ahead matters), then straight down onto it (a hill off to
    // a side, seen by the side probes, does not).
    Op { name: "slow", leaf: action::SLOW, need: s::HURT | s::CLOSE, not: AIR | s::ARRIVED | s::BLOCKED | ASIDE, set: s::ARRIVED, clear: 0, cost: 1 },
    Op { name: "land", leaf: action::LAND, need: s::HURT | s::ARRIVED, not: AIR | s::HIGH, set: s::GROUNDED, clear: s::LOW, cost: 1 },
    // Hurt and landed away from home: up again.
    Op { name: "relift", leaf: action::LIFT, need: s::GROUNDED | s::ROTOR | s::HURT, not: s::ARRIVED, set: s::LOW, clear: s::GROUNDED, cost: 1 },
    // Steer round: away from the obstacle, past it, back toward the waypoint.
    Op { name: "evade-l", leaf: action::EVADE_L, need: 0, not: AIR | s::LOW | s::BLOCKED_L, set: s::WP_R | s::BLOCKED_R, clear: s::BLOCKED | s::BLOCKED_FAR | s::WP_L, cost: 2 },
    Op { name: "evade-r", leaf: action::EVADE_R, need: 0, not: AIR | s::LOW | s::BLOCKED_R, set: s::WP_L | s::BLOCKED_L, clear: s::BLOCKED | s::BLOCKED_FAR | s::WP_R, cost: 2 },
    Op { name: "pass", leaf: action::FORWARD, need: 0, not: AIR | s::LOW | s::BLOCKED, set: 0, clear: s::BLOCKED_L | s::BLOCKED_R | s::BLOCKED_FAR, cost: 2 },
    // Or climb over it, and back down once clear.
    Op { name: "climb-over", leaf: action::CLIMB, need: 0, not: AIR | s::LOW, set: s::HIGH | s::OVER, clear: ALL_BLOCKS, cost: 6 },
    Op { name: "descend", leaf: action::DESCEND, need: s::HIGH, not: AIR | ALL_BLOCKS | s::OVER, set: 0, clear: s::HIGH, cost: 1 },
    // A turn brings the waypoint ahead only from outside the turning circle.
    Op { name: "turn-l", leaf: action::TURN_L, need: s::WP_L, not: AIR | s::LOW | s::BLOCKED_L | s::INSIDE, set: 0, clear: s::WP_L, cost: 1 },
    Op { name: "turn-r", leaf: action::TURN_R, need: s::WP_R, not: AIR | s::LOW | s::BLOCKED_R | s::INSIDE, set: 0, clear: s::WP_R, cost: 1 },
    Op { name: "fly-to", leaf: action::FORWARD, need: 0, not: AIR | s::LOW | s::HIGH | ALL_BLOCKS | ASIDE, set: s::ARRIVED, clear: 0, cost: 2 },
    Op { name: "fly-to-high", leaf: action::FORWARD, need: s::HIGH, not: AIR | ALL_BLOCKS | ASIDE, set: s::ARRIVED, clear: 0, cost: 4 },
    // Should the waypoint be behind a blocked side with the way ahead clear: hold on until it is not.
    Op { name: "hold", leaf: action::FORWARD, need: 0, not: AIR | s::LOW | s::BLOCKED, set: 0, clear: s::BLOCKED_L | s::BLOCKED_R, cost: 3 },
    // Over a hold-up below (no landing on it): on past it.
    Op { name: "overfly", leaf: action::FORWARD, need: s::OVER, not: AIR | s::HIGH, set: 0, clear: s::OVER | s::ARRIVED, cost: 4 },
];

/// The close calls the pilot learns in flight: operator pairs (by name) whose plans cost within
/// [`MARGIN`] of each other. A state where the first is the best and the second that good gets a learn
/// node between them (`brain`: the first under its odds, the second otherwise), one per pair, shared by
/// every such state.
pub const LEARN_PAIRS: [(&str, &str); 4] = [("launch", "strafe"), ("strafe", "launch"), ("evade-l", "evade-r"), ("evade-r", "evade-l")];
pub const MARGIN: u32 = 1;
/// A learn node's table entry: this plus the pair's index (the table's other entries are actions).
pub const LEARN: u8 = 0x80;

/// The goal: home, landed (when hurt); the prey engaged (when hunting); else at the waypoint, in the air
/// at cruise height or above.
pub fn goal(state: u32) -> bool {
    if state & s::HURT != 0 {
        state & (s::GROUNDED | s::ARRIVED) == s::GROUNDED | s::ARRIVED
    } else if state & s::HUNT != 0 {
        state & s::ENGAGED != 0
    } else {
        state & s::ARRIVED != 0 && state & (s::GROUNDED | s::LOW) == 0
    }
}

fn applies(op: &Op, state: u32) -> bool {
    state & op.need == op.need && state & op.not == 0
}

fn next(op: &Op, state: u32) -> u32 {
    (state | op.set) & !op.clear
}

/// Every planning state's cheapest cost to the goal (`u32::MAX`: none), relaxed to a fixed point.
pub fn costs() -> &'static [u32] {
    static C: std::sync::OnceLock<Vec<u32>> = std::sync::OnceLock::new();
    C.get_or_init(|| {
        let mut c: Vec<u32> = (0..STATES as u32).map(|st| if goal(st) { 0 } else { u32::MAX }).collect();
        loop {
            let mut changed = false;
            for st in 0..STATES as u32 {
                if c[st as usize] == 0 {
                    continue;
                }
                for op in OPS.iter().filter(|op| applies(op, st)) {
                    let n = c[next(op, st) as usize];
                    if n != u32::MAX && n + op.cost < c[st as usize] {
                        c[st as usize] = n + op.cost;
                        changed = true;
                    }
                }
            }
            if !changed {
                return c;
            }
        }
    })
}

/// Each applicable operator with a plan after it: (operator, the whole plan's cost).
fn options(state: u32) -> Vec<(usize, u32)> {
    let c = costs();
    OPS.iter()
        .enumerate()
        .filter(|(_, op)| applies(op, state) && c[next(op, state) as usize] != u32::MAX)
        .map(|(k, op)| (k, op.cost + c[next(op, state) as usize]))
        .collect()
}

/// The cheapest operator sequence from `start` to the goal (ties go to the earlier operator), or `None`
/// if there is none.
pub fn plan(start: u32) -> Option<Vec<usize>> {
    let mut st = start;
    let mut ops = Vec::new();
    while !goal(st) {
        let &(k, _) = options(st).iter().min_by_key(|&&(k, c)| (c, k))?;
        ops.push(k);
        st = next(&OPS[k], st);
    }
    Some(ops)
}

fn op_index(name: &str) -> usize {
    OPS.iter().position(|o| o.name == name).expect("an operator")
}

/// The action for every symbol state: at the goal [`action::NEXT_WP`] ([`action::IDLE`] when hurt:
/// landed home, rearming); a learn node ([`LEARN`] + its pair) where a learned pair's plans tie within
/// [`MARGIN`]; else the plan's first action ([`action::FORWARD`] where there is no plan).
pub fn table() -> Vec<u8> {
    (0..1u32 << N)
        .map(|st| {
            if goal(st) {
                return if st & s::HURT != 0 { action::IDLE } else { action::NEXT_WP };
            }
            let opts = options(st);
            let Some(&(best, cost)) = opts.iter().min_by_key(|&&(k, c)| (c, k)) else { return action::FORWARD };
            let near = |k: usize| opts.iter().any(|&(j, c)| j == k && c <= cost + MARGIN);
            for (i, &(a, b)) in LEARN_PAIRS.iter().enumerate() {
                let (a, b) = (op_index(a), op_index(b));
                if a == best && near(b) && OPS[a].leaf != OPS[b].leaf {
                    return LEARN + i as u8;
                }
            }
            OPS[best].leaf
        })
        .collect()
}

/// A learn entry's two actions: under the node's odds, then otherwise.
pub fn learn_pair(entry: u8) -> (u8, u8) {
    let (a, b) = LEARN_PAIRS[(entry - LEARN) as usize];
    (OPS[op_index(a)].leaf, OPS[op_index(b)].leaf)
}

/// The symbol state of a feature block.
pub fn state_of(features: &[u8]) -> u32 {
    SYMBOLS.iter().enumerate().fold(0, |st, (k, sy)| {
        let ge = features[sy.feature as usize] >= sy.threshold;
        st | ((ge == sy.ge) as u32) << k
    })
}

/// A decision tree over the symbols.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub enum Tree {
    Leaf(u8),
    /// Symbol, subtree where it is true, where it is false.
    If(usize, Box<Tree>, Box<Tree>),
}

impl Tree {
    pub fn eval(&self, state: u32) -> u8 {
        match self {
            Tree::Leaf(a) => *a,
            Tree::If(k, t, f) => if state >> k & 1 != 0 { t.eval(state) } else { f.eval(state) },
        }
    }

    pub fn nodes(&self) -> usize {
        match self {
            Tree::Leaf(_) => 1,
            Tree::If(_, t, f) => 1 + t.nodes() + f.nodes(),
        }
    }

    pub fn depth(&self) -> usize {
        match self {
            Tree::Leaf(_) => 1,
            Tree::If(_, t, f) => 1 + t.depth().max(f.depth()),
        }
    }
}

fn entropy(rows: &[u32], table: &[u8]) -> f64 {
    let mut count = [0usize; 256];
    for &r in rows {
        count[table[r as usize] as usize] += 1;
    }
    let n = rows.len() as f64;
    count.iter().filter(|&&c| c > 0).map(|&c| { let p = c as f64 / n; -p * p.log2() }).sum()
}

/// ID3: split on the symbol with the most information gain until every leaf is pure. Exact on the
/// table (every state is a row).
fn id3(rows: &[u32], table: &[u8], free: u32) -> Tree {
    let first = table[rows[0] as usize];
    if rows.iter().all(|&r| table[r as usize] == first) {
        return Tree::Leaf(first);
    }
    let mut best: Option<(f64, usize)> = None;
    for k in (0..N).filter(|k| free >> k & 1 != 0) {
        let (t, f): (Vec<u32>, Vec<u32>) = rows.iter().partition(|&&r| r >> k & 1 != 0);
        if t.is_empty() || f.is_empty() {
            continue;
        }
        let n = rows.len() as f64;
        let e = (t.len() as f64 * entropy(&t, table) + f.len() as f64 * entropy(&f, table)) / n;
        if best.is_none_or(|(b, _)| e < b - 1e-12) {
            best = Some((e, k));
        }
    }
    let (_, k) = best.expect("rows differ, so some symbol splits them");
    let (t, f): (Vec<u32>, Vec<u32>) = rows.iter().partition(|&&r| r >> k & 1 != 0);
    let free = free & !(1 << k);
    Tree::If(k, Box::new(id3(&t, table, free)), Box::new(id3(&f, table, free)))
}

/// The pilot's tree, exact on [`table`].
pub fn pilot_tree(table: &[u8]) -> Tree {
    let rows: Vec<u32> = (0..1u32 << N).collect();
    id3(&rows, table, (1u32 << N) - 1)
}

/// Postorder emission (impact's `Emitter`): children first, identical nodes once.
struct Emitter {
    out: Vec<u8>,
    written: HashMap<[u8; 6], u16>,
}

impl Emitter {
    fn push(&mut self, node: [u8; 6]) -> u16 {
        if let Some(&at) = self.written.get(&node) {
            return at;
        }
        let at = self.out.len();
        assert!(at + brain::NODE_BYTES <= brain::MAX_BYTES, "the trees exceed brain::MAX_BYTES");
        self.written.insert(node, at as u16);
        self.out.extend_from_slice(&node);
        at as u16
    }

    fn node(&mut self, tree: &Tree) -> u16 {
        match tree {
            Tree::Leaf(a) if *a >= LEARN => {
                // A learn node: a fresh random byte under its odds (from even) takes the first action.
                let (yes, no) = learn_pair(*a);
                let y = self.push([brain::LEAF, yes, 0, 0, 0, 0]).to_le_bytes();
                let n = self.push([brain::LEAF, no, 0, 0, 0, 0]).to_le_bytes();
                self.push([feature::RAND as u8, 128, n[0], n[1], y[0], y[1]])
            }
            Tree::Leaf(a) => self.push([brain::LEAF, *a, 0, 0, 0, 0]),
            Tree::If(k, t, f) => {
                let (yt, yf) = (self.node(t), self.node(f));
                let sy = SYMBOLS[*k];
                let (ge, lt) = if sy.ge { (yt, yf) } else { (yf, yt) };
                let (ge, lt) = (ge.to_le_bytes(), lt.to_le_bytes());
                self.push([sy.feature, sy.threshold, ge[0], ge[1], lt[0], lt[1]])
            }
        }
    }
}

/// The brain blob (`brain` format) with every tree.
pub fn compile() -> Vec<u8> {
    let trees = [pilot_tree(&table())];
    let mut e = Emitter { out: vec![0; brain::HEADER_BYTES], written: HashMap::new() };
    let roots: Vec<u16> = trees.iter().map(|t| e.node(t)).collect();
    let mut out = e.out;
    out[..4].copy_from_slice(&brain::MAGIC);
    let len = out.len() as u16;
    out[4..6].copy_from_slice(&len.to_le_bytes());
    for (k, r) in roots.iter().enumerate() {
        out[brain::ROOTS + 2 * k..][..2].copy_from_slice(&r.to_le_bytes());
    }
    out
}

/// Every mountain: centre (texels), base radius and peak height above the ground (texels).
pub fn mountains(world: &crate::world::World) -> Vec<(f64, f64, f64, f64)> {
    use crate::scene::mesh::{MOUNTAIN, MOUNTAINS};
    crate::scene::objects(world)
        .iter()
        .filter(|o| (MOUNTAIN..MOUNTAIN + MOUNTAINS).contains(&o.mesh))
        .map(|o| {
            let (r, h) = crate::scene::MOUNTAIN_SHAPES[(o.mesh - MOUNTAIN) as usize];
            (o.u as f64, o.v as f64, r as f64, h as f64 + o.y as f64)
        })
        .collect()
}

/// The hazard map's margins: a mountain counts this much wider (texels), and this much taller.
pub const SIDE_MARGIN: f64 = 16.0;
pub const TOP_MARGIN: f64 = 6.0;

/// The hazard map (`format::HAZARD`): per cell, the highest point of any mountain's cone (widened and
/// raised by the margins) over the cell; bit 7 where the cell's centre is sea.
pub fn hazard_map(world: &crate::world::World) -> Vec<u8> {
    use crate::format::{HAZARD_CELLS, HAZARD_SHIFT};
    let n = HAZARD_CELLS as usize;
    let size = 1usize << HAZARD_SHIFT;
    let hills = mountains(world);
    let texels = crate::floor::MAP_TEXELS as usize;
    let mut out = vec![0u8; n * n];
    for cv in 0..n {
        for cu in 0..n {
            let (u0, v0) = ((cu * size) as f64, (cv * size) as f64);
            let mut top = 0.0f64;
            for &(u, v, r, h) in &hills {
                let du = (u0 - u).max(0.0).max(u - (u0 + size as f64));
                let dv = (v0 - v).max(0.0).max(v - (v0 + size as f64));
                let d = du.hypot(dv);
                let reach = r + SIDE_MARGIN;
                if d < reach {
                    top = top.max(h * (1.0 - d / reach) + TOP_MARGIN);
                }
            }
            let (mu, mv) = (cu * size + size / 2, cv * size + size / 2);
            let sea = world.texels[mv * texels + mu] == crate::world::texel::WATER;
            out[cv * n + cu] = (top.ceil() as u8).min(127) | (sea as u8) << 7;
        }
    }
    out
}
