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

use std::collections::{BinaryHeap, HashMap};

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

pub const N: usize = 12;
/// Bits of a state.
pub mod s {
    pub const GROUNDED: u16 = 1 << 0;
    pub const ROTOR: u16 = 1 << 1;
    pub const LOW: u16 = 1 << 2;
    pub const HIGH: u16 = 1 << 3;
    pub const BLOCKED: u16 = 1 << 4;
    pub const BLOCKED_FAR: u16 = 1 << 5;
    pub const BLOCKED_L: u16 = 1 << 6;
    pub const BLOCKED_R: u16 = 1 << 7;
    /// Something within 12 texels below: no descending yet.
    pub const OVER: u16 = 1 << 8;
    pub const WP_L: u16 = 1 << 9;
    pub const WP_R: u16 = 1 << 10;
    pub const ARRIVED: u16 = 1 << 11;
}

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
];

/// An operator: applicable when every `need` bit is set and every `not` bit clear; sets `set`, clears
/// `clear`; flown as `leaf`.
#[derive(Clone, Copy, Debug)]
pub struct Op {
    pub name: &'static str,
    pub leaf: u8,
    pub need: u16,
    pub not: u16,
    pub set: u16,
    pub clear: u16,
    pub cost: u32,
}

const AIR: u16 = s::GROUNDED;
const ALL_BLOCKS: u16 = s::BLOCKED | s::BLOCKED_FAR | s::BLOCKED_L | s::BLOCKED_R;

/// The helicopter's operators, in tie-break order.
pub const OPS: [Op; 13] = [
    Op { name: "wait", leaf: action::IDLE, need: s::GROUNDED, not: s::ROTOR, set: s::ROTOR, clear: 0, cost: 1 },
    Op { name: "lift", leaf: action::LIFT, need: s::GROUNDED | s::ROTOR, not: 0, set: s::LOW, clear: s::GROUNDED, cost: 1 },
    Op { name: "climb-up", leaf: action::CLIMB, need: s::LOW, not: AIR, set: 0, clear: s::LOW, cost: 1 },
    // Steer round: away from the obstacle, past it, back toward the waypoint.
    Op { name: "evade-l", leaf: action::EVADE_L, need: 0, not: AIR | s::LOW | s::BLOCKED_L, set: s::WP_R | s::BLOCKED_R, clear: s::BLOCKED | s::BLOCKED_FAR | s::WP_L, cost: 2 },
    Op { name: "evade-r", leaf: action::EVADE_R, need: 0, not: AIR | s::LOW | s::BLOCKED_R, set: s::WP_L | s::BLOCKED_L, clear: s::BLOCKED | s::BLOCKED_FAR | s::WP_R, cost: 2 },
    Op { name: "pass", leaf: action::FORWARD, need: 0, not: AIR | s::LOW | s::BLOCKED, set: 0, clear: s::BLOCKED_L | s::BLOCKED_R | s::BLOCKED_FAR, cost: 2 },
    // Or climb over it, and back down once clear.
    Op { name: "climb-over", leaf: action::CLIMB, need: 0, not: AIR | s::LOW, set: s::HIGH | s::OVER, clear: ALL_BLOCKS, cost: 6 },
    Op { name: "descend", leaf: action::DESCEND, need: s::HIGH, not: AIR | ALL_BLOCKS | s::OVER, set: 0, clear: s::HIGH, cost: 1 },
    Op { name: "turn-l", leaf: action::TURN_L, need: s::WP_L, not: AIR | s::LOW | s::BLOCKED_L, set: 0, clear: s::WP_L, cost: 1 },
    Op { name: "turn-r", leaf: action::TURN_R, need: s::WP_R, not: AIR | s::LOW | s::BLOCKED_R, set: 0, clear: s::WP_R, cost: 1 },
    Op { name: "fly-to", leaf: action::FORWARD, need: 0, not: AIR | s::LOW | s::HIGH | ALL_BLOCKS | s::WP_L | s::WP_R, set: s::ARRIVED, clear: 0, cost: 2 },
    Op { name: "fly-to-high", leaf: action::FORWARD, need: s::HIGH, not: AIR | ALL_BLOCKS | s::WP_L | s::WP_R, set: s::ARRIVED, clear: 0, cost: 4 },
    // Should the waypoint be behind a blocked side with the way ahead clear: hold on until it is not.
    Op { name: "hold", leaf: action::FORWARD, need: 0, not: AIR | s::LOW | s::BLOCKED, set: 0, clear: s::BLOCKED_L | s::BLOCKED_R, cost: 3 },
];

/// The goal: at the waypoint, in the air at cruise height or above.
pub fn goal(state: u16) -> bool {
    state & s::ARRIVED != 0 && state & (s::GROUNDED | s::LOW) == 0
}

fn applies(op: &Op, state: u16) -> bool {
    state & op.need == op.need && state & op.not == 0
}

/// The cheapest operator sequence from `start` to the goal (A*; ties go to the earlier operator), or
/// `None` if there is none.
pub fn plan(start: u16) -> Option<Vec<usize>> {
    // Admissible: any unmet goal costs at least one more action.
    let h = |st: u16| (!goal(st)) as u32;
    let mut best: HashMap<u16, u32> = HashMap::new();
    let mut came: HashMap<u16, (u16, usize)> = HashMap::new();
    let mut open = BinaryHeap::new();
    let mut seq = 0u32;
    open.push(std::cmp::Reverse((h(start), seq, 0u32, start)));
    best.insert(start, 0);
    while let Some(std::cmp::Reverse((_, _, g, st))) = open.pop() {
        if g > best[&st] {
            continue;
        }
        if goal(st) {
            let mut ops = Vec::new();
            let mut at = st;
            while at != start {
                let (prev, op) = came[&at];
                ops.push(op);
                at = prev;
            }
            ops.reverse();
            return Some(ops);
        }
        for (k, op) in OPS.iter().enumerate() {
            if !applies(op, st) {
                continue;
            }
            let next = (st | op.set) & !op.clear;
            let g2 = g + op.cost;
            if best.get(&next).is_none_or(|&b| g2 < b) {
                best.insert(next, g2);
                came.insert(next, (st, k));
                seq += 1;
                open.push(std::cmp::Reverse((g2 + h(next), seq, g2, next)));
            }
        }
    }
    None
}

/// The action for every state: [`action::NEXT_WP`] at the goal, the plan's first action otherwise
/// ([`action::FORWARD`] where there is no plan).
pub fn table() -> Vec<u8> {
    (0..1u32 << N)
        .map(|st| {
            let st = st as u16;
            if goal(st) {
                action::NEXT_WP
            } else {
                plan(st).and_then(|p| p.first().map(|&k| OPS[k].leaf)).unwrap_or(action::FORWARD)
            }
        })
        .collect()
}

/// The symbol state of a feature block.
pub fn state_of(features: &[u8]) -> u16 {
    SYMBOLS.iter().enumerate().fold(0, |st, (k, sy)| {
        let ge = features[sy.feature as usize] >= sy.threshold;
        st | ((ge == sy.ge) as u16) << k
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
    pub fn eval(&self, state: u16) -> u8 {
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

fn entropy(rows: &[u16], table: &[u8]) -> f64 {
    let mut count = [0usize; 256];
    for &r in rows {
        count[table[r as usize] as usize] += 1;
    }
    let n = rows.len() as f64;
    count.iter().filter(|&&c| c > 0).map(|&c| { let p = c as f64 / n; -p * p.log2() }).sum()
}

/// ID3: split on the symbol with the most information gain until every leaf is pure. Exact on the
/// table (every state is a row).
fn id3(rows: &[u16], table: &[u8], free: u16) -> Tree {
    let first = table[rows[0] as usize];
    if rows.iter().all(|&r| table[r as usize] == first) {
        return Tree::Leaf(first);
    }
    let mut best: Option<(f64, usize)> = None;
    for k in (0..N).filter(|k| free >> k & 1 != 0) {
        let (t, f): (Vec<u16>, Vec<u16>) = rows.iter().partition(|&&r| r >> k & 1 != 0);
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
    let (t, f): (Vec<u16>, Vec<u16>) = rows.iter().partition(|&&r| r >> k & 1 != 0);
    let free = free & !(1 << k);
    Tree::If(k, Box::new(id3(&t, table, free)), Box::new(id3(&f, table, free)))
}

/// The pilot's tree, exact on [`table`].
pub fn pilot_tree(table: &[u8]) -> Tree {
    let rows: Vec<u16> = (0..1u32 << N).map(|s| s as u16).collect();
    id3(&rows, table, (1 << N) - 1)
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
