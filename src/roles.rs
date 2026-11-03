//! The war's units' brains (host): a GOAP domain for combat, planned from every situation, baked by ID3
//! into one decision tree per unit kind (`brain` format) with **learn nodes** where two first moves are
//! about as good: a byte of odds the units of that kind teach as they fight (`brain::learn`, in the REU,
//! one set of bytes per kind, shared by all its units), as in `../impact`.
//!
//! The situation is six yes/no features (`brain::unit`): wounded, foes in the sector, outnumbered there,
//! weapon ready, armed, mobile. The moves (leaves): advance, hold, attack, retreat. The goal: no foes in
//! the sector. With no foes the plan is empty and the unit advances along its route (or holds, if it
//! cannot move).

use crate::brain::{self, unit};
use std::collections::HashMap;

/// Symbols: bit `k` is feature `k` (0 or 1).
const N: usize = unit::SYMBOLS;
const WOUNDED: u8 = 1 << unit::WOUNDED;
const FOES: u8 = 1 << unit::FOES;
const OUTNUMBERED: u8 = 1 << unit::OUTNUMBERED;
const READY: u8 = 1 << unit::READY;
const ARMED: u8 = 1 << unit::ARMED;
const MOBILE: u8 = 1 << unit::MOBILE;

/// An operator: needs these bits set and those clear; clears and sets these; costs.
pub struct Op {
    pub name: &'static str,
    pub action: u8,
    pub need: u8,
    pub not: u8,
    pub clear: u8,
    pub set: u8,
    pub cost: u8,
}

pub const OPS: [Op; 6] = [
    Op { name: "attack", action: unit::ATTACK, need: FOES | READY | ARMED, not: OUTNUMBERED | WOUNDED, clear: FOES, set: 0, cost: 1 },
    Op { name: "attack-odds", action: unit::ATTACK, need: FOES | READY | ARMED | OUTNUMBERED, not: 0, clear: FOES, set: 0, cost: 3 },
    Op { name: "attack-hurt", action: unit::ATTACK, need: FOES | READY | ARMED | WOUNDED, not: OUTNUMBERED, clear: FOES, set: 0, cost: 3 },
    Op { name: "retreat", action: unit::RETREAT, need: FOES | MOBILE, not: 0, clear: FOES, set: 0, cost: 2 },
    Op { name: "reload", action: unit::HOLD, need: FOES, not: READY, clear: 0, set: READY, cost: 1 },
    Op { name: "dig-in", action: unit::HOLD, need: FOES, not: MOBILE, clear: FOES, set: 0, cost: 3 },
];

fn applies(op: &Op, s: u8) -> bool {
    s & op.need == op.need && s & op.not == 0
}

/// The cheapest plan's cost from `s` to the goal (no foes): a small uniform-cost search (states are
/// bytes, plans short).
pub fn cost(s: u8) -> Option<u32> {
    let mut best: HashMap<u8, u32> = HashMap::new();
    let mut frontier = vec![(0u32, s)];
    while let Some(i) = (0..frontier.len()).min_by_key(|&i| frontier[i].0) {
        let (c, st) = frontier.swap_remove(i);
        if st & FOES == 0 {
            return Some(c);
        }
        if best.get(&st).is_some_and(|&b| b <= c) {
            continue;
        }
        best.insert(st, c);
        for op in OPS.iter().filter(|op| applies(op, st)) {
            frontier.push((c + op.cost as u32, (st & !op.clear) | op.set));
        }
    }
    None
}

/// What a unit in situation `s` should do: one move, or a learned choice between the two best first moves
/// (within `MARGIN` of each other), with the prior odds of the first (of 256) from their plan costs.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Decision {
    Do(u8),
    Choose { yes: u8, no: u8, p: u8 },
}

pub const MARGIN: u32 = 1;

pub fn decide(s: u8) -> Decision {
    if s & FOES == 0 {
        return Decision::Do(if s & MOBILE != 0 { unit::ADVANCE } else { unit::HOLD });
    }
    // Each first move's best total: its cost plus the cheapest plan after it.
    let mut by_action: Vec<(u32, u8)> = Vec::new();
    for op in OPS.iter().filter(|op| applies(op, s)) {
        let Some(rest) = cost((s & !op.clear) | op.set) else { continue };
        let c = op.cost as u32 + rest;
        match by_action.iter_mut().find(|(_, a)| *a == op.action) {
            Some(e) => e.0 = e.0.min(c),
            None => by_action.push((c, op.action)),
        }
    }
    by_action.sort();
    match by_action.as_slice() {
        [] => Decision::Do(unit::HOLD),
        [(_, a)] => Decision::Do(*a),
        [(c1, a1), (c2, a2), ..] if c2 - c1 <= MARGIN => {
            let p = (256 * *c2 / (c1 + c2)).clamp(brain::LEARN_MIN as u32, brain::LEARN_MAX as u32) as u8;
            Decision::Choose { yes: *a1, no: *a2, p }
        }
        [(_, a), ..] => Decision::Do(*a),
    }
}

/// The decisions for every situation.
pub fn table() -> Vec<Decision> {
    (0..1u16 << N).map(|s| decide(s as u8)).collect()
}

/// A decision tree over the situations' bits.
#[derive(Clone, Debug)]
pub enum Tree {
    Leaf(Decision),
    If(usize, Box<Tree>, Box<Tree>),
}

impl Tree {
    pub fn nodes(&self) -> usize {
        match self {
            Tree::Leaf(Decision::Do(_)) => 1,
            Tree::Leaf(Decision::Choose { .. }) => 3,
            Tree::If(_, t, f) => 1 + t.nodes() + f.nodes(),
        }
    }

    /// The decision for situation `s`.
    pub fn eval(&self, s: u8) -> Decision {
        match self {
            Tree::Leaf(d) => *d,
            Tree::If(k, t, f) => if s >> k & 1 != 0 { t.eval(s) } else { f.eval(s) },
        }
    }
}

fn entropy(rows: &[u8], table: &[Decision]) -> f64 {
    let mut count: HashMap<Decision, usize> = HashMap::new();
    for &r in rows {
        *count.entry(table[r as usize]).or_default() += 1;
    }
    let n = rows.len() as f64;
    count.values().map(|&c| { let p = c as f64 / n; -p * p.log2() }).sum()
}

fn id3(rows: &[u8], table: &[Decision], free: u8) -> Tree {
    let first = table[rows[0] as usize];
    if rows.iter().all(|&r| table[r as usize] == first) {
        return Tree::Leaf(first);
    }
    let mut best: Option<(f64, usize)> = None;
    for k in (0..N).filter(|k| free >> k & 1 != 0) {
        let (t, f): (Vec<u8>, Vec<u8>) = rows.iter().partition(|&&r| r >> k & 1 != 0);
        if t.is_empty() || f.is_empty() {
            continue;
        }
        let e = (t.len() as f64 * entropy(&t, table) + f.len() as f64 * entropy(&f, table)) / rows.len() as f64;
        if best.is_none_or(|(b, _)| e < b - 1e-12) {
            best = Some((e, k));
        }
    }
    let (_, k) = best.expect("rows differ, so some symbol splits them");
    let (t, f): (Vec<u8>, Vec<u8>) = rows.iter().partition(|&&r| r >> k & 1 != 0);
    let free = free & !(1 << k);
    Tree::If(k, Box::new(id3(&t, table, free)), Box::new(id3(&f, table, free)))
}

/// The combat tree, exact on [`table`].
pub fn tree() -> Tree {
    let rows: Vec<u8> = (0..1u16 << N).map(|s| s as u8).collect();
    id3(&rows, &table(), ((1u16 << N) - 1) as u8)
}

/// Postorder emission; plain nodes shared within and across trees, learn nodes never (each is its own
/// byte of learned odds).
struct Emitter {
    out: Vec<u8>,
    written: HashMap<[u8; 6], u16>,
}

impl Emitter {
    fn push(&mut self, node: [u8; 6], share: bool) -> u16 {
        if share && let Some(&at) = self.written.get(&node) {
            return at;
        }
        let at = self.out.len();
        assert!(at + brain::NODE_BYTES <= brain::UNIT_BYTES, "the units' trees exceed brain::UNIT_BYTES");
        if share {
            self.written.insert(node, at as u16);
        }
        self.out.extend_from_slice(&node);
        at as u16
    }

    fn node(&mut self, tree: &Tree, depth: u8) -> u16 {
        match tree {
            Tree::Leaf(Decision::Do(a)) => self.push([brain::LEAF, *a, 0, 0, 0, 0], true),
            Tree::Leaf(Decision::Choose { yes, no, p }) => {
                let (y, n) = (self.push([brain::LEAF, *yes, 0, 0, 0, 0], true), self.push([brain::LEAF, *no, 0, 0, 0, 0], true));
                // `lt` (a random byte under the odds) is the yes branch.
                let (ge, lt) = (n.to_le_bytes(), y.to_le_bytes());
                self.push([unit::LEARN0 + depth.min(2), *p, ge[0], ge[1], lt[0], lt[1]], false)
            }
            Tree::If(k, t, f) => {
                let (yt, yf) = (self.node(t, depth), self.node(f, depth));
                let (ge, lt) = (yt.to_le_bytes(), yf.to_le_bytes());
                self.push([*k as u8, 1, ge[0], ge[1], lt[0], lt[1]], true)
            }
        }
    }
}

/// The units' blob (`format::TREES`): one tree per kind (`actors::kind`), each with its own learn nodes;
/// the kinds' learning baselines start at 0.
pub fn compile() -> Vec<u8> {
    let t = tree();
    let mut e = Emitter { out: vec![0; brain::HEADER_BYTES], written: HashMap::new() };
    let roots: Vec<u16> = (0..crate::actors::kind::COUNT).map(|_| e.node(&t, 0)).collect();
    let mut out = e.out;
    out[..4].copy_from_slice(&brain::MAGIC);
    let len = out.len() as u16;
    out[4..6].copy_from_slice(&len.to_le_bytes());
    for (k, r) in roots.iter().enumerate() {
        out[brain::ROOTS + 2 * k..][..2].copy_from_slice(&r.to_le_bytes());
    }
    out
}
