//! Side-chain key routing (T080): `gatesc`/`dynsc` `src` + `tap` of every gate/dynamics slot, and key
//! listen (`$solo`).
//!
//! # Model
//!
//! Every strip (ch, aux, bus, main, mtx) is a potential key **source**: it offers its signal at a set of
//! key taps ([`slot`]). Every gate/dynamics slot with `src` != SELF is a key **consumer** that names one
//! (source, tap). The routing lives in a shared [`KeyBank`] instead of graph edges: the graph is built
//! once, while the user re-points keys at any time (WING-Edit sends `bus/7/dynsc/src = BUS.5` live), and
//! keys can form cycles (a channel keyed by a bus it feeds, two channels keying each other).
//!
//! * **Latency: one block.** A consumer reads what its source published in the *previous* cycle, whatever
//!   the scheduling order of the two strips, so every external key is exactly one engine block (64
//!   samples = 1.33 ms at 48 kHz by default) late relative to the programme it acts on. This keeps the
//!   graph static and makes every source/consumer pair legal, including cycles; a ducker/compressor's
//!   attack is not meaningfully affected. SELF keys are not routed through the bank (no latency).
//! * **Demand driven.** A source writes a tap only while at least one consumer holds it
//!   ([`KeyBank::acquire`] / [`KeyBank::release`], reference counts changed between cycles), so unused taps
//!   cost nothing.
//! * **Double buffered.** Tap storage has two halves selected by cycle parity: writers fill half
//!   `cycle & 1`, consumers read half `(cycle - 1) & 1`. Nodes of one level run concurrently, so a source
//!   and its consumer may run at the same time; they never touch the same half. Across cycles the
//!   engine's level barrier orders everything. Samples are stored as `AtomicU32` bits (relaxed), so the
//!   bank is safe Rust; RT cost is one load/store per sample of an *active* key.
//! * **Key listen**: a strip whose `gatesc/$solo` or `dynsc/$solo` is on publishes the key its slot
//!   detects on (filtered external key, or its own filtered/unfiltered input) to [`slot::LISTEN_GATE`] /
//!   [`slot::LISTEN_DYN`] every cycle; the monitor section (which runs after every strip) reads it in the
//!   same cycle and plays it on the solo bus instead of the strip's PFL ([`super::logic::flag::KEY_GATE`]).
//!
//! # `src` / `tap` enums (wing-tree.json)
//!
//! * Channels (`ch/N/gatesc`, `ch/N/dynsc`): `src` = SELF, CH.1..CH.40; `tap` = IN, FILT, 3, 4, 5, PFL, AFL,
//!   POST (the source channel's tap points, as `ptap`).
//! * Aux, bus, main, matrix (`dynsc`): `src` = SELF, BUS.1..16, MAIN.1..4, MTX.1..8, AUX.1..8; `tap` = BUS
//!   (input), DYN (post dynamics), PFL, AFL (post fader), EQ (post EQ), INS2 (post insert 2 = PFL). On an
//!   aux source these map to IN, TAP 5, PFL, AFL, TAP 4, PFL ([I]).

use std::sync::atomic::{AtomicU16, AtomicU32, Ordering::Relaxed};
use std::sync::Arc;

use crate::routing::{Strip, STRIP_COUNT};

/// Key taps per source strip.
pub const KEY_SLOTS: usize = 10;

/// Key tap slots of a source strip.
pub mod slot {
    // input strips (ch/aux): the `ptap` points
    pub const IN: usize = 0;
    pub const FILT: usize = 1;
    pub const T3: usize = 2;
    pub const T4: usize = 3;
    pub const T5: usize = 4;
    pub const PFL: usize = 5;
    pub const AFL: usize = 6;
    pub const POST: usize = 7;
    // bus/main/mtx (= [`crate::mix::MixTap`] indices, then AFL)
    pub const MIX_IN: usize = 0;
    pub const MIX_INS1: usize = 1;
    pub const MIX_EQ: usize = 2;
    pub const MIX_DYN: usize = 3;
    pub const MIX_PFL: usize = 4;
    pub const MIX_AFL: usize = 5;
    // key listen (any strip)
    pub const LISTEN_GATE: usize = 8;
    pub const LISTEN_DYN: usize = 9;
}

/// Which `src`/`tap` enums a keyed slot uses.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum KeyFamily {
    /// `ch/N/gatesc`, `ch/N/dynsc`.
    Channel,
    /// `aux|bus|main|mtx/N/dynsc`.
    Mix,
}

impl KeyFamily {
    /// Family of a keyed strip.
    pub fn of(s: Strip) -> Self {
        match s {
            Strip::Ch(_) => KeyFamily::Channel,
            _ => KeyFamily::Mix,
        }
    }
}

/// The source strip of `src` enum index `src` (0 = SELF -> `None`).
pub fn key_source(family: KeyFamily, src: u32) -> Option<Strip> {
    let n = src as usize;
    Some(match family {
        KeyFamily::Channel => match n {
            1..=40 => Strip::Ch(n as u8),
            _ => return None,
        },
        KeyFamily::Mix => match n {
            1..=16 => Strip::Bus(n as u8),
            17..=20 => Strip::Main((n - 16) as u8),
            21..=28 => Strip::Mtx((n - 20) as u8),
            29..=36 => Strip::Aux((n - 28) as u8),
            _ => return None,
        },
    })
}

/// Resolve a slot's `src`/`tap` enum indices to (source strip index, key tap [`slot`]); `None` = SELF.
pub fn key_route(family: KeyFamily, src: u32, tap: u32) -> Option<(usize, usize)> {
    let s = key_source(family, src)?;
    let idx = s.index()?;
    let t = tap as usize;
    let k = match family {
        // IN FILT 3 4 5 PFL AFL POST on a channel = the channel slots
        KeyFamily::Channel => t.min(slot::POST),
        // BUS DYN PFL AFL EQ INS2
        KeyFamily::Mix => match s {
            Strip::Ch(_) | Strip::Aux(_) => [
                slot::IN,
                slot::T5,
                slot::PFL,
                slot::AFL,
                slot::T4,
                slot::PFL,
            ][t.min(5)],
            _ => [
                slot::MIX_IN,
                slot::MIX_DYN,
                slot::MIX_PFL,
                slot::MIX_AFL,
                slot::MIX_EQ,
                slot::MIX_PFL,
            ][t.min(5)],
        },
    };
    Some((idx, k))
}

struct Shared {
    block: usize,
    /// `[strip][slot][parity][side][block]` sample bits.
    data: Box<[AtomicU32]>,
    /// Consumers per `[strip][slot]`.
    demand: Box<[AtomicU16]>,
}

/// Shared key tap storage of every strip; see the module docs. Clone = another handle.
#[derive(Clone)]
pub struct KeyBank(Arc<Shared>);

impl KeyBank {
    /// Storage for blocks of `block` samples (allocates; control thread).
    pub fn new(block: usize) -> Self {
        let block = block.max(1);
        let n = STRIP_COUNT * KEY_SLOTS * 2 * 2 * block;
        Self(Arc::new(Shared {
            block,
            data: (0..n).map(|_| AtomicU32::new(0)).collect(),
            demand: (0..STRIP_COUNT * KEY_SLOTS)
                .map(|_| AtomicU16::new(0))
                .collect(),
        }))
    }

    pub fn block_size(&self) -> usize {
        self.0.block
    }

    #[inline]
    fn at(&self, src: usize, slot: usize, parity: u64, side: usize) -> usize {
        (((src * KEY_SLOTS + slot) * 2 + (parity & 1) as usize) * 2 + side) * self.0.block
    }

    /// Whether any consumer holds tap `slot` of strip `src`. RT-safe.
    #[inline]
    pub fn wanted(&self, src: usize, slot: usize) -> bool {
        self.0
            .demand
            .get(src * KEY_SLOTS + slot)
            .is_some_and(|d| d.load(Relaxed) != 0)
    }

    /// A consumer starts reading tap `slot` of `src` (between cycles: driver or control thread). The
    /// first consumer clears the tap so it never reads stale samples.
    pub fn acquire(&self, src: usize, slot: usize) {
        let Some(d) = self.0.demand.get(src * KEY_SLOTS + slot) else {
            return;
        };
        if d.fetch_add(1, Relaxed) == 0 {
            let s = self.at(src, slot, 0, 0);
            for v in &self.0.data[s..s + 4 * self.0.block] {
                v.store(0, Relaxed);
            }
        }
    }

    /// A consumer stops reading tap `slot` of `src` (between cycles).
    pub fn release(&self, src: usize, slot: usize) {
        if let Some(d) = self.0.demand.get(src * KEY_SLOTS + slot) {
            // saturating decrement (a CAS loop: `fetch_update` is deprecated on newer toolchains)
            let mut v = d.load(Relaxed);
            while v > 0 {
                match d.compare_exchange_weak(v, v - 1, Relaxed, Relaxed) {
                    Ok(_) => break,
                    Err(cur) => v = cur,
                }
            }
        }
    }

    /// Publish tap `slot` of strip `src` for `cycle` (only the strip's own node calls this). Samples
    /// beyond the block size are dropped. RT-safe.
    #[inline]
    pub fn write(&self, cycle: u64, src: usize, slot: usize, l: &[f32], r: &[f32]) {
        if src >= STRIP_COUNT || slot >= KEY_SLOTS {
            return;
        }
        let b = self.0.block;
        for (side, x) in [l, r].into_iter().enumerate() {
            let s = self.at(src, slot, cycle, side);
            for (d, &v) in self.0.data[s..s + b].iter().zip(x) {
                d.store(v.to_bits(), Relaxed);
            }
        }
    }

    /// Read tap `slot` of strip `src` as written in `cycle` into `l`/`r` (beyond the block size: 0).
    /// RT-safe.
    #[inline]
    pub fn read(&self, cycle: u64, src: usize, slot: usize, l: &mut [f32], r: &mut [f32]) {
        if src >= STRIP_COUNT || slot >= KEY_SLOTS {
            l.fill(0.0);
            r.fill(0.0);
            return;
        }
        let b = self.0.block;
        for (side, x) in [l, r].into_iter().enumerate() {
            let s = self.at(src, slot, cycle, side);
            let m = x.len().min(b);
            for (v, d) in x[..m].iter_mut().zip(&self.0.data[s..s + m]) {
                *v = f32::from_bits(d.load(Relaxed));
            }
            x[m..].fill(0.0);
        }
    }
}

/// One keyed slot's external key (consumer side): its route and a scratch copy of the key.
pub struct KeyIn {
    bank: KeyBank,
    want: Option<(usize, usize)>,
    held: Option<(usize, usize)>,
    l: Box<[f32]>,
    r: Box<[f32]>,
}

impl KeyIn {
    /// Allocates the key scratch (control thread).
    pub fn new(bank: KeyBank) -> Self {
        let b = bank.block_size();
        Self {
            bank,
            want: None,
            held: None,
            l: vec![0.0; b].into(),
            r: vec![0.0; b].into(),
        }
    }

    /// Desired route (from [`key_route`]); applied by [`KeyIn::sync`]. RT-safe.
    pub fn route(&mut self, r: Option<(usize, usize)>) {
        self.want = r;
    }

    /// The route currently held.
    pub fn held(&self) -> Option<(usize, usize)> {
        self.held
    }

    /// Apply the desired route (between cycles). RT-safe (atomics only).
    pub fn sync(&mut self) {
        if self.want == self.held {
            return;
        }
        if let Some((s, k)) = self.held {
            self.bank.release(s, k);
        }
        if let Some((s, k)) = self.want {
            self.bank.acquire(s, k);
        }
        self.held = self.want;
    }

    /// The key for the block of `cycle` (`n` samples): what the source published in `cycle - 1`. `None`
    /// when the slot keys from itself. RT-safe.
    #[inline]
    pub fn fetch(&mut self, cycle: u64, n: usize) -> Option<(&[f32], &[f32])> {
        let (s, k) = self.held?;
        let n = n.min(self.l.len());
        let (l, r) = (&mut self.l[..n], &mut self.r[..n]);
        self.bank.read(cycle.wrapping_sub(1), s, k, l, r);
        Some((&self.l[..n], &self.r[..n]))
    }
}

impl Drop for KeyIn {
    fn drop(&mut self) {
        if let Some((s, k)) = self.held.take() {
            self.bank.release(s, k);
        }
    }
}

/// A strip's key outputs (source side): publishes the taps consumers hold and the key-listen signal.
#[derive(Clone)]
pub struct KeyOut {
    bank: KeyBank,
    idx: usize,
    cycle: u64,
}

impl KeyOut {
    /// Outputs of strip index `idx` ([`Strip::index`]).
    pub fn new(bank: KeyBank, idx: usize) -> Self {
        Self {
            bank,
            idx,
            cycle: 0,
        }
    }

    /// Cycle of the block being processed (set by the node before processing). RT-safe.
    #[inline]
    pub fn set_cycle(&mut self, c: u64) {
        self.cycle = c;
    }

    pub fn cycle(&self) -> u64 {
        self.cycle
    }

    pub fn bank(&self) -> &KeyBank {
        &self.bank
    }

    pub fn index(&self) -> usize {
        self.idx
    }

    /// Publish tap `slot` if any consumer holds it. RT-safe.
    #[inline]
    pub fn tap(&self, slot: usize, l: &[f32], r: &[f32]) {
        if self.bank.wanted(self.idx, slot) {
            self.bank.write(self.cycle, self.idx, slot, l, r);
        }
    }

    /// Publish `slot` unconditionally (key listen). RT-safe.
    #[inline]
    pub fn put(&self, slot: usize, l: &[f32], r: &[f32]) {
        self.bank.write(self.cycle, self.idx, slot, l, r);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use wing_model::Schema;

    fn tokens(path: &str) -> Vec<&'static str> {
        let s = Schema::get();
        let d = s.base_desc(s.by_path(path).unwrap()).unwrap();
        s.enum_items(d).iter().map(|e| e.token).collect()
    }

    fn strip_token(s: Strip) -> String {
        match s {
            Strip::Ch(n) => format!("CH.{n}"),
            Strip::Aux(n) => format!("AUX.{n}"),
            Strip::Bus(n) => format!("BUS.{n}"),
            Strip::Main(n) => format!("MAIN.{n}"),
            Strip::Mtx(n) => format!("MTX.{n}"),
        }
    }

    #[test]
    fn src_enums_match_schema() {
        for (path, fam) in [
            ("ch/1/gatesc/src", KeyFamily::Channel),
            ("ch/1/dynsc/src", KeyFamily::Channel),
            ("aux/1/dynsc/src", KeyFamily::Mix),
            ("bus/1/dynsc/src", KeyFamily::Mix),
            ("main/1/dynsc/src", KeyFamily::Mix),
            ("mtx/1/dynsc/src", KeyFamily::Mix),
        ] {
            let t = tokens(path);
            assert_eq!(t[0], "SELF", "{path}");
            assert_eq!(key_source(fam, 0), None);
            for (i, tok) in t.iter().enumerate().skip(1) {
                let s = key_source(fam, i as u32).unwrap_or_else(|| panic!("{path} {tok}"));
                assert_eq!(&strip_token(s), tok, "{path} #{i}");
            }
            assert_eq!(key_source(fam, t.len() as u32), None);
        }
    }

    #[test]
    fn tap_enums_match_schema() {
        assert_eq!(
            tokens("ch/1/gatesc/tap"),
            ["IN", "FILT", "3", "4", "5", "PFL", "AFL", "POST"]
        );
        assert_eq!(tokens("ch/1/dynsc/tap"), tokens("ch/1/gatesc/tap"));
        for p in ["aux", "bus", "main", "mtx"] {
            assert_eq!(
                tokens(&format!("{p}/1/dynsc/tap")),
                ["BUS", "DYN", "PFL", "AFL", "EQ", "INS2"],
                "{p}"
            );
        }
        // bus 7 keyed from BUS.5 POST FDR (the user's session) = bus 5's AFL
        assert_eq!(key_route(KeyFamily::Mix, 5, 3), Some((52, slot::MIX_AFL)));
        assert_eq!(key_route(KeyFamily::Mix, 29, 1), Some((40, slot::T5)));
        assert_eq!(key_route(KeyFamily::Channel, 2, 6), Some((1, slot::AFL)));
        assert_eq!(key_route(KeyFamily::Channel, 0, 6), None);
    }

    #[test]
    fn bank_is_one_block_late_and_demand_driven() {
        let bank = KeyBank::new(4);
        let out = {
            let mut o = KeyOut::new(bank.clone(), 3);
            o.set_cycle(10);
            o
        };
        let mut k = KeyIn::new(bank.clone());
        // not held: nothing published
        out.tap(slot::PFL, &[1.0; 4], &[2.0; 4]);
        assert!(!bank.wanted(3, slot::PFL));
        k.route(Some((3, slot::PFL)));
        k.sync();
        assert!(bank.wanted(3, slot::PFL));
        // cycle 10's samples are the key of cycle 11
        out.tap(slot::PFL, &[1.0; 4], &[2.0; 4]);
        assert_eq!(k.fetch(10, 4).unwrap().0, &[0.0; 4]);
        assert_eq!(k.fetch(11, 4).unwrap(), (&[1.0; 4][..], &[2.0; 4][..]));
        k.route(None);
        k.sync();
        assert!(!bank.wanted(3, slot::PFL));
        assert!(k.fetch(11, 4).is_none());
        // drop releases
        k.route(Some((3, slot::IN)));
        k.sync();
        assert!(bank.wanted(3, slot::IN));
        drop(k);
        assert!(!bank.wanted(3, slot::IN));
    }
}
