//! DCA, mute-group and solo logic (T038): the control-side resolver ([`MixLogic`]) and the
//! lock-free state it publishes to the RT nodes ([`MixShared`]).
//!
//! # Semantics
//!
//! * **Membership** comes from each strip's `tags` string (signal-flow.md §9): `#Dn` = member of
//!   DCA n (1..16), `#Mn` = member of mute group n (1..8). Every strip type (ch, aux, bus, main,
//!   mtx) can be a member.
//! * **DCA level**: a member's effective fader is its own fader plus the *sum in dB* of the faders
//!   of every DCA it belongs to (X32 semantics [I]); any member DCA at -oo silences it; the total
//!   is clamped to +10 dB. Applied as [`MixShared`] external gain, smoothed by the strip's 5 ms
//!   fader ramp.
//! * **Mutes**: effective mute = own mute OR an active mute group it belongs to.
//! * **DCA mute** (T03F, confirmed by the user on a real WING): a muted DCA always mutes its
//!   members' *main sends* (ch/aux/bus -> main 1-4, whatever their `pre` tap) — the strip itself,
//!   its bus/matrix sends and its PFL/AFL solo keep running ([`flag::MAIN_MUTE`], applied by the
//!   main nodes). With `cfg/dcamgrp` "DCA MUTEGROUPS" on (factory default) the DCA mute acts as a
//!   mute group: a full strip mute ([`flag::MUTE`], incl. PRE sends). Main/matrix members have
//!   no main send and are muted outright either way [I].
//! * **Solo** (`cfg/solo`): a strip is soloed by its own `$solo`, by the `$solo` of a DCA it
//!   belongs to, or (source solo, `srcsolo` CH39/AUX7 with `$srcsolo` on) as the source-solo
//!   strip [I]. `$ctl/cfg/soloexcl` (EXCL SOLO, default on) makes a new solo clear every other one;
//!   writing `$stat/solo` = 0 while a solo is active clears every solo ("clear solo"). Taps:
//!   `chtap` (ch and aux), `bustap`, `maintap`, `mtxtap`: PFL = pre-fader, AFL = post-fader.
//!   Which monitor a soloed strip reaches: monitor 1 (PH) and/or 2 (SPK) by `cfg/solo/mon`
//!   (PH/SPK/PH+SPK); factory PH. The strip `mon` (MON BUS A/B/A+B) does *not* gate it (T03G):
//!   the earlier "strip `mon` AND `cfg/solo/mon`" inference made every solo silent with SPK, as
//!   factory strips are all `A`, yet the user's real-WING scene (`cfg/solo/mon` SPK, every strip
//!   `A`) is a working show file. What strip `mon` does is unknown [I] (not modelled).
//!   Monitor 1 is PH (phones) and monitor 2 SPK (speakers):
//!   `cfg/mon/1/lvl` is labelled "PHONES" and `cfg/mon/2/lvl` "SPEAKERS" in the schema, and
//!   WING's direct-input list names them `MON.PH` / `MON.SPK` (signal-flow.md §10) [S].
//! * **Listened monitors** (simulator only, T03G): a real WING has phones *and* speakers, so a
//!   solo routed to the speakers is still heard. The simulator's host bridge usually plays just
//!   one monitor (default `mon/1`). [`MixLogic::set_listen`] names the monitors the host plays;
//!   a solo whose WING destination (`cfg/solo/mon`) reaches neither of
//!   the listened ones is additionally sent to the listened ones, so e.g. `cfg/solo/mon` = SPK
//!   (the user's scene) is still audible on a host that plays `mon/1`. Without a listen mask
//!   (0, the engine default) the routing is exactly WING's.
//! * **Modes**: `LIVE` and `STUDIO` leave the mix untouched (solo only feeds the monitors; the
//!   difference between the two is not known [I]). `SIP` (solo in place) additionally mutes, while
//!   any channel/aux is soloed, every non-soloed channel/aux without `solosafe`, and while any bus
//!   is soloed every non-soloed bus [I]. Mains and matrices are never SIP-muted.
//! * **Indicators** (written back to the store, so WING-Edit sees them): `*/$sololed` 0 = off,
//!   1 = soloed by its own button, 2 = soloed through a DCA / source solo; `dca/*/$sololed`;
//!   `$stat/solo` = any solo active.
//!
//! Not modelled: `cfg/solo/mute`, `$flip`, `cfg/muteovr`/`$muteovr`, `$presolo` (tap solo), the
//! source-solo input selection (`$srcsgrp`/`$srcsin`), `$mute`/`$fdr` mirrors.
//!
//! # Threading
//!
//! [`MixLogic::change_hook`] (registered on the store) marks the state dirty when a watched
//! parameter changes and remembers the last strip/DCA whose solo was switched on (exclusive solo).
//! [`MixLogic::refresh`] (control thread, polled) recomputes everything, writes the indicators and
//! publishes per-strip values to [`MixShared`] (atomics + a generation counter). RT nodes compare
//! the generation once per block and re-read only on change: no locks, no allocation.

use std::sync::atomic::{AtomicBool, AtomicU32, Ordering::*};
use std::sync::Arc;

use wing_model::{Hook, NodeId as ParamId, ParamStore, Value};

use crate::routing::{Strip, STRIP_COUNT};

use super::law::FADER_OFF_DB;

/// First main strip index ([`Strip::index`]): ch, aux and bus strips come before it.
const MAIN_BASE: usize = 64;
const _: () = assert!(STRIP_COUNT == MAIN_BASE + 4 + 8);

/// Flag bits of [`MixShared::flags`].
pub mod flag {
    /// External mute (mute group, DCA mute, solo in place).
    pub const MUTE: u32 = 1;
    /// Soloed into monitor 1.
    pub const SOLO_MON1: u32 = 2;
    /// Soloed into monitor 2.
    pub const SOLO_MON2: u32 = 4;
    /// Solo tap is AFL (post fader) instead of PFL.
    pub const AFL: u32 = 8;
    /// The strip's main sends are muted (DCA mute with `cfg/dcamgrp` off, T03F); applied by the
    /// main nodes to the sends they receive.
    pub const MAIN_MUTE: u32 = 16;
}

/// Per-strip state shared with the RT nodes (indexed by [`Strip::index`]).
pub struct MixShared {
    gen: AtomicU32,
    gain_db: [AtomicU32; STRIP_COUNT],
    flags: [AtomicU32; STRIP_COUNT],
}

impl Default for MixShared {
    fn default() -> Self {
        Self {
            gen: AtomicU32::new(0),
            gain_db: std::array::from_fn(|_| AtomicU32::new(0f32.to_bits())),
            flags: std::array::from_fn(|_| AtomicU32::new(0)),
        }
    }
}

impl MixShared {
    /// Generation (changes whenever anything was published).
    #[inline]
    pub fn generation(&self) -> u32 {
        self.gen.load(Acquire)
    }

    /// External gain offset of strip `i` in dB (DCA sum; -144 = -oo).
    #[inline]
    pub fn gain_db(&self, i: usize) -> f32 {
        f32::from_bits(self.gain_db[i].load(Relaxed))
    }

    /// [`flag`] bits of strip `i`.
    #[inline]
    pub fn flags(&self, i: usize) -> u32 {
        self.flags[i].load(Relaxed)
    }

    fn publish(&self, gain: &[f32; STRIP_COUNT], flags: &[u32; STRIP_COUNT]) {
        for i in 0..STRIP_COUNT {
            self.gain_db[i].store(gain[i].to_bits(), Relaxed);
            self.flags[i].store(flags[i], Relaxed);
        }
        self.gen.fetch_add(1, Release);
    }
}

/// Solo mode (`cfg/solo/mode`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SoloMode {
    Live,
    Studio,
    InPlace,
}

/// Ids of one strip's parameters.
struct StripIds {
    tags: ParamId,
    solo: ParamId,
    sololed: ParamId,
    solosafe: Option<ParamId>,
}

struct Ids {
    strips: Vec<StripIds>,
    dca_fdr: [ParamId; 16],
    dca_mute: [ParamId; 16],
    dca_solo: [ParamId; 16],
    dca_sololed: [ParamId; 16],
    mgrp_mute: [ParamId; 8],
    dcamgrp: ParamId,
    mode: ParamId,
    solo_mon: ParamId,
    taps: [ParamId; 4],
    srcsolo: ParamId,
    srcsolo_on: ParamId,
    excl: ParamId,
    stat: ParamId,
}

fn strip_path(i: usize) -> String {
    match Strip::from_index(i).expect("strip") {
        Strip::Ch(n) => format!("ch/{n}"),
        Strip::Aux(n) => format!("aux/{n}"),
        Strip::Bus(n) => format!("bus/{n}"),
        Strip::Main(n) => format!("main/{n}"),
        Strip::Mtx(n) => format!("mtx/{n}"),
    }
}

impl Ids {
    fn new(store: &ParamStore) -> Self {
        let id = |p: &str| {
            store
                .lookup(p)
                .unwrap_or_else(|| panic!("schema has no {p}"))
        };
        let strips = (0..STRIP_COUNT)
            .map(|i| {
                let p = strip_path(i);
                StripIds {
                    tags: id(&format!("{p}/tags")),
                    solo: id(&format!("{p}/$solo")),
                    sololed: id(&format!("{p}/$sololed")),
                    solosafe: store.lookup(&format!("{p}/solosafe")),
                }
            })
            .collect();
        Self {
            strips,
            dca_fdr: std::array::from_fn(|k| id(&format!("dca/{}/fdr", k + 1))),
            dca_mute: std::array::from_fn(|k| id(&format!("dca/{}/mute", k + 1))),
            dca_solo: std::array::from_fn(|k| id(&format!("dca/{}/$solo", k + 1))),
            dca_sololed: std::array::from_fn(|k| id(&format!("dca/{}/$sololed", k + 1))),
            mgrp_mute: std::array::from_fn(|k| id(&format!("mgrp/{}/mute", k + 1))),
            dcamgrp: id("cfg/dcamgrp"),
            mode: id("cfg/solo/mode"),
            solo_mon: id("cfg/solo/mon"),
            taps: [
                id("cfg/solo/chtap"),
                id("cfg/solo/bustap"),
                id("cfg/solo/maintap"),
                id("cfg/solo/mtxtap"),
            ],
            srcsolo: id("cfg/solo/srcsolo"),
            srcsolo_on: id("cfg/solo/$srcsolo"),
            excl: id("$ctl/cfg/soloexcl"),
            stat: id("$stat/solo"),
        }
    }

    /// Every parameter whose change requires a refresh.
    fn watched(&self) -> Vec<ParamId> {
        let mut v = Vec::new();
        for s in &self.strips {
            v.extend([s.tags, s.solo]);
            v.extend(s.solosafe);
        }
        v.extend(self.dca_fdr);
        v.extend(self.dca_mute);
        v.extend(self.dca_solo);
        v.extend(self.mgrp_mute);
        v.extend(self.taps);
        v.extend([
            self.dcamgrp,
            self.mode,
            self.solo_mon,
            self.srcsolo,
            self.srcsolo_on,
            self.excl,
            self.stat,
        ]);
        v
    }

    /// Every solo button (strips then DCAs).
    fn solo_buttons(&self) -> impl Iterator<Item = ParamId> + '_ {
        self.strips
            .iter()
            .map(|s| s.solo)
            .chain(self.dca_solo.iter().copied())
    }
}

/// Parse `#Dn` / `#Mn` tags into (DCA mask bit n-1, mute group mask bit n-1).
pub fn parse_tags(tags: &str) -> (u16, u8) {
    let (mut d, mut m) = (0u16, 0u8);
    for t in tags.split(',') {
        let t = t.trim();
        let Some(rest) = t.strip_prefix('#') else {
            continue;
        };
        let (kind, num) = rest.split_at(rest.len().min(1));
        let Ok(n) = num.parse::<u32>() else {
            continue;
        };
        match kind {
            "D" | "d" if (1..=16).contains(&n) => d |= 1 << (n - 1),
            "M" | "m" if (1..=8).contains(&n) => m |= 1 << (n - 1),
            _ => {}
        }
    }
    (d, m)
}

/// Sum of the DCA faders (dB) of the DCAs in `mask`; -144 if any is at -oo; clamped to +10 dB
/// headroom by the strip ([`super::law::fader_gain`]).
pub fn dca_sum_db(mask: u16, dca_db: &[f32; 16]) -> f32 {
    let mut sum = 0.0f32;
    for (k, &g) in dca_db.iter().enumerate() {
        if mask & (1 << k) != 0 {
            if g <= FADER_OFF_DB + 0.01 {
                return FADER_OFF_DB;
            }
            sum += g;
        }
    }
    sum.max(FADER_OFF_DB)
}

/// Snapshot of the resolved state (for tests and status).
#[derive(Clone, Debug, PartialEq)]
pub struct LogicState {
    pub gain_db: [f32; STRIP_COUNT],
    pub flags: [u32; STRIP_COUNT],
    pub any_solo: bool,
    pub mode: SoloMode,
}

/// Control-side DCA / mute-group / solo resolver. See the module docs.
pub struct MixLogic {
    ids: Ids,
    shared: Arc<MixShared>,
    dirty: Arc<AtomicBool>,
    watch: Arc<[bool]>,
    solo_btn: Arc<[bool]>,
    /// 1 + node id of the last solo button switched on (0 = none).
    last_on: Arc<AtomicU32>,
    /// `$stat/solo` value written by the last refresh.
    stat_written: Option<bool>,
    /// Monitors the host plays (bit 0 = monitor 1, bit 1 = monitor 2), see the module docs.
    listen: u8,
    state: LogicState,
}

impl MixLogic {
    /// Resolve every id and compute the initial state (not yet published; call
    /// [`MixLogic::refresh`]).
    pub fn new(store: &ParamStore) -> Self {
        let ids = Ids::new(store);
        let n = store.schema().node_count();
        let mut watch = vec![false; n];
        for p in ids.watched() {
            watch[p.index()] = true;
        }
        let mut solo_btn = vec![false; n];
        for p in ids.solo_buttons() {
            solo_btn[p.index()] = true;
        }
        Self {
            ids,
            shared: Arc::new(MixShared::default()),
            dirty: Arc::new(AtomicBool::new(true)),
            watch: watch.into(),
            solo_btn: solo_btn.into(),
            last_on: Arc::new(AtomicU32::new(0)),
            stat_written: None,
            listen: 0,
            state: LogicState {
                gain_db: [0.0; STRIP_COUNT],
                flags: [0; STRIP_COUNT],
                any_solo: false,
                mode: SoloMode::Live,
            },
        }
    }

    /// The RT-side state (give it to [`super::MixConfig::logic`]).
    pub fn shared(&self) -> Arc<MixShared> {
        self.shared.clone()
    }

    /// Store hook: marks the logic dirty on watched changes. Register before sharing the store.
    pub fn change_hook(&self) -> Hook {
        let (watch, solo_btn) = (self.watch.clone(), self.solo_btn.clone());
        let (dirty, last_on) = (self.dirty.clone(), self.last_on.clone());
        Box::new(move |c| {
            let i = c.id as usize;
            if watch.get(i).copied().unwrap_or(false) {
                if solo_btn[i] && c.raw != 0 {
                    last_on.store(c.id as u32 + 1, Release);
                }
                dirty.store(true, Release);
            }
        })
    }

    /// Mark dirty if `id` is watched (for callers that see changes another way).
    pub fn notify(&self, id: ParamId, raw: u32) {
        if self.watch[id.index()] {
            if self.solo_btn[id.index()] && raw != 0 {
                self.last_on.store(id.0 as u32 + 1, Release);
            }
            self.dirty.store(true, Release);
        }
    }

    /// Monitors the host outputs play (bit 0 = monitor 1 / PH, bit 1 = monitor 2 / SPK; 0 =
    /// none, plain WING routing). A solo that reaches neither listened monitor is also sent to
    /// them (see the module docs). Takes effect on the next [`MixLogic::refresh`].
    pub fn set_listen(&mut self, mask: u8) {
        let mask = mask & 3;
        if mask != self.listen {
            self.listen = mask;
            self.dirty.store(true, Release);
        }
    }

    /// The listened monitors ([`MixLogic::set_listen`]).
    pub fn listen(&self) -> u8 {
        self.listen
    }

    /// The last resolved state.
    pub fn state(&self) -> &LogicState {
        &self.state
    }

    /// Recompute and publish if anything changed since the last call (control thread). Writes
    /// solo indicators and enforces exclusive solo / clear solo through `store`. Returns whether
    /// a new state was published.
    pub fn refresh(&mut self, store: &ParamStore) -> bool {
        if !self.dirty.swap(false, AcqRel) {
            return false;
        }
        let ids = &self.ids;
        let get = |p: ParamId| store.get_f64(p).unwrap_or(0.0);
        let on = |p: ParamId| get(p) != 0.0;
        let set = |p: ParamId, v: i32| {
            let _ = store.set(p, Value::Int(v));
        };

        // ---- solo buttons: exclusive and clear ------------------------------------------------
        let buttons: Vec<ParamId> = ids.solo_buttons().collect();
        let any_button = buttons.iter().any(|&p| on(p));
        let stat_now = on(ids.stat);
        if any_button && self.stat_written == Some(true) && !stat_now {
            for &p in &buttons {
                set(p, 0);
            }
        } else {
            let last = self.last_on.swap(0, AcqRel);
            if last != 0 && on(ids.excl) {
                let keep = ParamId((last - 1) as u16);
                if on(keep) {
                    for &p in &buttons {
                        if p != keep && on(p) {
                            set(p, 0);
                        }
                    }
                }
            }
        }

        // ---- inputs -----------------------------------------------------------------------------
        let dca_db: [f32; 16] = std::array::from_fn(|k| get(ids.dca_fdr[k]) as f32);
        let dca_mute = mask16(|k| on(ids.dca_mute[k]));
        let dca_solo = mask16(|k| on(ids.dca_solo[k]));
        let mgrp_mute = (0..8).fold(0u8, |m, k| m | ((on(ids.mgrp_mute[k]) as u8) << k));
        let dcamgrp = on(ids.dcamgrp);
        let mode = match get(ids.mode) as i32 {
            1 => SoloMode::Studio,
            2 => SoloMode::InPlace,
            _ => SoloMode::Live,
        };
        let solo_mon = mon_mask(get(ids.solo_mon) as i32);
        let afl: [bool; 4] = std::array::from_fn(|k| get(ids.taps[k]) as i32 == 1);
        let src_strip = if on(ids.srcsolo_on) {
            match get(ids.srcsolo) as i32 {
                1 => Strip::Ch(39).index(),
                2 => Strip::Aux(7).index(),
                _ => None,
            }
        } else {
            None
        };

        // ---- per strip --------------------------------------------------------------------------
        let mut gain = [0f32; STRIP_COUNT];
        let mut flags = [0u32; STRIP_COUNT];
        let mut soloed = [0u8; STRIP_COUNT]; // 0 no, 1 own, 2 implicit
        for (i, s) in ids.strips.iter().enumerate() {
            let (d, m) = store.with_str(s.tags, parse_tags).unwrap_or((0, 0));
            gain[i] = dca_sum_db(d, &dca_db);
            if m & mgrp_mute != 0 {
                flags[i] |= flag::MUTE;
            }
            if d & dca_mute != 0 {
                // mains and matrices have no main send: a DCA mute mutes them outright [I]
                flags[i] |= if dcamgrp || i >= MAIN_BASE {
                    flag::MUTE
                } else {
                    flag::MAIN_MUTE
                };
            }
            soloed[i] = if on(s.solo) {
                1
            } else if d & dca_solo != 0 || src_strip == Some(i) {
                2
            } else {
                0
            };
        }
        let any_solo = soloed.iter().any(|&s| s != 0);
        let input_solo = soloed[..48].iter().any(|&s| s != 0);
        let bus_solo = soloed[48..64].iter().any(|&s| s != 0);
        let dest = solo_dest(solo_mon, self.listen);
        for (i, s) in ids.strips.iter().enumerate() {
            if soloed[i] != 0 {
                let m = dest;
                if m & 1 != 0 {
                    flags[i] |= flag::SOLO_MON1;
                }
                if m & 2 != 0 {
                    flags[i] |= flag::SOLO_MON2;
                }
                let t = match i {
                    0..=47 => 0,
                    48..=63 => 1,
                    64..=67 => 2,
                    _ => 3,
                };
                if afl[t] {
                    flags[i] |= flag::AFL;
                }
            } else if mode == SoloMode::InPlace {
                let safe = s.solosafe.is_some_and(on);
                if (i < 48 && input_solo && !safe) || ((48..64).contains(&i) && bus_solo) {
                    flags[i] |= flag::MUTE;
                }
            }
        }
        if gain != self.state.gain_db || flags != self.state.flags || self.shared.generation() == 0
        {
            self.shared.publish(&gain, &flags);
        }

        // ---- indicators -------------------------------------------------------------------------
        for (i, s) in ids.strips.iter().enumerate() {
            set(s.sololed, soloed[i] as i32);
        }
        for k in 0..16 {
            set(ids.dca_sololed[k], (dca_solo >> k & 1) as i32);
        }
        set(ids.stat, any_solo as i32);
        // (our own writes above re-trigger the hook once; the next refresh recomputes the same
        // state and writes nothing, so it settles)
        self.stat_written = Some(any_solo);

        self.state = LogicState {
            gain_db: gain,
            flags,
            any_solo,
            mode,
        };
        true
    }
}

fn mask16(f: impl Fn(usize) -> bool) -> u16 {
    (0..16).fold(0u16, |m, k| m | ((f(k) as u16) << k))
}

/// Monitors a solo reaches: its WING destination `wing` (`cfg/solo/mon`), plus the `listen`ed
/// monitors when it reaches monitors but none of those (T03G).
pub fn solo_dest(wing: u8, listen: u8) -> u8 {
    if wing != 0 && wing & listen == 0 {
        wing | listen
    } else {
        wing
    }
}

/// `PH` = 1 (monitor 1), `SPK` = 2 (monitor 2), `PH+SPK` = 3 (`cfg/solo/mon` enum index 0/1/2).
fn mon_mask(idx: i32) -> u8 {
    match idx {
        1 => 2,
        2 => 3,
        _ => 1,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tags() {
        assert_eq!(parse_tags("#M1,DCA.1"), (0, 1));
        assert_eq!(parse_tags("#D5,#D7"), (0b101_0000, 0));
        assert_eq!(
            parse_tags(" #D16 , #M8,#D0,#M9,#X1,INSTRUM"),
            (1 << 15, 1 << 7)
        );
        assert_eq!(parse_tags(""), (0, 0));
    }

    #[test]
    fn solo_destination_follows_listened_monitor() {
        // WING routing without a listen mask
        for w in 0..4 {
            assert_eq!(solo_dest(w, 0), w);
        }
        // host plays mon/1 (PH): PH and PH+SPK unchanged, SPK also to mon/1, none stays none
        assert_eq!(solo_dest(1, 1), 1);
        assert_eq!(solo_dest(3, 1), 3);
        assert_eq!(solo_dest(2, 1), 3);
        assert_eq!(solo_dest(0, 1), 0);
        // host plays mon/2 (SPK)
        assert_eq!(solo_dest(1, 2), 3);
        assert_eq!(solo_dest(2, 2), 2);
        // both played: nothing to add
        assert_eq!(solo_dest(1, 3), 1);
        assert_eq!(solo_dest(2, 3), 2);
        assert_eq!(mon_mask(0), 1);
        assert_eq!(mon_mask(1), 2);
        assert_eq!(mon_mask(2), 3);
    }

    #[test]
    fn dca_sum() {
        let mut d = [0f32; 16];
        d[0] = -6.0;
        d[1] = -4.5;
        d[2] = FADER_OFF_DB;
        assert_eq!(dca_sum_db(0, &d), 0.0);
        assert_eq!(dca_sum_db(0b01, &d), -6.0);
        assert_eq!(dca_sum_db(0b11, &d), -10.5);
        assert_eq!(dca_sum_db(0b111, &d), FADER_OFF_DB);
    }
}
