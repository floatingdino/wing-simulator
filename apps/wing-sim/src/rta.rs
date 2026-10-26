//! Console RTA service (T084): the console-side state WING-Edit checks before it draws an RTA,
//! and the non-RT analyser thread that turns the engine's RTA ring into `AA` meter words.
//!
//! # Why WING-Edit said "RTA in use! Check mixer settings." (docs/research/rta.md)
//!
//! `gui_modules::RtaGraph::paint` (WING-Edit 3.3.4) compares the console-owned read-only RTA
//! status `cfg/rta/$src`, `$tap`, `$dec` with the RTA settings `rtasrc`, `rtatap`, `rtadecay`
//! (WING-Edit itself writes `rtasrc`/`rtatap` for the strip its EQ page shows). Any mismatch
//! draws that message instead of the spectrum. A console reports the RTA it actually runs in the
//! `$` leaves; the simulator never did, so they kept their power-on values.
//!
//! [`sync_status`] mirrors the settings into the status leaves (`$src = rtasrc`, `$tap =
//! rtatap`, `$dec = rtadecay`, `$det = rtadet` limited to PEAK/RMS) with the simulator's origin, so
//! every client receives them as change events.
//!
//! # Analyser thread ([`RtaService`])
//!
//! Every [`METER_INTERVAL`] (WING's 50 ms meter rate): sync the status, request the selection
//! from the engine ([`RtaFeed::request`]), drain the ring, run [`RtaAnalyser`] with the current
//! detector/decay and write the 120 bins (1/128 dB, [`db_to_rta`]) into the meter bank's `AA`
//! block, which the meter streamer sends to every report that selects it. While no audio session
//! feeds the ring the bank keeps the synthetic RTA ([`wing_proto::server::SyntheticMeters`]).
//! The analyser costs about 6 ms of one core per second of 48 kHz audio (release build;
//! `wing_engine::rta` test `analyser_cost`).

use std::io;
use std::sync::atomic::{AtomicBool, Ordering::Relaxed};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use wing_engine::fx::FX_SLOTS;
use wing_engine::routing::{Strip, STRIP_COUNT};
use wing_engine::rta::{
    RtaAnalyser, RtaDecay, RtaDetector, RtaFeed, RtaReader, RtaSel, RtaTap, RTA_BANDS, RTA_RING,
};
use wing_model::{NodeId, ParamStore, Value, ORIGIN_LOCAL};
use wing_proto::nrp::meter::db_to_rta;
use wing_proto::nrp::MeterKind;
use wing_proto::server::meters::METER_INTERVAL;
use wing_proto::server::MeterBank;

/// The RTA settings in effect (from `cfg/rta`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RtaConfig {
    /// `None` while `rtasrc` is 0 (off).
    pub sel: Option<RtaSel>,
    pub detector: RtaDetector,
    pub decay: RtaDecay,
}

/// Node ids of `cfg/rta` (settings, then the console-owned status leaves).
#[derive(Clone, Copy, Debug)]
pub struct RtaNodes {
    src: NodeId,
    tap: NodeId,
    decay: NodeId,
    det: NodeId,
    st_src: NodeId,
    st_tap: NodeId,
    st_dec: NodeId,
    st_det: NodeId,
}

impl RtaNodes {
    pub fn lookup(store: &ParamStore) -> Option<Self> {
        let l = |p: &str| store.lookup(&format!("cfg/rta/{p}"));
        Some(Self {
            src: l("rtasrc")?,
            tap: l("rtatap")?,
            decay: l("rtadecay")?,
            det: l("rtadet")?,
            st_src: l("$src")?,
            st_tap: l("$tap")?,
            st_dec: l("$dec")?,
            st_det: l("$det")?,
        })
    }
}

fn index(store: &ParamStore, id: NodeId) -> u32 {
    match store.get(id) {
        Some(Value::Int(i)) => i.max(0) as u32,
        Some(Value::Enum(i)) => i,
        _ => 0,
    }
}

/// Read the RTA settings and mirror them into the status leaves `$src $tap $dec $det` (sets
/// only what differs; clients get change events). `$src` (1..76) is left alone while `rtasrc`
/// is 0.
pub fn sync_status(store: &ParamStore, n: &RtaNodes) -> RtaConfig {
    let src = index(store, n.src);
    let tap = index(store, n.tap);
    let decay = index(store, n.decay);
    let det = index(store, n.det);
    let mirror = |id: NodeId, v: Value| {
        if store.get(id) != Some(v) {
            let _ = store.set_from(id, v, ORIGIN_LOCAL);
        }
    };
    if src > 0 {
        mirror(n.st_src, Value::Int(src as i32));
    }
    mirror(n.st_tap, Value::Enum(tap));
    mirror(n.st_dec, Value::Enum(decay));
    // `$det` only has PEAK/RMS; WING-Edit doubles the words whenever it is not PEAK
    mirror(n.st_det, Value::Enum(det.min(1)));
    RtaConfig {
        sel: (src > 0)
            .then(|| {
                RtaTap::from_index(tap).map(|tap| RtaSel {
                    src: src as u16,
                    tap,
                })
            })
            .flatten(),
        detector: RtaDetector::from_index(det),
        decay: RtaDecay::from_index(decay),
    }
}

/// Node ids of the FX insert status: every insert point (`<strip>/preins|postins`: `ins`, channel
/// post-insert `mode`, `$stat`) in insert-id order, and each slot's `fx/N/$a_chn` / `$a_pos`.
pub struct FxHostNodes {
    inserts: Vec<InsertNodes>,
    a_chn: Vec<Option<NodeId>>,
    a_pos: Vec<Option<NodeId>>,
}

/// One insert point.
struct InsertNodes {
    /// Strip index 0..75 (`wing_engine::routing::Strip`).
    strip: usize,
    /// 0 = Ins1 (`preins`), 1 = Ins2 (`postins`).
    pos: u8,
    ins: NodeId,
    /// Channel post-insert `mode` (FX / AUTO_X / AUTO_Y).
    mode: Option<NodeId>,
    stat: Option<NodeId>,
}

impl FxHostNodes {
    pub fn lookup(store: &ParamStore) -> Self {
        let mut inserts = Vec::new();
        for s in 0..STRIP_COUNT {
            let Some(strip) = Strip::from_index(s) else {
                continue;
            };
            let base = match strip {
                Strip::Ch(n) => format!("ch/{n}"),
                Strip::Aux(n) => format!("aux/{n}"),
                Strip::Bus(n) => format!("bus/{n}"),
                Strip::Main(n) => format!("main/{n}"),
                Strip::Mtx(n) => format!("mtx/{n}"),
            };
            for (pos, blk) in [(0u8, "preins"), (1, "postins")] {
                if let Some(ins) = store.lookup(&format!("{base}/{blk}/ins")) {
                    inserts.push(InsertNodes {
                        strip: s,
                        pos,
                        ins,
                        mode: store.lookup(&format!("{base}/{blk}/mode")),
                        stat: store.lookup(&format!("{base}/{blk}/$stat")),
                    });
                }
            }
        }
        let leaf = |p: &str| {
            (1..=FX_SLOTS)
                .map(|n| store.lookup(&format!("fx/{n}/{p}")))
                .collect()
        };
        Self {
            inserts,
            a_chn: leaf("$a_chn"),
            a_pos: leaf("$a_pos"),
        }
    }
}

/// Mirror the FX insert status the console owns (T086): `fx/N/$a_chn` = the owning strip (1..76,
/// the `cfg/rta/rtasrc` numbering; 0 = not inserted), `fx/N/$a_pos` = 0 for Ins1 / 1 for Ins2, and
/// every insert's `$stat` (`-` none, `OK`, `N/A` = the slot is owned by another insert). The owner
/// is the engine's rule (`wing_engine::fx`): every insert selecting `FXn` (a channel post-insert only
/// in FX mode) claims it, the lowest insert id wins. Sets only what differs.
///
/// WING-Edit online does not compute these (`ae::Cinsert::update` returns at once while connected,
/// 0x10001307c); its FX-page RTA (`RtaGraph::setFxRta`) only claims the RTA, and its `paint` only
/// chooses between "RTA in use on another window!" and "RTA in use! Select INSERTED channel." by
/// comparing the selected strip with `$a_chn - 1` (docs/research/rta.md §1a).
pub fn sync_fx_hosts(store: &ParamStore, n: &FxHostNodes) {
    let mut owner: [Option<(usize, u8)>; FX_SLOTS] = [None; FX_SLOTS];
    let mut claim = Vec::with_capacity(n.inserts.len());
    for i in &n.inserts {
        let fx = index(store, i.ins) as usize;
        let fx_mode = i.mode.is_none_or(|m| index(store, m) == 0);
        let fx = (fx_mode && (1..=FX_SLOTS).contains(&fx)).then_some(fx);
        if let Some(k) = fx {
            owner[k - 1].get_or_insert((i.strip, i.pos));
        }
        claim.push(fx);
    }
    let mirror = |id: Option<NodeId>, v: Value| {
        if let Some(id) = id {
            if store.get(id) != Some(v) {
                let _ = store.set_from(id, v, ORIGIN_LOCAL);
            }
        }
    };
    for (i, fx) in n.inserts.iter().zip(&claim) {
        let st = match fx {
            None => 0,
            Some(k) if owner[k - 1] == Some((i.strip, i.pos)) => 1,
            Some(_) => 2,
        };
        mirror(i.stat, Value::Enum(st));
    }
    for (k, o) in owner.iter().enumerate() {
        let (chn, pos) = o.map_or((0, 0), |(s, p)| (s as i32 + 1, p as i32));
        mirror(n.a_chn[k], Value::Int(chn));
        mirror(n.a_pos[k], Value::Int(pos));
    }
}

/// Write analyser levels (dBFS) to the bank's RTA block.
pub fn write_bins(bank: &MeterBank, levels: &[f32; RTA_BANDS]) {
    for (k, &db) in levels.iter().enumerate() {
        bank.set(MeterKind::Rta, 0, k, db_to_rta(db));
    }
}

/// One analyser step state (testable without a thread).
pub struct RtaRunner {
    nodes: Option<RtaNodes>,
    hosts: FxHostNodes,
    feed: Option<Arc<RtaFeed>>,
    reader: RtaReader,
    analyser: RtaAnalyser,
    epoch: u32,
    buf: Box<[f32]>,
    last: Option<Instant>,
}

impl RtaRunner {
    pub fn new(store: &ParamStore, feed: Option<Arc<RtaFeed>>) -> Self {
        let reader = feed.as_deref().map(RtaReader::at_end).unwrap_or_default();
        Self {
            nodes: RtaNodes::lookup(store),
            hosts: FxHostNodes::lookup(store),
            feed,
            reader,
            analyser: RtaAnalyser::new(48_000.0),
            epoch: 0,
            buf: vec![0.0; RTA_RING / 2].into(),
            last: None,
        }
    }

    pub fn analyser(&self) -> &RtaAnalyser {
        &self.analyser
    }

    /// Sync the status (RTA and FX insert hosts), update the request and, while audio feeds the ring, analyse what
    /// arrived since the last step and write the bins to `bank`. Returns the settings.
    pub fn step(
        &mut self,
        store: &ParamStore,
        bank: Option<&MeterBank>,
        now: Instant,
    ) -> Option<RtaConfig> {
        sync_fx_hosts(store, &self.hosts);
        let cfg = self.nodes.as_ref().map(|n| sync_status(store, n))?;
        let dt = self
            .last
            .map_or(0.0, |t| now.saturating_duration_since(t).as_secs_f32());
        self.last = Some(now);
        let Some(feed) = &self.feed else {
            return Some(cfg);
        };
        feed.request(cfg.sel);
        if !feed.is_attached() {
            return Some(cfg);
        }
        let fs = feed.sample_rate() as f64;
        if fs != self.analyser.sample_rate() {
            self.analyser.set_sample_rate(fs);
            self.reader = RtaReader::at_end(feed);
        }
        let e = feed.epoch();
        if e != self.epoch {
            self.epoch = e;
            self.analyser.reset();
        }
        self.analyser.set_detector(cfg.detector);
        self.analyser.set_decay(cfg.decay);
        loop {
            let n = self.reader.read(feed, &mut self.buf);
            if n == 0 {
                break;
            }
            self.analyser.process(&self.buf[..n]);
        }
        let levels = self.analyser.frame(dt);
        if let Some(b) = bank {
            write_bins(b, levels);
        }
        Some(cfg)
    }
}

/// The analyser thread (see the module docs).
pub struct RtaService {
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

impl RtaService {
    /// Run [`RtaRunner::step`] every `interval` ([`METER_INTERVAL`] in the daemon).
    pub fn spawn(
        store: Arc<ParamStore>,
        feed: Option<Arc<RtaFeed>>,
        bank: Option<Arc<MeterBank>>,
        interval: Duration,
    ) -> io::Result<Self> {
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = stop.clone();
        let thread = std::thread::Builder::new()
            .name("wing-rta".into())
            .spawn(move || {
                let mut run = RtaRunner::new(&store, feed);
                let mut next = Instant::now();
                while !stop2.load(Relaxed) {
                    run.step(&store, bank.as_deref(), Instant::now());
                    next += interval;
                    let now = Instant::now();
                    if next > now {
                        std::thread::sleep(next - now);
                    } else {
                        next = now;
                    }
                }
            })?;
        Ok(Self {
            stop,
            thread: Some(thread),
        })
    }

    /// The daemon's default: WING's meter rate.
    pub fn spawn_default(
        store: Arc<ParamStore>,
        feed: Option<Arc<RtaFeed>>,
        bank: Option<Arc<MeterBank>>,
    ) -> io::Result<Self> {
        Self::spawn(store, feed, bank, METER_INTERVAL)
    }

    pub fn stop(mut self) {
        self.shutdown();
    }

    fn shutdown(&mut self) {
        self.stop.store(true, Relaxed);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

impl Drop for RtaService {
    fn drop(&mut self) {
        self.shutdown();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use wing_engine::rta::band_freq;
    use wing_proto::nrp::meter::rta_to_db;

    fn set(st: &ParamStore, p: &str, v: Value) {
        st.set(st.lookup(p).unwrap(), v).unwrap();
    }

    /// The condition of WING-Edit's `RtaGraph::paint` (positions: `$src` min 1, `rtasrc` min 0).
    fn wing_edit_shows_rta(st: &ParamStore) -> bool {
        let g = |p: &str| index(st, st.lookup(&format!("cfg/rta/{p}")).unwrap()) as i64;
        g("$src") - 1 == g("rtasrc") - 1 && g("$tap") == g("rtatap") && g("$dec") == g("rtadecay")
    }

    /// What WING-Edit 3.3.4's `gui_modules::RtaGraph::paint` draws (0x1000e3a04..0x1000e3b18,
    /// docs/research/rta.md §4), from the values WING-Edit holds (the console's).
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    enum Overlay {
        Spectrum,
        /// "RTA in use! Check mixer settings."
        CheckMixer,
        /// "RTA in use on another window!"
        AnotherWindow,
        /// "RTA in use! Select INSERTED channel."
        SelectInserted,
    }

    /// One `RtaGraph`: its own source/tap (`+0x1d8` / `+0x1dc`), its FX slot (`+0x1d4`, 0-based;
    /// `None` = strip graph) and WING-Edit's selected strip (`CE+0xd0`, 0-based, rtasrc numbering - 1).
    struct Graph {
        src: i64,
        tap: i64,
        fx: Option<usize>,
        selected: i64,
    }

    fn pos(st: &ParamStore, p: &str) -> i64 {
        index(st, st.lookup(p).unwrap()) as i64
    }

    fn overlay(st: &ParamStore, g: &Graph) -> Overlay {
        let r = |p: &str| pos(st, &format!("cfg/rta/{p}"));
        // Cpar positions: `$src` has min 1 (pos = value - 1), `rtasrc` min 0
        if !(r("$src") - 1 == r("rtasrc") - 1
            && r("$tap") == r("rtatap")
            && r("$dec") == r("rtadecay"))
        {
            return Overlay::CheckMixer;
        }
        if g.src == r("rtasrc") && g.tap == r("$tap") {
            return Overlay::Spectrum;
        }
        match g.fx {
            Some(k) if g.selected != pos(st, &format!("fx/{}/$a_chn", k + 1)) - 1 => {
                Overlay::SelectInserted
            }
            _ => Overlay::AnotherWindow,
        }
    }

    /// The RTA claim of a graph: a strip graph's `listener_callback` (selection / RTA display change,
    /// display on) writes `rtasrc = strip + 1`, `rtatap = tap`; an FX graph's `setFxRta` writes
    /// `rtasrc = slot + 1`, `rtatap = FXIN` only while the selected strip is the slot's `$a_chn - 1`.
    fn claim(st: &ParamStore, g: &Graph) {
        if let Some(k) = g.fx {
            if g.selected != pos(st, &format!("fx/{}/$a_chn", k + 1)) - 1 {
                return;
            }
        }
        set(st, "cfg/rta/rtasrc", Value::Int(g.src as i32));
        set(st, "cfg/rta/rtatap", Value::Enum(g.tap as u32));
    }

    /// T086: the full RTA condition. A strip graph draws once its claim is mirrored; any other graph
    /// then reads "in use on another window"; an FX-page RTA needs the console's `$a_chn`.
    #[test]
    fn wing_edit_rta_overlay_condition() {
        let st = ParamStore::new();
        let n = RtaNodes::lookup(&st).unwrap();
        let hosts = FxHostNodes::lookup(&st);
        let fxin = RtaTap::FxIn as i64;
        // ch 5 EQ page, PRE EQ
        let eq = Graph {
            src: 5,
            tap: RtaTap::PreEq as i64,
            fx: None,
            selected: 4,
        };
        claim(&st, &eq);
        assert_eq!(overlay(&st, &eq), Overlay::CheckMixer, "before the mirror");
        sync_status(&st, &n);
        assert_eq!(overlay(&st, &eq), Overlay::Spectrum);
        // a second strip graph (another window) for ch 1
        let other = Graph {
            src: 1,
            tap: RtaTap::PreEq as i64,
            fx: None,
            selected: 4,
        };
        assert_eq!(overlay(&st, &other), Overlay::AnotherWindow);

        // FX 14 (GEQ) on bus 5 Ins2, its FX page open with bus 5 selected (strip index 52)
        set(&st, "bus/5/postins/ins", Value::Enum(14));
        let geq = Graph {
            src: 14,
            tap: fxin,
            fx: Some(13),
            selected: 52,
        };
        // without the console's $a_chn WING-Edit neither claims nor draws
        claim(&st, &geq);
        sync_status(&st, &n);
        assert_eq!(overlay(&st, &geq), Overlay::SelectInserted);
        sync_fx_hosts(&st, &hosts);
        assert_eq!(
            st.get(st.lookup("fx/14/$a_chn").unwrap()),
            Some(Value::Int(53))
        );
        claim(&st, &geq);
        sync_status(&st, &n);
        assert_eq!(overlay(&st, &geq), Overlay::Spectrum);
        // ... and now the ch 5 EQ graph is the other window, until its next claim
        assert_eq!(overlay(&st, &eq), Overlay::AnotherWindow);
        claim(&st, &eq);
        sync_status(&st, &n);
        assert_eq!(overlay(&st, &eq), Overlay::Spectrum);
        // FX page with another strip selected
        let geq_elsewhere = Graph { selected: 3, ..geq };
        assert_eq!(overlay(&st, &geq_elsewhere), Overlay::SelectInserted);
    }

    /// T086: `$a_chn` / `$a_pos` / `$stat` follow the engine's ownership rule.
    #[test]
    fn fx_host_status() {
        let st = ParamStore::new();
        let hosts = FxHostNodes::lookup(&st);
        let get = |p: &str| st.get(st.lookup(p).unwrap());
        // the user's scene: FX7 on bus 2 Ins1, FX9 on ch 26 Ins2 (FX mode), FX8 on ch 9 Ins1 (off)
        set(&st, "bus/2/preins/ins", Value::Enum(7));
        set(&st, "ch/26/postins/ins", Value::Enum(9));
        set(&st, "ch/9/preins/ins", Value::Enum(8));
        set(&st, "ch/9/preins/on", Value::Int(0));
        // FX8 also selected by bus 1 Ins2: ch 9 has the lower insert id and keeps it
        set(&st, "bus/1/postins/ins", Value::Enum(8));
        // a channel post-insert in AUTO_X mode does not claim its FX
        set(&st, "ch/3/postins/ins", Value::Enum(5));
        set(&st, "ch/3/postins/mode", Value::Enum(1));
        sync_fx_hosts(&st, &hosts);
        assert_eq!(get("fx/7/$a_chn"), Some(Value::Int(50)));
        assert_eq!(get("fx/7/$a_pos"), Some(Value::Int(0)));
        assert_eq!(get("fx/9/$a_chn"), Some(Value::Int(26)));
        assert_eq!(get("fx/9/$a_pos"), Some(Value::Int(1)));
        assert_eq!(get("fx/8/$a_chn"), Some(Value::Int(9)));
        assert_eq!(get("fx/5/$a_chn"), Some(Value::Int(0)));
        assert_eq!(get("bus/2/preins/$stat"), Some(Value::Enum(1)));
        assert_eq!(get("bus/1/postins/$stat"), Some(Value::Enum(2)));
        assert_eq!(get("ch/3/postins/$stat"), Some(Value::Enum(0)));
        // releasing the owner hands the slot over
        set(&st, "ch/9/preins/ins", Value::Enum(0));
        sync_fx_hosts(&st, &hosts);
        assert_eq!(get("fx/8/$a_chn"), Some(Value::Int(49)));
        assert_eq!(get("fx/8/$a_pos"), Some(Value::Int(1)));
        assert_eq!(get("bus/1/postins/$stat"), Some(Value::Enum(1)));
    }

    #[test]
    fn status_mirror_clears_rta_in_use() {
        let st = ParamStore::new();
        let n = RtaNodes::lookup(&st).unwrap();
        // what WING-Edit's EQ page writes for ch 5, PRE EQ
        set(&st, "cfg/rta/rtasrc", Value::Int(5));
        set(&st, "cfg/rta/rtatap", Value::Enum(RtaTap::PreEq as u32));
        set(&st, "cfg/rta/rtadecay", Value::Enum(2));
        set(&st, "cfg/rta/rtadet", Value::Enum(2));
        assert!(!wing_edit_shows_rta(&st), "power-on status differs");
        let cfg = sync_status(&st, &n);
        assert!(wing_edit_shows_rta(&st));
        assert_eq!(
            cfg.sel,
            Some(RtaSel {
                src: 5,
                tap: RtaTap::PreEq
            })
        );
        assert_eq!(cfg.sel.unwrap().strip(), Some(4));
        assert_eq!(cfg.detector, RtaDetector::Avg);
        assert_eq!(cfg.decay, RtaDecay::Fast);
        assert_eq!(st.get(n.st_det), Some(Value::Enum(1)));
        // rtasrc 0 = off: no request
        set(&st, "cfg/rta/rtasrc", Value::Int(0));
        assert_eq!(sync_status(&st, &n).sel, None);
    }

    #[test]
    fn runner_analyses_the_ring_into_the_bank() {
        let st = ParamStore::new();
        set(&st, "cfg/rta/rtasrc", Value::Int(1));
        let feed = RtaFeed::shared();
        let bank = MeterBank::for_schema();
        let mut run = RtaRunner::new(&st, Some(feed.clone()));
        let t0 = Instant::now();
        // detached: requests the selection but leaves the bank (synthetic RTA) alone
        let before = bank.get(MeterKind::Rta, 0, RTA_A4).unwrap();
        run.step(&st, Some(&bank), t0);
        assert_eq!(feed.requested().unwrap().src, 1);
        assert_eq!(bank.get(MeterKind::Rta, 0, RTA_A4).unwrap(), before);
        // attach and feed 0.6 s of a -6 dBFS A4 the way the engine does (latch, then push)
        feed.set_sample_rate(48_000);
        feed.latch();
        run.step(&st, Some(&bank), t0);
        let fs = 48_000.0;
        for blk in 0..450 {
            let x: Vec<f32> = (0..64)
                .map(|i| {
                    let t = (blk * 64 + i) as f64 / fs;
                    0.5 * (std::f64::consts::TAU * band_freq(RTA_A4) * t).sin() as f32
                })
                .collect();
            feed.push_mono(&x);
            if blk % 75 == 74 {
                run.step(&st, Some(&bank), t0 + Duration::from_millis(blk as u64));
            }
        }
        let db = rta_to_db(bank.get(MeterKind::Rta, 0, RTA_A4).unwrap());
        assert!((db + 6.02).abs() < 0.5, "A4 reads {db} dB");
        let far = rta_to_db(bank.get(MeterKind::Rta, 0, RTA_A4 + 12).unwrap());
        assert!(far < db - 30.0, "A5 reads {far} dB");
    }

    const RTA_A4: usize = wing_engine::rta::RTA_A4_BAND;
}
