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

/// Write analyser levels (dBFS) to the bank's RTA block.
pub fn write_bins(bank: &MeterBank, levels: &[f32; RTA_BANDS]) {
    for (k, &db) in levels.iter().enumerate() {
        bank.set(MeterKind::Rta, 0, k, db_to_rta(db));
    }
}

/// One analyser step state (testable without a thread).
pub struct RtaRunner {
    nodes: Option<RtaNodes>,
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

    /// Sync the status, update the request and, while audio feeds the ring, analyse what
    /// arrived since the last step and write the bins to `bank`. Returns the settings.
    pub fn step(
        &mut self,
        store: &ParamStore,
        bank: Option<&MeterBank>,
        now: Instant,
    ) -> Option<RtaConfig> {
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
