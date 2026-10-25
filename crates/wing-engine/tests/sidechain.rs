//! T080: gate/dynamics side-chain key source routing through the complete mix graph (`gatesc`/`dynsc`
//! `src`, `tap`, key filter, key listen `$solo`). Cross-strip keys arrive one block late (see
//! `wing_engine::mix::keys`).
//!
//! Levels are measured as the gain of the keyed strip relative to an unprocessed copy of the same
//! programme over the same window, so the programme's own level variation cancels.

mod common;

use common::rig::*;

use wing_engine::mix::{InputSignal as I, MixConfig, MixLogic, MixSignal as M};
use wing_engine::routing::{Side, Strip};
use wing_model::ParamStore;

const SETTLE: usize = 150;
const MEASURE: usize = 50;

fn rms_db(x: &[f32]) -> f64 {
    let s: f64 = x.iter().map(|&v| (v as f64) * (v as f64)).sum();
    10.0 * (s / x.len().max(1) as f64 + 1e-30).log10()
}

type Probe<'a> = &'a dyn Fn(&Rig) -> Vec<f32>;

struct T {
    store: ParamStore,
    logic: MixLogic,
    r: Rig,
}

impl T {
    /// ch1 <- USB 1 (trimmed to a quiet level), ch2 <- USB 2; ch1/ch2 to main 1; ch3 = an unprocessed copy
    /// of ch1 (the level reference), sent only to bus 8; `extra` before the build.
    fn new(extra: impl FnOnce(&ParamStore)) -> Self {
        let mut store = ParamStore::new();
        let mut logic = MixLogic::new(&store);
        store.add_hook(logic.change_hook());
        connect_usb(&store, "ch/1", 1);
        connect_usb(&store, "ch/2", 2);
        connect_usb(&store, "ch/3", 1);
        setf(&store, "ch/1/in/set/trim", -18.0);
        setf(&store, "ch/3/in/set/trim", -18.0);
        seti(&store, "ch/3/main/1/on", 0);
        seti(&store, "ch/3/send/8/on", 1);
        setf(&store, "ch/3/send/8/lvl", 0.0);
        for p in [
            "ch/1/fdr",
            "ch/2/fdr",
            "ch/3/fdr",
            "main/1/fdr",
            "bus/8/fdr",
        ] {
            setf(&store, p, 0.0);
        }
        extra(&store);
        logic.refresh(&store);
        let cfg = MixConfig {
            logic: Some(logic.shared()),
            ..MixConfig::default()
        };
        let r = rig(&store, cfg, 2);
        T { store, logic, r }
    }

    fn set(&self, p: &str, v: &str) {
        sett(&self.store, p, v);
    }

    /// Settle with USB inputs `usb` fed, then collect `probes` over [`MEASURE`] cycles; RMS (dB) each.
    fn measure<const N: usize>(&mut self, usb: &[u32], probes: [Probe; N]) -> [f64; N] {
        self.logic.refresh(&self.store);
        self.logic.refresh(&self.store);
        self.r.cycles(SETTLE, usb);
        let mut acc: [Vec<f32>; N] = std::array::from_fn(|_| Vec::new());
        for _ in 0..MEASURE {
            self.r.cycle(usb);
            for (a, p) in acc.iter_mut().zip(&probes) {
                a.extend(p(&self.r));
            }
        }
        acc.map(|a| rms_db(&a))
    }

    /// Gain (dB) of `probe` relative to `reference`.
    fn gain(&mut self, usb: &[u32], probe: Probe, reference: Probe) -> f64 {
        let [p, q] = self.measure(usb, [probe, reference]);
        p - q
    }

    /// ch1 PFL vs its unprocessed copy (ch3).
    fn ch1(&mut self, usb: &[u32]) -> f64 {
        self.gain(usb, &|r| r.input(0, I::Pfl, Side::L), &|r| {
            r.input(2, I::Pfl, Side::L)
        })
    }

    /// bus 7 PFL vs bus 8 (fed by the unprocessed copy of ch1).
    fn bus7(&mut self, usb: &[u32]) -> f64 {
        self.gain(usb, &|r| r.mix(Strip::Bus(7), M::Pfl, Side::L), &|r| {
            r.mix(Strip::Bus(8), M::Pfl, Side::L)
        })
    }
}

/// Ducker in ch1's gate slot (thr -40 dB, range 20 dB, fast attack, short hold/release).
fn ducker(s: &ParamStore) {
    sett(s, "ch/1/gate/mdl", "DUCK");
    setf(s, "ch/1/gate/1", -40.0);
    setf(s, "ch/1/gate/2", 20.0);
    setf(s, "ch/1/gate/3", 0.0);
    setf(s, "ch/1/gate/4", 30.0);
    setf(s, "ch/1/gate/5", 20.0);
    seti(s, "ch/1/gate/on", 1);
}

/// Compressor (thr -40 dB, 10:1, fast fixed release) in the dyn slot of `p`.
fn comp(s: &ParamStore, p: &str) {
    sett(s, &format!("{p}/dyn/mdl"), "COMP");
    setf(s, &format!("{p}/dyn/1"), -40.0);
    setf(s, &format!("{p}/dyn/2"), 10.0);
    setf(s, &format!("{p}/dyn/7"), 20.0);
    seti(s, &format!("{p}/dyn/9"), 0);
    seti(s, &format!("{p}/dyn/on"), 1);
}

#[test]
fn ducker_on_ch1_keyed_from_ch2_ducks() {
    let mut t = T::new(|s| {
        ducker(s);
        sett(s, "ch/1/gatesc/src", "CH.2");
    });
    // ch2 silent: no key, no ducking (ch1's own level does not count)
    let alone = t.ch1(&[1]);
    assert!(alone.abs() < 0.5, "unkeyed: {alone:.2} dB");
    let ducked = t.ch1(&[1, 2]);
    assert!(
        ducked < -15.0,
        "ch2 key should duck ch1 by ~20 dB: {ducked:.1} dB"
    );
    // back to SELF: ch1 keys from itself (above the threshold: ducked), ch2 makes no difference
    t.set("ch/1/gatesc/src", "SELF");
    let self_alone = t.ch1(&[1]);
    let self_both = t.ch1(&[1, 2]);
    assert!(
        (self_alone - self_both).abs() < 0.5,
        "SELF key ignores ch2: {self_alone:.1} vs {self_both:.1} dB"
    );
}

#[test]
fn channel_key_filter_and_tap_apply_to_external_keys() {
    let mut t = T::new(|s| {
        ducker(s);
        sett(s, "ch/1/gatesc/src", "CH.2");
    });
    assert!(t.ch1(&[1, 2]) < -15.0);
    // a key filter far above ch2's content (sig() is < 250 Hz) removes the key
    t.set("ch/1/gatesc/type", "HP12");
    t.set("ch/1/gatesc/f", "15000");
    let filtered = t.ch1(&[1, 2]);
    assert!(
        filtered.abs() < 0.5,
        "HP-filtered key should not duck: {filtered:.1} dB"
    );
    t.set("ch/1/gatesc/type", "OFF");
    // tap AFL with ch2's fader at -oo: no key; tap IN (before the fader): key
    t.set("ch/2/fdr", "-oo");
    t.set("ch/1/gatesc/tap", "AFL");
    let afl = t.ch1(&[1, 2]);
    assert!(afl.abs() < 0.5, "AFL tap, fader -oo: {afl:.1} dB");
    t.set("ch/1/gatesc/tap", "IN");
    let pre = t.ch1(&[1, 2]);
    assert!(pre < -15.0, "IN tap: {pre:.1} dB");
}

#[test]
fn dynamics_slot_keyed_from_another_channel() {
    let mut t = T::new(|s| {
        comp(s, "ch/1");
        sett(s, "ch/1/dynsc/src", "CH.2");
    });
    let alone = t.ch1(&[1]);
    let keyed = t.ch1(&[1, 2]);
    assert!(alone.abs() < 0.5, "unkeyed: {alone:.2} dB");
    assert!(keyed < -6.0, "keyed: {keyed:.1} dB");
}

/// ch1 -> bus 7, ch2 -> bus 5 (send level 0 dB, post); bus 7 compressor keyed from bus 5.
fn bus_setup(s: &ParamStore) {
    for (ch, bus) in [(1, 7), (2, 5)] {
        seti(s, &format!("ch/{ch}/send/{bus}/on"), 1);
        setf(s, &format!("ch/{ch}/send/{bus}/lvl"), 0.0);
    }
    for b in [5, 7] {
        setf(s, &format!("bus/{b}/fdr"), 0.0);
    }
    comp(s, "bus/7");
    // the user's WING-Edit session: bus/7/dynsc/src = BUS.5, tap = POST FDR
    sett(s, "bus/7/dynsc/src", "BUS.5");
    sett(s, "bus/7/dynsc/tap", "AFL");
}

#[test]
fn compressor_keyed_from_a_bus_reacts() {
    let mut t = T::new(bus_setup);
    let alone = t.bus7(&[1]);
    assert!(alone.abs() < 0.5, "unkeyed: {alone:.2} dB");
    let keyed = t.bus7(&[1, 2]);
    assert!(
        keyed < -6.0,
        "bus 5 key should compress bus 7: {keyed:.1} dB"
    );
    // post-fader tap: bus 5 fader at -oo removes the key; tap BUS (input) restores it
    t.set("bus/5/fdr", "-oo");
    let afl = t.bus7(&[1, 2]);
    assert!(afl.abs() < 0.5, "AFL tap, fader -oo: {afl:.1} dB");
    t.set("bus/7/dynsc/tap", "BUS");
    let pre = t.bus7(&[1, 2]);
    assert!(pre < -6.0, "BUS tap: {pre:.1} dB");
    // SELF: bus 7 keys from itself, bus 5 makes no difference
    t.set("bus/7/dynsc/src", "SELF");
    let (a, b) = (t.bus7(&[1]), t.bus7(&[1, 2]));
    assert!((a - b).abs() < 0.5, "SELF: {a:.1} vs {b:.1} dB");
}

#[test]
fn main_matrix_and_aux_keyed_from_a_bus() {
    let mut t = T::new(|s| {
        // bus 5 <- ch2 (the key); main 1 <- ch1 only; mtx 1 <- aux 1 <- USB 1
        seti(s, "ch/2/send/5/on", 1);
        setf(s, "ch/2/send/5/lvl", 0.0);
        setf(s, "bus/5/fdr", 0.0);
        seti(s, "ch/2/main/1/on", 0);
        connect_usb(s, "aux/1", 1);
        setf(s, "aux/1/in/set/trim", -18.0);
        seti(s, "aux/1/main/1/on", 0);
        seti(s, "aux/1/send/MX1/on", 1);
        setf(s, "aux/1/send/MX1/lvl", 0.0);
        setf(s, "mtx/1/fdr", 0.0);
        setf(s, "aux/1/fdr", 0.0);
        for p in ["main/1", "mtx/1", "aux/1"] {
            comp(s, p);
            sett(s, &format!("{p}/dynsc/src"), "BUS.5");
            sett(s, &format!("{p}/dynsc/tap"), "BUS");
        }
    });
    let reference: Probe = &|r| r.mix(Strip::Bus(8), M::Pfl, Side::L);
    let main: Probe = &|r| r.mix(Strip::Main(1), M::Pfl, Side::L);
    let mtx: Probe = &|r| r.mix(Strip::Mtx(1), M::Pfl, Side::L);
    let aux: Probe = &|r| r.input(40, I::Pfl, Side::L);
    for (name, probe) in [("main 1", main), ("mtx 1", mtx), ("aux 1", aux)] {
        let alone = t.gain(&[1], probe, reference);
        let keyed = t.gain(&[1, 2], probe, reference);
        assert!(
            alone - keyed > 6.0,
            "{name}: alone {alone:.1} dB, keyed by bus 5 {keyed:.1} dB"
        );
    }
}

#[test]
fn key_listen_solos_the_key() {
    let mut t = T::new(|s| {
        ducker(s);
        sett(s, "ch/1/gatesc/src", "CH.2");
        setf(s, "cfg/mon/1/lvl", 0.0);
        setf(s, "cfg/mon/1/pfldim", 0.0);
    });
    let mon: Probe = &|r| r.port(r.nodes.monitor_port(1, Side::L));
    let ch2: Probe = &|r| r.input(1, I::Pfl, Side::L);
    // USB 2 only: ch1 is silent; listening to ch1's key plays ch2 (its IN tap)
    t.set("ch/1/gatesc/$solo", "1");
    let [listen, key] = t.measure(&[2], [mon, ch2]);
    assert!(
        (listen - key).abs() < 0.5,
        "key listen: monitor {listen:.1} dB, ch2 {key:.1} dB"
    );
    assert_eq!(getf(&t.store, "$stat/solo"), 1.0, "key listen is a solo");
    // the key filter shapes what is heard
    t.set("ch/1/gatesc/type", "HP12");
    t.set("ch/1/gatesc/f", "15000");
    let [filtered] = t.measure(&[2], [mon]);
    assert!(listen - filtered > 20.0, "filtered key: {filtered:.1} dB");
    // the dynamics slot's key listen (SELF key = ch1's own, silent here)
    t.set("ch/1/gatesc/$solo", "0");
    t.set("ch/1/dynsc/$solo", "1");
    let [selfkey] = t.measure(&[2], [mon]);
    assert!(selfkey < -100.0, "dyn key listen, SELF: {selfkey:.1} dB");
    // off: no solo, the monitor plays its source (main 1 without ch2 = silent here)
    t.set("ch/1/dynsc/$solo", "0");
    t.set("ch/2/main/1/on", "0");
    let [off] = t.measure(&[2], [mon]);
    assert!(off < -100.0, "listen off: {off:.1} dB");
    assert_eq!(getf(&t.store, "$stat/solo"), 0.0);
}
