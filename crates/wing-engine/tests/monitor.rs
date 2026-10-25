//! T038: DCAs, mute groups, solo (PFL/AFL, exclusive/additive, clear, SIP, indicators) and the
//! monitor section (solo priority, srcmix, dim, mono, EQ, delay, limiter).

mod common;

use common::rig::*;
use std::sync::Arc;

use wing_engine::mix::{
    no_processing, InputSignal as I, MixConfig, MixLogic, MixSignal as M, MonitorLevels,
};
use wing_engine::routing::{Side, Strip};
use wing_model::ParamStore;

const SETTLE: usize = 80;

#[track_caller]
fn assert_gain(x: &[f32], y: &[f32], g: f64, what: &str) {
    let (got, res) = fit(x, y);
    if g.abs() < 1e-9 {
        let peak = y.iter().fold(0.0f32, |a, &v| a.max(v.abs()));
        assert!(peak < 1e-6, "{what}: expected silence, peak {peak}");
        return;
    }
    assert!(
        (got / g - 1.0).abs() < 1e-3 && res < 1e-3,
        "{what}: gain {got} ({:.3} dB) expected {g} ({:.3} dB), residual {res}",
        db(got),
        db(g)
    );
}

/// ch1 <- USB 1 at 0 dB into main 1 (0 dB), ch2 <- USB 2 at -oo; monitor levels 0 dB (the
/// factory scene leaves them at -oo); the logic hooked to the store.
fn setup() -> (ParamStore, MixLogic) {
    let mut store = ParamStore::new();
    let logic = MixLogic::new(&store);
    store.add_hook(logic.change_hook());
    connect_usb(&store, "ch/1", 1);
    connect_usb(&store, "ch/2", 2);
    setf(&store, "ch/1/fdr", 0.0);
    setf(&store, "main/1/fdr", 0.0);
    setf(&store, "cfg/mon/1/lvl", 0.0);
    setf(&store, "cfg/mon/2/lvl", 0.0);
    // factory DCA faders are -oo: put the ones used at 0 dB
    for k in 1..=4 {
        setf(&store, &format!("dca/{k}/fdr"), 0.0);
    }
    (store, logic)
}

struct T {
    store: ParamStore,
    logic: MixLogic,
    r: Rig,
}

impl T {
    fn new() -> Self {
        Self::with_knobs(None)
    }

    fn with_knobs(knobs: Option<Arc<MonitorLevels>>) -> Self {
        Self::with(knobs, |_| {})
    }

    /// [`T::with_knobs`] with `extra` applied to the store before the graph is built.
    fn with(knobs: Option<Arc<MonitorLevels>>, extra: impl FnOnce(&ParamStore)) -> Self {
        let (store, mut logic) = setup();
        extra(&store);
        logic.refresh(&store);
        // bare mix (no bus/main strip processing): gains are fitted sample by sample against the
        // channel PFL, which the main strip latency (T03E) would shift
        let cfg = MixConfig {
            logic: Some(logic.shared()),
            hooks: Some(no_processing()),
            monitor_levels: knobs,
            ..MixConfig::default()
        };
        let r = rig(&store, cfg, 2);
        let mut t = T { store, logic, r };
        t.run(SETTLE);
        t
    }

    /// Control refresh (twice: indicator write-back settles), then `n` cycles of USB 1/2.
    fn run(&mut self, n: usize) {
        self.logic.refresh(&self.store);
        self.logic.refresh(&self.store);
        self.r.cycles(n, &[1, 2]);
    }

    fn mon(&self, m: usize, side: Side) -> Vec<f32> {
        self.r.port(self.r.nodes.monitor_port(m, side))
    }

    fn main(&self, side: Side) -> Vec<f32> {
        self.r.mix(Strip::Main(1), M::Post, side)
    }

    fn pfl(&self, strip: usize) -> Vec<f32> {
        self.r.input(strip, I::Pfl, Side::L)
    }

    fn set(&self, p: &str, v: &str) {
        sett(&self.store, p, v);
    }
}

fn add(a: &[f32], b: &[f32]) -> Vec<f32> {
    a.iter().zip(b).map(|(x, y)| x + y).collect()
}

fn pan_c() -> f64 {
    std::f64::consts::FRAC_1_SQRT_2
}

// ---- DCA / mute groups -------------------------------------------------------------------------

#[test]
fn dca_levels_sum_in_db_and_follow_membership() {
    let mut t = T::new();
    let g_main = |t: &T| fit(&t.pfl(0), &t.main(Side::L)).0;
    assert_gain(&t.pfl(0), &t.main(Side::L), pan_c(), "no DCA");

    t.set("dca/1/fdr", "-6");
    t.set("dca/2/fdr", "-4.5");
    t.set("ch/1/tags", "#D1");
    t.run(SETTLE);
    assert_gain(&t.pfl(0), &t.main(Side::L), pan_c() * lin(-6.0), "DCA1 -6");
    t.set("ch/1/tags", "#D1,#D2,INSTRUM");
    t.run(SETTLE);
    assert_gain(&t.pfl(0), &t.main(Side::L), pan_c() * lin(-10.5), "DCA1+2");
    // own fader adds in dB, total clamped to +10
    t.set("ch/1/fdr", "-3");
    t.run(SETTLE);
    assert!((db(g_main(&t) / pan_c()) - -13.5).abs() < 0.01);
    t.set("ch/1/fdr", "10");
    t.set("dca/1/fdr", "8");
    t.set("dca/2/fdr", "6");
    t.run(SETTLE);
    assert!(
        (db(g_main(&t) / pan_c()) - 10.0).abs() < 0.01,
        "clamped +10"
    );
    // a DCA at -oo silences its members; ch2 is not a member
    t.set("dca/2/fdr", "-144");
    t.run(SETTLE);
    assert_gain(&t.pfl(0), &t.main(Side::L), 0.0, "DCA -oo");
    t.set("ch/1/tags", "");
    t.set("ch/1/fdr", "0");
    t.run(SETTLE);
    assert_gain(&t.pfl(0), &t.main(Side::L), pan_c(), "left the DCA");

    // busses are members too
    t.set("ch/2/send/3/on", "1");
    t.set("ch/2/send/3/lvl", "0");
    t.set("ch/2/send/3/mode", "PRE");
    t.set("bus/3/fdr", "0");
    t.set("bus/3/tags", "#D3");
    t.set("dca/3/fdr", "-12");
    t.run(SETTLE);
    let pre = t.r.input(1, I::Pre, Side::L);
    let bus = t.r.mix(Strip::Bus(3), M::Post, Side::L);
    let (g, _) = fit(&pre, &bus);
    assert!((db(g / pan_c()) - -12.0).abs() < 0.01, "bus DCA {}", db(g));
}

#[test]
fn dca_moves_are_smoothed() {
    let mut t = T::new();
    t.set("ch/1/tags", "#D1");
    t.run(SETTLE);
    // jump DCA 1 by 30 dB and look at the gain trajectory block by block
    t.set("dca/1/fdr", "-30");
    t.logic.refresh(&t.store);
    let mut gains = Vec::new();
    for _ in 0..20 {
        t.r.cycle(&[1, 2]);
        gains.push(fit(&t.pfl(0), &t.main(Side::L)).0 / pan_c());
    }
    assert!(gains[0] > 0.7, "first block jumped: {}", gains[0]);
    for w in gains.windows(2) {
        assert!(w[1] <= w[0] + 1e-6 && w[0] - w[1] < 0.3, "{gains:?}");
    }
    assert!((gains[19] - lin(-30.0)).abs() < 0.01);
}

#[test]
fn mute_groups_and_dca_mutes() {
    let mut t = T::new();
    t.set("ch/2/fdr", "0");
    let afl1 = |t: &T| t.r.input(0, I::Afl, Side::L);
    t.set("ch/1/tags", "#M2,#D4");
    t.set("mgrp/1/mute", "1");
    t.run(SETTLE);
    assert_gain(&t.pfl(0), &afl1(&t), 1.0, "other group muted");
    t.set("mgrp/2/mute", "1");
    t.run(SETTLE);
    assert_gain(&t.pfl(0), &afl1(&t), 0.0, "group 2 muted");
    assert_gain(
        &t.pfl(1),
        &t.r.input(1, I::Afl, Side::L),
        1.0,
        "ch2 not a member",
    );
    t.set("mgrp/2/mute", "0");
    t.run(SETTLE);
    assert_gain(&t.pfl(0), &afl1(&t), 1.0, "group 2 unmuted");
    // DCA mute with cfg/dcamgrp on (factory default): a full mute, like a mute group
    t.set("dca/4/mute", "1");
    t.run(SETTLE);
    assert_gain(&t.pfl(0), &afl1(&t), 0.0, "DCA mute");
}

/// T03F (user-confirmed WING behaviour): a DCA mute always mutes its members' main sends;
/// `cfg/dcamgrp` on additionally mutes everything else (bus sends incl. PRE sends: a full mute).
/// Regression: with `cfg/dcamgrp` off (the user's scene) DCA mutes did nothing.
#[test]
fn dca_mute_mutes_main_sends_and_dcamgrp_makes_it_a_full_mute() {
    let mut t = T::new();
    // ch1 (member of DCA 5) -> main 1 and PRE + POST sends to busses 1/2; bus 3 (member of
    // DCA 5) <- ch2 PRE send, bus 3 -> main 1
    t.set("ch/2/fdr", "-144");
    t.set("ch/1/tags", "#D5");
    t.set("dca/5/fdr", "0");
    for (b, mode) in [(1, "PRE"), (2, "POST")] {
        t.set(&format!("ch/1/send/{b}/on"), "1");
        t.set(&format!("ch/1/send/{b}/lvl"), "0");
        t.set(&format!("ch/1/send/{b}/mode"), mode);
        t.set(&format!("bus/{b}/fdr"), "0");
    }
    t.set("cfg/dcamgrp", "0");
    t.run(SETTLE);
    let bus = |t: &T, b: u8| t.r.mix(Strip::Bus(b), M::Post, Side::L);
    let afl1 = |t: &T| t.r.input(0, I::Afl, Side::L);
    let pre1 = |t: &T| t.r.input(0, I::Pre, Side::L);
    assert_gain(&t.pfl(0), &t.main(Side::L), pan_c(), "unmuted: main");
    let (g_pre, _) = fit(&pre1(&t), &bus(&t, 1));
    let (g_post, _) = fit(&afl1(&t), &bus(&t, 2));
    assert!(
        g_pre > 0.1 && g_post > 0.1,
        "unmuted sends {g_pre} {g_post}"
    );

    // dcamgrp off: main send muted, the strip itself and its bus sends are not
    t.set("dca/5/mute", "1");
    t.run(SETTLE);
    assert_gain(
        &t.pfl(0),
        &t.main(Side::L),
        0.0,
        "dcamgrp off: main send muted",
    );
    assert_gain(&t.pfl(0), &afl1(&t), 1.0, "dcamgrp off: fader not muted");
    assert_gain(&pre1(&t), &bus(&t, 1), g_pre, "dcamgrp off: PRE bus send");
    assert_gain(&afl1(&t), &bus(&t, 2), g_post, "dcamgrp off: POST bus send");
    assert!(t.logic.state().flags[0] & wing_engine::mix::logic::flag::MUTE == 0);

    // a bus member: its main send is muted, the bus output is not
    t.set("ch/1/tags", "");
    t.set("ch/1/main/1/on", "0");
    t.set("ch/2/fdr", "0");
    t.set("ch/2/main/1/on", "0");
    t.set("ch/2/send/3/on", "1");
    t.set("ch/2/send/3/lvl", "0");
    t.set("ch/2/send/3/mode", "PRE");
    t.set("bus/3/fdr", "0");
    t.set("bus/3/main/1/on", "1");
    t.set("bus/3/main/1/lvl", "0");
    t.set("bus/3/tags", "#D5");
    t.run(SETTLE);
    let bus3 = bus(&t, 3);
    assert!(bus3.iter().any(|x| x.abs() > 1e-3), "bus 3 output passes");
    assert_gain(&bus3, &t.main(Side::L), 0.0, "bus member: main send muted");
    t.set("dca/5/mute", "0");
    t.run(SETTLE);
    assert_gain(&bus(&t, 3), &t.main(Side::L), 1.0, "bus member: unmuted");
    t.set("bus/3/tags", "");
    t.set("bus/3/main/1/on", "0");
    t.set("ch/2/fdr", "-144");
    t.set("ch/1/main/1/on", "1");
    t.set("ch/1/tags", "#D5");

    // dcamgrp on: a full mute (main, PRE and POST bus sends)
    t.set("cfg/dcamgrp", "1");
    t.set("dca/5/mute", "1");
    t.run(SETTLE);
    assert_gain(&t.pfl(0), &t.main(Side::L), 0.0, "dcamgrp on: main");
    assert_gain(&t.pfl(0), &afl1(&t), 0.0, "dcamgrp on: fader");
    assert_gain(&t.pfl(0), &bus(&t, 1), 0.0, "dcamgrp on: PRE send");
    assert_gain(&t.pfl(0), &bus(&t, 2), 0.0, "dcamgrp on: POST send");

    // unmute restores everything
    t.set("dca/5/mute", "0");
    t.run(SETTLE);
    assert_gain(&t.pfl(0), &t.main(Side::L), pan_c(), "unmuted again: main");
    assert_gain(&pre1(&t), &bus(&t, 1), g_pre, "unmuted again: PRE send");
}

// ---- solo ---------------------------------------------------------------------------------------

#[test]
fn monitor_follows_source_without_solo() {
    let t = T::new();
    for side in [Side::L, Side::R] {
        assert_gain(&t.main(side), &t.mon(1, side), 1.0, "mon 1 = main 1");
        assert_gain(&t.main(side), &t.mon(2, side), 1.0, "mon 2 = main 1");
    }
    assert!(!t.logic.state().any_solo);
}

#[test]
fn pfl_and_afl_taps() {
    let mut t = T::new();
    t.set("ch/1/fdr", "-6");
    t.set("cfg/mon/1/pfldim", "0");
    t.set("ch/1/$solo", "1");
    t.run(SETTLE);
    assert!(t.logic.state().any_solo);
    // factory chtap = PFL: monitor = pre-fader signal only (srcmix -oo replaces main)
    assert_gain(&t.pfl(0), &t.mon(1, Side::L), 1.0, "PFL");
    assert_gain(&t.pfl(0), &t.mon(1, Side::R), 1.0, "PFL R");
    // PFL dim
    t.set("cfg/mon/1/pfldim", "12");
    t.run(SETTLE);
    assert_gain(&t.pfl(0), &t.mon(1, Side::L), lin(-12.0), "PFL dim 12");
    // AFL: post fader (-6), before pan
    t.set("cfg/solo/chtap", "AFL");
    t.run(SETTLE);
    let afl = t.r.input(0, I::Afl, Side::L);
    assert_gain(&afl, &t.mon(1, Side::L), 1.0, "AFL");
    assert_gain(&t.pfl(0), &t.mon(1, Side::L), lin(-6.0), "AFL = fader");
    // bus solo uses bustap (factory AFL): bus post
    t.set("ch/1/$solo", "0");
    t.set("ch/1/send/2/on", "1");
    t.set("ch/1/send/2/lvl", "0");
    t.set("bus/2/fdr", "-3");
    t.set("bus/2/$solo", "1");
    t.run(SETTLE);
    let post = t.r.mix(Strip::Bus(2), M::Post, Side::L);
    assert_gain(&post, &t.mon(1, Side::L), 1.0, "bus AFL");
    t.set("cfg/solo/bustap", "PFL");
    t.run(SETTLE);
    let pfl = t.r.mix(Strip::Bus(2), M::Pfl, Side::L);
    assert_gain(&pfl, &t.mon(1, Side::L), lin(-12.0), "bus PFL");
}

/// T03G: channel, aux, bus, main, matrix and DCA solo all reach the monitor(s) `cfg/solo/mon`
/// names (PH = monitor 1, SPK = monitor 2, PH+SPK = both), whatever the strip's MON BUS; with a
/// listen mask (the monitor the daemon's host bridge plays) a solo that misses it reaches it too.
#[test]
fn every_solo_kind_reaches_the_cfg_solo_monitor_and_the_listened_one() {
    let mut t = T::with(None, |s| {
        connect_usb(s, "aux/1", 2);
        setf(s, "aux/1/fdr", -5.0);
    });
    for tap in ["chtap", "bustap", "maintap", "mtxtap"] {
        t.set(&format!("cfg/solo/{tap}"), "PFL");
    }
    for m in [1, 2] {
        t.set(&format!("cfg/mon/{m}/pfldim"), "0");
    }
    // ch1 -> bus 2 (send 0 dB) and main 1; main 1 -> mtx 1 (MX1 0 dB); faders off unity so
    // PFL and post differ
    t.set("ch/1/send/2/on", "1");
    t.set("ch/1/send/2/lvl", "0");
    t.set("bus/2/fdr", "-4");
    t.set("main/1/fdr", "-2");
    t.set("main/1/send/MX1/on", "1");
    t.set("main/1/send/MX1/lvl", "0");
    t.set("mtx/1/fdr", "-3");
    t.set("ch/1/tags", "#D3");
    // every strip on monitor bus B: must not matter
    for p in ["ch/1", "aux/1", "bus/2", "main/1", "mtx/1", "dca/3"] {
        t.set(&format!("{p}/mon"), "B");
    }
    t.run(SETTLE);

    let pfl = |t: &T, what: &str| match what {
        "ch/1" | "dca/3" => t.pfl(0),
        "aux/1" => t.r.input(40, I::Pfl, Side::L),
        "bus/2" => t.r.mix(Strip::Bus(2), M::Pfl, Side::L),
        "main/1" => t.r.mix(Strip::Main(1), M::Pfl, Side::L),
        "mtx/1" => t.r.mix(Strip::Mtx(1), M::Pfl, Side::L),
        _ => unreachable!(),
    };
    // (cfg/solo/mon, listen mask, solo on monitor 1, solo on monitor 2)
    for (mon, listen, on1, on2) in [
        ("PH", 0, true, false),
        ("SPK", 0, false, true),
        ("PH+SPK", 0, true, true),
        ("SPK", 1, true, true), // the user's case: host plays mon/1
        ("PH", 1, true, false),
        ("PH", 2, true, true),
    ] {
        t.set("cfg/solo/mon", mon);
        t.logic.set_listen(listen);
        for strip in ["ch/1", "aux/1", "bus/2", "main/1", "mtx/1", "dca/3"] {
            t.set(&format!("{strip}/$solo"), "1");
            t.run(SETTLE);
            let what = format!("{strip} solo, cfg/solo/mon {mon}, listen {listen}");
            let want = pfl(&t, strip);
            assert!(want.iter().any(|x| x.abs() > 1e-3), "{what}: no signal");
            for (m, on) in [(1, on1), (2, on2)] {
                let want = if on { want.clone() } else { t.main(Side::L) };
                assert_gain(&want, &t.mon(m, Side::L), 1.0, &format!("{what}: mon {m}"));
            }
            t.set(&format!("{strip}/$solo"), "0");
            t.run(SETTLE);
            assert!(!t.logic.state().any_solo, "{what}: unsolo");
            for m in [1, 2] {
                assert_gain(&t.main(Side::L), &t.mon(m, Side::L), 1.0, "unsoloed");
            }
        }
    }
}

/// A 50 Hz sine on USB 1 and a 70 Hz one on USB 2 (slow: a gain jump is a click).
fn sines(r: &mut Rig) {
    for ch in 0..48 {
        let t = r.t;
        let f = match ch {
            0 => 50.0,
            1 => 70.0,
            _ => 0.0,
        };
        for (i, s) in r.e.external_mut(ch).iter_mut().enumerate() {
            let n = (t + i as u64) as f32;
            *s = 0.5 * (std::f32::consts::TAU * f * n / 48_000.0).sin();
        }
    }
    r.t += BLOCK as u64;
}

fn max_step(x: &[f32]) -> f32 {
    x.windows(2).fold(0.0, |a, w| a.max((w[1] - w[0]).abs()))
}

#[test]
fn solo_priority_switches_without_clicks() {
    let mut t = T::new();
    t.set("ch/2/fdr", "0");
    t.set("ch/1/fdr", "-144"); // ch1 is not in the main mix: soloing it changes the monitor a lot
    t.set("cfg/mon/1/pfldim", "0");
    t.run(SETTLE);
    let cycle_mon = |t: &mut T, n: usize, out: &mut Vec<f32>| {
        for _ in 0..n {
            sines(&mut t.r);
            t.r.pipe.apply(&mut t.r.e);
            t.r.e.process_cycle();
            out.extend(t.mon(1, Side::L));
        }
    };
    let mut steady = Vec::new();
    cycle_mon(&mut t, 20, &mut steady);
    let base = max_step(&steady);
    assert!(base > 0.001, "monitor carries main 1");

    // solo in, hold, clear (via $stat/solo), hold
    let mut y = Vec::new();
    t.set("ch/1/$solo", "1");
    t.logic.refresh(&t.store);
    cycle_mon(&mut t, 40, &mut y);
    assert!(t.logic.state().any_solo);
    let solo_part = y[y.len() - 640..].to_vec();
    t.set("$stat/solo", "0");
    t.logic.refresh(&t.store);
    t.logic.refresh(&t.store);
    assert_eq!(getf(&t.store, "ch/1/$solo"), 0.0, "clear solo");
    cycle_mon(&mut t, 40, &mut y);
    assert!(!t.logic.state().any_solo);

    let step = max_step(&y);
    assert!(
        step < 3.0 * base.max(max_step(&solo_part)),
        "click: max step {step} vs steady {base}"
    );
    // and the levels really switched: solo = ch1 PFL (50 Hz, 0.5 peak), back to main (ch2 only)
    let peak = |v: &[f32]| v.iter().fold(0.0f32, |a, &x| a.max(x.abs()));
    assert!(
        (peak(&solo_part) - 0.5).abs() < 0.02,
        "{}",
        peak(&solo_part)
    );
    let last = &y[y.len() - 640..];
    assert!(
        (peak(last) - 0.5 * pan_c() as f32).abs() < 0.03,
        "{}",
        peak(last)
    );
}

#[test]
fn srcmix_mixes_source_under_solo() {
    let mut t = T::new();
    t.set("cfg/mon/1/pfldim", "0");
    t.set("cfg/mon/1/srcmix", "-10");
    t.set("ch/2/$solo", "1");
    t.run(SETTLE);
    let want = add(
        &t.main(Side::L)
            .iter()
            .map(|x| x * lin(-10.0) as f32)
            .collect::<Vec<_>>(),
        &t.pfl(1),
    );
    assert_gain(&want, &t.mon(1, Side::L), 1.0, "srcmix + solo");
    // monitor 2 is not a solo destination (cfg/solo/mon = PH): it keeps the source at srclvl
    assert_gain(&t.main(Side::L), &t.mon(2, Side::L), 1.0, "mon 2");
    t.set("cfg/solo/mon", "PH+SPK");
    t.set("cfg/mon/2/pfldim", "0");
    t.run(SETTLE);
    assert_gain(&t.pfl(1), &t.mon(2, Side::L), 1.0, "mon 2 solo");
    // the strip's MON BUS does not gate the destination (T03G)
    t.set("ch/2/mon", "B");
    t.run(SETTLE);
    assert_gain(&t.pfl(1), &t.mon(2, Side::L), 1.0, "strip mon B: mon 2");
    let want = add(
        &t.main(Side::L)
            .iter()
            .map(|x| x * lin(-10.0) as f32)
            .collect::<Vec<_>>(),
        &t.pfl(1),
    );
    assert_gain(&want, &t.mon(1, Side::L), 1.0, "strip mon B: mon 1");
    t.set("ch/2/mon", "A");
    // srclvl only applies without solo
    t.set("ch/2/$solo", "0");
    t.set("cfg/mon/1/srclvl", "-6");
    t.run(SETTLE);
    assert_gain(&t.main(Side::L), &t.mon(1, Side::L), lin(-6.0), "srclvl");
}

#[test]
fn exclusive_additive_dca_solo_and_indicators() {
    let mut t = T::new();
    assert_eq!(getf(&t.store, "$ctl/cfg/soloexcl"), 1.0);
    t.set("ch/1/$solo", "1");
    t.run(1);
    t.set("ch/2/$solo", "1");
    t.run(1);
    assert_eq!(getf(&t.store, "ch/1/$solo"), 0.0, "exclusive");
    assert_eq!(getf(&t.store, "ch/2/$solo"), 1.0);
    assert_eq!(getf(&t.store, "ch/2/$sololed"), 1.0);
    assert_eq!(getf(&t.store, "ch/1/$sololed"), 0.0);
    assert_eq!(getf(&t.store, "$stat/solo"), 1.0);

    t.set("$ctl/cfg/soloexcl", "0");
    t.set("ch/1/$solo", "1");
    t.set("cfg/mon/1/pfldim", "0");
    t.run(SETTLE);
    assert_eq!(getf(&t.store, "ch/2/$solo"), 1.0, "additive");
    let both = add(&t.pfl(0), &t.pfl(1));
    assert_gain(&both, &t.mon(1, Side::L), 1.0, "two solos");

    // clear solo, then a DCA solo: members light with 2
    t.set("$stat/solo", "0");
    t.run(1);
    assert_eq!(getf(&t.store, "ch/1/$solo"), 0.0);
    assert_eq!(getf(&t.store, "ch/2/$solo"), 0.0);
    assert_eq!(getf(&t.store, "$stat/solo"), 0.0);
    t.set("ch/2/tags", "#D3");
    t.set("dca/3/$solo", "1");
    t.run(SETTLE);
    assert_eq!(getf(&t.store, "ch/2/$sololed"), 2.0);
    assert_eq!(getf(&t.store, "dca/3/$sololed"), 1.0);
    assert_eq!(getf(&t.store, "$stat/solo"), 1.0);
    assert_gain(&t.pfl(1), &t.mon(1, Side::L), 1.0, "DCA solo");
}

#[test]
fn solo_in_place_mutes_others() {
    let mut t = T::new();
    t.set("ch/2/fdr", "0");
    t.set("cfg/solo/mode", "SIP");
    t.set("ch/1/$solo", "1");
    t.run(SETTLE);
    // main 1 now carries ch1 only
    assert_gain(&t.pfl(0), &t.main(Side::L), pan_c(), "SIP keeps ch1");
    let afl2 = t.r.input(1, I::Afl, Side::L);
    assert!(afl2.iter().all(|&x| x == 0.0), "SIP mutes ch2");
    t.set("ch/2/solosafe", "1");
    t.run(SETTLE);
    let want = add(&t.pfl(0), &t.pfl(1))
        .iter()
        .map(|x| x * pan_c() as f32)
        .collect::<Vec<_>>();
    assert_gain(&want, &t.main(Side::L), 1.0, "solo safe");
    t.set("cfg/solo/mode", "LIVE");
    t.set("ch/2/solosafe", "0");
    t.run(SETTLE);
    let want = add(&t.pfl(0), &t.pfl(1))
        .iter()
        .map(|x| x * pan_c() as f32)
        .collect::<Vec<_>>();
    assert_gain(&want, &t.main(Side::L), 1.0, "LIVE does not mute");
}

// ---- monitor processing -------------------------------------------------------------------------

/// T03F: with knobs, the monitor level is the knob's; `cfg/mon/*/lvl` sets (e.g. WING-Edit's
/// scene push of -oo) do not touch it; knob moves are smoothed.
#[test]
fn monitor_level_knobs_override_lvl() {
    let knobs = Arc::new(MonitorLevels::default());
    let mut t = T::with_knobs(Some(knobs.clone()));
    t.set("cfg/mon/1/lvl", "-144");
    t.set("cfg/mon/2/lvl", "-144");
    t.run(SETTLE);
    let ml = t.main(Side::L);
    assert_gain(&ml, &t.mon(1, Side::L), 1.0, "knob A 0 dB, lvl -oo ignored");
    assert_gain(&ml, &t.mon(2, Side::L), 1.0, "knob B 0 dB");
    assert_eq!(knobs.set(1, -6.0), -6.0);
    assert_eq!(knobs.set(2, 99.0), 10.0, "clamped");
    t.r.cycle(&[1, 2]);
    let (g, _) = fit(&t.main(Side::L), &t.mon(1, Side::L));
    assert!(g > lin(-6.0) + 0.01, "ramped, not jumped: {g}");
    t.run(SETTLE);
    let ml = t.main(Side::L);
    assert_gain(&ml, &t.mon(1, Side::L), lin(-6.0), "knob A -6");
    assert_gain(&ml, &t.mon(2, Side::L), lin(10.0), "knob B +10");
    knobs.set(1, f32::NEG_INFINITY);
    t.run(SETTLE);
    assert_gain(&ml, &t.mon(1, Side::L), 0.0, "knob A -oo");
}

#[test]
fn monitor_level_dim_mono_pan_inv() {
    let mut t = T::new();
    t.set("ch/2/fdr", "0");
    t.set("ch/1/pan", "-100"); // main L = ch1, main R = ch2
    t.set("ch/2/pan", "100");
    t.set("cfg/mon/1/lvl", "-3");
    t.run(SETTLE);
    let ml = t.main(Side::L);
    assert_gain(&ml, &t.mon(1, Side::L), lin(-3.0), "lvl");
    t.set("cfg/mon/1/dim", "20");
    t.set("cfg/solo/$dim", "1");
    t.run(SETTLE);
    let (ml, mr) = (t.main(Side::L), t.main(Side::R));
    assert_gain(&ml, &t.mon(1, Side::L), lin(-23.0), "dim");
    assert_gain(
        &mr,
        &t.mon(2, Side::R),
        lin(-20.0),
        "dim on mon 2 (its own dim)",
    );
    t.set("cfg/solo/$dim", "0");
    t.set("cfg/solo/$mono", "1");
    t.run(SETTLE);
    let (ml, mr) = (t.main(Side::L), t.main(Side::R));
    let mono: Vec<f32> = ml.iter().zip(&mr).map(|(a, b)| 0.5 * (a + b)).collect();
    assert_gain(&mono, &t.mon(1, Side::L), lin(-3.0), "mono L");
    assert_gain(&mono, &t.mon(1, Side::R), lin(-3.0), "mono R");
    t.set("cfg/solo/$mono", "0");
    t.set("cfg/mon/1/inv", "1");
    t.set("cfg/mon/1/pan", "100");
    t.run(SETTLE);
    let (ml, mr) = (t.main(Side::L), t.main(Side::R));
    assert_gain(&ml, &t.mon(1, Side::L), 0.0, "balance right: L off");
    assert_gain(&mr, &t.mon(1, Side::R), -lin(-3.0), "inverted R");
}

#[test]
fn monitor_eq_delay_and_limiter() {
    let mut t = T::new();
    t.set("ch/2/fdr", "-144");
    t.set("cfg/mon/1/eq/on", "1");
    t.set("cfg/mon/1/eq/3g", "6");
    t.set("cfg/mon/1/eq/3f", "1000");
    t.set("cfg/mon/1/eq/3q", "2");
    t.run(SETTLE);
    // 1 kHz and 100 Hz sines on USB 1
    let run_sine = |t: &mut T, f: f32| -> (f32, f32) {
        let (mut a, mut b) = (0.0f32, 0.0f32);
        for c in 0..120 {
            let base = t.r.t;
            for (i, s) in t.r.e.external_mut(0).iter_mut().enumerate() {
                let n = (base + i as u64) as f32;
                *s = 0.25 * (std::f32::consts::TAU * f * n / 48_000.0).sin();
            }
            t.r.t += BLOCK as u64;
            t.r.pipe.apply(&mut t.r.e);
            t.r.e.process_cycle();
            if c >= 60 {
                a = a.max(t.main(Side::L).iter().fold(0.0, |m, &x| m.max(x.abs())));
                b = b.max(t.mon(1, Side::L).iter().fold(0.0, |m, &x| m.max(x.abs())));
            }
        }
        (a, b)
    };
    let (a, b) = run_sine(&mut t, 1000.0);
    assert!(
        (db((b / a) as f64) - 6.0).abs() < 0.2,
        "EQ +6 at 1k: {}",
        db((b / a) as f64)
    );
    let (a, b) = run_sine(&mut t, 100.0);
    assert!(
        db((b / a) as f64).abs() < 0.3,
        "EQ flat at 100 Hz: {}",
        db((b / a) as f64)
    );

    // limiter at -20 dBFS: 1 kHz +6 dB EQ'd peak ~0.35 -> held near 0.1
    t.set("cfg/mon/1/lim", "-20");
    t.run(1);
    let (_, b) = run_sine(&mut t, 1000.0);
    assert!(b <= 0.1 + 1e-4, "limited peak {b}");

    // delay: 34.3 m = 4800 samples (100 ms)
    t.set("cfg/mon/1/lim", "0");
    t.set("cfg/mon/1/eq/on", "0");
    t.set("cfg/mon/1/dly/on", "1");
    t.set("cfg/mon/1/dly/m", "34.3");
    t.run(SETTLE);
    let mut main = Vec::new();
    let mut mon = Vec::new();
    for _ in 0..120 {
        t.r.cycle(&[1]);
        main.extend(t.main(Side::L));
        mon.extend(t.mon(1, Side::L));
    }
    assert_gain(&main[..main.len() - 4800], &mon[4800..], 1.0, "delay 4800");
}

#[test]
fn monitor_and_solo_meters() {
    use wing_engine::meters::{solo_cell, source_cell, LevelBank, PortMeters};
    use wing_engine::routing::{SourceAddr, SrcGroup};
    let mut t = T::new();
    let bank = LevelBank::shared();
    let pm = PortMeters::for_mix(bank.clone(), &t.r.nodes);
    let mon_leg = |n| source_cell(SourceAddr::new(SrcGroup::Mon, n).unwrap());
    pm.capture(&t.r.e);
    let peak = |v: &[f32]| v.iter().fold(0.0f32, |a, &x| a.max(x.abs()));
    assert_eq!(bank.take(mon_leg(1)), peak(&t.mon(1, Side::L)));
    assert_eq!(bank.take(mon_leg(4)), peak(&t.mon(2, Side::R)));
    assert_eq!(bank.take(solo_cell(0)), 0.0, "no solo");
    t.set("cfg/mon/1/pfldim", "0");
    t.set("ch/1/$solo", "1");
    t.run(SETTLE);
    pm.capture(&t.r.e);
    assert_eq!(bank.take(solo_cell(0)), peak(&t.pfl(0)));
    assert_eq!(bank.take(mon_leg(1)), peak(&t.mon(1, Side::L)));
}
