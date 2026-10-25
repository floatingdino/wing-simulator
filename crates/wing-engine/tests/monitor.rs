//! T038: DCAs, mute groups, solo (PFL/AFL, exclusive/additive, clear, SIP, indicators) and the
//! monitor section (solo priority, srcmix, dim, mono, EQ, delay, limiter).

mod common;

use common::rig::*;
use wing_engine::mix::{no_processing, InputSignal as I, MixConfig, MixLogic, MixSignal as M};
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
        let (store, mut logic) = setup();
        logic.refresh(&store);
        // bare mix (no bus/main strip processing): gains are fitted sample by sample against the
        // channel PFL, which the main strip latency (T03E) would shift
        let cfg = MixConfig {
            logic: Some(logic.shared()),
            hooks: Some(no_processing()),
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
    // DCA mute: mutes members while cfg/dcamgrp (default on)
    t.set("dca/4/mute", "1");
    t.run(SETTLE);
    assert_gain(&t.pfl(0), &afl1(&t), 0.0, "DCA mute");
    t.set("cfg/dcamgrp", "0");
    t.run(SETTLE);
    assert_gain(&t.pfl(0), &afl1(&t), 1.0, "dcamgrp off");
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
    t.set("ch/2/mon", "A+B");
    t.run(SETTLE);
    assert_gain(&t.pfl(1), &t.mon(2, Side::L), 1.0, "mon 2 solo");
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
