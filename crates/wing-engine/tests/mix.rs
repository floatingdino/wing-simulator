//! T036: fader/pan/width laws, mute, send taps and modes, bus->bus / main / matrix routing, gain
//! smoothing, tap wiring to the USB outputs, re-inputs, and the PUBLIC.snap main mix.

mod common;

use common::rig::*;
use wing_engine::mix::{bus_tiers, BusOrder, InputSignal as I, MixConfig, MixSignal as M, PanLaw};
use wing_engine::routing::{Side, Strip};
use wing_engine::Port;
use wing_model::ParamStore;

const SETTLE: usize = 80;
const C: f64 = std::f64::consts::FRAC_1_SQRT_2;

fn cp(pan: f64) -> (f64, f64) {
    let t = (pan / 100.0 + 1.0) * std::f64::consts::FRAC_PI_4;
    (t.cos(), t.sin())
}

/// Assert `y = g * x` per side with `g` within 1e-4 relative and a clean fit.
#[track_caller]
fn assert_gain(x: &[f32], y: &[f32], g: f64, what: &str) {
    let (got, res) = fit(x, y);
    if g.abs() < 1e-9 {
        let peak = y.iter().fold(0.0f32, |a, &v| a.max(v.abs()));
        assert!(peak < 1e-6, "{what}: expected silence, peak {peak}");
        return;
    }
    assert!(
        (got / g - 1.0).abs() < 1e-4 && res < 1e-4,
        "{what}: gain {got} ({:.3} dB) expected {g} ({:.3} dB), residual {res}",
        db(got),
        db(g)
    );
}

fn base_store() -> ParamStore {
    let store = ParamStore::new();
    connect_usb(&store, "ch/1", 1);
    setf(&store, "main/1/fdr", 0.0);
    store
}

#[test]
fn fader_pan_and_width_laws() {
    let store = base_store();
    setf(&store, "ch/1/fdr", -6.0);
    sett(&store, "io/in/USB/3/mode", "ST");
    sett(&store, "ch/2/in/conn/grp", "USB");
    seti(&store, "ch/2/in/conn/in", 3);
    setf(&store, "ch/2/fdr", 0.0);
    setf(&store, "ch/2/wid", 0.0);
    let mut r = rig(&store, MixConfig::default(), 2);
    let main1 = Strip::Main(1);

    r.cycles(SETTLE, &[1]);
    let pfl = r.input(0, I::Pfl, Side::L);
    let g = lin(-6.0);
    assert_gain(&pfl, &r.input(0, I::Afl, Side::L), g, "AFL = fader");
    assert_gain(
        &pfl,
        &r.mix(main1, M::Post, Side::L),
        g * C,
        "main L, centre -3 dB",
    );
    assert_gain(
        &pfl,
        &r.mix(main1, M::Post, Side::R),
        g * C,
        "main R, centre -3 dB",
    );

    for pan in [-100.0f32, -40.0, 25.0, 100.0] {
        setf(&store, "ch/1/pan", pan);
        r.cycles(SETTLE, &[1]);
        let pfl = r.input(0, I::Pfl, Side::L);
        let (gl, gr) = cp(pan as f64);
        assert_gain(&pfl, &r.mix(main1, M::Post, Side::L), g * gl, "pan L");
        assert_gain(&pfl, &r.mix(main1, M::Post, Side::R), g * gr, "pan R");
    }

    // stereo channel (USB 3/4), width 0 = mono sum, then -100 = swapped
    r.cycles(SETTLE, &[3, 4]);
    let (l, rr) = (r.input(1, I::Pfl, Side::L), r.input(1, I::Pfl, Side::R));
    assert!(fit(&l, &rr).1 > 0.1, "sides differ");
    let mono: Vec<f32> = l.iter().zip(&rr).map(|(a, b)| 0.5 * (a + b)).collect();
    assert_gain(&mono, &r.input(1, I::Post, Side::L), C, "width 0 L");
    assert_gain(&mono, &r.input(1, I::Post, Side::R), C, "width 0 R");
    setf(&store, "ch/2/wid", -100.0);
    r.cycles(SETTLE, &[3, 4]);
    let rr = r.input(1, I::Pfl, Side::R);
    assert_gain(&rr, &r.input(1, I::Post, Side::L), C, "width -100 swaps");

    // main pan uses the balance law: unity at centre, far side attenuated
    setf(&store, "ch/2/wid", 100.0);
    setf(&store, "main/1/pan", 50.0);
    r.cycles(SETTLE, &[3, 4]);
    let (bl, br) = PanLaw::Balance.gains(50.0);
    assert!(br == 1.0 && bl < 0.6);
    let pfl = r.mix(main1, M::Pfl, Side::L);
    assert_gain(
        &pfl,
        &r.mix(main1, M::Post, Side::L),
        bl as f64,
        "main balance L",
    );
}

#[test]
fn mute_silences_fader_and_sends_not_pfl() {
    let store = base_store();
    setf(&store, "ch/1/fdr", 0.0);
    seti(&store, "ch/1/send/1/on", 1);
    setf(&store, "ch/1/send/1/lvl", 0.0);
    sett(&store, "ch/1/send/1/mode", "PRE");
    seti(&store, "ch/1/mute", 1);
    let mut r = rig(&store, MixConfig::default(), 0);
    r.cycles(4, &[1]);
    let pfl = r.input(0, I::Pfl, Side::L);
    assert!(pfl.iter().any(|v| v.abs() > 0.01), "PFL carries signal");
    assert_gain(&pfl, &r.input(0, I::Afl, Side::L), 0.0, "AFL muted");
    assert_gain(&pfl, &r.input(0, I::Pre, Side::L), 0.0, "PRE muted");
    assert_gain(&pfl, &r.mix(Strip::Main(1), M::Pfl, Side::L), 0.0, "main");
    assert_gain(
        &pfl,
        &r.mix(Strip::Bus(1), M::Pfl, Side::L),
        0.0,
        "PRE send",
    );

    // unmute ramps in (no step)
    seti(&store, "ch/1/mute", 0);
    r.cycle(&[1]);
    let afl = r.input(0, I::Afl, Side::L);
    let pfl = r.input(0, I::Pfl, Side::L);
    assert!(
        afl[0].abs() <= pfl[0].abs() * 0.05 + 1e-6,
        "first sample nearly silent"
    );
    r.cycles(SETTLE, &[1]);
    assert_gain(
        &r.input(0, I::Pfl, Side::L),
        &r.mix(Strip::Bus(1), M::Pfl, Side::L),
        C,
        "unmuted PRE send (send pan centre)",
    );

    // external (mute group / DCA) mute and mute-safe are reachable through the node API
    use wing_engine::mix::{block, param};
    use wing_engine::params::ParamMsg;
    let n = r.nodes.inputs[0];
    for (p, v) in [(param::EXT_MUTE, 1.0), (param::MUTE_SAFE, 0.0)] {
        r.e.processor_mut(n).set_param(&ParamMsg::Set {
            block: block::INPUT_MIX,
            param: p,
            value: v,
        });
    }
    r.e.processor_mut(n).set_param(&ParamMsg::Commit);
    r.cycles(SETTLE, &[1]);
    assert_gain(
        &r.input(0, I::Pfl, Side::L),
        &r.input(0, I::Afl, Side::L),
        0.0,
        "ext mute",
    );
    for (p, v) in [(param::MUTE_SAFE, 1.0), (param::EXT_GAIN, -6.0)] {
        r.e.processor_mut(n).set_param(&ParamMsg::Set {
            block: block::INPUT_MIX,
            param: p,
            value: v,
        });
    }
    r.e.processor_mut(n).set_param(&ParamMsg::Commit);
    r.cycles(SETTLE, &[1]);
    assert_gain(
        &r.input(0, I::Pfl, Side::L),
        &r.input(0, I::Afl, Side::L),
        lin(-6.0),
        "mute safe + DCA offset",
    );
}

#[test]
fn send_taps_and_modes() {
    let store = base_store();
    setf(&store, "ch/1/fdr", -10.0);
    setf(&store, "ch/1/pan", 50.0);
    sett(&store, "ch/1/ptap", "PFL");
    let send = |k: &str, mode: &str, lvl: f32, plink: i32, pan: f32| {
        seti(&store, &format!("ch/1/send/{k}/on"), 1);
        setf(&store, &format!("ch/1/send/{k}/lvl"), lvl);
        sett(&store, &format!("ch/1/send/{k}/mode"), mode);
        seti(&store, &format!("ch/1/send/{k}/plink"), plink);
        setf(&store, &format!("ch/1/send/{k}/pan"), pan);
    };
    send("1", "PRE", -6.0, 0, -30.0);
    send("2", "POST", -3.0, 0, 0.0);
    send("3", "POST", -3.0, 1, 0.0);
    send("4", "GRP", -20.0, 0, 0.0);
    send("5", "PRE", 0.0, 1, 0.0);
    send("6", "POST", 0.0, 0, 0.0);
    seti(&store, "ch/1/send/6/on", 0);
    send("MX1", "POST", -2.0, 0, 100.0);
    let mut r = rig(&store, MixConfig::default(), 3);
    r.cycles(SETTLE, &[1]);

    let pfl = r.input(0, I::Pfl, Side::L);
    let g = lin(-10.0);
    let (pl, pr) = cp(50.0);
    let bus = |r: &Rig, k: u8, s: Side| r.mix(Strip::Bus(k), M::Pfl, s);
    let (sl, sr) = cp(-30.0);
    assert_gain(
        &pfl,
        &bus(&r, 1, Side::L),
        lin(-6.0) * sl,
        "PRE, send pan L",
    );
    assert_gain(
        &pfl,
        &bus(&r, 1, Side::R),
        lin(-6.0) * sr,
        "PRE, send pan R",
    );
    assert_gain(
        &pfl,
        &bus(&r, 2, Side::L),
        g * lin(-3.0) * C,
        "POST from AFL",
    );
    assert_gain(
        &pfl,
        &bus(&r, 3, Side::L),
        g * lin(-3.0) * pl,
        "POST linked L",
    );
    assert_gain(
        &pfl,
        &bus(&r, 3, Side::R),
        g * lin(-3.0) * pr,
        "POST linked R",
    );
    assert_gain(&pfl, &bus(&r, 4, Side::L), g * pl, "GRP = POST at unity");
    assert_gain(&pfl, &bus(&r, 4, Side::R), g * pr, "GRP = POST at unity");
    assert_gain(&pfl, &bus(&r, 5, Side::R), pr, "PRE linked to channel pan");
    assert_gain(&pfl, &bus(&r, 6, Side::L), 0.0, "send off");
    assert_gain(&pfl, &bus(&r, 7, Side::L), 0.0, "no send");
    let mtx = r.mix(Strip::Mtx(1), M::Pfl, Side::R);
    assert_gain(&pfl, &mtx, g * lin(-2.0), "ch -> MX1, send pan hard right");
    assert_gain(&pfl, &r.mix(Strip::Mtx(1), M::Pfl, Side::L), 0.0, "MX1 L");

    // tap point AFL: PRE sends follow the fader
    sett(&store, "ch/1/ptap", "AFL");
    r.cycles(SETTLE, &[1]);
    let pfl = r.input(0, I::Pfl, Side::L);
    assert_gain(&pfl, &bus(&r, 1, Side::L), g * lin(-6.0) * sl, "ptap AFL");
    // tap point POST: PRE sends carry the panned channel (send pan on top)
    sett(&store, "ch/1/ptap", "POST");
    r.cycles(SETTLE, &[1]);
    let pfl = r.input(0, I::Pfl, Side::L);
    assert_gain(
        &pfl,
        &bus(&r, 1, Side::R),
        g * pr * lin(-6.0) * sr,
        "ptap POST",
    );
    // tap IN (factory strip: equal to PFL); tapwid 0 % is a mono sum of the (mono) tap
    sett(&store, "ch/1/ptap", "IN");
    setf(&store, "ch/1/tapwid", 0.0);
    r.cycles(SETTLE, &[1]);
    let pfl = r.input(0, I::Pfl, Side::L);
    assert_gain(&pfl, &bus(&r, 1, Side::L), lin(-6.0) * sl, "ptap IN");

    // mode change crossfades (no step): POST -> PRE on send 2
    sett(&store, "ch/1/send/2/mode", "PRE");
    r.cycle(&[1]);
    r.cycles(SETTLE, &[1]);
    let pfl = r.input(0, I::Pfl, Side::L);
    assert_gain(&pfl, &bus(&r, 2, Side::L), lin(-3.0) * C, "POST -> PRE");
}

#[test]
fn busses_mains_matrices_and_tiers() {
    let store = base_store();
    setf(&store, "ch/1/fdr", 0.0);
    seti(&store, "ch/1/main/1/on", 0);
    seti(&store, "ch/1/send/1/on", 1);
    setf(&store, "ch/1/send/1/lvl", 0.0);
    sett(&store, "ch/1/send/1/mode", "POST");
    setf(&store, "bus/1/fdr", -6.0);
    seti(&store, "bus/1/send/3/on", 1);
    setf(&store, "bus/1/send/3/lvl", -6.0);
    seti(&store, "bus/1/main/1/on", 1);
    setf(&store, "bus/1/main/1/lvl", 0.0);
    seti(&store, "bus/1/main/1/pre", 1);
    setf(&store, "bus/3/fdr", 0.0);
    seti(&store, "bus/3/main/2/on", 1);
    setf(&store, "bus/3/main/2/lvl", 0.0);
    setf(&store, "main/2/fdr", 0.0);
    seti(&store, "main/1/send/MX2/on", 1);
    setf(&store, "main/1/send/MX2/lvl", -10.0);
    setf(&store, "mtx/2/fdr", -1.0);
    // a backwards bus send (3 -> 1) is never wired
    seti(&store, "bus/3/send/1/on", 1);
    setf(&store, "bus/3/send/1/lvl", 0.0);

    let t = bus_tiers(&store, BusOrder::FromScene);
    assert_eq!(t[0], 0);
    assert_eq!(t[2], 1);
    assert_eq!(bus_tiers(&store, BusOrder::Flat), [0; 16]);
    assert_eq!(bus_tiers(&store, BusOrder::Chain)[15], 15);

    let mut r = rig(&store, MixConfig::default(), 4);
    assert_eq!(r.tiers, t);
    r.cycles(SETTLE, &[1]);
    let pfl = r.input(0, I::Pfl, Side::L);
    let b1 = C; // POST send, send pan centre
    assert_gain(&pfl, &r.mix(Strip::Bus(1), M::Pfl, Side::L), b1, "bus1 sum");
    assert_gain(
        &pfl,
        &r.mix(Strip::Bus(1), M::Post, Side::L),
        b1 * lin(-6.0),
        "bus1 out",
    );
    let b3 = b1 * lin(-6.0) * lin(-6.0);
    assert_gain(
        &pfl,
        &r.mix(Strip::Bus(3), M::Pfl, Side::L),
        b3,
        "bus1 -> bus3",
    );
    assert_gain(
        &pfl,
        &r.mix(Strip::Bus(1), M::Pfl, Side::R),
        b1,
        "no 3 -> 1 loop",
    );
    assert_gain(
        &pfl,
        &r.mix(Strip::Main(1), M::Pfl, Side::L),
        b1,
        "bus1 pre -> main1",
    );
    assert_gain(
        &pfl,
        &r.mix(Strip::Main(2), M::Post, Side::L),
        b3,
        "bus3 -> main2",
    );
    assert_gain(
        &pfl,
        &r.mix(Strip::Mtx(2), M::Post, Side::L),
        b1 * lin(-10.0) * lin(-1.0),
        "main1 -> mtx2",
    );

    // bus mute stops its post and pre sends
    seti(&store, "bus/1/mute", 1);
    r.cycles(SETTLE, &[1]);
    assert_gain(
        &pfl,
        &r.mix(Strip::Main(1), M::Pfl, Side::L),
        0.0,
        "muted bus pre send",
    );
    assert_gain(
        &pfl,
        &r.mix(Strip::Bus(3), M::Pfl, Side::L),
        0.0,
        "muted bus post send",
    );
    seti(&store, "bus/1/mute", 0);

    // busmono after a hard-left balance pan: both sides carry half the left side
    seti(&store, "bus/1/busmono", 1);
    setf(&store, "bus/1/pan", -100.0);
    r.cycles(SETTLE, &[1]);
    let pfl = r.input(0, I::Pfl, Side::L);
    let half = b1 * lin(-6.0) * 0.5;
    assert_gain(
        &pfl,
        &r.mix(Strip::Bus(1), M::Post, Side::L),
        half,
        "busmono L",
    );
    assert_gain(
        &pfl,
        &r.mix(Strip::Bus(1), M::Post, Side::R),
        half,
        "busmono R",
    );

    // flat bus order: no bus -> bus
    let cfg = MixConfig {
        bus_order: BusOrder::Flat,
        ..MixConfig::default()
    };
    let store2 = ParamStore::new();
    connect_usb(&store2, "ch/1", 1);
    seti(&store2, "ch/1/send/1/on", 1);
    setf(&store2, "ch/1/send/1/lvl", 0.0);
    setf(&store2, "bus/1/fdr", 0.0);
    seti(&store2, "bus/1/send/3/on", 1);
    setf(&store2, "bus/1/send/3/lvl", 0.0);
    let mut r = rig(&store2, cfg, 0);
    r.cycles(4, &[1]);
    let pfl = r.input(0, I::Pfl, Side::L);
    assert_gain(&pfl, &r.mix(Strip::Bus(3), M::Pfl, Side::L), 0.0, "flat");
}

#[test]
fn fader_changes_are_smoothed() {
    let store = base_store();
    setf(&store, "ch/1/fdr", 0.0);
    let mut r = rig(&store, MixConfig::default(), 0);
    r.cycles(4, &[1]);
    setf(&store, "ch/1/fdr", -144.0);
    let (mut prev_g, mut prev_n) = (1.0f64, 0usize);
    let mut blocks = 0;
    let mut n = 0usize;
    loop {
        r.cycle(&[1]);
        let pfl = r.input(0, I::Pfl, Side::L);
        let afl = r.input(0, I::Afl, Side::L);
        for (a, b) in pfl.iter().zip(&afl) {
            n += 1;
            if a.abs() > 0.05 {
                let g = (*b / *a) as f64;
                assert!(g <= prev_g + 1e-6, "monotonic");
                // at most a one-pole step (5 ms) per block, spread linearly over 64 samples
                let per_sample = (prev_g - g) / (n - prev_n) as f64;
                assert!(per_sample < 0.25 / 64.0, "step {prev_g} -> {g}");
                (prev_g, prev_n) = (g, n);
            }
        }
        blocks += 1;
        if afl.iter().all(|&v| v == 0.0) {
            break;
        }
        assert!(blocks < 100, "never reached -oo");
    }
    assert!(blocks > 20, "ramped over {blocks} blocks");
}

#[test]
fn taps_feed_usb_outputs_and_reinputs() {
    let store = base_store();
    setf(&store, "ch/1/fdr", -3.0);
    setf(&store, "ch/1/pan", -20.0);
    seti(&store, "ch/1/send/1/on", 1);
    setf(&store, "ch/1/send/1/lvl", 0.0);
    setf(&store, "bus/1/fdr", 0.0);
    // USB 1/2 <- MAIN 1 L/R, USB 3 <- USR 1 = CH 1 POST L+R, USB 4 <- BUS 2 (= bus 1 R)
    sett(&store, "io/out/USB/1/grp", "MAIN");
    seti(&store, "io/out/USB/1/in", 1);
    sett(&store, "io/out/USB/2/grp", "MAIN");
    seti(&store, "io/out/USB/2/in", 2);
    sett(&store, "io/in/USR/1/user/grp", "CH");
    seti(&store, "io/in/USR/1/user/in", 1);
    sett(&store, "io/in/USR/1/user/tap", "POST");
    sett(&store, "io/in/USR/1/user/lr", "L+R");
    sett(&store, "io/out/USB/3/grp", "USR");
    seti(&store, "io/out/USB/3/in", 1);
    sett(&store, "io/out/USB/4/grp", "BUS");
    seti(&store, "io/out/USB/4/in", 2);
    // ch 2 re-inputs bus 1 (stereo pair BUS.1/2)
    sett(&store, "ch/2/in/conn/grp", "BUS");
    seti(&store, "ch/2/in/conn/in", 1);
    let mut r = rig(&store, MixConfig::default(), 2);
    r.cycles(SETTLE, &[1]);
    let usb = |r: &Rig, n: u32| r.port(Port::node(r.nodes.usb_out, n - 1));
    assert_eq!(usb(&r, 1), r.mix(Strip::Main(1), M::Post, Side::L));
    assert_eq!(usb(&r, 2), r.mix(Strip::Main(1), M::Post, Side::R));
    let (pl, pr) = (r.input(0, I::Post, Side::L), r.input(0, I::Post, Side::R));
    let mono: Vec<f32> = pl.iter().zip(&pr).map(|(a, b)| 0.5 * (a + b)).collect();
    let u3 = usb(&r, 3);
    for (a, b) in u3.iter().zip(&mono) {
        assert!((a - b).abs() < 1e-6);
    }
    assert_eq!(usb(&r, 4), r.mix(Strip::Bus(1), M::Post, Side::R));

    // the re-input arrives one block later, through ch 2's (default, transparent) strip
    let prev = r.mix(Strip::Bus(1), M::Post, Side::L);
    r.cycle(&[1]);
    let ch2 = r.input(1, I::Pfl, Side::L);
    assert!(prev.iter().any(|v| v.abs() > 0.01));
    for (a, b) in ch2.iter().zip(&prev) {
        assert!((a - b).abs() < 1e-5, "{a} vs {b}");
    }
}

/// Hand-computed main mix gains of PUBLIC.snap (fw 1.13 scene, migrated by wing-model). Only one
/// channel carries signal at a time (moved to an otherwise unused USB input); its PFL tap is the
/// reference, so the channel's own scene processing (EQ, dynamics) does not matter. Constant-power
/// channel pan (-3 dB centre) and balance bus/main pan, GRP sends at unity [I].
#[test]
fn public_snap_main_mix() {
    let store = public_store();
    connect_usb(&store, "ch/26", 40);
    connect_usb(&store, "ch/21", 41);
    connect_usb(&store, "ch/1", 42);
    connect_usb(&store, "ch/23", 43);
    let f = |p: &str| getf(&store, p) as f64;
    // scene facts the expectations rely on
    assert_eq!(f("ch/26/pan"), 0.0);
    assert_eq!(store.get_f64(id(&store, "ch/1/send/6/mode")), Some(2.0)); // GRP
    assert_eq!(f("ch/1/send/6/lvl"), -144.0);
    assert_eq!(f("main/1/pan"), 0.0);

    let mut r = rig(&store, MixConfig::default(), 4);
    assert_eq!(r.tiers[0], 0); // bus 1 VOX VERB
    assert_eq!(r.tiers[4], 1); // bus 5 VOX <- busses 1, 2
    assert_eq!(r.tiers[5], 1); // bus 6 DRUMS FOH <- bus 4
    assert_eq!(r.tiers[6], 2); // bus 7 BAND BUS <- busses 3, 6

    let main1 = f("main/1/fdr");
    let check = |r: &mut Rig, usb: u32, ch: usize, g: f64, what: &str| {
        r.cycles(SETTLE, &[usb]);
        let mut x = Vec::new();
        let (mut yl, mut yr, mut m1) = (Vec::new(), Vec::new(), Vec::new());
        for _ in 0..20 {
            r.cycle(&[usb]);
            x.extend(r.input(ch - 1, I::Pfl, Side::L));
            yl.extend(r.mix(Strip::Main(1), M::Post, Side::L));
            yr.extend(r.mix(Strip::Main(1), M::Post, Side::R));
            m1.extend(r.mix(Strip::Mtx(1), M::Post, Side::L));
        }
        assert!(x.iter().any(|v| v.abs() > 1e-3), "{what}: no signal");
        for (y, side) in [(&yl, "L"), (&yr, "R")] {
            let (got, res) = fit(&x, y);
            if g == 0.0 {
                assert!(got.abs() < 1e-6, "{what} {side}: {got}");
                continue;
            }
            assert!(
                (db(got) - db(g)).abs() < 0.01 && res < 1e-3,
                "{what} {side}: {:.4} dB, expected {:.4} dB (residual {res})",
                db(got),
                db(g)
            );
        }
        // mtx 1 FOH <- main 1 (MX1 on, 0 dB, post), mtx fader
        let (got, _) = fit(&yl, &m1);
        if g != 0.0 {
            assert!((db(got) - f("mtx/1/fdr")).abs() < 0.01, "{what}: mtx1");
        }
    };

    // ch 26 PREACH: main 1 on at 0 dB, post fader, pan centre.
    let g26 = lin(f("ch/26/fdr")) * C * lin(f("ch/26/main/1/lvl")) * lin(main1);
    check(&mut r, 40, 26, g26, "ch26 -> main1");

    // ch 21 VOX2: no main send; GRP into bus 5 (unity) plus POST sends to the FX busses 1 and 2
    // (no FX inserted in the simulator: they return dry) which feed bus 5; bus 5 -> main 1.
    let g21 = lin(f("ch/21/fdr"))
        * C
        * (1.0
            + lin(f("ch/21/send/1/lvl")) * lin(f("bus/1/fdr")) * lin(f("bus/1/send/5/lvl"))
            + lin(f("ch/21/send/2/lvl")) * lin(f("bus/2/fdr")) * lin(f("bus/2/send/5/lvl")))
        * lin(f("bus/5/fdr"))
        * lin(f("bus/5/main/1/lvl"))
        * lin(main1);
    check(&mut r, 41, 21, g21, "ch21 -> bus5 -> main1");

    // ch 1 KICK: GRP into bus 6 at -oo send level (unity: GRP ignores it) plus POST -4.4 dB into
    // bus 4 DRUM VRB -> bus 6; bus 6 -> bus 7 BAND BUS -> main 1.
    let g1 = lin(f("ch/1/fdr"))
        * C
        * (1.0 + lin(f("ch/1/send/4/lvl")) * lin(f("bus/4/fdr")) * lin(f("bus/4/send/6/lvl")))
        * lin(f("bus/6/fdr"))
        * lin(f("bus/6/send/7/lvl"))
        * lin(f("bus/7/fdr"))
        * lin(f("bus/7/main/1/lvl"))
        * lin(main1);
    check(&mut r, 42, 1, g1, "ch1 -> bus6 -> bus7 -> main1");

    // ch 23 VOX4 is muted
    check(&mut r, 43, 23, 0.0, "ch23 muted");
}

#[test]
fn parallel_matches_serial() {
    let all: Vec<u32> = (1..=48).collect();
    let run = |workers: Option<usize>| -> Vec<u32> {
        let store = public_store();
        for n in 1..=40 {
            connect_usb(&store, &format!("ch/{n}"), n);
        }
        for n in 1..=8 {
            connect_usb(&store, &format!("aux/{n}"), 40 + n);
        }
        let mut r = rig(&store, MixConfig::default(), workers.unwrap_or(0));
        let mut out = Vec::new();
        for _ in 0..30 {
            r.feed(&all);
            r.pipe.apply(&mut r.e);
            match workers {
                Some(_) => {
                    r.e.process_cycle();
                }
                None => r.e.process_cycle_serial(),
            }
            for s in [Strip::Main(1), Strip::Main(4), Strip::Mtx(3), Strip::Bus(8)] {
                out.extend(r.mix(s, M::Post, Side::L).iter().map(|v| v.to_bits()));
            }
        }
        out
    };
    let serial = run(None);
    assert!(serial.iter().any(|&b| f32::from_bits(b) != 0.0));
    assert_eq!(run(Some(5)), serial);
}
