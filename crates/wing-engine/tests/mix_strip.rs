//! T037: bus/main/matrix strip processing (trim, Ins1, EQ, dyn, Ins2, output delay): process order
//! and bypass, EQ/dynamics identical to the standalone wing-blocks models, output-delay accuracy,
//! telemetry, and the schema bindings through the complete mix graph.

mod common;

use common::rig::*;
use wing_blocks::eq::EqVariant;
use wing_blocks::model::{create_model, BlockKind, Model, ParamValue};
use wing_engine::channel::InsertHost;
use wing_engine::mix::strip::hook;
use wing_engine::mix::{MixConfig, MixSignal as M, MixStrip, MixTap, StripHook};
use wing_engine::params::{ParamMsg, TARGET_BASE};
use wing_engine::routing::{Side, Strip};
use wing_model::ParamStore;

const FS: f64 = 48_000.0;
const B: usize = 64;

fn set(m: &mut MixStrip, block: u8, param: u16, value: f32) {
    m.apply(&ParamMsg::Set {
        block,
        param,
        value,
    });
}

fn model(m: &mut MixStrip, block: u8, model: u16) {
    m.apply(&ParamMsg::Model { block, model });
}

fn noise(n: usize, seed: u32, amp: f32) -> Vec<f32> {
    let mut s = seed.wrapping_mul(2_654_435_761).max(1);
    (0..n)
        .map(|_| {
            s ^= s << 13;
            s ^= s >> 17;
            s ^= s << 5;
            (s as f32 / u32::MAX as f32 * 2.0 - 1.0) * amp
        })
        .collect()
}

/// Run `x` (stereo) through the strip's pre-fader chain in blocks of `B`.
fn run_pre(m: &mut MixStrip, xl: &[f32], xr: &[f32]) -> (Vec<f32>, Vec<f32>) {
    let (mut l, mut r) = (xl.to_vec(), xr.to_vec());
    for (a, b) in l.chunks_mut(B).zip(r.chunks_mut(B)) {
        m.process_pre(a, b);
    }
    (l, r)
}

/// STD bus EQ settings as (0-based target position, value).
const EQ_SET: [(u16, f32); 7] = [
    (0, 4.0),    // lg
    (4, 6.0),    // 1g
    (5, 1000.0), // 1f
    (6, 2.0),    // 1q
    (10, -5.0),  // 3g
    (22, -3.0),  // hg
    (26, 2.5),   // tilt
];

/// COMP settings (0-based positions: thr, ratio).
const DYN_SET: [(u16, f32); 2] = [(0, -24.0), (1, 4.0)];

fn standalone(kind: BlockKind, id: &'static str, set: &[(u16, f32)]) -> Box<dyn Model> {
    let mut m = create_model(kind, id);
    m.prepare(FS, B);
    m.load_defaults();
    m.set_gain_scale(100.0);
    for &(pos, v) in set {
        m.set_param(pos + 1, ParamValue::Float(v));
    }
    m.commit();
    m.reset();
    m
}

#[test]
fn eq_and_dyn_match_standalone_blocks() {
    let (xl, xr) = (noise(4096, 1, 0.5), noise(4096, 2, 0.4));
    for strip in [Strip::Bus(3), Strip::Main(1), Strip::Mtx(8)] {
        // EQ alone (STD bus variant)
        let mut m = MixStrip::new(strip, FS, B);
        model(&mut m, hook::EQ, 0);
        set(&mut m, hook::EQ, 0, 1.0);
        set(&mut m, hook::EQ, 1, 100.0);
        for (p, v) in EQ_SET {
            set(&mut m, hook::EQ, TARGET_BASE + p, v);
        }
        set(&mut m, hook::DYN, 0, 0.0);
        m.settle();
        let (yl, yr) = run_pre(&mut m, &xl, &xr);
        let mut eq = standalone(BlockKind::Eq(EqVariant::Bus), "STD", &EQ_SET);
        let (mut zl, mut zr) = (xl.clone(), xr.clone());
        for (a, b) in zl.chunks_mut(B).zip(zr.chunks_mut(B)) {
            eq.process(a, b, None);
        }
        assert_eq!(yl, zl, "{strip:?} EQ L");
        assert_eq!(yr, zr, "{strip:?} EQ R");
        assert!(yl.iter().zip(&xl).any(|(a, b)| (a - b).abs() > 1e-3));

        // EQ -> COMP (stereo-linked, keyed from itself)
        model(&mut m, hook::DYN, 13);
        set(&mut m, hook::DYN, 0, 1.0);
        set(&mut m, hook::DYN, 1, 100.0);
        set(&mut m, hook::DYN, 2, 0.0);
        for (p, v) in DYN_SET {
            set(&mut m, hook::DYN, TARGET_BASE + p, v);
        }
        m.settle();
        let (yl, yr) = run_pre(&mut m, &xl, &xr);
        let mut eq = standalone(BlockKind::Eq(EqVariant::Bus), "STD", &EQ_SET);
        let mut comp = standalone(BlockKind::Dynamics, "COMP", &DYN_SET);
        let (mut zl, mut zr) = (xl.clone(), xr.clone());
        for (a, b) in zl.chunks_mut(B).zip(zr.chunks_mut(B)) {
            eq.process(a, b, None);
            comp.process(a, b, None);
        }
        assert_eq!(yl, zl, "{strip:?} EQ+COMP L");
        assert_eq!(yr, zr, "{strip:?} EQ+COMP R");
        let t = m.take_telemetry();
        assert!(t.dyn_gr_db > 1.0, "{strip:?}: GR {}", t.dyn_gr_db);
    }
}

/// Insert host: FX1 doubles, FX2 halves, and records the call order.
#[derive(Default)]
struct GainFx {
    calls: std::sync::Arc<std::sync::Mutex<Vec<u8>>>,
}

impl InsertHost for GainFx {
    fn process_insert(&mut self, fx: u8, l: &mut [f32], r: &mut [f32]) {
        let g = if fx == 1 { 2.0 } else { 0.5 };
        for v in l.iter_mut().chain(r.iter_mut()) {
            *v *= g;
        }
        if let Ok(mut c) = self.calls.try_lock() {
            if c.len() < 2 {
                c.push(fx);
            }
        }
    }
}

#[test]
fn process_order_and_bypass() {
    let x: Vec<f32> = (0..B).map(|i| (i as f32 * 0.2).sin() * 0.1).collect();
    let mut m = MixStrip::new(Strip::Bus(1), FS, B);
    let fx = GainFx::default();
    let calls = fx.calls.clone();
    m.set_insert_host(Box::new(fx));

    // trim +6 dB, balance and polarity
    set(&mut m, hook::INPUT, 0, 6.0);
    set(&mut m, hook::INPUT, 2, 1.0);
    // Ins1 = FX1, Ins2 = FX2
    set(&mut m, hook::PREINS, 0, 1.0);
    set(&mut m, hook::PREINS, 1, 1.0);
    set(&mut m, hook::POSTINS, 0, 1.0);
    set(&mut m, hook::POSTINS, 1, 2.0);
    // COMP with a threshold the trimmed + doubled signal crosses but the input does not
    model(&mut m, hook::DYN, 13);
    set(&mut m, hook::DYN, 0, 1.0);
    set(&mut m, hook::DYN, 1, 100.0);
    set(&mut m, hook::DYN, TARGET_BASE, -20.0);
    set(&mut m, hook::DYN, TARGET_BASE + 1, 10.0);
    m.settle();
    for k in 0..400 {
        if k == 200 {
            m.take_telemetry(); // measure with the compressor settled
        }
        let (mut l, mut r) = (x.clone(), x.clone());
        m.process_pre(&mut l, &mut r);
    }
    assert_eq!(*calls.lock().unwrap(), vec![1, 2], "Ins1 before Ins2");
    let t = m.take_telemetry();
    let p = |k: MixTap| t.tap_peak[k as usize];
    let trim = 10f32.powf(6.0 / 20.0);
    assert!(
        (p(MixTap::In) / (0.1 * trim) - 1.0).abs() < 0.02,
        "IN {}",
        p(MixTap::In)
    );
    assert!((p(MixTap::Ins1) / p(MixTap::In) - 2.0).abs() < 1e-4, "INS1");
    assert_eq!(p(MixTap::Eq), p(MixTap::Ins1), "EQ off is bypassed");
    assert!(
        p(MixTap::Dyn) < p(MixTap::Eq) * 0.8,
        "dyn after Ins1 compresses"
    );
    assert!(t.dyn_gr_db > 1.0);
    assert!(
        (p(MixTap::Pfl) / p(MixTap::Dyn) - 0.5).abs() < 1e-4,
        "Ins2 after dyn"
    );

    // polarity is inverted
    let (mut l, mut r) = (x.clone(), x.clone());
    set(&mut m, hook::DYN, 0, 0.0);
    set(&mut m, hook::PREINS, 0, 0.0);
    set(&mut m, hook::POSTINS, 0, 0.0);
    m.settle();
    m.process_pre(&mut l, &mut r);
    for (y, v) in l.iter().zip(&x) {
        assert!((y + v * trim).abs() < 1e-6);
    }

    // everything off again: bit-exact pass-through
    set(&mut m, hook::INPUT, 0, 0.0);
    set(&mut m, hook::INPUT, 2, 0.0);
    set(&mut m, hook::EQ, 0, 0.0);
    m.settle();
    let (mut l, mut r) = (x.clone(), x.clone());
    m.process_pre(&mut l, &mut r);
    m.process_out(&mut l, &mut r);
    assert_eq!(l, x);
    assert_eq!(r, x);
}

#[test]
fn balance_attenuates_opposite_side() {
    let x = vec![0.5f32; B];
    let mut m = MixStrip::new(Strip::Mtx(2), FS, B);
    set(&mut m, hook::INPUT, 1, 6.0);
    m.settle();
    let (mut l, mut r) = (x.clone(), x.clone());
    m.process_pre(&mut l, &mut r);
    assert!((l[0] - 0.5 * 10f32.powf(-6.0 / 20.0)).abs() < 1e-6);
    assert_eq!(r[0], 0.5);
}

fn impulse_delay(mode: u16, value: f32) -> Vec<f32> {
    impulse_delay_n(mode, value, 64 * B)
}

fn impulse_delay_n(mode: u16, value: f32, n: usize) -> Vec<f32> {
    let mut m = MixStrip::new(Strip::Main(2), FS, B);
    set(&mut m, hook::DLY, 0, 1.0);
    model(&mut m, hook::DLY, mode);
    set(&mut m, hook::DLY, TARGET_BASE, value);
    m.settle();
    let mut x = vec![0.0f32; n];
    x[10] = 1.0;
    let mut r = x.clone();
    for (a, b) in x.chunks_mut(B).zip(r.chunks_mut(B)) {
        m.process_out(a, b);
    }
    assert_eq!(x, r);
    x
}

#[test]
fn output_delay_accuracy() {
    // SMP: exact integer delay
    let y = impulse_delay(3, 100.0);
    assert_eq!(y[110], 1.0);
    assert_eq!(y.iter().filter(|&&v| v != 0.0).count(), 1);
    // MS: 10 ms = 480 samples
    let y = impulse_delay(2, 10.0);
    assert!((y[490] - 1.0).abs() < 1e-4, "{}", y[490]);
    // M: 3.43 m at 343 m/s = 10 ms
    let y = impulse_delay(0, 3.43);
    let s: f32 = y[488..493].iter().sum();
    assert!(
        (s - 1.0).abs() < 1e-4 && y[490] > 0.999,
        "{:?}",
        &y[488..493]
    );
    // FT: 10 ft = 3.048 m = 426.53 samples (linear interpolation between 426 and 427)
    let d = 10.0 * 0.3048 / 343.0 * FS;
    let y = impulse_delay(1, 10.0);
    let frac = (d - d.floor()) as f32;
    let i = 10 + d.floor() as usize;
    assert!((y[i] - (1.0 - frac)).abs() < 1e-3, "{} {}", y[i], frac);
    assert!((y[i + 1] - frac).abs() < 1e-3);
    // longest: 500 ms
    let y = impulse_delay_n(2, 500.0, 400 * B);
    assert!((y[24_010] - 1.0).abs() < 1e-4, "{}", y[24_010]);
}

// ---- through the complete graph -----------------------------------------------------------------

fn store_ch1_to(dest: &str) -> ParamStore {
    let store = ParamStore::new();
    connect_usb(&store, "ch/1", 1);
    setf(&store, "ch/1/fdr", 0.0);
    match dest {
        "bus" => {
            for b in [1, 2] {
                seti(&store, &format!("ch/1/send/{b}/on"), 1);
                setf(&store, &format!("ch/1/send/{b}/lvl"), 0.0);
                sett(&store, &format!("ch/1/send/{b}/mode"), "PRE");
                setf(&store, &format!("bus/{b}/fdr"), 0.0);
            }
        }
        _ => {
            seti(&store, "ch/1/main/1/on", 1);
            setf(&store, "ch/1/main/1/lvl", 0.0);
            setf(&store, "main/1/fdr", 0.0);
        }
    }
    store
}

/// `bus/1` EQ + COMP set through the schema equals bus 2 (same input, no processing) run through
/// the standalone models.
#[test]
fn bus_params_via_schema_match_standalone() {
    let store = store_ch1_to("bus");
    sett(&store, "bus/1/eq/mdl", "STD");
    seti(&store, "bus/1/eq/on", 1);
    setf(&store, "bus/1/eq/mix", 100.0);
    for (p, v) in EQ_SET {
        setf(&store, &format!("bus/1/eq/{}", p + 1), v);
    }
    sett(&store, "bus/1/dyn/mdl", "COMP");
    seti(&store, "bus/1/dyn/on", 1);
    setf(&store, "bus/1/dyn/mix", 100.0);
    setf(&store, "bus/1/dyn/gain", 0.0);
    setf(&store, "bus/1/dyn/1", -30.0);
    seti(&store, "bus/2/eq/on", 0);
    seti(&store, "bus/2/dyn/on", 0);
    let mut r = rig(&store, MixConfig::default(), 2);
    // the standalone models take every STD/COMP position as the store holds it
    let read = |blk: &str, n: usize| -> Vec<(u16, f32)> {
        (0..n)
            .map(|p| (p as u16, getf(&store, &format!("bus/1/{blk}/{}", p + 1))))
            .collect()
    };
    let mut eq = standalone(BlockKind::Eq(EqVariant::Bus), "STD", &read("eq", 27));
    let comp_set: Vec<(u16, f32)> = read("dyn", 19);
    let mut comp = create_model(BlockKind::Dynamics, "COMP");
    comp.prepare(FS, B);
    comp.load_defaults();
    // enum/float-enum positions keep the model defaults when the raw store value is not a value
    let defaults: Vec<Option<f32>> = (1..=19).map(|s| comp.param(s)).collect();
    for (p, v) in comp_set {
        if defaults[p as usize].is_some() {
            comp.set_param(p + 1, ParamValue::Float(v));
        }
    }
    comp.commit();
    comp.reset();

    let mut worst = 0.0f32;
    let mut gr = 0.0f32;
    for k in 0..400 {
        r.cycle(&[1]);
        let mut zl = r.mix(Strip::Bus(2), M::Pfl, Side::L);
        let mut zr = r.mix(Strip::Bus(2), M::Pfl, Side::R);
        eq.process(&mut zl, &mut zr, None);
        comp.process(&mut zl, &mut zr, None);
        gr = gr.max(comp.telemetry().gain_reduction_db);
        if k < 300 {
            continue; // slot on-ramp and parameter glides from the model defaults
        }
        for (a, b) in r.mix(Strip::Bus(1), M::Pfl, Side::L).iter().zip(&zl) {
            worst = worst.max((a - b).abs());
        }
        for (a, b) in r.mix(Strip::Bus(1), M::Pfl, Side::R).iter().zip(&zr) {
            worst = worst.max((a - b).abs());
        }
    }
    assert!(gr > 1.0, "comp engaged: {gr}");
    // the slot starts from the model defaults and glides (detector/parameter state) to the scene
    // values; the standalone model starts settled, so the residual decays rather than being 0
    assert!(worst < 1e-4, "bus 1 vs standalone: {worst}");
}

/// `main/1/dly` delays POST (output) but not PRE.
#[test]
fn output_delay_is_post_fader_only() {
    let store = store_ch1_to("main");
    seti(&store, "main/1/dly/on", 1);
    sett(&store, "main/1/dly/mode", "SMP");
    setf(&store, "main/1/dly/dly", 100.0);
    let mut r = rig(&store, MixConfig::default(), 0);
    let (mut pre, mut post) = (Vec::new(), Vec::new());
    for _ in 0..800 {
        r.cycle(&[1]);
        pre.extend(r.mix(Strip::Main(1), M::Pre, Side::L));
        post.extend(r.mix(Strip::Main(1), M::Post, Side::L));
    }
    assert!(pre.iter().any(|v| v.abs() > 1e-3));
    // after the 50 ms delay glide from the scene start
    for n in pre.len() - 6400..pre.len() {
        assert!((post[n] - pre[n - 100]).abs() < 1e-6, "n={n}");
    }
}

/// `MixStrip` works as a boxed `StripHook` (default settings are transparent).
#[test]
fn mix_strip_hook_trait_object() {
    let mut h: Box<dyn StripHook> = Box::new(MixStrip::new(Strip::Bus(16), FS, B));
    let (mut l, mut r) = (vec![0.25f32; B], vec![0.25f32; B]);
    h.process(&mut l, &mut r);
    h.process_post(&mut l, &mut r);
    assert_eq!(l[0], 0.25);
}

/// T039 hook: bus/main/matrix strips write IN, dynamics key and GR into the level bank.
#[test]
fn mix_strip_meters_fill_level_bank() {
    use wing_engine::meters::{strip_cell, word, LevelBank};
    let store = store_ch1_to("bus");
    sett(&store, "bus/1/dyn/mdl", "COMP");
    seti(&store, "bus/1/dyn/on", 1);
    setf(&store, "bus/1/dyn/1", -40.0);
    setf(&store, "bus/1/in/set/trim", -6.0);
    seti(&store, "bus/2/dyn/on", 0);
    let bank = LevelBank::shared();
    let cfg = MixConfig {
        meters: Some(bank.clone()),
        ..MixConfig::default()
    };
    let mut r = rig(&store, cfg, 0);
    r.cycles(100, &[1]);
    let si = Strip::Bus(1).index().unwrap();
    let cell = |w| bank.get(strip_cell(si, w));
    assert!(cell(word::IN_L) > 1e-3 && cell(word::IN_R) > 1e-3);
    assert!(cell(word::DYN_KEY) > 1e-3);
    assert!(cell(word::DYN_GR) > 1.0, "GR {}", cell(word::DYN_GR));
    assert_eq!(cell(word::GATE_GR), 0.0);
    // bus 2 (same input, no dynamics, no trim) reads 6 dB above bus 1's trimmed input
    let s2 = Strip::Bus(2).index().unwrap();
    let in2 = bank.get(strip_cell(s2, word::IN_L));
    assert!((20.0 * (in2 / cell(word::IN_L)).log10() - 6.0).abs() < 0.05);
    assert_eq!(bank.get(strip_cell(s2, word::DYN_GR)), 0.0);
}
