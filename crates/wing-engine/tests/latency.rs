//! T03E: latency compensation. Model slots run at a constant latency (dry path delayed, each model
//! fed at `slot latency - model latency`), strips pad to a common latency, so dry/wet blends do not
//! comb-filter and strips with different models stay sample-aligned when summed.

mod common;

use common::rig::*;
use wing_blocks::model::{create_model, BlockKind};
use wing_engine::channel::{block, ChannelStrip};
use wing_engine::mix::{MixConfig, MixSignal as M, MixStrip};
use wing_engine::params::std_models::model_factory;
use wing_engine::params::strip::ModelSlot;
use wing_engine::params::{ParamMsg, TARGET_BASE};
use wing_engine::routing::{Side, Strip};
use wing_model::{ParamStore, Schema};

const FS: f64 = 48_000.0;
const B: usize = 64;
/// 76LA/NSTR latency at 48 kHz (wing-blocks comp_fet).
const FET_LAT: usize = 57;

fn model_index(group: &str, name: &str) -> u16 {
    let s = Schema::get();
    let g = s.groups().iter().find(|g| g.key == group).unwrap();
    s.models(g)
        .iter()
        .find(|m| m.name == name)
        .unwrap()
        .enum_index as u16
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

/// Bursty programme: noise with a slow envelope (drives the compressors in and out).
fn programme(n: usize, seed: u32) -> Vec<f32> {
    noise(n, seed, 1.0)
        .iter()
        .enumerate()
        .map(|(i, v)| v * (0.05 + 0.45 * (0.5 + 0.5 * (i as f32 * 0.0007).sin())))
        .collect()
}

fn delayed(x: &[f32], d: usize) -> Vec<f32> {
    let mut y = vec![0.0; x.len()];
    y[d..].copy_from_slice(&x[..x.len() - d]);
    y
}

fn rms(x: &[f32]) -> f64 {
    (x.iter().map(|&v| v as f64 * v as f64).sum::<f64>() / x.len() as f64).sqrt()
}

/// Lag (in `-max..=max`) maximising the cross-correlation of `b` against `a`.
fn xcorr_lag(a: &[f32], b: &[f32], max: isize) -> isize {
    let n = a.len() as isize;
    let mut best = (f64::MIN, 0);
    for lag in -max..=max {
        let mut c = 0.0f64;
        for i in max..n - max {
            c += a[i as usize] as f64 * b[(i + lag) as usize] as f64;
        }
        if c > best.0 {
            best = (c, lag);
        }
    }
    best.1
}

#[test]
fn slot_latency_is_constant_and_documented() {
    let dyn_slot = |g: &str| {
        let k = wing_engine::params::std_models::block_kind(g).unwrap();
        ModelSlot::from_group(g, FS, B, model_factory(k, FS, B))
    };
    // gate and dynamics slots host the FET models; EQ / filter-tool slots are zero-latency
    assert_eq!(dyn_slot("ch/*/gate").latency(), FET_LAT);
    assert_eq!(dyn_slot("ch/*/dyn").latency(), FET_LAT);
    assert_eq!(dyn_slot("ch/*/eq").latency(), 0);
    assert_eq!(dyn_slot("ch/*/flt").latency(), 0);
    // documented totals (crate::latency)
    assert_eq!(ChannelStrip::channel(FS, B).latency(), 2 * FET_LAT);
    assert_eq!(ChannelStrip::aux(FS, B).latency(), 2 * FET_LAT);
    for s in [Strip::Bus(1), Strip::Main(1), Strip::Mtx(1)] {
        assert_eq!(MixStrip::new(s, FS, B).latency(), FET_LAT, "{s:?}");
    }
    // the slot latency does not follow the model
    let mut s = dyn_slot("ch/*/dyn");
    s.select(model_index("ch/*/dyn", "COMP"));
    s.settle();
    assert_eq!((s.latency(), s.model_latency()), (FET_LAT, 0));
    s.select(model_index("ch/*/dyn", "NSTR"));
    s.settle();
    assert_eq!((s.latency(), s.model_latency()), (FET_LAT, FET_LAT));
}

/// NSTR at mix 80 % equals the ideal parallel blend `0.2 * dry(t - 57) + 0.8 * NSTR(x)`: no comb
/// filtering. The uncompensated blend (dry not delayed) differs by a large comb residual.
#[test]
fn nstr_parallel_mix_nulls_against_ideal() {
    let x = programme(48_000, 7);
    let mut slot =
        ModelSlot::from_group("ch/*/dyn", FS, B, model_factory(BlockKind::Dynamics, FS, B));
    slot.select(model_index("ch/*/dyn", "NSTR"));
    slot.set_wrapper(0, 1.0);
    slot.set_wrapper(1, 80.0);
    slot.set_wrapper(2, 0.0);
    slot.settle();
    let (mut l, mut r) = (x.clone(), x.clone());
    for (a, b) in l.chunks_mut(B).zip(r.chunks_mut(B)) {
        slot.process(a, b);
    }
    assert!(slot.gain_reduction_db() > 0.0 || l.iter().zip(&x).any(|(a, b)| a != b));

    let mut nstr = create_model(BlockKind::Dynamics, "NSTR");
    nstr.prepare(FS, B);
    nstr.load_defaults();
    nstr.commit();
    nstr.reset();
    assert_eq!(nstr.latency(), FET_LAT);
    let (mut wl, mut wr) = (x.clone(), x.clone());
    for (a, b) in wl.chunks_mut(B).zip(wr.chunks_mut(B)) {
        nstr.process(a, b, None);
    }
    let dry = delayed(&x, FET_LAT);
    let ideal: Vec<f32> = dry
        .iter()
        .zip(&wl)
        .map(|(d, w)| 0.2 * d + 0.8 * w)
        .collect();
    let naive: Vec<f32> = x.iter().zip(&wl).map(|(d, w)| 0.2 * d + 0.8 * w).collect();
    let err: Vec<f32> = l.iter().zip(&ideal).map(|(a, b)| a - b).collect();
    let comb: Vec<f32> = naive.iter().zip(&ideal).map(|(a, b)| a - b).collect();
    let null_db = 20.0 * (rms(&err) / rms(&ideal)).log10();
    let comb_db = 20.0 * (rms(&comb) / rms(&ideal)).log10();
    println!("NSTR mix 80: null {null_db:.1} dB, uncompensated {comb_db:.1} dB");
    assert!(null_db < -120.0, "null {null_db:.1} dB");
    assert!(comb_db > -30.0, "test not sensitive: {comb_db:.1} dB");
    assert_eq!(l, r);
}

/// A model swap between models of different latency (COMP 0 -> NSTR 57) crossfades aligned signals:
/// at unity-ish levels the output has no step larger than the signal's own.
#[test]
fn swap_between_latencies_is_click_free() {
    let sine: Vec<f32> = (0..24_000)
        .map(|i| 0.05 * (i as f32 * 2.0 * std::f32::consts::PI * 500.0 / 48_000.0).sin())
        .collect();
    let mut slot =
        ModelSlot::from_group("ch/*/dyn", FS, B, model_factory(BlockKind::Dynamics, FS, B));
    slot.select(model_index("ch/*/dyn", "COMP"));
    slot.set_target(0, 0.0); // thr 0 dB: COMP is unity
    slot.set_wrapper(0, 1.0);
    slot.settle();
    let (mut l, mut r) = (sine.clone(), sine.clone());
    for (k, (a, b)) in l.chunks_mut(B).zip(r.chunks_mut(B)).enumerate() {
        if k == 150 {
            slot.select(model_index("ch/*/dyn", "NSTR"));
            slot.commit();
        }
        slot.process(a, b);
    }
    let d2 = |s: &[f32]| {
        s.windows(3)
            .map(|w| (w[2] - 2.0 * w[1] + w[0]).abs())
            .fold(0.0f32, f32::max)
    };
    let steady = d2(&l[100 * B..150 * B]);
    let around = d2(&l[150 * B..170 * B]);
    assert!(around < 2.0 * steady, "swap step {around} vs {steady}");
}

fn set(s: &mut ChannelStrip, blk: u8, param: u16, value: f32) {
    s.apply(&ParamMsg::Set {
        block: blk,
        param,
        value,
    });
}

/// Channels with different dynamics models (NSTR 57 samples vs COMP 0) and an aux strip arrive
/// sample-aligned: the dry contributions null exactly and the processed ones correlate at lag 0.
#[test]
fn strips_with_different_models_stay_aligned() {
    let x = programme(16_384, 3);
    let run = |s: &mut ChannelStrip| {
        let (mut l, mut r) = (x.clone(), x.clone());
        for (a, b) in l.chunks_mut(B).zip(r.chunks_mut(B)) {
            s.process_stereo(a, b);
        }
        l
    };
    let nstr = model_index("ch/*/dyn", "NSTR");
    // (1) NSTR selected at mix 0 % (dry through the slot) vs COMP channel (off) vs aux: exact null
    let mut a = ChannelStrip::channel(FS, B);
    a.apply(&ParamMsg::Model {
        block: block::DYN,
        model: nstr,
    });
    set(&mut a, block::DYN, 0, 1.0);
    set(&mut a, block::DYN, 1, 0.0);
    a.settle();
    let mut b = ChannelStrip::channel(FS, B);
    b.settle();
    let mut c = ChannelStrip::aux(FS, B);
    c.settle();
    let (ya, yb, yc) = (run(&mut a), run(&mut b), run(&mut c));
    assert_eq!(ya, yb, "NSTR (mix 0) vs COMP channel");
    assert_eq!(ya, yc, "channel vs aux");
    assert_eq!(ya, delayed(&x, a.latency()));

    // (2) NSTR compressing vs COMP compressing: the processed signals correlate at lag 0
    set(&mut a, block::DYN, 1, 100.0);
    a.settle();
    b.apply(&ParamMsg::Model {
        block: block::DYN,
        model: model_index("ch/*/dyn", "COMP"),
    });
    set(&mut b, block::DYN, TARGET_BASE, -20.0);
    set(&mut b, block::DYN, 0, 1.0);
    b.settle();
    let (ya, yb) = (run(&mut a), run(&mut b));
    assert_eq!(xcorr_lag(&ya, &yb, 100), 0);
    assert_eq!(xcorr_lag(&yc, &ya, 100), 0);
}

/// Through the mix graph: ch 1 (NSTR) -> bus 1 and ch 2 (COMP) -> bus 2 from the same input stay
/// aligned at the bus outputs; with both slots at mix 0 % the busses are identical.
#[test]
fn busses_fed_by_different_models_are_aligned() {
    let run = |mix: f32| {
        let store = ParamStore::new();
        for (ch, mdl, bus) in [(1, "NSTR", 1), (2, "COMP", 2)] {
            connect_usb(&store, &format!("ch/{ch}"), 1);
            setf(&store, &format!("ch/{ch}/fdr"), 0.0);
            sett(&store, &format!("ch/{ch}/dyn/mdl"), mdl);
            seti(&store, &format!("ch/{ch}/dyn/on"), 1);
            setf(&store, &format!("ch/{ch}/dyn/mix"), mix);
            seti(&store, &format!("ch/{ch}/main/1/on"), 0);
            seti(&store, &format!("ch/{ch}/send/{bus}/on"), 1);
            setf(&store, &format!("ch/{ch}/send/{bus}/lvl"), 0.0);
            sett(&store, &format!("ch/{ch}/send/{bus}/mode"), "PRE");
            setf(&store, &format!("bus/{bus}/fdr"), 0.0);
        }
        let mut r = rig(&store, MixConfig::default(), 2);
        let x = programme(200 * BLOCK, 11);
        let (mut b1, mut b2) = (Vec::new(), Vec::new());
        for blk in x.chunks(BLOCK) {
            // broadband programme on USB 1 (a sharp cross-correlation peak)
            r.e.external_mut(0).copy_from_slice(blk);
            r.pipe.apply(&mut r.e);
            r.e.process_cycle();
            b1.extend(r.mix(Strip::Bus(1), M::Pfl, Side::L));
            b2.extend(r.mix(Strip::Bus(2), M::Pfl, Side::L));
        }
        let tail = b1.len() - 4096;
        (b1[tail..].to_vec(), b2[tail..].to_vec())
    };
    let (b1, b2) = run(0.0);
    assert!(rms(&b1) > 1e-3);
    assert_eq!(b1, b2, "dry paths through NSTR / COMP slots");
    let (b1, b2) = run(100.0);
    assert_eq!(xcorr_lag(&b1, &b2, 100), 0);
}
