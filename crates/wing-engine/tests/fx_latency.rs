//! T05U: an FX slot's latency follows its loaded model and the engine re-aligns every strip of a family to
//! the largest latency in use, click-free. Expected latencies are derived from the engine (`FxRack`,
//! standalone strips), never hard-coded.

mod common;

use common::rig::*;
use wing_engine::channel::ChannelStrip;
use wing_engine::fx::FxRack;
use wing_engine::mix::{InputSignal as I, MixConfig, MixSignal as M, MixStrip};
use wing_engine::routing::{Side, Strip};
use wing_model::ParamStore;

/// Latency of an input strip without inserts (its model slots at their constant latency).
fn input_base() -> usize {
    ChannelStrip::channel(48_000.0, BLOCK).latency()
}

fn sine(n: usize, f: f32, a: f32) -> Vec<f32> {
    (0..n)
        .map(|i| a * (i as f32 * 2.0 * std::f32::consts::PI * f / 48_000.0).sin())
        .collect()
}

/// Largest second difference (a step or kink shows up far above a smooth signal's own).
fn d2(s: &[f32]) -> f32 {
    s.windows(3)
        .map(|w| (w[2] - 2.0 * w[1] + w[0]).abs())
        .fold(0.0f32, f32::max)
}

fn rms(x: &[f32]) -> f64 {
    (x.iter().map(|&v| v as f64 * v as f64).sum::<f64>() / x.len().max(1) as f64).sqrt()
}

/// `x` delayed by `d` samples, block `k` (silence before the start).
fn delayed_block(x: &[f32], k: usize, d: usize) -> Vec<f32> {
    (k * BLOCK..(k + 1) * BLOCK)
        .map(|i| i.checked_sub(d).map_or(0.0, |j| x[j]))
        .collect()
}

/// ch 1 (Ins1 = FX3, fxmix 0 = dry through the slot) and ch 2 (polarity inverted, no insert) from the same
/// USB input, both sent to bus 1 at unity: the bus nulls exactly while the two strips are aligned.
fn aligned_pair(store: &ParamStore) {
    for ch in 1..=2 {
        let p = format!("ch/{ch}");
        connect_usb(store, &p, 1);
        for k in [
            "gate/on", "eq/on", "dyn/on", "peq/on", "flt/lc", "flt/hc", "flt/tf",
        ] {
            seti(store, &format!("{p}/{k}"), 0);
        }
        setf(store, &format!("{p}/fdr"), 0.0);
        seti(store, &format!("{p}/main/1/on"), 0);
        seti(store, &format!("{p}/send/1/on"), 1);
        setf(store, &format!("{p}/send/1/lvl"), 0.0);
        sett(store, &format!("{p}/send/1/mode"), "POST");
    }
    seti(store, "ch/2/in/set/inv", 1);
    seti(store, "ch/1/preins/on", 1);
    sett(store, "ch/1/preins/ins", "FX3");
    sett(store, "fx/3/mdl", "GEQ");
    setf(store, "fx/3/fxmix", 0.0);
    setf(store, "bus/1/fdr", 0.0);
    seti(store, "bus/1/eq/on", 0);
    seti(store, "bus/1/dyn/on", 0);
}

struct Run {
    /// ch 1 / ch 2 PFL (left) and bus 1 PFL (left), per block.
    ch1: Vec<f32>,
    ch2: Vec<f32>,
    bus: Vec<f32>,
}

/// Run `x` (USB 1) through the rig; `at(block, store)` may change parameters before a block.
fn run(r: &mut Rig, store: &ParamStore, x: &[f32], mut at: impl FnMut(usize, &ParamStore)) -> Run {
    let mut out = Run {
        ch1: Vec::new(),
        ch2: Vec::new(),
        bus: Vec::new(),
    };
    for (k, blk) in x.chunks(BLOCK).enumerate() {
        at(k, store);
        r.e.external_mut(0).copy_from_slice(blk);
        r.pipe.apply(&mut r.e);
        r.e.process_cycle();
        out.ch1.extend(r.input(0, I::Pfl, Side::L));
        out.ch2.extend(r.input(1, I::Pfl, Side::L));
        out.bus.extend(r.mix(Strip::Bus(1), M::Pfl, Side::L));
    }
    out
}

/// Input-family target the engine should publish: base strip latency + the largest latency of the slots
/// owned by any input strip (derived from the rack).
fn expected_input_target(fx: &FxRack) -> usize {
    let mut ins = [0usize; 48];
    for n in 1..=16u8 {
        if let Some((s, _)) = fx.owner(n) {
            if let Some(i) = s.index().filter(|&i| i < 48) {
                ins[i] += fx.slot_latency(n);
            }
        }
    }
    input_base() + ins.iter().copied().max().unwrap_or(0)
}

/// A zero-latency model adds nothing; PCORR adds its latency to every input strip (all re-aligned, the
/// bus nulls exactly); going back removes it again. The switches are click-free in both strips.
#[test]
fn slot_latency_follows_model_and_strips_stay_aligned() {
    let store = ParamStore::new();
    aligned_pair(&store);
    let mut r = rig(&store, MixConfig::default(), 2);
    // zero-latency model (GEQ): no added latency
    assert_eq!(r.fx.slot_latency(3), 0);
    assert_eq!(r.fx.align_target(Strip::Ch(1)), input_base());
    assert_eq!(r.fx.align_target(Strip::Ch(2)), input_base());

    let x = sine(400 * BLOCK, 500.0, 0.25);
    let (on, off) = (150, 270);
    let res = run(&mut r, &store, &x, |k, s| {
        if k == on {
            sett(s, "fx/3/mdl", "PCORR");
        } else if k == off {
            sett(s, "fx/3/mdl", "GEQ");
        }
    });
    // steady states before the switch and after switching back (the PCORR state is checked below)
    let check = |k: usize, d: usize, what: &str| {
        let s = k * BLOCK..(k + 1) * BLOCK;
        assert_eq!(
            res.ch1[s.clone()],
            delayed_block(&x, k, d)[..],
            "{what} ch1"
        );
        assert_eq!(
            res.ch2[s.clone()],
            delayed_block(&x, k, d)
                .iter()
                .map(|v| -v)
                .collect::<Vec<_>>()[..],
            "{what} ch2"
        );
        assert!(rms(&res.bus[s]) == 0.0, "{what}: bus does not null");
    };
    check(on - 1, input_base(), "zero-latency model");
    check(off + 30, input_base(), "back to zero latency");
    assert!(rms(&res.ch1[(on - 10) * BLOCK..on * BLOCK]) > 0.1);

    // smooth across both switches (ch 1: slot dry-path crossfade; ch 2: alignment pad crossfade)
    for (name, y) in [("ch1", &res.ch1), ("ch2", &res.ch2)] {
        let steady = d2(&y[100 * BLOCK..on * BLOCK]);
        for k in [on, off] {
            let around = d2(&y[k * BLOCK..(k + 20) * BLOCK]);
            assert!(
                around < 2.0 * steady,
                "{name} switch at block {k}: {around} vs {steady}"
            );
        }
    }

    // PCORR loaded: the slot reports its latency, every input strip aligns to base + PCORR
    sett(&store, "fx/3/mdl", "PCORR");
    let y = run(&mut r, &store, &x[..60 * BLOCK], |_, _| {});
    let pc = r.fx.slot_latency(3);
    assert!(pc > 0, "PCORR has latency");
    let target = r.fx.align_target(Strip::Ch(1));
    assert_eq!(target, input_base() + pc);
    assert_eq!(target, expected_input_target(&r.fx));
    assert_eq!(r.fx.align_target(Strip::Aux(8)), target);
    let k = 59;
    let s = k * BLOCK..(k + 1) * BLOCK;
    assert_eq!(
        y.ch1[s.clone()],
        delayed_block(&x, k, target)[..],
        "PCORR ch1"
    );
    assert_eq!(rms(&y.bus[s.clone()]), 0.0, "PCORR: bus does not null");
    assert!(rms(&y.ch2[s]) > 0.1);
    // mix strips are not affected by an input-strip insert
    assert_eq!(
        r.fx.align_target(Strip::Bus(1)),
        MixStrip::new(Strip::Bus(1), 48_000.0, BLOCK).latency()
    );
    // the click measure is sensitive: a hard switch between the two delays steps far above the signal
    let steady = d2(&res.ch1[100 * BLOCK..on * BLOCK]);
    let hard: Vec<f32> = (on - 2..on + 2)
        .flat_map(|k| delayed_block(&x, k, if k < on { input_base() } else { target }))
        .collect();
    assert!(d2(&hard) > 20.0 * steady, "{} vs {steady}", d2(&hard));
}

/// The switching strip is not the family maximum (ch 3 already runs PCORR on FX4): ch 1's slot latency
/// rises while its pad shrinks by the same amount (both crossfaded), the family target stays, the bus
/// keeps nulling before and after and nothing clicks.
#[test]
fn realign_below_family_maximum() {
    let store = ParamStore::new();
    aligned_pair(&store);
    connect_usb(&store, "ch/3", 2);
    seti(&store, "ch/3/main/1/on", 0);
    seti(&store, "ch/3/preins/on", 1);
    sett(&store, "ch/3/preins/ins", "FX4");
    sett(&store, "fx/4/mdl", "PCORR");
    let mut r = rig(&store, MixConfig::default(), 2);
    let target = r.fx.align_target(Strip::Ch(1));
    assert_eq!(target, input_base() + r.fx.slot_latency(4));
    assert_eq!(target, expected_input_target(&r.fx));
    let x = sine(300 * BLOCK, 500.0, 0.25);
    let on = 150;
    let res = run(&mut r, &store, &x, |k, s| {
        if k == on {
            sett(s, "fx/3/mdl", "PCORR");
        }
    });
    assert_eq!(r.fx.align_target(Strip::Ch(1)), target);
    assert_eq!(r.fx.slot_latency(3), r.fx.slot_latency(4));
    for k in [on - 1, 299] {
        let s = k * BLOCK..(k + 1) * BLOCK;
        assert_eq!(
            res.ch1[s.clone()],
            delayed_block(&x, k, target)[..],
            "block {k}"
        );
        assert_eq!(rms(&res.bus[s]), 0.0, "block {k}: bus does not null");
    }
    let steady = d2(&res.ch1[100 * BLOCK..on * BLOCK]);
    let around = d2(&res.ch1[on * BLOCK..(on + 20) * BLOCK]);
    assert!(around < 2.0 * steady, "{around} vs {steady}");
    assert!(rms(&res.ch1[(on + 1) * BLOCK..(on + 6) * BLOCK]) > 0.05);
}

/// Typical latency of PUBLIC.snap (fw 1.13 scene: PCORR on ch 21 / ch 22 inserts): every strip of a family
/// is aligned to the family target = the largest own + owned-insert latency in it. Prints the figures
/// recorded in the T05U log (`cargo test --test fx_latency -- --nocapture`).
#[test]
fn public_snap_alignment_targets() {
    let store = public_store();
    let mut r = rig(&store, MixConfig::default(), 0);
    r.cycles(2, &[]);
    let base_in = input_base();
    let base_mix = MixStrip::new(Strip::Bus(1), 48_000.0, BLOCK).latency();
    let mut demand = [0usize; 76];
    let mut worst = [0usize; 76];
    for n in 1..=16u8 {
        if let Some((s, _)) = r.fx.owner(n) {
            let i = s.index().unwrap();
            demand[i] += r.fx.slot_latency(n);
            // T03E policy: every slot at the max over the models it can host
            worst[i] += unsafe { r.fx.with_slot(n, |s| s.model_slot().max_latency()) };
            eprintln!(
                "FX{n}: {:?} model {} latency {} (max {})",
                r.fx.owner(n),
                r.fx.slot_model(n),
                r.fx.slot_latency(n),
                unsafe { r.fx.with_slot(n, |s| s.model_slot().max_latency()) }
            );
        }
    }
    let fam = [
        (0..48, base_in, "input"),
        (48..64, base_mix, "bus"),
        (64..68, base_mix, "main"),
        (68..76, base_mix, "mtx"),
    ];
    for (range, base, name) in fam {
        let t = base + range.clone().map(|i| demand[i]).max().unwrap();
        let s = Strip::from_index(range.start).unwrap();
        assert_eq!(r.fx.align_target(s), t, "{name}");
        let before: Vec<usize> = range.clone().map(|i| base + worst[i]).collect();
        let (lo, hi) = (before.iter().min().unwrap(), before.iter().max().unwrap());
        eprintln!(
            "{name}: T05U aligned at {t} samples ({:.2} ms); before (T03E slot max, unaligned) {lo}..{hi}",
            t as f64 / 48.0
        );
    }
}
