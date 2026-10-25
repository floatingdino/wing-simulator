//! T05V vintage reverb tests (V-ROOM, V-REV, VSS3): T60 vs `dcy` and the band multipliers (+-10 %), freeze,
//! front/rear outputs, transformer, band-limit, VSS3 early reflections, echo density, decorrelation, silence /
//! denormals, stability at extreme settings, click-free sweeps, parameter round trip.

use super::tests::{corr, echo_density, hf_trace, ir, noise, peak, run, t60, FS};
use super::*;
use crate::model::{BlockKind, Model};
use wing_dsp::biquad::{Biquad, Coeffs};

const IDS: [&str; 3] = ["V-ROOM", "V-REV", "VSS3"];

fn params(id: &str) -> &'static [ParamSpec] {
    match id {
        "V-ROOM" => &vintage::V_ROOM_PARAMS,
        "V-REV" => &vintage::V_REV_PARAMS,
        _ => &vss3::PARAMS,
    }
}

fn model(id: &str) -> Box<dyn Model> {
    let id: &'static str = IDS.iter().find(|&&x| x == id).copied().unwrap();
    let mut m = crate::model::create_model(BlockKind::Fx, id);
    assert!(!m.is_stub(), "{id}");
    m.prepare(FS, 64);
    m
}

fn set(m: &mut dyn Model, id: &str, key: &str, v: f32) {
    let s = params(id).iter().position(|p| p.key == key).unwrap() as u16 + 1;
    m.set_param(s, ParamValue::Float(v));
}

/// Wide-open filters, no early reflections (tail only), unity band multipliers where they exist.
fn neutral(m: &mut dyn Model, id: &str) {
    match id {
        "V-ROOM" => {
            set(m, id, "lc", 20.0);
            set(m, id, "hc", 20_000.0);
            set(m, id, "erlvl", 0.0);
            set(m, id, "lmult", 1.0);
            set(m, id, "hmult", 1.0);
        }
        "V-REV" => {
            set(m, id, "lc", 20.0);
            set(m, id, "hc", 20_000.0);
            set(m, id, "lmult", 1.0);
        }
        _ => {
            set(m, id, "erlvl", -144.0);
            set(m, id, "rvlvl", 0.0);
            set(m, id, "hcut", 20_000.0);
            set(m, id, "lsdmp", 0.0);
            set(m, id, "hsoft", -50.0);
            for k in ["ldcy", "lmdcy", "hmdcy", "hdcy"] {
                set(m, id, k, 1.0);
            }
        }
    }
}

fn check(id: &str, what: &str, got: f64, want: f64) {
    let err = got / want - 1.0;
    eprintln!(
        "{id} {what}: T60 {got:.3} s (want {want:.3}, {:+.1} %)",
        err * 100.0
    );
    assert!(err.abs() <= 0.10, "{id} {what}: {got} vs {want}");
}

#[test]
fn t60_tracks_decay() {
    for (id, d) in [
        ("V-ROOM", 1.5),
        ("V-ROOM", 0.6),
        ("V-ROOM", 5.0),
        ("V-REV", 2.0),
        ("V-REV", 0.8),
        ("V-REV", 4.5),
        ("VSS3", 2.0),
        ("VSS3", 0.7),
        ("VSS3", 6.0),
    ] {
        let mut m = model(id);
        neutral(&mut *m, id);
        set(&mut *m, id, "dcy", d);
        let (l, r) = ir(&mut *m, 1.6 * d as f64 + 0.3);
        // V-REV's HF multiplier is always < 1 (corner 3 kHz): mid band only up to 500 Hz
        let bands: &[f64] = if id == "V-REV" {
            &[250.0, 500.0]
        } else {
            &[250.0, 1000.0, 2000.0]
        };
        for &f in bands {
            check(id, &format!("dcy {d} {f} Hz L"), t60(&l, f), d as f64);
        }
        check(id, &format!("dcy {d} 250 Hz R"), t60(&r, 250.0), d as f64);
    }
}

#[test]
fn t60_tracks_size_extremes() {
    for size in [1.0, 50.0] {
        let mut m = model("V-ROOM");
        neutral(&mut *m, "V-ROOM");
        set(&mut *m, "V-ROOM", "dcy", 1.2);
        set(&mut *m, "V-ROOM", "size", size);
        let (l, _) = ir(&mut *m, 2.2);
        check(
            "V-ROOM",
            &format!("size {size} 1 kHz"),
            t60(&l, 1000.0),
            1.2,
        );
    }
    // VSS3 reverb types change the geometry, not the decay
    for ty in 0..6 {
        let mut m = model("VSS3");
        neutral(&mut *m, "VSS3");
        set(&mut *m, "VSS3", "dcy", 1.5);
        set(&mut *m, "VSS3", "rvtype", ty as f32);
        let (l, _) = ir(&mut *m, 2.7);
        check("VSS3", &format!("rvtype {ty} 1 kHz"), t60(&l, 1000.0), 1.5);
    }
}

#[test]
fn t60_bass_and_treble_multipliers() {
    // V-ROOM: lmult below 350 Hz, hmult above 2 kHz, both directions. An HF T60 longer than the mid one is
    // limited by the interpolation loss of the modulated reads in the top octaves (more passes per second at
    // small sizes): measured at 6 kHz in the largest room.
    for (lm, hm) in [(2.0, 0.4), (0.5, 2.0)] {
        let d = 1.5;
        for size in [15.0, 50.0] {
            let mut m = model("V-ROOM");
            neutral(&mut *m, "V-ROOM");
            set(&mut *m, "V-ROOM", "size", size);
            set(&mut *m, "V-ROOM", "dcy", d);
            set(&mut *m, "V-ROOM", "lmult", lm);
            set(&mut *m, "V-ROOM", "hmult", hm);
            let (l, _) = ir(&mut *m, 1.6 * 2.0 * d as f64 + 0.3);
            let w = format!("size {size} lmult {lm} hmult {hm}");
            if size < 20.0 {
                check(
                    "V-ROOM",
                    &format!("{w} 63 Hz"),
                    t60(&l, 63.0),
                    (lm * d) as f64,
                );
                check("V-ROOM", &format!("{w} 900 Hz"), t60(&l, 900.0), d as f64);
            } else {
                check(
                    "V-ROOM",
                    &format!("{w} 6 kHz"),
                    t60(&l, 6000.0),
                    (hm * d) as f64,
                );
            }
        }
    }
    // V-REV: lmult below 500 Hz (measured 2.3 octaves below: the 8 sparse, chorused lines leave few modes in
    // lower octaves), hmult above 3 kHz
    for (lm, hm) in [(2.0, 0.25), (0.5, 0.5)] {
        let d = 2.0;
        let mut m = model("V-REV");
        neutral(&mut *m, "V-REV");
        set(&mut *m, "V-REV", "dcy", d);
        set(&mut *m, "V-REV", "lmult", lm);
        set(&mut *m, "V-REV", "hmult", hm);
        let (l, _) = ir(&mut *m, 1.6 * 2.0 * d as f64 + 0.3);
        let w = format!("lmult {lm} hmult {hm}");
        check(
            "V-REV",
            &format!("{w} 100 Hz"),
            t60(&l, 100.0),
            (lm * d) as f64,
        );
        check(
            "V-REV",
            &format!("{w} 9 kHz"),
            t60(&l, 9000.0),
            (hm * d) as f64,
        );
    }
}

#[test]
fn vss3_four_band_decay() {
    let d = 2.0;
    // (multipliers, crossovers, measurement frequency per band); a band that decays faster than its neighbour
    // needs more distance from the crossover (the neighbour's slower decay leaks into the octave filter)
    for (mults, xo, fc) in [
        (
            [2.0, 1.0, 0.5, 0.25],
            [100.0, 1000.0, 4000.0],
            [40.0, 316.0, 2000.0, 10_000.0],
        ),
        (
            [0.75, 1.5, 1.0, 0.6],
            [150.0, 1000.0, 6000.0],
            [40.0, 387.0, 2450.0, 10_000.0],
        ),
    ] {
        let mut m = model("VSS3");
        neutral(&mut *m, "VSS3");
        set(&mut *m, "VSS3", "dcy", d);
        for (k, v) in ["ldcy", "lmdcy", "hmdcy", "hdcy"].iter().zip(mults) {
            set(&mut *m, "VSS3", k, v);
        }
        for (k, v) in ["lxo", "mxo", "hxo"].iter().zip(xo) {
            set(&mut *m, "VSS3", k, v);
        }
        let longest = mults.iter().fold(0f32, |a, &b| a.max(b)) * d;
        let (l, _) = ir(&mut *m, 1.6 * longest as f64 + 0.3);
        for (b, (&f, &mu)) in fc.iter().zip(&mults).enumerate() {
            check(
                "VSS3",
                &format!("{mults:?} band {b} @ {f:.0} Hz"),
                t60(&l, f),
                (mu * d) as f64,
            );
        }
    }
}

fn band_energy(x: &[f32], f0: f64) -> f64 {
    let c = Coeffs::bandpass(FS, f0, 4.0);
    let (mut a, mut b) = (Biquad::new(c), Biquad::new(c));
    x.iter().map(|&v| (b.tick(a.tick(v)) as f64).powi(2)).sum()
}

fn rms(x: &[f32]) -> f64 {
    (x.iter().map(|&v| (v as f64).powi(2)).sum::<f64>() / x.len() as f64).sqrt()
}

#[test]
fn v_room_freeze_holds_and_mutes_input() {
    let id = "V-ROOM";
    // output of `secs` of noise / silence, band-passed at 500 Hz (the frozen loops are lossless there; the
    // interpolation of the modulated reads still takes the top octaves away slowly)
    let blk = |m: &mut dyn Model, secs: f64, input: bool| {
        let n = (secs * FS) as usize;
        let (mut l, mut r) = if input {
            (noise(n, 3), noise(n, 4))
        } else {
            (vec![0.0; n], vec![0.0; n])
        };
        run(m, &mut l, &mut r);
        let c = Coeffs::bandpass(FS, 500.0, 2.0);
        let (mut a, mut b) = (Biquad::new(c), Biquad::new(c));
        l.iter().map(|&v| b.tick(a.tick(v))).collect::<Vec<f32>>()
    };
    let mut m = model(id);
    neutral(&mut *m, id);
    set(&mut *m, id, "dcy", 1.0);
    blk(&mut *m, 0.5, true);
    set(&mut *m, id, "frz", 1.0);
    // freeze engaged while the input keeps running: it is muted, the room holds
    let a = rms(&blk(&mut *m, 0.5, true)[12_000..]);
    let b = rms(&blk(&mut *m, 3.0, true)[(2.5 * FS) as usize..]);
    let db = 20.0 * (b / a).log10();
    eprintln!("V-ROOM freeze: {a:.4} -> {b:.4} ({db:+.2} dB over 3 s)");
    assert!(db.abs() < 1.5, "freeze not holding: {db} dB");
    // without freeze the same 3 s would have decayed by 180 dB; release -> decays again
    set(&mut *m, id, "frz", 0.0);
    let c = rms(&blk(&mut *m, 2.0, false)[(1.5 * FS) as usize..]);
    assert!(c < 0.01 * b, "release: {c} vs {b}");

    // ADD: input keeps feeding the frozen room -> it builds up instead of holding
    let mut m = model(id);
    neutral(&mut *m, id);
    set(&mut *m, id, "dcy", 1.0);
    blk(&mut *m, 0.5, true);
    set(&mut *m, id, "frz", 1.0);
    set(&mut *m, id, "add", 1.0);
    let a2 = rms(&blk(&mut *m, 0.5, true)[12_000..]);
    let b2 = rms(&blk(&mut *m, 3.0, true)[(2.5 * FS) as usize..]);
    eprintln!("V-ROOM freeze+add: {a2:.4} -> {b2:.4}");
    assert!(b2 > 1.5 * a2, "add does not layer: {a2} {b2}");
}

#[test]
fn v_room_first_reflections_and_level() {
    let id = "V-ROOM";
    let run_with = |kv: &[(&str, f32)]| {
        let mut m = model(id);
        neutral(&mut *m, id);
        set(&mut *m, id, "pdel", 150.0);
        for &(k, v) in kv {
            set(&mut *m, id, k, v);
        }
        ir(&mut *m, 0.3)
    };
    // ER left at 60 ms, right at 70 ms; pre-delay 150 ms keeps the ER bank out of the way and size 50 (shortest
    // line 22 ms) keeps the reflections' own tail (fed back in at 60 + 22 ms) after both
    let (l, r) = run_with(&[
        ("erlvl", 100.0),
        ("erl", 60.0),
        ("err", 70.0),
        ("size", 50.0),
    ]);
    let first = |x: &[f32]| x.iter().position(|v| v.abs() > 1e-3).unwrap();
    let (fl, fr) = (first(&l), first(&r));
    eprintln!("V-ROOM first reflection L {fl} R {fr}");
    // the band-limit filter delays the onset by a few samples
    assert!((2880..2895).contains(&fl), "{fl}");
    assert!((3360..3375).contains(&fr), "{fr}");
    // LEVEL scales the whole wet output
    let (a, _) = run_with(&[("lvl", 0.0)]);
    let (b, _) = run_with(&[("lvl", 6.0)]);
    let g = rms(&b) / rms(&a);
    assert!((g - 10f64.powf(0.3)).abs() < 0.01, "{g}");
}

#[test]
fn v_rev_rear_outputs_and_transformer() {
    let id = "V-REV";
    let mut f = model(id);
    neutral(&mut *f, id);
    let (fl, _) = ir(&mut *f, 3.0);
    let mut r = model(id);
    neutral(&mut *r, id);
    set(&mut *r, id, "out", 1.0);
    let (rl, _) = ir(&mut *r, 3.0);
    // same decay, different (decorrelated) tap set
    check(id, "rear 250 Hz", t60(&rl, 250.0), t60(&fl, 250.0));
    let s = (0.1 * FS) as usize;
    let c = corr(&fl[s..], &rl[s..], 0).abs();
    eprintln!("V-REV front/rear correlation {c:.3}");
    assert!(c < 0.3, "{c}");
    // rear: no early reflections -> less energy in the first 40 ms
    let e = |x: &[f32]| x[..(0.06 * FS) as usize].iter().map(|v| v * v).sum::<f32>();
    assert!(e(&rl) < e(&fl), "{} {}", e(&rl), e(&fl));

    // transformer: transparent at low level, compresses hot signals
    let level = |trans: f32, amp: f32| {
        let mut m = model(id);
        set(&mut *m, id, "trans", trans);
        m.reset();
        let n = (1.0 * FS) as usize;
        let (mut l, mut r) = (noise(n, 5), noise(n, 6));
        l.iter_mut().chain(r.iter_mut()).for_each(|v| *v *= amp);
        run(&mut *m, &mut l, &mut r);
        rms(&l[n / 2..])
    };
    let quiet = level(1.0, 0.01) / level(0.0, 0.01);
    let hot = level(1.0, 4.0) / level(0.0, 4.0);
    eprintln!("V-REV transformer gain: quiet {quiet:.4}, hot {hot:.4}");
    assert!((quiet - 1.0).abs() < 0.01, "{quiet}");
    assert!(hot < 0.8, "{hot}");
}

#[test]
fn vintage_models_are_band_limited() {
    for (id, bw) in [("V-ROOM", 15_000.0), ("V-REV", 12_000.0)] {
        let mut m = model(id);
        neutral(&mut *m, id);
        set(&mut *m, id, "dcy", 1.5);
        if id == "V-REV" {
            set(&mut *m, id, "hmult", 0.67);
        } else {
            set(&mut *m, id, "hmult", 1.0);
        }
        let (l, _) = ir(&mut *m, 1.0);
        let lo = band_energy(&l, 1000.0);
        let hi = band_energy(&l, (bw * 1.45f64).min(21_000.0));
        let db = 10.0 * (hi / lo).log10();
        eprintln!("{id}: energy above the bandwidth {db:.1} dB re 1 kHz");
        assert!(db < -20.0, "{id}: {db}");
    }
}

#[test]
fn vss3_early_reflections() {
    let id = "VSS3";
    let er_only = |kv: &[(&str, f32)]| {
        let mut m = model(id);
        neutral(&mut *m, id);
        set(&mut *m, id, "rvlvl", -144.0);
        set(&mut *m, id, "erlvl", 0.0);
        set(&mut *m, id, "ercol", 0.0);
        for &(k, v) in kv {
            set(&mut *m, id, k, v);
        }
        ir(&mut *m, 0.6)
    };
    let onset = |x: &[f32]| x.iter().position(|v| v.abs() > 1e-3).unwrap();
    let last = |x: &[f32]| x.len() - x.iter().rev().position(|v| v.abs() > 1e-3).unwrap();
    // CHURCH LRG NEAR: first tap at erpdly + first_ms x 1.6
    let t = vss3::ER_TYPES[2];
    for pd in [0.0f32, 50.0] {
        let (l, _) = er_only(&[("ertype", 2.0), ("ersize", 2.0), ("erpdly", pd)]);
        let want = ((pd + t.first_ms * 1.6) * 48.0) as usize;
        let got = onset(&l);
        eprintln!("VSS3 ER onset (erpdly {pd}): {got} (first tap >= {want})");
        assert!(got + 2 >= want && got < want + (0.1 * t.len_ms * 1.6 * 48.0) as usize);
    }
    // size scales the pattern length
    let (s, _) = er_only(&[("ertype", 2.0), ("ersize", 0.0)]);
    let (b, _) = er_only(&[("ertype", 2.0), ("ersize", 2.0)]);
    let (ls, lb) = (last(&s) - onset(&s), last(&b) - onset(&b));
    eprintln!("VSS3 ER span SML {ls} LRG {lb}");
    assert!(lb as f32 > 2.0 * ls as f32, "{ls} {lb}");
    // DIST starts later than NEAR
    let (n, _) = er_only(&[("ertype", 4.0), ("erpos", 0.0)]);
    let (d, _) = er_only(&[("ertype", 4.0), ("erpos", 1.0)]);
    assert!(onset(&d) > onset(&n) + 100, "{} {}", onset(&n), onset(&d));
    // types differ, L/R differ, balance
    let (a, ar) = er_only(&[("ertype", 0.0)]);
    let (c, _) = er_only(&[("ertype", 13.0)]);
    assert!(corr(&a, &c, 0).abs() < 0.5);
    assert!(corr(&a, &ar, 0).abs() < 0.5);
    let (bl, br) = er_only(&[("erbal", 100.0)]);
    assert!(peak(&bl) < 1e-6 && peak(&br) > 0.05);
    // no ER energy in the tail-only setting
    let (z, _) = er_only(&[("erlvl", -144.0)]);
    assert_eq!(peak(&z), 0.0);
    // tables are energy-normalised
    for ty in 0..24 {
        for side in 0..2 {
            let tab = vss3::er_table(ty, 1, ty % 2 == 0, side);
            let e: f32 = tab.gain.iter().map(|g| g * g).sum();
            assert!((e - 1.0).abs() < 1e-4, "{ty} {e}");
        }
    }
}

#[test]
fn echo_density_and_decorrelation() {
    for id in IDS {
        let mut m = model(id);
        neutral(&mut *m, id);
        set(
            &mut *m,
            id,
            if id == "VSS3" { "rvpdly" } else { "pdel" },
            0.0,
        );
        let (l, r) = ir(&mut *m, 1.0);
        let ned = 0.5 * (echo_density(&l, 0.25, 1024) + echo_density(&r, 0.25, 1024));
        let s = (0.05 * FS) as usize;
        let mut worst = 0f64;
        for lag in -48..=48 {
            worst = worst.max(corr(&l[s..], &r[s..], lag).abs());
        }
        eprintln!("{id}: echo density {ned:.2}, max |IACC| {worst:.3}");
        assert!(ned > 0.75, "{id}: {ned}");
        assert!(worst < 0.3, "{id}: {worst}");
    }
}

#[test]
fn silence_decays_to_exact_zero_without_denormals() {
    for id in IDS {
        let mut m = model(id);
        let (mut l, mut r) = (vec![0.0f32; 64], vec![0.0f32; 64]);
        l[0] = 1.0;
        r[3] = -0.5;
        m.process(&mut l, &mut r, None);
        let blocks = (30.0 * FS) as usize / 64;
        let mut last_nonzero = 0;
        for b in 0..blocks {
            let (mut l, mut r) = ([0.0f32; 64], [0.0f32; 64]);
            m.process(&mut l, &mut r, None);
            for &v in l.iter().chain(r.iter()) {
                assert!(v.is_finite(), "{id}");
                assert!(v == 0.0 || v.abs() >= f32::MIN_POSITIVE, "{id}: subnormal");
                if v != 0.0 {
                    last_nonzero = b;
                }
            }
        }
        let t = last_nonzero as f64 * 64.0 / FS;
        eprintln!("{id}: output exactly zero after {t:.1} s");
        assert!(t < 28.0, "{id}: still ringing at {t} s");
    }
}

#[test]
fn stable_and_finite_at_extreme_settings() {
    for id in IDS {
        for hi in [true, false] {
            let mut m = model(id);
            for (i, p) in params(id).iter().enumerate() {
                let v = if hi { p.max } else { p.min };
                m.set_param(i as u16 + 1, ParamValue::Float(v));
            }
            if id == "V-ROOM" {
                set(&mut *m, id, "frz", 0.0); // a frozen room never decays (covered above)
            }
            m.reset();
            let n = (4.0 * FS) as usize;
            let mut l = noise(n, 1);
            let mut r = noise(n, 2);
            for v in l.iter_mut().chain(r.iter_mut()).skip(n / 2) {
                *v = 0.0;
            }
            run(&mut *m, &mut l, &mut r);
            let pk = peak(&l).max(peak(&r));
            eprintln!("{id} {}: peak {pk:.2}", if hi { "max" } else { "min" });
            assert!(pk.is_finite() && pk < 50.0, "{id}: {pk}");
        }
    }
    // VSS3 with reversed crossovers and extreme multipliers (core keeps the shelf corners ordered)
    let mut m = model("VSS3");
    for (k, v) in [
        ("lxo", 500.0),
        ("mxo", 200.0),
        ("hxo", 500.0),
        ("ldcy", 2.5),
        ("lmdcy", 0.1),
        ("hmdcy", 2.5),
        ("hdcy", 2.5),
        ("dcy", 20.0),
    ] {
        set(&mut *m, "VSS3", k, v);
    }
    m.reset();
    let n = (6.0 * FS) as usize;
    let (mut l, mut r) = (noise(n, 1), noise(n, 2));
    for v in l.iter_mut().chain(r.iter_mut()).skip(FS as usize) {
        *v = 0.0;
    }
    run(&mut *m, &mut l, &mut r);
    let (a, b) = (peak(&l[n / 2..3 * n / 4]), peak(&l[3 * n / 4..]));
    eprintln!("VSS3 reversed crossovers: {a:.3} -> {b:.3}");
    assert!(b.is_finite() && b <= a * 1.05, "{a} {b}");
}

#[test]
fn wet_level_is_sane_at_defaults() {
    for id in IDS {
        let mut m = model(id);
        let n = (3.0 * FS) as usize;
        let mut l = noise(n, 7);
        let mut r = noise(n, 9);
        let rin = rms(&l);
        run(&mut *m, &mut l, &mut r);
        let db = 20.0 * (rms(&l[n / 2..]) / rin).log10();
        eprintln!("{id}: wet/dry RMS at defaults {db:.1} dB");
        assert!((-24.0..=3.0).contains(&db), "{id}: {db} dB");
    }
}

#[test]
fn parameter_sweeps_do_not_click() {
    let blocks_per_sweep = (0.5 * FS) as usize / 64;
    for id in IDS {
        for (i, p) in params(id).iter().enumerate() {
            let slot = i as u16 + 1;
            let mut m = model(id);
            match id {
                "VSS3" => set(&mut *m, id, "hcut", 20_000.0),
                _ => set(&mut *m, id, "hc", 20_000.0),
            }
            let mut ph = 0.0f32;
            let mut fade = 0.0f32;
            let mut block = |m: &mut dyn Model| {
                let mut l = [0.0f32; 64];
                for v in l.iter_mut() {
                    ph += std::f32::consts::TAU * 150.0 / FS as f32;
                    if ph > std::f32::consts::TAU {
                        ph -= std::f32::consts::TAU;
                    }
                    fade = (fade + 1.0 / (0.05 * FS as f32)).min(1.0);
                    *v = 0.25 * (1.0 - (std::f32::consts::PI * fade).cos()) * ph.sin();
                }
                let mut r = l;
                m.process(&mut l, &mut r, None);
                l
            };
            let mut out = Vec::new();
            m.set_param(slot, ParamValue::Float(p.min));
            for _ in 0..blocks_per_sweep {
                out.extend_from_slice(&block(&mut *m));
            }
            let sweep_start = out.len();
            for k in 0..=blocks_per_sweep {
                let t = k as f32 / blocks_per_sweep as f32;
                m.set_param(slot, ParamValue::Float(p.min + t * (p.max - p.min)));
                out.extend_from_slice(&block(&mut *m));
            }
            let steady2 = out.len();
            for _ in 0..blocks_per_sweep {
                out.extend_from_slice(&block(&mut *m));
            }
            let jump = out.len();
            m.set_param(slot, ParamValue::Float(p.min));
            for _ in 0..blocks_per_sweep {
                out.extend_from_slice(&block(&mut *m));
            }
            let hp = hf_trace(&out);
            let pk = |r: std::ops::Range<usize>| hp[r].iter().fold(0f32, |a, v| a.max(*v));
            let steady: Vec<f32> = out[..sweep_start]
                .iter()
                .chain(&out[steady2..jump])
                .copied()
                .collect();
            let base = pk(4096..sweep_start).max(pk(steady2 + 4096..jump));
            let hf = pk(sweep_start..steady2).max(pk(jump..out.len()));
            let level = steady.iter().fold(0f32, |a, v| a.max(v.abs())).max(1e-3);
            if hf / level > 0.001 {
                eprintln!(
                    "{id} {}: hf {hf:.5} steady {base:.5} level {level:.4}",
                    p.key
                );
            }
            assert!(
                hf <= 3.0 * base + 0.01 * level,
                "{id} {}: click {hf} (steady {base}, level {level})",
                p.key
            );
        }
    }
}

#[test]
fn params_round_trip_defaults_and_clamp() {
    for id in IDS {
        let ps = params(id);
        let mut m = create(id).unwrap();
        m.prepare(FS, 64);
        assert_eq!(m.id(), id);
        for (i, p) in ps.iter().enumerate() {
            assert!(
                p.min <= p.max && (p.min..=p.max).contains(&p.default),
                "{id} {}",
                p.key
            );
            assert_eq!(m.param(i as u16 + 1), Some(p.default), "{id} {}", p.key);
        }
        assert_eq!(m.param(0), None);
        assert_eq!(m.param(ps.len() as u16 + 1), None);
        m.set_param(1, ParamValue::Float(1e9));
        assert_eq!(m.param(1), Some(ps[0].max));
        m.set_param(1, ParamValue::Int(-5000));
        assert_eq!(m.param(1), Some(ps[0].min));
        m.set_param(1, ParamValue::Float(f32::NAN));
        assert_eq!(m.param(1), Some(ps[0].min));
        m.load_defaults();
        assert_eq!(m.param(1), Some(ps[0].default));
        assert_eq!(m.latency(), 0);
    }
    assert_eq!(vss3::PARAMS.len(), 32);
    // enum slots by index
    let mut v = vss3();
    v.set_param(4, ParamValue::Enum(23));
    assert_eq!(v.param(4), Some(23.0));
}

#[test]
fn vss3_long_blocks_are_chunked() {
    // a block longer than max_block is processed in chunks: same output as 64-sample blocks
    let mut a = vss3();
    a.prepare(FS, 64);
    let mut b = vss3();
    b.prepare(FS, 64);
    let n = 4096;
    let (mut l1, mut r1) = (noise(n, 11), noise(n, 12));
    let (mut l2, mut r2) = (l1.clone(), r1.clone());
    a.process(&mut l1, &mut r1, None);
    run(&mut b, &mut l2, &mut r2);
    let d = l1
        .iter()
        .zip(&l2)
        .fold(0f32, |m, (x, y)| m.max((x - y).abs()));
    assert!(d < 1e-6, "{d}");
}
