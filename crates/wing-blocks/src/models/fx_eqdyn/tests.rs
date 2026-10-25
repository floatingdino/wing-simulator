use super::*;
use crate::fidelity::{json, reference_dir};
use crate::model::{create_model, BlockKind};
use crate::models::eq_alt::{prims, MAX_SECS};
use crate::wingcoef::{self, Ctx};

const FS: f64 = 48_000.0;

/// `data/reference-curves/fx-eqdyn.json` (WING-Edit's own fx_parchg output, tools/curve-extract/fx_eqdyn.py).
fn oracle() -> json::Value {
    let p = reference_dir().join("fx-eqdyn.json");
    let t = std::fs::read_to_string(&p).unwrap_or_else(|e| panic!("{}: {e}", p.display()));
    json::parse(&t).expect("json")
}

fn nums(v: &json::Value) -> Vec<f64> {
    v.as_arr()
        .unwrap()
        .iter()
        .map(|x| x.as_f64().unwrap())
        .collect()
}

fn bqs(v: &json::Value) -> Vec<[f64; 5]> {
    v.as_arr()
        .unwrap()
        .iter()
        .map(|b| {
            let n = nums(b);
            [n[0], n[1], n[2], n[3], n[4]]
        })
        .collect()
}

/// Max relative coefficient difference (scaled by the section's largest coefficient).
fn coef_err(a: &[f32; 5], b: &[f64; 5]) -> f64 {
    let scale = b.iter().fold(1.0f64, |m, x| m.max(x.abs()));
    (0..5)
        .map(|i| (a[i] as f64 - b[i]).abs() / scale)
        .fold(0.0, f64::max)
}

fn db_err(a: &[Bq], b: &[[f64; 5]], parallel: bool) -> f64 {
    let mut worst = 0.0f64;
    let mut f = 20.0;
    while f < 20_000.0 {
        let ha = sum_or_cascade(a.iter().map(coef).collect(), parallel, f);
        let hb = sum_or_cascade(b.to_vec(), parallel, f);
        worst = worst.max((lin2db(ha) - lin2db(hb)).abs());
        f *= 1.05;
    }
    worst
}

fn sum_or_cascade(s: Vec<[f64; 5]>, parallel: bool, f: f64) -> f64 {
    if !parallel {
        return response64(FS, &s, f).norm();
    }
    let mut h = wing_dsp::Complex::new(0.0, 0.0);
    for c in &s {
        h = h + response64(FS, &[*c], f);
    }
    h.norm()
}

// ---- reference fidelity (exact ports) ---------------------------------------------------------------------

#[test]
fn geq_matches_wing_edit_coefficients() {
    let o = oracle();
    let cases = o.get("geq").unwrap().as_arr().unwrap();
    assert!(cases.len() >= 20);
    let ctx = Ctx::new(FS);
    let mut worst = (0.0f64, 0.0f64);
    for c in cases {
        let v = nums(c.get("values").unwrap());
        let r = bqs(c.get("biquads").unwrap());
        assert_eq!(r.len(), geq::BANDS);
        let mut g = [0.0f32; geq::BANDS];
        for (i, x) in g.iter_mut().enumerate() {
            *x = v[1 + i] as f32;
        }
        let mut out = [wingcoef::IDENTITY; geq::BANDS];
        geq::design(&ctx, &g, v[0] != 0.0, v[32] as f32, &mut out);
        for i in 0..geq::BANDS {
            worst.0 = worst.0.max(coef_err(&out[i], &r[i]));
        }
        worst.1 = worst.1.max(db_err(&out, &r, false));
    }
    assert!(worst.0 < 1e-6, "GEQ coefficient error {:e}", worst.0);
    assert!(worst.1 < 1e-3, "GEQ response error {} dB", worst.1);
}

#[test]
fn fx_eqs_match_wing_edit_coefficients() {
    let o = oracle();
    let ctx = prims::Ctx::new(FS);
    let mut seen = std::collections::BTreeSet::new();
    for c in o.get("eq").unwrap().as_arr().unwrap() {
        let id = c.get("model").unwrap().as_str().unwrap();
        let kind = AltKind::from_id(id).unwrap();
        seen.insert(id.to_string());
        let v = nums(c.get("values").unwrap());
        let r = bqs(c.get("biquads").unwrap());
        // slot 1 = mix (gain scale), the rest = channel slots
        let p: Vec<f32> = v[1..].iter().map(|&x| x as f32).collect();
        let m = (v[0] as f32) / 100.0;
        let mut out = [prims::UNITY; MAX_SECS];
        let layout = kind.design(&ctx, &p, m, &mut out);
        let a = &out[..layout.n];
        let r = &r[..layout.n];
        let e = db_err(a, r, layout.parallel);
        assert!(e < 1e-3, "{id} {v:?}: {e} dB");
        for (x, y) in a.iter().zip(r) {
            assert!(coef_err(x, y) < 1e-5, "{id} {v:?}: {x:?} vs {y:?}");
        }
    }
    assert_eq!(seen.len(), 7, "{seen:?}");
}

#[test]
fn spkman_linear_part_matches_wing_edit() {
    let o = oracle();
    let ctx = Ctx::new(FS);
    let cases = o.get("spkman").unwrap().as_arr().unwrap();
    assert!(cases.len() >= 20);
    for c in cases {
        let v: Vec<f32> = nums(c.get("values").unwrap())
            .iter()
            .map(|&x| x as f32)
            .collect();
        let r = bqs(c.get("biquads").unwrap());
        let d = nums(c.get("delay").unwrap());
        let a = spkman::design_linear(&ctx, &v);
        assert_eq!(r.len(), spkman::SECS);
        for (i, (x, y)) in a.iter().zip(&r).enumerate() {
            assert!(coef_err(x, y) < 1e-5, "{v:?} section {i}: {x:?} vs {y:?}");
        }
        let (l, rr) = spkman::delays(FS, v[8], v[9]);
        assert_eq!((l as f64, rr as f64), (d[0], d[1]), "{v:?}");
    }
}

// ---- model behaviour ---------------------------------------------------------------------------------------

fn sine(f: f64, amp: f64, n: usize) -> Vec<f32> {
    (0..n)
        .map(|i| (amp * (2.0 * core::f64::consts::PI * f * i as f64 / FS).sin()) as f32)
        .collect()
}

fn rms(x: &[f32]) -> f64 {
    (x.iter().map(|&v| (v as f64) * (v as f64)).sum::<f64>() / x.len() as f64).sqrt()
}

fn run(m: &mut dyn Model, l: &mut [f32], r: &mut [f32]) {
    for (cl, cr) in l.chunks_mut(64).zip(r.chunks_mut(64)) {
        m.process(cl, cr, None);
    }
}

/// Gain (dB) of a model for a steady sine (after `settle` seconds).
fn sine_gain_db(m: &mut dyn Model, f: f64, amp: f64) -> f64 {
    let n = (0.5 * FS) as usize;
    let mut l = sine(f, amp, n);
    let mut r = l.clone();
    let x = l.clone();
    run(m, &mut l, &mut r);
    let tail = n / 2;
    20.0 * (rms(&l[tail..]) / rms(&x[tail..])).log10()
}

#[test]
fn every_model_is_registered_and_neutral_at_defaults() {
    for id in [
        "GEQ", "PIA", "SOUL", "E88", "E84", "F110", "PULSAR", "MACH4", "DE-S2", "DEQ3", "C5-CMB",
        "SPKMAN", "LIMITER",
    ] {
        let mut m = create_model(BlockKind::Fx, id);
        assert!(!m.is_stub(), "{id}");
        assert_eq!(m.id(), id);
        m.prepare(FS, 64);
        m.load_defaults();
        m.commit();
        m.reset();
        let g = sine_gain_db(&mut *m, 1000.0, 0.1);
        let lim = match id {
            // WING's MACH4 parallel bank is not exactly flat at 0 (-0.06 dB at 1 kHz, channel model too)
            "MACH4" => 0.1,
            _ => 0.05,
        };
        assert!(g.abs() < lim, "{id}: {g} dB at defaults");
    }
}

#[test]
fn geq_band_gain_and_trim() {
    let mut m = GraphicEq::new();
    m.prepare(FS, 64);
    m.set_param(geq::slot::BAND0 + 17, ParamValue::Float(9.0)); // 1 kHz
    m.reset();
    let h = lin2db(m.response(1000.0).norm());
    assert!((h - 9.0).abs() < 0.05, "{h}");
    let g = sine_gain_db(&mut m, 1000.0, 0.1);
    assert!((g - 9.0).abs() < 0.1, "{g}");
    // TRU: a single fader's summed response is close to the fader (neighbour compensation)
    m.set_param(geq::slot::TYPE, ParamValue::Enum(1));
    for i in 0..geq::BANDS as u16 {
        m.set_param(geq::slot::BAND0 + i, ParamValue::Float(6.0));
    }
    m.reset();
    let flat = lin2db(m.response(1000.0).norm());
    assert!((flat - 6.0).abs() < 1.0, "TRU all +6: {flat}");
    m.set_param(geq::slot::TRIM, ParamValue::Float(-6.0));
    m.reset();
    let t = lin2db(m.response(1000.0).norm());
    assert!((t - (flat - 6.0)).abs() < 0.01, "{t}");
}

#[test]
fn fx_eq_mix_is_gain_scale() {
    let mut m = FxEq::new(AltKind::Pia);
    m.prepare(FS, 64);
    m.set_param(3, ParamValue::Float(12.0)); // 31 Hz? slot 3 = "31"
    m.set_param(8, ParamValue::Float(12.0)); // 1k
    m.reset();
    let full = sine_gain_db(&mut m, 1000.0, 0.1);
    assert!(full > 8.0, "{full}");
    m.set_param(1, ParamValue::Float(50.0));
    m.reset();
    let half = sine_gain_db(&mut m, 1000.0, 0.1);
    assert!(half > 2.0 && half < full - 3.0, "{half} vs {full}");
    m.set_param(1, ParamValue::Float(0.0));
    m.reset();
    assert!(sine_gain_db(&mut m, 1000.0, 0.1).abs() < 0.05);
    assert_eq!(m.param(1), Some(0.0));
    assert_eq!(m.param(8), Some(12.0));
}

#[test]
fn limiter_never_exceeds_ceiling() {
    let mut m = PrecisionLimiter::new();
    m.prepare(FS, 64);
    for (gin, gout, att, knee, again, sqz) in [
        (18.0, -0.5, 0.05, 3, 1, 0),
        (12.0, -6.0, 1.0, 10, 1, 100),
        (6.0, -1.0, 0.5, 0, 0, 50),
        (18.0, -18.0, 0.05, 0, 1, 0),
    ] {
        m.set_param(limiter::slot::GIN, ParamValue::Float(gin));
        m.set_param(limiter::slot::GOUT, ParamValue::Float(gout));
        m.set_param(limiter::slot::ATT, ParamValue::Float(att));
        m.set_param(limiter::slot::KNEE, ParamValue::Int(knee));
        m.set_param(limiter::slot::AGAIN, ParamValue::Int(again));
        m.set_param(limiter::slot::SQZ, ParamValue::Int(sqz));
        m.set_param(limiter::slot::REL, ParamValue::Float(50.0));
        m.commit();
        m.reset();
        let ceil = 10f64.powf(gout as f64 / 20.0);
        // noise bursts, full-scale square, clicks, sines across the band
        let n = 48_000;
        let mut seed = 12345u32;
        let mut l = Vec::with_capacity(n);
        let mut r = Vec::with_capacity(n);
        for i in 0..n {
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            let noise = (seed >> 8) as f32 / (1u32 << 24) as f32 * 2.0 - 1.0;
            let t = i as f32 / FS as f32;
            let burst = if (i / 4000) % 2 == 0 { 1.0 } else { 0.05 };
            let x = match (i / 12_000) % 4 {
                0 => noise * burst,
                1 => (if (i / 37) % 2 == 0 { 1.0 } else { -1.0 }) * burst,
                2 => (2.0 * core::f32::consts::PI * 9000.0 * t).sin(),
                _ => {
                    if i % 1000 == 0 {
                        1.0
                    } else {
                        0.01 * noise
                    }
                }
            };
            l.push(x);
            r.push(-x * 0.7);
        }
        run(&mut m, &mut l, &mut r);
        let peak = l
            .iter()
            .chain(&r)
            .fold(0.0f64, |p, &x| p.max((x as f64).abs()));
        assert!(
            peak <= ceil * (1.0 + 1e-6),
            "gin {gin} gout {gout}: peak {peak} > ceiling {ceil}"
        );
        assert!(
            peak > ceil * 0.5,
            "limiter too aggressive: {peak} vs {ceil}"
        );
    }
}

#[test]
fn limiter_ceiling_holds_across_attack_changes() {
    let mut m = PrecisionLimiter::new();
    m.prepare(FS, 64);
    m.set_param(limiter::slot::GIN, ParamValue::Float(18.0));
    m.set_param(limiter::slot::KNEE, ParamValue::Int(0));
    m.commit();
    m.reset();
    let ceil = 10f64.powf(-0.5 / 20.0);
    let mut seed = 99u32;
    let mut peak = 0.0f64;
    for b in 0..600 {
        if b % 7 == 0 {
            let att = 0.05 + (b % 20) as f32 * 0.05;
            m.set_param(limiter::slot::ATT, ParamValue::Float(att));
        }
        let mut l = [0.0f32; 64];
        let mut r = [0.0f32; 64];
        for i in 0..64 {
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            let x = (seed >> 8) as f32 / (1u32 << 24) as f32 * 2.0 - 1.0;
            l[i] = x;
            r[i] = if i % 9 == 0 { 1.0 } else { x * 0.3 };
        }
        m.process(&mut l, &mut r, None);
        peak = l.iter().chain(&r).fold(peak, |p, &x| p.max((x as f64).abs()));
    }
    assert!(peak <= ceil * (1.0 + 1e-6), "{peak} > {ceil}");
}

#[test]
fn limiter_latency_and_transparency() {
    let mut m = PrecisionLimiter::new();
    m.prepare(FS, 64);
    let la = m.latency();
    assert_eq!(la, 50);
    // impulse below the ceiling comes out delayed and unchanged (auto gain on, gin 0)
    let mut l = vec![0.0f32; 128];
    let mut r = vec![0.0f32; 128];
    l[0] = 0.5;
    r[0] = -0.25;
    m.process(&mut l, &mut r, None);
    assert!((l[la] - 0.5).abs() < 1e-6 && (r[la] + 0.25).abs() < 1e-6);
    // attack does not change the latency
    m.set_param(limiter::slot::ATT, ParamValue::Float(1.0));
    m.commit();
    assert_eq!(m.latency(), la);
    // gain reduction telemetry
    m.set_param(limiter::slot::GIN, ParamValue::Float(12.0));
    m.reset();
    let n = 24_000;
    let mut l = sine(100.0, 0.9, n);
    let mut r = l.clone();
    run(&mut m, &mut l, &mut r);
    let peak = l[n / 2..].iter().fold(0.0f32, |p, x| p.max(x.abs()));
    let ceil = 10f32.powf(-0.5 / 20.0);
    assert!(peak <= ceil * 1.000_001 && peak > ceil * 0.97, "{peak}");
    assert!(m.telemetry().gain_reduction_db > 5.0);
}

#[test]
fn limiter_soft_knee_curve_stays_below_threshold() {
    for knee in [0.0, 3.0, 10.0] {
        let mut prev = f64::NEG_INFINITY;
        for i in 0..400 {
            let l_db = -30.0 + i as f64 * 0.1;
            let x = 10f64.powf(l_db / 20.0);
            let g = limiter::static_gain(x, 1.0, knee);
            let out = 20.0 * (x * g).log10();
            assert!(out <= 1e-9, "knee {knee}: {l_db} -> {out}");
            assert!(out >= prev - 1e-9, "monotonic");
            prev = out;
        }
    }
}

#[test]
fn combinator_bands_sum_flat() {
    for lr48 in [false, true] {
        let mut f = 10.0;
        while f < 23_000.0 {
            let db = combinator::sum_response_db(FS, lr48, f);
            assert!(db.abs() < 1e-3, "lr48 {lr48}: {db} dB at {f} Hz");
            f *= 1.1;
        }
    }
    // the runtime model (no compression, mix 100) is flat too
    for slope in [0u32, 1] {
        let mut m = Combinator::new();
        m.prepare(FS, 64);
        m.set_param(4, ParamValue::Enum(slope));
        m.reset();
        for f in [40.0, 80.0, 317.0, 1000.0, 1262.0, 5024.0, 12_000.0] {
            let g = sine_gain_db(&mut m, f, 0.01);
            assert!(g.abs() < 0.01, "slope {slope}: {g} dB at {f}");
        }
    }
}

#[test]
fn combinator_crossovers_match_wing() {
    let x = combinator::crossovers();
    for (a, b) in x.iter().zip([79.62, 316.98, 1261.9, 5023.8]) {
        assert!((a - b).abs() < 0.1, "{a} {b}");
    }
}

#[test]
fn combinator_compresses_loud_band() {
    let mut m = Combinator::new();
    m.prepare(FS, 64);
    m.set_param(1, ParamValue::Float(-30.0)); // thr
    m.set_param(3, ParamValue::Enum(13)); // ratio 100
    m.set_param(10, ParamValue::Int(0)); // SBC off
    m.reset();
    let g = sine_gain_db(&mut m, 1000.0, 0.5);
    // -9 dBFS sine (RMS reads its peak) -> about 21 dB of reduction
    assert!(g < -15.0, "{g}");
    assert!(m.telemetry().gain_reduction_db > 15.0);
    // band bypass (all bands)
    for s in 21..=25 {
        m.set_param(s, ParamValue::Int(1));
    }
    m.reset();
    let g = sine_gain_db(&mut m, 1000.0, 0.5);
    assert!(g.abs() < 0.01, "bypassed bands: {g}");
}

#[test]
fn combinator_width_and_solo() {
    let mut m = Combinator::new();
    m.prepare(FS, 64);
    // band 3 solo, 100 Hz is in band 2 -> silent
    m.set_param(5, ParamValue::Int(3));
    m.set_param(32, ParamValue::Int(1));
    m.reset();
    assert!(sine_gain_db(&mut m, 100.0, 0.1) < -30.0);
    assert!(sine_gain_db(&mut m, 700.0, 0.1).abs() < 1.0);
    // width -50 = mono: an L-only signal at 2 kHz comes out equal on both sides
    m.set_param(32, ParamValue::Int(0));
    for s in 26..=30 {
        m.set_param(s, ParamValue::Float(-50.0));
    }
    m.reset();
    let mut l = sine(2000.0, 0.1, 9600);
    let mut r = vec![0.0f32; 9600];
    run(&mut m, &mut l, &mut r);
    let (a, b) = (rms(&l[4800..]), rms(&r[4800..]));
    assert!((a - b).abs() / a < 0.01, "{a} {b}");
}

#[test]
fn deesser_reduces_sibilance_only() {
    let mut m = DeEsser2::new();
    m.prepare(FS, 64);
    m.set_param(1, ParamValue::Float(50.0));
    m.set_param(2, ParamValue::Float(50.0));
    m.reset();
    // low vowel-like tone: untouched
    let g = sine_gain_db(&mut m, 300.0, 0.3);
    assert!(g.abs() < 0.1, "{g}");
    // sibilant (female LOW band 6.5 kHz): reduced, level-independent
    m.reset();
    let g1 = sine_gain_db(&mut m, 6500.0, 0.3);
    m.reset();
    let g2 = sine_gain_db(&mut m, 6500.0, 0.03);
    assert!(g1 < -6.0, "{g1}");
    assert!((g1 - g2).abs() < 1.0, "level dependence {g1} {g2}");
    // amount 0 = bypass
    m.set_param(1, ParamValue::Float(0.0));
    m.set_param(2, ParamValue::Float(0.0));
    m.reset();
    assert!(sine_gain_db(&mut m, 6500.0, 0.3).abs() < 1e-3);
    // M/S: side-only sibilance is handled by the side amounts
    m.set_param(6, ParamValue::Enum(1));
    m.set_param(3, ParamValue::Float(50.0));
    m.reset();
    let n = 24_000;
    let mut l = sine(6500.0, 0.3, n);
    let mut r: Vec<f32> = l.iter().map(|x| -x).collect();
    let x = l.clone();
    run(&mut m, &mut l, &mut r);
    let g = 20.0 * (rms(&l[n / 2..]) / rms(&x[n / 2..])).log10();
    assert!(g < -6.0, "side de-essing {g}");
}

#[test]
fn deq3_bands_act_like_channel_deq() {
    let mut m = TripleDeq::new();
    m.prepare(FS, 64);
    // band 2: -10 dB max cut at 1 kHz above -40 dB
    m.set_param(10, ParamValue::Float(-40.0));
    m.set_param(11, ParamValue::Enum(6)); // ratio 10
    m.set_param(15, ParamValue::Float(-10.0));
    m.set_param(16, ParamValue::Float(1000.0));
    m.commit();
    m.reset();
    let g = sine_gain_db(&mut m, 1000.0, 0.5);
    assert!(g < -8.0 && g > -10.5, "{g}");
    let g = sine_gain_db(&mut m, 100.0, 0.5);
    assert!(g.abs() < 0.5, "{g}");
    assert_eq!(m.param(16), Some(1000.0));
    assert_eq!(m.param(7), Some(150.0));
}

#[test]
fn spkman_crossover_delay_polarity_and_limiter() {
    let mut m = SpeakerManager::new();
    m.prepare(FS, 64);
    // LR24 high-pass at 1 kHz: -6 dB at fc, strong cut an octave and a half below
    m.set_param(1, ParamValue::Float(1000.0));
    m.set_param(2, ParamValue::Enum(8));
    m.reset();
    let g = sine_gain_db(&mut m, 1000.0, 0.1);
    assert!((g + 6.0).abs() < 0.3, "{g}");
    assert!(sine_gain_db(&mut m, 300.0, 0.1) < -35.0);
    m.set_param(2, ParamValue::Enum(0));
    // delay + invert: impulse
    m.set_param(8, ParamValue::Int(1));
    m.set_param(9, ParamValue::Float(1.0));
    m.set_param(10, ParamValue::Float(0.5));
    m.reset();
    let (dl, dr) = m.delay_samples();
    assert_eq!((dl, dr), (209, 139));
    let mut l = vec![0.0f32; 512];
    let mut r = vec![0.0f32; 512];
    l[0] = 1.0;
    r[0] = 1.0;
    m.process(&mut l, &mut r, None);
    assert!((l[dl] + 1.0).abs() < 1e-6, "{}", l[dl]);
    assert!((r[dr] + 1.0).abs() < 1e-6);
    // limiter (peak): a 0 dBFS sine is held near -12 dBFS
    m.set_param(8, ParamValue::Int(0));
    m.set_param(21, ParamValue::Int(1));
    m.set_param(22, ParamValue::Float(-12.0));
    m.set_param(23, ParamValue::Int(0));
    m.reset();
    let g = sine_gain_db(&mut m, 200.0, 1.0);
    assert!((g + 12.0).abs() < 1.0, "{g}");
}

#[test]
fn spkman_phase_allpass_is_flat_with_phase_at_lpf() {
    let c = Ctx::new(FS);
    for p in [0.0f32, 45.0, 90.0, 180.0] {
        let s = spkman::phase_section(&c, 2000.0, p, false);
        let h = wingcoef::response(FS, &[s], 2000.0);
        assert!((h.norm() - 1.0).abs() < 1e-5);
        let ph = h.im.atan2(h.re).to_degrees();
        let want = -(p as f64);
        let d = (ph - want + 540.0).rem_euclid(360.0) - 180.0;
        // the corner is placed with the analog formula, so bilinear warping leaves ~1 degree at 2 kHz
        assert!(d.abs() < 2.0 || p == 0.0 && ph.abs() < 1e-6, "{p}: {ph}");
    }
}

#[test]
fn tilt_section_shelves() {
    let c = Ctx::new(FS);
    let s = spkman::tilt_section(&c, 1000.0, 6.0);
    let lo = lin2db(wingcoef::response(FS, &[s], 20.0).norm());
    let hi = lin2db(wingcoef::response(FS, &[s], 20_000.0).norm());
    assert!(
        (lo - 6.0).abs() < 0.5 && (hi + 6.0).abs() < 1.5,
        "{lo} {hi}"
    );
}
