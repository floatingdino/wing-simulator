//! T05P tests (PLATE, V-PLATE, BPLATE, SPRING): T60 vs `dcy`, bass multiplier / crossover / damping / colour,
//! fast plate build-up, spring dispersion (group delay rising with frequency) and repeating drips, silence /
//! denormals, stability at extremes, click-free parameter sweeps, wet level, parameter round trip.
//! (Helpers are local copies so this file merges independently of the T05R tests.)

use super::*;
use crate::model::{BlockKind, Model};
use wing_dsp::biquad::{Biquad, Coeffs};

const FS: f64 = 48_000.0;
const IDS: [&str; 4] = ["PLATE", "V-PLATE", "BPLATE", "SPRING"];

fn model(id: &str) -> Box<dyn Model> {
    let mut m = crate::model::create_model(BlockKind::Fx, IDS.iter().find(|&&x| x == id).unwrap());
    assert!(!m.is_stub(), "{id}");
    m.prepare(FS, 64);
    m
}

fn params(id: &str) -> &'static [ParamSpec] {
    match id {
        "PLATE" => &plate::PLATE_PARAMS,
        "V-PLATE" => &plate::VPLATE_PARAMS,
        "BPLATE" => &plate::BPLATE_PARAMS,
        _ => &spring::PARAMS,
    }
}

fn set(m: &mut dyn Model, id: &str, key: &str, v: f32) {
    let slot = params(id).iter().position(|p| p.key == key).unwrap() as u16 + 1;
    m.set_param(slot, ParamValue::Float(v));
}

/// Wide-open output filters, no pre-echoes, flat bass decay.
fn neutral(m: &mut dyn Model, id: &str) {
    match id {
        "PLATE" | "BPLATE" => {
            set(m, id, "lc", 20.0);
            set(m, id, "hc", 20_000.0);
            set(m, id, "damp", 20_000.0);
            set(m, id, "mult", 1.0);
            if id == "PLATE" {
                for k in ["ecl", "ecr", "efl", "efr"] {
                    set(m, id, k, 0.0);
                }
            }
        }
        "V-PLATE" => {
            set(m, id, "lc", 20.0);
            set(m, id, "col", 20.0);
        }
        _ => {
            set(m, id, "low", 25.5);
            set(m, id, "high", 50.0); // least in-loop HF damping
        }
    }
}

fn run(m: &mut dyn Model, l: &mut [f32], r: &mut [f32]) {
    for (a, b) in l.chunks_mut(64).zip(r.chunks_mut(64)) {
        m.process(a, b, None);
    }
}

fn ir(m: &mut dyn Model, secs: f64) -> (Vec<f32>, Vec<f32>) {
    m.reset();
    let n = (secs * FS) as usize;
    let (mut l, mut r) = (vec![0.0f32; n], vec![0.0f32; n]);
    l[0] = 1.0;
    r[0] = 1.0;
    run(m, &mut l, &mut r);
    (l, r)
}

/// T60 in the octave band around `f0` (Schroeder EDC, linear fit -5..-35 dB).
fn t60(x: &[f32], f0: f64) -> f64 {
    let c = Coeffs::bandpass(FS, f0, std::f64::consts::SQRT_2);
    let (mut b1, mut b2) = (Biquad::new(c), Biquad::new(c));
    let y: Vec<f64> = x.iter().map(|&v| b2.tick(b1.tick(v)) as f64).collect();
    let mut edc = vec![0.0f64; y.len()];
    let mut acc = 0.0;
    for i in (0..y.len()).rev() {
        acc += y[i] * y[i];
        edc[i] = acc;
    }
    let e0 = edc[0];
    let (mut sx, mut sy, mut sxx, mut sxy, mut n) = (0.0, 0.0, 0.0, 0.0, 0.0);
    for (i, &e) in edc.iter().enumerate() {
        let d = 10.0 * (e / e0).log10();
        if (-35.0..=-5.0).contains(&d) {
            let t = i as f64 / FS;
            sx += t;
            sy += d;
            sxx += t * t;
            sxy += t * d;
            n += 1.0;
        }
    }
    -60.0 / ((n * sxy - sx * sy) / (n * sxx - sx * sx))
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
    for (id, d, bands) in [
        ("PLATE", 0.5, &[250.0, 1000.0, 2000.0][..]),
        ("PLATE", 2.0, &[250.0, 1000.0, 2000.0]),
        ("PLATE", 6.0, &[1000.0]),
        ("V-PLATE", 1.0, &[250.0, 1000.0]),
        ("V-PLATE", 3.0, &[250.0, 1000.0, 2000.0]),
        ("V-PLATE", 6.0, &[1000.0]),
        ("BPLATE", 0.4, &[1000.0]),
        ("BPLATE", 1.6, &[250.0, 1000.0, 2000.0]),
        ("BPLATE", 5.0, &[1000.0]),
        ("SPRING", 1.5, &[500.0, 1000.0, 2000.0]),
        ("SPRING", 3.0, &[250.0, 1000.0]),
        ("SPRING", 6.0, &[1000.0]),
    ] {
        let mut m = model(id);
        neutral(&mut *m, id);
        set(&mut *m, id, "dcy", d);
        let (l, r) = ir(&mut *m, 1.6 * d as f64 + 0.3);
        for &f in bands {
            check(id, &format!("dcy {d} {f} Hz L"), t60(&l, f), d as f64);
        }
        check(id, &format!("dcy {d} 1 kHz R"), t60(&r, 1000.0), d as f64);
    }
}

#[test]
fn plate_size_and_spring_density_keep_t60() {
    for (id, key, v) in [
        ("PLATE", "size", 4.0),
        ("PLATE", "size", 76.0),
        ("BPLATE", "size", 2.0),
        ("BPLATE", "size", 100.0),
        ("SPRING", "dens", 1.0),
        ("SPRING", "dens", 30.0),
    ] {
        let mut m = model(id);
        neutral(&mut *m, id);
        set(&mut *m, id, "dcy", 2.0);
        set(&mut *m, id, key, v);
        let (l, _) = ir(&mut *m, 3.5);
        check(id, &format!("{key} {v} 1 kHz"), t60(&l, 1000.0), 2.0);
    }
}

#[test]
fn bass_mult_crossover_and_damping() {
    // PLATE: mult 2 below 600 Hz; damping 1.5 kHz -> HF T60 = 0.5 x dcy
    let mut m = model("PLATE");
    neutral(&mut *m, "PLATE");
    set(&mut *m, "PLATE", "dcy", 1.5);
    set(&mut *m, "PLATE", "mult", 2.0);
    let (l, _) = ir(&mut *m, 5.0);
    check("PLATE", "mult 2, 100 Hz", t60(&l, 100.0), 3.0);
    check("PLATE", "mult 2, 2 kHz", t60(&l, 2000.0), 1.5);
    set(&mut *m, "PLATE", "mult", 1.0);
    set(&mut *m, "PLATE", "damp", 1500.0);
    let (l, _) = ir(&mut *m, 2.8);
    check("PLATE", "damp 1.5k, 8 kHz", t60(&l, 8000.0), 0.75);
    check("PLATE", "damp 1.5k, 250 Hz", t60(&l, 250.0), 1.5);

    // BPLATE: the crossover is a parameter. mult 2 with xover 500 lengthens 125 Hz; with xover 20 it does not.
    for (xo, want) in [(500.0, 3.0), (20.0, 1.5)] {
        let mut m = model("BPLATE");
        neutral(&mut *m, "BPLATE");
        set(&mut *m, "BPLATE", "dcy", 1.5);
        set(&mut *m, "BPLATE", "mult", 2.0);
        set(&mut *m, "BPLATE", "xover", xo);
        let (l, _) = ir(&mut *m, 5.0);
        let tol_band = if xo > 100.0 { 100.0 } else { 500.0 };
        check(
            "BPLATE",
            &format!("xover {xo}, {tol_band} Hz"),
            t60(&l, tol_band),
            want,
        );
    }
    let mut m = model("BPLATE");
    neutral(&mut *m, "BPLATE");
    set(&mut *m, "BPLATE", "dcy", 2.0);
    set(&mut *m, "BPLATE", "damp", 1500.0);
    let (l, _) = ir(&mut *m, 3.5);
    check("BPLATE", "damp 1.5k, 8 kHz", t60(&l, 8000.0), 0.9);
}

/// Energy above 5 kHz relative to 300 Hz..2 kHz (brightness), 50..500 ms of the IR.
fn brightness(x: &[f32]) -> f64 {
    let s = &x[(0.05 * FS) as usize..(0.5 * FS) as usize];
    let band = |c: Coeffs| {
        let (mut a, mut b) = (Biquad::new(c), Biquad::new(c));
        s.iter()
            .map(|&v| (b.tick(a.tick(v)) as f64).powi(2))
            .sum::<f64>()
    };
    band(Coeffs::highpass(FS, 5000.0, 0.707)) / band(Coeffs::bandpass(FS, 800.0, 0.5))
}

#[test]
fn vplate_colour_and_spring_treble_set_brightness() {
    let mut last = 0.0;
    for col in [-20.0, 0.0, 20.0] {
        let mut m = model("V-PLATE");
        set(&mut *m, "V-PLATE", "col", col);
        let (l, _) = ir(&mut *m, 0.6);
        let b = brightness(&l);
        eprintln!("V-PLATE col {col}: brightness {b:.4}");
        assert!(b > 3.0 * last, "col {col}: {b} vs {last}");
        last = b;
    }
    assert_eq!(plate::vplate_colour(20.0), (20_000.0, 20_000.0));
    let mut last = 0.0;
    for high in [1.0, 25.0, 50.0] {
        let mut m = model("SPRING");
        set(&mut *m, "SPRING", "high", high);
        let (l, _) = ir(&mut *m, 0.6);
        let c = Coeffs::highpass(FS, 2500.0, 0.707);
        let (mut a, mut b) = (Biquad::new(c), Biquad::new(c));
        let e: f64 = l.iter().map(|&v| (b.tick(a.tick(v)) as f64).powi(2)).sum();
        assert!(e > 2.0 * last, "treble {high}: {e} vs {last}");
        last = e;
    }
}

/// Normalised echo density (Abel & Huang 2006) in a window starting at `t0` seconds.
fn echo_density(x: &[f32], t0: f64, win: usize) -> f64 {
    let s = (t0 * FS) as usize;
    let w = &x[s..s + win];
    let sd = (w.iter().map(|&v| (v as f64).powi(2)).sum::<f64>() / win as f64).sqrt();
    let above = w.iter().filter(|&&v| (v as f64).abs() > sd).count() as f64 / win as f64;
    above / 0.317_310_5
}

#[test]
fn plates_build_up_dense_fast() {
    for id in ["PLATE", "V-PLATE", "BPLATE"] {
        let mut m = model(id);
        neutral(&mut *m, id);
        set(&mut *m, id, "pdel", 0.0);
        let (l, r) = ir(&mut *m, 0.3);
        let ned = 0.5 * (echo_density(&l, 0.06, 1024) + echo_density(&r, 0.06, 1024));
        eprintln!("{id}: echo density at 60 ms = {ned:.2}");
        assert!(ned > 0.75, "{id}: {ned}");
    }
}

/// Group delay (samples) of `h` at `f` Hz: Re{ DFT(n h) / DFT(h) }.
fn group_delay(h: &[f32], f: f64) -> f64 {
    let w = std::f64::consts::TAU * f / FS;
    let (mut hr, mut hi, mut nr, mut ni) = (0.0, 0.0, 0.0, 0.0);
    for (n, &x) in h.iter().enumerate() {
        let (s, c) = (w * n as f64).sin_cos();
        let x = x as f64;
        hr += x * c;
        hi -= x * s;
        nr += n as f64 * x * c;
        ni -= n as f64 * x * s;
    }
    (nr * hr + ni * hi) / (hr * hr + hi * hi)
}

#[test]
fn spring_chirp_group_delay_rises_with_frequency() {
    let mut m = model("SPRING");
    neutral(&mut *m, "SPRING");
    set(&mut *m, "SPRING", "dens", 1.0); // one spring per side (lane 0: 33.1 ms round trip)
    let (l, _) = ir(&mut *m, 0.2);
    // first transit only: before the first round trip returns
    let first = &l[..1500];
    let mut last = -1.0;
    let mut gd = Vec::new();
    for f in [600.0, 1500.0, 2500.0, 3500.0] {
        let g = group_delay(first, f);
        eprintln!("SPRING group delay at {f} Hz: {:.2} ms", g / 48.0);
        assert!(g > last, "{f} Hz: {g} vs {last}");
        last = g;
        gd.push(g);
    }
    // a clear chirp: the top arrives > 3 ms after the bass
    assert!(gd[3] - gd[0] > 3.0 * 48.0, "{gd:?}");
    // model check: matches the analytic chain group delay within the output filters' contribution
    let w = std::f32::consts::TAU * 3500.0 / FS as f32;
    let want = spring::chain_group_delay(spring::TENSION[0], 5, w) as f64;
    assert!((gd[3] - want).abs() < 0.25 * want, "{} vs {want}", gd[3]);
}

#[test]
fn spring_drips_repeat_every_round_trip() {
    let mut m = model("SPRING");
    neutral(&mut *m, "SPRING");
    set(&mut *m, "SPRING", "dens", 1.0);
    let (l, _) = ir(&mut *m, 0.3);
    // envelope peaks: one chirp per round trip (33.1 ms on the first left spring)
    let env = |a: usize, b: usize| l[a..b].iter().map(|v| (*v as f64).powi(2)).sum::<f64>();
    let rt = (33.1e-3 * FS) as usize;
    let gap = env(rt - 400, rt - 50); // between drips
    let drip = env(rt, rt + 600); // second transit arrives after one round trip
    eprintln!("SPRING: drip/gap energy {:.1}", drip / gap.max(1e-30));
    assert!(drip > 10.0 * gap, "{drip} {gap}");
}

#[test]
fn spring_decay_factor_is_exact_without_damping_loss() {
    let mut s = spring();
    s.prepare(FS, 64);
    s.set_param(1, ParamValue::Float(3.0));
    s.reset();
    assert_eq!(s.stretch(), 5);
    let rho = 10f32.powf(-3.0 / (FS as f32 * 3.0));
    for i in 0..spring::LANES {
        let r = s.decay_factor(i);
        // r is >= rho (compensates the in-loop shelf) and within 1e-5 of it
        assert!(r >= rho - 1e-7 && r - rho < 1e-5, "{i}: {r} {rho}");
    }
}

#[test]
fn vplate_pre_delay_reaches_250_ms() {
    let onset = |pdel: f32| {
        let mut m = model("V-PLATE");
        set(&mut *m, "V-PLATE", "pdel", pdel);
        let (l, _) = ir(&mut *m, 0.5);
        l.iter().position(|v| v.abs() > 1e-3).unwrap()
    };
    let (a, b) = (onset(0.0), onset(250.0));
    assert!((b as i64 - a as i64 - 12_000).abs() <= 2, "{a} {b}");
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
    let mut s = spring();
    s.prepare(FS, 64);
    let (mut l, mut r) = ([0.0f32; 64], [0.0f32; 64]);
    l[0] = 1.0;
    s.process(&mut l, &mut r, None);
    for _ in 0..(25.0 * FS) as usize / 64 {
        let (mut l, mut r) = ([0.0f32; 64], [0.0f32; 64]);
        s.process(&mut l, &mut r, None);
    }
    let (sum, sub) = s.state_report();
    assert!(!sub);
    assert_eq!(sum, 0.0, "SPRING state not flushed");
}

fn noise(n: usize, seed: u32) -> Vec<f32> {
    let mut s = seed;
    (0..n)
        .map(|_| {
            s = s.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            (s >> 8) as f32 / (1u32 << 24) as f32 * 2.0 - 1.0
        })
        .collect()
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
            m.reset();
            let n = (4.0 * FS) as usize;
            let mut l = noise(n, 1);
            let mut r = noise(n, 2);
            for v in l.iter_mut().chain(r.iter_mut()).skip(n / 2) {
                *v = 0.0;
            }
            run(&mut *m, &mut l, &mut r);
            let peak = l.iter().chain(r.iter()).fold(0f32, |a, v| a.max(v.abs()));
            eprintln!("{id} {}: peak {peak:.2}", if hi { "max" } else { "min" });
            assert!(peak.is_finite() && peak < 50.0, "{id}: {peak}");
        }
    }
}

#[test]
fn wet_level_is_sane_at_defaults() {
    for id in IDS {
        let mut m = model(id);
        let n = (3.0 * FS) as usize;
        let mut l = noise(n, 7);
        let mut r = noise(n, 9);
        let rin = (l.iter().map(|v| (*v as f64).powi(2)).sum::<f64>() / n as f64).sqrt();
        run(&mut *m, &mut l, &mut r);
        let s = n / 2;
        let rout =
            (l[s..].iter().map(|v| (*v as f64).powi(2)).sum::<f64>() / (n - s) as f64).sqrt();
        let db = 20.0 * (rout / rin).log10();
        eprintln!("{id}: wet/dry RMS at defaults {db:.1} dB");
        assert!((-20.0..=3.0).contains(&db), "{id}: {db} dB");
    }
}

fn hf_trace(x: &[f32]) -> Vec<f32> {
    let c = Coeffs::highpass(FS, 6000.0, std::f64::consts::FRAC_1_SQRT_2);
    let (mut a, mut b) = (Biquad::new(c), Biquad::new(c));
    x.iter().map(|&v| b.tick(a.tick(v)).abs()).collect()
}

#[test]
fn parameter_sweeps_do_not_click() {
    let blocks_per_sweep = (0.5 * FS) as usize / 64;
    for id in IDS {
        for (i, p) in params(id).iter().enumerate() {
            let slot = i as u16 + 1;
            let mut m = model(id);
            if id != "SPRING" && id != "V-PLATE" {
                set(&mut *m, id, "hc", 20_000.0);
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
            let level = out[..sweep_start]
                .iter()
                .chain(&out[steady2..jump])
                .fold(0f32, |a, v| a.max(v.abs()))
                .max(1e-3);
            let base = pk(4096..sweep_start).max(pk(steady2 + 4096..jump));
            let hf = pk(sweep_start..steady2).max(pk(jump..out.len()));
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
            assert_eq!(m.param(i as u16 + 1), Some(p.default), "{id} {}", p.key);
        }
        assert_eq!(m.param(0), None);
        assert_eq!(m.param(ps.len() as u16 + 1), None);
        m.set_param(1, ParamValue::Float(1e9));
        assert_eq!(m.param(1), Some(ps[0].max));
        m.set_param(1, ParamValue::Int(-5));
        assert_eq!(m.param(1), Some(ps[0].min));
        m.set_param(1, ParamValue::Float(f32::NAN));
        assert_eq!(m.param(1), Some(ps[0].min));
        m.load_defaults();
        assert_eq!(m.param(1), Some(ps[0].default));
        assert_eq!(m.latency(), 0);
    }
}
