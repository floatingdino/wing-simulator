//! Reverb family tests: T60 accuracy vs `dcy` per band, echo density, stereo decorrelation, silence /
//! denormals, stability at extreme settings, click-free parameter sweeps, echo/pre-delay mapping.

use super::*;
use crate::model::{BlockKind, Model};
use wing_dsp::biquad::{Biquad, Coeffs};

pub(super) const FS: f64 = 48_000.0;
const IDS: [&str; 5] = ["HALL", "ROOM", "CHAMBER", "CONCERT", "AMBI"];

fn model(id: &str) -> Box<dyn Model> {
    let mut m = crate::model::create_model(
        BlockKind::Fx,
        IDS.iter().find(|&&x| x == id).copied().unwrap(),
    );
    assert!(!m.is_stub(), "{id}");
    m.prepare(FS, 64);
    m
}

fn slot_of(id: &str, key: &str) -> u16 {
    spec(id).params.iter().position(|p| p.key == key).unwrap() as u16 + 1
}

fn spec(id: &str) -> &'static ReverbSpec {
    match id {
        "HALL" => &hall::SPEC,
        "ROOM" => &room::SPEC,
        "CHAMBER" => &chamber::SPEC,
        "CONCERT" => &concert::SPEC,
        _ => &ambi::SPEC,
    }
}

fn set(m: &mut dyn Model, id: &str, key: &str, v: f32) {
    m.set_param(slot_of(id, key), ParamValue::Float(v));
}

/// Wide-open output filters, all echo / reflection levels off.
fn neutral(m: &mut dyn Model, id: &str) {
    set(m, id, "lc", 20.0);
    set(m, id, "hc", 20_000.0);
    match id {
        "CHAMBER" => {
            set(m, id, "ell", -144.0);
            set(m, id, "elr", -144.0);
        }
        "CONCERT" => {
            set(m, id, "rfll", -144.0);
            set(m, id, "rflr", -144.0);
        }
        "ROOM" => {
            // no level key: zero time + no feedback folds the echo into the direct ER
            set(m, id, "efl", 0.0);
            set(m, id, "efr", 0.0);
            set(m, id, "ecl", 0.0);
            set(m, id, "ecr", 0.0);
        }
        _ => {}
    }
}

pub(super) fn run(m: &mut dyn Model, l: &mut [f32], r: &mut [f32]) {
    for (a, b) in l.chunks_mut(64).zip(r.chunks_mut(64)) {
        m.process(a, b, None);
    }
}

/// Stereo impulse response (unit impulse into L and R).
pub(super) fn ir(m: &mut dyn Model, secs: f64) -> (Vec<f32>, Vec<f32>) {
    m.reset();
    let n = (secs * FS) as usize;
    let (mut l, mut r) = (vec![0.0f32; n], vec![0.0f32; n]);
    l[0] = 1.0;
    r[0] = 1.0;
    run(m, &mut l, &mut r);
    (l, r)
}

/// T60 in the octave band around `f0` from the Schroeder energy decay curve (linear fit -5..-35 dB).
pub(super) fn t60(x: &[f32], f0: f64) -> f64 {
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
    let db: Vec<f64> = edc.iter().map(|&e| 10.0 * (e / e0).log10()).collect();
    let (mut sx, mut sy, mut sxx, mut sxy, mut n) = (0.0, 0.0, 0.0, 0.0, 0.0);
    for (i, &d) in db.iter().enumerate() {
        if (-35.0..=-5.0).contains(&d) {
            let t = i as f64 / FS;
            sx += t;
            sy += d;
            sxx += t * t;
            sxy += t * d;
            n += 1.0;
        }
    }
    let slope = (n * sxy - sx * sy) / (n * sxx - sx * sx);
    -60.0 / slope
}

/// (decay for the mid tests, hf ratio, bass mult available)
fn decay_for(id: &str) -> (f32, f32, bool) {
    match id {
        "HALL" => (2.0, 0.3, true),
        "ROOM" => (1.5, 0.35, true),
        "CHAMBER" => (1.2, 0.4, true),
        "CONCERT" => (2.5, 0.3, true),
        _ => (1.0, 0.35, false),
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
fn t60_tracks_decay_mid_band() {
    for id in IDS {
        let (d, _, _) = decay_for(id);
        let mut m = model(id);
        neutral(&mut *m, id);
        set(&mut *m, id, "dcy", d);
        set(&mut *m, id, "damp", 20_000.0);
        if decay_for(id).2 {
            set(&mut *m, id, "mult", 1.0);
        }
        let (l, r) = ir(&mut *m, 1.6 * d as f64 + 0.3);
        for f in [250.0, 1000.0, 2000.0] {
            check(id, &format!("{f} Hz L"), t60(&l, f), d as f64);
        }
        check(id, "1 kHz R", t60(&r, 1000.0), d as f64);
    }
}

#[test]
fn t60_tracks_decay_at_other_settings() {
    // short and long decays, size extremes
    for (id, d, size) in [
        ("HALL", 0.6, 10.0),
        ("HALL", 4.5, 100.0),
        ("ROOM", 6.0, 70.0),
        ("ROOM", 0.5, 8.0),
        ("CHAMBER", 3.0, 40.0),
        ("CONCERT", 6.0, 76.0),
        ("AMBI", 0.4, 20.0),
        ("AMBI", 3.0, 100.0),
    ] {
        let mut m = model(id);
        neutral(&mut *m, id);
        set(&mut *m, id, "dcy", d);
        set(&mut *m, id, "size", size);
        set(&mut *m, id, "damp", 20_000.0);
        if decay_for(id).2 {
            set(&mut *m, id, "mult", 1.0);
        }
        let (l, _) = ir(&mut *m, 1.6 * d as f64 + 0.3);
        check(
            id,
            &format!("dcy {d} size {size} 1 kHz"),
            t60(&l, 1000.0),
            d as f64,
        );
    }
}

#[test]
fn t60_bass_mult_and_damping() {
    for id in IDS {
        let (d, hf, has_mult) = decay_for(id);
        if has_mult {
            let mut m = model(id);
            neutral(&mut *m, id);
            set(&mut *m, id, "dcy", d);
            set(&mut *m, id, "damp", 20_000.0);
            set(&mut *m, id, "mult", 2.0);
            let (l, _) = ir(&mut *m, 1.6 * 2.0 * d as f64 + 0.3);
            check(id, "mult 2, 100 Hz", t60(&l, 100.0), 2.0 * d as f64);
            check(id, "mult 2, 2 kHz", t60(&l, 2000.0), d as f64);
        }
        let mut m = model(id);
        neutral(&mut *m, id);
        set(&mut *m, id, "dcy", d);
        if has_mult {
            set(&mut *m, id, "mult", 1.0);
        }
        set(&mut *m, id, "damp", 1500.0);
        let (l, _) = ir(&mut *m, 1.6 * d as f64 + 0.3);
        check(id, "damp 1.5k, 8 kHz", t60(&l, 8000.0), (d * hf) as f64);
        check(id, "damp 1.5k, 250 Hz", t60(&l, 250.0), d as f64);
    }
}

/// Normalised echo density (Abel & Huang 2006) of `x` in a window starting at `t0` seconds.
pub(super) fn echo_density(x: &[f32], t0: f64, win: usize) -> f64 {
    let s = (t0 * FS) as usize;
    let w = &x[s..s + win];
    let sd = (w.iter().map(|&v| (v as f64).powi(2)).sum::<f64>() / win as f64).sqrt();
    let above = w.iter().filter(|&&v| (v as f64).abs() > sd).count() as f64 / win as f64;
    above / 0.317_310_5 // erfc(1/sqrt 2): fraction above 1 sd for Gaussian noise
}

#[test]
fn echo_density_builds_up_to_gaussian() {
    for id in IDS {
        let mut m = model(id);
        neutral(&mut *m, id);
        set(&mut *m, id, "pdel", 0.0);
        let (l, r) = ir(&mut *m, 0.6);
        let t = if id == "AMBI" { 0.08 } else { 0.25 };
        let ned = 0.5 * (echo_density(&l, t, 1024) + echo_density(&r, t, 1024));
        eprintln!("{id}: echo density at {t} s = {ned:.2}");
        assert!(ned > 0.75, "{id}: {ned}");
    }
}

pub(super) fn corr(a: &[f32], b: &[f32], lag: isize) -> f64 {
    let (mut ab, mut aa, mut bb) = (0.0, 0.0, 0.0);
    for (i, &x) in a.iter().enumerate() {
        let j = i as isize + lag;
        if j < 0 || j as usize >= b.len() {
            continue;
        }
        let (x, y) = (x as f64, b[j as usize] as f64);
        ab += x * y;
        aa += x * x;
        bb += y * y;
    }
    ab / (aa * bb).sqrt()
}

#[test]
fn stereo_outputs_are_decorrelated() {
    for id in IDS {
        let mut m = model(id);
        neutral(&mut *m, id);
        let (l, r) = ir(&mut *m, 1.0);
        let s = (0.05 * FS) as usize;
        let mut worst = 0f64;
        for lag in -48..=48 {
            worst = worst.max(corr(&l[s..], &r[s..], lag).abs());
        }
        eprintln!("{id}: max |IACC| (+-1 ms) = {worst:.3}");
        assert!(worst < 0.3, "{id}: {worst}");
    }
}

#[test]
fn silence_decays_to_exact_zero_without_denormals() {
    for id in IDS {
        let mut m = model(id);
        // defaults (incl. echoes on); impulse, then 30 s of silence
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
                assert!(
                    v == 0.0 || v.abs() >= f32::MIN_POSITIVE,
                    "{id}: subnormal output"
                );
                if v != 0.0 {
                    last_nonzero = b;
                }
            }
        }
        let t = last_nonzero as f64 * 64.0 / FS;
        eprintln!("{id}: output exactly zero after {t:.1} s");
        assert!(t < 28.0, "{id}: still ringing at {t} s");
    }
    // core-level check: the whole state is exactly zero
    let mut h = hall();
    h.prepare(FS, 64);
    let (mut l, mut r) = ([0.0f32; 64], [0.0f32; 64]);
    l[0] = 1.0;
    h.process(&mut l, &mut r, None);
    for _ in 0..(25.0 * FS) as usize / 64 {
        let (mut l, mut r) = ([0.0f32; 64], [0.0f32; 64]);
        h.process(&mut l, &mut r, None);
    }
    let (sum, sub) = h.core().state_report();
    assert!(!sub);
    assert_eq!(sum, 0.0, "HALL state not flushed");
}

pub(super) fn noise(n: usize, seed: u32) -> Vec<f32> {
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
            for (i, p) in spec(id).params.iter().enumerate() {
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

/// |4th-order 6 kHz high-pass| of `x` (click detector for a low-frequency test signal).
pub(super) fn hf_trace(x: &[f32]) -> Vec<f32> {
    let c = Coeffs::highpass(FS, 6000.0, std::f64::consts::FRAC_1_SQRT_2);
    let (mut a, mut b) = (Biquad::new(c), Biquad::new(c));
    x.iter().map(|&v| b.tick(a.tick(v)).abs()).collect()
}

#[test]
fn parameter_sweeps_do_not_click() {
    let blocks_per_sweep = (0.5 * FS) as usize / 64;
    for id in IDS {
        let sp = spec(id);
        for (i, p) in sp.params.iter().enumerate() {
            let slot = i as u16 + 1;
            // low-frequency sine through the model; HF energy only from discontinuities
            let mut m = model(id);
            set(&mut *m, id, "hc", 20_000.0);
            let mut ph = 0.0f32;
            let mut fade = 0.0f32;
            let mut block = |m: &mut dyn Model| {
                let mut l = [0.0f32; 64];
                for v in l.iter_mut() {
                    ph += std::f32::consts::TAU * 150.0 / FS as f32;
                    if ph > std::f32::consts::TAU {
                        ph -= std::f32::consts::TAU;
                    }
                    // 50 ms raised-cosine fade-in: no onset transient in the HF detector
                    fade = (fade + 1.0 / (0.05 * FS as f32)).min(1.0);
                    *v = 0.25 * (1.0 - (std::f32::consts::PI * fade).cos()) * ph.sin();
                }
                let mut r = l;
                m.process(&mut l, &mut r, None);
                l
            };
            // one contiguous output; phases: [steady at min | sweep min->max | steady at max | jump to min]
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
            // abrupt jump back (worst case: one step over the whole range)
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
            let ratio = hf / level;
            if ratio > 0.001 {
                eprintln!(
                    "{id} {}: hf {hf:.5} steady {base:.5} level {level:.4} ratio {ratio:.4}",
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

/// Impulse response of `id` with `keys` set, minus the one with `base` set (isolates the echo stage).
fn ir_diff(
    id: &str,
    keys: &[(&str, f32)],
    base: &[(&str, f32)],
    secs: f64,
) -> (Vec<f32>, Vec<f32>) {
    let run_with = |kv: &[(&str, f32)]| {
        let mut m = model(id);
        neutral(&mut *m, id);
        set(&mut *m, id, "dcy", 0.3);
        for &(k, v) in kv {
            set(&mut *m, id, k, v);
        }
        ir(&mut *m, secs)
    };
    let (a, b) = (run_with(keys), run_with(base));
    let d = |x: &[f32], y: &[f32]| x.iter().zip(y).map(|(p, q)| p - q).collect::<Vec<_>>();
    (d(&a.0, &b.0), d(&a.1, &b.1))
}

pub(super) fn peak(x: &[f32]) -> f32 {
    x.iter().fold(0f32, |a, v| a.max(v.abs()))
}

#[test]
fn chamber_echo_taps_and_room_recirculating_echoes() {
    // CHAMBER: ecl = 100 ms at 0 dB on L only -> a unit echo at 4800 samples on L, nothing before it
    let (l, r) = ir_diff(
        "CHAMBER",
        &[("ell", 0.0), ("ecl", 100.0)],
        &[("ecl", 100.0)],
        0.3,
    );
    let e = 4800;
    let first = l.iter().position(|v| v.abs() > 1e-6);
    assert!(
        peak(&l[..e - 2]) < 1e-6,
        "{} first {first:?}",
        peak(&l[..e - 2])
    );
    // the 20 kHz output low-pass spreads the impulse over a few samples: compare the area
    let area: f32 = l[e - 2..e + 6].iter().sum();
    assert!((area - 1.0).abs() < 0.05, "{area}");
    assert!(
        peak(&r[..e + 200]) < 1e-6,
        "R untouched before the echo's tail"
    );

    // ROOM: ecl = 150 ms, feedback 50 % -> echoes at 150 / 300 ms of 0.3 and 0.15
    let (l, _) = ir_diff(
        "ROOM",
        &[("ecl", 150.0), ("efl", 50.0)],
        &[("ecl", 0.0), ("efl", 0.0)],
        0.5,
    );
    let at = |i: usize| l[i - 2..i + 6].iter().sum::<f32>().abs();
    let (e1, e2) = (at(7200), at(14_400));
    assert!((e1 - room::ECHO_LEVEL).abs() < 0.05, "{e1}");
    assert!((e2 / e1 - 0.5).abs() < 0.1, "{e1} {e2}");
    // negative feedback inverts every other echo
    let (l, _) = ir_diff(
        "ROOM",
        &[("ecl", 150.0), ("efl", -50.0)],
        &[("ecl", 0.0), ("efl", 0.0)],
        0.5,
    );
    let s1 = l[7198..7203]
        .iter()
        .copied()
        .fold(0f32, |a, v| if v.abs() > a.abs() { v } else { a });
    let s2 = l[14_398..14_403]
        .iter()
        .copied()
        .fold(0f32, |a, v| if v.abs() > a.abs() { v } else { a });
    assert!(s1 * s2 < 0.0, "{s1} {s2}");
}

#[test]
fn pre_delay_shifts_onset() {
    let onset = |pdel: f32| {
        let mut m = model("HALL");
        neutral(&mut *m, "HALL");
        set(&mut *m, "HALL", "pdel", pdel);
        let (l, _) = ir(&mut *m, 0.4);
        l.iter().position(|v| v.abs() > 1e-3).unwrap()
    };
    let (a, b) = (onset(0.0), onset(100.0));
    assert!((b as i64 - a as i64 - 4800).abs() <= 2, "{a} {b}");
}

#[test]
fn size_scales_loop_lengths() {
    let mut h = hall();
    h.prepare(FS, 64);
    let small = h.core().loop_lengths()[0];
    h.set_param(slot_of("HALL", "size"), ParamValue::Float(100.0));
    h.reset();
    let big = h.core().loop_lengths()[0];
    assert!(big > small * 1.3, "{small} {big}");
}

#[test]
fn params_round_trip_defaults_and_clamp() {
    for id in IDS {
        let sp = spec(id);
        let mut m = create(id).unwrap();
        m.prepare(FS, 64);
        for (i, p) in sp.params.iter().enumerate() {
            assert_eq!(m.param(i as u16 + 1), Some(p.default), "{id} {}", p.key);
        }
        assert_eq!(m.param(0), None);
        assert_eq!(m.param(sp.params.len() as u16 + 1), None);
        m.set_param(1, ParamValue::Float(1e9));
        assert_eq!(m.param(1), Some(sp.params[0].max));
        m.set_param(1, ParamValue::Int(-5));
        assert_eq!(m.param(1), Some(sp.params[0].min));
        m.set_param(1, ParamValue::Float(f32::NAN));
        assert_eq!(m.param(1), Some(sp.params[0].min));
        m.load_defaults();
        assert_eq!(m.param(1), Some(sp.params[0].default));
        assert_eq!(m.latency(), 0);
    }
    assert!(create("PLATE").is_none());
    assert_eq!(fader_gain(-144.0), 0.0);
    assert!((fader_gain(-20.0) - 0.1).abs() < 1e-6);
}
