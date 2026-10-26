//! Behaviour tests for WAVE / RIDE / WARM (T049).

use super::*;
use crate::fidelity::run_mono;
use core::f64::consts::PI;

const FS: f64 = 48_000.0;

fn f(v: f32) -> ParamValue {
    ParamValue::Float(v)
}

fn setup<M: Model>(mut m: M, params: &[(u16, ParamValue)]) -> M {
    m.prepare(FS, 64);
    for &(s, v) in params {
        m.set_param(s, v);
    }
    m.commit();
    m.reset();
    m
}

fn tone(freq: f64, amp: f64, n: usize) -> Vec<f32> {
    (0..n)
        .map(|i| (amp * (2.0 * PI * freq * i as f64 / FS).sin()) as f32)
        .collect()
}

fn db(x: f64) -> f64 {
    20.0 * x.abs().max(1e-30).log10()
}

fn rms(x: &[f32]) -> f64 {
    (x.iter().map(|&v| (v as f64).powi(2)).sum::<f64>() / x.len().max(1) as f64).sqrt()
}

/// Single-bin amplitude with a 4-term Blackman-Harris window.
fn amp_at(x: &[f32], freq: f64) -> f64 {
    let n = x.len() as f64;
    let w = 2.0 * PI * freq / FS;
    let (mut re, mut im, mut ws) = (0.0f64, 0.0f64, 0.0f64);
    for (i, &v) in x.iter().enumerate() {
        let t = 2.0 * PI * i as f64 / n;
        let win =
            0.35875 - 0.48829 * t.cos() + 0.14128 * (2.0 * t).cos() - 0.01168 * (3.0 * t).cos();
        let p = w * i as f64;
        re += win * v as f64 * p.cos();
        im -= win * v as f64 * p.sin();
        ws += win;
    }
    2.0 * (re * re + im * im).sqrt() / ws
}

/// Windowed gain (dB) of `y` over `x` in `win`-sample windows.
fn window_gains(x: &[f32], y: &[f32], win: usize) -> Vec<f64> {
    x.chunks(win)
        .zip(y.chunks(win))
        .map(|(a, b)| db(rms(b) / rms(a).max(1e-12)))
        .collect()
}

/// Drum-like hit: sine burst, instant onset at `t0` s, exponential decay of `decay_db_s`.
fn hit(freq: f64, amp: f64, decay_db_s: f64, t0: f64, secs: f64) -> Vec<f32> {
    let n = (secs * FS) as usize;
    (0..n)
        .map(|i| {
            let t = i as f64 / FS;
            if t < t0 {
                0.0
            } else {
                let e = 10f64.powf(-decay_db_s * (t - t0) / 20.0);
                (amp * e * (2.0 * PI * freq * t).sin()) as f32
            }
        })
        .collect()
}

fn peak(x: &[f32]) -> f64 {
    x.iter().fold(0.0f64, |m, &v| m.max(v.abs() as f64))
}

// ---- WAVE ------------------------------------------------------------------------------------------------

#[test]
fn wave_defaults_are_unity_and_bit_exact() {
    let mut m = setup(TransientDesigner::new(), &[]);
    let x = hit(200.0, 0.5, 40.0, 0.05, 0.5);
    let y = run_mono(&mut m, &x);
    assert_eq!(x, y);
    assert_eq!(m.id(), "WAVE");
    assert_eq!(m.param(3), Some(0.0));
}

#[test]
fn wave_output_gain_param() {
    let mut m = setup(TransientDesigner::new(), &[(3, f(6.0))]);
    let x = tone(1000.0, 0.1, 9600);
    let y = run_mono(&mut m, &x);
    let g = db(amp_at(&y[4800..], 1000.0) / amp_at(&x[4800..], 1000.0));
    assert!((g - 6.0).abs() < 0.1, "{g}");
}

#[test]
fn wave_attack_shapes_the_onset_only() {
    // 60 dB/s decay: onset in the first ~30 ms, tail afterwards.
    let x = hit(200.0, 0.4, 60.0, 0.1, 0.8);
    let i0 = (0.1 * FS) as usize;
    let w = |a: f64, b: f64| ((a * FS) as usize + i0)..((b * FS) as usize + i0);
    let lvl = |y: &[f32], r: core::ops::Range<usize>| db(rms(&y[r]));
    let run = |att: f32| {
        let mut m = setup(TransientDesigner::new(), &[(1, f(att))]);
        run_mono(&mut m, &x)
    };
    let (dry, up, dn) = (run(0.0), run(12.0), run(-12.0));
    let on = lvl(&up, w(0.0, 0.01)) - lvl(&dry, w(0.0, 0.01));
    let od = lvl(&dn, w(0.0, 0.01)) - lvl(&dry, w(0.0, 0.01));
    let tu = lvl(&up, w(0.25, 0.35)) - lvl(&dry, w(0.25, 0.35));
    let td = lvl(&dn, w(0.25, 0.35)) - lvl(&dry, w(0.25, 0.35));
    assert!(on > 6.0 && on <= 12.5, "attack +12: onset {on}");
    assert!((-12.5..-6.0).contains(&od), "attack -12: onset {od}");
    assert!(tu.abs() < 0.5 && td.abs() < 0.5, "tail moved {tu} {td}");
    assert!(peak(&up) > peak(&dry) * 1.5 && peak(&dn) < peak(&dry));
}

#[test]
fn wave_sustain_shapes_the_tail_only() {
    let x = hit(200.0, 0.4, 120.0, 0.1, 0.8);
    let i0 = (0.1 * FS) as usize;
    let w = |a: f64, b: f64| ((a * FS) as usize + i0)..((b * FS) as usize + i0);
    let lvl = |y: &[f32], r: core::ops::Range<usize>| db(rms(&y[r]));
    let run = |sust: f32| {
        let mut m = setup(TransientDesigner::new(), &[(2, f(sust))]);
        run_mono(&mut m, &x)
    };
    let (dry, up, dn) = (run(0.0), run(12.0), run(-12.0));
    let first = |y: &[f32]| lvl(y, w(0.0, 0.005));
    assert!((first(&up) - first(&dry)).abs() < 0.5, "onset moved");
    let tail_up = lvl(&up, w(0.05, 0.09)) - lvl(&dry, w(0.05, 0.09));
    let tail_dn = lvl(&dn, w(0.05, 0.09)) - lvl(&dry, w(0.05, 0.09));
    assert!(tail_up > 4.0 && tail_up <= 12.5, "sustain +12: {tail_up}");
    assert!((-12.5..-4.0).contains(&tail_dn), "sustain -12: {tail_dn}");
}

#[test]
fn wave_is_level_independent() {
    let x1 = hit(200.0, 0.02, 60.0, 0.1, 0.7);
    let x2: Vec<f32> = x1.iter().map(|v| v * 15.85).collect(); // +24 dB
    let params = [(1, f(10.0)), (2, f(-10.0))];
    let y1 = run_mono(&mut setup(TransientDesigner::new(), &params), &x1);
    let y2 = run_mono(&mut setup(TransientDesigner::new(), &params), &x2);
    let g1 = window_gains(&x1, &y1, 480);
    let g2 = window_gains(&x2, &y2, 480);
    let worst = g1
        .iter()
        .zip(&g2)
        .skip(10)
        .take(40)
        .map(|(a, b)| (a - b).abs())
        .fold(0.0, f64::max);
    assert!(worst < 0.15, "gain trajectories differ by {worst} dB");
    assert!(g1.iter().any(|g| g.abs() > 3.0), "the gain actually moved");
}

#[test]
fn wave_steady_tone_passes_at_unity() {
    let x = tone(1000.0, 0.3, 48_000);
    let mut m = setup(TransientDesigner::new(), &[(1, f(15.0)), (2, f(24.0))]);
    let y = run_mono(&mut m, &x);
    let g = db(amp_at(&y[24_000..], 1000.0) / amp_at(&x[24_000..], 1000.0));
    assert!(g.abs() < 0.3, "{g}");
}

#[test]
fn wave_bass_ripple_does_not_modulate_the_gain() {
    // Steady 50 Hz at the strongest settings: the rectified-peak ripple stays inside the dead zone.
    let x = tone(50.0, 0.5, 48_000);
    let mut m = setup(TransientDesigner::new(), &[(1, f(15.0)), (2, f(24.0))]);
    let y = run_mono(&mut m, &x);
    let g = window_gains(&x[24_000..], &y[24_000..], 96);
    let worst = g.iter().fold(0.0f64, |m, v| m.max(v.abs()));
    assert!(worst < 0.5, "{worst} dB gain ripple");
}

#[test]
fn wave_floor_does_not_boost_the_noise_floor() {
    let x = hit(300.0, 1e-5, 200.0, 0.01, 0.3);
    let mut m = setup(TransientDesigner::new(), &[(2, f(24.0))]);
    let y = run_mono(&mut m, &x);
    assert!(peak(&y) < 1e-5 * 4.0, "{}", peak(&y));
}

#[test]
fn wave_sidechain_keys_the_detector() {
    let sig = tone(1000.0, 0.2, 24_000);
    let key = hit(200.0, 0.5, 60.0, 0.1, 0.5);
    let mut m = setup(TransientDesigner::new(), &[(1, f(12.0))]);
    let (mut l, mut r) = (sig.clone(), sig.clone());
    for (a, (b, k)) in l.chunks_mut(64).zip(r.chunks_mut(64).zip(key.chunks(64))) {
        m.process(a, b, Some((k, k)));
    }
    let g = window_gains(&sig, &l, 240);
    let onset = (0.1 * FS / 240.0) as usize;
    assert!(g[onset] > 3.0, "keyed onset gain {}", g[onset]);
    assert!(g[2].abs() < 0.1);
}

// ---- RIDE ------------------------------------------------------------------------------------------------

/// Sine amplitude with the given RMS level (dBFS, full-scale sine = -3 dB).
fn amp_for_rms_db(l: f64) -> f64 {
    10f64.powf(l / 20.0) * 2f64.sqrt()
}

#[test]
fn ride_defaults_and_params() {
    let mut m = setup(AutoRider::new(), &[]);
    assert_eq!(m.id(), "RIDE");
    let d: Vec<f32> = (1..=6).map(|s| m.param(s).unwrap()).collect();
    assert_eq!(d, vec![-40.0, -18.0, 10.0, 4.0, 1.5, 6.0]);
    assert_eq!(m.param(7), None);
    m.set_param(3, f(99.0));
    assert_eq!(m.param(3), Some(50.0));
    m.set_param(4, ParamValue::Enum(2));
    assert_eq!(m.param(4), Some(8.0));
    m.set_param(4, f(100.0));
    assert_eq!(m.param(4), Some(100.0));
    m.set_param(4, ParamValue::Enum(9));
    assert_eq!(m.param(4), Some(100.0));
}

#[test]
fn ride_lifts_quiet_signal_toward_target_scaled_by_ratio() {
    // level -30 dB RMS, target -18: error 12 dB; ratio 100 -> 11.9 dB, ratio 4 -> 9 dB, ratio 2 -> 6 dB.
    let x = tone(500.0, amp_for_rms_db(-30.0), 48_000 * 12);
    for (ratio, want) in [(100.0, 11.88), (4.0, 9.0), (2.0, 6.0)] {
        let mut m = setup(
            AutoRider::new(),
            &[(3, f(50.0)), (4, f(ratio)), (6, f(15.0))],
        );
        run_mono(&mut m, &x);
        assert!(
            (m.gain_db() - want).abs() < 0.3,
            "ratio {ratio}: {}",
            m.gain_db()
        );
    }
}

#[test]
fn ride_pulls_loud_signal_down_and_range_limits() {
    let x = tone(500.0, amp_for_rms_db(-6.0), 48_000 * 6);
    let mut m = setup(
        AutoRider::new(),
        &[(3, f(50.0)), (4, f(100.0)), (6, f(15.0))],
    );
    run_mono(&mut m, &x);
    assert!((m.gain_db() + 12.0).abs() < 0.3, "{}", m.gain_db());
    let mut m = setup(
        AutoRider::new(),
        &[(3, f(50.0)), (4, f(100.0)), (6, f(4.0))],
    );
    let y = run_mono(&mut m, &x);
    assert!((m.gain_db() + 4.0).abs() < 0.05, "{}", m.gain_db());
    let g = db(rms(&y[48_000 * 5..]) / rms(&x[48_000 * 5..]));
    assert!((g + 4.0).abs() < 0.2, "{g}");
    assert!(m.telemetry().gain_reduction_db > 3.9);
    assert!(m.telemetry().engaged);
}

#[test]
fn ride_is_speed_controlled() {
    let x = tone(500.0, amp_for_rms_db(-30.0), 48_000 * 2);
    let gain_after = |spd: f32| {
        let mut m = setup(
            AutoRider::new(),
            &[(3, f(spd)), (4, f(100.0)), (6, f(15.0))],
        );
        run_mono(&mut m, &x);
        m.gain_db()
    };
    let (slow, mid, fast) = (gain_after(1.0), gain_after(20.0), gain_after(50.0));
    assert!(slow < mid && mid < fast, "{slow} {mid} {fast}");
    // spd 1 climbs at ~2 dB/s for ~1.9 s (the 100 ms level detector settles first)
    assert!((slow - 3.8).abs() < 0.8, "{slow}");
    assert!((ride_rate_db_s(1.0) - 2.0).abs() < 1e-9);
    assert!((ride_rate_db_s(50.0) - 200.0).abs() < 1e-6);
}

#[test]
fn ride_freezes_below_threshold_then_holds_then_relaxes() {
    let loud = tone(500.0, amp_for_rms_db(-30.0), 48_000 * 6);
    let mut m = setup(
        AutoRider::new(),
        &[
            (1, f(-40.0)),
            (3, f(50.0)),
            (4, f(100.0)),
            (6, f(10.0)),
            (5, f(1.0)),
        ],
    );
    run_mono(&mut m, &loud);
    let g0 = m.gain_db();
    assert!(g0 > 9.0, "{g0}");
    // The 100 ms level detector needs ~0.2 s to fall below -40 dB; then the 1 s hold keeps the gain.
    let quiet = tone(500.0, amp_for_rms_db(-60.0), 48_000 / 2);
    run_mono(&mut m, &quiet);
    assert!(
        (m.gain_db() - g0).abs() < 0.01,
        "held through 0.5 s: {}",
        m.gain_db()
    );
    // After the hold the gain relaxes toward 0 dB.
    run_mono(&mut m, &tone(500.0, amp_for_rms_db(-60.0), 48_000 * 3));
    assert!(m.gain_db() < 0.5, "{}", m.gain_db());
    assert!(!m.telemetry().engaged);
}

#[test]
fn ride_hold_inf_never_releases() {
    let loud = tone(500.0, amp_for_rms_db(-30.0), 48_000 * 3);
    let mut m = setup(
        AutoRider::new(),
        &[(3, f(50.0)), (4, f(100.0)), (6, f(10.0)), (5, f(10.0))],
    );
    run_mono(&mut m, &loud);
    let g0 = m.gain_db();
    run_mono(&mut m, &tone(500.0, amp_for_rms_db(-70.0), 48_000 * 20));
    assert!((m.gain_db() - g0).abs() < 0.01);
}

#[test]
fn ride_sidechain_keys_the_level() {
    let sig = tone(500.0, 0.2, 48_000 * 4);
    let key = tone(500.0, amp_for_rms_db(-30.0), 48_000 * 4);
    let mut m = setup(
        AutoRider::new(),
        &[(3, f(50.0)), (4, f(100.0)), (6, f(15.0))],
    );
    let (mut l, mut r) = (sig.clone(), sig.clone());
    for (a, (b, k)) in l.chunks_mut(64).zip(r.chunks_mut(64).zip(key.chunks(64))) {
        m.process(a, b, Some((k, k)));
    }
    assert!((m.gain_db() - 11.9).abs() < 0.4, "{}", m.gain_db());
}

#[test]
fn ride_meter_mapping_matches_reference() {
    for (gr, want) in [
        (0.0, 0.0),
        (3.0, -0.2),
        (6.0, -0.4),
        (15.0, -1.0),
        (40.0, -1.0),
    ] {
        assert!((ride_gr_meter(gr) - want).abs() < 1e-6);
    }
    assert_eq!(wave_gr_meter(7.0), 1.0);
}

// ---- WARM ------------------------------------------------------------------------------------------------

fn warm(params: &[(u16, ParamValue)]) -> WarmthPre {
    setup(WarmthPre::new(), params)
}

fn tail(m: &mut WarmthPre, freq: f64, amp: f64) -> Vec<f32> {
    m.reset();
    let y = run_mono(m, &tone(freq, amp, 24_000 + 48_000));
    y[24_000..].to_vec()
}

/// (H1, H2, H3) amplitudes.
fn harmonics(y: &[f32], f0: f64) -> (f64, f64, f64) {
    (amp_at(y, f0), amp_at(y, 2.0 * f0), amp_at(y, 3.0 * f0))
}

#[test]
fn warm_small_signal_is_unity_and_dry_matches() {
    let mut m = warm(&[]);
    for freq in [100.0, 1000.0, 8000.0] {
        let y = tail(&mut m, freq, 0.003);
        let g = db(amp_at(&y, freq) / 0.003);
        assert!(g.abs() < 0.15, "{freq}: {g}");
    }
    assert_eq!(m.latency(), WARM_LATENCY);
    assert_eq!(m.id(), "WARM");
    // mix 0 = delay-compensated dry, bit exact
    let mut m = warm(&[(5, f(0.0))]);
    let x = tone(777.0, 0.5, 2000);
    let y = run_mono(&mut m, &x);
    assert_eq!(&y[WARM_LATENCY..], &x[..x.len() - WARM_LATENCY]);
}

#[test]
fn warm_hrm_sweeps_second_to_third() {
    let h = |hrm: f32| {
        // below clipping: the bias sweep is a small-signal property (T088: 80 % drive clips 0.25 hard)
        let mut m = warm(&[(1, f(80.0)), (2, f(hrm))]);
        let y = tail(&mut m, 1000.0, 0.08);
        harmonics(&y, 1000.0)
    };
    let (f1, h2e, h3e) = h(-100.0);
    let (_, h2m, h3m) = h(0.0);
    let (_, h2o, h3o) = h(100.0);
    assert!(
        db(h2e / h3e) > 20.0,
        "even end: H2 {} H3 {}",
        db(h2e / f1),
        db(h3e / f1)
    );
    assert!(
        db(h3o / h2o.max(1e-12)) > 50.0,
        "odd end: H2 {h2o} H3 {h3o}"
    );
    assert!(db(h3o / f1) > -60.0);
    assert!(
        h2m / f1 > 1e-3 && h3m / f1 > 1e-4,
        "{} {}",
        h2m / f1,
        h3m / f1
    );
    assert!(h2e > h2m && h2m > h2o && h3o > h3m && h3m > h3e);
}

#[test]
fn warm_thd_grows_with_drive_and_level() {
    let thd = |drv: f32, amp: f64| {
        let mut m = warm(&[(1, f(drv)), (2, f(-30.0))]);
        let y = tail(&mut m, 1000.0, amp);
        let (h1, h2, h3) = harmonics(&y, 1000.0);
        ((h2 * h2 + h3 * h3).sqrt() / h1) * 100.0
    };
    let (lo, mid, hi) = (thd(10.0, 0.25), thd(33.0, 0.25), thd(125.0, 0.25));
    assert!(lo < mid && mid < hi, "{lo} {mid} {hi}");
    assert!(lo < 1.5, "drive 10 % nearly clean: {lo}");
    // T081: the top of the range clearly saturates at channel level (-12 dBFS) and is still audible at -30
    assert!(hi > 10.0, "drive 125 % at -12 dBFS: {hi} %");
    assert!(thd(125.0, 0.0316) > 1.0, "drive 125 % at -30 dBFS");
    assert!(thd(60.0, 0.025) < thd(60.0, 0.25) / 5.0);
}

#[test]
fn warm_thd_profile() {
    for (drv, hrm, amp) in [
        (33.0, 0.0, 0.25),
        (33.0, -100.0, 0.25),
        (33.0, 100.0, 0.25),
        (80.0, 0.0, 0.25),
        (125.0, 0.0, 0.25),
        (125.0, 0.0, 0.9),
    ] {
        let mut m = warm(&[(1, f(drv)), (2, f(hrm))]);
        let y = tail(&mut m, 1000.0, amp);
        let (h1, h2, h3) = harmonics(&y, 1000.0);
        println!(
            "drv {drv} hrm {hrm} amp {amp}: H2 {:.2} dB H3 {:.2} dB THD {:.3} %",
            db(h2 / h1),
            db(h3 / h1),
            100.0 * (h2 * h2 + h3 * h3).sqrt() / h1
        );
    }
}

#[test]
fn warm_has_no_dc_offset() {
    let mut m = warm(&[(1, f(125.0)), (2, f(-100.0))]);
    let y = tail(&mut m, 200.0, 0.8);
    let mean = y.iter().map(|&v| v as f64).sum::<f64>() / y.len() as f64;
    assert!(mean.abs() < 2e-3, "{mean}");
}

#[test]
fn warm_trim_and_mix() {
    let a = 0.003;
    let mut m = warm(&[(4, f(6.0))]);
    let y = tail(&mut m, 1000.0, a);
    assert!((db(amp_at(&y, 1000.0) / a) - 6.0).abs() < 0.2);
    let mut m = warm(&[(5, f(50.0))]);
    let y = tail(&mut m, 1000.0, a);
    assert!(db(amp_at(&y, 1000.0) / a).abs() < 0.2);
}

#[test]
fn warm_colour_tilts_the_spectrum_and_distortion() {
    let level = |col: f32, freq: f64| {
        let mut m = warm(&[(1, f(10.0)), (3, f(col))]);
        let y = tail(&mut m, freq, 0.003);
        db(amp_at(&y, freq) / 0.003)
    };
    for col in [1.0f32, -1.0] {
        let tilt = level(col, 8000.0) - level(col, 100.0);
        let want = col as f64 * WARM_TILT_DB * (1.0 - WARM_TILT_RESTORE);
        assert!((tilt - want).abs() < 1.0, "col {col}: {tilt} vs {want}");
    }
    // pre-tilt up: the shaper sees more HF, so the 3rd harmonic of a 3 kHz tone rises with col
    let h3 = |col: f32| {
        let mut m = warm(&[(1, f(100.0)), (2, f(100.0)), (3, f(col))]);
        let y = tail(&mut m, 3000.0, 0.25);
        let (a, _, c) = harmonics(&y, 3000.0);
        db(c / a)
    };
    assert!(h3(1.0) > h3(0.0) + 1.0 && h3(0.0) > h3(-1.0) + 1.0);
}

#[test]
fn warm_is_alias_free() {
    // 10 kHz tones: the output is periodic in 24 samples (2 kHz grid). 10k and 20k are the in-band harmonics;
    // every other grid line is aliasing. Bound per case `(drv, hrm, amp, worst alias dBc)`: moderate drive stays
    // < -110 dBc even hot; 125 % drive at -30 dBFS < -120 dBc. T088 raised the top drive to +30 dB over the
    // default, so a -20 dBFS 10 kHz tone at 125 % is already clipped hard (~-67 dBc) and the extreme corner
    // (a near-square 10 kHz wave) is bounded by 2x oversampling + first-order ADAA at ~-50 dBc. Programme has
    // little energy that high; measured values + ~3 dB margin.
    for (drv, hrm, amp, bound) in [
        (60.0, 0.0, 0.5, -110.0),
        (80.0, 0.0, 0.25, -110.0),
        (125.0, 0.0, 0.03, -120.0),
        (125.0, 0.0, 0.1, -63.0),
        (125.0, -100.0, 0.1, -66.0),
        (100.0, 100.0, 0.25, -82.0),
        (125.0, 0.0, 0.5, -47.0),
        (125.0, 0.0, 0.9, -49.0),
        (125.0, -100.0, 0.9, -46.0),
    ] {
        let mut m = warm(&[(1, f(drv)), (2, f(hrm))]);
        let y = tail(&mut m, 10_000.0, amp);
        let h1 = amp_at(&y, 10_000.0);
        let mut worst = f64::MIN;
        for k in 1..12 {
            let fk = 2000.0 * k as f64;
            if fk == 10_000.0 || fk == 20_000.0 {
                continue;
            }
            worst = worst.max(db(amp_at(&y, fk) / h1));
        }
        println!("drv {drv} hrm {hrm} amp {amp}: worst alias {worst:.1} dBc");
        assert!(worst < bound, "drv {drv} hrm {hrm} amp {amp}: {worst}");
    }
}

#[test]
fn warm_chunking_is_transparent() {
    let x = tone(440.0, 0.4, 3000);
    let mut a = warm(&[(1, f(90.0)), (2, f(-40.0)), (3, f(0.3))]);
    let mut b = warm(&[(1, f(90.0)), (2, f(-40.0)), (3, f(0.3))]);
    let (mut l, mut r) = (x.clone(), x.clone());
    a.process(&mut l, &mut r, None);
    let yb = run_mono(&mut b, &x);
    let d = l
        .iter()
        .zip(&yb)
        .map(|(p, q)| (p - q).abs())
        .fold(0.0, f32::max);
    assert!(d < 1e-6, "{d}");
}

#[test]
fn warm_params_clamp_and_defaults() {
    let mut m = warm(&[]);
    let d: Vec<f32> = (1..=5).map(|s| m.param(s).unwrap()).collect();
    assert_eq!(d, vec![33.0, 0.0, 0.0, 0.0, 100.0]);
    m.set_param(1, f(500.0));
    m.set_param(2, f(-500.0));
    m.set_param(9, f(1.0));
    assert_eq!(m.param(1), Some(125.0));
    assert_eq!(m.param(2), Some(-100.0));
    assert_eq!(m.param(6), None);
    m.load_defaults();
    assert_eq!(m.param(1), Some(33.0));
}

#[test]
fn models_are_not_stubs_and_survive_extremes() {
    use crate::model::{create_model, BlockKind};
    for id in ["WAVE", "RIDE", "WARM"] {
        let mut m = create_model(BlockKind::Dynamics, id);
        assert!(!m.is_stub(), "{id}");
        m.prepare(FS, 64);
        for s in 1..=6u16 {
            m.set_param(s, ParamValue::Float(1e9));
        }
        m.commit();
        let mut l: Vec<f32> = (0..4096)
            .map(|i| ((i * 7919) % 200) as f32 / 100.0 - 1.0)
            .collect();
        let mut r = l.clone();
        for (a, b) in l.chunks_mut(64).zip(r.chunks_mut(64)) {
            m.process(a, b, None);
        }
        assert!(l.iter().chain(&r).all(|v| v.is_finite()), "{id}");
        for s in 1..=6u16 {
            m.set_param(s, ParamValue::Float(-1e9));
        }
        m.reset();
        m.process(&mut l, &mut r, None);
        assert!(l.iter().all(|v| v.is_finite()), "{id}");
    }
}

/// (THD % over H2..H10, output/input RMS gain dB) of a 1 kHz sine at `rms_dbfs` (T088 targets).
fn warm_thd(drv: f32, hrm: f32, rms_dbfs: f64) -> (f64, f64) {
    let amp = 10f64.powf(rms_dbfs / 20.0) * 2f64.sqrt();
    let mut m = warm(&[(1, f(drv)), (2, f(hrm))]);
    let y = tail(&mut m, 1000.0, amp);
    let h1 = amp_at(&y, 1000.0);
    let hs: f64 = (2..=10)
        .map(|k| amp_at(&y, 1000.0 * k as f64).powi(2))
        .sum();
    (100.0 * hs.sqrt() / h1, db(rms(&y) / (amp / 2f64.sqrt())))
}

/// T088: drive is clearly audible at nominal levels and the level stays roughly compensated. Targets (1 kHz
/// sine, `hrm` 0, see the module docs for their sources): default drive subtle (< 2 % at -18 dBFS RMS),
/// 125 % overt (> 20 % at -18, > 8 % at -24, > 2 % at -30 dBFS RMS), THD monotonic in drive; at the
/// -18 dBFS reference the output RMS stays within 1 dB of the input for every drive and harmonics setting.
#[test]
fn warm_drive_targets_at_nominal_level() {
    for hrm in [-100.0f32, 0.0, 100.0] {
        let mut last = 0.0;
        for drv in [10.0f32, 33.0, 60.0, 80.0, 100.0, 125.0] {
            let (thd, gain) = warm_thd(drv, hrm, -18.0);
            assert!(
                thd > last,
                "hrm {hrm} drv {drv}: THD {thd} not above {last}"
            );
            last = thd;
            assert!(
                gain.abs() < 1.0,
                "hrm {hrm} drv {drv}: level {gain} dB at -18 dBFS"
            );
        }
    }
    assert!(warm_thd(33.0, 0.0, -18.0).0 < 2.0);
    assert!(warm_thd(125.0, 0.0, -18.0).0 > 20.0);
    assert!(warm_thd(125.0, 0.0, -24.0).0 > 8.0);
    assert!(warm_thd(125.0, 0.0, -30.0).0 > 2.0);
    // the 2nd-harmonic end is as audible at full drive
    assert!(warm_thd(125.0, -100.0, -24.0).0 > 8.0);
}

/// Prints the WARM THD / level table of the module docs (`--nocapture`).
#[test]
fn warm_thd_table() {
    for hrm in [-100.0f32, 0.0, 100.0] {
        for drv in [10.0f32, 33.0, 60.0, 80.0, 100.0, 125.0] {
            let row: Vec<String> = [-30.0, -24.0, -18.0, -12.0]
                .iter()
                .map(|&l| {
                    let (t, g) = warm_thd(drv, hrm, l);
                    format!("{t:6.2} % ({g:+5.1} dB)")
                })
                .collect();
            println!("hrm {hrm:5} drv {drv:5}: {}", row.join("  "));
        }
    }
}
