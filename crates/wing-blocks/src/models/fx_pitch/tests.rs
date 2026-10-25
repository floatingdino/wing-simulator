use super::*;
use crate::model::{create_model, BlockKind};

const FS: f64 = 48_000.0;
const IDS: [&str; 5] = ["PITCH", "D-PITCH", "DOUBLE", "PCORR", "SUB"];

fn sine(f: f64, amp: f32, n: usize) -> Vec<f32> {
    (0..n)
        .map(|i| amp * (2.0 * PI * f * i as f64 / FS).sin() as f32)
        .collect()
}

/// Harmonic tone: harmonics 1..=5 at 1/k amplitude.
fn tone(f: f64, amp: f32, n: usize) -> Vec<f32> {
    (0..n)
        .map(|i| {
            let t = 2.0 * PI * f * i as f64 / FS;
            let s: f64 = (1..=5).map(|k| (k as f64 * t).sin() / k as f64).sum();
            amp * 0.5 * s as f32
        })
        .collect()
}

fn run(m: &mut dyn Model, l: &mut [f32], r: &mut [f32]) {
    for (a, b) in l.chunks_mut(64).zip(r.chunks_mut(64)) {
        m.process(a, b, None);
    }
}

/// Hann-windowed DFT magnitude of `x` at `f`.
fn dft_mag(x: &[f32], f: f64) -> f64 {
    let n = x.len();
    let w = 2.0 * PI * f / FS;
    let (mut re, mut im) = (0.0f64, 0.0f64);
    for (i, &v) in x.iter().enumerate() {
        let h = 0.5 - 0.5 * (2.0 * PI * i as f64 / n as f64).cos();
        let p = w * i as f64;
        re += h * v as f64 * p.cos();
        im += h * v as f64 * p.sin();
    }
    (re * re + im * im).sqrt() * 4.0 / n as f64
}

/// Spectral peak near `f` (searched over +/-40 cents in 1-cent steps, parabolic fit): offset in cents.
fn peak_cents(x: &[f32], f: f64) -> f64 {
    let mags: Vec<f64> = (-40..=40)
        .map(|c| dft_mag(x, f * 2f64.powf(c as f64 / 1200.0)))
        .collect();
    let (i, _) = mags
        .iter()
        .enumerate()
        .fold((0, 0.0), |b, (i, &m)| if m > b.1 { (i, m) } else { b });
    let mut c = i as f64 - 40.0;
    if i > 0 && i < mags.len() - 1 {
        let (a, b, d) = (mags[i - 1], mags[i], mags[i + 1]);
        c += 0.5 * (a - d) / (a - 2.0 * b + d);
    }
    c
}

// ----- registry / parameters ---------------------------------------------------------------------------

fn schema_num(s: &str) -> f32 {
    if let Some((a, b)) = s.split_once('k') {
        let v: f32 = format!("{a}.{b}").parse().unwrap();
        return v * 1000.0;
    }
    s.parse().unwrap()
}

#[test]
fn every_schema_parameter_is_mapped_with_its_default() {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../data/schema/wing-tree.json"
    );
    let tree: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(path).unwrap()).unwrap();
    let variants = tree["variants"]["fx/1"]["mdl"]["variants"]
        .as_array()
        .unwrap();
    for id in IDS {
        let v = variants.iter().find(|v| v["name"] == id).unwrap();
        let params = v["params"].as_array().unwrap();
        let mut m = create_model(BlockKind::Fx, id);
        m.prepare(FS, 64);
        m.load_defaults();
        assert!(!m.is_stub());
        for (i, p) in params.iter().enumerate() {
            let slot = (i + 1) as u16;
            let got = m
                .param(slot)
                .unwrap_or_else(|| panic!("{id} slot {slot} unmapped"));
            let def = p["default"].as_str().unwrap();
            let want = match p["values"].as_array() {
                Some(vals) => vals.iter().position(|t| t == def).unwrap() as f32,
                None => schema_num(def),
            };
            assert!(
                (got - want).abs() <= 0.01 * want.abs().max(1.0),
                "{id} {} default {got} != {want}",
                p["name"]
            );
            m.set_param(slot, ParamValue::Float(want));
            assert_eq!(m.param(slot), Some(want), "{id} {}", p["name"]);
        }
        assert_eq!(m.param(params.len() as u16 + 1), None, "{id}");
        let lat = if id == "PCORR" { 360 } else { 0 };
        assert_eq!(m.latency(), lat, "{id}");
        assert_eq!(m.id(), id);
    }
}

#[test]
fn latency_is_low_and_scales_with_rate() {
    for (fs, want) in [(44_100.0, 331), (48_000.0, 360), (96_000.0, 720)] {
        let mut m = PitchFix::new();
        m.prepare(fs, 64);
        assert_eq!(m.latency(), want);
        assert!(m.latency() as f64 <= 0.0105 * fs);
    }
}

// ----- shifter core ------------------------------------------------------------------------------------

#[test]
fn correlation_search_lands_on_a_period_multiple() {
    let mut line = DelayLine::new(4096);
    let f = 233.0; // period 206.0 samples
    for x in sine(f, 0.5, 4000) {
        line.push(x);
    }
    let per = FS / f;
    for (d, dir) in [(10.0, 1.0), (900.0, -1.0), (37.3, 1.0)] {
        let j = search_jump(&line, d, dir, 300.0, 700.0, 384);
        let k = (j / per).round();
        assert!(
            (j - k * per).abs() < 0.2,
            "d {d}: jump {j} not a multiple of {per}"
        );
    }
}

// ----- PITCH -------------------------------------------------------------------------------------------

fn pitch_out(semi: f32, cent: f32, x: &[f32]) -> (Vec<f32>, Vec<f32>) {
    let mut m = StereoPitch::new();
    m.prepare(FS, 64);
    m.set_params(&PitchParams {
        semi,
        cent,
        dly_ms: 0.0,
        lc_hz: 20.0,
        hc_hz: 20_000.0,
        mix_pct: 100.0,
    });
    let (mut l, mut r) = (x.to_vec(), x.to_vec());
    run(&mut m, &mut l, &mut r);
    (l, r)
}

#[test]
fn pitch_is_accurate_on_sines() {
    let n = 60_000;
    for (f, semi, cent) in [
        (440.0, 12.0, 0.0),
        (440.0, -12.0, 0.0),
        (220.0, 7.0, 0.0),
        (1000.0, -5.0, 0.0),
        (330.0, 3.0, 25.0),
        (523.0, -1.0, -50.0),
        (150.0, 12.0, 50.0),
    ] {
        let x = sine(f, 0.5, n);
        let (l, r) = pitch_out(semi, cent, &x);
        let want = f * ratio(semi, cent);
        let c = peak_cents(&l[12_000..], want);
        assert!(c.abs() < 5.0, "{f} Hz {semi}/{cent}: {c:.2} cents");
        assert_eq!(l, r, "stereo-linked");
        // splices are phase-coherent: the wet level stays close to the input level
        let rms = (l[12_000..].iter().map(|v| v * v).sum::<f32>() / (n - 12_000) as f32).sqrt();
        assert!((rms - 0.3536).abs() < 0.06, "{f} {semi}: rms {rms}");
    }
}

#[test]
fn pitch_is_accurate_on_harmonic_tones() {
    let n = 60_000;
    for (f, semi, cent) in [(196.0, -5.0, 0.0), (196.0, 4.0, 25.0), (110.0, 12.0, 0.0)] {
        let x = tone(f, 0.5, n);
        let (l, _) = pitch_out(semi, cent, &x);
        let want = f * ratio(semi, cent);
        let c = peak_cents(&l[12_000..], want);
        assert!(c.abs() < 5.0, "{f} Hz {semi}/{cent}: {c:.2} cents");
        // 2nd harmonic too
        let c2 = peak_cents(&l[12_000..], 2.0 * want);
        assert!(c2.abs() < 5.0, "{f} Hz h2 {semi}/{cent}: {c2:.2} cents");
    }
}

#[test]
fn pitch_unison_is_a_pure_delay_and_mix_blends() {
    let mut m = StereoPitch::new();
    m.prepare(FS, 64);
    m.set_params(&PitchParams {
        semi: 0.0,
        cent: 0.0,
        dly_ms: 20.0,
        lc_hz: 20.0,
        hc_hz: 20_000.0,
        mix_pct: 100.0,
    });
    assert_eq!(m.wet_delay_range().0, 960.0);
    let mut l = vec![0.0f32; 4096];
    l[100] = 1.0;
    let mut r = l.clone();
    run(&mut m, &mut l, &mut r);
    let peak = l
        .iter()
        .enumerate()
        .fold(
            (0, 0.0f32),
            |b, (i, &v)| if v.abs() > b.1 { (i, v.abs()) } else { b },
        )
        .0;
    assert_eq!(peak, 1060);
    // mix 0 = dry
    m.load_defaults();
    m.set_param(pitch_slot::MIX, ParamValue::Float(0.0));
    m.set_param(pitch_slot::SEMI, ParamValue::Int(5));
    m.reset();
    let x = sine(300.0, 0.5, 2048);
    let (mut l, mut r) = (x.clone(), x.clone());
    run(&mut m, &mut l, &mut r);
    assert_eq!(l, x);
}

#[test]
fn pitch_wet_delay_stays_in_window_and_delay_changes_splice() {
    let mut m = StereoPitch::new();
    m.prepare(FS, 64);
    m.set_param(pitch_slot::SEMI, ParamValue::Int(7));
    m.set_param(pitch_slot::DLY, ParamValue::Float(50.0));
    m.commit();
    let x = tone(180.0, 0.4, 2048);
    for k in 0..60 {
        if k == 30 {
            m.set_param(pitch_slot::DLY, ParamValue::Float(5.0));
        }
        let (mut l, mut r) = (x.clone(), x.clone());
        run(&mut m, &mut l, &mut r);
        assert!(l.iter().all(|v| v.is_finite() && v.abs() < 2.0));
        let (lo, hi) = m.wet_delay_range();
        if k != 30 {
            let d = m.head_delay();
            assert!(d >= lo - 1.0 && d <= hi + 1.0, "{k}: {d} not in {lo}..{hi}");
        }
    }
    let (lo, hi) = m.wet_delay_range();
    assert_eq!(lo, 240.0);
    assert!(hi - lo < 0.033 * FS, "window {} ms", (hi - lo) / 48.0);
}

// ----- D-PITCH -----------------------------------------------------------------------------------------

#[test]
fn dual_pitch_voices_pan_and_shift_independently() {
    let mut m = DualPitch::new();
    m.prepare(FS, 64);
    let mut p = DualPitchParams::default();
    p.v[0] = VoiceParams {
        semi: 7.0,
        cent: 0.0,
        dly_ms: 0.0,
        pan: -100.0,
        lvl_db: 0.0,
    };
    p.v[1] = VoiceParams {
        semi: -12.0,
        cent: 0.0,
        dly_ms: 30.0,
        pan: 100.0,
        lvl_db: -6.0,
    };
    m.set_params(&p);
    let n = 60_000;
    let x = sine(392.0, 0.5, n);
    let (mut l, mut r) = (x.clone(), x.clone());
    run(&mut m, &mut l, &mut r);
    let cl = peak_cents(&l[12_000..], 392.0 * ratio(7.0, 0.0));
    let cr = peak_cents(&r[12_000..], 196.0);
    assert!(cl.abs() < 5.0 && cr.abs() < 5.0, "{cl} {cr}");
    // hard pans: no cross-talk
    assert!(dft_mag(&l[12_000..], 196.0) < 1e-3);
    assert!(dft_mag(&r[12_000..], 392.0 * ratio(7.0, 0.0)) < 1e-3);
    // level: -6 dB voice
    let al = dft_mag(&l[12_000..], 392.0 * ratio(7.0, 0.0));
    let ar = dft_mag(&r[12_000..], 196.0);
    let db = 20.0 * (ar / al).log10();
    assert!((db + 6.0).abs() < 1.5, "{db}");
    // -oo level mutes
    p.v[0].lvl_db = -144.0;
    p.v[1].lvl_db = -144.0;
    m.set_params(&p);
    m.reset();
    let (mut l, mut r) = (x[..4096].to_vec(), x[..4096].to_vec());
    run(&mut m, &mut l, &mut r);
    assert!(l.iter().chain(&r).all(|v| v.abs() < 1e-6));
}

// ----- DOUBLE ------------------------------------------------------------------------------------------

#[test]
fn double_modes_stay_near_pitch_and_spread_widens() {
    let n = 48_000;
    let x = tone(220.0, 0.4, n);
    for mode in 0..5 {
        let mut widths = [0.0f32; 2];
        for (i, spread) in [0.0f32, 100.0].into_iter().enumerate() {
            let mut m = DoubleVocal::new();
            m.prepare(FS, 64);
            m.set_params(&DoubleParams {
                mode: mode as f32,
                mix_pct: 100.0,
                spread_pct: spread,
            });
            let (mut l, mut r) = (x.clone(), x.clone());
            run(&mut m, &mut l, &mut r);
            assert!(l.iter().chain(&r).all(|v| v.is_finite() && v.abs() < 2.0));
            let s = &l[9_600..];
            let rms = (s.iter().map(|v| v * v).sum::<f32>() / s.len() as f32).sqrt();
            assert!(rms > 0.1, "mode {mode}: rms {rms}");
            // the doubled voices stay within the detune range of the input pitch
            let c = peak_cents(s, 220.0);
            assert!(c.abs() < 25.0, "mode {mode}: {c}");
            let side: f32 = l[9_600..]
                .iter()
                .zip(&r[9_600..])
                .map(|(a, b)| (a - b) * (a - b))
                .sum();
            widths[i] = side;
        }
        assert!(widths[0] < 1e-6, "mode {mode}: spread 0 is mono");
        assert!(widths[1] > 1.0, "mode {mode}: spread 100 is wide");
    }
}

// ----- PCORR -------------------------------------------------------------------------------------------

fn pcorr(p: PcorrParams) -> PitchFix {
    let mut m = PitchFix::new();
    m.prepare(FS, 64);
    m.set_params(&p);
    m.commit();
    m
}

fn hard(sw: [f32; 12]) -> PcorrParams {
    PcorrParams {
        speed: 100.0,
        amount: 100.0,
        a4_hz: 440.0,
        sw,
    }
}

const C_MAJOR: [f32; 12] = [1., 0., 1., 0., 1., 1., 0., 1., 0., 1., 0., 1.];

fn pcorr_peak(m: &mut PitchFix, x: &[f32], want: f64) -> f64 {
    let (mut l, mut r) = (x.to_vec(), x.to_vec());
    run(m, &mut l, &mut r);
    peak_cents(&l[12_000..], want)
}

#[test]
fn pcorr_detects_pitch_accurately() {
    for f in [82.0, 110.0, 261.6, 450.0, 880.0] {
        let mut m = pcorr(PcorrParams {
            amount: 0.0,
            ..PcorrParams::default()
        });
        let x = tone(f, 0.3, 9600);
        let (mut l, mut r) = (x.clone(), x);
        run(&mut m, &mut l, &mut r);
        let got = m.detected_hz().unwrap();
        let c = 1200.0 * (got / f).log2();
        assert!(c.abs() < 1.0, "{f}: detected {got} ({c} cents)");
    }
}

#[test]
fn pcorr_snaps_to_the_enabled_scale() {
    let n = 60_000;
    // 455 Hz = A4 + 58 cents: chromatic -> Bb4, C major -> A4
    let x = sine(455.0, 0.4, n);
    let mut m = pcorr(hard([0.0; 12]));
    let c = pcorr_peak(&mut m, &x, 466.164);
    assert!(c.abs() < 5.0, "chromatic: {c}");
    assert_eq!(m.target_note(), Some(70));
    let mut m = pcorr(hard(C_MAJOR));
    let c = pcorr_peak(&mut m, &x, 440.0);
    assert!(c.abs() < 5.0, "C major: {c}");
    assert_eq!(m.target_note(), Some(69));
    // only C enabled: 430 Hz -> C5
    let mut sw = [0.0; 12];
    sw[0] = 1.0;
    let mut m = pcorr(hard(sw));
    let c = pcorr_peak(&mut m, &sine(430.0, 0.4, n), 523.251);
    assert!(c.abs() < 5.0, "C only: {c}");
    // A4 = 432 reference: 438 Hz (+24 cents) -> 432
    let mut p = hard([0.0; 12]);
    p.a4_hz = 432.0;
    let mut m = pcorr(p);
    let c = pcorr_peak(&mut m, &sine(438.0, 0.4, n), 432.0);
    assert!(c.abs() < 5.0, "A4 432: {c}");
    // harmonic tone: fundamental and 3rd harmonic land on the note
    let mut m = pcorr(hard(C_MAJOR));
    let x = tone(203.0, 0.4, n); // G3 (196) + 61 cents -> G3 is in C major, Ab not
    let (mut l, mut r) = (x.clone(), x);
    run(&mut m, &mut l, &mut r);
    let c1 = peak_cents(&l[12_000..], 195.998);
    let c3 = peak_cents(&l[12_000..], 3.0 * 195.998);
    assert!(c1.abs() < 5.0 && c3.abs() < 5.0, "{c1} {c3}");
}

#[test]
fn pcorr_amount_and_speed() {
    let n = 60_000;
    // 450 Hz is 38.9 cents above A4; amount 50 corrects half of it
    let x = sine(450.0, 0.4, n);
    let mut p = hard([0.0; 12]);
    p.amount = 50.0;
    let mut m = pcorr(p);
    let half = 450.0 * 2f64.powf(-0.5 * 1200.0 * (450.0f64 / 440.0).log2() / 1200.0);
    let c = pcorr_peak(&mut m, &x, half);
    assert!(c.abs() < 5.0, "amount 50: {c}");
    // speed: time constants
    assert!((pcorr_retune_ms(100.0) - 1.0).abs() < 1e-9);
    assert!((pcorr_retune_ms(1.0) - 400.0).abs() < 1e-6);
    let mut fast = pcorr(hard([0.0; 12]));
    let mut p = hard([0.0; 12]);
    p.speed = 1.0;
    let mut slow = pcorr(p);
    let x = sine(450.0, 0.4, 9600);
    for m in [&mut fast, &mut slow] {
        let (mut l, mut r) = (x.clone(), x.clone());
        run(m, &mut l, &mut r);
    }
    assert!(
        (fast.correction_cents() + 38.9).abs() < 1.0,
        "{}",
        fast.correction_cents()
    );
    assert!(
        slow.correction_cents().abs() < 25.0,
        "{}",
        slow.correction_cents()
    );
}

#[test]
fn pcorr_latency_is_exact_without_correction() {
    // amount 0: the output is the input delayed by exactly latency() samples
    let mut m = pcorr(PcorrParams {
        amount: 0.0,
        ..PcorrParams::default()
    });
    let lat = m.latency();
    let mut seed = 1u32;
    let x: Vec<f32> = (0..20_000)
        .map(|_| {
            seed ^= seed << 13;
            seed ^= seed >> 17;
            seed ^= seed << 5;
            (seed as f32 / u32::MAX as f32 - 0.5) * 0.5
        })
        .chain(tone(200.0, 0.3, 20_000))
        .collect();
    let (mut l, mut r) = (x.clone(), x.clone());
    run(&mut m, &mut l, &mut r);
    for i in lat..x.len() {
        assert_eq!(l[i], x[i - lat], "{i}");
    }
    // with correction the head stays centred on the latency on average
    let mut m = pcorr(hard([0.0; 12]));
    let x = tone(150.0, 0.3, 240_000);
    let mut sum = 0.0;
    let mut cnt = 0.0;
    for (k, c) in x.chunks(64).enumerate() {
        let (mut l, mut r) = (c.to_vec(), c.to_vec());
        m.process(&mut l, &mut r, None);
        if k > 100 {
            sum += m.sh.delay();
            cnt += 1.0;
            assert!(m.sh.delay() > 0.0 && m.sh.delay() < 2.0 * lat as f64);
        }
    }
    let mean = sum / cnt;
    assert!(
        (mean - lat as f64).abs() < 0.1 * lat as f64,
        "mean head delay {mean}"
    );
}

#[test]
fn pcorr_silence_and_noise_are_unvoiced() {
    let mut m = pcorr(hard([0.0; 12]));
    let mut l = vec![0.0f32; 9600];
    let mut r = l.clone();
    run(&mut m, &mut l, &mut r);
    assert!(m.detected_hz().is_none());
    assert!(l.iter().all(|&v| v == 0.0));
}

// ----- SUB ---------------------------------------------------------------------------------------------

fn sub_out(range: f32, oct1: f32, oct2: f32, x: &[f32]) -> Vec<f32> {
    let mut m = SubOctaver::new();
    m.prepare(FS, 64);
    m.set_params(&SubParams {
        range,
        oct1_pct: oct1,
        oct2_pct: oct2,
    });
    let (mut l, mut r) = (x.to_vec(), x.to_vec());
    run(&mut m, &mut l, &mut r);
    assert_eq!(l, r);
    l
}

#[test]
fn sub_tracks_the_fundamental() {
    let n = 48_000;
    for (f, range) in [(110.0, 0.0), (82.4, 0.0), (196.0, 1.0), (330.0, 2.0)] {
        let x = tone(f, 0.4, n);
        let y1 = sub_out(range, 100.0, 0.0, &x);
        let s = &y1[9_600..];
        let a1 = dft_mag(s, f / 2.0);
        let a2 = dft_mag(s, f / 4.0);
        assert!(a1 > 0.1, "{f}: oct1 level {a1}");
        assert!(a2 < 0.2 * a1, "{f}: oct2 leak {a2} vs {a1}");
        let c = peak_cents(s, f / 2.0);
        assert!(c.abs() < 5.0, "{f}: sub1 at {c} cents");
        // dry is preserved
        assert!((dft_mag(s, f) - dft_mag(&x[9_600..], f)).abs() < 0.05);
        let y2 = sub_out(range, 0.0, 100.0, &x);
        let s = &y2[9_600..];
        let b2 = dft_mag(s, f / 4.0);
        assert!(b2 > 0.05, "{f}: oct2 level {b2}");
        assert!(peak_cents(s, f / 4.0).abs() < 5.0);
        // octaves off: dry only
        let y0 = sub_out(range, 0.0, 0.0, &x);
        assert_eq!(y0, x);
    }
    // follows a pitch change
    let mut x = tone(110.0, 0.4, 24_000);
    x.extend(tone(146.8, 0.4, 24_000));
    let y = sub_out(0.0, 100.0, 0.0, &x);
    assert!(peak_cents(&y[28_000..], 73.4).abs() < 5.0);
    // silence stays silent (self-gating)
    let y = sub_out(0.0, 100.0, 100.0, &[0.0; 4096]);
    assert!(y.iter().all(|v| v.abs() < 1e-9));
}

// ----- robustness --------------------------------------------------------------------------------------

#[test]
fn extreme_settings_stay_finite_and_bounded() {
    let x = tone(70.0, 0.9, 4096);
    let y = sine(3000.0, 0.9, 4096);
    for id in IDS {
        let mut m = create_model(BlockKind::Fx, id);
        m.prepare(FS, 64);
        for k in 0..24u16 {
            for slot in 1..=15u16 {
                let v = match (k + slot) % 4 {
                    0 => -144.0,
                    1 => 500.0,
                    2 => -12.0,
                    _ => 12.0,
                };
                m.set_param(slot, ParamValue::Float(v));
            }
            let src = if k % 2 == 0 { &x } else { &y };
            let (mut l, mut r) = (src.clone(), src.clone());
            run(&mut *m, &mut l, &mut r);
            assert!(
                l.iter().chain(&r).all(|v| v.is_finite() && v.abs() < 8.0),
                "{id} {k}"
            );
        }
        m.reset();
        m.load_defaults();
        let mut l = vec![0.0f32; 256];
        let mut r = l.clone();
        run(&mut *m, &mut l, &mut r);
        assert!(l.iter().all(|v| v.abs() < 1e-6), "{id} reset clears state");
    }
}
