use super::*;
use crate::model::{create_model, BlockKind};

const FS: f64 = 48_000.0;
const IDS: [&str; 7] = [
    "DIMCRS", "CHORUS", "FLANGER", "PHASER", "ROTARY", "PANNER", "MOOD",
];

fn sine(f: f64, amp: f32, n: usize) -> Vec<f32> {
    (0..n)
        .map(|i| amp * (2.0 * core::f64::consts::PI * f * i as f64 / FS).sin() as f32)
        .collect()
}

fn run(m: &mut dyn Model, l: &mut [f32], r: &mut [f32]) {
    for (a, b) in l.chunks_mut(64).zip(r.chunks_mut(64)) {
        m.process(a, b, None);
    }
}

fn rms(x: &[f32]) -> f32 {
    (x.iter().map(|v| v * v).sum::<f32>() / x.len() as f32).sqrt()
}

/// Mean frequency (Hz) of the rising crossings of `x` through `level`.
fn crossing_rate(x: &[f32], level: f32) -> f64 {
    let mut first = None;
    let mut last = 0.0;
    let mut count = 0usize;
    for i in 1..x.len() {
        if x[i - 1] < level && x[i] >= level {
            let t = (i - 1) as f64 + ((level - x[i - 1]) / (x[i] - x[i - 1])) as f64;
            if first.is_none() {
                first = Some(t);
            } else {
                count += 1;
            }
            last = t;
        }
    }
    count as f64 * FS / (last - first.unwrap())
}

// ----- shared blocks -------------------------------------------------------------------------------------

#[test]
fn lfo_rate_is_exact_and_shapes_are_bounded() {
    let mut lfo = Lfo::default();
    lfo.set_rate(0.83, FS);
    let n = (100.5 * FS) as usize;
    let mut wraps = 0;
    let mut prev = 0.0;
    for _ in 0..n {
        let p = lfo.tick();
        if p < prev {
            wraps += 1;
        }
        prev = p;
    }
    assert_eq!(wraps, 83);
    assert!((lfo.phase() - 0.415).abs() < 1e-6, "{}", lfo.phase());
    for i in 0..1000 {
        let u = i as f32 / 1000.0;
        for v in [
            wave::sine(u),
            wave::tri(u),
            wave::skew(u, 0.0),
            wave::skew(u, 0.25),
            wave::skew(u, 1.0),
        ] {
            assert!((-1.0..=1.0).contains(&v), "{u} {v}");
        }
    }
    assert!((wave::tri(0.25) - 1.0).abs() < 1e-6 && wave::tri(0.0).abs() < 1e-6);
    assert!((wave::skew(0.25, 0.25) - 1.0).abs() < 1e-5);
    assert!(wave::skew(0.998, 1.0) > 0.99, "rising saw ends high");
}

#[test]
fn ahr_envelope_attacks_holds_releases() {
    let mut e = AhrEnvelope::default();
    e.set_times(1.0, 100.0, 10.0, FS);
    for _ in 0..480 {
        e.tick(0.5);
    }
    assert!((e.value() - 0.5).abs() < 0.01 && (e.normalized() - 1.0).abs() < 0.02);
    for _ in 0..(0.09 * FS) as usize {
        e.tick(0.0);
    }
    assert!(e.value() > 0.49, "held {}", e.value());
    for _ in 0..(0.06 * FS) as usize {
        e.tick(0.0);
    }
    assert!(e.value() < 0.5 * (-4.0f32).exp(), "released {}", e.value());
}

// ----- registry / parameters ---------------------------------------------------------------------------

/// Parse a schema default ("11k5", "0.48", "-10").
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
            // round trip
            m.set_param(slot, ParamValue::Float(want));
            assert_eq!(m.param(slot), Some(want), "{id} {}", p["name"]);
        }
        assert_eq!(m.param(params.len() as u16 + 1), None, "{id}");
        assert_eq!(m.latency(), 0);
        assert_eq!(m.id(), id);
    }
}

// ----- DIMCRS ------------------------------------------------------------------------------------------

#[test]
fn dimension_is_mono_compatible_and_buttons_set_rate_and_depth() {
    let mut m = DimensionChorus::new();
    m.prepare(FS, 64);
    let n = 48_000;
    let xl = sine(440.0, 0.4, n);
    let xr = sine(660.0, 0.3, n);
    let (mut l, mut r) = (xl.clone(), xr.clone());
    run(&mut m, &mut l, &mut r);
    let mut diff = 0.0f32;
    for i in 0..n {
        diff = diff.max(((l[i] + r[i]) - (xl[i] + xr[i])).abs());
    }
    assert!(diff < 1e-5, "mono sum = dry sum (wet cancels): {diff}");
    let wet: Vec<f32> = (0..n).map(|i| l[i] - xl[i]).collect();
    assert!(
        rms(&wet[4800..]) > 0.02,
        "wet present {}",
        rms(&wet[4800..])
    );

    // button table
    let mut p = DimensionParams {
        sw: [false; 4],
        ..Default::default()
    };
    assert_eq!(p.modulation(), None);
    p.sw[3] = true;
    assert_eq!(p.modulation(), Some((0.5, 1.2)));
    p.sw = [true, false, false, false];
    assert_eq!(p.modulation(), Some((0.25, 0.4)));
    p.sw = [true, true, true, true];
    assert_eq!(p.modulation(), Some((0.5, DIM_MAX_SWING_MS)));
    let depths: Vec<f32> = DIM_BUTTONS.iter().map(|b| b.1).collect();
    assert!(depths.windows(2).all(|w| w[0] <= w[1]), "depth rises 1..4");

    // no button: exact bypass
    m.set_params(&DimensionParams {
        sw: [false; 4],
        ..Default::default()
    });
    m.reset();
    let (mut l, mut r) = (xl.clone(), xr.clone());
    run(&mut m, &mut l, &mut r);
    assert!(l.iter().zip(&xl).all(|(a, b)| (a - b).abs() < 1e-6));

    // DRY off: wet only; MONO input feeds both BBDs the same signal
    m.set_params(&DimensionParams {
        sw: [false, false, false, true],
        stereo_in: false,
        dry: false,
    });
    m.reset();
    assert!((m.lfo_rate() - 0.5).abs() < 1e-9);
    let (mut l, mut r) = (xl.clone(), xl.clone());
    run(&mut m, &mut l, &mut r);
    assert!(rms(&l[4800..]) > 0.01 && rms(&l[4800..]) < 0.3);
    assert!(
        l.iter().zip(&r).all(|(a, b)| (a + b).abs() < 1e-5),
        "L = -R"
    );
}

// ----- CHORUS / FLANGER --------------------------------------------------------------------------------

#[test]
fn chorus_delay_swing_rate_and_phase() {
    let mut m = ModDelay::chorus();
    m.prepare(FS, 64);
    let n = (6.0 * FS) as usize;
    let (mut dl, mut dr) = (Vec::with_capacity(n), Vec::with_capacity(n));
    let (mut a, mut b) = ([0.1f32], [0.1f32]);
    for _ in 0..n {
        m.process(&mut a, &mut b, None);
        let (x, y) = m.current_delays();
        dl.push(x);
        dr.push(y);
    }
    // DELAY L 15.2 ms, DEPTH 20 %: 729.6 samples +/- 10 %
    let base = 15.2 * 48.0;
    let (lo, hi) = dl
        .iter()
        .fold((f32::MAX, f32::MIN), |(a, b), &v| (a.min(v), b.max(v)));
    assert!((lo / (base * 0.9) - 1.0).abs() < 1e-3, "{lo}");
    assert!((hi / (base * 1.1) - 1.0).abs() < 1e-3, "{hi}");
    let rate = crossing_rate(&dl, base);
    assert!((rate / 0.48 - 1.0).abs() < 2e-3, "rate {rate}");
    // R: own base 16.6 ms, LFO 120 deg later
    let rb = 16.6 * 48.0;
    let first_peak = |d: &[f32], b: f32| {
        d.iter()
            .position(|&v| v >= b * 1.0999)
            .map(|i| i as f64 / FS * 0.48)
            .unwrap()
    };
    // R LFO leads L by PHASE
    let lag = wrap(first_peak(&dl, base) - first_peak(&dr, rb));
    assert!((lag - 120.0 / 360.0).abs() < 0.01, "phase {lag}");
}

#[test]
fn chorus_spread_zero_is_mono_wet() {
    let mut m = ModDelay::chorus();
    let mut p = ModDelayParams::chorus();
    p.spread_pct = 0.0;
    p.mix_pct = 100.0;
    m.set_params(&p);
    m.prepare(FS, 64);
    let n = 24_000;
    let (mut l, mut r) = (sine(300.0, 0.5, n), sine(500.0, 0.5, n));
    run(&mut m, &mut l, &mut r);
    let d: f32 = l[4800..]
        .iter()
        .zip(&r[4800..])
        .map(|(a, b)| (a - b).abs())
        .fold(0.0, f32::max);
    assert!(d < 1e-5, "{d}");
}

/// Steady-state RMS gain of a sine through `m`.
fn sine_gain(m: &mut dyn Model, f: f64) -> f32 {
    m.reset();
    let n = 24_000;
    let x = sine(f, 0.25, n);
    let (mut l, mut r) = (x.clone(), x.clone());
    run(m, &mut l, &mut r);
    rms(&l[12_000..]) / rms(&x[12_000..])
}

#[test]
fn flanger_comb_notches_and_feedback_peaks() {
    let mut m = ModDelay::flanger();
    let mut p = ModDelayParams::flanger();
    p.depth_pct = [0.0; 2];
    p.dly_ms = [1.0; 2];
    p.feed_pct = 0.0;
    p.lc_hz = 20.0;
    p.hc_hz = 20_000.0;
    p.flc_hz = 20.0;
    p.fhc_hz = 20_000.0;
    m.set_params(&p);
    m.prepare(FS, 64);
    // 1 ms delay + dry at 50 %: notches at 500, 1500 Hz, peaks at 1000 Hz
    let notch = sine_gain(&mut m, 500.0);
    let peak = sine_gain(&mut m, 1000.0);
    assert!(notch < 0.06, "notch {notch}");
    assert!((peak - 1.0).abs() < 0.03, "peak {peak}");
    p.feed_pct = 70.0;
    m.set_params(&p);
    let fb_peak = sine_gain(&mut m, 1000.0);
    assert!(
        fb_peak > 1.8,
        "positive feedback resonates at 1/d: {fb_peak}"
    );
    // depth sweeps the delay
    p.depth_pct = [100.0; 2];
    p.feed_pct = 0.0;
    m.set_params(&p);
    m.reset();
    let (mut lo, mut hi) = (f32::MAX, 0.0f32);
    let (mut a, mut b) = ([0.0f32], [0.0f32]);
    for _ in 0..(3.2 * FS) as usize {
        m.process(&mut a, &mut b, None);
        lo = lo.min(m.current_delays().0);
        hi = hi.max(m.current_delays().0);
    }
    assert!(
        (lo - 4.8).abs() < 0.05 && (hi - 91.2).abs() < 0.05,
        "{lo} {hi}"
    );
}

// ----- PHASER ------------------------------------------------------------------------------------------

fn impulse_response(m: &mut dyn Model, n: usize) -> Vec<f32> {
    m.reset();
    let (mut l, mut r) = (vec![0.0f32; n], vec![0.0f32; n]);
    l[0] = 1.0;
    r[0] = 1.0;
    run(m, &mut l, &mut r);
    l
}

fn mag_db(h: &[f32], f: f64) -> f64 {
    let w = 2.0 * core::f64::consts::PI * f / FS;
    let (mut re, mut im) = (0.0f64, 0.0f64);
    for (i, &v) in h.iter().enumerate() {
        re += v as f64 * (w * i as f64).cos();
        im -= v as f64 * (w * i as f64).sin();
    }
    10.0 * (re * re + im * im).max(1e-30).log10()
}

fn count_notches(h: &[f32]) -> usize {
    let mags: Vec<f64> = (0..300)
        .map(|i| mag_db(h, 20.0 * (1000f64).powf(i as f64 / 299.0)))
        .collect();
    (1..mags.len() - 1)
        .filter(|&i| mags[i] < mags[i - 1] && mags[i] <= mags[i + 1] && mags[i] < -6.0)
        .count()
}

#[test]
fn phaser_notch_count_follows_stages_and_reso_peaks() {
    let mut m = Phaser::new();
    let static_p = |stages: u32, reso: f32| PhaserParams {
        depth_pct: 0.0,
        range_pct: 50.0,
        stages,
        reso_pct: reso,
        ..Default::default()
    };
    m.prepare(FS, 64);
    for (stg, want) in [(2, 1), (4, 2), (6, 3), (8, 4), (12, 6)] {
        m.set_params(&static_p(stg, 0.0));
        let h = impulse_response(&mut m, 8192);
        assert_eq!(count_notches(&h), want, "{stg} stages");
    }
    // resonance: feedback raises the peaks between notches
    m.set_params(&static_p(4, 0.0));
    let h0 = impulse_response(&mut m, 16_384);
    m.set_params(&static_p(4, 80.0));
    let h1 = impulse_response(&mut m, 16_384);
    let peak = |h: &[f32]| {
        (0..200)
            .map(|i| mag_db(h, 20.0 * (1000f64).powf(i as f64 / 199.0)))
            .fold(f64::MIN, f64::max)
    };
    assert!(peak(&h1) > peak(&h0) + 6.0, "{} {}", peak(&h1), peak(&h0));
}

#[test]
fn phaser_sweep_depth_rate_and_envelope() {
    let mut m = Phaser::new();
    m.set_params(&PhaserParams {
        depth_pct: 100.0,
        range_pct: 50.0,
        wave: 0.0,
        speed_hz: 2.0,
        ..Default::default()
    });
    m.prepare(FS, 64);
    let n = (5.0 * FS) as usize;
    let mut fl = Vec::with_capacity(n / 16);
    let (mut a, mut b) = ([0.0f32; 16], [0.0f32; 16]);
    for _ in 0..n / 16 {
        m.process(&mut a, &mut b, None);
        fl.push(m.current_freqs().0);
    }
    let (lo, hi) = fl
        .iter()
        .fold((f32::MAX, 0.0f32), |(a, b), &v| (a.min(v), b.max(v)));
    assert!(
        lo < PHASER_F_LO * 1.05 && hi > PHASER_F_HI * 0.95,
        "{lo} {hi}"
    );
    let centre = phaser_freq(0.5);
    let rate = crossing_rate(&fl, centre) / 16.0;
    assert!((rate / 2.0 - 1.0).abs() < 5e-3, "rate {rate}");

    // envelope: +100 % ENV MOD with a loud input raises the sweep
    m.set_params(&PhaserParams {
        depth_pct: 0.0,
        range_pct: 20.0,
        emod_pct: 100.0,
        ..Default::default()
    });
    m.reset();
    let quiet = {
        let (mut a, mut b) = ([0.0f32; 64], [0.0f32; 64]);
        m.process(&mut a, &mut b, None);
        m.current_freqs().0
    };
    let x = sine(200.0, 0.5, 9600);
    let (mut l, mut r) = (x.clone(), x);
    run(&mut m, &mut l, &mut r);
    let loud = m.current_freqs().0;
    assert!((quiet - phaser_freq(0.2)).abs() < 1.0, "{quiet}");
    assert!(loud > phaser_freq(0.6), "{loud}");
}

// ----- ROTARY ------------------------------------------------------------------------------------------

#[test]
fn rotary_rotors_ramp_with_inertia() {
    let mut m = Rotary::new();
    m.prepare(FS, 64);
    let (mut a, mut b) = ([0.0f32; 64], [0.0f32; 64]);
    m.process(&mut a, &mut b, None); // scene settings apply instantly to a fresh model
    let (h0, d0) = m.rotor_speeds();
    assert!((h0 - 0.68).abs() < 1e-3 && (d0 - 0.68 * DRUM_RATIO).abs() < 1e-3);
    m.set_param(rotary_slot::SW, ParamValue::Enum(2));
    let (th, td) = m.params().time_constants();
    let mut buf = vec![0.0f32; (th * FS) as usize];
    let mut buf2 = buf.clone();
    m.process(&mut buf, &mut buf2, None);
    let (h, d) = m.rotor_speeds();
    let frac_h = (h - 0.68) / (5.0 - 0.68);
    let frac_d = (d - d0) / (5.0 * DRUM_RATIO - d0);
    assert!((frac_h - 0.632).abs() < 0.01, "horn {frac_h}");
    assert!(
        (frac_d - (1.0 - (-th / td).exp())).abs() < 0.01,
        "drum slower: {frac_d}"
    );
    assert!(frac_d < 0.6 * frac_h);
    assert!(td > th);
    let mut l = vec![0.0f32; (20.0 * FS) as usize];
    let mut r = l.clone();
    run(&mut m, &mut l, &mut r);
    let (h, d) = m.rotor_speeds();
    assert!((h - 5.0).abs() < 1e-3 && (d - 4.25).abs() < 1e-2, "{h} {d}");
    // STOP: spins down, slower than it spun up
    m.set_param(rotary_slot::SW, ParamValue::Enum(0));
    let mut buf = vec![0.0f32; (th * FS) as usize];
    let mut buf2 = buf.clone();
    m.process(&mut buf, &mut buf2, None);
    let down = 1.0 - m.rotor_speeds().0 / 5.0;
    assert!(down > 0.3 && down < 0.6, "spin-down {down}");
}

#[test]
fn rotary_horn_am_rate_and_doppler_depth() {
    let mut m = Rotary::new();
    m.set_params(&RotaryParams {
        speed: RotorSpeed::Fast,
        balance: 100.0,
        mix_pct: 100.0,
        distance: 0.0,
        ..Default::default()
    });
    m.prepare(FS, 64);
    let n = (4.0 * FS) as usize;
    let x = sine(4000.0, 0.5, n);
    let (mut l, mut r) = (x.clone(), x);
    run(&mut m, &mut l, &mut r);
    // AM: 1 ms RMS envelope, autocorrelation peak at the 5 Hz horn period
    let env: Vec<f32> = l[9600..].chunks(48).map(rms).collect();
    let mean = env.iter().sum::<f32>() / env.len() as f32;
    let e: Vec<f32> = env.iter().map(|v| v - mean).collect();
    let ac = |lag: usize| -> f32 { e.iter().zip(&e[lag..]).map(|(a, b)| a * b).sum() };
    let best = (100..400).max_by(|&a, &b| ac(a).total_cmp(&ac(b))).unwrap();
    assert!((best as f32 - 200.0).abs() <= 2.0, "AM period {best} ms");
    let (emin, emax) = env
        .iter()
        .fold((f32::MAX, 0.0f32), |(a, b), &v| (a.min(v), b.max(v)));
    assert!(emin / emax < 0.4, "deep AM close-miked: {}", emin / emax);
    // Doppler: instantaneous frequency deviation ~ +/- 2 pi f_rot r / c = +/- 1.37 %
    let s = &l[9600..];
    let mut ups = Vec::new();
    for i in 1..s.len() {
        if s[i - 1] < 0.0 && s[i] >= 0.0 {
            ups.push((i - 1) as f64 + (-s[i - 1] / (s[i] - s[i - 1])) as f64);
        }
    }
    let freqs: Vec<f64> = ups.windows(9).map(|w| 8.0 * FS / (w[8] - w[0])).collect();
    let (fmin, fmax) = freqs
        .iter()
        .fold((f64::MAX, 0.0f64), |(a, b), &v| (a.min(v), b.max(v)));
    let dev = (fmax - fmin) / 4000.0;
    assert!(dev > 0.018 && dev < 0.04, "Doppler p-p {dev}");
    // stereo: the opposite mic hears the opposite phase of the rotation
    let d: f32 = l.iter().zip(&r).map(|(a, b)| (a - b).abs()).sum::<f32>() / n as f32;
    assert!(d > 0.05, "{d}");
}

// ----- PANNER ------------------------------------------------------------------------------------------

#[test]
fn panner_rate_depth_and_autopan() {
    let mut m = Panner::new();
    m.set_params(&PannerParams {
        phase_deg: 0.0,
        depth_pct: 50.0,
        ..Default::default()
    });
    m.prepare(FS, 64);
    let n = (20.0 * FS) as usize;
    let (mut l, mut r) = (vec![0.5f32; n], vec![0.5f32; n]);
    run(&mut m, &mut l, &mut r);
    let g: Vec<f32> = l.iter().map(|v| v / 0.5).collect();
    let (lo, hi) = g[4800..]
        .iter()
        .fold((f32::MAX, 0.0f32), |(a, b), &v| (a.min(v), b.max(v)));
    assert!(
        (lo - 0.5).abs() < 0.01 && (hi - 1.0).abs() < 0.01,
        "{lo} {hi}"
    );
    let rate = crossing_rate(&g, 0.75);
    assert!((rate / 1.44 - 1.0).abs() < 1e-3, "rate {rate}");
    assert!(l.iter().zip(&r).all(|(a, b)| a == b), "phase 0 = tremolo");

    // 180 deg, depth 100: triangle auto-pan, L + R gain = 1
    m.set_params(&PannerParams::default());
    m.reset();
    let (mut l, mut r) = (vec![0.5f32; 48_000], vec![0.5f32; 48_000]);
    run(&mut m, &mut l, &mut r);
    for i in 480..48_000 {
        assert!(((l[i] + r[i]) / 0.5 - 1.0).abs() < 0.01, "{i}");
    }
    let lmin = l.iter().fold(1.0f32, |a, &v| a.min(v));
    assert!(lmin < 0.01);
}

#[test]
fn panner_envelope_drives_speed_and_depth() {
    let mut m = Panner::new();
    m.set_params(&PannerParams {
        phase_deg: 0.0,
        env_speed_pct: 100.0,
        ..Default::default()
    });
    m.prepare(FS, 64);
    let n = (10.0 * FS) as usize;
    let (mut l, mut r) = (vec![0.5f32; n], vec![0.5f32; n]);
    run(&mut m, &mut l, &mut r);
    let g: Vec<f32> = l[4800..].iter().map(|v| v / 0.5).collect();
    let rate = crossing_rate(&g, 0.5);
    let want = 1.44 * (1.0 + PANNER_ENV_SPEED as f64);
    assert!((rate / want - 1.0).abs() < 0.01, "{rate} vs {want}");

    // ENV>DEPTH 100 %: a quiet input (env 0.1 of full) only gets 10 % depth
    m.set_params(&PannerParams {
        phase_deg: 0.0,
        env_depth_pct: 100.0,
        ..Default::default()
    });
    m.reset();
    let (mut l, mut r) = (vec![0.05f32; n], vec![0.05f32; n]);
    run(&mut m, &mut l, &mut r);
    let lo = l[4800..].iter().fold(1.0f32, |a, &v| a.min(v)) / 0.05;
    assert!((lo - 0.9).abs() < 0.01, "{lo}");
}

// ----- MOOD --------------------------------------------------------------------------------------------

fn mood_static(ftype: MoodType, slope24: bool, base: f32) -> MoodParams {
    MoodParams {
        base_hz: base,
        ftype,
        slope24,
        lfo_pct: 0.0,
        env_pct: 0.0,
        ..Default::default()
    }
}

#[test]
fn mood_filter_types_and_slopes() {
    let mut m = MoodFilter::new();
    m.prepare(FS, 64);
    // LP at 500 Hz, Q 0.5 per stage: two octaves up ~ -24 dB per 12 dB stage
    // Q 0.5 stage two octaves up: |1 / (1 - 16 + 8j)| = -24.6 dB; 24 dB: + Butterworth stage (-24.1 dB)
    for (slope24, want) in [(false, -24.6), (true, -48.7)] {
        m.set_params(&mood_static(MoodType::Lp, slope24, 500.0));
        let pass = 20.0 * sine_gain(&mut m, 60.0).log10();
        let stop = 20.0 * sine_gain(&mut m, 2000.0).log10();
        assert!(pass.abs() < 1.0, "{pass}");
        assert!((stop - want).abs() < 1.0, "slope24={slope24}: {stop}");
    }
    m.set_params(&mood_static(MoodType::Hp, true, 1000.0));
    assert!(20.0 * sine_gain(&mut m, 8000.0).log10() > -1.0);
    assert!(20.0 * sine_gain(&mut m, 125.0).log10() < -60.0);
    m.set_params(&MoodParams {
        reso: 10.0,
        ..mood_static(MoodType::Bp, false, 1000.0)
    });
    assert!(
        (sine_gain(&mut m, 1000.0) - 1.0).abs() < 0.05,
        "BP 0 dB peak"
    );
    assert!(sine_gain(&mut m, 2000.0) < 0.1);
    m.set_params(&MoodParams {
        reso: 5.0,
        ..mood_static(MoodType::Notch, false, 1000.0)
    });
    assert!(sine_gain(&mut m, 1000.0) < 0.02);
    assert!(sine_gain(&mut m, 4000.0) > 0.9);
}

#[test]
fn mood_lfo_and_envelope_sweep_the_cutoff() {
    let mut m = MoodFilter::new();
    m.prepare(FS, 64); // defaults: BASE 98 Hz, LFO 100 % sine at 1 Hz
    let n = (4.0 * FS) as usize;
    let mut fc = Vec::with_capacity(n / 8);
    let (mut a, mut b) = ([0.0f32; 8], [0.0f32; 8]);
    for _ in 0..n / 8 {
        m.process(&mut a, &mut b, None);
        fc.push(m.current_cutoff().0);
    }
    let (lo, hi) = fc
        .iter()
        .fold((f32::MAX, 0.0f32), |(a, b), &v| (a.min(v), b.max(v)));
    assert!((lo / 98.0 - 1.0).abs() < 0.02, "{lo}");
    assert!((hi / (98.0 * 32.0) - 1.0).abs() < 0.03, "{hi}");
    let mid = 98.0 * 2f32.powf(2.5);
    let rate = crossing_rate(&fc, mid) / 8.0;
    assert!((rate - 1.0).abs() < 5e-3, "{rate}");

    // envelope: +100 % opens 6 octaves on a full-scale signal, -100 % would close
    m.set_params(&MoodParams {
        env_pct: 100.0,
        ..mood_static(MoodType::Lp, true, 100.0)
    });
    m.reset();
    let x = sine(200.0, ENV_REF, 4800);
    let (mut l, mut r) = (x.clone(), x);
    run(&mut m, &mut l, &mut r);
    let f = m.current_cutoff().0;
    assert!(f > 100.0 * 2f32.powf(5.0), "{f}");

    // square / random waves stay bounded in range and are smoothed
    for w in [
        MoodWave::Square,
        MoodWave::Random,
        MoodWave::Ramp,
        MoodWave::SawDown,
    ] {
        m.set_params(&MoodParams {
            wave: w,
            speed_hz: 5.0,
            ..Default::default()
        });
        m.reset();
        let mut prev = 0.0f32;
        let mut max_step = 0.0f32;
        for i in 0..(2.0 * FS) as usize / 8 {
            m.process(&mut a, &mut b, None);
            let c = m.current_cutoff().0.ln();
            if i > 0 {
                max_step = max_step.max((c - prev).abs());
            }
            prev = c;
            assert!(c >= 98f32.ln() - 0.01 && c <= (98.0f32 * 32.0).ln() + 0.01);
        }
        // a 5-octave jump is spread over > 1 ms (8-sample steps < 1/4 of it)
        assert!(
            max_step < 0.25 * 5.0 * core::f32::consts::LN_2,
            "{w:?} {max_step}"
        );
    }
}

// ----- all models: stability, clicks -------------------------------------------------------------------

fn lcg(s: &mut u32) -> f32 {
    *s = s.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
    (*s >> 8) as f32 / (1u32 << 24) as f32 * 2.0 - 1.0
}

#[test]
fn extreme_settings_stay_stable() {
    // (slot, value) sets that push each model to its limits
    let extremes: [(&str, &[(u16, f32)]); 7] = [
        (
            "DIMCRS",
            &[(1, 1.0), (2, 1.0), (3, 1.0), (4, 1.0), (5, 0.0)],
        ),
        (
            "CHORUS",
            &[(6, 5.0), (8, 100.0), (9, 100.0), (11, 5.0), (5, 100.0)],
        ),
        (
            "FLANGER",
            &[
                (6, 0.5),
                (7, 20.0),
                (8, 100.0),
                (12, 90.0),
                (3, 20.0),
                (4, 20_000.0),
                (11, 5.0),
            ],
        ),
        (
            "PHASER",
            &[
                (12, 80.0),
                (11, 12.0),
                (5, 100.0),
                (6, 100.0),
                (1, 5.0),
                (4, 98.0),
            ],
        ),
        (
            "ROTARY",
            &[(1, 2.0), (3, 10.0), (7, 100.0), (8, 100.0), (6, 0.0)],
        ),
        ("PANNER", &[(4, 100.0), (5, 100.0), (6, 5.0), (8, 50.0)]),
        (
            "MOOD",
            &[
                (4, 10.0),
                (5, 10.0),
                (6, 100.0),
                (12, 20.0),
                (14, 5.0),
                (1, 15_000.0),
            ],
        ),
    ];
    for (id, set) in extremes {
        for neg in [false, true] {
            let mut m = create_model(BlockKind::Fx, id);
            m.prepare(FS, 64);
            for &(s, v) in set {
                m.set_param(s, ParamValue::Float(v));
            }
            if neg && id == "FLANGER" {
                m.set_param(flanger_slot::FEED, ParamValue::Float(-90.0));
            }
            if neg && id == "MOOD" {
                m.set_param(mood_slot::ENV, ParamValue::Float(-100.0));
                m.set_param(mood_slot::FBASE, ParamValue::Float(20.0));
            }
            let mut seed = 1u32;
            let mut peak = 0.0f32;
            for _ in 0..(5.0 * FS) as usize / 64 {
                let mut l = [0.0f32; 64];
                let mut r = [0.0f32; 64];
                for (a, b) in l.iter_mut().zip(r.iter_mut()) {
                    *a = lcg(&mut seed);
                    *b = lcg(&mut seed);
                }
                m.process(&mut l, &mut r, None);
                for v in l.iter().chain(&r) {
                    assert!(v.is_finite(), "{id}");
                    peak = peak.max(v.abs());
                }
            }
            assert!(peak < 20.0, "{id} neg={neg}: peak {peak}");
            // silence decays to silence (no self-oscillation, no denormal tails)
            let mut last = 0.0f32;
            for _ in 0..(2.0 * FS) as usize / 64 {
                let mut l = [0.0f32; 64];
                let mut r = [0.0f32; 64];
                m.process(&mut l, &mut r, None);
                last = l.iter().chain(&r).fold(0.0f32, |a, v| a.max(v.abs()));
            }
            assert!(last < 1e-3, "{id} neg={neg}: tail {last}");
        }
    }
}

/// Max |y[n] - y[n-1]| over `x`.
fn max_step(x: &[f32]) -> f32 {
    x.windows(2)
        .map(|w| (w[1] - w[0]).abs())
        .fold(0.0, f32::max)
}

#[test]
fn parameter_changes_do_not_click() {
    // abrupt changes per model: (slot, value)
    let changes: [(&str, &[(u16, f32)]); 7] = [
        ("DIMCRS", &[(1, 0.0), (3, 0.0), (4, 1.0), (6, 0.0)]),
        (
            "CHORUS",
            &[
                (5, 100.0),
                (6, 40.0),
                (8, 100.0),
                (10, 0.0),
                (3, 0.0),
                (11, 4.0),
            ],
        ),
        ("FLANGER", &[(5, 80.0), (6, 12.0), (12, -80.0), (8, 20.0)]),
        (
            "PHASER",
            &[(11, 12.0), (10, 100.0), (12, 80.0), (4, 80.0), (5, 0.0)],
        ),
        ("ROTARY", &[(1, 2.0), (4, 100.0), (5, 20.0), (6, 100.0)]),
        ("PANNER", &[(9, 20.0), (7, 0.0), (6, 4.0)]),
        (
            "MOOD",
            &[
                (2, 1.0),
                (3, 0.0),
                (4, 8.0),
                (1, 2000.0),
                (10, 50.0),
                (11, 0.0),
            ],
        ),
    ];
    for (id, set) in changes {
        let mut m = create_model(BlockKind::Fx, id);
        m.prepare(FS, 64);
        let n = (2.0 * FS) as usize;
        let x = sine(110.0, 0.5, n);
        let (mut l, mut r) = (x.clone(), x.clone());
        let half = n / 2;
        run(&mut *m, &mut l[..half], &mut r[..half]);
        for &(s, v) in set {
            m.set_param(s, ParamValue::Float(v));
        }
        run(&mut *m, &mut l[half..], &mut r[half..]);
        let before = max_step(&l[half / 2..half]).max(max_step(&r[half / 2..half]));
        let after = max_step(&l[n - half / 2..]).max(max_step(&r[n - half / 2..]));
        let w = half + 4800;
        let trans = max_step(&l[half - 1..w]).max(max_step(&r[half - 1..w]));
        let bound = 1.5 * before.max(after) + 2e-3;
        assert!(
            trans <= bound,
            "{id}: transition step {trans} > {bound} (before {before}, after {after})"
        );
    }
}

#[test]
fn registry_creates_all_modulation_models() {
    for id in IDS {
        let mut m = create_model(BlockKind::Fx, id);
        assert!(!m.is_stub() && m.todo().is_none(), "{id}");
        m.prepare(96_000.0, 128);
        let x = sine(80.0, 0.3, 4096);
        let (mut l, mut r) = (x.clone(), x);
        for (a, b) in l.chunks_mut(128).zip(r.chunks_mut(128)) {
            m.process(a, b, None);
        }
        assert!(l.iter().all(|v| v.is_finite()) && rms(&l) > 0.01, "{id}");
        // empty blocks are fine
        m.process(&mut [], &mut [], None);
    }
}
