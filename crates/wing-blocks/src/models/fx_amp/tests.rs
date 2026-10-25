use super::*;
use crate::model::{create_model, BlockKind};
use wing_dsp::Complex;

const FS: f64 = 48_000.0;
const AMPS: [AmpKind; 5] = [
    AmpKind::RackAmp,
    AmpKind::UkRock,
    AmpKind::Angel,
    AmpKind::JazzClean,
    AmpKind::Deluxe,
];
const IDS: [&str; 6] = ["RACKAMP", "UKROCK", "ANGEL", "JAZZC", "DELUXE", "BODY"];

fn sine(f: f64, amp: f32, n: usize) -> Vec<f32> {
    (0..n)
        .map(|i| amp * (2.0 * PI * f * i as f64 / FS).sin() as f32)
        .collect()
}

fn run(m: &mut dyn Model, l: &mut [f32], r: &mut [f32]) {
    for (a, b) in l.chunks_mut(64).zip(r.chunks_mut(64)) {
        m.process(a, b, None);
    }
}

fn amp(kind: AmpKind) -> Amp {
    let mut a = Amp::new(kind);
    a.prepare(FS, 64);
    a
}

fn set(m: &mut dyn Model, slot: u16, v: f32) {
    m.set_param(slot, ParamValue::Float(v));
}

fn rms(x: &[f32]) -> f64 {
    (x.iter().map(|&v| (v as f64) * (v as f64)).sum::<f64>() / x.len() as f64).sqrt()
}

fn lin_db(x: f64) -> f64 {
    20.0 * x.max(1e-30).log10()
}

/// In-place radix-2 FFT (tests only).
fn fft(re: &mut [f64], im: &mut [f64]) {
    let n = re.len();
    let mut j = 0;
    for i in 1..n {
        let mut bit = n >> 1;
        while j & bit != 0 {
            j ^= bit;
            bit >>= 1;
        }
        j |= bit;
        if i < j {
            re.swap(i, j);
            im.swap(i, j);
        }
    }
    let mut len = 2;
    while len <= n {
        let w = -2.0 * PI / len as f64;
        for s in (0..n).step_by(len) {
            for k in 0..len / 2 {
                let (c, si) = ((w * k as f64).cos(), (w * k as f64).sin());
                let (a, b) = (s + k, s + k + len / 2);
                let tr = re[b] * c - im[b] * si;
                let ti = re[b] * si + im[b] * c;
                re[b] = re[a] - tr;
                im[b] = im[a] - ti;
                re[a] += tr;
                im[a] += ti;
            }
        }
        len <<= 1;
    }
}

/// Amplitude spectrum (bins 0..=N/2) of `x` (rectangular window; use bin-exact tones).
fn spectrum(x: &[f32]) -> Vec<f64> {
    let n = x.len();
    let mut re: Vec<f64> = x.iter().map(|&v| v as f64).collect();
    let mut im = vec![0.0; n];
    fft(&mut re, &mut im);
    (0..=n / 2)
        .map(|k| 2.0 * re[k].hypot(im[k]) / n as f64)
        .collect()
}

const N_FFT: usize = 65_536;

/// Run a bin-exact tone (bin `k0` of an `N_FFT` FFT) through `m` (1 s settling) and return the spectrum.
fn tone_spectrum(m: &mut dyn Model, k0: usize, amp: f32) -> Vec<f64> {
    let f0 = k0 as f64 * FS / N_FFT as f64;
    let warm = 48_000;
    let mut l = sine(f0, amp, warm + N_FFT);
    let mut r = l.clone();
    run(m, &mut l, &mut r);
    assert!(l.iter().chain(&r).all(|v| v.is_finite()));
    spectrum(&l[warm..])
}

/// Worst non-harmonic component in 0..20 kHz relative to the fundamental (dBc). With a bin-exact `f0`
/// whose bin index is odd, every harmonic and every alias `n f0 mod fs` lands on its own bin.
fn worst_alias_dbc(spec: &[f64], k0: usize) -> (f64, f64) {
    let fund = spec[k0];
    let top = (20_000.0 / FS * N_FFT as f64) as usize;
    let mut worst: f64 = 0.0;
    let mut at = 0;
    for (b, &v) in spec.iter().enumerate().take(top + 1).skip(1) {
        if b % k0 == 0 {
            continue; // true harmonic
        }
        if v > worst {
            worst = v;
            at = b;
        }
    }
    (lin_db(worst / fund), at as f64 * FS / N_FFT as f64)
}

// ----- parameters ------------------------------------------------------------------------------------------

fn schema_variants() -> Vec<serde_json::Value> {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../data/schema/wing-tree.json"
    );
    let tree: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(path).unwrap()).unwrap();
    tree["variants"]["fx/1"]["mdl"]["variants"]
        .as_array()
        .unwrap()
        .clone()
}

#[test]
fn every_schema_parameter_is_mapped_with_its_default_and_range() {
    let variants = schema_variants();
    for id in IDS {
        let v = variants.iter().find(|v| v["name"] == id).unwrap();
        let params = v["params"].as_array().unwrap();
        let mut m = create_model(BlockKind::Fx, id);
        m.prepare(FS, 64);
        m.load_defaults();
        assert!(!m.is_stub(), "{id}");
        assert_eq!(m.id(), id);
        assert_eq!(
            m.latency(),
            if id == "BODY" { 0 } else { AMP_LATENCY },
            "{id}"
        );
        for (i, p) in params.iter().enumerate() {
            let slot = (i + 1) as u16;
            let got = m
                .param(slot)
                .unwrap_or_else(|| panic!("{id} slot {slot} unmapped"));
            let want: f32 = p["default"].as_str().unwrap().parse().unwrap();
            assert!(
                (got - want).abs() < 1e-4,
                "{id} {} {got} != {want}",
                p["name"]
            );
            let lo = p["min"].as_f64().unwrap() as f32;
            let hi = p["max"].as_f64().unwrap() as f32;
            for x in [lo, hi] {
                m.set_param(slot, ParamValue::Float(x));
                assert_eq!(m.param(slot), Some(x), "{id} {}", p["name"]);
            }
            // clamped to the schema range
            m.set_param(slot, ParamValue::Float(hi + 50.0));
            assert_eq!(m.param(slot), Some(hi), "{id} {}", p["name"]);
            m.set_param(slot, ParamValue::Float(lo - 50.0));
            assert_eq!(m.param(slot), Some(lo), "{id} {}", p["name"]);
            m.set_param(slot, ParamValue::Float(f32::NAN));
            assert_eq!(m.param(slot), Some(lo), "{id} NaN ignored");
            m.set_param(slot, ParamValue::Float(want));
        }
        assert_eq!(m.param(params.len() as u16 + 1), None, "{id}");
        assert_eq!(m.param(0), None, "{id}");
    }
}

// ----- building blocks -------------------------------------------------------------------------------------

#[test]
fn curve_antiderivatives_match_and_adaa2_is_transparent_for_small_signals() {
    for c in [Curve::new(1.0, 2.2), Curve::sym(4.0), Curve::sym(1.0)] {
        for i in -800..=800 {
            let u = i as f64 * 0.0137;
            let h = 1e-5;
            let d1 = (c.f1(u + h) - c.f1(u - h)) / (2.0 * h);
            let d2 = (c.f2(u + h) - c.f2(u - h)) / (2.0 * h);
            assert!(
                (d1 - c.f(u)).abs() < 1e-7,
                "{c:?} u={u}: {d1} vs {}",
                c.f(u)
            );
            assert!(
                (d2 - c.f1(u)).abs() < 1e-7,
                "{c:?} u={u}: {d2} vs {}",
                c.f1(u)
            );
        }
        assert_eq!(c.f(1e6), c.lp);
        assert_eq!(c.f(-1e6), -c.ln);
        assert!((c.f(1e-6) - 1e-6).abs() < 1e-15, "unity slope");
    }
    // small signals: output = input delayed by exactly one sample (linear part untouched)
    let c = Curve::new(1.0, 2.0);
    let mut a = Adaa2::default();
    let mut prev = 0.0;
    for i in 0..2000 {
        let u = 1e-4 * (i as f64 * 0.37).sin();
        let y = a.tick(c, u);
        assert!((y - prev).abs() < 1e-12, "{i}: {prev} {y}");
        prev = u;
    }
    // a slow large signal follows the curve (delayed one sample)
    let mut a = Adaa2::default();
    let mut prev = 0.0;
    for i in 0..20_000 {
        let u = 8.0 * (2.0 * PI * 50.0 * i as f64 / 192_000.0).sin();
        let y = a.tick(c, u);
        if i > 2 {
            assert!((y - c.f(prev)).abs() < 1e-4, "{i}: {} {y}", c.f(prev));
        }
        prev = u;
    }
    // constant input converges to the curve value
    for _ in 0..4 {
        a.tick(c, 50.0);
    }
    assert!((a.tick(c, 50.0) - c.f(50.0)).abs() < 1e-12);
}

#[test]
fn first_order_sections_have_the_designed_gains() {
    let fs = 192_000.0;
    let hs = Fo::high_shelf(fs, 3_500.0, 9.0);
    assert!(hs.magnitude_db(fs, 10.0).abs() < 0.01);
    assert!((hs.magnitude_db(fs, 60_000.0) - 9.0).abs() < 0.3);
    assert!((hs.magnitude_db(fs, 3_500.0) - 4.5).abs() < 0.05);
    let ls = Fo::low_shelf(fs, 110.0, 6.0);
    assert!((ls.magnitude_db(fs, 2.0) - 6.0).abs() < 0.01);
    assert!(ls.magnitude_db(fs, 20_000.0).abs() < 0.01);
    let hp = Fo::highpass(fs, 100.0);
    assert!((hp.magnitude_db(fs, 100.0) + 3.0103).abs() < 0.01);
    let lp = Fo::lowpass(fs, 8_000.0);
    assert!((lp.magnitude_db(fs, 8_000.0) + 3.0103).abs() < 0.01);
}

// ----- tone stack ----------------------------------------------------------------------------------------

/// Solve the complex linear system `y v = i` (Gaussian elimination, partial pivoting).
fn solve(mut y: [[Complex; 5]; 5], mut i: [Complex; 5]) -> [Complex; 5] {
    let n = 5;
    for c in 0..n {
        let p = (c..n)
            .max_by(|&a, &b| y[a][c].norm().partial_cmp(&y[b][c].norm()).unwrap())
            .unwrap();
        y.swap(c, p);
        i.swap(c, p);
        for r in c + 1..n {
            let f = y[r][c] / y[c][c];
            let pivot = y[c];
            for (yk, pk) in y[r].iter_mut().zip(pivot).skip(c) {
                *yk = *yk - f * pk;
            }
            i[r] = i[r] - f * i[c];
        }
    }
    let mut v = [Complex::new(0.0, 0.0); 5];
    for r in (0..n).rev() {
        let mut s = i[r];
        for k in r + 1..n {
            s = s - y[r][k] * v[k];
        }
        v[r] = s / y[r][r];
    }
    v
}

/// Independent reference: nodal analysis of the TMB network at frequency `f` (unit input, unloaded output).
fn tonestack_nodal(p: &ToneStackParts, t: f64, m: f64, l: f64, f: f64) -> Complex {
    let s = Complex::new(0.0, 2.0 * PI * f);
    let g = |r: f64| Complex::new(1.0 / r, 0.0);
    let c = |cap: f64| s * cap;
    // nodes: 0 = A, 1 = T, 2 = O, 3 = B, 4 = M; source node VIN = 1 V; -1 = ground
    let z = Complex::new(0.0, 0.0);
    let mut y = [[z; 5]; 5];
    let mut cur = [z; 5];
    let mut add = |a: i32, b: i32, adm: Complex| {
        for (p, q) in [(a, b), (b, a)] {
            if p >= 0 {
                let p = p as usize;
                y[p][p] = y[p][p] + adm;
                if q >= 0 {
                    y[p][q as usize] = y[p][q as usize] - adm;
                } else if q == -2 {
                    cur[p] = cur[p] + adm;
                }
            }
        }
    };
    const VIN: i32 = -2;
    const GND: i32 = -1;
    add(VIN, 0, g(p.r4));
    add(VIN, 1, c(p.c1));
    add(1, 2, g(p.r1 * (1.0 - t)));
    add(2, 3, g(p.r1 * t));
    add(0, 3, c(p.c2));
    add(3, 4, g(p.r2 * l));
    add(0, 4, c(p.c3));
    add(4, GND, g(p.r3 * m));
    solve(y, cur)[2]
}

fn poly(c: &[f64; 4], x: Complex) -> Complex {
    let mut acc = Complex::new(0.0, 0.0);
    for &k in c.iter().rev() {
        acc = acc * x + Complex::new(k, 0.0);
    }
    acc
}

fn analog_response(p: &ToneStackParts, t: f64, m: f64, l: f64, f: f64) -> Complex {
    let (b, a) = p.analog(t, m, l);
    let s = Complex::new(0.0, 2.0 * PI * f);
    poly(&b, s) / poly(&a, s)
}

const PARTS: [ToneStackParts; 5] = [
    ToneStackParts::BASSMAN,
    ToneStackParts::JCM800,
    ToneStackParts::ANGEL,
    ToneStackParts::JC120,
    ToneStackParts::DELUXE,
];
const POTS: [(f64, f64, f64); 6] = [
    (0.5, 0.5, 0.5),
    (0.1, 0.9, 0.2),
    (0.9, 0.1, 0.8),
    (0.3, 0.6, 0.95),
    (0.99, 0.99, 0.99),
    (0.02, 0.03, 0.04),
];
const FREQS: [f64; 8] = [
    20.0, 60.0, 150.0, 400.0, 1_000.0, 2_500.0, 6_000.0, 15_000.0,
];

#[test]
fn tone_stack_transfer_function_matches_nodal_analysis() {
    for p in &PARTS {
        for &(t, m, l) in &POTS {
            for &f in &FREQS {
                let a = analog_response(p, t, m, l, f);
                let n = tonestack_nodal(p, t, m, l, f);
                let err = (a - n).norm() / n.norm();
                assert!(err < 1e-9, "{p:?} t{t} m{m} l{l} f{f}: rel err {err}");
            }
        }
    }
}

#[test]
fn tone_stack_filter_matches_analytic_response() {
    let fs = FS * OS as f64;
    for p in &PARTS {
        for &(t, m, l) in &POTS {
            let mut ts = ToneStack::default();
            ts.set(p.digital(fs, t, m, l));
            // impulse response (long enough for the slowest pole to decay)
            let h: Vec<f64> = (0..400_000)
                .map(|i| ts.tick(if i == 0 { 1.0 } else { 0.0 }))
                .collect();
            for &f in &FREQS {
                let w = 2.0 * PI * f / fs;
                let got = wing_dsp::dtft(&h, w).db();
                // bilinear: exact at the prewarped analog frequency
                let fw = (fs / PI) * (w / 2.0).tan();
                let want = analog_response(p, t, m, l, fw).db();
                assert!(
                    (got - want).abs() < 0.01,
                    "{p:?} t{t} m{m} l{l} f{f}: {got} vs {want}"
                );
                // and within 0.05 dB of the unwarped analog response up to 6 kHz
                if f <= 6_000.0 {
                    let a = analog_response(p, t, m, l, f).db();
                    assert!((got - a).abs() < 0.05, "{p:?} f{f}: {got} vs {a}");
                }
            }
        }
    }
}

#[test]
fn tone_stack_controls_act_as_expected() {
    let p = ToneStackParts::JCM800;
    let r = |t, m, l, f| analog_response(&p, t, m, l, f).db();
    assert!(
        r(0.5, 0.5, 1.0, 40.0) > r(0.5, 0.5, 0.05, 40.0) + 6.0,
        "bass"
    );
    assert!(
        r(1.0, 0.5, 0.5, 6_000.0) > r(0.0, 0.5, 0.5, 6_000.0) + 6.0,
        "treble"
    );
    assert!(
        r(0.5, 1.0, 0.5, 600.0) > r(0.5, 0.0, 0.5, 600.0) + 6.0,
        "middle"
    );
    // classic mid scoop at noon
    assert!(r(0.5, 0.5, 0.5, 500.0) < r(0.5, 0.5, 0.5, 80.0));
    assert!(r(0.5, 0.5, 0.5, 500.0) < r(0.5, 0.5, 0.5, 5_000.0));
    // passive network: never gains
    for &f in &FREQS {
        for &(t, m, l) in &POTS {
            assert!(r(t, m, l, f) < 0.01);
        }
    }
}

// ----- oversampling alignment ----------------------------------------------------------------------------

#[test]
fn oversampled_path_latency_is_exactly_47_samples() {
    // the oversampled section delays every signal by OS_DELAY 4x samples (ADAA2 curves + alignment)
    for kind in AMPS {
        let v = voice(kind);
        assert!(v.n_stages <= MAX_STAGES, "{kind:?}");
    }
    let mut os = Oversampler4x::new(32);
    let mut delay = [0.0f32; OS_DELAY];
    let mut pos = 0;
    let n = 4800;
    let f = 3_000.0;
    let x = sine(f, 0.5, n);
    let mut y = vec![0.0f32; n];
    let mut hi = vec![0.0f32; 128];
    for (xc, yc) in x.chunks(32).zip(y.chunks_mut(32)) {
        let h = &mut hi[..4 * xc.len()];
        os.upsample(xc, h);
        for s in h.iter_mut() {
            std::mem::swap(s, &mut delay[pos]);
            pos = (pos + 1) % OS_DELAY;
        }
        os.downsample(h, yc);
    }
    for (i, &yi) in y.iter().enumerate().skip(200) {
        let want = 0.5 * (2.0 * PI * f * (i as f64 - AMP_LATENCY as f64) / FS).sin();
        assert!((yi as f64 - want).abs() < 1e-4, "{i}");
    }
    assert_eq!(
        os.latency() + OS_DELAY as f64 / OS as f64,
        AMP_LATENCY as f64
    );
}

// ----- aliasing --------------------------------------------------------------------------------------------

fn high_gain(kind: AmpKind) -> Amp {
    let mut a = amp(kind);
    match kind {
        AmpKind::RackAmp => {
            set(&mut a, rack_slot::PRE, 10.0);
            set(&mut a, rack_slot::DRIVE, 10.0);
            set(&mut a, rack_slot::CRUNCH, 10.0);
        }
        AmpKind::UkRock | AmpKind::Angel => {
            set(&mut a, tube_slot::GAIN, 10.0);
            set(&mut a, tube_slot::MSTR, 10.0);
            set(&mut a, tube_slot::TREB, 10.0);
            set(&mut a, tube_slot::PRES, 10.0);
            set(&mut a, tube_slot::SAG, 10.0);
        }
        AmpKind::JazzClean => {
            set(&mut a, jazz_slot::VOL, 10.0);
            set(&mut a, jazz_slot::TREB, 10.0);
            set(&mut a, jazz_slot::BRI, 1.0);
        }
        AmpKind::Deluxe => {
            set(&mut a, deluxe_slot::VOL, 10.0);
            set(&mut a, deluxe_slot::TREB, 10.0);
            set(&mut a, deluxe_slot::SAG, 10.0);
        }
    }
    a.reset();
    a
}

/// 4 kHz (bin-exact 3999.76 Hz), -6 dBFS, every amp at full gain: aliases < -80 dBc in 0..20 kHz.
#[test]
fn aliasing_below_minus_80_dbc_at_4khz_high_gain() {
    let k0 = 5461; // odd -> every alias on its own bin; f0 = 3999.76 Hz
    let mut fails = Vec::new();
    for kind in AMPS {
        for cab in [true, false] {
            let mut a = high_gain(kind);
            if !cab {
                let slot = slot_table(kind)
                    .iter()
                    .position(|s| s.1 == ctl::CAB)
                    .unwrap()
                    + 1;
                set(&mut a, slot as u16, 0.0);
                a.reset();
            }
            let spec = tone_spectrum(&mut a, k0, 0.5);
            let (dbc, at) = worst_alias_dbc(&spec, k0);
            // the amp really is distorting hard
            let h3 = lin_db(spec[3 * k0] / spec[k0]);
            eprintln!("{kind:?} cab {cab}: worst alias {dbc:.1} dBc at {at:.0} Hz, H3 {h3:.1} dBc");
            if dbc >= -80.0 {
                fails.push((kind, cab, dbc, at));
            }
        }
    }
    assert!(fails.is_empty(), "{fails:?}");
}

// ----- behaviour -------------------------------------------------------------------------------------------

/// THD-ish: energy of harmonics 2..10 relative to the fundamental (dB).
fn thd_db(spec: &[f64], k0: usize) -> f64 {
    let h: f64 = (2..=10)
        .filter(|h| h * k0 < spec.len())
        .map(|h| spec[h * k0].powi(2))
        .sum();
    10.0 * (h / spec[k0].powi(2)).log10()
}

#[test]
fn gain_knob_increases_distortion() {
    let k0 = 301; // 220.4 Hz
    for kind in AMPS {
        let gain_slot = 1; // PREAMP / GAIN / VOLUME are slot 1 everywhere
        let mut lo = amp(kind);
        set(
            &mut lo,
            gain_slot,
            if kind == AmpKind::Deluxe { 1.5 } else { 1.0 },
        );
        let mut hi = amp(kind);
        set(&mut hi, gain_slot, 10.0);
        if kind == AmpKind::RackAmp {
            // RACKAMP's DRIVE is a second (power) distortion stage: keep it low for the PREAMP sweep
            set(&mut lo, rack_slot::DRIVE, 0.0);
            set(&mut hi, rack_slot::DRIVE, 0.0);
        }
        lo.reset();
        hi.reset();
        let t_lo = thd_db(&tone_spectrum(&mut lo, k0, 0.25), k0);
        let t_hi = thd_db(&tone_spectrum(&mut hi, k0, 0.25), k0);
        eprintln!("{kind:?}: THD low gain {t_lo:.1} dB, high gain {t_hi:.1} dB");
        assert!(t_hi > t_lo + 10.0, "{kind:?}: {t_lo} -> {t_hi}");
    }
    // the high-gain amp really saturates at full gain: +12 dB more input gives < 2 dB more output
    let level = |amp_in: f32| {
        let mut a = amp(AmpKind::Angel);
        set(&mut a, tube_slot::GAIN, 10.0);
        a.reset();
        let mut l = sine(220.0, amp_in, 24_000);
        let mut r = l.clone();
        run(&mut a, &mut l, &mut r);
        rms(&l[12_000..])
    };
    let d = lin_db(level(0.4) / level(0.1));
    assert!(d < 2.0, "ANGEL compression {d}");
}

#[test]
fn defaults_give_sane_levels() {
    let mut levels = Vec::new();
    for kind in AMPS {
        let mut a = amp(kind);
        let mut l = sine(220.0, 0.25, 48_000);
        let mut r = l.clone();
        run(&mut a, &mut l, &mut r);
        let lv = lin_db(rms(&l[24_000..]) / rms(&sine(220.0, 0.25, 4800)));
        eprintln!("{kind:?}: default level {lv:.1} dB re input");
        levels.push((kind, lv));
        assert_eq!(l, r, "{kind:?}: identical channels stay identical");
    }
    for (kind, lv) in levels {
        assert!((-6.0..=6.0).contains(&lv), "{kind:?}: {lv}");
    }
}

#[test]
fn silence_in_silence_out() {
    for kind in AMPS {
        let mut a = high_gain(kind);
        let mut l = vec![0.0f32; 48_000];
        let mut r = l.clone();
        run(&mut a, &mut l, &mut r);
        assert!(l.iter().chain(&r).all(|&v| v == 0.0), "{kind:?}");
    }
}

/// Energy above `f` relative to total (dB).
fn hf_share_db(x: &[f32], f: f64) -> f64 {
    let spec = spectrum(x);
    let k = (f / FS * x.len() as f64) as usize;
    let tot: f64 = spec.iter().map(|v| v * v).sum();
    let hf: f64 = spec[k..].iter().map(|v| v * v).sum();
    10.0 * (hf / tot).log10()
}

fn noise(n: usize, amp: f32) -> Vec<f32> {
    let mut s = 0x1234_5678u32;
    (0..n)
        .map(|_| {
            s ^= s << 13;
            s ^= s >> 17;
            s ^= s << 5;
            amp * (s as f32 / u32::MAX as f32 * 2.0 - 1.0)
        })
        .collect()
}

fn render(a: &mut Amp, x: &[f32]) -> Vec<f32> {
    let mut l = x.to_vec();
    let mut r = x.to_vec();
    run(a, &mut l, &mut r);
    l
}

#[test]
fn cab_sim_removes_fizz_and_cross_fades() {
    for kind in AMPS {
        let cab = slot_table(kind)
            .iter()
            .position(|s| s.1 == ctl::CAB)
            .unwrap() as u16
            + 1;
        let x = noise(1 << 15, 0.2);
        let mut on = amp(kind);
        let mut off = amp(kind);
        set(&mut off, cab, 0.0);
        off.reset();
        let y_on = render(&mut on, &x);
        let y_off = render(&mut off, &x);
        let (s_on, s_off) = (hf_share_db(&y_on, 8_000.0), hf_share_db(&y_off, 8_000.0));
        eprintln!("{kind:?}: HF share cab on {s_on:.1} dB, off {s_off:.1} dB");
        assert!(s_on < s_off - 10.0, "{kind:?}: {s_on} vs {s_off}");
        // switching is click-free: no sample jumps more than the signal itself does
        let mut a = amp(kind);
        let mut l = sine(220.0, 0.25, 9600);
        let mut r = l.clone();
        for (k, (bl, br)) in l.chunks_mut(64).zip(r.chunks_mut(64)).enumerate() {
            if k == 60 {
                set(&mut a, cab, 0.0);
            }
            a.process(bl, br, None);
        }
        let max_step = l[2400..]
            .windows(2)
            .map(|w| (w[1] - w[0]).abs())
            .fold(0.0f32, f32::max);
        let base_step = l[2400..3800]
            .windows(2)
            .map(|w| (w[1] - w[0]).abs())
            .fold(0.0f32, f32::max);
        assert!(
            max_step < 3.0 * base_step + 0.05,
            "{kind:?}: {max_step} vs {base_step}"
        );
    }
}

#[test]
fn tone_controls_shape_the_spectrum() {
    let x = noise(1 << 15, 0.05);
    let band = |y: &[f32], f1: f64, f2: f64| {
        let spec = spectrum(y);
        let k1 = (f1 / FS * y.len() as f64) as usize;
        let k2 = (f2 / FS * y.len() as f64) as usize;
        10.0 * spec[k1..k2].iter().map(|v| v * v).sum::<f64>().log10()
    };
    let cases: [(AmpKind, u16, u16); 5] = [
        (AmpKind::UkRock, tube_slot::BASS, tube_slot::TREB),
        (AmpKind::Angel, tube_slot::BASS, tube_slot::TREB),
        (AmpKind::JazzClean, jazz_slot::BASS, jazz_slot::TREB),
        (AmpKind::Deluxe, deluxe_slot::BASS, deluxe_slot::TREB),
        (AmpKind::RackAmp, rack_slot::LEQ, rack_slot::HEQ),
    ];
    for (kind, bass, treb) in cases {
        let lo_v = if kind == AmpKind::Deluxe { 1.0 } else { 0.0 };
        let mk = |slot: u16, v: f32| {
            let mut a = amp(kind);
            set(&mut a, 1, if kind == AmpKind::Deluxe { 2.0 } else { 1.0 }); // low gain
            set(&mut a, slot, v);
            a.reset();
            render(&mut a, &x)
        };
        let (b_lo, b_hi) = (mk(bass, lo_v), mk(bass, 10.0));
        let (t_lo, t_hi) = (mk(treb, lo_v), mk(treb, 10.0));
        let db_bass = band(&b_hi, 80.0, 200.0) - band(&b_lo, 80.0, 200.0);
        let db_treb = band(&t_hi, 2_500.0, 5_000.0) - band(&t_lo, 2_500.0, 5_000.0);
        eprintln!("{kind:?}: bass {db_bass:.1} dB, treble {db_treb:.1} dB");
        assert!(db_bass > 4.0, "{kind:?} bass {db_bass}");
        assert!(db_treb > 4.0, "{kind:?} treble {db_treb}");
    }
}

#[test]
fn presence_bright_mid_boost_and_bottom_switches_work() {
    let x = noise(1 << 15, 0.02);
    let band = |y: &[f32], f1: f64, f2: f64| {
        let spec = spectrum(y);
        let k1 = (f1 / FS * y.len() as f64) as usize;
        let k2 = (f2 / FS * y.len() as f64) as usize;
        10.0 * spec[k1..k2].iter().map(|v| v * v).sum::<f64>().log10()
    };
    let mk = |kind: AmpKind, slot: u16, v: f32| {
        let mut a = amp(kind);
        set(&mut a, 1, 1.0);
        set(&mut a, slot, v);
        a.reset();
        render(&mut a, &x)
    };
    let d = |kind, slot, lo, hi, f1, f2| {
        band(&mk(kind, slot, hi), f1, f2) - band(&mk(kind, slot, lo), f1, f2)
    };
    let pres = d(
        AmpKind::UkRock,
        tube_slot::PRES,
        0.0,
        10.0,
        3_000.0,
        6_000.0,
    );
    let bri = d(AmpKind::Angel, tube_slot::BRI, 0.0, 1.0, 3_000.0, 6_000.0);
    let midb = d(AmpKind::Angel, tube_slot::MIDB, 0.0, 1.0, 500.0, 900.0);
    let bot = d(AmpKind::Angel, tube_slot::BOT, 0.0, 1.0, 80.0, 150.0);
    let jbri = d(
        AmpKind::JazzClean,
        jazz_slot::BRI,
        0.0,
        1.0,
        3_000.0,
        6_000.0,
    );
    let rbuzz = d(AmpKind::RackAmp, rack_slot::BUZZ, 0.0, 10.0, 100.0, 250.0);
    let rpunch = d(AmpKind::RackAmp, rack_slot::PUNCH, 0.0, 10.0, 300.0, 500.0);
    let rcrunch = d(
        AmpKind::RackAmp,
        rack_slot::CRUNCH,
        0.0,
        10.0,
        2_000.0,
        3_500.0,
    );
    eprintln!("pres {pres:.1} bri {bri:.1} midb {midb:.1} bot {bot:.1} jbri {jbri:.1} buzz {rbuzz:.1} punch {rpunch:.1} crunch {rcrunch:.1}");
    for (name, v) in [
        ("presence", pres),
        ("bright", bri),
        ("mid boost", midb),
        ("bottom", bot),
        ("jazz bright", jbri),
        ("buzz", rbuzz),
        ("punch", rpunch),
        ("crunch", rcrunch),
    ] {
        assert!(v > 2.0, "{name}: {v}");
    }
}

#[test]
fn sag_compresses_the_power_amp() {
    // a loud burst: with full sag the sustained level drops below the no-sag level
    for kind in [AmpKind::UkRock, AmpKind::Deluxe] {
        let sag_slot = if kind == AmpKind::Deluxe {
            deluxe_slot::SAG
        } else {
            tube_slot::SAG
        };
        let lvl = |sag: f32| {
            let mut a = amp(kind);
            set(&mut a, 1, 10.0);
            if kind == AmpKind::UkRock {
                set(&mut a, tube_slot::MSTR, 10.0);
            }
            set(&mut a, sag_slot, sag);
            a.reset();
            let y = render(&mut a, &sine(110.0, 0.5, 24_000));
            rms(&y[12_000..])
        };
        let (none, full) = (
            lvl(if kind == AmpKind::Deluxe { 1.0 } else { 0.0 }),
            lvl(10.0),
        );
        let d = lin_db(full / none);
        eprintln!("{kind:?}: sag level change {d:.1} dB");
        assert!(d < -1.0, "{kind:?}: {d}");
    }
}

#[test]
fn extreme_params_and_input_never_produce_nan() {
    let mut s = 0xdead_beefu32;
    let mut rnd = move || {
        s ^= s << 13;
        s ^= s >> 17;
        s ^= s << 5;
        s as f32 / u32::MAX as f32
    };
    for id in IDS {
        let mut m = create_model(BlockKind::Fx, id);
        m.prepare(FS, 64);
        let n_slots = (1..=12u16).filter(|&k| m.param(k).is_some()).count() as u16;
        let mut l = [0.0f32; 64];
        let mut r = [0.0f32; 64];
        for k in 0..1500 {
            // every block: random extreme parameter jumps
            for slot in 1..=n_slots {
                let v = match k % 4 {
                    0 => 0.0,
                    1 => 1e6,
                    2 => -1e6,
                    _ => rnd() * 10.0,
                };
                m.set_param(slot, ParamValue::Float(v));
            }
            m.commit();
            for (i, (a, b)) in l.iter_mut().zip(r.iter_mut()).enumerate() {
                *a = match (k / 100) % 5 {
                    0 => 20.0 * (rnd() - 0.5),
                    1 => {
                        if i == 0 {
                            1e30
                        } else {
                            0.0
                        }
                    }
                    2 => f32::NAN,
                    3 => f32::INFINITY * if i % 2 == 0 { 1.0 } else { -1.0 },
                    _ => 1e-38 * rnd(),
                };
                *b = -*a;
            }
            m.process(&mut l, &mut r, None);
            assert!(
                l.iter().chain(&r).all(|v| v.is_finite() && v.abs() < 1e3),
                "{id} block {k}: {:?}",
                &l[..4]
            );
        }
    }
}

#[test]
fn odd_block_sizes_match_whole_blocks() {
    for kind in AMPS {
        let x = sine(330.0, 0.3, 4096);
        let mut a = amp(kind);
        let mut b = amp(kind);
        let (mut l1, mut r1) = (x.clone(), x.clone());
        run(&mut a, &mut l1, &mut r1);
        let (mut l2, mut r2) = (x.clone(), x.clone());
        let mut i = 0;
        let mut step = 1;
        while i < x.len() {
            let e = (i + step).min(x.len());
            b.process(&mut l2[i..e], &mut r2[i..e], None);
            i = e;
            step = step % 61 + 7;
        }
        // parameters are smoothed per chunk, so allow a tiny difference; defaults have no ramps
        let d = l1
            .iter()
            .zip(&l2)
            .map(|(p, q)| (p - q).abs())
            .fold(0.0f32, f32::max);
        assert!(d < 1e-4, "{kind:?}: {d}");
    }
}

// ----- BODY ------------------------------------------------------------------------------------------------

#[test]
fn body_zero_is_passthrough_and_amount_adds_resonance() {
    let mut b = BodyRez::new();
    b.prepare(FS, 64);
    b.set_param(BODY_SLOT, ParamValue::Float(0.0));
    b.reset();
    let x = noise(4096, 0.3);
    let (mut l, mut r) = (x.clone(), x.clone());
    run(&mut b, &mut l, &mut r);
    assert!(l.iter().zip(&x).all(|(a, b)| (a - b).abs() < 1e-6));
    let c = body_coeffs(FS, 1.0);
    let resp = |f: f64| -> f64 { c.iter().map(|s| s.magnitude_db(FS, f)).sum() };
    assert!(resp(95.0) > 10.0, "{}", resp(95.0));
    assert!(resp(210.0) > 5.0);
    assert!(resp(1_300.0) < -3.0);
    assert!(resp(12_000.0) < -3.0);
    assert!(resp(3_000.0).abs() < 2.5);
    // amount scales the curve
    let half = body_coeffs(FS, 0.25);
    let r25: f64 = half.iter().map(|s| s.magnitude_db(FS, 95.0)).sum();
    assert!((r25 - 3.0).abs() < 0.5, "{r25}");
    // processed: low band rises with BODY
    let mut b = BodyRez::new();
    b.prepare(FS, 64);
    b.set_param(BODY_SLOT, ParamValue::Float(100.0));
    let (mut l, mut r) = (sine(95.0, 0.1, 48_000), sine(95.0, 0.1, 48_000));
    run(&mut b, &mut l, &mut r);
    let g = lin_db(rms(&l[24_000..]) / rms(&sine(95.0, 0.1, 4800)));
    assert!(g > 10.0, "{g}");
    assert_eq!(b.latency(), 0);
}
