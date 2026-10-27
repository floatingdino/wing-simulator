//! FX meter state words 4..9 that WING-Edit pages decode (T087, `wing_blocks::meter`): PCORR, V-IMG, SUB-M,
//! SPKMAN, DE-S2 (user report: the PCORR and similar visualisations stayed empty).

use wing_blocks::meter::FxMeterLaw;
use wing_blocks::model::{create_model, BlockKind, ParamValue};

const FS: f64 = 48_000.0;

/// Run `m` on a stereo signal `f(n) -> (l, r)` for `secs`, 64-sample blocks; return its words through the law.
fn words(
    id: &'static str,
    secs: f64,
    set: &[(u16, f32)],
    f: impl Fn(usize) -> (f32, f32),
) -> [i16; 6] {
    let mut m = create_model(BlockKind::Fx, id);
    m.prepare(FS, 64);
    m.load_defaults();
    for &(s, v) in set {
        m.set_param(s, ParamValue::Float(v));
    }
    m.commit();
    m.reset();
    let n = (secs * FS) as usize;
    let mut k = 0;
    while k < n {
        let (mut l, mut r): (Vec<f32>, Vec<f32>) = (k..k + 64).map(&f).unzip();
        m.process(&mut l, &mut r, None);
        k += 64;
    }
    let mut w = [0.0f32; 6];
    let c = m.fx_meter_words(&mut w);
    assert!(c > 0, "{id} reports words");
    FxMeterLaw::for_model(id).words(0.0, &w[..c])
}

fn sine(f: f64, a: f32) -> impl Fn(usize) -> f32 {
    move |n| a * (std::f64::consts::TAU * f * n as f64 / FS).sin() as f32
}

#[test]
fn pcorr_reports_period_note_and_ratio() {
    let s = sine(220.0, 0.5);
    let w = words("PCORR", 1.0, &[], |n| (s(n), s(n)));
    // detected period 64 * 48000 / 220 = 13964
    assert!((w[2] as i32 - 13_964).abs() < 150, "period word {}", w[2]);
    // A (pitch class 9) -> 2048 * 10
    assert_eq!(w[1], 2048 * 10, "note word");
    // in tune: ratio ~1
    assert!((w[0] as i32 - 2048).abs() < 40, "ratio word {}", w[0]);
    // silence: unvoiced
    let q = words("PCORR", 1.0, &[], |_| (0.0, 0.0));
    assert_eq!((q[1], q[2]), (0, 0));
}

#[test]
fn vimg_reports_mid_side_and_correlation() {
    let s = sine(1000.0, 0.25);
    // mono: correlation +1, no side
    let w = words("V-IMG", 0.5, &[], |n| (s(n), s(n)));
    assert!(w[2] > 900, "mid {}", w[2]);
    assert!(w[3] < 20, "side {}", w[3]);
    assert!(w[4] > 2000, "correlation {}", w[4]);
    // out of phase: correlation -1
    let w = words("V-IMG", 0.5, &[], |n| (s(n), -s(n)));
    assert!(w[3] > 500 && w[2] < 20, "anti-phase mid/side {w:?}");
    assert!(w[4] < 48, "anti-phase correlation {}", w[4]);
}

#[test]
fn subm_spkman_and_deesser_words() {
    let s = sine(55.0, 0.5);
    let w = words("SUB-M", 0.5, &[], |n| (s(n), s(n)));
    assert!(w[..5].iter().any(|&b| b > 20), "SUB-M band levels {w:?}");
    // SPKMAN: limiter input ~ the programme level, gain 2048 while not limiting
    let s = sine(1000.0, 0.5);
    let w = words("SPKMAN", 0.3, &[], |n| (s(n), s(n)));
    assert!(
        (w[2] as i32 - 1024).abs() < 80,
        "SPKMAN limiter input {}",
        w[2]
    );
    assert!(w[4] > 2000, "SPKMAN limiter gain {}", w[4]);
    // DE-S2: a loud 7 kHz tone reduces a band
    let s = sine(7000.0, 0.5);
    let w = words("DE-S2", 0.5, &[(1, 40.0), (2, 40.0)], |n| (s(n), s(n)));
    assert!(w[0] > 10 || w[1] > 10, "DE-S2 band GR {w:?}");
}
