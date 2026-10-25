//! T04J: WING's EQ `mix` law through the engine's model-slot host, for every EQ model.
//!
//! `ae::Cequalizer::update` passes `mix / 100` to the selected model's `parchg`, which scales the band gains; there
//! is no dry/wet blend. Every `*/eq` [`ModelSlot`] therefore forwards `mix` (0..125 %) to its models and never
//! blends. The processed impulse response of the slot is compared with WING-Edit's own coefficients at 44.1 kHz
//! and several mix values (`data/reference-curves/44k1/`, emulated WING-Edit): < 0.01 dB in 20 Hz..20 kHz.

use wing_blocks::eq::BandType;
use wing_blocks::fidelity::{FreqCurve, FreqCurves, RefParam};
use wing_blocks::models::eq_alt::AltKind;
use wing_engine::params::std_models::{block_kind, model_factory};
use wing_engine::params::strip::ModelSlot;
use wing_model::Schema;

const BLOCK: usize = 64;
const IR: usize = 1 << 15;

fn slot(key: &str, fs: f64) -> ModelSlot {
    let kind = block_kind(key).expect("EQ group");
    ModelSlot::from_group(key, fs, BLOCK, model_factory(kind, fs, BLOCK))
}

fn model_index(key: &str, name: &str) -> u16 {
    let s = Schema::get();
    let g = s.groups().iter().find(|g| g.key == key).unwrap();
    s.models(g)
        .iter()
        .find(|m| m.name == name)
        .unwrap()
        .enum_index as u16
}

/// Impulse response of the slot (on, mix %), magnitude in dB at `freqs`.
fn slot_response(s: &mut ModelSlot, mix: f32, fs: f64, freqs: &[f64]) -> Vec<f64> {
    s.set_wrapper(0, 1.0);
    s.set_wrapper(1, mix);
    s.settle();
    let mut h = vec![0.0f32; IR];
    h[0] = 1.0;
    for blk in h.chunks_mut(BLOCK) {
        let mut r = blk.to_vec();
        s.process(blk, &mut r);
    }
    freqs
        .iter()
        .map(|&f| {
            let w = 2.0 * std::f64::consts::PI * f / fs;
            let (sw, cw) = w.sin_cos();
            let (mut zr, mut zi) = (1.0f64, 0.0f64);
            let (mut ar, mut ai) = (0.0f64, 0.0f64);
            for &x in &h {
                ar += x as f64 * zr;
                ai += x as f64 * zi;
                (zr, zi) = (zr * cw + zi * sw, zi * cw - zr * sw);
            }
            10.0 * (ar * ar + ai * ai).max(1e-300).log10()
        })
        .collect()
}

fn check(tag: &str, got: &[f64], freqs: &[f64], c: &FreqCurve) -> f64 {
    let mut worst = 0.0f64;
    for ((&f, &g), &r) in freqs.iter().zip(got).zip(&c.mag_db) {
        if r < -100.0 || !(20.0..=20_000.0).contains(&f) {
            continue;
        }
        let e = (g - r).abs();
        assert!(e < 0.01, "{tag} @ {f} Hz: got {g:.4} want {r:.4}");
        worst = worst.max(e);
    }
    worst
}

#[test]
fn every_eq_slot_uses_the_gain_scale_mix_law() {
    let s = Schema::get();
    let keys: Vec<&str> = s
        .groups()
        .iter()
        .map(|g| g.key)
        .filter(|k| k.ends_with("/eq"))
        .collect();
    assert!(keys.len() >= 5, "{keys:?}");
    for key in keys {
        let mut sl = slot(key, 44_100.0);
        sl.set_wrapper(1, 200.0);
        assert_eq!(sl.wrapper().1, 125.0, "{key}: EQ mix goes to 125 %");
    }
    // other block kinds keep dry/wet with mix <= 100 %
    let mut d = slot("ch/*/dyn", 44_100.0);
    d.set_wrapper(1, 200.0);
    assert_eq!(d.wrapper().1, 100.0);
}

#[test]
fn alternate_eqs_mix_law_44k1_vs_wing_edit() {
    let Some(refs) = FreqCurves::load("44k1/eq-alt.json").expect("readable") else {
        eprintln!("SKIP: 44k1/eq-alt.json absent");
        return;
    };
    let fs = refs.fs;
    let mut worst = 0.0f64;
    for c in &refs.curves {
        let kind = AltKind::from_id(&c.case).unwrap();
        let key = if kind == AltKind::Pia {
            "bus/*/eq"
        } else {
            "ch/*/eq"
        };
        let mut s = slot(key, fs);
        s.select(model_index(key, kind.id()));
        for (i, (k, en)) in kind.slots().iter().enumerate() {
            let v = match (c.params.get(k), en) {
                (Some(RefParam::Num(v)), _) => *v as f32,
                (Some(RefParam::Str(t)), Some(tokens)) => {
                    tokens.iter().position(|x| x == t).unwrap() as f32
                }
                other => panic!("{k}: {other:?}"),
            };
            s.set_target(i as u16, v);
        }
        let mix = c.params.num("mix").unwrap() as f32;
        let got = slot_response(&mut s, mix, fs, &refs.freq_hz);
        worst = worst.max(check(
            &format!("{} mix {mix}", c.case),
            &got,
            &refs.freq_hz,
            c,
        ));
    }
    eprintln!(
        "alt EQ slots @44.1k: {} curves, worst {worst:.5} dB",
        refs.curves.len()
    );
}

#[test]
fn std_eq_mix_law_44k1_vs_wing_edit() {
    for (file, key, bands) in [
        ("44k1/eq-std-ch.json", "ch/*/eq", 4),
        ("44k1/eq-std-aux.json", "aux/*/eq", 4),
        ("44k1/eq-std-bus.json", "bus/*/eq", 6),
    ] {
        let Some(refs) = FreqCurves::load(file).expect("readable") else {
            eprintln!("SKIP: {file} absent");
            return;
        };
        let fs = refs.fs;
        let mut n = 0;
        let mut worst = 0.0f64;
        for c in refs.curves.iter().filter(|c| c.case.ends_with("-mix")) {
            let mut s = slot(key, fs);
            s.select(model_index(key, "STD"));
            // STD slots: lg lf lq leq, g f q per band, hg hf hq heq
            let mut keys: Vec<String> = ["lg", "lf", "lq", "leq"].map(String::from).to_vec();
            for b in 1..=bands {
                keys.extend(["g", "f", "q"].map(|x| format!("{b}{x}")));
            }
            keys.extend(["hg", "hf", "hq", "heq"].map(String::from));
            for (i, k) in keys.iter().enumerate() {
                let v = match c.params.get(k) {
                    Some(RefParam::Num(v)) => *v as f32,
                    Some(RefParam::Str(t)) => {
                        let ty = BandType::from_key(t).unwrap();
                        BandType::ALL.iter().position(|&x| x == ty).unwrap() as f32
                    }
                    None => panic!("{k}"),
                };
                s.set_target(i as u16, v);
            }
            let mix = c.params.num("mix").unwrap() as f32;
            let got = slot_response(&mut s, mix, fs, &refs.freq_hz);
            worst = worst.max(check(&format!("{file} mix {mix}"), &got, &refs.freq_hz, c));
            n += 1;
        }
        assert!(n >= 4, "{file}: {n} mix curves");
        eprintln!("{file} STD slot mix curves: {n}, worst {worst:.5} dB");
    }
}
