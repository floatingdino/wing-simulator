use super::prims::{Bq, Ctx};
use super::*;
use crate::fidelity::{self, json, FreqCurves, RefParam, RefParams};
use crate::model::{create_model, BlockKind, Model, ParamValue};

const FS: f64 = 48_000.0;

/// Model defaults as a slot array.
fn defaults(kind: AltKind) -> [f32; MAX_PARAMS] {
    let mut p = [0.0f32; MAX_PARAMS];
    p[..kind.slots().len()].copy_from_slice(kind.defaults());
    p
}

/// Slot values (enums as index): reference/oracle parameters applied on top of `base`.
fn params_from(kind: AltKind, base: &[f32; MAX_PARAMS], rp: &RefParams) -> [f32; MAX_PARAMS] {
    let mut p = *base;
    for (i, (key, en)) in kind.slots().iter().enumerate() {
        match (rp.get(key), en) {
            (Some(RefParam::Str(s)), Some(tokens)) => {
                p[i] = tokens
                    .iter()
                    .position(|t| t.eq_ignore_ascii_case(s))
                    .unwrap_or_else(|| panic!("{key}: unknown token {s}"))
                    as f32
            }
            (Some(RefParam::Num(v)), _) => p[i] = *v as f32,
            (Some(RefParam::Str(s)), None) => p[i] = s.parse().expect("number"),
            (None, _) => {}
        }
    }
    p
}

fn layout_db(q: &[Bq], layout: Layout, f: f64) -> f64 {
    let qs: Vec<[f64; 5]> = q[..layout.n].iter().map(|s| s.map(f64::from)).collect();
    if !layout.parallel {
        return fidelity::cascade_db(&qs, FS, f);
    }
    let w = 2.0 * core::f64::consts::PI * f / FS;
    let z1 = wing_dsp::Complex::new(w.cos(), -w.sin());
    let z2 = z1 * z1;
    let mut h = wing_dsp::Complex::new(0.0, 0.0);
    for c in &qs {
        let num = wing_dsp::Complex::new(c[0], 0.0) + z1 * c[1] + z2 * c[2];
        let den = wing_dsp::Complex::new(1.0, 0.0) + z1 * c[3] + z2 * c[4];
        h = h + num / den;
    }
    20.0 * h.norm().max(1e-30).log10()
}

fn ref_db(q: &[[f64; 5]], parallel: bool, f: f64) -> f64 {
    let qq: Vec<Bq> = q.iter().map(|s| s.map(|x| x as f32)).collect();
    layout_db(
        &qq,
        Layout {
            n: qq.len(),
            parallel,
        },
        f,
    )
}

/// Max |design - reference| (dB) over every curve of `eq-<model>.json`; `None` if the file is absent.
fn max_error(kind: AltKind) -> Option<(f64, String)> {
    let name = format!("eq-{}.json", kind.id().to_lowercase());
    let refs = FreqCurves::load(&name).expect("reference file")?;
    let ctx = Ctx::new(refs.fs);
    let mut worst = (0.0f64, String::new());
    // The extractor sets only each grid's keys on top of the previous state, so replay the file in order
    // (e.g. E84's HF curves were taken with the LF selector left at the last LF grid value).
    let mut p = defaults(kind);
    for c in &refs.curves {
        p = params_from(kind, &p, &c.params);
        let mut q = [prims::UNITY; MAX_SECS];
        let layout = kind.design(&ctx, &p[..kind.slots().len()], 1.0, &mut q);
        assert_eq!(layout.parallel, kind == AltKind::Mach4);
        for (k, (&f, &r)) in refs.freq_hz.iter().zip(&c.mag_db).enumerate() {
            if r < -100.0 {
                continue;
            }
            let e = (layout_db(&q, layout, f) - r).abs();
            if e > worst.0 || e.is_nan() {
                worst = (
                    if e.is_nan() { f64::INFINITY } else { e },
                    format!("{} {:?} f={f} ref={r} (#{k})", c.case, c.params),
                );
            }
        }
    }
    Some(worst)
}

fn check_model(kind: AltKind) {
    if let Some((e, at)) = max_error(kind) {
        eprintln!("{}: max |err| = {e:.6} dB at {at}", kind.id());
        assert!(e < 0.01, "{}: {e} dB at {at}", kind.id());
    }
}

#[test]
fn fidelity_soul() {
    check_model(AltKind::Soul);
}

#[test]
fn fidelity_e88() {
    check_model(AltKind::E88);
}

#[test]
fn fidelity_e84() {
    check_model(AltKind::E84);
}

#[test]
fn fidelity_f110() {
    check_model(AltKind::F110);
}

#[test]
fn fidelity_pulsar() {
    check_model(AltKind::Pulsar);
}

#[test]
fn fidelity_mach4() {
    check_model(AltKind::Mach4);
}

#[test]
fn fidelity_pia() {
    check_model(AltKind::Pia);
}

/// Development oracle: `WING_EQ_ALT_ORACLE=<json>` (list of `{model, params, sections:[{bq}]}` dumped from the
/// emulated WING-Edit at random settings) compares every design. Skipped when unset.
#[test]
fn oracle_random_settings() {
    let Some(path) = std::env::var_os("WING_EQ_ALT_ORACLE") else {
        return;
    };
    let only = std::env::var("WING_EQ_ALT_ONLY").ok();
    let text = std::fs::read_to_string(path).unwrap();
    let v = json::parse(&text).unwrap();
    let ctx = Ctx::new(FS);
    let freqs: Vec<f64> = (0..270)
        .map(|i| 10.0 * 2f64.powf(i as f64 / 24.0))
        .collect();
    let mut worst: Vec<(String, f64, String)> = Vec::new();
    for e in v.as_arr().unwrap() {
        let id = e.get("model").unwrap().as_str().unwrap();
        if only
            .as_deref()
            .is_some_and(|o| !o.split(',').any(|x| x == id))
        {
            continue;
        }
        let kind = AltKind::from_id(id).unwrap();
        let rp = params_json(e.get("params").unwrap());
        let p = params_from(kind, &defaults(kind), &rp);
        let secs: Vec<[f64; 5]> = e
            .get("sections")
            .unwrap()
            .as_arr()
            .unwrap()
            .iter()
            .map(|s| {
                let b: Vec<f64> = s
                    .get("bq")
                    .unwrap()
                    .as_arr()
                    .unwrap()
                    .iter()
                    .map(|x| x.as_f64().unwrap())
                    .collect();
                [b[0], b[1], b[2], b[3], b[4]]
            })
            .collect();
        let mut q = [prims::UNITY; MAX_SECS];
        let m = rp.num("mix").map_or(1.0, |x| (x / 100.0) as f32);
        let layout = kind.design(&ctx, &p[..kind.slots().len()], m, &mut q);
        let mut err = 0.0f64;
        let mut wf = 0.0;
        for &f in &freqs {
            let r = ref_db(&secs, layout.parallel, f);
            if r < -100.0 {
                continue;
            }
            let d = (layout_db(&q, layout, f) - r).abs();
            if d > err || d.is_nan() {
                err = if d.is_nan() { 1e9 } else { d };
                wf = f;
            }
        }
        let mut coef = String::new();
        for (k, s) in secs.iter().enumerate() {
            let m = q.get(k).copied().unwrap_or(prims::UNITY);
            let dmax = (0..5)
                .map(|j| (m[j] as f64 - s[j]).abs())
                .fold(0.0, f64::max);
            if dmax > 1e-4 || std::env::var_os("WING_EQ_ALT_VERBOSE").is_some() {
                coef += &format!(" s{k}: ours {m:?} ref {s:?};");
            }
        }
        let tag = format!("{rp:?} @{wf}Hz{coef}");
        match worst.iter_mut().find(|w| w.0 == id) {
            Some(w) if w.1 >= err => {}
            Some(w) => {
                w.1 = err;
                w.2 = tag;
            }
            None => worst.push((id.to_string(), err, tag)),
        }
    }
    let mut fail = false;
    for (id, e, tag) in &worst {
        eprintln!("ORACLE {id}: max {e:.6} dB  {tag}");
        fail |= *e > 0.01;
    }
    assert!(!fail);
}

/// Development oracle for the `Cbiquad` primitives: replays every traced primitive call of the oracle file
/// with its exact float arguments and reports the max ulp distance of our coefficients.
#[test]
fn oracle_primitives() {
    let Some(path) = std::env::var_os("WING_EQ_ALT_ORACLE") else {
        return;
    };
    let v = json::parse(&std::fs::read_to_string(path).unwrap()).unwrap();
    let ctx = Ctx::new(FS);
    let mut worst: Vec<(String, i64, String)> = Vec::new();
    for e in v.as_arr().unwrap() {
        let secs = e.get("sections").unwrap().as_arr().unwrap();
        for c in e.get("calls").unwrap().as_arr().unwrap() {
            let fname = c.get("fn").unwrap().as_str().unwrap();
            let dst = c.get("dst").unwrap().as_f64().unwrap();
            let s: Vec<f32> = c
                .get("s")
                .unwrap()
                .as_arr()
                .unwrap()
                .iter()
                .map(|x| x.as_f64().unwrap() as f32)
                .collect();
            let Some(sec) = secs
                .iter()
                .find(|x| x.get("addr").unwrap().as_f64().unwrap() == dst)
            else {
                continue;
            };
            // Section 0 of E84/F110/PIA is rescaled by the output gain after the primitive call.
            let model = e.get("model").unwrap().as_str().unwrap();
            if matches!(model, "E84" | "F110" | "PIA")
                && secs[0].get("addr").unwrap().as_f64().unwrap() == dst
            {
                continue;
            }
            let r: Vec<f32> = sec
                .get("bq")
                .unwrap()
                .as_arr()
                .unwrap()
                .iter()
                .map(|x| x.as_f64().unwrap() as f32)
                .collect();
            let ours = match fname {
                f if f.starts_with("5hqpeq") => prims::hqpeq(&ctx, s[0], s[1], s[2]),
                f if f.starts_with("6ls12q2") => prims::ls12q2(&ctx, s[0], s[1], s[2], s[3]),
                f if f.starts_with("6hs12q2") => prims::hs12q2(&ctx, s[0], s[1], s[2], s[3]),
                _ => continue,
            };
            let ulps = (0..5)
                .map(|k| ((ours[k].to_bits() as i64) - (r[k].to_bits() as i64)).abs())
                .max()
                .unwrap();
            let tag = format!("args {s:?} ours {ours:?} ref {r:?}");
            match worst.iter_mut().find(|w| w.0 == fname) {
                Some(w) if w.1 >= ulps => {}
                Some(w) => {
                    w.1 = ulps;
                    w.2 = tag;
                }
                None => worst.push((fname.to_string(), ulps, tag)),
            }
        }
    }
    for (f, u, t) in &worst {
        eprintln!("PRIM {f}: max {u} ulp  {t}");
    }
}

fn params_json(v: &json::Value) -> RefParams {
    let mut out = Vec::new();
    if let json::Value::Obj(kv) = v {
        for (k, x) in kv {
            let p = match x {
                json::Value::Num(n) => RefParam::Num(*n),
                json::Value::Bool(b) => RefParam::Num(*b as u8 as f64),
                json::Value::Str(s) => RefParam::Str(s.clone()),
                _ => continue,
            };
            out.push((k.clone(), p));
        }
    }
    RefParams(out)
}

#[test]
fn model_ids_and_slots() {
    for k in AltKind::ALL {
        assert_eq!(AltKind::from_id(k.id()), Some(k));
        assert_eq!(k.defaults().len(), k.slots().len());
        assert!(k.slots().len() <= MAX_PARAMS);
        let m = AltEq::new(k);
        assert_eq!(m.id(), k.id());
        assert!(!m.is_stub());
    }
    let pia = create_model(BlockKind::Eq(crate::eq::EqVariant::Bus), "PIA");
    assert_eq!(pia.id(), "PIA");
}

/// A non-trivial setting per model (slot values).
fn busy(kind: AltKind) -> Vec<f32> {
    match kind {
        AltKind::Soul => vec![3.0, 4.0, 6.0, 1.0, 7.0, -3.5, 4.0, 1.0, 2.0, 3.0, 6.0, -2.0],
        AltKind::E88 => vec![
            4.0, 3.0, 1.0, 0.0, 3.0, -4.0, 8.0, 5.0, 2.5, 2.0, 7.0, 4.0, 1.0, 1.0,
        ],
        AltKind::E84 => vec![-3.0, 2.0, 4.0, 3.0, -2.5, 1.0, 1.0, 3.5],
        AltKind::F110 => vec![
            1.0, 6.0, 4.0, 3.0, 0.0, 4.0, -3.0, 7.0, 1.0, 1.0, 2.0, 3.0, 3.0, -4.0, 2.0,
        ],
        AltKind::Pulsar => vec![
            1.0, 6.0, 4.0, 1.0, 3.0, 7.0, 4.0, 5.0, 1.0, 1.0, 5.0, 2.0, 4.0, 6.0, 3.0, 2.0,
        ],
        AltKind::Mach4 => vec![2.0, -3.0, 4.0, -1.0, 3.0, 6.0, 3.0, 1.0],
        AltKind::Pia => vec![-2.0, 6.0, -4.0, 2.0, 8.0, -12.0, 3.0, 5.0, -6.0, 9.0, 4.0],
    }
}

fn design_db(kind: AltKind, fs: f64, p: &[f32], f: f64) -> f64 {
    let mut q = [prims::UNITY; MAX_SECS];
    let ctx = Ctx::new(fs);
    let layout = kind.design(&ctx, p, 1.0, &mut q);
    let w = 2.0 * core::f64::consts::PI * f / fs;
    let z1 = wing_dsp::Complex::new(w.cos(), -w.sin());
    let z2 = z1 * z1;
    let one = wing_dsp::Complex::new(1.0, 0.0);
    let mut h = if layout.parallel {
        wing_dsp::Complex::new(0.0, 0.0)
    } else {
        one
    };
    for s in &q[..layout.n] {
        let c = s.map(f64::from);
        let hs = (wing_dsp::Complex::new(c[0], 0.0) + z1 * c[1] + z2 * c[2])
            / (one + z1 * c[3] + z2 * c[4]);
        h = if layout.parallel { h + hs } else { h * hs };
    }
    h.db()
}

/// The audio path (f64 TDF-II, parallel bank for MACH4) realises the designed response, at 48 and 44.1 kHz.
#[test]
fn processed_response_matches_design() {
    let freqs = [100.0, 1000.0, 5000.0, 12000.0];
    for fs in [48_000.0, 44_100.0] {
        for kind in AltKind::ALL {
            let p = busy(kind);
            let mut m = AltEq::new(kind);
            m.prepare(fs, 64);
            for (i, &v) in p.iter().enumerate() {
                m.set_param(i as u16 + 1, ParamValue::Float(v));
            }
            m.reset();
            let got = fidelity::magnitude_response(&mut m, fs, &freqs, 1 << 15);
            for (&f, g) in freqs.iter().zip(&got) {
                let want = design_db(kind, fs, &p, f);
                assert!(
                    want.is_finite() && want.abs() < 40.0,
                    "{} {f}: {want}",
                    kind.id()
                );
                assert!(
                    (g - want).abs() < 0.01,
                    "{} fs={fs} f={f}: {g} vs {want}",
                    kind.id()
                );
            }
        }
    }
}

/// Knob glides and enum switches stay finite and bounded, then settle exactly on the target design.
#[test]
fn glides_and_switches_settle() {
    for kind in AltKind::ALL {
        let mut m = AltEq::new(kind);
        m.prepare(48_000.0, 64);
        let p = busy(kind);
        let (mut l, mut r) = (vec![0.0f32; 64], vec![0.0f32; 64]);
        let mut n = 0usize;
        for blk in 0..400 {
            if blk == 10 {
                for (i, &v) in p.iter().enumerate() {
                    m.set_param(i as u16 + 1, ParamValue::Float(v));
                }
            }
            for (k, (x, y)) in l.iter_mut().zip(r.iter_mut()).enumerate() {
                let s = (((n + k) as f32) * 0.05).sin() * 0.25;
                *x = s;
                *y = -s;
            }
            n += 64;
            m.process(&mut l, &mut r, None);
            assert!(
                l.iter().chain(&r).all(|v| v.is_finite() && v.abs() < 20.0),
                "{} block {blk}",
                kind.id()
            );
        }
        for (i, &v) in p.iter().enumerate() {
            assert_eq!(m.param(i as u16 + 1), Some(v));
        }
        for f in [200.0, 3000.0] {
            let want = design_db(kind, 48_000.0, &p, f);
            assert!((m.magnitude_db(f) - want).abs() < 1e-6, "{} {f}", kind.id());
        }
        m.load_defaults();
        m.reset();
        assert!(m.magnitude_db(1000.0).abs() < 0.5, "{} defaults", kind.id());
    }
}

/// T04J: every model at fs = 44.1 kHz and mix 0..125 % against WING-Edit's own coefficients
/// (`44k1/eq-alt.json`, emulated with the engine sample rate patched to 44.1 kHz), configured only through the
/// `Model` interface (`set_param`, `set_gain_scale(mix %)`, `reset`): coefficients to a relative 1e-5 and the
/// runtime filter's magnitude to 0.01 dB.
#[test]
fn fidelity_44k1_with_mix() {
    let Some(refs) = FreqCurves::load("44k1/eq-alt.json").expect("reference file") else {
        eprintln!("SKIP: 44k1/eq-alt.json absent");
        return;
    };
    assert_eq!(refs.fs, 44_100.0);
    let mut worst = (0.0f64, 0.0f64, String::new());
    let mut mixes = Vec::new();
    for c in &refs.curves {
        let kind = AltKind::from_id(&c.case).unwrap();
        let p = params_from(kind, &defaults(kind), &c.params);
        let mix = c.params.num("mix").unwrap() as f32;
        if !mixes.contains(&(kind, mix as i32)) {
            mixes.push((kind, mix as i32));
        }
        let mut m = AltEq::new(kind);
        m.prepare(refs.fs, 64);
        for (i, &v) in p[..kind.slots().len()].iter().enumerate() {
            m.set_param(i as u16 + 1, ParamValue::Float(v));
        }
        Model::set_gain_scale(&mut m, mix);
        m.reset();
        let mut q = [prims::UNITY; MAX_SECS];
        let layout = m.design_now(&mut q);
        assert_eq!(layout.n, c.biquads.len(), "{} section count", c.case);
        for (k, (ours, r)) in q.iter().zip(&c.biquads).enumerate() {
            let e = ours
                .iter()
                .zip(r)
                .map(|(&x, &y)| (x as f64 - y).abs() / y.abs().max(1.0))
                .fold(0.0, f64::max);
            if e > worst.1 {
                worst.1 = e;
                worst.2 = format!("{} mix {mix} sec {k}", c.case);
            }
        }
        for (&f, &r) in refs.freq_hz.iter().zip(&c.mag_db) {
            if r < -100.0 {
                continue;
            }
            let e = (m.magnitude_db(f) - r).abs();
            assert!(
                e < 0.01,
                "{} mix {mix} {:?} @ {f} Hz: {e} dB",
                c.case,
                c.params
            );
            worst.0 = worst.0.max(e);
        }
    }
    eprintln!(
        "44.1k alt EQs: {} curves, max {:.6} dB, max coef err {:.2e} ({})",
        refs.curves.len(),
        worst.0,
        worst.1,
        worst.2
    );
    assert!(
        worst.1 < 1e-5,
        "coefficients: {:.2e} at {}",
        worst.1,
        worst.2
    );
    // every model is covered at several mix values, including 0 and 125 %
    for k in AltKind::ALL {
        let n = mixes.iter().filter(|(kk, _)| *kk == k).count();
        assert!(n >= 4, "{}: {n} mix values", k.id());
    }
}

/// `mix` glides (no zipper) and settles exactly on the scaled design; it survives `load_defaults`.
#[test]
fn gain_scale_glides_and_settles() {
    for kind in AltKind::ALL {
        let p = busy(kind);
        let mut m = AltEq::new(kind);
        m.prepare(48_000.0, 64);
        for (i, &v) in p.iter().enumerate() {
            m.set_param(i as u16 + 1, ParamValue::Float(v));
        }
        m.reset();
        Model::set_gain_scale(&mut m, 50.0);
        let (mut l, mut r) = (vec![0.0f32; 64], vec![0.0f32; 64]);
        let mut seen = Vec::new();
        for _ in 0..200 {
            m.process(&mut l, &mut r, None);
            seen.push(m.magnitude_db(3000.0));
        }
        assert!(
            seen.windows(2).filter(|w| w[0] != w[1]).count() > 3,
            "{} glides",
            kind.id()
        );
        let mut q = [prims::UNITY; MAX_SECS];
        let layout = kind.design(&Ctx::new(48_000.0), &p, 0.5, &mut q);
        let want = layout_db(&q, layout, 3000.0);
        assert!(
            (m.magnitude_db(3000.0) - want).abs() < 1e-6,
            "{} settles",
            kind.id()
        );
        assert_eq!(m.gain_scale(), 0.5);
        m.load_defaults();
        m.reset();
        assert_eq!(m.gain_scale(), 0.5, "mix survives load_defaults");
    }
}
