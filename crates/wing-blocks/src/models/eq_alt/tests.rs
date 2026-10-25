use super::prims::{Bq, Ctx};
use super::*;
use crate::fidelity::{self, json, FreqCurves, RefParam, RefParams};
use crate::model::{create_model, BlockKind, Model};

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
        let layout = kind.design(&ctx, &p[..kind.slots().len()], 1.0, &mut q);
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
            if dmax > 1e-4 {
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
    let _ = create_model(BlockKind::Eq(crate::eq::EqVariant::Bus), "PIA");
}
