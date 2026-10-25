//! T072: consolidated fidelity suite. Runs every model that has WING-Edit-derived reference data against it
//! (`data/reference-curves/*.json`), collects the per-model worst error and writes the report
//! `docs/fidelity-report.md`.
//!
//! * `cargo test -p wing-blocks --test fidelity_all`: the fast sampled set (part of `make check`).
//! * `make check-full` additionally runs the `#[ignore]`d full-grid variant (every reference curve).
//! * `make fidelity-report` runs the full grids in release and regenerates `docs/fidelity-report.md`
//!   (`WING_FIDELITY_REPORT=<path>` makes either variant write the report there; by default it goes to
//!   `$CARGO_TARGET_TMPDIR/fidelity-report.md`).
//!
//! Source-of-truth classes: *exact port* (WING-Edit's own coefficient code, traced biquads match to float32
//! rounding), *reference curve* (law recovered from WING-Edit's own curve, e.g. static dynamics laws),
//! *published spec* (manufacturer-published behaviour of the emulated hardware; WING's real behaviour
//! unmeasured) and *inferred* (assumption, see `docs/VERIFY-ON-HARDWARE.md`). Models without reference data
//! carry no measured error; their status is UNVERIFIED until `tools/verify-hardware` has been run on a console.

use serde_json::Value;
use std::collections::BTreeMap;
use std::path::PathBuf;
use wing_blocks::dynamics::{static_gain_db, DynMode};
use wing_blocks::eq::{BandType, EqVariant};
use wing_blocks::fidelity::{
    cascade_db, magnitude_response, reference_dir, static_curve, Deviation, DynStatic, FreqCurves,
    RefParams,
};
use wing_blocks::gate::{GateParams, GateRatio};
use wing_blocks::model::{catalog, create_model, BlockKind, Model, ParamValue};
use wing_blocks::models::eq_alt::{prims, AltKind, Layout, MAX_PARAMS, MAX_SECS};
use wing_blocks::models::fx_eqdyn::{geq, spkman};
use wing_blocks::std_models::{dyn_slot, flt_slot, gate_slot, FilterModel};
use wing_blocks::wingcoef;

/// Sampling of the heavy (model-driven) grids: fast set vs full.
#[derive(Clone, Copy)]
struct Mode {
    /// Take every `thin`-th curve of the model-driven STD EQ / dynamics grids.
    thin: usize,
}
const FAST: Mode = Mode { thin: 70 };
const FULL: Mode = Mode { thin: 1 };

const FS: f64 = 48_000.0;
const IR: usize = 1 << 14;

// ---- measurement rows ---------------------------------------------------------------------------------------

/// One measured scope of one model.
#[derive(Clone, Debug)]
struct Meas {
    block: &'static str,
    id: String,
    scope: String,
    curves: usize,
    err: f64,
    unit: &'static str,
    tol: f64,
    at: String,
}

impl Meas {
    fn pass(&self) -> bool {
        self.err.is_finite() && self.err <= self.tol
    }
}

#[derive(Default)]
struct Out(Vec<Meas>);

impl Out {
    #[allow(clippy::too_many_arguments)]
    fn push(
        &mut self,
        block: &'static str,
        id: &str,
        scope: &str,
        curves: usize,
        err: (f64, String),
        unit: &'static str,
        tol: f64,
    ) {
        eprintln!(
            "{block}/{id} [{scope}]: {curves} curves, max err {:.6} {unit} ({})",
            err.0, err.1
        );
        self.0.push(Meas {
            block,
            id: id.into(),
            scope: scope.into(),
            curves,
            err: if err.0.is_nan() { f64::INFINITY } else { err.0 },
            unit,
            tol,
            at: err.1,
        });
    }
}

fn track(w: &mut (f64, String), e: f64, what: impl FnOnce() -> String) {
    if e > w.0 || e.is_nan() {
        *w = (e, what());
    }
}

fn load(name: &str) -> Option<FreqCurves> {
    let f = FreqCurves::load(name).expect("reference file readable");
    if f.is_none() {
        eprintln!("SKIP: {name} not present");
    }
    f
}

fn load_json(name: &str) -> Option<Value> {
    let p = reference_dir().join(name);
    match std::fs::read_to_string(&p) {
        Ok(s) => Some(serde_json::from_str(&s).expect("valid json")),
        Err(_) => {
            eprintln!("SKIP: {} not present", p.display());
            None
        }
    }
}

fn arr(v: &Value) -> Vec<f64> {
    v.as_array()
        .unwrap()
        .iter()
        .map(|x| x.as_f64().unwrap())
        .collect()
}

fn bqs(v: &Value) -> Vec<[f64; 5]> {
    v.as_array()
        .unwrap()
        .iter()
        .map(|b| {
            let n = arr(b);
            [n[0], n[1], n[2], n[3], n[4]]
        })
        .collect()
}

fn jnum(p: &Value, k: &str) -> Option<f32> {
    p.get(k).and_then(|v| v.as_f64()).map(|v| v as f32)
}

fn coef_rel(a: &[f32; 5], b: &[f64; 5]) -> f64 {
    let scale = b.iter().fold(1.0f64, |m, x| m.max(x.abs()));
    (0..5)
        .map(|i| (a[i] as f64 - b[i]).abs() / scale)
        .fold(0.0, f64::max)
}

// ---- filters ------------------------------------------------------------------------------------------------

fn configure_filter(p: &RefParams) -> FilterModel {
    let mut m = FilterModel::new();
    m.prepare(FS, 64);
    let num = |k: &str| p.num(k).unwrap_or(0.0);
    m.set_param(flt_slot::LC, ParamValue::Int(num("lc") as i32));
    m.set_param(flt_slot::HC, ParamValue::Int(num("hc") as i32));
    if let Some(f) = p.num("lcf") {
        m.set_param(flt_slot::LCF, ParamValue::Float(f as f32));
    }
    if let Some(f) = p.num("hcf") {
        m.set_param(flt_slot::HCF, ParamValue::Float(f as f32));
    }
    if let Some(s) = p.num("lcs") {
        let idx = [6.0, 12.0, 18.0, 24.0].iter().position(|&x| x == s);
        m.set_param(flt_slot::LCS, ParamValue::Enum(idx.unwrap() as u32));
    }
    if let Some(s) = p.num("hcs") {
        let idx = [6.0, 12.0].iter().position(|&x| x == s);
        m.set_param(flt_slot::HCS, ParamValue::Enum(idx.unwrap() as u32));
    }
    m
}

fn filters(out: &mut Out, mode: Mode) {
    let Some(f) = load("filter.json") else { return };
    // Channel LC/HC through the real model.
    let (mut worst, mut n) = ((0.0, String::new()), 0);
    for c in f
        .curves
        .iter()
        .filter(|c| c.params.num("tf").unwrap_or(0.0) == 0.0)
        .step_by(mode.thin.div_ceil(4))
    {
        let mut m = configure_filter(&c.params);
        let meas = magnitude_response(&mut m, f.fs, &f.freq_hz, IR);
        let d = Deviation::between(&meas, &c.mag_db, |i| {
            c.mag_db[i] > -30.0 && (20.0..=20_000.0).contains(&f.freq_hz[i])
        });
        track(&mut worst, d.max_abs_db, || {
            format!("{} @ {} Hz", c.case, f.freq_hz[d.worst])
        });
        n += 1;
    }
    out.push("filter", "LC/HC", "impulse response", n, worst, "dB", 0.02);

    // Tool filters TILT/MAX/AP1/AP2.
    let tools: [(&str, &[(u16, &str)]); 4] = [
        ("TILT", &[(1, "tilt")]),
        ("MAX", &[(1, "low"), (2, "proc")]),
        ("AP1", &[(1, "f")]),
        ("AP2", &[(1, "f"), (2, "q")]),
    ];
    for (id, slots) in tools {
        let (mut worst, mut n) = ((0.0, String::new()), 0);
        for c in f
            .curves
            .iter()
            .filter(|c| c.case == format!("tool-{id}"))
            .step_by(mode.thin.div_ceil(14))
        {
            let mut m = create_model(BlockKind::FilterTool, id);
            m.prepare(f.fs, 64);
            for &(slot, key) in slots {
                m.set_param(slot, ParamValue::Float(c.params.num(key).unwrap() as f32));
            }
            m.commit();
            m.reset();
            let meas = magnitude_response(&mut *m, f.fs, &f.freq_hz, 1 << 15);
            let d = Deviation::between(&meas, &c.mag_db, |i| f.freq_hz[i] <= 20_000.0);
            track(&mut worst, d.max_abs_db, || {
                format!("{:?} @ {} Hz", c.params.0, f.freq_hz[d.worst])
            });
            n += 1;
        }
        out.push("filter", id, "impulse response", n, worst, "dB", 0.01);
    }
}

// ---- STD EQ ---------------------------------------------------------------------------------------------------

fn std_eq(out: &mut Out, file: &str, variant: EqVariant, scope: &str, mode: Mode) {
    let Some(f) = load(file) else { return };
    let n_bands = variant.peak_bands();
    let (mut worst, mut n) = ((0.0, String::new()), 0);
    for c in f.curves.iter().step_by(mode.thin) {
        let mut m = create_model(BlockKind::Eq(variant), "STD");
        m.prepare(f.fs, 64);
        let p = &c.params;
        let ty = |k: &str| {
            p.str(k)
                .and_then(BandType::from_key)
                .map(|t| BandType::ALL.iter().position(|&x| x == t).unwrap() as u32)
        };
        let mut set = |slot: u16, key: &str| {
            if let Some(v) = p.num(key) {
                m.set_param(slot, ParamValue::Float(v as f32));
            }
        };
        set(1, "lg");
        set(2, "lf");
        set(3, "lq");
        for b in 0..n_bands {
            let base = 5 + 3 * b as u16;
            set(base, &format!("{}g", b + 1));
            set(base + 1, &format!("{}f", b + 1));
            set(base + 2, &format!("{}q", b + 1));
        }
        let h = 5 + 3 * n_bands as u16;
        set(h, "hg");
        set(h + 1, "hf");
        set(h + 2, "hq");
        set(h + 4, "tilt");
        if let Some(t) = ty("leq") {
            m.set_param(4, ParamValue::Enum(t));
        }
        if let Some(t) = ty("heq") {
            m.set_param(h + 3, ParamValue::Enum(t));
        }
        // Narrow low-frequency bands ring for seconds: a short IR would truncate them (measurement artefact).
        let low_f = c.params.0.iter().any(|(k, v)| {
            k.ends_with('f') && matches!(v, wing_blocks::fidelity::RefParam::Num(x) if *x < 60.0)
        });
        let ir = if low_f { 1 << 16 } else { IR };
        let meas = magnitude_response(&mut *m, f.fs, &f.freq_hz, ir);
        let d = Deviation::between(&meas, &c.mag_db, |i| {
            c.mag_db[i] > -60.0 && (20.0..=20_000.0).contains(&f.freq_hz[i])
        });
        track(&mut worst, d.max_abs_db, || {
            format!("{} {:?} @ {} Hz", c.case, c.params.0, f.freq_hz[d.worst])
        });
        n += 1;
    }
    out.push("eq", "STD", scope, n, worst, "dB", 0.05);
}

// ---- alternate EQs (design level, whole files) ----------------------------------------------------------

fn alt_defaults(kind: AltKind) -> [f32; MAX_PARAMS] {
    let mut p = [0.0f32; MAX_PARAMS];
    p[..kind.slots().len()].copy_from_slice(kind.defaults());
    p
}

fn alt_params(kind: AltKind, base: &[f32; MAX_PARAMS], rp: &RefParams) -> [f32; MAX_PARAMS] {
    use wing_blocks::fidelity::RefParam;
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

/// Magnitude (dB) of `n` sections, cascade or parallel (MACH4).
fn layout_db(q: &[[f64; 5]], parallel: bool, f: f64) -> f64 {
    if !parallel {
        return cascade_db(q, FS, f);
    }
    let w = 2.0 * core::f64::consts::PI * f / FS;
    let z1 = wing_dsp::Complex::new(w.cos(), -w.sin());
    let z2 = z1 * z1;
    let mut h = wing_dsp::Complex::new(0.0, 0.0);
    for c in q {
        let num = wing_dsp::Complex::new(c[0], 0.0) + z1 * c[1] + z2 * c[2];
        let den = wing_dsp::Complex::new(1.0, 0.0) + z1 * c[3] + z2 * c[4];
        h = h + num / den;
    }
    20.0 * h.norm().max(1e-30).log10()
}

fn to64(q: &[prims::Bq]) -> Vec<[f64; 5]> {
    q.iter().map(|s| s.map(f64::from)).collect()
}

fn alt_eqs(out: &mut Out) {
    let ctx = prims::Ctx::new(FS);
    for kind in AltKind::ALL {
        let name = format!("eq-{}.json", kind.id().to_lowercase());
        let Some(refs) = load(&name) else { continue };
        let (mut worst, mut n) = ((0.0, String::new()), 0);
        // Replay in file order: the extractor only sets each grid's keys on top of the previous state.
        let mut p = alt_defaults(kind);
        for c in &refs.curves {
            p = alt_params(kind, &p, &c.params);
            let mut q = [prims::UNITY; MAX_SECS];
            let layout: Layout = kind.design(&ctx, &p[..kind.slots().len()], 1.0, &mut q);
            let secs = to64(&q[..layout.n]);
            for (k, (&f, &r)) in refs.freq_hz.iter().zip(&c.mag_db).enumerate() {
                if r < -100.0 {
                    continue;
                }
                let e = (layout_db(&secs, layout.parallel, f) - r).abs();
                track(&mut worst, e, || format!("{} f={f} (#{k})", c.case));
            }
            n += 1;
        }
        out.push(
            "eq",
            kind.id(),
            "design vs traced magnitude",
            n,
            worst,
            "dB",
            0.01,
        );
    }
}

// ---- dynamics -----------------------------------------------------------------------------------------------

fn ratio_index(s: &str) -> u32 {
    ["1:1.5", "1:2", "1:3", "1:4", "gate"]
        .iter()
        .position(|&r| r == s)
        .unwrap() as u32
}

fn dynamics(out: &mut Out, mode: Mode) {
    // Static laws (analytic, whole file).
    if let Some(d) = load_json("dyn-static.json") {
        let input = arr(&d["input_db"]);
        for model in ["GATE", "DUCK", "COMP", "EXP"] {
            let (mut worst, mut n) = ((0.0, String::new()), 0);
            for c in d["models"][model]["curves"].as_array().unwrap() {
                let p = &c["params"];
                let outs = arr(&c["out_db"]);
                let thr = jnum(p, "thr").unwrap();
                for (&x, &y) in input.iter().zip(&outs) {
                    let (got, want) = match model {
                        "GATE" | "DUCK" => {
                            let gp = GateParams {
                                threshold_db: thr,
                                range_db: jnum(p, "range").unwrap(),
                                ratio: p
                                    .get("ratio")
                                    .and_then(|v| v.as_str())
                                    .and_then(GateRatio::from_snap)
                                    .unwrap_or(GateRatio::Gate),
                                ..if model == "GATE" {
                                    GateParams::gate_default()
                                } else {
                                    GateParams::duck_default()
                                }
                            };
                            ((x as f32 + gp.static_gain_db(x as f32)) as f64, y)
                        }
                        _ => {
                            let m = if model == "COMP" {
                                DynMode::Compressor
                            } else {
                                DynMode::Expander
                            };
                            let knee = jnum(p, "knee").unwrap();
                            let g = x as f32
                                + static_gain_db(
                                    m,
                                    x as f32,
                                    thr,
                                    jnum(p, "ratio").unwrap(),
                                    knee * wing_blocks::dynamics::KNEE_DB_PER_STEP,
                                );
                            // WING's EXP has no range; the block floors at -EXP_MAX_RANGE_DB.
                            let want = if model == "EXP" {
                                y.max(x - wing_blocks::dynamics::EXP_MAX_RANGE_DB as f64)
                            } else {
                                y
                            };
                            (g as f64, want)
                        }
                    };
                    track(&mut worst, (got - want).abs(), || format!("{p} in {x} dB"));
                }
                n += 1;
            }
            out.push("dyn", model, "static law (analytic)", n, worst, "dB", 0.01);
        }
    }

    // Static curves through the real Model (DC settled), GATE/COMP/EXP.
    if let Ok(Some(ds)) = DynStatic::load() {
        let thin = mode.thin.div_ceil(7).max(1);
        let pick: Vec<usize> = (0..ds.input_db.len()).step_by(thin.min(4)).collect();
        let levels: Vec<f64> = pick.iter().map(|&i| ds.input_db[i]).collect();
        for (model, tol) in [("GATE", 0.02), ("COMP", 0.02), ("EXP", 0.2)] {
            let (mut worst, mut n) = ((0.0, String::new()), 0);
            for c in ds.curves(model).iter().step_by(5 * thin) {
                let mut m = create_model(BlockKind::Dynamics, model);
                m.prepare(FS, 64);
                let p = &c.params;
                if model == "GATE" {
                    m.set_param(
                        gate_slot::THR,
                        ParamValue::Float(p.num("thr").unwrap() as f32),
                    );
                    m.set_param(
                        gate_slot::RANGE,
                        ParamValue::Float(p.num("range").unwrap() as f32),
                    );
                    m.set_param(
                        gate_slot::RATIO,
                        ParamValue::Enum(ratio_index(p.str("ratio").unwrap())),
                    );
                } else {
                    m.set_param(
                        dyn_slot::THR,
                        ParamValue::Float(p.num("thr").unwrap() as f32),
                    );
                    m.set_param(
                        dyn_slot::RATIO,
                        ParamValue::Float(p.num("ratio").unwrap() as f32),
                    );
                    m.set_param(
                        dyn_slot::KNEE,
                        ParamValue::Int(p.num("knee").unwrap() as i32),
                    );
                    m.set_param(dyn_slot::AUTO, ParamValue::Int(0));
                    m.set_param(dyn_slot::DET, ParamValue::Enum(0));
                }
                let measured = static_curve(&mut *m, FS, &levels, 1.0);
                let refc: Vec<f64> = pick.iter().map(|&i| c.out_db[i]).collect();
                let d = Deviation::between(&measured, &refc, |i| {
                    (-80.0..=0.0).contains(&levels[i]) && refc[i] > -100.0
                });
                track(&mut worst, d.max_abs_db, || {
                    format!("{:?} @ in {} dB", p.0, levels[d.worst])
                });
                n += 1;
            }
            out.push(
                "dyn",
                model,
                "static curve (processed DC)",
                n,
                worst,
                "dB",
                tol,
            );
        }
    }

    // Key filter (dynsc/gatesc) and crossover (dynxo): coefficient + magnitude, whole file.
    if let Some(d) = load_json("dyn-sidechain-xover.json") {
        use wing_blocks::sidechain::{KeyFilterKind, XoverKind, XoverParams};
        let freqs = arr(&d["freq_hz"]);
        let c48 = wingcoef::Ctx::new(FS);
        let (mut wc, mut wm, mut n) = ((0.0, String::new()), (0.0, String::new()), 0);
        let mut one = |s: [f32; 5], c: &Value, what: String| {
            let refs = bqs(&c["biquads"]);
            track(&mut wc, coef_rel(&s, &refs[0]), || what.clone());
            for (&f, &r) in freqs.iter().zip(arr(&c["mag_db"]).iter()) {
                if r > -100.0 {
                    let e = (wingcoef::response(FS, &[s], f).db() - r).abs();
                    track(&mut wm, e, || format!("{what} @ {f} Hz"));
                }
            }
        };
        for c in d["sidechain"]["curves"].as_array().unwrap() {
            let p = &c["params"];
            let kind = match p["type"].as_str().unwrap() {
                "LP12" => KeyFilterKind::Lp12,
                "HP12" => KeyFilterKind::Hp12,
                "BP" => KeyFilterKind::Bp,
                "NOTCH" => KeyFilterKind::Notch,
                _ => KeyFilterKind::Off,
            };
            let s = wing_blocks::sidechain::key_filter_coeffs(
                &c48,
                kind,
                jnum(p, "f").unwrap_or(1000.0),
                jnum(p, "q").unwrap_or(1.0),
            );
            one(s, c, format!("dynsc {p}"));
            n += 1;
        }
        for c in d["dyn_crossover"]["curves"].as_array().unwrap() {
            let p = &c["params"];
            let kind = match p["type"].as_str().unwrap() {
                "LO" => XoverKind::Lo,
                "HI" => XoverKind::Hi,
                "S6" => XoverKind::S6,
                "S12" => XoverKind::S12,
                "PEAK" => XoverKind::Peak,
                _ => XoverKind::Off,
            };
            let xp = XoverParams {
                kind,
                freq: jnum(p, "f").unwrap_or(1000.0),
                q: jnum(p, "q").unwrap_or(XoverParams::default().q),
                depth_db: jnum(p, "depth").unwrap_or(-30.0),
            };
            let s = wing_blocks::sidechain::xover_coeffs(&c48, &xp);
            one(s, c, format!("dynxo {p}"));
            n += 1;
        }
        out.push(
            "dyn",
            "KEYFILTER/XOVER",
            "coefficients (rel)",
            n,
            wc,
            "rel",
            2e-5,
        );
        out.push("dyn", "KEYFILTER/XOVER", "magnitude", n, wm, "dB", 0.01);
    }
}

// ---- FX EQ / GEQ / SPKMAN (fx-eqdyn.json) ---------------------------------------------------------------

fn db_vs_ref(a: &[[f64; 5]], b: &[[f64; 5]], parallel: bool) -> f64 {
    let mut worst = 0.0f64;
    let mut f = 20.0;
    while f < 20_000.0 {
        worst = worst.max((layout_db(a, parallel, f) - layout_db(b, parallel, f)).abs());
        f *= 1.05;
    }
    worst
}

fn fx_eqdyn(out: &mut Out) {
    let Some(o) = load_json("fx-eqdyn.json") else {
        return;
    };
    // GEQ.
    {
        let c48 = wingcoef::Ctx::new(FS);
        let (mut wc, mut wm, mut n) = ((0.0, String::new()), (0.0, String::new()), 0);
        for c in o["geq"].as_array().unwrap() {
            let v = arr(&c["values"]);
            let r = bqs(&c["biquads"]);
            let mut g = [0.0f32; geq::BANDS];
            for (i, x) in g.iter_mut().enumerate() {
                *x = v[1 + i] as f32;
            }
            let mut o2 = [wingcoef::IDENTITY; geq::BANDS];
            geq::design(&c48, &g, v[0] != 0.0, v[32] as f32, &mut o2);
            for i in 0..geq::BANDS {
                track(&mut wc, coef_rel(&o2[i], &r[i]), || {
                    format!("{v:?} band {i}")
                });
            }
            let a: Vec<[f64; 5]> = o2.iter().map(|s| s.map(f64::from)).collect();
            track(&mut wm, db_vs_ref(&a, &r, false), || format!("{v:?}"));
            n += 1;
        }
        out.push("fx", "GEQ", "coefficients (rel)", n, wc, "rel", 1e-6);
        out.push("fx", "GEQ", "magnitude", n, wm, "dB", 1e-3);
    }
    // FX EQs (same parchg as the channel EQs, mix in slot 0).
    {
        let ctx = prims::Ctx::new(FS);
        type Worst = (f64, String);
        let mut by: BTreeMap<String, (Worst, Worst, usize)> = BTreeMap::new();
        for c in o["eq"].as_array().unwrap() {
            let id = c["model"].as_str().unwrap().to_string();
            let kind = AltKind::from_id(&id).unwrap();
            let v = arr(&c["values"]);
            let r = bqs(&c["biquads"]);
            let p: Vec<f32> = v[1..].iter().map(|&x| x as f32).collect();
            let mut q = [prims::UNITY; MAX_SECS];
            let layout = kind.design(&ctx, &p, (v[0] as f32) / 100.0, &mut q);
            let a = to64(&q[..layout.n]);
            let r = &r[..layout.n];
            let e = by.entry(id).or_default();
            track(&mut e.1, db_vs_ref(&a, r, layout.parallel), || {
                format!("{v:?}")
            });
            for (x, y) in q[..layout.n].iter().zip(r) {
                track(&mut e.0, coef_rel(x, y), || format!("{v:?}"));
            }
            e.2 += 1;
        }
        for (id, (wc, wm, n)) in by {
            out.push("fx", &id, "coefficients (rel)", n, wc, "rel", 1e-5);
            out.push("fx", &id, "magnitude", n, wm, "dB", 1e-3);
        }
    }
    // SPKMAN linear part + delays.
    {
        let c48 = wingcoef::Ctx::new(FS);
        let (mut wc, mut wd, mut n) = ((0.0, String::new()), (0.0, String::new()), 0);
        for c in o["spkman"].as_array().unwrap() {
            let v: Vec<f32> = arr(&c["values"]).iter().map(|&x| x as f32).collect();
            let r = bqs(&c["biquads"]);
            let d = arr(&c["delay"]);
            let a = spkman::design_linear(&c48, &v);
            for (i, (x, y)) in a.iter().zip(&r).enumerate() {
                track(&mut wc, coef_rel(x, y), || format!("{v:?} section {i}"));
            }
            let (l, rr) = spkman::delays(FS, v[8], v[9]);
            track(
                &mut wd,
                (l as f64 - d[0]).abs().max((rr as f64 - d[1]).abs()),
                || format!("{v:?}"),
            );
            n += 1;
        }
        out.push("fx", "SPKMAN", "coefficients (rel)", n, wc, "rel", 1e-5);
        out.push("fx", "SPKMAN", "delay (samples)", n, wd, "samples", 0.0);
    }
}

// ---- source-of-truth classification ---------------------------------------------------------------------

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Source {
    ExactPort,
    ReferenceCurve,
    PublishedSpec,
    Inferred,
}

impl Source {
    fn label(self) -> &'static str {
        match self {
            Source::ExactPort => "exact port",
            Source::ReferenceCurve => "reference curve",
            Source::PublishedSpec => "published spec",
            Source::Inferred => "inferred",
        }
    }
}

/// Source of truth per model. Panics for an unclassified model so new models must be added here.
fn source_of(block: &str, id: &str) -> Source {
    use Source::*;
    match (block, id) {
        ("filter", _) => ExactPort,
        ("eq", _) => ExactPort,
        ("dyn", "GATE" | "DUCK" | "COMP" | "EXP" | "KEYFILTER/XOVER") => ReferenceCurve,
        ("dyn", "E88" | "9000G" | "D241G" | "DS902") => PublishedSpec,
        ("dyn", "B160" | "B560" | "D241C" | "9000C" | "SBUS") => PublishedSpec,
        ("dyn", "ECL33" | "E88C" | "RED3" | "2250" | "LMT") => PublishedSpec,
        ("dyn", "76LA" | "NSTR" | "LA" | "L100" | "F670" | "BLISS") => PublishedSpec,
        ("dyn", "DEQ" | "DEQ2" | "WAVE" | "PSE" | "CMB" | "RIDE" | "WARM" | "ONEC" | "3CMP") => {
            Inferred
        }
        ("fx", "GEQ" | "PIA" | "SPKMAN") => ExactPort,
        ("fx", "SOUL" | "E88" | "E84" | "F110" | "PULSAR" | "MACH4") => ExactPort,
        ("fx", "NONE" | "EXT") => Inferred,
        ("fx", _) if wing_blocks::model::FX_IDS.contains(&id) => Inferred,
        _ => panic!("fidelity_all: model {block}/{id} has no source-of-truth classification"),
    }
}

/// Every (block, id) the report lists: the model catalogs plus the shared blocks that are not catalog entries.
fn all_models() -> Vec<(&'static str, String)> {
    let mut v: Vec<(&'static str, String)> = vec![
        ("filter", "LC/HC".into()),
        ("dyn", "KEYFILTER/XOVER".into()),
    ];
    for (block, kind) in [
        ("filter", BlockKind::FilterTool),
        ("dyn", BlockKind::Dynamics),
        ("eq", BlockKind::Eq(EqVariant::Channel)),
        ("fx", BlockKind::Fx),
    ] {
        v.extend(catalog(kind).iter().map(|id| (block, id.to_string())));
    }
    v
}

// ---- report ---------------------------------------------------------------------------------------------------

fn fmt_err(m: &Meas) -> String {
    match m.unit {
        "rel" => format!("{:.2e}", m.err),
        "samples" => format!("{}", m.err),
        _ => format!("{:.4}", m.err),
    }
}

fn report(rows: &[Meas], mode_name: &str) -> String {
    let mut s = String::new();
    s.push_str("# Fidelity report\n\n");
    s.push_str(
        "Generated by `make fidelity-report` (`cargo test -p wing-blocks --release --test fidelity_all`, full \
         reference grids). Do not edit by hand. `make check` runs the same measurements on a sampled subset and \
         checks that this file lists every model.\n\n",
    );
    s.push_str(&format!("Run mode: {mode_name}.\n\n"));
    s.push_str(
        "Source of truth: **exact port** = WING-Edit's own coefficient code (traced biquads match to float32 \
         rounding); **reference curve** = law recovered from WING-Edit's own curves; **published spec** = \
         manufacturer-published behaviour of the emulated hardware, WING's real behaviour unmeasured; \
         **inferred** = assumption listed in [VERIFY-ON-HARDWARE.md](VERIFY-ON-HARDWARE.md). Status: PASS/FAIL = \
         measured against WING-Edit-derived data; UNVERIFIED = no reference data exists, only a console can \
         confirm it (`tools/verify-hardware/`).\n\n",
    );

    s.push_str("## Per model\n\n| Block | Model | Source of truth | Max error | Status |\n|---|---|---|---|---|\n");
    let mut n = (0, 0, 0);
    for (block, id) in all_models() {
        let ms: Vec<&Meas> = rows
            .iter()
            .filter(|m| m.block == block && m.id == id)
            .collect();
        let src = source_of(block, &id);
        // EQ STD rows exist per variant; alt EQ ids also appear in the FX catalog under block "fx".
        let (err, status) = if ms.is_empty() {
            n.2 += 1;
            ("n/a".to_string(), "UNVERIFIED")
        } else {
            let ok = ms.iter().all(|m| m.pass());
            if ok {
                n.0 += 1
            } else {
                n.1 += 1
            }
            (
                ms.iter()
                    .map(|m| format!("{} {}", fmt_err(m), m.unit))
                    .collect::<Vec<_>>()
                    .join("; "),
                if ok { "PASS" } else { "FAIL" },
            )
        };
        s.push_str(&format!(
            "| {block} | `{id}` | {} | {err} | {status} |\n",
            src.label()
        ));
    }
    s.push_str(&format!(
        "\n{} models PASS, {} FAIL, {} UNVERIFIED (no reference data).\n\n",
        n.0, n.1, n.2
    ));

    s.push_str("## Measurements\n\n| Block | Model | Scope | Curves | Max error | Tolerance | Worst case |\n|---|---|---|---|---|---|---|\n");
    for m in rows {
        let at = m.at.replace('|', "/");
        let at: String = at.chars().take(90).collect();
        s.push_str(&format!(
            "| {} | `{}` | {} | {} | {} {} | {} | {} |\n",
            m.block,
            m.id,
            m.scope,
            m.curves,
            fmt_err(m),
            m.unit,
            if m.unit == "rel" {
                format!("{:.0e}", m.tol)
            } else {
                format!("{}", m.tol)
            },
            at
        ));
    }
    s
}

// ---- driver ---------------------------------------------------------------------------------------------------

fn run(mode: Mode, mode_name: &str) {
    let mut out = Out::default();
    filters(&mut out, mode);
    std_eq(
        &mut out,
        "eq-std-ch.json",
        EqVariant::Channel,
        "channel, impulse response",
        mode,
    );
    std_eq(
        &mut out,
        "eq-std-aux.json",
        EqVariant::Aux,
        "aux, impulse response",
        mode,
    );
    std_eq(
        &mut out,
        "eq-std-bus.json",
        EqVariant::Bus,
        "bus, impulse response",
        mode,
    );
    alt_eqs(&mut out);
    dynamics(&mut out, mode);
    fx_eqdyn(&mut out);

    // Alt EQ ids appear in both the EQ and FX catalogs; make the FX rows (design vs file) visible for the
    // channel versions' FX twin as well (same code path, FX rows measured in fx_eqdyn()).
    let text = report(&out.0, mode_name);
    // Only the full-grid run may overwrite the committed report.
    let path = std::env::var_os("WING_FIDELITY_REPORT")
        .filter(|_| mode.thin == FULL.thin)
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join("fidelity-report.md"));
    std::fs::write(&path, &text).unwrap_or_else(|e| panic!("write {}: {e}", path.display()));
    eprintln!("fidelity report written to {}", path.display());

    let failed: Vec<String> = out
        .0
        .iter()
        .filter(|m| !m.pass())
        .map(|m| {
            format!(
                "{}/{} [{}]: {} {} > {} ({})",
                m.block, m.id, m.scope, m.err, m.unit, m.tol, m.at
            )
        })
        .collect();
    assert!(
        failed.is_empty(),
        "fidelity failures:\n{}",
        failed.join("\n")
    );
    // The shared groups must have produced something (guards against silently skipped files when the
    // reference data is present).
    if reference_dir().join("eq-std-ch.json").exists() {
        assert!(out.0.len() > 40, "only {} measurements", out.0.len());
    }
}

#[test]
fn fidelity_all_sampled() {
    run(
        FAST,
        "sampled (STD EQ and processed dynamics thinned, everything else whole files)",
    );
}

#[test]
#[ignore = "slow: run with make check-full"]
fn fidelity_all_full() {
    run(FULL, "full grids");
}

/// The committed `docs/fidelity-report.md` must mention every model (catches new models that were never
/// classified or never regenerated into the report).
#[test]
fn committed_report_lists_every_model() {
    let p = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../docs/fidelity-report.md");
    let Ok(text) = std::fs::read_to_string(&p) else {
        panic!("{} missing: run `make fidelity-report`", p.display());
    };
    for (block, id) in all_models() {
        source_of(block, &id); // panics when unclassified
        assert!(
            text.contains(&format!("| {block} | `{id}` |")),
            "{block}/{id} missing from docs/fidelity-report.md: run `make fidelity-report`"
        );
    }
}
