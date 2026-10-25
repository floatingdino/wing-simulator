//! T03C fidelity tests: the standard blocks against WING-Edit's own coefficients and laws
//! (`data/reference-curves/*.json`, T00A), over the full reference parameter grids.
//!
//! Two checks per frequency-domain curve: (1) coefficients: every biquad the block designs equals the traced
//! WING-Edit biquad (`[b0,b1,b2,a1,a2]`, float32) to a relative 2e-5; (2) magnitude: the block's analytic
//! response (what it processes) matches `mag_db` to [`TOL_DB`] wherever the reference is above -100 dB.
//! Dynamics: the static curves of GATE/DUCK/COMP/EXP equal WING-Edit's `respf` output to [`TOL_DB`].
//! Each test prints its worst case; tests skip when the reference file is absent.
//!
//! T04J: the same checks run at fs = 44.1 kHz against `44k1/*.json` (WING-Edit emulated with its engine sample rate
//! patched to 44.1 kHz, `tools/curve-extract/curves44k1.py`), whose STD grids include EQ `mix` 0..125 %.

use serde_json::Value;
use std::path::PathBuf;
use wing_blocks::dynamics::{static_gain_db, DynMode};
use wing_blocks::eq::{BandParams, BandType, EqParams, EqVariant, StdEq};
use wing_blocks::filter::{FilterBlock, FilterParams, HcSlope, LcSlope};
use wing_blocks::gate::{GateParams, GateRatio};
use wing_blocks::wingcoef::{self, Bq};

/// Magnitude tolerance (dB).
const TOL_DB: f64 = 0.01;
/// Coefficient tolerance (relative to max(1, |ref|)).
const TOL_COEF: f64 = 2e-5;
const FS: f64 = 48_000.0;

/// Sample rate of a reference file (`fs`, 48 kHz if absent).
fn file_fs(d: &Value) -> f64 {
    d["fs"].as_f64().unwrap_or(FS)
}

fn load(name: &str) -> Option<Value> {
    let dir = std::env::var_os("WING_REFERENCE_CURVES")
        .map(PathBuf::from)
        .unwrap_or_else(|| {
            PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../data/reference-curves")
        });
    let p = dir.join(name);
    match std::fs::read_to_string(&p) {
        Ok(s) => Some(serde_json::from_str(&s).expect("valid json")),
        Err(_) => {
            eprintln!("SKIP: {} not present", p.display());
            None
        }
    }
}

fn arr_f64(v: &Value) -> Vec<f64> {
    v.as_array()
        .unwrap()
        .iter()
        .map(|x| x.as_f64().unwrap())
        .collect()
}

fn biquads(c: &Value) -> Vec<[f64; 5]> {
    c["biquads"]
        .as_array()
        .unwrap()
        .iter()
        .map(|b| {
            let v = arr_f64(b);
            [v[0], v[1], v[2], v[3], v[4]]
        })
        .collect()
}

fn num(p: &Value, k: &str) -> Option<f32> {
    p.get(k).and_then(|v| v.as_f64()).map(|v| v as f32)
}

fn coef_err(a: &Bq, b: &[f64; 5]) -> f64 {
    a.iter()
        .zip(b)
        .map(|(&x, &y)| (x as f64 - y).abs() / y.abs().max(1.0))
        .fold(0.0, f64::max)
}

/// Worst-case tracker.
#[derive(Default)]
struct Worst {
    db: f64,
    db_at: String,
    coef: f64,
    coef_at: String,
    n: usize,
}

impl Worst {
    fn db(&mut self, e: f64, what: impl FnOnce() -> String) {
        if e > self.db {
            self.db = e;
            self.db_at = what();
        }
    }
    fn coef(&mut self, e: f64, what: impl FnOnce() -> String) {
        if e > self.coef {
            self.coef = e;
            self.coef_at = what();
        }
    }
    fn report(&self, name: &str) {
        eprintln!(
            "{name}: {} curves, max |dB| err {:.5} ({}), max coef err {:.2e} ({})",
            self.n, self.db, self.db_at, self.coef, self.coef_at
        );
        assert!(self.n > 0, "{name}: no curves");
        assert!(
            self.db < TOL_DB,
            "{name}: magnitude error {:.5} dB at {}",
            self.db,
            self.db_at
        );
        assert!(
            self.coef < TOL_COEF,
            "{name}: coefficient error {:.2e} at {}",
            self.coef,
            self.coef_at
        );
    }
}

fn mag_err(freqs: &[f64], reference: &[f64], got: impl Fn(f64) -> f64) -> (f64, f64) {
    let mut worst = (0.0f64, 0.0f64);
    for (&f, &r) in freqs.iter().zip(reference) {
        // Audio band only: below 20 Hz a near-identity float32 section is hypersensitive (1 ulp of a
        // coefficient = 0.02 dB at 10 Hz for a 50 Hz shelf), so sub-audio points test rounding, not the law.
        if r < -100.0 || !(20.0..=20_000.0).contains(&f) {
            continue;
        }
        let e = (got(f) - r).abs();
        if e > worst.0 {
            worst = (e, f);
        }
    }
    worst
}

// ---- channel filter --------------------------------------------------------------------------------------

#[test]
fn filter_lc_hc_match_wing_edit() {
    filter_lc_hc("filter.json");
}

#[test]
fn filter_lc_hc_match_wing_edit_44k1() {
    filter_lc_hc("44k1/filter.json");
}

fn filter_lc_hc(file: &str) {
    let Some(d) = load(file) else { return };
    let fs = file_fs(&d);
    let freqs = arr_f64(&d["freq_hz"]);
    let mut w = Worst::default();
    for c in d["curves"].as_array().unwrap() {
        let p = &c["params"];
        if num(p, "tf").unwrap_or(0.0) != 0.0 {
            continue; // filter tool (TILT/MAX/AP1/AP2): T046
        }
        let lcs = match p.get("lcs").and_then(|v| v.as_str()) {
            Some("6") => LcSlope::Db6,
            Some("12") => LcSlope::Db12,
            Some("18") => LcSlope::Db18,
            _ => LcSlope::Db24,
        };
        let hcs = match p.get("hcs").and_then(|v| v.as_str()) {
            Some("6") => HcSlope::Db6,
            _ => HcSlope::Db12,
        };
        let fp = FilterParams {
            lc_on: num(p, "lc").unwrap_or(0.0) != 0.0,
            lc_freq: num(p, "lcf").unwrap_or(100.0),
            lc_slope: lcs,
            hc_on: num(p, "hc").unwrap_or(0.0) != 0.0,
            hc_freq: num(p, "hcf").unwrap_or(10_000.0),
            hc_slope: hcs,
        };
        let mut b = FilterBlock::new();
        b.set_params(&fp);
        b.prepare(fs, 64);
        let refs = biquads(c);
        let (secs, n) = b.wing_sections();
        let tag = || format!("{} {}", c["case"], p);
        for (k, s) in secs[..n].iter().enumerate() {
            w.coef(coef_err(s, &refs[k]), || format!("{} sec {k}", tag()));
        }
        let (e, f) = mag_err(&freqs, &arr_f64(&c["mag_db"]), |f| b.magnitude_db(f));
        w.db(e, || format!("{} @ {f} Hz", tag()));
        w.n += 1;
    }
    w.report(&format!("filter LC/HC ({file})"));
}

// ---- STD EQ ----------------------------------------------------------------------------------------------

fn eq_params(variant: EqVariant, p: &Value) -> EqParams {
    let mut e = EqParams {
        on: true,
        ..EqParams::defaults(variant)
    };
    let band = |b: &mut BandParams, pre: &str| {
        if let Some(v) = num(p, &format!("{pre}g")) {
            b.gain_db = v;
        }
        if let Some(v) = num(p, &format!("{pre}f")) {
            b.freq = v;
        }
        if let Some(v) = num(p, &format!("{pre}q")) {
            b.q = v;
        }
    };
    band(&mut e.low, "l");
    band(&mut e.high, "h");
    for (i, b) in e.bands.iter_mut().enumerate() {
        band(b, &(i + 1).to_string());
    }
    let ty = |k: &str| {
        p.get(k)
            .and_then(|v| v.as_str())
            .and_then(BandType::from_key)
    };
    if let Some(t) = ty("leq") {
        e.low_type = t;
    }
    if let Some(t) = ty("heq") {
        e.high_type = t;
    }
    if let Some(t) = num(p, "tilt") {
        e.tilt_db = t;
    }
    if let Some(m) = num(p, "mix") {
        e.mix = m;
    }
    e
}

/// Which WING sections a case defines (the others hold stale parameters in the export).
fn defined_sections(variant: EqVariant, case: &str, p: &Value) -> Vec<usize> {
    let bus = variant == EqVariant::Bus;
    let n = variant.peak_bands();
    let off = usize::from(bus);
    let hi = off + 1 + n;
    let scale = num(p, "mix").unwrap_or(100.0) / 100.0;
    let mut v = Vec::new();
    if case.starts_with("mid") {
        for b in 1..=n {
            if num(p, &format!("{b}g")).unwrap_or(0.0) * scale != 0.0 {
                v.push(off + b);
            }
        }
    } else if case.starts_with("l-") {
        v.push(0);
        if bus {
            v.push(1);
        }
        if case == "l-BW48" || case == "l-LR48" {
            v.extend([2, 3]);
        }
    } else if case.starts_with("h-") {
        v.push(hi);
        if matches!(
            case,
            "h-LR12" | "h-BW18" | "h-BW24" | "h-BS24" | "h-LR24" | "h-BW48" | "h-LR48"
        ) {
            v.push(hi - 1);
        }
        if case == "h-BW48" || case == "h-LR48" {
            v.extend([hi - 2, hi - 3]);
        }
    } else if case == "tilt" {
        v.push(hi + 1);
    }
    v
}

fn std_eq(file: &str, variant: EqVariant) {
    let Some(d) = load(file) else { return };
    let fs = file_fs(&d);
    let freqs = arr_f64(&d["freq_hz"]);
    let mut w = Worst::default();
    for c in d["curves"].as_array().unwrap() {
        let p = &c["params"];
        let case = c["case"].as_str().unwrap();
        let mut eq = StdEq::new(variant);
        eq.set_params(&eq_params(variant, p));
        eq.prepare(fs, 64);
        let refs = biquads(c);
        let (secs, n) = eq.wing_sections();
        assert_eq!(n, refs.len(), "section count");
        let tag = || format!("{case} {p}");
        let defined = defined_sections(variant, case, p);
        for &k in &defined {
            w.coef(coef_err(&secs[k], &refs[k]), || {
                format!("{} sec {k}", tag())
            });
        }
        // The 0 dB sections of the export hold stale/near-identity float32 designs (WING designs a 0 dB PEQ at
        // -0.001 dB, and float32 rounding of a near-identity 20 Hz section is worth ~0.02 dB at 10 Hz); the block
        // bypasses 0 dB bands exactly, so their reference contribution is removed from `mag_db`.
        let stale: Vec<[f32; 5]> = (0..refs.len())
            .filter(|k| !defined.contains(k))
            .map(|k| refs[k].map(|v| v as f32))
            .collect();
        let want: Vec<f64> = freqs
            .iter()
            .zip(arr_f64(&c["mag_db"]))
            .map(|(&f, m)| m - wingcoef::response(fs, &stale, f).db())
            .collect();
        // Parameters are exported as printed read-backs; a Cpar's float can be 1 ulp from the printed value
        // (e.g. "6.0" is 6.0000005 on the gain grid, worth 0.015 dB at 20 Hz for a 50 Hz shelf). Accept the
        // best of the printed gains and the gains nudged by +-1 ulp.
        let mut best = (f64::INFINITY, 0.0);
        for d in [0i32, 1, -1] {
            let mut ep = eq_params(variant, p);
            let nudge = |g: &mut f32| {
                if *g != 0.0 {
                    *g = f32::from_bits((g.to_bits() as i32 + d * g.signum() as i32) as u32);
                }
            };
            nudge(&mut ep.low.gain_db);
            nudge(&mut ep.high.gain_db);
            for b in &mut ep.bands {
                nudge(&mut b.gain_db);
            }
            eq.set_params(&ep);
            let r = mag_err(&freqs, &want, |f| eq.magnitude_db(f));
            if r.0 < best.0 {
                best = r;
            }
        }
        let (e, f) = best;
        w.db(e, || format!("{} @ {f} Hz", tag()));
        w.n += 1;
    }
    w.report(file);
}

#[test]
fn std_eq_channel_matches_wing_edit() {
    std_eq("eq-std-ch.json", EqVariant::Channel);
}

#[test]
fn std_eq_aux_matches_wing_edit() {
    std_eq("eq-std-aux.json", EqVariant::Aux);
}

#[test]
fn std_eq_bus_matches_wing_edit() {
    std_eq("eq-std-bus.json", EqVariant::Bus);
}

#[test]
fn std_eq_44k1_with_mix_matches_wing_edit() {
    std_eq("44k1/eq-std-ch.json", EqVariant::Channel);
    std_eq("44k1/eq-std-aux.json", EqVariant::Aux);
    std_eq("44k1/eq-std-bus.json", EqVariant::Bus);
}

// ---- dynamics key filter (dynsc/gatesc) and crossover (dynxo) ---------------------------------------------

#[test]
fn sidechain_filter_and_dynxo_match_wing_edit() {
    use wing_blocks::sidechain::{KeyFilterKind, XoverKind, XoverParams};
    let Some(d) = load("dyn-sidechain-xover.json") else {
        return;
    };
    let freqs = arr_f64(&d["freq_hz"]);
    let c48 = wingcoef::Ctx::new(FS);
    let mut w = Worst::default();
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
            num(p, "f").unwrap_or(1000.0),
            num(p, "q").unwrap_or(1.0),
        );
        let refs = biquads(c);
        w.coef(coef_err(&s, &refs[0]), || format!("dynsc {p}"));
        let (e, f) = mag_err(&freqs, &arr_f64(&c["mag_db"]), |f| {
            wingcoef::response(FS, &[s], f).db()
        });
        w.db(e, || format!("dynsc {p} @ {f} Hz"));
        w.n += 1;
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
            freq: num(p, "f").unwrap_or(1000.0),
            q: num(p, "q").unwrap_or(XoverParams::default().q),
            // The export leaves `depth` at -30 dB (not listed in params; visible in the curves).
            depth_db: num(p, "depth").unwrap_or(-30.0),
        };
        let s = wing_blocks::sidechain::xover_coeffs(&c48, &xp);
        let refs = biquads(c);
        w.coef(coef_err(&s, &refs[0]), || format!("dynxo {p}"));
        let (e, f) = mag_err(&freqs, &arr_f64(&c["mag_db"]), |f| {
            wingcoef::response(FS, &[s], f).db()
        });
        w.db(e, || format!("dynxo {p} @ {f} Hz"));
        w.n += 1;
    }
    w.report("dynsc + dynxo");
}

// ---- dynamics static laws ----------------------------------------------------------------------------------

#[test]
fn dynamics_static_laws_match_wing_edit() {
    let Some(d) = load("dyn-static.json") else {
        return;
    };
    let input = arr_f64(&d["input_db"]);
    for model in ["GATE", "DUCK", "COMP", "EXP"] {
        let mut w = Worst::default();
        for c in d["models"][model]["curves"].as_array().unwrap() {
            let p = &c["params"];
            let out = arr_f64(&c["out_db"]);
            let thr = num(p, "thr").unwrap();
            for (&x, &y) in input.iter().zip(&out) {
                let got = match model {
                    "GATE" | "DUCK" => {
                        let gp = GateParams {
                            threshold_db: thr,
                            range_db: num(p, "range").unwrap(),
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
                        x as f32 + gp.static_gain_db(x as f32)
                    }
                    _ => {
                        let mode = if model == "COMP" {
                            DynMode::Compressor
                        } else {
                            DynMode::Expander
                        };
                        let knee = num(p, "knee").unwrap();
                        x as f32
                            + static_gain_db(
                                mode,
                                x as f32,
                                thr,
                                num(p, "ratio").unwrap(),
                                knee * wing_blocks::dynamics::KNEE_DB_PER_STEP,
                            )
                    }
                };
                // EXP has no range in WING; the block floors the gain at -EXP_MAX_RANGE_DB (WING's -144 dB
                // silence sentinel, which GATE "oo" uses too), so compare against the floored law.
                let want = if model == "EXP" {
                    y.max(x - wing_blocks::dynamics::EXP_MAX_RANGE_DB as f64)
                } else {
                    y
                };
                let e = (got as f64 - want).abs();
                w.db(e, || format!("{p} in {x} dB: got {got} want {want}"));
            }
            w.n += 1;
        }
        w.report(model);
    }
}
