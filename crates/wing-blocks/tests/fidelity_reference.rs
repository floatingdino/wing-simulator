//! T045 fidelity harness against the T00A reference curves (`data/reference-curves`). Each test skips (with a
//! message) when its reference file is absent. T03C calibrated the standard models (exact WING-Edit designs and
//! laws); these impulse-response/DC measurements now assert tight tolerances. The exact coefficient-level
//! checks are in `t03c_fidelity.rs`.

use wing_blocks::eq::{BandType, EqVariant};
use wing_blocks::fidelity::{
    magnitude_response, static_curve, Deviation, DynStatic, FreqCurves, RefParams,
};
use wing_blocks::model::{create_model, BlockKind, Model, ParamValue};
use wing_blocks::std_models::{dyn_slot, flt_slot, gate_slot, FilterModel};

const IR: usize = 1 << 14;
/// Subsampling of the reference grids (debug builds run under `make check`, keep them quick).
const THIN: usize = if cfg!(debug_assertions) { 4 } else { 1 };

fn load(name: &str) -> Option<FreqCurves> {
    match FreqCurves::load(name).expect("reference file readable") {
        Some(f) => Some(f),
        None => {
            eprintln!("SKIP: data/reference-curves/{name} not present");
            None
        }
    }
}

/// Compare where the reference is above `floor_db` and inside 20 Hz .. 20 kHz.
fn compare(f: &FreqCurves, measured: &[f64], reference: &[f64], floor_db: f64) -> Deviation {
    Deviation::between(measured, reference, |i| {
        reference[i] > floor_db && (20.0..=20_000.0).contains(&f.freq_hz[i])
    })
}

fn configure_filter(p: &RefParams) -> FilterModel {
    let mut m = FilterModel::new();
    m.prepare(48_000.0, 64);
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
        let idx = [6.0, 12.0, 18.0, 24.0]
            .iter()
            .position(|&x| x == s)
            .unwrap();
        m.set_param(flt_slot::LCS, ParamValue::Enum(idx as u32));
    }
    if let Some(s) = p.num("hcs") {
        let idx = [6.0, 12.0].iter().position(|&x| x == s).unwrap();
        m.set_param(flt_slot::HCS, ParamValue::Enum(idx as u32));
    }
    m
}

#[test]
fn channel_lc_hc_vs_wing_edit() {
    let Some(f) = load("filter.json") else { return };
    // [lc-only, hc-involved]
    let mut worst = [(0.0f64, String::new()), (0.0f64, String::new())];
    let mut n = 0;
    for c in &f.curves {
        if c.params.num("tf").unwrap_or(0.0) != 0.0 {
            continue; // filter tool: T046 (stubs today)
        }
        let mut m = configure_filter(&c.params);
        let meas = magnitude_response(&mut m, f.fs, &f.freq_hz, IR);
        let d = compare(&f, &meas, &c.mag_db, -30.0);
        let k = (c.params.num("hc").unwrap_or(0.0) != 0.0) as usize;
        if d.max_abs_db > worst[k].0 {
            worst[k] = (
                d.max_abs_db,
                format!("{} {:?} @ {} Hz", c.case, c.params.0, f.freq_hz[d.worst]),
            );
        }
        n += 1;
    }
    eprintln!("filter LC: worst {:.3} dB ({})", worst[0].0, worst[0].1);
    eprintln!("filter HC: worst {:.3} dB ({})", worst[1].0, worst[1].1);
    assert!(n > 30);
    assert!(
        worst[0].0 < 0.02,
        "LC deviation {:.3} dB: {}",
        worst[0].0,
        worst[0].1
    );
    assert!(
        worst[1].0 < 0.02,
        "HC deviation {:.3} dB: {}",
        worst[1].0,
        worst[1].1
    );
}

/// STD EQ: set the slots from the reference params and compare (impulse response; the reference's near-identity
/// 0 dB sections and the finite IR leave a few hundredths of a dB at 20 Hz).
fn std_eq_report(file: &str, variant: EqVariant) {
    let Some(f) = load(file) else { return };
    let n_bands = variant.peak_bands();
    let mut devs: Vec<(f64, String)> = Vec::new();
    for c in f.curves.iter().step_by(7 * THIN) {
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
        let meas = magnitude_response(&mut *m, f.fs, &f.freq_hz, IR);
        let d = compare(&f, &meas, &c.mag_db, -60.0);
        devs.push((d.max_abs_db, format!("{} {:?}", c.case, c.params.0)));
    }
    devs.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap());
    let med = devs[devs.len() / 2].0;
    eprintln!(
        "{file}: {} curves, median max-dev {:.3} dB, worst {:.3} dB ({})",
        devs.len(),
        med,
        devs[0].0,
        devs[0].1
    );
    assert!(!devs.is_empty());
    assert!(
        devs[0].0 < 0.05,
        "{file}: worst {:.3} dB ({})",
        devs[0].0,
        devs[0].1
    );
}

#[test]
fn std_eq_ch_vs_wing_edit_report() {
    std_eq_report("eq-std-ch.json", EqVariant::Channel);
}

#[test]
fn std_eq_bus_vs_wing_edit_report() {
    std_eq_report("eq-std-bus.json", EqVariant::Bus);
}

fn ratio_index(s: &str) -> u32 {
    ["1:1.5", "1:2", "1:3", "1:4", "gate"]
        .iter()
        .position(|&r| r == s)
        .unwrap() as u32
}

/// Static laws of GATE/COMP/EXP vs WING-Edit `respf` (DC input, settled).
#[test]
fn dyn_static_vs_wing_edit() {
    let Some(ds) = DynStatic::load().expect("dyn-static.json readable") else {
        eprintln!("SKIP: data/reference-curves/dyn-static.json not present");
        return;
    };
    // Compare inside the usable range (-80 .. 0 dBFS input, outputs above -100 dB).
    let pick: Vec<usize> = (0..ds.input_db.len()).step_by(THIN).collect();
    let levels: Vec<f64> = pick.iter().map(|&i| ds.input_db[i]).collect();
    let mask = |out: &[f64]| {
        let lv = levels.clone();
        let o = out.to_vec();
        move |i: usize| (-80.0..=0.0).contains(&lv[i]) && o[i] > -100.0
    };
    let mut summary = Vec::new();
    for model in ["GATE", "COMP", "EXP"] {
        let curves = ds.curves(model);
        let mut worst = (0.0f64, String::new());
        for c in curves.iter().step_by(5 * THIN) {
            let mut m = create_model(
                BlockKind::Dynamics,
                Box::leak(model.to_string().into_boxed_str()),
            );
            m.prepare(48_000.0, 64);
            let p = &c.params;
            match model {
                "GATE" => {
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
                }
                _ => {
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
            }
            let out = static_curve(&mut *m, 48_000.0, &levels, 1.0);
            let refc: Vec<f64> = pick.iter().map(|&i| c.out_db[i]).collect();
            let d = Deviation::between(&out, &refc, mask(&refc));
            if d.max_abs_db > worst.0 {
                worst = (
                    d.max_abs_db,
                    format!("{:?} @ in {} dB", p.0, levels[d.worst]),
                );
            }
        }
        eprintln!(
            "{model} static: {} curves, worst {:.3} dB ({})",
            curves.len(),
            worst.0,
            worst.1
        );
        summary.push((model, worst.0));
    }
    // GATE/COMP match WING-Edit's law to measurement precision; EXP's steep slopes (up to 1:100 below the
    // threshold) amplify the detector's settling residue.
    assert!(
        summary[0].1 < 0.02,
        "GATE static deviation {:.3} dB",
        summary[0].1
    );
    assert!(
        summary[1].1 < 0.02,
        "COMP static deviation {:.3} dB",
        summary[1].1
    );
    assert!(
        summary[2].1 < 0.2,
        "EXP static deviation {:.3} dB",
        summary[2].1
    );
}
