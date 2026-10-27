//! T082: WING meter-word laws (`wing_blocks::meter`) against WING-Edit's GR-meter tables
//! (`data/reference-curves/dyn-models.json`, skipped when absent), and every gate/dynamics model and every
//! FX model with state words driving its meter under gain reduction.

use wing_blocks::fidelity::{json, reference_dir};
use wing_blocks::meter::{DynMeterLaw, FxMeterLaw, MAX_METER_BANDS};
use wing_blocks::model::{create_model, BlockKind, Model, ParamValue, DYN_IDS, FX_IDS};

const FS: f64 = 48_000.0;
const BLOCK: usize = 64;

#[test]
fn dyn_laws_match_reference_file() {
    let path = reference_dir().join("dyn-models.json");
    let Ok(text) = std::fs::read_to_string(&path) else {
        eprintln!("SKIP: {} not present", path.display());
        return;
    };
    let root = json::parse(&text).expect("dyn-models.json parses");
    let models = root.get("models").and_then(|m| m.as_arr()).unwrap();
    let mut checked = 0;
    for entry in models {
        let id = entry.get("model").and_then(|v| v.as_str()).unwrap();
        // DEQ/DEQ2: the dynamic-EQ display decodes band gains, not WING-Edit's GR-meter function (module doc).
        if id == "DEQ" || id == "DEQ2" {
            continue;
        }
        let law = DynMeterLaw::for_model(id);
        let meter = entry.get("gr_meter").unwrap();
        let nums = |k: &str| -> Vec<f64> {
            meter
                .get(k)
                .and_then(|v| v.as_arr())
                .unwrap()
                .iter()
                .map(|v| v.as_f64().unwrap())
                .collect()
        };
        for (gr, m) in nums("gain_reduction_db").into_iter().zip(nums("meter")) {
            let ours = law.position(gr as f32, &[]) as f64;
            assert!(
                (ours - m).abs() <= 1e-4 + 1e-3 * m.abs(),
                "{id}: GR {gr} dB -> {ours} vs {m}"
            );
        }
        checked += 1;
    }
    assert_eq!(checked, 30);
}

/// Run `x` through `m` in blocks; after every block, the largest `word(m)`.
fn drive(m: &mut (dyn Model + 'static), x: &[f32], word: &impl Fn(&dyn Model) -> i32) -> i32 {
    let mut l = x.to_vec();
    let mut r = x.to_vec();
    let mut peak = 0;
    for (a, b) in l.chunks_mut(BLOCK).zip(r.chunks_mut(BLOCK)) {
        m.process(a, b, None);
        peak = peak.max(word(m));
    }
    peak
}

fn bands(m: &dyn Model) -> [f32; MAX_METER_BANDS] {
    let mut b = [0.0; MAX_METER_BANDS];
    // raw FX state words take precedence, as in the engine's FX meter cells
    if m.fx_meter_words(&mut b) == 0 {
        m.meter_bands(&mut b);
    }
    b
}

/// Per-model settings (1-based schema slot, value) that make the model reduce gain; defaults otherwise.
fn provoke(kind: BlockKind, id: &str) -> &'static [(u16, f32)] {
    match (kind, id) {
        (BlockKind::Dynamics, "DS902") => &[(2, 12.0)],
        (BlockKind::Dynamics, "DEQ") => &[(6, -12.0), (7, 1000.0)],
        (BlockKind::Dynamics, "DEQ2") => &[(6, -12.0), (7, 1000.0), (15, -12.0), (16, 1000.0)],
        (BlockKind::Dynamics, "3CMP") => &[(5, -40.0), (10, -40.0), (15, -40.0)],
        (BlockKind::Fx, "DE-S2") => &[(1, 30.0), (2, 30.0)],
        (BlockKind::Fx, "C5-CMB") => &[(1, -40.0)],
        (BlockKind::Fx, "DEQ3") => &[
            (6, -12.0),
            (7, 1000.0),
            (15, -12.0),
            (16, 1000.0),
            (24, -12.0),
            (25, 1000.0),
        ],
        (BlockKind::Fx, "*EVEN*") => &[(7, 1.0), (14, 1.0), (15, -35.0)],
        (BlockKind::Fx, "*SOUL*") => &[(7, 1.0), (10, 1.0), (11, -30.0)],
        (BlockKind::Fx, "*VINTAGE*") => &[(6, 1.0), (10, 1.0)],
        (BlockKind::Fx, "*BUS*") => &[(17, -40.0), (22, 1.0)],
        _ => &[],
    }
}

/// Largest meter-word movement of model `id` over the test signals: quiet (-70 dBFS 1 kHz: gates and
/// expanders close), hot 1 kHz and hot 7 kHz (-1 dBFS: compressors, limiters, ducker, de-essers).
fn movement(kind: BlockKind, id: &'static str, word: impl Fn(&dyn Model) -> i32) -> i32 {
    let n = 24_000;
    let tone = |amp: f32, f: f32| -> Vec<f32> {
        (0..n)
            .map(|i| amp * (std::f32::consts::TAU * f * i as f32 / FS as f32).sin())
            .collect()
    };
    let mut moved = 0;
    for x in [tone(3e-4, 1000.0), tone(0.9, 1000.0), tone(0.9, 7000.0)] {
        let mut m = create_model(kind, id);
        m.prepare(FS, BLOCK);
        m.load_defaults();
        for &(slot, v) in provoke(kind, id) {
            m.set_param(slot, ParamValue::Float(v));
        }
        m.commit();
        moved = moved.max(drive(&mut *m, &x, &word));
    }
    moved
}

#[test]
fn every_dyn_model_moves_its_gr_meter() {
    let mut silent = Vec::new();
    for &id in DYN_IDS.iter() {
        let law = DynMeterLaw::for_model(id);
        // compare to the law's no-reduction position (B160/SBUS/ONEC rest off zero; some deflect negatively)
        let (rest, _) = law.words(0.0, &[0.0; MAX_METER_BANDS], false);
        let moved = movement(BlockKind::Dynamics, id, |m| {
            let t = m.telemetry();
            let (w, _) = law.words(t.gain_reduction_db, &bands(m), t.engaged);
            (w as i32 - rest as i32).abs()
        });
        eprintln!("{id}: GR word moved by {moved}");
        if moved == 0 {
            silent.push(id);
        }
    }
    // WAVE / WARM: WING-Edit's GR meter is constant 1 / 0 (`dyn_gain_transient` / `dyn_gain_warmth`).
    assert_eq!(
        silent,
        ["WAVE", "WARM"],
        "models whose GR meter never moves"
    );
}

#[test]
fn fx_models_with_state_words_drive_them() {
    let mut silent = Vec::new();
    let mut checked = 0;
    for &id in FX_IDS.iter() {
        let law = FxMeterLaw::for_model(id);
        if law == FxMeterLaw::None {
            continue;
        }
        let rest = law.words(0.0, &[0.0; MAX_METER_BANDS]);
        let moved = movement(BlockKind::Fx, id, |m| {
            let w = law.words(m.telemetry().gain_reduction_db, &bands(m));
            w.iter()
                .zip(rest)
                .map(|(a, b)| (*a as i32 - b as i32).abs())
                .max()
                .unwrap_or(0)
        });
        eprintln!("{id}: state words moved by {moved}");
        if moved == 0 {
            silent.push(id);
        }
        checked += 1;
    }
    // 9 dB/band laws + PCORR, V-IMG, SUB-M, SPKMAN (raw words; DE-S2 moved from the GR law)
    assert_eq!(checked, 13);
    assert!(silent.is_empty(), "FX state words never move: {silent:?}");
}
