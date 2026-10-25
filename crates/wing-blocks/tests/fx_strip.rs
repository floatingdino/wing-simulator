//! T05C: the channel-strip FX equal the manual chain of their blocks bit-exactly; parameter tables are
//! cross-checked against `data/schema/wing-tree.json`; EXT routing model.

use wing_blocks::eq::EqVariant;
use wing_blocks::model::{create_model, BlockKind, Model, ParamValue};
use wing_blocks::models::fx_strip::{ExternalLoop, Route, StripKind, XFADE_MS};

const FS: f64 = 48_000.0;
const BLOCK: usize = 64;

fn signal(blocks: usize, seed: u32) -> (Vec<f32>, Vec<f32>) {
    let mut s = seed;
    let mut noise = move || {
        s = s.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        (s >> 8) as f32 / (1u32 << 24) as f32 - 0.5
    };
    let n = blocks * BLOCK;
    let mut l = Vec::with_capacity(n);
    let mut r = Vec::with_capacity(n);
    for i in 0..n {
        // alternating loud / quiet passages so gates close and compressors act
        let level = if (i / 2400) % 2 == 0 { 0.7 } else { 0.02 };
        let t = i as f32 / FS as f32;
        l.push(level * ((t * 440.0 * std::f32::consts::TAU).sin() + 0.3 * noise()));
        r.push(level * ((t * 660.0 * std::f32::consts::TAU).sin() + 0.3 * noise()));
    }
    (l, r)
}

fn fx(id: &'static str) -> Box<dyn Model> {
    create_model(BlockKind::Fx, id)
}
fn dynm(id: &'static str) -> Box<dyn Model> {
    create_model(BlockKind::Dynamics, id)
}
fn eqm(id: &'static str) -> Box<dyn Model> {
    create_model(BlockKind::Eq(EqVariant::Channel), id)
}

type Overrides = &'static [(u16, f32)];

fn apply(m: &mut dyn Model, ov: Overrides) {
    for &(s, v) in ov {
        m.set_param(s, ParamValue::Float(v));
    }
}

/// Run the strip and the manual chain over the same signal and compare bit-exactly.
fn compare(kind: StripKind, strip_ov: Overrides, chain: Vec<(Box<dyn Model>, Overrides)>) {
    let mut strip = create_model(BlockKind::Fx, kind.id());
    strip.prepare(FS, BLOCK);
    strip.load_defaults();
    apply(&mut *strip, strip_ov);
    strip.commit();
    strip.reset(); // settle the stage switches
    let mut chain: Vec<Box<dyn Model>> = chain
        .into_iter()
        .map(|(mut m, ov)| {
            m.prepare(FS, BLOCK);
            m.load_defaults();
            apply(&mut *m, ov);
            m.commit();
            m.reset();
            m
        })
        .collect();
    let (il, ir) = signal(160, 7);
    let (mut sl, mut sr) = (il.clone(), ir.clone());
    let (mut ml, mut mr) = (il, ir);
    for (a, b) in sl.chunks_mut(BLOCK).zip(sr.chunks_mut(BLOCK)) {
        strip.process(a, b, None);
    }
    for (a, b) in ml.chunks_mut(BLOCK).zip(mr.chunks_mut(BLOCK)) {
        for m in chain.iter_mut() {
            m.process(a, b, None);
        }
    }
    assert!(sl.iter().all(|x| x.is_finite()));
    assert_eq!(sl, ml, "{} left", kind.id());
    assert_eq!(sr, mr, "{} right", kind.id());
    let lat: usize = chain.iter().map(|m| m.latency()).sum();
    assert_eq!(strip.latency(), lat, "{} latency", kind.id());
}

#[test]
fn even_equals_chain() {
    compare(
        StripKind::Even,
        &[
            (1, -30.0),
            (2, 4.0),
            (4, 500.0),
            (5, 1.0),
            (7, 1.0),
            (8, 1.0),
            (9, 3.5),
            (10, 2.0),
            (13, 4.0),
            (14, -1.5),
            (19, 8.0),
            (20, 1.0),
            (24, 1.0),
            (25, -6.0),
            (28, 1.0),
            (29, -20.0),
            (30, 4.0),
            (33, 2.0),
        ],
        vec![
            (dynm("E88"), &[(1, -30.0), (2, 4.0), (4, 500.0), (5, 1.0)]),
            (
                eqm("E88"),
                &[
                    (1, 3.5),
                    (2, 2.0),
                    (5, 4.0),
                    (6, -1.5),
                    (11, 8.0),
                    (12, 1.0),
                ],
            ),
            (
                dynm("ECL33"),
                &[
                    (1, 1.0),
                    (2, -6.0),
                    (5, 1.0),
                    (6, -20.0),
                    (7, 4.0),
                    (10, 2.0),
                ],
            ),
        ],
    );
}

#[test]
fn soul_equals_chain() {
    compare(
        StripKind::Soul,
        &[
            (1, -28.0),
            (2, 30.0),
            (3, 100.0),
            (6, 1.0),
            (7, 1.0),
            (8, 1.0),
            (9, 4.0),
            (10, -2.0),
            (11, 5.0),
            (14, 1.5),
            (19, 9.0),
            (20, 2.0),
            (22, 1.0),
            (23, -20.0),
            (24, 4.0),
            (26, 600.0),
            (27, 1.0),
        ],
        vec![
            (
                dynm("9000G"),
                &[(1, -28.0), (2, 30.0), (3, 100.0), (6, 1.0)],
            ),
            (
                eqm("SOUL"),
                &[
                    (1, 4.0),
                    (2, -2.0),
                    (3, 5.0),
                    (6, 1.5),
                    (11, 9.0),
                    (12, 2.0),
                ],
            ),
            (dynm("9000C"), &[(1, -20.0), (2, 4.0), (4, 600.0), (5, 1.0)]),
        ],
    );
}

#[test]
fn vintage_equals_chain() {
    compare(
        StripKind::Vintage,
        &[
            (1, -24.0),
            (2, -20.0),
            (3, 3.0),
            (5, 2.0),
            (6, 1.0),
            (7, 1.0),
            (8, 4.0),
            (12, 3.0),
            (16, 1.0),
            (17, 5.0),
            (21, 2.0),
            (23, 60.0),
            (24, 30.0),
            (25, 1.0),
            (26, 1.0),
        ],
        vec![
            (dynm("76LA"), &[(1, -24.0), (2, -20.0), (3, 3.0), (5, 2.0)]),
            (
                eqm("PULSAR"),
                &[
                    (1, 1.0),
                    (2, 4.0),
                    (6, 3.0),
                    (10, 1.0),
                    (11, 5.0),
                    (15, 2.0),
                ],
            ),
            (dynm("LA"), &[(1, 60.0), (2, 30.0), (3, 1.0)]),
        ],
    );
}

#[test]
fn bus_equals_chain() {
    compare(
        StripKind::Bus,
        &[
            (1, 60.0),
            (2, 30.0),
            (3, 0.4),
            (4, -2.0),
            (5, 80.0),
            (6, 1.0),
            (7, 1.0),
            (8, 3.0),
            (10, 2.5),
            (12, -1.5),
            (15, 1.0),
            (17, -20.0),
            (18, 4.0),
            (19, 3.0),
            (20, 2.0),
            (22, 1.0),
        ],
        vec![
            (
                dynm("WARM"),
                &[(1, 60.0), (2, 30.0), (3, 0.4), (4, -2.0), (5, 80.0)],
            ),
            (
                create_model(BlockKind::Eq(EqVariant::Bus), "E84"),
                &[(1, 3.0), (3, 2.5), (5, -1.5), (8, 1.0)],
            ),
            (dynm("SBUS"), &[(1, -20.0), (2, 4.0), (3, 3.0), (4, 2.0)]),
        ],
    );
}

#[test]
fn master_equals_chain() {
    compare(
        StripKind::Master,
        &[
            (1, 6.0),
            (2, 7.5),
            (3, 1.0),
            (5, 1.0),
            (6, 2.0),
            (7, -1.0),
            (8, 1.5),
            (11, 4.0),
            (12, 3.0),
            (13, 1.0),
            (14, 40.0),
            (19, 50.0),
            (22, 20.0),
            (25, 1.0),
            (26, 6.0),
            (27, -1.0),
            (28, 30.0),
            (33, 1.0),
        ],
        vec![
            (fx("TAPE"), &[(1, 6.0), (2, 7.5), (3, 1.0)]),
            (
                eqm("MACH4"),
                &[(1, 2.0), (2, -1.0), (3, 1.5), (6, 4.0), (7, 3.0)],
            ),
            (fx("ENHANCE"), &[(1, 40.0), (6, 50.0), (11, 20.0)]),
            (fx("LIMITER"), &[(1, 6.0), (2, -1.0), (3, 30.0)]),
        ],
    );
}

#[test]
fn eq_mix_is_the_band_gain_scale() {
    let mut strip = create_model(BlockKind::Fx, "*EVEN*");
    strip.prepare(FS, BLOCK);
    strip.load_defaults();
    for (s, v) in [(8, 1.0), (10, 4.0), (20, -3.0), (23, 50.0)] {
        strip.set_param(s, ParamValue::Float(v));
    }
    strip.commit();
    strip.reset();
    let mut eq = eqm("E88");
    eq.prepare(FS, BLOCK);
    eq.load_defaults();
    eq.set_param(2, ParamValue::Float(4.0));
    eq.set_param(12, ParamValue::Float(-3.0));
    eq.set_gain_scale(50.0);
    eq.commit();
    eq.reset();
    // ECL33 with both sections off is the third stage; run it after the EQ for the reference
    let mut ecl = dynm("ECL33");
    ecl.prepare(FS, BLOCK);
    ecl.load_defaults();
    for (slot, v) in [
        (2, -5.0),
        (3, 4.0),
        (5, 0.0),
        (6, -15.0),
        (7, 3.0),
        (8, 4.0),
    ] {
        ecl.set_param(slot, ParamValue::Float(v));
    }
    ecl.commit();
    ecl.reset();
    let (mut l, mut r) = signal(40, 3);
    let (mut l2, mut r2) = (l.clone(), r.clone());
    for (a, b) in l.chunks_mut(BLOCK).zip(r.chunks_mut(BLOCK)) {
        strip.process(a, b, None);
    }
    for (a, b) in l2.chunks_mut(BLOCK).zip(r2.chunks_mut(BLOCK)) {
        eq.process(a, b, None);
        ecl.process(a, b, None);
    }
    let d = l.iter().zip(&l2).position(|(a, b)| a != b);
    assert_eq!(
        d,
        None,
        "first diff; {:?} {:?}",
        d.map(|i| l[i]),
        d.map(|i| l2[i])
    );
    assert_eq!(r, r2);
}

#[test]
fn bus_make_up_gain_follows_comp() {
    let run = |gain: f32| {
        let mut m = create_model(BlockKind::Fx, "*BUS*");
        m.prepare(FS, BLOCK);
        m.load_defaults();
        m.set_param(22, ParamValue::Float(1.0));
        m.set_param(17, ParamValue::Float(-25.0));
        m.set_param(21, ParamValue::Float(gain));
        m.commit();
        m.reset();
        let (mut l, mut r) = signal(30, 5);
        for (a, b) in l.chunks_mut(BLOCK).zip(r.chunks_mut(BLOCK)) {
            m.process(a, b, None);
        }
        l
    };
    let (a, b) = (run(0.0), run(6.0));
    let g = 10f32.powf(6.0 / 20.0);
    // first block ramps; afterwards the make-up is exact
    for i in 4 * BLOCK..a.len() {
        assert!(
            (b[i] - a[i] * g).abs() <= 1e-5 * a[i].abs().max(1e-3),
            "{i}"
        );
    }
}

#[test]
fn stage_switch_crossfades() {
    let mut m = create_model(BlockKind::Fx, "*SOUL*");
    m.prepare(FS, BLOCK);
    m.load_defaults();
    m.commit();
    m.reset();
    let (il, ir) = signal(40, 9);
    // everything off: pass-through (no latency in this chain)
    assert_eq!(m.latency(), 0);
    let (mut l, mut r) = (il.clone(), ir.clone());
    for (a, b) in l.chunks_mut(BLOCK).zip(r.chunks_mut(BLOCK)) {
        m.process(a, b, None);
    }
    assert_eq!(l, il);
    // gate on: the first output samples are still (almost) the dry signal, the fade takes ~XFADE_MS
    m.set_param(7, ParamValue::Float(1.0));
    m.set_param(1, ParamValue::Float(0.0)); // gate threshold 0 dB: closed, wet = silence
    m.commit();
    let (mut l, mut r) = (il.clone(), ir.clone());
    m.process(&mut l[..BLOCK], &mut r[..BLOCK], None);
    let step = (1.0 / (XFADE_MS * 1e-3 * FS)) as f32;
    assert!((l[0] - il[0]).abs() <= step * il[0].abs() * 1.001 + 1e-6);
    let fade = (XFADE_MS * 1e-3 * FS) as usize;
    let blocks = fade / BLOCK + 2;
    for k in 1..blocks {
        m.process(
            &mut l[k * BLOCK..(k + 1) * BLOCK],
            &mut r[k * BLOCK..(k + 1) * BLOCK],
            None,
        );
    }
    let tail = &l[(blocks - 1) * BLOCK..blocks * BLOCK];
    let gated = tail.iter().map(|x| x.abs()).fold(0.0, f32::max);
    assert!(gated < 0.05, "gate closed after the fade: {gated}");
}

#[test]
fn defaults_and_params() {
    for kind in StripKind::ALL {
        let mut m = create_model(BlockKind::Fx, kind.id());
        m.prepare(FS, BLOCK);
        m.load_defaults();
        let d = kind.defaults();
        assert_eq!(d.len(), kind.routes().len());
        for (i, &v) in d.iter().enumerate() {
            assert_eq!(
                m.param(i as u16 + 1),
                Some(v),
                "{} slot {}",
                kind.id(),
                i + 1
            );
        }
        assert_eq!(m.param(d.len() as u16 + 1), None);
        m.set_param(d.len() as u16 + 5, ParamValue::Float(1.0)); // ignored
                                                                 // defaults: every switch off (except MASTER limiter): transparent apart from the always-in stages
        assert!(!m.is_stub());
        assert_eq!(m.id(), kind.id());
    }
}

// ---- schema cross-check ---------------------------------------------------------------------------

fn schema() -> serde_json::Value {
    let p =
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../data/schema/wing-tree.json");
    serde_json::from_str(&std::fs::read_to_string(p).unwrap()).unwrap()
}

fn model_params(v: &serde_json::Value, group: &str, name: &str) -> Vec<serde_json::Value> {
    fn walk<'a>(o: &'a serde_json::Value, name: &str, out: &mut Option<&'a serde_json::Value>) {
        match o {
            serde_json::Value::Object(m) => {
                if m.get("name").and_then(|n| n.as_str()) == Some(name)
                    && m.contains_key("params")
                    && m.contains_key("index")
                    && out.is_none()
                {
                    *out = Some(o);
                    return;
                }
                m.values().for_each(|c| walk(c, name, out));
            }
            serde_json::Value::Array(a) => a.iter().for_each(|c| walk(c, name, out)),
            _ => {}
        }
    }
    let mut found = None;
    walk(&v["variants"][group], name, &mut found);
    found.unwrap_or_else(|| panic!("{group} {name}"))["params"]
        .as_array()
        .unwrap()
        .clone()
}

fn stage_source(kind: BlockKind, id: &str) -> (&'static str, String) {
    let g = match kind {
        BlockKind::Dynamics if matches!(id, "E88" | "9000G") => "ch/*/gate",
        BlockKind::Dynamics => "ch/*/dyn",
        BlockKind::Eq(_) => "ch/*/eq",
        _ => "fx/1",
    };
    (g, id.to_string())
}

/// Schema slot encoding of a default token for a parameter descriptor.
fn default_value(p: &serde_json::Value) -> f32 {
    let d = p["default"].as_str().unwrap();
    match p["type"].as_str().unwrap() {
        "enum" => p["values"]
            .as_array()
            .unwrap()
            .iter()
            .position(|v| v.as_str().unwrap().eq_ignore_ascii_case(d))
            .unwrap() as f32,
        _ => d.parse().unwrap(),
    }
}

#[test]
fn tables_match_the_schema() {
    let v = schema();
    let slots = |name: &str| model_params(&v, "fx/1", name);
    for kind in StripKind::ALL {
        let params = slots(kind.id());
        // MASTER slot 7 ("40") is absent from the exported model
        let present: Vec<usize> = params
            .iter()
            .map(|p| p["slot"].as_u64().unwrap() as usize)
            .collect();
        let expect: Vec<usize> = (1..=kind.routes().len())
            .filter(|s| !(kind == StripKind::Master && *s == 7))
            .collect();
        assert_eq!(present, expect, "{} slots", kind.id());
        let chain: Vec<_> = kind.chain().collect();
        let sub: Vec<Vec<serde_json::Value>> = chain
            .iter()
            .map(|(k, id)| {
                let (g, n) = stage_source(*k, id);
                model_params(&v, g, &n)
            })
            .collect();
        for p in &params {
            let slot = p["slot"].as_u64().unwrap() as usize;
            let name = p["name"].as_str().unwrap();
            // defaults
            let want = default_value(p);
            let got = kind.defaults()[slot - 1];
            assert!(
                (want - got).abs() <= 1e-3 * want.abs().max(1.0),
                "{} {name}: schema default {want}, table {got}",
                kind.id()
            );
            // the targeted sub-block parameter has the same type and range
            if let Route::Sub(stage, s) = kind.routes()[slot - 1] {
                let q = sub[stage as usize]
                    .iter()
                    .find(|q| q["slot"].as_u64().unwrap() as u16 == s)
                    .unwrap_or_else(|| panic!("{} {name} -> stage {stage} slot {s}", kind.id()));
                let ctx = format!(
                    "{} {name} vs {}.{}",
                    kind.id(),
                    chain[stage as usize].1,
                    q["name"]
                );
                assert_eq!(p["type"], q["type"], "{ctx} type");
                assert_eq!(p.get("values"), q.get("values"), "{ctx} values");
                for key in ["min", "max"] {
                    // log grids differ in the last digit between models
                    match (p[key].as_f64(), q[key].as_f64()) {
                        (Some(a), Some(b)) => {
                            assert!((a - b).abs() <= 1e-3 * a.abs().max(1.0), "{ctx} {key}")
                        }
                        (a, b) => assert_eq!(a, b, "{ctx} {key}"),
                    }
                }
            }
        }
    }
    // EXT
    let ext = slots("EXT");
    assert_eq!(ext.len(), 5);
    let mut m = ExternalLoop::new();
    for p in &ext {
        let slot = p["slot"].as_u64().unwrap() as u16;
        assert_eq!(m.param(slot), Some(default_value(p)), "EXT slot {slot}");
    }
    m.load_defaults();
}

// ---- EXT -------------------------------------------------------------------------------------------

fn ext_with(egrp: u32, mode: u32, trim: f32) -> Box<dyn Model> {
    let mut m = create_model(BlockKind::Fx, "EXT");
    m.prepare(FS, BLOCK);
    m.set_param(1, ParamValue::Enum(egrp));
    m.set_param(3, ParamValue::Enum(mode));
    m.set_param(5, ParamValue::Float(trim));
    m.commit();
    m.reset();
    m
}

#[test]
fn ext_off_passes_through() {
    let mut m = create_model(BlockKind::Fx, "EXT");
    m.prepare(FS, BLOCK);
    m.load_defaults();
    let (il, ir) = signal(2, 1);
    let (mut l, mut r) = (il.clone(), ir.clone());
    m.process(&mut l, &mut r, Some((&ir, &il)));
    assert_eq!((&l, &r), (&il, &ir));
}

#[test]
fn ext_returns_the_return_with_trim_and_mode() {
    let (a, b) = signal(2, 4);
    let (dl, dr) = signal(2, 8);
    // ST
    let mut m = ext_with(7, 1, 6.0);
    let (mut l, mut r) = (dl.clone(), dr.clone());
    m.process(&mut l, &mut r, Some((&a, &b)));
    let g = 10f32.powf(6.0 / 20.0);
    for i in 0..a.len() {
        assert!((l[i] - a[i] * g).abs() < 1e-6 && (r[i] - b[i] * g).abs() < 1e-6);
    }
    // M: first leg on both sides
    let mut m = ext_with(7, 0, 0.0);
    let (mut l, mut r) = (dl.clone(), dr.clone());
    m.process(&mut l, &mut r, Some((&a, &b)));
    assert_eq!(l, a);
    assert_eq!(r, a);
    // M/S decode
    let mut m = ext_with(7, 2, 0.0);
    let (mut l, mut r) = (dl.clone(), dr.clone());
    m.process(&mut l, &mut r, Some((&a, &b)));
    for i in 0..a.len() {
        assert_eq!(l[i], a[i] + b[i]);
        assert_eq!(r[i], a[i] - b[i]);
    }
    // configured loop without return audio is silent (the slot input went out on the SEND legs)
    let mut m = ext_with(7, 1, 0.0);
    let (mut l, mut r) = (dl.clone(), dr.clone());
    m.process(&mut l, &mut r, None);
    assert!(l.iter().chain(r.iter()).all(|&x| x == 0.0));
    // a shorter return reads as zero beyond its end
    let mut m = ext_with(7, 1, 0.0);
    let (mut l, mut r) = (dl.clone(), dr.clone());
    m.process(&mut l, &mut r, Some((&a[..10], &b[..10])));
    assert_eq!(&l[..10], &a[..10]);
    assert!(l[10..].iter().all(|&x| x == 0.0));
    // parameters
    let mut m = ExternalLoop::new();
    m.set_param(2, ParamValue::Float(33.0));
    m.set_param(4, ParamValue::Float(120.0));
    assert_eq!(m.input(), 33);
    assert_eq!(m.latency_param(), 120);
    assert_eq!(m.latency(), 0);
    assert_eq!(m.group(), 0);
}
