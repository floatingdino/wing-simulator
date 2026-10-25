//! T046: filter-tool models (TILT, MAX, AP1, AP2) against the WING-Edit reference curves
//! (`data/reference-curves/filter.json`, cases `tool-*`): magnitude < 0.01 dB through the real `Model`
//! interface, phase vs the traced reference biquad (< 0.2 deg; the float32 reference coefficients at 100 Hz limit it).

use wing_blocks::fidelity::{magnitude_response, run_mono, Deviation, FreqCurves, RefParams};
use wing_blocks::model::{create_model, BlockKind, Model, ParamValue};

const IR: usize = 1 << 15;

fn configure(case: &str, p: &RefParams) -> Box<dyn Model> {
    let (id, slots): (&'static str, &[(u16, &str)]) = match case {
        "tool-TILT" => ("TILT", &[(1, "tilt")]),
        "tool-MAX" => ("MAX", &[(1, "low"), (2, "proc")]),
        "tool-AP1" => ("AP1", &[(1, "f")]),
        "tool-AP2" => ("AP2", &[(1, "f"), (2, "q")]),
        other => panic!("unknown case {other}"),
    };
    let mut m = create_model(BlockKind::FilterTool, id);
    m.prepare(48_000.0, 64);
    for &(slot, key) in slots {
        m.set_param(slot, ParamValue::Float(p.num(key).unwrap() as f32));
    }
    m.commit();
    m.reset();
    m
}

fn ref_phase_deg(q: &[f64; 5], w: f64) -> f64 {
    let (c1, s1, c2, s2) = (w.cos(), -w.sin(), (2.0 * w).cos(), -(2.0 * w).sin());
    let (nr, ni) = (q[0] + q[1] * c1 + q[2] * c2, q[1] * s1 + q[2] * s2);
    let (dr, di) = (1.0 + q[3] * c1 + q[4] * c2, q[3] * s1 + q[4] * s2);
    (ni.atan2(nr) - di.atan2(dr)).to_degrees()
}

fn wrap(d: f64) -> f64 {
    (d + 180.0).rem_euclid(360.0) - 180.0
}

#[test]
fn tool_filters_vs_wing_edit() {
    let Some(f) = FreqCurves::load("filter.json").expect("readable") else {
        eprintln!("SKIP: filter.json absent");
        return;
    };
    let thin = if cfg!(debug_assertions) { 2 } else { 1 };
    let mut worst: [(f64, f64, usize); 4] = [(0.0, 0.0, 0); 4];
    let ids = ["tool-TILT", "tool-MAX", "tool-AP1", "tool-AP2"];
    for c in f
        .curves
        .iter()
        .filter(|c| c.case.starts_with("tool-"))
        .enumerate()
        .filter(|(i, _)| i % thin == 0)
        .map(|(_, c)| c)
    {
        let k = ids.iter().position(|&x| x == c.case).unwrap();
        let mut m = configure(&c.case, &c.params);
        let meas = magnitude_response(&mut *m, f.fs, &f.freq_hz, IR);
        let d = Deviation::between(&meas, &c.mag_db, |i| f.freq_hz[i] <= 20_000.0);
        // Phase from the impulse response.
        m.reset();
        let mut x = vec![0.0f32; IR];
        x[0] = 1.0;
        let h: Vec<f64> = run_mono(&mut *m, &x).into_iter().map(f64::from).collect();
        let mut max_ph = 0.0f64;
        for &fr in f.freq_hz.iter().filter(|&&fr| fr <= 20_000.0) {
            let w = 2.0 * core::f64::consts::PI * fr / f.fs;
            let z = wing_dsp::dtft(&h, w);
            let ph = z.im.atan2(z.re).to_degrees();
            let r = ref_phase_deg(c.biquads.last().unwrap(), w);
            max_ph = max_ph.max(wrap(ph - r).abs());
        }
        worst[k].0 = worst[k].0.max(d.max_abs_db);
        worst[k].1 = worst[k].1.max(max_ph);
        worst[k].2 += 1;
        assert!(
            d.within(0.01),
            "{} {:?}: {:.4} dB @ {} Hz",
            c.case,
            c.params.0,
            d.max_abs_db,
            f.freq_hz[d.worst]
        );
        assert!(
            max_ph < 0.2,
            "{} {:?}: phase {max_ph:.3} deg",
            c.case,
            c.params.0
        );
    }
    for (k, id) in ids.iter().enumerate() {
        eprintln!(
            "{id}: {} curves, worst {:.5} dB, {:.4} deg",
            worst[k].2, worst[k].0, worst[k].1
        );
        assert!(worst[k].2 >= 2, "{id}: too few curves");
    }
}
