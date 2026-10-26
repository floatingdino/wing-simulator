//! T090 WARM calibration kit.
//!
//! * `warm_kit_generates`: the quick kit's scenes put WARM in ch 1's gate slot with the intended settings, and
//!   the capture stays under 5 minutes.
//! * `fit_warm_recovers_a_synthetic_console` (every scene, `make check-full`; `_sampled`: 3 scenes): recordings synthesised with a different WARM law (delayed, chain
//!   gain -3 dB, noise) are fitted back to within +-2 dB THD per point, the calibration file round-trips, and
//!   the uncalibrated law is shown to miss the same points.
//! * `kit_scene_renders_as_the_model` (`make check-full`): a kit scene through the whole console (`wing-render`) measures as WARM
//!   alone (the scene really hosts WARM at the settings the fit assumes).

use std::path::PathBuf;

use wing_blocks::models::dyn_special::WarmLaw;
use wing_render::load_scene;
use wing_render::verify::render_scene;
use wing_render::verify::warm::{self, Signal, FS};
use wing_sim::wav::write_pcm;

fn tmp(tag: &str) -> PathBuf {
    let d = std::env::temp_dir().join(format!("wingwarm-{tag}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&d);
    std::fs::create_dir_all(&d).unwrap();
    d
}

#[test]
fn warm_kit_generates() {
    let d = tmp("gen");
    let scenes = warm::gen(&d, true).unwrap();
    assert_eq!(scenes.len(), 8);
    let suite_s = warm::build().seconds();
    // capture time of the quick tier (suite + pad per scene) under 5 min, including ~10 s per recall
    let capture = scenes.len() as f64 * (suite_s + warm::CAPTURE_PAD_S as f64 + 10.0);
    assert!(capture < 300.0, "{capture} s");
    let manifest: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(d.join("manifest.json")).unwrap()).unwrap();
    assert_eq!(manifest["capture_pad_seconds"], warm::CAPTURE_PAD_S);
    assert!(manifest["suite_seconds"].as_f64().unwrap() >= suite_s);
    for s in &scenes {
        let (store, _) =
            load_scene(Some(&d.join("scenes").join(format!("{}.snap", s.name)))).unwrap();
        let text = |p: &str| {
            let id = store.lookup(p).unwrap();
            let mut t = String::new();
            store.format_token(id, &mut t).unwrap();
            t
        };
        let num = |p: &str| store.get_f64(store.lookup(p).unwrap()).unwrap();
        assert!(text("ch/1/in/conn/grp").starts_with("USB"), "{}", s.name);
        if s.on {
            assert_eq!(text("ch/1/gate/mdl"), "WARM", "{}", s.name);
            assert_eq!(num("ch/1/gate/on"), 1.0, "{}", s.name);
            assert!(
                (num("ch/1/gate/1") - s.drv as f64).abs() < 0.01,
                "{}",
                s.name
            );
            assert!(
                (num("ch/1/gate/2") - s.hrm as f64).abs() < 0.01,
                "{}",
                s.name
            );
            assert!((num("ch/1/gate/5") - 100.0).abs() < 0.01, "{}", s.name);
        } else {
            assert_eq!(num("ch/1/gate/on"), 0.0);
        }
    }
    assert!(warm::gen(&d.join("all"), false).unwrap().len() > scenes.len());
}

/// Every scene of the full tier (`make check-full`).
#[test]
#[ignore = "slow: run with make check-full"]
fn fit_warm_recovers_a_synthetic_console() {
    fit_synthetic("fit-all", &[]);
}

/// Sampled variant: thru, one drive at hrm 0 and the 2nd-harmonic end at full drive.
#[test]
fn fit_warm_recovers_a_synthetic_console_sampled() {
    fit_synthetic(
        "fit-sampled",
        &["warm-thru", "warm-d066", "warm-d125-hm100"],
    );
}

/// Synthesise recordings of `only` (empty: every scene) with another WARM law, fit them back and check.
fn fit_synthetic(tag: &str, only: &[&str]) {
    let d = tmp(tag);
    let rec = d.join("rec");
    std::fs::create_dir_all(&rec).unwrap();
    // a "console" whose WARM is hotter at the top, softer at the bottom, more 2nd-heavy, with another
    // make-up reference
    let truth = WarmLaw::from_points(
        &[
            (10.0, 0.35),
            (33.0, 0.9),
            (66.0, 2.6),
            (100.0, 9.0),
            (125.0, 28.0),
        ],
        0.32,
        -22.0,
    )
    .unwrap();
    let suite = warm::build();
    let mut seed = 7u32;
    for (k, s) in warm::catalog().iter().enumerate() {
        if !only.is_empty() && !only.contains(&s.name) {
            continue;
        }
        let y = warm::model_render(&truth, s, &suite);
        let lead = FS as usize + 4321 + 17 * k; // the recorder starts ~1 s early, plus device latency
        let mut out = vec![0.0f32; 2 * lead];
        for v in y.iter().chain(std::iter::repeat_n(&0.0, FS as usize * 2)) {
            seed ^= seed << 13;
            seed ^= seed >> 17;
            seed ^= seed << 5;
            let n = (seed as f32 / u32::MAX as f32 - 0.5) * 2e-5; // ~-100 dBFS noise
            out.extend([v * 0.707_9 + n, 0.0]); // chain: -3 dB (pan law), USB out 2 silent
        }
        write_pcm(&rec.join(format!("{}.wav", s.name)), 2, FS, 24, &out).unwrap();
    }
    let json = d.join("warm.json");
    let r = warm::fit_warm(&rec, &d.join("report"), Some(&json)).unwrap();
    let md = std::fs::read_to_string(d.join("report/warm-fit.md")).unwrap();
    println!("{md}");
    assert!(r.pass(), "fit fails:\n{md}");
    for c in r.checks.iter().filter(|c| c.graded) {
        assert!(c.delta().abs() <= 2.0, "{c:?}");
        // the level change through WARM is fitted too (make-up)
        assert!((c.model_gain_db - c.rec_gain_db).abs() < 1.0, "{c:?}");
    }
    // the law in use before the fit is far off on this console, so the check above is meaningful
    assert!(r
        .checks
        .iter()
        .any(|c| c.graded && c.delta_before().abs() > 2.0));
    assert!((r.law.t_max - truth.t_max).abs() < 0.03, "{:?}", r.law);
    assert!(
        (r.law.makeup_ref_dbfs_rms - truth.makeup_ref_dbfs_rms).abs() < 1.0,
        "{:?}",
        r.law
    );
    for &(drv, g) in r.law.points() {
        let t = truth.drive_gain(drv);
        assert!((g / t - 1.0).abs() < 0.1, "drive {drv}: {g} vs {t}");
    }
    // the written file is the fitted law, loadable by the model
    let back = warm::read_law(&json).unwrap();
    assert_eq!(back.points().len(), r.law.points().len());
    for (a, b) in back.points().iter().zip(r.law.points()) {
        assert!((a.1 / b.1 - 1.0).abs() < 1e-5);
    }
    let text = std::fs::read_to_string(&json).unwrap();
    assert!(text.contains("\"pass\": true"), "{text}");
}

/// The fast set covers the same path with `warm_kit_generates` (scene contents) and `tests/warm_live.rs`
/// (WARM in a gate slot through the whole console).
#[test]
#[ignore = "slow: run with make check-full"]
fn kit_scene_renders_as_the_model() {
    let d = tmp("render");
    warm::gen(&d, true).unwrap();
    let suite = warm::build();
    let s = warm::catalog()
        .into_iter()
        .find(|s| s.name == "warm-d125-hm100")
        .unwrap();
    let console = render_scene(
        &d.join("scenes").join(format!("{}.snap", s.name)),
        &d.join("suite.wav"),
        &d.join("work"),
    )
    .unwrap();
    let thru = render_scene(
        &d.join("scenes/warm-thru.snap"),
        &d.join("suite.wav"),
        &d.join("work"),
    )
    .unwrap();
    let mc = warm::measure(&suite, &console).unwrap();
    let mt = warm::measure(&suite, &thru).unwrap();
    let mm = warm::measure(&suite, &warm::model_render(WarmLaw::builtin(), &s, &suite)).unwrap();
    for ((c, m), t) in mc.points.iter().zip(&mm.points).zip(&mt.points) {
        if let Signal::Sine { .. } = c.signal {
            assert!(
                (c.thd_db - m.thd_db).abs() < 0.5,
                "{}: console {} model {}",
                c.signal.label(),
                c.thd_db,
                m.thd_db
            );
            // level through WARM, relative to the thru scene (the chain's own gain cancels)
            let (gc, gm) = (c.out_db - t.out_db, m.out_db - level(c.signal));
            assert!((gc - gm).abs() < 0.3, "{}: {gc} vs {gm}", c.signal.label());
        }
    }
}

fn level(s: Signal) -> f64 {
    match s {
        Signal::Sine { level, .. } | Signal::Imd { level } | Signal::Vocal { level } => level,
    }
}
