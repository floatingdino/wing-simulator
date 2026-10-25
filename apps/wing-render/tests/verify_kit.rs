//! T072 hardware verification kit.
//!
//! * `kit_generates_every_scene`: every scene's parameters exist and the kit files are written (fast).
//! * `compare_aligns_and_flags_synthetic`: the suite itself stands in for the render; a fake recording
//!   (delayed, stereo, padded) must align to the frame and show ~0 deviation, and gain / distortion / slower
//!   envelope perturbations must be flagged by the right metrics (fast).
//! * `compare_real_renders` (`make check-full`): the same with real `wing-render` output of a few scenes.

use std::path::{Path, PathBuf};

use wing_render::verify::scenes::catalog;
use wing_render::verify::suite::{self, Kind, FS};
use wing_render::verify::{compare, gen, read_recording, render_scene, Metric, SceneReport};
use wing_sim::wav::write_pcm;

fn tmp(tag: &str) -> PathBuf {
    let d = std::env::temp_dir().join(format!("wingverify-{tag}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&d);
    std::fs::create_dir_all(&d).unwrap();
    d
}

/// A fake recording of `reference`: `delay` frames of lead-in, `f` applied to the whole signal, 8 s of tail,
/// stereo (channel 2 silent). Returns channel 1 as `read_recording` sees it.
fn fake_recording(
    path: &Path,
    reference: &[f32],
    delay: usize,
    f: impl Fn(&[f32]) -> Vec<f32>,
) -> Vec<f32> {
    let mut mono = vec![0.0f32; delay];
    mono.extend(f(reference));
    mono.extend(std::iter::repeat_n(0.0, FS as usize * 8));
    let st: Vec<f32> = mono.iter().flat_map(|&v| [v, 0.0]).collect();
    write_pcm(path, 2, FS, 24, &st).unwrap();
    read_recording(path).unwrap()
}

fn metric<'a>(r: &'a SceneReport, kind: Kind, needle: &str) -> &'a Metric {
    r.metrics
        .iter()
        .find(|m| m.kind == kind && m.name.contains(needle))
        .unwrap_or_else(|| panic!("no metric {needle}"))
}

fn scene(name: &str) -> wing_render::verify::scenes::SceneDef {
    catalog().into_iter().find(|d| d.name == name).unwrap()
}

#[test]
fn kit_generates_every_scene() {
    let d = tmp("gen");
    let defs = gen(&d.join("all"), false).unwrap();
    assert_eq!(defs.len(), catalog().len());
    for def in &defs {
        let p = d.join("all/scenes").join(format!("{}.snap", def.name));
        let text = std::fs::read_to_string(&p).unwrap();
        assert!(text.starts_with("{\"type\":\"snapshot."), "{}", def.name);
    }
    assert!(d.join("all/suite.wav").exists() && d.join("all/manifest.json").exists());
    let quick = gen(&d.join("quick"), true).unwrap();
    assert!((10..defs.len()).contains(&quick.len()));
    std::fs::remove_dir_all(&d).ok();
}

#[test]
fn compare_aligns_and_flags_synthetic() {
    let d = tmp("syn");
    let suite = suite::build();
    // every kind in one scene definition
    let mut def = scene("00-thru");
    def.kinds = vec![
        Kind::Fr,
        Kind::Stairs,
        Kind::Steps,
        Kind::Bursts,
        Kind::Tone,
    ];
    let reference = suite.samples.clone();
    let delay = (0.3704 * FS as f64) as usize + 5;
    let rec_path = d.join("r.wav");

    let rec = fake_recording(&rec_path, &reference, delay, |x| x.to_vec());
    let r = compare(&def, &suite, &reference, &rec).unwrap();
    assert!(
        (r.latency_ms - delay as f64 / 48.0).abs() < 0.03,
        "latency {}",
        r.latency_ms
    );
    assert!(r.drift_frames.abs() < 1.0 && r.align_score > 0.99);
    assert!(r.ok());
    for m in &r.metrics {
        assert!(
            m.value.abs() < 0.1,
            "{} {:+.3} at {} (drift {})",
            m.name,
            m.value,
            m.at,
            r.drift_frames
        );
    }

    // +3 dB: level metrics flag it, shape metrics do not
    let g = 10f32.powf(3.0 / 20.0);
    let rec = fake_recording(&rec_path, &reference, delay, |x| {
        x.iter().map(|v| v * g).collect()
    });
    let r = compare(&def, &suite, &reference, &rec).unwrap();
    let m = metric(&r, Kind::Stairs, "mean offset");
    assert!(!m.ok() && (m.value - 3.0).abs() < 0.1, "{}", m.value);
    assert!(!metric(&r, Kind::Tone, "1000 Hz: level").ok());
    assert!(metric(&r, Kind::Stairs, "shape").ok());

    // saturation: harmonics flagged
    let rec = fake_recording(&rec_path, &reference, delay, |x| {
        x.iter().map(|v| (v * 3.0).tanh() / 3.0).collect()
    });
    let r = compare(&def, &suite, &reference, &rec).unwrap();
    assert!(!metric(&r, Kind::Tone, "1000 Hz: worst harmonic").ok());
    assert!(!metric(&r, Kind::Tone, "THD").ok());

    // a slow one-pole smoothing of the signal (a 'slower' detector stand-in): step settling times flagged
    let rec = fake_recording(&rec_path, &reference, delay, |x| {
        let (mut y, a) = (0.0f32, 0.001f32);
        x.iter()
            .map(|v| {
                y += a * (v.abs() - y);
                v * (0.2 + y * 0.0) + y * 0.5 * v.signum()
            })
            .collect()
    });
    let r = compare(&def, &suite, &reference, &rec).unwrap();
    assert!(!r.ok());

    // too-short recording is reported, not a panic
    let short = reference[..reference.len() / 2].to_vec();
    assert!(compare(&def, &suite, &reference, &short).is_err());
    std::fs::remove_dir_all(&d).ok();
}

#[test]
#[ignore = "slow: renders the 50 s suite through real scenes; run with make check-full"]
fn compare_real_renders() {
    let d = tmp("render");
    let kit = d.join("kit");
    gen(&kit, false).unwrap();
    let delay = (0.3704 * FS as f64) as usize;
    let suite = suite::build();
    for def in catalog() {
        let name = def.name.as_str();
        let reference = render_scene(
            &kit.join("scenes").join(format!("{name}.snap")),
            &kit.join("suite.wav"),
            &d,
        )
        .unwrap();
        let peak = reference.iter().fold(0f32, |m, v| m.max(v.abs()));
        assert!(
            peak > 0.01 && peak < 0.95,
            "{name}: scene renders silence or clips (peak {peak})"
        );
        let rec = fake_recording(&d.join("r.wav"), &reference, delay, |x| x.to_vec());
        let r = compare(&def, &suite, &reference, &rec).unwrap();
        assert!(
            (r.latency_ms - 370.4).abs() < 0.05,
            "{name}: latency {}",
            r.latency_ms
        );
        for m in &r.metrics {
            assert!(
                m.ok() && m.value.abs() < 0.2,
                "{name}: {} {:+.3}",
                m.name,
                m.value
            );
        }
        if ["00-thru", "20-comp-std", "35-fx-tape"].contains(&name) {
            // 3 dB hotter than the render: caught
            let g = 10f32.powf(3.0 / 20.0);
            let rec = fake_recording(&d.join("r.wav"), &reference, delay, |x| {
                x.iter().map(|v| v * g).collect()
            });
            let r = compare(&def, &suite, &reference, &rec).unwrap();
            assert!(!r.ok(), "{name}: +3 dB not flagged");
        }
    }
    std::fs::remove_dir_all(&d).ok();
}
