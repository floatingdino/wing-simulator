//! Hardware verification kit (T072): compare recordings made through a real WING with `wing-render` output.
//!
//! Workflow (see `tools/verify-hardware/README.md`):
//! 1. `wing-verify gen --out kit` writes `suite.wav` (one mono test signal), one `.snap` per scene and a
//!    `manifest.json` / `run-sheet.md`.
//! 2. On the console: recall a scene, play `suite.wav` into USB 1 and record USB out 1 (`capture.sh`).
//! 3. `wing-verify compare --kit kit --recordings rec --out report` renders every scene for the same signal,
//!    aligns the recording (pilot burst, drift check) and reports the deviation per scene and assumption.
//!
//! Metrics are all in dB (time constants as 20 log10 of the rec/ref ratio) and pass when `|value| <= tol`.

pub mod analyze;
pub mod scenes;
pub mod suite;

use std::path::{Path, PathBuf};

use serde_json::{json, Value};
use wing_model::Snapshot;
use wing_sim::wav::{read_all, write_pcm};

use crate::{load_scene, plan_from_map, render, RenderConfig};
use analyze::*;
use scenes::{base_sets, catalog, SceneDef};
use suite::{Kind, Section, Suite, FS};

/// One measured deviation.
#[derive(Clone, Debug)]
pub struct Metric {
    pub kind: Kind,
    pub name: String,
    /// Recording minus render (dB).
    pub value: f64,
    pub tol: f64,
    /// Where it was worst (level, frequency, ...).
    pub at: String,
}

impl Metric {
    pub fn ok(&self) -> bool {
        self.value.is_finite() && self.value.abs() <= self.tol
    }
}

/// Result for one scene.
#[derive(Clone, Debug)]
pub struct SceneReport {
    pub scene: String,
    pub area: &'static str,
    pub assumption: &'static str,
    /// Recording delay vs the render (ms): device latency + driver buffering.
    pub latency_ms: f64,
    /// Lag change between the start and end pilots (frames).
    pub drift_frames: f64,
    /// Pilot correlation score (1 = identical).
    pub align_score: f64,
    pub metrics: Vec<Metric>,
    pub notes: Vec<String>,
}

impl SceneReport {
    pub fn ok(&self) -> bool {
        self.metrics.iter().all(Metric::ok) && self.align_score > 0.3
    }
    pub fn worst(&self) -> Option<&Metric> {
        self.metrics
            .iter()
            .max_by(|a, b| (a.value.abs() / a.tol).total_cmp(&(b.value.abs() / b.tol)))
    }
}

// ---- gen --------------------------------------------------------------------------------------------------

/// The scene as `.snap` text.
pub fn scene_snap(def: &SceneDef) -> Result<String, String> {
    let (store, _) = load_scene(None)?;
    for (p, v) in base_sets().iter().chain(def.sets.iter()) {
        let id = store
            .lookup(p)
            .ok_or_else(|| format!("{}: no such parameter {p}", def.name))?;
        store
            .set_text(id, v, 0)
            .map_err(|e| format!("{}: {p} = {v}: {e:?}", def.name))?;
    }
    Ok(Snapshot::factory().save(&store))
}

/// Write the kit into `out`: `suite.wav`, `scenes/*.snap`, `manifest.json`, `run-sheet.md`.
pub fn gen(out: &Path, quick_only: bool) -> Result<Vec<SceneDef>, String> {
    let io = |e: std::io::Error| e.to_string();
    std::fs::create_dir_all(out.join("scenes")).map_err(io)?;
    let suite = suite::build();
    write_pcm(&out.join("suite.wav"), 1, FS, 24, &suite.samples).map_err(io)?;
    let defs: Vec<SceneDef> = catalog()
        .into_iter()
        .filter(|s| s.quick || !quick_only)
        .collect();
    for d in &defs {
        std::fs::write(
            out.join("scenes").join(format!("{}.snap", d.name)),
            scene_snap(d)?,
        )
        .map_err(io)?;
    }
    let manifest = json!({
        "suite_version": suite::VERSION,
        "sample_rate": FS,
        "suite_seconds": suite.seconds(),
        "scenes": defs.iter().map(|d| json!({
            "name": d.name, "area": d.area, "assumption": d.assumption,
            "tests": d.kinds.iter().map(|k| k.name()).collect::<Vec<_>>(),
            "quick": d.quick,
        })).collect::<Vec<_>>(),
        "sections": suite.sections.iter().map(|s| json!({
            "kind": s.kind.name(), "start_s": s.start as f64 / FS as f64, "end_s": s.end as f64 / FS as f64,
        })).collect::<Vec<_>>(),
    });
    std::fs::write(
        out.join("manifest.json"),
        serde_json::to_string_pretty(&manifest).map_err(|e| e.to_string())? + "\n",
    )
    .map_err(io)?;
    let mut sheet = String::from(
        "# Run sheet\n\nFor each scene in order: recall it on the WING (recall scope: everything, safes off), run\n\
         `tools/verify-hardware/capture.sh <kit> <scene> <recordings>`, tick it off.\n\n",
    );
    for (i, d) in defs.iter().enumerate() {
        sheet.push_str(&format!(
            "{}. [ ] `{}` - {}: {}\n",
            i + 1,
            d.name,
            d.area,
            d.assumption
        ));
    }
    std::fs::write(out.join("run-sheet.md"), sheet).map_err(io)?;
    Ok(defs)
}

// ---- render -----------------------------------------------------------------------------------------------

/// Render `suite_wav` through `snap`; returns USB out 1 (mono).
pub fn render_scene(snap: &Path, suite_wav: &Path, work: &Path) -> Result<Vec<f32>, String> {
    let (store, _) = load_scene(Some(snap))?;
    let plan = plan_from_map(&format!("usb:1={}", suite_wav.display()), None)?;
    let dir = work.join(snap.file_stem().and_then(|s| s.to_str()).unwrap_or("scene"));
    let cfg = RenderConfig {
        rate: FS,
        block: 64,
        threads: 0,
        start: 0,
        end: None,
        tail: FS as u64,
        taps: wing_sim::record::parse_tap_groups("usb_out/1")?,
        out_dir: dir.clone(),
    };
    render(store, &plan, &cfg, |_| {})?;
    let wav = std::fs::read_dir(&dir)
        .map_err(|e| e.to_string())?
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .find(|p| p.extension().is_some_and(|e| e == "wav"))
        .ok_or("render produced no wav")?;
    let (info, v) = read_all(&wav).map_err(|e| e.to_string())?;
    Ok(v.chunks(info.channels as usize).map(|c| c[0]).collect())
}

/// Channel 1 of a recording.
pub fn read_recording(path: &Path) -> Result<Vec<f32>, String> {
    let (info, v) = read_all(path).map_err(|e| format!("{}: {e}", path.display()))?;
    if info.sample_rate != FS {
        return Err(format!(
            "{}: sample rate {} (need {FS}): set the recorder/console to 48 kHz",
            path.display(),
            info.sample_rate
        ));
    }
    Ok(v.chunks(info.channels as usize).map(|c| c[0]).collect())
}

// ---- compare ----------------------------------------------------------------------------------------------

/// Align `rec` with `reference` (pilots) and measure every kind the scene lists.
pub fn compare(
    def: &SceneDef,
    suite: &Suite,
    reference: &[f32],
    rec: &[f32],
) -> Result<SceneReport, String> {
    let mut notes = Vec::new();
    // coarse lag from the first pilot (recording starts at an arbitrary time before the playback)
    let (p0, pl) = suite.pilot;
    let needle = &reference[p0..p0 + pl];
    let hay_len = rec.len().min(FS as usize * 12);
    let (pos, score) = locate(&rec[..hay_len], needle).ok_or("recording too short")?;
    let lag0 = pos as i64 - p0 as i64;
    // end pilot: search +-100 ms around the predicted place
    let pe = suite.end_pilot as i64 + lag0;
    let margin = (FS / 10) as i64;
    let (a, b) = (
        (pe - margin).max(0) as usize,
        (pe + pl as i64 + margin) as usize,
    );
    if b > rec.len() {
        return Err(
            "recording ends before the end pilot: record at least 8 s longer than the suite".into(),
        );
    }
    let reference_end = &reference[suite.end_pilot..suite.end_pilot + pl];
    let (pos_e, _) = locate(&rec[a..b], reference_end).ok_or("recording too short")?;
    let lag1 = (a + pos_e) as i64 - suite.end_pilot as i64;
    let drift = (lag1 - lag0) as f64;
    if drift.abs() > 2.0 {
        notes.push(format!(
            "clock drift {drift:+.0} frames over the suite: play and record on the same device clock"
        ));
    }
    if score < 0.5 {
        notes.push(format!(
            "weak pilot match (score {score:.2}): wrong scene recalled or recording clipped?"
        ));
    }
    let span = (suite.end_pilot - p0) as f64;
    let lag_at = |t: usize| (lag0 as f64 + drift * (t as f64 - p0 as f64) / span).round() as i64;

    let mut metrics = Vec::new();
    for &kind in &def.kinds {
        let sec = suite.section(kind);
        let lag = lag_at((sec.start + sec.end) / 2);
        let cut = |s: usize, e: usize| -> Option<(&[f32], &[f32])> {
            let (rs, re) = (s as i64 + lag, e as i64 + lag);
            (rs >= 0 && re as usize <= rec.len())
                .then(|| (&reference[s..e], &rec[rs as usize..re as usize]))
        };
        let m = match kind {
            Kind::Fr => fr(sec, &cut),
            Kind::Stairs => stairs(sec, &cut),
            Kind::Steps => steps(sec, &cut),
            Kind::Bursts => bursts(sec, &cut),
            Kind::Tone => tone(sec, &cut),
        };
        match m {
            Some(m) => metrics.extend(m),
            None => notes.push(format!(
                "{}: recording too short for this section",
                kind.name()
            )),
        }
    }
    Ok(SceneReport {
        scene: def.name.clone(),
        area: def.area,
        assumption: def.assumption,
        latency_ms: lag0 as f64 / FS as f64 * 1e3,
        drift_frames: drift,
        align_score: score,
        metrics,
        notes,
    })
}

type Cut<'a> = dyn Fn(usize, usize) -> Option<(&'a [f32], &'a [f32])> + 'a;

fn metric(kind: Kind, name: &str, value: f64, tol: f64, at: impl Into<String>) -> Metric {
    Metric {
        kind,
        name: name.into(),
        value,
        tol,
        at: at.into(),
    }
}

/// Frequency response: 1/3-octave band level difference over the sweep.
fn fr(sec: &Section, cut: &Cut) -> Option<Vec<Metric>> {
    let (rf, rc) = cut(sec.start, sec.end)?;
    let ks = -16..=13; // 25 Hz .. 16 kHz
    let (pf, pr) = (
        third_octave_power(rf, FS as f64, ks.clone()),
        third_octave_power(rc, FS as f64, ks),
    );
    let top = pf.iter().map(|b| b.1).fold(0.0, f64::max);
    let mut worst = (0.0f64, String::new());
    let (mut sum, mut n) = (0.0, 0);
    for (a, b) in pf.iter().zip(&pr) {
        if a.1 < top * 1e-6 {
            continue; // reference more than 60 dB down: below the recording's noise
        }
        let d = 10.0 * (b.1.max(1e-30) / a.1).log10();
        sum += d;
        n += 1;
        if d.abs() > worst.0.abs() {
            worst = (d, format!("{:.0} Hz", a.0));
        }
    }
    Some(vec![
        metric(
            Kind::Fr,
            "response, worst 1/3-octave band",
            worst.0,
            1.0,
            worst.1,
        ),
        metric(
            Kind::Fr,
            "response, mean level",
            sum / n.max(1) as f64,
            1.0,
            "",
        ),
    ])
}

/// Static curve: settled level per step.
fn stairs(sec: &Section, cut: &Cut) -> Option<Vec<Metric>> {
    let mut d = Vec::new();
    for &(s, e, lvl) in &sec.items {
        let (rf, rc) = cut(e - FS as usize * 15 / 100, e - FS as usize / 100)?;
        let (lr, lc) = (rms_db(rf), rms_db(rc));
        if lr > -80.0 {
            d.push((lc - lr, lvl, s));
        }
    }
    let mean = d.iter().map(|x| x.0).sum::<f64>() / d.len().max(1) as f64;
    let w = d.iter().max_by(|a, b| a.0.abs().total_cmp(&b.0.abs()))?;
    let shape = d
        .iter()
        .max_by(|a, b| (a.0 - mean).abs().total_cmp(&(b.0 - mean).abs()))?;
    Some(vec![
        metric(
            Kind::Stairs,
            "static level, worst step",
            w.0,
            1.0,
            format!("{} dBFS in", w.1),
        ),
        metric(Kind::Stairs, "static level, mean offset", mean, 1.0, ""),
        metric(
            Kind::Stairs,
            "static curve shape (worst step minus mean)",
            shape.0 - mean,
            0.75,
            format!("{} dBFS in", shape.1),
        ),
    ])
}

/// Envelope deviation (dB, 5 ms RMS) between two aligned segments, ignoring `skip_ms` after the start and where
/// both are quiet.
fn envelope_dev(rf: &[f32], rc: &[f32], skip_ms: usize) -> (f64, f64) {
    let (ef, ec) = (
        envelope_db(rf, FS as usize / 200, FS as usize / 1000),
        envelope_db(rc, FS as usize / 200, FS as usize / 1000),
    );
    let mut worst = (0.0f64, 0.0f64);
    for (i, (a, b)) in ef.iter().zip(&ec).enumerate() {
        if i < skip_ms || (*a < -75.0 && *b < -75.0) {
            continue;
        }
        let d = b - a;
        if d.abs() > worst.0.abs() {
            worst = (d, i as f64);
        }
    }
    worst
}

/// Level steps: settling times (up and down), settled level, envelope.
fn steps(sec: &Section, cut: &Cut) -> Option<Vec<Metric>> {
    let mut up = Vec::new();
    let mut down = Vec::new();
    let mut fin = Vec::new();
    let mut worst_env = (0.0f64, 0.0f64);
    for (i, &(s, e, _)) in sec.items.iter().enumerate().skip(1) {
        let (rf, rc) = cut(s, e)?;
        let hop = 1;
        let (ef, ec) = (
            envelope_db(rf, FS as usize / 200, FS as usize / 1000 * hop),
            envelope_db(rc, FS as usize / 200, FS as usize / 1000 * hop),
        );
        // skip the first 3 ms (alignment jitter at the edge)
        let (ef, ec) = (&ef[3..], &ec[3..]);
        let (tf, tc) = (settle_time_ms(ef, 1.0, 1.5), settle_time_ms(ec, 1.0, 1.5));
        if let (Some(a), Some(b)) = (tf, tc) {
            let r = 20.0 * (b.max(0.5) / a.max(0.5)).log10();
            if i % 2 == 1 {
                up.push(r)
            } else {
                down.push(r)
            }
        }
        let tail = ef.len() * 4 / 5;
        let mean = |x: &[f64]| x[tail..].iter().sum::<f64>() / (x.len() - tail) as f64;
        fin.push(mean(ec) - mean(ef));
        let (d, at) = envelope_dev(rf, rc, 8);
        if d.abs() > worst_env.0.abs() {
            worst_env = (d, at + i as f64 * 1500.0);
        }
    }
    let avg = |v: &[f64]| (!v.is_empty()).then(|| v.iter().sum::<f64>() / v.len() as f64);
    let mut m = Vec::new();
    if let Some(v) = avg(&up) {
        m.push(metric(
            Kind::Steps,
            "settling time after +30 dB step (rec/ref)",
            v,
            6.0,
            "",
        ));
    }
    if let Some(v) = avg(&down) {
        m.push(metric(
            Kind::Steps,
            "settling time after -30 dB step (rec/ref)",
            v,
            6.0,
            "",
        ));
    }
    m.push(metric(
        Kind::Steps,
        "settled level after step",
        avg(&fin)?,
        1.0,
        "",
    ));
    m.push(metric(
        Kind::Steps,
        "envelope, worst point",
        worst_env.0,
        4.0,
        format!("{:.0} ms into the section", worst_env.1),
    ));
    Some(m)
}

/// Noise bursts: burst levels and envelope.
fn bursts(sec: &Section, cut: &Cut) -> Option<Vec<Metric>> {
    let mut worst = (0.0f64, 0.0f64);
    for &(s, e, lvl) in &sec.items {
        let (m0, m1) = (s + (e - s) / 10, e - (e - s) / 10);
        let (rf, rc) = cut(m0, m1)?;
        let (lr, lc) = (rms_db(rf), rms_db(rc));
        if lr > -80.0 && (lc - lr).abs() > worst.0.abs() {
            worst = (lc - lr, lvl);
        }
    }
    let (rf, rc) = cut(sec.start, sec.end)?;
    let (d, at) = envelope_dev(rf, rc, 8);
    Some(vec![
        metric(
            Kind::Bursts,
            "burst level, worst burst",
            worst.0,
            1.5,
            format!("{} dBFS in", worst.1),
        ),
        metric(
            Kind::Bursts,
            "envelope, worst point",
            d,
            4.0,
            format!("{at:.0} ms into the section"),
        ),
    ])
}

/// Harmonics of the steady tones.
fn tone(sec: &Section, cut: &Cut) -> Option<Vec<Metric>> {
    let mut m = Vec::new();
    for &(s, e, f0) in &sec.items {
        let (a, b) = (s + FS as usize * 6 / 10, e - FS as usize / 10);
        let (rf, rc) = cut(a, b)?;
        let n = (b - a).next_power_of_two();
        let (pf, pc) = (power_spectrum(rf, n), power_spectrum(rc, n));
        let mut worst = (0.0f64, 0usize);
        let (mut tf, mut tc) = (0.0f64, 0.0f64);
        for h in 2..=5 {
            let (Some(x), Some(y)) = (
                harmonic_db(&pf, n, FS as f64, f0, h),
                harmonic_db(&pc, n, FS as f64, f0, h),
            ) else {
                continue;
            };
            tf += 10f64.powf(x / 10.0);
            tc += 10f64.powf(y / 10.0);
            if x.max(y) > -90.0 && (y - x).abs() > worst.0.abs() {
                worst = (y - x, h);
            }
        }
        let name = format!("{f0:.0} Hz");
        m.push(metric(
            Kind::Tone,
            &format!("{name}: worst harmonic (rec minus ref, dB re fundamental)"),
            worst.0,
            6.0,
            format!("H{}", worst.1),
        ));
        m.push(metric(
            Kind::Tone,
            &format!("{name}: THD H2-H5 (rec/ref)"),
            10.0 * (tc.max(1e-12) / tf.max(1e-12)).log10(),
            6.0,
            format!(
                "ref {:.4} %, rec {:.4} %",
                tf.sqrt() * 100.0,
                tc.sqrt() * 100.0
            ),
        ));
        let (lf, lc) = (rms_db(rf), rms_db(rc));
        m.push(metric(
            Kind::Tone,
            &format!("{name}: level"),
            lc - lf,
            1.0,
            "",
        ));
    }
    Some(m)
}

// ---- reporting ----------------------------------------------------------------------------------------------

/// Full Markdown report.
pub fn report_md(rs: &[SceneReport]) -> String {
    let mut s = String::from("# Hardware verification report\n\nDeviation = recording minus `wing-render` (dB); OK when within the tolerance.\n\n");
    let bad = rs.iter().filter(|r| !r.ok()).count();
    s.push_str(&format!(
        "{} scenes compared, {} deviate.\n\n",
        rs.len(),
        bad
    ));
    for r in rs {
        s.push_str(&format!(
            "## `{}` - {}\n\n{}\n\nlatency {:.2} ms (device, vs render), drift {:+.0} frames, pilot match {:.2}\n\n",
            r.scene, r.area, r.assumption, r.latency_ms, r.drift_frames, r.align_score
        ));
        for n in &r.notes {
            s.push_str(&format!("- note: {n}\n"));
        }
        s.push_str("\n| Test | Metric | Deviation (dB) | Tolerance | Worst at | Result |\n|---|---|---|---|---|---|\n");
        for m in &r.metrics {
            s.push_str(&format!(
                "| {} | {} | {:+.2} | {} | {} | {} |\n",
                m.kind.name(),
                m.name,
                m.value,
                m.tol,
                m.at,
                if m.ok() { "OK" } else { "DEVIATES" }
            ));
        }
        s.push('\n');
    }
    s
}

/// The per-scene results block for `docs/VERIFY-ON-HARDWARE.md`.
pub fn results_table(rs: &[SceneReport]) -> String {
    let mut s = String::from("| Scene | Area | Assumption | Worst deviation | Latency | Result |\n|---|---|---|---|---|---|\n");
    for r in rs {
        let w = r
            .worst()
            .map(|m| format!("{:+.2} dB ({}, {})", m.value, m.name, m.at))
            .unwrap_or_else(|| "-".into());
        s.push_str(&format!(
            "| `{}` | {} | {} | {} | {:.1} ms | {} |\n",
            r.scene,
            r.area,
            r.assumption,
            w,
            r.latency_ms,
            if r.ok() { "OK" } else { "DEVIATES" }
        ));
    }
    s
}

/// The scene/assumption map for the docs (static, independent of any recording).
pub fn scenes_table() -> String {
    let mut s = String::from(
        "| Scene | Tier | Tests | Area | Assumption under test |\n|---|---|---|---|---|\n",
    );
    for d in catalog() {
        s.push_str(&format!(
            "| `{}` | {} | {} | {} | {} |\n",
            d.name,
            if d.quick { "quick" } else { "full" },
            d.kinds
                .iter()
                .map(|k| k.name())
                .collect::<Vec<_>>()
                .join(", "),
            d.area,
            d.assumption
        ));
    }
    s
}

/// Replace the text between `<!-- verify:<tag>:begin -->` and `<!-- verify:<tag>:end -->` in `doc`.
pub fn replace_block(doc: &str, tag: &str, body: &str) -> Result<String, String> {
    let (b, e) = (
        format!("<!-- verify:{tag}:begin -->"),
        format!("<!-- verify:{tag}:end -->"),
    );
    let i = doc.find(&b).ok_or(format!("marker {b} not found"))? + b.len();
    let j = doc.find(&e).ok_or(format!("marker {e} not found"))?;
    if j < i {
        return Err("markers out of order".into());
    }
    Ok(format!(
        "{}\n\n{}\n{}",
        &doc[..i],
        body.trim_end(),
        &doc[j..]
    ))
}

/// Compare every recording found in `rec_dir` (`<scene>.wav`) with the render of the kit scene.
pub fn compare_all(
    kit: &Path,
    rec_dir: &Path,
    work: &Path,
    only: &[String],
) -> Result<Vec<SceneReport>, String> {
    let manifest: Value = serde_json::from_str(
        &std::fs::read_to_string(kit.join("manifest.json"))
            .map_err(|e| format!("{}: {e}", kit.display()))?,
    )
    .map_err(|e| e.to_string())?;
    if manifest["suite_version"] != json!(suite::VERSION) {
        return Err(
            "kit was generated by another suite version: run `gen` again and re-record".into(),
        );
    }
    let suite = suite::build();
    let mut out = Vec::new();
    for def in catalog() {
        if !only.is_empty() && !only.contains(&def.name) {
            continue;
        }
        let snap = kit.join("scenes").join(format!("{}.snap", def.name));
        let wav: PathBuf = rec_dir.join(format!("{}.wav", def.name));
        if !snap.exists() || !wav.exists() {
            continue;
        }
        let rec = read_recording(&wav)?;
        let reference = render_scene(&snap, &kit.join("suite.wav"), work)?;
        out.push(
            compare(&def, &suite, &reference, &rec).map_err(|e| format!("{}: {e}", def.name))?,
        );
    }
    if out.is_empty() {
        return Err(format!(
            "no recordings <scene>.wav for the kit's scenes in {}",
            rec_dir.display()
        ));
    }
    Ok(out)
}
