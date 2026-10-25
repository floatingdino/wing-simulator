//! Offline renderer (T060): multitrack WAVs (or a REAPER project's tracks) + a `.snap` scene →
//! console outputs as WAV files, faster than realtime and bit-identical to the `wing-sim` daemon
//! (same [`wing_sim::audio::Console`] code path; see [`render`]).
//!
//! * [`inputs`]: input files → USB inputs 1..48 (file order / track numbers / `--map`), and the
//!   streaming reader.
//! * [`rpp`]: read-only REAPER `.RPP` import (track hardware outputs → USB inputs).
//! * [`render`]: console construction (shared with the daemon) and the render loop.
//! * [`verify`]: hardware verification kit (`wing-verify`, T072): test signal, scenes, recording comparison.
//! * [`loudness`]: sample peak and BS.1770-4 integrated loudness for the report.
//! * [`report_json`]: the JSON render report.
//!
//! Tap groups (`--taps main/1,bus/3,mon/1,usb_out/1-48`) are `wing_sim::record`'s, shared with
//! the daemon's recorder (T061).

pub mod inputs;
pub mod loudness;
pub mod render;
pub mod rpp;
pub mod verify;

use serde_json::{json, Value};

pub use inputs::{plan_from_files, plan_from_map, InputPlan, InputReader};
pub use render::{build_console, load_scene, render, RenderConfig, RenderReport};
pub use rpp::plan_from_rpp;

/// The JSON report written next to the renders (`render_report.json`).
pub fn report_json(scene: &str, plan: &InputPlan, cfg: &RenderConfig, rep: &RenderReport) -> Value {
    let num = |x: f64| {
        if x.is_finite() {
            json!((x * 100.0).round() / 100.0)
        } else {
            Value::Null
        }
    };
    json!({
        "tool": format!("wing-render {}", env!("CARGO_PKG_VERSION")),
        "scene": scene,
        "sample_rate": cfg.rate,
        "block": cfg.block,
        "threads": cfg.threads,
        "start_frame": cfg.start,
        "frames": rep.frames,
        "seconds": num(rep.seconds),
        "build_seconds": num(rep.build_seconds),
        "render_seconds": (rep.render_seconds * 1000.0).round() / 1000.0,
        "speed_x_realtime": num(rep.speed_x_realtime),
        "inputs": plan.describe(),
        "notes": plan.notes,
        "taps": rep.taps.iter().map(|t| json!({
            "name": t.name,
            "file": t.file.display().to_string(),
            "channels": t.channels,
            "peak_dbfs": num(t.peak_dbfs),
            "lufs_integrated": t.lufs_integrated.map_or(Value::Null, num),
        })).collect::<Vec<_>>(),
    })
}
