//! `wing-render`: offline renderer (T060).
//!
//! ```text
//! wing-render --scene mix.snap (--inputs <dir|file>[,...] | --rpp project.RPP) [--map usb:1=f.wav,...]
//!             --out <dir> [--taps main/1,bus/3,mon/1,usb_out/1-48] [--rate 48000] [--threads N]
//!             [--start SECS] [--end SECS] [--tail SECS] [--block 64] [--set path=value]...
//!             [--dry-run] [--quiet]
//! wing-render --bench [...]     performance suite (T070; see wing_render::perf, docs/perf.md)
//! ```
//!
//! Inputs: `--inputs` (repeatable, comma-separated) files or directories of WAVs; USB inputs by
//! leading track number in the file names, else by name order (see `wing_render::inputs`).
//! `--map` assigns explicitly (`usb:N=file[:chan|:mix]`, `usb:N-M=stereo.wav`; names relative to
//! the first `--inputs` directory); with `--map` only mapped files play. `--rpp` reads a REAPER
//! project's track hardware outputs (read-only; limits in `wing_render::rpp`).
//! Outputs: `<out>/<tap>.wav` (32-bit float) per tap group and `<out>/render_report.json` (peak
//! dBFS, integrated LUFS per tap, render speed ×realtime). `--dry-run` prints the input map only.
//! `--set` changes a parameter after the scene is loaded (e.g. `--set io/altsw=1`: ALT INPUT on,
//! the virtual-soundcheck switch of a scene whose channels have USB alt sources).

use std::path::{Path, PathBuf};
use std::process::ExitCode;

use wing_render::{
    load_scene, plan_from_files, plan_from_map, plan_from_rpp, render, report_json, InputPlan,
    RenderConfig,
};
use wing_sim::record::parse_tap_groups;

const USAGE: &str =
    "usage: wing-render --scene mix.snap (--inputs <dir|files> | --rpp project.RPP) \
[--map usb:1=file.wav,...] --out <dir> [--taps main/1,bus/3,mon/1,usb_out/1-48] [--rate 48000] \
[--threads N] [--start SECS] [--end SECS] [--tail SECS] [--block 64] [--set path=value]... \
[--dry-run] [--quiet]";

#[derive(Debug)]
struct Args {
    scene: Option<PathBuf>,
    inputs: Vec<PathBuf>,
    rpp: Option<PathBuf>,
    map: Option<String>,
    out: Option<PathBuf>,
    taps: String,
    rate: u32,
    threads: usize,
    start: f64,
    end: Option<f64>,
    tail: f64,
    block: usize,
    sets: Vec<(String, String)>,
    dry_run: bool,
    quiet: bool,
}

fn default_threads() -> usize {
    std::thread::available_parallelism()
        .map(|n| n.get())
        .unwrap_or(1)
        .saturating_sub(1)
        .min(7)
}

fn parse_args(mut it: impl Iterator<Item = String>) -> Result<Args, String> {
    let mut a = Args {
        scene: None,
        inputs: Vec::new(),
        rpp: None,
        map: None,
        out: None,
        taps: "main/1".into(),
        rate: 48_000,
        threads: default_threads(),
        start: 0.0,
        end: None,
        tail: 0.0,
        block: 64,
        sets: Vec::new(),
        dry_run: false,
        quiet: false,
    };
    while let Some(flag) = it.next() {
        let mut val = || it.next().ok_or(format!("{flag} needs a value"));
        let secs = |s: String, f: &str| -> Result<f64, String> {
            s.parse::<f64>()
                .ok()
                .filter(|v| v.is_finite() && *v >= 0.0)
                .ok_or(format!("{f}: bad seconds '{s}'"))
        };
        match flag.as_str() {
            "--scene" => a.scene = Some(val()?.into()),
            "--inputs" => a.inputs.extend(
                val()?
                    .split(',')
                    .filter(|s| !s.trim().is_empty())
                    .map(|s| PathBuf::from(s.trim())),
            ),
            "--rpp" => a.rpp = Some(val()?.into()),
            "--map" => {
                let m = val()?;
                a.map = Some(match a.map.take() {
                    Some(prev) => format!("{prev},{m}"),
                    None => m,
                });
            }
            "--out" => a.out = Some(val()?.into()),
            "--taps" => a.taps = val()?,
            "--rate" => a.rate = val()?.parse().map_err(|e| format!("--rate: {e}"))?,
            "--threads" => a.threads = val()?.parse().map_err(|e| format!("--threads: {e}"))?,
            "--start" => a.start = secs(val()?, "--start")?,
            "--end" => a.end = Some(secs(val()?, "--end")?),
            "--tail" => a.tail = secs(val()?, "--tail")?,
            "--block" => a.block = val()?.parse().map_err(|e| format!("--block: {e}"))?,
            "--set" => {
                let v = val()?;
                let (p, x) = v
                    .split_once('=')
                    .ok_or(format!("--set '{v}': expected path=value"))?;
                a.sets
                    .push((p.trim().trim_start_matches('/').into(), x.trim().into()));
            }
            "--dry-run" => a.dry_run = true,
            "--quiet" | "-q" => a.quiet = true,
            "-h" | "--help" => return Err(USAGE.into()),
            other => return Err(format!("unknown argument {other}\n{USAGE}")),
        }
    }
    if a.inputs.is_empty() && a.rpp.is_none() && a.map.is_none() {
        return Err(format!("no inputs (--inputs, --rpp or --map)\n{USAGE}"));
    }
    if a.rpp.is_some() && a.map.is_some() {
        return Err("--rpp and --map are mutually exclusive".into());
    }
    if a.out.is_none() && !a.dry_run {
        return Err(format!("--out is required\n{USAGE}"));
    }
    if !(8_000..=192_000).contains(&a.rate) {
        return Err("--rate must be 8000..192000".into());
    }
    if a.end.is_some_and(|e| e <= a.start) {
        return Err("--end must be after --start".into());
    }
    Ok(a)
}

fn plan(a: &Args) -> Result<InputPlan, String> {
    if let Some(r) = &a.rpp {
        return plan_from_rpp(r, a.rate);
    }
    if let Some(m) = &a.map {
        let base = a.inputs.iter().find(|p| p.is_dir()).map(PathBuf::as_path);
        return plan_from_map(m, base.or(Some(Path::new("."))));
    }
    plan_from_files(&a.inputs)
}

fn run(a: Args) -> Result<(), String> {
    let say = |s: &str| {
        if !a.quiet {
            eprintln!("{s}");
        }
    };
    let taps = parse_tap_groups(&a.taps)?;
    let plan = plan(&a)?;
    for l in plan.describe() {
        say(&l);
    }
    for n in &plan.notes {
        say(&format!("note: {n}"));
    }
    if a.dry_run {
        if a.quiet {
            for l in plan.describe() {
                println!("{l}");
            }
        }
        return plan.validate(a.rate);
    }
    let (store, scene) = load_scene(a.scene.as_deref())?;
    say(&format!("scene {scene}"));
    for (p, v) in &a.sets {
        let id = store
            .lookup(p)
            .ok_or_else(|| format!("--set {p}: no such parameter"))?;
        store
            .set_text(id, v, 0)
            .map_err(|e| format!("--set {p}={v}: {e:?}"))?;
    }
    let frames = |s: f64| (s * a.rate as f64).round() as u64;
    let cfg = RenderConfig {
        rate: a.rate,
        block: a.block,
        threads: a.threads,
        start: frames(a.start),
        end: a.end.map(frames),
        tail: frames(a.tail),
        taps,
        out_dir: a.out.clone().unwrap_or_default(),
    };
    let mut last = -1i64;
    let rep = render(store, &plan, &cfg, |f| {
        let pct = (f * 100.0) as i64;
        if !a.quiet && pct / 10 != last / 10 {
            last = pct;
            eprint!("\r{pct:3}%");
        }
    })?;
    if !a.quiet {
        eprintln!();
    }
    let json = report_json(&scene, &plan, &cfg, &rep);
    let path = cfg.out_dir.join("render_report.json");
    let text = serde_json::to_string_pretty(&json).map_err(|e| e.to_string())?;
    std::fs::write(&path, text + "\n").map_err(|e| format!("{}: {e}", path.display()))?;
    for t in &rep.taps {
        say(&format!(
            "{:<12} peak {:7.2} dBFS  {}  -> {}",
            t.name,
            t.peak_dbfs,
            t.lufs_integrated
                .map_or("   -- LUFS".to_string(), |l| format!("{l:6.1} LUFS")),
            t.file.display()
        ));
    }
    println!(
        "rendered {:.1} s in {:.2} s ({:.1}x realtime, {} engine threads + 1); report {}",
        rep.seconds,
        rep.render_seconds,
        rep.speed_x_realtime,
        a.threads,
        path.display()
    );
    Ok(())
}

fn main() -> ExitCode {
    let mut argv = std::env::args().skip(1).peekable();
    if argv.peek().is_some_and(|a| a == "--version" || a == "-V") {
        println!(
            "wing-render {} (commit {}, built {})",
            env!("CARGO_PKG_VERSION"),
            env!("WING_GIT_HASH"),
            env!("WING_BUILD_TIME")
        );
        return ExitCode::SUCCESS;
    }
    if argv.peek().is_some_and(|a| a == "--bench") {
        argv.next();
        let r = wing_render::perf::parse_bench_args(argv)
            .and_then(|a| wing_render::perf::bench_main(&a));
        return match r {
            Ok(()) => ExitCode::SUCCESS,
            Err(e) => {
                eprintln!("wing-render --bench: {e}");
                ExitCode::FAILURE
            }
        };
    }
    match parse_args(argv).and_then(run) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("wing-render: {e}");
            ExitCode::FAILURE
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn args(v: &[&str]) -> Result<Args, String> {
        parse_args(v.iter().map(|s| s.to_string()))
    }

    #[test]
    fn cli() {
        let a = args(&[
            "--scene",
            "m.snap",
            "--inputs",
            "a,b",
            "--inputs",
            "c",
            "--out",
            "o",
            "--taps",
            "main/1,usb_out/1-2",
            "--rate",
            "44100",
            "--threads",
            "3",
            "--start",
            "1.5",
            "--end",
            "10",
            "--set",
            "/io/altsw=1",
        ])
        .unwrap();
        assert_eq!(a.sets, [("io/altsw".to_string(), "1".to_string())]);
        assert!(args(&["--inputs", "a", "--dry-run", "--set", "x"]).is_err());
        assert_eq!(a.inputs.len(), 3);
        assert_eq!(
            (a.rate, a.threads, a.start, a.end),
            (44_100, 3, 1.5, Some(10.0))
        );
        assert!(args(&["--out", "o"]).is_err()); // no inputs
        assert!(args(&["--inputs", "a"]).is_err()); // no out
        assert!(args(&["--inputs", "a", "--dry-run"]).is_ok());
        assert!(args(&["--inputs", "a", "--out", "o", "--start", "5", "--end", "2"]).is_err());
        assert!(args(&["--rpp", "p", "--map", "usb:1=x", "--out", "o"]).is_err());
        assert!(args(&["--bogus"]).is_err());
    }
}
