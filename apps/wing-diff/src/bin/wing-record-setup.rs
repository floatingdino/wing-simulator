//! `wing-record-setup [show.snap] [--out DIR] [--channels 1-40] [--aux 1-8|none] [--dry-run]`
//!
//! Writes `DIR/Recording-Scene.snap` (scoped recording patch for the console) and
//! `DIR/Recording.RPP` (armed DAW project); see `wing_diff::record` and
//! docs/user/recording-workflow.md.

use std::path::PathBuf;
use std::process::ExitCode;

use wing_diff::record::{parse_list, plan, Selection};
use wing_diff::{load_file, load_text};
use wing_model::FACTORY_SCENE;

const USAGE: &str = "usage: wing-record-setup [show.snap] [--out DIR] [--channels 1-40] [--aux 1-8|none] [--dry-run]

Reads the show scene's input patch (the factory scene if none is given) and writes to DIR
(default: current folder):
  Recording-Scene.snap  load on the console: USB outs <- each strip's source, strip ALT <- USB,
                        ALT switch off. Only those settings are in the file.
  Recording.RPP         REAPER project: one armed track per strip on its USB channel(s).";

fn run() -> Result<(), String> {
    let mut show = None;
    let mut out = PathBuf::from(".");
    let mut sel = Selection::default();
    let mut dry = false;
    let mut it = std::env::args().skip(1);
    while let Some(a) = it.next() {
        let mut val = |name: &str| it.next().ok_or(format!("{name} needs a value"));
        match a.as_str() {
            "--out" | "-o" => out = PathBuf::from(val("--out")?),
            "--channels" => sel.channels = parse_list(&val("--channels")?, 40)?,
            "--aux" => sel.aux = parse_list(&val("--aux")?, 8)?,
            "--dry-run" => dry = true,
            "--version" | "-V" => {
                println!(
                    "wing-record-setup {} (commit {}, built {})",
                    env!("CARGO_PKG_VERSION"),
                    env!("WING_GIT_HASH"),
                    env!("WING_BUILD_TIME")
                );
                return Ok(());
            }
            "-h" | "--help" => {
                println!("{USAGE}");
                return Ok(());
            }
            s if s.starts_with('-') => return Err(format!("unknown option {s}\n{USAGE}")),
            _ if show.is_none() => show = Some(a),
            _ => return Err(USAGE.into()),
        }
    }
    let scene = match &show {
        Some(p) => load_file(p)?,
        None => load_text("factory scene", FACTORY_SCENE)?,
    };
    let p = plan(&scene, &sel);
    print!("{}", p.summary());
    if p.slots.is_empty() {
        return Err("no strip has a recordable source".into());
    }
    if show.is_none() {
        eprintln!("note: no show given, used the factory patch; pass your show .snap for real names and sources");
    }
    if dry {
        return Ok(());
    }
    std::fs::create_dir_all(&out).map_err(|e| format!("{}: {e}", out.display()))?;
    for (name, text) in [
        ("Recording-Scene.snap", p.scene()),
        ("Recording.RPP", p.rpp()),
    ] {
        let path = out.join(name);
        std::fs::write(&path, text).map_err(|e| format!("{}: {e}", path.display()))?;
        println!("wrote {}", path.display());
    }
    Ok(())
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("wing-record-setup: {e}");
            ExitCode::from(2)
        }
    }
}
