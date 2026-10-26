//! `wing-verify`: hardware verification kit (T072). See `tools/verify-hardware/README.md`.
//!
//! ```text
//! wing-verify gen     --out kit [--tier quick|all]
//! wing-verify compare --kit kit --recordings rec [--out report] [--scenes a,b] [--update-doc docs/VERIFY-ON-HARDWARE.md]
//! wing-verify docs    [--update docs/VERIFY-ON-HARDWARE.md]
//! wing-verify warm-gen --out warm-kit [--tier quick|all]
//! wing-verify fit-warm <recordings> [--out report] [--write data/calibration/warm.json]
//! ```

use std::path::PathBuf;
use std::process::ExitCode;

use wing_render::verify::{
    compare_all, gen, replace_block, report_md, results_table, scenes_table, warm,
};

const USAGE: &str = "usage:
  wing-verify gen     --out <kit-dir> [--tier quick|all]
  wing-verify compare --kit <kit-dir> --recordings <dir> [--out <dir>] [--scenes a,b] [--update-doc <md>]
  wing-verify docs    [--update <md>]
  wing-verify warm-gen --out <kit-dir> [--tier quick|all]
  wing-verify fit-warm <recordings-dir> [--out <dir>] [--write <warm.json>]";

struct Flags(Vec<String>);

impl Flags {
    fn get(&self, name: &str) -> Option<String> {
        self.0
            .iter()
            .position(|a| a == name)
            .and_then(|i| self.0.get(i + 1).cloned())
    }
    fn req(&self, name: &str) -> Result<String, String> {
        self.get(name).ok_or(format!("{name} is required\n{USAGE}"))
    }
}

fn update_doc(path: &str, tag: &str, body: &str) -> Result<(), String> {
    let doc = std::fs::read_to_string(path).map_err(|e| format!("{path}: {e}"))?;
    std::fs::write(path, replace_block(&doc, tag, body)?).map_err(|e| format!("{path}: {e}"))
}

fn run() -> Result<(), String> {
    let mut args = std::env::args().skip(1);
    let cmd = args.next().ok_or(USAGE)?;
    let f = Flags(args.collect());
    match cmd.as_str() {
        "gen" => {
            let out = PathBuf::from(f.req("--out")?);
            let quick = match f.get("--tier").as_deref() {
                None | Some("quick") => true,
                Some("all") => false,
                Some(t) => return Err(format!("--tier {t}: quick or all")),
            };
            let defs = gen(&out, quick)?;
            println!(
                "kit written to {}: suite.wav ({} scenes in scenes/, run-sheet.md)",
                out.display(),
                defs.len()
            );
            Ok(())
        }
        "compare" => {
            let kit = PathBuf::from(f.req("--kit")?);
            let rec = PathBuf::from(f.req("--recordings")?);
            let out = PathBuf::from(f.get("--out").unwrap_or_else(|| "verify-report".into()));
            std::fs::create_dir_all(&out).map_err(|e| e.to_string())?;
            let only: Vec<String> = f
                .get("--scenes")
                .map(|s| s.split(',').map(|x| x.trim().to_string()).collect())
                .unwrap_or_default();
            let rs = compare_all(&kit, &rec, &out.join("render"), &only)?;
            let md = report_md(&rs);
            std::fs::write(out.join("report.md"), &md).map_err(|e| e.to_string())?;
            for r in &rs {
                let w = r
                    .worst()
                    .map(|m| format!("{:+.2} dB {}", m.value, m.name))
                    .unwrap_or_default();
                println!(
                    "{:<16} {:<9} latency {:6.2} ms  {}",
                    r.scene,
                    if r.ok() { "OK" } else { "DEVIATES" },
                    r.latency_ms,
                    w
                );
                for n in &r.notes {
                    println!("    note: {n}");
                }
            }
            println!("report: {}", out.join("report.md").display());
            if let Some(doc) = f.get("--update-doc") {
                update_doc(&doc, "results", &results_table(&rs))?;
                println!("results written to {doc}");
            }
            Ok(())
        }
        "warm-gen" => {
            let out = PathBuf::from(f.req("--out")?);
            let quick = match f.get("--tier").as_deref() {
                None | Some("quick") => true,
                Some("all") => false,
                Some(t) => return Err(format!("--tier {t}: quick or all")),
            };
            let scenes = warm::gen(&out, quick)?;
            println!(
                "WARM kit written to {}: suite.wav ({:.0} s), {} scenes in scenes/, run-sheet.md",
                out.display(),
                warm::build().seconds(),
                scenes.len()
            );
            Ok(())
        }
        "fit-warm" => {
            let rec =
                f.0.first()
                    .filter(|a| !a.starts_with("--"))
                    .map(PathBuf::from)
                    .ok_or(format!("recordings dir required\n{USAGE}"))?;
            let out = PathBuf::from(f.get("--out").unwrap_or_else(|| "warm-report".into()));
            let write = f.get("--write").map(PathBuf::from);
            let r = warm::fit_warm(&rec, &out, write.as_deref())?;
            let law = &r.law;
            println!(
                "fitted: drive gain {}; t_max {:.4}; make-up reference {:.2} dBFS RMS",
                law.points()
                    .iter()
                    .map(|p| format!("{}%={:.3}", p.0, p.1))
                    .collect::<Vec<_>>()
                    .join(" "),
                law.t_max,
                law.makeup_ref_dbfs_rms
            );
            for n in &r.notes {
                println!("note: {n}");
            }
            match r.worst() {
                Some(c) => println!(
                    "{}: worst THD delta {:+.2} dB ({} {}), tolerance {} dB",
                    if r.pass() { "PASS" } else { "FAIL" },
                    c.delta(),
                    c.scene,
                    c.signal,
                    warm::TOL_DB
                ),
                None => println!("no graded points"),
            }
            println!("report: {}", out.join("warm-fit.md").display());
            if let Some(w) = write {
                println!(
                    "calibration written to {} (rebuild to use it: make build / make app)",
                    w.display()
                );
            }
            Ok(())
        }
        "docs" => {
            match f.get("--update") {
                Some(doc) => {
                    update_doc(&doc, "scenes", &scenes_table())?;
                    println!("scene table written to {doc}");
                }
                None => print!("{}", scenes_table()),
            }
            Ok(())
        }
        _ => Err(USAGE.into()),
    }
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("wing-verify: {e}");
            ExitCode::FAILURE
        }
    }
}
