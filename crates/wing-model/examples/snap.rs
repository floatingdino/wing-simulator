//! `.snap` tool: load/inspect/convert scenes and diff two scenes (basis for T063).
//!
//! ```text
//! cargo run -p wing-model --example snap -- load fixtures/PUBLIC.snap [--dump ch/1] [--report] [--save out.snap]
//! cargo run -p wing-model --example snap -- diff a.snap b.snap
//! ```
//! `load` prints the load report (1.x files are migrated to 3.x); `--dump <prefix>` lists the
//! saved 3.x entries under a tree prefix with the console's display text; `--report` lists every
//! migration note and unmapped/rejected key; `--save` writes the 3.x file. `diff` compares two
//! scenes after loading both (so a 1.x and a 3.x scene compare in 3.x terms).

use std::process::ExitCode;

use wing_model::{diff, DiffEntry, LoadReport, ParamStore, Snapshot};

fn usage() -> ExitCode {
    eprintln!(
        "usage: snap load <file.snap> [--dump <prefix>] [--report] [--save <out.snap>]\n       snap diff <a.snap> <b.snap>"
    );
    ExitCode::from(2)
}

fn load(path: &str) -> Result<(ParamStore, Snapshot, LoadReport), String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("{path}: {e}"))?;
    let store = ParamStore::new();
    let (snap, report) = Snapshot::load(&store, &text).map_err(|e| format!("{path}: {e}"))?;
    Ok((store, snap, report))
}

fn summary(path: &str, r: &LoadReport) {
    println!(
        "{path}: {} (fw {}){}",
        r.format,
        r.creator_fw,
        if r.legacy {
            ", legacy: migrated to 3.x"
        } else {
            ""
        }
    );
    println!(
        "  keys {}  applied {}  migrated {} ({} lossy)  preserved {}  unmapped {}  rejected {}  adjusted {}",
        r.keys,
        r.applied,
        r.migrated.len(),
        r.lossy().count(),
        r.preserved.len(),
        r.unmapped.len(),
        r.rejected.len(),
        r.adjusted.len()
    );
}

fn under(path: &str, prefix: &str) -> bool {
    let p = prefix.trim_matches('/');
    p.is_empty() || path == p || path.strip_prefix(p).is_some_and(|r| r.starts_with('/'))
}

fn cmd_load(args: &[String]) -> Result<(), String> {
    let file = args.first().ok_or("missing file")?;
    let (mut dump, mut save, mut report) = (None, None, false);
    let mut it = args[1..].iter();
    while let Some(a) = it.next() {
        match a.as_str() {
            "--dump" => dump = Some(it.next().ok_or("--dump needs a prefix")?.clone()),
            "--save" => save = Some(it.next().ok_or("--save needs a file")?.clone()),
            "--report" => report = true,
            _ => return Err(format!("unknown option {a}")),
        }
    }
    let (store, snap, r) = load(file)?;
    summary(file, &r);
    if report {
        for n in &r.migrated {
            println!(
                "  migrate [{}{}] {} -> {}",
                n.rule,
                if n.lossy { ", LOSSY" } else { "" },
                n.from,
                if n.to.is_empty() { "(dropped)" } else { &n.to }
            );
        }
        for p in &r.preserved {
            println!("  preserved {p}");
        }
        for p in &r.unmapped {
            println!("  UNMAPPED {p}");
        }
        for (p, e) in &r.rejected {
            println!("  REJECTED {p}: {e}");
        }
        for (p, from, to) in &r.adjusted {
            println!("  adjusted {p}: {from} -> {to}");
        }
    }
    if let Some(prefix) = dump {
        for (path, v) in snap.entries(&store) {
            if !under(&path, &prefix) {
                continue;
            }
            let mut disp = String::new();
            if let Some(id) = store
                .lookup(&path)
                .filter(|&id| store.schema().is_param(id))
            {
                let _ = store.format(id, &mut disp);
            }
            println!("{path:<32} {:<16} {disp}", v.to_text());
        }
    }
    if let Some(out) = save {
        std::fs::write(&out, snap.save(&store)).map_err(|e| format!("{out}: {e}"))?;
        println!("wrote {out}");
    }
    Ok(())
}

fn cmd_diff(args: &[String]) -> Result<(), String> {
    let [a, b] = args else {
        return Err("diff needs two files".into());
    };
    let (sa, na, ra) = load(a)?;
    let (sb, nb, rb) = load(b)?;
    summary(a, &ra);
    summary(b, &rb);
    let d = diff(&na.entries(&sa), &nb.entries(&sb));
    for e in &d {
        match e {
            DiffEntry::Changed { path, a, b } => {
                println!("~ {path}: {} -> {}", a.to_text(), b.to_text())
            }
            DiffEntry::OnlyA { path, a } => println!("- {path}: {}", a.to_text()),
            DiffEntry::OnlyB { path, b } => println!("+ {path}: {}", b.to_text()),
        }
    }
    println!("{} differences", d.len());
    Ok(())
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let res = match args.first().map(String::as_str) {
        Some("load") => cmd_load(&args[1..]),
        Some("diff") => cmd_diff(&args[1..]),
        _ => return usage(),
    };
    match res {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("snap: {e}");
            ExitCode::FAILURE
        }
    }
}
