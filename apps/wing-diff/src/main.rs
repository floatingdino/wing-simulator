//! `wing-diff a.snap b.snap [--format text|markdown|json] [--filter ch/1..40,bus] [--ignore-ui]`

use std::process::ExitCode;

use wing_diff::{diff_files, render, Filter, Format, Options};

const USAGE: &str = "usage: wing-diff <a.snap> <b.snap> [--format text|markdown|json] [--filter ch/1..40,bus] [--ignore-ui]";

fn run() -> Result<(), String> {
    let mut files = Vec::new();
    let mut format = Format::Text;
    let mut opts = Options::default();
    let mut it = std::env::args().skip(1);
    while let Some(a) = it.next() {
        match a.as_str() {
            "--format" => {
                let v = it.next().ok_or("--format needs a value")?;
                format = Format::parse(&v).ok_or(format!("unknown format {v}"))?;
            }
            "--filter" => {
                let v = it.next().ok_or("--filter needs a value")?;
                opts.filter = Some(Filter::parse(&v)?);
            }
            "--ignore-ui" => opts.ignore_ui = true,
            "--version" | "-V" => {
                println!(
                    "wing-diff {} (commit {}, built {})",
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
            s if s.starts_with("--") => return Err(format!("unknown option {s}")),
            _ => files.push(a),
        }
    }
    let [a, b] = files.as_slice() else {
        return Err(USAGE.into());
    };
    let report = diff_files(a, b, &opts)?;
    print!("{}", render(&report, format));
    Ok(())
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("wing-diff: {e}");
            ExitCode::from(2)
        }
    }
}
