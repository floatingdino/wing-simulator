// Shared build script body for wing-sim, wing-render and wing-diff (T087): each app's `build.rs` is
// `include!("../build_info.rs");`. Exports `WING_GIT_HASH` (short commit, `-dirty` when tracked files
// have uncommitted changes, `unknown` without git), `WING_BUILD_UNIX` and `WING_BUILD_TIME` (UTC,
// minute resolution) so every binary can say exactly which source it was built from. No dependencies.
use std::process::Command;

fn git(args: &[&str]) -> Option<String> {
    let out = Command::new("git").args(args).output().ok()?;
    out.status
        .success()
        .then(|| String::from_utf8_lossy(&out.stdout).trim().to_string())
}

/// Days since 1970-01-01 to (year, month, day), proleptic Gregorian (Hinnant's algorithm).
fn civil(days: i64) -> (i64, i64, i64) {
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    (yoe + era * 400 + i64::from(m <= 2), m, d)
}

fn main() {
    let hash = git(&["rev-parse", "--short", "HEAD"]).filter(|h| !h.is_empty());
    let dirty =
        git(&["status", "--porcelain", "--untracked-files=no"]).is_some_and(|s| !s.is_empty());
    let hash = match hash {
        Some(h) if dirty => format!("{h}-dirty"),
        Some(h) => h,
        None => "unknown".into(),
    };
    let unix: i64 = std::env::var("SOURCE_DATE_EPOCH")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or_else(|| {
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map_or(0, |d| d.as_secs() as i64)
        });
    let (y, mo, d) = civil(unix.div_euclid(86_400));
    let secs = unix.rem_euclid(86_400);
    println!("cargo:rustc-env=WING_GIT_HASH={hash}");
    println!("cargo:rustc-env=WING_BUILD_UNIX={unix}");
    println!(
        "cargo:rustc-env=WING_BUILD_TIME={y:04}-{mo:02}-{d:02}T{:02}:{:02}Z",
        secs / 3600,
        secs % 3600 / 60
    );
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=src");
    println!("cargo:rerun-if-changed=../build_info.rs");
    for p in ["HEAD", "index"] {
        if let Some(g) = git(&["rev-parse", "--git-path", p]) {
            println!("cargo:rerun-if-changed={g}");
        }
    }
}
