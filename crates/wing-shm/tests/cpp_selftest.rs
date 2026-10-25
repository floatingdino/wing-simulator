//! Cross-process test: the C++ driver-side harness (`wing_shm_selftest`, which
//! drives the real `driver/src/ShmLink.cpp`) against the Rust daemon side.
//!
//! This test has its own `main` (harness = false). Run normally it compiles
//! the selftest with the system C++ compiler (no cmake/libASPL needed) and
//! runs it with `--client <this executable> --record-only`. The selftest then
//! spawns this executable with `--name ...`, which makes it act as the Rust
//! `gain_client` daemon (same code as `examples/gain_client.rs`).

use std::path::{Path, PathBuf};
use std::process::Command;

#[path = "../examples/gain_client.rs"]
mod gain_client;

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.iter().any(|a| a == "--name") {
        // Spawned by wing_shm_selftest as the daemon.
        std::process::exit(gain_client::run(args));
    }

    if !cfg!(target_os = "macos") {
        println!("cpp_selftest: skipped (macOS only)");
        return;
    }

    let driver = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../driver");
    let exe = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join("wing_shm_selftest");
    let status = Command::new("c++")
        .args([
            "-std=c++17",
            "-O2",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-Wno-unused-parameter",
        ])
        .arg("-I")
        .arg(driver.join("include"))
        .arg("-I")
        .arg(driver.join("src"))
        .arg(driver.join("tools/wing_shm_selftest.cpp"))
        .arg(driver.join("src/ShmLink.cpp"))
        .arg("-o")
        .arg(&exe)
        .status()
        .expect("run c++ (Xcode command line tools)");
    assert!(status.success(), "compiling wing_shm_selftest failed");

    let me = std::env::current_exe().expect("current_exe");
    let out = Command::new(&exe)
        .arg("--client")
        .arg(&me)
        .arg("--record-only")
        .output()
        .expect("run wing_shm_selftest");
    print!("{}", String::from_utf8_lossy(&out.stdout));
    eprint!("{}", String::from_utf8_lossy(&out.stderr));
    let stdout = String::from_utf8_lossy(&out.stdout);
    assert!(
        out.status.success() && stdout.contains("ALL PASSED"),
        "C++ selftest against the Rust daemon failed"
    );
    assert!(
        stdout.contains("PASS  record-only"),
        "record-only phase did not run"
    );
    println!("cpp_selftest: ok");
}
