//! Layout consistency: compile `driver/include/wing_shm.h` with the system C
//! compiler (as C11 and as C++17, so both static-assert paths run), print
//! every field offset, size and constant, and compare with the Rust mirror.

use std::mem::{offset_of, size_of};
use std::path::{Path, PathBuf};
use std::process::Command;

use wing_shm::layout::*;

/// (C expression, Rust value) pairs. Every field of both structs is listed.
fn table() -> Vec<(&'static str, u64)> {
    macro_rules! hdr {
        ($($f:ident),*) => {
            vec![$((concat!("offsetof(wing_shm_header, ", stringify!($f), ")"),
                    offset_of!(Header, $f) as u64)),*]
        };
    }
    macro_rules! ring {
        ($($f:ident),*) => {
            vec![$((concat!("offsetof(wing_shm_ring_ctl, ", stringify!($f), ")"),
                    offset_of!(RingCtl, $f) as u64)),*]
        };
    }
    let mut t = hdr!(
        magic,
        version,
        header_bytes,
        channels,
        ring_frames,
        input_delay_frames,
        total_bytes,
        to_daemon_offset,
        from_daemon_offset,
        segment_id,
        creator_pid,
        io_epoch,
        sample_rate,
        io_running,
        loopback_active,
        cycle_seq,
        input_frames,
        driver_heartbeat,
        driver_pid,
        input_time,
        daemon_heartbeat,
        daemon_pid,
        daemon_waiting,
        to_daemon,
        from_daemon
    );
    t.extend(ring!(write_time, epoch, read_time, overruns, underruns));
    t.extend([
        ("sizeof(wing_shm_header)", size_of::<Header>() as u64),
        ("sizeof(wing_shm_ring_ctl)", size_of::<RingCtl>() as u64),
        ("WING_SHM_MAGIC", MAGIC as u64),
        ("WING_SHM_VERSION", VERSION as u64),
        ("WING_SHM_CHANNELS", CHANNELS as u64),
        ("WING_SHM_RING_FRAMES", RING_FRAMES as u64),
        ("WING_SHM_HEADER_BYTES", HEADER_BYTES as u64),
        ("WING_SHM_CACHE_LINE", CACHE_LINE as u64),
        (
            "WING_SHM_DEFAULT_INPUT_DELAY_FRAMES",
            DEFAULT_INPUT_DELAY_FRAMES as u64,
        ),
        ("WING_SHM_HEARTBEAT_TIMEOUT_NS", HEARTBEAT_TIMEOUT_NS),
        ("WING_SHM_RING_GUARD_FRAMES", RING_GUARD_FRAMES as u64),
        ("WING_SHM_RING_BYTES", RING_BYTES as u64),
        ("WING_SHM_TOTAL_BYTES", TOTAL_BYTES as u64),
        ("sizeof(WING_SHM_NAME) - 1", SHM_NAME.len() as u64),
    ]);
    t
}

fn c_source(t: &[(&str, u64)]) -> String {
    let mut s = String::from(
        "#include \"wing_shm.h\"\n#include <stdio.h>\n#include <string.h>\nint main(void) {\n",
    );
    for (expr, _) in t {
        s += &format!("  printf(\"%llu\\n\", (unsigned long long)({expr}));\n");
    }
    s += &format!(
        "  return strcmp(WING_SHM_NAME, \"{}\") != 0;\n}}\n",
        SHM_NAME
    );
    s
}

fn compile_and_run(lang: &str, std: &str, src: &Path, out: &Path) -> Vec<u64> {
    let include = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../driver/include");
    let status = Command::new("cc")
        .args(["-x", lang, std, "-Wall", "-Wextra", "-Werror", "-I"])
        .arg(&include)
        .arg(src)
        .arg("-o")
        .arg(out)
        .status()
        .expect("run cc (Xcode command line tools)");
    assert!(status.success(), "compiling wing_shm.h as {lang} failed");
    let run = Command::new(out).output().expect("run layout probe");
    assert!(run.status.success(), "WING_SHM_NAME differs from SHM_NAME");
    String::from_utf8(run.stdout)
        .unwrap()
        .lines()
        .map(|l| l.parse().unwrap())
        .collect()
}

#[test]
fn c_header_matches_rust_layout() {
    let t = table();
    let dir = PathBuf::from(env!("CARGO_TARGET_TMPDIR"));
    let src = dir.join("wing_shm_layout_probe.c");
    std::fs::write(&src, c_source(&t)).unwrap();
    for (lang, std, bin) in [
        ("c", "-std=c11", "wing_shm_layout_probe_c"),
        ("c++", "-std=c++17", "wing_shm_layout_probe_cxx"),
    ] {
        let got = compile_and_run(lang, std, &src, &dir.join(bin));
        assert_eq!(got.len(), t.len());
        let diffs: Vec<String> = t
            .iter()
            .zip(&got)
            .filter(|((_, r), c)| r != *c)
            .map(|((e, r), c)| format!("{e}: C {c} != Rust {r}"))
            .collect();
        assert!(
            diffs.is_empty(),
            "{lang} layout mismatch:\n{}",
            diffs.join("\n")
        );
    }
}
