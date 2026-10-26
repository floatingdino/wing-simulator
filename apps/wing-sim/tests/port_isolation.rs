//! T089: a daemon started with a private `--port` must never bind UDP/TCP 2222 or UDP 2223
//! (a test run once answered the user's WING-Edit through discovery on 2222). The daemon is
//! started as a child process on ephemeral ports and its "listening:" log line is checked.

use std::io::{BufRead, BufReader};
use std::process::{Command, Stdio};

/// Start the daemon with `extra` flags, return the "listening:" line, then stop it.
fn listening_line(extra: &[&str]) -> String {
    let dir = std::env::temp_dir().join(format!(
        "wingsim-iso-{}-{}",
        std::process::id(),
        extra.join("_").replace(['-', '/'], "")
    ));
    let mut child = Command::new(env!("CARGO_BIN_EXE_wing-sim"))
        .args([
            "--port",
            "0",
            "--osc-port",
            "0",
            "--no-audio",
            "--no-control",
            "--no-autosave",
            "--state-dir",
        ])
        .arg(&dir)
        .args(extra)
        .stderr(Stdio::piped())
        .stdout(Stdio::null())
        .spawn()
        .expect("spawn wing-sim");
    let mut found = None;
    for line in BufReader::new(child.stderr.take().unwrap()).lines() {
        let line = line.unwrap();
        if line.contains("listening:") {
            found = Some(line);
            break;
        }
    }
    let _ = child.kill();
    let _ = child.wait();
    let _ = std::fs::remove_dir_all(&dir);
    found.expect("daemon never logged 'listening:'")
}

#[test]
fn private_port_does_not_bind_2222_or_2223() {
    let l = listening_line(&[]);
    assert!(l.contains("discovery udp "), "{l}");
    assert!(!l.contains(":2222") && !l.contains(":2223"), "{l}");
}

#[test]
fn no_discovery_binds_no_discovery_socket() {
    let l = listening_line(&["--no-discovery"]);
    assert!(l.contains("discovery off"), "{l}");
    assert!(!l.contains(":2222") && !l.contains(":2223"), "{l}");
}

#[test]
fn explicit_discovery_port_is_honoured() {
    let l = listening_line(&["--discovery-port", "0"]);
    assert!(l.contains("discovery udp "), "{l}");
    assert!(!l.contains(":2222"), "{l}");
}
