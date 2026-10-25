//! T065: consistency checks for the REAPER integration files in `integrations/reaper/`.
//!
//! - every OSC address in the `.ReaperOSC` pattern configs is one the daemon's OSC service
//!   handles (no `NODE NOT FOUND`), and marker feedback reaches the scene library;
//! - every `WING:` marker in the demo project names a scene target;
//! - the Lua scripts pass `luac -p` when a Lua 5.3/5.4 compiler is on PATH (skipped otherwise:
//!   REAPER embeds Lua, so none is required to use the scripts).

use std::net::{Ipv4Addr, SocketAddr};
use std::path::PathBuf;
use std::process::Command;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use wing_model::{Origin, ParamStore};
use wing_proto::discovery::Identity;
use wing_proto::osc::service::{marker_target, SubState};
use wing_proto::osc::{Arg, Message, OscService};
use wing_proto::server::SceneLibrary;

fn reaper_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../integrations/reaper")
}

#[derive(Default)]
struct Lib(Mutex<Vec<String>>);

impl SceneLibrary for Lib {
    fn scenes(&self) -> Vec<String> {
        Vec::new()
    }
    fn action(&self, action: &str, _index: i32, _origin: Origin) {
        self.0.lock().unwrap().push(action.into());
    }
}

/// `(type, address)` pairs of the pattern lines (`NAME t/addr [t/addr ...]`).
fn patterns(text: &str) -> Vec<(char, String)> {
    let mut v = Vec::new();
    for line in text.lines().map(str::trim) {
        if line.starts_with('#') || line.starts_with("DEVICE_") || line.is_empty() {
            continue;
        }
        for p in line.split_whitespace().skip(1) {
            let (t, a) = p.split_at(1);
            v.push((t.chars().next().unwrap(), a.to_string()));
        }
    }
    v
}

#[test]
fn pattern_addresses_are_handled() {
    let lib = Arc::new(Lib::default());
    let svc = OscService::new(
        Arc::new(ParamStore::new()),
        Identity::default(),
        Some(Ipv4Addr::LOCALHOST),
        Duration::from_secs(10),
        Arc::new(SubState::default()),
    )
    .with_library(Some(lib.clone()));
    let from: SocketAddr = "127.0.0.1:9000".parse().unwrap();
    let mut n = 0;
    for f in ["WingSim-Markers.ReaperOSC", "WingSim-Faders.ReaperOSC"] {
        let text = std::fs::read_to_string(reaper_dir().join(f)).unwrap();
        for (ty, addr) in patterns(&text) {
            let addr = addr.replace('@', "1");
            let arg = match ty {
                's' => Arg::Str(format!("WING:Scene {n}")),
                'f' => Arg::Float(-6.0),
                'b' => Arg::Float(1.0),
                other => panic!("{f}: unexpected pattern type {other}"),
            };
            let r = svc.handle(
                &Message::new(&addr, vec![arg]).to_vec(),
                from,
                Instant::now(),
            );
            assert!(r.is_empty(), "{f}: {addr} -> {r:?}");
            n += 1;
        }
    }
    assert!(n >= 6);
    let acts = lib.0.lock().unwrap().clone();
    assert_eq!(acts.len(), 4, "{acts:?}"); // two marker + two region addresses
    assert!(acts.iter().all(|a| a.starts_with("GO:Scene ")), "{acts:?}");
}

#[test]
fn demo_project_markers() {
    let text = std::fs::read_to_string(reaper_dir().join("demo/WingSim-Demo.RPP")).unwrap();
    let names: Vec<String> = text
        .lines()
        .map(str::trim)
        .filter(|l| l.starts_with("MARKER "))
        .filter_map(|l| Some(l.split('"').nth(1)?.to_string()))
        .collect();
    let targets: Vec<&str> = names.iter().filter_map(|n| marker_target(n)).collect();
    assert_eq!(targets, ["Intro", "Verse", "Chorus", "Bridge", "#1"]);
}

#[test]
fn lua_scripts_compile() {
    let lua: Vec<PathBuf> = std::fs::read_dir(reaper_dir())
        .unwrap()
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.extension().is_some_and(|x| x == "lua"))
        .collect();
    assert!(lua.len() >= 5, "{lua:?}");
    let Some(luac) = ["luac5.4", "luac54", "luac5.3", "luac"]
        .into_iter()
        .find(|c| Command::new(c).arg("-v").output().is_ok())
    else {
        eprintln!("lua_scripts_compile: no luac on PATH, syntax check skipped");
        return;
    };
    for f in &lua {
        let out = Command::new(luac).args(["-p"]).arg(f).output().unwrap();
        assert!(
            out.status.success(),
            "{}: {}",
            f.display(),
            String::from_utf8_lossy(&out.stderr)
        );
    }
}
