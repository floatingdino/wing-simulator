//! A/B scene compare and console-ready `.snap` export (T062).
//!
//! **A/B.** The daemon holds two in-memory scene slots (`.snap` text). `store_a`/`store_b` capture
//! the current state into a slot and make it the active side; `a`/`b`/`toggle` switch sides:
//! the state of the side being left is captured into its slot first (so edits made while on A
//! are still there when you come back), then the target slot is applied as a **diff** against
//! the live store ([`Snapshot::apply_diff`]): only parameters whose value differs are set, through
//! the normal `ParamStore` set path, so the RT engine is fed by the params pipeline (smoothed
//! on the RT side, no clicks) and every native/OSC client sees the changes through the usual
//! fan-out. Selecting the side you are already on re-applies its stored state (reverts edits).
//!
//! **Export.** [`SceneLib::export`] writes the current state as a firmware 3.x `.snap` for a real
//! console plus `<name>.changes.md`, a `wing-diff` markdown report of what changed since the
//! scene was loaded ("scene as loaded", tracked in [`SceneLib::set_loaded`]). The output is
//! re-loaded and checked (0 rejected keys, 3.x format, semantic round trip) before it is written.
//!
//! Header choice (see docs/research/snapshot-migration.md section 1): the document header is kept
//! when it came from a genuine 3.x console file. A scene that came from the simulator (factory
//! header `3.0.6-193-g60c910ab-dirty:dev_unidyn`/`emulation`, or migrated from 1.x) gets the
//! advertised console identity instead (`creator_fw` = the firmware the simulator advertises,
//! e.g. `3.3.4-0-g0c2b9d4a:release`, plus model/serial/name), because a real console only ever
//! writes release-style headers. `created` is always the export time (UTC). `--export-header
//! factory` writes the 3.0.6 factory header unchanged. The `type` stays `snapshot.11`: the
//! 3.3.4 schema is a superset of 3.0.6 and no other format number is known.

use std::path::{Path, PathBuf};
use std::time::SystemTime;

use wing_model::json::Json;
use wing_model::{ParamStore, Snapshot};

use crate::library::SceneLib;
use crate::persist::atomic_write;

/// Origin tag of changes made by an A/B switch.
pub const ORIGIN_AB: wing_model::Origin = 0x4142_0000;

/// Console-owned state an A/B switch never touches.
const AB_SKIP: [&str; 2] = ["$syscfg", "$ctl/lib"];

/// The two A/B slots.
#[derive(Default)]
pub struct AbState {
    slots: [Option<String>; 2],
    active: Option<usize>,
}

/// Which header an export carries.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum HeaderMode {
    /// Advertised console identity (firmware 3.3.x release header) for simulator-made scenes.
    #[default]
    Advertised,
    /// The 3.0.6 factory header as loaded/saved by the model crate.
    Factory,
}

impl HeaderMode {
    pub fn parse(s: &str) -> Option<HeaderMode> {
        match s {
            "advertised" => Some(HeaderMode::Advertised),
            "factory" => Some(HeaderMode::Factory),
            _ => None,
        }
    }
}

/// Identity written into exported headers.
#[derive(Clone, Debug)]
pub struct ExportCfg {
    pub mode: HeaderMode,
    pub firmware: String,
    pub console_model: String,
    pub serial: String,
    pub name: String,
}

impl Default for ExportCfg {
    fn default() -> Self {
        let id = wing_proto::discovery::Identity::default();
        ExportCfg {
            mode: HeaderMode::Advertised,
            firmware: id.firmware,
            console_model: "wing-fullsize".into(),
            serial: id.serial,
            name: id.name,
        }
    }
}

/// Result of an export.
#[derive(Clone, Debug)]
pub struct Exported {
    pub snap: PathBuf,
    pub report: PathBuf,
    pub bytes: usize,
    /// Leaf keys in the written file.
    pub keys: usize,
    /// Lines in the changes report.
    pub changes: usize,
    pub header_fw: String,
}

/// `YYYY-MM-DD HH:MM:SS` (UTC) for a time.
pub fn timestamp(t: SystemTime) -> String {
    let secs = t
        .duration_since(SystemTime::UNIX_EPOCH)
        .map_or(0, |d| d.as_secs());
    let (days, rem) = ((secs / 86400) as i64, secs % 86400);
    // civil-from-days (Howard Hinnant)
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = yoe + era * 400 + i64::from(m <= 2);
    format!(
        "{y:04}-{m:02}-{d:02} {:02}:{:02}:{:02}",
        rem / 3600,
        rem % 3600 / 60,
        rem % 60
    )
}

fn set_hdr(doc: &mut Snapshot, key: &str, value: &str) {
    match doc.header.iter_mut().find(|(k, _)| k == key) {
        Some(slot) => slot.1 = Json::Str(value.into()),
        None => doc.header.push((key.into(), Json::Str(value.into()))),
    }
}

/// The document with the export header applied (see the module docs).
pub fn export_doc(doc: &Snapshot, cfg: &ExportCfg, now: SystemTime) -> Snapshot {
    let mut d = doc.clone();
    let factory = Snapshot::factory();
    let from_sim = d.header_str("creator_fw") == factory.header_str("creator_fw")
        || d.header_str("creator_version") == Some("emulation");
    if cfg.mode == HeaderMode::Advertised && from_sim {
        set_hdr(&mut d, "creator_fw", &cfg.firmware);
        set_hdr(&mut d, "creator_model", &cfg.console_model);
        set_hdr(&mut d, "creator_sn", &cfg.serial);
        set_hdr(&mut d, "creator_name", &cfg.name);
    }
    set_hdr(&mut d, "created", &timestamp(now));
    d
}

impl SceneLib {
    /// Set the identity/header mode used by exports.
    pub fn set_export_cfg(&self, cfg: ExportCfg) {
        *self.export_cfg.lock().unwrap_or_else(|e| e.into_inner()) = cfg;
    }

    /// Remember the text of the scene the current state started from ("scene as loaded").
    pub fn set_loaded(&self, text: String) {
        *self.loaded.lock().unwrap_or_else(|e| e.into_inner()) = Some(text);
    }

    /// The scene as loaded; the factory scene when nothing was loaded.
    pub fn loaded(&self) -> String {
        self.loaded
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .clone()
            .unwrap_or_else(|| Snapshot::factory().save(&ParamStore::new()))
    }

    fn current_text(&self, store: &ParamStore) -> String {
        self.doc
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .save(store)
    }

    /// Active A/B side (`0` = A, `1` = B).
    #[cfg(test)]
    pub fn ab_active(&self) -> Option<usize> {
        self.ab.lock().unwrap_or_else(|e| e.into_inner()).active
    }

    /// Fill a slot from `.snap` text (validated by loading it) and, with `activate`, make it the
    /// active side without touching the store.
    pub fn ab_set_slot(&self, slot: usize, text: String, activate: bool) -> Result<(), String> {
        let (_, rep) = Snapshot::load(&ParamStore::new(), &text).map_err(|e| e.to_string())?;
        if !rep.rejected.is_empty() {
            return Err(format!("{} rejected keys", rep.rejected.len()));
        }
        let mut ab = self.ab.lock().unwrap_or_else(|e| e.into_inner());
        ab.slots[slot] = Some(text);
        if activate {
            ab.active = Some(slot);
        }
        Ok(())
    }

    /// Capture the current state into `slot` and make it the active side.
    pub fn ab_store(&self, store: &ParamStore, slot: usize) -> String {
        let text = self.current_text(store);
        let mut ab = self.ab.lock().unwrap_or_else(|e| e.into_inner());
        ab.slots[slot] = Some(text);
        ab.active = Some(slot);
        format!("stored slot {}", side(slot))
    }

    /// Switch to `target` (`None`: toggle). Returns a log line.
    pub fn ab_switch(&self, store: &ParamStore, target: Option<usize>) -> Result<String, String> {
        let mut ab = self.ab.lock().unwrap_or_else(|e| e.into_inner());
        let target = target.unwrap_or(match ab.active {
            Some(0) => 1,
            _ => 0,
        });
        let text = ab.slots[target].clone().ok_or_else(|| {
            format!(
                "slot {} is empty (store_{} first)",
                side(target),
                lower(target)
            )
        })?;
        if let Some(cur) = ab.active.filter(|&c| c != target) {
            // keep the edits made while on the side being left
            ab.slots[cur] = Some(self.current_text(store));
        }
        let scratch = ParamStore::new();
        let (snap, rep) = Snapshot::load(&scratch, &text).map_err(|e| e.to_string())?;
        let r = snap.apply_diff(&scratch, store, ORIGIN_AB, &AB_SKIP);
        ab.active = Some(target);
        Ok(format!(
            "switched to {}: {} parameters changed, {} rejected",
            side(target),
            r.changed,
            r.rejected.len() + rep.rejected.len()
        ))
    }

    /// Export the current state to `path` (`.snap`) plus `<path stem>.changes.md`.
    pub fn export(&self, store: &ParamStore, path: &Path) -> Result<Exported, String> {
        let path = if path.extension().is_none() {
            path.with_extension("snap")
        } else {
            path.to_path_buf()
        };
        let cfg = self
            .export_cfg
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .clone();
        let doc = export_doc(
            &self.doc.lock().unwrap_or_else(|e| e.into_inner()),
            &cfg,
            SystemTime::now(),
        );
        let text = doc.save(store);

        // validate: loads back clean as a 3.x scene with the same content
        let chk = ParamStore::new();
        let (back, rep) = Snapshot::load(&chk, &text).map_err(|e| format!("re-load: {e}"))?;
        if !rep.rejected.is_empty() {
            return Err(format!(
                "export check: {} rejected keys (first: {})",
                rep.rejected.len(),
                rep.rejected[0].0
            ));
        }
        if rep.legacy || !rep.creator_fw.starts_with("3.") || rep.format != "snapshot.11" {
            return Err(format!(
                "export check: header is {} / fw {:?}, not a 3.x snapshot.11",
                rep.format, rep.creator_fw
            ));
        }
        let d = wing_model::diff(&doc.entries(store), &back.entries(&chk));
        if !d.is_empty() {
            return Err(format!(
                "export check: {} keys differ after re-load",
                d.len()
            ));
        }

        let md = {
            let a = wing_diff::load_text("scene as loaded", &self.loaded())?;
            let b = wing_diff::load_text("current state", &text)?;
            let r = wing_diff::diff_scenes(&a, &b, &wing_diff::Options::default());
            (
                wing_diff::render(&r, wing_diff::Format::Markdown),
                r.changes(),
            )
        };
        let report = path.with_extension("changes.md");
        if let Some(dir) = path.parent().filter(|d| !d.as_os_str().is_empty()) {
            std::fs::create_dir_all(dir).map_err(|e| format!("{}: {e}", dir.display()))?;
        }
        atomic_write(&path, text.as_bytes()).map_err(|e| format!("{}: {e}", path.display()))?;
        atomic_write(&report, md.0.as_bytes()).map_err(|e| format!("{}: {e}", report.display()))?;
        Ok(Exported {
            snap: path,
            report,
            bytes: text.len(),
            keys: rep.keys,
            changes: md.1,
            header_fw: rep.creator_fw,
        })
    }
}

fn side(slot: usize) -> &'static str {
    ["A", "B"][slot]
}

fn lower(slot: usize) -> &'static str {
    ["a", "b"][slot]
}

/// Parse an `AB:<command>` argument.
pub fn run_ab(lib: &SceneLib, store: &ParamStore, arg: &str) -> Result<String, String> {
    match arg.trim().to_ascii_lowercase().as_str() {
        "store_a" => Ok(lib.ab_store(store, 0)),
        "store_b" => Ok(lib.ab_store(store, 1)),
        "a" => lib.ab_switch(store, Some(0)),
        "b" => lib.ab_switch(store, Some(1)),
        "toggle" => lib.ab_switch(store, None),
        other => Err(format!("unknown A/B command {other:?}")),
    }
}

/// `EXPORT:<path>` action.
pub fn run_export(lib: &SceneLib, store: &ParamStore, arg: &str) -> Result<String, String> {
    let p = arg.trim();
    if p.is_empty() {
        return Err("EXPORT needs a path (EXPORT:/path/mix.snap)".into());
    }
    let e = lib.export(store, Path::new(p))?;
    Ok(format!(
        "exported {} ({} keys, {} bytes, fw {}) and {} ({} changes since load)",
        e.snap.display(),
        e.keys,
        e.bytes,
        e.header_fw,
        e.report.display(),
        e.changes
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::persist::{test_dir, Doc};
    use std::sync::{Arc, Mutex};
    use wing_model::Value;

    fn lib_in(dir: &Path) -> (SceneLib, ParamStore) {
        let doc: Doc = Arc::new(Mutex::new(Snapshot::factory()));
        (
            SceneLib::open(dir.join("scenes"), "wing-fullsize", doc).unwrap(),
            ParamStore::new(),
        )
    }

    #[test]
    fn timestamps() {
        let t = SystemTime::UNIX_EPOCH + std::time::Duration::from_secs(1_760_000_000);
        assert_eq!(timestamp(t), "2025-10-09 08:53:20");
        assert_eq!(timestamp(SystemTime::UNIX_EPOCH), "1970-01-01 00:00:00");
    }

    #[test]
    fn ab_switch_applies_only_diffs_and_keeps_edits() {
        let d = test_dir("ab");
        let (lib, st) = lib_in(&d);
        let fdr = st.lookup("ch/1/fdr").unwrap();
        let mute = st.lookup("ch/2/mute").unwrap();
        let name = st.lookup("ch/1/name").unwrap();
        let mdl = st.lookup("ch/1/dyn/mdl").unwrap();
        st.set(fdr, Value::Float(-5.0)).unwrap();
        lib.ab_store(&st, 0);
        st.set(fdr, Value::Float(-20.0)).unwrap();
        st.set(mute, Value::Int(1)).unwrap();
        st.set_str(name, "Kick", 0).unwrap();
        st.set_enum_name(mdl, "76LA", 0).unwrap();
        lib.ab_store(&st, 1);
        assert_eq!(lib.ab_active(), Some(1));

        let msg = lib.ab_switch(&st, Some(0)).unwrap();
        assert!(msg.contains("switched to A"), "{msg}");
        // a second switch to the same stored state has nothing left to change
        assert!(lib
            .ab_switch(&st, Some(0))
            .unwrap()
            .contains(" 0 parameters changed"));
        assert_eq!(st.get(fdr), Some(Value::Float(-5.0)));
        assert_eq!(st.get(mute), Some(Value::Int(0)));
        assert_eq!(st.with_str(name, str::to_string).as_deref(), Some(""));
        assert_ne!(
            st.active_model(st.lookup("ch/1/dyn").unwrap())
                .unwrap()
                .name,
            "76LA"
        );
        // edit on A, toggle to B and back: the edit survived
        st.set(fdr, Value::Float(-7.0)).unwrap();
        lib.ab_switch(&st, None).unwrap();
        assert_eq!(lib.ab_active(), Some(1));
        assert_eq!(st.get(fdr), Some(Value::Float(-20.0)));
        assert_eq!(st.get(mute), Some(Value::Int(1)));
        assert_eq!(st.with_str(name, str::to_string).as_deref(), Some("Kick"));
        assert_eq!(
            st.active_model(st.lookup("ch/1/dyn").unwrap())
                .unwrap()
                .name,
            "76LA"
        );
        lib.ab_switch(&st, None).unwrap();
        assert_eq!(st.get(fdr), Some(Value::Float(-7.0)));
        // selecting the side you are on reverts to its stored state
        lib.ab_switch(&st, Some(0)).unwrap();
        assert_eq!(st.get(fdr), Some(Value::Float(-7.0)));
        // an empty slot is an error and changes nothing
        let (l2, s2) = lib_in(&test_dir("ab2"));
        assert!(l2.ab_switch(&s2, None).is_err());
        assert!(run_ab(&lib, &st, "bogus").is_err());
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn export_validates_and_reports_changes() {
        let d = test_dir("export");
        let (lib, st) = lib_in(&d);
        lib.set_loaded(lib.current_text(&st));
        let fdr = st.lookup("ch/3/fdr").unwrap();
        st.set(fdr, Value::Float(-3.5)).unwrap();
        let p = d.join("out/mix.snap");
        let e = lib.export(&st, &p).unwrap();
        assert_eq!(e.snap, p);
        assert_eq!(e.report, d.join("out/mix.changes.md"));
        assert!(e.changes >= 1);
        let md = std::fs::read_to_string(&e.report).unwrap();
        assert!(
            md.contains("scene as loaded") && md.contains("Ch 3"),
            "{md}"
        );
        // header: advertised firmware replaces the factory dev build
        let text = std::fs::read_to_string(&p).unwrap();
        let (snap, rep) = Snapshot::load(&ParamStore::new(), &text).unwrap();
        assert!(rep.rejected.is_empty() && !rep.legacy);
        assert_eq!(rep.creator_fw, "3.3.4-0-g0c2b9d4a:release");
        assert_eq!(snap.header_str("type"), Some("snapshot.11"));
        assert_ne!(snap.header_str("created"), Some("2025-10-02 14:27:23"));
        // factory mode keeps the 3.0.6 header
        lib.set_export_cfg(ExportCfg {
            mode: HeaderMode::Factory,
            ..ExportCfg::default()
        });
        let e = lib.export(&st, &d.join("f")).unwrap(); // `.snap` added
        assert!(e.snap.ends_with("f.snap"));
        assert_eq!(e.header_fw, "3.0.6-193-g60c910ab-dirty:dev_unidyn");
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn export_keeps_a_genuine_3x_header_and_migrates_1x() {
        let d = test_dir("export1x");
        let (lib, st) = lib_in(&d);
        // a 1.x scene: loaded through the library path, exported as 3.x
        let text = std::fs::read_to_string(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../fixtures/PUBLIC.snap"
        ))
        .unwrap();
        let (snap, _) = Snapshot::load(&st, &text).unwrap();
        *lib.doc.lock().unwrap() = snap;
        lib.set_loaded(text);
        let e = lib.export(&st, &d.join("pub.snap")).unwrap();
        assert_eq!(
            e.changes,
            0,
            "{}",
            std::fs::read_to_string(&e.report).unwrap()
        );
        assert_eq!(e.header_fw, "3.3.4-0-g0c2b9d4a:release");
        // a real console header stays as is
        let mut doc = Snapshot::factory();
        set_hdr(&mut doc, "creator_fw", "3.3.1-5-gdeadbeef:release");
        set_hdr(&mut doc, "creator_version", "WING-RACK");
        *lib.doc.lock().unwrap() = doc;
        let e = lib.export(&st, &d.join("real.snap")).unwrap();
        assert_eq!(e.header_fw, "3.3.1-5-gdeadbeef:release");
        let _ = std::fs::remove_dir_all(&d);
    }

    /// T062 acceptance: A/B over OSC (ephemeral loopback ports) reaches a native-protocol client
    /// (the WING-Edit transport) through the server fan-out, and `/wingsim/export` writes a
    /// file that loads back clean.
    #[test]
    fn osc_ab_reaches_native_client_and_export_loopback() {
        use std::io::Read;
        use std::net::{Ipv4Addr, SocketAddrV4, TcpStream, UdpSocket};
        use std::time::{Duration, Instant};
        use wing_proto::nrp::{NrpDecoder, Token};
        use wing_proto::osc::{Arg, Message, OscAttachment, OscConfig, OscServer};
        use wing_proto::server::{
            EventFn, Hub, LiveInfo, NativeServer, ServerConfig, ServerShared,
        };

        let d = test_dir("abnet");
        let doc: Doc = Arc::new(Mutex::new(Snapshot::factory()));
        let lib = Arc::new(SceneLib::open(d.join("scenes"), "wing-fullsize", doc).unwrap());
        let mut store = ParamStore::new();
        let hub = Hub::attach(&mut store);
        let att = OscAttachment::attach(&mut store);
        let store = Arc::new(store);
        lib.set_loaded(lib.current_text(&store));
        let svc =
            crate::library::LibraryService::spawn(lib.clone(), store.clone(), Arc::new(|_| {}))
                .unwrap();
        let on_event: EventFn = Arc::new(|_| {});
        let server = NativeServer::bind(
            ServerConfig {
                bind: SocketAddrV4::new(Ipv4Addr::LOCALHOST, 0),
                idle_timeout: None,
                live: LiveInfo {
                    library: Some(svc.clone()),
                    ..LiveInfo::default()
                },
                ..ServerConfig::default()
            },
            ServerShared {
                store: store.clone(),
                hub,
                meters: None,
            },
            on_event,
        )
        .unwrap()
        .spawn()
        .unwrap();
        let osc = OscServer::bind(
            store.clone(),
            att,
            OscConfig {
                bind: SocketAddrV4::new(Ipv4Addr::LOCALHOST, 0),
                library: Some(svc.clone()),
                ..OscConfig::default()
            },
        )
        .unwrap()
        .spawn()
        .unwrap();

        let fdr = store.lookup("ch/1/fdr").unwrap();
        let hash = store.schema().hash(fdr);
        let mut native = TcpStream::connect(server.local_addr()).unwrap();
        native
            .set_read_timeout(Some(Duration::from_millis(100)))
            .unwrap();
        std::thread::sleep(Duration::from_millis(100));
        let sock = UdpSocket::bind("127.0.0.1:0").unwrap();
        let send = |addr: &str, args: Vec<Arg>| {
            sock.send_to(&Message::new(addr, args).to_vec(), osc.local_addr())
                .unwrap();
        };
        let wait = |what: &str, f: &dyn Fn() -> bool| {
            let t = Instant::now();
            while !f() {
                // generous: each A/B switch re-serialises a full scene, slow in debug on CI
                assert!(t.elapsed() < Duration::from_secs(30), "timeout: {what}");
                std::thread::sleep(Duration::from_millis(5));
            }
        };
        // the native client collects (hash, value) change events
        let mut dec = NrpDecoder::new();
        dec.feed(&[0xDF, 0xD1]);
        let mut seen: Vec<(u32, f32)> = Vec::new();
        let pump = |native: &mut TcpStream, dec: &mut NrpDecoder, seen: &mut Vec<(u32, f32)>| {
            let mut buf = [0u8; 65536];
            if let Ok(n) = native.read(&mut buf) {
                dec.feed(&buf[..n]);
            }
            let mut cur = None;
            while let Some(Ok(t)) = dec.next_control() {
                match t {
                    Token::NavHash(h) => cur = Some(h),
                    Token::Float(f) => {
                        if let Some(h) = cur.take() {
                            seen.push((h, f));
                        }
                    }
                    _ => {}
                }
            }
        };

        send("/ch/1/fdr", vec![Arg::Float(-5.0)]);
        wait("fader A", &|| store.get(fdr) == Some(Value::Float(-5.0)));
        send("/wingsim/ab/store_a", vec![]);
        wait("A stored", &|| lib.ab_active() == Some(0));
        send("/ch/1/fdr", vec![Arg::Float(-20.0)]);
        wait("fader B", &|| store.get(fdr) == Some(Value::Float(-20.0)));
        send("/wingsim/ab/store_b", vec![]);
        wait("B stored", &|| lib.ab_active() == Some(1));

        let t = Instant::now();
        seen.clear();
        send("/wingsim/ab/toggle", vec![]);
        wait("toggle to A", &|| {
            store.get(fdr) == Some(Value::Float(-5.0))
        });
        while !seen.contains(&(hash, -5.0)) && t.elapsed() < Duration::from_secs(30) {
            pump(&mut native, &mut dec, &mut seen);
        }
        assert!(
            seen.contains(&(hash, -5.0)),
            "native client missed the A/B change: {seen:?}"
        );
        // only the differing parameter was set
        assert!(seen.len() < 20, "{} events", seen.len());
        send("/wingsim/ab/b", vec![]);
        wait("to B", &|| store.get(fdr) == Some(Value::Float(-20.0)));

        // export through OSC
        let out = d.join("exp/mix.snap");
        send("/wingsim/export", vec![Arg::Str(out.display().to_string())]);
        wait("export", &|| {
            out.exists() && out.with_extension("changes.md").exists()
        });
        let text = std::fs::read_to_string(&out).unwrap();
        let (_, rep) = Snapshot::load(&ParamStore::new(), &text).unwrap();
        assert!(rep.rejected.is_empty() && !rep.legacy && rep.creator_fw.starts_with("3."));
        let md = std::fs::read_to_string(out.with_extension("changes.md")).unwrap();
        assert!(md.contains("-20"), "{md}");

        osc.stop();
        server.stop();
        svc.stop();
        let _ = std::fs::remove_dir_all(&d);
    }
}
