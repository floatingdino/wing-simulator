//! Scene library of the simulated show (T017), backed by `.snap` files in
//! `<state dir>/scenes/` and exposed through `$ctl/lib` (see `wing_proto::server::library` for
//! the node semantics and what is inferred [H]).
//!
//! - Files are named `NNN Name.snap`; `NNN` fixes the show order (new scenes get the next
//!   number). `.snap` files dropped in by hand without a number are listed after them, by name.
//! - The simulator has one show, [`SHOW_NAME`], published as `$ctl/lib/$actshow`.
//! - Recall loads the file through `Snapshot::load` into the shared `ParamStore` (the normal set
//!   path, so every client sees the changes; T03A stages it for the RT engine), replaces the
//!   current document and publishes `$active`/`$activeid`.
//! - `$action` commands: `GO` recalls `$actionidx`; `GONEXT`/`GOPREV` recall the scene after/
//!   before the active one; `NEXT`/`PREV` move `$actionidx` without recalling; `GOTAG` is not
//!   supported (no scene tags). Simulator extensions: `STORE[:name]`, `RENAME:name`, `DELETE`.
//! - All file IO runs on the library worker thread ([`LibraryService`]), never on a client
//!   connection thread.

use std::fs;
use std::io;
use std::path::{Path, PathBuf};
use std::sync::mpsc::{self, Sender};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;

use wing_model::json::Json;
use wing_model::{LoadReport, Origin, ParamStore, Snapshot, Value, ORIGIN_LOCAL};
use wing_proto::server::SceneLibrary;

use crate::persist::{atomic_write, Doc, LogFn};

/// Name of the single simulated show (`$ctl/lib/$actshow`).
pub const SHOW_NAME: &str = "SIMULATOR";
/// Longest scene name accepted.
pub const MAX_NAME: usize = 64;

#[derive(Clone, Debug, PartialEq)]
struct Entry {
    seq: Option<u32>,
    name: String,
    file: PathBuf,
}

/// The scene library (synchronous; file IO happens on the calling thread).
pub struct SceneLib {
    dir: PathBuf,
    console_model: String,
    doc: Doc,
    entries: Mutex<Vec<Entry>>,
    active: Mutex<Option<String>>,
}

fn parse_entry(path: &Path) -> Option<Entry> {
    let fname = path.file_name()?.to_str()?;
    if fname.starts_with('.') || path.extension()? != "snap" {
        return None;
    }
    let stem = path.file_stem()?.to_str()?;
    let digits = stem.bytes().take_while(u8::is_ascii_digit).count();
    let (seq, name) = match stem[digits..].strip_prefix(' ') {
        Some(rest) if digits > 0 && !rest.is_empty() => (stem[..digits].parse().ok(), rest),
        _ => (None, stem),
    };
    Some(Entry {
        seq,
        name: name.to_string(),
        file: path.to_path_buf(),
    })
}

/// Check a scene name for use as a file name.
pub fn validate_name(name: &str) -> Result<&str, String> {
    let n = name.trim();
    if n.is_empty() {
        return Err("empty scene name".into());
    }
    if n.len() > MAX_NAME {
        return Err(format!("scene name longer than {MAX_NAME} bytes"));
    }
    if n.starts_with('.')
        || n.chars()
            .any(|c| c.is_control() || matches!(c, '/' | '\\' | ':'))
    {
        return Err(format!("invalid scene name {n:?}"));
    }
    Ok(n)
}

fn set_header(doc: &mut Snapshot, key: &str, value: &str) {
    match doc.header.iter_mut().find(|(k, _)| k == key) {
        Some(slot) => slot.1 = Json::Str(value.into()),
        None => doc.header.push((key.into(), Json::Str(value.into()))),
    }
}

impl SceneLib {
    /// Open (and create) the library directory. `doc` is the shared current document;
    /// `console_model` is re-applied to `$syscfg/$cnsmdl` after each recall.
    pub fn open(dir: PathBuf, console_model: &str, doc: Doc) -> io::Result<SceneLib> {
        fs::create_dir_all(&dir)?;
        let lib = SceneLib {
            dir,
            console_model: console_model.into(),
            doc,
            entries: Mutex::new(Vec::new()),
            active: Mutex::new(None),
        };
        lib.rescan()?;
        Ok(lib)
    }

    pub fn dir(&self) -> &Path {
        &self.dir
    }

    /// Re-read the directory.
    pub fn rescan(&self) -> io::Result<()> {
        let mut v: Vec<Entry> = fs::read_dir(&self.dir)?
            .filter_map(|e| e.ok())
            .filter(|e| e.file_type().is_ok_and(|t| t.is_file()))
            .filter_map(|e| parse_entry(&e.path()))
            .collect();
        v.sort_by(|a, b| (a.seq.is_none(), a.seq, &a.name).cmp(&(b.seq.is_none(), b.seq, &b.name)));
        *self.entries.lock().unwrap_or_else(|e| e.into_inner()) = v;
        Ok(())
    }

    /// Scene names in show order.
    pub fn names(&self) -> Vec<String> {
        self.entries
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .iter()
            .map(|e| e.name.clone())
            .collect()
    }

    /// Scene files in show order.
    pub fn files(&self) -> Vec<PathBuf> {
        self.entries
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .iter()
            .map(|e| e.file.clone())
            .collect()
    }

    /// Position of a scene by name (exact, then case-insensitive), or by `#N` (0-based).
    pub fn find(&self, name: &str) -> Option<usize> {
        let names = self.names();
        if let Some(i) = names.iter().position(|n| n == name) {
            return Some(i);
        }
        if let Some(i) = names.iter().position(|n| n.eq_ignore_ascii_case(name)) {
            return Some(i);
        }
        let i: usize = name.strip_prefix('#')?.parse().ok()?;
        (i < names.len()).then_some(i)
    }

    /// The active scene (last recalled or stored).
    pub fn active(&self) -> Option<String> {
        self.active
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .clone()
    }

    fn set_active(&self, name: Option<String>) {
        *self.active.lock().unwrap_or_else(|e| e.into_inner()) = name;
    }

    /// Take the active scene from the current document's `active_scene` header when that scene
    /// exists (after restoring the autosave).
    pub fn adopt_doc_active(&self) {
        let a = self
            .doc
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .header_str("active_scene")
            .map(str::to_string);
        let a = a.filter(|a| self.names().iter().any(|n| n == a));
        self.set_active(a);
    }

    /// Publish the library state to the console-owned `$ctl/lib` leaves and re-apply console
    /// values a scene load resets (`$syscfg/$cnsmdl`).
    pub fn publish(&self, store: &ParamStore) {
        let set_str = |p: &str, v: &str| {
            if let Some(id) = store.lookup(p) {
                let _ = store.set_str(id, v, ORIGIN_LOCAL);
            }
        };
        set_str("$syscfg/$cnsmdl", &self.console_model);
        set_str("$ctl/lib/$actshow", SHOW_NAME);
        let active = self.active();
        set_str("$ctl/lib/$active", active.as_deref().unwrap_or(""));
        let idx = active.and_then(|a| self.names().iter().position(|n| *n == a));
        if let Some(id) = store.lookup("$ctl/lib/$activeid") {
            let _ = store.set_from(id, Value::Int(idx.unwrap_or(0) as i32), ORIGIN_LOCAL);
        }
    }

    /// Load a scene file into the store: the store is reset and loaded, the document replaced
    /// (header `active_show`/`active_scene` set) and the library state published.
    pub fn recall(&self, store: &ParamStore, idx: usize) -> Result<(String, LoadReport), String> {
        let e = self
            .entries
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .get(idx)
            .cloned()
            .ok_or_else(|| format!("no scene #{idx}"))?;
        let text =
            fs::read_to_string(&e.file).map_err(|err| format!("{}: {err}", e.file.display()))?;
        {
            // hold the document while the store is rewritten so an autosave never pairs a
            // half-loaded store with the old document
            let mut doc = self.doc.lock().unwrap_or_else(|e| e.into_inner());
            let (mut snap, rep) =
                Snapshot::load(store, &text).map_err(|err| format!("{}: {err}", e.name))?;
            set_header(&mut snap, "active_show", SHOW_NAME);
            set_header(&mut snap, "active_scene", &e.name);
            *doc = snap;
            drop(doc);
            self.set_active(Some(e.name.clone()));
            self.publish(store);
            Ok((e.name, rep))
        }
    }

    /// Store the current state as a scene: `name` given: overwrite that scene or append a new
    /// one; `None`: overwrite the active scene, else append `Scene N`. Returns the position.
    pub fn store(&self, store: &ParamStore, name: Option<&str>) -> Result<usize, String> {
        let names = self.names();
        let name = match name {
            Some(n) => validate_name(n)?.to_string(),
            None => match self.active() {
                Some(a) => a,
                None => (names.len() + 1..)
                    .map(|i| format!("Scene {i}"))
                    .find(|n| !names.contains(n))
                    .expect("unbounded"),
            },
        };
        let existing = self
            .entries
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .iter()
            .find(|e| e.name == name)
            .map(|e| e.file.clone());
        let file = existing
            .unwrap_or_else(|| self.dir.join(format!("{:03} {name}.snap", self.next_seq())));
        let text = {
            let mut doc = self.doc.lock().unwrap_or_else(|e| e.into_inner());
            set_header(&mut doc, "active_show", SHOW_NAME);
            set_header(&mut doc, "active_scene", &name);
            doc.save(store)
        };
        atomic_write(&file, text.as_bytes()).map_err(|e| format!("{}: {e}", file.display()))?;
        self.rescan().map_err(|e| e.to_string())?;
        self.set_active(Some(name.clone()));
        self.publish(store);
        self.find(&name)
            .ok_or_else(|| "stored scene vanished".into())
    }

    fn next_seq(&self) -> u32 {
        self.entries
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .iter()
            .filter_map(|e| e.seq)
            .max()
            .map_or(1, |m| m + 1)
    }

    /// Rename the scene at `idx` (keeps its position).
    pub fn rename(&self, store: &ParamStore, idx: usize, new: &str) -> Result<(), String> {
        let new = validate_name(new)?;
        let e = self
            .entries
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .get(idx)
            .cloned()
            .ok_or_else(|| format!("no scene #{idx}"))?;
        if self
            .names()
            .iter()
            .enumerate()
            .any(|(i, n)| i != idx && n == new)
        {
            return Err(format!("scene {new:?} exists"));
        }
        let seq = e.seq.unwrap_or_else(|| self.next_seq());
        let to = self.dir.join(format!("{seq:03} {new}.snap"));
        fs::rename(&e.file, &to).map_err(|err| format!("{}: {err}", e.file.display()))?;
        self.rescan().map_err(|e| e.to_string())?;
        if self.active().as_deref() == Some(e.name.as_str()) {
            self.set_active(Some(new.to_string()));
            let mut doc = self.doc.lock().unwrap_or_else(|e| e.into_inner());
            set_header(&mut doc, "active_scene", new);
        }
        self.publish(store);
        Ok(())
    }

    /// Remove the scene at `idx` (moved to `.deleted/`, not destroyed).
    pub fn delete(&self, store: &ParamStore, idx: usize) -> Result<String, String> {
        let e = self
            .entries
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .get(idx)
            .cloned()
            .ok_or_else(|| format!("no scene #{idx}"))?;
        let trash = self.dir.join(".deleted");
        fs::create_dir_all(&trash).map_err(|err| err.to_string())?;
        let fname = e.file.file_name().unwrap_or_default().to_string_lossy();
        let stamp = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map_or(0, |d| d.as_secs());
        fs::rename(&e.file, trash.join(format!("{stamp} {fname}")))
            .map_err(|err| format!("{}: {err}", e.file.display()))?;
        self.rescan().map_err(|e| e.to_string())?;
        if self.active().as_deref() == Some(e.name.as_str()) {
            self.set_active(None);
        }
        self.publish(store);
        Ok(e.name)
    }

    /// Execute a `$ctl/lib/$action` command; returns a log line.
    pub fn run_action(
        &self,
        store: &ParamStore,
        action: &str,
        index: i32,
    ) -> Result<String, String> {
        let n = self.names().len();
        let at = |i: i64| -> Result<usize, String> {
            usize::try_from(i)
                .ok()
                .filter(|&i| i < n)
                .ok_or_else(|| format!("no scene #{i} ({n} scenes)"))
        };
        let active = self.active().and_then(|a| self.find(&a)).map(|i| i as i64);
        let recall = |i: usize| -> Result<String, String> {
            let (name, rep) = self.recall(store, i)?;
            Ok(format!(
                "recalled scene #{i} {name:?}: {} applied, {} rejected, {} unmapped",
                rep.applied,
                rep.rejected.len(),
                rep.unmapped.len()
            ))
        };
        let (cmd, arg) = match action.split_once(':') {
            Some((c, a)) => (c.trim(), Some(a)),
            None => (action.trim(), None),
        };
        match cmd.to_ascii_uppercase().as_str() {
            "GO" => recall(at(index as i64)?),
            "GONEXT" => recall(at(active.map_or(0, |a| a + 1))?),
            "GOPREV" => recall(at(active.map_or(0, |a| a - 1))?),
            "NEXT" | "PREV" => {
                let step = if cmd.eq_ignore_ascii_case("NEXT") {
                    1
                } else {
                    -1
                };
                let sel = (index as i64 + step).clamp(0, n.saturating_sub(1) as i64);
                if let Some(id) = store.lookup("$ctl/lib/$actionidx") {
                    let _ = store.set_from(id, Value::Int(sel as i32), ORIGIN_LOCAL);
                }
                Ok(format!("selected scene #{sel}"))
            }
            "STORE" => {
                let i = self.store(store, arg)?;
                Ok(format!("stored scene #{i} {:?}", self.names()[i]))
            }
            "RENAME" => {
                let new = arg.ok_or("RENAME needs a name (RENAME:name)")?;
                let i = at(index as i64)?;
                self.rename(store, i, new)?;
                Ok(format!("renamed scene #{i} to {new:?}"))
            }
            "DELETE" => {
                let name = self.delete(store, at(index as i64)?)?;
                Ok(format!("deleted scene {name:?}"))
            }
            "GOTAG" => Err("GOTAG: scene tags are not supported".into()),
            _ => Err(format!("unknown library action {action:?}")),
        }
    }
}

enum Job {
    Action {
        action: String,
        index: i32,
        origin: Origin,
    },
}

/// [`SceneLib`] behind the native server: `$action` requests are queued to a worker thread that
/// does the file IO and the store load.
pub struct LibraryService {
    lib: Arc<SceneLib>,
    tx: Mutex<Option<Sender<Job>>>,
    thread: Mutex<Option<JoinHandle<()>>>,
}

impl LibraryService {
    pub fn spawn(lib: Arc<SceneLib>, store: Arc<ParamStore>, log: LogFn) -> io::Result<Arc<Self>> {
        let (tx, rx) = mpsc::channel::<Job>();
        let l = lib.clone();
        let thread = std::thread::Builder::new()
            .name("wing-library".into())
            .spawn(move || {
                for job in rx {
                    let Job::Action {
                        action,
                        index,
                        origin,
                    } = job;
                    match l.run_action(&store, &action, index) {
                        Ok(msg) => log(&format!("library ({origin:#x}) {action}: {msg}")),
                        Err(e) => log(&format!("library ({origin:#x}) {action} failed: {e}")),
                    }
                }
            })?;
        Ok(Arc::new(LibraryService {
            lib,
            tx: Mutex::new(Some(tx)),
            thread: Mutex::new(Some(thread)),
        }))
    }

    /// Finish queued jobs and stop the worker.
    pub fn stop(&self) {
        self.tx.lock().unwrap_or_else(|e| e.into_inner()).take();
        if let Some(t) = self.thread.lock().unwrap_or_else(|e| e.into_inner()).take() {
            let _ = t.join();
        }
    }
}

impl SceneLibrary for LibraryService {
    fn scenes(&self) -> Vec<String> {
        self.lib.names()
    }

    fn action(&self, action: &str, index: i32, origin: Origin) {
        if let Some(tx) = &*self.tx.lock().unwrap_or_else(|e| e.into_inner()) {
            let _ = tx.send(Job::Action {
                action: action.into(),
                index,
                origin,
            });
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::persist::test_dir;

    fn lib(dir: &Path) -> (SceneLib, ParamStore) {
        let doc: Doc = Arc::new(Mutex::new(Snapshot::factory()));
        (
            SceneLib::open(dir.join("scenes"), "wing-rack", doc).unwrap(),
            ParamStore::new(),
        )
    }

    #[test]
    fn store_recall_rename_delete() {
        let d = test_dir("lib");
        let (lib, st) = lib(&d);
        let fdr = st.lookup("ch/1/fdr").unwrap();
        st.set(fdr, Value::Float(-5.0)).unwrap();
        assert_eq!(lib.store(&st, Some("Intro")).unwrap(), 0);
        st.set(fdr, Value::Float(-10.0)).unwrap();
        assert_eq!(lib.store(&st, Some("Verse")).unwrap(), 1);
        assert_eq!(lib.names(), ["Intro", "Verse"]);
        assert!(lib.dir().join("001 Intro.snap").exists());
        // STORE without a name updates the active scene
        st.set(fdr, Value::Float(-12.0)).unwrap();
        assert_eq!(lib.store(&st, None).unwrap(), 1);
        assert_eq!(lib.names().len(), 2);
        let s = |p| {
            let id = st.lookup(p).unwrap();
            st.with_str(id, str::to_string).unwrap()
        };
        assert_eq!(s("$ctl/lib/$active"), "Verse");
        assert_eq!(s("$ctl/lib/$actshow"), SHOW_NAME);

        lib.recall(&st, 0).unwrap();
        assert_eq!(st.get(fdr), Some(Value::Float(-5.0)));
        assert_eq!(s("$ctl/lib/$active"), "Intro");
        assert_eq!(s("$syscfg/$cnsmdl"), "wing-rack");
        assert_eq!(lib.run_action(&st, "GONEXT", 0).map(|_| ()), Ok(()));
        assert_eq!(st.get(fdr), Some(Value::Float(-12.0)));
        let aid = st.lookup("$ctl/lib/$activeid").unwrap();
        assert_eq!(st.get(aid), Some(Value::Int(1)));
        assert!(lib.run_action(&st, "GONEXT", 0).is_err());
        lib.run_action(&st, "NEXT", 0).unwrap();
        let sel = st.lookup("$ctl/lib/$actionidx").unwrap();
        assert_eq!(st.get(sel), Some(Value::Int(1)));

        lib.run_action(&st, "RENAME:Chorus", 1).unwrap();
        assert_eq!(lib.names(), ["Intro", "Chorus"]);
        assert_eq!(s("$ctl/lib/$active"), "Chorus");
        assert!(lib.run_action(&st, "RENAME:Intro", 1).is_err());
        assert!(lib.run_action(&st, "RENAME:a/b", 1).is_err());

        lib.run_action(&st, "DELETE", 0).unwrap();
        assert_eq!(lib.names(), ["Chorus"]);
        assert_eq!(fs::read_dir(lib.dir().join(".deleted")).unwrap().count(), 1);
        // the next new scene continues the numbering
        lib.store(&st, Some("Outro")).unwrap();
        assert!(lib.dir().join("003 Outro.snap").exists());
        assert!(lib.run_action(&st, "GOTAG", 0).is_err());
        assert!(lib.run_action(&st, "BOGUS", 0).is_err());
        let _ = fs::remove_dir_all(&d);
    }

    /// T017 acceptance: store and recall through native-protocol requests over TCP loopback
    /// (ephemeral port), with change events reaching the client.
    #[test]
    fn native_store_and_recall_loopback() {
        use std::io::{Read, Write};
        use std::net::{Ipv4Addr, SocketAddrV4, TcpStream};
        use std::time::{Duration, Instant};
        use wing_proto::nrp::{NodeDef, NrpDecoder, NrpEncoder, Token};
        use wing_proto::server::defs::LiveInfo;
        use wing_proto::server::{EventFn, Hub, NativeServer, ServerConfig, ServerShared};

        let d = test_dir("loopback");
        let doc: Doc = Arc::new(Mutex::new(Snapshot::factory()));
        let lib = Arc::new(SceneLib::open(d.join("scenes"), "wing-fullsize", doc).unwrap());
        let mut store = ParamStore::new();
        lib.publish(&store);
        let hub = Hub::attach(&mut store);
        let store = Arc::new(store);
        let svc = LibraryService::spawn(lib.clone(), store.clone(), Arc::new(|_| {})).unwrap();
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

        let mut sock = TcpStream::connect(server.local_addr()).unwrap();
        sock.set_read_timeout(Some(Duration::from_millis(20)))
            .unwrap();
        let mut enc = NrpEncoder::new();
        let h = |p: &str| store.schema().hash(store.lookup(p).unwrap());
        let mut send = |toks: &[Token]| {
            let mut v = Vec::new();
            for t in toks {
                enc.control(t, &mut v).unwrap();
            }
            sock.write_all(&v).unwrap();
        };
        let wait = |what: &str, f: &dyn Fn() -> bool| {
            let t = Instant::now();
            while !f() {
                assert!(t.elapsed() < Duration::from_secs(5), "timeout: {what}");
                std::thread::sleep(Duration::from_millis(5));
            }
        };
        let fdr = store.lookup("ch/1/fdr").unwrap();
        let name = store.lookup("ch/1/name").unwrap();

        // set a state and store it as two scenes
        send(&[
            Token::NavHash(h("ch/1/fdr")),
            Token::Float(-5.0),
            Token::NavHash(h("ch/1/name")),
            Token::Str(b"Kick"),
            Token::NavHash(h("$ctl/lib/$action")),
            Token::Str(b"STORE:Intro"),
        ]);
        wait("store Intro", &|| lib.names() == ["Intro"]);
        send(&[
            Token::NavHash(h("ch/1/fdr")),
            Token::Float(-20.0),
            Token::NavHash(h("$ctl/lib/$action")),
            Token::Str(b"STORE:Verse"),
        ]);
        wait("store Verse", &|| lib.names().len() == 2);
        assert!(d.join("scenes/001 Intro.snap").exists());

        // change the live state, then recall scene 0 with $actionidx + GO
        send(&[
            Token::NavHash(h("ch/1/fdr")),
            Token::Float(0.0),
            Token::NavHash(h("ch/1/name")),
            Token::Str(b"Other"),
        ]);
        wait("live edit", &|| store.get(fdr) == Some(Value::Float(0.0)));
        send(&[
            Token::NavHash(h("$ctl/lib/$actionidx")),
            Token::SmallInt(0),
            Token::NavHash(h("$ctl/lib/$action")),
            Token::Str(b"GO"),
        ]);
        wait("recall Intro", &|| {
            store.get(fdr) == Some(Value::Float(-5.0))
                && store.with_str(name, |s| s == "Kick") == Some(true)
        });
        let active = store.lookup("$ctl/lib/$active").unwrap();
        wait("active", &|| {
            store.with_str(active, |s| s == "Intro") == Some(true)
        });

        // GONEXT by enum index (2) recalls Verse
        send(&[Token::NavHash(h("$ctl/lib/$action")), Token::SmallInt(2)]);
        wait("recall Verse", &|| {
            store.get(fdr) == Some(Value::Float(-20.0))
        });

        // the recall reaches the client as change events; `$scenes` lists the library
        send(&[
            Token::NavHash(h("$ctl/lib/$scenes")),
            Token::End,
            Token::DefRequest,
        ]);
        let mut dec = NrpDecoder::new();
        let mut buf = [0u8; 65536];
        let (mut fdr_events, mut scenes) = (Vec::new(), None);
        let mut cur = 0;
        let t = Instant::now();
        while scenes.is_none() && t.elapsed() < Duration::from_secs(5) {
            match sock.read(&mut buf) {
                Ok(0) => break,
                Ok(n) => dec.feed(&buf[..n]),
                Err(_) => {}
            }
            while let Some(tok) = dec.next_control() {
                match tok.unwrap() {
                    Token::NavHash(x) => cur = x,
                    Token::Float(v) if cur == h("ch/1/fdr") => fdr_events.push(v),
                    Token::NodeDef(b) => {
                        let def = NodeDef::parse(b).unwrap();
                        if let wing_proto::nrp::nodedef::NodeKind::StringEnum(items) = def.kind {
                            scenes = Some(items.into_iter().map(|i| i.name).collect::<Vec<_>>());
                        }
                    }
                    _ => {}
                }
            }
        }
        assert_eq!(scenes.unwrap(), ["Intro", "Verse"]);
        assert!(fdr_events.contains(&-5.0), "{fdr_events:?}");
        assert_eq!(fdr_events.last(), Some(&-20.0));

        drop(sock);
        server.stop();
        svc.stop();
        let _ = fs::remove_dir_all(&d);
    }

    #[test]
    fn listing_order_and_lookup() {
        let d = test_dir("libscan");
        let sd = d.join("scenes");
        fs::create_dir_all(&sd).unwrap();
        for f in [
            "010 B.snap",
            "002 A.snap",
            "Loose.snap",
            ".hidden.snap",
            "x.txt",
            "7.snap",
        ] {
            fs::write(sd.join(f), "{}").unwrap();
        }
        let (lib, _) = lib(&d);
        assert_eq!(lib.names(), ["A", "B", "7", "Loose"]);
        assert_eq!(lib.find("b"), Some(1));
        assert_eq!(lib.find("#3"), Some(3));
        assert_eq!(lib.find("#4"), None);
        assert_eq!(lib.find("nope"), None);
        let _ = fs::remove_dir_all(&d);
    }
}
