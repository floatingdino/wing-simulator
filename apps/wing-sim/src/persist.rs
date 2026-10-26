//! State directory, atomic file writes and the debounced autosaver (T017).
//!
//! Layout of the state directory (default `~/Library/Application Support/WingSimulator` on macOS,
//! see [`default_state_dir`]):
//!
//! ```text
//! state.snap          current state, autosaved (restored on start unless --scene/--recall)
//! scenes/NNN Name.snap  scene library of the one simulated show (see library.rs)
//! scenes/.deleted/    scenes removed through the library (recoverable)
//! ```
//!
//! The autosaver never touches the control or RT path: its store hook only bumps an atomic
//! change counter. A background thread polls the counter and, once it has been stable for the
//! debounce time, serialises the store and writes `state.snap` atomically (temp file + fsync +
//! rename), so a crash mid-write leaves the previous state intact.

use std::fs::{self, File};
use std::io::{self, Write};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering::*};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use wing_model::{ParamStore, Snapshot};

/// Autosave file name inside the state directory.
pub const STATE_FILE: &str = "state.snap";
/// Scene library directory inside the state directory.
pub const SCENES_DIR: &str = "scenes";
/// Default quiet time before an autosave.
pub const AUTOSAVE_DEBOUNCE: Duration = Duration::from_secs(2);

/// Log sink shared by the daemon's background threads.
pub type LogFn = Arc<dyn Fn(&str) + Send + Sync>;

/// The current scene document (header, unknown keys) that goes with the store. Shared by the
/// autosaver, the scene library and `--save-on-exit`; a recall replaces it.
pub type Doc = Arc<Mutex<Snapshot>>;

/// macOS `~/Library/Application Support/WingSimulator`, Windows `%APPDATA%\\WingSimulator`, other
/// systems `$XDG_DATA_HOME/wing-simulator` (default `~/.local/share/wing-simulator`); or
/// `./WingSimulator` when the base folder is unknown.
pub fn default_state_dir() -> PathBuf {
    let env = |k: &str| {
        std::env::var_os(k)
            .filter(|v| !v.is_empty())
            .map(PathBuf::from)
    };
    let dir = if cfg!(target_os = "macos") {
        env("HOME").map(|h| h.join("Library/Application Support/WingSimulator"))
    } else if cfg!(windows) {
        env("APPDATA").map(|a| a.join("WingSimulator"))
    } else {
        env("XDG_DATA_HOME")
            .or_else(|| env("HOME").map(|h| h.join(".local/share")))
            .map(|d| d.join("wing-simulator"))
    };
    dir.unwrap_or_else(|| PathBuf::from("WingSimulator"))
}

/// Write `data` to `path` atomically: a temp file in the same directory is written, fsynced and
/// renamed over `path`. On error the temp file is removed and `path` is untouched.
pub fn atomic_write(path: &Path, data: &[u8]) -> io::Result<()> {
    static SEQ: AtomicU64 = AtomicU64::new(0);
    let dir = match path.parent() {
        Some(d) if !d.as_os_str().is_empty() => d,
        _ => Path::new("."),
    };
    let name = path
        .file_name()
        .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidInput, "no file name"))?;
    let tmp = dir.join(format!(
        ".{}.tmp-{}-{}",
        name.to_string_lossy(),
        std::process::id(),
        SEQ.fetch_add(1, Relaxed)
    ));
    let res = (|| {
        let mut f = File::create(&tmp)?;
        f.write_all(data)?;
        f.sync_all()?;
        drop(f);
        fs::rename(&tmp, path)?;
        // make the rename durable (best effort)
        if let Ok(d) = File::open(dir) {
            let _ = d.sync_all();
        }
        Ok(())
    })();
    if res.is_err() {
        let _ = fs::remove_file(&tmp);
    }
    res
}

/// Serialise the store with its document and write it atomically.
pub fn save_state(store: &ParamStore, doc: &Doc, path: &Path) -> io::Result<usize> {
    let text = doc.lock().unwrap_or_else(|e| e.into_inner()).save(store);
    atomic_write(path, text.as_bytes())?;
    Ok(text.len())
}

/// Change counter bumped by a store hook (lock-free, allocation-free).
#[derive(Clone, Default)]
pub struct ChangeCounter(Arc<AtomicU64>);

impl ChangeCounter {
    /// Install the counting hook on `store` (before it is shared).
    pub fn attach(store: &mut ParamStore) -> ChangeCounter {
        let c = ChangeCounter::default();
        let g = c.0.clone();
        store.add_hook(Box::new(move |_| {
            g.fetch_add(1, Relaxed);
        }));
        c
    }

    pub fn get(&self) -> u64 {
        self.0.load(Relaxed)
    }
}

/// Debounced background autosave of the store to one file.
pub struct Autosaver {
    stop: Arc<AtomicBool>,
    saves: Arc<AtomicU64>,
    thread: Option<JoinHandle<()>>,
}

impl Autosaver {
    /// Start the autosave thread: `path` is written `debounce` after the last change, and once
    /// more on [`Autosaver::stop`] if anything changed since.
    pub fn spawn(
        store: Arc<ParamStore>,
        doc: Doc,
        changes: ChangeCounter,
        path: PathBuf,
        debounce: Duration,
        log: LogFn,
    ) -> io::Result<Autosaver> {
        let stop = Arc::new(AtomicBool::new(false));
        let saves = Arc::new(AtomicU64::new(0));
        let (stop2, saves2) = (stop.clone(), saves.clone());
        let poll = (debounce / 4).clamp(Duration::from_millis(5), Duration::from_millis(100));
        let thread = std::thread::Builder::new()
            .name("wing-autosave".into())
            .spawn(move || {
                let mut saved = changes.get();
                let mut seen = saved;
                let mut seen_at = Instant::now();
                let save = |g: u64, saved: &mut u64| match save_state(&store, &doc, &path) {
                    Ok(_) => {
                        *saved = g;
                        saves2.fetch_add(1, Relaxed);
                    }
                    Err(e) => {
                        // retry after the next change/debounce; keep the old file
                        *saved = g;
                        log(&format!("autosave {} failed: {e}", path.display()));
                    }
                };
                while !stop2.load(Relaxed) {
                    std::thread::sleep(poll);
                    let g = changes.get();
                    if g != seen {
                        seen = g;
                        seen_at = Instant::now();
                    } else if g != saved && seen_at.elapsed() >= debounce {
                        save(g, &mut saved);
                    }
                }
                let g = changes.get();
                if g != saved {
                    save(g, &mut saved);
                }
            })?;
        Ok(Autosaver {
            stop,
            saves,
            thread: Some(thread),
        })
    }

    /// Completed saves.
    pub fn saves(&self) -> u64 {
        self.saves.load(Relaxed)
    }

    /// Stop the thread, saving pending changes first.
    pub fn stop(mut self) -> u64 {
        self.shutdown();
        self.saves()
    }

    fn shutdown(&mut self) {
        self.stop.store(true, Relaxed);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

impl Drop for Autosaver {
    fn drop(&mut self) {
        self.shutdown();
    }
}

/// A fresh, empty directory under the system temp dir (tests).
#[cfg(test)]
pub fn test_dir(tag: &str) -> PathBuf {
    static N: AtomicU64 = AtomicU64::new(0);
    let d = std::env::temp_dir().join(format!(
        "wing-sim-test-{tag}-{}-{}",
        std::process::id(),
        N.fetch_add(1, Relaxed)
    ));
    let _ = fs::remove_dir_all(&d);
    fs::create_dir_all(&d).unwrap();
    d
}

#[cfg(test)]
mod tests {
    use super::*;
    use wing_model::Value;

    fn noop_log() -> LogFn {
        Arc::new(|_| {})
    }

    fn wait_for(mut f: impl FnMut() -> bool) -> bool {
        let t = Instant::now();
        while t.elapsed() < Duration::from_secs(5) {
            if f() {
                return true;
            }
            std::thread::sleep(Duration::from_millis(5));
        }
        false
    }

    #[test]
    fn atomic_write_replaces_and_leaves_no_temp() {
        let d = test_dir("atomic");
        let p = d.join("x.snap");
        atomic_write(&p, b"one").unwrap();
        atomic_write(&p, b"two").unwrap();
        assert_eq!(fs::read(&p).unwrap(), b"two");
        let names: Vec<_> = fs::read_dir(&d)
            .unwrap()
            .map(|e| e.unwrap().file_name())
            .collect();
        assert_eq!(names, ["x.snap"]);
        // a failing write (missing directory) leaves the target untouched and no temp files
        assert!(atomic_write(&d.join("missing/x.snap"), b"three").is_err());
        // a failing rename (target is a non-empty directory) keeps the old content
        fs::create_dir_all(d.join("dir.snap/sub")).unwrap();
        assert!(atomic_write(&d.join("dir.snap"), b"x").is_err());
        assert_eq!(fs::read(&p).unwrap(), b"two");
        assert_eq!(fs::read_dir(&d).unwrap().count(), 2);
        let _ = fs::remove_dir_all(&d);
    }

    #[test]
    fn autosave_debounces_and_restores() {
        let d = test_dir("autosave");
        let path = d.join(STATE_FILE);
        let mut store = ParamStore::new();
        let changes = ChangeCounter::attach(&mut store);
        let store = Arc::new(store);
        let doc: Doc = Arc::new(Mutex::new(Snapshot::factory()));
        let saver = Autosaver::spawn(
            store.clone(),
            doc.clone(),
            changes,
            path.clone(),
            Duration::from_millis(80),
            noop_log(),
        )
        .unwrap();
        // nothing changed: nothing written
        std::thread::sleep(Duration::from_millis(150));
        assert_eq!(saver.saves(), 0);
        assert!(!path.exists());
        // a burst of changes is saved once, after the quiet time
        let fdr = store.lookup("ch/3/fdr").unwrap();
        let name = store.lookup("ch/3/name").unwrap();
        for i in 0..20 {
            store.set(fdr, Value::Float(-(i as f32))).unwrap();
            std::thread::sleep(Duration::from_millis(2));
        }
        store.set_str(name, "Snare", 0).unwrap();
        assert!(wait_for(|| saver.saves() == 1));
        std::thread::sleep(Duration::from_millis(200));
        assert_eq!(saver.saves(), 1);
        // a change right before stop is saved on stop
        store.set(fdr, Value::Float(-7.0)).unwrap();
        assert_eq!(saver.stop(), 2);

        // restart: restore into a fresh store
        let fresh = ParamStore::new();
        let (_, rep) = Snapshot::load(&fresh, &fs::read_to_string(&path).unwrap()).unwrap();
        assert!(rep.rejected.is_empty());
        assert_eq!(fresh.get(fdr), Some(Value::Float(-7.0)));
        assert_eq!(
            fresh.with_str(name, str::to_string).as_deref(),
            Some("Snare")
        );
        let _ = fs::remove_dir_all(&d);
    }
}
