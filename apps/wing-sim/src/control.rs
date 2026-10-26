//! Local control channel for front ends (T064: the `WingSimMenu` menu-bar app). Nothing here
//! runs on, or talks to, the audio threads.
//!
//! * **Status file** `<state-dir>/status.json` ([`STATUS_FILE`]): a [`DaemonStatus`] JSON
//!   document, rewritten atomically (temp file + rename) about once a second by the daemon's
//!   main loop, and once more with `"running": false` on exit. Readers check `pid` and
//!   `updated_unix` for liveness.
//! * **Control socket** `<state-dir>/control.sock` ([`SOCKET_FILE`]): Unix stream socket, line
//!   protocol. Each request line gets one reply line, `ok <text>` or `err <text>`:
//!
//! ```text
//! ping                  -> ok wing-sim <version> pid <pid>
//! status                -> ok <DaemonStatus JSON, one line>
//! save <path>           -> ok saved <bytes> bytes to <path>     (current state as a .snap;
//!                                                               becomes the scene file)
//! export <path>         -> ok exported <path> + <stem>.changes.md (console-ready 3.x .snap, validated; scene file unchanged)
//! load <path>           -> ok loaded <path>: <load report>      (replaces the current state)
//! reload-hostio [path]  -> ok <routes>   (re-read the --hostio TOML, or switch to <path>;
//!                                         the audio session restarts with the new bridges)
//! hostio-off            -> ok host outputs off
//! record start <taps> <dir> -> ok recording ... (T061; taps e.g. main/1,usb_out/1-2, dir to
//!                                               end of line)
//! record stop           -> ok recorded <secs> s, <n> file(s) to <dir>
//! record status         -> ok rec ...
//! monitor-level         -> ok mon/1 0.0 dB, mon/2 0.0 dB   (T03F: the PHONES / SPEAKERS knobs)
//! monitor-level <m> <dB> -> ok mon/1 -12.0 dB, mon/2 0.0 dB (m: 1|a|A|phones, 2|b|B|speakers;
//!                                                          dB: -144..10, -inf/-oo/off = -oo)
//! probe <target>        -> ok [{...}]   (T088: live model-slot introspection, one JSON object per slot;
//!                                        target ch/N[/gate|eq|dyn] or aux/N[/eq|dyn]; see wing_sim::probe)
//! quit                  -> ok stopping
//! ```
//!
//! The socket thread only parses lines and forwards [`Request`]s over a channel; the daemon's
//! main loop executes them (it owns the store, the scene document and the audio service) and
//! answers through [`Request::respond`].

use std::io::{self, BufRead, BufReader, Write};
use std::os::unix::net::{UnixListener, UnixStream};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering::Relaxed};
use std::sync::mpsc::{self, Receiver, SyncSender};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::Duration;

use serde::Serialize;

use crate::audio::driver::AudioReport;

/// Status file name in the state dir.
pub const STATUS_FILE: &str = "status.json";
/// Control socket name in the state dir.
pub const SOCKET_FILE: &str = "control.sock";
/// How long a connection may stay silent before the server drops it.
pub const IDLE_TIMEOUT: Duration = Duration::from_secs(5);
/// How long the server waits for the daemon to execute a request.
pub const EXEC_TIMEOUT: Duration = Duration::from_secs(20);

/// The daemon's machine-readable status (`status.json`, `status` command).
#[derive(Debug, Clone, Default, PartialEq, Serialize)]
pub struct DaemonStatus {
    /// `false` in the last document written on shutdown.
    pub running: bool,
    pub pid: u32,
    pub version: String,
    /// Console name (discovery identity).
    pub name: String,
    /// Wall-clock time of this document (seconds since the Unix epoch).
    pub updated_unix: f64,
    pub uptime_s: f64,
    pub state_dir: String,
    /// Connected native-protocol clients (WING-Edit).
    pub native_clients: usize,
    /// Live meter subscriptions.
    pub meter_reports: usize,
    pub discovery_replies: u64,
    /// Scene file last loaded or saved through `--scene` / the control socket.
    pub scene_file: Option<String>,
    /// Active library scene (`$ctl/lib`), if any.
    pub active_scene: Option<String>,
    /// The `--hostio` TOML in use (`None`: default route or off).
    pub hostio_path: Option<String>,
    /// Summary of the host-output routes.
    pub hostio: String,
    /// Audio engine (`None` with `--no-audio`).
    pub audio: Option<AudioReport>,
    /// Whether a recording (T061) is running.
    pub recording: bool,
    /// Recorder state line (`rec off`, `rec 12.3 s, 2 file(s) -> DIR, dropped 0`, ...).
    pub recorder: String,
    /// Monitor level knobs `[A (PHONES, mon/1), B (SPEAKERS, mon/2)]` in dB, -144 = -oo (T03F;
    /// `None` with `--no-audio`). Set with `monitor-level`.
    pub monitor_levels_db: Option<[f32; 2]>,
}

impl DaemonStatus {
    /// One-line JSON.
    pub fn to_json(&self) -> String {
        serde_json::to_string(self).unwrap_or_else(|e| format!("{{\"error\":\"{e}\"}}"))
    }
}

/// Seconds since the Unix epoch.
pub fn unix_now() -> f64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0.0, |d| d.as_secs_f64())
}

/// Write `status` to `path` atomically (temp file in the same directory + rename).
pub fn write_status(path: &Path, status: &DaemonStatus) -> io::Result<()> {
    static SEQ: AtomicU64 = AtomicU64::new(0);
    let tmp = path.with_file_name(format!(
        ".{STATUS_FILE}.tmp-{}-{}",
        std::process::id(),
        SEQ.fetch_add(1, Relaxed)
    ));
    let res = std::fs::write(&tmp, status.to_json()).and_then(|_| std::fs::rename(&tmp, path));
    if res.is_err() {
        let _ = std::fs::remove_file(&tmp);
    }
    res
}

/// A parsed control command.
#[derive(Debug, Clone, PartialEq)]
pub enum Command {
    Ping,
    Status,
    Save(PathBuf),
    /// Console-ready validated export plus a `.changes.md` report (T062); the scene file is not changed.
    Export(PathBuf),
    Load(PathBuf),
    /// Re-read the host-output TOML (`None`: the current `--hostio` file).
    ReloadHostIo(Option<PathBuf>),
    HostIoOff,
    /// Start recording taps into a directory (T061).
    RecordStart {
        taps: String,
        dir: PathBuf,
    },
    RecordStop,
    RecordStatus,
    /// Report (`None`) or set a monitor level knob: (monitor 1|2, dB) (T03F).
    MonitorLevel(Option<(usize, f32)>),
    /// Capture and describe a strip's model slot(s) on the live audio (T088, [`crate::probe`]).
    Probe(String),
    Quit,
}

/// Monitor number of `1`/`a`/`phones` (1) and `2`/`b`/`speakers` (2), case-insensitive.
pub fn parse_monitor(s: &str) -> Option<usize> {
    match s.trim().to_ascii_lowercase().as_str() {
        "1" | "a" | "phones" | "ph" => Some(1),
        "2" | "b" | "speakers" | "spk" => Some(2),
        _ => None,
    }
}

/// A level in dB: a number (optionally suffixed `dB`), or `-inf`/`-oo`/`off` (= -144).
pub fn parse_db(s: &str) -> Option<f32> {
    let t = s.trim().to_ascii_lowercase();
    match t.as_str() {
        "-inf" | "-oo" | "off" | "-144" => Some(-144.0),
        _ => t
            .trim_end_matches("db")
            .trim()
            .parse::<f32>()
            .ok()
            .filter(|v| v.is_finite()),
    }
}

impl Command {
    /// Parse one request line (`verb [argument to end of line]`).
    pub fn parse(line: &str) -> Result<Self, String> {
        let line = line.trim();
        let (verb, arg) = match line.split_once(char::is_whitespace) {
            Some((v, a)) => (v, a.trim()),
            None => (line, ""),
        };
        let path = |what: &str| {
            if arg.is_empty() {
                Err(format!("{what} needs a path"))
            } else {
                Ok(PathBuf::from(arg))
            }
        };
        let no_arg = |c: Command| {
            if arg.is_empty() {
                Ok(c)
            } else {
                Err(format!("{verb} takes no argument"))
            }
        };
        match verb {
            "ping" => no_arg(Command::Ping),
            "status" => no_arg(Command::Status),
            "save" => path("save").map(Command::Save),
            "export" => path("export").map(Command::Export),
            "load" => path("load").map(Command::Load),
            "reload-hostio" => Ok(Command::ReloadHostIo(
                (!arg.is_empty()).then(|| PathBuf::from(arg)),
            )),
            "hostio-off" => no_arg(Command::HostIoOff),
            "record" => {
                let (sub, rest) = match arg.split_once(char::is_whitespace) {
                    Some((s, r)) => (s, r.trim()),
                    None => (arg, ""),
                };
                match (sub, rest) {
                    ("start", r) => match r.split_once(char::is_whitespace) {
                        Some((taps, dir)) if !dir.trim().is_empty() => Ok(Command::RecordStart {
                            taps: taps.into(),
                            dir: PathBuf::from(dir.trim()),
                        }),
                        _ => Err("usage: record start <taps> <dir>".into()),
                    },
                    ("stop", "") => Ok(Command::RecordStop),
                    ("status", "") | ("", "") => Ok(Command::RecordStatus),
                    _ => Err("usage: record start <taps> <dir> | stop | status".into()),
                }
            }
            "monitor-level" => {
                let usage = || "usage: monitor-level [<1|2|a|b> <dB|-inf>]".to_string();
                if arg.is_empty() {
                    return Ok(Command::MonitorLevel(None));
                }
                let (m, v) = arg.split_once(char::is_whitespace).ok_or_else(usage)?;
                match (parse_monitor(m), parse_db(v)) {
                    (Some(m), Some(db)) => Ok(Command::MonitorLevel(Some((m, db)))),
                    _ => Err(usage()),
                }
            }
            "probe" => {
                crate::probe::parse_target(arg)?;
                Ok(Command::Probe(arg.to_string()))
            }
            "quit" => no_arg(Command::Quit),
            "" => Err("empty command".into()),
            v => Err(format!("unknown command {v:?}")),
        }
    }
}

/// A command waiting for the daemon to execute it.
pub struct Request {
    pub cmd: Command,
    reply: SyncSender<Result<String, String>>,
}

impl Request {
    /// Answer the client (`ok <text>` / `err <text>`).
    pub fn respond(self, result: Result<String, String>) {
        let _ = self.reply.send(result);
    }
}

/// The control socket server (one thread; serves connections one at a time).
pub struct ControlServer {
    path: PathBuf,
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

impl ControlServer {
    /// Bind `path` (a stale socket file is replaced; a live one is an `AddrInUse` error) and
    /// start serving. Requests arrive on the returned receiver.
    pub fn spawn(path: &Path) -> io::Result<(Self, Receiver<Request>)> {
        if path.exists() {
            if UnixStream::connect(path).is_ok() {
                return Err(io::Error::new(
                    io::ErrorKind::AddrInUse,
                    format!("{} is served by another process", path.display()),
                ));
            }
            std::fs::remove_file(path)?;
        }
        let listener = UnixListener::bind(path)?;
        listener.set_nonblocking(true)?;
        let (tx, rx) = mpsc::channel();
        let stop = Arc::new(AtomicBool::new(false));
        let st = stop.clone();
        let thread = std::thread::Builder::new()
            .name("wing-control".into())
            .spawn(move || {
                while !st.load(Relaxed) {
                    match listener.accept() {
                        Ok((conn, _)) => {
                            let _ = serve(conn, &tx, &st);
                        }
                        Err(e) if e.kind() == io::ErrorKind::WouldBlock => {
                            std::thread::sleep(Duration::from_millis(50));
                        }
                        Err(_) => std::thread::sleep(Duration::from_millis(200)),
                    }
                }
            })?;
        Ok((
            Self {
                path: path.to_path_buf(),
                stop,
                thread: Some(thread),
            },
            rx,
        ))
    }

    /// Socket path.
    pub fn path(&self) -> &Path {
        &self.path
    }

    /// Stop serving and remove the socket file.
    pub fn stop(mut self) {
        self.shutdown();
    }

    fn shutdown(&mut self) {
        self.stop.store(true, Relaxed);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
            let _ = std::fs::remove_file(&self.path);
        }
    }
}

impl Drop for ControlServer {
    fn drop(&mut self) {
        self.shutdown();
    }
}

fn serve(conn: UnixStream, tx: &mpsc::Sender<Request>, stop: &AtomicBool) -> io::Result<()> {
    conn.set_nonblocking(false)?;
    conn.set_read_timeout(Some(IDLE_TIMEOUT))?;
    let mut out = conn.try_clone()?;
    let mut lines = BufReader::new(conn);
    let mut line = String::new();
    while !stop.load(Relaxed) {
        line.clear();
        if lines.read_line(&mut line)? == 0 {
            break;
        }
        if line.trim().is_empty() {
            continue;
        }
        let result = match Command::parse(&line) {
            Err(e) => Err(e),
            Ok(cmd) => {
                let (reply, rx) = mpsc::sync_channel(1);
                if tx.send(Request { cmd, reply }).is_err() {
                    Err("daemon is shutting down".into())
                } else {
                    rx.recv_timeout(EXEC_TIMEOUT)
                        .unwrap_or_else(|_| Err("daemon did not answer".into()))
                }
            }
        };
        let text = match result {
            Ok(t) => format!("ok {}\n", one_line(&t)),
            Err(e) => format!("err {}\n", one_line(&e)),
        };
        out.write_all(text.as_bytes())?;
    }
    Ok(())
}

fn one_line(s: &str) -> String {
    s.trim().replace(['\n', '\r'], " ")
}

/// Client side: send one command line and return the reply (`Ok` for `ok ...`).
pub fn request(path: &Path, line: &str, timeout: Duration) -> Result<String, String> {
    let mut conn = UnixStream::connect(path).map_err(|e| format!("{}: {e}", path.display()))?;
    conn.set_read_timeout(Some(timeout))
        .map_err(|e| e.to_string())?;
    conn.write_all(format!("{}\n", line.trim()).as_bytes())
        .map_err(|e| e.to_string())?;
    let mut reply = String::new();
    BufReader::new(conn)
        .read_line(&mut reply)
        .map_err(|e| e.to_string())?;
    let reply = reply.trim_end();
    if let Some(r) = reply.strip_prefix("ok") {
        Ok(r.trim_start().to_string())
    } else if let Some(r) = reply.strip_prefix("err") {
        Err(r.trim_start().to_string())
    } else {
        Err(format!("bad reply {reply:?}"))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tmp(tag: &str) -> PathBuf {
        let d = std::env::temp_dir().join(format!("wsctl-{}-{tag}", std::process::id()));
        let _ = std::fs::remove_dir_all(&d);
        std::fs::create_dir_all(&d).unwrap();
        d
    }

    #[test]
    fn parse_commands() {
        assert_eq!(Command::parse("ping\n"), Ok(Command::Ping));
        assert_eq!(Command::parse(" status "), Ok(Command::Status));
        assert_eq!(
            Command::parse("save /tmp/My Scene.snap"),
            Ok(Command::Save("/tmp/My Scene.snap".into()))
        );
        assert_eq!(
            Command::parse("load a.snap"),
            Ok(Command::Load("a.snap".into()))
        );
        assert_eq!(
            Command::parse("reload-hostio"),
            Ok(Command::ReloadHostIo(None))
        );
        assert_eq!(
            Command::parse("reload-hostio /x/h.toml"),
            Ok(Command::ReloadHostIo(Some("/x/h.toml".into())))
        );
        assert_eq!(Command::parse("hostio-off"), Ok(Command::HostIoOff));
        assert_eq!(Command::parse("quit"), Ok(Command::Quit));
        assert_eq!(
            Command::parse("record start main/1,usb_out/1-2 /tmp/My Takes"),
            Ok(Command::RecordStart {
                taps: "main/1,usb_out/1-2".into(),
                dir: PathBuf::from("/tmp/My Takes")
            })
        );
        assert_eq!(Command::parse("record stop"), Ok(Command::RecordStop));
        assert_eq!(Command::parse("record status"), Ok(Command::RecordStatus));
        assert_eq!(Command::parse("record"), Ok(Command::RecordStatus));
        assert_eq!(
            Command::parse("monitor-level"),
            Ok(Command::MonitorLevel(None))
        );
        assert_eq!(
            Command::parse("monitor-level a -12.5"),
            Ok(Command::MonitorLevel(Some((1, -12.5))))
        );
        assert_eq!(
            Command::parse("monitor-level SPEAKERS -inf"),
            Ok(Command::MonitorLevel(Some((2, -144.0))))
        );
        assert_eq!(
            Command::parse("monitor-level 2 +3dB"),
            Ok(Command::MonitorLevel(Some((2, 3.0))))
        );
        assert!(Command::parse("monitor-level 3 0").is_err());
        assert!(Command::parse("monitor-level a").is_err());
        assert!(Command::parse("monitor-level a loud").is_err());
        assert!(Command::parse("record start main/1").is_err());
        assert!(Command::parse("record pause").is_err());
        assert!(Command::parse("save").is_err());
        assert_eq!(
            Command::parse("export /t/x.snap"),
            Ok(Command::Export("/t/x.snap".into()))
        );
        assert_eq!(
            Command::parse("probe ch/21/gate"),
            Ok(Command::Probe("ch/21/gate".into()))
        );
        assert!(Command::parse("probe").is_err());
        assert!(Command::parse("probe bus/1/dyn").is_err());
        assert!(Command::parse("ping now").is_err());
        assert!(Command::parse("").is_err());
        assert!(Command::parse("rm -rf").is_err());
    }

    #[test]
    fn socket_round_trip_and_cleanup() {
        let dir = tmp("sock");
        let path = dir.join(SOCKET_FILE);
        // a stale socket file (nobody listening) is replaced
        drop(UnixListener::bind(&path).unwrap());
        assert!(path.exists());
        let (server, rx) = ControlServer::spawn(&path).unwrap();
        // a second server on the same live socket is refused
        assert_eq!(
            ControlServer::spawn(&path).err().map(|e| e.kind()),
            Some(io::ErrorKind::AddrInUse)
        );
        let daemon = std::thread::spawn(move || {
            let mut seen = Vec::new();
            while let Ok(req) = rx.recv() {
                let r = match &req.cmd {
                    Command::Ping => Ok("pong\nwith newline".into()),
                    Command::Save(p) => Ok(format!("saved {}", p.display())),
                    Command::Quit => {
                        seen.push(req.cmd.clone());
                        req.respond(Ok("stopping".into()));
                        break;
                    }
                    c => Err(format!("unsupported {c:?}")),
                };
                seen.push(req.cmd.clone());
                req.respond(r);
            }
            seen
        });
        let t = Duration::from_secs(5);
        assert_eq!(request(&path, "ping", t), Ok("pong with newline".into()));
        assert_eq!(
            request(&path, "save /a b.snap", t),
            Ok("saved /a b.snap".into())
        );
        assert!(request(&path, "bogus", t).unwrap_err().contains("unknown"));
        assert!(request(&path, "status", t)
            .unwrap_err()
            .contains("unsupported"));
        // several commands on one connection
        let mut c = UnixStream::connect(&path).unwrap();
        c.write_all(b"ping\nquit\n").unwrap();
        let mut r = BufReader::new(c);
        let (mut a, mut b) = (String::new(), String::new());
        r.read_line(&mut a).unwrap();
        r.read_line(&mut b).unwrap();
        drop(r); // closes the connection: the server goes back to accepting
        assert_eq!(
            (a.as_str(), b.as_str()),
            ("ok pong with newline\n", "ok stopping\n")
        );
        let seen = daemon.join().unwrap();
        assert_eq!(seen.last(), Some(&Command::Quit));
        // the daemon side is gone: requests fail cleanly instead of hanging
        assert!(request(&path, "ping", t).is_err());
        server.stop();
        assert!(!path.exists());
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// `fixtures/control/status.json` is the schema WingSimMenu's tests decode: keep in sync.
    #[test]
    fn status_matches_shared_fixture() {
        use crate::audio::hostio::BridgeReport;
        let dir = "/Users/x/Library/Application Support/WingSimulator";
        let st = DaemonStatus {
            running: true,
            pid: 4242,
            version: "0.1.0".into(),
            name: "WINGSIM".into(),
            updated_unix: 1_793_000_000.5,
            uptime_s: 12.25,
            state_dir: dir.into(),
            native_clients: 1,
            meter_reports: 3,
            discovery_replies: 17,
            scene_file: Some("/Users/x/Scenes/Show.snap".into()),
            active_scene: None,
            hostio_path: Some(format!("{dir}/hostio.toml")),
            hostio: "host outputs: mon/1 -> \"Scarlett 2i4 USB\" [1, 2]".into(),
            audio: Some(AudioReport {
                attached: true,
                rate: 48_000,
                driver: "running".into(),
                load_pct: 23.5,
                peak_load_pct: 41.0,
                cycle_p50_us: 310,
                cycle_p99_us: 520,
                xruns: 2,
                underruns: 1,
                overruns: 1,
                slow_cycles: 0,
                bridges: vec![BridgeReport {
                    device: "Scarlett 2i4 USB".into(),
                    state: "Running".into(),
                    fill_ms: 21.0,
                    target_ms: 21.0,
                    e2e_ms: Some(23.25),
                    correction_ppm: 1.5,
                    drift_ppm: -0.5,
                    underruns: 0,
                    resyncs: 1,
                }],
                message: "audio: 48000 Hz, driver running".into(),
            }),
            recording: true,
            recorder: "rec 12.3 s, 2 file(s) -> /Users/x/Takes, dropped 0".into(),
            monitor_levels_db: Some([0.0, -12.5]),
        };
        let fixture: serde_json::Value =
            serde_json::from_str(include_str!("../../../fixtures/control/status.json")).unwrap();
        let ours: serde_json::Value = serde_json::from_str(&st.to_json()).unwrap();
        assert_eq!(ours, fixture);
    }

    #[test]
    fn status_file_is_json() {
        let dir = tmp("status");
        let path = dir.join(STATUS_FILE);
        let st = DaemonStatus {
            running: true,
            pid: 42,
            name: "WINGSIM".into(),
            native_clients: 2,
            audio: Some(AudioReport {
                attached: true,
                rate: 48_000,
                load_pct: 12.5,
                bridges: vec![],
                ..AudioReport::default()
            }),
            ..DaemonStatus::default()
        };
        write_status(&path, &st).unwrap();
        let v: serde_json::Value =
            serde_json::from_str(&std::fs::read_to_string(&path).unwrap()).unwrap();
        assert_eq!(v["pid"], 42);
        assert_eq!(v["native_clients"], 2);
        assert_eq!(v["audio"]["rate"], 48_000);
        assert_eq!(v["audio"]["load_pct"], 12.5);
        assert!(v["scene_file"].is_null());
        // no temp files left behind
        assert_eq!(std::fs::read_dir(&dir).unwrap().count(), 1);
        let _ = std::fs::remove_dir_all(&dir);
    }
}
