//! Wing Simulator daemon: discovery (UDP 2222), native protocol server (TCP 2222), meter
//! streaming (UDP, from port 2222) and OSC (UDP 2223) over one shared `ParamStore`.
//!
//! ```text
//! wing-sim [--name WINGSIM] [--ip A.B.C.D] [--fw 3.3.4-0-g0c2b9d4a:release] [--model ngc-full]
//!          [--serial S] [--scene PATH] [--idle-timeout SECS(0=off)] [--no-meters] [--no-osc]
//!          [--port 2222] [--osc-port 2223] [--save-on-exit PATH]
//!          [--state-dir DIR] [--no-autosave] [--recall NAME] [--list-scenes]
//!          [--no-audio] [--hostio PATH | --no-hostio] [--shm-name NAME] [--block N]
//!          [--workers N] [--no-rt]
//! wing-sim record start <TAPS> <DIR> | stop | status   [--host H] [--osc-port N]
//! ```
//! Ctrl-C (SIGINT) or SIGTERM shuts down cleanly.
//!
//! State (T017, see `persist.rs`/`library.rs`): the store is autosaved to
//! `<state-dir>/state.snap` 2 s after the last change and on exit, and restored on start unless
//! `--scene` or `--recall` is given. `<state-dir>/scenes/` is the scene library served as
//! `$ctl/lib` (WING-Edit's library page); `--list-scenes` prints it, `--recall NAME` (or `#N`)
//! starts from a library scene.
//!
//! Audio (T026, see `wing_sim::audio`): attaches to the Wing Simulator HAL driver's shared memory
//! and runs the console on a real-time thread (USB In 1..48 -> routing -> 40 ch + 8 aux strips ->
//! busses, mains, matrices (T036) -> output patch -> USB Out 1..48, plus host-device bridges). Without a
//! (compatible) driver it runs control-only and keeps retrying. `--hostio PATH` takes a
//! wing-hostio TOML (see `crates/wing-hostio/hostio.example.toml`); by default monitor A (`mon/1`: the monitor source, MAIN.1 by default, with solo priority) plays on
//! "Scarlett 2i4 USB" 1/2 when that device exists.
//!
//! Recording (T061, see `wing_sim::record`): `/wingsim/record/start ,ss <taps> <dir>`,
//! `/wingsim/record/stop`, `/wingsim/record/status` over OSC, or the `wing-sim record ...` client
//! subcommand (sends those to a running daemon, default 127.0.0.1:2223, and prints the reply).
//! Taps: `main/1,bus/3,mon/1,usb_out/1-48` (one 32-bit float WAV per stereo tap / flat lane).
//! The audio status line shows the recording state.

mod library;
mod persist;

use wing_engine::meters::LevelBank;
use wing_sim::audio::{self, AudioOptions, AudioService, ConsoleConfig, HostIoSetting};
use wing_sim::meters::meter_source;
use wing_sim::record::Recorder;

use std::net::{Ipv4Addr, SocketAddrV4, UdpSocket};
use std::path::PathBuf;
use std::process::ExitCode;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::sync::Mutex;
use std::time::{Duration, Instant};

use wing_model::{ParamStore, Snapshot};
use wing_proto::discovery::{Identity, Responder, ResponderConfig, DISCOVERY_PORT};
use wing_proto::osc::{Arg, Message, OscAttachment, OscConfig, OscServer, SimExtension, OSC_PORT};
use wing_proto::server::meters::METER_INTERVAL;
use wing_proto::server::{
    EventFn, Hub, LiveInfo, MeterBank, MeterStreamer, MeterSubscriptions, NativeServer,
    ServerConfig, ServerEvent, ServerShared, NATIVE_PORT,
};

static STOP: AtomicBool = AtomicBool::new(false);

extern "C" fn on_signal(_: libc::c_int) {
    STOP.store(true, Ordering::SeqCst);
}

#[derive(Debug)]
struct Args {
    identity: Identity,
    ip: Option<Ipv4Addr>,
    scene: Option<String>,
    save_on_exit: Option<String>,
    idle: Option<Duration>,
    meters: bool,
    osc: bool,
    port: u16,
    osc_port: u16,
    state_dir: PathBuf,
    autosave: bool,
    recall: Option<String>,
    list_scenes: bool,
    audio: bool,
    hostio: HostIoSetting,
    shm_name: String,
    block: usize,
    workers: Option<usize>,
    rt: bool,
}

const USAGE: &str = "usage: wing-sim [--name NAME] [--ip A.B.C.D] [--fw FIRMWARE] [--model MODEL] \
[--serial SERIAL] [--scene PATH] [--idle-timeout SECS] [--no-meters] [--no-osc] [--port N] \
[--osc-port N] [--save-on-exit PATH] [--state-dir DIR] [--no-autosave] [--recall NAME|#N] \
[--list-scenes] [--no-audio] [--hostio PATH | --no-hostio] [--shm-name NAME] [--block N] \
[--workers N] [--no-rt]";

fn parse_args(mut it: impl Iterator<Item = String>) -> Result<Args, String> {
    let mut a = Args {
        identity: Identity::default(),
        ip: None,
        scene: None,
        save_on_exit: None,
        idle: Some(Duration::from_secs(10)),
        meters: true,
        osc: true,
        port: NATIVE_PORT,
        osc_port: OSC_PORT,
        state_dir: persist::default_state_dir(),
        autosave: true,
        recall: None,
        list_scenes: false,
        audio: true,
        hostio: HostIoSetting::Default,
        shm_name: wing_shm::SHM_NAME.to_string(),
        block: 64,
        workers: None,
        rt: true,
    };
    while let Some(flag) = it.next() {
        let mut val = || it.next().ok_or(format!("{flag} needs a value"));
        match flag.as_str() {
            "--name" => a.identity.name = val()?,
            "--fw" => a.identity.firmware = val()?,
            "--model" => a.identity.model = val()?,
            "--serial" => a.identity.serial = val()?,
            "--ip" => a.ip = Some(val()?.parse().map_err(|e| format!("--ip: {e}"))?),
            "--scene" => a.scene = Some(val()?),
            "--save-on-exit" => a.save_on_exit = Some(val()?),
            "--idle-timeout" => {
                let s: f64 = val()?.parse().map_err(|e| format!("--idle-timeout: {e}"))?;
                a.idle = (s > 0.0).then(|| Duration::from_secs_f64(s));
            }
            "--port" => a.port = val()?.parse().map_err(|e| format!("--port: {e}"))?,
            "--osc-port" => a.osc_port = val()?.parse().map_err(|e| format!("--osc-port: {e}"))?,
            "--no-meters" => a.meters = false,
            "--no-osc" => a.osc = false,
            "--state-dir" => a.state_dir = PathBuf::from(val()?),
            "--no-autosave" => a.autosave = false,
            "--recall" => a.recall = Some(val()?),
            "--list-scenes" => a.list_scenes = true,
            "--no-audio" => a.audio = false,
            "--hostio" => a.hostio = HostIoSetting::from_file(&val()?)?,
            "--no-hostio" => a.hostio = HostIoSetting::Off,
            "--shm-name" => a.shm_name = val()?,
            "--block" => {
                a.block = val()?.parse().map_err(|e| format!("--block: {e}"))?;
                if !(16..=1024).contains(&a.block) {
                    return Err("--block must be 16..1024".into());
                }
            }
            "--workers" => a.workers = Some(val()?.parse().map_err(|e| format!("--workers: {e}"))?),
            "--no-rt" => a.rt = false,
            "-h" | "--help" => return Err(USAGE.into()),
            other => return Err(format!("unknown argument {other}\n{USAGE}")),
        }
    }
    a.identity.validate()?;
    if a.scene.is_some() && a.recall.is_some() {
        return Err("--scene and --recall are mutually exclusive".into());
    }
    Ok(a)
}

/// `wing-sim record start <TAPS> <DIR> | stop | status [--host H] [--osc-port N]`: send the
/// command to a running daemon over OSC and print its reply.
fn record_cli(mut it: impl Iterator<Item = String>) -> Result<String, String> {
    const RUSAGE: &str =
        "usage: wing-sim record start <TAPS> <DIR> | stop | status [--host H] [--osc-port N]";
    let mut pos = Vec::new();
    let mut host = "127.0.0.1".to_string();
    let mut port = OSC_PORT;
    while let Some(a) = it.next() {
        match a.as_str() {
            "--host" => host = it.next().ok_or(RUSAGE)?,
            "--osc-port" => {
                port = it
                    .next()
                    .ok_or(RUSAGE)?
                    .parse()
                    .map_err(|e| format!("--osc-port: {e}"))?
            }
            "-h" | "--help" => return Err(RUSAGE.into()),
            _ => pos.push(a),
        }
    }
    let msg = record_message(&pos).ok_or(RUSAGE)?;
    let sock = UdpSocket::bind("0.0.0.0:0").map_err(|e| e.to_string())?;
    // stop waits for the writer to finalize the files
    sock.set_read_timeout(Some(Duration::from_secs(5)))
        .map_err(|e| e.to_string())?;
    sock.send_to(&msg.to_vec(), (host.as_str(), port))
        .map_err(|e| format!("send to {host}:{port}: {e}"))?;
    let mut buf = [0u8; 4096];
    let (n, _) = sock
        .recv_from(&mut buf)
        .map_err(|e| format!("no reply from {host}:{port} ({e}); is wing-sim running?"))?;
    let reply = Message::decode(&buf[..n]).map_err(|e| format!("bad reply: {e:?}"))?;
    match reply.args.first() {
        Some(Arg::Str(s)) if s.starts_with("ERROR") => Err(s.clone()),
        Some(Arg::Str(s)) => Ok(s.clone()),
        _ => Err(format!("unexpected reply {reply:?}")),
    }
}

/// The OSC message for `record` subcommand words.
fn record_message(pos: &[String]) -> Option<Message> {
    let p: Vec<&str> = pos.iter().map(String::as_str).collect();
    Some(match p.as_slice() {
        ["start", taps, dir] => {
            // the daemon may run elsewhere (cwd): send an absolute path
            let dir = std::path::absolute(dir).unwrap_or_else(|_| PathBuf::from(dir));
            Message::new(
                "/wingsim/record/start",
                vec![
                    Arg::Str(taps.to_string()),
                    Arg::Str(dir.display().to_string()),
                ],
            )
        }
        ["stop"] => Message::new("/wingsim/record/stop", vec![]),
        ["status"] | [] => Message::new("/wingsim/record/status", vec![]),
        _ => return None,
    })
}

/// The recorder as an OSC extension, logging what it does.
struct LoggedRecorder {
    rec: Arc<Recorder>,
    log: persist::LogFn,
}

impl SimExtension for LoggedRecorder {
    fn command(&self, addr: &str, args: &[Arg]) -> Option<Option<(String, Vec<Arg>)>> {
        let r = self.rec.command(addr, args)?;
        if !addr.ends_with("status") && addr != "/wingsim/record" {
            if let Some((_, a)) = &r {
                if let Some(Arg::Str(t)) = a.first() {
                    (self.log)(&format!("record: {t}"));
                }
            }
        }
        Some(r)
    }
}

fn main() -> ExitCode {
    if std::env::args().nth(1).as_deref() == Some("record") {
        return match record_cli(std::env::args().skip(2)) {
            Ok(s) => {
                println!("{s}");
                ExitCode::SUCCESS
            }
            Err(e) => {
                eprintln!("wing-sim record: {e}");
                ExitCode::FAILURE
            }
        };
    }
    let args = match parse_args(std::env::args().skip(1)) {
        Ok(a) => a,
        Err(e) => {
            eprintln!("{e}");
            return ExitCode::from(2);
        }
    };
    match run(args) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("wing-sim: {e}");
            ExitCode::FAILURE
        }
    }
}

fn run(args: Args) -> Result<(), String> {
    let t0 = Instant::now();
    let log = move |msg: &str| eprintln!("[{:9.3}s] {msg}", t0.elapsed().as_secs_f64());
    log(&format!(
        "wing-sim {} name={} model={} fw={}",
        env!("CARGO_PKG_VERSION"),
        args.identity.name,
        args.identity.model,
        args.identity.firmware
    ));

    // ---- state ----
    let console_model = LiveInfo::console_model_for(&args.identity.model);
    let mut store = ParamStore::new();
    let doc: persist::Doc = Arc::new(Mutex::new(Snapshot::factory()));
    let state_file = args.state_dir.join(persist::STATE_FILE);
    let lib = Arc::new(
        library::SceneLib::open(
            args.state_dir.join(persist::SCENES_DIR),
            console_model,
            doc.clone(),
        )
        .map_err(|e| format!("state dir {}: {e}", args.state_dir.display()))?,
    );
    if args.list_scenes {
        println!("# show {} ({})", library::SHOW_NAME, lib.dir().display());
        for (i, (n, f)) in lib.names().iter().zip(lib.files()).enumerate() {
            println!("#{i}\t{n}\t{}", f.display());
        }
        return Ok(());
    }
    let report = |what: &str, rep: &wing_model::LoadReport| {
        log(&format!(
            "{what}: {} (fw {}{}), {} keys, {} applied, {} migrated ({} lossy), {} unmapped, \
             {} rejected, {} adjusted",
            rep.format,
            rep.creator_fw,
            if rep.legacy {
                ", migrated from 1.x/2.x"
            } else {
                ""
            },
            rep.keys,
            rep.applied,
            rep.migrated.len(),
            rep.lossy().count(),
            rep.unmapped.len(),
            rep.rejected.len(),
            rep.adjusted.len()
        ))
    };
    if let Some(p) = &args.scene {
        let text = std::fs::read_to_string(p).map_err(|e| format!("--scene {p}: {e}"))?;
        let (snap, rep) = Snapshot::load(&store, &text).map_err(|e| format!("--scene {p}: {e}"))?;
        report(&format!("scene {p}"), &rep);
        *doc.lock().unwrap_or_else(|e| e.into_inner()) = snap;
    } else if let Some(name) = &args.recall {
        let i = lib.find(name).ok_or_else(|| {
            format!(
                "--recall {name}: no such scene in {} (have: {})",
                lib.dir().display(),
                lib.names().join(", ")
            )
        })?;
        let (n, rep) = lib
            .recall(&store, i)
            .map_err(|e| format!("--recall {name}: {e}"))?;
        report(&format!("recalled library scene #{i} {n:?}"), &rep);
    } else if state_file.exists() {
        let loaded = std::fs::read_to_string(&state_file)
            .map_err(|e| e.to_string())
            .and_then(|t| Snapshot::load(&store, &t).map_err(|e| e.to_string()));
        match loaded {
            Ok((snap, rep)) => {
                report(&format!("restored {}", state_file.display()), &rep);
                *doc.lock().unwrap_or_else(|e| e.into_inner()) = snap;
            }
            Err(e) => {
                // keep the unreadable file for inspection and start from the factory scene
                let bad = state_file.with_extension("snap.bad");
                let _ = std::fs::rename(&state_file, &bad);
                store.reset_to_defaults();
                log(&format!(
                    "restoring {} failed ({e}); moved to {}, starting from factory defaults",
                    state_file.display(),
                    bad.display()
                ));
            }
        }
    }
    lib.adopt_doc_active();
    lib.publish(&store); // also sets `$syscfg/$cnsmdl`
    log(&format!(
        "state dir {}: autosave {}, library {} scenes{}",
        args.state_dir.display(),
        if args.autosave { "on" } else { "off" },
        lib.names().len(),
        lib.active()
            .map_or(String::new(), |a| format!(", active {a:?}"))
    ));
    // change hooks: native fan-out, OSC subscription events and the autosave counter coexist
    let hub = Hub::attach(&mut store);
    let osc_att = args.osc.then(|| OscAttachment::attach(&mut store));
    let changes = persist::ChangeCounter::attach(&mut store);
    let routing = args.audio.then(|| audio::prepare(&mut store));
    let store = Arc::new(store);
    let logf: persist::LogFn = Arc::new(log);
    let autosaver = if args.autosave {
        Some(
            persist::Autosaver::spawn(
                store.clone(),
                doc.clone(),
                changes,
                state_file.clone(),
                persist::AUTOSAVE_DEBOUNCE,
                logf.clone(),
            )
            .map_err(|e| format!("autosave: {e}"))?,
        )
    } else {
        None
    };
    let library = library::LibraryService::spawn(lib.clone(), store.clone(), logf.clone())
        .map_err(|e| format!("library: {e}"))?;

    // ---- discovery (and the UDP 2222 socket meters are sent from) ----
    let responder = Responder::bind(ResponderConfig {
        bind: SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, DISCOVERY_PORT),
        identity: args.identity.clone(),
        advertise_ip: args.ip,
    })
    .map_err(|e| format!("discovery bind :{DISCOVERY_PORT}: {e}"))?;
    let meter_sock = responder
        .try_clone_socket()
        .or_else(|_| UdpSocket::bind("0.0.0.0:0"))
        .map_err(|e| format!("meter socket: {e}"))?;
    let probes_logged = Arc::new(AtomicBool::new(false));
    let pl = probes_logged.clone();
    let discovery = responder
        .spawn(move |from, reply| {
            if !pl.swap(true, Ordering::Relaxed) {
                log(&format!(
                    "discovery: first probe from {from}, replied {reply:?}"
                ));
            }
        })
        .map_err(|e| format!("discovery: {e}"))?;

    // ---- meters ----
    let subs = Arc::new(MeterSubscriptions::new());
    let bank = Arc::new(MeterBank::for_schema());
    // Real meters (T039) from the audio engine while a session is attached, synthetic otherwise.
    let levels = (args.meters && routing.is_some()).then(LevelBank::shared);
    let streamer = if args.meters {
        Some(
            MeterStreamer::spawn_with_source(
                subs.clone(),
                bank.clone(),
                meter_sock,
                METER_INTERVAL,
                meter_source(levels.clone(), Some(store.clone())),
            )
            .map_err(|e| e.to_string())?,
        )
    } else {
        None
    };

    // ---- native server ----
    let schema = store.schema();
    let ev_store = store.clone();
    let on_event: EventFn = Arc::new(move |e: &ServerEvent| {
        match e {
        ServerEvent::Sets {
            client,
            count,
            changed,
            total,
            last,
        } => {
            let mut s = format!(
                "native client {client}: {count} sets ({changed} changed, total {total})"
            );
            if let Some(id) = last {
                let mut v = String::new();
                let _ = ev_store.format(*id, &mut v);
                s += &format!(", last {} = {v}", schema.path(*id));
            }
            log(&s);
        }
        ServerEvent::Disconnected {
            client,
            peer,
            reason,
            stats,
        } => log(&format!(
            "native client {client} {peer} disconnected: {reason}; sets {} (changed {}, rejected {}, \
             hash0 {}), gets {}, defs {}, model requests {}, bytes in {}",
            stats.sets,
            stats.changed,
            stats.rejected,
            stats.hash0,
            stats.gets,
            stats.defs,
            stats.model_requests,
            stats.bytes_in
        )),
        other => log(&format!("native: {other:?}")),
    }
    });
    let server = NativeServer::bind(
        ServerConfig {
            bind: SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, args.port),
            idle_timeout: args.idle,
            live: LiveInfo {
                console_model: console_model.into(),
                library: Some(library.clone()),
                ..LiveInfo::default()
            },
            ..ServerConfig::default()
        },
        ServerShared {
            store: store.clone(),
            hub,
            meters: args.meters.then(|| subs.clone()),
        },
        on_event,
    )
    .map_err(|e| format!("native server bind tcp :{}: {e}", args.port))?
    .spawn()
    .map_err(|e| e.to_string())?;

    // ---- OSC ----
    let recorder = Recorder::new();
    let osc = match osc_att {
        Some(att) => Some(
            OscServer::bind(
                store.clone(),
                att,
                OscConfig {
                    bind: SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, args.osc_port),
                    identity: args.identity.clone(),
                    advertise_ip: args.ip,
                    library: Some(library.clone()),
                    extension: Some(Arc::new(LoggedRecorder {
                        rec: recorder.clone(),
                        log: logf.clone(),
                    })),
                    ..OscConfig::default()
                },
            )
            .map_err(|e| format!("osc bind udp :{}: {e}", args.osc_port))?
            .spawn()
            .map_err(|e| e.to_string())?,
        ),
        None => None,
    };

    log(&format!(
        "listening: discovery udp {}, native tcp {}, osc {}, meters {}, idle timeout {:?}",
        discovery.local_addr(),
        server.local_addr(),
        osc.as_ref()
            .map_or("off".to_string(), |o| o.local_addr().to_string()),
        match (args.meters, levels.is_some()) {
            (false, _) => "off",
            (true, true) => "on (real while audio is attached, else synthetic)",
            (true, false) => "on (synthetic)",
        },
        args.idle
    ));

    // ---- audio ----
    let audio = match routing {
        Some(routing) => {
            let (hostio, note) = args.hostio.resolve();
            log(&format!("audio: {note}"));
            let mut console = ConsoleConfig {
                block: args.block,
                rt_workers: args.rt,
                meters: levels.clone(),
                ..ConsoleConfig::default()
            };
            if let Some(w) = args.workers {
                console.workers = w;
            }
            Some(
                AudioService::spawn_with_recorder(
                    store.clone(),
                    routing,
                    AudioOptions {
                        shm_name: args.shm_name.clone(),
                        console,
                        rt: args.rt,
                        hostio,
                        ..AudioOptions::default()
                    },
                    logf.clone(),
                    recorder.clone(),
                )
                .map_err(|e| format!("audio: {e}"))?,
            )
        }
        None => {
            log("audio: off (--no-audio)");
            None
        }
    };

    unsafe {
        libc::signal(
            libc::SIGINT,
            on_signal as extern "C" fn(libc::c_int) as libc::sighandler_t,
        );
        libc::signal(
            libc::SIGTERM,
            on_signal as extern "C" fn(libc::c_int) as libc::sighandler_t,
        );
    }
    let mut last_status = Instant::now();
    while !STOP.load(Ordering::SeqCst) {
        std::thread::sleep(Duration::from_millis(100));
        if last_status.elapsed() >= Duration::from_secs(60) {
            last_status = Instant::now();
            log(&format!(
                "status: {} native clients, {} live meter reports, {} meter packets, {} discovery replies",
                server.clients(),
                subs.live_reports(Instant::now()),
                streamer.as_ref().map_or(0, |s| s.packets()),
                discovery.replies()
            ));
            if let Some(a) = &audio {
                log(&a.status_line());
            }
            if recorder.is_recording() {
                log(&recorder.status());
            }
        }
    }
    log("shutting down");
    if recorder.is_recording() {
        log(&format!("record: {}", recorder.stop()));
    }
    if let Some(a) = audio {
        a.stop(); // detaches: the driver falls back to loopback
    }
    server.stop();
    if let Some(o) = osc {
        o.stop();
    }
    if let Some(s) = streamer {
        s.stop();
    }
    discovery.stop();
    library.stop();
    if let Some(a) = autosaver {
        let n = a.stop();
        log(&format!("autosave: {n} saves to {}", state_file.display()));
    }
    if let Some(p) = &args.save_on_exit {
        match persist::save_state(&store, &doc, std::path::Path::new(p)) {
            Ok(_) => log(&format!("saved scene to {p}")),
            Err(e) => log(&format!("saving scene to {p} failed: {e}")),
        }
    }
    log("stopped");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn args(v: &[&str]) -> Result<Args, String> {
        parse_args(v.iter().map(|s| s.to_string()))
    }

    #[test]
    fn cli_defaults_and_flags() {
        let a = args(&[]).unwrap();
        assert_eq!(a.identity.firmware, "3.3.4-0-g0c2b9d4a:release");
        assert_eq!(a.idle, Some(Duration::from_secs(10)));
        let a = args(&[
            "--name",
            "STUDIO",
            "--ip",
            "10.0.0.5",
            "--fw",
            "3.1.2-0-gabc:release",
            "--idle-timeout",
            "0",
            "--no-osc",
        ])
        .unwrap();
        assert_eq!(a.identity.name, "STUDIO");
        assert_eq!(a.ip, Some(Ipv4Addr::new(10, 0, 0, 5)));
        assert_eq!(a.idle, None);
        assert!(!a.osc);
        assert!(a.autosave && a.recall.is_none() && !a.list_scenes);
        assert!(a
            .state_dir
            .ends_with("Library/Application Support/WingSimulator"));
        let a = args(&[
            "--state-dir",
            "/tmp/ws",
            "--no-autosave",
            "--recall",
            "Verse",
            "--list-scenes",
        ])
        .unwrap();
        assert_eq!(a.state_dir, PathBuf::from("/tmp/ws"));
        assert!(!a.autosave && a.list_scenes);
        assert_eq!(a.recall.as_deref(), Some("Verse"));
        assert!(args(&["--scene", "x.snap", "--recall", "y"]).is_err());
        assert!(args(&["--bogus"]).is_err());
        let a = args(&[
            "--no-audio",
            "--no-hostio",
            "--block",
            "128",
            "--workers",
            "2",
        ])
        .unwrap();
        assert!(!a.audio && a.hostio == HostIoSetting::Off && a.block == 128);
        assert_eq!(a.workers, Some(2));
        assert!(args(&["--block", "7"]).is_err());
        assert!(args(&["--hostio", "/nonexistent.toml"]).is_err());
        assert!(args(&["--name", "a,b"]).is_err());
    }

    #[test]
    fn record_subcommand_messages() {
        let w = |v: &[&str]| v.iter().map(|s| s.to_string()).collect::<Vec<_>>();
        let m = record_message(&w(&["start", "main/1,usb_out/1-2", "/tmp/rec"])).unwrap();
        assert_eq!(m.addr, "/wingsim/record/start");
        assert_eq!(
            m.args,
            vec![
                Arg::Str("main/1,usb_out/1-2".into()),
                Arg::Str("/tmp/rec".into())
            ]
        );
        assert_eq!(
            record_message(&w(&["stop"])).unwrap().addr,
            "/wingsim/record/stop"
        );
        assert_eq!(
            record_message(&w(&["status"])).unwrap().addr,
            "/wingsim/record/status"
        );
        assert!(record_message(&w(&["start", "main/1"])).is_none());
        assert!(record_message(&w(&["bogus"])).is_none());
    }
}
