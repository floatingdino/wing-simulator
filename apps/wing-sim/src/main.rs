//! Wing Simulator daemon: discovery (UDP 2222), native protocol server (TCP 2222), meter
//! streaming (UDP, from port 2222) and OSC (UDP 2223) over one shared `ParamStore`.
//!
//! ```text
//! wing-sim [--name WINGSIM] [--ip A.B.C.D] [--fw 3.3.4-0-g0c2b9d4a:release] [--model ngc-full]
//!          [--serial S] [--scene PATH] [--idle-timeout SECS(0=off)] [--no-meters] [--no-osc]
//!          [--port 2222] [--osc-port 2223] [--save-on-exit PATH]
//!          [--state-dir DIR] [--no-autosave] [--recall NAME] [--list-scenes]
//! ```
//! Ctrl-C (SIGINT) or SIGTERM shuts down cleanly.
//!
//! State (T017, see `persist.rs`/`library.rs`): the store is autosaved to
//! `<state-dir>/state.snap` 2 s after the last change and on exit, and restored on start unless
//! `--scene` or `--recall` is given. `<state-dir>/scenes/` is the scene library served as
//! `$ctl/lib` (WING-Edit's library page); `--list-scenes` prints it, `--recall NAME` (or `#N`)
//! starts from a library scene.

mod library;
mod persist;

use std::net::{Ipv4Addr, SocketAddrV4, UdpSocket};
use std::path::PathBuf;
use std::process::ExitCode;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::sync::Mutex;
use std::time::{Duration, Instant};

use wing_model::{ParamStore, Snapshot};
use wing_proto::discovery::{Identity, Responder, ResponderConfig, DISCOVERY_PORT};
use wing_proto::osc::{OscAttachment, OscConfig, OscServer, OSC_PORT};
use wing_proto::server::meters::METER_INTERVAL;
use wing_proto::server::{
    EventFn, Hub, LiveInfo, MeterBank, MeterStreamer, MeterSubscriptions, NativeServer,
    ServerConfig, ServerEvent, ServerShared, SyntheticMeters, NATIVE_PORT,
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
}

const USAGE: &str = "usage: wing-sim [--name NAME] [--ip A.B.C.D] [--fw FIRMWARE] [--model MODEL] \
[--serial SERIAL] [--scene PATH] [--idle-timeout SECS] [--no-meters] [--no-osc] [--port N] \
[--osc-port N] [--save-on-exit PATH] [--state-dir DIR] [--no-autosave] [--recall NAME|#N] \
[--list-scenes]";

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

fn main() -> ExitCode {
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
    let (synth, streamer) = if args.meters {
        (
            Some(
                SyntheticMeters::spawn(bank.clone(), Some(store.clone()), METER_INTERVAL)
                    .map_err(|e| e.to_string())?,
            ),
            Some(
                MeterStreamer::spawn(subs.clone(), bank.clone(), meter_sock, METER_INTERVAL)
                    .map_err(|e| e.to_string())?,
            ),
        )
    } else {
        (None, None)
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
    let osc = match osc_att {
        Some(att) => Some(
            OscServer::bind(
                store.clone(),
                att,
                OscConfig {
                    bind: SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, args.osc_port),
                    identity: args.identity.clone(),
                    advertise_ip: args.ip,
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
        if args.meters { "on (synthetic)" } else { "off" },
        args.idle
    ));

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
        }
    }
    log("shutting down");
    server.stop();
    if let Some(o) = osc {
        o.stop();
    }
    if let Some(s) = streamer {
        s.stop();
    }
    if let Some(s) = synth {
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
        assert!(args(&["--name", "a,b"]).is_err());
    }
}
