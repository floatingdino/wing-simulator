//! Run only the daemon's audio path (no network servers, factory scene or `--scene`): attach to
//! the driver, process, print the status line every second. For checking a driver install
//! without disturbing a running `wing-sim` on ports 2222/2223.
//!
//! `cargo run --release -p wing-sim --example audio_only -- [--seconds 10] [--scene X.snap]
//!  [--hostio hostio.toml | --default-hostio] [--shm-name /wingsim.audio.v1]`

use std::sync::Arc;
use std::time::{Duration, Instant};

use wing_model::{ParamStore, Snapshot};
use wing_sim::audio::{self, AudioOptions, AudioService, HostIoSetting, LogFn};

fn main() {
    let mut args = std::env::args().skip(1);
    let mut seconds = 10.0f64;
    let mut scene = None;
    let mut hostio = HostIoSetting::Off;
    let mut opts = AudioOptions {
        status_every: None,
        ..AudioOptions::default()
    };
    while let Some(a) = args.next() {
        let mut val = || args.next().expect("missing value");
        match a.as_str() {
            "--seconds" => seconds = val().parse().expect("--seconds"),
            "--scene" => scene = Some(val()),
            "--hostio" => hostio = HostIoSetting::from_file(&val()).expect("--hostio"),
            "--default-hostio" => hostio = HostIoSetting::Default,
            "--shm-name" => opts.shm_name = val(),
            _ => {
                eprintln!("usage: audio_only [--seconds S] [--scene X.snap] [--hostio F | --default-hostio] [--shm-name N]");
                std::process::exit(2);
            }
        }
    }
    let mut store = ParamStore::new();
    if let Some(p) = scene {
        let text = std::fs::read_to_string(&p).expect("read scene");
        Snapshot::load(&store, &text).expect("load scene");
    }
    let routing = audio::prepare(&mut store);
    let (cfg, note) = hostio.resolve();
    eprintln!("{note}");
    opts.hostio = cfg;
    let t0 = Instant::now();
    let log: LogFn =
        Arc::new(move |m: &str| eprintln!("[{:7.3}s] {m}", t0.elapsed().as_secs_f64()));
    let svc = AudioService::spawn(Arc::new(store), routing, opts, log.clone()).expect("spawn");
    while t0.elapsed().as_secs_f64() < seconds {
        std::thread::sleep(Duration::from_secs(1));
        log(&svc.status_line());
    }
    svc.stop();
}
