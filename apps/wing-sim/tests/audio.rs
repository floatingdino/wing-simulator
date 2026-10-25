//! T026 integration tests for the daemon's audio path.
//!
//! * `offline_sine_*`: free-running (offline-clocked) console: sine on USB In 1 -> ch 1 (trim,
//!   EQ) -> main 1 (T036 mix) -> USB Out 1/2 per the output patch -> host bridge (mock device
//!   side: the bridge consumer rendered by the test).
//! * `rt_path_does_not_allocate`: counting allocator around `Console::cycle` and the shm
//!   `service` loop (with parameter changes and routing swaps in between).
//! * `load_48_strips`: cycle time of the full console (40 ch + 8 aux, all processing on).
//! * `service_*`: the real `AudioService` (control thread + RT thread, os_sync wakes) against
//!   `wing_shm::sim::DriverSim` (Rust port of the driver's ring logic) on a private shm name:
//!   attach, processing, sample-rate change, detach -> loopback, incompatible (v1) segment ->
//!   control-only, segment recreated -> reattach.

use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering::Relaxed};
use std::sync::Arc;
use std::time::{Duration, Instant};

use wing_engine::meters::LevelBank;
use wing_hostio::{BridgeState, HostIoConfig, Route};
use wing_model::ParamStore;
use wing_shm::sim::DriverSim;
use wing_shm::{DaemonLink, Header, OpenOptions, Segment, WakeMode, CHANNELS};
use wing_sim::audio::console::{Console, ConsoleConfig};
use wing_sim::audio::driver::{service, AudioOptions, AudioService};
use wing_sim::audio::stats::AudioStats;
use wing_sim::audio::ConsoleControl;
use wing_sim::audio::{hostio, prepare, LogFn};

// ---- counting allocator -------------------------------------------------------------------------

struct Counting;
static COUNTING: AtomicBool = AtomicBool::new(false);
static ALLOCS: AtomicUsize = AtomicUsize::new(0);

thread_local! {
    static COUNT_THIS_THREAD: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

fn counted() -> bool {
    COUNTING.load(Relaxed) && COUNT_THIS_THREAD.try_with(|c| c.get()).unwrap_or(false)
}

unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, l: Layout) -> *mut u8 {
        if counted() {
            ALLOCS.fetch_add(1, Relaxed);
        }
        System.alloc(l)
    }
    unsafe fn dealloc(&self, p: *mut u8, l: Layout) {
        if counted() {
            ALLOCS.fetch_add(1, Relaxed);
        }
        System.dealloc(p, l)
    }
    unsafe fn realloc(&self, p: *mut u8, l: Layout, n: usize) -> *mut u8 {
        if counted() {
            ALLOCS.fetch_add(1, Relaxed);
        }
        System.realloc(p, l, n)
    }
}

#[global_allocator]
static GLOBAL: Counting = Counting;

// ---- helpers ------------------------------------------------------------------------------------

const FS: u32 = 48_000;
const BLOCK: usize = 64;

fn set(store: &ParamStore, path: &str, v: &str) {
    let id = store
        .lookup(path)
        .unwrap_or_else(|| panic!("no such node {path}"));
    store
        .set_text(id, v, 0)
        .unwrap_or_else(|e| panic!("{path} = {v}: {e:?}"));
}

/// ch 1 <- USB 1 with +6 dB trim, fader/main at 0 dB; USB Out 1/2 = main 1 L/R, USB Out 3 =
/// USB In 3 (direct patch), USB Out 4 = ch 1 pre-fader L (user signal).
fn scene(store: &ParamStore) {
    set(store, "ch/1/in/conn/grp", "USB");
    set(store, "ch/1/in/conn/in", "1");
    set(store, "io/in/USB/1/mute", "0");
    // mono source (the factory USB legs are stereo pairs; the router decodes the source mode)
    set(store, "io/in/USB/1/mode", "M");
    set(store, "io/in/USB/3/mute", "0");
    set(store, "ch/1/in/set/trim", "6");
    set(store, "ch/1/fdr", "0");
    set(store, "ch/1/main/1/on", "1");
    set(store, "ch/1/main/1/lvl", "0");
    set(store, "main/1/fdr", "0");
    set(store, "main/1/mute", "0");
    set(store, "io/out/USB/1/grp", "MAIN");
    set(store, "io/out/USB/1/in", "1");
    set(store, "io/out/USB/2/grp", "MAIN");
    set(store, "io/out/USB/2/in", "2");
    set(store, "io/out/USB/3/grp", "USB");
    set(store, "io/out/USB/3/in", "3");
}

fn console_cfg() -> ConsoleConfig {
    ConsoleConfig {
        sample_rate: FS,
        block: BLOCK,
        workers: 2,
        rt_workers: false,
        workgroup: None,
        // metering on: the alloc and load tests cover it (T039)
        meters: Some(LevelBank::shared()),
    }
}

fn sine(n: u64, f: f64, amp: f64) -> f32 {
    (amp * (2.0 * std::f64::consts::PI * f * n as f64 / FS as f64).sin()) as f32
}

fn rms_db(x: &[f32]) -> f64 {
    let p = x.iter().map(|v| (*v as f64).powi(2)).sum::<f64>() / x.len().max(1) as f64;
    10.0 * p.max(1e-30).log10()
}

/// Expected channel -> main 1 gain: trim +6 dB, fader 0 dB, centre pan (-3.01 dB, T036 law).
const MAIN_DB: f64 = 6.0 - 3.0103;

struct Rig {
    store: ParamStore,
    routing: ConsoleControl,
}

fn rig() -> Rig {
    let mut store = ParamStore::new();
    let mut routing = prepare(&mut store);
    scene(&store);
    routing.refresh(&store);
    Rig { store, routing }
}

/// Run `cycles` blocks with a 1 kHz -20 dBFS-peak sine on USB In 1 (and a 300 Hz one on USB In
/// 3); returns (USB Out 1, USB Out 2, USB Out 3) concatenated.
fn run_sine(console: &mut Console, t0: u64, cycles: usize) -> [Vec<f32>; 3] {
    let mut out: [Vec<f32>; 3] = Default::default();
    for c in 0..cycles {
        let base = t0 + (c * BLOCK) as u64;
        console.cycle(
            |ch, buf| match ch {
                0 => buf
                    .iter_mut()
                    .enumerate()
                    .for_each(|(i, s)| *s = sine(base + i as u64, 1000.0, 0.1)),
                2 => buf
                    .iter_mut()
                    .enumerate()
                    .for_each(|(i, s)| *s = sine(base + i as u64, 300.0, 0.2)),
                _ => buf.fill(0.0),
            },
            |ch, buf| {
                if ch < 3 {
                    out[ch].extend_from_slice(buf);
                }
            },
        );
    }
    out
}

// ---- offline ------------------------------------------------------------------------------------

#[test]
fn offline_sine_through_channel_main_usb_out_and_bridge() {
    let Rig { store, mut routing } = rig();
    // Host bridge "mock device": mon/1 (the default route: monitor A = MAIN.1 with solo
    // priority) -> 2 channels, consumer rendered here.
    let hcfg = HostIoConfig {
        routes: vec![Route::parse("mon/1 -> \"Mock\" 1/2").unwrap()],
        ..HostIoConfig::default()
    };
    let mut planned = hostio::plan(&hcfg, FS, BLOCK).unwrap();
    let p = planned.pop().unwrap();
    let mut consumer = p.consumer;
    consumer.configure(FS as f64, 64);
    consumer.start();
    let stats = Arc::new(AudioStats::default());
    let mut console = Console::build(
        console_cfg(),
        &store,
        &mut routing,
        None,
        vec![(p.producer, p.taps)],
        stats.clone(),
    )
    .unwrap();

    // Settle (fader ramps), then measure: trim +6 dB, flat EQ, centre pan -3 dB (T036
    // constant-power channel pan law): +2.99 dB.
    run_sine(&mut console, 0, 50);
    let [l, r, direct] = run_sine(&mut console, 50 * BLOCK as u64, 150);
    let input_db = rms_db(
        &(0..l.len() as u64)
            .map(|n| sine(n, 1000.0, 0.1))
            .collect::<Vec<_>>(),
    );
    let gain = rms_db(&l) - input_db;
    assert!((gain - MAIN_DB).abs() < 0.1, "main L gain {gain:.2} dB");
    assert!((rms_db(&r) - rms_db(&l)).abs() < 0.01, "L/R differ");
    // USB Out 3 patched straight to USB In 3: bit exact.
    let t = 200 * BLOCK as u64 - direct.len() as u64;
    for (i, &v) in direct.iter().enumerate() {
        assert_eq!(v, sine(t + i as u64, 300.0, 0.2));
    }

    // Host bridge got monitor A = main 1: render from the device side and check the level.
    // Simulated clock: one block = 1.333 ms; the console stamps each block as a burst (T027).
    let mut t = 200 * BLOCK as u64;
    let bstats_ok = consumer.stats().clone();
    let period = BLOCK as u64 * 1_000_000_000 / FS as u64;
    let mut now = 1_000_000_000u64;
    consumer.render(64, now, |_, _| {}); // flushes what the settle cycles pushed
    let mut primed = false;
    let mut bridge_db = |console: &mut Console, t: &mut u64| {
        let mut dev: Vec<f32> = Vec::new();
        // First call: a few bursts of headroom before the device starts pulling.
        let skip = if primed { 0 } else { 6 };
        primed = true;
        for i in 0..40 + skip {
            if i >= skip {
                consumer.render(64, now, |_frame, x| dev.push(x[0]));
            }
            run_sine(console, *t, 1);
            now += period;
            console.end_burst(now, BLOCK);
            *t += BLOCK as u64;
        }
        rms_db(&dev[dev.len() - 1024..]) - input_db
    };
    let bdb = bridge_db(&mut console, &mut t);
    assert!((bdb - MAIN_DB).abs() < 0.5, "bridge level {bdb:.2} dB");
    assert_eq!(bstats_ok.snapshot().state, BridgeState::Running);

    // Solo priority: soloing ch 1 (PFL, PFL dim 0) puts its pre-fader signal (+6 dB trim, no
    // pan) on monitor A instead of main 1; clearing the solo returns to main 1.
    set(&store, "cfg/mon/1/pfldim", "0");
    set(&store, "ch/1/$solo", "1");
    routing.refresh(&store);
    let bdb = bridge_db(&mut console, &mut t);
    assert!((bdb - 6.0).abs() < 0.5, "solo level {bdb:.2} dB");
    assert_eq!(
        store.get_f64(store.lookup("$stat/solo").unwrap()),
        Some(1.0)
    );
    set(&store, "$stat/solo", "0");
    routing.refresh(&store);
    let bdb = bridge_db(&mut console, &mut t);
    assert!((bdb - MAIN_DB).abs() < 0.5, "after clear {bdb:.2} dB");
    assert_eq!(
        store.get_f64(store.lookup("ch/1/$solo").unwrap()),
        Some(0.0)
    );

    // EQ: STD band 2 +6 dB at 1 kHz on top of the trim (live edit through the pipeline).
    set(&store, "ch/1/eq/on", "1");
    set(&store, "ch/1/eq/9", "1000"); // 2f
    set(&store, "ch/1/eq/8", "6"); // 2g
    run_sine(&mut console, t, 100);
    let [l, ..] = run_sine(&mut console, t + 100 * BLOCK as u64, 150);
    let gain = rms_db(&l) - input_db;
    assert!(
        (gain - (MAIN_DB + 6.0)).abs() < 0.6,
        "trim + EQ gain {gain:.2} dB"
    );

    // Mute the channel: main goes silent (after the 5 ms one-pole ramp reaches -100 dB).
    set(&store, "ch/1/mute", "1");
    run_sine(&mut console, 0, 60);
    let [l, ..] = run_sine(&mut console, 0, 10);
    assert!(l.iter().all(|&v| v == 0.0));
    set(&store, "ch/1/mute", "0");

    // Re-patch USB Out 1 to USB In 1 (routing table swap on the RT side):
    set(&store, "io/out/USB/1/grp", "USB");
    set(&store, "io/out/USB/1/in", "1");
    assert!(routing.refresh(&store));
    run_sine(&mut console, 0, 1);
    let [l, ..] = run_sine(&mut console, 1000, 1);
    assert_eq!(l[5], sine(1005, 1000.0, 0.1));

    let s = stats.snapshot();
    assert!(s.cycles > 400 && s.load_pct() > 0.0);
}

// ---- allocations --------------------------------------------------------------------------------

#[test]
fn rt_path_does_not_allocate() {
    let Rig { store, mut routing } = rig();
    let hcfg = HostIoConfig {
        routes: vec![Route::parse("usb_out/1-2 -> \"Mock\" 1/2").unwrap()],
        ..HostIoConfig::default()
    };
    let p = hostio::plan(&hcfg, FS, BLOCK).unwrap().pop().unwrap();
    let mut consumer = p.consumer;
    consumer.configure(FS as f64, 64);
    consumer.start();
    let mut console = Console::build(
        console_cfg(),
        &store,
        &mut routing,
        None,
        vec![(p.producer, p.taps)],
        Arc::new(AudioStats::default()),
    )
    .unwrap();

    let name = format!("/wsa.alloc.{}", std::process::id());
    let _ = Segment::unlink(&name);
    let create = OpenOptions {
        create: true,
        ..Default::default()
    };
    let drv = DriverSim::new(Segment::open(&name, &create).unwrap(), FS);
    let mut link = DaemonLink::attach(&name, &OpenOptions::default(), WakeMode::Auto).unwrap();
    let play: Vec<f32> = (0..128 * CHANNELS)
        .map(|i| (i as f32 * 0.01).sin())
        .collect();
    let mut rec = vec![0.0f32; 128 * CHANNELS];

    COUNT_THIS_THREAD.with(|c| c.set(true));
    // The counter works.
    COUNTING.store(true, Relaxed);
    drop(std::hint::black_box(Box::new(1u64)));
    COUNTING.store(false, Relaxed);
    assert_eq!(ALLOCS.swap(0, Relaxed), 2);
    for pass in 0..3 {
        // Control-side changes (allocation allowed): parameters, a model swap, a re-patch.
        set(&store, "ch/1/eq/on", if pass % 2 == 0 { "1" } else { "0" });
        set(&store, "ch/2/dyn/on", "1");
        set(
            &store,
            "ch/3/gate/mdl",
            if pass % 2 == 0 { "DUCK" } else { "GATE" },
        );
        set(&store, "ch/1/fdr", if pass % 2 == 0 { "-3" } else { "0" });
        set(&store, "io/out/USB/5/grp", "USB");
        set(
            &store,
            "io/out/USB/5/in",
            if pass % 2 == 0 { "5" } else { "6" },
        );
        routing.refresh(&store);
        // Pass 0 warms up lazily initialised statics.
        COUNTING.store(pass > 0, Relaxed);
        for k in 0..60i64 {
            let t = (pass * 60 + k) * 128;
            console.cycle(|_, b| b.fill(0.25), |_, _| {});
            drv.write_output(t, &play, 128, CHANNELS);
            link.heartbeat();
            link.wait(Duration::from_millis(1));
            service(&mut link, &mut console);
            drv.read_input(t, &mut rec, 128, CHANNELS);
        }
        COUNTING.store(false, Relaxed);
        consumer.render(256, wing_hostio::host_time_ns(), |_, _| {});
        routing.routing.collect();
    }
    COUNT_THIS_THREAD.with(|c| c.set(false));
    assert_eq!(ALLOCS.load(Relaxed), 0, "RT path allocated/freed");
    drop(link);
    let _ = Segment::unlink(&name);
}

// ---- load ---------------------------------------------------------------------------------------

#[test]
fn load_48_strips() {
    let Rig { store, mut routing } = rig();
    // Everything on in all 48 strips: filters, gate, PRE EQ, EQ, compressor.
    for i in 1..=40 {
        let c = format!("ch/{i}");
        set(&store, &format!("{c}/in/conn/grp"), "USB");
        set(&store, &format!("{c}/in/conn/in"), &i.to_string());
        set(&store, &format!("{c}/fdr"), "0");
        for (p, v) in [
            ("flt/lc", "1"),
            ("flt/hc", "1"),
            ("gate/on", "1"),
            ("peq/on", "1"),
            ("eq/on", "1"),
            ("dyn/on", "1"),
        ] {
            set(&store, &format!("{c}/{p}"), v);
        }
    }
    for i in 1..=8 {
        let a = format!("aux/{i}");
        set(&store, &format!("{a}/in/conn/grp"), "USB");
        set(&store, &format!("{a}/in/conn/in"), &(40 + i).to_string());
        set(&store, &format!("{a}/fdr"), "0");
        set(&store, &format!("{a}/eq/on"), "1");
        set(&store, &format!("{a}/dyn/on"), "1");
    }
    assert!(routing.refresh(&store));
    let stats = Arc::new(AudioStats::default());
    let mut console = Console::build(
        ConsoleConfig {
            workers: ConsoleConfig::default().workers,
            ..console_cfg()
        },
        &store,
        &mut routing,
        None,
        Vec::new(),
        stats.clone(),
    )
    .unwrap();
    let mut n = 0u64;
    let mut cycle = |c: &mut Console| {
        c.cycle(
            |ch, b| {
                for (i, s) in b.iter_mut().enumerate() {
                    *s = sine(n + i as u64, 100.0 * (ch + 1) as f64, 0.3);
                }
            },
            |_, _| {},
        );
        n += BLOCK as u64;
    };
    for _ in 0..200 {
        cycle(&mut console);
    }
    let before = stats.snapshot();
    stats.reset_peak();
    for _ in 0..2000 {
        cycle(&mut console);
    }
    let d = stats.snapshot().since(&before);
    println!(
        "48 strips, {} workers, {BLOCK} frames @ {FS} Hz: load {:.1}% (peak {:.1}%), p50 {} us, p99 \
         {} us ({} build)",
        console.engine().num_workers(),
        d.load_pct(),
        d.peak_load_pct(),
        d.percentile_ns(0.5) / 1000,
        d.percentile_ns(0.99) / 1000,
        if cfg!(debug_assertions) {
            "debug"
        } else {
            "release"
        }
    );
    assert_eq!(d.cycles, 2000);
    if !cfg!(debug_assertions) {
        assert!(d.load_pct() < 50.0, "load {:.1}%", d.load_pct());
    }
}

// ---- the service against a simulated driver -----------------------------------------------------

struct TmpShm(String);
impl TmpShm {
    fn new(tag: &str) -> Self {
        let n = format!("/wsa.{tag}.{}", std::process::id());
        let _ = Segment::unlink(&n);
        Self(n)
    }
}
impl Drop for TmpShm {
    fn drop(&mut self) {
        let _ = Segment::unlink(&self.0);
    }
}

fn capture_log() -> (LogFn, Arc<std::sync::Mutex<Vec<String>>>) {
    let lines = Arc::new(std::sync::Mutex::new(Vec::new()));
    let l2 = lines.clone();
    (
        Arc::new(move |m: &str| {
            eprintln!("{m}");
            l2.lock().unwrap().push(m.to_string());
        }),
        lines,
    )
}

fn wait_for(what: &str, timeout: Duration, mut f: impl FnMut() -> bool) {
    let t0 = Instant::now();
    while !f() {
        assert!(t0.elapsed() < timeout, "timed out waiting for {what}");
        std::thread::sleep(Duration::from_millis(5));
    }
}

fn spawn_service(
    name: &str,
    store: ParamStore,
    routing: ConsoleControl,
) -> (AudioService, Arc<std::sync::Mutex<Vec<String>>>) {
    let (log, lines) = capture_log();
    let svc = AudioService::spawn(
        Arc::new(store),
        routing,
        AudioOptions {
            shm_name: name.to_string(),
            console: console_cfg(),
            rt: true,
            hostio: None,
            status_every: None,
        },
        log,
    )
    .unwrap();
    (svc, lines)
}

/// Test playback: 1 kHz on USB In 1, 300 Hz on USB In 3.
fn play_block(t: i64, buf: usize) -> Vec<f32> {
    (0..buf * CHANNELS)
        .map(|i| {
            let (f, c) = (i / CHANNELS, i % CHANNELS);
            let n = (t + f as i64) as u64;
            match c {
                0 => sine(n, 1000.0, 0.1),
                2 => sine(n, 300.0, 0.2),
                _ => 0.0,
            }
        })
        .collect()
}

/// Lock-step driver cycle at input time `t + delay`: like the HAL, playback runs ahead of
/// capture (WriteMix for `[t + delay, +buf)`), then the capture of `[t, t + buf)` is read once
/// the daemon has produced it. Call [`prefill`] once per epoch first.
fn drive(drv: &DriverSim, t: i64, buf: usize, rec: &mut [f32]) {
    let h: &Header = drv.segment().header();
    let delay = h.input_delay_frames as i64;
    drv.write_output(t + delay, &play_block(t + delay, buf), buf, CHANNELS);
    wait_for("daemon block", Duration::from_secs(20), || {
        h.from_daemon.write_time.load(Relaxed) >= t + buf as i64
    });
    drv.read_input(t + delay, rec, buf, CHANNELS);
}

/// The first `delay` frames of an epoch's playback.
fn prefill(drv: &DriverSim) {
    let delay = drv.segment().header().input_delay_frames as usize;
    drv.write_output(0, &play_block(0, delay), delay, CHANNELS);
}

#[test]
fn service_attaches_processes_follows_rate_and_detaches() {
    let shm = TmpShm::new("svc");
    let create = OpenOptions {
        create: true,
        ..Default::default()
    };
    let drv = DriverSim::new(Segment::open(&shm.0, &create).unwrap(), FS);
    let Rig { store, routing } = rig();
    let (svc, lines) = spawn_service(&shm.0, store, routing);
    wait_for("attach", Duration::from_secs(60), || {
        drv.daemon_alive() && svc.status_line().starts_with("audio: 48000 Hz")
    });

    const BUF: usize = 128;
    let mut rec = vec![0.0f32; BUF * CHANNELS];
    let mut out3 = Vec::new();
    let mut out1 = Vec::new();
    prefill(&drv);
    for k in 0..200i64 {
        drive(&drv, k * BUF as i64, BUF, &mut rec);
        if k >= 100 {
            for f in 0..BUF {
                out1.push(rec[f * CHANNELS]);
                out3.push(rec[f * CHANNELS + 2]);
            }
        }
    }
    // USB Out 3 = USB In 3 exactly (through the driver ring and the daemon).
    for (i, &v) in out3.iter().enumerate() {
        let n = (100 * BUF + i) as u64;
        assert_eq!(v, sine(n, 300.0, 0.2), "frame {i}");
    }
    // USB Out 1 = main L = ch 1 (+6 dB trim).
    let g = rms_db(&out1)
        - rms_db(
            &(0..out1.len() as u64)
                .map(|n| sine(n, 1000.0, 0.1))
                .collect::<Vec<_>>(),
        );
    assert!((g - MAIN_DB).abs() < 0.1, "main via shm {g:.2} dB");
    assert_eq!(
        drv.segment().header().from_daemon.underruns.load(Relaxed),
        0
    );
    assert!(
        svc.status_line().contains("xruns 0"),
        "{}",
        svc.status_line()
    );

    // Sample-rate change: new epoch at 44.1 kHz -> console rebuilt, processing continues.
    drv.begin_epoch(44_100);
    wait_for("rebuild at 44.1k", Duration::from_secs(60), || {
        svc.status_line().starts_with("audio: 44100 Hz")
    });
    prefill(&drv);
    for k in 0..50i64 {
        drive(&drv, k * BUF as i64, BUF, &mut rec);
    }
    let rec3: Vec<f32> = (0..BUF).map(|f| rec[f * CHANNELS + 2]).collect();
    for (i, &v) in rec3.iter().enumerate() {
        assert_eq!(v, sine((49 * BUF + i) as u64, 300.0, 0.2));
    }

    // Shutdown detaches: the driver is in loopback immediately.
    svc.stop();
    assert!(!drv.daemon_alive());
    let log = lines.lock().unwrap().join("\n");
    assert!(log.contains("rebuilding the console"), "{log}");
    assert!(log.contains("falls back to loopback"), "{log}");
}

#[test]
fn service_reports_incompatible_driver_then_reattaches_after_reinstall() {
    let shm = TmpShm::new("v1");
    let create = OpenOptions {
        create: true,
        ..Default::default()
    };
    // An "old driver" segment: same name, layout version 1.
    let old = Segment::open(&shm.0, &create).unwrap();
    // SAFETY: test-only write to the version field of our own mapping.
    unsafe {
        old.base()
            .add(std::mem::offset_of!(Header, version))
            .cast::<u32>()
            .write_volatile(1);
    }
    let Rig { store, routing } = rig();
    let (svc, lines) = spawn_service(&shm.0, store, routing);
    wait_for("incompatible report", Duration::from_secs(10), || {
        svc.status_line().contains("layout v1")
    });
    assert!(
        svc.status_line().contains("outdated"),
        "{}",
        svc.status_line()
    );
    // The daemon must not have touched the old segment.
    assert_eq!(old.header().version, 1);
    assert_eq!(old.header().daemon_pid.load(Relaxed), 0);

    // "Reinstall": the new driver replaces the segment -> the daemon attaches.
    let opts = OpenOptions {
        replace_incompatible: true,
        ..Default::default()
    };
    let drv = DriverSim::new(Segment::open(&shm.0, &opts).unwrap(), FS);
    wait_for("attach after reinstall", Duration::from_secs(60), || {
        drv.daemon_alive() && svc.status_line().starts_with("audio: 48000 Hz")
    });
    let seg_id = drv.segment().header().segment_id;

    // Driver recreates its segment again (e.g. other input delay): the daemon reattaches.
    drop(drv);
    Segment::unlink(&shm.0).unwrap();
    let drv2 = DriverSim::new(Segment::open(&shm.0, &create).unwrap(), FS);
    assert_ne!(drv2.segment().header().segment_id, seg_id);
    wait_for("reattach", Duration::from_secs(60), || drv2.daemon_alive());
    let mut rec = vec![0.0f32; 128 * CHANNELS];
    prefill(&drv2);
    for k in 0..20i64 {
        drive(&drv2, k * 128, 128, &mut rec);
    }
    svc.stop();
    assert!(!drv2.daemon_alive());
    let log = lines.lock().unwrap().join("\n");
    assert!(log.contains("removed or recreated"), "{log}");
}

/// T027 startup priming: when the driver starts a new IO epoch (the DAW starts its device) the
/// RT loop warms the console up on silence before the first buffer arrives, and does not touch
/// the driver's rings doing so (no underruns, nothing written).
#[test]
fn rt_loop_warms_up_on_driver_start() {
    use wing_sim::audio::driver::{rt_loop, RtShared, EPOCH_WARM_UP_CYCLES};
    let Rig { store, mut routing } = rig();
    let console = Console::build(
        console_cfg(),
        &store,
        &mut routing,
        None,
        Vec::new(),
        Arc::new(AudioStats::default()),
    )
    .unwrap();
    let shm = TmpShm::new("warm");
    let create = OpenOptions {
        create: true,
        ..Default::default()
    };
    let drv = DriverSim::new(Segment::open(&shm.0, &create).unwrap(), FS);
    let link = DaemonLink::attach(&shm.0, &OpenOptions::default(), WakeMode::Auto).unwrap();
    let shared = Arc::new(RtShared::default());
    let s2 = shared.clone();
    let h = std::thread::spawn(move || rt_loop(link, console, s2, false, None));
    std::thread::sleep(Duration::from_millis(50));
    drv.begin_epoch(FS); // DAW starts IO; no buffer written yet
    std::thread::sleep(Duration::from_millis(100));
    shared.stop.store(true, Relaxed);
    let (_link, console, _) = h.join().unwrap();
    assert_eq!(console.engine().cycle(), EPOCH_WARM_UP_CYCLES as u64);
    assert_eq!(shared.frames.load(Relaxed), 0);
    let hd = drv.segment().header();
    assert_eq!(hd.from_daemon.underruns.load(Relaxed), 0);
    assert_eq!(hd.from_daemon.write_time.load(Relaxed), 0);
}
