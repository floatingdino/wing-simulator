//! T071 soak harness: the full real audio service against the simulated driver (`DriverSim`) on
//! a PRIVATE shm name, with realistic timing, client load and long-run monitoring. See docs/soak.md.
//!
//! ```text
//! cargo run --release -p wing-sim --example soak -- --minutes 60 \
//!     --shm-name /wingsim.soak.$$ --scene fixtures/PUBLIC.snap
//! ```
//!
//! What runs (everything in this process; nothing touches ports 2222/2223, the production shm
//! `/wingsim.audio.v1`, the HAL driver or a real audio device):
//!
//! * `AudioService` (control thread, RT thread, engine workers, real meters + RTA feed) with a
//!   host-output bridge whose CoreAudio side is replaced by a simulated hardware device
//!   ([`wing_sim::audio::hostio::DeviceFactory`]) pulling on a clock that wanders +-`--drift-ppm`
//!   against the host clock.
//! * A simulated HAL driver thread (`DriverSim`) cycling at the host clock with alternating
//!   64 / 512-frame bursts; every phase change restarts IO (new epoch, like a DAW restart).
//! * In-process native-protocol (TCP) and OSC (UDP) servers on ephemeral loopback ports, the
//!   control socket + status.json (private state dir), the meter source and the RTA analyser,
//!   all as in the daemon.
//! * A client thread: OSC and native parameter storms, model swaps, A/B toggles, scene recalls,
//!   control-socket status polls.
//! * A counting allocator (RT threads only: `wing-audio-rt`, `wing-worker-*`), RSS sampling,
//!   cycle-time histograms, bridge statistics, a CSV time series and a pass/fail summary.

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::ffi::{c_char, c_int, c_void};
use std::io::{Read, Write};
use std::net::{Ipv4Addr, SocketAddrV4, TcpStream, UdpSocket};
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering::*};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use wing_engine::meters::LevelBank;
use wing_engine::rt::{set_current_thread_time_constraint, TimeConstraint};
use wing_engine::rta::RtaFeed;
use wing_hostio::{
    host_time_ns, BridgeConsumer, BridgeState, BridgeStats, HostIoConfig, Route, StatsSnapshot,
};
use wing_model::{NodeId, ParamStore, ParamType, Snapshot};
use wing_proto::nrp::{NrpEncoder, Token};
use wing_proto::osc::{Arg, Message, OscAttachment, OscConfig, OscServer};
use wing_proto::server::{
    EventFn, Hub, LiveInfo, MeterBank, NativeServer, ServerConfig, ServerShared,
};
use wing_shm::sim::DriverSim;
use wing_shm::{OpenOptions, Segment, CHANNELS};
use wing_sim::audio::driver::{AudioOptions, AudioService};
use wing_sim::audio::hostio::DeviceFactory;
use wing_sim::audio::stats::Snapshot as CycleSnapshot;
use wing_sim::audio::{self, ConsoleConfig, LogFn};
use wing_sim::control;
use wing_sim::meters::meter_source;
use wing_sim::rta::RtaService;

const FS: u32 = 48_000;
const DEVICE_NAME: &str = "Soak Simulated Device";

// ---- counting allocator (RT threads only) ------------------------------------------------------

struct Counting;

static COUNTING: AtomicBool = AtomicBool::new(false);
static RT_ALLOCS: AtomicU64 = AtomicU64::new(0);
static RT_DEALLOCS: AtomicU64 = AtomicU64::new(0);
static RT_REALLOCS: AtomicU64 = AtomicU64::new(0);
static RT_BYTES: AtomicU64 = AtomicU64::new(0);

/// Call stacks of the first offenders (raw return addresses, resolved at the end).
const OFFENDERS: usize = 8;
const DEPTH: usize = 14;
static OFF_N: AtomicUsize = AtomicUsize::new(0);
static OFF_STACK: [AtomicUsize; OFFENDERS * DEPTH] =
    [const { AtomicUsize::new(0) }; OFFENDERS * DEPTH];
static OFF_INFO: [AtomicUsize; OFFENDERS] = [const { AtomicUsize::new(0) }; OFFENDERS];

extern "C" {
    fn backtrace(buf: *mut *mut c_void, size: c_int) -> c_int;
    fn backtrace_symbols(buf: *const *mut c_void, size: c_int) -> *mut *mut c_char;
}

thread_local! {
    /// 0 = not classified yet, 1 = RT thread, 2 = other. No destructor: safe inside the allocator.
    static CLASS: Cell<u8> = const { Cell::new(0) };
    static TRIES: Cell<u8> = const { Cell::new(0) };
}

/// Whether the calling thread is an audio thread (by its pthread name: no allocation).
fn on_rt_thread() -> bool {
    CLASS
        .try_with(|c| match c.get() {
            1 => true,
            2 => false,
            _ => {
                #[cfg_attr(not(unix), allow(unused_mut))]
                let mut buf = [0 as c_char; 64];
                // SAFETY: valid buffer of the stated length.
                #[cfg(unix)]
                let r =
                    unsafe { libc::pthread_getname_np(libc::pthread_self(), buf.as_mut_ptr(), 64) };
                #[cfg(not(unix))]
                let r = 1;
                if r != 0 {
                    return false;
                }
                let bytes: &[u8] =
                    // SAFETY: c_char and u8 have the same layout.
                    unsafe { std::slice::from_raw_parts(buf.as_ptr() as *const u8, 64) };
                let len = bytes.iter().position(|&b| b == 0).unwrap_or(64);
                let n = &bytes[..len];
                if n.is_empty() {
                    // unnamed (or not yet named): retry a few times, then call it "other"
                    let t = TRIES.try_with(|t| {
                        t.set(t.get().saturating_add(1));
                        t.get()
                    });
                    if t.unwrap_or(255) > 8 {
                        c.set(2);
                    }
                    return false;
                }
                let rt = n.starts_with(b"wing-audio-rt") || n.starts_with(b"wing-worker-");
                c.set(if rt { 1 } else { 2 });
                rt
            }
        })
        .unwrap_or(false)
}

#[inline]
fn note(kind: usize, size: usize) {
    if !COUNTING.load(Relaxed) || !on_rt_thread() {
        return;
    }
    match kind {
        0 => RT_ALLOCS.fetch_add(1, Relaxed),
        1 => RT_DEALLOCS.fetch_add(1, Relaxed),
        _ => RT_REALLOCS.fetch_add(1, Relaxed),
    };
    RT_BYTES.fetch_add(size as u64, Relaxed);
    let slot = OFF_N.fetch_add(1, Relaxed);
    if slot < OFFENDERS {
        let mut frames = [std::ptr::null_mut::<c_void>(); DEPTH];
        // SAFETY: frame-pointer walk into a stack array (no allocation).
        let n = unsafe { backtrace(frames.as_mut_ptr(), DEPTH as c_int) }.max(0) as usize;
        for (i, f) in frames.iter().enumerate().take(n) {
            OFF_STACK[slot * DEPTH + i].store(*f as usize, Relaxed);
        }
        OFF_INFO[slot].store(kind | (size << 2), Relaxed);
    }
}

unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, l: Layout) -> *mut u8 {
        note(0, l.size());
        System.alloc(l)
    }
    unsafe fn dealloc(&self, p: *mut u8, l: Layout) {
        note(1, l.size());
        System.dealloc(p, l)
    }
    unsafe fn realloc(&self, p: *mut u8, l: Layout, n: usize) -> *mut u8 {
        note(2, n);
        System.realloc(p, l, n)
    }
}

#[global_allocator]
static GLOBAL: Counting = Counting;

fn rt_alloc_events() -> u64 {
    RT_ALLOCS.load(Relaxed) + RT_DEALLOCS.load(Relaxed) + RT_REALLOCS.load(Relaxed)
}

/// Resolve the recorded offender stacks (allocation allowed: counting is off by then).
fn offender_report() -> Vec<String> {
    let n = OFF_N.load(Relaxed).min(OFFENDERS);
    let mut out = Vec::new();
    for slot in 0..n {
        let info = OFF_INFO[slot].load(Relaxed);
        let kind = ["alloc", "dealloc", "realloc"][(info & 3).min(2)];
        let mut frames: Vec<*mut c_void> = (0..DEPTH)
            .map(|i| OFF_STACK[slot * DEPTH + i].load(Relaxed) as *mut c_void)
            .take_while(|p| !p.is_null())
            .collect();
        let mut s = format!("#{slot}: {kind} of {} bytes\n", info >> 2);
        if !frames.is_empty() {
            // SAFETY: `frames` holds `len` return addresses; the result is freed below.
            let syms = unsafe { backtrace_symbols(frames.as_mut_ptr(), frames.len() as c_int) };
            for (i, f) in frames.iter().enumerate() {
                let name = if syms.is_null() {
                    format!("{f:p}")
                } else {
                    // SAFETY: backtrace_symbols returned `len` NUL-terminated strings.
                    unsafe { std::ffi::CStr::from_ptr(*syms.add(i)) }
                        .to_string_lossy()
                        .into_owned()
                };
                s += &format!("    {name}\n");
            }
            if !syms.is_null() {
                // SAFETY: allocated by backtrace_symbols with malloc.
                unsafe { libc::free(syms as *mut c_void) };
            }
        }
        out.push(s);
    }
    out
}

// ---- small helpers -----------------------------------------------------------------------------

struct Rng(u64);
impl Rng {
    fn u64(&mut self) -> u64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        self.0.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
    fn unit(&mut self) -> f32 {
        (self.u64() >> 40) as f32 / (1u64 << 24) as f32
    }
    fn below(&mut self, n: usize) -> usize {
        (self.u64() % n.max(1) as u64) as usize
    }
}

fn f64_bits(a: &AtomicU64) -> f64 {
    f64::from_bits(a.load(Relaxed))
}

fn rss_mb() -> f64 {
    std::process::Command::new("ps")
        .args(["-o", "rss=", "-p", &std::process::id().to_string()])
        .output()
        .ok()
        .and_then(|o| {
            String::from_utf8_lossy(&o.stdout)
                .trim()
                .parse::<f64>()
                .ok()
        })
        .map_or(0.0, |kb| kb / 1024.0)
}

static STOP: AtomicBool = AtomicBool::new(false);
extern "C" fn on_signal(_: c_int) {
    STOP.store(true, SeqCst);
}

// ---- the simulated hardware device (host bridge consumer) ---------------------------------------

/// State shared between the simulated device threads and the sampler.
#[derive(Default)]
struct DeviceShared {
    /// True clock error of the device vs the host clock (ppm, f64 bits).
    hw_ppm: AtomicU64,
    callbacks: AtomicU64,
    /// Callbacks that started more than 1 ms after their ideal time (harness scheduling lag).
    late: AtomicU64,
    max_late_us: AtomicU64,
    /// Stats of the current bridge.
    stats: Mutex<Option<Arc<BridgeStats>>>,
}

struct DeviceCfg {
    buffer: usize,
    amp_ppm: f64,
    period_s: f64,
    rt: bool,
    t_start: Instant,
}

/// Device clock error at `t` seconds: a slow sinusoid between -amp and +amp.
fn hw_ppm_at(cfg: &DeviceCfg, t: f64) -> f64 {
    cfg.amp_ppm * (2.0 * std::f64::consts::PI * t / cfg.period_s).sin()
}

struct SimDevice {
    stop: Arc<AtomicBool>,
    join: Option<JoinHandle<()>>,
}
impl Drop for SimDevice {
    fn drop(&mut self) {
        self.stop.store(true, Relaxed);
        if let Some(j) = self.join.take() {
            let _ = j.join();
        }
    }
}

fn start_sim_device(
    mut cons: BridgeConsumer,
    shared: Arc<DeviceShared>,
    cfg: Arc<DeviceCfg>,
) -> SimDevice {
    cons.configure(FS as f64, cfg.buffer);
    cons.start();
    *shared.stats.lock().unwrap() = Some(cons.stats().clone());
    let stop = Arc::new(AtomicBool::new(false));
    let s2 = stop.clone();
    let join = std::thread::Builder::new()
        .name("soak-device".into())
        .spawn(move || {
            if cfg.rt {
                let period = cfg.buffer as u64 * 1_000_000_000 / FS as u64;
                let _ = set_current_thread_time_constraint(TimeConstraint::for_period(period));
            }
            let t0 = host_time_ns();
            let mut next_s = 0.0f64; // ideal device-callback time, seconds after t0
            let mut sink = 0.0f32;
            while !s2.load(Relaxed) {
                let ideal = t0 + (next_s * 1e9) as u64;
                let now = host_time_ns();
                if now < ideal {
                    std::thread::sleep(Duration::from_nanos((ideal - now).min(50_000_000)));
                    if host_time_ns() < ideal {
                        continue;
                    }
                }
                let late_us = host_time_ns().saturating_sub(ideal) / 1000;
                if late_us > 1000 {
                    shared.late.fetch_add(1, Relaxed);
                }
                shared.max_late_us.fetch_max(late_us, Relaxed);
                cons.render(cfg.buffer, ideal, |_, s| sink += s[0]);
                shared.callbacks.fetch_add(1, Relaxed);
                let ppm = hw_ppm_at(&cfg, cfg.t_start.elapsed().as_secs_f64());
                shared.hw_ppm.store(ppm.to_bits(), Relaxed);
                next_s += cfg.buffer as f64 / (FS as f64 * (1.0 + ppm * 1e-6));
            }
            std::hint::black_box(sink);
        })
        .expect("spawn device thread");
    SimDevice {
        stop,
        join: Some(join),
    }
}

// ---- the simulated HAL driver --------------------------------------------------------------------

#[derive(Default)]
struct HalShared {
    phase: AtomicUsize,
    /// IO restarts begun so far (bumped when the stop gap starts).
    restarts: AtomicU64,
    burst: AtomicUsize,
    /// Ms after the run start from which the current phase counts as settled.
    settled_ms: AtomicU64,
    cycles: AtomicU64,
    /// Driver underruns + overruns attributed to settled / transition periods.
    steady_xruns: AtomicU64,
    trans_xruns: AtomicU64,
    /// Cycles where the daemon had not delivered by 90 % of the period (read went ahead).
    late_reads: AtomicU64,
    /// Cycles during which the driver was in loopback (daemon considered dead).
    loopback_cycles: AtomicU64,
    stop: AtomicBool,
}

struct HalCfg {
    bursts: Vec<usize>,
    phase: Duration,
    settle: Duration,
    rt: bool,
    t_run: Instant,
}

fn hal_thread(drv: DriverSim, sh: Arc<HalShared>, cfg: HalCfg) -> DriverSim {
    let h = drv.segment().header();
    let delay = h.input_delay_frames as i64;
    let xr =
        || h.from_daemon.underruns.load(Relaxed) as u64 + h.to_daemon.overruns.load(Relaxed) as u64;
    let mut rng = Rng(0x1234_5678_9ABC_DEF1);
    let mut rec = vec![0.0f32; 4096 * CHANNELS];
    let mut play = vec![0.0f32; 4096 * CHANNELS];
    let mut noise = |buf: &mut [f32], n: usize| {
        // deterministic white noise at about -19 dBFS RMS on all 48 USB inputs
        for s in buf[..n * CHANNELS].iter_mut() {
            *s = (rng.unit() * 2.0 - 1.0) * 0.2;
        }
    };
    drv.set_running(true);
    let mut last_xr = xr();
    let mut phase = 0usize;
    'run: loop {
        let buf = cfg.bursts[phase % cfg.bursts.len()];
        let period_ns = buf as u64 * 1_000_000_000 / FS as u64;
        if phase > 0 {
            // DAW stops and restarts the device (new IO epoch)
            sh.restarts.fetch_add(1, Relaxed);
            drv.set_running(false);
            let t = Instant::now();
            while t.elapsed() < Duration::from_millis(500) {
                if sh.stop.load(Relaxed) {
                    break 'run;
                }
                std::thread::sleep(Duration::from_millis(10));
            }
            drv.begin_epoch(FS);
            drv.set_running(true);
            last_xr = xr();
        }
        let started = Instant::now();
        sh.phase.store(phase, Relaxed);
        sh.burst.store(buf, Relaxed);
        sh.settled_ms.store(
            (cfg.t_run.elapsed() + cfg.settle).as_millis() as u64,
            Relaxed,
        );
        if cfg.rt {
            let _ = set_current_thread_time_constraint(TimeConstraint::for_period(period_ns));
        }
        if phase > 0 {
            // the HAL's first IO cycle comes one buffer period after StartIO: the daemon warms
            // up on the new epoch meanwhile
            std::thread::sleep(Duration::from_nanos(period_ns));
        }
        noise(&mut play, delay as usize);
        drv.write_output(0, &play, delay as usize, CHANNELS);
        let mut t = 0i64;
        let mut deadline = Instant::now();
        while started.elapsed() < cfg.phase {
            if sh.stop.load(Relaxed) {
                break 'run;
            }
            // wait for the cycle's (host clock) start
            let now = Instant::now();
            if deadline > now {
                std::thread::sleep(deadline - now);
            }
            noise(&mut play, buf);
            drv.write_output(t + delay, &play, buf, CHANNELS);
            // capture of [t, t+buf) needs the daemon's output for it; give it up to 90 % of the
            // period, then read what is there (a driver underrun if it was not delivered)
            let limit = deadline + Duration::from_nanos(period_ns * 9 / 10);
            let mut waited = false;
            while h.from_daemon.write_time.load(Relaxed) < t + buf as i64 {
                if Instant::now() >= limit || sh.stop.load(Relaxed) {
                    waited = true;
                    break;
                }
                std::thread::sleep(Duration::from_micros(30));
            }
            if waited {
                sh.late_reads.fetch_add(1, Relaxed);
            }
            drv.read_input(t + delay, &mut rec, buf, CHANNELS);
            if h.loopback_active.load(Relaxed) != 0 {
                sh.loopback_cycles.fetch_add(1, Relaxed);
            }
            let now_xr = xr();
            if now_xr > last_xr {
                let in_trans =
                    (cfg.t_run.elapsed().as_millis() as u64) < sh.settled_ms.load(Relaxed);
                let c = if in_trans {
                    &sh.trans_xruns
                } else {
                    &sh.steady_xruns
                };
                c.fetch_add(now_xr - last_xr, Relaxed);
                last_xr = now_xr;
            }
            sh.cycles.fetch_add(1, Relaxed);
            t += buf as i64;
            deadline += Duration::from_nanos(period_ns);
            // never try to catch up more than a few periods (the host was suspended / starved)
            let now = Instant::now();
            if now > deadline + Duration::from_nanos(period_ns * 8) {
                deadline = now;
            }
        }
        phase += 1;
    }
    drv.set_running(false);
    drv
}

// ---- client load -------------------------------------------------------------------------------

#[derive(Default)]
struct Counters {
    osc: AtomicU64,
    native: AtomicU64,
    recalls: AtomicU64,
    ab: AtomicU64,
    swaps: AtomicU64,
    storms: AtomicU64,
    ctl: AtomicU64,
    errors: AtomicU64,
}

struct Pool {
    params: Vec<NodeId>,
    models: Vec<NodeId>,
}

fn build_pool(store: &ParamStore) -> Pool {
    let schema = store.schema();
    let (mut params, mut models) = (Vec::new(), Vec::new());
    for id in schema.ids() {
        if !schema.is_param(id) || schema.is_live(id) {
            continue;
        }
        let Some(d) = store.desc(id) else { continue };
        if d.ty == ParamType::Str {
            continue;
        }
        let p = schema.path(id);
        if !["ch/", "aux/", "bus/", "main/", "mtx/", "fx/"]
            .iter()
            .any(|x| p.starts_with(x))
        {
            continue;
        }
        if p.contains("/$")
            || p.contains("/in/conn")
            || p.ends_with("/mute")
            || (p.ends_with("/on") && p.starts_with("main/"))
        {
            continue;
        }
        if p.ends_with("/mdl") && d.ty == ParamType::Enum {
            models.push(id);
        } else {
            params.push(id);
        }
    }
    Pool { params, models }
}

/// A random value for `id` as OSC text and as a native token (via a scratch store).
fn random_value(
    scratch: &ParamStore,
    id: NodeId,
    rng: &mut Rng,
) -> (String, Option<Token<'static>>) {
    let _ = scratch.set_norm(id, rng.unit(), 0);
    let mut txt = String::new();
    let _ = scratch.format_token(id, &mut txt);
    let tok = match scratch.desc(id).map(|d| d.ty) {
        Some(ParamType::LinF | ParamType::LogF | ParamType::Fader) => {
            scratch.get_f64(id).map(|v| Token::Float(v as f32))
        }
        Some(ParamType::Int | ParamType::Enum | ParamType::FEnum) => {
            scratch.get_f64(id).map(|v| v as i64).and_then(|v| {
                u8::try_from(v)
                    .ok()
                    .filter(|v| *v < 64)
                    .map(Token::SmallInt)
            })
        }
        _ => None,
    };
    (txt, tok)
}

struct Native {
    stream: TcpStream,
    enc: NrpEncoder,
    out: Vec<u8>,
}

impl Native {
    fn connect(addr: std::net::SocketAddr, name: &str) -> std::io::Result<Self> {
        let stream = TcpStream::connect(addr)?;
        stream.set_write_timeout(Some(Duration::from_secs(2)))?;
        let mut rd = stream.try_clone()?;
        rd.set_read_timeout(Some(Duration::from_millis(200)))?;
        std::thread::Builder::new()
            .name(name.into())
            .spawn(move || {
                let mut b = vec![0u8; 65536];
                // drain until the server closes the connection or the process exits
                while !STOP.load(Relaxed) {
                    match rd.read(&mut b) {
                        Ok(0) => break,
                        Ok(_) => {}
                        Err(e)
                            if matches!(
                                e.kind(),
                                std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut
                            ) => {}
                        Err(_) => break,
                    }
                }
            })?;
        Ok(Self {
            stream,
            enc: NrpEncoder::new(),
            out: Vec::new(),
        })
    }
    fn set(&mut self, hash: u32, tok: &Token) {
        let _ = self.enc.control(&Token::NavHash(hash), &mut self.out);
        let _ = self.enc.control(tok, &mut self.out);
    }
    fn flush(&mut self) -> bool {
        let r = self.stream.write_all(&self.out).is_ok();
        self.out.clear();
        r
    }
}

struct ClientCfg {
    osc: std::net::SocketAddr,
    native: std::net::SocketAddr,
    ctl_socket: PathBuf,
    scene_text: String,
    seed: u64,
    storm_every: Duration,
    storm_len: Duration,
    swap_every: Duration,
    ab_every: Duration,
    recall_every: Duration,
}

fn client_thread(
    store: Arc<ParamStore>,
    cfg: ClientCfg,
    counters: Arc<Counters>,
    stop: Arc<AtomicBool>,
) {
    let mut rng = Rng(cfg.seed | 1);
    let pool = build_pool(&store);
    let scratch = ParamStore::new();
    let osc = UdpSocket::bind("127.0.0.1:0").expect("osc client socket");
    osc.set_nonblocking(true).ok();
    osc.connect(cfg.osc).ok();
    let mut native = Native::connect(cfg.native, "soak-native-rd-a").ok();
    // a second connection only observes: it receives every change fan-out
    let _observer = Native::connect(cfg.native, "soak-native-rd-b").ok();

    // scenes: A = the loaded scene, B = A with a few hundred parameters moved and models swapped
    let b_text = {
        let sc = ParamStore::new();
        let (snap, _) = Snapshot::load(&sc, &cfg.scene_text).expect("scene A");
        for _ in 0..400 {
            let id = pool.params[rng.below(pool.params.len())];
            let _ = sc.set_norm(id, rng.unit(), 0);
        }
        for _ in 0..30 {
            let id = pool.models[rng.below(pool.models.len())];
            let _ = sc.set_norm(id, rng.unit(), 0);
        }
        snap.save(&sc)
    };
    let a_text = cfg.scene_text.clone();
    let mut on_b = false;

    let origin = 0x5A00_0001;
    let t0 = Instant::now();
    let (mut last_storm, mut last_swap, mut last_ab, mut last_recall, mut last_ctl) =
        (t0, t0, t0, t0, t0);
    let mut buf = vec![0u8; 2048];
    let mut storm_until: Option<Instant> = None;
    while !stop.load(Relaxed) {
        std::thread::sleep(Duration::from_millis(10));
        let now = Instant::now();
        while osc.recv(&mut buf).is_ok() {}
        // parameter traffic: a trickle (about 50/s each way), a storm (3000/s each) on schedule
        if now.duration_since(last_storm) >= cfg.storm_every {
            last_storm = now;
            storm_until = Some(now + cfg.storm_len);
            counters.storms.fetch_add(1, Relaxed);
        }
        let storming = storm_until.is_some_and(|u| now < u);
        let n = if storming {
            30
        } else {
            usize::from(rng.below(2) == 0)
        };
        for _ in 0..n {
            let id = pool.params[rng.below(pool.params.len())];
            let (txt, tok) = random_value(&scratch, id, &mut rng);
            let path = store.schema().path(id);
            if rng.below(2) == 0 {
                let m = Message::new(format!("/{path}"), vec![Arg::Str(txt)]);
                if osc.send(&m.to_vec()).is_ok() {
                    counters.osc.fetch_add(1, Relaxed);
                }
            } else if let (Some(nc), Some(tok)) = (native.as_mut(), tok) {
                nc.set(store.schema().hash(id), &tok);
                counters.native.fetch_add(1, Relaxed);
            }
        }
        if let Some(nc) = native.as_mut() {
            if !nc.out.is_empty() && !nc.flush() {
                counters.errors.fetch_add(1, Relaxed);
                native = Native::connect(cfg.native, "soak-native-rd-a").ok();
            }
        }
        if now.duration_since(last_swap) >= cfg.swap_every {
            last_swap = now;
            let id = pool.models[rng.below(pool.models.len())];
            let (txt, _) = random_value(&scratch, id, &mut rng);
            let path = store.schema().path(id);
            let m = Message::new(format!("/{path}"), vec![Arg::Str(txt)]);
            let _ = osc.send(&m.to_vec());
            counters.swaps.fetch_add(1, Relaxed);
        }
        if now.duration_since(last_ab) >= cfg.ab_every {
            last_ab = now;
            on_b = !on_b;
            let text = if on_b { &b_text } else { &a_text };
            let target = ParamStore::new();
            if let Ok((snap, _)) = Snapshot::load(&target, text) {
                snap.apply_diff(&target, &store, origin, &["$syscfg", "$ctl/lib"]);
                counters.ab.fetch_add(1, Relaxed);
            }
        }
        if now.duration_since(last_recall) >= cfg.recall_every {
            last_recall = now;
            if Snapshot::load(&store, &a_text).is_ok() {
                on_b = false;
                counters.recalls.fetch_add(1, Relaxed);
            }
        }
        if now.duration_since(last_ctl) >= Duration::from_secs(5) {
            last_ctl = now;
            match control::request(&cfg.ctl_socket, "status", Duration::from_secs(2)) {
                Ok(_) => counters.ctl.fetch_add(1, Relaxed),
                Err(_) => counters.errors.fetch_add(1, Relaxed),
            };
        }
    }
}

// ---- measurement ----------------------------------------------------------------------------------

struct Row {
    restarts: u64,
    t_s: f64,
    phase: usize,
    burst: usize,
    steady: bool,
    rss_mb: f64,
    cycles: u64,
    load_pct: f64,
    p50: u64,
    p99: u64,
    p999: u64,
    max: u64,
    slow: u64,
    xr_total: u64,
    xr_steady: u64,
    xr_trans: u64,
    b: StatsSnapshot,
    hw_ppm: f64,
    dev_late: u64,
    rt_allocs: u64,
    osc: u64,
    native: u64,
    recalls: u64,
    ab: u64,
    swaps: u64,
}

const CSV_HEADER: &str = "t_s,phase,burst,steady,rss_mb,cycles,load_pct,p50_us,p99_us,p999_us,max_us,slow,\
xruns_total,xruns_steady,xruns_trans,bridge_state,fill_ms,target_ms,corr_ppm,drift_ppm,hw_ppm,ratio,\
bridge_underruns,bridge_resyncs,bridge_overflow,dev_late_cb,rt_alloc_events,osc,native,recalls,ab,swaps";

impl Row {
    fn csv(&self) -> String {
        let b = &self.b;
        format!(
            "{:.1},{},{},{},{:.1},{},{:.2},{},{},{},{},{},{},{},{},{:?},{:.3},{:.3},{:.2},{:.2},{:.2},{:.8},{},{},{},{},{},{},{},{},{},{}",
            self.t_s, self.phase, self.burst, u8::from(self.steady), self.rss_mb, self.cycles,
            self.load_pct, self.p50, self.p99, self.p999, self.max, self.slow, self.xr_total,
            self.xr_steady, self.xr_trans, b.state, b.fill_frames / FS as f64 * 1e3,
            b.target_frames / FS as f64 * 1e3, b.correction_ppm, b.drift_ppm, self.hw_ppm, b.ratio,
            b.underruns, b.resyncs, b.overflow_frames, self.dev_late, self.rt_allocs, self.osc,
            self.native, self.recalls, self.ab, self.swaps
        )
    }
}

fn percentile_us(s: &CycleSnapshot, p: f64) -> u64 {
    s.percentile_ns(p) / 1000
}

/// Least-squares slope (y per x).
fn slope(points: &[(f64, f64)]) -> f64 {
    let n = points.len() as f64;
    if points.len() < 2 {
        return 0.0;
    }
    let (sx, sy) = points
        .iter()
        .fold((0.0, 0.0), |a, p| (a.0 + p.0, a.1 + p.1));
    let (mx, my) = (sx / n, sy / n);
    let (num, den) = points.iter().fold((0.0, 0.0), |a, p| {
        (a.0 + (p.0 - mx) * (p.1 - my), a.1 + (p.0 - mx).powi(2))
    });
    if den == 0.0 {
        0.0
    } else {
        num / den
    }
}

struct Args {
    minutes: f64,
    warmup_s: f64,
    shm_name: String,
    scene: String,
    out: PathBuf,
    interval_s: f64,
    phase_s: f64,
    settle_s: f64,
    bursts: Vec<usize>,
    drift_ppm: f64,
    drift_period_min: f64,
    device_buffer: usize,
    workers: Option<usize>,
    rt: bool,
    storm: bool,
    seed: u64,
    max_rss_mb_h: f64,
    max_fill_dev_ms: f64,
    verbose: bool,
}

const USAGE: &str = "usage: soak --minutes N --shm-name /wingsim.soak.<pid> [--scene fixtures/PUBLIC.snap] [--out DIR] \
[--warmup-secs 30] [--interval-secs 10] [--phase-secs 300] [--settle-secs 15] [--bursts 64,512] [--drift-ppm 200] \
[--drift-period-min 20] [--device-buffer 256] [--workers N] [--no-rt] [--no-storm] [--seed N] \
[--max-rss-growth-mb-h 4] [--max-fill-dev-ms 2] [--quiet]";

fn parse_args() -> Result<Args, String> {
    let mut a = Args {
        minutes: 1.0,
        warmup_s: 30.0,
        shm_name: format!("/wingsim.soak.{}", std::process::id()),
        scene: "fixtures/PUBLIC.snap".into(),
        out: PathBuf::new(),
        interval_s: 10.0,
        phase_s: 300.0,
        settle_s: 15.0,
        bursts: vec![64, 512],
        drift_ppm: 200.0,
        drift_period_min: 20.0,
        device_buffer: 256,
        workers: None,
        rt: true,
        storm: true,
        seed: 0x5EED_0071,
        max_rss_mb_h: 4.0,
        max_fill_dev_ms: 2.0,
        verbose: true,
    };
    let mut it = std::env::args().skip(1);
    while let Some(k) = it.next() {
        let mut v = |name: &str| it.next().ok_or(format!("{name}: missing value"));
        let num = |s: String, n: &str| s.parse::<f64>().map_err(|e| format!("{n}: {e}"));
        match k.as_str() {
            "--minutes" => a.minutes = num(v("--minutes")?, "--minutes")?,
            "--seconds" => a.minutes = num(v("--seconds")?, "--seconds")? / 60.0,
            "--warmup-secs" => a.warmup_s = num(v("--warmup-secs")?, "--warmup-secs")?,
            "--shm-name" => a.shm_name = v("--shm-name")?,
            "--scene" => a.scene = v("--scene")?,
            "--out" => a.out = v("--out")?.into(),
            "--interval-secs" => a.interval_s = num(v("--interval-secs")?, "--interval-secs")?,
            "--phase-secs" => a.phase_s = num(v("--phase-secs")?, "--phase-secs")?,
            "--settle-secs" => a.settle_s = num(v("--settle-secs")?, "--settle-secs")?,
            "--bursts" => {
                a.bursts = v("--bursts")?
                    .split(',')
                    .map(|x| {
                        x.trim()
                            .parse::<usize>()
                            .map_err(|e| format!("--bursts: {e}"))
                    })
                    .collect::<Result<_, _>>()?
            }
            "--drift-ppm" => a.drift_ppm = num(v("--drift-ppm")?, "--drift-ppm")?,
            "--drift-period-min" => {
                a.drift_period_min = num(v("--drift-period-min")?, "--drift-period-min")?
            }
            "--device-buffer" => {
                a.device_buffer = num(v("--device-buffer")?, "--device-buffer")? as usize
            }
            "--workers" => a.workers = Some(num(v("--workers")?, "--workers")? as usize),
            "--no-rt" => a.rt = false,
            "--no-storm" => a.storm = false,
            "--seed" => a.seed = num(v("--seed")?, "--seed")? as u64,
            "--max-rss-growth-mb-h" => {
                a.max_rss_mb_h = num(v("--max-rss-growth-mb-h")?, "--max-rss-growth-mb-h")?
            }
            "--max-fill-dev-ms" => {
                a.max_fill_dev_ms = num(v("--max-fill-dev-ms")?, "--max-fill-dev-ms")?
            }
            "--quiet" => a.verbose = false,
            _ => return Err(format!("unknown option {k}")),
        }
    }
    if a.shm_name == wing_shm::SHM_NAME {
        return Err(format!(
            "refusing to use the production shm {}: pick a private name such as /wingsim.soak.{}",
            wing_shm::SHM_NAME,
            std::process::id()
        ));
    }
    if !a.shm_name.starts_with('/') || a.shm_name.len() > 30 {
        return Err("--shm-name must start with '/' and be at most 30 characters".into());
    }
    if a.bursts.is_empty()
        || a.bursts
            .iter()
            .any(|b| *b == 0 || *b % 64 != 0 || *b > 2048)
    {
        return Err("--bursts: multiples of 64 up to 2048".into());
    }
    if a.out.as_os_str().is_empty() {
        let stamp = control::unix_now() as u64;
        a.out = PathBuf::from(format!("target/soak/{stamp}"));
    }
    Ok(a)
}

fn main() {
    let a = match parse_args() {
        Ok(a) => a,
        Err(e) => {
            eprintln!("{e}\n{USAGE}");
            std::process::exit(2);
        }
    };
    std::process::exit(run(a));
}

/// The counter must see an allocation on an audio-named thread and ignore other threads.
fn counting_self_test() {
    COUNTING.store(true, SeqCst);
    for (name, rt) in [("wing-worker-selftest", true), ("soak-selftest", false)] {
        std::thread::Builder::new()
            .name(name.into())
            .spawn(|| drop(std::hint::black_box(Box::new([0u8; 100]))))
            .unwrap()
            .join()
            .unwrap();
        let n = rt_alloc_events();
        // (a thread's own start-up allocations count too, hence "at least" the boxed array)
        assert!(
            if rt { n >= 2 } else { n == 0 },
            "allocation counter self-test failed on thread {name}: {n} events"
        );
        RT_ALLOCS.store(0, SeqCst);
        RT_DEALLOCS.store(0, SeqCst);
        RT_REALLOCS.store(0, SeqCst);
        RT_BYTES.store(0, SeqCst);
        OFF_N.store(0, SeqCst);
    }
    COUNTING.store(false, SeqCst);
}

fn run(a: Args) -> i32 {
    counting_self_test();
    std::fs::create_dir_all(&a.out).expect("create --out");
    let t_run = Instant::now();
    let log: LogFn = {
        let f = Arc::new(Mutex::new(
            std::fs::File::create(a.out.join("soak.log")).expect("soak.log"),
        ));
        Arc::new(move |m: &str| {
            let line = format!("[{:8.1}s] {m}", t_run.elapsed().as_secs_f64());
            eprintln!("{line}");
            let _ = writeln!(f.lock().unwrap(), "{line}");
        })
    };
    log(&format!(
        "soak: {} min (+{} s warm-up), shm {}, scene {}, out {}",
        a.minutes,
        a.warmup_s,
        a.shm_name,
        a.scene,
        a.out.display()
    ));

    // ---- store, servers (hooks first, then share) ----
    let scene_text = std::fs::read_to_string(&a.scene).unwrap_or_else(|e| {
        eprintln!("--scene {}: {e}", a.scene);
        std::process::exit(2)
    });
    let mut store = ParamStore::new();
    Snapshot::load(&store, &scene_text).expect("load scene");
    let hub = Hub::attach(&mut store);
    let osc_att = OscAttachment::attach(&mut store);
    let routing = audio::prepare(&mut store);
    let store = Arc::new(store);

    let native = NativeServer::bind(
        ServerConfig {
            bind: SocketAddrV4::new(Ipv4Addr::LOCALHOST, 0),
            idle_timeout: None,
            live: LiveInfo::default(),
            ..ServerConfig::default()
        },
        ServerShared {
            store: store.clone(),
            hub,
            meters: None,
        },
        Arc::new(|_: &wing_proto::server::ServerEvent| {}) as EventFn,
    )
    .expect("native bind")
    .spawn()
    .expect("native spawn");
    let osc = OscServer::bind(
        store.clone(),
        osc_att,
        OscConfig {
            bind: SocketAddrV4::new(Ipv4Addr::LOCALHOST, 0),
            ..OscConfig::default()
        },
    )
    .expect("osc bind")
    .spawn()
    .expect("osc spawn");
    log(&format!(
        "ephemeral ports: native tcp {}, osc udp {}",
        native.local_addr(),
        osc.local_addr()
    ));

    // meters + RTA as in the daemon
    let levels = LevelBank::shared();
    let rta_feed = RtaFeed::shared();
    let bank = Arc::new(MeterBank::for_schema());
    let rta = RtaService::spawn_default(store.clone(), Some(rta_feed.clone()), Some(bank.clone()))
        .expect("rta");
    let meters_stop = Arc::new(AtomicBool::new(false));
    let meters_thread = {
        let (l, s, b, stop) = (
            levels.clone(),
            store.clone(),
            bank.clone(),
            meters_stop.clone(),
        );
        std::thread::Builder::new()
            .name("soak-meters".into())
            .spawn(move || {
                let mut src = meter_source(Some(l), Some(s));
                while !stop.load(Relaxed) {
                    src(&b);
                    std::thread::sleep(wing_proto::server::meters::METER_INTERVAL);
                }
            })
            .expect("meters thread")
    };

    // ---- simulated driver segment + service ----
    let _ = Segment::unlink(&a.shm_name);
    let create = OpenOptions {
        create: true,
        ..Default::default()
    };
    let drv = DriverSim::new(Segment::open(&a.shm_name, &create).expect("create shm"), FS);
    let watch = Segment::open(&a.shm_name, &OpenOptions::default()).expect("open shm");

    let dev_shared = Arc::new(DeviceShared::default());
    let dev_cfg = Arc::new(DeviceCfg {
        buffer: a.device_buffer,
        amp_ppm: a.drift_ppm,
        period_s: a.drift_period_min * 60.0,
        rt: a.rt,
        t_start: Instant::now(),
    });
    let factory = {
        let (s, c) = (dev_shared.clone(), dev_cfg.clone());
        DeviceFactory(Arc::new(move |name: &str, cons: BridgeConsumer| {
            debug_assert_eq!(name, DEVICE_NAME);
            Box::new(start_sim_device(cons, s.clone(), c.clone())) as Box<dyn std::any::Any + Send>
        }))
    };
    let hostio = HostIoConfig {
        routes: vec![Route::parse(&format!("mon/1 -> \"{DEVICE_NAME}\" 1/2")).unwrap()],
        ..HostIoConfig::default()
    };
    let mut console = ConsoleConfig {
        block: 64,
        rt_workers: a.rt,
        meters: Some(levels.clone()),
        rta: Some(rta_feed.clone()),
        ..ConsoleConfig::default()
    };
    if let Some(w) = a.workers {
        console.workers = w;
    }
    let workers = console.workers;
    let svc = AudioService::spawn(
        store.clone(),
        routing,
        AudioOptions {
            shm_name: a.shm_name.clone(),
            console,
            rt: a.rt,
            hostio: Some(hostio),
            status_every: None,
            device_factory: Some(factory),
        },
        log.clone(),
    )
    .expect("spawn audio service");

    // control socket + status.json in a private state dir
    let state_dir = std::env::temp_dir().join(format!("wingsoak.{}", std::process::id()));
    std::fs::create_dir_all(&state_dir).expect("state dir");
    let (ctl_srv, ctl_rx) = control::ControlServer::spawn(&state_dir.join(control::SOCKET_FILE))
        .expect("control socket");
    let svc = Arc::new(svc);
    let ctl_stop = Arc::new(AtomicBool::new(false));
    let ctl_thread = {
        let (svc, stop, dir) = (svc.clone(), ctl_stop.clone(), state_dir.clone());
        let counters_native = native.local_addr();
        std::thread::Builder::new()
            .name("soak-control".into())
            .spawn(move || {
                let status = || control::DaemonStatus {
                    running: true,
                    pid: std::process::id(),
                    version: "soak".into(),
                    name: "SOAK".into(),
                    updated_unix: control::unix_now(),
                    state_dir: dir.display().to_string(),
                    hostio: format!("simulated device ({counters_native})"),
                    audio: Some(svc.report()),
                    ..Default::default()
                };
                let mut last = Instant::now() - Duration::from_secs(2);
                while !stop.load(Relaxed) {
                    if let Ok(req) = ctl_rx.recv_timeout(Duration::from_millis(100)) {
                        let r = match &req.cmd {
                            control::Command::Status => Ok(status().to_json()),
                            _ => Ok("ok".to_string()),
                        };
                        req.respond(r);
                    }
                    if last.elapsed() >= Duration::from_secs(1) {
                        last = Instant::now();
                        let _ = control::write_status(&dir.join(control::STATUS_FILE), &status());
                    }
                }
            })
            .expect("control thread")
    };

    // attach
    let t = Instant::now();
    while !(drv.daemon_alive() && svc.status_line().starts_with("audio: 48000 Hz")) {
        if t.elapsed() > Duration::from_secs(120) || STOP.load(Relaxed) {
            log("FAIL: the service did not attach to the simulated driver");
            return 1;
        }
        std::thread::sleep(Duration::from_millis(20));
    }
    log(&format!("attached: {}", svc.status_line()));
    unsafe {
        libc::signal(
            libc::SIGINT,
            on_signal as extern "C" fn(c_int) as libc::sighandler_t,
        );
        libc::signal(
            libc::SIGTERM,
            on_signal as extern "C" fn(c_int) as libc::sighandler_t,
        );
    }

    // ---- start the simulated HAL and the clients ----
    let hal_sh = Arc::new(HalShared::default());
    let hal = {
        let sh = hal_sh.clone();
        let cfg = HalCfg {
            bursts: a.bursts.clone(),
            phase: Duration::from_secs_f64(a.phase_s),
            settle: Duration::from_secs_f64(a.settle_s),
            rt: a.rt,
            t_run,
        };
        std::thread::Builder::new()
            .name("soak-hal".into())
            .spawn(move || hal_thread(drv, sh, cfg))
            .expect("hal thread")
    };
    let counters = Arc::new(Counters::default());
    let client_stop = Arc::new(AtomicBool::new(false));
    let client = a.storm.then(|| {
        let cfg = ClientCfg {
            osc: osc.local_addr(),
            native: native.local_addr(),
            ctl_socket: state_dir.join(control::SOCKET_FILE),
            scene_text: scene_text.clone(),
            seed: a.seed,
            storm_every: Duration::from_secs(45),
            storm_len: Duration::from_secs(3),
            swap_every: Duration::from_secs(10),
            ab_every: Duration::from_secs(30),
            recall_every: Duration::from_secs(300),
        };
        let (s, c, st) = (store.clone(), counters.clone(), client_stop.clone());
        std::thread::Builder::new()
            .name("soak-client".into())
            .spawn(move || client_thread(s, cfg, c, st))
            .expect("client thread")
    });

    // ---- sampling loop ----
    let mut csv = std::fs::File::create(a.out.join("soak.csv")).expect("soak.csv");
    writeln!(csv, "{CSV_HEADER}").unwrap();
    let hdr = watch.header();
    let xruns = || {
        hdr.from_daemon.underruns.load(Relaxed) as u64 + hdr.to_daemon.overruns.load(Relaxed) as u64
    };
    let stats = svc.stats().clone();
    let total = Duration::from_secs_f64(a.warmup_s + a.minutes * 60.0);
    let t_measure = t_run + Duration::from_secs_f64(a.warmup_s);
    let mut rows: Vec<Row> = Vec::new();
    let mut base_cycles: Option<CycleSnapshot> = None;
    let mut base_xr = 0u64;
    let mut last_snap = stats.snapshot();
    let mut next = Instant::now() + Duration::from_secs_f64(a.interval_s);
    let mut detached_samples = 0u64;
    let mut peak_load = 0.0f64;
    let mut off_sample = 0usize;
    log(&format!(
        "running: {workers} workers, rt {}, bursts {:?} every {} s, device buffer {} frames, hw drift +-{} ppm / {} min",
        a.rt, a.bursts, a.phase_s, a.device_buffer, a.drift_ppm, a.drift_period_min
    ));
    while t_run.elapsed() < total && !STOP.load(SeqCst) {
        std::thread::sleep(Duration::from_millis(50));
        if base_cycles.is_none() && Instant::now() >= t_measure {
            // end of warm-up: from here on RT allocations count and the statistics start
            base_cycles = Some(stats.snapshot());
            base_xr = xruns();
            hal_sh.steady_xruns.store(0, Relaxed);
            hal_sh.trans_xruns.store(0, Relaxed);
            hal_sh.loopback_cycles.store(0, Relaxed);
            hal_sh.late_reads.store(0, Relaxed);
            stats.reset_peak();
            last_snap = stats.snapshot();
            COUNTING.store(true, SeqCst);
            log("warm-up over: measuring (RT allocation counting on)");
        }
        if Instant::now() < next {
            continue;
        }
        next += Duration::from_secs_f64(a.interval_s);
        let snap = stats.snapshot();
        let d = snap.since(&last_snap);
        let max_ns = snap.max_ns;
        stats.reset_peak();
        last_snap = snap;
        let rep = svc.report();
        if !rep.attached {
            detached_samples += 1;
        }
        let b = dev_shared
            .stats
            .lock()
            .unwrap()
            .as_ref()
            .map(|s| s.snapshot())
            .unwrap_or_else(|| StatsSnapshot {
                state: BridgeState::Stopped,
                ..empty_bridge()
            });
        let now_ms = t_run.elapsed().as_millis() as u64;
        let phase = hal_sh.phase.load(Relaxed);
        let restarts = hal_sh.restarts.load(Relaxed);
        let prev_restarts = rows.last().map(|r| r.restarts);
        let prev_phase = rows.last().map(|r| r.phase);
        let prev_t_ms = rows.last().map_or(0, |r| (r.t_s * 1000.0) as u64);
        let steady = base_cycles.is_some()
            && prev_phase == Some(phase)
            && prev_restarts == Some(restarts)
            && prev_t_ms >= hal_sh.settled_ms.load(Relaxed);
        let row = Row {
            restarts,
            t_s: t_run.elapsed().as_secs_f64(),
            phase,
            burst: hal_sh.burst.load(Relaxed),
            steady,
            rss_mb: rss_mb(),
            cycles: d.cycles,
            load_pct: d.load_pct(),
            p50: percentile_us(&d, 0.5),
            p99: percentile_us(&d, 0.99),
            p999: percentile_us(&d, 0.999),
            max: max_ns / 1000,
            slow: d.slow,
            xr_total: xruns() - base_xr.min(xruns()),
            xr_steady: hal_sh.steady_xruns.load(Relaxed),
            xr_trans: hal_sh.trans_xruns.load(Relaxed),
            b,
            hw_ppm: f64_bits(&dev_shared.hw_ppm),
            dev_late: dev_shared.late.load(Relaxed),
            rt_allocs: rt_alloc_events(),
            osc: counters.osc.load(Relaxed),
            native: counters.native.load(Relaxed),
            recalls: counters.recalls.load(Relaxed),
            ab: counters.ab.load(Relaxed),
            swaps: counters.swaps.load(Relaxed),
        };
        let _ = now_ms;
        peak_load = peak_load.max(row.load_pct);
        writeln!(csv, "{}", row.csv()).unwrap();
        let _ = csv.flush();
        if a.verbose {
            log(&format!(
                "t {:6.0}s ph{} {:3}fr{} rss {:6.1} MB load {:4.1}% p50 {:3} p99 {:3} p99.9 {:3} max {:4} us slow {} | xr {} (steady {}, trans {}) | bridge {:?} fill {:.2}/{:.2} ms corr {:+.0} (hw {:+.0}) ppm uf {} rs {} | rtalloc {} | osc {} nat {} rec {} ab {} mdl {}",
                row.t_s, row.phase, row.burst, if row.steady { " " } else { "*" }, row.rss_mb, row.load_pct, row.p50, row.p99,
                row.p999, row.max, row.slow, row.xr_total, row.xr_steady, row.xr_trans, row.b.state,
                row.b.fill_frames / FS as f64 * 1e3, row.b.target_frames / FS as f64 * 1e3,
                row.b.correction_ppm, row.hw_ppm, row.b.underruns, row.b.resyncs, row.rt_allocs,
                row.osc, row.native, row.recalls, row.ab, row.swaps
            ));
        }
        // keep a handful of full status lines in the log
        if off_sample.is_multiple_of(30) {
            log(&svc.status_line());
        }
        off_sample += 1;
        rows.push(row);
    }
    COUNTING.store(false, SeqCst);
    let interrupted = STOP.load(SeqCst);

    // ---- shut down ----
    let end_snap = stats.snapshot();
    let end_late_reads = hal_sh.late_reads.load(Relaxed);
    let end_loopback = hal_sh.loopback_cycles.load(Relaxed);
    let (hal_cycles, steady_xr, trans_xr) = (
        hal_sh.cycles.load(Relaxed),
        hal_sh.steady_xruns.load(Relaxed),
        hal_sh.trans_xruns.load(Relaxed),
    );
    client_stop.store(true, Relaxed);
    if let Some(c) = client {
        let _ = c.join();
    }
    hal_sh.stop.store(true, Relaxed);
    let drv = hal.join().expect("hal thread");
    let detached_alive = {
        // the control thread holds a clone: stop it first
        ctl_stop.store(true, Relaxed);
        let _ = ctl_thread.join();
        let svc = Arc::try_unwrap(svc).ok().expect("service still shared");
        svc.stop();
        drv.daemon_alive()
    };
    meters_stop.store(true, Relaxed);
    let _ = meters_thread.join();
    rta.stop();
    osc.stop();
    native.stop();
    ctl_srv.stop();
    let _ = Segment::unlink(&a.shm_name);
    let _ = std::fs::remove_dir_all(&state_dir);

    summarize(
        &a,
        &rows,
        base_cycles.as_ref(),
        &end_snap,
        Summary {
            hal_cycles,
            steady_xr,
            trans_xr,
            late_reads: end_late_reads,
            loopback: end_loopback,
            detached_samples,
            detached_alive,
            interrupted,
            peak_load,
            dev_cb: dev_shared.callbacks.load(Relaxed),
            dev_late: dev_shared.late.load(Relaxed),
            dev_max_late_us: dev_shared.max_late_us.load(Relaxed),
            counters: &counters,
            workers,
            run_s: t_run.elapsed().as_secs_f64(),
        },
        &log,
    )
}

fn empty_bridge() -> StatsSnapshot {
    StatsSnapshot {
        state: BridgeState::Stopped,
        fill_frames: 0.0,
        ring_frames: 0.0,
        target_frames: 0.0,
        correction_ppm: 0.0,
        drift_ppm: 0.0,
        ratio: 1.0,
        underruns: 0,
        resyncs: 0,
        callbacks: 0,
        frames_out: 0,
        device_rate: 0.0,
        device_buffer: 0,
        reconnects: 0,
        overflow_frames: 0,
        burst_frames: 0,
        resampler_frames: 0.0,
        output_latency_s: 0.0,
    }
}

struct Summary<'a> {
    hal_cycles: u64,
    steady_xr: u64,
    trans_xr: u64,
    late_reads: u64,
    loopback: u64,
    detached_samples: u64,
    detached_alive: bool,
    interrupted: bool,
    peak_load: f64,
    dev_cb: u64,
    dev_late: u64,
    dev_max_late_us: u64,
    counters: &'a Counters,
    workers: usize,
    run_s: f64,
}

fn summarize(
    a: &Args,
    rows: &[Row],
    base: Option<&CycleSnapshot>,
    end: &CycleSnapshot,
    s: Summary,
    log: &LogFn,
) -> i32 {
    let mut lines: Vec<String> = Vec::new();
    let mut fails = 0;
    let mut warns = 0;
    let mut check = |lines: &mut Vec<String>, ok: Option<bool>, what: &str, detail: String| {
        let tag = match ok {
            Some(true) => "PASS",
            Some(false) => {
                fails += 1;
                "FAIL"
            }
            None => {
                warns += 1;
                "WARN"
            }
        };
        lines.push(format!("{tag}  {what}: {detail}"));
    };
    let measured = base.is_some();
    let all = base.map_or_else(|| end.clone(), |b| end.since(b));
    let period_us = all.period_ns as f64 / 1000.0;
    let steady_rows: Vec<&Row> = rows.iter().filter(|r| r.steady).collect();
    let rss_pts: Vec<(f64, f64)> = rows
        .iter()
        .filter(|r| r.t_s >= a.warmup_s + 120.0)
        .map(|r| (r.t_s / 3600.0, r.rss_mb))
        .collect();
    let rss_span_min = rss_pts
        .last()
        .zip(rss_pts.first())
        .map_or(0.0, |(l, f)| (l.0 - f.0) * 60.0);
    let rss_growth = slope(&rss_pts);
    let rss_first = rows.first().map_or(0.0, |r| r.rss_mb);
    let rss_last = rows.last().map_or(0.0, |r| r.rss_mb);
    let rss_max = rows.iter().map(|r| r.rss_mb).fold(0.0, f64::max);

    lines.push(format!(
        "run: {:.1} min wall ({:.1} min measured), {} engine cycles measured, {} HAL cycles, {} workers, rt {}, bursts {:?}, phase {} s{}",
        s.run_s / 60.0,
        (s.run_s - a.warmup_s).max(0.0) / 60.0,
        all.cycles,
        s.hal_cycles,
        s.workers,
        a.rt,
        a.bursts,
        a.phase_s,
        if s.interrupted { ", INTERRUPTED" } else { "" }
    ));
    lines.push(format!(
        "cycle time over the whole run: p50 {} us, p99 {} us, p99.9 {} us, max {:.0} us (peak of any interval), mean load {:.1}% of the {:.0} us period (worst interval {:.1}%), slow cycles (> period): {}",
        percentile_us(&all, 0.5),
        percentile_us(&all, 0.99),
        percentile_us(&all, 0.999),
        rows.iter().map(|r| r.max).max().unwrap_or(0),
        all.load_pct(),
        period_us,
        s.peak_load,
        all.slow
    ));
    lines.push(format!(
        "client load: {} OSC sets, {} native sets, {} storms, {} model swaps, {} A/B toggles, {} scene recalls, {} control-socket polls, {} client errors",
        s.counters.osc.load(Relaxed),
        s.counters.native.load(Relaxed),
        s.counters.storms.load(Relaxed),
        s.counters.swaps.load(Relaxed),
        s.counters.ab.load(Relaxed),
        s.counters.recalls.load(Relaxed),
        s.counters.ctl.load(Relaxed),
        s.counters.errors.load(Relaxed)
    ));
    lines.push(format!(
        "driver reads that went ahead of the daemon (daemon not done by 90% of the period): {}",
        s.late_reads
    ));
    lines.push(format!(
        "simulated host device: {} callbacks, {} started > 1 ms late (max {} us late: harness scheduling lag)",
        s.dev_cb, s.dev_late, s.dev_max_late_us
    ));
    lines.push(format!(
        "memory: RSS {rss_first:.1} MB at first sample, {rss_last:.1} MB at the end, max {rss_max:.1} MB; trend {}",
        if rss_span_min >= 20.0 {
            format!("{rss_growth:+.2} MB/h (least squares over the last {rss_span_min:.0} min)")
        } else {
            "not evaluated (run too short)".to_string()
        }
    ));
    lines.push(String::new());

    check(
        &mut lines,
        Some(measured),
        "ran past the warm-up",
        format!("{} samples", rows.len()),
    );
    check(
        &mut lines,
        Some(s.steady_xr == 0),
        "driver xruns (underruns + overruns) in settled periods",
        format!(
            "{} (during the {} s after each IO restart: {})",
            s.steady_xr, a.settle_s, s.trans_xr
        ),
    );
    check(
        &mut lines,
        Some(s.loopback == 0 && s.detached_samples == 0),
        "daemon stayed attached (no loopback fallback, no detached status)",
        format!(
            "{} loopback cycles, {} detached samples",
            s.loopback, s.detached_samples
        ),
    );
    check(
        &mut lines,
        Some(!s.detached_alive),
        "clean detach at shutdown (driver back in loopback)",
        format!(
            "daemon heartbeat {}",
            if s.detached_alive {
                "still fresh"
            } else {
                "cleared"
            }
        ),
    );
    let rt_events = RT_ALLOCS.load(Relaxed) + RT_DEALLOCS.load(Relaxed) + RT_REALLOCS.load(Relaxed);
    check(
        &mut lines,
        Some(rt_events == 0),
        "allocations on RT threads (wing-audio-rt, wing-worker-*)",
        format!(
            "{} allocs, {} deallocs, {} reallocs, {} bytes",
            RT_ALLOCS.load(Relaxed),
            RT_DEALLOCS.load(Relaxed),
            RT_REALLOCS.load(Relaxed),
            RT_BYTES.load(Relaxed)
        ),
    );
    let slow_pct = if all.cycles > 0 {
        all.slow as f64 * 100.0 / all.cycles as f64
    } else {
        0.0
    };
    check(
        &mut lines,
        (all.slow == 0).then_some(true),
        "cycles slower than one engine period",
        format!(
            "{} ({slow_pct:.5}% of cycles; harness-induced scheduling or machine load possible)",
            all.slow
        ),
    );
    check(
        &mut lines,
        Some(percentile_us(&all, 0.999) as f64 <= period_us * 0.5),
        "cycle p99.9 within the T070 budget (50% of the period)",
        format!(
            "{} us vs {:.0} us",
            percentile_us(&all, 0.999),
            period_us * 0.5
        ),
    );
    let b_uf: u64 = delta_sum(rows, |r| r.b.underruns, true);
    let b_rs: u64 = delta_sum(rows, |r| r.b.resyncs, true);
    let b_uf_t: u64 = delta_sum(rows, |r| r.b.underruns, false);
    let b_rs_t: u64 = delta_sum(rows, |r| r.b.resyncs, false);
    check(
        &mut lines,
        Some(b_uf == 0 && b_rs == 0),
        "host bridge underruns / resyncs in settled periods",
        format!("{b_uf} underruns, {b_rs} resyncs (in transition periods: {b_uf_t} underruns, {b_rs_t} resyncs)"),
    );
    let max_fill_dev = steady_rows
        .iter()
        .map(|r| (r.b.fill_frames - r.b.target_frames).abs() / FS as f64 * 1e3)
        .fold(0.0, f64::max);
    let max_overflow = rows.last().map_or(0, |r| r.b.overflow_frames);
    check(
        &mut lines,
        Some(max_fill_dev <= a.max_fill_dev_ms),
        "bridge fill within target in settled periods",
        format!("max |fill - target| {max_fill_dev:.2} ms (limit {} ms), overflow frames dropped {max_overflow}", a.max_fill_dev_ms),
    );
    let max_track = steady_rows
        .iter()
        .map(|r| (r.b.drift_ppm + r.hw_ppm).abs())
        .fold(0.0, f64::max);
    let max_corr = steady_rows
        .iter()
        .map(|r| r.b.correction_ppm.abs())
        .fold(0.0, f64::max);
    lines.push(format!(
        "INFO  bridge drift tracking: max |controller drift estimate + true device error| {max_track:.0} ppm in settled periods (sign convention: estimate = -device error), max |ratio correction| {max_corr:.0} ppm"
    ));
    if rss_span_min >= 20.0 {
        check(
            &mut lines,
            Some(rss_growth <= a.max_rss_mb_h),
            "RSS growth",
            format!("{rss_growth:+.2} MB/h (limit {} MB/h)", a.max_rss_mb_h),
        );
    } else {
        lines.push("INFO  RSS growth not evaluated: fewer than 20 minutes of samples after the first 2 minutes".into());
    }
    if s.interrupted {
        check(
            &mut lines,
            None,
            "run completed",
            "interrupted by a signal; results cover the time run".into(),
        );
    }
    let off = offender_report();
    if !off.is_empty() {
        lines.push(String::new());
        lines.push("first RT allocation call stacks:".into());
        lines.extend(off);
    }
    lines.push(String::new());
    lines.push(format!(
        "RESULT: {}{}",
        if fails == 0 { "PASS" } else { "FAIL" },
        if warns > 0 {
            format!(" ({warns} warning(s))")
        } else {
            String::new()
        }
    ));
    let text = lines.join("\n");
    log("---- soak summary ----");
    println!("{text}");
    let _ = std::fs::write(a.out.join("summary.txt"), &text);
    let json = serde_json::json!({
        "pass": fails == 0,
        "warnings": warns,
        "minutes_measured": (s.run_s - a.warmup_s).max(0.0) / 60.0,
        "cycles": all.cycles,
        "p50_us": percentile_us(&all, 0.5),
        "p99_us": percentile_us(&all, 0.99),
        "p999_us": percentile_us(&all, 0.999),
        "slow_cycles": all.slow,
        "load_pct": all.load_pct(),
        "xruns_steady": s.steady_xr,
        "xruns_transition": s.trans_xr,
        "rt_alloc_events": rt_events,
        "rss_growth_mb_per_h": rss_growth,
        "rss_max_mb": rss_max,
        "bridge_underruns_steady": b_uf,
        "bridge_resyncs_steady": b_rs,
        "max_fill_dev_ms": max_fill_dev,
    });
    let _ = std::fs::write(a.out.join("summary.json"), json.to_string());
    log(&format!("wrote {}", a.out.display()));
    i32::from(fails != 0)
}

/// Sum of the increases of a cumulative counter over settled (`steady`) or other intervals.
fn delta_sum(rows: &[Row], f: impl Fn(&Row) -> u64, steady: bool) -> u64 {
    rows.windows(2)
        .filter(|w| w[1].steady == steady && w[0].t_s > 0.0)
        .map(|w| f(&w[1]).saturating_sub(f(&w[0])))
        .sum()
}
