//! The daemon's audio service: attaches to the HAL driver's shared memory and runs the console
//! on a real-time driver thread.
//!
//! Threads:
//! * **`wing-audio`** (control, normal priority): attach/reattach, routing-table refresh,
//!   console (re)builds on sample-rate changes, host outputs, telemetry drain, status.
//! * **`wing-audio-rt`** (one per session): time-constraint policy sized to the driver's IO
//!   buffer period, member of the daemon's audio work interval (the engine workers join it too).
//!   Loop: heartbeat → `DaemonLink::wait` (os_sync wake from the driver's cycle) → for every
//!   complete engine block the driver has written: USB In 1..48 → [`Console::cycle`] →
//!   USB Out 1..48 → commit. No allocation, locks, logging or syscalls besides the wait and the
//!   work-interval calls.
//!
//! Lifecycle:
//! * **No driver** (no shm segment): retry every second, control-only meanwhile.
//! * **Incompatible segment** (e.g. the v1 driver is still installed while this daemon speaks
//!   v2): logged once with the found version; the daemon never replaces a driver's segment and
//!   keeps running control-only, retrying (a reinstalled driver recreates the segment).
//! * **Driver restarted / segment recreated** (`Segment::is_current` false, checked every
//!   second on the control thread): the session ends and the daemon reattaches. A coreaudiod
//!   restart that keeps the segment is a new IO epoch, handled inside `DaemonLink`.
//! * **Sample-rate change** (header `sample_rate` differs from the console's): the RT thread
//!   hands the link back, the console and host bridges are rebuilt at the new rate (the
//!   parameter pipeline is kept) and the session resumes on the same link.
//! * **Shutdown**: the session stops and the link is dropped, which zeroes the heartbeat so the
//!   driver falls back to loopback immediately.

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering::Relaxed};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use wing_engine::params::ParamPipeline;
use wing_engine::routing::RoutingControl;
use wing_engine::rt::{set_current_thread_time_constraint, TimeConstraint};
use wing_engine::WorkgroupMembership;
use wing_hostio::HostIoConfig;
use wing_model::ParamStore;
use wing_shm::{DaemonLink, Error as ShmError, LayoutError, OpenOptions, Segment, WakeMode};

use super::console::{Console, ConsoleConfig};
use super::hostio::{self, HostOutputs};
use super::stats::{AudioStats, Snapshot};
use super::workgroup::WorkInterval;
use super::LogFn;

/// How long the RT thread blocks waiting for a driver cycle before re-checking its flags.
pub const WAIT_TIMEOUT: Duration = Duration::from_millis(10);
/// Attach retry interval while no (compatible) driver is present.
pub const RETRY: Duration = Duration::from_secs(1);
/// Segment-currency check interval while attached.
pub const CURRENCY_CHECK: Duration = Duration::from_secs(1);

/// Audio service options.
#[derive(Debug, Clone)]
pub struct AudioOptions {
    /// shm name (`wing_shm::SHM_NAME`).
    pub shm_name: String,
    /// Engine block, workers, RT workers (the rate comes from the driver).
    pub console: ConsoleConfig,
    /// Time-constraint policy + work interval on the driver thread.
    pub rt: bool,
    /// Host outputs (resolved).
    pub hostio: Option<HostIoConfig>,
    /// Log a status line this often while attached (`None`: never).
    pub status_every: Option<Duration>,
}

impl Default for AudioOptions {
    fn default() -> Self {
        Self {
            shm_name: wing_shm::SHM_NAME.to_string(),
            console: ConsoleConfig::default(),
            rt: true,
            hostio: None,
            status_every: Some(Duration::from_secs(10)),
        }
    }
}

/// Process every complete engine block the driver has made available: USB In → console →
/// USB Out → commit. Partial blocks wait for the next driver cycle. Returns frames processed.
/// RT-safe.
#[inline]
pub fn service(link: &mut DaemonLink, console: &mut Console) -> usize {
    let block = console.block();
    let mut done = 0;
    while let Some(b) = link.next_block(block) {
        if b.frames < block {
            break;
        }
        let l = &*link;
        console.cycle(
            |ch, buf| l.read_input(&b, ch, buf),
            |ch, buf| l.write_output(&b, ch, buf),
        );
        link.commit(&b);
        done += b.frames;
    }
    done
}

/// Why an RT session ended.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Exit {
    /// Asked to stop.
    Stopped,
    /// The driver switched to this rate.
    RateChanged(u32),
}

/// Flags/counters shared with the RT thread.
#[derive(Default)]
pub struct RtShared {
    pub stop: AtomicBool,
    /// 0 = not attempted, 1 = ok, 2 + kern_return = failed.
    pub policy: AtomicU64,
    /// Driver buffer (frames) the policy is sized for.
    pub policy_frames: AtomicU64,
    pub joined_workgroup: AtomicBool,
    /// Frames processed.
    pub frames: AtomicU64,
}

fn rate_ok(r: u32) -> bool {
    (8_000..=384_000).contains(&r)
}

/// The RT driver loop (runs on its own thread; see the module docs).
pub fn rt_loop(
    mut link: DaemonLink,
    mut console: Console,
    shared: Arc<RtShared>,
    rt: bool,
    wi: Option<Arc<WorkInterval>>,
) -> (DaemonLink, Console, Exit) {
    let fs = console.sample_rate();
    let block = console.block() as u64;
    let membership = wi
        .as_ref()
        .and_then(|w| WorkgroupMembership::join(w.raw()).ok());
    shared.joined_workgroup.store(membership.is_some(), Relaxed);
    let mut policy_frames = 0u64;
    let exit = loop {
        if shared.stop.load(Relaxed) {
            break Exit::Stopped;
        }
        link.heartbeat();
        link.wait(WAIT_TIMEOUT);
        let h = link.header();
        let rate = h.sample_rate.load(Relaxed);
        if rate_ok(rate) && rate != fs {
            break Exit::RateChanged(rate);
        }
        let buf = (h.input_frames.load(Relaxed) as u64).max(block);
        let period_ns = buf * 1_000_000_000 / fs as u64;
        if rt && buf != policy_frames {
            // One syscall per driver buffer-size change (like CoreAudio's own IO threads).
            policy_frames = buf;
            let st = match set_current_thread_time_constraint(TimeConstraint::for_period(period_ns))
            {
                Ok(()) => 1,
                Err(wing_engine::RtError::Policy(k)) => 2 + k as u32 as u64,
                Err(_) => 2,
            };
            shared.policy.store(st, Relaxed);
            shared.policy_frames.store(buf, Relaxed);
        }
        let interval = membership.is_some()
            && wi.as_ref().is_some_and(|w| {
                let t0 = wing_shm::clock::now_ticks();
                w.start(t0, t0 + wing_shm::clock::ns_to_ticks(period_ns))
            });
        let n = service(&mut link, &mut console);
        if interval {
            if let Some(w) = &wi {
                w.finish();
            }
        }
        if n > 0 {
            shared.frames.fetch_add(n as u64, Relaxed);
        }
    };
    drop(membership);
    (link, console, exit)
}

struct Session {
    shared: Arc<RtShared>,
    handle: JoinHandle<(DaemonLink, Console, Exit)>,
    /// Our own mapping of the segment for currency checks and status (control thread only).
    watch: Segment,
    hosts: HostOutputs,
    rate: u32,
    base_underruns: u32,
    base_overruns: u32,
    last_check: Instant,
}

/// Handle on the running audio service.
pub struct AudioService {
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
    status: Arc<Mutex<String>>,
    stats: Arc<AudioStats>,
}

impl AudioService {
    /// Start the control thread. `routing` must already be hooked to the store (see
    /// [`super::prepare`]).
    pub fn spawn(
        store: Arc<ParamStore>,
        routing: RoutingControl,
        opts: AudioOptions,
        log: LogFn,
    ) -> std::io::Result<Self> {
        let stop = Arc::new(AtomicBool::new(false));
        let status = Arc::new(Mutex::new("audio: starting".to_string()));
        let stats = Arc::new(AudioStats::default());
        let ctl = Control {
            store,
            routing,
            opts,
            log,
            stop: stop.clone(),
            status: status.clone(),
            stats: stats.clone(),
            pipeline: None,
            last_msg: String::new(),
            wi: None,
            last_snap: Snapshot::default(),
            last_status: Instant::now(),
        };
        let thread = std::thread::Builder::new()
            .name("wing-audio".into())
            .spawn(move || ctl.run())?;
        Ok(Self {
            stop,
            thread: Some(thread),
            status,
            stats,
        })
    }

    /// The latest status line.
    pub fn status_line(&self) -> String {
        match self.status.lock() {
            Ok(s) => s.clone(),
            Err(e) => e.into_inner().clone(),
        }
    }

    /// Cycle statistics.
    pub fn stats(&self) -> &Arc<AudioStats> {
        &self.stats
    }

    /// Stop: end the session and detach (the driver falls back to loopback).
    pub fn stop(mut self) {
        self.shutdown();
    }

    fn shutdown(&mut self) {
        self.stop.store(true, Relaxed);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

impl Drop for AudioService {
    fn drop(&mut self) {
        self.shutdown();
    }
}

/// Human-readable reason an attach failed.
pub fn describe_open_error(name: &str, e: &ShmError) -> String {
    match e {
        ShmError::NotFound => format!(
            "audio: no driver (shm {name} not found); running control-only, retrying every {}s",
            RETRY.as_secs()
        ),
        ShmError::Incompatible(LayoutError::Version(v)) => format!(
            "audio: driver shm {name} has layout v{v}, this daemon needs v{}: the installed Wing \
             Simulator driver is outdated (reinstall: driver/build.sh && sudo driver/install.sh). \
             Running control-only; the driver stays in loopback",
            wing_shm::VERSION
        ),
        ShmError::Incompatible(l) => format!(
            "audio: driver shm {name} is incompatible ({l:?}; probably another driver version, \
             this daemon needs layout v{}). Running control-only; the driver stays in loopback",
            wing_shm::VERSION
        ),
        e => format!("audio: attaching shm {name} failed: {e}; retrying"),
    }
}

struct Control {
    store: Arc<ParamStore>,
    routing: RoutingControl,
    opts: AudioOptions,
    log: LogFn,
    stop: Arc<AtomicBool>,
    status: Arc<Mutex<String>>,
    stats: Arc<AudioStats>,
    pipeline: Option<ParamPipeline>,
    last_msg: String,
    wi: Option<Arc<WorkInterval>>,
    last_snap: Snapshot,
    last_status: Instant,
}

impl Control {
    /// Log `msg` if it differs from the last one, and make it the status.
    fn say(&mut self, msg: String) {
        if msg != self.last_msg {
            (self.log)(&msg);
            self.last_msg = msg.clone();
        }
        self.set_status(msg);
    }

    fn set_status(&self, s: String) {
        if let Ok(mut st) = self.status.lock() {
            *st = s;
        }
    }

    fn run(mut self) {
        if self.opts.rt {
            self.wi = WorkInterval::create("wing-sim audio").map(Arc::new);
        }
        let mut telem = None;
        let mut session: Option<Session> = None;
        let mut next_try = Instant::now();
        while !self.stop.load(Relaxed) {
            self.routing.refresh(&self.store);
            if telem.is_none() {
                telem = self.pipeline.as_ref().and_then(|p| p.telemetry_rx());
            }
            // TODO(meters): telemetry is drained and dropped until metering consumes it.
            if let Some(rx) = telem.as_mut() {
                while rx.pop().is_some() {}
            }
            session = match session {
                None if Instant::now() >= next_try => {
                    next_try = Instant::now() + RETRY;
                    self.try_attach()
                }
                None => None,
                Some(s) => {
                    let s = self.supervise(s);
                    if s.is_none() {
                        next_try = Instant::now();
                    }
                    s
                }
            };
            std::thread::sleep(Duration::from_millis(10));
        }
        if let Some(s) = session {
            self.end_session(s, "shutdown");
        }
        self.pipeline = None;
        self.wi = None;
    }

    fn try_attach(&mut self) -> Option<Session> {
        let name = self.opts.shm_name.clone();
        let opened = Segment::open(&name, &OpenOptions::default()).and_then(|watch| {
            DaemonLink::attach(&name, &OpenOptions::default(), WakeMode::Auto)
                .map(|link| (watch, link))
        });
        let (watch, link) = match opened {
            Ok(x) => x,
            Err(e) => {
                let m = describe_open_error(&name, &e);
                self.say(m);
                return None;
            }
        };
        let rate = match link.header().sample_rate.load(Relaxed) {
            r if rate_ok(r) => r,
            _ => 48_000,
        };
        let st = watch.status();
        self.say(format!(
            "audio: attached shm {name} (v{}, segment {:x}, driver pid {}, {rate} Hz, input delay \
             {} frames, wake {:?})",
            st.version,
            st.segment_id,
            st.driver_pid,
            st.input_delay_frames,
            link.mechanism()
        ));
        match self.start_session(link, watch, rate) {
            Ok(s) => Some(s),
            Err(e) => {
                self.say(format!("audio: starting the engine failed: {e}"));
                None
            }
        }
    }

    fn burst(&self, link: &DaemonLink) -> usize {
        (link.header().input_frames.load(Relaxed) as usize)
            .max(512)
            .max(self.opts.console.block)
    }

    fn build_console(&mut self, rate: u32, burst: usize) -> Result<(Console, HostOutputs), String> {
        let t0 = Instant::now();
        let mut cfg = self.opts.console.clone();
        cfg.sample_rate = rate;
        cfg.workgroup = self.wi.as_ref().map(|w| w.raw());
        let (hosts, feeds, errs) = match &self.opts.hostio {
            Some(h) => HostOutputs::start(hostio::plan(h, rate, burst)?, h.buffer_frames),
            None => (HostOutputs::none(), Vec::new(), Vec::new()),
        };
        for e in errs {
            (self.log)(&format!("audio: {e}"));
        }
        let console = Console::build(
            cfg.clone(),
            &self.store,
            &mut self.routing,
            self.pipeline.take(),
            feeds,
            self.stats.clone(),
        )?;
        (self.log)(&format!(
            "audio: console built in {:.0} ms: {rate} Hz, block {}, {} workers{}, {} host \
             bridge(s)",
            t0.elapsed().as_secs_f64() * 1e3,
            cfg.block,
            console.engine().num_workers(),
            if cfg.workgroup.is_some() {
                " in the work interval"
            } else {
                ""
            },
            hosts.outputs.len()
        ));
        Ok((console, hosts))
    }

    fn start_session(
        &mut self,
        link: DaemonLink,
        watch: Segment,
        rate: u32,
    ) -> Result<Session, String> {
        let h = link.header();
        let (base_underruns, base_overruns) = (
            h.from_daemon.underruns.load(Relaxed),
            h.to_daemon.overruns.load(Relaxed),
        );
        let (console, hosts) = self.build_console(rate, self.burst(&link))?;
        let shared = Arc::new(RtShared::default());
        let handle = self.spawn_rt(link, console, shared.clone())?;
        self.stats.reset_peak();
        self.last_snap = self.stats.snapshot();
        self.last_status = Instant::now();
        Ok(Session {
            shared,
            handle,
            watch,
            hosts,
            rate,
            base_underruns,
            base_overruns,
            last_check: Instant::now(),
        })
    }

    fn spawn_rt(
        &self,
        link: DaemonLink,
        console: Console,
        shared: Arc<RtShared>,
    ) -> Result<JoinHandle<(DaemonLink, Console, Exit)>, String> {
        let rt = self.opts.rt;
        let wi = self.wi.clone();
        std::thread::Builder::new()
            .name("wing-audio-rt".into())
            .spawn(move || rt_loop(link, console, shared, rt, wi))
            .map_err(|e| format!("spawn RT thread: {e}"))
    }

    /// One control tick of a running session. Returns `None` once it ended.
    fn supervise(&mut self, mut s: Session) -> Option<Session> {
        if s.handle.is_finished() {
            let Session {
                handle,
                hosts,
                watch,
                ..
            } = s;
            drop(hosts);
            match handle.join() {
                Ok((link, console, Exit::RateChanged(r))) => {
                    (self.log)(&format!(
                        "audio: driver sample rate changed to {r} Hz, rebuilding the console"
                    ));
                    self.pipeline = Some(console.into_pipeline());
                    match self.start_session(link, watch, r) {
                        Ok(s) => return Some(s),
                        Err(e) => self.say(format!("audio: rebuilding the console failed: {e}")),
                    }
                }
                Ok((link, console, Exit::Stopped)) => {
                    drop(link);
                    self.pipeline = Some(console.into_pipeline());
                    self.say("audio: detached".into());
                }
                Err(_) => self.say("audio: RT thread panicked; detached".into()),
            }
            return None;
        }
        if s.last_check.elapsed() >= CURRENCY_CHECK {
            s.last_check = Instant::now();
            if !s.watch.is_current() {
                self.end_session(
                    s,
                    "the driver's shm segment was removed or recreated, e.g. driver reinstalled; \
                     reattaching",
                );
                return None;
            }
        }
        let periodic = self
            .opts
            .status_every
            .is_some_and(|e| self.last_status.elapsed() >= e);
        let line = self.format_status(&s, periodic);
        if periodic {
            self.last_status = Instant::now();
            (self.log)(&line);
        }
        self.set_status(line);
        Some(s)
    }

    fn end_session(&mut self, s: Session, why: &str) {
        s.shared.stop.store(true, Relaxed);
        match s.handle.join() {
            Ok((link, console, _)) => {
                drop(link); // detach first: the driver fades to loopback right away
                self.pipeline = Some(console.into_pipeline());
            }
            Err(_) => (self.log)("audio: RT thread panicked"),
        }
        drop(s.hosts);
        self.say(format!(
            "audio: detached ({why}); the driver falls back to loopback"
        ));
    }

    /// Status line; `advance` starts a new measurement interval (load/percentiles/peak).
    fn format_status(&mut self, s: &Session, advance: bool) -> String {
        let snap = self.stats.snapshot();
        let d = snap.since(&self.last_snap);
        if advance {
            self.last_snap = snap;
            self.stats.reset_peak();
        }
        let st = s.watch.status();
        let driver = match st.driver_heartbeat_age_ns {
            Some(a) if a < 200_000_000 => "running",
            _ => "idle",
        };
        let policy = match s.shared.policy.load(Relaxed) {
            0 => "rt -".to_string(),
            1 => format!("rt ok ({} fr)", s.shared.policy_frames.load(Relaxed)),
            n => format!("rt FAILED ({})", n as i64 - 2),
        };
        let under = st.from_daemon.underruns.wrapping_sub(s.base_underruns);
        let over = st.to_daemon.overruns.wrapping_sub(s.base_overruns);
        format!(
            "audio: {} Hz, driver {driver}, load {:.1}% (peak {:.1}%), cycle p50 {} us p99 {} us, \
             xruns {} (driver underruns {under}, overruns {over}), slow cycles {}, {policy}{}, {}",
            s.rate,
            d.load_pct(),
            d.peak_load_pct(),
            d.percentile_ns(0.5) / 1000,
            d.percentile_ns(0.99) / 1000,
            under.wrapping_add(over),
            d.slow,
            if s.shared.joined_workgroup.load(Relaxed) {
                ", work interval"
            } else {
                ""
            },
            s.hosts.status(s.rate)
        )
    }
}
