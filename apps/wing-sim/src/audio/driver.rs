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
use wing_engine::rt::{set_current_thread_time_constraint, TimeConstraint};
use wing_engine::WorkgroupMembership;
use wing_hostio::HostIoConfig;
use wing_model::ParamStore;
use wing_shm::{DaemonLink, Error as ShmError, LayoutError, OpenOptions, Segment, WakeMode};

use super::console::{Console, ConsoleConfig};
use super::hostio::{self, BridgeReport, HostOutputs};
use super::stats::{AudioStats, Snapshot};
use super::workgroup::WorkInterval;
use super::ConsoleControl;
use super::LogFn;
use crate::record::Recorder;

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
    /// Replaces the CoreAudio side of the host bridges (soak harness / tests); `None`: real devices.
    pub device_factory: Option<super::hostio::DeviceFactory>,
}

impl Default for AudioOptions {
    fn default() -> Self {
        Self {
            shm_name: wing_shm::SHM_NAME.to_string(),
            console: ConsoleConfig::default(),
            rt: true,
            hostio: None,
            status_every: Some(Duration::from_secs(10)),
            device_factory: None,
        }
    }
}

/// Process every complete engine block the driver has made available: USB In → console →
/// USB Out → commit, then stamp the host bridges once for the whole burst (T027). Partial
/// blocks wait for the next driver cycle. Returns frames processed. RT-safe.
///
/// The daemon heartbeat is refreshed per block (one `mach_absolute_time` + relaxed store), so a
/// long burst (a console slower than realtime fed continuously, or a large backlog) never looks
/// like a dead daemon to the driver, which would drop to loopback after 100 ms.
#[inline]
pub fn service(link: &mut DaemonLink, console: &mut Console) -> usize {
    let block = console.block();
    let mut done = 0;
    while let Some(b) = link.next_block(block) {
        if b.frames < block {
            break;
        }
        link.heartbeat();
        let l = &*link;
        console.cycle(
            |ch, buf| l.read_input(&b, ch, buf),
            |ch, buf| l.write_output(&b, ch, buf),
        );
        link.commit(&b);
        done += b.frames;
    }
    if done > 0 {
        let burst = link.header().input_frames.load(Relaxed) as usize;
        console.end_burst(wing_hostio::host_time_ns(), burst.max(block));
    }
    done
}

/// Engine cycles run on silence when the driver starts a new IO epoch (DAW pressed play on
/// its device), before its first buffer arrives: after an idle period the workers sleep and
/// caches are cold, and the first bursts would otherwise run late (T027 startup underruns).
pub const EPOCH_WARM_UP_CYCLES: usize = 8;
/// Engine cycles run on silence before a session's RT thread starts.
pub const SESSION_WARM_UP_CYCLES: usize = 32;

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
    // Real-time policy before the first driver cycle (not lazily on its wake-up), sized for
    // the driver's current buffer (or 512 frames while it is idle).
    let mut apply_policy = |buf: u64| {
        if rt && buf != policy_frames {
            // One syscall per driver buffer-size change (like CoreAudio's own IO threads).
            policy_frames = buf;
            let period_ns = buf * 1_000_000_000 / fs as u64;
            let st = match set_current_thread_time_constraint(TimeConstraint::for_period(period_ns))
            {
                Ok(()) => 1,
                Err(wing_engine::RtError::Policy(k)) => 2 + k as u32 as u64,
                Err(_) => 2,
            };
            shared.policy.store(st, Relaxed);
            shared.policy_frames.store(buf, Relaxed);
        }
    };
    let initial = link.header().input_frames.load(Relaxed) as u64;
    apply_policy(if initial == 0 {
        512
    } else {
        initial.max(block)
    });
    let mut epoch = link.header().io_epoch.load(Relaxed);
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
        apply_policy(buf);
        let e = h.io_epoch.load(Relaxed);
        if e != epoch {
            epoch = e;
            // Driver IO (re)started: warm up while its first buffer is still on the way.
            if h.to_daemon.write_time.load(Relaxed) == 0 {
                console.warm_up(EPOCH_WARM_UP_CYCLES);
            }
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

/// Machine-readable audio status (T064 status file), updated by the control thread together
/// with the status line.
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize)]
pub struct AudioReport {
    /// A driver session is running.
    pub attached: bool,
    /// Engine sample rate (0 while detached).
    pub rate: u32,
    /// `running` (the driver's IO is cycling) or `idle`.
    pub driver: String,
    /// Engine load over the current measurement window (%).
    pub load_pct: f64,
    /// Peak cycle load (%).
    pub peak_load_pct: f64,
    /// Cycle-time percentiles (us).
    pub cycle_p50_us: u64,
    pub cycle_p99_us: u64,
    /// Driver underruns + overruns since the session started.
    pub xruns: u32,
    pub underruns: u32,
    pub overruns: u32,
    /// Engine cycles over budget.
    pub slow_cycles: u64,
    /// Host output bridges.
    pub bridges: Vec<BridgeReport>,
    /// The human-readable status line.
    pub message: String,
}

/// Host-output reconfiguration request: `Some(new)` replaces `AudioOptions::hostio`.
type PendingHostIo = Arc<Mutex<Option<Option<HostIoConfig>>>>;

/// Handle on the running audio service.
pub struct AudioService {
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
    status: Arc<Mutex<String>>,
    report: Arc<Mutex<AudioReport>>,
    pending_hostio: PendingHostIo,
    stats: Arc<AudioStats>,
    recorder: Arc<Recorder>,
}

impl AudioService {
    /// Start the control thread. `routing` must already be hooked to the store (see
    /// [`super::prepare`]).
    pub fn spawn(
        store: Arc<ParamStore>,
        routing: ConsoleControl,
        opts: AudioOptions,
        log: LogFn,
    ) -> std::io::Result<Self> {
        Self::spawn_with_recorder(store, routing, opts, log, Recorder::new())
    }

    /// [`spawn`](Self::spawn) with a recorder (T061) shared with e.g. the OSC server: every
    /// console the service builds polls its slot.
    pub fn spawn_with_recorder(
        store: Arc<ParamStore>,
        routing: ConsoleControl,
        opts: AudioOptions,
        log: LogFn,
        recorder: Arc<Recorder>,
    ) -> std::io::Result<Self> {
        let stop = Arc::new(AtomicBool::new(false));
        let status = Arc::new(Mutex::new("audio: starting".to_string()));
        let report = Arc::new(Mutex::new(AudioReport {
            message: "audio: starting".into(),
            ..AudioReport::default()
        }));
        let pending_hostio = PendingHostIo::default();
        let stats = Arc::new(AudioStats::default());
        let ctl = Control {
            store,
            routing,
            opts,
            log,
            stop: stop.clone(),
            status: status.clone(),
            report: report.clone(),
            pending_hostio: pending_hostio.clone(),
            stats: stats.clone(),
            pipeline: None,
            last_msg: String::new(),
            wi: None,
            last_snap: Snapshot::default(),
            last_status: Instant::now(),
            recorder: recorder.clone(),
        };
        let thread = std::thread::Builder::new()
            .name("wing-audio".into())
            .spawn(move || ctl.run())?;
        Ok(Self {
            stop,
            thread: Some(thread),
            status,
            report,
            pending_hostio,
            stats,
            recorder,
        })
    }

    /// The recorder (T061).
    pub fn recorder(&self) -> &Arc<Recorder> {
        &self.recorder
    }

    /// The latest machine-readable status.
    pub fn report(&self) -> AudioReport {
        self.report
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .clone()
    }

    /// Replace the host-output config (live reload, T064). A running session is restarted
    /// with the new bridges (a short audio gap); the parameter pipeline is kept.
    pub fn set_hostio(&self, cfg: Option<HostIoConfig>) {
        *self
            .pending_hostio
            .lock()
            .unwrap_or_else(|e| e.into_inner()) = Some(cfg);
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
        ShmError::NotFound if !cfg!(target_os = "macos") => {
            "audio: the Wing Simulator audio device is macOS-only; running control-only \
             (WING-Edit, OSC, scenes; use wing-render for audio)"
                .to_string()
        }
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
    routing: ConsoleControl,
    opts: AudioOptions,
    log: LogFn,
    stop: Arc<AtomicBool>,
    status: Arc<Mutex<String>>,
    report: Arc<Mutex<AudioReport>>,
    pending_hostio: PendingHostIo,
    stats: Arc<AudioStats>,
    pipeline: Option<ParamPipeline>,
    last_msg: String,
    wi: Option<Arc<WorkInterval>>,
    last_snap: Snapshot,
    last_status: Instant,
    recorder: Arc<Recorder>,
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

    /// Status while no session runs.
    fn set_status(&self, s: String) {
        self.set_report(AudioReport {
            message: s.clone(),
            ..AudioReport::default()
        });
        if let Ok(mut st) = self.status.lock() {
            *st = s;
        }
    }

    fn set_report(&self, r: AudioReport) {
        if let Ok(mut rep) = self.report.lock() {
            *rep = r;
        }
    }

    fn take_pending_hostio(&self) -> Option<Option<HostIoConfig>> {
        self.pending_hostio
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .take()
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
            // Meters do not use the telemetry ring (they go through `ConsoleConfig::meters`);
            // it is drained so it never fills.
            if let Some(rx) = telem.as_mut() {
                while rx.pop().is_some() {}
            }
            if let Some(cfg) = self.take_pending_hostio() {
                self.opts.hostio = cfg;
                (self.log)(&format!(
                    "audio: host outputs reconfigured: {}",
                    hostio::describe_opt(self.opts.hostio.as_ref())
                ));
                if let Some(s) = session.take() {
                    self.end_session(s, "host outputs reconfigured; restarting the session");
                }
                next_try = Instant::now();
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
            if let Some(m) = &self.opts.console.meters {
                // the meter sender uses real meters while a session runs, synthetic otherwise
                m.set_attached(session.is_some());
            }
            if let (Some(f), None) = (&self.opts.console.rta, &session) {
                // the RTA analyser falls back to synthetic data (Console::build sets the rate)
                f.set_sample_rate(0);
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        if let Some(s) = session {
            self.end_session(s, "shutdown");
        }
        if let Some(m) = &self.opts.console.meters {
            m.set_attached(false);
        }
        if let Some(f) = &self.opts.console.rta {
            f.set_sample_rate(0);
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

    /// Initial producer burst for the host bridges: the driver's IO buffer (512 while it is
    /// idle). The bridges follow the actual buffer at run time (`Console::end_burst`).
    fn burst(&self, link: &DaemonLink) -> usize {
        match link.header().input_frames.load(Relaxed) as usize {
            0 => 512,
            n => n,
        }
        .max(self.opts.console.block)
    }

    fn build_console(&mut self, rate: u32, burst: usize) -> Result<(Console, HostOutputs), String> {
        let t0 = Instant::now();
        let mut cfg = self.opts.console.clone();
        cfg.sample_rate = rate;
        cfg.workgroup = self.wi.as_ref().map(|w| w.raw());
        let (hosts, feeds, errs) = match &self.opts.hostio {
            Some(h) => HostOutputs::start(
                hostio::plan(h, rate, burst)?,
                h.buffer_frames,
                self.opts.device_factory.as_ref(),
            ),
            None => (HostOutputs::none(), Vec::new(), Vec::new()),
        };
        for e in errs {
            (self.log)(&format!("audio: {e}"));
        }
        let mut console = Console::build(
            cfg.clone(),
            &self.store,
            &mut self.routing,
            self.pipeline.take(),
            feeds,
            self.stats.clone(),
        )?;
        console.attach_recorder(self.recorder.slot().clone());
        // Prime: fault in buffers and wake the workers before the first driver cycle.
        let built_ms = t0.elapsed().as_secs_f64() * 1e3;
        console.warm_up(SESSION_WARM_UP_CYCLES);
        (self.log)(&format!(
            "audio: console built in {built_ms:.0} ms (+{:.0} ms warm-up): {rate} Hz, block {}, \
             {} workers{}, {} host bridge(s)",
            t0.elapsed().as_secs_f64() * 1e3 - built_ms,
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
        let mut report = self.format_status(&s, periodic);
        if periodic {
            self.last_status = Instant::now();
            (self.log)(&report.message);
        }
        if let Ok(mut st) = self.status.lock() {
            *st = report.message.clone();
        }
        report.attached = true;
        self.set_report(report);
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

    /// Status (line in `message`); `advance` starts a new measurement interval
    /// (load/percentiles/peak).
    fn format_status(&mut self, s: &Session, advance: bool) -> AudioReport {
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
        // Daemon share of the end-to-end latency: processing one driver buffer's worth of
        // engine blocks (median cycle time).
        let burst = (s.watch.header().input_frames.load(Relaxed) as u64)
            .max(self.opts.console.block as u64);
        let upstream_ms =
            d.percentile_ns(0.5) as f64 * 1e-6 * burst as f64 / self.opts.console.block as f64;
        let under = st.from_daemon.underruns.wrapping_sub(s.base_underruns);
        let over = st.to_daemon.overruns.wrapping_sub(s.base_overruns);
        let message = format!(
            "audio: {} Hz, driver {driver}, load {:.1}% (peak {:.1}%), cycle p50 {} us p99 {} us, \
             xruns {} (driver underruns {under}, overruns {over}), slow cycles {}, {policy}{}, {}, {}",
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
            s.hosts.status(s.rate, upstream_ms),
            self.recorder.status()
        );
        AudioReport {
            attached: true,
            rate: s.rate,
            driver: driver.into(),
            load_pct: d.load_pct(),
            peak_load_pct: d.peak_load_pct(),
            cycle_p50_us: d.percentile_ns(0.5) / 1000,
            cycle_p99_us: d.percentile_ns(0.99) / 1000,
            xruns: under.wrapping_add(over),
            underruns: under,
            overruns: over,
            slow_cycles: d.slow,
            bridges: s.hosts.report(s.rate, upstream_ms),
            message,
        }
    }
}
