//! Parallel level scheduler: one driver thread (the caller of [`Engine::process_cycle`]) plus
//! `N` worker threads.
//!
//! # Protocol
//!
//! Each cycle has an *epoch* (the cycle number). Per level there are two counters, each a `u64`
//! packing `(epoch_lo32 << 32) | count`:
//!
//! * `claim[l]`: next unclaimed schedule position within the level. Threads claim with a CAS that
//!   also checks the epoch tag, so a late thread still running an old cycle can never steal a
//!   claim from the current one.
//! * `done[l]`: completed nodes. Everyone waits for `done[l] == (tag | len)` before moving on to
//!   level `l + 1`; the thread completing a level wakes any worker sleeping on it.
//!
//! The driver resets all counters to the new tag, publishes the epoch, rings the `bell` (wakes
//! sleeping workers), then participates like a worker and returns once the last level is
//! complete. Workers wait (for the bell and at level barriers) by spinning for
//! [`EngineConfig::spin`] and then sleeping on the counter's address (`os_sync_wait_on_address`,
//! a futex). Wakers only make the wake syscall if a sleeper counter says someone is asleep. The
//! driver never sleeps: it spins (then yields) at level barriers.
//!
//! Nothing on this path allocates or takes locks.

use std::sync::atomic::{AtomicBool, AtomicPtr, AtomicU32, AtomicU64, Ordering::*};
use std::sync::Arc;
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use crate::graph::{Graph, NodeId, Port, ProcessCtx, Processor};
use crate::rt::{self, RawWorkgroup, TimeConstraint, WorkgroupMembership};
use crate::stats::{Stats, StatsHandle};

/// Engine configuration.
#[derive(Debug, Clone)]
pub struct EngineConfig {
    /// Number of worker threads in addition to the driver (0 = single-threaded).
    pub workers: usize,
    /// How long a waiting worker spins before sleeping.
    pub spin: Duration,
    /// Apply `THREAD_TIME_CONSTRAINT_POLICY` to worker threads.
    pub rt_workers: bool,
    /// Workgroup that workers join at start (see [`Engine::set_workgroup`]).
    pub workgroup: Option<RawWorkgroup>,
}

impl Default for EngineConfig {
    fn default() -> Self {
        let n = thread::available_parallelism()
            .map(|n| n.get())
            .unwrap_or(1);
        Self {
            workers: n.saturating_sub(1).min(7),
            spin: Duration::from_micros(50),
            rt_workers: true,
            workgroup: None,
        }
    }
}

/// Address-based wait/wake (futex). macOS 14.4+: `os_sync_wait_on_address`; elsewhere a
/// yield-based fallback (correct, but busy).
mod futex {
    use std::sync::atomic::AtomicU64;

    /// Sleep while `*addr == expected` (may return spuriously).
    #[inline]
    pub fn wait(addr: &AtomicU64, expected: u64) {
        #[cfg(target_os = "macos")]
        // SAFETY: `addr` is a valid, 8-byte aligned atomic for the duration of the call.
        unsafe {
            libc::os_sync_wait_on_address(addr.as_ptr().cast(), expected, 8, 0);
        }
        #[cfg(not(target_os = "macos"))]
        {
            let _ = (addr, expected);
            std::thread::yield_now();
        }
    }

    /// Wake every thread sleeping on `addr`.
    #[inline]
    pub fn wake_all(addr: &AtomicU64) {
        #[cfg(target_os = "macos")]
        // SAFETY: as above. ENOENT (no waiters) is fine.
        unsafe {
            libc::os_sync_wake_by_address_all(addr.as_ptr().cast(), 8, 0);
        }
        #[cfg(not(target_os = "macos"))]
        let _ = addr;
    }
}

#[repr(align(128))]
#[derive(Default)]
struct Padded<T>(T);

impl<T> std::ops::Deref for Padded<T> {
    type Target = T;
    fn deref(&self) -> &T {
        &self.0
    }
}

struct Shared {
    graph: Graph,
    epoch: Padded<AtomicU64>,
    /// Incremented for every event workers must react to (new cycle, workgroup change,
    /// shutdown); idle workers sleep on it.
    bell: Padded<AtomicU64>,
    bell_sleepers: Padded<AtomicU32>,
    level_sleepers: Padded<AtomicU32>,
    claim: Box<[Padded<AtomicU64>]>,
    done: Box<[Padded<AtomicU64>]>,
    shutdown: AtomicBool,
    /// Per worker: 0 = not attempted, 1 = ok, n = kern_return + 2.
    rt_status: Box<[AtomicU64]>,
    spin: Duration,
    wg_ptr: AtomicPtr<std::ffi::c_void>,
    wg_gen: AtomicU64,
}

const TAG_MASK: u64 = !0xffff_ffff;

#[inline]
fn tag(epoch: u64) -> u64 {
    (epoch & 0xffff_ffff) << 32
}

impl Shared {
    /// Wake idle workers (call after the event's state change).
    #[inline]
    fn ring(&self) {
        self.bell.fetch_add(1, SeqCst);
        if self.bell_sleepers.load(SeqCst) > 0 {
            futex::wake_all(&self.bell);
        }
    }

    /// Spin, then (workers only) sleep on `addr` until `cond` holds. The driver (`sleep ==
    /// false`) yields instead of sleeping.
    #[inline]
    fn wait(
        &self,
        addr: &AtomicU64,
        sleepers: &AtomicU32,
        sleep: bool,
        mut cond: impl FnMut() -> bool,
    ) {
        let mut spins = 0u32;
        let mut start: Option<Instant> = None;
        loop {
            if cond() {
                return;
            }
            std::hint::spin_loop();
            spins = spins.wrapping_add(1);
            if spins & 63 != 0 {
                continue;
            }
            let t0 = *start.get_or_insert_with(Instant::now);
            if t0.elapsed() < self.spin {
                continue;
            }
            if !sleep {
                thread::yield_now();
                continue;
            }
            // Dekker-style handshake with the waker (all SeqCst): either we see the new value
            // (cond true / futex value mismatch) or the waker sees `sleepers > 0` and wakes us.
            let v = addr.load(SeqCst);
            sleepers.fetch_add(1, SeqCst);
            if !cond() {
                futex::wait(addr, v);
            }
            sleepers.fetch_sub(1, SeqCst);
        }
    }

    /// Participate in cycle `epoch`. Returns early if the cycle is already over (late worker).
    #[inline]
    fn run_cycle(&self, epoch: u64, worker: bool) {
        let t = tag(epoch);
        let ctx = ProcessCtx {
            cycle: epoch,
            block_size: self.graph.block_size(),
            sample_rate: self.graph.sample_rate(),
        };
        let nl = self.graph.num_levels();
        for l in 0..nl {
            let range = self.graph.level_range(l);
            let len = range.len() as u64;
            let claim = &self.claim[l];
            let done = &self.done[l];
            let target = t | len;
            loop {
                let v = claim.load(Acquire);
                if v & TAG_MASK != t {
                    return; // stale
                }
                let idx = v & 0xffff_ffff;
                if idx >= len {
                    break;
                }
                if claim
                    .compare_exchange_weak(v, v + 1, AcqRel, Relaxed)
                    .is_err()
                {
                    continue;
                }
                // SAFETY: we hold the unique claim on this position for this epoch, and the
                // previous level's barrier (acquire on `done`) completed before we got here.
                unsafe { self.graph.run_node(range.start + idx as usize, &ctx) };
                if done.fetch_add(1, SeqCst) + 1 == target && self.level_sleepers.load(SeqCst) > 0 {
                    futex::wake_all(done);
                }
            }
            if worker && l + 1 == nl {
                // Workers need not wait for the final level; the driver does.
                return;
            }
            self.wait(done, &self.level_sleepers, worker, || {
                let d = done.load(SeqCst);
                d == target || d & TAG_MASK != t
            });
            if done.load(Acquire) != target {
                return; // stale: a newer cycle began
            }
        }
    }
}

fn worker_main(shared: Arc<Shared>, idx: usize, rt_tc: Option<TimeConstraint>) {
    if let Some(tc) = rt_tc {
        let st = match rt::set_current_thread_time_constraint(tc) {
            Ok(()) => 1,
            Err(rt::RtError::Policy(k)) => 2 + k as u32 as u64,
            Err(_) => 2,
        };
        shared.rt_status[idx].store(st, Relaxed);
    }
    let mut membership: Option<WorkgroupMembership> = None;
    let mut wg_gen = 0u64;
    // Start from the initial values (not a fresh load) so events raised before this thread got
    // scheduled (first cycle, shutdown) are never missed.
    let mut seen_epoch = 0u64;
    let mut bell = 0u64;
    loop {
        // Workgroup changes are applied between cycles (join allocates its token + syscalls).
        let g = shared.wg_gen.load(Acquire);
        if g != wg_gen {
            wg_gen = g;
            membership = None;
            let p = shared.wg_ptr.load(Acquire);
            if !p.is_null() {
                // SAFETY: `Engine::set_workgroup`'s contract keeps the pointer valid.
                membership = WorkgroupMembership::join(unsafe { RawWorkgroup::from_raw(p) }).ok();
            }
        }
        shared.wait(&shared.bell, &shared.bell_sleepers, true, || {
            shared.bell.load(SeqCst) != bell || shared.shutdown.load(SeqCst)
        });
        bell = shared.bell.load(Acquire);
        if shared.shutdown.load(SeqCst) {
            break;
        }
        let ep = shared.epoch.load(Acquire);
        if ep != seen_epoch {
            seen_epoch = ep;
            shared.run_cycle(ep, true);
        }
    }
    drop(membership);
}

/// The realtime engine host: owns the graph, the worker pool and the statistics.
pub struct Engine {
    shared: Arc<Shared>,
    handles: Vec<JoinHandle<()>>,
    stats: Arc<Stats>,
    epoch: u64,
}

impl Engine {
    /// Start an engine for `graph`, spawning `config.workers` worker threads.
    pub fn new(graph: Graph, config: EngineConfig) -> Self {
        let nl = graph.num_levels();
        let period_ns = (graph.block_size() as f64 * 1e9 / graph.sample_rate() as f64) as u64;
        let wg = config
            .workgroup
            .map_or(std::ptr::null_mut(), |w| w.as_ptr());
        let shared = Arc::new(Shared {
            claim: (0..nl).map(|_| Padded(AtomicU64::new(0))).collect(),
            done: (0..nl).map(|_| Padded(AtomicU64::new(0))).collect(),
            graph,
            epoch: Padded(AtomicU64::new(0)),
            bell: Padded(AtomicU64::new(0)),
            bell_sleepers: Padded(AtomicU32::new(0)),
            level_sleepers: Padded(AtomicU32::new(0)),
            shutdown: AtomicBool::new(false),
            rt_status: (0..config.workers).map(|_| AtomicU64::new(0)).collect(),
            spin: config.spin,
            wg_ptr: AtomicPtr::new(wg),
            wg_gen: AtomicU64::new(config.workgroup.is_some() as u64),
        });
        let rt_tc = config
            .rt_workers
            .then(|| TimeConstraint::for_period(period_ns));
        let handles = (0..config.workers)
            .map(|i| {
                let s = shared.clone();
                thread::Builder::new()
                    .name(format!("wing-worker-{i}"))
                    .stack_size(256 * 1024)
                    .spawn(move || worker_main(s, i, rt_tc))
                    .expect("spawn worker thread")
            })
            .collect();
        let stats = Arc::new(Stats::default());
        stats.set_period_ns(period_ns);
        Self {
            shared,
            handles,
            stats,
            epoch: 0,
        }
    }

    /// The graph (read-only; outputs reflect the last completed cycle).
    pub fn graph(&self) -> &Graph {
        &self.shared.graph
    }

    /// Mutable external input channel `ch` (fill before `process_cycle`).
    pub fn external_mut(&mut self, ch: usize) -> &mut [f32] {
        // SAFETY: `&mut self` excludes a running cycle (cycles only run inside
        // `process_cycle(&mut self)`; workers only touch node buffers while holding a claim).
        unsafe { self.shared.graph.external_shared(ch) }
    }

    /// Read a port after a cycle.
    pub fn port(&self, p: Port) -> &[f32] {
        self.shared.graph.port(p)
    }

    /// Mutable access to a node's processor between cycles.
    pub fn processor_mut(&mut self, id: NodeId) -> &mut dyn Processor {
        // SAFETY: as for `external_mut`.
        unsafe { self.shared.graph.processor_shared(id) }
    }

    /// Statistics handle (clone it to a control thread).
    pub fn stats(&self) -> StatsHandle {
        StatsHandle(self.stats.clone())
    }

    /// Nominal cycle period.
    pub fn period(&self) -> Duration {
        Duration::from_nanos(self.stats.period_ns())
    }

    /// Number of worker threads.
    pub fn num_workers(&self) -> usize {
        self.handles.len()
    }

    /// Per-worker result of applying the time-constraint policy: `None` = not attempted / not
    /// yet, `Some(Ok)`, or `Some(Err(kern_return))`.
    pub fn worker_rt_status(&self) -> Vec<Option<Result<(), i32>>> {
        self.shared
            .rt_status
            .iter()
            .map(|s| match s.load(Relaxed) {
                0 => None,
                1 => Some(Ok(())),
                n => Some(Err((n - 2) as u32 as i32)),
            })
            .collect()
    }

    /// Ask every worker to (re)join `wg` (or leave, with `None`) between cycles.
    ///
    /// # Safety
    /// The workgroup must stay valid until it is replaced and every worker has observed the
    /// change, or until the engine is dropped.
    pub unsafe fn set_workgroup(&self, wg: Option<RawWorkgroup>) {
        let p = wg.map_or(std::ptr::null_mut(), |w| w.as_ptr());
        self.shared.wg_ptr.store(p, Release);
        self.shared.wg_gen.fetch_add(1, Release);
        self.shared.ring();
    }

    /// Run one cycle on the calling thread plus the worker pool and record its duration.
    /// RT-safe: no allocation and no locks; the only syscall is a single futex wake, made only
    /// if some worker is asleep.
    pub fn process_cycle(&mut self) -> Duration {
        let t0 = Instant::now();
        self.epoch += 1;
        let ep = self.epoch;
        let s = &*self.shared;
        let t = tag(ep);
        for (c, d) in s.claim.iter().zip(s.done.iter()) {
            c.store(t, Relaxed);
            d.store(t, Relaxed);
        }
        s.epoch.store(ep, SeqCst);
        s.ring();
        s.run_cycle(ep, false);
        let dt = t0.elapsed();
        self.stats.record_cycle(dt.as_nanos() as u64);
        dt
    }

    /// Run one cycle entirely on the calling thread with no scheduler involvement (reference
    /// path for tests and benchmarks). Not recorded in stats.
    pub fn process_cycle_serial(&mut self) {
        self.epoch += 1;
        let ctx = ProcessCtx {
            cycle: self.epoch,
            block_size: self.shared.graph.block_size(),
            sample_rate: self.shared.graph.sample_rate(),
        };
        // Workers never see this cycle (the shared epoch is not published) and the next
        // `process_cycle` uses a fresh epoch tag.
        // SAFETY: `&mut self` excludes a running parallel cycle; idle workers touch only atomics.
        unsafe { self.shared.graph.run_serial_shared(&ctx) };
    }

    pub(crate) fn stats_inner(&self) -> &Stats {
        &self.stats
    }

    /// Cycles run so far (parallel and serial).
    pub fn cycle(&self) -> u64 {
        self.epoch
    }
}

impl Drop for Engine {
    fn drop(&mut self) {
        self.shared.shutdown.store(true, SeqCst);
        self.shared.ring();
        for h in self.handles.drain(..) {
            let _ = h.join();
        }
    }
}
