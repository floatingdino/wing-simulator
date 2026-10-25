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
//! * `done[l]`: completed nodes. A thread that finishes the level's last node wakes parked
//!   workers. Everyone waits for `done[l] == (tag | len)` before moving to level `l + 1`.
//!
//! The driver resets all counters to the new tag, stores the epoch (release) and wakes workers;
//! then it participates like a worker and returns once the last level is complete. Workers wait
//! for a new epoch and for level barriers with spin-then-park (`thread::park`, woken via a
//! per-worker `parked` flag so no syscall happens unless a worker is actually asleep). The
//! driver never sleeps: it spins (then yields) at level barriers.
//!
//! Nothing on this path allocates or takes locks.

use std::sync::atomic::{AtomicBool, AtomicPtr, AtomicU64, Ordering::*};
use std::sync::{Arc, OnceLock};
use std::thread::{self, JoinHandle, Thread};
use std::time::{Duration, Instant};

use crate::graph::{Graph, NodeId, Port, ProcessCtx, Processor};
use crate::rt::{self, RawWorkgroup, TimeConstraint, WorkgroupMembership};
use crate::stats::{Stats, StatsHandle};

/// Engine configuration.
#[derive(Debug, Clone)]
pub struct EngineConfig {
    /// Number of worker threads in addition to the driver (0 = single-threaded).
    pub workers: usize,
    /// How long a waiting worker spins before parking.
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

#[repr(align(128))]
#[derive(Default)]
struct Padded<T>(T);

impl<T> std::ops::Deref for Padded<T> {
    type Target = T;
    fn deref(&self) -> &T {
        &self.0
    }
}

struct WorkerSlot {
    parked: Padded<AtomicBool>,
    thread: OnceLock<Thread>,
    /// 0 = not attempted, 1 = ok, other = error code (+2 to avoid collisions).
    rt_status: AtomicU64,
}

struct Shared {
    graph: Graph,
    epoch: Padded<AtomicU64>,
    claim: Box<[Padded<AtomicU64>]>,
    done: Box<[Padded<AtomicU64>]>,
    shutdown: AtomicBool,
    workers: Box<[WorkerSlot]>,
    spin: Duration,
    wg_ptr: AtomicPtr<std::ffi::c_void>,
    wg_gen: AtomicU64,
}

#[inline]
fn tag(epoch: u64) -> u64 {
    (epoch & 0xffff_ffff) << 32
}

impl Shared {
    #[inline]
    fn wake_parked(&self) {
        for w in self.workers.iter() {
            if w.parked.swap(false, SeqCst) {
                if let Some(t) = w.thread.get() {
                    t.unpark();
                }
            }
        }
    }

    /// Spin-then-park until `cond` holds. `me` = worker index, or `None` for the driver (never
    /// parks; yields instead).
    #[inline]
    fn wait(&self, me: Option<usize>, mut cond: impl FnMut() -> bool) {
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
            match me {
                None => thread::yield_now(),
                Some(i) => {
                    let slot = &self.workers[i];
                    slot.parked.store(true, SeqCst);
                    if cond() || self.shutdown.load(SeqCst) {
                        slot.parked.store(false, SeqCst);
                        return;
                    }
                    thread::park();
                    slot.parked.store(false, SeqCst);
                    if self.shutdown.load(SeqCst) {
                        return;
                    }
                }
            }
        }
    }

    /// Participate in cycle `epoch`. Returns early if the cycle is already over (late worker).
    #[inline]
    fn run_cycle(&self, epoch: u64, me: Option<usize>) {
        let t = tag(epoch);
        let ctx = ProcessCtx {
            cycle: epoch,
            block_size: self.graph.block_size(),
            sample_rate: self.graph.sample_rate(),
        };
        for l in 0..self.graph.num_levels() {
            let range = self.graph.level_range(l);
            let len = range.len() as u64;
            let claim = &self.claim[l];
            let done = &self.done[l];
            let target = t | len;
            loop {
                let v = claim.load(Acquire);
                if v & !0xffff_ffff != t {
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
                if done.fetch_add(1, SeqCst) + 1 == target {
                    self.wake_parked();
                }
            }
            if me.is_some() && l + 1 == self.graph.num_levels() {
                // Workers need not wait for the final level; the driver does.
                return;
            }
            self.wait(me, || {
                let d = done.load(SeqCst);
                d == target || d & !0xffff_ffff != t
            });
            if done.load(Acquire) != target {
                return; // stale: a newer cycle began
            }
        }
    }
}

fn worker_main(shared: Arc<Shared>, idx: usize, rt_tc: Option<TimeConstraint>) {
    // Register our handle before we can ever park, so wakers always find it.
    let _ = shared.workers[idx].thread.set(thread::current());
    if let Some(tc) = rt_tc {
        let st = match rt::set_current_thread_time_constraint(tc) {
            Ok(()) => 1,
            Err(rt::RtError::Policy(k)) => 2 + k as u32 as u64,
            Err(_) => 2,
        };
        shared.workers[idx].rt_status.store(st, Relaxed);
    }
    let mut membership: Option<WorkgroupMembership> = None;
    let mut wg_gen = 0u64;
    let mut seen = shared.epoch.load(Acquire);
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
        shared.wait(Some(idx), || shared.epoch.load(SeqCst) != seen);
        if shared.shutdown.load(SeqCst) {
            break;
        }
        seen = shared.epoch.load(Acquire);
        shared.run_cycle(seen, Some(idx));
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
        let shared = Arc::new(Shared {
            claim: (0..nl).map(|_| Padded(AtomicU64::new(0))).collect(),
            done: (0..nl).map(|_| Padded(AtomicU64::new(0))).collect(),
            graph,
            epoch: Padded(AtomicU64::new(0)),
            shutdown: AtomicBool::new(false),
            workers: (0..config.workers)
                .map(|_| WorkerSlot {
                    parked: Padded(AtomicBool::new(false)),
                    thread: OnceLock::new(),
                    rt_status: AtomicU64::new(0),
                })
                .collect(),
            spin: config.spin,
            wg_ptr: AtomicPtr::new(
                config
                    .workgroup
                    .map_or(std::ptr::null_mut(), |w| w.as_ptr()),
            ),
            wg_gen: AtomicU64::new(config.workgroup.is_some() as u64),
        });
        let rt_tc = config
            .rt_workers
            .then(|| TimeConstraint::for_period(period_ns));
        let handles = (0..config.workers)
            .map(|i| {
                let s = shared.clone();
                let h = thread::Builder::new()
                    .name(format!("wing-worker-{i}"))
                    .stack_size(256 * 1024)
                    .spawn(move || worker_main(s, i, rt_tc))
                    .expect("spawn worker thread");
                let _ = shared.workers[i].thread.set(h.thread().clone());
                h
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

    /// Per-worker result of applying the time-constraint policy: `None` = not attempted / not yet,
    /// `Some(Ok)`, or `Some(Err(kern_return))`.
    pub fn worker_rt_status(&self) -> Vec<Option<Result<(), i32>>> {
        self.shared
            .workers
            .iter()
            .map(|w| match w.rt_status.load(Relaxed) {
                0 => None,
                1 => Some(Ok(())),
                n => Some(Err((n - 2) as u32 as i32)),
            })
            .collect()
    }

    /// Ask every worker to (re)join `wg` (or leave, with `None`) before its next cycle.
    ///
    /// # Safety
    /// The workgroup must stay valid until it is replaced and every worker has run at least one
    /// more cycle, or until the engine is dropped.
    pub unsafe fn set_workgroup(&self, wg: Option<RawWorkgroup>) {
        self.shared
            .wg_ptr
            .store(wg.map_or(std::ptr::null_mut(), |w| w.as_ptr()), Release);
        self.shared.wg_gen.fetch_add(1, Release);
        self.shared.wake_parked();
    }

    /// Run one cycle on the calling thread plus the worker pool and record its duration.
    /// RT-safe: no allocation, no locks; the only syscalls are `unpark`s of sleeping workers.
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
        s.wake_parked();
        s.run_cycle(ep, None);
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
        for w in self.shared.workers.iter() {
            w.parked.store(false, SeqCst);
            if let Some(t) = w.thread.get() {
                t.unpark();
            }
        }
        for h in self.handles.drain(..) {
            let _ = h.join();
        }
    }
}
