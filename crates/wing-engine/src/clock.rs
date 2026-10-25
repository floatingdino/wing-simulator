//! Clock abstraction: what decides *when* a cycle runs.
//!
//! * [`LiveClock`]: driven by an external callback (CoreAudio IO proc / shm ring wakeup). The
//!   callback calls [`Clock::tick`] once per period; the first tick on a thread applies the RT
//!   policy (and optionally joins a workgroup). Cycles longer than the period count as xruns.
//! * [`OfflineClock`]: free-running loop for offline rendering, as fast as the CPU allows.
//!
//! Both feed the engine through the same [`CycleIo`] hooks.

use std::ops::ControlFlow;

use crate::graph::Port;
use crate::rt::{self, RawWorkgroup, TimeConstraint, WorkgroupMembership};
use crate::sched::Engine;

/// External I/O for one cycle. Implementations used with [`LiveClock`] run on the RT thread.
pub trait CycleIo {
    /// Fill external inputs (`inputs.channel(ch)`) for `cycle`. Return `Break` to stop
    /// (offline: end of input) without running the cycle.
    fn before_cycle(&mut self, cycle: u64, inputs: &mut Inputs<'_>) -> ControlFlow<()>;
    /// Consume outputs (any port) of the cycle that just ran.
    fn after_cycle(&mut self, cycle: u64, outputs: &Outputs<'_>);
}

/// Writable external inputs for the upcoming cycle.
pub struct Inputs<'a>(&'a mut Engine);

impl Inputs<'_> {
    /// External input channel `ch`.
    pub fn channel(&mut self, ch: usize) -> &mut [f32] {
        self.0.external_mut(ch)
    }

    /// Number of external channels.
    pub fn num_channels(&self) -> usize {
        self.0.graph().num_external()
    }
}

/// Read-only view of the graph's buffers after a cycle.
pub struct Outputs<'a>(&'a Engine);

impl Outputs<'_> {
    /// Samples of `port` produced by the cycle.
    pub fn port(&self, port: Port) -> &[f32] {
        self.0.port(port)
    }
}

/// Something that drives an [`Engine`].
pub trait Clock {
    /// Drive the engine. Live clocks run exactly one cycle per call (call from the device
    /// callback); offline clocks loop until done. Returns the number of cycles run.
    fn tick(&mut self, engine: &mut Engine, io: &mut dyn CycleIo) -> u64;
}

/// Run a single cycle with I/O hooks. Returns `None` if `before_cycle` asked to stop.
#[inline]
pub(crate) fn run_one(engine: &mut Engine, io: &mut dyn CycleIo) -> Option<std::time::Duration> {
    let next = engine.cycle() + 1;
    if io.before_cycle(next, &mut Inputs(engine)).is_break() {
        return None;
    }
    let dt = engine.process_cycle();
    io.after_cycle(engine.cycle(), &Outputs(engine));
    Some(dt)
}

/// Clock for an external periodic callback.
pub struct LiveClock {
    rt_policy: bool,
    workgroup: Option<RawWorkgroup>,
    initialised: bool,
    membership: Option<WorkgroupMembership>,
    rt_result: Option<Result<(), rt::RtError>>,
}

impl LiveClock {
    /// `rt_policy`: apply the time-constraint policy on the first tick. `workgroup`: join it on
    /// the first tick (pass `None` if the callback thread is already a member, as CoreAudio IO
    /// threads are of their device's workgroup).
    pub fn new(rt_policy: bool, workgroup: Option<RawWorkgroup>) -> Self {
        Self {
            rt_policy,
            workgroup,
            initialised: false,
            membership: None,
            rt_result: None,
        }
    }

    /// Result of the first-tick RT setup, once it ran.
    pub fn rt_result(&self) -> Option<Result<(), rt::RtError>> {
        self.rt_result
    }
}

impl Clock for LiveClock {
    fn tick(&mut self, engine: &mut Engine, io: &mut dyn CycleIo) -> u64 {
        if !self.initialised {
            // One-time setup on the callback thread (syscalls/allocation acceptable once).
            self.initialised = true;
            if self.rt_policy {
                let tc = TimeConstraint::for_period(engine.period().as_nanos() as u64);
                self.rt_result = Some(rt::set_current_thread_time_constraint(tc));
            }
            if let Some(wg) = self.workgroup {
                self.membership = WorkgroupMembership::join(wg).ok();
            }
        }
        match run_one(engine, io) {
            Some(dt) => {
                if dt > engine.period() {
                    engine.stats_inner().record_xrun();
                }
                1
            }
            None => 0,
        }
    }
}

/// Free-running offline clock.
#[derive(Debug, Clone, Copy, Default)]
pub struct OfflineClock {
    /// Stop after this many cycles (`None` = until `before_cycle` breaks).
    pub max_cycles: Option<u64>,
}

impl Clock for OfflineClock {
    fn tick(&mut self, engine: &mut Engine, io: &mut dyn CycleIo) -> u64 {
        let mut n = 0;
        while self.max_cycles.is_none_or(|m| n < m) {
            if run_one(engine, io).is_none() {
                break;
            }
            n += 1;
        }
        n
    }
}
