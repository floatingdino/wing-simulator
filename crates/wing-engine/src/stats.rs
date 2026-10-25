//! Lock-free engine statistics: written by the driver thread once per cycle, read from any
//! control thread via [`StatsHandle::snapshot`].

use std::sync::atomic::{AtomicU64, Ordering::Relaxed};
use std::sync::Arc;

#[derive(Default)]
pub(crate) struct Stats {
    cycles: AtomicU64,
    last_ns: AtomicU64,
    max_ns: AtomicU64,
    total_ns: AtomicU64,
    xruns: AtomicU64,
    period_ns: AtomicU64,
}

impl Stats {
    /// Record one completed cycle. Called only by the driver thread (single writer).
    #[inline]
    pub(crate) fn record_cycle(&self, ns: u64) {
        self.cycles.fetch_add(1, Relaxed);
        self.last_ns.store(ns, Relaxed);
        self.total_ns.fetch_add(ns, Relaxed);
        if ns > self.max_ns.load(Relaxed) {
            self.max_ns.store(ns, Relaxed);
        }
    }

    #[inline]
    pub(crate) fn record_xrun(&self) {
        self.xruns.fetch_add(1, Relaxed);
    }

    pub(crate) fn set_period_ns(&self, ns: u64) {
        self.period_ns.store(ns, Relaxed);
    }

    pub(crate) fn period_ns(&self) -> u64 {
        self.period_ns.load(Relaxed)
    }
}

/// Point-in-time copy of the engine statistics. Fields are read individually (not as one
/// atomic unit), which is fine for monitoring.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct StatsSnapshot {
    /// Cycles processed since start/reset.
    pub cycles: u64,
    /// Duration of the most recent cycle (ns).
    pub last_cycle_ns: u64,
    /// Longest cycle since start/reset (ns).
    pub max_cycle_ns: u64,
    /// Mean cycle duration (ns).
    pub avg_cycle_ns: u64,
    /// Nominal cycle period, `block_size / sample_rate` (ns).
    pub period_ns: u64,
    /// Number of xruns (cycles over deadline in live mode, plus host-reported ones).
    pub xruns: u64,
}

impl StatsSnapshot {
    /// Load of the last cycle in percent of the period.
    pub fn load_pct(&self) -> f64 {
        pct(self.last_cycle_ns, self.period_ns)
    }

    /// Average load in percent of the period.
    pub fn avg_load_pct(&self) -> f64 {
        pct(self.avg_cycle_ns, self.period_ns)
    }

    /// Peak load in percent of the period.
    pub fn peak_load_pct(&self) -> f64 {
        pct(self.max_cycle_ns, self.period_ns)
    }
}

fn pct(ns: u64, period: u64) -> f64 {
    if period == 0 {
        0.0
    } else {
        ns as f64 * 100.0 / period as f64
    }
}

/// Cloneable, thread-safe handle for reading engine statistics.
#[derive(Clone)]
pub struct StatsHandle(pub(crate) Arc<Stats>);

impl StatsHandle {
    /// Read the current statistics.
    pub fn snapshot(&self) -> StatsSnapshot {
        let s = &self.0;
        let cycles = s.cycles.load(Relaxed);
        let total = s.total_ns.load(Relaxed);
        StatsSnapshot {
            cycles,
            last_cycle_ns: s.last_ns.load(Relaxed),
            max_cycle_ns: s.max_ns.load(Relaxed),
            avg_cycle_ns: total.checked_div(cycles).unwrap_or(0),
            period_ns: s.period_ns.load(Relaxed),
            xruns: s.xruns.load(Relaxed),
        }
    }

    /// Report an xrun detected outside the engine (e.g. by the host/driver).
    pub fn record_xrun(&self) {
        self.0.record_xrun();
    }

    /// Reset the peak cycle time (racy with the writer by design; monitoring only).
    pub fn reset_peak(&self) {
        self.0.max_ns.store(0, Relaxed);
    }
}

impl std::fmt::Debug for StatsHandle {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.snapshot().fmt(f)
    }
}
