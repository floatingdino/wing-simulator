//! Lock-free audio statistics: written by the RT driver thread once per engine cycle, read by the
//! control thread for the status line (load %, cycle-time percentiles, slow cycles).

use std::sync::atomic::{AtomicU64, Ordering::Relaxed};

/// Histogram resolution.
pub const BUCKET_NS: u64 = 5_000;
/// Number of buckets (the last one collects everything >= `(BUCKETS-1) * BUCKET_NS`).
pub const BUCKETS: usize = 1024;

/// Per-cycle counters and a cycle-time histogram.
pub struct AudioStats {
    hist: Box<[AtomicU64]>,
    cycles: AtomicU64,
    total_ns: AtomicU64,
    max_ns: AtomicU64,
    /// Cycles that took longer than one engine period.
    slow: AtomicU64,
    period_ns: AtomicU64,
}

impl Default for AudioStats {
    fn default() -> Self {
        Self {
            hist: (0..BUCKETS).map(|_| AtomicU64::new(0)).collect(),
            cycles: AtomicU64::new(0),
            total_ns: AtomicU64::new(0),
            max_ns: AtomicU64::new(0),
            slow: AtomicU64::new(0),
            period_ns: AtomicU64::new(0),
        }
    }
}

impl AudioStats {
    /// Nominal engine period (block / rate).
    pub fn set_period_ns(&self, ns: u64) {
        self.period_ns.store(ns, Relaxed);
    }

    /// Record one cycle (RT: atomics only). Single writer.
    #[inline]
    pub fn record(&self, ns: u64) {
        let b = ((ns / BUCKET_NS) as usize).min(BUCKETS - 1);
        self.hist[b].fetch_add(1, Relaxed);
        self.cycles.fetch_add(1, Relaxed);
        self.total_ns.fetch_add(ns, Relaxed);
        if ns > self.max_ns.load(Relaxed) {
            self.max_ns.store(ns, Relaxed);
        }
        if ns > self.period_ns.load(Relaxed) {
            self.slow.fetch_add(1, Relaxed);
        }
    }

    /// Copy everything (control thread; allocates).
    pub fn snapshot(&self) -> Snapshot {
        Snapshot {
            hist: self.hist.iter().map(|h| h.load(Relaxed)).collect(),
            cycles: self.cycles.load(Relaxed),
            total_ns: self.total_ns.load(Relaxed),
            max_ns: self.max_ns.load(Relaxed),
            slow: self.slow.load(Relaxed),
            period_ns: self.period_ns.load(Relaxed),
        }
    }

    /// Reset the peak (racy with the writer by design; monitoring only).
    pub fn reset_peak(&self) {
        self.max_ns.store(0, Relaxed);
    }
}

/// Plain copy of [`AudioStats`]; subtract two to get an interval.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Snapshot {
    pub hist: Vec<u64>,
    pub cycles: u64,
    pub total_ns: u64,
    /// Peak since the last [`AudioStats::reset_peak`] (not interval-subtracted).
    pub max_ns: u64,
    pub slow: u64,
    pub period_ns: u64,
}

impl Snapshot {
    /// `self - earlier` (counters and histogram), keeping `self`'s peak and period.
    pub fn since(&self, earlier: &Snapshot) -> Snapshot {
        let hist = if earlier.hist.len() == self.hist.len() {
            self.hist
                .iter()
                .zip(&earlier.hist)
                .map(|(a, b)| a.saturating_sub(*b))
                .collect()
        } else {
            self.hist.clone()
        };
        Snapshot {
            hist,
            cycles: self.cycles.saturating_sub(earlier.cycles),
            total_ns: self.total_ns.saturating_sub(earlier.total_ns),
            max_ns: self.max_ns,
            slow: self.slow.saturating_sub(earlier.slow),
            period_ns: self.period_ns,
        }
    }

    /// Cycle-time percentile `p` (0..=1) in ns, at bucket resolution (upper bucket edge).
    pub fn percentile_ns(&self, p: f64) -> u64 {
        let n: u64 = self.hist.iter().sum();
        if n == 0 {
            return 0;
        }
        let rank = ((p.clamp(0.0, 1.0) * n as f64).ceil() as u64).max(1);
        let mut acc = 0;
        for (i, &c) in self.hist.iter().enumerate() {
            acc += c;
            if acc >= rank {
                return (i as u64 + 1) * BUCKET_NS;
            }
        }
        BUCKETS as u64 * BUCKET_NS
    }

    /// Mean cycle time in percent of the engine period.
    pub fn load_pct(&self) -> f64 {
        if self.cycles == 0 || self.period_ns == 0 {
            return 0.0;
        }
        self.total_ns as f64 / self.cycles as f64 * 100.0 / self.period_ns as f64
    }

    /// Peak cycle time in percent of the engine period.
    pub fn peak_load_pct(&self) -> f64 {
        if self.period_ns == 0 {
            return 0.0;
        }
        self.max_ns as f64 * 100.0 / self.period_ns as f64
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn percentiles_and_load() {
        let s = AudioStats::default();
        s.set_period_ns(1_000_000);
        for _ in 0..99 {
            s.record(100_000);
        }
        s.record(2_000_000);
        let a = s.snapshot();
        assert_eq!(a.cycles, 100);
        assert_eq!(a.slow, 1);
        assert_eq!(a.percentile_ns(0.5), 105_000);
        assert_eq!(a.percentile_ns(0.99), 105_000);
        assert_eq!(a.percentile_ns(1.0), 2_005_000);
        assert!((a.load_pct() - 11.9).abs() < 1e-9);
        assert!((a.peak_load_pct() - 200.0).abs() < 1e-9);
        s.record(300_000);
        let d = s.snapshot().since(&a);
        assert_eq!(d.cycles, 1);
        assert_eq!(d.percentile_ns(0.5), 305_000);
        // overflow bucket
        s.record(u64::MAX / 2);
        assert_eq!(s.snapshot().hist[BUCKETS - 1], 1);
    }
}
