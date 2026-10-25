//! Level / envelope detectors for dynamics and metering.
//!
//! Time-constant convention everywhere: a one-pole with time constant `tau` moves 63.2% (1 - 1/e) of the way
//! to a step target after `tau` seconds (`coeff = exp(-1/(tau*fs))`).
//!
//! - [`PeakDetector`]: instant attack, exponential release (meters, limiter sidechains).
//! - [`EnvelopeFollower`]: branching one-pole attack/release on `|x|` (peak) or `x^2` (mean-square, output is
//!   RMS), with optional program-dependent release (dual time constant).
//! - [`RmsWindow`]: exact moving-window RMS over a fixed number of samples (preallocated).

use crate::smooth::one_pole_coeff;

/// Instant-attack, one-pole-release peak follower.
#[derive(Clone, Copy, Debug)]
pub struct PeakDetector {
    env: f32,
    release: f32,
}

impl PeakDetector {
    pub fn new(release_tau_s: f64, fs: f64) -> Self {
        Self {
            env: 0.0,
            release: one_pole_coeff(release_tau_s, fs) as f32,
        }
    }

    pub fn set_release(&mut self, tau_s: f64, fs: f64) {
        self.release = one_pole_coeff(tau_s, fs) as f32;
    }

    pub fn reset(&mut self) {
        self.env = 0.0;
    }

    #[inline]
    pub fn value(&self) -> f32 {
        self.env
    }

    #[inline]
    pub fn process(&mut self, x: f32) -> f32 {
        let a = x.abs();
        self.env = if a > self.env {
            a
        } else {
            self.env * self.release
        };
        self.env
    }

    /// Process a block, returning the envelope at its end.
    #[inline]
    pub fn process_block(&mut self, buf: &[f32]) -> f32 {
        for &x in buf {
            self.process(x);
        }
        self.env
    }
}

/// What the follower integrates.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DetectMode {
    /// `|x|`; output is a peak-ish envelope.
    Peak,
    /// `x^2`; output is `sqrt(env)` (RMS with exponential window).
    MeanSquare,
}

/// Release behaviour.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ReleaseMode {
    /// Single release time constant.
    Fixed,
    /// Program-dependent (dual) release: a second, slow envelope charges from the fast one with time
    /// constant `charge_tau_s` and releases with `slow_tau_s`; output is the max of both. Short transients
    /// barely charge the slow stage (fast recovery); sustained material does (slow recovery), as in opto /
    /// "auto" release compressors.
    ProgramDependent { charge_tau_s: f64, slow_tau_s: f64 },
}

/// Branching attack/release one-pole envelope follower.
#[derive(Clone, Copy, Debug)]
pub struct EnvelopeFollower {
    mode: DetectMode,
    attack: f32,
    release: f32,
    pd: Option<(f32, f32)>, // (charge, slow release)
    fast: f32,
    slow: f32,
}

impl EnvelopeFollower {
    pub fn new(
        mode: DetectMode,
        attack_tau_s: f64,
        release_tau_s: f64,
        release_mode: ReleaseMode,
        fs: f64,
    ) -> Self {
        let mut e = Self {
            mode,
            attack: 0.0,
            release: 0.0,
            pd: None,
            fast: 0.0,
            slow: 0.0,
        };
        e.set_times(attack_tau_s, release_tau_s, release_mode, fs);
        e
    }

    pub fn set_times(
        &mut self,
        attack_tau_s: f64,
        release_tau_s: f64,
        release_mode: ReleaseMode,
        fs: f64,
    ) {
        self.attack = one_pole_coeff(attack_tau_s, fs) as f32;
        self.release = one_pole_coeff(release_tau_s, fs) as f32;
        self.pd = match release_mode {
            ReleaseMode::Fixed => None,
            ReleaseMode::ProgramDependent {
                charge_tau_s,
                slow_tau_s,
            } => Some((
                one_pole_coeff(charge_tau_s, fs) as f32,
                one_pole_coeff(slow_tau_s, fs) as f32,
            )),
        };
    }

    pub fn reset(&mut self) {
        self.fast = 0.0;
        self.slow = 0.0;
    }

    #[inline]
    fn output(&self, e: f32) -> f32 {
        match self.mode {
            DetectMode::Peak => e,
            DetectMode::MeanSquare => e.sqrt(),
        }
    }

    /// Current envelope (linear amplitude).
    #[inline]
    pub fn value(&self) -> f32 {
        self.output(self.fast.max(self.slow))
    }

    #[inline]
    pub fn process(&mut self, x: f32) -> f32 {
        let d = match self.mode {
            DetectMode::Peak => x.abs(),
            DetectMode::MeanSquare => x * x,
        };
        let c = if d > self.fast {
            self.attack
        } else {
            self.release
        };
        self.fast = d + (self.fast - d) * c;
        if let Some((charge, slow_rel)) = self.pd {
            let c2 = if self.fast > self.slow {
                charge
            } else {
                slow_rel
            };
            self.slow = self.fast + (self.slow - self.fast) * c2;
            self.output(self.fast.max(self.slow))
        } else {
            self.output(self.fast)
        }
    }

    /// Envelope per sample into `out` (same length as `input`).
    #[inline]
    pub fn process_block(&mut self, input: &[f32], out: &mut [f32]) {
        for (o, &x) in out.iter_mut().zip(input) {
            *o = self.process(x);
        }
    }
}

/// Exact moving-window RMS. The ring of squares is allocated once (`max_window`); the running sum is f64
/// and recomputed from the ring once per window wrap so rounding drift cannot accumulate.
pub struct RmsWindow {
    ring: Vec<f32>,
    len: usize,
    pos: usize,
    sum: f64,
}

impl RmsWindow {
    pub fn new(max_window: usize, window: usize) -> Self {
        assert!(max_window > 0);
        let mut r = Self {
            ring: vec![0.0; max_window],
            len: 1,
            pos: 0,
            sum: 0.0,
        };
        r.set_window(window);
        r
    }

    /// Change window length (clamped to `1..=max_window`); clears history.
    pub fn set_window(&mut self, window: usize) {
        self.len = window.clamp(1, self.ring.len());
        self.reset();
    }

    pub fn reset(&mut self) {
        self.ring.iter_mut().for_each(|x| *x = 0.0);
        self.pos = 0;
        self.sum = 0.0;
    }

    #[inline]
    pub fn process(&mut self, x: f32) -> f32 {
        let sq = x * x;
        self.sum += sq as f64 - self.ring[self.pos] as f64;
        self.ring[self.pos] = sq;
        self.pos += 1;
        if self.pos == self.len {
            self.pos = 0;
            self.sum = self.ring[..self.len].iter().map(|&v| v as f64).sum();
        }
        (self.sum.max(0.0) / self.len as f64).sqrt() as f32
    }

    #[inline]
    pub fn process_block(&mut self, buf: &[f32]) -> f32 {
        let mut r = 0.0;
        for &x in buf {
            r = self.process(x);
        }
        r
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FS: f64 = 48_000.0;
    const E1: f32 = 0.632_120_56; // 1 - 1/e

    #[test]
    fn attack_and_release_time_constants() {
        let mut e = EnvelopeFollower::new(DetectMode::Peak, 0.005, 0.1, ReleaseMode::Fixed, FS);
        let mut v = 0.0;
        for _ in 0..240 {
            v = e.process(1.0);
        }
        assert!((v - E1).abs() < 1e-3, "attack 63% at tau: {v}");
        for _ in 0..48_000 {
            e.process(1.0);
        }
        for _ in 0..4800 {
            v = e.process(0.0);
        }
        assert!((v - (1.0 - E1)).abs() < 1e-3, "release 36.8% at tau: {v}");
    }

    #[test]
    fn mean_square_mode_measures_rms() {
        let mut e =
            EnvelopeFollower::new(DetectMode::MeanSquare, 0.05, 0.05, ReleaseMode::Fixed, FS);
        let mut v = 0.0;
        for n in 0..96_000 {
            v = e.process((2.0 * core::f32::consts::PI * 1000.0 * n as f32 / FS as f32).sin());
        }
        assert!(
            (v - core::f32::consts::FRAC_1_SQRT_2).abs() < 0.01,
            "rms {v}"
        );
    }

    #[test]
    fn peak_detector_release() {
        let mut p = PeakDetector::new(0.02, FS);
        assert_eq!(p.process(-0.8), 0.8);
        let mut v = 0.0;
        for _ in 0..960 {
            v = p.process(0.0);
        }
        assert!((v - 0.8 * (1.0 - E1)).abs() < 1e-3);
    }

    #[test]
    fn program_dependent_release() {
        let pd = ReleaseMode::ProgramDependent {
            charge_tau_s: 0.3,
            slow_tau_s: 1.5,
        };
        let recovery_after = |burst: usize| {
            let mut e = EnvelopeFollower::new(DetectMode::Peak, 0.001, 0.05, pd, FS);
            for _ in 0..burst {
                e.process(1.0);
            }
            // samples until envelope falls below 0.1
            (0..10 * 48_000).position(|_| e.process(0.0) < 0.1).unwrap()
        };
        let short = recovery_after(480); // 10 ms transient
        let long = recovery_after(96_000); // 2 s sustained
        assert!(long > 5 * short, "short {short} long {long}");
    }

    #[test]
    fn rms_window_exact() {
        let period = 48; // 1 kHz at 48 kHz
        let mut r = RmsWindow::new(4800, period * 10);
        let mut v = 0.0;
        for n in 0..10_000 {
            v = r.process((2.0 * core::f32::consts::PI * n as f32 / period as f32).sin());
        }
        assert!(
            (v - core::f32::consts::FRAC_1_SQRT_2).abs() < 1e-4,
            "rms {v}"
        );
        // DC step: exactly 1 after one full window
        let mut r = RmsWindow::new(100, 100);
        let v = (0..100).map(|_| r.process(1.0)).last().unwrap();
        assert!((v - 1.0).abs() < 1e-6);
    }
}
