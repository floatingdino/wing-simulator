//! PI drift controller: turns ring-fill error into a resampling-ratio correction.
//!
//! Plant: with the resampler consuming `step = nominal * (1 + c)` input frames per output frame,
//! the ring fill obeys `d fill / dt = fs_in * (-(drift) - c)` (an integrator). A PI controller
//! on the (low-pass filtered) fill error gives a 2nd-order loop
//! `s^2 + fs_in*Kp*s + fs_in*Ki = 0`; we pick `Kp`, `Ki` from a natural frequency and damping.
//! The correction is clamped (with integrator anti-windup) and slew-limited so the pitch never
//! moves abruptly.

/// Controller tuning.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DriftConfig {
    /// Loop natural frequency (Hz). Lower = smoother ratio, slower convergence.
    pub bandwidth_hz: f64,
    /// Damping ratio.
    pub damping: f64,
    /// Time constant of the fill-error low-pass (s).
    pub smoothing_s: f64,
    /// Correction clamp (ppm).
    pub max_correction_ppm: f64,
    /// Maximum correction change rate (ppm per second).
    pub max_slew_ppm_per_s: f64,
}

impl Default for DriftConfig {
    fn default() -> Self {
        Self {
            bandwidth_hz: 0.015,
            damping: 1.0,
            smoothing_s: 1.0,
            max_correction_ppm: 2000.0,
            max_slew_ppm_per_s: 200.0,
        }
    }
}

/// PI controller state.
#[derive(Debug, Clone)]
pub struct DriftController {
    kp: f64,
    ki: f64,
    smoothing_s: f64,
    max_c: f64,
    max_slew: f64,
    err_f: f64,
    integ: f64,
    correction: f64,
    primed: bool,
}

impl DriftController {
    /// Controller for an input (ring) rate of `fs_in` Hz.
    pub fn new(fs_in: f64, cfg: DriftConfig) -> Self {
        let wn = 2.0 * std::f64::consts::PI * cfg.bandwidth_hz;
        Self {
            kp: 2.0 * cfg.damping * wn / fs_in,
            ki: wn * wn / fs_in,
            smoothing_s: cfg.smoothing_s.max(1e-6),
            max_c: cfg.max_correction_ppm * 1e-6,
            max_slew: cfg.max_slew_ppm_per_s * 1e-6,
            err_f: 0.0,
            integ: 0.0,
            correction: 0.0,
            primed: false,
        }
    }

    /// Forget the error history (after a resync) but keep the learned drift (integrator).
    pub fn reset_error(&mut self) {
        self.primed = false;
        self.err_f = 0.0;
    }

    /// Forget everything (new device / sample rate).
    pub fn reset(&mut self) {
        self.reset_error();
        self.integ = 0.0;
        self.correction = 0.0;
    }

    /// Current correction (fractional, e.g. 1e-4 = +100 ppm).
    pub fn correction(&self) -> f64 {
        self.correction
    }

    /// Filtered fill error (frames).
    pub fn filtered_error(&self) -> f64 {
        self.err_f
    }

    /// Feed one measurement: `error` = fill - target (input frames), `dt` = seconds since the
    /// previous update. Returns the new correction.
    pub fn update(&mut self, error: f64, dt: f64) -> f64 {
        if !self.primed {
            self.primed = true;
            self.err_f = error;
        } else {
            let a = 1.0 - (-dt / self.smoothing_s).exp();
            self.err_f += a * (error - self.err_f);
        }
        let integ = self.integ + self.err_f * dt;
        let raw = self.kp * self.err_f + self.ki * integ;
        let clamped = raw.clamp(-self.max_c, self.max_c);
        if clamped == raw {
            self.integ = integ; // anti-windup: only integrate while unsaturated
        }
        let max_step = self.max_slew * dt;
        self.correction += (clamped - self.correction).clamp(-max_step, max_step);
        self.correction
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Ideal plant: converge on a 300 ppm drift, final error ~0, correction = -drift... sign:
    /// fill grows when the consumer is slow, so correction must be +drift to consume faster.
    #[test]
    fn converges_on_constant_drift() {
        let fs = 48_000.0;
        let mut c = DriftController::new(fs, DriftConfig::default());
        let drift = 300e-6; // producer faster than consumer by 300 ppm
        let dt = 0.01;
        let mut fill_err = 0.0;
        let mut max_err: f64 = 0.0;
        for _ in 0..(300.0 / dt) as usize {
            let corr = c.update(fill_err, dt);
            fill_err += fs * (drift - corr) * dt;
            max_err = max_err.max(fill_err.abs());
        }
        assert!((c.correction() - drift).abs() < 1e-7, "{}", c.correction());
        assert!(fill_err.abs() < 0.5, "{fill_err}");
        assert!(max_err < 200.0, "{max_err}");
    }

    #[test]
    fn clamp_and_slew() {
        let mut c = DriftController::new(48_000.0, DriftConfig::default());
        let v = c.update(1e9, 0.01);
        assert!((v - 2e-6).abs() < 1e-12, "slew-limited: {v}");
        for _ in 0..10_000 {
            c.update(1e9, 0.01);
        }
        assert!((c.correction() - 2000e-6).abs() < 1e-12);
    }
}
