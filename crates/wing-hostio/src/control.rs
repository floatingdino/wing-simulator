//! Drift controller: turns the ring-fill error into a resampling-ratio correction.
//!
//! Plant: with the resampler consuming `step = nominal * (1 + c)` input frames per output frame,
//! the ring fill obeys `d fill / dt = fs_in * (drift - c)`: an integrator of the frequency
//! error. The controller splits the correction into two parts (T027):
//!
//! * **Drift (frequency) estimate** `f`: a frequency-locked loop. Over windows of
//!   `freq_window_s` it measures the true drift directly as
//!   `mean(c) + d(fill)/dt / fs_in` (box-averaged fill per window, so a phase step can only
//!   corrupt two consecutive measurements), takes the median of the last five measurements
//!   (rejecting those). Once the history allows, a measurement over a long baseline of eight
//!   windows replaces that median when the two agree (the fill noise cancels between windows,
//!   so the long baseline is far less noisy; a phase step makes them disagree). `f` moves
//!   towards the measurement: directly for the first three measurements
//!   after a cold start (acquisition), then with time constant `freq_tc_s` (tracking). Phase errors do
//!   not leak into `f`: correcting a phase error at rate `p` shows up as `d(fill)/dt = -p`, so
//!   the measurement still equals the drift. No integrator, hence nothing to wind up.
//! * **Phase (fill) correction** `p`: proportional to the low-passed fill error outside a small
//!   deadband, with time constant `phase_tc_s`, clamped to `max_phase_ppm`. A step in the fill
//!   (e.g. the DAW skipped a cycle and the driver delivered a whole extra buffer) is drained at
//!   a bounded, constant rate instead of kicking the ratio by thousands of ppm.
//!
//! `c = f + p`, clamped to `max_correction_ppm` and slew-limited. The pre-T027 PI controller
//! (Kp ~ 4 ppm per frame of error, integrator only frozen at the clamp) turned such a step into
//! a +-500 ppm limit cycle at the loop's natural frequency (live v0.2 log; `tests/drift.rs`).

/// Controller tuning.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DriftConfig {
    /// Fill-error decay time constant of the phase correction (s).
    pub phase_tc_s: f64,
    /// Largest phase-correction rate on top of the drift estimate (ppm).
    pub max_phase_ppm: f64,
    /// Fill errors within this many frames cause no phase correction.
    pub deadband_frames: f64,
    /// Time constant of the fill-error low-pass feeding the phase correction (s).
    pub smoothing_s: f64,
    /// Drift measurement window (s).
    pub freq_window_s: f64,
    /// Time constant with which the drift estimate follows the measurements (s).
    pub freq_tc_s: f64,
    /// Correction clamp (ppm).
    pub max_correction_ppm: f64,
    /// Maximum correction change rate (ppm per second).
    pub max_slew_ppm_per_s: f64,
}

impl Default for DriftConfig {
    fn default() -> Self {
        Self {
            phase_tc_s: 8.0,
            max_phase_ppm: 150.0,
            deadband_frames: 3.0,
            smoothing_s: 3.0,
            freq_window_s: 2.0,
            freq_tc_s: 15.0,
            max_correction_ppm: 2000.0,
            max_slew_ppm_per_s: 200.0,
        }
    }
}

/// Number of window measurements the drift median is taken over.
const MEDIAN_OF: usize = 5;
/// Measurements taken directly (fast acquisition) after a cold start, before the drift
/// estimate switches to slow tracking with `freq_tc_s`.
const ACQUIRE: u32 = 3;
/// Windows in the long drift baseline.
const LONG: usize = 8;
/// Largest disagreement (fractional) between the long baseline and the short-window median
/// for the long baseline to be trusted.
const LONG_AGREE: f64 = 40e-6;

/// Controller state.
#[derive(Debug, Clone)]
pub struct DriftController {
    fs: f64,
    cfg: DriftConfig,
    kp: f64,
    // phase path
    err_f: f64,
    primed: bool,
    // frequency path
    freq: f64,
    freq_target: f64,
    /// Measurements taken since the last cold reset (acquisition while < ACQUIRE).
    acquired: u32,
    win_t: f64,
    win_err: f64,
    win_corr: f64,
    prev_mean_err: Option<f64>,
    meas: [f64; MEDIAN_OF],
    n_meas: usize,
    /// Per-window (mean error, mean correction), newest first, for the long baseline.
    hist: [(f64, f64); LONG + 1],
    n_hist: usize,
    // output
    correction: f64,
}

impl DriftController {
    /// Controller for an input (ring) rate of `fs_in` Hz.
    pub fn new(fs_in: f64, cfg: DriftConfig) -> Self {
        Self {
            fs: fs_in,
            cfg,
            kp: 1.0 / (fs_in * cfg.phase_tc_s.max(1e-3)),
            err_f: 0.0,
            primed: false,
            freq: 0.0,
            freq_target: 0.0,
            acquired: 0,
            win_t: 0.0,
            win_err: 0.0,
            win_corr: 0.0,
            prev_mean_err: None,
            meas: [0.0; MEDIAN_OF],
            n_meas: 0,
            hist: [(0.0, 0.0); LONG + 1],
            n_hist: 0,
            correction: 0.0,
        }
    }

    /// Forget the error history (after a resync / re-prime) but keep the learned drift.
    pub fn reset_error(&mut self) {
        self.primed = false;
        self.err_f = 0.0;
        self.win_t = 0.0;
        self.win_err = 0.0;
        self.win_corr = 0.0;
        self.prev_mean_err = None;
        self.n_meas = 0;
        self.n_hist = 0;
        self.freq_target = self.freq;
        self.correction = self.freq;
    }

    /// Forget everything (new device / sample rate).
    pub fn reset(&mut self) {
        self.freq = 0.0;
        self.acquired = 0;
        self.reset_error();
    }

    /// Current correction (fractional, e.g. 1e-4 = +100 ppm).
    pub fn correction(&self) -> f64 {
        self.correction
    }

    /// Current drift estimate (fractional).
    pub fn drift(&self) -> f64 {
        self.freq
    }

    /// Filtered fill error (frames).
    pub fn filtered_error(&self) -> f64 {
        self.err_f
    }

    /// Feed one measurement: `error` = fill - target (input frames), `dt` = seconds since the
    /// previous update. Returns the new correction. RT-safe.
    pub fn update(&mut self, error: f64, dt: f64) -> f64 {
        let cfg = &self.cfg;
        // Phase path: low-passed error, deadband, proportional, clamped.
        if !self.primed {
            self.primed = true;
            self.err_f = error;
        } else {
            let a = 1.0 - (-dt / cfg.smoothing_s.max(1e-6)).exp();
            self.err_f += a * (error - self.err_f);
        }
        let e = self.err_f;
        let e_db = e.signum() * (e.abs() - cfg.deadband_frames).max(0.0);
        let max_p = cfg.max_phase_ppm * 1e-6;
        let phase = (self.kp * e_db).clamp(-max_p, max_p);

        // Frequency path: box averages of the raw error and of the applied correction.
        self.win_t += dt;
        self.win_err += error * dt;
        self.win_corr += self.correction * dt;
        if self.win_t >= cfg.freq_window_s {
            let t = self.win_t;
            let mean_err = self.win_err / t;
            let mean_corr = self.win_corr / t;
            if let Some(prev) = self.prev_mean_err {
                // Window centres are `t` apart.
                let m = mean_corr + (mean_err - prev) / (self.fs * t);
                self.meas.copy_within(0..MEDIAN_OF - 1, 1);
                self.meas[0] = m;
                self.n_meas = (self.n_meas + 1).min(MEDIAN_OF);
                let med = median(&self.meas[..self.n_meas]);
                // Long baseline: the per-window fill noise cancels between consecutive
                // windows, so over LONG windows it is LONG times smaller. A fill step inside
                // the baseline corrupts it for LONG windows; then it disagrees with the
                // (step-robust) median and the median is used instead.
                self.hist.copy_within(0..LONG, 1);
                self.hist[0] = (mean_err, mean_corr);
                self.n_hist = (self.n_hist + 1).min(LONG + 1);
                let mut meas = med;
                if self.n_hist == LONG + 1 {
                    let corr = self.hist[..LONG].iter().map(|h| h.1).sum::<f64>() / LONG as f64;
                    let long = corr + (mean_err - self.hist[LONG].0) / (self.fs * t * LONG as f64);
                    if (long - med).abs() < LONG_AGREE {
                        meas = long;
                    }
                }
                if self.acquired < ACQUIRE {
                    // Acquisition after a cold start: take the median of what we have.
                    self.freq_target = med;
                    self.acquired += 1;
                } else {
                    let g = 1.0 - (-t / cfg.freq_tc_s.max(1e-6)).exp();
                    self.freq_target += g * (meas - self.freq_target);
                }
            }
            self.prev_mean_err = Some(mean_err);
            self.win_t = 0.0;
            self.win_err = 0.0;
            self.win_corr = 0.0;
        }
        // The estimate glides to its target over one window (no steps in the ratio).
        let g = 1.0 - (-dt / cfg.freq_window_s.max(1e-6)).exp();
        self.freq += g * (self.freq_target - self.freq);

        let max_c = cfg.max_correction_ppm * 1e-6;
        let want = (self.freq + phase).clamp(-max_c, max_c);
        let max_step = cfg.max_slew_ppm_per_s * 1e-6 * dt;
        self.correction += (want - self.correction).clamp(-max_step, max_step);
        self.correction
    }
}

/// Median of up to [`MEDIAN_OF`] values (no allocation).
fn median(v: &[f64]) -> f64 {
    let mut s = [0.0f64; MEDIAN_OF];
    let n = v.len().min(MEDIAN_OF);
    s[..n].copy_from_slice(&v[..n]);
    let s = &mut s[..n];
    s.sort_unstable_by(|a, b| a.total_cmp(b));
    if n == 0 {
        0.0
    } else if n % 2 == 1 {
        s[n / 2]
    } else {
        0.5 * (s[n / 2 - 1] + s[n / 2])
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Ideal plant: fill grows when the consumer is slow, so the correction must become +drift
    /// to consume faster; converges within 30 s with a small fill excursion.
    #[test]
    fn converges_on_constant_drift() {
        let fs = 48_000.0;
        let mut c = DriftController::new(fs, DriftConfig::default());
        let drift = 300e-6;
        let dt = 0.01;
        let mut fill_err = 0.0;
        let mut max_err: f64 = 0.0;
        let mut settled_at = 0.0;
        for i in 0..(300.0 / dt) as usize {
            let corr = c.update(fill_err, dt);
            fill_err += fs * (drift - corr) * dt;
            max_err = max_err.max(fill_err.abs());
            if (corr - drift).abs() > 20e-6 || fill_err.abs() > 48.0 {
                settled_at = i as f64 * dt;
            }
        }
        assert!((c.correction() - drift).abs() < 1e-7, "{}", c.correction());
        assert!(fill_err.abs() <= 3.5, "within the deadband: {fill_err}");
        assert!(max_err < 200.0, "{max_err}");
        assert!(settled_at < 30.0, "settled at {settled_at}");
    }

    /// A fill step (e.g. a whole extra driver buffer) is drained at the bounded phase rate,
    /// monotonically, without disturbing the drift estimate much and without overshoot.
    #[test]
    fn fill_step_drains_without_oscillation() {
        let fs = 48_000.0;
        let mut c = DriftController::new(fs, DriftConfig::default());
        let drift = -120e-6;
        let dt = 0.01;
        let mut fill_err = 0.0;
        for _ in 0..(60.0 / dt) as usize {
            let corr = c.update(fill_err, dt);
            fill_err += fs * (drift - corr) * dt;
        }
        fill_err += 512.0;
        let (mut cmin, mut cmax, mut min_err) = (f64::MAX, f64::MIN, f64::MAX);
        let mut max_drift_dev: f64 = 0.0;
        for _ in 0..(200.0 / dt) as usize {
            let corr = c.update(fill_err, dt);
            fill_err += fs * (drift - corr) * dt;
            cmin = cmin.min(corr);
            cmax = cmax.max(corr);
            min_err = min_err.min(fill_err);
            max_drift_dev = max_drift_dev.max((c.drift() - drift).abs());
        }
        assert!(fill_err.abs() < 3.5, "{fill_err}");
        assert!(min_err > -10.0, "overshoot {min_err}");
        assert!(cmax - drift <= 151e-6, "{cmax}");
        assert!(drift - cmin < 30e-6, "{cmin}");
        assert!(max_drift_dev < 30e-6, "{max_drift_dev}");
    }

    #[test]
    fn clamp_and_slew() {
        let mut c = DriftController::new(48_000.0, DriftConfig::default());
        let v = c.update(1e9, 0.01);
        assert!((v - 2e-6).abs() < 1e-12, "slew-limited: {v}");
        for _ in 0..10_000 {
            c.update(1e9, 0.01);
        }
        assert!(c.correction() <= 2000e-6 + 1e-12, "{}", c.correction());
    }

    #[test]
    fn median_small() {
        assert_eq!(median(&[3.0, 1.0, 2.0]), 2.0);
        assert_eq!(median(&[4.0, 1.0]), 2.5);
        assert_eq!(median(&[]), 0.0);
    }
}
