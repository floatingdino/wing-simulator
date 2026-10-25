//! Parameter smoothing (control -> RT de-zippering).
//!
//! - [`OnePoleSmoother`]: exponential approach, time constant `tau` (reaches 63.2% of a step after `tau`).
//!   Per-sample [`OnePoleSmoother::tick`], block fill [`OnePoleSmoother::fill`], or per-block
//!   [`OnePoleSmoother::advance`] (closed form, one value per block).
//! - [`LinearRamp`]: reaches the target exactly after a fixed number of samples. Per-sample
//!   [`LinearRamp::tick`], block fill, or per-block [`LinearRamp::advance`].

/// One-pole coefficient for time constant `tau_s` seconds at `fs` Hz: `exp(-1/(tau*fs))`.
/// `tau_s <= 0` yields 0 (no smoothing).
#[inline]
pub fn one_pole_coeff(tau_s: f64, fs: f64) -> f64 {
    if tau_s <= 0.0 {
        0.0
    } else {
        (-1.0 / (tau_s * fs)).exp()
    }
}

/// Exponential smoother: `y = target + c * (y - target)`. State is f64 so long time constants converge all the
/// way to the target (an f32 one-pole stalls ~1e-5 short of it once the per-step change drops below half an
/// ULP).
#[derive(Clone, Copy, Debug)]
pub struct OnePoleSmoother {
    value: f64,
    target: f64,
    coeff: f64,
}

impl OnePoleSmoother {
    pub fn new(initial: f32, tau_s: f64, fs: f64) -> Self {
        Self {
            value: initial as f64,
            target: initial as f64,
            coeff: one_pole_coeff(tau_s, fs),
        }
    }

    pub fn set_time(&mut self, tau_s: f64, fs: f64) {
        self.coeff = one_pole_coeff(tau_s, fs);
    }

    #[inline]
    pub fn set_target(&mut self, target: f32) {
        self.target = target as f64;
    }

    /// Jump to `v` immediately (no smoothing).
    pub fn reset(&mut self, v: f32) {
        self.value = v as f64;
        self.target = v as f64;
    }

    #[inline]
    pub fn value(&self) -> f32 {
        self.value as f32
    }

    #[inline]
    pub fn target(&self) -> f32 {
        self.target as f32
    }

    /// True once within `eps` of the target (callers may then skip per-sample work).
    #[inline]
    pub fn is_settled(&self, eps: f32) -> bool {
        (self.value - self.target).abs() <= eps as f64
    }

    #[inline]
    pub fn tick(&mut self) -> f32 {
        self.value = self.target + (self.value - self.target) * self.coeff;
        self.value as f32
    }

    /// Fill `out` with consecutive smoothed values.
    #[inline]
    pub fn fill(&mut self, out: &mut [f32]) {
        for o in out {
            *o = self.tick();
        }
    }

    /// Advance `n` samples at once (per-block smoothing); returns the new value.
    #[inline]
    pub fn advance(&mut self, n: usize) -> f32 {
        self.value = self.target + (self.value - self.target) * self.coeff.powi(n as i32);
        self.value as f32
    }
}

/// Linear ramp to a target over a fixed number of samples.
#[derive(Clone, Copy, Debug)]
pub struct LinearRamp {
    value: f32,
    target: f32,
    step: f32,
    remaining: u32,
    ramp_len: u32,
}

impl LinearRamp {
    /// `ramp_len` samples per transition (0 = jump).
    pub fn new(initial: f32, ramp_len: u32) -> Self {
        Self {
            value: initial,
            target: initial,
            step: 0.0,
            remaining: 0,
            ramp_len,
        }
    }

    pub fn set_ramp_len(&mut self, ramp_len: u32) {
        self.ramp_len = ramp_len;
    }

    /// Start a new ramp from the current value.
    pub fn set_target(&mut self, target: f32) {
        self.target = target;
        if self.ramp_len == 0 {
            self.value = target;
            self.remaining = 0;
        } else {
            self.step = (target - self.value) / self.ramp_len as f32;
            self.remaining = self.ramp_len;
        }
    }

    pub fn reset(&mut self, v: f32) {
        self.value = v;
        self.target = v;
        self.remaining = 0;
    }

    #[inline]
    pub fn value(&self) -> f32 {
        self.value
    }

    #[inline]
    pub fn is_ramping(&self) -> bool {
        self.remaining > 0
    }

    #[inline]
    pub fn tick(&mut self) -> f32 {
        if self.remaining > 0 {
            self.remaining -= 1;
            self.value = if self.remaining == 0 {
                self.target
            } else {
                self.value + self.step
            };
        }
        self.value
    }

    #[inline]
    pub fn fill(&mut self, out: &mut [f32]) {
        for o in out {
            *o = self.tick();
        }
    }

    /// Advance `n` samples at once; returns the new value.
    #[inline]
    pub fn advance(&mut self, n: u32) -> f32 {
        if n >= self.remaining {
            self.remaining = 0;
            self.value = self.target;
        } else {
            self.remaining -= n;
            self.value += self.step * n as f32;
        }
        self.value
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn one_pole_step_reaches_63_percent_at_tau() {
        let fs = 48_000.0;
        let tau = 0.01; // 480 samples
        let mut s = OnePoleSmoother::new(0.0, tau, fs);
        s.set_target(1.0);
        let mut v = 0.0;
        for _ in 0..480 {
            v = s.tick();
        }
        assert!((v - (1.0 - (-1.0f32).exp())).abs() < 1e-4, "v={v}");
        // per-block advance agrees with per-sample
        let mut b = OnePoleSmoother::new(0.0, tau, fs);
        b.set_target(1.0);
        for _ in 0..(480 / 32) {
            b.advance(32);
        }
        assert!((b.value() - v).abs() < 1e-4);
        // settles
        for _ in 0..48_000 {
            s.tick();
        }
        assert!(s.is_settled(1e-6));
    }

    #[test]
    fn one_pole_fill_monotonic() {
        let mut s = OnePoleSmoother::new(1.0, 0.002, 48_000.0);
        s.set_target(-1.0);
        let mut buf = [0.0; 256];
        s.fill(&mut buf);
        assert!(buf.windows(2).all(|w| w[1] <= w[0]));
        assert!(buf[255] > -1.0);
    }

    #[test]
    fn linear_ramp_exact() {
        let mut r = LinearRamp::new(0.0, 100);
        r.set_target(1.0);
        let mut buf = [0.0; 100];
        r.fill(&mut buf);
        assert!((buf[49] - 0.5).abs() < 1e-5);
        assert_eq!(buf[99], 1.0);
        assert!(!r.is_ramping());
        assert_eq!(r.tick(), 1.0);

        let mut b = LinearRamp::new(2.0, 64);
        b.set_target(-2.0);
        assert!((b.advance(32) - 0.0).abs() < 1e-5);
        assert_eq!(b.advance(1000), -2.0);

        let mut j = LinearRamp::new(0.0, 0);
        j.set_target(3.0);
        assert_eq!(j.value(), 3.0);
    }

    #[test]
    fn retarget_mid_ramp() {
        let mut r = LinearRamp::new(0.0, 10);
        r.set_target(10.0);
        r.advance(5);
        r.set_target(0.0);
        let mut last = 0.0;
        for _ in 0..10 {
            last = r.tick();
        }
        assert_eq!(last, 0.0);
    }
}
