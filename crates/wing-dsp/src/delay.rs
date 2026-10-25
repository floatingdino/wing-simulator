//! Fractional delay line with preallocated maximum length.
//!
//! Write one sample with [`DelayLine::push`], then read at a delay of `d` samples, where `d = 0` returns the
//! sample just pushed. Readers:
//! - [`DelayLine::read_int`]: integer delay.
//! - [`DelayLine::read_linear`]: 2-point linear interpolation. Error for a unit sine at `w` rad/sample is
//!   bounded by `w^2/8`.
//! - [`DelayLine::read_lagrange3`]: 4-point (3rd-order) Lagrange. Error bounded by `w^4 * (9/16) / 24`.
//! - [`AllpassReader`]: 1st-order Thiran allpass interpolation (flat magnitude, approximate phase; state per
//!   reader, delay must change slowly). Error for a unit sine is bounded by `0.06 * w^3` (measured). Fractional part kept in [0.5, 1.5) for a well-placed pole.
//!
//! Valid delays: `0..=max_delay` (linear), `1..=max_delay` (Lagrange needs one future/past neighbour on each
//! side, `d >= 1`), `0.5..=max_delay` (allpass). Out-of-range delays are clamped.

/// Preallocated power-of-two ring buffer delay line.
pub struct DelayLine {
    buf: Vec<f32>,
    mask: usize,
    write: usize,
    max_delay: usize,
}

impl DelayLine {
    /// Allocate for delays up to `max_delay` samples (plus interpolation headroom).
    pub fn new(max_delay: usize) -> Self {
        let size = (max_delay + 4).next_power_of_two();
        Self {
            buf: vec![0.0; size],
            mask: size - 1,
            write: 0,
            max_delay,
        }
    }

    pub fn max_delay(&self) -> usize {
        self.max_delay
    }

    pub fn reset(&mut self) {
        self.buf.iter_mut().for_each(|x| *x = 0.0);
    }

    #[inline]
    pub fn push(&mut self, x: f32) {
        self.write = (self.write + 1) & self.mask;
        self.buf[self.write] = x;
    }

    #[inline(always)]
    fn at(&self, d: usize) -> f32 {
        self.buf[self.write.wrapping_sub(d) & self.mask]
    }

    #[inline]
    pub fn read_int(&self, d: usize) -> f32 {
        self.at(d.min(self.max_delay))
    }

    #[inline]
    pub fn read_linear(&self, d: f32) -> f32 {
        let d = d.clamp(0.0, self.max_delay as f32);
        let i = d as usize;
        let f = d - i as f32;
        let a = self.at(i);
        let b = self.at(i + 1);
        a + (b - a) * f
    }

    /// 3rd-order Lagrange through samples at delays i-1, i, i+1, i+2 (i = floor(d)).
    #[inline]
    pub fn read_lagrange3(&self, d: f32) -> f32 {
        let d = d.clamp(1.0, self.max_delay as f32);
        let i = d as usize;
        let f = d - i as f32;
        let ym1 = self.at(i - 1);
        let y0 = self.at(i);
        let y1 = self.at(i + 1);
        let y2 = self.at(i + 2);
        // Nodes at t = -1, 0, 1, 2; evaluate at t = f.
        let fm1 = f + 1.0;
        let f1 = f - 1.0;
        let f2 = f - 2.0;
        -ym1 * f * f1 * f2 * (1.0 / 6.0) + y0 * fm1 * f1 * f2 * 0.5 - y1 * fm1 * f * f2 * 0.5
            + y2 * fm1 * f * f1 * (1.0 / 6.0)
    }

    /// Allpass-interpolated read using `r`'s state (see [`AllpassReader`]).
    #[inline]
    pub fn read_allpass(&self, r: &mut AllpassReader, d: f32) -> f32 {
        let d = d.clamp(0.5, self.max_delay as f32 - 1.0);
        let m = (d - 0.5) as usize; // integer part, so frac in [0.5, 1.5)
        let frac = d - m as f32;
        let eta = (1.0 - frac) / (1.0 + frac);
        let x0 = self.at(m);
        let x1 = self.at(m + 1);
        let y = eta * x0 + x1 - eta * r.y1;
        r.y1 = y;
        y
    }
}

/// State for one allpass-interpolated tap.
#[derive(Clone, Copy, Debug, Default)]
pub struct AllpassReader {
    y1: f32,
}

impl AllpassReader {
    pub fn reset(&mut self) {
        self.y1 = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(w: f64, d: f32, read: impl Fn(&DelayLine, &mut AllpassReader) -> f32) -> f64 {
        let mut dl = DelayLine::new(64);
        let mut ap = AllpassReader::default();
        let mut worst = 0.0f64;
        for n in 0..4000 {
            dl.push((w * n as f64).sin() as f32);
            let y = read(&dl, &mut ap) as f64;
            if n > 2000 {
                let want = (w * (n as f64 - d as f64)).sin();
                worst = worst.max((y - want).abs());
            }
        }
        worst
    }

    #[test]
    fn integer_delay_exact() {
        let mut dl = DelayLine::new(10);
        for i in 0..20 {
            dl.push(i as f32);
        }
        assert_eq!(dl.read_int(0), 19.0);
        assert_eq!(dl.read_int(7), 12.0);
        assert_eq!(dl.read_int(100), 9.0); // clamped to max
    }

    #[test]
    fn interpolation_error_bounds() {
        for &f_hz in &[100.0, 1000.0, 4000.0] {
            let w = 2.0 * core::f64::consts::PI * f_hz / 48_000.0;
            for &d in &[3.25f32, 10.5, 17.9] {
                let lin = run(w, d, |dl, _| dl.read_linear(d));
                let lin_bound = w * w / 8.0 + 1e-6;
                assert!(lin <= lin_bound, "linear f={f_hz} d={d}: {lin} > {lin_bound}");
                let lag = run(w, d, |dl, _| dl.read_lagrange3(d));
                let lag_bound = w.powi(4) * (9.0 / 16.0) / 24.0 + 1e-6;
                assert!(lag <= lag_bound, "lagrange f={f_hz} d={d}: {lag} > {lag_bound}");
                assert!(lag <= lin + 1e-6);
                // Thiran phase-delay error grows ~w^3; bound measured: < 2e-3 up to 4 kHz.
                let ap = run(w, d, |dl, r| dl.read_allpass(r, d));
                // Thiran phase-delay error grows ~w^3 (measured max ratio 0.056).
                let ap_bound = 0.06 * w.powi(3) + 1e-6;
                assert!(ap <= ap_bound, "allpass f={f_hz} d={d}: {ap} > {ap_bound}");
            }
        }
    }
}
