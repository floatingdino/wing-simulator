//! Topology-preserving-transform state-variable filter (Zavalishin "The Art of VA Filter Design", Simper
//! "SvfLinearTrapOptimised2"). Trapezoidal integration = bilinear transform, so static responses equal the RBJ
//! designs (verified against the analog prototypes in tests), but the structure stays stable and click-free
//! under per-sample cutoff/Q modulation, which makes it the choice for swept/modulated filters.
//!
//! Each `set_*` computes `g = tan(pi f/fs)`, `k = 1/Q` and an output mix `m0*v0 + m1*v1 + m2*v2`
//! (input, band, low). `tan` per update; for audio-rate modulation callers can precompute `g` and use
//! [`Svf::set_g`].

use core::f64::consts::PI;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SvfMode {
    Lowpass,
    Highpass,
    Bandpass,
    Notch,
    Allpass,
    Peak,
    Bell,
    LowShelf,
    HighShelf,
}

/// Simultaneous outputs of one SVF tick.
#[derive(Clone, Copy, Debug)]
pub struct SvfOutputs {
    pub low: f32,
    pub band: f32,
    pub high: f32,
}

#[derive(Clone, Copy, Debug)]
pub struct Svf {
    a1: f32,
    a2: f32,
    a3: f32,
    k: f32,
    m0: f32,
    m1: f32,
    m2: f32,
    ic1: f32,
    ic2: f32,
}

impl Default for Svf {
    fn default() -> Self {
        let mut s = Self {
            a1: 0.0,
            a2: 0.0,
            a3: 0.0,
            k: 1.0,
            m0: 1.0,
            m1: 0.0,
            m2: 0.0,
            ic1: 0.0,
            ic2: 0.0,
        };
        s.set_g(0.1, 1.0);
        s
    }
}

impl Svf {
    pub fn new(mode: SvfMode, fs: f64, f0: f64, q: f64, gain_db: f64) -> Self {
        let mut s = Self::default();
        s.set(mode, fs, f0, q, gain_db);
        s
    }

    /// Set raw integrator gain `g = tan(pi f/fs)` and damping `k = 1/Q` (output mix unchanged).
    #[inline]
    pub fn set_g(&mut self, g: f32, k: f32) {
        self.k = k;
        self.a1 = 1.0 / (1.0 + g * (g + k));
        self.a2 = g * self.a1;
        self.a3 = g * self.a2;
    }

    /// Full (re)design. `gain_db` is used by Bell/LowShelf/HighShelf only. State is kept, so this can be
    /// called per sample for modulation.
    pub fn set(&mut self, mode: SvfMode, fs: f64, f0: f64, q: f64, gain_db: f64) {
        let f0 = f0.clamp(1e-3, fs * 0.499_9);
        let g = (PI * f0 / fs).tan();
        let mut k = 1.0 / q;
        let a = 10f64.powf(gain_db / 40.0);
        let (g, m0, m1, m2) = match mode {
            SvfMode::Lowpass => (g, 0.0, 0.0, 1.0),
            SvfMode::Highpass => (g, 1.0, -k, -1.0),
            SvfMode::Bandpass => (g, 0.0, k, 0.0), // 0 dB peak (RBJ constant-0dB BPF)
            SvfMode::Notch => (g, 1.0, -k, 0.0),
            SvfMode::Allpass => (g, 1.0, -2.0 * k, 0.0),
            SvfMode::Peak => (g, -1.0, k, 2.0), // low - high
            SvfMode::Bell => {
                k = 1.0 / (q * a);
                (g, 1.0, k * (a * a - 1.0), 0.0)
            }
            SvfMode::LowShelf => (g / a.sqrt(), 1.0, k * (a - 1.0), a * a - 1.0),
            SvfMode::HighShelf => (g * a.sqrt(), a * a, k * (1.0 - a) * a, 1.0 - a * a),
        };
        self.set_g(g as f32, k as f32);
        self.m0 = m0 as f32;
        self.m1 = m1 as f32;
        self.m2 = m2 as f32;
    }

    pub fn reset(&mut self) {
        self.ic1 = 0.0;
        self.ic2 = 0.0;
    }

    /// Zero tiny integrator states (portable denormal fallback; call once per block).
    pub fn flush_denormals(&mut self) {
        if self.ic1.abs() < 1e-15 {
            self.ic1 = 0.0;
        }
        if self.ic2.abs() < 1e-15 {
            self.ic2 = 0.0;
        }
    }

    /// One tick returning low/band/high simultaneously.
    #[inline(always)]
    pub fn tick_all(&mut self, v0: f32) -> SvfOutputs {
        let v3 = v0 - self.ic2;
        let v1 = self.a1 * self.ic1 + self.a2 * v3;
        let v2 = self.ic2 + self.a2 * self.ic1 + self.a3 * v3;
        self.ic1 = 2.0 * v1 - self.ic1;
        self.ic2 = 2.0 * v2 - self.ic2;
        SvfOutputs {
            low: v2,
            band: v1,
            high: v0 - self.k * v1 - v2,
        }
    }

    /// One tick returning the configured mode's output.
    #[inline(always)]
    pub fn process(&mut self, v0: f32) -> f32 {
        let v3 = v0 - self.ic2;
        let v1 = self.a1 * self.ic1 + self.a2 * v3;
        let v2 = self.ic2 + self.a2 * self.ic1 + self.a3 * v3;
        self.ic1 = 2.0 * v1 - self.ic1;
        self.ic2 = 2.0 * v2 - self.ic2;
        self.m0 * v0 + self.m1 * v1 + self.m2 * v2
    }

    #[inline]
    pub fn process_block(&mut self, buf: &mut [f32]) {
        for x in buf {
            *x = self.process(*x);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::biquad::Coeffs;
    use crate::dtft;

    const FS: f64 = 48_000.0;

    fn ir(s: &mut Svf, n: usize) -> Vec<f64> {
        (0..n)
            .map(|i| s.process(if i == 0 { 1.0 } else { 0.0 }) as f64)
            .collect()
    }

    #[test]
    fn svf_matches_rbj_responses() {
        for &(f0, q, g) in &[(200.0, 0.707, 6.0), (1500.0, 2.0, -9.0), (9000.0, 0.5, 12.0)] {
            let cases = [
                (SvfMode::Lowpass, Coeffs::lowpass(FS, f0, q)),
                (SvfMode::Highpass, Coeffs::highpass(FS, f0, q)),
                (SvfMode::Bandpass, Coeffs::bandpass(FS, f0, q)),
                (SvfMode::Notch, Coeffs::notch(FS, f0, q)),
                (SvfMode::Allpass, Coeffs::allpass(FS, f0, q)),
                (SvfMode::Bell, Coeffs::peak(FS, f0, q, g)),
                (SvfMode::LowShelf, Coeffs::low_shelf(FS, f0, q, g)),
                (SvfMode::HighShelf, Coeffs::high_shelf(FS, f0, q, g)),
            ];
            for (mode, c) in cases {
                let mut s = Svf::new(mode, FS, f0, q, g);
                let h = ir(&mut s, 16_384);
                for i in 0..60 {
                    let f = 30.0 * (600f64).powf(i as f64 / 59.0);
                    let w = 2.0 * PI * f / FS;
                    let want = c.response_w(w);
                    let got = dtft(&h, w);
                    if want.db() > -80.0 {
                        let e = (got.db() - want.db()).abs();
                        assert!(e < 0.01, "{mode:?} f0={f0} f={f}: err {e} dB");
                        let pe = crate::wrap_phase(got.arg() - want.arg()).abs();
                        assert!(pe < 1e-3, "{mode:?} f0={f0} f={f}: phase err {pe}");
                    }
                }
            }
        }
    }

    #[test]
    fn modulated_svf_stays_bounded() {
        let mut s = Svf::new(SvfMode::Lowpass, FS, 1000.0, 10.0, 0.0);
        let mut peak = 0.0f32;
        for n in 0..96_000 {
            let f = 1000.0 + 900.0 * ((n as f64) * 2.0 * PI * 5.0 / FS).sin() * 15.0;
            s.set(SvfMode::Lowpass, FS, f.abs() + 20.0, 10.0, 0.0);
            let y = s.process(((n as f32) * 0.05).sin());
            peak = peak.max(y.abs());
        }
        assert!(peak.is_finite() && peak < 50.0, "peak {peak}");
        let o = s.tick_all(0.0);
        assert!(o.low.is_finite() && o.band.is_finite() && o.high.is_finite());
    }
}
