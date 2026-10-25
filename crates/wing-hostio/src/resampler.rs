//! Variable-ratio polyphase windowed-sinc resampler (pull model, RT-safe).
//!
//! Each output frame is a `taps`-point FIR over the input history, with the kernel taken from
//! a table of `phases + 1` Kaiser-windowed sinc phases and linearly interpolated between the
//! two nearest phases. The ratio (`step` = input frames per output frame) may change on every
//! call, which is what the drift controller needs. All buffers are allocated in [`Resampler::new`];
//! [`Resampler::next_frame`] never allocates.
//!
//! Why not rubato (the dsp-reuse recommendation): the device IO callback pulls a variable
//! number of output frames, and the input comes straight out of the SPSC ring frame by frame.
//! rubato's async resamplers work on fixed chunks, which would need an extra staging FIFO
//! (more latency, more state) and its API has churned between releases. A pull-driven
//! polyphase kernel is ~150 lines, has no dependencies, and measures < -110 dB THD+N (f32-limited) at
//! near-unity ratios and 44.1/48 kHz conversion (see the tests).

use std::f64::consts::PI;

/// Resampler parameters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ResamplerConfig {
    /// FIR length (even). Latency is `taps / 2` input frames.
    pub taps: usize,
    /// Number of kernel phases in the table.
    pub phases: usize,
    /// Kaiser window beta (beta 10 gives roughly 100 dB stopband attenuation).
    pub kaiser_beta: f64,
    /// Cutoff as a fraction of the lower Nyquist frequency.
    pub rolloff: f64,
}

impl Default for ResamplerConfig {
    fn default() -> Self {
        Self {
            taps: 64,
            phases: 512,
            kaiser_beta: 10.0,
            rolloff: 0.91,
        }
    }
}

/// Variable-ratio resampler for `channels` channels.
pub struct Resampler {
    channels: usize,
    taps: usize,
    phases: usize,
    /// `(phases + 1) * taps` kernel table, phase-major.
    table: Box<[f32]>,
    /// Per-channel double-written history (`2 * taps` each).
    hist: Box<[f32]>,
    /// Next write position in the history (0..taps).
    pos: usize,
    /// Fractional read position in [0, 1) between window samples `taps/2 - 1` and `taps/2`.
    frac: f64,
    /// Interpolated kernel for the current frame.
    weights: Box<[f32]>,
    /// Pulled input frame.
    input: Box<[f32]>,
    /// Current output frame.
    output: Box<[f32]>,
}

impl Resampler {
    /// Build a resampler whose anti-aliasing cutoff suits a nominal `step` (input/output
    /// rate ratio, > 1 when downsampling).
    pub fn new(channels: usize, nominal_step: f64, cfg: ResamplerConfig) -> Self {
        assert!(channels > 0);
        assert!(
            cfg.taps >= 4 && cfg.taps.is_multiple_of(2),
            "taps must be even and >= 4"
        );
        assert!(cfg.phases >= 1);
        let taps = cfg.taps;
        // Cutoff in cycles per input sample.
        let fc = 0.5 * cfg.rolloff * (1.0 / nominal_step).min(1.0);
        let half = taps as f64 / 2.0;
        let i0b = bessel_i0(cfg.kaiser_beta);
        let mut table = vec![0.0f32; (cfg.phases + 1) * taps];
        let mut row = vec![0.0f64; taps];
        for p in 0..=cfg.phases {
            let frac = p as f64 / cfg.phases as f64;
            for (j, r) in row.iter_mut().enumerate() {
                let t = j as f64 - (half - 1.0) - frac;
                let x = t / half;
                let w = if x.abs() >= 1.0 {
                    0.0
                } else {
                    bessel_i0(cfg.kaiser_beta * (1.0 - x * x).sqrt()) / i0b
                };
                *r = 2.0 * fc * sinc(2.0 * fc * t) * w;
            }
            // Normalise every phase to unity DC gain (removes phase-dependent DC ripple).
            let sum: f64 = row.iter().sum();
            for (j, r) in row.iter().enumerate() {
                table[p * taps + j] = (r / sum) as f32;
            }
        }
        Self {
            channels,
            taps,
            phases: cfg.phases,
            table: table.into(),
            hist: vec![0.0; channels * 2 * taps].into(),
            pos: 0,
            frac: 0.0,
            weights: vec![0.0; taps].into(),
            input: vec![0.0; channels].into(),
            output: vec![0.0; channels].into(),
        }
    }

    /// Number of channels.
    pub fn channels(&self) -> usize {
        self.channels
    }

    /// Group delay in input frames (excluding the fractional phase).
    pub fn latency_frames(&self) -> f64 {
        self.taps as f64 / 2.0 - 1.0
    }

    /// Fractional position in [0, 1): how far past the current input frame the next output
    /// frame lies (input frames).
    pub fn phase(&self) -> f64 {
        self.frac
    }

    /// Clear history (silence) and phase.
    pub fn reset(&mut self) {
        self.hist.fill(0.0);
        self.pos = 0;
        self.frac = 0.0;
    }

    /// Number of input frames that `out_frames` output frames will pull at `step` (closed
    /// form; may differ from the exact pull count by one at rounding boundaries, so callers
    /// checking availability should allow one extra frame).
    pub fn input_needed(&self, out_frames: usize, step: f64) -> u64 {
        (self.frac + out_frames as f64 * step).floor() as u64
    }

    /// Produce one output frame at ratio `step` (input frames per output frame). `pull`
    /// fills one input frame (one sample per channel); it is called `floor(frac + step)` times.
    /// Returns the output frame (one sample per channel).
    #[inline]
    pub fn next_frame(&mut self, step: f64, mut pull: impl FnMut(&mut [f32])) -> &[f32] {
        let taps = self.taps;
        // Interpolate the kernel for the current phase.
        let fp = self.frac * self.phases as f64;
        let p = (fp as usize).min(self.phases - 1);
        let a = (fp - p as f64) as f32;
        let r0 = &self.table[p * taps..(p + 1) * taps];
        let r1 = &self.table[(p + 1) * taps..(p + 2) * taps];
        for ((w, &x0), &x1) in self.weights.iter_mut().zip(r0).zip(r1) {
            *w = x0 + a * (x1 - x0);
        }
        for ch in 0..self.channels {
            let h = &self.hist[ch * 2 * taps + self.pos..ch * 2 * taps + self.pos + taps];
            let mut acc = 0.0f32;
            for (x, w) in h.iter().zip(self.weights.iter()) {
                acc += x * w;
            }
            self.output[ch] = acc;
        }
        self.frac += step;
        while self.frac >= 1.0 {
            self.frac -= 1.0;
            pull(&mut self.input);
            for ch in 0..self.channels {
                let base = ch * 2 * taps;
                self.hist[base + self.pos] = self.input[ch];
                self.hist[base + self.pos + taps] = self.input[ch];
            }
            self.pos += 1;
            if self.pos == taps {
                self.pos = 0;
            }
        }
        &self.output
    }
}

fn sinc(x: f64) -> f64 {
    if x.abs() < 1e-12 {
        1.0
    } else {
        (PI * x).sin() / (PI * x)
    }
}

/// Modified Bessel function of the first kind, order 0 (power series).
fn bessel_i0(x: f64) -> f64 {
    let mut sum = 1.0;
    let mut term = 1.0;
    let q = x * x / 4.0;
    for k in 1..64 {
        term *= q / (k * k) as f64;
        sum += term;
        if term < sum * 1e-17 {
            break;
        }
    }
    sum
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Resample a sine of `freq` at input rate `fs_in` with fixed `step`; return
    /// (amplitude, THD+N in dB) measured by least-squares fitting the expected output sine.
    fn measure(freq: f64, fs_in: f64, step: f64, cfg: ResamplerConfig) -> (f64, f64) {
        let mut r = Resampler::new(1, step, cfg);
        let mut n_in = 0u64;
        let n_out = 48_000usize;
        let mut out = Vec::with_capacity(n_out);
        for _ in 0..n_out {
            let y = r.next_frame(step, |f| {
                f[0] = (0.5 * (2.0 * PI * freq * n_in as f64 / fs_in).sin()) as f32;
                n_in += 1;
            })[0];
            out.push(y as f64);
        }
        // Expected output frequency in cycles per output sample.
        let w = 2.0 * PI * freq / fs_in * step;
        // Skip the startup transient, fit a*sin + b*cos + c.
        let skip = 4 * cfg.taps;
        let (mut ss, mut cc, mut sc, mut sy, mut cy) = (0.0, 0.0, 0.0, 0.0, 0.0);
        for (i, &y) in out.iter().enumerate().skip(skip) {
            let (s, c) = (w * i as f64).sin_cos();
            ss += s * s;
            cc += c * c;
            sc += s * c;
            sy += s * y;
            cy += c * y;
        }
        let det = ss * cc - sc * sc;
        let a = (sy * cc - cy * sc) / det;
        let b = (cy * ss - sy * sc) / det;
        let (mut sig, mut err) = (0.0, 0.0);
        for (i, &y) in out.iter().enumerate().skip(skip) {
            let (s, c) = (w * i as f64).sin_cos();
            let fit = a * s + b * c;
            sig += fit * fit;
            err += (y - fit) * (y - fit);
        }
        let amp = (a * a + b * b).sqrt();
        (amp / 0.5, 10.0 * (err / sig).log10())
    }

    #[test]
    fn thd_n_near_unity_ratio() {
        for &step in &[1.0, 1.0002, 0.9998, 1.001] {
            for &f in &[100.0, 1000.0, 5000.0, 15000.0] {
                let (_, thdn) = measure(f, 48_000.0, step, ResamplerConfig::default());
                assert!(thdn < -110.0, "step {step} f {f}: THD+N {thdn:.1} dB");
            }
        }
    }

    #[test]
    fn thd_n_48k_to_44k1() {
        let step = 48_000.0 / 44_100.0;
        for &f in &[100.0, 1000.0, 10000.0] {
            let (_, thdn) = measure(f, 48_000.0, step, ResamplerConfig::default());
            assert!(thdn < -110.0, "f {f}: THD+N {thdn:.1} dB");
        }
        // 44.1k -> 48k too.
        let (_, thdn) = measure(1000.0, 44_100.0, 1.0 / step, ResamplerConfig::default());
        assert!(thdn < -110.0, "44.1->48 THD+N {thdn:.1} dB");
    }

    #[test]
    fn passband_flat_and_stopband_rejected() {
        let cfg = ResamplerConfig::default();
        // Unity-ish ratio: flat to 20 kHz.
        for &f in &[20.0, 1000.0, 10_000.0, 18_000.0, 20_000.0] {
            let (g, _) = measure(f, 48_000.0, 1.0001, cfg);
            let db = 20.0 * g.log10();
            assert!(db.abs() < 0.05, "f {f}: {db:.3} dB");
        }
        // 48 -> 44.1: flat to 18 kHz, a 23 kHz tone (above the output Nyquist) must not alias.
        let step = 48_000.0 / 44_100.0;
        for &f in &[1000.0, 18_000.0] {
            let (g, _) = measure(f, 48_000.0, step, cfg);
            let db = 20.0 * g.log10();
            assert!(db.abs() < 0.05, "f {f}: {db:.3} dB");
        }
        let mut r = Resampler::new(1, step, cfg);
        let mut n = 0u64;
        let mut peak = 0.0f32;
        for i in 0..20_000 {
            let y = r.next_frame(step, |x| {
                x[0] = (2.0 * PI * 23_000.0 * n as f64 / 48_000.0).sin() as f32;
                n += 1;
            })[0];
            if i > 1000 {
                peak = peak.max(y.abs());
            }
        }
        let db = 20.0 * (peak as f64).log10();
        assert!(db < -80.0, "23 kHz leak {db:.1} dB");
    }

    #[test]
    fn input_needed_matches_pull_count() {
        let mut r = Resampler::new(2, 1.0, ResamplerConfig::default());
        for &(n, step) in &[
            (512usize, 1.0001),
            (33, 0.9999),
            (480, 48.0 / 44.1),
            (7, 1.0),
        ] {
            let want = r.input_needed(n, step);
            let mut got = 0u64;
            for _ in 0..n {
                r.next_frame(step, |_| got += 1);
            }
            assert!(want.abs_diff(got) <= 1, "{want} vs {got}");
        }
    }

    #[test]
    fn dc_passes_unity() {
        let mut r = Resampler::new(1, 1.0, ResamplerConfig::default());
        let mut y = 0.0;
        for _ in 0..1000 {
            y = r.next_frame(1.00037, |x| x[0] = 0.25)[0];
        }
        assert!((y - 0.25).abs() < 1e-6, "{y}");
    }
}
