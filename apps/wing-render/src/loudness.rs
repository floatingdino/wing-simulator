//! Sample peak and ITU-R BS.1770-4 integrated loudness (LUFS) for the render report.
//!
//! K-weighting: the two BS.1770 biquads (high shelf ≈ +4 dB above 1.5 kHz, then the RLB high-pass
//! at 38 Hz), with coefficients derived for any sample rate from the analogue prototypes (the
//! formulation used by libebur128; at 48 kHz they equal the tabulated BS.1770 coefficients to
//! ~1e-8). Mean square per 100 ms sub-block, 400 ms gating blocks with 75 % overlap, absolute gate
//! −70 LUFS, relative gate −10 LU. Channel weights are 1 (mono and L/R; no surround channels here).
//! A mono tap is measured as one channel (no dual-mono +3 dB).

/// One biquad in direct form I, f64.
#[derive(Debug, Clone, Copy, Default)]
struct Biquad {
    b: [f64; 3],
    a: [f64; 2],
    x: [f64; 2],
    y: [f64; 2],
}

impl Biquad {
    #[inline]
    fn run(&mut self, x: f64) -> f64 {
        let y = self.b[0] * x + self.b[1] * self.x[0] + self.b[2] * self.x[1]
            - self.a[0] * self.y[0]
            - self.a[1] * self.y[1];
        self.x = [x, self.x[0]];
        self.y = [y, self.y[0]];
        y
    }
}

/// K-weighting filter pair for `fs`.
fn k_weighting(fs: f64) -> [Biquad; 2] {
    use std::f64::consts::PI;
    // stage 1: high shelf
    let f0 = 1_681.974_450_955_533;
    let g = 3.999_843_853_973_347;
    let q = 0.707_175_236_955_419_6;
    let k = (PI * f0 / fs).tan();
    let vh = 10f64.powf(g / 20.0);
    let vb = vh.powf(0.499_666_774_154_541_6);
    let a0 = 1.0 + k / q + k * k;
    let shelf = Biquad {
        b: [
            (vh + vb * k / q + k * k) / a0,
            2.0 * (k * k - vh) / a0,
            (vh - vb * k / q + k * k) / a0,
        ],
        a: [2.0 * (k * k - 1.0) / a0, (1.0 - k / q + k * k) / a0],
        ..Biquad::default()
    };
    // stage 2: RLB high-pass
    let f0 = 38.135_470_876_024_44;
    let q = 0.500_327_037_323_877_3;
    let k = (PI * f0 / fs).tan();
    let a0 = 1.0 + k / q + k * k;
    let hp = Biquad {
        b: [1.0, -2.0, 1.0],
        a: [2.0 * (k * k - 1.0) / a0, (1.0 - k / q + k * k) / a0],
        ..Biquad::default()
    };
    [shelf, hp]
}

/// Streaming peak + integrated-loudness meter for an interleaved signal.
#[derive(Debug, Clone)]
pub struct LoudnessMeter {
    channels: usize,
    filters: Vec<[Biquad; 2]>,
    /// Samples per 100 ms sub-block.
    sub_len: usize,
    sub_fill: usize,
    sub_energy: f64,
    /// Energies (channel-summed mean squares) of the completed sub-blocks.
    subs: Vec<f64>,
    peak: f32,
}

impl LoudnessMeter {
    pub fn new(channels: usize, sample_rate: u32) -> Self {
        Self {
            channels,
            filters: vec![k_weighting(sample_rate as f64); channels],
            sub_len: (sample_rate as usize / 10).max(1),
            sub_fill: 0,
            sub_energy: 0.0,
            subs: Vec::new(),
            peak: 0.0,
        }
    }

    /// Feed interleaved frames.
    pub fn push(&mut self, interleaved: &[f32]) {
        for frame in interleaved.chunks_exact(self.channels) {
            for (x, f) in frame.iter().zip(self.filters.iter_mut()) {
                self.peak = self.peak.max(x.abs());
                let s = f[0].run(*x as f64);
                let y = f[1].run(s);
                self.sub_energy += y * y;
            }
            self.sub_fill += 1;
            if self.sub_fill == self.sub_len {
                self.subs.push(self.sub_energy / self.sub_len as f64);
                self.sub_fill = 0;
                self.sub_energy = 0.0;
            }
        }
    }

    /// Sample peak in dBFS (−inf for silence).
    pub fn peak_dbfs(&self) -> f64 {
        20.0 * (self.peak as f64).log10()
    }

    /// Integrated loudness (LUFS); `None` when no 400 ms block passes the gates (too short or
    /// silent).
    pub fn integrated_lufs(&self) -> Option<f64> {
        let lufs = |e: f64| -0.691 + 10.0 * e.log10();
        let blocks: Vec<f64> = self
            .subs
            .windows(4)
            .map(|w| w.iter().sum::<f64>() / 4.0)
            .collect();
        let abs: Vec<f64> = blocks.into_iter().filter(|&e| lufs(e) > -70.0).collect();
        if abs.is_empty() {
            return None;
        }
        let rel = lufs(abs.iter().sum::<f64>() / abs.len() as f64) - 10.0;
        let gated: Vec<f64> = abs.into_iter().filter(|&e| lufs(e) > rel).collect();
        if gated.is_empty() {
            return None;
        }
        Some(lufs(gated.iter().sum::<f64>() / gated.len() as f64))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sine(fs: u32, secs: f64, f: f64, dbfs: f64, ch: usize) -> Vec<f32> {
        let a = 10f64.powf(dbfs / 20.0);
        let n = (fs as f64 * secs) as usize;
        (0..n * ch)
            .map(|i| {
                let t = (i / ch) as f64 / fs as f64;
                (a * (2.0 * std::f64::consts::PI * f * t).sin()) as f32
            })
            .collect()
    }

    #[test]
    fn coefficients_match_bs1770_table_at_48k() {
        let [s, h] = k_weighting(48_000.0);
        let close = |a: f64, b: f64| (a - b).abs() < 1e-7;
        assert!(close(s.b[0], 1.535_124_859_586_97));
        assert!(close(s.b[1], -2.691_696_189_406_38));
        assert!(close(s.b[2], 1.198_392_810_852_85));
        assert!(close(s.a[0], -1.690_659_293_182_41));
        assert!(close(s.a[1], 0.732_480_774_215_85));
        assert!(close(h.a[0], -1.990_047_454_833_98));
        assert!(close(h.a[1], 0.990_072_250_366_21));
    }

    #[test]
    fn ebu_3341_cases() {
        for fs in [44_100u32, 48_000] {
            // Case 1: stereo 1 kHz at -23 dBFS for 20 s -> -23.0 LUFS
            let mut m = LoudnessMeter::new(2, fs);
            m.push(&sine(fs, 20.0, 1000.0, -23.0, 2));
            let l = m.integrated_lufs().unwrap();
            assert!((l + 23.0).abs() < 0.1, "{fs}: {l}");
            assert!((m.peak_dbfs() + 23.0).abs() < 0.01);
            // Case 3: -36 / -23 / -36 dBFS (10 s / 60 s / 10 s) -> -23.0 (relative gate)
            let mut m = LoudnessMeter::new(2, fs);
            m.push(&sine(fs, 10.0, 1000.0, -36.0, 2));
            m.push(&sine(fs, 60.0, 1000.0, -23.0, 2));
            m.push(&sine(fs, 10.0, 1000.0, -36.0, 2));
            let l = m.integrated_lufs().unwrap();
            assert!((l + 23.0).abs() < 0.1, "{fs}: {l}");
            // Case 5-like: silence around the tone is gated out (absolute gate)
            let mut m = LoudnessMeter::new(2, fs);
            m.push(&vec![0.0; fs as usize * 20]);
            m.push(&sine(fs, 20.0, 1000.0, -23.0, 2));
            m.push(&vec![0.0; fs as usize * 20]);
            let l = m.integrated_lufs().unwrap();
            assert!((l + 23.0).abs() < 0.1, "{fs}: {l}");
        }
        // mono: one channel at 0 dBFS -> -3.01 LUFS
        let mut m = LoudnessMeter::new(1, 48_000);
        m.push(&sine(48_000, 5.0, 1000.0, 0.0, 1));
        let l = m.integrated_lufs().unwrap();
        assert!((l + 3.01).abs() < 0.1, "{l}");
        // silence / too short
        let mut m = LoudnessMeter::new(2, 48_000);
        m.push(&[0.0; 96_000]);
        assert_eq!(m.integrated_lufs(), None);
        assert!(m.peak_dbfs().is_infinite());
    }
}
