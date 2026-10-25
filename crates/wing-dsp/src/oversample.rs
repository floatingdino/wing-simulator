//! 2x / 4x oversampling with polyphase half-band FIR filters (linear phase).
//!
//! Half-band FIR of length `N = 4K - 1` (Kaiser-windowed sinc, designed in f64 at construction): every
//! second tap except the centre (0.5) is zero, so each 2x stage costs `2K` multiply-adds per base-rate sample
//! for upsampling and `2K + 1` for downsampling (polyphase: the zero-stuffed and discarded samples are never
//! computed).
//!
//! Stage specs (at fs = 48 kHz; all scale with fs):
//! - Stage A (base <-> 2x): K = 20 (79 taps), Kaiser beta 10.0. Passband ripple < 0.001 dB to 20 kHz,
//!   stopband >= 96 dB from 28 kHz (i.e. images/aliases of 0-20 kHz content are attenuated >= 96 dB).
//! - Stage B (2x <-> 4x): K = 7 (27 taps), Kaiser beta 10.0. Passband ripple < 0.001 dB to 20 kHz,
//!   stopband >= 96 dB from 76 kHz (the first image of the 0-20 kHz band at 4x).
//!
//! Latency (up + down round trip, in base-rate samples), reported by `latency()`:
//! - [`Oversampler2x`]: `2*K_A - 1` = **39** samples (integer).
//! - [`Oversampler4x`]: `(2*K_A - 1) + (2*K_B - 1)/2` = 39 + 6.5 = **45.5** samples. A half-band stage running
//!   at 4x always contributes a half-sample at base rate; the graph's delay compensation must handle the
//!   fractional part (or compensate in the dry path with a fractional delay).
//!
//! Allocation: tap tables, histories and the internal high-rate block buffers are allocated in `new()`.

use core::f64::consts::PI;

const K_A: usize = 20;
const K_B: usize = 7;
const BETA: f64 = 10.0;

/// Zeroth-order modified Bessel function of the first kind (series).
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

/// Full half-band impulse response of length `4k - 1` (centre tap 0.5, DC gain 1).
pub fn halfband_taps(k: usize, beta: f64) -> Vec<f64> {
    let n = 4 * k - 1;
    let c = (n - 1) / 2;
    let i0b = bessel_i0(beta);
    let mut h: Vec<f64> = (0..n)
        .map(|i| {
            let m = i as f64 - c as f64;
            if m == 0.0 {
                return 0.5;
            }
            if (i as isize - c as isize) % 2 == 0 {
                return 0.0;
            }
            let sinc = (PI * m / 2.0).sin() / (PI * m);
            let r = m / c as f64;
            let w = bessel_i0(beta * (1.0 - r * r).max(0.0).sqrt()) / i0b;
            sinc * w
        })
        .collect();
    // Normalise side taps so they sum to exactly 0.5 (unity DC gain, exact half-band symmetry).
    let side: f64 = h
        .iter()
        .enumerate()
        .filter(|&(i, _)| i != c)
        .map(|(_, v)| v)
        .sum();
    for (i, v) in h.iter_mut().enumerate() {
        if i != c {
            *v *= 0.5 / side;
        }
    }
    h
}

/// Newest-first history with a mirrored buffer so the window is always a contiguous slice.
struct History {
    buf: Vec<f32>,
    len: usize,
    pos: usize,
}

impl History {
    fn new(len: usize) -> Self {
        Self {
            buf: vec![0.0; 2 * len],
            len,
            pos: 0,
        }
    }
    #[inline(always)]
    fn push(&mut self, x: f32) {
        self.pos = if self.pos == 0 {
            self.len - 1
        } else {
            self.pos - 1
        };
        self.buf[self.pos] = x;
        self.buf[self.pos + self.len] = x;
    }
    /// `[x[m], x[m-1], ..., x[m-len+1]]`.
    #[inline(always)]
    fn window(&self) -> &[f32] {
        &self.buf[self.pos..self.pos + self.len]
    }
    fn reset(&mut self) {
        self.buf.iter_mut().for_each(|x| *x = 0.0);
    }
}

#[inline(always)]
fn dot(a: &[f32], b: &[f32]) -> f32 {
    // 4 accumulators so the reduction vectorises without reassociation flags.
    let mut acc = [0.0f32; 4];
    let chunks = a.len() / 4;
    for i in 0..chunks {
        for l in 0..4 {
            acc[l] += a[4 * i + l] * b[4 * i + l];
        }
    }
    let mut s = (acc[0] + acc[1]) + (acc[2] + acc[3]);
    for i in chunks * 4..a.len() {
        s += a[i] * b[i];
    }
    s
}

/// One polyphase half-band 2x stage (independent up and down state).
struct HalfbandStage {
    k: usize,
    /// Even-phase taps h[2j], j = 0..2K (the non-zero side taps).
    he: Vec<f32>,
    he_up: Vec<f32>, // 2 * he (upsampler gain compensation)
    up: History,
    down_even: History,
    down_odd: History,
}

impl HalfbandStage {
    fn new(k: usize) -> Self {
        let h = halfband_taps(k, BETA);
        let he: Vec<f32> = (0..2 * k).map(|j| h[2 * j] as f32).collect();
        let he_up = he.iter().map(|v| 2.0 * v).collect();
        Self {
            k,
            he,
            he_up,
            up: History::new(2 * k),
            down_even: History::new(2 * k),
            down_odd: History::new(k + 1),
        }
    }

    fn reset(&mut self) {
        self.up.reset();
        self.down_even.reset();
        self.down_odd.reset();
    }

    /// `out.len() == 2 * input.len()`.
    #[inline]
    fn upsample(&mut self, input: &[f32], out: &mut [f32]) {
        debug_assert_eq!(out.len(), 2 * input.len());
        let k = self.k;
        for (&x, o) in input.iter().zip(out.chunks_exact_mut(2)) {
            self.up.push(x);
            let w = self.up.window();
            o[0] = dot(&self.he_up, w);
            o[1] = w[k - 1];
        }
    }

    /// `input.len() == 2 * out.len()`.
    #[inline]
    fn downsample(&mut self, input: &[f32], out: &mut [f32]) {
        debug_assert_eq!(input.len(), 2 * out.len());
        let k = self.k;
        for (pair, o) in input.chunks_exact(2).zip(out.iter_mut()) {
            self.down_even.push(pair[0]);
            self.down_odd.push(pair[1]);
            *o = dot(&self.he, self.down_even.window()) + 0.5 * self.down_odd.window()[k];
        }
    }

    /// Round-trip (up + down) latency in samples at this stage's *input* rate.
    fn latency(&self) -> usize {
        2 * self.k - 1
    }
}

/// 2x oversampler. Latency: see module docs ([`Oversampler2x::latency`]).
pub struct Oversampler2x {
    stage: HalfbandStage,
    hi: Vec<f32>,
    max_block: usize,
}

impl Oversampler2x {
    /// `max_block`: largest base-rate block passed to [`Oversampler2x::process`].
    pub fn new(max_block: usize) -> Self {
        Self {
            stage: HalfbandStage::new(K_A),
            hi: vec![0.0; 2 * max_block],
            max_block,
        }
    }

    pub fn reset(&mut self) {
        self.stage.reset();
    }

    /// Round-trip latency in base-rate samples (39).
    pub fn latency(&self) -> f64 {
        self.stage.latency() as f64
    }

    /// Upsample `input` into `out` (`out.len() == 2 * input.len()`).
    pub fn upsample(&mut self, input: &[f32], out: &mut [f32]) {
        assert_eq!(out.len(), 2 * input.len());
        self.stage.upsample(input, out);
    }

    /// Downsample `input` into `out` (`input.len() == 2 * out.len()`).
    pub fn downsample(&mut self, input: &[f32], out: &mut [f32]) {
        assert_eq!(input.len(), 2 * out.len());
        self.stage.downsample(input, out);
    }

    /// Upsample `buf`, run `f` on the 2x block, downsample back into `buf` (in place).
    pub fn process(&mut self, buf: &mut [f32], mut f: impl FnMut(&mut [f32])) {
        for chunk in buf.chunks_mut(self.max_block) {
            let hi = &mut self.hi[..2 * chunk.len()];
            self.stage.upsample(chunk, hi);
            f(hi);
            self.stage.downsample(hi, chunk);
        }
    }
}

/// 4x oversampler (two cascaded half-band stages). Latency: see module docs ([`Oversampler4x::latency`]).
pub struct Oversampler4x {
    a: HalfbandStage,
    b: HalfbandStage,
    mid: Vec<f32>,
    hi: Vec<f32>,
    max_block: usize,
}

impl Oversampler4x {
    pub fn new(max_block: usize) -> Self {
        Self {
            a: HalfbandStage::new(K_A),
            b: HalfbandStage::new(K_B),
            mid: vec![0.0; 2 * max_block],
            hi: vec![0.0; 4 * max_block],
            max_block,
        }
    }

    pub fn reset(&mut self) {
        self.a.reset();
        self.b.reset();
    }

    /// Round-trip latency in base-rate samples (45.5).
    pub fn latency(&self) -> f64 {
        self.a.latency() as f64 + self.b.latency() as f64 / 2.0
    }

    /// Upsample `input` (len <= max_block) into `out` (`out.len() == 4 * input.len()`).
    pub fn upsample(&mut self, input: &[f32], out: &mut [f32]) {
        assert!(input.len() <= self.max_block && out.len() == 4 * input.len());
        let mid = &mut self.mid[..2 * input.len()];
        self.a.upsample(input, mid);
        self.b.upsample(mid, out);
    }

    /// Downsample `input` into `out` (`input.len() == 4 * out.len()`, out.len() <= max_block).
    pub fn downsample(&mut self, input: &[f32], out: &mut [f32]) {
        assert!(out.len() <= self.max_block && input.len() == 4 * out.len());
        let mid = &mut self.mid[..2 * out.len()];
        self.b.downsample(input, mid);
        self.a.downsample(mid, out);
    }

    /// Upsample `buf`, run `f` on the 4x block, downsample back into `buf` (in place).
    pub fn process(&mut self, buf: &mut [f32], mut f: impl FnMut(&mut [f32])) {
        for chunk in buf.chunks_mut(self.max_block) {
            let n = chunk.len();
            let mid = &mut self.mid[..2 * n];
            let hi = &mut self.hi[..4 * n];
            self.a.upsample(chunk, mid);
            self.b.upsample(mid, hi);
            f(hi);
            self.b.downsample(hi, mid);
            self.a.downsample(mid, chunk);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dtft;

    const FS: f64 = 48_000.0;

    fn mag_db(h: &[f64], f: f64, rate: f64) -> f64 {
        dtft(h, 2.0 * PI * f / rate).db()
    }

    #[test]
    fn halfband_designs_meet_spec() {
        let checks = [
            // (K, rate, pass edge, stop edge)
            (K_A, 2.0 * FS, 20_000.0, 28_000.0),
            (K_B, 4.0 * FS, 20_000.0, 76_000.0),
        ];
        for (k, rate, fp, fstop) in checks {
            let h = halfband_taps(k, BETA);
            let mut ripple = 0.0f64;
            for i in 0..=400 {
                let f = fp * i as f64 / 400.0;
                ripple = ripple.max(mag_db(&h, f, rate).abs());
            }
            let mut stop = f64::NEG_INFINITY;
            for i in 0..=400 {
                let f = fstop + (rate / 2.0 - fstop) * i as f64 / 400.0;
                stop = stop.max(mag_db(&h, f, rate));
            }
            assert!(ripple < 0.001, "K={k} passband ripple {ripple} dB");
            assert!(stop < -96.0, "K={k} stopband {stop} dB");
        }
    }

    fn sine(f: f64, n: usize) -> Vec<f32> {
        (0..n)
            .map(|i| (2.0 * PI * f * i as f64 / FS).sin() as f32)
            .collect()
    }

    /// Goertzel-style single-bin magnitude (amplitude units) of `x` at `f` for sample rate `rate`.
    fn tone_amp(x: &[f32], f: f64, rate: f64) -> f64 {
        let h: Vec<f64> = x.iter().map(|&v| v as f64).collect();
        2.0 * dtft(&h, 2.0 * PI * f / rate).norm() / x.len() as f64
    }

    #[test]
    fn roundtrip_latency_and_passband() {
        let n = 4800;
        for (f, tol) in [(1000.0, 1e-4), (15_000.0, 1e-4)] {
            let x = sine(f, n);
            let mut y2 = x.clone();
            let mut o2 = Oversampler2x::new(64);
            o2.process(&mut y2, |_| {});
            let mut y4 = x.clone();
            let mut o4 = Oversampler4x::new(64);
            o4.process(&mut y4, |_| {});
            for (y, lat) in [(&y2, o2.latency()), (&y4, o4.latency())] {
                let mut worst = 0.0f64;
                for (i, &v) in y.iter().enumerate().skip(200) {
                    let want = (2.0 * PI * f * (i as f64 - lat) / FS).sin();
                    worst = worst.max((v as f64 - want).abs());
                }
                assert!(worst < tol, "f={f} lat={lat} worst={worst}");
            }
        }
        assert_eq!(Oversampler2x::new(1).latency(), 39.0);
        assert_eq!(Oversampler4x::new(1).latency(), 45.5);
    }

    #[test]
    fn upsampler_image_rejection() {
        // 10 kHz tone upsampled 2x: image at 96k - 10k = 86k folds to 38 kHz in the 96 kHz domain.
        let x = sine(10_000.0, 9600);
        let mut o = Oversampler2x::new(9600);
        let mut hi = vec![0.0; 2 * x.len()];
        o.upsample(&x, &mut hi);
        let tail = &hi[480..]; // 18720 samples: whole cycles of every probed tone
        let want = tone_amp(tail, 10_000.0, 2.0 * FS);
        let image = tone_amp(tail, 38_000.0, 2.0 * FS);
        assert!((want - 1.0).abs() < 1e-3, "passband amp {want}");
        assert!(20.0 * (image / want).log10() < -90.0, "image {image}");

        // 4x: stage A leaves a residual image at 38k; stage B zero-stuffing mirrors 10k/38k about 96k to
        // 86k/58k. Check all three positions in the 192 kHz domain.
        let mut o4 = Oversampler4x::new(9600);
        let mut hi4 = vec![0.0; 4 * x.len()];
        o4.upsample(&x, &mut hi4);
        let tail = &hi4[960..];
        let want = tone_amp(tail, 10_000.0, 4.0 * FS);
        assert!((want - 1.0).abs() < 1e-3);
        for img in [38_000.0, 58_000.0, 86_000.0] {
            let a = tone_amp(tail, img, 4.0 * FS);
            assert!(20.0 * (a / want).log10() < -90.0, "img {img}: {a}");
        }
    }

    #[test]
    fn downsampler_alias_rejection() {
        // 30 kHz at 96 kHz aliases to 18 kHz at 48 kHz: must be attenuated by the stage-A stopband.
        let n = 19_200;
        let hi: Vec<f32> = (0..n)
            .map(|i| (2.0 * PI * 30_000.0 * i as f64 / (2.0 * FS)).sin() as f32)
            .collect();
        let mut out = vec![0.0; n / 2];
        let mut o = Oversampler2x::new(n);
        o.downsample(&hi, &mut out);
        let alias = tone_amp(&out[400..], 18_000.0, FS);
        assert!(20.0 * alias.log10() < -90.0, "alias {alias}");
    }
}
