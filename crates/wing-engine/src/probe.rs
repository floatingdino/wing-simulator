//! Model-slot probe (T088): a debug capture of what one live model slot of an input strip does.
//!
//! ```text
//! control: SlotProbeFeed::arm(strip, block) ──▶ RT: the matching strip's slot copies its input and output
//!          (mono (L+R)/2, PROBE_LEN samples, one shot) and, when full, a snapshot of the slot as the DSP
//!          sees it (model, wrapper on/mix/gain, effective wet, owns-mix, latency, model params) ──▶ DONE
//! control: SlotProbeFeed::take() ──▶ ProbeCapture ──▶ analyse(): in/out RMS, gain, nonlinear residual
//! ```
//!
//! * **RT cost**: one relaxed load per slot per block while idle; while armed, the one matching slot
//!   stores 2 x block samples (relaxed atomics). No locks, no allocation. Single producer: exactly
//!   one (strip, block) matches a request.
//! * **Analysis** ([`analyse`]): both signals are high-passed at [`PROBE_HP_HZ`] (removes DC and the
//!   sub-audible envelope products of even-order shapers), then the best linear FIR
//!   ([`PROBE_FIR`] taps, least squares) predicting the output from the input is removed. What is left
//!   is the part of the output no linear filter explains: harmonic + intermodulation distortion and
//!   noise. `residual_pct = 100 * rms(residual) / rms(out)` is THD+N for a sine and a (conservative)
//!   distortion measure on programme: distortion products that coincide with existing harmonics of a
//!   periodic input are partly absorbed by the fit.
//!
//! Used by the daemon's `probe` control command and `/wingsim/probe` OSC (`wing_sim::probe`).

use std::sync::atomic::{AtomicU32, AtomicUsize, Ordering::*};

/// Captured samples per signal (0.34 s at 48 kHz).
pub const PROBE_LEN: usize = 1 << 14;
/// Model parameters captured (slots 1..=PROBE_PARAMS).
pub const PROBE_PARAMS: usize = 32;
/// High-pass corner applied before the analysis (Hz).
pub const PROBE_HP_HZ: f64 = 40.0;
/// Linear-fit length (taps); covers model latencies (oversamplers ~40 samples) and short filters.
pub const PROBE_FIR: usize = 96;
/// Samples skipped at the start of the analysis (high-pass settling).
const SKIP: usize = 2048;

/// Model slot of an input strip.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum ProbeBlock {
    Gate = 0,
    Eq = 1,
    Dyn = 2,
}

impl ProbeBlock {
    pub fn parse(s: &str) -> Option<Self> {
        Some(match s {
            "gate" => Self::Gate,
            "eq" => Self::Eq,
            "dyn" => Self::Dyn,
            _ => return None,
        })
    }

    pub fn name(self) -> &'static str {
        match self {
            Self::Gate => "gate",
            Self::Eq => "eq",
            Self::Dyn => "dyn",
        }
    }
}

const IDLE: u32 = 0;
const ARMED: u32 = 1;
const DONE: u32 = 2;
const NO_REQ: u32 = u32::MAX;

/// The slot as the DSP saw it at the end of a capture.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SlotState {
    /// Selected WING model enum index (`None`: nothing selected).
    pub model_index: Option<u16>,
    /// The selected model has an implementation (else the slot passes audio through).
    pub has_instance: bool,
    /// Wrapper `on`, `mix` (%), `gain` (dB).
    pub on: bool,
    pub mix_pct: f32,
    pub gain_db: f32,
    /// Effective dry/wet weight `w` the slot applies (ramp value).
    pub wet: f32,
    /// The model's own MIX is the block MIX (block `mix` inert, T085).
    pub owns_mix: bool,
    /// Slot latency (samples).
    pub latency: usize,
    /// The model's parameter values by position (`params[0]` = `<slot>/1`) as the model holds them.
    pub params: Vec<Option<f32>>,
}

/// Shared probe state between control and RT (see the module docs).
pub struct SlotProbeFeed {
    request: AtomicU32,
    state: AtomicU32,
    pos: AtomicUsize,
    inb: Box<[AtomicU32]>,
    outb: Box<[AtomicU32]>,
    // slot snapshot
    model: AtomicU32,
    flags: AtomicU32,
    mix: AtomicU32,
    gain: AtomicU32,
    wet: AtomicU32,
    latency: AtomicU32,
    params: Box<[AtomicU32]>,
    sample_rate: AtomicU32,
}

impl std::fmt::Debug for SlotProbeFeed {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SlotProbeFeed")
            .field("request", &self.request.load(Relaxed))
            .field("state", &self.state.load(Relaxed))
            .finish()
    }
}

impl Default for SlotProbeFeed {
    fn default() -> Self {
        Self::new()
    }
}

/// A finished capture.
#[derive(Clone, Debug)]
pub struct ProbeCapture {
    pub input: Vec<f32>,
    pub output: Vec<f32>,
    pub slot: SlotState,
    pub sample_rate: u32,
}

const NONE_BITS: u32 = 0x7FC0_DEAD; // a NaN payload marking "no parameter"

impl SlotProbeFeed {
    /// Allocates (control thread).
    pub fn new() -> Self {
        let buf = || (0..PROBE_LEN).map(|_| AtomicU32::new(0)).collect();
        Self {
            request: AtomicU32::new(NO_REQ),
            state: AtomicU32::new(IDLE),
            pos: AtomicUsize::new(0),
            inb: buf(),
            outb: buf(),
            model: AtomicU32::new(u32::MAX),
            flags: AtomicU32::new(0),
            mix: AtomicU32::new(0),
            gain: AtomicU32::new(0),
            wet: AtomicU32::new(0),
            latency: AtomicU32::new(0),
            params: (0..PROBE_PARAMS)
                .map(|_| AtomicU32::new(NONE_BITS))
                .collect(),
            sample_rate: AtomicU32::new(0),
        }
    }

    pub fn shared() -> std::sync::Arc<Self> {
        std::sync::Arc::new(Self::new())
    }

    /// An audio session processes at `fs` (0 = none).
    pub fn set_sample_rate(&self, fs: u32) {
        self.sample_rate.store(fs, Relaxed);
    }

    pub fn sample_rate(&self) -> u32 {
        self.sample_rate.load(Relaxed)
    }

    /// Start a capture of input strip `strip` (flat index: ch 1..40 = 0..39, aux 1..8 = 40..47),
    /// slot `block` (control side). Restarts a running capture.
    pub fn arm(&self, strip: usize, block: ProbeBlock) {
        self.state.store(IDLE, Release);
        self.request
            .store((strip as u32) << 8 | block as u32, Relaxed);
        self.pos.store(0, Relaxed);
        self.state.store(ARMED, Release);
    }

    /// Stop capturing.
    pub fn disarm(&self) {
        self.state.store(IDLE, Release);
        self.request.store(NO_REQ, Relaxed);
    }

    /// Whether a capture finished.
    pub fn is_done(&self) -> bool {
        self.state.load(Acquire) == DONE
    }

    /// Samples captured so far.
    pub fn captured(&self) -> usize {
        self.pos.load(Relaxed)
    }

    /// The finished capture (control side, allocates); `None` while running.
    pub fn take(&self) -> Option<ProbeCapture> {
        if !self.is_done() {
            return None;
        }
        let read = |b: &[AtomicU32]| -> Vec<f32> {
            b.iter().map(|v| f32::from_bits(v.load(Relaxed))).collect()
        };
        let m = self.model.load(Relaxed);
        let fl = self.flags.load(Relaxed);
        let slot = SlotState {
            model_index: (m != u32::MAX).then_some(m as u16),
            has_instance: fl & 1 != 0,
            on: fl & 2 != 0,
            owns_mix: fl & 4 != 0,
            mix_pct: f32::from_bits(self.mix.load(Relaxed)),
            gain_db: f32::from_bits(self.gain.load(Relaxed)),
            wet: f32::from_bits(self.wet.load(Relaxed)),
            latency: self.latency.load(Relaxed) as usize,
            params: self
                .params
                .iter()
                .map(|p| {
                    let b = p.load(Relaxed);
                    (b != NONE_BITS).then(|| f32::from_bits(b))
                })
                .collect(),
        };
        let out = ProbeCapture {
            input: read(&self.inb),
            output: read(&self.outb),
            slot,
            sample_rate: self.sample_rate(),
        };
        self.disarm();
        Some(out)
    }

    /// RT: whether `(strip, block)` is being captured. One acquire load.
    #[inline]
    fn wants(&self, strip: usize, block: ProbeBlock) -> bool {
        self.state.load(Acquire) == ARMED
            && self.request.load(Relaxed) == ((strip as u32) << 8 | block as u32)
    }
}

/// A strip's view of the feed.
#[derive(Clone, Debug)]
pub struct SlotProbe {
    feed: std::sync::Arc<SlotProbeFeed>,
    strip: usize,
}

impl SlotProbe {
    pub fn new(feed: std::sync::Arc<SlotProbeFeed>, strip: usize) -> Self {
        Self { feed, strip }
    }

    /// Whether this strip's `block` slot is being captured this block. RT-safe.
    #[inline]
    pub fn wants(&self, block: ProbeBlock) -> bool {
        self.feed.wants(self.strip, block)
    }

    /// Record the slot input (before the slot processes in place). RT-safe.
    #[inline]
    pub fn input(&self, l: &[f32], r: &[f32]) {
        let p = self.feed.pos.load(Relaxed);
        for (i, (a, b)) in l.iter().zip(r).enumerate() {
            if let Some(c) = self.feed.inb.get(p + i) {
                c.store((0.5 * (a + b)).to_bits(), Relaxed);
            }
        }
    }

    /// Record the slot output and, once the capture is full, the slot snapshot. RT-safe.
    #[inline]
    pub fn output(&self, l: &[f32], r: &[f32], slot: &crate::params::strip::ModelSlot) {
        let f = &*self.feed;
        let p = f.pos.load(Relaxed);
        for (i, (a, b)) in l.iter().zip(r).enumerate() {
            if let Some(c) = f.outb.get(p + i) {
                c.store((0.5 * (a + b)).to_bits(), Relaxed);
            }
        }
        let np = (p + l.len()).min(PROBE_LEN);
        f.pos.store(np, Relaxed);
        if np < PROBE_LEN {
            return;
        }
        let (on, mix, gain) = slot.wrapper();
        let model = slot.model();
        f.model
            .store(slot.model_index().map_or(u32::MAX, u32::from), Relaxed);
        f.flags.store(
            slot.has_instance() as u32
                | (on as u32) << 1
                | (model.is_some_and(|m| m.owns_mix()) as u32) << 2,
            Relaxed,
        );
        f.mix.store(mix.to_bits(), Relaxed);
        f.gain.store(gain.to_bits(), Relaxed);
        f.wet.store(slot.wet_value().to_bits(), Relaxed);
        f.latency.store(slot.latency() as u32, Relaxed);
        for (k, c) in f.params.iter().enumerate() {
            let v = model.and_then(|m| m.param(k as u16));
            c.store(v.map_or(NONE_BITS, f32::to_bits), Relaxed);
        }
        f.state.store(DONE, Release);
    }
}

// ---- analysis ---------------------------------------------------------------------------------------------

/// What a slot did to its input (see the module docs).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SlotAnalysis {
    /// RMS of the slot input / output (dBFS, full-band) and their ratio (dB).
    pub in_rms_db: f64,
    pub out_rms_db: f64,
    pub gain_db: f64,
    /// Input sample peak (dBFS).
    pub in_peak_db: f64,
    /// Nonlinear residual relative to the (high-passed) output: THD+N on a sine, distortion on
    /// programme (dB and %).
    pub residual_db: f64,
    pub residual_pct: f64,
    /// Lag (samples, < [`PROBE_FIR`]) of the input/output cross-correlation peak (≈ the slot's latency on
    /// broadband input; ambiguous on strongly periodic input).
    pub lag: usize,
}

fn rms_db(x: &[f64]) -> f64 {
    let p = x.iter().map(|v| v * v).sum::<f64>() / x.len().max(1) as f64;
    10.0 * p.max(1e-30).log10()
}

/// Two cascaded one-pole high-passes at `fc` (f64).
fn highpass(x: &[f32], fs: f64, fc: f64) -> Vec<f64> {
    let a = (-2.0 * std::f64::consts::PI * fc / fs).exp();
    let mut y: Vec<f64> = x.iter().map(|&v| v as f64).collect();
    for _ in 0..2 {
        let (mut px, mut py) = (0.0f64, 0.0f64);
        for v in y.iter_mut() {
            let o = a * (py + *v - px);
            px = *v;
            py = o;
            *v = o;
        }
    }
    y
}

/// Solve `A h = b` for symmetric positive-definite `A` (n x n, row-major) by Cholesky. In place.
fn cholesky_solve(a: &mut [f64], b: &mut [f64], n: usize) -> bool {
    for j in 0..n {
        let mut d = a[j * n + j];
        for k in 0..j {
            d -= a[j * n + k] * a[j * n + k];
        }
        if d <= 0.0 {
            return false;
        }
        let d = d.sqrt();
        a[j * n + j] = d;
        for i in j + 1..n {
            let mut s = a[i * n + j];
            for k in 0..j {
                s -= a[i * n + k] * a[j * n + k];
            }
            a[i * n + j] = s / d;
        }
    }
    for i in 0..n {
        let mut s = b[i];
        for k in 0..i {
            s -= a[i * n + k] * b[k];
        }
        b[i] = s / a[i * n + i];
    }
    for i in (0..n).rev() {
        let mut s = b[i];
        for k in i + 1..n {
            s -= a[k * n + i] * b[k];
        }
        b[i] = s / a[i * n + i];
    }
    true
}

/// Analyse a slot capture: `x` = input, `y` = output (same length, same clock), sample rate `fs`.
/// `None` when the input is silent (nothing to measure).
pub fn analyse(x: &[f32], y: &[f32], fs: f64) -> Option<SlotAnalysis> {
    let n = x.len().min(y.len());
    if n < SKIP + 4 * PROBE_FIR {
        return None;
    }
    let raw = |v: &[f32]| -> Vec<f64> { v[SKIP..n].iter().map(|&s| s as f64).collect() };
    let (xr, yr) = (raw(x), raw(y));
    let in_rms_db = rms_db(&xr);
    let out_rms_db = rms_db(&yr);
    if in_rms_db < -120.0 {
        return None;
    }
    let in_peak_db = 20.0
        * xr.iter()
            .fold(0.0f64, |m, v| m.max(v.abs()))
            .max(1e-15)
            .log10();
    let xh = highpass(&x[..n], fs, PROBE_HP_HZ);
    let yh = highpass(&y[..n], fs, PROBE_HP_HZ);
    let (xh, yh) = (&xh[SKIP..], &yh[SKIP..]);
    let m = xh.len();
    let l = PROBE_FIR;
    // exact normal equations over n in [l, m): C[i][j] = sum x[n-i] x[n-j] (first row direct, the rest
    // by the diagonal recursion C[i+1][j+1] = C[i][j] + x[l-1-i] x[l-1-j] - x[m-1-i] x[m-1-j])
    let mut a = vec![0.0f64; l * l];
    let mut h = vec![0.0f64; l];
    for j in 0..l {
        let (mut c, mut p) = (0.0, 0.0);
        for n in l..m {
            c += xh[n] * xh[n - j];
            p += yh[n] * xh[n - j];
        }
        a[j] = c;
        h[j] = p;
    }
    for i in 0..l - 1 {
        for j in i..l - 1 {
            let v = a[i * l + j] + xh[l - 1 - i] * xh[l - 1 - j] - xh[m - 1 - i] * xh[m - 1 - j];
            a[(i + 1) * l + j + 1] = v;
        }
    }
    for i in 0..l {
        for j in 0..i {
            a[i * l + j] = a[j * l + i];
        }
    }
    // lag: peak of the input/output cross-correlation (the raw right-hand side)
    let lag = h
        .iter()
        .enumerate()
        .fold(
            (0, 0.0f64),
            |b, (i, v)| if v.abs() > b.1 { (i, v.abs()) } else { b },
        )
        .0;
    let r0 = a[0];
    for i in 0..l {
        a[i * l + i] += 1e-9 * r0 + 1e-30;
    }
    if !cholesky_solve(&mut a, &mut h, l) {
        return None;
    }
    let e: Vec<f64> = (l..m)
        .map(|i| {
            let p: f64 = (0..l).map(|k| h[k] * xh[i - k]).sum();
            yh[i] - p
        })
        .collect();
    let residual_db = rms_db(&e) - rms_db(&yh[l..]);
    Some(SlotAnalysis {
        in_rms_db,
        out_rms_db,
        gain_db: out_rms_db - in_rms_db,
        in_peak_db,
        residual_db,
        residual_pct: 100.0 * 10f64.powf(residual_db / 20.0),
        lag,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sine(f: f64, a: f64, n: usize) -> Vec<f32> {
        (0..n)
            .map(|i| (a * (2.0 * std::f64::consts::PI * f * i as f64 / 48e3).sin()) as f32)
            .collect()
    }

    #[test]
    fn linear_filters_and_delays_have_no_residual() {
        let mut seed = 0x2545_f491u32;
        let x: Vec<f32> = (0..PROBE_LEN)
            .map(|_| {
                seed ^= seed << 13;
                seed ^= seed >> 17;
                seed ^= seed << 5;
                (seed as f32 / u32::MAX as f32 - 0.5) * 0.2 * 12f32.sqrt()
            })
            .collect();
        // delay 39 + gain 0.5 + a one-pole low-pass
        let mut y = vec![0.0f32; x.len()];
        let mut z = 0.0f32;
        for i in 39..x.len() {
            z += 0.3 * (x[i - 39] - z);
            y[i] = 0.5 * z;
        }
        let a = analyse(&x, &y, 48e3).unwrap();
        assert!(a.residual_db < -60.0, "{a:?}");
        assert!((a.in_rms_db - (-14.0)).abs() < 0.3, "{a:?}");
        assert!((39..=42).contains(&a.lag), "{a:?}");
    }

    #[test]
    fn residual_is_thd_for_a_sine() {
        let x = sine(220.0, 0.25, PROBE_LEN);
        // 10 % third harmonic: y = x + 0.1 * a * sin(3wt) via the Chebyshev identity
        let y: Vec<f32> = x
            .iter()
            .map(|&v| {
                let s = v as f64 / 0.25;
                (v as f64 + 0.1 * 0.25 * (3.0 * s - 4.0 * s * s * s)) as f32
            })
            .collect();
        let a = analyse(&x, &y, 48e3).unwrap();
        assert!((a.residual_pct - 10.0).abs() < 0.3, "{a:?}");
        assert!(analyse(&vec![0.0; PROBE_LEN], &y, 48e3).is_none());
    }

    #[test]
    fn feed_one_shot_round_trip() {
        let feed = SlotProbeFeed::shared();
        let p = SlotProbe::new(feed.clone(), 20);
        let other = SlotProbe::new(feed.clone(), 3);
        assert!(!p.wants(ProbeBlock::Gate));
        feed.arm(20, ProbeBlock::Gate);
        assert!(p.wants(ProbeBlock::Gate) && !p.wants(ProbeBlock::Dyn));
        assert!(!other.wants(ProbeBlock::Gate));
        let slot = crate::params::strip::ModelSlot::from_group("ch/*/gate", 48e3, 64, |_, _| None);
        let blk = vec![0.25f32; 64];
        let mut n = 0;
        while p.wants(ProbeBlock::Gate) {
            p.input(&blk, &blk);
            p.output(&blk, &blk, &slot);
            n += 64;
        }
        assert_eq!(n, PROBE_LEN);
        let c = feed.take().unwrap();
        assert_eq!(c.input.len(), PROBE_LEN);
        assert!(c.output.iter().all(|&v| v == 0.25));
        assert!(feed.take().is_none());
        assert!(!p.wants(ProbeBlock::Gate));
    }
}
