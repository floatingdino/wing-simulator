//! Shared algorithmic reverb core (T05R) for HALL, ROOM, CHAMBER, CONCERT and AMBI.
//!
//! ```text
//!  in L/R --> pre-delay line --+--> ER tap bank (8 taps/side, window = size x spread, envelope = shape) --+--> ER out
//!                              |                                                                         |
//!                              +--> echo / reflection taps (recirculating for ROOM, plain for CHAMBER/  --+--> echo out
//!                                   CONCERT)                                                             |
//!                                                                                  ER + echo x to_tail  v
//!                                                    4 series Schroeder allpass input diffusers per side
//!                                                                                                        v
//!       +------------------------- N-line FDN (N = 8 or 16), per sample: -----------------------------------+
//!       |  y_i  = line_i read at  base_i x size + LFO_i   (3rd-order Lagrange interpolation)          |
//!       |  f_i  = g_mid,i * LowShelf_i(g_lo/g_mid @ xover) * HighShelf_i(g_hi/g_mid @ damp) (y_i)  (3-band T60)|
//!       |  a_i  = in-loop Schroeder allpass_i(f_i)                                       (density, lossless)|
//!       |  tail L/R = c_L . a , c_R . a                       (two orthogonal sign vectors: decorrelation)  |
//!       |  line_i <- (H_N a)_i + b_L,i in_L + b_R,i in_R      (H_N = normalised Hadamard, fast WHT)         |
//!       +---------------------------------------------------------------------------------------------------+
//!  wet = ER x er_level + echo + tail x tail_level (CONCERT: + modulated chorus of the tail) -> LO CUT -> HI CUT
//! ```
//!
//! Design sources (algorithms only, no code copied; all permissive / public):
//! - Feedback delay network with a unitary feedback matrix and per-line absorbent gains
//!   `g_i = 10^(-3 L_i / (fs T60))` (L_i = loop length of line i in samples) so every path decays at the same
//!   rate: J.-M. Jot and A. Chaigne, "Digital delay networks for designing artificial reverberators", AES 90th
//!   convention (1991). Frequency-dependent T60 by making `g_i` a filter: here the mid gain times a cascade of
//!   two RBJ shelves (slope 1, monotone, minimum phase) per line, low shelf at the model's crossover to the
//!   bass-multiplied T60 and high shelf at `damp` to the HF T60. A cascade (not a sum of LP/HP branches) keeps
//!   the magnitude monotone between the band gains, so `|H_i| <= max(g)` < 1 (stable) and the T60 is accurate
//!   away from the corners. (A parallel `g_mid + dg LP + dg HP` form was tried first and rejected: the branch
//!   phases make the T60 non-monotone, +10 % at 250 Hz with damping at 1.5 kHz.)
//! - Hadamard mixing via the fast Walsh-Hadamard transform, input diffusion by allpass chains, modulated delay
//!   reads: as described in G. Smith (Signalsmith Audio), "Let's write a reverb" (ADC 2021, MIT-licensed example
//!   code) and J. Dattorro, "Effect Design Part 1: Reverberator and Other Filters", JAES 45(9), 1997 (input
//!   diffusers, in-loop allpasses, modulated delays). Modulated reads use 3rd-order Lagrange interpolation
//!   (stateless, so no switching transient when the LFO or a size glide crosses a sample boundary, unlike
//!   Dattorro's allpass interpolation; its small HF loss only shortens the T60 above ~10 kHz).
//! - Lexicon-style SHAPE / SPREAD (envelope of the reverb build-up) and pre-echoes: behaviour per the Lexicon
//!   480L / PCM manuals' descriptions, implemented here as a tap bank whose gain envelope and time window feed
//!   the tail (our own design, [I]).
//!
//! RT-safety: every buffer is allocated in [`ReverbCore::prepare`]; `process`, `set_params` and `reset` never
//! allocate. SIMD: the FDN state is `[f32; N]` lane arrays (N = 8/16) processed in lane loops that LLVM
//! vectorises (2 or 4 NEON f32x4 per op); the delay memory is frame-interleaved (`Vec<[f32; N]>`), so the
//! write of all lines is one contiguous store. Denormals: values below `1e-18` are flushed on every write into
//! a recursive structure and filter states are flushed once per block, so silence decays to exact zeros.
//!
//! Click-free parameter changes: every time-like value (pre-delay, size, ER window) and every level glides with a
//! one-pole [`GLIDE_S`] at block rate and is ramped linearly per sample inside the block (ER tap and line gains
//! too); decay / damping / filter values glide the same way and their coefficients are recomputed per block while
//! moving (output filters are TPT SVFs, stable under modulation). Echo / reflection times (up to 1.2 s) change
//! by a [`XFADE_S`] two-tap crossfade instead (no pitch sweep).

// Lane loops index several parallel `[f32; N]` arrays by line number; explicit indices keep them vectorisable.
#![allow(clippy::needless_range_loop)]

use wing_dsp::biquad::Coeffs;
use wing_dsp::delay::DelayLine;
use wing_dsp::svf::{Svf, SvfMode};

/// Early-reflection taps per side.
pub const ER_TAPS: usize = 8;
/// Input diffuser stages per side.
pub const DIFFUSERS: usize = 4;
/// Glide time constant of every smoothed parameter (s).
pub const GLIDE_S: f32 = 0.08;
/// Crossfade time of an echo / reflection time change (s).
pub const XFADE_S: f32 = 0.03;
/// Values below this are flushed to zero in recursive paths.
const FLUSH: f32 = 1e-18;
/// Upper bound of any per-pass loop gain (stability margin; corresponds to T60 > 400 s for a 30 ms line).
const MAX_GAIN: f32 = 0.9995;
/// `ln(1000)`: T60 is a 60 dB (x1000 amplitude) decay.
const LN_1000: f32 = 6.907_755;

/// ER tap positions inside the window (0..1), per side.
const ER_POS: [[f32; ER_TAPS]; 2] = [
    [0.0, 0.093, 0.205, 0.326, 0.452, 0.583, 0.741, 0.913],
    [0.037, 0.141, 0.262, 0.377, 0.509, 0.664, 0.802, 0.968],
];
/// ER tap signs per side (whitens the early pattern).
const ER_SIGN: [[f32; ER_TAPS]; 2] = [
    [1.0, -1.0, 1.0, 1.0, -1.0, 1.0, -1.0, -1.0],
    [1.0, 1.0, -1.0, 1.0, -1.0, -1.0, 1.0, -1.0],
];
/// Right-side diffuser length multipliers (left uses the character's lengths as-is).
const DIFF_R: [f32; DIFFUSERS] = [1.113, 0.927, 1.071, 0.953];
/// LFO rate multipliers per line (cycled), keep lines from moving in lock-step.
const LFO_SPREAD: [f32; 16] = [
    1.0, 0.73, 1.29, 0.87, 1.13, 0.61, 1.41, 0.94, 0.79, 1.21, 0.67, 1.07, 1.35, 0.83, 1.17, 0.71,
];

/// Static per-model character (delay geometry, crossover, colour). All times at size scale 1.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Character {
    /// Shortest / longest FDN line (ms); the others are spread geometrically in between.
    pub line_ms: (f32, f32),
    /// Largest size scale the model can request (sets the allocation).
    pub max_scale: f32,
    /// In-loop allpass delays, shortest / longest (ms; fixed, not size-scaled).
    pub loop_ap_ms: (f32, f32),
    /// In-loop allpass gain at diffusion 1 (half of it at diffusion 0).
    pub loop_ap_gain: f32,
    /// Input diffuser delays (ms, left side).
    pub diffuser_ms: [f32; DIFFUSERS],
    /// Largest input diffuser gain (at diffusion 1).
    pub diffuser_gain: f32,
    /// Bass-multiplier crossover (Hz).
    pub xover_hz: f32,
    /// High-frequency T60 above the damping frequency, relative to the mid T60.
    pub hf_ratio: f32,
    /// LFO depth (ms) at `mod_depth = 1`.
    pub mod_depth_ms: f32,
    /// ER window at size scale 1 (ms) at spread 0 / spread 1.
    pub er_window_ms: (f32, f32),
    /// Longest echo / reflection time (s); 0 = no echo stage.
    pub max_echo_s: f32,
    /// Echoes recirculate (ROOM `efl/efr` feedback) instead of being plain taps of the input.
    pub echo_feedback: bool,
    /// Output gain of the tail.
    pub tail_gain: f32,
}

/// One echo / reflection tap.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Echo {
    /// Delay from the input (ms).
    pub time_ms: f32,
    /// Recirculation gain (-1..1, only with [`Character::echo_feedback`]).
    pub feedback: f32,
    /// Output level (linear).
    pub level: f32,
}

/// Engineering-unit parameters of the core (the models map WING parameters onto these).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CoreParams {
    pub pre_ms: f32,
    /// Size scale (multiplies line lengths and the ER window), `0 < scale <= max_scale`.
    pub scale: f32,
    /// Mid-band T60 (s).
    pub decay_s: f32,
    /// LF T60 = `decay_s * bass_mult` below the crossover.
    pub bass_mult: f32,
    /// Above this frequency the T60 falls to `decay_s * hf_ratio`.
    pub damp_hz: f32,
    pub lc_hz: f32,
    pub hc_hz: f32,
    /// Build-up envelope, 0 = front-loaded (fast attack, decaying), ~0.35 = flat, 1 = rising (slow build).
    pub shape: f32,
    /// Build-up duration 0..1 (ER window from the character's spread-0 to spread-1 length).
    pub spread: f32,
    /// Input diffusion 0..1.
    pub diffusion: f32,
    pub mod_rate_hz: f32,
    /// LFO depth multiplier of [`Character::mod_depth_ms`].
    pub mod_depth: f32,
    pub er_level: f32,
    pub tail_level: f32,
    pub echo: [Echo; 2],
    /// Share of the echo signal that is also fed into the tail.
    pub echo_to_tail: f32,
    /// Chorus on the tail output 0..1 (CONCERT `crs`).
    pub chorus: f32,
}

impl Default for CoreParams {
    fn default() -> Self {
        Self {
            pre_ms: 20.0,
            scale: 1.0,
            decay_s: 2.0,
            bass_mult: 1.0,
            damp_hz: 6000.0,
            lc_hz: 20.0,
            hc_hz: 20_000.0,
            shape: 0.35,
            spread: 0.5,
            diffusion: 0.8,
            mod_rate_hz: 0.5,
            mod_depth: 1.0,
            er_level: 0.3,
            tail_level: 1.0,
            echo: [Echo::default(); 2],
            echo_to_tail: 0.0,
            chorus: 0.0,
        }
    }
}

/// One-pole glide evaluated at block rate.
#[derive(Clone, Copy, Debug, Default)]
struct Glide {
    cur: f32,
    tgt: f32,
}

impl Glide {
    /// Advance by one block with coefficient `a`; returns (start, end).
    #[inline]
    fn step(&mut self, a: f32) -> (f32, f32) {
        let s = self.cur;
        let d = self.tgt - self.cur;
        if d.abs() <= 1e-6 * self.tgt.abs().max(1e-3) {
            self.cur = self.tgt;
        } else {
            self.cur += a * d;
        }
        (s, self.cur)
    }

    fn snap(&mut self) {
        self.cur = self.tgt;
    }
}

/// Echo / reflection read position that changes by crossfading two taps ([`XFADE_S`]) instead of gliding:
/// a 1.2 s echo time jump would otherwise read the line at many times normal speed (pitch sweep, aliasing).
#[derive(Clone, Copy, Debug, Default)]
struct XfadeTap {
    cur: f32,
    next: f32,
    tgt: f32,
    /// Crossfade progress 0..1 (0 = idle on `cur`).
    x: f32,
}

impl XfadeTap {
    /// Advance one sample; returns (current, next, progress).
    #[inline(always)]
    fn next(&mut self, inc: f32) -> (f32, f32, f32) {
        if self.x > 0.0 {
            self.x += inc;
            if self.x >= 1.0 {
                self.cur = self.next;
                self.x = 0.0;
            }
        } else if self.tgt != self.cur {
            self.next = self.tgt;
            self.x = inc;
        }
        (self.cur, self.next, self.x)
    }

    fn snap(&mut self) {
        self.cur = self.tgt;
        self.next = self.tgt;
        self.x = 0.0;
    }
}

/// `N` independent TDF-II biquads in structure-of-arrays layout (vectorised lane loop).
#[derive(Clone, Copy, Debug)]
struct LaneBiquads<const N: usize> {
    b0: [f32; N],
    b1: [f32; N],
    b2: [f32; N],
    a1: [f32; N],
    a2: [f32; N],
    s1: [f32; N],
    s2: [f32; N],
}

impl<const N: usize> Default for LaneBiquads<N> {
    fn default() -> Self {
        Self {
            b0: [1.0; N],
            b1: [0.0; N],
            b2: [0.0; N],
            a1: [0.0; N],
            a2: [0.0; N],
            s1: [0.0; N],
            s2: [0.0; N],
        }
    }
}

impl<const N: usize> LaneBiquads<N> {
    fn set(&mut self, i: usize, c: Coeffs) {
        self.b0[i] = c.b0 as f32;
        self.b1[i] = c.b1 as f32;
        self.b2[i] = c.b2 as f32;
        self.a1[i] = c.a1 as f32;
        self.a2[i] = c.a2 as f32;
    }

    #[inline(always)]
    fn tick(&mut self, x: &mut [f32; N]) {
        for i in 0..N {
            let v = x[i];
            let y = self.b0[i] * v + self.s1[i];
            self.s1[i] = self.b1[i] * v - self.a1[i] * y + self.s2[i];
            self.s2[i] = self.b2[i] * v - self.a2[i] * y;
            x[i] = y;
        }
    }

    fn reset(&mut self) {
        self.s1 = [0.0; N];
        self.s2 = [0.0; N];
    }

    fn flush(&mut self) {
        for i in 0..N {
            self.s1[i] = flush(self.s1[i]);
            self.s2[i] = flush(self.s2[i]);
        }
    }
}

/// Deterministic +-1 pattern (hash bit) used for the injection / output vectors.
const fn sign(seed: u32, i: usize) -> f32 {
    let mut x = seed.wrapping_mul(0x9E37_79B9) ^ (i as u32).wrapping_mul(0x85EB_CA6B);
    x ^= x >> 15;
    x = x.wrapping_mul(0x2C1B_3C6D);
    x ^= x >> 12;
    if x & 1 == 0 {
        1.0
    } else {
        -1.0
    }
}

#[inline(always)]
fn flush(x: f32) -> f32 {
    if x.abs() < FLUSH {
        0.0
    } else {
        x
    }
}

/// Linear-interpolated read `d` samples back from write index `w` (d = 0: newest) of a power-of-two ring.
#[inline(always)]
fn tap(buf: &[f32], mask: usize, w: usize, d: f32) -> f32 {
    let i = d as usize;
    let f = d - i as f32;
    let a = buf[w.wrapping_sub(i) & mask];
    let b = buf[w.wrapping_sub(i + 1) & mask];
    a + (b - a) * f
}

/// In-place normalised fast Walsh-Hadamard transform (orthogonal, `H H = I`).
#[inline(always)]
fn hadamard<const N: usize>(x: &mut [f32; N]) {
    let mut h = 1;
    while h < N {
        let mut i = 0;
        while i < N {
            for j in i..i + h {
                let (a, b) = (x[j], x[j + h]);
                x[j] = a + b;
                x[j + h] = a - b;
            }
            i += 2 * h;
        }
        h *= 2;
    }
    let k = 1.0 / (N as f32).sqrt();
    for v in x.iter_mut() {
        *v *= k;
    }
}

/// The reverb core; `N` = FDN lines (power of two, 8 or 16). Outputs the wet signal only.
pub struct ReverbCore<const N: usize> {
    ch: Character,
    p: CoreParams,
    fs: f32,
    /// Next block jumps to the targets (after `prepare`/`reset`).
    fresh: bool,
    dirty: bool,

    // ---- input stage: pre-delay ring (also the ER and plain echo source)
    pre: [Vec<f32>; 2],
    pre_mask: usize,
    pre_w: usize,
    pre_d: Glide,
    er_win: Glide,
    shape: Glide,
    er_g: [[f32; ER_TAPS]; 2],
    echo_d: [XfadeTap; 2],
    echo_lvl: [Glide; 2],
    echo_fb: [Glide; 2],
    echo_line: [DelayLine; 2],
    diff: [[DelayLine; DIFFUSERS]; 2],
    diff_len: [[usize; DIFFUSERS]; 2],
    diffusion: Glide,

    // ---- FDN
    buf: Vec<[f32; N]>,
    mask: usize,
    w: usize,
    base_ms: [f32; N],
    scale: Glide,
    ap_buf: Vec<[f32; N]>,
    ap_mask: usize,
    ap_w: usize,
    ap_len: [usize; N],
    /// Per-line absorbent filter: gain, low shelf (bass mult), high shelf (damping).
    gm: [f32; N],
    ls: LaneBiquads<N>,
    hs: LaneBiquads<N>,
    decay: Glide,
    mult: Glide,
    damp: Glide,
    /// Decay / damping / size values the gains were last computed for.
    gain_key: [f32; 4],
    lfo_c: [f32; N],
    lfo_s: [f32; N],
    rot_c: [f32; N],
    rot_s: [f32; N],
    mod_rate: Glide,
    mod_depth: Glide,
    rate_key: f32,
    in_l: [f32; N],
    in_r: [f32; N],
    out_l: [f32; N],
    out_r: [f32; N],

    // ---- output
    er_lvl: Glide,
    tail_lvl: Glide,
    to_tail: Glide,
    chorus: Glide,
    chorus_line: [DelayLine; 2],
    chorus_ph: [f32; 2],
    lc: Glide,
    hc: Glide,
    filt_key: [f32; 2],
    /// Output LO CUT / HI CUT: TPT state-variable filters (click-free under cutoff sweeps).
    out_hp: [Svf; 2],
    out_lp: [Svf; 2],
}

impl<const N: usize> ReverbCore<N> {
    /// Unprepared core (call [`ReverbCore::prepare`] before processing).
    pub fn new(ch: Character) -> Self {
        assert!(N.is_power_of_two() && (4..=16).contains(&N));
        let mut base_ms = [0.0f32; N];
        let (lo, hi) = ch.line_ms;
        for (i, b) in base_ms.iter_mut().enumerate() {
            // geometric spread, small irrational jitter, interleaved order (short/long alternate)
            let k = if i % 2 == 0 { i / 2 } else { N - 1 - i / 2 };
            let t = k as f32 / (N - 1) as f32;
            *b = lo * (hi / lo).powf(t) * (1.0 + 0.031 * ((i as f32 * 2.399).sin()));
        }
        let norm = 1.0 / (N as f32).sqrt();
        let mut in_l = [0.0; N];
        let mut in_r = [0.0; N];
        let mut out_l = [0.0; N];
        let mut out_r = [0.0; N];
        for i in 0..N {
            in_l[i] = sign(11, i) * norm;
            in_r[i] = sign(23, i) * norm;
            out_l[i] = sign(37, i) * norm;
            out_r[i] = sign(53, i) * norm;
        }
        Self {
            ch,
            p: CoreParams::default(),
            fs: 48_000.0,
            fresh: true,
            dirty: true,
            pre: [vec![0.0; 1], vec![0.0; 1]],
            pre_mask: 0,
            pre_w: 0,
            pre_d: Glide::default(),
            er_win: Glide::default(),
            shape: Glide::default(),
            er_g: [[0.0; ER_TAPS]; 2],
            echo_d: [XfadeTap::default(); 2],
            echo_lvl: [Glide::default(); 2],
            echo_fb: [Glide::default(); 2],
            echo_line: [DelayLine::new(1), DelayLine::new(1)],
            diff: core::array::from_fn(|_| core::array::from_fn(|_| DelayLine::new(1))),
            diff_len: [[1; DIFFUSERS]; 2],
            diffusion: Glide::default(),
            buf: vec![[0.0; N]; 1],
            mask: 0,
            w: 0,
            base_ms,
            scale: Glide::default(),
            ap_buf: vec![[0.0; N]; 1],
            ap_mask: 0,
            ap_w: 0,
            ap_len: [1; N],
            gm: [0.0; N],
            ls: LaneBiquads::default(),
            hs: LaneBiquads::default(),
            decay: Glide::default(),
            mult: Glide::default(),
            damp: Glide::default(),
            gain_key: [f32::NAN; 4],
            lfo_c: [1.0; N],
            lfo_s: [0.0; N],
            rot_c: [1.0; N],
            rot_s: [0.0; N],
            mod_rate: Glide::default(),
            mod_depth: Glide::default(),
            rate_key: f32::NAN,
            in_l,
            in_r,
            out_l,
            out_r,
            er_lvl: Glide::default(),
            tail_lvl: Glide::default(),
            to_tail: Glide::default(),
            chorus: Glide::default(),
            chorus_line: [DelayLine::new(1), DelayLine::new(1)],
            chorus_ph: [0.0, 1.7],
            lc: Glide::default(),
            hc: Glide::default(),
            filt_key: [f32::NAN; 2],
            out_hp: [Svf::default(); 2],
            out_lp: [Svf::default(); 2],
        }
    }

    pub fn character(&self) -> &Character {
        &self.ch
    }

    pub fn params(&self) -> &CoreParams {
        &self.p
    }

    /// Allocate every buffer for `sample_rate` and reset. Off the RT thread.
    pub fn prepare(&mut self, sample_rate: f64) {
        self.fs = sample_rate as f32;
        let ms = sample_rate as f32 * 0.001;
        let ch = self.ch;
        // pre-delay ring: pre-delay + ER window + plain echo taps
        let pre_max = (200.0 + ch.er_window_ms.1 * ch.max_scale + 5.0) * ms
            + if ch.echo_feedback {
                0.0
            } else {
                ch.max_echo_s * 1000.0 * ms
            };
        let pre_len = (pre_max as usize + 8).next_power_of_two();
        self.pre = [vec![0.0; pre_len], vec![0.0; pre_len]];
        self.pre_mask = pre_len - 1;
        let echo_len = if ch.echo_feedback {
            (ch.max_echo_s * sample_rate as f32) as usize + 4
        } else {
            1
        };
        self.echo_line = [DelayLine::new(echo_len), DelayLine::new(echo_len)];
        for c in 0..2 {
            for j in 0..DIFFUSERS {
                let mul = if c == 0 { 1.0 } else { DIFF_R[j] };
                let n = ((ch.diffuser_ms[j] * mul * ms) as usize).max(1);
                self.diff_len[c][j] = n;
                self.diff[c][j] = DelayLine::new(n);
            }
        }
        // FDN lines: longest base x max scale + modulation headroom
        let max_line = self.base_ms.iter().fold(0f32, |a, &b| a.max(b)) * ch.max_scale * ms
            + ch.mod_depth_ms * 2.0 * ms
            + 8.0;
        let frames = (max_line as usize).next_power_of_two();
        self.buf = vec![[0.0; N]; frames];
        self.mask = frames - 1;
        let (alo, ahi) = ch.loop_ap_ms;
        for i in 0..N {
            let k = (i * 5 + 3) % N; // decorrelate from the line-length order
            let t = k as f32 / (N - 1) as f32;
            self.ap_len[i] = ((alo * (ahi / alo).powf(t) * ms) as usize).max(1);
        }
        let ap_frames = (self.ap_len.iter().copied().max().unwrap_or(1) + 2).next_power_of_two();
        self.ap_buf = vec![[0.0; N]; ap_frames];
        self.ap_mask = ap_frames - 1;
        let chorus_len = (12.0 * ms) as usize + 4;
        self.chorus_line = [DelayLine::new(chorus_len), DelayLine::new(chorus_len)];
        self.gain_key = [f32::NAN; 4];
        self.rate_key = f32::NAN;
        self.filt_key = [f32::NAN; 2];
        self.reset();
    }

    /// Stage new parameters (applied with glides from the next block).
    pub fn set_params(&mut self, p: &CoreParams) {
        self.p = *p;
        self.dirty = true;
    }

    /// Clear all state; the next block starts at the current targets without gliding.
    pub fn reset(&mut self) {
        for b in &mut self.pre {
            b.iter_mut().for_each(|x| *x = 0.0);
        }
        for l in &mut self.echo_line {
            l.reset();
        }
        for side in &mut self.diff {
            for l in side.iter_mut() {
                l.reset();
            }
        }
        self.buf.iter_mut().for_each(|f| *f = [0.0; N]);
        self.ap_buf.iter_mut().for_each(|f| *f = [0.0; N]);
        for l in &mut self.chorus_line {
            l.reset();
        }
        self.ls.reset();
        self.hs.reset();
        for i in 0..N {
            let ph = i as f32 * 2.399_963; // golden angle
            self.lfo_c[i] = ph.cos();
            self.lfo_s[i] = ph.sin();
        }
        for b in self.out_hp.iter_mut().chain(self.out_lp.iter_mut()) {
            b.reset();
        }
        self.fresh = true;
        self.dirty = true;
        self.update_targets();
        self.snap_all();
        self.update_coeffs();
    }

    /// Sum of |state| over every recursive structure and whether any value is subnormal (tests).
    pub fn state_report(&self) -> (f64, bool) {
        let mut sum = 0.0f64;
        let mut sub = false;
        let mut chk = |x: f32| {
            sum += x.abs() as f64;
            sub |= x != 0.0 && x.abs() < f32::MIN_POSITIVE;
        };
        self.buf.iter().flatten().for_each(|&x| chk(x));
        self.ap_buf.iter().flatten().for_each(|&x| chk(x));
        self.pre.iter().flatten().for_each(|&x| chk(x));
        for v in [&self.ls.s1, &self.ls.s2, &self.hs.s1, &self.hs.s2] {
            v.iter().for_each(|&x| chk(x));
        }
        for side in &self.diff {
            for l in side.iter() {
                for d in 0..=l.max_delay() {
                    chk(l.read_int(d));
                }
            }
        }
        for l in self.echo_line.iter().chain(self.chorus_line.iter()) {
            for d in 0..=l.max_delay() {
                chk(l.read_int(d));
            }
        }
        (sum, sub)
    }

    /// Current per-line loop lengths in samples (line + in-loop allpass), for tests.
    pub fn loop_lengths(&self) -> [f32; N] {
        let ms = self.fs * 0.001;
        core::array::from_fn(|i| self.base_ms[i] * self.scale.cur * ms + self.ap_len[i] as f32)
    }

    fn update_targets(&mut self) {
        self.dirty = false;
        let p = self.p;
        let ch = self.ch;
        let ms = self.fs * 0.001;
        let scale = p.scale.clamp(0.05, ch.max_scale);
        self.scale.tgt = scale;
        self.pre_d.tgt = p.pre_ms.clamp(0.0, 200.0) * ms;
        let spread = p.spread.clamp(0.0, 1.0);
        let (w0, w1) = ch.er_window_ms;
        self.er_win.tgt = scale * (w0 + spread * (w1 - w0)) * ms;
        self.shape.tgt = p.shape.clamp(0.0, 1.0);
        let max_echo = ch.max_echo_s * 1000.0 * ms;
        for c in 0..2 {
            let e = p.echo[c];
            self.echo_d[c].tgt = (e.time_ms * ms).clamp(1.0, max_echo.max(1.0));
            self.echo_lvl[c].tgt = if ch.max_echo_s > 0.0 {
                e.level.max(0.0)
            } else {
                0.0
            };
            self.echo_fb[c].tgt = e.feedback.clamp(-0.98, 0.98);
        }
        self.diffusion.tgt = p.diffusion.clamp(0.0, 1.0);
        self.decay.tgt = p.decay_s.clamp(0.05, 200.0);
        self.mult.tgt = p.bass_mult.clamp(0.1, 10.0);
        self.damp.tgt = p.damp_hz.clamp(200.0, 0.45 * self.fs);
        self.mod_rate.tgt = p.mod_rate_hz.clamp(0.0, 20.0);
        // depth never reaches the shortest line (reads stay >= 2 samples)
        let min_line = self.base_ms.iter().fold(f32::MAX, |a, &b| a.min(b)) * scale * ms;
        self.mod_depth.tgt = (p.mod_depth.max(0.0) * ch.mod_depth_ms * ms).min(min_line * 0.5);
        self.er_lvl.tgt = p.er_level.max(0.0);
        self.tail_lvl.tgt = p.tail_level.max(0.0);
        self.to_tail.tgt = p.echo_to_tail.max(0.0);
        self.chorus.tgt = p.chorus.clamp(0.0, 1.0);
        self.lc.tgt = p.lc_hz.clamp(10.0, 0.45 * self.fs);
        self.hc.tgt = p.hc_hz.clamp(100.0, 0.45 * self.fs);
    }

    fn glides(&mut self) -> [&mut Glide; 20] {
        let [l0, l1] = &mut self.echo_lvl;
        let [f0, f1] = &mut self.echo_fb;
        [
            &mut self.pre_d,
            &mut self.er_win,
            &mut self.shape,
            f0,
            f1,
            l0,
            l1,
            &mut self.diffusion,
            &mut self.scale,
            &mut self.decay,
            &mut self.mult,
            &mut self.damp,
            &mut self.mod_rate,
            &mut self.mod_depth,
            &mut self.er_lvl,
            &mut self.tail_lvl,
            &mut self.to_tail,
            &mut self.chorus,
            &mut self.lc,
            &mut self.hc,
        ]
    }

    fn snap_all(&mut self) {
        for g in self.glides() {
            g.snap();
        }
        for t in &mut self.echo_d {
            t.snap();
        }
    }

    /// Recompute block-rate coefficients from the current glide values (only what changed).
    fn update_coeffs(&mut self) {
        let fs = self.fs;
        let ms = fs * 0.001;
        // ER gains from shape: envelope exp(-alpha u), normalised to unit energy
        let sh = self.shape.cur;
        let alpha = if sh < 0.35 {
            7.0 * (0.35 - sh) / 0.35
        } else {
            -5.0 * (sh - 0.35) / 0.65
        };
        for c in 0..2 {
            let mut e = 0.0;
            for k in 0..ER_TAPS {
                let g = (-alpha * ER_POS[c][k]).exp();
                self.er_g[c][k] = g;
                e += g * g;
            }
            let n = 1.0 / e.sqrt();
            for k in 0..ER_TAPS {
                self.er_g[c][k] *= n * ER_SIGN[c][k];
            }
        }
        // decay gains (3 bands per line)
        let key = [self.decay.cur, self.mult.cur, self.damp.cur, self.scale.cur];
        if key != self.gain_key {
            self.gain_key = key;
            let fs64 = fs as f64;
            let t_mid = key[0];
            let t_lo = t_mid * key[1];
            let t_hi = t_mid * self.ch.hf_ratio;
            let xover = (self.ch.xover_hz as f64).min(fs64 * 0.45);
            let damp = key[2] as f64;
            for i in 0..N {
                let len = self.base_ms[i] * key[3] * ms + self.ap_len[i] as f32;
                let k = -LN_1000 * len / fs;
                let gm = (k / t_mid).exp().min(MAX_GAIN);
                let gl = (k / t_lo).exp().min(MAX_GAIN);
                let gh = (k / t_hi).exp().min(MAX_GAIN);
                self.gm[i] = gm;
                let db = |g: f32| 20.0 * (g / gm).log10() as f64;
                self.ls
                    .set(i, Coeffs::low_shelf_slope(fs64, xover, 1.0, db(gl)));
                self.hs
                    .set(i, Coeffs::high_shelf_slope(fs64, damp, 1.0, db(gh)));
            }
        }
        // LFO rotation per line
        if self.mod_rate.cur != self.rate_key {
            self.rate_key = self.mod_rate.cur;
            for i in 0..N {
                let w = core::f32::consts::TAU * self.rate_key * LFO_SPREAD[i % 16] / fs;
                self.rot_c[i] = w.cos();
                self.rot_s[i] = w.sin();
            }
        }
        // output filters
        let fk = [self.lc.cur, self.hc.cur];
        if fk != self.filt_key {
            self.filt_key = fk;
            let q = core::f64::consts::FRAC_1_SQRT_2;
            for c in 0..2 {
                self.out_hp[c].set(SvfMode::Highpass, fs as f64, fk[0] as f64, q, 0.0);
                self.out_lp[c].set(SvfMode::Lowpass, fs as f64, fk[1] as f64, q, 0.0);
            }
        }
    }

    /// Process one stereo block in place: input = dry, output = wet.
    pub fn process(&mut self, left: &mut [f32], right: &mut [f32]) {
        let len = left.len().min(right.len());
        if len == 0 {
            return;
        }
        if self.dirty {
            self.update_targets();
        }
        let a = if self.fresh {
            1.0
        } else {
            1.0 - (-(len as f32) / (GLIDE_S * self.fs)).exp()
        };
        self.fresh = false;
        let inv = 1.0 / len as f32;
        // block ramps: (start, per-sample increment)
        let ramp = |g: &mut Glide| {
            let (s, e) = g.step(a);
            (s, (e - s) * inv)
        };
        let (mut pre_d, pre_i) = ramp(&mut self.pre_d);
        let (mut win, win_i) = ramp(&mut self.er_win);
        let xf_inc = 1.0 / (XFADE_S * self.fs);
        let (mut fb0, fb0_i) = ramp(&mut self.echo_fb[0]);
        let (mut fb1, fb1_i) = ramp(&mut self.echo_fb[1]);
        let (mut el0, el0_i) = ramp(&mut self.echo_lvl[0]);
        let (mut el1, el1_i) = ramp(&mut self.echo_lvl[1]);
        let (mut erl, erl_i) = ramp(&mut self.er_lvl);
        let (mut tl, tl_i) = ramp(&mut self.tail_lvl);
        let (mut tt, tt_i) = ramp(&mut self.to_tail);
        let (mut cr, cr_i) = ramp(&mut self.chorus);
        let (mut depth, depth_i) = ramp(&mut self.mod_depth);
        let (sc0, sc1) = self.scale.step(a);
        for g in [
            &mut self.shape,
            &mut self.diffusion,
            &mut self.decay,
            &mut self.mult,
            &mut self.damp,
            &mut self.mod_rate,
            &mut self.lc,
            &mut self.hc,
        ] {
            g.step(a);
        }
        // ER tap gains and line gains are ramped per sample from the previous block's values
        let (er_g0, gm0) = (self.er_g, self.gm);
        self.update_coeffs();
        let mut er_g = er_g0;
        let mut er_gi = [[0.0f32; ER_TAPS]; 2];
        for c in 0..2 {
            for k in 0..ER_TAPS {
                er_gi[c][k] = (self.er_g[c][k] - er_g0[c][k]) * inv;
            }
        }
        let mut gm = gm0;
        let mut gm_i = [0.0f32; N];
        for i in 0..N {
            gm_i[i] = (self.gm[i] - gm0[i]) * inv;
        }

        let ms = self.fs * 0.001;
        let mut base = [0.0f32; N];
        let mut base_i = [0.0f32; N];
        for i in 0..N {
            base[i] = self.base_ms[i] * sc0 * ms;
            base_i[i] = self.base_ms[i] * (sc1 - sc0) * ms * inv;
        }
        let dg = self.ch.diffuser_gain * self.diffusion.cur;
        let apg = self.ch.loop_ap_gain * (0.5 + 0.5 * self.diffusion.cur);
        let echo_fb = self.ch.echo_feedback;
        let has_echo = self.ch.max_echo_s > 0.0;
        let tail_gain = self.ch.tail_gain;
        let chorus_on = cr > 0.0 || self.chorus.tgt > 0.0;
        let (ch_w0, ch_w1) = {
            let w = core::f32::consts::TAU / self.fs;
            (0.43 * w, 0.61 * w)
        };
        let (pm, mask, apm) = (self.pre_mask, self.mask, self.ap_mask);

        for n in 0..len {
            let xin = [left[n], right[n]];
            // ---- pre-delay ring, ER taps, echoes
            self.pre_w = (self.pre_w + 1) & pm;
            let pw = self.pre_w;
            self.pre[0][pw] = xin[0];
            self.pre[1][pw] = xin[1];
            pre_d += pre_i;
            win += win_i;
            let mut er = [0.0f32; 2];
            for (c, e) in er.iter_mut().enumerate() {
                let b = &self.pre[c];
                let mut acc = 0.0;
                for k in 0..ER_TAPS {
                    er_g[c][k] += er_gi[c][k];
                    acc += er_g[c][k] * tap(b, pm, pw, pre_d + win * ER_POS[c][k]);
                }
                *e = acc;
            }
            let mut eo = [0.0f32; 2];
            if has_echo {
                el0 += el0_i;
                el1 += el1_i;
                let el = [el0, el1];
                fb0 += fb0_i;
                fb1 += fb1_i;
                let fb = [fb0, fb1];
                for c in 0..2 {
                    let xt = &mut self.echo_d[c];
                    let (d0, d1, x) = xt.next(xf_inc);
                    let y = if echo_fb {
                        let line = &mut self.echo_line[c];
                        let mut y = line.read_linear(d0 - 1.0);
                        if x > 0.0 {
                            y += x * (line.read_linear(d1 - 1.0) - y);
                        }
                        line.push(flush(xin[c] + fb[c] * y));
                        y
                    } else {
                        let mut y = tap(&self.pre[c], pm, pw, d0);
                        if x > 0.0 {
                            y += x * (tap(&self.pre[c], pm, pw, d1) - y);
                        }
                        y
                    };
                    eo[c] = y * el[c];
                }
                tt += tt_i;
            }
            // ---- tail input: diffusers
            let mut tin = [er[0] + tt * eo[0], er[1] + tt * eo[1]];
            for (c, t) in tin.iter_mut().enumerate() {
                let mut x = *t;
                for j in 0..DIFFUSERS {
                    let l = &mut self.diff[c][j];
                    let s = l.read_int(self.diff_len[c][j] - 1);
                    let v = flush(x + dg * s);
                    l.push(v);
                    x = s - dg * v;
                }
                *t = x;
            }
            // ---- FDN
            let wn = (self.w + 1) & mask;
            depth += depth_i;
            let mut y = [0.0f32; N];
            for i in 0..N {
                base[i] += base_i[i];
                // 3rd-order Lagrange through offsets m-1..m+2 (offset 0 = the frame being written: d >= 2)
                let d = (base[i] + depth * self.lfo_s[i]).max(2.0);
                let m = d as usize;
                let f = d - m as f32;
                let ym1 = self.buf[wn.wrapping_sub(m - 1) & mask][i];
                let y0 = self.buf[wn.wrapping_sub(m) & mask][i];
                let y1 = self.buf[wn.wrapping_sub(m + 1) & mask][i];
                let y2 = self.buf[wn.wrapping_sub(m + 2) & mask][i];
                let (fm1, f1, f2) = (f + 1.0, f - 1.0, f - 2.0);
                y[i] = -ym1 * f * f1 * f2 * (1.0 / 6.0) + y0 * fm1 * f1 * f2 * 0.5
                    - y1 * fm1 * f * f2 * 0.5
                    + y2 * fm1 * f * f1 * (1.0 / 6.0);
            }
            let aw = (self.ap_w + 1) & apm;
            self.ap_w = aw;
            let mut av = [0.0f32; N];
            let mut a = [0.0f32; N];
            let mut f = y;
            for i in 0..N {
                gm[i] += gm_i[i];
                f[i] *= gm[i];
            }
            self.ls.tick(&mut f);
            self.hs.tick(&mut f);
            for i in 0..N {
                let f = f[i];
                let s = self.ap_buf[aw.wrapping_sub(self.ap_len[i]) & apm][i];
                let v = f + apg * s;
                av[i] = if v.abs() < FLUSH { 0.0 } else { v };
                a[i] = s - apg * v;
            }
            self.ap_buf[aw] = av;
            let mut ol = 0.0;
            let mut or = 0.0;
            for i in 0..N {
                ol += self.out_l[i] * a[i];
                or += self.out_r[i] * a[i];
            }
            hadamard(&mut a);
            let mut wv = [0.0f32; N];
            for i in 0..N {
                let v = a[i] + self.in_l[i] * tin[0] + self.in_r[i] * tin[1];
                wv[i] = if v.abs() < FLUSH { 0.0 } else { v };
            }
            self.buf[wn] = wv;
            self.w = wn;
            for i in 0..N {
                let (c, s) = (self.lfo_c[i], self.lfo_s[i]);
                self.lfo_c[i] = c * self.rot_c[i] - s * self.rot_s[i];
                self.lfo_s[i] = s * self.rot_c[i] + c * self.rot_s[i];
            }
            // ---- output
            tl += tl_i;
            erl += erl_i;
            let mut tail = [ol * tl * tail_gain, or * tl * tail_gain];
            if chorus_on {
                cr += cr_i;
                for (c, t) in tail.iter_mut().enumerate() {
                    let line = &mut self.chorus_line[c];
                    line.push(*t);
                    let ph = &mut self.chorus_ph[c];
                    *ph += if c == 0 { ch_w0 } else { ch_w1 };
                    if *ph > core::f32::consts::TAU {
                        *ph -= core::f32::consts::TAU;
                    }
                    let d = (7.0 + 3.0 * ph.sin()) * ms;
                    let wet = line.read_linear(d);
                    *t = *t * (1.0 - 0.5 * cr) + 0.7 * cr * wet;
                }
            }
            let ol = erl * er[0] + eo[0] + tail[0];
            let or = erl * er[1] + eo[1] + tail[1];
            left[n] = self.out_lp[0].process(self.out_hp[0].process(ol));
            right[n] = self.out_lp[1].process(self.out_hp[1].process(or));
        }

        // ---- per-block housekeeping: LFO renormalisation, denormal flush of filter states
        for i in 0..N {
            let (c, s) = (self.lfo_c[i], self.lfo_s[i]);
            let k = 1.5 - 0.5 * (c * c + s * s);
            self.lfo_c[i] = c * k;
            self.lfo_s[i] = s * k;
        }
        self.ls.flush();
        self.hs.flush();
        for b in self.out_hp.iter_mut().chain(self.out_lp.iter_mut()) {
            b.flush_denormals();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FS: f64 = 48_000.0;

    fn ch() -> Character {
        Character {
            line_ms: (30.0, 100.0),
            max_scale: 1.5,
            loop_ap_ms: (1.5, 6.0),
            loop_ap_gain: 0.5,
            diffuser_ms: [1.3, 2.7, 4.1, 6.3],
            diffuser_gain: 0.7,
            xover_hz: 500.0,
            hf_ratio: 0.3,
            mod_depth_ms: 0.15,
            er_window_ms: (4.0, 100.0),
            max_echo_s: 0.0,
            echo_feedback: false,
            tail_gain: 1.0,
        }
    }

    #[test]
    fn hadamard_is_orthogonal() {
        let mut x = [0.0f32; 16];
        x[3] = 1.0;
        hadamard(&mut x);
        let e: f32 = x.iter().map(|v| v * v).sum();
        assert!((e - 1.0).abs() < 1e-6);
        hadamard(&mut x);
        assert!((x[3] - 1.0).abs() < 1e-6);
        assert!(x.iter().enumerate().all(|(i, v)| i == 3 || v.abs() < 1e-6));
    }

    #[test]
    fn decay_gains_follow_jot_formula() {
        let mut c = ReverbCore::<8>::new(ch());
        c.prepare(FS);
        let p = CoreParams {
            decay_s: 2.0,
            bass_mult: 2.0,
            ..Default::default()
        };
        c.set_params(&p);
        c.reset();
        let len = c.loop_lengths();
        for i in 0..8 {
            let want = 10f32.powf(-3.0 * len[i] / (FS as f32 * 2.0));
            assert!((c.gm[i] - want).abs() < 1e-5, "{} {}", c.gm[i], want);
            let lo = 10f32.powf(-3.0 * len[i] / (FS as f32 * 4.0));
            let ls = &c.ls;
            let dc = (ls.b0[i] + ls.b1[i] + ls.b2[i]) / (1.0 + ls.a1[i] + ls.a2[i]);
            assert!((c.gm[i] * dc - lo).abs() < 1e-4, "{} {}", c.gm[i] * dc, lo);
        }
    }

    #[test]
    fn silence_stays_silent_and_finite() {
        let mut c = ReverbCore::<16>::new(ch());
        c.prepare(FS);
        let (mut l, mut r) = ([0.0f32; 64], [0.0f32; 64]);
        for _ in 0..100 {
            c.process(&mut l, &mut r);
        }
        assert!(l.iter().chain(r.iter()).all(|&v| v == 0.0));
    }
}
