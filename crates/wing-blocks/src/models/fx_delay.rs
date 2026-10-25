//! FX rack delay family (T050 proof model `ST-DL`, completed by T05D): `ST-DL` STEREO DELAY, `TAP-DL` ULTRATAP
//! DELAY, `TAPE-DL` TAPE DELAY, `OILCAN` OILCAN DELAY (Tel-Ray), `BBD-DL` BBD DELAY (models.md §6).
//!
//! Every model outputs the **wet** signal only; the FX slot host blends it with the dry signal by `fxmix`
//! (wing-engine `FxSlot`), like every FX rack model. All models are zero-latency.
//!
//! # Tempo
//!
//! The WING schema has no global tempo, BPM or tap key (neither in `fx/N` nor in `cfg`), so nothing global is
//! bound. Time is per model: `ST-DL` and `TAP-DL` carry `time` (ms, the quarter note when used as a tempo)
//! times a note `fact`or (`1/4 .. 2` incl. dotted `3/8`, `3/4`, `3/2` and triplet `1/3`, `2/3`, `4/3`);
//! `TAPE-DL` has `time` in ms, `OILCAN` and `BBD-DL` have 0..10 / 0..100 knobs mapped to a time ([I] tapers
//! below).
//!
//! # Time changes
//!
//! * Digital models (`ST-DL`, `TAP-DL`) **crossfade** ([`XFADE_S`]) from the old read position to the new
//!   one: no pitch artefact, no click. A change that arrives mid-fade is queued and starts when the fade ends.
//! * Analog models (`TAPE-DL`, `OILCAN`, `BBD-DL`) **glide** like a motor / clock change: a one-pole approach
//!   whose slope is rate-limited (max pitch deviation per model), so the echoes bend in pitch but never click.
//!
//! A freshly reset model starts at its parameters' times (no glide from the previous model's settings).
//!
//! # Models
//!
//! ```text
//! ST-DL   in L/R --(M: (L+R)/2)--> + --> line L/R --> LO CUT -> HI CUT --> wet
//!                                  ^         |
//!                                  +-- feed * FEED LCUT (HP1) * FEED HCUT (LP1) <--+   (X: L feeds R, R feeds L)
//!
//! TAP-DL  (L+R)/2 --> line --> REPEATS taps at PRE DLY + TIME*FACT*i/REPEATS (i = 1..REPEATS),
//!                              gain slope over the taps, pan per MODE * WIDTH --> DIFFUSION --> LO/HI CUT --> wet
//!
//! TAPE-DL in --> + --> sat(DRIVE) --> tape (wow/flutter-modulated read) --> HP -> head bump -> HF loss --> wet
//!                ^------------------------------------- SUSTAIN ----------------------------------+
//!
//! OILCAN  in --> + --> sat --> oil can (3-point smeared, WOBBLE-modulated read) --> HP -> TONE LP --> wet
//!                ^---------------------------------- SUSTAIN -----------------------------+
//!
//! BBD-DL  in --> + --> AA LPF --> 2:1 compressor --> clip --> S&H @ f_clk --> BBD --> +noise/clock
//!                ^                                   --> reconstruction LPF --> 1:2 expander --> wet
//!                +------------------------------ FEED --------------------------------------------+
//! ```
//!
//! Mappings and constants marked [I] are inferred (no console measurement yet); ranges, defaults and enum
//! orders come from the schema (`data/schema/wing-tree.json`, WING-Edit `fx_pardef_*`).
//!
//! * `ST-DL`: `L = time * fact * patL`, `R = time * fact * patR`; a positive `offs` delays R, a negative one
//!   delays L [I]. Feedback max 0.99.
//! * `TAP-DL` [I] (Eventide UltraTap-style): `time * fact` is the length over which `rep` taps are spaced
//!   evenly after `pdel`; `slp` is the level difference first -> last tap (loudest tap at 0 dB); `mode` is the
//!   tap pan trajectory (MOVE: sweep L -> R; JUMP: alternate L/R; FOCUS: alternate, wide -> centre; SPREAD:
//!   alternate, centre -> wide) scaled by `wid` (negative mirrors); `diff` blends a 4-stage allpass smear.
//! * `TAPE-DL` [I] (RE-201 / Echoplex): `time` sets the tape speed, so bandwidth (HF loss 1.5..12 kHz), head
//!   bump (+3 dB, 25..300 Hz) and wow/flutter depth scale with it; `sust` = loop gain (max 0.95 / bump peak),
//!   `drv` = record drive 0..+24 dB into a soft saturator (gain compensated), `wf` = wow (0.7 Hz) + flutter
//!   (7.3 Hz) + random drift as a transport-speed deviation.
//! * `OILCAN` [I] (Tel-Ray): `time` 0..10 -> 40..500 ms (exponential), `sust` 0..10 -> loop gain 0..0.92,
//!   `wb` -> wiper/disc wobble (5.3 Hz + drift, up to +-1.2 ms, plus 2.1 Hz amplitude wobble), `tone` -> loop
//!   low-pass 800..6400 Hz; fixed 160 Hz high-pass, 3-point read smear (+-0.6 ms) and mild saturation.
//! * `BBD-DL` [I] (Boss DM-2 style, 4096 stages): `dly` 0..100 -> 20..300 ms (exponential), clock
//!   `f_clk = 4096 / (2 * delay)`; anti-alias and reconstruction filters are 4th-order Butterworth at
//!   `min(5 kHz, 0.38 f_clk)` (bandwidth falls with delay time); a sample-and-hold at `f_clk` (when below
//!   `fs`) gives the BBD's aliasing grit; NE570-style 2:1 / 1:2 compander (10 ms average detectors) around the
//!   BBD, so hiss and clock feed-through ride under the signal; `fb` = loop gain 0..0.95.
//!
//! Real-time: `prepare` allocates every line; `process`, `set_param`, `commit`, `reset` never allocate
//! (`tests/fx_delay_no_alloc.rs`). Bench (`benches/fx_delay.rs`, M1, stereo 64-sample block): ST-DL ~0.7 µs,
//! TAP-DL (16 taps + diffusion) ~2.4 µs, TAPE-DL / OILCAN / BBD-DL ~1.3 µs.

use crate::model::{Model, ParamValue};
use wing_dsp::biquad::{Biquad, Coeffs};
use wing_dsp::delay::DelayLine;
use wing_dsp::smooth::one_pole_coeff;

use core::f32::consts::{FRAC_PI_4, SQRT_2, TAU};
use core::f64::consts::FRAC_1_SQRT_2;

/// Longest effective delay per side of `ST-DL` (TIME range plus offset), seconds.
pub const MAX_DELAY_S: f64 = 3.05;
/// Crossfade length of the digital models' time changes, seconds [I].
pub const XFADE_S: f64 = 0.03;
/// Parameter (gain) smoothing time constant.
const SMOOTH_S: f64 = 0.02;
/// Maximum `ST-DL` loop gain (keeps FEED 100 % below self-oscillation).
const MAX_FEED: f32 = 0.99;

/// One-pole step coefficient `1 - exp(-1/(tau fs))`.
fn step_coeff(tau_s: f64, fs: f64) -> f32 {
    (1.0 - one_pole_coeff(tau_s, fs)) as f32
}

/// Soft saturator: odd rational tanh approximation, unity slope at 0, `+-1` with zero slope at `|x| >= 3`.
#[inline(always)]
fn sat(x: f32) -> f32 {
    let x = x.clamp(-3.0, 3.0);
    let x2 = x * x;
    x * (27.0 + x2) / (27.0 + 9.0 * x2)
}

/// Clamp a cutoff to `[lo, min(hi, 0.45 fs)]`.
fn fc(v: f32, lo: f32, hi: f32, fs: f64) -> f64 {
    v.clamp(lo, hi.min((fs * 0.45) as f32)) as f64
}

fn flush(bq: &mut [Biquad]) {
    for b in bq {
        b.flush_denormals();
    }
}

fn reset_bq(bq: &mut [Biquad]) {
    for b in bq {
        b.reset();
    }
}

/// Deterministic xorshift32 noise in `[-1, 1)`.
#[derive(Clone, Copy, Debug)]
struct Rng(u32);

impl Rng {
    #[inline(always)]
    fn next(&mut self) -> f32 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.0 = x;
        (x >> 8) as f32 * (2.0 / 16_777_216.0) - 1.0
    }
}

/// Smooth random drift in about `[-1, 1]`: a new random target every `period` samples, approached by a
/// one-pole (transport irregularities).
#[derive(Clone, Copy, Debug)]
struct Drift {
    rng: Rng,
    target: f32,
    value: f32,
    coeff: f32,
    count: u32,
    period: u32,
}

impl Drift {
    fn new(seed: u32) -> Self {
        Self {
            rng: Rng(seed),
            target: 0.0,
            value: 0.0,
            coeff: 0.0,
            count: 0,
            period: 1,
        }
    }

    fn prepare(&mut self, rate_hz: f64, fs: f64) {
        self.period = (fs / rate_hz).max(1.0) as u32;
        self.coeff = step_coeff(0.5 / rate_hz, fs);
    }

    fn reset(&mut self, seed: u32) {
        self.rng = Rng(seed);
        self.target = 0.0;
        self.value = 0.0;
        self.count = 0;
    }

    #[inline(always)]
    fn tick(&mut self) -> f32 {
        if self.count == 0 {
            self.count = self.period;
            self.target = self.rng.next();
        }
        self.count -= 1;
        self.value += self.coeff * (self.target - self.value);
        self.value
    }
}

/// Sine LFO by phase accumulation.
#[derive(Clone, Copy, Debug, Default)]
struct Lfo {
    phase: f32,
    inc: f32,
}

impl Lfo {
    fn prepare(&mut self, hz: f64, fs: f64) {
        self.inc = (hz / fs) as f32;
    }

    #[inline(always)]
    fn tick(&mut self) -> f32 {
        self.phase += self.inc;
        if self.phase >= 1.0 {
            self.phase -= 1.0;
        }
        (TAU * self.phase).sin()
    }
}

/// Rate-limited one-pole glide of a delay time in samples (analog models). f64 state: an f32 one-pole
/// stalls samples short of a target of ~10^4.
#[derive(Clone, Copy, Debug)]
struct Glide {
    value: f64,
    target: f64,
    coeff: f64,
    max_rate: f64,
}

impl Glide {
    fn new(max_rate: f64) -> Self {
        Self {
            value: 1.0,
            target: 1.0,
            coeff: 1.0,
            max_rate,
        }
    }

    fn value(&self) -> f32 {
        self.value as f32
    }

    #[inline(always)]
    fn tick(&mut self) -> f32 {
        let step = (self.coeff * (self.target - self.value)).clamp(-self.max_rate, self.max_rate);
        self.value += step;
        self.value as f32
    }
}

/// Crossfading read head of a digital delay: a target change starts a linear crossfade from the current
/// read position to the new one; a change during a fade waits for its end.
#[derive(Clone, Copy, Debug)]
struct XfadeHead {
    cur: f32,
    next: f32,
    g: f32,
    fading: bool,
}

impl XfadeHead {
    fn new() -> Self {
        Self {
            cur: 1.0,
            next: 1.0,
            g: 0.0,
            fading: false,
        }
    }

    fn jump(&mut self, d: f32) {
        *self = Self {
            cur: d,
            next: d,
            g: 0.0,
            fading: false,
        };
    }

    /// Read `line` (before this sample's push) at the head's delay, moving towards `target`.
    #[inline(always)]
    fn read(&mut self, line: &DelayLine, target: f32, inc: f32) -> f32 {
        if !self.fading && target != self.cur {
            self.next = target;
            self.fading = true;
            self.g = 0.0;
        }
        let a = line.read_linear(self.cur - 1.0);
        if !self.fading {
            return a;
        }
        let b = line.read_linear(self.next - 1.0);
        let y = a + (b - a) * self.g;
        self.g += inc;
        if self.g >= 1.0 {
            self.cur = self.next;
            self.fading = false;
        }
        y
    }
}

// ---------------------------------------------------------------------------------------------------------
// ST-DL
// ---------------------------------------------------------------------------------------------------------

/// `ST-DL` `fact` tokens in schema order.
pub const FACTORS: [f32; 11] = [
    0.25,
    1.0 / 3.0,
    0.5,
    2.0 / 3.0,
    0.75,
    1.0,
    0.375,
    1.25,
    4.0 / 3.0,
    1.5,
    2.0,
];

/// `pat` tokens (L:R) in schema order.
pub const PATTERNS: [(f32, f32); 9] = [
    (0.5, 1.0),
    (2.0 / 3.0, 1.0),
    (0.75, 1.0),
    (0.875, 1.0),
    (1.0, 1.0),
    (1.0, 1.125),
    (1.0, 1.25),
    (1.0, 4.0 / 3.0),
    (1.0, 1.5),
];

/// `ST-DL` schema slots.
pub mod slot {
    pub const TIME: u16 = 1;
    pub const MODE: u16 = 2;
    pub const FACT: u16 = 3;
    pub const PAT: u16 = 4;
    pub const OFFS: u16 = 5;
    pub const FEED: u16 = 6;
    pub const FLC: u16 = 7;
    pub const FHC: u16 = 8;
    pub const LC: u16 = 9;
    pub const HC: u16 = 10;
}

/// `ST-DL` `mode`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DelayMode {
    /// Two independent lines.
    Stereo,
    /// Cross feedback (ping-pong).
    Cross,
    /// Mono input to both lines.
    Mono,
}

/// `ST-DL` parameters in engineering units.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StereoDelayParams {
    pub time_ms: f32,
    pub mode: DelayMode,
    /// Index into [`FACTORS`].
    pub fact: usize,
    /// Index into [`PATTERNS`].
    pub pat: usize,
    pub offs_ms: f32,
    pub feed_pct: f32,
    pub flc_hz: f32,
    pub fhc_hz: f32,
    pub lc_hz: f32,
    pub hc_hz: f32,
}

impl Default for StereoDelayParams {
    /// WING-Edit defaults (schema): 333 ms, ST, 1, 1:1, 0 ms, 35 %, 101 Hz / 8 kHz, 101 Hz / 9.6 kHz.
    fn default() -> Self {
        Self {
            time_ms: 333.0,
            mode: DelayMode::Stereo,
            fact: 5,
            pat: 4,
            offs_ms: 0.0,
            feed_pct: 35.0,
            flc_hz: 101.0,
            fhc_hz: 8000.0,
            lc_hz: 101.0,
            hc_hz: 9600.0,
        }
    }
}

impl StereoDelayParams {
    /// Effective (L, R) delays in ms before clamping.
    pub fn delays_ms(&self) -> (f32, f32) {
        let base = self.time_ms.clamp(1.0, 3000.0) * FACTORS[self.fact.min(FACTORS.len() - 1)];
        let (pl, pr) = PATTERNS[self.pat.min(PATTERNS.len() - 1)];
        let (mut l, mut r) = (base * pl, base * pr);
        let o = self.offs_ms.clamp(-50.0, 50.0);
        if o >= 0.0 {
            r += o;
        } else {
            l -= o;
        }
        (l, r)
    }
}

/// STEREO DELAY (`ST-DL`); see the module doc.
pub struct StereoDelay {
    p: StereoDelayParams,
    dirty: bool,
    fs: f64,
    lines: [DelayLine; 2],
    heads: [XfadeHead; 2],
    /// Target delay in samples per side.
    d_target: [f32; 2],
    xf_inc: f32,
    /// No sample processed since `reset`: parameter changes jump instead of crossfading (a freshly selected
    /// model starts at its scene's times).
    fresh: bool,
    feed: f32,
    feed_s: f32,
    sm: f32,
    fb_hp: [Biquad; 2],
    fb_lp: [Biquad; 2],
    out_hp: [Biquad; 2],
    out_lp: [Biquad; 2],
}

impl Default for StereoDelay {
    fn default() -> Self {
        Self::new()
    }
}

impl StereoDelay {
    /// Unprepared instance (call [`Model::prepare`] before processing).
    pub fn new() -> Self {
        Self {
            p: StereoDelayParams::default(),
            dirty: true,
            fs: 48_000.0,
            lines: [DelayLine::new(1), DelayLine::new(1)],
            heads: [XfadeHead::new(); 2],
            d_target: [1.0; 2],
            xf_inc: 1.0,
            fresh: true,
            feed: 0.0,
            feed_s: 0.0,
            sm: 1.0,
            fb_hp: [Biquad::default(); 2],
            fb_lp: [Biquad::default(); 2],
            out_hp: [Biquad::default(); 2],
            out_lp: [Biquad::default(); 2],
        }
    }

    pub fn params(&self) -> &StereoDelayParams {
        &self.p
    }

    /// Set every parameter (staged; applied at the next `commit`/`process`).
    pub fn set_params(&mut self, p: &StereoDelayParams) {
        self.p = *p;
        self.dirty = true;
    }

    /// Current target delays in samples (L, R).
    pub fn delay_samples(&self) -> (f32, f32) {
        (self.d_target[0], self.d_target[1])
    }

    fn update(&mut self) {
        self.dirty = false;
        let fs = self.fs;
        let (l, r) = self.p.delays_ms();
        let max = self.lines[0].max_delay() as f32;
        self.d_target = [
            (l * 0.001 * fs as f32).clamp(1.0, max),
            (r * 0.001 * fs as f32).clamp(1.0, max),
        ];
        if self.fresh {
            for k in 0..2 {
                self.heads[k].jump(self.d_target[k]);
            }
        }
        self.feed = (self.p.feed_pct.clamp(0.0, 100.0) * 0.01).min(MAX_FEED);
        if self.fresh {
            self.feed_s = self.feed;
        }
        let hp1 = Coeffs::highpass1(fs, fc(self.p.flc_hz, 20.0, 400.0, fs));
        let lp1 = Coeffs::lowpass1(fs, fc(self.p.fhc_hz, 200.0, 20_000.0, fs));
        let hp = Coeffs::highpass(fs, fc(self.p.lc_hz, 20.0, 400.0, fs), FRAC_1_SQRT_2);
        let lp = Coeffs::lowpass(fs, fc(self.p.hc_hz, 200.0, 20_000.0, fs), FRAC_1_SQRT_2);
        for k in 0..2 {
            self.fb_hp[k].set_coeffs(hp1);
            self.fb_lp[k].set_coeffs(lp1);
            self.out_hp[k].set_coeffs(hp);
            self.out_lp[k].set_coeffs(lp);
        }
    }
}

impl Model for StereoDelay {
    fn id(&self) -> &'static str {
        "ST-DL"
    }

    fn prepare(&mut self, sample_rate: f64, _max_block: usize) {
        self.fs = sample_rate;
        let n = (MAX_DELAY_S * sample_rate).ceil() as usize + 2;
        self.lines = [DelayLine::new(n), DelayLine::new(n)];
        self.xf_inc = (1.0 / (XFADE_S * sample_rate)) as f32;
        self.sm = step_coeff(SMOOTH_S, sample_rate);
        self.fresh = true;
        self.update();
        self.reset();
    }

    fn process(&mut self, left: &mut [f32], right: &mut [f32], _sc: Option<(&[f32], &[f32])>) {
        if self.dirty {
            self.update();
        }
        let (mode, inc, sm) = (self.p.mode, self.xf_inc, self.sm);
        let (tl, tr) = (self.d_target[0], self.d_target[1]);
        self.fresh = left.is_empty() && self.fresh;
        for (xl, xr) in left.iter_mut().zip(right.iter_mut()) {
            let (il, ir) = match mode {
                DelayMode::Mono => {
                    let m = 0.5 * (*xl + *xr);
                    (m, m)
                }
                _ => (*xl, *xr),
            };
            self.feed_s += sm * (self.feed - self.feed_s);
            let g = self.feed_s;
            // read before the push: delay d = read position d - 1
            let yl = self.heads[0].read(&self.lines[0], tl, inc);
            let yr = self.heads[1].read(&self.lines[1], tr, inc);
            let fl = self.fb_lp[0].tick(self.fb_hp[0].tick(yl));
            let fr = self.fb_lp[1].tick(self.fb_hp[1].tick(yr));
            let (pl, pr) = match mode {
                DelayMode::Cross => (il + g * fr, ir + g * fl),
                _ => (il + g * fl, ir + g * fr),
            };
            self.lines[0].push(pl);
            self.lines[1].push(pr);
            *xl = self.out_lp[0].tick(self.out_hp[0].tick(yl));
            *xr = self.out_lp[1].tick(self.out_hp[1].tick(yr));
        }
        flush(&mut self.fb_hp);
        flush(&mut self.fb_lp);
        flush(&mut self.out_hp);
        flush(&mut self.out_lp);
    }

    fn reset(&mut self) {
        self.fresh = true;
        self.update();
        for l in &mut self.lines {
            l.reset();
        }
        reset_bq(&mut self.fb_hp);
        reset_bq(&mut self.fb_lp);
        reset_bq(&mut self.out_hp);
        reset_bq(&mut self.out_lp);
    }

    fn set_param(&mut self, slot: u16, v: ParamValue) {
        let p = &mut self.p;
        match slot {
            slot::TIME => p.time_ms = v.as_f32(),
            slot::MODE => {
                p.mode = match v.as_index() {
                    0 => DelayMode::Stereo,
                    1 => DelayMode::Cross,
                    _ => DelayMode::Mono,
                }
            }
            slot::FACT => p.fact = v.as_index().min(FACTORS.len() - 1),
            slot::PAT => p.pat = v.as_index().min(PATTERNS.len() - 1),
            slot::OFFS => p.offs_ms = v.as_f32(),
            slot::FEED => p.feed_pct = v.as_f32(),
            slot::FLC => p.flc_hz = v.as_f32(),
            slot::FHC => p.fhc_hz = v.as_f32(),
            slot::LC => p.lc_hz = v.as_f32(),
            slot::HC => p.hc_hz = v.as_f32(),
            _ => return,
        }
        self.dirty = true;
    }

    fn param(&self, slot: u16) -> Option<f32> {
        let p = &self.p;
        Some(match slot {
            slot::TIME => p.time_ms,
            slot::MODE => p.mode as u8 as f32,
            slot::FACT => p.fact as f32,
            slot::PAT => p.pat as f32,
            slot::OFFS => p.offs_ms,
            slot::FEED => p.feed_pct,
            slot::FLC => p.flc_hz,
            slot::FHC => p.fhc_hz,
            slot::LC => p.lc_hz,
            slot::HC => p.hc_hz,
            _ => return None,
        })
    }

    fn load_defaults(&mut self) {
        self.set_params(&StereoDelayParams::default());
    }

    fn commit(&mut self) {
        if self.dirty {
            self.update();
        }
    }
}

// ---------------------------------------------------------------------------------------------------------
// TAP-DL
// ---------------------------------------------------------------------------------------------------------

/// `TAP-DL` schema slots.
pub mod tap_slot {
    pub const TIME: u16 = 1;
    pub const REP: u16 = 2;
    pub const SLP: u16 = 3;
    pub const FACT: u16 = 4;
    pub const PDEL: u16 = 5;
    pub const MODE: u16 = 6;
    pub const WID: u16 = 7;
    pub const DIFF: u16 = 8;
    pub const LC: u16 = 9;
    pub const HC: u16 = 10;
}

/// `TAP-DL` `fact` tokens in schema order.
pub const TAP_FACTORS: [f32; 9] = [
    1.0 / 3.0,
    0.5,
    2.0 / 3.0,
    0.75,
    1.0,
    1.25,
    4.0 / 3.0,
    1.5,
    2.0,
];

/// Most taps (`rep` range 1..16).
pub const MAX_TAPS: usize = 16;
/// Longest `TAP-DL` tap: PRE DLY 500 ms + TIME 2000 ms * FACTOR 2.
pub const TAP_MAX_S: f64 = 4.5;

/// `TAP-DL` `mode`: pan trajectory of the taps (see the module doc).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TapMode {
    Move,
    Jump,
    Focus,
    Spread,
}

/// `TAP-DL` parameters in engineering units.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TapDelayParams {
    pub time_ms: f32,
    pub rep: usize,
    pub slp_db: f32,
    /// Index into [`TAP_FACTORS`].
    pub fact: usize,
    pub pdel_ms: f32,
    pub mode: TapMode,
    pub wid_pct: f32,
    pub diff: f32,
    pub lc_hz: f32,
    pub hc_hz: f32,
}

impl Default for TapDelayParams {
    /// Schema defaults: 300 ms, 4 taps, -3 dB, 1, 0 ms, MOVE, 70 %, 0, 20 Hz, 20 kHz.
    fn default() -> Self {
        Self {
            time_ms: 300.0,
            rep: 4,
            slp_db: -3.0,
            fact: 4,
            pdel_ms: 0.0,
            mode: TapMode::Move,
            wid_pct: 70.0,
            diff: 0.0,
            lc_hz: 20.0,
            hc_hz: 20_000.0,
        }
    }
}

/// One tap of [`TapDelayParams::taps`].
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Tap {
    pub delay_ms: f32,
    pub gain: f32,
    /// Pan position -1 (L) .. +1 (R).
    pub pan: f32,
}

impl TapDelayParams {
    /// The tap layout (`rep` entries of the returned array are valid).
    pub fn taps(&self) -> (usize, [Tap; MAX_TAPS]) {
        let n = self.rep.clamp(1, MAX_TAPS);
        let len =
            self.time_ms.clamp(1.0, 2000.0) * TAP_FACTORS[self.fact.min(TAP_FACTORS.len() - 1)];
        let pdel = self.pdel_ms.clamp(0.0, 500.0);
        let slp = self.slp_db.clamp(-24.0, 24.0);
        let w = self.wid_pct.clamp(-100.0, 100.0) * 0.01;
        let mut t = [Tap::default(); MAX_TAPS];
        for (i, tap) in t.iter_mut().enumerate().take(n) {
            let u = if n > 1 {
                i as f32 / (n - 1) as f32
            } else {
                0.0
            };
            let alt = if i % 2 == 0 { -1.0 } else { 1.0 };
            let pan = if n == 1 && self.mode != TapMode::Jump {
                0.0
            } else {
                match self.mode {
                    TapMode::Move => 2.0 * u - 1.0,
                    TapMode::Jump => alt,
                    TapMode::Focus => alt * (1.0 - u),
                    TapMode::Spread => alt * u,
                }
            };
            *tap = Tap {
                delay_ms: pdel + len * (i + 1) as f32 / n as f32,
                gain: 10f32.powf((slp * u - slp.max(0.0)) / 20.0),
                pan: pan * w,
            };
        }
        (n, t)
    }
}

/// Resolved tap layout in samples and per-side gains.
#[derive(Clone, Copy, Debug, PartialEq)]
struct TapSet {
    n: usize,
    d: [f32; MAX_TAPS],
    gl: [f32; MAX_TAPS],
    gr: [f32; MAX_TAPS],
}

impl TapSet {
    const EMPTY: Self = Self {
        n: 0,
        d: [1.0; MAX_TAPS],
        gl: [0.0; MAX_TAPS],
        gr: [0.0; MAX_TAPS],
    };

    #[inline(always)]
    fn read(&self, line: &DelayLine) -> (f32, f32) {
        let (mut l, mut r) = (0.0, 0.0);
        for i in 0..self.n {
            let v = line.read_linear(self.d[i] - 1.0);
            l += v * self.gl[i];
            r += v * self.gr[i];
        }
        (l, r)
    }
}

/// Schroeder allpass on its own line.
struct Allpass {
    line: DelayLine,
    d: usize,
}

impl Allpass {
    fn new(d: usize) -> Self {
        Self {
            line: DelayLine::new(d + 1),
            d: d.max(1),
        }
    }

    #[inline(always)]
    fn tick(&mut self, x: f32, g: f32) -> f32 {
        let w = self.line.read_int(self.d - 1);
        let v = x + g * w;
        self.line.push(v);
        w - g * v
    }
}

/// Diffusion allpass delays at 48 kHz (L, R; mutually prime, ~19 ms total) [I].
const DIFF_D: [[usize; 4]; 2] = [[113, 173, 251, 367], [127, 181, 239, 383]];
/// Allpass gain of the diffuser.
const DIFF_G: f32 = 0.6;

/// ULTRATAP DELAY (`TAP-DL`); see the module doc.
pub struct TapDelay {
    p: TapDelayParams,
    dirty: bool,
    fs: f64,
    line: DelayLine,
    target: TapSet,
    a: TapSet,
    b: TapSet,
    fading: bool,
    g: f32,
    xf_inc: f32,
    fresh: bool,
    diff: [[Allpass; 4]; 2],
    diff_amt: f32,
    diff_s: f32,
    sm: f32,
    out_hp: [Biquad; 2],
    out_lp: [Biquad; 2],
}

impl Default for TapDelay {
    fn default() -> Self {
        Self::new()
    }
}

impl TapDelay {
    pub fn new() -> Self {
        Self {
            p: TapDelayParams::default(),
            dirty: true,
            fs: 48_000.0,
            line: DelayLine::new(1),
            target: TapSet::EMPTY,
            a: TapSet::EMPTY,
            b: TapSet::EMPTY,
            fading: false,
            g: 0.0,
            xf_inc: 1.0,
            fresh: true,
            diff: core::array::from_fn(|k| core::array::from_fn(|i| Allpass::new(DIFF_D[k][i]))),
            diff_amt: 0.0,
            diff_s: 0.0,
            sm: 1.0,
            out_hp: [Biquad::default(); 2],
            out_lp: [Biquad::default(); 2],
        }
    }

    pub fn params(&self) -> &TapDelayParams {
        &self.p
    }

    pub fn set_params(&mut self, p: &TapDelayParams) {
        self.p = *p;
        self.dirty = true;
    }

    fn update(&mut self) {
        self.dirty = false;
        let fs = self.fs;
        let (n, taps) = self.p.taps();
        let max = self.line.max_delay() as f32;
        let mut s = TapSet::EMPTY;
        s.n = n;
        for (i, t) in taps.iter().enumerate().take(n) {
            s.d[i] = (t.delay_ms * 0.001 * fs as f32).clamp(1.0, max);
            // -3 dB compensated balance: centre = unity on both sides
            let th = (t.pan + 1.0) * FRAC_PI_4;
            s.gl[i] = t.gain * (SQRT_2 * th.cos()).min(1.0);
            s.gr[i] = t.gain * (SQRT_2 * th.sin()).min(1.0);
        }
        self.target = s;
        self.diff_amt = self.p.diff.clamp(0.0, 100.0) * 0.01;
        if self.fresh {
            self.a = s;
            self.fading = false;
            self.diff_s = self.diff_amt;
        }
        let hp = Coeffs::highpass(fs, fc(self.p.lc_hz, 20.0, 400.0, fs), FRAC_1_SQRT_2);
        let lp = Coeffs::lowpass(fs, fc(self.p.hc_hz, 200.0, 20_000.0, fs), FRAC_1_SQRT_2);
        for k in 0..2 {
            self.out_hp[k].set_coeffs(hp);
            self.out_lp[k].set_coeffs(lp);
        }
    }
}

impl Model for TapDelay {
    fn id(&self) -> &'static str {
        "TAP-DL"
    }

    fn prepare(&mut self, sample_rate: f64, _max_block: usize) {
        self.fs = sample_rate;
        self.line = DelayLine::new((TAP_MAX_S * sample_rate).ceil() as usize + 2);
        let scale = sample_rate / 48_000.0;
        self.diff = core::array::from_fn(|k| {
            core::array::from_fn(|i| Allpass::new((DIFF_D[k][i] as f64 * scale).round() as usize))
        });
        self.xf_inc = (1.0 / (XFADE_S * sample_rate)) as f32;
        self.sm = step_coeff(SMOOTH_S, sample_rate);
        self.fresh = true;
        self.update();
        self.reset();
    }

    fn process(&mut self, left: &mut [f32], right: &mut [f32], _sc: Option<(&[f32], &[f32])>) {
        if self.dirty {
            self.update();
        }
        if !self.fading && self.target != self.a {
            self.b = self.target;
            self.fading = true;
            self.g = 0.0;
        }
        self.fresh = left.is_empty() && self.fresh;
        let (inc, sm) = (self.xf_inc, self.sm);
        for (xl, xr) in left.iter_mut().zip(right.iter_mut()) {
            let (mut yl, mut yr) = self.a.read(&self.line);
            if self.fading {
                let (bl, br) = self.b.read(&self.line);
                yl += (bl - yl) * self.g;
                yr += (br - yr) * self.g;
                self.g += inc;
                if self.g >= 1.0 {
                    self.a = self.b;
                    self.fading = false;
                }
            }
            self.line.push(0.5 * (*xl + *xr));
            self.diff_s += sm * (self.diff_amt - self.diff_s);
            if self.diff_s > 1e-4 {
                let (mut zl, mut zr) = (yl, yr);
                for ap in &mut self.diff[0] {
                    zl = ap.tick(zl, DIFF_G);
                }
                for ap in &mut self.diff[1] {
                    zr = ap.tick(zr, DIFF_G);
                }
                yl += (zl - yl) * self.diff_s;
                yr += (zr - yr) * self.diff_s;
            }
            *xl = self.out_lp[0].tick(self.out_hp[0].tick(yl));
            *xr = self.out_lp[1].tick(self.out_hp[1].tick(yr));
        }
        flush(&mut self.out_hp);
        flush(&mut self.out_lp);
    }

    fn reset(&mut self) {
        self.fresh = true;
        self.update();
        self.line.reset();
        for ap in self.diff.iter_mut().flatten() {
            ap.line.reset();
        }
        reset_bq(&mut self.out_hp);
        reset_bq(&mut self.out_lp);
    }

    fn set_param(&mut self, slot: u16, v: ParamValue) {
        let p = &mut self.p;
        match slot {
            tap_slot::TIME => p.time_ms = v.as_f32(),
            tap_slot::REP => p.rep = v.as_index().clamp(1, MAX_TAPS),
            tap_slot::SLP => p.slp_db = v.as_f32(),
            tap_slot::FACT => p.fact = v.as_index().min(TAP_FACTORS.len() - 1),
            tap_slot::PDEL => p.pdel_ms = v.as_f32(),
            tap_slot::MODE => {
                p.mode = match v.as_index() {
                    0 => TapMode::Move,
                    1 => TapMode::Jump,
                    2 => TapMode::Focus,
                    _ => TapMode::Spread,
                }
            }
            tap_slot::WID => p.wid_pct = v.as_f32(),
            tap_slot::DIFF => p.diff = v.as_f32(),
            tap_slot::LC => p.lc_hz = v.as_f32(),
            tap_slot::HC => p.hc_hz = v.as_f32(),
            _ => return,
        }
        self.dirty = true;
    }

    fn param(&self, slot: u16) -> Option<f32> {
        let p = &self.p;
        Some(match slot {
            tap_slot::TIME => p.time_ms,
            tap_slot::REP => p.rep as f32,
            tap_slot::SLP => p.slp_db,
            tap_slot::FACT => p.fact as f32,
            tap_slot::PDEL => p.pdel_ms,
            tap_slot::MODE => p.mode as u8 as f32,
            tap_slot::WID => p.wid_pct,
            tap_slot::DIFF => p.diff,
            tap_slot::LC => p.lc_hz,
            tap_slot::HC => p.hc_hz,
            _ => return None,
        })
    }

    fn load_defaults(&mut self) {
        self.set_params(&TapDelayParams::default());
    }

    fn commit(&mut self) {
        if self.dirty {
            self.update();
        }
    }
}

// ---------------------------------------------------------------------------------------------------------
// TAPE-DL
// ---------------------------------------------------------------------------------------------------------

/// `TAPE-DL` schema slots.
pub mod tape_slot {
    pub const TIME: u16 = 1;
    pub const SUST: u16 = 2;
    pub const DRV: u16 = 3;
    pub const WF: u16 = 4;
}

/// `TAPE-DL` parameters.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TapeDelayParams {
    pub time_ms: f32,
    pub sust_pct: f32,
    pub drv_pct: f32,
    pub wf_pct: f32,
}

impl Default for TapeDelayParams {
    /// Schema defaults: 300 ms, 20 %, 0 %, 0 %.
    fn default() -> Self {
        Self {
            time_ms: 300.0,
            sust_pct: 20.0,
            drv_pct: 0.0,
            wf_pct: 0.0,
        }
    }
}

/// Head-bump peak gain (dB) [I].
const TAPE_BUMP_DB: f64 = 3.0;
/// Fractional transport-speed deviation at FLUTTER 100 %: wow, flutter, drift [I].
const TAPE_WOW: f32 = 0.002;
const TAPE_FLUTTER: f32 = 0.0002;
const TAPE_DRIFT: f32 = 0.001;
/// Longest tape delay incl. modulation headroom.
const TAPE_MAX_S: f64 = 0.7;

/// Tape-speed dependent playback filters for a delay of `d_ms`: (HP1, head bump, HF-loss LP).
fn tape_filters(d_ms: f32, fs: f64) -> (Coeffs, Coeffs, Coeffs) {
    let speed = 300.0 / d_ms.max(1.0);
    let hp = Coeffs::highpass1(fs, 40.0);
    let bump = Coeffs::peak(fs, fc(60.0 * speed, 25.0, 300.0, fs), 1.0, TAPE_BUMP_DB);
    let lp = Coeffs::lowpass(fs, fc(5000.0 * speed.powf(0.7), 1500.0, 12_000.0, fs), 0.6);
    (hp, bump, lp)
}

/// TAPE DELAY (`TAPE-DL`); see the module doc.
pub struct TapeDelay {
    p: TapeDelayParams,
    dirty: bool,
    fs: f64,
    lines: [DelayLine; 2],
    glide: Glide,
    fresh: bool,
    /// Delay the playback filters were last computed for.
    coeff_d: f32,
    feed: f32,
    feed_s: f32,
    pre: f32,
    pre_s: f32,
    wf: f32,
    wf_s: f32,
    sm: f32,
    wow: Lfo,
    flutter: Lfo,
    drift: Drift,
    hp: [Biquad; 2],
    bump: [Biquad; 2],
    lp: [Biquad; 2],
}

impl Default for TapeDelay {
    fn default() -> Self {
        Self::new()
    }
}

impl TapeDelay {
    pub fn new() -> Self {
        Self {
            p: TapeDelayParams::default(),
            dirty: true,
            fs: 48_000.0,
            lines: [DelayLine::new(1), DelayLine::new(1)],
            glide: Glide::new(0.6),
            fresh: true,
            coeff_d: 0.0,
            feed: 0.0,
            feed_s: 0.0,
            pre: 0.5,
            pre_s: 0.5,
            wf: 0.0,
            wf_s: 0.0,
            sm: 1.0,
            wow: Lfo::default(),
            flutter: Lfo::default(),
            drift: Drift::new(0x1234_5678),
            hp: [Biquad::default(); 2],
            bump: [Biquad::default(); 2],
            lp: [Biquad::default(); 2],
        }
    }

    pub fn params(&self) -> &TapeDelayParams {
        &self.p
    }

    pub fn set_params(&mut self, p: &TapeDelayParams) {
        self.p = *p;
        self.dirty = true;
    }

    /// Current (gliding) delay in samples.
    pub fn delay_samples(&self) -> f32 {
        self.glide.value()
    }

    fn update(&mut self) {
        self.dirty = false;
        let fs = self.fs as f32;
        self.glide.target = (self.p.time_ms.clamp(60.0, 650.0) * 0.001 * fs) as f64;
        self.feed = self.p.sust_pct.clamp(0.0, 100.0) * 0.01 * 0.95
            / 10f32.powf(TAPE_BUMP_DB as f32 / 20.0);
        self.pre = 0.5 * 10f32.powf(self.p.drv_pct.clamp(0.0, 100.0) * 0.01 * 24.0 / 20.0);
        self.wf = self.p.wf_pct.clamp(0.0, 100.0) * 0.01;
        if self.fresh {
            self.glide.value = self.glide.target;
            self.feed_s = self.feed;
            self.pre_s = self.pre;
            self.wf_s = self.wf;
            self.set_filters(self.glide.value());
        }
    }

    fn set_filters(&mut self, d: f32) {
        self.coeff_d = d;
        let (hp, bump, lp) = tape_filters(d * 1000.0 / self.fs as f32, self.fs);
        for k in 0..2 {
            self.hp[k].set_coeffs(hp);
            self.bump[k].set_coeffs(bump);
            self.lp[k].set_coeffs(lp);
        }
    }
}

impl Model for TapeDelay {
    fn id(&self) -> &'static str {
        "TAPE-DL"
    }

    fn prepare(&mut self, sample_rate: f64, _max_block: usize) {
        self.fs = sample_rate;
        let n = (TAPE_MAX_S * sample_rate).ceil() as usize + 4;
        self.lines = [DelayLine::new(n), DelayLine::new(n)];
        self.glide.coeff = 1.0 - one_pole_coeff(0.25, sample_rate);
        self.sm = step_coeff(SMOOTH_S, sample_rate);
        self.wow.prepare(0.7, sample_rate);
        self.flutter.prepare(7.3, sample_rate);
        self.drift.prepare(0.4, sample_rate);
        self.fresh = true;
        self.update();
        self.reset();
    }

    fn process(&mut self, left: &mut [f32], right: &mut [f32], _sc: Option<(&[f32], &[f32])>) {
        if self.dirty {
            self.update();
        }
        if (self.glide.value() - self.coeff_d).abs() > 0.002 * self.coeff_d {
            self.set_filters(self.glide.value());
        }
        self.fresh = left.is_empty() && self.fresh;
        let sm = self.sm;
        for (xl, xr) in left.iter_mut().zip(right.iter_mut()) {
            let d = self.glide.tick();
            self.feed_s += sm * (self.feed - self.feed_s);
            self.pre_s += sm * (self.pre - self.pre_s);
            self.wf_s += sm * (self.wf - self.wf_s);
            let (w, f, dr) = (self.wow.tick(), self.flutter.tick(), self.drift.tick());
            let dd = d * (1.0 + self.wf_s * (TAPE_WOW * w + TAPE_FLUTTER * f + TAPE_DRIFT * dr));
            let (g, pre) = (self.feed_s, self.pre_s);
            let inv = 1.0 / pre;
            for (k, x) in [xl, xr].into_iter().enumerate() {
                let y = self.lines[k].read_lagrange3(dd - 1.0);
                let out = self.lp[k].tick(self.bump[k].tick(self.hp[k].tick(y)));
                self.lines[k].push(sat(pre * (*x + g * out)) * inv);
                *x = out;
            }
        }
        flush(&mut self.hp);
        flush(&mut self.bump);
        flush(&mut self.lp);
    }

    fn reset(&mut self) {
        self.fresh = true;
        self.update();
        for l in &mut self.lines {
            l.reset();
        }
        self.wow.phase = 0.0;
        self.flutter.phase = 0.25;
        self.drift.reset(0x1234_5678);
        reset_bq(&mut self.hp);
        reset_bq(&mut self.bump);
        reset_bq(&mut self.lp);
    }

    fn set_param(&mut self, slot: u16, v: ParamValue) {
        let p = &mut self.p;
        match slot {
            tape_slot::TIME => p.time_ms = v.as_f32(),
            tape_slot::SUST => p.sust_pct = v.as_f32(),
            tape_slot::DRV => p.drv_pct = v.as_f32(),
            tape_slot::WF => p.wf_pct = v.as_f32(),
            _ => return,
        }
        self.dirty = true;
    }

    fn param(&self, slot: u16) -> Option<f32> {
        let p = &self.p;
        Some(match slot {
            tape_slot::TIME => p.time_ms,
            tape_slot::SUST => p.sust_pct,
            tape_slot::DRV => p.drv_pct,
            tape_slot::WF => p.wf_pct,
            _ => return None,
        })
    }

    fn load_defaults(&mut self) {
        self.set_params(&TapeDelayParams::default());
    }

    fn commit(&mut self) {
        if self.dirty {
            self.update();
        }
    }
}

// ---------------------------------------------------------------------------------------------------------
// OILCAN
// ---------------------------------------------------------------------------------------------------------

/// `OILCAN` schema slots.
pub mod oil_slot {
    pub const TIME: u16 = 1;
    pub const SUST: u16 = 2;
    pub const WB: u16 = 3;
    pub const TONE: u16 = 4;
}

/// `OILCAN` parameters (0..10 knobs).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OilcanParams {
    pub time: f32,
    pub sust: f32,
    pub wb: f32,
    pub tone: f32,
}

impl Default for OilcanParams {
    /// Schema defaults: 6.5, 4, 0, 5.
    fn default() -> Self {
        Self {
            time: 6.5,
            sust: 4.0,
            wb: 0.0,
            tone: 5.0,
        }
    }
}

impl OilcanParams {
    /// TIME knob -> delay in ms (40..500, exponential) [I].
    pub fn delay_ms(&self) -> f32 {
        40.0 * 12.5f32.powf(self.time.clamp(0.0, 10.0) * 0.1)
    }

    /// TONE knob -> loop low-pass cutoff (800..6400 Hz) [I].
    pub fn tone_hz(&self) -> f32 {
        800.0 * 8f32.powf(self.tone.clamp(0.0, 10.0) * 0.1)
    }
}

/// Read smear (+- ms) of the oil-can pickup [I].
const OIL_SMEAR_MS: f32 = 0.6;
/// Wobble depth at WOBBLE 10 (ms) [I].
const OIL_WOBBLE_MS: f32 = 1.2;
const OIL_MAX_S: f64 = 0.51;
/// Record-path drive into the saturator.
const OIL_DRIVE: f32 = 0.8;

/// OILCAN DELAY (`OILCAN`); see the module doc.
pub struct Oilcan {
    p: OilcanParams,
    dirty: bool,
    fs: f64,
    lines: [DelayLine; 2],
    glide: Glide,
    fresh: bool,
    feed: f32,
    feed_s: f32,
    wb: f32,
    wb_s: f32,
    sm: f32,
    smear: f32,
    wob_depth: f32,
    lfo: Lfo,
    am: Lfo,
    drift: Drift,
    hp: [Biquad; 2],
    lp: [Biquad; 2],
}

impl Default for Oilcan {
    fn default() -> Self {
        Self::new()
    }
}

impl Oilcan {
    pub fn new() -> Self {
        Self {
            p: OilcanParams::default(),
            dirty: true,
            fs: 48_000.0,
            lines: [DelayLine::new(1), DelayLine::new(1)],
            glide: Glide::new(0.6),
            fresh: true,
            feed: 0.0,
            feed_s: 0.0,
            wb: 0.0,
            wb_s: 0.0,
            sm: 1.0,
            smear: 1.0,
            wob_depth: 1.0,
            lfo: Lfo::default(),
            am: Lfo::default(),
            drift: Drift::new(0x0bad_cafe),
            hp: [Biquad::default(); 2],
            lp: [Biquad::default(); 2],
        }
    }

    pub fn params(&self) -> &OilcanParams {
        &self.p
    }

    pub fn set_params(&mut self, p: &OilcanParams) {
        self.p = *p;
        self.dirty = true;
    }

    /// Current (gliding) delay in samples.
    pub fn delay_samples(&self) -> f32 {
        self.glide.value()
    }

    fn update(&mut self) {
        self.dirty = false;
        let fs = self.fs;
        self.glide.target = self.p.delay_ms() as f64 * 0.001 * fs;
        self.feed = self.p.sust.clamp(0.0, 10.0) * 0.1 * 0.92;
        self.wb = self.p.wb.clamp(0.0, 10.0) * 0.1;
        let lp = Coeffs::lowpass(fs, fc(self.p.tone_hz(), 200.0, 20_000.0, fs), FRAC_1_SQRT_2);
        let hp = Coeffs::highpass(fs, 160.0, FRAC_1_SQRT_2);
        for k in 0..2 {
            self.hp[k].set_coeffs(hp);
            self.lp[k].set_coeffs(lp);
        }
        if self.fresh {
            self.glide.value = self.glide.target;
            self.feed_s = self.feed;
            self.wb_s = self.wb;
        }
    }
}

impl Model for Oilcan {
    fn id(&self) -> &'static str {
        "OILCAN"
    }

    fn prepare(&mut self, sample_rate: f64, _max_block: usize) {
        self.fs = sample_rate;
        let n = (OIL_MAX_S * sample_rate).ceil() as usize + 4;
        self.lines = [DelayLine::new(n), DelayLine::new(n)];
        self.glide.coeff = 1.0 - one_pole_coeff(0.2, sample_rate);
        self.sm = step_coeff(SMOOTH_S, sample_rate);
        self.smear = OIL_SMEAR_MS * 0.001 * sample_rate as f32;
        self.wob_depth = OIL_WOBBLE_MS * 0.001 * sample_rate as f32;
        self.lfo.prepare(5.3, sample_rate);
        self.am.prepare(2.1, sample_rate);
        self.drift.prepare(3.0, sample_rate);
        self.fresh = true;
        self.update();
        self.reset();
    }

    fn process(&mut self, left: &mut [f32], right: &mut [f32], _sc: Option<(&[f32], &[f32])>) {
        if self.dirty {
            self.update();
        }
        self.fresh = left.is_empty() && self.fresh;
        let (sm, s) = (self.sm, self.smear);
        for (xl, xr) in left.iter_mut().zip(right.iter_mut()) {
            let d = self.glide.tick();
            self.feed_s += sm * (self.feed - self.feed_s);
            self.wb_s += sm * (self.wb - self.wb_s);
            let wob = 0.7 * self.lfo.tick() + 0.3 * self.drift.tick();
            let am_lfo = self.am.tick();
            let dd = d + self.wb_s * self.wob_depth * wob;
            let gain = 1.0 - self.wb_s * 0.12 * (0.5 + 0.5 * am_lfo);
            let g = self.feed_s;
            for (k, x) in [xl, xr].into_iter().enumerate() {
                let line = &self.lines[k];
                let y = 0.25 * line.read_linear(dd - s - 1.0)
                    + 0.5 * line.read_linear(dd - 1.0)
                    + 0.25 * line.read_linear(dd + s - 1.0);
                let out = gain * self.lp[k].tick(self.hp[k].tick(y));
                self.lines[k].push(sat(OIL_DRIVE * (*x + g * out)) / OIL_DRIVE);
                *x = out;
            }
        }
        flush(&mut self.hp);
        flush(&mut self.lp);
    }

    fn reset(&mut self) {
        self.fresh = true;
        self.update();
        for l in &mut self.lines {
            l.reset();
        }
        self.lfo.phase = 0.0;
        self.am.phase = 0.0;
        self.drift.reset(0x0bad_cafe);
        reset_bq(&mut self.hp);
        reset_bq(&mut self.lp);
    }

    fn set_param(&mut self, slot: u16, v: ParamValue) {
        let p = &mut self.p;
        match slot {
            oil_slot::TIME => p.time = v.as_f32(),
            oil_slot::SUST => p.sust = v.as_f32(),
            oil_slot::WB => p.wb = v.as_f32(),
            oil_slot::TONE => p.tone = v.as_f32(),
            _ => return,
        }
        self.dirty = true;
    }

    fn param(&self, slot: u16) -> Option<f32> {
        let p = &self.p;
        Some(match slot {
            oil_slot::TIME => p.time,
            oil_slot::SUST => p.sust,
            oil_slot::WB => p.wb,
            oil_slot::TONE => p.tone,
            _ => return None,
        })
    }

    fn load_defaults(&mut self) {
        self.set_params(&OilcanParams::default());
    }

    fn commit(&mut self) {
        if self.dirty {
            self.update();
        }
    }
}

// ---------------------------------------------------------------------------------------------------------
// BBD-DL
// ---------------------------------------------------------------------------------------------------------

/// `BBD-DL` schema slots.
pub mod bbd_slot {
    pub const DLY: u16 = 1;
    pub const FB: u16 = 2;
}

/// `BBD-DL` parameters.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BbdParams {
    pub dly: f32,
    pub fb_pct: f32,
}

impl Default for BbdParams {
    /// Schema defaults: 65, 30 %.
    fn default() -> Self {
        Self {
            dly: 65.0,
            fb_pct: 30.0,
        }
    }
}

/// BBD stages (MN3005-class).
pub const BBD_STAGES: f32 = 4096.0;
/// Fixed anti-alias / reconstruction bandwidth cap (Hz) [I].
pub const BBD_BW_MAX: f32 = 5000.0;
/// Filter cutoff as a fraction of the clock frequency [I].
pub const BBD_BW_CLOCK: f32 = 0.38;
const BBD_MAX_S: f64 = 0.31;
/// Compander reference level and detector floor (= compressor gain limit `REF / FLOOR` = +40 dB).
const BBD_REF: f32 = 0.25;
const BBD_FLOOR: f32 = BBD_REF / 100.0;
/// BBD headroom (compressed domain).
const BBD_CLIP: f32 = 1.5;
/// Hiss and clock feed-through levels in the compressed domain [I].
const BBD_HISS: f32 = 1.3e-4;
const BBD_CLOCK: f32 = 1e-3;

impl BbdParams {
    /// DELAY knob -> delay in ms (20..300, exponential) [I].
    pub fn delay_ms(&self) -> f32 {
        20.0 * 15f32.powf(self.dly.clamp(0.0, 100.0) * 0.01)
    }

    /// BBD clock (Hz) for a delay of `ms`.
    pub fn clock_hz(ms: f32) -> f32 {
        BBD_STAGES / (2.0 * ms * 0.001)
    }

    /// Anti-alias / reconstruction cutoff (Hz) for a delay of `ms` (before the `0.45 fs` clamp).
    pub fn bandwidth_hz(ms: f32) -> f32 {
        BBD_BW_MAX.min(BBD_BW_CLOCK * Self::clock_hz(ms))
    }
}

/// BBD DELAY (`BBD-DL`); see the module doc.
pub struct BbdDelay {
    p: BbdParams,
    dirty: bool,
    fs: f64,
    lines: [DelayLine; 2],
    glide: Glide,
    fresh: bool,
    feed: f32,
    feed_s: f32,
    sm: f32,
    env_a: f32,
    /// Cutoff the filters were last computed for.
    bw: f32,
    aa: [[Biquad; 2]; 2],
    rc: [[Biquad; 2]; 2],
    env_c: [f32; 2],
    env_e: [f32; 2],
    held: [f32; 2],
    sh_phase: f32,
    clk_phase: f32,
    rng: Rng,
}

impl Default for BbdDelay {
    fn default() -> Self {
        Self::new()
    }
}

impl BbdDelay {
    pub fn new() -> Self {
        Self {
            p: BbdParams::default(),
            dirty: true,
            fs: 48_000.0,
            lines: [DelayLine::new(1), DelayLine::new(1)],
            glide: Glide::new(0.8),
            fresh: true,
            feed: 0.0,
            feed_s: 0.0,
            sm: 1.0,
            env_a: 1.0,
            bw: 0.0,
            aa: [[Biquad::default(); 2]; 2],
            rc: [[Biquad::default(); 2]; 2],
            env_c: [0.0; 2],
            env_e: [0.0; 2],
            held: [0.0; 2],
            sh_phase: 0.0,
            clk_phase: 0.0,
            rng: Rng(0x2545_f491),
        }
    }

    pub fn params(&self) -> &BbdParams {
        &self.p
    }

    pub fn set_params(&mut self, p: &BbdParams) {
        self.p = *p;
        self.dirty = true;
    }

    /// Current (gliding) delay in samples.
    pub fn delay_samples(&self) -> f32 {
        self.glide.value()
    }

    /// Current anti-alias / reconstruction cutoff (Hz).
    pub fn bandwidth(&self) -> f32 {
        self.bw
    }

    fn update(&mut self) {
        self.dirty = false;
        self.glide.target = self.p.delay_ms() as f64 * 0.001 * self.fs;
        self.feed = self.p.fb_pct.clamp(0.0, 100.0) * 0.01 * 0.95;
        if self.fresh {
            self.glide.value = self.glide.target;
            self.feed_s = self.feed;
            self.set_filters();
        }
    }

    fn set_filters(&mut self) {
        let fs = self.fs;
        let ms = self.glide.value() * 1000.0 / fs as f32;
        let f = fc(BbdParams::bandwidth_hz(ms), 200.0, 20_000.0, fs);
        self.bw = f as f32;
        // 4th-order Butterworth
        let c1 = Coeffs::lowpass(fs, f, 0.541_196_1);
        let c2 = Coeffs::lowpass(fs, f, 1.306_563);
        for k in 0..2 {
            self.aa[k][0].set_coeffs(c1);
            self.aa[k][1].set_coeffs(c2);
            self.rc[k][0].set_coeffs(c1);
            self.rc[k][1].set_coeffs(c2);
        }
    }
}

impl Model for BbdDelay {
    fn id(&self) -> &'static str {
        "BBD-DL"
    }

    fn prepare(&mut self, sample_rate: f64, _max_block: usize) {
        self.fs = sample_rate;
        let n = (BBD_MAX_S * sample_rate).ceil() as usize + 4;
        self.lines = [DelayLine::new(n), DelayLine::new(n)];
        self.glide.coeff = 1.0 - one_pole_coeff(0.1, sample_rate);
        self.sm = step_coeff(SMOOTH_S, sample_rate);
        self.env_a = step_coeff(0.01, sample_rate);
        self.fresh = true;
        self.update();
        self.reset();
    }

    fn process(&mut self, left: &mut [f32], right: &mut [f32], _sc: Option<(&[f32], &[f32])>) {
        if self.dirty {
            self.update();
        }
        let ms = self.glide.value() * 1000.0 / self.fs as f32;
        if (BbdParams::bandwidth_hz(ms).min((self.fs * 0.45) as f32) - self.bw).abs()
            > 0.002 * self.bw
        {
            self.set_filters();
        }
        self.fresh = left.is_empty() && self.fresh;
        let (sm, ea) = (self.sm, self.env_a);
        for (xl, xr) in left.iter_mut().zip(right.iter_mut()) {
            let d = self.glide.tick();
            self.feed_s += sm * (self.feed - self.feed_s);
            // clock / fs = stages / (2 d)
            let ratio = BBD_STAGES / (2.0 * d);
            let mut tick = true;
            if ratio < 1.0 {
                self.sh_phase += ratio;
                tick = self.sh_phase >= 1.0;
                if tick {
                    self.sh_phase -= 1.0;
                }
            }
            let mut noise = BBD_HISS * self.rng.next();
            if ratio < 0.45 {
                self.clk_phase += ratio;
                if self.clk_phase >= 1.0 {
                    self.clk_phase -= 1.0;
                }
                noise += BBD_CLOCK * (TAU * self.clk_phase).sin();
            }
            let g = self.feed_s;
            for (k, x) in [xl, xr].into_iter().enumerate() {
                let y = self.lines[k].read_linear(d - 1.0) + noise;
                let z = self.rc[k][0].tick(y);
                let z = self.rc[k][1].tick(z);
                // expander: gain = its own (previous) detector of the compressed signal
                let out = z * self.env_e[k].max(BBD_FLOOR) * (1.0 / BBD_REF);
                self.env_e[k] += ea * (z.abs() - self.env_e[k]);
                let w = self.aa[k][0].tick(*x + g * out);
                let w = self.aa[k][1].tick(w);
                // feedback-detector compressor (NE570): |c| settles at sqrt(REF * |w|); the expander's
                // detector sees the same signal one BBD delay later, so the pair tracks
                let c = w * BBD_REF / self.env_c[k].max(BBD_FLOOR);
                let c = BBD_CLIP * sat(c * (1.0 / BBD_CLIP));
                self.env_c[k] += ea * (c.abs() - self.env_c[k]);
                if tick {
                    self.held[k] = c;
                }
                self.lines[k].push(self.held[k]);
                *x = out;
            }
        }
        for k in 0..2 {
            flush(&mut self.aa[k]);
            flush(&mut self.rc[k]);
        }
    }

    fn reset(&mut self) {
        self.fresh = true;
        self.update();
        for l in &mut self.lines {
            l.reset();
        }
        for k in 0..2 {
            reset_bq(&mut self.aa[k]);
            reset_bq(&mut self.rc[k]);
        }
        self.env_c = [0.0; 2];
        self.env_e = [0.0; 2];
        self.held = [0.0; 2];
        self.sh_phase = 0.0;
        self.clk_phase = 0.0;
        self.rng = Rng(0x2545_f491);
    }

    fn set_param(&mut self, slot: u16, v: ParamValue) {
        match slot {
            bbd_slot::DLY => self.p.dly = v.as_f32(),
            bbd_slot::FB => self.p.fb_pct = v.as_f32(),
            _ => return,
        }
        self.dirty = true;
    }

    fn param(&self, slot: u16) -> Option<f32> {
        Some(match slot {
            bbd_slot::DLY => self.p.dly,
            bbd_slot::FB => self.p.fb_pct,
            _ => return None,
        })
    }

    fn load_defaults(&mut self) {
        self.set_params(&BbdParams::default());
    }

    fn commit(&mut self) {
        if self.dirty {
            self.update();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FS: f64 = 48_000.0;

    /// Wide-open filters so echoes are (nearly) unfiltered impulses.
    fn open(p: &mut StereoDelayParams) {
        p.flc_hz = 20.0;
        p.fhc_hz = 20_000.0;
        p.lc_hz = 20.0;
        p.hc_hz = 20_000.0;
    }

    fn run(m: &mut dyn Model, l: &mut [f32], r: &mut [f32]) {
        for (a, b) in l.chunks_mut(64).zip(r.chunks_mut(64)) {
            m.process(a, b, None);
        }
    }

    fn peak_at(x: &[f32]) -> usize {
        x.iter()
            .enumerate()
            .max_by(|a, b| a.1.abs().total_cmp(&b.1.abs()))
            .unwrap()
            .0
    }

    fn peak(x: &[f32]) -> f32 {
        x.iter().fold(0f32, |a, v| a.max(v.abs()))
    }

    /// Hann-windowed sine burst (`len` samples, peak `amp`) at the start of an `n`-sample buffer.
    fn burst_f(n: usize, len: usize, hz: f32, amp: f32) -> Vec<f32> {
        let mut x = vec![0.0f32; n];
        for (i, v) in x.iter_mut().take(len).enumerate() {
            let w = 0.5 - 0.5 * (TAU * i as f32 / len as f32).cos();
            *v = amp * w * (TAU * hz * i as f32 / FS as f32).sin();
        }
        x
    }

    /// 1 kHz Hann-windowed burst (96 samples, peak ~1) at the start of an `n`-sample buffer.
    fn burst(n: usize) -> Vec<f32> {
        burst_f(n, 96, 1000.0, 1.0)
    }

    /// Peak in the window where echo `k` of a 240-sample delay lands.
    fn echo(x: &[f32], k: usize) -> f32 {
        peak(&x[k * 240..k * 240 + 96])
    }

    /// Energy centroid (samples) of `x[a..b]`.
    fn centroid(x: &[f32], a: usize, b: usize) -> f32 {
        let (mut s, mut w) = (0.0f64, 0.0f64);
        for (i, v) in x[a..b].iter().enumerate() {
            let e = (*v as f64) * (*v as f64);
            s += e * (a + i) as f64;
            w += e;
        }
        (s / w.max(1e-30)) as f32
    }

    #[test]
    fn echo_times_follow_time_factor_pattern_offset() {
        let mut m = StereoDelay::new();
        let mut p = StereoDelayParams {
            time_ms: 10.0,
            fact: 10, // x2
            pat: 0,   // 1/2 : 1
            offs_ms: 1.0,
            feed_pct: 0.0,
            ..Default::default()
        };
        open(&mut p);
        m.set_params(&p);
        m.prepare(FS, 64);
        let n = 4096;
        let (mut l, mut r) = (vec![0.0f32; n], vec![0.0f32; n]);
        l[0] = 1.0;
        r[0] = 1.0;
        run(&mut m, &mut l, &mut r);
        // L = 10 * 2 * 0.5 = 10 ms = 480 samples, R = 20 ms + 1 ms = 1008 samples (filter group delay
        // shifts the impulse peak by a sample or two)
        assert!((peak_at(&l) as i64 - 481).abs() <= 2, "L {}", peak_at(&l));
        assert!((peak_at(&r) as i64 - 1009).abs() <= 2, "R {}", peak_at(&r));
        assert!(
            l[..470].iter().all(|v| v.abs() < 1e-3),
            "no dry signal in the wet output"
        );
    }

    #[test]
    fn st_dl_dotted_and_triplet_factors() {
        // 500 ms quarter: 3/4 = dotted eighth 375 ms, 1/3 = eighth triplet 166.67 ms, 3/8 = dotted 16th
        for (fact, ms) in [(4usize, 375.0f32), (1, 500.0 / 3.0), (6, 187.5), (9, 750.0)] {
            let p = StereoDelayParams {
                time_ms: 500.0,
                fact,
                ..Default::default()
            };
            let (l, r) = p.delays_ms();
            assert!(
                (l - ms).abs() < 1e-3 && (r - ms).abs() < 1e-3,
                "{fact}: {l}"
            );
        }
    }

    #[test]
    fn feedback_repeats_and_decays() {
        let mut m = StereoDelay::new();
        let mut p = StereoDelayParams {
            time_ms: 5.0,
            feed_pct: 50.0,
            ..Default::default()
        };
        open(&mut p);
        m.set_params(&p);
        m.prepare(FS, 64);
        let n = 2048;
        let x = burst(n);
        let (mut l, mut r) = (x.clone(), vec![0.0f32; n]);
        let dry = echo(&x, 0);
        run(&mut m, &mut l, &mut r);
        let (e1, e2, e3) = (echo(&l, 1), echo(&l, 2), echo(&l, 3));
        assert!((e1 / dry - 1.0).abs() < 0.03, "{e1} vs {dry}");
        assert!((e2 / e1 - 0.5).abs() < 0.03, "{e2} / {e1}");
        assert!((e3 / e2 - 0.5).abs() < 0.03, "{e3} / {e2}");
        assert!(echo(&l, 0) < 1e-3, "wet only");
        assert!(r.iter().all(|v| v.abs() < 1e-6), "ST: no crosstalk");
    }

    /// The feedback filters act once per repeat: echo k+1 / echo k = feed * |H_flc(f) H_fhc(f)|.
    #[test]
    fn st_dl_filters_in_the_loop() {
        let mut m = StereoDelay::new();
        let p = StereoDelayParams {
            time_ms: 40.0,
            feed_pct: 80.0,
            flc_hz: 300.0,
            fhc_hz: 1000.0,
            lc_hz: 20.0,
            hc_hz: 20_000.0,
            ..Default::default()
        };
        for hz in [150.0f32, 1000.0, 4000.0] {
            m.set_params(&p);
            m.prepare(FS, 64);
            let n = 48_000;
            let x = burst_f(n, 960, hz, 0.5);
            let (mut l, mut r) = (x.clone(), vec![0.0f32; n]);
            run(&mut m, &mut l, &mut r);
            let e = |k: usize| peak(&l[k * 1920..k * 1920 + 1200]);
            let loop_db = Coeffs::highpass1(FS, 300.0).magnitude_db(FS, hz as f64)
                + Coeffs::lowpass1(FS, 1000.0).magnitude_db(FS, hz as f64);
            let want = 0.8 * 10f32.powf(loop_db as f32 / 20.0);
            let got = e(3) / e(2);
            assert!((got / want - 1.0).abs() < 0.06, "{hz} Hz: {got} vs {want}");
        }
    }

    #[test]
    fn cross_mode_ping_pongs_and_mono_sums() {
        let mut m = StereoDelay::new();
        let mut p = StereoDelayParams {
            time_ms: 5.0,
            feed_pct: 50.0,
            mode: DelayMode::Cross,
            ..Default::default()
        };
        open(&mut p);
        m.set_params(&p);
        m.prepare(FS, 64);
        let n = 1024;
        let x = burst(n);
        let dry = echo(&x, 0);
        let (mut l, mut r) = (x.clone(), vec![0.0f32; n]);
        run(&mut m, &mut l, &mut r);
        // first echo on L, second on R (fed back from L), third on L again
        assert!((echo(&l, 1) / dry - 1.0).abs() < 0.03);
        assert!(echo(&r, 1) < 1e-3 && echo(&l, 2) < 1e-3);
        assert!((echo(&r, 2) / dry - 0.5).abs() < 0.03, "{}", echo(&r, 2));
        assert!((echo(&l, 3) / dry - 0.25).abs() < 0.03, "{}", echo(&l, 3));

        m.set_param(slot::MODE, ParamValue::Float(2.0));
        m.set_param(slot::FEED, ParamValue::Float(0.0));
        m.reset();
        let (mut l, mut r) = (x.clone(), vec![0.0f32; n]);
        run(&mut m, &mut l, &mut r);
        let (a, b) = (echo(&l, 1) / dry, echo(&r, 1) / dry);
        assert!((a - 0.5).abs() < 0.03 && (b - 0.5).abs() < 0.03, "{a} {b}");
    }

    /// A time change crossfades over [`XFADE_S`]: an impulse right after the change is heard from the new
    /// head (fading in) and from the old head (fading out), weighted by the fade position.
    #[test]
    fn st_dl_time_change_crossfades() {
        let mut m = StereoDelay::new();
        let mut p = StereoDelayParams {
            time_ms: 10.0,
            feed_pct: 0.0,
            ..Default::default()
        };
        open(&mut p);
        m.set_params(&p);
        m.prepare(FS, 64);
        let mut l = vec![0.0f32; 64];
        let mut r = l.clone();
        m.process(&mut l, &mut r, None);
        m.set_param(slot::TIME, ParamValue::Float(5.0));
        let n = 1200;
        let (mut l, mut r) = (vec![0.0f32; n], vec![0.0f32; n]);
        l[0] = 1.0;
        r[0] = 1.0;
        run(&mut m, &mut l, &mut r);
        // fade = 1440 samples: new head (240) at weight 240/1440, old head (480) at 1 - 480/1440
        let (new, old) = (peak(&l[230..250]), peak(&l[470..490]));
        assert!((new / old - 0.25).abs() < 0.01, "{new} {old}");
        assert!((m.delay_samples().0 - 240.0).abs() < 1e-3);
    }

    #[test]
    fn params_round_trip_and_defaults() {
        let mut m = StereoDelay::new();
        m.prepare(FS, 64);
        assert_eq!(m.param(slot::TIME), Some(333.0));
        m.set_param(slot::TIME, ParamValue::Float(100.0));
        m.set_param(slot::PAT, ParamValue::Enum(8));
        assert_eq!(m.param(slot::TIME), Some(100.0));
        assert_eq!(m.param(slot::PAT), Some(8.0));
        assert_eq!(m.param(99), None);
        m.commit();
        let (dl, dr) = m.delay_samples();
        assert!(
            (dl - 4800.0).abs() < 0.01 && (dr - 7200.0).abs() < 0.01,
            "{dl} {dr}"
        );
        m.load_defaults();
        assert_eq!(*m.params(), StereoDelayParams::default());
        // effective delay is clamped to the line length
        m.set_param(slot::TIME, ParamValue::Float(3000.0));
        m.set_param(slot::FACT, ParamValue::Enum(10));
        m.commit();
        assert!(m.delay_samples().0 <= (MAX_DELAY_S * FS) as f32 + 2.0);
        assert_eq!(m.latency(), 0);
    }

    // ---- TAP-DL ----

    fn tap_open() -> TapDelayParams {
        TapDelayParams {
            lc_hz: 20.0,
            hc_hz: 20_000.0,
            ..Default::default()
        }
    }

    #[test]
    fn tap_dl_tap_times_and_slope() {
        let mut m = TapDelay::new();
        let p = TapDelayParams {
            time_ms: 40.0,
            rep: 4,
            slp_db: -6.0,
            fact: 1, // 1/2 -> 20 ms length
            pdel_ms: 5.0,
            wid_pct: 0.0,
            ..tap_open()
        };
        m.set_params(&p);
        m.prepare(FS, 64);
        let n = 2400;
        let (mut l, mut r) = (vec![0.0f32; n], vec![0.0f32; n]);
        l[0] = 1.0;
        r[0] = 1.0;
        run(&mut m, &mut l, &mut r);
        // taps at 5 + 5, 10, 15, 20 ms = 480, 720, 960, 1200 samples; gains 0, -2, -4, -6 dB
        for (i, at) in [480usize, 720, 960, 1200].into_iter().enumerate() {
            let w = &l[at - 20..at + 20];
            let pk = peak_at(w) + at - 20;
            assert!((pk as i64 - at as i64).abs() <= 2, "tap {i} at {pk}");
            // relative to the first tap (the 20 kHz HI CUT rounds every impulse the same way)
            let want = 10f32.powf(-2.0 * i as f32 / 20.0);
            let got = peak(w) / peak(&l[460..500]);
            assert!((got / want - 1.0).abs() < 0.03, "tap {i}: {got}");
            assert!(
                (peak(&r[at - 20..at + 20]) - peak(w)).abs() < 1e-3,
                "centred"
            );
        }
        assert!(peak(&l[..470]) < 1e-3 && peak(&l[1220..]) < 1e-2);
        // positive slope: the last tap is the loudest at 0 dB
        let (k, t) = TapDelayParams { slp_db: 12.0, ..p }.taps();
        assert_eq!(k, 4);
        assert!((t[3].gain - 1.0).abs() < 1e-6 && (t[0].gain - 0.2512).abs() < 1e-3);
    }

    #[test]
    fn tap_dl_pan_modes() {
        let base = TapDelayParams {
            rep: 5,
            wid_pct: 100.0,
            ..Default::default()
        };
        let pans = |mode| {
            let (n, t) = TapDelayParams { mode, ..base }.taps();
            t[..n].iter().map(|t| t.pan).collect::<Vec<_>>()
        };
        assert_eq!(pans(TapMode::Move), [-1.0, -0.5, 0.0, 0.5, 1.0]);
        assert_eq!(pans(TapMode::Jump), [-1.0, 1.0, -1.0, 1.0, -1.0]);
        assert_eq!(pans(TapMode::Focus), [-1.0, 0.75, -0.5, 0.25, 0.0]);
        assert_eq!(pans(TapMode::Spread), [-0.0, 0.25, -0.5, 0.75, -1.0]);
        let (_, t) = TapDelayParams {
            wid_pct: -50.0,
            mode: TapMode::Jump,
            ..base
        }
        .taps();
        assert_eq!(t[0].pan, 0.5);

        // JUMP, full width: first tap only on L, second only on R
        let mut m = TapDelay::new();
        m.set_params(&TapDelayParams {
            time_ms: 20.0,
            rep: 2,
            slp_db: 0.0,
            mode: TapMode::Jump,
            wid_pct: 100.0,
            ..tap_open()
        });
        m.prepare(FS, 64);
        let n = 1200;
        let (mut l, mut r) = (vec![0.0f32; n], vec![0.0f32; n]);
        l[0] = 1.0;
        r[0] = 1.0;
        run(&mut m, &mut l, &mut r);
        // (the 20 Hz LO CUT leaves a small tail of the first tap on L)
        assert!(peak(&l[460..500]) > 0.5 && peak(&r[460..500]) < 1e-3);
        assert!(peak(&r[940..980]) > 0.5 && peak(&l[940..980]) < 1e-2);
    }

    #[test]
    fn tap_dl_diffusion_smears() {
        let ir = |diff: f32| {
            let mut m = TapDelay::new();
            m.set_params(&TapDelayParams {
                time_ms: 10.0,
                rep: 1,
                diff,
                ..tap_open()
            });
            m.prepare(FS, 64);
            let n = 4800;
            let (mut l, mut r) = (vec![0.0f32; n], vec![0.0f32; n]);
            l[0] = 1.0;
            r[0] = 1.0;
            run(&mut m, &mut l, &mut r);
            assert_eq!(m.param(tap_slot::DIFF), Some(diff));
            l
        };
        let (dry, wet) = (ir(0.0), ir(100.0));
        // energy spread over many samples (allpass: same energy), no single impulse left
        let e = |x: &[f32]| x.iter().map(|v| v * v).sum::<f32>();
        assert!(peak(&wet) < 0.5 * peak(&dry), "{}", peak(&wet));
        assert!(
            (e(&wet) / e(&dry) - 1.0).abs() < 0.1,
            "{} {}",
            e(&wet),
            e(&dry)
        );
        assert!(wet[..470].iter().all(|v| v.abs() < 1e-3));
    }

    // ---- TAPE-DL ----

    #[test]
    fn tape_dl_time_accuracy_and_glide() {
        let mut m = TapeDelay::new();
        m.set_params(&TapeDelayParams {
            time_ms: 100.0,
            sust_pct: 0.0,
            ..Default::default()
        });
        m.prepare(FS, 64);
        let n = 9600;
        let x = burst_f(n, 960, 1000.0, 0.3);
        let (mut l, mut r) = (x.clone(), x.clone());
        run(&mut m, &mut l, &mut r);
        let c = centroid(&l, 4000, 8000) - centroid(&x, 0, 960);
        assert!((c - 4800.0).abs() < 10.0, "{c}");
        // a time change glides (rate-limited), it does not jump
        m.set_param(tape_slot::TIME, ParamValue::Float(300.0));
        let mut b = vec![0.0f32; 64];
        let mut b2 = b.clone();
        m.process(&mut b, &mut b2, None);
        let d = m.delay_samples();
        assert!(d > 4800.0 && d <= 4800.0 + 0.6 * 64.0 + 0.01, "{d}");
        for _ in 0..2000 {
            m.process(&mut b, &mut b2, None);
        }
        assert!((m.delay_samples() - 14_400.0).abs() < 1.0);
    }

    /// High frequencies die faster in the loop than mids (tape HF loss); drive saturates.
    #[test]
    fn tape_dl_hf_loss_in_loop_and_drive() {
        let decay = |hz: f32| {
            let mut m = TapeDelay::new();
            m.set_params(&TapeDelayParams {
                time_ms: 300.0,
                sust_pct: 80.0,
                ..Default::default()
            });
            m.prepare(FS, 64);
            let n = 4 * 14_400;
            let x = burst_f(n, 2400, hz, 0.2);
            let (mut l, mut r) = (x.clone(), x.clone());
            run(&mut m, &mut l, &mut r);
            peak(&l[3 * 14_400..3 * 14_400 + 3000]) / peak(&l[14_400..14_400 + 3000])
        };
        let (mid, hi) = (decay(1000.0), decay(7000.0));
        assert!(mid > 0.2 && hi < 0.5 * mid, "{mid} {hi}");

        let level = |drv: f32| {
            let mut m = TapeDelay::new();
            m.set_params(&TapeDelayParams {
                time_ms: 100.0,
                sust_pct: 0.0,
                drv_pct: drv,
                ..Default::default()
            });
            m.prepare(FS, 64);
            let n = 9600;
            let x = burst_f(n, 2400, 500.0, 1.0);
            let (mut l, mut r) = (x.clone(), x.clone());
            run(&mut m, &mut l, &mut r);
            peak(&l)
        };
        assert!(level(100.0) < 0.6 * level(0.0));
    }

    #[test]
    fn tape_dl_flutter_modulates_pitch() {
        let mut m = TapeDelay::new();
        m.set_params(&TapeDelayParams {
            time_ms: 300.0,
            sust_pct: 0.0,
            wf_pct: 100.0,
            ..Default::default()
        });
        m.prepare(FS, 64);
        // a steady sine: with flutter the wet output deviates from a pure delayed copy
        let n = 96_000;
        let x: Vec<f32> = (0..n)
            .map(|i| 0.5 * (TAU * 1000.0 * i as f32 / FS as f32).sin())
            .collect();
        let (mut l, mut r) = (x.clone(), x.clone());
        run(&mut m, &mut l, &mut r);
        let mut m0 = TapeDelay::new();
        m0.set_params(&TapeDelayParams {
            time_ms: 300.0,
            sust_pct: 0.0,
            ..Default::default()
        });
        m0.prepare(FS, 64);
        let (mut l0, mut r0) = (x.clone(), x);
        run(&mut m0, &mut l0, &mut r0);
        let diff = l[48_000..]
            .iter()
            .zip(&l0[48_000..])
            .fold(0f32, |a, (p, q)| a.max((p - q).abs()));
        assert!(diff > 0.1, "{diff}");
    }

    // ---- OILCAN ----

    #[test]
    fn oilcan_time_mapping_and_tone() {
        let p = OilcanParams::default();
        assert!((p.delay_ms() - 40.0 * 12.5f32.powf(0.65)).abs() < 1e-3);
        assert_eq!(OilcanParams { time: 0.0, ..p }.delay_ms(), 40.0);
        assert!((OilcanParams { time: 10.0, ..p }.delay_ms() - 500.0).abs() < 0.01);
        assert!((OilcanParams { tone: 10.0, ..p }.tone_hz() - 6400.0).abs() < 0.1);

        let mut m = Oilcan::new();
        m.set_params(&OilcanParams {
            time: 3.0,
            sust: 0.0,
            ..p
        });
        m.prepare(FS, 64);
        let d = m.delay_samples();
        let n = 2 * d as usize + 2000;
        let x = burst_f(n, 960, 1000.0, 0.3);
        let (mut l, mut r) = (x.clone(), x.clone());
        run(&mut m, &mut l, &mut r);
        let c = centroid(&l, d as usize - 500, d as usize + 1600) - centroid(&x, 0, 960);
        assert!((c - d).abs() < 15.0, "{c} vs {d}");

        // TONE darkens: a 5 kHz burst is much quieter at tone 0 than at tone 10
        let lvl = |tone: f32| {
            let mut m = Oilcan::new();
            m.set_params(&OilcanParams {
                time: 3.0,
                sust: 0.0,
                tone,
                wb: 0.0,
            });
            m.prepare(FS, 64);
            let x = burst_f(n, 960, 5000.0, 0.3);
            let (mut l, mut r) = (x.clone(), x);
            run(&mut m, &mut l, &mut r);
            peak(&l)
        };
        assert!(lvl(0.0) < 0.2 * lvl(10.0));
    }

    // ---- BBD-DL ----

    #[test]
    fn bbd_time_clock_and_bandwidth() {
        let p = BbdParams::default();
        assert!((BbdParams { dly: 0.0, ..p }.delay_ms() - 20.0).abs() < 1e-3);
        assert!((BbdParams { dly: 100.0, ..p }.delay_ms() - 300.0).abs() < 0.01);
        // 300 ms: clock 6827 Hz -> 2594 Hz bandwidth; 20 ms: capped at 5 kHz
        assert!((BbdParams::clock_hz(300.0) - 6826.67).abs() < 0.1);
        assert!((BbdParams::bandwidth_hz(300.0) - 2594.1).abs() < 0.5);
        assert_eq!(BbdParams::bandwidth_hz(20.0), BBD_BW_MAX);

        let mut m = BbdDelay::new();
        m.set_params(&BbdParams {
            dly: 100.0,
            fb_pct: 0.0,
        });
        m.prepare(FS, 64);
        assert!((m.bandwidth() - 2594.1).abs() < 1.0);
        m.set_params(&BbdParams {
            dly: 0.0,
            fb_pct: 0.0,
        });
        m.reset();
        assert_eq!(m.bandwidth(), BBD_BW_MAX);
    }

    #[test]
    fn bbd_delay_accuracy_and_compander_unity() {
        for dly in [10.0f32, 65.0, 100.0] {
            let mut m = BbdDelay::new();
            m.set_params(&BbdParams { dly, fb_pct: 0.0 });
            m.prepare(FS, 64);
            let d = m.delay_samples() as usize;
            // delay by cross-correlation with steady noise (the compander mistracks bursts, by design)
            let n = d + 24_000;
            let mut rng = Rng(7);
            let x: Vec<f32> = (0..n).map(|_| 0.2 * rng.next()).collect();
            let (mut l, mut r) = (x.clone(), x.clone());
            run(&mut m, &mut l, &mut r);
            let xc = |lag: usize| -> f64 {
                (d + 4800..n).map(|i| l[i] as f64 * x[i - lag] as f64).sum()
            };
            let lag = (d - 50..d + 100)
                .max_by(|a, b| xc(*a).total_cmp(&xc(*b)))
                .unwrap();
            // AA + reconstruction filters add their group delay (<= 0.5 ms)
            assert!(lag >= d && lag <= d + 24, "dly {dly}: lag {lag} vs {d}");

            // the compander is transparent in steady state: wet level ~ dry level
            let mut m = BbdDelay::new();
            m.set_params(&BbdParams { dly, fb_pct: 0.0 });
            m.prepare(FS, 64);
            let x: Vec<f32> = (0..n)
                .map(|i| 0.3 * (TAU * 500.0 * i as f32 / FS as f32).sin())
                .collect();
            let (mut l, mut r) = (x.clone(), x.clone());
            run(&mut m, &mut l, &mut r);
            let g = peak(&l[n - 4800..]) / 0.3;
            assert!((g - 1.0).abs() < 0.05, "dly {dly}: gain {g}");
        }
    }

    /// Bandwidth falls with delay time: a 4 kHz tone is attenuated far more at 300 ms than at 30 ms.
    #[test]
    fn bbd_bandwidth_tracks_delay_and_noise_is_companded() {
        let lvl = |dly: f32| {
            let mut m = BbdDelay::new();
            m.set_params(&BbdParams { dly, fb_pct: 0.0 });
            m.prepare(FS, 64);
            let d = m.delay_samples() as usize;
            let n = d + 9600;
            let x: Vec<f32> = (0..n)
                .map(|i| 0.3 * (TAU * 4000.0 * i as f32 / FS as f32).sin())
                .collect();
            let (mut l, mut r) = (x.clone(), x);
            run(&mut m, &mut l, &mut r);
            let rms = |s: &[f32]| (s.iter().map(|v| v * v).sum::<f32>() / s.len() as f32).sqrt();
            rms(&l[d + 4800..])
        };
        let (short, long) = (lvl(15.0), lvl(100.0));
        assert!(long < 0.3 * short, "{short} {long}");

        // silence in: the expander keeps hiss / clock feed-through far down
        let mut m = BbdDelay::new();
        m.set_params(&BbdParams {
            dly: 100.0,
            fb_pct: 50.0,
        });
        m.prepare(FS, 64);
        let (mut l, mut r) = (vec![0.0f32; 48_000], vec![0.0f32; 48_000]);
        run(&mut m, &mut l, &mut r);
        assert!(peak(&l) < 1e-5, "{}", peak(&l));
    }
}
