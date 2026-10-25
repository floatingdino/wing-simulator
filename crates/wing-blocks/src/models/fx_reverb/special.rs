//! Special reverbs (T05S): `GATED`, `REVERSE`, `DEL/REV`, `SHIMMER` (models.md §6). Wet signal only (the FX slot
//! host blends by `fxmix`). The diffuse tail of every model is the shared FDN [`ReverbCore`] (8 lines); what is
//! special is the processing around it. Everything marked [I] is inferred (no console measurement yet).
//!
//! ```text
//! GATED   in -> pre-delay -+-> trigger (peak env > -40 dBFS, edge, re-arm below -46 dBFS) -> hold `dcy` -> gate
//!                          +-> dense FDN (short lines, T60 = 6 x dcy: ~-10 dB droop over the hold) -> x gate -> HI shelf
//! REVERSE in -> pre-delay -+-> trigger (level) -> capture W = dcy of FDN wet into buffer A (double buffer)
//!                          +-> FDN                A played back time-reversed (raised-cosine fade in, 3 ms cut) -> HI shelf
//! DEL/REV in -> stereo delay (TIME, FEED + HC one-pole in the loop) -> x DLY LVL -------------------------------> out
//!                  in x IN->REV + delay x DLY->REV -> FDN (PRE DLY, SIZE, DECAY, DAMPING, LO CUT) --------------> out
//! SHIMMER in -> pre-delay (0..250 ms) -> + fb -> FDN -> out;  fb = tanh(LP(HP(w12 x PS(+12) + w7 x PS(+7)))) x g
//!            PS = two-tap overlap-add grain shifter on the wet output, read >= 2 sub-blocks behind the write head
//! ```
//!
//! Parameter maps (raw WING value -> behaviour):
//!
//! | model | key | mapping |
//! |---|---|---|
//! | GATED | `pdel` | pre-delay 0..200 ms (own line, so the gate sees the delayed signal) |
//! | | `att` 0..30 | gate fade-in `max(att,1)` ms and build-up shape of the FDN (`0.1 + 0.4 att/30`) |
//! | | `dcy` 0.14..1 s | gate hold time from the trigger (the "gate length"); gate closes in 4 ms |
//! | | `dens` 1..50 | FDN line scale `0.9 - 0.65 (dens-1)/49` (longer lines = sparser) + ER level |
//! | | `diff` 1..30, `sprd` 0..100 | input diffusion `(diff-1)/29`, build-up spread `sprd/100` |
//! | | `lc`, `hsf`, `hsg` | 12 dB/oct low cut, high shelf (freq, 0..-30 dB) on the output |
//! | REVERSE | `pdel` | pre-delay; the swell starts `pdel + dcy` after the trigger |
//! | | `rise` 0..50 | fade-in of the reversed segment: `5 % + 75 % x rise/50` of its length |
//! | | `dcy` 0.14..1 s | segment length W = FDN T60 (the tail is captured for W, then reversed) |
//! | | `diff`, `sprd`, `lc`, `hsf`, `hsg` | as GATED |
//! | DEL/REV | `time` 1..3000 ms, `feed`, `fhc` | stereo delay time (glided), feedback 0..99.5 %, HC one-pole in loop |
//! | | `dly` | delay level to the output; `d2r` delay -> reverb send; `i2r` input -> reverb send |
//! | | `pdel`, `size`, `dcy`, `damp`, `rlc` | reverb pre-delay, size scale `0.25 + 0.95 (size-2)/98`, T60, HF damping freq, low cut |
//! | SHIMMER | `pdel` 0..250 | pre-delay before the loop |
//! | | `size` 1..50, `dcy` 1..20 | size scale `0.3 + 1.2 (size-1)/49`, T60 |
//! | | `lc`, `hc`, `damp` | output filters; damping freq `16 kHz x 0.1^(damp/100)` |
//! | | `shim` 0..100 | feedback gain of the pitched signal `0.5 shim/100` (soft-limited by tanh) |
//! | | `shine` 0..100 | mix of +7 (fifth) `0.5 shine/100` vs +12 (octave), and brightness LP `4 -> 12 kHz` |
//!
//! Latency: GATED, DEL/REV, SHIMMER have none beyond the pre-delay (>= 1 sample). REVERSE is inherently
//! non-causal: the reversed swell starts `pdel + W` after the trigger ([`Reverse::wet_latency_samples`]); the model
//! reports `latency() == 0` (it is a wet-only effect, the swell is the effect, not a delay to compensate).
//!
//! RT-safety: all buffers are allocated in `prepare`; `process`/`set_param`/`reset` never allocate.

// Per-channel / per-sample loops index several parallel arrays.
#![allow(clippy::needless_range_loop)]

use super::core::{Character, CoreParams, ReverbCore};
use super::{ps, ParamSpec, MAX_PARAMS};
use crate::model::{Model, ParamValue};
use wing_dsp::biquad::{Biquad, Coeffs};
use wing_dsp::delay::DelayLine;

/// Sub-block of the SHIMMER loop (samples); the grain shifter reads at least `2 x` this behind its write head.
const SB: usize = 32;
/// Gate / reverse trigger threshold (linear peak, -40 dBFS) and re-arm level (-46 dBFS). [I]
const TRIG_ON: f32 = 0.01;
const TRIG_OFF: f32 = 0.005;
const FLUSH: f32 = 1e-18;

fn glide_k(fs: f32, secs: f32) -> f32 {
    1.0 - (-1.0 / (secs * fs)).exp()
}

/// Raw WING values by slot, clamped to the spec ranges.
struct Raw {
    specs: &'static [ParamSpec],
    v: [f32; MAX_PARAMS],
}

impl Raw {
    fn new(specs: &'static [ParamSpec]) -> Self {
        let mut r = Self {
            specs,
            v: [0.0; MAX_PARAMS],
        };
        r.defaults();
        r
    }
    fn defaults(&mut self) {
        for (v, p) in self.v.iter_mut().zip(self.specs) {
            *v = p.default;
        }
    }
    fn set(&mut self, slot: u16, x: f32) -> bool {
        let i = slot as usize;
        if i == 0 || i > self.specs.len() || !x.is_finite() {
            return false;
        }
        let s = &self.specs[i - 1];
        self.v[i - 1] = x.clamp(s.min, s.max);
        true
    }
    fn get(&self, slot: u16) -> Option<f32> {
        let i = slot as usize;
        (i >= 1 && i <= self.specs.len()).then(|| self.v[i - 1])
    }
}

/// Stereo pre-delay with a glided time (Lagrange read; min 1 sample).
struct PreDelay {
    line: [DelayLine; 2],
    cur: f32,
    tgt: f32,
    k: f32,
}

impl PreDelay {
    fn new() -> Self {
        Self {
            line: [DelayLine::new(1), DelayLine::new(1)],
            cur: 1.0,
            tgt: 1.0,
            k: 0.01,
        }
    }
    fn prepare(&mut self, fs: f32, max_ms: f32) {
        let n = (max_ms * 0.001 * fs) as usize + 4;
        self.line = [DelayLine::new(n), DelayLine::new(n)];
        self.k = glide_k(fs, 0.05);
        self.cur = self.tgt;
    }
    fn set_ms(&mut self, ms: f32, fs: f32) {
        self.tgt = (ms * 0.001 * fs).max(1.0);
    }
    fn reset(&mut self) {
        self.line[0].reset();
        self.line[1].reset();
        self.cur = self.tgt;
    }
    #[inline]
    fn process(&mut self, l: &mut f32, r: &mut f32) {
        self.cur += (self.tgt - self.cur) * self.k;
        self.line[0].push(*l);
        self.line[1].push(*r);
        *l = self.line[0].read_lagrange3(self.cur);
        *r = self.line[1].read_lagrange3(self.cur);
    }
}

/// Peak envelope follower (instant attack, 20 ms release) used by the gate / reverse triggers.
struct Env {
    env: f32,
    rel: f32,
}

impl Env {
    fn new() -> Self {
        Self {
            env: 0.0,
            rel: 0.999,
        }
    }
    fn prepare(&mut self, fs: f32) {
        self.rel = (-1.0 / (0.02 * fs)).exp();
        self.env = 0.0;
    }
    #[inline]
    fn step(&mut self, l: f32, r: f32) -> f32 {
        self.env = l.abs().max(r.abs()).max(self.env * self.rel);
        if self.env < FLUSH {
            self.env = 0.0;
        }
        self.env
    }
}

/// High shelf on the output (HI FREQ / HI GAIN).
struct Shelf([Biquad; 2]);

impl Shelf {
    fn new() -> Self {
        Shelf([Biquad::default(), Biquad::default()])
    }
    fn set(&mut self, fs: f32, hz: f32, gain_db: f32) {
        let c = Coeffs::high_shelf(fs as f64, hz.min(0.45 * fs) as f64, 0.707, gain_db as f64);
        self.0[0].set_coeffs(c);
        self.0[1].set_coeffs(c);
    }
    fn reset(&mut self) {
        self.0[0].reset();
        self.0[1].reset();
    }
    #[inline]
    fn process(&mut self, l: &mut [f32], r: &mut [f32]) {
        self.0[0].process_block(l);
        self.0[1].process_block(r);
    }
}

/// Implements [`Model`] for a type with `raw: Raw`, `dirty: bool`, `fs: f32` and inherent `apply`,
/// `prepare_impl`, `process_impl`, `reset_impl`.
macro_rules! model_impl {
    ($ty:ty, $id:literal) => {
        impl Model for $ty {
            fn id(&self) -> &'static str {
                $id
            }
            fn prepare(&mut self, sample_rate: f64, max_block: usize) {
                self.fs = sample_rate as f32;
                self.prepare_impl(max_block);
                self.apply();
                self.reset_impl();
            }
            fn process(&mut self, l: &mut [f32], r: &mut [f32], _sc: Option<(&[f32], &[f32])>) {
                if self.dirty {
                    self.apply();
                }
                self.process_impl(l, r);
            }
            fn reset(&mut self) {
                if self.dirty {
                    self.apply();
                }
                self.reset_impl();
            }
            fn set_param(&mut self, slot: u16, v: ParamValue) {
                if self.raw.set(slot, v.as_f32()) {
                    self.dirty = true;
                }
            }
            fn param(&self, slot: u16) -> Option<f32> {
                self.raw.get(slot)
            }
            fn load_defaults(&mut self) {
                self.raw.defaults();
                self.dirty = true;
            }
            fn commit(&mut self) {
                if self.dirty {
                    self.apply();
                }
            }
        }
    };
}

// ---------------------------------------------------------------------------------------------------------
// GATED
// ---------------------------------------------------------------------------------------------------------

pub const GATED_PARAMS: [ParamSpec; 9] = [
    ps("pdel", 0.0, 200.0, 8.0),
    ps("att", 0.0, 30.0, 6.0),
    ps("dcy", 0.14, 1.0, 0.3),
    ps("dens", 1.0, 50.0, 22.0),
    ps("diff", 1.0, 30.0, 30.0),
    ps("sprd", 0.0, 100.0, 60.0),
    ps("lc", 20.0, 400.0, 80.0),
    ps("hsf", 200.0, 20_000.0, 5000.0),
    ps("hsg", -30.0, 0.0, -12.0),
];

const GATED_CHAR: Character = Character {
    line_ms: (11.0, 47.0),
    max_scale: 1.0,
    loop_ap_ms: (1.4, 5.0),
    loop_ap_gain: 0.6,
    diffuser_ms: [1.2, 2.3, 3.7, 5.9],
    diffuser_gain: 0.72,
    xover_hz: 600.0,
    hf_ratio: 0.5,
    mod_depth_ms: 0.1,
    er_window_ms: (6.0, 60.0),
    max_echo_s: 0.0,
    echo_feedback: false,
    tail_gain: 1.1,
};

/// `GATED` reverb: dense FDN burst under a trigger/hold gate.
pub struct Gated {
    raw: Raw,
    dirty: bool,
    fs: f32,
    core: ReverbCore<8>,
    pre: PreDelay,
    env: Env,
    armed: bool,
    hold: u32,
    hold_len: u32,
    gain: f32,
    att_inc: f32,
    rel_inc: f32,
    gains: Vec<f32>,
    shelf: Shelf,
}

impl Default for Gated {
    fn default() -> Self {
        Self::new()
    }
}

impl Gated {
    pub fn new() -> Self {
        Self {
            raw: Raw::new(&GATED_PARAMS),
            dirty: true,
            fs: 48_000.0,
            core: ReverbCore::new(GATED_CHAR),
            pre: PreDelay::new(),
            env: Env::new(),
            armed: true,
            hold: 0,
            hold_len: 0,
            gain: 0.0,
            att_inc: 0.01,
            rel_inc: 0.01,
            gains: Vec::new(),
            shelf: Shelf::new(),
        }
    }

    /// Core parameters the current values map to (tests).
    pub fn core_params(&self) -> CoreParams {
        let v = &self.raw.v;
        CoreParams {
            pre_ms: 0.0,
            scale: 0.9 - 0.65 * (v[3] - 1.0) / 49.0,
            decay_s: v[2] * 6.0,
            damp_hz: 12_000.0,
            lc_hz: v[6],
            hc_hz: 20_000.0,
            shape: 0.1 + 0.4 * v[1] / 30.0,
            spread: v[5] / 100.0,
            diffusion: (v[4] - 1.0) / 29.0,
            mod_rate_hz: 0.3,
            mod_depth: 0.5,
            er_level: 0.3 + 0.7 * (v[3] - 1.0) / 49.0,
            tail_level: 1.0,
            ..CoreParams::default()
        }
    }

    fn apply(&mut self) {
        self.dirty = false;
        let v = self.raw.v;
        self.core.set_params(&self.core_params());
        self.pre.set_ms(v[0], self.fs);
        self.hold_len = (v[2] * self.fs) as u32;
        self.att_inc = 1.0 / (v[1].max(1.0) * 0.001 * self.fs);
        self.rel_inc = 1.0 / (0.004 * self.fs);
        self.shelf.set(self.fs, v[7], v[8]);
    }

    fn prepare_impl(&mut self, max_block: usize) {
        self.core.prepare(self.fs as f64);
        self.pre.prepare(self.fs, 200.0);
        self.env.prepare(self.fs);
        self.gains = vec![0.0; max_block.max(1)];
    }

    fn reset_impl(&mut self) {
        self.core.reset();
        self.pre.reset();
        self.env.env = 0.0;
        self.armed = true;
        self.hold = 0;
        self.gain = 0.0;
        self.shelf.reset();
    }

    fn process_impl(&mut self, left: &mut [f32], right: &mut [f32]) {
        let n = left.len().min(right.len());
        let cap = self.gains.len();
        let mut s = 0;
        while s < n {
            let e = (s + cap).min(n);
            self.chunk(&mut left[s..e], &mut right[s..e]);
            s = e;
        }
    }

    fn chunk(&mut self, l: &mut [f32], r: &mut [f32]) {
        for i in 0..l.len() {
            self.pre.process(&mut l[i], &mut r[i]);
            let e = self.env.step(l[i], r[i]);
            if e > TRIG_ON {
                if self.armed {
                    self.armed = false;
                    self.hold = self.hold_len;
                }
            } else if e < TRIG_OFF {
                self.armed = true;
            }
            if self.hold > 0 {
                self.hold -= 1;
                self.gain = (self.gain + self.att_inc).min(1.0);
            } else {
                self.gain = (self.gain - self.rel_inc).max(0.0);
            }
            self.gains[i] = self.gain;
        }
        self.core.process(l, r);
        for i in 0..l.len() {
            l[i] *= self.gains[i];
            r[i] *= self.gains[i];
        }
        self.shelf.process(l, r);
    }
}

model_impl!(Gated, "GATED");

// ---------------------------------------------------------------------------------------------------------
// REVERSE
// ---------------------------------------------------------------------------------------------------------

pub const REVERSE_PARAMS: [ParamSpec; 8] = [
    ps("pdel", 0.0, 200.0, 30.0),
    ps("rise", 0.0, 50.0, 40.0),
    ps("dcy", 0.14, 1.0, 0.6),
    ps("diff", 1.0, 30.0, 24.0),
    ps("sprd", 0.0, 100.0, 78.0),
    ps("lc", 20.0, 400.0, 100.0),
    ps("hsf", 200.0, 20_000.0, 8000.0),
    ps("hsg", -30.0, 0.0, -12.0),
];

const REVERSE_CHAR: Character = Character {
    line_ms: (14.0, 58.0),
    max_scale: 1.0,
    loop_ap_ms: (1.6, 6.0),
    loop_ap_gain: 0.6,
    diffuser_ms: [1.3, 2.5, 4.1, 6.3],
    diffuser_gain: 0.72,
    xover_hz: 600.0,
    hf_ratio: 0.5,
    mod_depth_ms: 0.15,
    er_window_ms: (6.0, 70.0),
    max_echo_s: 0.0,
    echo_feedback: false,
    tail_gain: 1.1,
};

/// `REVERSE` reverb: FDN wet captured in windows of `dcy` seconds and played back time-reversed.
pub struct Reverse {
    raw: Raw,
    dirty: bool,
    fs: f32,
    core: ReverbCore<8>,
    pre: PreDelay,
    env: Env,
    trig: Vec<f32>,
    /// `[buffer][channel]`
    buf: [[Vec<f32>; 2]; 2],
    cap_buf: usize,
    cap_on: bool,
    cap_pos: usize,
    cap_len: usize,
    play_on: bool,
    play_pos: usize,
    play_len: usize,
    w_len: usize,
    fade_in: f32,
    fade_out: usize,
    shelf: Shelf,
}

impl Default for Reverse {
    fn default() -> Self {
        Self::new()
    }
}

impl Reverse {
    pub fn new() -> Self {
        Self {
            raw: Raw::new(&REVERSE_PARAMS),
            dirty: true,
            fs: 48_000.0,
            core: ReverbCore::new(REVERSE_CHAR),
            pre: PreDelay::new(),
            env: Env::new(),
            trig: Vec::new(),
            buf: [[Vec::new(), Vec::new()], [Vec::new(), Vec::new()]],
            cap_buf: 0,
            cap_on: false,
            cap_pos: 0,
            cap_len: 0,
            play_on: false,
            play_pos: 0,
            play_len: 0,
            w_len: 1,
            fade_in: 0.1,
            fade_out: 1,
            shelf: Shelf::new(),
        }
    }

    /// Samples between a trigger (at the pre-delayed input) and the start of the reversed swell, excluding the
    /// pre-delay: the window length `W`.
    pub fn wet_latency_samples(&self) -> usize {
        self.w_len
    }

    pub fn core_params(&self) -> CoreParams {
        let v = &self.raw.v;
        CoreParams {
            pre_ms: 0.0,
            scale: 0.7,
            decay_s: v[2],
            damp_hz: 10_000.0,
            lc_hz: v[5],
            hc_hz: 20_000.0,
            shape: 0.2,
            spread: v[4] / 100.0,
            diffusion: (v[3] - 1.0) / 29.0,
            mod_rate_hz: 0.3,
            mod_depth: 0.5,
            er_level: 0.5,
            tail_level: 1.0,
            ..CoreParams::default()
        }
    }

    fn apply(&mut self) {
        self.dirty = false;
        let v = self.raw.v;
        self.core.set_params(&self.core_params());
        self.pre.set_ms(v[0], self.fs);
        let cap = self.buf[0][0].len().max(1);
        self.w_len = ((v[2] * self.fs) as usize).clamp(1, cap);
        self.fade_in = 0.05 + 0.75 * v[1] / 50.0;
        self.fade_out = ((0.003 * self.fs) as usize).max(1);
        self.shelf.set(self.fs, v[6], v[7]);
    }

    fn prepare_impl(&mut self, max_block: usize) {
        self.core.prepare(self.fs as f64);
        self.pre.prepare(self.fs, 200.0);
        self.env.prepare(self.fs);
        self.trig = vec![0.0; max_block.max(1)];
        let n = (1.0 * self.fs) as usize + 8;
        for b in &mut self.buf {
            for c in b.iter_mut() {
                *c = vec![0.0; n];
            }
        }
    }

    fn reset_impl(&mut self) {
        self.core.reset();
        self.pre.reset();
        self.env.env = 0.0;
        for b in &mut self.buf {
            for c in b.iter_mut() {
                c.iter_mut().for_each(|x| *x = 0.0);
            }
        }
        self.cap_on = false;
        self.play_on = false;
        self.cap_buf = 0;
        self.shelf.reset();
    }

    fn process_impl(&mut self, left: &mut [f32], right: &mut [f32]) {
        let n = left.len().min(right.len());
        let cap = self.trig.len();
        let mut s = 0;
        while s < n {
            let e = (s + cap).min(n);
            self.chunk(&mut left[s..e], &mut right[s..e]);
            s = e;
        }
    }

    #[inline]
    fn play_env(&self) -> f32 {
        let x = self.play_pos as f32 / self.play_len as f32;
        let fi = if x < self.fade_in {
            let t = x / self.fade_in;
            t * t * (3.0 - 2.0 * t)
        } else {
            1.0
        };
        let rem = self.play_len - self.play_pos;
        let fo = if rem < self.fade_out {
            rem as f32 / self.fade_out as f32
        } else {
            1.0
        };
        fi * fo
    }

    fn chunk(&mut self, l: &mut [f32], r: &mut [f32]) {
        for i in 0..l.len() {
            self.pre.process(&mut l[i], &mut r[i]);
            self.trig[i] = f32::from(self.env.step(l[i], r[i]) > TRIG_ON);
        }
        self.core.process(l, r);
        for i in 0..l.len() {
            let (wl, wr) = (l[i], r[i]);
            if !self.cap_on && self.trig[i] > 0.0 {
                self.cap_on = true;
                self.cap_pos = 0;
                self.cap_len = self.w_len;
            }
            let (mut ol, mut or) = (0.0, 0.0);
            if self.play_on {
                let pb = self.cap_buf ^ 1;
                let src = self.play_len - 1 - self.play_pos;
                let g = self.play_env();
                ol = self.buf[pb][0][src] * g;
                or = self.buf[pb][1][src] * g;
                self.play_pos += 1;
                if self.play_pos >= self.play_len {
                    self.play_on = false;
                }
            }
            if self.cap_on {
                self.buf[self.cap_buf][0][self.cap_pos] = wl;
                self.buf[self.cap_buf][1][self.cap_pos] = wr;
                self.cap_pos += 1;
                if self.cap_pos >= self.cap_len {
                    self.cap_on = false;
                    self.cap_buf ^= 1; // the captured buffer is now the playback buffer
                    self.play_on = true;
                    self.play_pos = 0;
                    self.play_len = self.cap_len;
                }
            }
            l[i] = ol;
            r[i] = or;
        }
        self.shelf.process(l, r);
    }
}

model_impl!(Reverse, "REVERSE");

// ---------------------------------------------------------------------------------------------------------
// DEL/REV
// ---------------------------------------------------------------------------------------------------------

pub const DELREV_PARAMS: [ParamSpec; 11] = [
    ps("time", 1.0, 3000.0, 240.0),
    ps("feed", 0.0, 100.0, 40.0),
    ps("fhc", 200.0, 20_000.0, 7600.0),
    ps("dly", 0.0, 100.0, 40.0),
    ps("d2r", 0.0, 100.0, 70.0),
    ps("pdel", 0.0, 200.0, 24.0),
    ps("size", 2.0, 100.0, 52.0),
    ps("dcy", 0.1, 5.0, 1.55),
    ps("damp", 1000.0, 20_000.0, 5600.0),
    ps("rlc", 20.0, 400.0, 90.0),
    ps("i2r", 0.0, 100.0, 0.0),
];

const DELREV_CHAR: Character = Character {
    line_ms: (24.0, 86.0),
    max_scale: 1.2,
    loop_ap_ms: (2.0, 7.0),
    loop_ap_gain: 0.6,
    diffuser_ms: [1.4, 2.6, 4.3, 6.7],
    diffuser_gain: 0.72,
    xover_hz: 600.0,
    hf_ratio: 0.3,
    mod_depth_ms: 0.2,
    er_window_ms: (8.0, 80.0),
    max_echo_s: 0.0,
    echo_feedback: false,
    tail_gain: 1.1,
};

/// `DEL/REV`: stereo delay in parallel with a reverb, cross-sent per `d2r` / `i2r`.
pub struct DelRev {
    raw: Raw,
    dirty: bool,
    fs: f32,
    core: ReverbCore<8>,
    line: [DelayLine; 2],
    d_cur: f32,
    d_tgt: f32,
    d_k: f32,
    fb: f32,
    lp_a: f32,
    lp: [f32; 2],
    dly: f32,
    d2r: f32,
    i2r: f32,
    rin: [Vec<f32>; 2],
}

impl Default for DelRev {
    fn default() -> Self {
        Self::new()
    }
}

impl DelRev {
    pub fn new() -> Self {
        Self {
            raw: Raw::new(&DELREV_PARAMS),
            dirty: true,
            fs: 48_000.0,
            core: ReverbCore::new(DELREV_CHAR),
            line: [DelayLine::new(1), DelayLine::new(1)],
            d_cur: 1.0,
            d_tgt: 1.0,
            d_k: 0.01,
            fb: 0.0,
            lp_a: 0.5,
            lp: [0.0; 2],
            dly: 0.0,
            d2r: 0.0,
            i2r: 0.0,
            rin: [Vec::new(), Vec::new()],
        }
    }

    pub fn core_params(&self) -> CoreParams {
        let v = &self.raw.v;
        CoreParams {
            pre_ms: v[5],
            scale: 0.25 + 0.95 * (v[6] - 2.0) / 98.0,
            decay_s: v[7],
            damp_hz: v[8],
            lc_hz: v[9],
            hc_hz: 20_000.0,
            shape: 0.3,
            spread: 0.5,
            diffusion: 0.8,
            mod_rate_hz: 0.4,
            mod_depth: 1.0,
            er_level: 0.3,
            tail_level: 1.0,
            ..CoreParams::default()
        }
    }

    fn apply(&mut self) {
        self.dirty = false;
        let v = self.raw.v;
        self.core.set_params(&self.core_params());
        self.d_tgt = (v[0] * 0.001 * self.fs).max(1.0);
        self.fb = (v[1] * 0.01).min(0.995);
        self.lp_a = 1.0 - (-std::f32::consts::TAU * v[2].min(0.45 * self.fs) / self.fs).exp();
        self.dly = v[3] * 0.01;
        self.d2r = v[4] * 0.01;
        self.i2r = v[10] * 0.01;
    }

    fn prepare_impl(&mut self, max_block: usize) {
        self.core.prepare(self.fs as f64);
        let n = (3.0 * self.fs) as usize + 8;
        self.line = [DelayLine::new(n), DelayLine::new(n)];
        self.d_k = glide_k(self.fs, 0.08);
        self.rin = [vec![0.0; max_block.max(1)], vec![0.0; max_block.max(1)]];
    }

    fn reset_impl(&mut self) {
        self.core.reset();
        self.line[0].reset();
        self.line[1].reset();
        self.lp = [0.0; 2];
        self.d_cur = self.d_tgt;
    }

    fn process_impl(&mut self, left: &mut [f32], right: &mut [f32]) {
        let n = left.len().min(right.len());
        let cap = self.rin[0].len();
        let mut s = 0;
        while s < n {
            let e = (s + cap).min(n);
            self.chunk(&mut left[s..e], &mut right[s..e]);
            s = e;
        }
    }

    fn chunk(&mut self, l: &mut [f32], r: &mut [f32]) {
        let m = l.len();
        for i in 0..m {
            self.d_cur += (self.d_tgt - self.d_cur) * self.d_k;
            let x = [l[i], r[i]];
            let mut y = [0.0f32; 2];
            for c in 0..2 {
                let d = self.line[c].read_lagrange3(self.d_cur);
                self.lp[c] += self.lp_a * (d - self.lp[c]);
                if self.lp[c].abs() < FLUSH {
                    self.lp[c] = 0.0;
                }
                let w = x[c] + self.fb * self.lp[c];
                self.line[c].push(if w.abs() < FLUSH { 0.0 } else { w });
                y[c] = d;
            }
            self.rin[0][i] = self.i2r * x[0] + self.d2r * y[0];
            self.rin[1][i] = self.i2r * x[1] + self.d2r * y[1];
            l[i] = self.dly * y[0];
            r[i] = self.dly * y[1];
        }
        let [a, b] = &mut self.rin;
        self.core.process(&mut a[..m], &mut b[..m]);
        for i in 0..m {
            l[i] += a[i];
            r[i] += b[i];
        }
    }
}

model_impl!(DelRev, "DEL/REV");

// ---------------------------------------------------------------------------------------------------------
// SHIMMER
// ---------------------------------------------------------------------------------------------------------

pub const SHIMMER_PARAMS: [ParamSpec; 8] = [
    ps("pdel", 0.0, 250.0, 100.0),
    ps("size", 1.0, 50.0, 25.0),
    ps("dcy", 1.0, 20.0, 3.0),
    ps("lc", 25.0, 250.0, 80.0),
    ps("hc", 500.0, 7000.0, 5000.0),
    ps("damp", 0.0, 100.0, 50.0),
    ps("shim", 0.0, 100.0, 50.0),
    ps("shine", 0.0, 100.0, 50.0),
];

const SHIMMER_CHAR: Character = Character {
    line_ms: (30.0, 100.0),
    max_scale: 1.5,
    loop_ap_ms: (2.1, 7.9),
    loop_ap_gain: 0.6,
    diffuser_ms: [1.43, 2.61, 4.37, 6.83],
    diffuser_gain: 0.72,
    xover_hz: 500.0,
    hf_ratio: 0.3,
    mod_depth_ms: 0.3,
    er_window_ms: (8.0, 120.0),
    max_echo_s: 0.0,
    echo_feedback: false,
    tail_gain: 1.1,
};

/// Pitch ratios of the two grain shifters: +12 and +7 semitones.
const RATIO_12: f32 = 2.0;
const RATIO_7: f32 = 1.498_307_1;

/// Two overlap-add grain pitch shifters (+12 / +7) sharing one history ring per channel. Reads trail the write
/// head by at least `min_d` samples so they can run a sub-block ahead of the writes (see module doc).
struct Grain {
    line: [DelayLine; 2],
    w: f32,
    min_d: f32,
    ph: [f32; 2],
    inc: [f32; 2],
}

impl Grain {
    fn new() -> Self {
        Self {
            line: [DelayLine::new(1), DelayLine::new(1)],
            w: 4096.0,
            min_d: 64.0,
            ph: [0.0, 0.37],
            inc: [0.0; 2],
        }
    }
    fn prepare(&mut self, fs: f32) {
        self.w = (4096.0 * fs / 48_000.0).round().max(512.0);
        self.min_d = (2 * SB) as f32;
        let n = (self.min_d + self.w) as usize + 8;
        self.line = [DelayLine::new(n), DelayLine::new(n)];
        self.inc = [(RATIO_12 - 1.0) / self.w, (RATIO_7 - 1.0) / self.w];
        self.reset();
    }
    fn reset(&mut self) {
        self.line[0].reset();
        self.line[1].reset();
        self.ph = [0.0, 0.37];
    }
    /// Shifted pair of the mix `w12 x (+12) + w7 x (+7)` for the sample `off` samples after the write head's
    /// last pushed sample.
    #[inline]
    fn read(&mut self, off: usize, w12: f32, w7: f32) -> (f32, f32) {
        let mut out = [0.0f32; 2];
        for s in 0..2 {
            let wt = if s == 0 { w12 } else { w7 };
            let p1 = self.ph[s];
            let p2 = if p1 >= 0.5 { p1 - 0.5 } else { p1 + 0.5 };
            if wt > 0.0 {
                let a = 0.5 - 0.5 * (std::f32::consts::TAU * p1).cos();
                let b = 1.0 - a;
                let d1 = self.min_d + (1.0 - p1) * self.w - off as f32 - 1.0;
                let d2 = self.min_d + (1.0 - p2) * self.w - off as f32 - 1.0;
                for c in 0..2 {
                    out[c] += wt
                        * (a * self.line[c].read_lagrange3(d1)
                            + b * self.line[c].read_lagrange3(d2));
                }
            }
            let mut p = p1 + self.inc[s];
            if p >= 1.0 {
                p -= 1.0;
            }
            self.ph[s] = p;
        }
        (out[0], out[1])
    }
    #[inline]
    fn push(&mut self, l: f32, r: f32) {
        self.line[0].push(l);
        self.line[1].push(r);
    }
}

/// `SHIMMER` reverb: FDN with a +12 / +7 semitone grain shifter in the feedback loop.
pub struct Shimmer {
    raw: Raw,
    dirty: bool,
    fs: f32,
    core: ReverbCore<8>,
    pre: PreDelay,
    grain: Grain,
    fb_gain: f32,
    w12: f32,
    w7: f32,
    lp_a: f32,
    hp_a: f32,
    lp: [f32; 2],
    hp: [f32; 2],
}

impl Default for Shimmer {
    fn default() -> Self {
        Self::new()
    }
}

impl Shimmer {
    pub fn new() -> Self {
        Self {
            raw: Raw::new(&SHIMMER_PARAMS),
            dirty: true,
            fs: 48_000.0,
            core: ReverbCore::new(SHIMMER_CHAR),
            pre: PreDelay::new(),
            grain: Grain::new(),
            fb_gain: 0.0,
            w12: 1.0,
            w7: 0.0,
            lp_a: 0.3,
            hp_a: 0.01,
            lp: [0.0; 2],
            hp: [0.0; 2],
        }
    }

    pub fn core_params(&self) -> CoreParams {
        let v = &self.raw.v;
        CoreParams {
            pre_ms: 0.0,
            scale: 0.3 + 1.2 * (v[1] - 1.0) / 49.0,
            decay_s: v[2],
            damp_hz: 16_000.0 * 0.1f32.powf(v[5] / 100.0),
            lc_hz: v[3],
            hc_hz: v[4],
            shape: 0.3,
            spread: 0.5,
            diffusion: 0.9,
            mod_rate_hz: 0.5,
            mod_depth: 1.0,
            er_level: 0.2,
            tail_level: 1.0,
            ..CoreParams::default()
        }
    }

    fn apply(&mut self) {
        self.dirty = false;
        let v = self.raw.v;
        self.core.set_params(&self.core_params());
        self.pre.set_ms(v[0], self.fs);
        self.fb_gain = 0.5 * v[6] * 0.01;
        self.w7 = 0.5 * v[7] * 0.01;
        self.w12 = 1.0 - self.w7;
        let lp_hz = 4000.0 + 8000.0 * v[7] * 0.01;
        self.lp_a = 1.0 - (-std::f32::consts::TAU * lp_hz / self.fs).exp();
        self.hp_a = 1.0 - (-std::f32::consts::TAU * 120.0 / self.fs).exp();
    }

    fn prepare_impl(&mut self, _max_block: usize) {
        self.core.prepare(self.fs as f64);
        self.pre.prepare(self.fs, 250.0);
        self.grain.prepare(self.fs);
    }

    fn reset_impl(&mut self) {
        self.core.reset();
        self.pre.reset();
        self.grain.reset();
        self.lp = [0.0; 2];
        self.hp = [0.0; 2];
    }

    fn process_impl(&mut self, left: &mut [f32], right: &mut [f32]) {
        let n = left.len().min(right.len());
        for i in 0..n {
            self.pre.process(&mut left[i], &mut right[i]);
        }
        let mut s = 0;
        while s < n {
            let m = SB.min(n - s);
            for i in 0..m {
                let (a, b) = self.grain.read(i, self.w12, self.w7);
                let mut fb = [a, b];
                for c in 0..2 {
                    let x = fb[c];
                    self.hp[c] += self.hp_a * (x - self.hp[c]);
                    let h = x - self.hp[c];
                    self.lp[c] += self.lp_a * (h - self.lp[c]);
                    if self.lp[c].abs() < FLUSH {
                        self.lp[c] = 0.0;
                    }
                    fb[c] = self.fb_gain * self.lp[c].tanh();
                }
                left[s + i] += fb[0];
                right[s + i] += fb[1];
            }
            self.core.process(&mut left[s..s + m], &mut right[s..s + m]);
            for i in 0..m {
                self.grain.push(left[s + i], right[s + i]);
            }
            s += m;
        }
    }
}

model_impl!(Shimmer, "SHIMMER");

/// Factory for the four models (`None` for other ids).
pub fn create(id: &str) -> Option<Box<dyn Model>> {
    Some(match id {
        "GATED" => Box::new(Gated::new()),
        "REVERSE" => Box::new(Reverse::new()),
        "DEL/REV" => Box::new(DelRev::new()),
        "SHIMMER" => Box::new(Shimmer::new()),
        _ => return None,
    })
}
