//! `SPRING` (SPRING REVERB, `fx_pardef_spring_reverb`, T05P): Fender / Accutronics-style spring tank
//! (models.md §6).
//!
//! A helical spring is strongly dispersive: below its transition frequency (~4-5 kHz for guitar-amp tanks)
//! the group delay rises with frequency, so every transit of the spring turns a click into a rising chirp,
//! and the reflections at the spring ends repeat that chirp every round trip (the "drip"). Model after
//! V. Välimäki, J. Parker, J. S. Abel, "Parametric spring reverberation effect", JAES 58(7/8), 2010, and
//! J. Parker, "Efficient dispersion generation structures for spring reverb emulation", EURASIP JASP 2011
//! (algorithms only, own implementation):
//!
//! ```text
//!  per spring (lane i, 4 springs per side, SIMD across the 8 lanes):
//!   in --(+)--> [ STAGES x stretched allpass A_i(z^K) ] --+--> chirp out x weight_i
//!        ^                                                |
//!        +-- -1 x r_i^D x HighShelf_i <-- delay D_i(t) <--+     (round trip, slightly wobbling)
//!  out L/R = sum of the side's springs -> 4th-order LP at the chirp band edge -> BASS / TREBLE shelves
//! ```
//!
//! - **Dispersion / chirp**: `A(z^K) = (a + z^-K) / (1 + a z^-K)` with `a > 0` has group delay
//!   `K (1 - a^2) / (1 + 2 a cos(K w) + a^2)`, rising from `K (1-a)/(1+a)` at DC to `K (1+a)/(1-a)` at
//!   `fs / 2K`. `K = round(fs / 9600)` puts that band edge at ~4.8 kHz (the spring's transition frequency);
//!   the stretched images above it are removed by the output low-pass. [`STAGES`] = 80 sections give a chirp
//!   whose top end lags the bass by tens of ms, as measured on real tanks.
//! - **Tension / length**: per spring the allpass coefficient `a` (0.62..0.71, "tension": larger = steeper
//!   chirp) and the round-trip delay (27.8..44.9 ms, "length") differ, so the springs' drips never align.
//!   A 0.04 ms, 0.4..0.7 Hz wobble of each round trip mimics the springs' slight sway.
//! - **Decay**: the loss is applied as `z^-1 -> r z^-1` on every delay element of the loop (each stretched
//!   allpass state scaled by `r^K`, the round trip by `r^D`), which multiplies the lossless impulse response by
//!   `r^n`: the T60 is exact at every frequency regardless of the dispersion (a single loop gain would give
//!   a T60 that varies with the frequency-dependent loop time). `r` per spring = `10^(-3 / (fs dcy))`
//!   corrected for the in-loop damping shelf's loss at 1 kHz per round trip. The loop (lossless allpasses,
//!   `|shelf| <= 1`, `r < 1`) is stable for every setting.
//! - **Damping**: an in-loop high shelf (4 kHz, -0.2..-3 dB per round trip from TREBLE) shortens the HF decay.
//!
//! | slot | key | range | mapping [I] |
//! |---|---|---|---|
//! | 1 | `dcy` | 1.5..6 s | T60 (exact, see above) |
//! | 2 | `dens` | 1..30 | number of springs per side: 1 at `dens` 1, fading in the 2nd, 3rd and 4th spring over thirds of the range (power-normalised) |
//! | 3 | `low` | 1..50 | BASS: first-order low shelf at 250 Hz, `12 (low - 25.5) / 24.5` dB (-12..+12; default 15 = -5 dB, tanks are bass-light) |
//! | 4 | `high` | 1..50 | TREBLE: first-order high shelf at 2.5 kHz, `12 (high - 25.5) / 24.5` dB, and in-loop HF damping `-(0.2 + 2.8 (1 - (high-1)/49))` dB per round trip |
//!
//! RT-safety: buffers are allocated in [`Model::prepare`]; `process`, `set_param`, `reset` never allocate.
//! Denormals: recursive writes are flushed below 1e-18, filter states once per block.

// Lane loops index several parallel `[f32; LANES]` arrays by spring number.
#![allow(clippy::needless_range_loop)]

use super::core::{flush, Glide, LaneBiquads, GLIDE_S};
use super::{ps, ParamSpec};
use crate::model::{Model, ParamValue};
use wing_dsp::biquad::{butterworth, Biquad, Coeffs, CutKind};

/// Springs (4 per side).
pub const LANES: usize = 8;
/// Stretched allpass sections per spring.
pub const STAGES: usize = 80;
/// Largest allpass stretch (192 kHz).
const K_MAX: usize = 24;
/// Frequency at which the decay correction for the in-loop damping is made (Hz).
const F_REF: f64 = 1000.0;
/// Round-trip delay per spring (ms); lanes 0..3 = left springs, 4..7 = right.
pub const LOOP_MS: [f32; LANES] = [33.1, 38.7, 29.3, 44.9, 35.6, 31.2, 41.4, 27.8];
/// Allpass coefficient per spring ("tension").
pub const TENSION: [f32; LANES] = [0.68, 0.64, 0.71, 0.62, 0.66, 0.70, 0.63, 0.69];
const IN_SIGN: [f32; LANES] = [1.0, -1.0, 1.0, -1.0, -1.0, 1.0, -1.0, 1.0];
const OUT_SIGN: [f32; LANES] = [1.0, 1.0, -1.0, -1.0, 1.0, -1.0, 1.0, -1.0];
const WOBBLE_HZ: [f32; LANES] = [0.37, 0.53, 0.71, 0.43, 0.47, 0.61, 0.39, 0.67];
const WOBBLE_MS: f32 = 0.04;
const IN_GAIN: f32 = 0.5;
const OUT_GAIN: f32 = 0.5;
const DAMP_HZ: f64 = 4000.0;

pub const PARAMS: [ParamSpec; 4] = [
    ps("dcy", 1.5, 6.0, 3.0),
    ps("dens", 1.0, 30.0, 20.0),
    ps("low", 1.0, 50.0, 15.0),
    ps("high", 1.0, 50.0, 35.0),
];

/// Group delay (samples) of `STAGES` stretched first-order allpasses `A(z^k)` with coefficient `a` at `w`
/// (rad/sample).
pub fn chain_group_delay(a: f32, k: usize, w: f32) -> f32 {
    let (a, kf) = (a as f64, k as f64);
    let d = 1.0 + 2.0 * a * (kf * w as f64).cos() + a * a;
    (STAGES as f64 * kf * (1.0 - a * a) / d) as f32
}

/// Spring weights for density `d` 0..1 (power-normalised, rank 0 always on).
fn weights(d: f32) -> [f32; 4] {
    let t = 3.0 * d.clamp(0.0, 1.0);
    let w = [
        1.0,
        t.clamp(0.0, 1.0),
        (t - 1.0).clamp(0.0, 1.0),
        (t - 2.0).clamp(0.0, 1.0),
    ];
    let n = 1.0 / w.iter().map(|x| x * x).sum::<f32>().sqrt();
    w.map(|x| x * n)
}

fn shelf_db(v: f32) -> f32 {
    12.0 * (v - 25.5) / 24.5
}

fn damp_db(high: f32) -> f32 {
    -(0.2 + 2.8 * (1.0 - (high - 1.0) / 49.0))
}

pub struct Spring {
    raw: [f32; 4],
    dirty: bool,
    fs: f32,
    fresh: bool,
    // ---- dispersion chains
    k: usize,
    ki: usize,
    chain: Vec<[f32; LANES]>,
    a: [f32; LANES],
    /// `r^K` per spring.
    rk: [f32; LANES],
    // ---- round trip
    line: Vec<[f32; LANES]>,
    mask: usize,
    w: usize,
    /// Nominal round-trip read delay (samples; the loop adds 1).
    d: [f32; LANES],
    /// `r^(d+1)` per spring.
    rl: [f32; LANES],
    fb: [f32; LANES],
    damp: LaneBiquads<LANES>,
    lfo_c: [f32; LANES],
    lfo_s: [f32; LANES],
    rot_c: [f32; LANES],
    rot_s: [f32; LANES],
    // ---- parameters
    decay: Glide,
    dens: Glide,
    low: Glide,
    high: Glide,
    loop_key: [f32; 2],
    wt: [f32; LANES],
    /// BASS / TREBLE shelf gains (linear) at the end of the last block.
    tone_g: [f32; 2],
    // ---- output
    lp: [[Biquad; 2]; 2],
    /// One-pole 250 Hz low-pass / 2.5 kHz high-pass: shelf = x + (g - 1) x filtered, `g` ramped per sample.
    tone_lp: [Biquad; 2],
    tone_hp: [Biquad; 2],
}

impl Default for Spring {
    fn default() -> Self {
        Self::new()
    }
}

impl Spring {
    pub fn new() -> Self {
        let mut s = Self {
            raw: [0.0; 4],
            dirty: true,
            fs: 48_000.0,
            fresh: true,
            k: 1,
            ki: 0,
            chain: vec![[0.0; LANES]; STAGES],
            a: TENSION,
            rk: [0.0; LANES],
            line: vec![[0.0; LANES]; 1],
            mask: 0,
            w: 0,
            d: [1.0; LANES],
            rl: [0.0; LANES],
            fb: [0.0; LANES],
            damp: LaneBiquads::default(),
            lfo_c: [1.0; LANES],
            lfo_s: [0.0; LANES],
            rot_c: [1.0; LANES],
            rot_s: [0.0; LANES],
            decay: Glide::default(),
            dens: Glide::default(),
            low: Glide::default(),
            high: Glide::default(),
            loop_key: [f32::NAN; 2],
            wt: [0.0; LANES],
            tone_g: [1.0; 2],
            lp: [[Biquad::default(); 2]; 2],
            tone_lp: [Biquad::default(); 2],
            tone_hp: [Biquad::default(); 2],
        };
        s.load_defaults();
        s
    }

    /// Allpass stretch factor `K` at the prepared rate.
    pub fn stretch(&self) -> usize {
        self.k
    }

    /// Per-sample decay factor `r` of spring `i` (tests).
    pub fn decay_factor(&self, i: usize) -> f32 {
        self.rk[i].powf(1.0 / self.k as f32)
    }

    /// Sum of |state| and whether any value is subnormal (tests).
    pub fn state_report(&self) -> (f64, bool) {
        let mut sum = 0.0f64;
        let mut sub = false;
        for &x in self.chain.iter().chain(self.line.iter()).flatten() {
            sum += x.abs() as f64;
            sub |= x != 0.0 && x.abs() < f32::MIN_POSITIVE;
        }
        (sum, sub)
    }

    fn update_targets(&mut self) {
        self.dirty = false;
        self.decay.tgt = self.raw[0];
        self.dens.tgt = (self.raw[1] - 1.0) / 29.0;
        self.low.tgt = self.raw[2];
        self.high.tgt = self.raw[3];
    }

    fn snap(&mut self) {
        for g in [
            &mut self.decay,
            &mut self.dens,
            &mut self.low,
            &mut self.high,
        ] {
            g.snap();
        }
    }

    fn target_weights(&self) -> [f32; LANES] {
        let w = weights(self.dens.cur);
        core::array::from_fn(|i| w[i % 4] * OUT_SIGN[i])
    }

    /// Recompute loop and tone coefficients from the current glide values (only what changed).
    fn update_coeffs(&mut self) {
        let fs = self.fs as f64;
        let key = [self.decay.cur, self.high.cur];
        if key != self.loop_key {
            self.loop_key = key;
            let rho = 10f64.powf(-3.0 / (fs * key[0] as f64));
            let c =
                Coeffs::high_shelf_slope(fs, DAMP_HZ.min(0.45 * fs), 1.0, damp_db(key[1]) as f64);
            let loss_db = c.magnitude_db(fs, F_REF).min(0.0);
            let w_ref = (core::f64::consts::TAU * F_REF / fs) as f32;
            for i in 0..LANES {
                self.damp.set(i, c);
                let lt =
                    (self.d[i] + 1.0) as f64 + chain_group_delay(self.a[i], self.k, w_ref) as f64;
                // per-sample factor so that r^lt x shelf loss = rho^lt
                let r = (rho * 10f64.powf(-loss_db / (20.0 * lt))).min(0.999_999);
                self.rk[i] = r.powi(self.k as i32) as f32;
                self.rl[i] = r.powf((self.d[i] + 1.0) as f64) as f32;
            }
        }
    }

    fn target_tone(&self) -> [f32; 2] {
        [self.low.cur, self.high.cur].map(|v| 10f32.powf(shelf_db(v) / 20.0) - 1.0)
    }
}

impl Model for Spring {
    fn id(&self) -> &'static str {
        "SPRING"
    }

    fn prepare(&mut self, sample_rate: f64, _max_block: usize) {
        self.fs = sample_rate as f32;
        let ms = self.fs * 0.001;
        self.k = ((sample_rate / 9600.0).round() as usize).clamp(1, K_MAX);
        self.chain = vec![[0.0; LANES]; STAGES * self.k];
        let max_d = LOOP_MS.iter().fold(0f32, |a, &b| a.max(b)) * ms + WOBBLE_MS * ms + 8.0;
        let n = (max_d as usize + 2).next_power_of_two();
        self.line = vec![[0.0; LANES]; n];
        self.mask = n - 1;
        for i in 0..LANES {
            self.d[i] = LOOP_MS[i] * ms - 1.0;
            let w = core::f32::consts::TAU * WOBBLE_HZ[i] / self.fs;
            self.rot_c[i] = w.cos();
            self.rot_s[i] = w.sin();
        }
        let lp = butterworth(
            CutKind::Lowpass,
            sample_rate,
            0.92 * sample_rate / (2.0 * self.k as f64),
            4,
        );
        for c in 0..2 {
            for (b, co) in self.lp[c].iter_mut().zip(lp.as_slice()) {
                *b = Biquad::new(*co);
            }
        }
        for c in 0..2 {
            self.tone_lp[c] = Biquad::new(Coeffs::lowpass1(sample_rate, 250.0));
            self.tone_hp[c] = Biquad::new(Coeffs::highpass1(sample_rate, 2500.0));
        }
        self.loop_key = [f32::NAN; 2];
        self.reset();
    }

    fn process(&mut self, left: &mut [f32], right: &mut [f32], _sc: Option<(&[f32], &[f32])>) {
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
        for g in [
            &mut self.decay,
            &mut self.dens,
            &mut self.low,
            &mut self.high,
        ] {
            g.step(a);
        }
        let wt0 = self.wt;
        self.update_coeffs();
        let wt1 = self.target_weights();
        self.wt = wt1;
        let inv = 1.0 / len as f32;
        let mut wt = wt0;
        let (tg0, tg1) = (self.tone_g, self.target_tone());
        self.tone_g = tg1;
        let mut tg = tg0;
        let tg_i = [(tg1[0] - tg0[0]) * inv, (tg1[1] - tg0[1]) * inv];
        let wt_i: [f32; LANES] = core::array::from_fn(|i| (wt1[i] - wt0[i]) * inv);
        let k = self.k;
        let mask = self.mask;
        let depth = WOBBLE_MS * self.fs * 0.001;
        let (ap, rk, rl) = (self.a, self.rk, self.rl);

        for n in 0..len {
            let x = [left[n] * IN_GAIN, right[n] * IN_GAIN];
            let mut u = [0.0f32; LANES];
            for i in 0..LANES {
                u[i] = x[i / 4] * IN_SIGN[i] + self.fb[i];
            }
            // dispersion: STAGES stretched allpasses, state v[n-K] scaled by r^K
            let ki = self.ki;
            for j in 0..STAGES {
                let idx = j * k + ki;
                let s = self.chain[idx];
                let mut nv = [0.0f32; LANES];
                for i in 0..LANES {
                    let sd = rk[i] * s[i];
                    let v = u[i] - ap[i] * sd;
                    nv[i] = flush(v);
                    u[i] = ap[i] * v + sd;
                }
                self.chain[idx] = nv;
            }
            self.ki = if ki + 1 == k { 0 } else { ki + 1 };
            // round trip
            let wn = (self.w + 1) & mask;
            self.w = wn;
            let mut wv = [0.0f32; LANES];
            for i in 0..LANES {
                wv[i] = flush(u[i]);
            }
            self.line[wn] = wv;
            let mut fb = [0.0f32; LANES];
            for i in 0..LANES {
                let d = self.d[i] + depth * self.lfo_s[i];
                // 3rd-order Lagrange through offsets m-1..m+2 (linear interpolation's HF loss per round
                // trip would shorten the T60 by ~5 % at 6 s)
                let m = d as usize;
                let f = d - m as f32;
                let ym1 = self.line[wn.wrapping_sub(m - 1) & mask][i];
                let y0 = self.line[wn.wrapping_sub(m) & mask][i];
                let y1 = self.line[wn.wrapping_sub(m + 1) & mask][i];
                let y2 = self.line[wn.wrapping_sub(m + 2) & mask][i];
                let (fm1, f1, f2) = (f + 1.0, f - 1.0, f - 2.0);
                let y = -ym1 * f * f1 * f2 * (1.0 / 6.0) + y0 * fm1 * f1 * f2 * 0.5
                    - y1 * fm1 * f * f2 * 0.5
                    + y2 * fm1 * f * f1 * (1.0 / 6.0);
                fb[i] = -rl[i] * y;
            }
            self.damp.tick(&mut fb);
            for i in 0..LANES {
                self.fb[i] = flush(fb[i]);
                let (c, s) = (self.lfo_c[i], self.lfo_s[i]);
                self.lfo_c[i] = c * self.rot_c[i] - s * self.rot_s[i];
                self.lfo_s[i] = s * self.rot_c[i] + c * self.rot_s[i];
            }
            // output
            let mut o = [0.0f32; 2];
            for i in 0..LANES {
                wt[i] += wt_i[i];
                o[i / 4] += wt[i] * u[i];
            }
            tg[0] += tg_i[0];
            tg[1] += tg_i[1];
            for (c, y) in o.iter_mut().enumerate() {
                let mut v = *y;
                for b in self.lp[c].iter_mut() {
                    v = b.tick(v);
                }
                v += tg[0] * self.tone_lp[c].tick(v);
                v += tg[1] * self.tone_hp[c].tick(v);
                *y = v * OUT_GAIN;
            }
            left[n] = o[0];
            right[n] = o[1];
        }

        for i in 0..LANES {
            let (c, s) = (self.lfo_c[i], self.lfo_s[i]);
            let g = 1.5 - 0.5 * (c * c + s * s);
            self.lfo_c[i] = c * g;
            self.lfo_s[i] = s * g;
        }
        self.damp.flush();
        for c in 0..2 {
            for b in self.lp[c].iter_mut() {
                b.flush_denormals();
            }
            self.tone_lp[c].flush_denormals();
            self.tone_hp[c].flush_denormals();
        }
    }

    fn reset(&mut self) {
        self.chain.iter_mut().for_each(|f| *f = [0.0; LANES]);
        self.line.iter_mut().for_each(|f| *f = [0.0; LANES]);
        self.fb = [0.0; LANES];
        self.ki = 0;
        self.damp.reset();
        for i in 0..LANES {
            let ph = i as f32 * 2.399_963;
            self.lfo_c[i] = ph.cos();
            self.lfo_s[i] = ph.sin();
        }
        for c in 0..2 {
            for b in self.lp[c].iter_mut() {
                b.reset();
            }
            self.tone_lp[c].reset();
            self.tone_hp[c].reset();
        }
        self.update_targets();
        self.snap();
        self.fresh = true;
        self.update_coeffs();
        self.wt = self.target_weights();
        self.tone_g = self.target_tone();
    }

    fn set_param(&mut self, slot: u16, v: ParamValue) {
        let i = slot as usize;
        if i == 0 || i > PARAMS.len() {
            return;
        }
        let x = v.as_f32();
        if !x.is_finite() {
            return;
        }
        let p = &PARAMS[i - 1];
        self.raw[i - 1] = x.clamp(p.min, p.max);
        self.dirty = true;
    }

    fn param(&self, slot: u16) -> Option<f32> {
        let i = slot as usize;
        (1..=PARAMS.len()).contains(&i).then(|| self.raw[i - 1])
    }

    fn load_defaults(&mut self) {
        for (r, p) in self.raw.iter_mut().zip(PARAMS.iter()) {
            *r = p.default;
        }
        self.dirty = true;
    }

    fn commit(&mut self) {
        if self.dirty {
            self.update_targets();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn chain_group_delay_rises_with_frequency() {
        let k = 5;
        let mut last = 0.0;
        for f in [100.0f32, 500.0, 1000.0, 2000.0, 3000.0, 4000.0, 4700.0] {
            let w = core::f32::consts::TAU * f / 48_000.0;
            let g = chain_group_delay(0.68, k, w);
            assert!(g > last, "{f} {g} {last}");
            last = g;
        }
        let dc = chain_group_delay(0.68, k, 0.0);
        assert!((dc - 80.0 * 5.0 * 0.32 / 1.68).abs() < 1e-2, "{dc}");
    }

    #[test]
    fn weights_are_power_normalised() {
        for d in [0.0, 0.2, 0.5, 0.9, 1.0] {
            let w = weights(d);
            let e: f32 = w.iter().map(|x| x * x).sum();
            assert!((e - 1.0).abs() < 1e-5);
        }
        assert_eq!(weights(0.0), [1.0, 0.0, 0.0, 0.0]);
        assert!(weights(1.0).iter().all(|&x| (x - 0.5).abs() < 1e-6));
    }
}
