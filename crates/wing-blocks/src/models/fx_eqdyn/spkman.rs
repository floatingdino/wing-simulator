//! `SPKMAN` (SPEAKER MANAGER): crossover filters, tilt, phase, polarity, alignment delay, a dynamic EQ band
//! and a limiter, for driving one speaker way.
//!
//! Slots: 1 `hpf` 20..20k Hz, 2 `hptype`, 3 `lpf`, 4 `lptype` (FLAT BW6 BW12 BS12 LR12 BW18 BW24 BS24 LR24
//! BW48 LR48), 5 `tiltf` 100..10k Hz, 6 `tiltg` -6..6 dB, 7 `phase` 0..180, 8 `invert`, 9 `dist` 0..5 m,
//! 10 `pos` -5..5 m, 11 `dyneq`, 12..20 the DEQ band (`deqthr deqratio deqatt deqrel deqfilt deqg deqf deqq
//! deqmode`, as `DEQ`), 21 `lim`, 22 `limthr` -24..0 dB, 23 `limrms`, 24 `limrel` 50..2000 ms.
//!
//! ```text
//! in -> HP (Cxover48 hpf/hptype) -> LP (Cxover48 lpf/lptype) -> tilt -> phase all-pass * polarity
//!    -> delay L/R -> [DEQ band] -> [limiter] -> out
//! ```
//!
//! **Exact** (port of `ae::fx_parchg_spkpro`, 0x10007d790, checked against the emulated original):
//! * HP/LP: `Cxover48(f, type, hi)` = [`wingcoef::xover`] (type index = enum index, FLAT = none).
//! * Tilt: `hqhs12q25(tiltf * k(|g|), -2 g)` with the numerator scaled by `10^(g/20)` (positive = more low
//!   end), `k` = 1 .. 1.2002 .. 1.4233 for `|g|` 0 .. 3 .. 6 dB (the pivot moves up with the amount, as the bus
//!   EQ tilt).
//! * Phase: first-order all-pass whose phase shift at `lpf` is `-phase` degrees: corner
//!   `clamp(lpf * tan((180 - phase) pi/360), 10, 20000)`, `a = (1 - t)/(1 + t)`, `t = tan(pi f/fs)`; phase 0 is
//!   no all-pass (a plain gain section); `invert` negates the section.
//! * Delay: `L = dist + max(pos, 0)`, `R = dist - min(pos, 0)` metres at 343 m/s, truncated to samples, at
//!   most 4000 samples.
//!
//! **Reused**: the dynamic EQ is the channel DEQ band ([`DeqBand`]), keyed from its own input.
//!
//! **Limiter** [I] from the posted constants (`fx_parchg_spkpro` posts `T*1000/limrel`, `1 - limrms` and
//! `sqrt 2` for RMS): stereo-linked, threshold `limthr`, ratio infinite; peak mode: instantaneous attack on the
//! louder channel's |x|; RMS mode: 5 ms mean square times 2 (= level of a sine's peak); release `limrel`
//! (dB-domain one-pole). No look-ahead (latency 0).
//!
//! Parameter changes: filter coefficients ramp over [`RAMP`] samples; delay changes crossfade over 5 ms.

use super::Section;
use crate::model::{Model, ModelTelemetry, ParamValue};
use crate::models::dyn_eq::{DeqBand, DeqBandParams, DeqFilter, DEQ_RATIOS};
use crate::wingcoef::{self, Bq, Ctx};
use wing_dsp::delay::DelayLine;
use wing_dsp::smooth::one_pole_coeff;

/// Coefficient ramp length (samples).
pub const RAMP: u32 = 32;
/// WING's delay clamp (samples).
pub const MAX_DELAY: usize = 4000;
/// Speed of sound used by WING (m/s).
pub const SPEED_OF_SOUND: f64 = 343.0;
const XFADE_S: f64 = 0.005;
const RMS_TAU_S: f64 = 0.005;
/// Number of linear sections (HP 4, LP 4, tilt, phase).
pub const SECS: usize = 10;

/// Filter type tokens (`hptype` / `lptype`).
pub const TYPES: [&str; 11] = [
    "FLAT", "BW6", "BW12", "BS12", "LR12", "BW18", "BW24", "BS24", "LR24", "BW48", "LR48",
];

/// Schema defaults in slot order (enums as index, deqratio as value).
pub const DEFAULTS: [f32; 24] = [
    100.2, 0.0, 2000.0, 0.0, 1000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -20.0, 2.0, 50.0, 301.0, 1.0,
    0.0, 1000.0, 1.0, 1.0, 0.0, 0.0, 1.0, 151.0,
];
const RANGES: [(f32, f32); 24] = [
    (20.0, 20_000.0),
    (0.0, 10.0),
    (20.0, 20_000.0),
    (0.0, 10.0),
    (100.0, 10_000.0),
    (-6.0, 6.0),
    (0.0, 180.0),
    (0.0, 1.0),
    (0.0, 5.0),
    (-5.0, 5.0),
    (0.0, 1.0),
    (-60.0, 0.0),
    (1.2, 10.0),
    (0.0, 200.0),
    (20.0, 4000.0),
    (0.0, 5.0),
    (-15.0, 15.0),
    (20.0, 20_000.0),
    (0.44, 10.0),
    (0.0, 1.0),
    (0.0, 1.0),
    (-24.0, 0.0),
    (0.0, 1.0),
    (50.0, 2000.0),
];

const TILT_K1: f32 = f32::from_bits(0x3f99a027); // 1.2002
const TILT_K2: f32 = f32::from_bits(0x3fb62eb2); // 1.42329
const PHASE_MAX: f64 = f64::from_bits(0x40667ccccccccccd); // 179.9
const PHASE_MAX_F: f32 = f32::from_bits(0x4333e666); // 179.9f
const WPI: f64 = f64::from_bits(0x400921fb54524550);
const PHASE_SMALL: f64 = f64::from_bits(0x3ff91e6837e593e1);

/// Tilt section (`fx_parchg_spkpro`).
pub fn tilt_section(c: &Ctx, tiltf: f32, tiltg: f32) -> Bq {
    let u = ((tiltg.abs() + tiltg.abs()) / 6.0).min(2.0);
    let k = if u >= 1.0 {
        (u + -1.0).mul_add(TILT_K2, (2.0 - u) * TILT_K1)
    } else {
        u.mul_add(TILT_K1, 1.0 - u)
    };
    let mut s = wingcoef::hqhs12q25(c, tiltf * k, tiltg * -2.0);
    let g = wingcoef::db2lin(tiltg);
    s[0] *= g;
    s[1] *= g;
    s[2] *= g;
    s
}

/// Phase all-pass with polarity (`fx_parchg_spkpro`).
pub fn phase_section(c: &Ctx, lpf: f32, phase: f32, invert: bool) -> Bq {
    let sg = if invert { -1.0f32 } else { 1.0 };
    if phase == 0.0 {
        // no all-pass: WING posts (b0, b1) = (1, 0) * polarity
        return [sg, 0.0 * sg, 0.0 * sg, 0.0, 0.0];
    }
    let a = {
        let p = if phase as f64 > PHASE_MAX {
            PHASE_MAX_F
        } else {
            phase
        };
        let d1 = p as f64;
        let d0 = ((180.0f32 - p) as f64 * WPI) / 360.0;
        let d0 = if d1 < 0.1 { PHASE_SMALL } else { d0 };
        let f = (lpf * (d0.tan() as f32)).min(20_000.0).max(10.0);
        let t = (c.pi_over_fs() * f).tan();
        (1.0 - t) / (t + 1.0)
    };
    [-a * sg, sg, 0.0 * sg, -a, 0.0]
}

/// Delays (L, R) in samples (`fx_parchg_spkpro`).
pub fn delays(fs: f64, dist: f32, pos: f32) -> (usize, usize) {
    let lp = if pos > 0.0 { pos } else { -0.0 };
    let rp = if 0.0 > pos { pos } else { 0.0 };
    let (l, r) = (dist + lp, dist - rp);
    let s =
        |m: f32| ((m as f64 / SPEED_OF_SOUND * fs as f32 as f64) as i64).clamp(0, MAX_DELAY as i64);
    (s(l) as usize, s(r) as usize)
}

/// The ten linear sections (HP x4, LP x4, tilt, phase) for slot values `v` (slot order).
pub fn design_linear(c: &Ctx, v: &[f32]) -> [Bq; SECS] {
    let mut out = [wingcoef::IDENTITY; SECS];
    let (hp, n) = wingcoef::xover(c, v[1] as u8, false, v[0]);
    out[..n].copy_from_slice(&hp[..n]);
    let (lp, n) = wingcoef::xover(c, v[3] as u8, true, v[2]);
    out[4..4 + n].copy_from_slice(&lp[..n]);
    out[8] = tilt_section(c, v[4], v[5]);
    out[9] = phase_section(c, v[2], v[6], v[7] != 0.0);
    out
}

fn deq_params(v: &[f32]) -> DeqBandParams {
    DeqBandParams {
        thr_db: v[11],
        ratio: v[12],
        att_ms: v[13],
        rel_ms: v[14],
        filt: DeqFilter::from_index(v[15] as usize),
        gain_db: v[16],
        freq: v[17],
        q: v[18],
        above: v[19] != 0.0,
    }
}

/// SPEAKER MANAGER.
pub struct SpeakerManager {
    ctx: Ctx,
    v: [f32; 24],
    dirty: bool,
    fresh: bool,
    secs: [Section; SECS],
    dl: [DelayLine; 2],
    d_cur: (usize, usize),
    d_new: (usize, usize),
    fade: usize,
    fade_len: usize,
    deq: DeqBand,
    // limiter
    lim_ms: f64,
    lim_gr: f64,
    lim_rel_c: f64,
    rms_c: f64,
}

impl Default for SpeakerManager {
    fn default() -> Self {
        Self::new()
    }
}

impl SpeakerManager {
    pub fn new() -> Self {
        let mut m = Self {
            ctx: Ctx::new(48_000.0),
            v: DEFAULTS,
            dirty: true,
            fresh: true,
            secs: [Section::default(); SECS],
            dl: [DelayLine::new(1), DelayLine::new(1)],
            d_cur: (0, 0),
            d_new: (0, 0),
            fade: 0,
            fade_len: 1,
            deq: DeqBand::new(deq_params(&DEFAULTS)),
            lim_ms: 0.0,
            lim_gr: 0.0,
            lim_rel_c: 0.0,
            rms_c: 0.0,
        };
        m.prepare(48_000.0, 64);
        m
    }

    /// Linear sections of the staged parameters.
    pub fn design_now(&self) -> [Bq; SECS] {
        design_linear(&self.ctx, &self.v)
    }

    /// Target delays (L, R) in samples.
    pub fn delay_samples(&self) -> (usize, usize) {
        self.d_new
    }

    /// Limiter gain reduction (dB).
    pub fn limiter_gr_db(&self) -> f32 {
        self.lim_gr as f32
    }

    fn update(&mut self) {
        self.dirty = false;
        let s = design_linear(&self.ctx, &self.v);
        let n = if self.fresh { 0 } else { RAMP };
        for (sec, b) in self.secs.iter_mut().zip(&s) {
            sec.ramp_to(b, n);
        }
        let d = delays(self.ctx.fs, self.v[8], self.v[9]);
        if self.fresh {
            self.d_cur = d;
            self.d_new = d;
        } else if d != self.d_new {
            if self.fade > 0 {
                // finish the running fade first (jump to its target)
                self.d_cur = self.d_new;
            }
            self.d_new = d;
            self.fade = self.fade_len;
        }
        self.deq.set_params(deq_params(&self.v));
        self.lim_rel_c = one_pole_coeff(self.v[23] as f64 * 1e-3, self.ctx.fs);
    }
}

impl Model for SpeakerManager {
    fn id(&self) -> &'static str {
        "SPKMAN"
    }

    fn prepare(&mut self, sample_rate: f64, _max_block: usize) {
        self.ctx = Ctx::new(sample_rate);
        self.dl = [DelayLine::new(MAX_DELAY + 1), DelayLine::new(MAX_DELAY + 1)];
        self.fade_len = ((XFADE_S * sample_rate) as usize).max(1);
        self.rms_c = one_pole_coeff(RMS_TAU_S, sample_rate);
        self.deq.prepare(sample_rate);
        self.fresh = true;
        self.dirty = true;
        self.reset();
    }

    fn process(&mut self, left: &mut [f32], right: &mut [f32], _sc: Option<(&[f32], &[f32])>) {
        if self.dirty {
            self.update();
        }
        self.fresh = false;
        let n = left.len().min(right.len());
        for i in 0..n {
            let (mut l, mut r) = (left[i] as f64, right[i] as f64);
            for s in self.secs.iter_mut() {
                if !s.is_identity() {
                    (l, r) = s.process(l, r);
                }
            }
            self.dl[0].push(l as f32);
            self.dl[1].push(r as f32);
            let (mut yl, mut yr) = (
                self.dl[0].read_int(self.d_cur.0),
                self.dl[1].read_int(self.d_cur.1),
            );
            if self.fade > 0 {
                let g = 1.0 - self.fade as f32 / self.fade_len as f32;
                let (nl, nr) = (
                    self.dl[0].read_int(self.d_new.0),
                    self.dl[1].read_int(self.d_new.1),
                );
                yl += (nl - yl) * g;
                yr += (nr - yr) * g;
                self.fade -= 1;
                if self.fade == 0 {
                    self.d_cur = self.d_new;
                }
            }
            left[i] = yl;
            right[i] = yr;
        }
        if self.v[10] != 0.0 {
            self.deq.process(left, right, None);
        }
        if self.v[20] != 0.0 {
            let thr = self.v[21] as f64;
            let rms = self.v[22] != 0.0;
            let (rc, mc) = (self.lim_rel_c, self.rms_c);
            for i in 0..n {
                let (l, r) = (left[i] as f64, right[i] as f64);
                let level = if rms {
                    let p = (l * l).max(r * r);
                    self.lim_ms = p + mc * (self.lim_ms - p);
                    10.0 * (2.0 * self.lim_ms).max(1e-24).log10()
                } else {
                    20.0 * l.abs().max(r.abs()).max(1e-12).log10()
                };
                let target = (level - thr).max(0.0);
                self.lim_gr = if target > self.lim_gr {
                    target
                } else {
                    target + rc * (self.lim_gr - target)
                };
                let g = 10f64.powf(-self.lim_gr / 20.0);
                left[i] = (l * g) as f32;
                right[i] = (r * g) as f32;
            }
        } else {
            self.lim_gr = 0.0;
        }
        for s in &mut self.secs {
            s.flush();
        }
    }

    fn reset(&mut self) {
        self.fresh = true;
        self.update();
        for s in &mut self.secs {
            s.clear();
            s.settle();
        }
        for d in &mut self.dl {
            d.reset();
        }
        self.fade = 0;
        self.d_cur = self.d_new;
        self.deq.reset();
        self.lim_ms = 0.0;
        self.lim_gr = 0.0;
    }

    fn set_param(&mut self, slot: u16, value: ParamValue) {
        let Some(i) = (slot as usize).checked_sub(1).filter(|&i| i < 24) else {
            return;
        };
        let (lo, hi) = RANGES[i];
        let x = match i {
            1 | 3 | 7 | 10 | 15 | 19 | 20 | 22 => value.as_i32() as f32,
            12 => match value {
                ParamValue::Enum(e) => match DEQ_RATIOS.get(e as usize) {
                    Some(&r) => r,
                    None => return,
                },
                other => {
                    let v = other.as_f32();
                    DEQ_RATIOS
                        .iter()
                        .copied()
                        .min_by(|a, b| (a - v).abs().total_cmp(&(b - v).abs()))
                        .unwrap_or(2.0)
                }
            },
            _ => value.as_f32(),
        }
        .clamp(lo, hi);
        if x.is_finite() && x != self.v[i] {
            self.v[i] = x;
            self.dirty = true;
        }
    }

    fn param(&self, slot: u16) -> Option<f32> {
        (slot as usize)
            .checked_sub(1)
            .and_then(|i| self.v.get(i).copied())
    }

    fn load_defaults(&mut self) {
        self.v = DEFAULTS;
        self.dirty = true;
    }

    fn commit(&mut self) {
        if self.dirty {
            self.update();
        }
    }

    fn telemetry(&self) -> ModelTelemetry {
        let g = (self.lim_gr as f32).max(self.deq.dynamic_gain_db().abs());
        ModelTelemetry {
            gain_reduction_db: g,
            engaged: g > 0.01,
        }
    }
}
