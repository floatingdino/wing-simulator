//! `GEQ` (GRAPHIC EQ, 31 ISO 1/3-octave bands): exact port of WING-Edit's `ae::fx_parchg_geq`
//! (0x100060730).
//!
//! Slots: 1 `type` (STD / TRU), 2..32 the bands 20 Hz..20 kHz (-15..+15 dB), 33 `TRIM` (-15..+15 dB).
//!
//! * **STD**: band i is `hqpeq(f_i, g_i, Q)` with the proportional Q `Q = 4.32 * 10^(|g_i|/40)`
//!   (narrower when boosted/cut harder).
//! * **TRU** ("true response"): every band is driven with its neighbours subtracted,
//!   `g'_i = 1.67 g_i - 0.67 (g_{i-1} + g_{i+1})`, where the missing outer neighbours are `0.5075 g_0` and
//!   `0.5075 g_30`, and a wider `Q = 2.069 * 10^(|g'_i|/40)`, so the summed response follows the fader curve.
//! * `TRIM` scales the numerator of the last (20 kHz) section by `10^(TRIM/20)` (0 for TRIM <= -98).
//!
//! The constants are the double literals of the original. Runtime: 31 cascaded DF1 sections (f64); a 0 dB
//! STD band is the identity section and is skipped. Fader moves glide (20 ms one-pole, redesign every
//! [`SUB_BLOCK`] samples with a per-sample coefficient ramp); a STD/TRU switch ramps over one sub-block.

use super::Section;
use crate::model::{Model, ParamValue};
use crate::wingcoef::{self, Bq, Ctx};
use wing_dsp::smooth::one_pole_coeff;

/// Number of bands.
pub const BANDS: usize = 31;
/// Band centre frequencies (`0x1006173d0`).
pub const FREQS: [f32; BANDS] = [
    20.0, 25.0, 31.5, 40.0, 50.0, 63.0, 80.0, 100.0, 125.0, 160.0, 200.0, 250.0, 315.0, 400.0,
    500.0, 630.0, 800.0, 1000.0, 1250.0, 1600.0, 2000.0, 2500.0, 3150.0, 4000.0, 5000.0, 6300.0,
    8000.0, 10000.0, 12500.0, 16000.0, 20000.0,
];
/// Redesign interval while faders glide.
pub const SUB_BLOCK: usize = 16;
const GLIDE_S: f64 = 0.02;

const LN10_40: f64 = f64::from_bits(0x3fbd791c6109b02d); // ln10/20 (times |g|/2 -> ln10/40)
const Q_STD: f64 = f64::from_bits(0x401147ae147ae148); // 4.32
const Q_TRU: f64 = f64::from_bits(0x40008d3dc8b86b16); // 2.069
const TRU_SELF: f64 = f64::from_bits(0x3ffab851eb851eb8); // 1.67
const TRU_NEIGH: f64 = f64::from_bits(0xbfe570a3d70a3d71); // -0.67
const TRU_EDGE: f64 = f64::from_bits(0x3fe03d22652faf9b); // 0.5075 (missing outer neighbour)
const PEQ_FLAGS: u32 = 0x34021;

/// Schema slots.
pub mod slot {
    pub const TYPE: u16 = 1;
    /// First band (20 Hz); band `i` is slot `BAND0 + i`.
    pub const BAND0: u16 = 2;
    pub const TRIM: u16 = 33;
}

/// `hqpeq` call as WING-Edit's GEQ makes it (flags `0x34021` are not used by the design).
fn band(c: &Ctx, f: f32, g: f32, qbase: f64) -> Bq {
    let _ = PEQ_FLAGS;
    let q = (((g.abs() * 0.5) as f64 * LN10_40).exp() * qbase) as f32;
    wingcoef::peq(c, f, g, q)
}

/// WING's 31 sections for gains `g` (dB), mode `tru` and `trim` (dB).
pub fn design(c: &Ctx, g: &[f32; BANDS], tru: bool, trim: f32, out: &mut [Bq; BANDS]) {
    if !tru {
        for i in 0..BANDS {
            out[i] = band(c, FREQS[i], g[i], Q_STD);
        }
    } else {
        for i in 0..BANDS {
            let prev = if i == 0 {
                ((g[0] as f64) * TRU_EDGE) as f32
            } else {
                g[i - 1]
            };
            let next = if i + 1 < BANDS {
                g[i + 1]
            } else {
                ((g[BANDS - 1] as f64) * TRU_EDGE) as f32
            };
            // d1 = (prev + next) * -0.67 (double); g' = fma(g, 1.67, d1) (double) -> float
            let s = prev + next;
            let d1 = s as f64 * TRU_NEIGH;
            let gp = (g[i] as f64).mul_add(TRU_SELF, d1) as f32;
            out[i] = band(c, FREQS[i], gp, Q_TRU);
        }
    }
    let t = wingcoef::db2lin(trim);
    let last = &mut out[BANDS - 1];
    last[0] *= t;
    last[1] *= t;
    last[2] *= t;
}

/// The GRAPHIC EQ model.
pub struct GraphicEq {
    ctx: Ctx,
    tru: bool,
    target: [f32; BANDS],
    cur: [f32; BANDS],
    trim_target: f32,
    trim: f32,
    glide: f32,
    dirty: bool,
    mode_changed: bool,
    secs: [Section; BANDS],
    scratch: [Bq; BANDS],
}

impl Default for GraphicEq {
    fn default() -> Self {
        Self::new()
    }
}

impl GraphicEq {
    pub fn new() -> Self {
        let mut m = Self {
            ctx: Ctx::new(48_000.0),
            tru: false,
            target: [0.0; BANDS],
            cur: [0.0; BANDS],
            trim_target: 0.0,
            trim: 0.0,
            glide: 0.0,
            dirty: true,
            mode_changed: false,
            secs: [Section::default(); BANDS],
            scratch: [wingcoef::IDENTITY; BANDS],
        };
        m.prepare(48_000.0, 64);
        m
    }

    /// Sections designed for the staged values (no glide), e.g. for display and tests.
    pub fn design_now(&self, out: &mut [Bq; BANDS]) {
        design(&self.ctx, &self.target, self.tru, self.trim_target, out);
    }

    /// Exact response of the staged design at `f` Hz.
    pub fn response(&self, f: f64) -> wing_dsp::Complex {
        let mut s = [wingcoef::IDENTITY; BANDS];
        self.design_now(&mut s);
        wingcoef::response(self.ctx.fs, &s, f)
    }

    fn snap(&mut self) {
        self.cur = self.target;
        self.trim = self.trim_target;
        design(&self.ctx, &self.cur, self.tru, self.trim, &mut self.scratch);
        for (s, b) in self.secs.iter_mut().zip(&self.scratch) {
            s.set(b);
        }
        self.dirty = false;
        self.mode_changed = false;
    }

    fn control(&mut self, n: usize) {
        if !self.dirty {
            return;
        }
        let mut gliding = false;
        let a = self.glide;
        for i in 0..BANDS {
            let d = self.target[i] - self.cur[i];
            if d == 0.0 {
                continue;
            }
            if d.abs() < 1e-3 {
                self.cur[i] = self.target[i];
            } else {
                self.cur[i] += d * a;
                gliding = true;
            }
        }
        let d = self.trim_target - self.trim;
        if d != 0.0 {
            if d.abs() < 1e-3 {
                self.trim = self.trim_target;
            } else {
                self.trim += d * a;
                gliding = true;
            }
        }
        design(&self.ctx, &self.cur, self.tru, self.trim, &mut self.scratch);
        for (s, b) in self.secs.iter_mut().zip(&self.scratch) {
            s.ramp_to(b, n as u32);
        }
        self.mode_changed = false;
        self.dirty = gliding;
    }
}

impl Model for GraphicEq {
    fn id(&self) -> &'static str {
        "GEQ"
    }

    fn prepare(&mut self, sample_rate: f64, _max_block: usize) {
        self.ctx = Ctx::new(sample_rate);
        self.glide = (1.0 - one_pole_coeff(GLIDE_S, sample_rate / SUB_BLOCK as f64)) as f32;
        for s in &mut self.secs {
            s.clear();
        }
        self.snap();
    }

    fn process(&mut self, left: &mut [f32], right: &mut [f32], _sc: Option<(&[f32], &[f32])>) {
        let n = left.len().min(right.len());
        let mut pos = 0;
        while pos < n {
            let len = SUB_BLOCK.min(n - pos);
            self.control(len);
            for i in pos..pos + len {
                let (mut l, mut r) = (left[i] as f64, right[i] as f64);
                for s in self.secs.iter_mut() {
                    if !s.is_identity() {
                        (l, r) = s.process(l, r);
                    }
                }
                left[i] = l as f32;
                right[i] = r as f32;
            }
            pos += len;
        }
        for s in &mut self.secs {
            s.flush();
        }
    }

    fn reset(&mut self) {
        for s in &mut self.secs {
            s.clear();
        }
        self.snap();
    }

    fn set_param(&mut self, slot: u16, value: ParamValue) {
        match slot {
            slot::TYPE => {
                let t = value.as_i32() != 0;
                if t != self.tru {
                    self.tru = t;
                    self.dirty = true;
                    self.mode_changed = true;
                }
            }
            s if (slot::BAND0..slot::BAND0 + BANDS as u16).contains(&s) => {
                let v = value.as_f32().clamp(-15.0, 15.0);
                let i = (s - slot::BAND0) as usize;
                if v.is_finite() && v != self.target[i] {
                    self.target[i] = v;
                    self.dirty = true;
                }
            }
            slot::TRIM => {
                let v = value.as_f32().clamp(-15.0, 15.0);
                if v.is_finite() && v != self.trim_target {
                    self.trim_target = v;
                    self.dirty = true;
                }
            }
            _ => {}
        }
    }

    fn param(&self, slot: u16) -> Option<f32> {
        match slot {
            slot::TYPE => Some(self.tru as u32 as f32),
            s if (slot::BAND0..slot::BAND0 + BANDS as u16).contains(&s) => {
                Some(self.target[(s - slot::BAND0) as usize])
            }
            slot::TRIM => Some(self.trim_target),
            _ => None,
        }
    }

    /// Defaults: STD, all bands and TRIM 0 dB.
    fn load_defaults(&mut self) {
        self.tru = false;
        self.target = [0.0; BANDS];
        self.trim_target = 0.0;
        self.dirty = true;
    }
}
