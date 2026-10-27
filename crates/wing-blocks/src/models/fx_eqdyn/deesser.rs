//! `DE-S2` (2-BAND DEESSER, SPL De-Esser style): level-independent two-band sibilance reduction.
//!
//! Slots: 1 `lo` LOW 0..50, 2 `hi` HIGH 0..50, 3 `los` LOW (S), 4 `his` HIGH (S) (side amounts in M/S mode),
//! 5 `gdr` GENDER (f, m), 6 `mode` (st = STEREO, ms = MID/SIDE). Defaults: all 0, f, st.
//!
//! WING-Edit (`fx_parchg_deesser`) posts the amounts as `amount/1000` and a gender-dependent filter set; the
//! detector itself is DSP firmware. Design [I] (after the SPL De-Esser, which needs no threshold because it
//! compares the sibilant band with the whole signal):
//!
//! ```text
//! per band b (LOW, HIGH):  key -> band-pass (WING hqbp, f_b, Q 2) -> mean-square E_b
//!                          key ->                                     mean-square E
//!   reduction_b = min(0.4 dB * amount_b, 1.5 * max(0, 10 log10(E_b / E) - ONSET_DB))
//!   -> attack/release (dB) -> dynamic WING peaking cut (hqpeq, f_b, Q 1.4, -reduction_b)
//! ```
//!
//! * Band centres [I]: female LOW 6.5 kHz / HIGH 11 kHz, male LOW 5 kHz / HIGH 8.5 kHz.
//! * `st`: both channels share the detector (louder channel) and the gain (stereo-linked). `ms`: mid
//!   `(L+R)/2` uses `lo`/`hi`, side `(L-R)/2` uses `los`/`his`, each with its own detectors.
//! * An amount of 0 is an exact bypass of that band.

use super::Section;
use crate::model::{Model, ModelTelemetry, ParamValue};
use crate::wingcoef::{self, Ctx};
use wing_dsp::smooth::one_pole_coeff;

/// Control-rate block (samples).
pub const CTRL: usize = 8;
/// Band centres (female, male) for (LOW, HIGH).
pub const FREQS: [[f32; 2]; 2] = [[6500.0, 11_000.0], [5000.0, 8500.0]];
/// Band energy ratio (dB) at which reduction starts. [I]
pub const ONSET_DB: f64 = -15.0;
/// dB of reduction per dB of ratio above the onset. [I]
pub const SLOPE: f64 = 1.5;
/// Maximum reduction per amount unit (50 -> 20 dB). [I]
pub const DB_PER_UNIT: f64 = 0.4;
const DET_TAU_S: f64 = 0.002;
const ATTACK_S: f64 = 0.0005;
const RELEASE_S: f64 = 0.04;
const CUT_Q: f32 = 1.4;
const DET_Q: f32 = 2.0;
const FLOOR: f64 = 1e-14;

/// WING-Edit defaults (slot order).
pub const DEFAULTS: [f32; 6] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];

/// One path (stereo pair, mid or side): detectors, gains and cut sections for LOW and HIGH.
#[derive(Clone, Copy, Debug, Default)]
struct Path {
    det: [Section; 2],
    cut: [Section; 2],
    /// Mean squares: [band][channel], broadband [channel].
    eb: [[f64; 2]; 2],
    e: [f64; 2],
    red: [f64; 2],
}

/// 2-BAND DEESSER.
pub struct DeEsser2 {
    ctx: Ctx,
    v: [f32; 6],
    dirty: bool,
    det_c: f64,
    att_c: f64,
    rel_c: f64,
    /// [0] = stereo / mid, [1] = side.
    paths: [Path; 2],
    phase: usize,
}

impl Default for DeEsser2 {
    fn default() -> Self {
        Self::new()
    }
}

impl DeEsser2 {
    pub fn new() -> Self {
        let mut m = Self {
            ctx: Ctx::new(48_000.0),
            v: DEFAULTS,
            dirty: true,
            det_c: 0.0,
            att_c: 0.0,
            rel_c: 0.0,
            paths: [Path::default(); 2],
            phase: 0,
        };
        m.prepare(48_000.0, 64);
        m
    }

    fn male(&self) -> bool {
        self.v[4] != 0.0
    }

    fn ms(&self) -> bool {
        self.v[5] != 0.0
    }

    /// Band centres in use (LOW, HIGH).
    pub fn freqs(&self) -> [f32; 2] {
        FREQS[self.male() as usize]
    }

    /// Current reductions (dB): [path][band] (path 1 only in M/S mode).
    pub fn reductions_db(&self) -> [[f32; 2]; 2] {
        self.paths.map(|p| p.red.map(|r| r as f32))
    }

    /// Static reduction (dB) for a band-to-total energy ratio (dB) at amount `amount`.
    pub fn static_reduction_db(amount: f32, ratio_db: f64) -> f64 {
        let cap = DB_PER_UNIT * amount.clamp(0.0, 50.0) as f64;
        ((ratio_db - ONSET_DB).max(0.0) * SLOPE).min(cap)
    }

    fn update(&mut self) {
        self.dirty = false;
        let f = self.freqs();
        for p in &mut self.paths {
            for b in 0..2 {
                p.det[b].set(&wingcoef::bp(&self.ctx, f[b], DET_Q));
            }
        }
    }

    fn amounts(&self, path: usize) -> [f32; 2] {
        if path == 0 {
            [self.v[0], self.v[1]]
        } else {
            [self.v[2], self.v[3]]
        }
    }

    fn control(&mut self) {
        let f = self.freqs();
        let npaths = if self.ms() { 2 } else { 1 };
        for pi in 0..npaths {
            let amt = self.amounts(pi);
            let p = &mut self.paths[pi];
            for b in 0..2 {
                let target = if amt[b] <= 0.0 {
                    0.0
                } else {
                    // louder channel (st) / the single channel (ms paths use channel 0 only)
                    let (eb, e) = if npaths == 1 {
                        let k = if p.eb[b][0] >= p.eb[b][1] { 0 } else { 1 };
                        (p.eb[b][k], p.e[k])
                    } else {
                        (p.eb[b][0], p.e[0])
                    };
                    let ratio = 10.0 * (eb.max(FLOOR) / e.max(FLOOR)).log10();
                    if e < FLOOR * 10.0 {
                        0.0
                    } else {
                        Self::static_reduction_db(amt[b], ratio)
                    }
                };
                let c = if target > p.red[b] {
                    self.att_c
                } else {
                    self.rel_c
                };
                p.red[b] = target + c * (p.red[b] - target);
                if amt[b] <= 0.0 && p.red[b] < 1e-3 {
                    p.red[b] = 0.0;
                }
                let sec = if p.red[b] <= 1e-4 {
                    wingcoef::IDENTITY
                } else {
                    wingcoef::peq(&self.ctx, f[b], -(p.red[b] as f32), CUT_Q)
                };
                p.cut[b].ramp_to(&sec, CTRL as u32);
            }
            for s in p.det.iter_mut().chain(p.cut.iter_mut()) {
                s.flush();
            }
        }
    }
}

impl Model for DeEsser2 {
    fn id(&self) -> &'static str {
        "DE-S2"
    }

    fn prepare(&mut self, sample_rate: f64, _max_block: usize) {
        self.ctx = Ctx::new(sample_rate);
        let cfs = sample_rate / CTRL as f64;
        self.det_c = one_pole_coeff(DET_TAU_S, sample_rate);
        self.att_c = one_pole_coeff(ATTACK_S, cfs);
        self.rel_c = one_pole_coeff(RELEASE_S, cfs);
        self.update();
        self.reset();
    }

    fn process(&mut self, left: &mut [f32], right: &mut [f32], _sc: Option<(&[f32], &[f32])>) {
        if self.dirty {
            self.update();
        }
        let n = left.len().min(right.len());
        let ms = self.ms();
        let dc = self.det_c;
        for i in 0..n {
            if self.phase == 0 {
                self.control();
            }
            self.phase = (self.phase + 1) % CTRL;
            let (l, r) = (left[i] as f64, right[i] as f64);
            if !ms {
                let p = &mut self.paths[0];
                for (ch, x) in [l, r].into_iter().enumerate() {
                    p.e[ch] = x * x + dc * (p.e[ch] - x * x);
                }
                for b in 0..2 {
                    let (dl, dr) = p.det[b].process(l, r);
                    p.eb[b][0] = dl * dl + dc * (p.eb[b][0] - dl * dl);
                    p.eb[b][1] = dr * dr + dc * (p.eb[b][1] - dr * dr);
                }
                let (mut yl, mut yr) = (l, r);
                for b in 0..2 {
                    if !p.cut[b].is_identity() {
                        (yl, yr) = p.cut[b].process(yl, yr);
                    }
                }
                left[i] = yl as f32;
                right[i] = yr as f32;
            } else {
                let xs = [(l + r) * 0.5, (l - r) * 0.5];
                let mut ys = xs;
                for (pi, &x) in xs.iter().enumerate() {
                    let p = &mut self.paths[pi];
                    p.e[0] = x * x + dc * (p.e[0] - x * x);
                    for b in 0..2 {
                        p.det[b].step();
                        let d = p.det[b].process_ch(0, x);
                        p.eb[b][0] = d * d + dc * (p.eb[b][0] - d * d);
                    }
                    let mut y = x;
                    for b in 0..2 {
                        p.cut[b].step();
                        y = p.cut[b].process_ch(0, y);
                    }
                    ys[pi] = y;
                }
                left[i] = (ys[0] + ys[1]) as f32;
                right[i] = (ys[0] - ys[1]) as f32;
            }
        }
    }

    fn reset(&mut self) {
        if self.dirty {
            self.update();
        }
        for p in &mut self.paths {
            for s in p.det.iter_mut().chain(p.cut.iter_mut()) {
                s.clear();
            }
            for s in &mut p.cut {
                s.set(&wingcoef::IDENTITY);
            }
            p.eb = [[0.0; 2]; 2];
            p.e = [0.0; 2];
            p.red = [0.0; 2];
        }
        self.phase = 0;
    }

    fn set_param(&mut self, slot: u16, value: ParamValue) {
        let Some(i) = (slot as usize).checked_sub(1).filter(|&i| i < 6) else {
            return;
        };
        let x = if i < 4 {
            value.as_f32().clamp(0.0, 50.0)
        } else {
            value.as_i32().clamp(0, 1) as f32
        };
        if x.is_finite() && x != self.v[i] {
            if i == 5 {
                // mode switch: start the new topology from a clean state
                for p in &mut self.paths {
                    for s in p.det.iter_mut().chain(p.cut.iter_mut()) {
                        s.clear();
                    }
                }
            }
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

    /// Words 4..7 (`crate::meter`): GR of LOW, HIGH, LOW (SIDE), HIGH (SIDE) as `GR * 2048 / 6`; the side
    /// words only in M/S mode.
    fn fx_meter_words(&self, out: &mut [f32]) -> usize {
        let r = self.reductions_db();
        let side = if self.ms() { 1.0 } else { 0.0 };
        let w =
            [r[0][0], r[0][1], side * r[1][0], side * r[1][1]].map(|g| g.max(0.0) * 2048.0 / 6.0);
        let n = w.len().min(out.len());
        out[..n].copy_from_slice(&w[..n]);
        n
    }

    fn telemetry(&self) -> ModelTelemetry {
        let g = self.paths.iter().flat_map(|p| p.red).fold(0.0f64, f64::max) as f32;
        ModelTelemetry {
            gain_reduction_db: g,
            engaged: g > 0.01,
        }
    }
}
