//! `C5-CMB` (COMBINATOR): 5-band multiband compressor/limiter.
//!
//! Slots (schema): 1 `thr` -40..0 dB, 2 `gain` -10..10 dB, 3 `ratio` (1.1 1.2 1.3 1.5 1.7 2 2.5 3 3.5 4 5 7 10
//! 100), 4 `slope` (24, 48 dB/oct), 5 `bandsel` 1..5, 6 `att` 0..20 ms, 7 `rel` 20..3000 ms, 8 `arel` auto
//! release, 9 `sbc` SBC speed 1..10, 10 `sbcon`, 11..15 `thr_n` -10..10 dB, 16..20 `gain_n` -10..10 dB,
//! 21..25 `byp_n`, 26..30 `width_n` -50..50, 31 `mix` 0..100 %, 32 `$bdsolo`. Defaults
//! (`fx_init_combinator`): 0, 0, 2.5, 48, 3, 5, 500 (494 on the log grid), on, 5, on, all band values 0,
//! mix 100, solo off.
//!
//! # Crossover (from `fx_parchg_combinator`)
//! Four fixed crossovers at `20 kHz / 10^(0.6 m)`, m = 4..1 = 79.6, 317, 1262, 5024 Hz, Linkwitz-Riley built
//! from bilinear Butterworth sections with shared denominators (WING posts LP/HP pairs with identical poles,
//! Q 0.5412/1.3066 for 48 dB/oct, Q 0.7071 for 24 dB/oct). Tree: split at f1, the high part at f2, ...; the
//! lower bands are passed through the all-pass of every later crossover (`LP + HP` of an LR filter is an
//! all-pass with the same poles), so the five bands sum to an all-pass: **flat magnitude** (tested). Designed in f64.
//!
//! # Dynamics [I] (the gain computer is DSP firmware)
//! * Per band: stereo-linked mean-square detector (tau 2 ms) -> level dB; static curve: hard knee at
//!   `thr + thr_n`, `ratio`; gain reduction smoothed in dB (attack `att` ms, release `rel` ms; `att` 0 = 0.05 ms).
//! * `arel` (auto release): release time scales with the crest of the reduction, `rel_eff = rel * (0.2 + 0.8 *
//!   min(1, GR/12 dB))`... i.e. short for light gain reduction, up to `rel` for heavy reduction.
//! * SBC (spectral balance control): when on, each band's gain reduction is pulled towards the mean reduction
//!   of the five bands by 50 %, smoothed with `tau = 2 s / sbc` (speed 1..10), so one loud band does not
//!   swing the tonal balance.
//! * Band output: `* 10^((gain + gain_n)/20)`; `byp_n` bypasses the band's compressor and gain; `width_n`
//!   scales the band's side signal by `1 + width/50` (0 = mono .. 2 = double width).
//! * `mix`: blend with the dry signal passed through the same all-pass chain (phase-coherent).
//! * `$bdsolo`: output only band `bandsel` (listen).

use super::{lin2db, Section};
use crate::model::{Model, ModelTelemetry, ParamValue};
use core::f64::consts::PI;
use wing_dsp::smooth::one_pole_coeff;

/// Number of bands.
pub const BANDS: usize = 5;
/// Crossover frequencies (Hz), low to high.
pub fn crossovers() -> [f64; 4] {
    [4.0, 3.0, 2.0, 1.0].map(|m| 20_000.0 / 10f64.powf(0.6 * m))
}
/// `ratio` table.
pub const RATIOS: [f32; 14] = [
    1.1, 1.2, 1.3, 1.5, 1.7, 2.0, 2.5, 3.0, 3.5, 4.0, 5.0, 7.0, 10.0, 100.0,
];
/// Control-rate block.
pub const CTRL: usize = 8;
const DET_TAU_S: f64 = 0.002;
const SBC_STRENGTH: f64 = 0.5;

/// WING-Edit defaults in slot order (ratio as value, slope as index 1 = 48).
pub const DEFAULTS: [f32; 32] = [
    0.0, 0.0, 2.5, 1.0, 3.0, 5.0, 494.0, 1.0, 5.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0,
];

/// Bilinear Butterworth 2nd-order LP / HP with a shared denominator (`[b0, b1, b2, a1, a2]`, f64).
pub fn bw2(fs: f64, f: f64, q: f64, hp: bool) -> [f64; 5] {
    let k = (PI * f / fs).tan();
    let k2 = k * k;
    let n = 1.0 / (1.0 + k / q + k2);
    let a1 = 2.0 * (k2 - 1.0) * n;
    let a2 = (1.0 - k / q + k2) * n;
    let (b0, b1) = if hp {
        (n, -2.0 * n)
    } else {
        (k2 * n, 2.0 * k2 * n)
    };
    [b0, b1, b0, a1, a2]
}

/// All-pass with the poles of `s` (`(a2 + a1 z^-1 + z^-2) / (1 + a1 z^-1 + a2 z^-2)`).
pub fn allpass_of(s: &[f64; 5]) -> [f64; 5] {
    [s[4], s[3], 1.0, s[3], s[4]]
}

/// Q values of the Butterworth stages of one LR side: LR24 = BW2 (squared), LR48 = BW4 (squared).
fn stage_qs(lr48: bool) -> &'static [f64] {
    if lr48 {
        &[0.541_196_100_146_197, 1.306_562_964_876_376_7]
    } else {
        &[core::f64::consts::FRAC_1_SQRT_2]
    }
}

/// One crossover: LP and HP chains (each BW stage applied twice) and the matching all-pass.
#[derive(Clone, Copy, Debug, Default)]
struct Xover {
    lp: [Section; 4],
    hp: [Section; 4],
    n: usize,
}

impl Xover {
    fn design(&mut self, fs: f64, f: f64, lr48: bool) -> [[f64; 5]; 2] {
        let qs = stage_qs(lr48);
        let mut ap = [[1.0, 0.0, 0.0, 0.0, 0.0]; 2];
        self.n = 2 * qs.len();
        for (k, &q) in qs.iter().enumerate() {
            let l = bw2(fs, f, q, false);
            let h = bw2(fs, f, q, true);
            self.lp[2 * k].set64(l);
            self.lp[2 * k + 1].set64(l);
            self.hp[2 * k].set64(h);
            self.hp[2 * k + 1].set64(h);
            ap[k] = allpass_of(&l);
        }
        ap
    }
    #[inline]
    fn split(&mut self, l: f64, r: f64) -> ((f64, f64), (f64, f64)) {
        let (mut ll, mut lr) = (l, r);
        let (mut hl, mut hr) = (l, r);
        for s in &mut self.lp[..self.n] {
            (ll, lr) = s.process(ll, lr);
        }
        for s in &mut self.hp[..self.n] {
            (hl, hr) = s.process(hl, hr);
        }
        ((ll, lr), (hl, hr))
    }
    fn clear(&mut self) {
        for s in self.lp.iter_mut().chain(self.hp.iter_mut()) {
            s.clear();
        }
    }
    fn flush(&mut self) {
        for s in self.lp.iter_mut().chain(self.hp.iter_mut()) {
            s.flush();
        }
    }
}

/// Per-band dynamics state.
#[derive(Clone, Copy, Debug, Default)]
struct Band {
    ms: [f64; 2],
    gr: f64,
    dev: f64,
    gain: f64,
    gain_target: f64,
    step: f64,
}

/// COMBINATOR.
pub struct Combinator {
    fs: f64,
    v: [f32; 32],
    dirty: bool,
    xo: [Xover; 4],
    /// Compensation all-passes: band b (0..3) passes through the all-passes of crossovers b+1..3; the dry path
    /// through all four. `ap[x][k]` = stage k of crossover x, one section per (band, x) user.
    ap_band: [[[Section; 2]; 4]; 4],
    ap_dry: [[Section; 2]; 4],
    n_ap: usize,
    bands: [Band; BANDS],
    det_c: f64,
    sbc_c: f64,
    phase: usize,
    gr_max: f32,
}

impl Default for Combinator {
    fn default() -> Self {
        Self::new()
    }
}

impl Combinator {
    pub fn new() -> Self {
        let mut m = Self {
            fs: 48_000.0,
            v: DEFAULTS,
            dirty: true,
            xo: [Xover::default(); 4],
            ap_band: [[[Section::default(); 2]; 4]; 4],
            ap_dry: [[Section::default(); 2]; 4],
            n_ap: 1,
            bands: [Band::default(); BANDS],
            det_c: 0.0,
            sbc_c: 0.0,
            phase: 0,
            gr_max: 0.0,
        };
        m.prepare(48_000.0, 64);
        m
    }

    fn lr48(&self) -> bool {
        self.v[3] != 0.0
    }

    /// Current per-band gain reduction (dB).
    pub fn gain_reduction_db(&self) -> [f32; BANDS] {
        self.bands.map(|b| b.gr as f32)
    }

    fn design(&mut self) {
        let fs = self.fs;
        let lr48 = self.lr48();
        let fx = crossovers();
        self.n_ap = stage_qs(lr48).len();
        for x in 0..4 {
            let ap = self.xo[x].design(fs, fx[x], lr48);
            for k in 0..self.n_ap {
                for b in 0..4 {
                    self.ap_band[b][x][k].set64(ap[k]);
                }
                self.ap_dry[x][k].set64(ap[k]);
            }
        }
    }

    fn update(&mut self) {
        self.dirty = false;
        let speed = self.v[8].clamp(1.0, 10.0) as f64;
        self.sbc_c = one_pole_coeff(2.0 / speed, self.fs / CTRL as f64);
    }

    /// Static gain reduction (dB, >= 0) of band `b` for a level (dB).
    pub fn static_gr_db(&self, b: usize, level_db: f64) -> f64 {
        let thr = (self.v[0] + self.v[10 + b]) as f64;
        let ratio = self.v[2].max(1.0) as f64;
        (level_db - thr).max(0.0) * (1.0 - 1.0 / ratio)
    }

    fn control(&mut self) {
        let cfs = self.fs / CTRL as f64;
        let att_ms = (self.v[5] as f64).max(0.05);
        let att_c = one_pole_coeff(att_ms * 1e-3, cfs);
        let rel_ms = (self.v[6] as f64).clamp(20.0, 3000.0);
        let arel = self.v[7] != 0.0;
        let mut target = [0.0f64; BANDS];
        for b in 0..BANDS {
            let ms = self.bands[b].ms[0].max(self.bands[b].ms[1]);
            let level = 10.0 * ms.max(1e-24).log10() + 3.0103; // RMS of a sine reads its peak
            target[b] = self.static_gr_db(b, level);
        }
        let mean = target.iter().sum::<f64>() / BANDS as f64;
        let sbc = self.v[9] != 0.0;
        let mut gr_max = 0.0f64;
        for (b, t) in target.into_iter().enumerate() {
            let band = &mut self.bands[b];
            let dev = t - mean;
            band.dev = dev + self.sbc_c * (band.dev - dev);
            let t = if sbc {
                (t - SBC_STRENGTH * band.dev).max(0.0)
            } else {
                t
            };
            let c = if t > band.gr {
                att_c
            } else {
                let rel = if arel {
                    rel_ms * (0.2 + 0.8 * (band.gr / 12.0).min(1.0))
                } else {
                    rel_ms
                };
                one_pole_coeff(rel * 1e-3, cfs)
            };
            band.gr = t + c * (band.gr - t);
            let byp = self.v[20 + b] != 0.0;
            let g_db = if byp {
                0.0
            } else {
                (self.v[1] + self.v[15 + b]) as f64 - band.gr
            };
            band.gain_target = 10f64.powf(g_db / 20.0);
            band.step = (band.gain_target - band.gain) / CTRL as f64;
            if !byp {
                gr_max = gr_max.max(band.gr);
            }
        }
        self.gr_max = gr_max as f32;
        for x in &mut self.xo {
            x.flush();
        }
    }
}

impl Model for Combinator {
    fn id(&self) -> &'static str {
        "C5-CMB"
    }

    fn prepare(&mut self, sample_rate: f64, _max_block: usize) {
        self.fs = sample_rate;
        self.det_c = one_pole_coeff(DET_TAU_S, sample_rate);
        self.design();
        self.update();
        self.reset();
    }

    fn process(&mut self, left: &mut [f32], right: &mut [f32], _sc: Option<(&[f32], &[f32])>) {
        if self.dirty {
            self.update();
        }
        let n = left.len().min(right.len());
        let dc = self.det_c;
        let mix = (self.v[30].clamp(0.0, 100.0) / 100.0) as f64;
        let solo = (self.v[31] != 0.0).then(|| (self.v[4].clamp(1.0, 5.0) as usize) - 1);
        let width: [f64; BANDS] = core::array::from_fn(|b| 1.0 + self.v[25 + b] as f64 / 50.0);
        let nap = self.n_ap;
        for i in 0..n {
            if self.phase == 0 {
                self.control();
            }
            self.phase = (self.phase + 1) % CTRL;
            let (l, r) = (left[i] as f64, right[i] as f64);
            // split
            let mut bands = [(0.0f64, 0.0f64); BANDS];
            let mut rest = (l, r);
            for x in 0..4 {
                let (lo, hi) = self.xo[x].split(rest.0, rest.1);
                bands[x] = lo;
                rest = hi;
            }
            bands[4] = rest;
            // all-pass compensation of the lower bands
            for (b, band) in bands.iter_mut().enumerate().take(4) {
                for x in b + 1..4 {
                    for k in 0..nap {
                        *band = self.ap_band[b][x][k].process(band.0, band.1);
                    }
                }
            }
            let (mut yl, mut yr) = (0.0, 0.0);
            for (b, &(bl, br)) in bands.iter().enumerate() {
                let st = &mut self.bands[b];
                st.ms[0] = bl * bl + dc * (st.ms[0] - bl * bl);
                st.ms[1] = br * br + dc * (st.ms[1] - br * br);
                st.gain += st.step;
                let g = st.gain;
                let (m, s) = ((bl + br) * 0.5, (bl - br) * 0.5 * width[b]);
                let (ol, or) = ((m + s) * g, (m - s) * g);
                if solo.is_none_or(|k| k == b) {
                    yl += ol;
                    yr += or;
                }
            }
            if mix < 1.0 {
                let (mut dl, mut dr) = (l, r);
                for x in 0..4 {
                    for k in 0..nap {
                        (dl, dr) = self.ap_dry[x][k].process(dl, dr);
                    }
                }
                yl = dl + (yl - dl) * mix;
                yr = dr + (yr - dr) * mix;
            }
            left[i] = yl as f32;
            right[i] = yr as f32;
        }
    }

    fn reset(&mut self) {
        if self.dirty {
            self.update();
        }
        for x in &mut self.xo {
            x.clear();
        }
        for s in self.ap_band.iter_mut().flatten().flatten() {
            s.clear();
        }
        for s in self.ap_dry.iter_mut().flatten() {
            s.clear();
        }
        for b in 0..BANDS {
            let byp = self.v[20 + b] != 0.0;
            let g_db = if byp {
                0.0
            } else {
                (self.v[1] + self.v[15 + b]) as f64
            };
            let g = 10f64.powf(g_db / 20.0);
            self.bands[b] = Band {
                gain: g,
                gain_target: g,
                ..Band::default()
            };
        }
        self.phase = 0;
        self.gr_max = 0.0;
    }

    fn set_param(&mut self, slot: u16, value: ParamValue) {
        let Some(i) = (slot as usize).checked_sub(1).filter(|&i| i < 32) else {
            return;
        };
        let x = match i {
            0 => value.as_f32().clamp(-40.0, 0.0),
            1 => value.as_f32().clamp(-10.0, 10.0),
            2 => match value {
                ParamValue::Enum(e) => match RATIOS.get(e as usize) {
                    Some(&r) => r,
                    None => return,
                },
                other => {
                    let v = other.as_f32();
                    RATIOS
                        .iter()
                        .copied()
                        .min_by(|a, b| (a - v).abs().total_cmp(&(b - v).abs()))
                        .unwrap_or(2.5)
                }
            },
            3 => match value {
                // enum index (0 = 24, 1 = 48; the engine pipeline sends it as `Float(index)`, T081) or the
                // value itself (24 / 48)
                ParamValue::Float(f) if f >= 2.0 => (f > 36.0) as u32 as f32,
                other => other.as_i32().clamp(0, 1) as f32,
            },
            4 => value.as_i32().clamp(1, 5) as f32,
            5 => value.as_f32().clamp(0.0, 20.0),
            6 => value.as_f32().clamp(20.0, 3000.0),
            8 => value.as_f32().clamp(1.0, 10.0),
            10..=19 => value.as_f32().clamp(-10.0, 10.0),
            25..=29 => value.as_f32().clamp(-50.0, 50.0),
            30 => value.as_f32().clamp(0.0, 100.0),
            _ => value.as_i32().clamp(0, 1) as f32,
        };
        if !x.is_finite() || x == self.v[i] {
            return;
        }
        let slope_changed = i == 3;
        self.v[i] = x;
        self.dirty = true;
        if slope_changed {
            // crossover topology change (RT-safe: coefficient design only)
            self.design();
        }
    }

    fn param(&self, slot: u16) -> Option<f32> {
        (slot as usize)
            .checked_sub(1)
            .and_then(|i| self.v.get(i).copied())
    }

    fn load_defaults(&mut self) {
        let slope = self.v[3];
        self.v = DEFAULTS;
        self.dirty = true;
        if slope != self.v[3] {
            self.design();
        }
    }

    fn commit(&mut self) {
        if self.dirty {
            self.update();
        }
    }

    fn telemetry(&self) -> ModelTelemetry {
        ModelTelemetry {
            gain_reduction_db: self.gr_max,
            engaged: self.gr_max > 0.01,
        }
    }

    /// Each band's dynamic gain change (`-GR`, 0 while the band is bypassed).
    fn meter_bands(&self, out: &mut [f32]) -> usize {
        let n = BANDS.min(out.len());
        for (b, o) in out.iter_mut().take(n).enumerate() {
            *o = if self.v[20 + b] != 0.0 {
                0.0
            } else {
                -self.bands[b].gr as f32
            };
        }
        n
    }
}

/// Magnitude (dB) of the summed five bands of a crossover tree at `f` Hz (all band gains 1, no dynamics).
pub fn sum_response_db(fs: f64, lr48: bool, f: f64) -> f64 {
    use super::response64;
    let fx = crossovers();
    let qs = stage_qs(lr48);
    let chain = |hp: bool, x: usize| -> wing_dsp::Complex {
        let secs: Vec<[f64; 5]> = qs
            .iter()
            .flat_map(|&q| {
                let b = bw2(fs, fx[x], q, hp);
                [b, b]
            })
            .collect();
        response64(fs, &secs, f)
    };
    let ap = |x: usize| -> wing_dsp::Complex {
        let secs: Vec<[f64; 5]> = qs
            .iter()
            .map(|&q| allpass_of(&bw2(fs, fx[x], q, false)))
            .collect();
        response64(fs, &secs, f)
    };
    let mut total = wing_dsp::Complex::new(0.0, 0.0);
    let mut rest = wing_dsp::Complex::new(1.0, 0.0);
    for b in 0..BANDS {
        let mut h = if b < 4 { rest * chain(false, b) } else { rest };
        if b < 4 {
            rest = rest * chain(true, b);
        }
        for x in b + 1..4 {
            if b < 4 {
                h = h * ap(x);
            }
        }
        total = total + h;
    }
    lin2db(total.norm())
}
