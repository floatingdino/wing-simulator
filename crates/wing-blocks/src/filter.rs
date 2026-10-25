//! Standard channel filter block (`ch.<n>.flt`, models.md §2): low cut and high cut.
//!
//! - Low cut: `lc` on/off, `lcf` 20 .. 2000 Hz, `lcs` 6 / 12 / 18 / 24 dB/oct.
//! - High cut: `hc` on/off, `hcf` 50 .. 20000 Hz, `hcs` 6 / 12 dB/oct.
//! - Order inside the block (signal-flow.md §2, [I]): low cut -> high cut. The filter *tool* (`tf`, `mdl`
//!   TILT/MAX/AP1/AP2) is a separate task (T046) and not part of this block.
//!
//! Response (T03C, verified against `data/reference-curves/filter.json`): WING-Edit's own designs from
//! `ae::Cchfilter::update` ([`crate::wingcoef`]), all -3.0 dB at the cutoff:
//! - LC 6: 1st-order bilinear HP (`hqhp6`; its >2.5 kHz table never applies, LC <= 2 kHz).
//! - LC 12: `hqhp12` (bilinear Butterworth HP, Q 0.707 below 2.5 kHz).
//! - LC 18: 1st-order HP + bilinear 2nd-order HP with Q 1.0. LC 24: bilinear HPs with Q 0.541 and 1.307.
//! - HC 6: Massberg analog-matched 1st-order LP (no cramping near Nyquist). HC 12: `hqlp12`, Massberg
//!   analog-matched 2nd-order LP with Q 0.707.
//!
//! Sections run in f64 ([`wing_dsp::Cascade`]) with WING's f32 coefficients, so 20 Hz cuts stay accurate.
//!
//! Smoothing: cutoff frequencies follow a 20 ms one-pole in log2(Hz), coefficients are recomputed every
//! [`SUB_BLOCK`] samples while gliding. On/off switches crossfade dry/filtered over a 5 ms one-pole; a cut
//! that is off and faded out costs nothing. A slope change switches immediately (added sections start
//! from zero state).

use crate::wingcoef::{self, Bq, Ctx};
use wing_dsp::biquad::{CutKind, Sections};
use wing_dsp::{Cascade, Coeffs, OnePoleSmoother};

/// Coefficient update interval while a cutoff glides (samples).
pub const SUB_BLOCK: usize = 16;
const FREQ_TAU_S: f64 = 0.02;
const MIX_TAU_S: f64 = 0.005;

/// Low-cut slope (`lcs`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum LcSlope {
    Db6,
    Db12,
    Db18,
    /// Factory default (fw 3.x). fw 1.13 snapshots without `lcs` should load as 12 (**verify**).
    #[default]
    Db24,
}

impl LcSlope {
    pub fn order(self) -> usize {
        match self {
            LcSlope::Db6 => 1,
            LcSlope::Db12 => 2,
            LcSlope::Db18 => 3,
            LcSlope::Db24 => 4,
        }
    }
}

/// High-cut slope (`hcs`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum HcSlope {
    Db6,
    #[default]
    Db12,
}

impl HcSlope {
    pub fn order(self) -> usize {
        match self {
            HcSlope::Db6 => 1,
            HcSlope::Db12 => 2,
        }
    }
}

/// Filter block parameters in engineering units.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FilterParams {
    pub lc_on: bool,
    /// 20 .. 2000 Hz.
    pub lc_freq: f32,
    pub lc_slope: LcSlope,
    pub hc_on: bool,
    /// 50 .. 20000 Hz.
    pub hc_freq: f32,
    pub hc_slope: HcSlope,
}

impl Default for FilterParams {
    /// WING-Edit / T003 defaults.
    fn default() -> Self {
        Self {
            lc_on: false,
            lc_freq: 100.237_46,
            lc_slope: LcSlope::Db24,
            hc_on: false,
            hc_freq: 10_018.261,
            hc_slope: HcSlope::Db12,
        }
    }
}

impl FilterParams {
    pub fn clamped(mut self) -> Self {
        self.lc_freq = self.lc_freq.clamp(20.0, 2000.0);
        self.hc_freq = self.hc_freq.clamp(50.0, 20_000.0);
        self
    }
}

/// WING sections of one cut (`Cchfilter` layout: LC uses 2 slots, HC 1).
fn wing_cut(kind: CutKind, order: usize, f: f32, fs: f64) -> ([Bq; 2], usize) {
    let c = Ctx::new(fs);
    let mut s = [wingcoef::IDENTITY; 2];
    let n = match (kind, order) {
        (CutKind::Highpass, 1) => {
            s[0] = wingcoef::hp6(&c, f);
            1
        }
        (CutKind::Highpass, 2) => {
            s[0] = wingcoef::hp12(&c, f);
            1
        }
        (CutKind::Highpass, 3) => {
            s[0] = wingcoef::hp6(&c, f);
            s[1] = wingcoef::hp12_bilinear(&c, f, 1.0);
            2
        }
        (CutKind::Highpass, _) => {
            s[0] = wingcoef::hp12_bilinear(&c, f, f32::from_bits(0x3f0a8bd4));
            s[1] = wingcoef::hp12_bilinear(&c, f, f32::from_bits(0x3fa73d75));
            2
        }
        (CutKind::Lowpass, 1) => {
            s[0] = wingcoef::lp6(&c, f);
            1
        }
        (CutKind::Lowpass, _) => {
            s[0] = wingcoef::lp12(&c, f, core::f32::consts::FRAC_1_SQRT_2);
            1
        }
    };
    (s, n)
}

fn cut_sections(kind: CutKind, order: usize, f: f32, fs: f64) -> Sections {
    let (s, n) = wing_cut(kind, order, f, fs);
    let mut coeffs = [Coeffs::IDENTITY; 4];
    for (c, b) in coeffs.iter_mut().zip(&s[..n]) {
        *c = Coeffs {
            b0: b[0] as f64,
            b1: b[1] as f64,
            b2: b[2] as f64,
            a1: b[3] as f64,
            a2: b[4] as f64,
        };
    }
    Sections { coeffs, len: n }
}

/// One stereo cut (LC or HC) with smoothed cutoff and on/off crossfade.
struct Cut {
    kind: CutKind,
    order: usize,
    on: bool,
    /// Exact target cutoff (designs use it once the glide settles).
    freq: f32,
    stages: [Cascade; 2],
    log2_f: OnePoleSmoother,
    mix: OnePoleSmoother,
    active: bool,
}

impl Cut {
    fn new(kind: CutKind) -> Self {
        Self {
            kind,
            order: 2,
            on: false,
            freq: 1000.0,
            stages: [Cascade::default(); 2],
            log2_f: OnePoleSmoother::new(10.0, FREQ_TAU_S, 48_000.0),
            mix: OnePoleSmoother::new(0.0, MIX_TAU_S, 48_000.0),
            active: false,
        }
    }

    fn design(&mut self, fs: f64) {
        let f = if self.log2_f.is_settled(1e-6) {
            self.freq
        } else {
            2f64.powf(self.log2_f.value() as f64) as f32
        };
        let s = cut_sections(self.kind, self.order, f, fs);
        for c in &mut self.stages {
            c.set_sections(&s);
        }
    }

    fn set(&mut self, on: bool, freq: f32, order: usize, fs: f64) {
        self.freq = freq;
        self.log2_f.set_target(freq.log2());
        self.mix.set_target(if on { 1.0 } else { 0.0 });
        let order_changed = order != self.order;
        self.order = order;
        self.on = on;
        if on && !self.active {
            // Fade in from bypass: start at the target cutoff with fresh state.
            self.log2_f.reset(freq.log2());
            for c in &mut self.stages {
                c.reset();
            }
            self.active = true;
            self.design(fs);
        } else if order_changed && self.active {
            self.design(fs);
        }
    }

    fn reset(&mut self, fs: f64) {
        self.log2_f.reset(self.log2_f.target());
        self.mix.reset(self.mix.target());
        self.active = self.on;
        for c in &mut self.stages {
            c.reset();
        }
        self.design(fs);
    }

    fn process(&mut self, left: &mut [f32], right: &mut [f32], fs: f64) {
        if !self.active {
            return;
        }
        let n = left.len();
        let mut start = 0;
        while start < n {
            let end = (start + SUB_BLOCK).min(n);
            if !self.log2_f.is_settled(1e-6) {
                self.log2_f.advance(end - start);
                self.design(fs);
            }
            let (l, r) = (&mut left[start..end], &mut right[start..end]);
            if self.mix.is_settled(1e-6) && self.mix.target() == 1.0 {
                self.stages[0].process_block(l);
                self.stages[1].process_block(r);
            } else {
                for (xl, xr) in l.iter_mut().zip(r.iter_mut()) {
                    let m = self.mix.tick();
                    let yl = self.stages[0].process(*xl);
                    let yr = self.stages[1].process(*xr);
                    *xl += m * (yl - *xl);
                    *xr += m * (yr - *xr);
                }
            }
            start = end;
        }
        for c in &mut self.stages {
            c.flush_denormals();
        }
        if !self.on && self.mix.is_settled(1e-5) {
            self.mix.reset(0.0);
            self.active = false;
        }
    }
}

/// Standard low-cut / high-cut filter block (stereo).
pub struct FilterBlock {
    params: FilterParams,
    fs: f64,
    lc: Cut,
    hc: Cut,
}

impl Default for FilterBlock {
    fn default() -> Self {
        Self::new()
    }
}

impl FilterBlock {
    pub fn new() -> Self {
        let mut s = Self {
            params: FilterParams::default(),
            fs: 48_000.0,
            lc: Cut::new(CutKind::Highpass),
            hc: Cut::new(CutKind::Lowpass),
        };
        let p = s.params;
        s.set_params(&p);
        s.reset();
        s
    }

    /// Set the sample rate and reset state (settles all smoothers). `max_block` is unused.
    pub fn prepare(&mut self, sample_rate: f64, _max_block: usize) {
        self.fs = sample_rate;
        for c in [&mut self.lc, &mut self.hc] {
            c.log2_f.set_time(FREQ_TAU_S, sample_rate);
            c.mix.set_time(MIX_TAU_S, sample_rate);
        }
        self.reset();
    }

    /// Clear filter state and jump smoothers to their targets.
    pub fn reset(&mut self) {
        self.lc.reset(self.fs);
        self.hc.reset(self.fs);
    }

    pub fn params(&self) -> &FilterParams {
        &self.params
    }

    /// Update parameters (RT-safe). Values are clamped.
    pub fn set_params(&mut self, p: &FilterParams) {
        let p = p.clamped();
        self.params = p;
        self.lc.set(p.lc_on, p.lc_freq, p.lc_slope.order(), self.fs);
        self.hc.set(p.hc_on, p.hc_freq, p.hc_slope.order(), self.fs);
    }

    /// Process one stereo block in place.
    pub fn process(&mut self, left: &mut [f32], right: &mut [f32]) {
        debug_assert_eq!(left.len(), right.len());
        self.lc.process(left, right, self.fs);
        self.hc.process(left, right, self.fs);
    }

    /// Analytic magnitude (dB) of the block at `f` Hz for the current target parameters (UI curves, tests).
    pub fn magnitude_db(&self, f: f64) -> f64 {
        let (s, n) = self.wing_sections();
        wingcoef::response(self.fs, &s[..n], f).db()
    }

    /// The four `Cchfilter` biquads WING-Edit computes (LC, LC, HC, filter tool = identity here) as
    /// `[b0, b1, b2, a1, a2]`; switched-off cuts are identity.
    pub fn wing_sections(&self) -> ([Bq; 4], usize) {
        let p = &self.params;
        let mut out = [wingcoef::IDENTITY; 4];
        if p.lc_on {
            let (s, n) = wing_cut(CutKind::Highpass, p.lc_slope.order(), p.lc_freq, self.fs);
            out[..n].copy_from_slice(&s[..n]);
        }
        if p.hc_on {
            let (s, _) = wing_cut(CutKind::Lowpass, p.hc_slope.order(), p.hc_freq, self.fs);
            out[2] = s[0];
        }
        (out, 4)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f64::consts::PI;
    use wing_dsp::dtft;

    const FS: f64 = 48_000.0;

    /// Closed-form digital Butterworth magnitude (dB) of the bilinear design.
    fn analytic_db(kind: CutKind, fc: f64, order: usize, f: f64) -> f64 {
        let w = (PI * f / FS).tan() / (PI * fc / FS).tan();
        let w = match kind {
            CutKind::Lowpass => w,
            CutKind::Highpass => 1.0 / w,
        };
        -10.0 * (1.0 + w.powi(2 * order as i32)).log10()
    }

    fn impulse_response(block: &mut FilterBlock, n: usize) -> Vec<f64> {
        let mut l = vec![0.0f32; n];
        let mut r = vec![0.0f32; n];
        l[0] = 1.0;
        for (cl, cr) in l.chunks_mut(64).zip(r.chunks_mut(64)) {
            block.process(cl, cr);
        }
        l.iter().map(|&x| x as f64).collect()
    }

    fn freqs() -> impl Iterator<Item = f64> {
        (0..=60)
            .map(|i| 10.0 * 2f64.powf(i as f64 / 6.0))
            .filter(|&f| f < 22_000.0)
    }

    #[test]
    fn low_cut_matches_butterworth_all_slopes() {
        for (slope, order) in [
            (LcSlope::Db6, 1),
            (LcSlope::Db12, 2),
            (LcSlope::Db18, 3),
            (LcSlope::Db24, 4),
        ] {
            for fc in [20.0, 100.0, 2000.0] {
                let mut b = FilterBlock::new();
                b.set_params(&FilterParams {
                    lc_on: true,
                    lc_freq: fc as f32,
                    lc_slope: slope,
                    ..Default::default()
                });
                b.prepare(FS, 64);
                let h = impulse_response(&mut b, 96_000);
                for f in freqs() {
                    let want = analytic_db(CutKind::Highpass, fc, order, f);
                    if want < -60.0 {
                        continue;
                    }
                    let got = dtft(&h, 2.0 * PI * f / FS).db();
                    assert!(
                        (got - want).abs() < 0.1,
                        "LC {fc} Hz order {order} at {f:.1} Hz: got {got:.3} want {want:.3}"
                    );
                    // f32 coefficients: sub-audio points of a 20 Hz cut are rounding-sensitive.
                    if f >= 20.0 {
                        assert!(
                            (b.magnitude_db(f) - want).abs() < 0.05,
                            "LC = bilinear Butterworth: {fc} {order} {f}"
                        );
                    }
                }
                let at_fc = dtft(&h, 2.0 * PI * fc / FS).db();
                assert!((at_fc + 3.0103).abs() < 0.02, "-3 dB at fc: {at_fc}");
            }
        }
    }

    #[test]
    fn high_cut_matches_butterworth_all_slopes() {
        for (slope, order) in [(HcSlope::Db6, 1), (HcSlope::Db12, 2)] {
            for fc in [50.0, 1000.0, 20_000.0] {
                let mut b = FilterBlock::new();
                b.set_params(&FilterParams {
                    hc_on: true,
                    hc_freq: fc as f32,
                    hc_slope: slope,
                    ..Default::default()
                });
                b.prepare(FS, 64);
                let h = impulse_response(&mut b, 48_000);
                // WING HC is analog-matched (Massberg): processed = analytic WING design; -3 dB at fc and
                // Butterworth-like well below Nyquist.
                for f in freqs() {
                    let want = b.magnitude_db(f);
                    if want < -60.0 {
                        continue;
                    }
                    let got = dtft(&h, 2.0 * PI * f / FS).db();
                    assert!(
                        (got - want).abs() < 0.1,
                        "HC {fc} Hz order {order} at {f:.1} Hz: got {got:.3} want {want:.3}"
                    );
                    if f < 2000.0 && fc <= 1000.0 {
                        let bw = analytic_db(CutKind::Lowpass, fc, order, f);
                        if bw > -40.0 {
                            assert!((want - bw).abs() < 0.3, "Butterworth-like at {f}");
                        }
                    }
                }
                if fc < 20_000.0 {
                    assert!((b.magnitude_db(fc) + 3.01).abs() < 0.05, "-3 dB at fc");
                }
            }
        }
    }

    #[test]
    fn combined_and_bypass() {
        let mut b = FilterBlock::new();
        b.prepare(FS, 64);
        let mut l: Vec<f32> = (0..256).map(|i| (i as f32 * 0.1).sin()).collect();
        let orig = l.clone();
        let mut r = l.clone();
        b.process(&mut l, &mut r);
        assert_eq!(l, orig, "off = bit-exact bypass");
        b.set_params(&FilterParams {
            lc_on: true,
            lc_freq: 80.0,
            hc_on: true,
            hc_freq: 8000.0,
            ..Default::default()
        });
        b.prepare(FS, 64);
        let h = impulse_response(&mut b, 48_000);
        for f in freqs() {
            let want = b.magnitude_db(f);
            let bw = analytic_db(CutKind::Highpass, 80.0, 4, f)
                + analytic_db(CutKind::Lowpass, 8000.0, 2, f);
            if f < 4000.0 && bw > -40.0 {
                assert!((want - bw).abs() < 0.1);
            }
            if want < -60.0 {
                continue;
            }
            assert!((dtft(&h, 2.0 * PI * f / FS).db() - want).abs() < 0.1);
        }
    }

    #[test]
    fn switching_and_sweeps_are_smooth() {
        // A 1 kHz sine through an LC switched on at 900 Hz and swept: no discontinuities beyond the
        // sine's own slope.
        let mut b = FilterBlock::new();
        b.prepare(FS, 64);
        let w = 2.0 * PI * 1000.0 / FS;
        let mut prev = 0.0f32;
        let mut max_step = 0.0f32;
        for blk in 0..200 {
            if blk == 20 {
                b.set_params(&FilterParams {
                    lc_on: true,
                    lc_freq: 900.0,
                    ..Default::default()
                });
            }
            if blk == 80 {
                b.set_params(&FilterParams {
                    lc_on: true,
                    lc_freq: 20.0,
                    ..Default::default()
                });
            }
            if blk == 140 {
                b.set_params(&FilterParams::default());
            }
            let mut l: Vec<f32> = (0..64)
                .map(|i| ((blk * 64 + i) as f64 * w).sin() as f32)
                .collect();
            let mut r = l.clone();
            b.process(&mut l, &mut r);
            for &x in &l {
                max_step = max_step.max((x - prev).abs());
                prev = x;
            }
        }
        // Sine slope bound: w * peak (peak may reach ~1.3 with LC resonance at 24 dB/oct).
        assert!(max_step < 1.4 * w as f32, "max step {max_step}");
        assert!(!b.lc.active, "faded-out cut goes idle");
    }
}
