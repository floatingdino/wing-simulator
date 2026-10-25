//! One-knob and three-band compressors (T04F): `ONEC` (ONE KNOB COMP) and `3CMP` (THREE BAND COMP).
//! Parameters, ranges, defaults: `docs/research/models.md` §3.2 and `data/schema/wing-tree.json` (ONEC).
//!
//! # ONEC
//! One macro `gr` (0..10, "GAIN RED", default 3) drives the **standard WING COMP engine**
//! ([`crate::dynamics::Dynamics`], RMS detector, log envelope, AUTO release, knee 3) through a 1-D mapping.
//! WING publishes no curve for the knob (models.md: "Map gr through a 1-D table into COMP parameters"), so
//! the mapping is derived from typical one-knob compressor behaviour (more knob = lower threshold, higher
//! ratio, faster timing, auto make-up), with `x = gr / 10`:
//! ```text
//! thr    = -30 x dB                    (0 dB .. -30 dB; gr 0 = no gain reduction)
//! ratio  = 1.5 * (8 / 1.5)^x           (1.5:1 .. 8:1, geometric)
//! att    = 30 * (3 / 30)^x ms          (30 ms .. 3 ms)
//! rel    = 300 * (80 / 300)^x ms       (300 ms .. 80 ms; AUTO release on)
//! makeup = 0.5 * GR(-6 dBFS)           (half of the static gain reduction at -6 dBFS, <= 12 dB)
//! ```
//! `dag` (undocumented, models.md) is taken as "dynamic auto gain": on (default) = the make-up above, off =
//! no make-up. [`onec_map`] returns the mapping; every constant is an assumption (VERIFY-ON-HARDWARE.md).
//!
//! # 3CMP
//! Hidden model (in WING-Edit's table, in no selector; no schema entry, so the slot order below is taken from
//! models.md): `1 knee`, `2 lmf`, `3 mhf`, then per band b = 0..3 `4+5b g`, `5+5b thr`, `6+5b ratio`,
//! `7+5b att`, `8+5b rel`. Signal path (per channel, stereo-linked inside each band):
//! ```text
//! x -> LR4 split @ lmf -> low -> LR4 allpass @ mhf (phase compensation) -> comp1 -> g1 --\
//!                      \-> high -> LR4 split @ mhf -> mid ----------------> comp2 -> g2 ---> sum
//!                                              \-> high ----------------> comp3 -> g3 --/
//! ```
//! The crossover is Linkwitz-Riley 4th order (two cascaded Butterworth biquads,
//! `wing_dsp::biquad::linkwitz_riley`): LP and HP are -6.02 dB at the corner and sum to an **allpass**, and
//! the low band gets an LP+HP pair at `mhf` so all three bands share one phase response: with no gain
//! reduction and 0 dB band gains the output magnitude is flat (tested within +-0.05 dB). WING's own `dynxo`
//! code (`wingcoef::xover`) is a single-corner filter whose LP and HP halves are different designs (Massberg
//! analog-matched LP vs plain bilinear HP), so the pair does not sum flat and is not used. Each band runs the
//! standard COMP engine (RMS, log, no AUTO, global `knee` 0..25 dB rounded to the engine's 7.22 dB steps)
//! followed by the band gain `g` (+-15 dB, smoothed). External key is ignored. Latency 0.
//!
//! Real-time: `prepare` allocates the band scratch; everything else is allocation-free. Bench:
//! `benches/comp_multi.rs`.

use crate::dynamics::{
    static_gain_db, Detector, DynMode, DynParams, Dynamics, Envelope, KNEE_DB_PER_STEP, RATIOS,
};
use crate::model::{Model, ModelTelemetry, ParamValue};
use wing_dsp::biquad::{linkwitz_riley, BiquadF64, CutKind};
use wing_dsp::gain::db_to_lin;
use wing_dsp::smooth::OnePoleSmoother;

/// ONEC engine parameters derived from the macro knob.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OneKnobMap {
    pub thr_db: f32,
    pub ratio: f32,
    pub att_ms: f32,
    pub rel_ms: f32,
    pub makeup_db: f32,
}

/// Knee steps (x 7.22 dB) used by ONEC (the COMP default).
pub const ONEC_KNEE: u8 = 3;
/// Level at which ONEC's auto make-up is evaluated, and the fraction of the GR it restores.
pub const ONEC_MAKEUP_REF_DB: f32 = -6.0;
pub const ONEC_MAKEUP_FRACTION: f32 = 0.5;

/// The `gr` (0..10) -> COMP parameter mapping (see module doc). `auto_gain` = the `dag` switch.
pub fn onec_map(gr: f32, auto_gain: bool) -> OneKnobMap {
    let x = (gr / 10.0).clamp(0.0, 1.0);
    let thr_db = -30.0 * x;
    let ratio = 1.5 * (8.0f32 / 1.5).powf(x);
    let makeup_db = if auto_gain {
        let g = static_gain_db(
            DynMode::Compressor,
            ONEC_MAKEUP_REF_DB,
            thr_db,
            ratio,
            ONEC_KNEE as f32 * KNEE_DB_PER_STEP,
        );
        (-ONEC_MAKEUP_FRACTION * g).clamp(0.0, 12.0)
    } else {
        0.0
    };
    OneKnobMap {
        thr_db,
        ratio,
        att_ms: 30.0 * 0.1f32.powf(x),
        rel_ms: 300.0 * (80.0f32 / 300.0).powf(x),
        makeup_db,
    }
}

/// `ONEC` model.
pub struct OneKnobComp {
    d: Dynamics,
    gr: f32,
    dag: bool,
    dirty: bool,
}

impl Default for OneKnobComp {
    fn default() -> Self {
        Self::new()
    }
}

impl OneKnobComp {
    pub fn new() -> Self {
        let mut m = Self {
            d: Dynamics::new(DynParams::comp()),
            gr: 3.0,
            dag: true,
            dirty: true,
        };
        m.d.prepare(48_000.0, 2);
        m.apply();
        m.d.reset();
        m
    }

    fn apply(&mut self) {
        if !self.dirty {
            return;
        }
        self.dirty = false;
        let m = onec_map(self.gr, self.dag);
        let p = DynParams {
            thr_db: m.thr_db,
            ratio: m.ratio,
            knee: ONEC_KNEE,
            att_ms: m.att_ms,
            rel_ms: m.rel_ms,
            gain_db: m.makeup_db,
            det: Detector::Rms,
            env: Envelope::Log,
            auto: true,
            ..DynParams::comp()
        };
        self.d.set_params(&p);
    }
}

impl Model for OneKnobComp {
    fn id(&self) -> &'static str {
        "ONEC"
    }
    fn prepare(&mut self, sample_rate: f64, _max_block: usize) {
        self.dirty = true;
        self.d.prepare(sample_rate, 2);
        self.apply();
        self.d.reset();
    }
    fn process(
        &mut self,
        left: &mut [f32],
        right: &mut [f32],
        sidechain: Option<(&[f32], &[f32])>,
    ) {
        self.apply();
        let mut io = [left, right];
        match sidechain {
            Some((kl, kr)) => self.d.process(&mut io, Some(&[kl, kr])),
            None => self.d.process(&mut io, None),
        }
    }
    fn reset(&mut self) {
        self.apply();
        self.d.reset();
    }
    fn set_param(&mut self, slot: u16, v: ParamValue) {
        match slot {
            1 => self.gr = v.as_f32().clamp(0.0, 10.0),
            2 => self.dag = v.as_bool(),
            _ => return,
        }
        self.dirty = true;
    }
    fn param(&self, slot: u16) -> Option<f32> {
        match slot {
            1 => Some(self.gr),
            2 => Some(self.dag as u32 as f32),
            _ => None,
        }
    }
    fn load_defaults(&mut self) {
        self.gr = 3.0;
        self.dag = true;
        self.dirty = true;
    }
    fn commit(&mut self) {
        self.apply();
    }
    fn telemetry(&self) -> ModelTelemetry {
        let gr = self.d.gain_reduction_db();
        ModelTelemetry {
            gain_reduction_db: gr,
            engaged: gr > 0.01,
        }
    }
}

// ---- 3CMP -------------------------------------------------------------------------------------------------

/// Number of bands.
pub const BANDS: usize = 3;

/// Per-band parameters.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BandParams {
    pub gain_db: f32,
    pub thr_db: f32,
    pub ratio: f32,
    pub att_ms: f32,
    pub rel_ms: f32,
}

impl Default for BandParams {
    /// models.md defaults (ratio default 3 = 3:1 as for COMP).
    fn default() -> Self {
        Self {
            gain_db: 0.0,
            thr_db: 0.0,
            ratio: 3.0,
            att_ms: 10.0,
            rel_ms: 350.0,
        }
    }
}

/// Stereo LR4 (two biquads per channel).
#[derive(Clone, Copy, Default)]
struct Lr4 {
    s: [[BiquadF64; 2]; 2],
}

impl Lr4 {
    fn set(&mut self, kind: CutKind, fs: f64, f: f64) {
        let sec = linkwitz_riley(kind, fs, f, 4);
        for ch in &mut self.s {
            for (b, c) in ch.iter_mut().zip(sec.as_slice()) {
                b.set_coeffs(*c);
            }
        }
    }
    fn reset(&mut self) {
        for ch in &mut self.s {
            for b in ch {
                b.reset();
            }
        }
    }
    #[inline(always)]
    fn tick(&mut self, ch: usize, x: f32) -> f32 {
        let c = &mut self.s[ch];
        let y = c[0].tick(x as f64);
        c[1].tick(y) as f32
    }
    fn flush(&mut self) {
        for ch in &mut self.s {
            for b in ch {
                b.flush_denormals();
            }
        }
    }
}

/// `3CMP` model.
pub struct ThreeBandComp {
    fs: f64,
    knee_db: f32,
    lmf: f32,
    mhf: f32,
    bands: [BandParams; BANDS],
    dirty: bool,
    xo_dirty: bool,
    lp1: Lr4,
    hp1: Lr4,
    lp2: Lr4,
    hp2: Lr4,
    ap_lp: Lr4,
    ap_hp: Lr4,
    comps: [Dynamics; BANDS],
    gains: [OnePoleSmoother; BANDS],
    buf: [[Vec<f32>; 2]; BANDS],
}

impl Default for ThreeBandComp {
    fn default() -> Self {
        Self::new()
    }
}

impl ThreeBandComp {
    pub fn new() -> Self {
        let mut m = Self {
            fs: 48_000.0,
            knee_db: 5.0,
            lmf: 200.0,
            mhf: 2000.0,
            bands: [BandParams::default(); BANDS],
            dirty: true,
            xo_dirty: true,
            lp1: Lr4::default(),
            hp1: Lr4::default(),
            lp2: Lr4::default(),
            hp2: Lr4::default(),
            ap_lp: Lr4::default(),
            ap_hp: Lr4::default(),
            comps: [
                Dynamics::new(DynParams::comp()),
                Dynamics::new(DynParams::comp()),
                Dynamics::new(DynParams::comp()),
            ],
            gains: [OnePoleSmoother::new(1.0, 0.01, 48_000.0); BANDS],
            buf: Default::default(),
        };
        m.prepare(48_000.0, 64);
        m
    }

    /// Current gain reduction (dB, >= 0) of band `b` (0 = low).
    pub fn band_gr_db(&self, b: usize) -> f32 {
        self.comps[b.min(BANDS - 1)].gain_reduction_db()
    }

    fn apply(&mut self) {
        if self.xo_dirty {
            self.xo_dirty = false;
            let fs = self.fs;
            let nyq = fs * 0.45;
            let lmf = (self.lmf as f64).clamp(20.0, nyq);
            let mhf = (self.mhf as f64).clamp(lmf, nyq);
            self.lp1.set(CutKind::Lowpass, fs, lmf);
            self.hp1.set(CutKind::Highpass, fs, lmf);
            self.lp2.set(CutKind::Lowpass, fs, mhf);
            self.hp2.set(CutKind::Highpass, fs, mhf);
            self.ap_lp.set(CutKind::Lowpass, fs, mhf);
            self.ap_hp.set(CutKind::Highpass, fs, mhf);
        }
        if self.dirty {
            self.dirty = false;
            let knee = (self.knee_db / KNEE_DB_PER_STEP).round().clamp(0.0, 5.0) as u8;
            for (b, p) in self.bands.iter().enumerate() {
                let dp = DynParams {
                    thr_db: p.thr_db,
                    ratio: p.ratio,
                    knee,
                    att_ms: p.att_ms,
                    rel_ms: p.rel_ms,
                    hld_ms: 1.0,
                    det: Detector::Rms,
                    env: Envelope::Log,
                    auto: false,
                    gain_db: 0.0,
                    ..DynParams::comp()
                };
                self.comps[b].set_params(&dp);
                self.gains[b].set_target(db_to_lin(p.gain_db));
            }
        }
    }
}

fn ratio_of(v: ParamValue) -> Option<f32> {
    match v {
        ParamValue::Enum(i) => RATIOS.get(i as usize).copied(),
        other => Some(other.as_f32().clamp(1.0, 100.0)),
    }
}

impl Model for ThreeBandComp {
    fn id(&self) -> &'static str {
        "3CMP"
    }

    fn prepare(&mut self, sample_rate: f64, max_block: usize) {
        self.fs = sample_rate;
        for c in &mut self.comps {
            c.prepare(sample_rate, 2);
        }
        for g in &mut self.gains {
            g.set_time(0.01, sample_rate);
        }
        for b in &mut self.buf {
            for ch in b {
                ch.clear();
                ch.resize(max_block.max(1), 0.0);
            }
        }
        self.xo_dirty = true;
        self.dirty = true;
        self.apply();
        self.reset();
    }

    fn process(
        &mut self,
        left: &mut [f32],
        right: &mut [f32],
        _sidechain: Option<(&[f32], &[f32])>,
    ) {
        self.apply();
        let n = left.len().min(right.len()).min(self.buf[0][0].len());
        for i in 0..n {
            for (ch, s) in [(0usize, left[i]), (1, right[i])] {
                let lo = self.lp1.tick(ch, s);
                let hi = self.hp1.tick(ch, s);
                let lo = self.ap_lp.tick(ch, lo) + self.ap_hp.tick(ch, lo);
                self.buf[0][ch][i] = lo;
                self.buf[1][ch][i] = self.lp2.tick(ch, hi);
                self.buf[2][ch][i] = self.hp2.tick(ch, hi);
            }
        }
        for b in 0..BANDS {
            let [bl, br] = &mut self.buf[b];
            let mut io = [&mut bl[..n], &mut br[..n]];
            self.comps[b].process(&mut io, None);
        }
        for i in 0..n {
            let mut sl = 0.0;
            let mut sr = 0.0;
            for b in 0..BANDS {
                let g = self.gains[b].tick();
                sl += self.buf[b][0][i] * g;
                sr += self.buf[b][1][i] * g;
            }
            left[i] = sl;
            right[i] = sr;
        }
        for f in [
            &mut self.lp1,
            &mut self.hp1,
            &mut self.lp2,
            &mut self.hp2,
            &mut self.ap_lp,
            &mut self.ap_hp,
        ] {
            f.flush();
        }
    }

    fn reset(&mut self) {
        self.apply();
        for f in [
            &mut self.lp1,
            &mut self.hp1,
            &mut self.lp2,
            &mut self.hp2,
            &mut self.ap_lp,
            &mut self.ap_hp,
        ] {
            f.reset();
        }
        for c in &mut self.comps {
            c.reset();
        }
        for (g, p) in self.gains.iter_mut().zip(&self.bands) {
            g.reset(db_to_lin(p.gain_db));
        }
    }

    fn set_param(&mut self, slot: u16, v: ParamValue) {
        match slot {
            1 => {
                self.knee_db = v.as_f32().clamp(0.0, 25.0);
                self.dirty = true;
            }
            2 => {
                self.lmf = v.as_f32().clamp(50.0, 500.0);
                self.xo_dirty = true;
            }
            3 => {
                self.mhf = v.as_f32().clamp(1000.0, 5000.0);
                self.xo_dirty = true;
            }
            4..=18 => {
                let i = slot - 4;
                let p = &mut self.bands[(i / 5) as usize];
                match i % 5 {
                    0 => p.gain_db = v.as_f32().clamp(-15.0, 15.0),
                    1 => p.thr_db = v.as_f32().clamp(-60.0, 0.0),
                    2 => match ratio_of(v) {
                        Some(r) => p.ratio = r,
                        None => return,
                    },
                    3 => p.att_ms = v.as_f32().clamp(0.2, 50.0),
                    _ => p.rel_ms = v.as_f32().clamp(50.0, 4000.0),
                }
                self.dirty = true;
            }
            _ => {}
        }
    }

    fn param(&self, slot: u16) -> Option<f32> {
        match slot {
            1 => Some(self.knee_db),
            2 => Some(self.lmf),
            3 => Some(self.mhf),
            4..=18 => {
                let i = slot - 4;
                let p = &self.bands[(i / 5) as usize];
                Some(match i % 5 {
                    0 => p.gain_db,
                    1 => p.thr_db,
                    2 => p.ratio,
                    3 => p.att_ms,
                    _ => p.rel_ms,
                })
            }
            _ => None,
        }
    }

    fn load_defaults(&mut self) {
        self.knee_db = 5.0;
        self.lmf = 200.0;
        self.mhf = 2000.0;
        self.bands = [BandParams::default(); BANDS];
        self.dirty = true;
        self.xo_dirty = true;
    }

    fn commit(&mut self) {
        self.apply();
    }

    fn telemetry(&self) -> ModelTelemetry {
        let gr = (0..BANDS).map(|b| self.band_gr_db(b)).fold(0.0, f32::max);
        ModelTelemetry {
            gain_reduction_db: gr,
            engaged: gr > 0.01,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f32::consts::TAU;

    const FS: f64 = 48_000.0;

    fn sine(f: f32, amp: f32, n: usize) -> Vec<f32> {
        (0..n)
            .map(|i| amp * (TAU * f * i as f32 / FS as f32).sin())
            .collect()
    }

    fn rms_db(x: &[f32]) -> f32 {
        let s: f64 = x.iter().map(|v| (*v as f64).powi(2)).sum::<f64>() / x.len() as f64;
        (10.0 * s.log10()) as f32
    }

    fn run(m: &mut dyn Model, f: f32, amp: f32, n: usize) -> f32 {
        let mut l = sine(f, amp, n);
        let mut r = l.clone();
        let ref_db = rms_db(&l[n / 2..]);
        for (cl, cr) in l.chunks_mut(64).zip(r.chunks_mut(64)) {
            m.process(cl, cr, None);
        }
        rms_db(&l[n / 2..]) - ref_db
    }

    #[test]
    fn crossover_sum_is_flat() {
        let mut m = ThreeBandComp::new();
        m.prepare(FS, 64);
        for &(lmf, mhf) in &[(200.0, 2000.0), (50.0, 5000.0), (500.0, 1000.0)] {
            m.set_param(2, ParamValue::Float(lmf));
            m.set_param(3, ParamValue::Float(mhf));
            m.commit();
            for &f in &[
                30.0, 50.0, 100.0, 200.0, 350.0, 500.0, 900.0, 1000.0, 2000.0, 3000.0, 5000.0,
                8000.0, 12000.0, 18000.0,
            ] {
                m.reset();
                let d = run(&mut m, f, 0.1, 48_000);
                assert!(d.abs() < 0.05, "lmf {lmf} mhf {mhf} f {f}: {d} dB");
            }
        }
    }

    #[test]
    fn per_band_gr_and_gain() {
        let mut m = ThreeBandComp::new();
        m.prepare(FS, 64);
        // Compress band 1 only: -30 dB, 10:1.
        m.set_param(5, ParamValue::Float(-30.0));
        m.set_param(6, ParamValue::Float(10.0));
        m.set_param(1, ParamValue::Float(0.0));
        m.commit();
        let d = run(&mut m, 80.0, 0.5, 48_000);
        assert!(m.band_gr_db(0) > 10.0, "low GR {}", m.band_gr_db(0));
        assert!(m.band_gr_db(1) < 0.01 && m.band_gr_db(2) < 0.01);
        assert!(d < -10.0, "low tone attenuated {d}");
        m.reset();
        let d = run(&mut m, 8000.0, 0.5, 48_000);
        assert!(d.abs() < 0.1, "high tone untouched {d}");
        assert!(m.band_gr_db(0) < 0.01);
        // Compress band 3 only.
        let mut m = ThreeBandComp::new();
        m.prepare(FS, 64);
        m.set_param(15, ParamValue::Float(-30.0));
        m.set_param(16, ParamValue::Float(10.0));
        m.commit();
        run(&mut m, 8000.0, 0.5, 48_000);
        assert!(m.band_gr_db(2) > 10.0 && m.band_gr_db(0) < 0.01 && m.band_gr_db(1) < 0.01);
        // Band gain: +6 dB on the mid band.
        let mut m = ThreeBandComp::new();
        m.prepare(FS, 64);
        m.set_param(9, ParamValue::Float(6.0));
        m.commit();
        m.reset();
        let d = run(&mut m, 800.0, 0.1, 48_000);
        assert!((d - 6.0).abs() < 0.3, "mid +6 dB: {d}");
    }

    #[test]
    fn ratio_enum_and_params_roundtrip() {
        let mut m = ThreeBandComp::new();
        m.set_param(6, ParamValue::Enum(9));
        assert_eq!(m.param(6), Some(4.0));
        m.set_param(6, ParamValue::Float(8.0));
        assert_eq!(m.param(6), Some(8.0));
        m.set_param(18, ParamValue::Float(99999.0));
        assert_eq!(m.param(18), Some(4000.0));
        assert_eq!(m.param(19), None);
        m.load_defaults();
        assert_eq!(m.param(6), Some(3.0));
        assert_eq!(m.latency(), 0);
    }

    #[test]
    fn onec_mapping_monotonic() {
        let lo = onec_map(0.0, true);
        assert_eq!(lo.thr_db, 0.0);
        assert!((lo.ratio - 1.5).abs() < 1e-4);
        assert!(lo.makeup_db < 1.0, "{}", lo.makeup_db);
        let hi = onec_map(10.0, true);
        assert!((hi.thr_db + 30.0).abs() < 1e-4 && (hi.ratio - 8.0).abs() < 1e-3);
        assert!((hi.att_ms - 3.0).abs() < 1e-3 && (hi.rel_ms - 80.0).abs() < 1e-2);
        assert!(hi.makeup_db > 5.0 && hi.makeup_db <= 12.0);
        assert_eq!(onec_map(10.0, false).makeup_db, 0.0);
        let mut prev = onec_map(0.0, true);
        for k in 1..=100 {
            let m = onec_map(k as f32 * 0.1, true);
            assert!(m.thr_db <= prev.thr_db && m.ratio >= prev.ratio);
            assert!(m.att_ms <= prev.att_ms && m.rel_ms <= prev.rel_ms);
            assert!(m.makeup_db >= prev.makeup_db - 1e-4);
            prev = m;
        }
    }

    #[test]
    fn onec_gr_grows_with_knob() {
        let mut grs = Vec::new();
        for gr in [0.0f32, 3.0, 6.0, 10.0] {
            let mut m = OneKnobComp::new();
            m.prepare(FS, 64);
            m.set_param(1, ParamValue::Float(gr));
            m.set_param(2, ParamValue::Int(0));
            m.commit();
            run(&mut m, 1000.0, 0.5, 48_000);
            grs.push(m.telemetry().gain_reduction_db);
        }
        assert!(grs[0] < 0.5, "{grs:?}");
        assert!(grs.windows(2).all(|w| w[1] > w[0]), "{grs:?}");
        assert!(grs[3] > 15.0, "{grs:?}");
    }

    #[test]
    fn onec_dag_makeup() {
        let mut off = OneKnobComp::new();
        let mut on = OneKnobComp::new();
        for (m, dag) in [(&mut off, 0), (&mut on, 1)] {
            m.prepare(FS, 64);
            m.set_param(1, ParamValue::Float(8.0));
            m.set_param(2, ParamValue::Int(dag));
            m.commit();
        }
        let a = run(&mut off, 1000.0, 0.5, 96_000);
        let b = run(&mut on, 1000.0, 0.5, 96_000);
        let mk = onec_map(8.0, true).makeup_db;
        assert!((b - a - mk).abs() < 0.3, "{a} {b} {mk}");
    }
}
