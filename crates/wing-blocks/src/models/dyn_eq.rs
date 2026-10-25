//! Dynamic EQs and de-esser (T048): `DEQ` (DYNAMIC EQ), `DEQ2` (DUAL DYNAMIC EQ) and `DS902` (BDX902 DEESSER,
//! dbx 902). Parameters, ranges and defaults: `docs/research/models.md` §3.1 and `data/schema/wing-tree.json`
//! (slot order, enum tables, defaults). Assumptions are listed in `docs/VERIFY-ON-HARDWARE.md`.
//!
//! # DEQ / DEQ2
//! One (DEQ) or two series (DEQ2) dynamic bands. Each band:
//! ```text
//! key -> side-chain filter (WING key-filter shapes at f/Q) -> mean-square detector -> level dB
//!     -> gain computer (thr, ratio, mode above/below, |g| = maximum dynamic gain) -> attack/release (dB)
//!     -> dynamic gain -> exact WING band section at that gain (recomputed every 8 samples,
//!        coefficients blended per sample) -> out
//! ```
//! - **Band sections** (`filt`) come from [`crate::wingcoef`], the float32 port of WING-Edit's own coefficient
//!   code (T03C), called exactly as the STD EQ calls it (`eq.rs::wing_band`); DEQ is assumed to share it:
//!   - `BP` = [`wingcoef::peq`] (`hqpeq`, Orfanidis peak, Q on the un-boosted side, >1 kHz behaviour incl.);
//!   - `LP12` = [`wingcoef::ls12`] at Q 0.707 with the STD low-shelf frequency handling (`trunc`, <= 8 kHz);
//!   - `HP12` = [`wingcoef::hqhs12`] with the STD high-shelf handling (`trunc`, >= 50 Hz);
//!   - `LP6` / `HP6` = [`wingcoef::hqls6`] / [`wingcoef::hqhs6`] (WING's first-order shelves; their use in
//!     DEQ is an assumption);
//!   - `OFF` = wide band (gain on the whole signal, detector unfiltered).
//!
//!   A band within [`IDENTITY_DB`] of 0 dB is bypassed (identity section), like the STD EQ's 0 dB bands.
//! - **Side-chain filter**: `BP` -> [`wingcoef::bp`] (`hqbp`, unity-peak band-pass at f/Q), `LP12` ->
//!   [`wingcoef::lp12`] (Q 0.707), `LP6` -> [`wingcoef::lp6`], `HP12` -> [`wingcoef::hp12`], `HP6` ->
//!   [`wingcoef::hp6`]. Assumption: the band listens where it acts.
//! - **Gain computer** (assumption; WING-Edit has no static curve for DEQ, reference-curves.md §5):
//!   `over = level - thr` (mode `high`) or `thr - level` (mode `low`), dynamic gain magnitude
//!   `min(|g|, max(0, over) * (1 - 1/ratio))` with the sign of `g`. So `g < 0, high` is a band compressor,
//!   `g > 0, low` an upward band compressor, `g > 0, high` an upward expander, `g < 0, low` a band expander.
//!   `g = 0` (the default) is transparent.
//! - **Detector**: stereo-linked (louder channel) mean-square of the filtered key, one-pole
//!   [`DEQ_DET_TAU_S`]; the threshold compares to the RMS level (a sine of peak 0 dBFS reads -3 dB).
//! - **Ballistics**: dB-domain one-poles at the control rate, `tau = att` while the gain moves away from 0 dB
//!   and `tau = rel` while it returns (WING COMP convention `att_c = 8T*1000/att`, reference-curves.md §4).
//! - **Control rate**: gain computer, parameter glides and the WING section design run every [`CTRL_BLOCK`] = 8
//!   samples (the WING dynamics control path runs once per 8 samples, reference-curves.md §4); the five
//!   coefficients are interpolated linearly per sample in between.
//! - **Structure**: f64 direct form I. DF1 keeps only input/output history, so a coefficient change never
//!   reinterprets internal state, and a linear blend of two stable sections stays inside the (convex)
//!   stability triangle, so per-sample gain modulation is click-, zipper- and blow-up-free (tested).
//! - **Listen** ([`DynEq::set_listen`], not a schema parameter; WING solos the key via the console): the
//!   output is the filtered side-chain signal of the chosen band.
//!
//! # DS902 (dbx 902 de-esser)
//! ```text
//! key -> HF detector (WING hp12 at f) and wide-band detector -> HF/total energy ratio (dB)
//!     -> reduction = clamp((ratio_db - DS902_ONSET_DB) * DS902_SLOPE, 0, range) -> attack/release (dB)
//!     -> FULL: wide-band gain | HF: dynamic WING high shelf (hqhs12) at f
//! ```
//! The dbx 902 compares the HF content with the total programme level, so de-essing depends on the spectral
//! balance, not on the absolute level (no threshold control; "range" caps the reduction). Onset, slope and
//! times are assumptions (dbx publishes none for the detector law).
//!
//! Real-time: no allocation; `set_param` stages values, `commit`/`process`/`reset` apply them. Per control
//! block: one or two `wingcoef` designs (a few `exp`/`tan`); per sample: two DF1 biquads per channel.

use crate::model::{Model, ModelTelemetry, ParamValue};
use crate::wingcoef::{self, Bq, Ctx};
use core::f32::consts::FRAC_1_SQRT_2;
use wing_dsp::smooth::one_pole_coeff;
use wing_dsp::Complex;

/// Control block length (samples): "the control path runs once per 8 samples" (reference-curves.md §4,
/// `comp_parchg_xseries_comp` / `gate_parchg_xseries_gate`).
pub const CTRL_BLOCK: usize = 8;
/// `ratio` table of DEQ / DEQ2 (schema fenum).
pub const DEQ_RATIOS: [f32; 7] = [1.2, 1.3, 1.5, 2.0, 3.0, 5.0, 10.0];
/// DEQ detector mean-square time constant. Assumption (WING publishes none; 5 ms keeps the level ripple of a
/// 100 Hz tone below 0.4 dB while staying fast against the 0 ms minimum attack).
pub const DEQ_DET_TAU_S: f64 = 0.005;
/// Smoothing of user changes to band frequency and Q (log domain), as the STD EQ (`eq.rs`, 20 ms).
pub const PARAM_TAU_S: f64 = 0.02;
/// Dynamic gains closer to 0 dB than this use the identity section (as the STD EQ bypasses 0 dB bands).
pub const IDENTITY_DB: f32 = 1e-3;
/// DS902: HF/total energy ratio (dB) at which de-essing starts. Assumption: plain speech has < -20 dB of its
/// energy above 4-5 kHz, sibilants are dominated by it (ratio near 0 dB).
pub const DS902_ONSET_DB: f64 = -12.0;
/// DS902: dB of reduction per dB of HF ratio above the onset (before the `range` cap). Assumption.
pub const DS902_SLOPE: f64 = 1.0;
/// DS902 detector mean-square time constant. Assumption (fast enough for sibilant onsets).
pub const DS902_DET_TAU_S: f64 = 0.001;
/// DS902 attack / release (dB-domain one-poles). Assumptions (dbx 902: fixed, programme-independent times).
pub const DS902_ATTACK_S: f64 = 0.001;
pub const DS902_RELEASE_S: f64 = 0.05;
/// Total-power floor of the DS902 ratio (-120 dBFS) so silence never reads as "all HF".
const DS902_POWER_FLOOR: f64 = 1e-12;
/// Detector power floor (-240 dB).
const POWER_FLOOR: f64 = 1e-24;
/// Filter states below this are flushed (denormal guard).
const DENORMAL: f64 = 1e-30;

// ---------------------------------------------------------------------------------------------------------
// Band design (exact WING sections)
// ---------------------------------------------------------------------------------------------------------

/// Band shape (`filt` enum, schema order).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum DeqFilter {
    /// Wide band: gain applies to the whole signal, detector unfiltered.
    Off,
    /// Peaking band (`hqpeq`).
    #[default]
    Bp,
    /// First-order low shelf (`hqls6`).
    Lp6,
    /// Second-order low shelf (`ls12`, STD low SHV).
    Lp12,
    /// First-order high shelf (`hqhs6`).
    Hp6,
    /// Second-order high shelf (`hqhs12`, STD high SHV).
    Hp12,
}

impl DeqFilter {
    pub const ALL: [DeqFilter; 6] = [
        DeqFilter::Off,
        DeqFilter::Bp,
        DeqFilter::Lp6,
        DeqFilter::Lp12,
        DeqFilter::Hp6,
        DeqFilter::Hp12,
    ];

    /// Shape for a schema enum index (out of range clamps to the last entry).
    pub fn from_index(i: usize) -> Self {
        Self::ALL[i.min(Self::ALL.len() - 1)]
    }

    /// Schema token.
    pub fn key(self) -> &'static str {
        match self {
            DeqFilter::Off => "OFF",
            DeqFilter::Bp => "BP",
            DeqFilter::Lp6 => "LP6",
            DeqFilter::Lp12 => "LP12",
            DeqFilter::Hp6 => "HP6",
            DeqFilter::Hp12 => "HP12",
        }
    }
}

/// The WING section of a band of shape `filt` at frequency `f` (Hz), Q `q` and gain `gain_db`, called the
/// way `eq.rs::wing_band` calls `wingcoef` for the STD EQ.
pub fn band_section(filt: DeqFilter, c: &Ctx, f: f32, q: f32, gain_db: f32) -> Bq {
    if filt == DeqFilter::Off {
        return [wingcoef::db2lin(gain_db), 0.0, 0.0, 0.0, 0.0];
    }
    if gain_db.abs() < IDENTITY_DB {
        return wingcoef::IDENTITY;
    }
    match filt {
        DeqFilter::Bp => wingcoef::peq(c, f, gain_db, q),
        DeqFilter::Lp12 => {
            let f = if f > 8000.0 { 8000.0 } else { f.trunc() };
            wingcoef::ls12(c, f, gain_db, FRAC_1_SQRT_2)
        }
        DeqFilter::Hp12 => {
            let f = if f < 50.0 { 50.0 } else { f.trunc() };
            wingcoef::hqhs12(c, f, gain_db)
        }
        DeqFilter::Lp6 => wingcoef::hqls6(c, f, gain_db),
        DeqFilter::Hp6 => wingcoef::hqhs6(c, f, gain_db),
        DeqFilter::Off => unreachable!(),
    }
}

/// Side-chain (detector) section of a band shape.
pub fn sidechain_section(filt: DeqFilter, c: &Ctx, f: f32, q: f32) -> Bq {
    match filt {
        DeqFilter::Off => wingcoef::IDENTITY,
        DeqFilter::Bp => wingcoef::bp(c, f, q),
        DeqFilter::Lp12 => wingcoef::lp12(c, f, FRAC_1_SQRT_2),
        DeqFilter::Lp6 => wingcoef::lp6(c, f),
        DeqFilter::Hp12 => wingcoef::hp12(c, f),
        DeqFilter::Hp6 => wingcoef::hp6(c, f),
    }
}

/// Exact frequency response of a band section at `at_hz`.
pub fn band_response(
    filt: DeqFilter,
    fs: f64,
    f: f64,
    q: f64,
    gain_db: f64,
    at_hz: f64,
) -> Complex {
    let c = Ctx::new(fs);
    let s = band_section(filt, &c, f as f32, q as f32, gain_db as f32);
    wingcoef::response(fs, &[s], at_hz)
}

// ---------------------------------------------------------------------------------------------------------
// Direct form I
// ---------------------------------------------------------------------------------------------------------

type Coef = [f64; 5];

#[inline]
fn coef(b: &Bq) -> Coef {
    [
        b[0] as f64,
        b[1] as f64,
        b[2] as f64,
        b[3] as f64,
        b[4] as f64,
    ]
}

/// DF1 tick; `s` = [x1, x2, y1, y2].
#[inline(always)]
fn df1(c: &Coef, s: &mut [f64; 4], x: f64) -> f64 {
    let y = c[0] * x + c[1] * s[0] + c[2] * s[1] - c[3] * s[2] - c[4] * s[3];
    s[1] = s[0];
    s[0] = x;
    s[3] = s[2];
    s[2] = y;
    y
}

#[inline]
fn flush(s: &mut [[f64; 4]; 2]) {
    for v in s.iter_mut().flatten() {
        if v.abs() < DENORMAL {
            *v = 0.0;
        }
    }
}

#[inline]
fn step_to(cur: &Coef, target: &Coef) -> Coef {
    let n = CTRL_BLOCK as f64;
    core::array::from_fn(|i| (target[i] - cur[i]) / n)
}

// ---------------------------------------------------------------------------------------------------------
// Dynamic band
// ---------------------------------------------------------------------------------------------------------

/// Parameters of one dynamic band (engineering units).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DeqBandParams {
    pub thr_db: f32,
    pub ratio: f32,
    pub att_ms: f32,
    pub rel_ms: f32,
    pub filt: DeqFilter,
    /// Maximum dynamic gain (sign = boost/cut).
    pub gain_db: f32,
    pub freq: f32,
    pub q: f32,
    /// Mode `high` (act above the threshold) vs `low` (below).
    pub above: bool,
}

impl DeqBandParams {
    /// WING-Edit defaults with band frequency `freq`.
    pub const fn defaults(freq: f32) -> Self {
        Self {
            thr_db: -20.0,
            ratio: 2.0,
            att_ms: 50.0,
            rel_ms: 301.0,
            filt: DeqFilter::Bp,
            gain_db: 0.0,
            freq,
            q: 1.0,
            above: true,
        }
    }

    /// Static dynamic gain (dB) for a detected RMS level (dB), before ballistics.
    pub fn static_gain_db(&self, level_db: f32) -> f32 {
        let over = if self.above {
            level_db - self.thr_db
        } else {
            self.thr_db - level_db
        };
        let amt = (over.max(0.0) * (1.0 - 1.0 / self.ratio)).min(self.gain_db.abs());
        amt.copysign(self.gain_db)
    }
}

/// One dynamic EQ band (stereo-linked), the DSP core of DEQ / DEQ2.
#[derive(Clone, Debug)]
pub struct DeqBand {
    ctx: Ctx,
    p: DeqBandParams,
    // derived at set_params()
    att_c: f64,
    rel_c: f64,
    param_c: f64,
    det_c: f64,
    lf_target: f64,
    lq_target: f64,
    // control state
    lf: f64,
    lq: f64,
    det: Coef,
    env: f64,
    dyn_db: f64,
    phase: usize,
    cur: Coef,
    step: Coef,
    // per-channel state
    det_s: [[f64; 4]; 2],
    st: [[f64; 4]; 2],
    ms: [f64; 2],
    listen: bool,
}

impl DeqBand {
    pub fn new(p: DeqBandParams) -> Self {
        let mut b = Self {
            ctx: Ctx::new(48_000.0),
            p,
            att_c: 0.0,
            rel_c: 0.0,
            param_c: 0.0,
            det_c: 0.0,
            lf_target: 0.0,
            lq_target: 0.0,
            lf: 0.0,
            lq: 0.0,
            det: coef(&wingcoef::IDENTITY),
            env: 0.0,
            dyn_db: 0.0,
            phase: 0,
            cur: coef(&wingcoef::IDENTITY),
            step: [0.0; 5],
            det_s: [[0.0; 4]; 2],
            st: [[0.0; 4]; 2],
            ms: [POWER_FLOOR; 2],
            listen: false,
        };
        b.prepare(48_000.0);
        b
    }

    pub fn prepare(&mut self, fs: f64) {
        self.ctx = Ctx::new(fs);
        self.set_params(self.p);
        self.reset();
    }

    pub fn params(&self) -> &DeqBandParams {
        &self.p
    }

    /// Apply new parameters (RT-safe: a few `exp`/`log2`). Frequency and Q glide ([`PARAM_TAU_S`]); the
    /// rest takes effect at the next control block.
    pub fn set_params(&mut self, p: DeqBandParams) {
        let fs = self.ctx.fs;
        let ctrl_fs = fs / CTRL_BLOCK as f64;
        self.p = DeqBandParams {
            thr_db: p.thr_db.clamp(-60.0, 0.0),
            ratio: p.ratio.clamp(1.0, 100.0),
            att_ms: p.att_ms.clamp(0.0, 200.0),
            rel_ms: p.rel_ms.clamp(20.0, 4000.0),
            gain_db: p.gain_db.clamp(-15.0, 15.0),
            freq: p.freq.clamp(20.0, 20_000.0),
            q: p.q.clamp(0.442, 10.0),
            ..p
        };
        self.att_c = one_pole_coeff(self.p.att_ms as f64 * 1e-3, ctrl_fs);
        self.rel_c = one_pole_coeff(self.p.rel_ms as f64 * 1e-3, ctrl_fs);
        self.param_c = one_pole_coeff(PARAM_TAU_S, ctrl_fs);
        self.det_c = one_pole_coeff(DEQ_DET_TAU_S, fs);
        self.lf_target = (self.p.freq as f64).log2();
        self.lq_target = (self.p.q as f64).log2();
    }

    /// Output the filtered side-chain instead of the processed signal.
    pub fn set_listen(&mut self, on: bool) {
        self.listen = on;
    }

    /// Clear filter/detector state, settle the parameter glides and the gain envelope.
    pub fn reset(&mut self) {
        self.lf = self.lf_target;
        self.lq = self.lq_target;
        self.env = 0.0;
        self.dyn_db = 0.0;
        self.phase = 0;
        self.det_s = [[0.0; 4]; 2];
        self.st = [[0.0; 4]; 2];
        self.ms = [POWER_FLOOR; 2];
        let (f, q) = (self.lf.exp2() as f32, self.lq.exp2() as f32);
        self.det = coef(&sidechain_section(self.p.filt, &self.ctx, f, q));
        self.cur = coef(&band_section(self.p.filt, &self.ctx, f, q, 0.0));
        self.step = [0.0; 5];
    }

    /// Current dynamic gain (dB, signed).
    pub fn dynamic_gain_db(&self) -> f32 {
        self.dyn_db as f32
    }

    /// Control-rate update: parameter glides, gain computer, ballistics, coefficient targets.
    fn control(&mut self) {
        self.lf = self.lf_target + self.param_c * (self.lf - self.lf_target);
        self.lq = self.lq_target + self.param_c * (self.lq - self.lq_target);
        let (f, q) = (self.lf.exp2() as f32, self.lq.exp2() as f32);
        self.det = coef(&sidechain_section(self.p.filt, &self.ctx, f, q));

        let level = 10.0 * self.ms[0].max(self.ms[1]).max(POWER_FLOOR).log10();
        let amt = self.p.static_gain_db(level as f32).abs() as f64;
        let c = if amt > self.env {
            self.att_c
        } else {
            self.rel_c
        };
        self.env = amt + c * (self.env - amt);
        self.dyn_db = self.env.copysign(self.p.gain_db as f64);

        let target = coef(&band_section(
            self.p.filt,
            &self.ctx,
            f,
            q,
            self.dyn_db as f32,
        ));
        self.step = step_to(&self.cur, &target);
        flush(&mut self.st);
        flush(&mut self.det_s);
    }

    /// Process a stereo block in place; `key` = external side-chain (else the input).
    pub fn process(&mut self, left: &mut [f32], right: &mut [f32], key: Option<(&[f32], &[f32])>) {
        let n = left.len().min(right.len());
        for i in 0..n {
            if self.phase == 0 {
                self.control();
            }
            self.phase += 1;
            if self.phase == CTRL_BLOCK {
                self.phase = 0;
            }
            let (xl, xr) = (left[i] as f64, right[i] as f64);
            let (kl, kr) = match key {
                Some((a, b)) => (
                    a.get(i).copied().unwrap_or(0.0) as f64,
                    b.get(i).copied().unwrap_or(0.0) as f64,
                ),
                None => (xl, xr),
            };
            let dl = df1(&self.det, &mut self.det_s[0], kl);
            let dr = df1(&self.det, &mut self.det_s[1], kr);
            let c = self.det_c;
            self.ms[0] = dl * dl + c * (self.ms[0] - dl * dl);
            self.ms[1] = dr * dr + c * (self.ms[1] - dr * dr);

            for (v, d) in self.cur.iter_mut().zip(&self.step) {
                *v += d;
            }
            let yl = df1(&self.cur, &mut self.st[0], xl);
            let yr = df1(&self.cur, &mut self.st[1], xr);
            if self.listen {
                left[i] = dl as f32;
                right[i] = dr as f32;
            } else {
                left[i] = yl as f32;
                right[i] = yr as f32;
            }
        }
    }
}

// ---------------------------------------------------------------------------------------------------------
// DEQ / DEQ2 models
// ---------------------------------------------------------------------------------------------------------

#[derive(Clone, Copy, Debug)]
enum SlotKind {
    Range(f32, f32),
    /// `DEQ_RATIOS` (Enum = index, Float snaps to the nearest entry).
    Ratio,
    /// Plain enum stored as its index `0..n`.
    Index(u32),
}

/// Per-band slot order (schema): `thr ratio att rel filt g f q mode`.
const BAND_SLOTS: [SlotKind; 9] = [
    SlotKind::Range(-60.0, 0.0),
    SlotKind::Ratio,
    SlotKind::Range(0.0, 200.0),
    SlotKind::Range(20.0, 4000.0),
    SlotKind::Index(6),
    SlotKind::Range(-15.0, 15.0),
    SlotKind::Range(20.0, 20_000.0),
    SlotKind::Range(0.442, 10.0),
    SlotKind::Index(2),
];
/// Schema band frequency defaults: DEQ `f` 1k00; DEQ2 `1-f` 200.0, `2-f` 2k99.
pub const DEQ_FREQ_DEFAULT: f32 = 1000.0;
pub const DEQ2_FREQ_DEFAULTS: [f32; 2] = [200.0, 2990.0];
/// DEQ2 slot 19: `$page` (UI page selector, stored but not used by the DSP).
const PAGE_SLOT: u16 = 19;

fn snap_ratio(v: f32) -> f32 {
    let mut best = DEQ_RATIOS[0];
    for &r in &DEQ_RATIOS {
        if (r - v).abs() < (best - v).abs() {
            best = r;
        }
    }
    best
}

fn band_values(p: &DeqBandParams) -> [f32; 9] {
    [
        p.thr_db,
        p.ratio,
        p.att_ms,
        p.rel_ms,
        DeqFilter::ALL
            .iter()
            .position(|&f| f == p.filt)
            .unwrap_or(1) as f32,
        p.gain_db,
        p.freq,
        p.q,
        p.above as u32 as f32,
    ]
}

fn band_from_values(v: &[f32]) -> DeqBandParams {
    DeqBandParams {
        thr_db: v[0],
        ratio: v[1],
        att_ms: v[2],
        rel_ms: v[3],
        filt: DeqFilter::from_index(v[4] as usize),
        gain_db: v[5],
        freq: v[6],
        q: v[7],
        above: v[8] != 0.0,
    }
}

/// `DEQ` (one band) or `DEQ2` (two bands in series) dynamic EQ model.
pub struct DynEq {
    dual: bool,
    /// Staged slot values (19 = DEQ2 `$page`).
    v: [f32; 19],
    dirty: bool,
    bands: [DeqBand; 2],
    listen: Option<usize>,
}

impl DynEq {
    /// `DEQ`.
    pub fn single() -> Self {
        Self::new(false)
    }

    /// `DEQ2`.
    pub fn dual() -> Self {
        Self::new(true)
    }

    fn new(dual: bool) -> Self {
        let f = if dual {
            DEQ2_FREQ_DEFAULTS
        } else {
            [DEQ_FREQ_DEFAULT; 2]
        };
        let mut m = Self {
            dual,
            v: [0.0; 19],
            dirty: true,
            bands: [
                DeqBand::new(DeqBandParams::defaults(f[0])),
                DeqBand::new(DeqBandParams::defaults(f[1])),
            ],
            listen: None,
        };
        m.load_defaults();
        m.apply();
        m
    }

    pub fn band_count(&self) -> usize {
        if self.dual {
            2
        } else {
            1
        }
    }

    /// Band `i` (applies staged parameters first).
    pub fn band(&mut self, i: usize) -> &DeqBand {
        self.apply();
        &self.bands[i.min(self.band_count() - 1)]
    }

    /// Listen to band `i`'s side-chain (`None` = normal output). Not a WING snapshot parameter.
    pub fn set_listen(&mut self, band: Option<usize>) {
        self.listen = band.filter(|&b| b < self.band_count());
        for (i, b) in self.bands.iter_mut().enumerate() {
            b.set_listen(self.listen == Some(i));
        }
    }

    fn slot_count(&self) -> usize {
        if self.dual {
            19
        } else {
            9
        }
    }

    fn apply(&mut self) {
        if !self.dirty {
            return;
        }
        self.dirty = false;
        for i in 0..self.band_count() {
            let p = band_from_values(&self.v[9 * i..9 * i + 9]);
            self.bands[i].set_params(p);
        }
    }
}

impl Model for DynEq {
    fn id(&self) -> &'static str {
        if self.dual {
            "DEQ2"
        } else {
            "DEQ"
        }
    }

    fn prepare(&mut self, sample_rate: f64, _max_block: usize) {
        self.apply();
        for b in &mut self.bands {
            b.prepare(sample_rate);
        }
    }

    fn process(
        &mut self,
        left: &mut [f32],
        right: &mut [f32],
        sidechain: Option<(&[f32], &[f32])>,
    ) {
        self.apply();
        match self.listen {
            Some(0) => self.bands[0].process(left, right, sidechain),
            _ => {
                for i in 0..self.band_count() {
                    self.bands[i].process(left, right, sidechain);
                }
            }
        }
    }

    fn reset(&mut self) {
        self.apply();
        for b in &mut self.bands {
            b.reset();
        }
    }

    fn set_param(&mut self, slot: u16, value: ParamValue) {
        let Some(i) = (slot as usize).checked_sub(1) else {
            return;
        };
        if i >= self.slot_count() {
            return;
        }
        if self.dual && slot == PAGE_SLOT {
            self.v[i] = value.as_bool() as u32 as f32;
            return;
        }
        let x = match BAND_SLOTS[i % 9] {
            SlotKind::Range(lo, hi) => value.as_f32().clamp(lo, hi),
            SlotKind::Ratio => match value {
                ParamValue::Enum(e) => match DEQ_RATIOS.get(e as usize) {
                    Some(&r) => r,
                    None => return,
                },
                other => snap_ratio(other.as_f32()),
            },
            SlotKind::Index(n) => value.as_i32().clamp(0, n as i32 - 1) as f32,
        };
        self.v[i] = x;
        self.dirty = true;
    }

    fn param(&self, slot: u16) -> Option<f32> {
        let i = (slot as usize).checked_sub(1)?;
        (i < self.slot_count()).then(|| self.v[i])
    }

    fn load_defaults(&mut self) {
        let f = if self.dual {
            DEQ2_FREQ_DEFAULTS
        } else {
            [DEQ_FREQ_DEFAULT; 2]
        };
        for (b, &freq) in f.iter().enumerate() {
            self.v[9 * b..9 * b + 9].copy_from_slice(&band_values(&DeqBandParams::defaults(freq)));
        }
        self.v[18] = 0.0;
        self.dirty = true;
    }

    fn commit(&mut self) {
        self.apply();
    }

    fn telemetry(&self) -> ModelTelemetry {
        let g = self.bands[..self.band_count()]
            .iter()
            .map(|b| b.dyn_db.abs() as f32)
            .fold(0.0, f32::max);
        ModelTelemetry {
            gain_reduction_db: g,
            engaged: g > 0.01,
        }
    }
}

// ---------------------------------------------------------------------------------------------------------
// DS902
// ---------------------------------------------------------------------------------------------------------

/// DS902 slot defaults (`f range mode`): 5k0, 4.0 dB, HF.
const DS902_DEFAULTS: [f32; 3] = [5000.0, 4.0, 1.0];

/// `DS902` BDX902 DEESSER (dbx 902), stereo-linked.
pub struct DeEsser902 {
    ctx: Ctx,
    /// Staged `f`, `range`, `mode` (0 = FULL, 1 = HF).
    v: [f32; 3],
    dirty: bool,
    hf_mode: bool,
    lf_target: f64,
    lf: f64,
    param_c: f64,
    det_c: f64,
    att_c: f64,
    rel_c: f64,
    /// Detector: WING `hqhp12` at f.
    det: Coef,
    phase: usize,
    env: f64,
    /// Audio section (WING `hqhs12` cut in HF mode, wide-band gain in FULL), blended per sample.
    cur: Coef,
    step: Coef,
    hf_ms: [f64; 2],
    full_ms: [f64; 2],
    det_s: [[f64; 4]; 2],
    st: [[f64; 4]; 2],
}

impl Default for DeEsser902 {
    fn default() -> Self {
        Self::new()
    }
}

impl DeEsser902 {
    pub fn new() -> Self {
        let mut m = Self {
            ctx: Ctx::new(48_000.0),
            v: DS902_DEFAULTS,
            dirty: true,
            hf_mode: true,
            lf_target: 0.0,
            lf: 0.0,
            param_c: 0.0,
            det_c: 0.0,
            att_c: 0.0,
            rel_c: 0.0,
            det: coef(&wingcoef::IDENTITY),
            phase: 0,
            env: 0.0,
            cur: coef(&wingcoef::IDENTITY),
            step: [0.0; 5],
            hf_ms: [0.0; 2],
            full_ms: [0.0; 2],
            det_s: [[0.0; 4]; 2],
            st: [[0.0; 4]; 2],
        };
        m.apply();
        m.reset();
        m
    }

    /// Current reduction (dB, >= 0).
    pub fn reduction_db(&self) -> f32 {
        self.env as f32
    }

    /// Static reduction (dB) for an HF/total energy ratio (dB), before ballistics.
    pub fn static_reduction_db(&self, hf_ratio_db: f64) -> f64 {
        ((hf_ratio_db - DS902_ONSET_DB) * DS902_SLOPE).clamp(0.0, self.v[1] as f64)
    }

    fn apply(&mut self) {
        if !self.dirty {
            return;
        }
        self.dirty = false;
        let fs = self.ctx.fs;
        let ctrl_fs = fs / CTRL_BLOCK as f64;
        self.lf_target = (self.v[0] as f64).log2();
        self.hf_mode = self.v[2] != 0.0;
        self.param_c = one_pole_coeff(PARAM_TAU_S, ctrl_fs);
        self.det_c = one_pole_coeff(DS902_DET_TAU_S, fs);
        self.att_c = one_pole_coeff(DS902_ATTACK_S, ctrl_fs);
        self.rel_c = one_pole_coeff(DS902_RELEASE_S, ctrl_fs);
    }

    /// Audio section at `reduction_db`.
    fn audio_section(&self, reduction_db: f64) -> Coef {
        let shape = if self.hf_mode {
            DeqFilter::Hp12
        } else {
            DeqFilter::Off
        };
        let f = self.lf.exp2() as f32;
        coef(&band_section(
            shape,
            &self.ctx,
            f,
            1.0,
            -reduction_db as f32,
        ))
    }

    fn control(&mut self) {
        self.lf = self.lf_target + self.param_c * (self.lf - self.lf_target);
        self.det = coef(&wingcoef::hp12(&self.ctx, self.lf.exp2() as f32));
        let hf = self.hf_ms[0] + self.hf_ms[1];
        let full = self.full_ms[0] + self.full_ms[1] + DS902_POWER_FLOOR;
        let ratio_db = 10.0 * (hf / full).max(POWER_FLOOR).log10();
        let target = self.static_reduction_db(ratio_db);
        let c = if target > self.env {
            self.att_c
        } else {
            self.rel_c
        };
        self.env = target + c * (self.env - target);
        let t = self.audio_section(self.env);
        self.step = step_to(&self.cur, &t);
        flush(&mut self.det_s);
        flush(&mut self.st);
    }
}

impl Model for DeEsser902 {
    fn id(&self) -> &'static str {
        "DS902"
    }

    fn prepare(&mut self, sample_rate: f64, _max_block: usize) {
        self.ctx = Ctx::new(sample_rate);
        self.dirty = true;
        self.apply();
        self.reset();
    }

    fn process(
        &mut self,
        left: &mut [f32],
        right: &mut [f32],
        sidechain: Option<(&[f32], &[f32])>,
    ) {
        self.apply();
        let n = left.len().min(right.len());
        for i in 0..n {
            if self.phase == 0 {
                self.control();
            }
            self.phase += 1;
            if self.phase == CTRL_BLOCK {
                self.phase = 0;
            }
            let x = [left[i] as f64, right[i] as f64];
            let key = match sidechain {
                Some((a, b)) => [
                    a.get(i).copied().unwrap_or(0.0) as f64,
                    b.get(i).copied().unwrap_or(0.0) as f64,
                ],
                None => x,
            };
            for (v, d) in self.cur.iter_mut().zip(&self.step) {
                *v += d;
            }
            let c = self.det_c;
            let mut y = [0.0; 2];
            for ch in 0..2 {
                let hp = df1(&self.det, &mut self.det_s[ch], key[ch]);
                let (kp, hpp) = (key[ch] * key[ch], hp * hp);
                self.full_ms[ch] = kp + c * (self.full_ms[ch] - kp);
                self.hf_ms[ch] = hpp + c * (self.hf_ms[ch] - hpp);
                y[ch] = df1(&self.cur, &mut self.st[ch], x[ch]);
            }
            left[i] = y[0] as f32;
            right[i] = y[1] as f32;
        }
    }

    fn reset(&mut self) {
        self.apply();
        self.lf = self.lf_target;
        self.det = coef(&wingcoef::hp12(&self.ctx, self.lf.exp2() as f32));
        self.phase = 0;
        self.env = 0.0;
        self.cur = self.audio_section(0.0);
        self.step = [0.0; 5];
        self.hf_ms = [0.0; 2];
        self.full_ms = [0.0; 2];
        self.det_s = [[0.0; 4]; 2];
        self.st = [[0.0; 4]; 2];
    }

    fn set_param(&mut self, slot: u16, value: ParamValue) {
        let x = match slot {
            1 => value.as_f32().clamp(800.0, 8000.0),
            2 => value.as_f32().clamp(0.0, 12.0),
            3 => value.as_i32().clamp(0, 1) as f32,
            _ => return,
        };
        self.v[slot as usize - 1] = x;
        self.dirty = true;
    }

    fn param(&self, slot: u16) -> Option<f32> {
        let i = (slot as usize).checked_sub(1)?;
        self.v.get(i).copied()
    }

    fn load_defaults(&mut self) {
        self.v = DS902_DEFAULTS;
        self.dirty = true;
    }

    fn commit(&mut self) {
        self.apply();
    }

    fn telemetry(&self) -> ModelTelemetry {
        ModelTelemetry {
            gain_reduction_db: self.env as f32,
            engaged: self.env > 0.01,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fidelity::{cascade_db, FreqCurves};
    use core::f64::consts::{PI, SQRT_2};
    use ParamValue::{Enum, Float};

    const FS: f64 = 48_000.0;

    fn sine(f: f64, amp: f64, n: usize) -> Vec<f32> {
        (0..n)
            .map(|i| (amp * (2.0 * PI * f * i as f64 / FS).sin()) as f32)
            .collect()
    }

    /// Amplitude of the `f` component of `s` (correlation over the slice; use whole periods).
    fn amp_at(s: &[f32], f: f64) -> f64 {
        let w = 2.0 * PI * f / FS;
        let (mut re, mut im) = (0.0, 0.0);
        for (n, &x) in s.iter().enumerate() {
            re += x as f64 * (w * n as f64).cos();
            im += x as f64 * (w * n as f64).sin();
        }
        2.0 * re.hypot(im) / s.len() as f64
    }

    fn db(x: f64) -> f64 {
        20.0 * x.max(1e-30).log10()
    }

    /// Run `x` through `m` (both channels) in 64-sample blocks with an optional mono key.
    fn run(m: &mut dyn Model, x: &[f32], key: Option<&[f32]>) -> Vec<f32> {
        let mut l = x.to_vec();
        let mut r = x.to_vec();
        for (i, (lc, rc)) in l.chunks_mut(64).zip(r.chunks_mut(64)).enumerate() {
            let k = key.map(|k| &k[i * 64..i * 64 + lc.len()]);
            m.process(lc, rc, k.map(|k| (k, k)));
        }
        l
    }

    fn deq(params: &[(u16, ParamValue)]) -> DynEq {
        let mut m = DynEq::single();
        m.prepare(FS, 64);
        for &(s, v) in params {
            m.set_param(s, v);
        }
        m.commit();
        m.reset();
        m
    }

    #[test]
    fn deq_static_band_gain_vs_level_above_and_below() {
        // BP 1 kHz q 1, thr -30, ratio 2, g -12 (cut above threshold).
        let mut m = deq(&[
            (1, Float(-30.0)),
            (2, Enum(3)),
            (3, Float(1.0)),
            (4, Float(20.0)),
            (5, Enum(1)),
            (6, Float(-12.0)),
            (7, Float(1000.0)),
            (8, Float(1.0)),
            (9, Enum(1)),
        ]);
        let n = 24_000;
        for l_rms in [-50.0, -34.0, -30.0, -26.0, -20.0, -10.0, -3.0] {
            let amp = 10f64.powf(l_rms / 20.0) * SQRT_2;
            let x = sine(1000.0, amp, n);
            m.reset();
            let y = run(&mut m, &x, None);
            let g = db(amp_at(&y[n - 480..], 1000.0) / amp);
            let e = m.band(0).params().static_gain_db(l_rms as f32) as f64;
            let e_law = -((l_rms + 30.0) / 2.0).clamp(0.0, 12.0);
            assert!((e - e_law).abs() < 1e-5);
            assert!((g - e).abs() < 0.15, "above, {l_rms} dB: {g:.3} vs {e:.3}");
            assert!((m.telemetry().gain_reduction_db as f64 - e.abs()).abs() < 0.15);
        }
        // Mode low with a boost: upward band compression below the threshold, capped at +6 dB.
        m.set_param(6, Float(6.0));
        m.set_param(9, Enum(0));
        m.commit();
        for l_rms in [-55.0, -40.0, -36.0, -30.0, -20.0] {
            let amp = 10f64.powf(l_rms / 20.0) * SQRT_2;
            let x = sine(1000.0, amp, n);
            m.reset();
            let y = run(&mut m, &x, None);
            let g = db(amp_at(&y[n - 480..], 1000.0) / amp);
            let e = ((-30.0 - l_rms) / 2.0).clamp(0.0, 6.0);
            assert!((g - e).abs() < 0.15, "below, {l_rms} dB: {g:.3} vs {e:.3}");
        }
    }

    #[test]
    fn deq_band_shapes_follow_wing_sections() {
        for filt in DeqFilter::ALL {
            // 0 dB (and anything within IDENTITY_DB) is the identity: a band at rest is transparent.
            for f in [20.0, 250.0, 1000.0, 12_000.0] {
                assert_eq!(
                    band_section(filt, &Ctx::new(FS), f, 2.0, 0.0),
                    if filt == DeqFilter::Off {
                        [1.0, 0.0, 0.0, 0.0, 0.0]
                    } else {
                        wingcoef::IDENTITY
                    }
                );
            }
            for g in [-15.0, -6.0, 6.0, 15.0] {
                let at = |f: f64| band_response(filt, FS, 1000.0, 2.0, g, f).db();
                match filt {
                    DeqFilter::Off => {
                        assert!((at(37.0) - g).abs() < 1e-5 && (at(15e3) - g).abs() < 1e-5)
                    }
                    DeqFilter::Bp => {
                        // hqpeq: exact gain at f.
                        assert!((at(1000.0) - g).abs() < 1e-3, "bp {g}: {}", at(1000.0));
                        assert!(at(20.0).abs() < 0.1, "bp {g}: {}", at(20.0));
                    }
                    DeqFilter::Lp6 | DeqFilter::Lp12 => {
                        assert!((at(5.0) - g).abs() < 0.3, "{filt:?} {g} dc {}", at(5.0));
                        assert!(at(20e3).abs() < 0.5, "{filt:?} {g} hf {}", at(20e3));
                    }
                    DeqFilter::Hp6 | DeqFilter::Hp12 => {
                        assert!((at(20e3) - g).abs() < 0.5, "{filt:?} {g} hf {}", at(20e3));
                        assert!(at(20.0).abs() < 0.3, "{filt:?} {g} lf {}", at(20.0));
                    }
                }
            }
        }
        // Shelves ignore Q (WING STD SHV), the peak does not.
        let a = band_response(DeqFilter::Lp12, FS, 300.0, 0.5, 9.0, 400.0);
        let b = band_response(DeqFilter::Lp12, FS, 300.0, 8.0, 9.0, 400.0);
        assert!((a - b).norm() < 1e-12);
        let a = band_response(DeqFilter::Bp, FS, 300.0, 0.5, 9.0, 400.0);
        let b = band_response(DeqFilter::Bp, FS, 300.0, 8.0, 9.0, 400.0);
        assert!((a - b).norm() > 0.1);
    }

    /// DEQ band sections vs the WING-Edit STD EQ biquads (reference curves, T00A) over every reference
    /// case and frequency: BP = mid PEQ, LP12 = low SHV, HP12 = high SHV.
    #[test]
    fn deq_bands_match_wing_std_reference_biquads() {
        let Some(refc) = FreqCurves::load("eq-std-ch.json").unwrap() else {
            eprintln!("reference curves absent, skipping");
            return;
        };
        let fs = refc.fs;
        let eval: Vec<f64> = refc
            .freq_hz
            .iter()
            .copied()
            .filter(|&f| f < 20_000.0)
            .collect();
        let (mut checked, mut worst) = (0, 0.0f64);
        for c in &refc.curves {
            let (filt, key, idx) = match c.case.as_str() {
                "mid-peq" => (DeqFilter::Bp, "2", 2),
                "l-SHV" => (DeqFilter::Lp12, "l", 0),
                "h-SHV" => (DeqFilter::Hp12, "h", 5),
                _ => continue,
            };
            let num = |s: &str| c.params.num(&format!("{key}{s}"));
            let (Some(g), Some(f), Some(q)) = (num("g"), num("f"), num("q")) else {
                continue;
            };
            let bq = [c.biquads[idx]];
            // Exported params are printed read-backs: as in T03C, accept the gain's +-1 f32 ulp neighbours.
            let g32 = g as f32;
            let ulp = |d: i32| f32::from_bits((g32.to_bits() as i32 + d) as u32) as f64;
            let dev = [ulp(0), ulp(1), ulp(-1)]
                .into_iter()
                .map(|gg| {
                    eval.iter()
                        .map(|&x| {
                            (band_response(filt, fs, f, q, gg, x).db() - cascade_db(&bq, fs, x))
                                .abs()
                        })
                        .fold(0.0, f64::max)
                })
                .fold(f64::INFINITY, f64::min);
            assert!(dev < 0.01, "{} g {g} f {f} q {q}: {dev:.4} dB", c.case);
            worst = worst.max(dev);
            checked += 1;
        }
        assert!(checked > 200, "only {checked} reference cases");
        eprintln!("DEQ vs WING STD biquads: {checked} cases, max {worst:.5} dB");
    }

    /// The running band (detector, ballistics, per-sample blend, DF1) settles exactly on the WING section:
    /// mode low with thr 0 dB holds the band at full `g` on a quiet signal; its impulse response then matches
    /// `band_response`.
    #[test]
    fn deq_runtime_settles_on_the_wing_section() {
        for (fi, f, g) in [
            (1u32, 3000.0, -9.0),
            (3, 120.0, 6.0),
            (5, 6000.0, -12.0),
            (2, 500.0, 9.0),
        ] {
            let filt = DeqFilter::from_index(fi as usize);
            let mut m = deq(&[
                (1, Float(0.0)),
                (2, Enum(6)),
                (3, Float(0.0)),
                (5, Enum(fi)),
                (6, Float(g)),
                (7, Float(f)),
                (8, Float(0.7)),
                (9, Enum(0)),
            ]);
            let mut x = vec![0.0f32; 4800 + 16_384];
            x[4800] = 1e-4;
            let y = run(&mut m, &x, None);
            assert!((m.band(0).dynamic_gain_db() - g).abs() < 1e-4);
            let h: Vec<f64> = y[4800..].iter().map(|&v| v as f64 * 1e4).collect();
            for at in [60.0, 400.0, 1000.0, 2500.0, 6000.0, 15_000.0] {
                let meas = wing_dsp::dtft(&h, 2.0 * PI * at / FS).db();
                let want = band_response(filt, FS, f as f64, 0.7, g as f64, at).db();
                assert!(
                    (meas - want).abs() < 0.01,
                    "{filt:?} at {at}: {meas:.4} vs {want:.4}"
                );
            }
        }
    }

    /// Gain-envelope trajectory (telemetry per control block) for a 1 kHz key that is on for `on_s`.
    fn envelope(m: &mut DynEq, on_s: f64, total_s: f64) -> Vec<f32> {
        let n = (total_s * FS) as usize;
        let n_on = (on_s * FS) as usize;
        let key: Vec<f32> = sine(1000.0, 1.0, n)
            .into_iter()
            .enumerate()
            .map(|(i, v)| if i < n_on { v } else { 0.0 })
            .collect();
        let x = vec![0.0f32; n];
        m.reset();
        let mut out = Vec::new();
        for (xc, kc) in x.chunks(CTRL_BLOCK).zip(key.chunks(CTRL_BLOCK)) {
            let (mut l, mut r) = (xc.to_vec(), xc.to_vec());
            m.process(&mut l, &mut r, Some((kc, kc)));
            out.push(m.telemetry().gain_reduction_db);
        }
        out
    }

    #[test]
    fn deq_attack_and_release_times() {
        let mut m = deq(&[
            (1, Float(-40.0)),
            (2, Enum(6)),
            (3, Float(50.0)),
            (4, Float(300.0)),
            (6, Float(-12.0)),
        ]);
        let ms_per = CTRL_BLOCK as f64 / FS * 1e3;
        let env = envelope(&mut m, 1.0, 2.0);
        let t_att = env.iter().position(|&g| g >= 0.632 * 12.0).unwrap() as f64 * ms_per;
        assert!((t_att - 50.0).abs() < 5.0, "attack 63 % at {t_att:.1} ms");
        assert!((env[(0.9 * FS) as usize / CTRL_BLOCK] - 12.0).abs() < 0.01);
        let off = (1.0 * FS) as usize / CTRL_BLOCK;
        let t_rel = env[off..].iter().position(|&g| g <= 0.368 * 12.0).unwrap() as f64 * ms_per;
        // Release tau 300 ms after the detector has fallen below the gain law's saturation (~27 ms).
        assert!(
            (t_rel - 300.0).abs() < 40.0,
            "release 63 % at {t_rel:.1} ms"
        );
        // att = 0: the gain follows within the detector time.
        m.set_param(3, Float(0.0));
        m.commit();
        let env = envelope(&mut m, 0.2, 0.25);
        let t0 = env.iter().position(|&g| g >= 0.632 * 12.0).unwrap() as f64 * ms_per;
        assert!(t0 < 1.0, "att 0: {t0} ms");
        // Mode low, boost: the gain rises (attack) while the key is silent.
        m.set_param(3, Float(20.0));
        m.set_param(6, Float(9.0));
        m.set_param(9, Enum(0));
        m.commit();
        let env = envelope(&mut m, 0.0, 0.3);
        let t = env.iter().position(|&g| g >= 0.632 * 9.0).unwrap() as f64 * ms_per;
        assert!((t - 20.0).abs() < 3.0, "low-mode attack {t}");
    }

    #[test]
    fn deq_stable_under_fast_modulation() {
        let mut seed = 1u32;
        let mut rnd = move || {
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            (seed >> 8) as f32 / (1u32 << 24) as f32 * 2.0 - 1.0
        };
        let n = 96_000;
        let x: Vec<f32> = (0..n)
            .map(|i| {
                let a = if (i / 37) % 2 == 0 { 0.9 } else { 0.001 };
                rnd() * a
            })
            .collect();
        for (fi, filt) in DeqFilter::ALL.into_iter().enumerate() {
            for g in [-15.0, 15.0] {
                for above in [0, 1] {
                    let mut m = deq(&[
                        (1, Float(-40.0)),
                        (2, Enum(6)),
                        (3, Float(0.0)),
                        (4, Float(20.0)),
                        (5, Enum(fi as u32)),
                        (6, Float(g)),
                        (9, Enum(above)),
                    ]);
                    let (mut l, mut r) = (x.clone(), x.clone());
                    for (b, (lc, rc)) in l.chunks_mut(64).zip(r.chunks_mut(64)).enumerate() {
                        // Slam frequency, Q and even the shape every block.
                        m.set_param(7, Float(if b % 2 == 0 { 20.0 } else { 20_000.0 }));
                        m.set_param(8, Float(if b % 3 == 0 { 0.442 } else { 10.0 }));
                        if b % 50 == 49 {
                            m.set_param(5, Enum(((fi + b / 50) % 6) as u32));
                        }
                        m.commit();
                        m.process(lc, rc, None);
                    }
                    let peak = l.iter().chain(&r).fold(0.0f32, |a, v| a.max(v.abs()));
                    assert!(
                        peak.is_finite() && peak < 20.0,
                        "{filt:?} g {g}: peak {peak}"
                    );
                    // Decays to silence afterwards (no limit cycle / runaway).
                    let mut z = vec![0.0f32; 48_000];
                    let mut z2 = z.clone();
                    for (lc, rc) in z.chunks_mut(64).zip(z2.chunks_mut(64)) {
                        m.process(lc, rc, None);
                    }
                    let tail = z[47_000..].iter().fold(0.0f32, |a, v| a.max(v.abs()));
                    assert!(tail < 1e-6, "{filt:?} g {g}: tail {tail}");
                }
            }
        }
    }

    #[test]
    fn deq_gain_modulation_is_zipper_free() {
        // Steady 1 kHz tone; gain slammed by a gated external key (att 5 ms, 15 dB swing every 100 ms).
        // Stepping the gain once per 8-sample control block would jump ~0.5 dB (5.5 % of the amplitude,
        // ~3x the bound below); per-sample coefficient interpolation keeps the output as smooth as a sine.
        let mut m = deq(&[
            (1, Float(-40.0)),
            (2, Enum(6)),
            (3, Float(5.0)),
            (4, Float(20.0)),
            (6, Float(-15.0)),
            (8, Float(1.0)),
        ]);
        let n = 48_000;
        let a = 0.1;
        let x = sine(1000.0, a, n);
        let key: Vec<f32> = sine(1000.0, 0.9, n)
            .into_iter()
            .enumerate()
            .map(|(i, v)| if (i / 4800) % 2 == 0 { v } else { 0.0 })
            .collect();
        let y = run(&mut m, &x, Some(&key));
        let w = 2.0 * PI * 1000.0 / FS;
        let bound = a * (2.0 - 2.0 * w.cos());
        let d2 = y
            .windows(3)
            .map(|v| (v[2] as f64 - 2.0 * v[1] as f64 + v[0] as f64).abs())
            .fold(0.0, f64::max);
        assert!(
            d2 < 1.5 * bound,
            "second difference {d2:.5} vs sine {bound:.5}"
        );
        // The gain really moved.
        let lo = amp_at(&y[4800 - 480..4800], 1000.0);
        let hi = amp_at(&y[9600 - 480..9600], 1000.0);
        assert!(db(hi / lo) > 12.0, "{}", db(hi / lo));
    }

    #[test]
    fn deq2_bands_are_independent_and_in_series() {
        let mut m = DynEq::dual();
        m.prepare(FS, 64);
        for b in 0..2u16 {
            let o = 9 * b;
            m.set_param(o + 1, Float(-30.0));
            m.set_param(o + 2, Enum(6));
            m.set_param(o + 3, Float(5.0));
            m.set_param(o + 6, Float(-10.0));
        }
        m.commit();
        m.reset();
        let n = 24_000;
        let amp = 10f64.powf(-20.0 / 20.0) * SQRT_2;
        let x = sine(200.0, amp, n);
        let y = run(&mut m, &x, None);
        let g0 = m.band(0).dynamic_gain_db() as f64;
        let g1 = m.band(1).dynamic_gain_db() as f64;
        // Tolerance covers the 5 ms mean-square ripple of a 200 Hz tone (~±0.35 dB of level).
        assert!((g0 + 9.0).abs() < 0.4, "band 1 {g0}");
        assert!(g1.abs() < 0.05, "band 2 {g1}");
        let g = db(amp_at(&y[n - 2400..], 200.0) / amp);
        assert!((g + 9.0).abs() < 0.4, "tone {g}");
        assert_eq!(m.band_count(), 2);
        assert_eq!(m.id(), "DEQ2");
    }

    #[test]
    fn deq_listen_outputs_the_sidechain_band() {
        let mut m = deq(&[(7, Float(1000.0)), (8, Float(4.0)), (6, Float(-12.0))]);
        m.set_listen(Some(0));
        let n = 9600;
        for (f, expect) in [(1000.0, 0.0), (250.0, -23.5)] {
            let x = sine(f, 0.5, n);
            m.reset();
            let y = run(&mut m, &x, None);
            let g = db(amp_at(&y[n - 4800..], f) / 0.5);
            assert!((g - expect).abs() < 0.3, "{f}: {g}");
        }
        m.set_listen(Some(1));
        assert!(m.listen.is_none(), "DEQ has one band");
    }

    #[test]
    fn deq_off_is_a_wideband_dynamics_stage() {
        let mut m = deq(&[
            (1, Float(-40.0)),
            (2, Enum(6)),
            (3, Float(1.0)),
            (5, Enum(0)),
            (6, Float(-6.0)),
        ]);
        let n = 24_000;
        for f in [100.0, 6000.0] {
            let x = sine(f, 0.5, n);
            m.reset();
            let y = run(&mut m, &x, None);
            let g = db(amp_at(&y[n - 2400..], f) / 0.5);
            assert!((g + 6.0).abs() < 0.1, "{f}: {g}");
        }
    }

    #[test]
    fn deq_params_defaults_and_clamps() {
        let mut m = DynEq::single();
        assert_eq!(m.param(1), Some(-20.0));
        assert_eq!(m.param(2), Some(2.0));
        assert_eq!(m.param(4), Some(301.0));
        assert_eq!(m.param(5), Some(1.0));
        assert_eq!(m.param(7), Some(1000.0));
        assert_eq!(m.param(9), Some(1.0));
        assert_eq!(m.param(10), None);
        m.set_param(10, Float(1.0));
        m.set_param(1, Float(-90.0));
        assert_eq!(m.param(1), Some(-60.0));
        m.set_param(2, Float(4.5));
        assert_eq!(m.param(2), Some(5.0));
        m.set_param(2, Enum(9));
        assert_eq!(m.param(2), Some(5.0));
        m.set_param(5, Enum(9));
        assert_eq!(m.param(5), Some(5.0));
        m.set_param(8, Float(0.1));
        assert_eq!(m.param(8), Some(0.442));
        m.commit();
        assert_eq!(m.band(0).params().filt, DeqFilter::Hp12);
        m.load_defaults();
        assert_eq!(m.param(1), Some(-20.0));
        let mut d = DynEq::dual();
        assert_eq!(d.param(7), Some(200.0));
        assert_eq!(d.param(16), Some(2990.0));
        assert_eq!(d.param(19), Some(0.0));
        d.set_param(19, ParamValue::Int(1));
        assert_eq!(d.param(19), Some(1.0));
        assert_eq!(d.param(20), None);
        // Default gain 0 dB: transparent apart from rounding.
        d.prepare(FS, 64);
        let x = sine(440.0, 0.5, 4800);
        let y = run(&mut d, &x, None);
        let e = x
            .iter()
            .zip(&y)
            .map(|(a, b)| (a - b).abs())
            .fold(0.0, f32::max);
        assert!(e < 1e-6, "{e}");
        assert_eq!(DeqFilter::from_index(3).key(), "LP12");
    }

    fn ds902(params: &[(u16, ParamValue)]) -> DeEsser902 {
        let mut m = DeEsser902::new();
        m.prepare(FS, 64);
        for &(s, v) in params {
            m.set_param(s, v);
        }
        m.commit();
        m.reset();
        m
    }

    #[test]
    fn ds902_is_transparent_without_sibilance() {
        let mut m = ds902(&[(2, Float(12.0))]);
        let x = sine(300.0, 0.9, 24_000);
        let y = run(&mut m, &x, None);
        assert_eq!(m.reduction_db(), 0.0);
        let e = x
            .iter()
            .zip(&y)
            .map(|(a, b)| (a - b).abs())
            .fold(0.0, f32::max);
        assert!(e < 1e-6, "{e}");
        // Silence never triggers.
        let _ = run(&mut m, &[0.0; 4800], None);
        assert_eq!(m.telemetry().gain_reduction_db, 0.0);
        // Range 0: never reduces.
        let mut m = ds902(&[(2, Float(0.0))]);
        let x = sine(8000.0, 0.5, 9600);
        let y = run(&mut m, &x, None);
        let e = x
            .iter()
            .zip(&y)
            .map(|(a, b)| (a - b).abs())
            .fold(0.0, f32::max);
        assert!(e < 1e-6, "{e}");
    }

    #[test]
    fn ds902_reduction_is_level_independent_and_capped_by_range() {
        // FULL mode, f 5 kHz, range 6: a pure 8 kHz tone is all-HF -> full range, at any level.
        let mut m = ds902(&[(1, Float(5000.0)), (2, Float(6.0)), (3, Enum(0))]);
        let n = 9600;
        for lvl in [-40.0, -20.0, -3.0] {
            let amp = 10f64.powf(lvl / 20.0);
            let x = sine(8000.0, amp, n);
            m.reset();
            let y = run(&mut m, &x, None);
            assert!(
                (m.reduction_db() - 6.0).abs() < 0.01,
                "{lvl}: {}",
                m.reduction_db()
            );
            let g = db(amp_at(&y[n - 480..], 8000.0) / amp);
            assert!((g + 6.0).abs() < 0.05, "{lvl}: {g}");
        }
        // Static law: onset at -12 dB HF ratio, slope 1, capped.
        assert_eq!(m.static_reduction_db(-20.0), 0.0);
        assert!((m.static_reduction_db(-9.0) - 3.0).abs() < 1e-12);
        assert_eq!(m.static_reduction_db(0.0), 6.0);
        // Timing: attack ~1 ms, release ~50 ms (dB one-poles).
        let x: Vec<f32> = sine(8000.0, 0.5, 9600)
            .into_iter()
            .enumerate()
            .map(|(i, v)| if i < 4800 { v } else { 0.0 })
            .collect();
        m.reset();
        let mut env = Vec::new();
        for c in x.chunks(CTRL_BLOCK) {
            let (mut l, mut r) = (c.to_vec(), c.to_vec());
            m.process(&mut l, &mut r, None);
            env.push(m.reduction_db());
        }
        let ms = CTRL_BLOCK as f64 / FS * 1e3;
        let ta = env.iter().position(|&g| g >= 0.632 * 6.0).unwrap() as f64 * ms;
        assert!(ta < 3.0, "attack {ta}");
        let off = 4800 / CTRL_BLOCK;
        let tr = env[off..].iter().position(|&g| g <= 0.368 * 6.0).unwrap() as f64 * ms;
        // Release 50 ms, delayed while the decaying tail is still all-HF: the ratio only drops once the
        // total power reaches the -120 dBFS floor (~25 ms with the 1 ms detector).
        assert!((50.0..90.0).contains(&tr), "release {tr}");
    }

    #[test]
    fn ds902_hf_mode_only_reduces_the_band_above_f() {
        let n = 24_000;
        let x: Vec<f32> = sine(300.0, 0.3, n)
            .iter()
            .zip(sine(8000.0, 0.3, n))
            .map(|(a, b)| a + b)
            .collect();
        // HF ratio about -3.4 dB -> 8.6 dB, capped at range 6. HF mode: -6 dB high shelf at 5 kHz is
        // -5.67 dB at 8 kHz and ~0 dB at 300 Hz.
        for (mode, lf_expect, hf_max) in [(0u32, -6.0, -5.9), (1, 0.0, -5.5)] {
            let mut m = ds902(&[(1, Float(5000.0)), (2, Float(6.0)), (3, Enum(mode))]);
            let y = run(&mut m, &x, None);
            assert!((m.reduction_db() - 6.0).abs() < 0.05);
            let w = &y[n - 2400..];
            let lf = db(amp_at(w, 300.0) / 0.3);
            let hf = db(amp_at(w, 8000.0) / 0.3);
            assert!((lf - lf_expect).abs() < 0.05, "mode {mode}: 300 Hz {lf}");
            assert!(hf <= hf_max, "mode {mode}: 8 kHz {hf}");
        }
        // External key: a sibilant key ducks a signal with no HF of its own (FULL mode).
        let mut m = ds902(&[(2, Float(6.0)), (3, Enum(0))]);
        let x = sine(300.0, 0.3, n);
        let key = sine(8000.0, 0.3, n);
        let y = run(&mut m, &x, Some(&key));
        let lf = db(amp_at(&y[n - 2400..], 300.0) / 0.3);
        assert!((lf + 6.0).abs() < 0.05, "{lf}");
    }

    #[test]
    fn ds902_params() {
        let mut m = DeEsser902::new();
        assert_eq!(m.id(), "DS902");
        assert_eq!(m.param(1), Some(5000.0));
        assert_eq!(m.param(2), Some(4.0));
        assert_eq!(m.param(3), Some(1.0));
        assert_eq!(m.param(4), None);
        m.set_param(1, Float(100.0));
        m.set_param(2, Float(40.0));
        m.set_param(3, Enum(5));
        m.set_param(4, Float(1.0));
        assert_eq!(m.param(1), Some(800.0));
        assert_eq!(m.param(2), Some(12.0));
        assert_eq!(m.param(3), Some(1.0));
        m.load_defaults();
        assert_eq!(m.param(1), Some(5000.0));
    }
}
