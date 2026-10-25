//! FET-style compressors (T04D): `76LA` (76 LIMITER AMP, UREI/UA 1176LN) and `NSTR` (NO-STRESSOR, Empirical
//! Labs EL8 Distressor). Parameters, ranges and defaults: `docs/research/models.md` §3.2 and
//! `data/schema/wing-tree.json` (slot order `in out att rel ratio` for both).
//!
//! Both share one stereo-linked core that runs at **2x oversampling** (`wing_dsp::oversample::Oversampler2x`,
//! linear-phase half-band, latency 39 samples reported by [`Model::latency`]):
//! ```text
//! x -> input drive (in knob) -> 2x up -> |peak| key (max of L/R) -> level dB -> static curve (fixed threshold,
//!      ratio-dependent knee / threshold / ALL-buttons dip) -> dB-domain ballistics (attack, programme-dependent
//!      or opto release, ALL overshoot) -> gain cell -> colouration (FET / harmonic stage) -> output gain
//!      -> output-stage soft saturation -> 2x down -> DC blocker -> y
//! ```
//! Detection, gain and every nonlinearity run at the oversampled rate, so the gain-modulation sidebands of the
//! fast FET ballistics and the polynomial colouration stages (order <= 3) stay alias-free (tested: aliasing
//! below -90 dBc for a 10 kHz input). Both units have a *fixed* threshold: the input knob drives the gain cell,
//! the output knob is make-up.
//!
//! **76LA** (1176LN): input/output knobs are dB attenuators (-48..0; the schema defaults -30/-27 give unity
//! gain below threshold); attack 1..7 and release 1..7 are 1176 knob numbers (7 = fastest) mapped
//! geometrically onto the published 800..20 µs and 1100..50 ms ranges; ratio buttons 4/8/12/20 with
//! ratio-dependent knee (soft at 4:1, harder at 20:1) and threshold; ALL ("all buttons in"): 20:1 with a
//! rolled-over "dip" in the static curve, transient GR overshoot and a changed FET bias (more distortion).
//! Release is programme-dependent: it runs at the published time after sustained gain reduction and up to
//! [`PD_MIN`] x faster after short transients. Colouration: FET gain-cell 2nd harmonic growing with GR,
//! class-A output stage (2nd + 3rd), plus the inherent GR-ripple distortion of fast attack/release at low
//! frequencies.
//!
//! **NSTR** (Distressor): knobs 0..10 (input drive and output make-up 4 dB per step, unity at the schema
//! defaults 4/3); attack 0..10 = 50 µs..30 ms, release 0..10 = 50 ms..3.5 s (geometric); ratios 1.5:1..20:1 and
//! NUKE (brick wall) with ratio-specific knees (1.5/2/3:1 very soft and opto-like release, 10:1 the 1176
//! emulation with its programme-dependent release, 20:1/NUKE hard). WING exposes neither the Distressor's
//! 1:1 ratio, Dist 2 / Dist 3 modes, detector HP / band emphasis nor the release-mode switch
//! (models.md: "distortion modes not exposed"), so none are modelled; the always-on "normal mode" harmonic
//! signature (low-order, low-frequency weighted, growing with ratio) is.
//!
//! Every constant below names its source; values marked *assumption* are listed in
//! `docs/VERIFY-ON-HARDWARE.md`. GR telemetry is dB; [`gr_meter`] reproduces WING-Edit's GR-meter mapping
//! (`comp_gain_u1176` / `comp_gain_distress`, `data/reference-curves/dyn-models.json`).
//!
//! THD profile (2x oversampled, measured by `thd_profile_is_documented_and_bounded`, 1 kHz unless stated):
//! see the T04D task Log for the measured table.
//!
//! Real-time: `prepare` allocates the oversamplers and scratch; `process`, `set_param`, `commit`, `reset` are
//! allocation-free. Per oversampled sample: one fast log, one or two fast exp, a few multiply-adds.

use crate::model::{Model, ModelTelemetry, ParamValue};
use wing_dsp::gain::{db_to_lin, db_to_lin_fast, lin_to_db_fast};
use wing_dsp::oversample::{Oversampler2x, Oversampler4x};
use wing_dsp::smooth::{one_pole_coeff, OnePoleSmoother};

/// Oversampling factor of the gain cell and colouration stages.
pub const OS: usize = 2;
/// Control-path sub-steps per oversampled sample: the detector / static curve / ballistics run at
/// `fs * OS * CTRL_SUB` (768 kHz at 48 kHz) on a key interpolated by half-band stages, and the resulting GR
/// (dB) is low-pass decimated back to the audio rate. A fast (20 µs) attack catches every half-cycle peak
/// within a few samples, so how much it catches depends on where the samples fall on the peak; that
/// sample-phase dependence repeats with the beat of ripple and sample rate and modulates the signal. Measured
/// worst-case aliasing for a 10 kHz input (76LA attack 7, release 7): -68 dBc with the detector at 96 kHz,
/// -72 dBc at 384 kHz on a linearly interpolated key, -86 dBc at 384 kHz with a half-band key, -100 dBc at
/// 768 kHz (this setting).
pub const CTRL_SUB: usize = 8;
/// GR decimation filter (control rate -> oversampled audio rate): Kaiser-windowed sinc low-pass,
/// [`GR_FIR_LEN`] taps, cut-off [`GR_FIR_CUTOFF_HZ`] at 48 kHz base rate (scaled with fs), beta
/// [`GR_FIR_BETA`]. Its group delay ((LEN-1)/2 = 41 control samples) plus the key up-sampler delay
/// ([`KEY_UP_DELAY_CTL`], 247) is matched by an equal delay of the oversampled audio path (36 samples = 18
/// base samples), so the gain stays time-aligned with the signal. Total model latency: 39 + 18 = 57.
pub const GR_FIR_LEN: usize = 83;
pub const GR_FIR_CUTOFF_HZ: f64 = 30_000.0;
pub const GR_FIR_BETA: f64 = 9.0;
/// Group delay (control samples) of the 2x -> 16x key interpolator (`Oversampler2x` then
/// `Oversampler4x`: three half-band stages of 79 / 79 / 27 taps, i.e. 39 / 39 / 13 samples at their output
/// rates 4x / 8x / 16x).
pub const KEY_UP_DELAY_CTL: usize = 4 * 39 + 2 * 39 + 13;
/// Audio-path delay (oversampled samples) matching key interpolator + GR FIR.
const AUDIO_DELAY_OS: usize = (KEY_UP_DELAY_CTL + (GR_FIR_LEN - 1) / 2) / CTRL_SUB;
const _: () = assert!(AUDIO_DELAY_OS * CTRL_SUB == KEY_UP_DELAY_CTL + (GR_FIR_LEN - 1) / 2);
const _: () = assert!(AUDIO_DELAY_OS % OS == 0);

// ---- 76LA (1176LN) ---------------------------------------------------------------------------------------

/// 1176LN attack at knob 1 (slowest) and 7 (fastest): "800 µs" / "20 µs" (UA 1176LN manual specs, also
/// models.md §3.2: "attack 20 us..800 us ... 7 = fastest"). Modelled as the dB-domain one-pole time constant.
pub const LA76_ATTACK_SLOW_S: f64 = 800e-6;
pub const LA76_ATTACK_FAST_S: f64 = 20e-6;
/// 1176LN release at knob 1 / 7: "1100 ms" / "50 ms" (UA 1176LN manual specs; models.md §3.2).
pub const LA76_RELEASE_SLOW_S: f64 = 1.1;
pub const LA76_RELEASE_FAST_S: f64 = 0.050;
/// Input / output knob values (dB) that give 0 dB drive / make-up: the schema defaults (-30 / -27), i.e. the
/// default setting is unity gain below threshold. *Assumption*: WING's pot law and the 1176's 45 dB total
/// gain are not mapped by any source.
pub const LA76_IN_UNITY_DB: f32 = -30.0;
pub const LA76_OUT_UNITY_DB: f32 = -27.0;
/// Fixed gain-cell threshold (dBFS at the cell input, 4:1). *Assumption* (0 dBu at EBU R68 alignment).
pub const LA76_THR_DBFS: f32 = -18.0;
/// Ratio buttons (schema enum `4 / 8 / 12 / 20 / ALL`).
pub const LA76_RATIOS: [f32; 4] = [4.0, 8.0, 12.0, 20.0];
/// Threshold offset per button (dB, index 4 = ALL): higher ratios raise the 1176 threshold (the ratio
/// switch also changes the detector divider). *Assumption* for the amounts.
pub const LA76_THR_OFFSET_DB: [f32; 5] = [0.0, 2.0, 3.0, 4.0, 4.0];
/// Quadratic knee width per button (dB): "soft knee at 4:1, harder at 20:1" (models.md §3.2). *Assumption*
/// for the widths.
pub const LA76_KNEE_DB: [f32; 5] = [10.0, 6.0, 4.0, 2.0, 2.0];
/// ALL buttons: base ratio ("between 12:1 and 20:1", commonly documented; 20 used) plus a rolled-over dip
/// ("over-compression with nasty knee", models.md): extra GR `S * D * (1 - e^(-d/D))` above threshold, i.e.
/// the output first *falls* by ~1.3 dB, then rises at 20:1. *Assumption* for `S`, `D`.
pub const LA76_ALL_RATIO: f32 = 20.0;
pub const LA76_ALL_DIP_SLOPE: f32 = 0.3;
pub const LA76_ALL_DIP_SCALE_DB: f32 = 8.0;
/// ALL buttons transient overshoot ("special static curve + transient overshoot", models.md): extra GR =
/// `K * max(gr - slow(gr), 0)`, slow = one-pole of [`LA76_ALL_OVERSHOOT_TAU_S`]. *Assumption*.
pub const LA76_ALL_OVERSHOOT: f32 = 0.5;
pub const LA76_ALL_OVERSHOOT_TAU_S: f64 = 0.010;
/// Programme-dependent release (1176, and NSTR 10:1): history `h` = one-pole of GR with
/// [`PD_HIST_TAU_S`]; release time = published * (PD_MIN + (1 - PD_MIN) * min(h / gr, 1)). Sustained GR
/// releases at the published time, a short transient up to `1 / PD_MIN` faster. *Assumption* for both.
pub const PD_HIST_TAU_S: f64 = 0.3;
pub const PD_MIN: f32 = 0.5;
/// FET gain-cell 2nd-harmonic coefficient (normalised to the threshold level) and its GR dependence
/// `depth = gr / (gr + LA76_FET_GR_HALF_DB)`. Target: ~0.3 % H2 at 10 dB GR with the cell at threshold
/// level, within the 1176LN spec "< 0.5 % THD". *Assumption* for the split between stages.
pub const LA76_FET_K2: f32 = 0.01;
pub const LA76_FET_GR_HALF_DB: f32 = 6.0;
/// ALL buttons changes the FET bias: FET distortion x3. *Assumption*.
pub const LA76_ALL_BIAS: f32 = 3.0;
/// Class-A output stage: 2nd-harmonic coefficient and soft-saturation headroom (linear, 4.0 = +12 dBFS;
/// cubic `z - z^3/(3H^2)`, ~0.5 % H3 at 0 dBFS). *Assumption*.
pub const LA76_OUT_K2: f32 = 0.004;
pub const LA76_OUT_HEADROOM: f32 = 4.0;

// ---- NSTR (Distressor) -----------------------------------------------------------------------------------

/// Distressor attack at knob 0 / 10: "~50 us..30 ms" (models.md §3.2, EL8 manual). Higher knob = slower.
pub const NSTR_ATTACK_FAST_S: f64 = 50e-6;
pub const NSTR_ATTACK_SLOW_S: f64 = 0.030;
/// Distressor release at knob 0 / 10: "~50 ms..3.5 s" (models.md §3.2, EL8 manual "normal" release modes).
pub const NSTR_RELEASE_FAST_S: f64 = 0.050;
pub const NSTR_RELEASE_SLOW_S: f64 = 3.5;
/// Input / output knob law: dB per knob step and the knob value of unity (schema defaults 4.0 / 3.0).
/// *Assumption*: the EL8 knob calibration is not published.
pub const NSTR_DB_PER_STEP: f32 = 4.0;
pub const NSTR_IN_UNITY: f32 = 4.0;
pub const NSTR_OUT_UNITY: f32 = 3.0;
/// Fixed threshold (dBFS at the gain cell). *Assumption*.
pub const NSTR_THR_DBFS: f32 = -18.0;
/// Ratio enum `1.5:1 / 2:1 / 3:1 / 4:1 / 6:1 / 10:1 / 20:1 / NUKE` (schema); NUKE = infinity (brick wall).
pub const NSTR_RATIOS: [f32; 8] = [1.5, 2.0, 3.0, 4.0, 6.0, 10.0, 20.0, f32::INFINITY];
/// Knee widths per ratio (dB): "1:1 & 2:1 are opto-like soft ... 10:1 emulates 1176" (models.md; EL8
/// manual ratio descriptions: soft low ratios, progressively harder, NUKE brick wall). *Assumption* for
/// the widths.
pub const NSTR_KNEE_DB: [f32; 8] = [12.0, 12.0, 10.0, 6.0, 4.0, 2.0, 1.0, 0.0];
/// Opto-like release of the soft ratios (1.5/2/3:1): release time = published * m(gr),
/// `m = MIN + (MAX - MIN) * e^(-gr / GR_DB)`: fast while GR is large, slow tail near 0 dB GR (T4-cell style,
/// EL8 manual: low ratios "opto"-like). *Assumption*.
pub const NSTR_OPTO_MIN: f32 = 0.5;
pub const NSTR_OPTO_MAX: f32 = 2.0;
pub const NSTR_OPTO_GR_DB: f32 = 6.0;
/// Normal-mode harmonic signature: `y + K2*lp^2/T - K3*lp^3/T^2` with `lp` = one-pole low-pass of the cell
/// output at [`NSTR_LF_HZ`] (low-frequency weighted, as tape/transformer stages) and `T` the threshold
/// level; scaled per ratio by [`NSTR_COLOUR`] ("mild harmonic generation tied to ratio", models.md).
/// *Assumption* for all values.
pub const NSTR_K2: f32 = 0.004;
pub const NSTR_K3: f32 = 0.002;
pub const NSTR_LF_HZ: f64 = 300.0;
pub const NSTR_COLOUR: [f32; 8] = [0.5, 0.6, 0.7, 0.8, 1.0, 1.4, 1.7, 2.0];
/// Output-stage saturation headroom (linear, +18 dBFS). *Assumption*.
pub const NSTR_OUT_HEADROOM: f32 = 8.0;

// ---- shared ----------------------------------------------------------------------------------------------

/// DC blocker after the even-order stages (one-pole high-pass; -0.04 dB at 20 Hz).
pub const DC_BLOCK_HZ: f64 = 2.0;
/// Drive / make-up de-zipper time constant.
pub const GAIN_SMOOTH_S: f64 = 0.010;
/// GR ceiling (dB).
pub const GR_MAX_DB: f32 = 80.0;
const KEY_FLOOR: f32 = 1e-12;

/// Which model an instance is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FetKind {
    /// `76LA` 76 LIMITER AMP (1176LN).
    La76,
    /// `NSTR` NO-STRESSOR (Distressor).
    Nstr,
}

impl FetKind {
    pub const ALL: [FetKind; 2] = [FetKind::La76, FetKind::Nstr];

    /// Snapshot `mdl` token.
    pub fn id(self) -> &'static str {
        match self {
            FetKind::La76 => "76LA",
            FetKind::Nstr => "NSTR",
        }
    }

    pub fn from_id(id: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|k| k.id() == id)
    }

    /// Number of entries in the ratio enum.
    pub fn num_ratios(self) -> usize {
        match self {
            FetKind::La76 => 5,
            FetKind::Nstr => NSTR_RATIOS.len(),
        }
    }

    /// Slot ranges and schema defaults `in out att rel ratio(index)`.
    fn slots(self) -> [(f32, f32, f32); 5] {
        match self {
            FetKind::La76 => [
                (-48.0, 0.0, -30.0),
                (-48.0, 0.0, -27.0),
                (1.0, 7.0, 2.0),
                (1.0, 7.0, 2.0),
                (0.0, 4.0, 1.0),
            ],
            FetKind::Nstr => [
                (0.0, 10.0, 4.0),
                (0.0, 10.0, 3.0),
                (0.0, 10.0, 4.0),
                (0.0, 10.0, 5.0),
                (0.0, 7.0, 2.0),
            ],
        }
    }
}

/// WING-Edit GR-meter position (`dyn-models.json`): 76LA deflects negatively, `-GR/20` capped at -1;
/// NSTR `GR/20` capped at 1.3.
pub fn gr_meter(kind: FetKind, gr_db: f32) -> f32 {
    let gr = gr_db.max(0.0);
    match kind {
        FetKind::La76 => -(gr / 20.0).min(1.0),
        FetKind::Nstr => (gr / 20.0).min(1.3),
    }
}

/// 1176 attack time constant (s) for knob 1..7 (7 = fastest), geometric between the published endpoints.
pub fn la76_attack_s(knob: f32) -> f64 {
    let t = ((knob.clamp(1.0, 7.0) - 1.0) / 6.0) as f64;
    LA76_ATTACK_SLOW_S * (LA76_ATTACK_FAST_S / LA76_ATTACK_SLOW_S).powf(t)
}

/// 1176 release time constant (s) for knob 1..7 (7 = fastest).
pub fn la76_release_s(knob: f32) -> f64 {
    let t = ((knob.clamp(1.0, 7.0) - 1.0) / 6.0) as f64;
    LA76_RELEASE_SLOW_S * (LA76_RELEASE_FAST_S / LA76_RELEASE_SLOW_S).powf(t)
}

/// Distressor attack time constant (s) for knob 0..10 (10 = slowest).
pub fn nstr_attack_s(knob: f32) -> f64 {
    let t = (knob.clamp(0.0, 10.0) / 10.0) as f64;
    NSTR_ATTACK_FAST_S * (NSTR_ATTACK_SLOW_S / NSTR_ATTACK_FAST_S).powf(t)
}

/// Distressor release time constant (s) for knob 0..10 (10 = slowest).
pub fn nstr_release_s(knob: f32) -> f64 {
    let t = (knob.clamp(0.0, 10.0) / 10.0) as f64;
    NSTR_RELEASE_FAST_S * (NSTR_RELEASE_SLOW_S / NSTR_RELEASE_FAST_S).powf(t)
}

/// Static gain computer: quadratic soft knee around `thr`, slope `1 - 1/ratio`, optional ALL-buttons dip.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Curve {
    pub thr_db: f32,
    pub slope: f32,
    pub knee_db: f32,
    pub dip_slope: f32,
    pub dip_scale_db: f32,
}

impl Curve {
    /// Gain reduction (dB, >= 0) for a detected level (dBFS at the cell).
    #[inline]
    pub fn gr(&self, level_db: f32) -> f32 {
        let d = level_db - self.thr_db;
        let w = self.knee_db;
        let base = if w <= 0.0 {
            self.slope * d.max(0.0)
        } else if d <= -0.5 * w {
            0.0
        } else if d < 0.5 * w {
            let e = d + 0.5 * w;
            self.slope * e * e / (2.0 * w)
        } else {
            self.slope * d
        };
        let dip = if self.dip_slope > 0.0 && d > 0.0 {
            // S * D * (1 - e^(-d/D)); e^-u = 10^(-u / ln10) -> dB form for the fast helper.
            let u = d / self.dip_scale_db;
            self.dip_slope
                * self.dip_scale_db
                * (1.0 - db_to_lin_fast(-u * (20.0 / core::f32::consts::LN_10)))
        } else {
            0.0
        };
        (base + dip).min(GR_MAX_DB)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ReleaseMode {
    Plain,
    /// 1176 programme-dependent (history ratio).
    ProgramDependent,
    /// Distressor low-ratio opto-like (GR-dependent).
    Opto,
}

/// Output-stage soft saturation: `zc - zc^3 / (3 H^2) + a2 zc^2`, `zc = clamp(z, -H, H)`.
#[inline(always)]
fn saturate(z: f32, h: f32, inv3h2: f32, a2: f32) -> f32 {
    let zc = z.clamp(-h, h);
    zc - zc * zc * zc * inv3h2 + a2 * zc * zc
}

/// One of the two FET-style compressors (stereo-linked, 2x oversampled).
pub struct FetComp {
    kind: FetKind,
    fs: f64,
    max_block: usize,
    /// Staged slot values `in out att rel ratio-index`.
    v: [f32; 5],
    dirty: bool,
    colour: bool,
    // derived
    curve: Curve,
    drive_db: f32,
    out_db: f32,
    att_s: f64,
    rel_s: f64,
    att_c: f32,
    /// Release rate `1 / (tau * fs_os)` (per-sample exponent), scaled per sample in PD / opto modes.
    rel_rate: f32,
    rel_c: f32,
    rel_mode: ReleaseMode,
    hist_c: f32,
    overshoot: f32,
    ov_c: f32,
    fet_k: f32,
    out_k2: f32,
    out_h: f32,
    out_inv3h2: f32,
    nstr_k2: f32,
    nstr_k3: f32,
    lf_a: f32,
    thr_lin: f32,
    inv_thr: f32,
    dc_r: f32,
    // state
    gr: f32,
    hist: f32,
    ov_slow: f32,
    gr_now: f32,
    lp: [f32; 2],
    dc_x: [f32; 2],
    dc_y: [f32; 2],
    drive: OnePoleSmoother,
    out_g: OnePoleSmoother,
    // oversampling (allocated in prepare)
    os: [Oversampler2x; 2],
    pre: [Vec<f32>; 2],
    hi: [Vec<f32>; 2],
    kos2: [Oversampler2x; 2],
    kmid: [Vec<f32>; 2],
    kos: [Oversampler4x; 2],
    khi: [Vec<f32>; 2],
    sc_os: [Oversampler2x; 2],
    sc_pre: [Vec<f32>; 2],
    sc_hi: [Vec<f32>; 2],
    gr_taps: Vec<f32>,
    /// Mirrored control-rate GR history (newest first window at `gr_pos`).
    gr_hist: Vec<f32>,
    gr_pos: usize,
    /// Audio delay lines (oversampled rate) per channel.
    adly: [[f32; AUDIO_DELAY_OS]; 2],
    adly_pos: usize,
}

impl FetComp {
    pub fn new(kind: FetKind) -> Self {
        let mut m = Self {
            kind,
            fs: 48_000.0,
            max_block: 0,
            v: [0.0; 5],
            dirty: true,
            colour: true,
            curve: Curve {
                thr_db: 0.0,
                slope: 0.0,
                knee_db: 0.0,
                dip_slope: 0.0,
                dip_scale_db: 1.0,
            },
            drive_db: 0.0,
            out_db: 0.0,
            att_s: 0.0,
            rel_s: 0.0,
            att_c: 0.0,
            rel_rate: 0.0,
            rel_c: 0.0,
            rel_mode: ReleaseMode::Plain,
            hist_c: 0.0,
            overshoot: 0.0,
            ov_c: 0.0,
            fet_k: 0.0,
            out_k2: 0.0,
            out_h: 1.0,
            out_inv3h2: 0.0,
            nstr_k2: 0.0,
            nstr_k3: 0.0,
            lf_a: 0.0,
            thr_lin: 1.0,
            inv_thr: 1.0,
            dc_r: 0.0,
            gr: 0.0,
            hist: 0.0,
            ov_slow: 0.0,
            gr_now: 0.0,
            lp: [0.0; 2],
            dc_x: [0.0; 2],
            dc_y: [0.0; 2],
            drive: OnePoleSmoother::new(1.0, 0.0, 48_000.0),
            out_g: OnePoleSmoother::new(1.0, 0.0, 96_000.0),
            os: [Oversampler2x::new(1), Oversampler2x::new(1)],
            pre: [Vec::new(), Vec::new()],
            hi: [Vec::new(), Vec::new()],
            kos2: [Oversampler2x::new(1), Oversampler2x::new(1)],
            kmid: [Vec::new(), Vec::new()],
            kos: [Oversampler4x::new(1), Oversampler4x::new(1)],
            khi: [Vec::new(), Vec::new()],
            sc_os: [Oversampler2x::new(1), Oversampler2x::new(1)],
            sc_pre: [Vec::new(), Vec::new()],
            sc_hi: [Vec::new(), Vec::new()],
            gr_taps: Vec::new(),
            gr_hist: Vec::new(),
            gr_pos: 0,
            adly: [[0.0; AUDIO_DELAY_OS]; 2],
            adly_pos: 0,
        };
        m.load_defaults();
        m.prepare(48_000.0, 64);
        m
    }

    pub fn kind(&self) -> FetKind {
        self.kind
    }

    /// Enable / disable the colouration stages (FET / harmonic terms, output saturation, DC blocker). On by
    /// default; off leaves the pure dynamics law (for static-curve and timing measurements with DC input).
    pub fn set_coloration(&mut self, on: bool) {
        self.colour = on;
    }

    /// Oversampled audio rate (Hz).
    pub fn os_rate(&self) -> f64 {
        self.fs * OS as f64
    }

    /// Control-path (detector / ballistics) rate (Hz).
    pub fn control_rate(&self) -> f64 {
        self.fs * (OS * CTRL_SUB) as f64
    }

    /// Current static curve (cell levels, dBFS).
    pub fn curve(&mut self) -> Curve {
        self.apply();
        self.curve
    }

    /// Input drive (dB) of the current `in` setting.
    pub fn drive_db(&mut self) -> f32 {
        self.apply();
        self.drive_db
    }

    /// Output make-up (dB) of the current `out` setting.
    pub fn makeup_db(&mut self) -> f32 {
        self.apply();
        self.out_db
    }

    /// Current attack / release time constants (s) from the knobs.
    pub fn times_s(&mut self) -> (f64, f64) {
        self.apply();
        (self.att_s, self.rel_s)
    }

    /// Static output level (dBFS) for a steady input peak level (dBFS), colouration ignored.
    pub fn static_out_db(&mut self, in_db: f32) -> f32 {
        self.apply();
        let cell = in_db + self.drive_db;
        cell - self.curve.gr(cell) + self.out_db
    }

    /// Run one control-rate detector step ([`FetComp::control_rate`]) for an input-referred key magnitude
    /// (linear, before drive); returns the GR (dB). Same code path as `process` (for timing tests).
    pub fn detector_step(&mut self, key_in: f32) -> f32 {
        self.apply();
        let d = db_to_lin(self.drive_db);
        self.gr_step(key_in.abs() * d)
    }

    fn apply(&mut self) {
        if !self.dirty {
            return;
        }
        self.dirty = false;
        let fs_os = self.fs * OS as f64;
        let fs_ctl = fs_os * CTRL_SUB as f64;
        let c = |tau: f64| one_pole_coeff(tau, fs_ctl) as f32;
        let v = self.v;
        let ri = v[4] as usize;
        match self.kind {
            FetKind::La76 => {
                self.drive_db = v[0] - LA76_IN_UNITY_DB;
                self.out_db = v[1] - LA76_OUT_UNITY_DB;
                self.att_s = la76_attack_s(v[2]);
                self.rel_s = la76_release_s(v[3]);
                let all = ri >= LA76_RATIOS.len();
                let ratio = if all { LA76_ALL_RATIO } else { LA76_RATIOS[ri] };
                let thr = LA76_THR_DBFS + LA76_THR_OFFSET_DB[ri.min(4)];
                self.curve = Curve {
                    thr_db: thr,
                    slope: 1.0 - 1.0 / ratio,
                    knee_db: LA76_KNEE_DB[ri.min(4)],
                    dip_slope: if all { LA76_ALL_DIP_SLOPE } else { 0.0 },
                    dip_scale_db: LA76_ALL_DIP_SCALE_DB,
                };
                self.rel_mode = ReleaseMode::ProgramDependent;
                self.overshoot = if all { LA76_ALL_OVERSHOOT } else { 0.0 };
                self.fet_k = LA76_FET_K2 * if all { LA76_ALL_BIAS } else { 1.0 };
                self.out_k2 = LA76_OUT_K2;
                self.out_h = LA76_OUT_HEADROOM;
                self.nstr_k2 = 0.0;
                self.nstr_k3 = 0.0;
                self.thr_lin = db_to_lin(thr);
            }
            FetKind::Nstr => {
                self.drive_db = (v[0] - NSTR_IN_UNITY) * NSTR_DB_PER_STEP;
                self.out_db = (v[1] - NSTR_OUT_UNITY) * NSTR_DB_PER_STEP;
                self.att_s = nstr_attack_s(v[2]);
                self.rel_s = nstr_release_s(v[3]);
                let ri = ri.min(NSTR_RATIOS.len() - 1);
                self.curve = Curve {
                    thr_db: NSTR_THR_DBFS,
                    slope: 1.0 - 1.0 / NSTR_RATIOS[ri],
                    knee_db: NSTR_KNEE_DB[ri],
                    dip_slope: 0.0,
                    dip_scale_db: 1.0,
                };
                self.rel_mode = match ri {
                    0..=2 => ReleaseMode::Opto,
                    5 => ReleaseMode::ProgramDependent,
                    _ => ReleaseMode::Plain,
                };
                self.overshoot = 0.0;
                self.fet_k = 0.0;
                self.out_k2 = 0.0;
                self.out_h = NSTR_OUT_HEADROOM;
                self.nstr_k2 = NSTR_K2 * NSTR_COLOUR[ri];
                self.nstr_k3 = NSTR_K3 * NSTR_COLOUR[ri];
                self.thr_lin = db_to_lin(NSTR_THR_DBFS);
            }
        }
        self.inv_thr = 1.0 / self.thr_lin;
        self.out_inv3h2 = 1.0 / (3.0 * self.out_h * self.out_h);
        self.att_c = c(self.att_s);
        self.rel_rate = (1.0 / (self.rel_s * fs_ctl)) as f32;
        self.rel_c = c(self.rel_s);
        self.hist_c = c(PD_HIST_TAU_S);
        self.ov_c = c(LA76_ALL_OVERSHOOT_TAU_S);
        self.lf_a =
            1.0 - one_pole_coeff(1.0 / (2.0 * core::f64::consts::PI * NSTR_LF_HZ), fs_os) as f32;
        self.dc_r = one_pole_coeff(1.0 / (2.0 * core::f64::consts::PI * DC_BLOCK_HZ), self.fs) as f32;
        self.drive.set_target(db_to_lin(self.drive_db));
        self.out_g.set_target(db_to_lin(self.out_db));
    }

    /// Static curve + ballistics for one control-rate sample of the (driven) key magnitude.
    #[inline(always)]
    fn gr_step(&mut self, key: f32) -> f32 {
        let target = self.curve.gr(lin_to_db_fast(key.max(KEY_FLOOR)));
        let g = self.gr;
        let rel_c = match self.rel_mode {
            ReleaseMode::Plain => self.rel_c,
            ReleaseMode::ProgramDependent => {
                let h = if g > 1e-3 {
                    (self.hist / g).min(1.0)
                } else {
                    1.0
                };
                let m = PD_MIN + (1.0 - PD_MIN) * h;
                rate_coeff(self.rel_rate / m)
            }
            ReleaseMode::Opto => {
                let e = db_to_lin_fast(-g * (20.0 / core::f32::consts::LN_10) / NSTR_OPTO_GR_DB);
                let m = NSTR_OPTO_MIN + (NSTR_OPTO_MAX - NSTR_OPTO_MIN) * e;
                rate_coeff(self.rel_rate / m)
            }
        };
        let c = if target > g { self.att_c } else { rel_c };
        let g = target + c * (g - target);
        self.gr = g;
        self.hist = g + self.hist_c * (self.hist - g);
        if self.overshoot > 0.0 {
            self.ov_slow = g + self.ov_c * (self.ov_slow - g);
            g + self.overshoot * (g - self.ov_slow).max(0.0)
        } else {
            g
        }
    }

    /// Push one control-rate GR value into the mirrored FIR history.
    #[inline(always)]
    fn push_gr(&mut self, g: f32) {
        self.gr_pos = if self.gr_pos == 0 {
            GR_FIR_LEN - 1
        } else {
            self.gr_pos - 1
        };
        self.gr_hist[self.gr_pos] = g;
        self.gr_hist[self.gr_pos + GR_FIR_LEN] = g;
    }

    /// Colouration of one channel sample after the gain cell (oversampled rate), then make-up and output
    /// stage.
    #[inline(always)]
    fn colour_out(&mut self, ch: usize, y: f32, gr: f32, out_g: f32) -> f32 {
        let y = match self.kind {
            FetKind::La76 => {
                let depth = gr / (gr + LA76_FET_GR_HALF_DB);
                y + self.fet_k * depth * self.inv_thr * y * y
            }
            FetKind::Nstr => {
                let lp = self.lp[ch] + self.lf_a * (y - self.lp[ch]);
                self.lp[ch] = lp;
                let u = lp * self.inv_thr;
                y + self.thr_lin * u * u * (self.nstr_k2 - self.nstr_k3 * u)
            }
        };
        saturate(y * out_g, self.out_h, self.out_inv3h2, self.out_k2)
    }

    fn process_chunk(&mut self, l: &mut [f32], r: &mut [f32], sc: Option<(&[f32], &[f32])>) {
        let n = l.len();
        let n2 = OS * n;
        // Input drive (de-zippered at base rate); an external key gets the same drive.
        for i in 0..n {
            let d = self.drive.tick();
            self.pre[0][i] = l[i] * d;
            self.pre[1][i] = r[i] * d;
            if let Some((a, b)) = sc {
                self.sc_pre[0][i] = a.get(i).copied().unwrap_or(0.0) * d;
                self.sc_pre[1][i] = b.get(i).copied().unwrap_or(0.0) * d;
            }
        }
        let n_ctl = CTRL_SUB * n2;
        for ch in 0..2 {
            self.os[ch].upsample(&self.pre[ch][..n], &mut self.hi[ch][..n2]);
            // Key: the audio (or the external key, 2x upsampled) interpolated 4x further by half-band
            // stages, so the rectifier sees no interpolation images.
            // (two disjoint field borrows: kin may alias hi / sc_hi, the up-samplers write kmid / khi)
            let kin: &[f32] = if sc.is_some() {
                self.sc_os[ch].upsample(&self.sc_pre[ch][..n], &mut self.sc_hi[ch][..n2]);
                &self.sc_hi[ch][..n2]
            } else {
                &self.hi[ch][..n2]
            };
            self.kos2[ch].upsample(kin, &mut self.kmid[ch][..2 * n2]);
            self.kos[ch].upsample(&self.kmid[ch][..2 * n2], &mut self.khi[ch][..n_ctl]);
        }
        let colour = self.colour;
        for j in 0..n2 {
            // Control path at CTRL_SUB x the audio rate; GR decimated back through the low-pass FIR; audio
            // delayed so it meets the GR it produced (key up-sampler + FIR group delay).
            for s in 0..CTRL_SUB {
                let c = CTRL_SUB * j + s;
                let key = self.khi[0][c].abs().max(self.khi[1][c].abs());
                let g = self.gr_step(key);
                self.push_gr(g);
            }
            let gr = dot(&self.gr_taps, &self.gr_hist[self.gr_pos..self.gr_pos + GR_FIR_LEN]).max(0.0);
            let p = self.adly_pos;
            let (xl, xr) = (self.adly[0][p], self.adly[1][p]);
            self.adly[0][p] = self.hi[0][j];
            self.adly[1][p] = self.hi[1][j];
            self.adly_pos = if p + 1 == AUDIO_DELAY_OS { 0 } else { p + 1 };
            let g = db_to_lin_fast(-gr);
            let og = self.out_g.tick();
            let (yl, yr) = if colour {
                (
                    self.colour_out(0, xl * g, gr, og),
                    self.colour_out(1, xr * g, gr, og),
                )
            } else {
                (xl * g * og, xr * g * og)
            };
            self.hi[0][j] = yl;
            self.hi[1][j] = yr;
            self.gr_now = gr;
        }
        let (a, b) = self.hi.split_at_mut(1);
        self.os[0].downsample(&a[0][..n2], l);
        self.os[1].downsample(&b[0][..n2], r);
        if colour {
            let rr = self.dc_r;
            for (ch, buf) in [l, r].into_iter().enumerate() {
                let (mut x1, mut y1) = (self.dc_x[ch], self.dc_y[ch]);
                for s in buf.iter_mut() {
                    let x = *s;
                    y1 = x - x1 + rr * y1;
                    x1 = x;
                    *s = y1;
                }
                self.dc_x[ch] = x1;
                self.dc_y[ch] = y1;
            }
        }
    }
}

/// Kaiser-windowed sinc low-pass for the GR decimation (unity DC gain), designed at control rate `fc`.
fn gr_fir_taps(fc: f64) -> Vec<f32> {
    fn i0(x: f64) -> f64 {
        let (mut sum, mut term, q) = (1.0, 1.0, x * x / 4.0);
        for k in 1..64 {
            term *= q / (k * k) as f64;
            sum += term;
        }
        sum
    }
    let n = GR_FIR_LEN;
    let c = (n - 1) as f64 / 2.0;
    let wc = 2.0 * core::f64::consts::PI * GR_FIR_CUTOFF_HZ / fc;
    let h: Vec<f64> = (0..n)
        .map(|i| {
            let m = i as f64 - c;
            let sinc = if m == 0.0 {
                wc / core::f64::consts::PI
            } else {
                (wc * m).sin() / (core::f64::consts::PI * m)
            };
            let r = m / c;
            sinc * i0(GR_FIR_BETA * (1.0 - r * r).max(0.0).sqrt()) / i0(GR_FIR_BETA)
        })
        .collect();
    let sum: f64 = h.iter().sum();
    h.iter().map(|v| (v / sum) as f32).collect()
}

#[inline(always)]
fn dot(a: &[f32], b: &[f32]) -> f32 {
    let mut acc = [0.0f32; 4];
    let (ac, ar) = a.as_chunks::<4>();
    let (bc, br) = b[..a.len()].as_chunks::<4>();
    for (x, y) in ac.iter().zip(bc) {
        for l in 0..4 {
            acc[l] += x[l] * y[l];
        }
    }
    let mut s = (acc[0] + acc[2]) + (acc[1] + acc[3]);
    for (x, y) in ar.iter().zip(br) {
        s += x * y;
    }
    s
}

/// `exp(-rate)` for small per-sample rates (release coefficients), via the fast exp2.
#[inline(always)]
fn rate_coeff(rate: f32) -> f32 {
    wing_dsp::gain::fast_exp2(-rate * core::f32::consts::LOG2_E)
}

impl Model for FetComp {
    fn id(&self) -> &'static str {
        self.kind.id()
    }

    fn prepare(&mut self, sample_rate: f64, max_block: usize) {
        self.fs = sample_rate;
        self.max_block = max_block.max(1);
        let mb = self.max_block;
        self.os = [Oversampler2x::new(mb), Oversampler2x::new(mb)];
        self.pre = [vec![0.0; mb], vec![0.0; mb]];
        self.hi = [vec![0.0; OS * mb], vec![0.0; OS * mb]];
        self.kos2 = [Oversampler2x::new(OS * mb), Oversampler2x::new(OS * mb)];
        self.kmid = [vec![0.0; 2 * OS * mb], vec![0.0; 2 * OS * mb]];
        self.kos = [Oversampler4x::new(2 * OS * mb), Oversampler4x::new(2 * OS * mb)];
        self.khi = [vec![0.0; OS * CTRL_SUB * mb], vec![0.0; OS * CTRL_SUB * mb]];
        self.sc_os = [Oversampler2x::new(mb), Oversampler2x::new(mb)];
        self.sc_pre = [vec![0.0; mb], vec![0.0; mb]];
        self.sc_hi = [vec![0.0; OS * mb], vec![0.0; OS * mb]];
        self.gr_taps = gr_fir_taps(sample_rate * (OS * CTRL_SUB) as f64);
        self.gr_hist = vec![0.0; 2 * GR_FIR_LEN];
        self.gr_pos = 0;
        self.drive.set_time(GAIN_SMOOTH_S, sample_rate);
        self.out_g.set_time(GAIN_SMOOTH_S, sample_rate * OS as f64);
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
        let mb = self.max_block;
        let mut i = 0;
        while i < n {
            let e = (i + mb).min(n);
            let sc = sidechain.map(|(a, b)| {
                (
                    a.get(i..e.min(a.len())).unwrap_or(&[]),
                    b.get(i..e.min(b.len())).unwrap_or(&[]),
                )
            });
            self.process_chunk(&mut left[i..e], &mut right[i..e], sc);
            i = e;
        }
    }

    fn latency(&self) -> usize {
        self.os[0].latency() as usize + AUDIO_DELAY_OS / OS
    }

    fn reset(&mut self) {
        self.apply();
        for o in &mut self.os {
            o.reset();
        }
        self.gr = 0.0;
        self.hist = 0.0;
        self.ov_slow = 0.0;
        self.gr_now = 0.0;
        self.lp = [0.0; 2];
        for o in &mut self.kos {
            o.reset();
        }
        for o in &mut self.kos2 {
            o.reset();
        }
        for o in &mut self.sc_os {
            o.reset();
        }
        self.gr_hist.iter_mut().for_each(|v| *v = 0.0);
        self.adly = [[0.0; AUDIO_DELAY_OS]; 2];
        self.adly_pos = 0;
        self.dc_x = [0.0; 2];
        self.dc_y = [0.0; 2];
        self.drive.reset(self.drive.target());
        self.out_g.reset(self.out_g.target());
    }

    fn set_param(&mut self, slot: u16, value: ParamValue) {
        let Some(i) = (slot as usize).checked_sub(1).filter(|&i| i < 5) else {
            return;
        };
        let (lo, hi, _) = self.kind.slots()[i];
        let x = if i == 4 {
            match value {
                ParamValue::Float(f) => nearest_ratio(self.kind, f),
                other => {
                    let k = other.as_i32();
                    if k < 0 || k as usize >= self.kind.num_ratios() {
                        return;
                    }
                    k as f32
                }
            }
        } else {
            value.as_f32().clamp(lo, hi)
        };
        self.v[i] = x;
        self.dirty = true;
    }

    fn param(&self, slot: u16) -> Option<f32> {
        let i = (slot as usize).checked_sub(1)?;
        self.v.get(i).copied()
    }

    fn load_defaults(&mut self) {
        for (dst, s) in self.v.iter_mut().zip(self.kind.slots()) {
            *dst = s.2;
        }
        self.dirty = true;
    }

    fn commit(&mut self) {
        self.apply();
    }

    fn telemetry(&self) -> ModelTelemetry {
        ModelTelemetry {
            gain_reduction_db: self.gr_now,
            engaged: self.gr_now > 0.01,
        }
    }
}

/// Ratio index for a numeric ratio value (snapshot strings `"4"`, `"2:1"` parsed upstream to numbers);
/// values above the largest finite ratio select ALL (76LA) / NUKE (NSTR).
fn nearest_ratio(kind: FetKind, v: f32) -> f32 {
    let table: &[f32] = match kind {
        FetKind::La76 => &LA76_RATIOS,
        FetKind::Nstr => &NSTR_RATIOS[..7],
    };
    if v > 1.5 * table[table.len() - 1] {
        return table.len() as f32;
    }
    let mut best = 0;
    for (i, &r) in table.iter().enumerate() {
        if (r - v).abs() < (table[best] - v).abs() {
            best = i;
        }
    }
    best as f32
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fidelity::{run_mono, static_curve};

    const FS: f64 = 48_000.0;

    fn model(kind: FetKind, params: &[(u16, ParamValue)]) -> FetComp {
        let mut m = FetComp::new(kind);
        m.prepare(FS, 64);
        for &(s, v) in params {
            m.set_param(s, v);
        }
        m.commit();
        m.reset();
        m
    }

    fn tone(f: f64, amp: f64, n: usize) -> Vec<f32> {
        (0..n)
            .map(|i| (amp * (2.0 * core::f64::consts::PI * f * i as f64 / FS).sin()) as f32)
            .collect()
    }

    /// Single-bin amplitude of `x` at `f`, 4-term Blackman-Harris window (-92 dB sidelobes) so slow
    /// settling of the release history cannot leak into neighbouring 2 kHz lines.
    fn amp_at(x: &[f32], f: f64) -> f64 {
        use core::f64::consts::PI;
        let n = x.len() as f64;
        let w = 2.0 * PI * f / FS;
        let (mut re, mut im, mut wsum) = (0.0f64, 0.0f64, 0.0f64);
        for (i, &v) in x.iter().enumerate() {
            let t = 2.0 * PI * i as f64 / n;
            let win = 0.35875 - 0.48829 * t.cos() + 0.14128 * (2.0 * t).cos() - 0.01168 * (3.0 * t).cos();
            let p = w * i as f64;
            re += win * v as f64 * p.cos();
            im -= win * v as f64 * p.sin();
            wsum += win;
        }
        2.0 * (re * re + im * im).sqrt() / wsum
    }

    fn db(x: f64) -> f64 {
        20.0 * x.abs().max(1e-30).log10()
    }

    /// Worst non-harmonic line (dBc) for a 10 kHz input: the steady-state output is periodic in 24 samples
    /// (2 kHz grid); 10k / 20k are the in-band harmonics, every other 2 kHz line is aliasing.
    fn alias_dbc(m: &mut FetComp, amp: f64) -> (f64, f64) {
        let n_settle = 48_000;
        let n = 4800;
        let x = tone(10_000.0, amp, n_settle + n);
        let y = run_mono(m, &x);
        let tail = &y[n_settle..];
        let fund = amp_at(tail, 10_000.0);
        let mut worst = f64::NEG_INFINITY;
        for k in 1..12 {
            let f = 2000.0 * k as f64;
            if k == 5 || k == 10 {
                continue;
            }
            worst = worst.max(db(amp_at(tail, f) / fund));
            if std::env::var("FET_DBG").is_ok() {
                eprint!(" {}k:{:.0}", 2 * k, db(amp_at(tail, f) / fund));
            }
        }
        (worst, m.telemetry().gain_reduction_db as f64)
    }

    #[test]
    fn explore_alias() {
        for kind in FetKind::ALL {
            for ri in [0u32, 3] {
                for (att, rel) in [(7.0, 7.0), (7.0, 6.0), (7.0, 5.0), (6.0, 7.0), (6.5, 7.0), (0.0, 0.0), (0.0, 1.0)] {
                    let mut m = model(
                        kind,
                        &[
                            (3, ParamValue::Float(att)),
                            (4, ParamValue::Float(rel)),
                            (5, ParamValue::Enum(ri)),
                        ],
                    );
                    let (a, gr) = alias_dbc(&mut m, 0.5);
                    m.set_coloration(false);
                    m.reset();
                    let (a2, _) = alias_dbc(&mut m, 0.5);
                    eprintln!("{kind:?} r{ri} att{att} rel{rel}: alias {a:.1} dBc (nocol {a2:.1}), gr {gr:.1}");
                }
            }
        }
        let _ = static_curve;
    }
}
