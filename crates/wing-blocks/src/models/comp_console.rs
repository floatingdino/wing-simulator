//! Console / diode-bridge compressors (T04C): `ECL33` (EVEN COMP/LIM, Neve 33609), `E88C` (EVEN 88 COMP, Neve
//! 88RS channel compressor), `RED3` (RED3 COMPRESSOR, Focusrite Red 3), `2250` (PIA2250 RACK, API 225L /
//! 2500 family) and `LMT` (LMT COMPRESSOR, SSL 4000E listen-mic compressor "LMC+"-style with a transient
//! section). Parameters, slot order, ranges and defaults: `docs/research/models.md` §3.2 and
//! `data/schema/wing-tree.json`.
//!
//! All five run on one gain-computer core ([`Stage`], stereo-linked: one gain from the louder channel):
//! ```text
//! key -> detector (PEAK: x^2 | RMS: one-pole mean square) -> level dB
//!     -> static curve (thr, ratio, hard / quadratic soft knee)          feed-forward (FF)
//!        or loop gain k on the *output* level (k = R - 1)               feedback (FB)
//!     -> gain-reduction ballistics in dB (attack / release one-poles, optional two-stage programme-dependent
//!        release as in `comp_vca`) -> gain cell
//! ```
//! A feedback stage with loop gain `k` settles to `GR = k / (1 + k) * (L - T)`, i.e. exactly the dialled ratio
//! `R = k + 1`; its closed-loop attack time constant is `tau / (1 + k)`, so the internal attack is scaled by
//! `1 + k` to reproduce the published attack times (release mostly runs with the loop open, below threshold,
//! and is not scaled).
//!
//! Per model (constants below cite their sources; the T04C task Log lists the references):
//! - **ECL33** 33609: feedback diode-bridge compressor (ratios 1.5..6, soft knee) followed by a feed-forward
//!   limiter (ratio > 100:1), each with its own attack (comp 3 / 6 ms, limiter 2 / 4 ms, AMS Neve spec) and
//!   recovery (comp 100..1500 ms, limiter 50..800 ms, A1 = 100 ms / 2 s, A2 = 50 ms / 5 s programme-dependent).
//!   Diode-bridge colouration: odd-order distortion proportional to gain reduction, bounded by the published
//!   THD (< 0.2 % compressing, < 0.45 % limiting). Make-up `cgain` is part of the model.
//! - **E88C** 88RS: feed-forward peak compressor, SOFT / HARD knee, threshold +20..-10 dBu (THR -20 pull),
//!   ratio 1..10 with the top end = limit, FAST / SLOW attack, release 10..3000 ms with the top end = AUTO.
//! - **RED3** Red 3: feed-forward VCA compressor with a soft knee, attack 1..50 ms, release 0.1..4 s, AUTO
//!   (programme-dependent release).
//! - **2250** API 225L: threshold knob 0..10 = +10..-20 dBu, ratio knob 0..10 = 1:1..infinity, attack switch
//!   FAST / MED / SLOW = 2 / 18 / 75 ms, release 50..3000 ms, HARD / SOFT knee, TYPE NEW = feed-forward with an
//!   RMS detector, OLD = feedback (detector after the gain cell, peak).
//! - **LMT** LMC-style: transient shaper (`ton`, speed, amount, gain) then a fixed-ratio, very fast
//!   feed-forward compressor whose `comp` amount lowers the threshold, with its own make-up `cgain`.
//!
//! Threshold calibration: the dBu-labelled controls (E88C, 2250) use `comp_vca::DBU_AT_0DBFS` (0 dBu = -18
//! dBFS); ECL33 / RED3 thresholds are taken as dBFS (WING's ranges are already offset from the hardware's dBu
//! ranges, see the Log).
//!
//! GR telemetry is in dB; [`gr_meter`] reproduces WING-Edit's GR-meter mapping (`comp_gain_<m>`,
//! `data/reference-curves/dyn-models.json`: `GR / 20 dB`, capped at 1.0 for all five).
//!
//! Real-time: no allocation; `set_param` stages values, `commit`/`process`/`reset` apply them, `process` is
//! per-sample arithmetic with the fast dB helpers of `wing_dsp::gain`.

use crate::model::{Model, ModelTelemetry, ParamValue};
use crate::models::comp_vca::{
    AUTO_CHARGE_S, AUTO_FAST_RELEASE_S, AUTO_SLOW_RELEASE_S, DBU_AT_0DBFS, GR_MAX_DB,
};
use wing_dsp::gain::{db_to_lin_fast, lin_to_db_fast};
use wing_dsp::smooth::one_pole_coeff;

/// Detector power floor (-240 dB).
const POWER_FLOOR: f32 = 1e-24;

// ---- ECL33 (Neve 33609) -----------------------------------------------------------------------------------

/// 33609 compressor ratios (schema `ratio`, = hardware 1.5/2/3/4/6).
pub const ECL33_RATIOS: [f32; 5] = [1.5, 2.0, 3.0, 4.0, 6.0];
/// 33609 compressor attack, slow / fast (AMS Neve 33609 spec: "Slow 6 ms +/-1, Fast 3 ms +/-1").
pub const ECL33_COMP_ATTACK_S: [f64; 2] = [0.006, 0.003];
/// 33609 limiter attack, slow / fast (AMS Neve 33609 spec: "Slow 4 ms +/-1, Fast 2 ms +/-1").
pub const ECL33_LIM_ATTACK_S: [f64; 2] = [0.004, 0.002];
/// Compressor recovery `crec` 100 / 400 / 800 / 1500 ms (schema; AMS spec 100 ms..1.5 s).
pub const ECL33_COMP_REC_S: [f64; 4] = [0.1, 0.4, 0.8, 1.5];
/// Limiter recovery `lrec` 50 / 100 / 200 / 800 ms (schema; AMS spec 50..800 ms).
pub const ECL33_LIM_REC_S: [f64; 4] = [0.05, 0.1, 0.2, 0.8];
/// Auto recovery (fast, slow) of A1 / A2 (AMS spec: "A1 100 ms / 2 s, A2 50 ms / 5 s", both sections).
pub const ECL33_AUTO_S: [(f64, f64); 2] = [(0.1, 2.0), (0.05, 5.0)];
/// Limiter ratio: AMS spec "output change < 0.1 dB for a 10 dB input step above threshold" => R > 100.
pub const ECL33_LIM_RATIO: f32 = 200.0;
/// Diode-bridge soft-knee width (dB, quadratic knee centred on the threshold). Assumption: unpublished.
pub const ECL33_KNEE_DB: f32 = 6.0;
/// Diode-bridge colouration: cubic coefficient per dB of gain reduction (`y = u - h u^3`, `h = c * GR`).
/// Calibrated so a -6 dBFS-peak output at 20 dB GR has 0.1 % THD, inside the AMS spec ceilings (< 0.2 %
/// compressing at 20 dB, < 0.45 % limiting). Assumption: harmonic order and level dependence unpublished.
pub const ECL33_COLOUR_PER_DB: f32 = 0.0008;

// ---- E88C (Neve 88RS) -------------------------------------------------------------------------------------

/// 88RS FAST attack (Neve 88R dynamics: "extra fast attack circuit, 0.1 ms"). SLOW: see below.
pub const E88C_FAST_ATTACK_S: f64 = 0.0001;
/// 88RS SLOW attack. Assumption (Neve: "adaptive", no figure published).
pub const E88C_SLOW_ATTACK_S: f64 = 0.010;
/// Ratio at/above which the 88RS ratio pot is at its "LIM" end (schema max 10).
pub const E88C_LIM_RATIO: f32 = 9.99;
/// Release values at/above this are the AUTO end of the release pot (schema max 3000 ms).
pub const E88C_AUTO_REL_MS: f32 = 2999.0;
/// THR -20 pull: shifts the threshold range by -20 dB (models.md, 88R: +20..-30 dBu overall).
pub const E88C_THR_PULL_DB: f32 = -20.0;
/// SOFT knee width (dB). Assumption.
pub const E88C_KNEE_DB: f32 = 10.0;

// ---- RED3 (Focusrite Red 3) -------------------------------------------------------------------------------

/// Red 3 ratios (schema fenum).
pub const RED3_RATIOS: [f32; 13] = [
    1.1, 1.2, 1.3, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 5.0, 6.0, 8.0, 10.0,
];
/// Red 3 soft-knee width (dB); Focusrite describes a "soft knee" compressor, width unpublished (assumption).
pub const RED3_KNEE_DB: f32 = 10.0;

// ---- 2250 (API 225L) --------------------------------------------------------------------------------------

/// Attack switch FAST / MED / SLOW (API 225L: "Fast (2 mS), Medium (18 mS) or Slow (75 mS)").
pub const API_ATTACK_S: [f64; 3] = [0.002, 0.018, 0.075];
/// Threshold knob end points (API 225L: "Threshold control from +10 to -20 dBu"); knob 0 = +10 dBu.
pub const API_THR_TOP_DBU: f32 = 10.0;
pub const API_THR_BOTTOM_DBU: f32 = -20.0;
/// Soft-knee width (dB). Assumption (API: "SOFT ... more subtle compression"; width unpublished).
pub const API_KNEE_DB: f32 = 10.0;
/// NEW (feed-forward) RMS averaging time (API: "rely on RMS detectors"). Assumption: window unpublished;
/// 1 ms keeps the detector from slowing the published 2 ms FAST attack (5 ms measured 3.2 ms).
pub const API_RMS_TAU_S: f64 = 0.001;
/// Largest loop gain of the OLD (feedback) path, i.e. maximum OLD ratio 31:1 at the infinity end of the
/// ratio knob. Assumption: a feedback loop cannot reach infinity:1.
pub const API_FB_MAX_K: f32 = 30.0;

// ---- LMT (SSL LMC-style) ----------------------------------------------------------------------------------

/// LMC fixed ratio. Assumption (the 4000E LMC is "very high ratio"; no figure published).
pub const LMT_RATIO: f32 = 20.0;
/// LMC fixed attack / release (SSL LMC: "fixed attack and release"; values unpublished, assumptions).
pub const LMT_ATTACK_S: f64 = 0.0003;
pub const LMT_RELEASE_S: f64 = 0.15;
/// `comp` 0..100 lowers the threshold from 0 to -40 dBFS (assumption: amount = drive into the fixed curve).
pub const LMT_THR_DB_PER_UNIT: f32 = -0.4;
/// Transient shaper: fast follower attack; slow follower attack spans `[slow, fast]` over T SPEED 0..1;
/// both release with [`LMT_TR_RELEASE_S`]. Shaping gain = `trans * 0.1 * (fast_db - slow_db)` (trans +10
/// doubles the transient's dB excess, -10 removes it), clamped to +/-[`LMT_TR_MAX_DB`]. Assumptions.
pub const LMT_TR_FAST_ATTACK_S: f64 = 0.0002;
pub const LMT_TR_SLOW_ATTACK_S: [f64; 2] = [0.050, 0.002];
pub const LMT_TR_RELEASE_S: f64 = 0.1;
pub const LMT_TR_MAX_DB: f32 = 18.0;

/// Which of the five models an instance is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConsoleKind {
    Ecl33,
    E88c,
    Red3,
    Api2250,
    Lmt,
}

impl ConsoleKind {
    pub const ALL: [ConsoleKind; 5] = [
        ConsoleKind::Ecl33,
        ConsoleKind::E88c,
        ConsoleKind::Red3,
        ConsoleKind::Api2250,
        ConsoleKind::Lmt,
    ];

    /// Snapshot `mdl` token.
    pub fn id(self) -> &'static str {
        match self {
            ConsoleKind::Ecl33 => "ECL33",
            ConsoleKind::E88c => "E88C",
            ConsoleKind::Red3 => "RED3",
            ConsoleKind::Api2250 => "2250",
            ConsoleKind::Lmt => "LMT",
        }
    }

    pub fn from_id(id: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|k| k.id() == id)
    }

    fn slots(self) -> &'static [SlotDef] {
        match self {
            ConsoleKind::Ecl33 => &ECL33_SLOTS,
            ConsoleKind::E88c => &E88C_SLOTS,
            ConsoleKind::Red3 => &RED3_SLOTS,
            ConsoleKind::Api2250 => &API_SLOTS,
            ConsoleKind::Lmt => &LMT_SLOTS,
        }
    }
}

/// WING-Edit GR-meter position (`comp_gain_<m>`, `dyn-models.json`): `GR / 20 dB` capped at 1.0 for all five.
pub fn gr_meter(_kind: ConsoleKind, gr_db: f32) -> f32 {
    (gr_db.max(0.0) / 20.0).min(1.0)
}

// ---- slots ------------------------------------------------------------------------------------------------

#[derive(Clone, Copy, Debug)]
enum SlotKind {
    /// Continuous (clamped to `[min(lo,hi), max(lo,hi)]`).
    Range(f32, f32),
    /// Float enum: `Enum(i)` = table index, `Float(v)` snaps to the nearest entry.
    Table(&'static [f32]),
    /// Plain enum stored as its index (`0..n`).
    Index(u32),
    Bool,
}

#[derive(Clone, Copy, Debug)]
struct SlotDef {
    kind: SlotKind,
    default: f32,
}

const fn range(lo: f32, hi: f32, default: f32) -> SlotDef {
    SlotDef {
        kind: SlotKind::Range(lo, hi),
        default,
    }
}
const fn table(t: &'static [f32], default: f32) -> SlotDef {
    SlotDef {
        kind: SlotKind::Table(t),
        default,
    }
}
const fn index(n: u32, default: u32) -> SlotDef {
    SlotDef {
        kind: SlotKind::Index(n),
        default: default as f32,
    }
}
const fn boolean(default: bool) -> SlotDef {
    SlotDef {
        kind: SlotKind::Bool,
        default: if default { 1.0 } else { 0.0 },
    }
}

/// `lon lthr lrec lfast con cthr ratio crec cfast cgain` (rec enums: 4 = A1, 5 = A2).
const ECL33_SLOTS: [SlotDef; 10] = [
    boolean(false),
    range(-12.0, 0.0, -5.0),
    index(6, 4),
    boolean(false),
    boolean(true),
    range(-35.0, -5.0, -15.0),
    table(&ECL33_RATIOS, 3.0),
    index(6, 4),
    boolean(false),
    range(-6.0, 12.0, 0.0),
];
/// `knee(SOFT,HARD) thr thrpull ratio att(SLOW,FAST) rel`.
const E88C_SLOTS: [SlotDef; 6] = [
    index(2, 0),
    range(20.0, -10.0, 0.0),
    boolean(false),
    range(1.0, 10.0, 3.0),
    index(2, 0),
    range(10.0, 3000.0, 353.0),
];
/// `thr ratio att rel auto`.
const RED3_SLOTS: [SlotDef; 5] = [
    range(-48.0, 0.0, -12.0),
    table(&RED3_RATIOS, 3.0),
    range(1.0, 50.0, 20.1),
    range(100.0, 4000.0, 399.0),
    boolean(false),
];
/// `thr ratio att(FAST,MED,SLOW) rel knee(HARD,SOFT) type(OLD,NEW)`.
const API_SLOTS: [SlotDef; 6] = [
    range(0.0, 10.0, 2.0),
    range(0.0, 10.0, 3.0),
    index(3, 1),
    range(50.0, 3000.0, 202.0),
    index(2, 1),
    index(2, 1),
];
/// `tspd trans tgain ton comp cgain con`.
const LMT_SLOTS: [SlotDef; 7] = [
    range(0.0, 1.0, 0.5),
    range(-10.0, 10.0, 0.0),
    range(-12.0, 12.0, 0.0),
    boolean(false),
    range(0.0, 100.0, 10.0),
    range(-12.0, 12.0, 0.0),
    boolean(true),
];
const MAX_SLOTS: usize = 10;

// ---- shared gain-computer core ----------------------------------------------------------------------------

/// Gain-reduction amount above the knee (dB of "overshoot"): hard knee or quadratic soft knee of width `w`
/// centred on the threshold.
#[inline]
fn knee_over(d: f32, w: f32) -> f32 {
    if w <= 0.0 {
        d.max(0.0)
    } else if d <= -0.5 * w {
        0.0
    } else if d >= 0.5 * w {
        d
    } else {
        let t = d + 0.5 * w;
        t * t / (2.0 * w)
    }
}

/// One compressor / limiter stage: detector, static curve (FF) or loop gain (FB), ballistics.
#[derive(Clone, Copy, Debug)]
struct Stage {
    thr_db: f32,
    /// FF: `1 - 1/R`; FB: loop gain `k = R - 1`.
    g: f32,
    knee_db: f32,
    feedback: bool,
    rms: bool,
    rms_c: f32,
    att_c: f32,
    rel_c: f32,
    auto: bool,
    charge_c: f32,
    slow_rel_c: f32,
    // state
    ms: [f32; 2],
    gr_fast: f32,
    gr_slow: f32,
}

impl Stage {
    const fn new() -> Self {
        Self {
            thr_db: 0.0,
            g: 0.0,
            knee_db: 0.0,
            feedback: false,
            rms: false,
            rms_c: 0.0,
            att_c: 0.0,
            rel_c: 0.0,
            auto: false,
            charge_c: 0.0,
            slow_rel_c: 0.0,
            ms: [POWER_FLOOR; 2],
            gr_fast: 0.0,
            gr_slow: 0.0,
        }
    }

    /// Static curve `ratio` (>= 1, `f32::INFINITY` allowed) as FF slope or FB loop gain (capped).
    fn set_ratio(&mut self, ratio: f32, feedback: bool, max_k: f32) {
        self.feedback = feedback;
        self.g = if feedback {
            (ratio - 1.0).clamp(0.0, max_k)
        } else if ratio.is_finite() {
            1.0 - 1.0 / ratio.max(1.0)
        } else {
            1.0
        };
    }

    /// Ballistics: attack / release time constants (s); the FB attack is scaled by `1 + k` (see module doc).
    /// `auto = Some((fast_rel, slow_rel, charge))` enables the two-stage programme-dependent release.
    fn set_times(&mut self, fs: f64, att_s: f64, rel_s: f64, auto: Option<(f64, f64, f64)>) {
        let c = |tau: f64| one_pole_coeff(tau, fs) as f32;
        let att = if self.feedback {
            att_s * (1.0 + self.g as f64)
        } else {
            att_s
        };
        self.att_c = c(att);
        match auto {
            Some((fast, slow, charge)) => {
                self.auto = true;
                self.rel_c = c(fast);
                self.slow_rel_c = c(slow);
                self.charge_c = c(charge);
            }
            None => {
                self.auto = false;
                self.rel_c = c(rel_s);
                self.gr_slow = 0.0;
            }
        }
    }

    fn reset(&mut self) {
        self.ms = [POWER_FLOOR; 2];
        self.gr_fast = 0.0;
        self.gr_slow = 0.0;
    }

    /// Target gain reduction (dB) for a detected level (dB): FF static curve or FB loop law.
    #[inline]
    fn target(&self, level_db: f32) -> f32 {
        (self.g * knee_over(level_db - self.thr_db, self.knee_db)).min(GR_MAX_DB)
    }

    /// Detected level (dB) of a stereo key sample (linked: louder side).
    #[inline]
    fn level_db(&mut self, kl: f32, kr: f32) -> f32 {
        let (pl, pr) = (kl * kl, kr * kr);
        let p = if self.rms {
            let c = self.rms_c;
            self.ms[0] = (pl + c * (self.ms[0] - pl)).max(POWER_FLOOR);
            self.ms[1] = (pr + c * (self.ms[1] - pr)).max(POWER_FLOOR);
            self.ms[0].max(self.ms[1])
        } else {
            pl.max(pr).max(POWER_FLOOR)
        };
        0.5 * lin_to_db_fast(p)
    }

    #[inline]
    fn ballistics(&mut self, target: f32) -> f32 {
        let f = self.gr_fast;
        let c = if target > f { self.att_c } else { self.rel_c };
        let f = target + c * (f - target);
        self.gr_fast = f;
        if self.auto {
            let s = self.gr_slow;
            let c2 = if f > s {
                self.charge_c
            } else {
                self.slow_rel_c
            };
            let s = f + c2 * (s - f);
            self.gr_slow = s;
            f.max(s)
        } else {
            f
        }
    }

    /// Static output (dB) of this stage for a steady input level (dB).
    fn static_out(&self, in_db: f32) -> f32 {
        if !self.feedback {
            return in_db - self.target(in_db);
        }
        // Solve out = in - k * knee(out - T) (monotonic in out) by bisection.
        let (mut lo, mut hi) = (in_db - GR_MAX_DB, in_db);
        for _ in 0..60 {
            let mid = 0.5 * (lo + hi);
            if mid + self.target(mid) > in_db {
                hi = mid;
            } else {
                lo = mid;
            }
        }
        0.5 * (lo + hi)
    }
}

/// Transient shaper of LMT (two envelope followers, dB difference drives the gain).
#[derive(Clone, Copy, Debug)]
struct Transient {
    fast_att_c: f32,
    slow_att_c: f32,
    rel_c: f32,
    amount: f32,
    fast: f32,
    slow: f32,
}

impl Transient {
    const fn new() -> Self {
        Self {
            fast_att_c: 0.0,
            slow_att_c: 0.0,
            rel_c: 0.0,
            amount: 0.0,
            fast: 0.0,
            slow: 0.0,
        }
    }

    #[inline]
    fn follow(env: f32, x: f32, att_c: f32, rel_c: f32) -> f32 {
        let c = if x > env { att_c } else { rel_c };
        x + c * (env - x)
    }

    /// Shaping gain (dB) for the rectified, linked input sample `a`.
    #[inline]
    fn gain_db(&mut self, a: f32) -> f32 {
        self.fast = Self::follow(self.fast, a, self.fast_att_c, self.rel_c);
        self.slow = Self::follow(self.slow, a, self.slow_att_c, self.rel_c);
        let d = lin_to_db_fast(self.fast.max(1e-12)) - lin_to_db_fast(self.slow.max(1e-12));
        (self.amount * 0.1 * d).clamp(-LMT_TR_MAX_DB, LMT_TR_MAX_DB)
    }
}

/// One of the five console compressor models (stereo-linked).
pub struct ConsoleComp {
    kind: ConsoleKind,
    fs: f64,
    /// Staged slot values (engineering units; enums as index, bools 0/1).
    v: [f32; MAX_SLOTS],
    dirty: bool,
    comp_on: bool,
    lim_on: bool,
    comp: Stage,
    lim: Stage,
    /// Make-up after the compressor / limiter (linear; ECL33 `cgain`, LMT `cgain`).
    makeup: f32,
    colour_per_db: f32,
    tr_on: bool,
    tr_gain: f32,
    tr: Transient,
    /// Previous key-through-gain-cell sample (feedback detector input).
    fb_prev: [f32; 2],
    gr_now: f32,
}

impl ConsoleComp {
    pub fn new(kind: ConsoleKind) -> Self {
        let mut m = Self {
            kind,
            fs: 48_000.0,
            v: [0.0; MAX_SLOTS],
            dirty: true,
            comp_on: true,
            lim_on: false,
            comp: Stage::new(),
            lim: Stage::new(),
            makeup: 1.0,
            colour_per_db: 0.0,
            tr_on: false,
            tr_gain: 1.0,
            tr: Transient::new(),
            fb_prev: [0.0; 2],
            gr_now: 0.0,
        };
        m.load_defaults();
        m.apply();
        m
    }

    pub fn kind(&self) -> ConsoleKind {
        self.kind
    }

    /// Effective compressor threshold in dBFS.
    pub fn threshold_dbfs(&mut self) -> f32 {
        self.apply();
        self.comp.thr_db
    }

    /// Static output level (dBFS) for a steady input level (dBFS): transient section at rest, compressor,
    /// limiter, make-up; no colouration. The law the DSP settles to for DC input.
    pub fn static_out_db(&mut self, in_db: f32) -> f32 {
        self.apply();
        let mut l = in_db;
        if self.tr_on {
            l += lin_to_db_fast(self.tr_gain);
        }
        if self.comp_on {
            l = self.comp.static_out(l);
        }
        if self.lim_on {
            l = self.lim.static_out(l);
        }
        l + 20.0 * self.makeup.log10()
    }

    fn apply(&mut self) {
        if !self.dirty {
            return;
        }
        self.dirty = false;
        let fs = self.fs;
        let v = self.v;
        let dbu = |x: f32| x - DBU_AT_0DBFS;
        self.lim_on = false;
        self.comp_on = true;
        self.makeup = 1.0;
        self.colour_per_db = 0.0;
        self.tr_on = false;
        self.tr_gain = 1.0;
        let vca_auto = Some((AUTO_FAST_RELEASE_S, AUTO_SLOW_RELEASE_S, AUTO_CHARGE_S));
        match self.kind {
            ConsoleKind::Ecl33 => {
                // Limiter: feed-forward peak, ratio > 100:1, diode knee.
                self.lim_on = v[0] != 0.0;
                let l = &mut self.lim;
                l.thr_db = v[1];
                l.knee_db = ECL33_KNEE_DB;
                l.rms = false;
                l.set_ratio(ECL33_LIM_RATIO, false, 0.0);
                let (att, rec, auto) = ecl33_times(
                    v[3] != 0.0,
                    v[2] as usize,
                    &ECL33_LIM_ATTACK_S,
                    &ECL33_LIM_REC_S,
                );
                l.set_times(fs, att, rec, auto);
                // Compressor: feedback diode bridge.
                self.comp_on = v[4] != 0.0;
                let c = &mut self.comp;
                c.thr_db = v[5];
                c.knee_db = ECL33_KNEE_DB;
                c.rms = false;
                c.set_ratio(v[6], true, f32::MAX);
                let (att, rec, auto) = ecl33_times(
                    v[8] != 0.0,
                    v[7] as usize,
                    &ECL33_COMP_ATTACK_S,
                    &ECL33_COMP_REC_S,
                );
                c.set_times(fs, att, rec, auto);
                self.makeup = db_to_lin_fast(v[9]);
                self.colour_per_db = ECL33_COLOUR_PER_DB;
            }
            ConsoleKind::E88c => {
                let c = &mut self.comp;
                c.knee_db = if v[0] == 0.0 { E88C_KNEE_DB } else { 0.0 };
                c.thr_db = dbu(v[1] + if v[2] != 0.0 { E88C_THR_PULL_DB } else { 0.0 });
                let r = if v[3] >= E88C_LIM_RATIO {
                    f32::INFINITY
                } else {
                    v[3]
                };
                c.set_ratio(r, false, 0.0);
                c.rms = false;
                let att = if v[4] != 0.0 {
                    E88C_FAST_ATTACK_S
                } else {
                    E88C_SLOW_ATTACK_S
                };
                let auto = (v[5] >= E88C_AUTO_REL_MS).then_some(vca_auto).flatten();
                c.set_times(fs, att, v[5] as f64 * 1e-3, auto);
            }
            ConsoleKind::Red3 => {
                let c = &mut self.comp;
                c.thr_db = v[0];
                c.knee_db = RED3_KNEE_DB;
                c.set_ratio(v[1], false, 0.0);
                c.rms = false;
                let auto = (v[4] != 0.0).then_some(vca_auto).flatten();
                c.set_times(fs, v[2] as f64 * 1e-3, v[3] as f64 * 1e-3, auto);
            }
            ConsoleKind::Api2250 => {
                let c = &mut self.comp;
                c.thr_db = dbu(api_threshold_dbu(v[0]));
                c.knee_db = if v[4] == 0.0 { 0.0 } else { API_KNEE_DB };
                let old = v[5] == 0.0;
                c.set_ratio(api_ratio(v[1]), old, API_FB_MAX_K);
                c.rms = !old;
                c.rms_c = one_pole_coeff(API_RMS_TAU_S, fs) as f32;
                let att = API_ATTACK_S[(v[2] as usize).min(2)];
                c.set_times(fs, att, v[3] as f64 * 1e-3, None);
            }
            ConsoleKind::Lmt => {
                let t = &mut self.tr;
                let spd = v[0].clamp(0.0, 1.0) as f64;
                let [slow0, slow1] = LMT_TR_SLOW_ATTACK_S;
                let slow_att = slow0 * (slow1 / slow0).powf(spd);
                t.fast_att_c = one_pole_coeff(LMT_TR_FAST_ATTACK_S, fs) as f32;
                t.slow_att_c = one_pole_coeff(slow_att, fs) as f32;
                t.rel_c = one_pole_coeff(LMT_TR_RELEASE_S, fs) as f32;
                t.amount = v[1];
                self.tr_on = v[3] != 0.0;
                self.tr_gain = db_to_lin_fast(v[2]);
                self.comp_on = v[6] != 0.0;
                let c = &mut self.comp;
                c.thr_db = v[4] * LMT_THR_DB_PER_UNIT;
                c.knee_db = 0.0;
                c.rms = false;
                c.set_ratio(LMT_RATIO, false, 0.0);
                c.set_times(fs, LMT_ATTACK_S, LMT_RELEASE_S, None);
                if self.comp_on {
                    self.makeup = db_to_lin_fast(v[5]);
                }
            }
        }
        if !self.comp.feedback {
            self.fb_prev = [0.0; 2];
        }
    }
}

/// ECL33 attack / recovery selection: `(attack_s, release_s, auto)`; rec index 4 = A1, 5 = A2.
fn ecl33_times(
    fast: bool,
    rec: usize,
    att: &[f64; 2],
    recs: &[f64; 4],
) -> (f64, f64, Option<(f64, f64, f64)>) {
    let a = att[fast as usize];
    match recs.get(rec) {
        Some(&r) => (a, r, None),
        None => {
            let (f, s) = ECL33_AUTO_S[(rec - recs.len()).min(1)];
            (a, f, Some((f, s, AUTO_CHARGE_S)))
        }
    }
}

/// 2250 threshold knob 0..10 -> dBu (linear from +10 dBu at 0 to -20 dBu at 10). Assumption: knob law.
pub fn api_threshold_dbu(knob: f32) -> f32 {
    API_THR_TOP_DBU + (API_THR_BOTTOM_DBU - API_THR_TOP_DBU) * knob.clamp(0.0, 10.0) / 10.0
}

/// 2250 ratio knob 0..10 -> ratio: slope `1 - 1/R = knob / 10` (1:1 at 0, 2:1 at 5, 10:1 at 9, infinity at
/// 10; API: "variable compression ratio from 1 to infinity"). Assumption: knob law.
pub fn api_ratio(knob: f32) -> f32 {
    let s = knob.clamp(0.0, 10.0) / 10.0;
    if s >= 1.0 {
        f32::INFINITY
    } else {
        1.0 / (1.0 - s)
    }
}

impl Model for ConsoleComp {
    fn id(&self) -> &'static str {
        self.kind.id()
    }

    fn prepare(&mut self, sample_rate: f64, _max_block: usize) {
        self.fs = sample_rate;
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
            let (mut xl, mut xr) = (left[i], right[i]);
            if self.tr_on {
                let gdb = self.tr.gain_db(xl.abs().max(xr.abs()));
                let g = db_to_lin_fast(gdb) * self.tr_gain;
                xl *= g;
                xr *= g;
            }
            let (kl, kr) = match sidechain {
                Some((a, b)) => (
                    a.get(i).copied().unwrap_or(0.0),
                    b.get(i).copied().unwrap_or(0.0),
                ),
                None => (xl, xr),
            };
            let mut total = 0.0;
            if self.comp_on {
                let level = if self.comp.feedback {
                    self.comp.level_db(self.fb_prev[0], self.fb_prev[1])
                } else {
                    self.comp.level_db(kl, kr)
                };
                let gr = self.comp.ballistics(self.comp.target(level));
                let g = db_to_lin_fast(-gr);
                xl *= g;
                xr *= g;
                self.fb_prev = [kl * g, kr * g];
                total = gr;
            }
            if self.lim_on {
                let level = self.lim.level_db(xl, xr);
                let gr = self.lim.ballistics(self.lim.target(level));
                let g = db_to_lin_fast(-gr);
                xl *= g;
                xr *= g;
                total += gr;
            }
            if self.colour_per_db > 0.0 {
                let h = self.colour_per_db * total;
                let (cl, cr) = (xl.clamp(-2.0, 2.0), xr.clamp(-2.0, 2.0));
                xl -= h * cl * cl * cl;
                xr -= h * cr * cr * cr;
            }
            left[i] = xl * self.makeup;
            right[i] = xr * self.makeup;
            self.gr_now = total;
        }
    }

    fn reset(&mut self) {
        self.apply();
        self.comp.reset();
        self.lim.reset();
        self.tr.fast = 0.0;
        self.tr.slow = 0.0;
        self.fb_prev = [0.0; 2];
        self.gr_now = 0.0;
    }

    fn set_param(&mut self, slot: u16, value: ParamValue) {
        let slots = self.kind.slots();
        let Some(def) = (slot as usize).checked_sub(1).and_then(|i| slots.get(i)) else {
            return;
        };
        let x = match def.kind {
            SlotKind::Range(lo, hi) => value.as_f32().clamp(lo.min(hi), lo.max(hi)),
            SlotKind::Table(t) => match value {
                ParamValue::Enum(i) => match t.get(i as usize) {
                    Some(&r) => r,
                    None => return,
                },
                other => {
                    let v = other.as_f32();
                    let mut best = t[0];
                    for &r in t {
                        if (r - v).abs() < (best - v).abs() {
                            best = r;
                        }
                    }
                    best
                }
            },
            SlotKind::Index(n) => value.as_i32().clamp(0, n as i32 - 1) as f32,
            SlotKind::Bool => value.as_bool() as u32 as f32,
        };
        self.v[slot as usize - 1] = x;
        self.dirty = true;
    }

    fn param(&self, slot: u16) -> Option<f32> {
        let i = (slot as usize).checked_sub(1)?;
        self.kind.slots().get(i).map(|_| self.v[i])
    }

    fn load_defaults(&mut self) {
        for (dst, def) in self.v.iter_mut().zip(self.kind.slots()) {
            *dst = def.default;
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fidelity::{run_mono, static_curve};

    const FS: f64 = 48_000.0;

    fn model(kind: ConsoleKind, params: &[(u16, ParamValue)]) -> ConsoleComp {
        let mut m = ConsoleComp::new(kind);
        m.prepare(FS, 64);
        for &(s, v) in params {
            m.set_param(s, v);
        }
        m.commit();
        m.reset();
        m
    }

    fn db(x: f64) -> f64 {
        20.0 * x.abs().max(1e-30).log10()
    }

    fn lin(db: f64) -> f32 {
        10f64.powf(db / 20.0) as f32
    }

    /// Gain (dB) trajectory after a DC step `lo_db -> hi_db` (after settling `settle_s` at `lo_db`).
    fn step_gain(
        m: &mut ConsoleComp,
        lo_db: f64,
        hi_db: f64,
        settle_s: f64,
        after_s: f64,
    ) -> Vec<f64> {
        let n0 = (settle_s * FS) as usize;
        let n1 = (after_s * FS) as usize;
        let mut x = vec![lin(lo_db); n0 + n1];
        x[n0..].fill(lin(hi_db));
        m.reset();
        let y = run_mono(m, &x);
        y[n0..]
            .iter()
            .zip(&x[n0..])
            .map(|(&o, &i)| db((o / i) as f64))
            .collect()
    }

    /// Time (ms) until the gain trajectory has covered `frac` of its total change from `g0`.
    fn time_to(g: &[f64], g0: f64, frac: f64) -> f64 {
        let d = g.last().unwrap() - g0;
        let i = g.iter().position(|&v| (v - g0) / d >= frac).unwrap();
        i as f64 / FS * 1e3
    }

    /// Time (ms) for GR to fall below `gr_db` after `burst_s` at `hi_db` followed by `lo_db`.
    fn recovery_ms(m: &mut ConsoleComp, hi_db: f64, lo_db: f64, burst_s: f64, gr_db: f64) -> f64 {
        let n0 = (burst_s * FS) as usize;
        let n1 = (12.0 * FS) as usize;
        let mut x = vec![lin(lo_db); n0 + n1];
        x[..n0].fill(lin(hi_db));
        m.reset();
        let y = run_mono(m, &x);
        let i = (n0..n0 + n1)
            .position(|k| db((y[k] / x[k]) as f64) > -gr_db)
            .unwrap();
        i as f64 / FS * 1e3
    }

    fn assert_static(m: &mut ConsoleComp, levels: &[f64], law: impl Fn(f64) -> f64, tol: f64) {
        let meas = static_curve(m, FS, levels, 1.0);
        for (&l, &o) in levels.iter().zip(&meas) {
            let e = law(l);
            assert!((o - e).abs() < tol, "{:?} in {l}: {o} vs {e}", m.kind());
        }
    }

    #[test]
    fn ecl33_feedback_compressor_reaches_published_ratios() {
        // Compressor only (limiter off by default), thr -20 dBFS; hard-knee asymptote checked away from the
        // 6 dB diode knee.
        for (i, &r) in ECL33_RATIOS.iter().enumerate() {
            let mut m = model(
                ConsoleKind::Ecl33,
                &[
                    (6, ParamValue::Float(-20.0)),
                    (7, ParamValue::Enum(i as u32)),
                ],
            );
            let r = r as f64;
            let law = |l: f64| if l > -20.0 { -20.0 + (l + 20.0) / r } else { l };
            // Static curve law (bisection) vs the published ratio, far from the knee.
            for l in [-40.0f32, 0.0, 10.0] {
                assert!(
                    (m.static_out_db(l) as f64 - law(l as f64)).abs() < 0.01,
                    "R {r} in {l}"
                );
            }
            // Diode knee: some gain reduction already at the threshold (knee centred on it).
            assert!(m.static_out_db(-20.0) < -20.0 + 1e-4);
            // The DSP (feedback loop) settles to the law, colouration included (< 0.1 dB at DC).
            assert_static(&mut m, &[-40.0, 0.0, 10.0], law, 0.1);
        }
    }

    #[test]
    fn ecl33_attack_recovery_and_auto_modes_match_ams_spec() {
        // Ratio 6 (index 4): closed-loop attack 63 % after 6 ms (slow) / 3 ms (fast).
        for (fast, spec) in [(0, 6.0), (1, 3.0)] {
            let mut m = model(
                ConsoleKind::Ecl33,
                &[
                    (6, ParamValue::Float(-30.0)),
                    (7, ParamValue::Enum(4)),
                    (8, ParamValue::Enum(1)),
                    (9, ParamValue::Int(fast)),
                ],
            );
            let g = step_gain(&mut m, -50.0, 0.0, 0.3, 0.3);
            let t = time_to(&g, 0.0, 0.632);
            assert!(
                (t - spec).abs() < 0.15 * spec + 0.1,
                "comp attack {t} vs {spec}"
            );
        }
        // Manual recovery: 63 % of the GR recovered after the recovery time (loop open below threshold).
        for (ri, rec) in ECL33_COMP_REC_S.iter().enumerate() {
            let mut m = model(
                ConsoleKind::Ecl33,
                &[
                    (6, ParamValue::Float(-30.0)),
                    (8, ParamValue::Enum(ri as u32)),
                ],
            );
            let g = step_gain(&mut m, 0.0, -80.0, 0.5, 8.0);
            let g0 = -(0.0 - m.static_out_db(0.0) as f64);
            let t = time_to(&g, g0, 0.632);
            let e = rec * 1e3;
            assert!((t - e).abs() < 0.03 * e + 1.0, "comp rec {e}: {t}");
        }
        // A1 / A2: short bursts recover with the fast constant, sustained compression with the slow one;
        // A2 recovers short bursts faster and sustained compression slower than A1 (AMS: 100 ms/2 s vs
        // 50 ms/5 s).
        let mut burst = [0.0; 2];
        let mut sustained = [0.0; 2];
        for (k, rec) in [4u32, 5].into_iter().enumerate() {
            let mut m = model(
                ConsoleKind::Ecl33,
                &[(6, ParamValue::Float(-30.0)), (8, ParamValue::Enum(rec))],
            );
            burst[k] = recovery_ms(&mut m, 0.0, -80.0, 0.01, 3.0);
            sustained[k] = recovery_ms(&mut m, 0.0, -80.0, 4.0, 3.0);
            assert!(
                sustained[k] > 4.0 * burst[k],
                "rec {rec}: {burst:?} {sustained:?}"
            );
        }
        assert!(burst[1] < burst[0] && sustained[1] > sustained[0]);
    }

    #[test]
    fn ecl33_limiter_static_timing_and_colour() {
        // Limiter on at -6 dBFS, compressor off: a 10 dB step above threshold moves the output < 0.1 dB.
        let mut m = model(
            ConsoleKind::Ecl33,
            &[
                (1, ParamValue::Int(1)),
                (2, ParamValue::Float(-6.0)),
                (5, ParamValue::Int(0)),
                (3, ParamValue::Enum(1)),
            ],
        );
        let a = m.static_out_db(4.0) - m.static_out_db(-6.0 + ECL33_KNEE_DB);
        assert!(a.abs() < 0.1 && a > 0.0, "limiter 10 dB step -> {a} dB");
        let meas = static_curve(&mut m, FS, &[-20.0, 6.0], 1.0);
        assert!((meas[0] + 20.0).abs() < 0.01);
        assert!((meas[1] - m.static_out_db(6.0) as f64).abs() < 0.1);
        // Limiter attack 2 / 4 ms (fast / slow), feed-forward.
        for (fast, spec) in [(1, 2.0), (0, 4.0)] {
            m.set_param(4, ParamValue::Int(fast));
            m.commit();
            let g = step_gain(&mut m, -30.0, 6.0, 0.2, 0.3);
            let t = time_to(&g, 0.0, 0.632);
            assert!(
                (t - spec).abs() < 0.1 * spec + 0.05,
                "lim attack {t} vs {spec}"
            );
        }
        // Limiter recovery steps.
        for (ri, rec) in ECL33_LIM_REC_S.iter().enumerate() {
            m.set_param(3, ParamValue::Enum(ri as u32));
            m.commit();
            let g = step_gain(&mut m, 6.0, -80.0, 0.3, 4.0);
            let g0 = m.static_out_db(6.0) as f64 - 6.0;
            let t = time_to(&g, g0, 0.632);
            let e = rec * 1e3;
            assert!((t - e).abs() < 0.03 * e + 1.0, "lim rec {e}: {t}");
        }
        // Make-up gain.
        m.set_param(10, ParamValue::Float(9.0));
        assert!((m.static_out_db(-20.0) + 11.0).abs() < 1e-3);
        // Diode-bridge colouration: 1 kHz sine, comp 6:1 at 20 dB GR, -6 dBFS output: THD within the AMS
        // ceiling (< 0.2 %) and non-zero; no colouration without GR.
        let thd_at = |thr: f32, amp: f32| {
            let mut m = model(
                ConsoleKind::Ecl33,
                &[
                    (6, ParamValue::Float(thr)),
                    (7, ParamValue::Enum(4)),
                    (8, ParamValue::Enum(3)),
                ],
            );
            let n = 48_000;
            let x: Vec<f32> = (0..n)
                .map(|i| amp * (2.0 * core::f32::consts::PI * 1000.0 * i as f32 / 48_000.0).sin())
                .collect();
            let y = run_mono(&mut m, &x);
            let tail = &y[n - 4800..];
            let h = |k: f64| {
                let w = 2.0 * core::f64::consts::PI * 1000.0 * k / FS;
                let (mut re, mut im) = (0.0, 0.0);
                for (j, &s) in tail.iter().enumerate() {
                    re += s as f64 * (w * j as f64).cos();
                    im += s as f64 * (w * j as f64).sin();
                }
                (re * re + im * im).sqrt()
            };
            let f = h(1.0);
            (
                (h(2.0).powi(2) + h(3.0).powi(2) + h(5.0).powi(2)).sqrt() / f,
                m.gr_now,
            )
        };
        // Threshold so that a ~ +18 dBFS-peak input yields ~20 dB GR at 6:1 (peak detector).
        let (thd, gr) = thd_at(-6.0 - 4.0, 10f32.powf(18.0 / 20.0));
        assert!(gr > 15.0, "gr {gr}");
        assert!(thd > 1e-5 && thd < 0.002, "thd {thd}");
        let (thd0, gr0) = thd_at(-5.0, 0.1);
        assert!(gr0 < 1e-6 && thd0 < 1e-5, "thd without GR {thd0}");
    }

    #[test]
    fn e88c_knee_threshold_pull_lim_and_timing() {
        // thr +4 dBu = -14 dBFS, ratio 4, HARD knee.
        let mut m = model(
            ConsoleKind::E88c,
            &[
                (1, ParamValue::Enum(1)),
                (2, ParamValue::Float(4.0)),
                (4, ParamValue::Float(4.0)),
            ],
        );
        assert!((m.threshold_dbfs() + 14.0).abs() < 1e-4);
        assert_static(
            &mut m,
            &[-30.0, -14.0, -2.0],
            |l| {
                if l > -14.0 {
                    -14.0 + (l + 14.0) / 4.0
                } else {
                    l
                }
            },
            0.05,
        );
        // SOFT knee: quadratic over 10 dB centred on the threshold.
        m.set_param(1, ParamValue::Enum(0));
        let at = m.static_out_db(-14.0);
        assert!((at - (-14.0 - 0.75 * 5.0 * 5.0 / 20.0)).abs() < 1e-3);
        assert!((m.static_out_db(-4.0) - (-14.0 + 10.0 / 4.0)).abs() < 1e-3);
        // THR -20 pull, LIM end of the ratio pot.
        m.set_param(3, ParamValue::Int(1));
        m.set_param(4, ParamValue::Float(10.0));
        assert!((m.threshold_dbfs() + 34.0).abs() < 1e-4);
        assert!((m.static_out_db(-10.0) + 34.0).abs() < 1e-3);
        // Attack FAST 0.1 ms / SLOW 10 ms (63 %), release time constant.
        m.set_param(1, ParamValue::Enum(1));
        m.set_param(4, ParamValue::Float(4.0));
        for (fast, spec) in [(1, 0.1), (0, 10.0)] {
            m.set_param(5, ParamValue::Enum(fast));
            m.commit();
            let g = step_gain(&mut m, -60.0, -4.0, 0.1, 0.2);
            let t = time_to(&g, 0.0, 0.632);
            assert!(
                (t - spec).abs() < 0.05 * spec + 0.03,
                "attack {t} vs {spec}"
            );
        }
        m.set_param(6, ParamValue::Float(500.0));
        m.commit();
        let g = step_gain(&mut m, -4.0, -80.0, 0.3, 3.0);
        let g0 = m.static_out_db(-4.0) as f64 + 4.0;
        let t = time_to(&g, g0, 0.632);
        assert!((t - 500.0).abs() < 15.0, "release {t}");
        // AUTO end of the release pot: programme-dependent.
        m.set_param(6, ParamValue::Float(3000.0));
        m.commit();
        let fast = recovery_ms(&mut m, -4.0, -80.0, 0.01, 3.0);
        let slow = recovery_ms(&mut m, -4.0, -80.0, 3.0, 3.0);
        assert!(slow > 3.0 * fast, "auto {fast} {slow}");
    }

    #[test]
    fn red3_soft_knee_times_and_auto() {
        let mut m = model(
            ConsoleKind::Red3,
            &[
                (1, ParamValue::Float(-24.0)),
                (2, ParamValue::Enum(8)),
                (3, ParamValue::Float(5.0)),
                (4, ParamValue::Float(800.0)),
            ],
        );
        assert_eq!(m.param(2), Some(4.0));
        // Above / below the 10 dB knee: hard-knee asymptotes; inside: smooth.
        assert_static(
            &mut m,
            &[-40.0, -10.0, 0.0],
            |l| {
                if l > -24.0 {
                    -24.0 + (l + 24.0) / 4.0
                } else {
                    l
                }
            },
            0.05,
        );
        let at = m.static_out_db(-24.0);
        assert!((at - (-24.0 - 0.75 * 25.0 / 20.0)).abs() < 1e-3);
        let mut prev = m.static_out_db(-40.0);
        for k in 1..400 {
            let o = m.static_out_db(-40.0 + k as f32 * 0.1);
            let s = (o - prev) / 0.1;
            assert!((0.249..=1.001).contains(&s), "slope {s}");
            prev = o;
        }
        // Attack 5 ms, release 800 ms (63 %).
        let g = step_gain(&mut m, -60.0, 0.0, 0.1, 0.3);
        let t = time_to(&g, 0.0, 0.632);
        assert!((t - 5.0).abs() < 0.3, "attack {t}");
        let g = step_gain(&mut m, 0.0, -80.0, 0.3, 5.0);
        let t = time_to(&g, -18.0, 0.632);
        assert!((t - 800.0).abs() < 20.0, "release {t}");
        // AUTO.
        m.set_param(5, ParamValue::Int(1));
        m.commit();
        let fast = recovery_ms(&mut m, 0.0, -80.0, 0.01, 3.0);
        let slow = recovery_ms(&mut m, 0.0, -80.0, 3.0, 3.0);
        assert!(slow > 3.0 * fast, "auto {fast} {slow}");
    }

    #[test]
    fn api2250_knob_laws_attack_switch_and_new_old() {
        assert!((api_threshold_dbu(0.0) - 10.0).abs() < 1e-6);
        assert!((api_threshold_dbu(10.0) + 20.0).abs() < 1e-6);
        assert!((api_ratio(0.0) - 1.0).abs() < 1e-6);
        assert!((api_ratio(5.0) - 2.0).abs() < 1e-5);
        assert!(api_ratio(10.0).is_infinite());
        // thr knob 6 = -8 dBu = -26 dBFS; ratio knob 7.5 = 4:1; HARD knee, NEW (feed-forward RMS).
        let mut m = model(
            ConsoleKind::Api2250,
            &[
                (1, ParamValue::Float(6.0)),
                (2, ParamValue::Float(7.5)),
                (5, ParamValue::Enum(0)),
                (6, ParamValue::Enum(1)),
            ],
        );
        assert!((m.threshold_dbfs() + 26.0).abs() < 1e-4);
        let law = |l: f64| {
            if l > -26.0 {
                -26.0 + (l + 26.0) / 4.0
            } else {
                l
            }
        };
        assert_static(&mut m, &[-40.0, -14.0, 0.0], law, 0.05);
        // RMS: a 0 dBFS-peak sine reads -3 dBFS.
        let x: Vec<f32> = (0..48_000).map(|i| (i as f32 * 0.13).sin()).collect();
        m.reset();
        let _ = run_mono(&mut m, &x);
        let gr = m.telemetry().gain_reduction_db;
        assert!((gr - 0.75 * (26.0 - 3.01)).abs() < 0.5, "rms gr {gr}");
        // Attack switch FAST / MED / SLOW = 2 / 18 / 75 ms (63 %).
        for (i, spec) in [(0u32, 2.0), (1, 18.0), (2, 75.0)] {
            m.set_param(3, ParamValue::Enum(i));
            m.commit();
            let g = step_gain(&mut m, -60.0, 0.0, 0.1, 0.6);
            let t = time_to(&g, 0.0, 0.632);
            assert!((t - spec).abs() < 0.06 * spec + 0.3, "attack {t} vs {spec}");
        }
        // OLD (feedback): same static ratio, attack still follows the switch.
        m.set_param(6, ParamValue::Enum(0));
        m.set_param(3, ParamValue::Enum(1));
        m.commit();
        assert_static(&mut m, &[-40.0, -14.0, 0.0], law, 0.05);
        let g = step_gain(&mut m, -60.0, 0.0, 0.1, 0.6);
        let t = time_to(&g, 0.0, 0.632);
        assert!((t - 18.0).abs() < 1.5, "old attack {t}");
        // OLD at the infinity end is capped (feedback cannot limit perfectly): 31:1.
        m.set_param(2, ParamValue::Float(10.0));
        let o = m.static_out_db(4.0) as f64;
        assert!(
            (o - (-26.0 + 30.0 / (API_FB_MAX_K as f64 + 1.0))).abs() < 0.01,
            "{o}"
        );
        // SOFT knee.
        m.set_param(5, ParamValue::Enum(1));
        m.set_param(6, ParamValue::Enum(1));
        m.set_param(2, ParamValue::Float(7.5));
        let at = m.static_out_db(-26.0);
        assert!((at - (-26.0 - 0.75 * 25.0 / 20.0)).abs() < 1e-3);
    }

    #[test]
    fn lmt_amount_ratio_transients_and_gains() {
        // comp 50 => thr -20 dBFS, ratio 20, C GAIN +6.
        let mut m = model(
            ConsoleKind::Lmt,
            &[(5, ParamValue::Float(50.0)), (6, ParamValue::Float(6.0))],
        );
        assert!((m.threshold_dbfs() + 20.0).abs() < 1e-4);
        assert_static(
            &mut m,
            &[-30.0, -10.0, 0.0],
            |l| {
                6.0 + if l > -20.0 {
                    -20.0 + (l + 20.0) / 20.0
                } else {
                    l
                }
            },
            0.05,
        );
        // Very fast attack (0.3 ms), 150 ms release.
        let g = step_gain(&mut m, -60.0, 0.0, 0.1, 0.1);
        assert!((time_to(&g, 6.0, 0.632) - 0.3).abs() < 0.05);
        // C ON off: unity, no make-up.
        m.set_param(7, ParamValue::Int(0));
        assert!((m.static_out_db(-10.0) + 10.0).abs() < 1e-4);
        // Transient section: +10 boosts the onset of a burst, -10 attenuates it, steady state = T GAIN.
        let burst = |m: &mut ConsoleComp, trans: f32| {
            m.set_param(4, ParamValue::Int(1));
            m.set_param(2, ParamValue::Float(trans));
            m.set_param(3, ParamValue::Float(-3.0));
            m.commit();
            let mut x = vec![0.0f32; 24_000];
            x[4800..].fill(0.25);
            m.reset();
            let y = run_mono(m, &x);
            let onset = db((y[4800 + 48] / 0.25) as f64);
            let steady = db((y[23_999] / 0.25) as f64);
            (onset, steady)
        };
        let (on_p, st_p) = burst(&mut m, 10.0);
        let (on_n, st_n) = burst(&mut m, -10.0);
        let (on_0, _) = burst(&mut m, 0.0);
        assert!(on_p > 3.0 && on_n < -9.0, "onsets {on_p} {on_n}");
        assert!((on_0 + 3.0).abs() < 0.01);
        assert!((st_p + 3.0).abs() < 0.05 && (st_n + 3.0).abs() < 0.05);
        // Faster T SPEED shortens the boosted transient.
        let width = |m: &mut ConsoleComp, spd: f32| {
            m.set_param(1, ParamValue::Float(spd));
            m.set_param(2, ParamValue::Float(10.0));
            m.set_param(3, ParamValue::Float(0.0));
            m.commit();
            let mut x = vec![0.0f32; 24_000];
            x[4800..].fill(0.25);
            m.reset();
            let y = run_mono(m, &x);
            (4800..24_000)
                .position(|k| db((y[k] / 0.25) as f64) < 1.0)
                .unwrap()
        };
        assert!(width(&mut m, 1.0) * 5 < width(&mut m, 0.0));
    }

    #[test]
    fn stereo_link_sidechain_params_and_meter() {
        let mut m = model(ConsoleKind::Red3, &[(1, ParamValue::Float(-30.0))]);
        let mut l = vec![0.5f32; 4800];
        let mut r = vec![0.01f32; 4800];
        for (a, b) in l.chunks_mut(64).zip(r.chunks_mut(64)) {
            m.process(a, b, None);
        }
        let gl = db((l[4799] / 0.5) as f64);
        let gr = db((r[4799] / 0.01) as f64);
        assert!((gl - gr).abs() < 1e-3 && gl < -3.0);
        // External key: silent key leaves the signal untouched (FF and FB).
        for kind in [ConsoleKind::Red3, ConsoleKind::Ecl33] {
            let mut m = model(kind, &[]);
            let (mut l, mut r) = (vec![0.9f32; 64], vec![0.9f32; 64]);
            let k = [0.0f32; 64];
            m.process(&mut l, &mut r, Some((&k, &k)));
            assert!(l.iter().all(|&v| (v - 0.9).abs() < 1e-6), "{kind:?}");
        }
        // Unknown slots ignored, ranges clamped (E88C inverted range), defaults restorable.
        let mut e = ConsoleComp::new(ConsoleKind::E88c);
        e.set_param(9, ParamValue::Float(3.0));
        assert_eq!(e.param(9), None);
        e.set_param(2, ParamValue::Float(40.0));
        assert_eq!(e.param(2), Some(20.0));
        e.load_defaults();
        assert_eq!(e.param(6), Some(353.0));
        let mut c = ConsoleComp::new(ConsoleKind::Ecl33);
        assert_eq!(c.param(8), Some(4.0));
        c.set_param(7, ParamValue::Float(5.5));
        assert_eq!(c.param(7), Some(6.0));
        assert_eq!(ConsoleKind::from_id("2250"), Some(ConsoleKind::Api2250));
        // GR meter (dyn-models.json, embedded): GR / 20 capped at 1.0.
        let grs = [0.0, 1.0, 6.0, 20.0, 30.0, 80.0];
        let meter = [0.0, 0.05, 0.3, 1.0, 1.0, 1.0];
        for kind in ConsoleKind::ALL {
            for (g, e) in grs.iter().zip(meter) {
                assert!((gr_meter(kind, *g) - e).abs() < 1e-6);
            }
        }
    }

    #[test]
    fn telemetry_tracks_gain_reduction() {
        for kind in ConsoleKind::ALL {
            let mut m = model(kind, &[]);
            let x = vec![0.9f32; 24_000];
            let y = run_mono(&mut m, &x);
            let gr = m.telemetry().gain_reduction_db as f64;
            assert!(gr > 0.0, "{kind:?}");
            let applied = -db((y[23_999] / 0.9) as f64) + 20.0 * (m.makeup as f64).log10();
            // ECL33 colouration adds a little at DC.
            assert!((applied - gr).abs() < 0.05, "{kind:?}: {applied} vs {gr}");
            assert_eq!(m.telemetry().engaged, gr > 0.01);
        }
    }
}
