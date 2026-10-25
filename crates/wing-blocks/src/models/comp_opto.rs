//! Opto / vari-mu / class-A compressors (T04E): `LA` (LA LEVELER, Teletronix LA-2A), `L100` (LTA100 LEVELER,
//! Summit Audio TLA-100A), `F670` (FAIR KID, Fairchild 670) and `BLISS` (ETERNAL BLISS, elysia mpressor).
//! Parameters and ranges: `docs/research/models.md` §3.2 and `data/schema/wing-tree.json` (slot order, enum
//! tables, defaults). Every published constant below names its source; the T04E task Log has the references
//! and `docs/VERIFY-ON-HARDWARE.md` lists the assumptions.
//!
//! Signal flow (stereo-linked: one gain from the louder channel):
//! ```text
//! LA / L100 (feedback leveler):
//!   x -> [cell gain g] -> GAIN knob -> tube stage -> out
//!          ^ detector (EL panel: short mean square | instantaneous power) of x * g, soft knee, slope k
//!          | two-part cell: fast part + slow part (release time grows with the cell's illumination history)
//! F670 (feedback vari-mu):
//!   x -> INPUT attenuator -> [vari-mu gain g] -> fixed amp -> push-pull tube stage -> out
//!          ^ full-wave peak rectifier of the cell output against the DC threshold, GR = gm law * excess,
//!          | six time-constant positions (5, 6: two / three parallel programme-dependent RC stages)
//! BLISS (feed-forward VCA, class A, no colouration):
//!   x -> [VCA] -> out;  peak detector -> hard knee incl. negative ratios -> GR limiter -> log / anti-log
//!   ballistics with optional auto-fast attack
//! ```
//!
//! Feedback topology (LA, L100, F670): the detector sees the already-reduced signal (one-sample loop delay), so a
//! log-domain slope `k` gives a steady-state ratio of `1 + k`, and the loop closes `1 + k` times faster than the
//! cell's own attack time constant. The leveler cell attack is therefore `tau * (1 + k)` so the published
//! attack time is the closed-loop figure; release is open loop (target 0) and follows the cell constants.
//!
//! Tube colouration: the hardware has tube make-up stages (LA-2A 12AX7/12BH7, TLA-100A tube output, 670 push-pull
//! 6386/6973); WING's colouration is undocumented, so the stage is deliberately mild (2nd and 3rd harmonics
//! ≈ -45 dBc for a 0 dBFS sine at the stage input, falling 6 / 12 dB per 6 dB less level). It is a memoryless cubic soft clipper with a bias offset (even harmonics),
//! implemented as `y = x + HP(ADAA1(r(x)))` where `r(x) = shaper(x) - x` is the nonlinear residual: the linear
//! path is untouched (no half-sample averaging droop), the residual is first-order antiderivative
//! anti-aliased (no oversampling, no latency) and DC-blocked at [`TUBE_RESIDUAL_HP_HZ`] (the bias's level
//! dependent DC shift, as an AC-coupled stage would). DC input therefore passes the stage unchanged, so static
//! curves measured with DC are the compressor laws alone.
//!
//! GR telemetry is the applied cell gain reduction in dB; [`gr_meter`] reproduces WING-Edit's GR-meter mapping
//! (`comp_gain_<m>`, `data/reference-curves/dyn-models.json`).
//!
//! Real-time: no allocation; `set_param` stages values, `commit`/`process`/`reset` apply them. Ballistics state
//! is f64 (25 s release constants would stall in f32). Per block: one `exp` (leveler programme-dependent
//! release); per sample: the fast dB helpers of `wing_dsp::gain` plus polynomial tube math.

use crate::model::{Model, ModelTelemetry, ParamValue};
use wing_dsp::gain::{db_to_lin_fast, lin_to_db_fast};
use wing_dsp::smooth::one_pole_coeff;

// ---- LA (Teletronix LA-2A) ----------------------------------------------------------------------------------

/// LA-2A attack time, "10 milliseconds average" (Universal Audio LA-2A Classic spec,
/// help.uaudio.com article 206356233). Closed-loop figure (see module doc).
pub const LA_ATTACK_S: f64 = 0.010;
/// LA-2A release: "approximately 0.06 seconds for 50 % release" (same UA spec).
pub const LA_HALF_RELEASE_S: f64 = 0.060;
/// LA-2A release: "0.5 to 5 seconds for complete release" depending on programme history (same UA spec;
/// Wikipedia quotes 1..15 s, not used).
pub const LA_COMPLETE_RELEASE_S: [f64; 2] = [0.5, 5.0];
/// "Complete" release is taken as GR below 5 % of its starting value. The slow part (weight
/// `1 - LA_FAST_WEIGHT` = 0.4) reaches 0.05 after `tau * ln 8`, so the slow time constant spans
/// `LA_COMPLETE_RELEASE_S / ln 8`.
pub const LA_SLOW_RELEASE_S: [f64; 2] = [
    LA_COMPLETE_RELEASE_S[0] / core::f64::consts::LN_2 / 3.0,
    LA_COMPLETE_RELEASE_S[1] / core::f64::consts::LN_2 / 3.0,
];
/// Two-part T4 cell (fast + slow photoresistor components, the usual T4 two-time-constant description):
/// weight of the fast part and its release constant, chosen so 50 % release happens after 51..60 ms over the
/// slow-constant range (0.6 e^(-t/42 ms) + 0.4 e^(-t/tau_s) = 0.5). Weight is an assumption.
pub const LA_FAST_WEIGHT: f64 = 0.6;
pub const LA_FAST_RELEASE_S: f64 = 0.042;
/// COMPRESS ≈ 3:1 (models.md; UA LA-2A docs). LIMIT: UA only says "higher ratio"; models.md "~inf".
/// 20:1 is an assumption.
pub const LA_COMP_RATIO: f64 = 3.0;
pub const LA_LIM_RATIO: f64 = 20.0;
/// T4 soft knee width (dB, quadratic, centred on the threshold). Assumption.
pub const LA_KNEE_DB: f64 = 10.0;
/// Electroluminescent panel persistence: mean-square averaging time of the detector. Assumption.
pub const LA_EL_TAU_S: f64 = 0.005;
/// PEAK REDUCTION is the side-chain drive pot (linear 0..100 % of [`LA_SC_MAX_DB`]): threshold at the cell
/// output = `LA_CELL_THR_DBFS - LA_SC_MAX_DB - 20 log10(peak / 100)` (100 -> -30 dBFS, 20 -> -16 dBFS,
/// 0 -> no reduction). Assumption.
pub const LA_CELL_THR_DBFS: f64 = 0.0;
pub const LA_SC_MAX_DB: f64 = 30.0;
/// GAIN knob (output gain after the cell, before the tube stage; independent of compression as on the
/// LA-2A): `(ingain - 40) * 0.4 dB`, unity at the WING default 40. Assumption.
pub const LA_GAIN_UNITY: f32 = 40.0;
pub const LA_GAIN_DB_PER_STEP: f32 = 0.4;
/// LA tube stage (12AX7/12BH7 make-up): drive and bias of the cubic shaper. Assumption.
pub const LA_TUBE: (f64, f64) = (0.25, 0.05);

/// Cell memory: the slow-part release interpolates (log) between its limits with a memory `m` in 0..1 that
/// charges toward `target GR / MEM_FULL_GR_DB` while the cell is lit (driven) and decays otherwise. Assumptions (bounded by
/// the published 0.5..5 s range).
pub const MEM_FULL_GR_DB: f64 = 10.0;
pub const MEM_CHARGE_S: f64 = 3.0;
pub const MEM_DECAY_S: f64 = 30.0;

// ---- L100 (Summit Audio TLA-100A) ---------------------------------------------------------------------------

/// TLA-100A attack/release spans "attack 180 uS - 100 mS, release 32 mS - 1.5 S" (Zen Pro Audio spec listing;
/// the Summit manual names only FAST/MEDIUM/SLOW). FAST/SLOW are the span ends, MED the geometric middle
/// (assumption).
pub const L100_ATTACK_S: [f64; 3] = [180e-6, 4.243e-3, 0.100];
pub const L100_RELEASE_S: [f64; 3] = [0.032, 0.219, 1.5];
/// "The slower the release time setting, the more the program material determines the release time"
/// (Softube TLA-100A manual): slow-part release spans `release * [1, spread]` with the cell memory.
/// Spread values are assumptions.
pub const L100_PROGRAM_SPREAD: [f64; 3] = [1.0, 3.0, 6.0];
pub const L100_FAST_WEIGHT: f64 = 0.5;
/// GAIN RED "works almost like a threshold control" and "increases the ratio" (Softube manual):
/// threshold `6 - 3.6 * gr` dBFS, ratio `2 + 0.4 * gr` (2:1 .. 6:1). Assumption.
pub const L100_THR_AT_0: f64 = 6.0;
pub const L100_THR_PER_STEP: f64 = 3.6;
pub const L100_RATIO_AT_0: f64 = 2.0;
pub const L100_RATIO_PER_STEP: f64 = 0.4;
/// Very soft knee (models.md). Width is an assumption.
pub const L100_KNEE_DB: f64 = 12.0;
/// GAIN: `(ingain - 3) * 3 dB`, unity at the WING default 3. Assumption.
pub const L100_GAIN_UNITY: f32 = 3.0;
pub const L100_GAIN_DB_PER_STEP: f32 = 3.0;
pub const L100_TUBE: (f64, f64) = (0.2, 0.05);

// ---- F670 (Fairchild 670) -----------------------------------------------------------------------------------

/// Fairchild 670 time-constant switch (Fairchild 670 manual table, as repeated in retailer spec listings,
/// e.g. thomann.de "fairchild_670"): position -> attack / release.
/// 1: 0.2 ms / 0.3 s, 2: 0.2 ms / 0.8 s, 3: 0.4 ms / 2 s, 4: 0.4 ms / 5 s,
/// 5: 0.4 ms / 2 s individual peaks, 10 s multiple peaks,
/// 6: 0.2 ms / 0.3 s individual peaks, 10 s multiple peaks, 25 s consistently high average level.
/// (Thomann prints 0.2 s for position 5's single peaks; the manual's 2 s is used.)
pub const F670_ATTACK_S: [f64; 6] = [0.2e-3, 0.2e-3, 0.4e-3, 0.4e-3, 0.4e-3, 0.2e-3];
pub const F670_RELEASE_S: [f64; 6] = [0.3, 0.8, 2.0, 5.0, 2.0, 0.3];
pub const F670_MULTI_RELEASE_S: f64 = 10.0;
pub const F670_SUSTAINED_RELEASE_S: f64 = 25.0;
/// Charge times of the programme-dependent stages (how quickly "multiple peaks" / a "consistently high
/// average level" build up the slower stages). Assumptions.
pub const F670_MULTI_CHARGE_S: f64 = 0.1;
pub const F670_SUSTAINED_CHARGE_S: f64 = 2.0;
/// Vari-mu gain law: remote-cutoff 6386 gm falls ~exponentially with grid bias, i.e. GR in dB is linear in the
/// control voltage: `GR = F670_GM_DB * bias_scale * knee(|y| / v_thr - 1)`. With feedback the local ratio
/// above the knee is `1 + F670_GM_DB * ln10/20 * |y|/v_thr`: ≈2.8:1 at 6 dB GR, ≈4.5:1 at 20 dB GR (1:1 below
/// the knee). Constant is an assumption (no published static curve).
pub const F670_GM_DB: f64 = 10.0;
/// Remote-cutoff region: the excess `|y| / v_thr - 1` passes a quadratic knee of this width (threshold units,
/// i.e. 0.75..1.25 x threshold) so GR and ratio grow smoothly from zero. Assumption.
pub const F670_KNEE: f64 = 0.5;
pub const F670_GR_MAX_DB: f64 = 40.0;
/// THRESHOLD knob 0..10 -> DC threshold `6 - 3.6 * thr` dBFS at the cell output. Assumption.
pub const F670_THR_AT_0: f64 = 6.0;
pub const F670_THR_PER_STEP: f64 = 3.6;
/// Fixed amplifier gain after the cell: unity overall at the WING default INPUT -10 dB. Assumption.
pub const F670_AMP_DB: f32 = 10.0;
/// Push-pull stage (odd harmonics at CAL); BIAS below CAL unbalances it (even harmonics up to
/// `F670_BIAS_ASYM`) and softens the gm law down to half slope. Assumptions.
pub const F670_TUBE_DRIVE: f64 = 0.25;
pub const F670_BIAS_ASYM: f64 = 0.25;

// ---- BLISS (elysia mpressor) --------------------------------------------------------------------------------

/// `ratio` table (schema): positive ratios and mpressor negative ("beyond infinity") ratios.
pub const BLISS_RATIOS: [f32; 12] = [
    1.2, 1.3, 1.6, 2.0, 3.0, 5.0, 10.0, -0.3, -1.0, -2.0, -3.0, -4.0,
];
/// Auto Fast "varies the attack time predicated upon the signal's speed and instantaneous level" (Barry
/// Rudolph, mpressor review): the attack rate is multiplied by `1 + overshoot / BLISS_AUTO_FAST_DB`.
/// Constant is an assumption.
pub const BLISS_AUTO_FAST_DB: f64 = 6.0;
pub const BLISS_GR_MAX_DB: f64 = 60.0;

// ---- shared -------------------------------------------------------------------------------------------------

/// High-pass corner of the tube residual (DC blocker of the bias-induced level-dependent offset).
pub const TUBE_RESIDUAL_HP_HZ: f64 = 5.0;
const POWER_FLOOR: f32 = 1e-24;

/// Which of the four models an instance is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OptoKind {
    La,
    L100,
    F670,
    Bliss,
}

impl OptoKind {
    pub const ALL: [OptoKind; 4] = [
        OptoKind::La,
        OptoKind::L100,
        OptoKind::F670,
        OptoKind::Bliss,
    ];

    /// Snapshot `mdl` token.
    pub fn id(self) -> &'static str {
        match self {
            OptoKind::La => "LA",
            OptoKind::L100 => "L100",
            OptoKind::F670 => "F670",
            OptoKind::Bliss => "BLISS",
        }
    }

    pub fn from_id(id: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|k| k.id() == id)
    }

    fn slots(self) -> &'static [SlotDef] {
        match self {
            OptoKind::La => &LA_SLOTS,
            OptoKind::L100 => &L100_SLOTS,
            OptoKind::F670 => &F670_SLOTS,
            OptoKind::Bliss => &BLISS_SLOTS,
        }
    }
}

/// WING-Edit GR-meter position (`comp_gain_<m>`, `dyn-models.json`): `GR / 20 dB` capped at 1.0 for LA, F670
/// and BLISS; L100 deflects the other way (`-GR / 20`, capped at -1.0).
pub fn gr_meter(kind: OptoKind, gr_db: f32) -> f32 {
    let m = (gr_db.max(0.0) / 20.0).min(1.0);
    match kind {
        OptoKind::L100 => -m,
        _ => m,
    }
}

#[derive(Clone, Copy, Debug)]
enum SlotKind {
    Range(f32, f32),
    Table(&'static [f32]),
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

/// `ingain peak mode` (mode: COMP / LIM).
const LA_SLOTS: [SlotDef; 3] = [
    range(0.0, 100.0, 40.0),
    range(0.0, 100.0, 20.0),
    index(2, 0),
];
/// `ingain gr att rel` (att/rel: FAST / MED / SLOW).
const L100_SLOTS: [SlotDef; 4] = [
    range(0.0, 10.0, 3.0),
    range(0.0, 10.0, 3.0),
    index(3, 1),
    index(3, 1),
];
/// `in thr time bias` (time is int 1..6).
const F670_SLOTS: [SlotDef; 4] = [
    range(-20.0, 0.0, -10.0),
    range(0.0, 10.0, 5.0),
    range(1.0, 6.0, 2.0),
    range(0.0, 1.0, 1.0),
];
/// `thr ratio att rel afast alog glon glim`.
const BLISS_SLOTS: [SlotDef; 8] = [
    range(-50.0, 0.0, -15.0),
    SlotDef {
        kind: SlotKind::Table(&BLISS_RATIOS),
        default: 3.0,
    },
    range(0.4, 150.0, 9.9),
    range(5.0, 1200.0, 102.0),
    boolean(false),
    boolean(false),
    boolean(false),
    range(-21.0, 0.0, -12.0),
];
const MAX_SLOTS: usize = 8;

/// Quadratic soft knee centred on 0 with width `w` (dB): 0 below `-w/2`, identity above `w/2`.
#[inline]
fn knee(over: f64, w: f64) -> f64 {
    if w <= 0.0 {
        over.max(0.0)
    } else if over <= -0.5 * w {
        0.0
    } else if over >= 0.5 * w {
        over
    } else {
        let t = over + 0.5 * w;
        t * t / (2.0 * w)
    }
}

/// Solve `gr = law(gr)` for a decreasing `law(gr) - gr` on `[0, hi]` (bisection; feedback steady state).
fn solve_feedback(hi: f64, law: impl Fn(f64) -> f64) -> f64 {
    let (mut lo, mut hi) = (0.0f64, hi);
    if law(0.0) <= 0.0 {
        return 0.0;
    }
    for _ in 0..80 {
        let mid = 0.5 * (lo + hi);
        if law(mid) - mid > 0.0 {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    0.5 * (lo + hi)
}

// ---- tube stage ---------------------------------------------------------------------------------------------

/// Cubic soft clipper `s(u) = u - u^3/3` (|u| <= 1), `±2/3` beyond.
#[inline]
fn clip(u: f64) -> f64 {
    if u.abs() <= 1.0 {
        u - u * u * u / 3.0
    } else {
        u.signum() * (2.0 / 3.0)
    }
}

/// Antiderivative of [`clip`] (continuous at |u| = 1: 5/12).
#[inline]
fn clip_ad(u: f64) -> f64 {
    let a = u.abs();
    if a <= 1.0 {
        let u2 = u * u;
        0.5 * u2 - u2 * u2 / 12.0
    } else {
        (2.0 / 3.0) * a - 0.25
    }
}

/// Normalised shaper `g(x) = (s(d x + b) - s(b)) / (d (1 - b^2))` (unity small-signal gain, g(0) = 0).
#[derive(Clone, Copy, Debug)]
struct TubeParams {
    d: f64,
    b: f64,
    k: f64,
    sb: f64,
    hp_c: f64,
}

impl TubeParams {
    fn new(drive: f64, bias: f64, fs: f64) -> Self {
        let b = bias.clamp(-0.5, 0.5);
        Self {
            d: drive,
            b,
            k: 1.0 / (drive * (1.0 - b * b)),
            sb: clip(b),
            hp_c: (-2.0 * core::f64::consts::PI * TUBE_RESIDUAL_HP_HZ / fs).exp(),
        }
    }

    /// Residual `r(x) = g(x) - x`.
    #[inline]
    fn residual(&self, x: f64) -> f64 {
        (clip(self.d * x + self.b) - self.sb) * self.k - x
    }

    /// Antiderivative of the residual.
    #[inline]
    fn residual_ad(&self, x: f64) -> f64 {
        (clip_ad(self.d * x + self.b) / self.d - self.sb * x) * self.k - 0.5 * x * x
    }
}

#[derive(Clone, Copy, Debug, Default)]
struct TubeCh {
    x1: f64,
    hp_x1: f64,
    hp_y1: f64,
}

impl TubeCh {
    /// `x + HP(ADAA1(residual))`.
    #[inline]
    fn tick(&mut self, p: &TubeParams, x: f32) -> f32 {
        let x = x as f64;
        let dx = x - self.x1;
        let r = if dx.abs() > 1e-6 {
            (p.residual_ad(x) - p.residual_ad(self.x1)) / dx
        } else {
            p.residual(0.5 * (x + self.x1))
        };
        self.x1 = x;
        let y = r - self.hp_x1 + p.hp_c * self.hp_y1;
        self.hp_x1 = r;
        self.hp_y1 = y;
        (x + y) as f32
    }

    /// Same stage without anti-aliasing (reference for the aliasing test).
    #[cfg(test)]
    fn tick_naive(&mut self, p: &TubeParams, x: f32) -> f32 {
        let x = x as f64;
        let r = p.residual(x);
        let y = r - self.hp_x1 + p.hp_c * self.hp_y1;
        self.hp_x1 = r;
        self.hp_y1 = y;
        (x + y) as f32
    }
}

// ---- ballistics ---------------------------------------------------------------------------------------------

/// Up to three parallel RC stages on the control voltage (F670): all are charged by the side-chain rectifier
/// only while it drives them (target above the stage): stage 0 with the attack constant, stages 1-2 with
/// their (slower) charge constants; each discharges with its own release constant. GR = max of the stages, so
/// a single peak barely charges the slow stages (fast release) while repeated peaks / a sustained level
/// charge them (slow release), as the 670's programme-dependent positions 5 and 6 describe.
#[derive(Clone, Copy, Debug, Default)]
struct Cascade {
    n: usize,
    att: f64,
    charge: [f64; 3],
    rel: [f64; 3],
    s: [f64; 3],
}

impl Cascade {
    #[inline]
    fn tick(&mut self, target: f64) -> f64 {
        let mut out = 0.0f64;
        for i in 0..self.n {
            let s = self.s[i];
            let c = if target > s {
                if i == 0 {
                    self.att
                } else {
                    self.charge[i]
                }
            } else {
                self.rel[i]
            };
            let s = target + c * (s - target);
            self.s[i] = s;
            out = out.max(s);
        }
        out
    }
}

/// One of the four models (stereo-linked).
pub struct OptoComp {
    kind: OptoKind,
    fs: f64,
    v: [f32; MAX_SLOTS],
    dirty: bool,
    colour: bool,
    // derived (leveler: LA, L100)
    pre: f32,
    post: f32,
    thr_db: f64,
    k: f64,
    knee_w: f64,
    det_c: f32,
    att_c: f64,
    rel_f_c: f64,
    slow_tau: [f64; 2],
    w_fast: f64,
    mem_charge_c: f64,
    mem_decay_c: f64,
    // derived (F670)
    inv_vthr: f32,
    gm_db: f64,
    // derived (BLISS)
    slope: f64,
    gr_cap: f64,
    att_alpha: f64,
    rel_c: f64,
    afast: bool,
    alog: bool,
    tube: Option<TubeParams>,
    // state
    ms: [f32; 2],
    fast: f64,
    slow: f64,
    mem: f64,
    cascade: Cascade,
    lin_g: f64,
    g_prev: f32,
    gr_now: f32,
    tube_ch: [TubeCh; 2],
}

impl OptoComp {
    pub fn new(kind: OptoKind) -> Self {
        let mut m = Self {
            kind,
            fs: 48_000.0,
            v: [0.0; MAX_SLOTS],
            dirty: true,
            colour: true,
            pre: 1.0,
            post: 1.0,
            thr_db: 0.0,
            k: 0.0,
            knee_w: 0.0,
            det_c: 0.0,
            att_c: 0.0,
            rel_f_c: 0.0,
            slow_tau: [1.0; 2],
            w_fast: 1.0,
            mem_charge_c: 0.0,
            mem_decay_c: 0.0,
            inv_vthr: 1.0,
            gm_db: 0.0,
            slope: 0.0,
            gr_cap: BLISS_GR_MAX_DB,
            att_alpha: 1.0,
            rel_c: 0.0,
            afast: false,
            alog: false,
            tube: None,
            ms: [POWER_FLOOR; 2],
            fast: 0.0,
            slow: 0.0,
            mem: 0.0,
            cascade: Cascade::default(),
            lin_g: 1.0,
            g_prev: 1.0,
            gr_now: 0.0,
            tube_ch: [TubeCh::default(); 2],
        };
        m.load_defaults();
        m.apply();
        m
    }

    pub fn kind(&self) -> OptoKind {
        self.kind
    }

    /// Enable/disable the tube colouration stage (on by default; analysis hook).
    pub fn set_colour(&mut self, on: bool) {
        self.colour = on;
        self.dirty = true;
    }

    /// Effective threshold in dBFS at the gain-cell output (leveler, F670) or input (BLISS).
    pub fn threshold_dbfs(&mut self) -> f64 {
        self.apply();
        match self.kind {
            OptoKind::F670 => -20.0 * (self.inv_vthr as f64).log10(),
            _ => self.thr_db,
        }
    }

    /// Steady-state output level (dBFS) of the whole model for a DC input level (dBFS): the law the DSP
    /// settles to (feedback models solved for their fixed point; the tube stage passes DC unchanged).
    pub fn static_out_db(&mut self, in_db: f64) -> f64 {
        self.apply();
        match self.kind {
            OptoKind::La | OptoKind::L100 => {
                let (k, thr, w) = (self.k, self.thr_db, self.knee_w);
                let gr = solve_feedback(400.0, |gr| k * knee(in_db - gr - thr, w));
                in_db - gr + 20.0 * (self.post as f64).log10()
            }
            OptoKind::F670 => {
                let pre = 20.0 * (self.pre as f64).log10();
                let x = in_db + pre;
                let (gm, iv) = (self.gm_db, self.inv_vthr as f64);
                let gr = solve_feedback(F670_GR_MAX_DB, |gr| {
                    let e = 10f64.powf((x - gr) / 20.0) * iv;
                    (gm * knee(e - 1.0, F670_KNEE)).min(F670_GR_MAX_DB)
                });
                x - gr + 20.0 * (self.post as f64).log10()
            }
            OptoKind::Bliss => {
                let gr = (self.slope * (in_db - self.thr_db).max(0.0)).min(self.gr_cap);
                in_db - gr
            }
        }
    }

    fn apply(&mut self) {
        if !self.dirty {
            return;
        }
        self.dirty = false;
        let fs = self.fs;
        let c = |tau: f64| one_pole_coeff(tau, fs);
        let v = self.v;
        self.mem_charge_c = c(MEM_CHARGE_S);
        self.mem_decay_c = c(MEM_DECAY_S);
        self.pre = 1.0;
        self.post = 1.0;
        let tube = match self.kind {
            OptoKind::La => {
                let ratio = if v[2] as u32 == 0 {
                    LA_COMP_RATIO
                } else {
                    LA_LIM_RATIO
                };
                self.k = ratio - 1.0;
                let peak = v[1] as f64;
                self.thr_db = if peak <= 0.0 {
                    1e6
                } else {
                    LA_CELL_THR_DBFS - LA_SC_MAX_DB - 20.0 * (peak / 100.0).log10()
                };
                self.knee_w = LA_KNEE_DB;
                self.det_c = c(LA_EL_TAU_S) as f32;
                self.att_c = c(LA_ATTACK_S * (1.0 + self.k));
                self.rel_f_c = c(LA_FAST_RELEASE_S);
                self.slow_tau = LA_SLOW_RELEASE_S;
                self.w_fast = LA_FAST_WEIGHT;
                self.post = db_to_lin_fast((v[0] - LA_GAIN_UNITY) * LA_GAIN_DB_PER_STEP);
                Some(LA_TUBE)
            }
            OptoKind::L100 => {
                let gr = v[1] as f64;
                self.k = L100_RATIO_AT_0 + L100_RATIO_PER_STEP * gr - 1.0;
                self.thr_db = L100_THR_AT_0 - L100_THR_PER_STEP * gr;
                self.knee_w = L100_KNEE_DB;
                self.det_c = 0.0;
                let (a, r) = ((v[2] as usize).min(2), (v[3] as usize).min(2));
                self.att_c = c(L100_ATTACK_S[a] * (1.0 + self.k));
                self.rel_f_c = c(L100_RELEASE_S[r]);
                self.slow_tau = [
                    L100_RELEASE_S[r],
                    L100_RELEASE_S[r] * L100_PROGRAM_SPREAD[r],
                ];
                self.w_fast = L100_FAST_WEIGHT;
                self.post = db_to_lin_fast((v[0] - L100_GAIN_UNITY) * L100_GAIN_DB_PER_STEP);
                Some(L100_TUBE)
            }
            OptoKind::F670 => {
                self.pre = db_to_lin_fast(v[0]);
                self.post = db_to_lin_fast(F670_AMP_DB);
                let thr = F670_THR_AT_0 - F670_THR_PER_STEP * v[1] as f64;
                self.inv_vthr = 10f64.powf(-thr / 20.0) as f32;
                let bias = (v[3] as f64).clamp(0.0, 1.0);
                self.gm_db = F670_GM_DB * (0.5 + 0.5 * bias);
                let pos = (v[2].round() as usize).clamp(1, 6) - 1;
                let cas = &mut self.cascade;
                cas.att = c(F670_ATTACK_S[pos]);
                cas.rel[0] = c(F670_RELEASE_S[pos]);
                cas.n = match pos {
                    4 => 2,
                    5 => 3,
                    _ => 1,
                };
                cas.charge[1] = c(F670_MULTI_CHARGE_S);
                cas.rel[1] = c(F670_MULTI_RELEASE_S);
                cas.charge[2] = c(F670_SUSTAINED_CHARGE_S);
                cas.rel[2] = c(F670_SUSTAINED_RELEASE_S);
                for i in cas.n..3 {
                    cas.s[i] = 0.0;
                }
                Some((F670_TUBE_DRIVE, F670_BIAS_ASYM * (1.0 - bias)))
            }
            OptoKind::Bliss => {
                self.thr_db = v[0] as f64;
                self.slope = 1.0 - 1.0 / v[1] as f64;
                self.att_alpha = 1.0 - c(v[2] as f64 * 1e-3);
                self.rel_c = c(v[3] as f64 * 1e-3);
                self.afast = v[4] != 0.0;
                self.alog = v[5] != 0.0;
                self.gr_cap = if v[6] != 0.0 {
                    -(v[7] as f64)
                } else {
                    BLISS_GR_MAX_DB
                };
                None
            }
        };
        self.tube = if self.colour {
            tube.map(|(d, b)| TubeParams::new(d, b, fs))
        } else {
            None
        };
    }

    #[inline]
    fn key(sc: Option<(&[f32], &[f32])>, l: &[f32], r: &[f32], i: usize) -> (f32, f32) {
        match sc {
            Some((a, b)) => (
                a.get(i).copied().unwrap_or(0.0),
                b.get(i).copied().unwrap_or(0.0),
            ),
            None => (l[i], r[i]),
        }
    }

    fn process_leveler(&mut self, l: &mut [f32], r: &mut [f32], sc: Option<(&[f32], &[f32])>) {
        let n = l.len().min(r.len());
        // Programme-dependent slow-part release from the cell memory (once per block).
        let [lo, hi] = self.slow_tau;
        let slow_c = one_pole_coeff(lo * (hi / lo).powf(self.mem), self.fs);
        let (att, rel_f, wf) = (self.att_c, self.rel_f_c, self.w_fast);
        let (k, thr, w, det_c, post) = (self.k, self.thr_db, self.knee_w, self.det_c, self.post);
        let (mc, md) = (self.mem_charge_c, self.mem_decay_c);
        for i in 0..n {
            let (kl, kr) = Self::key(sc, l, r, i);
            let g = self.g_prev;
            let (pl, pr) = ((kl * g) * (kl * g), (kr * g) * (kr * g));
            let p = if det_c > 0.0 {
                self.ms[0] = (pl + det_c * (self.ms[0] - pl)).max(POWER_FLOOR);
                self.ms[1] = (pr + det_c * (self.ms[1] - pr)).max(POWER_FLOOR);
                self.ms[0].max(self.ms[1])
            } else {
                pl.max(pr).max(POWER_FLOOR)
            };
            let level = 0.5 * lin_to_db_fast(p) as f64;
            let target = k * knee(level - thr, w);
            let ca = if target > self.fast { att } else { rel_f };
            self.fast = target + ca * (self.fast - target);
            let cs = if target > self.slow { att } else { slow_c };
            self.slow = target + cs * (self.slow - target);
            let gr = wf * self.fast + (1.0 - wf) * self.slow;
            let mt = (target / MEM_FULL_GR_DB).min(1.0);
            let cm = if mt > self.mem { mc } else { md };
            self.mem = mt + cm * (self.mem - mt);
            let g = db_to_lin_fast(-gr as f32);
            self.g_prev = g;
            l[i] *= g * post;
            r[i] *= g * post;
            self.gr_now = gr as f32;
        }
    }

    fn process_f670(&mut self, l: &mut [f32], r: &mut [f32], sc: Option<(&[f32], &[f32])>) {
        let n = l.len().min(r.len());
        let (pre, post, iv, gm) = (self.pre, self.post, self.inv_vthr, self.gm_db);
        for i in 0..n {
            let (kl, kr) = Self::key(sc, l, r, i);
            let g = self.g_prev * pre;
            let e = (kl * g).abs().max((kr * g).abs()) * iv;
            let target = (gm * knee(e as f64 - 1.0, F670_KNEE)).min(F670_GR_MAX_DB);
            let gr = self.cascade.tick(target);
            let g = db_to_lin_fast(-gr as f32);
            self.g_prev = g;
            let t = pre * g * post;
            l[i] *= t;
            r[i] *= t;
            self.gr_now = gr as f32;
        }
    }

    fn process_bliss(&mut self, l: &mut [f32], r: &mut [f32], sc: Option<(&[f32], &[f32])>) {
        let n = l.len().min(r.len());
        let (thr, slope, cap) = (self.thr_db, self.slope, self.gr_cap);
        let (att_a, rel_c) = (self.att_alpha, self.rel_c);
        for i in 0..n {
            let (kl, kr) = Self::key(sc, l, r, i);
            let level = lin_to_db_fast(kl.abs().max(kr.abs()).max(1e-12)) as f64;
            let target = (slope * (level - thr).max(0.0)).min(cap);
            let gr_prev = self.fast;
            let alpha_att = if self.afast {
                (att_a * (1.0 + (target - gr_prev).max(0.0) / BLISS_AUTO_FAST_DB)).min(1.0)
            } else {
                att_a
            };
            let gr = if self.alog {
                // Anti-log: ballistics on the linear gain (recovery slow at first, fast near unity in dB).
                let gt = db_to_lin_fast(-target as f32) as f64;
                let lg = self.lin_g;
                self.lin_g = if gt < lg {
                    lg + alpha_att * (gt - lg)
                } else {
                    gt + rel_c * (lg - gt)
                };
                -(lin_to_db_fast(self.lin_g.max(1e-12) as f32) as f64)
            } else {
                if target > gr_prev {
                    gr_prev + alpha_att * (target - gr_prev)
                } else {
                    target + rel_c * (gr_prev - target)
                }
            };
            let gr = gr.max(0.0);
            self.fast = gr;
            if !self.alog {
                self.lin_g = db_to_lin_fast(-gr as f32) as f64;
            }
            let g = db_to_lin_fast(-gr as f32);
            l[i] *= g;
            r[i] *= g;
            self.gr_now = gr as f32;
        }
    }
}

impl Model for OptoComp {
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
        match self.kind {
            OptoKind::La | OptoKind::L100 => self.process_leveler(left, right, sidechain),
            OptoKind::F670 => self.process_f670(left, right, sidechain),
            OptoKind::Bliss => self.process_bliss(left, right, sidechain),
        }
        if let Some(p) = self.tube {
            let n = left.len().min(right.len());
            let [tl, tr] = &mut self.tube_ch;
            for (a, b) in left[..n].iter_mut().zip(right[..n].iter_mut()) {
                *a = tl.tick(&p, *a);
                *b = tr.tick(&p, *b);
            }
        }
    }

    fn reset(&mut self) {
        self.apply();
        self.ms = [POWER_FLOOR; 2];
        self.fast = 0.0;
        self.slow = 0.0;
        self.mem = 0.0;
        self.cascade.s = [0.0; 3];
        self.lin_g = 1.0;
        self.g_prev = 1.0;
        self.gr_now = 0.0;
        self.tube_ch = [TubeCh::default(); 2];
    }

    fn set_param(&mut self, slot: u16, value: ParamValue) {
        let slots = self.kind.slots();
        let Some(def) = (slot as usize).checked_sub(1).and_then(|i| slots.get(i)) else {
            return;
        };
        let x = match def.kind {
            SlotKind::Range(lo, hi) => value.as_f32().clamp(lo, hi),
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

    fn model(kind: OptoKind, params: &[(u16, ParamValue)]) -> OptoComp {
        let mut m = OptoComp::new(kind);
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

    fn dc(db: f64) -> f32 {
        10f64.powf(db / 20.0) as f32
    }

    /// Run a piecewise-constant DC level sequence `(level_db, seconds)` from reset; returns the gain (dB)
    /// trajectory of the whole run.
    fn run_levels(m: &mut OptoComp, seq: &[(f64, f64)]) -> Vec<f64> {
        let mut x = Vec::new();
        for &(l, s) in seq {
            x.extend(std::iter::repeat_n(dc(l), (s * FS) as usize));
        }
        m.reset();
        let y = run_mono(m, &x);
        y.iter()
            .zip(&x)
            .map(|(&o, &i)| db((o / i) as f64))
            .collect()
    }

    /// Seconds after sample `from` until the GR has shrunk to `1 - frac` of its value just before `from`
    /// (gain `gain0_db` = no reduction); `None` if it never does.
    fn recovery_s(g: &[f64], from: usize, gain0_db: f64, frac: f64) -> Option<f64> {
        let gr0 = gain0_db - g[from - 1];
        g[from..]
            .iter()
            .position(|&v| gain0_db - v <= (1.0 - frac) * gr0)
            .map(|i| i as f64 / FS)
    }

    /// Seconds after `from` until the gain change reaches `frac` of its final value (attack).
    fn attack_s(g: &[f64], from: usize, frac: f64) -> f64 {
        let fin = *g.last().unwrap();
        let g0 = g[from - 1];
        let i = g[from..]
            .iter()
            .position(|&v| (v - g0) / (fin - g0) >= frac)
            .unwrap();
        i as f64 / FS
    }

    // ---- LA ------------------------------------------------------------------------------------------------

    #[test]
    fn la_static_curve_comp_lim_and_knobs() {
        // PEAK 100 -> threshold -30 dBFS at the cell output.
        let mut m = model(OptoKind::La, &[(2, ParamValue::Float(100.0))]);
        assert!((m.threshold_dbfs() + 30.0).abs() < 1e-9);
        // COMP: steady-state ratio 3:1 well above the knee (output-referred feedback law).
        let (a, b) = (m.static_out_db(-10.0), m.static_out_db(0.0));
        assert!((10.0 / (b - a) - LA_COMP_RATIO).abs() < 1e-3, "{a} {b}");
        // Below the knee: unity.
        assert!((m.static_out_db(-60.0) + 60.0).abs() < 1e-9);
        // DSP settles to the law (DC through the tube stage is unchanged).
        let levels = [-50.0, -35.0, -25.0, -10.0, 0.0];
        let meas = static_curve(&mut m, FS, &levels, 1.5);
        for (&l, &o) in levels.iter().zip(&meas) {
            assert!((o - m.static_out_db(l)).abs() < 0.1, "COMP {l}: {o}");
        }
        // LIM: 20:1 asymptotically (output-referred knee: the full ratio is reached only once the output is
        // above the knee, i.e. far above threshold); at 0 dBFS the local ratio is already > 8:1.
        m.set_param(3, ParamValue::Enum(1));
        let (a, b) = (m.static_out_db(110.0), m.static_out_db(120.0));
        assert!((10.0 / (b - a) - LA_LIM_RATIO).abs() < 0.01);
        let s0 = (m.static_out_db(0.01) - m.static_out_db(0.0)) / 0.01;
        assert!(s0 < 1.0 / 8.0, "LIM slope at 0 dBFS {s0}");
        let meas = static_curve(&mut m, FS, &levels, 1.5);
        for (&l, &o) in levels.iter().zip(&meas) {
            assert!((o - m.static_out_db(l)).abs() < 0.1, "LIM {l}: {o}");
        }
        // Soft knee: the local slope moves smoothly from 1 to 1/20 (monotonic decrease).
        let mut prev_slope = 1.01;
        for i in 0..80 {
            let l = -45.0 + i as f64 * 0.25;
            let s = (m.static_out_db(l + 0.01) - m.static_out_db(l)) / 0.01;
            assert!(s <= prev_slope + 1e-6 && s > 0.04, "{l}: {s}");
            prev_slope = s;
        }
        // PEAK REDUCTION 0 = no compression; 20 -> threshold -16 dBFS.
        m.set_param(2, ParamValue::Float(0.0));
        assert!(m.static_out_db(0.0).abs() < 1e-9);
        m.set_param(2, ParamValue::Float(20.0));
        assert!((m.threshold_dbfs() + 16.0206).abs() < 1e-3);
        // GAIN is output gain independent of compression: default 40 = unity, 100 = +24 dB.
        m.set_param(2, ParamValue::Float(0.0));
        m.set_param(1, ParamValue::Float(100.0));
        assert!((m.static_out_db(-20.0) - 4.0).abs() < 0.01);
        m.set_param(2, ParamValue::Float(100.0));
        let with_gain = m.static_out_db(-5.0);
        m.set_param(1, ParamValue::Float(40.0));
        assert!((with_gain - m.static_out_db(-5.0) - 24.0).abs() < 0.01);
    }

    #[test]
    fn la_attack_matches_published_10ms() {
        for mode in [0u32, 1] {
            let mut m = model(
                OptoKind::La,
                &[(2, ParamValue::Float(100.0)), (3, ParamValue::Enum(mode))],
            );
            m.set_colour(false);
            let g = run_levels(&mut m, &[(-60.0, 0.2), (-15.0, 1.5)]);
            let t = attack_s(&g, (0.2 * FS) as usize, 0.632) * 1e3;
            assert!(
                (5.0..=15.0).contains(&t),
                "mode {mode}: attack {t:.1} ms vs 10 ms"
            );
        }
    }

    #[test]
    fn la_two_stage_programme_dependent_release_matches_published() {
        // UA LA-2A spec: 50 % release ~0.06 s; complete release 0.5..5 s depending on history.
        let mut m = model(OptoKind::La, &[(2, ParamValue::Float(100.0))]);
        m.set_colour(false);
        let mut results = Vec::new();
        for hold in [0.1, 12.0] {
            // ~10 dB GR (input -15 dBFS), then near silence.
            let g = run_levels(&mut m, &[(-15.0, hold), (-90.0, 8.0)]);
            let from = (hold * FS) as usize;
            let gr0 = -g[from - 1];
            assert!(gr0 > 8.0, "GR {gr0}");
            let half = recovery_s(&g, from, 0.0, 0.5).unwrap();
            let complete = recovery_s(&g, from, 0.0, 0.95).unwrap();
            assert!(
                (0.04..=0.08).contains(&half),
                "hold {hold}: 50 % after {half:.3} s"
            );
            results.push(complete);
        }
        let (short, long) = (results[0], results[1]);
        assert!((0.4..=1.0).contains(&short), "short history {short:.2} s");
        assert!((3.5..=5.5).contains(&long), "long history {long:.2} s");
    }

    // ---- L100 ----------------------------------------------------------------------------------------------

    #[test]
    fn l100_static_curve_gain_reduction_knob() {
        let mut m = model(OptoKind::L100, &[(2, ParamValue::Float(5.0))]);
        // GAIN RED 5: threshold -12 dBFS, ratio 4:1.
        assert!((m.threshold_dbfs() + 12.0).abs() < 1e-9);
        let (a, b) = (m.static_out_db(20.0), m.static_out_db(30.0));
        assert!((10.0 / (b - a) - 4.0).abs() < 1e-3);
        let levels = [-40.0, -15.0, -5.0, 0.0];
        let meas = static_curve(&mut m, FS, &levels, 1.0);
        for (&l, &o) in levels.iter().zip(&meas) {
            assert!((o - m.static_out_db(l)).abs() < 0.15, "{l}: {o}");
        }
        // More GAIN RED = lower threshold and higher ratio = more GR.
        let gr = |m: &mut OptoComp| -6.0 - m.static_out_db(-6.0);
        let g5 = gr(&mut m);
        m.set_param(2, ParamValue::Float(10.0));
        let g10 = gr(&mut m);
        m.set_param(2, ParamValue::Float(0.0));
        let g0 = gr(&mut m);
        assert!(g0 < 0.01 && g5 > 2.0 && g10 > g5 + 5.0, "{g0} {g5} {g10}");
        // GAIN: default 3 = unity, 10 = +21 dB.
        m.set_param(1, ParamValue::Float(10.0));
        assert!((m.static_out_db(-40.0) + 19.0).abs() < 0.01);
    }

    #[test]
    fn l100_attack_release_switches() {
        let mut atts = Vec::new();
        for a in 0..3u32 {
            let mut m = model(
                OptoKind::L100,
                &[(2, ParamValue::Float(8.0)), (3, ParamValue::Enum(a))],
            );
            m.set_colour(false);
            let g = run_levels(&mut m, &[(-60.0, 0.2), (0.0, 1.5)]);
            atts.push(attack_s(&g, (0.2 * FS) as usize, 0.632));
        }
        assert!(atts[0] < atts[1] && atts[1] < atts[2], "{atts:?}");
        assert!(atts[0] < 0.002, "FAST {}", atts[0]);
        assert!((0.05..=0.15).contains(&atts[2]), "SLOW {}", atts[2]);
        for (r, &e) in L100_RELEASE_S.iter().enumerate() {
            let mut m = model(
                OptoKind::L100,
                &[(2, ParamValue::Float(8.0)), (4, ParamValue::Enum(r as u32))],
            );
            m.set_colour(false);
            // Short burst (fresh cell): 63 % release ~ the switch's time constant.
            let g = run_levels(&mut m, &[(0.0, 0.1), (-90.0, 12.0)]);
            let t = recovery_s(&g, (0.1 * FS) as usize, 0.0, 0.632).unwrap();
            assert!((t - e).abs() < 0.25 * e, "release {r}: {t} vs {e}");
            // Long programme: slower release on MED / SLOW only.
            let g = run_levels(&mut m, &[(0.0, 8.0), (-90.0, 30.0)]);
            let tl = recovery_s(&g, (8.0 * FS) as usize, 0.0, 0.632).unwrap();
            if r == 0 {
                assert!((tl - t).abs() < 0.1 * t, "FAST: {tl} vs {t}");
            } else {
                assert!(tl > 1.4 * t, "release {r}: long {tl} vs short {t}");
            }
        }
    }

    // ---- F670 ----------------------------------------------------------------------------------------------

    #[test]
    fn f670_static_curve_vari_mu_knee_and_controls() {
        // Defaults: INPUT -10 dB, THR 5 -> -12 dBFS at the cell, amp +10 dB.
        let mut m = model(OptoKind::F670, &[]);
        assert!((m.threshold_dbfs() + 12.0).abs() < 1e-4);
        assert!(
            (m.static_out_db(-40.0) + 40.0).abs() < 1e-3,
            "unity below thr"
        );
        let levels = [-30.0, -10.0, 0.0, 10.0];
        let meas = static_curve(&mut m, FS, &levels, 1.0);
        for (&l, &o) in levels.iter().zip(&meas) {
            assert!((o - m.static_out_db(l)).abs() < 0.1, "{l}: {o}");
        }
        // Soft knee from the gm law: local ratio rises with GR (1:1 at threshold, >4:1 at 20 dB GR).
        let ratio_at =
            |m: &mut OptoComp, l: f64| 0.1 / (m.static_out_db(l + 0.1) - m.static_out_db(l));
        let gr_at = |m: &mut OptoComp, l: f64| {
            let g0 = m.static_out_db(-100.0) + 100.0;
            l + g0 - m.static_out_db(l)
        };
        let mut last = 0.0;
        for l in [-4.0, -1.0, 2.0, 6.0, 12.0, 20.0, 30.0] {
            let r = ratio_at(&mut m, l);
            assert!(r > last, "ratio not increasing at {l}: {r}");
            last = r;
        }
        assert!(ratio_at(&mut m, -4.0) < 1.3);
        let l20 = (0..600)
            .map(|i| -2.0 + i as f64 * 0.1)
            .find(|&l| gr_at(&mut m, l) >= 20.0)
            .unwrap();
        assert!(ratio_at(&mut m, l20) > 4.0);
        // INPUT drives the cell harder (more GR).
        let g_def = gr_at(&mut m, 0.0);
        m.set_param(1, ParamValue::Float(0.0));
        let g_hot = gr_at(&mut m, 0.0);
        assert!(g_hot > g_def + 3.0, "{g_def} {g_hot}");
        // BIAS below CAL softens the gain law.
        m.set_param(4, ParamValue::Float(0.0));
        assert!(gr_at(&mut m, 0.0) < g_hot - 1.0);
        // THR 0 -> +6 dBFS threshold: no GR at 0 dBFS with INPUT 0.
        m.set_param(2, ParamValue::Float(0.0));
        assert!(gr_at(&mut m, 0.0) < 0.01);
    }

    #[test]
    fn f670_fixed_time_constants_match_published() {
        for pos in 1..=4usize {
            let mut m = model(OptoKind::F670, &[(3, ParamValue::Int(pos as i32))]);
            m.set_colour(false);
            let rel = F670_RELEASE_S[pos - 1];
            let g = run_levels(&mut m, &[(-40.0, 0.05), (10.0, 0.5), (-90.0, 4.0 * rel)]);
            // Attack: 63 % of the GR within a few attack constants (feedback loop, peak rectifier).
            let a = attack_s(&g[..(0.55 * FS) as usize], (0.05 * FS) as usize, 0.632);
            assert!(a < 5.0 * F670_ATTACK_S[pos - 1], "pos {pos}: attack {a}");
            // Release: control-voltage RC = GR in dB decays with the published constant.
            let t = recovery_s(&g, (0.55 * FS) as usize, 0.0, 0.632).unwrap();
            assert!(
                (t - rel).abs() < 0.03 * rel,
                "pos {pos}: release {t} vs {rel}"
            );
        }
    }

    #[test]
    fn f670_programme_dependent_positions() {
        // Position 6: 0.3 s for individual peaks, 10 s for multiple peaks, 25 s for a consistently high level.
        let mut m = model(OptoKind::F670, &[(3, ParamValue::Int(6))]);
        m.set_colour(false);
        let single = {
            let g = run_levels(&mut m, &[(10.0, 0.02), (-90.0, 3.0)]);
            recovery_s(&g, (0.02 * FS) as usize, 0.0, 0.632).unwrap()
        };
        assert!((single - 0.3).abs() < 0.06, "single peak {single}");
        let multi = {
            let mut seq = Vec::new();
            for _ in 0..8 {
                seq.push((10.0, 0.02));
                seq.push((-90.0, 0.13));
            }
            seq.pop();
            seq.push((-90.0, 40.0));
            let g = run_levels(&mut m, &seq);
            let from = (((8.0 * 0.15) - 0.13) * FS) as usize;
            recovery_s(&g, from, 0.0, 0.632).unwrap()
        };
        assert!(multi > 5.0 * single, "multiple peaks {multi}");
        let sustained = {
            let g = run_levels(&mut m, &[(10.0, 15.0), (-90.0, 45.0)]);
            recovery_s(&g, (15.0 * FS) as usize, 0.0, 0.632).unwrap()
        };
        assert!(
            sustained > 1.3 * multi && sustained > 10.0,
            "sustained {sustained} vs multi {multi}"
        );
        // Position 5: 2 s single peaks, slower after sustained programme.
        m.set_param(3, ParamValue::Int(5));
        m.commit();
        let g = run_levels(&mut m, &[(10.0, 0.02), (-90.0, 8.0)]);
        let t = recovery_s(&g, (0.02 * FS) as usize, 0.0, 0.632).unwrap();
        assert!((t - 2.0).abs() < 0.3, "pos 5 single {t}");
        let g = run_levels(&mut m, &[(10.0, 5.0), (-90.0, 30.0)]);
        let tl = recovery_s(&g, (5.0 * FS) as usize, 0.0, 0.632).unwrap();
        assert!(tl > 6.0, "pos 5 sustained {tl}");
    }

    // ---- BLISS ---------------------------------------------------------------------------------------------

    #[test]
    fn bliss_static_negative_ratios_and_gr_limit() {
        let mut m = model(
            OptoKind::Bliss,
            &[(1, ParamValue::Float(-30.0)), (2, ParamValue::Enum(3))],
        );
        let levels = [-40.0, -20.0, 0.0];
        let meas = static_curve(&mut m, FS, &levels, 0.5);
        for (&l, &o) in levels.iter().zip(&meas) {
            let e = if l > -30.0 {
                -30.0 + (l + 30.0) / 2.0
            } else {
                l
            };
            assert!((o - e).abs() < 0.05, "{l}: {o} vs {e}");
        }
        // -1:1: output falls dB for dB above threshold; -0.3: slope 1 - 1/(-0.3).
        m.set_param(2, ParamValue::Enum(8));
        assert!((m.static_out_db(-20.0) + 40.0).abs() < 1e-6);
        m.set_param(2, ParamValue::Float(-0.3));
        assert_eq!(m.param(2), Some(-0.3));
        let slope = 1.0 - 1.0 / (-0.3f32) as f64;
        assert!((m.static_out_db(-27.0) - (-27.0 - 3.0 * slope)).abs() < 1e-4);
        let meas = static_curve(&mut m, FS, &[-27.0], 0.5);
        assert!((meas[0] - m.static_out_db(-27.0)).abs() < 0.1);
        // GR limiter caps GR at |glim|.
        m.set_param(7, ParamValue::Int(1));
        m.set_param(8, ParamValue::Float(-6.0));
        assert!((m.static_out_db(0.0) + 6.0).abs() < 1e-9);
        let meas = static_curve(&mut m, FS, &[0.0], 0.5);
        assert!((meas[0] + 6.0).abs() < 0.05);
        m.set_param(8, ParamValue::Float(0.0));
        assert!(m.static_out_db(0.0).abs() < 1e-9);
    }

    #[test]
    fn bliss_ballistics_log_antilog_and_auto_fast() {
        let mut m = model(
            OptoKind::Bliss,
            &[
                (1, ParamValue::Float(-40.0)),
                (2, ParamValue::Enum(6)),
                (3, ParamValue::Float(20.0)),
                (4, ParamValue::Float(300.0)),
            ],
        );
        // Log mode: dB one-poles, 63 % at the knob times.
        let g = run_levels(&mut m, &[(-60.0, 0.1), (-10.0, 0.5), (-90.0, 2.0)]);
        let a = attack_s(&g[..(0.6 * FS) as usize], (0.1 * FS) as usize, 0.632);
        assert!((a - 0.020).abs() < 0.001, "attack {a}");
        let half_log = recovery_s(&g, (0.6 * FS) as usize, 0.0, 0.5).unwrap();
        assert!((half_log - 0.3 * 2f64.ln()).abs() < 0.01, "half {half_log}");
        // Anti-log: linear-gain one-pole, so the dB recovery is front-loaded.
        m.set_param(6, ParamValue::Int(1));
        m.commit();
        let g = run_levels(&mut m, &[(-60.0, 0.1), (-10.0, 0.5), (-90.0, 2.0)]);
        let gr0 = -g[(0.6 * FS) as usize - 1];
        let half_alog = recovery_s(&g, (0.6 * FS) as usize, 0.0, 0.5).unwrap();
        let g_half = 10f64.powf(-gr0 / 40.0);
        let g0 = 10f64.powf(-gr0 / 20.0);
        let expect = 0.3 * ((1.0 - g0) / (1.0 - g_half)).ln();
        assert!((half_alog - expect).abs() < 0.01, "{half_alog} vs {expect}");
        assert!(half_alog < 0.5 * half_log);
        // Auto Fast: large overshoots attack faster.
        m.set_param(6, ParamValue::Int(0));
        m.set_param(5, ParamValue::Int(1));
        m.commit();
        let g = run_levels(&mut m, &[(-60.0, 0.1), (-10.0, 0.5)]);
        let af = attack_s(&g, (0.1 * FS) as usize, 0.632);
        assert!(af < 0.5 * a, "auto fast {af} vs {a}");
    }

    // ---- tube stage ----------------------------------------------------------------------------------------

    /// |DFT| amplitude at integer frequency `f` over `x` (whole seconds, so integer Hz are exact bins).
    fn bin(x: &[f32], f: f64) -> f64 {
        let w = 2.0 * core::f64::consts::PI * f / FS;
        let (mut re, mut im) = (0.0, 0.0);
        for (n, &v) in x.iter().enumerate() {
            re += v as f64 * (w * n as f64).cos();
            im -= v as f64 * (w * n as f64).sin();
        }
        2.0 * (re * re + im * im).sqrt() / x.len() as f64
    }

    fn shaped(p: &TubeParams, f: f64, amp: f64, naive: bool) -> Vec<f32> {
        let mut ch = TubeCh::default();
        let n0 = (0.25 * FS) as usize;
        let y: Vec<f32> = (0..n0 + FS as usize)
            .map(|n| {
                let x = (amp * (2.0 * core::f64::consts::PI * f * n as f64 / FS).sin()) as f32;
                if naive {
                    ch.tick_naive(p, x)
                } else {
                    ch.tick(p, x)
                }
            })
            .collect();
        y[n0..].to_vec()
    }

    #[test]
    fn tube_stage_is_mild_dc_transparent_and_unity_small_signal() {
        for (d, b) in [LA_TUBE, L100_TUBE, (F670_TUBE_DRIVE, 0.0)] {
            let p = TubeParams::new(d, b, FS);
            // DC passes unchanged once the residual high-pass settles.
            let mut ch = TubeCh::default();
            let mut y = 0.0;
            for _ in 0..48_000 {
                y = ch.tick(&p, 0.7);
            }
            assert!((y - 0.7).abs() < 1e-5, "DC {y}");
            // 1 kHz at 0 dBFS: ~unity fundamental, harmonics between -60 and -38 dBc.
            let y = shaped(&p, 1000.0, 1.0, false);
            let f1 = bin(&y, 1000.0);
            assert!(db(f1).abs() < 0.3, "fundamental {}", db(f1));
            let h2 = db(bin(&y, 2000.0) / f1);
            let h3 = db(bin(&y, 3000.0) / f1);
            assert!((-60.0..=-38.0).contains(&h3), "h3 {h3}");
            if b > 0.0 {
                assert!((-60.0..=-38.0).contains(&h2), "h2 {h2}");
            } else {
                assert!(h2 < -100.0, "push-pull h2 {h2}");
            }
            // -40 dBFS: essentially linear.
            let y = shaped(&p, 1000.0, 0.01, false);
            assert!(db(bin(&y, 3000.0) / bin(&y, 1000.0)) < -120.0);
        }
    }

    #[test]
    fn tube_stage_adaa_suppresses_aliasing() {
        // 15 kHz at +6 dBFS into the LA stage (cubic region): the 3rd harmonic (45 kHz) folds to 3 kHz, the
        // 2nd (30 kHz) to 18 kHz.
        let p = TubeParams::new(LA_TUBE.0, LA_TUBE.1, FS);
        let (f0, amp) = (15_000.0, 2.0);
        let ad = shaped(&p, f0, amp, false);
        let nv = shaped(&p, f0, amp, true);
        let (fa, fv) = (bin(&ad, f0), bin(&nv, f0));
        assert!(
            (db(fa) - db(fv)).abs() < 0.5,
            "fundamental {} vs {}",
            db(fa),
            db(fv)
        );
        for alias in [3000.0, 18_000.0] {
            let a = db(bin(&ad, alias) / fa);
            let n = db(bin(&nv, alias) / fv);
            println!("{alias} Hz: ADAA {a:.1} dBc vs naive {n:.1} dBc");
            assert!(
                a < n - 10.0,
                "{alias} Hz: ADAA {a:.1} dBc vs naive {n:.1} dBc"
            );
            assert!(a < -45.0, "{alias} Hz: ADAA {a:.1} dBc");
        }
        // +16 dBFS (into the clipping region): the 5th (75 kHz) folds to 21 kHz, the 7th (105 kHz) to 9 kHz.
        let ad = shaped(&p, f0, 6.0, false);
        let nv = shaped(&p, f0, 6.0, true);
        let (fa, fv) = (bin(&ad, f0), bin(&nv, f0));
        for alias in [3000.0, 21_000.0, 9000.0] {
            let a = db(bin(&ad, alias) / fa);
            let n = db(bin(&nv, alias) / fv);
            println!("clip {alias} Hz: ADAA {a:.1} dBc vs naive {n:.1} dBc");
            assert!(
                a < n - 6.0,
                "clip {alias} Hz: ADAA {a:.1} dBc vs naive {n:.1} dBc"
            );
        }
        // In-band harmonics of a low tone are kept (ADAA only removes what folds).
        let ad = shaped(&p, 1000.0, amp, false);
        let nv = shaped(&p, 1000.0, amp, true);
        for h in [2000.0, 3000.0] {
            let (a, n) = (db(bin(&ad, h)), db(bin(&nv, h)));
            assert!((a - n).abs() < 0.5, "{h}: {a} vs {n}");
        }
    }

    // ---- shared --------------------------------------------------------------------------------------------

    #[test]
    fn gr_meter_matches_wing_edit_mapping() {
        // data/reference-curves/dyn-models.json (comp_gain_la2a / f670 / mpress / tla100a), embedded.
        let grs = [0.0, 1.0, 6.0, 15.0, 20.0, 30.0, 80.0];
        let pos = [0.0, 0.05, 0.3, 0.75, 1.0, 1.0, 1.0];
        for (i, &g) in grs.iter().enumerate() {
            for kind in [OptoKind::La, OptoKind::F670, OptoKind::Bliss] {
                assert!((gr_meter(kind, g) - pos[i]).abs() < 1e-6);
            }
            assert!((gr_meter(OptoKind::L100, g) + pos[i]).abs() < 1e-6);
        }
    }

    #[test]
    fn telemetry_equals_applied_gain() {
        for kind in OptoKind::ALL {
            let mut m = model(kind, &[]);
            if kind == OptoKind::La {
                m.set_param(2, ParamValue::Float(60.0));
            }
            m.set_colour(false);
            let x = vec![0.9f32; 24_000];
            let y = run_mono(&mut m, &x);
            let post = m.static_out_db(-120.0) + 120.0;
            let applied = post - db((y[23_999] / 0.9) as f64);
            let gr = m.telemetry().gain_reduction_db as f64;
            assert!(gr > 0.5, "{kind:?}: {gr}");
            assert!((applied - gr).abs() < 1e-3, "{kind:?}: {applied} vs {gr}");
            assert!(m.telemetry().engaged);
        }
    }

    #[test]
    fn stereo_link_sidechain_and_param_handling() {
        for kind in OptoKind::ALL {
            let mut m = model(kind, &[]);
            m.set_colour(false);
            if kind == OptoKind::La {
                m.set_param(2, ParamValue::Float(100.0));
            }
            let mut l = vec![0.9f32; 9600];
            let mut r = vec![0.001f32; 9600];
            for (a, b) in l.chunks_mut(64).zip(r.chunks_mut(64)) {
                m.process(a, b, None);
            }
            let gl = db((l[9599] / 0.9) as f64);
            let gr = db((r[9599] / 0.001) as f64);
            assert!((gl - gr).abs() < 1e-3, "{kind:?}");
            assert!(m.telemetry().gain_reduction_db > 0.5, "{kind:?}");
            // Silent external key: no gain reduction.
            m.reset();
            let (mut l, mut r) = (vec![0.9f32; 64], vec![0.9f32; 64]);
            let k = [0.0f32; 64];
            m.process(&mut l, &mut r, Some((&k, &k)));
            assert_eq!(m.telemetry().gain_reduction_db, 0.0, "{kind:?}");
        }
        let mut m = OptoComp::new(OptoKind::Bliss);
        m.set_param(9, ParamValue::Float(1.0));
        assert_eq!(m.param(9), None);
        m.set_param(3, ParamValue::Float(0.01));
        assert_eq!(m.param(3), Some(0.4));
        m.load_defaults();
        assert_eq!(m.param(2), Some(3.0));
        let mut m = OptoComp::new(OptoKind::F670);
        m.set_param(3, ParamValue::Int(9));
        assert_eq!(m.param(3), Some(6.0));
        assert_eq!(m.param(4), Some(1.0));
        let mut m = OptoComp::new(OptoKind::L100);
        m.set_param(3, ParamValue::Enum(7));
        assert_eq!(m.param(3), Some(2.0));
        assert_eq!(OptoKind::from_id("BLISS"), Some(OptoKind::Bliss));
        assert_eq!(OptoKind::from_id("COMP"), None);
    }
}
