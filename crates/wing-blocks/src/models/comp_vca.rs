//! VCA compressors (T04B): `B160` (BDX 160 COMP, dbx 160), `B560` (BDX 560 EASY, dbx 160X/560A),
//! `D241C` (DRAW MORE COMP, Drawmer DL241), `9000C` (SOUL 9000, SSL 9000 J channel compressor) and `SBUS`
//! (SOUL BUS COMP, SSL G-series stereo bus compressor). Parameters and ranges: `docs/research/models.md` §3.2
//! and `data/schema/wing-tree.json` (slot order, enum tables, defaults).
//!
//! All five share one feed-forward VCA core (stereo-linked: one gain from the louder channel):
//! ```text
//! key -> power detector (RMS: one-pole mean square | PEAK: x^2) -> level dB -> static curve (thr, ratio, knee)
//!     -> gain-reduction ballistics in dB (attack / release one-poles, optional two-stage AUTO release)
//!     -> VCA gain -> [D241C: zero-overshoot peak limiter] -> out
//! ```
//! WING applies make-up gain, mix and the key filter in the slot wrapper, so none of these models has an
//! output/auto-gain stage of its own (the hardware's output-gain knobs map to the wrapper `gain`).
//!
//! Per model (constants below cite their sources; the task Log lists the references):
//! - **B160** dbx 160: true-RMS detector whose log-domain capacitor gives the dbx programme-dependent timing:
//!   a linear mean-square one-pole with `tau = 10 / (ln 10 * 125 dB/s)` releases at exactly 125 dB/s and
//!   attacks (63 % of the dB change) in 15.7 / 6.6 / 2.8 ms for a 10 / 20 / 30 dB rise, matching the dbx
//!   160A spec (15 / 5 / 3 ms, 125 dB/s). Hard knee; threshold in volts RMS (10 mV..5 V).
//! - **B560** dbx 160X / 560A: same detector and timing, threshold in dBu, OverEasy soft knee (`easy`) or hard
//!   knee, ratios beyond infinity (negative: output falls as input rises above threshold).
//! - **D241C** Drawmer DL241: soft knee in which the ratio ramps in over the first 10 dB above threshold,
//!   manual attack 0.5..100 ms / release 0.05..5 s or AUTO (programme-dependent), followed by the
//!   DL241's zero-attack, zero-overshoot peak limiter (`lim`, `lrel`).
//! - **9000C** SSL 9000 J channel compressor: RMS detector or PEAK, hard knee, attack 30 ms (normal) or 3 ms
//!   (`fast`) at 20 dB GR, release 0.1..4 s. Threshold in dBu.
//! - **SBUS** SSL G bus compressor: stepped attack 0.1..30 ms, release 0.1..1.6 s or AUTO (two-stage
//!   programme-dependent release), ratios 1.5..10, full-wave (peak) detector, hard knee, threshold in dBFS.
//!
//! Threshold calibration: models whose hardware calibrates the threshold in dBu / volts (B160, B560, 9000C)
//! use [`DBU_AT_0DBFS`] (EBU R68 alignment, 0 dBu = -18 dBFS). The native-style ones (D241C, SBUS) use dBFS.
//!
//! GR telemetry is in dB; [`gr_meter`] reproduces WING-Edit's GR-meter mapping (`comp_gain_<m>`,
//! `data/reference-curves/dyn-models.json`).
//!
//! Real-time: no allocation; `set_param` stages values, `commit`/`process`/`reset` apply them (a few
//! `exp`/`log` per change), `process` is per-sample arithmetic with the fast dB helpers of `wing_dsp::gain`.

use crate::model::{Model, ModelTelemetry, ParamValue};
use wing_dsp::gain::{db_to_lin_fast, lin_to_db_fast};
use wing_dsp::smooth::one_pole_coeff;

/// dBu level that corresponds to 0 dBFS for dBu-calibrated thresholds (EBU R68: 0 dBu = -18 dBFS).
/// Assumption: WING's internal alignment of these models is unpublished.
pub const DBU_AT_0DBFS: f32 = 18.0;
/// 0 dBu in volts RMS (sqrt(0.6) V: 1 mW into 600 ohm).
pub const DBU_REF_VOLTS: f32 = 0.774_596_7;
/// dbx 160A release rate, dB/s ("125 dB/sec rate", dbx 160A spec sheet).
pub const DBX_RELEASE_DB_PER_S: f64 = 125.0;
/// Mean-square time constant of the dbx RMS detector: a mean-square one-pole decays in dB at
/// `10 / (ln 10 * tau)` dB/s, so `tau = 10 / (ln 10 * 125)` = 34.7 ms.
pub const DBX_RMS_TAU_S: f64 = 10.0 / (core::f64::consts::LN_10 * DBX_RELEASE_DB_PER_S);
/// OverEasy knee scale (dB): the static curve is `(1 - 1/R) * w * log10(1 + 10^((L - T) / w))`, a log-domain
/// soft knee whose local ratio goes from 1:1 to R:1 (10 %..90 %) over ~11.5 dB around the threshold.
/// Assumption: dbx publishes no OverEasy width; models.md estimates 10-15 dB.
pub const OVEREASY_SCALE_DB: f32 = 6.0;
/// DL241 soft knee: "the ratio ramps in over a 10 dB input range" above the threshold (Drawmer DL241 docs).
pub const DL241_KNEE_DB: f32 = 10.0;
/// DL241 AUTO attack time constant. Assumption (Drawmer: "continually optimised", no figure).
pub const DL241_AUTO_ATTACK_S: f64 = 0.010;
/// SSL channel compressor attack: 30 ms (normal) / 3 ms (fast) "at 20 dB gain reduction" (Mix review of the
/// SSL E-series dynamics). Modelled as the dB-domain attack time constant.
pub const SSL_ATTACK_S: f64 = 0.030;
pub const SSL_FAST_ATTACK_S: f64 = 0.003;
/// SSL channel compressor RMS averaging time. Assumption (not published).
pub const SSL_RMS_TAU_S: f64 = 0.010;
/// SBUS stepped controls (schema / SSL G-comp: attack 0.1..30 ms, release 0.1..1.6 s + AUTO).
pub const SBUS_ATTACK_MS: [f32; 6] = [0.1, 0.3, 1.0, 3.0, 10.0, 30.0];
pub const SBUS_RELEASE_S: [f32; 5] = [0.1, 0.2, 0.4, 0.8, 1.6];
/// Two-stage AUTO release (SBUS AUTO, D241C AUTO): a fast stage releases with [`AUTO_FAST_RELEASE_S`]; a slow
/// stage charges from it with [`AUTO_CHARGE_S`] and releases with [`AUTO_SLOW_RELEASE_S`]; GR is the larger.
/// Short peaks recover fast, sustained compression slowly (SSL: "program-dependent, 2-stage release").
/// The three constants are assumptions bounded by the manual release range (0.1..1.6 s).
pub const AUTO_FAST_RELEASE_S: f64 = 0.1;
pub const AUTO_SLOW_RELEASE_S: f64 = 1.6;
pub const AUTO_CHARGE_S: f64 = 0.5;
/// Gain-reduction ceiling (dB), keeps negative ratios finite.
pub const GR_MAX_DB: f32 = 120.0;
/// Detector power floor (-240 dB) to keep the mean square out of denormals.
const POWER_FLOOR: f32 = 1e-24;

/// `comp` ratios of B160.
pub const B160_RATIOS: [f32; 16] = [
    1.1, 1.2, 1.3, 1.5, 1.7, 2.0, 2.5, 3.0, 3.5, 4.0, 5.0, 6.0, 8.0, 10.0, 20.0, 50.0,
];
/// `ratio` of B560: 999 = infinity:1, negative = dbx "beyond infinity".
pub const B560_RATIOS: [f32; 15] = [
    1.1, 1.2, 1.5, 2.0, 3.0, 4.0, 5.0, 7.0, 10.0, 50.0, 999.0, -5.0, -3.0, -2.0, -1.0,
];
/// `ratio` of D241C (same table as COMP).
pub const D241C_RATIOS: [f32; 17] = crate::dynamics::RATIOS;
/// `ratio` of 9000C (sampled from SSL's continuous ratio pot).
pub const SSL9000_RATIOS: [f32; 17] = [
    1.3, 1.43, 1.57, 1.8, 2.0, 2.5, 2.8, 3.3, 4.0, 5.0, 6.0, 7.0, 9.0, 12.0, 20.0, 50.0, 100.0,
];
/// `ratio` of SBUS (original console: 2/4/10).
pub const SBUS_RATIOS: [f32; 6] = [1.5, 2.0, 3.0, 4.0, 5.0, 10.0];

/// Which of the five models an instance is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VcaKind {
    B160,
    B560,
    D241C,
    Ssl9000,
    SBus,
}

impl VcaKind {
    pub const ALL: [VcaKind; 5] = [
        VcaKind::B160,
        VcaKind::B560,
        VcaKind::D241C,
        VcaKind::Ssl9000,
        VcaKind::SBus,
    ];

    /// Snapshot `mdl` token.
    pub fn id(self) -> &'static str {
        match self {
            VcaKind::B160 => "B160",
            VcaKind::B560 => "B560",
            VcaKind::D241C => "D241C",
            VcaKind::Ssl9000 => "9000C",
            VcaKind::SBus => "SBUS",
        }
    }

    pub fn from_id(id: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|k| k.id() == id)
    }

    fn slots(self) -> &'static [SlotDef] {
        match self {
            VcaKind::B160 => &B160_SLOTS,
            VcaKind::B560 => &B560_SLOTS,
            VcaKind::D241C => &D241C_SLOTS,
            VcaKind::Ssl9000 => &SSL9000_SLOTS,
            VcaKind::SBus => &SBUS_SLOTS,
        }
    }
}

/// WING-Edit GR-meter position for a gain reduction (`comp_gain_<m>`, `dyn-models.json`): B160 and SBUS drive
/// a VU-style needle `-0.3 * g` (g = linear gain); the others `GR / 20 dB`, capped at 2.0 (B560), 1.5 (D241C)
/// or 1.0 (9000C).
pub fn gr_meter(kind: VcaKind, gr_db: f32) -> f32 {
    let gr = gr_db.max(0.0);
    match kind {
        VcaKind::B160 | VcaKind::SBus => -0.3 * 10f32.powf(-gr / 20.0),
        VcaKind::B560 => (gr / 20.0).min(2.0),
        VcaKind::D241C => (gr / 20.0).min(1.5),
        VcaKind::Ssl9000 => (gr / 20.0).min(1.0),
    }
}

#[derive(Clone, Copy, Debug)]
enum SlotKind {
    /// Continuous (clamped to `[lo, hi]`).
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
const fn boolean(default: bool) -> SlotDef {
    SlotDef {
        kind: SlotKind::Bool,
        default: if default { 1.0 } else { 0.0 },
    }
}

/// Slot order and defaults from the schema (`thr comp`).
const B160_SLOTS: [SlotDef; 2] = [range(0.01, 5.0, 0.51), table(&B160_RATIOS, 3.0)];
/// `thr ratio easy`.
const B560_SLOTS: [SlotDef; 3] = [
    range(-40.0, 20.0, 0.0),
    table(&B560_RATIOS, 3.0),
    boolean(true),
];
/// `thr ratio att rel lim lrel auto`.
const D241C_SLOTS: [SlotDef; 7] = [
    range(-60.0, 0.0, -20.0),
    table(&D241C_RATIOS, 3.0),
    range(0.5, 100.0, 10.1),
    range(50.0, 5000.0, 397.0),
    range(-20.0, 0.0, 0.0),
    range(50.0, 5000.0, 397.0),
    boolean(true),
];
/// `thr ratio fast rel peak`.
const SSL9000_SLOTS: [SlotDef; 5] = [
    range(-30.0, 18.0, -10.0),
    table(&SSL9000_RATIOS, 2.8),
    boolean(false),
    range(100.0, 4000.0, 399.0),
    boolean(false),
];
/// `thr ratio att rel` (rel is an enum index, 5 = AUTO).
const SBUS_SLOTS: [SlotDef; 4] = [
    range(-40.0, 0.0, -12.0),
    table(&SBUS_RATIOS, 3.0),
    table(&SBUS_ATTACK_MS, 10.0),
    SlotDef {
        kind: SlotKind::Index(6),
        default: 5.0,
    },
];
const MAX_SLOTS: usize = 7;

#[derive(Clone, Copy, Debug, PartialEq)]
enum Knee {
    Hard,
    /// dbx OverEasy log-sum knee with the given scale (dB).
    OverEasy(f32),
    /// Ratio ramps in over the given width above the threshold (DL241).
    Above(f32),
}

/// Static gain reduction (dB, >= 0) for a detected level: `slope = 1 - 1/ratio` (> 1 for negative ratios).
#[inline]
fn static_gr(level_db: f32, thr_db: f32, slope: f32, knee: Knee) -> f32 {
    let d = level_db - thr_db;
    let x = match knee {
        Knee::Hard => d.max(0.0),
        Knee::OverEasy(w) => {
            if d > 8.0 * w {
                d
            } else if d < -8.0 * w {
                0.0
            } else {
                // w * log10(1 + 10^(d/w)), via the fast dB helpers (20 log10).
                let y = db_to_lin_fast(20.0 * d / w);
                lin_to_db_fast(1.0 + y) * (w / 20.0)
            }
        }
        Knee::Above(w) => {
            if d <= 0.0 {
                0.0
            } else if d < w {
                d * d / (2.0 * w)
            } else {
                d - 0.5 * w
            }
        }
    };
    (slope * x).min(GR_MAX_DB)
}

/// One of the five VCA compressor models (stereo-linked).
pub struct VcaComp {
    kind: VcaKind,
    fs: f64,
    /// Staged slot values (engineering units; enum slots hold the value or index, bools 0/1).
    v: [f32; MAX_SLOTS],
    dirty: bool,
    // derived
    thr_db: f32,
    slope: f32,
    knee: Knee,
    rms: bool,
    rms_c: f32,
    att_c: f32,
    rel_c: f32,
    auto: bool,
    charge_c: f32,
    slow_rel_c: f32,
    lim_on: bool,
    lim_thr_db: f32,
    lim_rel_c: f32,
    // state
    ms: [f32; 2],
    gr_fast: f32,
    gr_slow: f32,
    lim_gr: f32,
    gr_now: f32,
}

impl VcaComp {
    pub fn new(kind: VcaKind) -> Self {
        let mut m = Self {
            kind,
            fs: 48_000.0,
            v: [0.0; MAX_SLOTS],
            dirty: true,
            thr_db: 0.0,
            slope: 0.0,
            knee: Knee::Hard,
            rms: false,
            rms_c: 0.0,
            att_c: 0.0,
            rel_c: 0.0,
            auto: false,
            charge_c: 0.0,
            slow_rel_c: 0.0,
            lim_on: false,
            lim_thr_db: 0.0,
            lim_rel_c: 0.0,
            ms: [POWER_FLOOR; 2],
            gr_fast: 0.0,
            gr_slow: 0.0,
            lim_gr: 0.0,
            gr_now: 0.0,
        };
        m.load_defaults();
        m.apply();
        m
    }

    pub fn kind(&self) -> VcaKind {
        self.kind
    }

    /// Effective threshold in dBFS (after dBu / volts calibration).
    pub fn threshold_dbfs(&self) -> f32 {
        self.thr_db
    }

    /// Static output level (dBFS) for a steady input level (dBFS) under the current parameters, compressor
    /// stage only (no limiter); the law the DSP settles to for DC / RMS-steady input.
    pub fn static_out_db(&mut self, in_db: f32) -> f32 {
        self.apply();
        in_db - static_gr(in_db, self.thr_db, self.slope, self.knee)
    }

    fn apply(&mut self) {
        if !self.dirty {
            return;
        }
        self.dirty = false;
        let fs = self.fs;
        let c = |tau_s: f64| one_pole_coeff(tau_s, fs) as f32;
        let v = self.v;
        let slope_of = |r: f32| 1.0 - 1.0 / r;
        let dbu_to_dbfs = |dbu: f32| dbu - DBU_AT_0DBFS;
        // Defaults for fields not every model sets.
        self.auto = false;
        self.lim_on = false;
        self.charge_c = c(AUTO_CHARGE_S);
        self.slow_rel_c = c(AUTO_SLOW_RELEASE_S);
        match self.kind {
            VcaKind::B160 | VcaKind::B560 => {
                if self.kind == VcaKind::B160 {
                    self.thr_db = dbu_to_dbfs(20.0 * (v[0] / DBU_REF_VOLTS).log10());
                    self.knee = Knee::Hard;
                } else {
                    self.thr_db = dbu_to_dbfs(v[0]);
                    self.knee = if v[2] != 0.0 {
                        Knee::OverEasy(OVEREASY_SCALE_DB)
                    } else {
                        Knee::Hard
                    };
                }
                self.slope = slope_of(v[1]);
                // All timing comes from the RMS detector (dbx programme-dependent ballistics).
                self.rms = true;
                self.rms_c = c(DBX_RMS_TAU_S);
                self.att_c = 0.0;
                self.rel_c = 0.0;
            }
            VcaKind::D241C => {
                self.thr_db = v[0];
                self.slope = slope_of(v[1]);
                self.knee = Knee::Above(DL241_KNEE_DB);
                self.rms = false;
                if v[6] != 0.0 {
                    self.auto = true;
                    self.att_c = c(DL241_AUTO_ATTACK_S);
                    self.rel_c = c(AUTO_FAST_RELEASE_S);
                } else {
                    self.att_c = c(v[2] as f64 * 1e-3);
                    self.rel_c = c(v[3] as f64 * 1e-3);
                }
                self.lim_on = true;
                self.lim_thr_db = v[4];
                self.lim_rel_c = c(v[5] as f64 * 1e-3);
            }
            VcaKind::Ssl9000 => {
                self.thr_db = dbu_to_dbfs(v[0]);
                self.slope = slope_of(v[1]);
                self.knee = Knee::Hard;
                self.rms = v[4] == 0.0;
                self.rms_c = c(SSL_RMS_TAU_S);
                self.att_c = c(if v[2] != 0.0 {
                    SSL_FAST_ATTACK_S
                } else {
                    SSL_ATTACK_S
                });
                self.rel_c = c(v[3] as f64 * 1e-3);
            }
            VcaKind::SBus => {
                self.thr_db = v[0];
                self.slope = slope_of(v[1]);
                self.knee = Knee::Hard;
                self.rms = false;
                self.att_c = c(v[2] as f64 * 1e-3);
                let rel = v[3] as usize;
                if rel < SBUS_RELEASE_S.len() {
                    self.rel_c = c(SBUS_RELEASE_S[rel] as f64);
                } else {
                    self.auto = true;
                    self.rel_c = c(AUTO_FAST_RELEASE_S);
                }
            }
        }
        if !self.auto {
            self.gr_slow = 0.0;
        }
    }

    #[inline]
    fn detect(&mut self, kl: f32, kr: f32) -> f32 {
        let (pl, pr) = (kl * kl, kr * kr);
        if self.rms {
            let c = self.rms_c;
            self.ms[0] = (pl + c * (self.ms[0] - pl)).max(POWER_FLOOR);
            self.ms[1] = (pr + c * (self.ms[1] - pr)).max(POWER_FLOOR);
            self.ms[0].max(self.ms[1])
        } else {
            pl.max(pr).max(POWER_FLOOR)
        }
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
}

impl Model for VcaComp {
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
            let (kl, kr) = match sidechain {
                Some((a, b)) => (
                    a.get(i).copied().unwrap_or(0.0),
                    b.get(i).copied().unwrap_or(0.0),
                ),
                None => (left[i], right[i]),
            };
            let p = self.detect(kl, kr);
            let level_db = 0.5 * lin_to_db_fast(p);
            let target = static_gr(level_db, self.thr_db, self.slope, self.knee);
            let gr = self.ballistics(target);
            let g = db_to_lin_fast(-gr);
            let (mut yl, mut yr) = (left[i] * g, right[i] * g);
            let mut total = gr;
            if self.lim_on {
                // Zero-attack, zero-overshoot peak limiter: GR jumps to the overshoot, releases exponentially.
                let over = (lin_to_db_fast(yl.abs().max(yr.abs())) - self.lim_thr_db).max(0.0);
                self.lim_gr = if over >= self.lim_gr {
                    over
                } else {
                    over + self.lim_rel_c * (self.lim_gr - over)
                };
                let lg = db_to_lin_fast(-self.lim_gr);
                yl *= lg;
                yr *= lg;
                total += self.lim_gr;
            }
            left[i] = yl;
            right[i] = yr;
            self.gr_now = total;
        }
    }

    fn reset(&mut self) {
        self.apply();
        self.ms = [POWER_FLOOR; 2];
        self.gr_fast = 0.0;
        self.gr_slow = 0.0;
        self.lim_gr = 0.0;
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

    fn model(kind: VcaKind, params: &[(u16, ParamValue)]) -> VcaComp {
        let mut m = VcaComp::new(kind);
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

    /// Gain (dB) trajectory of a DC step `lo_db -> hi_db` (after settling at `lo_db`), sampled per sample.
    fn step_gain(m: &mut VcaComp, lo_db: f64, hi_db: f64, settle_s: f64, after_s: f64) -> Vec<f64> {
        let n0 = (settle_s * FS) as usize;
        let n1 = (after_s * FS) as usize;
        let lo = 10f64.powf(lo_db / 20.0) as f32;
        let hi = 10f64.powf(hi_db / 20.0) as f32;
        let mut x = vec![lo; n0 + n1];
        x[n0..].fill(hi);
        m.reset();
        let y = run_mono(m, &x);
        y[n0..]
            .iter()
            .zip(&x[n0..])
            .map(|(&o, &i)| db((o / i) as f64))
            .collect()
    }

    /// Time (ms) until the gain trajectory has covered `frac` of its total change.
    fn time_to(g: &[f64], g0: f64, frac: f64) -> f64 {
        let g1 = *g.last().unwrap();
        let d = g1 - g0;
        let i = g.iter().position(|&v| (v - g0) / d >= frac).unwrap();
        i as f64 / FS * 1e3
    }

    #[test]
    fn b160_static_curve_hard_knee_volts_threshold() {
        // thr 0.5 V = -3.80 dBu = -21.80 dBFS; ratio 4 (enum index 9).
        let mut m = model(
            VcaKind::B160,
            &[(1, ParamValue::Float(0.5)), (2, ParamValue::Enum(9))],
        );
        let t = (20.0 * (0.5f64 / 0.774_596_7).log10() - 18.0) as f32;
        assert!((m.threshold_dbfs() - t).abs() < 1e-3);
        let levels = [-40.0, -30.0, -21.8, -15.0, -5.0, 0.0];
        let meas = static_curve(&mut m, FS, &levels, 0.5);
        for (&l, &o) in levels.iter().zip(&meas) {
            let expect = if l > t as f64 {
                t as f64 + (l - t as f64) / 4.0
            } else {
                l
            };
            assert!((o - expect).abs() < 0.05, "in {l}: {o} vs {expect}");
        }
    }

    #[test]
    fn b160_attack_and_release_match_dbx_spec() {
        // Ratio 50 (~infinity). Attack: 63 % of the GR change after a 10/20/30 dB rise above threshold
        // (dbx 160A: typically 15 / 5 / 3 ms). Release: 125 dB/s.
        let thr = [(1, ParamValue::Float(0.1)), (2, ParamValue::Enum(15))];
        let mut m = model(VcaKind::B160, &thr);
        let t = m.threshold_dbfs() as f64;
        for (rise, spec_ms) in [(10.0, 15.0), (20.0, 5.0), (30.0, 3.0)] {
            let g = step_gain(&mut m, t, t + rise, 0.5, 0.5);
            let ms = time_to(&g, 0.0, 0.632);
            assert!(
                (ms - spec_ms).abs() <= 0.4 * spec_ms + 0.5,
                "rise {rise} dB: {ms:.2} ms vs {spec_ms}"
            );
        }
        // Release: from 40 dB GR, input drops far below threshold: GR recovers at 125 dB/s * slope.
        let g = step_gain(&mut m, t + 40.0, t - 80.0, 0.5, 0.5);
        let gr: Vec<f64> = g.iter().map(|&v| -v).collect();
        // GR between 30 and 10 dB: rate.
        let i30 = gr.iter().position(|&v| v <= 30.0).unwrap();
        let i10 = gr.iter().position(|&v| v <= 10.0).unwrap();
        let rate = 20.0 / ((i10 - i30) as f64 / FS);
        let expect = DBX_RELEASE_DB_PER_S * (1.0 - 1.0 / 50.0);
        assert!(
            (rate - expect).abs() < 0.03 * expect,
            "release {rate:.1} dB/s"
        );
    }

    #[test]
    fn b560_overeasy_negative_ratios_and_hard_knee() {
        // thr 0 dBu = -18 dBFS, ratio 4 (index 5), OverEasy on.
        let mut m = model(
            VcaKind::B560,
            &[(1, ParamValue::Float(0.0)), (2, ParamValue::Enum(5))],
        );
        let t = -18.0f32;
        let slope = 0.75f32;
        // At threshold: GR = slope * w * log10(2).
        let at = m.static_out_db(t);
        assert!((t - at - slope * OVEREASY_SCALE_DB * 2f32.log10()).abs() < 1e-3);
        // Far above: asymptotically the hard-knee line; far below: unity.
        assert!((m.static_out_db(t + 60.0) - (t + 15.0)).abs() < 1e-3);
        assert!((m.static_out_db(t - 60.0) - (t - 60.0)).abs() < 1e-3);
        // Monotonic, smooth (local slope between 1 and 1/4).
        let mut prev = m.static_out_db(t - 30.0);
        for k in 1..600 {
            let l = t - 30.0 + k as f32 * 0.1;
            let o = m.static_out_db(l);
            let s = (o - prev) / 0.1;
            assert!((0.249..=1.001).contains(&s), "slope {s} at {l}");
            prev = o;
        }
        // DSP settles to the law.
        let levels = [-40.0, -18.0, -6.0];
        let meas = static_curve(&mut m, FS, &levels, 0.5);
        for (&l, &o) in levels.iter().zip(&meas) {
            assert!((o - m.static_out_db(l as f32) as f64).abs() < 0.05, "{l}");
        }
        // Hard knee.
        m.set_param(3, ParamValue::Int(0));
        assert!((m.static_out_db(t) - t).abs() < 1e-4);
        assert!((m.static_out_db(t + 8.0) - (t + 2.0)).abs() < 1e-4);
        // Negative ratio -1:1 (index 14): output falls dB for dB above threshold.
        m.set_param(2, ParamValue::Enum(14));
        assert!((m.static_out_db(t + 10.0) - (t - 10.0)).abs() < 1e-3);
        // -5:1 (Float snaps to the table).
        m.set_param(2, ParamValue::Float(-5.0));
        assert_eq!(m.param(2), Some(-5.0));
        assert!((m.static_out_db(t + 10.0) - (t - 2.0)).abs() < 1e-3);
        // 999 = infinity:1.
        m.set_param(2, ParamValue::Enum(10));
        assert!((m.static_out_db(t + 20.0) - t).abs() < 0.03);
    }

    #[test]
    fn d241c_soft_knee_times_and_limiter() {
        // thr -30, ratio 4, manual times att 5 ms rel 200 ms, AUTO off, limiter at -6 dBFS.
        let mut m = model(
            VcaKind::D241C,
            &[
                (1, ParamValue::Float(-30.0)),
                (2, ParamValue::Enum(9)),
                (3, ParamValue::Float(5.0)),
                (4, ParamValue::Float(200.0)),
                (5, ParamValue::Float(-6.0)),
                (6, ParamValue::Float(100.0)),
                (7, ParamValue::Int(0)),
            ],
        );
        // Knee: ratio ramps in over 10 dB above threshold.
        assert!((m.static_out_db(-30.0) + 30.0).abs() < 1e-4);
        assert!((m.static_out_db(-25.0) - (-25.0 - 0.75 * 25.0 / 20.0)).abs() < 1e-4);
        assert!((m.static_out_db(-10.0) - (-10.0 - 0.75 * 15.0)).abs() < 1e-4);
        let levels = [-40.0, -25.0, -15.0];
        let meas = static_curve(&mut m, FS, &levels, 0.5);
        for (&l, &o) in levels.iter().zip(&meas) {
            assert!((o - m.static_out_db(l as f32) as f64).abs() < 0.05, "{l}");
        }
        // Attack / release time constants: dB one-poles (10-90 % = tau * ln 9).
        let g = step_gain(&mut m, -40.0, -10.0, 0.2, 0.3);
        let t = time_to(&g, 0.0, 0.9) - time_to(&g, 0.0, 0.1);
        assert!((t - 5.0 * 9f64.ln()).abs() < 0.5, "attack 10-90 {t}");
        let gr_hi = -g.last().unwrap();
        let g = step_gain(&mut m, -10.0, -40.0, 0.3, 2.0);
        let t = time_to(&g, -gr_hi, 0.9) - time_to(&g, -gr_hi, 0.1);
        assert!((t - 200.0 * 9f64.ln()).abs() < 10.0, "release 10-90 {t}");
        // Limiter: zero overshoot on a burst that the slow compressor does not catch.
        let x: Vec<f32> = (0..4800).map(|i| (i as f32 * 0.05).sin() * 0.9).collect();
        m.reset();
        let lim = 10f32.powf(-6.0 / 20.0);
        let y = run_mono(&mut m, &x);
        assert!(y.iter().all(|v| v.abs() <= lim * 1.0002));
        assert!(m.telemetry().gain_reduction_db > 6.0);
        // AUTO: two-stage release (short burst recovers faster than sustained compression).
        m.set_param(7, ParamValue::Int(1));
        m.set_param(5, ParamValue::Float(0.0));
        m.commit();
        let fast = recovery_ms(&mut m, -10.0, -40.0, 0.02, 1.0);
        let slow = recovery_ms(&mut m, -10.0, -40.0, 3.0, 1.0);
        assert!(
            slow > 2.0 * fast,
            "auto release burst {fast} ms, sustained {slow} ms"
        );
    }

    /// Time (ms) for GR to fall below `gr_db` after `burst_s` at `hi_db` followed by `lo_db`.
    fn recovery_ms(m: &mut VcaComp, hi_db: f64, lo_db: f64, burst_s: f64, gr_db: f64) -> f64 {
        let n0 = (burst_s * FS) as usize;
        let n1 = (8.0 * FS) as usize;
        let mut x = vec![10f64.powf(lo_db / 20.0) as f32; n0 + n1];
        x[..n0].fill(10f64.powf(hi_db / 20.0) as f32);
        m.reset();
        let y = run_mono(m, &x);
        let i = (n0..n0 + n1)
            .position(|k| db((y[k] / x[k]) as f64) > -gr_db)
            .unwrap();
        i as f64 / FS * 1e3
    }

    #[test]
    fn ssl9000_static_attack_fast_and_peak() {
        // thr -10 dBu = -28 dBFS, ratio 4 (index 8), rel 400 ms, PEAK detection.
        let mut m = model(
            VcaKind::Ssl9000,
            &[
                (1, ParamValue::Float(-10.0)),
                (2, ParamValue::Enum(8)),
                (4, ParamValue::Float(400.0)),
                (5, ParamValue::Int(1)),
            ],
        );
        assert!((m.threshold_dbfs() + 28.0).abs() < 1e-4);
        let levels = [-40.0, -20.0, -4.0];
        let meas = static_curve(&mut m, FS, &levels, 1.0);
        for (&l, &o) in levels.iter().zip(&meas) {
            let e = if l > -28.0 {
                -28.0 + (l + 28.0) / 4.0
            } else {
                l
            };
            assert!((o - e).abs() < 0.05, "{l}: {o} vs {e}");
        }
        // Attack at 20 dB GR: 63 % after 30 ms (normal) / 3 ms (fast).
        for (fast, spec) in [(0, 30.0), (1, 3.0)] {
            m.set_param(3, ParamValue::Int(fast));
            m.commit();
            let g = step_gain(&mut m, -40.0, -28.0 + 20.0 / 0.75, 0.2, 0.5);
            let t = time_to(&g, 0.0, 0.632);
            assert!((t - spec).abs() < 0.1 * spec + 0.1, "fast {fast}: {t} ms");
        }
        // Release time constant follows `rel`.
        let g = step_gain(&mut m, -4.0, -60.0, 0.5, 3.0);
        let g0: f64 = -(-4.0 - (-28.0 + 24.0 / 4.0));
        let t = time_to(&g, g0, 0.632);
        assert!((t - 400.0).abs() < 15.0, "release {t}");
        // RMS mode reads a sine at its RMS level: 0 dBFS-peak sine = -3 dBFS RMS.
        m.set_param(5, ParamValue::Int(0));
        m.commit();
        m.reset();
        let x: Vec<f32> = (0..48_000).map(|i| (i as f32 * 0.13).sin()).collect();
        let _ = run_mono(&mut m, &x);
        let gr = m.telemetry().gain_reduction_db;
        let expect = 0.75 * (-3.01 + 28.0);
        assert!((gr - expect).abs() < 0.5, "rms gr {gr} vs {expect}");
    }

    #[test]
    fn sbus_steps_and_auto_release() {
        // thr -20 dBFS, ratio 4 (index 3), attack 1 ms (index 2), release 0.4 s (index 2).
        let mut m = model(
            VcaKind::SBus,
            &[
                (1, ParamValue::Float(-20.0)),
                (2, ParamValue::Enum(3)),
                (3, ParamValue::Enum(2)),
                (4, ParamValue::Enum(2)),
            ],
        );
        let levels = [-30.0, -10.0, 0.0];
        let meas = static_curve(&mut m, FS, &levels, 0.5);
        for (&l, &o) in levels.iter().zip(&meas) {
            let e = if l > -20.0 {
                -20.0 + (l + 20.0) / 4.0
            } else {
                l
            };
            assert!((o - e).abs() < 0.05, "{l}: {o} vs {e}");
        }
        for (ai, att) in SBUS_ATTACK_MS.iter().enumerate() {
            m.set_param(3, ParamValue::Enum(ai as u32));
            m.commit();
            let g = step_gain(&mut m, -40.0, 0.0, 0.1, 0.3);
            let t = time_to(&g, 0.0, 0.632);
            let e = *att as f64;
            assert!((t - e).abs() < 0.03 * e + 0.03, "attack {e}: {t}");
        }
        for (ri, rel) in SBUS_RELEASE_S.iter().enumerate() {
            m.set_param(4, ParamValue::Enum(ri as u32));
            m.commit();
            let g = step_gain(&mut m, 0.0, -60.0, 0.3, 8.0);
            let t = time_to(&g, -15.0, 0.632);
            let e = *rel as f64 * 1e3;
            assert!((t - e).abs() < 0.02 * e + 1.0, "release {e}: {t}");
        }
        // AUTO: a short peak recovers quickly, sustained compression slowly.
        m.set_param(4, ParamValue::Enum(5));
        m.set_param(3, ParamValue::Float(0.3));
        assert_eq!(m.param(3), Some(0.3));
        m.commit();
        let fast = recovery_ms(&mut m, 0.0, -40.0, 0.01, 5.0);
        let slow = recovery_ms(&mut m, 0.0, -40.0, 3.0, 5.0);
        assert!(fast < 400.0, "burst {fast}");
        assert!(slow > 1500.0, "sustained {slow}");
    }

    #[test]
    fn stereo_link_sidechain_and_param_handling() {
        let mut m = model(VcaKind::SBus, &[(1, ParamValue::Float(-20.0))]);
        // Loud left, silent right: the right channel is reduced by the same gain (linked).
        let mut l = vec![0.5f32; 4800];
        let mut r = vec![0.01f32; 4800];
        for (a, b) in l.chunks_mut(64).zip(r.chunks_mut(64)) {
            m.process(a, b, None);
        }
        let gl = db((l[4799] / 0.5) as f64);
        let gr = db((r[4799] / 0.01) as f64);
        assert!((gl - gr).abs() < 1e-3 && gl < -3.0);
        // External key: silent key leaves the signal untouched.
        m.reset();
        let (mut l, mut r) = (vec![0.5f32; 64], vec![0.5f32; 64]);
        let k = [0.0f32; 64];
        m.process(&mut l, &mut r, Some((&k, &k)));
        assert!(l.iter().all(|&v| (v - 0.5).abs() < 1e-6));
        // Unknown slots ignored, ranges clamped, defaults restorable.
        m.set_param(9, ParamValue::Float(3.0));
        assert_eq!(m.param(9), None);
        m.set_param(1, ParamValue::Float(10.0));
        assert_eq!(m.param(1), Some(0.0));
        m.load_defaults();
        assert_eq!(m.param(1), Some(-12.0));
        assert_eq!(m.param(4), Some(5.0));
        let mut d = VcaComp::new(VcaKind::D241C);
        d.set_param(1, ParamValue::Float(-80.0));
        assert_eq!(d.param(1), Some(-60.0));
        assert_eq!(VcaKind::from_id("9000C"), Some(VcaKind::Ssl9000));
    }

    #[test]
    fn gr_meter_matches_wing_edit_mapping() {
        // Values from data/reference-curves/dyn-models.json (WING-Edit comp_gain_<m>), embedded so the test
        // runs without the data directory; the fidelity test re-checks against the file.
        let grs = [0.0, 1.0, 6.0, 20.0, 30.0, 48.0, 80.0];
        let b160 = [-0.3, -0.26738, -0.15036, -0.03, -0.00949, -0.00119, -3e-05];
        let b560 = [0.0, 0.05, 0.3, 1.0, 1.5, 2.0, 2.0];
        let d241 = [0.0, 0.05, 0.3, 1.0, 1.5, 1.5, 1.5];
        let ssl = [0.0, 0.05, 0.3, 1.0, 1.0, 1.0, 1.0];
        for (i, &g) in grs.iter().enumerate() {
            assert!((gr_meter(VcaKind::B160, g) - b160[i]).abs() < 1e-4);
            assert!((gr_meter(VcaKind::SBus, g) - b160[i]).abs() < 1e-4);
            assert!((gr_meter(VcaKind::B560, g) - b560[i]).abs() < 1e-4);
            assert!((gr_meter(VcaKind::D241C, g) - d241[i]).abs() < 1e-4);
            assert!((gr_meter(VcaKind::Ssl9000, g) - ssl[i]).abs() < 1e-4);
        }
    }

    #[test]
    fn telemetry_equals_applied_gain() {
        for kind in VcaKind::ALL {
            let mut m = model(kind, &[]);
            let x = vec![0.9f32; 9600];
            let y = run_mono(&mut m, &x);
            let applied = -db((y[9599] / 0.9) as f64);
            let gr = m.telemetry().gain_reduction_db as f64;
            assert!(gr >= 0.0);
            assert!((applied - gr).abs() < 1e-3, "{kind:?}: {applied} vs {gr}");
            assert_eq!(m.telemetry().engaged, gr > 0.01);
        }
    }
}
