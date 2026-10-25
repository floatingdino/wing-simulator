//! Standard gate-family models WING GATE (`GATE`) and WING DUCKER (`DUCK`) (models.md §3.1) with the gate
//! side-chain `gatesc` (signal-flow.md §2: filter type/freq/Q, source, tap, key listen).
//!
//! Laws recovered from WING-Edit 3.3.4 (T03C: `gate_parchg/respf_xseries_gate|ducker`, see the T03C log):
//! - Key filter: WING's `dynsc` biquads ([`crate::sidechain::key_filter_coeffs`]).
//! - Detector: one-pole with a 40 Hz bilinear corner, `(1 - tan(40 pi/fs)) / (1 + tan(40 pi/fs))` (tau 3.98
//!   ms). WING-Edit only posts the coefficient; it is used here as the release of an instant-attack peak
//!   follower on `max(|L|, |R|)` **[I]**.
//! - Gain law in the linear domain: open = 1; closed: GATE -> floor `10^(-range/20)` (0 = full mute when
//!   `range` is at its last step, 60 dB = "oo"); 1:1.5..1:4 -> downward expander `(env/thr)^(r-1)` clamped to
//!   the floor (i.e. `(r - 1) * (level - thr)` dB).
//! - Ballistics (WING computes them per 8-sample block; applied per sample with the same time constants):
//!   attack = one-pole with **10-90 % time = `att`** (`1 - exp(-2200 T / att)`; instant below 0.08 ms);
//!   hold = `hld`; release: ratio GATE = **linear ramp of the linear gain**, full scale (1 -> 0) in `rel`;
//!   expander ratios = one-pole with 10-90 % time `rel`.
//! - DUCK: attenuates to `10^(-range/20)` while the key is above `thr`; the falling (ducking) one-pole uses
//!   `att`, the rising (recovery) one-pole uses `rel` (10-90 % times), hold keeps it ducked.
//! - ACCENT (`acc`): WING-Edit posts only `acc/50` (0..2) to the DSP; its meaning is not recoverable from
//!   WING-Edit. Kept as threshold hysteresis `acc * [ACCENT_MAX_HYST_DB] / 100` **[unverified]**.
//!
//! Parameter changes need no extra smoothing: the gain only moves at the attack/release rates.

use crate::sidechain::{self, KeyFilterKind};
use wing_dsp::gain::lin_to_db_fast;

/// Detector release time constant (ms): WING's 40 Hz one-pole, 1000 / (2 pi 40).
pub const DETECTOR_RELEASE_MS: f64 = 3.978_873_6;
/// Hysteresis at ACCENT = 100 % (unverified, see module doc).
pub const ACCENT_MAX_HYST_DB: f32 = 10.0;
/// `range` at or above this value means full mute ("oo dB").
pub const RANGE_INF_DB: f32 = 60.0;
/// Gain (dB) reported for full mute (WING's -144 dB silence sentinel is clamped to this for meters).
pub const MUTE_FLOOR_DB: f32 = -120.0;
/// WING's 10-90 % constant (`2200 ~ 1000 ln 9`, per ms).
const TEN_NINETY: f64 = 2200.0;

/// Gate-slot model handled by [`Gate`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum GateModel {
    /// WING GATE (`GATE`): gate / downward expander.
    #[default]
    Gate,
    /// WING DUCKER (`DUCK`).
    Duck,
}

/// GATE `ratio` enum (snapshot strings `"1:1.5"`, `"1:2"`, `"1:3"`, `"1:4"`, `"gate"`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum GateRatio {
    R1_5,
    R2,
    #[default]
    R3,
    R4,
    Gate,
}

impl GateRatio {
    /// Expansion ratio `r` (output dB per input dB below threshold), `None` for a hard gate.
    pub fn expansion(self) -> Option<f32> {
        match self {
            GateRatio::R1_5 => Some(1.5),
            GateRatio::R2 => Some(2.0),
            GateRatio::R3 => Some(3.0),
            GateRatio::R4 => Some(4.0),
            GateRatio::Gate => None,
        }
    }

    /// Parse the snapshot string.
    pub fn from_snap(s: &str) -> Option<Self> {
        Some(match s {
            "1:1.5" => GateRatio::R1_5,
            "1:2" => GateRatio::R2,
            "1:3" => GateRatio::R3,
            "1:4" => GateRatio::R4,
            "gate" | "GATE" => GateRatio::Gate,
            _ => return None,
        })
    }
}

/// GATE / DUCK model parameters (keys `thr range att hld rel acc ratio`), engineering units.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GateParams {
    pub model: GateModel,
    /// `thr` -80 .. 0 dB.
    pub threshold_db: f32,
    /// `range` 3 .. 60 dB (60 = oo).
    pub range_db: f32,
    /// `att` 0 .. 120 ms.
    pub attack_ms: f32,
    /// `hld` GATE 1 .. 1000 ms, DUCK 30 .. 30000 ms.
    pub hold_ms: f32,
    /// `rel` 4 .. 4000 ms.
    pub release_ms: f32,
    /// `acc` 0 .. 100 % (GATE only).
    pub accent_pct: f32,
    /// `ratio` (GATE only).
    pub ratio: GateRatio,
}

impl Default for GateParams {
    fn default() -> Self {
        Self::gate_default()
    }
}

impl GateParams {
    /// WING-Edit default string for `GATE`: thr=-40, range=40, att=10, hld=10, rel=200, acc=0, ratio=1:3.
    pub fn gate_default() -> Self {
        Self {
            model: GateModel::Gate,
            threshold_db: -40.0,
            range_db: 40.0,
            attack_ms: 10.0,
            hold_ms: 10.0,
            release_ms: 200.0,
            accent_pct: 0.0,
            ratio: GateRatio::R3,
        }
    }

    /// WING-Edit default for `DUCK`: thr=-20, range=20, att=100, hld=100, rel=500.
    pub fn duck_default() -> Self {
        Self {
            model: GateModel::Duck,
            threshold_db: -20.0,
            range_db: 20.0,
            attack_ms: 100.0,
            hold_ms: 100.0,
            release_ms: 500.0,
            accent_pct: 0.0,
            ratio: GateRatio::Gate,
        }
    }

    pub fn clamped(mut self) -> Self {
        self.threshold_db = self.threshold_db.clamp(-80.0, 0.0);
        self.range_db = self.range_db.clamp(3.0, RANGE_INF_DB);
        self.attack_ms = self.attack_ms.clamp(0.0, 120.0);
        self.hold_ms = match self.model {
            GateModel::Gate => self.hold_ms.clamp(1.0, 1000.0),
            GateModel::Duck => self.hold_ms.clamp(30.0, 30_000.0),
        };
        self.release_ms = self.release_ms.clamp(4.0, 4000.0);
        self.accent_pct = self.accent_pct.clamp(0.0, 100.0);
        self
    }

    /// Hysteresis in dB (GATE only).
    pub fn hysteresis_db(&self) -> f32 {
        match self.model {
            GateModel::Gate => self.accent_pct.clamp(0.0, 100.0) * 0.01 * ACCENT_MAX_HYST_DB,
            GateModel::Duck => 0.0,
        }
    }

    /// Attenuation depth in dB used internally (positive; [`MUTE_FLOOR_DB`] magnitude for "oo").
    pub fn depth_db(&self) -> f32 {
        if self.range_db >= RANGE_INF_DB {
            -MUTE_FLOOR_DB
        } else {
            self.range_db.max(3.0)
        }
    }

    /// Closed/ducked gain floor (linear) as WING posts it: `10^(-range/20)`, 0 for "oo".
    pub fn floor_lin(&self) -> f32 {
        if self.range_db >= RANGE_INF_DB {
            0.0
        } else {
            crate::wingcoef::db2lin(-self.range_db.max(3.0))
        }
    }

    /// Static gain (dB) for a steady key level, WING-Edit's `respf` law: GATE: 0 at/above `thr`; below it
    /// `-R` (ratio GATE) or `max((r-1)(level-thr), -R)` with `R = trunc(range)` (144 = oo at the last step).
    /// DUCK: `-range` above `thr`, else 0.
    pub fn static_gain_db(&self, level_db: f32) -> f32 {
        match self.model {
            GateModel::Gate => {
                if level_db >= self.threshold_db {
                    return 0.0;
                }
                let r = if self.range_db >= RANGE_INF_DB {
                    144.0
                } else {
                    self.range_db.trunc()
                };
                let g = match self.ratio.expansion() {
                    None => -144.0,
                    Some(x) => (level_db - self.threshold_db) * (x - 1.0),
                };
                g.max(-r)
            }
            GateModel::Duck => {
                if level_db > self.threshold_db {
                    -self.range_db
                } else {
                    0.0
                }
            }
        }
    }
}

/// `gatesc.type`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum ScFilter {
    #[default]
    Off,
    Lp12,
    Hp12,
    Bp,
    Notch,
}

impl ScFilter {
    /// The equivalent [`KeyFilterKind`] (same WING biquads for `gatesc` and `dynsc`).
    pub fn kind(self) -> KeyFilterKind {
        match self {
            ScFilter::Off => KeyFilterKind::Off,
            ScFilter::Lp12 => KeyFilterKind::Lp12,
            ScFilter::Hp12 => KeyFilterKind::Hp12,
            ScFilter::Bp => KeyFilterKind::Bp,
            ScFilter::Notch => KeyFilterKind::Notch,
        }
    }
}

/// `gatesc.src`: the key source. The engine resolves the buffer and passes it to [`Gate::process`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum ScSource {
    #[default]
    SelfKey,
    /// `CH.n` (1..=40).
    Channel(u8),
}

/// `gatesc.tap`: where on the key source the key is taken (resolved by the engine).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum ScTap {
    #[default]
    In,
    Filt,
    Tap3,
    Tap4,
    Tap5,
    Pfl,
    Afl,
    Post,
}

/// Gate side-chain parameters (`gatesc`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SidechainParams {
    pub filter: ScFilter,
    /// `f` 20 .. 20000 Hz.
    pub freq: f32,
    /// `q` 0.44 .. 10.
    pub q: f32,
    pub source: ScSource,
    pub tap: ScTap,
    /// `$solo` (SC SOLO / key listen): the engine sends [`Gate::key_signal`] to the solo bus while set.
    pub key_listen: bool,
}

impl Default for SidechainParams {
    fn default() -> Self {
        Self {
            filter: ScFilter::Off,
            freq: 1_002.374_4,
            q: 1.995_882,
            source: ScSource::SelfKey,
            tap: ScTap::In,
            key_listen: false,
        }
    }
}

impl SidechainParams {
    pub fn clamped(mut self) -> Self {
        self.freq = self.freq.clamp(20.0, 20_000.0);
        self.q = self.q.clamp(0.44, 10.0);
        self
    }
}

/// GATE / DUCK processor (stereo-linked).
pub struct Gate {
    params: GateParams,
    sc: SidechainParams,
    fs: f64,
    // Derived per-parameter values.
    thr: f32,
    thr_close: f32,
    floor: f32,
    /// One-pole coefficient (fraction of the remaining distance per sample) for a rising gain.
    up_c: f32,
    /// Same for a falling gain (one-pole), or 0 when `down_lin` is used.
    down_c: f32,
    /// Linear-gain decrement per sample (GATE ratio release), 0 = one-pole.
    down_lin: f32,
    hold_samples: u32,
    // State.
    /// Peak envelope (instant attack, one-pole release; holds exactly on a constant input).
    env: f32,
    env_rel: f32,
    sc_filt: sidechain::KeyFilter,
    /// Linear gain (f64 so slow one-poles reach their target).
    gain: f64,
    engaged: bool,
    hold_ctr: u32,
    key_l: Vec<f32>,
    key_r: Vec<f32>,
    key_len: usize,
}

impl Default for Gate {
    fn default() -> Self {
        Self::new()
    }
}

/// Per-sample one-pole step for a WING 10-90 % time `ms` (1 = instant).
fn ten_ninety(ms: f32, fs: f64) -> f32 {
    if (ms as f64) / 8.0 < 0.01 {
        1.0
    } else {
        (1.0 - (-TEN_NINETY / (fs * ms as f64)).exp()) as f32
    }
}

impl Gate {
    pub fn new() -> Self {
        let mut g = Self {
            params: GateParams::default(),
            sc: SidechainParams::default(),
            fs: 48_000.0,
            thr: 0.0,
            thr_close: 0.0,
            floor: 0.0,
            up_c: 1.0,
            down_c: 1.0,
            down_lin: 0.0,
            hold_samples: 0,
            env: 0.0,
            env_rel: 0.0,
            sc_filt: sidechain::KeyFilter::default(),
            gain: 1.0,
            engaged: false,
            hold_ctr: 0,
            key_l: Vec::new(),
            key_r: Vec::new(),
            key_len: 0,
        };
        g.prepare(48_000.0, 64);
        g
    }

    /// Allocate key scratch for blocks up to `max_block` samples and reset state.
    pub fn prepare(&mut self, sample_rate: f64, max_block: usize) {
        self.fs = sample_rate;
        let n = max_block.max(1);
        self.key_l = vec![0.0; n];
        self.key_r = vec![0.0; n];
        // WING: (1 - tanf(40 pi/fs)) / (1 + tanf(40 pi/fs)).
        let t = (crate::wingcoef::Ctx::new(sample_rate).pi_over_fs() * 8.0 * 5.0).tan();
        self.env_rel = (1.0 - t) / (1.0 + t);
        let (p, s) = (self.params, self.sc);
        self.set_params(&p);
        self.set_sidechain(&s);
        self.reset();
    }

    /// Clear detector/filter state and return to the idle gain (GATE closed, DUCK released).
    pub fn reset(&mut self) {
        self.env = 0.0;
        self.sc_filt.reset();
        self.engaged = false;
        self.hold_ctr = 0;
        self.gain = self.idle_gain();
        self.key_len = 0;
    }

    fn idle_gain(&self) -> f64 {
        match self.params.model {
            GateModel::Gate => self.floor as f64,
            GateModel::Duck => 1.0,
        }
    }

    pub fn params(&self) -> &GateParams {
        &self.params
    }

    pub fn sidechain(&self) -> &SidechainParams {
        &self.sc
    }

    /// Update model parameters (RT-safe). Values are clamped per model.
    pub fn set_params(&mut self, p: &GateParams) {
        let p = p.clamped();
        self.params = p;
        self.thr = p.threshold_db;
        self.thr_close = p.threshold_db - p.hysteresis_db();
        self.floor = p.floor_lin();
        let fs = self.fs;
        match p.model {
            GateModel::Gate => {
                self.up_c = ten_ninety(p.attack_ms, fs);
                if p.ratio == GateRatio::Gate {
                    self.down_c = 0.0;
                    self.down_lin = (1000.0 / (fs * p.release_ms as f64)) as f32;
                } else {
                    self.down_c = ten_ninety(p.release_ms, fs);
                    self.down_lin = 0.0;
                }
            }
            GateModel::Duck => {
                self.up_c = ten_ninety(p.release_ms, fs);
                self.down_c = ten_ninety(p.attack_ms, fs);
                self.down_lin = 0.0;
            }
        }
        self.hold_samples = (p.hold_ms as f64 * 1e-3 * fs).round() as u32;
        self.gain = self.gain.clamp(self.floor as f64, 1.0);
    }

    /// Update side-chain parameters (RT-safe).
    pub fn set_sidechain(&mut self, s: &SidechainParams) {
        let s = s.clamped();
        self.sc = s;
        self.sc_filt.set(s.filter.kind(), s.freq, s.q, self.fs);
    }

    /// Current gain in dB (0 = open / not ducking). For gain-reduction metering.
    pub fn gain_db(&self) -> f32 {
        if self.gain <= 0.0 {
            MUTE_FLOOR_DB
        } else {
            ((20.0 * self.gain.log10()) as f32).max(MUTE_FLOOR_DB)
        }
    }

    /// Current linear gain.
    pub fn gain_lin(&self) -> f32 {
        self.gain as f32
    }

    /// GATE: open (or holding). DUCK: ducking (or holding).
    pub fn is_engaged(&self) -> bool {
        self.engaged
    }

    /// Filtered key of the most recent block (or last `max_block` chunk): the key-listen signal.
    pub fn key_signal(&self) -> (&[f32], &[f32]) {
        (&self.key_l[..self.key_len], &self.key_r[..self.key_len])
    }

    /// Process a stereo block in place. `key` is the external side-chain (already taken at `gatesc.tap` of
    /// `gatesc.src`); it is used only when `source != SelfKey`, otherwise (or if `None`) the block keys
    /// from its own input. Blocks longer than `max_block` are processed in chunks.
    pub fn process(&mut self, left: &mut [f32], right: &mut [f32], key: Option<(&[f32], &[f32])>) {
        debug_assert_eq!(left.len(), right.len());
        let key = if self.sc.source == ScSource::SelfKey {
            None
        } else {
            key
        };
        let chunk = self.key_l.len();
        let mut start = 0;
        while start < left.len() {
            let end = (start + chunk).min(left.len());
            let k = key.map(|(kl, kr)| (&kl[start..end], &kr[start..end]));
            self.process_chunk(&mut left[start..end], &mut right[start..end], k);
            start = end;
        }
    }

    fn process_chunk(
        &mut self,
        left: &mut [f32],
        right: &mut [f32],
        key: Option<(&[f32], &[f32])>,
    ) {
        let n = left.len();
        let (src_l, src_r): (&[f32], &[f32]) = match key {
            Some((kl, kr)) => (kl, kr),
            None => (left, right),
        };
        self.key_l[..n].copy_from_slice(src_l);
        self.key_r[..n].copy_from_slice(src_r);
        self.key_len = n;
        if self.sc_filt.is_on() {
            self.sc_filt.process_block(0, &mut self.key_l[..n]);
            self.sc_filt.process_block(1, &mut self.key_r[..n]);
            self.sc_filt.flush_denormals();
        }
        let p = self.params;
        let expansion = match p.model {
            GateModel::Gate => p.ratio.expansion(),
            GateModel::Duck => None,
        };
        let floor = self.floor;
        let thr_lin = crate::wingcoef::db2lin(self.thr);
        for i in 0..n {
            let k = self.key_l[i].abs().max(self.key_r[i].abs());
            self.env = if k >= self.env {
                k
            } else {
                self.env * self.env_rel
            };
            let level = lin_to_db_fast(self.env);
            // Open/hold state machine.
            if level > self.thr {
                self.engaged = true;
                self.hold_ctr = self.hold_samples;
            } else if self.engaged {
                if level >= self.thr_close {
                    self.hold_ctr = self.hold_samples;
                } else if self.hold_ctr > 0 {
                    self.hold_ctr -= 1;
                } else {
                    self.engaged = false;
                }
            }
            let target = match p.model {
                GateModel::Gate => {
                    if self.engaged {
                        1.0
                    } else {
                        match expansion {
                            None => floor,
                            Some(r) => {
                                if thr_lin > 0.0 {
                                    (self.env / thr_lin).powf(r - 1.0).clamp(floor, 1.0)
                                } else {
                                    1.0
                                }
                            }
                        }
                    }
                }
                GateModel::Duck => {
                    if self.engaged {
                        floor
                    } else {
                        1.0
                    }
                }
            };
            let (g, target) = (self.gain, target as f64);
            self.gain = if target > g {
                g + (target - g) * self.up_c as f64
            } else if self.down_lin > 0.0 {
                (g - self.down_lin as f64).max(target)
            } else {
                g + (target - g) * self.down_c as f64
            };
            if (target - self.gain).abs() < 1e-9 {
                self.gain = target;
            }
            let lin = self.gain as f32;
            left[i] *= lin;
            right[i] *= lin;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use wing_dsp::gain::db_to_lin;

    const FS: f64 = 48_000.0;

    fn gate(p: GateParams) -> Gate {
        let mut g = Gate::new();
        g.set_params(&p);
        g.prepare(FS, 64);
        g
    }

    /// Run `n` samples of a constant (DC) key at `level_db` through the gate keyed on itself.
    fn run_dc(g: &mut Gate, level_db: f32, n: usize) -> f32 {
        let x = db_to_lin(level_db);
        let mut l = vec![x; n];
        let mut r = vec![x; n];
        g.process(&mut l, &mut r, None);
        l[n - 1]
    }

    /// Per-sample gain trace for a constant key at `level_db` (one-sample blocks).
    fn trace(g: &mut Gate, level_db: Option<f32>, n: usize) -> Vec<f32> {
        let x = level_db.map_or(0.0, db_to_lin);
        (0..n)
            .map(|_| {
                let (mut l, mut r) = ([x], [x]);
                g.process(&mut l, &mut r, None);
                g.gain_db()
            })
            .collect()
    }

    fn ms(n: f64) -> f64 {
        n * 1e-3 * FS
    }

    #[test]
    fn hard_gate_static_curve_and_range() {
        for range in [3.0f32, 20.0, 40.0, 59.0] {
            let mut g = gate(GateParams {
                ratio: GateRatio::Gate,
                range_db: range,
                ..GateParams::gate_default()
            });
            for lvl in [-70.0f32, -50.0, -41.0, -39.0, -20.0, -1.0] {
                g.reset();
                let y = run_dc(&mut g, lvl, 48_000);
                let want = if lvl > -40.0 { 0.0 } else { -range };
                assert!(
                    (g.gain_db() - want).abs() < 1e-3,
                    "range {range} lvl {lvl}: gain {}",
                    g.gain_db()
                );
                let out_db = 20.0 * y.abs().log10();
                assert!((out_db - (lvl + want)).abs() < 1e-3);
            }
        }
        // "oo" range = exact silence.
        let mut g = gate(GateParams {
            ratio: GateRatio::Gate,
            range_db: 60.0,
            ..GateParams::gate_default()
        });
        assert_eq!(run_dc(&mut g, -60.0, 48_000), 0.0);
    }

    #[test]
    fn expander_static_curve() {
        for (ratio, r) in [
            (GateRatio::R1_5, 1.5f32),
            (GateRatio::R2, 2.0),
            (GateRatio::R3, 3.0),
            (GateRatio::R4, 4.0),
        ] {
            let mut g = gate(GateParams {
                ratio,
                range_db: 50.0,
                release_ms: 4.0,
                hold_ms: 1.0,
                ..GateParams::gate_default()
            });
            for lvl in [-80.0f32, -60.0, -50.0, -45.0, -40.5, -30.0] {
                g.reset();
                run_dc(&mut g, lvl, 48_000);
                let want = ((r - 1.0) * (lvl + 40.0)).clamp(-50.0, 0.0);
                assert!(
                    (g.gain_db() - want).abs() < 2e-3,
                    "ratio {r} lvl {lvl}: {} vs {want}",
                    g.gain_db()
                );
            }
        }
    }

    #[test]
    fn accent_hysteresis() {
        let mut g = gate(GateParams {
            ratio: GateRatio::Gate,
            accent_pct: 50.0, // 5 dB
            attack_ms: 0.0,
            hold_ms: 1.0,
            release_ms: 4.0,
            ..GateParams::gate_default()
        });
        run_dc(&mut g, -35.0, 4800);
        assert!(g.is_engaged());
        run_dc(&mut g, -43.0, 4800); // inside the hysteresis band: stays open
        assert!(g.is_engaged() && g.gain_db() == 0.0);
        run_dc(&mut g, -46.0, 4800); // below thr - 5: closes
        assert!(!g.is_engaged() && g.gain_db() == -40.0);
        run_dc(&mut g, -41.0, 4800); // re-opening needs thr
        assert!(!g.is_engaged());
        run_dc(&mut g, -39.0, 4800);
        assert!(g.is_engaged());
    }

    #[test]
    fn attack_hold_release_timing() {
        let (att, hld, rel) = (10.0f64, 50.0f64, 100.0f64);
        let mut g = gate(GateParams {
            ratio: GateRatio::Gate,
            range_db: 40.0,
            attack_ms: att as f32,
            hold_ms: hld as f32,
            release_ms: rel as f32,
            ..GateParams::gate_default()
        });
        // Attack: closed (floor 0.01) -> open (1) as a one-pole in linear gain, 10-90 % time = `att`.
        let lin = |db: f32| 10f32.powf(db / 20.0);
        let t: Vec<f32> = trace(&mut g, Some(-20.0), ms(200.0) as usize)
            .into_iter()
            .map(lin)
            .collect();
        let (lo, hi) = (0.01 + 0.1 * 0.99, 0.01 + 0.9 * 0.99);
        let t10 = t.iter().position(|&x| x >= lo).unwrap() as f64;
        let t90 = t.iter().position(|&x| x >= hi).unwrap() as f64;
        assert!(
            (t90 - t10 - ms(att)).abs() <= 2.0,
            "attack 10-90 {}",
            t90 - t10
        );
        assert!((t.last().unwrap() - 1.0).abs() < 1e-6, "fully open");
        // Key off: the detector falls from -20 dB to thr (-40) in tau*ln(10); then hold; then a linear ramp of
        // the linear gain, 1 -> 0 in `rel` (stopping at the floor).
        let fall = DETECTOR_RELEASE_MS * (10f64).ln();
        let t = trace(&mut g, None, ms(400.0) as usize);
        let rel_start = t.iter().position(|&x| x < 0.0).unwrap() as f64;
        assert!(
            (rel_start - ms(fall + hld)).abs() <= ms(0.2),
            "release start {rel_start} vs {}",
            ms(fall + hld)
        );
        let closed_at = t.iter().position(|&x| x <= -40.0 + 1e-3).unwrap() as f64;
        assert!(
            (closed_at - rel_start - ms(rel * 0.99)).abs() <= 2.0,
            "release {}",
            closed_at - rel_start
        );
        let mid = 10f32.powf(t[(rel_start + ms(rel / 2.0)) as usize] / 20.0);
        assert!((mid - 0.5).abs() < 0.01, "linear ramp midpoint {mid}");
    }

    #[test]
    fn ducker_follows_external_key() {
        let mut g = Gate::new();
        g.set_params(&GateParams {
            attack_ms: 20.0,
            hold_ms: 30.0,
            release_ms: 200.0,
            ..GateParams::duck_default()
        });
        g.set_sidechain(&SidechainParams {
            source: ScSource::Channel(5),
            ..Default::default()
        });
        g.prepare(FS, 64);
        assert_eq!(g.gain_db(), 0.0, "idle ducker passes");
        let n = 64;
        let run = |g: &mut Gate, key_db: Option<f32>, blocks: usize| -> f32 {
            let k = key_db.map_or(0.0, db_to_lin);
            let key = vec![k; n];
            let mut out = 0.0;
            for _ in 0..blocks {
                // Loud programme (0 dBFS) must not duck itself: only the key counts.
                let mut l = vec![1.0f32; n];
                let mut r = vec![1.0f32; n];
                g.process(&mut l, &mut r, Some((&key, &key)));
                out = l[n - 1];
            }
            out
        };
        assert!((run(&mut g, None, 100) - 1.0).abs() < 1e-6);
        // Key above thr (-20): ducks by range (20 dB), one-pole with 10-90 % time = attack.
        run(&mut g, Some(-10.0), (ms(200.0) as usize).div_ceil(n));
        assert!((g.gain_db() + 20.0).abs() < 1e-3, "{}", g.gain_db());
        let y = run(&mut g, Some(-10.0), 10);
        assert!((y - 0.1).abs() < 1e-4);
        // Key off: held, then released over 200 ms.
        run(&mut g, None, (ms(30.0) as usize) / n);
        assert!(g.gain_db() <= -19.99, "held: {}", g.gain_db());
        run(&mut g, None, (ms(2000.0) as usize) / n);
        assert!(g.gain_db() > -1e-3, "recovered: {}", g.gain_db());
        // SELF source ignores the external key.
        g.set_sidechain(&SidechainParams::default());
        let mut l = vec![db_to_lin(-40.0); n];
        let mut r = l.clone();
        let key = vec![1.0f32; n];
        g.process(&mut l, &mut r, Some((&key, &key)));
        assert_eq!(g.gain_db(), 0.0);
    }

    #[test]
    fn sidechain_filter_and_key_listen() {
        // 100 Hz key at -30 dB, threshold -40: opens unfiltered, stays closed with HP12 at 4 kHz.
        let key: Vec<f32> = (0..4920)
            .map(|i| {
                db_to_lin(-30.0) * (i as f32 * 2.0 * core::f32::consts::PI * 100.0 / 48_000.0).sin()
            })
            .collect();
        for (filter, open) in [
            (ScFilter::Off, true),
            (ScFilter::Hp12, false),
            (ScFilter::Lp12, true),
        ] {
            let mut g = Gate::new();
            g.set_params(&GateParams {
                ratio: GateRatio::Gate,
                ..GateParams::gate_default()
            });
            g.set_sidechain(&SidechainParams {
                filter,
                freq: 4000.0,
                q: 0.707,
                key_listen: true,
                ..Default::default()
            });
            g.prepare(FS, 8192);
            let mut l = key.clone();
            let mut r = key.clone();
            g.process(&mut l, &mut r, None);
            assert_eq!(g.is_engaged(), open, "{filter:?}");
            let (kl, _) = g.key_signal();
            assert_eq!(kl.len(), key.len());
            let kl = &kl[kl.len() - 480..];
            let peak = kl.iter().fold(0.0f32, |m, x| m.max(x.abs()));
            if filter == ScFilter::Hp12 {
                assert!(
                    peak < db_to_lin(-60.0),
                    "key listen hears filtered key: {peak}"
                );
            } else {
                assert!(peak > db_to_lin(-31.0));
            }
        }
    }

    #[test]
    fn ratio_strings_and_clamps() {
        assert_eq!(GateRatio::from_snap("1:3"), Some(GateRatio::R3));
        assert_eq!(GateRatio::from_snap("gate"), Some(GateRatio::Gate));
        let p = GateParams {
            hold_ms: 5.0,
            release_ms: 8.02,
            ..GateParams::duck_default()
        }
        .clamped();
        assert_eq!(p.hold_ms, 30.0);
        assert_eq!(p.release_ms, 8.02);
    }
}
