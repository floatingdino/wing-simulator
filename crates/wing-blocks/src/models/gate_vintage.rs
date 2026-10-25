//! Vintage console gates (T047): `E88` (EVEN 88 GATE, Neve 88RS), `9000G` (SOUL 9000 GATE, SSL 9000 J) and
//! `D241G` (DRAW MORE 241, Drawmer DL241/DS201). Parameters, ranges and defaults: `docs/research/models.md`
//! §3.1 and `data/schema/wing-tree.json` (slot = position in the model's parameter list).
//!
//! All three share one stereo-linked gate/expander core:
//! ```text
//! key (self or external) -> full-wave peak detector (instant attack, one-pole release) -> level dB
//!   -> open/hold/close state machine with hysteresis -> target gain
//!        open  : 0 dB
//!        closed: GATE = -range, EXPANDER = (R - 1) * (level - thr) clamped to -range
//!   -> gain slewed linearly in dB (constant dB/s: the VCA control voltage of the hardware ramps) -> both sides
//! ```
//! WING applies the key filter, mix and make-up in the slot wrapper (`ModelSlot`), so the hardware key filters
//! (SSL/Drawmer sidechain HF/LF filters) are not modelled here: these three WING models expose none.
//!
//! - **E88** (slots `thr hyst range rel fast m40`): open threshold `thr` (`m40` shifts it down 40 dB), close
//!   threshold `thr - hyst`; no hold control; attack 50 us (`fast`) or 2 ms (models.md: 88RS 50 us fast vs ~2 ms);
//!   `hyst` at its end stop (25 dB, UI label "EXP") switches to expander mode (2:1 below the threshold, no
//!   hysteresis); release `rel` = time to traverse 60 dB (the full `range` span).
//! - **9000G** (slots `thr range hld rel fast mode`): SSL 9000 J gate/expander; attack 100 us (`fast`) or 1.5 ms
//!   (models.md); hold 10 ms..4 s; release = time to traverse 40 dB (the full `range` span); `mode` GATE or EXP
//!   (2:1 expander below the threshold, still range-limited).
//! - **D241G** (slots `thr slow`): Drawmer: fixed fast attack ([`D241_ATTACK_MS`]), hard gate with
//!   [`D241_DEPTH_DB`] of attenuation (exact zero at the floor), programme-dependent release: the release time
//!   moves from a short to a long value as the gate has been open longer (`slow` switch lengthens both).
//!
//! Timing laws, detector release, hysteresis of 9000G/D241G and the programme-dependent release constants are
//! unpublished and flagged as assumptions in `docs/VERIFY-ON-HARDWARE.md`.
//!
//! Real-time: no allocation; `set_param` stages, `commit`/`process`/`reset` apply (a few divisions).

use crate::model::{Model, ModelTelemetry, ParamValue};
use wing_dsp::gain::{db_to_lin_fast, lin_to_db_fast};
use wing_dsp::smooth::one_pole_coeff;

/// Peak detector release (ms). Assumption: one value for all three (rectified 100 Hz ripple ~2 dB).
pub const DETECTOR_RELEASE_MS: f64 = 20.0;
/// Expander slope below threshold in expander modes (SSL "EXP" ~1:2, models.md; Neve assumed the same).
pub const EXPANDER_RATIO: f32 = 2.0;
/// E88 hysteresis end stop (UI label "EXP"): expander mode at or above this value (dB).
pub const E88_EXP_HYST_DB: f32 = 25.0;
/// E88 attack, slow / fast (ms) (models.md: 88RS 50 us fast vs ~2 ms).
pub const E88_ATTACK_MS: [f32; 2] = [2.0, 0.05];
/// E88 release is the time to traverse this many dB.
pub const E88_RELEASE_SPAN_DB: f32 = 60.0;
/// 9000G attack, slow / fast (ms) (models.md: SSL fast ~100 us vs ~1.5 ms).
pub const SSL_ATTACK_MS: [f32; 2] = [1.5, 0.1];
/// 9000G release is the time to traverse this many dB (the 40 dB range maximum).
pub const SSL_RELEASE_SPAN_DB: f32 = 40.0;
/// 9000G fixed hysteresis (dB). Assumption: the SSL gate shows a small close/open gap.
pub const SSL_HYST_DB: f32 = 2.0;
/// D241G: fixed attack (ms) (Drawmer DS201/DL241 "10 us" class; one sample at 48 kHz is 21 us).
pub const D241_ATTACK_MS: f32 = 0.02;
/// D241G: gate depth (dB) and the span the release times refer to.
pub const D241_DEPTH_DB: f32 = 80.0;
/// D241G fixed hysteresis (dB). Assumption.
pub const D241_HYST_DB: f32 = 2.0;
/// D241G hold (ms), normal / slow. Assumption (DS201 hold 2 ms..2 s).
pub const D241_HOLD_MS: [f32; 2] = [10.0, 100.0];
/// D241G programme-dependent release: (short, long) in ms for the 80 dB span, normal / slow.
pub const D241_RELEASE_MS: [[f32; 2]; 2] = [[150.0, 600.0], [500.0, 2000.0]];
/// D241G: time constant (s) with which "open time" charges while the gate is open.
pub const D241_OPEN_CHARGE_S: f64 = 0.3;

/// Which of the three models an instance is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VintageGateKind {
    E88,
    Ssl9000,
    D241,
}

impl VintageGateKind {
    pub const ALL: [VintageGateKind; 3] = [
        VintageGateKind::E88,
        VintageGateKind::Ssl9000,
        VintageGateKind::D241,
    ];

    /// Snapshot `mdl` token.
    pub fn id(self) -> &'static str {
        match self {
            VintageGateKind::E88 => "E88",
            VintageGateKind::Ssl9000 => "9000G",
            VintageGateKind::D241 => "D241G",
        }
    }

    pub fn from_id(id: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|k| k.id() == id)
    }

    fn slots(self) -> &'static [SlotDef] {
        match self {
            VintageGateKind::E88 => &E88_SLOTS,
            VintageGateKind::Ssl9000 => &SSL_SLOTS,
            VintageGateKind::D241 => &D241_SLOTS,
        }
    }
}

#[derive(Clone, Copy)]
enum SlotKind {
    Range(f32, f32),
    Bool,
    Index(u32),
}

#[derive(Clone, Copy)]
struct SlotDef {
    kind: SlotKind,
    default: f32,
}

const fn rng(lo: f32, hi: f32, default: f32) -> SlotDef {
    SlotDef {
        kind: SlotKind::Range(lo, hi),
        default,
    }
}
const fn flag(default: f32) -> SlotDef {
    SlotDef {
        kind: SlotKind::Bool,
        default,
    }
}

/// E88 slot positions.
pub mod e88_slot {
    pub const THR: u16 = 1;
    pub const HYST: u16 = 2;
    pub const RANGE: u16 = 3;
    pub const REL: u16 = 4;
    pub const FAST: u16 = 5;
    pub const M40: u16 = 6;
}
/// 9000G slot positions.
pub mod ssl_slot {
    pub const THR: u16 = 1;
    pub const RANGE: u16 = 2;
    pub const HLD: u16 = 3;
    pub const REL: u16 = 4;
    pub const FAST: u16 = 5;
    pub const MODE: u16 = 6;
}
/// D241G slot positions.
pub mod d241_slot {
    pub const THR: u16 = 1;
    pub const SLOW: u16 = 2;
}

// Schema defaults (rel 355 / hld 51 / rel 806 are the log-grid values WING-Edit stores).
static E88_SLOTS: [SlotDef; 6] = [
    rng(-40.0, 0.0, -35.0),
    rng(0.0, 25.0, 6.0),
    rng(0.0, 60.0, 60.0),
    rng(100.0, 3000.0, 355.0),
    flag(0.0),
    flag(0.0),
];
static SSL_SLOTS: [SlotDef; 6] = [
    rng(-40.0, 0.0, -35.0),
    rng(0.0, 40.0, 40.0),
    rng(10.0, 4000.0, 51.0),
    rng(100.0, 4000.0, 806.0),
    flag(0.0),
    SlotDef {
        kind: SlotKind::Index(2),
        default: 0.0,
    },
];
static D241_SLOTS: [SlotDef; 2] = [rng(-80.0, 0.0, -40.0), flag(0.0)];

/// WING-Edit GR-meter position (`gate_gain_<m>`, `data/reference-curves/dyn-models.json`) for `gr_db` dB of
/// attenuation: E88 `GR/20` capped at 2.5, 9000G `GR/20` capped at 1, D241G `(GR-1)/11` in 0..1.
pub fn gr_meter(kind: VintageGateKind, gr_db: f32) -> f32 {
    let g = gr_db.max(0.0);
    match kind {
        VintageGateKind::E88 => (g / 20.0).min(2.5),
        VintageGateKind::Ssl9000 => (g / 20.0).min(1.0),
        VintageGateKind::D241 => ((g - 1.0) / 11.0).clamp(0.0, 1.0),
    }
}

/// dB per sample to traverse `span` dB in `ms`; `ms <= 0` = instant.
fn rate(span: f32, ms: f32, fs: f64) -> f32 {
    if ms <= 0.0 {
        f32::INFINITY
    } else {
        (span as f64 / (ms as f64 * 1e-3 * fs)) as f32
    }
}

/// Vintage gate (stereo-linked).
pub struct VintageGate {
    kind: VintageGateKind,
    fs: f64,
    v: [f32; 6],
    dirty: bool,
    // Derived.
    thr: f32,
    thr_close: f32,
    depth: f32,
    expander: bool,
    up_rate: f32,
    down_rate: f32,
    hold_samples: u32,
    env_rel: f32,
    open_charge: f32,
    // State.
    env: f32,
    gain_db: f32,
    engaged: bool,
    hold_ctr: u32,
    /// D241G: 0..1 "has been open for a while" integrator (programme-dependent release).
    open_amt: f32,
}

impl VintageGate {
    pub fn new(kind: VintageGateKind) -> Self {
        let mut v = [0.0; 6];
        for (dst, d) in v.iter_mut().zip(kind.slots()) {
            *dst = d.default;
        }
        let mut g = Self {
            kind,
            fs: 48_000.0,
            v,
            dirty: true,
            thr: 0.0,
            thr_close: 0.0,
            depth: 0.0,
            expander: false,
            up_rate: 0.0,
            down_rate: 0.0,
            hold_samples: 0,
            env_rel: 0.0,
            open_charge: 0.0,
            env: 0.0,
            gain_db: 0.0,
            engaged: false,
            hold_ctr: 0,
            open_amt: 0.0,
        };
        g.apply();
        g.reset();
        g
    }

    pub fn kind(&self) -> VintageGateKind {
        self.kind
    }

    /// Open threshold in dBFS after `m40`.
    pub fn threshold_db(&self) -> f32 {
        self.thr
    }

    /// Close threshold in dBFS (open threshold minus hysteresis).
    pub fn close_threshold_db(&self) -> f32 {
        self.thr_close
    }

    /// Current gain in dB (<= 0).
    pub fn gain_db(&self) -> f32 {
        self.gain_db
    }

    fn apply(&mut self) {
        if !self.dirty {
            return;
        }
        self.dirty = false;
        let v = self.v;
        let fs = self.fs;
        self.env_rel = one_pole_coeff(DETECTOR_RELEASE_MS * 1e-3, fs) as f32;
        self.open_charge = (1.0 - one_pole_coeff(D241_OPEN_CHARGE_S, fs)) as f32;
        let (hyst, hold_ms, attack_ms, rel_span, rel_ms);
        match self.kind {
            VintageGateKind::E88 => {
                self.thr = v[0] - if v[5] >= 0.5 { 40.0 } else { 0.0 };
                self.expander = v[1] >= E88_EXP_HYST_DB;
                hyst = if self.expander { 0.0 } else { v[1] };
                self.depth = v[2];
                hold_ms = 0.0;
                attack_ms = E88_ATTACK_MS[(v[4] >= 0.5) as usize];
                rel_span = E88_RELEASE_SPAN_DB;
                rel_ms = v[3];
            }
            VintageGateKind::Ssl9000 => {
                self.thr = v[0];
                self.expander = v[5] >= 0.5;
                hyst = SSL_HYST_DB;
                self.depth = v[1];
                hold_ms = v[2];
                attack_ms = SSL_ATTACK_MS[(v[4] >= 0.5) as usize];
                rel_span = SSL_RELEASE_SPAN_DB;
                rel_ms = v[3];
            }
            VintageGateKind::D241 => {
                let slow = (v[1] >= 0.5) as usize;
                self.thr = v[0];
                self.expander = false;
                hyst = D241_HYST_DB;
                self.depth = D241_DEPTH_DB;
                hold_ms = D241_HOLD_MS[slow];
                attack_ms = D241_ATTACK_MS;
                rel_span = D241_DEPTH_DB;
                rel_ms = D241_RELEASE_MS[slow][0];
            }
        }
        self.thr_close = self.thr - hyst;
        self.up_rate = rate(self.depth, attack_ms, fs);
        self.down_rate = rate(rel_span, rel_ms, fs);
        self.hold_samples = (hold_ms as f64 * 1e-3 * fs).round() as u32;
        self.gain_db = self.gain_db.clamp(-self.depth, 0.0);
    }

    /// D241G programme-dependent release rate for the current open-time amount.
    #[inline]
    fn d241_down_rate(&self) -> f32 {
        let slow = (self.v[1] >= 0.5) as usize;
        let [short, long] = D241_RELEASE_MS[slow];
        let ms = short + (long - short) * self.open_amt;
        rate(D241_DEPTH_DB, ms, self.fs)
    }
}

impl Model for VintageGate {
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
        let d241 = self.kind == VintageGateKind::D241;
        let floor = -self.depth;
        for i in 0..n {
            let (kl, kr) = match sidechain {
                Some((a, b)) => (
                    a.get(i).copied().unwrap_or(0.0),
                    b.get(i).copied().unwrap_or(0.0),
                ),
                None => (left[i], right[i]),
            };
            let k = kl.abs().max(kr.abs());
            self.env = if k >= self.env {
                k
            } else {
                self.env * self.env_rel
            };
            let level = lin_to_db_fast(self.env);
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
            let target = if self.engaged {
                0.0
            } else if self.expander {
                ((EXPANDER_RATIO - 1.0) * (level - self.thr)).clamp(floor, 0.0)
            } else {
                floor
            };
            if d241 {
                if self.engaged {
                    self.open_amt += (1.0 - self.open_amt) * self.open_charge;
                } else if self.gain_db <= floor + 1e-3 {
                    self.open_amt = 0.0;
                }
            }
            let down = if d241 {
                self.d241_down_rate()
            } else {
                self.down_rate
            };
            let g = self.gain_db;
            self.gain_db = if target > g {
                (g + self.up_rate).min(target)
            } else {
                (g - down).max(target)
            };
            let lin = if d241 && self.gain_db <= floor + 1e-3 {
                0.0
            } else {
                db_to_lin_fast(self.gain_db)
            };
            left[i] *= lin;
            right[i] *= lin;
        }
    }

    fn reset(&mut self) {
        self.apply();
        self.env = 0.0;
        self.engaged = false;
        self.hold_ctr = 0;
        self.open_amt = 0.0;
        // Idle = closed (a gate before any signal).
        self.gain_db = -self.depth;
    }

    fn set_param(&mut self, slot: u16, value: ParamValue) {
        let slots = self.kind.slots();
        let Some(def) = (slot as usize).checked_sub(1).and_then(|i| slots.get(i)) else {
            return;
        };
        let x = match def.kind {
            SlotKind::Range(lo, hi) => value.as_f32().clamp(lo, hi),
            SlotKind::Bool => value.as_bool() as u32 as f32,
            SlotKind::Index(n) => value.as_i32().clamp(0, n as i32 - 1) as f32,
        };
        self.v[slot as usize - 1] = x;
        self.dirty = true;
    }

    fn param(&self, slot: u16) -> Option<f32> {
        let i = (slot as usize).checked_sub(1)?;
        self.kind.slots().get(i).map(|_| self.v[i])
    }

    fn load_defaults(&mut self) {
        for (dst, d) in self.v.iter_mut().zip(self.kind.slots()) {
            *dst = d.default;
        }
        self.dirty = true;
    }

    fn commit(&mut self) {
        self.apply();
    }

    fn telemetry(&self) -> ModelTelemetry {
        ModelTelemetry {
            gain_reduction_db: (-self.gain_db).max(0.0),
            engaged: self.engaged,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fidelity::{run_mono, static_curve, time_constants};

    const FS: f64 = 48_000.0;

    fn model(kind: VintageGateKind, params: &[(u16, ParamValue)]) -> VintageGate {
        let mut m = VintageGate::new(kind);
        m.prepare(FS, 64);
        for &(s, v) in params {
            m.set_param(s, v);
        }
        m.commit();
        m.reset();
        m
    }

    fn f(v: f32) -> ParamValue {
        ParamValue::Float(v)
    }

    fn lin(db: f64) -> f32 {
        10f64.powf(db / 20.0) as f32
    }

    /// Gain (dB) at the end of `ms` of DC at `level_db`, continuing from the current state.
    fn run_level(m: &mut VintageGate, level_db: f64, ms: f64) -> f64 {
        let n = ((ms * 1e-3 * FS) as usize).div_ceil(64) * 64;
        let x = vec![lin(level_db); n];
        let y = run_mono(m, &x);
        20.0 * ((y[n - 1] / x[n - 1]).abs() as f64).max(1e-12).log10()
    }

    /// Time (ms) of DC at `level_db`, from the current state, until the gain reaches `target_db`.
    fn time_to_gain(m: &mut VintageGate, level_db: f64, target_db: f64, rising: bool) -> f64 {
        let x = lin(level_db);
        for i in 0..(10.0 * FS) as usize {
            let (mut l, mut r) = ([x], [x]);
            m.process(&mut l, &mut r, None);
            let g = m.gain_db() as f64;
            if (rising && g >= target_db) || (!rising && g <= target_db) {
                return i as f64 / FS * 1e3;
            }
        }
        f64::INFINITY
    }

    #[test]
    fn e88_static_curve_gate_and_range() {
        let mut m = model(
            VintageGateKind::E88,
            &[
                (e88_slot::THR, f(-30.0)),
                (e88_slot::RANGE, f(40.0)),
                (e88_slot::REL, f(100.0)),
                (e88_slot::FAST, f(1.0)),
            ],
        );
        let levels = [-60.0, -45.0, -35.0, -20.0, -10.0, 0.0];
        let out = static_curve(&mut m, FS, &levels, 2.0);
        for (&l, &o) in levels.iter().zip(&out) {
            let expect = if l > -30.0 { l } else { l - 40.0 };
            assert!((o - expect).abs() < 0.3, "{l}: {o} vs {expect}");
        }
    }

    #[test]
    fn e88_m40_shifts_threshold() {
        let mut m = model(
            VintageGateKind::E88,
            &[
                (e88_slot::THR, f(-10.0)),
                (e88_slot::M40, ParamValue::Int(1)),
                (e88_slot::FAST, f(1.0)),
            ],
        );
        assert_eq!(m.threshold_db(), -50.0);
        let out = static_curve(&mut m, FS, &[-45.0, -55.0], 1.0);
        assert!((out[0] + 45.0).abs() < 0.3);
        assert!(out[1] < -55.0 - 55.0);
    }

    #[test]
    fn e88_range_zero_never_attenuates_and_max_is_60() {
        let mut m = model(VintageGateKind::E88, &[(e88_slot::RANGE, f(0.0))]);
        let out = static_curve(&mut m, FS, &[-70.0], 0.5);
        assert!((out[0] + 70.0).abs() < 0.05);
        m.set_param(e88_slot::RANGE, f(99.0));
        assert_eq!(m.param(e88_slot::RANGE), Some(60.0));
        m.commit();
        m.reset();
        assert!((m.gain_db() + 60.0).abs() < 1e-4);
    }

    #[test]
    fn e88_hysteresis_window() {
        let mut m = model(
            VintageGateKind::E88,
            &[
                (e88_slot::THR, f(-30.0)),
                (e88_slot::HYST, f(10.0)),
                (e88_slot::FAST, f(1.0)),
                (e88_slot::REL, f(100.0)),
            ],
        );
        assert_eq!(m.close_threshold_db(), -40.0);
        // Open with a -20 dB burst, then sit at -35 dB (below thr, above close): stays open.
        run_level(&mut m, -20.0, 100.0);
        let g_in_window = run_level(&mut m, -35.0, 500.0);
        assert!(g_in_window > -0.1, "{g_in_window}");
        // Below the close threshold: closes.
        let g_closed = run_level(&mut m, -45.0, 2000.0);
        assert!(g_closed < -55.0, "{g_closed}");
        // From closed, -35 dB (inside the window) does NOT reopen.
        let g_stay = run_level(&mut m, -35.0, 500.0);
        assert!(g_stay < -55.0, "{g_stay}");
        assert!(m.telemetry().gain_reduction_db > 55.0);
    }

    #[test]
    fn e88_attack_fast_vs_slow() {
        let mk = |fast: f32| {
            model(
                VintageGateKind::E88,
                &[
                    (e88_slot::THR, f(-30.0)),
                    (e88_slot::RANGE, f(40.0)),
                    (e88_slot::FAST, f(fast)),
                ],
            )
        };
        // Time to fully open = attack time (range / rate), +-1 sample.
        let slow = time_to_gain(&mut mk(0.0), -10.0, -0.001, true);
        assert!((slow - 2.0).abs() < 0.05, "slow {slow}");
        let fast = time_to_gain(&mut mk(1.0), -10.0, -0.001, true);
        assert!(fast < 0.1, "fast {fast}");
    }

    #[test]
    fn e88_release_time_is_60_db_span() {
        let mut m = model(
            VintageGateKind::E88,
            &[
                (e88_slot::THR, f(-30.0)),
                (e88_slot::RANGE, f(60.0)),
                (e88_slot::REL, f(500.0)),
                (e88_slot::FAST, f(1.0)),
            ],
        );
        run_level(&mut m, -10.0, 50.0);
        assert!(m.gain_db() > -0.01);
        // Silence: detector falls, hysteresis closes, then the gain ramps at 60 dB / 500 ms (linear in dB).
        let t30 = time_to_gain(&mut m, -120.0, -30.0, false);
        let t60 = time_to_gain(&mut m, -120.0, -59.9, false);
        assert!((t60 - 250.0).abs() < 3.0, "{t60} (t30 {t30})");
    }

    #[test]
    fn e88_expander_mode_at_hyst_end_stop() {
        let mut m = model(
            VintageGateKind::E88,
            &[
                (e88_slot::THR, f(-30.0)),
                (e88_slot::HYST, f(25.0)),
                (e88_slot::RANGE, f(40.0)),
                (e88_slot::FAST, f(1.0)),
                (e88_slot::REL, f(100.0)),
            ],
        );
        assert_eq!(m.close_threshold_db(), -30.0);
        let levels = [-60.0, -45.0, -35.0, -20.0];
        let out = static_curve(&mut m, FS, &levels, 2.0);
        for (&l, &o) in levels.iter().zip(&out) {
            let expect = if l > -30.0 {
                l
            } else {
                (-30.0 + 2.0 * (l + 30.0)).max(l - 40.0)
            };
            assert!((o - expect).abs() < 0.4, "{l}: {o} vs {expect}");
        }
    }

    #[test]
    fn ssl_static_gate_and_expander() {
        let gate = |mode: u32| {
            model(
                VintageGateKind::Ssl9000,
                &[
                    (ssl_slot::THR, f(-20.0)),
                    (ssl_slot::RANGE, f(30.0)),
                    (ssl_slot::HLD, f(10.0)),
                    (ssl_slot::REL, f(100.0)),
                    (ssl_slot::FAST, f(1.0)),
                    (ssl_slot::MODE, ParamValue::Enum(mode)),
                ],
            )
        };
        let levels = [-60.0, -40.0, -25.0, -10.0, 0.0];
        let g = static_curve(&mut gate(0), FS, &levels, 2.0);
        let e = static_curve(&mut gate(1), FS, &levels, 2.0);
        for (i, &l) in levels.iter().enumerate() {
            let (eg, ee) = if l > -20.0 {
                (l, l)
            } else {
                (l - 30.0, (-20.0 + 2.0 * (l + 20.0)).max(l - 30.0))
            };
            assert!((g[i] - eg).abs() < 0.4, "gate {l}: {} vs {eg}", g[i]);
            assert!((e[i] - ee).abs() < 0.4, "exp {l}: {} vs {ee}", e[i]);
        }
    }

    #[test]
    fn ssl_hold_time() {
        let mut m = model(
            VintageGateKind::Ssl9000,
            &[
                (ssl_slot::THR, f(-20.0)),
                (ssl_slot::RANGE, f(40.0)),
                (ssl_slot::HLD, f(200.0)),
                (ssl_slot::REL, f(100.0)),
                (ssl_slot::FAST, f(1.0)),
            ],
        );
        run_level(&mut m, -5.0, 50.0);
        // After the key goes silent the gain starts falling only after hold plus the detector fall-through
        // (20 ms * ln(10^(15/20)) ~ 40 ms).
        let t = time_to_gain(&mut m, -120.0, -0.5, false);
        assert!(t > 200.0 && t < 300.0, "{t}");
        m.set_param(ssl_slot::HLD, f(1.0));
        assert_eq!(m.param(ssl_slot::HLD), Some(10.0));
    }

    #[test]
    fn ssl_release_is_40_db_span_and_attacks() {
        let mut m = model(
            VintageGateKind::Ssl9000,
            &[
                (ssl_slot::THR, f(-20.0)),
                (ssl_slot::RANGE, f(40.0)),
                (ssl_slot::HLD, f(10.0)),
                (ssl_slot::REL, f(400.0)),
                (ssl_slot::FAST, f(0.0)),
            ],
        );
        let slow = time_to_gain(&mut m, -5.0, -0.001, true);
        assert!((slow - 1.5).abs() < 0.05, "{slow}");
        m.set_param(ssl_slot::FAST, f(1.0));
        m.commit();
        m.reset();
        let fast = time_to_gain(&mut m, -5.0, -0.001, true);
        assert!(fast < 0.15, "{fast}");
        // Release: 40 dB in 400 ms -> -30 dB is 300 ms after the ramp begins at 0 dB, -10 dB after 100 ms.
        let t10 = time_to_gain(&mut m, -120.0, -10.0, false);
        let t30 = time_to_gain(&mut m, -120.0, -30.0, false);
        assert!((t30 - 200.0).abs() < 3.0, "{t30} (t10 {t10})");
    }

    #[test]
    fn ssl_param_ranges() {
        let mut m = VintageGate::new(VintageGateKind::Ssl9000);
        m.set_param(ssl_slot::THR, f(5.0));
        m.set_param(ssl_slot::RANGE, f(99.0));
        m.set_param(ssl_slot::REL, f(1.0));
        m.set_param(ssl_slot::MODE, ParamValue::Enum(7));
        assert_eq!(m.param(ssl_slot::THR), Some(0.0));
        assert_eq!(m.param(ssl_slot::RANGE), Some(40.0));
        assert_eq!(m.param(ssl_slot::REL), Some(100.0));
        assert_eq!(m.param(ssl_slot::MODE), Some(1.0));
        assert_eq!(m.param(9), None);
    }

    #[test]
    fn d241_static_hard_gate_exact_zero() {
        let mut m = model(VintageGateKind::D241, &[(d241_slot::THR, f(-50.0))]);
        let x: Vec<f32> = vec![lin(-60.0); 24_000];
        let y = run_mono(&mut m, &x);
        assert_eq!(y[y.len() - 1], 0.0);
        let levels = [-40.0, -20.0, 0.0];
        let out = static_curve(&mut m, FS, &levels, 1.0);
        for (&l, &o) in levels.iter().zip(&out) {
            assert!((o - l).abs() < 0.05, "{l}: {o}");
        }
        assert_eq!(m.close_threshold_db(), -52.0);
    }

    #[test]
    fn d241_fixed_fast_attack() {
        let mut m = model(VintageGateKind::D241, &[(d241_slot::THR, f(-40.0))]);
        let t = time_to_gain(&mut m, -20.0, -0.001, true);
        assert!(t < 0.1, "{t}");
    }

    #[test]
    fn d241_programme_dependent_and_slow_release() {
        let release_80 = |slow: i32, open_ms: f64| {
            let mut m = model(
                VintageGateKind::D241,
                &[
                    (d241_slot::THR, f(-40.0)),
                    (d241_slot::SLOW, ParamValue::Int(slow)),
                ],
            );
            run_level(&mut m, -10.0, open_ms);
            // Time from the key going silent to fully closed, minus the hold + detector lead-in.
            let lead = time_to_gain(&mut m, -120.0, -0.5, false);
            let total = time_to_gain(&mut m, -120.0, -79.99, false);
            let _ = lead;
            total
        };
        let short = release_80(0, 20.0);
        let long = release_80(0, 3000.0);
        let slow_long = release_80(1, 3000.0);
        assert!(short > 130.0 && short < 320.0, "short {short}");
        assert!(long > 520.0 && long < 650.0, "long {long}");
        assert!(
            slow_long > 1700.0 && slow_long < 2100.0,
            "slow_long {slow_long}"
        );
        assert!(short < long && long < slow_long);
    }

    #[test]
    fn d241_slow_switch_lengthens_hold() {
        let hold = |slow: i32| {
            let mut m = model(
                VintageGateKind::D241,
                &[
                    (d241_slot::THR, f(-40.0)),
                    (d241_slot::SLOW, ParamValue::Int(slow)),
                ],
            );
            run_level(&mut m, -10.0, 100.0);
            time_to_gain(&mut m, -120.0, -0.5, false)
        };
        let (a, b) = (hold(0), hold(1));
        assert!(b > a + 60.0, "{a} {b}");
    }

    #[test]
    fn time_constants_harness_runs_on_all() {
        for kind in VintageGateKind::ALL {
            let mut m = model(kind, &[]);
            m.load_defaults();
            let tc = time_constants(&mut m, FS, -60.0, -10.0, 4.0);
            assert!(
                tc.delta_up_db > 1.0 && tc.delta_down_db < -1.0,
                "{}",
                kind.id()
            );
        }
    }

    #[test]
    fn external_key_and_stereo_link() {
        let mut m = model(VintageGateKind::E88, &[(e88_slot::THR, f(-30.0))]);
        let key = vec![lin(-10.0); 4800];
        let mut l = vec![lin(-60.0); 4800];
        let mut r = vec![lin(-60.0); 4800];
        for c in 0..75 {
            let s = c * 64..(c + 1) * 64;
            m.process(
                &mut l[s.clone()],
                &mut r[s.clone()],
                Some((&key[s.clone()], &key[s])),
            );
        }
        // A loud key opens the gate on quiet programme.
        assert!((l[4799] / lin(-60.0) - 1.0).abs() < 0.01);
        // Stereo link: only the right key channel loud still opens both.
        m.reset();
        let zero = vec![0.0f32; 64];
        let loud = vec![lin(-10.0); 64];
        let (mut a, mut b) = (vec![0.5f32; 64], vec![0.5f32; 64]);
        for _ in 0..80 {
            a.fill(0.5);
            b.fill(0.5);
            m.process(&mut a, &mut b, Some((&zero, &loud)));
        }
        assert!(a[63] > 0.49 && b[63] > 0.49);
    }

    #[test]
    fn gr_meter_mapping() {
        assert_eq!(gr_meter(VintageGateKind::E88, 20.0), 1.0);
        assert_eq!(gr_meter(VintageGateKind::E88, 80.0), 2.5);
        assert_eq!(gr_meter(VintageGateKind::Ssl9000, 30.0), 1.0);
        assert!((gr_meter(VintageGateKind::D241, 6.0) - 0.45455).abs() < 1e-4);
    }

    #[test]
    fn defaults_load() {
        for kind in VintageGateKind::ALL {
            let mut m = VintageGate::new(kind);
            for s in 1..=6 {
                m.set_param(s, f(1.0));
            }
            m.load_defaults();
            assert_eq!(m.param(1), Some(kind.slots()[0].default));
        }
        let m = VintageGate::new(VintageGateKind::E88);
        assert_eq!(m.param(e88_slot::REL), Some(355.0));
    }
}
