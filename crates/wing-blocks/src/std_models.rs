//! The standard WING blocks behind the [`Model`] trait (T045 port; DSP unchanged, these are thin adapters):
//!
//! - [`GateModel`]: `GATE` / `DUCK` ([`crate::gate::Gate`]).
//! - [`DynModel`]: `COMP` / `EXP` ([`crate::dynamics::Dynamics`]).
//! - [`StdEqModel`]: `STD` EQ for channel / aux / bus ([`crate::eq::StdEq`]).
//! - [`FilterModel`]: channel LC/HC ([`crate::filter::FilterBlock`]).
//! - [`InputModel`]: channel/aux input stage ([`crate::input::InputStage`]).
//!
//! Slot numbers are the WING schema positions (see `crate::model`). Wrapper parameters (`on`, `mix`, `gain`,
//! key filter) belong to the slot host (wing-engine `ModelSlot`), so the adapters run their block fully wet with unity
//! make-up and the block's own key filter off. Parameter sets are staged and applied once at the start of the
//! next `commit`, `process` or `reset`, so loading 20 slots costs one coefficient update.

use crate::dynamics::{Detector, DynParams, Dynamics, Envelope, RATIOS};
use crate::eq::{BandType, EqParams, EqVariant, StdEq};
use crate::filter::{FilterBlock, FilterParams, HcSlope, LcSlope};
use crate::gate::{Gate, GateParams, GateRatio, ScSource};
use crate::input::{DelayMode, InputParams, InputStage, SourceMode};
use crate::model::{Model, ModelTelemetry, ParamValue};

// ---- GATE / DUCK ------------------------------------------------------------------------------------------

/// GATE/DUCK slots: `thr range att hld rel acc ratio` (DUCK: first five).
pub mod gate_slot {
    pub const THR: u16 = 1;
    pub const RANGE: u16 = 2;
    pub const ATT: u16 = 3;
    pub const HLD: u16 = 4;
    pub const REL: u16 = 5;
    pub const ACC: u16 = 6;
    pub const RATIO: u16 = 7;
}

const GATE_RATIOS: [GateRatio; 5] = [
    GateRatio::R1_5,
    GateRatio::R2,
    GateRatio::R3,
    GateRatio::R4,
    GateRatio::Gate,
];

/// `GATE` / `DUCK` adapter.
pub struct GateModel {
    id: &'static str,
    gate: Gate,
    p: GateParams,
    defaults: GateParams,
    dirty: bool,
    external: bool,
}

impl GateModel {
    fn with(id: &'static str, p: GateParams) -> Self {
        let mut gate = Gate::new();
        gate.set_params(&p);
        gate.reset();
        Self {
            id,
            gate,
            p,
            defaults: p,
            dirty: false,
            external: false,
        }
    }

    pub fn gate() -> Self {
        Self::with("GATE", GateParams::gate_default())
    }

    pub fn duck() -> Self {
        Self::with("DUCK", GateParams::duck_default())
    }

    pub fn params(&self) -> &GateParams {
        &self.p
    }

    pub fn inner(&self) -> &Gate {
        &self.gate
    }

    fn apply(&mut self) {
        if self.dirty {
            self.dirty = false;
            self.gate.set_params(&self.p);
        }
    }
}

impl Model for GateModel {
    fn id(&self) -> &'static str {
        self.id
    }
    fn prepare(&mut self, sample_rate: f64, max_block: usize) {
        self.dirty = true;
        self.apply();
        self.gate.prepare(sample_rate, max_block);
    }
    fn process(
        &mut self,
        left: &mut [f32],
        right: &mut [f32],
        sidechain: Option<(&[f32], &[f32])>,
    ) {
        self.apply();
        let ext = sidechain.is_some();
        if ext != self.external {
            self.external = ext;
            let mut sc = *self.gate.sidechain();
            // Any non-SELF source makes the gate key from the buffer we pass; which strip it comes from is
            // resolved by the engine.
            sc.source = if ext {
                ScSource::Channel(0)
            } else {
                ScSource::SelfKey
            };
            self.gate.set_sidechain(&sc);
        }
        self.gate.process(left, right, sidechain);
    }
    fn reset(&mut self) {
        self.apply();
        self.gate.reset();
    }
    fn set_param(&mut self, slot: u16, v: ParamValue) {
        use gate_slot::*;
        let p = &mut self.p;
        match slot {
            THR => p.threshold_db = v.as_f32(),
            RANGE => p.range_db = v.as_f32(),
            ATT => p.attack_ms = v.as_f32(),
            HLD => p.hold_ms = v.as_f32(),
            REL => p.release_ms = v.as_f32(),
            ACC if self.id == "GATE" => p.accent_pct = v.as_f32(),
            RATIO if self.id == "GATE" => {
                p.ratio = match GATE_RATIOS.get(v.as_index()) {
                    Some(&r) => r,
                    None => return,
                }
            }
            _ => return,
        }
        self.dirty = true;
    }
    fn param(&self, slot: u16) -> Option<f32> {
        use gate_slot::*;
        let p = &self.p;
        let gate = self.id == "GATE";
        Some(match slot {
            THR => p.threshold_db,
            RANGE => p.range_db,
            ATT => p.attack_ms,
            HLD => p.hold_ms,
            REL => p.release_ms,
            ACC if gate => p.accent_pct,
            RATIO if gate => GATE_RATIOS.iter().position(|&r| r == p.ratio).unwrap_or(2) as f32,
            _ => return None,
        })
    }
    fn load_defaults(&mut self) {
        self.p = self.defaults;
        self.dirty = true;
    }
    fn commit(&mut self) {
        self.apply();
    }
    fn telemetry(&self) -> ModelTelemetry {
        ModelTelemetry {
            gain_reduction_db: (-self.gate.gain_db()).max(0.0),
            engaged: self.gate.is_engaged(),
        }
    }
}

// ---- COMP / EXP -------------------------------------------------------------------------------------------

/// COMP/EXP slots: `thr ratio knee det att hld rel env auto`.
pub mod dyn_slot {
    pub const THR: u16 = 1;
    pub const RATIO: u16 = 2;
    pub const KNEE: u16 = 3;
    pub const DET: u16 = 4;
    pub const ATT: u16 = 5;
    pub const HLD: u16 = 6;
    pub const REL: u16 = 7;
    pub const ENV: u16 = 8;
    pub const AUTO: u16 = 9;
}

/// `COMP` / `EXP` adapter (stereo-linked).
pub struct DynModel {
    id: &'static str,
    d: Dynamics,
    p: DynParams,
    defaults: DynParams,
    dirty: bool,
}

impl DynModel {
    fn with(id: &'static str, p: DynParams) -> Self {
        let mut d = Dynamics::new(p);
        d.prepare(48_000.0, 2);
        Self {
            id,
            d,
            p,
            defaults: p,
            dirty: false,
        }
    }

    pub fn comp() -> Self {
        Self::with("COMP", DynParams::comp())
    }

    pub fn exp() -> Self {
        Self::with("EXP", DynParams::exp())
    }

    pub fn params(&self) -> &DynParams {
        &self.p
    }

    fn apply(&mut self) {
        if self.dirty {
            self.dirty = false;
            self.d.set_params(&self.p);
        }
    }
}

impl Model for DynModel {
    fn id(&self) -> &'static str {
        self.id
    }
    fn prepare(&mut self, sample_rate: f64, _max_block: usize) {
        self.dirty = true;
        self.apply();
        self.d.prepare(sample_rate, 2);
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
            Some((kl, kr)) => {
                let keys = [kl, kr];
                self.d.process(&mut io, Some(&keys));
            }
            None => self.d.process(&mut io, None),
        }
    }
    fn latency(&self) -> usize {
        self.d.latency()
    }
    fn reset(&mut self) {
        self.apply();
        self.d.reset();
    }
    fn set_param(&mut self, slot: u16, v: ParamValue) {
        use dyn_slot::*;
        let p = &mut self.p;
        match slot {
            THR => p.thr_db = v.as_f32(),
            RATIO => {
                p.ratio = match v {
                    ParamValue::Enum(i) => match RATIOS.get(i as usize) {
                        Some(&r) => r,
                        None => return,
                    },
                    other => other.as_f32(),
                }
            }
            KNEE => p.knee = v.as_i32().clamp(0, 5) as u8,
            DET => {
                p.det = if v.as_i32() == 0 {
                    Detector::Peak
                } else {
                    Detector::Rms
                }
            }
            ATT => p.att_ms = v.as_f32(),
            HLD => p.hld_ms = v.as_f32(),
            REL => p.rel_ms = v.as_f32(),
            ENV => {
                p.env = if v.as_i32() == 0 {
                    Envelope::Lin
                } else {
                    Envelope::Log
                }
            }
            AUTO => p.auto = v.as_bool(),
            _ => return,
        }
        self.dirty = true;
    }
    fn param(&self, slot: u16) -> Option<f32> {
        use dyn_slot::*;
        let p = &self.p;
        Some(match slot {
            THR => p.thr_db,
            RATIO => p.ratio,
            KNEE => p.knee as f32,
            DET => (p.det == Detector::Rms) as u32 as f32,
            ATT => p.att_ms,
            HLD => p.hld_ms,
            REL => p.rel_ms,
            ENV => (p.env == Envelope::Log) as u32 as f32,
            AUTO => p.auto as u32 as f32,
            _ => return None,
        })
    }
    fn load_defaults(&mut self) {
        self.p = self.defaults;
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

// ---- STD EQ -----------------------------------------------------------------------------------------------

/// `STD` EQ adapter. Slots (schema order): `lg lf lq leq`, then `g f q` per peaking band (4 or 6), then
/// `hg hf hq heq`, then `tilt` (bus only). `leq`/`heq` enum indices follow [`BandType::ALL`].
pub struct StdEqModel {
    eq: StdEq,
    p: EqParams,
    defaults: EqParams,
    dirty: bool,
}

impl StdEqModel {
    pub fn new(variant: EqVariant) -> Self {
        let p = EqParams {
            on: true,
            mix: 100.0,
            ..EqParams::defaults(variant)
        };
        let mut eq = StdEq::new(variant);
        eq.set_params(&p);
        eq.reset();
        Self {
            eq,
            p,
            defaults: p,
            dirty: false,
        }
    }

    pub fn params(&self) -> &EqParams {
        &self.p
    }

    pub fn inner(&self) -> &StdEq {
        &self.eq
    }

    fn apply(&mut self) {
        if self.dirty {
            self.dirty = false;
            self.eq.set_params(&self.p);
        }
    }
}

impl Model for StdEqModel {
    fn id(&self) -> &'static str {
        "STD"
    }
    fn prepare(&mut self, sample_rate: f64, max_block: usize) {
        self.dirty = true;
        self.apply();
        self.eq.prepare(sample_rate, max_block);
    }
    fn process(&mut self, left: &mut [f32], right: &mut [f32], _sc: Option<(&[f32], &[f32])>) {
        self.apply();
        self.eq.process(left, right);
    }
    fn reset(&mut self) {
        self.apply();
        self.eq.reset();
    }
    fn set_param(&mut self, slot: u16, v: ParamValue) {
        let n = self.eq.variant().peak_bands() as u16;
        let p = &mut self.p;
        let band_type = |v: ParamValue| BandType::ALL.get(v.as_index()).copied();
        let s = slot;
        match s {
            1 => p.low.gain_db = v.as_f32(),
            2 => p.low.freq = v.as_f32(),
            3 => p.low.q = v.as_f32(),
            4 => match band_type(v) {
                Some(t) => p.low_type = t,
                None => return,
            },
            _ if s >= 5 && s < 5 + 3 * n => {
                let b = &mut p.bands[((s - 5) / 3) as usize];
                match (s - 5) % 3 {
                    0 => b.gain_db = v.as_f32(),
                    1 => b.freq = v.as_f32(),
                    _ => b.q = v.as_f32(),
                }
            }
            _ if s >= 5 + 3 * n && s < 9 + 3 * n => match s - (5 + 3 * n) {
                0 => p.high.gain_db = v.as_f32(),
                1 => p.high.freq = v.as_f32(),
                2 => p.high.q = v.as_f32(),
                _ => match band_type(v) {
                    Some(t) => p.high_type = t,
                    None => return,
                },
            },
            _ if s == 9 + 3 * n && self.eq.variant().has_tilt() => p.tilt_db = v.as_f32(),
            _ => return,
        }
        self.dirty = true;
    }
    fn param(&self, s: u16) -> Option<f32> {
        let n = self.eq.variant().peak_bands() as u16;
        let p = &self.p;
        let ty = |t: BandType| BandType::ALL.iter().position(|&x| x == t).unwrap_or(1) as f32;
        Some(match s {
            1 => p.low.gain_db,
            2 => p.low.freq,
            3 => p.low.q,
            4 => ty(p.low_type),
            _ if s >= 5 && s < 5 + 3 * n => {
                let b = &p.bands[((s - 5) / 3) as usize];
                [b.gain_db, b.freq, b.q][((s - 5) % 3) as usize]
            }
            _ if s >= 5 + 3 * n && s < 9 + 3 * n => match s - (5 + 3 * n) {
                0 => p.high.gain_db,
                1 => p.high.freq,
                2 => p.high.q,
                _ => ty(p.high_type),
            },
            _ if s == 9 + 3 * n && self.eq.variant().has_tilt() => p.tilt_db,
            _ => return None,
        })
    }
    fn load_defaults(&mut self) {
        self.p = self.defaults;
        self.dirty = true;
    }
    fn commit(&mut self) {
        self.apply();
    }
}

// ---- Filter (LC/HC) ---------------------------------------------------------------------------------------

/// Channel `flt` keys by position: `lc lcf lcs hc hcf hcs` (`tf mdl 1 2` are the filter tool slot).
pub mod flt_slot {
    pub const LC: u16 = 1;
    pub const LCF: u16 = 2;
    pub const LCS: u16 = 3;
    pub const HC: u16 = 4;
    pub const HCF: u16 = 5;
    pub const HCS: u16 = 6;
}

/// Channel low/high cut adapter (single model, id `STD`).
pub struct FilterModel {
    f: FilterBlock,
    p: FilterParams,
    dirty: bool,
}

impl Default for FilterModel {
    fn default() -> Self {
        Self::new()
    }
}

impl FilterModel {
    pub fn new() -> Self {
        let f = FilterBlock::new();
        let p = *f.params();
        Self { f, p, dirty: false }
    }

    pub fn params(&self) -> &FilterParams {
        &self.p
    }

    /// True when neither cut is on (and settled): the strip may skip the block.
    pub fn is_off(&self) -> bool {
        !self.p.lc_on && !self.p.hc_on && !self.dirty
    }

    fn apply(&mut self) {
        if self.dirty {
            self.dirty = false;
            self.f.set_params(&self.p);
        }
    }
}

impl Model for FilterModel {
    fn id(&self) -> &'static str {
        "STD"
    }
    fn prepare(&mut self, sample_rate: f64, max_block: usize) {
        self.dirty = true;
        self.apply();
        self.f.prepare(sample_rate, max_block);
    }
    fn process(&mut self, left: &mut [f32], right: &mut [f32], _sc: Option<(&[f32], &[f32])>) {
        self.apply();
        self.f.process(left, right);
    }
    fn reset(&mut self) {
        self.apply();
        self.f.reset();
    }
    fn set_param(&mut self, slot: u16, v: ParamValue) {
        use flt_slot::*;
        let p = &mut self.p;
        match slot {
            LC => p.lc_on = v.as_bool(),
            LCF => p.lc_freq = v.as_f32(),
            LCS => {
                p.lc_slope = match v.as_i32() {
                    0 => LcSlope::Db6,
                    1 => LcSlope::Db12,
                    2 => LcSlope::Db18,
                    3 => LcSlope::Db24,
                    _ => return,
                }
            }
            HC => p.hc_on = v.as_bool(),
            HCF => p.hc_freq = v.as_f32(),
            HCS => {
                p.hc_slope = match v.as_i32() {
                    0 => HcSlope::Db6,
                    1 => HcSlope::Db12,
                    _ => return,
                }
            }
            _ => return,
        }
        self.dirty = true;
    }
    fn load_defaults(&mut self) {
        self.p = FilterParams::default();
        self.dirty = true;
    }
    fn commit(&mut self) {
        self.apply();
    }
}

// ---- Input stage ------------------------------------------------------------------------------------------

/// `in/set` keys by position: `$mode srcauto altsrc inv trim bal $g $vph dlymode dly dlyon`.
/// `srcauto`, `altsrc` and `$vph` are routing / no-ops and ignored here.
pub mod in_slot {
    pub const MODE: u16 = 1;
    pub const INV: u16 = 4;
    pub const TRIM: u16 = 5;
    pub const BAL: u16 = 6;
    pub const GAIN: u16 = 7;
    pub const DLYMODE: u16 = 9;
    pub const DLY: u16 = 10;
    pub const DLYON: u16 = 11;
}

/// Input stage adapter (single model, id `STD`).
pub struct InputModel {
    s: InputStage,
    p: InputParams,
    dirty: bool,
}

impl Default for InputModel {
    fn default() -> Self {
        Self::new()
    }
}

impl InputModel {
    pub fn new() -> Self {
        let s = InputStage::new();
        let p = *s.params();
        Self { s, p, dirty: false }
    }

    pub fn params(&self) -> &InputParams {
        &self.p
    }

    fn apply(&mut self) {
        if self.dirty {
            self.dirty = false;
            self.s.set_params(&self.p);
        }
    }
}

impl Model for InputModel {
    fn id(&self) -> &'static str {
        "STD"
    }
    fn prepare(&mut self, sample_rate: f64, max_block: usize) {
        self.dirty = true;
        self.apply();
        self.s.prepare(sample_rate, max_block);
    }
    fn process(&mut self, left: &mut [f32], right: &mut [f32], _sc: Option<(&[f32], &[f32])>) {
        self.apply();
        self.s.process(left, right);
    }
    fn latency(&self) -> usize {
        // The input delay is a deliberate alignment delay, not processing latency.
        0
    }
    fn reset(&mut self) {
        self.apply();
        self.s.reset();
    }
    fn set_param(&mut self, slot: u16, v: ParamValue) {
        use in_slot::*;
        let p = &mut self.p;
        match slot {
            MODE => {
                p.source_mode = match v.as_i32() {
                    1 => SourceMode::Stereo,
                    2 => SourceMode::MidSide,
                    _ => SourceMode::Mono,
                }
            }
            INV => p.invert = v.as_bool(),
            TRIM => p.trim_db = v.as_f32(),
            BAL => p.balance_db = v.as_f32(),
            GAIN => p.gain_db = v.as_f32(),
            DLYMODE => {
                p.delay_mode = match v.as_i32() {
                    1 => DelayMode::Feet,
                    2 => DelayMode::Ms,
                    3 => DelayMode::Samples,
                    _ => DelayMode::Meters,
                }
            }
            DLY => p.delay = v.as_f32(),
            DLYON => p.delay_on = v.as_bool(),
            _ => return,
        }
        self.dirty = true;
    }
    fn load_defaults(&mut self) {
        self.p = InputParams::default();
        self.dirty = true;
    }
    fn commit(&mut self) {
        self.apply();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn dc(m: &mut dyn Model, level: f32, n: usize) -> f32 {
        let mut out = 0.0;
        let mut l = [0.0f32; 64];
        let mut r = [0.0f32; 64];
        for _ in 0..n / 64 {
            l.fill(level);
            r.fill(level);
            m.process(&mut l, &mut r, None);
            out = l[63];
        }
        out
    }

    #[test]
    fn adapters_match_direct_blocks() {
        // GATE via slots == Gate with the same params.
        let mut gm = GateModel::gate();
        gm.prepare(48_000.0, 64);
        gm.set_param(gate_slot::THR, ParamValue::Float(-30.0));
        gm.set_param(gate_slot::RANGE, ParamValue::Float(20.0));
        gm.set_param(gate_slot::RATIO, ParamValue::Enum(4));
        let mut g = Gate::new();
        g.prepare(48_000.0, 64);
        g.set_params(&GateParams {
            threshold_db: -30.0,
            range_db: 20.0,
            ratio: GateRatio::Gate,
            ..GateParams::gate_default()
        });
        g.reset();
        gm.reset();
        let mut a: Vec<f32> = (0..256).map(|i| ((i as f32) * 0.05).sin() * 0.02).collect();
        let mut b = a.clone();
        let mut a2 = a.clone();
        let mut b2 = a.clone();
        for c in 0..4 {
            let s = c * 64..(c + 1) * 64;
            gm.process(&mut a[s.clone()], &mut b[s.clone()], None);
            g.process(&mut a2[s.clone()], &mut b2[s], None);
        }
        assert_eq!(a, a2);
        assert_eq!(b, b2);
        assert!(gm.telemetry().gain_reduction_db > 0.0);

        // COMP ratio via float or enum index.
        let mut c = DynModel::comp();
        c.prepare(48_000.0, 64);
        c.set_param(dyn_slot::RATIO, ParamValue::Enum(9));
        c.apply();
        assert_eq!(c.params().ratio, 4.0);
        c.set_param(dyn_slot::RATIO, ParamValue::Float(8.0));
        c.set_param(dyn_slot::THR, ParamValue::Float(-30.0));
        c.set_param(dyn_slot::KNEE, ParamValue::Int(0));
        let out = dc(&mut c, 0.5, 48_000);
        // -6 dB in, thr -30, ratio 8: -30 + 24/8 = -27 dB out (static).
        let db = 20.0 * out.log10();
        assert!((db + 27.0).abs() < 0.3, "{db}");
        assert!(c.telemetry().gain_reduction_db > 20.0);
    }

    #[test]
    fn std_eq_slot_layout() {
        let mut ch = StdEqModel::new(EqVariant::Channel);
        ch.set_param(8, ParamValue::Float(6.0)); // 2g
        ch.set_param(17, ParamValue::Float(-3.0)); // hg
        ch.set_param(20, ParamValue::Enum(0)); // heq PEQ
        ch.apply();
        assert_eq!(ch.params().bands[1].gain_db, 6.0);
        assert_eq!(ch.params().high.gain_db, -3.0);
        assert_eq!(ch.params().high_type, BandType::Peq);
        let mut bus = StdEqModel::new(EqVariant::Bus);
        bus.set_param(20, ParamValue::Float(4.0)); // 6g
        bus.set_param(26, ParamValue::Enum(12)); // heq LR48
        bus.set_param(27, ParamValue::Float(2.0)); // tilt
        bus.apply();
        assert_eq!(bus.params().bands[5].gain_db, 4.0);
        assert_eq!(bus.params().high_type, BandType::Lr48);
        assert_eq!(bus.params().tilt_db, 2.0);
    }

    #[test]
    fn filter_and_input_slots() {
        let mut f = FilterModel::new();
        assert!(f.is_off());
        f.set_param(flt_slot::LC, ParamValue::Int(1));
        f.set_param(flt_slot::LCS, ParamValue::Enum(1));
        f.set_param(flt_slot::LCF, ParamValue::Float(200.0));
        f.apply();
        assert!(f.params().lc_on && f.params().lc_slope == LcSlope::Db12);
        let mut i = InputModel::new();
        i.prepare(48_000.0, 64);
        i.set_param(in_slot::TRIM, ParamValue::Float(6.0));
        i.set_param(in_slot::INV, ParamValue::Int(1));
        let out = dc(&mut i, 0.25, 4800);
        assert!((out + 0.25 * 1.995).abs() < 1e-3, "{out}");
    }
}
