//! [`RtModel`] adapters for the standard models implemented in wing-blocks today (GATE, DUCK,
//! COMP, EXP). Interim glue: T045's per-block model registry is expected to provide `RtModel`
//! instances for every model and replace [`std_factory`].

use wing_blocks::dynamics::{Detector, DynParams, Dynamics, Envelope};
use wing_blocks::gate::{Gate, GateParams, GateRatio};

use super::strip::RtModel;

/// GATE / DUCK (`wing_blocks::gate::Gate`). Positions: thr, range, att, hld, rel, acc, ratio.
pub struct GateModel {
    block: Gate,
    p: GateParams,
    defaults: GateParams,
    dirty: bool,
}

impl GateModel {
    pub fn new(defaults: GateParams, fs: f64, max_block: usize) -> Self {
        let mut block = Gate::new();
        block.prepare(fs, max_block);
        block.set_params(&defaults);
        Self {
            block,
            p: defaults,
            defaults,
            dirty: false,
        }
    }

    pub fn params(&self) -> &GateParams {
        &self.p
    }
}

impl RtModel for GateModel {
    fn load_defaults(&mut self) {
        self.p = self.defaults;
        self.dirty = true;
    }

    fn set_param(&mut self, pos: u16, v: f32) {
        let p = &mut self.p;
        match pos {
            0 => p.threshold_db = v,
            1 => p.range_db = v,
            2 => p.attack_ms = v,
            3 => p.hold_ms = v,
            4 => p.release_ms = v,
            5 => p.accent_pct = v,
            6 => {
                p.ratio = match v as u32 {
                    0 => GateRatio::R1_5,
                    1 => GateRatio::R2,
                    2 => GateRatio::R3,
                    3 => GateRatio::R4,
                    _ => GateRatio::Gate,
                }
            }
            _ => return,
        }
        self.dirty = true;
    }

    fn param(&self, pos: u16) -> Option<f32> {
        let p = &self.p;
        Some(match pos {
            0 => p.threshold_db,
            1 => p.range_db,
            2 => p.attack_ms,
            3 => p.hold_ms,
            4 => p.release_ms,
            5 => p.accent_pct,
            6 => p.ratio as u32 as f32,
            _ => return None,
        })
    }

    fn commit(&mut self) {
        if self.dirty {
            self.block.set_params(&self.p);
            self.dirty = false;
        }
    }

    fn reset(&mut self) {
        self.block.reset();
    }

    fn process(&mut self, l: &mut [f32], r: &mut [f32]) {
        self.block.process(l, r, None);
    }

    fn gain_reduction_db(&self) -> f32 {
        -self.block.gain_db()
    }
}

/// COMP / EXP (`wing_blocks::dynamics::Dynamics`, stereo-linked, fully wet: the slot applies
/// mix/makeup). Positions: thr, ratio, knee, det, att, hld, rel, env, auto.
pub struct DynModel {
    block: Dynamics,
    p: DynParams,
    defaults: DynParams,
    dirty: bool,
}

impl DynModel {
    pub fn new(defaults: DynParams, fs: f64) -> Self {
        let defaults = DynParams {
            mix_pct: 100.0,
            gain_db: 0.0,
            ..defaults
        };
        let mut block = Dynamics::new(defaults);
        block.prepare(fs, 2);
        Self {
            block,
            p: defaults,
            defaults,
            dirty: false,
        }
    }

    pub fn params(&self) -> &DynParams {
        &self.p
    }
}

impl RtModel for DynModel {
    fn load_defaults(&mut self) {
        self.p = self.defaults;
        self.dirty = true;
    }

    fn set_param(&mut self, pos: u16, v: f32) {
        let p = &mut self.p;
        match pos {
            0 => p.thr_db = v,
            1 => p.ratio = v,
            2 => p.knee = v.clamp(0.0, 5.0) as u8,
            3 => {
                p.det = if v as u32 == 0 {
                    Detector::Peak
                } else {
                    Detector::Rms
                }
            }
            4 => p.att_ms = v,
            5 => p.hld_ms = v,
            6 => p.rel_ms = v,
            7 => {
                p.env = if v as u32 == 0 {
                    Envelope::Lin
                } else {
                    Envelope::Log
                }
            }
            8 => p.auto = v != 0.0,
            _ => return,
        }
        self.dirty = true;
    }

    fn param(&self, pos: u16) -> Option<f32> {
        let p = &self.p;
        Some(match pos {
            0 => p.thr_db,
            1 => p.ratio,
            2 => p.knee as f32,
            3 => (p.det == Detector::Rms) as u32 as f32,
            4 => p.att_ms,
            5 => p.hld_ms,
            6 => p.rel_ms,
            7 => (p.env == Envelope::Log) as u32 as f32,
            8 => p.auto as u32 as f32,
            _ => return None,
        })
    }

    fn commit(&mut self) {
        if self.dirty {
            self.block.set_params(&self.p);
            self.dirty = false;
        }
    }

    fn reset(&mut self) {
        self.block.reset();
    }

    fn process(&mut self, l: &mut [f32], r: &mut [f32]) {
        self.block.process(&mut [l, r], None);
    }

    fn gain_reduction_db(&self) -> f32 {
        self.block.gain_reduction_db()
    }
}

/// Factory for [`super::strip::ModelSlot::from_group`]: instances for GATE, DUCK, COMP and EXP;
/// every other model is pass-through.
pub fn std_factory(fs: f64, max_block: usize) -> impl FnMut(u16, &str) -> Option<Box<dyn RtModel>> {
    move |_index, name| -> Option<Box<dyn RtModel>> {
        Some(match name {
            "GATE" => Box::new(GateModel::new(GateParams::gate_default(), fs, max_block)),
            "DUCK" => Box::new(GateModel::new(GateParams::duck_default(), fs, max_block)),
            "COMP" => Box::new(DynModel::new(DynParams::comp(), fs)),
            "EXP" => Box::new(DynModel::new(DynParams::exp(), fs)),
            _ => return None,
        })
    }
}
