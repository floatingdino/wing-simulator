//! Glue between wing-blocks' model framework (T045: [`wing_blocks::model::Model`], catalogs, stubs) and the
//! engine's model-swap host ([`RtModel`] / [`super::strip::ModelSlot`]).
//!
//! `Box<dyn Model>` implements [`RtModel`]: 0-based target positions map to 1-based schema slots and the
//! pipeline's `f32` engineering values are passed as [`ParamValue::Float`] (wing-blocks' adapters accept
//! enum indices and float-enum values in that form). [`model_factory`] creates one instance per model of
//! a wing-model group, implemented models and passthrough stubs alike, so the whole catalog is selectable.

use wing_blocks::eq::EqVariant;
use wing_blocks::model::{create_model, BlockKind, Model, ParamValue};

use super::strip::RtModel;

impl RtModel for Box<dyn Model> {
    fn load_defaults(&mut self) {
        Model::load_defaults(&mut **self);
    }

    #[inline]
    fn set_param(&mut self, pos: u16, value: f32) {
        Model::set_param(&mut **self, pos + 1, ParamValue::Float(value));
    }

    fn param(&self, pos: u16) -> Option<f32> {
        Model::param(&**self, pos + 1)
    }

    fn commit(&mut self) {
        Model::commit(&mut **self);
    }

    fn set_gain_scale(&mut self, mix_pct: f32) {
        Model::set_gain_scale(&mut **self, mix_pct);
    }

    fn reset(&mut self) {
        Model::reset(&mut **self);
    }

    #[inline]
    fn process(&mut self, l: &mut [f32], r: &mut [f32]) {
        Model::process(&mut **self, l, r, None);
    }

    #[inline]
    fn process_keyed(&mut self, l: &mut [f32], r: &mut [f32], key: Option<(&[f32], &[f32])>) {
        Model::process(&mut **self, l, r, key);
    }

    fn gain_reduction_db(&self) -> f32 {
        self.telemetry().gain_reduction_db
    }

    fn meter_bands(&self, out: &mut [f32]) -> usize {
        Model::meter_bands(&**self, out)
    }

    fn latency(&self) -> usize {
        Model::latency(&**self)
    }
}

/// Block kind of a wing-model group key (`ch/*/dyn`, `aux/*/eq`, `ch/*/flt`, `fx/1`, ...).
pub fn block_kind(group_key: &str) -> Option<BlockKind> {
    let (strip, rest) = group_key.split_once('/')?;
    let block = rest.rsplit('/').next()?;
    Some(match block {
        "gate" | "dyn" => BlockKind::Dynamics,
        "flt" => BlockKind::FilterTool,
        "eq" => BlockKind::Eq(match strip {
            "ch" => EqVariant::Channel,
            "aux" => EqVariant::Aux,
            _ => EqVariant::Bus,
        }),
        _ if strip == "fx" => BlockKind::Fx,
        _ => return None,
    })
}

/// Factory for [`super::strip::ModelSlot::from_group`]: a prepared wing-blocks model (or stub) for every
/// model of the group.
pub fn model_factory(
    kind: BlockKind,
    fs: f64,
    max_block: usize,
) -> impl FnMut(u16, &'static str) -> Option<Box<dyn RtModel>> {
    move |_index, name| {
        let mut m = create_model(kind, name);
        m.prepare(fs, max_block);
        Some(Box::new(m) as Box<dyn RtModel>)
    }
}

/// Gate/dynamics factory (kept for [`super::strip::ParamStrip`]).
pub fn std_factory(
    fs: f64,
    max_block: usize,
) -> impl FnMut(u16, &'static str) -> Option<Box<dyn RtModel>> {
    model_factory(BlockKind::Dynamics, fs, max_block)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn group_kinds() {
        assert_eq!(block_kind("ch/*/dyn"), Some(BlockKind::Dynamics));
        assert_eq!(block_kind("ch/*/gate"), Some(BlockKind::Dynamics));
        assert_eq!(block_kind("aux/*/eq"), Some(BlockKind::Eq(EqVariant::Aux)));
        assert_eq!(block_kind("main/*/eq"), Some(BlockKind::Eq(EqVariant::Bus)));
        assert_eq!(block_kind("ch/*/flt"), Some(BlockKind::FilterTool));
        assert_eq!(block_kind("fx/9"), Some(BlockKind::Fx));
        assert_eq!(block_kind("ch/*/in/set"), None);
    }
}
