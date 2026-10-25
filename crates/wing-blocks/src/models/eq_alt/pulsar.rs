//! placeholder
use super::prims::*;
use super::Layout;

pub(super) const DEFAULTS: [f32; 16] = [0.0; 16];

pub(super) fn design(_ctx: &Ctx, _p: &[f32], _m: f32, out: &mut [Bq]) -> Layout {
    out[0] = UNITY;
    Layout::cascade(1)
}
