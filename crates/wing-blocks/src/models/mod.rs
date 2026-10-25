//! Alternate processing models (S4/S5 tasks), one module per model family. Each family registers its models
//! in [`crate::model::create_model`].

pub mod comp_vca;
pub mod eq_alt;
pub mod filter_tool;
pub mod gate_vintage;
