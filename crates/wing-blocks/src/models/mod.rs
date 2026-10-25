//! Alternate processing models (S4/S5 tasks), one module per model family. Each family registers its models
//! in [`crate::model::create_model`].

pub mod comp_console;
pub mod comp_fet;
pub mod comp_multi;
pub mod comp_opto;
pub mod comp_vca;
pub mod dyn_eq;
pub mod eq_alt;
pub mod filter_tool;
pub mod gate_vintage;
pub mod pse;
