//! Library part of the `wing-sim` daemon (shared with its integration tests): the realtime audio
//! path and the real-meter conversion. The binary (`main.rs`) adds the control servers,
//! persistence and the CLI.

pub mod audio;
pub mod meters;
