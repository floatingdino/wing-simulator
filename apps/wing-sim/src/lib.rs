//! Library part of the `wing-sim` daemon (shared with its integration tests): the realtime audio
//! path, the real-meter conversion and the local control socket + status file (T064). The
//! binary (`main.rs`) adds the network control servers, persistence and the CLI.

pub mod audio;
pub mod control;
pub mod meters;
pub mod record;
pub mod wav;
