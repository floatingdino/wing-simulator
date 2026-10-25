//! WING OSC server (UDP 2223), T014. Spec: `docs/research/protocol.md` section 5.
//!
//! - [`codec`]: a small OSC 1.0 message codec (`i f s b`, no bundles; the type-tag string may be
//!   omitted). Hand-rolled instead of depending on `rosc`: WING's subset is tiny, the codec must
//!   be permissive in WING-specific ways (missing tags, no bundles) and the crate keeps its
//!   dependencies minimal.
//! - [`service`]: the protocol logic (address mapping to `wing-model` schema paths/hashes, get,
//!   set, node set/dumps, `/?`, `/*s` `/*S` `/*b` subscription), independent of sockets.
//! - [`server`]: the UDP socket, receive thread and event-push thread (std threads, `socket2`).
//!
//! Semantics chosen where protocol.md is silent are listed in `service`'s module docs.

pub mod codec;
pub mod server;
pub mod service;

pub use codec::{Arg, DecodeError, Message};
pub use server::{OscAttachment, OscConfig, OscHandle, OscServer, OSC_PORT};
pub use service::{OscService, SubMode, DEFAULT_SUB_TIMEOUT};
