//! Discovery, native binary protocol and OSC codec/server for the WING control plane.
//!
//! - [`discovery`]: the UDP 2222 `WING?` responder (T012).
//! - [`nrp`]: the native binary protocol ("NRP", TCP 2222) codec (T018): DF-escaped channel
//!   multiplexing, channel-1 tokens, node definition records, channel-3 meter requests and UDP
//!   meter packets. Pure and allocation-light; no I/O.
//!
//! Networking in this crate uses `std` blocking sockets (plus `socket2` for socket options),
//! one thread per socket/client. The RT audio engine never touches this crate.
//!
//! The spec is `docs/research/protocol.md`. Parts of the codec are derived from libwing
//! (Apache-2.0); see the `NOTICE` file in this crate.

pub mod discovery;
pub mod nrp;
pub mod osc;
