//! WING native binary protocol ("NRP", TCP 2222) codec. Pure: no I/O.
//!
//! Layers (spec `docs/research/protocol.md` §2–§4):
//! - [`framing`]: `DF`-escaped multiplexing of 14 logical channels ([`framing::Demux`],
//!   [`framing::Mux`]).
//! - [`token`]: channel-1 tokens (navigate/get/set/values/requests) ([`token::Token`]).
//! - [`nodedef`]: node definition records carried by the `DF len` token.
//! - [`meter`]: channel-3 meter requests and UDP meter data packets.
//!
//! [`NrpDecoder`] ties the receive side together: feed raw TCP bytes in any chunking, then drain
//! channel-1 and channel-3 tokens. [`NrpEncoder`] produces escaped wire bytes.

pub mod framing;
pub mod meter;
pub mod nodedef;
mod queue;
pub mod token;

pub use framing::{Demux, Mux, Sink, CH_CONTROL, CH_METER};
pub use meter::{
    MeterCommand, MeterKind, MeterPacket, MeterRequestParser, MeterSelection, MeterToken,
};
pub use nodedef::{NodeDef, NodeKind, Unit};
pub use token::Token;

use std::fmt;

/// Decoding errors. Streaming decoders skip the offending byte and continue.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DecodeError {
    /// Channel-1 lead byte `E0..=FF` (unused).
    UnknownToken(u8),
    /// Channel-3 byte that is not a meter token.
    UnknownMeterToken(u8),
    /// A node definition record longer than the configured cap.
    RecordTooLong(u32),
    /// A record/packet ended early.
    Truncated,
    /// Bytes left over after a complete record.
    TrailingBytes(usize),
    /// Node type code 8..=15.
    UnknownNodeType(u8),
    /// A name in a node definition is not UTF-8.
    InvalidUtf8,
}

/// Encoding errors (value out of the range the token can carry).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EncodeError {
    SmallIntRange(u8),
    StringTooLong(usize),
    IndexRange(u32),
    NameLength(usize),
    RecordLength(usize),
    MeterIndexRange(u8),
}

impl fmt::Display for DecodeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "NRP decode error: {self:?}")
    }
}
impl fmt::Display for EncodeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "NRP encode error: {self:?}")
    }
}
impl std::error::Error for DecodeError {}
impl std::error::Error for EncodeError {}

/// Receive side of one NRP connection.
#[derive(Debug, Default)]
pub struct NrpDecoder {
    demux: Demux,
    control: token::TokenDecoder,
    meter: meter::MeterTokenDecoder,
    other: [u64; framing::NUM_CHANNELS as usize],
}

impl NrpDecoder {
    pub fn new() -> Self {
        Self::default()
    }

    /// Feed raw wire bytes (any chunking). Payload is routed to the per-channel decoders.
    pub fn feed(&mut self, wire: &[u8]) {
        let control = &mut self.control;
        let meter = &mut self.meter;
        let other = &mut self.other;
        self.demux.feed(wire, |ch, b| match ch {
            CH_CONTROL => control.push(b),
            CH_METER => meter.push(b),
            _ => other[ch as usize] += 1,
        });
    }

    /// Next channel-1 token.
    pub fn next_control(&mut self) -> Option<Result<Token<'_>, DecodeError>> {
        self.control.next_token()
    }

    /// Next channel-3 token.
    pub fn next_meter(&mut self) -> Option<Result<MeterToken, DecodeError>> {
        self.meter.next_token()
    }

    /// Payload bytes received (and discarded) on channels other than 1 and 3.
    pub fn other_channel_bytes(&self, ch: u8) -> u64 {
        self.other.get(ch as usize).copied().unwrap_or(0)
    }

    /// Payload bytes received before any channel was selected (discarded).
    pub fn unselected_bytes(&self) -> u64 {
        self.demux.dropped()
    }
}

/// Transmit side of one NRP connection. Appends escaped wire bytes to `out`.
#[derive(Debug, Default)]
pub struct NrpEncoder {
    mux: Mux,
}

impl NrpEncoder {
    pub fn new() -> Self {
        Self::default()
    }

    /// Write a channel-1 token. On error a channel select may already have been written, which
    /// is harmless.
    pub fn control(&mut self, t: &Token, out: &mut Vec<u8>) -> Result<(), EncodeError> {
        t.encode(&mut self.mux.channel_sink(CH_CONTROL, out))
    }

    /// Write a node definition record (`DF len body`) on channel 1.
    pub fn node_def(&mut self, def: &NodeDef, out: &mut Vec<u8>) -> Result<(), EncodeError> {
        let body = def.to_body();
        self.control(&Token::NodeDef(&body), out)
    }

    /// Write a channel-3 token.
    pub fn meter(&mut self, t: &MeterToken, out: &mut Vec<u8>) -> Result<(), EncodeError> {
        t.encode(&mut self.mux.channel_sink(CH_METER, out))
    }

    /// Write raw payload bytes on `ch`, escaped.
    pub fn raw(&mut self, ch: u8, payload: &[u8], out: &mut Vec<u8>) {
        self.mux.write(ch, payload, out);
    }

    /// Unconditionally write `DF Dx` (client keep-alive is `DF D1`).
    pub fn select(&mut self, ch: u8, out: &mut Vec<u8>) {
        self.mux.force_select(ch, out);
    }
}
