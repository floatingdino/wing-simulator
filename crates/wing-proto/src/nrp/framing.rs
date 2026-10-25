//! Escape-byte channel multiplexing (spec §2.2).
//!
//! Derived from libwing (Apache-2.0, <https://github.com/dannydulai/libwing>, `src/console.rs`
//! `decode_next`) and the Behringer sample receive/transmit routines; see this crate's NOTICE.
//!
//! The byte stream carries 14 logical channels. `DF Dx` (x = 0..=D) switches the current channel,
//! `DF DE` is a literal `DF`, `DF DF` delivers `DF` and stays in the escape state, and `DF xx` for
//! any other `xx` delivers both bytes literally. The encoder side always writes a payload `DF` as
//! `DF DE`, which is unambiguous under the receive routine.

/// The escape byte.
pub const ESC: u8 = 0xDF;
/// Number of logical channels (ChID 0..=13).
pub const NUM_CHANNELS: u8 = 14;
/// Channel carrying get/set/navigate/node-definition traffic.
pub const CH_CONTROL: u8 = 1;
/// Channel carrying meter subscription requests.
pub const CH_METER: u8 = 3;

/// Receive-side demultiplexer state. Feed it bytes in any chunking; it calls `deliver(channel,
/// byte)` for every unescaped payload byte. Bytes before the first channel select are dropped.
#[derive(Clone, Debug, Default)]
pub struct Demux {
    esc: bool,
    ch: Option<u8>,
    dropped: u64,
}

impl Demux {
    pub fn new() -> Self {
        Self::default()
    }

    /// The current receive channel, if one has been selected.
    pub fn channel(&self) -> Option<u8> {
        self.ch
    }

    /// Payload bytes dropped because no channel had been selected yet.
    pub fn dropped(&self) -> u64 {
        self.dropped
    }

    /// Process one wire byte (Behringer normative receive routine).
    #[inline]
    pub fn push(&mut self, b: u8, deliver: &mut impl FnMut(u8, u8)) {
        if !self.esc {
            if b == ESC {
                self.esc = true;
                return;
            }
            self.emit(b, deliver);
            return;
        }
        if b == ESC {
            // DF DF: literal DF, remain escaped.
            self.emit(ESC, deliver);
            return;
        }
        self.esc = false;
        match b {
            0xDE => self.emit(ESC, deliver),
            0xD0..=0xDD => self.ch = Some(b - 0xD0),
            _ => {
                self.emit(ESC, deliver);
                self.emit(b, deliver);
            }
        }
    }

    /// Process a chunk of wire bytes.
    pub fn feed(&mut self, data: &[u8], mut deliver: impl FnMut(u8, u8)) {
        for &b in data {
            self.push(b, &mut deliver);
        }
    }

    #[inline]
    fn emit(&mut self, b: u8, deliver: &mut impl FnMut(u8, u8)) {
        match self.ch {
            Some(ch) => deliver(ch, b),
            None => self.dropped += 1,
        }
    }
}

/// A byte sink. Token encoders write through this so the same code produces raw (unescaped)
/// payload bytes or escaped wire bytes without an intermediate buffer.
pub trait Sink {
    fn put(&mut self, b: u8);
    fn put_all(&mut self, bytes: &[u8]) {
        for &b in bytes {
            self.put(b);
        }
    }
}

impl Sink for Vec<u8> {
    #[inline]
    fn put(&mut self, b: u8) {
        self.push(b);
    }
    fn put_all(&mut self, bytes: &[u8]) {
        self.extend_from_slice(bytes);
    }
}

/// A sink that escapes every payload `DF` as `DF DE` into the wrapped buffer.
pub struct Escaped<'a>(pub &'a mut Vec<u8>);

impl Sink for Escaped<'_> {
    #[inline]
    fn put(&mut self, b: u8) {
        self.0.push(b);
        if b == ESC {
            self.0.push(0xDE);
        }
    }
}

/// Transmit-side multiplexer: tracks the current tx channel and emits `DF Dx` only on change.
#[derive(Clone, Debug, Default)]
pub struct Mux {
    ch: Option<u8>,
}

impl Mux {
    pub fn new() -> Self {
        Self::default()
    }

    /// The current transmit channel.
    pub fn channel(&self) -> Option<u8> {
        self.ch
    }

    /// Switch to `ch` (writing `DF Dx`) if it is not already current. Panics if `ch >= 14`.
    pub fn select(&mut self, ch: u8, out: &mut Vec<u8>) {
        if self.ch != Some(ch) {
            self.force_select(ch, out);
        }
    }

    /// Always write `DF Dx` (e.g. a client keep-alive `DF D1`).
    pub fn force_select(&mut self, ch: u8, out: &mut Vec<u8>) {
        assert!(ch < NUM_CHANNELS, "NRP channel out of range: {ch}");
        out.push(ESC);
        out.push(0xD0 + ch);
        self.ch = Some(ch);
    }

    /// Select `ch` and return an escaping sink for its payload.
    pub fn channel_sink<'a>(&mut self, ch: u8, out: &'a mut Vec<u8>) -> Escaped<'a> {
        self.select(ch, out);
        Escaped(out)
    }

    /// Write raw payload bytes on `ch`, escaped.
    pub fn write(&mut self, ch: u8, payload: &[u8], out: &mut Vec<u8>) {
        self.channel_sink(ch, out).put_all(payload);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn demux_all(wire: &[u8]) -> Vec<(u8, u8)> {
        let mut d = Demux::new();
        let mut v = Vec::new();
        d.feed(wire, |c, b| v.push((c, b)));
        v
    }

    fn payload(v: &[(u8, u8)]) -> Vec<u8> {
        v.iter().map(|&(_, b)| b).collect()
    }

    #[test]
    fn doc_examples() {
        // 3.0.5 doc: D7 02 DF DE AF 0E 02 -> D7 02 DF AF 0E 02
        let w = [0xDF, 0xD1, 0xD7, 0x02, 0xDF, 0xDE, 0xAF, 0x0E, 0x02];
        assert_eq!(
            payload(&demux_all(&w)),
            [0xD7, 0x02, 0xDF, 0xAF, 0x0E, 0x02]
        );
        // 3.1.2 doc: D7 DF DE D4 AB CF 08 -> D7 DF D4 AB CF 08
        let w = [0xDF, 0xD1, 0xD7, 0xDF, 0xDE, 0xD4, 0xAB, 0xCF, 0x08];
        assert_eq!(
            payload(&demux_all(&w)),
            [0xD7, 0xDF, 0xD4, 0xAB, 0xCF, 0x08]
        );
        // 3.1.2 doc: DF followed by a non-special byte is passed through.
        let w = [0xDF, 0xD1, 0xD7, 0xDF, 0xE0, 0x66, 0xC1, 0xD3, 0x00, 0x7F];
        assert_eq!(
            payload(&demux_all(&w)),
            [0xD7, 0xDF, 0xE0, 0x66, 0xC1, 0xD3, 0x00, 0x7F]
        );
    }

    #[test]
    fn channel_switch_and_pre_select_drop() {
        let mut d = Demux::new();
        let mut v = Vec::new();
        d.feed(&[0x01, 0x02, 0xDF, 0xD3, 0xAA, 0xDF, 0xD1, 0xBB], |c, b| {
            v.push((c, b))
        });
        assert_eq!(v, [(3, 0xAA), (1, 0xBB)]);
        assert_eq!(d.dropped(), 2);
    }

    #[test]
    fn df_df_stays_escaped() {
        // DF DF DE D1 -> DF, DF, D1 (as produced by the Behringer tx routine for DF DF D1)
        assert_eq!(
            payload(&demux_all(&[0xDF, 0xD1, 0xDF, 0xDF, 0xDE, 0xD1])),
            [0xDF, 0xDF, 0xD1]
        );
        // DF DF D3 -> DF then channel switch to 3
        let v = demux_all(&[0xDF, 0xD1, 0xDF, 0xDF, 0xD3, 0x05]);
        assert_eq!(v, [(1, 0xDF), (3, 0x05)]);
    }

    #[test]
    fn mux_doc_example() {
        // With tx channel 3, sending d702dfaf0e02 on channel 1 -> df d1 d7 02 df de af 0e 02
        let mut m = Mux::new();
        let mut out = Vec::new();
        m.select(3, &mut out);
        out.clear();
        m.write(1, &[0xD7, 0x02, 0xDF, 0xAF, 0x0E, 0x02], &mut out);
        assert_eq!(out, [0xDF, 0xD1, 0xD7, 0x02, 0xDF, 0xDE, 0xAF, 0x0E, 0x02]);
        // No re-select on the same channel.
        out.clear();
        m.write(1, &[0x01], &mut out);
        assert_eq!(out, [0x01]);
    }

    #[test]
    fn mux_demux_roundtrip_all_bytes() {
        let data: Vec<u8> = (0..=255u8).chain((0..=255u8).rev()).collect();
        let mut m = Mux::new();
        let mut out = Vec::new();
        m.write(1, &data[..300], &mut out);
        m.write(3, &data[300..], &mut out);
        let v = demux_all(&out);
        let ch1: Vec<u8> = v.iter().filter(|x| x.0 == 1).map(|x| x.1).collect();
        let ch3: Vec<u8> = v.iter().filter(|x| x.0 == 3).map(|x| x.1).collect();
        assert_eq!(ch1, &data[..300]);
        assert_eq!(ch3, &data[300..]);
    }
}
