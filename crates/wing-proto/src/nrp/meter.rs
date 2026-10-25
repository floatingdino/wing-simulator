//! Meters: channel-3 subscription requests and UDP meter data packets (spec §4).
//!
//! Derived from libwing (Apache-2.0, <https://github.com/dannydulai/libwing>, `src/console.rs`
//! `request_meter`, `read_meters`); see this crate's NOTICE. Report ids are full u32 here (libwing
//! reads only the top 16 bits).

use super::framing::Sink;
use super::queue::ByteQueue;
use super::{DecodeError, EncodeError};

/// Meter type selector tokens `A0..=AF`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum MeterKind {
    Channel = 0xA0,
    Aux = 0xA1,
    Bus = 0xA2,
    Main = 0xA3,
    Matrix = 0xA4,
    Dca = 0xA5,
    Fx = 0xA6,
    /// Source group 1..=16 (local, aux, AES50 A/B/C, ...; see spec §4.2).
    Source = 0xA7,
    /// Output group 1..=11.
    Output = 0xA8,
    Monitor = 0xA9,
    Rta = 0xAA,
    ChannelV2 = 0xAB,
    AuxV2 = 0xAC,
    BusV2 = 0xAD,
    MainV2 = 0xAE,
    MatrixV2 = 0xAF,
}

impl MeterKind {
    pub fn from_token(t: u8) -> Option<MeterKind> {
        use MeterKind::*;
        Some(match t {
            0xA0 => Channel,
            0xA1 => Aux,
            0xA2 => Bus,
            0xA3 => Main,
            0xA4 => Matrix,
            0xA5 => Dca,
            0xA6 => Fx,
            0xA7 => Source,
            0xA8 => Output,
            0xA9 => Monitor,
            0xAA => Rta,
            0xAB => ChannelV2,
            0xAC => AuxV2,
            0xAD => BusV2,
            0xAE => MainV2,
            0xAF => MatrixV2,
            _ => return None,
        })
    }

    pub fn token(self) -> u8 {
        self as u8
    }

    /// int16 words per selected index, where fixed. `None` for source/output groups, whose size
    /// depends on the group (spec §4.2).
    pub fn words_per_index(self) -> Option<usize> {
        use MeterKind::*;
        match self {
            Channel | Aux | Bus | Main | Matrix => Some(8),
            Dca => Some(4),
            Fx => Some(10),
            Source | Output => None,
            Monitor => Some(6),
            Rta => Some(120),
            ChannelV2 | AuxV2 | BusV2 | MainV2 | MatrixV2 => Some(11),
        }
    }
}

/// One channel-3 token.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MeterToken {
    /// `D3 u16`: client UDP port for meter data.
    Port(u16),
    /// `D4 u32`: report id; alone it renews the report's 5 s lifetime.
    ReportId(u32),
    /// `DC`: start of a collection definition for the preceding report id.
    Begin,
    /// `A0..=AF`: meter type.
    Kind(MeterKind),
    /// `00..=7F`: zero-based index (index 0 = ch 1).
    Index(u8),
    /// `DE`: end of collection.
    End,
}

impl MeterToken {
    pub fn encode(&self, s: &mut impl Sink) -> Result<(), EncodeError> {
        match *self {
            MeterToken::Port(p) => {
                s.put(0xD3);
                s.put_all(&p.to_be_bytes());
            }
            MeterToken::ReportId(id) => {
                s.put(0xD4);
                s.put_all(&id.to_be_bytes());
            }
            MeterToken::Begin => s.put(0xDC),
            MeterToken::Kind(k) => s.put(k.token()),
            MeterToken::Index(i) => {
                if i > 0x7F {
                    return Err(EncodeError::MeterIndexRange(i));
                }
                s.put(i);
            }
            MeterToken::End => s.put(0xDE),
        }
        Ok(())
    }
}

/// Parse one channel-3 token; same contract as [`super::token::parse_token`].
pub fn parse_meter_token(input: &[u8]) -> Result<Option<(MeterToken, usize)>, DecodeError> {
    let Some(&t) = input.first() else {
        return Ok(None);
    };
    Ok(Some(match t {
        0x00..=0x7F => (MeterToken::Index(t), 1),
        0xA0..=0xAF => (MeterToken::Kind(MeterKind::from_token(t).unwrap()), 1),
        0xD3 => {
            if input.len() < 3 {
                return Ok(None);
            }
            (
                MeterToken::Port(u16::from_be_bytes([input[1], input[2]])),
                3,
            )
        }
        0xD4 => {
            if input.len() < 5 {
                return Ok(None);
            }
            let id = u32::from_be_bytes([input[1], input[2], input[3], input[4]]);
            (MeterToken::ReportId(id), 5)
        }
        0xDC => (MeterToken::Begin, 1),
        0xDE => (MeterToken::End, 1),
        _ => return Err(DecodeError::UnknownMeterToken(t)),
    }))
}

/// Streaming channel-3 token decoder.
#[derive(Debug, Default)]
pub struct MeterTokenDecoder {
    q: ByteQueue,
}

impl MeterTokenDecoder {
    pub fn new() -> Self {
        Self::default()
    }

    #[inline]
    pub fn push(&mut self, b: u8) {
        self.q.push(b);
    }

    pub fn extend(&mut self, bytes: &[u8]) {
        self.q.extend(bytes);
    }

    pub fn next_token(&mut self) -> Option<Result<MeterToken, DecodeError>> {
        self.q.compact();
        match parse_meter_token(self.q.pending()) {
            Ok(None) => None,
            Ok(Some((t, n))) => {
                self.q.pos += n;
                Some(Ok(t))
            }
            Err(e) => {
                self.q.pos += 1;
                Some(Err(e))
            }
        }
    }
}

/// One meter type with its zero-based indices, in request order.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct MeterSelection {
    pub kind: MeterKind,
    pub indices: Vec<u8>,
}

/// Human-readable summary of a report definition for logs (T082): kinds with 1-based index ranges, e.g.
/// `ChannelV2 1-16, MainV2 1, Fx 5`.
pub fn describe_selections(selections: &[MeterSelection]) -> String {
    use std::fmt::Write;
    let mut s = String::new();
    for (k, sel) in selections.iter().enumerate() {
        if k > 0 {
            s.push_str(", ");
        }
        let _ = write!(s, "{:?}", sel.kind);
        let mut i = 0;
        while i < sel.indices.len() {
            let a = sel.indices[i];
            let mut j = i;
            while j + 1 < sel.indices.len() && sel.indices[j + 1] == sel.indices[j].wrapping_add(1)
            {
                j += 1;
            }
            let sep = if i == 0 { " " } else { "," };
            if j > i {
                let _ = write!(s, "{sep}{}-{}", a as u32 + 1, sel.indices[j] as u32 + 1);
            } else {
                let _ = write!(s, "{sep}{}", a as u32 + 1);
            }
            i = j + 1;
        }
    }
    s
}

/// A semantic meter command assembled from channel-3 tokens.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum MeterCommand {
    /// Send meter data to this UDP port (on the client's IP).
    SetPort(u16),
    /// A `D4 id` was seen: (re)start the 5 s lifetime of report `id` if it exists. Emitted for
    /// every `D4`, including the one that precedes a definition.
    Renew(u32),
    /// Define (or redefine) report `id` with these selections.
    Define {
        id: u32,
        selections: Vec<MeterSelection>,
    },
}

/// Folds [`MeterToken`]s into [`MeterCommand`]s.
#[derive(Debug, Default)]
pub struct MeterRequestParser {
    last_id: Option<u32>,
    collecting: Option<Vec<MeterSelection>>,
}

impl MeterRequestParser {
    pub fn new() -> Self {
        Self::default()
    }

    /// Feed one token. Index tokens outside a collection or before any type are ignored, as is
    /// `DC` with no preceding report id.
    pub fn feed(&mut self, t: MeterToken) -> Option<MeterCommand> {
        match t {
            MeterToken::Port(p) => Some(MeterCommand::SetPort(p)),
            MeterToken::ReportId(id) => {
                self.last_id = Some(id);
                Some(MeterCommand::Renew(id))
            }
            MeterToken::Begin => {
                if self.last_id.is_some() {
                    self.collecting = Some(Vec::new());
                }
                None
            }
            MeterToken::Kind(kind) => {
                if let Some(c) = self.collecting.as_mut() {
                    c.push(MeterSelection {
                        kind,
                        indices: Vec::new(),
                    });
                }
                None
            }
            MeterToken::Index(i) => {
                if let Some(sel) = self.collecting.as_mut().and_then(|c| c.last_mut()) {
                    sel.indices.push(i);
                }
                None
            }
            MeterToken::End => {
                let selections = self.collecting.take()?;
                Some(MeterCommand::Define {
                    id: self.last_id?,
                    selections,
                })
            }
        }
    }
}

/// Write a complete definition (`D4 id DC {kind idx*}* DE`) as channel-3 payload.
pub fn encode_meter_define(
    id: u32,
    selections: &[MeterSelection],
    s: &mut impl Sink,
) -> Result<(), EncodeError> {
    if let Some(i) = selections
        .iter()
        .flat_map(|x| x.indices.iter())
        .find(|&&i| i > 0x7F)
    {
        return Err(EncodeError::MeterIndexRange(*i));
    }
    MeterToken::ReportId(id).encode(s)?;
    MeterToken::Begin.encode(s)?;
    for sel in selections {
        MeterToken::Kind(sel.kind).encode(s)?;
        for &i in &sel.indices {
            MeterToken::Index(i).encode(s)?;
        }
    }
    MeterToken::End.encode(s)
}

/// Append a UDP meter packet: `reportId.l { int16 }*`, big-endian.
pub fn encode_meter_packet(report_id: u32, values: &[i16], out: &mut Vec<u8>) {
    out.reserve(4 + 2 * values.len());
    out.extend_from_slice(&report_id.to_be_bytes());
    for v in values {
        out.extend_from_slice(&v.to_be_bytes());
    }
}

/// Write a UDP meter packet into a fixed buffer; returns the length, or `None` if it does not fit.
pub fn encode_meter_packet_into(report_id: u32, values: &[i16], out: &mut [u8]) -> Option<usize> {
    let len = 4 + 2 * values.len();
    if out.len() < len {
        return None;
    }
    out[..4].copy_from_slice(&report_id.to_be_bytes());
    for (dst, v) in out[4..len].as_chunks_mut::<2>().0.iter_mut().zip(values) {
        *dst = v.to_be_bytes();
    }
    Some(len)
}

/// A borrowed, parsed UDP meter packet.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MeterPacket<'a> {
    pub report_id: u32,
    data: &'a [u8],
}

impl<'a> MeterPacket<'a> {
    /// Parse a datagram. A trailing odd byte is an error.
    pub fn parse(datagram: &'a [u8]) -> Result<MeterPacket<'a>, DecodeError> {
        if datagram.len() < 4 {
            return Err(DecodeError::Truncated);
        }
        let data = &datagram[4..];
        if !data.len().is_multiple_of(2) {
            return Err(DecodeError::TrailingBytes(1));
        }
        Ok(MeterPacket {
            report_id: u32::from_be_bytes([datagram[0], datagram[1], datagram[2], datagram[3]]),
            data,
        })
    }

    pub fn len(&self) -> usize {
        self.data.len() / 2
    }

    pub fn is_empty(&self) -> bool {
        self.data.is_empty()
    }

    pub fn values(&self) -> impl Iterator<Item = i16> + 'a {
        self.data
            .as_chunks::<2>()
            .0
            .iter()
            .map(|c| i16::from_be_bytes(*c))
    }
}

/// Convert dB to the wire's signed 1/256 dB scale, saturating.
pub fn db_to_meter(db: f32) -> i16 {
    if db.is_nan() {
        return i16::MIN;
    }
    (db * 256.0).round().clamp(i16::MIN as f32, i16::MAX as f32) as i16
}

/// Convert the wire's 1/256 dB scale to dB.
pub fn meter_to_db(v: i16) -> f32 {
    v as f32 / 256.0
}

/// Gain-reduction full scale (dB at word 256) of most dynamics models (protocol.md §4.2).
pub const GR_FULL_SCALE_DB: f32 = 20.0;
/// Gain-reduction full scale of the gate slot (the standard gate's 60 dB range, protocol.md §4.2).
pub const GATE_GR_FULL_SCALE_DB: f32 = 60.0;

/// Gain reduction (dB, >= 0) to a gate/dyn "gain" meter word: `256 * gr / full_scale`, so 256 is
/// `full_scale` dB of reduction. [H] The sign convention (positive = more reduction) is not
/// confirmed on the wire.
pub fn gr_to_meter(gr_db: f32, full_scale_db: f32) -> i16 {
    if gr_db.is_nan() || full_scale_db <= 0.0 {
        return 0;
    }
    (gr_db.max(0.0) * 256.0 / full_scale_db)
        .round()
        .min(i16::MAX as f32) as i16
}

/// Inverse of [`gr_to_meter`].
pub fn meter_to_gr(v: i16, full_scale_db: f32) -> f32 {
    v as f32 * full_scale_db / 256.0
}

#[cfg(test)]
mod tests {

    #[test]
    fn describe_selections_ranges() {
        let sel = vec![
            MeterSelection {
                kind: MeterKind::ChannelV2,
                indices: (0..16).collect(),
            },
            MeterSelection {
                kind: MeterKind::Source,
                indices: vec![0, 3, 4, 7],
            },
            MeterSelection {
                kind: MeterKind::Fx,
                indices: vec![4],
            },
        ];
        assert_eq!(
            describe_selections(&sel),
            "ChannelV2 1-16, Source 1,4-5,8, Fx 5"
        );
    }
    use super::*;

    fn tokens(bytes: &[u8]) -> Vec<MeterToken> {
        let mut d = MeterTokenDecoder::new();
        d.extend(bytes);
        let mut v = Vec::new();
        while let Some(t) = d.next_token() {
            v.push(t.unwrap());
        }
        v
    }

    #[test]
    fn doc_selection_example() {
        // D4 2, DC A0 00 01 08 A6 04 DE: ch 1, 2, 9 and FX 5
        let toks = tokens(&[
            0xD4, 0, 0, 0, 2, 0xDC, 0xA0, 0x00, 0x01, 0x08, 0xA6, 0x04, 0xDE,
        ]);
        let mut p = MeterRequestParser::new();
        let cmds: Vec<_> = toks.into_iter().filter_map(|t| p.feed(t)).collect();
        let sel = vec![
            MeterSelection {
                kind: MeterKind::Channel,
                indices: vec![0, 1, 8],
            },
            MeterSelection {
                kind: MeterKind::Fx,
                indices: vec![4],
            },
        ];
        assert_eq!(
            cmds,
            vec![
                MeterCommand::Renew(2),
                MeterCommand::Define {
                    id: 2,
                    selections: sel.clone()
                }
            ]
        );
        let mut enc = Vec::new();
        encode_meter_define(2, &sel, &mut enc).unwrap();
        assert_eq!(
            enc,
            [0xD4, 0, 0, 0, 2, 0xDC, 0xA0, 0x00, 0x01, 0x08, 0xA6, 0x04, 0xDE]
        );
    }

    #[test]
    fn port_and_renew() {
        let toks = tokens(&[0xD3, 0x37, 0x37, 0xD4, 0, 0, 0, 2]);
        assert_eq!(toks, [MeterToken::Port(14135), MeterToken::ReportId(2)]);
    }

    #[test]
    fn packet_roundtrip_and_scale() {
        let mut out = Vec::new();
        encode_meter_packet(0x0001_3737, &[-3840, 0, 256], &mut out);
        assert_eq!(&out[..6], [0x00, 0x01, 0x37, 0x37, 0xF1, 0x00]);
        let p = MeterPacket::parse(&out).unwrap();
        assert_eq!(p.report_id, 0x0001_3737);
        assert_eq!(p.values().collect::<Vec<_>>(), [-3840, 0, 256]);
        let mut fixed = [0u8; 10];
        assert_eq!(
            encode_meter_packet_into(0x0001_3737, &[-3840, 0, 256], &mut fixed),
            Some(10)
        );
        assert_eq!(&fixed[..], &out[..]);
        assert_eq!(encode_meter_packet_into(1, &[1, 2, 3], &mut [0u8; 9]), None);
        assert_eq!(db_to_meter(-15.0), 0xF100u16 as i16);
        assert_eq!(meter_to_db(0xF100u16 as i16), -15.0);
        assert_eq!(db_to_meter(-1000.0), i16::MIN);
        assert!(MeterPacket::parse(&[0, 0, 0]).is_err());
        assert!(MeterPacket::parse(&[0, 0, 0, 0, 1]).is_err());
    }

    #[test]
    fn unknown_meter_byte() {
        let mut d = MeterTokenDecoder::new();
        d.extend(&[0x90, 0xDE]);
        assert_eq!(
            d.next_token(),
            Some(Err(DecodeError::UnknownMeterToken(0x90)))
        );
        assert_eq!(d.next_token(), Some(Ok(MeterToken::End)));
    }
}
