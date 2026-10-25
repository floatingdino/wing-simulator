//! Channel-1 token layer (spec §3.1).
//!
//! Derived from libwing (Apache-2.0, <https://github.com/dannydulai/libwing>, `src/console.rs`
//! `read`, `set_int`, `set_string`, `format_id`); see this crate's NOTICE. Rewritten as a
//! zero-copy parser plus an encoder for the server side.
//!
//! Tokens operate on a per-connection *current node* cursor: navigation tokens move it, value
//! tokens set (client → console) or report (console → client) its value.

use super::framing::Sink;
use super::queue::ByteQueue;
use super::{DecodeError, EncodeError};

/// Default cap on a node definition record length; larger records are rejected so a corrupt
/// length cannot make the decoder buffer unbounded data.
pub const DEFAULT_MAX_RECORD_LEN: u32 = 1 << 20;

/// One channel-1 token. String-like payloads borrow from the input.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Token<'a> {
    // ---- values ----
    /// `00..=3F`: integer 0..=63 (0 = false/off, 1 = true/on).
    SmallInt(u8),
    /// `D0`, `80..=BF`, `D1`: string value, 0..=256 bytes.
    Str(&'a [u8]),
    /// `D3 i16`.
    Int16(i16),
    /// `D4 i32`.
    Int32(i32),
    /// `D5 f32`: float in engineering units (dB, Hz, ms ...).
    Float(f32),
    /// `D6 f32`: raw float, normalised 0.0..=1.0.
    RawFloat(f32),
    // ---- navigation ----
    /// `40..=7F` (index 1..=64) or `D2 u16` (index 1..=65536, wire value is index-1 as in libwing).
    NavIndex(u32),
    /// `C0..=CF`: child by name, 1..=16 bytes.
    NavName(&'a [u8]),
    /// `D7 u32`: absolute select by node hash.
    NavHash(u32),
    /// `DA`: go to root.
    Root,
    /// `DB`: go up one level.
    Up,
    // ---- actions / requests ----
    /// `D8`: click (toggle) the current node.
    Click,
    /// `D9 i8`: step the current node by n.
    Step(i8),
    /// `DC`: data request for the current node.
    DataRequest,
    /// `DD`: node definition request (children of the current node).
    DefRequest,
    /// `DE`: end of a data/definition response.
    End,
    /// `DF len body`: a node definition record body (see [`super::nodedef::NodeDef`]).
    NodeDef(&'a [u8]),
}

impl Token<'_> {
    /// True for tokens that set/report the current node's value.
    pub fn is_value(&self) -> bool {
        matches!(
            self,
            Token::SmallInt(_)
                | Token::Str(_)
                | Token::Int16(_)
                | Token::Int32(_)
                | Token::Float(_)
                | Token::RawFloat(_)
        )
    }

    /// True for tokens that move the current-node cursor.
    pub fn is_navigation(&self) -> bool {
        matches!(
            self,
            Token::NavIndex(_) | Token::NavName(_) | Token::NavHash(_) | Token::Root | Token::Up
        )
    }

    /// The canonical compact integer token for `v`: `00..3F` for 0..=63, else `D3` if it fits
    /// in i16, else `D4` (libwing's `set_int` rule).
    pub fn int(v: i32) -> Token<'static> {
        if (0..=0x3F).contains(&v) {
            Token::SmallInt(v as u8)
        } else if let Ok(s) = i16::try_from(v) {
            Token::Int16(s)
        } else {
            Token::Int32(v)
        }
    }

    /// The integer value, if this is an integer token.
    pub fn as_int(&self) -> Option<i32> {
        match *self {
            Token::SmallInt(v) => Some(v as i32),
            Token::Int16(v) => Some(v as i32),
            Token::Int32(v) => Some(v),
            _ => None,
        }
    }

    /// Validate and write the token's (unescaped) payload bytes to `s`. Strings use the
    /// shortest form (`D0`, `80+n-1`, or `D1`). Nothing is written on error.
    pub fn encode(&self, s: &mut impl Sink) -> Result<(), EncodeError> {
        match *self {
            Token::SmallInt(v) => {
                if v > 0x3F {
                    return Err(EncodeError::SmallIntRange(v));
                }
                s.put(v);
            }
            Token::Str(b) => match b.len() {
                0 => s.put(0xD0),
                1..=64 => {
                    s.put(0x7F + b.len() as u8);
                    s.put_all(b);
                }
                65..=256 => {
                    s.put(0xD1);
                    s.put((b.len() - 1) as u8);
                    s.put_all(b);
                }
                n => return Err(EncodeError::StringTooLong(n)),
            },
            Token::Int16(v) => {
                s.put(0xD3);
                s.put_all(&v.to_be_bytes());
            }
            Token::Int32(v) => {
                s.put(0xD4);
                s.put_all(&v.to_be_bytes());
            }
            Token::Float(v) => {
                s.put(0xD5);
                s.put_all(&v.to_be_bytes());
            }
            Token::RawFloat(v) => {
                s.put(0xD6);
                s.put_all(&v.to_be_bytes());
            }
            Token::NavIndex(i) => match i {
                1..=64 => s.put(0x3F + i as u8),
                65..=65536 => {
                    s.put(0xD2);
                    s.put_all(&((i - 1) as u16).to_be_bytes());
                }
                _ => return Err(EncodeError::IndexRange(i)),
            },
            Token::NavName(n) => {
                if n.is_empty() || n.len() > 16 {
                    return Err(EncodeError::NameLength(n.len()));
                }
                s.put(0xBF + n.len() as u8);
                s.put_all(n);
            }
            Token::NavHash(h) => {
                s.put(0xD7);
                s.put_all(&h.to_be_bytes());
            }
            Token::Root => s.put(0xDA),
            Token::Up => s.put(0xDB),
            Token::Click => s.put(0xD8),
            Token::Step(n) => {
                s.put(0xD9);
                s.put(n as u8);
            }
            Token::DataRequest => s.put(0xDC),
            Token::DefRequest => s.put(0xDD),
            Token::End => s.put(0xDE),
            Token::NodeDef(body) => {
                let len = body.len();
                if len == 0 || len > u32::MAX as usize {
                    return Err(EncodeError::RecordLength(len));
                }
                s.put(0xDF);
                if len <= 0xFFFF {
                    s.put_all(&(len as u16).to_be_bytes());
                } else {
                    s.put_all(&[0, 0]);
                    s.put_all(&(len as u32).to_be_bytes());
                }
                s.put_all(body);
            }
        }
        Ok(())
    }
}

/// Parse one token from the start of `input` (unescaped channel-1 payload).
///
/// Returns `Ok(None)` if `input` holds only a prefix of a token, `Ok(Some((token, consumed)))`
/// otherwise. On `Err`, the offending lead byte should be skipped (the streaming decoder does).
pub fn parse_token(
    input: &[u8],
    max_record_len: u32,
) -> Result<Option<(Token<'_>, usize)>, DecodeError> {
    let Some(&t) = input.first() else {
        return Ok(None);
    };
    let rest = &input[1..];
    // Fixed-size payload helper.
    macro_rules! need {
        ($n:expr) => {
            if rest.len() < $n {
                return Ok(None);
            }
        };
    }
    let tok = match t {
        0x00..=0x3F => (Token::SmallInt(t), 1),
        0x40..=0x7F => (Token::NavIndex((t - 0x3F) as u32), 1),
        0x80..=0xBF => {
            let n = (t - 0x7F) as usize;
            need!(n);
            (Token::Str(&rest[..n]), 1 + n)
        }
        0xC0..=0xCF => {
            let n = (t - 0xBF) as usize;
            need!(n);
            (Token::NavName(&rest[..n]), 1 + n)
        }
        0xD0 => (Token::Str(&[]), 1),
        0xD1 => {
            need!(1);
            let n = rest[0] as usize + 1;
            need!(1 + n);
            (Token::Str(&rest[1..1 + n]), 2 + n)
        }
        0xD2 => {
            need!(2);
            let i = u16::from_be_bytes([rest[0], rest[1]]) as u32 + 1;
            (Token::NavIndex(i), 3)
        }
        0xD3 => {
            need!(2);
            (Token::Int16(i16::from_be_bytes([rest[0], rest[1]])), 3)
        }
        0xD4 => {
            need!(4);
            (Token::Int32(i32::from_be_bytes(be4(rest))), 5)
        }
        0xD5 => {
            need!(4);
            (Token::Float(f32::from_be_bytes(be4(rest))), 5)
        }
        0xD6 => {
            need!(4);
            (Token::RawFloat(f32::from_be_bytes(be4(rest))), 5)
        }
        0xD7 => {
            need!(4);
            (Token::NavHash(u32::from_be_bytes(be4(rest))), 5)
        }
        0xD8 => (Token::Click, 1),
        0xD9 => {
            need!(1);
            (Token::Step(rest[0] as i8), 2)
        }
        0xDA => (Token::Root, 1),
        0xDB => (Token::Up, 1),
        0xDC => (Token::DataRequest, 1),
        0xDD => (Token::DefRequest, 1),
        0xDE => (Token::End, 1),
        0xDF => {
            need!(2);
            let short = u16::from_be_bytes([rest[0], rest[1]]) as u32;
            let (len, hdr) = if short != 0 {
                (short, 3)
            } else {
                need!(6);
                (u32::from_be_bytes(be4(&rest[2..])), 7)
            };
            if len > max_record_len {
                return Err(DecodeError::RecordTooLong(len));
            }
            let len = len as usize;
            if input.len() < hdr + len {
                return Ok(None);
            }
            (Token::NodeDef(&input[hdr..hdr + len]), hdr + len)
        }
        0xE0..=0xFF => return Err(DecodeError::UnknownToken(t)),
    };
    Ok(Some(tok))
}

#[inline]
fn be4(b: &[u8]) -> [u8; 4] {
    [b[0], b[1], b[2], b[3]]
}

/// Streaming channel-1 token decoder. Push unescaped channel payload in any chunking, then drain
/// tokens with [`TokenDecoder::next_token`]. The internal buffer is reused (no per-token
/// allocation).
#[derive(Debug)]
pub struct TokenDecoder {
    q: ByteQueue,
    max_record_len: u32,
}

impl Default for TokenDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl TokenDecoder {
    pub fn new() -> Self {
        Self::with_max_record_len(DEFAULT_MAX_RECORD_LEN)
    }

    pub fn with_max_record_len(max_record_len: u32) -> Self {
        Self {
            q: ByteQueue::default(),
            max_record_len,
        }
    }

    /// Append one payload byte.
    #[inline]
    pub fn push(&mut self, b: u8) {
        self.q.push(b);
    }

    /// Append payload bytes.
    pub fn extend(&mut self, bytes: &[u8]) {
        self.q.extend(bytes);
    }

    /// Bytes buffered but not yet consumed (an incomplete trailing token).
    pub fn pending(&self) -> usize {
        self.q.pending().len()
    }

    /// Next complete token, or `None` if more input is needed. After an `Err` the bad byte has
    /// been skipped and decoding can continue.
    pub fn next_token(&mut self) -> Option<Result<Token<'_>, DecodeError>> {
        self.q.compact();
        let input = &self.q.buf[self.q.pos..];
        match parse_token(input, self.max_record_len) {
            Ok(None) => None,
            Ok(Some((tok, n))) => {
                self.q.pos += n;
                Some(Ok(tok))
            }
            Err(e) => {
                self.q.pos += 1;
                Some(Err(e))
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn enc(t: Token) -> Vec<u8> {
        let mut v = Vec::new();
        t.encode(&mut v).unwrap();
        v
    }

    #[test]
    fn int_forms() {
        assert_eq!(enc(Token::int(0)), [0x00]);
        assert_eq!(enc(Token::int(63)), [0x3F]);
        assert_eq!(enc(Token::int(64)), [0xD3, 0x00, 0x40]);
        assert_eq!(enc(Token::int(-1)), [0xD3, 0xFF, 0xFF]);
        assert_eq!(enc(Token::int(70000)), [0xD4, 0x00, 0x01, 0x11, 0x70]);
    }

    #[test]
    fn string_forms() {
        assert_eq!(enc(Token::Str(b"")), [0xD0]);
        assert_eq!(enc(Token::Str(b"ab")), [0x81, b'a', b'b']);
        let s64 = [b'x'; 64];
        assert_eq!(enc(Token::Str(&s64))[0], 0xBF);
        let s65 = [b'x'; 65];
        assert_eq!(&enc(Token::Str(&s65))[..2], [0xD1, 64]);
        let s256 = [b'x'; 256];
        assert_eq!(&enc(Token::Str(&s256))[..2], [0xD1, 255]);
        assert!(Token::Str(&[0u8; 257]).encode(&mut Vec::new()).is_err());
        // D1 with a short length decodes too (non-canonical form).
        let (t, n) = parse_token(&[0xD1, 1, b'h', b'i'], 1024).unwrap().unwrap();
        assert_eq!((t, n), (Token::Str(b"hi"), 4));
    }

    #[test]
    fn nav_index_forms() {
        assert_eq!(enc(Token::NavIndex(1)), [0x40]);
        assert_eq!(enc(Token::NavIndex(64)), [0x7F]);
        assert_eq!(enc(Token::NavIndex(65)), [0xD2, 0x00, 0x40]);
        assert_eq!(enc(Token::NavIndex(65536)), [0xD2, 0xFF, 0xFF]);
        assert!(Token::NavIndex(0).encode(&mut Vec::new()).is_err());
        assert!(Token::NavName(b"").encode(&mut Vec::new()).is_err());
        assert!(Token::NavName(&[b'a'; 17]).encode(&mut Vec::new()).is_err());
        assert!(Token::SmallInt(64).encode(&mut Vec::new()).is_err());
    }

    #[test]
    fn incomplete_and_unknown() {
        assert_eq!(parse_token(&[0xD7, 1, 2, 3], 1024).unwrap(), None);
        assert_eq!(parse_token(&[0x83, b'a'], 1024).unwrap(), None);
        assert_eq!(parse_token(&[0xDF, 0x00], 1024).unwrap(), None);
        assert_eq!(parse_token(&[0xDF, 0x00, 0x00, 0, 0], 1024).unwrap(), None);
        assert_eq!(
            parse_token(&[0xE5], 1024),
            Err(DecodeError::UnknownToken(0xE5))
        );
        assert_eq!(
            parse_token(&[0xDF, 0x08, 0x00], 1024),
            Err(DecodeError::RecordTooLong(0x800))
        );
    }

    #[test]
    fn long_record_length_form() {
        let body = vec![7u8; 70000];
        let wire = enc(Token::NodeDef(&body));
        assert_eq!(&wire[..7], [0xDF, 0, 0, 0x00, 0x01, 0x11, 0x70]);
        let (t, n) = parse_token(&wire, DEFAULT_MAX_RECORD_LEN).unwrap().unwrap();
        assert_eq!(n, wire.len());
        assert_eq!(t, Token::NodeDef(&body));
    }

    #[test]
    fn decoder_skips_bad_byte() {
        let mut d = TokenDecoder::new();
        d.extend(&[0xE0, 0x01, 0xD7, 0xF5]);
        assert_eq!(d.next_token(), Some(Err(DecodeError::UnknownToken(0xE0))));
        assert_eq!(d.next_token(), Some(Ok(Token::SmallInt(1))));
        assert_eq!(d.next_token(), None);
        d.extend(&[0x0F, 0x69, 0xF8]);
        assert_eq!(d.next_token(), Some(Ok(Token::NavHash(0xF50F69F8))));
        assert_eq!(d.pending(), 0);
    }
}
