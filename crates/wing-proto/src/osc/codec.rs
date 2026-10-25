//! OSC 1.0 message codec. Types `i` (i32), `f` (f32), `s` (string), `b` (blob); big-endian, 4-byte
//! padded. Bundles are rejected (WING does not use them). A missing type-tag string means "no
//! arguments".

use std::fmt;

/// Largest datagram WING accepts or sends.
pub const MAX_PACKET: usize = 32 * 1024;

#[derive(Clone, Debug, PartialEq)]
pub enum Arg {
    Int(i32),
    Float(f32),
    Str(String),
    Blob(Vec<u8>),
}

#[derive(Clone, Debug, PartialEq)]
pub struct Message {
    pub addr: String,
    pub args: Vec<Arg>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum DecodeError {
    Empty,
    Bundle,
    BadAddress,
    Truncated,
    BadString,
    BadTypeTag(char),
}

impl fmt::Display for DecodeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            DecodeError::Empty => f.write_str("empty packet"),
            DecodeError::Bundle => f.write_str("OSC bundles are not supported"),
            DecodeError::BadAddress => f.write_str("address does not start with '/'"),
            DecodeError::Truncated => f.write_str("truncated packet"),
            DecodeError::BadString => f.write_str("unterminated or non-UTF-8 string"),
            DecodeError::BadTypeTag(c) => write!(f, "unsupported type tag '{c}'"),
        }
    }
}

impl std::error::Error for DecodeError {}

impl Message {
    pub fn new(addr: impl Into<String>, args: Vec<Arg>) -> Self {
        Message {
            addr: addr.into(),
            args,
        }
    }

    /// Decode one message datagram.
    pub fn decode(buf: &[u8]) -> Result<Message, DecodeError> {
        if buf.is_empty() {
            return Err(DecodeError::Empty);
        }
        if buf.starts_with(b"#bundle") {
            return Err(DecodeError::Bundle);
        }
        let mut pos = 0;
        let addr = read_str(buf, &mut pos)?;
        if !addr.starts_with('/') {
            return Err(DecodeError::BadAddress);
        }
        let mut args = Vec::new();
        if pos >= buf.len() {
            return Ok(Message { addr, args });
        }
        let tags = read_str(buf, &mut pos)?;
        let Some(tags) = tags.strip_prefix(',') else {
            // tag string without a comma: treat as absent args (old-style OSC)
            return Ok(Message { addr, args });
        };
        for t in tags.chars() {
            args.push(match t {
                'i' => Arg::Int(i32::from_be_bytes(
                    take(buf, &mut pos, 4)?.try_into().unwrap(),
                )),
                'f' => Arg::Float(f32::from_be_bytes(
                    take(buf, &mut pos, 4)?.try_into().unwrap(),
                )),
                's' => Arg::Str(read_str(buf, &mut pos)?),
                'b' => {
                    let n =
                        u32::from_be_bytes(take(buf, &mut pos, 4)?.try_into().unwrap()) as usize;
                    let data = take(buf, &mut pos, n)?.to_vec();
                    // padding may be absent on the final blob; be permissive
                    pos = ((pos + 3) & !3).min(buf.len());
                    Arg::Blob(data)
                }
                c => return Err(DecodeError::BadTypeTag(c)),
            });
        }
        Ok(Message { addr, args })
    }

    /// Encode, appending to `out`.
    pub fn encode(&self, out: &mut Vec<u8>) {
        write_str(out, &self.addr);
        let mut tags = String::from(",");
        for a in &self.args {
            tags.push(match a {
                Arg::Int(_) => 'i',
                Arg::Float(_) => 'f',
                Arg::Str(_) => 's',
                Arg::Blob(_) => 'b',
            });
        }
        write_str(out, &tags);
        for a in &self.args {
            match a {
                Arg::Int(i) => out.extend_from_slice(&i.to_be_bytes()),
                Arg::Float(f) => out.extend_from_slice(&f.to_be_bytes()),
                Arg::Str(s) => write_str(out, s),
                Arg::Blob(b) => {
                    out.extend_from_slice(&(b.len() as u32).to_be_bytes());
                    out.extend_from_slice(b);
                    pad(out);
                }
            }
        }
    }

    pub fn to_vec(&self) -> Vec<u8> {
        let mut v = Vec::new();
        self.encode(&mut v);
        v
    }
}

fn pad(out: &mut Vec<u8>) {
    while !out.len().is_multiple_of(4) {
        out.push(0);
    }
}

fn write_str(out: &mut Vec<u8>, s: &str) {
    out.extend_from_slice(s.as_bytes());
    out.push(0);
    pad(out);
}

fn take<'a>(buf: &'a [u8], pos: &mut usize, n: usize) -> Result<&'a [u8], DecodeError> {
    let end = pos.checked_add(n).ok_or(DecodeError::Truncated)?;
    let s = buf.get(*pos..end).ok_or(DecodeError::Truncated)?;
    *pos = end;
    Ok(s)
}

fn read_str(buf: &[u8], pos: &mut usize) -> Result<String, DecodeError> {
    let rest = buf.get(*pos..).ok_or(DecodeError::Truncated)?;
    let n = rest
        .iter()
        .position(|&b| b == 0)
        .ok_or(DecodeError::BadString)?;
    let s = std::str::from_utf8(&rest[..n]).map_err(|_| DecodeError::BadString)?;
    *pos = ((*pos + n + 1 + 3) & !3).min(buf.len());
    Ok(s.to_owned())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip_all_types() {
        let m = Message::new(
            "/ch/1/fdr",
            vec![
                Arg::Str("-oo".into()),
                Arg::Float(0.5),
                Arg::Int(-7),
                Arg::Blob(vec![1, 2, 3, 4, 5]),
                Arg::Str(String::new()),
            ],
        );
        let b = m.to_vec();
        assert_eq!(b.len() % 4, 0);
        assert_eq!(Message::decode(&b).unwrap(), m);
    }

    #[test]
    fn known_bytes() {
        let m = Message::new("/ch/1/mute", vec![Arg::Int(1)]);
        let mut want = b"/ch/1/mute\0\0".to_vec();
        want.extend_from_slice(b",i\0\0");
        want.extend_from_slice(&[0, 0, 0, 1]);
        assert_eq!(m.to_vec(), want);
    }

    #[test]
    fn no_type_tags_means_no_args() {
        assert_eq!(
            Message::decode(b"/ch/1/fdr\0\0\0").unwrap(),
            Message::new("/ch/1/fdr", vec![])
        );
        assert_eq!(Message::decode(b"/ch/1/fdr\0").unwrap().addr, "/ch/1/fdr");
    }

    #[test]
    fn rejects_bad_input() {
        assert_eq!(Message::decode(b""), Err(DecodeError::Empty));
        assert_eq!(Message::decode(b"#bundle\0"), Err(DecodeError::Bundle));
        assert_eq!(Message::decode(b"ch\0\0"), Err(DecodeError::BadAddress));
        assert_eq!(
            Message::decode(b"/a\0\0,i\0\0\0\0"),
            Err(DecodeError::Truncated)
        );
        assert_eq!(
            Message::decode(b"/a\0\0,x\0\0"),
            Err(DecodeError::BadTypeTag('x'))
        );
        assert_eq!(Message::decode(b"/abc"), Err(DecodeError::BadString));
    }

    #[test]
    fn blob_without_final_padding() {
        let mut b = b"/a\0\0,b\0\0".to_vec();
        b.extend_from_slice(&[0, 0, 0, 2, 9, 8]);
        assert_eq!(
            Message::decode(&b).unwrap().args,
            vec![Arg::Blob(vec![9, 8])]
        );
    }
}
