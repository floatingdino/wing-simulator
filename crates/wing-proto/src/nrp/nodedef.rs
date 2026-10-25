//! Node definition records (spec §3.3): the body of a `DF len ...` token.
//!
//! Derived from libwing (Apache-2.0, <https://github.com/dannydulai/libwing>, `src/node.rs`
//! `WingNodeDef::from_bytes`); see this crate's NOTICE. Adds encoding, bounds checking and
//! lossless preservation of unknown flag bits.
//!
//! ```text
//! parent.l hash.l index.w namelen.b name longnamelen.b longname flags.w <type-specific>
//! flags: bits0-3 unit, bits4-7 type, bit9 read-only
//! ```

use super::framing::Sink;
use super::DecodeError;

/// Physical unit (flags bits 0..=3).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Unit {
    None,
    Db,
    Percent,
    Ms,
    Hz,
    Meters,
    Seconds,
    Octaves,
    /// Undocumented unit code 8..=15, preserved as-is.
    Other(u8),
}

impl Unit {
    pub fn from_bits(b: u8) -> Unit {
        match b & 0x0F {
            0 => Unit::None,
            1 => Unit::Db,
            2 => Unit::Percent,
            3 => Unit::Ms,
            4 => Unit::Hz,
            5 => Unit::Meters,
            6 => Unit::Seconds,
            7 => Unit::Octaves,
            n => Unit::Other(n),
        }
    }

    pub fn bits(self) -> u8 {
        match self {
            Unit::None => 0,
            Unit::Db => 1,
            Unit::Percent => 2,
            Unit::Ms => 3,
            Unit::Hz => 4,
            Unit::Meters => 5,
            Unit::Seconds => 6,
            Unit::Octaves => 7,
            Unit::Other(n) => n & 0x0F,
        }
    }
}

/// A string-enum item: short name and long (display) name.
#[derive(Clone, Debug, PartialEq)]
pub struct EnumItem {
    pub name: String,
    pub long_name: String,
}

/// A float-enum item: value and long (display) name.
#[derive(Clone, Debug, PartialEq)]
pub struct FloatEnumItem {
    pub value: f32,
    pub long_name: String,
}

/// Node type (flags bits 4..=7) plus its type-specific payload.
#[derive(Clone, Debug, PartialEq)]
pub enum NodeKind {
    /// 0: an interior node.
    Node,
    /// 1: linear float.
    LinFloat { min: f32, max: f32, steps: i32 },
    /// 2: logarithmic float.
    LogFloat { min: f32, max: f32, steps: i32 },
    /// 3: fader level.
    FaderLevel,
    /// 4: integer.
    Int { min: i32, max: i32 },
    /// 5: string enum.
    StringEnum(Vec<EnumItem>),
    /// 6: float enum.
    FloatEnum(Vec<FloatEnumItem>),
    /// 7: free string.
    String { max_len: u16 },
}

impl NodeKind {
    pub fn type_code(&self) -> u8 {
        match self {
            NodeKind::Node => 0,
            NodeKind::LinFloat { .. } => 1,
            NodeKind::LogFloat { .. } => 2,
            NodeKind::FaderLevel => 3,
            NodeKind::Int { .. } => 4,
            NodeKind::StringEnum(_) => 5,
            NodeKind::FloatEnum(_) => 6,
            NodeKind::String { .. } => 7,
        }
    }
}

const FLAG_READ_ONLY: u16 = 1 << 9;
const FLAG_KNOWN: u16 = FLAG_READ_ONLY | 0x00FF;

/// A decoded node definition record.
#[derive(Clone, Debug, PartialEq)]
pub struct NodeDef {
    pub parent: u32,
    pub hash: u32,
    /// 1-based index for numbered children (empty name), 0 for named children.
    pub index: u16,
    pub name: String,
    pub long_name: String,
    pub unit: Unit,
    pub read_only: bool,
    /// Flag bits other than unit/type/read-only, preserved for byte-exact round trips.
    pub extra_flags: u16,
    pub kind: NodeKind,
}

struct Reader<'a> {
    b: &'a [u8],
    i: usize,
}

impl<'a> Reader<'a> {
    fn take(&mut self, n: usize) -> Result<&'a [u8], DecodeError> {
        if self.b.len() - self.i < n {
            return Err(DecodeError::Truncated);
        }
        let s = &self.b[self.i..self.i + n];
        self.i += n;
        Ok(s)
    }
    fn u8(&mut self) -> Result<u8, DecodeError> {
        Ok(self.take(1)?[0])
    }
    fn u16(&mut self) -> Result<u16, DecodeError> {
        let s = self.take(2)?;
        Ok(u16::from_be_bytes([s[0], s[1]]))
    }
    fn u32(&mut self) -> Result<u32, DecodeError> {
        let s = self.take(4)?;
        Ok(u32::from_be_bytes([s[0], s[1], s[2], s[3]]))
    }
    fn f32(&mut self) -> Result<f32, DecodeError> {
        Ok(f32::from_bits(self.u32()?))
    }
    fn str8(&mut self) -> Result<String, DecodeError> {
        let n = self.u8()? as usize;
        let s = self.take(n)?;
        String::from_utf8(s.to_vec()).map_err(|_| DecodeError::InvalidUtf8)
    }
}

impl NodeDef {
    /// Parse a record body (the bytes after the `DF len` header, already unescaped).
    pub fn parse(body: &[u8]) -> Result<NodeDef, DecodeError> {
        let mut r = Reader { b: body, i: 0 };
        let parent = r.u32()?;
        let hash = r.u32()?;
        let index = r.u16()?;
        let name = r.str8()?;
        let long_name = r.str8()?;
        let flags = r.u16()?;
        let kind = match (flags >> 4) & 0x0F {
            0 => NodeKind::Node,
            1 => NodeKind::LinFloat {
                min: r.f32()?,
                max: r.f32()?,
                steps: r.u32()? as i32,
            },
            2 => NodeKind::LogFloat {
                min: r.f32()?,
                max: r.f32()?,
                steps: r.u32()? as i32,
            },
            3 => NodeKind::FaderLevel,
            4 => NodeKind::Int {
                min: r.u32()? as i32,
                max: r.u32()? as i32,
            },
            5 => {
                let n = r.u16()?;
                let mut items = Vec::with_capacity(n as usize);
                for _ in 0..n {
                    items.push(EnumItem {
                        name: r.str8()?,
                        long_name: r.str8()?,
                    });
                }
                NodeKind::StringEnum(items)
            }
            6 => {
                let n = r.u16()?;
                let mut items = Vec::with_capacity(n as usize);
                for _ in 0..n {
                    items.push(FloatEnumItem {
                        value: r.f32()?,
                        long_name: r.str8()?,
                    });
                }
                NodeKind::FloatEnum(items)
            }
            7 => NodeKind::String { max_len: r.u16()? },
            t => return Err(DecodeError::UnknownNodeType(t as u8)),
        };
        if r.i != body.len() {
            return Err(DecodeError::TrailingBytes(body.len() - r.i));
        }
        Ok(NodeDef {
            parent,
            hash,
            index,
            name,
            long_name,
            unit: Unit::from_bits(flags as u8),
            read_only: flags & FLAG_READ_ONLY != 0,
            extra_flags: flags & !FLAG_KNOWN,
            kind,
        })
    }

    /// The flags word as it appears on the wire.
    pub fn flags(&self) -> u16 {
        (self.extra_flags & !FLAG_KNOWN)
            | if self.read_only { FLAG_READ_ONLY } else { 0 }
            | ((self.kind.type_code() as u16) << 4)
            | self.unit.bits() as u16
    }

    /// Write the record body (no `DF len` header). Names/items longer than 255 bytes are
    /// truncated; enum lists longer than 65535 items are truncated.
    pub fn encode_body(&self, s: &mut impl Sink) {
        fn str8(s: &mut impl Sink, v: &str) {
            let b = &v.as_bytes()[..v.len().min(255)];
            s.put(b.len() as u8);
            s.put_all(b);
        }
        s.put_all(&self.parent.to_be_bytes());
        s.put_all(&self.hash.to_be_bytes());
        s.put_all(&self.index.to_be_bytes());
        str8(s, &self.name);
        str8(s, &self.long_name);
        s.put_all(&self.flags().to_be_bytes());
        match &self.kind {
            NodeKind::Node | NodeKind::FaderLevel => {}
            NodeKind::LinFloat { min, max, steps } | NodeKind::LogFloat { min, max, steps } => {
                s.put_all(&min.to_be_bytes());
                s.put_all(&max.to_be_bytes());
                s.put_all(&steps.to_be_bytes());
            }
            NodeKind::Int { min, max } => {
                s.put_all(&min.to_be_bytes());
                s.put_all(&max.to_be_bytes());
            }
            NodeKind::StringEnum(items) => {
                let n = items.len().min(0xFFFF);
                s.put_all(&(n as u16).to_be_bytes());
                for it in &items[..n] {
                    str8(s, &it.name);
                    str8(s, &it.long_name);
                }
            }
            NodeKind::FloatEnum(items) => {
                let n = items.len().min(0xFFFF);
                s.put_all(&(n as u16).to_be_bytes());
                for it in &items[..n] {
                    s.put_all(&it.value.to_be_bytes());
                    str8(s, &it.long_name);
                }
            }
            NodeKind::String { max_len } => s.put_all(&max_len.to_be_bytes()),
        }
    }

    /// The record body as a new buffer.
    pub fn to_body(&self) -> Vec<u8> {
        let mut v = Vec::with_capacity(32);
        self.encode_body(&mut v);
        v
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn doc_example_root_child() {
        // df 0018 00000000 97a00439 0000 05 "$stat" 05 "STATE" 0000
        let body = [
            0x00, 0x00, 0x00, 0x00, 0x97, 0xA0, 0x04, 0x39, 0x00, 0x00, 0x05, b'$', b's', b't',
            b'a', b't', 0x05, b'S', b'T', b'A', b'T', b'E', 0x00, 0x00,
        ];
        let d = NodeDef::parse(&body).unwrap();
        assert_eq!(d.hash, 0x97A00439);
        assert_eq!(d.name, "$stat");
        assert_eq!(d.long_name, "STATE");
        assert_eq!(d.kind, NodeKind::Node);
        assert_eq!(d.to_body(), body);
    }

    #[test]
    fn truncated_and_trailing() {
        assert_eq!(NodeDef::parse(&[0; 5]), Err(DecodeError::Truncated));
        let mut body = NodeDef {
            parent: 1,
            hash: 2,
            index: 3,
            name: String::new(),
            long_name: String::new(),
            unit: Unit::None,
            read_only: false,
            extra_flags: 0,
            kind: NodeKind::Node,
        }
        .to_body();
        body.push(0);
        assert_eq!(NodeDef::parse(&body), Err(DecodeError::TrailingBytes(1)));
    }
}
