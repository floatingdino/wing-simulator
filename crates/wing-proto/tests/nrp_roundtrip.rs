//! Property-style tests (hand-rolled PRNG to keep dependencies minimal):
//! - encode → decode round trip for every token type, node definitions and meter tokens;
//! - a mixed multi-channel stream decodes identically under random chunking;
//! - random garbage never panics and decodes identically under random chunking.

use wing_proto::nrp::meter::{encode_meter_packet, MeterPacket};
use wing_proto::nrp::nodedef::{EnumItem, FloatEnumItem};
use wing_proto::nrp::*;

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        // xorshift64*
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        self.0.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
    fn below(&mut self, n: u64) -> u64 {
        self.next() % n
    }
    fn byte(&mut self) -> u8 {
        // Bias toward the interesting escape range D0..DF.
        if self.below(4) == 0 {
            0xD0 + self.below(16) as u8
        } else {
            self.next() as u8
        }
    }
    fn bytes(&mut self, n: usize) -> Vec<u8> {
        (0..n).map(|_| self.byte()).collect()
    }
    fn f32(&mut self) -> f32 {
        // Any non-NaN bit pattern (NaN != NaN would break equality checks).
        loop {
            let f = f32::from_bits(self.next() as u32);
            if !f.is_nan() {
                return f;
            }
        }
    }
    fn ascii(&mut self, max: usize) -> String {
        let n = self.below(max as u64 + 1) as usize;
        (0..n)
            .map(|_| (b' ' + self.below(95) as u8) as char)
            .collect()
    }
}

/// Owned mirror of `Token` so generated tokens can be stored.
#[derive(Clone, Debug, PartialEq)]
enum Owned {
    SmallInt(u8),
    Str(Vec<u8>),
    Int16(i16),
    Int32(i32),
    Float(f32),
    RawFloat(f32),
    NavIndex(u32),
    NavName(Vec<u8>),
    NavHash(u32),
    Root,
    Up,
    Click,
    Step(i8),
    DataRequest,
    DefRequest,
    End,
    NodeDef(Vec<u8>),
}

impl Owned {
    fn as_token(&self) -> Token<'_> {
        match self {
            Owned::SmallInt(v) => Token::SmallInt(*v),
            Owned::Str(b) => Token::Str(b),
            Owned::Int16(v) => Token::Int16(*v),
            Owned::Int32(v) => Token::Int32(*v),
            Owned::Float(v) => Token::Float(*v),
            Owned::RawFloat(v) => Token::RawFloat(*v),
            Owned::NavIndex(v) => Token::NavIndex(*v),
            Owned::NavName(b) => Token::NavName(b),
            Owned::NavHash(v) => Token::NavHash(*v),
            Owned::Root => Token::Root,
            Owned::Up => Token::Up,
            Owned::Click => Token::Click,
            Owned::Step(v) => Token::Step(*v),
            Owned::DataRequest => Token::DataRequest,
            Owned::DefRequest => Token::DefRequest,
            Owned::End => Token::End,
            Owned::NodeDef(b) => Token::NodeDef(b),
        }
    }
    fn from_token(t: Token) -> Owned {
        match t {
            Token::SmallInt(v) => Owned::SmallInt(v),
            Token::Str(b) => Owned::Str(b.to_vec()),
            Token::Int16(v) => Owned::Int16(v),
            Token::Int32(v) => Owned::Int32(v),
            Token::Float(v) => Owned::Float(v),
            Token::RawFloat(v) => Owned::RawFloat(v),
            Token::NavIndex(v) => Owned::NavIndex(v),
            Token::NavName(b) => Owned::NavName(b.to_vec()),
            Token::NavHash(v) => Owned::NavHash(v),
            Token::Root => Owned::Root,
            Token::Up => Owned::Up,
            Token::Click => Owned::Click,
            Token::Step(v) => Owned::Step(v),
            Token::DataRequest => Owned::DataRequest,
            Token::DefRequest => Owned::DefRequest,
            Token::End => Owned::End,
            Token::NodeDef(b) => Owned::NodeDef(b.to_vec()),
        }
    }
}

fn gen_def(r: &mut Rng) -> NodeDef {
    let kind = match r.below(8) {
        0 => NodeKind::Node,
        1 => NodeKind::LinFloat {
            min: r.f32(),
            max: r.f32(),
            steps: r.next() as i32,
        },
        2 => NodeKind::LogFloat {
            min: r.f32(),
            max: r.f32(),
            steps: r.next() as i32,
        },
        3 => NodeKind::FaderLevel,
        4 => NodeKind::Int {
            min: r.next() as i32,
            max: r.next() as i32,
        },
        5 => NodeKind::StringEnum(
            (0..r.below(6))
                .map(|_| EnumItem {
                    name: r.ascii(12),
                    long_name: r.ascii(20),
                })
                .collect(),
        ),
        6 => NodeKind::FloatEnum(
            (0..r.below(6))
                .map(|_| FloatEnumItem {
                    value: r.f32(),
                    long_name: r.ascii(20),
                })
                .collect(),
        ),
        _ => NodeKind::String {
            max_len: r.next() as u16,
        },
    };
    NodeDef {
        parent: r.next() as u32,
        hash: r.next() as u32,
        index: r.next() as u16,
        name: r.ascii(16),
        long_name: r.ascii(32),
        unit: Unit::from_bits(r.below(16) as u8),
        read_only: r.below(2) == 0,
        extra_flags: (r.next() as u16) & 0xFD00,
        kind,
    }
}

fn gen_token(r: &mut Rng) -> Owned {
    match r.below(17) {
        0 => Owned::SmallInt(r.below(64) as u8),
        1 => {
            let n = match r.below(4) {
                0 => 0,
                1 => 1 + r.below(64),
                2 => 65 + r.below(192),
                _ => [1, 64, 65, 256][r.below(4) as usize],
            } as usize;
            Owned::Str(r.bytes(n))
        }
        2 => Owned::Int16(r.next() as i16),
        3 => Owned::Int32(r.next() as i32),
        4 => Owned::Float(r.f32()),
        5 => Owned::RawFloat(r.f32()),
        6 => Owned::NavIndex(1 + r.below(65536) as u32),
        7 => {
            let n = 1 + r.below(16) as usize;
            Owned::NavName(r.bytes(n))
        }
        8 => Owned::NavHash(r.next() as u32),
        9 => Owned::Root,
        10 => Owned::Up,
        11 => Owned::Click,
        12 => Owned::Step(r.next() as i8),
        13 => Owned::DataRequest,
        14 => Owned::DefRequest,
        15 => Owned::End,
        _ => Owned::NodeDef(gen_def(r).to_body()),
    }
}

fn gen_meter(r: &mut Rng) -> MeterToken {
    match r.below(6) {
        0 => MeterToken::Port(r.next() as u16),
        1 => MeterToken::ReportId(r.next() as u32),
        2 => MeterToken::Begin,
        3 => MeterToken::Kind(MeterKind::from_token(0xA0 + r.below(16) as u8).unwrap()),
        4 => MeterToken::Index(r.below(128) as u8),
        _ => MeterToken::End,
    }
}

#[test]
fn token_roundtrip_every_type() {
    let mut r = Rng(0x1234_5678_9ABC_DEF1);
    let mut seen = [false; 17];
    for _ in 0..20_000 {
        let o = gen_token(&mut r);
        let mut raw = Vec::new();
        o.as_token().encode(&mut raw).unwrap();
        let (t, n) = token::parse_token(&raw, token::DEFAULT_MAX_RECORD_LEN)
            .unwrap()
            .unwrap();
        assert_eq!(n, raw.len(), "{o:?}");
        let back = Owned::from_token(t);
        assert_eq!(back, o);
        // Every proper prefix is reported incomplete, never an error.
        for k in 0..raw.len() {
            assert_eq!(
                token::parse_token(&raw[..k], token::DEFAULT_MAX_RECORD_LEN),
                Ok(None)
            );
        }
        seen[kind_index(&o)] = true;
    }
    assert!(seen.iter().all(|&s| s), "not every token type generated");
}

fn kind_index(o: &Owned) -> usize {
    use Owned::*;
    match o {
        SmallInt(_) => 0,
        Str(_) => 1,
        Int16(_) => 2,
        Int32(_) => 3,
        Float(_) => 4,
        RawFloat(_) => 5,
        NavIndex(_) => 6,
        NavName(_) => 7,
        NavHash(_) => 8,
        Root => 9,
        Up => 10,
        Click => 11,
        Step(_) => 12,
        DataRequest => 13,
        DefRequest => 14,
        End => 15,
        NodeDef(_) => 16,
    }
}

#[test]
fn nodedef_roundtrip() {
    let mut r = Rng(0xDEAD_BEEF_0BAD_F00D);
    for _ in 0..5_000 {
        let d = gen_def(&mut r);
        let body = d.to_body();
        let back = NodeDef::parse(&body).unwrap();
        // Compare via re-encoding (float fields may hold -0.0 vs 0.0 bit patterns).
        assert_eq!(back.to_body(), body);
        assert_eq!(back.hash, d.hash);
        assert_eq!(back.name, d.name);
        assert_eq!(back.kind.type_code(), d.kind.type_code());
    }
}

#[test]
fn meter_token_and_packet_roundtrip() {
    let mut r = Rng(42);
    for _ in 0..5_000 {
        let t = gen_meter(&mut r);
        let mut raw = Vec::new();
        t.encode(&mut raw).unwrap();
        let (back, n) = meter::parse_meter_token(&raw).unwrap().unwrap();
        assert_eq!((back, n), (t, raw.len()));
    }
    for _ in 0..1_000 {
        let id = r.next() as u32;
        let vals: Vec<i16> = (0..r.below(200)).map(|_| r.next() as i16).collect();
        let mut out = Vec::new();
        encode_meter_packet(id, &vals, &mut out);
        let p = MeterPacket::parse(&out).unwrap();
        assert_eq!(p.report_id, id);
        assert_eq!(p.values().collect::<Vec<_>>(), vals);
    }
}

#[derive(Debug, PartialEq)]
enum Ev {
    C(Result<Owned, DecodeError>),
    M(Result<MeterToken, DecodeError>),
}

/// Feed `wire` in chunks of the given sizes, draining after each chunk.
fn decode_chunked(wire: &[u8], r: Option<&mut Rng>) -> Vec<Ev> {
    let mut d = NrpDecoder::new();
    let mut c = Vec::new();
    let mut m = Vec::new();
    let mut drain = |d: &mut NrpDecoder| {
        while let Some(t) = d.next_control() {
            c.push(Ev::C(t.map(Owned::from_token)));
        }
        while let Some(t) = d.next_meter() {
            m.push(Ev::M(t));
        }
    };
    match r {
        None => {
            d.feed(wire);
            drain(&mut d);
        }
        Some(r) => {
            let mut i = 0;
            while i < wire.len() {
                let n = match r.below(4) {
                    0 => 1,
                    1 => 1 + r.below(4) as usize,
                    _ => 1 + r.below(300) as usize,
                }
                .min(wire.len() - i);
                d.feed(&wire[i..i + n]);
                drain(&mut d);
                i += n;
            }
        }
    }
    c.extend(m);
    c
}

#[test]
fn random_chunking_decodes_identically() {
    let mut r = Rng(0x0F0F_1234_ABCD_0001);
    for round in 0..50 {
        // Build an interleaved stream of control tokens and meter tokens.
        let mut e = NrpEncoder::new();
        let mut wire = Vec::new();
        let mut want_c = Vec::new();
        let mut want_m = Vec::new();
        e.select(CH_CONTROL, &mut wire); // client keep-alive
        for _ in 0..400 {
            if r.below(4) == 0 {
                let t = gen_meter(&mut r);
                e.meter(&t, &mut wire).unwrap();
                want_m.push(Ev::M(Ok(t)));
            } else {
                let o = gen_token(&mut r);
                e.control(&o.as_token(), &mut wire).unwrap();
                want_c.push(Ev::C(Ok(o)));
            }
            if r.below(50) == 0 {
                e.select(CH_CONTROL, &mut wire); // stray keep-alive mid-stream
            }
        }
        want_c.extend(want_m);
        let whole = decode_chunked(&wire, None);
        assert_eq!(whole, want_c, "round {round}: whole-buffer decode");
        for _ in 0..5 {
            assert_eq!(decode_chunked(&wire, Some(&mut r)), whole, "round {round}");
        }
    }
}

#[test]
fn garbage_never_panics_and_chunking_is_stable() {
    let mut r = Rng(7);
    for _ in 0..200 {
        let n = r.below(2000) as usize;
        let wire = r.bytes(n);
        let whole = decode_chunked(&wire, None);
        assert_eq!(decode_chunked(&wire, Some(&mut r)), whole);
        // Node definition bodies from garbage must not panic either.
        for ev in &whole {
            if let Ev::C(Ok(Owned::NodeDef(b))) = ev {
                let _ = NodeDef::parse(b);
            }
        }
    }
}
