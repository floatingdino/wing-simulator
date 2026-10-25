//! Decoder tests against byte sequences from the WING Remote Protocols doc and libwing.
//!
//! The raw node definition records below are copied from libwing 1.0.5 `src/propmap.rs`
//! (Apache-2.0, <https://github.com/dannydulai/libwing>; captured from a Wing Compact on fw 3.0.5).
//! See this crate's NOTICE. Only a handful are included as test vectors.

use wing_proto::nrp::nodedef::{EnumItem, FloatEnumItem};
use wing_proto::nrp::*;

fn hex(s: &str) -> Vec<u8> {
    let s: String = s.chars().filter(|c| !c.is_whitespace()).collect();
    (0..s.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap())
        .collect()
}

fn control_tokens(wire: &[u8]) -> Vec<String> {
    let mut d = NrpDecoder::new();
    d.feed(wire);
    let mut v = Vec::new();
    while let Some(t) = d.next_control() {
        v.push(format!("{:?}", t.unwrap()));
    }
    v
}

fn s(v: &[Token]) -> Vec<String> {
    v.iter().map(|t| format!("{t:?}")).collect()
}

/// (path, record body hex) from libwing propmap.
const LIBWING_RECORDS: &[(&str, &str)] = &[
    ("/ch", "0000000070b101390000026368074348414e4e454c0000"),
    ("/ch/1", "70b10139878392cf00010001310000"),
    ("/ch/1/fdr", "878392cf38ae75c20000036664720546414445520031"),
    ("/ch/1/mute", "878392cff50f69f80000046d757465044d55544500400000000000000001"),
    ("/ch/1/name", "878392cfc6ad548d0000046e616d65044e414d4500700010"),
    ("/ch/1/flt/lcf", "ea0b9382401e6cf60000036c6366074c432046524551002441a0000044fa000100000281"),
    ("/$ctl/daw/$on", "df17c242beefaeab000003246f6e06444157204f4e02400000000000000001"),
    ("/$stat/A/stat", "cf5072e22105bba0000004737461740653544154555302500004012d00024f4b0003455252000355504400"),
    ("/cfg/mon/1/pan", "e3744a3c9cd2795500000370616e0350414e0010c2c8000042c80000000000c9"),
    ("/cfg/mon/1/eq/5q", "d1ff5641ff5368df00000235710351203500203ee24dd341200000000000b5"),
    ("/cfg/rta/rtarange", "b7f091988dcdc4d500000872746172616e67650552414e47450061000241f0000005333020644242700000053630206442"),
];

#[test]
fn libwing_records_roundtrip_byte_exact() {
    for (path, h) in LIBWING_RECORDS {
        let body = hex(h);
        let def = NodeDef::parse(&body).unwrap_or_else(|e| panic!("{path}: {e}"));
        assert_eq!(def.to_body(), body, "{path} re-encode");
        assert_eq!(def.flags().to_be_bytes(), {
            let off = 10 + 1 + body[10] as usize;
            let off = off + 1 + body[off] as usize;
            [body[off], body[off + 1]]
        });
    }
}

#[test]
fn libwing_record_fields() {
    let get = |p: &str| {
        let h = LIBWING_RECORDS.iter().find(|r| r.0 == p).unwrap().1;
        NodeDef::parse(&hex(h)).unwrap()
    };
    let ch = get("/ch");
    assert_eq!((ch.parent, ch.hash, ch.index), (0, 0x70B10139, 0));
    assert_eq!((ch.name.as_str(), ch.long_name.as_str()), ("ch", "CHANNEL"));

    let ch1 = get("/ch/1");
    assert_eq!(
        (ch1.parent, ch1.hash, ch1.index),
        (0x70B10139, 0x878392CF, 1)
    );
    assert_eq!((ch1.name.as_str(), ch1.long_name.as_str()), ("", "1"));
    assert_eq!(ch1.kind, NodeKind::Node);

    let fdr = get("/ch/1/fdr");
    assert_eq!(fdr.hash, 0x38AE75C2);
    assert_eq!(fdr.kind, NodeKind::FaderLevel);
    assert_eq!(fdr.unit, Unit::Db);

    let mute = get("/ch/1/mute");
    assert_eq!(mute.hash, 0xF50F69F8);
    assert_eq!(mute.kind, NodeKind::Int { min: 0, max: 1 });
    assert!(!mute.read_only);

    assert_eq!(get("/ch/1/name").kind, NodeKind::String { max_len: 16 });

    let lcf = get("/ch/1/flt/lcf");
    assert_eq!(lcf.unit, Unit::Hz);
    match lcf.kind {
        NodeKind::LogFloat { min, max, steps } => {
            assert_eq!(min, 20.0);
            assert!((max - 2000.0).abs() < 0.01);
            assert_eq!(steps, 641);
        }
        k => panic!("{k:?}"),
    }

    let on = get("/$ctl/daw/$on");
    assert_eq!((on.parent, on.hash), (0xDF17C242, 0xBEEFAEAB));
    assert!(on.read_only);
    assert_eq!(on.kind, NodeKind::Int { min: 0, max: 1 });

    let stat = get("/$stat/A/stat");
    assert!(stat.read_only);
    assert_eq!(
        stat.kind,
        NodeKind::StringEnum(
            ["-", "OK", "ERR", "UPD"]
                .iter()
                .map(|n| EnumItem {
                    name: n.to_string(),
                    long_name: String::new()
                })
                .collect()
        )
    );

    let pan = get("/cfg/mon/1/pan");
    assert_eq!(
        pan.kind,
        NodeKind::LinFloat {
            min: -100.0,
            max: 100.0,
            steps: 201
        }
    );

    let rr = get("/cfg/rta/rtarange");
    assert_eq!(rr.unit, Unit::Db);
    assert_eq!(
        rr.kind,
        NodeKind::FloatEnum(vec![
            FloatEnumItem {
                value: 30.0,
                long_name: "30 dB".into()
            },
            FloatEnumItem {
                value: 60.0,
                long_name: "60 dB".into()
            },
        ])
    );
}

#[test]
fn doc_def_example_with_end() {
    // Doc example 2 (/$ctl/daw/$on, as dumped in an OSC blob, i.e. unescaped):
    let payload =
        hex("df 001f df17c242 beefaeab 0000 03 246f6e 06 444157204f4e 0040 00000000 00000001 de");
    // Put it on the wire on channel 1 (escaping every DF).
    let mut wire = Vec::new();
    let mut m = Mux::new();
    m.write(CH_CONTROL, &payload, &mut wire);
    assert_eq!(&wire[..5], [0xDF, 0xD1, 0xDF, 0xDE, 0x00]);
    let mut d = NrpDecoder::new();
    d.feed(&wire);
    let body = match d.next_control().unwrap().unwrap() {
        Token::NodeDef(b) => b.to_vec(),
        t => panic!("{t:?}"),
    };
    assert_eq!(d.next_control(), Some(Ok(Token::End)));
    let def = NodeDef::parse(&body).unwrap();
    assert_eq!(def.name, "$on");
    assert_eq!(def.long_name, "DAW ON");
    assert_eq!(def.kind, NodeKind::Int { min: 0, max: 1 });

    // And the encoder reproduces the escaped wire bytes.
    let mut e = NrpEncoder::new();
    let mut out = Vec::new();
    e.node_def(&def, &mut out).unwrap();
    e.control(&Token::End, &mut out).unwrap();
    assert_eq!(out, wire);
}

#[test]
fn doc_worked_examples() {
    // get a leaf
    assert_eq!(
        control_tokens(&hex("df d1 d7 be ef ae ab dc")),
        s(&[Token::NavHash(0xBEEFAEAB), Token::DataRequest])
    );
    // answer
    assert_eq!(
        control_tokens(&hex("df d1 d7 be ef ae ab 01 de")),
        s(&[Token::NavHash(0xBEEFAEAB), Token::SmallInt(1), Token::End])
    );
    // set fader -3 dB (D5) and raw 0.675 (D6)
    assert_eq!(
        control_tokens(&hex(
            "df d1 d7 38 ae 75 c2 d5 c0 40 00 00 d7 38 ae 75 c2 d6 3f 2c cc cd"
        )),
        s(&[
            Token::NavHash(0x38AE75C2),
            Token::Float(-3.0),
            Token::NavHash(0x38AE75C2),
            Token::RawFloat(0.675),
        ])
    );
    // toggle
    assert_eq!(
        control_tokens(&hex("df d1 d7 f5 0f 69 f8 d8")),
        s(&[Token::NavHash(0xF50F69F8), Token::Click])
    );
    // path navigation /ch/1/mute + data request
    assert_eq!(
        control_tokens(&hex("df d1 da c1 63 68 40 c3 6d 75 74 65 dc")),
        s(&[
            Token::Root,
            Token::NavName(b"ch"),
            Token::NavIndex(1),
            Token::NavName(b"mute"),
            Token::DataRequest,
        ])
    );
    // hash containing DF: libwing form (escaped) and the also-legal unescaped form
    let want = s(&[Token::NavHash(0xDF17C242), Token::DefRequest]);
    assert_eq!(control_tokens(&hex("df d1 d7 df de 17 c2 42 dd")), want);
    assert_eq!(control_tokens(&hex("df d1 d7 df 17 c2 42 dd")), want);
    // change events from the doc's /*b dump
    assert_eq!(
        control_tokens(&hex(
            "df d1 d7 38ae75c2 d5 c3100000 d7 7e463474 d5 c3100000 d7 f50f69f8 01 d7 26855cd3 01"
        )),
        s(&[
            Token::NavHash(0x38AE75C2),
            Token::Float(-144.0),
            Token::NavHash(0x7E463474),
            Token::Float(-144.0),
            Token::NavHash(0xF50F69F8),
            Token::SmallInt(1),
            Token::NavHash(0x26855CD3),
            Token::SmallInt(1),
        ])
    );
    // root definition request
    assert_eq!(
        control_tokens(&hex("df d1 da dd")),
        s(&[Token::Root, Token::DefRequest])
    );
}

#[test]
fn libwing_client_sequences() {
    // libwing set_string(0xC6AD548D /ch/1/name, "Kick"), set_int(mute, 1), set_int(x, 1000),
    // set_float(fdr, 0.0), as written by libwing (keep-alive DF D1 first).
    let wire = hex(
        "df d1 d7 c6 ad 54 8d 83 4b 69 63 6b d7 f5 0f 69 f8 01 d7 00 00 00 01 d3 03 e8 d7 38 ae 75 c2 d5 00 00 00 00",
    );
    assert_eq!(
        control_tokens(&wire),
        s(&[
            Token::NavHash(0xC6AD548D),
            Token::Str(b"Kick"),
            Token::NavHash(0xF50F69F8),
            Token::SmallInt(1),
            Token::NavHash(1),
            Token::Int16(1000),
            Token::NavHash(0x38AE75C2),
            Token::Float(0.0),
        ])
    );

    // libwing request_meter(&[Channel(0), Fx(4)]) with UDP port 0x3737 and meter id 1, then
    // its renew packet.
    let wire =
        hex("df d3 d3 37 37 d4 00 01 37 37 dc a0 00 a6 04 de df d1 df d3 d4 00 01 37 37 df d1");
    let mut d = NrpDecoder::new();
    d.feed(&wire);
    let mut p = MeterRequestParser::new();
    let mut cmds = Vec::new();
    while let Some(t) = d.next_meter() {
        if let Some(c) = p.feed(t.unwrap()) {
            cmds.push(c);
        }
    }
    assert_eq!(
        cmds,
        vec![
            MeterCommand::SetPort(0x3737),
            MeterCommand::Renew(0x0001_3737),
            MeterCommand::Define {
                id: 0x0001_3737,
                selections: vec![
                    MeterSelection {
                        kind: MeterKind::Channel,
                        indices: vec![0]
                    },
                    MeterSelection {
                        kind: MeterKind::Fx,
                        indices: vec![4]
                    },
                ]
            },
            MeterCommand::Renew(0x0001_3737),
        ]
    );
    assert_eq!(d.next_control(), None);
}

#[test]
fn doc_meter_sequence() {
    // Doc 3.1.2 full sequence.
    let wire = hex("df d3 d3 37 37  df d3 d4 00 00 00 02 dc a0 01 de  df d3 d4 00 00 00 02");
    let mut d = NrpDecoder::new();
    d.feed(&wire);
    let mut toks = Vec::new();
    while let Some(t) = d.next_meter() {
        toks.push(t.unwrap());
    }
    assert_eq!(
        toks,
        vec![
            MeterToken::Port(14135),
            MeterToken::ReportId(2),
            MeterToken::Begin,
            MeterToken::Kind(MeterKind::Channel),
            MeterToken::Index(1),
            MeterToken::End,
            MeterToken::ReportId(2),
        ]
    );
}

#[test]
fn hash_with_df_is_escaped_by_encoder() {
    let mut e = NrpEncoder::new();
    let mut out = Vec::new();
    e.control(&Token::NavHash(0xDF17C242), &mut out).unwrap();
    e.control(&Token::DefRequest, &mut out).unwrap();
    assert_eq!(out, hex("df d1 d7 df de 17 c2 42 dd"));
}
