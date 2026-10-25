//! T013/T015 replay and integration tests: recorded WING-Edit 3.3.4 client streams
//! (fixtures/protocol, T00B) against the native server.

use std::collections::HashSet;
use std::io::{Read, Write};
use std::net::{Ipv4Addr, SocketAddrV4, TcpStream, UdpSocket};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use wing_model::{ParamStore, ParamType, Value};
use wing_proto::nrp::{MeterPacket, NrpDecoder, Token};
use wing_proto::osc::{Arg, Message, OscAttachment, OscConfig, OscServer};
use wing_proto::server::meters::{MeterBank, MeterStreamer, MeterSubscriptions, SyntheticMeters};
use wing_proto::server::session::HASH_CNSMDL;
use wing_proto::server::{
    EventFn, Hub, LiveInfo, NativeServer, ServerConfig, ServerEvent, ServerShared, Session,
};

fn fixture(name: &str) -> Vec<u8> {
    let p = format!(
        "{}/../../fixtures/protocol/{name}",
        env!("CARGO_MANIFEST_DIR")
    );
    std::fs::read(&p).unwrap_or_else(|e| panic!("{p}: {e}"))
}

/// Replay a client stream through a session in 1400-byte segments (TCP-sized); returns the
/// decoded replies.
fn replay(store: &ParamStore, wire: &[u8]) -> (Session, Vec<u8>) {
    let mut s = Session::new(0x4E52_0001, Arc::new(LiveInfo::default()));
    let mut out = Vec::new();
    for chunk in wire.chunks(1400) {
        s.feed(store, chunk, &mut out);
    }
    (s, out)
}

#[derive(Debug, PartialEq)]
enum Reply {
    Value(u32, String),
    End,
    Def(u32),
}

fn decode_replies(out: &[u8]) -> Vec<Reply> {
    let mut d = NrpDecoder::new();
    // the server selects ch1 once per connection; later reads continue on ch1
    d.feed(&[0xDF, 0xD1]);
    d.feed(out);
    let mut v = Vec::new();
    let mut cur = None;
    while let Some(t) = d.next_control() {
        match t.unwrap() {
            Token::NavHash(h) => cur = Some(h),
            Token::End => v.push(Reply::End),
            Token::NodeDef(b) => {
                v.push(Reply::Def(wing_proto::nrp::NodeDef::parse(b).unwrap().hash))
            }
            t if t.is_value() => v.push(Reply::Value(cur.take().unwrap(), format!("{t:?}"))),
            t => panic!("unexpected {t:?}"),
        }
    }
    v
}

/// The store value of `path` equals the fixture's JSON value (after the console's own
/// quantisation).
fn matches_json(st: &ParamStore, path: &str, v: &serde_json::Value) -> Result<(), String> {
    // fixture paths are tree (hash) paths: `fx/13/20` is slot 20, not GEQ's 20 Hz band
    let id = st.schema().by_path(path).ok_or("unknown path")?;
    let d = st.desc(id).ok_or("not a param")?;
    if d.unused_slot {
        return Err("inactive slot".into());
    }
    let raw = st.get_raw(id);
    let ok = match (d.ty, v) {
        (ParamType::Str, serde_json::Value::String(s)) => {
            st.with_str(id, |x| x == s.as_str()).unwrap()
        }
        (ParamType::Enum | ParamType::FEnum, serde_json::Value::String(s)) => {
            d.enum_index(s) == Some(raw)
        }
        (_, serde_json::Value::String(s)) => d.parse_text(s) == Ok(raw),
        (_, serde_json::Value::Number(n)) => {
            let f = n.as_f64().unwrap();
            let want = match d.ty {
                ParamType::Int | ParamType::Enum => d.encode(Value::Int(f as i32)),
                _ => d.encode(Value::Float(f as f32)),
            };
            want == Ok(raw)
        }
        _ => false,
    };
    if ok {
        Ok(())
    } else {
        let mut cur = String::new();
        st.format_token(id, &mut cur).unwrap();
        Err(format!("store has {cur}, fixture {v}"))
    }
}

#[test]
fn replay_capture_a1_push_reproduces_scene() {
    let st = ParamStore::new();
    let wire = fixture("wing-edit-connect-1.bin");
    let t0 = Instant::now();
    let (s, out) = replay(&st, &wire);
    let elapsed = t0.elapsed();
    let stats = s.stats();
    let json: serde_json::Value =
        serde_json::from_slice(&fixture("pc-to-mixer-scene.json")).unwrap();
    // 29,914 sets incl. 1,392 to hash 0 and 3 RTA sets interleaved with the gets
    assert_eq!(stats.sets, json["set_count"].as_u64().unwrap());
    assert_eq!(stats.hash0, 1392);
    assert_eq!(stats.dropped, 1392);
    assert_eq!(stats.unknown_hash, 0);
    assert_eq!(stats.decode_errors, 0);
    assert_eq!(stats.gets, 25);
    assert_eq!(stats.defs, 2);
    assert!(stats.rejected < 10, "rejected {}", stats.rejected);
    // must keep up with the ~0.5 s burst (generous for debug builds)
    assert!(elapsed < Duration::from_secs(5), "{elapsed:?}");
    eprintln!("A1 replay: {stats:?} in {elapsed:?}");

    // replies: 25 values each followed by DE, then 2 DD answers (records + DE)
    let r = decode_replies(&out);
    let values = r.iter().filter(|x| matches!(x, Reply::Value(..))).count();
    assert_eq!(values, 25);
    assert_eq!(r.iter().filter(|x| **x == Reply::End).count(), 27);

    // resulting state == the pushed scene
    let params = json["params"].as_object().unwrap();
    assert_eq!(params.len(), 28_522);
    let mut bad = Vec::new();
    for (path, v) in params {
        if path == "$syscfg/$cnsmdl" {
            continue; // console-local, never taken from the client
        }
        if let Err(e) = matches_json(&st, path, v) {
            bad.push(format!("{path}: {e}"));
        }
    }
    assert!(
        bad.is_empty(),
        "{} mismatches, e.g. {:?}",
        bad.len(),
        &bad[..bad.len().min(10)]
    );
}

#[test]
fn replay_capture_b_answers_every_request() {
    let st = ParamStore::new();
    let (mut s, out) = replay(&st, &fixture("wing-edit-connect-tokens-1.bin"));
    let stats = s.stats();
    assert_eq!(stats.decode_errors, 0);
    assert_eq!(stats.gets, 25);
    assert_eq!(stats.defs, 2);
    assert_eq!(stats.model_requests, 1);
    assert_eq!(stats.hash0, 192);
    let r = decode_replies(&out);
    // one `D7 h <value> DE` per get, the model reply (no DE), and DE after each DD
    let mut gets = 0;
    let mut i = 0;
    let mut model = None;
    let mut defs_end = 0;
    while i < r.len() {
        match &r[i] {
            Reply::Value(h, v) if *h == HASH_CNSMDL => {
                model = Some(v.clone());
                i += 1;
            }
            Reply::Value(..) => {
                assert_eq!(r.get(i + 1), Some(&Reply::End), "{:?}", r[i]);
                gets += 1;
                i += 2;
            }
            Reply::Def(_) => i += 1,
            Reply::End => {
                defs_end += 1;
                i += 1;
            }
        }
    }
    assert_eq!(gets, 25);
    assert_eq!(defs_end, 2);
    assert_eq!(model.unwrap(), format!("Str({:?})", b"wing-fullsize"));
    // the same hashes the token-mode fake answered (fixture server bytes)
    let fake = decode_replies(&fixture("wing-edit-connect-tokens-1.server.bin"));
    let fake_hashes: HashSet<u32> = fake
        .iter()
        .filter_map(|x| match x {
            Reply::Value(h, _) => Some(*h),
            _ => None,
        })
        .collect();
    let our_hashes: HashSet<u32> = r
        .iter()
        .filter_map(|x| match x {
            Reply::Value(h, _) if *h != HASH_CNSMDL => Some(*h),
            _ => None,
        })
        .collect();
    assert_eq!(fake_hashes, our_hashes);
    // channel 3: port, defines 0-3 and 28 renews of report 3
    let cmds: Vec<_> = s.take_meter_commands().collect();
    use wing_proto::nrp::MeterCommand::*;
    assert!(matches!(cmds[0], SetPort(60383)));
    let defines: Vec<u32> = cmds
        .iter()
        .filter_map(|c| match c {
            Define { id, .. } => Some(*id),
            _ => None,
        })
        .collect();
    assert_eq!(defines, [0, 1, 2, 3]);
    let renews3 = cmds.iter().filter(|c| matches!(c, Renew(3))).count();
    assert!(renews3 >= 28, "{renews3}");
}

#[test]
fn mixer_to_pc_root_dump() {
    // `DA DC` (WING-Edit's MIXER -> PC request): every active param, then DE [unverified]
    let st = ParamStore::new();
    let mut s = Session::new(1, Arc::new(LiveInfo::default()));
    let mut out = Vec::new();
    s.feed(&st, &[0xDF, 0xD1, 0xDA, 0xDC], &mut out);
    let r = decode_replies(&out);
    assert_eq!(r.last(), Some(&Reply::End));
    let n = r.len() - 1;
    assert!(n > 28_000 && n < 36_499, "{n}");
    let fdr = st.schema().hash(st.lookup("ch/1/fdr").unwrap());
    assert!(r.contains(&Reply::Value(fdr, "Float(-144.0)".into())));
}

// ---- TCP integration ----------------------------------------------------------------------

struct Rig {
    store: Arc<ParamStore>,
    server: wing_proto::server::ServerHandle,
    events: Arc<Mutex<Vec<String>>>,
    _streamer: MeterStreamer,
    osc: wing_proto::osc::OscHandle,
}

fn rig(idle: Option<Duration>) -> Rig {
    let mut store = ParamStore::new();
    let hub = Hub::attach(&mut store);
    let osc_att = OscAttachment::attach(&mut store);
    let store = Arc::new(store);
    let subs = Arc::new(MeterSubscriptions::new());
    let bank = Arc::new(MeterBank::for_schema());
    SyntheticMeters::fill(&bank, None, 1.0);
    let streamer = MeterStreamer::spawn(
        subs.clone(),
        bank,
        UdpSocket::bind("127.0.0.1:0").unwrap(),
        Duration::from_millis(50),
    )
    .unwrap();
    let events = Arc::new(Mutex::new(Vec::new()));
    let ev = events.clone();
    let on_event: EventFn = Arc::new(move |e: &ServerEvent| {
        ev.lock().unwrap().push(format!("{e:?}"));
    });
    let cfg = ServerConfig {
        bind: SocketAddrV4::new(Ipv4Addr::LOCALHOST, 0),
        idle_timeout: idle,
        set_report_gap: Duration::from_millis(50),
        ..Default::default()
    };
    let shared = ServerShared {
        store: store.clone(),
        hub,
        meters: Some(subs),
    };
    let server = NativeServer::bind(cfg, shared, on_event)
        .unwrap()
        .spawn()
        .unwrap();
    let osc = OscServer::bind(
        store.clone(),
        osc_att,
        OscConfig {
            bind: SocketAddrV4::new(Ipv4Addr::LOCALHOST, 0),
            ..Default::default()
        },
    )
    .unwrap()
    .spawn()
    .unwrap();
    Rig {
        store,
        server,
        events,
        _streamer: streamer,
        osc,
    }
}

fn connect(r: &Rig) -> TcpStream {
    let c = TcpStream::connect(r.server.local_addr()).unwrap();
    c.set_read_timeout(Some(Duration::from_millis(100)))
        .unwrap();
    c
}

/// Read until `pred(decoded replies)` or timeout.
fn read_until(c: &mut TcpStream, mut pred: impl FnMut(&[Reply]) -> bool) -> Vec<Reply> {
    let mut all = Vec::new();
    let t0 = Instant::now();
    let mut quiet = Instant::now();
    let mut buf = [0u8; 65536];
    while t0.elapsed() < Duration::from_secs(5) && quiet.elapsed() < Duration::from_millis(500) {
        if let Ok(n) = c.read(&mut buf) {
            if n == 0 {
                break;
            }
            quiet = Instant::now();
            all.extend_from_slice(&buf[..n]);
        }
        let r = decode_replies(&all);
        if pred(&r) {
            return r;
        }
    }
    decode_replies(&all)
}

fn set_wire(hash: u32, t: Token) -> Vec<u8> {
    let mut e = wing_proto::nrp::NrpEncoder::new();
    let mut v = Vec::new();
    e.control(&Token::NavHash(hash), &mut v).unwrap();
    e.control(&t, &mut v).unwrap();
    v
}

#[test]
fn tcp_push_fanout_meters_and_osc() {
    let r = rig(None);
    let s = r.store.schema();
    let fdr2 = s.hash(r.store.lookup("ch/2/fdr").unwrap());

    let mut b = connect(&r); // observer
    std::thread::sleep(Duration::from_millis(100));
    // WING-Edit A (capture B stream, with a UDP meter socket standing in for its port 60383:
    // the meter port in the stream is rewritten to ours)
    let meter_rx = UdpSocket::bind("127.0.0.1:0").unwrap();
    meter_rx
        .set_read_timeout(Some(Duration::from_secs(2)))
        .unwrap();
    let port = meter_rx.local_addr().unwrap().port();
    let mut wire = fixture("wing-edit-connect-tokens-1.bin");
    // first segment: DF D3 D3 EB DF DE DE (port 0xEBDF escaped)
    assert_eq!(&wire[..7], &[0xDF, 0xD3, 0xD3, 0xEB, 0xDF, 0xDE, 0xDE]);
    let mut head = vec![0xDF, 0xD3, 0xD3];
    for byte in port.to_be_bytes() {
        head.push(byte);
        if byte == 0xDF {
            head.push(0xDE);
        }
    }
    head.push(0xDE);
    wire.splice(..7, head);
    let mut a = connect(&r);
    let t0 = Instant::now();
    a.write_all(&wire).unwrap();
    let ra = read_until(&mut a, |x| {
        x.iter()
            .any(|y| matches!(y, Reply::Value(h, _) if *h == HASH_CNSMDL))
    });
    assert!(ra
        .iter()
        .any(|y| matches!(y, Reply::Value(h, _) if *h == HASH_CNSMDL)));
    eprintln!("push + replies in {:?}", t0.elapsed());
    // meter packets for A
    let mut buf = [0u8; 8192];
    let n = meter_rx.recv(&mut buf).expect("meter packet");
    let p = MeterPacket::parse(&buf[..n]).unwrap();
    assert!(p.report_id <= 3);
    assert!(p.len() >= 16 * 11);

    // B got A's push as change events (coalesced)
    let rb = read_until(&mut b, |_| false);
    assert!(rb.len() > 1000, "{}", rb.len());

    // a change from B reaches A (not echoed to B)
    std::thread::sleep(Duration::from_millis(200));
    let _ = read_until(&mut a, |_| false); // drain
    b.write_all(&set_wire(fdr2, Token::Float(-7.5))).unwrap();
    let ra = read_until(&mut a, |x| {
        x.contains(&Reply::Value(fdr2, "Float(-7.5)".into()))
    });
    assert!(
        ra.contains(&Reply::Value(fdr2, "Float(-7.5)".into())),
        "{:?} {:?}",
        &ra[..ra.len().min(20)],
        r.events.lock().unwrap()
    );
    assert_eq!(
        r.store.get(r.store.lookup("ch/2/fdr").unwrap()),
        Some(Value::Float(-7.5))
    );
    let rb = read_until(&mut b, |_| false);
    assert!(!rb.contains(&Reply::Value(fdr2, "Float(-7.5)".into())));

    // OSC <-> native fan-out both ways
    let oc = UdpSocket::bind("127.0.0.1:0").unwrap();
    oc.set_read_timeout(Some(Duration::from_millis(500)))
        .unwrap();
    oc.connect(r.osc.local_addr()).unwrap();
    oc.send(&Message::new("/*S", vec![]).to_vec()).unwrap();
    oc.send(&Message::new("/?", vec![]).to_vec()).unwrap();
    let mut ob = vec![0u8; 65536];
    let n = oc.recv(&mut ob).unwrap();
    assert_eq!(Message::decode(&ob[..n]).unwrap().addr, "/?");
    oc.send(&Message::new("/ch/3/fdr", vec![Arg::Float(-12.0)]).to_vec())
        .unwrap();
    let fdr3 = s.hash(r.store.lookup("ch/3/fdr").unwrap());
    let ra = read_until(&mut a, |x| {
        x.contains(&Reply::Value(fdr3, "Float(-12.0)".into()))
    });
    assert!(ra.contains(&Reply::Value(fdr3, "Float(-12.0)".into())));
    b.write_all(&set_wire(fdr2, Token::Float(-3.0))).unwrap();
    let mut got = false;
    let t0 = Instant::now();
    while !got && t0.elapsed() < Duration::from_secs(3) {
        if let Ok(n) = oc.recv(&mut ob) {
            let m = Message::decode(&ob[..n]).unwrap();
            got = m.addr == "/ch/2/fdr";
        }
    }
    assert!(got, "native set did not reach the OSC subscriber");

    drop(a);
    drop(b);
    std::thread::sleep(Duration::from_millis(200));
    assert_eq!(r.server.clients(), 0);
    let ev = r.events.lock().unwrap().join("\n");
    assert!(ev.contains("ModelRequest"), "{ev}");
    assert!(ev.contains("Sets"), "{ev}");
    assert!(ev.contains("Disconnected"), "{ev}");
}

#[test]
fn idle_timeout_closes_connection() {
    let r = rig(Some(Duration::from_millis(300)));
    let mut c = connect(&r);
    c.write_all(&[0xDF, 0xD1]).unwrap();
    let t0 = Instant::now();
    let mut buf = [0u8; 16];
    loop {
        match c.read(&mut buf) {
            Ok(0) => break,
            _ if t0.elapsed() > Duration::from_secs(3) => panic!("not closed"),
            _ => {}
        }
    }
    assert!(t0.elapsed() >= Duration::from_millis(250));
    std::thread::sleep(Duration::from_millis(100));
    assert!(r.events.lock().unwrap().iter().any(|e| e.contains("idle")));
}

#[test]
fn max_clients_and_shutdown() {
    let r = rig(None);
    let mut cs = Vec::new();
    for _ in 0..24 {
        cs.push(connect(&r));
    }
    std::thread::sleep(Duration::from_millis(200));
    assert_eq!(r.server.clients(), 24);
    let mut extra = connect(&r);
    let mut buf = [0u8; 4];
    let t0 = Instant::now();
    loop {
        match extra.read(&mut buf) {
            Ok(0) => break,
            Err(e) if e.kind() != std::io::ErrorKind::WouldBlock => break,
            _ => assert!(
                t0.elapsed() < Duration::from_secs(3),
                "extra client not refused"
            ),
        }
    }
    assert!(r
        .events
        .lock()
        .unwrap()
        .iter()
        .any(|e| e.contains("Refused")));
    let Rig { server, .. } = r;
    server.stop();
    for mut c in cs {
        c.set_read_timeout(Some(Duration::from_secs(2))).unwrap();
        assert!(matches!(c.read(&mut buf), Ok(0) | Err(_)));
    }
}
