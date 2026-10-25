//! OSC server over real loopback UDP sockets (ephemeral ports).

use std::net::{Ipv4Addr, SocketAddrV4, UdpSocket};
use std::sync::Arc;
use std::time::Duration;

use wing_model::ParamStore;
use wing_proto::osc::{Arg, Message, OscAttachment, OscConfig, OscHandle, OscServer};

fn start(sub_timeout: Duration) -> (OscHandle, UdpSocket, Arc<ParamStore>) {
    let mut store = ParamStore::new();
    let att = OscAttachment::attach(&mut store);
    let store = Arc::new(store);
    let cfg = OscConfig {
        bind: SocketAddrV4::new(Ipv4Addr::LOCALHOST, 0),
        sub_timeout,
        ..Default::default()
    };
    let h = OscServer::bind(store.clone(), att, cfg)
        .unwrap()
        .spawn()
        .unwrap();
    let c = UdpSocket::bind("127.0.0.1:0").unwrap();
    c.set_read_timeout(Some(Duration::from_millis(500)))
        .unwrap();
    c.connect(h.local_addr()).unwrap();
    (h, c, store)
}

fn send(c: &UdpSocket, addr: &str, args: Vec<Arg>) {
    c.send(&Message::new(addr, args).to_vec()).unwrap();
}

fn recv(c: &UdpSocket) -> Option<Message> {
    let mut b = vec![0u8; 65536];
    let n = c.recv(&mut b).ok()?;
    Some(Message::decode(&b[..n]).unwrap())
}

fn get(c: &UdpSocket, addr: &str) -> Message {
    send(c, addr, vec![]);
    recv(c).expect("no reply")
}

#[test]
fn get_set_fader() {
    let (_h, c, _s) = start(Duration::from_secs(10));
    send(&c, "/ch/1/fdr", vec![Arg::Float(-10.0)]);
    let m = get(&c, "/ch/1/fdr");
    assert_eq!(m.addr, "/ch/1/fdr");
    assert_eq!(m.args[0], Arg::Str("-10.0".into()));
    assert_eq!(m.args[2], Arg::Float(-10.0));
    let Arg::Float(norm) = m.args[1] else {
        panic!()
    };
    // -10 dB is the middle of the WING fader law
    assert!((norm - 0.5).abs() < 0.01, "{norm}");
    // text set
    send(&c, "/ch/1/fdr", vec![Arg::Str("-oo".into())]);
    assert_eq!(get(&c, "/ch/1/fdr").args[2], Arg::Float(-144.0));
}

#[test]
fn get_set_mute_enum_string() {
    let (_h, c, st) = start(Duration::from_secs(10));
    send(&c, "/ch/1/mute", vec![Arg::Int(1)]);
    let m = get(&c, "/ch/1/mute");
    assert_eq!(m.args[2], Arg::Int(1));

    // an enum: the input source group of channel 1 (set by name, read back as the token)
    let id = st
        .lookup("ch/1/in/set/srcauto")
        .or_else(|| st.lookup("ch/1/clink"));
    let _ = id;
    let (path, tok) = find_enum(&st);
    send(&c, &path, vec![Arg::Str(tok.clone())]);
    let m = get(&c, &path);
    assert_eq!(m.args, vec![Arg::Str(tok)]);

    // a string: channel name
    send(&c, "/ch/1/name", vec![Arg::Str("Kick".into())]);
    assert_eq!(get(&c, "/ch/1/name").args, vec![Arg::Str("Kick".into())]);
}

fn find_enum(st: &ParamStore) -> (String, String) {
    use wing_model::ParamType;
    let s = st.schema();
    let id = s
        .ids()
        .find(|&i| {
            s.path(i).starts_with("ch/1/")
                && st.desc(i).is_some_and(|d| {
                    d.ty == ParamType::Enum && !d.unused_slot && d.enum_items().len() > 2
                })
        })
        .expect("an enum under ch/1");
    let d = st.desc(id).unwrap();
    let cur = st.get_raw(id) as usize;
    let other = (cur + 1) % d.enum_items().len();
    (
        format!("/{}", s.path(id)),
        d.enum_items()[other].token.into(),
    )
}

#[test]
fn subscription_push_and_expiry() {
    let (h, c, st) = start(Duration::from_millis(400));
    let ch1 = st.lookup("ch/1/fdr").unwrap();
    send(&c, "/*s", vec![]);
    // make sure the subscribe was processed before changing anything
    send(&c, "/?", vec![]);
    assert_eq!(recv(&c).unwrap().addr, "/?");

    st.set(ch1, wing_model::Value::Float(-20.0)).unwrap();
    let m = recv(&c).expect("event");
    assert_eq!(m.addr, "/ch/1/fdr");
    assert_eq!(m.args[2], Arg::Float(-20.0));

    // a change made by the subscriber itself is not echoed
    send(&c, "/ch/1/mute", vec![Arg::Int(1)]);
    c.set_read_timeout(Some(Duration::from_millis(100)))
        .unwrap();
    assert!(recv(&c).is_none());

    // renewal keeps it alive past the original deadline
    std::thread::sleep(Duration::from_millis(250));
    send(&c, "/*s", vec![]);
    std::thread::sleep(Duration::from_millis(250));
    c.set_read_timeout(Some(Duration::from_millis(500)))
        .unwrap();
    st.set(ch1, wing_model::Value::Float(-21.0)).unwrap();
    assert_eq!(
        recv(&c).expect("event after renew").args[2],
        Arg::Float(-21.0)
    );

    // expiry
    std::thread::sleep(Duration::from_millis(500));
    st.set(ch1, wing_model::Value::Float(-22.0)).unwrap();
    c.set_read_timeout(Some(Duration::from_millis(200)))
        .unwrap();
    assert!(recv(&c).is_none(), "event after expiry");
    assert!(h.requests() >= 4);
}

#[test]
fn native_subscription_and_redirect_port() {
    let (_h, c, st) = start(Duration::from_secs(10));
    let sink = UdpSocket::bind("127.0.0.1:0").unwrap();
    sink.set_read_timeout(Some(Duration::from_millis(500)))
        .unwrap();
    let port = sink.local_addr().unwrap().port();
    send(&c, &format!("/%{port}/*S"), vec![]);
    send(&c, "/?", vec![]);
    recv(&c).unwrap();
    let id = st.lookup("ch/2/mute").unwrap();
    st.set(id, wing_model::Value::Int(1)).unwrap();
    let m = recv(&sink).expect("event on redirect port");
    assert_eq!(m.addr, "/ch/2/mute");
    assert_eq!(m.args, vec![Arg::Int(1)]);
}
