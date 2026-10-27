//! Discovery responder on loopback: send `WING?`, expect the identity reply (ephemeral port, so
//! it never collides with a real 2222 listener).

use std::net::{Ipv4Addr, SocketAddrV4, UdpSocket};
use std::time::Duration;
use wing_proto::discovery::*;

fn responder(advertise_ip: Option<Ipv4Addr>, identity: Identity) -> ResponderHandle {
    let cfg = ResponderConfig {
        bind: SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, 0),
        identity,
        advertise_ip,
    };
    Responder::bind(cfg).unwrap().spawn(|_, _| {}).unwrap()
}

fn probe(port: u16, payload: &[u8]) -> Option<(Vec<u8>, u16)> {
    let c = UdpSocket::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
    c.set_read_timeout(Some(Duration::from_millis(500)))
        .unwrap();
    c.send_to(payload, (Ipv4Addr::LOCALHOST, port)).unwrap();
    let mut buf = [0u8; 512];
    match c.recv_from(&mut buf) {
        Ok((n, from)) => Some((buf[..n].to_vec(), from.port())),
        Err(_) => None,
    }
}

#[test]
fn answers_probe_with_interface_ip() {
    let h = responder(None, Identity::default());
    let port = h.local_addr().port();
    let (reply, from_port) = probe(port, b"WING?").expect("no reply");
    // Reply comes from the responder's port (2222 in production).
    assert_eq!(from_port, port);
    // No trailing NUL.
    assert_ne!(reply.last(), Some(&0));
    let info = parse_reply(&reply).unwrap();
    // The probe arrived on loopback, so that is the advertised address.
    assert_eq!(info.ip, "127.0.0.1");
    assert_eq!(info.identity, Identity::default());
    // The responder counts a reply after sending it: give its thread a moment.
    let t0 = std::time::Instant::now();
    while h.replies() == 0 && t0.elapsed() < Duration::from_secs(2) {
        std::thread::sleep(Duration::from_millis(1));
    }
    assert_eq!(h.replies(), 1);
    // Non-probes are ignored.
    assert!(probe(port, b"HELLO").is_none());
    h.stop();
}

#[test]
fn configured_identity_and_ip_override() {
    let id = Identity {
        name: "STUDIO-B".into(),
        model: "wing-rack".into(),
        serial: "S123".into(),
        firmware: "3.0.5-0-g0c2b9d4a:release".into(),
    };
    let h = responder(Some(Ipv4Addr::new(10, 9, 9, 9)), id.clone());
    let (reply, _) = probe(h.local_addr().port(), b"WING?").expect("no reply");
    assert_eq!(
        std::str::from_utf8(&reply).unwrap(),
        "WING,10.9.9.9,STUDIO-B,wing-rack,S123,3.0.5-0-g0c2b9d4a:release"
    );
}

#[test]
fn coexists_with_another_listener_on_the_same_port() {
    let h = responder(None, Identity::default());
    // A second responder can bind the same port thanks to SO_REUSEADDR/SO_REUSEPORT.
    let cfg = ResponderConfig {
        bind: SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, h.local_addr().port()),
        ..Default::default()
    };
    assert!(Responder::bind(cfg).is_ok());
}

#[test]
fn rejects_invalid_identity() {
    let cfg = ResponderConfig {
        bind: SocketAddrV4::new(Ipv4Addr::LOCALHOST, 0),
        identity: Identity {
            name: "a,b".into(),
            ..Default::default()
        },
        advertise_ip: None,
    };
    assert!(Responder::bind(cfg).is_err());
}
