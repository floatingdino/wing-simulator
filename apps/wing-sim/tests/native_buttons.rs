//! T083: WING-Edit-style native client against the daemon's control side (native server + mix
//! logic, no audio): strip mute clicks are confirmed with `mute` and the `$mute` indicator, and
//! solos from the client are additive with the factory EXCL SOLO on.

use std::io::{Read, Write};
use std::net::{Ipv4Addr, SocketAddrV4, TcpStream};
use std::sync::atomic::{AtomicBool, Ordering::Relaxed};
use std::sync::Arc;
use std::time::{Duration, Instant};

use wing_model::{ParamStore, Value};
use wing_proto::nrp::{NrpDecoder, NrpEncoder, Token};
use wing_proto::server::{EventFn, Hub, NativeServer, ServerConfig, ServerShared};

struct Rig {
    store: Arc<ParamStore>,
    server: wing_proto::server::ServerHandle,
    stop: Arc<AtomicBool>,
    ctl: Option<std::thread::JoinHandle<()>>,
}

impl Drop for Rig {
    fn drop(&mut self) {
        self.stop.store(true, Relaxed);
        if let Some(t) = self.ctl.take() {
            let _ = t.join();
        }
    }
}

fn rig() -> Rig {
    let mut store = ParamStore::new();
    let mut ctl = wing_sim::audio::prepare(&mut store);
    let hub = Hub::attach(&mut store);
    let store = Arc::new(store);
    let stop = Arc::new(AtomicBool::new(false));
    // the audio control thread's refresh loop
    let (st, sp) = (store.clone(), stop.clone());
    let ctl = std::thread::spawn(move || {
        while !sp.load(Relaxed) {
            ctl.refresh(&st);
            std::thread::sleep(Duration::from_millis(2));
        }
    });
    let cfg = ServerConfig {
        bind: SocketAddrV4::new(Ipv4Addr::LOCALHOST, 0),
        idle_timeout: None,
        ..Default::default()
    };
    let shared = ServerShared {
        store: store.clone(),
        hub,
        meters: None,
    };
    let on_event: EventFn = Arc::new(|_| {});
    let server = NativeServer::bind(cfg, shared, on_event)
        .unwrap()
        .spawn()
        .unwrap();
    Rig {
        store,
        server,
        stop,
        ctl: Some(ctl),
    }
}

struct Client {
    s: TcpStream,
    enc: NrpEncoder,
    dec: NrpDecoder,
    cur: Option<u32>,
    /// Every `(hash, value)` event received, in order.
    seen: Vec<(u32, String)>,
}

impl Client {
    fn connect(r: &Rig) -> Self {
        let s = TcpStream::connect(r.server.local_addr()).unwrap();
        s.set_read_timeout(Some(Duration::from_millis(20))).unwrap();
        let mut dec = NrpDecoder::new();
        dec.feed(&[0xDF, 0xD1]); // the server selects ch1 once
        Client {
            s,
            enc: NrpEncoder::new(),
            dec,
            cur: None,
            seen: Vec::new(),
        }
    }

    fn send(&mut self, tokens: &[Token]) {
        let mut v = Vec::new();
        for t in tokens {
            self.enc.control(t, &mut v).unwrap();
        }
        self.s.write_all(&v).unwrap();
    }

    /// Read events until `(hash, value)` arrives (true) or 3 s pass (false).
    fn wait_for(&mut self, hash: u32, value: &str) -> bool {
        self.wait_within(hash, value, Duration::from_secs(3))
    }

    /// [`Client::wait_for`] with a timeout.
    fn wait_within(&mut self, hash: u32, value: &str, timeout: Duration) -> bool {
        let t0 = Instant::now();
        let mut buf = [0u8; 4096];
        loop {
            if self.seen.iter().any(|(h, v)| *h == hash && v == value) {
                return true;
            }
            if t0.elapsed() > timeout {
                return false;
            }
            if let Ok(n) = self.s.read(&mut buf) {
                self.dec.feed(&buf[..n]);
                while let Some(t) = self.dec.next_control() {
                    match t.unwrap() {
                        Token::NavHash(h) => self.cur = Some(h),
                        t if t.is_value() => {
                            if let Some(h) = self.cur.take() {
                                self.seen.push((h, format!("{t:?}")));
                            }
                        }
                        _ => {}
                    }
                }
            }
        }
    }

    /// Drain for `d` and return the events of `hash`.
    fn events_of(&mut self, hash: u32, d: Duration) -> Vec<String> {
        self.wait_within(0, "never", d);
        self.seen
            .iter()
            .filter(|(h, _)| *h == hash)
            .map(|(_, v)| v.clone())
            .collect()
    }
}

fn h(r: &Rig, p: &str) -> u32 {
    r.store.schema().hash(r.store.lookup(p).unwrap())
}

fn get(r: &Rig, p: &str) -> Option<Value> {
    r.store.get(r.store.lookup(p).unwrap())
}

#[test]
fn wing_edit_mute_clicks_are_confirmed_with_the_indicator() {
    let r = rig();
    let mut c = Client::connect(&r);
    for strip in ["ch/1", "bus/2", "main/1"] {
        let (mute, led) = (
            h(&r, &format!("{strip}/mute")),
            h(&r, &format!("{strip}/$mute")),
        );
        for v in ["SmallInt(1)", "SmallInt(0)"] {
            c.send(&[Token::NavHash(mute), Token::Click]);
            assert!(c.wait_for(mute, v), "{strip}/mute -> {v}: {:?}", c.seen);
            assert!(c.wait_for(led, v), "{strip}/$mute -> {v}: {:?}", c.seen);
            c.seen.clear();
        }
    }
    // a value set (WING-Edit's DCA-style button) also updates the indicator for the setter
    let (mute, led) = (h(&r, "ch/7/mute"), h(&r, "ch/7/$mute"));
    c.send(&[Token::NavHash(mute), Token::SmallInt(1)]);
    assert!(c.wait_for(led, "SmallInt(1)"), "{:?}", c.seen);
    assert_eq!(get(&r, "ch/7/mute"), Some(Value::Int(1)));
}

#[test]
fn wing_edit_solos_add_up_with_factory_excl_solo() {
    let r = rig();
    assert_eq!(get(&r, "$ctl/cfg/soloexcl"), Some(Value::Int(1)));
    let mut c = Client::connect(&r);
    // the log's WING-Edit sequence: `ch/1/$solo = 1`, then `bus/5/$solo = 1`, then a click
    let solos = ["ch/1/$solo", "bus/5/$solo", "ch/2/$solo"];
    c.send(&[Token::NavHash(h(&r, solos[0])), Token::SmallInt(1)]);
    assert!(c.wait_for(h(&r, "ch/1/$sololed"), "SmallInt(1)"));
    c.send(&[Token::NavHash(h(&r, solos[1])), Token::SmallInt(1)]);
    assert!(c.wait_for(h(&r, "bus/5/$sololed"), "SmallInt(1)"));
    c.send(&[Token::NavHash(h(&r, solos[2])), Token::Click]);
    assert!(c.wait_for(h(&r, solos[2]), "SmallInt(1)"), "{:?}", c.seen);
    assert!(c.wait_for(h(&r, "ch/2/$sololed"), "SmallInt(1)"));
    // nothing switched the earlier solos off
    for p in solos {
        assert_eq!(get(&r, p), Some(Value::Int(1)), "{p}");
        assert!(
            !c.events_of(h(&r, p), Duration::from_millis(100))
                .contains(&"SmallInt(0)".to_string()),
            "{p} cleared"
        );
    }
    assert_eq!(get(&r, "$stat/solo"), Some(Value::Int(1)));
}
