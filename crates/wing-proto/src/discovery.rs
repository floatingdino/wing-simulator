//! Discovery responder (UDP 2222, spec §1).
//!
//! WING-Edit broadcasts the 5-byte probe `WING?` every 2 s (subnet-directed and limited
//! broadcast). The console answers to the probe's source ip:port, from port 2222, with
//! `WING,<ip>,<name>,<model>,<serial>,<firmware>` (ASCII, no terminator). WING-Edit lists and
//! connects to the `<ip>` field, not the datagram's source address, so `<ip>` must be an address
//! the prober can reach: by default the local address the OS routes toward the prober (on a
//! normal network that is the interface the probe arrived on), or a configured override.
//!
//! The socket is bound with `SO_REUSEADDR` + `SO_REUSEPORT` so it can co-exist with other
//! listeners on 2222 (e.g. the protocol-probe scripts); broadcasts are delivered to all of them.

use std::io;
use std::net::{IpAddr, Ipv4Addr, SocketAddr, SocketAddrV4, UdpSocket};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::Duration;

use socket2::{Domain, Protocol, Socket, Type};

/// UDP discovery port (the native TCP protocol uses the same number).
pub const DISCOVERY_PORT: u16 = 2222;
/// The probe payload.
pub const PROBE: &[u8] = b"WING?";
/// Default console name.
pub const DEFAULT_NAME: &str = "WINGSIM";
/// Default model: `ngc-full` is the full-size WING (shown as "WING" in WING-Edit).
pub const DEFAULT_MODEL: &str = "ngc-full";
/// Default serial number.
pub const DEFAULT_SERIAL: &str = "WSIM00000001";
/// Default firmware string. WING-Edit 3.3.4 gates CONNECT on this field only (disassembly, see
/// docs/research/protocol.md "Firmware compatibility check"): char[0] >= '3' and char[2] >= '1',
/// i.e. major.minor >= 3.1; the suffix/hash is not checked. Format follows the console's
/// `<maj>.<min>.<patch>-<n>-g<hash>:<branch>`. Override via `Identity::firmware`.
pub const DEFAULT_FIRMWARE: &str = "3.3.4-0-g0c2b9d4a:release";

/// What the simulator advertises.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Identity {
    pub name: String,
    pub model: String,
    pub serial: String,
    pub firmware: String,
}

impl Default for Identity {
    fn default() -> Self {
        Identity {
            name: DEFAULT_NAME.into(),
            model: DEFAULT_MODEL.into(),
            serial: DEFAULT_SERIAL.into(),
            firmware: DEFAULT_FIRMWARE.into(),
        }
    }
}

impl Identity {
    /// Every field must be non-empty printable ASCII without commas (the reply is a CSV).
    pub fn validate(&self) -> Result<(), String> {
        for (k, v) in [
            ("name", &self.name),
            ("model", &self.model),
            ("serial", &self.serial),
            ("firmware", &self.firmware),
        ] {
            if v.is_empty() {
                return Err(format!("identity {k} is empty"));
            }
            if let Some(c) = v.chars().find(|&c| c == ',' || !(' '..='~').contains(&c)) {
                return Err(format!("identity {k} contains invalid character {c:?}"));
            }
        }
        Ok(())
    }
}

/// True if `datagram` is a discovery probe. Tolerates a trailing NUL/CR/LF.
pub fn is_probe(datagram: &[u8]) -> bool {
    let end = datagram
        .iter()
        .rposition(|&b| !matches!(b, 0 | b'\r' | b'\n'))
        .map_or(0, |i| i + 1);
    &datagram[..end] == PROBE
}

/// Format the reply string.
pub fn format_reply(ip: Ipv4Addr, id: &Identity) -> String {
    format!(
        "WING,{ip},{},{},{},{}",
        id.name, id.model, id.serial, id.firmware
    )
}

/// A parsed discovery reply (client side; used by tests and tools).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DiscoveryInfo {
    pub ip: String,
    pub identity: Identity,
}

/// Parse `WING,<ip>,<name>,<model>,<serial>,<firmware>`.
pub fn parse_reply(datagram: &[u8]) -> Option<DiscoveryInfo> {
    let s = std::str::from_utf8(datagram).ok()?.trim_end_matches('\0');
    let mut f = s.splitn(6, ',');
    if f.next()? != "WING" {
        return None;
    }
    let ip = f.next()?.to_string();
    let name = f.next()?.to_string();
    let model = f.next()?.to_string();
    let serial = f.next()?.to_string();
    let firmware = f.next()?.to_string();
    Some(DiscoveryInfo {
        ip,
        identity: Identity {
            name,
            model,
            serial,
            firmware,
        },
    })
}

/// The local IPv4 address the OS would use to reach `peer` (a connected UDP socket's local
/// address; no packet is sent).
pub fn local_ipv4_toward(peer: Ipv4Addr) -> io::Result<Ipv4Addr> {
    let s = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, 0))?;
    s.connect((peer, 9))?;
    match s.local_addr()?.ip() {
        IpAddr::V4(ip) if !ip.is_unspecified() => Ok(ip),
        other => Err(io::Error::other(format!(
            "no usable local IPv4 toward {peer} (got {other})"
        ))),
    }
}

/// Responder configuration.
#[derive(Clone, Debug)]
pub struct ResponderConfig {
    /// Address to bind (default `0.0.0.0:2222`).
    pub bind: SocketAddrV4,
    pub identity: Identity,
    /// Advertise this IP instead of the auto-detected one (e.g. a loopback alias, or when the
    /// TCP server listens on a specific interface).
    pub advertise_ip: Option<Ipv4Addr>,
}

impl Default for ResponderConfig {
    fn default() -> Self {
        ResponderConfig {
            bind: SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, DISCOVERY_PORT),
            identity: Identity::default(),
            advertise_ip: None,
        }
    }
}

/// A bound discovery responder.
pub struct Responder {
    sock: UdpSocket,
    cfg: ResponderConfig,
}

impl Responder {
    /// Bind the UDP socket with SO_REUSEADDR/SO_REUSEPORT and SO_BROADCAST.
    pub fn bind(cfg: ResponderConfig) -> io::Result<Responder> {
        cfg.identity
            .validate()
            .map_err(|e| io::Error::new(io::ErrorKind::InvalidInput, e))?;
        let s = Socket::new(Domain::IPV4, Type::DGRAM, Some(Protocol::UDP))?;
        s.set_reuse_address(true)?;
        #[cfg(unix)]
        s.set_reuse_port(true)?;
        s.set_broadcast(true)?;
        s.bind(&SocketAddr::V4(cfg.bind).into())?;
        Ok(Responder {
            sock: s.into(),
            cfg,
        })
    }

    pub fn local_addr(&self) -> io::Result<SocketAddr> {
        self.sock.local_addr()
    }

    /// A clone of the bound UDP socket (port 2222), e.g. for sending meter data from the
    /// console's port as a real WING does.
    pub fn try_clone_socket(&self) -> io::Result<UdpSocket> {
        self.sock.try_clone()
    }

    pub fn config(&self) -> &ResponderConfig {
        &self.cfg
    }

    /// The IP to advertise to a prober at `from`.
    pub fn advertised_ip(&self, from: Ipv4Addr) -> io::Result<Ipv4Addr> {
        if let Some(ip) = self.cfg.advertise_ip {
            return Ok(ip);
        }
        if !self.cfg.bind.ip().is_unspecified() {
            return Ok(*self.cfg.bind.ip());
        }
        local_ipv4_toward(from)
    }

    /// The reply to send for a datagram, or `None` if it is not a probe.
    pub fn reply_for(&self, datagram: &[u8], from: SocketAddr) -> io::Result<Option<String>> {
        let SocketAddr::V4(from) = from else {
            return Ok(None);
        };
        if !is_probe(datagram) {
            return Ok(None);
        }
        let ip = self.advertised_ip(*from.ip())?;
        Ok(Some(format_reply(ip, &self.cfg.identity)))
    }

    /// Receive one datagram (honouring `timeout`) and answer it if it is a probe. Returns the
    /// prober and the reply sent.
    pub fn serve_once(
        &self,
        timeout: Option<Duration>,
    ) -> io::Result<Option<(SocketAddr, String)>> {
        self.sock.set_read_timeout(timeout)?;
        let mut buf = [0u8; 512];
        let (n, from) = match self.sock.recv_from(&mut buf) {
            Ok(x) => x,
            Err(e)
                if matches!(
                    e.kind(),
                    io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
                ) =>
            {
                return Ok(None)
            }
            Err(e) => return Err(e),
        };
        match self.reply_for(&buf[..n], from)? {
            Some(reply) => {
                self.sock.send_to(reply.as_bytes(), from)?;
                Ok(Some((from, reply)))
            }
            None => Ok(None),
        }
    }

    /// Serve on a background thread until the handle is stopped or dropped. `on_reply` is
    /// called after each reply (for logging).
    pub fn spawn(
        self,
        mut on_reply: impl FnMut(SocketAddr, &str) + Send + 'static,
    ) -> io::Result<ResponderHandle> {
        let addr = self.local_addr()?;
        let stop = Arc::new(AtomicBool::new(false));
        let replies = Arc::new(AtomicU64::new(0));
        let (stop2, replies2) = (stop.clone(), replies.clone());
        let thread = std::thread::Builder::new()
            .name("wing-discovery".into())
            .spawn(move || {
                while !stop2.load(Ordering::Relaxed) {
                    match self.serve_once(Some(Duration::from_millis(100))) {
                        Ok(Some((from, reply))) => {
                            replies2.fetch_add(1, Ordering::Relaxed);
                            on_reply(from, &reply);
                        }
                        Ok(None) => {}
                        // Transient errors (e.g. no route to a prober) must not kill the
                        // responder; back off briefly.
                        Err(_) => std::thread::sleep(Duration::from_millis(50)),
                    }
                }
            })?;
        Ok(ResponderHandle {
            stop,
            replies,
            addr,
            thread: Some(thread),
        })
    }
}

/// Handle to a background responder; stops it on drop.
pub struct ResponderHandle {
    stop: Arc<AtomicBool>,
    replies: Arc<AtomicU64>,
    addr: SocketAddr,
    thread: Option<JoinHandle<()>>,
}

impl ResponderHandle {
    pub fn local_addr(&self) -> SocketAddr {
        self.addr
    }

    /// Number of replies sent so far.
    pub fn replies(&self) -> u64 {
        self.replies.load(Ordering::Relaxed)
    }

    /// Stop the thread and wait for it.
    pub fn stop(mut self) {
        self.shutdown();
    }

    fn shutdown(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

impl Drop for ResponderHandle {
    fn drop(&mut self) {
        self.shutdown();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reply_format() {
        let r = format_reply(Ipv4Addr::new(192, 168, 1, 19), &Identity::default());
        assert_eq!(
            r,
            "WING,192.168.1.19,WINGSIM,ngc-full,WSIM00000001,3.3.4-0-g0c2b9d4a:release"
        );
        let p = parse_reply(r.as_bytes()).unwrap();
        assert_eq!(p.ip, "192.168.1.19");
        assert_eq!(p.identity, Identity::default());
    }

    #[test]
    fn parses_libwing_example() {
        let p = parse_reply(
            b"WING,192.168.1.19,WING-PP-93892216,wing-compact,01XXXUX06X3AEX,3.0.5-0-g0c2b9d4a:release",
        )
        .unwrap();
        assert_eq!(p.identity.model, "wing-compact");
        assert_eq!(p.identity.firmware, "3.0.5-0-g0c2b9d4a:release");
        assert!(parse_reply(b"WING,1.2.3.4,x").is_none());
        assert!(parse_reply(b"XING,1,2,3,4,5").is_none());
    }

    #[test]
    fn probe_detection() {
        assert!(is_probe(b"WING?"));
        assert!(is_probe(b"WING?\0"));
        assert!(is_probe(b"WING?\r\n"));
        assert!(!is_probe(b"WING"));
        assert!(!is_probe(b"WING?x"));
        assert!(!is_probe(b""));
        assert!(!is_probe(b"wing?"));
    }

    #[test]
    fn identity_validation() {
        assert!(Identity::default().validate().is_ok());
        let mut id = Identity {
            name: "A,B".into(),
            ..Default::default()
        };
        assert!(id.validate().is_err());
        id.name = String::new();
        assert!(id.validate().is_err());
        id.name = "caf\u{e9}".into();
        assert!(id.validate().is_err());
    }

    #[test]
    fn advertise_override_and_loopback_route() {
        let cfg = ResponderConfig {
            bind: SocketAddrV4::new(Ipv4Addr::LOCALHOST, 0),
            advertise_ip: Some(Ipv4Addr::new(10, 9, 9, 9)),
            ..Default::default()
        };
        let r = Responder::bind(cfg).unwrap();
        let reply = r
            .reply_for(b"WING?", "127.0.0.1:5000".parse().unwrap())
            .unwrap()
            .unwrap();
        assert!(reply.starts_with("WING,10.9.9.9,WINGSIM,"));
        assert_eq!(
            r.reply_for(b"hello", "127.0.0.1:5000".parse().unwrap())
                .unwrap(),
            None
        );
        assert_eq!(
            local_ipv4_toward(Ipv4Addr::LOCALHOST).unwrap(),
            Ipv4Addr::LOCALHOST
        );
    }
}
