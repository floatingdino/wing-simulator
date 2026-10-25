//! UDP socket, receive thread and event-push thread for the OSC service.
//!
//! Event flow: [`OscAttachment::attach`] registers a `ParamStore` hook (before the store is
//! shared) that, while a subscriber is live, queues the `Change` on a channel. The event thread
//! formats and sends it, so setters never block on socket I/O.

use std::io;
use std::net::{Ipv4Addr, SocketAddr, SocketAddrV4, UdpSocket};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{channel, Receiver, RecvTimeoutError, Sender};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use socket2::{Domain, Protocol, Socket, Type};
use wing_model::{Change, ParamStore};

use super::codec::MAX_PACKET;
use super::service::{OscService, SubState, DEFAULT_SUB_TIMEOUT};
use crate::discovery::Identity;

/// The WING OSC port.
pub const OSC_PORT: u16 = 2223;

#[derive(Clone, Debug)]
pub struct OscConfig {
    /// Address to bind (default `0.0.0.0:2223`; use port 0 in tests).
    pub bind: SocketAddrV4,
    /// Identity reported by `/?`.
    pub identity: Identity,
    /// IP reported by `/?` instead of the auto-detected one.
    pub advertise_ip: Option<Ipv4Addr>,
    /// Subscription lifetime without renewal.
    pub sub_timeout: Duration,
}

impl Default for OscConfig {
    fn default() -> Self {
        OscConfig {
            bind: SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, OSC_PORT),
            identity: Identity::default(),
            advertise_ip: None,
            sub_timeout: DEFAULT_SUB_TIMEOUT,
        }
    }
}

/// Subscription state plus the change feed; created before the store is shared.
pub struct OscAttachment {
    subs: Arc<SubState>,
    rx: Receiver<Change>,
}

impl OscAttachment {
    /// Register the change hook on `store` (call before wrapping it in an `Arc`).
    pub fn attach(store: &mut ParamStore) -> OscAttachment {
        let subs = Arc::new(SubState::default());
        let (tx, rx): (Sender<Change>, _) = channel();
        let tx = Mutex::new(tx);
        let hook_subs = subs.clone();
        store.add_hook(Box::new(move |c: &Change| {
            if let Some(sub) = hook_subs.current(Instant::now()) {
                if sub.origin != c.origin {
                    let _ = tx.lock().unwrap_or_else(|e| e.into_inner()).send(*c);
                }
            }
        }));
        OscAttachment { subs, rx }
    }
}

pub struct OscServer {
    sock: UdpSocket,
    svc: Arc<OscService>,
    rx: Receiver<Change>,
}

impl OscServer {
    pub fn bind(
        store: Arc<ParamStore>,
        att: OscAttachment,
        cfg: OscConfig,
    ) -> io::Result<OscServer> {
        cfg.identity
            .validate()
            .map_err(|e| io::Error::new(io::ErrorKind::InvalidInput, e))?;
        let s = Socket::new(Domain::IPV4, Type::DGRAM, Some(Protocol::UDP))?;
        s.set_reuse_address(true)?;
        s.bind(&SocketAddr::V4(cfg.bind).into())?;
        let svc = Arc::new(OscService::new(
            store,
            cfg.identity,
            cfg.advertise_ip,
            cfg.sub_timeout,
            att.subs,
        ));
        Ok(OscServer {
            sock: s.into(),
            svc,
            rx: att.rx,
        })
    }

    pub fn local_addr(&self) -> io::Result<SocketAddr> {
        self.sock.local_addr()
    }

    pub fn service(&self) -> &Arc<OscService> {
        &self.svc
    }

    /// Serve on background threads until the handle is stopped or dropped.
    pub fn spawn(self) -> io::Result<OscHandle> {
        let addr = self.local_addr()?;
        let stop = Arc::new(AtomicBool::new(false));
        let requests = Arc::new(AtomicU64::new(0));
        let ev_sock = self.sock.try_clone()?;
        self.sock
            .set_read_timeout(Some(Duration::from_millis(100)))?;

        let (stop_a, req_a, svc_a, sock_a) =
            (stop.clone(), requests.clone(), self.svc.clone(), self.sock);
        let rx_thread = std::thread::Builder::new()
            .name("wing-osc".into())
            .spawn(move || {
                let mut buf = vec![0u8; 65536];
                while !stop_a.load(Ordering::Relaxed) {
                    let (n, from) = match sock_a.recv_from(&mut buf) {
                        Ok(x) => x,
                        Err(e)
                            if matches!(
                                e.kind(),
                                io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
                            ) =>
                        {
                            continue
                        }
                        Err(_) => {
                            std::thread::sleep(Duration::from_millis(50));
                            continue;
                        }
                    };
                    if n > MAX_PACKET {
                        continue;
                    }
                    req_a.fetch_add(1, Ordering::Relaxed);
                    for r in svc_a.handle(&buf[..n], from, Instant::now()) {
                        let _ = sock_a.send_to(&r.msg.to_vec(), r.to);
                    }
                }
            })?;

        let (stop_b, svc_b, rx) = (stop.clone(), self.svc, self.rx);
        let ev_thread = std::thread::Builder::new()
            .name("wing-osc-events".into())
            .spawn(move || {
                while !stop_b.load(Ordering::Relaxed) {
                    match rx.recv_timeout(Duration::from_millis(50)) {
                        Ok(c) => {
                            let Some(sub) = svc_b.subs().current(Instant::now()) else {
                                continue;
                            };
                            if sub.origin == c.origin {
                                continue;
                            }
                            if let Some(m) = svc_b.event_message(c.node(), sub.mode) {
                                let _ = ev_sock.send_to(&m.to_vec(), sub.addr);
                            }
                        }
                        Err(RecvTimeoutError::Timeout) => {}
                        Err(RecvTimeoutError::Disconnected) => break,
                    }
                }
            })?;

        Ok(OscHandle {
            stop,
            requests,
            addr,
            threads: vec![rx_thread, ev_thread],
        })
    }
}

/// Handle to a running server; stops it on drop.
pub struct OscHandle {
    stop: Arc<AtomicBool>,
    requests: Arc<AtomicU64>,
    addr: SocketAddr,
    threads: Vec<JoinHandle<()>>,
}

impl OscHandle {
    pub fn local_addr(&self) -> SocketAddr {
        self.addr
    }

    /// Datagrams received so far.
    pub fn requests(&self) -> u64 {
        self.requests.load(Ordering::Relaxed)
    }

    pub fn stop(mut self) {
        self.shutdown();
    }

    fn shutdown(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        for t in self.threads.drain(..) {
            let _ = t.join();
        }
    }
}

impl Drop for OscHandle {
    fn drop(&mut self) {
        self.shutdown();
    }
}
