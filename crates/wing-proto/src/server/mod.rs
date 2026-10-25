//! Native protocol server (T013, TCP 2222) and meter transport (T015).
//!
//! - [`session::Session`]: per-connection protocol state (pure; replay-testable).
//! - [`Hub`]: change fan-out. Installed as a `ParamStore` hook (before the store is shared), it
//!   marks changed node ids in each other client's coalescing outbox; client threads then send
//!   `D7 <hash> <current value>` events. Changes from OSC or the simulator itself (any origin
//!   that is not the client) reach every client.
//! - [`NativeServer`]: listener + one thread per client (std blocking sockets, short read
//!   timeout to interleave events), max clients, inactivity timeout, clean shutdown.
//! - [`meters`]: channel-3 subscriptions, the UDP meter streamer and the meter bank.
//! - [`library`]: the `$ctl/lib` show/scene library hook ([`SceneLibrary`], T017).
//!
//! Spec: docs/research/wing-edit-session.md (decoded WING-Edit 3.3.4 session) and
//! docs/research/protocol.md §2-§4.

pub mod defs;
pub mod library;
pub mod meters;
pub mod session;
pub mod values;

use std::io::{self, Read, Write};
use std::net::{Ipv4Addr, SocketAddr, SocketAddrV4, TcpListener, TcpStream};
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicUsize, Ordering::*};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use socket2::{Domain, Protocol, Socket, Type};
use wing_model::{Change, NodeId, Origin, ParamStore};

pub use defs::LiveInfo;
pub use library::SceneLibrary;
pub use meters::{MeterBank, MeterStreamer, MeterSubscriptions, SyntheticMeters};
pub use session::{Session, SessionStats};

/// Origins of native clients are `ORIGIN_NATIVE_BASE + client id` (high bit clear; OSC origins
/// have the high bit set, 0 is the simulator itself).
pub const ORIGIN_NATIVE_BASE: Origin = 0x4E52_0000;
/// Native protocol TCP port.
pub const NATIVE_PORT: u16 = 2222;

// ---- fan-out ------------------------------------------------------------------------------

/// A client's pending change set: each node at most once, values read when sent (coalescing).
struct Outbox {
    origin: Origin,
    inner: Mutex<OutboxInner>,
}

struct OutboxInner {
    ids: Vec<u16>,
    marked: Vec<u64>,
}

impl Outbox {
    fn new(origin: Origin, nodes: usize) -> Self {
        Outbox {
            origin,
            inner: Mutex::new(OutboxInner {
                ids: Vec::new(),
                marked: vec![0; nodes.div_ceil(64)],
            }),
        }
    }

    fn mark(&self, id: u16) {
        let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        let (w, b) = (id as usize / 64, 1u64 << (id % 64));
        if let Some(m) = g.marked.get_mut(w) {
            if *m & b == 0 {
                *m |= b;
                g.ids.push(id);
            }
        }
    }

    /// Move pending ids into `out` (cleared first).
    fn drain(&self, out: &mut Vec<u16>) {
        out.clear();
        let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        std::mem::swap(&mut g.ids, out);
        for &id in out.iter() {
            g.marked[id as usize / 64] &= !(1u64 << (id % 64));
        }
    }
}

/// Change fan-out to connected native clients.
pub struct Hub {
    clients: Mutex<Vec<Arc<Outbox>>>,
    nodes: usize,
}

impl Hub {
    /// Create a hub and install its change hook on `store` (call before sharing the store; other
    /// hooks such as `OscAttachment::attach` coexist).
    pub fn attach(store: &mut ParamStore) -> Arc<Hub> {
        let hub = Arc::new(Hub {
            clients: Mutex::new(Vec::new()),
            nodes: store.schema().node_count(),
        });
        let h = hub.clone();
        store.add_hook(Box::new(move |c: &Change| h.on_change(c)));
        hub
    }

    fn on_change(&self, c: &Change) {
        let g = self.clients.lock().unwrap_or_else(|e| e.into_inner());
        for o in g.iter() {
            if o.origin != c.origin {
                o.mark(c.id);
            }
        }
    }

    fn register(&self, origin: Origin) -> Arc<Outbox> {
        let o = Arc::new(Outbox::new(origin, self.nodes));
        self.clients
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .push(o.clone());
        o
    }

    fn unregister(&self, o: &Arc<Outbox>) {
        self.clients
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .retain(|x| !Arc::ptr_eq(x, o));
    }
}

// ---- server -------------------------------------------------------------------------------

/// Server configuration.
#[derive(Clone, Debug)]
pub struct ServerConfig {
    /// Listen address (default `0.0.0.0:2222`; port 0 in tests).
    pub bind: SocketAddrV4,
    /// Further connections are closed immediately (fw 3.1.2: 24).
    pub max_clients: usize,
    /// Close a connection after this long without receiving anything (console: 10 s; channel-3
    /// meter renews count). `None` disables.
    pub idle_timeout: Option<Duration>,
    /// Quiet time after which a run of sets is reported as one [`ServerEvent::Sets`].
    pub set_report_gap: Duration,
    pub live: LiveInfo,
}

impl Default for ServerConfig {
    fn default() -> Self {
        ServerConfig {
            bind: SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, NATIVE_PORT),
            max_clients: 24,
            idle_timeout: Some(Duration::from_secs(10)),
            set_report_gap: Duration::from_millis(300),
            live: LiveInfo::default(),
        }
    }
}

/// Server notifications (for logging).
#[derive(Clone, Debug)]
pub enum ServerEvent {
    Connected {
        client: u32,
        peer: SocketAddr,
    },
    /// Connection refused because `max_clients` are connected.
    Refused {
        peer: SocketAddr,
    },
    /// A run of sets from a client ended (quiet for `set_report_gap`).
    Sets {
        client: u32,
        count: u64,
        changed: u64,
        total: u64,
        last: Option<NodeId>,
    },
    /// The client asked for the console model and got `model`.
    ModelRequest {
        client: u32,
        model: String,
    },
    /// A meter report was defined/renewed for the first time, or the UDP port was set.
    Meter {
        client: u32,
        what: String,
    },
    Disconnected {
        client: u32,
        peer: SocketAddr,
        reason: String,
        stats: SessionStats,
    },
}

/// Callback for [`ServerEvent`]s (called from server threads).
pub type EventFn = Arc<dyn Fn(&ServerEvent) + Send + Sync>;

/// Shared state given to the server.
#[derive(Clone)]
pub struct ServerShared {
    pub store: Arc<ParamStore>,
    pub hub: Arc<Hub>,
    /// Meter subscriptions (channel 3); `None` ignores meter requests.
    pub meters: Option<Arc<MeterSubscriptions>>,
}

/// Native protocol server (not yet listening).
pub struct NativeServer {
    listener: TcpListener,
    cfg: ServerConfig,
    shared: ServerShared,
    on_event: EventFn,
}

impl NativeServer {
    /// Bind the listener (SO_REUSEADDR, no SO_REUSEPORT: a second server on the port fails).
    pub fn bind(cfg: ServerConfig, shared: ServerShared, on_event: EventFn) -> io::Result<Self> {
        let s = Socket::new(Domain::IPV4, Type::STREAM, Some(Protocol::TCP))?;
        s.set_reuse_address(true)?;
        s.bind(&SocketAddr::V4(cfg.bind).into())?;
        s.listen(32)?;
        let listener: TcpListener = s.into();
        listener.set_nonblocking(true)?;
        Ok(NativeServer {
            listener,
            cfg,
            shared,
            on_event,
        })
    }

    pub fn local_addr(&self) -> io::Result<SocketAddr> {
        self.listener.local_addr()
    }

    /// Accept and serve on background threads until the handle is stopped or dropped.
    pub fn spawn(self) -> io::Result<ServerHandle> {
        let addr = self.local_addr()?;
        let stop = Arc::new(AtomicBool::new(false));
        let active = Arc::new(AtomicUsize::new(0));
        let (stop2, active2) = (stop.clone(), active.clone());
        let thread = std::thread::Builder::new()
            .name("wing-native-accept".into())
            .spawn(move || self.accept_loop(stop2, active2))?;
        Ok(ServerHandle {
            stop,
            active,
            addr,
            thread: Some(thread),
        })
    }

    fn accept_loop(self, stop: Arc<AtomicBool>, active: Arc<AtomicUsize>) {
        let next_id = AtomicU32::new(1);
        let live = Arc::new(self.cfg.live.clone());
        let mut clients: Vec<JoinHandle<()>> = Vec::new();
        while !stop.load(Relaxed) {
            match self.listener.accept() {
                Ok((stream, peer)) => {
                    clients.retain(|t| !t.is_finished());
                    if active.load(SeqCst) >= self.cfg.max_clients {
                        (self.on_event)(&ServerEvent::Refused { peer });
                        drop(stream);
                        continue;
                    }
                    let id = next_id.fetch_add(1, Relaxed);
                    active.fetch_add(1, SeqCst);
                    let ctx = ClientCtx {
                        id,
                        peer,
                        cfg: self.cfg.clone(),
                        shared: self.shared.clone(),
                        on_event: self.on_event.clone(),
                        stop: stop.clone(),
                        live: live.clone(),
                    };
                    let active3 = active.clone();
                    match std::thread::Builder::new()
                        .name(format!("wing-native-{id}"))
                        .spawn(move || {
                            ctx.run(stream);
                            active3.fetch_sub(1, SeqCst);
                        }) {
                        Ok(t) => clients.push(t),
                        Err(_) => {
                            active.fetch_sub(1, SeqCst);
                        }
                    }
                }
                Err(e) if e.kind() == io::ErrorKind::WouldBlock => {
                    std::thread::sleep(Duration::from_millis(20));
                }
                Err(_) => std::thread::sleep(Duration::from_millis(50)),
            }
        }
        for t in clients {
            let _ = t.join();
        }
    }
}

struct ClientCtx {
    id: u32,
    peer: SocketAddr,
    cfg: ServerConfig,
    shared: ServerShared,
    on_event: EventFn,
    stop: Arc<AtomicBool>,
    live: Arc<LiveInfo>,
}

impl ClientCtx {
    fn run(self, mut stream: TcpStream) {
        let origin = ORIGIN_NATIVE_BASE.wrapping_add(self.id);
        let outbox = self.shared.hub.register(origin);
        (self.on_event)(&ServerEvent::Connected {
            client: self.id,
            peer: self.peer,
        });
        let mut session = Session::new(origin, self.live.clone());
        let reason = self.serve(&mut stream, &mut session, &outbox);
        self.shared.hub.unregister(&outbox);
        if let Some(m) = &self.shared.meters {
            m.remove_client(self.id);
        }
        let _ = stream.shutdown(std::net::Shutdown::Both);
        (self.on_event)(&ServerEvent::Disconnected {
            client: self.id,
            peer: self.peer,
            reason,
            stats: session.stats(),
        });
    }

    fn serve(&self, stream: &mut TcpStream, session: &mut Session, outbox: &Outbox) -> String {
        let store = &*self.shared.store;
        let setup = || -> io::Result<()> {
            stream.set_nodelay(true)?;
            stream.set_read_timeout(Some(Duration::from_millis(10)))?;
            stream.set_write_timeout(Some(Duration::from_secs(5)))?;
            socket2::SockRef::from(&*stream).set_keepalive(true)
        };
        if let Err(e) = setup() {
            return format!("socket setup: {e}");
        }
        let mut buf = vec![0u8; 64 * 1024];
        let mut out = Vec::with_capacity(64 * 1024);
        let mut ids = Vec::new();
        let mut last_rx = Instant::now();
        let mut last_set_at = Instant::now();
        let (mut reported_sets, mut reported_changed, mut reported_models) = (0u64, 0u64, 0u64);
        let mut meter_port_logged = false;
        let mut meter_ids_logged: Vec<u32> = Vec::new();
        loop {
            if self.stop.load(Relaxed) {
                return "server shutdown".into();
            }
            match stream.read(&mut buf) {
                Ok(0) => return "closed by client".into(),
                Ok(n) => {
                    last_rx = Instant::now();
                    let before = session.stats().sets;
                    session.feed(store, &buf[..n], &mut out);
                    if session.stats().sets != before {
                        last_set_at = last_rx;
                    }
                    let ip = self.peer.ip();
                    let cmds: Vec<_> = session.take_meter_commands().collect();
                    for c in &cmds {
                        if let Some(m) = &self.shared.meters {
                            m.apply(self.id, ip, c, last_rx);
                        }
                        let what = match c {
                            crate::nrp::MeterCommand::SetPort(p) if !meter_port_logged => {
                                meter_port_logged = true;
                                Some(format!("udp port {p}"))
                            }
                            crate::nrp::MeterCommand::Define { id, selections }
                                if !meter_ids_logged.contains(id) =>
                            {
                                meter_ids_logged.push(*id);
                                let n: usize = selections.iter().map(|s| s.indices.len()).sum();
                                Some(format!("report {id} defined ({n} meters)"))
                            }
                            _ => None,
                        };
                        if let Some(what) = what {
                            (self.on_event)(&ServerEvent::Meter {
                                client: self.id,
                                what,
                            });
                        }
                    }
                }
                Err(e)
                    if matches!(
                        e.kind(),
                        io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
                    ) => {}
                Err(e) if e.kind() == io::ErrorKind::Interrupted => {}
                Err(e) => return format!("read error: {e}"),
            }
            outbox.drain(&mut ids);
            for &id in &ids {
                session.encode_event(store, NodeId(id), &mut out);
            }
            if !out.is_empty() {
                if let Err(e) = stream.write_all(&out) {
                    return format!("write error: {e}");
                }
                out.clear();
            }
            let st = session.stats();
            if st.model_requests != reported_models {
                reported_models = st.model_requests;
                (self.on_event)(&ServerEvent::ModelRequest {
                    client: self.id,
                    model: self.live.console_model.clone(),
                });
            }
            if st.sets != reported_sets && last_set_at.elapsed() >= self.cfg.set_report_gap {
                (self.on_event)(&ServerEvent::Sets {
                    client: self.id,
                    count: st.sets - reported_sets,
                    changed: st.changed - reported_changed,
                    total: st.sets,
                    last: session.last_set(),
                });
                reported_sets = st.sets;
                reported_changed = st.changed;
            }
            if let Some(t) = self.cfg.idle_timeout {
                if last_rx.elapsed() > t {
                    return format!("idle for {:.1} s", t.as_secs_f64());
                }
            }
        }
    }
}

/// Handle to a running server; stops it (closing all client connections) on drop.
pub struct ServerHandle {
    stop: Arc<AtomicBool>,
    active: Arc<AtomicUsize>,
    addr: SocketAddr,
    thread: Option<JoinHandle<()>>,
}

impl ServerHandle {
    pub fn local_addr(&self) -> SocketAddr {
        self.addr
    }

    /// Connected clients.
    pub fn clients(&self) -> usize {
        self.active.load(SeqCst)
    }

    pub fn stop(mut self) {
        self.shutdown();
    }

    fn shutdown(&mut self) {
        self.stop.store(true, Relaxed);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

impl Drop for ServerHandle {
    fn drop(&mut self) {
        self.shutdown();
    }
}
