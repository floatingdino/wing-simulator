//! OSC protocol logic over a [`ParamStore`], independent of sockets.
//!
//! Follows `docs/research/protocol.md` section 5. Where that section is silent, this module
//! chooses (and tests) the following:
//!
//! - **Values.** A `,f` set is the *engineering* value (dB, Hz, ...; the native `D5` form), a `,i`
//!   set is an int / enum index, a `,s` set is display or snapshot text / enum name. The store
//!   converts between kinds as the console does. The normalised 0..1 position is only *reported*
//!   (second element of a get triplet). Enum gets return the enum token as `,s`.
//! - **Errors.** Failed gets/sets reply `/* ,s "<ERROR>"` with the documented strings
//!   (`NODE NOT FOUND`, `VALUE ERROR`, `NODE IS NOT PAR`, `INCOMPLETE DATA`, `STACK EMPTY`,
//!   `BUFFER OVERFLOW`). Successful leaf sets are not echoed.
//! - **Node dumps.** `,s "*"` returns `rel.path=value,` for every parameter under the node,
//!   `,s "?"` returns `name definition` entries joined with `~ ` for the direct children, `,s "#"`
//!   the same with `=value` appended. The exact console text of `?`/`#` is unverified.
//! - **Events.** `/*s` pushes the get triplet at the parameter's address, `/*S` a single
//!   native-typed argument (float = engineering value, int = int/enum index, string), `/*b` a
//!   blob at `/` holding the native token `D7 <hash> <value token>`. A change made by the
//!   subscriber itself is not echoed back to it. `$`-prefixed shadow nodes are not emitted.
//! - Blob passthrough of native tokens to the console (`/ ,b ...`) is not implemented.
//! - **Scene library (T017/T065).** With a [`SceneLibrary`] configured ([`OscService::with_library`])
//!   a set of `/$ctl/lib/$action` (`,s` token / simulator extension such as `STORE:Verse`, or `,i`
//!   enum index) goes to the library with the current `$ctl/lib/$actionidx`, exactly like the
//!   native protocol; sets of the other console-owned `$ctl/lib/$*` leaves are dropped.
//! - **Simulator extensions under `/wingsim/` (T065, for DAW integration).** Not WING addresses;
//!   successful commands are not acknowledged (the library runs them on its worker thread and
//!   logs the result); without a library they answer `NODE NOT FOUND`:
//!   - `/wingsim/scene/recall ,s name` (or `#N`, 0-based) / `,i N`: recall a scene.
//!   - `/wingsim/scene/store` [`,s name`]: store the current state (no name / empty: overwrite the
//!     active scene, else append `Scene N`).
//!   - `/wingsim/scene/next`, `/wingsim/scene/prev`: recall the scene after/before the active one
//!     (a numeric argument of 0, a button release, is ignored).
//!   - `/wingsim/scenes` (no args): reply `/wingsim/scenes ,s...` with the scene names.
//!   - `/wingsim/ab/{store_a,store_b,toggle,a,b}` (T062): A/B compare. `store_a`/`store_b` capture
//!     the current state into a slot (and make it the active side); `a`/`b`/`toggle` switch sides,
//!     applying only the parameters that differ through the normal set path (RT-staged, fanned
//!     out to native/OSC clients). A numeric argument of 0 (button release) is ignored.
//!   - `/wingsim/export ,s path`: write the current state as a console-ready 3.x `.snap` at
//!     `path` plus `<path stem>.changes.md` (wing-diff report vs the scene as loaded).
//!   - `/wingsim/marker ,s text`, `/wingsim/region ,s text`: a DAW timeline marker/region name
//!     (REAPER's `LAST_MARKER_NAME`/`LAST_REGION_NAME` feedback). `WING:<scene or #N>` (prefix
//!     case-insensitive) recalls that scene; other names are ignored. A name equal to the
//!     previous one on the same address is ignored, so feedback refreshes never re-recall.

use std::net::{Ipv4Addr, SocketAddr};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use wing_model::{Desc, NodeId};
use wing_model::{Origin, ParamStore, ParamType, SetError, Value};

use super::codec::{Arg, Message, MAX_PACKET};
use crate::discovery::{format_reply, local_ipv4_toward, Identity};
use crate::server::library::{lib_ids, SceneLibrary};

/// Subscription lifetime without renewal (the console documents "< 10 s").
pub const DEFAULT_SUB_TIMEOUT: Duration = Duration::from_secs(10);

/// Event format requested by a subscriber.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SubMode {
    /// `/*s`: `,sff` / `,sfi` / `,s` triplets.
    Text,
    /// `/*S`: one native-typed argument.
    Native,
    /// `/*b`: native token blobs.
    Blob,
}

#[derive(Clone, Copy, Debug)]
pub struct Sub {
    pub addr: SocketAddr,
    pub mode: SubMode,
    pub expires: Instant,
    pub origin: Origin,
}

/// The single event subscriber (last requester wins).
#[derive(Default)]
pub struct SubState(Mutex<Option<Sub>>);

impl SubState {
    pub fn set(&self, s: Sub) {
        *self.0.lock().unwrap_or_else(|e| e.into_inner()) = Some(s);
    }

    /// The subscriber if still live at `now` (an expired one is dropped).
    pub fn current(&self, now: Instant) -> Option<Sub> {
        let mut g = self.0.lock().unwrap_or_else(|e| e.into_inner());
        match *g {
            Some(s) if s.expires > now => Some(s),
            Some(_) => {
                *g = None;
                None
            }
            None => None,
        }
    }
}

/// A datagram to send.
#[derive(Debug, Clone, PartialEq)]
pub struct Reply {
    pub to: SocketAddr,
    pub msg: Message,
}

pub struct OscService {
    store: Arc<ParamStore>,
    identity: Identity,
    advertise_ip: Option<Ipv4Addr>,
    sub_timeout: Duration,
    subs: Arc<SubState>,
    library: Option<Arc<dyn SceneLibrary>>,
    /// Last `/wingsim/marker` and `/wingsim/region` text (de-duplication).
    last_marks: Mutex<[String; 2]>,
}

/// Origin tag for sets coming from `addr` (so hooks can skip echoing to the setter).
pub fn origin_of(addr: SocketAddr) -> Origin {
    let mut h: u32 = 0x811c_9dc5;
    let mut feed = |b: u8| h = (h ^ b as u32).wrapping_mul(0x0100_0193);
    match addr.ip() {
        std::net::IpAddr::V4(a) => a.octets().iter().for_each(|&b| feed(b)),
        std::net::IpAddr::V6(a) => a.octets().iter().for_each(|&b| feed(b)),
    }
    addr.port().to_be_bytes().iter().for_each(|&b| feed(b));
    h | 0x8000_0000
}

impl OscService {
    pub fn new(
        store: Arc<ParamStore>,
        identity: Identity,
        advertise_ip: Option<Ipv4Addr>,
        sub_timeout: Duration,
        subs: Arc<SubState>,
    ) -> Self {
        OscService {
            store,
            identity,
            advertise_ip,
            sub_timeout,
            subs,
            library: None,
            last_marks: Mutex::default(),
        }
    }

    /// Route `$ctl/lib/$action` and the `/wingsim/scene...` commands to a scene library.
    pub fn with_library(mut self, library: Option<Arc<dyn SceneLibrary>>) -> Self {
        self.library = library;
        self
    }

    pub fn store(&self) -> &Arc<ParamStore> {
        &self.store
    }

    pub fn subs(&self) -> &Arc<SubState> {
        &self.subs
    }

    /// Handle one received datagram; returns the datagrams to send.
    pub fn handle(&self, datagram: &[u8], from: SocketAddr, now: Instant) -> Vec<Reply> {
        let mut out = Vec::new();
        if let Ok(m) = Message::decode(datagram) {
            self.handle_msg(&m, from, now, &mut out);
        }
        out.retain(|r| r.msg.to_vec().len() <= MAX_PACKET);
        out
    }

    fn handle_msg(&self, m: &Message, from: SocketAddr, now: Instant, out: &mut Vec<Reply>) {
        let (redirect, addr) = split_prefix(&m.addr);
        let to = match redirect {
            Some(p) => SocketAddr::new(from.ip(), p),
            None => from,
        };
        let mut reply = |addr: &str, args: Vec<Arg>| {
            out.push(Reply {
                to,
                msg: Message::new(addr, args),
            })
        };
        let err = |e: &str| vec![Arg::Str(e.into())];

        match addr {
            "/?" => {
                let ip = self.advertise_ip.or_else(|| match from {
                    SocketAddr::V4(a) => local_ipv4_toward(*a.ip()).ok(),
                    _ => None,
                });
                let ip = ip.unwrap_or(Ipv4Addr::LOCALHOST);
                reply("/?", vec![Arg::Str(format_reply(ip, &self.identity))]);
            }
            "/*s" | "/*S" | "/*b" => {
                let mode = match addr {
                    "/*s" => SubMode::Text,
                    "/*S" => SubMode::Native,
                    _ => SubMode::Blob,
                };
                self.subs.set(Sub {
                    addr: to,
                    mode,
                    expires: now + self.sub_timeout,
                    origin: origin_of(from),
                });
            }
            a if a.starts_with("/wingsim/") => {
                if let Some(r) = self.sim_command(a, m.args.first(), origin_of(from)) {
                    let (addr, args) = r;
                    reply(&addr, args);
                }
            }
            _ => {
                let Some(id) = self.resolve(addr) else {
                    reply("/*", err("NODE NOT FOUND"));
                    return;
                };
                let s = self.store.schema();
                let origin = origin_of(from);
                if s.is_param(id) {
                    match m.args.first() {
                        None => match self.leaf_args(id, SubMode::Text) {
                            Some(a) => reply(addr, a),
                            None => reply("/*", err("NODE NOT FOUND")),
                        },
                        Some(a) if self.library_leaf(id, a, origin) => {}
                        Some(a) => {
                            if let Err(e) = self.set_leaf(id, a, origin) {
                                reply("/*", err(e));
                            }
                        }
                    }
                } else {
                    match m.args.first() {
                        None => {
                            let names = s.children(id).map(|c| Arg::Str(s.name(c).into()));
                            reply(addr, names.collect());
                        }
                        Some(Arg::Str(cmd)) => match cmd.as_str() {
                            "*" => match self.dump(id, false) {
                                Ok(t) => reply(addr, vec![Arg::Str(t)]),
                                Err(e) => reply("/*", err(e)),
                            },
                            "?" | "#" => match self.defs(id, cmd == "#") {
                                Ok(t) => reply(addr, vec![Arg::Str(t)]),
                                Err(e) => reply("/*", err(e)),
                            },
                            text => {
                                let ack = format!("/{}*", addr.trim_matches('/'));
                                match self.node_set(id, text, origin) {
                                    Ok(()) => reply(&ack, vec![Arg::Str("OK".into())]),
                                    Err(e) => reply(&ack, err(e)),
                                }
                            }
                        },
                        Some(_) => reply("/*", err("NODE IS NOT PAR")),
                    }
                }
            }
        }
    }

    /// Console-owned `$ctl/lib` leaf sets with a library configured: `$action` goes to the
    /// library, the others are dropped. Returns whether the set was handled here.
    fn library_leaf(&self, id: NodeId, a: &Arg, origin: Origin) -> bool {
        let (Some(lib), Some(ids)) = (&self.library, lib_ids(&self.store)) else {
            return false;
        };
        if self.store.schema().parent(id) != Some(ids.lib) || id == ids.actionidx {
            return false;
        }
        if id == ids.action {
            let text = match a {
                Arg::Str(t) => Some(t.trim().to_string()),
                Arg::Int(_) | Arg::Float(_) => {
                    let n = match *a {
                        Arg::Float(f) => f as i32,
                        Arg::Int(i) => i,
                        _ => unreachable!(),
                    };
                    let items = self.store.desc(id).map(|d| d.enum_items()).unwrap_or(&[]);
                    usize::try_from(n)
                        .ok()
                        .and_then(|n| items.get(n))
                        .map(|e| e.token.to_string())
                }
                Arg::Blob(_) => None,
            };
            if let Some(t) = text.filter(|t| !t.is_empty() && t != "IDLE") {
                let idx = match self.store.get(ids.actionidx) {
                    Some(Value::Int(n)) => n,
                    _ => 0,
                };
                lib.action(&t, idx, origin);
            }
        }
        true
    }

    /// A `/wingsim/...` simulator command (see the module docs); returns the reply, if any.
    fn sim_command(
        &self,
        addr: &str,
        arg: Option<&Arg>,
        origin: Origin,
    ) -> Option<(String, Vec<Arg>)> {
        let err = |e: &str| Some(("/*".to_string(), vec![Arg::Str(e.into())]));
        let Some(lib) = &self.library else {
            return err("NODE NOT FOUND");
        };
        let num = |a: &Arg| match *a {
            Arg::Int(i) => Some(i),
            Arg::Float(f) => Some(f as i32),
            _ => None,
        };
        let text = |a: Option<&Arg>| match a {
            Some(Arg::Str(t)) => Some(t.trim().to_string()),
            _ => None,
        };
        match addr.trim_end_matches('/') {
            "/wingsim/scenes" => {
                let names = lib.scenes().into_iter().map(Arg::Str).collect();
                return Some(("/wingsim/scenes".into(), names));
            }
            "/wingsim/scene/recall" => match arg {
                Some(Arg::Str(t)) if !t.trim().is_empty() => {
                    lib.action(&format!("GO:{}", t.trim()), 0, origin)
                }
                Some(a) if num(a).is_some() => lib.action("GO", num(a).unwrap_or(0), origin),
                _ => return err("INCOMPLETE DATA"),
            },
            "/wingsim/scene/store" => match text(arg) {
                Some(t) if !t.is_empty() => lib.action(&format!("STORE:{t}"), 0, origin),
                _ => lib.action("STORE", 0, origin),
            },
            "/wingsim/scene/next" | "/wingsim/scene/prev" => {
                if arg.and_then(num) == Some(0) {
                    return None;
                }
                let a = if addr.contains("next") {
                    "GONEXT"
                } else {
                    "GOPREV"
                };
                lib.action(a, 0, origin);
            }
            "/wingsim/ab/store_a"
            | "/wingsim/ab/store_b"
            | "/wingsim/ab/toggle"
            | "/wingsim/ab/a"
            | "/wingsim/ab/b" => {
                if arg.and_then(num) == Some(0) {
                    return None; // button release
                }
                let cmd = addr
                    .trim_end_matches('/')
                    .trim_start_matches("/wingsim/ab/");
                lib.action(&format!("AB:{cmd}"), 0, origin);
            }
            "/wingsim/export" => match text(arg) {
                Some(p) if !p.is_empty() => lib.action(&format!("EXPORT:{p}"), 0, origin),
                _ => return err("INCOMPLETE DATA"),
            },
            "/wingsim/marker" | "/wingsim/region" => {
                let t = text(arg)?;
                let slot = usize::from(addr.contains("region"));
                {
                    let mut last = self.last_marks.lock().unwrap_or_else(|e| e.into_inner());
                    if last[slot] == t {
                        return None;
                    }
                    last[slot] = t.clone();
                }
                if let Some(target) = marker_target(&t) {
                    lib.action(&format!("GO:{target}"), 0, origin);
                }
            }
            _ => return err("NODE NOT FOUND"),
        }
        None
    }

    /// Address (path or `#hash`) to node.
    fn resolve(&self, addr: &str) -> Option<NodeId> {
        let p = addr.trim_matches('/');
        if let Some(h) = p.strip_prefix('#') {
            return self.store.by_hash(u32::from_str_radix(h, 16).ok()?);
        }
        self.store.lookup(p)
    }

    fn set_leaf(&self, id: NodeId, a: &Arg, origin: Origin) -> Result<(), &'static str> {
        let r = match a {
            Arg::Float(f) => self.store.set_from(id, Value::Float(*f), origin),
            Arg::Int(i) => self.store.set_from(id, Value::Int(*i), origin),
            Arg::Str(t) => self.store.set_text(id, t, origin),
            Arg::Blob(_) => return Err("VALUE ERROR"),
        };
        r.map(|_| ()).map_err(|e| match e {
            SetError::NotParam => "NODE IS NOT PAR",
            _ => "VALUE ERROR",
        })
    }

    /// `/ch/1 ,s "fdr=3,mute=0"`-style node set (see protocol.md).
    fn node_set(&self, start: NodeId, text: &str, origin: Origin) -> Result<(), &'static str> {
        let s = self.store.schema();
        let mut stack = vec![start];
        while let Some(p) = s.parent(*stack.last().unwrap()) {
            stack.push(p);
        }
        stack.reverse(); // ROOT .. start
        for item in text.split(',') {
            let mut item = item.trim();
            if item.is_empty() {
                continue;
            }
            if let Some(r) = item.strip_prefix('/') {
                stack.truncate(1);
                item = r;
            }
            while let Some(r) = item.strip_prefix('.') {
                if stack.len() <= 1 {
                    return Err("STACK EMPTY");
                }
                stack.pop();
                item = r;
            }
            let (lhs, rhs) = match item.split_once('=') {
                Some((l, r)) => (l, Some(r)),
                None => (item, None),
            };
            let mut segs: Vec<&str> = lhs.split('.').filter(|x| !x.is_empty()).collect();
            let last = if rhs.is_some() { segs.pop() } else { None };
            for seg in segs {
                let id = self.child_of(*stack.last().unwrap(), seg)?;
                if s.is_param(id) {
                    return Err("INCOMPLETE DATA");
                }
                stack.push(id);
            }
            if let (Some(name), Some(val)) = (last, rhs) {
                let id = self.child_of(*stack.last().unwrap(), name)?;
                if !s.is_param(id) {
                    return Err("NODE IS NOT PAR");
                }
                self.store
                    .set_text(id, val, origin)
                    .map_err(|_| "VALUE ERROR")?;
            }
        }
        Ok(())
    }

    fn child_of(&self, parent: NodeId, name: &str) -> Result<NodeId, &'static str> {
        let base = self.store.schema().path(parent);
        let path = if base.is_empty() {
            name.to_string()
        } else {
            format!("{base}/{name}")
        };
        match self.store.lookup(&path) {
            // model aliases resolve to a node under the parent; reject anything else
            Some(id) if self.store.schema().parent(id) == Some(parent) => Ok(id),
            _ => Err("NODE NOT FOUND"),
        }
    }

    fn dump(&self, id: NodeId, _values: bool) -> Result<String, &'static str> {
        use std::fmt::Write;
        let s = self.store.schema();
        let mut out = String::new();
        let mut stack: Vec<(NodeId, String)> = vec![(id, String::new())];
        while let Some((n, rel)) = stack.pop() {
            let mut kids: Vec<_> = s.children(n).collect();
            kids.reverse();
            for c in kids {
                let name = if rel.is_empty() {
                    s.name(c).to_string()
                } else {
                    format!("{rel}.{}", s.name(c))
                };
                if s.is_param(c) {
                    if self.store.desc(c).is_none_or(|d| d.unused_slot) {
                        continue;
                    }
                    let _ = write!(out, "{name}=");
                    let _ = self.store.format_token(c, &mut out);
                    out.push(',');
                    if out.len() > MAX_PACKET - 256 {
                        return Err("BUFFER OVERFLOW");
                    }
                } else {
                    stack.push((c, name));
                }
            }
        }
        Ok(out)
    }

    fn defs(&self, id: NodeId, with_values: bool) -> Result<String, &'static str> {
        use std::fmt::Write;
        let s = self.store.schema();
        let mut parts = Vec::new();
        let mut len = 0;
        for c in s.children(id) {
            let mut t = String::new();
            let _ = write!(t, "{} ", s.name(c));
            match self.store.desc(c) {
                None => t.push_str("node"),
                Some(d) => {
                    t.push_str(&def_text(d));
                    if with_values {
                        t.push_str(" = ");
                        let _ = self.store.format_token(c, &mut t);
                    }
                }
            }
            len += t.len() + 2;
            if len > MAX_PACKET - 256 {
                return Err("BUFFER OVERFLOW");
            }
            parts.push(t);
        }
        Ok(parts.join("~ "))
    }

    /// Value arguments of a leaf for the given event/get format. `None` for containers and
    /// inactive slots.
    pub fn leaf_args(&self, id: NodeId, mode: SubMode) -> Option<Vec<Arg>> {
        let st = &*self.store;
        let d = st.desc(id)?;
        let raw = st.get_raw(id);
        let native = mode == SubMode::Native;
        let mut text = String::new();
        let _ = st.format_token(id, &mut text);
        Some(match d.ty {
            ParamType::Str => vec![Arg::Str(text)],
            ParamType::Enum if native => vec![Arg::Int(raw as i32)],
            ParamType::Enum => vec![Arg::Str(text)],
            ParamType::Int if native => vec![Arg::Int(raw as i32)],
            ParamType::Int => {
                let mut disp = String::new();
                let _ = d.format(raw, &mut disp);
                vec![
                    Arg::Str(disp),
                    Arg::Float(d.to_norm(raw)),
                    Arg::Int(raw as i32),
                ]
            }
            _ => {
                let v = d.as_f64(raw) as f32;
                if native {
                    vec![Arg::Float(v)]
                } else {
                    let mut disp = String::new();
                    let _ = d.format(raw, &mut disp);
                    vec![Arg::Str(disp), Arg::Float(d.to_norm(raw)), Arg::Float(v)]
                }
            }
        })
    }

    /// The event message pushed to a subscriber for a changed node.
    pub fn event_message(&self, id: NodeId, mode: SubMode) -> Option<Message> {
        let s = self.store.schema();
        match mode {
            SubMode::Text | SubMode::Native => {
                let args = self.leaf_args(id, mode)?;
                Some(Message::new(format!("/{}", s.path(id)), args))
            }
            SubMode::Blob => {
                let d = self.store.desc(id)?;
                let mut b = vec![0xD7];
                b.extend_from_slice(&s.hash(id).to_be_bytes());
                match self.leaf_args(id, SubMode::Native)?.into_iter().next()? {
                    Arg::Float(f) => {
                        b.push(0xD5);
                        b.extend_from_slice(&f.to_be_bytes());
                    }
                    Arg::Int(i) => push_int(&mut b, i),
                    Arg::Str(t) => push_str(&mut b, &t),
                    Arg::Blob(_) => return None,
                }
                let _ = d;
                Some(Message::new("/", vec![Arg::Blob(b)]))
            }
        }
    }
}

fn push_int(b: &mut Vec<u8>, i: i32) {
    if (0..64).contains(&i) {
        b.push(i as u8);
    } else if let Ok(v) = i16::try_from(i) {
        b.push(0xD3);
        b.extend_from_slice(&v.to_be_bytes());
    } else {
        b.push(0xD4);
        b.extend_from_slice(&i.to_be_bytes());
    }
}

fn push_str(b: &mut Vec<u8>, t: &str) {
    let t = &t.as_bytes()[..t.len().min(256)];
    match t.len() {
        0 => b.push(0xD0),
        1..=64 => b.push(0x7F + t.len() as u8),
        n => {
            b.push(0xD1);
            b.push((n - 1) as u8);
        }
    }
    b.extend_from_slice(t);
}

fn def_text(d: &Desc) -> String {
    let unit = d.unit.as_str();
    let range = |kind: &str| {
        let u = if unit.is_empty() {
            String::new()
        } else {
            format!(" {unit}")
        };
        format!("{kind} [{} .. {}{u}], {} steps", d.min, d.max, d.steps)
    };
    match d.ty {
        ParamType::LinF => range("lin"),
        ParamType::LogF => range("log"),
        ParamType::Fader => range("fader"),
        ParamType::Int => format!("int [{} .. {}]", d.min, d.max),
        ParamType::Str => format!("str [maxlen {}]", d.maxlen),
        ParamType::Enum | ParamType::FEnum => {
            let items = d.enum_items();
            let mut l: Vec<&str> = items.iter().take(16).map(|e| e.token).collect();
            if items.len() > 16 {
                l.push("...");
            }
            let kind = if d.ty == ParamType::Enum {
                "list"
            } else {
                "flist"
            };
            format!("{kind} [{}]", l.join(", "))
        }
    }
}

/// The scene named by a `WING:<scene or #N>` marker (prefix case-insensitive), if any.
pub fn marker_target(name: &str) -> Option<&str> {
    let n = name.trim();
    let head = n.get(..5)?;
    if !head.eq_ignore_ascii_case("WING:") {
        return None;
    }
    let t = n[5..].trim();
    (!t.is_empty()).then_some(t)
}

/// Split an optional `/%<port>` prefix: `/%9000/ch/1` -> (Some(9000), `/ch/1`).
fn split_prefix(addr: &str) -> (Option<u16>, &str) {
    if let Some(rest) = addr.strip_prefix("/%") {
        let n = rest.bytes().take_while(u8::is_ascii_digit).count();
        if n > 0 {
            if let Ok(p) = rest[..n].parse::<u16>() {
                let tail = &rest[n..];
                return (Some(p), if tail.is_empty() { "/" } else { tail });
            }
        }
    }
    (None, addr)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn svc() -> OscService {
        OscService::new(
            Arc::new(ParamStore::new()),
            Identity::default(),
            Some(Ipv4Addr::new(10, 0, 0, 1)),
            Duration::from_secs(10),
            Arc::new(SubState::default()),
        )
    }

    fn from() -> SocketAddr {
        "127.0.0.1:40000".parse().unwrap()
    }

    fn ask(s: &OscService, addr: &str, args: Vec<Arg>) -> Vec<Reply> {
        s.handle(&Message::new(addr, args).to_vec(), from(), Instant::now())
    }

    #[test]
    fn prefix_split() {
        assert_eq!(split_prefix("/%9000/ch/1"), (Some(9000), "/ch/1"));
        assert_eq!(split_prefix("/%9000"), (Some(9000), "/"));
        assert_eq!(split_prefix("/ch/1"), (None, "/ch/1"));
    }

    #[test]
    fn get_fader_triplet() {
        let s = svc();
        let r = ask(&s, "/ch/1/fdr", vec![]);
        assert_eq!(r.len(), 1);
        assert_eq!(r[0].msg.addr, "/ch/1/fdr");
        let a = &r[0].msg.args;
        assert!(matches!(&a[0], Arg::Str(_)));
        assert!(matches!(a[1], Arg::Float(_)));
        assert!(matches!(a[2], Arg::Float(_)));
    }

    #[test]
    fn set_then_get_fader_and_mute() {
        let s = svc();
        assert!(ask(&s, "/ch/1/fdr", vec![Arg::Float(-3.0)]).is_empty());
        let r = ask(&s, "/ch/1/fdr", vec![]);
        assert_eq!(r[0].msg.args[2], Arg::Float(-3.0));
        assert_eq!(r[0].msg.args[0], Arg::Str("-3.0".into()));

        // out-of-range dB (REAPER TRACK_VOLUME_DB at -inf / +12 dB) clamps (T065)
        assert!(ask(&s, "/ch/1/fdr", vec![Arg::Float(-150.0)]).is_empty());
        assert_eq!(
            ask(&s, "/ch/1/fdr", vec![])[0].msg.args[2],
            Arg::Float(-144.0)
        );
        assert!(ask(&s, "/ch/1/fdr", vec![Arg::Float(12.0)]).is_empty());
        assert_eq!(
            ask(&s, "/ch/1/fdr", vec![])[0].msg.args[2],
            Arg::Float(10.0)
        );
        ask(&s, "/ch/1/fdr", vec![Arg::Float(-3.0)]);

        ask(&s, "/ch/1/mute", vec![Arg::Int(1)]);
        let r = ask(&s, "/ch/1/mute", vec![]);
        assert_eq!(r[0].msg.args[0], Arg::Str("1".into()));
        assert_eq!(r[0].msg.args[1], Arg::Float(1.0));
        assert_eq!(r[0].msg.args[2], Arg::Int(1));
    }

    #[test]
    fn hash_address_equals_path() {
        let s = svc();
        ask(&s, "/#f50f69f8", vec![Arg::Int(1)]);
        assert_eq!(ask(&s, "/ch/1/mute", vec![])[0].msg.args[2], Arg::Int(1));
    }

    #[test]
    fn unknown_node_errors() {
        let s = svc();
        let r = ask(&s, "/nope/x", vec![]);
        assert_eq!(r[0].msg.addr, "/*");
        assert_eq!(r[0].msg.args, vec![Arg::Str("NODE NOT FOUND".into())]);
    }

    #[test]
    fn node_get_lists_children_and_node_set_works() {
        let s = svc();
        let r = ask(&s, "/ch/1", vec![]);
        assert!(r[0].msg.args.contains(&Arg::Str("fdr".into())));
        let r = ask(&s, "/ch/1", vec![Arg::Str("fdr=-6,mute=1".into())]);
        assert_eq!(r[0].msg.addr, "/ch/1*");
        assert_eq!(r[0].msg.args, vec![Arg::Str("OK".into())]);
        assert_eq!(ask(&s, "/ch/1/mute", vec![])[0].msg.args[2], Arg::Int(1));
        let r = ask(
            &s,
            "/",
            vec![Arg::Str("/ch.1.fdr=-1,mute=0,.2.fdr=0,mute=1".into())],
        );
        assert_eq!(r[0].msg.addr, "/*");
        assert_eq!(r[0].msg.args, vec![Arg::Str("OK".into())]);
        assert_eq!(ask(&s, "/ch/1/mute", vec![])[0].msg.args[2], Arg::Int(0));
        assert_eq!(ask(&s, "/ch/2/mute", vec![])[0].msg.args[2], Arg::Int(1));
        let r = ask(&s, "/ch/1", vec![Arg::Str("bogus=1".into())]);
        assert_eq!(r[0].msg.args, vec![Arg::Str("NODE NOT FOUND".into())]);
        let r = ask(&s, "/", vec![Arg::Str(".fdr=1".into())]);
        assert_eq!(r[0].msg.args, vec![Arg::Str("STACK EMPTY".into())]);
        let r = ask(&s, "/ch/1", vec![Arg::Str("fdr=abc".into())]);
        assert_eq!(r[0].msg.args, vec![Arg::Str("VALUE ERROR".into())]);
    }

    #[test]
    fn dumps() {
        let s = svc();
        let r = ask(&s, "/ch/1/eq", vec![Arg::Str("*".into())]);
        let Arg::Str(t) = &r[0].msg.args[0] else {
            panic!()
        };
        assert!(t.contains("=") && t.ends_with(','), "{t}");
        let r = ask(&s, "/ch/1/eq", vec![Arg::Str("?".into())]);
        let Arg::Str(t) = &r[0].msg.args[0] else {
            panic!()
        };
        assert!(t.contains("~ "), "{t}");
    }

    #[test]
    fn info_reply() {
        let s = svc();
        let r = ask(&s, "/?", vec![]);
        let Arg::Str(t) = &r[0].msg.args[0] else {
            panic!()
        };
        assert!(t.starts_with("WING,10.0.0.1,"), "{t}");
    }

    #[test]
    fn redirect_prefix_changes_reply_port() {
        let s = svc();
        let r = ask(&s, "/%9001/ch/1/mute", vec![]);
        assert_eq!(r[0].to.port(), 9001);
        assert_eq!(r[0].msg.addr, "/ch/1/mute");
    }

    #[test]
    fn blob_event_tokens() {
        let s = svc();
        let id = s.store.lookup("ch/1/mute").unwrap();
        s.store.set(id, Value::Int(1)).unwrap();
        let m = s.event_message(id, SubMode::Blob).unwrap();
        assert_eq!(
            m.args,
            vec![Arg::Blob(vec![0xD7, 0xF5, 0x0F, 0x69, 0xF8, 0x01])]
        );
    }

    #[derive(Default)]
    struct FakeLib(Mutex<Vec<(String, i32)>>);

    impl SceneLibrary for FakeLib {
        fn scenes(&self) -> Vec<String> {
            vec!["Intro".into(), "Verse".into()]
        }
        fn action(&self, action: &str, index: i32, _origin: Origin) {
            self.0.lock().unwrap().push((action.into(), index));
        }
    }

    fn lib_svc() -> (OscService, Arc<FakeLib>) {
        let lib = Arc::new(FakeLib::default());
        (svc().with_library(Some(lib.clone())), lib)
    }

    fn take(l: &FakeLib) -> Vec<(String, i32)> {
        std::mem::take(&mut *l.0.lock().unwrap())
    }

    #[test]
    fn lib_action_leaf_routes_to_library() {
        let (s, l) = lib_svc();
        assert!(ask(&s, "/$ctl/lib/$actionidx", vec![Arg::Int(1)]).is_empty());
        assert!(ask(&s, "/$ctl/lib/$action", vec![Arg::Str("GO".into())]).is_empty());
        ask(&s, "/$ctl/lib/$action", vec![Arg::Int(2)]); // enum index: GONEXT
        ask(
            &s,
            "/$ctl/lib/$action",
            vec![Arg::Str("STORE:Verse".into())],
        );
        ask(&s, "/$ctl/lib/$action", vec![Arg::Str("IDLE".into())]);
        ask(&s, "/$ctl/lib/$active", vec![Arg::Str("Hacked".into())]); // dropped
        assert_eq!(
            take(&l),
            [
                ("GO".into(), 1),
                ("GONEXT".into(), 1),
                ("STORE:Verse".into(), 1)
            ]
        );
        let r = ask(&s, "/$ctl/lib/$active", vec![]);
        assert_eq!(r[0].msg.args, vec![Arg::Str(String::new())]);
        // without a library the leaves are plain parameters (T014 behaviour)
        let plain = svc();
        ask(&plain, "/$ctl/lib/$action", vec![Arg::Str("GO".into())]);
        let r = ask(&plain, "/$ctl/lib/$action", vec![]);
        assert_eq!(r[0].msg.args, vec![Arg::Str("GO".into())]);
    }

    #[test]
    fn wingsim_scene_commands() {
        let (s, l) = lib_svc();
        let st = |t: &str| vec![Arg::Str(t.into())];
        assert!(ask(&s, "/wingsim/scene/recall", st(" Verse ")).is_empty());
        ask(&s, "/wingsim/scene/recall", st("#0"));
        ask(&s, "/wingsim/scene/recall", vec![Arg::Int(1)]);
        ask(&s, "/wingsim/scene/store", st("Chorus"));
        ask(&s, "/wingsim/scene/store", vec![]);
        ask(&s, "/wingsim/scene/next", vec![]);
        ask(&s, "/wingsim/scene/next", vec![Arg::Float(0.0)]); // button release
        ask(&s, "/wingsim/scene/prev", vec![Arg::Float(1.0)]);
        assert_eq!(
            take(&l),
            [
                ("GO:Verse".into(), 0),
                ("GO:#0".into(), 0),
                ("GO".into(), 1),
                ("STORE:Chorus".into(), 0),
                ("STORE".into(), 0),
                ("GONEXT".into(), 0),
                ("GOPREV".into(), 0),
            ]
        );
        let r = ask(&s, "/wingsim/scene/recall", vec![]);
        assert_eq!(r[0].msg.args, st("INCOMPLETE DATA"));
        let r = ask(&s, "/wingsim/bogus", vec![]);
        assert_eq!(r[0].msg.args, st("NODE NOT FOUND"));
        let r = ask(&s, "/wingsim/scenes", vec![]);
        assert_eq!(r[0].msg.addr, "/wingsim/scenes");
        assert_eq!(r[0].msg.args, [st("Intro"), st("Verse")].concat());
        // no library: not found
        let r = ask(&svc(), "/wingsim/scene/next", vec![]);
        assert_eq!(r[0].msg.args, st("NODE NOT FOUND"));
    }

    #[test]
    fn wingsim_ab_and_export() {
        let (s, l) = lib_svc();
        let st = |t: &str| vec![Arg::Str(t.into())];
        for c in ["store_a", "store_b", "toggle", "a", "b"] {
            assert!(ask(&s, &format!("/wingsim/ab/{c}"), vec![]).is_empty());
        }
        ask(&s, "/wingsim/ab/toggle", vec![Arg::Float(0.0)]); // button release
        ask(&s, "/wingsim/ab/toggle", vec![Arg::Float(1.0)]);
        ask(&s, "/wingsim/export", st("/tmp/mix.snap"));
        assert_eq!(
            take(&l),
            [
                ("AB:store_a".into(), 0),
                ("AB:store_b".into(), 0),
                ("AB:toggle".into(), 0),
                ("AB:a".into(), 0),
                ("AB:b".into(), 0),
                ("AB:toggle".into(), 0),
                ("EXPORT:/tmp/mix.snap".into(), 0),
            ]
        );
        let r = ask(&s, "/wingsim/export", vec![]);
        assert_eq!(r[0].msg.args, st("INCOMPLETE DATA"));
        let r = ask(&s, "/wingsim/ab/bogus", vec![]);
        assert_eq!(r[0].msg.args, st("NODE NOT FOUND"));
    }

    #[test]
    fn wingsim_markers() {
        let (s, l) = lib_svc();
        let mark = |a: &str, t: &str| ask(&s, a, vec![Arg::Str(t.into())]);
        mark("/wingsim/marker", "WING:Intro");
        mark("/wingsim/marker", "WING:Intro"); // feedback refresh: ignored
        mark("/wingsim/marker", "Guitar solo"); // not ours
        mark("/wingsim/marker", "wing: #1 ");
        mark("/wingsim/region", "WING:Intro"); // separate slot
        mark("/wingsim/marker", "");
        mark("/wingsim/marker", "WING:Intro"); // crossed again after leaving it
        mark("/wingsim/marker", "WING:");
        assert_eq!(
            take(&l),
            [
                ("GO:Intro".into(), 0),
                ("GO:#1".into(), 0),
                ("GO:Intro".into(), 0),
                ("GO:Intro".into(), 0),
            ]
        );
        assert_eq!(marker_target("WING:Verse"), Some("Verse"));
        assert_eq!(marker_target("Wing: Verse 2 "), Some("Verse 2"));
        assert_eq!(marker_target("WINGS"), None);
        assert_eq!(marker_target("é"), None);
    }
}
