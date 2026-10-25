//! Protocol state of one native (TCP 2222) client connection. Pure: bytes in, reply bytes out,
//! effects on the [`ParamStore`]; no I/O, so replay tests drive it directly.
//!
//! Behaviour follows the decoded WING-Edit 3.3.4 session (docs/research/wing-edit-session.md):
//! - channel 1 is a cursor machine: `D7 h` / `DA` / `DB` / index / name tokens move the cursor,
//!   value tokens set the current node (no echo to the setter), `DC` answers `D7 h <value> DE`,
//!   `DD` answers definition records + `DE`, a client `DE` is ignored;
//! - an unknown hash (including `00000000`, ~1.4k per WING-Edit push) invalidates the cursor so
//!   the following value is dropped instead of landing on the previous node;
//! - channel 3 meter requests are parsed into [`MeterCommand`]s for the meter streamer;
//! - `$syscfg/$cnsmdl = "ModelRequest"` is answered with `D7 985729a2 <model>` (no `DE`) [H].

use wing_model::{NodeId, Origin, ParamStore};

use super::defs::{hidden, node_def, LiveInfo};
use super::values::{apply_value, click_value, step_value, with_value_token};
use crate::nrp::{MeterCommand, MeterRequestParser, NrpDecoder, NrpEncoder, Token};

/// Hash of `$syscfg/$cnsmdl` (console model; WING-Edit sets it to `ModelRequest`).
pub const HASH_CNSMDL: u32 = 0x9857_29a2;
/// The value WING-Edit writes to `$cnsmdl` to ask for the console model.
pub const MODEL_REQUEST: &[u8] = b"ModelRequest";

/// Per-connection counters.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SessionStats {
    /// Raw bytes received.
    pub bytes_in: u64,
    /// Raw bytes produced as replies (not counting fan-out events).
    pub bytes_out: u64,
    /// Value tokens received (sets), including dropped ones.
    pub sets: u64,
    /// Sets that changed the stored value.
    pub changed: u64,
    /// Sets rejected by the store (inactive model slot, type mismatch, ...).
    pub rejected: u64,
    /// Value tokens dropped because the cursor was invalid (unknown hash / hash 0).
    pub dropped: u64,
    /// `D7 00000000` selects.
    pub hash0: u64,
    /// Selects of hashes not in the schema (other than 0).
    pub unknown_hash: u64,
    /// `DC` requests.
    pub gets: u64,
    /// `DD` requests.
    pub defs: u64,
    /// `D8`/`D9` actions.
    pub actions: u64,
    /// `$cnsmdl` model requests answered.
    pub model_requests: u64,
    /// Channel-3 meter tokens received.
    pub meter_tokens: u64,
    /// Undecodable bytes (both channels).
    pub decode_errors: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Cursor {
    Node(NodeId),
    /// Unknown or zero hash selected: values are dropped, requests answered with a bare `DE`.
    Invalid(u32),
}

/// Session state other than the decoder (split so tokens can borrow the decoder).
struct State {
    origin: Origin,
    cursor: Cursor,
    enc: NrpEncoder,
    meter: MeterRequestParser,
    meter_cmds: Vec<MeterCommand>,
    stats: SessionStats,
    last_set: Option<NodeId>,
}

/// One client connection's protocol state.
pub struct Session {
    dec: NrpDecoder,
    st: State,
    live: std::sync::Arc<LiveInfo>,
}

impl Session {
    /// `origin` tags this client's sets so change fan-out skips it.
    pub fn new(origin: Origin, live: std::sync::Arc<LiveInfo>) -> Self {
        Session {
            dec: NrpDecoder::new(),
            st: State {
                origin,
                cursor: Cursor::Node(NodeId::ROOT),
                enc: NrpEncoder::new(),
                meter: MeterRequestParser::new(),
                meter_cmds: Vec::new(),
                stats: SessionStats::default(),
                last_set: None,
            },
            live,
        }
    }

    pub fn origin(&self) -> Origin {
        self.st.origin
    }

    pub fn stats(&self) -> SessionStats {
        self.st.stats
    }

    /// The node of the most recent set (for logging).
    pub fn last_set(&self) -> Option<NodeId> {
        self.st.last_set
    }

    /// Meter commands parsed since the last call.
    pub fn take_meter_commands(&mut self) -> std::vec::Drain<'_, MeterCommand> {
        self.st.meter_cmds.drain(..)
    }

    /// Process received wire bytes; replies are appended to `out` (escaped wire bytes).
    pub fn feed(&mut self, store: &ParamStore, wire: &[u8], out: &mut Vec<u8>) {
        let before = out.len();
        self.st.stats.bytes_in += wire.len() as u64;
        self.dec.feed(wire);
        while let Some(r) = self.dec.next_control() {
            match r {
                Ok(t) => self.st.handle(t, store, &self.live, out),
                Err(_) => self.st.stats.decode_errors += 1,
            }
        }
        while let Some(r) = self.dec.next_meter() {
            match r {
                Ok(t) => {
                    self.st.stats.meter_tokens += 1;
                    if let Some(c) = self.st.meter.feed(t) {
                        self.st.meter_cmds.push(c);
                    }
                }
                Err(_) => self.st.stats.decode_errors += 1,
            }
        }
        self.st.stats.bytes_out += (out.len() - before) as u64;
    }

    /// Append a change event `D7 <hash> <value>` for `id` (console -> client, no `DE`).
    pub fn encode_event(&mut self, store: &ParamStore, id: NodeId, out: &mut Vec<u8>) {
        self.st.value_msg(store, id, false, out);
    }
}

impl State {
    fn handle(&mut self, t: Token<'_>, store: &ParamStore, live: &LiveInfo, out: &mut Vec<u8>) {
        let s = store.schema();
        match t {
            Token::NavHash(0) => {
                self.stats.hash0 += 1;
                self.cursor = Cursor::Invalid(0);
            }
            Token::NavHash(h) => {
                self.cursor = match store.by_hash(h) {
                    Some(id) => Cursor::Node(id),
                    None => {
                        self.stats.unknown_hash += 1;
                        Cursor::Invalid(h)
                    }
                }
            }
            Token::Root => self.cursor = Cursor::Node(NodeId::ROOT),
            Token::Up => {
                if let Cursor::Node(id) = self.cursor {
                    self.cursor = Cursor::Node(s.parent(id).unwrap_or(NodeId::ROOT));
                }
            }
            Token::NavIndex(i) => {
                if let Cursor::Node(id) = self.cursor {
                    let by_name = s.child(id, &i.to_string());
                    let by_pos = || s.children(id).nth(i as usize - 1);
                    self.cursor = match by_name.or_else(by_pos) {
                        Some(c) => Cursor::Node(c),
                        None => Cursor::Invalid(0),
                    };
                }
            }
            Token::NavName(n) => {
                if let Cursor::Node(id) = self.cursor {
                    let name = String::from_utf8_lossy(n);
                    let found = s.child(id, &name).or_else(|| {
                        let mut p = String::new();
                        let _ = s.write_path(id, &mut p);
                        store.lookup(&format!("{p}/{name}"))
                    });
                    self.cursor = match found {
                        Some(c) => Cursor::Node(c),
                        None => Cursor::Invalid(0),
                    };
                }
            }
            t if t.is_value() => {
                self.stats.sets += 1;
                let Cursor::Node(id) = self.cursor else {
                    self.stats.dropped += 1;
                    return;
                };
                if s.hash(id) == HASH_CNSMDL {
                    // console-local and read-only: answer a model request, ignore anything else
                    if t == Token::Str(MODEL_REQUEST) {
                        self.stats.model_requests += 1;
                        self.enc
                            .control(&Token::NavHash(HASH_CNSMDL), out)
                            .expect("hash");
                        let m = live.console_model.as_bytes();
                        let _ = self.enc.control(&Token::Str(&m[..m.len().min(256)]), out);
                    }
                    return;
                }
                self.count(apply_value(store, id, &t, self.origin), id);
            }
            Token::Click | Token::Step(_) => {
                self.stats.actions += 1;
                if let Cursor::Node(id) = self.cursor {
                    let r = match t {
                        Token::Step(n) => step_value(store, id, n, self.origin),
                        _ => click_value(store, id, self.origin),
                    };
                    self.count(r, id);
                }
            }
            Token::DataRequest => {
                self.stats.gets += 1;
                match self.cursor {
                    Cursor::Node(id) if s.is_param(id) => self.value_msg(store, id, true, out),
                    Cursor::Node(id) => {
                        // node dump [H]: every active parameter below `id`, then DE (this is
                        // what `DA DC`, WING-Edit's MIXER -> PC sync request, receives)
                        self.dump(store, id, 0, out);
                        self.end(out);
                    }
                    Cursor::Invalid(_) => self.end(out),
                }
            }
            Token::DefRequest => {
                self.stats.defs += 1;
                let node = match self.cursor {
                    Cursor::Node(id) => Some(id),
                    // libwing uses id 0 for the root
                    Cursor::Invalid(0) => Some(NodeId::ROOT),
                    Cursor::Invalid(_) => None,
                };
                if let Some(id) = node {
                    if s.is_param(id) {
                        // dynamic lists (`play/$songs`, `$ctl/lib/$scenes`): their own record
                        let _ = self.enc.node_def(&node_def(store, id, live), out);
                    } else {
                        for c in s.children(id) {
                            if !hidden(store, c) {
                                let _ = self.enc.node_def(&node_def(store, c, live), out);
                            }
                        }
                    }
                }
                self.end(out);
            }
            // client DE (`D7 h DE DD`, `D3 port DE`) and stray records: ignored
            _ => {}
        }
    }

    fn count(&mut self, r: Result<bool, wing_model::SetError>, id: NodeId) {
        match r {
            Ok(true) => self.stats.changed += 1,
            Ok(false) => {}
            Err(_) => self.stats.rejected += 1,
        }
        self.last_set = Some(id);
    }

    fn end(&mut self, out: &mut Vec<u8>) {
        self.enc.control(&Token::End, out).expect("DE");
    }

    fn value_msg(&mut self, store: &ParamStore, id: NodeId, end: bool, out: &mut Vec<u8>) {
        let h = store.schema().hash(id);
        self.enc.control(&Token::NavHash(h), out).expect("hash");
        let enc = &mut self.enc;
        with_value_token(store, id, |t| {
            let _ = enc.control(&t, out);
        });
        if end {
            self.end(out);
        }
    }

    /// Preorder dump of the active parameters below `id`. Like WING-Edit's own push, subtrees
    /// whose name starts with `$` below the first level (live/read-only values) are skipped.
    fn dump(&mut self, store: &ParamStore, id: NodeId, depth: u32, out: &mut Vec<u8>) {
        let s = store.schema();
        for c in s.children(id) {
            if depth > 0 && s.name(c).starts_with('$') && id != NodeId::ROOT {
                continue;
            }
            if s.is_param(c) {
                if !store.is_inactive(c) {
                    self.value_msg(store, c, false, out);
                }
            } else {
                self.dump(store, c, depth + 1, out);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::nrp::NrpDecoder;
    use std::sync::Arc;
    use wing_model::Value;

    fn wire(tokens: &[Token]) -> Vec<u8> {
        let mut e = NrpEncoder::new();
        let mut v = Vec::new();
        for t in tokens {
            e.control(t, &mut v).unwrap();
        }
        v
    }

    fn replies(out: &[u8]) -> Vec<String> {
        let mut d = NrpDecoder::new();
        d.feed(&[0xDF, 0xD1]); // the session's encoder only selects ch1 once
        d.feed(out);
        let mut v = Vec::new();
        while let Some(t) = d.next_control() {
            v.push(format!("{:?}", t.unwrap()));
        }
        v
    }

    #[test]
    fn set_get_and_hash0() {
        let st = ParamStore::new();
        let mut s = Session::new(7, Arc::new(LiveInfo::default()));
        let fdr = st.lookup("ch/1/fdr").unwrap();
        let h = st.schema().hash(fdr);
        let mut out = Vec::new();
        s.feed(
            &st,
            &wire(&[
                Token::NavHash(h),
                Token::Float(-5.0),
                Token::NavHash(0),
                Token::Float(3.0), // dropped, must not land on ch/1/fdr
                Token::NavHash(h),
                Token::DataRequest,
            ]),
            &mut out,
        );
        assert_eq!(st.get(fdr), Some(Value::Float(-5.0)));
        assert_eq!(
            replies(&out),
            [
                format!("NavHash({h})"),
                "Float(-5.0)".to_string(),
                "End".to_string()
            ]
        );
        let stats = s.stats();
        assert_eq!(
            (stats.sets, stats.dropped, stats.hash0, stats.gets),
            (2, 1, 1, 1)
        );
    }

    #[test]
    fn model_request_and_defs() {
        let st = ParamStore::new();
        let mut s = Session::new(7, Arc::new(LiveInfo::default()));
        let mut out = Vec::new();
        s.feed(
            &st,
            &wire(&[Token::NavHash(HASH_CNSMDL), Token::Str(MODEL_REQUEST)]),
            &mut out,
        );
        assert_eq!(
            replies(&out),
            [
                format!("NavHash({HASH_CNSMDL})"),
                format!("Str({:?})", b"wing-fullsize")
            ]
        );
        out.clear();
        // root definitions: 16 root children + DE
        s.feed(&st, &wire(&[Token::Root, Token::DefRequest]), &mut out);
        let r = replies(&out);
        assert_eq!(r.len(), st.schema().children(NodeId::ROOT).count() + 1);
        assert_eq!(r.last().unwrap(), "End");
    }
}
