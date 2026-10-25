//! Protocol state of one native (TCP 2222) client connection. Pure: bytes in, reply bytes out,
//! effects on the [`ParamStore`]; no I/O, so replay tests drive it directly.
//!
//! Behaviour follows the decoded WING-Edit 3.3.4 session (docs/research/wing-edit-session.md):
//! - channel 1 is a cursor machine: `D7 h` / `DA` / `DB` / index / name tokens move the cursor,
//!   value tokens set the current node, `DC` answers `D7 h <value> DE`,
//!   `DD` answers definition records + `DE`, a client `DE` is ignored;
//! - an unknown hash (including `00000000`, ~1.4k per WING-Edit push) invalidates the cursor so
//!   the following value is dropped instead of landing on the previous node;
//! - **echo to the setter** (T083): change fan-out skips the client that made the change, so the
//!   session itself confirms the result to its client as a `D7 h <value>` event when the client
//!   cannot know it: after every `D8` click / `D9` step (WING-Edit toggles strip mutes with `D8`
//!   and only redraws the button from the console's reply), and after a value set the console
//!   changed (clamped, rounded beyond 1 %, converted to another form such as an enum index to
//!   its name, or a `D6` normalised value). Sets stored as sent are not echoed (WING-Edit
//!   updates its own UI for those; its 29k-set scene push needs no reply).
//! - channel 3 meter requests are parsed into [`MeterCommand`]s for the meter streamer;
//! - `$syscfg/$cnsmdl = "ModelRequest"` is answered with `D7 985729a2 <model>` (no `DE`) [H];
//! - with a scene library configured, `$ctl/lib/$action` sets go to it (see [`super::library`]).

use wing_model::{NodeId, Origin, ParamStore, Value};

use super::defs::{hidden, node_def, LiveInfo};
use super::library::{action_text, lib_ids};
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
    /// Values confirmed back to this client after its own click/step/adjusted set (T083).
    pub echoes: u64,
    /// `$cnsmdl` model requests answered.
    pub model_requests: u64,
    /// `$ctl/lib/$action` requests passed to the scene library (T017).
    pub library_actions: u64,
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
                if let (Some(lib), Some(ids)) = (&live.library, lib_ids(store)) {
                    if s.parent(id) == Some(ids.lib) && id != ids.actionidx {
                        // console-owned library state (T017): `$action` goes to the library,
                        // client sets of `$scenes`/`$active`/`$actshow`/... are dropped
                        let text = (id == ids.action)
                            .then(|| action_text(store, id, &t))
                            .flatten();
                        match text {
                            Some(a) if a != "IDLE" => {
                                let idx = match store.get(ids.actionidx) {
                                    Some(Value::Int(n)) => n,
                                    _ => 0,
                                };
                                self.stats.library_actions += 1;
                                lib.action(&a, idx, self.origin);
                            }
                            Some(_) => {}
                            None => self.stats.rejected += 1,
                        }
                        self.last_set = Some(id);
                        return;
                    }
                }
                let r = apply_value(store, id, &t, self.origin);
                if r.is_ok() && adjusted(store, id, &t) {
                    self.echo(store, id, out);
                }
                self.count(r, id);
            }
            Token::Click | Token::Step(_) => {
                self.stats.actions += 1;
                if let Cursor::Node(id) = self.cursor {
                    let r = match t {
                        Token::Step(n) => step_value(store, id, n, self.origin),
                        _ => click_value(store, id, self.origin),
                    };
                    if r.is_ok() {
                        self.echo(store, id, out);
                    }
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

    /// Confirm `id`'s current value to this client (`D7 h <value>`, an event, no `DE`).
    fn echo(&mut self, store: &ParamStore, id: NodeId, out: &mut Vec<u8>) {
        if store.desc(id).is_some() && !store.is_inactive(id) {
            self.stats.echoes += 1;
            self.value_msg(store, id, false, out);
        }
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

/// Whether the value stored for `id` differs from what the client sent in `sent` (so the client
/// must be told): numbers beyond 1 % (min 0.01 absolute, so grid rounding is not reported),
/// strings byte for byte, any change of form (number <-> name) and every `D6` normalised value.
fn adjusted(store: &ParamStore, id: NodeId, sent: &Token<'_>) -> bool {
    fn num(t: &Token<'_>) -> Option<f64> {
        match *t {
            Token::Float(v) => Some(v as f64),
            _ => t.as_int().map(|n| n as f64),
        }
    }
    with_value_token(store, id, |cur| match (num(sent), num(&cur), sent, &cur) {
        (Some(a), Some(b), _, _) => (a - b).abs() > 0.01 * a.abs().max(1.0),
        (_, _, Token::Str(a), Token::Str(b)) => a != b,
        _ => true,
    })
    .unwrap_or(false)
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

    #[derive(Default)]
    struct MockLib(std::sync::Mutex<Vec<(String, i32)>>);

    impl crate::server::SceneLibrary for MockLib {
        fn scenes(&self) -> Vec<String> {
            vec!["Intro".into(), "Verse".into()]
        }
        fn action(&self, action: &str, index: i32, _origin: Origin) {
            self.0.lock().unwrap().push((action.into(), index));
        }
    }

    #[test]
    fn library_actions_are_routed() {
        let st = ParamStore::new();
        let lib = Arc::new(MockLib::default());
        let live = LiveInfo {
            library: Some(lib.clone()),
            ..LiveInfo::default()
        };
        let mut s = Session::new(7, Arc::new(live));
        let h = |p| st.schema().hash(st.lookup(p).unwrap());
        let mut out = Vec::new();
        s.feed(
            &st,
            &wire(&[
                Token::NavHash(h("$ctl/lib/$actshow")),
                Token::Str(b"PC SHOW"), // console-owned: dropped
                Token::NavHash(h("$ctl/lib/$actionidx")),
                Token::SmallInt(1),
                Token::NavHash(h("$ctl/lib/$action")),
                Token::Str(b"GO"),
                Token::SmallInt(5), // enum index 5 = NEXT
                Token::Str(b"STORE:Chorus"),
                Token::Str(b"IDLE"), // no-op
                Token::NavHash(h("$ctl/lib/$scenes")),
                Token::End,
                Token::DefRequest,
            ]),
            &mut out,
        );
        assert_eq!(
            *lib.0.lock().unwrap(),
            [
                ("GO".to_string(), 1),
                ("NEXT".to_string(), 1),
                ("STORE:Chorus".to_string(), 1)
            ]
        );
        assert_eq!(s.stats().library_actions, 3);
        let actshow = st.lookup("$ctl/lib/$actshow").unwrap();
        assert_eq!(st.with_str(actshow, |x| x.is_empty()), Some(true));
        // `$scenes` definition carries the library's list
        let mut d = NrpDecoder::new();
        d.feed(&[0xDF, 0xD1]);
        d.feed(&out);
        let Some(Ok(Token::NodeDef(body))) = d.next_control() else {
            panic!("no def record")
        };
        let def = crate::nrp::NodeDef::parse(body).unwrap();
        let crate::nrp::nodedef::NodeKind::StringEnum(items) = def.kind else {
            panic!("{def:?}")
        };
        let names: Vec<_> = items.iter().map(|i| i.name.as_str()).collect();
        assert_eq!(names, ["Intro", "Verse"]);
    }
}
