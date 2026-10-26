//! T086 replay: the meter-report definitions WING-Edit 3.3.4 sent in the user's v0.5 session
//! (`fixtures/t086-meter-defines.txt`, extracted from `scratch/wing-sim-v0.5.log`) played against the
//! server's [`MeterSubscriptions`] by a model of WING-Edit's `nrp::MeterHandler`.
//!
//! # The client model (static analysis of WING-Edit 3.3.4, arm64; docs/research/protocol.md §4.3)
//!
//! * `MeterHandler` keeps two sets of meter bit masks, one per type (11 types: ch/aux/bus/main/mtx V2,
//!   dca, fx, source, output, monitor, RTA): `wanted` (`+0x38`, OR-ed in by every visible meter
//!   component from its `listener_callback` on the meter notification) and `current` (`+0x90`, the
//!   last request), plus the report id (`+0xe8`) and the time of the last request (`+0xec`, ms).
//! * `rxMeters` (`0x1000d72d4`) runs on the message thread for **every** received packet: it applies
//!   the values when the packet's id is the current id, posts the (coalesced, asynchronous)
//!   `Cnode::notify` of the meter node, then compares `wanted` with `current`: equal and more than
//!   4000 ms since the last request -> send the renew `D4 id`; different -> `current = wanted`,
//!   `id += 1`, `sendMeterRequest(id)` and reset `wanted` to `ChannelV2 1` only.
//! * The components re-register only when the posted notification runs, i.e. after every packet
//!   already queued on the message thread.
//!
//! So two packets queued together (two live reports in one 50 ms tick) make the second comparison
//! see the reset `wanted` (`ChannelV2 1`), and the client re-defines; each definition adds a live
//! report on a server that keeps superseded ids alive for 5 s, so the storm feeds itself (the log's
//! alternating `N meters` / `1 meters: ChannelV2 1` definitions). With one report per client
//! (T086) a page change costs at most two definitions (the union of the old and the new page's
//! registrations, then the new page alone, as in the log at 235.49 s) and nothing else.

use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::net::IpAddr;
use std::time::{Duration, Instant};

use wing_proto::nrp::meter::MeterTokenDecoder;
use wing_proto::nrp::{MeterCommand, MeterKind, MeterRequestParser};
use wing_proto::server::meters::{MeterSubscriptions, METER_INTERVAL, REPORT_LIFETIME};

/// WING-Edit's request order (`sendMeterRequest`: slot k = token `A0 + k`, V2 strips for k <= 4).
const ORDER: [MeterKind; 11] = [
    MeterKind::ChannelV2,
    MeterKind::AuxV2,
    MeterKind::BusV2,
    MeterKind::MainV2,
    MeterKind::MatrixV2,
    MeterKind::Dca,
    MeterKind::Fx,
    MeterKind::Source,
    MeterKind::Output,
    MeterKind::Monitor,
    MeterKind::Rta,
];

type Want = BTreeMap<usize, BTreeSet<u8>>;

fn kind_by_name(name: &str) -> MeterKind {
    *ORDER
        .iter()
        .find(|k| format!("{k:?}") == name)
        .unwrap_or_else(|| panic!("meter kind {name}"))
}

/// Parse a logged selection list (`describe_selections`): `ChannelV2 1-16, Fx 7, Source 1,4-5,8, Rta`.
fn parse_want(desc: &str) -> Want {
    let mut w = Want::new();
    for item in desc.split(", ") {
        let (name, idx) = item.split_once(' ').unwrap_or((item, "1"));
        let k = ORDER.iter().position(|&o| o == kind_by_name(name)).unwrap();
        let set = w.entry(k).or_default();
        for r in idx.split(',') {
            let (a, b) = r.split_once('-').unwrap_or((r, r));
            let (a, b): (u8, u8) = (a.parse().unwrap(), b.parse().unwrap());
            set.extend(a - 1..b);
        }
    }
    w
}

fn ch1() -> Want {
    Want::from([(0, BTreeSet::from([0]))])
}

/// The channel-3 bytes `sendMeterRequest` writes: `D4 id DC {type idx*}* DE`, monitor and RTA
/// without an index.
fn request_bytes(id: u32, w: &Want) -> Vec<u8> {
    let mut b = vec![0xD4];
    b.extend_from_slice(&id.to_be_bytes());
    b.push(0xDC);
    for (&k, idx) in w {
        b.push(ORDER[k].token());
        if !ORDER[k].single_instance() {
            b.extend(idx.iter().copied());
        }
    }
    b.push(0xDE);
    b
}

/// The server side under test.
trait Server {
    fn apply(&mut self, cmd: &MeterCommand, now: Instant);
    /// Report ids sent in one tick, in send order.
    fn tick(&mut self, now: Instant) -> Vec<u32>;
    fn live(&self, now: Instant) -> usize;
}

const CLIENT: u32 = 1;

fn ip() -> IpAddr {
    "127.0.0.1".parse().unwrap()
}

impl Server for MeterSubscriptions {
    fn apply(&mut self, cmd: &MeterCommand, now: Instant) {
        MeterSubscriptions::apply(self, CLIENT, ip(), cmd, now);
    }
    fn tick(&mut self, now: Instant) -> Vec<u32> {
        let mut ids = Vec::new();
        self.for_each_live(now, |_, id, _| ids.push(id));
        ids
    }
    fn live(&self, now: Instant) -> usize {
        self.live_reports(now)
    }
}

/// The pre-T086 behaviour (every id lives 5 s after its definition or renew), to show that the
/// client model reproduces the logged storm.
#[derive(Default)]
struct Legacy(Vec<(u32, Instant)>);

impl Server for Legacy {
    fn apply(&mut self, cmd: &MeterCommand, now: Instant) {
        match cmd {
            MeterCommand::Define { id, .. } => {
                self.0.retain(|r| r.0 != *id);
                self.0.push((*id, now + REPORT_LIFETIME));
            }
            MeterCommand::Renew(id) => {
                if let Some(r) = self.0.iter_mut().find(|r| r.0 == *id) {
                    r.1 = now + REPORT_LIFETIME;
                }
            }
            MeterCommand::SetPort(_) => {}
        }
    }
    fn tick(&mut self, now: Instant) -> Vec<u32> {
        self.0.retain(|r| r.1 > now);
        self.0.iter().map(|r| r.0).collect()
    }
    fn live(&self, now: Instant) -> usize {
        self.0.iter().filter(|r| r.1 > now).count()
    }
}

enum Msg {
    Packet,
    Notify,
}

/// WING-Edit's `MeterHandler` and message thread (see the module doc).
struct WingEdit {
    page: Want,
    wanted: Want,
    current: Want,
    id: u32,
    last_ms: u64,
    notify_pending: bool,
    queue: VecDeque<Msg>,
    parser: MeterRequestParser,
    decoder: MeterTokenDecoder,
    /// Definitions sent (id, selections).
    defines: Vec<(u32, Want)>,
    renews: usize,
}

impl WingEdit {
    fn new(page: Want) -> Self {
        Self {
            page,
            wanted: ch1(),
            current: Want::new(),
            id: 0,
            last_ms: 0,
            notify_pending: false,
            queue: VecDeque::new(),
            parser: MeterRequestParser::new(),
            decoder: MeterTokenDecoder::new(),
            defines: Vec::new(),
            renews: 0,
        }
    }

    fn send(&mut self, bytes: &[u8], srv: &mut dyn Server, now: Instant) {
        self.decoder.extend(bytes);
        while let Some(t) = self.decoder.next_token() {
            if let Some(cmd) = self.parser.feed(t.unwrap()) {
                srv.apply(&cmd, now);
            }
        }
    }

    /// `rxMeters`' bookkeeping for one packet.
    fn rx(&mut self, srv: &mut dyn Server, now: Instant, ms: u64) {
        if !self.notify_pending {
            self.notify_pending = true;
            self.queue.push_back(Msg::Notify);
        }
        if self.wanted == self.current {
            if ms - self.last_ms > 4000 {
                self.last_ms = ms;
                self.renews += 1;
                let mut b = vec![0xD4];
                b.extend_from_slice(&self.id.to_be_bytes());
                self.send(&b, srv, now);
            }
        } else {
            self.current = self.wanted.clone();
            self.id += 1;
            self.last_ms = ms;
            let b = request_bytes(self.id, &self.current);
            self.send(&b, srv, now);
            self.defines.push((self.id, self.current.clone()));
            self.wanted = ch1();
        }
    }

    /// One meter tick: every packet the server sends arrives (back to back) before the message
    /// thread runs; then the queue drains, notifications included.
    fn tick(&mut self, srv: &mut dyn Server, now: Instant, ms: u64) {
        for _id in srv.tick(now) {
            self.queue.push_back(Msg::Packet);
        }
        while let Some(m) = self.queue.pop_front() {
            match m {
                Msg::Packet => self.rx(srv, now, ms),
                Msg::Notify => {
                    self.notify_pending = false;
                    for (k, v) in &self.page {
                        self.wanted.entry(*k).or_default().extend(v.iter().copied());
                    }
                }
            }
        }
    }
}

/// The page sets of the session, in order: every logged definition except WING-Edit's reset set
/// (`ChannelV2 1`) and the partial sets of its storms (a strict subset of a neighbouring definition:
/// some components had re-registered, others not), consecutive duplicates collapsed.
fn session_pages() -> Vec<Want> {
    let text = include_str!("fixtures/t086-meter-defines.txt");
    let defs: Vec<Want> = text
        .lines()
        .filter(|l| !l.starts_with('#') && !l.is_empty())
        .map(|l| {
            let mut it = l.splitn(3, ' ');
            let (_t, _id) = (it.next(), it.next());
            parse_want(it.next().unwrap())
        })
        .collect();
    assert!(defs.len() > 1000, "fixture: {} definitions", defs.len());
    let subset = |a: &Want, b: &Want| {
        a != b
            && a.iter()
                .all(|(k, v)| b.get(k).is_some_and(|w| v.is_subset(w)))
    };
    let mut pages: Vec<Want> = Vec::new();
    for (i, d) in defs.iter().enumerate() {
        if *d == ch1() {
            continue;
        }
        let near = [i.checked_sub(1), i.checked_sub(2), Some(i + 1), Some(i + 2)];
        if near
            .iter()
            .flatten()
            .filter_map(|&j| defs.get(j))
            .any(|n| subset(d, n))
        {
            continue;
        }
        if pages.last() != Some(d) {
            pages.push(d.clone());
        }
    }
    pages
}

struct Outcome {
    defines: Vec<(u32, Want)>,
    renews: usize,
    page_changes: usize,
    max_live: usize,
    /// (page index, defines while on it)
    worst: (usize, usize),
    settled: bool,
}

/// Replay the session's pages, `secs` seconds each, then 12 s idle on the last page.
fn replay(srv: &mut dyn Server, pages: &[Want], secs: u64) -> Outcome {
    let t0 = Instant::now();
    let mut we = WingEdit::new(pages[0].clone());
    let mut ms = 10_000u64;
    let tick_ms = METER_INTERVAL.as_millis() as u64;
    let at = |ms: u64| t0 + Duration::from_millis(ms);
    // connect: port, then report 0 as ConnectPage::connect_device sends it
    we.send(&[0xD3, 0x37, 0x37], srv, at(ms));
    let b = request_bytes(0, &pages[0]);
    we.send(&b, srv, at(ms));
    we.current = pages[0].clone();
    we.wanted = pages[0].clone(); // the first page registered before the connect
    we.last_ms = ms;
    let (mut max_live, mut worst, mut changes, mut settled) = (0, (0, 0), 0, true);
    for (p, page) in pages.iter().enumerate() {
        if p > 0 && *page != pages[p - 1] {
            changes += 1;
        }
        we.page = page.clone();
        let before = we.defines.len();
        let ticks = if p + 1 == pages.len() {
            (secs * 1000 + 12_000) / tick_ms
        } else {
            secs * 1000 / tick_ms
        };
        for _ in 0..ticks {
            ms += tick_ms;
            we.tick(srv, at(ms), ms);
            max_live = max_live.max(srv.live(at(ms)));
        }
        let n = we.defines.len() - before;
        if n > worst.1 {
            worst = (p, n);
        }
        // the request covers the page (WING-Edit only re-defines when a page adds a meter, so a
        // page that only drops meters keeps the larger request, as in the log)
        settled &= page
            .iter()
            .all(|(k, v)| we.current.get(k).is_some_and(|c| v.is_subset(c)));
    }
    settled &= srv.live(at(ms)) >= 1;
    Outcome {
        defines: we.defines,
        renews: we.renews,
        page_changes: changes,
        max_live,
        worst,
        settled,
    }
}

#[test]
fn session_replay_has_no_definition_churn() {
    let pages = session_pages();
    assert!(pages.len() > 20, "{} pages", pages.len());
    let mut srv = MeterSubscriptions::new();
    let o = replay(&mut srv, &pages, 2);
    eprintln!(
        "fixed server: {} pages, {} changes, {} definitions, {} renews, max {} live",
        pages.len(),
        o.page_changes,
        o.defines.len(),
        o.renews,
        o.max_live
    );
    // one live report at any time; every page reached and kept alive by renews
    assert_eq!(o.max_live, 1);
    assert!(o.settled, "WING-Edit requests every meter of each page");
    assert!(
        o.renews >= 2,
        "renews keep the last report alive: {}",
        o.renews
    );
    // a page change costs at most the union and the page itself; the reset set is never requested
    assert!(
        o.worst.1 <= 2,
        "page {} took {} definitions",
        o.worst.0,
        o.worst.1
    );
    assert!(o.defines.len() <= 2 * o.page_changes);
    assert!(o.defines.iter().all(|d| d.1 != ch1()));
}

/// The model reproduces the logged storm against the pre-T086 server (sanity check of the model).
#[test]
fn legacy_server_reproduces_the_logged_churn() {
    let pages = session_pages();
    // the first page change of the log (235.49 s): ch 1,5,8 + mains/matrices -> ch 1-5, bus 1-16, main 1
    let first = &pages[..3.min(pages.len())];
    let mut legacy = Legacy::default();
    let o = replay(&mut legacy, first, 2);
    eprintln!(
        "legacy server: {} changes, {} definitions, max {} live",
        o.page_changes,
        o.defines.len(),
        o.max_live
    );
    assert!(o.max_live >= 2, "superseded ids stay live: {}", o.max_live);
    assert!(
        o.defines.iter().any(|d| d.1 == ch1()),
        "WING-Edit re-defines its reset set ChannelV2 1"
    );
    assert!(
        o.defines.len() > 4 * o.page_changes.max(1),
        "{} definitions for {} page changes",
        o.defines.len(),
        o.page_changes
    );
    let mut fixed = MeterSubscriptions::new();
    let f = replay(&mut fixed, first, 2);
    assert!(f.defines.len() <= 2 * f.page_changes.max(1));
}
