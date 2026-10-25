//! Meter transport (T015): channel-3 subscriptions, UDP meter streaming, and the lock-free
//! [`MeterBank`] the meter values are read from.
//!
//! Wire format (protocol.md §4, wing-edit-session.md §1): a client sets its UDP port (`D3`),
//! defines reports (`D4 id DC <kind idx...>... DE`) and renews them (`D4 id`, WING-Edit every
//! 4.04 s). Each live report is sent as `id.l + int16[]` (big-endian, 1/256 dB levels) to
//! `client_ip:port` every 50 ms until 5 s after its last renew.
//!
//! Producers: [`SyntheticMeters`] (no audio), or real levels (T039: the daemon converts the
//! engine's lock-free level bank into this [`MeterBank`] from the streamer's per-tick source hook,
//! [`MeterStreamer::spawn_with_source`], right before each send). `MeterBank::set` is a relaxed
//! atomic store: no locks, no allocation.
//!
//! Unverified [H]: the per-group word counts of source/output groups (`A7`/`A8`) are taken from
//! the schema's `io/in` / `io/out` group sizes (local 24, AES50 48, card 64, ...), matching
//! protocol doc 3.1.2's "local ins: 24"; the packet sizes WING-Edit expects are not confirmed.

use std::collections::HashMap;
use std::io;
use std::net::{IpAddr, SocketAddr, UdpSocket};
use std::sync::atomic::{AtomicBool, AtomicU16, AtomicU64, Ordering::Relaxed};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use wing_model::{ParamStore, Schema, Value};

use crate::nrp::meter::{db_to_meter, encode_meter_packet};
use crate::nrp::{MeterCommand, MeterKind, MeterSelection};

/// Report lifetime after a define or renew.
pub const REPORT_LIFETIME: Duration = Duration::from_secs(5);
/// Console send interval.
pub const METER_INTERVAL: Duration = Duration::from_millis(50);
/// WING-Edit's UDP receive buffer (wing-edit-schema.md §6); larger packets are truncated.
pub const MAX_METER_PACKET: usize = 0xFC8;

const ALL_KINDS: [MeterKind; 16] = [
    MeterKind::Channel,
    MeterKind::Aux,
    MeterKind::Bus,
    MeterKind::Main,
    MeterKind::Matrix,
    MeterKind::Dca,
    MeterKind::Fx,
    MeterKind::Source,
    MeterKind::Output,
    MeterKind::Monitor,
    MeterKind::Rta,
    MeterKind::ChannelV2,
    MeterKind::AuxV2,
    MeterKind::BusV2,
    MeterKind::MainV2,
    MeterKind::MatrixV2,
];

/// Source groups (`A7`) in protocol order, as `io/in` group names.
pub const SOURCE_GROUPS: [&str; 16] = [
    "LCL", "AUX", "A", "B", "C", "SC", "USB", "CRD", "MOD", "PLAY", "AES", "USR", "OSC", "$BUS",
    "$MAIN", "$MTX",
];
/// Output groups (`A8`) in protocol order, as `io/out` group names.
pub const OUTPUT_GROUPS: [&str; 11] = [
    "LCL", "AUX", "A", "B", "C", "SC", "USB", "CRD", "MOD", "REC", "AES",
];

/// Where each (kind, index) block lives in the bank.
#[derive(Clone, Debug)]
pub struct MeterLayout {
    /// Per kind (indexed by `token - 0xA0`): (first word, words per index; one entry per index).
    blocks: Vec<Vec<(usize, usize)>>,
    total: usize,
}

impl MeterLayout {
    /// Layout for the full-size WING from the schema's instance counts.
    pub fn from_schema(s: &Schema) -> MeterLayout {
        let count = |path: &str, fallback: usize| {
            s.by_path(path)
                .map(|id| s.children(id).count())
                .filter(|&n| n > 0)
                .unwrap_or(fallback)
        };
        let group = |dir: &str, g: &str| count(&format!("io/{dir}/{g}"), 8);
        let mut blocks = Vec::with_capacity(16);
        let mut total = 0;
        for k in ALL_KINDS {
            let sizes: Vec<usize> = match k {
                MeterKind::Source => SOURCE_GROUPS.iter().map(|g| group("in", g)).collect(),
                MeterKind::Output => OUTPUT_GROUPS.iter().map(|g| group("out", g)).collect(),
                _ => {
                    let n = match k {
                        MeterKind::Channel | MeterKind::ChannelV2 => count("ch", 40),
                        MeterKind::Aux | MeterKind::AuxV2 => count("aux", 8),
                        MeterKind::Bus | MeterKind::BusV2 => count("bus", 16),
                        MeterKind::Main | MeterKind::MainV2 => count("main", 4),
                        MeterKind::Matrix | MeterKind::MatrixV2 => count("mtx", 8),
                        MeterKind::Dca => count("dca", 16),
                        MeterKind::Fx => count("fx", 16),
                        _ => 1,
                    };
                    vec![k.words_per_index().unwrap_or(1); n]
                }
            };
            let mut v = Vec::with_capacity(sizes.len());
            for w in sizes {
                v.push((total, w));
                total += w;
            }
            blocks.push(v);
        }
        MeterLayout { blocks, total }
    }

    /// Number of indices of a kind.
    pub fn count(&self, kind: MeterKind) -> usize {
        self.blocks[(kind.token() - 0xA0) as usize].len()
    }

    /// (first word, word count) of a block.
    pub fn block(&self, kind: MeterKind, index: usize) -> Option<(usize, usize)> {
        self.blocks[(kind.token() - 0xA0) as usize]
            .get(index)
            .copied()
    }

    /// Words a selection produces (out-of-range indices produce zeros of the kind's width).
    pub fn words(&self, sel: &MeterSelection) -> usize {
        sel.indices
            .iter()
            .map(|&i| match self.block(sel.kind, i as usize) {
                Some((_, w)) => w,
                None => sel.kind.words_per_index().unwrap_or(0),
            })
            .sum()
    }
}

/// Latest meter values (1/256 dB units, see protocol.md §4.2), one relaxed atomic per word.
/// Readers may observe a block half-updated, which is harmless for meters; in exchange writes
/// are wait-free and allocation-free (safe from an RT/metering thread).
pub struct MeterBank {
    layout: MeterLayout,
    words: Box<[AtomicU16]>,
}

impl MeterBank {
    pub fn new(layout: MeterLayout) -> Self {
        let silence = db_to_meter(-128.0) as u16;
        let words = (0..layout.total).map(|_| AtomicU16::new(silence)).collect();
        MeterBank { layout, words }
    }

    /// Bank laid out for the schema's full-size WING.
    pub fn for_schema() -> Self {
        Self::new(MeterLayout::from_schema(Schema::get()))
    }

    pub fn layout(&self) -> &MeterLayout {
        &self.layout
    }

    /// Set one word of a block (raw 1/256 dB or state value). Out-of-range is ignored.
    #[inline]
    pub fn set(&self, kind: MeterKind, index: usize, word: usize, v: i16) {
        if let Some((start, w)) = self.layout.block(kind, index) {
            if word < w {
                self.words[start + word].store(v as u16, Relaxed);
            }
        }
    }

    /// Set a whole block from a slice (extra words ignored).
    pub fn set_block(&self, kind: MeterKind, index: usize, values: &[i16]) {
        if let Some((start, w)) = self.layout.block(kind, index) {
            for (dst, &v) in self.words[start..start + w].iter().zip(values) {
                dst.store(v as u16, Relaxed);
            }
        }
    }

    pub fn get(&self, kind: MeterKind, index: usize, word: usize) -> Option<i16> {
        let (start, w) = self.layout.block(kind, index)?;
        (word < w).then(|| self.words[start + word].load(Relaxed) as i16)
    }

    /// Append the words of a report (selections in request order) to `out`.
    pub fn read_into(&self, sels: &[MeterSelection], out: &mut Vec<i16>) {
        for sel in sels {
            for &i in &sel.indices {
                match self.layout.block(sel.kind, i as usize) {
                    Some((start, w)) => out.extend(
                        self.words[start..start + w]
                            .iter()
                            .map(|a| a.load(Relaxed) as i16),
                    ),
                    None => out.extend(std::iter::repeat_n(
                        0,
                        sel.kind.words_per_index().unwrap_or(0),
                    )),
                }
            }
        }
    }
}

#[derive(Clone, Debug)]
struct Report {
    client: u32,
    id: u32,
    selections: Vec<MeterSelection>,
    expires: Instant,
}

#[derive(Default)]
struct SubsInner {
    /// client -> (ip, udp port)
    clients: HashMap<u32, (IpAddr, Option<u16>)>,
    reports: Vec<Report>,
}

/// Meter subscriptions of all native clients.
#[derive(Default)]
pub struct MeterSubscriptions {
    inner: Mutex<SubsInner>,
}

impl MeterSubscriptions {
    pub fn new() -> Self {
        Self::default()
    }

    fn lock(&self) -> std::sync::MutexGuard<'_, SubsInner> {
        self.inner.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Apply a channel-3 command from `client` (connected from `ip`).
    pub fn apply(&self, client: u32, ip: IpAddr, cmd: &MeterCommand, now: Instant) {
        let mut g = self.lock();
        match cmd {
            MeterCommand::SetPort(p) => {
                g.clients.insert(client, (ip, Some(*p)));
            }
            MeterCommand::Renew(id) => {
                g.clients.entry(client).or_insert((ip, None));
                if let Some(r) = g
                    .reports
                    .iter_mut()
                    .find(|r| r.client == client && r.id == *id)
                {
                    r.expires = now + REPORT_LIFETIME;
                }
            }
            MeterCommand::Define { id, selections } => {
                g.clients.entry(client).or_insert((ip, None));
                let rep = Report {
                    client,
                    id: *id,
                    selections: selections.clone(),
                    expires: now + REPORT_LIFETIME,
                };
                match g
                    .reports
                    .iter_mut()
                    .find(|r| r.client == client && r.id == *id)
                {
                    Some(r) => *r = rep,
                    None => g.reports.push(rep),
                }
            }
        }
    }

    /// Forget a disconnected client.
    pub fn remove_client(&self, client: u32) {
        let mut g = self.lock();
        g.clients.remove(&client);
        g.reports.retain(|r| r.client != client);
    }

    /// Number of live (unexpired) reports.
    pub fn live_reports(&self, now: Instant) -> usize {
        self.lock()
            .reports
            .iter()
            .filter(|r| r.expires > now)
            .count()
    }

    /// Call `f(dest, report_id, selections)` for every live report with a known port; expired
    /// reports are dropped.
    pub fn for_each_live(
        &self,
        now: Instant,
        mut f: impl FnMut(SocketAddr, u32, &[MeterSelection]),
    ) {
        let mut g = self.lock();
        g.reports.retain(|r| r.expires > now);
        let g = &*g;
        for r in &g.reports {
            if let Some(&(ip, Some(port))) = g.clients.get(&r.client) {
                f(SocketAddr::new(ip, port), r.id, &r.selections);
            }
        }
    }
}

/// Background thread sending live reports every `interval`.
pub struct MeterStreamer {
    stop: Arc<AtomicBool>,
    packets: Arc<AtomicU64>,
    thread: Option<JoinHandle<()>>,
}

impl MeterStreamer {
    /// Send from `sock` (the console sends from UDP 2222: pass a clone of the discovery
    /// socket, see `Responder::try_clone_socket`).
    pub fn spawn(
        subs: Arc<MeterSubscriptions>,
        bank: Arc<MeterBank>,
        sock: UdpSocket,
        interval: Duration,
    ) -> io::Result<MeterStreamer> {
        Self::spawn_with_source(subs, bank, sock, interval, |_| {})
    }

    /// Like [`MeterStreamer::spawn`], calling `source(&bank)` on the streamer thread at the start
    /// of every tick (before the live reports are read), so a producer can refresh the bank in
    /// step with the packets.
    pub fn spawn_with_source(
        subs: Arc<MeterSubscriptions>,
        bank: Arc<MeterBank>,
        sock: UdpSocket,
        interval: Duration,
        mut source: impl FnMut(&MeterBank) + Send + 'static,
    ) -> io::Result<MeterStreamer> {
        let stop = Arc::new(AtomicBool::new(false));
        let packets = Arc::new(AtomicU64::new(0));
        let (stop2, packets2) = (stop.clone(), packets.clone());
        let thread = std::thread::Builder::new()
            .name("wing-meters".into())
            .spawn(move || {
                let mut words = Vec::new();
                let mut buf = Vec::new();
                let mut next = Instant::now();
                while !stop2.load(Relaxed) {
                    source(&bank);
                    let now = Instant::now();
                    subs.for_each_live(now, |dest, id, sels| {
                        words.clear();
                        bank.read_into(sels, &mut words);
                        buf.clear();
                        encode_meter_packet(id, &words, &mut buf);
                        buf.truncate(MAX_METER_PACKET);
                        if sock.send_to(&buf, dest).is_ok() {
                            packets2.fetch_add(1, Relaxed);
                        }
                    });
                    next += interval;
                    let now = Instant::now();
                    if next > now {
                        std::thread::sleep(next - now);
                    } else {
                        next = now;
                    }
                }
            })?;
        Ok(MeterStreamer {
            stop,
            packets,
            thread: Some(thread),
        })
    }

    /// Packets sent so far.
    pub fn packets(&self) -> u64 {
        self.packets.load(Relaxed)
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

impl Drop for MeterStreamer {
    fn drop(&mut self) {
        self.shutdown();
    }
}

/// Synthetic but plausible meter values: a slow sine per strip (each with its own phase and
/// rate, -40..-8 dB) plus a little flicker; strip outputs follow the fader and mute when a
/// store is given. Placeholder until T039 feeds real levels.
pub struct SyntheticMeters {
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

impl SyntheticMeters {
    /// Fill `bank` once for time `t` (seconds).
    pub fn fill(bank: &MeterBank, store: Option<&ParamStore>, t: f64) {
        let lay = bank.layout();
        let strip = |kind: MeterKind, v2: MeterKind, prefix: &str, seed: f64| {
            for i in 0..lay.count(kind) {
                let ph = seed + i as f64 * 0.77;
                let rate = 0.15 + 0.05 * ((i % 5) as f64);
                let flick = 1.5 * (t * 7.3 + ph * 3.1).sin();
                let in_db = -24.0 + 16.0 * (std::f64::consts::TAU * rate * t + ph).sin() + flick;
                let (gain, muted) = store
                    .and_then(|s| {
                        let n = i + 1;
                        let fdr = s.lookup(&format!("{prefix}/{n}/fdr"))?;
                        let mute = s.lookup(&format!("{prefix}/{n}/mute"))?;
                        let g = match s.get(fdr)? {
                            Value::Float(f) => f as f64,
                            _ => 0.0,
                        };
                        Some((g, s.get(mute) == Some(Value::Int(1))))
                    })
                    .unwrap_or((0.0, false));
                let out_db = if muted { -128.0 } else { in_db + gain };
                let l_in = db_to_meter(in_db as f32);
                let r_in = db_to_meter((in_db - 0.7) as f32);
                let l_out = db_to_meter(out_db.max(-128.0) as f32);
                let r_out = db_to_meter((out_db - 0.7).max(-128.0) as f32);
                let key = db_to_meter((in_db - 3.0) as f32);
                // V1: in L/R, out L/R, gate key, gate gain, dyn key, dyn gain
                bank.set_block(kind, i, &[l_in, r_in, l_out, r_out, key, 0, key, 0]);
                // V2: in L/R, out L/R, gate key, gate gain, gate LED, dyn key, dyn gain,
                // dyn state, automix gain
                bank.set_block(v2, i, &[l_in, r_in, l_out, r_out, key, 0, 0, key, 0, 0, 0]);
            }
        };
        strip(MeterKind::Channel, MeterKind::ChannelV2, "ch", 0.0);
        strip(MeterKind::Aux, MeterKind::AuxV2, "aux", 1.3);
        strip(MeterKind::Bus, MeterKind::BusV2, "bus", 2.1);
        strip(MeterKind::Main, MeterKind::MainV2, "main", 0.4);
        strip(MeterKind::Matrix, MeterKind::MatrixV2, "mtx", 3.3);
        for i in 0..lay.count(MeterKind::Dca) {
            let v = db_to_meter((-20.0 + 10.0 * (t * 0.2 + i as f64).sin()) as f32);
            bank.set_block(MeterKind::Dca, i, &[v, v, v, v]);
        }
        for i in 0..lay.count(MeterKind::Fx) {
            let v = db_to_meter((-30.0 + 10.0 * (t * 0.3 + i as f64 * 0.5).sin()) as f32);
            bank.set_block(MeterKind::Fx, i, &[v, v, v, v, 0, 0, 0, 0, 0, 0]);
        }
        for (kind, seed) in [(MeterKind::Source, 0.0), (MeterKind::Output, 5.0)] {
            for g in 0..lay.count(kind) {
                let (_, w) = lay.block(kind, g).unwrap_or((0, 0));
                for k in 0..w {
                    let ph = seed + g as f64 * 1.9 + k as f64 * 0.61;
                    let db = -30.0 + 14.0 * (std::f64::consts::TAU * 0.2 * t + ph).sin();
                    bank.set(kind, g, k, db_to_meter(db as f32));
                }
            }
        }
        let mon = db_to_meter((-18.0 + 6.0 * (t * 0.5).sin()) as f32);
        bank.set_block(MeterKind::Monitor, 0, &[mon; 6]);
        for k in 0..120 {
            let db = -60.0
                + 30.0 * (-(k as f64 - 40.0).powi(2) / 800.0).exp()
                + 6.0 * (t * 1.1 + k as f64 * 0.3).sin();
            bank.set(MeterKind::Rta, 0, k, db_to_meter(db as f32));
        }
    }

    /// Update `bank` every `interval` on a background thread.
    pub fn spawn(
        bank: Arc<MeterBank>,
        store: Option<Arc<ParamStore>>,
        interval: Duration,
    ) -> io::Result<SyntheticMeters> {
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = stop.clone();
        let thread = std::thread::Builder::new()
            .name("wing-meter-synth".into())
            .spawn(move || {
                let t0 = Instant::now();
                while !stop2.load(Relaxed) {
                    Self::fill(&bank, store.as_deref(), t0.elapsed().as_secs_f64());
                    std::thread::sleep(interval);
                }
            })?;
        Ok(SyntheticMeters {
            stop,
            thread: Some(thread),
        })
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

impl Drop for SyntheticMeters {
    fn drop(&mut self) {
        self.shutdown();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::nrp::MeterPacket;

    fn sel(kind: MeterKind, idx: &[u8]) -> MeterSelection {
        MeterSelection {
            kind,
            indices: idx.to_vec(),
        }
    }

    /// WING-Edit's report 0: ch 1-16 V2, main 1 V2, source groups 0,3,4,7.
    fn report0() -> Vec<MeterSelection> {
        vec![
            sel(MeterKind::ChannelV2, &(0..16).collect::<Vec<_>>()),
            sel(MeterKind::MainV2, &[0]),
            sel(MeterKind::Source, &[0, 3, 4, 7]),
        ]
    }

    #[test]
    fn layout_and_report_size() {
        let bank = MeterBank::for_schema();
        let lay = bank.layout();
        assert_eq!(lay.count(MeterKind::Channel), 40);
        assert_eq!(lay.count(MeterKind::Source), 16);
        assert_eq!(lay.block(MeterKind::Source, 0).unwrap().1, 24);
        let words: usize = report0().iter().map(|s| lay.words(s)).sum();
        assert_eq!(words, 16 * 11 + 11 + 24 + 48 + 48 + 64);
        assert!(4 + 2 * words <= MAX_METER_PACKET);
        SyntheticMeters::fill(&bank, None, 1.0);
        let mut out = Vec::new();
        bank.read_into(&report0(), &mut out);
        assert_eq!(out.len(), words);
        assert!(out[0] > db_to_meter(-60.0) && out[0] < db_to_meter(0.0));
    }

    #[test]
    fn synthetic_follows_mute() {
        let st = ParamStore::new();
        let bank = MeterBank::for_schema();
        st.set(st.lookup("ch/2/mute").unwrap(), Value::Int(1))
            .unwrap();
        SyntheticMeters::fill(&bank, Some(&st), 3.0);
        assert_eq!(
            bank.get(MeterKind::ChannelV2, 1, 2),
            Some(db_to_meter(-128.0))
        );
        assert!(bank.get(MeterKind::ChannelV2, 0, 0).unwrap() > db_to_meter(-60.0));
    }

    #[test]
    fn subscriptions_stream_and_expire() {
        let subs = Arc::new(MeterSubscriptions::new());
        let bank = Arc::new(MeterBank::for_schema());
        SyntheticMeters::fill(&bank, None, 0.5);
        let rx = UdpSocket::bind("127.0.0.1:0").unwrap();
        rx.set_read_timeout(Some(Duration::from_secs(2))).unwrap();
        let port = rx.local_addr().unwrap().port();
        let ip: IpAddr = "127.0.0.1".parse().unwrap();
        let now = Instant::now();
        subs.apply(1, ip, &MeterCommand::SetPort(port), now);
        subs.apply(
            1,
            ip,
            &MeterCommand::Define {
                id: 3,
                selections: report0(),
            },
            now,
        );
        assert_eq!(subs.live_reports(now), 1);
        assert_eq!(subs.live_reports(now + Duration::from_secs(6)), 0);
        subs.apply(1, ip, &MeterCommand::Renew(3), now + Duration::from_secs(4));
        assert_eq!(subs.live_reports(now + Duration::from_secs(6)), 1);
        let tx = UdpSocket::bind("127.0.0.1:0").unwrap();
        let streamer =
            MeterStreamer::spawn(subs.clone(), bank, tx, Duration::from_millis(10)).unwrap();
        let mut buf = [0u8; 8192];
        let mut got = 0;
        let t0 = Instant::now();
        while got < 3 {
            let n = rx.recv(&mut buf).unwrap();
            let p = MeterPacket::parse(&buf[..n]).unwrap();
            assert_eq!(p.report_id, 3);
            assert_eq!(p.len(), 16 * 11 + 11 + 184);
            got += 1;
        }
        // 3 packets at 10 ms
        assert!(t0.elapsed() < Duration::from_millis(500));
        subs.remove_client(1);
        assert_eq!(subs.live_reports(Instant::now()), 0);
        assert!(streamer.packets() >= 3);
        streamer.stop();
    }
}
