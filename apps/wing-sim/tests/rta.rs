//! T084 RTA end to end: `cfg/rta` settings -> status mirror + engine request -> console tap
//! (strip probe or port) -> lock-free ring -> analyser -> meter bank -> UDP `AA` report.
//!
//! * `rta_packets_carry_the_selected_tap`: a band-centre sine on USB In 1 -> ch 1 (trim +6 dB,
//!   fader -6 dB) reads at the right bin and level for the IN, PRE EQ and POST taps in the
//!   packets the meter streamer sends (loopback, ephemeral ports; never 2222/2223): 4-byte
//!   report id + 120 big-endian words in 1/128 dB.
//! * `analyser_tracks_tap_changes`: switching the source to a silent strip empties the spectrum.

use std::net::{IpAddr, UdpSocket};
use std::sync::Arc;
use std::time::{Duration, Instant};

use wing_engine::rta::{band_freq, RtaFeed, RTA_BANDS};
use wing_model::ParamStore;
use wing_proto::nrp::meter::rta_to_db;
use wing_proto::nrp::{MeterCommand, MeterKind, MeterPacket, MeterSelection};
use wing_proto::server::{MeterBank, MeterStreamer, MeterSubscriptions};
use wing_sim::audio::console::{Console, ConsoleConfig};
use wing_sim::audio::prepare;
use wing_sim::audio::stats::AudioStats;
use wing_sim::rta::RtaRunner;

const FS: u32 = 48_000;
const BLOCK: usize = 64;
/// D#5 = 622.25 Hz.
const BAND: usize = 59;

fn set(store: &ParamStore, path: &str, v: &str) {
    let id = store.lookup(path).unwrap_or_else(|| panic!("{path}"));
    store
        .set_text(id, v, 0)
        .unwrap_or_else(|e| panic!("{path} = {v}: {e:?}"));
}

fn build(store: &mut ParamStore, feed: Arc<RtaFeed>) -> Console {
    let mut routing = prepare(store);
    for (p, v) in [
        ("ch/1/in/conn/grp", "USB"),
        ("ch/1/in/conn/in", "1"),
        ("io/in/USB/1/mode", "M"),
        ("io/in/USB/1/mute", "0"),
        ("ch/1/in/set/trim", "6"),
        ("ch/1/fdr", "-6"),
        ("ch/1/mute", "0"),
    ] {
        set(store, p, v);
    }
    routing.refresh(store);
    Console::build(
        ConsoleConfig {
            sample_rate: FS,
            block: BLOCK,
            workers: 1,
            rt_workers: false,
            workgroup: None,
            meters: None,
            rta: Some(feed),
        },
        store,
        &mut routing,
        None,
        Vec::new(),
        Arc::new(AudioStats::default()),
    )
    .unwrap()
}

/// Run `secs` of a 0.1-amplitude (-20 dBFS) band-centre sine on USB In 1, stepping the RTA
/// runner every 50 ms of audio.
fn run(
    c: &mut Console,
    rta: &mut RtaRunner,
    st: &ParamStore,
    bank: &MeterBank,
    n: &mut u64,
    secs: f64,
) {
    let f = band_freq(BAND);
    let t0 = Instant::now();
    let cycles = (secs * FS as f64 / BLOCK as f64) as usize;
    let per_tick = (0.05 * FS as f64 / BLOCK as f64) as usize;
    for k in 0..cycles {
        let base = *n;
        c.cycle(
            |ch, buf| match ch {
                0 => buf.iter_mut().enumerate().for_each(|(i, s)| {
                    let t = (base + i as u64) as f64 / FS as f64;
                    *s = (0.1 * (std::f64::consts::TAU * f * t).sin()) as f32
                }),
                _ => buf.fill(0.0),
            },
            |_, _| {},
        );
        *n += BLOCK as u64;
        if k % per_tick == 0 {
            let at = t0 + Duration::from_secs_f64(*n as f64 / FS as f64);
            rta.step(st, Some(bank), at);
        }
    }
}

fn peak_bin(words: &[i16]) -> (usize, f32) {
    let (k, &w) = words.iter().enumerate().max_by_key(|(_, &w)| w).unwrap();
    (k, rta_to_db(w))
}

#[test]
fn rta_packets_carry_the_selected_tap() {
    let mut store = ParamStore::new();
    let feed = RtaFeed::shared();
    let mut c = build(&mut store, feed.clone());
    let bank = Arc::new(MeterBank::for_schema());
    let mut rta = RtaRunner::new(&store, Some(feed.clone()));

    // the meter streamer and a WING-Edit-like AA report (id 7) on loopback
    let subs = Arc::new(MeterSubscriptions::new());
    let rx = UdpSocket::bind("127.0.0.1:0").unwrap();
    rx.set_read_timeout(Some(Duration::from_secs(2))).unwrap();
    let ip: IpAddr = "127.0.0.1".parse().unwrap();
    let now = Instant::now();
    subs.apply(
        1,
        ip,
        &MeterCommand::SetPort(rx.local_addr().unwrap().port()),
        now,
    );
    let define = MeterCommand::Define {
        id: 7,
        selections: vec![MeterSelection {
            kind: MeterKind::Rta,
            indices: vec![0],
        }],
    };
    subs.apply(1, ip, &define, now);
    let tx = UdpSocket::bind("127.0.0.1:0").unwrap();
    let streamer =
        MeterStreamer::spawn(subs.clone(), bank.clone(), tx, Duration::from_millis(10)).unwrap();
    let recv = || {
        // drain what queued while the audio ran; the newest packet carries the current bank
        let mut buf = [0u8; 4096];
        let mut last = Vec::new();
        let mut take = |n: usize, buf: &[u8]| {
            assert_eq!(n, 4 + 2 * RTA_BANDS, "packet size");
            let p = MeterPacket::parse(&buf[..n]).unwrap();
            assert_eq!(p.report_id, 7);
            last = p.values().collect::<Vec<i16>>();
        };
        rx.set_nonblocking(true).unwrap();
        while let Ok(n) = rx.recv(&mut buf) {
            take(n, &buf);
        }
        rx.set_nonblocking(false).unwrap();
        // one packet sent after the drain (the bank is final by now)
        let n = rx.recv(&mut buf).unwrap();
        take(n, &buf);
        last
    };

    let mut n = 0;
    // (tap, expected dBFS at the band): IN = after trim (-20 + 6), PRE EQ = IN (EQ off),
    // POST = after the -6 dB fader and the centre const-power pan (-3 dB; a port tap)
    for (tap, want, tol) in [
        ("IN", -14.0, 0.5),
        ("PREEQ", -14.0, 0.5),
        ("POST", -23.0, 0.5),
    ] {
        set(&store, "cfg/rta/rtasrc", "1");
        set(&store, "cfg/rta/rtatap", tap);
        set(&store, "cfg/rta/rtadet", "PEAK");
        run(&mut c, &mut rta, &store, &bank, &mut n, 0.4);
        // (re)define the report: the debug-build audio run can outlast its 5 s lifetime
        subs.apply(1, ip, &define, Instant::now());
        let words = recv();
        let (k, db) = peak_bin(&words);
        assert_eq!(k, BAND, "{tap}: peak bin");
        assert!((db - want).abs() < tol, "{tap}: {db} dB, want {want}");
        // neighbours 3+ semitones away are well down
        assert!(rta_to_db(words[BAND + 3]) < db - 15.0, "{tap}: skirt");
    }
    // the console status WING-Edit checks follows the settings
    let v = |p: &str| {
        let mut s = String::new();
        store
            .format(store.lookup(&format!("cfg/rta/{p}")).unwrap(), &mut s)
            .unwrap();
        s
    };
    assert_eq!(v("$src"), "1");
    assert_eq!(v("$tap"), "POST");
    assert_eq!(v("$dec"), v("rtadecay"));
    streamer.stop();
}

#[test]
fn analyser_tracks_tap_changes() {
    let mut store = ParamStore::new();
    let feed = RtaFeed::shared();
    let mut c = build(&mut store, feed.clone());
    let bank = MeterBank::for_schema();
    let mut rta = RtaRunner::new(&store, Some(feed.clone()));
    let mut n = 0;
    set(&store, "cfg/rta/rtasrc", "1");
    set(&store, "cfg/rta/rtatap", "IN");
    run(&mut c, &mut rta, &store, &bank, &mut n, 0.4);
    let lvl = rta_to_db(bank.get(MeterKind::Rta, 0, BAND).unwrap());
    assert!(lvl > -20.0, "{lvl}");
    // ch 2 carries nothing: the display decays to silence (reset on the source change)
    set(&store, "cfg/rta/rtasrc", "2");
    run(&mut c, &mut rta, &store, &bank, &mut n, 0.3);
    let lvl = rta_to_db(bank.get(MeterKind::Rta, 0, BAND).unwrap());
    assert!(lvl <= -127.0, "{lvl}");
}
