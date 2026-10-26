//! T039 real metering, end to end: console (strips, T036 mix-graph port meters) -> level bank ->
//! `RealMeters` (ballistics, WING scaling) -> `MeterBank` -> UDP meter packets.
//!
//! * `sine_levels_reach_the_packets`: a known-level sine on USB In 1/3 reads within 0.5 dB in the
//!   packets the meter streamer sends (loopback, ephemeral ports) at the taps WING-Edit requests:
//!   ch 1 in/out (V1/V2), main 1 and bus 1 in/out, USB and $BUS source groups, USB output group.
//! * `gain_reduction_meter_matches_strip_telemetry`: the dyn GR word equals the strip's telemetry GR.
//! * `metering_overhead_per_cycle`: console cycle time with vs without metering (printed).
//!
//! Zero RT allocation with metering on is covered by `audio.rs` (`rt_path_does_not_allocate` runs
//! with `ConsoleConfig::meters` set).

use std::net::{IpAddr, UdpSocket};
use std::sync::atomic::{AtomicBool, Ordering::Relaxed};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use wing_engine::meters::LevelBank;
use wing_engine::params::telemetry_kind;
use wing_model::ParamStore;
use wing_proto::nrp::meter::{db_to_meter, meter_to_db, meter_to_gr, GR_FULL_SCALE_DB};
use wing_proto::nrp::{MeterCommand, MeterKind, MeterPacket, MeterSelection};
use wing_proto::server::{MeterBank, MeterStreamer, MeterSubscriptions};
use wing_sim::audio::console::{Console, ConsoleConfig};
use wing_sim::audio::prepare;
use wing_sim::audio::stats::AudioStats;
use wing_sim::meters::{meter_source, RealMeters};

const FS: u32 = 48_000;
const BLOCK: usize = 64;

fn set(store: &ParamStore, path: &str, v: &str) {
    let id = store.lookup(path).unwrap_or_else(|| panic!("{path}"));
    store
        .set_text(id, v, 0)
        .unwrap_or_else(|e| panic!("{path} = {v}: {e:?}"));
}

/// ch 1 <- USB 1, trim +6, fader -6, main 1 send 0 dB; USB Out 1/2 = main 1, USB Out 3 = USB In 3.
fn scene(store: &ParamStore) {
    for (p, v) in [
        ("ch/1/in/conn/grp", "USB"),
        ("ch/1/in/conn/in", "1"),
        ("io/in/USB/1/mode", "M"),
        ("io/in/USB/3/mode", "M"),
        ("io/in/USB/1/mute", "0"),
        ("io/in/USB/3/mute", "0"),
        ("ch/1/in/set/trim", "6"),
        ("ch/1/fdr", "-6"),
        ("ch/1/main/1/on", "1"),
        ("ch/1/main/1/lvl", "0"),
        ("main/1/fdr", "0"),
        ("main/1/mute", "0"),
        ("io/out/USB/1/grp", "MAIN"),
        ("io/out/USB/1/in", "1"),
        ("io/out/USB/2/grp", "MAIN"),
        ("io/out/USB/2/in", "2"),
        ("io/out/USB/3/grp", "USB"),
        ("io/out/USB/3/in", "3"),
        ("ch/1/send/1/on", "1"),
        ("ch/1/send/1/lvl", "0"),
        ("ch/1/send/1/mode", "POST"),
        ("bus/1/fdr", "-10"),
        ("bus/1/mute", "0"),
    ] {
        set(store, p, v);
    }
}

fn build(store: &mut ParamStore, meters: Option<Arc<LevelBank>>, workers: usize) -> Console {
    let mut routing = prepare(store);
    scene(store);
    routing.refresh(store);
    Console::build(
        ConsoleConfig {
            sample_rate: FS,
            block: BLOCK,
            workers,
            rt_workers: false,
            workgroup: None,
            meters,
            rta: None,
        },
        store,
        &mut routing,
        None,
        Vec::new(),
        Arc::new(AudioStats::default()),
    )
    .unwrap()
}

fn sine(n: u64, f: f64, amp: f64) -> f32 {
    (amp * (2.0 * std::f64::consts::PI * f * n as f64 / FS as f64).sin()) as f32
}

/// USB In 1: 1 kHz at 0.1 (-20 dBFS peak), USB In 3: 300 Hz at 0.2 (-13.98 dBFS).
fn cycle(c: &mut Console, n: &mut u64) {
    let base = *n;
    c.cycle(
        |ch, buf| match ch {
            0 => buf
                .iter_mut()
                .enumerate()
                .for_each(|(i, s)| *s = sine(base + i as u64, 1000.0, 0.1)),
            2 => buf
                .iter_mut()
                .enumerate()
                .for_each(|(i, s)| *s = sine(base + i as u64, 300.0, 0.2)),
            _ => buf.fill(0.0),
        },
        |_, _| {},
    );
    *n += BLOCK as u64;
}

fn sel(kind: MeterKind, idx: &[u8]) -> MeterSelection {
    MeterSelection {
        kind,
        indices: idx.to_vec(),
    }
}

#[test]
fn sine_levels_reach_the_packets() {
    let mut store = ParamStore::new();
    let levels = LevelBank::shared();
    let console = build(&mut store, Some(levels.clone()), 1);
    levels.set_attached(true);

    // meter streamer on loopback, ephemeral ports (never 2222/2223)
    let subs = Arc::new(MeterSubscriptions::new());
    let bank = Arc::new(MeterBank::for_schema());
    let rx = UdpSocket::bind("127.0.0.1:0").unwrap();
    rx.set_read_timeout(Some(Duration::from_secs(2))).unwrap();
    let ip: IpAddr = "127.0.0.1".parse().unwrap();
    let now = Instant::now();
    let port = rx.local_addr().unwrap().port();
    subs.apply(1, ip, &MeterCommand::SetPort(port), now);
    let report = vec![
        sel(MeterKind::ChannelV2, &[0]),
        sel(MeterKind::MainV2, &[0]),
        sel(MeterKind::Source, &[6]), // USB
        sel(MeterKind::Output, &[6]), // USB
        sel(MeterKind::Channel, &[0, 1]),
        sel(MeterKind::BusV2, &[0]),
        sel(MeterKind::Source, &[13]), // $BUS
    ];
    subs.apply(
        1,
        ip,
        &MeterCommand::Define {
            id: 7,
            selections: report.clone(),
        },
        now,
    );

    // audio runs on its own thread (as the RT thread does), roughly in real time
    let stop = Arc::new(AtomicBool::new(false));
    let console = Arc::new(Mutex::new(console));
    let audio = {
        let (stop, console) = (stop.clone(), console.clone());
        std::thread::spawn(move || {
            let mut n = 0u64;
            let mut c = console.lock().unwrap();
            while !stop.load(Relaxed) {
                for _ in 0..8 {
                    cycle(&mut c, &mut n);
                }
                std::thread::sleep(Duration::from_millis(10));
            }
        })
    };
    let streamer = MeterStreamer::spawn_with_source(
        subs.clone(),
        bank.clone(),
        UdpSocket::bind("127.0.0.1:0").unwrap(),
        Duration::from_millis(20),
        meter_source(Some(levels.clone()), None),
    )
    .unwrap();

    let words: usize = report.iter().map(|s| bank.layout().words(s)).sum();
    let mut buf = [0u8; 8192];
    let mut last = Vec::new();
    // skip the first packets (fader ramps, ballistics start)
    for _ in 0..15 {
        let n = rx.recv(&mut buf).unwrap();
        let p = MeterPacket::parse(&buf[..n]).unwrap();
        assert_eq!(p.report_id, 7);
        assert_eq!(p.len(), words);
        last = p.values().collect();
    }
    stop.store(true, Relaxed);
    audio.join().unwrap();
    streamer.stop();

    let db = |i: usize| meter_to_db(last[i]);
    let near = |i: usize, want: f32, what: &str| {
        let got = db(i);
        assert!(
            (got - want).abs() <= 0.5,
            "{what}: {got:.2} dB, want {want:.2} dB (±0.5)"
        );
    };
    // ch 1 V2: in = -20 + 6 trim, out = -14 - 6 fader
    near(0, -14.0, "ch1 in L");
    near(1, -14.0, "ch1 in R (mono source on both sides)");
    near(2, -20.0, "ch1 out L");
    near(3, -20.0, "ch1 out R");
    assert_eq!(last[5], 0, "no gate GR");
    assert_eq!(last[8], 0, "no dyn GR");
    // main 1 V2 (11 words at 11): ch 1 POST (constant-power pan, centre -3.01 dB) summed at
    // 0 dB, main fader 0 dB, balance pan
    near(11, -23.01, "main in L");
    near(13, -23.01, "main out L");
    near(14, -23.01, "main out R");
    // USB source group (48 legs at 22): USB 1 -20, USB 2 silent, USB 3 -14
    near(22, -20.0, "source USB.1");
    assert_eq!(last[23], i16::MIN, "source USB.2 silent");
    near(24, -13.98, "source USB.3");
    // USB output group (48 legs at 70): Out 1/2 = main, Out 3 = USB In 3
    near(70, -23.01, "USB out 1");
    near(71, -23.01, "USB out 2");
    near(72, -13.98, "USB out 3");
    assert_eq!(last[73], i16::MIN, "USB out 4 silent");
    // V1 ch 1 (8 words at 118), then ch 2 (silent)
    near(118, -14.0, "ch1 V1 in L");
    near(120, -20.0, "ch1 V1 out L");
    assert_eq!(last[126], i16::MIN, "ch2 in L silent");
    // bus 1 V2 (at 134): POST send (-23.01) in, -10 dB fader out
    near(134, -23.01, "bus1 in L");
    near(135, -23.01, "bus1 in R");
    near(136, -33.01, "bus1 out L");
    near(137, -33.01, "bus1 out R");
    // $BUS source group (32 legs at 145): legs 1/2 = bus 1 L/R, bus 2 silent
    near(145, -33.01, "source $BUS.1");
    near(146, -33.01, "source $BUS.2");
    assert_eq!(last[147], i16::MIN, "source $BUS.3 silent");
}

#[test]
fn gain_reduction_meter_matches_strip_telemetry() {
    let mut store = ParamStore::new();
    let levels = LevelBank::shared();
    let mut console = build(&mut store, Some(levels.clone()), 0);
    // compressor on ch 1: -30 dB threshold, 4:1
    set(&store, "ch/1/dyn/mdl", "COMP");
    set(&store, "ch/1/dyn/thr", "-30");
    set(&store, "ch/1/dyn/ratio", "4");
    set(&store, "ch/1/dyn/on", "1");
    let mut rx = console.pipeline().telemetry_rx().unwrap();
    let node = console.nodes().mix.inputs[0].0;
    let mut rm = RealMeters::new(levels.clone());
    rm.set_release(f32::INFINITY);
    let bank = MeterBank::for_schema();
    let mut n = 0u64;
    for _ in 0..400 {
        cycle(&mut console, &mut n);
    }
    for _ in 0..10 {
        while rx.pop().is_some() {}
        rm.update(0.05); // drop the previous interval
        for _ in 0..16 {
            cycle(&mut console, &mut n);
        }
        rm.update(0.05);
        rm.fill(&bank);
        let mut gr = None;
        while let Some(t) = rx.pop() {
            if t.node == node
                && t.block == wing_engine::channel::block::DYN
                && t.kind == telemetry_kind::GAIN_REDUCTION_DB
            {
                gr = Some(t.value);
            }
        }
        let gr = gr.expect("dyn GR telemetry");
        assert!(gr > 5.0, "compressor reduces: {gr}");
        let w = bank.get(MeterKind::ChannelV2, 0, 8).unwrap();
        let m = meter_to_gr(w, GR_FULL_SCALE_DB);
        assert!(
            (m - gr).abs() <= 0.5,
            "GR meter {m:.2} dB vs telemetry {gr:.2} dB"
        );
        assert_eq!(bank.get(MeterKind::ChannelV2, 0, 9), Some(1), "dyn state");
        assert_eq!(bank.get(MeterKind::Channel, 0, 7), Some(w), "V1 = V2");
    }
}

#[test]
fn metering_overhead_per_cycle() {
    let run = |meters: Option<Arc<LevelBank>>| {
        let mut store = ParamStore::new();
        let mut console = build(&mut store, meters, 0);
        // every channel fed and up, so the strip cells are written
        for i in 1..=40 {
            let src = ((i - 1) % 3 + 1).to_string();
            set(&store, &format!("ch/{i}/in/conn/grp"), "USB");
            set(&store, &format!("ch/{i}/in/conn/in"), &src);
            set(&store, &format!("ch/{i}/fdr"), "0");
            set(&store, &format!("ch/{i}/dyn/on"), "1");
        }
        let mut n = 0u64;
        for _ in 0..200 {
            cycle(&mut console, &mut n);
        }
        let mut best = Duration::MAX;
        for _ in 0..5 {
            let t0 = Instant::now();
            for _ in 0..400 {
                cycle(&mut console, &mut n);
            }
            best = best.min(t0.elapsed() / 400);
        }
        best
    };
    let off = run(None);
    let on = run(Some(LevelBank::shared()));
    let d = on.as_secs_f64() - off.as_secs_f64();
    let period = BLOCK as f64 / FS as f64;
    println!(
        "console cycle ({BLOCK} frames, 1 thread, {} build): {:.1} us without meters, {:.1} us \
         with (overhead {:.2} us = {:.2}% of the {:.0} us period)",
        if cfg!(debug_assertions) {
            "debug"
        } else {
            "release"
        },
        off.as_secs_f64() * 1e6,
        on.as_secs_f64() * 1e6,
        d * 1e6,
        d * 100.0 / period,
        period * 1e6
    );
}

/// Run `cycles` blocks, then one meter interval (`rm` sees only that interval), and fill `bank`.
fn settle(
    console: &mut Console,
    rm: &mut RealMeters,
    bank: &MeterBank,
    n: &mut u64,
    cycles: usize,
) {
    for _ in 0..cycles {
        cycle(console, n);
    }
    rm.update(0.05);
    for _ in 0..16 {
        cycle(console, n);
    }
    rm.update(0.05);
    rm.fill(bank);
}

/// T086: a DEQ (dynamic EQ) in the gate or dynamics slot fills the words WING-Edit 3.3.4 reads for
/// its DEQ page: `rxMeters` stores `gain word / 256` and `gui_modules::DynEqGraph::updateCurve`
/// draws the dynamic band at `15 x` that dB (so word = 256 * band dB / 15, clamped to +-256); the
/// LED / state word is 0 (`ae::dyn_led_dyneq` / `dyn_state_dyneq` return 0).
#[test]
fn deq_band_gain_words() {
    use wing_engine::meters::{band, band_cell};
    for slot in ["gate", "dyn"] {
        let mut store = ParamStore::new();
        let levels = LevelBank::shared();
        let mut console = build(&mut store, Some(levels.clone()), 0);
        // ch 1: 1 kHz at -14 dBFS peak; band compressor at 1 kHz, -6 dB maximum
        for (p, v) in [
            ("mdl", "DEQ"),
            ("on", "1"),
            ("thr", "-40"),
            ("ratio", "3.0"),
            ("filt", "BP"),
            ("f", "1000"),
            ("g", "-6"),
            ("mode", "high"),
        ] {
            set(&store, &format!("ch/1/{slot}/{p}"), v);
        }
        let store = Arc::new(store);
        let mut rm = RealMeters::new(levels.clone()).with_store(store.clone());
        rm.set_release(f32::INFINITY);
        let bank = MeterBank::for_schema();
        let mut n = 0u64;
        settle(&mut console, &mut rm, &bank, &mut n, 600);
        let (b, gain_w, led_w) = if slot == "gate" {
            (band::GATE, 5, 6)
        } else {
            (band::DYN, 8, 9)
        };
        let db = rm.value(band_cell(0, b));
        assert!((db + 6.0).abs() < 0.5, "{slot}: DEQ band at {db:.2} dB");
        let w = bank.get(MeterKind::ChannelV2, 0, gain_w).unwrap();
        assert_eq!(w, (256.0 * db / 15.0).round() as i16, "{slot} gain word");
        // as WING-Edit draws it
        assert!((w as f32 / 256.0 * 15.0 + 6.0).abs() < 0.5);
        assert_eq!(
            bank.get(MeterKind::ChannelV2, 0, led_w),
            Some(0),
            "{slot} LED/state"
        );
        let v1 = if slot == "gate" { 5 } else { 7 };
        assert_eq!(bank.get(MeterKind::Channel, 0, v1), Some(w), "V1 = V2");
    }
}

/// T086: DEQ3 (FX) and TAPE-DL as WING-Edit's FX pages read them: DEQ3 band k in FX word 4 + 2k as
/// `2048 (g_k - 1)` (`DynEqGraph::updateCurve`); TAPE-DL's two VU needles from FX words 2 / 3, the
/// slot's output level in 1/256 dB (`FxTapeDelay::listener_callback`: `x = word / 256 + 3`, needle
/// `668 + 551 (x + 3) / 6` from -3 up, `668 + 551 (x + 3) / 17` below, clamped to 117..1219).
#[test]
fn deq3_and_tape_delay_fx_words() {
    use wing_engine::meters::{fx_cell, fx_word};
    let mut store = ParamStore::new();
    let levels = LevelBank::shared();
    let mut console = build(&mut store, Some(levels.clone()), 0);
    // bus 1 (fed by ch 1 at -23 dB peak): Ins1 = FX7 TAPE-DL, Ins2 = FX10 DEQ3, as in the user's scene
    for (p, v) in [
        ("fx/7/mdl", "TAPE-DL"),
        ("fx/7/fxmix", "100"),
        ("fx/7/time", "100"),
        ("bus/1/preins/ins", "FX7"),
        ("bus/1/preins/on", "1"),
        ("fx/10/mdl", "DEQ3"),
        ("fx/10/fxmix", "100"),
        ("fx/10/1-thr", "-40"),
        ("fx/10/1-ratio", "3.0"),
        ("fx/10/1-filt", "BP"),
        ("fx/10/1-f", "1000"),
        ("fx/10/1-g", "-6"),
        ("fx/10/1-mode", "high"),
        ("bus/1/postins/ins", "FX10"),
        ("bus/1/postins/on", "1"),
    ] {
        set(&store, p, v);
    }
    let store = Arc::new(store);
    let mut rm = RealMeters::new(levels.clone()).with_store(store.clone());
    rm.set_release(f32::INFINITY);
    let bank = MeterBank::for_schema();
    let mut n = 0u64;
    settle(&mut console, &mut rm, &bank, &mut n, 1500);
    let w = |slot: usize, k: usize| bank.get(MeterKind::Fx, slot, k).unwrap();
    // TAPE-DL (FX 7): in = the bus, out = the delayed signal, both live
    for k in 0..4 {
        let db = meter_to_db(w(6, k));
        assert!(db > -40.0 && db < 0.0, "TAPE-DL word {k}: {db:.2} dB");
        assert_eq!(w(6, k), db_to_meter(rm.value(fx_cell(6, k))));
    }
    assert!((meter_to_db(w(6, fx_word::IN_L)) + 23.01).abs() < 0.5);
    assert_eq!([w(6, 4), w(6, 5), w(6, 9)], [0, 0, 0], "no state words");
    // DEQ3 (FX 10): band 1 near -6 dB in word 4, bands 2/3 (g = 0) at 0
    let band1 = rm.value(fx_cell(9, fx_word::BAND0));
    assert!((band1 + 6.0).abs() < 0.5, "DEQ3 band 1 at {band1:.2} dB");
    let want = (2048.0 * (10f32.powf(band1 / 20.0) - 1.0)).round() as i16;
    assert_eq!(w(9, 4), want);
    assert!(((w(9, 4) as f32 / 2048.0 + 1.0).log10() * 20.0 + 6.0).abs() < 0.5);
    assert_eq!((w(9, 6), w(9, 8)), (0, 0));
}
