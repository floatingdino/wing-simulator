//! T061: recording console outputs while the console runs.
//!
//! * `records_taps_bit_exact_without_rt_allocation`: a console with the recorder slot attached;
//!   start (control side) → the console installs the tap on its next cycle → 200 cycles with a
//!   counting allocator on the "RT" thread (zero allocations/frees) → stop → the WAV files equal
//!   the console's ports sample for sample.
//! * `stop_while_never_installed_and_restart`: arm without any console cycling, stop (taken back
//!   on the control side), restart over a running recording.
//! * `osc_record_commands_end_to_end`: the OSC server on an ephemeral port with the recorder as
//!   its `/wingsim/` extension; start/status/stop over UDP.

use std::alloc::{GlobalAlloc, Layout, System};
use std::net::{Ipv4Addr, SocketAddrV4, UdpSocket};
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering::Relaxed};
use std::sync::Arc;
use std::time::Duration;

use wing_hostio::{Tap, TapChannel};
use wing_model::ParamStore;
use wing_proto::osc::{Arg, Message, OscAttachment, OscConfig, OscServer};
use wing_sim::audio::console::{Console, ConsoleConfig};
use wing_sim::audio::stats::AudioStats;
use wing_sim::audio::{prepare, ConsoleControl};
use wing_sim::record::Recorder;
use wing_sim::wav::read_all;

struct Counting;
static COUNTING: AtomicBool = AtomicBool::new(false);
static ALLOCS: AtomicUsize = AtomicUsize::new(0);
thread_local! {
    static COUNT_THIS_THREAD: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}
fn counted() -> bool {
    COUNTING.load(Relaxed) && COUNT_THIS_THREAD.try_with(|c| c.get()).unwrap_or(false)
}
unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, l: Layout) -> *mut u8 {
        if counted() {
            ALLOCS.fetch_add(1, Relaxed);
        }
        System.alloc(l)
    }
    unsafe fn dealloc(&self, p: *mut u8, l: Layout) {
        if counted() {
            ALLOCS.fetch_add(1, Relaxed);
        }
        System.dealloc(p, l)
    }
    unsafe fn realloc(&self, p: *mut u8, l: Layout, n: usize) -> *mut u8 {
        if counted() {
            ALLOCS.fetch_add(1, Relaxed);
        }
        System.realloc(p, l, n)
    }
}
#[global_allocator]
static GLOBAL: Counting = Counting;

const FS: u32 = 48_000;
const BLOCK: usize = 64;

fn set(store: &ParamStore, path: &str, v: &str) {
    let id = store.lookup(path).unwrap();
    store.set_text(id, v, 0).unwrap();
}

/// ch 1 <- USB 1 (mono) -> main 1; USB Out 3 = USB In 3.
fn scene(store: &ParamStore) {
    for (p, v) in [
        ("ch/1/in/conn/grp", "USB"),
        ("ch/1/in/conn/in", "1"),
        ("io/in/USB/1/mute", "0"),
        ("io/in/USB/1/mode", "M"),
        ("io/in/USB/3/mute", "0"),
        ("ch/1/fdr", "0"),
        ("ch/1/main/1/on", "1"),
        ("ch/1/main/1/lvl", "0"),
        ("main/1/fdr", "0"),
        ("main/1/mute", "0"),
        ("io/out/USB/3/grp", "USB"),
        ("io/out/USB/3/in", "3"),
    ] {
        set(store, p, v);
    }
}

fn rig() -> (ParamStore, ConsoleControl) {
    let mut store = ParamStore::new();
    let mut ctl = prepare(&mut store);
    scene(&store);
    ctl.refresh(&store);
    (store, ctl)
}

fn console(store: &ParamStore, ctl: &mut ConsoleControl) -> Console {
    Console::build(
        ConsoleConfig {
            sample_rate: FS,
            block: BLOCK,
            workers: 2,
            rt_workers: false,
            workgroup: None,
            meters: None,
        },
        store,
        ctl,
        None,
        Vec::new(),
        Arc::new(AudioStats::default()),
    )
    .unwrap()
}

fn sig(n: usize, ch: usize) -> f32 {
    ((n as f32) * (0.013 + ch as f32 * 0.007)).sin() * 0.3
}

fn tmpdir(tag: &str) -> PathBuf {
    let d = std::env::temp_dir().join(format!("wingsim-rec-{tag}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&d);
    d
}

#[test]
fn records_taps_bit_exact_without_rt_allocation() {
    let (store, mut ctl) = rig();
    let mut c = console(&store, &mut ctl);
    let rec = Recorder::new();
    c.attach_recorder(rec.slot().clone());
    c.warm_up(4);
    let dir = tmpdir("exact");
    let msg = rec.start("usb_out/3,main/1", &dir).unwrap();
    assert!(
        msg.contains("usb_out_03") && msg.contains("main_1"),
        "{msg}"
    );
    assert!(rec.status().starts_with("rec armed"), "{}", rec.status());

    let main_l = c
        .nodes()
        .tap_port(TapChannel {
            tap: Tap::Main(1),
            lane: 1,
        })
        .unwrap();
    let main_r = c
        .nodes()
        .tap_port(TapChannel {
            tap: Tap::Main(1),
            lane: 2,
        })
        .unwrap();
    const CYCLES: usize = 200;
    let mut exp_usb = vec![0.0f32; CYCLES * BLOCK];
    let mut exp_main = vec![0.0f32; CYCLES * BLOCK * 2];
    COUNT_THIS_THREAD.with(|t| t.set(true));
    COUNTING.store(true, Relaxed);
    for k in 0..CYCLES {
        let t0 = k * BLOCK;
        c.cycle(
            |ch, buf| {
                for (i, s) in buf.iter_mut().enumerate() {
                    *s = if ch < 4 { sig(t0 + i, ch) } else { 0.0 };
                }
            },
            |_, _| {},
        );
        exp_usb[t0..t0 + BLOCK].copy_from_slice(c.usb_out(2));
        let (l, r) = (c.port(main_l), c.port(main_r));
        for i in 0..BLOCK {
            exp_main[(t0 + i) * 2] = l[i];
            exp_main[(t0 + i) * 2 + 1] = r[i];
        }
    }
    COUNTING.store(false, Relaxed);
    COUNT_THIS_THREAD.with(|t| t.set(false));
    assert_eq!(ALLOCS.load(Relaxed), 0, "allocations on the RT path");
    assert!(rec.status().starts_with("rec "), "{}", rec.status());
    let sum = rec.stop();
    assert!(sum.starts_with("recorded"), "{sum}");
    assert!(!sum.contains("DROPPED"), "{sum}");
    assert!(!rec.is_recording());

    let (info, usb) = read_all(&dir.join("usb_out_03.wav")).unwrap();
    assert_eq!((info.channels, info.sample_rate, info.float), (1, FS, true));
    assert_eq!(usb, exp_usb, "usb_out/3 recording");
    // USB Out 3 is patched straight to USB In 3
    assert_eq!(usb[1000], sig(1000, 2));
    let (info, main) = read_all(&dir.join("main_1.wav")).unwrap();
    assert_eq!(info.channels, 2);
    assert_eq!(main, exp_main, "main/1 recording");
    assert!(main.iter().any(|&v| v.abs() > 0.05), "main is silent");
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn stop_while_never_installed_and_restart() {
    let rec = Recorder::new();
    let d1 = tmpdir("armed");
    rec.start("bus/2", &d1).unwrap();
    assert!(rec.is_recording());
    let s = rec.stop();
    assert!(s.starts_with("recorded 0.0 s"), "{s}");
    assert_eq!(read_all(&d1.join("bus_2.wav")).unwrap().0.frames, 0);
    assert!(rec.status().starts_with("rec off (last: recorded"));
    assert!(rec.start("main/9", &d1).is_err());

    // start over a running recording: the first one is finalized, the console switches taps
    let (store, mut ctl) = rig();
    let mut c = console(&store, &mut ctl);
    c.attach_recorder(rec.slot().clone());
    let d2 = tmpdir("re");
    rec.start("usb_out/3", &d1).unwrap();
    for _ in 0..10 {
        c.cycle(|_, b| b.fill(0.25), |_, _| {});
    }
    rec.start("usb_out/3", &d2).unwrap();
    // nothing serves the stop of the first recording while the console is idle; the slot then
    // hands the old tap back when the new one is installed
    for _ in 0..5 {
        c.cycle(|_, b| b.fill(0.5), |_, _| {});
    }
    let s = rec.stop();
    assert!(s.starts_with("recorded"), "{s}");
    let (_, a) = read_all(&d1.join("usb_out_03.wav")).unwrap();
    let (_, b) = read_all(&d2.join("usb_out_03.wav")).unwrap();
    assert_eq!(a.len(), 10 * BLOCK);
    assert!(a.iter().all(|&v| v == 0.25));
    assert_eq!(b.len(), 5 * BLOCK);
    assert!(b.iter().all(|&v| v == 0.5));
    for d in [d1, d2] {
        let _ = std::fs::remove_dir_all(d);
    }
}

#[test]
fn osc_record_commands_end_to_end() {
    let mut store = ParamStore::new();
    let att = OscAttachment::attach(&mut store);
    let rec = Recorder::new();
    let srv = OscServer::bind(
        Arc::new(store),
        att,
        OscConfig {
            bind: SocketAddrV4::new(Ipv4Addr::LOCALHOST, 0),
            extension: Some(rec.clone()),
            ..OscConfig::default()
        },
    )
    .unwrap();
    let addr = srv.local_addr().unwrap();
    let h = srv.spawn().unwrap();
    let cl = UdpSocket::bind("127.0.0.1:0").unwrap();
    cl.set_read_timeout(Some(Duration::from_secs(5))).unwrap();
    let ask = |m: Message| -> String {
        cl.send_to(&m.to_vec(), addr).unwrap();
        let mut buf = [0u8; 2048];
        let (n, _) = cl.recv_from(&mut buf).unwrap();
        let r = Message::decode(&buf[..n]).unwrap();
        assert_eq!(r.addr, "/wingsim/record");
        match &r.args[0] {
            Arg::Str(s) => s.clone(),
            a => panic!("{a:?}"),
        }
    };
    let d = tmpdir("osc");
    let r = ask(Message::new(
        "/wingsim/record/start",
        vec![
            Arg::Str("main/1,usb_out/1-2".into()),
            Arg::Str(d.display().to_string()),
        ],
    ));
    assert!(r.starts_with("recording 3 file(s)"), "{r}");
    assert!(rec.is_recording());
    let r = ask(Message::new("/wingsim/record/status", vec![]));
    assert!(r.starts_with("rec armed"), "{r}");
    let r = ask(Message::new("/wingsim/record/start", vec![]));
    assert!(r.starts_with("ERROR usage"), "{r}");
    let r = ask(Message::new("/wingsim/record/stop", vec![]));
    assert!(r.starts_with("recorded"), "{r}");
    for f in ["main_1.wav", "usb_out_01.wav", "usb_out_02.wav"] {
        assert!(d.join(f).exists(), "{f}");
    }
    h.stop();
    let _ = std::fs::remove_dir_all(&d);
}
