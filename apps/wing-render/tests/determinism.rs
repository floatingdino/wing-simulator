//! T060 acceptance: an offline render is bit-identical to the realtime path for the same input.
//!
//! Realtime path: the daemon's real `AudioService` (control thread + RT driver thread + engine
//! workers) attached to `wing_shm::sim::DriverSim` on a private shm name (never the user's
//! `/wingsim.audio.v1`), fed lock-step with the same input plan, capturing USB Out 1..48 from the
//! driver side. Offline path: `wing_render::render` writing `usb_out/1-48` and `main/1`.
//! Scene: `fixtures/PUBLIC.snap` (full processing) switched to USB inputs (virtual soundcheck),
//! with USB Out 45..48 patched to MAIN 1 and BUS 1, so the comparison covers the whole mix.
//!
//! Also: the render is identical for 0 and 3 engine worker threads, and the daemon's status line
//! carries the recorder state (T061) while recording.

use std::path::{Path, PathBuf};
use std::sync::atomic::Ordering::Relaxed;
use std::sync::Arc;
use std::time::{Duration, Instant};

use wing_model::ParamStore;
use wing_render::inputs::{plan_from_files, InputPlan, InputReader, USB_INPUTS};
use wing_render::{load_scene, render, RenderConfig};
use wing_shm::sim::DriverSim;
use wing_shm::{OpenOptions, Segment, CHANNELS};
use wing_sim::audio::console::ConsoleConfig;
use wing_sim::audio::driver::{AudioOptions, AudioService};
use wing_sim::audio::prepare;
use wing_sim::record::parse_tap_groups;
use wing_sim::wav::{read_all, write_pcm, WavWriter};

const FS: u32 = 48_000;
const FRAMES: usize = 48_000 + 1_000; // ~1 s, not a multiple of the block

fn set(store: &ParamStore, path: &str, v: &str) {
    let id = store
        .lookup(path)
        .unwrap_or_else(|| panic!("no such node {path}"));
    store
        .set_text(id, v, 0)
        .unwrap_or_else(|e| panic!("{path} = {v}: {e:?}"));
}

fn scene() -> ParamStore {
    let snap = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../fixtures/PUBLIC.snap");
    let (store, _) = load_scene(Some(&snap)).unwrap();
    // PUBLIC.snap mutes DCAs 5 and 8, whose members (busses 1, 2, 5, 7: the group busses the
    // first channels feed) then have their main sends muted (T03F DCA semantics): unmute them so
    // main 1 carries signal
    set(&store, "dca/5/mute", "0");
    set(&store, "dca/8/mute", "0");
    for ch in 1..=40 {
        set(&store, &format!("ch/{ch}/in/conn/grp"), "USB");
        set(&store, &format!("ch/{ch}/in/conn/in"), &ch.to_string());
    }
    for (o, grp, n) in [
        (45, "MAIN", 1),
        (46, "MAIN", 2),
        (47, "BUS", 1),
        (48, "BUS", 2),
    ] {
        set(&store, &format!("io/out/USB/{o}/grp"), grp);
        set(&store, &format!("io/out/USB/{o}/in"), &n.to_string());
    }
    store
}

fn tmp(tag: &str) -> PathBuf {
    let d = std::env::temp_dir().join(format!("wingrender-det-{tag}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&d);
    std::fs::create_dir_all(&d).unwrap();
    d
}

/// Six numbered sources: mono 24-bit PCM and float, one stereo float on USB 5/6, noise-like
/// content with transients so gates/compressors move.
fn inputs(dir: &Path) -> InputPlan {
    let mut seed = 0x1234_5678u32;
    let mut noise = move || {
        seed ^= seed << 13;
        seed ^= seed >> 17;
        seed ^= seed << 5;
        (seed as f32 / u32::MAX as f32) * 2.0 - 1.0
    };
    for (name, f) in [
        ("01-Kick.wav", 55.0f32),
        ("02-Snare.wav", 190.0),
        ("03-Bass.wav", 82.0),
        ("09-Vox.wav", 440.0),
    ] {
        let s: Vec<f32> = (0..FRAMES)
            .map(|n| {
                let env = if (n / 6000) % 2 == 0 { 0.6 } else { 0.05 };
                env * ((n as f32 * f * std::f32::consts::TAU / FS as f32).sin() * 0.8
                    + noise() * 0.2)
            })
            .collect();
        write_pcm(&dir.join(name), 1, FS, 24, &s).unwrap();
    }
    let st: Vec<f32> = (0..FRAMES * 2).map(|_| noise() * 0.3).collect();
    let mut w = WavWriter::create(&dir.join("05-Keys.wav"), 2, FS).unwrap();
    w.write_interleaved(&st).unwrap();
    w.finish().unwrap();
    let plan = plan_from_files(&[dir.to_path_buf()]).unwrap();
    assert_eq!(plan.sources.len(), 5);
    plan
}

fn offline(plan: &InputPlan, out: &Path, threads: usize) -> wing_render::RenderReport {
    let cfg = RenderConfig {
        rate: FS,
        block: 64,
        threads,
        start: 0,
        end: None,
        tail: 0,
        taps: parse_tap_groups("usb_out/1-48,main/1").unwrap(),
        out_dir: out.to_path_buf(),
    };
    render(scene(), plan, &cfg, |_| {}).unwrap()
}

struct TmpShm(String);
impl Drop for TmpShm {
    fn drop(&mut self) {
        let _ = Segment::unlink(&self.0);
    }
}

fn wait_for(what: &str, timeout: Duration, mut f: impl FnMut() -> bool) {
    let t0 = Instant::now();
    while !f() {
        assert!(t0.elapsed() < timeout, "timed out waiting for {what}");
        std::thread::sleep(Duration::from_millis(2));
    }
}

/// The daemon's realtime path; returns USB Out 1..48 (planar) for the first `FRAMES` frames.
fn realtime(plan: &InputPlan, rec_dir: &Path) -> Vec<Vec<f32>> {
    let shm = TmpShm(format!("/wsr.det.{}", std::process::id()));
    let _ = Segment::unlink(&shm.0);
    let create = OpenOptions {
        create: true,
        ..Default::default()
    };
    let drv = DriverSim::new(Segment::open(&shm.0, &create).unwrap(), FS);
    let mut store = scene();
    let routing = prepare(&mut store);
    let svc = AudioService::spawn(
        Arc::new(store),
        routing,
        AudioOptions {
            shm_name: shm.0.clone(),
            console: ConsoleConfig {
                sample_rate: FS,
                block: 64,
                workers: 2,
                rt_workers: false,
                workgroup: None,
                meters: None,
                rta: None,
                probe: None,
            },
            rt: false,
            hostio: None,
            status_every: None,
            device_factory: None,
        },
        Arc::new(|m: &str| eprintln!("{m}")),
    )
    .unwrap();
    wait_for("attach", Duration::from_secs(60), || {
        drv.daemon_alive() && svc.status_line().starts_with("audio: 48000 Hz")
    });
    assert!(
        svc.status_line().contains("rec off"),
        "{}",
        svc.status_line()
    );
    svc.recorder().start("main/1", rec_dir).unwrap();

    let h = drv.segment().header();
    let delay = h.input_delay_frames as usize;
    let mut reader = InputReader::open(plan, 0).unwrap();
    let mut usb = vec![vec![0.0f32; 4096]; USB_INPUTS];
    let mut play = |n: usize| -> Vec<f32> {
        reader.read(&mut usb, n).unwrap();
        let mut v = vec![0.0f32; n * CHANNELS];
        for f in 0..n {
            for c in 0..USB_INPUTS {
                v[f * CHANNELS + c] = usb[c][f];
            }
        }
        v
    };
    // like the HAL: playback runs `delay` frames ahead of capture
    drv.write_output(0, &play(delay), delay, CHANNELS);
    const BUF: usize = 128;
    let mut rec = vec![0.0f32; BUF * CHANNELS];
    let mut out: Vec<Vec<f32>> = (0..USB_INPUTS)
        .map(|_| Vec::with_capacity(FRAMES + BUF))
        .collect();
    let mut t = 0i64;
    while out[0].len() < FRAMES {
        let at = t + delay as i64;
        drv.write_output(at, &play(BUF), BUF, CHANNELS);
        // Unpaced lock-step feed: a debug-build PUBLIC.snap runs slower than realtime, so the
        // daemon stays in one long `service` burst; its per-block heartbeat must keep the
        // simulated driver out of loopback (100 ms timeout).
        wait_for("daemon block", Duration::from_secs(20), || {
            h.from_daemon.write_time.load(Relaxed) >= t + BUF as i64
        });
        drv.read_input(at, &mut rec, BUF, CHANNELS);
        assert_eq!(
            h.loopback_active.load(Relaxed),
            0,
            "driver in loopback at {t}"
        );
        for f in 0..BUF {
            for (c, o) in out.iter_mut().enumerate() {
                o.push(rec[f * CHANNELS + c]);
            }
        }
        t += BUF as i64;
    }
    assert_eq!(h.from_daemon.underruns.load(Relaxed), 0);
    let line = svc.status_line();
    assert!(line.contains("rec ") && line.contains("file(s)"), "{line}");
    let sum = svc.recorder().stop();
    assert!(sum.starts_with("recorded"), "{sum}");
    svc.stop();
    for o in &mut out {
        o.truncate(FRAMES);
    }
    out
}

#[test]
fn offline_render_is_bit_identical_to_the_realtime_path() {
    let dir = tmp("in");
    let plan = inputs(&dir);
    let out_a = tmp("out-a");
    let rep = offline(&plan, &out_a, 3);
    assert_eq!(rep.frames, FRAMES as u64);
    let main = rep.taps.iter().find(|t| t.name == "main_1").unwrap();
    assert!(main.peak_dbfs > -60.0, "main is silent: {main:?}");

    // engine thread count does not change a bit
    let out_b = tmp("out-b");
    offline(&plan, &out_b, 0);
    for t in &rep.taps {
        let a = read_all(&t.file).unwrap().1;
        let b = read_all(&out_b.join(t.file.file_name().unwrap()))
            .unwrap()
            .1;
        assert!(a == b, "{}: 3 vs 0 workers differ", t.name);
    }

    let rec_dir = tmp("rec");
    let rt = realtime(&plan, &rec_dir);
    let mut nonzero = 0;
    for (c, rt_c) in rt.iter().enumerate() {
        let (info, off) = read_all(&out_a.join(format!("usb_out_{:02}.wav", c + 1))).unwrap();
        assert_eq!(info.frames as usize, FRAMES);
        if let Some(i) = (0..FRAMES).find(|&i| off[i].to_bits() != rt_c[i].to_bits()) {
            panic!(
                "USB Out {}: offline != realtime at frame {i} ({} vs {})",
                c + 1,
                off[i],
                rt_c[i]
            );
        }
        if off.iter().any(|&v| v != 0.0) {
            nonzero += 1;
        }
    }
    // USB Out 45/46 (MAIN 1) carry the mix; outs patched to USB In 1 etc. carry the inputs
    assert!(nonzero >= 3, "only {nonzero} non-silent outputs");
    let (_, m) = read_all(&out_a.join("main_1.wav")).unwrap();
    let l: Vec<f32> = m.iter().step_by(2).copied().collect();
    assert_eq!(l, rt[44], "main_1.wav L != USB Out 45");
    // the T061 recording of the running daemon is non-empty and readable
    let (info, recd) = read_all(&rec_dir.join("main_1.wav")).unwrap();
    assert_eq!(info.channels, 2);
    assert!(!recd.is_empty());
    for d in [dir, out_a, out_b, rec_dir] {
        let _ = std::fs::remove_dir_all(d);
    }
}
