//! gain_client: reference daemon side of the shm transport (Rust port of
//! `driver/tools/wing_shm_client.cpp`).
//!
//! Attaches to the driver's shared memory and, for every frame the DAW plays
//! on "USB In" c, writes `gain * sample` to "USB Out" c (the DAW's record
//! inputs). Default gain is -1 (polarity invert). Stop it and the driver falls
//! back to loopback immediately (heartbeat cleared on exit).
//!
//! ```text
//! cargo run -p wing-shm --example gain_client -- [--name /wingsim.audio.v1]
//!     [--gain -1.0] [--seconds 0] [--create] [--status] [--poll-us N] [--block N]
//! ```
//!
//! Unlike the C++ client it is woken by the driver (os_sync_wait_on_address on
//! `cycle_seq`) instead of polling; `--poll-us` forces nanosleep polling. It
//! still runs on a normal-priority thread (T022 adds the RT thread/workgroup).
//! `tests/cpp_selftest.rs` runs this code against the C++ driver selftest.

use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use wing_shm::{DaemonLink, OpenOptions, Segment, WakeMode, CHANNELS, SHM_NAME};

static STOP: AtomicBool = AtomicBool::new(false);

extern "C" fn on_signal(_: libc::c_int) {
    STOP.store(true, Ordering::Relaxed);
}

fn usage() -> i32 {
    eprintln!(
        "usage: gain_client [--name NAME] [--gain G] [--seconds S] [--create] [--status] \
         [--poll-us N] [--block N]"
    );
    2
}

/// Entry point shared with `tests/cpp_selftest.rs`. Returns the exit code.
pub fn run<I: IntoIterator<Item = String>>(args: I) -> i32 {
    let mut name = SHM_NAME.to_string();
    let mut gain = -1.0f32;
    let mut seconds = 0.0f64;
    let mut create = false;
    let mut status = false;
    let mut wake = WakeMode::Auto;
    let mut block = 512usize;

    let mut it = args.into_iter();
    while let Some(a) = it.next() {
        let mut val = || it.next();
        match a.as_str() {
            "--name" => match val() {
                Some(v) => name = v,
                None => return usage(),
            },
            "--gain" => match val().and_then(|v| v.parse().ok()) {
                Some(v) => gain = v,
                None => return usage(),
            },
            "--seconds" => match val().and_then(|v| v.parse().ok()) {
                Some(v) => seconds = v,
                None => return usage(),
            },
            "--poll-us" => match val().and_then(|v| v.parse().ok()) {
                Some(v) => {
                    wake = WakeMode::Poll {
                        interval: Duration::from_micros(v),
                    }
                }
                None => return usage(),
            },
            "--block" => match val().and_then(|v| v.parse().ok()) {
                Some(v) if v > 0 => block = v,
                _ => return usage(),
            },
            "--create" => create = true,
            "--status" => status = true,
            _ => return usage(),
        }
    }

    let opts = OpenOptions {
        create,
        ..Default::default()
    };
    if status {
        return match Segment::open(&name, &opts) {
            Ok(seg) => {
                print!("{}", seg.status());
                0
            }
            Err(e) => {
                eprintln!("gain_client: {name}: {e}");
                1
            }
        };
    }

    let mut link = match DaemonLink::attach(&name, &opts, wake) {
        Ok(l) => l,
        Err(e) => {
            eprintln!("gain_client: {name}: {e} (use --create to create it)");
            return 1;
        }
    };
    // SAFETY: installing an async-signal-safe handler (one atomic store).
    unsafe {
        libc::signal(libc::SIGINT, on_signal as *const () as libc::sighandler_t);
        libc::signal(libc::SIGTERM, on_signal as *const () as libc::sighandler_t);
    }
    eprintln!(
        "gain_client: attached {name}, gain {gain:.3}, {CHANNELS} ch @ {} Hz, wake {:?}",
        link.stats().sample_rate,
        link.mechanism()
    );

    // Everything the loop needs is allocated here, before it starts.
    let mut buf = vec![0.0f32; block];
    let start = Instant::now();
    let mut last_report = start;

    while !STOP.load(Ordering::Relaxed) {
        link.heartbeat();
        link.wait(Duration::from_millis(10));
        while let Some(b) = link.next_block(buf.len()) {
            let x = &mut buf[..b.frames];
            for ch in 0..CHANNELS {
                link.read_input(&b, ch, x);
                for s in x.iter_mut() {
                    *s *= gain;
                }
                link.write_output(&b, ch, x);
            }
            link.commit(&b);
        }

        if seconds > 0.0 && start.elapsed().as_secs_f64() > seconds {
            break;
        }
        if last_report.elapsed() > Duration::from_secs(5) {
            last_report = Instant::now();
            let s = link.stats();
            eprintln!(
                "gain_client: t={} epoch={} underruns={} overruns={}",
                s.cursor, s.epoch, s.underruns, s.overruns
            );
        }
    }

    drop(link); // detach: heartbeat 0 -> driver loopback immediately
    eprintln!("gain_client: detached");
    0
}

#[allow(dead_code)]
fn main() {
    std::process::exit(run(std::env::args().skip(1)));
}
