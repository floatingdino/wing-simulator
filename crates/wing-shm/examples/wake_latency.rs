//! wake_latency: measures how late the daemon wakes after the driver signals
//! a cycle, for each wake mode.
//!
//! A "driver" thread (DriverSim on its own mapping) runs 128-frame cycles at
//! 48 kHz (2.67 ms); the "daemon" (DaemonLink on a second mapping of the same
//! shm name) waits for each cycle. Latency = daemon wake time minus the
//! driver's `driver_heartbeat`, which is stamped right before the signal.
//! Both threads are normal priority, so these are upper bounds for the
//! time-constraint threads T022 will use.
//!
//! ```text
//! cargo run --release -p wing-shm --example wake_latency [-- --cycles 1500]
//! ```

use std::sync::atomic::{AtomicBool, Ordering::Relaxed};
use std::sync::Arc;
use std::time::{Duration, Instant};

use wing_shm::clock;
use wing_shm::sim::DriverSim;
use wing_shm::{DaemonLink, OpenOptions, Segment, WakeMode, CHANNELS};

const BUF: usize = 128;
const RATE: u32 = 48_000;

fn measure(name: &str, mode: WakeMode, cycles: usize) -> (Vec<u64>, f64) {
    let _ = Segment::unlink(name);
    let create = OpenOptions {
        create: true,
        ..Default::default()
    };
    let drv = DriverSim::new(Segment::open(name, &create).unwrap(), RATE);
    let mut link = DaemonLink::attach(name, &OpenOptions::default(), mode).unwrap();
    let stop = Arc::new(AtomicBool::new(false));
    let stop2 = stop.clone();
    let period = Duration::from_nanos(1_000_000_000 * BUF as u64 / RATE as u64);

    let driver = std::thread::spawn(move || {
        let out = vec![0.0f32; BUF * CHANNELS];
        let start = Instant::now();
        let mut k = 0u32;
        while !stop2.load(Relaxed) {
            drv.write_output(k as i64 * BUF as i64, &out, BUF, CHANNELS);
            k += 1;
            let next = start + period * k;
            if let Some(d) = next.checked_duration_since(Instant::now()) {
                std::thread::sleep(d);
            }
        }
    });

    let mut lat = Vec::with_capacity(cycles);
    let mut buf = vec![0.0f32; 512];
    let cpu0 = cpu_time();
    let t0 = Instant::now();
    while lat.len() < cycles {
        link.heartbeat();
        if link.wait(Duration::from_millis(20)) {
            let now = clock::now_ticks();
            let hb = link.header().driver_heartbeat.load(Relaxed);
            lat.push(clock::ticks_to_ns(now.saturating_sub(hb)));
        }
        while let Some(b) = link.next_block(buf.len()) {
            link.read_input(&b, 0, &mut buf[..b.frames]);
            link.commit(&b);
        }
    }
    let cpu = (cpu_time() - cpu0) / t0.elapsed().as_secs_f64();
    stop.store(true, Relaxed);
    driver.join().unwrap();
    drop(link);
    let _ = Segment::unlink(name);
    lat.sort_unstable();
    (lat, cpu)
}

/// Process CPU seconds (user + system), for a rough busy-ness comparison.
fn cpu_time() -> f64 {
    // SAFETY: zeroed rusage is a valid out buffer.
    let mut ru: libc::rusage = unsafe { std::mem::zeroed() };
    unsafe { libc::getrusage(libc::RUSAGE_SELF, &mut ru) };
    let tv = |t: libc::timeval| t.tv_sec as f64 + t.tv_usec as f64 / 1e6;
    tv(ru.ru_utime) + tv(ru.ru_stime)
}

fn main() {
    let mut cycles = 1500usize;
    let args: Vec<String> = std::env::args().skip(1).collect();
    if let Some(i) = args.iter().position(|a| a == "--cycles") {
        cycles = args
            .get(i + 1)
            .and_then(|v| v.parse().ok())
            .unwrap_or(cycles);
    }
    let name = format!("/wingsim.lat.{}", std::process::id());
    println!(
        "{cycles} cycles of {BUF} frames @ {RATE} Hz; latency from driver signal to daemon wake"
    );
    println!(
        "{:<28} {:>8} {:>8} {:>8} {:>8}  {:>9}",
        "mode", "p50 us", "p90 us", "p99 us", "max us", "CPU (%)"
    );
    let modes = [
        (
            "os_sync",
            WakeMode::OsSync {
                spin: Duration::ZERO,
            },
        ),
        (
            "os_sync + 50us spin",
            WakeMode::OsSync {
                spin: Duration::from_micros(50),
            },
        ),
        (
            "poll nanosleep 100us",
            WakeMode::Poll {
                interval: Duration::from_micros(100),
            },
        ),
        (
            "poll nanosleep 250us",
            WakeMode::Poll {
                interval: Duration::from_micros(250),
            },
        ),
        (
            "poll nanosleep 1ms",
            WakeMode::Poll {
                interval: Duration::from_millis(1),
            },
        ),
    ];
    for (label, mode) in modes {
        let (lat, cpu) = measure(&name, mode, cycles);
        let pct = |p: f64| lat[((lat.len() - 1) as f64 * p) as usize] as f64 / 1000.0;
        println!(
            "{label:<28} {:>8.1} {:>8.1} {:>8.1} {:>8.1}  {:>9.1}",
            pct(0.5),
            pct(0.9),
            pct(0.99),
            pct(1.0),
            cpu * 100.0
        );
    }
}
