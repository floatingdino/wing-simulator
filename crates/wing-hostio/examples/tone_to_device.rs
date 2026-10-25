//! Play a quiet 440 Hz tone through the full bridge (ring + drift-compensating resampler +
//! HAL IO proc) on a host device.
//!
//! `cargo run -p wing-hostio --example tone_to_device -- "Scarlett 2i4 USB" 1 2`
//!
//! Args: device name or UID, then 1-based device channels (default 1 2). Options:
//! `--seconds N` (default 3), `--dbfs X` (default -30, clamped to <= -12).
//!
//! A producer thread stands in for the engine: it paces itself on the host clock at 48 kHz
//! (64-frame blocks), as the simulator's virtual device would.

#[cfg(target_os = "macos")]
fn main() {
    use std::time::{Duration, Instant};
    use wing_hostio::coreaudio::{HostOutput, HostOutputConfig};
    use wing_hostio::{bridge, host_time_ns, BridgeConfig, BridgeState};

    let mut args: Vec<String> = std::env::args().skip(1).collect();
    let mut opt = |name: &str, default: f64| -> f64 {
        match args.iter().position(|a| a == name) {
            Some(i) if i + 1 < args.len() => {
                let v = args[i + 1].parse().unwrap_or(default);
                args.drain(i..=i + 1);
                v
            }
            _ => default,
        }
    };
    let seconds = opt("--seconds", 3.0).clamp(0.1, 30.0);
    let dbfs = opt("--dbfs", -30.0).min(-12.0);
    let Some(device) = args.first().cloned() else {
        eprintln!("usage: tone_to_device <device name|UID> [ch ...] [--seconds N] [--dbfs X]");
        std::process::exit(2);
    };
    let channels: Vec<u16> = if args.len() > 1 {
        args[1..]
            .iter()
            .map(|a| a.parse().expect("channel number"))
            .collect()
    } else {
        vec![1, 2]
    };

    const FS: f64 = 48_000.0;
    const BLOCK: usize = 64;
    let cfg = BridgeConfig {
        channels: channels.len(),
        source_rate: FS,
        max_block_frames: BLOCK,
        ..Default::default()
    };
    let (mut producer, consumer) = bridge(cfg);
    let out = match HostOutput::start(
        HostOutputConfig {
            device: device.clone(),
            device_channels: channels.clone(),
            buffer_frames: None,
        },
        consumer,
        Duration::from_secs(2),
    ) {
        Ok(o) => o,
        Err(e) => {
            eprintln!("error: {e}");
            std::process::exit(1);
        }
    };
    let info = out.device().expect("device open");
    println!(
        "playing 440 Hz at {dbfs:.0} dBFS on '{}' ({} Hz, {} frames) channels {:?} for {seconds} s",
        info.name, info.sample_rate, info.buffer_frames, channels
    );

    let amp = 10f64.powf(dbfs / 20.0);
    let fade = (0.02 * FS) as u64; // 20 ms raised-cosine fade in/out
    let total = (seconds * FS) as u64;
    let tail = (0.2 * FS) as u64; // silence so the end of the tone drains
    let mut buf = vec![0.0f32; BLOCK];
    let mut n = 0u64;
    let start = Instant::now();
    let mut next = start;
    let mut last_print = start;
    let stats = out.stats().clone();
    // Wait until the bridge primes before counting tone time (so all of it is heard).
    let mut playing_from: Option<u64> = None;
    loop {
        next += Duration::from_secs_f64(BLOCK as f64 / FS);
        let now = Instant::now();
        if next > now {
            std::thread::sleep(next - now);
        }
        let s = stats.snapshot();
        if playing_from.is_none() && s.state == BridgeState::Running {
            playing_from = Some(n);
        }
        let t0 = playing_from.unwrap_or(u64::MAX);
        for (i, x) in buf.iter_mut().enumerate() {
            let k = n + i as u64;
            let rel = k.saturating_sub(t0);
            let env = if k < t0 || rel >= total {
                0.0
            } else if rel < fade {
                0.5 - 0.5 * (std::f64::consts::PI * rel as f64 / fade as f64).cos()
            } else if rel > total - fade {
                0.5 - 0.5 * (std::f64::consts::PI * (total - rel) as f64 / fade as f64).cos()
            } else {
                1.0
            };
            *x = (amp * env * (2.0 * std::f64::consts::PI * 440.0 * k as f64 / FS).sin()) as f32;
        }
        producer.push_with(BLOCK, host_time_ns(), |_| &buf);
        n += BLOCK as u64;
        if last_print.elapsed() >= Duration::from_millis(500) {
            last_print = Instant::now();
            println!(
                "{:?} fill {:6.1} / {:6.1} fr ({:.2} ms)  corr {:+8.2} ppm  underruns {}  resyncs {}",
                s.state,
                s.fill_frames,
                s.target_frames,
                s.fill_ms(FS),
                s.correction_ppm,
                s.underruns,
                s.resyncs
            );
        }
        if n > t0.saturating_add(total + tail)
            || start.elapsed() > Duration::from_secs_f64(seconds + 5.0)
        {
            break;
        }
    }
    let s = stats.snapshot();
    println!(
        "done: {} callbacks, {} frames out, underruns {}, resyncs {}, overflow {}",
        s.callbacks, s.frames_out, s.underruns, s.resyncs, s.overflow_frames
    );
    drop(out);
}

#[cfg(not(target_os = "macos"))]
fn main() {
    eprintln!("CoreAudio is only available on macOS");
}
