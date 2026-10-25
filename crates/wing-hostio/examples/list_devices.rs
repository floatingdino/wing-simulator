//! List CoreAudio devices with their output channels, rate and buffer size.
//!
//! `cargo run -p wing-hostio --example list_devices [-- --all]`

#[cfg(target_os = "macos")]
fn main() {
    use wing_hostio::coreaudio;

    let all = std::env::args().any(|a| a == "--all");
    let devs = if all {
        coreaudio::list_devices()
    } else {
        coreaudio::list_output_devices()
    }
    .unwrap_or_else(|e| {
        eprintln!("error: {e}");
        std::process::exit(1)
    });
    println!(
        "{:>4}  {:<32} {:>4} {:>4} {:>8} {:>6} {:>7}  UID",
        "id", "name", "out", "in", "rate", "buf", "lat"
    );
    for d in devs {
        println!(
            "{:>4}  {:<32} {:>4} {:>4} {:>8.0} {:>6} {:>7}  {}",
            d.id,
            d.name,
            d.output_channels,
            d.input_channels,
            d.sample_rate,
            d.buffer_frames,
            coreaudio::output_latency_frames(d.id),
            d.uid
        );
    }
}

#[cfg(not(target_os = "macos"))]
fn main() {
    eprintln!("CoreAudio is only available on macOS");
}
