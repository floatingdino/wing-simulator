//! Host output bridge: plays simulator outputs on a real CoreAudio device (e.g. a Scarlett),
//! absorbing the clock drift between the virtual "Wing Simulator" device and the hardware.
//!
//! ```text
//!  engine thread (virtual-device clock)          HAL IO thread (hardware clock)
//!  BridgeProducer::push_with(block) --SPSC ring--> BridgeConsumer::render(n frames)
//!                                                   ├─ fill estimate (burst stamps, sawtooth removed)
//!                                                   ├─ drift estimate + bounded phase correction
//!                                                   └─ polyphase sinc resampler -> device buffers
//! ```
//!
//! * [`ring`]: lock-free SPSC planar ring (wait-free both sides).
//! * [`resampler`]: variable-ratio polyphase windowed-sinc resampler, RT-safe.
//! * [`control`]: drift controller (ring fill -> ratio): frequency-locked drift estimate plus a
//!   bounded, deadbanded phase correction; clamped and slew-limited.
//! * [`bridge`]: ring + resampler + controller + lock-free [`BridgeStats`]; portable and
//!   tested with simulated drifting clocks (`tests/drift.rs`).
//! * [`config`]: route mapping from console taps (`mon/1 -> "Scarlett 2i4 USB" 1/2`,
//!   `main/1`, `bus/n`, `usb_out/45-46`) to device channels, and TOML parsing.
//! * [`coreaudio`] (macOS): device enumeration and [`coreaudio::HostOutput`], a HAL IO-proc
//!   client with automatic reconnect and sample-rate / buffer-size change handling.
//!
//! CoreAudio access is hand-written FFI against the HAL C API (`AudioDeviceCreateIOProcID`
//! etc.) rather than coreaudio-rs/coreaudio-sys (bindgen + libclang at build time) or AUHAL:
//! the IO proc is the lowest-latency path, gives direct multi-stream buffer access for
//! arbitrary channel mapping, and the device's IO thread is already in its `os_workgroup`.
//!
//! Examples: `cargo run -p wing-hostio --example list_devices`,
//! `cargo run -p wing-hostio --example tone_to_device -- "Scarlett 2i4 USB" 1 2`.

pub mod bridge;
pub mod config;
pub mod control;
#[cfg(target_os = "macos")]
pub mod coreaudio;
pub mod resampler;
pub mod ring;

pub use bridge::{
    bridge, BridgeConfig, BridgeConsumer, BridgeProducer, BridgeState, BridgeStats, StatsSnapshot,
};
pub use config::{ConfigError, DevicePlan, HostIoConfig, Route, Tap, TapChannel, EXAMPLE_TOML};
pub use control::DriftConfig;
pub use resampler::{Resampler, ResamplerConfig};

/// Host time in nanoseconds: the clock CoreAudio timestamps use (`mach_absolute_time`) on
/// macOS, a process-local monotonic clock elsewhere. Producers stamp pushes with it.
pub fn host_time_ns() -> u64 {
    #[cfg(target_os = "macos")]
    {
        coreaudio::host_time_ns()
    }
    #[cfg(not(target_os = "macos"))]
    {
        use std::sync::OnceLock;
        use std::time::Instant;
        static T0: OnceLock<Instant> = OnceLock::new();
        T0.get_or_init(Instant::now).elapsed().as_nanos() as u64 + 1
    }
}
