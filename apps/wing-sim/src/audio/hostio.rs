//! Host output bridges for the daemon: which config to use, and starting/stopping the bridges
//! together with an audio session.

use std::sync::Arc;
#[cfg(target_os = "macos")]
use std::time::Duration;

use wing_hostio::{
    bridge, BridgeConfig, BridgeConsumer, BridgeProducer, BridgeStats, HostIoConfig, Route,
    TapChannel,
};

/// Device used by the default route when present.
pub const DEFAULT_DEVICE: &str = "Scarlett 2i4 USB";
/// Default route: monitor A (console monitor output: `cfg/mon/1` source, e.g. MAIN.1, replaced or
/// mixed by any active solo). Use `--hostio` with e.g. `main/1 -> ...` to bypass the monitor
/// section.
pub const DEFAULT_ROUTE: &str = "mon/1 -> \"Scarlett 2i4 USB\" 1/2";

/// Where the host-output config comes from.
#[derive(Debug, Clone, PartialEq)]
pub enum HostIoSetting {
    /// No host outputs.
    Off,
    /// [`DEFAULT_ROUTE`] if [`DEFAULT_DEVICE`] exists, else nothing.
    Default,
    /// A parsed `--hostio` file.
    Config(HostIoConfig),
}

impl HostIoSetting {
    /// Load `--hostio PATH`.
    pub fn from_file(path: &str) -> Result<Self, String> {
        let text = std::fs::read_to_string(path).map_err(|e| format!("--hostio {path}: {e}"))?;
        HostIoConfig::from_toml_str(&text)
            .map(HostIoSetting::Config)
            .map_err(|e| format!("--hostio {path}: {e}"))
    }

    /// The config to run with (probes CoreAudio for [`HostIoSetting::Default`]) and a note.
    pub fn resolve(&self) -> (Option<HostIoConfig>, String) {
        match self {
            HostIoSetting::Off => (None, "host outputs off".into()),
            HostIoSetting::Config(c) => (Some(c.clone()), describe(c)),
            HostIoSetting::Default => {
                if default_device_present() {
                    let cfg = HostIoConfig {
                        routes: vec![Route::parse(DEFAULT_ROUTE).expect("default route parses")],
                        ..HostIoConfig::default()
                    };
                    let d = describe(&cfg);
                    (Some(cfg), format!("{d} (default)"))
                } else {
                    (
                        None,
                        format!("host outputs: none ('{DEFAULT_DEVICE}' not found; use --hostio)"),
                    )
                }
            }
        }
    }
}

fn describe(c: &HostIoConfig) -> String {
    let routes: Vec<String> = c
        .routes
        .iter()
        .map(|r| {
            let src: Vec<String> = r.sources.iter().map(|s| s.to_string()).collect();
            format!(
                "{} -> {:?} {:?}",
                src.join(","),
                r.device,
                r.device_channels
            )
        })
        .collect();
    format!("host outputs: {}", routes.join("; "))
}

#[cfg(target_os = "macos")]
fn default_device_present() -> bool {
    matches!(
        wing_hostio::coreaudio::find_output_device(DEFAULT_DEVICE),
        Ok(Some(_))
    )
}

#[cfg(not(target_os = "macos"))]
fn default_device_present() -> bool {
    false
}

/// One planned bridge: device name, taps, and the bridge halves.
pub struct PlannedBridge {
    pub device: String,
    pub device_channels: Vec<u16>,
    pub taps: Vec<TapChannel>,
    pub producer: BridgeProducer,
    pub consumer: BridgeConsumer,
}

/// Create the bridges of `cfg` for a session at `rate` with producer bursts up to `burst`
/// frames (the driver's IO buffer: the engine runs that many frames back to back).
pub fn plan(cfg: &HostIoConfig, rate: u32, burst: usize) -> Result<Vec<PlannedBridge>, String> {
    let plans = cfg.devices().map_err(|e| e.to_string())?;
    Ok(plans
        .into_iter()
        .map(|p| {
            let (producer, consumer) = bridge(BridgeConfig {
                channels: p.channels.len(),
                source_rate: rate as f64,
                max_block_frames: burst,
                target_latency: p.latency,
                ..BridgeConfig::default()
            });
            PlannedBridge {
                device: p.device,
                device_channels: p.channels.iter().map(|c| c.1).collect(),
                taps: p.channels.iter().map(|c| c.0).collect(),
                producer,
                consumer,
            }
        })
        .collect())
}

/// A bridge producer and the tap lane of each of its channels (what the console feeds).
pub type Feed = (BridgeProducer, Vec<TapChannel>);

/// A running device output (the `HostOutput` is kept alive as an opaque handle).
pub type OutputEntry = (
    String,
    Arc<BridgeStats>,
    Option<Box<dyn std::any::Any + Send>>,
);

/// Running host outputs of a session (stopped on drop).
pub struct HostOutputs {
    pub outputs: Vec<OutputEntry>,
}

impl HostOutputs {
    /// No outputs.
    pub fn none() -> Self {
        Self {
            outputs: Vec::new(),
        }
    }

    /// Start the device side of every planned bridge; returns the producer feeds for the
    /// console. Missing devices are retried in the background by `HostOutput`.
    pub fn start(
        planned: Vec<PlannedBridge>,
        buffer_frames: Option<u32>,
    ) -> (Self, Vec<Feed>, Vec<String>) {
        let mut outputs = Vec::new();
        let mut feeds = Vec::new();
        let mut errors = Vec::new();
        for p in planned {
            let stats = p.producer.stats().clone();
            #[cfg(target_os = "macos")]
            {
                use wing_hostio::coreaudio::{HostOutput, HostOutputConfig};
                match HostOutput::start(
                    HostOutputConfig {
                        device: p.device.clone(),
                        device_channels: p.device_channels.clone(),
                        buffer_frames,
                    },
                    p.consumer,
                    Duration::ZERO,
                ) {
                    Ok(h) => outputs.push((
                        p.device.clone(),
                        stats,
                        Some(Box::new(h) as Box<dyn std::any::Any + Send>),
                    )),
                    Err(e) => {
                        errors.push(format!("host output {:?}: {e}", p.device));
                        continue;
                    }
                }
            }
            #[cfg(not(target_os = "macos"))]
            {
                let _ = (buffer_frames, p.consumer);
                errors.push(format!("host output {:?}: CoreAudio unavailable", p.device));
                outputs.push((p.device.clone(), stats, None));
            }
            feeds.push((p.producer, p.taps));
        }
        (Self { outputs }, feeds, errors)
    }

    /// `name: state fill corr underruns` per bridge.
    pub fn status(&self, rate: u32) -> String {
        let v: Vec<String> = self
            .outputs
            .iter()
            .map(|(name, st, _)| {
                let s = st.snapshot();
                format!(
                    "{name:?} {:?} fill {:.1}ms corr {:+.1}ppm underruns {} resyncs {}",
                    s.state,
                    s.fill_ms(rate as f64),
                    s.correction_ppm,
                    s.underruns,
                    s.resyncs
                )
            })
            .collect();
        if v.is_empty() {
            "bridges: none".into()
        } else {
            format!("bridges: {}", v.join("; "))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_route_and_plan() {
        let r = Route::parse(DEFAULT_ROUTE).unwrap();
        assert_eq!(r.device, DEFAULT_DEVICE);
        assert_eq!(r.sources[0].tap, wing_hostio::Tap::Monitor(1));
        assert_eq!(r.device_channels, vec![1, 2]);
        let cfg = HostIoConfig {
            routes: vec![r],
            ..HostIoConfig::default()
        };
        let p = plan(&cfg, 48_000, 512).unwrap();
        assert_eq!(p.len(), 1);
        assert_eq!(p[0].taps.len(), 2);
        assert_eq!(p[0].producer.channels(), 2);
        assert!(HostIoSetting::Off.resolve().0.is_none());
        let ex = HostIoSetting::Config(HostIoConfig::example());
        assert!(ex.resolve().1.contains("mon/1"));
    }
}
