//! Route mapping config: which simulator outputs go to which host device channels.
//!
//! One route line maps a list of simulator channels to the same number of device channels:
//!
//! ```text
//! usb_out 45/46 -> "Scarlett 2i4 USB" 1/2
//! main 1/2      -> "MacBook Pro Speakers" 1/2
//! bus 1-4       -> "BuiltInSpeakerDevice" 1-4      (device by UID, unquoted: no spaces)
//! ```
//!
//! Channel lists are 1-based, `/`- or `,`-separated, with `a-b` ranges. Sources: `usb_out`
//! (1-48), `main` (1-8: mains 1-4 as L/R pairs), `bus` (1-16), `mtx` (1-8), `mon` (1-4:
//! monitor A/B L/R), `local_out` (1-8). Resolving a source to an engine port is the engine's
//! job (T024); this module only parses and validates.
//!
//! TOML (a `[hostio]` table in the daemon config, or a standalone document):
//!
//! ```toml
//! [hostio]
//! latency_ms = 5.0                 # default extra latency for every device
//!
//! [[hostio.route]]
//! map = 'usb_out 45/46 -> "Scarlett 2i4 USB" 1/2'
//!
//! [[hostio.route]]                 # structured form; per-route latency override
//! source = "mon 1/2"
//! device = "Scarlett 2i4 USB"
//! channels = "3/4"
//! latency_ms = 8.0
//! ```
//!
//! [`HostIoConfig::devices`] groups routes per device; each device becomes one bridge.

use std::fmt;
use std::time::Duration;

use serde::Deserialize;

/// Simulator output families that can feed a host device.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum SourceKind {
    /// USB outputs to the host (the virtual device's inputs), 48 channels.
    UsbOut,
    /// Main busses 1-4, as 8 channels (L/R pairs).
    Main,
    /// Mix busses 1-16.
    Bus,
    /// Matrices 1-8.
    Matrix,
    /// Monitor A/B, L/R.
    Monitor,
    /// Local (analogue) outputs 1-8.
    LocalOut,
}

impl SourceKind {
    /// Number of channels in this family.
    pub fn channel_count(self) -> u16 {
        match self {
            Self::UsbOut => 48,
            Self::Main => 8,
            Self::Bus => 16,
            Self::Matrix => 8,
            Self::Monitor => 4,
            Self::LocalOut => 8,
        }
    }

    /// Canonical config name.
    pub fn name(self) -> &'static str {
        match self {
            Self::UsbOut => "usb_out",
            Self::Main => "main",
            Self::Bus => "bus",
            Self::Matrix => "mtx",
            Self::Monitor => "mon",
            Self::LocalOut => "local_out",
        }
    }

    fn parse(s: &str) -> Option<Self> {
        Some(match s.to_ascii_lowercase().as_str() {
            "usb_out" | "usb" | "usbout" => Self::UsbOut,
            "main" | "mains" => Self::Main,
            "bus" => Self::Bus,
            "mtx" | "matrix" => Self::Matrix,
            "mon" | "monitor" => Self::Monitor,
            "local_out" | "local" | "out" => Self::LocalOut,
            _ => return None,
        })
    }
}

/// One simulator output channel (1-based within its family).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct SourceChannel {
    /// Family.
    pub kind: SourceKind,
    /// 1-based channel.
    pub channel: u16,
}

impl fmt::Display for SourceChannel {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{} {}", self.kind.name(), self.channel)
    }
}

/// A parsed route: `sources[i]` plays on device channel `device_channels[i]`.
#[derive(Debug, Clone, PartialEq)]
pub struct Route {
    /// Simulator channels.
    pub sources: Vec<SourceChannel>,
    /// Device name or UID.
    pub device: String,
    /// 1-based device output channels.
    pub device_channels: Vec<u16>,
    /// Per-route extra latency override.
    pub latency: Option<Duration>,
}

/// Config / parse error.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ConfigError(pub String);

impl fmt::Display for ConfigError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for ConfigError {}

fn err<T>(msg: impl Into<String>) -> Result<T, ConfigError> {
    Err(ConfigError(msg.into()))
}

/// Parse `45/46`, `1-4`, `1,3,5`.
pub fn parse_channel_list(s: &str) -> Result<Vec<u16>, ConfigError> {
    let mut out = Vec::new();
    for part in s.split(['/', ',']) {
        let part = part.trim();
        if part.is_empty() {
            return err(format!("empty channel in '{s}'"));
        }
        let num = |t: &str| -> Result<u16, ConfigError> {
            match t.trim().parse::<u16>() {
                Ok(0) | Err(_) => err(format!("bad channel '{t}' in '{s}' (1-based numbers)")),
                Ok(n) => Ok(n),
            }
        };
        if let Some((a, b)) = part.split_once('-') {
            let (a, b) = (num(a)?, num(b)?);
            if a > b {
                return err(format!("descending range '{part}'"));
            }
            out.extend(a..=b);
        } else {
            out.push(num(part)?);
        }
    }
    Ok(out)
}

/// Parse `usb_out 45/46`.
pub fn parse_source(s: &str) -> Result<Vec<SourceChannel>, ConfigError> {
    let s = s.trim();
    let (kind, chans) = s
        .split_once(char::is_whitespace)
        .ok_or_else(|| ConfigError(format!("source '{s}': expected '<kind> <channels>'")))?;
    let kind = SourceKind::parse(kind)
        .ok_or_else(|| ConfigError(format!("unknown source kind '{kind}'")))?;
    let chans = parse_channel_list(chans)?;
    for &c in &chans {
        if c > kind.channel_count() {
            return err(format!(
                "{} has {} channels, got {c}",
                kind.name(),
                kind.channel_count()
            ));
        }
    }
    Ok(chans
        .into_iter()
        .map(|channel| SourceChannel { kind, channel })
        .collect())
}

impl Route {
    /// Parse a route line `usb_out 45/46 -> "Scarlett 2i4 USB" 1/2`.
    pub fn parse(line: &str) -> Result<Self, ConfigError> {
        let (src, dst) = line
            .split_once("->")
            .ok_or_else(|| ConfigError(format!("route '{line}': missing '->'")))?;
        let dst = dst.trim();
        let (device, chans) = if let Some(rest) = dst.strip_prefix('"') {
            let end = rest
                .find('"')
                .ok_or_else(|| ConfigError(format!("route '{line}': unterminated quote")))?;
            (rest[..end].to_string(), rest[end + 1..].trim())
        } else {
            match dst.rsplit_once(char::is_whitespace) {
                Some((d, c)) => (d.trim().to_string(), c.trim()),
                None => return err(format!("route '{line}': expected '<device> <channels>'")),
            }
        };
        Self::build(parse_source(src)?, device, parse_channel_list(chans)?, None)
    }

    fn build(
        sources: Vec<SourceChannel>,
        device: String,
        device_channels: Vec<u16>,
        latency: Option<Duration>,
    ) -> Result<Self, ConfigError> {
        if device.is_empty() {
            return err("empty device name");
        }
        if sources.len() != device_channels.len() {
            return err(format!(
                "route to '{device}': {} source channels but {} device channels",
                sources.len(),
                device_channels.len()
            ));
        }
        Ok(Self {
            sources,
            device,
            device_channels,
            latency,
        })
    }
}

/// Raw TOML shape of one route.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct RawRoute {
    map: Option<String>,
    source: Option<String>,
    device: Option<String>,
    channels: Option<String>,
    latency_ms: Option<f64>,
}

#[derive(Debug, Deserialize, Default)]
#[serde(deny_unknown_fields)]
struct RawHostIo {
    latency_ms: Option<f64>,
    buffer_frames: Option<u32>,
    #[serde(default)]
    route: Vec<RawRoute>,
}

#[derive(Debug, Deserialize)]
struct RawDoc {
    hostio: Option<RawHostIo>,
}

/// Host output configuration.
#[derive(Debug, Clone, PartialEq)]
pub struct HostIoConfig {
    /// Default extra latency.
    pub latency: Duration,
    /// Optional device IO buffer size request (frames).
    pub buffer_frames: Option<u32>,
    /// Routes in file order.
    pub routes: Vec<Route>,
}

impl Default for HostIoConfig {
    fn default() -> Self {
        Self {
            latency: Duration::from_millis(5),
            buffer_frames: None,
            routes: Vec::new(),
        }
    }
}

/// All routes targeting one device, merged.
#[derive(Debug, Clone, PartialEq)]
pub struct DevicePlan {
    /// Device name or UID.
    pub device: String,
    /// Bridged channels: `(source, 1-based device channel)`.
    pub channels: Vec<(SourceChannel, u16)>,
    /// Extra latency (max of the routes' values).
    pub latency: Duration,
}

fn ms(v: f64) -> Result<Duration, ConfigError> {
    if !(0.0..=1000.0).contains(&v) {
        return err(format!("latency_ms {v} out of range 0..1000"));
    }
    Ok(Duration::from_secs_f64(v / 1e3))
}

impl HostIoConfig {
    /// Parse a TOML document containing a `[hostio]` table, or the table's keys at top level.
    pub fn from_toml_str(s: &str) -> Result<Self, ConfigError> {
        let raw = match toml::from_str::<RawDoc>(s) {
            Ok(RawDoc { hostio: Some(h) }) => h,
            _ => toml::from_str::<RawHostIo>(s).map_err(|e| ConfigError(e.to_string()))?,
        };
        let mut cfg = HostIoConfig::default();
        if let Some(l) = raw.latency_ms {
            cfg.latency = ms(l)?;
        }
        cfg.buffer_frames = raw.buffer_frames;
        for r in raw.route {
            let latency = r.latency_ms.map(ms).transpose()?;
            let mut route = match (r.map, r.source, r.device, r.channels) {
                (Some(m), None, None, None) => Route::parse(&m)?,
                (None, Some(src), Some(dev), Some(ch)) => {
                    Route::build(parse_source(&src)?, dev, parse_channel_list(&ch)?, None)?
                }
                _ => {
                    return err("route needs either 'map' or all of 'source', 'device', 'channels'")
                }
            };
            route.latency = latency;
            cfg.routes.push(route);
        }
        cfg.devices()?;
        Ok(cfg)
    }

    /// Group routes by device (first-seen order), rejecting a device channel used twice.
    pub fn devices(&self) -> Result<Vec<DevicePlan>, ConfigError> {
        let mut plans: Vec<DevicePlan> = Vec::new();
        for r in &self.routes {
            let lat = r.latency.unwrap_or(self.latency);
            let idx = match plans.iter().position(|p| p.device == r.device) {
                Some(i) => i,
                None => {
                    plans.push(DevicePlan {
                        device: r.device.clone(),
                        channels: Vec::new(),
                        latency: lat,
                    });
                    plans.len() - 1
                }
            };
            let plan = &mut plans[idx];
            plan.latency = plan.latency.max(lat);
            for (&s, &d) in r.sources.iter().zip(&r.device_channels) {
                if plan.channels.iter().any(|&(_, x)| x == d) {
                    return err(format!("device '{}' channel {d} is mapped twice", r.device));
                }
                plan.channels.push((s, d));
            }
        }
        Ok(plans)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_route_line() {
        let r = Route::parse(r#"usb_out 45/46 -> "Scarlett 2i4 USB" 1/2"#).unwrap();
        assert_eq!(r.device, "Scarlett 2i4 USB");
        assert_eq!(r.device_channels, vec![1, 2]);
        assert_eq!(
            r.sources,
            vec![
                SourceChannel {
                    kind: SourceKind::UsbOut,
                    channel: 45
                },
                SourceChannel {
                    kind: SourceKind::UsbOut,
                    channel: 46
                }
            ]
        );
        let r = Route::parse("bus 1-4 -> SomeUID:1 3,4,5,6").unwrap();
        assert_eq!(r.device, "SomeUID:1");
        assert_eq!(r.device_channels, vec![3, 4, 5, 6]);
    }

    #[test]
    fn rejects_bad_routes() {
        for bad in [
            r#"usb_out 49 -> "X" 1"#,
            r#"usb_out 1/2 -> "X" 1"#,
            r#"foo 1 -> "X" 1"#,
            r#"usb_out 1 "X" 1"#,
            r#"usb_out 0 -> "X" 1"#,
            r#"usb_out 1 -> "X 1"#,
            r#"usb_out 2-1 -> "X" 1/2"#,
        ] {
            assert!(Route::parse(bad).is_err(), "{bad}");
        }
    }

    #[test]
    fn toml_config_groups_devices() {
        let doc = r#"
            [hostio]
            latency_ms = 4.0

            [[hostio.route]]
            map = 'usb_out 45/46 -> "Scarlett 2i4 USB" 1/2'

            [[hostio.route]]
            source = "mon 1/2"
            device = "Scarlett 2i4 USB"
            channels = "3/4"
            latency_ms = 8.0

            [[hostio.route]]
            map = 'main 1/2 -> "MacBook Pro Speakers" 1/2'
        "#;
        let cfg = HostIoConfig::from_toml_str(doc).unwrap();
        assert_eq!(cfg.routes.len(), 3);
        let plans = cfg.devices().unwrap();
        assert_eq!(plans.len(), 2);
        assert_eq!(plans[0].device, "Scarlett 2i4 USB");
        assert_eq!(plans[0].channels.len(), 4);
        assert_eq!(plans[0].latency, Duration::from_millis(8));
        assert_eq!(plans[1].latency, Duration::from_millis(4));
        assert_eq!(plans[0].channels[2].0.kind, SourceKind::Monitor);
    }

    #[test]
    fn toml_rejects_double_mapping_and_bad_shape() {
        let doc = r#"
            [[route]]
            map = 'usb_out 1 -> "A" 1'
            [[route]]
            map = 'usb_out 2 -> "A" 1'
        "#;
        assert!(HostIoConfig::from_toml_str(doc).is_err());
        assert!(HostIoConfig::from_toml_str("[[route]]\nsource = 'usb 1'\n").is_err());
        assert!(HostIoConfig::from_toml_str("latency_ms = -1\n").is_err());
        let ok = HostIoConfig::from_toml_str("[[route]]\nmap = 'usb 1/2 -> \"A\" 1/2'\n").unwrap();
        assert_eq!(ok.latency, Duration::from_millis(5));
    }
}
