//! Route mapping config: which simulator signal taps go to which host device channels.
//!
//! The source side of a route is a general **tap reference** into the console, like patching a
//! real WING output: the monitor section (headphones/monitor speakers, which will follow solo
//! once T038 lands), mains, busses, matrices, USB outs or local outs.
//!
//! ```text
//! mon/1         -> "Scarlett 2i4 USB" 1/2      monitor A, L/R (follows solo, T038)
//! main/1        -> "MacBook Pro Speakers" 1/2   main 1 L/R
//! bus/3.L       -> "Scarlett 2i4 USB" 3        one side of a stereo tap
//! usb_out/45-46 -> "Scarlett 2i4 USB" 1/2      USB out channels (lists: 1,3,5 / ranges: 1-8)
//! usb_out 45/46 -> BuiltInSpeakerDevice 1/2   legacy form for channel taps; unquoted = UID
//! ```
//!
//! Taps ([`Tap`]): `mon/1..2` (A, B), `main/1..4`, `bus/1..16`, `mtx/1..8` are stereo
//! (lanes L, R; append `.L`/`.R` for one side); `usb_out/<channels>` (1-48) and
//! `local_out/<channels>` (1-8) are flat channel taps. Device channels are 1-based, `/`- or
//! `,`-separated with `a-b` ranges, and must match the tap's lane count. Resolving a tap to an
//! engine port is the engine's job (T024/T038); this module only parses and validates.
//!
//! TOML (a `[hostio]` table in the daemon config, or a standalone document); see
//! [`EXAMPLE_TOML`] for the default config (monitor A -> Scarlett 1/2):
//!
//! ```toml
//! [hostio]
//! latency_ms = 5.0                 # default extra latency for every device
//!
//! [[hostio.route]]
//! map = 'mon/1 -> "Scarlett 2i4 USB" 1/2'
//!
//! [[hostio.route]]                 # structured form; per-route latency override
//! source = "usb_out/45-46"
//! device = "Scarlett 2i4 USB"
//! channels = "3/4"
//! latency_ms = 8.0
//! ```
//!
//! [`HostIoConfig::devices`] groups routes per device; each device becomes one bridge.

use std::fmt;
use std::time::Duration;

use serde::Deserialize;

/// Default host-output config (`crates/wing-hostio/hostio.example.toml`): monitor A (follows
/// solo) on the Scarlett's outputs 1/2.
pub const EXAMPLE_TOML: &str = include_str!("../hostio.example.toml");

/// A signal tap in the simulated console. Stable: the engine maps each variant to a port
/// (T024 for USB/local outs, mains, busses, matrices; T038 for the monitor section).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Tap {
    /// Monitor section output: 1 = monitor A, 2 = monitor B. Stereo; follows solo.
    Monitor(u8),
    /// Main bus 1-4. Stereo.
    Main(u8),
    /// Mix bus 1-16. Stereo.
    Bus(u8),
    /// Matrix 1-8. Stereo.
    Matrix(u8),
    /// USB outputs to the host (the virtual device's inputs); 48 lanes = channels.
    UsbOut,
    /// Local (analogue) outputs; 8 lanes = channels.
    LocalOut,
}

/// Lane names of stereo taps.
pub const LEFT: u16 = 1;
/// Right lane of a stereo tap.
pub const RIGHT: u16 = 2;

impl Tap {
    /// Number of lanes (2 for stereo taps, channel count for flat taps).
    pub fn width(self) -> u16 {
        match self {
            Tap::Monitor(_) | Tap::Main(_) | Tap::Bus(_) | Tap::Matrix(_) => 2,
            Tap::UsbOut => 48,
            Tap::LocalOut => 8,
        }
    }

    /// Whether lanes are L/R of a stereo signal (vs. numbered channels).
    pub fn is_stereo(self) -> bool {
        self.width() == 2 && !matches!(self, Tap::UsbOut | Tap::LocalOut)
    }

    fn kind_name(self) -> &'static str {
        match self {
            Tap::Monitor(_) => "mon",
            Tap::Main(_) => "main",
            Tap::Bus(_) => "bus",
            Tap::Matrix(_) => "mtx",
            Tap::UsbOut => "usb_out",
            Tap::LocalOut => "local_out",
        }
    }

    /// Indexed tap from a kind name, validating the index range.
    fn indexed(kind: &str, n: u16) -> Result<Self, ConfigError> {
        let (max, f): (u16, fn(u8) -> Tap) = match kind {
            "mon" | "monitor" => (2, Tap::Monitor),
            "main" | "mains" => (4, Tap::Main),
            "bus" => (16, Tap::Bus),
            "mtx" | "matrix" => (8, Tap::Matrix),
            _ => return err(format!("unknown tap '{kind}'")),
        };
        if n == 0 || n > max {
            return err(format!("{kind}/{n}: index must be 1..{max}"));
        }
        Ok(f(n as u8))
    }

    fn flat(kind: &str) -> Option<Self> {
        match kind {
            "usb_out" | "usb" | "usbout" => Some(Tap::UsbOut),
            "local_out" | "local" | "out" => Some(Tap::LocalOut),
            _ => None,
        }
    }
}

impl fmt::Display for Tap {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Tap::Monitor(n) | Tap::Main(n) | Tap::Bus(n) | Tap::Matrix(n) => {
                write!(f, "{}/{n}", self.kind_name())
            }
            _ => f.write_str(self.kind_name()),
        }
    }
}

/// One lane of a tap: what a single bridged channel carries.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct TapChannel {
    /// The tap.
    pub tap: Tap,
    /// 1-based lane: [`LEFT`]/[`RIGHT`] for stereo taps, the channel number for flat taps.
    pub lane: u16,
}

impl fmt::Display for TapChannel {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.tap.is_stereo() {
            write!(
                f,
                "{}.{}",
                self.tap,
                if self.lane == LEFT { "L" } else { "R" }
            )
        } else {
            write!(f, "{}/{}", self.tap, self.lane)
        }
    }
}

/// A parsed route: `sources[i]` plays on device channel `device_channels[i]`.
#[derive(Debug, Clone, PartialEq)]
pub struct Route {
    /// Tap lanes.
    pub sources: Vec<TapChannel>,
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

/// Parse `1/2`, `45-46`, `1,3,5` (1-based).
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

/// Parse a tap reference: `mon/1`, `main/2.L`, `usb_out/45-46`, or legacy `usb_out 45/46`.
pub fn parse_tap(s: &str) -> Result<Vec<TapChannel>, ConfigError> {
    let s = s.trim();
    // Legacy "kind list" form, only for flat channel taps.
    if let Some((kind, list)) = s.split_once(char::is_whitespace) {
        let kind = kind.to_ascii_lowercase();
        let tap = Tap::flat(&kind).ok_or_else(|| {
            ConfigError(format!(
                "tap '{s}': use 'kind/index' (e.g. 'mon/1', 'bus/3.L')"
            ))
        })?;
        return flat_lanes(tap, &parse_channel_list(list.trim())?);
    }
    let (kind, rest) = s
        .split_once('/')
        .ok_or_else(|| ConfigError(format!("tap '{s}': expected 'kind/index'")))?;
    let kind = kind.trim().to_ascii_lowercase();
    if let Some(tap) = Tap::flat(&kind) {
        let list: Vec<u16> = parse_channel_list(&rest.replace('/', ","))?;
        return flat_lanes(tap, &list);
    }
    let (idx, side) = match rest.split_once('.') {
        Some((i, side)) => (i, Some(side.trim().to_ascii_uppercase())),
        None => (rest, None),
    };
    let n: u16 = idx
        .trim()
        .parse()
        .map_err(|_| ConfigError(format!("tap '{s}': bad index '{idx}'")))?;
    let tap = Tap::indexed(&kind, n)?;
    let lanes: &[u16] = match side.as_deref() {
        None => &[LEFT, RIGHT],
        Some("L") => &[LEFT],
        Some("R") => &[RIGHT],
        Some(x) => return err(format!("tap '{s}': side must be L or R, got '{x}'")),
    };
    Ok(lanes.iter().map(|&lane| TapChannel { tap, lane }).collect())
}

fn flat_lanes(tap: Tap, chans: &[u16]) -> Result<Vec<TapChannel>, ConfigError> {
    for &c in chans {
        if c > tap.width() {
            return err(format!("{tap} has {} channels, got {c}", tap.width()));
        }
    }
    Ok(chans.iter().map(|&lane| TapChannel { tap, lane }).collect())
}

impl Route {
    /// Parse a route line `mon/1 -> "Scarlett 2i4 USB" 1/2`.
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
        Self::build(parse_tap(src)?, device, parse_channel_list(chans)?, None)
    }

    fn build(
        sources: Vec<TapChannel>,
        device: String,
        device_channels: Vec<u16>,
        latency: Option<Duration>,
    ) -> Result<Self, ConfigError> {
        if device.is_empty() {
            return err("empty device name");
        }
        if sources.len() != device_channels.len() {
            return err(format!(
                "route to '{device}': tap has {} lanes but {} device channels given",
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
    /// Bridged channels: `(tap lane, 1-based device channel)`; bridge channel `i` is entry `i`.
    pub channels: Vec<(TapChannel, u16)>,
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
    /// The default config ([`EXAMPLE_TOML`]: monitor A -> Scarlett 2i4 USB 1/2).
    pub fn example() -> Self {
        Self::from_toml_str(EXAMPLE_TOML).expect("example config parses")
    }

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
                    Route::build(parse_tap(&src)?, dev, parse_channel_list(&ch)?, None)?
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

    fn tc(tap: Tap, lane: u16) -> TapChannel {
        TapChannel { tap, lane }
    }

    #[test]
    fn parses_tap_references() {
        let r = Route::parse(r#"mon/1 -> "Scarlett 2i4 USB" 1/2"#).unwrap();
        assert_eq!(r.device, "Scarlett 2i4 USB");
        assert_eq!(r.device_channels, vec![1, 2]);
        assert_eq!(
            r.sources,
            vec![tc(Tap::Monitor(1), LEFT), tc(Tap::Monitor(1), RIGHT)]
        );
        assert_eq!(parse_tap("main/1").unwrap()[1], tc(Tap::Main(1), RIGHT));
        assert_eq!(parse_tap("bus/16.L").unwrap(), vec![tc(Tap::Bus(16), LEFT)]);
        assert_eq!(
            parse_tap("mtx/2.r").unwrap(),
            vec![tc(Tap::Matrix(2), RIGHT)]
        );
        assert_eq!(
            parse_tap("usb_out/45-46").unwrap(),
            vec![tc(Tap::UsbOut, 45), tc(Tap::UsbOut, 46)]
        );
        assert_eq!(
            parse_tap("local_out/1,3").unwrap(),
            vec![tc(Tap::LocalOut, 1), tc(Tap::LocalOut, 3)]
        );
        // Legacy channel form from the task description.
        let r = Route::parse(r#"usb_out 45/46 -> "Scarlett 2i4 USB" 1/2"#).unwrap();
        assert_eq!(r.sources, vec![tc(Tap::UsbOut, 45), tc(Tap::UsbOut, 46)]);
        let r = Route::parse("bus/1 -> SomeUID:1 3,4").unwrap();
        assert_eq!(r.device, "SomeUID:1");
        assert_eq!(r.device_channels, vec![3, 4]);
        assert_eq!(tc(Tap::Monitor(2), RIGHT).to_string(), "mon/2.R");
        assert_eq!(tc(Tap::UsbOut, 45).to_string(), "usb_out/45");
    }

    #[test]
    fn rejects_bad_routes() {
        for bad in [
            r#"usb_out/49 -> "X" 1"#,
            r#"mon/1 -> "X" 1"#,
            r#"mon/3 -> "X" 1/2"#,
            r#"main/0 -> "X" 1/2"#,
            r#"bus/1.C -> "X" 1"#,
            r#"mon 1 -> "X" 1/2"#,
            r#"foo/1 -> "X" 1/2"#,
            r#"usb_out/1 "X" 1"#,
            r#"usb_out/0 -> "X" 1"#,
            r#"usb_out/1 -> "X 1"#,
            r#"usb_out/2-1 -> "X" 1/2"#,
        ] {
            assert!(Route::parse(bad).is_err(), "{bad}");
        }
    }

    #[test]
    fn example_config_maps_monitor_a_to_scarlett() {
        let cfg = HostIoConfig::example();
        let plans = cfg.devices().unwrap();
        assert_eq!(plans.len(), 1);
        assert_eq!(plans[0].device, "Scarlett 2i4 USB");
        assert_eq!(
            plans[0].channels,
            vec![
                (tc(Tap::Monitor(1), LEFT), 1),
                (tc(Tap::Monitor(1), RIGHT), 2)
            ]
        );
        assert_eq!(plans[0].latency, Duration::from_millis(5));
    }

    #[test]
    fn toml_config_groups_devices() {
        let doc = r#"
            [hostio]
            latency_ms = 4.0

            [[hostio.route]]
            map = 'mon/1 -> "Scarlett 2i4 USB" 1/2'

            [[hostio.route]]
            source = "usb_out/45-46"
            device = "Scarlett 2i4 USB"
            channels = "3/4"
            latency_ms = 8.0

            [[hostio.route]]
            map = 'main/1 -> "MacBook Pro Speakers" 1/2'
        "#;
        let cfg = HostIoConfig::from_toml_str(doc).unwrap();
        assert_eq!(cfg.routes.len(), 3);
        let plans = cfg.devices().unwrap();
        assert_eq!(plans.len(), 2);
        assert_eq!(plans[0].device, "Scarlett 2i4 USB");
        assert_eq!(plans[0].channels.len(), 4);
        assert_eq!(plans[0].latency, Duration::from_millis(8));
        assert_eq!(plans[1].latency, Duration::from_millis(4));
        assert_eq!(plans[0].channels[2], (tc(Tap::UsbOut, 45), 3));
        assert_eq!(plans[1].channels[0].0.tap, Tap::Main(1));
    }

    #[test]
    fn toml_rejects_double_mapping_and_bad_shape() {
        let doc = r#"
            [[route]]
            map = 'mon/1 -> "A" 1/2'
            [[route]]
            map = 'usb_out/2 -> "A" 2'
        "#;
        assert!(HostIoConfig::from_toml_str(doc).is_err());
        assert!(HostIoConfig::from_toml_str("[[route]]\nsource = 'mon/1'\n").is_err());
        assert!(HostIoConfig::from_toml_str("latency_ms = -1\n").is_err());
        let ok = HostIoConfig::from_toml_str("[[route]]\nmap = 'bus/2 -> \"A\" 1/2'\n").unwrap();
        assert_eq!(ok.latency, Duration::from_millis(5));
    }
}
