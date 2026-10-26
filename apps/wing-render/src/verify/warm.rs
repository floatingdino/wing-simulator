//! WARM (SOUL WARMTH PRE) calibration against a real WING (T090).
//!
//! `wing-verify warm-gen --out <kit>` writes a short kit: `suite.wav` (~16 s: pilot, 1 kHz sines at -36 / -24 /
//! -18 / -12 / -6 dBFS RMS, 100 Hz at -18, SMPTE two-tone IMD (60 Hz + 7 kHz, 4:1) at -18, a vocal-like excerpt
//! at -24, end pilot) and one `.snap` per setting: channel 1 from USB 1 to MAIN (USB out 1/2), WARM in ch 1's
//! gate slot at MIX 100 %, trim 0, with a DRIVE sweep (10 / 33 / 66 / 100 / 125 %), HARMONICS -100 / +100 at
//! 125 % and a thru scene (WARM off) as the level reference. Quick tier: 8 scenes, ~20 s of capture each.
//!
//! `wing-verify fit-warm <recordings>` measures every recording (THD H2..H10, H2, H3 and level per 1 kHz
//! level, 100 Hz THD, IMD, vocal level and nonlinear residual), fits [`WarmLaw`] (a gain per measured drive
//! setting, the bias range `t_max` and the make-up reference level) and checks the fitted model, run as the
//! real `WarmthPre` over the same suite, against every point (pass: THD within +-2 dB). The fit itself uses a
//! static evaluation of the shaper on one sine period (exact at 1 kHz with COLOR 0: the tilts are then
//! identity and the oversampler is flat there).

use std::f64::consts::PI;
use std::path::Path;

use serde_json::json;
use wing_blocks::model::{Model, ParamValue};
use wing_blocks::models::dyn_special::{warm_makeup_at, WarmLaw, WarmthPre};
use wing_sim::wav::write_pcm;

use super::analyze::{locate, power_spectrum, rms_db};
use super::scenes::SceneDef;
use super::{read_recording, scene_snap};

pub const FS: u32 = 48_000;
/// Bump when the suite layout changes.
pub const VERSION: &str = "warm-1";
/// Extra recording time after the suite (s): the recorder starts 1.5 s early.
pub const CAPTURE_PAD_S: u32 = 4;
/// 1 kHz test levels (dBFS RMS).
pub const LEVELS: [f64; 5] = [-36.0, -24.0, -18.0, -12.0, -6.0];
/// THD / harmonic floor (dB re the fundamental): below it rec and model count as equal.
pub const FLOOR_DB: f64 = -80.0;
/// Acceptance: fitted model THD within this of the recording, per point.
pub const TOL_DB: f64 = 2.0;
/// Analysis length per tone (frames, power of two) and its offset into the tone.
const NFFT: usize = 32_768;
const SETTLE: usize = 14_400;

// ---- scenes ---------------------------------------------------------------------------------------------

/// One calibration setting of WARM in ch 1's gate slot.
#[derive(Clone, Copy, Debug)]
pub struct WarmScene {
    pub name: &'static str,
    /// WARM on (false: the thru reference).
    pub on: bool,
    pub drv: f32,
    pub hrm: f32,
    pub col: f32,
    pub quick: bool,
}

const fn sc(name: &'static str, drv: f32, hrm: f32, col: f32, quick: bool) -> WarmScene {
    WarmScene {
        name,
        on: true,
        drv,
        hrm,
        col,
        quick,
    }
}

pub fn catalog() -> Vec<WarmScene> {
    vec![
        WarmScene {
            name: "warm-thru",
            on: false,
            drv: 33.0,
            hrm: 0.0,
            col: 0.0,
            quick: true,
        },
        sc("warm-d010", 10.0, 0.0, 0.0, true),
        sc("warm-d033", 33.0, 0.0, 0.0, true),
        sc("warm-d066", 66.0, 0.0, 0.0, true),
        sc("warm-d100", 100.0, 0.0, 0.0, true),
        sc("warm-d125", 125.0, 0.0, 0.0, true),
        sc("warm-d125-hm100", 125.0, -100.0, 0.0, true),
        sc("warm-d125-hp100", 125.0, 100.0, 0.0, true),
        sc("warm-d066-hm100", 66.0, -100.0, 0.0, false),
        sc("warm-d066-hp100", 66.0, 100.0, 0.0, false),
        sc("warm-d100-cm1", 100.0, 0.0, -1.0, false),
        sc("warm-d100-cp1", 100.0, 0.0, 1.0, false),
    ]
}

impl WarmScene {
    pub fn describe(&self) -> String {
        if self.on {
            format!(
                "WARM drive {} %, harmonics {}, colour {}",
                self.drv, self.hrm, self.col
            )
        } else {
            "WARM off (level / chain reference)".into()
        }
    }

    /// As a kit scene (base routing plus the gate slot).
    pub fn def(&self) -> SceneDef {
        let mut sets: Vec<(String, String)> = Vec::new();
        if self.on {
            for (p, v) in [
                ("ch/1/gate/mdl", "WARM".to_string()),
                ("ch/1/gate/on", "true".into()),
                ("ch/1/gate/mix", "100".into()),
                ("ch/1/gate/1", format!("{}", self.drv)),
                ("ch/1/gate/2", format!("{}", self.hrm)),
                ("ch/1/gate/3", format!("{}", self.col)),
                ("ch/1/gate/4", "0".into()),
                ("ch/1/gate/5", "100".into()),
            ] {
                sets.push((p.into(), v));
            }
        }
        SceneDef {
            name: self.name.into(),
            area: "WARM",
            assumption: "WARM law (T090 calibration)",
            sets,
            kinds: vec![],
            quick: self.quick,
        }
    }
}

// ---- suite ----------------------------------------------------------------------------------------------

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Signal {
    /// Sine at `f` Hz, `level` dBFS RMS.
    Sine { f: f64, level: f64 },
    /// SMPTE IMD: 60 Hz + 7 kHz at 4:1 amplitude, total `level` dBFS RMS.
    Imd { level: f64 },
    /// Vocal-like programme at `level` dBFS RMS.
    Vocal { level: f64 },
}

impl Signal {
    pub fn label(&self) -> String {
        match *self {
            Signal::Sine { f, level } if f >= 1000.0 => format!("{:.0} kHz {level} dBFS", f / 1e3),
            Signal::Sine { f, level } => format!("{f:.0} Hz {level} dBFS"),
            Signal::Imd { level } => format!("IMD {level} dBFS"),
            Signal::Vocal { level } => format!("vocal {level} dBFS"),
        }
    }
}

#[derive(Clone, Debug)]
pub struct WarmSuite {
    pub samples: Vec<f32>,
    /// `(signal, start, end)` frames.
    pub items: Vec<(Signal, usize, usize)>,
    /// Start frame and length of both pilots.
    pub pilot: (usize, usize),
    pub end_pilot: usize,
}

impl WarmSuite {
    pub fn seconds(&self) -> f64 {
        self.samples.len() as f64 / FS as f64
    }
}

fn frames(sec: f64) -> usize {
    (sec * FS as f64).round() as usize
}

fn ramp(i: usize, n: usize, r: usize) -> f64 {
    let edge = i.min(n - 1 - i);
    if edge >= r {
        1.0
    } else {
        0.5 - 0.5 * (PI * edge as f64 / r as f64).cos()
    }
}

/// Vocal-like programme (as `tests/warm_live.rs`): harmonics of a 180 Hz source with 5.5 Hz vibrato,
/// -12 dB/oct above 500 Hz with formant bumps at 700 / 1200 / 2600 Hz, 4 Hz syllable envelope; `rms_db`
/// dBFS RMS.
pub fn vocal(n: usize, rms_db: f64) -> Vec<f64> {
    let fs = FS as f64;
    let mut ph = 0.0f64;
    let mut x: Vec<f64> = (0..n)
        .map(|i| {
            let t = i as f64 / fs;
            let f0 = 180.0 * (1.0 + 0.03 * (2.0 * PI * 5.5 * t).sin());
            ph += f0 / fs;
            let mut s = 0.0;
            for h in 1..=30 {
                let f = h as f64 * f0;
                let tilt = 1.0 / (1.0 + (f / 500.0).powi(2));
                let fm = |fc: f64, bw: f64| 1.0 / (1.0 + ((f - fc) / bw).powi(2));
                let a = tilt
                    * (0.3 + fm(700.0, 150.0) + 0.6 * fm(1200.0, 200.0) + 0.5 * fm(2600.0, 300.0));
                s += a * (2.0 * PI * h as f64 * ph).sin();
            }
            s * (0.5 + 0.5 * (2.0 * PI * 4.0 * t).sin().max(-0.6))
        })
        .collect();
    let rms = (x.iter().map(|v| v * v).sum::<f64>() / n as f64).sqrt();
    let g = 10f64.powf(rms_db / 20.0) / rms;
    x.iter_mut().for_each(|v| *v *= g);
    x
}

/// Build the suite (deterministic).
pub fn build() -> WarmSuite {
    let mut x: Vec<f32> = Vec::new();
    let mut items = Vec::new();
    let silence = |x: &mut Vec<f32>, s: f64| x.resize(x.len() + frames(s), 0.0);
    silence(&mut x, 0.5);
    // pilot: seeded noise at ~-33 dBFS RMS (quiet, so WARM at full drive barely changes it)
    let pilot = (x.len(), frames(0.2));
    let mut seed = 0x9e37_79b9u32;
    let mut noise = || {
        let mut u = || {
            seed ^= seed << 13;
            seed ^= seed >> 17;
            seed ^= seed << 5;
            seed as f64 / u32::MAX as f64 * 2.0 - 1.0
        };
        (u() + u() + u() + u()) / 2.0
    };
    for i in 0..pilot.1 {
        x.push((noise() * 0.04 * ramp(i, pilot.1, frames(0.005))) as f32);
    }
    silence(&mut x, 0.8);
    let lin = |db: f64| 10f64.powf(db / 20.0);
    let mut signals: Vec<Signal> = LEVELS
        .iter()
        .map(|&level| Signal::Sine { f: 1000.0, level })
        .collect();
    signals.push(Signal::Sine {
        f: 100.0,
        level: -18.0,
    });
    signals.push(Signal::Imd { level: -18.0 });
    signals.push(Signal::Vocal { level: -24.0 });
    for sig in signals {
        let (n, body): (usize, Vec<f64>) = match sig {
            Signal::Sine { f, level } => {
                let n = frames(1.1);
                let a = lin(level) * 2f64.sqrt();
                (
                    n,
                    (0..n)
                        .map(|i| a * (2.0 * PI * f * i as f64 / FS as f64).sin())
                        .collect(),
                )
            }
            Signal::Imd { level } => {
                let n = frames(1.1);
                let a2 = lin(level) * 2f64.sqrt() / 17f64.sqrt();
                (
                    n,
                    (0..n)
                        .map(|i| {
                            let t = i as f64 / FS as f64;
                            4.0 * a2 * (2.0 * PI * 60.0 * t).sin()
                                + a2 * (2.0 * PI * 7000.0 * t).sin()
                        })
                        .collect(),
                )
            }
            Signal::Vocal { level } => {
                let n = frames(3.0);
                (n, vocal(n, level))
            }
        };
        let a = x.len();
        for (i, v) in body.iter().enumerate() {
            x.push((v * ramp(i, n, frames(0.02))) as f32);
        }
        items.push((sig, a, x.len()));
        silence(&mut x, 0.2);
    }
    silence(&mut x, 0.3);
    let end_pilot = x.len();
    let first: Vec<f32> = x[pilot.0..pilot.0 + pilot.1].to_vec();
    x.extend_from_slice(&first);
    silence(&mut x, 1.0);
    WarmSuite {
        samples: x,
        items,
        pilot,
        end_pilot,
    }
}

// ---- gen ------------------------------------------------------------------------------------------------

/// Write the WARM kit into `out`: `suite.wav`, `scenes/*.snap`, `manifest.json`, `run-sheet.md`.
pub fn gen(out: &Path, quick_only: bool) -> Result<Vec<WarmScene>, String> {
    let io = |e: std::io::Error| e.to_string();
    std::fs::create_dir_all(out.join("scenes")).map_err(io)?;
    let suite = build();
    write_pcm(&out.join("suite.wav"), 1, FS, 24, &suite.samples).map_err(io)?;
    let scenes: Vec<WarmScene> = catalog()
        .into_iter()
        .filter(|s| s.quick || !quick_only)
        .collect();
    for s in &scenes {
        std::fs::write(
            out.join("scenes").join(format!("{}.snap", s.name)),
            scene_snap(&s.def())?,
        )
        .map_err(io)?;
    }
    let manifest = json!({
        "kit": "warm",
        "suite_version": VERSION,
        "sample_rate": FS,
        "suite_seconds": suite.seconds().ceil() as u32,
        "capture_pad_seconds": CAPTURE_PAD_S,
        "scenes": scenes.iter().map(|s| json!({
            "name": s.name, "on": s.on, "drive": s.drv, "harmonics": s.hrm, "colour": s.col, "quick": s.quick,
        })).collect::<Vec<_>>(),
        "signals": suite.items.iter().map(|(s, a, b)| json!({
            "signal": s.label(), "start_s": *a as f64 / FS as f64, "end_s": *b as f64 / FS as f64,
        })).collect::<Vec<_>>(),
    });
    std::fs::write(
        out.join("manifest.json"),
        serde_json::to_string_pretty(&manifest).map_err(|e| e.to_string())? + "\n",
    )
    .map_err(io)?;
    let mut sheet = String::from(
        "# WARM calibration run sheet\n\nFor each scene: recall it on the WING (all scopes, safes off), run\n\
         `tools/verify-hardware/capture.sh <kit> <scene> <recordings>`, tick it off. Then\n\
         `tools/verify-hardware/verify.sh fit-warm <recordings>`.\n\n",
    );
    for (i, s) in scenes.iter().enumerate() {
        sheet.push_str(&format!("{}. [ ] `{}` - {}\n", i + 1, s.name, s.describe()));
    }
    std::fs::write(out.join("run-sheet.md"), sheet).map_err(io)?;
    Ok(scenes)
}

// ---- measurement ------------------------------------------------------------------------------------------

/// One analysed signal of a recording (or of the model's output).
#[derive(Clone, Debug)]
pub struct Point {
    pub signal: Signal,
    /// THD H2..H10 (dB re the fundamental; IMD for the two-tone; residual for the vocal).
    pub thd_db: f64,
    /// H2 and H3 (dB re the fundamental; sine only).
    pub h2_db: f64,
    pub h3_db: f64,
    /// Output RMS (dBFS).
    pub out_db: f64,
}

#[derive(Clone, Debug)]
pub struct Measured {
    pub latency_ms: f64,
    pub align_score: f64,
    pub drift_frames: f64,
    pub points: Vec<Point>,
}

/// Power summed over +-3 bins around `f`.
fn band(p: &[f64], f: f64) -> f64 {
    let k = (f / (FS as f64 / NFFT as f64)).round() as usize;
    p[k.saturating_sub(3)..=(k + 3).min(p.len() - 1)]
        .iter()
        .sum()
}

fn db10(x: f64) -> f64 {
    10.0 * x.max(1e-30).log10()
}

/// Align `rec` (pilot) and analyse every signal of the suite.
pub fn measure(suite: &WarmSuite, rec: &[f32]) -> Result<Measured, String> {
    let (p0, pl) = suite.pilot;
    let needle = &suite.samples[p0..p0 + pl];
    let hay = &rec[..rec.len().min(FS as usize * 8)];
    let (pos, score) = locate(hay, needle).ok_or("recording too short")?;
    let lag0 = pos as i64 - p0 as i64;
    let (pe, m) = (suite.end_pilot as i64 + lag0, (FS / 10) as i64);
    let (a, b) = ((pe - m).max(0) as usize, (pe + pl as i64 + m) as usize);
    if b > rec.len() {
        return Err("recording ends before the end pilot: record the whole suite plus 2 s".into());
    }
    let (pos_e, _) = locate(&rec[a..b], needle).ok_or("recording too short")?;
    let drift = ((a + pos_e) as i64 - suite.end_pilot as i64 - lag0) as f64;
    let span = (suite.end_pilot - p0) as f64;
    let lag_at = |t: usize| (lag0 as f64 + drift * (t as f64 - p0 as f64) / span).round() as i64;
    let seg = |s: usize, n: usize| -> Option<&[f32]> {
        let r = s as i64 + lag_at(s);
        (r >= 0 && r as usize + n <= rec.len()).then(|| &rec[r as usize..r as usize + n])
    };
    let mut points = Vec::new();
    for &(signal, s, e) in &suite.items {
        let short = || format!("{}: recording too short", signal.label());
        let p = match signal {
            Signal::Sine { f, .. } => {
                let y = seg(s + SETTLE, NFFT).ok_or_else(short)?;
                let p = power_spectrum(y, NFFT);
                let h1 = band(&p, f);
                let hs: f64 = (2..=10).map(|h| band(&p, f * h as f64)).sum();
                Point {
                    signal,
                    thd_db: db10(hs / h1),
                    h2_db: db10(band(&p, 2.0 * f) / h1),
                    h3_db: db10(band(&p, 3.0 * f) / h1),
                    out_db: rms_db(y),
                }
            }
            Signal::Imd { .. } => {
                let y = seg(s + SETTLE, NFFT).ok_or_else(short)?;
                let p = power_spectrum(y, NFFT);
                let side: f64 = (1..=3)
                    .map(|k| {
                        band(&p, 7000.0 - 60.0 * k as f64) + band(&p, 7000.0 + 60.0 * k as f64)
                    })
                    .sum();
                Point {
                    signal,
                    thd_db: db10(side / band(&p, 7000.0)),
                    h2_db: f64::NAN,
                    h3_db: f64::NAN,
                    out_db: rms_db(y),
                }
            }
            Signal::Vocal { .. } => {
                // start 8 frames early so the output lags the input (the residual fit is causal)
                let (x0, n) = (s + FS as usize / 10, e - s - FS as usize / 5);
                let x = &suite.samples[x0..x0 + n];
                let r = x0 as i64 + lag_at(x0) - 8;
                if r < 0 || r as usize + n > rec.len() {
                    return Err(short());
                }
                let y = &rec[r as usize..r as usize + n];
                let res = wing_engine::probe::analyse(x, y, FS as f64)
                    .map(|a| a.residual_db)
                    .unwrap_or(f64::NAN);
                Point {
                    signal,
                    thd_db: res,
                    h2_db: f64::NAN,
                    h3_db: f64::NAN,
                    out_db: rms_db(y),
                }
            }
        };
        points.push(p);
    }
    Ok(Measured {
        latency_ms: lag0 as f64 / FS as f64 * 1e3,
        align_score: score,
        drift_frames: drift,
        points,
    })
}

// ---- model ------------------------------------------------------------------------------------------------

/// The suite through WARM with `law` at the scene's settings (the thru scene: unchanged), plus 1 s of tail.
pub fn model_render(law: &WarmLaw, s: &WarmScene, suite: &WarmSuite) -> Vec<f32> {
    let mut x = suite.samples.clone();
    x.extend(std::iter::repeat_n(0.0, FS as usize));
    if !s.on {
        return x;
    }
    let mut m = WarmthPre::with_law(*law);
    m.prepare(FS as f64, 64);
    for (slot, v) in [(1, s.drv), (2, s.hrm), (3, s.col), (4, 0.0), (5, 100.0)] {
        m.set_param(slot, ParamValue::Float(v));
    }
    m.commit();
    m.reset();
    let mut r = x.clone();
    for (l, r) in x.chunks_mut(64).zip(r.chunks_mut(64)) {
        m.process(l, r, None);
    }
    x
}

/// Static shaper on one period of a sine of RMS `level` dBFS: (THD dB, H2 dB, H3 dB, gain dB incl. make-up).
pub fn static_point(law: &WarmLaw, drv: f32, hrm: f32, level: f64) -> (f64, f64, f64, f64) {
    static_point_g(
        law.drive_gain(drv) as f64,
        law.bias(hrm),
        law.ref_peak(),
        level,
    )
}

const N_STATIC: usize = 128;

/// `(sin, cos)` of `2 pi k / N_STATIC`.
fn trig_table() -> &'static [(f64, f64); N_STATIC] {
    static T: std::sync::OnceLock<[(f64, f64); N_STATIC]> = std::sync::OnceLock::new();
    T.get_or_init(|| {
        let mut t = [(0.0, 0.0); N_STATIC];
        for (k, v) in t.iter_mut().enumerate() {
            let a = 2.0 * PI * k as f64 / N_STATIC as f64;
            *v = (a.sin(), a.cos());
        }
        t
    })
}

fn static_point_g(g: f64, t: f64, ref_peak: f64, level: f64) -> (f64, f64, f64, f64) {
    const N: usize = N_STATIC;
    let tab = trig_table();
    let b = t.atanh();
    let amp = 10f64.powf(level / 20.0) * 2f64.sqrt();
    let norm = 1.0 / ((1.0 - t * t) * g);
    let y: Vec<f64> = (0..N)
        .map(|k| ((g * amp * tab[k].0 + b).tanh() - t) * norm)
        .collect();
    let mean = y.iter().sum::<f64>() / N as f64;
    let h = |k: usize| {
        let (mut re, mut im) = (0.0, 0.0);
        for (i, v) in y.iter().enumerate() {
            let (sn, cs) = tab[(k * i) % N];
            re += v * cs;
            im += v * sn;
        }
        re * re + im * im
    };
    let h1 = h(1);
    let hs: f64 = (2..=10).map(h).sum();
    let rms = (y.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / N as f64).sqrt();
    let gain = 20.0 * (warm_makeup_at(g, b, ref_peak) * rms / (amp / 2f64.sqrt())).log10();
    (db10(hs / h1), db10(h(2) / h1), db10(h(3) / h1), gain)
}

// ---- fit --------------------------------------------------------------------------------------------------

/// A recording paired with its scene.
pub struct SceneData {
    pub scene: WarmScene,
    pub rec: Measured,
}

/// One checked point: recording vs model.
#[derive(Clone, Debug)]
pub struct Check {
    pub scene: String,
    pub signal: String,
    /// THD (dB; IMD / residual for those signals) of the recording, the fitted model and the previous law.
    pub rec_db: f64,
    pub model_db: f64,
    pub before_db: f64,
    /// Level change through WARM (dB): recording (vs the thru scene, or vs the input level), fitted model.
    pub rec_gain_db: f64,
    pub model_gain_db: f64,
    /// Counts for the +-2 dB acceptance (1 kHz and 100 Hz sines).
    pub graded: bool,
}

impl Check {
    pub fn delta(&self) -> f64 {
        self.model_db.max(FLOOR_DB) - self.rec_db.max(FLOOR_DB)
    }
    pub fn delta_before(&self) -> f64 {
        self.before_db.max(FLOOR_DB) - self.rec_db.max(FLOOR_DB)
    }
    pub fn ok(&self) -> bool {
        !self.graded || self.delta().abs() <= TOL_DB
    }
}

pub struct FitResult {
    pub law: WarmLaw,
    pub before: WarmLaw,
    pub checks: Vec<Check>,
    pub notes: Vec<String>,
    pub gain_ref: &'static str,
}

impl FitResult {
    pub fn pass(&self) -> bool {
        self.checks.iter().all(Check::ok)
    }
    pub fn worst(&self) -> Option<&Check> {
        self.checks
            .iter()
            .filter(|c| c.graded)
            .max_by(|a, b| a.delta().abs().total_cmp(&b.delta().abs()))
    }
}

/// Golden-section minimum of `f` on `[a, b]`.
fn golden(mut a: f64, mut b: f64, f: &dyn Fn(f64) -> f64) -> f64 {
    let r = (5f64.sqrt() - 1.0) / 2.0;
    let (mut c, mut d) = (b - r * (b - a), a + r * (b - a));
    let (mut fc, mut fd) = (f(c), f(d));
    for _ in 0..30 {
        if fc < fd {
            b = d;
            d = c;
            fd = fc;
            c = b - r * (b - a);
            fc = f(c);
        } else {
            a = c;
            c = d;
            fc = fd;
            d = a + r * (b - a);
            fd = f(d);
        }
    }
    (a + b) / 2.0
}

/// Grid search then golden refinement around the best grid point.
fn minimise(lo: f64, hi: f64, steps: usize, f: &dyn Fn(f64) -> f64) -> f64 {
    let dx = (hi - lo) / steps as f64;
    let best = (0..=steps)
        .map(|i| {
            let x = lo + dx * i as f64;
            (x, f(x))
        })
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map_or(lo, |b| b.0);
    golden((best - dx).max(lo), (best + dx).min(hi), f)
}

/// A 1 kHz point: (level, THD, H2, H3, out dB).
type SinePt = (f64, f64, f64, f64, f64);

/// 1 kHz points of a measurement.
fn sine_points(m: &Measured) -> Vec<SinePt> {
    m.points
        .iter()
        .filter_map(|p| match p.signal {
            Signal::Sine { f: 1000.0, level } => {
                Some((level, p.thd_db, p.h2_db, p.h3_db, p.out_db))
            }
            _ => None,
        })
        .collect()
}

/// Shape error of one scene's 1 kHz points for shaper gain `g` and bias `t`.
fn shape_err(pts: &[SinePt], g: f64, t: f64) -> f64 {
    let c = |x: f64| x.max(FLOOR_DB);
    pts.iter()
        .map(|&(level, thd, h2, h3, _)| {
            let (mt, m2, m3, _) = static_point_g(g, t, 1.0, level);
            (c(mt) - c(thd)).powi(2) + 0.25 * ((c(m2) - c(h2)).powi(2) + (c(m3) - c(h3)).powi(2))
        })
        .sum()
}

/// Fit the WARM law to measured scenes and check the result with the real model.
pub fn fit(data: &[SceneData], before: &WarmLaw) -> Result<FitResult, String> {
    let mut notes = Vec::new();
    let thru = data.iter().find(|d| !d.scene.on);
    let gain_ref = if thru.is_some() {
        "vs the warm-thru recording"
    } else {
        notes.push("no warm-thru recording: levels are referenced to the input level (assumes a unity chain)".into());
        "vs the input level"
    };
    // recorded level change through WARM per point
    let rec_gain = |d: &SceneData, i: usize| -> f64 {
        let p = &d.rec.points[i];
        match thru {
            Some(t) => p.out_db - t.rec.points[i].out_db,
            None => {
                p.out_db
                    - match p.signal {
                        Signal::Sine { level, .. }
                        | Signal::Imd { level }
                        | Signal::Vocal { level } => level,
                    }
            }
        }
    };
    let fit_set: Vec<&SceneData> = data
        .iter()
        .filter(|d| d.scene.on && d.scene.col == 0.0)
        .collect();
    let mut drives: Vec<f32> = fit_set.iter().map(|d| d.scene.drv).collect();
    drives.sort_by(f32::total_cmp);
    drives.dedup();
    let pts_of = |drv: f32| -> Vec<(f32, Vec<SinePt>)> {
        fit_set
            .iter()
            .filter(|d| d.scene.drv == drv)
            .map(|d| (d.scene.hrm, sine_points(&d.rec)))
            .collect()
    };
    // drive settings whose distortion rises above the floor are identifiable
    let usable: Vec<f32> = drives
        .iter()
        .copied()
        .filter(|&d| {
            pts_of(d)
                .iter()
                .any(|(_, p)| p.iter().any(|q| q.1 > FLOOR_DB + 6.0))
        })
        .collect();
    for d in drives.iter().filter(|d| !usable.contains(d)) {
        notes.push(format!(
            "drive {d} %: THD below {FLOOR_DB} dB at every level, not used for the drive curve"
        ));
    }
    if usable.is_empty() {
        return Err(
            "no recording with measurable WARM distortion (wrong scene, or WARM not in the path?)"
                .into(),
        );
    }
    let (lg_lo, lg_hi) = (0.005f64.ln(), 400f64.ln());
    let best_g = |drv: f32, t_max: f64| -> (f64, f64) {
        let sets = pts_of(drv);
        let err = |lg: f64| -> f64 {
            sets.iter()
                .map(|(hrm, p)| shape_err(p, lg.exp(), t_max * (1.0 - *hrm as f64 / 100.0) * 0.5))
                .sum()
        };
        let lg = minimise(lg_lo, lg_hi, 48, &err);
        (lg.exp(), err(lg))
    };
    let total = |t_max: f64| -> f64 { usable.iter().map(|&d| best_g(d, t_max).1).sum() };
    let t_max = minimise(0.0, 0.8, 32, &total);
    let gains: Vec<(f32, f32)> = usable
        .iter()
        .map(|&d| (d, best_g(d, t_max).0 as f32))
        .collect();
    for w in gains.windows(2) {
        if w[1].1 <= w[0].1 {
            notes.push(format!(
                "drive gain not increasing between {} % and {} % ({:.3} -> {:.3})",
                w[0].0, w[1].0, w[0].1, w[1].1
            ));
        }
    }
    // make-up reference: level changes of the fitted shaper vs the recordings (1 kHz points)
    let mut lvl_pts = Vec::new();
    for d in &fit_set {
        let g = gains
            .iter()
            .find(|x| x.0 == d.scene.drv)
            .map(|x| x.1 as f64);
        for (i, p) in d.rec.points.iter().enumerate() {
            if let (Some(g), Signal::Sine { f, level }) = (g, p.signal) {
                if f == 1000.0 {
                    let t = t_max * (1.0 - d.scene.hrm as f64 / 100.0) * 0.5;
                    lvl_pts.push((g, t, level, rec_gain(d, i)));
                }
            }
        }
    }
    let lvl_err = |r: f64| -> f64 {
        let rp = 10f64.powf(r / 20.0) * 2f64.sqrt();
        lvl_pts
            .iter()
            .map(|&(g, t, level, rg)| (static_point_g(g, t, rp, level).3 - rg).powi(2))
            .sum()
    };
    let ref_db = minimise(-48.0, 0.0, 96, &lvl_err);
    let law = WarmLaw::from_points(
        &round_points(&gains),
        round6(t_max) as f32,
        round6(ref_db) as f32,
    )?;

    // check with the real model
    let suite = build();
    let mut checks = Vec::new();
    for d in data.iter().filter(|d| d.scene.on) {
        let (fm, bm) = (
            measure(&suite, &model_render(&law, &d.scene, &suite))?,
            measure(&suite, &model_render(before, &d.scene, &suite))?,
        );
        for (i, p) in d.rec.points.iter().enumerate() {
            let graded = matches!(p.signal, Signal::Sine { .. });
            let level = match p.signal {
                Signal::Sine { level, .. } | Signal::Imd { level } | Signal::Vocal { level } => {
                    level
                }
            };
            checks.push(Check {
                scene: d.scene.name.into(),
                signal: p.signal.label(),
                rec_db: p.thd_db,
                model_db: fm.points[i].thd_db,
                before_db: bm.points[i].thd_db,
                rec_gain_db: rec_gain(d, i),
                model_gain_db: fm.points[i].out_db - level,
                graded,
            });
        }
    }
    Ok(FitResult {
        law,
        before: *before,
        checks,
        notes,
        gain_ref,
    })
}

fn round6(x: f64) -> f64 {
    (x * 1e6).round() / 1e6
}

fn round_points(p: &[(f32, f32)]) -> Vec<(f32, f32)> {
    p.iter()
        .map(|&(d, g)| (d, round6(g as f64) as f32))
        .collect()
}

/// Read `<scene>.wav` for every catalog scene found in `rec_dir`, measure it.
pub fn load_recordings(rec_dir: &Path) -> Result<Vec<SceneData>, String> {
    let suite = build();
    let mut out = Vec::new();
    for scene in catalog() {
        let wav = rec_dir.join(format!("{}.wav", scene.name));
        if !wav.exists() {
            continue;
        }
        let rec = read_recording(&wav)?;
        let m = measure(&suite, &rec).map_err(|e| format!("{}: {e}", scene.name))?;
        out.push(SceneData { scene, rec: m });
    }
    if out.is_empty() {
        return Err(format!(
            "no WARM recordings (warm-*.wav) in {}",
            rec_dir.display()
        ));
    }
    Ok(out)
}

fn pct(db: f64) -> f64 {
    100.0 * 10f64.powf(db / 20.0)
}

/// The calibration file for a fit.
pub fn law_json(r: &FitResult, source: &str) -> String {
    let worst = r.worst().map(|c| c.delta()).unwrap_or(0.0);
    let v = json!({
        "model": "WARM",
        "version": 1,
        "source": source,
        "drive_points": r.law.points().iter().map(|p| json!([p.0 as f64, round6(p.1 as f64)])).collect::<Vec<_>>(),
        "t_max": round6(r.law.t_max as f64),
        "makeup_ref_dbfs_rms": round6(r.law.makeup_ref_dbfs_rms as f64),
        "fit": {
            "pass": r.pass(),
            "tolerance_db": TOL_DB,
            "worst_thd_delta_db": (worst * 100.0).round() / 100.0,
            "notes": r.notes,
            "points": r.checks.iter().map(|c| json!({
                "scene": c.scene, "signal": c.signal,
                "rec_db": (c.rec_db * 100.0).round() / 100.0,
                "model_db": (c.model_db * 100.0).round() / 100.0,
                "delta_db": (c.delta() * 100.0).round() / 100.0,
            })).collect::<Vec<_>>(),
        },
    });
    serde_json::to_string_pretty(&v).unwrap_or_default() + "\n"
}

/// Markdown report of a fit.
pub fn report_md(r: &FitResult) -> String {
    let mut s = String::from("# WARM calibration (fit-warm)\n\n");
    s.push_str(&format!(
        "Result: **{}** (THD of every 1 kHz / 100 Hz point within +-{TOL_DB} dB; worst {})\n\n",
        if r.pass() { "PASS" } else { "FAIL" },
        r.worst()
            .map(|c| format!("{:+.2} dB, {} {}", c.delta(), c.scene, c.signal))
            .unwrap_or_else(|| "-".into())
    ));
    s.push_str("Fitted law:\n\n| | before | fitted |\n|---|---|---|\n");
    let pts = |l: &WarmLaw| {
        l.points()
            .iter()
            .map(|p| format!("{} %: {:.4}", p.0, p.1))
            .collect::<Vec<_>>()
            .join(", ")
    };
    s.push_str(&format!(
        "| drive gain | {} | {} |\n| t_max (bias at hrm -100) | {:.4} | {:.4} |\n| make-up reference (dBFS RMS) | {:.2} | {:.2} |\n\n",
        pts(&r.before),
        pts(&r.law),
        r.before.t_max,
        r.law.t_max,
        r.before.makeup_ref_dbfs_rms,
        r.law.makeup_ref_dbfs_rms
    ));
    for n in &r.notes {
        s.push_str(&format!("- note: {n}\n"));
    }
    s.push_str(&format!(
        "\nTHD = H2..H10 (IMD: SMPTE sidebands; vocal: nonlinear residual). Level change {}.\n\n\
         | Scene | Signal | Rec | Fitted | Delta dB | Before | Delta dB | Level rec / fitted (dB) | |\n\
         |---|---|---|---|---|---|---|---|---|\n",
        r.gain_ref
    ));
    for c in &r.checks {
        s.push_str(&format!(
            "| `{}` | {} | {:.3} % | {:.3} % | {:+.2} | {:.3} % | {:+.2} | {:+.2} / {:+.2} | {} |\n",
            c.scene,
            c.signal,
            pct(c.rec_db),
            pct(c.model_db),
            c.delta(),
            pct(c.before_db),
            c.delta_before(),
            c.rec_gain_db,
            c.model_gain_db,
            if !c.graded {
                "info"
            } else if c.ok() {
                "OK"
            } else {
                "OFF"
            }
        ));
    }
    s
}

/// `fit-warm`: measure, fit, write the report into `out` and (optionally) the calibration file.
pub fn fit_warm(rec_dir: &Path, out: &Path, write: Option<&Path>) -> Result<FitResult, String> {
    let data = load_recordings(rec_dir)?;
    for d in &data {
        if d.rec.align_score < 0.5 {
            return Err(format!(
                "{}: weak pilot match ({:.2}): wrong file or clipped recording",
                d.scene.name, d.rec.align_score
            ));
        }
    }
    let mut r = fit(&data, WarmLaw::builtin())?;
    for d in &data {
        if d.rec.drift_frames.abs() > 2.0 {
            r.notes.push(format!(
                "{}: clock drift {:+.0} frames (play and record on the same device)",
                d.scene.name, d.rec.drift_frames
            ));
        }
    }
    std::fs::create_dir_all(out).map_err(|e| e.to_string())?;
    std::fs::write(out.join("warm-fit.md"), report_md(&r)).map_err(|e| e.to_string())?;
    if let Some(p) = write {
        let src = format!(
            "fit-warm on {} recordings from {} (unix time {})",
            data.len(),
            rec_dir.display(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map(|d| d.as_secs())
                .unwrap_or(0)
        );
        std::fs::write(p, law_json(&r, &src)).map_err(|e| format!("{}: {e}", p.display()))?;
    }
    Ok(r)
}

/// Parse a written calibration file back (round trip check).
pub fn read_law(path: &Path) -> Result<WarmLaw, String> {
    WarmLaw::from_json(&std::fs::read_to_string(path).map_err(|e| e.to_string())?)
}
