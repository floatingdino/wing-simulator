//! Fidelity harness (T045): measure a [`Model`] the way the reference data describes the console and compare.
//!
//! Measurements (all through the public [`Model`] interface, 64-sample blocks like the engine):
//! - [`magnitude_response`]: impulse response -> DTFT at arbitrary frequencies (dB).
//! - [`static_curve`]: output level vs DC input level after settling (dynamics static law, dB in / dB out).
//! - [`time_constants`]: 10 %-90 % gain-change times after a level step up (attack) and down (release).
//!
//! References: `data/reference-curves/*.json` (T00A, `docs/research/reference-curves.md`, format
//! `wing-reference-curves/1`). [`FreqCurves::load`] reads the frequency-response files (`filter.json`,
//! `eq-*.json`, `dyn-sidechain-xover.json`), [`DynStatic::load`] reads `dyn-static.json`. Both return
//! `Ok(None)` when the file is absent so tests skip gracefully. The directory is
//! `$WING_REFERENCE_CURVES` or `<repo>/data/reference-curves`.
//!
//! [`Deviation`] summarises measured-vs-reference differences over a mask (e.g. only where the reference is
//! above -60 dB). Not RT code: everything here allocates freely. JSON is read by a small built-in parser so
//! wing-blocks keeps zero non-DSP dependencies.

use std::path::PathBuf;

use crate::model::Model;

/// Block length used to drive models.
pub const BLOCK: usize = 64;

// ---- measurement ------------------------------------------------------------------------------------------

/// Run `input` (mono, fed to both sides) through `m` in [`BLOCK`]-sample blocks; returns the left output.
pub fn run_mono(m: &mut dyn Model, input: &[f32]) -> Vec<f32> {
    let mut out = input.to_vec();
    let mut r = input.to_vec();
    for (lc, rc) in out.chunks_mut(BLOCK).zip(r.chunks_mut(BLOCK)) {
        m.process(lc, rc, None);
    }
    out
}

/// Magnitude response (dB) at `freqs` from an impulse response of `ir_len` samples (model reset first).
pub fn magnitude_response(m: &mut dyn Model, fs: f64, freqs: &[f64], ir_len: usize) -> Vec<f64> {
    m.reset();
    let mut x = vec![0.0f32; ir_len];
    x[0] = 1.0;
    let h: Vec<f64> = run_mono(m, &x).into_iter().map(f64::from).collect();
    freqs
        .iter()
        .map(|&f| {
            let w = 2.0 * core::f64::consts::PI * f / fs;
            20.0 * wing_dsp::dtft(&h, w).norm().max(1e-30).log10()
        })
        .collect()
}

/// Static curve: for each input level (dBFS), reset, feed DC at that level for `settle_s` and report the
/// output level (dBFS) at the end.
pub fn static_curve(m: &mut dyn Model, fs: f64, levels_db: &[f64], settle_s: f64) -> Vec<f64> {
    let n = ((settle_s * fs) as usize).div_ceil(BLOCK) * BLOCK;
    levels_db
        .iter()
        .map(|&db| {
            m.reset();
            let x = vec![10f64.powf(db / 20.0) as f32; n];
            let y = run_mono(m, &x);
            20.0 * (y[n - 1].abs() as f64).max(1e-30).log10()
        })
        .collect()
}

/// Measured attack/release times (ms) of the gain change, 10 % -> 90 % in dB.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TimeConstants {
    pub attack_ms: f64,
    pub release_ms: f64,
    /// Total gain change (dB) after the step up and after the step down.
    pub delta_up_db: f64,
    pub delta_down_db: f64,
}

/// Step the DC input from `low_db` to `high_db` (after `phase_s` at `low_db`), hold `phase_s`, step back,
/// hold `phase_s`; derive 10-90 % times of the gain (output/input in dB) trajectory after each step. A step
/// with no gain change reports 0 ms.
pub fn time_constants(
    m: &mut dyn Model,
    fs: f64,
    low_db: f64,
    high_db: f64,
    phase_s: f64,
) -> TimeConstants {
    let n = ((phase_s * fs) as usize).div_ceil(BLOCK) * BLOCK;
    let (lo, hi) = (
        10f64.powf(low_db / 20.0) as f32,
        10f64.powf(high_db / 20.0) as f32,
    );
    let mut x = vec![lo; 3 * n];
    x[n..2 * n].fill(hi);
    m.reset();
    let y = run_mono(m, &x);
    let gain: Vec<f64> = y
        .iter()
        .zip(&x)
        .map(|(&o, &i)| 20.0 * ((o / i).abs() as f64).max(1e-12).log10())
        .collect();
    let seg = |a: usize, b: usize| -> (f64, f64) {
        let g = &gain[a..b];
        let (g0, g1) = (gain[a - 1], g[g.len() - 1]);
        let d = g1 - g0;
        if d.abs() < 1e-3 {
            return (0.0, d);
        }
        let t = |frac: f64| {
            g.iter()
                .position(|&v| (v - g0) / d >= frac)
                .unwrap_or(g.len()) as f64
        };
        ((t(0.9) - t(0.1)) / fs * 1e3, d)
    };
    let (attack_ms, delta_up_db) = seg(n, 2 * n);
    let (release_ms, delta_down_db) = seg(2 * n, 3 * n);
    TimeConstants {
        attack_ms,
        release_ms,
        delta_up_db,
        delta_down_db,
    }
}

/// Magnitude (dB) of a reference biquad cascade `[b0,b1,b2,a1,a2]` at `f` Hz.
pub fn cascade_db(biquads: &[[f64; 5]], fs: f64, f: f64) -> f64 {
    let w = 2.0 * core::f64::consts::PI * f / fs;
    let (c1, s1, c2, s2) = (w.cos(), -w.sin(), (2.0 * w).cos(), -(2.0 * w).sin());
    let mut db = 0.0;
    for q in biquads {
        let (nr, ni) = (q[0] + q[1] * c1 + q[2] * c2, q[1] * s1 + q[2] * s2);
        let (dr, di) = (1.0 + q[3] * c1 + q[4] * c2, q[3] * s1 + q[4] * s2);
        db += 10.0
            * ((nr * nr + ni * ni) / (dr * dr + di * di))
                .max(1e-300)
                .log10();
    }
    db
}

// ---- comparison -------------------------------------------------------------------------------------------

/// Measured vs reference deviation summary.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Deviation {
    /// Largest |measured - reference| (dB) over the compared points.
    pub max_abs_db: f64,
    pub rms_db: f64,
    /// Index of the worst point.
    pub worst: usize,
    /// Number of compared points.
    pub n: usize,
}

impl Deviation {
    /// Compare point by point where `mask(i)` holds.
    pub fn between(measured: &[f64], reference: &[f64], mask: impl Fn(usize) -> bool) -> Self {
        let mut d = Deviation::default();
        let mut sq = 0.0;
        for (i, (m, r)) in measured.iter().zip(reference).enumerate() {
            if !mask(i) {
                continue;
            }
            let e = (m - r).abs();
            sq += e * e;
            d.n += 1;
            if e > d.max_abs_db {
                d.max_abs_db = e;
                d.worst = i;
            }
        }
        if d.n > 0 {
            d.rms_db = (sq / d.n as f64).sqrt();
        }
        d
    }

    pub fn within(&self, tol_db: f64) -> bool {
        self.max_abs_db <= tol_db
    }
}

// ---- reference files --------------------------------------------------------------------------------------

/// Directory holding the reference curves.
pub fn reference_dir() -> PathBuf {
    match std::env::var_os("WING_REFERENCE_CURVES") {
        Some(p) => PathBuf::from(p),
        None => PathBuf::from(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../data/reference-curves"
        )),
    }
}

fn read_reference(name: &str) -> Result<Option<json::Value>, String> {
    let path = reference_dir().join(name);
    let text = match std::fs::read_to_string(&path) {
        Ok(t) => t,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(None),
        Err(e) => return Err(format!("{}: {e}", path.display())),
    };
    let v = json::parse(&text).map_err(|e| format!("{}: {e}", path.display()))?;
    match v.get("format").and_then(|f| f.as_str()) {
        Some("wing-reference-curves/1") => Ok(Some(v)),
        other => Err(format!("{}: unsupported format {other:?}", path.display())),
    }
}

/// A parameter value recorded with a reference curve.
#[derive(Clone, Debug, PartialEq)]
pub enum RefParam {
    Num(f64),
    Str(String),
}

/// Parameters of one reference curve (as read back from WING-Edit, i.e. on the parameter grid).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct RefParams(pub Vec<(String, RefParam)>);

impl RefParams {
    fn from_json(v: Option<&json::Value>) -> Self {
        let mut out = Vec::new();
        if let Some(json::Value::Obj(kv)) = v {
            for (k, v) in kv {
                let p = match v {
                    json::Value::Num(n) => RefParam::Num(*n),
                    json::Value::Bool(b) => RefParam::Num(*b as u8 as f64),
                    json::Value::Str(s) => RefParam::Str(s.clone()),
                    _ => continue,
                };
                out.push((k.clone(), p));
            }
        }
        Self(out)
    }

    pub fn get(&self, key: &str) -> Option<&RefParam> {
        self.0.iter().find(|(k, _)| k == key).map(|(_, v)| v)
    }

    /// Numeric value (strings that parse as numbers are accepted, e.g. `lcs: "18"`).
    pub fn num(&self, key: &str) -> Option<f64> {
        match self.get(key)? {
            RefParam::Num(n) => Some(*n),
            RefParam::Str(s) => s.parse().ok(),
        }
    }

    pub fn str(&self, key: &str) -> Option<&str> {
        match self.get(key)? {
            RefParam::Str(s) => Some(s),
            RefParam::Num(_) => None,
        }
    }
}

/// One frequency-response reference curve.
#[derive(Clone, Debug, PartialEq)]
pub struct FreqCurve {
    pub case: String,
    pub params: RefParams,
    /// Exact magnitude of the traced cascade (dB) at the file's `freq_hz`.
    pub mag_db: Vec<f64>,
    /// The cascade `[b0,b1,b2,a1,a2]` WING-Edit computes for these parameters.
    pub biquads: Vec<[f64; 5]>,
}

/// A frequency-response reference file.
#[derive(Clone, Debug, PartialEq)]
pub struct FreqCurves {
    pub fs: f64,
    pub block: String,
    pub freq_hz: Vec<f64>,
    pub curves: Vec<FreqCurve>,
}

impl FreqCurves {
    /// Load `name` (e.g. `filter.json`); `Ok(None)` if the file does not exist.
    pub fn load(name: &str) -> Result<Option<Self>, String> {
        let Some(v) = read_reference(name)? else {
            return Ok(None);
        };
        let nums = |v: Option<&json::Value>| -> Vec<f64> {
            v.and_then(|a| a.as_arr())
                .map(|a| a.iter().filter_map(|x| x.as_f64()).collect())
                .unwrap_or_default()
        };
        let curves = v
            .get("curves")
            .and_then(|c| c.as_arr())
            .ok_or_else(|| format!("{name}: no curves"))?
            .iter()
            .map(|c| FreqCurve {
                case: c
                    .get("case")
                    .and_then(|s| s.as_str())
                    .unwrap_or("")
                    .to_string(),
                params: RefParams::from_json(c.get("params")),
                mag_db: nums(c.get("mag_db")),
                biquads: c
                    .get("biquads")
                    .and_then(|b| b.as_arr())
                    .map(|b| {
                        b.iter()
                            .filter_map(|q| {
                                let q = nums(Some(q));
                                (q.len() == 5).then(|| [q[0], q[1], q[2], q[3], q[4]])
                            })
                            .collect()
                    })
                    .unwrap_or_default(),
            })
            .collect();
        Ok(Some(Self {
            fs: v.get("fs").and_then(|f| f.as_f64()).unwrap_or(48_000.0),
            block: v
                .get("block")
                .and_then(|s| s.as_str())
                .unwrap_or("")
                .to_string(),
            freq_hz: nums(v.get("freq_hz")),
            curves,
        }))
    }
}

/// One static dynamics curve (`out_db` over [`DynStatic::input_db`]).
#[derive(Clone, Debug, PartialEq)]
pub struct StaticCurve {
    pub params: RefParams,
    pub out_db: Vec<f64>,
}

/// `dyn-static.json`: GATE/DUCK/COMP/EXP static laws.
#[derive(Clone, Debug, PartialEq)]
pub struct DynStatic {
    pub input_db: Vec<f64>,
    /// `(model id, curves)`.
    pub models: Vec<(String, Vec<StaticCurve>)>,
}

impl DynStatic {
    pub fn load() -> Result<Option<Self>, String> {
        let Some(v) = read_reference("dyn-static.json")? else {
            return Ok(None);
        };
        let nums = |v: Option<&json::Value>| -> Vec<f64> {
            v.and_then(|a| a.as_arr())
                .map(|a| a.iter().filter_map(|x| x.as_f64()).collect())
                .unwrap_or_default()
        };
        let mut models = Vec::new();
        if let Some(json::Value::Obj(kv)) = v.get("models") {
            for (id, m) in kv {
                let curves = m
                    .get("curves")
                    .and_then(|c| c.as_arr())
                    .map(|cs| {
                        cs.iter()
                            .map(|c| StaticCurve {
                                params: RefParams::from_json(c.get("params")),
                                out_db: nums(c.get("out_db")),
                            })
                            .collect()
                    })
                    .unwrap_or_default();
                models.push((id.clone(), curves));
            }
        }
        Ok(Some(Self {
            input_db: nums(v.get("input_db")),
            models,
        }))
    }

    pub fn curves(&self, model: &str) -> &[StaticCurve] {
        self.models
            .iter()
            .find(|(m, _)| m == model)
            .map(|(_, c)| &c[..])
            .unwrap_or(&[])
    }
}

/// Minimal JSON reader (numbers as f64). Enough for the reference-curve files.
pub mod json {
    use std::fmt;

    #[derive(Clone, Debug, PartialEq)]
    pub enum Value {
        Null,
        Bool(bool),
        Num(f64),
        Str(String),
        Arr(Vec<Value>),
        Obj(Vec<(String, Value)>),
    }

    impl Value {
        pub fn get(&self, key: &str) -> Option<&Value> {
            match self {
                Value::Obj(kv) => kv.iter().find(|(k, _)| k == key).map(|(_, v)| v),
                _ => None,
            }
        }
        pub fn as_f64(&self) -> Option<f64> {
            match self {
                Value::Num(n) => Some(*n),
                _ => None,
            }
        }
        pub fn as_str(&self) -> Option<&str> {
            match self {
                Value::Str(s) => Some(s),
                _ => None,
            }
        }
        pub fn as_arr(&self) -> Option<&[Value]> {
            match self {
                Value::Arr(a) => Some(a),
                _ => None,
            }
        }
    }

    #[derive(Clone, Debug, PartialEq, Eq)]
    pub struct Error {
        pub offset: usize,
        pub msg: &'static str,
    }

    impl fmt::Display for Error {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            write!(f, "JSON error at byte {}: {}", self.offset, self.msg)
        }
    }

    pub fn parse(text: &str) -> Result<Value, Error> {
        let mut p = Parser {
            b: text.as_bytes(),
            i: 0,
        };
        let v = p.value()?;
        p.ws();
        if p.i != p.b.len() {
            return Err(p.err("trailing characters"));
        }
        Ok(v)
    }

    struct Parser<'a> {
        b: &'a [u8],
        i: usize,
    }

    impl Parser<'_> {
        fn err(&self, msg: &'static str) -> Error {
            Error {
                offset: self.i,
                msg,
            }
        }
        fn ws(&mut self) {
            while self.i < self.b.len() && self.b[self.i].is_ascii_whitespace() {
                self.i += 1;
            }
        }
        fn eat(&mut self, c: u8) -> Result<(), Error> {
            self.ws();
            if self.b.get(self.i) == Some(&c) {
                self.i += 1;
                Ok(())
            } else {
                Err(self.err("unexpected character"))
            }
        }
        fn lit(&mut self, s: &str, v: Value) -> Result<Value, Error> {
            if self.b[self.i..].starts_with(s.as_bytes()) {
                self.i += s.len();
                Ok(v)
            } else {
                Err(self.err("bad literal"))
            }
        }
        fn value(&mut self) -> Result<Value, Error> {
            self.ws();
            match self.b.get(self.i) {
                Some(b'{') => {
                    self.i += 1;
                    let mut kv = Vec::new();
                    self.ws();
                    if self.b.get(self.i) == Some(&b'}') {
                        self.i += 1;
                        return Ok(Value::Obj(kv));
                    }
                    loop {
                        self.ws();
                        let k = self.string()?;
                        self.eat(b':')?;
                        kv.push((k, self.value()?));
                        self.ws();
                        match self.b.get(self.i) {
                            Some(b',') => self.i += 1,
                            Some(b'}') => {
                                self.i += 1;
                                return Ok(Value::Obj(kv));
                            }
                            _ => return Err(self.err("expected , or }")),
                        }
                    }
                }
                Some(b'[') => {
                    self.i += 1;
                    let mut a = Vec::new();
                    self.ws();
                    if self.b.get(self.i) == Some(&b']') {
                        self.i += 1;
                        return Ok(Value::Arr(a));
                    }
                    loop {
                        a.push(self.value()?);
                        self.ws();
                        match self.b.get(self.i) {
                            Some(b',') => self.i += 1,
                            Some(b']') => {
                                self.i += 1;
                                return Ok(Value::Arr(a));
                            }
                            _ => return Err(self.err("expected , or ]")),
                        }
                    }
                }
                Some(b'"') => Ok(Value::Str(self.string()?)),
                Some(b't') => self.lit("true", Value::Bool(true)),
                Some(b'f') => self.lit("false", Value::Bool(false)),
                Some(b'n') => self.lit("null", Value::Null),
                Some(_) => {
                    let s = self.i;
                    while self.i < self.b.len()
                        && matches!(
                            self.b[self.i],
                            b'-' | b'+' | b'.' | b'e' | b'E' | b'0'..=b'9'
                        )
                    {
                        self.i += 1;
                    }
                    std::str::from_utf8(&self.b[s..self.i])
                        .ok()
                        .and_then(|t| t.parse().ok())
                        .map(Value::Num)
                        .ok_or_else(|| self.err("bad number"))
                }
                None => Err(self.err("unexpected end")),
            }
        }
        fn string(&mut self) -> Result<String, Error> {
            if self.b.get(self.i) != Some(&b'"') {
                return Err(self.err("expected string"));
            }
            self.i += 1;
            let mut out = Vec::new();
            loop {
                match self.b.get(self.i) {
                    None => return Err(self.err("unterminated string")),
                    Some(b'"') => {
                        self.i += 1;
                        return String::from_utf8(out).map_err(|_| self.err("bad utf-8"));
                    }
                    Some(b'\\') => {
                        let c = *self
                            .b
                            .get(self.i + 1)
                            .ok_or_else(|| self.err("bad escape"))?;
                        self.i += 2;
                        match c {
                            b'n' => out.push(b'\n'),
                            b't' => out.push(b'\t'),
                            b'r' => out.push(b'\r'),
                            b'b' => out.push(8),
                            b'f' => out.push(12),
                            b'u' => {
                                let h = self
                                    .b
                                    .get(self.i..self.i + 4)
                                    .and_then(|h| std::str::from_utf8(h).ok())
                                    .and_then(|h| u32::from_str_radix(h, 16).ok())
                                    .ok_or_else(|| self.err("bad \\u escape"))?;
                                self.i += 4;
                                let ch = char::from_u32(h).unwrap_or('\u{fffd}');
                                let mut buf = [0u8; 4];
                                out.extend_from_slice(ch.encode_utf8(&mut buf).as_bytes());
                            }
                            other => out.push(other),
                        }
                    }
                    Some(&c) => {
                        out.push(c);
                        self.i += 1;
                    }
                }
            }
        }
    }

    #[cfg(test)]
    mod tests {
        use super::*;

        #[test]
        fn parses_reference_like_json() {
            let v = parse(r#"{"a":[1,-2.5e-1,true,null],"s":"x\"yA","o":{}}"#).unwrap();
            assert_eq!(
                v.get("a").unwrap().as_arr().unwrap()[1].as_f64(),
                Some(-0.25)
            );
            assert_eq!(v.get("s").unwrap().as_str(), Some("x\"yA"));
            assert!(parse("[1,").is_err());
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::{create_model, BlockKind, ParamValue};
    use crate::std_models::dyn_slot;

    #[test]
    fn measures_comp_static_and_times() {
        let mut c = create_model(BlockKind::Dynamics, "COMP");
        c.prepare(48_000.0, BLOCK);
        c.set_param(dyn_slot::THR, ParamValue::Float(-20.0));
        c.set_param(dyn_slot::RATIO, ParamValue::Float(4.0));
        c.set_param(dyn_slot::KNEE, ParamValue::Int(0));
        c.set_param(dyn_slot::AUTO, ParamValue::Int(0));
        let out = static_curve(&mut *c, 48_000.0, &[-40.0, -20.0, 0.0], 1.0);
        assert!((out[0] + 40.0).abs() < 0.1, "{out:?}");
        assert!((out[2] + 15.0).abs() < 0.3, "{out:?}");
        let t = time_constants(&mut *c, 48_000.0, -40.0, 0.0, 1.0);
        assert!(t.attack_ms > 1.0 && t.release_ms > t.attack_ms, "{t:?}");
        assert!((t.delta_up_db + 15.0).abs() < 0.5, "{t:?}");
    }

    #[test]
    fn stub_is_flat_and_cascade_eval() {
        let mut s = create_model(BlockKind::Fx, "HALL");
        let m = magnitude_response(&mut *s, 48_000.0, &[100.0, 1000.0], 256);
        assert!(m.iter().all(|d| d.abs() < 1e-9));
        // unity and a pure gain section
        assert!(cascade_db(&[[1.0, 0.0, 0.0, 0.0, 0.0]], 48_000.0, 1000.0).abs() < 1e-12);
        assert!((cascade_db(&[[2.0, 0.0, 0.0, 0.0, 0.0]], 48_000.0, 50.0) - 6.0206).abs() < 1e-3);
        let d = Deviation::between(&[0.0, 1.0, 5.0], &[0.0, 0.5, 0.0], |i| i < 2);
        assert_eq!((d.n, d.worst), (2, 1));
        assert!(d.within(0.5) && !d.within(0.4));
    }
}
