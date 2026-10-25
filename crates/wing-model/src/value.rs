//! Values and conversions: clamping, step quantisation, lin/log/fader tapers, normalised
//! positions, enum lookup and WING-Edit display formatting.
//!
//! Raw encoding (one `u32` per parameter): `LinF`/`LogF`/`Fader` = f32 bits of the engineering
//! value (dB, Hz, ms, ...), `Int` = i32, `Enum`/`FEnum` = index, `Str` = change generation (the
//! text lives in the store's string table).

use std::fmt::{self, Write};

use crate::schema::{Desc, EnumItem, ParamType, Schema};

/// Floor of the fader law; displayed as `-oo`.
pub const FADER_MIN_DB: f32 = -144.0;

/// A typed parameter value.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Value {
    /// Engineering value of a float parameter (dB, Hz, %, ...).
    Float(f32),
    /// Integer / bool / trigger.
    Int(i32),
    /// Enum or float-enum index.
    Enum(u32),
    /// String parameter (read the text with `ParamStore::with_str`).
    Str,
}

/// Why a set was rejected.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SetError {
    /// The node is a container, not a parameter.
    NotParam,
    /// The slot is not used by the block's active model.
    Inactive,
    /// The value kind does not fit the parameter (e.g. number for a string).
    TypeMismatch,
    /// Unknown enum name or unparsable text.
    BadValue,
}

impl fmt::Display for SetError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            SetError::NotParam => "not a parameter",
            SetError::Inactive => "slot not used by the active model",
            SetError::TypeMismatch => "value type does not match the parameter",
            SetError::BadValue => "invalid value",
        })
    }
}

impl std::error::Error for SetError {}

/// WING fader law (`ncl::Cpar::recalcFIB`): normalised position -> dB.
pub fn fader_pos_to_db(p: f32) -> f32 {
    let p = if p.is_nan() { 0.0 } else { p.clamp(0.0, 1.0) };
    if p < 1.0 / 1024.0 {
        FADER_MIN_DB
    } else if p < 0.0625 {
        480.0 * p - 90.0
    } else if p < 0.25 {
        160.0 * p - 70.0
    } else if p < 0.5 {
        80.0 * p - 50.0
    } else {
        40.0 * p - 30.0
    }
}

/// Inverse fader law: dB -> normalised position (0 for -oo and anything below the curve).
pub fn fader_db_to_pos(db: f32) -> f32 {
    if db.is_nan() || db <= FADER_MIN_DB {
        return 0.0;
    }
    let p = if db >= -10.0 {
        (db + 30.0) / 40.0
    } else if db >= -30.0 {
        (db + 50.0) / 80.0
    } else if db >= -60.0 {
        (db + 70.0) / 160.0
    } else {
        (db + 90.0) / 480.0
    };
    if p < 1.0 / 1024.0 {
        0.0
    } else {
        p.min(1.0)
    }
}

impl Desc {
    /// Enum items (empty for non-enums and for the dynamic `ae::Cstringlist`).
    pub fn enum_items(&self) -> &'static [EnumItem] {
        Schema::get().enum_items(self)
    }

    /// Number of discrete positions (`steps` for floats, `max-min+1` for ints, item count for
    /// enums); 0 when continuous (fader) or unbounded.
    pub fn positions(&self) -> u32 {
        match self.ty {
            ParamType::LinF | ParamType::LogF => self.steps,
            ParamType::Int => (self.max - self.min) as u32 + 1,
            ParamType::Enum | ParamType::FEnum => self.enum_len,
            ParamType::Fader | ParamType::Str => 0,
        }
    }

    /// Step index of a float value on the lin/log grid (clamped).
    pub fn step_index(&self, v: f64) -> u32 {
        let last = self.steps.saturating_sub(1) as f64;
        let i = match self.ty {
            ParamType::LinF if self.step != 0.0 => (v - self.min) / self.step,
            // v <= 0 on a log taper lands on the first step (falls through to 0)
            ParamType::LogF if self.step > 0.0 && self.min > 0.0 && v > 0.0 => {
                (v / self.min).ln() / self.step
            }
            _ => 0.0,
        };
        if i.is_nan() {
            0
        } else {
            i.round().clamp(0.0, last) as u32
        }
    }

    /// Value of grid position `i` (`min + step*i` or `min * exp(logstep*i)`).
    pub fn value_at(&self, i: u32) -> f64 {
        let i = i.min(self.steps.saturating_sub(1)) as f64;
        match self.ty {
            ParamType::LogF => self.min * (self.step * i).exp(),
            _ => self.min + self.step * i,
        }
    }

    /// Clamp and quantise a float to what the console would store.
    pub fn quantize(&self, v: f64) -> f32 {
        match self.ty {
            ParamType::LinF | ParamType::LogF if self.steps >= 2 && self.step != 0.0 => {
                self.value_at(self.step_index(v)) as f32
            }
            ParamType::Fader => {
                if v.is_nan() || v <= FADER_MIN_DB as f64 {
                    FADER_MIN_DB
                } else {
                    v.min(self.max) as f32
                }
            }
            _ => {
                if v.is_nan() {
                    self.min as f32
                } else {
                    v.clamp(self.min.min(self.max), self.max.max(self.min)) as f32
                }
            }
        }
    }

    fn clamp_index(&self, i: i64) -> u32 {
        if self.enum_len == 0 {
            i.max(0) as u32 // dynamic list: no bound known
        } else {
            i.clamp(0, self.enum_len as i64 - 1) as u32
        }
    }

    fn clamp_int(&self, i: i64) -> i32 {
        i.clamp(self.min as i64, (self.max as i64).max(self.min as i64)) as i32
    }

    /// Validate, clamp and quantise a value into its raw encoding.
    pub fn encode(&self, v: Value) -> Result<u32, SetError> {
        let num = match v {
            Value::Float(f) => f as f64,
            Value::Int(i) => i as f64,
            Value::Enum(i) => i as f64,
            Value::Str => return Err(SetError::TypeMismatch),
        };
        Ok(match self.ty {
            ParamType::LinF | ParamType::LogF | ParamType::Fader => {
                if matches!(v, Value::Enum(_)) {
                    return Err(SetError::TypeMismatch);
                }
                self.quantize(num).to_bits()
            }
            ParamType::Int => self.clamp_int(round_i64(num)) as u32,
            ParamType::Enum => self.clamp_index(round_i64(num)),
            ParamType::FEnum => match v {
                Value::Float(f) => self.nearest_fenum(f as f64),
                _ => self.clamp_index(round_i64(num)),
            },
            ParamType::Str => return Err(SetError::TypeMismatch),
        })
    }

    fn nearest_fenum(&self, f: f64) -> u32 {
        let items = self.enum_items();
        let mut best = 0;
        for (i, it) in items.iter().enumerate() {
            if (it.value - f).abs() < (items[best].value - f).abs() {
                best = i;
            }
        }
        best as u32
    }

    /// Decode a raw value.
    pub fn decode(&self, raw: u32) -> Value {
        match self.ty {
            ParamType::LinF | ParamType::LogF | ParamType::Fader => {
                Value::Float(f32::from_bits(raw))
            }
            ParamType::Int => Value::Int(raw as i32),
            ParamType::Enum | ParamType::FEnum => Value::Enum(raw),
            ParamType::Str => Value::Str,
        }
    }

    /// Numeric view of a raw value (float engineering value, int, index; FEnum = its float).
    pub fn as_f64(&self, raw: u32) -> f64 {
        match self.ty {
            ParamType::LinF | ParamType::LogF | ParamType::Fader => f32::from_bits(raw) as f64,
            ParamType::Int => raw as i32 as f64,
            ParamType::FEnum => self.enum_items().get(raw as usize).map_or(0.0, |e| e.value),
            ParamType::Enum => raw as f64,
            ParamType::Str => 0.0,
        }
    }

    /// Raw value -> normalised position 0..1 (fader law for `Fader`, grid index otherwise).
    pub fn to_norm(&self, raw: u32) -> f32 {
        let frac = |i: f64, n: f64| {
            if n > 0.0 {
                (i / n).clamp(0.0, 1.0) as f32
            } else {
                0.0
            }
        };
        match self.ty {
            ParamType::Fader => fader_db_to_pos(f32::from_bits(raw)),
            ParamType::LinF | ParamType::LogF => {
                let v = f32::from_bits(raw) as f64;
                if self.steps >= 2 && self.step != 0.0 {
                    frac(self.step_index(v) as f64, (self.steps - 1) as f64)
                } else {
                    frac(v - self.min, self.max - self.min)
                }
            }
            ParamType::Int => frac(raw as i32 as f64 - self.min, self.max - self.min),
            ParamType::Enum | ParamType::FEnum => {
                frac(raw as f64, self.enum_len.saturating_sub(1) as f64)
            }
            ParamType::Str => 0.0,
        }
    }

    /// Normalised position 0..1 -> raw value.
    pub fn from_norm(&self, p: f32) -> Result<u32, SetError> {
        let p = if p.is_nan() {
            0.0
        } else {
            p.clamp(0.0, 1.0) as f64
        };
        Ok(match self.ty {
            ParamType::Fader => fader_pos_to_db(p as f32).to_bits(),
            ParamType::LinF | ParamType::LogF => {
                if self.steps >= 2 && self.step != 0.0 {
                    let i = (p * (self.steps - 1) as f64).round() as u32;
                    (self.value_at(i) as f32).to_bits()
                } else {
                    self.quantize(self.min + p * (self.max - self.min))
                        .to_bits()
                }
            }
            ParamType::Int => {
                self.clamp_int((self.min + p * (self.max - self.min)).round() as i64) as u32
            }
            ParamType::Enum | ParamType::FEnum => {
                self.clamp_index((p * self.enum_len.saturating_sub(1) as f64).round() as i64)
            }
            ParamType::Str => return Err(SetError::TypeMismatch),
        })
    }

    /// Enum index by wire token (exact, then case-insensitive) or display name (case-insensitive).
    pub fn enum_index(&self, name: &str) -> Option<u32> {
        let items = self.enum_items();
        let name = name.trim();
        items
            .iter()
            .position(|e| e.token == name)
            .or_else(|| {
                items
                    .iter()
                    .position(|e| e.token.eq_ignore_ascii_case(name))
            })
            .or_else(|| items.iter().position(|e| e.long.eq_ignore_ascii_case(name)))
            .map(|i| i as u32)
    }

    /// Wire/snapshot token of an enum value (`SHV`); `None` if not an enum or out of range.
    pub fn enum_token(&self, raw: u32) -> Option<&'static str> {
        self.enum_items().get(raw as usize).map(|e| e.token)
    }

    /// Parse display or snapshot text (`-40.0`, `2k00`, `-oo`, `1:30`, `SHV`, `ON`) to a raw value.
    pub fn parse_text(&self, text: &str) -> Result<u32, SetError> {
        let mut t = text.trim();
        let unit = self.unit.as_str();
        if !unit.is_empty() {
            if let Some(s) = t.strip_suffix(unit) {
                t = s.trim_end();
            }
        }
        match self.ty {
            ParamType::Enum | ParamType::FEnum => {
                if let Some(i) = self.enum_index(t) {
                    return Ok(i);
                }
                if let Some(i) = self
                    .enum_items()
                    .iter()
                    .position(|e| e.long.eq_ignore_ascii_case(text.trim()))
                {
                    return Ok(i as u32);
                }
                let n = parse_number(t).ok_or(SetError::BadValue)?;
                if self.ty == ParamType::FEnum {
                    Ok(self.nearest_fenum(n))
                } else {
                    self.encode(Value::Int(n.round() as i32))
                }
            }
            ParamType::Int => {
                let n = if t.eq_ignore_ascii_case("on") || t.eq_ignore_ascii_case("true") {
                    1.0
                } else if t.eq_ignore_ascii_case("off") || t.eq_ignore_ascii_case("false") {
                    0.0
                } else {
                    parse_number(t).ok_or(SetError::BadValue)?
                };
                self.encode(Value::Int(n.round() as i32))
            }
            ParamType::LinF | ParamType::LogF | ParamType::Fader => {
                let n = if self.fmt == "%T" || self.fmt == "%t" {
                    parse_time(t, self.fmt == "%t").ok_or(SetError::BadValue)?
                } else {
                    parse_number(t).ok_or(SetError::BadValue)?
                };
                self.encode(Value::Float(n as f32))
            }
            ParamType::Str => Err(SetError::TypeMismatch),
        }
    }

    /// Write the value as WING-Edit displays it, without unit (`2k00`, `-oo`, `+3.0`, `SHELV`).
    /// Strings are not handled here (see `ParamStore::format`).
    pub fn format<W: Write>(&self, raw: u32, w: &mut W) -> fmt::Result {
        match self.ty {
            ParamType::Fader => {
                let v = f32::from_bits(raw);
                if v <= FADER_MIN_DB {
                    w.write_str("-oo")
                } else {
                    format_float(w, "%.1f", v as f64)
                }
            }
            ParamType::LinF | ParamType::LogF => {
                let fmt = if self.fmt.is_empty() {
                    default_fmt(self.step)
                } else {
                    self.fmt
                };
                format_float(w, fmt, f32::from_bits(raw) as f64)
            }
            ParamType::Int => write!(w, "{}", raw as i32),
            ParamType::Enum | ParamType::FEnum => match self.enum_items().get(raw as usize) {
                Some(e) => w.write_str(e.long),
                None => write!(w, "{raw}"),
            },
            ParamType::Str => Ok(()),
        }
    }

    /// Like [`Desc::format`] followed by ` <unit>` when the parameter has one.
    pub fn format_with_unit<W: Write>(&self, raw: u32, w: &mut W) -> fmt::Result {
        self.format(raw, w)?;
        if matches!(
            self.ty,
            ParamType::LinF | ParamType::LogF | ParamType::Fader | ParamType::Int
        ) && !self.unit.as_str().is_empty()
        {
            write!(w, " {}", self.unit.as_str())?;
        }
        Ok(())
    }
}

fn round_i64(v: f64) -> i64 {
    if v.is_nan() {
        0
    } else {
        v.round().clamp(i64::MIN as f64, i64::MAX as f64) as i64
    }
}

fn default_fmt(step: f64) -> &'static str {
    if step >= 1.0 {
        "%.0f"
    } else if step >= 0.1 {
        "%.1f"
    } else {
        "%.2f"
    }
}

/// Parse a number with WING's kilo notation (`2k00` = 2000, `20k0`, `1k5`) and `-oo`.
pub fn parse_number(t: &str) -> Option<f64> {
    let t = t.trim();
    if t == "-oo" || t.eq_ignore_ascii_case("-inf") {
        return Some(FADER_MIN_DB as f64);
    }
    if let Some((a, b)) = t.split_once(['k', 'K']) {
        if b.bytes().all(|c| c.is_ascii_digit()) && !a.is_empty() {
            let int: f64 = a.parse().ok()?;
            let frac: f64 = if b.is_empty() {
                0.0
            } else {
                format_args_parse_frac(b)?
            };
            let v = int.abs() + frac;
            return Some(if a.starts_with('-') { -v } else { v } * 1000.0);
        }
        return None;
    }
    t.parse().ok()
}

fn format_args_parse_frac(digits: &str) -> Option<f64> {
    let mut v = 0.0;
    let mut scale = 0.1;
    for c in digits.bytes() {
        v += (c - b'0') as f64 * scale;
        scale *= 0.1;
    }
    Some(v)
}

fn parse_time(t: &str, millis: bool) -> Option<f64> {
    let mut secs = 0.0;
    for part in t.split(':') {
        secs = secs * 60.0 + part.trim().parse::<f64>().ok()?;
    }
    Some(if millis { secs * 1000.0 } else { secs })
}

/// Small stack buffer implementing `fmt::Write` (used to post-process formatted numbers).
pub struct StackStr<const N: usize> {
    buf: [u8; N],
    len: usize,
}

impl<const N: usize> StackStr<N> {
    pub const fn new() -> Self {
        Self {
            buf: [0; N],
            len: 0,
        }
    }
    pub fn as_str(&self) -> &str {
        // only whole &str pieces are ever appended
        std::str::from_utf8(&self.buf[..self.len]).unwrap_or("")
    }
    pub fn clear(&mut self) {
        self.len = 0;
    }
}

impl<const N: usize> Default for StackStr<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> Write for StackStr<N> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let b = s.as_bytes();
        if self.len + b.len() > N {
            return Err(fmt::Error);
        }
        self.buf[self.len..self.len + b.len()].copy_from_slice(b);
        self.len += b.len();
        Ok(())
    }
}

/// Format per a WING-Edit printf-like spec: `%[+][k][.N](f|T|t)`.
/// `k`: values >= 1000 print in kilo notation with one more decimal (`2k00`, `20k0`).
/// `T`: seconds as `m:ss` / `h:mm:ss`; `t`: milliseconds as `h:mm:ss`.
pub fn format_float<W: Write>(w: &mut W, spec: &str, v: f64) -> fmt::Result {
    let body = spec.strip_prefix('%').unwrap_or(spec);
    let (plus, body) = match body.strip_prefix('+') {
        Some(b) => (true, b),
        None => (false, body),
    };
    let (kilo, body) = match body.strip_prefix('k') {
        Some(b) => (true, b),
        None => (false, body),
    };
    let conv = body.chars().last().unwrap_or('f');
    let prec = body
        .strip_prefix('.')
        .map(|b| b.trim_end_matches(conv).parse::<usize>().unwrap_or(0))
        .unwrap_or(0);
    let v = if v == 0.0 { 0.0 } else { v }; // no "-0"
    match conv {
        'T' | 't' => {
            let secs = if conv == 't' { v / 1000.0 } else { v };
            let s = secs.max(0.0).round() as u64;
            let (h, m, s) = (s / 3600, (s / 60) % 60, s % 60);
            if h > 0 || conv == 't' {
                write!(w, "{h}:{m:02}:{s:02}")
            } else {
                write!(w, "{m}:{s:02}")
            }
        }
        _ if kilo && v.abs() >= 1000.0 => {
            let mut tmp = StackStr::<48>::new();
            if plus {
                write!(tmp, "{:+.*}", prec + 1, v / 1000.0)?;
            } else {
                write!(tmp, "{:.*}", prec + 1, v / 1000.0)?;
            }
            for c in tmp.as_str().chars() {
                w.write_char(if c == '.' { 'k' } else { c })?;
            }
            Ok(())
        }
        _ => {
            if plus {
                write!(w, "{v:+.prec$}")
            } else {
                write!(w, "{v:.prec$}")
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::schema::Schema;

    fn desc(path: &str) -> Desc {
        let s = Schema::get();
        *s.base_desc(s.by_path(path).unwrap()).unwrap()
    }

    fn fmt(d: &Desc, raw: u32) -> String {
        let mut s = String::new();
        d.format_with_unit(raw, &mut s).unwrap();
        s
    }

    #[test]
    fn fader_law_roundtrip_and_edges() {
        assert_eq!(fader_pos_to_db(0.0), -144.0);
        assert_eq!(fader_pos_to_db(1.0 / 2048.0), -144.0);
        assert_eq!(fader_pos_to_db(0.75), 0.0);
        assert_eq!(fader_pos_to_db(1.0), 10.0);
        assert_eq!(fader_pos_to_db(0.5), -10.0);
        assert_eq!(fader_pos_to_db(0.25), -30.0);
        assert_eq!(fader_pos_to_db(0.0625), -60.0);
        assert_eq!(fader_pos_to_db(f32::NAN), -144.0);
        for i in 1..=1024 {
            let p = i as f32 / 1024.0;
            let back = fader_db_to_pos(fader_pos_to_db(p));
            assert!((back - p).abs() < 1e-5, "{p} -> {back}");
        }
        assert_eq!(fader_db_to_pos(-144.0), 0.0);
        assert_eq!(fader_db_to_pos(-120.0), 0.0);
        assert_eq!(fader_db_to_pos(20.0), 1.0);
    }

    #[test]
    fn fader_clamps_without_quantising() {
        let d = desc("ch/1/fdr");
        assert_eq!(d.ty, ParamType::Fader);
        assert_eq!(f32::from_bits(d.encode(Value::Float(-6.0)).unwrap()), -6.0);
        assert_eq!(f32::from_bits(d.encode(Value::Float(25.0)).unwrap()), 10.0);
        assert_eq!(
            f32::from_bits(d.encode(Value::Float(-200.0)).unwrap()),
            -144.0
        );
        assert_eq!(
            f32::from_bits(d.encode(Value::Float(f32::NEG_INFINITY)).unwrap()),
            -144.0
        );
        assert_eq!(fmt(&d, (-144.0f32).to_bits()), "-oo dB");
        assert_eq!(fmt(&d, 0.0f32.to_bits()), "0.0 dB");
        assert_eq!(d.from_norm(0.75).unwrap(), 0.0f32.to_bits());
    }

    #[test]
    fn linear_quantisation() {
        let d = desc("ch/1/in/set/trim"); // lin -18..18, 361 steps (0.1)
        let q = |v: f32| f32::from_bits(d.encode(Value::Float(v)).unwrap());
        assert!((q(3.04) - 3.0).abs() < 1e-5);
        assert!((q(3.06) - 3.1).abs() < 1e-5);
        assert_eq!(q(99.0), 18.0);
        assert_eq!(q(-99.0), -18.0);
        assert_eq!(q(f32::NAN), -18.0);
        assert_eq!(d.to_norm(q(0.0).to_bits()), 0.5);
        assert_eq!(fmt(&d, q(3.0).to_bits()), "+3.0 dB");
        assert_eq!(fmt(&d, q(0.0).to_bits()), "+0.0 dB");
        assert_eq!(fmt(&d, q(-0.04).to_bits()), "+0.0 dB"); // no "-0.0"
        assert_eq!(f32::from_bits(d.parse_text("+3.0 dB").unwrap()), q(3.0));
    }

    #[test]
    fn log_quantisation_and_kilo_format() {
        let d = desc("ch/1/flt/lcf"); // log 20..2000 Hz, 641 steps
        let q = |v: f32| f32::from_bits(d.encode(Value::Float(v)).unwrap());
        // factory default 100.2374573 is grid index 224
        assert_eq!(d.step_index(100.2374573), 224);
        assert!((q(100.0) - 100.237_46).abs() < 1e-3);
        assert!((q(1e9) - 2000.0).abs() < 1e-2);
        assert_eq!(q(0.0), 20.0);
        assert_eq!(q(-5.0), 20.0);
        assert_eq!(fmt(&d, q(2000.0).to_bits()), "2k00 Hz");
        assert_eq!(fmt(&d, q(20.0).to_bits()), "20.0 Hz");
        assert_eq!(fmt(&d, q(1500.0).to_bits()), "1k50 Hz");
        assert!((f32::from_bits(d.parse_text("1k50").unwrap()) - q(1500.0)).abs() < 1e-3);
        assert_eq!(d.from_norm(1.0).unwrap(), q(2000.0).to_bits());
        assert_eq!(d.to_norm(q(20.0).to_bits()), 0.0);
    }

    #[test]
    fn enums_by_index_and_name() {
        let d = desc("ch/1/flt/lcs"); // 6 12 18 24
        assert_eq!(d.enum_index("24"), Some(3));
        assert_eq!(d.enum_index("12 dB/oct"), Some(1));
        assert_eq!(d.enum_index("13"), None);
        assert_eq!(d.encode(Value::Enum(9)).unwrap(), 3);
        assert_eq!(d.encode(Value::Int(-1)).unwrap(), 0);
        assert_eq!(d.parse_text("18").unwrap(), 2);
        assert_eq!(d.enum_token(2), Some("18"));
        assert_eq!(fmt(&d, 0), "6 dB/Oct");
        assert_eq!(d.to_norm(3), 1.0);
        assert_eq!(d.from_norm(0.34).unwrap(), 1);
        assert_eq!(d.encode(Value::Str), Err(SetError::TypeMismatch));
    }

    #[test]
    fn fenum_and_int() {
        let d = desc("cfg/rta/rtarange");
        assert_eq!(d.ty, ParamType::FEnum);
        assert_eq!(d.encode(Value::Float(55.0)).unwrap(), 1);
        assert_eq!(d.as_f64(1), 60.0);
        assert_eq!(d.parse_text("30 dB").unwrap(), 0);
        let b = desc("ch/1/flt/lc");
        assert_eq!(b.encode(Value::Int(5)).unwrap(), 1);
        assert_eq!(b.encode(Value::Float(0.4)).unwrap(), 0);
        assert_eq!(b.parse_text("ON").unwrap(), 1);
        assert_eq!(b.parse_text("nope"), Err(SetError::BadValue));
    }

    #[test]
    fn number_parsing() {
        assert_eq!(parse_number("2k00"), Some(2000.0));
        assert_eq!(parse_number("20k0"), Some(20000.0));
        assert_eq!(parse_number("1k5"), Some(1500.0));
        assert_eq!(parse_number("-oo"), Some(-144.0));
        assert_eq!(parse_number("+3.5"), Some(3.5));
        assert_eq!(parse_number("k"), None);
        assert_eq!(parse_time("1:02:03", false), Some(3723.0));
    }

    #[test]
    fn time_formats() {
        let mut s = String::new();
        format_float(&mut s, "%T", 35999.0).unwrap();
        assert_eq!(s, "9:59:59");
        s.clear();
        format_float(&mut s, "%T", 59.0).unwrap();
        assert_eq!(s, "0:59");
    }

    /// Display formatting must reproduce WING-Edit's own `describe` bounds for every float and
    /// int descriptor (tree and model descriptors).
    #[test]
    fn formatting_matches_wing_edit_describe() {
        let s = Schema::get();
        let mut checked = 0;
        for d in &s.descs {
            let Some(inner) = d
                .describe
                .split_once('[')
                .and_then(|(_, r)| r.split_once(']'))
                .map(|x| x.0)
            else {
                continue;
            };
            if d.fmt == "%t" {
                continue; // wlive card time fields: unit/wrap not known (describe shows 0:00:00 for both)
            }
            let Some((lo, hi)) = inner.split_once(" .. ") else {
                continue;
            };
            let (lo_raw, hi_raw) = match d.ty {
                ParamType::LinF | ParamType::LogF | ParamType::Fader => {
                    ((d.min as f32).to_bits(), (d.max as f32).to_bits())
                }
                ParamType::Int => ((d.min as i32) as u32, (d.max as i32) as u32),
                _ => continue,
            };
            let mut got_lo = String::new();
            d.format(lo_raw, &mut got_lo).unwrap();
            assert_eq!(got_lo, lo.trim(), "min of {d:?}");
            assert_eq!(fmt(d, hi_raw), hi.trim(), "max of {d:?}");
            if d.steps > 0 && d.ty != ParamType::Int {
                assert!(
                    d.describe.ends_with(&format!("], {} steps", d.steps)),
                    "{d:?}"
                );
            }
            checked += 1;
        }
        assert!(checked > 1000, "{checked}");
    }
}
