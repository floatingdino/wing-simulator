//! Minimal order-preserving JSON reader/writer for `.snap` files.
//!
//! Numbers keep their source text so keys the schema does not know can be written back verbatim.
//! Objects keep key order (WING snapshots are ordered by tree position). No dependency needed.

use std::fmt::{self, Write};

/// A JSON value. Objects are ordered `(key, value)` lists.
#[derive(Clone, Debug, PartialEq)]
pub enum Json {
    Null,
    Bool(bool),
    /// Number as written in the source (or as formatted by the writer).
    Num(String),
    Str(String),
    Arr(Vec<Json>),
    Obj(Vec<(String, Json)>),
}

/// Parse error with byte offset.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct JsonError {
    pub offset: usize,
    pub msg: &'static str,
}

impl fmt::Display for JsonError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "JSON error at byte {}: {}", self.offset, self.msg)
    }
}

impl std::error::Error for JsonError {}

impl Json {
    /// Parse a complete JSON document.
    pub fn parse(text: &str) -> Result<Json, JsonError> {
        let mut p = Parser {
            b: text.as_bytes(),
            s: text,
            i: 0,
        };
        p.ws();
        let v = p.value(0)?;
        p.ws();
        if p.i != p.b.len() {
            return Err(p.err("trailing characters"));
        }
        Ok(v)
    }

    /// Member of an object.
    pub fn get(&self, key: &str) -> Option<&Json> {
        match self {
            Json::Obj(m) => m.iter().find(|(k, _)| k == key).map(|(_, v)| v),
            _ => None,
        }
    }

    /// Numeric value of a `Num`.
    pub fn as_f64(&self) -> Option<f64> {
        match self {
            Json::Num(s) => s.parse().ok(),
            _ => None,
        }
    }

    pub fn as_str(&self) -> Option<&str> {
        match self {
            Json::Str(s) => Some(s),
            _ => None,
        }
    }

    /// Compact serialisation (the console writes snapshots on one line).
    pub fn write<W: Write>(&self, w: &mut W) -> fmt::Result {
        match self {
            Json::Null => w.write_str("null"),
            Json::Bool(b) => w.write_str(if *b { "true" } else { "false" }),
            Json::Num(n) => w.write_str(n),
            Json::Str(s) => write_str(w, s),
            Json::Arr(a) => {
                w.write_char('[')?;
                for (i, v) in a.iter().enumerate() {
                    if i > 0 {
                        w.write_char(',')?;
                    }
                    v.write(w)?;
                }
                w.write_char(']')
            }
            Json::Obj(m) => {
                w.write_char('{')?;
                for (i, (k, v)) in m.iter().enumerate() {
                    if i > 0 {
                        w.write_char(',')?;
                    }
                    write_str(w, k)?;
                    w.write_char(':')?;
                    v.write(w)?;
                }
                w.write_char('}')
            }
        }
    }

    /// Compact text.
    pub fn to_text(&self) -> String {
        let mut s = String::new();
        self.write(&mut s).expect("write to String");
        s
    }
}

/// Write a JSON string literal.
pub fn write_str<W: Write>(w: &mut W, s: &str) -> fmt::Result {
    w.write_char('"')?;
    for c in s.chars() {
        match c {
            '"' => w.write_str("\\\"")?,
            '\\' => w.write_str("\\\\")?,
            '\n' => w.write_str("\\n")?,
            '\r' => w.write_str("\\r")?,
            '\t' => w.write_str("\\t")?,
            c if (c as u32) < 0x20 => write!(w, "\\u{:04x}", c as u32)?,
            c => w.write_char(c)?,
        }
    }
    w.write_char('"')
}

struct Parser<'a> {
    b: &'a [u8],
    s: &'a str,
    i: usize,
}

const MAX_DEPTH: usize = 128;

impl Parser<'_> {
    fn err(&self, msg: &'static str) -> JsonError {
        JsonError {
            offset: self.i,
            msg,
        }
    }

    fn ws(&mut self) {
        while self.i < self.b.len() && matches!(self.b[self.i], b' ' | b'\t' | b'\n' | b'\r') {
            self.i += 1;
        }
    }

    fn lit(&mut self, word: &str, v: Json) -> Result<Json, JsonError> {
        if self.b[self.i..].starts_with(word.as_bytes()) {
            self.i += word.len();
            Ok(v)
        } else {
            Err(self.err("invalid literal"))
        }
    }

    fn value(&mut self, depth: usize) -> Result<Json, JsonError> {
        if depth > MAX_DEPTH {
            return Err(self.err("nesting too deep"));
        }
        match self.b.get(self.i) {
            None => Err(self.err("unexpected end")),
            Some(b'{') => {
                self.i += 1;
                let mut m = Vec::new();
                self.ws();
                if self.b.get(self.i) == Some(&b'}') {
                    self.i += 1;
                    return Ok(Json::Obj(m));
                }
                loop {
                    self.ws();
                    if self.b.get(self.i) != Some(&b'"') {
                        return Err(self.err("expected key"));
                    }
                    let k = self.string()?;
                    self.ws();
                    if self.b.get(self.i) != Some(&b':') {
                        return Err(self.err("expected ':'"));
                    }
                    self.i += 1;
                    self.ws();
                    let v = self.value(depth + 1)?;
                    m.push((k, v));
                    self.ws();
                    match self.b.get(self.i) {
                        Some(b',') => self.i += 1,
                        Some(b'}') => {
                            self.i += 1;
                            return Ok(Json::Obj(m));
                        }
                        _ => return Err(self.err("expected ',' or '}'")),
                    }
                }
            }
            Some(b'[') => {
                self.i += 1;
                let mut a = Vec::new();
                self.ws();
                if self.b.get(self.i) == Some(&b']') {
                    self.i += 1;
                    return Ok(Json::Arr(a));
                }
                loop {
                    self.ws();
                    a.push(self.value(depth + 1)?);
                    self.ws();
                    match self.b.get(self.i) {
                        Some(b',') => self.i += 1,
                        Some(b']') => {
                            self.i += 1;
                            return Ok(Json::Arr(a));
                        }
                        _ => return Err(self.err("expected ',' or ']'")),
                    }
                }
            }
            Some(b'"') => Ok(Json::Str(self.string()?)),
            Some(b't') => self.lit("true", Json::Bool(true)),
            Some(b'f') => self.lit("false", Json::Bool(false)),
            Some(b'n') => self.lit("null", Json::Null),
            Some(b'-' | b'0'..=b'9') => {
                let start = self.i;
                if self.b[self.i] == b'-' {
                    self.i += 1;
                }
                while self.i < self.b.len()
                    && matches!(
                        self.b[self.i],
                        b'0'..=b'9' | b'.' | b'e' | b'E' | b'+' | b'-'
                    )
                {
                    self.i += 1;
                }
                let t = &self.s[start..self.i];
                if t.parse::<f64>().is_err() {
                    self.i = start;
                    return Err(self.err("invalid number"));
                }
                Ok(Json::Num(t.to_string()))
            }
            Some(_) => Err(self.err("unexpected character")),
        }
    }

    fn hex4(&mut self) -> Result<u32, JsonError> {
        let h = self
            .s
            .get(self.i..self.i + 4)
            .and_then(|h| u32::from_str_radix(h, 16).ok())
            .ok_or_else(|| self.err("bad \\u escape"))?;
        self.i += 4;
        Ok(h)
    }

    fn string(&mut self) -> Result<String, JsonError> {
        self.i += 1; // opening quote
        let mut out = String::new();
        loop {
            let start = self.i;
            while self.i < self.b.len() && !matches!(self.b[self.i], b'"' | b'\\') {
                self.i += 1;
            }
            out.push_str(&self.s[start..self.i]);
            match self.b.get(self.i) {
                None => return Err(self.err("unterminated string")),
                Some(b'"') => {
                    self.i += 1;
                    return Ok(out);
                }
                _ => {
                    self.i += 1;
                    let c = *self.b.get(self.i).ok_or_else(|| self.err("bad escape"))?;
                    self.i += 1;
                    match c {
                        b'"' => out.push('"'),
                        b'\\' => out.push('\\'),
                        b'/' => out.push('/'),
                        b'b' => out.push('\u{8}'),
                        b'f' => out.push('\u{c}'),
                        b'n' => out.push('\n'),
                        b'r' => out.push('\r'),
                        b't' => out.push('\t'),
                        b'u' => {
                            let mut cp = self.hex4()?;
                            if (0xD800..0xDC00).contains(&cp)
                                && self.b[self.i..].starts_with(b"\\u")
                            {
                                self.i += 2;
                                let lo = self.hex4()?;
                                cp = 0x10000
                                    + ((cp - 0xD800) << 10)
                                    + (lo.wrapping_sub(0xDC00) & 0x3FF);
                            }
                            out.push(char::from_u32(cp).unwrap_or('\u{FFFD}'));
                        }
                        _ => return Err(self.err("bad escape")),
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trips_text_and_order() {
        let src =
            r#"{"b":1.230000019,"a":[true,false,null],"s":"x\"\\é\n","n":-144,"e":1e-05,"o":{}}"#;
        let v = Json::parse(src).unwrap();
        assert_eq!(v.get("b"), Some(&Json::Num("1.230000019".into())));
        assert_eq!(v.get("s").unwrap().as_str(), Some("x\"\\\u{e9}\n"));
        assert_eq!(v.get("e").unwrap().as_f64(), Some(1e-5));
        let out = v.to_text();
        assert_eq!(Json::parse(&out).unwrap(), v);
        assert!(out.starts_with(r#"{"b":1.230000019,"a":"#));
    }

    #[test]
    fn rejects_garbage() {
        for bad in [
            "",
            "{",
            "{\"a\" 1}",
            "[1,]",
            "tru",
            "{\"a\":1} x",
            "\"abc",
            "-",
        ] {
            assert!(Json::parse(bad).is_err(), "{bad}");
        }
        assert_eq!(
            Json::parse("\"\\ud83c\\udfb9\"").unwrap(),
            Json::Str("\u{1F3B9}".into())
        );
    }
}
