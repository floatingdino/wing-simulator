//! Parameter value <-> NRP value token conversions (wing-edit-session.md §2.4).
//!
//! Console -> client values use the encodings WING-Edit itself sends: `D5 f32` for every float
//! class (lin/log/fader and float enums, in engineering units), the compact integer forms for
//! ints/bools, the **enum token string** for enums, and string tokens for strings.
//!
//! Client -> console values are accepted in every form and converted as the console does
//! (protocol.md §3.1): numbers into enums by index, names into enums, text into numbers, numbers
//! into strings, `D6` normalised floats through the parameter's taper.

use std::borrow::Cow;

use wing_model::{NodeId, Origin, ParamStore, ParamType, SetError, Value};

use crate::nrp::Token;

/// Longest string a single value token can carry (`D1`).
const MAX_STR: usize = 256;

fn clip(b: &[u8]) -> &[u8] {
    &b[..b.len().min(MAX_STR)]
}

/// Call `f` with the value token for parameter `id` (`None` for containers).
pub fn with_value_token<R>(
    store: &ParamStore,
    id: NodeId,
    f: impl FnOnce(Token<'_>) -> R,
) -> Option<R> {
    let d = store.desc(id)?;
    let raw = store.get_raw(id);
    Some(match d.ty {
        ParamType::LinF | ParamType::LogF | ParamType::Fader => {
            f(Token::Float(f32::from_bits(raw)))
        }
        ParamType::FEnum => f(Token::Float(d.as_f64(raw) as f32)),
        ParamType::Int => f(Token::int(raw as i32)),
        ParamType::Enum => match d.enum_token(raw) {
            Some(t) => f(Token::Str(clip(t.as_bytes()))),
            // dynamic string lists (`ae::Cstringlist`) have no static items: report the index
            None => f(Token::int(raw as i32)),
        },
        ParamType::Str => return store.with_str(id, |s| f(Token::Str(clip(s.as_bytes())))),
    })
}

/// Apply a client value token to parameter `id`. Returns whether the stored value changed.
pub fn apply_value(
    store: &ParamStore,
    id: NodeId,
    tok: &Token<'_>,
    origin: Origin,
) -> Result<bool, SetError> {
    let d = store.desc(id).ok_or(SetError::NotParam)?;
    let is_str = d.ty == ParamType::Str;
    match *tok {
        Token::Str(b) => {
            let text: Cow<str> = String::from_utf8_lossy(b);
            match d.ty {
                ParamType::Str => store.set_str(id, &text, origin),
                ParamType::Enum | ParamType::FEnum => store
                    .set_enum_name(id, &text, origin)
                    .or_else(|_| store.set_text(id, &text, origin)),
                _ => store.set_text(id, &text, origin),
            }
        }
        Token::Float(v) if is_str => store.set_str(id, &v.to_string(), origin),
        Token::Float(v) => store.set_from(id, Value::Float(v), origin),
        Token::RawFloat(p) => store.set_norm(id, p, origin),
        Token::SmallInt(_) | Token::Int16(_) | Token::Int32(_) => {
            let n = tok.as_int().unwrap_or(0);
            if is_str {
                store.set_str(id, &n.to_string(), origin)
            } else {
                store.set_from(id, Value::Int(n), origin)
            }
        }
        _ => Err(SetError::TypeMismatch),
    }
}

/// `D9 n`: step the parameter by `n` positions (grid steps for stepped floats, ints and enums;
/// 1% of the travel for continuous parameters such as faders).
pub fn step_value(store: &ParamStore, id: NodeId, n: i8, origin: Origin) -> Result<bool, SetError> {
    let d = store.desc(id).ok_or(SetError::NotParam)?;
    if d.ty == ParamType::Str {
        return Err(SetError::TypeMismatch);
    }
    let norm = store.get_norm(id).unwrap_or(0.0);
    let pos = d.positions();
    let step = if pos >= 2 {
        1.0 / (pos - 1) as f32
    } else {
        0.01
    };
    store.set_norm(id, norm + n as f32 * step, origin)
}

/// `D8`: click the parameter: toggle two-position parameters, advance (wrapping) enums and ints.
pub fn click_value(store: &ParamStore, id: NodeId, origin: Origin) -> Result<bool, SetError> {
    let d = store.desc(id).ok_or(SetError::NotParam)?;
    if d.ty == ParamType::Str {
        return Err(SetError::TypeMismatch);
    }
    let norm = store.get_norm(id).unwrap_or(0.0);
    match d.positions() {
        2 => store.set_norm(id, 1.0 - norm.round(), origin),
        n if n > 2 => {
            let next = norm + 1.0 / (n - 1) as f32;
            store.set_norm(id, if next > 1.0 + 1e-6 { 0.0 } else { next }, origin)
        }
        _ => Ok(false),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tok_bytes(store: &ParamStore, path: &str) -> Vec<u8> {
        let id = store.lookup(path).unwrap();
        with_value_token(store, id, |t| {
            let mut v = Vec::new();
            t.encode(&mut v).unwrap();
            v
        })
        .unwrap()
    }

    #[test]
    fn encodings_follow_wing_edit() {
        let st = ParamStore::new();
        // fader -oo as D5 float
        assert_eq!(tok_bytes(&st, "ch/1/fdr"), [0xD5, 0xC3, 0x10, 0x00, 0x00]);
        // enum by token name
        let mdl = tok_bytes(&st, "ch/1/dyn/mdl");
        assert_eq!(&mdl[1..], b"COMP");
        assert_eq!(mdl[0], 0x7F + 4);
        // int/bool as small int
        assert_eq!(tok_bytes(&st, "ch/1/mute"), [0x00]);
        // empty string
        assert_eq!(tok_bytes(&st, "ch/1/name"), [0xD0]);
        assert!(with_value_token(&st, NodeId::ROOT, |_| ()).is_none());
    }

    #[test]
    fn apply_accepts_every_form() {
        let st = ParamStore::new();
        let id = |p| st.lookup(p).unwrap();
        assert_eq!(
            apply_value(&st, id("ch/1/fdr"), &Token::Float(-3.0), 9),
            Ok(true)
        );
        assert_eq!(st.get(id("ch/1/fdr")), Some(Value::Float(-3.0)));
        assert_eq!(
            apply_value(&st, id("ch/1/mute"), &Token::SmallInt(1), 9),
            Ok(true)
        );
        assert_eq!(
            apply_value(&st, id("ch/1/dyn/mdl"), &Token::Str(b"GATE"), 9),
            Ok(true)
        );
        assert_eq!(st.active_model(id("ch/1/dyn")).unwrap().name, "GATE");
        // enum by index
        let mdl = id("ch/1/dyn/mdl");
        let comp = st.desc(mdl).unwrap().enum_index("COMP").unwrap() as u8;
        assert_eq!(apply_value(&st, mdl, &Token::SmallInt(comp), 9), Ok(true));
        assert_eq!(st.active_model(id("ch/1/dyn")).unwrap().name, "COMP");
        assert_eq!(
            apply_value(&st, id("ch/1/name"), &Token::Str(b"Kick"), 9),
            Ok(true)
        );
        assert_eq!(
            apply_value(&st, id("ch/1/name"), &Token::SmallInt(7), 9),
            Ok(true)
        );
        assert_eq!(
            st.with_str(id("ch/1/name"), |s| s.to_string()).unwrap(),
            "7"
        );
        // text into a float
        assert_eq!(
            apply_value(&st, id("ch/2/fdr"), &Token::Str(b"-10"), 9),
            Ok(true)
        );
        assert_eq!(st.get(id("ch/2/fdr")), Some(Value::Float(-10.0)));
        // normalised
        assert_eq!(
            apply_value(&st, id("ch/2/fdr"), &Token::RawFloat(0.75), 9),
            Ok(true)
        );
        assert_eq!(st.get(id("ch/2/fdr")), Some(Value::Float(0.0)));
        assert_eq!(
            apply_value(&st, NodeId::ROOT, &Token::SmallInt(1), 9),
            Err(SetError::NotParam)
        );
    }

    #[test]
    fn click_and_step() {
        let st = ParamStore::new();
        let mute = st.lookup("ch/4/mute").unwrap();
        click_value(&st, mute, 1).unwrap();
        assert_eq!(st.get(mute), Some(Value::Int(1)));
        click_value(&st, mute, 1).unwrap();
        assert_eq!(st.get(mute), Some(Value::Int(0)));
        let fdr = st.lookup("ch/4/fdr").unwrap();
        st.set(fdr, Value::Float(0.0)).unwrap();
        step_value(&st, fdr, -10, 1).unwrap();
        assert_eq!(st.get(fdr), Some(Value::Float(-4.0)));
    }
}
