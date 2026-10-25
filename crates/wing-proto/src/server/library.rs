//! Show/scene library hooks for the `$ctl/lib` subtree (T017).
//!
//! The WING-Edit 3.3.4 schema has these `$ctl/lib` leaves (all `$`, console-owned):
//!
//! | leaf | type | meaning (inferred) |
//! |---|---|---|
//! | `$scenes` | dynamic string list | scene names of the active show, in show order |
//! | `$actshow` | string | ACTIVE SHOW name (WING-Edit says "PLEASE OPEN SHOWFILE ON CONNECTED MIXER" when empty [H]) |
//! | `$active` | string | ACTIVE SCENE name |
//! | `$activeid` | int 0..128 | index of the active scene [H: 0-based position in `$scenes`] |
//! | `$actidx` | int 0..0 | unknown; left alone |
//! | `$actionidx` | int 0..16384 | argument of `$action` (client-writable) [H: 0-based position in `$scenes`] |
//! | `$action` | enum IDLE GOPREV GONEXT GO PREV NEXT GOTAG | library command (client-writable) |
//!
//! The schema has no store/rename/delete command: on the console those are done on the surface,
//! and WING-Edit edits its own (PC-side) library. The simulator accepts three **simulator
//! extensions** as `$action` string values so a client can manage the library remotely:
//! `STORE[:name]`, `RENAME:name` and `DELETE` (the latter two act on `$actionidx`).
//!
//! When a [`SceneLibrary`] is configured ([`super::LiveInfo::library`]), the native session
//! routes every `$ctl/lib/$action` set to it instead of the store, and drops client sets of the
//! other console-owned `$ctl/lib/$*` leaves (WING-Edit's connect push includes `$actshow`).
//! None of these semantics are verified against a console [H].

use std::sync::OnceLock;

use wing_model::{NodeId, Origin, ParamStore, ParamType};

use crate::nrp::Token;

/// The daemon's show/scene library behind `$ctl/lib`.
pub trait SceneLibrary: Send + Sync {
    /// Items of `$ctl/lib/$scenes` (scene names in show order).
    fn scenes(&self) -> Vec<String>;
    /// A `$ctl/lib/$action` request: `action` is the enum token (`GO`, `NEXT`, ...) or a simulator
    /// extension (`STORE:Verse`), `index` the current `$ctl/lib/$actionidx`. Must not block on
    /// disk IO (the caller is a client connection thread): queue the work.
    fn action(&self, action: &str, index: i32, origin: Origin);
}

/// Node ids of the `$ctl/lib` leaves the session treats specially.
pub(crate) struct LibIds {
    pub lib: NodeId,
    pub action: NodeId,
    pub actionidx: NodeId,
}

pub(crate) fn lib_ids(store: &ParamStore) -> Option<&'static LibIds> {
    static IDS: OnceLock<Option<LibIds>> = OnceLock::new();
    IDS.get_or_init(|| {
        let s = store.schema();
        Some(LibIds {
            lib: s.by_path("$ctl/lib")?,
            action: s.by_path("$ctl/lib/$action")?,
            actionidx: s.by_path("$ctl/lib/$actionidx")?,
        })
    })
    .as_ref()
}

/// The action text of a `$action` value token: strings as sent, numbers as the enum token at
/// that index.
pub(crate) fn action_text(store: &ParamStore, id: NodeId, tok: &Token<'_>) -> Option<String> {
    match *tok {
        Token::Str(b) => Some(String::from_utf8_lossy(b).trim().to_string()),
        Token::SmallInt(_) | Token::Int16(_) | Token::Int32(_) | Token::Float(_) => {
            let n = match *tok {
                Token::Float(f) => f as i32,
                _ => tok.as_int()?,
            };
            let d = store.desc(id)?;
            if d.ty != ParamType::Enum {
                return None;
            }
            d.enum_items()
                .get(usize::try_from(n).ok()?)
                .map(|e| e.token.to_string())
        }
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ids_and_action_text() {
        let st = ParamStore::new();
        let ids = lib_ids(&st).unwrap();
        assert_eq!(st.schema().parent(ids.action), Some(ids.lib));
        assert_eq!(
            action_text(&st, ids.action, &Token::Str(b"STORE:Verse")).as_deref(),
            Some("STORE:Verse")
        );
        // enum by index: IDLE GOPREV GONEXT GO ...
        assert_eq!(
            action_text(&st, ids.action, &Token::SmallInt(3)).as_deref(),
            Some("GO")
        );
        assert_eq!(action_text(&st, ids.action, &Token::SmallInt(60)), None);
    }
}
