//! Node definition records served for `DD` requests, built from the wing-model schema plus the
//! store's active (model-dependent) descriptors, and the console-local "live" values the server
//! answers itself (`$`-prefixed dynamic lists and the console model).

use wing_model::{NodeId, ParamClass, ParamStore, ParamType};

use crate::nrp::nodedef::{EnumItem, FloatEnumItem, NodeDef, NodeKind, Unit};

/// Console-local values that are not ordinary stored parameters.
#[derive(Clone, Debug)]
pub struct LiveInfo {
    /// Reply to `$syscfg/$cnsmdl = "ModelRequest"`. WING-Edit goes offline unless this is
    /// `wing-fullsize`, `wing-rack` or `wing-compact` (wing-edit-schema.md §6).
    pub console_model: String,
    /// Items of `play/$songs` (USB player song list). [H] empty = no USB stick.
    pub songs: Vec<String>,
    /// Items of `$ctl/lib/$scenes` (show/scene library list). [H]
    pub scenes: Vec<String>,
}

impl Default for LiveInfo {
    fn default() -> Self {
        LiveInfo {
            console_model: "wing-fullsize".into(),
            songs: Vec::new(),
            scenes: Vec::new(),
        }
    }
}

impl LiveInfo {
    /// The `$cnsmdl` model string for a discovery model id (`ngc-full`/`wing-bk` = full size).
    pub fn console_model_for(discovery_model: &str) -> &'static str {
        match discovery_model {
            "wing-rack" => "wing-rack",
            "wing-compact" => "wing-compact",
            _ => "wing-fullsize",
        }
    }

    /// The items of a dynamic string list parameter, by path.
    pub fn list_for(&self, path: &str) -> &[String] {
        match path {
            "play/$songs" => &self.songs,
            "$ctl/lib/$scenes" => &self.scenes,
            _ => &[],
        }
    }
}

/// Whether a node is hidden from definition listings and dumps: model slots the active model
/// leaves unused (WING-Edit skips n/a parameters too).
pub fn hidden(store: &ParamStore, id: NodeId) -> bool {
    store.schema().is_param(id) && store.is_inactive(id)
}

/// The definition record of node `id` as the console would describe it now.
///
/// Unverified details [H]: numbered children (`ch/1`) get an empty name and their number as the
/// index (protocol.md §3.3); model slots are named after the active model's parameter (`thr`);
/// the read-only bit is set for `$`-prefixed parameters (the schema has no per-parameter
/// read-only flag, T009); the unit code is the WING-Edit `Cnode` unit nibble.
pub fn node_def(store: &ParamStore, id: NodeId, live: &LiveInfo) -> NodeDef {
    let s = store.schema();
    let parent = match s.parent(id) {
        Some(NodeId::ROOT) | None => 0,
        Some(p) => s.hash(p),
    };
    let name = store.display_name(id);
    let (index, name) = match name.parse::<u16>() {
        Ok(n) if n >= 1 && name.bytes().all(|b| b.is_ascii_digit()) => (n, ""),
        _ => (0, name),
    };
    let Some(d) = store.desc(id) else {
        return NodeDef {
            parent,
            hash: s.hash(id),
            index,
            name: name.into(),
            long_name: s.long_name(id).into(),
            unit: Unit::None,
            read_only: false,
            extra_flags: 0,
            kind: NodeKind::Node,
        };
    };
    let long = if d.model_param && !d.long.is_empty() {
        d.long
    } else {
        s.long_name(id)
    };
    let kind = match d.ty {
        ParamType::LinF => NodeKind::LinFloat {
            min: d.min as f32,
            max: d.max as f32,
            steps: d.steps as i32,
        },
        ParamType::LogF => NodeKind::LogFloat {
            min: d.min as f32,
            max: d.max as f32,
            steps: d.steps as i32,
        },
        ParamType::Fader => NodeKind::FaderLevel,
        ParamType::Int => NodeKind::Int {
            min: d.min as i32,
            max: d.max as i32,
        },
        ParamType::Enum if d.class == ParamClass::Stringlist || d.enum_items().is_empty() => {
            let mut path = String::new();
            let _ = s.write_path(id, &mut path);
            NodeKind::StringEnum(
                live.list_for(&path)
                    .iter()
                    .map(|i| EnumItem {
                        name: i.clone(),
                        long_name: i.clone(),
                    })
                    .collect(),
            )
        }
        ParamType::Enum => NodeKind::StringEnum(
            d.enum_items()
                .iter()
                .map(|e| EnumItem {
                    name: e.token.into(),
                    long_name: e.long.into(),
                })
                .collect(),
        ),
        ParamType::FEnum => NodeKind::FloatEnum(
            d.enum_items()
                .iter()
                .map(|e| FloatEnumItem {
                    value: e.value as f32,
                    long_name: e.long.into(),
                })
                .collect(),
        ),
        ParamType::Str => NodeKind::String {
            max_len: d.maxlen.min(u16::MAX as u32) as u16,
        },
    };
    NodeDef {
        parent,
        hash: s.hash(id),
        index,
        name: name.into(),
        long_name: long.into(),
        unit: Unit::from_bits(((d.flags >> 16) & 0xF) as u8),
        read_only: s.name(id).starts_with('$'),
        extra_flags: 0,
        kind,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defs_for_nodes_and_params() {
        let st = ParamStore::new();
        let live = LiveInfo::default();
        let s = st.schema();
        let ch1 = s.by_path("ch/1").unwrap();
        let d = node_def(&st, ch1, &live);
        assert_eq!((d.index, d.name.as_str()), (1, ""));
        assert_eq!(d.parent, s.hash(s.by_path("ch").unwrap()));
        assert_eq!(d.kind, NodeKind::Node);
        let stat = node_def(&st, s.by_path("$stat").unwrap(), &live);
        assert_eq!(
            (stat.parent, stat.hash, stat.name.as_str()),
            (0, 0x97a0_0439, "$stat")
        );
        let fdr = node_def(&st, s.by_path("ch/1/fdr").unwrap(), &live);
        assert_eq!(fdr.kind, NodeKind::FaderLevel);
        assert_eq!(fdr.unit, Unit::Db);
        let thr = node_def(&st, s.by_path("ch/1/dyn/1").unwrap(), &live);
        assert_eq!(thr.name, "thr");
        assert!(matches!(thr.kind, NodeKind::LinFloat { .. }));
        let mdl = node_def(&st, s.by_path("ch/1/dyn/mdl").unwrap(), &live);
        let NodeKind::StringEnum(items) = &mdl.kind else {
            panic!("{mdl:?}")
        };
        assert!(items.iter().any(|i| i.name == "COMP"));
        let scenes = LiveInfo {
            scenes: vec!["A".into(), "B".into()],
            ..LiveInfo::default()
        };
        let sc = node_def(&st, s.by_path("$ctl/lib/$scenes").unwrap(), &scenes);
        assert!(sc.read_only);
        assert_eq!(
            sc.kind,
            NodeKind::StringEnum(vec![
                EnumItem {
                    name: "A".into(),
                    long_name: "A".into()
                },
                EnumItem {
                    name: "B".into(),
                    long_name: "B".into()
                }
            ])
        );
        // every record encodes and parses back
        for id in s.ids().skip(1).step_by(97) {
            let d = node_def(&st, id, &live);
            assert_eq!(NodeDef::parse(&d.to_body()).unwrap(), d, "{}", s.path(id));
        }
    }
}
