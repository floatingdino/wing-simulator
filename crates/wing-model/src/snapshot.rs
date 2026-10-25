//! WING scene snapshots (`.snap`): load into a [`ParamStore`] and save back.
//!
//! A `.snap` file is one JSON object: header fields (`type`, `creator_*`, `created`, `active_show`,
//! `active_scene`) and four data sections that map onto the parameter tree (T003
//! `snapshot_sections`): `ae_data` -> `/`, `ce_data` -> `$ctl`, `ae_globals` -> `$globals`,
//! `ce_globals` -> `$ctl/$globals`. Leaves hold engineering values (numbers), enum tokens
//! (strings), booleans and strings; model-dependent blocks store their parameters under the active
//! model's names (`ch/1/gate/thr`), so model selectors are applied before everything else.
//!
//! Loading ([`Snapshot::load`]) resets the store to the factory scene (a scene recall is complete),
//! migrates firmware 1.x keys to the 3.x schema (see `migrate.rs` and
//! docs/research/snapshot-migration.md), applies selectors, then all other values. Keys the schema
//! does not know are kept verbatim in a side table and written back on save; the header is kept
//! verbatim for 3.x files. [`LoadReport`] lists what was migrated, dropped, preserved or rejected.
//!
//! Saving ([`Snapshot::save`]) writes the 3.x layout: every parameter the firmware 3.0.6 factory
//! scene persists (plus any other tree parameter the loaded file contained), active model slots
//! under their model names, in schema order, compact like the console. Floats are written as the
//! shortest text that round-trips the stored `f32` (the console prints ~10 significant digits; the
//! value is identical).

use std::collections::HashMap;
use std::fmt;
use std::sync::OnceLock;

use crate::json::{Json, JsonError};
use crate::migrate;
use crate::schema::{NodeId, ParamClass, ParamType, Schema, NONE};
use crate::store::{Origin, ParamStore};
use crate::value::{SetError, Value};

/// Origin tag for changes made by a snapshot load.
pub const ORIGIN_SNAPSHOT: Origin = 0x5E_0000;

/// Snapshot section -> tree prefix.
pub const SECTIONS: [(&str, &str); 4] = [
    ("ae_data", ""),
    ("ce_data", "$ctl"),
    ("ae_globals", "$globals"),
    ("ce_globals", "$ctl/$globals"),
];

/// The firmware 3.0.6 factory scene (T008), the reference for the 3.x file layout.
pub static FACTORY_SCENE: &str =
    include_str!("../../../data/firmware/json/0006211b-snapshot_11.json");

/// Keys present in real 3.x snapshots that the WING-Edit tree does not model (GUI preferences
/// owned by the control engine, wing-edit-schema.md §5). Kept verbatim, not reported as unmapped.
pub const KNOWN_UNMODELLED: [&str; 3] = [
    "$ctl/$globals/usewheel",
    "$ctl/$globals/timefmt",
    "$ctl/$globals/datefmt",
];

/// Why a snapshot could not be loaded.
#[derive(Debug, Clone, PartialEq)]
pub enum SnapError {
    Json(JsonError),
    /// The document is not a JSON object or has no `ae_data` section.
    NotASnapshot,
}

impl fmt::Display for SnapError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SnapError::Json(e) => e.fmt(f),
            SnapError::NotASnapshot => f.write_str("not a WING snapshot (no ae_data object)"),
        }
    }
}

impl std::error::Error for SnapError {}

impl From<JsonError> for SnapError {
    fn from(e: JsonError) -> Self {
        SnapError::Json(e)
    }
}

/// What happened to one legacy key during migration.
#[derive(Clone, Debug, PartialEq)]
pub struct MigrationNote {
    /// Source key (tree path, e.g. `main/1/dly/m`).
    pub from: String,
    /// Destination key(s) in the 3.x tree; empty when the key was dropped.
    pub to: String,
    /// Rule id (see docs/research/snapshot-migration.md).
    pub rule: &'static str,
    /// The source value differed from what 3.x can represent (information lost).
    pub lossy: bool,
}

/// Result of [`Snapshot::load`].
#[derive(Clone, Debug, Default)]
pub struct LoadReport {
    /// `type` header field (`snapshot.8`, `snapshot.11`).
    pub format: String,
    /// `creator_fw` header field.
    pub creator_fw: String,
    /// Firmware 1.x/2.x file: migrated to the 3.x schema.
    pub legacy: bool,
    /// Leaf keys in the file.
    pub keys: usize,
    /// Values applied to the store (including migrated ones).
    pub applied: usize,
    /// Legacy keys renamed/converted or dropped by a documented rule.
    pub migrated: Vec<MigrationNote>,
    /// Known non-tree keys kept verbatim ([`KNOWN_UNMODELLED`]).
    pub preserved: Vec<String>,
    /// Unexplained keys the schema does not know (kept verbatim and written back).
    pub unmapped: Vec<String>,
    /// Keys that resolved to a parameter but whose value was refused.
    pub rejected: Vec<(String, SetError)>,
    /// Numeric values that were clamped or moved to the parameter's step grid:
    /// `(key, file value, stored value)` (e.g. 1.x values under a changed 3.x law).
    pub adjusted: Vec<(String, f64, f64)>,
}

impl LoadReport {
    /// Migration notes that lost information.
    pub fn lossy(&self) -> impl Iterator<Item = &MigrationNote> {
        self.migrated.iter().filter(|n| n.lossy)
    }
}

/// Result of [`Snapshot::apply_diff`].
#[derive(Clone, Debug, Default)]
pub struct ApplyReport {
    /// Parameters whose stored value changed.
    pub changed: usize,
    /// Parameters the store refused.
    pub rejected: Vec<(String, SetError)>,
}

/// One flattened snapshot leaf during loading.
#[derive(Clone, Debug)]
pub(crate) struct Leaf {
    /// Tree path (`ch/1/flt/lcf`, `$ctl/cfg/...`).
    pub path: String,
    /// Original JSON key path, `/`-joined, starting with the section (`ae_data/ch/1/flt/lcf`).
    pub src: String,
    pub value: Json,
}

/// Snapshot document state that is not held by the [`ParamStore`]: header fields, unknown keys
/// and which extra tree parameters the source persisted.
#[derive(Clone, Debug)]
pub struct Snapshot {
    /// Top-level header fields in file order (everything except the four sections).
    pub header: Vec<(String, Json)>,
    /// Keys the schema does not know: JSON key path (section first) and value, kept verbatim.
    pub extras: Vec<(Vec<String>, Json)>,
    /// Tree parameters present in the source that the factory scene does not persist (by node).
    extra_nodes: Vec<bool>,
}

struct Factory {
    /// Node persisted by the factory scene (tree params and selectors; model slots resolve via
    /// their block's selector).
    mask: Vec<bool>,
    header: Vec<(String, Json)>,
    /// Non-tree content of the factory scene: empty container objects (`ce_data/lib`) and the
    /// [`KNOWN_UNMODELLED`] keys. Written by new scenes and added to migrated 1.x scenes.
    empties: Vec<(Vec<String>, Json)>,
}

fn factory() -> &'static Factory {
    static F: OnceLock<Factory> = OnceLock::new();
    F.get_or_init(|| {
        let doc = Json::parse(FACTORY_SCENE).expect("factory scene is valid JSON");
        let store = ParamStore::new();
        let mut mask = vec![false; store.schema().node_count()];
        let mut empties = Vec::new();
        for leaf in flatten(&doc) {
            if matches!(&leaf.value, Json::Obj(m) if m.is_empty())
                || KNOWN_UNMODELLED.contains(&leaf.path.as_str())
            {
                empties.push((src_path(&leaf.src), leaf.value.clone()));
            }
            if let Some(id) = store
                .lookup(&leaf.path)
                .filter(|&id| store.schema().is_param(id))
            {
                mask[id.index()] = true;
            }
        }
        Factory {
            mask,
            header: header_of(&doc),
            empties,
        }
    })
}

fn header_of(doc: &Json) -> Vec<(String, Json)> {
    match doc {
        Json::Obj(m) => m
            .iter()
            .filter(|(k, _)| !SECTIONS.iter().any(|(s, _)| s == k))
            .cloned()
            .collect(),
        _ => Vec::new(),
    }
}

/// Flatten the four data sections into leaves (non-object values and empty objects).
pub(crate) fn flatten(doc: &Json) -> Vec<Leaf> {
    fn rec(v: &Json, tree: &mut String, src: &mut String, out: &mut Vec<Leaf>) {
        match v {
            Json::Obj(m) if !m.is_empty() => {
                for (k, c) in m {
                    let (tl, sl) = (tree.len(), src.len());
                    if !tree.is_empty() {
                        tree.push('/');
                    }
                    tree.push_str(k);
                    src.push('/');
                    src.push_str(k);
                    rec(c, tree, src, out);
                    tree.truncate(tl);
                    src.truncate(sl);
                }
            }
            _ => out.push(Leaf {
                path: tree.clone(),
                src: src.clone(),
                value: v.clone(),
            }),
        }
    }
    let mut out = Vec::new();
    if let Json::Obj(m) = doc {
        for (k, v) in m {
            if let Some((sect, root)) = SECTIONS.iter().find(|(s, _)| s == k) {
                let mut tree = root.to_string();
                let mut src = sect.to_string();
                rec(v, &mut tree, &mut src, &mut out);
            }
        }
    }
    out
}

/// Parse the leading major version from `creator_fw` (`1.13.9-0-g...` -> 1).
pub(crate) fn fw_major(fw: &str) -> Option<u32> {
    fw.split(['.', '-']).next()?.parse().ok()
}

/// Apply one JSON value to a parameter.
fn apply(store: &ParamStore, id: NodeId, v: &Json, origin: Origin) -> Result<bool, SetError> {
    let d = store.desc(id).ok_or(SetError::NotParam)?;
    match v {
        Json::Str(s) => store.set_text(id, s, origin),
        Json::Bool(b) => match d.ty {
            ParamType::Int => store.set_from(id, Value::Int(*b as i32), origin),
            ParamType::Enum => store.set_from(id, Value::Enum(*b as u32), origin),
            ParamType::Str => store.set_str(id, if *b { "true" } else { "false" }, origin),
            _ => store.set_from(id, Value::Float(*b as i32 as f32), origin),
        },
        Json::Num(n) => {
            let f = v.as_f64().ok_or(SetError::BadValue)?;
            match d.ty {
                ParamType::LinF | ParamType::LogF | ParamType::Fader => {
                    // Keep the console's own f32 when it is on our step grid at f32 precision
                    // (re-quantising in f64 can move it by an ulp); otherwise clamp/quantise.
                    let x = f as f32;
                    let q = d.quantize(f);
                    let tol = 4.0 * f32::EPSILON * q.abs().max(1.0);
                    let v = if (q - x).abs() <= tol { x } else { q };
                    store.set_raw(id, v.to_bits(), origin)
                }
                ParamType::FEnum => store.set_from(id, Value::Float(f as f32), origin),
                ParamType::Int => store.set_from(id, Value::Int(f.round() as i32), origin),
                ParamType::Enum | ParamType::Str => store.set_text(id, n, origin),
            }
        }
        _ => Err(SetError::TypeMismatch),
    }
}

/// `(file value, stored value)` when a numeric file value was clamped/quantised/rounded.
fn adjustment(store: &ParamStore, id: NodeId, v: &Json) -> Option<(f64, f64)> {
    let f = v.as_f64()?;
    let d = store.desc(id)?;
    if matches!(d.ty, ParamType::Enum | ParamType::Str) {
        return None;
    }
    let got = store.get_f64(id)?;
    let tol = 4.0 * f32::EPSILON as f64 * f.abs().max(1.0);
    ((got - f).abs() > tol).then_some((f, got))
}

impl Default for Snapshot {
    fn default() -> Self {
        Self::factory()
    }
}

impl Snapshot {
    /// An empty document with the firmware 3.0.6 factory scene header (save a store as a new
    /// scene).
    pub fn factory() -> Snapshot {
        Snapshot {
            header: factory().header.clone(),
            extras: factory().empties.clone(),
            extra_nodes: vec![false; Schema::get().node_count()],
        }
    }

    /// Header field by name.
    pub fn header_str(&self, key: &str) -> Option<&str> {
        self.header
            .iter()
            .find(|(k, _)| k == key)
            .and_then(|(_, v)| v.as_str())
    }

    /// Load a `.snap` text into `store` (reset to factory defaults first). Legacy (fw < 3)
    /// files are migrated; the returned [`Snapshot`] then carries a 3.x header.
    pub fn load(store: &ParamStore, text: &str) -> Result<(Snapshot, LoadReport), SnapError> {
        let doc = Json::parse(text)?;
        if !matches!(doc.get("ae_data"), Some(Json::Obj(_))) {
            return Err(SnapError::NotASnapshot);
        }
        let s = store.schema();
        let mut header = header_of(&doc);
        let format = doc
            .get("type")
            .and_then(Json::as_str)
            .unwrap_or("")
            .to_string();
        let creator_fw = doc
            .get("creator_fw")
            .and_then(Json::as_str)
            .unwrap_or("")
            .to_string();
        let type_ver: Option<u32> = format
            .strip_prefix("snapshot.")
            .and_then(|v| v.parse().ok());
        let legacy = match fw_major(&creator_fw) {
            Some(m) => m < 3,
            None => type_ver.is_some_and(|v| v < 11),
        };
        let mut leaves = flatten(&doc);
        let mut report = LoadReport {
            format,
            creator_fw,
            legacy,
            keys: leaves.len(),
            ..Default::default()
        };
        if legacy {
            leaves = migrate::migrate_1x(leaves, &mut report.migrated);
            header = migrate::migrate_header(&header, &factory().header);
        }

        store.reset_to_defaults();
        let fmask = &factory().mask;
        let mut extras = Vec::new();
        let mut extra_nodes = vec![false; s.node_count()];
        let mut done = vec![false; leaves.len()];
        // Pass 1: model selectors (tree nodes), so model-dependent names resolve in pass 2.
        // Repeat while selectors keep resolving (a selector can sit inside another model's slots).
        loop {
            let mut progress = false;
            for (i, leaf) in leaves.iter().enumerate() {
                if done[i] {
                    continue;
                }
                let Some(id) = store.lookup(&leaf.path) else {
                    continue;
                };
                if s.selector_inst[id.index()] == NONE {
                    continue;
                }
                done[i] = true;
                progress = true;
                match apply(store, id, &leaf.value, ORIGIN_SNAPSHOT) {
                    Ok(_) => report.applied += 1,
                    Err(e) => report.rejected.push((leaf.path.clone(), e)),
                }
            }
            if !progress {
                break;
            }
        }
        // Pass 2: everything else.
        for (i, leaf) in leaves.iter().enumerate() {
            if done[i] {
                continue;
            }
            let resolved = store.lookup(&leaf.path).filter(|&id| s.is_param(id));
            match resolved {
                Some(id) => {
                    if !fmask[id.index()] && s.target_inst[id.index()] == NONE {
                        extra_nodes[id.index()] = true;
                    }
                    match apply(store, id, &leaf.value, ORIGIN_SNAPSHOT) {
                        Ok(_) => {
                            report.applied += 1;
                            if let Some((from, to)) = adjustment(store, id, &leaf.value) {
                                report.adjusted.push((leaf.path.clone(), from, to));
                            }
                        }
                        Err(e) => report.rejected.push((leaf.path.clone(), e)),
                    }
                }
                None => {
                    if s.by_path(&leaf.path).is_some_and(|id| !s.is_param(id))
                        && matches!(&leaf.value, Json::Obj(m) if m.is_empty())
                    {
                        // empty container object (`ce_data/lib`): kept as written
                        extras.push((src_path(&leaf.src), leaf.value.clone()));
                        continue;
                    }
                    if KNOWN_UNMODELLED.contains(&leaf.path.as_str()) {
                        report.preserved.push(leaf.path.clone());
                    } else {
                        report.unmapped.push(leaf.path.clone());
                    }
                    extras.push((src_path(&leaf.src), leaf.value.clone()));
                }
            }
        }
        if legacy {
            // 3.x keys a 1.x file cannot have (timefmt/datefmt): factory values
            for (p, v) in &factory().empties {
                if !extras.iter().any(|(q, _)| q == p) {
                    extras.push((p.clone(), v.clone()));
                }
            }
        }
        Ok((
            Snapshot {
                header,
                extras,
                extra_nodes,
            },
            report,
        ))
    }

    /// Whether a node is written by [`Snapshot::save`] under the store's current models.
    pub fn persists(&self, store: &ParamStore, id: NodeId) -> bool {
        let s = store.schema();
        if !s.is_param(id) {
            return false;
        }
        let k = s.target_inst[id.index()];
        if k != NONE {
            // model slot: saved when active and the block's selector is saved
            let sel = s.insts[k as usize].selector;
            return !store.is_inactive(id)
                && (factory().mask[sel as usize] || self.extra_nodes[sel as usize]);
        }
        factory().mask[id.index()] || self.extra_nodes[id.index()]
    }

    /// Move `live` to the state of `target` by setting only the persisted parameters that differ
    /// (`self` is the document `target` was loaded with, see [`Snapshot::load`]). This is the
    /// A/B-compare path: no reset to factory defaults in between, so unchanged parameters are
    /// never touched and every change goes through the normal set path (hooks, native/OSC
    /// fan-out, RT staging) tagged with `origin`. Model selectors are switched first (the new
    /// model's defaults, then the target's slot values). Parameters whose path starts with one of
    /// `skip_prefixes` (console-owned state such as `$syscfg`) are left alone.
    #[allow(clippy::needless_range_loop)]
    pub fn apply_diff(
        &self,
        target: &ParamStore,
        live: &ParamStore,
        origin: Origin,
        skip_prefixes: &[&str],
    ) -> ApplyReport {
        let s = live.schema();
        let mut rep = ApplyReport::default();
        let differs = |id: NodeId| -> bool {
            let d = match live.desc(id) {
                Some(d) => d,
                None => return false,
            };
            if d.ty == ParamType::Str {
                let a = live.with_str(id, str::to_string);
                a != target.with_str(id, str::to_string)
            } else {
                live.get_raw(id) != target.get_raw(id)
            }
        };
        let skipped = |id: NodeId| {
            let p = s.path(id);
            skip_prefixes.iter().any(|x| p.starts_with(x))
        };
        let apply = |id: NodeId, rep: &mut ApplyReport| {
            if !self.persists(target, id) || !differs(id) || skipped(id) {
                return false;
            }
            let r = match live.desc(id) {
                Some(d) if d.ty == ParamType::Str => {
                    let t = target.with_str(id, str::to_string).unwrap_or_default();
                    live.set_str(id, &t, origin)
                }
                _ => live.set_raw(id, target.get_raw(id), origin),
            };
            match r {
                Ok(c) => {
                    rep.changed += c as usize;
                    true
                }
                Err(e) => {
                    rep.rejected.push((s.path(id), e));
                    false
                }
            }
        };
        // selectors first; repeat while new ones become active (selectors inside model slots)
        let mut done = vec![false; s.node_count()];
        loop {
            let mut progress = false;
            for i in 0..s.node_count() {
                let id = NodeId(i as u16);
                if done[i] || s.selector_inst[i] == NONE || !s.is_param(id) || live.is_inactive(id)
                {
                    continue;
                }
                done[i] = true;
                apply(id, &mut rep);
                progress = true;
            }
            if !progress {
                break;
            }
        }
        for i in 0..s.node_count() {
            let id = NodeId(i as u16);
            if !done[i] && s.is_param(id) && !live.is_inactive(id) {
                apply(id, &mut rep);
            }
        }
        rep
    }

    /// The JSON value the console would store for a parameter.
    pub fn value_json(store: &ParamStore, id: NodeId) -> Option<Json> {
        let d = store.desc(id)?;
        let raw = store.get_raw(id);
        Some(match d.ty {
            ParamType::Str => Json::Str(store.with_str(id, str::to_string).unwrap_or_default()),
            ParamType::Enum => match d.enum_token(raw) {
                Some(t) => Json::Str(t.to_string()),
                None => Json::Num(raw.to_string()),
            },
            // tree bools are CparBool; model-slot bools are CparVar ints 0..1 (`fast`, `auto`)
            ParamType::Int
                if d.class == ParamClass::Bool
                    || (d.class == ParamClass::Var && d.min == 0.0 && d.max == 1.0) =>
            {
                Json::Bool(raw != 0)
            }
            ParamType::Int => Json::Num((raw as i32).to_string()),
            ParamType::FEnum => Json::Num(fmt_num(d.as_f64(raw))),
            _ => Json::Num(fmt_num(f32::from_bits(raw) as f64)),
        })
    }

    /// Build the 3.x snapshot document from the store.
    pub fn to_json(&self, store: &ParamStore) -> Json {
        let s = store.schema();
        let mut sections: Vec<(String, Json)> = Vec::new();
        for (sect, root) in SECTIONS {
            let Some(rid) = s.by_path(root) else { continue };
            let skip = |c: NodeId| {
                // `$ctl` and `$globals` have their own sections
                (root.is_empty() && matches!(s.name(c), "$ctl" | "$globals"))
                    || (root == "$ctl" && s.name(c) == "$globals")
            };
            let mut m = Vec::new();
            for c in s.children(rid) {
                if !skip(c) {
                    self.emit(store, c, &mut m);
                }
            }
            sections.push((sect.to_string(), Json::Obj(m)));
        }
        let mut doc = Json::Obj(Vec::new());
        if let Json::Obj(top) = &mut doc {
            top.extend(self.header.iter().cloned());
            top.extend(sections);
        }
        for (path, v) in &self.extras {
            insert(&mut doc, path, v.clone());
        }
        doc
    }

    fn emit(&self, store: &ParamStore, id: NodeId, out: &mut Vec<(String, Json)>) {
        let s = store.schema();
        if s.is_param(id) {
            if self.persists(store, id) {
                if let Some(v) = Self::value_json(store, id) {
                    out.push((store.display_name(id).to_string(), v));
                }
            }
            return;
        }
        let mut m = Vec::new();
        for c in s.children(id) {
            self.emit(store, c, &mut m);
        }
        if !m.is_empty() {
            out.push((s.name(id).to_string(), Json::Obj(m)));
        }
    }

    /// Serialise the store as `.snap` text (3.x layout, compact).
    pub fn save(&self, store: &ParamStore) -> String {
        self.to_json(store).to_text()
    }

    /// Flat `(tree path, value)` list of what [`Snapshot::save`] writes (data sections only,
    /// extras included), e.g. for scene diffs.
    pub fn entries(&self, store: &ParamStore) -> Vec<(String, Json)> {
        flatten(&self.to_json(store))
            .into_iter()
            .map(|l| (l.path, l.value))
            .collect()
    }
}

fn src_path(src: &str) -> Vec<String> {
    src.split('/').map(str::to_string).collect()
}

/// Insert `v` at a JSON key path, creating objects; existing non-objects on the way are kept.
fn insert(doc: &mut Json, path: &[String], v: Json) {
    let Json::Obj(m) = doc else { return };
    let Some((first, rest)) = path.split_first() else {
        return;
    };
    if rest.is_empty() {
        match m.iter_mut().find(|(k, _)| k == first) {
            // an empty object never hides generated content
            Some(_) if matches!(&v, Json::Obj(e) if e.is_empty()) => {}
            Some(slot) => slot.1 = v,
            None => m.push((first.clone(), v)),
        }
        return;
    }
    let idx = match m.iter().position(|(k, _)| k == first) {
        Some(i) => i,
        None => {
            m.push((first.clone(), Json::Obj(Vec::new())));
            m.len() - 1
        }
    };
    insert(&mut m[idx].1, rest, v);
}

/// Shortest decimal text that round-trips the `f32` value.
pub(crate) fn fmt_num(v: f64) -> String {
    let f = v as f32;
    if f == f.trunc() && f.abs() < 1e9 {
        format!("{}", f as i64)
    } else {
        format!("{f}")
    }
}

/// One difference between two snapshots' entries.
#[derive(Clone, Debug, PartialEq)]
pub enum DiffEntry {
    Changed { path: String, a: Json, b: Json },
    OnlyA { path: String, a: Json },
    OnlyB { path: String, b: Json },
}

/// Compare two entry lists (from [`Snapshot::entries`]); numbers compare at `f32` precision.
/// Order follows `a`, then keys only in `b`.
pub fn diff(a: &[(String, Json)], b: &[(String, Json)]) -> Vec<DiffEntry> {
    let bm: HashMap<&str, &Json> = b.iter().map(|(k, v)| (k.as_str(), v)).collect();
    let am: HashMap<&str, &Json> = a.iter().map(|(k, v)| (k.as_str(), v)).collect();
    let mut out = Vec::new();
    for (k, va) in a {
        match bm.get(k.as_str()) {
            None => out.push(DiffEntry::OnlyA {
                path: k.clone(),
                a: va.clone(),
            }),
            Some(vb) if !json_eq(va, vb) => out.push(DiffEntry::Changed {
                path: k.clone(),
                a: va.clone(),
                b: (*vb).clone(),
            }),
            _ => {}
        }
    }
    for (k, vb) in b {
        if !am.contains_key(k.as_str()) {
            out.push(DiffEntry::OnlyB {
                path: k.clone(),
                b: vb.clone(),
            });
        }
    }
    out
}

/// Semantic JSON equality: object key order ignored, numbers equal at `f32` precision.
pub fn json_eq(a: &Json, b: &Json) -> bool {
    match (a, b) {
        (Json::Num(_), Json::Num(_)) => {
            let (x, y) = (
                a.as_f64().unwrap_or(f64::NAN),
                b.as_f64().unwrap_or(f64::NAN),
            );
            x as f32 == y as f32 || (x - y).abs() <= 1e-6 * x.abs().max(y.abs())
        }
        (Json::Arr(x), Json::Arr(y)) => {
            x.len() == y.len() && x.iter().zip(y).all(|(p, q)| json_eq(p, q))
        }
        (Json::Obj(x), Json::Obj(y)) => {
            x.len() == y.len()
                && x.iter().all(|(k, v)| {
                    y.iter()
                        .find(|(k2, _)| k2 == k)
                        .is_some_and(|(_, w)| json_eq(v, w))
                })
        }
        _ => a == b,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fw_major_parses() {
        assert_eq!(fw_major("1.13.9-0-g2d999d50:release"), Some(1));
        assert_eq!(fw_major("3.0.6-193-g60c910ab-dirty:dev_unidyn"), Some(3));
        assert_eq!(fw_major(""), None);
    }

    #[test]
    fn number_format_round_trips_f32() {
        assert_eq!(fmt_num(-144.0), "-144");
        assert_eq!(fmt_num(0.1f32 as f64), "0.1");
        let v = "60.13883591".parse::<f32>().unwrap();
        assert_eq!(fmt_num(v as f64).parse::<f32>().unwrap(), v);
    }

    #[test]
    fn insert_creates_objects() {
        let mut d = Json::Obj(vec![]);
        let p = |s: &str| s.split('/').map(str::to_string).collect::<Vec<_>>();
        insert(&mut d, &p("a/b/c"), Json::Bool(true));
        insert(&mut d, &p("a/b/d"), Json::Null);
        assert_eq!(d.to_text(), r#"{"a":{"b":{"c":true,"d":null}}}"#);
    }

    #[test]
    fn factory_mask_covers_factory_scene() {
        let n = factory().mask.iter().filter(|&&b| b).count();
        // 25,081 keys minus the 3 unmodelled GUI prefs
        assert_eq!(n, 25_078);
    }
}
