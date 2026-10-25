//! The live parameter store: one atomic value per node, the active descriptor per node (model
//! slots), a string table, change notification to the RT engine and protocol hooks.

use std::fmt::{self, Write};
use std::sync::atomic::{AtomicBool, AtomicU16, AtomicU32, Ordering::*};
use std::sync::{Arc, Mutex};

use crate::queue::BoundedQueue;
use crate::schema::{Desc, Model, NodeId, ParamType, Schema, NONE};
use crate::value::{SetError, Value};

/// Who caused a change (lets protocol servers avoid echoing a set back to its sender).
pub type Origin = u32;
/// Changes made by the simulator itself (defaults, model switches without a client origin).
pub const ORIGIN_LOCAL: Origin = 0;

/// A parameter change notification.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Change {
    pub id: u16,
    /// Active descriptor index at notification time (interpret `raw` with `Schema::desc_at`).
    pub desc: u16,
    /// New raw value (see `value` module docs for the encoding).
    pub raw: u32,
    pub origin: Origin,
}

impl Change {
    pub fn node(&self) -> NodeId {
        NodeId(self.id)
    }
}

/// Protocol broadcast hook, called synchronously on the setting thread after every change.
pub type Hook = Box<dyn Fn(&Change) + Send + Sync>;

struct StrSlot {
    buf: Box<[u8]>,
    len: usize,
}

struct Shared {
    values: Box<[AtomicU32]>,
    active: Box<[AtomicU16]>,
    dirty: Box<[AtomicBool]>,
    queue: BoundedQueue<u16>,
}

/// Flat, index-addressed parameter store for the whole WING tree.
///
/// `get*` are wait-free; numeric `set*` are lock-free and allocation-free (sets on model-slot
/// parameters and selector sets take a short internal mutex that serialises model switches; no
/// allocation). Share it with `Arc<ParamStore>`.
pub struct ParamStore {
    schema: &'static Schema,
    shared: Arc<Shared>,
    str_index: Box<[u16]>,
    strings: Box<[Mutex<StrSlot>]>,
    hooks: Vec<Hook>,
    rt_taken: AtomicBool,
    model_lock: Mutex<()>,
}

/// The single RT-side consumer of the change queue. Changes to the same parameter coalesce
/// (the queue holds each parameter at most once, so it can never overflow); `pop` returns the
/// latest value.
pub struct RtConsumer {
    shared: Arc<Shared>,
}

impl RtConsumer {
    /// Next changed parameter with its current value. Lock-free, allocation-free.
    #[inline]
    pub fn pop(&mut self) -> Option<Change> {
        let id = self.shared.queue.pop()?;
        self.shared.dirty[id as usize].store(false, SeqCst);
        let raw = self.shared.values[id as usize].load(SeqCst);
        let desc = self.shared.active[id as usize].load(Acquire);
        Some(Change {
            id,
            desc,
            raw,
            origin: ORIGIN_LOCAL,
        })
    }
}

impl Default for ParamStore {
    fn default() -> Self {
        Self::new()
    }
}

impl ParamStore {
    /// A store holding the factory defaults (firmware 3.0.6 scene, per-model defaults for the
    /// active models).
    pub fn new() -> Self {
        let schema = Schema::get();
        let n = schema.node_count();
        let values: Box<[AtomicU32]> = (0..n).map(|i| AtomicU32::new(schema.defaults[i])).collect();
        let active: Box<[AtomicU16]> = schema
            .nodes
            .iter()
            .map(|r| AtomicU16::new(r.desc))
            .collect();
        let dirty: Box<[AtomicBool]> = (0..n).map(|_| AtomicBool::new(false)).collect();
        let mut str_index = vec![NONE; n].into_boxed_slice();
        let mut strings = Vec::new();
        for (i, r) in schema.nodes.iter().enumerate() {
            if r.desc != NONE {
                let d = &schema.descs[r.desc as usize];
                if d.ty == ParamType::Str {
                    str_index[i] = strings.len() as u16;
                    let init = schema.string(schema.defaults[i]).as_bytes();
                    let cap = (d.maxlen as usize).max(init.len()).max(1);
                    let mut buf = vec![0u8; cap].into_boxed_slice();
                    buf[..init.len()].copy_from_slice(init);
                    strings.push(Mutex::new(StrSlot {
                        buf,
                        len: init.len(),
                    }));
                    values[i].store(0, Relaxed); // generation counter
                }
            }
        }
        let shared = Arc::new(Shared {
            values,
            active,
            dirty,
            queue: BoundedQueue::new(n),
        });
        let store = ParamStore {
            schema,
            shared,
            str_index,
            strings: strings.into_boxed_slice(),
            hooks: Vec::new(),
            rt_taken: AtomicBool::new(false),
            model_lock: Mutex::new(()),
        };
        // activate the default model descriptors (values are already the factory defaults)
        for k in 0..schema.insts.len() {
            store.activate_model(k, false, ORIGIN_LOCAL);
        }
        store
    }

    pub fn schema(&self) -> &'static Schema {
        self.schema
    }

    /// Register a broadcast hook (before sharing the store).
    pub fn add_hook(&mut self, hook: Hook) {
        self.hooks.push(hook);
    }

    /// Take the RT consumer (only once).
    pub fn take_rt_consumer(&self) -> Option<RtConsumer> {
        (!self.rt_taken.swap(true, AcqRel)).then(|| RtConsumer {
            shared: self.shared.clone(),
        })
    }

    // ---- lookup -------------------------------------------------------------------------------

    /// Node by hash (protocol id). O(1), allocation-free.
    #[inline]
    pub fn by_hash(&self, hash: u32) -> Option<NodeId> {
        self.schema.by_hash(hash)
    }

    /// Node by path. Accepts tree paths (`ch/1/dyn/1`) and model parameter names under the
    /// active model (`ch/1/dyn/thr`, as used in snapshots/OSC). For text paths a name of the
    /// active model wins over a numeric slot of the same name (`fx/1/40` under GEQ = the 40 Hz
    /// band in slot 5, not slot 40). Allocation-free.
    pub fn lookup(&self, path: &str) -> Option<NodeId> {
        let p = crate::schema::trim(path);
        if let Some(id) = self.schema.by_path(p) {
            if let Some(parent) = self.schema.parent(id) {
                if let Some(alias) = self.model_alias(parent, self.schema.name(id)) {
                    return Some(alias);
                }
            }
            return Some(id);
        }
        let (prefix, last) = p.rsplit_once('/')?;
        self.model_alias(self.schema.by_path(prefix)?, last)
    }

    fn model_alias(&self, block: NodeId, name: &str) -> Option<NodeId> {
        let s = self.schema;
        let mut k = s.block_inst[block.index()];
        if k == NONE {
            return None;
        }
        while (k as usize) < s.insts.len() && s.insts[k as usize].block == block.0 {
            let inst = &s.insts[k as usize];
            for &t in &s.inst_targets[inst.targets.start as usize..inst.targets.end as usize] {
                let d = s.desc_at(self.shared.active[t as usize].load(Acquire));
                if d.model_param && d.name == name {
                    return Some(NodeId(t));
                }
            }
            k += 1;
        }
        None
    }

    // ---- metadata -----------------------------------------------------------------------------

    /// Active descriptor (depends on the block's model for model slots).
    #[inline]
    pub fn desc(&self, id: NodeId) -> Option<&'static Desc> {
        match self.shared.active[id.index()].load(Acquire) {
            NONE => None,
            d => Some(&self.schema.descs[d as usize]),
        }
    }

    /// Name under which the parameter currently appears (`thr` for an active dyn slot 1).
    pub fn display_name(&self, id: NodeId) -> &'static str {
        match self.desc(id) {
            Some(d) if d.model_param => d.name,
            _ => self.schema.name(id),
        }
    }

    /// Whether the slot is unused by the active model (or the node is not a parameter).
    pub fn is_inactive(&self, id: NodeId) -> bool {
        self.desc(id).is_none_or(|d| d.unused_slot)
    }

    /// Active model of a model-dependent block (first group if several).
    pub fn active_model(&self, block: NodeId) -> Option<&'static Model> {
        let s = self.schema;
        let k = s.block_inst[block.index()];
        if k == NONE {
            return None;
        }
        self.model_for(k as usize)
    }

    fn model_for(&self, k: usize) -> Option<&'static Model> {
        let s = self.schema;
        let inst = &s.insts[k];
        let g = &s.groups[inst.group as usize];
        let sel = self.shared.values[inst.selector as usize].load(SeqCst);
        s.models(g).iter().find(|m| m.enum_index == sel)
    }

    // ---- get ----------------------------------------------------------------------------------

    /// Raw value (see `value` module for the encoding).
    #[inline]
    pub fn get_raw(&self, id: NodeId) -> u32 {
        self.shared.values[id.index()].load(Acquire)
    }

    /// Typed value, `None` for containers.
    #[inline]
    pub fn get(&self, id: NodeId) -> Option<Value> {
        self.desc(id).map(|d| d.decode(self.get_raw(id)))
    }

    /// Numeric view (float engineering value, int, enum index, fenum float).
    pub fn get_f64(&self, id: NodeId) -> Option<f64> {
        self.desc(id).map(|d| d.as_f64(self.get_raw(id)))
    }

    /// Normalised 0..1 position.
    pub fn get_norm(&self, id: NodeId) -> Option<f32> {
        self.desc(id).map(|d| d.to_norm(self.get_raw(id)))
    }

    /// Call `f` with the text of a string parameter.
    pub fn with_str<R>(&self, id: NodeId, f: impl FnOnce(&str) -> R) -> Option<R> {
        let k = self.str_index[id.index()];
        if k == NONE {
            return None;
        }
        let g = self.strings[k as usize]
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        Some(f(std::str::from_utf8(&g.buf[..g.len]).unwrap_or("")))
    }

    /// Display text as WING-Edit shows it (`2k00 Hz`, `-oo dB`, `SHELV`, string text).
    pub fn format<W: Write>(&self, id: NodeId, w: &mut W) -> fmt::Result {
        let Some(d) = self.desc(id) else {
            return Ok(());
        };
        if d.ty == ParamType::Str {
            return self.with_str(id, |s| w.write_str(s)).unwrap_or(Ok(()));
        }
        d.format_with_unit(self.get_raw(id), w)
    }

    /// Snapshot/wire text: enum token, number in plain notation, or string text.
    pub fn format_token<W: Write>(&self, id: NodeId, w: &mut W) -> fmt::Result {
        let Some(d) = self.desc(id) else {
            return Ok(());
        };
        let raw = self.get_raw(id);
        match d.ty {
            ParamType::Str => self.with_str(id, |s| w.write_str(s)).unwrap_or(Ok(())),
            ParamType::Enum => match d.enum_token(raw) {
                Some(t) => w.write_str(t),
                None => write!(w, "{raw}"),
            },
            ParamType::Int => write!(w, "{}", raw as i32),
            ParamType::FEnum => write!(w, "{}", d.as_f64(raw)),
            _ => write!(w, "{}", f32::from_bits(raw)),
        }
    }

    // ---- set ----------------------------------------------------------------------------------

    fn writable(&self, id: NodeId) -> Result<&'static Desc, SetError> {
        match self.desc(id) {
            None => Err(SetError::NotParam),
            Some(d) if d.unused_slot => Err(SetError::Inactive),
            Some(d) => Ok(d),
        }
    }

    /// Set a value (clamped/quantised against the active descriptor). Returns whether the stored
    /// value changed. Notifies the RT queue and hooks on change; a changed model selector
    /// switches the block's model and applies the new model's defaults.
    pub fn set(&self, id: NodeId, v: Value) -> Result<bool, SetError> {
        self.set_from(id, v, ORIGIN_LOCAL)
    }

    /// [`ParamStore::set`] with an origin tag passed to hooks.
    pub fn set_from(&self, id: NodeId, v: Value, origin: Origin) -> Result<bool, SetError> {
        self.set_with(id, origin, |d| d.encode(v))
    }

    /// Store an already-encoded raw value (see the `value` module) without clamping or
    /// quantising, e.g. a float a snapshot holds exactly. Model/inactive checks and notification
    /// as [`ParamStore::set`]; string parameters are refused.
    pub fn set_raw(&self, id: NodeId, raw: u32, origin: Origin) -> Result<bool, SetError> {
        self.set_with(id, origin, |_| Ok(raw))
    }

    /// Set from a normalised 0..1 position.
    pub fn set_norm(&self, id: NodeId, p: f32, origin: Origin) -> Result<bool, SetError> {
        self.set_with(id, origin, |d| d.from_norm(p))
    }

    /// Set an enum by token or display name.
    pub fn set_enum_name(&self, id: NodeId, name: &str, origin: Origin) -> Result<bool, SetError> {
        self.set_with(id, origin, |d| d.enum_index(name).ok_or(SetError::BadValue))
    }

    /// Set from display/snapshot text (`-40.0`, `2k00`, `SHV`, `ON`); strings take the text.
    pub fn set_text(&self, id: NodeId, text: &str, origin: Origin) -> Result<bool, SetError> {
        if self.str_index[id.index()] != NONE {
            return self.set_str(id, text, origin);
        }
        self.set_with(id, origin, |d| d.parse_text(text))
    }

    fn set_with(
        &self,
        id: NodeId,
        origin: Origin,
        enc: impl FnOnce(&Desc) -> Result<u32, SetError>,
    ) -> Result<bool, SetError> {
        let s = self.schema;
        let in_model = s.target_inst[id.index()] != NONE || s.selector_inst[id.index()] != NONE;
        // serialise against model switches so the value is quantised for the descriptor it is
        // stored under
        let _g = in_model.then(|| self.model_lock.lock().unwrap_or_else(|e| e.into_inner()));
        let d = self.writable(id)?;
        if d.ty == ParamType::Str {
            return Err(SetError::TypeMismatch);
        }
        let raw = enc(d)?;
        let changed = self.store_raw(id, raw, origin);
        if changed {
            let k = s.selector_inst[id.index()];
            if k != NONE {
                // all instances whose selector is this node (normally one)
                self.activate_model(k as usize, true, origin);
            }
        }
        Ok(changed)
    }

    /// Set a string parameter (truncated to `maxlen` on a char boundary).
    pub fn set_str(&self, id: NodeId, text: &str, origin: Origin) -> Result<bool, SetError> {
        let d = self.writable(id)?;
        let k = self.str_index[id.index()];
        if k == NONE || d.ty != ParamType::Str {
            return Err(SetError::TypeMismatch);
        }
        let changed = {
            let mut g = self.strings[k as usize]
                .lock()
                .unwrap_or_else(|e| e.into_inner());
            let mut n = text.len().min(g.buf.len());
            while !text.is_char_boundary(n) {
                n -= 1;
            }
            let b = &text.as_bytes()[..n];
            if &g.buf[..g.len] == b {
                false
            } else {
                g.buf[..n].copy_from_slice(b);
                g.len = n;
                true
            }
        };
        if changed {
            let gen = self.shared.values[id.index()]
                .fetch_add(1, SeqCst)
                .wrapping_add(1);
            self.notify(id, gen, origin);
        }
        Ok(changed)
    }

    /// Store a raw value and notify on change.
    fn store_raw(&self, id: NodeId, raw: u32, origin: Origin) -> bool {
        let old = self.shared.values[id.index()].swap(raw, SeqCst);
        if old != raw {
            self.notify(id, raw, origin);
            true
        } else {
            false
        }
    }

    fn notify(&self, id: NodeId, raw: u32, origin: Origin) {
        let sh = &*self.shared;
        if !sh.dirty[id.index()].swap(true, SeqCst) {
            // capacity >= node count and each id is queued at most once: cannot fail
            let _ = sh.queue.push(id.0);
        }
        if !self.hooks.is_empty() {
            let c = Change {
                id: id.0,
                desc: sh.active[id.index()].load(Acquire),
                raw,
                origin,
            };
            for h in &self.hooks {
                h(&c);
            }
        }
    }

    /// Point every target of instance `k` at the active model's descriptor; optionally load the
    /// model's defaults (model switch). Caller holds `model_lock` when `apply_defaults`.
    fn activate_model(&self, k: usize, apply_defaults: bool, origin: Origin) {
        let s = self.schema;
        let inst = &s.insts[k];
        let model = self.model_for(k);
        let g = &s.groups[inst.group as usize];
        for (j, &t) in s.inst_targets[inst.targets.start as usize..inst.targets.end as usize]
            .iter()
            .enumerate()
        {
            let tname = s.targets[g.targets.start as usize + j];
            let entry = model.and_then(|m| {
                s.entries[m.entries.start as usize..m.entries.end as usize]
                    .iter()
                    .find(|e| e.target == tname)
            });
            let (desc, default) = match entry {
                Some(e) => (e.desc, s.descs[e.desc as usize].default_raw),
                None => (s.nodes[t as usize].desc, 0),
            };
            let old = self.shared.active[t as usize].swap(desc, AcqRel);
            if apply_defaults {
                let changed = self.store_raw(NodeId(t), default, origin);
                if !changed && old != desc {
                    // same raw but a different meaning: still tell listeners
                    self.notify(NodeId(t), default, origin);
                }
            }
        }
    }

    /// Reset every parameter to the factory defaults (notifies changes).
    pub fn reset_to_defaults(&self) {
        let s = self.schema;
        let _g = self.model_lock.lock().unwrap_or_else(|e| e.into_inner());
        for i in 0..s.node_count() {
            let id = NodeId(i as u16);
            if self.str_index[i] != NONE {
                let _ = self.set_str(id, s.string(s.defaults[i]), ORIGIN_LOCAL);
            } else if s.nodes[i].desc != NONE {
                self.store_raw(id, s.defaults[i], ORIGIN_LOCAL);
            }
        }
        for k in 0..s.insts.len() {
            self.activate_model(k, false, ORIGIN_LOCAL);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::AtomicUsize;

    fn id(st: &ParamStore, p: &str) -> NodeId {
        st.lookup(p).unwrap_or_else(|| panic!("{p}"))
    }

    #[test]
    fn factory_defaults() {
        let st = ParamStore::new();
        assert_eq!(st.get(id(&st, "ch/1/fdr")), Some(Value::Float(-144.0)));
        // ch dyn default model COMP; its threshold from the factory scene
        let dyn_ = id(&st, "ch/1/dyn");
        assert_eq!(st.active_model(dyn_).unwrap().name, "COMP");
        let thr = id(&st, "ch/1/dyn/thr");
        assert_eq!(st.schema().name(thr), "1");
        assert_eq!(st.display_name(thr), "thr");
        assert_eq!(st.get(thr), Some(Value::Float(-10.0)));
        let lcf = id(&st, "ch/1/flt/lcf");
        assert!((st.get_f64(lcf).unwrap() - 100.2374573).abs() < 1e-4);
        assert_eq!(
            st.with_str(id(&st, "ch/1/name"), |s| s.to_string()),
            Some(String::new())
        );
    }

    #[test]
    fn set_returns_changed_and_validates() {
        let st = ParamStore::new();
        let fdr = id(&st, "ch/3/fdr");
        assert_eq!(st.set(fdr, Value::Float(0.0)), Ok(true));
        assert_eq!(st.set(fdr, Value::Float(0.0)), Ok(false));
        assert_eq!(st.set(fdr, Value::Float(99.0)), Ok(true));
        assert_eq!(st.get(fdr), Some(Value::Float(10.0)));
        assert_eq!(
            st.set(id(&st, "ch/3"), Value::Int(1)),
            Err(SetError::NotParam)
        );
        assert_eq!(
            st.set(id(&st, "ch/3/name"), Value::Int(1)),
            Err(SetError::TypeMismatch)
        );
        // slot 19 of COMP is unused
        assert_eq!(
            st.set(id(&st, "ch/3/dyn/19"), Value::Int(0)),
            Err(SetError::Inactive)
        );
        assert_eq!(st.set_str(id(&st, "ch/3/name"), "Kick", 7), Ok(true));
        assert_eq!(st.set_str(id(&st, "ch/3/name"), "Kick", 7), Ok(false));
        assert_eq!(
            st.with_str(id(&st, "ch/3/name"), |s| s.to_string())
                .unwrap(),
            "Kick"
        );
        let long = "x".repeat(500);
        st.set_str(id(&st, "ch/3/name"), &long, 0).unwrap();
        let len = st.with_str(id(&st, "ch/3/name"), |s| s.len()).unwrap();
        assert_eq!(len as u32, st.desc(id(&st, "ch/3/name")).unwrap().maxlen);
    }

    #[test]
    fn model_switch_applies_model_defaults() {
        let st = ParamStore::new();
        let mdl = id(&st, "ch/2/dyn/mdl");
        let slot1 = id(&st, "ch/2/dyn/1");
        assert_eq!(st.set_enum_name(mdl, "GATE", 0), Ok(true));
        assert_eq!(st.display_name(slot1), "thr");
        assert_eq!(st.get(slot1), Some(Value::Float(-40.0)));
        assert_eq!(st.lookup("ch/2/dyn/range"), Some(id(&st, "ch/2/dyn/2")));
        assert_eq!(st.get(id(&st, "ch/2/dyn/range")), Some(Value::Float(40.0)));
        // GATE has 7 params; slot 8 is unused
        assert!(st.is_inactive(id(&st, "ch/2/dyn/8")));
        // validation follows the active model: GATE thr range is -80..0
        assert_eq!(st.set(slot1, Value::Float(-100.0)), Ok(true));
        assert_eq!(st.get(slot1), Some(Value::Float(-80.0)));
        // DUCK: same slot, different default
        st.set_enum_name(mdl, "DUCK", 0).unwrap();
        assert_eq!(st.get(slot1), Some(Value::Float(-20.0)));
        // LA: slot 1 is `ingain` now (hash by position)
        st.set_enum_name(mdl, "LA", 0).unwrap();
        assert_eq!(st.display_name(slot1), "ingain");
        assert_eq!(st.lookup("ch/2/dyn/ingain"), Some(slot1));
        assert_eq!(st.schema().hash(slot1), crate::wing_hash("ch/2/dyn/1"));
        // other channels unaffected
        assert_eq!(st.active_model(id(&st, "ch/3/dyn")).unwrap().name, "COMP");
    }

    #[test]
    fn named_variant_param_and_numeric_names() {
        let st = ParamStore::new();
        // in/set dlymode changes the unit/range of `dly` (not a slot)
        let dly = id(&st, "ch/1/in/set/dly");
        assert_eq!(st.desc(dly).unwrap().unit.as_str(), "m");
        st.set_enum_name(id(&st, "ch/1/in/set/dlymode"), "FT", 0)
            .unwrap();
        assert_eq!(st.desc(dly).unwrap().unit.as_str(), "ft");
        assert_eq!(st.get(dly), Some(Value::Float(0.5)));
        // fx GEQ: `40` is the 40 Hz band in slot 5, not slot 40
        let fx = id(&st, "fx/3/mdl");
        st.set_enum_name(fx, "GEQ", 0).unwrap();
        let band = id(&st, "fx/3/40");
        assert_eq!(st.schema().name(band), "5");
        assert!(st.desc(band).unwrap().from_models_md);
        assert_eq!(st.set(band, Value::Float(20.0)), Ok(true));
        assert_eq!(st.get(band), Some(Value::Float(15.0)));
        // a slot path still addresses the slot when the model has no such name
        st.set_enum_name(fx, "PIA", 0).unwrap();
        assert_eq!(st.schema().name(id(&st, "fx/3/40")), "40");
        assert_eq!(st.schema().name(id(&st, "fx/3/31")), "3");
        // bus EQ PIA (models.md §4) has its 31 Hz band in slot 2
        st.set_enum_name(id(&st, "bus/1/eq/mdl"), "PIA", 0).unwrap();
        assert_eq!(st.schema().name(id(&st, "bus/1/eq/31")), "2");
        assert!(st
            .set_enum_name(id(&st, "bus/1/eq/mdl"), "MACH4", 0)
            .is_err());
    }

    #[test]
    fn rt_queue_coalesces_and_hooks_fire() {
        let mut st = ParamStore::new();
        let hits = Arc::new(AtomicUsize::new(0));
        let h = hits.clone();
        st.add_hook(Box::new(move |c| {
            if c.origin == 42 {
                h.fetch_add(1, SeqCst);
            }
        }));
        let mut rt = st.take_rt_consumer().unwrap();
        assert!(st.take_rt_consumer().is_none());
        let a = id(&st, "ch/1/fdr");
        let b = id(&st, "bus/1/fdr");
        st.set_from(a, Value::Float(-10.0), 42).unwrap();
        st.set_from(a, Value::Float(-5.0), 42).unwrap();
        st.set_from(b, Value::Float(-3.0), 42).unwrap();
        st.set_from(b, Value::Float(-3.0), 42).unwrap(); // no change, no notification
        assert_eq!(hits.load(SeqCst), 3);
        let c1 = rt.pop().unwrap();
        assert_eq!((c1.node(), f32::from_bits(c1.raw)), (a, -5.0));
        let c2 = rt.pop().unwrap();
        assert_eq!((c2.node(), f32::from_bits(c2.raw)), (b, -3.0));
        assert!(rt.pop().is_none());
        st.set(a, Value::Float(1.0)).unwrap();
        assert_eq!(rt.pop().unwrap().node(), a);
    }

    #[test]
    fn requantise_defaults_is_stable() {
        // every float default sits on its grid (or is within one f32 ulp-ish of it)
        let st = ParamStore::new();
        let s = st.schema();
        let mut n = 0;
        for i in s.ids() {
            let Some(d) = st.desc(i) else { continue };
            if !matches!(d.ty, ParamType::LinF | ParamType::LogF) || d.unused_slot || s.is_live(i) {
                continue;
            }
            let v = f32::from_bits(st.get_raw(i));
            let q = d.quantize(v as f64);
            assert!(
                (q - v).abs() <= 1e-3 * v.abs().max(1.0),
                "{} {v} -> {q}",
                s.path(i)
            );
            n += 1;
        }
        assert!(n > 5000);
    }

    #[test]
    fn reset_restores_defaults() {
        let st = ParamStore::new();
        let fdr = id(&st, "main/1/fdr");
        let before = st.get_raw(fdr);
        st.set(fdr, Value::Float(3.0)).unwrap();
        st.set_enum_name(id(&st, "ch/1/dyn/mdl"), "GATE", 0)
            .unwrap();
        st.set_str(id(&st, "ch/1/name"), "x", 0).unwrap();
        st.reset_to_defaults();
        assert_eq!(st.get_raw(fdr), before);
        assert_eq!(st.active_model(id(&st, "ch/1/dyn")).unwrap().name, "COMP");
        assert_eq!(st.display_name(id(&st, "ch/1/dyn/1")), "thr");
        assert_eq!(
            st.with_str(id(&st, "ch/1/name"), |s| s.is_empty()),
            Some(true)
        );
    }
}
