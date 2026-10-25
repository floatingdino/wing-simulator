//! Static parameter schema decoded from the generated blob `schema.bin`.
//!
//! The blob is produced by `tools/schema-extract/gen_model.py` from `data/schema/wing-tree.json`
//! (WING-Edit 3.3.4, T003) and the firmware 3.0.6 factory scene (T008). It holds every concrete
//! node of the default tree (43,008 incl. root) in preorder, the parameter descriptors (deduplicated),
//! the per-model descriptor tables of the model-dependent blocks and the per-node factory defaults.
//! Decoding happens once (`Schema::get()`); all lookups afterwards are allocation-free.

use std::fmt;
use std::ops::Range;
use std::sync::OnceLock;

use crate::hash::wing_hash;

static BLOB: &[u8] = include_bytes!("schema.bin");

/// Sentinel used in the blob's u16 link fields.
pub(crate) const NONE: u16 = u16::MAX;

/// Index of a node in the flat tree (preorder; root is 0).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct NodeId(pub u16);

impl NodeId {
    pub const ROOT: NodeId = NodeId(0);
    #[inline]
    pub fn index(self) -> usize {
        self.0 as usize
    }
}

/// Parameter type (the `Cnode` flags type nibble).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum ParamType {
    /// Linear float: `v = min + step*i`.
    LinF = 1,
    /// Logarithmic float: `v = min * exp(logstep*i)`.
    LogF = 2,
    /// Fader level in dB (-144 = -oo .. +10), WING fader law for normalised positions.
    Fader = 3,
    /// Integer (also bools, triggers).
    Int = 4,
    /// Enumeration; the value is the index into the token list.
    Enum = 5,
    /// Float enumeration (`values` + `labels`); the value is the index.
    FEnum = 6,
    /// String with a maximum length.
    Str = 7,
}

/// WING-Edit parameter class (`ncl::Cpar` subclass).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum ParamClass {
    Int,
    Bool,
    Enum,
    LinFloat,
    LogFloat,
    Level,
    Str,
    Fenum,
    Trigger,
    Var,
    Tag,
    Stringlist,
}

/// Unit nibble (flags bits 16..19); names from the binary's unit table.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Unit {
    None,
    Db,
    Percent,
    Ms,
    Hz,
    M,
    S,
    Ft,
}

impl Unit {
    pub fn as_str(self) -> &'static str {
        match self {
            Unit::None => "",
            Unit::Db => "dB",
            Unit::Percent => "%",
            Unit::Ms => "ms",
            Unit::Hz => "Hz",
            Unit::M => "m",
            Unit::S => "s",
            Unit::Ft => "ft",
        }
    }
}

/// One enum entry: wire/snapshot token, display name and (for `FEnum`) the float value.
#[derive(Clone, Copy, Debug)]
pub struct EnumItem {
    pub token: &'static str,
    pub long: &'static str,
    pub value: f64,
}

/// Parameter descriptor (metadata). Shared by all instances of a template; model-dependent
/// slots switch between descriptors (see [`crate::ParamStore`]).
#[derive(Clone, Copy, Debug)]
pub struct Desc {
    /// Model parameter name (`thr`) for model descriptors; empty for tree descriptors (use the
    /// node name).
    pub name: &'static str,
    /// Console display name (`LC FREQ`).
    pub long: &'static str,
    /// printf-like display format (`%.1f`, `%+.f`, `%k.1f`, `%T`, ...); may be empty.
    pub fmt: &'static str,
    /// WING-Edit's own `describe` text, e.g. `log [20.0 .. 2k00 Hz], 641 steps`.
    pub describe: &'static str,
    /// Raw `Cnode` flags.
    pub flags: u32,
    pub class: ParamClass,
    pub ty: ParamType,
    pub unit: Unit,
    /// Descriptor of a model (variant) parameter.
    pub model_param: bool,
    /// The n/a slot descriptor (slot not used by the active model).
    pub unused_slot: bool,
    /// Filled in from docs/research/models.md because T003's extraction missed it.
    pub from_models_md: bool,
    pub steps: u32,
    pub min: f64,
    pub max: f64,
    /// Linear step (`LinF`) or natural-log step (`LogF`).
    pub step: f64,
    pub(crate) enum_start: u32,
    pub(crate) enum_len: u32,
    /// Maximum string length (`Str`).
    pub maxlen: u32,
    /// Raw default (model descriptors only: the model's `pardef` default).
    pub default_raw: u32,
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct NodeRec {
    pub name: &'static str,
    pub long: &'static str,
    pub parent: u16,
    pub first_child: u16,
    pub next_sibling: u16,
    pub desc: u16,
    pub hash: u32,
}

/// A model-dependent block type (e.g. `ch/*/dyn` with selector `mdl`).
#[derive(Clone, Debug)]
pub struct Group {
    /// Variant key from wing-tree.json (`ch/*/dyn`, `fx/1`, ...).
    pub key: &'static str,
    /// Name of the selector child (`mdl`, `mode`, `dlymode`).
    pub selector: &'static str,
    pub(crate) models: Range<u32>,
    pub(crate) targets: Range<u32>,
}

/// One model of a [`Group`]: which descriptor each target child uses under it.
#[derive(Clone, Debug)]
pub struct Model {
    pub name: &'static str,
    pub long: &'static str,
    /// Selector enum index that activates this model.
    pub enum_index: u32,
    pub(crate) entries: Range<u32>,
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct Entry {
    pub target: &'static str,
    pub desc: u16,
}

/// A concrete model-dependent block instance (resolved node ids).
#[derive(Clone, Debug)]
pub(crate) struct Inst {
    pub block: u16,
    pub group: u16,
    pub selector: u16,
    /// Range into `Schema::inst_targets` (parallel to the group's target list).
    pub targets: Range<u32>,
}

/// The decoded schema. Obtain with [`Schema::get`].
pub struct Schema {
    pub(crate) descs: Vec<Desc>,
    pub(crate) enums: Vec<EnumItem>,
    pub(crate) nodes: Vec<NodeRec>,
    pub(crate) defaults: Vec<u32>,
    pub(crate) strings: Vec<&'static str>,
    pub(crate) groups: Vec<Group>,
    pub(crate) models: Vec<Model>,
    pub(crate) entries: Vec<Entry>,
    pub(crate) targets: Vec<&'static str>,
    pub(crate) insts: Vec<Inst>,
    pub(crate) inst_targets: Vec<u16>,
    /// node -> first instance whose block is this node (NONE if none).
    pub(crate) block_inst: Vec<u16>,
    /// node -> instance whose selector is this node (NONE if none).
    pub(crate) selector_inst: Vec<u16>,
    /// node -> instance in which this node is a target (NONE if none).
    pub(crate) target_inst: Vec<u16>,
    hash_index: Vec<u16>,
    hash_shift: u32,
    param_count: usize,
}

struct Cursor {
    pos: usize,
}

impl Cursor {
    fn u32(&mut self) -> u32 {
        let v = u32::from_le_bytes(BLOB[self.pos..self.pos + 4].try_into().unwrap());
        self.pos += 4;
        v
    }
    fn u16(&mut self) -> u16 {
        let v = u16::from_le_bytes(BLOB[self.pos..self.pos + 2].try_into().unwrap());
        self.pos += 2;
        v
    }
    fn u8(&mut self) -> u8 {
        let v = BLOB[self.pos];
        self.pos += 1;
        v
    }
    fn f64(&mut self) -> f64 {
        let v = f64::from_le_bytes(BLOB[self.pos..self.pos + 8].try_into().unwrap());
        self.pos += 8;
        v
    }
}

fn class_of(v: u8) -> ParamClass {
    use ParamClass::*;
    [
        Int, Bool, Enum, LinFloat, LogFloat, Level, Str, Fenum, Trigger, Var, Tag, Stringlist,
    ][v as usize]
}

fn type_of(v: u8) -> ParamType {
    use ParamType::*;
    [LinF, LinF, LogF, Fader, Int, Enum, FEnum, Str][v as usize]
}

fn unit_of(v: u8) -> Unit {
    use Unit::*;
    [None, Db, Percent, Ms, Hz, M, S, Ft][v as usize]
}

impl Schema {
    /// The process-wide schema (decoded on first use, a few milliseconds).
    pub fn get() -> &'static Schema {
        static S: OnceLock<Schema> = OnceLock::new();
        S.get_or_init(Schema::decode)
    }

    fn decode() -> Schema {
        assert_eq!(&BLOB[0..4], b"WMDL", "schema.bin: bad magic");
        let mut c = Cursor { pos: 4 };
        assert_eq!(c.u32(), 1, "schema.bin: unsupported version");
        let [n_str, n_desc, n_enum, n_node, n_group, n_model, n_entry, n_target, n_inst] =
            [(); 9].map(|_| c.u32() as usize);
        let offs: Vec<usize> = (0..=n_str).map(|_| c.u32() as usize).collect();
        let base = c.pos;
        let strings: Vec<&'static str> = (0..n_str)
            .map(|i| std::str::from_utf8(&BLOB[base + offs[i]..base + offs[i + 1]]).expect("utf-8"))
            .collect();
        c.pos = (base + offs[n_str]).next_multiple_of(4);
        let s = |id: u32| strings[id as usize];

        let descs: Vec<Desc> = (0..n_desc)
            .map(|_| {
                let (name, long, fmt, describe, flags) =
                    (c.u32(), c.u32(), c.u32(), c.u32(), c.u32());
                let (class, ty, unit, var) = (c.u8(), c.u8(), c.u8(), c.u8());
                let steps = c.u32();
                let _pad = c.u32();
                let (min, max, step) = (c.f64(), c.f64(), c.f64());
                let (enum_start, enum_len, maxlen, default_raw) =
                    (c.u32(), c.u32(), c.u32(), c.u32());
                Desc {
                    name: s(name),
                    long: s(long),
                    fmt: s(fmt),
                    describe: s(describe),
                    flags,
                    class: class_of(class),
                    ty: type_of(ty),
                    unit: unit_of(unit),
                    model_param: var & 1 != 0,
                    unused_slot: var & 2 != 0,
                    from_models_md: var & 4 != 0,
                    steps,
                    min,
                    max,
                    step,
                    enum_start,
                    enum_len,
                    maxlen,
                    default_raw,
                }
            })
            .collect();
        let enums: Vec<EnumItem> = (0..n_enum)
            .map(|_| EnumItem {
                token: s(c.u32()),
                long: s(c.u32()),
                value: c.f64(),
            })
            .collect();
        let mut nodes: Vec<NodeRec> = (0..n_node)
            .map(|_| NodeRec {
                name: s(c.u32()),
                long: s(c.u32()),
                parent: c.u16(),
                first_child: c.u16(),
                next_sibling: c.u16(),
                desc: c.u16(),
                hash: 0,
            })
            .collect();
        let defaults: Vec<u32> = (0..n_node).map(|_| c.u32()).collect();
        let groups: Vec<Group> = (0..n_group)
            .map(|_| {
                let (key, sel, ms, mc, ts, tc) =
                    (c.u32(), c.u32(), c.u32(), c.u32(), c.u32(), c.u32());
                Group {
                    key: s(key),
                    selector: s(sel),
                    models: ms..ms + mc,
                    targets: ts..ts + tc,
                }
            })
            .collect();
        let models: Vec<Model> = (0..n_model)
            .map(|_| {
                let (name, long, idx, es, ec) = (c.u32(), c.u32(), c.u32(), c.u32(), c.u32());
                Model {
                    name: s(name),
                    long: s(long),
                    enum_index: idx,
                    entries: es..es + ec,
                }
            })
            .collect();
        let entries: Vec<Entry> = (0..n_entry)
            .map(|_| Entry {
                target: s(c.u32()),
                desc: c.u32() as u16,
            })
            .collect();
        let targets: Vec<&'static str> = (0..n_target).map(|_| s(c.u32())).collect();
        let raw_insts: Vec<(u16, u16)> = (0..n_inst)
            .map(|_| (c.u32() as u16, c.u32() as u16))
            .collect();
        assert_eq!(c.pos, BLOB.len(), "schema.bin: trailing bytes");

        // Node hashes: computed from the path (parity with WING-Edit is tested against the JSON).
        let mut path = String::with_capacity(128);
        let mut stack: Vec<usize> = Vec::with_capacity(16); // path length before each depth
        for i in 1..nodes.len() {
            // nodes are in preorder: unwind to the parent's depth
            let mut depth = 0;
            let mut p = nodes[i].parent;
            while p != 0 {
                depth += 1;
                p = nodes[p as usize].parent;
            }
            stack.truncate(depth);
            path.truncate(stack.last().copied().unwrap_or(0));
            if depth > 0 {
                path.push('/');
            }
            path.push_str(nodes[i].name);
            stack.push(path.len());
            nodes[i].hash = wing_hash(&path);
        }

        // Open-addressing hash index (Fibonacci hashing, linear probing), load factor < 0.33.
        let bits = (nodes.len() * 3).next_power_of_two().trailing_zeros();
        let mut hash_index = vec![NONE; 1 << bits];
        let shift = 32 - bits;
        let mask = hash_index.len() - 1;
        for (i, n) in nodes.iter().enumerate().skip(1) {
            let mut slot = (n.hash.wrapping_mul(0x9E37_79B1) >> shift) as usize;
            while hash_index[slot] != NONE {
                assert_ne!(
                    nodes[hash_index[slot] as usize].hash, n.hash,
                    "hash collision"
                );
                slot = (slot + 1) & mask;
            }
            hash_index[slot] = i as u16;
        }

        let child_named = |nodes: &[NodeRec], parent: u16, name: &str| -> u16 {
            let mut ch = nodes[parent as usize].first_child;
            while ch != NONE {
                if nodes[ch as usize].name == name {
                    return ch;
                }
                ch = nodes[ch as usize].next_sibling;
            }
            panic!("schema.bin: missing child {name}");
        };
        let mut block_inst = vec![NONE; nodes.len()];
        let mut selector_inst = vec![NONE; nodes.len()];
        let mut target_inst = vec![NONE; nodes.len()];
        let mut insts = Vec::with_capacity(raw_insts.len());
        let mut inst_targets = Vec::new();
        for (k, &(block, group)) in raw_insts.iter().enumerate() {
            let g = &groups[group as usize];
            let sel = child_named(&nodes, block, g.selector);
            let start = inst_targets.len() as u32;
            for t in g.targets.clone() {
                let id = child_named(&nodes, block, targets[t as usize]);
                target_inst[id as usize] = k as u16;
                inst_targets.push(id);
            }
            if block_inst[block as usize] == NONE {
                block_inst[block as usize] = k as u16;
            }
            selector_inst[sel as usize] = k as u16;
            insts.push(Inst {
                block,
                group,
                selector: sel,
                targets: start..inst_targets.len() as u32,
            });
        }
        let param_count = nodes.iter().filter(|n| n.desc != NONE).count();
        Schema {
            descs,
            enums,
            nodes,
            defaults,
            strings,
            groups,
            models,
            entries,
            targets,
            insts,
            inst_targets,
            block_inst,
            selector_inst,
            target_inst,
            hash_index,
            hash_shift: shift,
            param_count,
        }
    }

    /// Number of nodes including the root (43,008 for WING-Edit 3.3.4).
    pub fn node_count(&self) -> usize {
        self.nodes.len()
    }

    /// Number of parameter nodes (36,499).
    pub fn param_count(&self) -> usize {
        self.param_count
    }

    /// All node ids in preorder (dump order).
    pub fn ids(&self) -> impl Iterator<Item = NodeId> + '_ {
        (0..self.nodes.len() as u16).map(NodeId)
    }

    /// Node name (last path segment).
    #[inline]
    pub fn name(&self, id: NodeId) -> &'static str {
        self.nodes[id.index()].name
    }

    /// Console display name of the node (per-instance where the console has one, e.g. `A1`).
    pub fn long_name(&self, id: NodeId) -> &'static str {
        let n = &self.nodes[id.index()];
        if n.long.is_empty() && n.desc != NONE {
            self.descs[n.desc as usize].long
        } else {
            n.long
        }
    }

    /// The node's hash id as used by the native protocol.
    #[inline]
    pub fn hash(&self, id: NodeId) -> u32 {
        self.nodes[id.index()].hash
    }

    #[inline]
    pub fn parent(&self, id: NodeId) -> Option<NodeId> {
        match self.nodes[id.index()].parent {
            NONE => None,
            p => Some(NodeId(p)),
        }
    }

    /// Iterate the direct children of a node in tree order.
    pub fn children(&self, id: NodeId) -> Children<'_> {
        Children {
            s: self,
            next: self.nodes[id.index()].first_child,
        }
    }

    /// Direct child by name.
    pub fn child(&self, id: NodeId, name: &str) -> Option<NodeId> {
        self.children(id).find(|&c| self.name(c) == name)
    }

    /// Whether the node is a parameter (leaf) rather than a container node.
    #[inline]
    pub fn is_param(&self, id: NodeId) -> bool {
        self.nodes[id.index()].desc != NONE
    }

    /// The node's tree descriptor (for model slots: the n/a descriptor; the store knows the
    /// active one).
    pub fn base_desc(&self, id: NodeId) -> Option<&Desc> {
        let d = self.nodes[id.index()].desc;
        (d != NONE).then(|| &self.descs[d as usize])
    }

    /// Descriptor by index.
    #[inline]
    pub fn desc_at(&self, idx: u16) -> &Desc {
        &self.descs[idx as usize]
    }

    /// Enum items of a descriptor.
    pub fn enum_items(&self, d: &Desc) -> &[EnumItem] {
        &self.enums[d.enum_start as usize..(d.enum_start + d.enum_len) as usize]
    }

    /// Factory default (raw) of a node, as in the firmware 3.0.6 factory scene.
    #[inline]
    pub fn default_raw(&self, id: NodeId) -> u32 {
        self.defaults[id.index()]
    }

    /// String by blob string id (string defaults are stored this way).
    pub(crate) fn string(&self, id: u32) -> &'static str {
        self.strings[id as usize]
    }

    /// Whether the name marks a live/read-only value not stored in snapshots (`$` prefix).
    pub fn is_live(&self, id: NodeId) -> bool {
        self.name(id).starts_with('$')
    }

    /// Node by hash; O(1) expected, allocation-free.
    #[inline]
    pub fn by_hash(&self, hash: u32) -> Option<NodeId> {
        let mask = self.hash_index.len() - 1;
        let mut slot = (hash.wrapping_mul(0x9E37_79B1) >> self.hash_shift) as usize;
        loop {
            let id = self.hash_index[slot];
            if id == NONE {
                return None;
            }
            if self.nodes[id as usize].hash == hash {
                return Some(NodeId(id));
            }
            slot = (slot + 1) & mask;
        }
    }

    /// Node by exact tree path (`ch/1/fdr`, leading/trailing `/` optional; slot paths such as
    /// `ch/1/dyn/1`). Model parameter names (`ch/1/dyn/thr`) need the active model: use
    /// [`crate::ParamStore::lookup`]. Allocation-free: hash, then verify the path.
    pub fn by_path(&self, path: &str) -> Option<NodeId> {
        let p = trim(path);
        if p.is_empty() {
            return Some(NodeId::ROOT);
        }
        let id = self.by_hash(wing_hash(p))?;
        self.path_eq(id, p).then_some(id)
    }

    fn path_eq(&self, mut id: NodeId, p: &str) -> bool {
        let mut rest = p;
        loop {
            let name = self.name(id);
            match rest.strip_suffix(name) {
                Some(r) => rest = r,
                None => return false,
            }
            id = match self.parent(id) {
                Some(NodeId(0)) | None => return rest.is_empty(),
                Some(p) => p,
            };
            match rest.strip_suffix('/') {
                Some(r) => rest = r,
                None => return false,
            }
        }
    }

    /// Write the node's slash path (`ch/1/fdr`, root = ``) without allocating.
    pub fn write_path<W: fmt::Write>(&self, id: NodeId, w: &mut W) -> fmt::Result {
        match self.parent(id) {
            None => Ok(()),
            Some(NodeId(0)) => w.write_str(self.name(id)),
            Some(p) => {
                self.write_path(p, w)?;
                w.write_char('/')?;
                w.write_str(self.name(id))
            }
        }
    }

    /// Convenience: the node's path as a `String` (allocates).
    pub fn path(&self, id: NodeId) -> String {
        let mut s = String::new();
        self.write_path(id, &mut s).unwrap();
        s
    }

    /// Model-dependent block groups.
    pub fn groups(&self) -> &[Group] {
        &self.groups
    }

    /// Models of a group.
    pub fn models(&self, g: &Group) -> &[Model] {
        &self.models[g.models.start as usize..g.models.end as usize]
    }

    /// `(target child name, descriptor)` pairs a model defines.
    pub fn model_params<'a>(
        &'a self,
        m: &Model,
    ) -> impl Iterator<Item = (&'static str, &'a Desc)> + 'a {
        self.entries[m.entries.start as usize..m.entries.end as usize]
            .iter()
            .map(|e| (e.target, &self.descs[e.desc as usize]))
    }

    /// The group of the model-dependent block `block` (first group if it has several).
    pub fn group_of_block(&self, block: NodeId) -> Option<&Group> {
        match self.block_inst[block.index()] {
            NONE => None,
            k => Some(&self.groups[self.insts[k as usize].group as usize]),
        }
    }
}

pub(crate) fn trim(path: &str) -> &str {
    path.trim_matches('/')
}

/// Iterator over a node's children.
pub struct Children<'a> {
    s: &'a Schema,
    next: u16,
}

impl Iterator for Children<'_> {
    type Item = NodeId;
    #[inline]
    fn next(&mut self) -> Option<NodeId> {
        if self.next == NONE {
            return None;
        }
        let id = NodeId(self.next);
        self.next = self.s.nodes[id.index()].next_sibling;
        Some(id)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decodes_full_tree() {
        let s = Schema::get();
        assert_eq!(s.node_count(), 43_008);
        assert_eq!(s.param_count(), 36_499);
        let fdr = s.by_path("/ch/1/fdr").unwrap();
        assert_eq!(s.hash(fdr), 0x38AE_75C2);
        assert_eq!(s.path(fdr), "ch/1/fdr");
        assert_eq!(s.by_hash(0x38AE_75C2), Some(fdr));
        assert_eq!(s.by_path("ch/1/nope"), None);
        assert_eq!(s.by_path(""), Some(NodeId::ROOT));
        assert_eq!(s.long_name(s.by_path("io/in/A").unwrap()), "AES50 A");
    }

    #[test]
    fn children_walk_covers_every_node_once() {
        let s = Schema::get();
        let mut seen = vec![false; s.node_count()];
        let mut stack = vec![NodeId::ROOT];
        while let Some(id) = stack.pop() {
            assert!(!seen[id.index()]);
            seen[id.index()] = true;
            for c in s.children(id) {
                assert_eq!(s.parent(c), Some(id));
                stack.push(c);
            }
        }
        assert!(seen.iter().all(|&x| x));
    }

    #[test]
    fn eq_selectors_per_strip_type() {
        // docs/research/models.md §4: bus/main/mtx offer PIA in place of MACH4, aux has neither.
        let s = Schema::get();
        let names = |p: &str| -> Vec<&str> {
            let g = s.group_of_block(s.by_path(p).unwrap()).unwrap();
            s.models(g).iter().map(|m| m.name).collect()
        };
        assert_eq!(
            names("ch/1/eq"),
            ["STD", "SOUL", "E88", "E84", "F110", "PULSAR", "MACH4"]
        );
        assert_eq!(
            names("aux/1/eq"),
            ["STD", "SOUL", "E88", "E84", "F110", "PULSAR"]
        );
        for b in ["bus/3/eq", "main/2/eq", "mtx/8/eq"] {
            assert_eq!(
                names(b),
                ["STD", "SOUL", "E88", "E84", "F110", "PULSAR", "PIA"]
            );
        }
        assert_eq!(
            s.models(s.group_of_block(s.by_path("fx/12").unwrap()).unwrap())
                .len(),
            39
        );
        assert_eq!(
            s.models(s.group_of_block(s.by_path("fx/8").unwrap()).unwrap())
                .len(),
            65
        );
    }

    #[test]
    fn numeric_band_params_present() {
        // '31' (PIA) and '40' (MACH4/GEQ) for EQ and FX blocks; FX ones are filled from models.md.
        let s = Schema::get();
        let find = |block: &str, model: &str, name: &str| {
            let g = s.group_of_block(s.by_path(block).unwrap()).unwrap();
            let m = s.models(g).iter().find(|m| m.name == model).unwrap();
            s.model_params(m)
                .find(|(_, d)| d.name == name)
                .map(|(t, d)| (t, *d))
        };
        let (t, d) = find("ch/1/eq", "MACH4", "40").unwrap();
        assert_eq!((t, d.min, d.max, d.from_models_md), ("2", -5.0, 5.0, false));
        let (t, d) = find("bus/1/eq", "PIA", "31").unwrap();
        assert_eq!(
            (t, d.min, d.max, d.from_models_md),
            ("2", -12.0, 12.0, false)
        );
        for fx in ["fx/1", "fx/16"] {
            let (t, d) = find(fx, "PIA", "31").unwrap();
            assert_eq!(
                (t, d.min, d.max, d.from_models_md),
                ("3", -12.0, 12.0, true)
            );
            let (t, d) = find(fx, "MACH4", "40").unwrap();
            assert_eq!((t, d.min, d.max, d.from_models_md), ("3", -5.0, 5.0, true));
            let (t, d) = find(fx, "GEQ", "40").unwrap();
            assert_eq!(
                (t, d.min, d.max, d.from_models_md),
                ("5", -15.0, 15.0, true)
            );
            assert_eq!(find(fx, "*MASTER*", "40").unwrap().0, "7");
        }
    }
}
