//! Off-RT construction of the `wing-model node -> (graph node, block, param)` dispatch tables.

use std::fmt;

use wing_model::{Desc, NodeId as ParamId, Schema};

use crate::graph::NodeId;

/// Parameter ids at or above this value address model targets (`TARGET_BASE + index` into
/// [`BlockLayout::targets`]).
pub const TARGET_BASE: u16 = 0x100;

pub(crate) const NONE: u32 = u32::MAX;
/// "No model selected yet" marker for an instance's current model.
pub(crate) const NO_MODEL: u16 = u16::MAX;

/// Which children of a wing-model block a processor block consumes.
#[derive(Debug, Clone, Copy)]
pub struct BlockLayout {
    /// Model-independent children: `(child name, block-local param id < TARGET_BASE)`.
    pub params: &'static [(&'static str, u16)],
    /// Model-dependent children (targets of the block's model group), numbered
    /// `TARGET_BASE + index`. Non-empty binds the group's selector too; empty leaves the block's
    /// model group (if any) unbound.
    pub targets: &'static [&'static str],
}

/// Why a binding was rejected (off-RT, so errors carry text).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum BindError {
    UnknownPath(String),
    UnknownChild(String),
    NotParam(String),
    AlreadyBound(String),
    NoModelGroup(String),
    TooManyBlocks,
}

impl fmt::Display for BindError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            BindError::UnknownPath(p) => write!(f, "unknown block path {p}"),
            BindError::UnknownChild(p) => write!(f, "unknown child {p}"),
            BindError::NotParam(p) => write!(f, "{p} is not a parameter"),
            BindError::AlreadyBound(p) => write!(f, "{p} is already bound"),
            BindError::NoModelGroup(p) => write!(f, "{p} has no model group"),
            BindError::TooManyBlocks => f.write_str("too many bindings"),
        }
    }
}

impl std::error::Error for BindError {}

#[derive(Clone, Copy, Debug)]
pub(crate) enum Kind {
    Plain { param: u16 },
    Selector { inst: u32 },
    Target { inst: u32, pos: u16 },
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct Binding {
    pub node: u32,
    pub block: u8,
    pub kind: Kind,
}

/// `(model enum index, target position) -> expected descriptor / default` for one model group
/// and target list.
pub(crate) struct ModelTable {
    key: &'static str,
    targets: &'static [&'static str],
    pub n_targets: usize,
    pub n_models: usize,
    pub valid: Box<[bool]>,
    pub expected: Box<[Option<&'static Desc>]>,
    pub defaults: Box<[u32]>,
}

impl ModelTable {
    #[inline]
    pub fn expected(&self, model: u16, pos: usize) -> Option<&'static Desc> {
        let m = model as usize;
        if m >= self.n_models {
            return None;
        }
        self.expected[m * self.n_targets + pos]
    }

    #[inline]
    pub fn default(&self, model: u16, pos: usize) -> u32 {
        self.defaults[model as usize * self.n_targets + pos]
    }

    #[inline]
    pub fn is_valid(&self, model: u32) -> bool {
        (model as usize) < self.n_models && self.valid[model as usize]
    }
}

/// One bound model-dependent block instance.
pub(crate) struct ModelInst {
    pub node: u32,
    pub block: u8,
    pub table: u32,
    /// Offset of this instance's targets in the mirror arrays.
    pub base: u32,
    pub selector: ParamId,
}

/// Compiled dispatch tables (see [`ParamMapBuilder`]).
pub struct ParamMap {
    pub(crate) schema: &'static Schema,
    pub(crate) node_map: Box<[u32]>,
    pub(crate) bindings: Box<[Binding]>,
    pub(crate) insts: Box<[ModelInst]>,
    pub(crate) tables: Box<[ModelTable]>,
    /// wing-model id of every mirrored target (parallel to the mirror arrays).
    pub(crate) mirror_ids: Box<[ParamId]>,
    /// Every bound wing-model node (plain, selector, target), for full syncs.
    pub(crate) bound: Box<[ParamId]>,
    pub(crate) telemetry_nodes: Box<[NodeId]>,
    /// 1 + highest graph node index referenced.
    pub(crate) num_graph_nodes: usize,
}

impl ParamMap {
    /// Number of bound wing-model parameters.
    pub fn num_bound(&self) -> usize {
        self.bound.len()
    }

    /// Graph node, block and param id a wing-model node is bound to (`param` is `TARGET_BASE +
    /// pos` for targets, `None` for selectors).
    pub fn binding_of(&self, id: ParamId) -> Option<(NodeId, u8, Option<u16>)> {
        let b = self
            .node_map
            .get(id.index())
            .copied()
            .filter(|&b| b != NONE)?;
        let b = self.bindings[b as usize];
        let p = match b.kind {
            Kind::Plain { param } => Some(param),
            Kind::Selector { .. } => None,
            Kind::Target { pos, .. } => Some(TARGET_BASE + pos),
        };
        Some((NodeId(b.node), b.block, p))
    }
}

/// Builds a [`ParamMap`] on a control thread.
pub struct ParamMapBuilder {
    schema: &'static Schema,
    node_map: Vec<u32>,
    bindings: Vec<Binding>,
    insts: Vec<ModelInst>,
    tables: Vec<ModelTable>,
    mirror_ids: Vec<ParamId>,
    bound: Vec<ParamId>,
    telemetry: Vec<NodeId>,
    max_node: usize,
}

impl Default for ParamMapBuilder {
    fn default() -> Self {
        Self::new()
    }
}

impl ParamMapBuilder {
    pub fn new() -> Self {
        let schema = Schema::get();
        Self {
            schema,
            node_map: vec![NONE; schema.node_count()],
            bindings: Vec::new(),
            insts: Vec::new(),
            tables: Vec::new(),
            mirror_ids: Vec::new(),
            bound: Vec::new(),
            telemetry: Vec::new(),
            max_node: 0,
        }
    }

    fn bind(&mut self, id: ParamId, b: Binding) -> Result<(), BindError> {
        if !self.schema.is_param(id) {
            return Err(BindError::NotParam(self.schema.path(id)));
        }
        if self.node_map[id.index()] != NONE {
            return Err(BindError::AlreadyBound(self.schema.path(id)));
        }
        if self.bindings.len() >= NONE as usize {
            return Err(BindError::TooManyBlocks);
        }
        self.node_map[id.index()] = self.bindings.len() as u32;
        self.bindings.push(b);
        self.bound.push(id);
        self.max_node = self.max_node.max(b.node as usize + 1);
        Ok(())
    }

    fn child(&self, block: ParamId, path: &str, name: &str) -> Result<ParamId, BindError> {
        self.schema
            .child(block, name)
            .ok_or_else(|| BindError::UnknownChild(format!("{path}/{name}")))
    }

    /// Bind the wing-model block at `path` (e.g. `ch/1/dyn`) to block index `block` of graph node
    /// `node`, consuming the children listed in `layout`.
    pub fn bind_block(
        &mut self,
        path: &str,
        node: NodeId,
        block: u8,
        layout: &BlockLayout,
    ) -> Result<(), BindError> {
        let s = self.schema;
        let blk = s
            .by_path(path)
            .ok_or_else(|| BindError::UnknownPath(path.to_string()))?;
        for &(name, param) in layout.params {
            debug_assert!(param < TARGET_BASE);
            let id = self.child(blk, path, name)?;
            self.bind(
                id,
                Binding {
                    node: node.0,
                    block,
                    kind: Kind::Plain { param },
                },
            )?;
        }
        if layout.targets.is_empty() {
            return Ok(());
        }
        let group = s
            .group_of_block(blk)
            .ok_or_else(|| BindError::NoModelGroup(path.to_string()))?;
        let table = self.table_for(group.key, layout.targets)?;
        let selector = self.child(blk, path, group.selector)?;
        let inst = self.insts.len() as u32;
        let base = self.mirror_ids.len() as u32;
        self.bind(
            selector,
            Binding {
                node: node.0,
                block,
                kind: Kind::Selector { inst },
            },
        )?;
        for (pos, &t) in layout.targets.iter().enumerate() {
            let id = self.child(blk, path, t)?;
            self.bind(
                id,
                Binding {
                    node: node.0,
                    block,
                    kind: Kind::Target {
                        inst,
                        pos: pos as u16,
                    },
                },
            )?;
            self.mirror_ids.push(id);
        }
        self.insts.push(ModelInst {
            node: node.0,
            block,
            table,
            base,
            selector,
        });
        Ok(())
    }

    fn table_for(
        &mut self,
        key: &'static str,
        targets: &'static [&'static str],
    ) -> Result<u32, BindError> {
        if let Some(i) = self
            .tables
            .iter()
            .position(|t| t.key == key && t.targets == targets)
        {
            return Ok(i as u32);
        }
        let s = self.schema;
        let group = s
            .groups()
            .iter()
            .find(|g| g.key == key)
            .ok_or_else(|| BindError::NoModelGroup(key.to_string()))?;
        let models = s.models(group);
        let n_models = models
            .iter()
            .map(|m| m.enum_index as usize + 1)
            .max()
            .unwrap_or(0);
        let n = targets.len();
        let mut valid = vec![false; n_models];
        let mut expected = vec![None; n_models * n];
        let mut defaults = vec![0u32; n_models * n];
        for m in models {
            let e = m.enum_index as usize;
            valid[e] = true;
            for (pos, &t) in targets.iter().enumerate() {
                if let Some((_, d)) = s.model_params(m).find(|(name, _)| *name == t) {
                    if !d.unused_slot {
                        expected[e * n + pos] = Some(d);
                        defaults[e * n + pos] = d.default_raw;
                    }
                }
            }
        }
        self.tables.push(ModelTable {
            key,
            targets,
            n_targets: n,
            n_models,
            valid: valid.into(),
            expected: expected.into(),
            defaults: defaults.into(),
        });
        Ok(self.tables.len() as u32 - 1)
    }

    /// Register `node` as a telemetry source (its `Processor::telemetry` is polled).
    pub fn telemetry_node(&mut self, node: NodeId) {
        if !self.telemetry.contains(&node) {
            self.telemetry.push(node);
            self.max_node = self.max_node.max(node.index() + 1);
        }
    }

    pub fn build(self) -> ParamMap {
        ParamMap {
            schema: self.schema,
            node_map: self.node_map.into(),
            bindings: self.bindings.into(),
            insts: self.insts.into(),
            tables: self.tables.into(),
            mirror_ids: self.mirror_ids.into(),
            bound: self.bound.into(),
            telemetry_nodes: self.telemetry.into(),
            num_graph_nodes: self.max_node,
        }
    }
}
