//! Block-processing graph: nodes with fixed planar `f32` I/O, a preallocated buffer arena and
//! dependency levels.
//!
//! A graph is built once on a control thread with [`GraphBuilder`] and then compiled into a
//! [`Graph`]. Compilation allocates every buffer the graph will ever touch (one contiguous
//! arena: external inputs first, then each node's output channels) and precomputes, per node,
//! the raw pointers to its input channels. Processing a node therefore never allocates.
//!
//! Nodes may only consume ports of nodes added *before* them, so the graph is acyclic by
//! construction. Each node's level is `1 + max(level of its inputs)` (nodes reading only
//! external inputs, or nothing, are level 0). All nodes in a level are independent and may run
//! in parallel; level `n + 1` starts only after level `n` has completed.

use std::cell::UnsafeCell;
use std::fmt;
use std::marker::PhantomData;

/// Index of a node in a graph (insertion order).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct NodeId(pub u32);

impl NodeId {
    /// Index as `usize`.
    pub fn index(self) -> usize {
        self.0 as usize
    }
}

/// A single planar audio channel that a node can read.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Port {
    /// External input channel (filled by the clock/host before each cycle).
    External(u32),
    /// Output channel `ch` of node `node`.
    Node {
        /// Producing node.
        node: NodeId,
        /// Output channel of that node.
        ch: u32,
    },
}

impl Port {
    /// Shorthand for `Port::Node { node, ch }`.
    pub fn node(node: NodeId, ch: u32) -> Self {
        Port::Node { node, ch }
    }
}

/// Per-cycle context passed to every node.
#[derive(Debug, Clone, Copy)]
pub struct ProcessCtx {
    /// Monotonic cycle counter (starts at 1).
    pub cycle: u64,
    /// Samples per channel per cycle.
    pub block_size: usize,
    /// Sample rate in Hz.
    pub sample_rate: f32,
}

/// A processing node. Implementations run on RT threads: they must not allocate, lock, log or
/// make syscalls in [`Processor::process`].
pub trait Processor: Send {
    /// Process one block: read `io.input(i)`, write every `io.output(j)`.
    fn process(&mut self, ctx: &ProcessCtx, io: &mut NodeIo<'_>);
}

impl<F: FnMut(&ProcessCtx, &mut NodeIo<'_>) + Send> Processor for F {
    fn process(&mut self, ctx: &ProcessCtx, io: &mut NodeIo<'_>) {
        self(ctx, io)
    }
}

/// Borrowed view of one node's input and output channels for one block.
pub struct NodeIo<'a> {
    inputs: &'a [*const f32],
    outputs: *mut f32,
    num_outputs: usize,
    block: usize,
    _marker: PhantomData<&'a mut f32>,
}

impl<'a> NodeIo<'a> {
    /// Number of input channels.
    #[inline]
    pub fn num_inputs(&self) -> usize {
        self.inputs.len()
    }

    /// Number of output channels.
    #[inline]
    pub fn num_outputs(&self) -> usize {
        self.num_outputs
    }

    /// Samples per channel.
    #[inline]
    pub fn block_size(&self) -> usize {
        self.block
    }

    /// Input channel `i` (panics if out of range).
    #[inline]
    pub fn input(&self, i: usize) -> &'a [f32] {
        let p = self.inputs[i];
        // SAFETY: the pointer addresses `block` samples inside the graph arena that belong to an
        // external input or to a node in an earlier level; the scheduler guarantees nobody writes
        // them while this node runs. Inputs never alias this node's outputs (acyclic graph).
        unsafe { std::slice::from_raw_parts(p, self.block) }
    }

    /// All inputs as a copyable list (usable while holding an output borrow).
    #[inline]
    pub fn inputs(&self) -> InputList<'a> {
        InputList {
            ptrs: self.inputs,
            block: self.block,
        }
    }

    /// Output channel `j` (panics if out of range).
    #[inline]
    pub fn output(&mut self, j: usize) -> &mut [f32] {
        assert!(j < self.num_outputs, "output channel out of range");
        // SAFETY: this node exclusively owns its output channels during `process`.
        unsafe { std::slice::from_raw_parts_mut(self.outputs.add(j * self.block), self.block) }
    }

    /// All output channels as one planar slice (`num_outputs * block_size` samples).
    #[inline]
    pub fn outputs(&mut self) -> &mut [f32] {
        // SAFETY: as for `output`.
        unsafe { std::slice::from_raw_parts_mut(self.outputs, self.num_outputs * self.block) }
    }
}

/// A node's input channels, detached from the `NodeIo` borrow.
#[derive(Clone, Copy)]
pub struct InputList<'a> {
    ptrs: &'a [*const f32],
    block: usize,
}

impl<'a> InputList<'a> {
    /// Number of input channels.
    #[inline]
    pub fn len(&self) -> usize {
        self.ptrs.len()
    }

    /// True if there are no inputs.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.ptrs.is_empty()
    }

    /// Input channel `i` (panics if out of range).
    #[inline]
    pub fn get(&self, i: usize) -> &'a [f32] {
        // SAFETY: see `NodeIo::input`.
        unsafe { std::slice::from_raw_parts(self.ptrs[i], self.block) }
    }
}

/// Error from [`GraphBuilder`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum GraphError {
    /// A port referenced a node that was not yet added (would allow cycles) or does not exist.
    UnknownNode(NodeId),
    /// A port referenced a channel that its node/external input does not have.
    BadChannel(Port),
    /// `block_size` was zero.
    ZeroBlockSize,
}

impl fmt::Display for GraphError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            GraphError::UnknownNode(n) => write!(f, "unknown or later node {n:?}"),
            GraphError::BadChannel(p) => write!(f, "channel out of range: {p:?}"),
            GraphError::ZeroBlockSize => write!(f, "block size must be > 0"),
        }
    }
}

impl std::error::Error for GraphError {}

struct PendingNode {
    name: String,
    proc: Box<dyn Processor>,
    inputs: Vec<Port>,
    num_outputs: u32,
    level: u32,
}

/// Builds a [`Graph`].
pub struct GraphBuilder {
    block_size: usize,
    sample_rate: f32,
    num_external: u32,
    nodes: Vec<PendingNode>,
}

impl GraphBuilder {
    /// New builder with `num_external` external input channels.
    pub fn new(block_size: usize, sample_rate: f32, num_external: u32) -> Self {
        Self {
            block_size,
            sample_rate,
            num_external,
            nodes: Vec::new(),
        }
    }

    /// Add a node reading `inputs` and producing `num_outputs` planar channels.
    pub fn add_node(
        &mut self,
        name: impl Into<String>,
        num_outputs: u32,
        inputs: &[Port],
        proc: impl Processor + 'static,
    ) -> Result<NodeId, GraphError> {
        self.add_boxed(name, num_outputs, inputs, Box::new(proc))
    }

    /// As [`GraphBuilder::add_node`] for an already boxed processor.
    pub fn add_boxed(
        &mut self,
        name: impl Into<String>,
        num_outputs: u32,
        inputs: &[Port],
        proc: Box<dyn Processor>,
    ) -> Result<NodeId, GraphError> {
        let mut level = 0u32;
        for &p in inputs {
            match p {
                Port::External(ch) => {
                    if ch >= self.num_external {
                        return Err(GraphError::BadChannel(p));
                    }
                }
                Port::Node { node, ch } => {
                    let n = self
                        .nodes
                        .get(node.index())
                        .ok_or(GraphError::UnknownNode(node))?;
                    if ch >= n.num_outputs {
                        return Err(GraphError::BadChannel(p));
                    }
                    level = level.max(n.level + 1);
                }
            }
        }
        let id = NodeId(self.nodes.len() as u32);
        self.nodes.push(PendingNode {
            name: name.into(),
            proc,
            inputs: inputs.to_vec(),
            num_outputs,
            level,
        });
        Ok(id)
    }

    /// Level a node was assigned (for inspection/tests).
    pub fn level_of(&self, id: NodeId) -> Option<u32> {
        self.nodes.get(id.index()).map(|n| n.level)
    }

    /// Compile into a runnable graph, preallocating every buffer.
    pub fn build(self) -> Result<Graph, GraphError> {
        if self.block_size == 0 {
            return Err(GraphError::ZeroBlockSize);
        }
        let b = self.block_size;
        // Arena layout: [external inputs][node 0 outputs][node 1 outputs]...
        let mut out_offset = Vec::with_capacity(self.nodes.len());
        let mut total = self.num_external as usize * b;
        for n in &self.nodes {
            out_offset.push(total);
            total += n.num_outputs as usize * b;
        }
        let arena: Box<[UnsafeCell<f32>]> = (0..total).map(|_| UnsafeCell::new(0.0)).collect();
        let base = arena.as_ptr() as *mut f32;

        let num_levels = self.nodes.iter().map(|n| n.level + 1).max().unwrap_or(0) as usize;
        // Stable order: by level, then insertion order.
        let mut order: Vec<usize> = (0..self.nodes.len()).collect();
        order.sort_by_key(|&i| (self.nodes[i].level, i));
        let mut counts = vec![0usize; num_levels];
        for n in &self.nodes {
            counts[n.level as usize] += 1;
        }
        let mut start = 0;
        let levels: Vec<(usize, usize)> = counts
            .iter()
            .map(|&c| {
                let r = (start, start + c);
                start += c;
                r
            })
            .collect();

        let mut slots: Vec<Option<NodeSlot>> = Vec::with_capacity(self.nodes.len());
        for (idx, n) in self.nodes.into_iter().enumerate() {
            let inputs: Box<[*const f32]> = n
                .inputs
                .iter()
                .map(|p| {
                    let off = match *p {
                        Port::External(ch) => ch as usize * b,
                        Port::Node { node, ch } => out_offset[node.index()] + ch as usize * b,
                    };
                    // SAFETY: offset within arena (validated above).
                    unsafe { base.add(off) as *const f32 }
                })
                .collect();
            slots.push(Some(NodeSlot {
                name: n.name,
                proc: UnsafeCell::new(n.proc),
                ports: n.inputs.into_boxed_slice(),
                inputs,
                // SAFETY: offset within arena.
                outputs: unsafe { base.add(out_offset[idx]) },
                num_outputs: n.num_outputs as usize,
                level: n.level,
            }));
        }
        let slots: Box<[NodeSlot]> = order.iter().map(|&i| slots[i].take().unwrap()).collect();
        let mut position = vec![0usize; slots.len()];
        for (pos, &i) in order.iter().enumerate() {
            position[i] = pos;
        }

        Ok(Graph {
            block_size: b,
            sample_rate: self.sample_rate,
            num_external: self.num_external as usize,
            arena,
            slots,
            position: position.into_boxed_slice(),
            levels: levels.into_boxed_slice(),
        })
    }
}

pub(crate) struct NodeSlot {
    name: String,
    proc: UnsafeCell<Box<dyn Processor>>,
    ports: Box<[Port]>,
    inputs: Box<[*const f32]>,
    outputs: *mut f32,
    num_outputs: usize,
    level: u32,
}

/// A compiled, runnable graph. Nodes are stored in schedule order (by level).
pub struct Graph {
    block_size: usize,
    sample_rate: f32,
    num_external: usize,
    arena: Box<[UnsafeCell<f32>]>,
    slots: Box<[NodeSlot]>,
    /// NodeId index -> schedule position.
    position: Box<[usize]>,
    /// Per level: half-open range of schedule positions.
    levels: Box<[(usize, usize)]>,
}

// SAFETY: the graph is shared between the driver and worker threads. Mutable state (processors
// and arena samples) is only accessed under the scheduler protocol: each node is claimed by exactly
// one thread per cycle, it writes only its own outputs, and reads only buffers of earlier levels,
// with happens-before established by the level barrier atomics.
unsafe impl Sync for Graph {}
// SAFETY: processors are `Send`; raw pointers point into the owned arena.
unsafe impl Send for Graph {}

impl Graph {
    /// Samples per channel per cycle.
    pub fn block_size(&self) -> usize {
        self.block_size
    }

    /// Sample rate in Hz.
    pub fn sample_rate(&self) -> f32 {
        self.sample_rate
    }

    /// Number of nodes.
    pub fn num_nodes(&self) -> usize {
        self.slots.len()
    }

    /// Number of external input channels.
    pub fn num_external(&self) -> usize {
        self.num_external
    }

    /// Number of dependency levels.
    pub fn num_levels(&self) -> usize {
        self.levels.len()
    }

    /// Schedule-position range of level `l`.
    pub fn level_range(&self, l: usize) -> std::ops::Range<usize> {
        let (a, b) = self.levels[l];
        a..b
    }

    /// Level of a node.
    pub fn level_of(&self, id: NodeId) -> u32 {
        self.slots[self.position[id.index()]].level
    }

    /// Name of a node.
    pub fn name_of(&self, id: NodeId) -> &str {
        &self.slots[self.position[id.index()]].name
    }

    /// Input ports of a node.
    pub fn inputs_of(&self, id: NodeId) -> &[Port] {
        &self.slots[self.position[id.index()]].ports
    }

    /// Node ids in schedule order.
    pub fn schedule(&self) -> impl Iterator<Item = NodeId> + '_ {
        let mut inv = vec![NodeId(0); self.position.len()];
        for (id, &pos) in self.position.iter().enumerate() {
            inv[pos] = NodeId(id as u32);
        }
        inv.into_iter()
    }

    fn port_ptr(&self, p: Port) -> *mut f32 {
        let base = self.arena.as_ptr() as *mut f32;
        let off = match p {
            Port::External(ch) => {
                assert!(
                    (ch as usize) < self.num_external,
                    "external channel out of range"
                );
                ch as usize * self.block_size
            }
            Port::Node { node, ch } => {
                let s = &self.slots[self.position[node.index()]];
                assert!((ch as usize) < s.num_outputs, "node channel out of range");
                // SAFETY: pointer arithmetic within the arena.
                return unsafe { s.outputs.add(ch as usize * self.block_size) };
            }
        };
        // SAFETY: offset within the arena.
        unsafe { base.add(off) }
    }

    /// Read a port. Requires `&mut self`-level exclusivity at the engine level (no cycle running).
    pub fn port(&self, p: Port) -> &[f32] {
        // SAFETY: callers only reach this through `&Engine`/`&Graph` while no cycle runs.
        unsafe { std::slice::from_raw_parts(self.port_ptr(p), self.block_size) }
    }

    /// Mutable access to an external input channel.
    pub fn external_mut(&mut self, ch: usize) -> &mut [f32] {
        // SAFETY: `&mut self` gives exclusive access.
        unsafe { self.external_shared(ch) }
    }

    /// External input channel through a shared reference.
    ///
    /// # Safety
    /// No cycle may be running and no other reference to this channel may be live.
    #[allow(clippy::mut_from_ref)]
    pub(crate) unsafe fn external_shared(&self, ch: usize) -> &mut [f32] {
        let p = self.port_ptr(Port::External(ch as u32));
        std::slice::from_raw_parts_mut(p, self.block_size)
    }

    /// Mutable access to a node's processor (no cycle may be running).
    pub fn processor_mut(&mut self, id: NodeId) -> &mut dyn Processor {
        let pos = self.position[id.index()];
        self.slots[pos].proc.get_mut().as_mut()
    }

    /// Processor through a shared reference.
    ///
    /// # Safety
    /// No cycle may be running and no other reference to this processor may be live.
    #[allow(clippy::mut_from_ref)]
    pub(crate) unsafe fn processor_shared(&self, id: NodeId) -> &mut dyn Processor {
        let pos = self.position[id.index()];
        (*self.slots[pos].proc.get()).as_mut()
    }

    /// Run the node at schedule position `pos`.
    ///
    /// # Safety
    /// The caller must hold the exclusive claim on `pos` for this cycle, and every node of every
    /// earlier level must have completed (with happens-before) for this cycle.
    #[inline]
    pub(crate) unsafe fn run_node(&self, pos: usize, ctx: &ProcessCtx) {
        let s = self.slots.get_unchecked(pos);
        let mut io = NodeIo {
            inputs: &s.inputs,
            outputs: s.outputs,
            num_outputs: s.num_outputs,
            block: self.block_size,
            _marker: PhantomData,
        };
        (*s.proc.get()).process(ctx, &mut io);
    }

    /// Run every node in schedule order on the calling thread (reference/serial path).
    pub fn run_serial(&mut self, ctx: &ProcessCtx) {
        // SAFETY: `&mut self` makes this thread the only one touching the graph.
        unsafe { self.run_serial_shared(ctx) }
    }

    /// As [`Graph::run_serial`] through a shared reference.
    ///
    /// # Safety
    /// No other thread may be running nodes or touching buffers concurrently.
    pub(crate) unsafe fn run_serial_shared(&self, ctx: &ProcessCtx) {
        for pos in 0..self.slots.len() {
            self.run_node(pos, ctx);
        }
    }
}

impl fmt::Debug for Graph {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Graph")
            .field("block_size", &self.block_size)
            .field("nodes", &self.slots.len())
            .field("levels", &self.levels)
            .finish()
    }
}
