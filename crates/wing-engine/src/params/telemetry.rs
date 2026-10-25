//! RT -> control telemetry: fixed-capacity lock-free ring, single producer (the RT driver
//! thread) and single consumer (a control thread). No allocation after construction.

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering::*};
use std::sync::Arc;

use wing_model::BoundedQueue;

/// Telemetry record kinds.
pub mod kind {
    /// Gain reduction in dB (>= 0) of a dynamics/gate slot.
    pub const GAIN_REDUCTION_DB: u8 = 0;
}

/// One telemetry record (16 bytes, `Copy`).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Telemetry {
    /// Graph node index of the source.
    pub node: u32,
    /// Block within the node (see the processor's block constants).
    pub block: u8,
    /// One of [`kind`].
    pub kind: u8,
    pub value: f32,
    /// Engine cycle (low 32 bits) the value belongs to.
    pub cycle: u32,
}

pub(crate) struct Shared {
    pub queue: BoundedQueue<Telemetry>,
    pub dropped: AtomicU64,
    pub rx_taken: AtomicBool,
}

impl Shared {
    pub fn new(capacity: usize) -> Arc<Self> {
        Arc::new(Self {
            queue: BoundedQueue::new(capacity),
            dropped: AtomicU64::new(0),
            rx_taken: AtomicBool::new(false),
        })
    }
}

/// Handed to [`crate::Processor::telemetry`] for one node.
pub struct TelemetryWriter<'a> {
    pub(crate) shared: &'a Shared,
    pub(crate) node: u32,
    pub(crate) cycle: u32,
}

impl TelemetryWriter<'_> {
    /// Publish a value. Lock-free, allocation-free; on a full ring the record is dropped and
    /// counted.
    #[inline]
    pub fn push(&mut self, block: u8, kind: u8, value: f32) {
        let t = Telemetry {
            node: self.node,
            block,
            kind,
            value,
            cycle: self.cycle,
        };
        if self.shared.queue.push(t).is_err() {
            self.shared.dropped.fetch_add(1, Relaxed);
        }
    }
}

/// Control-side end of the telemetry ring (one per pipeline).
pub struct TelemetryRx {
    pub(crate) shared: Arc<Shared>,
}

impl TelemetryRx {
    /// Next record, oldest first.
    #[inline]
    pub fn pop(&mut self) -> Option<Telemetry> {
        self.shared.queue.pop()
    }

    /// Records dropped because the ring was full.
    pub fn dropped(&self) -> u64 {
        self.shared.dropped.load(Relaxed)
    }
}
