//! One-block-delayed taps for mix outputs used as channel sources (re-inputs such as an FX-return
//! channel fed from `BUS.1`, or a user signal of a strip).
//!
//! The graph is acyclic, so a channel cannot read a bus in the same cycle. The
//! [`FeedbackWriter`] node (scheduled after every mixing node) copies the taps the current
//! routing table uses as channel sources into a shared buffer; the [`super::InputRouter`] reads
//! them at the start of the next cycle. Re-inputs therefore carry one block of latency (as the
//! signal-flow notes anticipate for feedback paths [I]).
//!
//! # Safety model
//!
//! The buffer is written only by the writer node and read only by the input router. The writer
//! depends on the router's output (it is added with the router as an input), so within a cycle the
//! two run in different levels, ordered by the scheduler's level barrier; across cycles the
//! driver waits for the last level before it publishes the next epoch. A reader and the writer
//! therefore never touch the buffer concurrently.

use std::cell::UnsafeCell;
use std::sync::Arc;

use crate::graph::{NodeIo, Port, ProcessCtx, Processor};

use super::addr::{TapId, TAP_COUNT};
use super::nodes::TapPorts;
use super::table::{Feed, INPUT_STRIPS};
use super::RoutingReader;

struct Shared {
    data: Box<[UnsafeCell<f32>]>,
    block: usize,
}

// SAFETY: see the module docs: the scheduler orders every write before every read and vice versa.
unsafe impl Sync for Shared {}
// SAFETY: plain sample storage.
unsafe impl Send for Shared {}

/// Shared storage for one block of every tap (`TAP_COUNT x block` samples).
#[derive(Clone)]
pub struct TapFeedback(Arc<Shared>);

impl TapFeedback {
    /// Allocate (control thread).
    pub fn new(block: usize) -> Self {
        let data = (0..TAP_COUNT * block)
            .map(|_| UnsafeCell::new(0.0))
            .collect();
        Self(Arc::new(Shared { data, block }))
    }

    pub fn block_size(&self) -> usize {
        self.0.block
    }

    /// Last cycle's samples of tap `id` (the input router's view).
    #[inline]
    pub(crate) fn get(&self, id: TapId) -> Option<&[f32]> {
        let i = id.0 as usize;
        if i >= TAP_COUNT {
            return None;
        }
        let b = self.0.block;
        // SAFETY: in range; never concurrent with the writer (module docs).
        Some(unsafe { std::slice::from_raw_parts(self.0.data[i * b].get(), b) })
    }

    /// Mutable slice of tap `id` (the writer's view).
    ///
    /// # Safety
    /// Only the [`FeedbackWriter`] may call this, from its `process`.
    #[allow(clippy::mut_from_ref)]
    #[inline]
    unsafe fn get_mut(&self, i: usize) -> &mut [f32] {
        let b = self.0.block;
        std::slice::from_raw_parts_mut(self.0.data[i * b].get(), b)
    }
}

/// Copies the taps used as channel sources into a [`TapFeedback`].
///
/// Inputs: port 0 = any input-router output (orders the writer after the router), then every wired
/// tap. No outputs are needed; the node has one dummy output.
pub struct FeedbackWriter {
    fb: TapFeedback,
    reader: RoutingReader,
    /// Input index per tap, `u16::MAX` = not wired.
    tap_input: Box<[u16]>,
    used: Box<[bool]>,
    primed: bool,
}

impl FeedbackWriter {
    /// Writer plus its input list. `order_after` is a port of the input router.
    pub fn new(
        fb: TapFeedback,
        reader: RoutingReader,
        order_after: Port,
        taps: &TapPorts,
    ) -> (Self, Vec<Port>) {
        let mut inputs = vec![order_after];
        let mut tap_input = vec![u16::MAX; TAP_COUNT].into_boxed_slice();
        for (i, p) in taps.iter().enumerate() {
            if let Some(p) = p {
                tap_input[i] = inputs.len() as u16;
                inputs.push(*p);
            }
        }
        let w = Self {
            fb,
            reader,
            tap_input,
            used: vec![false; TAP_COUNT].into_boxed_slice(),
            primed: false,
        };
        (w, inputs)
    }

    /// Recompute which taps the routing table reads back (and silence the others).
    fn refresh_used(&mut self) {
        self.used.fill(false);
        let t = self.reader.get();
        let used = &mut self.used;
        let mut mark = |i: usize| {
            if let Some(u) = used.get_mut(i) {
                *u = true;
            }
        };
        for i in 0..INPUT_STRIPS {
            for leg in t.input(i).legs {
                match leg.feed {
                    Feed::Tap(id) => mark(id.0 as usize),
                    Feed::TapMono(id) => {
                        mark(id.0 as usize);
                        mark(id.0 as usize + 1);
                    }
                    _ => {}
                }
            }
        }
        for i in 0..TAP_COUNT {
            if !self.used[i] {
                // SAFETY: writer-only access (module docs).
                unsafe { self.fb.get_mut(i) }.fill(0.0);
            }
        }
    }

    /// Whether tap `id` is currently copied.
    pub fn is_used(&self, id: TapId) -> bool {
        self.used.get(id.0 as usize).copied().unwrap_or(false)
    }
}

impl Processor for FeedbackWriter {
    fn process(&mut self, _ctx: &ProcessCtx, io: &mut NodeIo<'_>) {
        if self.reader.update() || !self.primed {
            self.primed = true;
            self.refresh_used();
        }
        let ins = io.inputs();
        for i in 0..TAP_COUNT {
            if !self.used[i] {
                continue;
            }
            // SAFETY: writer-only access (module docs).
            let dst = unsafe { self.fb.get_mut(i) };
            match self.tap_input[i] {
                u16::MAX => dst.fill(0.0),
                k => dst.copy_from_slice(&ins.get(k as usize)[..dst.len()]),
            }
        }
        io.output(0).fill(0.0);
    }
}
