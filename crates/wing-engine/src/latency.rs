//! Latency compensation (T03E): constant per-slot and per-strip latency.
//!
//! WING's DSP runs fixed-latency blocks, so a strip's delay never depends on which model is selected or
//! whether a block is on. The emulation follows that:
//!
//! * **Model slot** ([`crate::params::strip::ModelSlot`]): the slot latency is the *maximum* latency of the
//!   models it can host. One input delay line ([`TapLine`]) feeds the dry path at that latency and each
//!   model at `slot latency - model latency` (its side-chain key likewise), so `mix < 100 %` blends
//!   phase-aligned signals (no comb filtering), model crossfades blend aligned signals, an incoming model
//!   sees continuous input history, and the slot's latency stays the same when the model, `on` or `mix`
//!   changes. Slots whose models are all zero-latency add nothing (bit-exact as before).
//! * **Strip** ([`crate::channel::ChannelStrip`], [`crate::mix::strip::MixStrip`]): the sum of its slot
//!   latencies plus an alignment pad ([`LatencyPad`]) at the strip output. Channel and aux strips pad to
//!   the same *input-strip* latency (aux strips have no filter tool / gate, so they pad by those slots'
//!   latency), bus/main/matrix strips to the *mix-strip* latency. Every source summed into a bus therefore
//!   arrives sample-aligned regardless of strip kind or model choice. Inserts (FX rack, T050) report their
//!   latency to the strip (`set_insert_latency`); the pad shrinks so the total stays constant while the
//!   insert latency fits within the alignment target, and the engine can raise the target with
//!   `set_align_latency`.
//!
//! Model latencies at 48 kHz: 76LA/NSTR (oversampled FET compressors) 57 samples, WARM 39, every other
//! dynamics model and every EQ / filter-tool model 0. Slot latency = max per block kind: gate and
//! dynamics slots (one shared model table) 57, EQ and filter-tool slots 0 (EQ slots keep the WING
//! gain-scale mix law, no dry/wet blend, and add nothing). Added latency:
//!
//! | path | samples | ms |
//! |---|---|---|
//! | channel / aux strip (gate 57 + dyn 57) | 114 | 2.375 |
//! | bus / main / matrix strip (dyn 57) | 57 | 1.1875 |
//! | channel -> bus -> main -> matrix | 114 + 57 + 57 + 57 = 285 | 5.94 |
//!
//! Taps and sends taken inside a strip (PRE/IN/... taps, meters) see the latency accumulated up to that
//! point (not compensated), as on a console without per-tap alignment.

use wing_dsp::delay::DelayLine;

/// Capacity (samples) of a strip's alignment pad: the largest alignment target
/// (`set_align_latency`) minus the strip's own latency that can be honoured.
pub const MAX_ALIGN_SAMPLES: usize = 2048;

/// Multi-tap mono delay line for block processing: [`TapLine::push`] a block, then
/// [`TapLine::read`] that block delayed by any `d <= max_delay` (several reads per push are fine).
/// Preallocated for `max_delay + max_block`; RT-safe.
pub struct TapLine {
    buf: Vec<f32>,
    mask: usize,
    /// Next write index.
    w: usize,
}

impl TapLine {
    /// Allocates.
    pub fn new(max_delay: usize, max_block: usize) -> Self {
        let size = (max_delay + max_block.max(1)).next_power_of_two();
        Self {
            buf: vec![0.0; size],
            mask: size - 1,
            w: 0,
        }
    }

    pub fn reset(&mut self) {
        self.buf.fill(0.0);
    }

    /// Append a block (`len <= max_block`).
    #[inline]
    pub fn push(&mut self, x: &[f32]) {
        for &v in x {
            self.buf[self.w] = v;
            self.w = (self.w + 1) & self.mask;
        }
    }

    /// The last pushed block (`out.len()` samples) delayed by `d` samples: `out[i] = x[t_i - d]`.
    #[inline]
    pub fn read(&self, out: &mut [f32], d: usize) {
        let start = self.w.wrapping_sub(out.len() + d);
        for (i, o) in out.iter_mut().enumerate() {
            *o = self.buf[start.wrapping_add(i) & self.mask];
        }
    }
}

/// Stereo integer-sample delay with preallocated capacity (control thread allocates, RT thread uses).
/// A delay of 0 passes audio through untouched and costs nothing.
pub struct LatencyPad {
    l: DelayLine,
    r: DelayLine,
    delay: usize,
}

impl LatencyPad {
    /// Pad for delays up to `capacity` samples, delay 0. Allocates.
    pub fn new(capacity: usize) -> Self {
        Self {
            l: DelayLine::new(capacity),
            r: DelayLine::new(capacity),
            delay: 0,
        }
    }

    /// Largest delay this pad can apply.
    pub fn capacity(&self) -> usize {
        self.l.max_delay()
    }

    /// Current delay in samples.
    pub fn delay(&self) -> usize {
        self.delay
    }

    /// Set the delay (clamped to the capacity; returns the applied delay). The history is cleared
    /// when the delay changes. RT-safe.
    pub fn set_delay(&mut self, d: usize) -> usize {
        let d = d.min(self.capacity());
        if d != self.delay {
            self.delay = d;
            self.reset();
        }
        d
    }

    /// Clear the history (RT-safe).
    pub fn reset(&mut self) {
        if self.delay > 0 {
            self.l.reset();
            self.r.reset();
        }
    }

    /// Delay a stereo block in place (RT-safe).
    #[inline]
    pub fn process(&mut self, l: &mut [f32], r: &mut [f32]) {
        let d = self.delay;
        if d == 0 {
            return;
        }
        for x in l.iter_mut() {
            self.l.push(*x);
            *x = self.l.read_int(d);
        }
        for x in r.iter_mut() {
            self.r.push(*x);
            *x = self.r.read_int(d);
        }
    }
}

/// A strip's latency bookkeeping and output alignment pad: `own` (the strip's model-slot latency on its
/// processing path) + insert latency + pad = alignment target (default `natural`, the latency every strip
/// of the same family shares). The pad is preallocated for [`MAX_ALIGN_SAMPLES`]; a target below
/// `own + inserts` cannot be met (the pad is 0 and the strip runs late by the difference).
pub struct StripAlign {
    pad: LatencyPad,
    own: usize,
    natural: usize,
    target: Option<usize>,
    inserts: usize,
}

impl StripAlign {
    /// Allocates the pad.
    pub fn new(own: usize, natural: usize) -> Self {
        let mut s = Self {
            pad: LatencyPad::new(MAX_ALIGN_SAMPLES),
            own,
            natural,
            target: None,
            inserts: 0,
        };
        s.update();
        s
    }

    fn update(&mut self) {
        let t = self.target.unwrap_or(self.natural);
        self.pad
            .set_delay(t.saturating_sub(self.own + self.inserts));
    }

    /// Model-slot latency of the strip (constant).
    pub fn own(&self) -> usize {
        self.own
    }

    /// Default alignment target of this strip family.
    pub fn natural(&self) -> usize {
        self.natural
    }

    /// Alignment target (`None` = [`StripAlign::natural`]); returns the resulting total. RT-safe.
    pub fn set_target(&mut self, target: Option<usize>) -> usize {
        self.target = target;
        self.update();
        self.total()
    }

    /// Latency of the engaged inserts (FX rack); the pad shrinks to keep the total. RT-safe.
    pub fn set_insert_latency(&mut self, n: usize) -> usize {
        self.inserts = n;
        self.update();
        self.total()
    }

    pub fn insert_latency(&self) -> usize {
        self.inserts
    }

    /// Current padding in samples.
    pub fn pad(&self) -> usize {
        self.pad.delay()
    }

    /// Total strip latency: own + inserts + pad.
    pub fn total(&self) -> usize {
        self.own + self.inserts + self.pad.delay()
    }

    pub fn reset(&mut self) {
        self.pad.reset();
    }

    /// Pad the strip output in place (RT-safe).
    #[inline]
    pub fn process(&mut self, l: &mut [f32], r: &mut [f32]) {
        self.pad.process(l, r);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pad_delays_across_blocks() {
        let mut p = LatencyPad::new(64);
        assert_eq!(p.set_delay(5), 5);
        let x: Vec<f32> = (1..=40).map(|i| i as f32).collect();
        let (mut l, mut r) = (x.clone(), x.clone());
        for c in 0..4 {
            let s = c * 10..c * 10 + 10;
            let (a, b) = (&mut l[s.clone()], &mut r[s]);
            p.process(a, b);
        }
        for i in 0..40 {
            let want = if i < 5 { 0.0 } else { x[i - 5] };
            assert_eq!(l[i], want);
            assert_eq!(r[i], want);
        }
        assert_eq!(p.set_delay(1000), 64);
        let mut z = LatencyPad::new(8);
        let (mut a, mut b) = (x.clone(), x.clone());
        z.process(&mut a, &mut b);
        assert_eq!(a, x);
    }

    #[test]
    fn tap_line_reads_any_delay() {
        let mut t = TapLine::new(7, 4);
        let x: Vec<f32> = (1..=16).map(|i| i as f32).collect();
        let mut out = [0.0f32; 4];
        for (c, blk) in x.chunks(4).enumerate() {
            t.push(blk);
            for d in [0, 3, 7] {
                t.read(&mut out, d);
                for (i, &o) in out.iter().enumerate() {
                    let n = (c * 4 + i) as isize - d as isize;
                    assert_eq!(o, if n < 0 { 0.0 } else { x[n as usize] });
                }
            }
        }
    }

    #[test]
    fn align_keeps_total() {
        let mut a = StripAlign::new(57, 114);
        assert_eq!((a.pad(), a.total()), (57, 114));
        assert_eq!(a.set_insert_latency(20), 114);
        assert_eq!(a.pad(), 37);
        assert_eq!(a.set_insert_latency(100), 157);
        assert_eq!(a.set_target(Some(300)), 300);
        assert_eq!(a.set_target(None), 157);
    }
}
