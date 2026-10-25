//! Latency compensation (T03E, T05U): per-slot latency and per-family strip alignment.
//!
//! * **Model slot** ([`crate::params::strip::ModelSlot`]): by default (gate, dynamics, EQ, filter-tool
//!   slots) the slot latency is the *maximum* latency of the models it can host (T03E, WING-style
//!   fixed-latency blocks). One input delay line ([`TapLine`]) feeds the dry path at that latency and each
//!   model at `slot latency - model latency` (its side-chain key likewise), so `mix < 100 %` blends
//!   phase-aligned signals (no comb filtering), model crossfades blend aligned signals, an incoming model
//!   sees continuous input history, and the slot's latency stays the same when the model, `on` or `mix`
//!   changes. Slots whose models are all zero-latency add nothing (bit-exact as before).
//! * **FX slots** (T05U, [`crate::fx`]) are *dynamic*: the latency is that of the **loaded** model (STD-type
//!   models 0, PCORR 360, ...). Each model gets the live input, the dry path runs at the model's latency,
//!   and a swap between models of different latency crossfades the dry path between the two delays (raised
//!   cosine, [`smooth_fade`]) in step with the model crossfade.
//! * **Strip** ([`crate::channel::ChannelStrip`], [`crate::mix::strip::MixStrip`]): own slot latency + owned
//!   insert latency + an alignment pad ([`StripAlign`] / [`LatencyPad`]) at the strip output = the strip's
//!   alignment target. Under the engine's FX rack the target is the **family** target published by the
//!   rack's alignment board (largest `own + inserts` among input strips ch+aux / busses / mains / matrices,
//!   recomputed between cycles when a slot's model or an insert assignment changes), so every source summed
//!   into a bus arrives sample-aligned at the lowest common latency. A pad change crossfades its two taps
//!   (raised cosine over [`PAD_FADE_SAMPLES`], the 5 ms model crossfade): re-alignment is click-free and
//!   allocation-free (pads preallocated for [`MAX_ALIGN_SAMPLES`]). A standalone strip (no rack) pads to
//!   its family's worst case (T03E behaviour).
//!
//! Model latencies at 48 kHz: 76LA/NSTR (oversampled FET compressors) 57 samples, WARM 39, every other
//! dynamics model and every EQ / filter-tool model 0; gate and dynamics slots (one shared model table) 57,
//! EQ and filter-tool slots 0. FX (loaded model): PCORR 360, LIMITER 50, VINTAGE strip 57, enhancers 46,
//! the rest 0. An FX insert with latency L on an input strip raises the input-family target to `114 + L`
//! (PCORR: 474 = 9.9 ms). PUBLIC.snap (PCORR on ch 21/22, a 142-sample *MASTER* on main 4): inputs 474,
//! busses 57, mains 199, matrices 57 (before T05U every insert-using strip ran 360 per insert late,
//! unaligned: inputs 114..834, busses 57..777).
//!
//! Why gate/dynamics slots stay constant (T05U evaluation): a dynamic dynamics slot would shift every
//! pre-fader tap after it (`ptap` T3..T5: PRE sends, monitor mixes) by its model's latency relative to the
//! other channels, since only the strip output is padded; keeping them constant keeps those taps aligned
//! for a cost of at most 114 / 57 samples. Added latency without latency-carrying FX inserts:
//!
//! | path | samples | ms |
//! |---|---|---|
//! | channel / aux strip (gate 57 + dyn 57) | 114 | 2.375 |
//! | bus / main / matrix strip (dyn 57) | 57 | 1.1875 |
//! | channel -> bus -> main -> matrix | 114 + 57 + 57 + 57 = 285 | 5.94 |
//!
//! Taps and sends taken inside a strip (PRE/IN/... taps, meters) see the latency accumulated up to that
//! point (not compensated; a tap after an FX insert includes its latency), as on a console without
//! per-tap alignment.

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

/// Raised-cosine crossfade gain for fade position `x` (clamped to 0..1): `0.5 - 0.5 cos(pi x)`. Used for
/// blends of time-shifted copies of a signal (delay changes): unlike a linear ramp its slope is 0 at both
/// ends, so the fade adds no kink (`|b - a| / len` second-difference step) at its start and end.
#[inline]
pub fn smooth_fade(x: f32) -> f32 {
    0.5 - 0.5 * (std::f32::consts::PI * x.clamp(0.0, 1.0)).cos()
}

/// Default length of a [`LatencyPad`] delay change crossfade (samples; 5 ms at 48 kHz, the model-swap
/// crossfade length, so a strip's pad change runs in step with the slot swap that caused it).
pub const PAD_FADE_SAMPLES: usize = 240;
/// Largest chunk the pad processes at once (its delay line holds `capacity + PAD_CHUNK`).
const PAD_CHUNK: usize = 256;

/// Stereo integer-sample delay with preallocated capacity (control thread allocates, RT thread uses).
///
/// A delay change ([`LatencyPad::set_delay`]) crossfades linearly from the old tap to the new one over
/// [`LatencyPad::set_fade_samples`] samples (T05U): both taps read the same continuous history, so the change
/// is click-free. A change requested while a fade runs is applied when it completes (a request for the
/// fading-out delay reverses the fade). The history is always written; a delay of 0 outside a fade leaves
/// the audio untouched (bit-exact).
pub struct LatencyPad {
    line: [TapLine; 2],
    tmp: Vec<f32>,
    cap: usize,
    /// Audible delay (the fade-in side while fading).
    cur: usize,
    /// Fade-out side.
    prev: usize,
    /// Requested delay (applied when the running fade completes).
    pending: usize,
    fading: bool,
    pos: usize,
    len: usize,
}

impl LatencyPad {
    /// Pad for delays up to `capacity` samples, delay 0. Allocates.
    pub fn new(capacity: usize) -> Self {
        Self {
            line: [
                TapLine::new(capacity, PAD_CHUNK),
                TapLine::new(capacity, PAD_CHUNK),
            ],
            tmp: vec![0.0; PAD_CHUNK],
            cap: capacity,
            cur: 0,
            prev: 0,
            pending: 0,
            fading: false,
            pos: 0,
            len: PAD_FADE_SAMPLES,
        }
    }

    /// Largest delay this pad can apply.
    pub fn capacity(&self) -> usize {
        self.cap
    }

    /// Delay in samples (the requested one: during a fade, where the pad is heading).
    pub fn delay(&self) -> usize {
        self.pending
    }

    /// Whether a delay change is being crossfaded.
    pub fn is_fading(&self) -> bool {
        self.fading
    }

    /// Crossfade length of a delay change (0 = hard switch). RT-safe.
    pub fn set_fade_samples(&mut self, n: usize) {
        self.len = n;
    }

    /// Set the delay (clamped to the capacity; returns the applied delay), crossfading from the current
    /// delay. RT-safe.
    pub fn set_delay(&mut self, d: usize) -> usize {
        let d = d.min(self.cap);
        self.pending = d;
        if !self.fading {
            self.start(d);
        } else if d == self.prev {
            // reverse the running fade
            std::mem::swap(&mut self.cur, &mut self.prev);
            self.pos = self.len - self.pos.min(self.len);
        }
        d
    }

    /// Jump to delay `d` without a fade, clearing the history (construction, scene start). RT-safe.
    pub fn set_delay_now(&mut self, d: usize) -> usize {
        let d = d.min(self.cap);
        self.pending = d;
        self.reset();
        d
    }

    fn start(&mut self, d: usize) {
        if d == self.cur {
            return;
        }
        if self.len == 0 {
            self.cur = d;
            return;
        }
        self.prev = self.cur;
        self.cur = d;
        self.pos = 0;
        self.fading = true;
    }

    /// Clear the history and finish any fade at the requested delay (RT-safe).
    pub fn reset(&mut self) {
        for t in &mut self.line {
            t.reset();
        }
        self.fading = false;
        self.cur = self.pending;
        self.prev = self.pending;
    }

    /// Delay a stereo block in place (RT-safe).
    #[inline]
    pub fn process(&mut self, l: &mut [f32], r: &mut [f32]) {
        let mut s = 0;
        while s < l.len() {
            let e = (s + PAD_CHUNK).min(l.len());
            self.chunk(&mut l[s..e], &mut r[s..e]);
            s = e;
        }
    }

    fn chunk(&mut self, l: &mut [f32], r: &mut [f32]) {
        let n = l.len();
        self.line[0].push(l);
        self.line[1].push(r);
        if !self.fading {
            if self.cur > 0 {
                self.line[0].read(l, self.cur);
                self.line[1].read(r, self.cur);
            }
            return;
        }
        let inv = 1.0 / self.len as f32;
        for (c, x) in [l, r].into_iter().enumerate() {
            let tmp = &mut self.tmp[..n];
            self.line[c].read(tmp, self.prev);
            self.line[c].read(x, self.cur);
            for (i, (v, &p)) in x.iter_mut().zip(tmp.iter()).enumerate() {
                let g = smooth_fade((self.pos + i + 1) as f32 * inv);
                *v = p + g * (*v - p);
            }
        }
        self.pos += n;
        if self.pos >= self.len {
            self.fading = false;
            self.prev = self.cur;
            let p = self.pending;
            self.start(p);
        }
    }
}

/// A strip's latency bookkeeping and output alignment pad: `own` (the latency of the models loaded in the
/// strip's slots) + insert latency + pad = alignment target. The target is, by priority, an explicit
/// [`StripAlign::set_target`], the engine-wide family target ([`StripAlign::set_shared_target`], published by
/// the FX rack's alignment board, T05U), else `natural` (the worst case of the strip family: every slot at
/// its highest-latency model; a standalone strip behaves as in T03E). The pad is preallocated for
/// [`MAX_ALIGN_SAMPLES`] and crossfades its changes; a target below `own + inserts` cannot be met (the pad
/// is 0 and the strip runs late by the difference).
pub struct StripAlign {
    pad: LatencyPad,
    own: usize,
    natural: usize,
    target: Option<usize>,
    shared: Option<usize>,
    inserts: usize,
}

impl StripAlign {
    /// Allocates the pad (initial delay without a fade).
    pub fn new(own: usize, natural: usize) -> Self {
        let mut s = Self {
            pad: LatencyPad::new(MAX_ALIGN_SAMPLES),
            own,
            natural,
            target: None,
            shared: None,
            inserts: 0,
        };
        let d = s.wanted_pad();
        s.pad.set_delay_now(d);
        s
    }

    fn goal(&self) -> usize {
        self.target.or(self.shared).unwrap_or(self.natural)
    }

    fn wanted_pad(&self) -> usize {
        self.goal().saturating_sub(self.own + self.inserts)
    }

    fn update(&mut self) {
        let d = self.wanted_pad();
        if d != self.pad.delay() {
            self.pad.set_delay(d);
        }
    }

    /// Model-slot latency of the strip (the models currently loaded).
    pub fn own(&self) -> usize {
        self.own
    }

    /// Worst-case latency of this strip family (the default alignment target).
    pub fn natural(&self) -> usize {
        self.natural
    }

    /// Alignment target in effect (explicit, else engine family target, else natural).
    pub fn target(&self) -> usize {
        self.goal()
    }

    /// Explicit alignment target (`None` = engine family target or [`StripAlign::natural`]); returns the
    /// resulting total. RT-safe.
    pub fn set_target(&mut self, target: Option<usize>) -> usize {
        self.target = target;
        self.update();
        self.total()
    }

    /// Engine-wide family target (`None` = none published). RT-safe.
    pub fn set_shared_target(&mut self, target: Option<usize>) -> usize {
        self.shared = target;
        self.update();
        self.total()
    }

    /// Latency of the engaged inserts (FX rack); the pad shrinks to keep the total. RT-safe.
    pub fn set_insert_latency(&mut self, n: usize) -> usize {
        self.inserts = n;
        self.update();
        self.total()
    }

    /// Latency of the models loaded in the strip's slots; the pad absorbs changes. RT-safe.
    pub fn set_own_latency(&mut self, n: usize) -> usize {
        self.own = n;
        self.update();
        self.total()
    }

    /// Set own latency, insert latency and the engine family target at once (one pad change). RT-safe.
    #[inline]
    pub fn sync(&mut self, own: usize, inserts: usize, shared: Option<usize>) {
        if own != self.own || inserts != self.inserts || shared != self.shared {
            self.own = own;
            self.inserts = inserts;
            self.shared = shared;
            self.update();
        }
    }

    pub fn insert_latency(&self) -> usize {
        self.inserts
    }

    /// Padding in samples (where the pad is heading during a crossfade).
    pub fn pad(&self) -> usize {
        self.pad.delay()
    }

    /// Whether the pad is crossfading a change.
    pub fn is_fading(&self) -> bool {
        self.pad.is_fading()
    }

    /// Crossfade length of a pad change (samples). RT-safe.
    pub fn set_fade_samples(&mut self, n: usize) {
        self.pad.set_fade_samples(n);
    }

    /// Total strip latency: own + inserts + pad.
    pub fn total(&self) -> usize {
        self.own + self.inserts + self.pad.delay()
    }

    /// Clear the pad history and finish a running crossfade (scene start, tests).
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
        assert_eq!(p.set_delay_now(5), 5);
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
        assert_eq!(p.set_delay_now(1000), 64);
        let mut z = LatencyPad::new(8);
        let (mut a, mut b) = (x.clone(), x.clone());
        z.process(&mut a, &mut b);
        assert_eq!(a, x);
    }

    #[test]
    fn pad_crossfades_delay_changes() {
        let mut p = LatencyPad::new(512);
        p.set_fade_samples(100);
        let x: Vec<f32> = (0..2000).map(|i| (i as f32 * 0.01).sin()).collect();
        let (mut l, mut r) = (x.clone(), x.clone());
        for (c, (a, b)) in l.chunks_mut(64).zip(r.chunks_mut(64)).enumerate() {
            match c {
                2 => assert_eq!(p.set_delay(300), 300),
                3 => {
                    // retarget mid-fade: applied after the running fade
                    p.set_delay(10);
                    assert_eq!(p.delay(), 10);
                }
                _ => {}
            }
            p.process(a, b);
        }
        assert!(!p.is_fading());
        // before: untouched; after both fades: delayed by 10; smooth throughout
        assert_eq!(l[..128], x[..128]);
        for i in 1500..2000 {
            assert_eq!(l[i], x[i - 10]);
        }
        let step = l
            .windows(2)
            .map(|w| (w[1] - w[0]).abs())
            .fold(0.0, f32::max);
        assert!(step < 0.05, "step {step}");
        assert_eq!(l, r);
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
