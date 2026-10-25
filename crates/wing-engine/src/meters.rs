//! Real-time metering (T039): per-block peak levels measured on the audio threads and handed to
//! the meter sender through a lock-free [`LevelBank`].
//!
//! ```text
//! RT (driver thread + engine workers)                      meter sender (20 Hz)
//!   ChannelStrip: IN / PFL L+R, gate/dyn key + GR ─┐
//!   mix node (main 1 today, T036 busses/mains/mtx):├─ fetch_max ─▶ LevelBank ─ take (swap 0) ─▶ dB,
//!     strip post-fader out, mix in/out            │   (f32 bits)               ballistics, WING words
//!   PortMeters (after the cycle): source legs,    ─┘
//!     output legs (USB In/Out, oscillators, ...)
//! ```
//!
//! * **What is measured**: the absolute peak of each block ([`abs_max`], vectorised integer max),
//!   folded into the cell with an atomic max. A cell therefore holds the peak since the sender
//!   last took it, so no transient between two 50 ms meter packets is lost. WING reports levels
//!   only (protocol.md §4.2: no RMS words), so no RMS is computed.
//! * **Gain reduction** cells hold the largest GR (dB, >= 0) seen in the interval; **key** cells
//!   the peak of the detector key (the filtered side-chain when one is formed, else the slot
//!   input).
//! * **Cost**: zero cells cost nothing (`peak` skips `v <= 0`), so silent strips publish nothing.
//!   A writer does one `ldumax` (ARMv8.1 LSE) per non-silent word; no locks, no allocation.
//! * **Layout** (cell index): [`STRIP_COUNT`] strips × [`STRIP_WORDS`] (ch 1..40, aux 1..8, bus
//!   1..16, main 1..4, mtx 1..8, the [`Strip::index`] order), then [`SOURCE_COUNT`] source legs
//!   ([`SourceAddr::index`]) and [`OUTPUT_COUNT`] output legs ([`OutAddr::index`]).
//!
//! Hooks for the mixing nodes (T036/T037): a bus/main/matrix processor holds a [`StripMeter`] for
//! its strip (`StripMeter::new(bank, Strip::Bus(n))`) and calls `level(word::IN_L, ..)` etc. per
//! block; its output legs as sources (`$BUS`, `$MAIN`, `$MTX`) are metered by adding their ports
//! to [`PortMeters`]. Cells nobody writes read as silence.

use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering::Relaxed};
use std::sync::Arc;

use crate::graph::Port;
use crate::routing::{OutAddr, SourceAddr, Strip, OUTPUT_COUNT, SOURCE_COUNT, STRIP_COUNT};
use crate::Engine;

/// Words per strip.
pub const STRIP_WORDS: usize = 10;

/// Word offsets within a strip.
pub mod word {
    /// Input level (channel: after the input stage = tap IN; mix strips: the summed input).
    pub const IN_L: usize = 0;
    pub const IN_R: usize = 1;
    /// Pre-fader level (tap PFL).
    pub const PRE_L: usize = 2;
    pub const PRE_R: usize = 3;
    /// Post-fader (post mute and pan) level.
    pub const OUT_L: usize = 4;
    pub const OUT_R: usize = 5;
    /// Gate detector key peak / gate gain reduction (dB).
    pub const GATE_KEY: usize = 6;
    pub const GATE_GR: usize = 7;
    /// Dynamics detector key peak / dynamics gain reduction (dB).
    pub const DYN_KEY: usize = 8;
    pub const DYN_GR: usize = 9;
}

/// First source cell.
pub const SOURCE_BASE: usize = STRIP_COUNT * STRIP_WORDS;
/// First output cell.
pub const OUTPUT_BASE: usize = SOURCE_BASE + SOURCE_COUNT;
/// Total cells.
pub const CELLS: usize = OUTPUT_BASE + OUTPUT_COUNT;

/// Cell of a strip word (`strip` = [`Strip::index`]).
#[inline]
pub const fn strip_cell(strip: usize, w: usize) -> usize {
    strip * STRIP_WORDS + w
}

/// Cell of a source leg.
#[inline]
pub fn source_cell(s: SourceAddr) -> usize {
    SOURCE_BASE + s.index()
}

/// Cell of an output leg.
#[inline]
pub fn output_cell(o: OutAddr) -> usize {
    OUTPUT_BASE + o.index()
}

/// Absolute peak of a block. Works on the IEEE bit patterns (`|x|` = sign bit cleared; for
/// non-negative floats the integer order is the numeric order), which makes the reduction an
/// associative integer max that LLVM vectorises (NEON `umax`). NaN samples read as +inf (a broken
/// signal meters as over, not as silence).
#[inline]
pub fn abs_max(x: &[f32]) -> f32 {
    const ABS: u32 = 0x7fff_ffff;
    const INF: u32 = 0x7f80_0000;
    let m = x.iter().fold(0u32, |m, v| m.max(v.to_bits() & ABS));
    f32::from_bits(m.min(INF))
}

/// Latest linear levels (peak since the last [`LevelBank::take`]), one atomic `f32` per cell.
/// Written from any audio thread, read-and-cleared by one reader.
pub struct LevelBank {
    cells: Box<[AtomicU32]>,
    /// Set by the audio service while an RT session is attached (the sender falls back to
    /// synthetic meters otherwise).
    attached: AtomicBool,
    /// Engine cycles metered (diagnostics).
    cycles: AtomicU64,
}

impl Default for LevelBank {
    fn default() -> Self {
        Self::new()
    }
}

impl std::fmt::Debug for LevelBank {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("LevelBank")
            .field("cells", &self.cells.len())
            .field("attached", &self.is_attached())
            .field("cycles", &self.cycles())
            .finish()
    }
}

impl LevelBank {
    /// All cells silent. Allocates.
    pub fn new() -> Self {
        Self {
            cells: (0..CELLS).map(|_| AtomicU32::new(0)).collect(),
            attached: AtomicBool::new(false),
            cycles: AtomicU64::new(0),
        }
    }

    pub fn shared() -> Arc<Self> {
        Arc::new(Self::new())
    }

    /// Fold a non-negative level into a cell (atomic max). `v <= 0`, NaN and out-of-range
    /// cells are ignored. RT-safe, wait-free on LSE targets.
    #[inline]
    pub fn peak(&self, cell: usize, v: f32) {
        if v > 0.0 {
            if let Some(c) = self.cells.get(cell) {
                // Non-negative IEEE floats order like their bit patterns.
                c.fetch_max(v.to_bits(), Relaxed);
            }
        }
    }

    /// Read and clear a cell (0 when out of range).
    #[inline]
    pub fn take(&self, cell: usize) -> f32 {
        self.cells
            .get(cell)
            .map_or(0.0, |c| f32::from_bits(c.swap(0, Relaxed)))
    }

    /// Read a cell without clearing it.
    #[inline]
    pub fn get(&self, cell: usize) -> f32 {
        self.cells
            .get(cell)
            .map_or(0.0, |c| f32::from_bits(c.load(Relaxed)))
    }

    /// Clear every cell.
    pub fn clear(&self) {
        for c in self.cells.iter() {
            c.store(0, Relaxed);
        }
    }

    pub fn set_attached(&self, on: bool) {
        self.attached.store(on, Relaxed);
    }

    /// An RT session is feeding the bank.
    pub fn is_attached(&self) -> bool {
        self.attached.load(Relaxed)
    }

    /// Count one metered cycle. RT-safe.
    #[inline]
    pub fn tick(&self) {
        self.cycles.fetch_add(1, Relaxed);
    }

    pub fn cycles(&self) -> u64 {
        self.cycles.load(Relaxed)
    }
}

/// A strip's slice of a [`LevelBank`] (held by the strip's processor).
#[derive(Clone, Debug)]
pub struct StripMeter {
    bank: Arc<LevelBank>,
    base: usize,
}

impl StripMeter {
    /// `None` for an out-of-range strip.
    pub fn new(bank: Arc<LevelBank>, strip: Strip) -> Option<Self> {
        Some(Self::at(bank, strip.index()?))
    }

    /// By flat strip index (`0..STRIP_COUNT`).
    pub fn at(bank: Arc<LevelBank>, strip: usize) -> Self {
        Self {
            bank,
            base: strip_cell(strip.min(STRIP_COUNT - 1), 0),
        }
    }

    /// Fold a level / GR value into word `w` ([`word`]). RT-safe.
    #[inline]
    pub fn level(&self, w: usize, v: f32) {
        if w < STRIP_WORDS {
            self.bank.peak(self.base + w, v);
        }
    }

    /// Measure a stereo block into `l_word` / `l_word + 1`. RT-safe.
    #[inline]
    pub fn stereo(&self, l_word: usize, l: &[f32], r: &[f32]) {
        self.level(l_word, abs_max(l));
        self.level(l_word + 1, abs_max(r));
    }

    pub fn bank(&self) -> &Arc<LevelBank> {
        &self.bank
    }
}

/// Engine ports metered after each cycle (sources and outputs the graph exposes as ports).
/// Built off the RT thread; [`PortMeters::capture`] is RT-safe.
pub struct PortMeters {
    bank: Arc<LevelBank>,
    entries: Box<[(Port, u32)]>,
}

impl PortMeters {
    /// `entries`: (port, cell). Allocates.
    pub fn new(bank: Arc<LevelBank>, entries: Vec<(Port, usize)>) -> Self {
        Self {
            bank,
            entries: entries
                .into_iter()
                .filter(|&(_, c)| c < CELLS)
                .map(|(p, c)| (p, c as u32))
                .collect(),
        }
    }

    pub fn len(&self) -> usize {
        self.entries.len()
    }

    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    pub fn bank(&self) -> &Arc<LevelBank> {
        &self.bank
    }

    /// Measure every entry from the last cycle's buffers and count the cycle. RT-safe.
    #[inline]
    pub fn capture(&self, engine: &Engine) {
        for &(p, c) in self.entries.iter() {
            self.bank.peak(c as usize, abs_max(engine.port(p)));
        }
        self.bank.tick();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::routing::{OutGroup, SrcGroup};

    #[test]
    fn abs_max_matches_scalar() {
        for n in [0usize, 1, 7, 8, 9, 63, 64, 65, 200] {
            let x: Vec<f32> = (0..n)
                .map(|i| ((i as f32) * 0.37).sin() * (1.0 + i as f32 * 0.01))
                .collect();
            let want = x.iter().fold(0.0f32, |m, v| m.max(v.abs()));
            assert_eq!(abs_max(&x), want, "n={n}");
        }
        assert_eq!(abs_max(&[f32::NAN, -0.5]), f32::INFINITY);
        assert_eq!(abs_max(&[-0.0, -0.25, f32::NEG_INFINITY]), f32::INFINITY);
    }

    #[test]
    fn bank_peak_holds_until_taken() {
        let b = LevelBank::new();
        b.peak(3, 0.25);
        b.peak(3, 0.5);
        b.peak(3, 0.1);
        b.peak(3, -1.0);
        b.peak(3, f32::NAN);
        b.peak(CELLS + 5, 1.0);
        assert_eq!(b.get(3), 0.5);
        assert_eq!(b.take(3), 0.5);
        assert_eq!(b.take(3), 0.0);
        assert_eq!(b.take(CELLS + 5), 0.0);
    }

    #[test]
    fn layout_is_dense() {
        assert_eq!(strip_cell(STRIP_COUNT - 1, STRIP_WORDS - 1) + 1, SOURCE_BASE);
        let last_src = SourceAddr::new(SrcGroup::Mon, SrcGroup::Mon.count()).unwrap();
        assert_eq!(source_cell(last_src) + 1, OUTPUT_BASE);
        let last_out = OutAddr::new(OutGroup::Aes, 2).unwrap();
        assert_eq!(output_cell(last_out) + 1, CELLS);
        let m = StripMeter::new(LevelBank::shared(), Strip::Main(1)).unwrap();
        m.stereo(word::OUT_L, &[0.5, -0.75], &[0.25]);
        assert_eq!(m.bank().get(strip_cell(64, word::OUT_L)), 0.75);
        assert_eq!(m.bank().get(strip_cell(64, word::OUT_R)), 0.25);
    }
}
