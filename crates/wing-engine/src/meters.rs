//! Real-time metering (T039): per-block peak levels measured on the audio threads and handed to
//! the meter sender through a lock-free [`LevelBank`].
//!
//! ```text
//! RT (driver thread + engine workers)                      meter sender (20 Hz)
//!   ChannelStrip: IN / PFL L+R, gate/dyn key + GR ─┐
//!   PortMeters (after the cycle, T036 mix graph): ├─ fetch_max ─▶ LevelBank ─ take (swap 0) ─▶ dB,
//!     ch/aux post-fader (AFL) out, mix in/out,    │   (f32 bits)               ballistics, WING words
//!     source legs (USB, OSC, $BUS/$MAIN/$MTX),    │
//!     output legs (USB Out)                       ─┘
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
//! Mix graph (T036): [`PortMeters::for_mix`] meters every strip output, bus/main/matrix in/out
//! and the mix-output source legs from the graph's ports. Bus/main/matrix processing (T037): the
//! [`crate::mix::MixStrip`] hook holds a [`StripMeter`] for its strip and writes `IN_*` (sum after
//! trim) and `DYN_KEY`/`DYN_GR` per block (no gate on these strips). Cells nobody writes read as
//! silence.

use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering::Relaxed};
use std::sync::Arc;

pub use wing_blocks::meter::{DynMeterLaw, FxMeterLaw, MAX_METER_BANDS};

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
/// First solo-bus cell (L, R; T038).
pub const SOLO_BASE: usize = OUTPUT_BASE + OUTPUT_COUNT;
/// First band cell (T082): per strip [`BAND_WORDS`] signed band gains (dB, latest value) of the gate and
/// dynamics models, for the dynamic-EQ meter laws (`wing_blocks::meter`).
pub const BAND_BASE: usize = SOLO_BASE + 2;
/// Band cells per strip: gate bands 0/1, dynamics bands 0/1 ([`band`]).
pub const BAND_WORDS: usize = 4;
/// First FX cell (T082): per FX slot [`FX_WORDS`] ([`fx_word`]).
pub const FX_BASE: usize = BAND_BASE + STRIP_COUNT * BAND_WORDS;
/// Cells per FX slot.
pub const FX_WORDS: usize = 5 + wing_blocks::meter::MAX_METER_BANDS;
/// Total cells.
pub const CELLS: usize = FX_BASE + crate::fx::FX_SLOTS * FX_WORDS;

/// Band cell offsets within a strip's band cells.
pub mod band {
    pub const GATE: usize = 0;
    pub const DYN: usize = 2;
    /// Bands kept per slot.
    pub const PER_SLOT: usize = 2;
}

/// FX slot cell offsets: in/out level peaks, the model's GR (dB, interval max) and its signed band gains
/// (dB, latest value).
pub mod fx_word {
    pub const IN_L: usize = 0;
    pub const IN_R: usize = 1;
    pub const OUT_L: usize = 2;
    pub const OUT_R: usize = 3;
    pub const GR: usize = 4;
    pub const BAND0: usize = 5;
}

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

/// Cell of the solo bus level (side 0 = L, 1 = R): the louder of the two monitors' solo busses.
#[inline]
pub const fn solo_cell(side: usize) -> usize {
    SOLO_BASE + side
}

/// Band cell `w` ([`band`]) of a strip.
#[inline]
pub const fn band_cell(strip: usize, w: usize) -> usize {
    BAND_BASE + strip * BAND_WORDS + w
}

/// Cell `w` ([`fx_word`]) of FX slot `slot` (0-based).
#[inline]
pub const fn fx_cell(slot: usize, w: usize) -> usize {
    FX_BASE + slot * FX_WORDS + w
}

/// Cells holding a value rather than a level: gain reduction (dB, interval max) and band gains (dB, latest
/// value). The sender shows them without level ballistics.
pub fn is_value_cell(cell: usize) -> bool {
    if cell < SOURCE_BASE {
        matches!(cell % STRIP_WORDS, word::GATE_GR | word::DYN_GR)
    } else if cell >= FX_BASE {
        (cell - FX_BASE) % FX_WORDS >= fx_word::GR
    } else {
        cell >= BAND_BASE
    }
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

    /// Store a signed value (latest-value cells: band gains). NaN and out-of-range cells are ignored.
    /// RT-safe.
    #[inline]
    pub fn set(&self, cell: usize, v: f32) {
        if !v.is_nan() {
            if let Some(c) = self.cells.get(cell) {
                c.store(v.to_bits(), Relaxed);
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

    /// Publish a model slot's signed band gains (`slot` = [`band::GATE`] or [`band::DYN`]); bands the model
    /// does not report read 0. RT-safe.
    #[inline]
    pub fn bands(&self, slot: usize, bands: &[f32]) {
        let strip = self.base / STRIP_WORDS;
        for k in 0..band::PER_SLOT {
            self.bank.set(
                band_cell(strip, slot + k),
                bands.get(k).copied().unwrap_or(0.0),
            );
        }
    }

    /// Measure a model slot's band gains and publish them (see [`StripMeter::bands`]). RT-safe.
    #[inline]
    pub fn slot_bands(&self, slot: usize, model: &crate::params::strip::ModelSlot) {
        let mut b = [0.0f32; wing_blocks::meter::MAX_METER_BANDS];
        let n = model.meter_bands(&mut b);
        self.bands(slot, &b[..n]);
    }

    pub fn bank(&self) -> &Arc<LevelBank> {
        &self.bank
    }
}

/// An FX slot's cells of a [`LevelBank`] (held by the slot, T082).
#[derive(Clone, Debug)]
pub struct FxMeter {
    bank: Arc<LevelBank>,
    base: usize,
}

impl FxMeter {
    /// FX slot `n` (1..16).
    pub fn new(bank: Arc<LevelBank>, n: u8) -> Self {
        let k = (n as usize).clamp(1, crate::fx::FX_SLOTS) - 1;
        Self {
            bank,
            base: fx_cell(k, 0),
        }
    }

    /// Slot input level. RT-safe.
    #[inline]
    pub fn input(&self, l: &[f32], r: &[f32]) {
        self.bank.peak(self.base + fx_word::IN_L, abs_max(l));
        self.bank.peak(self.base + fx_word::IN_R, abs_max(r));
    }

    /// Slot output level, the model's GR and band gains. RT-safe.
    #[inline]
    pub fn output(&self, l: &[f32], r: &[f32], model: &crate::params::strip::ModelSlot) {
        self.bank.peak(self.base + fx_word::OUT_L, abs_max(l));
        self.bank.peak(self.base + fx_word::OUT_R, abs_max(r));
        self.bank
            .peak(self.base + fx_word::GR, model.gain_reduction_db());
        let mut b = [0.0f32; wing_blocks::meter::MAX_METER_BANDS];
        let n = model.meter_bands(&mut b);
        for (k, v) in b.iter().enumerate() {
            self.bank
                .set(self.base + fx_word::BAND0 + k, if k < n { *v } else { 0.0 });
        }
    }

    pub fn bank(&self) -> &Arc<LevelBank> {
        &self.bank
    }
}

/// Engine ports metered after each cycle: each port's block peak goes to one or two cells (e.g. a
/// main's POST port feeds both its strip OUT word and its `$MAIN` source leg).
/// Built off the RT thread; [`PortMeters::capture`] is RT-safe.
pub struct PortMeters {
    bank: Arc<LevelBank>,
    /// (port, cells; `u32::MAX` = unused).
    entries: Box<[(Port, [u32; 2])]>,
}

const NO_CELL: u32 = u32::MAX;

impl PortMeters {
    /// `entries`: (port, cell). A port listed twice (up to two cells) is measured once.
    /// Allocates.
    pub fn new(bank: Arc<LevelBank>, entries: Vec<(Port, usize)>) -> Self {
        let mut out: Vec<(Port, [u32; 2])> = Vec::with_capacity(entries.len());
        for (p, c) in entries {
            if c >= CELLS {
                continue;
            }
            match out.iter_mut().find(|(q, cs)| *q == p && cs[1] == NO_CELL) {
                Some((_, cs)) => cs[1] = c as u32,
                None => out.push((p, [c as u32, NO_CELL])),
            }
        }
        Self {
            bank,
            entries: out.into_boxed_slice(),
        }
    }

    /// Every meter the T036 mix graph exposes as ports:
    /// * sources: USB In 1..48 (externals), OSC 1/2, `$BUS` 1..32, `$MAIN` 1..8, `$MTX` 1..16
    ///   (the mix strips' POST legs);
    /// * outputs: USB Out 1..48 (the only rendered output group);
    /// * ch/aux OUT = AFL (post mute and fader, before width/pan: a centred mono channel reads
    ///   its fader level, not the pan law's -3 dB) [choice];
    /// * bus/main/matrix PRE = PFL (after the T037 strip processing), OUT = POST. Their IN word
    ///   (sum after trim) and dynamics key/GR are written by the [`crate::mix::MixStrip`] hook's
    ///   [`StripMeter`] (`MixConfig::meters`).
    /// * monitor (T038): `MON` source legs 1..4 = monitor A/B outputs L/R, solo cells = the solo
    ///   busses ([`solo_cell`]).
    ///
    /// The input strips' own words (IN, PFL, keys, GR) come from their [`StripMeter`]
    /// ([`crate::mix::MixConfig::meters`]). Allocates.
    pub fn for_mix(bank: Arc<LevelBank>, nodes: &crate::mix::MixNodes) -> Self {
        use crate::mix::{InputSignal, MixSignal};
        use crate::routing::{OutGroup, Side, SrcGroup, USB_CHANNELS};
        let mut e = Vec::new();
        for k in 0..USB_CHANNELS as u16 {
            if let Some(a) = SourceAddr::new(SrcGroup::Usb, k + 1) {
                e.push((Port::External(k as u32), source_cell(a)));
            }
            if let Some(a) = OutAddr::new(OutGroup::Usb, k + 1) {
                e.push((Port::node(nodes.usb_out, k as u32), output_cell(a)));
            }
        }
        for k in 0..2u16 {
            if let Some(a) = SourceAddr::new(SrcGroup::Osc, k + 1) {
                e.push((Port::node(nodes.io.osc, k as u32), source_cell(a)));
            }
        }
        let sides = [(Side::L, 0usize), (Side::R, 1)];
        for i in 0..nodes.inputs.len() {
            for (side, k) in sides {
                let p = nodes.input_port(i, InputSignal::Afl, side);
                e.push((p, strip_cell(i, word::OUT_L + k)));
            }
        }
        for si in 48..STRIP_COUNT {
            let Some(strip) = Strip::from_index(si) else {
                continue;
            };
            let (grp, n) = match strip {
                Strip::Bus(n) => (SrcGroup::Bus, n),
                Strip::Main(n) => (SrcGroup::Main, n),
                Strip::Mtx(n) => (SrcGroup::Mtx, n),
                _ => continue,
            };
            for (side, k) in sides {
                if let Some(p) = nodes.mix_port(strip, MixSignal::Pfl, side) {
                    e.push((p, strip_cell(si, word::PRE_L + k)));
                }
                if let Some(p) = nodes.mix_port(strip, MixSignal::Post, side) {
                    e.push((p, strip_cell(si, word::OUT_L + k)));
                    // mix-output legs 2n-1 / 2n = L / R of strip n
                    if let Some(a) = SourceAddr::new(grp, 2 * n as u16 - 1 + k as u16) {
                        e.push((p, source_cell(a)));
                    }
                }
            }
        }
        for (m, side, k) in [
            (1, Side::L, 0usize),
            (1, Side::R, 1),
            (2, Side::L, 0),
            (2, Side::R, 1),
        ] {
            let leg = 2 * (m as u16 - 1) + 1 + k as u16;
            if let Some(a) = SourceAddr::new(SrcGroup::Mon, leg) {
                e.push((nodes.monitor_port(m, side), source_cell(a)));
            }
            e.push((nodes.solo_port(m, side), solo_cell(k)));
        }
        Self::new(bank, e)
    }

    /// Ports measured per cycle.
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
        for &(p, [c0, c1]) in self.entries.iter() {
            let v = abs_max(engine.port(p));
            self.bank.peak(c0 as usize, v);
            if c1 != NO_CELL {
                self.bank.peak(c1 as usize, v);
            }
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
        assert_eq!(
            strip_cell(STRIP_COUNT - 1, STRIP_WORDS - 1) + 1,
            SOURCE_BASE
        );
        let last_src = SourceAddr::new(SrcGroup::Mon, SrcGroup::Mon.count()).unwrap();
        assert_eq!(source_cell(last_src) + 1, OUTPUT_BASE);
        let last_out = OutAddr::new(OutGroup::Aes, 2).unwrap();
        assert_eq!(output_cell(last_out) + 1, SOLO_BASE);
        assert_eq!(solo_cell(1) + 1, BAND_BASE);
        assert_eq!(band_cell(STRIP_COUNT - 1, BAND_WORDS - 1) + 1, FX_BASE);
        assert_eq!(fx_cell(crate::fx::FX_SLOTS - 1, FX_WORDS - 1) + 1, CELLS);
        assert!(is_value_cell(strip_cell(3, word::DYN_GR)));
        assert!(!is_value_cell(strip_cell(3, word::DYN_KEY)));
        assert!(!is_value_cell(solo_cell(1)));
        assert!(is_value_cell(band_cell(0, band::GATE)));
        assert!(!is_value_cell(fx_cell(2, fx_word::OUT_R)));
        assert!(is_value_cell(fx_cell(2, fx_word::GR)));
        assert!(is_value_cell(fx_cell(2, fx_word::BAND0 + 5)));
        let m = StripMeter::new(LevelBank::shared(), Strip::Main(1)).unwrap();
        m.stereo(word::OUT_L, &[0.5, -0.75], &[0.25]);
        assert_eq!(m.bank().get(strip_cell(64, word::OUT_L)), 0.75);
        assert_eq!(m.bank().get(strip_cell(64, word::OUT_R)), 0.25);
    }
}
