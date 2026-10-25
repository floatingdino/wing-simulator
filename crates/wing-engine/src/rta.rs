//! Real-time analyser (T084): the console RTA behind WING-Edit's EQ/RTA overlays.
//!
//! ```text
//! control (cfg/rta rtasrc/rtatap) ─request─▶ RtaFeed ◀─latch (driver thread, once per cycle)
//! RT: the strip / port selected by the latched (src, tap) ─push_stereo─▶ lock-free ring (mono sum)
//! analyser thread (non-RT): RtaReader ─▶ RtaAnalyser (120 semitone bands, detector, decay) ─▶ dB
//! ```
//!
//! * **Selection** ([`RtaSel`]): `src` = `cfg/rta/rtasrc` (1..40 ch, 41..48 aux, 49..64 bus,
//!   65..68 main, 69..76 mtx = [`crate::routing::Strip::index`] + 1; FX slot k for the FX taps;
//!   0 = off) and `tap` = `rtatap` ([`RtaTap`], schema enum order). The control side writes a
//!   request; the driver thread latches it once per cycle ([`RtaFeed::latch`]) so every node of a
//!   cycle sees the same selection: exactly one tap point writes the ring per cycle (single
//!   producer).
//! * **RT cost**: one relaxed load per strip per block, and for the one selected tap point a
//!   copy of the block (mono `(L+R)/2`, relaxed stores) into the ring. No locks, no allocation.
//! * **Ring**: overwrite-on-full ([`RtaFeed::push_stereo`] never blocks); the reader
//!   ([`RtaReader`]) detects being lapped and resynchronises to the newest audio.
//! * **Bands** ([`band_freq`]): 120 semitone bands from E0 (20.6 Hz) to D#10 (19.9 kHz), A4 =
//!   440 Hz at index 53 (protocol.md §4.2). Each band is a 4th-order band-pass (two identical RBJ
//!   band-pass sections, 0 dB at the centre) whose -3 dB width is 1/12 octave [choice: WING's
//!   filter shape is not documented]; bands at or above 0.45 fs read as silence.
//! * **Detector** ([`RtaDetector`], `rtadet`): PEAK = largest |y| of the interval, RMS =
//!   sqrt(mean y²), AVG = mean |y| [H: WING's AVG is undocumented]. A full-scale sine at a band
//!   centre reads 0 dBFS PEAK, -3.01 dB RMS, -3.92 dB AVG.
//! * **Decay** ([`RtaDecay`], `rtadecay`): instant rise, linear-in-dB fall at
//!   [`RtaDecay::db_per_s`] (SLOW 6, MED 12, FAST 24 dB/s) [H: WING's rates are undocumented].

use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering::*};

/// Number of RTA bands (semitones).
pub const RTA_BANDS: usize = 120;
/// Band of A4 = 440 Hz.
pub const RTA_A4_BAND: usize = 53;
/// Silence (dB).
pub const RTA_FLOOR_DB: f32 = -128.0;
/// Ring capacity (samples): 0.34 s at 96 kHz, enough for the analyser's 50 ms tick with slack.
pub const RTA_RING: usize = 1 << 15;

/// Centre frequency of band `k`: `440 * 2^((k - 53) / 12)` Hz.
pub fn band_freq(k: usize) -> f64 {
    440.0 * 2f64.powf((k as f64 - RTA_A4_BAND as f64) / 12.0)
}

/// `cfg/rta/rtatap` (schema enum order).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum RtaTap {
    In = 0,
    Post = 1,
    Filt = 2,
    PreEq = 3,
    PostEq = 4,
    PreFdr = 5,
    GateKey = 6,
    DynKey = 7,
    DynXo = 8,
    PreTap = 9,
    Solo = 10,
    MonPh = 11,
    MonSpk = 12,
    FxIn = 13,
    FxOut = 14,
}

impl RtaTap {
    pub const ALL: [RtaTap; 15] = [
        RtaTap::In,
        RtaTap::Post,
        RtaTap::Filt,
        RtaTap::PreEq,
        RtaTap::PostEq,
        RtaTap::PreFdr,
        RtaTap::GateKey,
        RtaTap::DynKey,
        RtaTap::DynXo,
        RtaTap::PreTap,
        RtaTap::Solo,
        RtaTap::MonPh,
        RtaTap::MonSpk,
        RtaTap::FxIn,
        RtaTap::FxOut,
    ];

    pub fn from_index(i: u32) -> Option<Self> {
        Self::ALL.get(i as usize).copied()
    }

    /// Taps whose signal does not depend on `rtasrc` (solo bus, monitor outputs).
    pub fn is_global(self) -> bool {
        matches!(self, RtaTap::Solo | RtaTap::MonPh | RtaTap::MonSpk)
    }
}

/// A latched RTA selection.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RtaSel {
    /// `rtasrc` (strip index + 1, or the FX slot for the FX taps).
    pub src: u16,
    pub tap: RtaTap,
}

const NO_SEL: u32 = u32::MAX;

impl RtaSel {
    fn pack(s: Option<RtaSel>) -> u32 {
        s.map_or(NO_SEL, |s| (s.src as u32) << 8 | s.tap as u32)
    }

    fn unpack(v: u32) -> Option<RtaSel> {
        if v == NO_SEL {
            return None;
        }
        Some(RtaSel {
            src: (v >> 8) as u16,
            tap: RtaTap::from_index(v & 0xFF)?,
        })
    }

    /// The flat strip index (`Strip::index`) the selection reads, for strip taps.
    pub fn strip(self) -> Option<usize> {
        match self.tap {
            RtaTap::FxIn | RtaTap::FxOut => None,
            t if t.is_global() => None,
            _ => (1..=76).contains(&self.src).then(|| self.src as usize - 1),
        }
    }
}

/// Shared RTA state between control, RT and analyser (see the module docs).
pub struct RtaFeed {
    request: AtomicU32,
    active: AtomicU32,
    /// Bumped by [`RtaFeed::latch`] whenever the active selection changes.
    epoch: AtomicU32,
    ring: Box<[AtomicU32]>,
    write: AtomicU64,
    sample_rate: AtomicU32,
    attached: AtomicBool,
}

impl std::fmt::Debug for RtaFeed {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("RtaFeed")
            .field("active", &self.active())
            .field("written", &self.written())
            .field("sample_rate", &self.sample_rate())
            .finish()
    }
}

impl Default for RtaFeed {
    fn default() -> Self {
        Self::new(RTA_RING)
    }
}

impl RtaFeed {
    /// A feed with a ring of `capacity` samples (rounded up to a power of two). Allocates.
    pub fn new(capacity: usize) -> Self {
        let cap = capacity.max(64).next_power_of_two();
        Self {
            request: AtomicU32::new(NO_SEL),
            active: AtomicU32::new(NO_SEL),
            epoch: AtomicU32::new(0),
            ring: (0..cap).map(|_| AtomicU32::new(0)).collect(),
            write: AtomicU64::new(0),
            sample_rate: AtomicU32::new(0),
            attached: AtomicBool::new(false),
        }
    }

    pub fn shared() -> std::sync::Arc<Self> {
        std::sync::Arc::new(Self::default())
    }

    pub fn capacity(&self) -> usize {
        self.ring.len()
    }

    /// Request a selection (control side; takes effect at the next [`RtaFeed::latch`]).
    pub fn request(&self, sel: Option<RtaSel>) {
        self.request.store(RtaSel::pack(sel), Relaxed);
    }

    pub fn requested(&self) -> Option<RtaSel> {
        RtaSel::unpack(self.request.load(Relaxed))
    }

    /// Latch the requested selection for the coming cycle. Call once per cycle on the driver
    /// thread before the graph runs. RT-safe.
    #[inline]
    pub fn latch(&self) -> Option<RtaSel> {
        let r = self.request.load(Relaxed);
        if self.active.load(Relaxed) != r {
            self.active.store(r, Relaxed);
            self.epoch.fetch_add(1, Release);
        }
        RtaSel::unpack(r)
    }

    /// The selection of the current cycle. RT-safe.
    #[inline]
    pub fn active(&self) -> Option<RtaSel> {
        RtaSel::unpack(self.active.load(Relaxed))
    }

    pub fn epoch(&self) -> u32 {
        self.epoch.load(Acquire)
    }

    /// Append a stereo block as its mono sum `(L+R)/2` (single producer per cycle). RT-safe.
    #[inline]
    pub fn push_stereo(&self, l: &[f32], r: &[f32]) {
        let n = l.len().min(r.len());
        let w = self.write.load(Relaxed);
        let mask = self.ring.len() - 1;
        for i in 0..n {
            let v = 0.5 * (l[i] + r[i]);
            self.ring[(w as usize + i) & mask].store(v.to_bits(), Relaxed);
        }
        self.write.store(w + n as u64, Release);
    }

    /// Append a mono block. RT-safe.
    #[inline]
    pub fn push_mono(&self, x: &[f32]) {
        self.push_stereo(x, x);
    }

    /// Samples written so far.
    pub fn written(&self) -> u64 {
        self.write.load(Acquire)
    }

    /// An audio session feeds the ring at `fs` (0 = detached).
    pub fn set_sample_rate(&self, fs: u32) {
        self.sample_rate.store(fs, Relaxed);
        self.attached.store(fs > 0, Release);
    }

    pub fn sample_rate(&self) -> u32 {
        self.sample_rate.load(Relaxed)
    }

    pub fn is_attached(&self) -> bool {
        self.attached.load(Acquire)
    }
}

/// A strip's view of the feed: pushes when the latched selection is this strip at a given tap.
#[derive(Clone, Debug)]
pub struct RtaProbe {
    feed: std::sync::Arc<RtaFeed>,
    strip: usize,
}

impl RtaProbe {
    /// Probe for flat strip index `strip` ([`crate::routing::Strip::index`]).
    pub fn new(feed: std::sync::Arc<RtaFeed>, strip: usize) -> Self {
        Self { feed, strip }
    }

    /// The tap wanted from this strip this cycle, if any. RT-safe.
    #[inline]
    pub fn wanted(&self) -> Option<RtaTap> {
        let s = self.feed.active()?;
        (s.strip() == Some(self.strip)).then_some(s.tap)
    }

    #[inline]
    pub fn push(&self, l: &[f32], r: &[f32]) {
        self.feed.push_stereo(l, r);
    }

    pub fn feed(&self) -> &std::sync::Arc<RtaFeed> {
        &self.feed
    }
}

/// Consumer position in an [`RtaFeed`] ring (one reader).
#[derive(Debug, Default)]
pub struct RtaReader {
    pos: u64,
    lapped: u64,
}

impl RtaReader {
    /// Start at the newest sample.
    pub fn at_end(feed: &RtaFeed) -> Self {
        Self {
            pos: feed.written(),
            lapped: 0,
        }
    }

    /// Times the reader fell more than half a ring behind (or was overwritten) and skipped ahead.
    pub fn lapped(&self) -> u64 {
        self.lapped
    }

    /// Copy up to `out.len()` new samples (oldest first); returns the count. Never allocates.
    pub fn read(&mut self, feed: &RtaFeed, out: &mut [f32]) -> usize {
        let cap = feed.ring.len() as u64;
        let w = feed.written();
        if w < self.pos {
            self.pos = w; // feed restarted
        }
        if w - self.pos > cap / 2 {
            self.pos = w - cap / 2;
            self.lapped += 1;
        }
        let n = ((w - self.pos) as usize).min(out.len());
        let mask = feed.ring.len() - 1;
        for (i, o) in out[..n].iter_mut().enumerate() {
            *o = f32::from_bits(feed.ring[(self.pos as usize + i) & mask].load(Relaxed));
        }
        // a producer that wrapped onto the copied range invalidates it
        if feed.written() - self.pos > cap {
            self.pos = feed.written();
            self.lapped += 1;
            return 0;
        }
        self.pos += n as u64;
        n
    }
}

/// `cfg/rta/rtadet`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum RtaDetector {
    #[default]
    Peak,
    Rms,
    Avg,
}

impl RtaDetector {
    pub fn from_index(i: u32) -> Self {
        match i {
            1 => RtaDetector::Rms,
            2 => RtaDetector::Avg,
            _ => RtaDetector::Peak,
        }
    }
}

/// `cfg/rta/rtadecay`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum RtaDecay {
    Slow,
    #[default]
    Med,
    Fast,
}

impl RtaDecay {
    pub fn from_index(i: u32) -> Self {
        match i {
            0 => RtaDecay::Slow,
            2 => RtaDecay::Fast,
            _ => RtaDecay::Med,
        }
    }

    /// Fall rate (dB/s) [H].
    pub fn db_per_s(self) -> f32 {
        match self {
            RtaDecay::Slow => 6.0,
            RtaDecay::Med => 12.0,
            RtaDecay::Fast => 24.0,
        }
    }
}

/// -3 dB half-width parameter of one of two identical cascaded band-pass sections: the pair's
/// response `1/(1+u²)` is -3 dB at `u = sqrt(sqrt(2) - 1)`.
const CASCADE_U: f64 = 0.643_594_252_905_582_6;

/// The band filter bank, detector and decay (non-RT; see the module docs).
pub struct RtaAnalyser {
    fs: f64,
    /// Per band: b0 (= -b2), a1, a2 of each (identical) section; `active` = below 0.45 fs.
    b0: [f64; RTA_BANDS],
    a1: [f64; RTA_BANDS],
    a2: [f64; RTA_BANDS],
    active: usize,
    /// TDF-II states of section 1 and 2.
    s1: [f64; RTA_BANDS],
    s2: [f64; RTA_BANDS],
    s3: [f64; RTA_BANDS],
    s4: [f64; RTA_BANDS],
    peak: [f64; RTA_BANDS],
    sum_sq: [f64; RTA_BANDS],
    sum_abs: [f64; RTA_BANDS],
    n: u64,
    disp: [f32; RTA_BANDS],
    detector: RtaDetector,
    decay: RtaDecay,
}

impl std::fmt::Debug for RtaAnalyser {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("RtaAnalyser")
            .field("fs", &self.fs)
            .field("bands", &self.active)
            .field("detector", &self.detector)
            .field("decay", &self.decay)
            .finish()
    }
}

impl RtaAnalyser {
    pub fn new(fs: f64) -> Self {
        let mut a = Self {
            fs: 0.0,
            b0: [0.0; RTA_BANDS],
            a1: [0.0; RTA_BANDS],
            a2: [0.0; RTA_BANDS],
            active: 0,
            s1: [0.0; RTA_BANDS],
            s2: [0.0; RTA_BANDS],
            s3: [0.0; RTA_BANDS],
            s4: [0.0; RTA_BANDS],
            peak: [0.0; RTA_BANDS],
            sum_sq: [0.0; RTA_BANDS],
            sum_abs: [0.0; RTA_BANDS],
            n: 0,
            disp: [RTA_FLOOR_DB; RTA_BANDS],
            detector: RtaDetector::Peak,
            decay: RtaDecay::Med,
        };
        a.set_sample_rate(fs);
        a
    }

    pub fn sample_rate(&self) -> f64 {
        self.fs
    }

    /// Bands analysed at this rate (the rest read as silence).
    pub fn active_bands(&self) -> usize {
        self.active
    }

    /// Redesign the filters for `fs` and reset.
    pub fn set_sample_rate(&mut self, fs: f64) {
        self.fs = fs;
        let bw = 2f64.powf(1.0 / 12.0);
        let q_band = bw.sqrt() / (bw - 1.0);
        let q = CASCADE_U * q_band;
        self.active = 0;
        for k in 0..RTA_BANDS {
            let f0 = band_freq(k);
            if fs <= 0.0 || f0 >= 0.45 * fs {
                self.b0[k] = 0.0;
                self.a1[k] = 0.0;
                self.a2[k] = 0.0;
                continue;
            }
            self.active = k + 1;
            let w0 = std::f64::consts::TAU * f0 / fs;
            let alpha = w0.sin() / (2.0 * q);
            let a0 = 1.0 + alpha;
            self.b0[k] = alpha / a0;
            self.a1[k] = -2.0 * w0.cos() / a0;
            self.a2[k] = (1.0 - alpha) / a0;
        }
        self.reset();
    }

    pub fn set_detector(&mut self, d: RtaDetector) {
        self.detector = d;
    }

    pub fn set_decay(&mut self, d: RtaDecay) {
        self.decay = d;
    }

    /// Clear the filter states, the open interval and the display.
    pub fn reset(&mut self) {
        for k in 0..RTA_BANDS {
            self.s1[k] = 0.0;
            self.s2[k] = 0.0;
            self.s3[k] = 0.0;
            self.s4[k] = 0.0;
        }
        self.clear_interval();
        self.disp = [RTA_FLOOR_DB; RTA_BANDS];
    }

    fn clear_interval(&mut self) {
        self.peak = [0.0; RTA_BANDS];
        self.sum_sq = [0.0; RTA_BANDS];
        self.sum_abs = [0.0; RTA_BANDS];
        self.n = 0;
    }

    /// Run `x` through the bank, accumulating the open interval. Never allocates.
    pub fn process(&mut self, x: &[f32]) {
        let m = self.active;
        let (b0, a1, a2) = (&self.b0[..m], &self.a1[..m], &self.a2[..m]);
        let (s1, s2) = (&mut self.s1[..m], &mut self.s2[..m]);
        let (s3, s4) = (&mut self.s3[..m], &mut self.s4[..m]);
        let (pk, sq, ab) = (
            &mut self.peak[..m],
            &mut self.sum_sq[..m],
            &mut self.sum_abs[..m],
        );
        for &v in x {
            let v = v as f64;
            // band-independent inner loop over struct-of-arrays: vectorises across bands
            for k in 0..m {
                // section 1 (TDF-II, b1 = 0, b2 = -b0)
                let y = b0[k] * v + s1[k];
                s1[k] = s2[k] - a1[k] * y;
                s2[k] = -b0[k] * v - a2[k] * y;
                // section 2
                let z = b0[k] * y + s3[k];
                s3[k] = s4[k] - a1[k] * z;
                s4[k] = -b0[k] * y - a2[k] * z;
                let az = z.abs();
                pk[k] = pk[k].max(az);
                sq[k] += z * z;
                ab[k] += az;
            }
        }
        self.n += x.len() as u64;
        // keep denormals out of the long decays
        for k in 0..m {
            for s in [&mut s1[k], &mut s2[k], &mut s3[k], &mut s4[k]] {
                if s.abs() < 1e-30 {
                    *s = 0.0;
                }
            }
        }
    }

    /// Close the interval: detector level per band (dBFS), then decay over `dt` seconds.
    /// Returns the displayed levels. An interval without samples only decays.
    pub fn frame(&mut self, dt: f32) -> &[f32; RTA_BANDS] {
        let fall = self.decay.db_per_s() * dt.max(0.0);
        let n = self.n as f64;
        for k in 0..RTA_BANDS {
            let lvl = if self.n == 0 || k >= self.active {
                RTA_FLOOR_DB
            } else {
                let db = match self.detector {
                    RtaDetector::Peak => 20.0 * self.peak[k].log10(),
                    RtaDetector::Rms => 10.0 * (self.sum_sq[k] / n).log10(),
                    RtaDetector::Avg => 20.0 * (self.sum_abs[k] / n).log10(),
                };
                if db.is_finite() {
                    (db as f32).max(RTA_FLOOR_DB)
                } else {
                    RTA_FLOOR_DB
                }
            };
            let d = &mut self.disp[k];
            *d = if lvl >= *d { lvl } else { lvl.max(*d - fall) };
        }
        self.clear_interval();
        &self.disp
    }

    /// Displayed levels of the last frame (dBFS).
    pub fn levels(&self) -> &[f32; RTA_BANDS] {
        &self.disp
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sine(f: f64, fs: f64, amp: f32, n: usize) -> Vec<f32> {
        (0..n)
            .map(|i| amp * (std::f64::consts::TAU * f * i as f64 / fs).sin() as f32)
            .collect()
    }

    /// Settle on a sine, then measure one 50 ms frame with no decay carry-over.
    fn measure(fs: f64, f: f64, amp: f32, det: RtaDetector) -> [f32; RTA_BANDS] {
        let mut a = RtaAnalyser::new(fs);
        a.set_detector(det);
        // settle 1.5 s (the 20 Hz bands ring for ~0.3 s), then measure 100 ms
        let x = sine(f, fs, amp, (fs * 1.6) as usize);
        a.process(&x[..(fs * 1.5) as usize]);
        a.frame(0.0);
        a.reset_display();
        a.process(&x[(fs * 1.5) as usize..]);
        *a.frame(0.0)
    }

    impl RtaAnalyser {
        fn reset_display(&mut self) {
            self.disp = [RTA_FLOOR_DB; RTA_BANDS];
        }
    }

    #[test]
    fn band_layout() {
        assert!((band_freq(0) - 20.60).abs() < 0.01, "{}", band_freq(0));
        assert_eq!(band_freq(RTA_A4_BAND), 440.0);
        assert!((band_freq(119) - 19912.1).abs() < 0.5, "{}", band_freq(119));
        let a = RtaAnalyser::new(48_000.0);
        assert_eq!(a.active_bands(), RTA_BANDS);
        let a = RtaAnalyser::new(32_000.0);
        assert!(a.active_bands() < RTA_BANDS);
        assert!(band_freq(a.active_bands() - 1) < 0.45 * 32_000.0);
    }

    #[test]
    fn band_accuracy_at_centres() {
        let fs = 48_000.0;
        for k in [5usize, 30, RTA_A4_BAND, 80, 110, 119] {
            let l = measure(fs, band_freq(k), 1.0, RtaDetector::Peak);
            let (best, _) = l
                .iter()
                .enumerate()
                .max_by(|a, b| a.1.total_cmp(b.1))
                .unwrap();
            assert_eq!(best, k, "band {k}");
            assert!(l[k].abs() < 0.5, "band {k} reads {} dB", l[k]);
            for (j, &v) in l.iter().enumerate() {
                let d = j.abs_diff(k);
                if d == 1 {
                    assert!(v < l[k] - 5.0, "band {k}: neighbour {j} {v}");
                } else if d >= 3 {
                    assert!(v < l[k] - 18.0, "band {k}: band {j} {v}");
                }
            }
        }
    }

    #[test]
    fn detectors_and_level_tracking() {
        let fs = 48_000.0;
        let f = band_freq(RTA_A4_BAND);
        let k = RTA_A4_BAND;
        let p = measure(fs, f, 0.1, RtaDetector::Peak)[k];
        assert!((p + 20.0).abs() < 0.3, "peak {p}");
        let r = measure(fs, f, 1.0, RtaDetector::Rms)[k];
        assert!((r + 3.01).abs() < 0.3, "rms {r}");
        let a = measure(fs, f, 1.0, RtaDetector::Avg)[k];
        assert!((a + 3.92).abs() < 0.3, "avg {a}");
        // 44.1 kHz designs too
        let p = measure(44_100.0, band_freq(90), 0.5, RtaDetector::Peak)[90];
        assert!((p + 6.02).abs() < 0.4, "44.1k {p}");
    }

    #[test]
    fn decay_is_linear_in_db() {
        let fs = 48_000.0;
        let mut a = RtaAnalyser::new(fs);
        a.set_decay(RtaDecay::Fast);
        a.process(&sine(1000.0, fs, 1.0, 24_000));
        let k = (0..RTA_BANDS)
            .max_by(|&x, &y| a.peak[x].total_cmp(&a.peak[y]))
            .unwrap();
        a.frame(0.05);
        // the first silent interval still holds the filter's ring-down peak
        a.process(&vec![0.0; 2400]);
        let start = a.frame(0.05)[k];
        // then the band is far below the display: it decays at the FAST rate
        a.process(&vec![0.0; 2400]);
        let next = a.frame(0.05)[k];
        assert!(
            (start - next - 24.0 * 0.05).abs() < 1e-3,
            "{start} -> {next}"
        );
        for _ in 0..200 {
            a.frame(0.05);
        }
        assert_eq!(a.levels()[k], RTA_FLOOR_DB);
    }

    #[test]
    fn ring_single_producer_reader() {
        let feed = RtaFeed::new(256);
        assert_eq!(feed.capacity(), 256);
        let mut rd = RtaReader::at_end(&feed);
        let mut out = [0.0f32; 512];
        assert_eq!(rd.read(&feed, &mut out), 0);
        feed.push_stereo(&[1.0, 2.0, 3.0], &[3.0, 2.0, 1.0]);
        assert_eq!(rd.read(&feed, &mut out), 3);
        assert_eq!(&out[..3], &[2.0, 2.0, 2.0]);
        // lapped: keeps the newest half ring
        for i in 0..10 {
            feed.push_mono(&[i as f32; 64]);
        }
        let n = rd.read(&feed, &mut out);
        assert_eq!(n, 128);
        assert_eq!(rd.lapped(), 1);
        assert_eq!(out[0], 8.0);
        assert_eq!(out[127], 9.0);
    }

    #[test]
    fn selection_latch_and_probe() {
        let feed = RtaFeed::shared();
        assert_eq!(feed.latch(), None);
        let e0 = feed.epoch();
        feed.request(Some(RtaSel {
            src: 3,
            tap: RtaTap::PreEq,
        }));
        assert_eq!(feed.active(), None, "not before the latch");
        let s = feed.latch().unwrap();
        assert_eq!(s.strip(), Some(2));
        assert_eq!(feed.epoch(), e0 + 1);
        feed.latch();
        assert_eq!(feed.epoch(), e0 + 1, "unchanged selection keeps the epoch");
        let ch3 = RtaProbe::new(feed.clone(), 2);
        let ch4 = RtaProbe::new(feed.clone(), 3);
        assert_eq!(ch3.wanted(), Some(RtaTap::PreEq));
        assert_eq!(ch4.wanted(), None);
        let solo = RtaSel {
            src: 3,
            tap: RtaTap::Solo,
        };
        assert_eq!(solo.strip(), None);
        assert_eq!(
            RtaSel {
                src: 0,
                tap: RtaTap::In
            }
            .strip(),
            None
        );
    }

    /// Analyser cost (printed): one second of 48 kHz audio through all 120 bands.
    #[test]
    fn analyser_cost() {
        let fs = 48_000.0;
        let mut a = RtaAnalyser::new(fs);
        let x = sine(997.0, fs, 0.5, 48_000);
        let t = std::time::Instant::now();
        a.process(&x);
        a.frame(1.0);
        let el = t.elapsed();
        eprintln!("rta analyser: 1 s of 48 kHz audio in {el:?}");
    }
}
