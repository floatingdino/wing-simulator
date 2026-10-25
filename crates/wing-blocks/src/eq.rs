//! Standard WING EQ (`eq.mdl = STD`, "WING EQ", models.md §4) and the channel 3-band PRE EQ (`peq`).
//!
//! Strip variants ([`EqVariant`], WING-Edit tables A/B/C):
//! - **Channel** (`xseries`): low band (`lg/lf/lq` + `leq` PEQ/SHV), peaking bands `1..4`, high band
//!   (`hg/hf/hq` + `heq` PEQ/SHV).
//! - **Aux** (`xseries`): as channel, `leq/heq` additionally offer CUT.
//! - **Bus / main / matrix** (`xseries_bus`): low band, peaking bands `1..6`, high band, `tilt` -6..+6 dB; `leq/heq`
//!   offer PEQ, SHV, CUT, BW6, BW12, BS12, LR12, BW18, BW24, BS24, LR24, BW48, LR48.
//! - Wrapper: `on` (default off), `mix` 0..125 % (default 100).
//! - [`PreEq`]: channel `peq` ("PRE EQ"): `on` + 3 peaking bands `1..3` g/f/q (same ranges as the main EQ).
//!   Placement (signal-flow.md §2, [I]) is directly before the EQ model inside the E block; whether `eq.mix`
//!   also covers it is an open question (signal-flow.md §13.2), so it is a separate block with its own `on`.
//!
//! Ranges: gain -15..+15 dB, freq 20..20000 Hz, Q 0.442..10 (WING-Edit 3.3.4 / T003 schema); values are clamped.
//!
//! Response conventions (all bilinear transforms of analog prototypes, pre-warped at the band frequency, so the
//! band frequency lands exactly where the analog prototype puts it):
//! - PEQ = RBJ cookbook peaking EQ with Q used directly: `H(s) = (s^2 + s*A/Q + 1)/(s^2 + s/(A*Q) + 1)`,
//!   `A = 10^(g/40)`. Boost/cut are mirror images, the gain at f is exact, and Q is the RBJ "bandwidth" Q
//!   (Q = 1.414 <-> 1 octave between the half-gain(dB) points; WING's own default 1.41 suggests this octave
//!   convention). **Assumption, verify** against WING-Edit reference curves (T03C).
//! - SHV = RBJ low/high shelf with the band Q as RBJ shelf Q (gain/2 at f; Q > 0.707 gives the usual
//!   overshoot/undershoot "bump"). **Assumption, verify** (T03C).
//! - CUT = 2nd-order RBJ high-pass (low band) / low-pass (high band) using the band Q (resonant cut). **Assumption**.
//! - BWn / LRn = Butterworth / Linkwitz-Riley of slope n dB/oct (gain and Q ignored), BSn = Bessel low-pass/high-pass
//!   normalised to -3.01 dB at f (n = 12, 24). The Bessel normalisation (-3 dB vs phase-delay) is an
//!   **assumption**; BW is -3.01 dB at f, LR -6.02 dB at f.
//! - Tilt (bus/main/mtx): 1st-order tilt `H(s) = r (s + 1/r)/(s + r)`, `r = 10^(tilt/40)`, pivot
//!   [`TILT_PIVOT_HZ`]: 0 dB at the pivot, -tilt/2 dB at DC and +tilt/2 dB at Nyquist (positive = brighter).
//!   Pivot and span convention are **assumptions** (models.md TILT tool: pivot not recovered).
//! - `mix`: dry/wet blend `y = x + m (eq(x) - x)`, `m = mix/100` (125 % extrapolates = exaggerated EQ).
//!
//! Real-time behaviour: f64 coefficients and state (20 Hz shelves/cuts at 48 kHz need it). Gain (dB), log2(f) and
//! log2(Q) follow 20 ms one-pole smoothers; while gliding, coefficients are redesigned every [`SUB_BLOCK`]
//! samples and interpolated linearly per sample in between (convex combination of stable 1st/2nd-order sections
//! is stable), so fast automation is zipper- and click-free. A band-type change crossfades over 10 ms from the old
//! filter to the new one (fresh state); `on`/`mix` changes ramp over a 5 ms one-pole. Bands at 0 dB (PEQ/SHV)
//! and a switched-off EQ cost nothing. `prepare` is the only call that may allocate (it does not, currently).

use core::f64::consts::PI;
use wing_dsp::biquad::{butterworth, linkwitz_riley, CutKind, Sections};
use wing_dsp::{Coeffs, Complex, LinearRamp, OnePoleSmoother};

/// Coefficient redesign interval while a parameter glides (samples).
pub const SUB_BLOCK: usize = 16;
/// Tilt pivot frequency (assumption, see module doc).
pub const TILT_PIVOT_HZ: f64 = 1000.0;
/// Maximum number of peaking bands (bus/main/mtx).
pub const MAX_PEAK_BANDS: usize = 6;

pub const GAIN_MIN_DB: f32 = -15.0;
pub const GAIN_MAX_DB: f32 = 15.0;
pub const FREQ_MIN_HZ: f32 = 20.0;
pub const FREQ_MAX_HZ: f32 = 20_000.0;
pub const Q_MIN: f32 = 0.442;
pub const Q_MAX: f32 = 10.0;
pub const TILT_MAX_DB: f32 = 6.0;
pub const MIX_MAX_PCT: f32 = 125.0;

const PARAM_TAU_S: f64 = 0.02;
const WET_TAU_S: f64 = 0.005;
const XFADE_S: f64 = 0.01;
const MAX_SECTIONS: usize = 4;

/// Which EQ table the strip uses.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum EqVariant {
    /// Input channels (table A).
    #[default]
    Channel,
    /// Aux channels (table B).
    Aux,
    /// Bus, main and matrix (table C, `xseries_bus`).
    Bus,
}

impl EqVariant {
    /// Number of peaking bands between the low and high band.
    pub fn peak_bands(self) -> usize {
        match self {
            EqVariant::Channel | EqVariant::Aux => 4,
            EqVariant::Bus => 6,
        }
    }

    pub fn has_tilt(self) -> bool {
        self == EqVariant::Bus
    }

    /// Whether the `leq`/`heq` enum of this variant offers `t`.
    pub fn allows(self, t: BandType) -> bool {
        match self {
            EqVariant::Channel => matches!(t, BandType::Peq | BandType::Shv),
            EqVariant::Aux => matches!(t, BandType::Peq | BandType::Shv | BandType::Cut),
            EqVariant::Bus => true,
        }
    }
}

/// Low/high band type (`leq` / `heq`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum BandType {
    /// Peaking band.
    Peq,
    /// Shelf (factory default).
    #[default]
    Shv,
    /// Resonant 12 dB/oct cut using the band Q.
    Cut,
    Bw6,
    Bw12,
    Bs12,
    Lr12,
    Bw18,
    Bw24,
    Bs24,
    Lr24,
    Bw48,
    Lr48,
}

impl BandType {
    pub const ALL: [BandType; 13] = [
        BandType::Peq,
        BandType::Shv,
        BandType::Cut,
        BandType::Bw6,
        BandType::Bw12,
        BandType::Bs12,
        BandType::Lr12,
        BandType::Bw18,
        BandType::Bw24,
        BandType::Bs24,
        BandType::Lr24,
        BandType::Bw48,
        BandType::Lr48,
    ];

    /// Snapshot enum string.
    pub fn key(self) -> &'static str {
        match self {
            BandType::Peq => "PEQ",
            BandType::Shv => "SHV",
            BandType::Cut => "CUT",
            BandType::Bw6 => "BW6",
            BandType::Bw12 => "BW12",
            BandType::Bs12 => "BS12",
            BandType::Lr12 => "LR12",
            BandType::Bw18 => "BW18",
            BandType::Bw24 => "BW24",
            BandType::Bs24 => "BS24",
            BandType::Lr24 => "LR24",
            BandType::Bw48 => "BW48",
            BandType::Lr48 => "LR48",
        }
    }

    pub fn from_key(s: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|t| t.key() == s)
    }

    /// PEQ and SHV use the gain (and are identity at 0 dB); the cut types ignore it.
    pub fn uses_gain(self) -> bool {
        matches!(self, BandType::Peq | BandType::Shv)
    }

    /// PEQ, SHV and CUT use Q; fixed-slope filters ignore it.
    pub fn uses_q(self) -> bool {
        matches!(self, BandType::Peq | BandType::Shv | BandType::Cut)
    }
}

/// One band in engineering units.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BandParams {
    /// -15 .. +15 dB.
    pub gain_db: f32,
    /// 20 .. 20000 Hz.
    pub freq: f32,
    /// 0.442 .. 10.
    pub q: f32,
}

impl BandParams {
    pub const fn new(gain_db: f32, freq: f32, q: f32) -> Self {
        Self { gain_db, freq, q }
    }

    pub fn clamped(self) -> Self {
        Self {
            gain_db: self.gain_db.clamp(GAIN_MIN_DB, GAIN_MAX_DB),
            freq: self.freq.clamp(FREQ_MIN_HZ, FREQ_MAX_HZ),
            q: self.q.clamp(Q_MIN, Q_MAX),
        }
    }
}

/// Default Q (grid value displayed as 1.00).
const DEF_Q: f32 = 0.997_970_2;

/// Parameters of the STD EQ (all variants; fields a variant lacks are ignored).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EqParams {
    pub on: bool,
    /// 0 .. 125 %.
    pub mix: f32,
    pub low_type: BandType,
    pub low: BandParams,
    /// Peaking bands `1..`; channel/aux use the first 4.
    pub bands: [BandParams; MAX_PEAK_BANDS],
    pub high_type: BandType,
    pub high: BandParams,
    /// -6 .. +6 dB (bus/main/mtx only).
    pub tilt_db: f32,
}

impl Default for EqParams {
    fn default() -> Self {
        Self::defaults(EqVariant::Channel)
    }
}

impl EqParams {
    /// WING-Edit 3.3.4 defaults (T003 schema; frequencies on the 961-step log grid).
    pub fn defaults(variant: EqVariant) -> Self {
        let b = |f| BandParams::new(0.0, f, DEF_Q);
        match variant {
            EqVariant::Channel | EqVariant::Aux => Self {
                on: false,
                mix: 100.0,
                low_type: BandType::Shv,
                low: b(80.196_42),
                bands: [
                    b(200.0),
                    b(601.4),
                    b(1_499.788_3),
                    b(3_990.524_4),
                    b(1_000.0),
                    b(1_000.0),
                ],
                high_type: BandType::Shv,
                high: b(11_999.275),
                tilt_db: 0.0,
            },
            EqVariant::Bus => Self {
                on: false,
                mix: 100.0,
                low_type: BandType::Shv,
                low: b(60.138_836),
                bands: [
                    b(129.876_34),
                    b(299.247_16),
                    b(699.487_55),
                    b(1_499.788_3),
                    b(2_992.471),
                    b(6_013.884),
                ],
                high_type: BandType::Shv,
                high: b(11_999.275),
                tilt_db: 0.0,
            },
        }
    }

    /// Clamp to the WING ranges; band types the variant does not offer fall back to SHV, tilt is zeroed when
    /// the variant has none.
    pub fn clamped(mut self, variant: EqVariant) -> Self {
        self.mix = self.mix.clamp(0.0, MIX_MAX_PCT);
        self.low = self.low.clamped();
        self.high = self.high.clamped();
        for b in &mut self.bands {
            *b = b.clamped();
        }
        if !variant.allows(self.low_type) {
            self.low_type = BandType::Shv;
        }
        if !variant.allows(self.high_type) {
            self.high_type = BandType::Shv;
        }
        self.tilt_db = if variant.has_tilt() {
            self.tilt_db.clamp(-TILT_MAX_DB, TILT_MAX_DB)
        } else {
            0.0
        };
        self
    }
}

/// Channel PRE EQ (`peq`) parameters.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PreEqParams {
    pub on: bool,
    pub bands: [BandParams; 3],
}

impl Default for PreEqParams {
    /// WING-Edit 3.3.4 defaults (T003 schema).
    fn default() -> Self {
        Self {
            on: false,
            bands: [
                BandParams::new(0.0, 99.685_43, DEF_Q),
                BandParams::new(0.0, 999.250_5, DEF_Q),
                BandParams::new(0.0, 10_016.527, DEF_Q),
            ],
        }
    }
}

// ---------------------------------------------------------------------------------------------------------
// Design
// ---------------------------------------------------------------------------------------------------------

/// Position of a band in the strip (decides cut direction and shelf side).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Role {
    Low,
    Peak,
    High,
    Tilt,
}

fn single(c: Coeffs) -> Sections {
    let mut coeffs = [Coeffs::IDENTITY; MAX_SECTIONS];
    coeffs[0] = c;
    Sections { coeffs, len: 1 }
}

/// Bessel (-3.01 dB at fc) pole-pair (frequency factor relative to fc, Q) for orders 2 and 4.
const BESSEL2: [(f64, f64); 1] = [(1.272_019_649_514_069, 0.577_350_269_189_625_8)];
const BESSEL4: [(f64, f64); 2] = [
    (1.430_171_559_993_991_5, 0.521_934_581_668_980_1),
    (1.603_357_516_216_973, 0.805_538_281_841_664_9),
];

/// Bessel cut built with a common pre-warp at `fc` (exact bilinear transform of the analog prototype).
fn bessel(kind: CutKind, fs: f64, fc: f64, poles: &[(f64, f64)]) -> Sections {
    let k = (PI * fc.clamp(1e-3, fs * 0.4999) / fs).tan();
    let mut coeffs = [Coeffs::IDENTITY; MAX_SECTIONS];
    for (i, &(w, q)) in poles.iter().enumerate() {
        let wa = match kind {
            CutKind::Lowpass => k * w,
            CutKind::Highpass => k / w,
        };
        let f = fs / PI * wa.atan();
        coeffs[i] = match kind {
            CutKind::Lowpass => Coeffs::lowpass(fs, f, q),
            CutKind::Highpass => Coeffs::highpass(fs, f, q),
        };
    }
    Sections {
        coeffs,
        len: poles.len(),
    }
}

/// First-order tilt around `pivot`: `r` at Nyquist, `1/r` at DC, 1 at the pivot.
fn tilt(fs: f64, pivot: f64, tilt_db: f64) -> Coeffs {
    let r = 10f64.powf(tilt_db / 40.0);
    let k = (PI * pivot / fs).tan();
    Coeffs::from_raw(r + k, k - r, 0.0, 1.0 + k * r, k * r - 1.0, 0.0)
}

fn design(role: Role, ty: BandType, fs: f64, f: f64, q: f64, g: f64) -> Sections {
    let (cut, shelf_low) = match role {
        Role::Peak => return single(Coeffs::peak(fs, f, q, g)),
        Role::Tilt => return single(tilt(fs, TILT_PIVOT_HZ, g)),
        Role::Low => (CutKind::Highpass, true),
        Role::High => (CutKind::Lowpass, false),
    };
    match ty {
        BandType::Peq => single(Coeffs::peak(fs, f, q, g)),
        BandType::Shv if shelf_low => single(Coeffs::low_shelf(fs, f, q, g)),
        BandType::Shv => single(Coeffs::high_shelf(fs, f, q, g)),
        BandType::Cut => single(match cut {
            CutKind::Highpass => Coeffs::highpass(fs, f, q),
            CutKind::Lowpass => Coeffs::lowpass(fs, f, q),
        }),
        BandType::Bw6 => butterworth(cut, fs, f, 1),
        BandType::Bw12 => butterworth(cut, fs, f, 2),
        BandType::Bw18 => butterworth(cut, fs, f, 3),
        BandType::Bw24 => butterworth(cut, fs, f, 4),
        BandType::Bw48 => butterworth(cut, fs, f, 8),
        BandType::Lr12 => linkwitz_riley(cut, fs, f, 2),
        BandType::Lr24 => linkwitz_riley(cut, fs, f, 4),
        BandType::Lr48 => linkwitz_riley(cut, fs, f, 8),
        BandType::Bs12 => bessel(cut, fs, f, &BESSEL2),
        BandType::Bs24 => bessel(cut, fs, f, &BESSEL4),
    }
}

/// Analytic complex response of one band (for UI curves and tests).
fn band_response(role: Role, ty: BandType, fs: f64, p: BandParams, f: f64) -> Complex {
    design(role, ty, fs, p.freq as f64, p.q as f64, p.gain_db as f64).response(fs, f)
}

// ---------------------------------------------------------------------------------------------------------
// Runtime
// ---------------------------------------------------------------------------------------------------------

type Buf = [[f64; SUB_BLOCK]; 2];

/// Stereo TDF-II section with per-sample linear coefficient ramp.
#[derive(Clone, Copy, Debug, Default)]
struct Sec {
    /// b0, b1, b2, a1, a2.
    c: [f64; 5],
    d: [f64; 5],
    t: [f64; 5],
    s: [[f64; 2]; 2],
}

fn carr(c: &Coeffs) -> [f64; 5] {
    [c.b0, c.b1, c.b2, c.a1, c.a2]
}

impl Sec {
    #[inline(always)]
    fn tick(c: &[f64; 5], s: &mut [f64; 2], x: f64) -> f64 {
        let y = c[0] * x + s[0];
        s[0] = c[1] * x - c[3] * y + s[1];
        s[1] = c[2] * x - c[4] * y;
        y
    }
}

#[derive(Clone, Copy, Debug, Default)]
struct Chain {
    secs: [Sec; MAX_SECTIONS],
    len: usize,
    ramping: bool,
}

impl Chain {
    /// Load coefficients immediately with zero state.
    fn load(&mut self, s: &Sections) {
        self.len = s.len;
        self.ramping = false;
        for (sec, c) in self.secs.iter_mut().zip(s.as_slice()) {
            sec.c = carr(c);
            sec.t = sec.c;
            sec.s = [[0.0; 2]; 2];
        }
    }

    /// Ramp linearly to `s` over the next `n` samples (same section count).
    fn ramp_to(&mut self, s: &Sections, n: usize) {
        debug_assert_eq!(s.len, self.len);
        let inv = 1.0 / n as f64;
        for (sec, c) in self.secs.iter_mut().zip(s.as_slice()) {
            sec.t = carr(c);
            for i in 0..5 {
                sec.d[i] = (sec.t[i] - sec.c[i]) * inv;
            }
        }
        self.ramping = true;
    }

    fn run(&mut self, buf: &mut Buf, n: usize) {
        let secs = &mut self.secs[..self.len];
        if self.ramping {
            let [bl, br] = buf;
            for (xl, xr) in bl[..n].iter_mut().zip(br[..n].iter_mut()) {
                let (mut l, mut r) = (*xl, *xr);
                for sec in secs.iter_mut() {
                    for k in 0..5 {
                        sec.c[k] += sec.d[k];
                    }
                    let [s0, s1] = &mut sec.s;
                    l = Sec::tick(&sec.c, s0, l);
                    r = Sec::tick(&sec.c, s1, r);
                }
                *xl = l;
                *xr = r;
            }
            for sec in secs.iter_mut() {
                sec.c = sec.t;
            }
            self.ramping = false;
        } else {
            for sec in secs.iter_mut() {
                for (ch, b) in buf.iter_mut().enumerate() {
                    let s = &mut sec.s[ch];
                    for x in &mut b[..n] {
                        *x = Sec::tick(&sec.c, s, *x);
                    }
                }
            }
        }
    }

    fn flush_denormals(&mut self) {
        for sec in &mut self.secs[..self.len] {
            for s in sec.s.iter_mut().flatten() {
                if s.abs() < 1e-25 {
                    *s = 0.0;
                }
            }
        }
    }
}

/// One band: smoothed parameters, two chains (current + crossfade source).
#[derive(Clone, Copy, Debug)]
struct Band {
    role: Role,
    ty: BandType,
    gain: OnePoleSmoother,
    log2f: OnePoleSmoother,
    log2q: OnePoleSmoother,
    chains: [Chain; 2],
    cur: usize,
    /// Crossfade from `chains[1 - cur]` (or identity when `fade_from_identity`) to `chains[cur]`.
    fade: LinearRamp,
    fading: bool,
    fade_from_identity: bool,
    active: bool,
}

impl Band {
    fn new(role: Role) -> Self {
        Self {
            role,
            ty: if role == Role::Peak || role == Role::Tilt {
                BandType::Peq
            } else {
                BandType::Shv
            },
            gain: OnePoleSmoother::new(0.0, PARAM_TAU_S, 48_000.0),
            log2f: OnePoleSmoother::new(10.0, PARAM_TAU_S, 48_000.0),
            log2q: OnePoleSmoother::new(0.0, PARAM_TAU_S, 48_000.0),
            chains: [Chain::default(); 2],
            cur: 0,
            fade: LinearRamp::new(1.0, 480),
            fading: false,
            fade_from_identity: false,
            active: false,
        }
    }

    fn set_fs(&mut self, fs: f64) {
        self.gain.set_time(PARAM_TAU_S, fs);
        self.log2f.set_time(PARAM_TAU_S, fs);
        self.log2q.set_time(PARAM_TAU_S, fs);
        self.fade
            .set_ramp_len((XFADE_S * fs).round().max(1.0) as u32);
    }

    fn is_identity_target(&self, ty: BandType) -> bool {
        ty.uses_gain() && self.gain.target() == 0.0
    }

    fn current_design(&self, fs: f64) -> Sections {
        design(
            self.role,
            self.ty,
            fs,
            2f64.powf(self.log2f.value() as f64),
            2f64.powf(self.log2q.value() as f64),
            self.gain.value() as f64,
        )
    }

    fn start_fade(&mut self, from_identity: bool) {
        self.fade.reset(0.0);
        self.fade.set_target(1.0);
        self.fading = true;
        self.fade_from_identity = from_identity;
    }

    fn set(&mut self, ty: BandType, p: BandParams, fs: f64) {
        self.gain.set_target(p.gain_db);
        self.log2f.set_target(p.freq.log2());
        self.log2q.set_target(p.q.log2());
        if !self.active {
            self.ty = ty;
            // Inactive invariant: gain smoother sits at 0 dB; frequency and Q are inaudible, so jump them.
            self.log2f.reset(self.log2f.target());
            self.log2q.reset(self.log2q.target());
            if !self.is_identity_target(ty) {
                self.active = true;
                let d = self.current_design(fs);
                self.chains[self.cur].load(&d);
                if ty.uses_gain() {
                    // Starts at 0 dB = identity coefficients, then glides: seamless.
                    self.fading = false;
                } else {
                    self.start_fade(true);
                }
            }
        } else if ty != self.ty {
            // If a fade is in progress its source is dropped (rare: two type changes within 10 ms).
            self.ty = ty;
            let d = self.current_design(fs);
            self.cur = 1 - self.cur;
            self.chains[self.cur].load(&d);
            self.start_fade(false);
        }
    }

    /// Snap smoothers to targets, clear state.
    fn reset(&mut self, fs: f64) {
        self.gain.reset(self.gain.target());
        self.log2f.reset(self.log2f.target());
        self.log2q.reset(self.log2q.target());
        self.fading = false;
        self.fade.reset(1.0);
        self.active = !self.is_identity_target(self.ty);
        if self.active {
            let d = self.current_design(fs);
            self.chains[self.cur].load(&d);
        }
    }

    fn gliding(&self) -> bool {
        !(self.gain.is_settled(1e-5) && self.log2f.is_settled(1e-6) && self.log2q.is_settled(1e-6))
    }

    /// Process `n <= SUB_BLOCK` samples of `buf` in place.
    fn process(&mut self, buf: &mut Buf, n: usize, fs: f64) {
        if !self.active {
            return;
        }
        if self.gliding() {
            self.gain.advance(n);
            self.log2f.advance(n);
            self.log2q.advance(n);
            if !self.gliding() {
                self.gain.reset(self.gain.target());
                self.log2f.reset(self.log2f.target());
                self.log2q.reset(self.log2q.target());
            }
            let d = self.current_design(fs);
            self.chains[self.cur].ramp_to(&d, n);
        }
        if self.fading {
            let mut src = *buf;
            if !self.fade_from_identity {
                self.chains[1 - self.cur].run(&mut src, n);
            }
            self.chains[self.cur].run(buf, n);
            for i in 0..n {
                let m = self.fade.tick() as f64;
                for ch in 0..2 {
                    buf[ch][i] = src[ch][i] + m * (buf[ch][i] - src[ch][i]);
                }
            }
            if !self.fade.is_ramping() {
                self.fading = false;
            }
        } else {
            self.chains[self.cur].run(buf, n);
        }
    }

    fn end_block(&mut self) {
        if !self.active {
            return;
        }
        self.chains[self.cur].flush_denormals();
        if !self.fading
            && self.ty.uses_gain()
            && self.gain.target() == 0.0
            && self.gain.is_settled(1e-5)
        {
            self.active = false;
            self.gain.reset(0.0);
        }
    }
}

/// Shared engine: ordered bands + wet (on * mix) smoother.
#[derive(Clone, Copy, Debug)]
struct Core<const N: usize> {
    bands: [Band; N],
    count: usize,
    fs: f64,
    wet: OnePoleSmoother,
    running: bool,
}

impl<const N: usize> Core<N> {
    fn new(roles: &[Role]) -> Self {
        let mut bands = [Band::new(Role::Peak); N];
        for (b, &r) in bands.iter_mut().zip(roles) {
            *b = Band::new(r);
        }
        Self {
            bands,
            count: roles.len(),
            fs: 48_000.0,
            wet: OnePoleSmoother::new(0.0, WET_TAU_S, 48_000.0),
            running: false,
        }
    }

    fn prepare(&mut self, fs: f64) {
        self.fs = fs;
        self.wet.set_time(WET_TAU_S, fs);
        for b in &mut self.bands {
            b.set_fs(fs);
        }
        self.reset();
    }

    fn reset(&mut self) {
        self.wet.reset(self.wet.target());
        self.running = self.wet.target() != 0.0;
        let fs = self.fs;
        for b in &mut self.bands[..self.count] {
            b.reset(fs);
        }
    }

    fn set_wet(&mut self, wet: f32) {
        self.wet.set_target(wet);
        if wet != 0.0 && !self.running {
            // Resume from bypass with fresh state at the target parameters.
            self.running = true;
            let fs = self.fs;
            for b in &mut self.bands[..self.count] {
                b.reset(fs);
            }
        }
    }

    fn process(&mut self, left: &mut [f32], right: &mut [f32]) {
        debug_assert_eq!(left.len(), right.len());
        if !self.running {
            return;
        }
        let fs = self.fs;
        let mut buf: Buf = [[0.0; SUB_BLOCK]; 2];
        for (l, r) in left.chunks_mut(SUB_BLOCK).zip(right.chunks_mut(SUB_BLOCK)) {
            let n = l.len();
            for i in 0..n {
                buf[0][i] = l[i] as f64;
                buf[1][i] = r[i] as f64;
            }
            for b in &mut self.bands[..self.count] {
                b.process(&mut buf, n, fs);
            }
            let full = self.wet.target() == 1.0 && self.wet.is_settled(1e-7);
            if full {
                for i in 0..n {
                    l[i] = buf[0][i] as f32;
                    r[i] = buf[1][i] as f32;
                }
            } else {
                for i in 0..n {
                    let m = self.wet.tick() as f64;
                    let (dl, dr) = (l[i] as f64, r[i] as f64);
                    l[i] = (dl + m * (buf[0][i] - dl)) as f32;
                    r[i] = (dr + m * (buf[1][i] - dr)) as f32;
                }
            }
        }
        for b in &mut self.bands[..self.count] {
            b.end_block();
        }
        if self.wet.target() == 0.0 && self.wet.is_settled(1e-6) {
            self.wet.reset(0.0);
            self.running = false;
        }
    }
}

/// Standard WING EQ (`STD`) for one stereo strip.
#[derive(Clone, Debug)]
pub struct StdEq {
    variant: EqVariant,
    params: EqParams,
    core: Core<9>,
}

impl StdEq {
    pub fn new(variant: EqVariant) -> Self {
        let n = variant.peak_bands();
        let mut roles = [Role::Peak; 9];
        roles[0] = Role::Low;
        roles[n + 1] = Role::High;
        let count = if variant.has_tilt() {
            roles[n + 2] = Role::Tilt;
            n + 3
        } else {
            n + 2
        };
        let mut s = Self {
            variant,
            params: EqParams::defaults(variant),
            core: Core::new(&roles[..count]),
        };
        let p = s.params;
        s.set_params(&p);
        s.reset();
        s
    }

    pub fn variant(&self) -> EqVariant {
        self.variant
    }

    /// Set the sample rate and reset (settles all smoothers). `max_block` is unused (no allocation).
    pub fn prepare(&mut self, sample_rate: f64, _max_block: usize) {
        self.core.prepare(sample_rate);
        let p = self.params;
        self.set_params(&p);
        self.core.reset();
    }

    /// Clear filter state and jump smoothers to their targets.
    pub fn reset(&mut self) {
        self.core.reset();
    }

    pub fn params(&self) -> &EqParams {
        &self.params
    }

    /// Update parameters (RT-safe, clamped to the variant).
    pub fn set_params(&mut self, p: &EqParams) {
        let p = p.clamped(self.variant);
        self.params = p;
        let fs = self.core.fs;
        let n = self.variant.peak_bands();
        let bands = &mut self.core.bands;
        bands[0].set(p.low_type, p.low, fs);
        for i in 0..n {
            bands[i + 1].set(BandType::Peq, p.bands[i], fs);
        }
        bands[n + 1].set(p.high_type, p.high, fs);
        if self.variant.has_tilt() {
            bands[n + 2].set(
                BandType::Peq,
                BandParams::new(p.tilt_db, TILT_PIVOT_HZ as f32, 1.0),
                fs,
            );
        }
        self.core.set_wet(if p.on { p.mix / 100.0 } else { 0.0 });
    }

    /// Process one stereo block in place.
    pub fn process(&mut self, left: &mut [f32], right: &mut [f32]) {
        self.core.process(left, right);
    }

    /// Analytic complex response at `f` Hz for the target parameters (incl. on/mix).
    pub fn response(&self, f: f64) -> Complex {
        let p = &self.params;
        let one = Complex::new(1.0, 0.0);
        if !p.on {
            return one;
        }
        let fs = self.core.fs;
        let n = self.variant.peak_bands();
        let mut h = band_response(Role::Low, p.low_type, fs, p.low, f)
            * band_response(Role::High, p.high_type, fs, p.high, f);
        for b in &p.bands[..n] {
            h = h * band_response(Role::Peak, BandType::Peq, fs, *b, f);
        }
        if self.variant.has_tilt() {
            h = h * tilt(fs, TILT_PIVOT_HZ, p.tilt_db as f64).response(fs, f);
        }
        let m = p.mix as f64 / 100.0;
        one + (h - one) * m
    }

    /// Analytic magnitude (dB) at `f` Hz for the target parameters.
    pub fn magnitude_db(&self, f: f64) -> f64 {
        self.response(f).db()
    }
}

/// Channel PRE EQ (`peq`): three peaking bands with its own on switch.
#[derive(Clone, Debug)]
pub struct PreEq {
    params: PreEqParams,
    core: Core<3>,
}

impl Default for PreEq {
    fn default() -> Self {
        Self::new()
    }
}

impl PreEq {
    pub fn new() -> Self {
        let mut s = Self {
            params: PreEqParams::default(),
            core: Core::new(&[Role::Peak; 3]),
        };
        let p = s.params;
        s.set_params(&p);
        s.reset();
        s
    }

    pub fn prepare(&mut self, sample_rate: f64, _max_block: usize) {
        self.core.prepare(sample_rate);
        let p = self.params;
        self.set_params(&p);
        self.core.reset();
    }

    pub fn reset(&mut self) {
        self.core.reset();
    }

    pub fn params(&self) -> &PreEqParams {
        &self.params
    }

    pub fn set_params(&mut self, p: &PreEqParams) {
        let mut p = *p;
        for b in &mut p.bands {
            *b = b.clamped();
        }
        self.params = p;
        let fs = self.core.fs;
        for (band, bp) in self.core.bands.iter_mut().zip(p.bands) {
            band.set(BandType::Peq, bp, fs);
        }
        self.core.set_wet(if p.on { 1.0 } else { 0.0 });
    }

    pub fn process(&mut self, left: &mut [f32], right: &mut [f32]) {
        self.core.process(left, right);
    }

    pub fn response(&self, f: f64) -> Complex {
        let mut h = Complex::new(1.0, 0.0);
        if self.params.on {
            for b in &self.params.bands {
                h = h * band_response(Role::Peak, BandType::Peq, self.core.fs, *b, f);
            }
        }
        h
    }

    pub fn magnitude_db(&self, f: f64) -> f64 {
        self.response(f).db()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use wing_dsp::{dtft, BiquadF64};

    const FS: f64 = 48_000.0;

    fn freqs() -> impl Iterator<Item = f64> {
        (0..64).map(|i| 20.0 * 1000f64.powf(i as f64 / 63.0)) // 20 Hz .. 20 kHz
    }

    /// Analog prototype at the bilinear-warped normalised frequency.
    fn s_at(f: f64, f0: f64) -> Complex {
        Complex::new(0.0, (PI * f / FS).tan() / (PI * f0 / FS).tan())
    }

    fn poly(s: Complex, c: &[f64]) -> Complex {
        c.iter().fold(Complex::new(0.0, 0.0), |acc, &k| {
            acc * s + Complex::new(k, 0.0)
        })
    }

    fn on(variant: EqVariant) -> EqParams {
        EqParams {
            on: true,
            ..EqParams::defaults(variant)
        }
    }

    /// Run a unit impulse through `proc` (stereo, both channels) and return the left response.
    fn impulse(mut proc: impl FnMut(&mut [f32], &mut [f32]), n: usize) -> Vec<f64> {
        let mut l = vec![0.0f32; n];
        l[0] = 1.0;
        let mut r = l.clone();
        for (cl, cr) in l.chunks_mut(64).zip(r.chunks_mut(64)) {
            proc(cl, cr);
        }
        assert_eq!(
            l, r,
            "stereo channels must be identical for identical input"
        );
        l.into_iter().map(|x| x as f64).collect()
    }

    fn assert_processed_matches(eq: &mut StdEq, tol_db: f64, what: &str) {
        let want: Vec<f64> = freqs().map(|f| eq.magnitude_db(f)).collect();
        let h = impulse(|l, r| eq.process(l, r), 1 << 17);
        for (f, want) in freqs().zip(want) {
            if want < -90.0 {
                continue; // f32 I/O quantisation floor
            }
            let got = dtft(&h, 2.0 * PI * f / FS).db();
            assert!(
                (got - want).abs() < tol_db,
                "{what}: f={f:.1} got {got:.4} dB want {want:.4} dB"
            );
        }
    }

    #[test]
    fn band_designs_match_analog_prototypes() {
        let one = Complex::new(1.0, 0.0);
        for &f0 in &[20.0, 80.0, 1000.0, 9000.0] {
            for &q in &[0.442, 0.7, 1.41, 4.0, 10.0] {
                for &g in &[-15.0, -6.0, 3.0, 15.0] {
                    let a: f64 = 10f64.powf(g / 40.0);
                    let sa = a.sqrt() / q;
                    for f in freqs() {
                        let s = s_at(f, f0);
                        let cases = [
                            (
                                design(Role::Peak, BandType::Peq, FS, f0, q, g),
                                poly(s, &[1.0, a / q, 1.0]) / poly(s, &[1.0, 1.0 / (a * q), 1.0]),
                            ),
                            (
                                design(Role::Low, BandType::Shv, FS, f0, q, g),
                                poly(s, &[a, sa * a, a * a]) / poly(s, &[a, sa, 1.0]),
                            ),
                            (
                                design(Role::High, BandType::Shv, FS, f0, q, g),
                                poly(s, &[a * a, sa * a, a]) / poly(s, &[1.0, sa, a]),
                            ),
                            (
                                design(Role::Low, BandType::Cut, FS, f0, q, g),
                                s * s / poly(s, &[1.0, 1.0 / q, 1.0]),
                            ),
                            (
                                design(Role::High, BandType::Cut, FS, f0, q, g),
                                one / poly(s, &[1.0, 1.0 / q, 1.0]),
                            ),
                        ];
                        for (i, (d, ha)) in cases.iter().enumerate() {
                            let err = (d.response(FS, f).db() - ha.db()).abs();
                            assert!(err < 0.01, "case {i} f0={f0} q={q} g={g} f={f}: {err}");
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn cut_slopes_match_closed_forms() {
        // Bessel analog prototypes normalised to -3.01 dB at w = 1.
        let w2 = 1.361_654_128_716_130_8;
        let w4 = 2.113_917_674_904_215_4;
        for &fc in &[30.0, 500.0, 8000.0] {
            for (role, lp) in [(Role::Low, false), (Role::High, true)] {
                for f in freqs() {
                    let w = (PI * f / FS).tan() / (PI * fc / FS).tan();
                    let wn = if lp { w } else { 1.0 / w };
                    let bw = |n: i32| -10.0 * (1.0 + wn.powi(2 * n)).log10();
                    let s = Complex::new(0.0, wn);
                    let bs2 = (Complex::new(3.0, 0.0) / poly(s * w2, &[1.0, 3.0, 3.0])).db();
                    let bs4 = (Complex::new(105.0, 0.0)
                        / poly(s * w4, &[1.0, 10.0, 45.0, 105.0, 105.0]))
                    .db();
                    let want = [
                        (BandType::Bw6, bw(1)),
                        (BandType::Bw12, bw(2)),
                        (BandType::Bw18, bw(3)),
                        (BandType::Bw24, bw(4)),
                        (BandType::Bw48, bw(8)),
                        (BandType::Lr12, 2.0 * bw(1)),
                        (BandType::Lr24, 2.0 * bw(2)),
                        (BandType::Lr48, 2.0 * bw(4)),
                        (BandType::Bs12, bs2),
                        (BandType::Bs24, bs4),
                    ];
                    for (ty, want) in want {
                        if want < -150.0 {
                            continue;
                        }
                        let got = design(role, ty, FS, fc, 1.0, 9.0).response(FS, f).db();
                        assert!(
                            (got - want).abs() < 0.01,
                            "{ty:?} lp={lp} fc={fc} f={f}: {got} vs {want}"
                        );
                    }
                }
                // Landmarks at the band frequency.
                let at = |ty| design(role, ty, FS, fc, 1.0, 0.0).response(FS, fc).db();
                for ty in [
                    BandType::Bw6,
                    BandType::Bw12,
                    BandType::Bw24,
                    BandType::Bw48,
                ] {
                    assert!((at(ty) + 3.0103).abs() < 1e-3, "{ty:?}");
                }
                for ty in [BandType::Bs12, BandType::Bs24] {
                    assert!((at(ty) + 3.0103).abs() < 1e-3, "{ty:?} {}", at(ty));
                }
                for ty in [BandType::Lr12, BandType::Lr24, BandType::Lr48] {
                    assert!((at(ty) + 6.0206).abs() < 1e-3, "{ty:?}");
                }
            }
        }
    }

    #[test]
    fn landmarks_and_q_convention() {
        // Peak: exact gain at f0; shelves: half gain at f0; CUT: |H(f0)| = Q.
        let d = |r, t, q, g| design(r, t, FS, 1000.0, q, g).response(FS, 1000.0).db();
        assert!((d(Role::Peak, BandType::Peq, 2.0, 12.0) - 12.0).abs() < 1e-9);
        assert!((d(Role::Low, BandType::Shv, 0.707, -10.0) + 5.0).abs() < 1e-9);
        assert!((d(Role::High, BandType::Shv, 1.0, 8.0) - 4.0).abs() < 1e-9);
        assert!((d(Role::Low, BandType::Cut, 4.0, 0.0) - 20.0 * 4f64.log10()).abs() < 1e-9);
        // Q 1.414 (RBJ bandwidth Q) = 1 octave between the half-gain (dB) points of a peak.
        let c = design(Role::Peak, BandType::Peq, FS, 1000.0, 2f64.sqrt(), 12.0);
        let edge = |mut lo: f64, mut hi: f64| {
            let below = c.response(FS, lo).db() < 6.0;
            for _ in 0..100 {
                let mid = (lo * hi).sqrt();
                if (c.response(FS, mid).db() < 6.0) == below {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            lo
        };
        let oct = (edge(1000.0, 20_000.0) / edge(20.0, 1000.0)).log2();
        assert!((oct - 1.0).abs() < 0.01, "bandwidth {oct} oct");
        // Tilt: 0 dB at the pivot, -T/2 at DC, +T/2 at Nyquist.
        let t = tilt(FS, TILT_PIVOT_HZ, 6.0);
        assert!(t.response(FS, TILT_PIVOT_HZ).db().abs() < 1e-9);
        assert!((t.response_w(0.0).db() + 3.0).abs() < 1e-9);
        assert!((t.response_w(PI).db() - 3.0).abs() < 1e-9);
    }

    #[test]
    fn processed_response_matches_analytic_all_variants() {
        let mut ch = on(EqVariant::Channel);
        ch.low = BandParams::new(15.0, 20.0, 0.7); // 20 Hz shelf: needs f64
        ch.bands = [
            BandParams::new(-12.0, 60.0, 8.0),
            BandParams::new(6.0, 450.0, 0.442),
            BandParams::new(-3.5, 2500.0, 3.0),
            BandParams::new(9.0, 7000.0, 1.41),
            BandParams::new(15.0, 1000.0, 1.0), // ignored on channels
            BandParams::new(15.0, 1000.0, 1.0),
        ];
        ch.high_type = BandType::Peq;
        ch.high = BandParams::new(-8.0, 16_000.0, 2.0);
        let mut aux = ch;
        aux.low_type = BandType::Cut;
        aux.low = BandParams::new(0.0, 40.0, 2.0);
        aux.high_type = BandType::Shv;
        let mut bus = ch;
        bus.low_type = BandType::Lr48;
        bus.low = BandParams::new(0.0, 35.0, 1.0);
        bus.high_type = BandType::Bs24;
        bus.high = BandParams::new(0.0, 9000.0, 1.0);
        bus.bands[4] = BandParams::new(4.0, 11_000.0, 5.0);
        bus.bands[5] = BandParams::new(-6.0, 150.0, 0.8);
        bus.tilt_db = -4.5;
        for (variant, p) in [
            (EqVariant::Channel, ch),
            (EqVariant::Aux, aux),
            (EqVariant::Bus, bus),
        ] {
            let mut eq = StdEq::new(variant);
            eq.set_params(&p);
            eq.prepare(FS, 64);
            assert_processed_matches(&mut eq, 0.1, &format!("{variant:?}"));
        }
        // Every L/H type on the bus strip, at 50 % and 125 % mix too.
        for ty in BandType::ALL {
            for mix in [100.0, 50.0, 125.0] {
                let mut p = on(EqVariant::Bus);
                p.mix = mix;
                p.low_type = ty;
                p.low = BandParams::new(6.0, 120.0, 2.0);
                p.high_type = ty;
                p.high = BandParams::new(-6.0, 6000.0, 0.7);
                let mut eq = StdEq::new(EqVariant::Bus);
                eq.set_params(&p);
                eq.prepare(FS, 64);
                assert_processed_matches(&mut eq, 0.1, &format!("{ty:?} mix {mix}"));
            }
        }
    }

    #[test]
    fn variant_band_counts_and_type_coercion() {
        assert_eq!(EqVariant::Channel.peak_bands(), 4);
        assert_eq!(EqVariant::Aux.peak_bands(), 4);
        assert_eq!(EqVariant::Bus.peak_bands(), 6);
        let mut p = on(EqVariant::Channel);
        p.bands[4].gain_db = 12.0;
        p.bands[5].gain_db = 12.0;
        p.tilt_db = 6.0;
        p.low_type = BandType::Bw24;
        p.high_type = BandType::Cut;
        let mut ch = StdEq::new(EqVariant::Channel);
        ch.set_params(&p);
        assert_eq!(ch.params().low_type, BandType::Shv);
        assert_eq!(ch.params().high_type, BandType::Shv);
        assert_eq!(ch.params().tilt_db, 0.0);
        for f in freqs() {
            assert!(
                ch.magnitude_db(f).abs() < 1e-9,
                "channel ignores bands 5/6 and tilt"
            );
        }
        let mut aux = StdEq::new(EqVariant::Aux);
        aux.set_params(&p);
        assert_eq!(aux.params().low_type, BandType::Shv);
        assert_eq!(aux.params().high_type, BandType::Cut);
        let mut bus = StdEq::new(EqVariant::Bus);
        bus.set_params(&p);
        assert_eq!(bus.params().low_type, BandType::Bw24);
        let mut t = on(EqVariant::Bus);
        t.tilt_db = 6.0;
        bus.set_params(&t);
        assert!(bus.magnitude_db(TILT_PIVOT_HZ).abs() < 1e-9);
        assert!((bus.magnitude_db(20.0) + 3.0).abs() < 0.05);
        assert!((bus.magnitude_db(20_000.0) - 3.0).abs() < 0.2);
        // Clamping.
        p.bands[0] = BandParams::new(40.0, 5.0, 100.0);
        p.mix = 300.0;
        ch.set_params(&p);
        assert_eq!(ch.params().bands[0], BandParams::new(15.0, 20.0, 10.0));
        assert_eq!(ch.params().mix, 125.0);
        // Snapshot keys.
        for t in BandType::ALL {
            assert_eq!(BandType::from_key(t.key()), Some(t));
        }
        assert_eq!(BandType::from_key("XX"), None);
    }

    #[test]
    fn off_is_exact_bypass_and_on_off_is_smooth() {
        let mut p = on(EqVariant::Channel);
        p.on = false;
        p.bands[1].gain_db = 12.0;
        let mut eq = StdEq::new(EqVariant::Channel);
        eq.set_params(&p);
        eq.prepare(FS, 64);
        let x: Vec<f32> = (0..4096).map(|i| (i as f32 * 0.05).sin() * 0.7).collect();
        let (mut l, mut r) = (x.clone(), x.clone());
        eq.process(&mut l, &mut r);
        assert_eq!(l, x);
        // Toggle on then off: no discontinuity, then exact bypass again.
        let mut y = Vec::new();
        for (k, chunk) in x.chunks(64).cycle().take(64 * 3).enumerate() {
            if k == 10 || k == 80 {
                p.on = k == 10;
                eq.set_params(&p);
            }
            let (mut l, mut r) = (chunk.to_vec(), chunk.to_vec());
            eq.process(&mut l, &mut r);
            y.extend(l);
        }
        let steady = max_second_diff(&y[64 * 40..64 * 80]);
        assert!(max_second_diff(&y) < steady * 1.05);
        let (mut l, mut r) = (x.clone(), x.clone());
        eq.process(&mut l, &mut r);
        assert_eq!(l, x);
        // A 0 dB band costs nothing and is exact.
        let mut q = on(EqVariant::Channel);
        q.mix = 100.0;
        eq.set_params(&q);
        eq.reset();
        let (mut l, mut r) = (x.clone(), x.clone());
        eq.process(&mut l, &mut r);
        assert_eq!(l, x);
    }

    fn max_second_diff(y: &[f32]) -> f32 {
        y.windows(3)
            .map(|w| (w[2] - 2.0 * w[1] + w[0]).abs())
            .fold(0.0, f32::max)
    }

    fn sine(f: f64, n: usize, amp: f32) -> Vec<f32> {
        (0..n)
            .map(|i| (2.0 * PI * f * i as f64 / FS).sin() as f32 * amp)
            .collect()
    }

    #[test]
    fn automation_is_zipper_free() {
        // 1 kHz sine, peak band jumps 0 -> +15 dB and 300 Hz -> 1 kHz in one control step.
        let x = sine(1000.0, 48_000, 0.25);
        let run = |smoothed: bool| -> Vec<f32> {
            let mut p = on(EqVariant::Channel);
            p.bands[1] = BandParams::new(-15.0, 300.0, 1.0);
            let mut eq = StdEq::new(EqVariant::Channel);
            eq.set_params(&p);
            eq.prepare(FS, 64);
            let mut abrupt = BiquadF64::new(Coeffs::peak(FS, 300.0, 1.0, -15.0));
            let mut y = Vec::new();
            for (k, c) in x.chunks(64).enumerate() {
                let (mut l, mut r) = (c.to_vec(), c.to_vec());
                if k == 100 {
                    p.bands[1] = BandParams::new(15.0, 1000.0, 1.0);
                    eq.set_params(&p);
                    abrupt.set_coeffs(Coeffs::peak(FS, 1000.0, 1.0, 15.0));
                }
                if smoothed {
                    eq.process(&mut l, &mut r);
                } else {
                    abrupt.process_block(&mut l);
                }
                y.extend(l);
            }
            y
        };
        // Zipper/click energy: everything above 8 kHz (8th-order Butterworth HP; the 1 kHz tone is < -140 dB).
        let hf = |y: &[f32]| -> f32 {
            let mut hp = wing_dsp::Cascade::butterworth(CutKind::Highpass, FS, 8000.0, 8);
            let mut z = y.to_vec();
            hp.process_block(&mut z);
            z[6000..].iter().fold(0.0f32, |m, v| m.max(v.abs())) // skip the HP's own start-up
        };
        let smooth = hf(&run(true));
        let abrupt = hf(&run(false));
        // Measured: smoothed ~2e-6 (-113 dBFS), abrupt ~1e-2 (-40 dBFS).
        assert!(smooth < 1e-4, "smoothed HF residue {smooth}");
        assert!(
            abrupt > 100.0 * smooth,
            "abrupt {abrupt} vs smoothed {smooth}"
        );
    }

    #[test]
    fn band_type_change_crossfades() {
        let x = sine(80.0, 48_000, 0.5);
        let mut p = on(EqVariant::Bus);
        p.low_type = BandType::Peq;
        p.low = BandParams::new(6.0, 100.0, 1.0);
        let mut eq = StdEq::new(EqVariant::Bus);
        eq.set_params(&p);
        eq.prepare(FS, 64);
        let mut y = Vec::new();
        for (k, c) in x.chunks(64).enumerate() {
            if k == 100 {
                p.low_type = BandType::Lr48;
                eq.set_params(&p);
            }
            if k == 200 {
                p.low_type = BandType::Shv;
                p.low.gain_db = 0.0;
                eq.set_params(&p);
            }
            let (mut l, mut r) = (c.to_vec(), c.to_vec());
            eq.process(&mut l, &mut r);
            y.extend(l);
        }
        // The 80 Hz sine at +6 dB peaks at ~1.0: max sample-to-sample slope of the boosted sine bounds any
        // crossfade (a hard switch to fresh-state LR48 would jump).
        let slope = |s: &[f32]| {
            s.windows(2)
                .map(|w| (w[1] - w[0]).abs())
                .fold(0.0, f32::max)
        };
        let steady = slope(&y[3000..6400]);
        assert!(slope(&y) <= steady * 1.05, "{} vs {steady}", slope(&y));
        // Finally SHV at 0 dB -> band deactivates -> bit-exact pass-through.
        let tail = &y[y.len() - 64..];
        assert_eq!(tail, &x[x.len() - 64..]);
    }

    #[test]
    fn stable_under_random_fast_automation() {
        let mut seed = 0x1234_5678u32;
        let mut rnd = move || {
            seed ^= seed << 13;
            seed ^= seed >> 17;
            seed ^= seed << 5;
            seed as f32 / u32::MAX as f32
        };
        for variant in [EqVariant::Channel, EqVariant::Aux, EqVariant::Bus] {
            let mut eq = StdEq::new(variant);
            let mut p = on(variant);
            eq.set_params(&p);
            eq.prepare(FS, 64);
            let mut peak = 0.0f32;
            for k in 0..3000 {
                let mut band = || {
                    BandParams::new(
                        rnd() * 30.0 - 15.0,
                        20.0 * 1000f32.powf(rnd()),
                        Q_MIN * (Q_MAX / Q_MIN).powf(rnd()),
                    )
                };
                p.low = band();
                p.high = band();
                for b in &mut p.bands {
                    *b = band();
                }
                if k % 7 == 0 {
                    p.low_type = BandType::ALL[(rnd() * 12.99) as usize];
                    p.high_type = BandType::ALL[(rnd() * 12.99) as usize];
                }
                p.tilt_db = rnd() * 12.0 - 6.0;
                p.mix = rnd() * 125.0;
                p.on = rnd() > 0.05;
                eq.set_params(&p);
                let n = 1 + (rnd() * 63.0) as usize; // odd block sizes too
                let mut l: Vec<f32> = (0..n).map(|_| rnd() * 2.0 - 1.0).collect();
                let mut r = l.clone();
                eq.process(&mut l, &mut r);
                for v in l.iter().chain(&r) {
                    assert!(v.is_finite());
                    peak = peak.max(v.abs());
                }
            }
            // +15 dB bands, resonant shelves/cuts at Q 10 and 125 % mix can stack, but must stay bounded.
            assert!(peak < 1e3, "{variant:?} peak {peak}");
            // Settles back to a clean response.
            let mut q = on(variant);
            q.bands[0] = BandParams::new(6.0, 1000.0, 1.0);
            eq.set_params(&q);
            let mut l = vec![0.0f32; 96_000];
            let mut r = l.clone();
            eq.process(&mut l, &mut r);
            let tail = l[90_000..].iter().fold(0.0f32, |m, v| m.max(v.abs()));
            assert!(tail < 1e-4, "{variant:?} tail {tail}");
            eq.reset();
            assert_processed_matches(&mut eq, 0.1, "after automation");
        }
    }

    #[test]
    fn pre_eq_matches_analytic_and_bypasses() {
        let mut pre = PreEq::new();
        let p = PreEqParams {
            on: true,
            bands: [
                BandParams::new(8.0, 40.0, 2.0),
                BandParams::new(-10.0, 900.0, 6.0),
                BandParams::new(4.0, 12_000.0, 0.5),
            ],
        };
        pre.set_params(&p);
        pre.prepare(FS, 64);
        let want: Vec<f64> = freqs().map(|f| pre.magnitude_db(f)).collect();
        let h = impulse(|l, r| pre.process(l, r), 1 << 16);
        for (f, want) in freqs().zip(want) {
            let got = dtft(&h, 2.0 * PI * f / FS).db();
            assert!((got - want).abs() < 0.1, "pre eq f={f}: {got} vs {want}");
        }
        let mut off = PreEq::new();
        off.prepare(FS, 64);
        let x = sine(300.0, 1024, 0.5);
        let (mut l, mut r) = (x.clone(), x.clone());
        off.process(&mut l, &mut r);
        assert_eq!(l, x);
    }
}
