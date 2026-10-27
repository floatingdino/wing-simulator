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
//!   Placement (signal-flow.md §2, [I]) is directly before the EQ model inside the E block; `mix` does not scale
//!   it (`Cequalizer::update` designs it without the mix factor).
//!
//! Ranges: gain -15..+15 dB, freq 20..20000 Hz, Q 0.442..10 (WING-Edit 3.3.4 / T003 schema); values are clamped.
//!
//! Response (T03C): every band uses WING-Edit's own coefficient routines ([`crate::wingcoef`], ported from
//! `ae::eq_parchg_xseries[_bus]`), so magnitudes match `data/reference-curves/eq-std-*.json` to float32 rounding:
//! - PEQ ([`wingcoef::peq`], `hqpeq`): Orfanidis peaking EQ, exact gain at f, analog gain at Nyquist (no
//!   cramping). **Q convention**: Q describes the un-boosted side; the bandwidth (octaves) is that of
//!   `Q / sqrt(G)` (`G` = linear |gain|), measured at the dB midpoint. Boost and cut are exact inverses.
//! - SHV low: `ls12(f', g, 0.707)` with `f' = min(trunc(f), 8000)`; SHV high: `hqhs12(max(trunc(f), 50), g)`.
//!   **Shelf Q is ignored** (fixed Butterworth 0.707, table-corrected above 2.5 kHz for the high shelf). `f` is
//!   the corner of the unity-gain side, so a +6 dB shelf reads about +3.9 dB at `f`.
//! - CUT: aux low = bilinear 12 dB/oct Butterworth HP, low on bus = two `hqhp12` (= LR24), high (aux/bus) =
//!   `hqlp12` 12 dB/oct (Massberg matched). **Q is ignored.**
//! - BW/LR/BS (bus): `Cxover48` shapes. Bessel types are *not* -3 dB-normalised: BS12 = Q 0.577 at f, BS24 =
//!   sections at f*1.0588 (Q 0.806) and f*0.9444 (Q 0.522).
//! - Bus section budget (WING-Edit layout, 10 biquads): a low cut needing n > 2 sections takes over bands 1..
//!   (n-2), a high cut needing n > 1 sections takes over bands 6, 5, 4 (n-1 of them); those bands go silent.
//! - Tilt (bus): `hs12(pivot, -2*tilt, Q 0.2) * 10^(tilt/20)`: **positive tilt raises the low end** by `tilt` dB
//!   and lowers the top by `tilt` dB. Pivot [`tilt_pivot_hz`] 650 Hz (0 dB) .. 780 Hz (3 dB) .. 925 Hz (6 dB).
//! - **`mix` law**: `mix/100` scales the gain (dB) of every PEQ/SHV band (L, 1..n, H; not tilt, not cuts),
//!   i.e. 50 % = half the boost/cut in dB, 125 % = exaggerated. It is not a dry/wet blend
//!   (`ae::Cequalizer::update` passes `mix/100` to `eq_parchg_*`, which multiplies each band gain).
//!
//! Real-time behaviour: f64 state; coefficients are the f32 WING designs. Gain (dB), log2(f) and log2(Q) follow
//! 20 ms one-pole smoothers; while gliding, coefficients are redesigned every [`SUB_BLOCK`] samples and
//! interpolated linearly per sample in between, so fast automation is zipper- and click-free. A band-type change
//! crossfades over 10 ms from the old filter to the new one (fresh state); `on` ramps over a 5 ms one-pole.
//! PEQ/SHV bands at 0 dB and a switched-off EQ cost nothing (WING's 0 dB PEQ is a -0.001 dB peak; treated as
//! identity). `prepare` is the only call that may allocate (it does not, currently).

use crate::wingcoef::{self, Bq, Ctx};
use wing_dsp::biquad::Sections;
use wing_dsp::{Coeffs, Complex, LinearRamp, OnePoleSmoother};

/// Coefficient redesign interval while a parameter glides (samples).
pub const SUB_BLOCK: usize = 16;
/// Tilt pivot at |tilt| = 3 dB (the pivot moves with the amount, see [`tilt_pivot_hz`]).
pub const TILT_PIVOT_HZ: f64 = 780.0;
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
/// Biquads in WING-Edit's bus EQ layout (L, L2, bands 1..6, H, tilt).
pub const BUS_SECTIONS: usize = 10;

/// Tilt shelf pivot (Hz) for a tilt amount (dB), as WING-Edit computes it.
pub fn tilt_pivot_hz(tilt_db: f32) -> f32 {
    wingcoef::tilt_pivot_hz(tilt_db)
}

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
    /// 12 dB/oct Butterworth cut (aux, bus high band) or LR24 (bus low band); Q is ignored.
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

    /// Only PEQ uses Q: WING's shelves (fixed 0.707 / table) and cuts ignore it (T03C, `eq_parchg_xseries`).
    pub fn uses_q(self) -> bool {
        matches!(self, BandType::Peq)
    }

    /// `Cxover48` type index (1 = BW6 .. 10 = LR48), 0 for PEQ/SHV/CUT.
    fn xover_index(self) -> u8 {
        match self {
            BandType::Peq | BandType::Shv | BandType::Cut => 0,
            t => BandType::ALL.iter().position(|&x| x == t).unwrap_or(2) as u8 - 2,
        }
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
    /// 0 .. 125 %: scales every PEQ/SHV band gain (see module doc).
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

/// Listen filter of an EQ band solo (`eq/$solo` + `$solobd`): what the solo bus plays instead of the strip
/// while a band is soloed [I: WING's band-solo filter shape is not documented; X32-style band listen].
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum BandSolo {
    /// Peaking band: band-pass (0 dB at `freq`) with the band's Q.
    BandPass { freq: f32, q: f32 },
    /// Low band as shelf or cut: the region below its corner (12 dB/oct Butterworth low-pass).
    LowPass { freq: f32 },
    /// High band as shelf or cut: the region above its corner (12 dB/oct Butterworth high-pass).
    HighPass { freq: f32 },
}

impl BandSolo {
    /// Biquad at sample rate `fs` (frequencies clamped below 0.45 fs).
    pub fn coeffs(self, fs: f64) -> Coeffs {
        let f = |f: f32| (f as f64).clamp(FREQ_MIN_HZ as f64, 0.45 * fs);
        match self {
            BandSolo::BandPass { freq, q } => {
                Coeffs::bandpass(fs, f(freq), (q as f64).clamp(Q_MIN as f64, Q_MAX as f64))
            }
            BandSolo::LowPass { freq } => {
                Coeffs::lowpass(fs, f(freq), std::f64::consts::FRAC_1_SQRT_2)
            }
            BandSolo::HighPass { freq } => {
                Coeffs::highpass(fs, f(freq), std::f64::consts::FRAC_1_SQRT_2)
            }
        }
    }
}

impl EqParams {
    /// Listen filter of solo band `band` (`$solobd`, 1-based: 1 = low band, 2.. = peaking bands, last = high
    /// band); `None` out of range.
    pub fn solo_band(&self, variant: EqVariant, band: usize) -> Option<BandSolo> {
        let n = variant.peak_bands();
        let peq = |b: &BandParams| BandSolo::BandPass {
            freq: b.freq,
            q: b.q,
        };
        Some(match band {
            1 if self.low_type == BandType::Peq => peq(&self.low),
            1 => BandSolo::LowPass {
                freq: self.low.freq,
            },
            k if (2..=n + 1).contains(&k) => peq(&self.bands[k - 2]),
            k if k == n + 2 && self.high_type == BandType::Peq => peq(&self.high),
            k if k == n + 2 => BandSolo::HighPass {
                freq: self.high.freq,
            },
            _ => return None,
        })
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

/// WING sections of one band: `f`, `q`, `g` exactly as `eq_parchg_xseries[_bus]` passes them (gain already
/// scaled by mix). `bus` selects the bus low CUT (two `hqhp12`) instead of the aux one (bilinear HP).
fn wing_band(
    c: &Ctx,
    bus: bool,
    role: Role,
    ty: BandType,
    f: f32,
    q: f32,
    g: f32,
) -> ([Bq; 4], usize) {
    let one = |b: Bq| {
        let mut s = [wingcoef::IDENTITY; 4];
        s[0] = b;
        (s, 1)
    };
    match role {
        Role::Peak => one(wingcoef::peq(c, f, g, q)),
        Role::Tilt => one(wingcoef::tilt(c, g)),
        Role::Low => match ty {
            BandType::Peq => one(wingcoef::peq(c, f, g, q)),
            BandType::Shv => {
                let f = if f > 8000.0 { 8000.0 } else { f.trunc() };
                one(wingcoef::ls12(c, f, g, core::f32::consts::FRAC_1_SQRT_2))
            }
            BandType::Cut if bus => {
                let mut s = [wingcoef::IDENTITY; 4];
                s[0] = wingcoef::hp12(c, f);
                s[1] = s[0];
                (s, 2)
            }
            BandType::Cut => one(wingcoef::hp12_bilinear(
                c,
                f,
                core::f32::consts::FRAC_1_SQRT_2,
            )),
            t => wingcoef::xover(c, t.xover_index(), false, f),
        },
        Role::High => match ty {
            BandType::Peq => one(wingcoef::peq(c, f, g, q)),
            BandType::Shv => {
                let f = if f < 50.0 { 50.0 } else { f.trunc() };
                one(wingcoef::hqhs12(c, f, g))
            }
            BandType::Cut => one(wingcoef::lp12(c, f, core::f32::consts::FRAC_1_SQRT_2)),
            t => wingcoef::xover(c, t.xover_index(), true, f),
        },
    }
}

/// Number of WING sections a low/high band type occupies (bus layout).
fn section_count(role: Role, ty: BandType, bus: bool) -> usize {
    match ty {
        BandType::Peq | BandType::Shv => 1,
        BandType::Cut => {
            if bus && role == Role::Low {
                2
            } else {
                1
            }
        }
        BandType::Bw6 | BandType::Bw12 | BandType::Bs12 => 1,
        BandType::Lr12 | BandType::Bw18 | BandType::Bw24 | BandType::Bs24 | BandType::Lr24 => 2,
        BandType::Bw48 | BandType::Lr48 => 4,
    }
}

fn to_coeffs(b: &Bq) -> Coeffs {
    Coeffs {
        b0: b[0] as f64,
        b1: b[1] as f64,
        b2: b[2] as f64,
        a1: b[3] as f64,
        a2: b[4] as f64,
    }
}

fn to_sections((s, n): ([Bq; 4], usize)) -> Sections {
    let mut coeffs = [Coeffs::IDENTITY; MAX_SECTIONS];
    for (c, b) in coeffs.iter_mut().zip(&s[..n]) {
        *c = to_coeffs(b);
    }
    Sections { coeffs, len: n }
}

/// Band design for the runtime (f64 smoothed values rounded to the f32 WING inputs).
fn design(c: &Ctx, bus: bool, role: Role, ty: BandType, f: f32, q: f32, g: f32) -> Sections {
    to_sections(wing_band(c, bus, role, ty, f, q, g))
}

/// One band of the effective strip: what is actually designed after mix scaling and bus section take-over.
#[derive(Clone, Copy, Debug)]
struct Eff {
    role: Role,
    ty: BandType,
    p: BandParams,
    /// Displaced by a multi-section cut (bus): forced to 0 dB.
    displaced: bool,
}

/// Effective band list (L, 1..n, H[, tilt]) for clamped params.
fn effective(variant: EqVariant, p: &EqParams) -> ([Eff; 9], usize) {
    let n = variant.peak_bands();
    let bus = variant == EqVariant::Bus;
    let scale = p.mix / 100.0;
    let scaled = |b: BandParams, ty: BandType| {
        if ty.uses_gain() {
            BandParams::new(b.gain_db * scale, b.freq, b.q)
        } else {
            b
        }
    };
    let blank = Eff {
        role: Role::Peak,
        ty: BandType::Peq,
        p: BandParams::new(0.0, 1000.0, 1.0),
        displaced: false,
    };
    let mut e = [blank; 9];
    e[0] = Eff {
        role: Role::Low,
        ty: p.low_type,
        p: scaled(p.low, p.low_type),
        displaced: false,
    };
    for i in 0..n {
        e[i + 1] = Eff {
            role: Role::Peak,
            ty: BandType::Peq,
            p: scaled(p.bands[i], BandType::Peq),
            displaced: false,
        };
    }
    e[n + 1] = Eff {
        role: Role::High,
        ty: p.high_type,
        p: scaled(p.high, p.high_type),
        displaced: false,
    };
    let mut count = n + 2;
    if bus {
        let nl = section_count(Role::Low, p.low_type, true);
        for b in e[1..].iter_mut().take(nl.saturating_sub(2)) {
            b.displaced = true;
        }
        let nh = section_count(Role::High, p.high_type, true);
        for b in e[n + 1 - (nh - 1)..=n].iter_mut() {
            b.displaced = true;
        }
        e[n + 2] = Eff {
            role: Role::Tilt,
            ty: BandType::Peq,
            p: BandParams::new(p.tilt_db, 1000.0, 1.0),
            displaced: false,
        };
        count += 1;
    }
    for b in &mut e[..count] {
        if b.displaced {
            b.p.gain_db = 0.0;
        }
    }
    (e, count)
}

/// Analytic complex response of one effective band (identity when it would be bypassed at runtime).
fn band_response(c: &Ctx, bus: bool, b: &Eff, f: f64) -> Complex {
    if b.ty.uses_gain() && b.p.gain_db == 0.0 {
        return Complex::new(1.0, 0.0);
    }
    let (s, n) = wing_band(c, bus, b.role, b.ty, b.p.freq, b.p.q, b.p.gain_db);
    wingcoef::response(c.fs, &s[..n], f)
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
    /// Bus layout (bus low CUT design).
    bus: bool,
    ty: BandType,
    /// Exact (unsmoothed) target parameters: designs use them once the smoothers settle.
    tgt: BandParams,
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
    fn new(role: Role, bus: bool) -> Self {
        Self {
            role,
            bus,
            tgt: BandParams::new(0.0, 1000.0, 1.0),
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
        let c = Ctx::new(fs);
        if self.gliding() {
            design(
                &c,
                self.bus,
                self.role,
                self.ty,
                2f64.powf(self.log2f.value() as f64) as f32,
                2f64.powf(self.log2q.value() as f64) as f32,
                self.gain.value(),
            )
        } else {
            let t = self.tgt;
            design(&c, self.bus, self.role, self.ty, t.freq, t.q, t.gain_db)
        }
    }

    fn start_fade(&mut self, from_identity: bool) {
        self.fade.reset(0.0);
        self.fade.set_target(1.0);
        self.fading = true;
        self.fade_from_identity = from_identity;
    }

    fn set(&mut self, ty: BandType, p: BandParams, fs: f64) {
        self.tgt = p;
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

/// Shared engine: ordered bands + wet (`on`) smoother.
#[derive(Clone, Copy, Debug)]
struct Core<const N: usize> {
    bands: [Band; N],
    count: usize,
    fs: f64,
    wet: OnePoleSmoother,
    running: bool,
}

impl<const N: usize> Core<N> {
    fn new(roles: &[Role], bus: bool) -> Self {
        let mut bands = [Band::new(Role::Peak, bus); N];
        for (b, &r) in bands.iter_mut().zip(roles) {
            *b = Band::new(r, bus);
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
            core: Core::new(&roles[..count], variant == EqVariant::Bus),
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
        let (eff, count) = effective(self.variant, &p);
        for (band, e) in self.core.bands.iter_mut().zip(&eff[..count]) {
            band.set(e.ty, e.p, fs);
        }
        self.core.set_wet(if p.on { 1.0 } else { 0.0 });
    }

    /// The biquads WING-Edit computes for the current parameters, in its memory layout (channel/aux: L, 1..4,
    /// H; bus/main/mtx: L, L2, 1..6, H, tilt with multi-section cuts overwriting bands), as
    /// `[b0, b1, b2, a1, a2]` f32. Unlike the runtime this includes WING's near-identity 0 dB sections (a 0 dB
    /// PEQ is designed at -0.001 dB). `on` is ignored.
    pub fn wing_sections(&self) -> ([Bq; BUS_SECTIONS], usize) {
        let p = &self.params;
        let c = Ctx::new(self.core.fs);
        let bus = self.variant == EqVariant::Bus;
        let n = self.variant.peak_bands();
        let scale = p.mix / 100.0;
        let mut out = [wingcoef::IDENTITY; BUS_SECTIONS];
        let gain = |b: &BandParams, ty: BandType| {
            if ty.uses_gain() {
                b.gain_db * scale
            } else {
                0.0
            }
        };
        let off = usize::from(bus);
        for (i, b) in p.bands[..n].iter().enumerate() {
            out[off + 1 + i] = wingcoef::peq(&c, b.freq, b.gain_db * scale, b.q);
        }
        let (ls, ln) = wing_band(
            &c,
            bus,
            Role::Low,
            p.low_type,
            p.low.freq,
            p.low.q,
            gain(&p.low, p.low_type),
        );
        out[..ln].copy_from_slice(&ls[..ln]);
        let hi = off + 1 + n;
        let (hs, hn) = wing_band(
            &c,
            bus,
            Role::High,
            p.high_type,
            p.high.freq,
            p.high.q,
            gain(&p.high, p.high_type),
        );
        for (k, s) in hs[..hn].iter().enumerate() {
            out[hi - k] = *s;
        }
        if bus {
            out[hi + 1] = wingcoef::tilt(&c, p.tilt_db);
            (out, BUS_SECTIONS)
        } else {
            (out, n + 2)
        }
    }

    /// Process one stereo block in place.
    pub fn process(&mut self, left: &mut [f32], right: &mut [f32]) {
        self.core.process(left, right);
    }

    /// Analytic complex response at `f` Hz for the target parameters (incl. on/mix), as processed.
    pub fn response(&self, f: f64) -> Complex {
        let p = &self.params;
        let mut h = Complex::new(1.0, 0.0);
        if !p.on {
            return h;
        }
        let c = Ctx::new(self.core.fs);
        let bus = self.variant == EqVariant::Bus;
        let (eff, count) = effective(self.variant, p);
        for e in &eff[..count] {
            h = h * band_response(&c, bus, e, f);
        }
        h
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
            core: Core::new(&[Role::Peak; 3], false),
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
            let c = Ctx::new(self.core.fs);
            for b in &self.params.bands {
                let e = Eff {
                    role: Role::Peak,
                    ty: BandType::Peq,
                    p: *b,
                    displaced: false,
                };
                h = h * band_response(&c, false, &e, f);
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
    use core::f64::consts::PI;
    use wing_dsp::biquad::CutKind;
    use wing_dsp::{dtft, BiquadF64};

    const FS: f64 = 48_000.0;

    fn freqs() -> impl Iterator<Item = f64> {
        (0..64).map(|i| 20.0 * 1000f64.powf(i as f64 / 63.0)) // 20 Hz .. 20 kHz
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
    fn wing_design_landmarks() {
        let c = Ctx::new(FS);
        let db = |s: &[Bq], f: f64| wingcoef::response(FS, s, f).db();
        // PEQ: exact gain at f0; boost and cut are exact inverses.
        for &(f0, q, g) in &[
            (100.0f32, 0.7f32, 12.0f32),
            (1000.0, 2.0, -9.0),
            (8000.0, 4.0, 15.0),
        ] {
            let b = wingcoef::peq(&c, f0, g, q);
            assert!(
                (db(&[b], f0 as f64) - g as f64).abs() < 0.01,
                "peq {f0} {g}"
            );
            let inv = wingcoef::peq(&c, f0, -g, q);
            for f in freqs() {
                // f32 coefficients: ~0.01 dB rounding residue near 20 Hz.
                assert!(db(&[b, inv], f).abs() < 0.02, "boost*cut flat at {f}");
            }
        }
        // Shelves: corner on the unity side (full gain far below, partial at f).
        let ls = wingcoef::ls12(&c, 100.0, 6.0, core::f32::consts::FRAC_1_SQRT_2);
        assert!((db(&[ls], 10.0) - 6.0).abs() < 0.1);
        let at_f = db(&[ls], 100.0);
        assert!(at_f > 1.0 && at_f < 4.5, "{at_f}");
        assert!(db(&[ls], 2000.0).abs() < 0.05);
        let hs = wingcoef::hqhs12(&c, 1000.0, -12.0);
        assert!((db(&[hs], 20_000.0) + 12.0).abs() < 0.3);
        assert!(db(&[hs], 50.0).abs() < 0.05);
        // Cuts: BW -3.01 dB at fc, LR -6.02 dB, bus CUT low = LR24.
        let at = |role, ty, fc: f32| {
            let (s, n) = wing_band(&c, true, role, ty, fc, 1.0, 0.0);
            db(&s[..n], fc as f64)
        };
        for fc in [40.0f32, 500.0, 2000.0] {
            for ty in [
                BandType::Bw6,
                BandType::Bw12,
                BandType::Bw24,
                BandType::Bw48,
            ] {
                assert!((at(Role::Low, ty, fc) + 3.01).abs() < 0.1, "{ty:?} {fc}");
                assert!((at(Role::High, ty, fc) + 3.01).abs() < 0.1, "{ty:?} {fc}");
            }
            for ty in [
                BandType::Lr12,
                BandType::Lr24,
                BandType::Lr48,
                BandType::Cut,
            ] {
                assert!((at(Role::Low, ty, fc) + 6.02).abs() < 0.1, "{ty:?} {fc}");
            }
        }
        // Bessel is not -3 dB-normalised: BS12 = Q 0.577 at f (-4.77 dB).
        assert!((at(Role::High, BandType::Bs12, 1000.0) + 4.77).abs() < 0.05);
        // Tilt: +T raises the lows by T and lowers the top by T; pivot moves 650..925 Hz.
        let t = wingcoef::tilt(&c, 6.0);
        assert!((db(&[t], 20.0) - 6.0).abs() < 0.2);
        assert!((db(&[t], 22_000.0) + 6.0).abs() < 0.2);
        assert!(db(&[t], 650.0).abs() < 0.05, "crosses 0 dB near the pivot");
        assert_eq!(tilt_pivot_hz(0.0), 650.0);
        assert_eq!(tilt_pivot_hz(-3.0), 780.0);
        assert_eq!(tilt_pivot_hz(6.0), 925.0);
    }

    #[test]
    fn mix_scales_band_gains_and_bus_cuts_take_over_bands() {
        let mut p = on(EqVariant::Bus);
        p.bands[0] = BandParams::new(12.0, 1000.0, 1.0);
        p.low_type = BandType::Lr12;
        p.low = BandParams::new(0.0, 30.0, 1.0);
        let mut eq = StdEq::new(EqVariant::Bus);
        p.mix = 50.0;
        eq.set_params(&p);
        assert!(
            (eq.magnitude_db(1000.0) - 6.0).abs() < 0.01,
            "50 % = half the dB"
        );
        p.mix = 125.0;
        eq.set_params(&p);
        assert!(
            (eq.magnitude_db(1000.0) - 15.0).abs() < 0.01,
            "125 % exaggerates"
        );
        // The low cut is not scaled by mix.
        let mut cut = p;
        cut.bands[0].gain_db = 0.0;
        cut.mix = 50.0;
        eq.set_params(&cut);
        assert!((eq.magnitude_db(30.0) + 6.02).abs() < 0.05);
        // LR48 low takes the sections of bands 1 and 2: band 1 goes silent.
        p.mix = 100.0;
        p.low_type = BandType::Lr48;
        eq.set_params(&p);
        assert!(eq.magnitude_db(1000.0).abs() < 0.05, "band 1 displaced");
        // BW24 high takes band 6.
        let mut q = on(EqVariant::Bus);
        q.bands[5] = BandParams::new(9.0, 3000.0, 1.0);
        q.high_type = BandType::Bw24;
        q.high = BandParams::new(0.0, 18_000.0, 1.0);
        eq.set_params(&q);
        assert!(eq.magnitude_db(3000.0).abs() < 0.05, "band 6 displaced");
        q.high_type = BandType::Bw12;
        eq.set_params(&q);
        assert!((eq.magnitude_db(3000.0) - 9.0).abs() < 0.05);
        // wing_sections mirrors the layout.
        let (s, n) = eq.wing_sections();
        assert_eq!(n, BUS_SECTIONS);
        assert!((wingcoef::response(FS, &s[7..8], 3000.0).db() - 9.0).abs() < 0.01);
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
        assert!((bus.magnitude_db(20.0) - 6.0).abs() < 0.2);
        assert!((bus.magnitude_db(20_000.0) + 6.0).abs() < 0.2);
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
