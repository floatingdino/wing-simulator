//! FX rack enhancers, bass/sub synthesis, imaging and tape (T05E): `ENHANCE` (ULTRA ENHANCER), `EXCITER`
//! (Aphex-style), `P-BASS` (PSYCHO BASS), `SUB-M` (SUB MONSTER), `V-IMG` (VELVET IMAGER) and `TAPE`
//! (TAPE MACHINE). Parameters, ranges and defaults: `docs/research/models.md` §6 and `data/schema/wing-tree.json`
//! (slot order = schema order, listed per model below).
//!
//! Like every FX rack model they output the **wet** signal (for the enhancers: the complete processed signal,
//! dry path included); the slot host blends it with the dry input by `fxmix`.
//!
//! Every constant marked *[I]* is an inference: the console's exact mapping is not published (see
//! `docs/VERIFY-ON-HARDWARE.md`).
//!
//! # Common building blocks
//!
//! * **4x oversampling + ADAA.** Every nonlinear stage that can create harmonics above the audio band
//!   (ENHANCE, EXCITER, TAPE) runs at 4x ([`Oversampler4x`], round-trip latency 45.5 samples) with first-order
//!   antiderivative anti-aliasing (ADAA1, `ln cosh` antiderivative of the `tanh` shaper). The dry path of those
//!   models is delayed by [`OS_LATENCY`] = 46 samples (the 0.5-sample remainder is the half-band stage at 4x, the
//!   same convention as the dynamics oversamplers), and [`Model::latency`] reports 46. P-BASS / SUB-M / V-IMG have
//!   no latency (no oversampling: their nonlinearities are band-limited to the bass range or absent).
//!   Measured worst non-harmonic line for a 0.8 sine (tests): ENHANCE (all bands 100 %) -122 dBc at 9.7 kHz,
//!   EXCITER defaults 100 % wet -87 dBc, TAPE 12 dB drive -89 dBc (9.7 kHz) / -121 dBc (5.9 kHz); all < -80 dBc.
//!   Known limit: driven into a hard clip (EXCITER HARM/PEAK/ZFILL 100 %, TAPE +25 dB) a 9.7 kHz carrier
//!   aliases at about -70 / -62 dBc (21st harmonic folded; 1st-order ADAA cannot do more at 4x). 13 kHz carriers
//!   at the EXCITER defaults reach about -62 dBc. 2x was measured at -52..-63 dBc for the same cases.
//! * Shaper: `f(u) = tanh(u + b) - tanh(b)` (bias `b` = asymmetry, i.e. even harmonics). Its linear part
//!   `f'(0) u` is split off ("residual" form) where unity small-signal gain matters, as in WARM (T049).
//!
//! # ENHANCE (`fx_pardef_uenhancer`, slots 1-13: `stlvl lmf mlvl st m bass mid high g solo bassf midq highf`)
//! Band-split harmonic enhancer. At 4x: bass band = LP2 at BASS FREQ, mid band = band-pass at 1 kHz with Q = MID Q,
//! high band = HP2 at HIGH FREQ. Each band drives a biased tanh residual (pure harmonics, high-passed above the
//! band's lower edge) and is added back with its linear band with weight `gain_b` (BASS/MID/HIGH GAIN, 0..100 %):
//! `enh = sum gain_b * (lin_b * band_b + 1.5 * harm_b)` with `lin` = 1.0 / 0.5 / 0.6 (bass / mid / high, [`ENH_LIN`]). Everything at 0 is bit-exact dry (delayed 39 samples).
//! Frequency maps *[I]*: BASS FREQ 1..50 -> 30..400 Hz, HIGH FREQ 1..50 -> 2..12 kHz (both log), MID Q 1..50 ->
//! 0.3..8 (log); defaults 15/15/25 give 63 Hz / 0.77 / 4.8 kHz. Stage after the enhancement *[I]*: LMF SPRD scales
//! the side component of the enhancement (`1 + lmf/100`), SOLO outputs the enhancement only, then M/S levels
//! (`1 + MONO LVL/100`, `1 + ST LVL/100`), MONO PAN pans the mid component (linear balance), ST PAN is an output
//! balance, GAIN +-12 dB.
//!
//! # EXCITER (`fx_pardef_exciter`, slots 1-7: `tune peak zero tmb hrm mix solo`)
//! Aphex Type C chain: HP2 at TUNE (Q 0.71 .. 2.5 with PEAK) -> drive `k = 1 + 5 HARM` -> biased tanh
//! (`b = 0.6 TIMBRE/50`, signed asymmetry = even-harmonic weight) + ZFILL (`0.3 ZFILL tanh(5 u)` term, a steep zero-crossing
//! filler adding a hard odd edge) -> `/sqrt(k)` -> HP2 (DC / lows) -> TIMBRE tilt (+-6 dB shelf at 6 kHz) ->
//! `MIX` -> added to the dry signal; SOLO (DRY OFF) drops the dry signal. Channels independent.
//!
//! # P-BASS (`fx_pardef_ren_bass`, slots 1-4: `int bass xf solo`)
//! LR4 split at X/O FREQ. Low band `b` -> AGC (`u = b / (env + 1e-3)`) -> harmonic generator
//! `0.8 |u| + 0.6 (u|u| - 1.3 u)` (ADAA1; rectifier = 2nd, 4th ..., `u|u|` = 3rd, 5th ...) scaled back by the envelope ->
//! HP2 at X/O, LP2 at 4 X/O -> INTENSITY. Output `HP4(x) + BASS GAIN * b + INTENSITY * harm`; SOLO drops the
//! high band *[I]*. The harmonics reproduce the missing fundamental psychoacoustically (MaxxBass principle).
//!
//! # SUB-M (`fx_pardef_submonster`, slots 1-7: `mix freq bd1..bd5`)
//! dbx 120A-style synthesis on the mono sum: five band-pass filters (Q 1.4) at `FREQ * 2^((i-2)/2)`; per band a
//! Schmitt-trigger flip-flop (threshold 0.25 x band envelope) toggles on every second positive crossing -> a
//! square at exactly half the band's input frequency, amplitude from the band envelope (soft gate below -50 dB),
//! LP2 at 0.6 f_band -> band level BAND i. Sum scaled by MIX and added to both channels (the dry signal is
//! untouched). Because every band follows the actual zero crossings, the *input frequency* selects which bands
//! carry level, so BAND 1..5 act as the frequency response of the synthesis *[I]*.
//!
//! # V-IMG (`fx_pardef_velvetimg`, slots 1-5: `wid st g mode deep`)
//! M/S: `M = (L+R)/2`, `S = (L-R)/2`. WIDTH scales S (`1 + wid`; positive widening acts only above 150 Hz unless
//! DEEP, negative narrowing acts full range). STEREO adds the recovered ambience `0.7 * st * A` to S, where `A` is
//! the decorrelated source (HP2 250 Hz, two allpass sections, 1 ms delay; DEEP: HP2 100 Hz, 3 ms): K-STEREO uses
//! the existing side signal, VELVET additionally feeds in the high-passed mid (pseudo-stereo from near-mono
//! sources) *[I]*. Output `L = g (M + S')`, `R = g (M - S')`: every effect lives in S', so **L + R = 2 g M exactly**
//! (mono compatible by construction, tested).
//!
//! # TAPE (`fx_pardef_tapemach`, slots 1-5: `drv spd low hi out`)
//! HIGH SHELV (record pre-emphasis +6 dB / playback de-emphasis -4 dB around the saturator, net +2 dB at HF) ->
//! drive `g = 10^(DRIVE/20)` -> 4x biased-tanh saturator (`b = 0.12`, ADAA1, unity linear part), output normalised
//! `1/(K sqrt(g))` with `K = 1.5` *[I]* -> LOW BUMP: head-bump peak at `63 Hz * SPEED/15`, +4 dB at 7.5 ips falling
//! to +2 dB at 30 ips (Q 1.2) -> HF loss LP2 at `14 kHz (SPEED/15)^0.6` -> OUT GAIN. Extras that WING does not
//! expose ([`TapeExtras`], default off): wow (0.6 Hz) / flutter (9 Hz) as modulated delay (scaled by 15/SPEED)
//! and tape hiss (HF-tilted noise, +3 dB per halving of the speed). Enabling wow/flutter adds 1 ms (centre
//! delay) of unreported delay.
//!
//! Real-time: `prepare` allocates; `process`, `set_param`, `commit`, `reset` do not.

use crate::model::{Model, ParamValue};
use wing_dsp::biquad::{BiquadF64, Coeffs};
use wing_dsp::delay::DelayLine;
use wing_dsp::oversample::Oversampler4x;
use wing_dsp::smooth::OnePoleSmoother;

/// Dry-path delay (and reported latency) of the 4x oversampler: its 45.5-sample round trip rounded up.
pub const OS_LATENCY: usize = 46;
const ADAA_EPS: f64 = 1e-6;
const SMOOTH_S: f64 = 0.01;
const Q_BW: f64 = core::f64::consts::FRAC_1_SQRT_2;

// ---------------------------------------------------------------------------------------------------------
// shared helpers

#[inline]
fn db_lin(db: f32) -> f32 {
    10f32.powf(db * 0.05)
}

/// `ln cosh x`, overflow-free.
#[inline]
fn ln_cosh(x: f64) -> f64 {
    let a = x.abs();
    a + (-2.0 * a).exp().ln_1p() - core::f64::consts::LN_2
}

/// Log map of `v in lo..hi` to `a..b`.
#[inline]
fn logmap(v: f32, lo: f32, hi: f32, a: f32, b: f32) -> f32 {
    let t = ((v - lo) / (hi - lo)).clamp(0.0, 1.0);
    a * (b / a).powf(t)
}

/// First-order antiderivative anti-aliasing state.
#[derive(Clone, Copy, Default)]
struct Adaa {
    px: f64,
    pf: f64,
}

impl Adaa {
    fn reset(&mut self) {
        *self = Self::default();
    }

    /// `y = (F(x) - F(x_prev)) / (x - x_prev)`, with `f` the shaper and `bf` its antiderivative.
    #[inline(always)]
    fn tick(&mut self, x: f64, f: impl Fn(f64) -> f64, bf: impl Fn(f64) -> f64) -> f64 {
        let big = bf(x);
        let dx = x - self.px;
        let y = if dx.abs() < ADAA_EPS {
            f(0.5 * (x + self.px))
        } else {
            (big - self.pf) / dx
        };
        self.px = x;
        self.pf = big;
        y
    }
}

/// Biased tanh shaper `f(u) = tanh(u + b) - tanh(b)` with its antiderivative and linear-part split.
#[derive(Clone, Copy)]
struct Sat {
    b: f64,
    tb: f64,
    c1: f64,
}

impl Sat {
    fn new(b: f64) -> Self {
        let tb = b.tanh();
        Self {
            b,
            tb,
            c1: 1.0 - tb * tb,
        }
    }
    #[inline(always)]
    fn f(&self, u: f64) -> f64 {
        (u + self.b).tanh() - self.tb
    }
    #[inline(always)]
    fn bf(&self, u: f64) -> f64 {
        ln_cosh(u + self.b) - self.tb * u
    }
    /// Nonlinear remainder `f(u) - f'(0) u`.
    #[inline(always)]
    fn r(&self, u: f64) -> f64 {
        self.f(u) - self.c1 * u
    }
    #[inline(always)]
    fn br(&self, u: f64) -> f64 {
        self.bf(u) - 0.5 * self.c1 * u * u
    }
}

/// Fixed delay of [`OS_LATENCY`] samples (dry path alignment).
#[derive(Clone)]
struct Lat {
    b: [f32; 64],
    p: usize,
}

impl Lat {
    fn new() -> Self {
        Self { b: [0.0; 64], p: 0 }
    }
    fn reset(&mut self) {
        self.b = [0.0; 64];
    }
    #[inline(always)]
    fn tick(&mut self, x: f32) -> f32 {
        self.p = (self.p + 1) & 63;
        self.b[self.p] = x;
        self.b[self.p.wrapping_sub(OS_LATENCY) & 63]
    }
}

fn smoother(v: f32, fs: f64) -> OnePoleSmoother {
    OnePoleSmoother::new(v, SMOOTH_S, fs)
}

/// Stage `value` into `v[slot-1]`; returns false for unknown slots.
fn stage(v: &mut [f32], slot: u16, value: ParamValue) -> bool {
    match (slot as usize).checked_sub(1).and_then(|i| v.get_mut(i)) {
        Some(x) => {
            *x = value.as_f32();
            true
        }
        None => false,
    }
}

fn staged(v: &[f32], slot: u16) -> Option<f32> {
    (slot as usize)
        .checked_sub(1)
        .and_then(|i| v.get(i))
        .copied()
}

fn flush(bs: &mut [BiquadF64]) {
    for b in bs {
        b.flush_denormals();
    }
}

// ---------------------------------------------------------------------------------------------------------
// ENHANCE

/// Schema slots of ENHANCE.
pub mod enhance_slot {
    pub const STLVL: u16 = 1;
    pub const LMF: u16 = 2;
    pub const MLVL: u16 = 3;
    pub const ST: u16 = 4;
    pub const M: u16 = 5;
    pub const BASS: u16 = 6;
    pub const MID: u16 = 7;
    pub const HIGH: u16 = 8;
    pub const G: u16 = 9;
    pub const SOLO: u16 = 10;
    pub const BASSF: u16 = 11;
    pub const MIDQ: u16 = 12;
    pub const HIGHF: u16 = 13;
}

const ENH_DEFAULTS: [f32; 13] = [
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 15.0, 15.0, 25.0,
];
const ENH_MID_HZ: f64 = 1000.0;
/// Linear band share added back per band (bass, mid, high).
const ENH_LIN: [f64; 3] = [1.0, 0.5, 0.6];

struct EnhCh {
    os: Oversampler4x,
    lp: BiquadF64,
    bp: BiquadF64,
    hp: BiquadF64,
    hb: BiquadF64,
    hm: BiquadF64,
    hh: BiquadF64,
    ad: [Adaa; 3],
    dry: Lat,
}

impl EnhCh {
    fn new(mb: usize) -> Self {
        Self {
            os: Oversampler4x::new(mb),
            lp: BiquadF64::default(),
            bp: BiquadF64::default(),
            hp: BiquadF64::default(),
            hb: BiquadF64::default(),
            hm: BiquadF64::default(),
            hh: BiquadF64::default(),
            ad: [Adaa::default(); 3],
            dry: Lat::new(),
        }
    }
    fn reset(&mut self) {
        self.os.reset();
        for b in [
            &mut self.lp,
            &mut self.bp,
            &mut self.hp,
            &mut self.hb,
            &mut self.hm,
            &mut self.hh,
        ] {
            b.reset();
        }
        self.ad.iter_mut().for_each(Adaa::reset);
        self.dry.reset();
    }
}

#[derive(Clone, Copy)]
struct EnhCoef {
    lp: Coeffs,
    bp: Coeffs,
    hp: Coeffs,
    hb: Coeffs,
    hm: Coeffs,
    hh: Coeffs,
    gains: [f64; 3],
    sat: [Sat; 3],
    drive: [f64; 3],
}

/// ULTRA ENHANCER (`ENHANCE`); see the module doc.
pub struct UltraEnhancer {
    v: [f32; 13],
    dirty: bool,
    fs: f64,
    mb: usize,
    ch: [EnhCh; 2],
    el: Vec<f32>,
    er: Vec<f32>,
    c: EnhCoef,
    /// m gain, s gain, pan L, pan R, balance L, balance R, out gain, lmf side scale
    sm: [OnePoleSmoother; 8],
}

impl Default for UltraEnhancer {
    fn default() -> Self {
        Self::new()
    }
}

impl UltraEnhancer {
    pub fn new() -> Self {
        let sat = [Sat::new(0.5), Sat::new(0.3), Sat::new(0.2)];
        let id = Coeffs::IDENTITY;
        Self {
            v: ENH_DEFAULTS,
            dirty: true,
            fs: 48_000.0,
            mb: 64,
            ch: [EnhCh::new(64), EnhCh::new(64)],
            el: vec![0.0; 64],
            er: vec![0.0; 64],
            c: EnhCoef {
                lp: id,
                bp: id,
                hp: id,
                hb: id,
                hm: id,
                hh: id,
                gains: [0.0; 3],
                sat,
                drive: [4.0, 3.0, 3.0],
            },
            sm: [smoother(1.0, 48_000.0); 8],
        }
    }

    /// Band centre / corner frequencies in Hz for the current BASS FREQ, MID Q and HIGH FREQ values.
    pub fn band_map(bassf: f32, midq: f32, highf: f32) -> (f32, f32, f32) {
        (
            logmap(bassf, 1.0, 50.0, 30.0, 400.0),
            logmap(midq, 1.0, 50.0, 0.3, 8.0),
            logmap(highf, 1.0, 50.0, 2000.0, 12_000.0),
        )
    }

    fn targets(&self) -> [f32; 8] {
        use enhance_slot as s;
        let g = |slot: u16| self.v[slot as usize - 1];
        let pan = (g(s::M) * 0.01).clamp(-1.0, 1.0);
        let bal = (g(s::ST) * 0.01).clamp(-1.0, 1.0);
        [
            1.0 + (g(s::MLVL) * 0.01).clamp(-1.0, 1.0),
            1.0 + (g(s::STLVL) * 0.01).clamp(-1.0, 1.0),
            1.0 - pan.max(0.0),
            1.0 - (-pan).max(0.0),
            1.0 - bal.max(0.0),
            1.0 - (-bal).max(0.0),
            db_lin(g(s::G).clamp(-12.0, 12.0)),
            1.0 + (g(s::LMF) * 0.01).clamp(-1.0, 1.0),
        ]
    }

    fn update(&mut self) {
        use enhance_slot as s;
        self.dirty = false;
        let fs2 = self.fs * 4.0;
        let (fb, q, fh) = Self::band_map(
            self.v[s::BASSF as usize - 1],
            self.v[s::MIDQ as usize - 1],
            self.v[s::HIGHF as usize - 1],
        );
        let (fb, q, fh) = (fb as f64, q as f64, (fh as f64).min(self.fs * 0.45));
        let pct = |slot: u16| (self.v[slot as usize - 1] * 0.01).clamp(0.0, 1.0) as f64;
        self.c.lp = Coeffs::lowpass(fs2, fb, Q_BW);
        self.c.bp = Coeffs::bandpass(fs2, ENH_MID_HZ, q);
        self.c.hp = Coeffs::highpass(fs2, fh, Q_BW);
        self.c.hb = Coeffs::highpass(fs2, fb * 1.6, Q_BW);
        self.c.hm = Coeffs::highpass(fs2, ENH_MID_HZ * 1.4, Q_BW);
        self.c.hh = Coeffs::highpass(fs2, fh, Q_BW);
        self.c.gains = [pct(s::BASS), pct(s::MID), pct(s::HIGH)];
        for ch in &mut self.ch {
            ch.lp.set_coeffs(self.c.lp);
            ch.bp.set_coeffs(self.c.bp);
            ch.hp.set_coeffs(self.c.hp);
            ch.hb.set_coeffs(self.c.hb);
            ch.hm.set_coeffs(self.c.hm);
            ch.hh.set_coeffs(self.c.hh);
        }
        let t = self.targets();
        for (sm, t) in self.sm.iter_mut().zip(t) {
            sm.set_target(t);
        }
    }

    fn chunk(&mut self, left: &mut [f32], right: &mut [f32]) {
        let n = left.len();
        let c = self.c;
        let solo = self.v[enhance_slot::SOLO as usize - 1] >= 0.5;
        for (k, (buf, enh)) in [(&*left, &mut self.el), (&*right, &mut self.er)]
            .into_iter()
            .enumerate()
        {
            let enh = &mut enh[..n];
            enh.copy_from_slice(buf);
            let EnhCh {
                os,
                lp,
                bp,
                hp,
                hb,
                hm,
                hh,
                ad,
                ..
            } = &mut self.ch[k];
            os.process(enh, |hi| {
                for s in hi.iter_mut() {
                    let x = *s as f64;
                    let (b, m, h) = (lp.tick(x), bp.tick(x), hp.tick(x));
                    let mut e = 0.0;
                    for (i, (band, hf)) in [(b, &mut *hb), (m, &mut *hm), (h, &mut *hh)]
                        .into_iter()
                        .enumerate()
                    {
                        let (sat, kd) = (c.sat[i], c.drive[i]);
                        let r = ad[i].tick(kd * band, |u| sat.r(u), |u| sat.br(u));
                        let harm = hf.tick(r / (kd * sat.c1));
                        e += c.gains[i] * (ENH_LIN[i] * band + 1.5 * harm);
                    }
                    *s = e as f32;
                }
            });
        }
        for i in 0..n {
            let dl = self.ch[0].dry.tick(left[i]);
            let dr = self.ch[1].dry.tick(right[i]);
            let t: [f32; 8] = std::array::from_fn(|k| self.sm[k].tick());
            let (mut a, mut b) = (self.el[i], self.er[i]);
            // LMF SPRD: side scale of the enhancement
            let (em, es) = (0.5 * (a + b), 0.5 * (a - b) * t[7]);
            a = em + es;
            b = em - es;
            if !solo {
                a += dl;
                b += dr;
            }
            let m = 0.5 * (a + b) * t[0];
            let s = 0.5 * (a - b) * t[1];
            left[i] = (m * t[2] + s) * t[4] * t[6];
            right[i] = (m * t[3] - s) * t[5] * t[6];
        }
        for ch in &mut self.ch {
            flush(&mut [ch.lp, ch.bp, ch.hp, ch.hb, ch.hm, ch.hh]);
        }
    }
}

macro_rules! chunked_process {
    ($self:ident, $left:ident, $right:ident) => {{
        if $self.dirty {
            $self.update();
        }
        let mb = $self.mb;
        let n = $left.len().min($right.len());
        let (mut l, mut r) = (&mut $left[..n], &mut $right[..n]);
        while !l.is_empty() {
            let k = l.len().min(mb);
            let (lh, lt) = l.split_at_mut(k);
            let (rh, rt) = r.split_at_mut(k);
            $self.chunk(lh, rh);
            l = lt;
            r = rt;
        }
    }};
}

impl Model for UltraEnhancer {
    fn id(&self) -> &'static str {
        "ENHANCE"
    }

    fn prepare(&mut self, sample_rate: f64, max_block: usize) {
        self.fs = sample_rate;
        self.mb = max_block.max(1);
        self.ch = [EnhCh::new(self.mb), EnhCh::new(self.mb)];
        self.el = vec![0.0; self.mb];
        self.er = vec![0.0; self.mb];
        self.sm = [smoother(1.0, sample_rate); 8];
        self.dirty = true;
        self.reset();
    }

    fn process(&mut self, left: &mut [f32], right: &mut [f32], _sc: Option<(&[f32], &[f32])>) {
        chunked_process!(self, left, right);
    }

    fn latency(&self) -> usize {
        OS_LATENCY
    }

    fn reset(&mut self) {
        self.update();
        self.ch.iter_mut().for_each(EnhCh::reset);
        let t = self.targets();
        for (sm, t) in self.sm.iter_mut().zip(t) {
            sm.reset(t);
        }
    }

    fn set_param(&mut self, slot: u16, value: ParamValue) {
        if stage(&mut self.v, slot, value) {
            self.dirty = true;
        }
    }

    fn param(&self, slot: u16) -> Option<f32> {
        staged(&self.v, slot)
    }

    fn load_defaults(&mut self) {
        self.v = ENH_DEFAULTS;
        self.dirty = true;
    }

    fn commit(&mut self) {
        if self.dirty {
            self.update();
        }
    }
}

// ---------------------------------------------------------------------------------------------------------
// EXCITER

/// Schema slots of EXCITER.
pub mod exciter_slot {
    pub const TUNE: u16 = 1;
    pub const PEAK: u16 = 2;
    pub const ZERO: u16 = 3;
    pub const TMB: u16 = 4;
    pub const HRM: u16 = 5;
    pub const MIX: u16 = 6;
    /// DRY OFF.
    pub const SOLO: u16 = 7;
}

const EXC_DEFAULTS: [f32; 7] = [3500.0, 28.0, 24.0, 14.0, 30.0, 30.0, 0.0];

struct ExcCh {
    os: Oversampler4x,
    hp1: BiquadF64,
    hp2: BiquadF64,
    tilt: BiquadF64,
    ad: Adaa,
    dry: Lat,
}

impl ExcCh {
    fn new(mb: usize) -> Self {
        Self {
            os: Oversampler4x::new(mb),
            hp1: BiquadF64::default(),
            hp2: BiquadF64::default(),
            tilt: BiquadF64::default(),
            ad: Adaa::default(),
            dry: Lat::new(),
        }
    }
    fn reset(&mut self) {
        self.os.reset();
        self.hp1.reset();
        self.hp2.reset();
        self.tilt.reset();
        self.ad.reset();
        self.dry.reset();
    }
}

/// EXCITER (`EXCITER`); see the module doc.
pub struct Exciter {
    v: [f32; 7],
    dirty: bool,
    fs: f64,
    mb: usize,
    ch: [ExcCh; 2],
    ex: [Vec<f32>; 2],
    sat: Sat,
    k: f64,
    zc: f64,
    norm: f64,
    /// mix, dry level
    sm: [OnePoleSmoother; 2],
}

impl Default for Exciter {
    fn default() -> Self {
        Self::new()
    }
}

impl Exciter {
    pub fn new() -> Self {
        Self {
            v: EXC_DEFAULTS,
            dirty: true,
            fs: 48_000.0,
            mb: 64,
            ch: [ExcCh::new(64), ExcCh::new(64)],
            ex: [vec![0.0; 64], vec![0.0; 64]],
            sat: Sat::new(0.0),
            k: 1.0,
            zc: 0.0,
            norm: 1.0,
            sm: [smoother(0.0, 48_000.0), smoother(1.0, 48_000.0)],
        }
    }

    fn targets(&self) -> [f32; 2] {
        use exciter_slot as s;
        [
            (self.v[s::MIX as usize - 1] * 0.01).clamp(0.0, 1.0),
            if self.v[s::SOLO as usize - 1] >= 0.5 {
                0.0
            } else {
                1.0
            },
        ]
    }

    fn update(&mut self) {
        use exciter_slot as s;
        self.dirty = false;
        let g = |slot: u16| self.v[slot as usize - 1];
        let fs2 = self.fs * 4.0;
        let tune = (g(s::TUNE) as f64)
            .clamp(1000.0, 10_000.0)
            .min(self.fs * 0.4);
        let q = Q_BW * (2.5 / Q_BW).powf((g(s::PEAK) * 0.01).clamp(0.0, 1.0) as f64);
        let tmb = (g(s::TMB) / 50.0).clamp(-1.0, 1.0);
        let h = (g(s::HRM) * 0.01).clamp(0.0, 1.0) as f64;
        self.k = 1.0 + 5.0 * h;
        self.norm = 1.0 / self.k.sqrt();
        self.sat = Sat::new(0.6 * tmb as f64);
        self.zc = 0.3 * (g(s::ZERO) * 0.01).clamp(0.0, 1.0) as f64;
        let c1 = Coeffs::highpass(fs2, tune, q);
        let c2 = Coeffs::highpass(fs2, tune * 0.7, Q_BW);
        let c3 = Coeffs::high_shelf(fs2, 6000.0, Q_BW, 6.0 * tmb as f64);
        for ch in &mut self.ch {
            ch.hp1.set_coeffs(c1);
            ch.hp2.set_coeffs(c2);
            ch.tilt.set_coeffs(c3);
        }
        let t = self.targets();
        for (sm, t) in self.sm.iter_mut().zip(t) {
            sm.set_target(t);
        }
    }

    fn chunk(&mut self, left: &mut [f32], right: &mut [f32]) {
        let n = left.len();
        let (sat, k, zc, norm) = (self.sat, self.k, self.zc, self.norm);
        for (c, buf) in [(0usize, &*left), (1, &*right)] {
            let ex = &mut self.ex[c][..n];
            ex.copy_from_slice(buf);
            let ExcCh {
                os,
                hp1,
                hp2,
                tilt,
                ad,
                ..
            } = &mut self.ch[c];
            os.process(ex, |hi| {
                for s in hi.iter_mut() {
                    let u = k * hp1.tick(*s as f64);
                    let y = ad.tick(
                        u,
                        |u| sat.f(u) + zc * (5.0 * u).tanh(),
                        |u| sat.bf(u) + zc * 0.2 * ln_cosh(5.0 * u),
                    );
                    *s = tilt.tick(hp2.tick(y * norm)) as f32;
                }
            });
        }
        for i in 0..n {
            let mix = self.sm[0].tick();
            let dry = self.sm[1].tick();
            let dl = self.ch[0].dry.tick(left[i]);
            let dr = self.ch[1].dry.tick(right[i]);
            left[i] = dl * dry + mix * self.ex[0][i];
            right[i] = dr * dry + mix * self.ex[1][i];
        }
        for ch in &mut self.ch {
            flush(&mut [ch.hp1, ch.hp2, ch.tilt]);
        }
    }
}

impl Model for Exciter {
    fn id(&self) -> &'static str {
        "EXCITER"
    }

    fn prepare(&mut self, sample_rate: f64, max_block: usize) {
        self.fs = sample_rate;
        self.mb = max_block.max(1);
        self.ch = [ExcCh::new(self.mb), ExcCh::new(self.mb)];
        self.ex = [vec![0.0; self.mb], vec![0.0; self.mb]];
        self.sm = [smoother(0.0, sample_rate), smoother(1.0, sample_rate)];
        self.dirty = true;
        self.reset();
    }

    fn process(&mut self, left: &mut [f32], right: &mut [f32], _sc: Option<(&[f32], &[f32])>) {
        chunked_process!(self, left, right);
    }

    fn latency(&self) -> usize {
        OS_LATENCY
    }

    fn reset(&mut self) {
        self.update();
        self.ch.iter_mut().for_each(ExcCh::reset);
        let t = self.targets();
        for (sm, t) in self.sm.iter_mut().zip(t) {
            sm.reset(t);
        }
    }

    fn set_param(&mut self, slot: u16, value: ParamValue) {
        if stage(&mut self.v, slot, value) {
            self.dirty = true;
        }
    }

    fn param(&self, slot: u16) -> Option<f32> {
        staged(&self.v, slot)
    }

    fn load_defaults(&mut self) {
        self.v = EXC_DEFAULTS;
        self.dirty = true;
    }

    fn commit(&mut self) {
        if self.dirty {
            self.update();
        }
    }
}

// ---------------------------------------------------------------------------------------------------------
// P-BASS

/// Schema slots of P-BASS.
pub mod pbass_slot {
    pub const INT: u16 = 1;
    pub const BASS: u16 = 2;
    pub const XF: u16 = 3;
    pub const SOLO: u16 = 4;
}

const PB_DEFAULTS: [f32; 4] = [-10.0, -3.0, 60.0, 0.0];
/// Fundamental share of `u|u|` for the AGC-normalised bass (u peaks at ~1.5): removed from the odd term so the
/// harmonic path carries (almost) no fundamental.
const PB_KAPPA: f64 = 1.3;

struct PbCh {
    lp: [BiquadF64; 2],
    hp: [BiquadF64; 2],
    hh: [BiquadF64; 2],
    hl: BiquadF64,
    env: f64,
    ad: Adaa,
}

impl PbCh {
    fn new() -> Self {
        Self {
            lp: Default::default(),
            hp: Default::default(),
            hh: Default::default(),
            hl: BiquadF64::default(),
            env: 0.0,
            ad: Adaa::default(),
        }
    }
    fn reset(&mut self) {
        for b in self.lp.iter_mut().chain(self.hp.iter_mut()) {
            b.reset();
        }
        self.hh.iter_mut().for_each(BiquadF64::reset);
        self.hl.reset();
        self.env = 0.0;
        self.ad.reset();
    }
}

/// PSYCHO BASS (`P-BASS`); see the module doc.
pub struct PsychoBass {
    v: [f32; 4],
    dirty: bool,
    fs: f64,
    ch: [PbCh; 2],
    att: f64,
    rel: f64,
    /// intensity, bass gain, high-band level
    sm: [OnePoleSmoother; 3],
}

impl Default for PsychoBass {
    fn default() -> Self {
        Self::new()
    }
}

impl PsychoBass {
    pub fn new() -> Self {
        Self {
            v: PB_DEFAULTS,
            dirty: true,
            fs: 48_000.0,
            ch: [PbCh::new(), PbCh::new()],
            att: 0.01,
            rel: 0.001,
            sm: [smoother(0.3, 48_000.0); 3],
        }
    }

    fn targets(&self) -> [f32; 3] {
        use pbass_slot as s;
        [
            db_lin(self.v[s::INT as usize - 1].clamp(-24.0, 6.0)),
            db_lin(self.v[s::BASS as usize - 1].clamp(-60.0, 0.0)),
            if self.v[s::SOLO as usize - 1] >= 0.5 {
                0.0
            } else {
                1.0
            },
        ]
    }

    fn update(&mut self) {
        self.dirty = false;
        let fs = self.fs;
        let xf = (self.v[pbass_slot::XF as usize - 1] as f64).clamp(32.0, 200.0);
        let lp = Coeffs::lowpass(fs, xf, Q_BW);
        let hp = Coeffs::highpass(fs, xf, Q_BW);
        let hh = Coeffs::highpass(fs, xf, Q_BW);
        let hl = Coeffs::lowpass(fs, (4.0 * xf).min(fs * 0.4), Q_BW);
        for ch in &mut self.ch {
            ch.lp.iter_mut().for_each(|b| b.set_coeffs(lp));
            ch.hp.iter_mut().for_each(|b| b.set_coeffs(hp));
            ch.hh.iter_mut().for_each(|b| b.set_coeffs(hh));
            ch.hl.set_coeffs(hl);
        }
        self.att = 1.0 - (-1.0 / (0.005 * fs)).exp();
        self.rel = 1.0 - (-1.0 / (0.1 * fs)).exp();
        let t = self.targets();
        for (sm, t) in self.sm.iter_mut().zip(t) {
            sm.set_target(t);
        }
    }
}

impl Model for PsychoBass {
    fn id(&self) -> &'static str {
        "P-BASS"
    }

    fn prepare(&mut self, sample_rate: f64, _max_block: usize) {
        self.fs = sample_rate;
        self.sm = [smoother(0.3, sample_rate); 3];
        self.dirty = true;
        self.reset();
    }

    fn process(&mut self, left: &mut [f32], right: &mut [f32], _sc: Option<(&[f32], &[f32])>) {
        if self.dirty {
            self.update();
        }
        let n = left.len().min(right.len());
        let (att, rel) = (self.att, self.rel);
        for i in 0..n {
            let (ig, bg, hi_g) = (self.sm[0].tick(), self.sm[1].tick(), self.sm[2].tick());
            for (ch, x) in self.ch.iter_mut().zip([&mut left[i], &mut right[i]]) {
                let xin = *x as f64;
                let b = ch.lp[0].tick(xin);
                let b = ch.lp[1].tick(b);
                let hi = ch.hp[0].tick(xin);
                let hi = ch.hp[1].tick(hi);
                let a = b.abs();
                ch.env += (a - ch.env) * if a > ch.env { att } else { rel };
                let u = b / (ch.env + 1e-3);
                let h = ch.ad.tick(
                    u,
                    |u| 0.8 * u.abs() + 0.6 * (u * u.abs() - PB_KAPPA * u),
                    |u| 0.4 * u * u.abs() + 0.2 * u.abs().powi(3) - 0.3 * PB_KAPPA * u * u,
                ) * ch.env;
                let h = ch.hh[0].tick(h);
                let h = ch.hh[1].tick(h);
                let h = ch.hl.tick(h);
                *x = (hi as f32) * hi_g + (b as f32) * bg + (h as f32) * ig;
            }
        }
        for ch in &mut self.ch {
            if ch.env < 1e-20 {
                ch.env = 0.0;
            }
            flush(&mut ch.lp);
            flush(&mut ch.hp);
            flush(&mut ch.hh);
            flush(&mut [ch.hl]);
        }
    }

    fn reset(&mut self) {
        self.update();
        self.ch.iter_mut().for_each(PbCh::reset);
        let t = self.targets();
        for (sm, t) in self.sm.iter_mut().zip(t) {
            sm.reset(t);
        }
    }

    fn set_param(&mut self, slot: u16, value: ParamValue) {
        if stage(&mut self.v, slot, value) {
            self.dirty = true;
        }
    }

    fn param(&self, slot: u16) -> Option<f32> {
        staged(&self.v, slot)
    }

    fn load_defaults(&mut self) {
        self.v = PB_DEFAULTS;
        self.dirty = true;
    }

    fn commit(&mut self) {
        if self.dirty {
            self.update();
        }
    }
}

// ---------------------------------------------------------------------------------------------------------
// SUB-M

/// Schema slots of SUB-M.
pub mod subm_slot {
    pub const MIX: u16 = 1;
    pub const FREQ: u16 = 2;
    pub const BD1: u16 = 3;
    pub const BD5: u16 = 7;
}

const SM_DEFAULTS: [f32; 7] = [50.0, 50.0, 60.0, 80.0, 100.0, 80.0, 60.0];
const SUB_BANDS: usize = 5;

#[derive(Clone, Copy)]
struct SubBand {
    bp: BiquadF64,
    lp: BiquadF64,
    env: f64,
    hi: bool,
    ff: f64,
}

impl SubBand {
    fn new() -> Self {
        Self {
            bp: BiquadF64::default(),
            lp: BiquadF64::default(),
            env: 0.0,
            hi: false,
            ff: 1.0,
        }
    }
}

/// SUB MONSTER (`SUB-M`); see the module doc.
pub struct SubMonster {
    v: [f32; 7],
    dirty: bool,
    fs: f64,
    bands: [SubBand; SUB_BANDS],
    att: f64,
    rel: f64,
    lvl: [f64; SUB_BANDS],
    mix: OnePoleSmoother,
}

impl Default for SubMonster {
    fn default() -> Self {
        Self::new()
    }
}

impl SubMonster {
    pub fn new() -> Self {
        Self {
            v: SM_DEFAULTS,
            dirty: true,
            fs: 48_000.0,
            bands: [SubBand::new(); SUB_BANDS],
            att: 0.01,
            rel: 0.001,
            lvl: [0.0; SUB_BANDS],
            mix: smoother(0.5, 48_000.0),
        }
    }

    /// Analysis band centre frequencies (Hz) for a FREQ value.
    pub fn band_centres(freq: f32) -> [f32; SUB_BANDS] {
        std::array::from_fn(|i| freq * 2f32.powf((i as f32 - 1.0) * 0.5))
    }

    fn update(&mut self) {
        self.dirty = false;
        let fs = self.fs;
        let freq = self.v[subm_slot::FREQ as usize - 1].clamp(45.0, 67.5);
        for (i, (b, f)) in self
            .bands
            .iter_mut()
            .zip(Self::band_centres(freq))
            .enumerate()
        {
            b.bp.set_coeffs(Coeffs::bandpass(fs, f as f64, 1.4));
            b.lp.set_coeffs(Coeffs::lowpass(fs, 0.6 * f as f64, Q_BW));
            self.lvl[i] = (self.v[subm_slot::BD1 as usize - 1 + i] * 0.01).clamp(0.0, 1.0) as f64;
        }
        self.att = 1.0 - (-1.0 / (0.003 * fs)).exp();
        self.rel = 1.0 - (-1.0 / (0.08 * fs)).exp();
        self.mix
            .set_target((self.v[subm_slot::MIX as usize - 1] * 0.01).clamp(0.0, 1.0));
    }
}

impl Model for SubMonster {
    fn id(&self) -> &'static str {
        "SUB-M"
    }

    fn prepare(&mut self, sample_rate: f64, _max_block: usize) {
        self.fs = sample_rate;
        self.mix = smoother(0.5, sample_rate);
        self.dirty = true;
        self.reset();
    }

    fn process(&mut self, left: &mut [f32], right: &mut [f32], _sc: Option<(&[f32], &[f32])>) {
        if self.dirty {
            self.update();
        }
        let n = left.len().min(right.len());
        let (att, rel) = (self.att, self.rel);
        for i in 0..n {
            let x = 0.5 * (left[i] as f64 + right[i] as f64);
            let mut sub = 0.0;
            for (b, lvl) in self.bands.iter_mut().zip(self.lvl) {
                let y = b.bp.tick(x);
                let a = y.abs();
                b.env += (a - b.env) * if a > b.env { att } else { rel };
                if b.env < 1e-20 {
                    b.env = 0.0;
                }
                let th = 0.25 * b.env + 1e-6;
                if !b.hi && y > th {
                    b.hi = true;
                    b.ff = -b.ff;
                } else if b.hi && y < -th {
                    b.hi = false;
                }
                let gate = b.env / (b.env + 0.003);
                sub += b.lp.tick(b.ff * b.env * gate * lvl * 0.8);
            }
            let s = (sub * self.mix.tick() as f64) as f32;
            left[i] += s;
            right[i] += s;
        }
        for b in &mut self.bands {
            flush(&mut [b.bp, b.lp]);
        }
    }

    fn reset(&mut self) {
        self.update();
        for b in &mut self.bands {
            b.bp.reset();
            b.lp.reset();
            b.env = 0.0;
            b.hi = false;
            b.ff = 1.0;
        }
        let t = self.mix.target();
        self.mix.reset(t);
    }

    fn set_param(&mut self, slot: u16, value: ParamValue) {
        if stage(&mut self.v, slot, value) {
            self.dirty = true;
        }
    }

    fn param(&self, slot: u16) -> Option<f32> {
        staged(&self.v, slot)
    }

    fn load_defaults(&mut self) {
        self.v = SM_DEFAULTS;
        self.dirty = true;
    }

    fn commit(&mut self) {
        if self.dirty {
            self.update();
        }
    }
}

// ---------------------------------------------------------------------------------------------------------
// V-IMG

/// Schema slots of V-IMG.
pub mod vimg_slot {
    pub const WID: u16 = 1;
    pub const ST: u16 = 2;
    pub const G: u16 = 3;
    pub const MODE: u16 = 4;
    pub const DEEP: u16 = 5;
}

const VI_DEFAULTS: [f32; 5] = [0.0, 75.0, 0.0, 0.0, 0.0];
/// Widening below this frequency is left alone unless DEEP (Hz).
const VI_LOW_HZ: f64 = 150.0;
/// Ambience injection gain at STEREO = 100 %.
const VI_AMB: f32 = 0.7;

/// VELVET IMAGER (`V-IMG`); see the module doc.
pub struct VelvetImager {
    v: [f32; 5],
    dirty: bool,
    fs: f64,
    hp_s: BiquadF64,
    hp_a: BiquadF64,
    hp_m: BiquadF64,
    ap: [BiquadF64; 2],
    dl: DelayLine,
    d: f32,
    d_target: f32,
    /// width, stereo amount, gain
    sm: [OnePoleSmoother; 3],
}

impl Default for VelvetImager {
    fn default() -> Self {
        Self::new()
    }
}

impl VelvetImager {
    pub fn new() -> Self {
        Self {
            v: VI_DEFAULTS,
            dirty: true,
            fs: 48_000.0,
            hp_s: BiquadF64::default(),
            hp_a: BiquadF64::default(),
            hp_m: BiquadF64::default(),
            ap: Default::default(),
            dl: DelayLine::new(8),
            d: 1.0,
            d_target: 1.0,
            sm: [smoother(0.0, 48_000.0); 3],
        }
    }

    fn targets(&self) -> [f32; 3] {
        use vimg_slot as s;
        let g = |slot: u16| self.v[slot as usize - 1];
        [
            g(s::WID).clamp(-1.0, 1.0),
            (g(s::ST) * 0.01).clamp(0.0, 1.0),
            db_lin(g(s::G).clamp(-6.0, 6.0)),
        ]
    }

    fn update(&mut self) {
        self.dirty = false;
        let fs = self.fs;
        let deep = self.v[vimg_slot::DEEP as usize - 1] >= 0.5;
        self.hp_s.set_coeffs(Coeffs::highpass(fs, VI_LOW_HZ, Q_BW));
        let hz = if deep { 100.0 } else { 250.0 };
        self.hp_a.set_coeffs(Coeffs::highpass(fs, hz, Q_BW));
        self.hp_m.set_coeffs(Coeffs::highpass(fs, 250.0, Q_BW));
        self.ap[0].set_coeffs(Coeffs::allpass1(fs, 700.0));
        self.ap[1].set_coeffs(Coeffs::allpass1(fs, 2700.0));
        let ms = if deep { 3.0 } else { 1.0 };
        self.d_target = ((ms * 1e-3 * fs) as f32).clamp(1.0, self.dl.max_delay() as f32);
        let t = self.targets();
        for (sm, t) in self.sm.iter_mut().zip(t) {
            sm.set_target(t);
        }
    }
}

impl Model for VelvetImager {
    fn id(&self) -> &'static str {
        "V-IMG"
    }

    fn prepare(&mut self, sample_rate: f64, _max_block: usize) {
        self.fs = sample_rate;
        self.dl = DelayLine::new((0.004 * sample_rate) as usize + 4);
        self.sm = [smoother(0.0, sample_rate); 3];
        self.dirty = true;
        self.reset();
    }

    fn process(&mut self, left: &mut [f32], right: &mut [f32], _sc: Option<(&[f32], &[f32])>) {
        if self.dirty {
            self.update();
        }
        let n = left.len().min(right.len());
        let velvet = self.v[vimg_slot::MODE as usize - 1] >= 0.5;
        let deep = self.v[vimg_slot::DEEP as usize - 1] >= 0.5;
        for i in 0..n {
            let (wid, st, g) = (self.sm[0].tick(), self.sm[1].tick(), self.sm[2].tick());
            self.d += 0.002 * (self.d_target - self.d);
            let (l, r) = (left[i], right[i]);
            let m = 0.5 * (l + r);
            let s = 0.5 * (l - r);
            let sh = self.hp_s.tick(s as f64) as f32;
            let hm = self.hp_m.tick(m as f64) as f32;
            // narrowing and DEEP act full range, plain widening only above VI_LOW_HZ
            let s_w = if wid < 0.0 || deep {
                s * (1.0 + wid)
            } else {
                s + wid * sh
            };
            let src = if velvet { 0.5 * s + 0.5 * hm } else { s };
            let a = self.hp_a.tick(src as f64);
            let a = self.ap[0].tick(a);
            let a = self.ap[1].tick(a) as f32;
            self.dl.push(a);
            let a = self.dl.read_linear(self.d);
            let s_out = s_w + VI_AMB * st * a;
            left[i] = g * (m + s_out);
            right[i] = g * (m - s_out);
        }
        flush(&mut [self.hp_s, self.hp_a, self.hp_m, self.ap[0], self.ap[1]]);
    }

    fn reset(&mut self) {
        self.update();
        for b in [&mut self.hp_s, &mut self.hp_a, &mut self.hp_m] {
            b.reset();
        }
        self.ap.iter_mut().for_each(BiquadF64::reset);
        self.dl.reset();
        self.d = self.d_target;
        let t = self.targets();
        for (sm, t) in self.sm.iter_mut().zip(t) {
            sm.reset(t);
        }
    }

    fn set_param(&mut self, slot: u16, value: ParamValue) {
        if stage(&mut self.v, slot, value) {
            self.dirty = true;
        }
    }

    fn param(&self, slot: u16) -> Option<f32> {
        staged(&self.v, slot)
    }

    fn load_defaults(&mut self) {
        self.v = VI_DEFAULTS;
        self.dirty = true;
    }

    fn commit(&mut self) {
        if self.dirty {
            self.update();
        }
    }
}

// ---------------------------------------------------------------------------------------------------------
// TAPE

/// Schema slots of TAPE.
pub mod tape_slot {
    pub const DRV: u16 = 1;
    pub const SPD: u16 = 2;
    pub const LOW: u16 = 3;
    pub const HI: u16 = 4;
    pub const OUT: u16 = 5;
}

const TP_DEFAULTS: [f32; 5] = [0.0, 15.0, 0.0, 0.0, 0.0];
/// Saturation knee scaling (see module doc).
const TAPE_K: f64 = 1.5;
const TAPE_BIAS: f64 = 0.12;
/// Centre delay of the wow/flutter modulated line (seconds).
const TAPE_WOW_CENTRE_S: f64 = 0.001;

/// Tape imperfections that WING does not expose (default: all off).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TapeExtras {
    /// Wow depth 0..1 (0.6 Hz, up to 0.35 ms of delay modulation at 15 ips).
    pub wow: f32,
    /// Flutter depth 0..1 (9 Hz, up to 0.02 ms).
    pub flutter: f32,
    /// Hiss level in dBFS at 15 ips (`<= -150` = off).
    pub hiss_db: f32,
}

impl Default for TapeExtras {
    fn default() -> Self {
        Self {
            wow: 0.0,
            flutter: 0.0,
            hiss_db: -200.0,
        }
    }
}

struct TapeCh {
    os: Oversampler4x,
    pre: BiquadF64,
    post: BiquadF64,
    bump: BiquadF64,
    lp: BiquadF64,
    ad: Adaa,
    dl: DelayLine,
    hiss_lp: f32,
}

impl TapeCh {
    fn new(mb: usize, dl: usize) -> Self {
        Self {
            os: Oversampler4x::new(mb),
            pre: BiquadF64::default(),
            post: BiquadF64::default(),
            bump: BiquadF64::default(),
            lp: BiquadF64::default(),
            ad: Adaa::default(),
            dl: DelayLine::new(dl),
            hiss_lp: 0.0,
        }
    }
    fn reset(&mut self) {
        self.os.reset();
        for b in [&mut self.pre, &mut self.post, &mut self.bump, &mut self.lp] {
            b.reset();
        }
        self.ad.reset();
        self.dl.reset();
        self.hiss_lp = 0.0;
    }
}

/// TAPE MACHINE (`TAPE`); see the module doc.
pub struct TapeMachine {
    v: [f32; 5],
    extras: TapeExtras,
    dirty: bool,
    fs: f64,
    mb: usize,
    ch: [TapeCh; 2],
    sat: Sat,
    kg: f64,
    inv: f64,
    wow_ph: f64,
    flut_ph: f64,
    rng: u64,
    /// output gain
    sm: OnePoleSmoother,
}

impl Default for TapeMachine {
    fn default() -> Self {
        Self::new()
    }
}

impl TapeMachine {
    pub fn new() -> Self {
        Self {
            v: TP_DEFAULTS,
            extras: TapeExtras::default(),
            dirty: true,
            fs: 48_000.0,
            mb: 64,
            ch: [TapeCh::new(64, 4), TapeCh::new(64, 4)],
            sat: Sat::new(TAPE_BIAS),
            kg: TAPE_K,
            inv: 1.0 / TAPE_K,
            wow_ph: 0.0,
            flut_ph: 0.0,
            rng: 0x9E37_79B9_7F4A_7C15,
            sm: smoother(1.0, 48_000.0),
        }
    }

    /// Set the wow / flutter / hiss extras (staged; RT-safe).
    pub fn set_extras(&mut self, e: TapeExtras) {
        self.extras = e;
    }

    /// Head-bump frequency (Hz) and gain (dB) at tape speed `ips`.
    pub fn bump(ips: f32) -> (f32, f32) {
        let ips = ips.clamp(7.5, 30.0);
        (63.0 * ips / 15.0, 4.0 - (ips / 7.5).log2())
    }

    /// HF-loss low-pass corner (Hz) at tape speed `ips`.
    pub fn hf_corner(ips: f32) -> f32 {
        14_000.0 * (ips.clamp(7.5, 30.0) / 15.0).powf(0.6)
    }

    fn update(&mut self) {
        use tape_slot as s;
        self.dirty = false;
        let fs = self.fs;
        let g = |slot: u16| self.v[slot as usize - 1];
        let ips = g(s::SPD).clamp(7.5, 30.0);
        let drive = db_lin(g(s::DRV).clamp(-5.0, 25.0)) as f64;
        self.kg = TAPE_K * drive;
        self.inv = 1.0 / (TAPE_K * drive.sqrt());
        let (pre, post) = if g(s::HI) >= 0.5 {
            (
                Coeffs::high_shelf(fs, 8000.0, Q_BW, 6.0),
                Coeffs::high_shelf(fs, 8000.0, Q_BW, -4.0),
            )
        } else {
            (Coeffs::IDENTITY, Coeffs::IDENTITY)
        };
        let bump = if g(s::LOW) >= 0.5 {
            let (f, db) = Self::bump(ips);
            Coeffs::peak(fs, f as f64, 1.2, db as f64)
        } else {
            Coeffs::IDENTITY
        };
        let lp = Coeffs::lowpass(fs, (Self::hf_corner(ips) as f64).min(fs * 0.45), Q_BW);
        for ch in &mut self.ch {
            ch.pre.set_coeffs(pre);
            ch.post.set_coeffs(post);
            ch.bump.set_coeffs(bump);
            ch.lp.set_coeffs(lp);
        }
        self.sm.set_target(db_lin(g(s::OUT).clamp(-12.0, 12.0)));
    }

    #[inline]
    fn noise(&mut self) -> f32 {
        // xorshift64*, two uniforms summed (triangular)
        let mut x = self.rng;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.rng = x;
        let r = x.wrapping_mul(0x2545_F491_4F6C_DD1D);
        let a = (r >> 40) as f32 / (1u64 << 24) as f32;
        let b = ((r >> 8) & 0xFF_FFFF) as f32 / (1u64 << 24) as f32;
        a + b - 1.0
    }

    fn chunk(&mut self, left: &mut [f32], right: &mut [f32]) {
        let n = left.len();
        let (sat, kg, inv) = (self.sat, self.kg, self.inv);
        for (c, buf) in [(0usize, &mut *left), (1, &mut *right)] {
            let TapeCh {
                os,
                pre,
                post,
                bump,
                lp,
                ad,
                ..
            } = &mut self.ch[c];
            for x in buf.iter_mut() {
                *x = pre.tick(*x as f64) as f32;
            }
            os.process(buf, |hi| {
                for s in hi.iter_mut() {
                    let u = kg * *s as f64;
                    let r = ad.tick(u, |u| sat.r(u), |u| sat.br(u));
                    *s = ((u + r / sat.c1) * inv) as f32;
                }
            });
            for x in buf.iter_mut() {
                let y = post.tick(*x as f64);
                *x = lp.tick(bump.tick(y)) as f32;
            }
        }
        let e = self.extras;
        let ips = self.v[tape_slot::SPD as usize - 1].clamp(7.5, 30.0);
        let wf = e.wow > 0.0 || e.flutter > 0.0;
        let hiss_on = e.hiss_db > -150.0;
        let hiss_g = db_lin(e.hiss_db) * (15.0 / ips).sqrt();
        let slow = 15.0 / ips as f64;
        let (dw, df) = (
            core::f64::consts::TAU * 0.6 / self.fs,
            core::f64::consts::TAU * 9.0 / self.fs,
        );
        let centre = (TAPE_WOW_CENTRE_S * self.fs) as f32;
        let (aw, af) = (
            (e.wow as f64 * 0.00035 * slow * self.fs) as f32,
            (e.flutter as f64 * 0.00002 * slow * self.fs) as f32,
        );
        for i in 0..n {
            let og = self.sm.tick();
            let d = if wf {
                self.wow_ph += dw;
                self.flut_ph += df;
                if self.wow_ph > core::f64::consts::TAU {
                    self.wow_ph -= core::f64::consts::TAU;
                }
                if self.flut_ph > core::f64::consts::TAU {
                    self.flut_ph -= core::f64::consts::TAU;
                }
                let m = aw * (self.wow_ph.sin() as f32)
                    + aw * 0.3 * ((2.7 * self.wow_ph).sin() as f32)
                    + af * (self.flut_ph.sin() as f32);
                (centre + m).clamp(1.0, centre * 2.0)
            } else {
                0.0
            };
            let nzs = if hiss_on {
                [self.noise(), self.noise()]
            } else {
                [0.0; 2]
            };
            for (c, x) in [(0usize, &mut left[i]), (1, &mut right[i])] {
                let ch = &mut self.ch[c];
                let mut y = *x;
                if wf {
                    ch.dl.push(y);
                    y = ch.dl.read_lagrange3(d);
                }
                if hiss_on {
                    // tilt towards HF: white minus a one-pole low-pass
                    ch.hiss_lp += 0.13 * (nzs[c] - ch.hiss_lp);
                    y += hiss_g * 1.7 * (nzs[c] - 0.9 * ch.hiss_lp);
                }
                *x = y * og;
            }
        }
        for ch in &mut self.ch {
            flush(&mut [ch.pre, ch.post, ch.bump, ch.lp]);
        }
    }
}

impl Model for TapeMachine {
    fn id(&self) -> &'static str {
        "TAPE"
    }

    fn prepare(&mut self, sample_rate: f64, max_block: usize) {
        self.fs = sample_rate;
        self.mb = max_block.max(1);
        let dl = (2.0 * TAPE_WOW_CENTRE_S * sample_rate) as usize + 8;
        self.ch = [TapeCh::new(self.mb, dl), TapeCh::new(self.mb, dl)];
        self.sm = smoother(1.0, sample_rate);
        self.dirty = true;
        self.reset();
    }

    fn process(&mut self, left: &mut [f32], right: &mut [f32], _sc: Option<(&[f32], &[f32])>) {
        chunked_process!(self, left, right);
    }

    fn latency(&self) -> usize {
        OS_LATENCY
    }

    fn reset(&mut self) {
        self.update();
        self.ch.iter_mut().for_each(TapeCh::reset);
        self.wow_ph = 0.0;
        self.flut_ph = 0.0;
        let t = self.sm.target();
        self.sm.reset(t);
    }

    fn set_param(&mut self, slot: u16, value: ParamValue) {
        if stage(&mut self.v, slot, value) {
            self.dirty = true;
        }
    }

    fn param(&self, slot: u16) -> Option<f32> {
        staged(&self.v, slot)
    }

    fn load_defaults(&mut self) {
        self.v = TP_DEFAULTS;
        self.dirty = true;
    }

    fn commit(&mut self) {
        if self.dirty {
            self.update();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::{create_model, BlockKind};

    const FS: f64 = 48_000.0;
    const N: usize = 16_384;
    const WARM: usize = 8192;

    fn mk(id: &'static str) -> Box<dyn Model> {
        let mut m = create_model(BlockKind::Fx, id);
        m.prepare(FS, 64);
        m
    }

    fn set(m: &mut dyn Model, p: &[(u16, f32)]) {
        for &(s, v) in p {
            m.set_param(s, ParamValue::Float(v));
        }
        m.commit();
    }

    fn render(m: &mut dyn Model, l: &[f32], r: &[f32], block: usize) -> (Vec<f32>, Vec<f32>) {
        let (mut ol, mut or) = (l.to_vec(), r.to_vec());
        for (a, b) in ol.chunks_mut(block).zip(or.chunks_mut(block)) {
            m.process(a, b, None);
        }
        (ol, or)
    }

    /// Coherent sine at bin `k` of an `N`-point frame (amplitude `a`), `WARM + N` samples long.
    fn sine(k: usize, a: f32) -> Vec<f32> {
        (0..WARM + N)
            .map(|i| {
                a * (2.0 * std::f64::consts::PI * ((k * i) % N) as f64 / N as f64).sin() as f32
            })
            .collect()
    }

    /// Amplitude of bin `k` (coherent DFT of the last N samples).
    fn amp(x: &[f32], k: usize) -> f64 {
        let x = &x[x.len() - N..];
        let (mut re, mut im) = (0.0, 0.0);
        for (i, v) in x.iter().enumerate() {
            let ph = 2.0 * std::f64::consts::PI * ((k * i) % N) as f64 / N as f64;
            re += *v as f64 * ph.cos();
            im -= *v as f64 * ph.sin();
        }
        2.0 / N as f64 * (re * re + im * im).sqrt()
    }

    fn db(x: f64) -> f64 {
        20.0 * x.max(1e-12).log10()
    }

    /// In-place radix-2 FFT.
    fn fft(re: &mut [f64], im: &mut [f64]) {
        let n = re.len();
        let mut j = 0;
        for i in 1..n {
            let mut bit = n >> 1;
            while j & bit != 0 {
                j ^= bit;
                bit >>= 1;
            }
            j |= bit;
            if i < j {
                re.swap(i, j);
                im.swap(i, j);
            }
        }
        let mut len = 2;
        while len <= n {
            let ang = -2.0 * std::f64::consts::PI / len as f64;
            for s in (0..n).step_by(len) {
                for k in 0..len / 2 {
                    let (wr, wi) = ((ang * k as f64).cos(), (ang * k as f64).sin());
                    let (a, b) = (s + k, s + k + len / 2);
                    let (tr, ti) = (re[b] * wr - im[b] * wi, re[b] * wi + im[b] * wr);
                    re[b] = re[a] - tr;
                    im[b] = im[a] - ti;
                    re[a] += tr;
                    im[a] += ti;
                }
            }
            len *= 2;
        }
    }

    /// Worst non-harmonic line (dBc re the carrier at bin `kc`): every bin that is not a true in-band
    /// harmonic `m * kc` (+-8 bins), not DC and not the Nyquist edge counts as alias.
    fn alias_dbc(x: &[f32], kc: usize) -> f64 {
        let x = &x[x.len() - N..];
        let mut re: Vec<f64> = x
            .iter()
            .enumerate()
            .map(|(i, v)| {
                let w = 0.5 - 0.5 * (2.0 * std::f64::consts::PI * i as f64 / N as f64).cos();
                *v as f64 * w
            })
            .collect();
        let mut im = vec![0.0; N];
        fft(&mut re, &mut im);
        let mag: Vec<f64> = (0..N / 2)
            .map(|k| (re[k] * re[k] + im[k] * im[k]).sqrt())
            .collect();
        let carrier = mag[kc];
        let mut worst: f64 = 0.0;
        let mut wk = 0;
        for (k, &m) in mag.iter().enumerate().take(N / 2 - 8).skip(8) {
            let legit = (1..=N / 2 / kc + 1).any(|h| (k as i64 - (h * kc) as i64).abs() <= 8);
            if !legit {
                if m > worst {
                    wk = k;
                }
                worst = worst.max(m);
            }
        }
        eprintln!("  worst bin {} = {:.0} Hz", wk, wk as f64 * FS / N as f64);
        db(worst / carrier)
    }

    fn noise(seed: u64, n: usize, a: f32) -> Vec<f32> {
        let mut s = seed;
        (0..n)
            .map(|_| {
                s = s
                    .wrapping_mul(6364136223846793005)
                    .wrapping_add(1442695040888963407);
                ((s >> 33) as f32 / (1u64 << 31) as f32 - 0.5) * 2.0 * a
            })
            .collect()
    }

    fn rms(x: &[f32]) -> f64 {
        (x.iter().map(|v| (*v as f64).powi(2)).sum::<f64>() / x.len() as f64).sqrt()
    }

    const ALL: [&str; 6] = ["ENHANCE", "EXCITER", "P-BASS", "SUB-M", "V-IMG", "TAPE"];

    #[test]
    fn latencies_and_ids() {
        for id in ALL {
            let m = mk(id);
            assert_eq!(m.id(), id);
            assert!(!m.is_stub());
            let want = match id {
                "ENHANCE" | "EXCITER" | "TAPE" => OS_LATENCY,
                _ => 0,
            };
            assert_eq!(m.latency(), want, "{id}");
        }
    }

    #[test]
    fn defaults_match_schema_and_slots_roundtrip() {
        let defaults: [(&str, &[f32]); 6] = [
            (
                "ENHANCE",
                &[
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 15.0, 15.0, 25.0,
                ],
            ),
            ("EXCITER", &[3500.0, 28.0, 24.0, 14.0, 30.0, 30.0, 0.0]),
            ("P-BASS", &[-10.0, -3.0, 60.0, 0.0]),
            ("SUB-M", &[50.0, 50.0, 60.0, 80.0, 100.0, 80.0, 60.0]),
            ("V-IMG", &[0.0, 75.0, 0.0, 0.0, 0.0]),
            ("TAPE", &[0.0, 15.0, 0.0, 0.0, 0.0]),
        ];
        for (id, d) in defaults {
            let mut m = mk(id);
            m.load_defaults();
            for (i, v) in d.iter().enumerate() {
                assert_eq!(m.param(i as u16 + 1), Some(*v), "{id} slot {}", i + 1);
            }
            assert_eq!(m.param(d.len() as u16 + 1), None, "{id}");
            m.set_param(99, ParamValue::Float(1.0)); // ignored
        }
    }

    #[test]
    fn enhance_defaults_are_a_delayed_bypass() {
        let mut m = mk("ENHANCE");
        let x = noise(1, 4000, 0.5);
        let y = noise(2, 4000, 0.5);
        let (ol, or) = render(&mut *m, &x, &y, 64);
        for i in OS_LATENCY..4000 {
            assert!((ol[i] - x[i - OS_LATENCY]).abs() < 1e-6, "L {i}");
            assert!((or[i] - y[i - OS_LATENCY]).abs() < 1e-6, "R {i}");
        }
    }

    #[test]
    fn exciter_mix_zero_is_a_delayed_bypass() {
        let mut m = mk("EXCITER");
        set(&mut *m, &[(exciter_slot::MIX, 0.0)]);
        m.reset();
        let x = noise(3, 4000, 0.5);
        let (ol, _) = render(&mut *m, &x, &x, 64);
        for i in OS_LATENCY..4000 {
            assert!((ol[i] - x[i - OS_LATENCY]).abs() < 1e-6, "{i}");
        }
    }

    #[test]
    fn enhance_generates_harmonics_per_band() {
        // bass band: 80 Hz (bin 27) -> 2nd/3rd harmonics
        let k = 27;
        let x = sine(k, 0.5);
        let mut m = mk("ENHANCE");
        let (off, _) = render(&mut *m, &x, &x, 64);
        let mut m = mk("ENHANCE");
        set(&mut *m, &[(enhance_slot::BASS, 100.0)]);
        m.reset();
        let (on, _) = render(&mut *m, &x, &x, 64);
        let h = |y: &[f32]| db(amp(y, 2 * k).max(amp(y, 3 * k)));
        eprintln!(
            "bass band: harmonics off {:.1} dB on {:.1} dB",
            h(&off),
            h(&on)
        );
        assert!(h(&off) < -100.0, "{}", h(&off));
        assert!(h(&on) > -45.0, "{}", h(&on));
        // linear boost of the band
        eprintln!("fund on {} off {}", amp(&on, k), amp(&off, k));
        assert!(
            (amp(&on, k) / amp(&off, k) - 1.0).abs() < 0.3,
            "band stays within +-30 %"
        );
        // high band: 6 kHz (bin 2048) -> 12 kHz
        let k = 2048;
        let x = sine(k, 0.4);
        let mut m = mk("ENHANCE");
        set(&mut *m, &[(enhance_slot::HIGH, 100.0)]);
        m.reset();
        let (on, _) = render(&mut *m, &x, &x, 64);
        eprintln!("high band: 12 kHz {:.1} dB", db(amp(&on, 2 * k)));
        assert!(db(amp(&on, 2 * k)) > -45.0);
        // mid band: 1 kHz (bin 341)
        let k = 341;
        let x = sine(k, 0.4);
        let mut m = mk("ENHANCE");
        set(&mut *m, &[(enhance_slot::MID, 100.0)]);
        m.reset();
        let (on, _) = render(&mut *m, &x, &x, 64);
        eprintln!(
            "mid band: 2k/3k {:.1} dB",
            db(amp(&on, 2 * k).max(amp(&on, 3 * k)))
        );
        assert!(db(amp(&on, 2 * k).max(amp(&on, 3 * k))) > -45.0);
    }

    #[test]
    fn enhance_stereo_stage() {
        let x = noise(5, 4096, 0.3);
        let y = noise(6, 4096, 0.3);
        let m_in: Vec<f32> = x.iter().zip(&y).map(|(a, b)| 0.5 * (a + b)).collect();
        // MONO LVL -100 % removes the mid component, leaves the side
        let mut m = mk("ENHANCE");
        set(&mut *m, &[(enhance_slot::MLVL, -100.0)]);
        m.reset();
        let (ol, or) = render(&mut *m, &x, &y, 64);
        let mid_out: Vec<f32> = ol.iter().zip(&or).map(|(a, b)| 0.5 * (a + b)).collect();
        assert!(rms(&mid_out[200..]) < 1e-3 * rms(&m_in), "mid not removed");
        // ST LVL -100 % collapses to mono
        let mut m = mk("ENHANCE");
        set(&mut *m, &[(enhance_slot::STLVL, -100.0)]);
        m.reset();
        let (ol, or) = render(&mut *m, &x, &y, 64);
        for i in 200..4096 {
            assert!((ol[i] - or[i]).abs() < 1e-5);
        }
        // MONO PAN hard right silences a mono signal on the left
        let mut m = mk("ENHANCE");
        set(&mut *m, &[(enhance_slot::M, 100.0)]);
        m.reset();
        let (ol, or) = render(&mut *m, &x, &x, 64);
        assert!(rms(&ol[200..]) < 1e-5 && rms(&or[200..]) > 0.05);
        // GAIN +12 dB
        let mut m = mk("ENHANCE");
        set(&mut *m, &[(enhance_slot::G, 12.0)]);
        m.reset();
        let (ol, _) = render(&mut *m, &x, &x, 64);
        let r = rms(&ol[200..]) / rms(&x[200 - OS_LATENCY..4096 - OS_LATENCY]);
        assert!((db(r) - 12.0).abs() < 0.1, "{}", db(r));
        // SOLO with no enhancement is silent
        let mut m = mk("ENHANCE");
        set(&mut *m, &[(enhance_slot::SOLO, 1.0)]);
        m.reset();
        let (ol, _) = render(&mut *m, &x, &x, 64);
        assert!(rms(&ol) < 1e-6);
    }

    #[test]
    fn exciter_adds_high_harmonics() {
        // 3 kHz tone (bin 1024): 6/9/12 kHz appear
        let k = 1024;
        let x = sine(k, 0.5);
        let mut m = mk("EXCITER");
        let (y, _) = render(&mut *m, &x, &x, 64);
        let h = db(amp(&y, 2 * k).max(amp(&y, 3 * k)));
        eprintln!("exciter default: 2nd/3rd {h:.1} dB");
        assert!(h > -50.0, "{h}");
        // HARM 0 -> less than HARM 100
        let hh = |hrm: f32| {
            let mut m = mk("EXCITER");
            set(
                &mut *m,
                &[(exciter_slot::HRM, hrm), (exciter_slot::MIX, 100.0)],
            );
            m.reset();
            let (y, _) = render(&mut *m, &x, &x, 64);
            db(amp(&y, 2 * k).max(amp(&y, 3 * k)))
        };
        assert!(hh(100.0) > hh(0.0) + 3.0);
        // low content stays clean: a 200 Hz tone (bin 68) is far below TUNE
        let k = 68;
        let x = sine(k, 0.5);
        let mut m = mk("EXCITER");
        let (y, _) = render(&mut *m, &x, &x, 64);
        assert!(db(amp(&y, 2 * k).max(amp(&y, 3 * k))) < -55.0);
        // DRY OFF removes the dry fundamental
        let k = 341;
        let x = sine(k, 0.5);
        let mut m = mk("EXCITER");
        set(&mut *m, &[(exciter_slot::SOLO, 1.0)]);
        m.reset();
        let (y, _) = render(&mut *m, &x, &x, 64);
        assert!(amp(&y, k) < 0.05, "{}", amp(&y, k));
        // TIMBRE changes the even-harmonic balance
        let k = 1024;
        let x = sine(k, 0.3);
        let h2 = |t: f32| {
            let mut m = mk("EXCITER");
            set(
                &mut *m,
                &[(exciter_slot::TMB, t), (exciter_slot::MIX, 100.0)],
            );
            m.reset();
            let (y, _) = render(&mut *m, &x, &x, 64);
            amp(&y, 2 * k)
        };
        assert!(h2(50.0) > h2(0.0) * 1.2 || h2(-50.0) > h2(0.0) * 1.2);
    }

    /// Worst alias line (dBc) of model `id` at `params` for a 0.8 sine at bin `kc`.
    fn alias_of(id: &'static str, params: &[(u16, f32)], kc: usize) -> f64 {
        let x = sine(kc, 0.8);
        let mut m = mk(id);
        set(&mut *m, params);
        m.reset();
        let (y, _) = render(&mut *m, &x, &x, 64);
        alias_dbc(&y, kc)
    }

    #[test]
    fn aliasing_below_minus_80_dbc() {
        // 9.7 kHz (bin 3313) and 5.9 kHz (bin 2000) full-scale-ish carriers, hot but musical settings
        let hot: [(&'static str, Vec<(u16, f32)>); 4] = [
            (
                "ENHANCE",
                vec![
                    (enhance_slot::BASS, 100.0),
                    (enhance_slot::MID, 100.0),
                    (enhance_slot::HIGH, 100.0),
                    (enhance_slot::HIGHF, 1.0),
                ],
            ),
            // EXCITER at its WING-Edit defaults, 100 % wet
            ("EXCITER", vec![(exciter_slot::MIX, 100.0)]),
            ("TAPE", vec![(tape_slot::DRV, 12.0)]),
            ("TAPE", vec![(tape_slot::DRV, 6.0), (tape_slot::HI, 1.0)]),
        ];
        for kc in [3313usize, 2000] {
            for (id, p) in &hot {
                let a = alias_of(id, p, kc);
                eprintln!("{id} bin {kc}: worst alias line {a:.1} dBc");
                assert!(a < -80.0, "{id} bin {kc}: {a} dBc");
            }
        }
    }

    #[test]
    fn aliasing_at_maximum_settings_is_bounded() {
        // Known limit (module doc): a 4x ADAA1 shaper driven to a hard clip at a 9.7 kHz carrier aliases at
        // about -63 dBc (21st harmonic folded); bound it so regressions show.
        let kc = 3313;
        let ex = alias_of(
            "EXCITER",
            &[
                (exciter_slot::HRM, 100.0),
                (exciter_slot::PEAK, 100.0),
                (exciter_slot::ZERO, 100.0),
                (exciter_slot::MIX, 100.0),
                (exciter_slot::TUNE, 1000.0),
            ],
            kc,
        );
        let tp = alias_of(
            "TAPE",
            &[(tape_slot::DRV, 25.0), (tape_slot::SPD, 30.0)],
            kc,
        );
        eprintln!("max settings: EXCITER {ex:.1} dBc, TAPE {tp:.1} dBc");
        assert!(ex < -60.0 && tp < -55.0, "{ex} {tp}");
    }

    #[test]
    fn pbass_psychoacoustic_harmonics() {
        // 50 Hz (bin 17 = 49.8 Hz), crossover 100 Hz
        let k = 17;
        let x = sine(k, 0.5);
        let run = |int: f32, bass: f32| {
            let mut m = mk("P-BASS");
            set(
                &mut *m,
                &[
                    (pbass_slot::INT, int),
                    (pbass_slot::BASS, bass),
                    (pbass_slot::XF, 100.0),
                ],
            );
            m.reset();
            render(&mut *m, &x, &x, 64).0
        };
        let y = run(0.0, -60.0);
        let fund = amp(&y, k);
        let (h2, h3) = (amp(&y, 2 * k), amp(&y, 3 * k));
        eprintln!(
            "p-bass: fund {:.1} dB, 2nd {:.1} dB, 3rd {:.1} dB",
            db(fund),
            db(h2),
            db(h3)
        );
        assert!(h2 > 0.02 && h3 > 0.01, "harmonics present: {h2} {h3}");
        // (the LR4 high band still passes the fundamental at -24 dB: 50 Hz is half the crossover)
        assert!(
            fund < 0.04 && h2 > 1.5 * fund,
            "fundamental removed: {fund}"
        );
        // INTENSITY scales the harmonics linearly in dB
        let lo = run(-24.0, -60.0);
        let ratio = db(amp(&y, 2 * k)) - db(amp(&lo, 2 * k));
        assert!((ratio - 24.0).abs() < 1.0, "{ratio}");
        // BASS GAIN 0 dB keeps the fundamental
        let y0 = run(-24.0, 0.0);
        assert!(
            (db(amp(&y0, k)) - db(0.5)).abs() < 1.5,
            "{}",
            db(amp(&y0, k))
        );
        // crossover is flat above X/O: 2 kHz passes at unity (bin 683)
        let xs = sine(683, 0.5);
        let mut m = mk("P-BASS");
        set(
            &mut *m,
            &[(pbass_slot::INT, -24.0), (pbass_slot::BASS, 0.0)],
        );
        m.reset();
        let (ys, _) = render(&mut *m, &xs, &xs, 64);
        assert!((db(amp(&ys, 683)) - db(0.5)).abs() < 0.2);
        // SOLO drops the high band
        let mut m = mk("P-BASS");
        set(&mut *m, &[(pbass_slot::SOLO, 1.0)]);
        m.reset();
        let (ys, _) = render(&mut *m, &xs, &xs, 64);
        assert!(amp(&ys, 683) < 0.01);
    }

    #[test]
    fn submonster_synthesises_the_octave_below() {
        for (k, name) in [(34usize, "100 Hz"), (20, "58.6 Hz")] {
            let x = sine(k, 0.5);
            let mut m = mk("SUB-M");
            set(&mut *m, &[(subm_slot::MIX, 100.0)]);
            m.reset();
            let (y, _) = render(&mut *m, &x, &x, 64);
            // the sub is at k/2 bins (k even); input keeps its level
            let sub = amp(&y, k / 2);
            eprintln!(
                "sub-m {name}: sub {:.1} dB, input {:.1} dB",
                db(sub),
                db(amp(&y, k))
            );
            assert!(sub > 0.1, "{name}: {sub}");
            assert!((amp(&y, k) - 0.5).abs() < 0.08);
        }
        // mix 0 is a pass-through
        let x = sine(34, 0.5);
        let mut m = mk("SUB-M");
        set(&mut *m, &[(subm_slot::MIX, 0.0)]);
        m.reset();
        let (y, _) = render(&mut *m, &x, &x, 64);
        for i in 0..x.len() {
            assert!((y[i] - x[i]).abs() < 1e-6);
        }
        // below the gate: no synthesised content
        let x = sine(34, 1e-4);
        let mut m = mk("SUB-M");
        set(&mut *m, &[(subm_slot::MIX, 100.0)]);
        m.reset();
        let (y, _) = render(&mut *m, &x, &x, 64);
        assert!(amp(&y, 17) < 2e-5);
        // BAND levels shape the response: all bands 0 -> no sub
        let x = sine(34, 0.5);
        let mut m = mk("SUB-M");
        set(&mut *m, &[(subm_slot::MIX, 100.0)]);
        for s in subm_slot::BD1..=subm_slot::BD5 {
            set(&mut *m, &[(s, 0.0)]);
        }
        m.reset();
        let (y, _) = render(&mut *m, &x, &x, 64);
        assert!(amp(&y, 17) < 1e-3);
    }

    #[test]
    fn vimg_is_mono_compatible() {
        let (x, y) = (noise(11, 6000, 0.5), noise(12, 6000, 0.5));
        let m_in: Vec<f32> = x.iter().zip(&y).map(|(a, b)| 0.5 * (a + b)).collect();
        for wid in [-1.0f32, -0.4, 0.0, 0.6, 1.0] {
            for st in [0.0f32, 75.0, 100.0] {
                for mode in [0.0f32, 1.0] {
                    for deep in [0.0f32, 1.0] {
                        let mut m = mk("V-IMG");
                        let g = 3.0;
                        set(
                            &mut *m,
                            &[
                                (vimg_slot::WID, wid),
                                (vimg_slot::ST, st),
                                (vimg_slot::G, g),
                                (vimg_slot::MODE, mode),
                                (vimg_slot::DEEP, deep),
                            ],
                        );
                        m.reset();
                        let (ol, or) = render(&mut *m, &x, &y, 64);
                        let gl = db_lin(g);
                        for i in 0..6000 {
                            let mono = 0.5 * (ol[i] + or[i]);
                            assert!(
                                (mono - gl * m_in[i]).abs() < 2e-6,
                                "w{wid} st{st} m{mode} d{deep} i{i}"
                            );
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn vimg_width_behaviour() {
        let (x, y) = (noise(21, 8000, 0.5), noise(22, 8000, 0.5));
        let side = |l: &[f32], r: &[f32]| {
            rms(&l
                .iter()
                .zip(r)
                .map(|(a, b)| 0.5 * (a - b))
                .collect::<Vec<_>>()[500..])
        };
        let s_in = side(&x, &y);
        // STEREO 0, WIDTH 0: identity
        let mut m = mk("V-IMG");
        set(&mut *m, &[(vimg_slot::ST, 0.0)]);
        m.reset();
        let (ol, or) = render(&mut *m, &x, &y, 64);
        for i in 0..8000 {
            assert!((ol[i] - x[i]).abs() < 1e-5 && (or[i] - y[i]).abs() < 1e-5);
        }
        // narrowing to -1 with DEEP: mono
        let mut m = mk("V-IMG");
        set(
            &mut *m,
            &[
                (vimg_slot::WID, -1.0),
                (vimg_slot::ST, 0.0),
                (vimg_slot::DEEP, 1.0),
            ],
        );
        m.reset();
        let (ol, or) = render(&mut *m, &x, &y, 64);
        assert!(side(&ol, &or) < 1e-5);
        // widening raises the side energy; DEEP raises it more (also widens the lows)
        let widened = |deep: f32| {
            let mut m = mk("V-IMG");
            set(
                &mut *m,
                &[
                    (vimg_slot::WID, 1.0),
                    (vimg_slot::ST, 0.0),
                    (vimg_slot::DEEP, deep),
                ],
            );
            m.reset();
            let (ol, or) = render(&mut *m, &x, &y, 64);
            side(&ol, &or)
        };
        assert!(widened(0.0) > 1.5 * s_in);
        assert!(widened(1.0) > widened(0.0));
        // a mono source: K-STEREO stays mono, VELVET makes pseudo-stereo
        let mono = noise(31, 8000, 0.5);
        for (mode, wide) in [(0.0f32, false), (1.0, true)] {
            let mut m = mk("V-IMG");
            set(&mut *m, &[(vimg_slot::MODE, mode), (vimg_slot::ST, 100.0)]);
            m.reset();
            let (ol, or) = render(&mut *m, &mono, &mono, 64);
            let s = side(&ol, &or);
            assert_eq!(s > 0.01, wide, "mode {mode}: side {s}");
        }
        // STEREO adds ambience from an existing side signal
        let mut m = mk("V-IMG");
        set(&mut *m, &[(vimg_slot::ST, 100.0)]);
        m.reset();
        let (ol, or) = render(&mut *m, &x, &y, 64);
        assert!(side(&ol, &or) > 1.1 * s_in);
    }

    #[test]
    fn tape_character() {
        let at = |p: &[(u16, f32)], k: usize, a: f32| {
            let x = sine(k, a);
            let mut m = mk("TAPE");
            set(&mut *m, p);
            m.reset();
            let (y, _) = render(&mut *m, &x, &x, 64);
            y
        };
        // unity small-signal gain at defaults (1 kHz, -40 dBFS)
        let y = at(&[], 341, 0.01);
        assert!(
            (db(amp(&y, 341)) - db(0.01)).abs() < 0.3,
            "{}",
            db(amp(&y, 341))
        );
        // drive creates odd (3rd) dominated harmonics that grow with drive
        let h = |drv: f32| {
            let y = at(&[(tape_slot::DRV, drv)], 341, 0.5);
            (db(amp(&y, 2 * 341)), db(amp(&y, 3 * 341)))
        };
        let (h2a, h3a) = h(6.0);
        let (h2b, h3b) = h(18.0);
        eprintln!("tape drv 6: H2 {h2a:.1} H3 {h3a:.1}; drv 18: H2 {h2b:.1} H3 {h3b:.1}");
        assert!(h3b > h3a + 3.0 && h3b > -40.0);
        assert!(h3a > h2a, "odd harmonics lead");
        // head bump (63 Hz at 15 ips, bin 21 = 61.5 Hz): ~ +3 dB
        let flat = at(&[], 21, 0.01);
        let bump = at(&[(tape_slot::LOW, 1.0)], 21, 0.01);
        let g = db(amp(&bump, 21)) - db(amp(&flat, 21));
        assert!(g > 2.0 && g < 4.0, "bump {g}");
        // bump moves with the speed: at 30 ips 61.5 Hz is below the 126 Hz peak
        let bump30 = at(&[(tape_slot::LOW, 1.0), (tape_slot::SPD, 30.0)], 21, 0.01);
        assert!(db(amp(&bump30, 21)) - db(amp(&flat, 21)) < g - 0.5);
        // HF loss grows as the speed drops (15 kHz = bin 5120)
        let hf = |spd: f32| db(amp(&at(&[(tape_slot::SPD, spd)], 5120, 0.01), 5120));
        assert!(
            hf(30.0) > hf(15.0) + 2.0 && hf(15.0) > hf(7.5) + 2.0,
            "{} {} {}",
            hf(30.0),
            hf(15.0),
            hf(7.5)
        );
        // HIGH SHELV: net HF lift (10 kHz = bin 3413)
        let hi = db(amp(&at(&[(tape_slot::HI, 1.0)], 3413, 0.01), 3413))
            - db(amp(&at(&[], 3413, 0.01), 3413));
        assert!(hi > 1.0 && hi < 3.0, "{hi}");
        // OUT GAIN
        let og = db(amp(&at(&[(tape_slot::OUT, 6.0)], 341, 0.01), 341))
            - db(amp(&at(&[], 341, 0.01), 341));
        assert!((og - 6.0).abs() < 0.1, "{og}");
    }

    #[test]
    fn tape_extras_wow_flutter_hiss() {
        let x = sine(341, 0.3);
        let mut m = mk("TAPE");
        let (base, _) = render(&mut *m, &x, &x, 64);
        // hiss: silent input gives a noise floor
        let mut t = TapeMachine::new();
        t.prepare(FS, 64);
        t.set_extras(TapeExtras {
            hiss_db: -60.0,
            ..Default::default()
        });
        let silence = vec![0.0f32; 8000];
        let (hl, hr) = render(&mut t, &silence, &silence, 64);
        let r = db(rms(&hl));
        assert!(r > -75.0 && r < -50.0, "hiss {r} dBFS");
        assert!(
            hl.iter().zip(&hr).any(|(a, b)| a != b),
            "channels decorrelated"
        );
        // slower speed hisses more
        let mut t2 = TapeMachine::new();
        t2.prepare(FS, 64);
        t2.set_param(tape_slot::SPD, ParamValue::Float(7.5));
        t2.set_extras(TapeExtras {
            hiss_db: -60.0,
            ..Default::default()
        });
        let (h2, _) = render(&mut t2, &silence, &silence, 64);
        assert!(rms(&h2) > rms(&hl) * 1.3);
        // wow/flutter smears the pure tone
        let mut t = TapeMachine::new();
        t.prepare(FS, 64);
        t.set_extras(TapeExtras {
            wow: 1.0,
            flutter: 1.0,
            ..Default::default()
        });
        let (w, _) = render(&mut t, &x, &x, 64);
        let purity = |y: &[f32]| amp(y, 341) / rms(&y[y.len() - N..]) / std::f64::consts::SQRT_2;
        assert!(purity(&base) > 0.99);
        assert!(purity(&w) < 0.97, "{}", purity(&w));
    }

    #[test]
    fn block_size_independence() {
        let (x, y) = (noise(41, 3000, 0.4), noise(42, 3000, 0.4));
        for id in ALL {
            let mut a = mk(id);
            let mut b = mk(id);
            let mut c = mk(id);
            for m in [&mut a, &mut b, &mut c] {
                // turn the effect up so every path is exercised
                for s in 1..=5u16 {
                    m.set_param(s, ParamValue::Float(if s == 1 { 0.5 } else { 40.0 }));
                }
                m.commit();
                m.reset();
            }
            let (al, ar) = render(&mut *a, &x, &y, 64);
            let (bl, br) = render(&mut *b, &x, &y, 1);
            let (cl, cr) = render(&mut *c, &x, &y, 200);
            for i in 0..3000 {
                assert!((al[i] - bl[i]).abs() < 1e-5, "{id} 1 vs 64 at {i}");
                assert!((ar[i] - br[i]).abs() < 1e-5, "{id}");
                assert!((al[i] - cl[i]).abs() < 1e-5, "{id} 200 vs 64 at {i}");
                assert!((ar[i] - cr[i]).abs() < 1e-5, "{id}");
            }
        }
    }

    #[test]
    fn extreme_parameters_stay_finite_and_bounded() {
        let x = noise(51, 4000, 1.0);
        let lows = [
            -100.0f32, -50.0, -24.0, -12.0, -6.0, -5.0, -1.0, 0.0, 1.0, 7.5, 32.0, 45.0,
        ];
        let highs = [
            100.0f32, 50.0, 25.0, 30.0, 12.0, 10_000.0, 200.0, 67.5, 6.0, 1.0,
        ];
        for id in ALL {
            for set_hi in [false, true] {
                let mut m = mk(id);
                let vals = if set_hi { &highs[..] } else { &lows[..] };
                for (i, v) in vals.iter().enumerate() {
                    for s in 1..=13u16 {
                        m.set_param(s, ParamValue::Float(vals[(i + s as usize) % vals.len()]));
                    }
                    m.commit();
                    let (l, r) = render(&mut *m, &x[..500], &x[500..1000], 64);
                    let mx = l.iter().chain(&r).fold(0.0f32, |a, v| a.max(v.abs()));
                    assert!(
                        l.iter().chain(&r).all(|v| v.is_finite()) && mx < 256.0,
                        "{id} {set_hi} {v}: max {mx}"
                    );
                }
            }
        }
    }

    #[test]
    fn reset_clears_state() {
        let x = noise(61, 2000, 0.5);
        for id in ALL {
            let mut m = mk(id);
            for s in 1..=5u16 {
                m.set_param(s, ParamValue::Float(if s == 1 { 0.5 } else { 40.0 }));
            }
            m.commit();
            m.reset();
            let (a, _) = render(&mut *m, &x, &x, 64);
            m.reset();
            let (b, _) = render(&mut *m, &x, &x, 64);
            assert_eq!(a, b, "{id}");
        }
    }
}
