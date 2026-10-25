//! FX rack pitch models (T05T, models.md §6): `PITCH` (STEREO PITCH), `D-PITCH` (DUAL PITCH), `DOUBLE`
//! (DOUBLE VOCAL), `PCORR` (PITCH FIX) and `SUB` (SUB OCTAVER).
//!
//! All parameter keys, ranges and defaults follow the schema (`data/schema/wing-tree.json`, WING-Edit
//! `fx_pardef_*`); note the schema's D-PITCH has no 1k..16k EQ and PCORR no PHASE/WAVE keys (models.md lists
//! them from an older table), so neither does this module.
//!
//! **Shifter.** Every pitch voice is a low-latency time-domain shifter: a delay line read by a head whose
//! delay changes by `1 - ratio` per sample (so the head reads the input at `ratio` x speed, which gives the
//! exact shifted frequency between splices). When the head is about to leave its window it *splices*: a
//! second head starts one jump `J` away and the two are cross-faded (raised cosine, equal gain). `J` is
//! chosen by normalised cross-correlation of the signal at the two positions ([`search_jump`], coarse
//! search + full-resolution refinement + parabolic peak), so for periodic material the new head lands an
//! integer number of periods away and the splice is phase-coherent (no comb, no pitch error). Algorithm:
//! classic dual-head delay-line shifter with correlated splice points (Lent 1989 / "SOLA"-style), own
//! implementation; no third-party code (Signalsmith Stretch would add ~tens of ms latency, Rubber Band is GPL).
//!
//! **Latency.** The FX slot runs at the loaded model's [`Model::latency`] and the engine aligns every strip of a
//! family to the largest one in use (T05U), so a PCORR insert delays all input strips: it is kept as small as
//! quality allows:
//! * `PITCH`, `D-PITCH`, `DOUBLE`: `latency() = 0`. The dry signal is not delayed; the shifted voice is
//!   delayed by `DELAY` plus the shifter window (between `DELAY` and `DELAY + ~31 ms` at 48 kHz, mean ~12..19
//!   ms depending on the direction of the shift; at unison exactly `DELAY`, min 2 samples), which is part of
//!   the effect like a harmonizer's own delay. [`StereoPitch::wet_delay_range`] reports the window.
//! * `PCORR`: the corrected signal *replaces* the source, so its head is kept centred on a fixed delay of
//!   [`PCORR_LATENCY_MS`] (7.5 ms = **360 samples at 48 kHz**, 331 at 44.1 kHz) and splices by whole detected
//!   periods around it; `latency()` reports that figure and with no correction the output is the input
//!   delayed by exactly `latency()` samples. The detector looks at the newest input, so the correction is
//!   roughly time-aligned with the delayed audio.
//! * `SUB`: `latency() = 0` (analog-style divider, filters only).
//!
//! WING's own pitch FX latency is not documented (signal-flow.md §11: not obtainable; [I] uncompensated).
//!
//! Model notes (items marked [I] are inferred, no console measurement yet):
//! * **PITCH** - stereo, both channels share one head schedule (splices are correlated on the mono sum, so
//!   the image stays coherent). Ratio `2^((SEMI + CENT/100)/12)`; `DELAY` is the base delay of the wet
//!   voice; LO/HI CUT 12 dB/oct on the wet path; `MIX` dry/wet.
//! * **D-PITCH** - two such voices from the mono sum, each with its own semitones/cents/delay, constant-power
//!   `PAN` and fader `LEVEL`; LO/HI CUT on the output; no internal mix (the slot's `fxmix` blends dry).
//! * **DOUBLE** - 2 or 4 shifter voices from the mono sum with small static detune plus a slow smoothed
//!   random pitch wander (which also makes their timing drift inside the window) and per-voice base delays,
//!   spread across the stereo field by `SPREAD`; `MIX` dry/wet. Mode table [`DOUBLE_MODES`] [I].
//! * **PCORR** - YIN pitch detector (cumulative-mean-normalised difference, threshold 0.15) on a 4x
//!   decimated (~12 kHz) mono sum every 32 decimated samples (~2.7 ms), refined at full rate with a
//!   difference-function search and parabolic interpolation (accurate to well under a cent on steady
//!   tones), 75..1000 Hz. The detected note (`A4 PITCH` reference) snaps to the nearest enabled note of the
//!   12 `sw_*` switches (none enabled = chromatic [I]), with 0.15 semitone hysteresis. `AMOUNT` 0..100 =
//!   fraction of the deviation corrected [I]; `SPEED` 1..100 = retune time constant `400^((100-SPD)/99)`
//!   ms (100 = 1 ms "hard tune", 50 = 20 ms, 1 = 400 ms) [I]. Unvoiced input glides the correction to 0 and
//!   re-centres the head after 50 ms.
//! * **SUB** - Boss OC-2 style: the mono sum is band-limited (25 Hz HP, 4th-order LP at the `RANGE`
//!   frequency, LOW/MID/HIGH = 200/400/800 Hz [I]), a Schmitt trigger (hysteresis 10 % of the envelope)
//!   clocks a flip-flop (/2) which clocks a second one (/4); like the OC-2, each square switches the
//!   polarity of the filtered signal, so the sub follows the playing dynamics and gates itself. Sub
//!   outputs are low-passed at the range frequency and added to the (unchanged) dry signal at `OCTAVE 1/2`
//!   levels (100 % = unity sub gain) [I].
//!
//! RT safety: all buffers are allocated in `prepare`; `process`, `set_param`, `commit`, `reset` never
//! allocate (see `tests/fx_pitch_no_alloc.rs`).

use crate::model::{Model, ParamValue};
use core::f64::consts::PI;
use wing_dsp::delay::DelayLine;
use wing_dsp::svf::{Svf, SvfMode};

const Q_BW: f64 = core::f64::consts::FRAC_1_SQRT_2;
/// Parameter glide time constant (mix, levels, pans).
const GLIDE_S: f64 = 0.02;
/// Largest PITCH / D-PITCH ratio (+12 semitones + 50 cents).
const RATIO_MAX: f64 = 2.059_463_094_359_295;

// ---------------------------------------------------------------------------------------------------------
// Shared building blocks
// ---------------------------------------------------------------------------------------------------------

/// Per-sample one-pole glide towards a target.
#[derive(Clone, Copy, Debug)]
struct Lag {
    v: f32,
    t: f32,
    a: f32,
}

impl Lag {
    const fn new(v: f32) -> Self {
        Self { v, t: v, a: 1.0 }
    }

    fn set_tau(&mut self, tau_s: f64, fs: f64) {
        self.a = (1.0 - (-1.0 / (tau_s * fs)).exp()) as f32;
    }

    #[inline]
    fn tick(&mut self) -> f32 {
        self.v += self.a * (self.t - self.v);
        self.v
    }

    fn snap(&mut self) {
        self.v = self.t;
    }

    fn settle(&mut self) {
        if (self.t - self.v).abs() < 1e-7 {
            self.v = self.t;
        }
    }
}

fn hz(v: f32, lo: f32, hi: f32, fs: f64) -> f64 {
    v.clamp(lo, hi).min((fs * 0.45) as f32) as f64
}

/// Fader dB (-144 = -oo .. +10) to linear.
fn fader(db: f32) -> f32 {
    if db <= -143.9 {
        0.0
    } else {
        wing_dsp::gain::db_to_lin(db.min(10.0))
    }
}

/// Constant-power pan, `p` in -1..1: (left, right) gains, -3 dB each at centre.
fn pan_gains(p: f32) -> (f32, f32) {
    let th = (p.clamp(-1.0, 1.0) + 1.0) * core::f32::consts::FRAC_PI_4;
    (th.cos(), th.sin())
}

/// Shifted-pitch ratio for semitones + cents.
pub fn ratio(semi: f32, cent: f32) -> f64 {
    2f64.powf((semi.round() as f64 + cent.round() as f64 / 100.0) / 12.0)
}

/// Shifter geometry in samples (see the module doc).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ShiftCfg {
    /// Splice cross-fade length.
    pub fade: u32,
    /// Allowed jump range.
    pub jmin: f64,
    pub jmax: f64,
    /// Head window: delays stay within `[lo, lo + win]`.
    pub win: f64,
    /// Correlation segment length.
    pub seg: usize,
}

impl ShiftCfg {
    pub fn new(
        fs: f64,
        fade_ms: f64,
        jmin_ms: f64,
        jmax_ms: f64,
        seg_ms: f64,
        ratio_max: f64,
    ) -> Self {
        let s = fs * 1e-3;
        let fade = (fade_ms * s).round().max(1.0) as u32;
        let jmax = jmax_ms * s;
        Self {
            fade,
            jmin: jmin_ms * s,
            jmax,
            win: jmax + fade as f64 * (ratio_max - 1.0).abs() + 4.0,
            seg: (seg_ms * s).round().max(8.0) as usize,
        }
    }

    /// Delay-line headroom needed beyond the base delay (window + correlation segment + interpolation).
    pub fn span(&self) -> usize {
        (self.win + self.seg as f64 + 8.0).ceil() as usize
    }
}

/// Two read heads with a splice cross-fade (the active head fades in, the other out).
#[derive(Clone, Copy, Debug, Default)]
pub struct Shifter {
    d: [f64; 2],
    act: usize,
    left: u32,
    len: u32,
}

impl Shifter {
    pub fn reset(&mut self, d: f64) {
        self.d = [d; 2];
        self.act = 0;
        self.left = 0;
        self.len = 0;
    }

    /// Delay (samples) of the active head.
    #[inline]
    pub fn delay(&self) -> f64 {
        self.d[self.act]
    }

    #[inline]
    pub fn fading(&self) -> bool {
        self.left > 0
    }

    /// Start a new head at delay `d`, cross-faded over `len` samples.
    pub fn splice_to(&mut self, d: f64, len: u32) {
        let n = 1 - self.act;
        self.d[n] = d;
        self.act = n;
        self.left = len.max(1);
        self.len = self.left;
    }

    /// `[(delay, gain); 2]`: active head first.
    #[inline]
    pub fn taps(&self) -> [(f32, f32); 2] {
        let (da, db) = (self.d[self.act] as f32, self.d[1 - self.act] as f32);
        if self.left == 0 {
            [(da, 1.0), (db, 0.0)]
        } else {
            let x = 1.0 - self.left as f64 / self.len as f64;
            let g = (0.5 - 0.5 * (PI * x).cos()) as f32;
            [(da, g), (db, 1.0 - g)]
        }
    }

    /// Read the (cross-faded) output from `line`.
    #[inline]
    pub fn read(&self, line: &DelayLine) -> f32 {
        let [(da, ga), (db, gb)] = self.taps();
        let mut y = ga * read(line, da);
        if gb > 0.0 {
            y += gb * read(line, db);
        }
        y
    }

    /// Advance both heads by one output sample at `ratio`.
    #[inline]
    pub fn advance(&mut self, ratio: f64) {
        let step = 1.0 - ratio;
        self.d[0] += step;
        self.d[1] += step;
        if self.left > 0 {
            self.left -= 1;
        }
    }
}

#[inline]
fn read(line: &DelayLine, d: f32) -> f32 {
    line.read_lagrange3(d)
}

/// Correlation of the `seg` samples behind delay `base` with those behind `base + dir * j`
/// (energy-normalised by the candidate), sampling every `step`-th sample.
fn corr_score(line: &DelayLine, base: usize, dir: f64, j: usize, seg: usize, step: usize) -> f32 {
    let (mut ab, mut bb) = (0.0f32, 1e-20f32);
    let mut n = 0;
    while n < seg {
        let a = line.read_int(base + n);
        let k = if dir > 0.0 {
            base + j + n
        } else {
            (base + n).saturating_sub(j)
        };
        let b = line.read_int(k);
        ab += a * b;
        bb += b * b;
        n += step;
    }
    ab / bb.sqrt()
}

/// Best splice jump in `[jmin, jmax]` (fractional samples) from a head at delay `d` towards larger
/// (`dir > 0`) or smaller (`dir < 0`) delays: maximum normalised cross-correlation, coarse (every 2nd lag,
/// every 4th sample) then full resolution around the coarse peak, then a parabolic peak fit. Ties prefer
/// the largest jump (fewest splices). RT-safe.
pub fn search_jump(line: &DelayLine, d: f64, dir: f64, jmin: f64, jmax: f64, seg: usize) -> f64 {
    let jmin = jmin.max(1.0);
    if jmax <= jmin + 2.0 {
        return jmax.max(1.0);
    }
    let base = d.round().max(0.0) as usize;
    let (jlo, jhi) = (jmin.ceil() as usize, jmax.floor() as usize);
    let mut best = jhi;
    let mut best_s = f32::NEG_INFINITY;
    let mut j = jhi;
    loop {
        let s = corr_score(line, base, dir, j, seg, 4);
        if s > best_s {
            best_s = s;
            best = j;
        }
        if j < jlo + 2 {
            break;
        }
        j -= 2;
    }
    // full-resolution refinement around the coarse peak
    let lo = best.saturating_sub(2).max(jlo);
    let hi = (best + 2).min(jhi);
    let mut s = [f32::NEG_INFINITY; 5];
    let mut k_best = 0;
    for (i, jj) in (lo..=hi).enumerate() {
        s[i] = corr_score(line, base, dir, jj, seg, 1);
        if s[i] > s[k_best] {
            k_best = i;
        }
    }
    let n = hi - lo + 1;
    let mut frac = 0.0;
    if k_best > 0 && k_best + 1 < n {
        let (a, b, c) = (s[k_best - 1], s[k_best], s[k_best + 1]);
        let den = a - 2.0 * b + c;
        if den < 0.0 {
            frac = (0.5 * (a - c) / den).clamp(-0.5, 0.5) as f64;
        }
    }
    ((lo + k_best) as f64 + frac).clamp(jmin, jmax)
}

/// Splice policy of the PITCH / D-PITCH / DOUBLE voices: keep the active head inside `[lo, lo + win]`.
/// Call once per sample after pushing the input and before reading.
pub fn grain_step(sh: &mut Shifter, cfg: &ShiftCfg, line: &DelayLine, ratio: f64, lo: f64) {
    if sh.fading() {
        return;
    }
    let hi = lo + cfg.win;
    let d = sh.delay();
    let fade = cfg.fade as f64;
    if ratio == 1.0 {
        if (d - lo).abs() > 0.5 {
            sh.splice_to(lo, cfg.fade);
        }
        return;
    }
    if d < lo - 1.0 || d > hi + 1.0 {
        // window moved (DELAY / mode change): restart at the window edge the head travels away from
        let to = if ratio > 1.0 { lo + cfg.jmax } else { lo };
        sh.splice_to(to, cfg.fade);
        return;
    }
    if ratio > 1.0 {
        if d <= lo + fade * (ratio - 1.0) + 1.0 {
            let jmax = cfg.jmax.min(hi - d);
            let j = search_jump(line, d, 1.0, cfg.jmin.min(jmax - 3.0), jmax, cfg.seg);
            sh.splice_to(d + j, cfg.fade);
        }
    } else if d >= hi - fade * (1.0 - ratio) - 1.0 {
        let jmax = cfg.jmax.min(d - lo);
        let j = search_jump(line, d, -1.0, cfg.jmin.min(jmax - 3.0), jmax, cfg.seg);
        sh.splice_to(d - j, cfg.fade);
    }
}

/// Start delay of a voice for `ratio` (the window edge the head moves away from).
fn start_delay(cfg: &ShiftCfg, ratio: f64, lo: f64) -> f64 {
    if ratio > 1.0 {
        lo + cfg.jmax
    } else {
        lo
    }
}

/// Minimum base delay (Lagrange reads need `d >= 1`; one sample of headroom for the head motion).
const DMIN: f64 = 2.0;

/// PITCH / D-PITCH shifter geometry: 6 ms splice fade, jumps 12..24 ms (covers periods down to ~83 Hz),
/// 8 ms correlation segment.
pub fn pitch_cfg(fs: f64) -> ShiftCfg {
    ShiftCfg::new(fs, 6.0, 12.0, 24.0, 8.0, RATIO_MAX)
}

#[inline]
fn cascade(s: &mut [Svf; 2], x: f32) -> f32 {
    let y = s[0].process(x);
    s[1].process(y)
}

fn flush(it: &mut [Svf]) {
    for s in it {
        s.flush_denormals();
    }
}

// ---------------------------------------------------------------------------------------------------------
// PITCH
// ---------------------------------------------------------------------------------------------------------

/// PITCH schema slots.
pub mod pitch_slot {
    pub const SEMI: u16 = 1;
    pub const CENT: u16 = 2;
    pub const DLY: u16 = 3;
    pub const LC: u16 = 4;
    pub const HC: u16 = 5;
    pub const MIX: u16 = 6;
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PitchParams {
    pub semi: f32,
    pub cent: f32,
    pub dly_ms: f32,
    pub lc_hz: f32,
    pub hc_hz: f32,
    pub mix_pct: f32,
}

impl Default for PitchParams {
    /// Schema defaults.
    fn default() -> Self {
        Self {
            semi: 0.0,
            cent: 0.0,
            dly_ms: 20.0,
            lc_hz: 20.0,
            hc_hz: 20_000.0,
            mix_pct: 100.0,
        }
    }
}

/// STEREO PITCH (`PITCH`); see the module doc.
pub struct StereoPitch {
    p: PitchParams,
    dirty: bool,
    fresh: bool,
    fs: f64,
    cfg: ShiftCfg,
    lines: [DelayLine; 2],
    ana: DelayLine,
    sh: Shifter,
    ratio: f64,
    lo: f64,
    mix: Lag,
    lc: [Svf; 2],
    hc: [Svf; 2],
}

impl Default for StereoPitch {
    fn default() -> Self {
        Self::new()
    }
}

impl StereoPitch {
    pub fn new() -> Self {
        Self {
            p: PitchParams::default(),
            dirty: true,
            fresh: true,
            fs: 48_000.0,
            cfg: pitch_cfg(48_000.0),
            lines: [DelayLine::new(1), DelayLine::new(1)],
            ana: DelayLine::new(1),
            sh: Shifter::default(),
            ratio: 1.0,
            lo: DMIN,
            mix: Lag::new(1.0),
            lc: [Svf::default(); 2],
            hc: [Svf::default(); 2],
        }
    }

    pub fn params(&self) -> &PitchParams {
        &self.p
    }

    pub fn set_params(&mut self, p: &PitchParams) {
        self.p = *p;
        self.dirty = true;
    }

    /// Delay range (samples) of the shifted voice for the current settings: `[DELAY, DELAY + window]`.
    pub fn wet_delay_range(&self) -> (f64, f64) {
        (self.lo, self.lo + self.cfg.win)
    }

    /// Active head delay (samples).
    pub fn head_delay(&self) -> f64 {
        self.sh.delay()
    }

    fn update(&mut self) {
        self.dirty = false;
        let p = self.p;
        let fs = self.fs;
        self.ratio = ratio(p.semi.clamp(-12.0, 12.0), p.cent.clamp(-50.0, 50.0));
        self.lo = (p.dly_ms.clamp(0.0, 500.0) as f64 * 1e-3 * fs).max(DMIN);
        self.mix.t = p.mix_pct.clamp(0.0, 100.0) * 0.01;
        let lc = hz(p.lc_hz, 20.0, 400.0, fs);
        let hc = hz(p.hc_hz, 200.0, 20_000.0, fs);
        for k in 0..2 {
            self.lc[k].set(SvfMode::Highpass, fs, lc, Q_BW, 0.0);
            self.hc[k].set(SvfMode::Lowpass, fs, hc, Q_BW, 0.0);
        }
        if self.fresh {
            self.mix.snap();
            self.sh.reset(start_delay(&self.cfg, self.ratio, self.lo));
        }
    }
}

impl Model for StereoPitch {
    fn id(&self) -> &'static str {
        "PITCH"
    }

    fn prepare(&mut self, fs: f64, _max_block: usize) {
        self.fs = fs;
        self.cfg = pitch_cfg(fs);
        let n = (0.5 * fs).ceil() as usize + self.cfg.span() + 8;
        self.lines = [DelayLine::new(n), DelayLine::new(n)];
        self.ana = DelayLine::new(n);
        self.mix.set_tau(GLIDE_S, fs);
        self.reset();
    }

    fn process(&mut self, left: &mut [f32], right: &mut [f32], _sc: Option<(&[f32], &[f32])>) {
        if self.dirty {
            self.update();
        }
        let (r, lo) = (self.ratio, self.lo);
        for (xl, xr) in left.iter_mut().zip(right.iter_mut()) {
            let x = [*xl, *xr];
            self.lines[0].push(x[0]);
            self.lines[1].push(x[1]);
            self.ana.push(0.5 * (x[0] + x[1]));
            grain_step(&mut self.sh, &self.cfg, &self.ana, r, lo);
            let mut y = [self.sh.read(&self.lines[0]), self.sh.read(&self.lines[1])];
            self.sh.advance(r);
            for ((yk, lc), hc) in y.iter_mut().zip(&mut self.lc).zip(&mut self.hc) {
                *yk = hc.process(lc.process(*yk));
            }
            let m = self.mix.tick();
            *xl = x[0] + m * (y[0] - x[0]);
            *xr = x[1] + m * (y[1] - x[1]);
        }
        if !left.is_empty() {
            self.fresh = false;
        }
        flush(&mut self.lc);
        flush(&mut self.hc);
        self.mix.settle();
    }

    fn reset(&mut self) {
        self.fresh = true;
        self.update();
        for l in self.lines.iter_mut().chain(core::iter::once(&mut self.ana)) {
            l.reset();
        }
        for s in self.lc.iter_mut().chain(self.hc.iter_mut()) {
            s.reset();
        }
    }

    fn set_param(&mut self, slot: u16, v: ParamValue) {
        let x = v.as_f32();
        let p = &mut self.p;
        match slot {
            pitch_slot::SEMI => p.semi = x,
            pitch_slot::CENT => p.cent = x,
            pitch_slot::DLY => p.dly_ms = x,
            pitch_slot::LC => p.lc_hz = x,
            pitch_slot::HC => p.hc_hz = x,
            pitch_slot::MIX => p.mix_pct = x,
            _ => return,
        }
        self.dirty = true;
    }

    fn param(&self, slot: u16) -> Option<f32> {
        let p = &self.p;
        Some(match slot {
            pitch_slot::SEMI => p.semi,
            pitch_slot::CENT => p.cent,
            pitch_slot::DLY => p.dly_ms,
            pitch_slot::LC => p.lc_hz,
            pitch_slot::HC => p.hc_hz,
            pitch_slot::MIX => p.mix_pct,
            _ => return None,
        })
    }

    fn load_defaults(&mut self) {
        self.set_params(&PitchParams::default());
    }

    fn commit(&mut self) {
        if self.dirty {
            self.update();
        }
    }
}

// ---------------------------------------------------------------------------------------------------------
// D-PITCH
// ---------------------------------------------------------------------------------------------------------

/// D-PITCH schema slots.
pub mod dpitch_slot {
    pub const SEMI1: u16 = 1;
    pub const CENT1: u16 = 2;
    pub const DLY1: u16 = 3;
    pub const PAN1: u16 = 4;
    pub const LVL1: u16 = 5;
    pub const SEMI2: u16 = 6;
    pub const CENT2: u16 = 7;
    pub const DLY2: u16 = 8;
    pub const PAN2: u16 = 9;
    pub const LVL2: u16 = 10;
    pub const LC: u16 = 11;
    pub const HC: u16 = 12;
}

/// One D-PITCH voice.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VoiceParams {
    pub semi: f32,
    pub cent: f32,
    pub dly_ms: f32,
    /// -100..100.
    pub pan: f32,
    /// Fader dB (-144 = off .. +10).
    pub lvl_db: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DualPitchParams {
    pub v: [VoiceParams; 2],
    pub lc_hz: f32,
    pub hc_hz: f32,
}

impl Default for DualPitchParams {
    /// Schema defaults.
    fn default() -> Self {
        let v = |cent, pan| VoiceParams {
            semi: 0.0,
            cent,
            dly_ms: 0.0,
            pan,
            lvl_db: 0.0,
        };
        Self {
            v: [v(-10.0, -50.0), v(10.0, 50.0)],
            lc_hz: 20.0,
            hc_hz: 20_000.0,
        }
    }
}

/// DUAL PITCH (`D-PITCH`); see the module doc.
pub struct DualPitch {
    p: DualPitchParams,
    dirty: bool,
    fresh: bool,
    fs: f64,
    cfg: ShiftCfg,
    line: DelayLine,
    sh: [Shifter; 2],
    ratio: [f64; 2],
    lo: [f64; 2],
    gl: [Lag; 2],
    gr: [Lag; 2],
    lc: [Svf; 2],
    hc: [Svf; 2],
}

impl Default for DualPitch {
    fn default() -> Self {
        Self::new()
    }
}

impl DualPitch {
    pub fn new() -> Self {
        Self {
            p: DualPitchParams::default(),
            dirty: true,
            fresh: true,
            fs: 48_000.0,
            cfg: pitch_cfg(48_000.0),
            line: DelayLine::new(1),
            sh: [Shifter::default(); 2],
            ratio: [1.0; 2],
            lo: [DMIN; 2],
            gl: [Lag::new(0.0); 2],
            gr: [Lag::new(0.0); 2],
            lc: [Svf::default(); 2],
            hc: [Svf::default(); 2],
        }
    }

    pub fn params(&self) -> &DualPitchParams {
        &self.p
    }

    pub fn set_params(&mut self, p: &DualPitchParams) {
        self.p = *p;
        self.dirty = true;
    }

    fn update(&mut self) {
        self.dirty = false;
        let fs = self.fs;
        for k in 0..2 {
            let v = self.p.v[k];
            self.ratio[k] = ratio(v.semi.clamp(-12.0, 12.0), v.cent.clamp(-50.0, 50.0));
            self.lo[k] = (v.dly_ms.clamp(0.0, 500.0) as f64 * 1e-3 * fs).max(DMIN);
            let g = fader(v.lvl_db);
            let (l, r) = pan_gains(v.pan * 0.01);
            self.gl[k].t = g * l;
            self.gr[k].t = g * r;
        }
        let lc = hz(self.p.lc_hz, 20.0, 400.0, fs);
        let hc = hz(self.p.hc_hz, 200.0, 20_000.0, fs);
        for k in 0..2 {
            self.lc[k].set(SvfMode::Highpass, fs, lc, Q_BW, 0.0);
            self.hc[k].set(SvfMode::Lowpass, fs, hc, Q_BW, 0.0);
        }
        if self.fresh {
            for k in 0..2 {
                self.gl[k].snap();
                self.gr[k].snap();
                self.sh[k].reset(start_delay(&self.cfg, self.ratio[k], self.lo[k]));
            }
        }
    }
}

impl Model for DualPitch {
    fn id(&self) -> &'static str {
        "D-PITCH"
    }

    fn prepare(&mut self, fs: f64, _max_block: usize) {
        self.fs = fs;
        self.cfg = pitch_cfg(fs);
        self.line = DelayLine::new((0.5 * fs).ceil() as usize + self.cfg.span() + 8);
        for l in self.gl.iter_mut().chain(self.gr.iter_mut()) {
            l.set_tau(GLIDE_S, fs);
        }
        self.reset();
    }

    fn process(&mut self, left: &mut [f32], right: &mut [f32], _sc: Option<(&[f32], &[f32])>) {
        if self.dirty {
            self.update();
        }
        for (xl, xr) in left.iter_mut().zip(right.iter_mut()) {
            self.line.push(0.5 * (*xl + *xr));
            let mut y = [0.0f32; 2];
            for k in 0..2 {
                grain_step(
                    &mut self.sh[k],
                    &self.cfg,
                    &self.line,
                    self.ratio[k],
                    self.lo[k],
                );
                let v = self.sh[k].read(&self.line);
                self.sh[k].advance(self.ratio[k]);
                y[0] += self.gl[k].tick() * v;
                y[1] += self.gr[k].tick() * v;
            }
            *xl = self.hc[0].process(self.lc[0].process(y[0]));
            *xr = self.hc[1].process(self.lc[1].process(y[1]));
        }
        if !left.is_empty() {
            self.fresh = false;
        }
        flush(&mut self.lc);
        flush(&mut self.hc);
        for l in self.gl.iter_mut().chain(self.gr.iter_mut()) {
            l.settle();
        }
    }

    fn reset(&mut self) {
        self.fresh = true;
        self.update();
        self.line.reset();
        for s in self.lc.iter_mut().chain(self.hc.iter_mut()) {
            s.reset();
        }
    }

    fn set_param(&mut self, slot: u16, v: ParamValue) {
        let x = v.as_f32();
        let p = &mut self.p;
        match slot {
            1..=10 => {
                let vp = &mut p.v[((slot - 1) / 5) as usize];
                match (slot - 1) % 5 {
                    0 => vp.semi = x,
                    1 => vp.cent = x,
                    2 => vp.dly_ms = x,
                    3 => vp.pan = x,
                    _ => vp.lvl_db = x,
                }
            }
            dpitch_slot::LC => p.lc_hz = x,
            dpitch_slot::HC => p.hc_hz = x,
            _ => return,
        }
        self.dirty = true;
    }

    fn param(&self, slot: u16) -> Option<f32> {
        let p = &self.p;
        Some(match slot {
            1..=10 => {
                let vp = &p.v[((slot - 1) / 5) as usize];
                match (slot - 1) % 5 {
                    0 => vp.semi,
                    1 => vp.cent,
                    2 => vp.dly_ms,
                    3 => vp.pan,
                    _ => vp.lvl_db,
                }
            }
            dpitch_slot::LC => p.lc_hz,
            dpitch_slot::HC => p.hc_hz,
            _ => return None,
        })
    }

    fn load_defaults(&mut self) {
        self.set_params(&DualPitchParams::default());
    }

    fn commit(&mut self) {
        if self.dirty {
            self.update();
        }
    }
}

// ---------------------------------------------------------------------------------------------------------
// DOUBLE
// ---------------------------------------------------------------------------------------------------------

/// DOUBLE schema slots.
pub mod double_slot {
    pub const MODE: u16 = 1;
    pub const MIX: u16 = 2;
    pub const SPRD: u16 = 3;
}

/// One DOUBLE mode: voices, static detune (cents), base delays (ms), pitch wander depth (cents) and
/// rate (Hz), pan positions at full spread [I].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DoubleMode {
    pub voices: usize,
    pub detune_c: [f32; 4],
    pub delay_ms: [f32; 4],
    pub wander_c: f32,
    pub rate_hz: f32,
    pub pan: [f32; 4],
}

/// TIGHT / LOOSE / GROUP / DETUNE / THICK [I].
pub const DOUBLE_MODES: [DoubleMode; 5] = [
    DoubleMode {
        voices: 2,
        detune_c: [-3.0, 3.0, 0.0, 0.0],
        delay_ms: [6.0, 9.0, 0.0, 0.0],
        wander_c: 2.0,
        rate_hz: 0.7,
        pan: [-1.0, 1.0, 0.0, 0.0],
    },
    DoubleMode {
        voices: 2,
        detune_c: [-6.0, 6.0, 0.0, 0.0],
        delay_ms: [18.0, 30.0, 0.0, 0.0],
        wander_c: 6.0,
        rate_hz: 0.4,
        pan: [-1.0, 1.0, 0.0, 0.0],
    },
    DoubleMode {
        voices: 4,
        detune_c: [-5.0, 5.0, -11.0, 11.0],
        delay_ms: [14.0, 22.0, 31.0, 40.0],
        wander_c: 8.0,
        rate_hz: 0.5,
        pan: [-1.0, 1.0, -0.35, 0.35],
    },
    DoubleMode {
        voices: 2,
        detune_c: [-12.0, 12.0, 0.0, 0.0],
        delay_ms: [3.0, 3.0, 0.0, 0.0],
        wander_c: 0.0,
        rate_hz: 0.5,
        pan: [-1.0, 1.0, 0.0, 0.0],
    },
    DoubleMode {
        voices: 4,
        detune_c: [-4.0, 4.0, -9.0, 9.0],
        delay_ms: [8.0, 12.0, 17.0, 23.0],
        wander_c: 4.0,
        rate_hz: 0.8,
        pan: [-1.0, 1.0, -0.4, 0.4],
    },
];

const DOUBLE_MAX_DELAY_MS: f64 = 40.0;
/// Control-rate period for the doubler's wander / pan updates.
const DOUBLE_CR: u32 = 16;

/// Smoothed random wander: cosine interpolation between uniform random targets, one per cycle.
#[derive(Clone, Copy, Debug)]
struct Wander {
    phase: f64,
    a: f32,
    b: f32,
    seed: u32,
}

impl Wander {
    fn new(seed: u32) -> Self {
        Self {
            phase: 0.0,
            a: 0.0,
            b: 0.0,
            seed,
        }
    }

    fn rand(&mut self) -> f32 {
        let mut x = self.seed;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.seed = x;
        (x as f32 / u32::MAX as f32) * 2.0 - 1.0
    }

    /// Value in -1..1 after advancing `inc` cycles.
    fn tick(&mut self, inc: f64) -> f32 {
        self.phase += inc;
        if self.phase >= 1.0 {
            self.phase -= 1.0;
            self.a = self.b;
            self.b = self.rand();
        }
        let w = (0.5 - 0.5 * (PI * self.phase).cos()) as f32;
        self.a + w * (self.b - self.a)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DoubleParams {
    /// Mode index (TIGHT, LOOSE, GROUP, DETUNE, THICK).
    pub mode: f32,
    pub mix_pct: f32,
    pub spread_pct: f32,
}

impl Default for DoubleParams {
    /// Schema defaults (GROUP, 50 %, 50 %).
    fn default() -> Self {
        Self {
            mode: 2.0,
            mix_pct: 50.0,
            spread_pct: 50.0,
        }
    }
}

/// Seeds of the four voices' pitch/time wander generators (restored on `reset`).
const DOUBLE_WANDER_SEEDS: [u32; 4] = [0x9e37_79b9, 0x85eb_ca6b, 0xc2b2_ae35, 0x27d4_eb2f];

/// DOUBLE VOCAL (`DOUBLE`); see the module doc.
pub struct DoubleVocal {
    p: DoubleParams,
    dirty: bool,
    fresh: bool,
    fs: f64,
    cfg: ShiftCfg,
    mode: DoubleMode,
    line: DelayLine,
    sh: [Shifter; 4],
    wander: [Wander; 4],
    ratio: [f64; 4],
    lo: [f64; 4],
    gain: [(f32, f32); 4],
    ctr: u32,
    mix: Lag,
    spread: Lag,
}

impl Default for DoubleVocal {
    fn default() -> Self {
        Self::new()
    }
}

impl DoubleVocal {
    pub fn new() -> Self {
        Self {
            p: DoubleParams::default(),
            dirty: true,
            fresh: true,
            fs: 48_000.0,
            cfg: double_cfg(48_000.0),
            mode: DOUBLE_MODES[2],
            line: DelayLine::new(1),
            sh: [Shifter::default(); 4],
            wander: DOUBLE_WANDER_SEEDS.map(Wander::new),
            ratio: [1.0; 4],
            lo: [DMIN; 4],
            gain: [(0.0, 0.0); 4],
            ctr: 0,
            mix: Lag::new(0.5),
            spread: Lag::new(0.5),
        }
    }

    pub fn params(&self) -> &DoubleParams {
        &self.p
    }

    pub fn set_params(&mut self, p: &DoubleParams) {
        self.p = *p;
        self.dirty = true;
    }

    fn update(&mut self) {
        self.dirty = false;
        let idx = (self.p.mode.round().max(0.0) as usize).min(DOUBLE_MODES.len() - 1);
        self.mode = DOUBLE_MODES[idx];
        for k in 0..4 {
            self.lo[k] = (self.mode.delay_ms[k] as f64 * 1e-3 * self.fs).max(DMIN);
        }
        self.mix.t = self.p.mix_pct.clamp(0.0, 100.0) * 0.01;
        self.spread.t = self.p.spread_pct.clamp(0.0, 100.0) * 0.01;
        if self.fresh {
            self.mix.snap();
            self.spread.snap();
            self.control(0.0);
            for k in 0..4 {
                self.sh[k].reset(self.lo[k]);
            }
        }
    }

    /// Control-rate update of ratios and pan gains (`inc` = wander phase advance in cycles).
    fn control(&mut self, inc: f64) {
        let m = self.mode;
        let s = self.spread.v;
        let norm = 1.0 / (m.voices as f32 * 0.5).sqrt();
        for k in 0..4 {
            let w = self.wander[k].tick(inc * (1.0 + 0.13 * k as f64));
            let cents = m.detune_c[k] + m.wander_c * w;
            self.ratio[k] = 2f64.powf(cents as f64 / 1200.0);
            let (l, r) = pan_gains(m.pan[k] * s);
            self.gain[k] = if k < m.voices {
                (l * norm, r * norm)
            } else {
                (0.0, 0.0)
            };
        }
    }
}

/// DOUBLE shifter geometry: ratios stay within +/-20 cents, so splices are rare; 8 ms fade, jumps 8..16 ms.
pub fn double_cfg(fs: f64) -> ShiftCfg {
    ShiftCfg::new(fs, 8.0, 8.0, 16.0, 8.0, 1.02)
}

impl Model for DoubleVocal {
    fn id(&self) -> &'static str {
        "DOUBLE"
    }

    fn prepare(&mut self, fs: f64, _max_block: usize) {
        self.fs = fs;
        self.cfg = double_cfg(fs);
        self.line =
            DelayLine::new((DOUBLE_MAX_DELAY_MS * 1e-3 * fs).ceil() as usize + self.cfg.span() + 8);
        self.mix.set_tau(GLIDE_S, fs);
        self.spread.set_tau(GLIDE_S, fs);
        self.reset();
    }

    fn process(&mut self, left: &mut [f32], right: &mut [f32], _sc: Option<(&[f32], &[f32])>) {
        if self.dirty {
            self.update();
        }
        let inc = self.mode.rate_hz as f64 * DOUBLE_CR as f64 / self.fs;
        let nv = self.mode.voices;
        for (xl, xr) in left.iter_mut().zip(right.iter_mut()) {
            if self.ctr == 0 {
                self.control(inc);
                self.ctr = DOUBLE_CR;
            }
            self.ctr -= 1;
            let x = [*xl, *xr];
            self.line.push(0.5 * (x[0] + x[1]));
            let mut y = [0.0f32; 2];
            for k in 0..nv {
                grain_step(
                    &mut self.sh[k],
                    &self.cfg,
                    &self.line,
                    self.ratio[k],
                    self.lo[k],
                );
                let v = self.sh[k].read(&self.line);
                self.sh[k].advance(self.ratio[k]);
                y[0] += self.gain[k].0 * v;
                y[1] += self.gain[k].1 * v;
            }
            self.spread.tick();
            let m = self.mix.tick();
            *xl = x[0] + m * (y[0] - x[0]);
            *xr = x[1] + m * (y[1] - x[1]);
        }
        if !left.is_empty() {
            self.fresh = false;
        }
        self.mix.settle();
        self.spread.settle();
    }

    fn reset(&mut self) {
        self.fresh = true;
        self.ctr = 0;
        // T081: voices and their wander restart, so a reset makes the output repeatable
        self.sh = [Shifter::default(); 4];
        self.wander = DOUBLE_WANDER_SEEDS.map(Wander::new);
        self.update();
        self.line.reset();
    }

    fn set_param(&mut self, slot: u16, v: ParamValue) {
        let x = v.as_f32();
        match slot {
            double_slot::MODE => self.p.mode = x,
            double_slot::MIX => self.p.mix_pct = x,
            double_slot::SPRD => self.p.spread_pct = x,
            _ => return,
        }
        self.dirty = true;
    }

    fn param(&self, slot: u16) -> Option<f32> {
        Some(match slot {
            double_slot::MODE => self.p.mode,
            double_slot::MIX => self.p.mix_pct,
            double_slot::SPRD => self.p.spread_pct,
            _ => return None,
        })
    }

    fn load_defaults(&mut self) {
        self.set_params(&DoubleParams::default());
    }

    fn commit(&mut self) {
        if self.dirty {
            self.update();
        }
    }
}

// ---------------------------------------------------------------------------------------------------------
// PCORR
// ---------------------------------------------------------------------------------------------------------

/// PCORR schema slots (`sw_c` .. `sw_b` = slots 4..15).
pub mod pcorr_slot {
    pub const SPD: u16 = 1;
    pub const AMNT: u16 = 2;
    pub const A4: u16 = 3;
    pub const SW_C: u16 = 4;
    pub const SW_B: u16 = 15;
}

/// PCORR fixed processing latency (head centre delay).
pub const PCORR_LATENCY_MS: f64 = 7.5;
/// Detector range.
pub const PCORR_FMIN: f64 = 75.0;
pub const PCORR_FMAX: f64 = 1000.0;
/// Decimated detector rate (approx.).
const PCORR_FS_DET: f64 = 12_000.0;
const DBUF: usize = 512;
const HOP_D: usize = 32;
const YIN_THR: f32 = 0.15;
const YIN_FALLBACK: f32 = 0.3;
/// Below this mean square (about -60 dBFS RMS) the input counts as unvoiced.
const PCORR_GATE: f32 = 1e-6;
const SNAP_HYST: f64 = 0.15;
const RECENTRE_S: f64 = 0.05;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PcorrParams {
    pub speed: f32,
    pub amount: f32,
    pub a4_hz: f32,
    /// Note enables C, Db, D, Eb, E, F, Gb, G, Ab, A, Bb, B (0/1).
    pub sw: [f32; 12],
}

impl Default for PcorrParams {
    /// Schema defaults.
    fn default() -> Self {
        Self {
            speed: 50.0,
            amount: 25.0,
            a4_hz: 440.0,
            sw: [0.0; 12],
        }
    }
}

/// Retune time constant (ms) for SPEED 1..100.
pub fn pcorr_retune_ms(speed: f32) -> f64 {
    400f64.powf((100.0 - speed.clamp(1.0, 100.0) as f64) / 99.0)
}

/// PITCH FIX (`PCORR`); see the module doc.
pub struct PitchFix {
    p: PcorrParams,
    dirty: bool,
    fs: f64,
    lines: [DelayLine; 2],
    ana: DelayLine,
    sh: Shifter,
    centre: f64,
    fade: u32,
    seg: usize,
    // detector
    dec: usize,
    dec_ctr: usize,
    dec_lp: [Svf; 2],
    dbuf: [f32; DBUF],
    dpos: usize,
    hop: usize,
    scratch: [f32; DBUF],
    dfn: [f32; DBUF],
    tau_min: usize,
    tau_max: usize,
    nw: usize,
    nf: usize,
    // state
    period: f64,
    unvoiced: u32,
    note: Option<i32>,
    corr: f64,
    corr_t: f64,
    glide: f64,
    amount: f64,
    a4: f64,
    mask: [bool; 12],
    ratio: f64,
}

impl Default for PitchFix {
    fn default() -> Self {
        Self::new()
    }
}

impl PitchFix {
    pub fn new() -> Self {
        Self {
            p: PcorrParams::default(),
            dirty: true,
            fs: 48_000.0,
            lines: [DelayLine::new(1), DelayLine::new(1)],
            ana: DelayLine::new(1),
            sh: Shifter::default(),
            centre: 360.0,
            fade: 96,
            seg: 256,
            dec: 4,
            dec_ctr: 0,
            dec_lp: [Svf::default(); 2],
            dbuf: [0.0; DBUF],
            dpos: 0,
            hop: 0,
            scratch: [0.0; DBUF],
            dfn: [0.0; DBUF],
            tau_min: 12,
            tau_max: 160,
            nw: 192,
            nf: 512,
            period: 0.0,
            unvoiced: 0,
            note: None,
            corr: 0.0,
            corr_t: 0.0,
            glide: 1.0,
            amount: 0.25,
            a4: 440.0,
            mask: [true; 12],
            ratio: 1.0,
        }
    }

    pub fn params(&self) -> &PcorrParams {
        &self.p
    }

    pub fn set_params(&mut self, p: &PcorrParams) {
        self.p = *p;
        self.dirty = true;
    }

    /// Last detected input fundamental (Hz), `None` while unvoiced.
    pub fn detected_hz(&self) -> Option<f64> {
        (self.period > 0.0).then(|| self.fs / self.period)
    }

    /// Current applied correction in cents.
    pub fn correction_cents(&self) -> f64 {
        self.corr * 100.0
    }

    /// Current target MIDI note.
    pub fn target_note(&self) -> Option<i32> {
        self.note
    }

    fn update(&mut self) {
        self.dirty = false;
        let p = self.p;
        self.amount = (p.amount.clamp(0.0, 100.0) * 0.01) as f64;
        self.a4 = p.a4_hz.clamp(410.0, 470.0) as f64;
        let tau = pcorr_retune_ms(p.speed) * 1e-3;
        self.glide = 1.0 - (-1.0 / (tau * self.fs)).exp();
        let any = p.sw.iter().any(|&s| s >= 0.5);
        for k in 0..12 {
            self.mask[k] = !any || p.sw[k] >= 0.5;
        }
        if let Some(n) = self.note {
            if !self.mask[n.rem_euclid(12) as usize] {
                self.note = None;
            }
        }
    }

    /// Run the YIN detector on the decimated buffer (+ full-rate refinement); updates `period`.
    fn detect(&mut self) {
        let (nw, tmax, tmin) = (self.nw, self.tau_max, self.tau_min);
        let n = nw + tmax;
        // oldest first
        let start = (self.dpos + DBUF - n) % DBUF;
        for i in 0..n {
            self.scratch[i] = self.dbuf[(start + i) % DBUF];
        }
        let x = &self.scratch[..n];
        let ms = x[..nw].iter().map(|v| v * v).sum::<f32>() / nw as f32;
        if ms < PCORR_GATE {
            self.period = 0.0;
            return;
        }
        // difference function + CMNDF
        let mut run = 0.0f32;
        self.dfn[0] = 1.0;
        let mut found = 0usize;
        let mut min_t = 0usize;
        let mut min_v = f32::INFINITY;
        for tau in 1..=tmax {
            let mut s = 0.0f32;
            for j in 0..nw {
                let e = x[j] - x[j + tau];
                s += e * e;
            }
            run += s;
            self.dfn[tau] = if run > 0.0 { s * tau as f32 / run } else { 1.0 };
            if tau >= tmin && self.dfn[tau] < min_v {
                min_v = self.dfn[tau];
                min_t = tau;
            }
        }
        for tau in tmin..=tmax {
            if self.dfn[tau] < YIN_THR {
                let mut t = tau;
                while t < tmax && self.dfn[t + 1] < self.dfn[t] {
                    t += 1;
                }
                found = t;
                break;
            }
        }
        if found == 0 {
            if min_v < YIN_FALLBACK {
                found = min_t;
            } else {
                self.period = 0.0;
                return;
            }
        }
        // full-rate refinement around dec * tau
        let c = found * self.dec;
        let r = self.dec + 2;
        let lo = c.saturating_sub(r).max(2);
        let hi = c + r;
        let mut best = lo;
        let mut best_v = f32::INFINITY;
        let mut vals = [0.0f32; 24];
        for (i, tau) in (lo..=hi).enumerate().take(24) {
            let mut s = 0.0f32;
            for k in 0..self.nf {
                let e = self.ana.read_int(k) - self.ana.read_int(k + tau);
                s += e * e;
            }
            vals[i] = s;
            if s < best_v {
                best_v = s;
                best = tau;
            }
        }
        let i = best - lo;
        let mut frac = 0.0;
        if i > 0 && i < (hi - lo).min(23) {
            let (a, b, cc) = (vals[i - 1], vals[i], vals[i + 1]);
            let den = a - 2.0 * b + cc;
            if den > 0.0 {
                frac = (0.5 * (a - cc) / den).clamp(-0.5, 0.5) as f64;
            }
        }
        let per = best as f64 + frac;
        let pmax = self.fs / PCORR_FMIN;
        let pmin = self.fs / PCORR_FMAX;
        self.period = if (pmin..=pmax).contains(&per) {
            per
        } else {
            0.0
        };
    }

    /// Nearest enabled note to `m` (MIDI, fractional).
    fn nearest(&self, m: f64) -> i32 {
        let mut best = m.round() as i32;
        let mut bd = f64::INFINITY;
        let c = m.round() as i32;
        for n in c - 7..=c + 7 {
            if self.mask[n.rem_euclid(12) as usize] {
                let d = (m - n as f64).abs();
                if d < bd {
                    bd = d;
                    best = n;
                }
            }
        }
        best
    }

    fn retarget(&mut self) {
        if self.period <= 0.0 {
            self.corr_t = 0.0;
            return;
        }
        let f0 = self.fs / self.period;
        let m = 69.0 + 12.0 * (f0 / self.a4).log2();
        let cand = self.nearest(m);
        let keep = match self.note {
            Some(cur) => (m - cand as f64).abs() + SNAP_HYST >= (m - cur as f64).abs(),
            None => false,
        };
        if !keep {
            self.note = Some(cand);
        }
        let t = self.note.unwrap_or(cand) as f64;
        self.corr_t = self.amount * (t - m);
    }

    /// Head policy: while voiced, keep the head within `centre +/- K*P/2` (K = whole periods that fit the
    /// window) and splice by `K*P` across the centre; while unvoiced, return to the centre.
    fn head_step(&mut self) {
        if self.sh.fading() {
            return;
        }
        let d = self.sh.delay();
        let c = self.centre;
        let guard = self.fade as f64 * (1.0 - self.ratio).abs() + 2.0;
        let (lo, hi) = (DMIN + guard, 2.0 * c - DMIN - guard);
        if self.period > 0.0 {
            // swing symmetrically around the centre by K whole periods, so the mean delay is the latency
            let p = self.period;
            let kk = ((hi - lo) / p).floor().max(1.0);
            let b = 0.5 * kk * p;
            let off = d - c;
            if off.abs() > b {
                let k = ((off.abs() + b) / p).round().max(kk);
                let j0 = k * p;
                let dir = if off > 0.0 { -1.0 } else { 1.0 };
                let j = search_jump(&self.ana, d, dir, j0 - 3.0, j0 + 3.0, self.seg);
                let to = d + dir * j;
                let to = if (DMIN..=2.0 * c - DMIN).contains(&to) {
                    to
                } else {
                    c
                };
                self.sh.splice_to(to, self.fade);
            }
        } else if (self.unvoiced as f64 > RECENTRE_S * self.fs && (d - c).abs() > 0.5)
            || d < DMIN
            || d > 2.0 * c - DMIN
        {
            self.sh.splice_to(c, self.fade);
        }
    }
}

impl Model for PitchFix {
    fn id(&self) -> &'static str {
        "PCORR"
    }

    fn prepare(&mut self, fs: f64, _max_block: usize) {
        self.fs = fs;
        self.centre = (PCORR_LATENCY_MS * 1e-3 * fs).round();
        self.fade = (2.0e-3 * fs).round() as u32;
        self.seg = (5.0e-3 * fs).round() as usize;
        self.dec = ((fs / PCORR_FS_DET).round() as usize).max(1);
        let fsd = fs / self.dec as f64;
        self.tau_min = (fsd / PCORR_FMAX).floor() as usize;
        self.tau_max = ((fsd / PCORR_FMIN).ceil() as usize).min(DBUF / 2 - 8);
        self.nw = (self.tau_max + 32).min(DBUF - self.tau_max - 1);
        self.nf = (10.7e-3 * fs).round() as usize;
        let pmax = (fs / PCORR_FMIN).ceil() as usize;
        let line = (2.0 * self.centre) as usize + self.seg + pmax + 16;
        self.lines = [DelayLine::new(line), DelayLine::new(line)];
        self.ana = DelayLine::new(line.max(self.nf + pmax + self.dec + 8));
        for s in &mut self.dec_lp {
            s.set(SvfMode::Lowpass, fs, (fsd * 0.17).min(fs * 0.45), Q_BW, 0.0);
        }
        self.reset();
    }

    fn process(&mut self, left: &mut [f32], right: &mut [f32], _sc: Option<(&[f32], &[f32])>) {
        if self.dirty {
            self.update();
        }
        for (xl, xr) in left.iter_mut().zip(right.iter_mut()) {
            let x = [*xl, *xr];
            let m = 0.5 * (x[0] + x[1]);
            self.lines[0].push(x[0]);
            self.lines[1].push(x[1]);
            self.ana.push(m);
            let v = cascade(&mut self.dec_lp, m);
            self.dec_ctr += 1;
            if self.dec_ctr >= self.dec {
                self.dec_ctr = 0;
                self.dbuf[self.dpos] = v;
                self.dpos = (self.dpos + 1) % DBUF;
                self.hop += 1;
                if self.hop >= HOP_D {
                    self.hop = 0;
                    self.detect();
                    self.retarget();
                }
            }
            if self.period > 0.0 {
                self.unvoiced = 0;
            } else {
                self.unvoiced = self.unvoiced.saturating_add(1);
            }
            self.corr += self.glide * (self.corr_t - self.corr);
            self.ratio = if self.corr == 0.0 {
                1.0
            } else {
                (self.corr / 12.0).exp2()
            };
            self.head_step();
            *xl = self.sh.read(&self.lines[0]);
            *xr = self.sh.read(&self.lines[1]);
            self.sh.advance(self.ratio);
        }
        if self.corr.abs() < 1e-9 && self.corr_t == 0.0 {
            self.corr = 0.0;
        }
        flush(&mut self.dec_lp);
    }

    fn latency(&self) -> usize {
        self.centre as usize
    }

    fn reset(&mut self) {
        self.update();
        for l in self.lines.iter_mut().chain(core::iter::once(&mut self.ana)) {
            l.reset();
        }
        for s in &mut self.dec_lp {
            s.reset();
        }
        self.dbuf = [0.0; DBUF];
        self.dpos = 0;
        self.dec_ctr = 0;
        self.hop = 0;
        self.period = 0.0;
        self.unvoiced = 0;
        self.note = None;
        self.corr = 0.0;
        self.corr_t = 0.0;
        self.ratio = 1.0;
        self.sh.reset(self.centre);
    }

    fn set_param(&mut self, slot: u16, v: ParamValue) {
        let x = v.as_f32();
        match slot {
            pcorr_slot::SPD => self.p.speed = x,
            pcorr_slot::AMNT => self.p.amount = x,
            pcorr_slot::A4 => self.p.a4_hz = x,
            pcorr_slot::SW_C..=pcorr_slot::SW_B => {
                self.p.sw[(slot - pcorr_slot::SW_C) as usize] = x
            }
            _ => return,
        }
        self.dirty = true;
    }

    fn param(&self, slot: u16) -> Option<f32> {
        Some(match slot {
            pcorr_slot::SPD => self.p.speed,
            pcorr_slot::AMNT => self.p.amount,
            pcorr_slot::A4 => self.p.a4_hz,
            pcorr_slot::SW_C..=pcorr_slot::SW_B => self.p.sw[(slot - pcorr_slot::SW_C) as usize],
            _ => return None,
        })
    }

    fn load_defaults(&mut self) {
        self.set_params(&PcorrParams::default());
    }

    fn commit(&mut self) {
        if self.dirty {
            self.update();
        }
    }
}

// ---------------------------------------------------------------------------------------------------------
// SUB
// ---------------------------------------------------------------------------------------------------------

/// SUB schema slots.
pub mod sub_slot {
    pub const RNG: u16 = 1;
    pub const OCT1: u16 = 2;
    pub const OCT2: u16 = 3;
}

/// Tracking / output filter frequency for RANGE LOW / MID / HIGH [I].
pub const SUB_RANGE_HZ: [f64; 3] = [200.0, 400.0, 800.0];
const SUB_HP_HZ: f64 = 25.0;
const SUB_HYST: f32 = 0.1;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SubParams {
    /// RANGE index (LOW, MID, HIGH).
    pub range: f32,
    pub oct1_pct: f32,
    pub oct2_pct: f32,
}

impl Default for SubParams {
    /// Schema defaults.
    fn default() -> Self {
        Self {
            range: 0.0,
            oct1_pct: 50.0,
            oct2_pct: 0.0,
        }
    }
}

/// SUB OCTAVER (`SUB`); see the module doc.
pub struct SubOctaver {
    p: SubParams,
    dirty: bool,
    fresh: bool,
    fs: f64,
    hp: Svf,
    track: [Svf; 2],
    out1: [Svf; 2],
    out2: [Svf; 2],
    env: f32,
    env_att: f32,
    env_rel: f32,
    high: bool,
    sq1: f32,
    sq2: f32,
    g1: Lag,
    g2: Lag,
}

impl Default for SubOctaver {
    fn default() -> Self {
        Self::new()
    }
}

impl SubOctaver {
    pub fn new() -> Self {
        Self {
            p: SubParams::default(),
            dirty: true,
            fresh: true,
            fs: 48_000.0,
            hp: Svf::default(),
            track: [Svf::default(); 2],
            out1: [Svf::default(); 2],
            out2: [Svf::default(); 2],
            env: 0.0,
            env_att: 1.0,
            env_rel: 1.0,
            high: false,
            sq1: 1.0,
            sq2: 1.0,
            g1: Lag::new(0.0),
            g2: Lag::new(0.0),
        }
    }

    pub fn params(&self) -> &SubParams {
        &self.p
    }

    pub fn set_params(&mut self, p: &SubParams) {
        self.p = *p;
        self.dirty = true;
    }

    fn update(&mut self) {
        self.dirty = false;
        let fs = self.fs;
        let idx = (self.p.range.round().max(0.0) as usize).min(2);
        let f = SUB_RANGE_HZ[idx].min(fs * 0.45);
        self.hp.set(SvfMode::Highpass, fs, SUB_HP_HZ, Q_BW, 0.0);
        // 4th-order Butterworth: Q 0.541, 1.307
        for (s, q) in self.track.iter_mut().zip([0.541, 1.307]) {
            s.set(SvfMode::Lowpass, fs, f, q, 0.0);
        }
        for s in self.out1.iter_mut().chain(self.out2.iter_mut()) {
            s.set(SvfMode::Lowpass, fs, f, Q_BW, 0.0);
        }
        self.g1.t = self.p.oct1_pct.clamp(0.0, 100.0) * 0.01;
        self.g2.t = self.p.oct2_pct.clamp(0.0, 100.0) * 0.01;
        if self.fresh {
            self.g1.snap();
            self.g2.snap();
        }
    }
}

impl Model for SubOctaver {
    fn id(&self) -> &'static str {
        "SUB"
    }

    fn prepare(&mut self, fs: f64, _max_block: usize) {
        self.fs = fs;
        self.env_att = (1.0 - (-1.0 / (1e-3 * fs)).exp()) as f32;
        self.env_rel = (1.0 - (-1.0 / (0.08 * fs)).exp()) as f32;
        self.g1.set_tau(GLIDE_S, fs);
        self.g2.set_tau(GLIDE_S, fs);
        self.reset();
    }

    fn process(&mut self, left: &mut [f32], right: &mut [f32], _sc: Option<(&[f32], &[f32])>) {
        if self.dirty {
            self.update();
        }
        for (xl, xr) in left.iter_mut().zip(right.iter_mut()) {
            let m = 0.5 * (*xl + *xr);
            let f = cascade(&mut self.track, self.hp.process(m));
            let a = f.abs();
            let c = if a > self.env {
                self.env_att
            } else {
                self.env_rel
            };
            self.env += c * (a - self.env);
            let h = SUB_HYST * self.env + 1e-6;
            if !self.high && f > h {
                self.high = true;
                self.sq1 = -self.sq1;
                if self.sq1 > 0.0 {
                    self.sq2 = -self.sq2;
                }
            } else if self.high && f < -h {
                self.high = false;
            }
            let s1 = cascade(&mut self.out1, f * self.sq1);
            let s2 = cascade(&mut self.out2, f * self.sq2);
            let sub = self.g1.tick() * s1 + self.g2.tick() * s2;
            *xl += sub;
            *xr += sub;
        }
        if !left.is_empty() {
            self.fresh = false;
        }
        if self.env < 1e-12 {
            self.env = 0.0;
        }
        self.hp.flush_denormals();
        flush(&mut self.track);
        flush(&mut self.out1);
        flush(&mut self.out2);
        self.g1.settle();
        self.g2.settle();
    }

    fn reset(&mut self) {
        self.fresh = true;
        self.update();
        self.hp.reset();
        for s in self
            .track
            .iter_mut()
            .chain(self.out1.iter_mut())
            .chain(self.out2.iter_mut())
        {
            s.reset();
        }
        self.env = 0.0;
        self.high = false;
        self.sq1 = 1.0;
        self.sq2 = 1.0;
    }

    fn set_param(&mut self, slot: u16, v: ParamValue) {
        let x = v.as_f32();
        match slot {
            sub_slot::RNG => self.p.range = x,
            sub_slot::OCT1 => self.p.oct1_pct = x,
            sub_slot::OCT2 => self.p.oct2_pct = x,
            _ => return,
        }
        self.dirty = true;
    }

    fn param(&self, slot: u16) -> Option<f32> {
        Some(match slot {
            sub_slot::RNG => self.p.range,
            sub_slot::OCT1 => self.p.oct1_pct,
            sub_slot::OCT2 => self.p.oct2_pct,
            _ => return None,
        })
    }

    fn load_defaults(&mut self) {
        self.set_params(&SubParams::default());
    }

    fn commit(&mut self) {
        if self.dirty {
            self.update();
        }
    }
}

#[cfg(test)]
mod tests;
