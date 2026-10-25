//! FX rack modulation models (T05M, models.md §6): `DIMCRS` (DIMENSION CRS), `CHORUS` (STEREO CHORUS),
//! `FLANGER` (STEREO FLANGER), `PHASER`, `ROTARY` (ROTARY SPEAKER), `PANNER` (TREMOLO/PANNER) and `MOOD`
//! (MOOD FILTER).
//!
//! Like every FX rack model these process a stereo block in place; the FX slot host (wing-engine `FxSlot`)
//! blends the result with the dry signal by `fxmix`. Models with their own `MIX` (CHORUS, FLANGER, PHASER,
//! ROTARY, MOOD) or `DRY` switch (DIMCRS) apply it internally, as WING does; PANNER is a gain modulator and
//! has no mix.
//!
//! **Tempo sync:** the schema gives none of these models a tempo, sync or note-factor key (rates are free
//! `SPEED` values in Hz, and there is no global tempo/tap key, see wing-engine `fx.rs`), so all LFOs are
//! free-running at their Hz setting.
//!
//! Shared building blocks: [`Lfo`] (f64 phase accumulator, exact long-term rate), [`wave`] shapes,
//! [`AhrEnvelope`] (attack / hold / release follower for the PHASER, PANNER and MOOD envelope sections),
//! per-sample one-pole glides on every continuous parameter (no zipper/clicks on parameter changes), the
//! fractional delay lines (3rd-order Lagrange reads) and TPT SVFs of wing-dsp.
//!
//! Model notes (items marked [I] are inferred: no console measurement yet):
//! * **DIMCRS** - Roland SDD-320 Dimension D: two BBD delay lines (5 ms centre) swept in antiphase by one
//!   triangle LFO, BBD tone (10 kHz 2-pole anti-alias/reconstruction low-passes and a soft-saturating
//!   bucket stage) and the SDD-320 output matrix `L = dry + g(A - B)`, `R = dry + g(B - A)`: the wet
//!   signal cancels in a mono sum (the unit's famous mono compatibility). Buttons 1-3 run the LFO at
//!   0.25 Hz with increasing depth, button 4 at 0.5 Hz with the deepest sweep ([`DIM_BUTTONS`], [I]);
//!   combined buttons add their depths (capped) and take the fastest rate; no button = bypass (dry only).
//!   `IN MODE` MONO feeds `(L+R)/2` to both BBDs. `DRY` off removes the direct signal.
//! * **CHORUS** - one modulated Lagrange-read delay per side; delay `= DELAY * (1 + DEPTH/2 * lfo)` [I];
//!   `WAVEFORM` morphs triangle (0) to sine (100) [I]; `PHASE` = R LFO offset in degrees; `SPREAD` blends
//!   the wet sides from mono (0 %) to full stereo (100 %); LO/HI CUT (12 dB/oct) on the wet path.
//! * **FLANGER** - as CHORUS with a triangle LFO, delay `= DELAY * (1 + 0.9 * DEPTH * lfo)` [I] and a
//!   feedback path (`FEED` +/-90 %, FEED LC/HC 12 dB/oct filters in the loop).
//! * **PHASER** - 2..12 first-order allpass stages per side (one shared break frequency, swept on a log axis
//!   100 Hz..8 kHz: `RANGE` = sweep centre, `DEPTH` = LFO swing, `ENV MOD` = envelope shift, all in % of
//!   that axis [I]), resonance = feedback of the chain output (max 0.8). `WAVE` -50..50 skews the triangle
//!   LFO (-50 falling saw, 0 triangle, +50 rising saw) [I]. A stage-count change cross-fades between chain
//!   taps instead of switching. Coefficients are computed every 16 samples and interpolated per sample.
//! * **ROTARY** - Leslie 122/147: mono input, 800 Hz crossover into horn (treble) and drum (bass) rotors.
//!   Each rotor has its own speed with inertia (asymmetric spin-up / spin-down one-pole, time constants
//!   from `HORN/DRUM ACCEL`), Doppler from a delay read modulated by the rotor angle (horn radius 15 cm =
//!   +/-0.44 ms), amplitude modulation and, for the horn, a beaming low-pass that closes when the horn
//!   faces away. Two mics on opposite sides of the cabinet give the stereo image; `DISTANCE` reduces the
//!   AM depth and narrows the image [I]. Cabinet colour: horn driver roll-off (7 kHz), drum port bump
//!   (90 Hz). STOP / SLOW / FAST select 0 / `LO` / `HI` horn speed; the drum runs at 0.85x the horn
//!   setpoint (Leslie chorale/tremolo ratios). `BALANCE` -100 = drum only, +100 = horn only.
//! * **PANNER** - tremolo / auto-panner: gain `1 - DEPTH * (1 - u)` with a skewed-triangle LFO `u` in
//!   0..1 (`WAVE` as PHASER), R LFO offset by `PHASE` (0 = tremolo, +/-180 = auto-pan). The AHR envelope
//!   speeds up the LFO (`ENV>SPEED`, up to 4x at full envelope) and scales the depth (`ENV>DEPTH`: depth
//!   follows the envelope at 100 %) [I].
//! * **MOOD** - Mu-Tron style envelope/LFO filter: TPT SVF (12 dB) or two cascaded (24 dB; the second LP/HP
//!   stage is Butterworth-damped so the resonance is not squared), LP/HP/BP/NOTCH with cross-faded output taps, `RESO` 0..10 -> Q 0.5..20, `DRIVE` 0..10 -> up to +24 dB into a soft
//!   clipper (partially level-compensated). Cutoff = `BASE * 2^(ENV * 6 oct * env + LFO * 5 oct * w)` with
//!   a unipolar LFO shape `w` (TRI/SIN/SAW+/SAW-/RMP (= u^2)/SQU/RND = sample & hold per cycle) [I].
//!
//! The envelope sections normalise the detector to 1.0 at -6 dBFS ([`ENV_REF`], [I]).

use crate::model::{Model, ParamValue};
use core::f32::consts::{PI, TAU};
use wing_dsp::delay::DelayLine;
use wing_dsp::svf::{Svf, SvfMode};

/// Envelope level (linear peak) that counts as "full" envelope for ENV MOD / ENV>SPEED / ENV>DEPTH / ENV.
pub const ENV_REF: f32 = 0.5;
/// Control-rate period (samples) for filter coefficient updates (interpolated per sample).
const CR: u32 = 16;
/// Parameter glide time constant (mix, depth, gains, delay times).
const GLIDE_S: f64 = 0.02;
/// Delay-time glide (CHORUS / FLANGER base delay).
const DELAY_GLIDE_S: f64 = 0.05;

// ---------------------------------------------------------------------------------------------------------
// Shared building blocks
// ---------------------------------------------------------------------------------------------------------

#[inline]
fn wrap(x: f64) -> f64 {
    x - x.floor()
}

/// Bipolar LFO shapes of a phase `u` in `[0, 1)`, range -1..1.
pub mod wave {
    use core::f32::consts::TAU;

    /// Sine, 0 at `u = 0`.
    #[inline]
    pub fn sine(u: f32) -> f32 {
        (TAU * u).sin()
    }

    /// Triangle aligned with [`sine`] (0 at `u = 0`, +1 at `u = 1/4`).
    #[inline]
    pub fn tri(u: f32) -> f32 {
        let t = u + 0.25;
        let t = t - t.floor();
        1.0 - 4.0 * (t - 0.5).abs()
    }

    /// Skewed triangle: rises -1 -> 1 over `[0, duty)`, falls over `[duty, 1)`. `duty` 0.5 = triangle,
    /// near 0 = falling saw, near 1 = rising saw.
    #[inline]
    pub fn skew(u: f32, duty: f32) -> f32 {
        let d = duty.clamp(1e-3, 1.0 - 1e-3);
        let u = u.clamp(0.0, 1.0);
        if u < d {
            -1.0 + 2.0 * u / d
        } else {
            1.0 - 2.0 * (u - d) / (1.0 - d)
        }
    }
}

/// Free-running phase accumulator (phase in cycles, `[0, 1)`), f64 so the long-term rate is exact.
#[derive(Clone, Copy, Debug, Default)]
pub struct Lfo {
    phase: f64,
    inc: f64,
}

impl Lfo {
    pub fn set_rate(&mut self, hz: f64, fs: f64) {
        self.inc = (hz / fs).max(0.0);
    }

    /// Rate in cycles per sample.
    pub fn increment(&self) -> f64 {
        self.inc
    }

    pub fn phase(&self) -> f64 {
        self.phase
    }

    pub fn reset(&mut self) {
        self.phase = 0.0;
    }

    /// Return the current phase and advance by one sample.
    #[inline]
    pub fn tick(&mut self) -> f64 {
        let p = self.phase;
        self.phase += self.inc;
        if self.phase >= 1.0 {
            self.phase -= 1.0;
        }
        p
    }

    /// As [`Lfo::tick`] with a per-sample increment (cycles/sample), for rate-modulated LFOs.
    #[inline]
    pub fn tick_with(&mut self, inc: f64) -> f64 {
        self.inc = inc;
        self.tick()
    }
}

/// Attack / hold / release envelope follower on `|x|` (one-pole attack and release, hold timer restarted
/// whenever the input reaches the envelope).
#[derive(Clone, Copy, Debug)]
pub struct AhrEnvelope {
    env: f32,
    att: f32,
    rel: f32,
    hold: u32,
    ctr: u32,
}

impl Default for AhrEnvelope {
    fn default() -> Self {
        Self {
            env: 0.0,
            att: 1.0,
            rel: 1.0,
            hold: 0,
            ctr: 0,
        }
    }
}

impl AhrEnvelope {
    pub fn set_times(&mut self, att_ms: f32, hold_ms: f32, rel_ms: f32, fs: f64) {
        let c = |ms: f32| (1.0 - (-1.0 / (ms.max(0.01) as f64 * 1e-3 * fs)).exp()) as f32;
        self.att = c(att_ms);
        self.rel = c(rel_ms);
        self.hold = (hold_ms.max(0.0) as f64 * 1e-3 * fs) as u32;
    }

    pub fn reset(&mut self) {
        self.env = 0.0;
        self.ctr = 0;
    }

    #[inline]
    pub fn tick(&mut self, x: f32) -> f32 {
        let a = x.abs();
        if a >= self.env {
            self.env += self.att * (a - self.env);
            self.ctr = self.hold;
        } else if self.ctr > 0 {
            self.ctr -= 1;
        } else {
            self.env += self.rel * (a - self.env);
        }
        self.env
    }

    /// Raw envelope (linear peak).
    pub fn value(&self) -> f32 {
        self.env
    }

    /// Envelope normalised to [`ENV_REF`], clamped to 0..1.
    #[inline]
    pub fn normalized(&self) -> f32 {
        (self.env * (1.0 / ENV_REF)).min(1.0)
    }

    fn flush(&mut self) {
        if self.env < 1e-12 {
            self.env = 0.0;
        }
    }
}

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

    /// Land exactly on the target once within float noise (no denormal tails).
    fn settle(&mut self) {
        if (self.t - self.v).abs() < 1e-7 {
            self.v = self.t;
        }
    }
}

/// Cheap tanh-like soft clipper (Pade 3/2, clamped at +/-3 where it reaches +/-1).
#[inline]
fn soft_clip(x: f32) -> f32 {
    let x = x.clamp(-3.0, 3.0);
    x * (27.0 + x * x) / (27.0 + 9.0 * x * x)
}

fn svf(mode: SvfMode, fs: f64, f: f64, q: f64) -> Svf {
    Svf::new(mode, fs, f.clamp(10.0, fs * 0.45), q, 0.0)
}

const Q_BW: f64 = core::f64::consts::FRAC_1_SQRT_2;

fn flush_svfs<'a>(it: impl IntoIterator<Item = &'a mut Svf>) {
    for s in it {
        s.flush_denormals();
    }
}

/// Clamp a frequency parameter to its schema range and below 0.45 fs.
fn hz(v: f32, lo: f32, hi: f32, fs: f64) -> f64 {
    v.clamp(lo, hi).min((fs * 0.45) as f32) as f64
}

// ---------------------------------------------------------------------------------------------------------
// DIMCRS
// ---------------------------------------------------------------------------------------------------------

/// DIMCRS schema slots.
pub mod dim_slot {
    pub const SW1: u16 = 1;
    pub const SW2: u16 = 2;
    pub const SW3: u16 = 3;
    pub const SW4: u16 = 4;
    pub const IN: u16 = 5;
    pub const DRY: u16 = 6;
}

/// Per button: (LFO rate Hz, delay swing ms peak) [I].
pub const DIM_BUTTONS: [(f32, f32); 4] = [(0.25, 0.4), (0.25, 0.7), (0.25, 1.0), (0.5, 1.2)];
/// BBD centre delay.
pub const DIM_BASE_MS: f32 = 5.0;
/// Cap for combined buttons.
pub const DIM_MAX_SWING_MS: f32 = 2.5;
const DIM_BBD_HZ: f64 = 10_000.0;
const DIM_WET: f32 = 0.5;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DimensionParams {
    pub sw: [bool; 4],
    /// `IN MODE` STEREO (true) / MONO.
    pub stereo_in: bool,
    pub dry: bool,
}

impl Default for DimensionParams {
    /// Schema defaults: SW1 + SW3 on, STEREO, DRY on.
    fn default() -> Self {
        Self {
            sw: [true, false, true, false],
            stereo_in: true,
            dry: true,
        }
    }
}

impl DimensionParams {
    /// Effective (rate Hz, swing ms) of the pressed buttons, `None` when no button is pressed (bypass).
    pub fn modulation(&self) -> Option<(f32, f32)> {
        let mut rate = 0.0f32;
        let mut swing = 0.0f32;
        let mut any = false;
        for (on, (r, s)) in self.sw.iter().zip(DIM_BUTTONS) {
            if *on {
                any = true;
                rate = rate.max(r);
                swing += s;
            }
        }
        any.then_some((rate, swing.min(DIM_MAX_SWING_MS)))
    }
}

/// DIMENSION CRS (`DIMCRS`).
pub struct DimensionChorus {
    p: DimensionParams,
    dirty: bool,
    fresh: bool,
    fs: f64,
    lines: [DelayLine; 2],
    lfo: Lfo,
    base: f32,
    swing: Lag,
    wet: Lag,
    dry: Lag,
    pre: [Svf; 2],
    post: [Svf; 2],
}

impl Default for DimensionChorus {
    fn default() -> Self {
        Self::new()
    }
}

impl DimensionChorus {
    pub fn new() -> Self {
        Self {
            p: DimensionParams::default(),
            dirty: true,
            fresh: true,
            fs: 48_000.0,
            lines: [DelayLine::new(1), DelayLine::new(1)],
            lfo: Lfo::default(),
            base: 240.0,
            swing: Lag::new(0.0),
            wet: Lag::new(0.0),
            dry: Lag::new(1.0),
            pre: [Svf::default(); 2],
            post: [Svf::default(); 2],
        }
    }

    pub fn params(&self) -> &DimensionParams {
        &self.p
    }

    pub fn set_params(&mut self, p: &DimensionParams) {
        self.p = *p;
        self.dirty = true;
    }

    /// Current LFO rate (Hz).
    pub fn lfo_rate(&self) -> f64 {
        self.lfo.increment() * self.fs
    }

    fn update(&mut self) {
        self.dirty = false;
        let fs = self.fs;
        let ms = (fs * 1e-3) as f32;
        self.base = DIM_BASE_MS * ms;
        match self.p.modulation() {
            Some((rate, swing)) => {
                self.lfo.set_rate(rate as f64, fs);
                self.swing.t = swing * ms;
                self.wet.t = 1.0;
            }
            None => {
                self.swing.t = 0.0;
                self.wet.t = 0.0;
            }
        }
        self.dry.t = if self.p.dry { 1.0 } else { 0.0 };
        if self.fresh {
            for l in [&mut self.swing, &mut self.wet, &mut self.dry] {
                l.snap();
            }
        }
    }
}

impl Model for DimensionChorus {
    fn id(&self) -> &'static str {
        "DIMCRS"
    }

    fn prepare(&mut self, fs: f64, _max_block: usize) {
        self.fs = fs;
        let n = ((DIM_BASE_MS + DIM_MAX_SWING_MS) as f64 * 1e-3 * fs).ceil() as usize + 4;
        self.lines = [DelayLine::new(n), DelayLine::new(n)];
        for l in [&mut self.swing, &mut self.wet, &mut self.dry] {
            l.set_tau(0.05, fs);
        }
        let lp = svf(SvfMode::Lowpass, fs, DIM_BBD_HZ, Q_BW);
        self.pre = [lp; 2];
        self.post = [lp; 2];
        self.dirty = true;
        self.reset();
    }

    fn process(&mut self, left: &mut [f32], right: &mut [f32], _sc: Option<(&[f32], &[f32])>) {
        if self.dirty {
            self.update();
        }
        let stereo = self.p.stereo_in;
        let base = self.base;
        for (xl, xr) in left.iter_mut().zip(right.iter_mut()) {
            let (il, ir) = if stereo {
                (*xl, *xr)
            } else {
                let m = 0.5 * (*xl + *xr);
                (m, m)
            };
            self.lines[0].push(soft_clip(self.pre[0].process(il)));
            self.lines[1].push(soft_clip(self.pre[1].process(ir)));
            let tri = wave::tri(self.lfo.tick() as f32);
            let s = self.swing.tick() * tri;
            let a = self.post[0].process(self.lines[0].read_lagrange3(base + s));
            let b = self.post[1].process(self.lines[1].read_lagrange3(base - s));
            let w = self.wet.tick() * DIM_WET * (a - b);
            let d = self.dry.tick();
            *xl = d * *xl + w;
            *xr = d * *xr - w;
        }
        if !left.is_empty() {
            self.fresh = false;
        }
        flush_svfs(self.pre.iter_mut().chain(self.post.iter_mut()));
        for l in [&mut self.swing, &mut self.wet, &mut self.dry] {
            l.settle();
        }
    }

    fn reset(&mut self) {
        self.fresh = true;
        self.dirty = true;
        self.update();
        for l in &mut self.lines {
            l.reset();
        }
        self.lfo.reset();
        for s in self.pre.iter_mut().chain(self.post.iter_mut()) {
            s.reset();
        }
    }

    fn set_param(&mut self, slot: u16, v: ParamValue) {
        let p = &mut self.p;
        match slot {
            dim_slot::SW1..=dim_slot::SW4 => p.sw[(slot - 1) as usize] = v.as_bool(),
            dim_slot::IN => p.stereo_in = v.as_index() != 0,
            dim_slot::DRY => p.dry = v.as_bool(),
            _ => return,
        }
        self.dirty = true;
    }

    fn param(&self, slot: u16) -> Option<f32> {
        let b = |x: bool| if x { 1.0 } else { 0.0 };
        Some(match slot {
            dim_slot::SW1..=dim_slot::SW4 => b(self.p.sw[(slot - 1) as usize]),
            dim_slot::IN => b(self.p.stereo_in),
            dim_slot::DRY => b(self.p.dry),
            _ => return None,
        })
    }

    fn load_defaults(&mut self) {
        self.set_params(&DimensionParams::default());
    }

    fn commit(&mut self) {
        if self.dirty {
            self.update();
        }
    }
}

// ---------------------------------------------------------------------------------------------------------
// CHORUS / FLANGER (shared modulated-delay core)
// ---------------------------------------------------------------------------------------------------------

/// CHORUS schema slots.
pub mod chorus_slot {
    pub const LC: u16 = 1;
    pub const HC: u16 = 2;
    pub const WAVE: u16 = 3;
    pub const PHASE: u16 = 4;
    pub const MIX: u16 = 5;
    pub const DLYL: u16 = 6;
    pub const DLYR: u16 = 7;
    pub const DEPL: u16 = 8;
    pub const DEPR: u16 = 9;
    pub const SPRD: u16 = 10;
    pub const SPD: u16 = 11;
}

/// FLANGER schema slots.
pub mod flanger_slot {
    pub const LC: u16 = 1;
    pub const HC: u16 = 2;
    pub const FLC: u16 = 3;
    pub const FHC: u16 = 4;
    pub const MIX: u16 = 5;
    pub const DLYL: u16 = 6;
    pub const DLYR: u16 = 7;
    pub const DEPL: u16 = 8;
    pub const DEPR: u16 = 9;
    pub const PHASE: u16 = 10;
    pub const SPD: u16 = 11;
    pub const FEED: u16 = 12;
}

/// Parameters of CHORUS and FLANGER (fields a model lacks keep their defaults and are ignored).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ModDelayParams {
    pub lc_hz: f32,
    pub hc_hz: f32,
    /// CHORUS only: 0 = triangle .. 100 = sine.
    pub wave: f32,
    pub phase_deg: f32,
    pub mix_pct: f32,
    pub dly_ms: [f32; 2],
    pub depth_pct: [f32; 2],
    /// CHORUS only.
    pub spread_pct: f32,
    pub speed_hz: f32,
    /// FLANGER only.
    pub flc_hz: f32,
    pub fhc_hz: f32,
    pub feed_pct: f32,
}

impl ModDelayParams {
    /// CHORUS schema defaults.
    pub fn chorus() -> Self {
        Self {
            lc_hz: 75.0,
            hc_hz: 11_500.0,
            wave: 100.0,
            phase_deg: 120.0,
            mix_pct: 50.0,
            dly_ms: [15.2, 16.6],
            depth_pct: [20.0, 20.0],
            spread_pct: 100.0,
            speed_hz: 0.48,
            flc_hz: 20.0,
            fhc_hz: 20_000.0,
            feed_pct: 0.0,
        }
    }

    /// FLANGER schema defaults.
    pub fn flanger() -> Self {
        Self {
            lc_hz: 79.0,
            hc_hz: 15_200.0,
            wave: 0.0,
            phase_deg: 180.0,
            mix_pct: 50.0,
            dly_ms: [4.2, 4.2],
            depth_pct: [80.0, 80.0],
            spread_pct: 100.0,
            speed_hz: 0.32,
            flc_hz: 121.0,
            fhc_hz: 11_500.0,
            feed_pct: 70.0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ModDelayKind {
    Chorus,
    Flanger,
}

/// Flanger sweep: delay = base * (1 + FLANGE_SWING * depth * tri) [I].
pub const FLANGE_SWING: f32 = 0.9;
const MAX_FEED: f32 = 0.9;

/// STEREO CHORUS (`CHORUS`) and STEREO FLANGER (`FLANGER`); see the module doc.
pub struct ModDelay {
    kind: ModDelayKind,
    p: ModDelayParams,
    dirty: bool,
    fresh: bool,
    fs: f64,
    lines: [DelayLine; 2],
    lfo: Lfo,
    phase_off: f64,
    base: [Lag; 2],
    depth: [Lag; 2],
    mix: Lag,
    spread: Lag,
    wave: Lag,
    feed: Lag,
    lc: [Svf; 2],
    hc: [Svf; 2],
    flc: [Svf; 2],
    fhc: [Svf; 2],
    last_delay: [f32; 2],
}

impl ModDelay {
    pub fn chorus() -> Self {
        Self::with(ModDelayKind::Chorus, ModDelayParams::chorus())
    }

    pub fn flanger() -> Self {
        Self::with(ModDelayKind::Flanger, ModDelayParams::flanger())
    }

    fn with(kind: ModDelayKind, p: ModDelayParams) -> Self {
        Self {
            kind,
            p,
            dirty: true,
            fresh: true,
            fs: 48_000.0,
            lines: [DelayLine::new(1), DelayLine::new(1)],
            lfo: Lfo::default(),
            phase_off: 0.0,
            base: [Lag::new(100.0); 2],
            depth: [Lag::new(0.0); 2],
            mix: Lag::new(0.5),
            spread: Lag::new(1.0),
            wave: Lag::new(1.0),
            feed: Lag::new(0.0),
            lc: [Svf::default(); 2],
            hc: [Svf::default(); 2],
            flc: [Svf::default(); 2],
            fhc: [Svf::default(); 2],
            last_delay: [0.0; 2],
        }
    }

    fn defaults(&self) -> ModDelayParams {
        match self.kind {
            ModDelayKind::Chorus => ModDelayParams::chorus(),
            ModDelayKind::Flanger => ModDelayParams::flanger(),
        }
    }

    pub fn params(&self) -> &ModDelayParams {
        &self.p
    }

    pub fn set_params(&mut self, p: &ModDelayParams) {
        self.p = *p;
        self.dirty = true;
    }

    /// Delays (samples, L/R) used for the last processed sample.
    pub fn current_delays(&self) -> (f32, f32) {
        (self.last_delay[0], self.last_delay[1])
    }

    fn max_ms(&self) -> f64 {
        match self.kind {
            ModDelayKind::Chorus => 50.0 * 1.5,
            ModDelayKind::Flanger => 20.0 * (1.0 + FLANGE_SWING as f64),
        }
    }

    fn update(&mut self) {
        self.dirty = false;
        let fs = self.fs;
        let p = self.p;
        let ms = (fs * 1e-3) as f32;
        let (dlo, dhi) = match self.kind {
            ModDelayKind::Chorus => (5.0, 50.0),
            ModDelayKind::Flanger => (0.5, 20.0),
        };
        for k in 0..2 {
            self.base[k].t = p.dly_ms[k].clamp(dlo, dhi) * ms;
            self.depth[k].t = p.depth_pct[k].clamp(0.0, 100.0) * 0.01;
        }
        self.lfo.set_rate(p.speed_hz.clamp(0.05, 5.0) as f64, fs);
        self.phase_off = (p.phase_deg.clamp(0.0, 180.0) / 360.0) as f64;
        self.mix.t = p.mix_pct.clamp(0.0, 100.0) * 0.01;
        self.spread.t = p.spread_pct.clamp(0.0, 100.0) * 0.01;
        self.wave.t = p.wave.clamp(0.0, 100.0) * 0.01;
        self.feed.t = (p.feed_pct.clamp(-90.0, 90.0) * 0.01).clamp(-MAX_FEED, MAX_FEED);
        let lc = hz(p.lc_hz, 20.0, 400.0, fs);
        let hc = hz(p.hc_hz, 200.0, 20_000.0, fs);
        let flc = hz(p.flc_hz, 20.0, 400.0, fs);
        let fhc = hz(p.fhc_hz, 200.0, 20_000.0, fs);
        for k in 0..2 {
            self.lc[k].set(SvfMode::Highpass, fs, lc, Q_BW, 0.0);
            self.hc[k].set(SvfMode::Lowpass, fs, hc, Q_BW, 0.0);
            self.flc[k].set(SvfMode::Highpass, fs, flc, Q_BW, 0.0);
            self.fhc[k].set(SvfMode::Lowpass, fs, fhc, Q_BW, 0.0);
        }
        if self.fresh {
            self.snap();
        }
    }

    fn for_lags(&mut self, f: impl Fn(&mut Lag)) {
        for l in self.base.iter_mut().chain(self.depth.iter_mut()) {
            f(l);
        }
        for l in [
            &mut self.mix,
            &mut self.spread,
            &mut self.wave,
            &mut self.feed,
        ] {
            f(l);
        }
    }

    fn snap(&mut self) {
        self.for_lags(Lag::snap);
    }
}

impl Model for ModDelay {
    fn id(&self) -> &'static str {
        match self.kind {
            ModDelayKind::Chorus => "CHORUS",
            ModDelayKind::Flanger => "FLANGER",
        }
    }

    fn prepare(&mut self, fs: f64, _max_block: usize) {
        self.fs = fs;
        let n = (self.max_ms() * 1e-3 * fs).ceil() as usize + 4;
        self.lines = [DelayLine::new(n), DelayLine::new(n)];
        self.for_lags(|l| l.set_tau(GLIDE_S, fs));
        for b in &mut self.base {
            b.set_tau(DELAY_GLIDE_S, fs);
        }
        self.dirty = true;
        self.reset();
    }

    fn process(&mut self, left: &mut [f32], right: &mut [f32], _sc: Option<(&[f32], &[f32])>) {
        if self.dirty {
            self.update();
        }
        let chorus = self.kind == ModDelayKind::Chorus;
        let max_d = self.lines[0].max_delay() as f32 - 2.0;
        for (xl, xr) in left.iter_mut().zip(right.iter_mut()) {
            let u = self.lfo.tick();
            let ul = u as f32;
            let ur = wrap(u + self.phase_off) as f32;
            let x = [*xl, *xr];
            let mut y = [0.0f32; 2];
            if chorus {
                let w = self.wave.tick();
                let shape = |u: f32| wave::tri(u) + w * (wave::sine(u) - wave::tri(u));
                let m = [shape(ul), shape(ur)];
                for k in 0..2 {
                    let base = self.base[k].tick();
                    let dep = self.depth[k].tick();
                    // push first, read at d >= 1 (Lagrange)
                    let d = (base * (1.0 + 0.5 * dep * m[k])).clamp(1.0, max_d);
                    self.last_delay[k] = d;
                    self.lines[k].push(x[k]);
                    y[k] = self.lines[k].read_lagrange3(d);
                }
                let s = self.spread.tick();
                let (a, b) = (0.5 + 0.5 * s, 0.5 - 0.5 * s);
                y = [a * y[0] + b * y[1], a * y[1] + b * y[0]];
            } else {
                let m = [wave::tri(ul), wave::tri(ur)];
                let g = self.feed.tick();
                for k in 0..2 {
                    let base = self.base[k].tick();
                    let dep = self.depth[k].tick();
                    // read before the push: delay d = read position d - 1 (Lagrange needs >= 1)
                    let d = (base * (1.0 + FLANGE_SWING * dep * m[k])).clamp(2.0, max_d);
                    self.last_delay[k] = d;
                    let v = self.lines[k].read_lagrange3(d - 1.0);
                    let fb = self.fhc[k].process(self.flc[k].process(v));
                    self.lines[k].push(x[k] + g * fb);
                    y[k] = v;
                }
            }
            let mix = self.mix.tick();
            for ((yk, lc), hc) in y.iter_mut().zip(&mut self.lc).zip(&mut self.hc) {
                *yk = hc.process(lc.process(*yk));
            }
            *xl = x[0] + mix * (y[0] - x[0]);
            *xr = x[1] + mix * (y[1] - x[1]);
        }
        if !left.is_empty() {
            self.fresh = false;
        }
        flush_svfs(
            self.lc
                .iter_mut()
                .chain(self.hc.iter_mut())
                .chain(self.flc.iter_mut())
                .chain(self.fhc.iter_mut()),
        );
        self.for_lags(Lag::settle);
    }

    fn reset(&mut self) {
        self.fresh = true;
        self.dirty = true;
        self.update();
        for l in &mut self.lines {
            l.reset();
        }
        self.lfo.reset();
        for s in self
            .lc
            .iter_mut()
            .chain(self.hc.iter_mut())
            .chain(self.flc.iter_mut())
            .chain(self.fhc.iter_mut())
        {
            s.reset();
        }
    }

    fn set_param(&mut self, slot: u16, v: ParamValue) {
        let p = &mut self.p;
        let x = v.as_f32();
        match self.kind {
            ModDelayKind::Chorus => match slot {
                chorus_slot::LC => p.lc_hz = x,
                chorus_slot::HC => p.hc_hz = x,
                chorus_slot::WAVE => p.wave = x,
                chorus_slot::PHASE => p.phase_deg = x,
                chorus_slot::MIX => p.mix_pct = x,
                chorus_slot::DLYL => p.dly_ms[0] = x,
                chorus_slot::DLYR => p.dly_ms[1] = x,
                chorus_slot::DEPL => p.depth_pct[0] = x,
                chorus_slot::DEPR => p.depth_pct[1] = x,
                chorus_slot::SPRD => p.spread_pct = x,
                chorus_slot::SPD => p.speed_hz = x,
                _ => return,
            },
            ModDelayKind::Flanger => match slot {
                flanger_slot::LC => p.lc_hz = x,
                flanger_slot::HC => p.hc_hz = x,
                flanger_slot::FLC => p.flc_hz = x,
                flanger_slot::FHC => p.fhc_hz = x,
                flanger_slot::MIX => p.mix_pct = x,
                flanger_slot::DLYL => p.dly_ms[0] = x,
                flanger_slot::DLYR => p.dly_ms[1] = x,
                flanger_slot::DEPL => p.depth_pct[0] = x,
                flanger_slot::DEPR => p.depth_pct[1] = x,
                flanger_slot::PHASE => p.phase_deg = x,
                flanger_slot::SPD => p.speed_hz = x,
                flanger_slot::FEED => p.feed_pct = x,
                _ => return,
            },
        }
        self.dirty = true;
    }

    fn param(&self, slot: u16) -> Option<f32> {
        let p = &self.p;
        Some(match self.kind {
            ModDelayKind::Chorus => match slot {
                chorus_slot::LC => p.lc_hz,
                chorus_slot::HC => p.hc_hz,
                chorus_slot::WAVE => p.wave,
                chorus_slot::PHASE => p.phase_deg,
                chorus_slot::MIX => p.mix_pct,
                chorus_slot::DLYL => p.dly_ms[0],
                chorus_slot::DLYR => p.dly_ms[1],
                chorus_slot::DEPL => p.depth_pct[0],
                chorus_slot::DEPR => p.depth_pct[1],
                chorus_slot::SPRD => p.spread_pct,
                chorus_slot::SPD => p.speed_hz,
                _ => return None,
            },
            ModDelayKind::Flanger => match slot {
                flanger_slot::LC => p.lc_hz,
                flanger_slot::HC => p.hc_hz,
                flanger_slot::FLC => p.flc_hz,
                flanger_slot::FHC => p.fhc_hz,
                flanger_slot::MIX => p.mix_pct,
                flanger_slot::DLYL => p.dly_ms[0],
                flanger_slot::DLYR => p.dly_ms[1],
                flanger_slot::DEPL => p.depth_pct[0],
                flanger_slot::DEPR => p.depth_pct[1],
                flanger_slot::PHASE => p.phase_deg,
                flanger_slot::SPD => p.speed_hz,
                flanger_slot::FEED => p.feed_pct,
                _ => return None,
            },
        })
    }

    fn load_defaults(&mut self) {
        let d = self.defaults();
        self.set_params(&d);
    }

    fn commit(&mut self) {
        if self.dirty {
            self.update();
        }
    }
}

// ---------------------------------------------------------------------------------------------------------
// PHASER
// ---------------------------------------------------------------------------------------------------------

/// PHASER schema slots.
pub mod phaser_slot {
    pub const SPD: u16 = 1;
    pub const PHASE: u16 = 2;
    pub const WAVE: u16 = 3;
    pub const RANGE: u16 = 4;
    pub const DEPTH: u16 = 5;
    pub const EMOD: u16 = 6;
    pub const ATT: u16 = 7;
    pub const HLD: u16 = 8;
    pub const REL: u16 = 9;
    pub const MIX: u16 = 10;
    pub const STG: u16 = 11;
    pub const RESO: u16 = 12;
}

/// Sweep axis of the allpass break frequency (log) [I].
pub const PHASER_F_LO: f32 = 100.0;
pub const PHASER_F_HI: f32 = 8000.0;
pub const PHASER_MAX_STAGES: usize = 12;
const PHASER_MAX_RESO: f32 = 0.8;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PhaserParams {
    pub speed_hz: f32,
    pub phase_deg: f32,
    pub wave: f32,
    pub range_pct: f32,
    pub depth_pct: f32,
    pub emod_pct: f32,
    pub att_ms: f32,
    pub hld_ms: f32,
    pub rel_ms: f32,
    pub mix_pct: f32,
    pub stages: u32,
    pub reso_pct: f32,
}

impl Default for PhaserParams {
    fn default() -> Self {
        Self {
            speed_hz: 0.83,
            phase_deg: 120.0,
            wave: -25.0,
            range_pct: 20.0,
            depth_pct: 50.0,
            emod_pct: 0.0,
            att_ms: 20.0,
            hld_ms: 50.0,
            rel_ms: 200.0,
            mix_pct: 50.0,
            stages: 6,
            reso_pct: 20.0,
        }
    }
}

/// Break frequency (Hz) of sweep position `pos` (0..1 on the log axis).
pub fn phaser_freq(pos: f32) -> f32 {
    PHASER_F_LO * (PHASER_F_HI / PHASER_F_LO).powf(pos.clamp(0.0, 1.0))
}

/// PHASER (`PHASER`).
pub struct Phaser {
    p: PhaserParams,
    dirty: bool,
    fresh: bool,
    fs: f64,
    lfo: Lfo,
    phase_off: f64,
    duty: f32,
    env: AhrEnvelope,
    z: [[f32; PHASER_MAX_STAGES]; 2],
    fb: [f32; 2],
    a: [f32; 2],
    a_step: [f32; 2],
    ctr: u32,
    range: Lag,
    depth: Lag,
    emod: Lag,
    mix: Lag,
    reso: Lag,
    stage: Lag,
    primed: bool,
    last_freq: [f32; 2],
}

impl Default for Phaser {
    fn default() -> Self {
        Self::new()
    }
}

impl Phaser {
    pub fn new() -> Self {
        Self {
            p: PhaserParams::default(),
            dirty: true,
            fresh: true,
            fs: 48_000.0,
            lfo: Lfo::default(),
            phase_off: 0.0,
            duty: 0.5,
            env: AhrEnvelope::default(),
            z: [[0.0; PHASER_MAX_STAGES]; 2],
            fb: [0.0; 2],
            a: [0.0; 2],
            a_step: [0.0; 2],
            ctr: 0,
            range: Lag::new(0.2),
            depth: Lag::new(0.5),
            emod: Lag::new(0.0),
            mix: Lag::new(0.5),
            reso: Lag::new(0.2),
            stage: Lag::new(6.0),
            primed: false,
            last_freq: [0.0; 2],
        }
    }

    pub fn params(&self) -> &PhaserParams {
        &self.p
    }

    pub fn set_params(&mut self, p: &PhaserParams) {
        self.p = *p;
        self.dirty = true;
    }

    /// Break frequencies (Hz, L/R) of the last control-rate update.
    pub fn current_freqs(&self) -> (f32, f32) {
        (self.last_freq[0], self.last_freq[1])
    }

    fn update(&mut self) {
        self.dirty = false;
        let p = self.p;
        let fs = self.fs;
        self.lfo.set_rate(p.speed_hz.clamp(0.05, 5.0) as f64, fs);
        self.phase_off = (p.phase_deg.clamp(0.0, 180.0) / 360.0) as f64;
        self.duty = 0.5 + p.wave.clamp(-50.0, 50.0) * 0.01;
        self.env.set_times(
            p.att_ms.clamp(10.0, 1000.0),
            p.hld_ms.clamp(10.0, 2000.0),
            p.rel_ms.clamp(10.0, 1000.0),
            fs,
        );
        self.range.t = p.range_pct.clamp(2.0, 98.0) * 0.01;
        self.depth.t = p.depth_pct.clamp(0.0, 100.0) * 0.01;
        self.emod.t = p.emod_pct.clamp(-100.0, 100.0) * 0.01;
        self.mix.t = p.mix_pct.clamp(0.0, 100.0) * 0.01;
        self.reso.t = p.reso_pct.clamp(0.0, 80.0) * 0.01 * (PHASER_MAX_RESO / 0.8);
        self.stage.t = p.stages.clamp(2, PHASER_MAX_STAGES as u32) as f32;
        if self.fresh {
            self.for_lags(Lag::snap);
            self.ctr = 0;
        }
    }

    fn for_lags(&mut self, f: impl Fn(&mut Lag)) {
        for l in [
            &mut self.range,
            &mut self.depth,
            &mut self.emod,
            &mut self.mix,
            &mut self.reso,
            &mut self.stage,
        ] {
            f(l);
        }
    }

    #[inline]
    fn coef(f: f32, fs: f32) -> f32 {
        let t = (PI * f / fs).tan();
        (t - 1.0) / (t + 1.0)
    }
}

impl Model for Phaser {
    fn id(&self) -> &'static str {
        "PHASER"
    }

    fn prepare(&mut self, fs: f64, _max_block: usize) {
        self.fs = fs;
        self.for_lags(|l| l.set_tau(GLIDE_S, fs));
        self.stage.set_tau(0.015, fs);
        self.dirty = true;
        self.reset();
    }

    fn process(&mut self, left: &mut [f32], right: &mut [f32], _sc: Option<(&[f32], &[f32])>) {
        if self.dirty {
            self.update();
        }
        let fs = self.fs as f32;
        for (xl, xr) in left.iter_mut().zip(right.iter_mut()) {
            self.env.tick(xl.abs().max(xr.abs()));
            let u = self.lfo.tick();
            let (range, depth, emod) = (self.range.tick(), self.depth.tick(), self.emod.tick());
            if self.ctr == 0 {
                self.ctr = CR;
                let en = self.env.normalized();
                for k in 0..2 {
                    let uk = if k == 0 { u } else { wrap(u + self.phase_off) };
                    let m = wave::skew(uk as f32, self.duty);
                    let pos = range + 0.5 * depth * m + 0.5 * emod * en;
                    let f = phaser_freq(pos).min(0.45 * fs);
                    self.last_freq[k] = f;
                    let target = Self::coef(f, fs);
                    if !self.primed {
                        self.a[k] = target;
                    }
                    self.a_step[k] = (target - self.a[k]) / CR as f32;
                }
                self.primed = true;
            }
            self.ctr -= 1;
            let mix = self.mix.tick();
            let reso = self.reso.tick();
            let st = self.stage.tick();
            let lo = (st.floor() as usize).clamp(1, PHASER_MAX_STAGES);
            let frac = st - lo as f32;
            let x = [*xl, *xr];
            let mut out = [0.0f32; 2];
            for k in 0..2 {
                self.a[k] += self.a_step[k];
                let a = self.a[k];
                let z = &mut self.z[k];
                let mut s = x[k] + reso * self.fb[k];
                let (mut tap_lo, mut tap_hi) = (0.0, 0.0);
                for (i, zi) in z.iter_mut().enumerate() {
                    let y = a * s + *zi;
                    *zi = s - a * y;
                    s = y;
                    if i + 1 == lo {
                        tap_lo = s;
                    } else if i == lo {
                        tap_hi = s;
                    }
                }
                if lo == PHASER_MAX_STAGES {
                    tap_hi = tap_lo;
                }
                let wet = tap_lo + frac * (tap_hi - tap_lo);
                self.fb[k] = wet;
                out[k] = x[k] + mix * (wet - x[k]);
            }
            *xl = out[0];
            *xr = out[1];
        }
        if !left.is_empty() {
            self.fresh = false;
        }
        for k in 0..2 {
            for z in self.z[k]
                .iter_mut()
                .chain(core::iter::once(&mut self.fb[k]))
            {
                if z.abs() < 1e-15 {
                    *z = 0.0;
                }
            }
        }
        self.env.flush();
        self.for_lags(Lag::settle);
    }

    fn reset(&mut self) {
        self.fresh = true;
        self.dirty = true;
        self.update();
        self.lfo.reset();
        self.env.reset();
        self.z = [[0.0; PHASER_MAX_STAGES]; 2];
        self.fb = [0.0; 2];
        self.a = [0.0; 2];
        self.a_step = [0.0; 2];
        self.ctr = 0;
        self.primed = false;
    }

    fn set_param(&mut self, slot: u16, v: ParamValue) {
        let p = &mut self.p;
        let x = v.as_f32();
        match slot {
            phaser_slot::SPD => p.speed_hz = x,
            phaser_slot::PHASE => p.phase_deg = x,
            phaser_slot::WAVE => p.wave = x,
            phaser_slot::RANGE => p.range_pct = x,
            phaser_slot::DEPTH => p.depth_pct = x,
            phaser_slot::EMOD => p.emod_pct = x,
            phaser_slot::ATT => p.att_ms = x,
            phaser_slot::HLD => p.hld_ms = x,
            phaser_slot::REL => p.rel_ms = x,
            phaser_slot::MIX => p.mix_pct = x,
            phaser_slot::STG => p.stages = v.as_i32().clamp(2, PHASER_MAX_STAGES as i32) as u32,
            phaser_slot::RESO => p.reso_pct = x,
            _ => return,
        }
        self.dirty = true;
    }

    fn param(&self, slot: u16) -> Option<f32> {
        let p = &self.p;
        Some(match slot {
            phaser_slot::SPD => p.speed_hz,
            phaser_slot::PHASE => p.phase_deg,
            phaser_slot::WAVE => p.wave,
            phaser_slot::RANGE => p.range_pct,
            phaser_slot::DEPTH => p.depth_pct,
            phaser_slot::EMOD => p.emod_pct,
            phaser_slot::ATT => p.att_ms,
            phaser_slot::HLD => p.hld_ms,
            phaser_slot::REL => p.rel_ms,
            phaser_slot::MIX => p.mix_pct,
            phaser_slot::STG => p.stages as f32,
            phaser_slot::RESO => p.reso_pct,
            _ => return None,
        })
    }

    fn load_defaults(&mut self) {
        self.set_params(&PhaserParams::default());
    }

    fn commit(&mut self) {
        if self.dirty {
            self.update();
        }
    }
}

// ---------------------------------------------------------------------------------------------------------
// ROTARY
// ---------------------------------------------------------------------------------------------------------

/// ROTARY schema slots.
pub mod rotary_slot {
    pub const SW: u16 = 1;
    pub const LO: u16 = 2;
    pub const HI: u16 = 3;
    pub const BAL: u16 = 4;
    pub const MIX: u16 = 5;
    pub const DIST: u16 = 6;
    pub const DAC: u16 = 7;
    pub const HAC: u16 = 8;
}

/// Horn / drum crossover (Leslie 122 passive crossover).
pub const ROTARY_XOVER_HZ: f64 = 800.0;
/// Horn Doppler swing: 15 cm horn radius at 343 m/s.
pub const HORN_DOPPLER_MS: f32 = 0.15 / 343.0 * 1000.0;
/// Drum (rotating baffle) effective Doppler swing [I].
pub const DRUM_DOPPLER_MS: f32 = 0.15;
/// Drum setpoint relative to the horn (Leslie chorale 0.67/0.83 Hz, tremolo 5.7/6.75 Hz).
pub const DRUM_RATIO: f64 = 0.85;
/// Spin-down is slower than spin-up by this factor.
const ROTARY_DECEL: f64 = 1.6;
const ROTARY_BASE_MS: f32 = 1.0;

/// `SPD CTL`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RotorSpeed {
    Stop,
    Slow,
    Fast,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RotaryParams {
    pub speed: RotorSpeed,
    pub lo_hz: f32,
    pub hi_hz: f32,
    pub balance: f32,
    pub mix_pct: f32,
    pub distance: f32,
    pub drum_accel_pct: f32,
    pub horn_accel_pct: f32,
}

impl Default for RotaryParams {
    fn default() -> Self {
        Self {
            speed: RotorSpeed::Slow,
            lo_hz: 0.68,
            hi_hz: 5.0,
            balance: -10.0,
            mix_pct: 80.0,
            distance: 20.0,
            drum_accel_pct: 80.0,
            horn_accel_pct: 70.0,
        }
    }
}

impl RotaryParams {
    /// Spin-up time constants (horn, drum) in seconds [I]: horn 0.25..2.75 s, drum 0.8..8.8 s.
    pub fn time_constants(&self) -> (f64, f64) {
        let h = 0.25 + (1.0 - self.horn_accel_pct.clamp(0.0, 100.0) as f64 * 0.01) * 2.5;
        let d = 0.8 + (1.0 - self.drum_accel_pct.clamp(0.0, 100.0) as f64 * 0.01) * 8.0;
        (h, d)
    }

    /// Horn setpoint (Hz).
    pub fn horn_target(&self) -> f64 {
        match self.speed {
            RotorSpeed::Stop => 0.0,
            RotorSpeed::Slow => self.lo_hz.clamp(0.1, 4.0) as f64,
            RotorSpeed::Fast => self.hi_hz.clamp(4.0, 10.0) as f64,
        }
    }
}

/// ROTARY SPEAKER (`ROTARY`).
pub struct Rotary {
    p: RotaryParams,
    dirty: bool,
    fresh: bool,
    fs: f64,
    horn_line: DelayLine,
    drum_line: DelayLine,
    xo_lp: Svf,
    xo_hp: Svf,
    horn_lp: Svf,
    drum_body: Svf,
    beam: [f32; 2],
    horn_speed: f64,
    drum_speed: f64,
    horn_phase: f64,
    drum_phase: f64,
    coefs: [f64; 4],
    mix: Lag,
    horn_g: Lag,
    drum_g: Lag,
    am_h: Lag,
    am_d: Lag,
    blend: Lag,
}

impl Default for Rotary {
    fn default() -> Self {
        Self::new()
    }
}

impl Rotary {
    pub fn new() -> Self {
        Self {
            p: RotaryParams::default(),
            dirty: true,
            fresh: true,
            fs: 48_000.0,
            horn_line: DelayLine::new(1),
            drum_line: DelayLine::new(1),
            xo_lp: Svf::default(),
            xo_hp: Svf::default(),
            horn_lp: Svf::default(),
            drum_body: Svf::default(),
            beam: [0.0; 2],
            horn_speed: 0.0,
            drum_speed: 0.0,
            horn_phase: 0.0,
            drum_phase: 0.25,
            coefs: [1.0; 4],
            mix: Lag::new(0.8),
            horn_g: Lag::new(1.0),
            drum_g: Lag::new(1.0),
            am_h: Lag::new(0.5),
            am_d: Lag::new(0.3),
            blend: Lag::new(0.0),
        }
    }

    pub fn params(&self) -> &RotaryParams {
        &self.p
    }

    pub fn set_params(&mut self, p: &RotaryParams) {
        self.p = *p;
        self.dirty = true;
    }

    /// Current rotor speeds (horn, drum) in Hz.
    pub fn rotor_speeds(&self) -> (f64, f64) {
        (self.horn_speed, self.drum_speed)
    }

    fn update(&mut self) {
        self.dirty = false;
        let p = self.p;
        let fs = self.fs;
        let (th, td) = p.time_constants();
        let c = |tau: f64| 1.0 - (-1.0 / (tau * fs)).exp();
        self.coefs = [c(th), c(th * ROTARY_DECEL), c(td), c(td * ROTARY_DECEL)];
        let bal = p.balance.clamp(-100.0, 100.0) * 0.01;
        self.horn_g.t = (1.0 + bal).min(1.0);
        self.drum_g.t = (1.0 - bal).min(1.0);
        self.mix.t = p.mix_pct.clamp(0.0, 100.0) * 0.01;
        let d = p.distance.clamp(0.0, 100.0) * 0.01;
        self.am_h.t = 0.8 + (0.35 - 0.8) * d;
        self.am_d.t = 0.5 + (0.2 - 0.5) * d;
        self.blend.t = 0.35 * d;
        if self.fresh {
            self.for_lags(Lag::snap);
            // a freshly selected rotary starts at its setpoint
            self.horn_speed = p.horn_target();
            self.drum_speed = p.horn_target() * DRUM_RATIO;
        }
    }

    fn for_lags(&mut self, f: impl Fn(&mut Lag)) {
        for l in [
            &mut self.mix,
            &mut self.horn_g,
            &mut self.drum_g,
            &mut self.am_h,
            &mut self.am_d,
            &mut self.blend,
        ] {
            f(l);
        }
    }
}

impl Model for Rotary {
    fn id(&self) -> &'static str {
        "ROTARY"
    }

    fn prepare(&mut self, fs: f64, _max_block: usize) {
        self.fs = fs;
        let n = ((ROTARY_BASE_MS + HORN_DOPPLER_MS) as f64 * 1e-3 * fs).ceil() as usize + 8;
        self.horn_line = DelayLine::new(n);
        self.drum_line = DelayLine::new(n);
        self.xo_lp = svf(SvfMode::Lowpass, fs, ROTARY_XOVER_HZ, Q_BW);
        self.xo_hp = svf(SvfMode::Highpass, fs, ROTARY_XOVER_HZ, Q_BW);
        self.horn_lp = svf(SvfMode::Lowpass, fs, 7000.0, 0.9);
        self.drum_body = Svf::new(SvfMode::Bell, fs, 90.0, 1.0, 2.0);
        self.for_lags(|l| l.set_tau(GLIDE_S, fs));
        self.dirty = true;
        self.reset();
    }

    fn process(&mut self, left: &mut [f32], right: &mut [f32], _sc: Option<(&[f32], &[f32])>) {
        if self.dirty {
            self.update();
        }
        let fs = self.fs;
        let inv_fs = 1.0 / fs;
        let ms = (fs * 1e-3) as f32;
        let base = ROTARY_BASE_MS * ms;
        let (hr, dr) = (HORN_DOPPLER_MS * ms, DRUM_DOPPLER_MS * ms);
        let ht = self.p.horn_target();
        let dt = ht * DRUM_RATIO;
        let [hu, hd, du, dd] = self.coefs;
        let w_scale = TAU / fs as f32;
        for (xl, xr) in left.iter_mut().zip(right.iter_mut()) {
            let m = 0.5 * (*xl + *xr);
            let lo = self.drum_body.process(self.xo_lp.process(m));
            let hi = self.horn_lp.process(self.xo_hp.process(m));
            self.horn_line.push(hi);
            self.drum_line.push(lo);
            self.horn_speed += (ht - self.horn_speed) * if ht > self.horn_speed { hu } else { hd };
            self.drum_speed += (dt - self.drum_speed) * if dt > self.drum_speed { du } else { dd };
            self.horn_phase = wrap(self.horn_phase + self.horn_speed * inv_fs);
            self.drum_phase = wrap(self.drum_phase + self.drum_speed * inv_fs);
            let ch = (TAU * self.horn_phase as f32).cos();
            let cd = (TAU * self.drum_phase as f32).cos();
            let (hg, dg) = (self.horn_g.tick(), self.drum_g.tick());
            let (amh, amd) = (self.am_h.tick(), self.am_d.tick());
            let mut wet = [0.0f32; 2];
            for (k, sgn) in [(0usize, 1.0f32), (1, -1.0)] {
                // mic k faces the rotor at angle 0 (L) / pi (R): facing = closest = shortest path
                let c = sgn * ch;
                let h = self.horn_line.read_lagrange3(base - hr * c);
                // horn beaming: brighter when the mouth faces the mic (3..12 kHz one-pole)
                let f = 3000.0 * (1.0 + 3.0 * (0.5 + 0.5 * c));
                let w = f * w_scale;
                self.beam[k] += w / (1.0 + w) * (h - self.beam[k]);
                let h = self.beam[k] * (1.0 - amh * (0.5 - 0.5 * c));
                let c = sgn * cd;
                let d =
                    self.drum_line.read_lagrange3(base - dr * c) * (1.0 - amd * (0.5 - 0.5 * c));
                wet[k] = hg * h + dg * d;
            }
            let b = self.blend.tick();
            let (wl, wr) = (
                wet[0] + b * (wet[1] - wet[0]),
                wet[1] + b * (wet[0] - wet[1]),
            );
            let mix = self.mix.tick();
            *xl += mix * (wl - *xl);
            *xr += mix * (wr - *xr);
        }
        if !left.is_empty() {
            self.fresh = false;
        }
        flush_svfs([
            &mut self.xo_lp,
            &mut self.xo_hp,
            &mut self.horn_lp,
            &mut self.drum_body,
        ]);
        for b in &mut self.beam {
            if b.abs() < 1e-15 {
                *b = 0.0;
            }
        }
        self.for_lags(Lag::settle);
    }

    fn reset(&mut self) {
        self.fresh = true;
        self.dirty = true;
        self.update();
        self.horn_line.reset();
        self.drum_line.reset();
        for s in [
            &mut self.xo_lp,
            &mut self.xo_hp,
            &mut self.horn_lp,
            &mut self.drum_body,
        ] {
            s.reset();
        }
        self.beam = [0.0; 2];
        self.horn_phase = 0.0;
        self.drum_phase = 0.25;
    }

    fn set_param(&mut self, slot: u16, v: ParamValue) {
        let p = &mut self.p;
        let x = v.as_f32();
        match slot {
            rotary_slot::SW => {
                p.speed = match v.as_index() {
                    0 => RotorSpeed::Stop,
                    1 => RotorSpeed::Slow,
                    _ => RotorSpeed::Fast,
                }
            }
            rotary_slot::LO => p.lo_hz = x,
            rotary_slot::HI => p.hi_hz = x,
            rotary_slot::BAL => p.balance = x,
            rotary_slot::MIX => p.mix_pct = x,
            rotary_slot::DIST => p.distance = x,
            rotary_slot::DAC => p.drum_accel_pct = x,
            rotary_slot::HAC => p.horn_accel_pct = x,
            _ => return,
        }
        self.dirty = true;
    }

    fn param(&self, slot: u16) -> Option<f32> {
        let p = &self.p;
        Some(match slot {
            rotary_slot::SW => p.speed as u8 as f32,
            rotary_slot::LO => p.lo_hz,
            rotary_slot::HI => p.hi_hz,
            rotary_slot::BAL => p.balance,
            rotary_slot::MIX => p.mix_pct,
            rotary_slot::DIST => p.distance,
            rotary_slot::DAC => p.drum_accel_pct,
            rotary_slot::HAC => p.horn_accel_pct,
            _ => return None,
        })
    }

    fn load_defaults(&mut self) {
        self.set_params(&RotaryParams::default());
    }

    fn commit(&mut self) {
        if self.dirty {
            self.update();
        }
    }
}

// ---------------------------------------------------------------------------------------------------------
// PANNER
// ---------------------------------------------------------------------------------------------------------

/// PANNER schema slots.
pub mod panner_slot {
    pub const ATT: u16 = 1;
    pub const HLD: u16 = 2;
    pub const REL: u16 = 3;
    pub const ESPD: u16 = 4;
    pub const EDEP: u16 = 5;
    pub const SPD: u16 = 6;
    pub const PHASE: u16 = 7;
    pub const WAVE: u16 = 8;
    pub const DEPTH: u16 = 9;
}

/// ENV>SPEED 100 % at full envelope multiplies the LFO rate by `1 + PANNER_ENV_SPEED` [I].
pub const PANNER_ENV_SPEED: f32 = 3.0;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PannerParams {
    pub att_ms: f32,
    pub hld_ms: f32,
    pub rel_ms: f32,
    pub env_speed_pct: f32,
    pub env_depth_pct: f32,
    pub speed_hz: f32,
    pub phase_deg: f32,
    pub wave: f32,
    pub depth_pct: f32,
}

impl Default for PannerParams {
    fn default() -> Self {
        Self {
            att_ms: 20.0,
            hld_ms: 50.0,
            rel_ms: 200.0,
            env_speed_pct: 0.0,
            env_depth_pct: 0.0,
            speed_hz: 1.44,
            phase_deg: 180.0,
            wave: 0.0,
            depth_pct: 100.0,
        }
    }
}

/// TREMOLO/PANNER (`PANNER`).
pub struct Panner {
    p: PannerParams,
    dirty: bool,
    fresh: bool,
    fs: f64,
    lfo: Lfo,
    phase_off: f64,
    duty: f32,
    env: AhrEnvelope,
    espd: f32,
    edep: f32,
    depth: Lag,
    gain: [Lag; 2],
    primed: bool,
}

impl Default for Panner {
    fn default() -> Self {
        Self::new()
    }
}

impl Panner {
    pub fn new() -> Self {
        Self {
            p: PannerParams::default(),
            dirty: true,
            fresh: true,
            fs: 48_000.0,
            lfo: Lfo::default(),
            phase_off: 0.5,
            duty: 0.5,
            env: AhrEnvelope::default(),
            espd: 0.0,
            edep: 0.0,
            depth: Lag::new(1.0),
            gain: [Lag::new(1.0); 2],
            primed: false,
        }
    }

    pub fn params(&self) -> &PannerParams {
        &self.p
    }

    pub fn set_params(&mut self, p: &PannerParams) {
        self.p = *p;
        self.dirty = true;
    }

    /// Current gains (L, R).
    pub fn gains(&self) -> (f32, f32) {
        (self.gain[0].v, self.gain[1].v)
    }

    fn update(&mut self) {
        self.dirty = false;
        let p = self.p;
        self.lfo
            .set_rate(p.speed_hz.clamp(0.05, 5.0) as f64, self.fs);
        self.phase_off = wrap((p.phase_deg.clamp(-180.0, 180.0) / 360.0) as f64);
        self.duty = 0.5 + p.wave.clamp(-50.0, 50.0) * 0.01;
        self.env.set_times(
            p.att_ms.clamp(10.0, 1000.0),
            p.hld_ms.clamp(10.0, 2000.0),
            p.rel_ms.clamp(10.0, 1000.0),
            self.fs,
        );
        self.espd = p.env_speed_pct.clamp(0.0, 100.0) * 0.01;
        self.edep = p.env_depth_pct.clamp(0.0, 100.0) * 0.01;
        self.depth.t = p.depth_pct.clamp(0.0, 100.0) * 0.01;
        if self.fresh {
            self.depth.snap();
        }
    }
}

impl Model for Panner {
    fn id(&self) -> &'static str {
        "PANNER"
    }

    fn prepare(&mut self, fs: f64, _max_block: usize) {
        self.fs = fs;
        self.depth.set_tau(GLIDE_S, fs);
        for g in &mut self.gain {
            g.set_tau(0.001, fs);
        }
        self.dirty = true;
        self.reset();
    }

    fn process(&mut self, left: &mut [f32], right: &mut [f32], _sc: Option<(&[f32], &[f32])>) {
        if self.dirty {
            self.update();
        }
        let base_inc = self.lfo.increment();
        let (espd, edep, duty) = (self.espd, self.edep, self.duty);
        for (xl, xr) in left.iter_mut().zip(right.iter_mut()) {
            self.env.tick(xl.abs().max(xr.abs()));
            let en = self.env.normalized();
            let inc = base_inc * (1.0 + (PANNER_ENV_SPEED * espd * en) as f64);
            let u = self.lfo.tick_with(inc);
            let dep = self.depth.tick() * ((1.0 - edep) + edep * en);
            for (k, uk) in [(0usize, u), (1, wrap(u + self.phase_off))] {
                let w = 0.5 + 0.5 * wave::skew(uk as f32, duty);
                self.gain[k].t = 1.0 - dep * (1.0 - w);
            }
            if !self.primed {
                self.primed = true;
                self.gain[0].snap();
                self.gain[1].snap();
            }
            *xl *= self.gain[0].tick();
            *xr *= self.gain[1].tick();
        }
        self.lfo.set_rate(base_inc * self.fs, self.fs);
        if !left.is_empty() {
            self.fresh = false;
        }
        self.env.flush();
        self.depth.settle();
    }

    fn reset(&mut self) {
        self.fresh = true;
        self.dirty = true;
        self.update();
        self.lfo.reset();
        self.env.reset();
        self.primed = false;
    }

    fn set_param(&mut self, slot: u16, v: ParamValue) {
        let p = &mut self.p;
        let x = v.as_f32();
        match slot {
            panner_slot::ATT => p.att_ms = x,
            panner_slot::HLD => p.hld_ms = x,
            panner_slot::REL => p.rel_ms = x,
            panner_slot::ESPD => p.env_speed_pct = x,
            panner_slot::EDEP => p.env_depth_pct = x,
            panner_slot::SPD => p.speed_hz = x,
            panner_slot::PHASE => p.phase_deg = x,
            panner_slot::WAVE => p.wave = x,
            panner_slot::DEPTH => p.depth_pct = x,
            _ => return,
        }
        self.dirty = true;
    }

    fn param(&self, slot: u16) -> Option<f32> {
        let p = &self.p;
        Some(match slot {
            panner_slot::ATT => p.att_ms,
            panner_slot::HLD => p.hld_ms,
            panner_slot::REL => p.rel_ms,
            panner_slot::ESPD => p.env_speed_pct,
            panner_slot::EDEP => p.env_depth_pct,
            panner_slot::SPD => p.speed_hz,
            panner_slot::PHASE => p.phase_deg,
            panner_slot::WAVE => p.wave,
            panner_slot::DEPTH => p.depth_pct,
            _ => return None,
        })
    }

    fn load_defaults(&mut self) {
        self.set_params(&PannerParams::default());
    }

    fn commit(&mut self) {
        if self.dirty {
            self.update();
        }
    }
}

// ---------------------------------------------------------------------------------------------------------
// MOOD
// ---------------------------------------------------------------------------------------------------------

/// MOOD schema slots.
pub mod mood_slot {
    pub const FBASE: u16 = 1;
    pub const FILT: u16 = 2;
    pub const SLOPE: u16 = 3;
    pub const RESO: u16 = 4;
    pub const DRV: u16 = 5;
    pub const ENV: u16 = 6;
    pub const ATT: u16 = 7;
    pub const HLD: u16 = 8;
    pub const REL: u16 = 9;
    pub const MIX: u16 = 10;
    pub const LFO: u16 = 11;
    pub const SPD: u16 = 12;
    pub const PHASE: u16 = 13;
    pub const WAVE: u16 = 14;
}

/// ENV 100 % at full envelope sweeps the cutoff this many octaves [I].
pub const MOOD_ENV_OCT: f32 = 6.0;
/// LFO 100 % sweeps the cutoff up to this many octaves above BASE [I].
pub const MOOD_LFO_OCT: f32 = 5.0;
/// Maximum DRIVE gain (dB) into the soft clipper [I].
pub const MOOD_DRIVE_DB: f32 = 24.0;
const MOOD_CR: u32 = 8;

/// `TYPE`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MoodType {
    Lp,
    Hp,
    Bp,
    Notch,
}

/// `LFO WAVE`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MoodWave {
    Tri,
    Sin,
    SawUp,
    SawDown,
    Ramp,
    Square,
    Random,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MoodParams {
    pub base_hz: f32,
    pub ftype: MoodType,
    /// SLOPE 24 dB (true) / 12 dB.
    pub slope24: bool,
    pub reso: f32,
    pub drive: f32,
    pub env_pct: f32,
    pub att_ms: f32,
    pub hld_ms: f32,
    pub rel_ms: f32,
    pub mix_pct: f32,
    pub lfo_pct: f32,
    pub speed_hz: f32,
    pub phase_deg: f32,
    pub wave: MoodWave,
}

impl Default for MoodParams {
    fn default() -> Self {
        Self {
            base_hz: 98.0,
            ftype: MoodType::Lp,
            slope24: true,
            reso: 0.0,
            drive: 0.0,
            env_pct: 0.0,
            att_ms: 20.0,
            hld_ms: 20.0,
            rel_ms: 100.0,
            mix_pct: 100.0,
            lfo_pct: 100.0,
            speed_hz: 1.0,
            phase_deg: 0.0,
            wave: MoodWave::Sin,
        }
    }
}

impl MoodParams {
    /// Filter Q for RESO 0..10 (0.5 .. 20, log) [I].
    pub fn q(&self) -> f32 {
        0.5 * 40f32.powf(self.reso.clamp(0.0, 10.0) / 10.0)
    }
}

/// Unipolar (0..1) MOOD LFO shape.
pub fn mood_wave(w: MoodWave, u: f32, rnd: f32) -> f32 {
    match w {
        MoodWave::Tri => 0.5 + 0.5 * wave::tri(u - 0.25),
        MoodWave::Sin => 0.5 - 0.5 * (TAU * u).cos(),
        MoodWave::SawUp => u,
        MoodWave::SawDown => 1.0 - u,
        MoodWave::Ramp => u * u,
        MoodWave::Square => {
            if u < 0.5 {
                1.0
            } else {
                0.0
            }
        }
        MoodWave::Random => rnd,
    }
}

/// Seeds of the random LFO shape (L, R), restored on `reset`.
const MOOD_RNG_SEEDS: [u32; 2] = [0x1234_5678, 0x9e37_79b9];

/// MOOD FILTER (`MOOD`).
pub struct MoodFilter {
    p: MoodParams,
    dirty: bool,
    fresh: bool,
    fs: f64,
    svf: [[Svf; 2]; 2],
    lfo: Lfo,
    phase_off: f64,
    env: AhrEnvelope,
    rng: [u32; 2],
    rnd: [f32; 2],
    prev_u: [f32; 2],
    oct: [f32; 2],
    oct_a: f32,
    g: [f32; 2],
    g_step: [f32; 2],
    ctr: u32,
    base: Lag,
    env_amt: Lag,
    lfo_amt: Lag,
    k: Lag,
    /// Damping of the second (24 dB) stage: Butterworth-limited for LP/HP so the resonance peak is not
    /// squared, equal to the first stage for BP/NOTCH (narrower band).
    k2: Lag,
    cl: Lag,
    cb: Lag,
    ch: Lag,
    slope: Lag,
    mix: Lag,
    drive: Lag,
    primed: bool,
    last_freq: [f32; 2],
}

impl Default for MoodFilter {
    fn default() -> Self {
        Self::new()
    }
}

impl MoodFilter {
    pub fn new() -> Self {
        Self {
            p: MoodParams::default(),
            dirty: true,
            fresh: true,
            fs: 48_000.0,
            svf: [[Svf::default(); 2]; 2],
            lfo: Lfo::default(),
            phase_off: 0.0,
            env: AhrEnvelope::default(),
            rng: MOOD_RNG_SEEDS,
            rnd: [0.5; 2],
            prev_u: [0.0; 2],
            oct: [0.0; 2],
            oct_a: 1.0,
            g: [0.0; 2],
            g_step: [0.0; 2],
            ctr: 0,
            base: Lag::new(98.0),
            env_amt: Lag::new(0.0),
            lfo_amt: Lag::new(1.0),
            k: Lag::new(2.0),
            k2: Lag::new(2.0),
            cl: Lag::new(1.0),
            cb: Lag::new(0.0),
            ch: Lag::new(0.0),
            slope: Lag::new(1.0),
            mix: Lag::new(1.0),
            drive: Lag::new(1.0),
            primed: false,
            last_freq: [0.0; 2],
        }
    }

    pub fn params(&self) -> &MoodParams {
        &self.p
    }

    pub fn set_params(&mut self, p: &MoodParams) {
        self.p = *p;
        self.dirty = true;
    }

    /// Cutoff (Hz, L/R) of the last control-rate update.
    pub fn current_cutoff(&self) -> (f32, f32) {
        (self.last_freq[0], self.last_freq[1])
    }

    fn update(&mut self) {
        self.dirty = false;
        let p = self.p;
        let fs = self.fs;
        self.lfo.set_rate(p.speed_hz.clamp(0.05, 20.0) as f64, fs);
        self.phase_off = (p.phase_deg.clamp(0.0, 180.0) / 360.0) as f64;
        self.env.set_times(
            p.att_ms.clamp(10.0, 250.0),
            p.hld_ms.clamp(1.0, 500.0),
            p.rel_ms.clamp(10.0, 500.0),
            fs,
        );
        self.base.t = p.base_hz.clamp(20.0, 15_000.0).ln();
        self.env_amt.t = p.env_pct.clamp(-100.0, 100.0) * 0.01;
        self.lfo_amt.t = p.lfo_pct.clamp(0.0, 100.0) * 0.01;
        self.k.t = 1.0 / p.q();
        self.k2.t = match p.ftype {
            MoodType::Lp | MoodType::Hp => self.k.t.max(core::f32::consts::SQRT_2),
            MoodType::Bp | MoodType::Notch => self.k.t,
        };
        let (l, b, h) = match p.ftype {
            MoodType::Lp => (1.0, 0.0, 0.0),
            MoodType::Hp => (0.0, 0.0, 1.0),
            MoodType::Bp => (0.0, 1.0, 0.0),
            MoodType::Notch => (1.0, 0.0, 1.0),
        };
        self.cl.t = l;
        self.cb.t = b;
        self.ch.t = h;
        self.slope.t = if p.slope24 { 1.0 } else { 0.0 };
        self.mix.t = p.mix_pct.clamp(0.0, 100.0) * 0.01;
        self.drive.t = 10f32.powf(p.drive.clamp(0.0, 10.0) * 0.1 * MOOD_DRIVE_DB / 20.0);
        if self.fresh {
            self.for_lags(Lag::snap);
            self.ctr = 0;
        }
    }

    fn for_lags(&mut self, f: impl Fn(&mut Lag)) {
        for l in [
            &mut self.base,
            &mut self.env_amt,
            &mut self.lfo_amt,
            &mut self.k,
            &mut self.k2,
            &mut self.cl,
            &mut self.cb,
            &mut self.ch,
            &mut self.slope,
            &mut self.mix,
            &mut self.drive,
        ] {
            f(l);
        }
    }

    #[inline]
    fn next_rnd(state: &mut u32) -> f32 {
        let mut x = *state;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        *state = x;
        (x >> 8) as f32 * (1.0 / (1u32 << 24) as f32)
    }
}

impl Model for MoodFilter {
    fn id(&self) -> &'static str {
        "MOOD"
    }

    fn prepare(&mut self, fs: f64, _max_block: usize) {
        self.fs = fs;
        self.for_lags(|l| l.set_tau(GLIDE_S, fs));
        for l in [&mut self.cl, &mut self.cb, &mut self.ch, &mut self.slope] {
            l.set_tau(0.005, fs);
        }
        // modulation smoothing (1.5 ms) evaluated once per control period
        self.oct_a = (1.0 - (-(MOOD_CR as f64) / (0.0015 * fs)).exp()) as f32;
        self.dirty = true;
        self.reset();
    }

    fn process(&mut self, left: &mut [f32], right: &mut [f32], _sc: Option<(&[f32], &[f32])>) {
        if self.dirty {
            self.update();
        }
        let fs = self.fs as f32;
        let f_hi = (0.45 * fs).min(20_000.0);
        let wave_kind = self.p.wave;
        let use_drive = self.drive.t > 1.0001 || self.drive.v > 1.0001;
        for (xl, xr) in left.iter_mut().zip(right.iter_mut()) {
            self.env.tick(xl.abs().max(xr.abs()));
            let u = self.lfo.tick();
            let base = self.base.tick();
            let (ea, la) = (self.env_amt.tick(), self.lfo_amt.tick());
            if self.ctr == 0 {
                self.ctr = MOOD_CR;
                let en = self.env.normalized();
                for k in 0..2 {
                    let uk = (if k == 0 { u } else { wrap(u + self.phase_off) }) as f32;
                    if uk < self.prev_u[k] {
                        self.rnd[k] = Self::next_rnd(&mut self.rng[k]);
                    }
                    self.prev_u[k] = uk;
                    let w = mood_wave(wave_kind, uk, self.rnd[k]);
                    let target = ea * MOOD_ENV_OCT * en + la * MOOD_LFO_OCT * w;
                    if !self.primed {
                        self.oct[k] = target;
                    } else {
                        self.oct[k] += self.oct_a * (target - self.oct[k]);
                    }
                    let f = (base + self.oct[k] * core::f32::consts::LN_2)
                        .exp()
                        .clamp(20.0, f_hi);
                    self.last_freq[k] = f;
                    let gt = (PI * f / fs).tan();
                    if !self.primed {
                        self.g[k] = gt;
                    }
                    self.g_step[k] = (gt - self.g[k]) / MOOD_CR as f32;
                }
                self.primed = true;
            }
            self.ctr -= 1;
            let kd = self.k.tick();
            let kd2 = self.k2.tick();
            let (cl, cb, ch) = (self.cl.tick(), self.cb.tick(), self.ch.tick());
            let slope = self.slope.tick();
            let mix = self.mix.tick();
            let drv = self.drive.tick();
            let x = [*xl, *xr];
            let mut out = [0.0f32; 2];
            for c in 0..2 {
                self.g[c] += self.g_step[c];
                let g = self.g[c];
                let mut s = x[c];
                if use_drive {
                    s = soft_clip(s * drv) / drv.sqrt();
                }
                let [s1, s2] = &mut self.svf[c];
                s1.set_g(g, kd);
                s2.set_g(g, kd2);
                let o = s1.tick_all(s);
                let y1 = cl * o.low + cb * kd * o.band + ch * o.high;
                let o = s2.tick_all(y1);
                let y2 = cl * o.low + cb * kd2 * o.band + ch * o.high;
                let y = y1 + slope * (y2 - y1);
                out[c] = x[c] + mix * (y - x[c]);
            }
            *xl = out[0];
            *xr = out[1];
        }
        if !left.is_empty() {
            self.fresh = false;
        }
        flush_svfs(self.svf.iter_mut().flatten());
        self.env.flush();
        self.for_lags(Lag::settle);
    }

    fn reset(&mut self) {
        self.fresh = true;
        self.dirty = true;
        self.update();
        self.lfo.reset();
        self.env.reset();
        for s in self.svf.iter_mut().flatten() {
            s.reset();
        }
        self.g = [0.0; 2];
        self.g_step = [0.0; 2];
        self.oct = [0.0; 2];
        self.prev_u = [0.0; 2];
        // T081: the random LFO restarts too (a reset must make the output repeatable)
        self.rng = MOOD_RNG_SEEDS;
        self.rnd = [0.5; 2];
        self.ctr = 0;
        self.primed = false;
    }

    fn set_param(&mut self, slot: u16, v: ParamValue) {
        let p = &mut self.p;
        let x = v.as_f32();
        match slot {
            mood_slot::FBASE => p.base_hz = x,
            mood_slot::FILT => {
                p.ftype = match v.as_index() {
                    0 => MoodType::Lp,
                    1 => MoodType::Hp,
                    2 => MoodType::Bp,
                    _ => MoodType::Notch,
                }
            }
            mood_slot::SLOPE => p.slope24 = v.as_index() != 0,
            mood_slot::RESO => p.reso = x,
            mood_slot::DRV => p.drive = x,
            mood_slot::ENV => p.env_pct = x,
            mood_slot::ATT => p.att_ms = x,
            mood_slot::HLD => p.hld_ms = x,
            mood_slot::REL => p.rel_ms = x,
            mood_slot::MIX => p.mix_pct = x,
            mood_slot::LFO => p.lfo_pct = x,
            mood_slot::SPD => p.speed_hz = x,
            mood_slot::PHASE => p.phase_deg = x,
            mood_slot::WAVE => {
                p.wave = match v.as_index() {
                    0 => MoodWave::Tri,
                    1 => MoodWave::Sin,
                    2 => MoodWave::SawUp,
                    3 => MoodWave::SawDown,
                    4 => MoodWave::Ramp,
                    5 => MoodWave::Square,
                    _ => MoodWave::Random,
                }
            }
            _ => return,
        }
        self.dirty = true;
    }

    fn param(&self, slot: u16) -> Option<f32> {
        let p = &self.p;
        Some(match slot {
            mood_slot::FBASE => p.base_hz,
            mood_slot::FILT => p.ftype as u8 as f32,
            mood_slot::SLOPE => {
                if p.slope24 {
                    1.0
                } else {
                    0.0
                }
            }
            mood_slot::RESO => p.reso,
            mood_slot::DRV => p.drive,
            mood_slot::ENV => p.env_pct,
            mood_slot::ATT => p.att_ms,
            mood_slot::HLD => p.hld_ms,
            mood_slot::REL => p.rel_ms,
            mood_slot::MIX => p.mix_pct,
            mood_slot::LFO => p.lfo_pct,
            mood_slot::SPD => p.speed_hz,
            mood_slot::PHASE => p.phase_deg,
            mood_slot::WAVE => p.wave as u8 as f32,
            _ => return None,
        })
    }

    fn load_defaults(&mut self) {
        self.set_params(&MoodParams::default());
    }

    fn commit(&mut self) {
        if self.dirty {
            self.update();
        }
    }
}

#[cfg(test)]
mod tests;
