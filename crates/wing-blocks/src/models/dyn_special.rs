//! Special dynamics models (T049): `WAVE` (WAVE DESIGNER, SPL Transient Designer style), `RIDE` (AUTO RIDER,
//! Waves Vocal Rider style) and `WARM` (SOUL WARMTH PRE, SSL "Variable Harmonic Drive" preamp style).
//! Parameters, ranges and defaults: `docs/research/models.md` §3.1 and `data/schema/wing-tree.json`
//! (slot order `att sust g` / `thr tgt spd ratio hld range` / `drv hrm col trim wmix`).
//!
//! # WAVE: differential-envelope transient designer
//! ```text
//! key = max(|L|,|R|) (or the external side-chain) -> fast peak follower --> slow follower of the fast one
//!                                                    d = fast_dB - slow_dB
//! d > dead zone  -> onset weight  wa = ramp((d - DEAD)/(A_FULL - DEAD))     (attack region)
//! d < -dead zone -> decay weight  ws = ramp((-d - DEAD)/(S_FULL - DEAD))    (sustain / tail region)
//! gain_dB = (att * wa + sust * ws) * presence(fast_dB)  -> 0.5 ms smoother -> VCA -> output gain
//! ```
//! Level independence: the control signal is a *difference of two dB envelopes*, so scaling the input by any
//! factor leaves the gain trajectory unchanged (tested: 24 dB more input gives the same gain curve to
//! < 0.1 dB). `att`/`sust` are the dB gain applied at full weight (the maximum boost/cut the knob can add).
//! A steady tone has `d = 0` and passes at unity; so does a decay slower than the slow follower's release.
//! The SPL manual publishes no time constants; every constant below is an *assumption* (listed in
//! `docs/VERIFY-ON-HARDWARE.md`).
//!
//! # RIDE: auto rider
//! ```text
//! key -> mean-square one-pole (tau 100 ms) -> level_dBFS(RMS) -> every CTL samples:
//!   level < thr : hold (gain frozen for `hld`, INF = forever), then relax to 0 dB at the up rate
//!   else        : desired = clamp((tgt - level) * (1 - 1/ratio), -range, +range)
//!                 gain slews toward desired at rate(spd) dB/s up, 2x that down (louder -> pull down quicker)
//! gain -> 20 ms one-pole -> VCA
//! ```
//! The rider is deliberately slow: a vocal "fader ride", not a compressor, so there is no look-ahead and no
//! latency. `spd` 1..50 maps geometrically to 2..200 dB/s.
//!
//! # WARM: VHD preamp
//! ```text
//! x -> tilt (pre, col) -> 2x up -> g = drive(drv) -> x + ADAA(remainder of tanh(u + b) - tanh(b)) / ((1 - T^2) g) -> 2x down
//!   -> tilt (post, 50 % of the pre tilt reversed) -> DC block -> mix with delay-compensated dry -> trim
//! ```
//! `T = tanh(b)` is the bias: expanding `tanh(u + b)` gives `H2 ~ T u / 2` and `H3 ~ (1 - 3 T^2) u^2 / 12`
//! (u = drive-scaled signal), so `hrm` sweeps `T` from 0 (`hrm = +100`: symmetric, 3rd harmonic only) to
//! `T_MAX` (`hrm = -100`: biased, 2nd dominant and the 3rd largely cancelled), as the VHD's 2nd -> 3rd sweep.
//! The small-signal gain is normalised to unity (the `drv` knob adds harmonics, not level); the saturated
//! peak falls as `1/g`. 2x oversampling (`Oversampler2x`, latency 39) plus first-order antiderivative
//! anti-aliasing (ADAA, ln cosh antiderivative) keep the harmonics of a 10 kHz input aliasing-free (tested).
//! Channels are processed independently (a preamp, not a stereo-linked dynamics unit).
//!
//! Every constant names its source; *assumption* values are listed in `docs/VERIFY-ON-HARDWARE.md`.
//!
//! WARM harmonic profile (1 kHz sine, peak -12 dBFS unless noted; `warm_thd_profile` prints it):
//! ```text
//! drv   hrm    H2        H3        THD
//!  33     0   -37.0 dB  -59.4 dB  1.4 %    (defaults)
//!  33  -100   -31.0 dB  -66.6 dB  2.8 %    even end: 2nd dominant, 3rd cancelled
//!  33  +100  < -180 dB  -57.9 dB  0.13 %   odd end: 3rd only
//!  80     0   -26.5 dB  -37.5 dB  4.9 %
//! 125     0   -21.1 dB  -19.0 dB  14.3 %
//! 125     0   -27.6 dB  -10.7 dB  29.5 %   at 0 dBFS * 0.9 (hard drive)
//! ```
//! (T081: exponential drive law, see [`WARM_DRIVE_G_MAX`]; the default row is unchanged.) Worst alias line for
//! a 10 kHz input: < -100 dBc up to 125 % drive at -20 dBFS; -48 dBc in the extreme corner (125 % drive,
//! 0.9 amplitude, hrm -100: a near-square 10 kHz wave), bounded in `warm_is_alias_free`. WAVE has
//! no latency; RIDE none; WARM 39 samples (the oversampler).
//!
//! Real-time: `prepare` allocates the oversamplers and scratch; `process`, `set_param`, `commit`, `reset` do
//! not allocate (`tests/dyn_special_no_alloc.rs`). Bench (`benches/dyn_special.rs`, M1, stereo 64-sample
//! block, 48 kHz): WAVE ~0.8 µs, RIDE ~0.14 µs, WARM ~5.2 µs (~0.4 % of a core per instance).

use crate::model::{Model, ModelTelemetry, ParamValue};
use wing_dsp::gain::{db_to_lin_fast, lin_to_db_fast};
use wing_dsp::oversample::Oversampler2x;
use wing_dsp::smooth::{one_pole_coeff, OnePoleSmoother};

// ---- shared helpers ---------------------------------------------------------------------------------------

#[inline]
fn ramp01(x: f32) -> f32 {
    x.clamp(0.0, 1.0)
}

/// Clamp the value of `slot` (1-based) into its `(min, max, default)` table row.
fn stage(v: &mut [f32], rows: &[(f32, f32, f32)], slot: u16, value: ParamValue) -> bool {
    let Some(i) = (slot as usize).checked_sub(1).filter(|&i| i < rows.len()) else {
        return false;
    };
    v[i] = value.as_f32().clamp(rows[i].0, rows[i].1);
    true
}

// =============================================================================================================
// WAVE: transient designer
// =============================================================================================================

/// Fast follower: attack / release (s). *Assumption* (SPL publishes no time constants): fast enough to catch
/// the first cycle of a drum transient (0.3 ms), release long enough to ride over the half-wave ripple of
/// bass notes (down to ~50 Hz, ripple 10 ms / 40 ms = 2.2 dB, absorbed by [`WAVE_DEAD_DB`]).
pub const WAVE_FAST_ATT_S: f64 = 0.0003;
pub const WAVE_FAST_REL_S: f64 = 0.040;
/// Slow follower attack / release (s). *Assumption*: the slow envelope needs ~15 ms to catch up with a new
/// level (the "attack" window of the designer, 10-40 ms in the SPL descriptions of "the first 20-30 ms"),
/// and releases at 300 ms, i.e. up to ~29 dB/s: decays faster than that open the sustain region.
pub const WAVE_SLOW_ATT_S: f64 = 0.015;
pub const WAVE_SLOW_REL_S: f64 = 0.300;
/// Dead zone (dB) of the envelope difference before either region reacts (rejects LF ripple), and the
/// difference (dB) at which attack / sustain weights reach 1. *Assumption*.
pub const WAVE_DEAD_DB: f32 = 1.0;
pub const WAVE_ATTACK_FULL_DB: f32 = 9.0;
pub const WAVE_SUSTAIN_FULL_DB: f32 = 5.0;
/// Presence ramp: no sustain / attack gain below `WAVE_FLOOR_DB` (fast follower, dBFS), full above
/// `WAVE_FLOOR_DB + WAVE_FLOOR_RAMP_DB`; stops +24 dB sustain from pumping up the noise floor. *Assumption*.
pub const WAVE_FLOOR_DB: f32 = -80.0;
pub const WAVE_FLOOR_RAMP_DB: f32 = 20.0;
/// Smoothing of the combined gain (s): short enough not to dull transients, long enough to suppress ripple.
pub const WAVE_GAIN_SMOOTH_S: f64 = 0.0005;
/// Output gain (`g`) smoothing (s).
pub const WAVE_OUT_SMOOTH_S: f64 = 0.010;

const WAVE_SLOTS: [(f32, f32, f32); 3] =
    [(-15.0, 15.0, 0.0), (-24.0, 24.0, 0.0), (-18.0, 9.0, 0.0)];

/// `WAVE`: WAVE DESIGNER, see the module docs.
pub struct TransientDesigner {
    fs: f64,
    v: [f32; 3],
    dirty: bool,
    fast_att: f32,
    fast_rel: f32,
    slow_att: f32,
    slow_rel: f32,
    gain_c: f64,
    ef: f32,
    es: f32,
    g_db: f64,
    out: OnePoleSmoother,
    gr_now: f32,
}

impl TransientDesigner {
    pub fn new() -> Self {
        let mut m = Self {
            fs: 48_000.0,
            v: [0.0; 3],
            dirty: true,
            fast_att: 0.0,
            fast_rel: 0.0,
            slow_att: 0.0,
            slow_rel: 0.0,
            gain_c: 0.0,
            ef: 0.0,
            es: 0.0,
            g_db: 0.0,
            out: OnePoleSmoother::new(1.0, WAVE_OUT_SMOOTH_S, 48_000.0),
            gr_now: 0.0,
        };
        m.load_defaults();
        m.apply();
        m
    }

    fn apply(&mut self) {
        if !self.dirty {
            return;
        }
        self.dirty = false;
        let c = |tau: f64| one_pole_coeff(tau, self.fs) as f32;
        self.fast_att = c(WAVE_FAST_ATT_S);
        self.fast_rel = c(WAVE_FAST_REL_S);
        self.slow_att = c(WAVE_SLOW_ATT_S);
        self.slow_rel = c(WAVE_SLOW_REL_S);
        self.gain_c = one_pole_coeff(WAVE_GAIN_SMOOTH_S, self.fs);
        self.out.set_time(WAVE_OUT_SMOOTH_S, self.fs);
        self.out.set_target(db_to_lin_fast(self.v[2]));
    }

    /// Current gain (dB) applied by the attack / sustain regions (before the output `g`).
    pub fn gain_db(&self) -> f32 {
        self.g_db as f32
    }
}

impl Default for TransientDesigner {
    fn default() -> Self {
        Self::new()
    }
}

impl Model for TransientDesigner {
    fn id(&self) -> &'static str {
        "WAVE"
    }

    fn prepare(&mut self, sample_rate: f64, _max_block: usize) {
        self.fs = sample_rate;
        self.dirty = true;
        self.apply();
        self.reset();
    }

    fn process(
        &mut self,
        left: &mut [f32],
        right: &mut [f32],
        sidechain: Option<(&[f32], &[f32])>,
    ) {
        self.apply();
        let n = left.len().min(right.len());
        let (att, sust) = (self.v[0], self.v[1]);
        let off = att == 0.0 && sust == 0.0;
        let inv_ramp = 1.0 / WAVE_FLOOR_RAMP_DB;
        let ia = 1.0 / (WAVE_ATTACK_FULL_DB - WAVE_DEAD_DB);
        let is = 1.0 / (WAVE_SUSTAIN_FULL_DB - WAVE_DEAD_DB);
        let gc = self.gain_c;
        let mut min_g = 0.0f32;
        for i in 0..n {
            let (kl, kr) = match sidechain {
                Some((a, b)) if i < a.len() && i < b.len() => (a[i], b[i]),
                _ => (left[i], right[i]),
            };
            let k = kl.abs().max(kr.abs());
            self.ef = if k > self.ef {
                k + self.fast_att * (self.ef - k)
            } else {
                k + self.fast_rel * (self.ef - k)
            };
            // The slow follower smooths the fast one (not the raw rectified signal), so both sit at the same
            // level for a steady signal and `d` is exactly 0 there.
            let ef = self.ef;
            self.es = if ef > self.es {
                ef + self.slow_att * (self.es - ef)
            } else {
                ef + self.slow_rel * (self.es - ef)
            };
            let target = if off {
                0.0
            } else {
                let ef_db = lin_to_db_fast(self.ef);
                let es_db = lin_to_db_fast(self.es);
                let d = ef_db - es_db;
                let wa = ramp01((d - WAVE_DEAD_DB) * ia);
                let ws = ramp01((-d - WAVE_DEAD_DB) * is);
                let presence = ramp01((ef_db - WAVE_FLOOR_DB) * inv_ramp);
                (att * wa + sust * ws) * presence
            };
            self.g_db = target as f64 + gc * (self.g_db - target as f64);
            let g = db_to_lin_fast(self.g_db as f32) * self.out.tick();
            left[i] *= g;
            right[i] *= g;
            min_g = min_g.min(self.g_db as f32);
        }
        self.gr_now = -min_g;
        if off {
            // keep the smoother converging when the block is empty / settled
            self.g_db *= gc;
        }
    }

    fn reset(&mut self) {
        self.apply();
        self.ef = 0.0;
        self.es = 0.0;
        self.g_db = 0.0;
        self.gr_now = 0.0;
        self.out.reset(self.out.target());
    }

    fn set_param(&mut self, slot: u16, value: ParamValue) {
        if stage(&mut self.v, &WAVE_SLOTS, slot, value) {
            self.dirty = true;
        }
    }

    fn param(&self, slot: u16) -> Option<f32> {
        self.v.get((slot as usize).checked_sub(1)?).copied()
    }

    fn load_defaults(&mut self) {
        for (d, s) in self.v.iter_mut().zip(WAVE_SLOTS) {
            *d = s.2;
        }
        self.dirty = true;
    }

    fn commit(&mut self) {
        self.apply();
    }

    fn telemetry(&self) -> ModelTelemetry {
        ModelTelemetry {
            gain_reduction_db: self.gr_now.max(0.0),
            engaged: self.g_db.abs() > 0.1,
        }
    }
}

/// WING-Edit GR meter for WAVE: constant 1 (`data/reference-curves/dyn-models.json`).
pub fn wave_gr_meter(_gr_db: f32) -> f32 {
    1.0
}

// =============================================================================================================
// RIDE: auto rider
// =============================================================================================================

/// Level detector: mean-square one-pole time constant (s). *Assumption*: Vocal Rider follows phrase-level
/// energy over a few hundred ms; 100 ms follows words, not syllable peaks.
pub const RIDE_LEVEL_TAU_S: f64 = 0.100;
/// Control-rate decimation (samples per level / gain update).
pub const RIDE_CTL: usize = 16;
/// Slew rate at `spd` = 1 and 50 (dB/s, geometric between). *Assumption* (WING gives only the 1..50 step
/// index): 2 dB/s up to 200 dB/s, default `spd` 10 = 4.7 dB/s.
pub const RIDE_RATE_MIN_DB_S: f64 = 2.0;
pub const RIDE_RATE_MAX_DB_S: f64 = 200.0;
/// Downward (gain reduction) slews this much faster than upward. *Assumption* (riders pull loud words down
/// faster than they lift quiet ones, avoiding pumping of the phrase tail).
pub const RIDE_DOWN_FACTOR: f32 = 2.0;
/// Hold values at or above this (s) mean `INF` (the schema range ends at 10 s labelled INF, models.md).
pub const RIDE_HOLD_INF_S: f32 = 9.99;
/// Gain smoothing after the slew limiter (s).
pub const RIDE_GAIN_SMOOTH_S: f64 = 0.020;
/// Ratio enum values (schema `fenum`).
pub const RIDE_RATIOS: [f32; 5] = [2.0, 4.0, 8.0, 20.0, 100.0];

const RIDE_SLOTS: [(f32, f32, f32); 6] = [
    (-54.0, -18.0, -40.0),
    (-48.0, 0.0, -18.0),
    (1.0, 50.0, 10.0),
    (2.0, 100.0, 4.0),
    (0.1, 10.0, 1.5),
    (1.0, 15.0, 6.0),
];

/// Slew rate (dB/s) for the speed knob 1..50.
pub fn ride_rate_db_s(spd: f32) -> f64 {
    let t = ((spd.clamp(1.0, 50.0) - 1.0) / 49.0) as f64;
    RIDE_RATE_MIN_DB_S * (RIDE_RATE_MAX_DB_S / RIDE_RATE_MIN_DB_S).powf(t)
}

/// WING-Edit GR meter for RIDE: negative deflection `-gr/15`, capped at -1 (`dyn-models.json`).
pub fn ride_gr_meter(gr_db: f32) -> f32 {
    -(gr_db.max(0.0) / 15.0).min(1.0)
}

/// `RIDE`: AUTO RIDER, see the module docs.
pub struct AutoRider {
    fs: f64,
    v: [f32; 6],
    dirty: bool,
    ms_c: f32,
    ms: f32,
    gain_db: f32,
    hold_left: f32,
    step_up: f32,
    step_down: f32,
    gain: OnePoleSmoother,
    ctl_phase: usize,
    below: bool,
    gr_now: f32,
}

impl AutoRider {
    pub fn new() -> Self {
        let mut m = Self {
            fs: 48_000.0,
            v: [0.0; 6],
            dirty: true,
            ms_c: 0.0,
            ms: 0.0,
            gain_db: 0.0,
            hold_left: 0.0,
            step_up: 0.0,
            step_down: 0.0,
            gain: OnePoleSmoother::new(1.0, RIDE_GAIN_SMOOTH_S, 48_000.0),
            ctl_phase: 0,
            below: false,
            gr_now: 0.0,
        };
        m.load_defaults();
        m.apply();
        m
    }

    fn apply(&mut self) {
        if !self.dirty {
            return;
        }
        self.dirty = false;
        self.ms_c = one_pole_coeff(RIDE_LEVEL_TAU_S, self.fs) as f32;
        let per_ctl = RIDE_CTL as f64 / self.fs;
        self.step_up = (ride_rate_db_s(self.v[2]) * per_ctl) as f32;
        self.step_down = self.step_up * RIDE_DOWN_FACTOR;
        self.gain.set_time(RIDE_GAIN_SMOOTH_S, self.fs);
    }

    /// Current riding gain in dB (positive = boosting).
    pub fn gain_db(&self) -> f32 {
        self.gain_db
    }

    /// One control update from the detected mean-square.
    fn control(&mut self) {
        let (thr, tgt, ratio, hld, range) = (self.v[0], self.v[1], self.v[3], self.v[4], self.v[5]);
        let level = 0.5 * lin_to_db_fast(self.ms.max(1e-14));
        let per_ctl = (RIDE_CTL as f64 / self.fs) as f32;
        let goal = if level < thr {
            self.below = true;
            if hld >= RIDE_HOLD_INF_S {
                None
            } else if self.hold_left > 0.0 {
                self.hold_left -= per_ctl;
                None
            } else {
                Some(0.0)
            }
        } else {
            self.below = false;
            self.hold_left = hld;
            Some(((tgt - level) * (1.0 - 1.0 / ratio)).clamp(-range, range))
        };
        if let Some(goal) = goal {
            let d = goal - self.gain_db;
            let step = if d > 0.0 {
                self.step_up
            } else {
                self.step_down
            };
            self.gain_db += d.clamp(-step, step);
        }
        self.gain_db = self.gain_db.clamp(-range, range);
        self.gain.set_target(db_to_lin_fast(self.gain_db));
        self.gr_now = (-self.gain_db).max(0.0);
    }
}

impl Default for AutoRider {
    fn default() -> Self {
        Self::new()
    }
}

impl Model for AutoRider {
    fn id(&self) -> &'static str {
        "RIDE"
    }

    fn prepare(&mut self, sample_rate: f64, _max_block: usize) {
        self.fs = sample_rate;
        self.dirty = true;
        self.apply();
        self.reset();
    }

    fn process(
        &mut self,
        left: &mut [f32],
        right: &mut [f32],
        sidechain: Option<(&[f32], &[f32])>,
    ) {
        self.apply();
        let n = left.len().min(right.len());
        let c = self.ms_c;
        for i in 0..n {
            let (kl, kr) = match sidechain {
                Some((a, b)) if i < a.len() && i < b.len() => (a[i], b[i]),
                _ => (left[i], right[i]),
            };
            let p = 0.5 * (kl * kl + kr * kr);
            self.ms = p + c * (self.ms - p);
            self.ctl_phase += 1;
            if self.ctl_phase >= RIDE_CTL {
                self.ctl_phase = 0;
                self.control();
            }
            let g = self.gain.tick();
            left[i] *= g;
            right[i] *= g;
        }
    }

    fn reset(&mut self) {
        self.apply();
        self.ms = 0.0;
        self.gain_db = 0.0;
        self.hold_left = 0.0;
        self.ctl_phase = 0;
        self.below = false;
        self.gr_now = 0.0;
        self.gain.set_target(1.0);
        self.gain.reset(1.0);
    }

    fn set_param(&mut self, slot: u16, value: ParamValue) {
        match slot {
            // `ratio` is a float enum: `Float`/`Int` carry the ratio value, `Enum` the list index.
            4 => {
                let r = match value {
                    ParamValue::Enum(i) => match RIDE_RATIOS.get(i as usize) {
                        Some(&r) => r,
                        None => return,
                    },
                    other => other.as_f32().clamp(2.0, 100.0),
                };
                self.v[3] = r;
                self.dirty = true;
            }
            _ => {
                if stage(&mut self.v, &RIDE_SLOTS, slot, value) {
                    self.dirty = true;
                }
            }
        }
    }

    fn param(&self, slot: u16) -> Option<f32> {
        self.v.get((slot as usize).checked_sub(1)?).copied()
    }

    fn load_defaults(&mut self) {
        for (d, s) in self.v.iter_mut().zip(RIDE_SLOTS) {
            *d = s.2;
        }
        self.dirty = true;
    }

    fn commit(&mut self) {
        self.apply();
    }

    fn telemetry(&self) -> ModelTelemetry {
        ModelTelemetry {
            gain_reduction_db: self.gr_now,
            engaged: !self.below,
        }
    }
}

// =============================================================================================================
// WARM: VHD preamp
// =============================================================================================================

/// Drive: pre-gain of the shaper, exponential in the knob (schema 10..125 %, default 33):
/// `g = WARM_DRIVE_G_DEFAULT * (WARM_DRIVE_G_MAX / WARM_DRIVE_G_DEFAULT)^((drv - 33) / 92)`, i.e. 0.27 at 10 %,
/// 0.5 at the default and 6 (+21.6 dB over the default) at 125 %. *Assumption*: the default keeps the T049
/// calibration (~1.4 % THD at -12 dBFS, 1 kHz); the top of the range must saturate programme at typical
/// channel levels (-12 dBFS: ~14 % THD; -30 dBFS: still > 1 %), as a preamp drive does. T081: the
/// former linear law `1.5 * drv/100` topped out at 1.9, which left drive inaudible below ~-15 dBFS.
pub const WARM_DRIVE_G_DEFAULT: f32 = 0.5;
pub const WARM_DRIVE_G_MAX: f32 = 6.0;
/// Drive knob default and span above it (%), the anchors of the exponential law.
const WARM_DRIVE_REF: f32 = 33.0;
const WARM_DRIVE_SPAN: f32 = 92.0;
/// Largest shaper bias `T = tanh(b)` (at `hrm = -100`). 0.577 would cancel the 3rd harmonic exactly
/// (`1 - 3 T^2 = 0`); 80 % of it keeps a little 3rd. *Assumption*.
pub const WARM_T_MAX: f32 = 0.46;
/// Colour: pre-tilt in dB (HF minus LF) at `col = +1`, centre frequency, and the share of it that the post
/// tilt takes back (the rest remains as a net spectral tilt). *Assumption*.
pub const WARM_TILT_DB: f64 = 6.0;
pub const WARM_TILT_HZ: f64 = 1000.0;
pub const WARM_TILT_RESTORE: f64 = 0.5;
/// DC blocker corner (Hz) removing the offset of the even-order terms.
pub const WARM_DC_HZ: f64 = 3.0;
/// Parameter smoothing (s).
pub const WARM_SMOOTH_S: f64 = 0.010;
/// ADAA fallback: use the direct curve when consecutive inputs differ by less than this.
const ADAA_EPS: f64 = 1e-6;
const WARM_OS: usize = 2;
/// Oversampler round-trip latency (samples) = dry path delay.
const WARM_LATENCY: usize = 39;

const WARM_SLOTS: [(f32, f32, f32); 5] = [
    (10.0, 125.0, 33.0),
    (-100.0, 100.0, 0.0),
    (-1.0, 1.0, 0.0),
    (-18.0, 6.0, 0.0),
    (0.0, 100.0, 100.0),
];

/// Shaper pre-gain for the drive knob `drv` (%), see [`WARM_DRIVE_G_MAX`].
#[inline]
pub fn warm_drive_gain(drv: f32) -> f32 {
    WARM_DRIVE_G_DEFAULT
        * (WARM_DRIVE_G_MAX / WARM_DRIVE_G_DEFAULT).powf((drv - WARM_DRIVE_REF) / WARM_DRIVE_SPAN)
}

/// First-order section `g0 (1 + s/wz) / (1 + s/wp)` (bilinear, each corner prewarped), f64 TDF-II.
#[derive(Clone, Copy, Debug, Default)]
struct Shelf1 {
    b0: f64,
    b1: f64,
    a1: f64,
    z: f64,
}

impl Shelf1 {
    /// Tilt with HF/LF gain ratio `ratio_db` (dB), unity at `f0`.
    fn tilt(fs: f64, f0: f64, ratio_db: f64) -> Self {
        let r = 10f64.powf(ratio_db / 20.0);
        let sr = r.sqrt();
        let g0 = 1.0 / sr;
        let fz = (f0 / sr).min(0.49 * fs);
        let fp = (f0 * sr).min(0.49 * fs);
        let kz = (core::f64::consts::PI * fz / fs).tan();
        let kp = (core::f64::consts::PI * fp / fs).tan();
        let a0 = 1.0 + 1.0 / kp;
        Self {
            b0: g0 * (1.0 + 1.0 / kz) / a0,
            b1: g0 * (1.0 - 1.0 / kz) / a0,
            a1: (1.0 - 1.0 / kp) / a0,
            z: 0.0,
        }
    }

    fn set_coeffs(&mut self, o: Shelf1) {
        self.b0 = o.b0;
        self.b1 = o.b1;
        self.a1 = o.a1;
    }

    #[inline(always)]
    fn tick(&mut self, x: f64) -> f64 {
        let y = self.b0 * x + self.z;
        self.z = self.b1 * x - self.a1 * y;
        y
    }
}

/// `ln cosh x` (stable for large |x|).
#[inline(always)]
fn ln_cosh(x: f64) -> f64 {
    let a = x.abs();
    a + (-2.0 * a).exp().ln_1p() - core::f64::consts::LN_2
}

/// Per-channel processing state.
struct WarmChannel {
    os: Oversampler2x,
    pre: Shelf1,
    post: Shelf1,
    prev_u: f64,
    prev_f: f64,
    dc_x: f64,
    dc_y: f64,
    dry: [f32; WARM_LATENCY + 1],
    dry_pos: usize,
}

impl WarmChannel {
    fn new(max_block: usize) -> Self {
        Self {
            os: Oversampler2x::new(max_block),
            pre: Shelf1::default(),
            post: Shelf1::default(),
            prev_u: 0.0,
            prev_f: 0.0,
            dc_x: 0.0,
            dc_y: 0.0,
            dry: [0.0; WARM_LATENCY + 1],
            dry_pos: 0,
        }
    }

    fn reset(&mut self) {
        self.os.reset();
        self.pre.z = 0.0;
        self.post.z = 0.0;
        self.prev_u = 0.0;
        self.prev_f = 0.0;
        self.dc_x = 0.0;
        self.dc_y = 0.0;
        self.dry = [0.0; WARM_LATENCY + 1];
        self.dry_pos = 0;
    }
}

/// Parameters of the shaper for one chunk.
#[derive(Clone, Copy)]
struct Shaper {
    g: f64,
    b: f64,
    tb: f64,
    /// Small-signal slope `1 - T^2` of `tanh(u + b) - tanh(b)`.
    c1: f64,
    norm: f64,
}

/// `WARM`: SOUL WARMTH PRE, see the module docs.
pub struct WarmthPre {
    fs: f64,
    max_block: usize,
    v: [f32; 5],
    dirty: bool,
    ch: [WarmChannel; 2],
    drv: OnePoleSmoother,
    hrm: OnePoleSmoother,
    col: OnePoleSmoother,
    trim: OnePoleSmoother,
    mix: OnePoleSmoother,
    dc_r: f64,
    hi: [Vec<f32>; 2],
    dry_s: [Vec<f32>; 2],
    mix_s: Vec<f32>,
    trim_s: Vec<f32>,
}

impl WarmthPre {
    pub fn new() -> Self {
        let sm = |v: f32| OnePoleSmoother::new(v, WARM_SMOOTH_S, 48_000.0);
        let mut m = Self {
            fs: 48_000.0,
            max_block: 0,
            v: [0.0; 5],
            dirty: true,
            ch: [WarmChannel::new(1), WarmChannel::new(1)],
            drv: sm(0.0),
            hrm: sm(0.0),
            col: sm(0.0),
            trim: sm(1.0),
            mix: sm(1.0),
            dc_r: 0.0,
            hi: [Vec::new(), Vec::new()],
            dry_s: [Vec::new(), Vec::new()],
            mix_s: Vec::new(),
            trim_s: Vec::new(),
        };
        m.load_defaults();
        m
    }

    fn apply(&mut self) {
        if !self.dirty {
            return;
        }
        self.dirty = false;
        for s in [
            &mut self.drv,
            &mut self.hrm,
            &mut self.col,
            &mut self.trim,
            &mut self.mix,
        ] {
            s.set_time(WARM_SMOOTH_S, self.fs);
        }
        self.drv.set_target(self.v[0]);
        self.hrm.set_target(self.v[1]);
        self.col.set_target(self.v[2]);
        self.trim.set_target(db_to_lin_fast(self.v[3]));
        self.mix.set_target(self.v[4] * 0.01);
        self.dc_r = 1.0 - 2.0 * core::f64::consts::PI * WARM_DC_HZ / self.fs;
    }

    fn shaper(drv: f32, hrm: f32) -> Shaper {
        let g = warm_drive_gain(drv) as f64;
        let t = WARM_T_MAX as f64 * (1.0 - (hrm * 0.01) as f64) * 0.5;
        let b = t.atanh();
        Shaper {
            g,
            b,
            tb: t,
            c1: 1.0 - t * t,
            norm: 1.0 / ((1.0 - t * t) * g),
        }
    }

    fn set_tilts(&mut self, col: f32) {
        let pre = Shelf1::tilt(self.fs, WARM_TILT_HZ, col as f64 * WARM_TILT_DB);
        let post = Shelf1::tilt(
            self.fs,
            WARM_TILT_HZ,
            -(col as f64) * WARM_TILT_DB * WARM_TILT_RESTORE,
        );
        for c in &mut self.ch {
            c.pre.set_coeffs(pre);
            c.post.set_coeffs(post);
        }
    }

    fn process_chunk(&mut self, left: &mut [f32], right: &mut [f32]) {
        let n = left.len();
        let drv = self.drv.advance(n);
        let hrm = self.hrm.advance(n);
        let col = self.col.advance(n);
        self.set_tilts(col);
        let sh = Self::shaper(drv, hrm);
        self.trim.fill(&mut self.trim_s[..n]);
        self.mix.fill(&mut self.mix_s[..n]);
        let dc_r = self.dc_r;
        for (c, buf) in [left, right].into_iter().enumerate() {
            let ch = &mut self.ch[c];
            // dry path, delayed by the oversampler latency
            for (i, x) in buf.iter().enumerate() {
                ch.dry[ch.dry_pos] = *x;
                let rd = (ch.dry_pos + 1) % (WARM_LATENCY + 1);
                self.dry_s[c][i] = ch.dry[rd];
                ch.dry_pos = rd;
            }
            for x in buf.iter_mut() {
                *x = ch.pre.tick(*x as f64) as f32;
            }
            let hi = &mut self.hi[c][..WARM_OS * n];
            ch.os.upsample(buf, hi);
            let (mut pu, mut pf) = (ch.prev_u, ch.prev_f);
            for s in hi.iter_mut() {
                // Linear part passes untouched (exact unity, no ADAA half-sample low-pass); only the
                // nonlinear remainder r(u) = f(u) - f'(0) u goes through first-order ADAA.
                let u = sh.g * *s as f64;
                let f = ln_cosh(u + sh.b) - sh.tb * u - 0.5 * sh.c1 * u * u;
                let du = u - pu;
                let r = if du.abs() < ADAA_EPS {
                    let m = 0.5 * (u + pu);
                    (m + sh.b).tanh() - sh.tb - sh.c1 * m
                } else {
                    (f - pf) / du
                };
                pu = u;
                pf = f;
                *s += (r * sh.norm) as f32;
            }
            ch.prev_u = pu;
            ch.prev_f = pf;
            ch.os.downsample(hi, buf);
            for (i, x) in buf.iter_mut().enumerate() {
                let y = ch.post.tick(*x as f64);
                let o = y - ch.dc_x + dc_r * ch.dc_y;
                ch.dc_x = y;
                ch.dc_y = o;
                let wet = o as f32;
                let m = self.mix_s[i];
                *x = (wet * m + self.dry_s[c][i] * (1.0 - m)) * self.trim_s[i];
            }
        }
    }
}

impl Default for WarmthPre {
    fn default() -> Self {
        Self::new()
    }
}

impl Model for WarmthPre {
    fn id(&self) -> &'static str {
        "WARM"
    }

    /// `wmix` is WARM's MIX: the hosting slot does not blend with its block `mix` again (T085).
    fn owns_mix(&self) -> bool {
        true
    }

    fn prepare(&mut self, sample_rate: f64, max_block: usize) {
        self.fs = sample_rate;
        self.max_block = max_block.max(1);
        let mb = self.max_block;
        self.ch = [WarmChannel::new(mb), WarmChannel::new(mb)];
        self.hi = [vec![0.0; WARM_OS * mb], vec![0.0; WARM_OS * mb]];
        self.dry_s = [vec![0.0; mb], vec![0.0; mb]];
        self.mix_s = vec![0.0; mb];
        self.trim_s = vec![0.0; mb];
        self.dirty = true;
        self.apply();
        self.reset();
    }

    fn process(
        &mut self,
        left: &mut [f32],
        right: &mut [f32],
        _sidechain: Option<(&[f32], &[f32])>,
    ) {
        self.apply();
        let n = left.len().min(right.len());
        let mb = self.max_block;
        let mut i = 0;
        while i < n {
            let e = (i + mb).min(n);
            self.process_chunk(&mut left[i..e], &mut right[i..e]);
            i = e;
        }
    }

    fn latency(&self) -> usize {
        WARM_LATENCY
    }

    fn reset(&mut self) {
        self.apply();
        for c in &mut self.ch {
            c.reset();
        }
        for s in [
            &mut self.drv,
            &mut self.hrm,
            &mut self.col,
            &mut self.trim,
            &mut self.mix,
        ] {
            s.reset(s.target());
        }
        let col = self.col.value();
        self.set_tilts(col);
    }

    fn set_param(&mut self, slot: u16, value: ParamValue) {
        if stage(&mut self.v, &WARM_SLOTS, slot, value) {
            self.dirty = true;
        }
    }

    fn param(&self, slot: u16) -> Option<f32> {
        self.v.get((slot as usize).checked_sub(1)?).copied()
    }

    fn load_defaults(&mut self) {
        for (d, s) in self.v.iter_mut().zip(WARM_SLOTS) {
            *d = s.2;
        }
        self.dirty = true;
    }

    fn commit(&mut self) {
        self.apply();
    }
}

#[cfg(test)]
mod tests;
