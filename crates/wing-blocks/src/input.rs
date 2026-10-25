//! Channel input stage (models.md §2.1, signal-flow.md §2): source gain + trim, stereo balance, polarity
//! invert, input delay and mono / stereo / mid-side source handling.
//!
//! Processing order (signal-flow.md §2, `in.set`): source mode decode -> gain (`g` + `trim`) -> balance ->
//! polarity -> delay. Channels are stereo internally; a mono source feeds both sides.
//!
//! Smoothing: per-side linear gains (gain, trim, balance and polarity folded into one factor) follow a
//! 5 ms one-pole, so polarity flips cross through zero instead of clicking. The delay time follows a 50 ms
//! one-pole in samples and is read with linear interpolation (a delay change glides; settled integer delays
//! are exact). Delay off = target delay 0 (glides back to a pass-through).
//!
//! Assumptions (**verify** in T072): balance attenuates the opposite side only (`bal > 0` turns the left side
//! down by `bal` dB); mid-side decode is `L = M + S`, `R = M - S` (no 1/2 scaling); speed of sound 343 m/s.

use wing_dsp::gain::db_to_lin;
use wing_dsp::{DelayLine, OnePoleSmoother};

/// Speed of sound used for distance delay modes (m/s).
pub const SPEED_OF_SOUND: f64 = 343.0;
/// Longest input delay in any mode: 500 ms (MS mode; 150 m = 437 ms, 500 ft = 444 ms).
pub const MAX_DELAY_MS: f64 = 500.0;
const GAIN_TAU_S: f64 = 0.005;
const DELAY_TAU_S: f64 = 0.05;

/// Source channel mode (`io.in.<grp>.<n>.mode` / `in.set.$mode`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum SourceMode {
    /// Mono source: the left input feeds both sides.
    #[default]
    Mono,
    /// Stereo pair, passed as-is.
    Stereo,
    /// Mid (left input) / side (right input) pair, decoded to L/R.
    MidSide,
}

/// Input delay unit (`in.set.dlymode`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum DelayMode {
    /// Metres, 0.1 .. 150.
    #[default]
    Meters,
    /// Feet, 0.5 .. 500.
    Feet,
    /// Milliseconds, 0.5 .. 500.
    Ms,
    /// Samples, 16 .. 500.
    Samples,
}

impl DelayMode {
    /// Valid range of `dly` in this mode (models.md §2.1).
    pub fn range(self) -> (f32, f32) {
        match self {
            DelayMode::Meters => (0.1, 150.0),
            DelayMode::Feet => (0.5, 500.0),
            DelayMode::Ms => (0.5, 500.0),
            DelayMode::Samples => (16.0, 500.0),
        }
    }

    /// Convert a delay value in this mode to samples at `fs`.
    pub fn to_samples(self, value: f32, fs: f64) -> f64 {
        let (lo, hi) = self.range();
        let v = value.clamp(lo, hi) as f64;
        match self {
            DelayMode::Meters => v / SPEED_OF_SOUND * fs,
            DelayMode::Feet => v * 0.3048 / SPEED_OF_SOUND * fs,
            DelayMode::Ms => v * 1e-3 * fs,
            DelayMode::Samples => v,
        }
    }
}

/// Input stage parameters in engineering units.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct InputParams {
    /// Source preamp gain `io.in.*.g`, -3 .. +45.5 dB (a pure digital gain in the simulator).
    pub gain_db: f32,
    /// Channel trim `in.set.trim`, -18 .. +18 dB.
    pub trim_db: f32,
    /// Stereo balance `in.set.bal`, -9 .. +9 dB.
    pub balance_db: f32,
    /// Polarity invert `in.set.inv`.
    pub invert: bool,
    /// Input delay on `in.set.dlyon`.
    pub delay_on: bool,
    /// Input delay amount `in.set.dly` in `delay_mode` units.
    pub delay: f32,
    /// `in.set.dlymode`.
    pub delay_mode: DelayMode,
    /// Source mode M / ST / M-S.
    pub source_mode: SourceMode,
}

impl Default for InputParams {
    fn default() -> Self {
        Self {
            gain_db: 0.0,
            trim_db: 0.0,
            balance_db: 0.0,
            invert: false,
            delay_on: false,
            delay: 0.1,
            delay_mode: DelayMode::Meters,
            source_mode: SourceMode::Mono,
        }
    }
}

impl InputParams {
    /// Clamp every field to its documented range.
    pub fn clamped(mut self) -> Self {
        self.gain_db = self.gain_db.clamp(-3.0, 45.5);
        self.trim_db = self.trim_db.clamp(-18.0, 18.0);
        self.balance_db = self.balance_db.clamp(-9.0, 9.0);
        let (lo, hi) = self.delay_mode.range();
        self.delay = self.delay.clamp(lo, hi);
        self
    }

    /// Target linear gains (left, right) including polarity.
    pub fn side_gains(&self) -> (f32, f32) {
        let p = self.clamped();
        let g = db_to_lin(p.gain_db + p.trim_db) * if p.invert { -1.0 } else { 1.0 };
        let l = if p.balance_db > 0.0 {
            db_to_lin(-p.balance_db)
        } else {
            1.0
        };
        let r = if p.balance_db < 0.0 {
            db_to_lin(p.balance_db)
        } else {
            1.0
        };
        (g * l, g * r)
    }
}

/// Channel input stage. Call [`InputStage::prepare`] before processing.
pub struct InputStage {
    params: InputParams,
    fs: f64,
    gain_l: OnePoleSmoother,
    gain_r: OnePoleSmoother,
    delay_smp: OnePoleSmoother,
    dl_l: DelayLine,
    dl_r: DelayLine,
}

impl Default for InputStage {
    fn default() -> Self {
        Self::new()
    }
}

impl InputStage {
    pub fn new() -> Self {
        Self {
            params: InputParams::default(),
            fs: 48_000.0,
            gain_l: OnePoleSmoother::new(1.0, GAIN_TAU_S, 48_000.0),
            gain_r: OnePoleSmoother::new(1.0, GAIN_TAU_S, 48_000.0),
            delay_smp: OnePoleSmoother::new(0.0, DELAY_TAU_S, 48_000.0),
            dl_l: DelayLine::new(1),
            dl_r: DelayLine::new(1),
        }
    }

    /// Allocate delay lines for `sample_rate` and reset state. `max_block` is unused (no scratch needed).
    pub fn prepare(&mut self, sample_rate: f64, _max_block: usize) {
        self.fs = sample_rate;
        let max = (MAX_DELAY_MS * 1e-3 * sample_rate).ceil() as usize + 2;
        self.dl_l = DelayLine::new(max);
        self.dl_r = DelayLine::new(max);
        self.gain_l.set_time(GAIN_TAU_S, sample_rate);
        self.gain_r.set_time(GAIN_TAU_S, sample_rate);
        self.delay_smp.set_time(DELAY_TAU_S, sample_rate);
        let p = self.params;
        self.set_params(&p);
        self.reset();
    }

    /// Jump smoothers to their targets and clear the delay lines.
    pub fn reset(&mut self) {
        self.gain_l.reset(self.gain_l.target());
        self.gain_r.reset(self.gain_r.target());
        self.delay_smp.reset(self.delay_smp.target());
        self.dl_l.reset();
        self.dl_r.reset();
    }

    pub fn params(&self) -> &InputParams {
        &self.params
    }

    /// Update parameters (RT-safe, no allocation). Values are clamped.
    pub fn set_params(&mut self, p: &InputParams) {
        self.params = p.clamped();
        let (l, r) = self.params.side_gains();
        self.gain_l.set_target(l);
        self.gain_r.set_target(r);
        let d = if self.params.delay_on {
            self.params
                .delay_mode
                .to_samples(self.params.delay, self.fs)
                .min(self.dl_l.max_delay() as f64)
        } else {
            0.0
        };
        self.delay_smp.set_target(d as f32);
    }

    /// Current delay in samples (smoothed value).
    pub fn delay_samples(&self) -> f32 {
        self.delay_smp.value()
    }

    /// Process one block in place. `left`/`right` must have equal length. In mono mode the right input is
    /// ignored and overwritten.
    pub fn process(&mut self, left: &mut [f32], right: &mut [f32]) {
        debug_assert_eq!(left.len(), right.len());
        match self.params.source_mode {
            SourceMode::Mono => right.copy_from_slice(left),
            SourceMode::Stereo => {}
            SourceMode::MidSide => {
                for (l, r) in left.iter_mut().zip(right.iter_mut()) {
                    let (m, s) = (*l, *r);
                    *l = m + s;
                    *r = m - s;
                }
            }
        }
        let delay_idle = self.delay_smp.target() == 0.0 && self.delay_smp.is_settled(1e-4);
        for (l, r) in left.iter_mut().zip(right.iter_mut()) {
            let xl = *l * self.gain_l.tick();
            let xr = *r * self.gain_r.tick();
            if delay_idle {
                // Keep the delay lines primed so switching the delay on never reads stale audio.
                self.dl_l.push(xl);
                self.dl_r.push(xr);
                *l = xl;
                *r = xr;
            } else {
                let d = self.delay_smp.tick();
                self.dl_l.push(xl);
                self.dl_r.push(xr);
                *l = self.dl_l.read_linear(d);
                *r = self.dl_r.read_linear(d);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FS: f64 = 48_000.0;

    fn settled(p: InputParams) -> InputStage {
        let mut s = InputStage::new();
        s.set_params(&p);
        s.prepare(FS, 64);
        s
    }

    #[test]
    fn gain_trim_polarity_balance() {
        let p = InputParams {
            gain_db: 12.0,
            trim_db: -6.0,
            balance_db: 3.0,
            invert: true,
            source_mode: SourceMode::Stereo,
            ..Default::default()
        };
        let mut s = settled(p);
        let mut l = [0.5f32; 64];
        let mut r = [0.5f32; 64];
        s.process(&mut l, &mut r);
        let g = db_to_lin(6.0);
        assert!(
            (l[63] - (-0.5 * g * db_to_lin(-3.0))).abs() < 1e-5,
            "{}",
            l[63]
        );
        assert!((r[63] - (-0.5 * g)).abs() < 1e-5);
    }

    #[test]
    fn source_modes() {
        let mut s = settled(InputParams::default());
        let mut l = [0.25f32; 8];
        let mut r = [9.0f32; 8];
        s.process(&mut l, &mut r);
        assert_eq!(l, r);
        let mut s = settled(InputParams {
            source_mode: SourceMode::MidSide,
            ..Default::default()
        });
        let mut l = [0.5f32; 8];
        let mut r = [0.25f32; 8];
        s.process(&mut l, &mut r);
        assert!((l[7] - 0.75).abs() < 1e-6 && (r[7] - 0.25).abs() < 1e-6);
    }

    #[test]
    fn delay_modes_and_impulse() {
        assert!((DelayMode::Ms.to_samples(10.0, FS) - 480.0).abs() < 1e-9);
        assert!((DelayMode::Meters.to_samples(3.43, FS) - 480.0).abs() < 1e-3);
        assert!(
            (DelayMode::Feet.to_samples(1000.0, FS) - DelayMode::Feet.to_samples(500.0, FS)).abs()
                < 1e-9
        );
        assert_eq!(DelayMode::Samples.to_samples(1.0, FS), 16.0);
        let mut s = settled(InputParams {
            delay_on: true,
            delay: 100.0,
            delay_mode: DelayMode::Samples,
            source_mode: SourceMode::Stereo,
            ..Default::default()
        });
        let mut l = vec![0.0f32; 256];
        let mut r = vec![0.0f32; 256];
        l[3] = 1.0;
        r[5] = -1.0;
        s.process(&mut l, &mut r);
        assert!((l[103] - 1.0).abs() < 1e-6 && (r[105] + 1.0).abs() < 1e-6);
        assert_eq!(l.iter().filter(|x| x.abs() > 1e-6).count(), 1);
        // Max delay (500 ms) fits.
        let mut s = settled(InputParams {
            delay_on: true,
            delay: 500.0,
            delay_mode: DelayMode::Ms,
            ..Default::default()
        });
        assert!((s.delay_samples() - 24_000.0).abs() < 1e-3);
        let mut l = vec![0.0f32; 24_064];
        let mut r = vec![0.0f32; 24_064];
        l[0] = 1.0;
        s.process(&mut l, &mut r);
        assert!((l[24_000] - 1.0).abs() < 1e-6);
    }

    #[test]
    fn gain_change_is_smoothed() {
        let mut s = settled(InputParams::default());
        s.set_params(&InputParams {
            invert: true,
            ..Default::default()
        });
        let mut l = [1.0f32; 2400];
        let mut r = [1.0f32; 2400];
        s.process(&mut l, &mut r);
        // No sample-to-sample jump larger than the 5 ms one-pole allows (~2/240 per sample).
        let max_step = l
            .windows(2)
            .map(|w| (w[1] - w[0]).abs())
            .fold(0.0, f32::max);
        assert!(max_step < 0.01, "step {max_step}");
        assert!(l[2399] < -0.99);
    }
}
