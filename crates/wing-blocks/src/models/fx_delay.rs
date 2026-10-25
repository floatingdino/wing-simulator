//! FX rack delay models (T050 proof model; the rest of the delay family is T05D): `ST-DL` (STEREO DELAY,
//! models.md §6).
//!
//! The model outputs the **wet** signal only; the FX slot host blends it with the dry signal by `fxmix`
//! (wing-engine `FxSlot`), like every FX rack model.
//!
//! ```text
//!  in L/R --(mode M: (L+R)/2 to both)--> + --> delay L/R --> LO CUT -> HI CUT --> wet out
//!                                        ^          |
//!                                        +-- feed * FEED LCUT (HP1) * FEED HCUT (LP1) <--+
//!                                            (mode X: L line feeds R and R feeds L = ping-pong)
//! ```
//!
//! Parameters (schema slots, `fx/N/1..10`): `time` 1..3000 ms, `mode` ST/X/M, `fact` (11 note factors),
//! `pat` (9 L:R ratios), `offs` -50..+50 ms, `feed` 0..100 %, `flc`/`fhc` feedback filters, `lc`/`hc` output
//! filters. Mapping [I] (no console measurement yet): `L = time * fact * patL`, `R = time * fact * patR`;
//! a positive offset delays R, a negative one delays L. Delay times glide (one-pole, [`GLIDE_S`]) with
//! linear-interpolated reads, so time changes pitch-bend like a digital delay instead of clicking. The longest
//! line is [`MAX_DELAY_S`] (the effective time is clamped to it, [I]).

use crate::model::{Model, ParamValue};
use wing_dsp::biquad::{Biquad, Coeffs};
use wing_dsp::delay::DelayLine;

/// Longest effective delay per side (TIME range plus offset), seconds.
pub const MAX_DELAY_S: f64 = 3.05;
/// Delay-time glide time constant.
pub const GLIDE_S: f64 = 0.05;
/// Maximum loop gain (keeps FEED 100 % below self-oscillation).
const MAX_FEED: f32 = 0.99;

/// `fact` tokens in schema order.
pub const FACTORS: [f32; 11] = [
    0.25,
    1.0 / 3.0,
    0.5,
    2.0 / 3.0,
    0.75,
    1.0,
    0.375,
    1.25,
    4.0 / 3.0,
    1.5,
    2.0,
];

/// `pat` tokens (L:R) in schema order.
pub const PATTERNS: [(f32, f32); 9] = [
    (0.5, 1.0),
    (2.0 / 3.0, 1.0),
    (0.75, 1.0),
    (0.875, 1.0),
    (1.0, 1.0),
    (1.0, 1.125),
    (1.0, 1.25),
    (1.0, 4.0 / 3.0),
    (1.0, 1.5),
];

/// Schema slots.
pub mod slot {
    pub const TIME: u16 = 1;
    pub const MODE: u16 = 2;
    pub const FACT: u16 = 3;
    pub const PAT: u16 = 4;
    pub const OFFS: u16 = 5;
    pub const FEED: u16 = 6;
    pub const FLC: u16 = 7;
    pub const FHC: u16 = 8;
    pub const LC: u16 = 9;
    pub const HC: u16 = 10;
}

/// `mode`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DelayMode {
    /// Two independent lines.
    Stereo,
    /// Cross feedback (ping-pong).
    Cross,
    /// Mono input to both lines.
    Mono,
}

/// Parameters in engineering units.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StereoDelayParams {
    pub time_ms: f32,
    pub mode: DelayMode,
    /// Index into [`FACTORS`].
    pub fact: usize,
    /// Index into [`PATTERNS`].
    pub pat: usize,
    pub offs_ms: f32,
    pub feed_pct: f32,
    pub flc_hz: f32,
    pub fhc_hz: f32,
    pub lc_hz: f32,
    pub hc_hz: f32,
}

impl Default for StereoDelayParams {
    /// WING-Edit defaults (schema): 333 ms, ST, 1, 1:1, 0 ms, 35 %, 101 Hz / 8 kHz, 101 Hz / 9.6 kHz.
    fn default() -> Self {
        Self {
            time_ms: 333.0,
            mode: DelayMode::Stereo,
            fact: 5,
            pat: 4,
            offs_ms: 0.0,
            feed_pct: 35.0,
            flc_hz: 101.0,
            fhc_hz: 8000.0,
            lc_hz: 101.0,
            hc_hz: 9600.0,
        }
    }
}

impl StereoDelayParams {
    /// Effective (L, R) delays in ms before clamping.
    pub fn delays_ms(&self) -> (f32, f32) {
        let base = self.time_ms.clamp(1.0, 3000.0) * FACTORS[self.fact.min(FACTORS.len() - 1)];
        let (pl, pr) = PATTERNS[self.pat.min(PATTERNS.len() - 1)];
        let (mut l, mut r) = (base * pl, base * pr);
        let o = self.offs_ms.clamp(-50.0, 50.0);
        if o >= 0.0 {
            r += o;
        } else {
            l -= o;
        }
        (l, r)
    }
}

/// STEREO DELAY (`ST-DL`); see the module doc.
pub struct StereoDelay {
    p: StereoDelayParams,
    dirty: bool,
    fs: f64,
    lines: [DelayLine; 2],
    /// Current / target delay in samples per side.
    d: [f32; 2],
    d_target: [f32; 2],
    glide: f32,
    feed: f32,
    fb_hp: [Biquad; 2],
    fb_lp: [Biquad; 2],
    out_hp: [Biquad; 2],
    out_lp: [Biquad; 2],
}

impl Default for StereoDelay {
    fn default() -> Self {
        Self::new()
    }
}

impl StereoDelay {
    /// Unprepared instance (call [`Model::prepare`] before processing).
    pub fn new() -> Self {
        Self {
            p: StereoDelayParams::default(),
            dirty: true,
            fs: 48_000.0,
            lines: [DelayLine::new(1), DelayLine::new(1)],
            d: [1.0; 2],
            d_target: [1.0; 2],
            glide: 1.0,
            feed: 0.0,
            fb_hp: [Biquad::default(); 2],
            fb_lp: [Biquad::default(); 2],
            out_hp: [Biquad::default(); 2],
            out_lp: [Biquad::default(); 2],
        }
    }

    pub fn params(&self) -> &StereoDelayParams {
        &self.p
    }

    /// Set every parameter (staged; applied at the next `commit`/`process`).
    pub fn set_params(&mut self, p: &StereoDelayParams) {
        self.p = *p;
        self.dirty = true;
    }

    /// Current target delays in samples (L, R).
    pub fn delay_samples(&self) -> (f32, f32) {
        (self.d_target[0], self.d_target[1])
    }

    fn max_delay(&self) -> f32 {
        self.lines[0].max_delay() as f32
    }

    fn update(&mut self) {
        self.dirty = false;
        let fs = self.fs;
        let (l, r) = self.p.delays_ms();
        let max = self.max_delay();
        self.d_target = [
            (l * 0.001 * fs as f32).clamp(1.0, max),
            (r * 0.001 * fs as f32).clamp(1.0, max),
        ];
        self.feed = (self.p.feed_pct.clamp(0.0, 100.0) * 0.01).min(MAX_FEED);
        let nyq = (fs * 0.45) as f32;
        let f = |v: f32, lo: f32, hi: f32| v.clamp(lo, hi.min(nyq)) as f64;
        let hp1 = Coeffs::highpass1(fs, f(self.p.flc_hz, 20.0, 400.0));
        let lp1 = Coeffs::lowpass1(fs, f(self.p.fhc_hz, 200.0, 20_000.0));
        let q = core::f64::consts::FRAC_1_SQRT_2;
        let hp = Coeffs::highpass(fs, f(self.p.lc_hz, 20.0, 400.0), q);
        let lp = Coeffs::lowpass(fs, f(self.p.hc_hz, 200.0, 20_000.0), q);
        for k in 0..2 {
            self.fb_hp[k].set_coeffs(hp1);
            self.fb_lp[k].set_coeffs(lp1);
            self.out_hp[k].set_coeffs(hp);
            self.out_lp[k].set_coeffs(lp);
        }
    }
}

impl Model for StereoDelay {
    fn id(&self) -> &'static str {
        "ST-DL"
    }

    fn prepare(&mut self, sample_rate: f64, _max_block: usize) {
        self.fs = sample_rate;
        let n = (MAX_DELAY_S * sample_rate).ceil() as usize + 2;
        self.lines = [DelayLine::new(n), DelayLine::new(n)];
        self.glide = (1.0 - (-1.0 / (GLIDE_S * sample_rate)).exp()) as f32;
        self.update();
        self.reset();
    }

    fn process(&mut self, left: &mut [f32], right: &mut [f32], _sc: Option<(&[f32], &[f32])>) {
        if self.dirty {
            self.update();
        }
        let (mode, g, a) = (self.p.mode, self.feed, self.glide);
        for (xl, xr) in left.iter_mut().zip(right.iter_mut()) {
            let (il, ir) = match mode {
                DelayMode::Mono => {
                    let m = 0.5 * (*xl + *xr);
                    (m, m)
                }
                _ => (*xl, *xr),
            };
            for k in 0..2 {
                self.d[k] += a * (self.d_target[k] - self.d[k]);
            }
            // read before the push: delay d = read position d - 1
            let yl = self.lines[0].read_linear(self.d[0] - 1.0);
            let yr = self.lines[1].read_linear(self.d[1] - 1.0);
            let fl = self.fb_lp[0].tick(self.fb_hp[0].tick(yl));
            let fr = self.fb_lp[1].tick(self.fb_hp[1].tick(yr));
            let (pl, pr) = match mode {
                DelayMode::Cross => (il + g * fr, ir + g * fl),
                _ => (il + g * fl, ir + g * fr),
            };
            self.lines[0].push(pl);
            self.lines[1].push(pr);
            *xl = self.out_lp[0].tick(self.out_hp[0].tick(yl));
            *xr = self.out_lp[1].tick(self.out_hp[1].tick(yr));
        }
        for b in self
            .fb_hp
            .iter_mut()
            .chain(self.fb_lp.iter_mut())
            .chain(self.out_hp.iter_mut())
            .chain(self.out_lp.iter_mut())
        {
            b.flush_denormals();
        }
    }

    fn reset(&mut self) {
        if self.dirty {
            self.update();
        }
        for l in &mut self.lines {
            l.reset();
        }
        self.d = self.d_target;
        for b in self
            .fb_hp
            .iter_mut()
            .chain(self.fb_lp.iter_mut())
            .chain(self.out_hp.iter_mut())
            .chain(self.out_lp.iter_mut())
        {
            b.reset();
        }
    }

    fn set_param(&mut self, slot: u16, v: ParamValue) {
        let p = &mut self.p;
        match slot {
            slot::TIME => p.time_ms = v.as_f32(),
            slot::MODE => {
                p.mode = match v.as_index() {
                    0 => DelayMode::Stereo,
                    1 => DelayMode::Cross,
                    _ => DelayMode::Mono,
                }
            }
            slot::FACT => p.fact = v.as_index().min(FACTORS.len() - 1),
            slot::PAT => p.pat = v.as_index().min(PATTERNS.len() - 1),
            slot::OFFS => p.offs_ms = v.as_f32(),
            slot::FEED => p.feed_pct = v.as_f32(),
            slot::FLC => p.flc_hz = v.as_f32(),
            slot::FHC => p.fhc_hz = v.as_f32(),
            slot::LC => p.lc_hz = v.as_f32(),
            slot::HC => p.hc_hz = v.as_f32(),
            _ => return,
        }
        self.dirty = true;
    }

    fn param(&self, slot: u16) -> Option<f32> {
        let p = &self.p;
        Some(match slot {
            slot::TIME => p.time_ms,
            slot::MODE => p.mode as u8 as f32,
            slot::FACT => p.fact as f32,
            slot::PAT => p.pat as f32,
            slot::OFFS => p.offs_ms,
            slot::FEED => p.feed_pct,
            slot::FLC => p.flc_hz,
            slot::FHC => p.fhc_hz,
            slot::LC => p.lc_hz,
            slot::HC => p.hc_hz,
            _ => return None,
        })
    }

    fn load_defaults(&mut self) {
        self.set_params(&StereoDelayParams::default());
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

    const FS: f64 = 48_000.0;

    /// Wide-open filters so echoes are (nearly) unfiltered impulses.
    fn open(p: &mut StereoDelayParams) {
        p.flc_hz = 20.0;
        p.fhc_hz = 20_000.0;
        p.lc_hz = 20.0;
        p.hc_hz = 20_000.0;
    }

    fn run(m: &mut StereoDelay, l: &mut [f32], r: &mut [f32]) {
        for (a, b) in l.chunks_mut(64).zip(r.chunks_mut(64)) {
            m.process(a, b, None);
        }
    }

    fn peak_at(x: &[f32]) -> usize {
        x.iter()
            .enumerate()
            .max_by(|a, b| a.1.abs().total_cmp(&b.1.abs()))
            .unwrap()
            .0
    }

    #[test]
    fn echo_times_follow_time_factor_pattern_offset() {
        let mut m = StereoDelay::new();
        let mut p = StereoDelayParams {
            time_ms: 10.0,
            fact: 10, // x2
            pat: 0,   // 1/2 : 1
            offs_ms: 1.0,
            feed_pct: 0.0,
            ..Default::default()
        };
        open(&mut p);
        m.set_params(&p);
        m.prepare(FS, 64);
        let n = 4096;
        let (mut l, mut r) = (vec![0.0f32; n], vec![0.0f32; n]);
        l[0] = 1.0;
        r[0] = 1.0;
        run(&mut m, &mut l, &mut r);
        // L = 10 * 2 * 0.5 = 10 ms = 480 samples, R = 20 ms + 1 ms = 1008 samples (filter group delay
        // shifts the impulse peak by a sample or two)
        assert!((peak_at(&l) as i64 - 481).abs() <= 2, "L {}", peak_at(&l));
        assert!((peak_at(&r) as i64 - 1009).abs() <= 2, "R {}", peak_at(&r));
        assert!(
            l[..470].iter().all(|v| v.abs() < 1e-3),
            "no dry signal in the wet output"
        );
    }

    /// 1 kHz Hann-windowed burst (96 samples, peak ~1) at the start of an `n`-sample buffer.
    fn burst(n: usize) -> Vec<f32> {
        let mut x = vec![0.0f32; n];
        for (i, v) in x.iter_mut().take(96).enumerate() {
            let w = 0.5 - 0.5 * (2.0 * std::f32::consts::PI * i as f32 / 96.0).cos();
            *v = w * (2.0 * std::f32::consts::PI * 1000.0 * i as f32 / FS as f32).sin();
        }
        x
    }

    /// Peak in the window where echo `k` of a 240-sample delay lands.
    fn echo(x: &[f32], k: usize) -> f32 {
        x[k * 240..k * 240 + 96]
            .iter()
            .fold(0f32, |a, v| a.max(v.abs()))
    }

    #[test]
    fn feedback_repeats_and_decays() {
        let mut m = StereoDelay::new();
        let mut p = StereoDelayParams {
            time_ms: 5.0,
            feed_pct: 50.0,
            ..Default::default()
        };
        open(&mut p);
        m.set_params(&p);
        m.prepare(FS, 64);
        let n = 2048;
        let x = burst(n);
        let (mut l, mut r) = (x.clone(), vec![0.0f32; n]);
        let dry = echo(&x, 0);
        run(&mut m, &mut l, &mut r);
        let (e1, e2, e3) = (echo(&l, 1), echo(&l, 2), echo(&l, 3));
        assert!((e1 / dry - 1.0).abs() < 0.03, "{e1} vs {dry}");
        assert!((e2 / e1 - 0.5).abs() < 0.03, "{e2} / {e1}");
        assert!((e3 / e2 - 0.5).abs() < 0.03, "{e3} / {e2}");
        assert!(echo(&l, 0) < 1e-3, "wet only");
        assert!(r.iter().all(|v| v.abs() < 1e-6), "ST: no crosstalk");
    }

    #[test]
    fn cross_mode_ping_pongs_and_mono_sums() {
        let mut m = StereoDelay::new();
        let mut p = StereoDelayParams {
            time_ms: 5.0,
            feed_pct: 50.0,
            mode: DelayMode::Cross,
            ..Default::default()
        };
        open(&mut p);
        m.set_params(&p);
        m.prepare(FS, 64);
        let n = 1024;
        let x = burst(n);
        let dry = echo(&x, 0);
        let (mut l, mut r) = (x.clone(), vec![0.0f32; n]);
        run(&mut m, &mut l, &mut r);
        // first echo on L, second on R (fed back from L), third on L again
        assert!((echo(&l, 1) / dry - 1.0).abs() < 0.03);
        assert!(echo(&r, 1) < 1e-3 && echo(&l, 2) < 1e-3);
        assert!((echo(&r, 2) / dry - 0.5).abs() < 0.03, "{}", echo(&r, 2));
        assert!((echo(&l, 3) / dry - 0.25).abs() < 0.03, "{}", echo(&l, 3));

        m.set_param(slot::MODE, ParamValue::Float(2.0));
        m.set_param(slot::FEED, ParamValue::Float(0.0));
        m.reset();
        let (mut l, mut r) = (x.clone(), vec![0.0f32; n]);
        run(&mut m, &mut l, &mut r);
        let (a, b) = (echo(&l, 1) / dry, echo(&r, 1) / dry);
        assert!((a - 0.5).abs() < 0.03 && (b - 0.5).abs() < 0.03, "{a} {b}");
    }

    #[test]
    fn params_round_trip_and_defaults() {
        let mut m = StereoDelay::new();
        m.prepare(FS, 64);
        assert_eq!(m.param(slot::TIME), Some(333.0));
        m.set_param(slot::TIME, ParamValue::Float(100.0));
        m.set_param(slot::PAT, ParamValue::Enum(8));
        assert_eq!(m.param(slot::TIME), Some(100.0));
        assert_eq!(m.param(slot::PAT), Some(8.0));
        assert_eq!(m.param(99), None);
        m.commit();
        let (dl, dr) = m.delay_samples();
        assert!(
            (dl - 4800.0).abs() < 0.01 && (dr - 7200.0).abs() < 0.01,
            "{dl} {dr}"
        );
        m.load_defaults();
        assert_eq!(*m.params(), StereoDelayParams::default());
        // effective delay is clamped to the line length
        m.set_param(slot::TIME, ParamValue::Float(3000.0));
        m.set_param(slot::FACT, ParamValue::Enum(10));
        m.commit();
        assert!(m.delay_samples().0 <= (MAX_DELAY_S * FS) as f32 + 2.0);
        assert_eq!(m.latency(), 0);
    }
}
