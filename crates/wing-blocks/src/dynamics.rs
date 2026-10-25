//! Standard WING dynamics: `COMP` (WING COMPRESSOR, `comp_pardef_xseries_comp`) and `EXP` (WING EXPANDER,
//! `comp_pardef_xseries_exp`), `docs/research/models.md` §3.2, plus the dynamics-slot wrapper parameters
//! (`dyn.mix`, `dyn.gain`, `dynsc` key filter / external key) from `docs/research/signal-flow.md`.
//!
//! Topology (feed-forward, log domain, Giannoulis/Massberg/Reiss "smooth branching" in dB):
//! ```text
//! key (SELF = input, or external buffer) -> key filter (OFF/LP12/HP12/BP/NOTCH, SVF)
//!   -> level detector (PEAK = |k|, RMS = one-pole mean square, tau RMS_TAU_S) -> dB
//!   -> stereo link (max of channel levels) -> static curve (thr/ratio/quadratic soft knee)
//!   -> attack / hold / release smoothing of the gain in dB (env LOG = exponential, LIN = constant dB/s)
//!   -> optional AUTO slow release stage -> gain
//! out = x * ((1 - mix) + mix * gain * makeup)
//! ```
//! Attack always means "towards the target in the direction the processor reacts to signal": for the
//! compressor gain reduction increasing, for the expander gain reduction decreasing (opening), matching
//! the X32/WING gate/expander convention. Time constants follow the `wing_dsp` convention (63.2 % of a
//! step after `tau`).
//!
//! Unverified assumptions (no reference data yet; see the T034 Log, to be checked against
//! `data/reference-curves/` from T00A): knee step width [`KNEE_DB_PER_STEP`], RMS window [`RMS_TAU_S`],
//! LIN release rate [`LIN_RELEASE_DB_PER_TAU`], AUTO constants, expander floor [`EXP_MAX_RANGE_DB`],
//! makeup applied to the wet path only.
//!
//! Real-time: no allocation anywhere (state is fixed-size arrays); `set_params` does a few `exp`s and is
//! meant for block-rate calls; `process*` is allocation-, lock- and syscall-free.

use wing_dsp::gain::{db_to_lin, db_to_lin_fast, lin_to_db_fast};
use wing_dsp::smooth::{one_pole_coeff, LinearRamp};
use wing_dsp::svf::{Svf, SvfMode};

/// Maximum channels per instance (mono or stereo strip).
pub const MAX_CHANNELS: usize = 2;
/// The `ratio` enum shared by COMP and EXP (models.md §3.2). For EXP it is the expansion ratio 1:R.
pub const RATIOS: [f32; 17] = [
    1.1, 1.2, 1.3, 1.5, 1.7, 2.0, 2.5, 3.0, 3.5, 4.0, 5.0, 6.0, 8.0, 10.0, 20.0, 50.0, 100.0,
];
/// Knee width in dB per `knee` step (0..5 -> 0..10 dB). Assumption from models.md ("commonly assumed").
pub const KNEE_DB_PER_STEP: f32 = 2.0;
/// RMS detector time constant (exponential mean-square window). Assumption.
pub const RMS_TAU_S: f64 = 0.010;
/// `env = LIN`: release moves at a constant `LIN_RELEASE_DB_PER_TAU` dB per release time. Assumption.
pub const LIN_RELEASE_DB_PER_TAU: f32 = 10.0;
/// AUTO: slow stage charge time constant. Assumption.
pub const AUTO_CHARGE_TAU_S: f64 = 0.25;
/// AUTO: slow stage release = `rel * AUTO_SLOW_FACTOR`, clamped to `AUTO_SLOW_MIN_S..=AUTO_SLOW_MAX_S`.
pub const AUTO_SLOW_FACTOR: f64 = 4.0;
pub const AUTO_SLOW_MIN_S: f64 = 0.3;
pub const AUTO_SLOW_MAX_S: f64 = 8.0;
/// Expander maximum attenuation (EXP has no range control). Assumption.
pub const EXP_MAX_RANGE_DB: f32 = 90.0;
/// De-zipper ramp for `mix` and `gain` changes.
pub const PARAM_RAMP_S: f64 = 0.005;

/// Compressor (`COMP`) or downward expander (`EXP`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DynMode {
    Compressor,
    Expander,
}

/// `det`: PEAK / RMS.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Detector {
    Peak,
    Rms,
}

/// `env`: release curve. `Log` = exponential in dB, `Lin` = constant dB/s.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Envelope {
    Lin,
    Log,
}

/// `dynsc.type`: key filter type.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum KeyFilterKind {
    Off,
    Lp12,
    Hp12,
    Bp,
    Notch,
}

/// `dynsc` key filter (`type`, `f`, `q` 0.44..10).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct KeyFilter {
    pub kind: KeyFilterKind,
    pub freq_hz: f32,
    pub q: f32,
}

impl Default for KeyFilter {
    fn default() -> Self {
        Self {
            kind: KeyFilterKind::Off,
            freq_hz: 1000.0,
            q: 0.707,
        }
    }
}

/// Typed parameters. Field ranges follow models.md (values are clamped by [`Dynamics::set_params`]).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DynParams {
    pub mode: DynMode,
    /// `thr` -60..0 dB.
    pub thr_db: f32,
    /// `ratio`, one of [`RATIOS`] (any value >= 1 accepted).
    pub ratio: f32,
    /// `knee` 0..5 (width = `knee * KNEE_DB_PER_STEP` dB).
    pub knee: u8,
    pub det: Detector,
    /// `att` 0..120 ms.
    pub att_ms: f32,
    /// `hld` 1..200 ms (COMP), 1..1000 ms (EXP).
    pub hld_ms: f32,
    /// `rel` 4..4000 ms.
    pub rel_ms: f32,
    pub env: Envelope,
    /// `auto`: programme-dependent (dual) release.
    pub auto: bool,
    /// `dyn.mix` 0..100 %.
    pub mix_pct: f32,
    /// `dyn.gain` make-up -6..+12 dB.
    pub gain_db: f32,
    /// `dynsc` key filter.
    pub key: KeyFilter,
    /// Stereo link (one gain for both channels from the max channel level).
    pub link: bool,
}

impl DynParams {
    /// `COMP` defaults (models.md §3.2).
    pub fn comp() -> Self {
        Self {
            mode: DynMode::Compressor,
            thr_db: -10.0,
            ratio: 3.0,
            knee: 3,
            det: Detector::Rms,
            att_ms: 50.0,
            hld_ms: 20.0,
            rel_ms: 150.0,
            env: Envelope::Log,
            auto: true,
            mix_pct: 100.0,
            gain_db: 0.0,
            key: KeyFilter::default(),
            link: true,
        }
    }

    /// `EXP` defaults (models.md §3.2).
    pub fn exp() -> Self {
        Self {
            mode: DynMode::Expander,
            ..Self::comp()
        }
    }

    /// Knee width in dB.
    pub fn knee_db(&self) -> f32 {
        self.knee as f32 * KNEE_DB_PER_STEP
    }

    fn clamped(&self) -> Self {
        let hld_max = match self.mode {
            DynMode::Compressor => 200.0,
            DynMode::Expander => 1000.0,
        };
        Self {
            thr_db: self.thr_db.clamp(-60.0, 0.0),
            ratio: self.ratio.clamp(1.0, 100.0),
            knee: self.knee.min(5),
            att_ms: self.att_ms.clamp(0.0, 120.0),
            hld_ms: self.hld_ms.clamp(1.0, hld_max),
            rel_ms: self.rel_ms.clamp(4.0, 4000.0),
            mix_pct: self.mix_pct.clamp(0.0, 100.0),
            gain_db: self.gain_db.clamp(-6.0, 12.0),
            key: KeyFilter {
                kind: self.key.kind,
                freq_hz: self.key.freq_hz.clamp(20.0, 20_000.0),
                q: self.key.q.clamp(0.44, 10.0),
            },
            ..*self
        }
    }
}

impl Default for DynParams {
    fn default() -> Self {
        Self::comp()
    }
}

/// Static gain computer (gain in dB, <= 0) for a detected level in dB. Quadratic soft knee of width
/// `knee_db` centred on the threshold (Giannoulis et al. 2012, eq. 4; mirrored for the expander).
#[inline]
pub fn static_gain_db(mode: DynMode, level_db: f32, thr_db: f32, ratio: f32, knee_db: f32) -> f32 {
    let half = 0.5 * knee_db;
    let d = level_db - thr_db;
    match mode {
        DynMode::Compressor => {
            let slope = 1.0 / ratio - 1.0;
            if d <= -half {
                0.0
            } else if d < half {
                let e = d + half;
                slope * e * e / (2.0 * knee_db)
            } else {
                slope * d
            }
        }
        DynMode::Expander => {
            let slope = ratio - 1.0;
            let g = if d >= half {
                0.0
            } else if d > -half {
                let e = d - half;
                -slope * e * e / (2.0 * knee_db)
            } else {
                slope * d
            };
            g.max(-EXP_MAX_RANGE_DB)
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
struct ChanState {
    /// Mean-square detector state (RMS).
    ms: f32,
    /// Smoothed control value in "attack-positive" dB (comp: GR, exp: -GR).
    fast: f32,
    /// AUTO slow stage.
    slow: f32,
    hold_left: u32,
    /// Current gain reduction (dB, >= 0) for metering.
    gr_db: f32,
}

/// Standard WING compressor / expander instance (mono or stereo).
pub struct Dynamics {
    fs: f64,
    channels: usize,
    p: DynParams,
    // derived
    knee_db: f32,
    sign: f32,
    att_c: f32,
    rel_c: f32,
    lin_step: f32,
    hold_samples: u32,
    auto_charge_c: f32,
    auto_slow_c: f32,
    rms_c: f32,
    key_on: bool,
    // state
    st: [ChanState; MAX_CHANNELS],
    key_filt: [Svf; MAX_CHANNELS],
    mix: LinearRamp,
    makeup: LinearRamp,
    block_gr_db: f32,
}

impl Dynamics {
    /// New instance at 48 kHz, mono; call [`Dynamics::prepare`] before processing.
    pub fn new(params: DynParams) -> Self {
        let mut d = Self {
            fs: 48_000.0,
            channels: 1,
            p: params,
            knee_db: 0.0,
            sign: 1.0,
            att_c: 0.0,
            rel_c: 0.0,
            lin_step: 0.0,
            hold_samples: 0,
            auto_charge_c: 0.0,
            auto_slow_c: 0.0,
            rms_c: 0.0,
            key_on: false,
            st: [ChanState::default(); MAX_CHANNELS],
            key_filt: [Svf::default(); MAX_CHANNELS],
            mix: LinearRamp::new(0.0, 0),
            makeup: LinearRamp::new(1.0, 0),
            block_gr_db: 0.0,
        };
        d.prepare(48_000.0, 1);
        d
    }

    /// Set sample rate and channel count (1..=[`MAX_CHANNELS`]); resets state. Never allocates.
    pub fn prepare(&mut self, fs: f64, channels: usize) {
        self.fs = fs;
        self.channels = channels.clamp(1, MAX_CHANNELS);
        let ramp = (PARAM_RAMP_S * fs).round() as u32;
        self.mix.set_ramp_len(ramp);
        self.makeup.set_ramp_len(ramp);
        let p = self.p;
        self.set_params(&p);
        self.reset();
    }

    /// Update parameters (block rate). Values are clamped to the documented ranges.
    pub fn set_params(&mut self, params: &DynParams) {
        let p = params.clamped();
        let fs = self.fs;
        self.knee_db = p.knee_db();
        self.sign = match p.mode {
            DynMode::Compressor => 1.0,
            DynMode::Expander => -1.0,
        };
        let rel_s = p.rel_ms as f64 * 1e-3;
        self.att_c = one_pole_coeff(p.att_ms as f64 * 1e-3, fs) as f32;
        self.rel_c = one_pole_coeff(rel_s, fs) as f32;
        self.lin_step = (LIN_RELEASE_DB_PER_TAU as f64 / (rel_s * fs)) as f32;
        self.hold_samples = (p.hld_ms as f64 * 1e-3 * fs).round() as u32;
        self.auto_charge_c = one_pole_coeff(AUTO_CHARGE_TAU_S, fs) as f32;
        self.auto_slow_c = one_pole_coeff(
            (rel_s * AUTO_SLOW_FACTOR).clamp(AUTO_SLOW_MIN_S, AUTO_SLOW_MAX_S),
            fs,
        ) as f32;
        self.rms_c = one_pole_coeff(RMS_TAU_S, fs) as f32;
        let mode = match p.key.kind {
            KeyFilterKind::Off => None,
            KeyFilterKind::Lp12 => Some(SvfMode::Lowpass),
            KeyFilterKind::Hp12 => Some(SvfMode::Highpass),
            KeyFilterKind::Bp => Some(SvfMode::Bandpass),
            KeyFilterKind::Notch => Some(SvfMode::Notch),
        };
        self.key_on = mode.is_some();
        if let Some(m) = mode {
            for f in &mut self.key_filt {
                f.set(m, fs, p.key.freq_hz as f64, p.key.q as f64, 0.0);
            }
        }
        self.mix.set_target(p.mix_pct * 0.01);
        self.makeup.set_target(db_to_lin(p.gain_db));
        if p.auto != self.p.auto {
            for s in &mut self.st {
                s.slow = s.fast;
            }
        }
        self.p = p;
    }

    /// Current (clamped) parameters.
    pub fn params(&self) -> &DynParams {
        &self.p
    }

    /// Clear detector/envelope/filter state and jump parameter ramps to their targets.
    pub fn reset(&mut self) {
        self.st = [ChanState::default(); MAX_CHANNELS];
        for f in &mut self.key_filt {
            f.reset();
        }
        self.mix.reset(self.p.mix_pct * 0.01);
        self.makeup.reset(db_to_lin(self.p.gain_db));
        self.block_gr_db = 0.0;
    }

    /// Latency in samples (always 0: no look-ahead).
    pub fn latency(&self) -> usize {
        0
    }

    /// Current gain reduction in dB (>= 0, max over channels) for the GR meter.
    pub fn gain_reduction_db(&self) -> f32 {
        let mut g = 0.0f32;
        for s in &self.st[..self.channels] {
            g = g.max(s.gr_db);
        }
        g
    }

    /// Gain reduction of one channel in dB.
    pub fn channel_gain_reduction_db(&self, ch: usize) -> f32 {
        self.st[ch.min(MAX_CHANNELS - 1)].gr_db
    }

    /// Maximum gain reduction (dB) reached during the last `process` call (peak-hold for meters).
    pub fn block_peak_gain_reduction_db(&self) -> f32 {
        self.block_gr_db
    }

    /// Mono convenience wrapper around [`Dynamics::process`].
    pub fn process_mono(&mut self, buf: &mut [f32], key: Option<&[f32]>) {
        let keys = key.map(|k| [k]);
        let mut io = [buf];
        self.process(&mut io, keys.as_ref().map(|k| &k[..]));
    }

    /// Process `io` (one slice per channel, up to the prepared channel count) in place.
    ///
    /// `key`: external side-chain (`dynsc.src` != SELF), one slice per channel or one mono slice used for
    /// every channel; `None` = SELF (the input itself). Key samples beyond a key slice's length read as 0.
    pub fn process(&mut self, io: &mut [&mut [f32]], key: Option<&[&[f32]]>) {
        let nch = io.len().min(self.channels);
        self.block_gr_db = 0.0;
        if nch == 0 {
            return;
        }
        let n = io[..nch].iter().map(|b| b.len()).min().unwrap_or(0);
        let key = key.filter(|k| !k.is_empty());
        let link = self.p.link && nch > 1;
        let rms = self.p.det == Detector::Rms;
        let mut block_gr = 0.0f32;
        for i in 0..n {
            // ---- key + level detection (linear peak, or mean square for RMS) ----
            let mut lv = [0.0f32; MAX_CHANNELS];
            for (ch, l) in lv.iter_mut().enumerate().take(nch) {
                let mut k = match key {
                    Some(ks) => ks[ch.min(ks.len() - 1)].get(i).copied().unwrap_or(0.0),
                    None => io[ch][i],
                };
                if self.key_on {
                    k = self.key_filt[ch].process(k);
                }
                *l = if rms {
                    let st = &mut self.st[ch];
                    let sq = k * k;
                    st.ms = sq + (st.ms - sq) * self.rms_c;
                    st.ms
                } else {
                    k.abs()
                };
            }
            let mix = self.mix.tick();
            let makeup = self.makeup.tick();
            let dry = 1.0 - mix;
            let wet = mix * makeup;
            if link {
                let mut l = lv[0];
                for &v in &lv[1..nch] {
                    l = l.max(v);
                }
                let g = self.gain_step(0, l, rms);
                let gr = self.st[0].gr_db;
                for ch in 1..nch {
                    self.st[ch].gr_db = gr;
                }
                block_gr = block_gr.max(gr);
                let m = dry + wet * g;
                for buf in io[..nch].iter_mut() {
                    buf[i] *= m;
                }
            } else {
                for (ch, buf) in io[..nch].iter_mut().enumerate() {
                    let g = self.gain_step(ch, lv[ch], rms);
                    block_gr = block_gr.max(self.st[ch].gr_db);
                    buf[i] *= dry + wet * g;
                }
            }
        }
        self.block_gr_db = block_gr;
        for (st, f) in self.st.iter_mut().zip(self.key_filt.iter_mut()) {
            if st.ms < 1e-30 {
                st.ms = 0.0;
            }
            f.flush_denormals();
        }
    }

    /// One sample of gain computer + ballistics for channel state `ch`; returns the linear gain.
    #[inline(always)]
    fn gain_step(&mut self, ch: usize, level: f32, rms: bool) -> f32 {
        let level_db = if rms {
            0.5 * lin_to_db_fast(level)
        } else {
            lin_to_db_fast(level)
        };
        let g = static_gain_db(
            self.p.mode,
            level_db,
            self.p.thr_db,
            self.p.ratio,
            self.knee_db,
        );
        // attack-positive domain: comp -> GR (= -g), exp -> g (= -GR)
        let target = -self.sign * g;
        let st = &mut self.st[ch];
        if target >= st.fast {
            st.fast = target + (st.fast - target) * self.att_c;
            st.hold_left = self.hold_samples;
        } else if st.hold_left > 0 {
            st.hold_left -= 1;
        } else {
            st.fast = match self.p.env {
                Envelope::Log => target + (st.fast - target) * self.rel_c,
                Envelope::Lin => (st.fast - self.lin_step).max(target),
            };
        }
        let mut s = st.fast;
        if self.p.auto {
            let c = if st.fast > st.slow {
                self.auto_charge_c
            } else {
                self.auto_slow_c
            };
            st.slow = st.fast + (st.slow - st.fast) * c;
            s = s.max(st.slow);
        }
        let gain_db = -self.sign * s;
        st.gr_db = -gain_db;
        db_to_lin_fast(gain_db)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FS: f64 = 48_000.0;

    fn base(mode: DynMode) -> DynParams {
        DynParams {
            mode,
            det: Detector::Peak,
            att_ms: 0.0,
            hld_ms: 1.0,
            rel_ms: 100.0,
            auto: false,
            knee: 0,
            ..DynParams::comp()
        }
    }

    /// Exact f64 reference of the documented static curve.
    fn reference_gain(mode: DynMode, x: f64, t: f64, r: f64, w: f64) -> f64 {
        match mode {
            DynMode::Compressor => {
                if 2.0 * (x - t) < -w {
                    0.0
                } else if 2.0 * (x - t).abs() <= w && w > 0.0 {
                    (1.0 / r - 1.0) * (x - t + w / 2.0).powi(2) / (2.0 * w)
                } else {
                    (1.0 / r - 1.0) * (x - t)
                }
            }
            DynMode::Expander => {
                let g = if 2.0 * (x - t) > w {
                    0.0
                } else if 2.0 * (x - t).abs() <= w && w > 0.0 {
                    -(r - 1.0) * (x - t - w / 2.0).powi(2) / (2.0 * w)
                } else {
                    (r - 1.0) * (x - t)
                };
                g.max(-EXP_MAX_RANGE_DB as f64)
            }
        }
    }

    /// Run a DC level through the full block (peak detector, instant attack) and return the steady gain (dB).
    fn steady_gain_db(p: DynParams, level_db: f32) -> f32 {
        let mut d = Dynamics::new(p);
        d.prepare(FS, 1);
        let x = db_to_lin(level_db);
        let mut buf = [x; 256];
        // 1 s: the expander reaches its gain by *release* (closing), tau 100 ms
        for _ in 0..200 {
            buf.fill(x);
            d.process_mono(&mut buf, None);
        }
        20.0 * (buf[255] / x).log10()
    }

    #[test]
    fn static_curve_matches_reference_incl_knee() {
        for mode in [DynMode::Compressor, DynMode::Expander] {
            for &thr in &[-40.0f32, -20.0, -10.0, 0.0] {
                for &ratio in &[1.5f32, 3.0, 10.0, 100.0] {
                    for knee in 0..=5u8 {
                        let w = knee as f32 * KNEE_DB_PER_STEP;
                        let mut x = thr - 15.0;
                        while x <= (thr + 10.0).min(6.0) {
                            let g = static_gain_db(mode, x, thr, ratio, w);
                            let r =
                                reference_gain(mode, x as f64, thr as f64, ratio as f64, w as f64);
                            assert!(
                                (g as f64 - r).abs() < 0.01,
                                "{mode:?} thr {thr} ratio {ratio} knee {knee} x {x}: {g} vs {r}"
                            );
                            x += 0.25;
                        }
                    }
                }
            }
        }
        // Spot values: 4:1, 6 dB knee, at the threshold GR = 0.75 * 3^2 / 12 = 0.5625 dB.
        let g = static_gain_db(DynMode::Compressor, -20.0, -20.0, 4.0, 6.0);
        assert!((g + 0.5625).abs() < 1e-5);
        // Expander 2:1 hard knee, 10 dB below threshold -> -10 dB.
        let g = static_gain_db(DynMode::Expander, -30.0, -20.0, 2.0, 0.0);
        assert!((g + 10.0).abs() < 1e-5);
        // Expander floor.
        assert_eq!(
            static_gain_db(DynMode::Expander, -240.0, 0.0, 100.0, 0.0),
            -EXP_MAX_RANGE_DB
        );
    }

    #[test]
    fn static_curve_end_to_end_within_0_1_db() {
        for mode in [DynMode::Compressor, DynMode::Expander] {
            for knee in [0u8, 3, 5] {
                for &ratio in &[2.0f32, 4.0, 20.0] {
                    let thr = -20.0;
                    let p = DynParams {
                        thr_db: thr,
                        ratio,
                        knee,
                        ..base(mode)
                    };
                    let mut x = -40.0f32;
                    while x <= 0.0 {
                        let got = steady_gain_db(p, x);
                        let want = reference_gain(
                            mode,
                            x as f64,
                            thr as f64,
                            ratio as f64,
                            p.knee_db() as f64,
                        );
                        assert!(
                            (got as f64 - want).abs() < 0.1,
                            "{mode:?} knee {knee} ratio {ratio} x {x}: {got} vs {want}"
                        );
                        x += 1.0;
                    }
                }
            }
        }
    }

    #[test]
    fn rms_detector_reads_sine_rms() {
        // 0 dBFS-peak 1 kHz sine = -3.01 dB RMS; thr -20, 4:1 hard -> gain -(16.99)*0.75 = -12.74 dB.
        let p = DynParams {
            thr_db: -20.0,
            ratio: 4.0,
            det: Detector::Rms,
            att_ms: 5.0,
            rel_ms: 500.0,
            ..base(DynMode::Compressor)
        };
        let mut d = Dynamics::new(p);
        d.prepare(FS, 1);
        let w = 2.0 * std::f64::consts::PI * 1000.0 / FS;
        let mut buf = [0.0f32; 480];
        for blk in 0..40 {
            for (i, b) in buf.iter_mut().enumerate() {
                *b = ((blk * 480 + i) as f64 * w).sin() as f32;
            }
            d.process_mono(&mut buf, None);
        }
        let want = -(20.0 * (0.5f64).sqrt().log10() + 20.0) * 0.75;
        let gr = d.gain_reduction_db() as f64;
        assert!((-gr - want).abs() < 0.1, "gr {gr} want {}", -want);
    }

    /// Step response of the gain reduction (dB) for a key level step; returns GR per sample.
    fn gr_trace(d: &mut Dynamics, level: f32, n: usize) -> Vec<f32> {
        let mut out = Vec::with_capacity(n);
        let mut buf = [0.0f32; 1];
        for _ in 0..n {
            buf[0] = level;
            d.process_mono(&mut buf, None);
            out.push(d.gain_reduction_db());
        }
        out
    }

    #[test]
    fn attack_hold_release_time_constants() {
        let p = DynParams {
            thr_db: -40.0,
            ratio: 100.0,
            att_ms: 10.0,
            hld_ms: 20.0,
            rel_ms: 100.0,
            ..base(DynMode::Compressor)
        };
        let mut d = Dynamics::new(p);
        d.prepare(FS, 1);
        let target = 40.0 * (1.0 - 1.0 / 100.0);
        let att = gr_trace(&mut d, 1.0, 48_000);
        let at_tau = att[(0.010 * FS) as usize - 1];
        assert!(
            (at_tau / target - 0.632).abs() < 0.01,
            "attack {at_tau} / {target}"
        );
        let start = *att.last().unwrap();
        assert!((start - target).abs() < 0.01);
        let rel = gr_trace(&mut d, 0.0, 48_000);
        let hold = (0.020 * FS) as usize;
        assert!((rel[hold - 1] - start).abs() < 1e-4, "held during hold");
        let at_tau = rel[hold + (0.100 * FS) as usize];
        assert!(
            (at_tau / start - 0.368).abs() < 0.01,
            "release {at_tau} / {start}"
        );
    }

    #[test]
    fn lin_envelope_releases_at_constant_db_rate() {
        let p = DynParams {
            thr_db: -40.0,
            ratio: 100.0,
            rel_ms: 100.0,
            env: Envelope::Lin,
            ..base(DynMode::Compressor)
        };
        let mut d = Dynamics::new(p);
        d.prepare(FS, 1);
        gr_trace(&mut d, 1.0, 4800);
        let rel = gr_trace(&mut d, 0.0, 9600);
        let h = (0.001 * FS) as usize;
        let a = rel[h + 480];
        let b = rel[h + 480 + 4800];
        // 100 ms -> LIN_RELEASE_DB_PER_TAU dB
        assert!(
            ((a - b) - LIN_RELEASE_DB_PER_TAU).abs() < 0.05,
            "{a} -> {b}"
        );
    }

    #[test]
    fn auto_release_is_programme_dependent() {
        let p = DynParams {
            thr_db: -40.0,
            ratio: 100.0,
            rel_ms: 50.0,
            auto: true,
            ..base(DynMode::Compressor)
        };
        // after a sustained 2 s overload, auto releases much slower than after a 2 ms transient
        let measure = |overload: usize| {
            let mut d = Dynamics::new(p);
            d.prepare(FS, 1);
            gr_trace(&mut d, 1.0, overload);
            let start = d.gain_reduction_db();
            let rel = gr_trace(&mut d, 0.0, 9600);
            rel[(0.1 * FS) as usize] / start
        };
        let short = measure(96);
        let long = measure(96_000);
        let mut fixed = Dynamics::new(DynParams { auto: false, ..p });
        fixed.prepare(FS, 1);
        gr_trace(&mut fixed, 1.0, 96_000);
        let s = fixed.gain_reduction_db();
        let fixed_ratio = gr_trace(&mut fixed, 0.0, 9600)[(0.1 * FS) as usize] / s;
        assert!(long > 3.0 * short, "long {long} short {short}");
        assert!(long > 3.0 * fixed_ratio);
        assert!(short < 0.25, "short transient releases fast: {short}");
    }

    #[test]
    fn expander_attack_opens_release_closes() {
        let p = DynParams {
            thr_db: -30.0,
            ratio: 100.0,
            att_ms: 1.0,
            hld_ms: 10.0,
            rel_ms: 50.0,
            ..base(DynMode::Expander)
        };
        let mut d = Dynamics::new(p);
        d.prepare(FS, 1);
        let quiet = db_to_lin(-60.0);
        let closed = gr_trace(&mut d, quiet, 48_000);
        assert!((closed.last().unwrap() - EXP_MAX_RANGE_DB).abs() < 0.01);
        let open = gr_trace(&mut d, 0.5, 4800);
        // attack tau 1 ms: GR falls to 36.8 % after 1 ms
        let r = open[47] / EXP_MAX_RANGE_DB;
        assert!((r - 0.368).abs() < 0.01, "{r}");
        let rel = gr_trace(&mut d, quiet, 48_000);
        let hold = 480;
        assert!(rel[hold - 1] < 0.01, "held open");
        let r = rel[hold + 2400] / EXP_MAX_RANGE_DB;
        assert!((r - 0.632).abs() < 0.01, "{r}");
    }

    #[test]
    fn mix_and_makeup() {
        // 2:1 hard, thr -20, DC at -10 -> -5 dB gain
        let g = db_to_lin(-5.0);
        for (mix, makeup) in [(100.0f32, 0.0f32), (50.0, 0.0), (0.0, 6.0), (50.0, 6.0)] {
            let p = DynParams {
                thr_db: -20.0,
                ratio: 2.0,
                mix_pct: mix,
                gain_db: makeup,
                ..base(DynMode::Compressor)
            };
            let x = db_to_lin(-10.0);
            let mut d = Dynamics::new(p);
            d.prepare(FS, 1);
            let mut buf = [x; 64];
            d.process_mono(&mut buf, None);
            let m = mix * 0.01;
            let want = x * ((1.0 - m) + m * g * db_to_lin(makeup));
            assert!(
                (buf[63] / want - 1.0).abs() < 1e-3,
                "mix {mix} gain {makeup}: {} vs {want}",
                buf[63]
            );
        }
    }

    #[test]
    fn param_changes_are_ramped() {
        let mut d = Dynamics::new(DynParams {
            thr_db: 0.0,
            ..base(DynMode::Compressor)
        });
        d.prepare(FS, 1);
        let mut p = *d.params();
        p.gain_db = 12.0;
        d.set_params(&p);
        let mut buf = [0.1f32; 512];
        d.process_mono(&mut buf, None);
        assert!(buf[0] < 0.11, "no jump: {}", buf[0]);
        assert!((buf[511] - 0.1 * db_to_lin(12.0)).abs() < 1e-4);
    }

    fn sine(f: f64, amp: f32, n: usize) -> Vec<f32> {
        (0..n)
            .map(|i| amp * (2.0 * std::f64::consts::PI * f * i as f64 / FS).sin() as f32)
            .collect()
    }

    #[test]
    fn key_filter_shapes_detection() {
        let run = |kind: KeyFilterKind, f: f64| {
            let p = DynParams {
                thr_db: -20.0,
                ratio: 10.0,
                det: Detector::Rms,
                key: KeyFilter {
                    kind,
                    freq_hz: 2000.0,
                    q: 0.707,
                },
                ..base(DynMode::Compressor)
            };
            let mut d = Dynamics::new(p);
            d.prepare(FS, 1);
            let mut buf = sine(f, 0.5, 24_000);
            d.process_mono(&mut buf, None);
            d.gain_reduction_db()
        };
        let off = run(KeyFilterKind::Off, 100.0);
        // compare at the same frequency: RMS ripple (and so peak-biased GR) depends on frequency
        let off_2k = run(KeyFilterKind::Off, 2000.0);
        let hp = run(KeyFilterKind::Hp12, 100.0);
        let lp = run(KeyFilterKind::Lp12, 100.0);
        let bp_center = run(KeyFilterKind::Bp, 2000.0);
        let notch_center = run(KeyFilterKind::Notch, 2000.0);
        assert!(off > 10.0, "off {off}");
        assert!(
            hp < 0.01,
            "100 Hz key through 2 kHz HP12 (-52 dB) -> no GR: {hp}"
        );
        assert!((lp - off).abs() < 0.1, "LP passes 100 Hz: {lp} vs {off}");
        assert!(
            (bp_center - off_2k).abs() < 0.05,
            "BP 0 dB at centre: {bp_center} vs {off_2k}"
        );
        assert!(notch_center < 0.01, "notch removes centre: {notch_center}");
    }

    #[test]
    fn external_key() {
        let p = DynParams {
            thr_db: -20.0,
            ratio: 4.0,
            ..base(DynMode::Compressor)
        };
        let mut d = Dynamics::new(p);
        d.prepare(FS, 1);
        // loud input, silent key -> untouched
        let silent = [0.0f32; 64];
        let mut buf = [0.9f32; 64];
        d.process_mono(&mut buf, Some(&silent));
        assert!(buf.iter().all(|&v| (v - 0.9).abs() < 1e-6));
        // quiet input, loud key -> ducked by the key's static gain (0 dB key: -15 dB)
        let loud = [1.0f32; 64];
        let mut buf = [0.01f32; 64];
        d.process_mono(&mut buf, Some(&loud));
        let g = 20.0 * (buf[63] / 0.01).log10();
        assert!((g + 15.0).abs() < 0.05, "{g}");
    }

    #[test]
    fn stereo_link() {
        for link in [true, false] {
            let p = DynParams {
                thr_db: -20.0,
                ratio: 4.0,
                link,
                ..base(DynMode::Compressor)
            };
            let mut d = Dynamics::new(p);
            d.prepare(FS, 2);
            let mut l = [1.0f32; 64];
            let mut r = [0.01f32; 64];
            d.process(&mut [&mut l, &mut r], None);
            let gl = 20.0 * l[63].log10();
            let gr = 20.0 * (r[63] / 0.01).log10();
            assert!((gl + 15.0).abs() < 0.05);
            if link {
                assert!((gr + 15.0).abs() < 0.05, "linked R follows L: {gr}");
            } else {
                assert!(gr.abs() < 1e-3, "unlinked R untouched: {gr}");
            }
            assert!((d.gain_reduction_db() - 15.0).abs() < 0.05);
            assert!((d.block_peak_gain_reduction_db() - 15.0).abs() < 0.05);
        }
    }

    #[test]
    fn mono_key_feeds_both_channels() {
        let mut d = Dynamics::new(DynParams {
            thr_db: -20.0,
            ratio: 4.0,
            link: false,
            ..base(DynMode::Compressor)
        });
        d.prepare(FS, 2);
        let key = [1.0f32; 32];
        let mut l = [0.01f32; 32];
        let mut r = [0.01f32; 32];
        d.process(&mut [&mut l, &mut r], Some(&[&key]));
        assert!((d.channel_gain_reduction_db(0) - 15.0).abs() < 0.05);
        assert!((d.channel_gain_reduction_db(1) - 15.0).abs() < 0.05);
    }

    #[test]
    fn params_are_clamped_and_defaults_match_models_md() {
        let c = DynParams::comp();
        assert_eq!((c.thr_db, c.ratio, c.knee), (-10.0, 3.0, 3));
        assert_eq!((c.att_ms, c.hld_ms, c.rel_ms), (50.0, 20.0, 150.0));
        assert_eq!((c.det, c.env, c.auto), (Detector::Rms, Envelope::Log, true));
        let d = Dynamics::new(DynParams {
            thr_db: -100.0,
            hld_ms: 900.0,
            knee: 9,
            ..DynParams::comp()
        });
        assert_eq!(d.params().thr_db, -60.0);
        assert_eq!(d.params().hld_ms, 200.0);
        assert_eq!(d.params().knee, 5);
        let e = Dynamics::new(DynParams {
            hld_ms: 900.0,
            ..DynParams::exp()
        });
        assert_eq!(e.params().hld_ms, 900.0);
        assert!(RATIOS.windows(2).all(|w| w[0] < w[1]));
    }

    #[test]
    fn silence_stays_silent_and_finite() {
        for p in [DynParams::comp(), DynParams::exp()] {
            let mut d = Dynamics::new(p);
            d.prepare(FS, 2);
            let mut l = [0.0f32; 64];
            let mut r = [0.0f32; 64];
            for _ in 0..100 {
                d.process(&mut [&mut l, &mut r], None);
            }
            assert!(l.iter().chain(r.iter()).all(|v| *v == 0.0));
            assert!(d.gain_reduction_db().is_finite());
        }
    }
}
