//! `PSE` (SOURCE EXTRACTOR) and `CMB` (PSE/LA COMBO), T04A. Parameters: `docs/research/models.md` §3.1.
//!
//! # What "Source Extractor" is (research, T004 left the hardware unidentified)
//! WING-Edit's internal name is `dyn_pardef_pse545`, the model sits in the **gate slot** (`ch/N/gate/mdl`
//! = `PSE` in PUBLIC.snap) and has exactly the controls of the **Waves Primary Source Expander** (PSE):
//! a threshold, a `depth` (maximum attenuation, "range") and two timing/detector switches. The Waves PSE is
//! sold as a *soft* downward expander for reducing stage bleed and feedback: when the microphone's own
//! source stops playing, its level is gently pulled down by a limited amount instead of being hard-gated
//! (Waves product pages; Behringer forum users use it the same way on vocals, see VERIFY-ON-HARDWARE.md).
//! "Extractor" is therefore taken as "emphasise the primary source over the spill", not as a key-from-other-
//! channel device; the standard slot side-chain (`gatesc`, external key) is honoured like every gate model.
//!
//! # Model (all constants are assumptions, listed in docs/VERIFY-ON-HARDWARE.md)
//! ```text
//! key (own input or external) -> stereo-linked detector -> level L (dBFS)
//! x = L - thr ;  gr_target = soft-knee 2:1 downward expansion, capped at depth
//!   x >= +3 dB: 0 ; x <= -3 dB: -x ; in between: (3 - x)^2 / 12      (6 dB knee, expansion slope 2:1)
//! gr = one-pole smoothing in dB: rising (closing) tau_c, falling (opening) tau_o
//! out = in * 10^(-gr/20)                                              (no look-ahead, latency 0)
//! ```
//! * `peak` off: RMS detector (10 ms mean-square one-pole on the louder channel); on: peak follower
//!   (instant attack, 20 ms release).
//! * `fast` off: closing 120 ms / opening 8 ms; on: closing 30 ms / opening 1.5 ms.
//! * `thr` -36..12 dB (default 0), `depth` 0..20 dB (default 12); slot order `thr depth fast peak`.
//!
//! # CMB
//! The combo is the pre-3.1 fixed aux dynamics: PSE followed by the LA leveler
//! ([`OptoComp`](crate::models::comp_opto::OptoComp), `OptoKind::La`) in series. Slots 1-4 are the PSE
//! parameters, slots 5-7 are the LA `ingain cpeak cmode` (LA slots 1-3). CMB defaults: `thr` -36,
//! `cpeak` 0 (leveler idle), otherwise the PSE and LA defaults. An external key drives the PSE stage only;
//! the LA stage always detects on its own input.
//!
//! Real-time: no allocation after construction; `set_param` stages, `commit`/`process`/`reset` apply.
//! Bench: `benches/pse.rs`.

use crate::model::{Model, ModelTelemetry, ParamValue};
use crate::models::comp_opto::{OptoComp, OptoKind};

pub const KNEE_DB: f32 = 6.0;
pub const RMS_MS: f32 = 10.0;
pub const PEAK_RELEASE_MS: f32 = 20.0;
pub const CLOSE_MS: [f32; 2] = [120.0, 30.0];
pub const OPEN_MS: [f32; 2] = [8.0, 1.5];
const FLOOR: f32 = 1e-24;

/// Static gain reduction (dB, >= 0) of the soft 2:1 downward expander for key level `level_db`.
pub fn static_gr_db(level_db: f32, thr_db: f32, depth_db: f32) -> f32 {
    let x = level_db - thr_db;
    let h = KNEE_DB * 0.5;
    let gr = if x >= h {
        0.0
    } else if x <= -h {
        -x
    } else {
        (h - x) * (h - x) / (2.0 * KNEE_DB)
    };
    gr.min(depth_db.max(0.0))
}

/// Which model an instance is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PseKind {
    Pse,
    Combo,
}

/// `PSE` / `CMB` model.
pub struct SourceExtractor {
    kind: PseKind,
    la: Option<OptoComp>,
    /// thr, depth, fast, peak
    v: [f32; 4],
    fs: f32,
    dirty: bool,
    a_rms: f32,
    a_peak_rel: f32,
    a_close: f32,
    a_open: f32,
    ms: f32,
    env: f32,
    gr: f32,
}

fn coef(ms: f32, fs: f32) -> f32 {
    1.0 - (-1.0 / (ms * 1e-3 * fs)).exp()
}

impl SourceExtractor {
    pub fn new(kind: PseKind) -> Self {
        let la = (kind == PseKind::Combo).then(|| {
            let mut la = OptoComp::new(OptoKind::La);
            la.set_param(2, ParamValue::Float(0.0));
            la
        });
        let mut m = Self {
            kind,
            la,
            v: [0.0; 4],
            fs: 48_000.0,
            dirty: true,
            a_rms: 0.0,
            a_peak_rel: 0.0,
            a_close: 0.0,
            a_open: 0.0,
            ms: FLOOR,
            env: 0.0,
            gr: 0.0,
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
        let i = self.v[2] as usize;
        self.a_rms = coef(RMS_MS, self.fs);
        self.a_peak_rel = coef(PEAK_RELEASE_MS, self.fs);
        self.a_close = coef(CLOSE_MS[i], self.fs);
        self.a_open = coef(OPEN_MS[i], self.fs);
    }
}

impl Model for SourceExtractor {
    fn id(&self) -> &'static str {
        match self.kind {
            PseKind::Pse => "PSE",
            PseKind::Combo => "CMB",
        }
    }

    fn prepare(&mut self, sample_rate: f64, max_block: usize) {
        self.fs = sample_rate as f32;
        if let Some(la) = &mut self.la {
            la.prepare(sample_rate, max_block);
        }
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
        let [thr, depth, _, peak] = self.v;
        let use_peak = peak >= 0.5;
        for k in 0..n {
            let (kl, kr) = match sidechain {
                Some((a, b)) if k < a.len() && k < b.len() => (a[k], b[k]),
                _ => (left[k], right[k]),
            };
            let level_db = if use_peak {
                let p = kl.abs().max(kr.abs());
                self.env = p.max(self.env - self.a_peak_rel * self.env);
                20.0 * self.env.max(1e-12).log10()
            } else {
                let p = (kl * kl).max(kr * kr);
                self.ms += self.a_rms * (p - self.ms);
                10.0 * self.ms.max(FLOOR).log10()
            };
            let target = static_gr_db(level_db, thr, depth);
            let a = if target > self.gr {
                self.a_close
            } else {
                self.a_open
            };
            self.gr += a * (target - self.gr);
            let g = (-self.gr * (core::f32::consts::LN_10 / 20.0)).exp();
            left[k] *= g;
            right[k] *= g;
        }
        if let Some(la) = &mut self.la {
            la.process(left, right, None);
        }
    }

    fn latency(&self) -> usize {
        self.la.as_ref().map_or(0, |l| l.latency())
    }

    fn reset(&mut self) {
        self.apply();
        self.ms = FLOOR;
        self.env = 0.0;
        self.gr = 0.0;
        if let Some(la) = &mut self.la {
            la.reset();
        }
    }

    fn set_param(&mut self, slot: u16, value: ParamValue) {
        match slot {
            1 => self.v[0] = value.as_f32().clamp(-36.0, 12.0),
            2 => self.v[1] = value.as_f32().clamp(0.0, 20.0),
            3 => self.v[2] = value.as_bool() as u32 as f32,
            4 => self.v[3] = value.as_bool() as u32 as f32,
            5..=7 => {
                if let Some(la) = &mut self.la {
                    la.set_param(slot - 4, value);
                }
            }
            _ => return,
        }
        self.dirty = true;
    }

    fn param(&self, slot: u16) -> Option<f32> {
        match slot {
            1..=4 => Some(self.v[slot as usize - 1]),
            5..=7 => self.la.as_ref().and_then(|l| l.param(slot - 4)),
            _ => None,
        }
    }

    fn load_defaults(&mut self) {
        self.v = [
            if self.kind == PseKind::Combo {
                -36.0
            } else {
                0.0
            },
            12.0,
            0.0,
            0.0,
        ];
        if let Some(la) = &mut self.la {
            la.load_defaults();
            la.set_param(2, ParamValue::Float(0.0));
        }
        self.dirty = true;
    }

    fn commit(&mut self) {
        self.apply();
        if let Some(la) = &mut self.la {
            la.commit();
        }
    }

    fn telemetry(&self) -> ModelTelemetry {
        let la = self.la.as_ref().map(|l| l.telemetry()).unwrap_or_default();
        ModelTelemetry {
            gain_reduction_db: self.gr + la.gain_reduction_db,
            engaged: self.gr > 0.01 || la.engaged,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f32::consts::TAU;

    const FS: f64 = 48_000.0;

    fn make(kind: PseKind, p: &[(u16, f32)]) -> SourceExtractor {
        let mut m = SourceExtractor::new(kind);
        m.prepare(FS, 64);
        for &(s, v) in p {
            m.set_param(s, ParamValue::Float(v));
        }
        m.commit();
        m.reset();
        m
    }

    fn amp(db: f32) -> f32 {
        10f32.powf(db / 20.0)
    }

    fn sine(db: f32, n: usize) -> Vec<f32> {
        (0..n)
            .map(|i| amp(db) * (TAU * 997.0 * i as f32 / FS as f32).sin())
            .collect()
    }

    fn run(m: &mut dyn Model, l: &mut [f32], r: &mut [f32], key: Option<(&[f32], &[f32])>) {
        let mut o = 0;
        while o < l.len() {
            let e = (o + 64).min(l.len());
            let k = key.map(|(a, b)| (&a[o..e], &b[o..e]));
            m.process(&mut l[o..e], &mut r[o..e], k);
            o = e;
        }
    }

    fn rms_db(x: &[f32]) -> f32 {
        let s: f64 = x.iter().map(|v| (*v as f64).powi(2)).sum::<f64>() / x.len() as f64;
        (10.0 * s.log10()) as f32
    }

    fn gain_db(m: &mut dyn Model, db: f32) -> f32 {
        let mut l = sine(db, 48_000);
        let mut r = l.clone();
        let r0 = rms_db(&l[24_000..]);
        run(m, &mut l, &mut r, None);
        rms_db(&l[24_000..]) - r0
    }

    #[test]
    fn static_law() {
        assert_eq!(static_gr_db(-10.0, -20.0, 12.0), 0.0);
        assert!((static_gr_db(-26.0, -20.0, 20.0) - 6.0).abs() < 1e-6);
        assert!((static_gr_db(-23.0, -20.0, 20.0) - 3.0).abs() < 1e-6); // knee edge continuous
        assert!((static_gr_db(-20.0, -20.0, 20.0) - 0.75).abs() < 1e-6);
        assert_eq!(static_gr_db(-90.0, -20.0, 12.0), 12.0);
        assert_eq!(static_gr_db(-90.0, -20.0, 0.0), 0.0);
    }

    #[test]
    fn passes_above_and_attenuates_below_by_depth() {
        let mut m = make(PseKind::Pse, &[(1, -20.0), (2, 12.0)]);
        assert!(gain_db(&mut m, -6.0).abs() < 0.1);
        m.reset();
        let g = gain_db(&mut m, -60.0);
        assert!((g + 12.0).abs() < 0.2, "{g}");
        assert!(m.telemetry().engaged);
        assert!((m.telemetry().gain_reduction_db - 12.0).abs() < 0.2);
    }

    #[test]
    fn depth_zero_is_transparent_and_depth_scales() {
        let mut m = make(PseKind::Pse, &[(1, -10.0), (2, 0.0)]);
        assert!(gain_db(&mut m, -70.0).abs() < 0.01);
        m.set_param(2, ParamValue::Float(6.0));
        m.commit();
        m.reset();
        assert!((gain_db(&mut m, -70.0) + 6.0).abs() < 0.2);
    }

    #[test]
    fn soft_two_to_one_slope() {
        // Peak detector: 10 dB under threshold -> ~10 dB GR (2:1 expansion), 16 dB under -> 16 dB.
        let mut m = make(PseKind::Pse, &[(1, -20.0), (2, 20.0), (4, 1.0)]);
        let g1 = gain_db(&mut m, -30.0);
        m.reset();
        let g2 = gain_db(&mut m, -36.0);
        assert!((g1 + 10.0).abs() < 0.5, "{g1}");
        assert!((g2 + 16.0).abs() < 0.5, "{g2}");
    }

    #[test]
    fn peak_vs_rms_detector() {
        let mut a = make(PseKind::Pse, &[(1, -17.0), (2, 20.0)]);
        let rms = gain_db(&mut a, -18.0);
        let mut b = make(PseKind::Pse, &[(1, -17.0), (2, 20.0), (4, 1.0)]);
        let pk = gain_db(&mut b, -18.0);
        assert!(rms < pk - 1.5, "rms {rms} peak {pk}");
    }

    /// Samples after the signal stops until GR reaches 90 % of the depth.
    fn close_time(fast: f32) -> usize {
        let mut m = make(PseKind::Pse, &[(1, -20.0), (2, 12.0), (3, fast), (4, 1.0)]);
        let mut x = sine(-6.0, 9600);
        let mut xr = x.clone();
        run(&mut m, &mut x, &mut xr, None);
        let mut t = 0;
        while m.telemetry().gain_reduction_db < 10.8 && t < 48_000 {
            let mut a = [0.0f32; 16];
            let mut b = [0.0f32; 16];
            m.process(&mut a, &mut b, None);
            t += 16;
        }
        t
    }

    #[test]
    fn fast_switch_speeds_up_closing() {
        let (s, f) = (close_time(0.0), close_time(1.0));
        assert!(f * 2 < s, "slow {s} fast {f}");
    }

    #[test]
    fn external_key_and_stereo_link() {
        let mut m = make(PseKind::Pse, &[(1, -20.0), (2, 12.0)]);
        let quiet = sine(-50.0, 24_000);
        let loud = sine(-6.0, 24_000);
        let zero = vec![0.0f32; 24_000];
        let (mut l, mut r) = (quiet.clone(), quiet.clone());
        run(&mut m, &mut l, &mut r, Some((&loud, &loud)));
        assert!((rms_db(&l[12_000..]) - rms_db(&quiet[12_000..])).abs() < 0.2);
        m.reset();
        let (mut l, mut r) = (quiet.clone(), quiet.clone());
        run(&mut m, &mut l, &mut r, Some((&zero, &zero)));
        assert!(rms_db(&l[12_000..]) - rms_db(&quiet[12_000..]) < -11.0);
        // stereo link: loud left holds the quiet right open
        m.reset();
        let (mut l, mut r) = (loud.clone(), quiet.clone());
        run(&mut m, &mut l, &mut r, None);
        assert!((rms_db(&r[12_000..]) - rms_db(&quiet[12_000..])).abs() < 0.2);
    }

    #[test]
    fn combo_is_pse_then_la() {
        let mut c = make(PseKind::Combo, &[(1, -20.0), (2, 12.0)]);
        assert!(!c.is_stub());
        assert_eq!(c.id(), "CMB");
        // leveler idle (cpeak 0): below threshold -> PSE depth
        let g = gain_db(&mut c, -60.0);
        assert!((g + 12.0).abs() < 0.5, "{g}");
        // leveler engaged: hot signal above the PSE threshold is compressed, idle one is not
        let mut idle = make(PseKind::Combo, &[(1, -36.0)]);
        let mut act = make(PseKind::Combo, &[(1, -36.0), (6, 100.0)]);
        let gi = gain_db(&mut idle, -6.0);
        let ga = gain_db(&mut act, -6.0);
        assert!(gi.abs() < 0.5, "{gi}");
        assert!(ga < gi - 3.0, "idle {gi} active {ga}");
        assert!(act.telemetry().gain_reduction_db > 3.0);
        assert_eq!(act.param(5), Some(40.0));
        assert_eq!(act.param(6), Some(100.0));
    }

    #[test]
    fn params_and_defaults() {
        let p = SourceExtractor::new(PseKind::Pse);
        assert_eq!(p.param(1), Some(0.0));
        assert_eq!(p.param(2), Some(12.0));
        assert_eq!(p.param(5), None);
        let c = SourceExtractor::new(PseKind::Combo);
        assert_eq!(c.param(1), Some(-36.0));
        assert_eq!(c.param(6), Some(0.0));
        let mut p = p;
        p.set_param(1, ParamValue::Float(99.0));
        p.set_param(2, ParamValue::Float(-3.0));
        assert_eq!((p.param(1), p.param(2)), (Some(12.0), Some(0.0)));
    }
}
