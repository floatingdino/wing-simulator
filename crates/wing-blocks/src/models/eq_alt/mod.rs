//! Alternate EQ models (T04G: `SOUL`, `E88`, `E84`; T04H: `F110`, `PULSAR`, `MACH4`, `PIA`).
//!
//! **Linear response = WING-Edit's own coefficients.** Each model is a port of WING-Edit 3.3.4's
//! `ae::eq_parchg_<model>` (the function that computes the DSP biquads the editor also draws, see
//! `docs/research/reference-curves.md`), decoded from the arm64 disassembly: knob -> (f, gain, Q) maps are the
//! editor's own static tables ([`tables`], extracted verbatim) with the same interpolation, and the section
//! designs are WING's `Cbiquad` primitives ([`prims`]: an Orfanidis peaking EQ with prescribed Nyquist gain,
//! bilinear 2nd-order shelves with separate numerator/denominator Q, 1st-order shelves, and per-model inline
//! sections). Arithmetic mirrors the original (float32, fused multiply-adds, float64 where it uses double), so
//! the coefficients agree with WING-Edit's to float32 rounding; `fidelity_*` tests check every curve of
//! `data/reference-curves/eq-<model>.json`.
//!
//! Topology: cascaded sections, except `MACH4` (a **parallel** bank: H = sum of sections, as
//! `eq_respf_maag4` evaluates it on channels).
//!
//! **Nonlinear character:** none. WING-Edit models these EQs purely as linear biquads, and models.md §4 gives
//! no evidence of transformer/tube colour in WING's implementation (the vintage colour of the hardware, e.g.
//! the Pultec make-up stage or Neve transformers, is not in the parameter set or the coefficient code). Not added.
//!
//! **`mix` (wrapper, 0..125 %):** WING-Edit passes `mix/100` into every `parchg`, which scales the knob gains
//! (125 % = exaggerated EQ, extra table columns). The slot host currently implements `mix` as dry/wet, so these
//! models run at 100 %; [`AltEq::set_gain_scale`] exposes the WING behaviour for a host that wants it.
//!
//! Real-time behaviour: `prepare` sizes nothing (all state is inline arrays), so nothing allocates after
//! construction. Continuous parameters (knobs, gains) glide with a 20 ms one-pole; while gliding the sections
//! are redesigned every [`SUB_BLOCK`] samples and the coefficients ramp linearly per sample. Discrete
//! parameters (enum selectors, switches) crossfade from the old filter to the new one over 10 ms. Filters run in
//! float64 transposed direct form II (the reference coefficients are float32; 20 Hz sections need the headroom).

// The ports keep WING-Edit's `fmax`/`fmin` sequences (NaN behaviour, operation order) instead of `clamp`.
#![allow(clippy::manual_clamp)]

mod e84;
mod e88;
mod f110;
mod mach4;
mod pia;
pub mod prims;
mod pulsar;
mod soul;
#[allow(clippy::all)]
mod tables;

use crate::model::{Model, ParamValue};
use prims::{Bq, Ctx, UNITY};
use wing_dsp::smooth::one_pole_coeff;

/// Coefficient redesign interval while a parameter glides (samples).
pub const SUB_BLOCK: usize = 16;
/// Maximum number of sections of any model (PIA: 10).
pub const MAX_SECS: usize = 10;
/// Maximum number of parameter slots of any model (PULSAR: 16).
pub const MAX_PARAMS: usize = 16;

const PARAM_TAU_S: f64 = 0.02;
const XFADE_S: f64 = 0.01;

/// Section count and topology produced by a design function.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Layout {
    pub n: usize,
    pub parallel: bool,
}

impl Layout {
    pub(crate) const fn cascade(n: usize) -> Self {
        Self { n, parallel: false }
    }
    pub(crate) const fn parallel(n: usize) -> Self {
        Self { n, parallel: true }
    }
}

/// The seven alternate EQ models.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AltKind {
    Soul,
    E88,
    E84,
    F110,
    Pulsar,
    Mach4,
    Pia,
}

/// Parameter slot description: key and, for enums, the schema token list (`None` = number / bool).
pub type SlotDesc = (&'static str, Option<&'static [&'static str]>);

const LOHI: &[&str] = &["LOW", "HIGH"];
const BELLSHV: &[&str] = &["BELL", "SHELV"];

impl AltKind {
    pub const ALL: [AltKind; 7] = [
        AltKind::Soul,
        AltKind::E88,
        AltKind::E84,
        AltKind::F110,
        AltKind::Pulsar,
        AltKind::Mach4,
        AltKind::Pia,
    ];

    pub fn from_id(id: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|k| k.id() == id)
    }

    /// Snapshot model id.
    pub fn id(self) -> &'static str {
        match self {
            AltKind::Soul => "SOUL",
            AltKind::E88 => "E88",
            AltKind::E84 => "E84",
            AltKind::F110 => "F110",
            AltKind::Pulsar => "PULSAR",
            AltKind::Mach4 => "MACH4",
            AltKind::Pia => "PIA",
        }
    }

    /// Parameter slots in schema order (slot = index + 1).
    pub fn slots(self) -> &'static [SlotDesc] {
        match self {
            AltKind::Soul => &[
                ("lf", None),
                ("lg", None),
                ("lmf", None),
                ("lmf3", None),
                ("lmq", None),
                ("lmg", None),
                ("hmf", None),
                ("hmf3", None),
                ("hmq", None),
                ("hmg", None),
                ("hf", None),
                ("hg", None),
            ],
            AltKind::E88 => &[
                ("lf", None),
                ("lg", None),
                ("lq", Some(LOHI)),
                ("lt", Some(BELLSHV)),
                ("lmf", None),
                ("lmg", None),
                ("lmq", None),
                ("hmf", None),
                ("hmg", None),
                ("hmq", None),
                ("hf", None),
                ("hg", None),
                ("hq", Some(LOHI)),
                ("ht", Some(BELLSHV)),
            ],
            AltKind::E84 => &[
                ("g", None),
                ("lf", Some(&["OFF", "35", "60", "110", "220"])),
                ("lg", None),
                (
                    "mf",
                    Some(&["OFF", "350", "700", "1k6", "3k2", "4k8", "7k2"]),
                ),
                ("mg", None),
                ("mq", Some(LOHI)),
                ("hf", Some(&["10k", "12k", "16k", "OFF"])),
                ("hg", None),
            ],
            AltKind::F110 => &[
                ("peq", None),
                ("lmf", None),
                ("lmg", None),
                ("lmq", None),
                ("lmf3", None),
                ("hmf", None),
                ("hmg", None),
                ("hmq", None),
                ("hmf3", None),
                ("shv", None),
                ("lf", Some(&["33", "56", "95", "160", "270", "460"])),
                ("lg", None),
                ("hf", Some(&["3k3", "4k7", "6k8", "10k", "15k", "18k"])),
                ("hg", None),
                ("g", None),
            ],
            AltKind::Pulsar => &[
                ("eq1", None),
                ("1lb", None),
                ("1latt", None),
                ("1lf", Some(&["20", "30", "60", "100"])),
                ("1hw", None),
                ("1hb", None),
                ("1hf", Some(&["3k", "4k", "5k", "8k", "10k", "12k", "16k"])),
                ("1hatt", None),
                ("1hattf", Some(&["5k", "10k", "20k"])),
                ("eq5", None),
                ("5lb", None),
                ("5lf", Some(&["200", "300", "500", "700", "1k"])),
                ("5md", None),
                (
                    "5mf",
                    Some(&[
                        "200", "300", "500", "700", "1k", "1k5", "2k", "3k", "4k", "5k", "7k",
                    ]),
                ),
                ("5hb", None),
                ("5hf", Some(&["1k5", "2k", "3k", "4k", "5k"])),
            ],
            AltKind::Mach4 => &[
                ("sub", None),
                ("40", None),
                ("160", None),
                ("650", None),
                ("2k5", None),
                ("air", None),
                ("airm", Some(&["OFF", "2k5", "5k", "10k", "20k", "40k"])),
                ("again", None),
            ],
            AltKind::Pia => &[
                ("g", None),
                ("31", None),
                ("63", None),
                ("125", None),
                ("250", None),
                ("500", None),
                ("1k", None),
                ("2k", None),
                ("4k", None),
                ("8k", None),
                ("16k", None),
            ],
        }
    }

    /// Slots that are switched (enums and on/off switches) rather than continuous.
    fn is_discrete(self, idx: usize) -> bool {
        let (key, en) = self.slots()[idx];
        en.is_some()
            || matches!(
                (self, key),
                (AltKind::Soul, "lmf3" | "hmf3")
                    | (AltKind::F110, "peq" | "lmf3" | "hmf3" | "shv")
                    | (AltKind::Pulsar, "eq1" | "eq5")
                    | (AltKind::Mach4, "again")
            )
    }

    /// Valid range of a continuous slot (values are clamped to it).
    fn range(self, idx: usize) -> (f32, f32) {
        let (key, en) = self.slots()[idx];
        if let Some(e) = en {
            return (0.0, (e.len() - 1) as f32);
        }
        if self.is_discrete(idx) {
            return (0.0, 1.0);
        }
        match (self, key) {
            (AltKind::E84, "g") => (-20.0, 20.0),
            (AltKind::F110, "g") => (-18.0, 18.0),
            (AltKind::Pia, _) => (-12.0, 12.0),
            (AltKind::Pulsar, _) | (AltKind::Mach4, "air") => (0.0, 10.0),
            (_, k) if k.ends_with('g') || matches!(self, AltKind::Mach4) => (-5.0, 5.0),
            _ => (0.0, 10.0),
        }
    }

    /// WING-Edit / schema defaults per slot (enums as index).
    pub fn defaults(self) -> &'static [f32] {
        match self {
            AltKind::Soul => &soul::DEFAULTS,
            AltKind::E88 => &e88::DEFAULTS,
            AltKind::E84 => &e84::DEFAULTS,
            AltKind::F110 => &f110::DEFAULTS,
            AltKind::Pulsar => &pulsar::DEFAULTS,
            AltKind::Mach4 => &mach4::DEFAULTS,
            AltKind::Pia => &pia::DEFAULTS,
        }
    }

    /// Compute WING's sections for parameter values `p` (slot order, enums as index) at gain scale `m`
    /// (`mix/100`). Writes `out[..layout.n]`.
    pub fn design(self, ctx: &Ctx, p: &[f32], m: f32, out: &mut [Bq; MAX_SECS]) -> Layout {
        match self {
            AltKind::Soul => soul::design(ctx, p, m, out),
            AltKind::E88 => e88::design(ctx, p, m, out),
            AltKind::E84 => e84::design(ctx, p, m, out),
            AltKind::F110 => f110::design(ctx, p, m, out),
            AltKind::Pulsar => pulsar::design(ctx, p, m, out),
            AltKind::Mach4 => mach4::design(ctx, p, m, out),
            AltKind::Pia => pia::design(ctx, p, m, out),
        }
    }
}

// ---- runtime ----------------------------------------------------------------------------------------------

/// Stereo TDF-II section (float64) with a per-sample linear coefficient ramp.
#[derive(Clone, Copy, Debug, Default)]
struct Sec {
    c: [f64; 5],
    d: [f64; 5],
    t: [f64; 5],
    s: [[f64; 2]; 2],
}

#[inline(always)]
fn tick(c: &[f64; 5], s: &mut [f64; 2], x: f64) -> f64 {
    let y = c[0] * x + s[0];
    s[0] = c[1] * x - c[3] * y + s[1];
    s[1] = c[2] * x - c[4] * y;
    y
}

fn carr(q: &Bq) -> [f64; 5] {
    q.map(f64::from)
}

/// A filter (cascade or parallel bank) of up to [`MAX_SECS`] sections.
#[derive(Clone, Copy, Debug)]
struct Chain {
    secs: [Sec; MAX_SECS],
    layout: Layout,
    ramping: bool,
}

impl Default for Chain {
    fn default() -> Self {
        Self {
            secs: [Sec::default(); MAX_SECS],
            layout: Layout::cascade(0),
            ramping: false,
        }
    }
}

impl Chain {
    fn load(&mut self, q: &[Bq; MAX_SECS], layout: Layout) {
        self.layout = layout;
        self.ramping = false;
        for (sec, c) in self.secs.iter_mut().zip(q) {
            sec.c = carr(c);
            sec.t = sec.c;
        }
    }

    fn ramp_to(&mut self, q: &[Bq; MAX_SECS], n: usize) {
        let inv = 1.0 / n as f64;
        for (sec, c) in self.secs.iter_mut().zip(q) {
            sec.t = carr(c);
            for k in 0..5 {
                sec.d[k] = (sec.t[k] - sec.c[k]) * inv;
            }
        }
        self.ramping = true;
    }

    fn clear(&mut self) {
        for sec in &mut self.secs {
            sec.s = [[0.0; 2]; 2];
        }
    }

    #[inline(always)]
    fn sample(&mut self, l: f64, r: f64) -> (f64, f64) {
        let n = self.layout.n;
        let ramp = self.ramping;
        let secs = &mut self.secs[..n];
        if self.layout.parallel {
            let (mut sl, mut sr) = (0.0, 0.0);
            for sec in secs {
                if ramp {
                    for k in 0..5 {
                        sec.c[k] += sec.d[k];
                    }
                }
                let [s0, s1] = &mut sec.s;
                sl += tick(&sec.c, s0, l);
                sr += tick(&sec.c, s1, r);
            }
            (sl, sr)
        } else {
            let (mut l, mut r) = (l, r);
            for sec in secs {
                if ramp {
                    for k in 0..5 {
                        sec.c[k] += sec.d[k];
                    }
                }
                let [s0, s1] = &mut sec.s;
                l = tick(&sec.c, s0, l);
                r = tick(&sec.c, s1, r);
            }
            (l, r)
        }
    }

    fn end_ramp(&mut self) {
        if self.ramping {
            for sec in &mut self.secs[..self.layout.n] {
                sec.c = sec.t;
            }
            self.ramping = false;
        }
    }

    fn flush_denormals(&mut self) {
        for sec in &mut self.secs[..self.layout.n] {
            for s in sec.s.iter_mut().flatten() {
                if s.abs() < 1e-25 {
                    *s = 0.0;
                }
            }
        }
    }

    fn response(&self, fs: f64, f: f64) -> wing_dsp::Complex {
        use wing_dsp::Complex;
        let w = 2.0 * core::f64::consts::PI * f / fs;
        let z1 = Complex::new(w.cos(), -w.sin());
        let z2 = z1 * z1;
        let mut h = if self.layout.parallel {
            Complex::new(0.0, 0.0)
        } else {
            Complex::new(1.0, 0.0)
        };
        for sec in &self.secs[..self.layout.n] {
            let c = &sec.t;
            let num = Complex::new(c[0], 0.0) + z1 * c[1] + z2 * c[2];
            let den = Complex::new(1.0, 0.0) + z1 * c[3] + z2 * c[4];
            let hs = num / den;
            h = if self.layout.parallel { h + hs } else { h * hs };
        }
        h
    }
}

/// One alternate EQ model instance (implements [`Model`]).
pub struct AltEq {
    kind: AltKind,
    ctx: Ctx,
    fs: f64,
    nparams: usize,
    /// Staged values (slot order, enums as index).
    target: [f32; MAX_PARAMS],
    /// Values the current coefficients were designed from.
    cur: [f32; MAX_PARAMS],
    gain_scale: f32,
    smooth_a: f32,
    chains: [Chain; 2],
    active: usize,
    fade_pos: usize,
    fade_len: usize,
    fading: bool,
    scratch: [Bq; MAX_SECS],
    dirty: bool,
}

impl AltEq {
    pub fn new(kind: AltKind) -> Self {
        let mut m = Self {
            kind,
            ctx: Ctx::new(48_000.0),
            fs: 48_000.0,
            nparams: kind.slots().len(),
            target: [0.0; MAX_PARAMS],
            cur: [0.0; MAX_PARAMS],
            gain_scale: 1.0,
            smooth_a: 0.0,
            chains: [Chain::default(); 2],
            active: 0,
            fade_pos: 0,
            fade_len: 1,
            fading: false,
            scratch: [UNITY; MAX_SECS],
            dirty: false,
        };
        m.target[..m.nparams].copy_from_slice(kind.defaults());
        m.prepare(48_000.0, 64);
        m
    }

    pub fn kind(&self) -> AltKind {
        self.kind
    }

    /// WING's `mix/100` gain scale (0..1.25; default 1). See the module doc.
    pub fn set_gain_scale(&mut self, m: f32) {
        let m = m.clamp(0.0, 1.25);
        if m != self.gain_scale {
            self.gain_scale = m;
            self.dirty = true;
        }
    }

    /// Current (target) value of slot index `idx`.
    pub fn value(&self, idx: usize) -> Option<f32> {
        (idx < self.nparams).then(|| self.target[idx])
    }

    /// Design sections for the staged values immediately (no smoothing), e.g. for display or tests.
    pub fn design_now(&self, out: &mut [Bq; MAX_SECS]) -> Layout {
        self.kind.design(
            &self.ctx,
            &self.target[..self.nparams],
            self.gain_scale,
            out,
        )
    }

    /// Complex response of the coefficients currently targeted by the filter.
    pub fn response(&self, f: f64) -> wing_dsp::Complex {
        self.chains[self.active].response(self.fs, f)
    }

    pub fn magnitude_db(&self, f: f64) -> f64 {
        20.0 * self.response(f).norm().max(1e-30).log10()
    }

    fn redesign(&mut self) -> Layout {
        self.kind.design(
            &self.ctx,
            &self.cur[..self.nparams],
            self.gain_scale,
            &mut self.scratch,
        )
    }

    /// Settle everything at the targets (no glide, no fade).
    fn snap(&mut self) {
        self.cur = self.target;
        let layout = self.redesign();
        let s = self.scratch;
        self.chains[self.active].load(&s, layout);
        self.fading = false;
        self.dirty = false;
    }

    /// Per sub-block control update: discrete changes start a crossfade, continuous ones glide.
    fn control(&mut self, n: usize) {
        if !self.dirty {
            return;
        }
        let mut discrete = false;
        let mut gliding = false;
        for i in 0..self.nparams {
            if self.cur[i] == self.target[i] {
                continue;
            }
            if self.kind.is_discrete(i) {
                self.cur[i] = self.target[i];
                discrete = true;
            } else {
                let d = self.target[i] - self.cur[i];
                if d.abs() < 1e-3 {
                    self.cur[i] = self.target[i];
                } else {
                    self.cur[i] += d * self.smooth_a;
                    gliding = true;
                }
            }
        }
        let layout = self.redesign();
        let s = self.scratch;
        if discrete {
            // New chain continues from the old chain's state; crossfade old -> new.
            let old = self.active;
            let new = 1 - old;
            self.chains[old].end_ramp();
            self.chains[new] = self.chains[old];
            self.chains[new].load(&s, layout);
            if layout != self.chains[old].layout {
                self.chains[new].clear();
            }
            self.active = new;
            self.fading = true;
            self.fade_pos = 0;
        } else {
            let ch = &mut self.chains[self.active];
            if layout == ch.layout {
                ch.ramp_to(&s, n);
            } else {
                ch.load(&s, layout);
            }
        }
        self.dirty = gliding || self.cur[..self.nparams] != self.target[..self.nparams];
        if !self.dirty {
            // Final coefficients are exactly the target design (ramp ends on it).
        }
    }
}

impl Model for AltEq {
    fn id(&self) -> &'static str {
        self.kind.id()
    }

    fn prepare(&mut self, sample_rate: f64, _max_block: usize) {
        self.fs = sample_rate;
        self.ctx = Ctx::new(sample_rate);
        let per_sub = SUB_BLOCK as f64;
        self.smooth_a = (1.0 - one_pole_coeff(PARAM_TAU_S, sample_rate / per_sub)) as f32;
        self.fade_len = ((XFADE_S * sample_rate) as usize).max(1);
        for c in &mut self.chains {
            c.clear();
        }
        self.snap();
    }

    fn process(&mut self, left: &mut [f32], right: &mut [f32], _sc: Option<(&[f32], &[f32])>) {
        let n = left.len().min(right.len());
        let mut pos = 0;
        while pos < n {
            let len = SUB_BLOCK.min(n - pos);
            self.control(len);
            let (l, r) = (&mut left[pos..pos + len], &mut right[pos..pos + len]);
            if self.fading {
                let inv = 1.0 / self.fade_len as f64;
                let (a, b) = (self.active, 1 - self.active);
                for (xl, xr) in l.iter_mut().zip(r.iter_mut()) {
                    let (il, ir) = (*xl as f64, *xr as f64);
                    let (nl, nr) = self.chains[a].sample(il, ir);
                    let (ol, or) = self.chains[b].sample(il, ir);
                    let g = (self.fade_pos as f64 * inv).min(1.0);
                    self.fade_pos += 1;
                    *xl = (ol + (nl - ol) * g) as f32;
                    *xr = (or + (nr - or) * g) as f32;
                }
                if self.fade_pos >= self.fade_len {
                    self.fading = false;
                }
            } else {
                let ch = &mut self.chains[self.active];
                for (xl, xr) in l.iter_mut().zip(r.iter_mut()) {
                    let (yl, yr) = ch.sample(*xl as f64, *xr as f64);
                    *xl = yl as f32;
                    *xr = yr as f32;
                }
            }
            for c in &mut self.chains {
                c.end_ramp();
            }
            pos += len;
        }
        self.chains[self.active].flush_denormals();
    }

    fn reset(&mut self) {
        for c in &mut self.chains {
            c.clear();
        }
        self.snap();
    }

    fn set_param(&mut self, slot: u16, value: ParamValue) {
        let idx = slot as usize;
        if idx == 0 || idx > self.nparams {
            return;
        }
        let i = idx - 1;
        let (lo, hi) = self.kind.range(i);
        let v = if self.kind.slots()[i].1.is_some() || self.kind.is_discrete(i) {
            (value.as_i32() as f32).clamp(lo, hi)
        } else {
            value.as_f32().clamp(lo, hi)
        };
        if v.is_finite() && v != self.target[i] {
            self.target[i] = v;
            self.dirty = true;
        }
    }

    fn param(&self, slot: u16) -> Option<f32> {
        (slot as usize).checked_sub(1).and_then(|i| self.value(i))
    }

    fn load_defaults(&mut self) {
        self.target[..self.nparams].copy_from_slice(self.kind.defaults());
        self.dirty = true;
    }

    fn commit(&mut self) {}
}

#[cfg(test)]
mod tests;
