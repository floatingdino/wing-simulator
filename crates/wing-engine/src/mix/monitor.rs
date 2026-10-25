//! Monitor section (T038): solo busses and the two monitor outputs `cfg/mon/1` (A, "PHONES") and
//! `cfg/mon/2` (B, "SPEAKERS").
//!
//! ```text
//! per monitor m:
//!   source (cfg/mon/m/src: OFF, MAIN.1-4, MTX.1-8, BUS.1-16, AUX.1-8; POST signal)
//!     x srclvl            (no solo active)
//!     x srcmix            (any solo active into m: default -oo = solo replaces the source)
//!   + solo bus m = sum of soloed strips (MixShared flags): PFL (pre-fader) x -pfldim dB,
//!                  or AFL (channels: post fader before pan; bus/main/mtx: post fader + pan)
//!   -> monitor EQ (eq/on: low shelf lsg/lsf, 6 peaking 1..6 g/f/q, high shelf hsg/hsf)
//!   -> level lvl, dim (cfg/solo/$dim: -dim dB), pan/wid (balance law), mono (cfg/solo/$mono),
//!      inv (polarity)
//!   -> delay (dly/on, dly/m metres at 343 m/s)
//!   -> limiter (lim threshold dBFS, instant attack, 50 ms release)
//!   -> MON leg 2m-1 / 2m (output channels 2(m-1), 2(m-1)+1)
//! ```
//!
//! Solo priority: every solo path and the source gain are 5 ms ramps ([`law::Ramp`]), so entering
//! and leaving solo (and switching PFL/AFL) crossfade without clicks; EQ coefficients ramp per
//! sample ([`SmoothedBiquad`]). Delay changes jump (not smoothed). `eqbdtrim` (band solo),
//! `dirin` (direct input) and `$flip` are not modelled [I]; the shelf Q is 0.707 [I].

use std::sync::Arc;

use wing_dsp::biquad::{Coeffs, SmoothedBiquad};
use wing_dsp::delay::DelayLine;

use crate::graph::{NodeId, Port};
use crate::graph::{NodeIo, ProcessCtx, Processor};
use crate::params::{BindError, BlockLayout, ParamMapBuilder, ParamMsg};
use crate::routing::{Strip, STRIP_COUNT};

use super::law::{self, Matrix, PanLaw, Ramp, ZERO};
use super::logic::{flag, MixShared};
use super::nodes::{InputSignal, MixSignal};
use super::MixConfig;

/// Monitor output channels (= MON legs 1..4): mon 1 L/R, mon 2 L/R.
pub const MONITOR_OUTPUTS: u32 = 4;
/// Node output channels: the [`MONITOR_OUTPUTS`], then the solo busses (sum of the soloed
/// strips after their solo gains, before source, EQ and level; for metering): solo 1 L/R,
/// solo 2 L/R.
pub const MONITOR_CHANNELS: u32 = 8;
/// Input ports per input strip (Pfl, Afl, Post; L/R).
const IN_PORTS: usize = 6;
/// Input ports per bus/main/matrix (Pfl, Post; L/R).
const MIX_PORTS: usize = 4;
const INPUT_STRIPS: usize = 48;
const MIX_BASE: usize = INPUT_STRIPS * IN_PORTS;
/// Total input ports of the monitor node.
pub const MONITOR_INPUTS: usize = MIX_BASE + (STRIP_COUNT - INPUT_STRIPS) * MIX_PORTS;

const SPEED_OF_SOUND: f32 = 343.0;
const MAX_DIST_M: f32 = 100.0;
const LIM_RELEASE_S: f32 = 0.05;
const EQ_RAMP: u32 = 64;
const SHELF_Q: f64 = std::f64::consts::FRAC_1_SQRT_2;

/// Parameter block ids of the monitor node.
pub mod block {
    /// `cfg/mon/<m>` = `4 * (m - 1) + OWN`.
    pub const OWN: u8 = 0;
    /// `cfg/mon/<m>/eq`.
    pub const EQ: u8 = 1;
    /// `cfg/mon/<m>/dly`.
    pub const DLY: u8 = 2;
    /// `cfg/solo` (`$dim`, `$mono`).
    pub const SOLO: u8 = 16;
    pub const fn mon(m: usize, b: u8) -> u8 {
        4 * m as u8 + b
    }
}

/// Parameter ids.
pub mod param {
    pub const LVL: u16 = 0;
    pub const INV: u16 = 1;
    pub const PAN: u16 = 2;
    pub const WID: u16 = 3;
    pub const LIM: u16 = 4;
    pub const DIM: u16 = 5;
    pub const PFLDIM: u16 = 6;
    pub const SRCLVL: u16 = 7;
    pub const SRCMIX: u16 = 8;
    pub const SRC: u16 = 9;
    // eq: on = 0, then (g, f, q) per band: ls 1..3 (q unused), peaks, hs
    pub const EQ_ON: u16 = 0;
    /// `EQ_BAND + 3 * band + {0 g, 1 f, 2 q}`, bands 0 = ls, 1..=6 peaks, 7 = hs.
    pub const EQ_BAND: u16 = 1;
    pub const DLY_ON: u16 = 0;
    pub const DLY_M: u16 = 1;
    pub const SOLO_DIM: u16 = 0;
    pub const SOLO_MONO: u16 = 1;
}

const OWN_LAYOUT: BlockLayout = BlockLayout {
    params: &[
        ("lvl", param::LVL),
        ("inv", param::INV),
        ("pan", param::PAN),
        ("wid", param::WID),
        ("lim", param::LIM),
        ("dim", param::DIM),
        ("pfldim", param::PFLDIM),
        ("srclvl", param::SRCLVL),
        ("srcmix", param::SRCMIX),
        ("src", param::SRC),
    ],
    targets: &[],
};

const fn band(b: u16, k: u16) -> u16 {
    param::EQ_BAND + 3 * b + k
}

const EQ_LAYOUT: BlockLayout = BlockLayout {
    params: &[
        ("on", param::EQ_ON),
        ("lsg", band(0, 0)),
        ("lsf", band(0, 1)),
        ("1g", band(1, 0)),
        ("1f", band(1, 1)),
        ("1q", band(1, 2)),
        ("2g", band(2, 0)),
        ("2f", band(2, 1)),
        ("2q", band(2, 2)),
        ("3g", band(3, 0)),
        ("3f", band(3, 1)),
        ("3q", band(3, 2)),
        ("4g", band(4, 0)),
        ("4f", band(4, 1)),
        ("4q", band(4, 2)),
        ("5g", band(5, 0)),
        ("5f", band(5, 1)),
        ("5q", band(5, 2)),
        ("6g", band(6, 0)),
        ("6f", band(6, 1)),
        ("6q", band(6, 2)),
        ("hsg", band(7, 0)),
        ("hsf", band(7, 1)),
    ],
    targets: &[],
};

const DLY_LAYOUT: BlockLayout = BlockLayout {
    params: &[("on", param::DLY_ON), ("m", param::DLY_M)],
    targets: &[],
};

const SOLO_LAYOUT: BlockLayout = BlockLayout {
    params: &[("$dim", param::SOLO_DIM), ("$mono", param::SOLO_MONO)],
    targets: &[],
};

/// Bind `cfg/mon/1|2` and `cfg/solo/$dim|$mono` to the monitor node.
pub fn bind_monitor_params(b: &mut ParamMapBuilder, node: NodeId) -> Result<(), BindError> {
    for m in 0..2 {
        let p = format!("cfg/mon/{}", m + 1);
        b.bind_block(&p, node, block::mon(m, block::OWN), &OWN_LAYOUT)?;
        b.bind_block(
            &format!("{p}/eq"),
            node,
            block::mon(m, block::EQ),
            &EQ_LAYOUT,
        )?;
        b.bind_block(
            &format!("{p}/dly"),
            node,
            block::mon(m, block::DLY),
            &DLY_LAYOUT,
        )?;
    }
    b.bind_block("cfg/solo", node, block::SOLO, &SOLO_LAYOUT)
}

/// Input ports of the monitor node, given the strip nodes (`strip_node(i)` for
/// [`Strip::index`] `i`).
pub fn monitor_inputs(strip_node: impl Fn(usize) -> NodeId) -> Vec<Port> {
    let mut v = Vec::with_capacity(MONITOR_INPUTS);
    for i in 0..STRIP_COUNT {
        let n = strip_node(i);
        let sigs: &[u32] = if i < INPUT_STRIPS {
            &[
                InputSignal::Pfl as u32,
                InputSignal::Afl as u32,
                InputSignal::Post as u32,
            ]
        } else {
            &[MixSignal::Pfl as u32, MixSignal::Post as u32]
        };
        for &s in sigs {
            v.push(Port::node(n, 2 * s));
            v.push(Port::node(n, 2 * s + 1));
        }
    }
    debug_assert_eq!(v.len(), MONITOR_INPUTS);
    v
}

/// Port index (L) of strip `i`'s PFL / AFL / POST signal.
fn port_of(i: usize, sig: Sig) -> usize {
    if i < INPUT_STRIPS {
        IN_PORTS * i
            + match sig {
                Sig::Pfl => 0,
                Sig::Afl => 2,
                Sig::Post => 4,
            }
    } else {
        MIX_BASE
            + MIX_PORTS * (i - INPUT_STRIPS)
            + match sig {
                Sig::Pfl => 0,
                Sig::Afl | Sig::Post => 2,
            }
    }
}

#[derive(Clone, Copy)]
enum Sig {
    Pfl,
    Afl,
    Post,
}

/// Strip index of a `cfg/mon/*/src` enum value.
pub fn source_strip(src: u32) -> Option<usize> {
    let s = match src {
        1..=4 => Strip::Main(src as u8),
        5..=12 => Strip::Mtx(src as u8 - 4),
        13..=28 => Strip::Bus(src as u8 - 12),
        29..=36 => Strip::Aux(src as u8 - 28),
        _ => return None,
    };
    s.index()
}

#[derive(Clone, Copy, Debug)]
struct MonParams {
    lvl: f32,
    inv: bool,
    pan: f32,
    wid: f32,
    lim: f32,
    dim: f32,
    pfldim: f32,
    srclvl: f32,
    srcmix: f32,
    src: u32,
    eq_on: bool,
    eq: [[f32; 3]; 8],
    dly_on: bool,
    dly_m: f32,
}

impl Default for MonParams {
    fn default() -> Self {
        let mut eq = [[0.0, 1000.0, 2.0]; 8];
        eq[0][1] = 60.0;
        eq[7][1] = 12_000.0;
        Self {
            lvl: 0.0,
            inv: false,
            pan: 0.0,
            wid: 100.0,
            lim: 0.0,
            dim: 20.0,
            pfldim: 12.0,
            srclvl: 0.0,
            srcmix: law::FADER_OFF_DB,
            src: 1,
            eq_on: false,
            eq,
            dly_on: false,
            dly_m: 0.1,
        }
    }
}

struct Monitor {
    p: MonParams,
    dirty: bool,
    eq_dirty: bool,
    /// Solo path gains per strip: [pfl, afl].
    solo: Box<[[Ramp<1>; 2]]>,
    /// Source gain (srclvl / srcmix).
    src: Ramp<1>,
    /// Output matrix (level, dim, pan, width, mono, polarity).
    out: Ramp<4>,
    eq: [[SmoothedBiquad; 8]; 2],
    dly: [DelayLine; 2],
    dly_samples: usize,
    lim_env: f32,
    solo_active: bool,
}

/// Jump solo ramps to their targets now (silent ramps are skipped in `process`, so they must
/// never be left unprimed: a later target would otherwise jump instead of ramping).
fn prime(solo: &mut [[Ramp<1>; 2]]) {
    for r in solo.iter_mut().flatten() {
        r.settle();
        r.step();
    }
}

/// The monitor section node. Inputs: [`monitor_inputs`]. Outputs: [`MONITOR_CHANNELS`].
pub struct MonitorNode {
    shared: Arc<MixShared>,
    seen: u32,
    flags: [u32; STRIP_COUNT],
    mons: [Monitor; 2],
    dim_on: bool,
    mono_on: bool,
    fs: f32,
    lim_rel: f32,
    sl: Box<[f32]>,
    sr: Box<[f32]>,
}

impl MonitorNode {
    /// Allocates (control thread).
    pub fn new(shared: Arc<MixShared>, cfg: &MixConfig) -> Self {
        let fs = cfg.sample_rate;
        let a = law::block_alpha(law::MIX_TAU_S, fs as f64, cfg.block_size);
        let max_dly = (MAX_DIST_M / SPEED_OF_SOUND * fs).ceil() as usize + 2;
        let mk = || Monitor {
            p: MonParams::default(),
            dirty: true,
            eq_dirty: true,
            solo: vec![[Ramp::new(a); 2]; STRIP_COUNT].into(),
            src: Ramp::new(a),
            out: Ramp::new(a),
            eq: [[SmoothedBiquad::new(Coeffs::IDENTITY, EQ_RAMP); 8]; 2],
            dly: [DelayLine::new(max_dly), DelayLine::new(max_dly)],
            dly_samples: 0,
            lim_env: 0.0,
            solo_active: false,
        };
        let mut n = Self {
            shared,
            seen: u32::MAX,
            flags: [0; STRIP_COUNT],
            mons: [mk(), mk()],
            dim_on: false,
            mono_on: false,
            fs,
            lim_rel: (-1.0 / (LIM_RELEASE_S * fs)).exp(),
            sl: vec![0.0; cfg.block_size].into(),
            sr: vec![0.0; cfg.block_size].into(),
        };
        n.read_shared();
        n.update(true);
        for m in &mut n.mons {
            prime(&mut m.solo);
        }
        n
    }

    /// Whether any solo currently reaches monitor `m` (1 or 2).
    pub fn solo_active(&self, m: usize) -> bool {
        self.mons[m - 1].solo_active
    }

    /// Jump every ramp to its target on the next block.
    pub fn settle(&mut self) {
        self.read_shared();
        self.update(true);
        for m in &mut self.mons {
            m.src.settle();
            m.out.settle();
            prime(&mut m.solo);
        }
    }

    fn read_shared(&mut self) -> bool {
        let g = self.shared.generation();
        if g == self.seen {
            return false;
        }
        self.seen = g;
        let mut changed = false;
        for i in 0..STRIP_COUNT {
            let f = self.shared.flags(i);
            changed |= f != self.flags[i];
            self.flags[i] = f;
        }
        changed
    }

    fn update(&mut self, solo_changed: bool) {
        let fs = self.fs as f64;
        let (dim_on, mono_on) = (self.dim_on, self.mono_on);
        for (k, m) in self.mons.iter_mut().enumerate() {
            let bit = if k == 0 {
                flag::SOLO_MON1
            } else {
                flag::SOLO_MON2
            };
            if solo_changed || m.dirty {
                let pfl_g = law::fader_gain(-m.p.pfldim);
                let mut active = false;
                for (i, r) in m.solo.iter_mut().enumerate() {
                    let f = self.flags[i];
                    let (pfl, afl) = if f & bit == 0 {
                        (0.0, 0.0)
                    } else if f & flag::AFL != 0 {
                        (0.0, 1.0)
                    } else {
                        (pfl_g, 0.0)
                    };
                    active |= f & bit != 0;
                    r[0].set([pfl]);
                    r[1].set([afl]);
                }
                m.solo_active = active;
                let sg = if active { m.p.srcmix } else { m.p.srclvl };
                let has_src = source_strip(m.p.src).is_some();
                m.src.set([if has_src { law::fader_gain(sg) } else { 0.0 }]);
            }
            if m.dirty {
                let p = &m.p;
                let mut g = law::fader_gain(p.lvl);
                if dim_on {
                    g *= law::fader_gain(-p.dim);
                }
                if p.inv {
                    g = -g;
                }
                let mut mat: Matrix = law::pan_width(PanLaw::Balance, p.pan, p.wid);
                if mono_on {
                    mat = law::mul(mat, law::MONO);
                }
                m.out.set(law::scale(mat, g));
                m.dly_samples = if p.dly_on {
                    ((p.dly_m.clamp(0.0, MAX_DIST_M) / SPEED_OF_SOUND) * self.fs).round() as usize
                } else {
                    0
                }
                .min(m.dly[0].max_delay());
                m.dirty = false;
            }
            if m.eq_dirty {
                let p = &m.p;
                for b in 0..8 {
                    let [g, f, q] = p.eq[b];
                    let f = (f as f64).clamp(10.0, 0.45 * fs);
                    let c = if !p.eq_on || g == 0.0 {
                        Coeffs::IDENTITY
                    } else {
                        match b {
                            0 => Coeffs::low_shelf(fs, f, SHELF_Q, g as f64),
                            7 => Coeffs::high_shelf(fs, f, SHELF_Q, g as f64),
                            _ => Coeffs::peak(fs, f, (q as f64).max(0.1), g as f64),
                        }
                    };
                    for ch in &mut m.eq {
                        ch[b].set_target(c);
                    }
                }
                m.eq_dirty = false;
            }
        }
    }

    fn apply(&mut self, msg: &ParamMsg) {
        let ParamMsg::Set {
            block: b,
            param: p,
            value: v,
        } = *msg
        else {
            if let ParamMsg::Commit = msg {
                self.update(false);
            }
            return;
        };
        if b == block::SOLO {
            match p {
                param::SOLO_DIM => self.dim_on = v != 0.0,
                param::SOLO_MONO => self.mono_on = v != 0.0,
                _ => return,
            }
            for m in &mut self.mons {
                m.dirty = true;
            }
            return;
        }
        let Some(m) = self.mons.get_mut((b / 4) as usize) else {
            return;
        };
        let mp = &mut m.p;
        match b % 4 {
            block::OWN => {
                match p {
                    param::LVL => mp.lvl = v,
                    param::INV => mp.inv = v != 0.0,
                    param::PAN => mp.pan = v,
                    param::WID => mp.wid = v,
                    param::LIM => mp.lim = v,
                    param::DIM => mp.dim = v,
                    param::PFLDIM => mp.pfldim = v,
                    param::SRCLVL => mp.srclvl = v,
                    param::SRCMIX => mp.srcmix = v,
                    param::SRC => mp.src = v.max(0.0) as u32,
                    _ => return,
                }
                m.dirty = true;
            }
            block::EQ => {
                if p == param::EQ_ON {
                    mp.eq_on = v != 0.0;
                } else if (param::EQ_BAND..param::EQ_BAND + 24).contains(&p) {
                    let k = (p - param::EQ_BAND) as usize;
                    mp.eq[k / 3][k % 3] = v;
                } else {
                    return;
                }
                m.eq_dirty = true;
            }
            block::DLY => {
                match p {
                    param::DLY_ON => mp.dly_on = v != 0.0,
                    param::DLY_M => mp.dly_m = v,
                    _ => return,
                }
                m.dirty = true;
            }
            _ => {}
        }
    }
}

impl Processor for MonitorNode {
    fn process(&mut self, _ctx: &ProcessCtx, io: &mut NodeIo<'_>) {
        if self.read_shared() {
            self.update(true);
        }
        let ins = io.inputs();
        let b = io.block_size();
        let out = io.outputs();
        let (mons, solos) = out.split_at_mut(4 * b);
        let (o1, o2) = mons.split_at_mut(2 * b);
        let (s1, s2) = solos.split_at_mut(2 * b);
        let (sl, sr) = (&mut self.sl[..b], &mut self.sr[..b]);
        for ((m, o), so) in self.mons.iter_mut().zip([o1, o2]).zip([s1, s2]) {
            // solo bus (also an output, for the solo meter)
            let (bl, br) = so.split_at_mut(b);
            bl.fill(0.0);
            br.fill(0.0);
            for (i, r) in m.solo.iter_mut().enumerate() {
                for (t, ramp) in r.iter_mut().enumerate() {
                    if ramp.is_silent() {
                        continue;
                    }
                    let (a0, a1) = ramp.step();
                    let pi = port_of(i, if t == 0 { Sig::Pfl } else { Sig::Afl });
                    let s = [a0[0], 0.0, 0.0, a0[0]];
                    let e = [a1[0], 0.0, 0.0, a1[0]];
                    law::matrix_add(bl, br, ins.get(pi), ins.get(pi + 1), s, e);
                }
            }
            sl.copy_from_slice(bl);
            sr.copy_from_slice(br);
            // source
            let (g0, g1) = m.src.step();
            if let Some(i) = source_strip(m.p.src) {
                if g0[0] != 0.0 || g1[0] != 0.0 {
                    let pi = port_of(i, Sig::Post);
                    let s = [g0[0], 0.0, 0.0, g0[0]];
                    let e = [g1[0], 0.0, 0.0, g1[0]];
                    law::matrix_add(sl, sr, ins.get(pi), ins.get(pi + 1), s, e);
                }
            }
            // EQ
            for (ch, buf) in [&mut *sl, &mut *sr].into_iter().enumerate() {
                for bq in m.eq[ch].iter_mut() {
                    bq.process_block(buf);
                }
            }
            // output matrix
            let (ol, or) = o.split_at_mut(b);
            let (m0, m1) = m.out.step();
            if m0 == ZERO && m1 == ZERO {
                ol.fill(0.0);
                or.fill(0.0);
            } else {
                law::matrix_write(ol, or, sl, sr, m0, m1);
            }
            // delay
            if m.dly_samples > 0 {
                let d = m.dly_samples;
                for (dl, buf) in m.dly.iter_mut().zip([&mut *ol, &mut *or]) {
                    for x in buf.iter_mut() {
                        dl.push(*x);
                        *x = dl.read_int(d);
                    }
                }
            } else {
                for (dl, buf) in m.dly.iter_mut().zip([&*ol, &*or]) {
                    for &x in buf.iter() {
                        dl.push(x);
                    }
                }
            }
            // limiter
            let thr = law::fader_gain(m.p.lim.min(0.0));
            let mut env = m.lim_env;
            for (l, r) in ol.iter_mut().zip(or.iter_mut()) {
                let pk = l.abs().max(r.abs());
                env = if pk > env {
                    pk
                } else {
                    pk + (env - pk) * self.lim_rel
                };
                if env > thr {
                    let g = thr / env;
                    *l *= g;
                    *r *= g;
                }
            }
            m.lim_env = if env < 1e-20 { 0.0 } else { env };
        }
    }

    fn set_param(&mut self, msg: &ParamMsg) {
        self.apply(msg);
    }
}
