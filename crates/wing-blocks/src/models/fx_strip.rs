//! Channel-strip FX (T05C, models.md §6): `*EVEN*` (EVEN CHANNEL), `*SOUL*` (SOUL CHANNEL), `*VINTAGE*`
//! (VINTAGE CHANNEL), `*BUS*` (BUS CHANNEL) and `*MASTER*` (MASTERING), plus `EXT` (EXTERNAL, external
//! send / return loop).
//!
//! # Composition
//!
//! A strip is a fixed chain of the exact blocks WING's own models are made of, built with
//! [`create_model`] (so a strip is always bit-identical to the same blocks run one after the other with the
//! same parameters; the tests check this), stereo-linked, each stage behind its own on switch:
//!
//! | model | chain (stage: block, switch) |
//! |---|---|
//! | `*EVEN*` | `E88` gate (`g_on`) -> `E88` EQ (`eq_on`, `mix` = EQMIX) -> `ECL33` comp/limiter (always in; `d_lon` / `d_con` switch its limiter / compressor) |
//! | `*SOUL*` | `9000G` gate (`g_on`) -> `SOUL` EQ (`eq_on`, `mix`) -> `9000C` comp (`d_on`) |
//! | `*VINTAGE*` | `76LA` (`d_on`) -> `PULSAR` EQ (always in; `eq1` / `eq5` switch its two Pultec sections) -> `LA` LA-2A (`l_on`) |
//! | `*BUS*` | `WARM` VHD warmth (`w_on`) -> `E84` EQ (`eq_on`, `mix`) -> `SBUS` G-bus comp (`d_on`; `d_gain` = make-up after the comp, which the block itself lacks) |
//! | `*MASTER*` | `TAPE` (`t_on`) -> `MACH4` Maag EQ4 (`eq_on`) -> `ENHANCE` Ultra Enhancer (`e_on`) -> `LIMITER` (`l_on`) |
//!
//! The order is the one models.md gives for each strip ("gate + EQ + comp", "1176 + Pultec + LA-2A", "warmth,
//! EQ and bus comp", "tape, Maag EQ4, Ultra Enhancer and limiter"). Every FX parameter is forwarded unchanged
//! to the sub-block parameter it names (tables [`EVEN_ROUTES`] ... [`MASTER_ROUTES`], slot = schema position
//! of the strip model), the `*_on` switches and `mix` (EQ band-gain scale, see [`Model::set_gain_scale`]) are
//! handled here. Schema defaults of the strip (not of the blocks) are loaded by [`Model::load_defaults`].
//!
//! *MASTER* slot 7 (`40`, the Maag 40 Hz band) is missing from the exported schema (its name collides with
//! the generic `fx/N/40` slot); it is still routed to the `MACH4` 40 Hz band. The `MASTER` `TAPE` out gain,
//! `ENHANCE` `g` / `solo` and `MACH4` auto gain have no strip parameter and keep the block defaults.
//! `TAPE`, `ENHANCE` (T05E) and `LIMITER` (T05Q) are composed through the catalog, so *MASTER* picks the
//! real blocks up as those tasks land (until then they are the catalog's pass-through stubs).
//!
//! # Switches, latency
//!
//! A stage switch crossfades over [`XFADE_MS`] (dry path delayed by the stage's own latency); an enabled,
//! settled stage runs bare (bit-exact with the block). A stage that is switched on starts from a cleared
//! state. The strip's latency is the sum of its stages' latencies and is constant: a bypassed stage delays
//! the dry path by its own latency.
//!
//! # EXT
//!
//! [`ExternalLoop`] (`EXT`) is the I/O model of the FX slot: the slot input leaves on the FX SEND legs
//! (wing-engine `FxRackNode`) and the return of the external device arrives as the model's side-chain
//! `key` (wing-engine `InputRouter` resolves `egrp` / `ein` / `emode`, one block late like every re-input
//! path, and passes it to the slot). The model output is `trim` times the return. `egrp` OFF = no loop
//! configured: the slot passes its input through [I]; a configured loop without return audio is silent.
//! `emode`: `M` = both sides carry the return's first leg, `ST` = the stereo pair, `M/S` = pair decoded as
//! `L = M + S`, `R = M - S` (the engine router already resolves the legs for `M` and `ST`; the decode is
//! done here so it applies to whatever the host delivers). `lat` (device round-trip in samples, default 45)
//! is stored and readable ([`ExternalLoop::latency_param`]) but not reported as processing latency: the
//! engine aligns every strip of a family to the largest slot latency in use (T05U), so reporting a default of
//! 45 would delay every strip of that family whenever an EXT slot is inserted.

use crate::eq::EqVariant;
use crate::model::{create_model, BlockKind, Model, ModelTelemetry, ParamValue};
use wing_dsp::delay::DelayLine;

/// Stage on/off crossfade time.
pub const XFADE_MS: f64 = 5.0;

/// Where a strip parameter goes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Route {
    /// Parameter `slot` of stage `stage`.
    Sub(u8, u16),
    /// On switch of stage `stage`.
    On(u8),
    /// EQ `mix` (band-gain scale, %) of stage `stage`.
    Scale(u8),
    /// Output make-up gain (dB) of stage `stage`.
    Post(u8),
}
use Route::{On, Post, Scale, Sub};

/// Which blocks make up a strip stage.
#[derive(Clone, Copy, Debug)]
struct StageSpec {
    kind: BlockKind,
    id: &'static str,
    /// Dynamics stage (contributes to the strip's gain reduction).
    dynamic: bool,
    /// Switchable by an `On` route (otherwise always in).
    switched: bool,
}

const fn st(kind: BlockKind, id: &'static str, dynamic: bool, switched: bool) -> StageSpec {
    StageSpec {
        kind,
        id,
        dynamic,
        switched,
    }
}

const EQ: BlockKind = BlockKind::Eq(EqVariant::Channel);
const EQ_BUS: BlockKind = BlockKind::Eq(EqVariant::Bus);

/// `*EVEN*`: slot -> sub-block parameter (index = slot - 1).
pub const EVEN_ROUTES: [Route; 33] = [
    Sub(0, 1),
    Sub(0, 2),
    Sub(0, 3),
    Sub(0, 4),
    Sub(0, 5),
    Sub(0, 6),
    On(0), // g_on
    On(1), // eq_on
    Sub(1, 1),
    Sub(1, 2),
    Sub(1, 3),
    Sub(1, 4),
    Sub(1, 5),
    Sub(1, 6),
    Sub(1, 7),
    Sub(1, 8),
    Sub(1, 9),
    Sub(1, 10),
    Sub(1, 11),
    Sub(1, 12),
    Sub(1, 13),
    Sub(1, 14),
    Scale(1), // mix
    Sub(2, 1),
    Sub(2, 2),
    Sub(2, 3),
    Sub(2, 4),
    Sub(2, 5),
    Sub(2, 6),
    Sub(2, 7),
    Sub(2, 8),
    Sub(2, 9),
    Sub(2, 10),
];
const EVEN_STAGES: [StageSpec; 3] = [
    st(BlockKind::Dynamics, "E88", true, true),
    st(EQ, "E88", false, true),
    st(BlockKind::Dynamics, "ECL33", true, false),
];
const EVEN_DEFAULTS: [f32; 33] = [
    -35.0, 6.0, 60.0, 355.0, 0.0, 0.0, 0.0, 0.0, // gate, g_on, eq_on
    2.0, 0.0, 0.0, 1.0, 3.0, 0.0, 5.0, 5.0, 0.0, 5.0, 7.0, 0.0, 0.0, 1.0,   // EQ
    100.0, // mix
    0.0, -5.0, 4.0, 0.0, 0.0, -15.0, 3.0, 4.0, 0.0, 0.0, // ECL33
];

/// `*SOUL*`.
pub const SOUL_ROUTES: [Route; 27] = [
    Sub(0, 1),
    Sub(0, 2),
    Sub(0, 3),
    Sub(0, 4),
    Sub(0, 5),
    Sub(0, 6),
    On(0), // g_on
    On(1), // eq_on
    Sub(1, 1),
    Sub(1, 2),
    Sub(1, 3),
    Sub(1, 4),
    Sub(1, 5),
    Sub(1, 6),
    Sub(1, 7),
    Sub(1, 8),
    Sub(1, 9),
    Sub(1, 10),
    Sub(1, 11),
    Sub(1, 12),
    Scale(1), // mix
    On(2),    // d_on
    Sub(2, 1),
    Sub(2, 2),
    Sub(2, 3),
    Sub(2, 4),
    Sub(2, 5),
];
const SOUL_STAGES: [StageSpec; 3] = [
    st(BlockKind::Dynamics, "9000G", true, true),
    st(EQ, "SOUL", false, true),
    st(BlockKind::Dynamics, "9000C", true, true),
];
const SOUL_DEFAULTS: [f32; 27] = [
    -35.0, 40.0, 51.0, 806.0, 0.0, 0.0, 0.0, 0.0, // gate, g_on, eq_on
    2.0, 0.0, 6.0, 1.0, 5.0, 0.0, 3.0, 0.0, 5.0, 0.0, 8.0, 0.0,   // EQ
    100.0, // mix
    0.0, -10.0, 2.8, 0.0, 395.0, 0.0, // d_on, 9000C
];

/// `*VINTAGE*`.
pub const VINTAGE_ROUTES: [Route; 26] = [
    Sub(0, 1),
    Sub(0, 2),
    Sub(0, 3),
    Sub(0, 4),
    Sub(0, 5),
    On(0), // d_on
    Sub(1, 1),
    Sub(1, 2),
    Sub(1, 3),
    Sub(1, 4),
    Sub(1, 5),
    Sub(1, 6),
    Sub(1, 7),
    Sub(1, 8),
    Sub(1, 9),
    Sub(1, 10),
    Sub(1, 11),
    Sub(1, 12),
    Sub(1, 13),
    Sub(1, 14),
    Sub(1, 15),
    Sub(1, 16),
    Sub(2, 1),
    Sub(2, 2),
    Sub(2, 3),
    On(2), // l_on
];
const VINTAGE_STAGES: [StageSpec; 3] = [
    st(BlockKind::Dynamics, "76LA", true, true),
    st(EQ, "PULSAR", false, false),
    st(BlockKind::Dynamics, "LA", true, true),
];
const VINTAGE_DEFAULTS: [f32; 26] = [
    -30.0, -27.0, 2.0, 2.0, 1.0, 0.0, // 76LA, d_on
    0.0, 0.0, 0.0, 1.0, 5.0, 0.0, 4.0, 0.0, 2.0, // eq1 section
    0.0, 0.0, 1.0, 0.0, 4.0, 0.0, 2.0, // eq5 section
    40.0, 20.0, 0.0, 0.0, // LA, l_on
];

/// `*BUS*`.
pub const BUS_ROUTES: [Route; 22] = [
    Sub(0, 1),
    Sub(0, 2),
    Sub(0, 3),
    Sub(0, 4),
    Sub(0, 5),
    On(0), // w_on
    On(1), // eq_on
    Sub(1, 1),
    Sub(1, 2),
    Sub(1, 3),
    Sub(1, 4),
    Sub(1, 5),
    Sub(1, 6),
    Sub(1, 7),
    Sub(1, 8),
    Scale(1), // mix
    Sub(2, 1),
    Sub(2, 2),
    Sub(2, 3),
    Sub(2, 4),
    Post(2), // d_gain
    On(2),   // d_on
];
const BUS_STAGES: [StageSpec; 3] = [
    st(BlockKind::Dynamics, "WARM", false, true),
    st(EQ_BUS, "E84", false, true),
    st(BlockKind::Dynamics, "SBUS", true, true),
];
const BUS_DEFAULTS: [f32; 22] = [
    33.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, // warmth, w_on, eq_on
    0.0, 1.0, 0.0, 3.0, 0.0, 0.0, 2.0, 0.0,   // EQ
    100.0, // mix
    -12.0, 3.0, 10.0, 5.0, 0.0, 0.0, // SBUS, d_gain, d_on
];

/// `*MASTER*`.
pub const MASTER_ROUTES: [Route; 33] = [
    Sub(0, 1),
    Sub(0, 2),
    Sub(0, 3),
    Sub(0, 4),
    On(0),      // t_on
    Sub(1, 1),  // sub
    Sub(1, 2),  // 40 (absent from the schema export)
    Sub(1, 3),  // 160
    Sub(1, 4),  // 650
    Sub(1, 5),  // 2k5
    Sub(1, 6),  // air
    Sub(1, 7),  // airm
    On(1),      // eq_on
    Sub(2, 1),  // e_stlvl
    Sub(2, 2),  // e_lmf
    Sub(2, 3),  // e_mlvl
    Sub(2, 4),  // e_st
    Sub(2, 5),  // e_m
    Sub(2, 6),  // e_bass
    Sub(2, 7),  // e_mid
    Sub(2, 8),  // e_high
    Sub(2, 11), // e_bassf
    Sub(2, 12), // e_midq
    Sub(2, 13), // e_highf
    On(2),      // e_on
    Sub(3, 1),
    Sub(3, 2),
    Sub(3, 3),
    Sub(3, 4),
    Sub(3, 5),
    Sub(3, 6),
    Sub(3, 7),
    On(3), // l_on
];
const MASTER_STAGES: [StageSpec; 4] = [
    st(BlockKind::Fx, "TAPE", false, true),
    st(EQ, "MACH4", false, true),
    st(BlockKind::Fx, "ENHANCE", false, true),
    st(BlockKind::Fx, "LIMITER", true, true),
];
const MASTER_DEFAULTS: [f32; 33] = [
    0.0, 15.0, 0.0, 0.0, 0.0, // tape, t_on
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, // EQ, eq_on
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 15.0, 15.0, 25.0, 0.0, // enhancer, e_on
    0.0, -0.5, 0.0, 3.0, 1.0, 0.05, 502.0, 1.0, // limiter, l_on
];

/// The five channel-strip models.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StripKind {
    Even,
    Soul,
    Vintage,
    Bus,
    Master,
}

impl StripKind {
    pub const ALL: [StripKind; 5] = [
        StripKind::Even,
        StripKind::Soul,
        StripKind::Vintage,
        StripKind::Bus,
        StripKind::Master,
    ];

    /// Snapshot `mdl` token.
    pub fn id(self) -> &'static str {
        match self {
            StripKind::Even => "*EVEN*",
            StripKind::Soul => "*SOUL*",
            StripKind::Vintage => "*VINTAGE*",
            StripKind::Bus => "*BUS*",
            StripKind::Master => "*MASTER*",
        }
    }

    /// Slot -> sub-block parameter table (index = slot - 1).
    pub fn routes(self) -> &'static [Route] {
        match self {
            StripKind::Even => &EVEN_ROUTES,
            StripKind::Soul => &SOUL_ROUTES,
            StripKind::Vintage => &VINTAGE_ROUTES,
            StripKind::Bus => &BUS_ROUTES,
            StripKind::Master => &MASTER_ROUTES,
        }
    }

    fn stages(self) -> &'static [StageSpec] {
        match self {
            StripKind::Even => &EVEN_STAGES,
            StripKind::Soul => &SOUL_STAGES,
            StripKind::Vintage => &VINTAGE_STAGES,
            StripKind::Bus => &BUS_STAGES,
            StripKind::Master => &MASTER_STAGES,
        }
    }

    /// Schema defaults (index = slot - 1; enums as indices, ratio-like float enums as values).
    pub fn defaults(self) -> &'static [f32] {
        match self {
            StripKind::Even => &EVEN_DEFAULTS,
            StripKind::Soul => &SOUL_DEFAULTS,
            StripKind::Vintage => &VINTAGE_DEFAULTS,
            StripKind::Bus => &BUS_DEFAULTS,
            StripKind::Master => &MASTER_DEFAULTS,
        }
    }

    /// The `(block kind, model id)` of every stage in processing order.
    pub fn chain(self) -> impl Iterator<Item = (BlockKind, &'static str)> {
        self.stages().iter().map(|s| (s.kind, s.id))
    }
}

struct Stage {
    model: Box<dyn Model>,
    dynamic: bool,
    on: bool,
    lat: usize,
    /// Crossfade position 0 (bypassed) .. 1 (in).
    wet: f32,
    /// Make-up gain (linear): current and target.
    gain: f32,
    gain_target: f32,
    dly: Option<Box<[DelayLine; 2]>>,
}

impl Stage {
    fn new(spec: &StageSpec) -> Self {
        Self {
            model: create_model(spec.kind, spec.id),
            dynamic: spec.dynamic,
            on: !spec.switched,
            lat: 0,
            wet: if spec.switched { 0.0 } else { 1.0 },
            gain: 1.0,
            gain_target: 1.0,
            dly: None,
        }
    }

    fn settle(&mut self) {
        self.wet = self.on as u8 as f32;
        self.gain = self.gain_target;
    }
}

/// A channel-strip FX model (`*EVEN*`, `*SOUL*`, `*VINTAGE*`, `*BUS*`, `*MASTER*`); see the module doc.
pub struct ChannelStrip {
    kind: StripKind,
    fs: f64,
    stages: Vec<Stage>,
    vals: Vec<f32>,
    dry_l: Vec<f32>,
    dry_r: Vec<f32>,
    max_block: usize,
    xfade_step: f32,
}

impl ChannelStrip {
    pub fn new(kind: StripKind) -> Self {
        let stages = kind.stages().iter().map(Stage::new).collect();
        let mut s = Self {
            kind,
            fs: 48_000.0,
            stages,
            vals: kind.defaults().to_vec(),
            dry_l: Vec::new(),
            dry_r: Vec::new(),
            max_block: 0,
            xfade_step: 1.0,
        };
        s.apply_defaults();
        s
    }

    pub fn kind(&self) -> StripKind {
        self.kind
    }

    /// Current state of the stage switch `stage` (0-based, chain order).
    pub fn stage_on(&self, stage: usize) -> bool {
        self.stages.get(stage).is_some_and(|s| s.on)
    }

    fn apply_defaults(&mut self) {
        for s in &mut self.stages {
            s.model.load_defaults();
        }
        self.vals.copy_from_slice(self.kind.defaults());
        for slot in 1..=self.vals.len() as u16 {
            let v = self.vals[slot as usize - 1];
            self.route(slot, ParamValue::Float(v));
        }
        for s in &mut self.stages {
            s.model.commit();
            s.settle();
        }
    }

    fn route(&mut self, slot: u16, value: ParamValue) {
        let Some(r) = (slot as usize)
            .checked_sub(1)
            .and_then(|i| self.kind.routes().get(i))
        else {
            return;
        };
        match *r {
            Sub(stage, sub) => self.stages[stage as usize].model.set_param(sub, value),
            On(stage) => {
                let s = &mut self.stages[stage as usize];
                let on = value.as_bool();
                if on && !s.on {
                    s.model.reset(); // switched in from a cleared state
                }
                s.on = on;
            }
            Scale(stage) => self.stages[stage as usize]
                .model
                .set_gain_scale(value.as_f32()),
            Post(stage) => {
                self.stages[stage as usize].gain_target = 10f32.powf(value.as_f32() * 0.05);
            }
        }
    }

    fn process_chunk(&mut self, l: &mut [f32], r: &mut [f32]) {
        let n = l.len();
        let step = self.xfade_step;
        for s in &mut self.stages {
            let target = s.on as u8 as f32;
            let unity = s.gain == 1.0 && s.gain_target == 1.0;
            if s.wet == target && !s.on {
                // bypassed: dry path at the stage's latency
                if let Some(d) = s.dly.as_mut() {
                    for i in 0..n {
                        d[0].push(l[i]);
                        d[1].push(r[i]);
                        l[i] = d[0].read_int(s.lat);
                        r[i] = d[1].read_int(s.lat);
                    }
                }
                continue;
            }
            if s.wet == 1.0 && s.on {
                // settled: the bare block (plus a make-up gain ramp if it is moving)
                if let Some(d) = s.dly.as_mut() {
                    for i in 0..n {
                        d[0].push(l[i]);
                        d[1].push(r[i]);
                    }
                }
                s.model.process(l, r, None);
                if !unity {
                    let g0 = s.gain;
                    let gs = (s.gain_target - g0) / n as f32;
                    for i in 0..n {
                        let g = g0 + gs * (i + 1) as f32;
                        l[i] *= g;
                        r[i] *= g;
                    }
                    s.gain = s.gain_target;
                }
                continue;
            }
            // crossfading and / or make-up gain ramp
            match s.dly.as_mut() {
                Some(d) => {
                    for i in 0..n {
                        d[0].push(l[i]);
                        d[1].push(r[i]);
                        self.dry_l[i] = d[0].read_int(s.lat);
                        self.dry_r[i] = d[1].read_int(s.lat);
                    }
                }
                None => {
                    self.dry_l[..n].copy_from_slice(l);
                    self.dry_r[..n].copy_from_slice(r);
                }
            }
            s.model.process(l, r, None);
            let g0 = s.gain;
            let gs = (s.gain_target - g0) / n as f32;
            let mut w = s.wet;
            for i in 0..n {
                if w != target {
                    w = if target > w {
                        (w + step).min(target)
                    } else {
                        (w - step).max(target)
                    };
                }
                let g = g0 + gs * (i + 1) as f32;
                let (dl, dr) = (self.dry_l[i], self.dry_r[i]);
                l[i] = dl + w * (g * l[i] - dl);
                r[i] = dr + w * (g * r[i] - dr);
            }
            s.wet = w;
            s.gain = s.gain_target;
        }
    }
}

impl Model for ChannelStrip {
    fn id(&self) -> &'static str {
        self.kind.id()
    }

    fn prepare(&mut self, sample_rate: f64, max_block: usize) {
        self.fs = sample_rate;
        self.max_block = max_block.max(1);
        self.dry_l = vec![0.0; self.max_block];
        self.dry_r = vec![0.0; self.max_block];
        self.xfade_step = (1.0 / (XFADE_MS * 1e-3 * sample_rate)).min(1.0) as f32;
        for s in &mut self.stages {
            s.model.prepare(sample_rate, max_block);
            s.lat = s.model.latency();
            s.dly = (s.lat > 0).then(|| Box::new([DelayLine::new(s.lat), DelayLine::new(s.lat)]));
        }
        self.reset();
    }

    fn process(&mut self, left: &mut [f32], right: &mut [f32], _sc: Option<(&[f32], &[f32])>) {
        let n = left.len().min(right.len());
        let mut i = 0;
        while i < n {
            let e = (i + self.max_block).min(n);
            self.process_chunk(&mut left[i..e], &mut right[i..e]);
            i = e;
        }
    }

    fn latency(&self) -> usize {
        self.stages.iter().map(|s| s.model.latency()).sum()
    }

    fn reset(&mut self) {
        for s in &mut self.stages {
            s.model.commit();
            s.model.reset();
            if let Some(d) = s.dly.as_mut() {
                d[0].reset();
                d[1].reset();
            }
            s.settle();
        }
    }

    fn set_param(&mut self, slot: u16, value: ParamValue) {
        if let Some(v) = (slot as usize)
            .checked_sub(1)
            .and_then(|i| self.vals.get_mut(i))
        {
            *v = value.as_f32();
            self.route(slot, value);
        }
    }

    fn param(&self, slot: u16) -> Option<f32> {
        (slot as usize)
            .checked_sub(1)
            .and_then(|i| self.vals.get(i))
            .copied()
    }

    fn load_defaults(&mut self) {
        self.apply_defaults();
    }

    fn commit(&mut self) {
        for s in &mut self.stages {
            s.model.commit();
        }
    }

    fn telemetry(&self) -> ModelTelemetry {
        let mut t = ModelTelemetry::default();
        for s in self.stages.iter().filter(|s| s.dynamic && s.wet > 0.0) {
            let m = s.model.telemetry();
            t.gain_reduction_db += m.gain_reduction_db;
            t.engaged |= m.engaged;
        }
        t
    }
}

// ---------------------------------------------------------------------------------------------------------
// EXT
// ---------------------------------------------------------------------------------------------------------

/// EXT schema slots.
pub mod ext_slot {
    pub const EGRP: u16 = 1;
    pub const EIN: u16 = 2;
    pub const EMODE: u16 = 3;
    pub const LAT: u16 = 4;
    pub const TRIM: u16 = 5;
}

/// `egrp` index of OFF.
pub const EXT_GRP_OFF: usize = 0;
/// `emode` indices.
pub const EXT_MODE_M: usize = 0;
pub const EXT_MODE_ST: usize = 1;
pub const EXT_MODE_MS: usize = 2;

/// EXTERNAL: send / return loop model (see the module doc).
pub struct ExternalLoop {
    egrp: usize,
    ein: i32,
    emode: usize,
    lat: i32,
    trim_db: f32,
    gain: f32,
    gain_target: f32,
}

impl Default for ExternalLoop {
    fn default() -> Self {
        Self::new()
    }
}

impl ExternalLoop {
    pub fn new() -> Self {
        Self {
            egrp: EXT_GRP_OFF,
            ein: 1,
            emode: EXT_MODE_ST,
            lat: 45,
            trim_db: 0.0,
            gain: 1.0,
            gain_target: 1.0,
        }
    }

    /// Return source group index (`egrp` token index, 0 = OFF).
    pub fn group(&self) -> usize {
        self.egrp
    }

    /// Return source index within the group (1..64).
    pub fn input(&self) -> i32 {
        self.ein
    }

    /// `lat` parameter (samples).
    pub fn latency_param(&self) -> i32 {
        self.lat
    }
}

impl Model for ExternalLoop {
    fn id(&self) -> &'static str {
        "EXT"
    }

    fn prepare(&mut self, _sample_rate: f64, _max_block: usize) {
        self.reset();
    }

    fn process(&mut self, left: &mut [f32], right: &mut [f32], sc: Option<(&[f32], &[f32])>) {
        if self.egrp == EXT_GRP_OFF {
            return; // no loop configured: pass through
        }
        let n = left.len().min(right.len());
        let g0 = self.gain;
        let step = (self.gain_target - g0) / n.max(1) as f32;
        let pick = |s: &[f32], i: usize| s.get(i).copied().unwrap_or(0.0);
        for i in 0..n {
            let g = g0 + step * (i + 1) as f32;
            let (a, b) = sc.map_or((0.0, 0.0), |(a, b)| (pick(a, i), pick(b, i)));
            let (a, b) = match self.emode {
                EXT_MODE_M => (a, a),
                EXT_MODE_MS => (a + b, a - b),
                _ => (a, b),
            };
            left[i] = a * g;
            right[i] = b * g;
        }
        self.gain = self.gain_target;
    }

    fn reset(&mut self) {
        self.gain = self.gain_target;
    }

    fn set_param(&mut self, slot: u16, v: ParamValue) {
        match slot {
            ext_slot::EGRP => self.egrp = v.as_index().min(11),
            ext_slot::EIN => self.ein = v.as_i32().clamp(1, 64),
            ext_slot::EMODE => self.emode = v.as_index().min(2),
            ext_slot::LAT => self.lat = v.as_i32().clamp(0, 800),
            ext_slot::TRIM => {
                self.trim_db = v.as_f32().clamp(-18.0, 18.0);
                self.gain_target = 10f32.powf(self.trim_db * 0.05);
            }
            _ => {}
        }
    }

    fn param(&self, slot: u16) -> Option<f32> {
        Some(match slot {
            ext_slot::EGRP => self.egrp as f32,
            ext_slot::EIN => self.ein as f32,
            ext_slot::EMODE => self.emode as f32,
            ext_slot::LAT => self.lat as f32,
            ext_slot::TRIM => self.trim_db,
            _ => return None,
        })
    }

    fn load_defaults(&mut self) {
        *self = Self::new();
    }
}
