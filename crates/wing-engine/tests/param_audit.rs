//! T081: parameter-binding audit of every model in every model-dependent processing block (filter tool, gate,
//! dynamics, EQ, FX), driven exactly as WING-Edit drives the console: the model's slot path
//! (`ch/1/dyn/1`, the hash WING-Edit sends) is set in the [`ParamStore`], the T03A pipeline converts and
//! dispatches it, the real host (channel/aux strip, bus/main/matrix strip, FX slot) forwards it to its
//! [`ModelSlot`] and the model's `set_param(slot)`.
//!
//! Per `(group, model, parameter)`:
//! * **binding**: the slot path is bound by the host's layout to a model target (else `UNBOUND`);
//! * **readback**: after each set, the audible model reports (`Model::param`) the engineering value the
//!   pipeline produced (else `READBACK`: a range/scale/enum mismatch between schema and model table);
//! * **audibility**: the parameter is swept across its range (min .. max, small enums through every value) on
//!   a deterministic stereo programme (multi-tone bursts at -1/-6/-20/-40 dBFS, a gliding monophonic "voice"
//!   burst for pitch detectors, a noise floor, different L/R) and consecutive sweep outputs must differ
//!   (> [`LIVE_DB`]). A parameter without effect at the model defaults is retried, at its range ends, in up
//!   to `Opts::contexts` settings of the other parameters ([`contexts`]: switches flipped, one gain /
//!   threshold / mode at an extreme, seeded random, selector + amount pairs), since a band frequency needs a
//!   band gain, a section parameter its section switch, an attack time `auto` off. Still no effect -> `Dead`,
//!   unless listed in [`JUSTIFIED`] with the reason;
//! * **repeatability / finiteness**: two renders at the same setting are bit-identical (`reset` clears every
//!   state, else `NonDet`) and never contain NaN / inf (`NonFinite`).
//!
//! T085 adds, for the 32 models the gate and dynamics slots share: **slot parity** (each model hosted in
//! `ch/1/gate` and in `ch/1/dyn`, every parameter written explicitly, defaults and range ends: bit-identical
//! output) and the **block wrapper** of both slots (`gain` live, `mix` live, except for a model that owns its
//! mix, WARM, where the block `mix` must have no effect), plus the user's exact ch 21 gate-slot WARM case.
//!
//! `make check` runs the sampled audit: binding + readback of every parameter of every group (no audio), and
//! audio for every model of the channel groups and FX slot 1 on a rotating subset of parameters (range ends,
//! short signal). `make check-full` adds the full sweep of every parameter of every group (`full_*`,
//! `#[ignore]`d in dev). `WING_AUDIT_REPORT=1 cargo test -p wing-engine --release --test param_audit --
//! --include-ignored --nocapture` prints the per-group tables (counts, every non-OK finding, the context that
//! made each context-dependent parameter live).

use std::collections::BTreeMap;

use wing_blocks::model::{create_model, BlockKind};
use wing_engine::channel::{bind_aux_strip, bind_channel_strip, ChannelStrip};
use wing_engine::fx::{bind_fx_params, FxSlot};
use wing_engine::graph::{NodeIo, ProcessCtx};
use wing_engine::mix::strip::{bind_mix_strip, MixStrip};
use wing_engine::params::std_models::block_kind;
use wing_engine::params::strip::ModelSlot;
use wing_engine::params::{
    ParamMapBuilder, ParamMsg, ParamPipeline, PipelineConfig, ProcessorAccess, TARGET_BASE,
};
use wing_engine::routing::Strip;
use wing_engine::{NodeId, Processor};
use wing_model::{Desc, NodeId as ParamId, ParamStore, ParamType, Schema};

const FS: f64 = 48_000.0;
const BLOCK: usize = 64;
/// A sweep is live when two of its outputs differ by more than this (dB, RMS of the difference relative to
/// the louder output). Identical processing is bit-exact, so anything above float noise is a real effect.
const LIVE_DB: f64 = -100.0;

/// Parameters that legitimately have no audible effect under a stereo programme signal, with the reason.
/// Matched as `(model, parameter name)`; `*` matches any model.
const JUSTIFIED: &[(&str, &str, &str)] = &[
    (
        "DEQ2",
        "$page",
        "UI page selector (`$` = live UI state): which band the console shows, no DSP meaning",
    ),
    (
        "EXT",
        "ein",
        "routing: the engine's input router resolves `egrp`/`ein` to the return it hands the model as key \
         (tests/fx_ext.rs); the model itself only sees the return",
    ),
    (
        "EXT",
        "lat",
        "external device round trip: stored and readable, deliberately not reported as processing latency \
         (fx_strip.rs module doc)",
    ),
    (
        "ROTARY",
        "dac",
        "rotor acceleration only shapes speed transitions while SPEED switches; a reset starts the rotors at \
         their set speed (fx_mod rotary_rotors_ramp_with_inertia covers it)",
    ),
    (
        "ROTARY",
        "hac",
        "rotor acceleration only shapes speed transitions while SPEED switches; a reset starts the rotors at \
         their set speed (fx_mod rotary_rotors_ramp_with_inertia covers it)",
    ),
    (
        "VSS3",
        "preset",
        "UI only: the console applies the preset library on `load`; no DSP effect (vss3.rs)",
    ),
    ("VSS3", "load", "UI action, no DSP effect (vss3.rs)"),
    ("VSS3", "view", "UI action, no DSP effect (vss3.rs)"),
];

/// `EXT` in both FX model groups.
const EXT_INDEX: u16 = 1;

// ---- hosts ---------------------------------------------------------------------------------------------

enum Host {
    Ch(Box<ChannelStrip>),
    Mix(Box<MixStrip>),
    Fx(Box<FxSlot>, u8),
}

impl Host {
    fn slot(&self, which: Which) -> &ModelSlot {
        match (self, which) {
            (Host::Ch(s), Which::Tool) => s.tool_slot(),
            (Host::Ch(s), Which::Gate) => s.gate_slot(),
            (Host::Ch(s), Which::Eq) => s.eq_slot(),
            (Host::Ch(s), Which::Dyn) => s.dyn_slot(),
            (Host::Mix(s), Which::Eq) => s.eq_slot(),
            (Host::Mix(s), Which::Dyn) => s.dyn_slot(),
            (Host::Fx(s, _), Which::Fx) => s.model_slot(),
            _ => unreachable!(),
        }
    }

    fn settle(&mut self) {
        match self {
            Host::Ch(s) => s.settle(),
            Host::Mix(s) => s.settle(),
            Host::Fx(s, _) => {
                // the owning insert is on for the whole render (no fade-in on the first one)
                s.model_slot_mut()
                    .set_wrapper(wing_engine::fx::param::ON, 1.0);
                s.reset()
            }
        }
    }

    fn process(&mut self, l: &mut [f32], r: &mut [f32]) {
        match self {
            Host::Ch(s) => s.process_stereo(l, r),
            Host::Mix(s) => {
                s.process_pre(l, r);
                s.process_out(l, r);
            }
            Host::Fx(s, _) => {
                // EXT: the external device returns the slot input (an ideal loop), so the model's own
                // parameters (trim) are audible; other models never see a return
                if s.model_slot().model_index() == Some(EXT_INDEX) {
                    let (a, b) = (l.to_vec(), r.to_vec());
                    s.set_return(Some((&a, &b)));
                }
                s.process(l, r)
            }
        }
    }
}

impl Processor for Host {
    fn process(&mut self, _ctx: &ProcessCtx, _io: &mut NodeIo<'_>) {
        unreachable!("driven directly")
    }

    fn set_param(&mut self, msg: &ParamMsg) {
        match self {
            Host::Ch(s) => s.apply(msg),
            Host::Mix(s) => s.apply(msg),
            Host::Fx(s, n) => match *msg {
                ParamMsg::Set { block, .. } | ParamMsg::Model { block, .. } if block != *n - 1 => {}
                _ => s.apply(msg),
            },
        }
    }
}

struct One(Host);

impl ProcessorAccess for One {
    fn processor(&mut self, _id: NodeId) -> &mut dyn Processor {
        &mut self.0
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Which {
    Tool,
    Gate,
    Eq,
    Dyn,
    Fx,
}

/// One audited model group: its wing-model group key, the block path of instance 1, the switches that
/// engage the block, the host and which of its slots runs the group.
struct Group {
    key: &'static str,
    block: &'static str,
    on: &'static [(&'static str, &'static str)],
    which: Which,
}

const GROUPS: &[Group] = &[
    Group {
        key: "ch/*/flt",
        block: "ch/1/flt",
        on: &[("ch/1/flt/tf", "1")],
        which: Which::Tool,
    },
    Group {
        key: "ch/*/gate",
        block: "ch/1/gate",
        on: &[("ch/1/gate/on", "1")],
        which: Which::Gate,
    },
    Group {
        key: "ch/*/eq",
        block: "ch/1/eq",
        on: &[("ch/1/eq/on", "1")],
        which: Which::Eq,
    },
    Group {
        key: "ch/*/dyn",
        block: "ch/1/dyn",
        on: &[("ch/1/dyn/on", "1")],
        which: Which::Dyn,
    },
    Group {
        key: "aux/*/eq",
        block: "aux/1/eq",
        on: &[("aux/1/eq/on", "1")],
        which: Which::Eq,
    },
    Group {
        key: "aux/*/dyn",
        block: "aux/1/dyn",
        on: &[("aux/1/dyn/on", "1")],
        which: Which::Dyn,
    },
    Group {
        key: "bus/*/eq",
        block: "bus/1/eq",
        on: &[("bus/1/eq/on", "1")],
        which: Which::Eq,
    },
    Group {
        key: "bus/*/dyn",
        block: "bus/1/dyn",
        on: &[("bus/1/dyn/on", "1")],
        which: Which::Dyn,
    },
    Group {
        key: "main/*/eq",
        block: "main/1/eq",
        on: &[("main/1/eq/on", "1")],
        which: Which::Eq,
    },
    Group {
        key: "main/*/dyn",
        block: "main/1/dyn",
        on: &[("main/1/dyn/on", "1")],
        which: Which::Dyn,
    },
    Group {
        key: "mtx/*/eq",
        block: "mtx/1/eq",
        on: &[("mtx/1/eq/on", "1")],
        which: Which::Eq,
    },
    Group {
        key: "mtx/*/dyn",
        block: "mtx/1/dyn",
        on: &[("mtx/1/dyn/on", "1")],
        which: Which::Dyn,
    },
    Group {
        key: "fx/1",
        block: "fx/1",
        on: &[],
        which: Which::Fx,
    },
    Group {
        key: "fx/9",
        block: "fx/9",
        on: &[],
        which: Which::Fx,
    },
];

fn group(key: &str) -> &'static Group {
    GROUPS.iter().find(|g| g.key == key).unwrap()
}

/// The host of `g` with its block bound through a [`ParamMapBuilder`] (node 0).
fn build_host(g: &Group) -> (Host, ParamMapBuilder) {
    let mut b = ParamMapBuilder::new();
    let prefix = g.key.split('/').next().unwrap();
    let host = match prefix {
        "ch" => {
            bind_channel_strip(&mut b, 1, NodeId(0)).unwrap();
            Host::Ch(Box::new(ChannelStrip::channel(FS, BLOCK)))
        }
        "aux" => {
            bind_aux_strip(&mut b, 1, NodeId(0)).unwrap();
            Host::Ch(Box::new(ChannelStrip::aux(FS, BLOCK)))
        }
        "bus" | "main" | "mtx" => {
            let s = match prefix {
                "bus" => Strip::Bus(1),
                "main" => Strip::Main(1),
                _ => Strip::Mtx(1),
            };
            bind_mix_strip(&mut b, s, NodeId(0)).unwrap();
            Host::Mix(Box::new(MixStrip::new(s, FS, BLOCK)))
        }
        _ => {
            bind_fx_params(&mut b, NodeId(0)).unwrap();
            let n: u8 = g.block[3..].parse().unwrap();
            let slot = FxSlot::new(n, FS, BLOCK, |_, name| {
                let mut m = create_model(BlockKind::Fx, name);
                m.prepare(FS, BLOCK);
                Some(Box::new(m) as Box<dyn wing_engine::params::strip::RtModel>)
            });
            Host::Fx(Box::new(slot), n)
        }
    };
    (host, b)
}

// ---- signal --------------------------------------------------------------------------------------------

/// Deterministic stereo programme in 250 ms bursts (fast attack, exponential decay, gap): multi-tone bursts
/// at -1 / -6 / -20 / -40 dBFS peaks, a monophonic "voice" burst (harmonic tone gliding 196 -> 523 Hz, for pitch
/// detectors), a -70 dBFS noise floor; R is a level/phase-shifted variant with its own tone.
fn programme(secs: f64) -> (Vec<f32>, Vec<f32>) {
    let n = (secs * FS) as usize;
    let mut seed = 0x1234_5678u32;
    let mut noise = move || {
        seed ^= seed << 13;
        seed ^= seed >> 17;
        seed ^= seed << 5;
        seed as f32 / u32::MAX as f32 * 2.0 - 1.0
    };
    let freqs = [62.0, 180.0, 440.0, 1250.0, 3700.0, 9000.0, 15000.0];
    let burst = (0.25 * FS) as usize;
    // (peak, voice)
    let bursts = [
        (0.9f32, false),
        (0.3, true),
        (0.1, false),
        (0.01, false),
        (0.5, false),
        (0.02, false),
    ];
    let (mut l, mut r) = (vec![0.0f32; n], vec![0.0f32; n]);
    let mut vph = 0.0f64;
    for i in 0..n {
        let k = i / burst;
        let ph = i % burst;
        let env = if ph < (0.18 * FS) as usize {
            let a = (ph as f32 / 48.0).min(1.0);
            a * (-(ph as f32) / (0.05 * FS as f32)).exp() * 0.7 + a * 0.3
        } else {
            0.0
        };
        let (pk, voice) = bursts[k % bursts.len()];
        let t = i as f64 / FS;
        let (mut x, mut y) = (0.0f64, 0.0f64);
        if voice {
            let f0 = 196.0 * (523.0f64 / 196.0).powf(ph as f64 / burst as f64);
            vph += f0 / FS;
            for h in 1..=6 {
                let a = 1.0 / h as f64;
                x += a * (2.0 * std::f64::consts::PI * h as f64 * vph).sin();
            }
            y = x;
        } else {
            for (j, f) in freqs.iter().enumerate() {
                let a = 1.0 / (1.0 + j as f64).sqrt();
                x += a * (2.0 * std::f64::consts::PI * f * t).sin();
                y += a * (2.0 * std::f64::consts::PI * f * 1.003 * t + j as f64).sin();
            }
        }
        l[i] = pk * env * (x as f32) * 0.25 + 3e-4 * noise();
        r[i] = 0.8 * pk * env * (y as f32) * 0.25
            + 0.05 * pk * env * (2.0 * std::f32::consts::PI * 2500.0 * t as f32).sin()
            + 3e-4 * noise();
    }
    (l, r)
}

fn rel_db(a: &[f32], b: &[f32]) -> f64 {
    let (mut d, mut ea, mut eb) = (0.0f64, 0.0f64, 0.0f64);
    for (x, y) in a.iter().zip(b) {
        let (x, y) = (*x as f64, *y as f64);
        if !x.is_finite() || !y.is_finite() {
            return f64::INFINITY;
        }
        d += (x - y) * (x - y);
        ea += x * x;
        eb += y * y;
    }
    if d == 0.0 {
        return f64::NEG_INFINITY;
    }
    10.0 * (d / ea.max(eb).max(1e-30)).log10()
}

// ---- audit ---------------------------------------------------------------------------------------------

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
enum Verdict {
    Ok,
    /// Live only in a random context (needs another parameter, e.g. a band gain or a section switch).
    OkCtx,
    Justified,
    NoAudio,
    Stub,
    Dead,
    /// Two renders at the same setting differ (state not cleared by `reset`).
    NonDet,
    /// The output contains NaN / inf.
    NonFinite,
    Readback,
    Unbound,
}

struct Finding {
    model: &'static str,
    slot: &'static str,
    name: &'static str,
    verdict: Verdict,
    note: String,
}

struct Rig {
    store: ParamStore,
    pipe: ParamPipeline,
    host: One,
    which: Which,
    x: (Vec<f32>, Vec<f32>),
}

impl Rig {
    fn new(g: &Group, secs: f64) -> Self {
        let store = ParamStore::new();
        let (host, b) = build_host(g);
        let mut host = One(host);
        let mut pipe = ParamPipeline::new(&store, b.build(), PipelineConfig::default()).unwrap();
        pipe.sync_all(&store, &mut host);
        for (p, v) in g.on {
            store.set_text(store.lookup(p).unwrap(), v, 0).unwrap();
        }
        let mut r = Self {
            store,
            pipe,
            host,
            which: g.which,
            x: programme(secs),
        };
        r.drain();
        r
    }

    fn drain(&mut self) {
        while self.pipe.apply(&mut self.host).changes > 0 {}
    }

    fn slot(&self) -> &ModelSlot {
        self.host.0.slot(self.which)
    }

    fn render(&mut self) -> Vec<f32> {
        self.host.0.settle();
        let (xl, xr) = &self.x;
        let mut out = Vec::with_capacity(2 * xl.len());
        let (mut l, mut r) = (vec![0.0f32; BLOCK], vec![0.0f32; BLOCK]);
        for (a, b) in xl.chunks(BLOCK).zip(xr.chunks(BLOCK)) {
            let n = a.len();
            l[..n].copy_from_slice(a);
            r[..n].copy_from_slice(b);
            self.host.0.process(&mut l[..n], &mut r[..n]);
            out.extend_from_slice(&l[..n]);
            out.extend_from_slice(&r[..n]);
        }
        out
    }
}

/// Engineering value the pipeline delivers for `raw` under `d`.
fn eng(d: &Desc, raw: u32) -> f32 {
    match d.ty {
        ParamType::Enum => raw as f32,
        _ => d.as_f64(raw) as f32,
    }
}

/// Normalised sweep positions of a parameter: all positions of a small discrete parameter, else `points`
/// evenly spread over the range.
fn sweep(d: &Desc, points: usize) -> Vec<f32> {
    let pos = d.positions() as usize;
    let n = if pos >= 2 && pos <= points.max(2) * 2 && points > 2 {
        pos
    } else {
        points
    };
    if n <= 1 {
        return vec![0.0];
    }
    (0..n).map(|i| i as f32 / (n - 1) as f32).collect()
}

#[derive(Clone, Copy)]
struct Opts {
    audio: bool,
    points: usize,
    secs: f64,
    /// Alternative contexts tried for a parameter without effect at the model defaults.
    contexts: usize,
    /// Audio for every `stride`-th parameter of a model (offset rotating per model); 1 = all.
    stride: usize,
    /// Only models `k` of every `n` (enumeration order), to split a group over parallel tests.
    shard: (usize, usize),
}

fn audit_group(key: &str, o: &Opts) -> Vec<Finding> {
    let g = group(key);
    let s = Schema::get();
    let sg = s.groups().iter().find(|x| x.key == g.key).unwrap();
    let kind = block_kind(g.key).unwrap();
    let mut rig = Rig::new(g, o.secs);
    let block = s.by_path(g.block).unwrap();
    let sel = rig
        .store
        .lookup(&format!("{}/{}", g.block, sg.selector))
        .unwrap();
    let mut out = Vec::new();
    for (mi, m) in s.models(sg).iter().enumerate() {
        if m.name == "NONE" || mi % o.shard.1 != o.shard.0 {
            continue;
        }
        rig.store.set_raw(sel, m.enum_index, 0).unwrap();
        rig.drain();
        assert_eq!(
            rig.slot().model_index(),
            Some(m.enum_index as u16),
            "{key} {}",
            m.name
        );
        let stub = create_model(kind, m.name).is_stub();
        let params: Vec<(&'static str, &'static Desc)> =
            s.model_params(m).filter(|(_, d)| !d.unused_slot).collect();
        // every parameter back to its default (model switch already did) and the default render
        for (pi, &(target, d)) in params.iter().enumerate() {
            let mut f = Finding {
                model: m.name,
                slot: target,
                name: d.name,
                verdict: Verdict::Ok,
                note: String::new(),
            };
            let path = format!("{}/{}", g.block, target);
            let id = s.child(block, target).unwrap_or_else(|| panic!("{path}"));
            let pos = match rig.pipe.map().binding_of(id) {
                Some((_, _, Some(p))) if p >= TARGET_BASE => p - TARGET_BASE,
                _ => {
                    f.verdict = Verdict::Unbound;
                    f.note = format!("{path} not bound to a model target");
                    out.push(f);
                    continue;
                }
            };
            if stub {
                f.verdict = Verdict::Stub;
                out.push(f);
                continue;
            }
            if d.positions() == 1
                || (d.min == d.max && matches!(d.ty, ParamType::LinF | ParamType::LogF))
            {
                f.verdict = Verdict::Justified;
                f.note = "fixed: the schema range is a single value".into();
                out.push(f);
                continue;
            }
            let pts = sweep(d, o.points);
            // readback at every sweep point
            let mut rb = Vec::new();
            for &p in &pts {
                rig.store.set_norm(id, p, 0).unwrap();
                rig.drain();
                let want = eng(d, rig.store.get_raw(id));
                let got = rig.slot().model().and_then(|m| m.param(pos));
                match got {
                    Some(v) if (v - want).abs() <= 1e-4 * want.abs().max(1.0) => {}
                    Some(v) => rb.push(format!("{want}->{v}")),
                    None => rb.push(format!("{want}->none")),
                }
            }
            rig.store.set_raw(id, d.default_raw, 0).unwrap();
            rig.drain();
            if !rb.is_empty() {
                f.verdict = Verdict::Readback;
                f.note = rb.join(" ");
                out.push(f);
                continue;
            }
            if !o.audio || (pi + mi) % o.stride != 0 {
                f.verdict = Verdict::NoAudio;
                out.push(f);
                continue;
            }
            let just = JUSTIFIED
                .iter()
                .find(|j| (j.0 == "*" || j.0 == m.name) && j.1 == d.name);
            f.verdict = match audible(
                &mut rig,
                id,
                d,
                &params,
                g,
                &pts,
                o.contexts,
                just.is_some(),
            ) {
                Outcome::Live(0, db) => {
                    f.note = format!("{db:.0} dB");
                    Verdict::Ok
                }
                Outcome::Live(ctx, db) => {
                    f.note = format!("ctx {ctx}: {db:.0} dB");
                    Verdict::OkCtx
                }
                Outcome::NonFinite(n) => {
                    f.note = n;
                    Verdict::NonFinite
                }
                Outcome::NonDet(n) => {
                    f.note = n;
                    Verdict::NonDet
                }
                Outcome::Dead(best) => match just {
                    Some(j) => {
                        f.note = j.2.to_string();
                        Verdict::Justified
                    }
                    None => {
                        f.note = format!("max diff {best:.0} dB");
                        Verdict::Dead
                    }
                },
            };
            out.push(f);
        }
        // leave the model at its defaults
        for &(target, d) in &params {
            let id = s.child(block, target).unwrap();
            rig.store.set_raw(id, d.default_raw, 0).unwrap();
        }
        rig.drain();
    }
    out
}

/// A context: positions (normalised) of other parameters of the model, on top of the model defaults.
type Ctx = Vec<(ParamId, f32)>;

fn is_switch(d: &Desc) -> bool {
    (2..=8).contains(&d.positions()) && matches!(d.ty, ParamType::Int | ParamType::Enum)
}

/// The switch position furthest from its default.
fn flipped(d: &Desc) -> f32 {
    if d.to_norm(d.default_raw) < 0.5 {
        1.0
    } else {
        0.0
    }
}

/// Contexts for a parameter without effect at the defaults, most likely enablers first:
/// 1. every other switch / small enum flipped (section switches, auto modes, filter types);
/// 2. one other parameter at an extreme (a band gain, a threshold, a mode), the rest at defaults;
/// 3. the same on top of 1 (a band type *and* its gain);
/// 4. seeded random settings of everything (percent parameters kept above 50 % mostly, so mixes and levels
///    rarely mute the model);
/// 5. a multi-position selector at an end together with one continuous parameter at an end.
fn contexts(id: ParamId, others: &[(ParamId, &'static Desc)], max: usize) -> Vec<Ctx> {
    let switches: Ctx = others
        .iter()
        .filter(|(_, d)| is_switch(d))
        .map(|&(p, d)| (p, flipped(d)))
        .collect();
    let mut v = vec![switches.clone()];
    // gains and thresholds (dB) are the most common enablers: try them first
    let mut order: Vec<(ParamId, &'static Desc)> = others.to_vec();
    order.sort_by_key(|(_, d)| d.unit != wing_model::Unit::Db);
    for base in [Vec::new(), switches] {
        for &(p, d) in &order {
            // two-position switches flip; everything else goes to both ends (a 3-way STOP/SLOW/FAST)
            let ends: &[f32] = if is_switch(d) && d.positions() == 2 {
                &[-1.0]
            } else {
                &[1.0, 0.0]
            };
            for &e in ends {
                let e = if e < 0.0 { flipped(d) } else { e };
                let mut c = base.clone();
                c.retain(|x| x.0 != p);
                c.push((p, e));
                v.push(c);
            }
        }
    }
    let mut seed = 0x9E37_79B9u64 ^ id.0 as u64;
    for _ in 0..8 {
        let mut c = Ctx::new();
        for &(p, d) in others {
            seed = seed
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            let r = (seed >> 40) as f32 / (1u64 << 24) as f32;
            let r = if d.unit == wing_model::Unit::Percent && r < 0.3 {
                r + 0.5
            } else {
                r
            };
            c.push((p, r));
        }
        v.push(c);
    }
    // a multi-position selector at either end together with one continuous parameter at either end (a bus
    // EQ band switched from shelf to PEQ *and* given gain)
    let selectors = others
        .iter()
        .filter(|(_, d)| d.ty == ParamType::Enum && d.positions() > 2);
    for &(a, _) in selectors {
        for ea in [0.0, 1.0] {
            for &(b, db) in others {
                if matches!(db.ty, ParamType::LinF | ParamType::LogF) {
                    for eb in [1.0, 0.0] {
                        v.push(vec![(a, ea), (b, eb)]);
                    }
                }
            }
        }
    }
    v.truncate(max);
    v
}

/// Sweep `id` over `pts` at the model defaults (checking repeatability and finiteness), then over the range
/// ends in the [`contexts`] until live.
#[allow(clippy::too_many_arguments)]
fn audible(
    rig: &mut Rig,
    id: ParamId,
    d: &Desc,
    params: &[(&'static str, &'static Desc)],
    g: &Group,
    pts: &[f32],
    max_ctx: usize,
    justified: bool,
) -> Outcome {
    let s = Schema::get();
    let block = s.by_path(g.block).unwrap();
    let others: Vec<(ParamId, &'static Desc)> = params
        .iter()
        .map(|&(t, pd)| (s.child(block, t).unwrap(), pd))
        .filter(|&(p, _)| p != id)
        .collect();
    let ends = [pts[0], pts[pts.len() - 1]];
    let mut best = f64::NEG_INFINITY;
    let ctxs = if justified {
        Vec::new()
    } else {
        contexts(id, &others, max_ctx)
    };
    for (k, ctx) in std::iter::once(&Ctx::new()).chain(ctxs.iter()).enumerate() {
        for &(p, pd) in &others {
            rig.store.set_raw(p, pd.default_raw, 0).unwrap();
        }
        for &(p, x) in ctx {
            rig.store.set_norm(p, x, 0).unwrap();
        }
        let sweep: &[f32] = if k == 0 { pts } else { &ends };
        let mut outs = Vec::with_capacity(sweep.len());
        for &p in sweep {
            rig.store.set_norm(id, p, 0).unwrap();
            rig.drain();
            let y = rig.render();
            if y.iter().any(|v| !v.is_finite()) {
                restore(rig, id, d, params, block);
                return Outcome::NonFinite(format!("ctx {k}, at {p:.2} of the range"));
            }
            outs.push(y);
        }
        if k == 0 {
            // repeatability: the first point rendered again must be bit-identical
            rig.store.set_norm(id, pts[0], 0).unwrap();
            rig.drain();
            let again = rig.render();
            let floor = rel_db(&outs[0], &again);
            if floor > LIVE_DB {
                restore(rig, id, d, params, block);
                return Outcome::NonDet(format!("repeat differs by {floor:.0} dB"));
            }
        }
        let mut m = f64::NEG_INFINITY;
        for i in 1..outs.len() {
            m = m.max(rel_db(&outs[i - 1], &outs[i]));
        }
        best = best.max(m);
        if m > LIVE_DB {
            restore(rig, id, d, params, block);
            return Outcome::Live(k, m);
        }
    }
    restore(rig, id, d, params, block);
    Outcome::Dead(best)
}

enum Outcome {
    /// Context index that made it live, largest difference (dB).
    Live(usize, f64),
    Dead(f64),
    NonFinite(String),
    NonDet(String),
}

fn restore(
    rig: &mut Rig,
    id: ParamId,
    d: &Desc,
    params: &[(&'static str, &'static Desc)],
    block: ParamId,
) {
    let s = Schema::get();
    rig.store.set_raw(id, d.default_raw, 0).unwrap();
    for &(t, pd) in params {
        rig.store
            .set_raw(s.child(block, t).unwrap(), pd.default_raw, 0)
            .unwrap();
    }
    rig.drain();
}

fn report(key: &str, f: &[Finding]) -> String {
    let mut counts: BTreeMap<Verdict, usize> = BTreeMap::new();
    for x in f {
        *counts.entry(x.verdict).or_default() += 1;
    }
    let mut s = format!("{key}: {counts:?}\n");
    for x in f {
        if !matches!(
            x.verdict,
            Verdict::Ok | Verdict::OkCtx | Verdict::NoAudio | Verdict::Stub
        ) {
            s += &format!(
                "  {:?} {} slot {} ({}): {}\n",
                x.verdict, x.model, x.slot, x.name, x.note
            );
        }
    }
    s
}

fn check(key: &str, o: &Opts) {
    let f = audit_group(key, o);
    let r = report(key, &f);
    if std::env::var_os("WING_AUDIT_REPORT").is_some() {
        println!("{r}");
        for x in &f {
            if x.verdict == Verdict::OkCtx {
                println!("  ctx {} {} {}", x.model, x.name, x.note);
            }
        }
    }
    let bad: Vec<&Finding> = f
        .iter()
        .filter(|x| {
            matches!(
                x.verdict,
                Verdict::Dead
                    | Verdict::NonDet
                    | Verdict::NonFinite
                    | Verdict::Readback
                    | Verdict::Unbound
            )
        })
        .collect();
    assert!(bad.is_empty(), "{r}");
}

// ---- tests ---------------------------------------------------------------------------------------------

const BIND_ONLY: Opts = Opts {
    audio: false,
    points: 3,
    secs: 0.0,
    contexts: 0,
    stride: 1,
    shard: (0, 1),
};

/// Binding + readback for every parameter of every group (no audio).
#[test]
fn binding_and_readback_all_groups() {
    for g in GROUPS {
        check(g.key, &BIND_ONLY);
    }
}

const SAMPLED_STRIP: Opts = Opts {
    audio: true,
    points: 2,
    secs: 0.4,
    contexts: 400,
    stride: 3,
    shard: (0, 1),
};

const SAMPLED_FX: Opts = Opts {
    audio: true,
    points: 2,
    secs: 0.75,
    contexts: 120,
    stride: 6,
    shard: (0, 1),
};

#[test]
fn audio_sampled_ch_tool_gate() {
    check("ch/*/flt", &SAMPLED_STRIP);
    check("ch/*/gate", &SAMPLED_STRIP);
}

#[test]
fn audio_sampled_ch_eq_dyn() {
    check("ch/*/eq", &SAMPLED_STRIP);
    check("ch/*/dyn", &SAMPLED_STRIP);
}

const fn shard(o: Opts, k: usize, n: usize) -> Opts {
    Opts { shard: (k, n), ..o }
}

#[test]
fn audio_sampled_fx_a() {
    check("fx/1", &shard(SAMPLED_FX, 0, 2));
}

#[test]
fn audio_sampled_fx_b() {
    check("fx/1", &shard(SAMPLED_FX, 1, 2));
}

const FULL_STRIP: Opts = Opts {
    audio: true,
    points: 5,
    secs: 1.5,
    contexts: 400,
    stride: 1,
    shard: (0, 1),
};

const FULL_FX: Opts = Opts {
    audio: true,
    points: 5,
    secs: 4.0,
    contexts: 400,
    stride: 1,
    shard: (0, 1),
};

macro_rules! full {
    ($name:ident, $key:expr, $o:expr) => {
        #[test]
        #[ignore = "slow: run with make check-full"]
        fn $name() {
            check($key, &$o);
        }
    };
}

full!(full_ch_flt, "ch/*/flt", FULL_STRIP);
full!(full_ch_gate, "ch/*/gate", FULL_STRIP);
full!(full_ch_eq, "ch/*/eq", FULL_STRIP);
full!(full_ch_dyn, "ch/*/dyn", FULL_STRIP);
full!(full_aux_eq, "aux/*/eq", FULL_STRIP);
full!(full_aux_dyn, "aux/*/dyn", FULL_STRIP);
full!(full_bus_eq, "bus/*/eq", FULL_STRIP);
full!(full_bus_dyn, "bus/*/dyn", FULL_STRIP);
full!(full_main_eq, "main/*/eq", FULL_STRIP);
full!(full_main_dyn, "main/*/dyn", FULL_STRIP);
full!(full_mtx_eq, "mtx/*/eq", FULL_STRIP);
full!(full_mtx_dyn, "mtx/*/dyn", FULL_STRIP);
full!(full_fx1_a, "fx/1", shard(FULL_FX, 0, 4));
full!(full_fx1_b, "fx/1", shard(FULL_FX, 1, 4));
full!(full_fx1_c, "fx/1", shard(FULL_FX, 2, 4));
full!(full_fx1_d, "fx/1", shard(FULL_FX, 3, 4));
full!(full_fx9_a, "fx/9", shard(FULL_FX, 0, 2));
full!(full_fx9_b, "fx/9", shard(FULL_FX, 1, 2));

// ---- T081 regression: WARM drive -----------------------------------------------------------------------

/// WARM drive set exactly as WING-Edit does (`ch/7/dyn/1`, the slot hash, after `ch/7/dyn/mdl = WARM`)
/// audibly changes a -18 dBFS 1 kHz tone through the full pipeline -> channel strip -> slot -> model path.
#[test]
fn warm_drive_is_audible_through_the_pipeline() {
    let render = |drv: &str| -> Vec<f32> {
        let store = ParamStore::new();
        let mut b = ParamMapBuilder::new();
        bind_channel_strip(&mut b, 7, NodeId(0)).unwrap();
        let mut host = One(Host::Ch(Box::new(ChannelStrip::channel(FS, BLOCK))));
        let mut pipe = ParamPipeline::new(&store, b.build(), PipelineConfig::default()).unwrap();
        pipe.sync_all(&store, &mut host);
        let set = |p: &str, v: &str| {
            store.set_text(store.lookup(p).unwrap(), v, 0).unwrap();
        };
        set("ch/7/dyn/mdl", "WARM");
        set("ch/7/dyn/on", "1");
        set("ch/7/dyn/1", drv);
        while pipe.apply(&mut host).changes > 0 {}
        let Host::Ch(s) = &mut host.0 else {
            unreachable!()
        };
        assert_eq!(s.dyn_slot().model().unwrap().param(0), drv.parse().ok());
        let amp = 0.125f32; // -18 dBFS
        let mut out = Vec::new();
        for blk in 0..300 {
            let mut l: Vec<f32> = (0..BLOCK)
                .map(|i| {
                    let t = (blk * BLOCK + i) as f32 / FS as f32;
                    amp * (2.0 * std::f32::consts::PI * 1000.0 * t).sin()
                })
                .collect();
            let mut r = l.clone();
            s.process_stereo(&mut l, &mut r);
            if blk >= 150 {
                out.extend_from_slice(&l);
            }
        }
        out
    };
    let lo = render("10");
    let def = render("33");
    let hi = render("125");
    let d_hi = rel_db(&lo, &hi);
    let d_def = rel_db(&def, &hi);
    assert!(
        d_hi > -20.0,
        "drive 10 -> 125 changes the output by only {d_hi:.1} dB"
    );
    assert!(
        d_def > -25.0,
        "drive 33 -> 125 changes the output by only {d_def:.1} dB"
    );
}

// ---- T085: gate slot vs dynamics slot, block wrapper ----------------------------------------------------

/// A channel strip bound as `ch/<ch>` behind the real pipeline: apply `sets` (text, as WING-Edit's values
/// arrive) in order, then render `secs` of the programme.
fn ch_render(ch: u32, sets: &[(String, String)], secs: f64) -> Vec<f32> {
    let store = ParamStore::new();
    let mut b = ParamMapBuilder::new();
    bind_channel_strip(&mut b, ch, NodeId(0)).unwrap();
    let mut host = One(Host::Ch(Box::new(ChannelStrip::channel(FS, BLOCK))));
    let mut pipe = ParamPipeline::new(&store, b.build(), PipelineConfig::default()).unwrap();
    pipe.sync_all(&store, &mut host);
    for (p, v) in sets {
        let id = store.lookup(p).unwrap_or_else(|| panic!("{p}"));
        store
            .set_text(id, v, 0)
            .unwrap_or_else(|e| panic!("{p} = {v}: {e:?}"));
        while pipe.apply(&mut host).changes > 0 {}
    }
    host.0.settle();
    let (xl, xr) = programme(secs);
    let mut out = Vec::with_capacity(2 * xl.len());
    let (mut l, mut r) = (vec![0.0f32; BLOCK], vec![0.0f32; BLOCK]);
    for (a, c) in xl.chunks(BLOCK).zip(xr.chunks(BLOCK)) {
        let n = a.len();
        l[..n].copy_from_slice(a);
        r[..n].copy_from_slice(c);
        host.0.process(&mut l[..n], &mut r[..n]);
        out.extend_from_slice(&l[..n]);
        out.extend_from_slice(&r[..n]);
    }
    out
}

fn sets(v: &[(&str, &str)]) -> Vec<(String, String)> {
    v.iter()
        .map(|(a, b)| (a.to_string(), b.to_string()))
        .collect()
}

/// T085 regression, the user's exact state and path: ch 21 (VOX2) with WARM in the *gate* slot as WING-Edit
/// pushes it after its legacy-scene upgrade (`gate.wmix = gate.mix`, block `mix` kept: both 5 %), then the
/// live session's sets `ch/21/gate/5` (MIX) = 100 % and `ch/21/gate/1` (DRIVE) 10 -> 125 %. Before the fix the
/// slot blended WARM at the block's 5 % on top of `wmix`, so drive changed this programme by only -35 dB (a
/// -18 dBFS 220 Hz tone by -49 dB: inaudible); now -9 dB, as with block mix 100 %.
#[test]
fn warm_in_gate_slot_drive_is_audible_with_upgraded_block_mix() {
    let scene = [
        ("ch/21/gate/mdl", "WARM"),
        ("ch/21/gate/on", "1"),
        ("ch/21/gate/mix", "5"),
        ("ch/21/gate/gain", "0"),
        ("ch/21/gate/1", "100"),
        ("ch/21/gate/2", "100"),
        ("ch/21/gate/3", "0.1"),
        ("ch/21/gate/4", "0"),
        ("ch/21/gate/5", "5"),
    ];
    let with = |slot: &str, extra: &[(&str, &str)]| {
        let mut v: Vec<(String, String)> = sets(&scene)
            .into_iter()
            .chain(sets(extra))
            .map(|(p, x)| (p.replace("/gate/", &format!("/{slot}/")), x))
            .collect();
        v.dedup();
        ch_render(21, &v, 1.5)
    };
    for slot in ["gate", "dyn"] {
        let lo = with(slot, &[("ch/21/gate/5", "100"), ("ch/21/gate/1", "10")]);
        let hi = with(slot, &[("ch/21/gate/5", "100"), ("ch/21/gate/1", "125")]);
        let d = rel_db(&lo, &hi);
        assert!(
            d > -20.0,
            "{slot} WARM drive 10 -> 125 % changes the output by only {d:.1} dB"
        );
        // WARM's MIX (wmix) is the block MIX: one blend, not mix * wmix
        let a = with(slot, &[]);
        let b = with(slot, &[("ch/21/gate/mix", "100")]);
        assert_eq!(
            rel_db(&a, &b),
            f64::NEG_INFINITY,
            "{slot}: block mix applied on top of wmix"
        );
    }
}

/// Every dynamics model (all 32 of the shared gate/dyn table) hosted in the gate slot and in the dynamics
/// slot, set through `ch/1/gate/*` resp. `ch/1/dyn/*`: the outputs are bit-identical at the model defaults and
/// with each parameter at either range end (`stride` > 1: a rotating subset of the parameters). Every
/// parameter is written explicitly in both slots: the factory scene's dyn values (e.g. COMP `rel` 150.33 ms)
/// differ from the model's pardef defaults a freshly selected gate model gets (150 ms), which is state, not
/// slot processing.
fn slot_parity(stride: usize, secs: f64) {
    let s = Schema::get();
    let gate = s.groups().iter().find(|g| g.key == "ch/*/gate").unwrap();
    let dynm = s.groups().iter().find(|g| g.key == "ch/*/dyn").unwrap();
    let gm: Vec<_> = s.models(gate).iter().map(|m| m.name).collect();
    let dm: Vec<_> = s.models(dynm).iter().map(|m| m.name).collect();
    assert_eq!(gm, dm, "gate and dyn offer the same models");
    assert_eq!(gm.len(), 32);
    let tmp = ParamStore::new();
    let mut bad = Vec::new();
    for (mi, m) in s.models(gate).iter().enumerate() {
        // (target, value text) settings to compare; None = model defaults
        let mut cases: Vec<Option<(&str, String)>> = vec![None];
        tmp.set_text(tmp.lookup("ch/1/gate/mdl").unwrap(), m.name, 0)
            .unwrap();
        let params: Vec<_> = s.model_params(m).filter(|(_, d)| !d.unused_slot).collect();
        let text = |t: &str, raw: u32| {
            let id = tmp.lookup(&format!("ch/1/gate/{t}")).unwrap();
            tmp.set_raw(id, raw, 0).unwrap();
            let mut txt = String::new();
            tmp.format_token(id, &mut txt).unwrap();
            txt
        };
        let defaults: Vec<(&str, String)> = params
            .iter()
            .map(|(t, d)| (*t, text(t, d.default_raw)))
            .collect();
        for (pi, (t, d)) in params.iter().enumerate() {
            if (pi + mi) % stride != 0 {
                continue;
            }
            for p in [0.0, 1.0] {
                cases.push(Some((t, text(t, d.from_norm(p).unwrap()))));
            }
        }
        for case in cases {
            let run = |slot: &str| {
                // the hosting slot first in the process order, so both see the strip input on the
                // same sample grid (behind the other, idle slot the input arrives delayed by that slot's
                // constant latency, which shifts block-rate control updates)
                let order = if slot == "gate" { "GEDI" } else { "DGEI" };
                let mut v = vec![
                    ("ch/1/proc".to_string(), order.to_string()),
                    (format!("ch/1/{slot}/mdl"), m.name.to_string()),
                    (format!("ch/1/{slot}/on"), "1".to_string()),
                ];
                v.extend(
                    defaults
                        .iter()
                        .map(|(t, x)| (format!("ch/1/{slot}/{t}"), x.clone())),
                );
                if let Some((t, txt)) = &case {
                    v.push((format!("ch/1/{slot}/{t}"), txt.clone()));
                }
                ch_render(1, &v, secs)
            };
            let diff = rel_db(&run("gate"), &run("dyn"));
            if diff != f64::NEG_INFINITY {
                bad.push(format!("{} {case:?}: gate vs dyn {diff:.1} dB", m.name));
            }
        }
    }
    assert!(bad.is_empty(), "gate/dyn slot parity:\n{}", bad.join("\n"));
}

#[test]
fn gate_and_dyn_slot_parity_sampled() {
    slot_parity(4, 0.3);
}

#[test]
#[ignore = "slow: run with make check-full"]
fn full_gate_and_dyn_slot_parity() {
    slot_parity(1, 1.0);
}

/// The block wrapper (`on`, `mix`, `gain`) of both slots for every model: `gain` always acts; `mix` blends
/// dry/wet unless the model owns its mix (WARM: `wmix` is the MIX and the block `mix` is not applied again,
/// T085), in which case it must have no effect at all.
#[test]
fn slot_wrapper_audit_gate_and_dyn() {
    let s = Schema::get();
    let gate = s.groups().iter().find(|g| g.key == "ch/*/gate").unwrap();
    let mut bad = Vec::new();
    for m in s.models(gate) {
        let owns = create_model(BlockKind::Dynamics, m.name).owns_mix();
        for slot in ["gate", "dyn"] {
            let r = |extra: &[(&str, &str)]| {
                let mut v = vec![
                    (format!("ch/1/{slot}/mdl"), m.name.to_string()),
                    (format!("ch/1/{slot}/on"), "1".to_string()),
                ];
                v.extend(
                    extra
                        .iter()
                        .map(|(k, x)| (format!("ch/1/{slot}/{k}"), x.to_string())),
                );
                ch_render(1, &v, 0.3)
            };
            let full = r(&[]);
            let model_live = rel_db(&full, &r(&[("on", "0")])) > LIVE_DB;
            if rel_db(&full, &r(&[("gain", "6")])) <= LIVE_DB {
                bad.push(format!("{slot} {}: gain dead", m.name));
            }
            let mix_live = rel_db(&full, &r(&[("mix", "30")])) > LIVE_DB;
            match (owns, model_live, mix_live) {
                (true, _, true) => bad.push(format!(
                    "{slot} {}: block mix applied on top of the model's own",
                    m.name
                )),
                (false, true, false) => bad.push(format!("{slot} {}: block mix dead", m.name)),
                _ => {}
            }
        }
    }
    assert!(bad.is_empty(), "slot wrapper:\n{}", bad.join("\n"));
}
