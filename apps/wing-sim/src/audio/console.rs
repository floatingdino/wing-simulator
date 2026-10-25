//! The console processing graph run by the daemon, and [`Console`]: engine + parameter pipeline
//! + host-bridge feeds, processed one engine block at a time.
//!
//! ```text
//! USB In 1..48 (externals) ─┐
//! oscillators ──────────────┴─ input router (T024) ── 40 ch + 8 aux strips (T035, pre-fader)
//!                                                       │  (PRE taps → USB-out router)
//!                                                       ▼
//!                       provisional main 1 sum (fader, mute, pan, main/1 send; main/1 fader)
//!                                                       │  (MAIN.1/2 taps → USB-out router)
//!                                                       ▼
//!                              USB-out router (output patch) → USB Out 1..48 → driver
//! host bridges read any of: main/1 (mon/1..2 = main/1 until T038), usb_out/n.
//! ```
//!
//! TODO(T036): the provisional main sum is a stand-in for busses/mains/matrices, sends and the
//! proper WING pan law. It sums every input strip post-fader into main 1 when the strip's
//! `main/1/on` is set (at `main/1/lvl`), with a linear balance pan, then applies `main/1`
//! fader/mute/pan. Everything else (busses, main 2-4, matrices, `wid`, `pre`) is ignored.

use std::sync::Arc;
use std::time::Instant;

use wing_engine::channel::{bind_aux_strip, bind_channel_strip, ChannelStrip};
use wing_engine::params::{
    BindError, BlockLayout, ParamMap, ParamMapBuilder, ParamMsg, ParamPipeline, PipelineConfig,
};
use wing_engine::routing::{
    add_input_stage, add_usb_output, InputRouter, IoNodes, MixGroup, RoutingControl, Side, Strip,
    StripPoint, Tap as RTap, TapPorts, NO_TAPS, USB_CHANNELS,
};
use wing_engine::{
    Engine, EngineConfig, GraphBuilder, NodeId, NodeIo, Port, ProcessCtx, Processor, RawWorkgroup,
};
use wing_hostio::{BridgeProducer, Tap, TapChannel};
use wing_model::ParamStore;

use super::stats::AudioStats;

/// Input channels and aux channels.
pub const CHANNELS: usize = 40;
pub const AUXES: usize = 8;
/// Input strips (ch 1..40 then aux 1..8, the input router's strip order).
pub const STRIPS: usize = CHANNELS + AUXES;
/// Block index of `main/1` on the provisional mix node.
const MAIN_BLOCK: u8 = STRIPS as u8;
/// Fader value meaning -oo.
const FADER_MIN_DB: f32 = -144.0;

/// Engine configuration for the console.
#[derive(Debug, Clone)]
pub struct ConsoleConfig {
    /// Nominal sample rate.
    pub sample_rate: u32,
    /// Engine block (frames per cycle).
    pub block: usize,
    /// Worker threads (in addition to the driver thread).
    pub workers: usize,
    /// Time-constraint policy on workers.
    pub rt_workers: bool,
    /// Workgroup workers join (see `workgroup.rs`).
    pub workgroup: Option<RawWorkgroup>,
}

impl Default for ConsoleConfig {
    fn default() -> Self {
        let d = EngineConfig::default();
        Self {
            sample_rate: 48_000,
            block: 64,
            workers: d.workers,
            rt_workers: d.rt_workers,
            workgroup: None,
        }
    }
}

/// Graph node ids of the console.
#[derive(Debug, Clone, Copy)]
pub struct ConsoleNodes {
    pub io: IoNodes,
    /// Strip `i` (ch 1..40 = 0..39, aux 1..8 = 40..47).
    pub strips: [NodeId; STRIPS],
    /// Provisional main 1 sum (2 outputs).
    pub main: NodeId,
    /// USB output router (48 outputs = driver USB Out 1..48).
    pub usb_out: NodeId,
}

impl ConsoleNodes {
    /// Engine port carrying a host-bridge tap lane, `None` = silence (not modelled yet).
    pub fn tap_port(&self, t: TapChannel) -> Option<Port> {
        let side = t.lane.checked_sub(1)? as u32;
        match t.tap {
            // TODO(T038): monitor section; until then both monitors follow main 1.
            Tap::Main(1) | Tap::Monitor(_) if side < 2 => Some(Port::node(self.main, side)),
            Tap::UsbOut if (side as usize) < USB_CHANNELS => Some(Port::node(self.usb_out, side)),
            _ => None,
        }
    }
}

// ---- provisional main sum ---------------------------------------------------------------------

/// `ch/N`, `aux/N`, `main/1`: fader, mute, pan.
const STRIP_MIX_LAYOUT: BlockLayout = BlockLayout {
    params: &[("fdr", 0), ("mute", 1), ("pan", 2)],
    targets: &[],
};
/// `ch/N/main/1`, `aux/N/main/1`: send on/level.
const MAIN_SEND_LAYOUT: BlockLayout = BlockLayout {
    params: &[("on", 3), ("lvl", 4)],
    targets: &[],
};

fn fader_lin(db: f32) -> f32 {
    if db <= FADER_MIN_DB {
        0.0
    } else {
        10f32.powf(db / 20.0)
    }
}

/// Linear balance pan, unity at centre (provisional, T036 owns the WING pan law).
fn balance(pan: f32) -> (f32, f32) {
    let p = pan.clamp(-100.0, 100.0) / 100.0;
    ((1.0 - p).min(1.0), (1.0 + p).min(1.0))
}

/// Fader/mute/pan(/send) state of one strip, as a per-side gain ramped linearly over a block.
#[derive(Clone, Copy, Debug)]
struct MixGain {
    fdr_db: f32,
    mute: bool,
    pan: f32,
    on: bool,
    lvl_db: f32,
    cur: [f32; 2],
    target: [f32; 2],
}

impl MixGain {
    const fn new() -> Self {
        Self {
            fdr_db: FADER_MIN_DB,
            mute: false,
            pan: 0.0,
            on: true,
            lvl_db: 0.0,
            cur: [0.0; 2],
            target: [0.0; 2],
        }
    }

    fn set(&mut self, param: u16, v: f32) {
        match param {
            0 => self.fdr_db = v,
            1 => self.mute = v >= 0.5,
            2 => self.pan = v,
            3 => self.on = v >= 0.5,
            4 => self.lvl_db = v,
            _ => return,
        }
        let g = if self.mute || !self.on {
            0.0
        } else {
            fader_lin(self.fdr_db) * fader_lin(self.lvl_db)
        };
        let (l, r) = balance(self.pan);
        self.target = [g * l, g * r];
    }

    fn silent(&self) -> bool {
        self.cur == [0.0; 2] && self.target == [0.0; 2]
    }

    /// `out[s] += x[s] * gain[s]` ramping from `cur` to `target` over the block.
    #[inline]
    fn accumulate(&mut self, side: usize, x: &[f32], out: &mut [f32]) {
        let (a, b) = (self.cur[side], self.target[side]);
        if a == b {
            for (o, &v) in out.iter_mut().zip(x) {
                *o += v * a;
            }
        } else {
            let step = (b - a) / x.len().max(1) as f32;
            for (k, (o, &v)) in out.iter_mut().zip(x).enumerate() {
                *o += v * (a + step * (k + 1) as f32);
            }
        }
    }

    #[inline]
    fn apply(&mut self, side: usize, io: &mut [f32]) {
        let (a, b) = (self.cur[side], self.target[side]);
        if a == b {
            if a != 1.0 {
                io.iter_mut().for_each(|v| *v *= a);
            }
        } else {
            let step = (b - a) / io.len().max(1) as f32;
            for (k, v) in io.iter_mut().enumerate() {
                *v *= a + step * (k + 1) as f32;
            }
        }
    }

    fn advance(&mut self) {
        self.cur = self.target;
    }
}

/// Provisional main 1: inputs = 48 strips x (L, R), outputs = main 1 L/R. RT-safe.
pub struct ProvisionalMain {
    strips: [MixGain; STRIPS],
    main: MixGain,
}

impl Default for ProvisionalMain {
    fn default() -> Self {
        let mut main = MixGain::new();
        main.on = true;
        Self {
            strips: [MixGain::new(); STRIPS],
            main,
        }
    }
}

impl Processor for ProvisionalMain {
    fn process(&mut self, _ctx: &ProcessCtx, io: &mut NodeIo<'_>) {
        let ins = io.inputs();
        let b = io.block_size();
        let out = io.outputs();
        let (l, rest) = out.split_at_mut(b);
        let r = &mut rest[..b];
        l.fill(0.0);
        r.fill(0.0);
        for (i, g) in self.strips.iter_mut().enumerate() {
            if 2 * i + 1 >= ins.len() || g.silent() {
                continue;
            }
            g.accumulate(0, ins.get(2 * i), l);
            g.accumulate(1, ins.get(2 * i + 1), r);
            g.advance();
        }
        self.main.apply(0, l);
        self.main.apply(1, r);
        self.main.advance();
    }

    fn set_param(&mut self, msg: &ParamMsg) {
        if let ParamMsg::Set {
            block,
            param,
            value,
        } = *msg
        {
            if block == MAIN_BLOCK {
                self.main.set(param, value);
            } else if let Some(g) = self.strips.get_mut(block as usize) {
                g.set(param, value);
            }
        }
    }
}

// ---- graph ------------------------------------------------------------------------------------

/// Build the console graph. Deterministic: the same `routing` gives the same node ids, so a
/// [`ParamMap`] / [`ParamPipeline`] built for one instance is valid for a rebuild (sample-rate
/// change). Allocates (control thread).
pub fn build_engine(
    cfg: &ConsoleConfig,
    routing: &mut RoutingControl,
) -> Result<(Engine, ConsoleNodes), String> {
    let fs = cfg.sample_rate as f64;
    let mut g = GraphBuilder::new(cfg.block, cfg.sample_rate as f32, USB_CHANNELS as u32);
    let e = |e: wing_engine::GraphError| format!("console graph: {e:?}");
    let io = add_input_stage(&mut g, routing).map_err(e)?;
    let mut strips = [NodeId(0); STRIPS];
    for (i, s) in strips.iter_mut().enumerate() {
        let (name, proc_) = if i < CHANNELS {
            (format!("ch{}", i + 1), ChannelStrip::channel(fs, cfg.block))
        } else {
            (
                format!("aux{}", i - CHANNELS + 1),
                ChannelStrip::aux(fs, cfg.block),
            )
        };
        let ins = [
            InputRouter::port(io.inputs, i, 0),
            InputRouter::port(io.inputs, i, 1),
        ];
        *s = g.add_node(name, 2, &ins, proc_).map_err(e)?;
    }
    let mix_in: Vec<Port> = strips
        .iter()
        .flat_map(|&n| [Port::node(n, 0), Port::node(n, 1)])
        .collect();
    let main = g
        .add_node("main1-provisional", 2, &mix_in, ProvisionalMain::default())
        .map_err(e)?;
    let mut taps: TapPorts = NO_TAPS;
    let mut wire = |t: RTap, p: Port| {
        if let Some(id) = t.id() {
            taps[id.0 as usize] = Some(p);
        }
    };
    for leg in 1..=2u16 {
        wire(
            RTap::Mix {
                grp: MixGroup::Main,
                leg,
            },
            Port::node(main, leg as u32 - 1),
        );
    }
    for (i, &n) in strips.iter().enumerate() {
        let strip = if i < CHANNELS {
            Strip::Ch(i as u8 + 1)
        } else {
            Strip::Aux((i - CHANNELS) as u8 + 1)
        };
        for (k, side) in [Side::L, Side::R].into_iter().enumerate() {
            wire(
                RTap::Strip {
                    strip,
                    point: StripPoint::PreFader,
                    side,
                },
                Port::node(n, k as u32),
            );
        }
    }
    let usb_out = add_usb_output(&mut g, routing, io.osc, &taps).map_err(e)?;
    let graph = g.build().map_err(e)?;
    let engine = Engine::new(
        graph,
        EngineConfig {
            workers: cfg.workers,
            rt_workers: cfg.rt_workers,
            workgroup: cfg.workgroup,
            ..EngineConfig::default()
        },
    );
    Ok((
        engine,
        ConsoleNodes {
            io,
            strips,
            main,
            usb_out,
        },
    ))
}

/// Bind the console's parameters (strips via T035, provisional mix) for the pipeline.
pub fn build_param_map(nodes: &ConsoleNodes) -> Result<ParamMap, BindError> {
    let mut b = ParamMapBuilder::new();
    for (i, &n) in nodes.strips.iter().enumerate() {
        let path = if i < CHANNELS {
            bind_channel_strip(&mut b, i as u32 + 1, n)?;
            format!("ch/{}", i + 1)
        } else {
            let k = (i - CHANNELS) as u32 + 1;
            bind_aux_strip(&mut b, k, n)?;
            format!("aux/{k}")
        };
        b.bind_block(&path, nodes.main, i as u8, &STRIP_MIX_LAYOUT)?;
        b.bind_block(
            &format!("{path}/main/1"),
            nodes.main,
            i as u8,
            &MAIN_SEND_LAYOUT,
        )?;
    }
    b.bind_block("main/1", nodes.main, MAIN_BLOCK, &STRIP_MIX_LAYOUT)?;
    Ok(b.build())
}

// ---- console ----------------------------------------------------------------------------------

/// A host bridge fed by the console: the producer and the engine port of each bridged channel
/// (`None` = silence).
pub struct BridgeFeed {
    pub producer: BridgeProducer,
    pub ports: Vec<Option<Port>>,
}

/// Engine + pipeline + bridge feeds. Everything on the per-cycle path is preallocated.
pub struct Console {
    engine: Engine,
    pipeline: ParamPipeline,
    nodes: ConsoleNodes,
    feeds: Vec<BridgeFeed>,
    silence: Box<[f32]>,
    stats: Arc<AudioStats>,
    cfg: ConsoleConfig,
}

impl Console {
    /// Build the console for `cfg`. `pipeline`: reuse one from a previous instance (rebuild), or
    /// `None` to take `store`'s RT consumer (only possible once per store). Pushes the store's
    /// full state into the new processors. Allocates (control thread).
    pub fn build(
        cfg: ConsoleConfig,
        store: &ParamStore,
        routing: &mut RoutingControl,
        pipeline: Option<ParamPipeline>,
        bridges: Vec<super::hostio::Feed>,
        stats: Arc<AudioStats>,
    ) -> Result<Self, String> {
        let (mut engine, nodes) = build_engine(&cfg, routing)?;
        let mut pipeline = match pipeline {
            Some(p) => p,
            None => {
                let map = build_param_map(&nodes).map_err(|e| format!("param map: {e:?}"))?;
                ParamPipeline::new(store, map, PipelineConfig::default())
                    .map_err(|e| e.to_string())?
            }
        };
        pipeline.sync_all(store, &mut engine);
        // Mix gains start at 0 and ramp to the store state over the first block (fade-in).
        let feeds = bridges
            .into_iter()
            .map(|(producer, taps)| BridgeFeed {
                producer,
                ports: taps.iter().map(|&t| nodes.tap_port(t)).collect(),
            })
            .collect();
        stats.set_period_ns((cfg.block as f64 * 1e9 / cfg.sample_rate as f64) as u64);
        Ok(Self {
            engine,
            pipeline,
            nodes,
            feeds,
            silence: vec![0.0; cfg.block].into(),
            stats,
            cfg,
        })
    }

    /// Give back the pipeline (to rebuild at another rate).
    pub fn into_pipeline(self) -> ParamPipeline {
        self.pipeline
    }

    pub fn config(&self) -> &ConsoleConfig {
        &self.cfg
    }

    pub fn nodes(&self) -> &ConsoleNodes {
        &self.nodes
    }

    pub fn engine(&self) -> &Engine {
        &self.engine
    }

    pub fn pipeline(&self) -> &ParamPipeline {
        &self.pipeline
    }

    /// Frames per cycle.
    pub fn block(&self) -> usize {
        self.cfg.block
    }

    pub fn sample_rate(&self) -> u32 {
        self.cfg.sample_rate
    }

    /// Driver USB Out `ch` (0-based) of the last cycle.
    pub fn usb_out(&self, ch: usize) -> &[f32] {
        self.engine.port(Port::node(self.nodes.usb_out, ch as u32))
    }

    /// Any port of the last cycle.
    pub fn port(&self, p: Port) -> &[f32] {
        self.engine.port(p)
    }

    /// One engine cycle: apply queued parameter changes, fill USB In 1..48 with `input(ch, buf)`,
    /// process, hand USB Out 1..48 to `output(ch, buf)`, feed the host bridges, publish
    /// telemetry and record the cycle time. RT-safe (no allocation, locks or syscalls).
    #[inline]
    pub fn cycle(
        &mut self,
        mut input: impl FnMut(usize, &mut [f32]),
        mut output: impl FnMut(usize, &[f32]),
    ) {
        let t0 = Instant::now();
        self.pipeline.apply(&mut self.engine);
        for ch in 0..USB_CHANNELS {
            input(ch, self.engine.external_mut(ch));
        }
        self.engine.process_cycle();
        let c = self.engine.cycle();
        self.pipeline.publish_telemetry(&mut self.engine, c);
        let usb = self.nodes.usb_out;
        for ch in 0..USB_CHANNELS {
            output(ch, self.engine.port(Port::node(usb, ch as u32)));
        }
        if !self.feeds.is_empty() {
            let now = wing_hostio::host_time_ns();
            let n = self.cfg.block;
            let (engine, silence) = (&self.engine, &*self.silence);
            for f in &mut self.feeds {
                let ports = &f.ports;
                f.producer.push_with(n, now, |i| match ports[i] {
                    Some(p) => engine.port(p),
                    None => silence,
                });
            }
        }
        self.stats.record(t0.elapsed().as_nanos() as u64);
    }
}
