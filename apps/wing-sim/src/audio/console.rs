//! The console processing graph run by the daemon, and [`Console`]: engine + parameter pipeline
//! + host-bridge feeds, processed one engine block at a time.
//!
//! The graph is the T036 mix graph ([`wing_engine::mix::build_mix_graph`]):
//!
//! ```text
//! USB In 1..48 (externals) ─┐
//! oscillators ──────────────┴─ input router (T024, re-inputs one block late)
//!   ── 40 ch + 8 aux strips (T035 + fader/pan/sends) ── 16 busses (tiers) ── 4 mains ── 8 mtx
//!   ── USB-out router (output patch, every BUS/MAIN/MTX leg and user-signal tap) → driver
//!   ── monitor section (T038: solo busses with priority over cfg/mon/1|2 src)
//! host bridges read any of: mon/1..2 (monitor A/B incl. solo), main/1..4, bus/1..16,
//! matrix/1..8, usb_out/n.
//! ```
//!
//! Bus/main/matrix strip processing (EQ, dynamics, inserts, delay) is T037; the strips sum, fade
//! and pan only.
//!
//! Meters (T039, when [`ConsoleConfig::meters`] is set): the input strips write their IN/PFL/key/GR
//! cells while processing, and [`Console::cycle`] meters the mix graph's ports after each cycle
//! (strip outs, bus/main/matrix in/out, USB In/Out, oscillators, mix-output sources; see
//! `wing_engine::meters::PortMeters::for_mix`).

use std::sync::Arc;
use std::time::Instant;

use wing_engine::meters::{LevelBank, PortMeters};
use wing_engine::mix::{build_mix_graph, MixConfig, MixNodes};
use wing_engine::params::{ParamMap, ParamPipeline, PipelineConfig};
use wing_engine::routing::{Side, Strip, USB_CHANNELS};
use wing_engine::{Engine, EngineConfig, Port, RawWorkgroup};
use wing_hostio::{BridgeProducer, Tap, TapChannel};
use wing_model::ParamStore;

use super::stats::AudioStats;
use super::ConsoleControl;
use crate::record::{RecordSlot, RecordTap};

/// Input channels and aux channels.
pub const CHANNELS: usize = 40;
pub const AUXES: usize = 8;
/// Input strips (ch 1..40 then aux 1..8, the input router's strip order).
pub const STRIPS: usize = CHANNELS + AUXES;

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
    /// Level bank the console meters into (T039); `None`: no metering.
    pub meters: Option<Arc<LevelBank>>,
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
            meters: None,
        }
    }
}

/// Graph node ids of the console (the mix graph's nodes).
#[derive(Debug, Clone)]
pub struct ConsoleNodes {
    pub mix: MixNodes,
    /// USB output router (48 outputs = driver USB Out 1..48).
    pub usb_out: wing_engine::NodeId,
}

impl ConsoleNodes {
    /// Engine port carrying a host-bridge tap lane, `None` = silence (not modelled yet).
    pub fn tap_port(&self, t: TapChannel) -> Option<Port> {
        let strip = match t.tap {
            Tap::UsbOut if (1..=USB_CHANNELS as u16).contains(&t.lane) => {
                return Some(Port::node(self.usb_out, t.lane as u32 - 1));
            }
            Tap::UsbOut | Tap::LocalOut => return None,
            Tap::Monitor(m @ 1..=2) => {
                let side = match t.lane {
                    1 => Side::L,
                    2 => Side::R,
                    _ => return None,
                };
                return Some(self.mix.monitor_port(m as usize, side));
            }
            Tap::Monitor(_) => return None,
            Tap::Main(n) => Strip::Main(n),
            Tap::Bus(n) => Strip::Bus(n),
            Tap::Matrix(n) => Strip::Mtx(n),
        };
        let side = match t.lane {
            1 => Side::L,
            2 => Side::R,
            _ => return None,
        };
        strip.index()?;
        self.mix.post(strip, side)
    }
}

// ---- graph ------------------------------------------------------------------------------------

/// Build the console graph and its parameter map. Deterministic node ids: a [`ParamMap`] /
/// [`ParamPipeline`] built for one instance stays valid for a rebuild (sample-rate change).
/// Allocates (control thread).
pub fn build_engine(
    cfg: &ConsoleConfig,
    store: &ParamStore,
    ctl: &mut ConsoleControl,
) -> Result<(Engine, ConsoleNodes, ParamMap), String> {
    let mix_cfg = MixConfig {
        block_size: cfg.block,
        sample_rate: cfg.sample_rate as f32,
        logic: Some(ctl.logic.shared()),
        meters: cfg.meters.clone(),
        monitor_levels: Some(ctl.monitor.levels()),
        ..MixConfig::default()
    };
    ctl.logic.refresh(store);
    let mg =
        build_mix_graph(store, &mut ctl.routing, mix_cfg).map_err(|e| format!("mix graph: {e}"))?;
    let (graph, map, mix) = mg.finish().map_err(|e| format!("console graph: {e:?}"))?;
    let engine = Engine::new(
        graph,
        EngineConfig {
            workers: cfg.workers,
            rt_workers: cfg.rt_workers,
            workgroup: cfg.workgroup,
            ..EngineConfig::default()
        },
    );
    let usb_out = mix.usb_out;
    Ok((engine, ConsoleNodes { mix, usb_out }, map))
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
    port_meters: Option<PortMeters>,
    cfg: ConsoleConfig,
    /// Recorder hand-over slot (T061) and the tap being recorded.
    rec_slot: Option<Arc<RecordSlot>>,
    rec: Option<Box<RecordTap>>,
}

impl Console {
    /// Build the console for `cfg`. `pipeline`: reuse one from a previous instance (rebuild), or
    /// `None` to take `store`'s RT consumer (only possible once per store). Pushes the store's
    /// full state into the new processors. Allocates (control thread).
    pub fn build(
        cfg: ConsoleConfig,
        store: &ParamStore,
        routing: &mut ConsoleControl,
        pipeline: Option<ParamPipeline>,
        bridges: Vec<super::hostio::Feed>,
        stats: Arc<AudioStats>,
    ) -> Result<Self, String> {
        let (mut engine, nodes, map) = build_engine(&cfg, store, routing)?;
        let mut pipeline = match pipeline {
            Some(p) => p,
            None => ParamPipeline::new(store, map, PipelineConfig::default())
                .map_err(|e| e.to_string())?,
        };
        pipeline.sync_all(store, &mut engine);
        let feeds = bridges
            .into_iter()
            .map(|(producer, taps)| BridgeFeed {
                producer,
                ports: taps.iter().map(|&t| nodes.tap_port(t)).collect(),
            })
            .collect();
        stats.set_period_ns((cfg.block as f64 * 1e9 / cfg.sample_rate as f64) as u64);
        let port_meters = cfg
            .meters
            .clone()
            .map(|b| PortMeters::for_mix(b, &nodes.mix));
        Ok(Self {
            engine,
            pipeline,
            nodes,
            feeds,
            silence: vec![0.0; cfg.block].into(),
            stats,
            port_meters,
            cfg,
            rec_slot: None,
            rec: None,
        })
    }

    /// Poll `slot` for recordings (T061) from now on; publishes this console's rate to it.
    /// Control thread.
    pub fn attach_recorder(&mut self, slot: Arc<RecordSlot>) {
        slot.set_sample_rate(self.cfg.sample_rate);
        self.rec_slot = Some(slot);
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
    /// process, meter the ports, hand USB Out 1..48 to `output(ch, buf)`, feed the recorder (T061) and the host bridges, publish
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
        if let Some(m) = &self.port_meters {
            m.capture(&self.engine);
        }
        let c = self.engine.cycle();
        self.pipeline.publish_telemetry(&mut self.engine, c);
        let usb = self.nodes.usb_out;
        for ch in 0..USB_CHANNELS {
            output(ch, self.engine.port(Port::node(usb, ch as u32)));
        }
        if let Some(slot) = &self.rec_slot {
            if slot.poll(&mut self.rec) {
                if let Some(r) = &mut self.rec {
                    r.resolve(&self.nodes);
                }
            }
            if let Some(r) = &mut self.rec {
                let engine = &self.engine;
                r.push(self.cfg.block, &self.silence, |p| engine.port(p));
            }
        }
        if !self.feeds.is_empty() {
            // Unstamped: the burst is stamped once by `end_burst` (T027).
            let n = self.cfg.block;
            let (engine, silence) = (&self.engine, &*self.silence);
            for f in &mut self.feeds {
                let ports = &f.ports;
                f.producer.push_with(n, 0, |i| match ports[i] {
                    Some(p) => engine.port(p),
                    None => silence,
                });
            }
        }
        self.stats.record(t0.elapsed().as_nanos() as u64);
    }

    /// End of a burst of cycles (everything one driver IO cycle made available): stamp the
    /// host bridges' producer position with the host time `now_ns`
    /// ([`wing_hostio::host_time_ns`]) and report the driver IO buffer `burst_frames`, from
    /// which the bridges derive their latency target. RT-safe.
    #[inline]
    pub fn end_burst(&mut self, now_ns: u64, burst_frames: usize) {
        if !self.feeds.is_empty() {
            let now = now_ns;
            for f in &mut self.feeds {
                f.producer.end_burst(now, burst_frames);
            }
        }
    }

    /// Run `cycles` engine cycles on silence without touching the driver or the host bridges
    /// and without recording cycle statistics: faults in buffers, warms caches and wakes the
    /// workers before real-time work arrives (session start, driver IO start). RT-safe.
    pub fn warm_up(&mut self, cycles: usize) {
        for _ in 0..cycles {
            self.pipeline.apply(&mut self.engine);
            for ch in 0..USB_CHANNELS {
                self.engine.external_mut(ch).fill(0.0);
            }
            self.engine.process_cycle();
        }
    }
}
