//! Test rig for the complete mix graph: store -> routing -> `build_mix_graph` -> engine + pipeline.

#![allow(dead_code)]

use std::time::Duration;

use wing_engine::mix::{build_mix_graph, InputSignal, MixConfig, MixNodes, MixSignal};
use wing_engine::params::{ParamPipeline, PipelineConfig};
use wing_engine::routing::{RoutingControl, Side, Strip};
use wing_engine::{Engine, EngineConfig, Port};
use wing_model::{ParamStore, Snapshot, Value};

pub const BLOCK: usize = 64;

pub fn engine_cfg(workers: usize) -> EngineConfig {
    EngineConfig {
        workers,
        spin: Duration::from_micros(20),
        rt_workers: false,
        workgroup: None,
    }
}

pub struct Rig {
    pub e: Engine,
    pub pipe: ParamPipeline,
    pub nodes: MixNodes,
    pub routing: RoutingControl,
    pub tiers: [u8; 16],
    /// The FX rack of the graph.
    pub fx: wing_engine::fx::FxRack,
    /// External input generator state.
    pub t: u64,
}

pub fn rig(store: &ParamStore, cfg: MixConfig, workers: usize) -> Rig {
    let mut routing = RoutingControl::new(store);
    let mg = build_mix_graph(store, &mut routing, cfg).expect("build");
    let tiers = mg.bus_tiers;
    let fx = mg.fx.clone();
    let (graph, map, nodes) = mg.finish().expect("finish");
    let mut e = Engine::new(graph, engine_cfg(workers));
    let mut pipe = ParamPipeline::new(store, map, PipelineConfig::default()).unwrap();
    pipe.sync_all(store, &mut e);
    Rig {
        e,
        pipe,
        nodes,
        routing,
        tiers,
        fx,
        t: 0,
    }
}

pub fn id(store: &ParamStore, p: &str) -> wing_model::NodeId {
    store.lookup(p).unwrap_or_else(|| panic!("{p}"))
}

pub fn setf(store: &ParamStore, p: &str, v: f32) {
    store.set(id(store, p), Value::Float(v)).unwrap();
}

pub fn seti(store: &ParamStore, p: &str, v: i32) {
    store.set(id(store, p), Value::Int(v)).unwrap();
}

pub fn sett(store: &ParamStore, p: &str, v: &str) {
    store
        .set_text(id(store, p), v, 0)
        .unwrap_or_else(|e| panic!("{p}={v}: {e}"));
}

pub fn getf(store: &ParamStore, p: &str) -> f32 {
    store.get_f64(id(store, p)).unwrap() as f32
}

/// Connect an input strip (`ch/N` or `aux/N`) to USB input `usb` (mono).
pub fn connect_usb(store: &ParamStore, strip: &str, usb: u32) {
    sett(store, &format!("{strip}/in/conn/grp"), "USB");
    seti(store, &format!("{strip}/in/conn/in"), usb as i32);
    seti(store, &format!("{strip}/in/set/altsrc"), 0);
    seti(store, &format!("{strip}/in/set/srcauto"), 0);
    sett(store, &format!("io/in/USB/{usb}/mode"), "M");
}

/// Deterministic noise-ish sample for external channel `ch` at absolute sample `n`.
pub fn sig(ch: usize, n: u64) -> f32 {
    let x = (n as f32 * 0.0137 * (ch + 1) as f32).sin() * 0.3
        + (n as f32 * 0.0021 * (ch as f32 + 0.5)).cos() * 0.2;
    x * 0.8
}

impl Rig {
    /// Fill the given USB inputs (1-based) with [`sig`], every other input with silence.
    pub fn feed(&mut self, usb: &[u32]) {
        for ch in 0..48 {
            let on = usb.contains(&(ch as u32 + 1));
            let t = self.t;
            for (i, s) in self.e.external_mut(ch).iter_mut().enumerate() {
                *s = if on { sig(ch, t + i as u64) } else { 0.0 };
            }
        }
        self.t += BLOCK as u64;
    }

    /// Apply pending parameter changes, then run one cycle (parallel engine).
    pub fn cycle(&mut self, usb: &[u32]) {
        self.feed(usb);
        self.pipe.apply(&mut self.e);
        self.e.process_cycle();
    }

    pub fn cycles(&mut self, n: usize, usb: &[u32]) {
        for _ in 0..n {
            self.cycle(usb);
        }
    }

    pub fn port(&self, p: Port) -> Vec<f32> {
        self.e.port(p).to_vec()
    }

    pub fn input(&self, strip: usize, s: InputSignal, side: Side) -> Vec<f32> {
        self.port(self.nodes.input_port(strip, s, side))
    }

    pub fn mix(&self, strip: Strip, s: MixSignal, side: Side) -> Vec<f32> {
        self.port(self.nodes.mix_port(strip, s, side).unwrap())
    }
}

/// Load `fixtures/PUBLIC.snap` into a fresh store with the real snapshot loader.
pub fn public_store() -> ParamStore {
    let store = ParamStore::new();
    let text = std::fs::read_to_string(super::PUBLIC_SNAP).expect("read PUBLIC.snap");
    Snapshot::load(&store, &text).expect("load PUBLIC.snap");
    store
}

/// Least-squares gain `y ~ g x` and the relative residual.
pub fn fit(x: &[f32], y: &[f32]) -> (f64, f64) {
    let (mut xy, mut xx, mut yy) = (0.0f64, 0.0f64, 0.0f64);
    for (&a, &b) in x.iter().zip(y) {
        xy += a as f64 * b as f64;
        xx += a as f64 * a as f64;
        yy += b as f64 * b as f64;
    }
    if xx == 0.0 {
        return (0.0, if yy == 0.0 { 0.0 } else { 1.0 });
    }
    let g = xy / xx;
    let res: f64 = x
        .iter()
        .zip(y)
        .map(|(&a, &b)| {
            let d = b as f64 - g * a as f64;
            d * d
        })
        .sum();
    (g, (res / yy.max(1e-30)).sqrt())
}

pub fn db(g: f64) -> f64 {
    20.0 * g.abs().log10()
}

pub fn lin(db: f64) -> f64 {
    10f64.powf(db / 20.0)
}
