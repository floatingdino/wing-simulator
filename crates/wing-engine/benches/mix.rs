//! Full engine graph benchmark (T036): input router -> 40 ch + 8 aux strips -> 16 busses -> 4 mains
//! -> 8 matrices -> feedback writer + USB output router, 64-sample blocks at 48 kHz.
//!
//! * `mix/public_*`: the PUBLIC.snap scene (channel EQ/dynamics, sends, bus->bus tiers) with every
//!   input strip moved to a live USB input.
//! * `mix/dense_*`: worst-case summing: every input strip sends to all 16 busses, 8 matrices and 4
//!   mains, every bus to all mains and matrices, all faders open (factory strip processing).
//!
//! `_workers` = the engine's default worker pool (cores - 1, max 7), `_serial` = one thread. One
//! cycle has a budget of 1333 us.

use std::time::Duration;

use criterion::{criterion_group, criterion_main, Criterion};
use wing_engine::mix::{build_mix_graph, MixConfig};
use wing_engine::params::{ParamPipeline, PipelineConfig};
use wing_engine::routing::RoutingControl;
use wing_engine::{Engine, EngineConfig};
use wing_model::{ParamStore, Snapshot, Value};

const PUBLIC_SNAP: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/PUBLIC.snap");

fn set(store: &ParamStore, path: &str, v: Value) {
    store.set(store.lookup(path).unwrap(), v).unwrap();
}

fn sett(store: &ParamStore, path: &str, text: &str) {
    store
        .set_text(store.lookup(path).unwrap(), text, 0)
        .unwrap();
}

fn connect_all(store: &ParamStore) {
    for n in 1..=48u32 {
        let p = if n <= 40 {
            format!("ch/{n}")
        } else {
            format!("aux/{}", n - 40)
        };
        sett(store, &format!("{p}/in/conn/grp"), "USB");
        set(store, &format!("{p}/in/conn/in"), Value::Int(n as i32));
        set(store, &format!("{p}/in/set/altsrc"), Value::Int(0));
        set(store, &format!("{p}/in/set/srcauto"), Value::Int(0));
    }
}

fn public() -> ParamStore {
    let store = ParamStore::new();
    let text = std::fs::read_to_string(PUBLIC_SNAP).unwrap();
    Snapshot::load(&store, &text).unwrap();
    connect_all(&store);
    store
}

fn dense() -> ParamStore {
    let store = ParamStore::new();
    connect_all(&store);
    let on = |p: String| set(&store, &p, Value::Int(1));
    let lvl = |p: String| set(&store, &p, Value::Float(-10.0));
    for n in 1..=48u32 {
        let p = if n <= 40 {
            format!("ch/{n}")
        } else {
            format!("aux/{}", n - 40)
        };
        set(&store, &format!("{p}/fdr"), Value::Float(-5.0));
        set(
            &store,
            &format!("{p}/pan"),
            Value::Float((n as f32 * 7.0) % 200.0 - 100.0),
        );
        for k in 1..=16 {
            on(format!("{p}/send/{k}/on"));
            lvl(format!("{p}/send/{k}/lvl"));
        }
        for k in 1..=8 {
            on(format!("{p}/send/MX{k}/on"));
            lvl(format!("{p}/send/MX{k}/lvl"));
        }
        for m in 1..=4 {
            on(format!("{p}/main/{m}/on"));
        }
    }
    for b in 1..=16 {
        set(&store, &format!("bus/{b}/fdr"), Value::Float(-3.0));
        for m in 1..=4 {
            on(format!("bus/{b}/main/{m}/on"));
        }
        for k in 1..=8 {
            on(format!("bus/{b}/send/MX{k}/on"));
            lvl(format!("bus/{b}/send/MX{k}/lvl"));
        }
    }
    for m in 1..=4 {
        set(&store, &format!("main/{m}/fdr"), Value::Float(0.0));
        for k in 1..=8 {
            on(format!("main/{m}/send/MX{k}/on"));
        }
    }
    for k in 1..=8 {
        set(&store, &format!("mtx/{k}/fdr"), Value::Float(0.0));
    }
    store
}

fn engine(store: &ParamStore, workers: Option<usize>) -> (Engine, RoutingControl) {
    let mut routing = RoutingControl::new(store);
    let (graph, map, _nodes) = build_mix_graph(store, &mut routing, MixConfig::default())
        .unwrap()
        .finish()
        .unwrap();
    let mut cfg = EngineConfig::default();
    if let Some(w) = workers {
        cfg.workers = w;
    }
    cfg.spin = Duration::from_micros(200);
    let mut e = Engine::new(graph, cfg);
    let mut pipe = ParamPipeline::new(store, map, PipelineConfig::default()).unwrap();
    pipe.sync_all(store, &mut e);
    for ch in 0..48 {
        for (i, s) in e.external_mut(ch).iter_mut().enumerate() {
            *s = (i as f32 * 0.01 * (ch + 1) as f32).sin() * 0.3;
        }
    }
    for _ in 0..50 {
        e.process_cycle();
    }
    (e, routing)
}

fn bench(c: &mut Criterion) {
    type Scene = fn() -> ParamStore;
    let scenes: [(&str, Scene); 2] = [("public", public), ("dense", dense)];
    for (name, scene) in scenes {
        let store = scene();
        let (mut e, _r) = engine(&store, None);
        let w = e.num_workers();
        c.bench_function(&format!("mix/{name}_workers{w}"), |b| {
            b.iter(|| e.process_cycle())
        });
        drop(e);
        let store = scene();
        let (mut e, _r) = engine(&store, Some(0));
        c.bench_function(&format!("mix/{name}_serial"), |b| {
            b.iter(|| e.process_cycle_serial())
        });
    }
}

criterion_group!(benches, bench);
criterion_main!(benches);
