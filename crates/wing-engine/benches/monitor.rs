//! Monitor section / solo benchmark (T038): the PUBLIC.snap mix graph (serial, one thread) with
//! the DCA/solo logic attached, monitor A EQ (8 bands), delay and limiter on, with no solo vs 8
//! channel solos + 2 bus solos into both monitors. The difference is the cost of the solo busses.

use criterion::{criterion_group, criterion_main, Criterion};
use wing_engine::mix::{build_mix_graph, MixConfig, MixLogic};
use wing_engine::params::{ParamPipeline, PipelineConfig};
use wing_engine::routing::RoutingControl;
use wing_engine::{Engine, EngineConfig};
use wing_model::{ParamStore, Snapshot};

const PUBLIC_SNAP: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/PUBLIC.snap");

fn sett(store: &ParamStore, path: &str, text: &str) {
    store
        .set_text(store.lookup(path).unwrap(), text, 0)
        .unwrap();
}

fn engine(solos: bool) -> Engine {
    let mut store = ParamStore::new();
    let text = std::fs::read_to_string(PUBLIC_SNAP).unwrap();
    Snapshot::load(&store, &text).unwrap();
    let mut logic = MixLogic::new(&store);
    store.add_hook(logic.change_hook());
    for n in 1..=40 {
        sett(&store, &format!("ch/{n}/in/conn/grp"), "USB");
        sett(&store, &format!("ch/{n}/in/conn/in"), &n.to_string());
        sett(&store, &format!("ch/{n}/in/set/altsrc"), "0");
        sett(&store, &format!("ch/{n}/in/set/srcauto"), "0");
    }
    for (k, v) in [
        ("lvl", "0"),
        ("eq/on", "1"),
        ("eq/lsg", "3"),
        ("eq/2g", "-4"),
        ("eq/hsg", "2"),
        ("dly/on", "1"),
        ("dly/m", "5"),
        ("lim", "-10"),
    ] {
        sett(&store, &format!("cfg/mon/1/{k}"), v);
    }
    if solos {
        sett(&store, "$ctl/cfg/soloexcl", "0");
        sett(&store, "cfg/solo/mon", "PH+SPK");
        for n in 1..=8 {
            sett(&store, &format!("ch/{n}/mon"), "A+B");
            sett(&store, &format!("ch/{n}/$solo"), "1");
        }
        sett(&store, "bus/1/$solo", "1");
        sett(&store, "bus/5/$solo", "1");
    }
    logic.refresh(&store);
    let mut routing = RoutingControl::new(&store);
    let cfg = MixConfig {
        logic: Some(logic.shared()),
        ..MixConfig::default()
    };
    let (graph, map, _) = build_mix_graph(&store, &mut routing, cfg)
        .unwrap()
        .finish()
        .unwrap();
    let mut e = Engine::new(
        graph,
        EngineConfig {
            workers: 0,
            ..EngineConfig::default()
        },
    );
    let mut pipe = ParamPipeline::new(&store, map, PipelineConfig::default()).unwrap();
    pipe.sync_all(&store, &mut e);
    for ch in 0..48 {
        for (i, s) in e.external_mut(ch).iter_mut().enumerate() {
            *s = (i as f32 * 0.01 * (ch + 1) as f32).sin() * 0.3;
        }
    }
    for _ in 0..50 {
        e.process_cycle_serial();
    }
    e
}

fn bench(c: &mut Criterion) {
    for (name, solos) in [("no_solo", false), ("10_solos", true)] {
        let mut e = engine(solos);
        c.bench_function(&format!("monitor/public_{name}_serial"), |b| {
            b.iter(|| e.process_cycle_serial())
        });
    }
}

criterion_group!(benches, bench);
criterion_main!(benches);
