//! Routing stage benchmark: oscillators + input router (48 stereo strips) + USB output router,
//! 64-sample blocks, all inputs on USB (stereo pairs and M/S included), oscillators on pink
//! noise / sine, plus a table swap every cycle.

use criterion::{criterion_group, criterion_main, Criterion};
use wing_engine::routing::{add_input_stage, add_usb_output, RoutingControl, NO_TAPS};
use wing_engine::{Engine, EngineConfig, GraphBuilder};
use wing_model::{ParamStore, Value};

fn set(store: &ParamStore, path: &str, text: &str) {
    store
        .set_text(store.lookup(path).unwrap(), text, 0)
        .unwrap();
}

fn bench(c: &mut Criterion) {
    let store = ParamStore::new();
    for n in 1..=40 {
        set(&store, &format!("ch/{n}/in/conn/grp"), "USB");
        set(&store, &format!("ch/{n}/in/conn/in"), &n.to_string());
    }
    for n in 1..=8 {
        set(&store, &format!("aux/{n}/in/conn/grp"), "USB");
        set(
            &store,
            &format!("aux/{n}/in/conn/in"),
            &(40 + n).to_string(),
        );
    }
    set(&store, "io/in/USB/41/mode", "ST");
    set(&store, "io/in/USB/43/mode", "M/S");
    for n in 1..=48 {
        set(&store, &format!("io/out/USB/{n}/grp"), "USB");
        set(&store, &format!("io/out/USB/{n}/in"), &n.to_string());
    }
    set(&store, "io/out/USB/48/grp", "OSC");
    set(&store, "io/in/OSC/1/osc/mode", "PINK");
    let mut ctl = RoutingControl::new(&store);
    let mut g = GraphBuilder::new(64, 48_000.0, 48);
    let io = add_input_stage(&mut g, &mut ctl).unwrap();
    add_usb_output(&mut g, &mut ctl, io.osc, &NO_TAPS).unwrap();
    let mut e = Engine::new(
        g.build().unwrap(),
        EngineConfig {
            workers: 0,
            rt_workers: false,
            ..EngineConfig::default()
        },
    );
    for ch in 0..48 {
        for (i, s) in e.external_mut(ch).iter_mut().enumerate() {
            *s = (i as f32 * 0.01 * (ch + 1) as f32).sin();
        }
    }
    c.bench_function("routing/cycle_64", |b| b.iter(|| e.process_cycle_serial()));

    let altsw = store.lookup("io/altsw").unwrap();
    let mut on = false;
    c.bench_function("routing/cycle_64_with_swap", |b| {
        b.iter(|| {
            on = !on;
            store.set(altsw, Value::Int(on as i32)).unwrap();
            ctl.rebuild(&store);
            e.process_cycle_serial();
        })
    });
}

criterion_group!(benches, bench);
criterion_main!(benches);
