//! Criterion benches for the T048 dynamic EQs and de-esser (stereo, 64-sample blocks, 48 kHz), with signals
//! that keep the dynamic gain moving (worst case: coefficient interpolation every sample).

use criterion::{black_box, criterion_group, criterion_main, Criterion, Throughput};
use wing_blocks::model::{Model, ParamValue};
use wing_blocks::models::dyn_eq::{DeEsser902, DynEq};

const FS: f64 = 48_000.0;
const BLOCK: usize = 64;

fn input(n: usize, phase: f32) -> Vec<f32> {
    (0..n)
        .map(|i| (i as f32 * 0.031 + phase).sin() * 0.8 + (i as f32 * 1.1).sin() * 0.3)
        .collect()
}

fn bench_dyn_eq(c: &mut Criterion) {
    let mut g = c.benchmark_group("dyn_eq");
    g.throughput(Throughput::Elements(BLOCK as u64));
    let mut deq = DynEq::single();
    let mut deq2 = DynEq::dual();
    let mut ds = DeEsser902::new();
    // Active settings: -12 dB band cuts, low threshold, fast times.
    for (m, bands) in [(&mut deq, 1u16), (&mut deq2, 2)] {
        for b in 0..bands {
            m.set_param(9 * b + 1, ParamValue::Float(-50.0));
            m.set_param(9 * b + 3, ParamValue::Float(1.0));
            m.set_param(9 * b + 6, ParamValue::Float(-12.0));
        }
    }
    ds.set_param(2, ParamValue::Float(12.0));
    let models: [(&str, &mut dyn Model); 4] = [
        ("DEQ", &mut deq),
        ("DEQ2", &mut deq2),
        ("DS902", &mut ds),
        ("DS902_full", &mut DeEsser902::new()),
    ];
    for (name, m) in models {
        if name == "DS902_full" {
            m.set_param(2, ParamValue::Float(12.0));
            m.set_param(3, ParamValue::Enum(0));
        }
        m.prepare(FS, BLOCK);
        let mut l = input(BLOCK, 0.0);
        let mut r = input(BLOCK, 1.0);
        g.bench_function(format!("{name}_stereo_1x64"), |b| {
            b.iter(|| m.process(black_box(&mut l), black_box(&mut r), None))
        });
    }
    g.finish();
}

criterion_group!(benches, bench_dyn_eq);
criterion_main!(benches);
