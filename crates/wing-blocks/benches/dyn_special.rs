//! Criterion benches for the T049 special dynamics models (WAVE, RIDE, WARM): stereo, 64-sample blocks,
//! 48 kHz, with the effect engaged (`WARM` at high drive and colour).

use criterion::{black_box, criterion_group, criterion_main, Criterion, Throughput};
use wing_blocks::model::{Model, ParamValue};
use wing_blocks::models::dyn_special::{AutoRider, TransientDesigner, WarmthPre};

const FS: f64 = 48_000.0;
const BLOCK: usize = 64;

fn input(n: usize, phase: f32) -> Vec<f32> {
    (0..n)
        .map(|i| (i as f32 * 0.031 + phase).sin() * 0.5)
        .collect()
}

fn run(c: &mut Criterion, name: &str, m: &mut dyn Model) {
    let mut g = c.benchmark_group("dyn_special");
    g.throughput(Throughput::Elements(BLOCK as u64));
    m.prepare(FS, BLOCK);
    m.commit();
    let mut l = input(BLOCK, 0.0);
    let mut r = input(BLOCK, 1.0);
    g.bench_function(format!("{name}_stereo_64"), |b| {
        b.iter(|| m.process(black_box(&mut l), black_box(&mut r), None))
    });
    g.finish();
}

fn bench(c: &mut Criterion) {
    let mut w = TransientDesigner::new();
    w.set_param(1, ParamValue::Float(10.0));
    w.set_param(2, ParamValue::Float(-12.0));
    run(c, "wave", &mut w);
    let mut r = AutoRider::new();
    run(c, "ride", &mut r);
    let mut p = WarmthPre::new();
    p.set_param(1, ParamValue::Float(100.0));
    p.set_param(2, ParamValue::Float(-50.0));
    p.set_param(3, ParamValue::Float(0.5));
    run(c, "warm", &mut p);
}

criterion_group!(benches, bench);
criterion_main!(benches);
