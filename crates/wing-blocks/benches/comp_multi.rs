//! Criterion benches for the T04F one-knob and three-band compressors (stereo, 64-sample blocks, 48 kHz),
//! compressing.

use criterion::{black_box, criterion_group, criterion_main, Criterion, Throughput};
use wing_blocks::model::{Model, ParamValue};
use wing_blocks::models::comp_multi::{OneKnobComp, ThreeBandComp};

const FS: f64 = 48_000.0;
const BLOCK: usize = 64;

fn input(n: usize, phase: f32) -> Vec<f32> {
    (0..n)
        .map(|i| (i as f32 * 0.031 + phase).sin() * 0.5)
        .collect()
}

fn bench_multi(c: &mut Criterion) {
    let mut g = c.benchmark_group("comp_multi");
    g.throughput(Throughput::Elements(BLOCK as u64));
    let mut one = OneKnobComp::new();
    one.prepare(FS, BLOCK);
    one.set_param(1, ParamValue::Float(7.0));
    one.commit();
    let mut tri = ThreeBandComp::new();
    tri.prepare(FS, BLOCK);
    for b in 0..3u16 {
        tri.set_param(5 + 5 * b, ParamValue::Float(-30.0));
    }
    tri.commit();
    let mut l = input(BLOCK, 0.0);
    let mut r = input(BLOCK, 1.0);
    g.bench_function("onec_stereo_64", |b| {
        b.iter(|| one.process(black_box(&mut l), black_box(&mut r), None))
    });
    g.bench_function("3cmp_stereo_64", |b| {
        b.iter(|| tri.process(black_box(&mut l), black_box(&mut r), None))
    });
    g.finish();
}

criterion_group!(benches, bench_multi);
criterion_main!(benches);
