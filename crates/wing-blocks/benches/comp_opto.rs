//! Criterion benches for the T04E opto / vari-mu / class-A compressors (stereo-linked, 64-sample blocks,
//! 48 kHz, tube colouration on where the model has it).

use criterion::{black_box, criterion_group, criterion_main, Criterion, Throughput};
use wing_blocks::model::{Model, ParamValue};
use wing_blocks::models::comp_opto::{OptoComp, OptoKind};

const FS: f64 = 48_000.0;
const BLOCK: usize = 64;

fn input(n: usize, phase: f32) -> Vec<f32> {
    (0..n)
        .map(|i| (i as f32 * 0.031 + phase).sin() * 0.8)
        .collect()
}

fn bench_opto(c: &mut Criterion) {
    let mut g = c.benchmark_group("comp_opto");
    g.throughput(Throughput::Elements(BLOCK as u64));
    for kind in OptoKind::ALL {
        let mut m = OptoComp::new(kind);
        m.prepare(FS, BLOCK);
        let mut l = input(BLOCK, 0.0);
        let mut r = input(BLOCK, 1.0);
        g.bench_function(format!("{}_stereo_1x64", kind.id()), |b| {
            b.iter(|| m.process(black_box(&mut l), black_box(&mut r), None))
        });
    }
    // F670 programme-dependent position 6 (three parallel RC stages).
    let mut m = OptoComp::new(OptoKind::F670);
    m.prepare(FS, BLOCK);
    m.set_param(3, ParamValue::Int(6));
    m.commit();
    let mut l = input(BLOCK, 0.0);
    let mut r = input(BLOCK, 1.0);
    g.bench_function("F670_time6_stereo_1x64", |b| {
        b.iter(|| m.process(black_box(&mut l), black_box(&mut r), None))
    });
    g.finish();
}

criterion_group!(benches, bench_opto);
criterion_main!(benches);
