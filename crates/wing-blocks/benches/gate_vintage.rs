//! Criterion benches for the T047 vintage gates (stereo-linked, 64-sample blocks, 48 kHz).

use criterion::{black_box, criterion_group, criterion_main, Criterion, Throughput};
use wing_blocks::model::Model;
use wing_blocks::models::gate_vintage::{VintageGate, VintageGateKind};

const FS: f64 = 48_000.0;
const BLOCK: usize = 64;

fn input(n: usize, phase: f32) -> Vec<f32> {
    (0..n)
        .map(|i| (i as f32 * 0.031 + phase).sin() * 0.05)
        .collect()
}

fn bench_gates(c: &mut Criterion) {
    let mut g = c.benchmark_group("gate_vintage");
    g.throughput(Throughput::Elements(BLOCK as u64));
    for kind in VintageGateKind::ALL {
        let mut m = VintageGate::new(kind);
        m.prepare(FS, BLOCK);
        let mut l = input(BLOCK, 0.0);
        let mut r = input(BLOCK, 1.0);
        g.bench_function(format!("{}_stereo_1x64", kind.id()), |b| {
            b.iter(|| m.process(black_box(&mut l), black_box(&mut r), None))
        });
    }
    g.finish();
}

criterion_group!(benches, bench_gates);
criterion_main!(benches);
