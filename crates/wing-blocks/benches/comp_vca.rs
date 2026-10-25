//! Criterion benches for the T04B VCA compressors (stereo-linked, 64-sample blocks, 48 kHz).

use criterion::{black_box, criterion_group, criterion_main, Criterion, Throughput};
use wing_blocks::model::Model;
use wing_blocks::models::comp_vca::{VcaComp, VcaKind};

const FS: f64 = 48_000.0;
const BLOCK: usize = 64;

fn input(n: usize, phase: f32) -> Vec<f32> {
    (0..n)
        .map(|i| (i as f32 * 0.031 + phase).sin() * 0.8)
        .collect()
}

fn bench_vca(c: &mut Criterion) {
    let mut g = c.benchmark_group("comp_vca");
    g.throughput(Throughput::Elements(BLOCK as u64));
    for kind in VcaKind::ALL {
        let mut m = VcaComp::new(kind);
        m.prepare(FS, BLOCK);
        let mut l = input(BLOCK, 0.0);
        let mut r = input(BLOCK, 1.0);
        g.bench_function(format!("{}_stereo_1x64", kind.id()), |b| {
            b.iter(|| m.process(black_box(&mut l), black_box(&mut r), None))
        });
    }
    g.finish();
}

criterion_group!(benches, bench_vca);
criterion_main!(benches);
