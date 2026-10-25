//! Criterion benches for the T04D FET-style compressors (stereo-linked, 2x oversampled, 64-sample blocks,
//! 48 kHz), compressing (~10 dB GR) with colouration on.

use criterion::{black_box, criterion_group, criterion_main, Criterion, Throughput};
use wing_blocks::model::{Model, ParamValue};
use wing_blocks::models::comp_fet::{FetComp, FetKind};

const FS: f64 = 48_000.0;
const BLOCK: usize = 64;

fn input(n: usize, phase: f32) -> Vec<f32> {
    (0..n)
        .map(|i| (i as f32 * 0.031 + phase).sin() * 0.5)
        .collect()
}

fn bench_fet(c: &mut Criterion) {
    let mut g = c.benchmark_group("comp_fet");
    g.throughput(Throughput::Elements(BLOCK as u64));
    for kind in FetKind::ALL {
        for (label, ratio) in [("ratio2", 1u32), ("ratio_last", 4)] {
            let mut m = FetComp::new(kind);
            m.prepare(FS, BLOCK);
            m.set_param(5, ParamValue::Enum(ratio));
            m.commit();
            let mut l = input(BLOCK, 0.0);
            let mut r = input(BLOCK, 1.0);
            g.bench_function(format!("{}_{label}_stereo_os2x_64", kind.id()), |b| {
                b.iter(|| m.process(black_box(&mut l), black_box(&mut r), None))
            });
        }
    }
    g.finish();
}

criterion_group!(benches, bench_fet);
criterion_main!(benches);
