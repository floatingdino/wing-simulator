//! Criterion benches for the T04A source extractor and PSE/LA combo (stereo, 64-sample blocks, 48 kHz).

use criterion::{black_box, criterion_group, criterion_main, Criterion, Throughput};
use wing_blocks::model::{Model, ParamValue};
use wing_blocks::models::pse::{PseKind, SourceExtractor};

const FS: f64 = 48_000.0;
const BLOCK: usize = 64;

fn input(n: usize, phase: f32) -> Vec<f32> {
    (0..n)
        .map(|i| (i as f32 * 0.031 + phase).sin() * 0.05)
        .collect()
}

fn bench_pse(c: &mut Criterion) {
    let mut g = c.benchmark_group("pse");
    g.throughput(Throughput::Elements(BLOCK as u64));
    for (name, kind) in [
        ("pse_stereo_64", PseKind::Pse),
        ("cmb_stereo_64", PseKind::Combo),
    ] {
        let mut m = SourceExtractor::new(kind);
        m.prepare(FS, BLOCK);
        m.set_param(1, ParamValue::Float(-10.0));
        m.set_param(6, ParamValue::Float(60.0));
        m.commit();
        let mut l = input(BLOCK, 0.0);
        let mut r = input(BLOCK, 1.0);
        g.bench_function(name, |b| {
            b.iter(|| m.process(black_box(&mut l), black_box(&mut r), None))
        });
    }
    g.finish();
}

criterion_group!(benches, bench_pse);
criterion_main!(benches);
