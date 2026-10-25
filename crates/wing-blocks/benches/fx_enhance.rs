//! Criterion bench for the T05E enhancer / bass / imaging / tape FX models (stereo, 64-sample blocks, 48 kHz).

use criterion::{black_box, criterion_group, criterion_main, Criterion, Throughput};
use wing_blocks::model::{create_model, BlockKind, ParamValue};

const FS: f64 = 48_000.0;
const BLOCK: usize = 64;

fn bench(c: &mut Criterion) {
    let mut g = c.benchmark_group("fx_enhance");
    g.throughput(Throughput::Elements(BLOCK as u64));
    // (id, [(slot, value)]) with the enhancing parameters turned up
    let cases: [(&'static str, &[(u16, f32)]); 6] = [
        ("ENHANCE", &[(6, 60.0), (7, 60.0), (8, 60.0)]),
        ("EXCITER", &[(5, 80.0), (6, 60.0)]),
        ("P-BASS", &[(1, 0.0)]),
        ("SUB-M", &[(1, 80.0)]),
        ("V-IMG", &[(1, 0.5), (4, 1.0)]),
        ("TAPE", &[(1, 12.0), (3, 1.0), (4, 1.0)]),
    ];
    for (id, params) in cases {
        let mut m = create_model(BlockKind::Fx, id);
        m.prepare(FS, BLOCK);
        for &(s, v) in params {
            m.set_param(s, ParamValue::Float(v));
        }
        m.commit();
        let mut l: Vec<f32> = (0..BLOCK)
            .map(|i| (i as f32 * 0.05).sin() * 0.5 + (i as f32 * 0.9).sin() * 0.1)
            .collect();
        let mut r: Vec<f32> = l.iter().map(|v| v * 0.8).collect();
        g.bench_function(format!("{}_stereo_64", id.to_lowercase()), |b| {
            b.iter(|| m.process(black_box(&mut l), black_box(&mut r), None))
        });
    }
    g.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
