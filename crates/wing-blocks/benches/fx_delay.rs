//! Criterion bench for the T050 proof FX model `ST-DL` (stereo, 64-sample blocks, 48 kHz), in ST and X
//! (cross-feedback) mode with feedback on.

use criterion::{black_box, criterion_group, criterion_main, Criterion, Throughput};
use wing_blocks::model::{Model, ParamValue};
use wing_blocks::models::fx_delay::{slot, StereoDelay};

const FS: f64 = 48_000.0;
const BLOCK: usize = 64;

fn bench(c: &mut Criterion) {
    let mut g = c.benchmark_group("fx_delay");
    g.throughput(Throughput::Elements(BLOCK as u64));
    for (label, mode) in [("st", 0u32), ("x", 1)] {
        let mut m = StereoDelay::new();
        m.prepare(FS, BLOCK);
        m.set_param(slot::MODE, ParamValue::Enum(mode));
        m.set_param(slot::FEED, ParamValue::Float(60.0));
        m.commit();
        let mut l: Vec<f32> = (0..BLOCK).map(|i| (i as f32 * 0.03).sin() * 0.5).collect();
        let mut r = l.clone();
        g.bench_function(format!("st_dl_{label}_stereo_64"), |b| {
            b.iter(|| m.process(black_box(&mut l), black_box(&mut r), None))
        });
    }
    g.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
