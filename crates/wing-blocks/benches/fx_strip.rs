//! Criterion bench for the T05C channel-strip FX (stereo, 64-sample blocks, 48 kHz, every stage switched in).

use criterion::{black_box, criterion_group, criterion_main, Criterion, Throughput};
use wing_blocks::model::{create_model, BlockKind, ParamValue};
use wing_blocks::models::fx_strip::{Route, StripKind};

const FS: f64 = 48_000.0;
const BLOCK: usize = 64;

fn bench(c: &mut Criterion) {
    let mut g = c.benchmark_group("fx_strip");
    g.throughput(Throughput::Elements(BLOCK as u64));
    for kind in StripKind::ALL {
        let mut m = create_model(BlockKind::Fx, kind.id());
        m.prepare(FS, BLOCK);
        m.load_defaults();
        for (i, r) in kind.routes().iter().enumerate() {
            if matches!(r, Route::On(_)) {
                m.set_param(i as u16 + 1, ParamValue::Int(1));
            }
        }
        m.commit();
        let mut l: Vec<f32> = (0..BLOCK).map(|i| (i as f32 * 0.03).sin() * 0.5).collect();
        let mut r = l.clone();
        for _ in 0..50 {
            m.process(&mut l, &mut r, None); // settle the stage switches
        }
        let name = format!("{}_stereo_64", kind.id().trim_matches('*').to_lowercase());
        g.bench_function(name, |b| {
            b.iter(|| m.process(black_box(&mut l), black_box(&mut r), None))
        });
    }
    let mut ext = create_model(BlockKind::Fx, "EXT");
    ext.prepare(FS, BLOCK);
    ext.set_param(1, ParamValue::Enum(7));
    let ret: Vec<f32> = (0..BLOCK).map(|i| (i as f32 * 0.05).sin() * 0.5).collect();
    let mut l = ret.clone();
    let mut r = ret.clone();
    g.bench_function("ext_stereo_64", |b| {
        b.iter(|| ext.process(black_box(&mut l), black_box(&mut r), Some((&ret, &ret))))
    });
    g.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
