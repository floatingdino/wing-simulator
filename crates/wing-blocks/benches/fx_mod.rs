//! Criterion bench for the T05M modulation FX models (stereo, 64-sample blocks, 48 kHz, schema defaults;
//! PHASER at 12 stages, MOOD at 24 dB with drive and envelope).

use criterion::{black_box, criterion_group, criterion_main, Criterion, Throughput};
use wing_blocks::model::{create_model, BlockKind, ParamValue};
use wing_blocks::models::fx_mod::{mood_slot, phaser_slot};

const FS: f64 = 48_000.0;
const BLOCK: usize = 64;

fn bench(c: &mut Criterion) {
    let mut g = c.benchmark_group("fx_mod");
    g.throughput(Throughput::Elements(BLOCK as u64));
    for id in [
        "DIMCRS", "CHORUS", "FLANGER", "PHASER", "ROTARY", "PANNER", "MOOD",
    ] {
        let mut m = create_model(BlockKind::Fx, id);
        m.prepare(FS, BLOCK);
        m.load_defaults();
        match id {
            "PHASER" => m.set_param(phaser_slot::STG, ParamValue::Int(12)),
            "MOOD" => {
                m.set_param(mood_slot::DRV, ParamValue::Float(5.0));
                m.set_param(mood_slot::ENV, ParamValue::Float(50.0));
            }
            _ => {}
        }
        m.commit();
        let mut l: Vec<f32> = (0..BLOCK).map(|i| (i as f32 * 0.03).sin() * 0.5).collect();
        let mut r = l.clone();
        g.bench_function(format!("{}_stereo_64", id.to_lowercase()), |b| {
            b.iter(|| m.process(black_box(&mut l), black_box(&mut r), None))
        });
    }
    g.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
