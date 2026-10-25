//! Criterion bench for the T05A amp / cab models (stereo, 64-sample blocks, 48 kHz). Amps run at schema
//! defaults and at full gain (4x oversampled section, cab on); target < 20 us per block.

use criterion::{black_box, criterion_group, criterion_main, Criterion, Throughput};
use wing_blocks::model::{create_model, BlockKind, ParamValue};

const FS: f64 = 48_000.0;
const BLOCK: usize = 64;

fn bench(c: &mut Criterion) {
    let mut g = c.benchmark_group("fx_amp");
    g.throughput(Throughput::Elements(BLOCK as u64));
    for id in ["RACKAMP", "UKROCK", "ANGEL", "JAZZC", "DELUXE", "BODY"] {
        for high in [false, true] {
            if high && id == "BODY" {
                continue;
            }
            let mut m = create_model(BlockKind::Fx, id);
            m.prepare(FS, BLOCK);
            m.load_defaults();
            if high {
                m.set_param(1, ParamValue::Float(10.0));
            }
            m.reset();
            let mut l: Vec<f32> = (0..BLOCK).map(|i| (i as f32 * 0.03).sin() * 0.5).collect();
            let mut r = l.clone();
            let name = format!(
                "{}{}_stereo_64",
                id.to_lowercase(),
                if high { "_hi" } else { "" }
            );
            g.bench_function(name, |b| {
                b.iter(|| m.process(black_box(&mut l), black_box(&mut r), None))
            });
        }
    }
    g.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
