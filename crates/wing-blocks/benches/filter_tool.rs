//! Criterion benches for the filter-tool models (64-sample stereo blocks @ 48 kHz): static, and under
//! continuous parameter automation (coefficient redesign + ramp every block).

use criterion::{black_box, criterion_group, criterion_main, Criterion, Throughput};
use wing_blocks::model::{create_model, BlockKind, ParamValue};

const BLOCK: usize = 64;

fn bench_tools(c: &mut Criterion) {
    let mut g = c.benchmark_group("filter_tool");
    g.throughput(Throughput::Elements(BLOCK as u64));
    let input: Vec<f32> = (0..BLOCK).map(|i| (i as f32 * 0.031).sin() * 0.5).collect();
    for id in ["TILT", "MAX", "AP1", "AP2"] {
        let mut m = create_model(BlockKind::FilterTool, id);
        m.prepare(48_000.0, BLOCK);
        m.set_param(1, ParamValue::Float(if id == "MAX" { 40.0 } else { 3.0 }));
        m.set_param(2, ParamValue::Float(if id == "MAX" { 60.0 } else { 1.5 }));
        m.commit();
        m.reset();
        let (mut l, mut r) = (input.clone(), input.clone());
        g.bench_function(format!("{id}_64"), |b| {
            b.iter(|| m.process(black_box(&mut l), black_box(&mut r), None))
        });
        let mut k = 0u32;
        g.bench_function(format!("{id}_automated_64"), |b| {
            b.iter(|| {
                k = k.wrapping_add(1);
                let v = (k % 100) as f32;
                m.set_param(
                    1,
                    ParamValue::Float(if id == "TILT" {
                        v / 20.0 - 2.5
                    } else {
                        100.0 + v * 50.0
                    }),
                );
                if id == "MAX" {
                    m.set_param(1, ParamValue::Float(v));
                }
                m.process(black_box(&mut l), black_box(&mut r), None)
            })
        });
    }
    g.finish();
}

criterion_group!(benches, bench_tools);
criterion_main!(benches);
