//! Criterion benches for the alternate EQ models (T04G/T04H): one 64-sample stereo block @ 48 kHz per model
//! with static parameters, plus each model under continuous knob automation (redesign every 16 samples).

use criterion::{black_box, criterion_group, criterion_main, Criterion, Throughput};
use wing_blocks::model::{Model, ParamValue};
use wing_blocks::models::eq_alt::{AltEq, AltKind};

const FS: f64 = 48_000.0;
const BLOCK: usize = 64;

fn model(kind: AltKind) -> AltEq {
    let mut m = AltEq::new(kind);
    m.prepare(FS, BLOCK);
    // Every continuous slot moved off its default so all sections are active.
    for (i, (_, en)) in kind.slots().iter().enumerate() {
        if en.is_none() {
            m.set_param(i as u16 + 1, ParamValue::Float(3.0));
        }
    }
    m.reset();
    m
}

fn bench(c: &mut Criterion) {
    let mut g = c.benchmark_group("eq_alt");
    g.throughput(Throughput::Elements(BLOCK as u64));
    let input: Vec<f32> = (0..BLOCK).map(|i| (i as f32 * 0.031).sin() * 0.5).collect();
    for kind in AltKind::ALL {
        let mut m = model(kind);
        let (mut l, mut r) = (input.clone(), input.clone());
        g.bench_function(format!("{}/static", kind.id()), |b| {
            b.iter(|| {
                l.copy_from_slice(&input);
                r.copy_from_slice(&input);
                m.process(black_box(&mut l), black_box(&mut r), None);
            })
        });
        let mut m = model(kind);
        let mut k = 0u32;
        g.bench_function(format!("{}/automated", kind.id()), |b| {
            b.iter(|| {
                k = k.wrapping_add(1);
                // Second slot is a gain or knob on every model; sweep it so the glide never settles.
                let v = if k & 1 == 0 { 4.0 } else { -4.0 };
                m.set_param(2, ParamValue::Float(v));
                l.copy_from_slice(&input);
                r.copy_from_slice(&input);
                m.process(black_box(&mut l), black_box(&mut r), None);
            })
        });
    }
    g.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
