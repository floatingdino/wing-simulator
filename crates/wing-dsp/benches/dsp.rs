//! Criterion bench skeleton for wing-dsp. Every DSP block adds a bench here.

use criterion::{black_box, criterion_group, criterion_main, Criterion};

fn bench_placeholder(c: &mut Criterion) {
    let buf = [0.0f32; 64];
    c.bench_function("placeholder_sum_64", |b| {
        b.iter(|| black_box(buf).iter().sum::<f32>())
    });
}

criterion_group!(benches, bench_placeholder);
criterion_main!(benches);
