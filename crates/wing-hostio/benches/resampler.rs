//! Resampler + full bridge render cost per device callback.

use criterion::{black_box, criterion_group, criterion_main, Criterion};
use wing_hostio::{bridge, BridgeConfig, Resampler, ResamplerConfig};

fn resampler(c: &mut Criterion) {
    let mut g = c.benchmark_group("resampler");
    for &(name, step) in &[("near_unity", 1.0002), ("48k_to_44k1", 48.0 / 44.1)] {
        let mut r = Resampler::new(2, step, ResamplerConfig::default());
        let mut x = 0.0f32;
        g.bench_function(format!("{name}_2ch_512"), |b| {
            b.iter(|| {
                let mut acc = 0.0f32;
                for _ in 0..512 {
                    let o = r.next_frame(step, |f| {
                        x += 0.001;
                        f[0] = x;
                        f[1] = -x;
                    });
                    acc += o[0];
                }
                black_box(acc)
            })
        });
    }
    g.finish();
}

fn bridge_render(c: &mut Criterion) {
    let (mut p, mut cons) = bridge(BridgeConfig::default());
    cons.configure(48_000.0, 256);
    cons.start();
    let blk = [0.1f32; 64];
    let mut t = 1u64;
    c.bench_function("bridge_render_2ch_256", |b| {
        b.iter(|| {
            for _ in 0..4 {
                t += 1_333_333;
                p.push(&[&blk, &blk], t);
            }
            let mut acc = 0.0f32;
            cons.render(256, t, |_, s| acc += s[0]);
            black_box(acc)
        })
    });
}

criterion_group!(benches, resampler, bridge_render);
criterion_main!(benches);
