//! Criterion benches for the standard dynamics block (64-sample blocks, 48 kHz).

use criterion::{black_box, criterion_group, criterion_main, Criterion, Throughput};
use wing_blocks::dynamics::*;

const FS: f64 = 48_000.0;
const BLOCK: usize = 64;

fn input(n: usize, phase: f32) -> Vec<f32> {
    (0..n)
        .map(|i| (i as f32 * 0.031 + phase).sin() * 0.8)
        .collect()
}

fn bench_dynamics(c: &mut Criterion) {
    let mut g = c.benchmark_group("dynamics");
    g.throughput(Throughput::Elements(BLOCK as u64));

    let mut buf = input(BLOCK, 0.0);
    let mut comp = Dynamics::new(DynParams::comp());
    comp.prepare(FS, 1);
    g.bench_function("comp_mono_1x64", |b| {
        b.iter(|| comp.process_mono(black_box(&mut buf), None))
    });

    let mut l = input(BLOCK, 0.0);
    let mut r = input(BLOCK, 1.0);
    let mut keyed = DynParams::comp();
    keyed.key.kind = KeyFilterKind::Hp12;
    let mut st = Dynamics::new(keyed);
    st.prepare(FS, 2);
    g.bench_function("comp_stereo_keyfilt_1x64", |b| {
        b.iter(|| st.process(black_box(&mut [&mut l, &mut r]), None))
    });

    let mut exp = Dynamics::new(DynParams::exp());
    exp.prepare(FS, 1);
    g.bench_function("exp_mono_1x64", |b| {
        b.iter(|| exp.process_mono(black_box(&mut buf), None))
    });
    g.finish();

    let mut g = c.benchmark_group("dynamics_48");
    g.throughput(Throughput::Elements((48 * BLOCK) as u64));
    let mut insts: Vec<Dynamics> = (0..48)
        .map(|i| {
            let mut p = if i % 4 == 3 {
                DynParams::exp()
            } else {
                DynParams::comp()
            };
            p.thr_db = -10.0 - (i % 30) as f32;
            let mut d = Dynamics::new(p);
            d.prepare(FS, 1);
            d
        })
        .collect();
    let mut bufs: Vec<Vec<f32>> = (0..48).map(|i| input(BLOCK, i as f32)).collect();
    g.bench_function("comp_exp_mono_48x64", |b| {
        b.iter(|| {
            for (d, buf) in insts.iter_mut().zip(bufs.iter_mut()) {
                d.process_mono(black_box(buf), None);
            }
        })
    });
    g.finish();
}

criterion_group!(benches, bench_dynamics);
criterion_main!(benches);
