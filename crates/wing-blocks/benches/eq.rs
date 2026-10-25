//! Criterion benches for the STD EQ (64-sample stereo blocks @ 48 kHz): one channel strip with every band active,
//! 48 channel strips, a bus strip with 48 dB/oct cuts, a strip under continuous automation, and the PRE EQ.

use criterion::{black_box, criterion_group, criterion_main, Criterion, Throughput};
use wing_blocks::eq::{BandParams, BandType, EqParams, EqVariant, PreEq, PreEqParams, StdEq};

const FS: f64 = 48_000.0;
const BLOCK: usize = 64;

fn input(n: usize) -> Vec<f32> {
    (0..n).map(|i| (i as f32 * 0.031).sin() * 0.5).collect()
}

fn busy_params(variant: EqVariant) -> EqParams {
    let mut p = EqParams::defaults(variant);
    p.on = true;
    p.low.gain_db = 4.0;
    p.high.gain_db = -3.0;
    for (i, b) in p.bands.iter_mut().enumerate() {
        b.gain_db = if i % 2 == 0 { 3.0 } else { -4.5 };
        b.q = 2.0;
    }
    if variant == EqVariant::Bus {
        p.low_type = BandType::Lr48;
        p.high_type = BandType::Bw24;
        p.tilt_db = 2.0;
    }
    p
}

fn strip(variant: EqVariant) -> StdEq {
    let mut eq = StdEq::new(variant);
    eq.set_params(&busy_params(variant));
    eq.prepare(FS, BLOCK);
    eq
}

fn bench_eq(c: &mut Criterion) {
    let mut g = c.benchmark_group("eq");
    g.throughput(Throughput::Elements(BLOCK as u64));
    let mut l = input(BLOCK);
    let mut r = input(BLOCK);

    let mut ch = strip(EqVariant::Channel);
    g.bench_function("std_channel_all_bands_64", |b| {
        b.iter(|| ch.process(black_box(&mut l), black_box(&mut r)))
    });

    let mut bus = strip(EqVariant::Bus);
    g.bench_function("std_bus_8band_lr48_tilt_64", |b| {
        b.iter(|| bus.process(black_box(&mut l), black_box(&mut r)))
    });

    let mut auto = strip(EqVariant::Channel);
    let mut k = 0u32;
    g.bench_function("std_channel_automated_64", |b| {
        b.iter(|| {
            k = k.wrapping_add(1);
            let mut p = *auto.params();
            p.bands[1] = BandParams::new(
                (k % 30) as f32 - 15.0,
                200.0 + (k % 97) as f32 * 50.0,
                0.5 + (k % 9) as f32,
            );
            auto.set_params(&p);
            auto.process(black_box(&mut l), black_box(&mut r))
        })
    });

    let mut pre = PreEq::new();
    let mut pp = PreEqParams {
        on: true,
        ..Default::default()
    };
    for b in &mut pp.bands {
        b.gain_db = 3.0;
    }
    pre.set_params(&pp);
    pre.prepare(FS, BLOCK);
    g.bench_function("pre_eq_3band_64", |b| {
        b.iter(|| pre.process(black_box(&mut l), black_box(&mut r)))
    });
    g.finish();

    let mut g = c.benchmark_group("eq_48_strips");
    g.throughput(Throughput::Elements((BLOCK * 48) as u64));
    let mut strips: Vec<StdEq> = (0..48).map(|_| strip(EqVariant::Channel)).collect();
    let mut bufs: Vec<(Vec<f32>, Vec<f32>)> =
        (0..48).map(|_| (input(BLOCK), input(BLOCK))).collect();
    g.bench_function("std_channel_x48_64", |b| {
        b.iter(|| {
            for (eq, (l, r)) in strips.iter_mut().zip(bufs.iter_mut()) {
                eq.process(black_box(l), black_box(r));
            }
        })
    });
    g.finish();
}

criterion_group!(benches, bench_eq);
criterion_main!(benches);
