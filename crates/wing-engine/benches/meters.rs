//! T039 metering cost: the per-block primitives ([`abs_max`] over a 64-sample block, one
//! [`LevelBank::peak`]), a channel strip (COMP on) with and without its meter, and the port pass of
//! the daemon's console (98 ports: USB In 1..48, OSC 1/2, USB Out 1..48). Prints the estimated
//! metering overhead per engine cycle for the full console (48 strips + mix + ports).

use std::hint::black_box;
use std::time::Instant;

use criterion::{criterion_group, Criterion};
use wing_engine::channel::{block, ChannelStrip};
use wing_engine::meters::{abs_max, LevelBank, PortMeters, StripMeter};
use wing_engine::params::{ParamMsg, TARGET_BASE};
use wing_engine::{GraphBuilder, Port};

const N: usize = 64;
const FS: f64 = 48_000.0;
const PORTS: usize = 98;

fn signal(n: usize, seed: f32) -> Vec<f32> {
    (0..n).map(|i| (i as f32 * 0.37 + seed).sin() * 0.3).collect()
}

fn strip(meter: Option<StripMeter>) -> ChannelStrip {
    let mut s = ChannelStrip::channel(FS, N);
    let set = |s: &mut ChannelStrip, block, param, value| {
        s.apply(&ParamMsg::Set {
            block,
            param,
            value,
        })
    };
    set(&mut s, block::INPUT, 4, 1.0);
    set(&mut s, block::DYN, TARGET_BASE, -30.0);
    set(&mut s, block::DYN, 0, 1.0);
    s.set_meter(meter);
    s.settle();
    s
}

/// Best of 7 runs of `iters` calls (seconds per call).
fn time_per_iter(iters: u32, mut f: impl FnMut()) -> f64 {
    (0..7)
        .map(|_| {
            let t0 = Instant::now();
            for _ in 0..iters {
                f();
            }
            t0.elapsed().as_secs_f64() / iters as f64
        })
        .fold(f64::MAX, f64::min)
}

fn bench(c: &mut Criterion) {
    let x = signal(N, 0.0);
    let bank = LevelBank::shared();
    c.bench_function("meters/abs_max_64", |b| {
        b.iter(|| black_box(abs_max(black_box(&x))))
    });
    c.bench_function("meters/bank_peak", |b| {
        let mut v = 0.1f32;
        b.iter(|| {
            v = if v > 0.9 { 0.1 } else { v + 0.01 };
            bank.peak(black_box(17), v)
        })
    });

    // strip with vs without meter
    let (l0, r0) = (signal(N, 0.0), signal(N, 1.0));
    let mut plain = strip(None);
    let mut metered = strip(Some(StripMeter::at(bank.clone(), 0)));
    let (mut l, mut r) = (l0.clone(), r0.clone());
    c.bench_function("meters/strip_comp_no_meter", |b| {
        b.iter(|| {
            l.copy_from_slice(&l0);
            r.copy_from_slice(&r0);
            plain.process_stereo(&mut l, &mut r);
        })
    });
    c.bench_function("meters/strip_comp_metered", |b| {
        b.iter(|| {
            l.copy_from_slice(&l0);
            r.copy_from_slice(&r0);
            metered.process_stereo(&mut l, &mut r);
        })
    });

    // port pass: 98 externals
    let mut g = GraphBuilder::new(N, FS as f32, PORTS as u32);
    g.add_node(
        "sink",
        1,
        &[Port::External(0)],
        wing_engine::dummy::Sum { gain: 1.0 },
    )
    .unwrap();
    let mut engine = wing_engine::Engine::new(g.build().unwrap(), Default::default());
    for ch in 0..PORTS {
        engine
            .external_mut(ch)
            .copy_from_slice(&signal(N, ch as f32));
    }
    let pm = PortMeters::new(
        bank.clone(),
        (0..PORTS)
            .map(|k| (Port::External(k as u32), 1000 + k))
            .collect(),
    );
    c.bench_function("meters/ports_98", |b| b.iter(|| pm.capture(&engine)));

    // estimate per console cycle: 48 strips (delta) + mix (96 abs_max + peaks) + ports
    let iters = 200_000;
    let t_plain = time_per_iter(iters, || {
        l.copy_from_slice(&l0);
        r.copy_from_slice(&r0);
        plain.process_stereo(&mut l, &mut r);
    });
    let t_met = time_per_iter(iters, || {
        l.copy_from_slice(&l0);
        r.copy_from_slice(&r0);
        metered.process_stereo(&mut l, &mut r);
    });
    let t_ports = time_per_iter(iters / 10, || pm.capture(&engine));
    let t_abs = time_per_iter(iters * 5, || {
        black_box(abs_max(black_box(&x)));
    });
    let per_cycle = 48.0 * (t_met - t_plain).max(0.0) + 100.0 * t_abs + t_ports;
    println!(
        "metering estimate per engine cycle (48 kHz / {N}): strip +{:.0} ns each, ports {:.2} us, \
         abs_max {:.1} ns -> {:.2} us = {:.2}% of the {:.0} us period",
        (t_met - t_plain) * 1e9,
        t_ports * 1e6,
        t_abs * 1e9,
        per_cycle * 1e6,
        per_cycle * 100.0 * FS / N as f64,
        N as f64 * 1e6 / FS
    );
}

criterion_group!(benches, bench);

fn main() {
    benches();
    Criterion::default().configure_from_args().final_summary();
}
