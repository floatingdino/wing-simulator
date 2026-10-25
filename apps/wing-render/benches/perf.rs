//! T070 perf suite as criterion benches: one console cycle (the daemon's `Console::cycle`, meters
//! on, noise on all 48 USB inputs) of the worst-case scenes at 64 / 128-sample blocks, serial and
//! with the real scheduler, then a paced (one cycle per period, RT policy) p99 / p99.9 summary.
//! The full report with worker scaling is `wing-render --bench` (docs/perf.md).

use std::hint::black_box;
use std::time::Duration;

use criterion::{criterion_group, BenchmarkId, Criterion};
use wing_render::perf::{console, run_paced, timed_cycle, Noise, Scene};

fn bench(c: &mut Criterion) {
    for (scene, block, workers) in [
        (Scene::Everything, 64, &[0usize, 7, 10][..]),
        (Scene::Everything, 128, &[7][..]),
        (Scene::Public, 64, &[0, 7][..]),
    ] {
        let mut g = c.benchmark_group(format!("cycle_{}_{block}", scene.name()));
        g.measurement_time(Duration::from_secs(3));
        g.sample_size(30);
        for &w in workers {
            let mut store = scene.store().expect("scene");
            let mut con = console(&mut store, block, w, false).expect("console");
            let mut noise = Noise::default();
            g.bench_with_input(BenchmarkId::new("workers", w), &w, |b, _| {
                b.iter(|| black_box(timed_cycle(&mut con, &mut noise)))
            });
        }
        g.finish();
    }
}

fn summary() {
    println!("\n== T070 paced summary (48 kHz / 64, RT policy, 2000 cycles) ==");
    for scene in [Scene::Everything, Scene::Public] {
        for w in [7, 10] {
            let s = run_paced(scene, 64, w, 200, 2000).expect("paced run");
            println!(
                "{:<10} {w:2} workers: mean {:6.1} µs  p99 {:6.1} µs  p99.9 {:6.1} µs  max {:6.1} µs  = {:4.1}% / {:4.1}% / {:4.1}% of the period",
                scene.name(),
                s.mean,
                s.p99,
                s.p999,
                s.max,
                s.pct(s.mean),
                s.pct(s.p99),
                s.pct(s.p999),
            );
        }
    }
}

criterion_group!(benches, bench);

fn main() {
    benches();
    summary();
    Criterion::default().configure_from_args().final_summary();
}
