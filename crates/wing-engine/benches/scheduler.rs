//! Scheduler benchmark: 48 dummy strips + 16 bus sums + 16 dummy FX + main sum, 64-sample
//! blocks. Compares the raw serial loop (no scheduler) with the engine at 0..N workers, and
//! prints an overhead summary (serial overhead and parallel overhead vs. the ideal schedule).
//!
//! Two regimes:
//! * hot: cycles back to back, no RT policy (an RT thread running flat out exceeds its
//!   time-constraint `computation` budget and gets throttled by the kernel, which would distort
//!   the numbers);
//! * paced: one cycle per 1.33 ms (48 kHz / 64), RT policy on driver and workers, workers sleep
//!   between cycles, so wake-up latency is included. This is the production regime.

use std::hint::black_box;
use std::time::{Duration, Instant};

use criterion::{criterion_group, BenchmarkId, Criterion};
use wing_engine::dummy::{self, DummyWork, Sum};
use wing_engine::{rt, Engine, EngineConfig, GraphBuilder, Port, TimeConstraint};

const STRIPS: usize = 48;
const FX: usize = 16;
const STRIP_STAGES: usize = 15;
const FX_STAGES: usize = 24;
const BLOCK: usize = 64;
const PERIOD: Duration = Duration::from_nanos(1_333_333);

fn cfg(workers: usize, rt_workers: bool) -> EngineConfig {
    EngineConfig {
        workers,
        rt_workers,
        ..EngineConfig::default()
    }
}

fn engine(workers: usize, rt_workers: bool) -> Engine {
    let (g, _) = dummy::scene(STRIPS, FX, STRIP_STAGES, FX_STAGES, BLOCK);
    let mut e = Engine::new(g, cfg(workers, rt_workers));
    for ch in 0..STRIPS {
        for (i, s) in e.external_mut(ch).iter_mut().enumerate() {
            *s = (i as f32 * 0.05 * (ch + 1) as f32).sin();
        }
    }
    e
}

const WORKER_COUNTS: [usize; 5] = [1, 3, 5, 7, 11];

/// One paced cycle: run, then idle until the next period boundary.
fn paced_cycle(e: &mut Engine) -> Duration {
    let next = Instant::now() + PERIOD;
    let d = e.process_cycle();
    while Instant::now() < next {
        std::thread::sleep(Duration::from_micros(200));
    }
    d
}

fn bench(c: &mut Criterion) {
    let mut grp = c.benchmark_group("scene_48s_16fx_64");
    grp.measurement_time(Duration::from_secs(3));
    let mut e = engine(0, false);
    grp.bench_function("serial_raw", |b| b.iter(|| e.process_cycle_serial()));
    grp.bench_function("engine_0w", |b| b.iter(|| black_box(e.process_cycle())));
    drop(e);
    for &w in &WORKER_COUNTS {
        let mut e = engine(w, false);
        grp.bench_with_input(BenchmarkId::new("engine_hot", w), &w, |b, _| {
            b.iter(|| black_box(e.process_cycle()))
        });
    }
    let mut e = engine(7, true);
    grp.bench_function(BenchmarkId::new("engine_paced", 7), |b| {
        b.iter_custom(|iters| (0..iters).map(|_| paced_cycle(&mut e)).sum())
    });
    grp.finish();
}

/// Sorted timings (ns) of `n` calls of `f`.
fn sample(n: usize, mut f: impl FnMut() -> Duration) -> Vec<f64> {
    let mut v: Vec<f64> = (0..n).map(|_| f().as_nanos() as f64).collect();
    v.sort_by(|a, b| a.partial_cmp(b).unwrap());
    v
}

fn pctl(v: &[f64], p: f64) -> f64 {
    v[((v.len() - 1) as f64 * p) as usize]
}

/// Serial per-node cost of a one-node graph.
fn node_cost(n_inputs: usize, proc: Box<dyn wing_engine::Processor>) -> f64 {
    let mut g = GraphBuilder::new(BLOCK, 48_000.0, 1);
    let ins: Vec<Port> = (0..n_inputs).map(|_| Port::External(0)).collect();
    g.add_boxed("n", 2, &ins, proc).unwrap();
    let mut e = Engine::new(g.build().unwrap(), cfg(0, false));
    e.external_mut(0).fill(0.3);
    for _ in 0..1000 {
        e.process_cycle_serial();
    }
    let v = sample(20_001, || {
        let t = Instant::now();
        e.process_cycle_serial();
        t.elapsed()
    });
    pctl(&v, 0.5)
}

fn summary() {
    let strip = node_cost(1, Box::new(DummyWork::new(STRIP_STAGES, 2)));
    let bus = node_cost(STRIPS * 2, Box::new(Sum { gain: 0.1 }));
    let fx = node_cost(2, Box::new(DummyWork::new(FX_STAGES, 2)));
    let main = node_cost(FX * 2, Box::new(Sum { gain: 1.0 }));
    let work = STRIPS as f64 * strip + FX as f64 * (bus + fx) + main;
    println!("\n== T022 scheduler summary (ns; medians unless noted) ==");
    println!(
        "node cost: strip {strip:.0}  bus-sum {bus:.0}  fx {fx:.0}  main {main:.0}  => sum {work:.0}"
    );

    let mut e = engine(0, false);
    for _ in 0..2000 {
        e.process_cycle_serial();
        e.process_cycle();
    }
    let raw = pctl(
        &sample(20_001, || {
            let t = Instant::now();
            e.process_cycle_serial();
            t.elapsed()
        }),
        0.5,
    );
    let eng0 = pctl(&sample(20_001, || e.process_cycle()), 0.5);
    println!(
        "serial raw {raw:.0}  engine 0 workers {eng0:.0}  single-thread scheduler overhead {:+.2}%",
        (eng0 - raw) / raw * 100.0
    );
    drop(e);
    for &w in std::iter::once(&0).chain(&WORKER_COUNTS) {
        let p = (w + 1) as f64;
        let ceil = |n: usize| (n as f64 / p).ceil();
        // Ideal level-synchronous schedule with zero scheduling cost.
        let ideal = ceil(STRIPS) * strip + ceil(FX) * bus + ceil(FX) * fx + main;
        let mut e = engine(w, false);
        for _ in 0..2000 {
            e.process_cycle();
        }
        let hot = pctl(&sample(20_001, || e.process_cycle()), 0.5);
        drop(e);
        // Paced run on an RT driver thread with RT workers.
        let (pmed, p99, pmax) = std::thread::spawn(move || {
            let _ = rt::set_current_thread_time_constraint(TimeConstraint::for_period(1_333_333));
            let mut e = engine(w, true);
            for _ in 0..100 {
                paced_cycle(&mut e);
            }
            let v = sample(1_501, || paced_cycle(&mut e));
            (pctl(&v, 0.5), pctl(&v, 0.99), pctl(&v, 1.0))
        })
        .join()
        .unwrap();
        println!(
            "{w:2} workers: hot {hot:6.0} (speedup {:4.2}x, ideal {ideal:6.0}, overhead {:+5.1}% of serial work) | paced med {pmed:6.0} ({:+5.1}%) p99 {p99:6.0} max {pmax:6.0} = {:.1}% of period",
            raw / hot,
            (hot - ideal) / raw * 100.0,
            (pmed - ideal) / raw * 100.0,
            p99 / PERIOD.as_nanos() as f64 * 100.0,
        );
    }
}

criterion_group!(benches, bench);

fn main() {
    benches();
    summary();
    Criterion::default().configure_from_args().final_summary();
}
