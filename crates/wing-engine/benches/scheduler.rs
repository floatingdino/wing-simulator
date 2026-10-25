//! Scheduler benchmark: 48 dummy strips + 16 bus sums + 16 dummy FX + main sum, 64-sample
//! blocks. Compares the raw serial loop (no scheduler) with the engine at 0..N workers, and
//! prints an overhead summary (serial overhead and parallel overhead vs. the ideal schedule).

use std::hint::black_box;
use std::time::{Duration, Instant};

use criterion::{criterion_group, BenchmarkId, Criterion};
use wing_engine::dummy::{self, DummyWork, Sum};
use wing_engine::{Engine, EngineConfig, GraphBuilder, Port};

const STRIPS: usize = 48;
const FX: usize = 16;
const STRIP_STAGES: usize = 6;
const FX_STAGES: usize = 12;
const BLOCK: usize = 64;

fn cfg(workers: usize) -> EngineConfig {
    EngineConfig {
        workers,
        rt_workers: true,
        ..EngineConfig::default()
    }
}

fn engine(workers: usize) -> Engine {
    let (g, _) = dummy::scene(STRIPS, FX, STRIP_STAGES, FX_STAGES, BLOCK);
    let mut e = Engine::new(g, cfg(workers));
    for ch in 0..STRIPS {
        for (i, s) in e.external_mut(ch).iter_mut().enumerate() {
            *s = (i as f32 * 0.05 * (ch + 1) as f32).sin();
        }
    }
    e
}

const WORKER_COUNTS: [usize; 5] = [1, 3, 5, 7, 11];

fn bench(c: &mut Criterion) {
    let mut grp = c.benchmark_group("scene_48s_16fx_64");
    grp.measurement_time(Duration::from_secs(3));
    let mut e = engine(0);
    grp.bench_function("serial_raw", |b| b.iter(|| e.process_cycle_serial()));
    grp.bench_function("engine_0w", |b| b.iter(|| black_box(e.process_cycle())));
    drop(e);
    for &w in &WORKER_COUNTS {
        let mut e = engine(w);
        grp.bench_with_input(BenchmarkId::new("engine_hot", w), &w, |b, _| {
            b.iter(|| black_box(e.process_cycle()))
        });
        if w == 7 {
            // Paced like a real device (1.33 ms period): workers park between cycles, so this
            // includes wake-up latency. Only the cycle itself is timed.
            grp.bench_with_input(BenchmarkId::new("engine_paced", w), &w, |b, _| {
                b.iter_custom(|iters| {
                    let mut total = Duration::ZERO;
                    for _ in 0..iters {
                        let next = Instant::now() + Duration::from_micros(1333);
                        total += e.process_cycle();
                        while Instant::now() < next {
                            std::thread::sleep(Duration::from_micros(200));
                        }
                    }
                    total
                })
            });
        }
    }
    grp.finish();
}

/// Median of `n` timings of `f`.
fn median(n: usize, mut f: impl FnMut() -> Duration) -> f64 {
    let mut v: Vec<f64> = (0..n).map(|_| f().as_nanos() as f64).collect();
    v.sort_by(|a, b| a.partial_cmp(b).unwrap());
    v[n / 2]
}

/// Serial per-node cost of a one-node graph.
fn node_cost(n_inputs: usize, proc: Box<dyn wing_engine::Processor>) -> f64 {
    let mut g = GraphBuilder::new(BLOCK, 48_000.0, 1);
    let ins: Vec<Port> = (0..n_inputs).map(|_| Port::External(0)).collect();
    g.add_boxed("n", 2, &ins, proc).unwrap();
    let mut e = Engine::new(g.build().unwrap(), cfg(0));
    e.external_mut(0).fill(0.3);
    for _ in 0..1000 {
        e.process_cycle_serial();
    }
    median(20_001, || {
        let t = Instant::now();
        e.process_cycle_serial();
        t.elapsed()
    })
}

fn summary() {
    let strip = node_cost(1, Box::new(DummyWork::new(STRIP_STAGES, 2)));
    let bus = node_cost(STRIPS * 2, Box::new(Sum { gain: 0.1 }));
    let fx = node_cost(2, Box::new(DummyWork::new(FX_STAGES, 2)));
    let main = node_cost(FX * 2, Box::new(Sum { gain: 1.0 }));
    let work = STRIPS as f64 * strip + FX as f64 * (bus + fx) + main;
    println!("\n== T022 scheduler summary (medians, ns) ==");
    println!("node cost: strip {strip:.0}  bus-sum {bus:.0}  fx {fx:.0}  main {main:.0}  => sum {work:.0}");

    let mut e = engine(0);
    for _ in 0..2000 {
        e.process_cycle_serial();
        e.process_cycle();
    }
    let raw = median(20_001, || {
        let t = Instant::now();
        e.process_cycle_serial();
        t.elapsed()
    });
    let eng0 = median(20_001, || e.process_cycle());
    println!(
        "serial raw {raw:.0}  engine 0 workers {eng0:.0}  scheduler overhead {:+.2}%",
        (eng0 - raw) / raw * 100.0
    );
    drop(e);
    for &w in &WORKER_COUNTS {
        let p = (w + 1) as f64;
        let ceil = |n: usize| (n as f64 / p).ceil();
        // Ideal level-synchronous schedule with zero scheduling cost.
        let ideal = ceil(STRIPS) * strip + ceil(FX) * bus + ceil(FX) * fx + main;
        let mut e = engine(w);
        for _ in 0..2000 {
            e.process_cycle();
        }
        let hot = median(20_001, || e.process_cycle());
        let paced = median(1_001, || {
            let next = Instant::now() + Duration::from_micros(1333);
            let d = e.process_cycle();
            while Instant::now() < next {
                std::thread::sleep(Duration::from_micros(200));
            }
            d
        });
        println!(
            "{w:2} workers: hot {hot:6.0} (speedup {:4.2}x, ideal {ideal:5.0}, overhead vs ideal {:+5.1}% of serial work)  paced {paced:6.0} ({:+5.1}%)  load {:.1}% of 1333us",
            raw / hot,
            (hot - ideal) / raw * 100.0,
            (paced - ideal) / raw * 100.0,
            paced / 1_333_333.0 * 100.0,
        );
    }
}

criterion_group!(benches, bench);

fn main() {
    benches();
    summary();
    Criterion::default().configure_from_args().final_summary();
}
