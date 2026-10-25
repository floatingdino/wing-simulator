//! Per-cycle overhead of the control->RT parameter pipeline (`ParamPipeline::apply`) with 0, 10,
//! 1000 and all (~1480, two budget-limited cycles) bound parameters changed, dispatched into 40 `ParamStrip`s (input, filter,
//! gate slot, dynamics slot, fader). Changes are real, distinct, bound parameters (the queue
//! coalesces repeated changes of one parameter), alternating between two values per iteration.

use std::hint::black_box;
use std::time::Duration;

use criterion::{criterion_group, BatchSize, BenchmarkId, Criterion};
use wing_engine::params::strip::{bind_channel, ParamStrip};
use wing_engine::params::{ParamMapBuilder, ParamPipeline, PipelineConfig, ProcessorAccess};
use wing_engine::{NodeId, Processor};
use wing_model::{NodeId as ParamId, ParamStore};

const CHANNELS: usize = 40;

struct Strips(Vec<ParamStrip>);

impl ProcessorAccess for Strips {
    fn processor(&mut self, id: NodeId) -> &mut dyn Processor {
        &mut self.0[id.index()]
    }
}

fn bench(c: &mut Criterion) {
    let store = ParamStore::new();
    let mut b = ParamMapBuilder::new();
    for ch in 0..CHANNELS {
        bind_channel(&mut b, ch as u32 + 1, NodeId(ch as u32)).unwrap();
    }
    let map = b.build();
    let mut strips = Strips(
        (0..CHANNELS)
            .map(|_| ParamStrip::new(48_000.0, 64))
            .collect(),
    );
    let mut pipe = ParamPipeline::new(&store, map, PipelineConfig::default()).unwrap();
    pipe.sync_all(&store, &mut strips);
    // every active, non-selector, numeric bound parameter
    let params: Vec<ParamId> = store
        .schema()
        .ids()
        .filter(|&id| {
            matches!(pipe.map().binding_of(id), Some((_, _, Some(_))))
                && store.desc(id).is_some_and(|d| !d.unused_slot)
        })
        .collect();
    println!("bindable params: {}", params.len());

    let mut grp = c.benchmark_group("param_apply_40ch");
    grp.measurement_time(Duration::from_secs(2));
    for &n in &[0usize, 10, 1000, params.len()] {
        let mut flip = false;
        let budget = PipelineConfig::default().budget;
        grp.bench_with_input(BenchmarkId::from_parameter(n), &n, |bch, &n| {
            bch.iter_batched(
                || {
                    flip = !flip;
                    let p = if flip { 0.3 } else { 0.6 };
                    for &id in params.iter().cycle().take(n) {
                        store.set_norm(id, p, 0).unwrap();
                    }
                },
                |_| {
                    // drain fully (more than `budget` changes take several cycles)
                    let mut r = pipe.apply(&mut strips);
                    let mut cycles = 1;
                    while r.saturated {
                        r = pipe.apply(&mut strips);
                        cycles += 1;
                    }
                    black_box((r, cycles, budget))
                },
                BatchSize::PerIteration,
            )
        });
    }
    grp.finish();

    // One budget-limited cycle in isolation (the worst-case per-cycle cost at the default budget).
    let mut grp = c.benchmark_group("param_apply_one_cycle");
    grp.measurement_time(Duration::from_secs(2));
    let budget = PipelineConfig::default().budget;
    let mut flip = false;
    grp.bench_function(BenchmarkId::new("budget", budget), |bch| {
        bch.iter_batched(
            || {
                flip = !flip;
                let p = if flip { 0.3 } else { 0.6 };
                for &id in params.iter().take(budget) {
                    store.set_norm(id, p, 0).unwrap();
                }
            },
            |_| black_box(pipe.apply(&mut strips)),
            BatchSize::PerIteration,
        )
    });
    grp.finish();
}

criterion_group!(benches, bench);

fn main() {
    benches();
    Criterion::default().configure_from_args().final_summary();
}
