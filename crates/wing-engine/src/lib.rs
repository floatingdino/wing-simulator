//! Processing graph, routing, busses, scheduling, metering and RT threading.
//!
//! This crate hosts the realtime engine:
//!
//! * [`graph`]: block-processing graph of [`Processor`] nodes with fixed, preallocated planar
//!   `f32` buffers, organised into dependency levels (strips -> bus sums -> FX -> bus/main
//!   strips -> outputs).
//! * [`sched`]: the [`Engine`]: one driver thread (the caller) plus `N` workers that claim nodes
//!   per level with epoch-tagged atomic counters and meet at spin-then-park level barriers. No
//!   rayon, locks or allocation on the audio path.
//! * [`rt`]: macOS time-constraint policy and `os_workgroup` membership.
//! * [`clock`]: [`LiveClock`] (external callback) and [`OfflineClock`] (free-running).
//! * [`stats`]: per-cycle duration, load and xrun counters, readable lock-free.

pub mod clock;
pub mod dummy;
pub mod graph;
pub mod rt;
pub mod sched;
pub mod stats;

pub use clock::{Clock, CycleIo, Inputs, LiveClock, OfflineClock, Outputs};
pub use graph::{
    Graph, GraphBuilder, GraphError, InputList, NodeId, NodeIo, Port, ProcessCtx, Processor,
};
pub use rt::{RawWorkgroup, RtError, TimeConstraint, WorkgroupMembership};
pub use sched::{Engine, EngineConfig};
pub use stats::{StatsHandle, StatsSnapshot};

#[cfg(test)]
mod tests {
    use super::*;
    use std::ops::ControlFlow;
    use std::sync::atomic::{AtomicUsize, Ordering};
    use std::sync::Arc;
    use std::time::Duration;

    fn cfg(workers: usize) -> EngineConfig {
        EngineConfig {
            workers,
            spin: Duration::from_micros(20),
            rt_workers: false,
            workgroup: None,
        }
    }

    /// Tiny deterministic PRNG.
    struct Lcg(u64);
    impl Lcg {
        fn next(&mut self) -> u64 {
            self.0 = self
                .0
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            self.0 >> 33
        }
        fn below(&mut self, n: u64) -> u64 {
            self.next() % n
        }
    }

    /// Random DAG of "stamp checking" nodes. Each node writes `cycle` into sample 0 of every
    /// output and checks that sample 0 of every input already carries the current cycle, i.e.
    /// all its dependencies ran earlier in this cycle. Violations are counted.
    fn stamp_graph(
        seed: u64,
        n: usize,
        violations: Arc<AtomicUsize>,
        runs: Arc<AtomicUsize>,
    ) -> Graph {
        let mut rng = Lcg(seed);
        let mut g = GraphBuilder::new(16, 48_000.0, 4);
        let mut outs: Vec<(NodeId, u32)> = Vec::new();
        for i in 0..n {
            let n_in = rng.below(4) as usize;
            let mut inputs = Vec::new();
            for _ in 0..n_in {
                if outs.is_empty() || rng.below(4) == 0 {
                    inputs.push(Port::External(rng.below(4) as u32));
                } else {
                    let (node, nch) = outs[rng.below(outs.len() as u64) as usize];
                    inputs.push(Port::node(node, rng.below(nch as u64) as u32));
                }
            }
            let n_out = 1 + rng.below(2) as u32;
            let v = violations.clone();
            let r = runs.clone();
            let id = g
                .add_node(
                    format!("n{i}"),
                    n_out,
                    &inputs,
                    move |ctx: &ProcessCtx, io: &mut NodeIo<'_>| {
                        r.fetch_add(1, Ordering::Relaxed);
                        let ins = io.inputs();
                        let mut acc = 0.0f32;
                        for k in 0..ins.len() {
                            let x = ins.get(k);
                            if x[0] != ctx.cycle as f32 {
                                v.fetch_add(1, Ordering::Relaxed);
                            }
                            acc += x[1..].iter().sum::<f32>();
                        }
                        for j in 0..io.num_outputs() {
                            let o = io.output(j);
                            o[0] = ctx.cycle as f32;
                            for (s, y) in o[1..].iter_mut().enumerate() {
                                *y = (acc * 0.5 + (s + j) as f32 * 0.25).sin();
                            }
                        }
                    },
                )
                .unwrap();
            outs.push((id, n_out));
        }
        g.build().unwrap()
    }

    fn stamp_externals(e: &mut Engine) {
        let c = (e.cycle() + 1) as f32;
        for ch in 0..e.graph().num_external() {
            let x = e.external_mut(ch);
            x[0] = c;
            for (i, s) in x[1..].iter_mut().enumerate() {
                *s = ((i as f32) * 0.1 + c).cos();
            }
        }
    }

    #[test]
    fn levels_are_longest_path() {
        let mut g = GraphBuilder::new(8, 48_000.0, 1);
        let nop = |_: &ProcessCtx, _: &mut NodeIo<'_>| {};
        let a = g.add_node("a", 1, &[Port::External(0)], nop).unwrap();
        let b = g.add_node("b", 1, &[], nop).unwrap();
        let c = g.add_node("c", 1, &[Port::node(a, 0)], nop).unwrap();
        let d = g
            .add_node("d", 1, &[Port::node(c, 0), Port::node(b, 0)], nop)
            .unwrap();
        let e = g
            .add_node("e", 1, &[Port::node(a, 0), Port::node(d, 0)], nop)
            .unwrap();
        assert_eq!(
            g.add_node("x", 1, &[Port::node(NodeId(99), 0)], nop),
            Err(GraphError::UnknownNode(NodeId(99)))
        );
        assert_eq!(
            g.add_node("x", 1, &[Port::node(a, 1)], nop),
            Err(GraphError::BadChannel(Port::node(a, 1)))
        );
        assert_eq!(
            g.add_node("x", 1, &[Port::External(1)], nop),
            Err(GraphError::BadChannel(Port::External(1)))
        );
        let g = g.build().unwrap();
        let lv: Vec<u32> = [a, b, c, d, e].iter().map(|&n| g.level_of(n)).collect();
        assert_eq!(lv, vec![0, 0, 1, 2, 3]);
        assert_eq!(g.num_levels(), 4);
        assert_eq!(g.level_range(0), 0..2);
        let sched: Vec<NodeId> = g.schedule().collect();
        assert_eq!(sched, vec![a, b, c, d, e]);
    }

    #[test]
    fn parallel_respects_level_order() {
        for (seed, workers) in [(1u64, 3usize), (2, 7), (3, 1), (4, 0)] {
            let v = Arc::new(AtomicUsize::new(0));
            let r = Arc::new(AtomicUsize::new(0));
            let g = stamp_graph(seed, 120, v.clone(), r.clone());
            let mut e = Engine::new(g, cfg(workers));
            let cycles = 300;
            for _ in 0..cycles {
                stamp_externals(&mut e);
                e.process_cycle();
            }
            assert_eq!(
                v.load(Ordering::Relaxed),
                0,
                "dependency ran late (seed {seed})"
            );
            assert_eq!(r.load(Ordering::Relaxed), 120 * cycles, "node run count");
        }
    }

    #[test]
    fn parallel_matches_serial_bit_exact() {
        let run = |workers: Option<usize>| -> Vec<u32> {
            let v = Arc::new(AtomicUsize::new(0));
            let r = Arc::new(AtomicUsize::new(0));
            let g = stamp_graph(42, 200, v, r);
            let last = g.schedule().last().unwrap();
            let mut e = Engine::new(g, cfg(workers.unwrap_or(0)));
            let mut out = Vec::new();
            for _ in 0..50 {
                stamp_externals(&mut e);
                match workers {
                    Some(_) => {
                        e.process_cycle();
                    }
                    None => e.process_cycle_serial(),
                }
                out.extend(e.port(Port::node(last, 0)).iter().map(|x| x.to_bits()));
            }
            out
        };
        let serial = run(None);
        assert_eq!(run(Some(0)), serial);
        assert_eq!(run(Some(4)), serial);
        assert_eq!(run(Some(11)), serial);
    }

    #[test]
    fn scene_parallel_matches_serial() {
        let render = |workers: Option<usize>| -> Vec<u32> {
            let (g, main) = dummy::scene(48, 16, 4, 4, 64);
            let mut e = Engine::new(g, cfg(workers.unwrap_or(0)));
            let mut out = Vec::new();
            for c in 0..40 {
                for ch in 0..48 {
                    for (i, s) in e.external_mut(ch).iter_mut().enumerate() {
                        *s = ((c * 64 + i) as f32 * 0.01 * (ch + 1) as f32).sin();
                    }
                }
                match workers {
                    Some(_) => {
                        e.process_cycle();
                    }
                    None => e.process_cycle_serial(),
                }
                for ch in 0..2 {
                    out.extend(e.port(Port::node(main, ch)).iter().map(|x| x.to_bits()));
                }
            }
            out
        };
        let serial = render(None);
        assert!(serial.iter().any(|&b| f32::from_bits(b) != 0.0));
        assert_eq!(render(Some(6)), serial);
    }

    #[test]
    fn stats_and_offline_clock() {
        struct Io {
            produced: u64,
            seen: Vec<f32>,
        }
        impl CycleIo for Io {
            fn before_cycle(&mut self, cycle: u64, inputs: &mut Inputs<'_>) -> ControlFlow<()> {
                if self.produced == 10 {
                    return ControlFlow::Break(());
                }
                self.produced += 1;
                inputs.channel(0).fill(cycle as f32);
                ControlFlow::Continue(())
            }
            fn after_cycle(&mut self, _cycle: u64, outputs: &Outputs<'_>) {
                self.seen.push(outputs.port(Port::node(NodeId(0), 0))[3]);
            }
        }
        let mut g = GraphBuilder::new(64, 48_000.0, 1);
        g.add_node(
            "copy",
            1,
            &[Port::External(0)],
            |_: &ProcessCtx, io: &mut NodeIo<'_>| {
                let x = io.input(0);
                io.output(0).copy_from_slice(x);
            },
        )
        .unwrap();
        let mut e = Engine::new(g.build().unwrap(), cfg(2));
        let stats = e.stats();
        let mut io = Io {
            produced: 0,
            seen: Vec::new(),
        };
        let n = OfflineClock::default().tick(&mut e, &mut io);
        assert_eq!(n, 10);
        assert_eq!(io.seen, (1..=10).map(|c| c as f32).collect::<Vec<_>>());
        let snap = stats.snapshot();
        assert_eq!(snap.cycles, 10);
        assert_eq!(snap.period_ns, 1_333_333);
        assert!(snap.max_cycle_ns >= snap.avg_cycle_ns && snap.avg_cycle_ns > 0);
        assert!(snap.load_pct() >= 0.0);
        assert_eq!(snap.xruns, 0);
        stats.record_xrun();
        assert_eq!(stats.snapshot().xruns, 1);

        // Live clock: one cycle per tick; an over-period cycle counts as an xrun.
        let mut g = GraphBuilder::new(64, 48_000.0, 1);
        g.add_node("slow", 1, &[], |_: &ProcessCtx, _: &mut NodeIo<'_>| {
            std::thread::sleep(Duration::from_millis(3));
        })
        .unwrap();
        let mut e = Engine::new(g.build().unwrap(), cfg(1));
        let mut io = Io {
            produced: 0,
            seen: Vec::new(),
        };
        let mut live = LiveClock::new(false, None);
        assert_eq!(live.tick(&mut e, &mut io), 1);
        assert_eq!(e.stats().snapshot().xruns, 1);
        assert!(e.stats().snapshot().peak_load_pct() > 100.0);
    }

    #[test]
    fn stats_readable_from_other_thread_while_running() {
        let (g, _) = dummy::scene(8, 2, 1, 1, 64);
        let mut e = Engine::new(g, cfg(2));
        let h = e.stats();
        let reader = std::thread::spawn(move || {
            let mut last = 0;
            for _ in 0..1000 {
                let s = h.snapshot();
                assert!(s.cycles >= last);
                last = s.cycles;
                std::hint::spin_loop();
            }
        });
        for _ in 0..2000 {
            e.process_cycle();
        }
        reader.join().unwrap();
        assert_eq!(e.stats().snapshot().cycles, 2000);
    }

    #[test]
    fn workers_park_and_wake_between_slow_cycles() {
        let (g, main) = dummy::scene(8, 2, 1, 1, 64);
        let (g2, _) = dummy::scene(8, 2, 1, 1, 64);
        let mut e = Engine::new(g, cfg(3));
        let mut s = Engine::new(g2, cfg(0));
        for _ in 0..5 {
            for ch in 0..8 {
                e.external_mut(ch).fill(0.5);
                s.external_mut(ch).fill(0.5);
            }
            // Longer than the spin budget so workers park.
            std::thread::sleep(Duration::from_millis(2));
            e.process_cycle();
            s.process_cycle();
            assert_eq!(e.port(Port::node(main, 0)), s.port(Port::node(main, 0)));
        }
    }

    #[test]
    fn empty_graph_runs() {
        let g = GraphBuilder::new(32, 48_000.0, 0).build().unwrap();
        let mut e = Engine::new(g, cfg(2));
        e.process_cycle();
        assert_eq!(e.stats().snapshot().cycles, 1);
        assert!(GraphBuilder::new(0, 48_000.0, 0).build().is_err());
    }
}
