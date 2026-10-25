//! Proves the cycle path performs zero heap allocations, on the driver *and* worker threads.
//! A counting global allocator records every allocation in the process; this file holds a single
//! test so nothing else runs concurrently in this binary.

use std::alloc::{GlobalAlloc, Layout, System};
use std::ops::ControlFlow;
use std::sync::atomic::{AtomicU64, Ordering::Relaxed};
use std::time::Duration;

use wing_engine::{
    dummy, Clock, CycleIo, Engine, EngineConfig, Inputs, LiveClock, OfflineClock, Outputs, Port,
};

struct Counting;

static ALLOCS: AtomicU64 = AtomicU64::new(0);

unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, l: Layout) -> *mut u8 {
        ALLOCS.fetch_add(1, Relaxed);
        System.alloc(l)
    }
    unsafe fn alloc_zeroed(&self, l: Layout) -> *mut u8 {
        ALLOCS.fetch_add(1, Relaxed);
        System.alloc_zeroed(l)
    }
    unsafe fn realloc(&self, p: *mut u8, l: Layout, n: usize) -> *mut u8 {
        ALLOCS.fetch_add(1, Relaxed);
        System.realloc(p, l, n)
    }
    unsafe fn dealloc(&self, p: *mut u8, l: Layout) {
        System.dealloc(p, l)
    }
}

#[global_allocator]
static GLOBAL: Counting = Counting;

struct Io {
    left: u64,
    acc: f32,
}

impl CycleIo for Io {
    fn before_cycle(&mut self, cycle: u64, inputs: &mut Inputs<'_>) -> ControlFlow<()> {
        if self.left == 0 {
            return ControlFlow::Break(());
        }
        self.left -= 1;
        for ch in 0..inputs.num_channels() {
            inputs.channel(ch).fill((cycle % 7) as f32 * 0.1);
        }
        ControlFlow::Continue(())
    }
    fn after_cycle(&mut self, _cycle: u64, outputs: &Outputs<'_>) {
        self.acc += outputs.port(Port::External(0))[0];
    }
}

#[test]
fn zero_allocations_per_cycle() {
    let (g, main) = dummy::scene(48, 16, 2, 2, 64);
    let cfg = EngineConfig {
        workers: 6,
        spin: Duration::from_micros(30),
        rt_workers: true,
        workgroup: None,
    };
    let mut e = Engine::new(g, cfg);
    let stats = e.stats();

    // Warm-up: thread start-up, first parks, lazily initialised runtime state.
    for _ in 0..200 {
        e.process_cycle();
    }
    std::thread::sleep(Duration::from_millis(5));
    let mut live = LiveClock::new(true, None);
    let mut io = Io { left: 1, acc: 0.0 };
    live.tick(&mut e, &mut io); // one-time RT setup happens on the first tick
    std::thread::sleep(Duration::from_millis(5));

    let before = ALLOCS.load(Relaxed);
    assert!(before > 0, "counting allocator is not installed");

    // Hot cycles (workers spinning).
    for _ in 0..2000 {
        e.process_cycle();
    }
    // Paced cycles (workers park and are woken each cycle).
    for _ in 0..20 {
        std::thread::sleep(Duration::from_micros(300));
        e.process_cycle();
    }
    // Via the clocks and I/O hooks.
    io.left = 500;
    for _ in 0..500 {
        live.tick(&mut e, &mut io);
    }
    io.left = 500;
    OfflineClock { max_cycles: None }.tick(&mut e, &mut io);
    let snap = stats.snapshot();
    let out0 = e.port(Port::node(main, 0))[0];

    let after = ALLOCS.load(Relaxed);
    assert_eq!(after - before, 0, "allocations on the cycle path");
    assert!(snap.cycles >= 3000);
    assert!(out0.is_finite());
}
