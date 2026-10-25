//! Proves routing-table swaps are allocation- and free-free on the RT side: a counting global
//! allocator counts heap operations on threads that armed it (thread-local flag), while a control
//! thread rebuilds and publishes tables concurrently.

mod common;

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering::*};
use std::sync::Arc;

use wing_engine::routing::{add_input_stage, add_usb_output, RoutingControl, NO_TAPS};
use wing_engine::{Engine, EngineConfig, GraphBuilder};
use wing_model::{ParamStore, Value};

struct Counting;

thread_local! {
    static ARMED: Cell<bool> = const { Cell::new(false) };
}
static RT_HEAP_OPS: AtomicU64 = AtomicU64::new(0);

fn note() {
    if ARMED.try_with(|a| a.get()).unwrap_or(false) {
        RT_HEAP_OPS.fetch_add(1, Relaxed);
    }
}

unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, l: Layout) -> *mut u8 {
        note();
        System.alloc(l)
    }
    unsafe fn alloc_zeroed(&self, l: Layout) -> *mut u8 {
        note();
        System.alloc_zeroed(l)
    }
    unsafe fn realloc(&self, p: *mut u8, l: Layout, n: usize) -> *mut u8 {
        note();
        System.realloc(p, l, n)
    }
    unsafe fn dealloc(&self, p: *mut u8, l: Layout) {
        note();
        System.dealloc(p, l)
    }
}

#[global_allocator]
static GLOBAL: Counting = Counting;

#[test]
fn table_swap_is_allocation_free_on_rt_side() {
    let mut store = ParamStore::new();
    common::load_routing(&store, common::PUBLIC_SNAP);
    let mut ctl = RoutingControl::new(&store);
    store.add_hook(ctl.change_hook());
    let store = Arc::new(store);

    let mut g = GraphBuilder::new(64, 48_000.0, 48);
    let io = add_input_stage(&mut g, &mut ctl).unwrap();
    add_usb_output(&mut g, &mut ctl, io.osc, &NO_TAPS).unwrap();
    // serial engine: every node runs on this (armed) thread
    let mut e = Engine::new(
        g.build().unwrap(),
        EngineConfig {
            workers: 0,
            rt_workers: false,
            ..EngineConfig::default()
        },
    );
    e.process_cycle_serial(); // warm-up

    let stop = Arc::new(AtomicBool::new(false));
    let published = Arc::new(AtomicU64::new(0));
    let control = {
        let (store, stop, published) = (store.clone(), stop.clone(), published.clone());
        std::thread::spawn(move || {
            let altsw = store.lookup("io/altsw").unwrap();
            let pol = store.lookup("io/in/USB/3/pol").unwrap();
            let mut i = 0u32;
            while !stop.load(Relaxed) {
                i += 1;
                store.set(altsw, Value::Int((i & 1) as i32)).unwrap();
                store.set(pol, Value::Int(((i >> 1) & 1) as i32)).unwrap();
                if ctl.refresh(&store) {
                    published.fetch_add(1, Relaxed);
                }
                std::thread::yield_now();
            }
            ctl.collect();
            ctl
        })
    };

    // the counter works
    ARMED.with(|a| a.set(true));
    drop(std::hint::black_box(vec![0u8; 16]));
    ARMED.with(|a| a.set(false));
    assert_eq!(RT_HEAP_OPS.swap(0, Relaxed), 2);

    ARMED.with(|a| a.set(true));
    let mut cycles = 0u64;
    while cycles < 20_000 || published.load(Relaxed) < 500 {
        e.process_cycle_serial();
        cycles += 1;
        assert!(cycles < 5_000_000, "control thread starved");
    }
    ARMED.with(|a| a.set(false));
    stop.store(true, Relaxed);
    let ctl = control.join().unwrap();
    assert_eq!(RT_HEAP_OPS.load(Relaxed), 0, "RT side touched the heap");
    assert!(published.load(Relaxed) >= 500);
    // the RT side eventually adopts the latest table
    for _ in 0..3 {
        ctl.collect();
        e.process_cycle_serial();
    }
    assert!(ctl.settled());
}
