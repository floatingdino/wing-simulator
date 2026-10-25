//! T036/T037: the complete mix graph (PUBLIC.snap scene, bus/main/matrix strip processing on) runs
//! without touching the heap on the RT side, including parameter changes (faders, mutes, pans, sends,
//! modes, bus EQ/dyn/delay values and model swaps) applied by the pipeline while a control thread
//! keeps changing the store.

mod common;

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering::*};
use std::sync::Arc;

use common::rig::*;
use wing_engine::mix::MixConfig;
use wing_model::Value;

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
fn mix_graph_is_allocation_free_on_rt_side() {
    let store = public_store();
    for n in 1..=40 {
        connect_usb(&store, &format!("ch/{n}"), n);
    }
    let mut r = rig(&store, MixConfig::default(), 0);
    let all: Vec<u32> = (1..=48).collect();
    r.cycles(3, &all); // warm-up

    let store = Arc::new(store);
    let stop = Arc::new(AtomicBool::new(false));
    let control = {
        let (store, stop) = (store.clone(), stop.clone());
        std::thread::spawn(move || {
            let ids: Vec<_> = [
                "ch/1/fdr",
                "ch/5/pan",
                "ch/9/wid",
                "ch/21/send/1/lvl",
                "ch/2/send/13/pan",
                "bus/5/fdr",
                "main/1/pan",
                "mtx/1/fdr",
                "bus/1/send/5/lvl",
                // T037 strip processing
                "bus/3/in/set/trim",
                "bus/5/eq/5",
                "main/1/eq/27",
                "mtx/2/dyn/1",
                "main/2/dly/dly",
                "bus/6/dynxo/f",
            ]
            .iter()
            .map(|p| id(&store, p))
            .collect();
            let toggles: Vec<_> = [
                "ch/3/mute",
                "bus/7/busmono",
                "ch/12/send/3/on",
                "bus/2/eq/on",
                "bus/9/dyn/on",
                "main/3/dly/on",
                "mtx/4/eq/on",
                "mtx/4/in/set/inv",
                "bus/1/preins/on",
            ]
                .iter()
                .map(|p| id(&store, p))
                .collect();
            let mode = id(&store, "ch/4/send/4/mode");
            let bus_dyn = id(&store, "bus/9/dyn/mdl");
            let mtx_eq = id(&store, "mtx/4/eq/mdl");
            let dly_mode = id(&store, "main/3/dly/mode");
            let mut i = 0u32;
            while !stop.load(Relaxed) {
                i = i.wrapping_add(1);
                let v = (i % 40) as f32 - 30.0;
                for &p in &ids {
                    let _ = store.set(p, Value::Float(v));
                }
                for &p in &toggles {
                    let _ = store.set(p, Value::Int((i & 1) as i32));
                }
                let _ = store.set(mode, Value::Enum(i % 3));
                let _ = store.set(bus_dyn, Value::Enum(i % 32));
                let _ = store.set(mtx_eq, Value::Enum(i % 7));
                let _ = store.set(dly_mode, Value::Enum(i % 4));
                std::thread::yield_now();
            }
        })
    };

    // the counter works
    ARMED.with(|a| a.set(true));
    drop(std::hint::black_box(vec![0u8; 16]));
    ARMED.with(|a| a.set(false));
    assert_eq!(RT_HEAP_OPS.swap(0, Relaxed), 2);

    ARMED.with(|a| a.set(true));
    let mut changes = 0;
    for k in 0..3000 {
        r.feed(&all);
        changes += r.pipe.apply(&mut r.e).changes;
        if k % 2 == 0 {
            r.e.process_cycle_serial();
        } else {
            r.e.process_cycle();
        }
    }
    ARMED.with(|a| a.set(false));
    stop.store(true, Relaxed);
    control.join().unwrap();
    assert_eq!(RT_HEAP_OPS.load(Relaxed), 0, "RT side touched the heap");
    assert!(changes > 100, "only {changes} changes applied");
}
