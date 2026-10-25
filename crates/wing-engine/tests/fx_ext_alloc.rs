//! T05C: the EXT return path (routing table EXT routes, router EXT outputs, rack return capture, keyed slot
//! processing) runs without touching the heap on the RT side while a control thread keeps changing the
//! return source group / index / mode / trim and swapping the slot model.

mod common;

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering::*};
use std::sync::Arc;

use common::rig::*;
use wing_engine::mix::MixConfig;
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
fn ext_return_path_is_allocation_free_on_rt_side() {
    let store = ParamStore::new();
    for n in 1..=4 {
        connect_usb(&store, &format!("ch/{n}"), n);
        seti(&store, &format!("ch/{n}/preins/on"), 1);
        sett(&store, &format!("ch/{n}/preins/ins"), &format!("FX{n}"));
        sett(&store, &format!("fx/{n}/mdl"), "EXT");
        sett(&store, &format!("fx/{n}/1"), "USB");
        seti(&store, &format!("fx/{n}/2"), 10 + n as i32);
    }
    let mut r = rig(&store, MixConfig::default(), 0);
    let all: Vec<u32> = (1..=16).collect();
    r.cycles(3, &all); // warm-up

    let store = Arc::new(store);
    let stop = Arc::new(AtomicBool::new(false));
    let control = {
        let (store, stop) = (store.clone(), stop.clone());
        std::thread::spawn(move || {
            let sel = |p: &str| id(&store, p);
            let nodes: Vec<_> = (1..=4)
                .map(|n| {
                    (
                        sel(&format!("fx/{n}/mdl")),
                        sel(&format!("fx/{n}/1")),
                        sel(&format!("fx/{n}/2")),
                        sel(&format!("fx/{n}/3")),
                        sel(&format!("fx/{n}/5")),
                    )
                })
                .collect();
            let mut i = 0usize;
            while !stop.load(Relaxed) {
                i = i.wrapping_add(1);
                let (mdl, grp, inn, mode, trim) = nodes[i % 4];
                let _ = store.set_text(mdl, ["EXT", "EXT", "NONE", "HALL"][i / 4 % 4], 0);
                let _ = store.set(grp, Value::Enum((i / 3 % 12) as u32));
                let _ = store.set(inn, Value::Int((i % 20) as i32 + 1));
                let _ = store.set(mode, Value::Enum((i / 5 % 3) as u32));
                let _ = store.set(trim, Value::Float((i % 36) as f32 - 18.0));
                std::thread::yield_now();
            }
        })
    };

    ARMED.with(|a| a.set(true));
    drop(std::hint::black_box(vec![0u8; 16]));
    ARMED.with(|a| a.set(false));
    assert_eq!(RT_HEAP_OPS.swap(0, Relaxed), 2);

    ARMED.with(|a| a.set(true));
    for k in 0..3000 {
        r.feed(&all);
        r.pipe.apply(&mut r.e);
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
}
