//! T050 / T044: the FX rack runs without touching the heap on the RT side while a control thread keeps
//! hot-swapping slot models (incl. ST-DL), changing FX parameters and `fxmix`, moving slots between
//! inserts (owner changes reset slots) and toggling automix membership.

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
fn fx_model_swaps_and_inserts_are_allocation_free_on_rt_side() {
    let store = ParamStore::new();
    for n in 1..=8 {
        connect_usb(&store, &format!("ch/{n}"), n);
        seti(&store, &format!("ch/{n}/preins/on"), 1);
        sett(&store, &format!("ch/{n}/preins/ins"), &format!("FX{n}"));
        seti(&store, &format!("ch/{n}/send/1/on"), 1);
    }
    for b in 1..=8 {
        seti(&store, &format!("bus/{b}/preins/on"), 1);
        sett(
            &store,
            &format!("bus/{b}/preins/ins"),
            &format!("FX{}", b + 8),
        );
    }
    seti(&store, "ch/1/postins/on", 1);
    sett(&store, "fx/1/mdl", "ST-DL");
    let mut r = rig(&store, MixConfig::default(), 0);
    let all: Vec<u32> = (1..=8).collect();
    r.cycles(3, &all); // warm-up

    let store = Arc::new(store);
    let stop = Arc::new(AtomicBool::new(false));
    let control = {
        let (store, stop) = (store.clone(), stop.clone());
        std::thread::spawn(move || {
            let sel = |p: &str| id(&store, p);
            let full = ["ST-DL", "GEQ", "HALL", "NONE", "*MASTER*", "PCORR"];
            let lite = ["GEQ", "NONE", "LIMITER", "*BUS*"];
            let mdl: Vec<_> = (1..=16).map(|n| sel(&format!("fx/{n}/mdl"))).collect();
            let mix: Vec<_> = (1..=16).map(|n| sel(&format!("fx/{n}/fxmix"))).collect();
            let ins = sel("ch/1/preins/ins");
            let bus_ins = sel("bus/2/preins/ins");
            let mode = sel("ch/1/postins/mode");
            let w = sel("ch/1/postins/w");
            let amix = sel("cfg/amix/x");
            let mut i = 0usize;
            while !stop.load(Relaxed) {
                i = i.wrapping_add(1);
                let n = i % 16;
                let m = if n < 8 {
                    full[i / 16 % full.len()]
                } else {
                    lite[i / 16 % lite.len()]
                };
                let _ = store.set_text(mdl[n], m, 0);
                if n < 8 && m == "ST-DL" {
                    let _ = store.set(
                        sel(&format!("fx/{}/1", n + 1)),
                        Value::Float((i % 500) as f32 + 1.0),
                    );
                    let _ = store.set(
                        sel(&format!("fx/{}/6", n + 1)),
                        Value::Float((i % 90) as f32),
                    );
                }
                let _ = store.set(mix[n], Value::Float((i % 101) as f32));
                // move FX1/FX2 between ch 1 and bus 2 (owner changes reset the slot)
                let _ = store.set(ins, Value::Enum((i % 3) as u32));
                let _ = store.set(bus_ins, Value::Enum((i % 2 + 1) as u32));
                let _ = store.set(mode, Value::Enum((i / 3 % 3) as u32));
                let _ = store.set(w, Value::Float((i % 24) as f32 - 12.0));
                let _ = store.set(amix, Value::Int((i / 5 % 2) as i32));
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
