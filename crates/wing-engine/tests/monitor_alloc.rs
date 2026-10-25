//! T038: DCAs, mute groups, solo (incl. SIP) and the monitor section run without touching the
//! heap on the RT side while a control thread keeps toggling solos, DCA faders/mutes, mute groups,
//! tags and monitor parameters and refreshing the [`MixLogic`].

mod common;

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering::*};
use std::sync::Arc;

use common::rig::*;
use wing_engine::mix::{MixConfig, MixLogic};
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
fn dca_solo_monitor_is_allocation_free_on_rt_side() {
    let mut store = public_store();
    for n in 1..=40 {
        connect_usb(&store, &format!("ch/{n}"), n);
    }
    let mut logic = MixLogic::new(&store);
    store.add_hook(logic.change_hook());
    sett(&store, "cfg/mon/1/eq/on", "1");
    sett(&store, "cfg/mon/1/eq/2g", "4");
    sett(&store, "cfg/mon/1/dly/on", "1");
    sett(&store, "cfg/mon/1/lim", "-20");
    logic.refresh(&store);
    let cfg = MixConfig {
        logic: Some(logic.shared()),
        ..MixConfig::default()
    };
    let mut r = rig(&store, cfg, 0);
    let all: Vec<u32> = (1..=48).collect();
    r.cycles(3, &all); // warm-up

    let store = Arc::new(store);
    let stop = Arc::new(AtomicBool::new(false));
    let control = {
        let (store, stop) = (store.clone(), stop.clone());
        std::thread::spawn(move || {
            let sel = |p: &str| id(&store, p);
            let solos: Vec<_> = [
                "ch/1/$solo",
                "ch/7/$solo",
                "bus/3/$solo",
                "main/1/$solo",
                "dca/2/$solo",
            ]
            .iter()
            .map(|p| sel(p))
            .collect();
            let toggles: Vec<_> = [
                "mgrp/1/mute",
                "dca/1/mute",
                "cfg/solo/$dim",
                "cfg/solo/$mono",
                "cfg/solo/chtap",
                "$ctl/cfg/soloexcl",
                "cfg/mon/1/inv",
            ]
            .iter()
            .map(|p| sel(p))
            .collect();
            let floats: Vec<_> = [
                "dca/1/fdr",
                "dca/2/fdr",
                "cfg/mon/1/srcmix",
                "cfg/mon/1/eq/2f",
                "cfg/mon/1/dly/m",
                "cfg/mon/2/lvl",
            ]
            .iter()
            .map(|p| sel(p))
            .collect();
            let mode = sel("cfg/solo/mode");
            let stat = sel("$stat/solo");
            let mut i = 0u32;
            while !stop.load(Relaxed) {
                i = i.wrapping_add(1);
                let _ = store.set(solos[i as usize % solos.len()], Value::Int(1));
                for &p in &toggles {
                    let _ = store.set(p, Value::Int((i >> 1 & 1) as i32));
                }
                for &p in &floats {
                    let _ = store.set(p, Value::Float((i % 40) as f32 - 30.0));
                }
                let _ = store.set(mode, Value::Enum(i % 3));
                let tags = if i & 4 != 0 { "#D1,#M1" } else { "#D2" };
                let _ = store.set_str(sel("ch/3/tags"), tags, 0);
                if i.is_multiple_of(7) {
                    let _ = store.set(stat, Value::Int(0));
                }
                logic.refresh(&store);
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
