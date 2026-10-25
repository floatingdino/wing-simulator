//! T035/T045: model switches, parameter changes, order changes and processing of channel/aux strips never
//! touch the heap (counting global allocator on the test thread).

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;

use wing_engine::channel::{block, ChannelStrip, NoInserts, StripKeys};
use wing_engine::params::{ParamMsg, TARGET_BASE};
use wing_model::Schema;

struct Counting;

thread_local! {
    static ARMED: Cell<bool> = const { Cell::new(false) };
    static COUNT: Cell<usize> = const { Cell::new(0) };
}

unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let _ = ARMED.try_with(|a| {
            if a.get() {
                COUNT.with(|c| c.set(c.get() + 1));
            }
        });
        System.alloc(layout)
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        System.dealloc(ptr, layout)
    }
    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new: usize) -> *mut u8 {
        let _ = ARMED.try_with(|a| {
            if a.get() {
                COUNT.with(|c| c.set(c.get() + 1));
            }
        });
        System.realloc(ptr, layout, new)
    }
}

#[global_allocator]
static GLOBAL: Counting = Counting;

fn count_allocs(f: impl FnOnce()) -> usize {
    COUNT.with(|c| c.set(0));
    ARMED.with(|a| a.set(true));
    f();
    ARMED.with(|a| a.set(false));
    COUNT.with(|c| c.get())
}

fn models(key: &str) -> Vec<u16> {
    let s = Schema::get();
    let g = s.groups().iter().find(|g| g.key == key).unwrap();
    s.models(g).iter().map(|m| m.enum_index as u16).collect()
}

#[test]
fn strip_model_switches_and_processing_do_not_allocate() {
    let mut ch = ChannelStrip::channel(48_000.0, 64);
    let mut aux = ChannelStrip::aux(48_000.0, 64);
    let sets: Vec<(u8, Vec<u16>)> = vec![
        (block::GATE, models("ch/*/gate")),
        (block::DYN, models("ch/*/dyn")),
        (block::EQ, models("ch/*/eq")),
        (block::TOOL, models("ch/*/flt")),
    ];
    let aux_dyn = models("aux/*/dyn");
    let aux_eq = models("aux/*/eq");
    let key = vec![0.3f32; 64];
    let n = count_allocs(|| {
        let mut l = [0.1f32; 64];
        let mut r = [0.1f32; 64];
        for (blk, ms) in &sets {
            ch.apply(&ParamMsg::Set {
                block: *blk,
                param: 0,
                value: 1.0,
            });
            for &m in ms {
                ch.apply(&ParamMsg::Model {
                    block: *blk,
                    model: m,
                });
                for p in 0..19 {
                    ch.apply(&ParamMsg::Set {
                        block: *blk,
                        param: TARGET_BASE + p,
                        value: -10.0 + p as f32,
                    });
                }
                ch.apply(&ParamMsg::Commit);
                for k in 0..4 {
                    ch.apply(&ParamMsg::Set {
                        block: block::STRIP,
                        param: 0,
                        value: ((m as usize * 5 + k) % 24) as f32,
                    });
                    let keys = StripKeys {
                        gate: Some((&key, &key)),
                        dyn_: Some((&key, &key)),
                    };
                    ch.process_with(&mut l, &mut r, keys, &mut NoInserts);
                }
            }
        }
        for (blk, ms) in [(block::DYN, &aux_dyn), (block::EQ, &aux_eq)] {
            aux.apply(&ParamMsg::Set {
                block: blk,
                param: 0,
                value: 1.0,
            });
            for &m in ms.iter() {
                aux.apply(&ParamMsg::Model {
                    block: blk,
                    model: m,
                });
                aux.apply(&ParamMsg::Commit);
                aux.process_stereo(&mut l, &mut r);
                aux.process_stereo(&mut l, &mut r);
            }
        }
        let _ = ch.take_telemetry();
    });
    assert_eq!(n, 0, "allocations on the RT path");
}
