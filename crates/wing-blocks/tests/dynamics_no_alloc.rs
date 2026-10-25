//! Zero-allocation check for the standard dynamics block: after construction/prepare, `set_params` and
//! `process*` must not touch the heap.

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;

use wing_blocks::dynamics::*;

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

#[test]
fn dynamics_process_does_not_allocate() {
    let mut comp = Dynamics::new(DynParams::comp());
    comp.prepare(48_000.0, 2);
    let mut exp = Dynamics::new(DynParams::exp());
    exp.prepare(48_000.0, 1);
    let mut l: Vec<f32> = (0..64).map(|i| (i as f32 * 0.1).sin()).collect();
    let mut r = l.clone();
    let key = l.clone();
    let mut keyed = DynParams::comp();
    keyed.key.kind = KeyFilterKind::Hp12;
    keyed.det = Detector::Peak;
    keyed.env = Envelope::Lin;

    let n = count_allocs(|| {
        for _ in 0..8 {
            comp.process(&mut [&mut l, &mut r], None);
            comp.process(&mut [&mut l, &mut r], Some(&[&key]));
            exp.process_mono(&mut l, None);
            exp.process_mono(&mut r, Some(&key));
        }
        comp.set_params(&keyed);
        comp.process(&mut [&mut l, &mut r], Some(&[&key, &key]));
        exp.reset();
        std::hint::black_box(comp.gain_reduction_db() + exp.block_peak_gain_reduction_db());
    });
    assert_eq!(n, 0, "dynamics allocated on the process path");
}
