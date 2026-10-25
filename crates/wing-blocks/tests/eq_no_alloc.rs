//! Zero-allocation check for the STD EQ and PRE EQ: `set_params`, `process` and `reset` must not touch the heap
//! after construction/prepare (counting global allocator armed on the test thread).

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;

use wing_blocks::eq::{BandParams, BandType, EqParams, EqVariant, PreEq, PreEqParams, StdEq};

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
fn eq_process_paths_do_not_allocate() {
    let fs = 48_000.0;
    let mut strips: Vec<StdEq> = [EqVariant::Channel, EqVariant::Aux, EqVariant::Bus]
        .into_iter()
        .map(|v| {
            let mut e = StdEq::new(v);
            e.prepare(fs, 64);
            e
        })
        .collect();
    let mut pre = PreEq::new();
    pre.prepare(fs, 64);
    let mut l = vec![0.25f32; 64];
    let mut r = vec![-0.25f32; 64];

    let n = count_allocs(|| {
        for k in 0..400usize {
            for eq in strips.iter_mut() {
                let mut p = EqParams::defaults(eq.variant());
                p.on = k % 50 != 0;
                p.mix = (k % 126) as f32;
                p.low_type = BandType::ALL[k % 13];
                p.high_type = BandType::ALL[(k * 7) % 13];
                p.low = BandParams::new((k % 31) as f32 - 15.0, 20.0 + k as f32 * 10.0, 0.5);
                p.bands[k % 6] = BandParams::new(12.0, 100.0 + k as f32 * 40.0, 4.0);
                p.tilt_db = (k % 13) as f32 - 6.0;
                eq.set_params(&p);
                eq.process(&mut l, &mut r);
                eq.process(&mut l[..17], &mut r[..17]);
            }
            let mut pp = PreEqParams {
                on: k % 3 != 0,
                ..Default::default()
            };
            pp.bands[k % 3].gain_db = 9.0;
            pre.set_params(&pp);
            pre.process(&mut l, &mut r);
        }
        for eq in strips.iter_mut() {
            eq.reset();
            let _ = eq.magnitude_db(1000.0);
        }
        pre.reset();
    });
    assert_eq!(n, 0, "EQ RT paths allocated {n} times");
}
