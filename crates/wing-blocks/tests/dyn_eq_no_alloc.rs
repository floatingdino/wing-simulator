//! T048: DS902 / DEQ / DEQ2 never allocate after construction + `prepare`, including fast parameter
//! modulation, shape changes, listen, external keys and odd block lengths (counting global allocator).

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;

use wing_blocks::model::{Model, ParamValue};
use wing_blocks::models::dyn_eq::{DeEsser902, DynEq};

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
fn dyn_eq_family_does_not_allocate() {
    let mut deq = DynEq::single();
    let mut deq2 = DynEq::dual();
    let mut ds = DeEsser902::new();
    let models: [&mut dyn Model; 3] = [&mut deq, &mut deq2, &mut ds];
    for m in models {
        m.prepare(48_000.0, 64);
        let key = [0.7f32; 64];
        let n = count_allocs(|| {
            let (mut l, mut r) = ([0.0f32; 64], [0.0f32; 64]);
            for b in 0..400 {
                for (i, (a, c)) in l.iter_mut().zip(r.iter_mut()).enumerate() {
                    let v =
                        ((b * 64 + i) as f32 * 0.37).sin() * if b % 3 == 0 { 0.9 } else { 0.01 };
                    *a = v;
                    *c = -v;
                }
                for s in 1..=19u16 {
                    m.set_param(
                        s,
                        ParamValue::Float(((b as u16 * 7 + s) % 13) as f32 * 50.0 - 40.0),
                    );
                }
                m.set_param(5, ParamValue::Enum((b % 6) as u32));
                m.commit();
                let len = 1 + (b * 13) % 64;
                if b % 2 == 0 {
                    m.process(&mut l[..len], &mut r[..len], None);
                } else {
                    m.process(
                        &mut l[..len],
                        &mut r[..len],
                        Some((&key[..len], &key[..len])),
                    );
                }
                let _ = m.telemetry();
                if b % 97 == 0 {
                    m.load_defaults();
                    m.reset();
                }
            }
        });
        assert_eq!(n, 0, "{}", m.id());
        assert!(l_finite(m));
    }
    let n = count_allocs(|| {
        deq2.set_listen(Some(1));
        let (mut l, mut r) = ([0.3f32; 64], [0.3f32; 64]);
        deq2.process(&mut l, &mut r, None);
        deq2.set_listen(None);
    });
    assert_eq!(n, 0);
}

/// The model still produces finite output after the abuse above.
fn l_finite(m: &mut dyn Model) -> bool {
    let (mut l, mut r) = ([0.5f32; 64], [0.5f32; 64]);
    m.process(&mut l, &mut r, None);
    l.iter().chain(&r).all(|v| v.is_finite())
}
