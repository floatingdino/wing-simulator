//! T04F: ONEC and 3CMP never allocate after `prepare` (counting global allocator on the test thread),
//! including parameter modulation, odd block lengths and external keys.

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;

use wing_blocks::model::{create_model, BlockKind, ParamValue};

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
fn onec_and_3cmp_do_not_allocate() {
    for id in ["ONEC", "3CMP"] {
        let mut m = create_model(BlockKind::Dynamics, id);
        assert!(!m.is_stub());
        m.prepare(48_000.0, 64);
        let key = [0.5f32; 64];
        let n = count_allocs(|| {
            let mut l = [0.0f32; 64];
            let mut r = [0.0f32; 64];
            for k in 0..200 {
                for (i, (a, b)) in l.iter_mut().zip(r.iter_mut()).enumerate() {
                    *a = ((i + k) as f32 * 0.07).sin() * 0.6;
                    *b = *a;
                }
                m.set_param(1, ParamValue::Float((k % 11) as f32));
                m.set_param(2, ParamValue::Float(60.0 + (k % 400) as f32));
                m.set_param(3, ParamValue::Float(1200.0 + (k * 13 % 3000) as f32));
                m.set_param(5, ParamValue::Float(-30.0));
                m.set_param(6, ParamValue::Enum((k % 17) as u32));
                m.set_param(9, ParamValue::Float(((k % 30) as f32) - 15.0));
                m.commit();
                let len = 64 - (k % 5) * 9;
                m.process(&mut l[..len], &mut r[..len], None);
                m.process(&mut l, &mut r, Some((&key, &key)));
            }
            m.reset();
            m.load_defaults();
            m.commit();
            let _ = m.telemetry();
        });
        assert_eq!(n, 0, "{id} allocated {n} times");
    }
}
