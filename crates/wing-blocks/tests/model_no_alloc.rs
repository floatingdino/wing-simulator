//! T045: `ModelRegistry` selection, parameter sets and processing of every catalog model never allocate
//! after `prepare` (counting global allocator on the test thread).

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;

use wing_blocks::eq::EqVariant;
use wing_blocks::model::{BlockKind, ModelRegistry, ParamValue};

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
fn registry_switches_do_not_allocate() {
    for kind in [
        BlockKind::Dynamics,
        BlockKind::Eq(EqVariant::Channel),
        BlockKind::Eq(EqVariant::Bus),
        BlockKind::FilterTool,
        BlockKind::Fx,
    ] {
        let mut reg = ModelRegistry::with_catalog(kind);
        reg.prepare(48_000.0, 64);
        let key = [0.2f32; 64];
        let n = count_allocs(|| {
            let (mut l, mut r) = ([0.1f32; 64], [0.1f32; 64]);
            for i in 0..reg.len() {
                assert!(reg.select(i));
                let m = reg.active_mut();
                for s in 1..=27 {
                    m.set_param(s, ParamValue::Float(1.0));
                }
                m.commit();
                m.process(&mut l, &mut r, None);
                m.process(&mut l, &mut r, Some((&key, &key)));
                let _ = m.telemetry();
                m.load_defaults();
                m.reset();
            }
        });
        assert_eq!(n, 0, "{kind:?}");
    }
}
