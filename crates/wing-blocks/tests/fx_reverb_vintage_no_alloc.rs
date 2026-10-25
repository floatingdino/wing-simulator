//! T05V: the vintage reverbs (V-ROOM, V-REV, VSS3) never allocate after `prepare`: parameter
//! sets over every slot and range, commits, processing (incl. glides, freeze, ER table crossfades, long blocks
//! chunked), reset and defaults
//! (counting global allocator on the test thread).

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
fn vintage_reverbs_do_not_allocate() {
    for id in ["V-ROOM", "V-REV", "VSS3"] {
        let mut m = create_model(BlockKind::Fx, id);
        assert!(!m.is_stub());
        m.prepare(48_000.0, 64);
        let n = count_allocs(|| {
            let (mut l, mut r) = ([0.0f32; 64], [0.0f32; 64]);
            for k in 0..200 {
                for s in 1..=32u16 {
                    // alternate low / high / mid values (clamped by the model)
                    let v = match (k + s as usize) % 3 {
                        0 => -1e6,
                        1 => 1e6,
                        _ => 50.0,
                    };
                    m.set_param(s, ParamValue::Float(v));
                }
                m.commit();
                for (i, x) in l.iter_mut().enumerate() {
                    *x = ((i + k) as f32 * 0.37).sin() * 0.5;
                }
                r.copy_from_slice(&l);
                m.process(&mut l, &mut r, None);
                if k % 50 == 49 {
                    m.reset();
                    m.load_defaults();
                }
            }
            // a block longer than max_block (VSS3 chunks it)
            let (mut l, mut r) = ([0.1f32; 300], [0.1f32; 300]);
            m.process(&mut l, &mut r, None);
        });
        assert_eq!(n, 0, "{id}");
    }
}
