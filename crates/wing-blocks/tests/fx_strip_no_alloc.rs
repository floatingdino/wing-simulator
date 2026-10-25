//! T05C: the channel-strip FX and EXT never allocate after `prepare` (counting global allocator on the test
//! thread), including stage switching, parameter changes, resets and default loads.

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;

use wing_blocks::model::{create_model, BlockKind, ParamValue};
use wing_blocks::models::fx_strip::StripKind;

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
fn strips_and_ext_do_not_allocate() {
    let ids: Vec<&'static str> = StripKind::ALL
        .iter()
        .map(|k| k.id())
        .chain(["EXT"])
        .collect();
    for id in ids {
        let mut m = create_model(BlockKind::Fx, id);
        assert!(!m.is_stub());
        m.prepare(48_000.0, 64);
        let n = count_allocs(|| {
            let mut l = [0.0f32; 64];
            let mut r = [0.0f32; 64];
            let ret = [0.25f32; 64];
            for k in 0..300 {
                for (i, (a, b)) in l.iter_mut().zip(r.iter_mut()).enumerate() {
                    *a = ((i + k * 64) as f32 * 0.07).sin() * 0.5;
                    *b = *a * 0.8;
                }
                for slot in 1..=33u16 {
                    let v = ((k * 7 + slot as usize * 13) % 100) as f32 / 10.0;
                    if k % 3 == 0 {
                        m.set_param(slot, ParamValue::Float(v));
                    } else {
                        m.set_param(slot, ParamValue::Int(((k + slot as usize) % 2) as i32));
                    }
                }
                m.commit();
                let len = 64 - (k % 5) * 9;
                m.process(&mut l[..len], &mut r[..len], Some((&ret, &ret)));
                m.process(&mut l, &mut r, None);
            }
            m.reset();
            m.load_defaults();
            m.commit();
            m.process(&mut l, &mut r, None);
            let _ = m.telemetry();
            let _ = m.latency();
        });
        assert_eq!(n, 0, "{id} allocated {n} times");
    }
}
