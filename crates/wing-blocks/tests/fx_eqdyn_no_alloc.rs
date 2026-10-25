//! T05Q: the FX-rack EQ/dyn models (GEQ, PIA, FX EQs, LIMITER, DE-S2, DEQ3, C5-CMB, SPKMAN) never allocate
//! after construction + `prepare`, including parameter sweeps over every slot, mode/type switches,
//! `load_defaults`/`reset` and odd block lengths (counting global allocator).

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;

use wing_blocks::model::{create_model, BlockKind, Model, ParamValue};

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
fn fx_eqdyn_models_do_not_allocate() {
    for id in [
        "GEQ", "PIA", "SOUL", "E88", "E84", "F110", "PULSAR", "MACH4", "LIMITER", "DE-S2", "DEQ3", "C5-CMB",
        "SPKMAN",
    ] {
        let mut m = create_model(BlockKind::Fx, id);
        m.prepare(48_000.0, 64);
        let n = count_allocs(|| {
            let (mut l, mut r) = ([0.0f32; 64], [0.0f32; 64]);
            for b in 0..300usize {
                for (i, (a, c)) in l.iter_mut().zip(r.iter_mut()).enumerate() {
                    let v = ((b * 64 + i) as f32 * 0.37).sin() * if b % 3 == 0 { 0.9 } else { 0.02 };
                    *a = v;
                    *c = -0.5 * v;
                }
                // sweep every slot through a range of raw values (models clamp)
                let s = 1 + (b % 40) as u16;
                m.set_param(s, ParamValue::Float(((b * 7) % 23) as f32 * 3.0 - 30.0));
                m.set_param(s, ParamValue::Int((b % 11) as i32));
                m.set_param(1 + ((b + 3) % 40) as u16, ParamValue::Enum((b % 4) as u32));
                m.commit();
                let len = 1 + (b * 13) % 64;
                m.process(&mut l[..len], &mut r[..len], None);
                let _ = m.telemetry();
                let _ = m.latency();
                if b % 97 == 0 {
                    m.load_defaults();
                    m.reset();
                }
            }
        });
        assert_eq!(n, 0, "{id} allocated {n} times");
    }
}
