//! T049: WAVE / RIDE / WARM never allocate after construction + `prepare`, including fast parameter
//! modulation, resets, side-chain keys and odd block lengths (counting global allocator).

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;

use wing_blocks::model::{Model, ParamValue};
use wing_blocks::models::dyn_special::{AutoRider, TransientDesigner, WarmthPre};

struct Counting;

thread_local! {
    static ARMED: Cell<bool> = const { Cell::new(false) };
    static COUNT: Cell<usize> = const { Cell::new(0) };
}

fn bump() {
    let _ = ARMED.try_with(|a| {
        if a.get() {
            COUNT.with(|c| c.set(c.get() + 1));
        }
    });
}

unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        bump();
        System.alloc(layout)
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        System.dealloc(ptr, layout)
    }
    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new: usize) -> *mut u8 {
        bump();
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
fn dyn_special_family_does_not_allocate() {
    let mut wave = TransientDesigner::new();
    let mut ride = AutoRider::new();
    let mut warm = WarmthPre::new();
    let models: [&mut dyn Model; 3] = [&mut wave, &mut ride, &mut warm];
    for m in models {
        m.prepare(48_000.0, 64);
        let key = [0.7f32; 200];
        let n = count_allocs(|| {
            let (mut l, mut r) = ([0.0f32; 200], [0.0f32; 200]);
            for blk in 0..200 {
                for (i, v) in l.iter_mut().enumerate() {
                    *v = ((i + blk * 3) as f32 * 0.07).sin() * 0.8;
                }
                r.copy_from_slice(&l);
                for slot in 1..=6u16 {
                    let x = ((blk * 13 + slot as usize * 7) % 100) as f32 * 0.01;
                    m.set_param(slot, ParamValue::Float(x * 50.0 - 25.0));
                }
                m.set_param(4, ParamValue::Enum((blk % 5) as u32));
                if blk % 17 == 0 {
                    m.commit();
                }
                if blk % 50 == 49 {
                    m.reset();
                }
                let len = 1 + (blk * 37) % 200; // odd lengths, above and below max_block
                let sc = (blk % 2 == 0).then_some((&key[..len], &key[..len]));
                m.process(&mut l[..len], &mut r[..len], sc);
                let _ = m.telemetry();
                let _ = m.latency();
            }
            m.load_defaults();
        });
        assert_eq!(n, 0, "{} allocated {n} times", m.id());
    }
}
