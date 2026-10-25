//! T05E: ENHANCE / EXCITER / P-BASS / SUB-M / V-IMG / TAPE never allocate after construction + `prepare`,
//! including fast parameter modulation, resets, tape extras and odd block lengths (counting global allocator).

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;

use wing_blocks::model::{Model, ParamValue};
use wing_blocks::models::fx_enhance::{
    Exciter, PsychoBass, SubMonster, TapeExtras, TapeMachine, UltraEnhancer, VelvetImager,
};

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
fn enhance_family_does_not_allocate() {
    let mut tape = TapeMachine::new();
    tape.set_extras(TapeExtras {
        wow: 0.7,
        flutter: 0.5,
        hiss_db: -60.0,
    });
    let mut ms: Vec<Box<dyn Model>> = vec![
        Box::new(UltraEnhancer::new()),
        Box::new(Exciter::new()),
        Box::new(PsychoBass::new()),
        Box::new(SubMonster::new()),
        Box::new(VelvetImager::new()),
        Box::new(TapeMachine::new()),
        Box::new(tape),
    ];
    for m in &mut ms {
        m.prepare(48_000.0, 64);
        let n = count_allocs(|| {
            let (mut l, mut r) = ([0.0f32; 200], [0.0f32; 200]);
            for blk in 0..200 {
                for (i, v) in l.iter_mut().enumerate() {
                    *v = ((i + blk * 3) as f32 * 0.07).sin() * 0.8;
                }
                for (i, v) in r.iter_mut().enumerate() {
                    *v = ((i + blk * 5) as f32 * 0.11).sin() * 0.6;
                }
                for slot in 1..=13u16 {
                    let x = ((blk * 13 + slot as usize * 7) % 100) as f32 * 0.01;
                    m.set_param(slot, ParamValue::Float(x * 50.0 - 25.0));
                }
                m.set_param(4, ParamValue::Enum((blk % 2) as u32));
                if blk % 17 == 0 {
                    m.commit();
                }
                if blk % 50 == 49 {
                    m.reset();
                }
                let len = 1 + (blk * 37) % 200; // odd lengths, above and below max_block
                m.process(&mut l[..len], &mut r[..len], None);
                let _ = m.telemetry();
                let _ = m.latency();
            }
            m.load_defaults();
        });
        assert_eq!(n, 0, "{} allocated {n} times", m.id());
    }
}
