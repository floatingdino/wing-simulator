//! Zero-allocation check for the input stage, filter block and gate/ducker: after `prepare`, `set_*` and
//! `process` must not touch the heap. A counting global allocator records allocations on the test thread.

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;

use wing_blocks::filter::{FilterBlock, FilterParams, LcSlope};
use wing_blocks::gate::{Gate, GateParams, GateRatio, ScFilter, ScSource, SidechainParams};
use wing_blocks::input::{DelayMode, InputParams, InputStage, SourceMode};

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
fn process_paths_do_not_allocate() {
    let fs = 48_000.0;
    let mut input = InputStage::new();
    input.prepare(fs, 64);
    let mut filter = FilterBlock::new();
    filter.prepare(fs, 64);
    let mut gate = Gate::new();
    gate.prepare(fs, 64);
    let mut duck = Gate::new();
    duck.set_params(&GateParams::duck_default());
    duck.prepare(fs, 64);
    let mut l = [0.0f32; 64];
    let mut r = [0.0f32; 64];
    let key = [0.5f32; 64];

    let n = count_allocs(|| {
        for block in 0..64 {
            for (i, (a, b)) in l.iter_mut().zip(r.iter_mut()).enumerate() {
                *a = ((block * 64 + i) as f32 * 0.05).sin() * (block % 3) as f32 * 0.3;
                *b = *a * 0.5;
            }
            input.set_params(&InputParams {
                trim_db: block as f32 * 0.1,
                invert: block % 2 == 0,
                delay_on: block > 8,
                delay: 10.0 + block as f32,
                delay_mode: DelayMode::Ms,
                source_mode: SourceMode::Stereo,
                ..Default::default()
            });
            input.process(&mut l, &mut r);
            filter.set_params(&FilterParams {
                lc_on: block % 16 < 12,
                lc_freq: 20.0 + 10.0 * block as f32,
                lc_slope: if block % 2 == 0 {
                    LcSlope::Db24
                } else {
                    LcSlope::Db18
                },
                hc_on: true,
                hc_freq: 12_000.0 - 50.0 * block as f32,
                ..Default::default()
            });
            filter.process(&mut l, &mut r);
            gate.set_params(&GateParams {
                ratio: if block % 2 == 0 {
                    GateRatio::Gate
                } else {
                    GateRatio::R2
                },
                threshold_db: -30.0 - block as f32 * 0.2,
                ..GateParams::gate_default()
            });
            gate.set_sidechain(&SidechainParams {
                filter: ScFilter::Hp12,
                freq: 100.0 + block as f32,
                ..Default::default()
            });
            gate.process(&mut l, &mut r, None);
            duck.set_sidechain(&SidechainParams {
                source: ScSource::Channel(3),
                ..Default::default()
            });
            duck.process(&mut l, &mut r, Some((&key, &key)));
            let _ = gate.key_signal();
        }
    });
    assert_eq!(n, 0, "allocations on the process path: {n}");
    assert!(
        count_allocs(|| {
            std::hint::black_box(vec![1u8; 4]);
        }) > 0
    );
}
