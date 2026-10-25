//! T03A: the RT side of the parameter pipeline never allocates: `apply` while a 30k-change scene
//! recall drains, model swaps with crossfades, processing and telemetry publishing. A counting
//! allocator armed only around RT calls on this thread (control-side sets are not counted).

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;

use wing_engine::params::strip::{bind_channel, ParamStrip};
use wing_engine::params::{ParamMapBuilder, ParamPipeline, PipelineConfig, ProcessorAccess};
use wing_engine::{NodeId, Processor};
use wing_model::{ParamStore, ParamType, Value};

struct Counting;

thread_local! {
    static ARMED: Cell<bool> = const { Cell::new(false) };
    static COUNT: Cell<usize> = const { Cell::new(0) };
}

fn note() {
    let _ = ARMED.try_with(|a| {
        if a.get() {
            COUNT.with(|c| c.set(c.get() + 1));
        }
    });
}

unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, l: Layout) -> *mut u8 {
        note();
        System.alloc(l)
    }
    unsafe fn alloc_zeroed(&self, l: Layout) -> *mut u8 {
        note();
        System.alloc_zeroed(l)
    }
    unsafe fn realloc(&self, p: *mut u8, l: Layout, n: usize) -> *mut u8 {
        note();
        System.realloc(p, l, n)
    }
    unsafe fn dealloc(&self, p: *mut u8, l: Layout) {
        System.dealloc(p, l)
    }
}

#[global_allocator]
static GLOBAL: Counting = Counting;

fn rt<R>(f: impl FnOnce() -> R) -> (R, usize) {
    COUNT.with(|c| c.set(0));
    ARMED.with(|a| a.set(true));
    let r = f();
    ARMED.with(|a| a.set(false));
    (r, COUNT.with(|c| c.get()))
}

struct Strips(Vec<ParamStrip>);

impl ProcessorAccess for Strips {
    fn processor(&mut self, id: NodeId) -> &mut dyn Processor {
        &mut self.0[id.index()]
    }
}

#[test]
fn rt_paths_do_not_allocate() {
    const CH: usize = 40;
    let st = ParamStore::new();
    let mut b = ParamMapBuilder::new();
    for ch in 0..CH {
        bind_channel(&mut b, ch as u32 + 1, NodeId(ch as u32)).unwrap();
    }
    let mut strips = Strips((0..CH).map(|_| ParamStrip::new(48_000.0, 64)).collect());
    let mut pipe = ParamPipeline::new(&st, b.build(), PipelineConfig::default()).unwrap();
    let mut rx = pipe.telemetry_rx().unwrap();
    let ((), n) = rt(|| pipe.sync_all(&st, &mut strips));
    assert_eq!(n, 0, "sync_all allocated");

    // a full-tree recall (control side, not counted)
    let s = st.schema();
    let mut k = 0u32;
    for id in s.ids() {
        let Some(d) = st.desc(id) else { continue };
        if d.ty == ParamType::Str || d.unused_slot || s.is_live(id) {
            continue;
        }
        k = k.wrapping_mul(1_103_515_245).wrapping_add(12_345);
        let _ = st.set_norm(id, (k >> 8) as f32 / (1u32 << 24) as f32, 0);
    }
    for ch in 1..=CH {
        st.set(
            st.lookup(&format!("ch/{ch}/dyn/on")).unwrap(),
            Value::Int(1),
        )
        .unwrap();
        st.set(
            st.lookup(&format!("ch/{ch}/fdr")).unwrap(),
            Value::Float(0.0),
        )
        .unwrap();
    }

    let models = ["GATE", "COMP", "EXP", "DUCK", "LA", "COMP"];
    let (mut l, mut r) = (vec![0.0f32; 64], vec![0.0f32; 64]);
    let mut total = 0;
    for cycle in 1..=600u64 {
        if cycle % 50 == 0 {
            // model swaps on every channel (control side)
            let m = models[(cycle / 50) as usize % models.len()];
            for ch in 1..=CH {
                st.set_enum_name(st.lookup(&format!("ch/{ch}/dyn/mdl")).unwrap(), m, 0)
                    .unwrap();
                st.set_enum_name(st.lookup(&format!("ch/{ch}/gate/mdl")).unwrap(), m, 0)
                    .unwrap();
            }
        }
        let ((), n) = rt(|| {
            let rep = pipe.apply(&mut strips);
            total += rep.changes;
            for (c, s) in strips.0.iter_mut().enumerate() {
                for (i, v) in l.iter_mut().enumerate() {
                    *v = ((cycle as usize * 64 + i + c) as f32 * 0.07).sin() * 0.5;
                }
                r.copy_from_slice(&l);
                s.process_stereo(&mut l, &mut r);
            }
            pipe.publish_telemetry(&mut strips, cycle);
            while rx.pop().is_some() {}
        });
        assert_eq!(n, 0, "RT allocation in cycle {cycle}");
    }
    assert!(total >= 25_000, "{total}");
}
