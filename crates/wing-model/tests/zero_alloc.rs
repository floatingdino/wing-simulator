//! get/set/lookup/queue paths must not allocate (counting global allocator, per thread).

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;

use wing_model::{ParamStore, StackStr, Value};

struct Counting;

thread_local! {
    static ALLOCS: Cell<usize> = const { Cell::new(0) };
}

unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, l: Layout) -> *mut u8 {
        let _ = ALLOCS.try_with(|c| c.set(c.get() + 1));
        unsafe { System.alloc(l) }
    }
    unsafe fn dealloc(&self, p: *mut u8, l: Layout) {
        unsafe { System.dealloc(p, l) }
    }
    unsafe fn realloc(&self, p: *mut u8, l: Layout, n: usize) -> *mut u8 {
        let _ = ALLOCS.try_with(|c| c.set(c.get() + 1));
        unsafe { System.realloc(p, l, n) }
    }
}

#[global_allocator]
static A: Counting = Counting;

fn allocs() -> usize {
    ALLOCS.with(|c| c.get())
}

#[test]
fn get_set_lookup_are_allocation_free() {
    let st = ParamStore::new();
    let mut rt = st.take_rt_consumer().unwrap();
    let fdr = st.lookup("ch/1/fdr").unwrap();
    let mdl = st.lookup("ch/1/dyn/mdl").unwrap();
    let name = st.lookup("ch/1/name").unwrap();
    let h = st.schema().hash(fdr);
    let mut buf = StackStr::<64>::new();

    // std's Mutex on macOS boxes its pthread mutex on first lock: warm up once, then measure.
    let mut before = 0;
    for i in 0..1001 {
        if i == 1 {
            before = allocs();
        }
        let v = -60.0 + (i % 70) as f32;
        st.set(fdr, Value::Float(v)).unwrap();
        assert_eq!(st.get(fdr), Some(Value::Float(v)));
        st.set_norm(fdr, 0.5, 1).unwrap();
        assert_eq!(st.by_hash(h), Some(fdr));
        assert_eq!(st.lookup("/ch/1/fdr"), Some(fdr));
        let thr = st.lookup("ch/1/dyn/thr");
        if i % 2 == 0 {
            st.set_enum_name(mdl, "GATE", 0).unwrap(); // model switch
        } else {
            st.set_enum_name(mdl, "COMP", 0).unwrap();
            assert!(thr.is_some());
        }
        st.set_text(fdr, "-12.5", 0).unwrap();
        st.set_str(name, if i % 2 == 0 { "Kick" } else { "Snare" }, 0)
            .unwrap();
        buf.clear();
        st.format(fdr, &mut buf).unwrap();
        while rt.pop().is_some() {}
    }
    assert_eq!(allocs() - before, 0, "allocations on get/set paths");
    assert_eq!(buf.as_str(), "-12.5 dB");
}
