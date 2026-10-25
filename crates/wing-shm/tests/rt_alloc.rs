//! The daemon's per-cycle API must not allocate (counting global allocator).

use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering::Relaxed};
use std::time::Duration;

use wing_shm::sim::DriverSim;
use wing_shm::{DaemonLink, OpenOptions, Segment, WakeMode, CHANNELS};

struct Counting;
static COUNTING: AtomicBool = AtomicBool::new(false);
static ALLOCS: AtomicUsize = AtomicUsize::new(0);

unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, l: Layout) -> *mut u8 {
        if COUNTING.load(Relaxed) {
            ALLOCS.fetch_add(1, Relaxed);
        }
        System.alloc(l)
    }
    unsafe fn dealloc(&self, p: *mut u8, l: Layout) {
        System.dealloc(p, l)
    }
}

#[global_allocator]
static GLOBAL: Counting = Counting;

#[test]
fn daemon_cycle_does_not_allocate() {
    let name = format!("/wst.alloc.{}", std::process::id());
    let _ = Segment::unlink(&name);
    let create = OpenOptions {
        create: true,
        ..Default::default()
    };
    let drv = DriverSim::new(Segment::open(&name, &create).unwrap(), 48_000);
    let mut link = DaemonLink::attach(&name, &OpenOptions::default(), WakeMode::Auto).unwrap();
    let out = vec![0.25f32; 128 * CHANNELS];
    let mut inp = vec![0.0f32; 128 * CHANNELS];
    let mut buf = vec![0.0f32; 512];

    for pass in 0..2 {
        // Pass 0 warms up lazily initialised statics (timebase, dlsym).
        COUNTING.store(pass == 1, Relaxed);
        for k in 0..100i64 {
            drv.read_input(k * 128 - 128, &mut inp, 128, CHANNELS);
            drv.write_output(k * 128 + 128, &out, 128, CHANNELS);
            link.heartbeat();
            link.wait(Duration::from_millis(1));
            while let Some(b) = link.next_block(buf.len()) {
                let x = &mut buf[..b.frames];
                for ch in 0..CHANNELS {
                    link.read_input(&b, ch, x);
                    link.write_output(&b, ch, x);
                }
                link.commit(&b);
            }
            let _ = link.stats();
        }
        COUNTING.store(false, Relaxed);
    }
    assert_eq!(ALLOCS.load(Relaxed), 0, "RT path allocated");
    drop(link);
    let _ = Segment::unlink(&name);
}
