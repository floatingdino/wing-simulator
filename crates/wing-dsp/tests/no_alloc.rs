//! Zero-allocation check: every primitive's per-sample / per-block API must not touch the heap after
//! construction. A counting global allocator records allocations made on the test thread while armed.

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;

use wing_dsp::biquad::{CutKind, SmoothedBiquad};
use wing_dsp::detector::{DetectMode, ReleaseMode};
use wing_dsp::gain::{db_to_lin_fast, lin_to_db_fast, FaderGainTable};
use wing_dsp::svf::SvfMode;
use wing_dsp::*;

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
    // ---- construction (allocation allowed) ----
    let mut bq = Biquad::new(Coeffs::peak(fs, 1000.0, 1.0, 6.0));
    let mut bq64 = BiquadF64::new(Coeffs::highpass(fs, 20.0, 0.707));
    let mut sbq = SmoothedBiquad::new(Coeffs::IDENTITY, 64);
    let mut cas = Cascade::butterworth(CutKind::Highpass, fs, 80.0, 8);
    let mut bank = BiquadBank::new(48, 4);
    let mut svf = Svf::new(SvfMode::Lowpass, fs, 1000.0, 0.707, 0.0);
    let mut sm = OnePoleSmoother::new(0.0, 0.01, fs);
    let mut ramp = LinearRamp::new(0.0, 64);
    let mut env = EnvelopeFollower::new(
        DetectMode::MeanSquare,
        0.001,
        0.1,
        ReleaseMode::ProgramDependent {
            charge_tau_s: 0.3,
            slow_tau_s: 1.0,
        },
        fs,
    );
    let mut peak = PeakDetector::new(0.3, fs);
    let mut rms = RmsWindow::new(4800, 2400);
    let mut dl = DelayLine::new(4800);
    let mut ap = AllpassReader::default();
    let mut os2 = Oversampler2x::new(64);
    let mut os4 = Oversampler4x::new(64);
    let table = FaderGainTable::fader();
    let mut planar: Vec<Vec<f32>> = (0..48).map(|_| vec![0.1; 64]).collect();
    let mut interleaved = vec![0.1f32; 48 * 64];
    let mut buf = [0.0f32; 64];
    let mut out = [0.0f32; 64];
    let mut hi2 = [0.0f32; 128];
    let mut hi4 = [0.0f32; 256];

    // ---- process (no allocation allowed) ----
    let n = count_allocs(|| {
        let _guard = DenormalGuard::new();
        for block in 0..8 {
            for (i, x) in buf.iter_mut().enumerate() {
                *x = ((block * 64 + i) as f32 * 0.05).sin();
            }
            bq.process_block(&mut buf);
            bq64.process_block(&mut buf);
            sbq.set_target(Coeffs::low_shelf(fs, 200.0 + block as f64, 0.7, 3.0));
            sbq.process_block(&mut buf);
            cas.set_sections(&biquad::butterworth(CutKind::Lowpass, fs, 15_000.0, 4));
            cas.process_block(&mut buf);
            bank.set_coeffs(block, block % 4, Coeffs::peak(fs, 500.0, 2.0, 3.0));
            {
                let mut it = planar.iter_mut();
                let mut refs: [&mut [f32]; 48] =
                    std::array::from_fn(|_| it.next().unwrap().as_mut_slice());
                bank.process_planar(&mut refs);
            }
            bank.process_interleaved(&mut interleaved);
            bank.flush_denormals();
            svf.set(SvfMode::Bell, fs, 1000.0 + block as f64, 1.0, 6.0);
            svf.process_block(&mut buf);
            sm.set_target(block as f32);
            sm.fill(&mut out);
            sm.advance(64);
            ramp.set_target(-(block as f32));
            ramp.fill(&mut out);
            env.process_block(&buf, &mut out);
            peak.process_block(&buf);
            rms.process_block(&buf);
            for &x in &buf {
                dl.push(x);
                let _ = dl.read_linear(100.5)
                    + dl.read_lagrange3(200.25)
                    + dl.read_allpass(&mut ap, 300.7);
                let _ = db_to_lin_fast(lin_to_db_fast(x.abs() + 1e-3)) + table.lookup(x * 10.0);
            }
            os2.process(&mut buf, |hi| hi.iter_mut().for_each(|v| *v = v.tanh()));
            os4.process(&mut buf, |hi| hi.iter_mut().for_each(|v| *v = v.tanh()));
            os2.upsample(&buf, &mut hi2);
            os2.downsample(&hi2, &mut out);
            os4.upsample(&buf, &mut hi4);
            os4.downsample(&hi4, &mut out);
        }
    });
    assert_eq!(n, 0, "allocations on the process path: {n}");
    // Self-check: the counter does see allocations.
    assert!(
        count_allocs(|| {
            std::hint::black_box(vec![1u8; 4]);
        }) > 0
    );
    assert!(buf.iter().all(|x| x.is_finite()));
}
