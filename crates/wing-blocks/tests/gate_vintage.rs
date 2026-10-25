//! T047: vintage gates (E88, 9000G, D241G) through the model factory: WING-Edit GR-meter mapping
//! (`data/reference-curves/dyn-models.json`, skipped when absent), telemetry, and zero allocation after
//! `prepare` (counting global allocator on the test thread).

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;

use wing_blocks::fidelity::{json, reference_dir, run_mono};
use wing_blocks::model::{create_model, BlockKind, ParamValue};
use wing_blocks::models::gate_vintage::{gr_meter, VintageGateKind};

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
fn gr_meter_matches_reference_file() {
    let path = reference_dir().join("dyn-models.json");
    let Ok(text) = std::fs::read_to_string(&path) else {
        eprintln!("SKIP: {} not present", path.display());
        return;
    };
    let root = json::parse(&text).expect("dyn-models.json parses");
    let models = root.get("models").and_then(|m| m.as_arr()).unwrap();
    for kind in VintageGateKind::ALL {
        let entry = models
            .iter()
            .find(|m| m.get("model").and_then(|v| v.as_str()) == Some(kind.id()))
            .unwrap_or_else(|| panic!("{} missing", kind.id()));
        let meter = entry.get("gr_meter").unwrap();
        let nums = |k: &str| -> Vec<f64> {
            meter
                .get(k)
                .and_then(|v| v.as_arr())
                .unwrap()
                .iter()
                .map(|v| v.as_f64().unwrap())
                .collect()
        };
        for (gr, m) in nums("gain_reduction_db").into_iter().zip(nums("meter")) {
            let ours = gr_meter(kind, gr as f32) as f64;
            assert!(
                (ours - m).abs() <= 1e-4 + 1e-3 * m.abs(),
                "{}: GR {gr} dB -> {ours} vs {m}",
                kind.id()
            );
        }
    }
}

#[test]
fn factory_creates_real_models_with_telemetry() {
    for kind in VintageGateKind::ALL {
        let mut m = create_model(BlockKind::Dynamics, kind.id());
        assert!(!m.is_stub(), "{}", kind.id());
        assert_eq!(m.id(), kind.id());
        m.prepare(48_000.0, 64);
        // Quiet input: gate closed, GR > 0 and equal to the applied attenuation.
        let x = vec![0.001f32; 48_000];
        let y = run_mono(&mut *m, &x);
        let tel = m.telemetry();
        assert!(!tel.engaged && tel.gain_reduction_db > 1.0, "{}", kind.id());
        let last = y[y.len() - 1];
        if last > 0.0 {
            let applied = -20.0 * (last / 0.001).log10();
            assert!(
                (tel.gain_reduction_db - applied).abs() < 0.05,
                "{}",
                kind.id()
            );
        }
        // Loud input opens it.
        let y = run_mono(&mut *m, &vec![0.5f32; 48_000]);
        assert!(m.telemetry().engaged && (y[y.len() - 1] - 0.5).abs() < 1e-3);
    }
}

#[test]
fn vintage_gates_do_not_allocate() {
    let mut models: Vec<_> = VintageGateKind::ALL
        .iter()
        .map(|k| {
            let mut m = create_model(BlockKind::Dynamics, k.id());
            m.prepare(48_000.0, 64);
            m
        })
        .collect();
    let key = [0.3f32; 64];
    let n = count_allocs(|| {
        let (mut l, mut r) = ([0.5f32; 64], [0.4f32; 64]);
        for m in models.iter_mut() {
            for s in 1..=8 {
                m.set_param(s, ParamValue::Float(1.0));
                m.set_param(s, ParamValue::Enum(1));
            }
            m.commit();
            m.process(&mut l, &mut r, None);
            m.process(&mut l, &mut r, Some((&key, &key)));
            let _ = m.telemetry();
            m.load_defaults();
            m.reset();
            m.process(&mut l, &mut r, None);
        }
    });
    assert_eq!(n, 0);
}
