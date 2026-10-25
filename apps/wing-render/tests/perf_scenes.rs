//! T070 perf-suite scenes: the "everything on" scene loads and runs every heavy block, and the
//! parallel scheduler (with and without the LPT claim order) computes every buffer bit-identically
//! to the serial schedule.

use wing_render::perf::{console, output_digest, Scene, HEAVY_FX, LPT};
use wing_sim::audio::console::Console;

const CYCLES: usize = 12;

fn build(scene: Scene, workers: usize) -> Console {
    let mut store = scene.store().expect("scene");
    console(&mut store, 64, workers, false).expect("console")
}

#[test]
fn everything_scene_loads_heaviest_models_and_owns_every_fx_slot() {
    let store = Scene::Everything.store().unwrap();
    let get = |p: &str| {
        let id = store.lookup(p).unwrap_or_else(|| panic!("{p}"));
        let mut s = String::new();
        store.format_token(id, &mut s).unwrap();
        s
    };
    for (k, m) in HEAVY_FX.iter().enumerate() {
        assert_eq!(&get(&format!("fx/{}/mdl", k + 1)), m);
        assert_eq!(
            get(&format!("ch/{}/preins/ins", k + 1)),
            format!("FX{}", k + 1)
        );
    }
    assert_eq!(get("ch/40/dyn/mdl"), "76LA");
    assert_eq!(get("ch/40/gate/mdl"), "NSTR");
    assert_eq!(get("mtx/8/eq/mdl"), "PIA");
    assert_eq!(get("bus/16/dly/on"), "1");
}

#[test]
fn parallel_schedules_are_bit_identical_to_serial() {
    for scene in [Scene::Everything, Scene::Public] {
        let serial = output_digest(&mut build(scene, 0), CYCLES);
        let par = output_digest(&mut build(scene, 3), CYCLES);
        assert_eq!(serial, par, "{scene:?}: 3 workers, LPT");
        LPT.store(false, std::sync::atomic::Ordering::Relaxed);
        let plain = output_digest(&mut build(scene, 3), CYCLES);
        LPT.store(true, std::sync::atomic::Ordering::Relaxed);
        assert_eq!(serial, plain, "{scene:?}: 3 workers, schedule order");
    }
}
