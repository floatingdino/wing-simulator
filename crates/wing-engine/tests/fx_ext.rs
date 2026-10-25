//! T05C: the EXT FX model in the full mix graph. The slot input leaves on the FX SEND leg (routable to USB
//! out, e.g. to a DAW insert) and the return comes back from the source `egrp/ein/emode` name (here USB In),
//! one block late like every re-input path, trimmed by `trim`.

mod common;

use common::rig::*;
use wing_engine::mix::InputSignal as I;
use wing_engine::routing::Side;
use wing_model::ParamStore;

fn plain_input(store: &ParamStore, p: &str) {
    for k in [
        "gate/on", "eq/on", "dyn/on", "peq/on", "flt/lc", "flt/hc", "flt/tf",
    ] {
        if store.lookup(&format!("{p}/{k}")).is_some() {
            seti(store, &format!("{p}/{k}"), 0);
        }
    }
}

fn setup(store: &ParamStore) {
    connect_usb(store, "ch/1", 1);
    plain_input(store, "ch/1");
    seti(store, "ch/1/preins/on", 1);
    sett(store, "ch/1/preins/ins", "FX1");
    sett(store, "fx/1/mdl", "EXT");
    setf(store, "fx/1/fxmix", 100.0);
}

fn peak(x: &[f32]) -> f32 {
    x.iter().fold(0.0f32, |a, v| a.max(v.abs()))
}

#[track_caller]
fn assert_close(a: &[f32], b: &[f32], tol: f32, what: &str) {
    let d = a
        .iter()
        .zip(b)
        .fold(0.0f32, |m, (x, y)| m.max((x - y).abs()));
    assert!(d <= tol, "{what}: max diff {d}");
}

/// Samples `[n0, n0 + BLOCK)` of external input `usb` (1-based) as the rig feeds them.
fn usb_block(usb: usize, n0: u64) -> Vec<f32> {
    (0..BLOCK as u64).map(|i| sig(usb - 1, n0 + i)).collect()
}

#[test]
fn ext_return_comes_from_the_configured_source_one_block_late() {
    let store = ParamStore::new();
    setup(&store);
    sett(&store, "fx/1/1", "USB"); // egrp
    seti(&store, "fx/1/2", 5); // ein: USB In 5 (+ 6)
    sett(&store, "fx/1/3", "ST"); // emode
    setf(&store, "fx/1/5", 6.0); // trim
    sett(&store, "io/out/USB/1/grp", "SEND");
    seti(&store, "io/out/USB/1/in", 1);
    let mut r = rig(&store, Default::default(), 2);
    let d = wing_engine::channel::ChannelStrip::channel(48_000.0, BLOCK).latency()
        + r.fx.slot_latency(1);
    let g = 10f32.powf(6.0 / 20.0);
    let feeds = [1u32, 5, 6];
    // history of the return legs (input of the cycle that just ran)
    let (mut hl, mut hr): (Vec<f32>, Vec<f32>) = (vec![], vec![]);
    for k in 0..30 {
        let t0 = r.t;
        r.cycle(&feeds);
        hl.extend(usb_block(5, t0));
        hr.extend(usb_block(6, t0));
        // SEND leg 1: the strip's input goes out
        assert!(peak(&r.port(r.nodes.send_port(1).unwrap())) > 0.01);
        if k >= 12 {
            let pfl_l = r.input(0, I::Pfl, Side::L);
            let pfl_r = r.input(0, I::Pfl, Side::R);
            // block k output = return of block k-1 (captured at the end of the previous cycle), at the
            // strip's constant latency d
            let delay = BLOCK + d;
            let end = hl.len();
            let want = |h: &[f32]| -> Vec<f32> {
                (end - BLOCK..end)
                    .map(|i| h.get(i.wrapping_sub(delay)).map_or(0.0, |v| v * g))
                    .collect()
            };
            assert_close(&pfl_l, &want(&hl), 1e-5, "PFL L = trim * return");
            assert_close(&pfl_r, &want(&hr), 1e-5, "PFL R = trim * return");
        }
    }
    // the dry feed (USB 1) is not heard: only the return
    assert!(peak(&r.input(0, I::Pfl, Side::L)) > 0.01);
}

#[test]
fn ext_mono_mode_and_off() {
    let store = ParamStore::new();
    setup(&store);
    sett(&store, "fx/1/1", "USB");
    seti(&store, "fx/1/2", 5);
    sett(&store, "fx/1/3", "M");
    let mut r = rig(&store, Default::default(), 1);
    r.cycles(20, &[1, 5, 6]);
    let (l, rr) = (r.input(0, I::Pfl, Side::L), r.input(0, I::Pfl, Side::R));
    assert_eq!(l, rr, "M: both sides carry USB In 5");
    assert!(peak(&l) > 0.01);

    // return group OFF: the slot passes the strip signal through
    sett(&store, "fx/1/1", "OFF");
    r.cycles(20, &[1, 5, 6]);
    let pfl = r.input(0, I::Pfl, Side::L);
    let t = r.t - BLOCK as u64;
    let d = wing_engine::channel::ChannelStrip::channel(48_000.0, BLOCK).latency()
        + r.fx.slot_latency(1);
    let want: Vec<f32> = (0..BLOCK as u64)
        .map(|i| (t + i).checked_sub(d as u64).map_or(0.0, |n| sig(0, n)))
        .collect();
    assert_close(&pfl, &want, 1e-5, "EXT OFF passes the input");

    // another model in the slot: the return is not routed
    sett(&store, "fx/1/mdl", "NONE");
    r.cycles(10, &[1, 5, 6]);
    assert!(peak(&r.input(0, I::Pfl, Side::L)) > 0.01);
}
