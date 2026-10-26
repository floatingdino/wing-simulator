//! T088 regression: WARM (SOUL WARMTH PRE) in the gate slot of the user's VOX channel, measured on the live
//! path with the model-slot probe (`wing_engine::probe`, the daemon's `probe` command).
//!
//! The whole console as the daemon builds it ([`build_console_with`], the same `Console` the realtime path
//! runs), ch 21 set up as in the user's scene (`scratch/live-mix-v0.6.snap` / the autosaved `state.snap`:
//! LCL 2 with the USB 34 alternate source active, process order EDIG, COMP in the dyn slot, WARM in the
//! gate slot with WING-Edit's upgraded block `mix` = `wmix` = 5 %), then the live session's sets: DRIVE 125 %,
//! MIX (`wmix`) 100 %. A vocal-like programme (180 Hz glottal source with vibrato, vowel formants, syllable
//! envelope) at -24 dBFS RMS on USB 34. The probe captures the gate slot's own input and output.

use std::sync::Arc;

use wing_engine::probe::{analyse, ProbeBlock, SlotAnalysis, SlotProbeFeed, SlotState};
use wing_model::ParamStore;
use wing_render::build_console_with;
use wing_sim::audio::console::ConsoleConfig;

const FS: f64 = 48_000.0;

fn set(store: &ParamStore, path: &str, v: &str) {
    let id = store
        .lookup(path)
        .unwrap_or_else(|| panic!("no such node {path}"));
    store
        .set_text(id, v, 0)
        .unwrap_or_else(|e| panic!("{path} = {v}: {e:?}"));
}

/// Vocal-like programme at `rms_db` dBFS RMS: harmonics of a 180 Hz source (5.5 Hz vibrato), -12 dB/oct
/// above 500 Hz with formant bumps at 700 / 1200 / 2600 Hz, 4 Hz syllable envelope.
pub fn vocal(n: usize, rms_db: f64) -> Vec<f32> {
    use std::f64::consts::PI;
    let mut ph = 0.0f64;
    let mut x: Vec<f64> = (0..n)
        .map(|i| {
            let t = i as f64 / FS;
            let f0 = 180.0 * (1.0 + 0.03 * (2.0 * PI * 5.5 * t).sin());
            ph += f0 / FS;
            let mut s = 0.0;
            for h in 1..=30 {
                let f = h as f64 * f0;
                let tilt = 1.0 / (1.0 + (f / 500.0).powi(2));
                let fm = |fc: f64, bw: f64| 1.0 / (1.0 + ((f - fc) / bw).powi(2));
                let a = tilt
                    * (0.3 + fm(700.0, 150.0) + 0.6 * fm(1200.0, 200.0) + 0.5 * fm(2600.0, 300.0));
                s += a * (2.0 * PI * h as f64 * ph).sin();
            }
            s * (0.5 + 0.5 * (2.0 * PI * 4.0 * t).sin().max(-0.6))
        })
        .collect();
    let rms = (x.iter().map(|v| v * v).sum::<f64>() / n as f64).sqrt();
    let g = 10f64.powf(rms_db / 20.0) / rms;
    x.iter_mut().for_each(|v| *v *= g);
    x.into_iter().map(|v| v as f32).collect()
}

/// The user's ch 21 (VOX2) strip as saved, plus `extra` sets.
fn user_ch21(extra: &[(&str, &str)]) -> ParamStore {
    let st = ParamStore::new();
    for (p, v) in [
        ("ch/21/in/conn/grp", "LCL"),
        ("ch/21/in/conn/in", "2"),
        ("ch/21/in/conn/altgrp", "USB"),
        ("ch/21/in/conn/altin", "34"),
        ("ch/21/in/set/altsrc", "1"),
        ("ch/21/proc", "EDIG"),
        ("ch/21/dyn/mdl", "COMP"),
        ("ch/21/dyn/on", "1"),
        ("ch/21/dyn/gain", "3.5"),
        ("ch/21/dyn/thr", "-23"),
        ("ch/21/dyn/ratio", "2.5"),
        ("ch/21/gate/mdl", "WARM"),
        ("ch/21/gate/on", "1"),
        ("ch/21/gate/mix", "5"),
        ("ch/21/gate/gain", "0"),
        ("ch/21/gate/1", "100"),
        ("ch/21/gate/2", "100"),
        ("ch/21/gate/3", "0.1"),
        ("ch/21/gate/4", "0"),
        ("ch/21/gate/5", "5"),
    ]
    .iter()
    .chain(extra)
    {
        set(&st, p, v);
    }
    st
}

/// Run `x` into USB In 34 and capture ch 21's gate slot.
fn probe_gate(store: ParamStore, x: &[f32]) -> (SlotState, SlotAnalysis) {
    let feed = Arc::new(SlotProbeFeed::new());
    let (mut c, _store) = build_console_with(
        store,
        ConsoleConfig {
            workers: 0,
            probe: Some(feed.clone()),
            ..ConsoleConfig::default()
        },
    )
    .unwrap();
    // let the compressor and the parameter smoothing settle for 0.1 s before capturing
    for (k, blk) in x.chunks(64).enumerate() {
        if k == 75 {
            feed.arm(20, ProbeBlock::Gate);
        }
        c.cycle(
            |ch, buf| {
                if ch == 33 {
                    buf.copy_from_slice(blk)
                } else {
                    buf.fill(0.0)
                }
            },
            |_, _| {},
        );
        if feed.is_done() {
            break;
        }
    }
    let cap = feed.take().expect("probe capture completed");
    let a = analyse(&cap.input, &cap.output, FS).expect("signal at the gate slot");
    (cap.slot, a)
}

#[test]
fn warm_max_drive_is_audible_on_vocal_at_minus_24_dbfs() {
    let x = vocal(48_000, -24.0);
    // the live session: DRIVE 125 %, MIX 100 %, at both HARMONICS ends and centre
    for hrm in ["100", "0", "-100"] {
        let (slot, a) = probe_gate(
            user_ch21(&[
                ("ch/21/gate/1", "125"),
                ("ch/21/gate/5", "100"),
                ("ch/21/gate/2", hrm),
            ]),
            &x,
        );
        println!("hrm {hrm}: {slot:?}\n  {a:?}");
        // the path: WARM (enum 12) running fully wet although the block mix is 5 % (T085), values as set
        assert_eq!(slot.model_index, Some(12));
        assert!(slot.has_instance && slot.on && slot.owns_mix);
        assert_eq!(slot.mix_pct, 5.0);
        assert_eq!(slot.wet, 1.0);
        assert_eq!(slot.params[0], Some(125.0));
        assert_eq!(slot.params[1], Some(hrm.parse().unwrap()));
        assert_eq!(slot.params[4], Some(100.0));
        // the law: clearly audible distortion at a realistic vocal level, level kept. Before T088 (drive
        // gain max 6, small-signal normalisation only): residual 15.4 / 14.9 / 13.0 % (hrm 100 / 0 / -100)
        // and the level fell by 2.8 / 2.4 / 1.4 dB; now ~35-37 % at +-0.2 dB
        assert!(
            (-24.5..-23.0).contains(&a.in_rms_db),
            "programme reaches the gate slot at {:.1} dBFS",
            a.in_rms_db
        );
        assert!(a.residual_pct > 25.0, "hrm {hrm}: {a:?}");
        assert!(a.gain_db.abs() < 1.0, "hrm {hrm}: {a:?}");
    }
    // the saved state (MIX 5 %) and a low drive at MIX 100 % stay subtle
    let (_, a) = probe_gate(user_ch21(&[]), &x);
    assert!(a.residual_pct < 1.5, "saved state: {a:?}");
    let (_, a) = probe_gate(
        user_ch21(&[("ch/21/gate/1", "10"), ("ch/21/gate/5", "100")]),
        &x,
    );
    assert!(a.residual_pct < 1.5, "drive 10 %: {a:?}");
}
