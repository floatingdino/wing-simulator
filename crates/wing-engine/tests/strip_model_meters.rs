//! T082: every gate/dynamics model a channel's gate or dynamics slot can select drives that slot's meter
//! cells (GR, or the band cells of the dynamic EQs) under gain reduction.

use wing_engine::channel::{block, ChannelStrip, NoInserts, StripKeys};
use wing_engine::meters::{band, band_cell, strip_cell, word, LevelBank, StripMeter};
use wing_engine::params::{ParamMsg, TARGET_BASE};
use wing_model::Schema;

const FS: f64 = 48_000.0;
const N: usize = 64;

fn set(s: &mut ChannelStrip, block: u8, param: u16, value: f32) {
    s.apply(&ParamMsg::Set {
        block,
        param,
        value,
    });
}

/// Model parameters (0-based target, value) that make the model reduce gain at the test signals.
fn provoke(id: &str) -> &'static [(u16, f32)] {
    match id {
        "DS902" => &[(1, 12.0)],
        "DEQ" => &[(5, -12.0), (6, 1000.0)],
        "DEQ2" => &[(5, -12.0), (6, 1000.0), (14, -12.0), (15, 1000.0)],
        _ => &[],
    }
}

#[test]
fn every_gate_and_dyn_model_drives_its_meter_cells() {
    let s = Schema::get();
    let mut silent = Vec::new();
    for (blk, grp, gr_word, band_slot) in [
        (block::GATE, "ch/*/gate", word::GATE_GR, band::GATE),
        (block::DYN, "ch/*/dyn", word::DYN_GR, band::DYN),
    ] {
        let g = s.groups().iter().find(|g| g.key == grp).unwrap();
        for m in s.models(g) {
            let mut moved = false;
            // quiet (-70 dBFS: gates/expanders close), hot 1 kHz and hot 7 kHz (compressors, de-essers)
            for (amp, f) in [(3e-4f32, 1000.0f32), (0.9, 1000.0), (0.9, 7000.0)] {
                let bank = LevelBank::shared();
                let mut st = ChannelStrip::channel(FS, N);
                st.set_meter(Some(StripMeter::at(bank.clone(), 0)));
                st.apply(&ParamMsg::Model {
                    block: blk,
                    model: m.enum_index as u16,
                });
                for &(p, v) in provoke(m.name) {
                    set(&mut st, blk, TARGET_BASE + p, v);
                }
                set(&mut st, blk, 0, 1.0); // slot on
                st.apply(&ParamMsg::Commit);
                st.settle();
                let mut n = 0usize;
                let mut any = false;
                for _ in 0..150 {
                    let mut l: Vec<f32> = (0..N)
                        .map(|i| {
                            amp * (std::f32::consts::TAU * f * (n + i) as f32 / FS as f32).sin()
                        })
                        .collect();
                    let mut r = l.clone();
                    n += N;
                    st.process_with(&mut l, &mut r, StripKeys::default(), &mut NoInserts);
                    any |= bank.take(strip_cell(0, gr_word)) > 0.0
                        || bank.take(band_cell(0, band_slot)) != 0.0
                        || bank.take(band_cell(0, band_slot + 1)) != 0.0;
                }
                moved |= any;
            }
            if !moved {
                silent.push(format!("{grp}:{}", m.name));
            }
        }
    }
    // WAVE (transient shaper) and WARM (saturation) apply no gain reduction; WING-Edit's meters for them
    // are constant (1 / 0, `wing_blocks::meter`).
    assert_eq!(
        silent,
        [
            "ch/*/gate:WAVE",
            "ch/*/gate:WARM",
            "ch/*/dyn:WAVE",
            "ch/*/dyn:WARM"
        ]
    );
}
