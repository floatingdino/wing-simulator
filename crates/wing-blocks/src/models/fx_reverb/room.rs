//! `ROOM` (ROOM REVERB, `fx_pardef_lex_room`): Lexicon 480L/224-family room (models.md §6). 8-line FDN with a
//! short, strongly diffused early field and the Lexicon "pre-echoes": two recirculating echo lines
//! (`ecl/ecr` time, `efl/efr` feedback, negative = phase-inverted) that are heard and also feed the tail.
//!
//! | slot | key | range | mapping [I] |
//! |---|---|---|---|
//! | 1 | `pdel` | 0..200 ms | pre-delay |
//! | 2 | `size` | 4..76 m | size scale `size/50` (lines 30..105 ms at 50 m) |
//! | 3 | `dcy` | 0.3..25 s | mid T60 |
//! | 4 | `mult` | 0.25..4 | LF T60 multiplier below 700 Hz |
//! | 5 | `damp` | 1..20 kHz | HF T60 = 0.35 x `dcy` above it |
//! | 6/7 | `lc`/`hc` | Hz | wet output filters |
//! | 8 | `shp` | 0..250 | build-up shape `shp/250` |
//! | 9 | `sprd` | 0..50 | build-up spread `sprd/50` |
//! | 10 | `diff` | 0..100 | diffusion `/100` |
//! | 11 | `spin` | 0..100 | LFO rate 0.05..2.0 Hz |
//! | 12/13 | `ecl`/`ecr` | 0..1200 ms | echo delay from the input |
//! | 14/15 | `efl`/`efr` | -100..100 % | echo recirculation (clamped to +-98 %) |
//!
//! Echo output level is fixed at 0.3 (-10.5 dB) with half of it fed into the tail [I] (no level key exists).

use super::core::{Character, CoreParams, Echo};
use super::{ps, ParamSpec, ReverbSpec, MAX_PARAMS};

pub const PARAMS: [ParamSpec; 15] = [
    ps("pdel", 0.0, 200.0, 40.0),
    ps("size", 4.0, 76.0, 50.0),
    ps("dcy", 0.3, 25.0, 2.51),
    ps("mult", 0.25, 4.0, 1.21),
    ps("damp", 1000.0, 20_000.0, 5700.0),
    ps("lc", 20.0, 400.0, 84.0),
    ps("hc", 200.0, 20_000.0, 7300.0),
    ps("shp", 0.0, 250.0, 0.0),
    ps("sprd", 0.0, 50.0, 25.0),
    ps("diff", 0.0, 100.0, 40.0),
    ps("spin", 0.0, 100.0, 30.0),
    ps("ecl", 0.0, 1200.0, 250.0),
    ps("ecr", 0.0, 1200.0, 350.0),
    ps("efl", -100.0, 100.0, 20.0),
    ps("efr", -100.0, 100.0, 20.0),
];

/// Echo output level (fixed).
pub const ECHO_LEVEL: f32 = 0.3;

pub const CHARACTER: Character = Character {
    line_ms: (30.0, 105.0),
    max_scale: 76.0 / 50.0,
    loop_ap_ms: (1.3, 5.3),
    loop_ap_gain: 0.65,
    diffuser_ms: [0.97, 1.83, 3.11, 4.73],
    diffuser_gain: 0.75,
    xover_hz: 700.0,
    hf_ratio: 0.35,
    mod_depth_ms: 0.12,
    er_window_ms: (3.0, 70.0),
    max_echo_s: 1.2,
    echo_feedback: true,
    tail_gain: 1.0,
};

fn map(r: &[f32; MAX_PARAMS]) -> CoreParams {
    CoreParams {
        pre_ms: r[0],
        scale: r[1] / 50.0,
        decay_s: r[2],
        bass_mult: r[3],
        damp_hz: r[4],
        lc_hz: r[5],
        hc_hz: r[6],
        shape: r[7] / 250.0,
        spread: r[8] / 50.0,
        diffusion: r[9] / 100.0,
        mod_rate_hz: 0.05 + 1.95 * r[10] / 100.0,
        mod_depth: 1.0,
        er_level: 0.5,
        tail_level: 1.0,
        echo: [
            Echo {
                time_ms: r[11],
                feedback: r[13] / 100.0,
                level: ECHO_LEVEL,
            },
            Echo {
                time_ms: r[12],
                feedback: r[14] / 100.0,
                level: ECHO_LEVEL,
            },
        ],
        echo_to_tail: 0.5,
        ..CoreParams::default()
    }
}

pub static SPEC: ReverbSpec = ReverbSpec {
    id: "ROOM",
    params: &PARAMS,
    character: CHARACTER,
    map,
};
