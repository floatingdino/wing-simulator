//! `CONCERT` (CONCERT REVERB, `fx_pardef_lex_concert`): Lexicon-style concert hall (models.md §6): large,
//! warm, slow-building hall with discrete reflections and a chorused, "moving" tail. 16-line FDN; reflections
//! are two plain taps of the input (`rfl/rfr` 0..1200 ms) at fader levels `rfll/rflr`; `spin` = LFO rate,
//! `depth` = LFO depth; `crs` = chorus of the tail output (two 7 +- 3 ms modulated delays).
//!
//! | slot | key | range | mapping [I] |
//! |---|---|---|---|
//! | 1 | `pdel` | 0..200 ms | pre-delay |
//! | 2 | `size` | 20..76 m | size scale `size/50` (lines 40..130 ms at 50 m) |
//! | 3 | `dcy` | 0.3..29 s | mid T60 |
//! | 4 | `mult` | 0.25..4 | LF T60 multiplier below 400 Hz |
//! | 5 | `damp` | 1..20 kHz | HF T60 = 0.3 x `dcy` above it |
//! | 6/7 | `lc`/`hc` | Hz | wet output filters |
//! | 8 | `shp` | 0..50 | build-up shape `shp/50` |
//! | 9 | `sprd` | 0..50 | build-up spread `sprd/50` |
//! | 10 | `diff` | 0..100 | diffusion `/100` |
//! | 11 | `depth` | 1..16 | LFO depth `depth/8` x 0.2 ms |
//! | 12/13 | `rfl`/`rfr` | 0..1200 ms | reflection delay from the input |
//! | 14/15 | `rfll`/`rflr` | fader dB | reflection level (half of it also into the tail) |
//! | 16 | `spin` | 0..100 | LFO rate 0.05..2.0 Hz |
//! | 17 | `crs` | 0..100 | tail chorus amount `/100` |

use super::core::{Character, CoreParams, Echo};
use super::{fader_gain, ps, ParamSpec, ReverbSpec, MAX_PARAMS};

pub const PARAMS: [ParamSpec; 17] = [
    ps("pdel", 0.0, 200.0, 40.0),
    ps("size", 20.0, 76.0, 50.0),
    ps("dcy", 0.3, 29.0, 2.05),
    ps("mult", 0.25, 4.0, 1.21),
    ps("damp", 1000.0, 20_000.0, 5700.0),
    ps("lc", 20.0, 400.0, 84.0),
    ps("hc", 200.0, 20_000.0, 7300.0),
    ps("shp", 0.0, 50.0, 15.0),
    ps("sprd", 0.0, 50.0, 25.0),
    ps("diff", 0.0, 100.0, 25.0),
    ps("depth", 1.0, 16.0, 12.0),
    ps("rfl", 0.0, 1200.0, 270.0),
    ps("rfr", 0.0, 1200.0, 330.0),
    ps("rfll", -144.0, 10.0, -20.0),
    ps("rflr", -144.0, 10.0, -20.0),
    ps("spin", 0.0, 100.0, 30.0),
    ps("crs", 0.0, 100.0, 10.0),
];

pub const CHARACTER: Character = Character {
    line_ms: (40.0, 130.0),
    max_scale: 76.0 / 50.0,
    loop_ap_ms: (2.3, 8.9),
    loop_ap_gain: 0.6,
    diffuser_ms: [1.61, 2.93, 4.79, 7.37],
    diffuser_gain: 0.7,
    xover_hz: 400.0,
    hf_ratio: 0.3,
    mod_depth_ms: 0.2,
    er_window_ms: (10.0, 180.0),
    max_echo_s: 1.2,
    echo_feedback: false,
    tail_gain: 1.1,
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
        shape: r[7] / 50.0,
        spread: r[8] / 50.0,
        diffusion: r[9] / 100.0,
        mod_depth: r[10] / 8.0,
        echo: [
            Echo {
                time_ms: r[11],
                feedback: 0.0,
                level: fader_gain(r[13]),
            },
            Echo {
                time_ms: r[12],
                feedback: 0.0,
                level: fader_gain(r[14]),
            },
        ],
        mod_rate_hz: 0.05 + 1.95 * r[15] / 100.0,
        chorus: r[16] / 100.0,
        er_level: 0.3,
        tail_level: 1.0,
        echo_to_tail: 0.5,
    }
}

pub static SPEC: ReverbSpec = ReverbSpec {
    id: "CONCERT",
    params: &PARAMS,
    character: CHARACTER,
    map,
};
