//! `CHAMBER` (CHAMBER REVERB, `fx_pardef_lex_chamber`): Lexicon-style chamber (models.md §6): an echo chamber's
//! fast, very dense build-up and even decay. 8-line FDN with strong in-loop diffusion; echoes are two plain
//! delayed taps of the input (`ecl/ecr` 0..300 ms) at fader levels `ell/elr`, heard and fed into the tail.
//!
//! | slot | key | range | mapping [I] |
//! |---|---|---|---|
//! | 1 | `pdel` | 0..200 ms | pre-delay |
//! | 2 | `size` | 4..76 m | size scale `size/50` (lines 26..90 ms at 50 m) |
//! | 3 | `dcy` | 0.3..25 s | mid T60 |
//! | 4 | `mult` | 0.25..4 | LF T60 multiplier below 500 Hz |
//! | 5 | `damp` | 1..20 kHz | HF T60 = 0.4 x `dcy` above it |
//! | 6/7 | `lc`/`hc` | Hz | wet output filters |
//! | 8 | `shp` | 0..250 | build-up shape `shp/250` |
//! | 9 | `sprd` | 0..50 | build-up spread `sprd/50` |
//! | 10 | `diff` | 0..100 | diffusion `/100` |
//! | 11 | `spin` | 0..100 | LFO rate 0.05..2.0 Hz |
//! | 12/13 | `ecl`/`ecr` | 0..300 ms | echo delay from the input |
//! | 14/15 | `ell`/`elr` | fader dB | echo level (half of it also into the tail) |

use super::core::{Character, CoreParams, Echo};
use super::{fader_gain, ps, ParamSpec, ReverbSpec, MAX_PARAMS};

pub const PARAMS: [ParamSpec; 15] = [
    ps("pdel", 0.0, 200.0, 30.0),
    ps("size", 4.0, 76.0, 25.0),
    ps("dcy", 0.3, 25.0, 1.35),
    ps("mult", 0.25, 4.0, 1.15),
    ps("damp", 1000.0, 20_000.0, 5000.0),
    ps("lc", 20.0, 400.0, 121.0),
    ps("hc", 200.0, 20_000.0, 5000.0),
    ps("shp", 0.0, 250.0, 100.0),
    ps("sprd", 0.0, 50.0, 10.0),
    ps("diff", 0.0, 100.0, 50.0),
    ps("spin", 0.0, 100.0, 30.0),
    ps("ecl", 0.0, 300.0, 91.0),
    ps("ecr", 0.0, 300.0, 115.0),
    ps("ell", -144.0, 10.0, -25.0),
    ps("elr", -144.0, 10.0, -25.0),
];

pub const CHARACTER: Character = Character {
    line_ms: (26.0, 90.0),
    max_scale: 76.0 / 50.0,
    loop_ap_ms: (1.7, 6.7),
    loop_ap_gain: 0.7,
    diffuser_ms: [1.13, 2.29, 3.53, 5.41],
    diffuser_gain: 0.75,
    xover_hz: 500.0,
    hf_ratio: 0.4,
    mod_depth_ms: 0.1,
    er_window_ms: (3.0, 60.0),
    max_echo_s: 0.3,
    echo_feedback: false,
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
        er_level: 0.4,
        tail_level: 1.0,
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
        echo_to_tail: 0.5,
        ..CoreParams::default()
    }
}

pub static SPEC: ReverbSpec = ReverbSpec {
    id: "CHAMBER",
    params: &PARAMS,
    character: CHARACTER,
    map,
};
