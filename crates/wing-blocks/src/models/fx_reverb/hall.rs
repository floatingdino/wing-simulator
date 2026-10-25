//! `HALL` (HALL REVERB, `fx_pardef_hall_reverb`): the WING's own hall, a large smooth Lexicon/TC-style
//! algorithmic hall (models.md §6). 16-line FDN, long dense build-up, gentle modulation.
//!
//! | slot | key | range | mapping [I] |
//! |---|---|---|---|
//! | 1 | `pdel` | 0..200 ms | pre-delay |
//! | 2 | `size` | 2..100 | size scale 0.25..1 (lines 34..110 ms at 100, ER window) |
//! | 3 | `dcy` | 0.2..5 s | mid T60 |
//! | 4 | `mult` | 0.5..2 | LF T60 multiplier below 600 Hz |
//! | 5 | `damp` | 1..20 kHz | HF T60 = 0.3 x `dcy` above it |
//! | 6/7 | `lc`/`hc` | 20..400 / 200..20k Hz | wet output filters |
//! | 8 | `shp` | 0..250 | build-up shape `shp/250` |
//! | 9 | `sprd` | 0..50 | build-up spread `sprd/50` |
//! | 10 | `diff` | 1..30 | diffusion `(diff-1)/29` |
//! | 11 | `mspd` | 0..100 | LFO rate 0.05..2.5 Hz |

use super::core::{Character, CoreParams};
use super::{ps, ParamSpec, ReverbSpec, MAX_PARAMS};

pub const PARAMS: [ParamSpec; 11] = [
    ps("pdel", 0.0, 200.0, 50.0),
    ps("size", 2.0, 100.0, 50.0),
    ps("dcy", 0.2, 5.0, 1.57),
    ps("mult", 0.5, 2.0, 0.95),
    ps("damp", 1000.0, 20_000.0, 5700.0),
    ps("lc", 20.0, 400.0, 84.0),
    ps("hc", 200.0, 20_000.0, 7300.0),
    ps("shp", 0.0, 250.0, 50.0),
    ps("sprd", 0.0, 50.0, 25.0),
    ps("diff", 1.0, 30.0, 25.0),
    ps("mspd", 0.0, 100.0, 30.0),
];

pub const CHARACTER: Character = Character {
    line_ms: (34.0, 110.0),
    max_scale: 1.0,
    loop_ap_ms: (2.1, 7.9),
    loop_ap_gain: 0.6,
    diffuser_ms: [1.43, 2.61, 4.37, 6.83],
    diffuser_gain: 0.72,
    xover_hz: 600.0,
    hf_ratio: 0.3,
    mod_depth_ms: 0.25,
    er_window_ms: (8.0, 160.0),
    max_echo_s: 0.0,
    echo_feedback: false,
    tail_gain: 1.1,
};

fn map(r: &[f32; MAX_PARAMS]) -> CoreParams {
    CoreParams {
        pre_ms: r[0],
        scale: 0.25 + 0.75 * (r[1] - 2.0) / 98.0,
        decay_s: r[2],
        bass_mult: r[3],
        damp_hz: r[4],
        lc_hz: r[5],
        hc_hz: r[6],
        shape: r[7] / 250.0,
        spread: r[8] / 50.0,
        diffusion: (r[9] - 1.0) / 29.0,
        mod_rate_hz: 0.05 + 2.45 * r[10] / 100.0,
        mod_depth: 1.0,
        er_level: 0.35,
        tail_level: 1.0,
        ..CoreParams::default()
    }
}

pub static SPEC: ReverbSpec = ReverbSpec {
    id: "HALL",
    params: &PARAMS,
    character: CHARACTER,
    map,
};
