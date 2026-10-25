//! `AMBI` (AMBIENCE, `fx_pardef_ambience`): TC/Lexicon-style ambience (models.md §6): a short, dense
//! early-reflection field with a controllable tail, used to add space without an audible reverb. 8-line FDN
//! with short lines; the ER field is front-loaded and always present, `tail` scales the FDN tail.
//!
//! | slot | key | range | mapping [I] |
//! |---|---|---|---|
//! | 1 | `pdel` | 0..200 ms | pre-delay |
//! | 2 | `size` | 2..100 | size scale 0.3..1 (lines 10..42 ms at 100, ER window 2..40 ms) |
//! | 3 | `dcy` | 0.2..7.3 s | mid T60 (no bass multiplier: LF = mid) |
//! | 4 | `tail` | 0..100 | tail level `tail/100` |
//! | 5 | `damp` | 1..20 kHz | HF T60 = 0.35 x `dcy` above it |
//! | 6 | `diff` | 1..30 | diffusion `(diff-1)/29` |
//! | 7 | `mod` | 0..100 | LFO rate 0.05..2.5 Hz |
//! | 8/9 | `lc`/`hc` | Hz | wet output filters |

use super::core::{Character, CoreParams};
use super::{ps, ParamSpec, ReverbSpec, MAX_PARAMS};

pub const PARAMS: [ParamSpec; 9] = [
    ps("pdel", 0.0, 200.0, 4.0),
    ps("size", 2.0, 100.0, 60.0),
    ps("dcy", 0.2, 7.3, 0.84),
    ps("tail", 0.0, 100.0, 70.0),
    ps("damp", 1000.0, 20_000.0, 5000.0),
    ps("diff", 1.0, 30.0, 30.0),
    ps("mod", 0.0, 100.0, 20.0),
    ps("lc", 20.0, 400.0, 70.0),
    ps("hc", 200.0, 20_000.0, 8000.0),
];

pub const CHARACTER: Character = Character {
    line_ms: (10.0, 42.0),
    max_scale: 1.0,
    loop_ap_ms: (0.9, 3.7),
    loop_ap_gain: 0.65,
    diffuser_ms: [0.53, 1.07, 1.79, 2.71],
    diffuser_gain: 0.75,
    xover_hz: 500.0,
    hf_ratio: 0.35,
    mod_depth_ms: 0.08,
    er_window_ms: (2.0, 40.0),
    max_echo_s: 0.0,
    echo_feedback: false,
    tail_gain: 1.0,
};

fn map(r: &[f32; MAX_PARAMS]) -> CoreParams {
    CoreParams {
        pre_ms: r[0],
        scale: 0.3 + 0.7 * (r[1] - 2.0) / 98.0,
        decay_s: r[2],
        bass_mult: 1.0,
        tail_level: r[3] / 100.0,
        damp_hz: r[4],
        diffusion: (r[5] - 1.0) / 29.0,
        mod_rate_hz: 0.05 + 2.45 * r[6] / 100.0,
        lc_hz: r[7],
        hc_hz: r[8],
        shape: 0.1,
        spread: 1.0,
        mod_depth: 1.0,
        er_level: 0.8,
        ..CoreParams::default()
    }
}

pub static SPEC: ReverbSpec = ReverbSpec {
    id: "AMBI",
    params: &PARAMS,
    character: CHARACTER,
    map,
};
