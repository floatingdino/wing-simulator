//! Plate reverbs (T05P): `PLATE`, `V-PLATE`, `BPLATE` (models.md §6) on the shared FDN core ([`super::core`]).
//!
//! A mechanical plate (EMT 140: a 1 x 2 m steel sheet driven by one moving-coil exciter, two contact pickups)
//! has no discrete early reflections: bending waves are dispersive and the plate's modal density is high and
//! nearly constant over frequency, so the echo density reaches its maximum within a few ms and the tail is
//! bright and smooth. Dattorro's plate (JAES 1997, after the Griesinger/Lexicon topology) models this with
//! strong input allpass diffusion and short, allpass-loaded recirculating delays. Here the same character
//! comes from the T05R core configured as a plate: 16 short FDN lines (9..60 ms), Dattorro-length input
//! diffusers at high gain, strong in-loop allpasses, an ER window of only 0.3..15 ms at low level (the
//! "attack" of the build-up, no room reflections), and a high `hf_ratio` (bright HF decay).
//!
//! ## `PLATE` (PLATE REVERB, `fx_pardef_lex_plate`): Lexicon-style plate (480L / PCM "Plate" family)
//!
//! Behaves as `ROOM` with `att` instead of `shp` (models.md) including the recirculating pre-echoes.
//!
//! | slot | key | range | mapping [I] |
//! |---|---|---|---|
//! | 1 | `pdel` | 0..200 ms | pre-delay |
//! | 2 | `size` | 4..76 m | size scale `(size+16)/56` (0.36..1.64; lines 9..33 ms at the default 20 m) |
//! | 3 | `dcy` | 0.3..25 s | mid T60 |
//! | 4 | `mult` | 0.25..4 | LF T60 multiplier below 600 Hz |
//! | 5 | `damp` | 1..20 kHz | HF T60 = 0.5 x `dcy` above it (plates are bright) |
//! | 6/7 | `lc`/`hc` | Hz | wet output filters |
//! | 8 | `att` | 0..100 | sharpness of the onset: build-up shape `0.7 (1 - att/100)` (100 = explosive, 0 = soft swell) |
//! | 9 | `sprd` | 0..50 | build-up duration `sprd/50` (window 0.5..15 ms x size) |
//! | 10 | `diff` | 0..100 | diffusion `/100` |
//! | 11 | `spin` | 0..100 | LFO rate 0.05..2.0 Hz |
//! | 12/13 | `ecl`/`ecr` | 0..1200 ms | pre-echo delay |
//! | 14/15 | `efl`/`efr` | -100..100 % | pre-echo recirculation (clamped to +-98 %) |
//!
//! Echo level is fixed at 0.3 with half fed into the tail, as `ROOM` [I].
//!
//! ## `V-PLATE` (VINTAGE PLATE, `fx_pardef_vintage_plate`): EMT 140 (X32 "Vintage Plate")
//!
//! Fixed plate geometry (size scale 1, lines 10..41 ms), almost no modulation (a steel plate does not
//! chorus), full diffusion, flat bass decay (the EMT damper acts broadband). The four WING controls:
//!
//! | slot | key | range | mapping [I] |
//! |---|---|---|---|
//! | 1 | `pdel` | 0..250 ms | pre-delay (the core's pre-delay range was raised to 250 ms for this) |
//! | 2 | `dcy` | 1..6 s | mid T60 (the EMT damper plate's 1..5+ s range) |
//! | 3 | `lc` | 20..400 Hz | wet low cut |
//! | 4 | `col` | -20..20 | colour tilt: damping `5 kHz x 2^(col/10)` (1.25..20 kHz, HF T60 0.4 x `dcy` above) and hi cut `8 kHz x 2^(col/10)` (2..20 kHz); default +8 = bright |
//!
//! ## `BPLATE` (BLUE PLATE, `fx_pardef_plate_reverb`)
//!
//! Hardware reference unknown (models.md §8 item 6; a web search finds only WING usage guides). Its parameter
//! set is the `HALL` family's (WING's own TC-style algorithms: size 2..100, decay 0.2..5 s, bass mult 0.5..2)
//! plus an explicit crossover and modulation depth, so it is designed here as a distinct **modern modulated
//! plate**: longer lines than `PLATE` (16..60 ms at size 100), deeper chorus-like line modulation, a user
//! crossover, softer HF damping. The "blue" (cool, smooth) character comes from the modulation and
//! `hf_ratio` 0.45 [I].
//!
//! | slot | key | range | mapping [I] |
//! |---|---|---|---|
//! | 1 | `pdel` | 0..200 ms | pre-delay |
//! | 2 | `size` | 2..100 | size scale 0.25..1 |
//! | 3 | `dcy` | 0.2..5 s | mid T60 |
//! | 4 | `mult` | 0.5..2 | LF T60 multiplier below `xover` |
//! | 5 | `damp` | 1..20 kHz | HF T60 = 0.45 x `dcy` above it |
//! | 6/7 | `lc`/`hc` | Hz | wet output filters |
//! | 8 | `xover` | 20..500 Hz | bass-multiplier crossover (core `xover_hz`, added for this model) |
//! | 9 | `mdep` | 1..50 | LFO depth `mdep/20` x 0.35 ms (0.02..0.9 ms, limited to half the shortest line) |
//! | 10 | `mspd` | 0..100 | LFO rate 0.05..2.5 Hz |
//! | 11 | `diff` | 1..30 | diffusion `(diff-1)/29` |

use super::core::{Character, CoreParams, Echo};
use super::{ps, ParamSpec, ReverbSpec, MAX_PARAMS};

/// Dattorro's input diffuser lengths (142, 107, 379, 277 samples at 29.761 kHz) in ms.
const DATTORRO_DIFF_MS: [f32; 4] = [4.77, 3.60, 12.73, 9.31];

// ---------------------------------------------------------------- PLATE

pub const PLATE_PARAMS: [ParamSpec; 15] = [
    ps("pdel", 0.0, 200.0, 30.0),
    ps("size", 4.0, 76.0, 20.0),
    ps("dcy", 0.3, 25.0, 1.61),
    ps("mult", 0.25, 4.0, 1.2),
    ps("damp", 1000.0, 20_000.0, 9700.0),
    ps("lc", 20.0, 400.0, 105.0),
    ps("hc", 200.0, 20_000.0, 9500.0),
    ps("att", 0.0, 100.0, 60.0),
    ps("sprd", 0.0, 50.0, 40.0),
    ps("diff", 0.0, 100.0, 60.0),
    ps("spin", 0.0, 100.0, 30.0),
    ps("ecl", 0.0, 1200.0, 230.0),
    ps("ecr", 0.0, 1200.0, 200.0),
    ps("efl", -100.0, 100.0, 15.0),
    ps("efr", -100.0, 100.0, 15.0),
];

/// PLATE pre-echo output level (fixed, as ROOM).
pub const PLATE_ECHO_LEVEL: f32 = 0.3;

pub const PLATE_CHARACTER: Character = Character {
    line_ms: (14.0, 52.0),
    max_scale: 92.0 / 56.0,
    loop_ap_ms: (1.1, 4.3),
    loop_ap_gain: 0.7,
    diffuser_ms: DATTORRO_DIFF_MS,
    diffuser_gain: 0.72,
    xover_hz: 600.0,
    hf_ratio: 0.5,
    mod_depth_ms: 0.15,
    er_window_ms: (0.5, 15.0),
    max_echo_s: 1.2,
    echo_feedback: true,
    tail_gain: 1.0,
};

fn plate_map(r: &[f32; MAX_PARAMS]) -> CoreParams {
    CoreParams {
        pre_ms: r[0],
        scale: (r[1] + 16.0) / 56.0,
        decay_s: r[2],
        bass_mult: r[3],
        damp_hz: r[4],
        lc_hz: r[5],
        hc_hz: r[6],
        shape: 0.7 * (1.0 - r[7] / 100.0),
        spread: r[8] / 50.0,
        diffusion: r[9] / 100.0,
        mod_rate_hz: 0.05 + 1.95 * r[10] / 100.0,
        mod_depth: 1.0,
        er_level: 0.15,
        tail_level: 1.0,
        echo: [
            Echo {
                time_ms: r[11],
                feedback: r[13] / 100.0,
                level: PLATE_ECHO_LEVEL,
            },
            Echo {
                time_ms: r[12],
                feedback: r[14] / 100.0,
                level: PLATE_ECHO_LEVEL,
            },
        ],
        echo_to_tail: 0.5,
        ..CoreParams::default()
    }
}

pub static PLATE_SPEC: ReverbSpec = ReverbSpec {
    id: "PLATE",
    params: &PLATE_PARAMS,
    character: PLATE_CHARACTER,
    map: plate_map,
};

// ---------------------------------------------------------------- V-PLATE

pub const VPLATE_PARAMS: [ParamSpec; 4] = [
    ps("pdel", 0.0, 250.0, 20.0),
    ps("dcy", 1.0, 6.0, 2.0),
    ps("lc", 20.0, 400.0, 80.0),
    ps("col", -20.0, 20.0, 8.0),
];

pub const VPLATE_CHARACTER: Character = Character {
    line_ms: (10.0, 41.0),
    max_scale: 1.0,
    loop_ap_ms: (0.9, 3.7),
    loop_ap_gain: 0.72,
    diffuser_ms: DATTORRO_DIFF_MS,
    diffuser_gain: 0.75,
    xover_hz: 400.0,
    hf_ratio: 0.4,
    mod_depth_ms: 0.03,
    er_window_ms: (0.3, 3.0),
    max_echo_s: 0.0,
    echo_feedback: false,
    tail_gain: 1.0,
};

/// V-PLATE colour to (damping Hz, hi cut Hz).
pub fn vplate_colour(col: f32) -> (f32, f32) {
    let k = 2f32.powf(col / 10.0);
    ((5000.0 * k).min(20_000.0), (8000.0 * k).min(20_000.0))
}

fn vplate_map(r: &[f32; MAX_PARAMS]) -> CoreParams {
    let (damp, hc) = vplate_colour(r[3]);
    CoreParams {
        pre_ms: r[0],
        scale: 1.0,
        decay_s: r[1],
        bass_mult: 1.0,
        damp_hz: damp,
        lc_hz: r[2],
        hc_hz: hc,
        shape: 0.2,
        spread: 0.5,
        diffusion: 1.0,
        mod_rate_hz: 0.3,
        mod_depth: 1.0,
        er_level: 0.1,
        tail_level: 1.0,
        ..CoreParams::default()
    }
}

pub static VPLATE_SPEC: ReverbSpec = ReverbSpec {
    id: "V-PLATE",
    params: &VPLATE_PARAMS,
    character: VPLATE_CHARACTER,
    map: vplate_map,
};

// ---------------------------------------------------------------- BPLATE

pub const BPLATE_PARAMS: [ParamSpec; 11] = [
    ps("pdel", 0.0, 200.0, 32.0),
    ps("size", 2.0, 100.0, 40.0),
    ps("dcy", 0.2, 5.0, 1.62),
    ps("mult", 0.5, 2.0, 1.1),
    ps("damp", 1000.0, 20_000.0, 6500.0),
    ps("lc", 20.0, 400.0, 60.0),
    ps("hc", 200.0, 20_000.0, 7200.0),
    ps("xover", 20.0, 500.0, 217.0),
    ps("mdep", 1.0, 50.0, 20.0),
    ps("mspd", 0.0, 100.0, 20.0),
    ps("diff", 1.0, 30.0, 25.0),
];

pub const BPLATE_CHARACTER: Character = Character {
    line_ms: (16.0, 60.0),
    max_scale: 1.0,
    loop_ap_ms: (1.3, 5.1),
    loop_ap_gain: 0.65,
    diffuser_ms: [1.13, 2.71, 6.11, 8.93],
    diffuser_gain: 0.75,
    xover_hz: 217.0,
    hf_ratio: 0.45,
    mod_depth_ms: 0.35,
    er_window_ms: (0.5, 10.0),
    max_echo_s: 0.0,
    echo_feedback: false,
    tail_gain: 1.0,
};

fn bplate_map(r: &[f32; MAX_PARAMS]) -> CoreParams {
    CoreParams {
        pre_ms: r[0],
        scale: 0.25 + 0.75 * (r[1] - 2.0) / 98.0,
        decay_s: r[2],
        bass_mult: r[3],
        damp_hz: r[4],
        lc_hz: r[5],
        hc_hz: r[6],
        xover_hz: r[7],
        mod_depth: r[8] / 20.0,
        mod_rate_hz: 0.05 + 2.45 * r[9] / 100.0,
        diffusion: (r[10] - 1.0) / 29.0,
        shape: 0.2,
        spread: 0.3,
        er_level: 0.1,
        tail_level: 1.0,
        ..CoreParams::default()
    }
}

pub static BPLATE_SPEC: ReverbSpec = ReverbSpec {
    id: "BPLATE",
    params: &BPLATE_PARAMS,
    character: BPLATE_CHARACTER,
    map: bplate_map,
};
