//! Scene catalog of the hardware verification kit: one `.snap` per assumption (or model) under test.
//!
//! Every scene is the factory scene plus a common base (channel 1 from USB 1 straight to MAIN 1, everything
//! else off, MAIN L on USB out 1, R on USB out 2) and the settings of the block under test, so the console
//! and the simulator process exactly the same signal path.

use super::suite::Kind;

/// What a scene tests.
#[derive(Clone, Debug)]
pub struct SceneDef {
    /// File stem (`<name>.snap`, recording `<name>.wav`).
    pub name: String,
    /// Area as in `docs/VERIFY-ON-HARDWARE.md`.
    pub area: &'static str,
    /// The assumption (short; the full wording is in the VERIFY table row of the same area).
    pub assumption: &'static str,
    /// `(parameter path, value)` after the base settings.
    pub sets: Vec<(String, String)>,
    /// Sections of the suite that are meaningful for this scene.
    pub kinds: Vec<Kind>,
    /// Part of the short run (`gen --tier quick`).
    pub quick: bool,
}

/// Base settings shared by every scene.
pub fn base_sets() -> Vec<(String, String)> {
    let b = |p: &str, v: &str| (p.to_string(), v.to_string());
    vec![
        b("ch/1/in/conn/grp", "USB"),
        b("ch/1/in/conn/in", "1"),
        b("ch/1/in/set/trim", "0"),
        b("ch/1/flt/lc", "false"),
        b("ch/1/flt/hc", "false"),
        b("ch/1/flt/tf", "false"),
        b("ch/1/eq/on", "false"),
        b("ch/1/peq/on", "false"),
        b("ch/1/gate/on", "false"),
        b("ch/1/dyn/on", "false"),
        b("ch/1/preins/on", "false"),
        b("ch/1/postins/on", "false"),
        b("ch/1/mute", "false"),
        b("ch/1/fdr", "0"),
        b("ch/1/pan", "0"),
        b("ch/1/main/1/on", "true"),
        b("ch/1/main/1/lvl", "0"),
        b("main/1/fdr", "0"),
        b("main/1/mute", "false"),
        b("main/1/eq/on", "false"),
        b("main/1/dyn/on", "false"),
        b("main/1/preins/on", "false"),
        b("main/1/postins/on", "false"),
        b("io/out/USB/1/grp", "MAIN"),
        b("io/out/USB/1/in", "1"),
        b("io/out/USB/2/grp", "MAIN"),
        b("io/out/USB/2/in", "2"),
    ]
}

fn sets(v: &[(&str, &str)]) -> Vec<(String, String)> {
    v.iter()
        .map(|(p, x)| (p.to_string(), x.to_string()))
        .collect()
}

fn scene(
    name: &str,
    area: &'static str,
    assumption: &'static str,
    kinds: &[Kind],
    quick: bool,
    s: Vec<(String, String)>,
) -> SceneDef {
    SceneDef {
        name: name.into(),
        area,
        assumption,
        sets: s,
        kinds: kinds.to_vec(),
        quick,
    }
}

fn dyn_scene(
    name: &str,
    id: &str,
    area: &'static str,
    assumption: &'static str,
    kinds: &[Kind],
    quick: bool,
) -> SceneDef {
    scene(
        name,
        area,
        assumption,
        kinds,
        quick,
        sets(&[("ch/1/dyn/on", "true"), ("ch/1/dyn/mdl", id)]),
    )
}

fn gate_scene(
    name: &str,
    id: &str,
    area: &'static str,
    assumption: &'static str,
    quick: bool,
) -> SceneDef {
    scene(
        name,
        area,
        assumption,
        &[Kind::Stairs, Kind::Steps, Kind::Bursts],
        quick,
        sets(&[("ch/1/gate/on", "true"), ("ch/1/gate/mdl", id)]),
    )
}

fn fx_scene(
    name: &str,
    id: &str,
    area: &'static str,
    assumption: &'static str,
    kinds: &[Kind],
    quick: bool,
) -> SceneDef {
    scene(
        name,
        area,
        assumption,
        kinds,
        quick,
        sets(&[
            ("fx/1/mdl", id),
            ("fx/1/fxmix", "100"),
            ("ch/1/preins/on", "true"),
            ("ch/1/preins/ins", "FX1"),
        ]),
    )
}

/// All scenes, in run order.
pub fn catalog() -> Vec<SceneDef> {
    use Kind::*;
    let ds = &[Stairs, Steps];
    let dst = &[Stairs, Steps, Tone];
    vec![
        scene(
            "00-thru",
            "Input",
            "unity path: USB level, 0 dBFS in = 0 dBFS out, pan law, latency (baseline for every other scene)",
            &[Fr, Stairs, Tone],
            true,
            vec![],
        ),
        scene(
            "01-filter",
            "Filter (exact port)",
            "channel LC 100 Hz / HC 8 kHz match the traced WING-Edit design (sanity check of the whole chain)",
            &[Fr],
            true,
            sets(&[
                ("ch/1/flt/lc", "true"),
                ("ch/1/flt/lcf", "100"),
                ("ch/1/flt/hc", "true"),
                ("ch/1/flt/hcf", "8000"),
            ]),
        ),
        scene(
            "02-eq-std",
            "EQ (exact port)",
            "STD EQ bands match the traced WING-Edit design (sanity check of the whole chain)",
            &[Fr],
            true,
            sets(&[
                ("ch/1/eq/on", "true"),
                ("ch/1/eq/mdl", "STD"),
                ("ch/1/eq/lg", "6"),
                ("ch/1/eq/lf", "120"),
                ("ch/1/eq/2g", "-8"),
                ("ch/1/eq/2f", "1000"),
                ("ch/1/eq/2q", "2"),
                ("ch/1/eq/hg", "5"),
                ("ch/1/eq/hf", "8000"),
            ]),
        ),
        scene(
            "03-eq-e88",
            "EQ alt",
            "E88 EQ is purely linear (no transformer / tube colour in WING's version)",
            &[Fr, Tone],
            false,
            sets(&[
                ("ch/1/eq/on", "true"),
                ("ch/1/eq/mdl", "E88"),
                ("ch/1/eq/lg", "3"),
                ("ch/1/eq/hg", "3"),
            ]),
        ),
        scene(
            "04-eq-soul",
            "EQ alt",
            "SOUL EQ is purely linear (no console colour in WING's version)",
            &[Fr, Tone],
            false,
            sets(&[
                ("ch/1/eq/on", "true"),
                ("ch/1/eq/mdl", "SOUL"),
                ("ch/1/eq/lg", "3"),
                ("ch/1/eq/hg", "3"),
            ]),
        ),
        gate_scene(
            "10-gate-std",
            "GATE",
            "Gate",
            "WING GATE static law (exact) and ACCENT/detector timing",
            false,
        ),
        gate_scene(
            "11-gate-e88",
            "E88",
            "Gate vintage",
            "E88: peak detector release 20 ms, 60 dB release law, 2 ms / 50 us attack, EXP end stop",
            true,
        ),
        gate_scene(
            "12-gate-9000g",
            "9000G",
            "Gate vintage",
            "9000G: fixed 2 dB hysteresis, 1.5 ms / 100 us attack, 40 dB release law",
            false,
        ),
        gate_scene(
            "13-gate-d241g",
            "D241G",
            "Gate vintage",
            "D241G: 2 dB hysteresis, 20 us attack, 80 dB depth, programme-dependent release",
            false,
        ),
        gate_scene(
            "14-gate-pse",
            "PSE",
            "PSE/CMB",
            "PSE: 6 dB knee, 2:1 expansion capped at depth, fast/slow timing, 10 ms RMS",
            false,
        ),
        dyn_scene(
            "20-comp-std",
            "COMP",
            "Comp (reference curve)",
            "WING COMP static law (exact) and attack/release timing",
            ds,
            true,
        ),
        dyn_scene(
            "21-comp-b160",
            "B160",
            "Comp VCA",
            "B160: 0 dBu = -18 dBFS thresholds, RMS detector",
            ds,
            false,
        ),
        dyn_scene(
            "22-comp-b560",
            "B560",
            "Comp VCA",
            "B560: dbx OverEasy knee 6 dB, thresholds at 0 dBu = -18 dBFS",
            ds,
            false,
        ),
        dyn_scene(
            "23-comp-9000c",
            "9000C",
            "Comp VCA",
            "9000C: RMS window 10 ms, 0 dBu = -18 dBFS",
            ds,
            false,
        ),
        dyn_scene(
            "24-comp-sbus",
            "SBUS",
            "Comp VCA",
            "SBUS: auto-release constants",
            ds,
            false,
        ),
        dyn_scene(
            "25-comp-d241c",
            "D241C",
            "Comp VCA",
            "D241C: peak detector, 10 ms auto attack",
            ds,
            false,
        ),
        dyn_scene(
            "26-comp-ecl33",
            "ECL33",
            "Comp console",
            "ECL33: threshold read as dBFS, feedback comp, 6 dB soft knee, colouration 0.1 % THD at 20 dB GR",
            dst,
            true,
        ),
        dyn_scene(
            "27-comp-e88c",
            "E88C",
            "Comp console",
            "E88C: SLOW attack 10 ms, 10 dB soft knee, 0 dBu = -18 dBFS",
            ds,
            false,
        ),
        dyn_scene(
            "28-comp-red3",
            "RED3",
            "Comp console",
            "RED3: 10 dB soft knee, peak detector, thresholds as dBFS",
            ds,
            false,
        ),
        dyn_scene(
            "29-comp-2250",
            "2250",
            "Comp console",
            "2250: threshold knob law, 10 dB soft knee, NEW RMS 1 ms",
            ds,
            false,
        ),
        dyn_scene(
            "2a-comp-lmt",
            "LMT",
            "Comp console",
            "LMT: 20:1, 0.3 ms attack / 150 ms release, transient shaper laws",
            ds,
            false,
        ),
        dyn_scene(
            "2b-comp-76la",
            "76LA",
            "Comp FET",
            "76LA: fixed -18 dBFS cell threshold, in/out knob scaling, 0.4 % THD at 9 dB GR",
            dst,
            true,
        ),
        dyn_scene(
            "2c-comp-nstr",
            "NSTR",
            "Comp FET",
            "NSTR: knee widths per ratio, normal-mode harmonics (LF-weighted 2nd + 3rd)",
            dst,
            false,
        ),
        dyn_scene(
            "2d-comp-la",
            "LA",
            "Comp opto",
            "LA: PEAK = side-chain drive, GAIN = output gain, two-part cell memory, tube colouration",
            dst,
            true,
        ),
        dyn_scene(
            "2e-comp-l100",
            "L100",
            "Comp opto",
            "L100: per-switch times, threshold 6 - 3.6 GR dBFS, knee 12 dB",
            dst,
            false,
        ),
        dyn_scene(
            "2f-comp-f670",
            "F670",
            "Comp vari-mu",
            "F670: 10 dB per threshold unit, THR map, fixed amp +10 dB, tube colouration",
            dst,
            true,
        ),
        dyn_scene(
            "30-comp-bliss",
            "BLISS",
            "Comp class A",
            "BLISS: peak detector, hard knee, anti-log ballistics",
            dst,
            false,
        ),
        dyn_scene(
            "31-comp-onec",
            "ONEC",
            "ONEC",
            "ONEC: `gr` knob mapping onto the COMP engine, auto make-up",
            ds,
            false,
        ),
        dyn_scene(
            "32-dyn-wave",
            "WAVE",
            "WAVE",
            "WAVE: differential envelope followers, attack/sustain weights",
            &[Steps, Bursts],
            true,
        ),
        dyn_scene(
            "33-dyn-ride",
            "RIDE",
            "RIDE",
            "RIDE: RMS level 100 ms, 2..200 dB/s slew, hold",
            &[Steps, Bursts],
            false,
        ),
        dyn_scene(
            "34-dyn-warm",
            "WARM",
            "WARM",
            "WARM: shaper law, ~1.4 % THD at -12 dBFS (default), colour tilt",
            &[Fr, Stairs, Tone],
            true,
        ),
        fx_scene(
            "35-fx-tape",
            "TAPE",
            "TAPE",
            "TAPE: saturator law, head bump, HF loss, 46 samples latency",
            &[Fr, Stairs, Tone],
            true,
        ),
        fx_scene(
            "36-fx-exciter",
            "EXCITER",
            "EXCITER",
            "EXCITER: tanh bias harmonics, TIMBRE shelf",
            &[Fr, Tone],
            true,
        ),
        fx_scene(
            "37-fx-enhance",
            "ENHANCE",
            "ENHANCE",
            "ENHANCE: band maps, harmonic generator shares",
            &[Fr, Tone],
            false,
        ),
        fx_scene(
            "38-fx-pbass",
            "P-BASS",
            "P-BASS",
            "P-BASS: LR4 split, harmonic law",
            &[Fr, Tone],
            false,
        ),
        fx_scene(
            "39-fx-ukrock",
            "UKROCK",
            "FX amps",
            "UKROCK: gain staging and tone stack",
            &[Fr, Stairs, Tone],
            false,
        ),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn names_unique_and_quick_tier_is_short() {
        let c = catalog();
        let mut n: Vec<&str> = c.iter().map(|s| s.name.as_str()).collect();
        n.sort();
        n.dedup();
        assert_eq!(n.len(), c.len());
        let q = c.iter().filter(|s| s.quick).count();
        assert!((10..=16).contains(&q), "{q}");
    }
}
