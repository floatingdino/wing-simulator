//! Model-slot introspection (T088): the daemon's `probe` control command and `/wingsim/probe` OSC.
//!
//! `probe ch/21/gate` (or `ch/21` for every model slot of the strip; `aux/N/eq|dyn`) arms the engine's
//! [`SlotProbeFeed`], waits for the live audio to fill one capture (0.34 s at 48 kHz) per slot and answers
//! a one-line JSON array, one object per slot:
//!
//! ```text
//! {"target":"ch/21/gate","model":"WARM","enum":12,"implemented":true,
//!  "wrapper":{"on":true,"mix":5.0,"gain":0.0,"wet":1.0,"owns_mix":true},"latency":57,
//!  "params":[{"slot":1,"key":"drv","dsp":125.0,"store":"125"}, ...],
//!  "in_rms_dbfs":-23.5,"in_peak_dbfs":-10.7,"out_rms_dbfs":-23.4,"gain_db":0.1,"residual_pct":37.1,"residual_db":-8.6}
//! ```
//!
//! `dsp` is the value the model instance holds (what the DSP runs with), `store` the parameter tree's value
//! (what WING-Edit shows); `wet` is the dry/wet weight the slot applies (1.0 = block mix inert, T085).
//! `residual_pct` is the nonlinear part of the slot output ([`wing_engine::probe::analyse`]: THD+N on a
//! sine, a conservative distortion figure on programme); `null` measurements = no signal at the slot.

use std::time::{Duration, Instant};

use serde_json::{json, Value};
use wing_engine::probe::{analyse, ProbeBlock, SlotProbeFeed};
use wing_model::ParamStore;

/// How long a capture may take before the probe gives up (no audio session / driver stopped).
pub const PROBE_TIMEOUT: Duration = Duration::from_secs(3);

/// `(strip prefix, flat strip index, slots)` of a probe target `ch/N[/gate|eq|dyn]` / `aux/N[/eq|dyn]`.
pub fn parse_target(t: &str) -> Result<(String, usize, Vec<ProbeBlock>), String> {
    let parts: Vec<&str> = t.trim().trim_matches('/').split('/').collect();
    let bad = || format!("bad probe target {t:?}: use ch/1..40[/gate|eq|dyn] or aux/1..8[/eq|dyn]");
    let (kind, n) = match parts.as_slice() {
        [k, n] | [k, n, _] => (*k, n.parse::<usize>().map_err(|_| bad())?),
        _ => return Err(bad()),
    };
    let (index, all) = match kind {
        "ch" if (1..=40).contains(&n) => (
            n - 1,
            vec![ProbeBlock::Gate, ProbeBlock::Eq, ProbeBlock::Dyn],
        ),
        "aux" if (1..=8).contains(&n) => (40 + n - 1, vec![ProbeBlock::Eq, ProbeBlock::Dyn]),
        _ => return Err(bad()),
    };
    let blocks = match parts.get(2) {
        None => all,
        Some(b) => {
            let b = ProbeBlock::parse(b).ok_or_else(bad)?;
            if !all.contains(&b) {
                return Err(bad());
            }
            vec![b]
        }
    };
    Ok((format!("{kind}/{n}"), index, blocks))
}

fn num(x: f64) -> Value {
    if x.is_finite() {
        json!((x * 100.0).round() / 100.0)
    } else {
        Value::Null
    }
}

/// Capture and describe one slot (blocks the caller for up to [`PROBE_TIMEOUT`]).
pub fn probe_slot(
    feed: &SlotProbeFeed,
    store: &ParamStore,
    prefix: &str,
    index: usize,
    block: ProbeBlock,
) -> Result<Value, String> {
    let target = format!("{prefix}/{}", block.name());
    feed.arm(index, block);
    let t0 = Instant::now();
    let cap = loop {
        if let Some(c) = feed.take() {
            break c;
        }
        if t0.elapsed() > PROBE_TIMEOUT {
            let n = feed.captured();
            feed.disarm();
            return Err(format!(
                "{target}: no capture after {PROBE_TIMEOUT:?} ({n} samples; is an audio session running?)"
            ));
        }
        std::thread::sleep(Duration::from_millis(5));
    };
    let schema = store.schema();
    let group_key = format!(
        "{}/*/{}",
        prefix.split('/').next().unwrap_or("ch"),
        block.name()
    );
    let group = schema.groups().iter().find(|g| g.key == group_key);
    let model = cap.slot.model_index.and_then(|e| {
        group.and_then(|g| schema.models(g).iter().find(|m| m.enum_index == e as u32))
    });
    let mut params = Vec::new();
    if let Some(m) = model {
        for (target_child, d) in schema.model_params(m) {
            let Ok(slot) = target_child.parse::<usize>() else {
                continue;
            };
            if d.unused_slot {
                continue;
            }
            let dsp = cap.slot.params.get(slot - 1).copied().flatten();
            let mut txt = String::new();
            let store_v = store
                .lookup(&format!("{target}/{slot}"))
                .and_then(|id| store.format_token(id, &mut txt).ok().map(|_| txt));
            params.push(json!({
                "slot": slot,
                "key": d.name,
                "dsp": dsp.map(|v| num(v as f64)),
                "store": store_v,
            }));
        }
    }
    let a = analyse(&cap.input, &cap.output, cap.sample_rate.max(1) as f64);
    let s = &cap.slot;
    Ok(json!({
        "target": target,
        "model": model.map(|m| m.name),
        "enum": s.model_index,
        "implemented": s.has_instance,
        "wrapper": {
            "on": s.on,
            "mix": num(s.mix_pct as f64),
            "gain": num(s.gain_db as f64),
            "wet": num(s.wet as f64),
            "owns_mix": s.owns_mix,
        },
        "latency": s.latency,
        "params": params,
        "in_rms_dbfs": a.map(|a| num(a.in_rms_db)),
        "in_peak_dbfs": a.map(|a| num(a.in_peak_db)),
        "out_rms_dbfs": a.map(|a| num(a.out_rms_db)),
        "gain_db": a.map(|a| num(a.gain_db)),
        "residual_pct": a.map(|a| num(a.residual_pct)),
        "residual_db": a.map(|a| num(a.residual_db)),
        "seconds": num(cap.input.len() as f64 / cap.sample_rate.max(1) as f64),
    }))
}

/// Run a probe request (`target` as in [`parse_target`]): a one-line JSON array, one object per slot.
pub fn run(
    feed: Option<&SlotProbeFeed>,
    store: &ParamStore,
    target: &str,
) -> Result<String, String> {
    let feed = feed.ok_or("audio is off (--no-audio): nothing to probe")?;
    let (prefix, index, blocks) = parse_target(target)?;
    let mut out = Vec::new();
    for b in blocks {
        out.push(probe_slot(feed, store, &prefix, index, b)?);
    }
    Ok(Value::Array(out).to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn targets() {
        let (p, i, b) = parse_target("ch/21/gate").unwrap();
        assert_eq!((p.as_str(), i, b), ("ch/21", 20, vec![ProbeBlock::Gate]));
        assert_eq!(parse_target("/ch/1").unwrap().2.len(), 3);
        assert_eq!(parse_target("aux/8/dyn").unwrap().1, 47);
        assert_eq!(
            parse_target("aux/2").unwrap().2,
            vec![ProbeBlock::Eq, ProbeBlock::Dyn]
        );
        for bad in [
            "ch/41/gate",
            "ch/0",
            "aux/1/gate",
            "bus/1/dyn",
            "ch/x",
            "ch/1/flt",
            "",
        ] {
            assert!(parse_target(bad).is_err(), "{bad}");
        }
    }

    #[test]
    fn without_audio_is_an_error() {
        let s = ParamStore::new();
        assert!(run(None, &s, "ch/1/gate")
            .unwrap_err()
            .contains("--no-audio"));
        // armed but never fed: times out (short-circuited by a target error first)
        assert!(run(Some(&SlotProbeFeed::new()), &s, "ch/99").is_err());
    }
}
