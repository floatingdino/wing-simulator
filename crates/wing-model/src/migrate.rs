//! Firmware 1.x -> 3.x snapshot key migration (T019).
//!
//! Applied by [`crate::Snapshot::load`] to files whose `creator_fw` major version is < 3 (or,
//! without `creator_fw`, whose `type` is older than `snapshot.11`). Every rule has an id that is
//! documented with its rationale in docs/research/snapshot-migration.md; each touched key gets a
//! [`MigrationNote`] (`lossy` when the 1.x value cannot be represented in 3.x).
//!
//! The rules work on the flattened leaf list before anything is applied to the store, so the
//! loader's normal ordering (selectors first) also covers keys the migration adds.

use std::collections::HashMap;

use crate::json::Json;
use crate::snapshot::{Leaf, MigrationNote};

fn note(notes: &mut Vec<MigrationNote>, from: &str, to: &str, rule: &'static str, lossy: bool) {
    notes.push(MigrationNote {
        from: from.to_string(),
        to: to.to_string(),
        rule,
        lossy,
    });
}

fn leaf(path: String, src: &str, value: Json) -> Leaf {
    Leaf {
        path,
        src: src.to_string(),
        value,
    }
}

fn s(v: &str) -> Json {
    Json::Str(v.to_string())
}

fn is_num(v: &Json, x: f64) -> bool {
    v.as_f64().is_some_and(|f| (f - x).abs() < 1e-6)
}

/// 1.x `dynxo/type` tokens (band + slope in one token) -> 3.x `type` (`OFF LO HI S6 S12 PEAK`).
fn dynxo_type(v: &str) -> Option<&'static str> {
    Some(match v {
        "OFF" => "OFF",
        "LO6" | "LO12" => "LO",
        "HI6" | "HI12" => "HI",
        "PC" => "PEAK",
        _ => return None,
    })
}

/// Rewrite 1.x leaves into the 3.x schema; records a note for every key it touches.
pub(crate) fn migrate_1x(leaves: Vec<Leaf>, notes: &mut Vec<MigrationNote>) -> Vec<Leaf> {
    let idx: HashMap<&str, usize> = leaves
        .iter()
        .enumerate()
        .map(|(i, l)| (l.path.as_str(), i))
        .collect();
    let val = |p: &str| idx.get(p).map(|&i| &leaves[i].value);
    let mut out: Vec<Leaf> = Vec::with_capacity(leaves.len() + 64);

    for l in &leaves {
        let seg: Vec<&str> = l.path.split('/').collect();
        let p = l.path.as_str();
        match seg.as_slice() {
            // R1 send-ind: "independent" = the send uses its own `mode` instead of the bus's
            // `busmode`; 3.x stores the effective mode per send (see R10), so nothing is lost.
            ["ch" | "aux", _, "send", _, "ind"] => {
                let to = format!("{}/mode", &p[..p.len() - 4]);
                note(notes, p, &to, "send-ind", false);
            }
            // R2 send-wid: per-send stereo width was removed in 3.x.
            ["ch" | "aux", _, "send", _, "wid"] => {
                note(notes, p, "", "send-wid", !is_num(&l.value, 100.0));
            }
            // R10 (send side): a non-independent send follows its bus's `busmode`.
            [strip @ ("ch" | "aux"), m, "send", n, "mode"] => {
                let ind = val(&format!("{strip}/{m}/send/{n}/ind")) == Some(&Json::Bool(true));
                match val(&format!("bus/{n}/busmode")) {
                    Some(bm) if !ind && *bm != l.value => {
                        note(notes, p, &format!("{p}={}", bm.to_text()), "busmode", false);
                        out.push(leaf(l.path.clone(), &l.src, bm.clone()));
                    }
                    _ => out.push(l.clone()),
                }
            }
            // R3 warm-mix: the WARM model's own parallel mix was `mix` in 1.x, `wmix` in 3.x
            // (3.x `gate/mix` is the block-level mix every model has; it stays 100 %).
            ["ch", n, "gate", "mix"] if val(&format!("ch/{n}/gate/mdl")) == Some(&s("WARM")) => {
                let to = format!("ch/{n}/gate/wmix");
                note(notes, p, &to, "warm-mix", false);
                out.push(leaf(to, &l.src, l.value.clone()));
            }
            // R4 out-dly-m: `dly {on, m}` -> `dly {on, mode: M, dly}` (3.x METERS mode, same unit).
            [strip @ ("bus" | "main" | "mtx"), n, "dly", "m"] => {
                let mode = format!("{strip}/{n}/dly/mode");
                let to = format!("{strip}/{n}/dly/dly");
                note(notes, p, &format!("{mode}=M, {to}"), "out-dly-m", false);
                if val(&mode).is_none() {
                    out.push(leaf(mode, &l.src, s("M")));
                }
                out.push(leaf(to, &l.src, l.value.clone()));
            }
            // R5 dynxo-type: band+slope token -> band only (slope has no 3.x home).
            [_, _, "dynxo", "type"] => match l.value.as_str() {
                Some(v) if dynxo_type(v).is_some_and(|t| t != v) => {
                    let t = dynxo_type(v).unwrap_or("OFF");
                    let lossy = v.ends_with('6') || v.ends_with("12");
                    note(notes, p, &format!("{p}={t}"), "dynxo-type", lossy);
                    out.push(leaf(l.path.clone(), &l.src, s(t)));
                }
                _ => out.push(l.clone()),
            },
            // R6 solo-mon: monitor A/B -> PH/SPK.
            ["cfg", "solo", "mon"] => {
                let to = match l.value.as_str() {
                    Some("A") => Some("PH"),
                    Some("B") => Some("SPK"),
                    Some("A+B" | "AB") => Some("PH+SPK"),
                    _ => None,
                };
                match to {
                    Some(t) => {
                        note(notes, p, &format!("{p}={t}"), "solo-mon", false);
                        out.push(leaf(l.path.clone(), &l.src, s(t)));
                    }
                    None => out.push(l.clone()),
                }
            }
            // R7 aux-dyn-mdl: 1.x aux dynamics is the fixed PSE/LA combo without `mdl`.
            ["aux", n, "dyn", _] => {
                let mdl = format!("aux/{n}/dyn/mdl");
                if val(&mdl).is_none() && !out.iter().any(|o| o.path == mdl) {
                    note(notes, &mdl, &format!("{mdl}=CMB"), "aux-dyn-mdl", false);
                    out.push(leaf(mdl, &l.src, s("CMB")));
                }
                out.push(l.clone());
            }
            // R8 flt-slope: 1.x has no LC/HC slope keys; fixed 12 dB/oct (3.x defaults are 24/12).
            [strip, n, "flt", which @ ("lc" | "hc")] => {
                out.push(l.clone());
                let key = format!("{strip}/{n}/flt/{which}s");
                if val(&key).is_none() {
                    note(notes, &key, &format!("{key}=12"), "flt-slope", false);
                    out.push(leaf(key, &l.src, s("12")));
                }
            }
            // R9 mtx-dir: two 1.x direct inputs with tap -> one 3.x direct input without tap.
            ["mtx", n, "dir", k @ ("1" | "2"), last] => {
                let default = match *last {
                    "on" | "inv" => Json::Bool(false),
                    "lvl" => Json::Num("-144".into()),
                    "in" => s("OFF"),
                    "tap" => s("PRE"),
                    _ => Json::Null,
                };
                let differs = !crate::snapshot::json_eq(&l.value, &default);
                if *k == "1" && *last != "tap" {
                    let to = format!("mtx/{n}/dir/{last}");
                    note(notes, p, &to, "mtx-dir", false);
                    out.push(leaf(to, &l.src, l.value.clone()));
                } else {
                    note(notes, p, "", "mtx-dir", differs);
                }
            }
            // R10 busmode: the default send mode of every send into the bus; 3.x keeps only the
            // per-send `mode`, which the send-side rule above sets to the effective mode.
            ["bus", n, "busmode"] => {
                note(
                    notes,
                    p,
                    &format!("ch|aux/*/send/{n}/mode"),
                    "busmode",
                    false,
                );
            }
            // R11 talk-indiv: global flag -> per talkback channel A and B.
            ["cfg", "talk", "indiv"] => {
                for ab in ["A", "B"] {
                    let to = format!("cfg/talk/{ab}/indiv");
                    if val(&to).is_none() {
                        note(notes, p, &to, "talk-indiv", false);
                        out.push(leaf(to, &l.src, l.value.clone()));
                    }
                }
            }
            // R12 talk-mondim: on/off -> dim amount in dB (40..0); on = the monitor's dim.
            ["cfg", "talk", _, "mondim"] if matches!(l.value, Json::Bool(_)) => {
                let on = l.value == Json::Bool(true);
                let db = if on {
                    val("cfg/mon/1/dim").and_then(Json::as_f64).unwrap_or(20.0)
                } else {
                    0.0
                };
                let v = Json::Num(crate::snapshot::fmt_num(db));
                note(notes, p, &format!("{p}={}", v.to_text()), "talk-mondim", on);
                out.push(leaf(l.path.clone(), &l.src, v));
            }
            // R13 safes-area: L/C/R -> LEFT/CENTER/RIGHT; the 1.x value format is unknown, only
            // "no safes" ("0"/"") converts (to the 3.x default).
            ["$ctl", "safes", "area", a @ ("L" | "C" | "R")] => {
                let to = match *a {
                    "L" => "$ctl/safes/area/LEFT",
                    "C" => "$ctl/safes/area/CENTER",
                    _ => "$ctl/safes/area/RIGHT",
                };
                let none = matches!(l.value.as_str(), Some("0" | "")) || is_num(&l.value, 0.0);
                note(notes, p, to, "safes-area", !none);
            }
            // R14 seldblclick: bool -> enum OFF/HOME/BUSFX.
            ["$ctl", "cfg", "seldblclick"] if matches!(l.value, Json::Bool(_)) => {
                let on = l.value == Json::Bool(true);
                let t = if on { "HOME" } else { "OFF" };
                note(notes, p, &format!("{p}={t}"), "seldblclick", on);
                out.push(leaf(l.path.clone(), &l.src, s(t)));
            }
            // R15 mdl-rename: 1.x/3.0 `D241` was split into D241G (gate) and D241C (comp).
            [_, _, slot @ ("gate" | "dyn"), "mdl"] if l.value == s("D241") => {
                let t = if *slot == "gate" { "D241G" } else { "D241C" };
                note(notes, p, &format!("{p}={t}"), "mdl-rename", false);
                out.push(leaf(l.path.clone(), &l.src, s(t)));
            }
            _ => out.push(l.clone()),
        }
    }
    out
}

/// 1.x console model name -> 3.x `creator_model`.
fn model_name(v: &str) -> Option<&'static str> {
    Some(match v {
        "ngc-full" => "wing-fullsize",
        "ngc-rack" => "wing-rack",
        "ngc-compact" => "wing-compact",
        "wing-fullsize" => "wing-fullsize",
        "wing-rack" => "wing-rack",
        "wing-compact" => "wing-compact",
        _ => return None,
    })
}

/// Header for a migrated file: the factory scene's layout and format fields (`type`,
/// `creator_fw`), the source's identity fields (`creator_sn`, `creator_name`, `created`,
/// `creator_version`), `creator_model` mapped to the 3.x name; other source fields appended.
pub(crate) fn migrate_header(
    src: &[(String, Json)],
    factory: &[(String, Json)],
) -> Vec<(String, Json)> {
    let get = |k: &str| src.iter().find(|(kk, _)| kk == k).map(|(_, v)| v.clone());
    let mut out: Vec<(String, Json)> = factory
        .iter()
        .map(|(k, fv)| {
            let v = match k.as_str() {
                "type" | "creator_fw" => fv.clone(),
                "creator_model" => get(k)
                    .and_then(|v| v.as_str().and_then(model_name))
                    .map_or_else(|| fv.clone(), |m| Json::Str(m.to_string())),
                _ => get(k).unwrap_or_else(|| fv.clone()),
            };
            (k.clone(), v)
        })
        .collect();
    for (k, v) in src {
        if !out.iter().any(|(o, _)| o == k) {
            out.push((k.clone(), v.clone()));
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn l(p: &str, v: Json) -> Leaf {
        leaf(p.to_string(), &format!("ae_data/{p}"), v)
    }

    fn run(v: Vec<Leaf>) -> (Vec<(String, String)>, Vec<MigrationNote>) {
        let mut notes = Vec::new();
        let out = migrate_1x(v, &mut notes)
            .into_iter()
            .map(|l| (l.path, l.value.to_text()))
            .collect();
        (out, notes)
    }

    #[test]
    fn rules_rewrite_keys() {
        let (out, notes) = run(vec![
            l("ch/1/gate/mdl", s("WARM")),
            l("ch/1/gate/mix", Json::Num("5".into())),
            l("ch/2/gate/mdl", s("GATE")),
            l("ch/2/gate/mix", Json::Num("50".into())),
            l("main/1/dly/m", Json::Num("3".into())),
            l("ch/1/dynxo/type", s("HI6")),
            l("ch/1/flt/lc", Json::Bool(true)),
            l("aux/1/dyn/on", Json::Bool(true)),
            l("ch/1/send/1/ind", Json::Bool(true)),
            l("ch/1/send/1/wid", Json::Num("100".into())),
            l("cfg/talk/indiv", Json::Bool(false)),
            l("bus/3/busmode", s("PRE")),
            l("ch/1/send/3/mode", s("POST")),
            l("ch/2/send/3/ind", Json::Bool(true)),
            l("ch/2/send/3/mode", s("POST")),
        ]);
        let get = |p: &str| out.iter().find(|(k, _)| k == p).map(|(_, v)| v.as_str());
        assert_eq!(get("ch/1/gate/wmix"), Some("5"));
        assert_eq!(get("ch/1/gate/mix"), None);
        assert_eq!(get("ch/2/gate/mix"), Some("50"));
        assert_eq!(get("main/1/dly/mode"), Some("\"M\""));
        assert_eq!(get("main/1/dly/dly"), Some("3"));
        assert_eq!(get("ch/1/dynxo/type"), Some("\"HI\""));
        assert_eq!(get("ch/1/flt/lcs"), Some("\"12\""));
        assert_eq!(get("aux/1/dyn/mdl"), Some("\"CMB\""));
        assert_eq!(get("cfg/talk/A/indiv"), Some("false"));
        assert_eq!(get("cfg/talk/B/indiv"), Some("false"));
        assert!(get("ch/1/send/1/ind").is_none());
        assert!(get("bus/3/busmode").is_none());
        // a send follows its bus's mode unless it is independent
        assert_eq!(get("ch/1/send/3/mode"), Some("\"PRE\""));
        assert_eq!(get("ch/2/send/3/mode"), Some("\"POST\""));
        let lossy: Vec<_> = notes.iter().filter(|n| n.lossy).map(|n| n.rule).collect();
        assert_eq!(lossy, ["dynxo-type"]);
    }

    #[test]
    fn header_takes_3x_format_and_keeps_identity() {
        let src = vec![
            ("type".to_string(), s("snapshot.8")),
            ("creator_fw".to_string(), s("1.13.9")),
            ("creator_model".to_string(), s("ngc-full")),
            ("creator_name".to_string(), s("PUBLICWING")),
        ];
        let fac = vec![
            ("type".to_string(), s("snapshot.11")),
            ("creator_fw".to_string(), s("3.0.6")),
            ("creator_model".to_string(), s("wing-fullsize")),
            ("creator_name".to_string(), s("X")),
            ("active_show".to_string(), s("")),
        ];
        let h = migrate_header(&src, &fac);
        let t: Vec<String> = h
            .iter()
            .map(|(k, v)| format!("{k}={}", v.to_text()))
            .collect();
        assert_eq!(
            t,
            [
                "type=\"snapshot.11\"",
                "creator_fw=\"3.0.6\"",
                "creator_model=\"wing-fullsize\"",
                "creator_name=\"PUBLICWING\"",
                "active_show=\"\""
            ]
        );
    }
}
