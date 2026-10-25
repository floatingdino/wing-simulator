//! Test-only, minimal `.snap` -> ParamStore loader for the routing-relevant subtrees (`io`,
//! `ch/*/in`, `aux/*/in`). Stand-in until the real snapshot loader (T011) lands.

#![allow(dead_code)]

pub mod rig;

use serde_json::Value as Json;
use wing_model::{ParamStore, ParamType, Value};

pub const PUBLIC_SNAP: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/PUBLIC.snap");

/// Load the routing subtrees of a snapshot into `store`. Returns (applied, skipped) leaf counts;
/// keys absent from the 3.3.4 schema (fw 1.13 extras) are skipped.
pub fn load_routing(store: &ParamStore, snap_path: &str) -> (usize, usize) {
    let text = std::fs::read_to_string(snap_path).expect("read snapshot");
    let snap: Json = serde_json::from_str(&text).expect("parse snapshot");
    let ae = &snap["ae_data"];
    let mut n = (0, 0);
    walk(store, "io", &ae["io"], &mut n);
    for strip in ["ch", "aux"] {
        if let Some(map) = ae[strip].as_object() {
            for (k, v) in map {
                walk(store, &format!("{strip}/{k}/in"), &v["in"], &mut n);
            }
        }
    }
    n
}

fn walk(store: &ParamStore, path: &str, v: &Json, n: &mut (usize, usize)) {
    if let Some(map) = v.as_object() {
        for (k, child) in map {
            walk(store, &format!("{path}/{k}"), child, n);
        }
        return;
    }
    let Some(id) = store.lookup(path) else {
        n.1 += 1;
        return;
    };
    let Some(desc) = store.desc(id) else {
        n.1 += 1;
        return;
    };
    let r = match v {
        Json::Bool(b) => store.set(id, Value::Int(*b as i32)),
        Json::Number(x) => {
            let x = x.as_f64().unwrap();
            match desc.ty {
                ParamType::Int => store.set(id, Value::Int(x as i32)),
                ParamType::Enum | ParamType::FEnum | ParamType::Str => {
                    store.set_text(id, &x.to_string(), 0)
                }
                _ => store.set(id, Value::Float(x as f32)),
            }
        }
        Json::String(s) => store.set_text(id, s, 0),
        _ => {
            n.1 += 1;
            return;
        }
    };
    match r {
        Ok(_) => n.0 += 1,
        Err(_) => n.1 += 1,
    }
}
