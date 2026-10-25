//! .snap load/save round-trips (T011) and 1.x migration (T019).

use wing_model::json::Json;
use wing_model::{diff, json_eq, DiffEntry, ParamStore, Snapshot, Value, FACTORY_SCENE};

const PUBLIC: &str = include_str!("../../../fixtures/PUBLIC.snap");

/// Describe the first few semantic differences between two documents (empty = equal).
/// Numbers must be the same `f32`.
fn differences(a: &Json, b: &Json) -> Vec<String> {
    fn rec(a: &Json, b: &Json, path: &str, out: &mut Vec<String>) {
        if out.len() >= 20 {
            return;
        }
        match (a, b) {
            (Json::Obj(x), Json::Obj(y)) => {
                for (k, v) in x {
                    match y.iter().find(|(k2, _)| k2 == k) {
                        Some((_, w)) => rec(v, w, &format!("{path}/{k}"), out),
                        None => out.push(format!("missing in output: {path}/{k}")),
                    }
                }
                for (k, _) in y {
                    if !x.iter().any(|(k2, _)| k2 == k) {
                        out.push(format!("extra in output: {path}/{k}"));
                    }
                }
            }
            (Json::Num(_), Json::Num(_)) => {
                let f = |v: &Json| v.as_f64().map(|x| x as f32);
                if f(a) != f(b) {
                    out.push(format!("{path}: {} != {}", a.to_text(), b.to_text()))
                }
            }
            _ if !json_eq(a, b) => out.push(format!("{path}: {} != {}", a.to_text(), b.to_text())),
            _ => {}
        }
    }
    let mut out = Vec::new();
    rec(a, b, "", &mut out);
    out
}

#[test]
fn factory_scene_round_trips_semantically() {
    let store = ParamStore::new();
    let (snap, report) = Snapshot::load(&store, FACTORY_SCENE).unwrap();
    assert!(!report.legacy);
    assert_eq!(report.keys, 25_082); // 25,081 values + the empty `ce_data/lib` object
    assert!(report.unmapped.is_empty(), "{:?}", report.unmapped);
    assert!(report.rejected.is_empty(), "{:?}", report.rejected);
    assert!(report.adjusted.is_empty(), "{:?}", report.adjusted);
    assert!(report.migrated.is_empty());
    assert_eq!(report.preserved.len(), 3, "{:?}", report.preserved);
    let out = snap.save(&store);
    let d = differences(
        &Json::parse(FACTORY_SCENE).unwrap(),
        &Json::parse(&out).unwrap(),
    );
    assert!(d.is_empty(), "{d:#?}");
    // header written verbatim, first
    assert!(out.starts_with(
        r#"{"type":"snapshot.11","creator_fw":"3.0.6-193-g60c910ab-dirty:dev_unidyn","#
    ));
}

#[test]
fn modified_3x_scene_round_trips_values_and_unknown_keys() {
    let mut doc = Json::parse(FACTORY_SCENE).unwrap();
    // an unknown key deep in the tree and an unknown top-level header field
    let text = doc.to_text().replacen(
        r#""ch":{"1":{"#,
        r#""ch":{"1":{"futurekey":{"x":[1,2.50,"y"]},"#,
        1,
    );
    doc = Json::parse(&text.replacen('{', r#"{"future_hdr":1e3,"#, 1)).unwrap();
    let store = ParamStore::new();
    let (snap, report) = Snapshot::load(&store, &doc.to_text()).unwrap();
    assert_eq!(report.unmapped, ["ch/1/futurekey/x"]);
    // change some values, including a model switch
    let id = store.lookup("ch/1/fdr").unwrap();
    store.set(id, Value::Float(-3.5)).unwrap();
    let mdl = store.lookup("ch/2/dyn/mdl").unwrap();
    store.set_enum_name(mdl, "LA", 0).unwrap();
    let out = snap.save(&store);
    assert!(out.contains(r#""futurekey":{"x":[1,2.50,"y"]}"#));
    assert!(out.starts_with(r#"{"future_hdr":1e3,"type":"snapshot.11""#));
    let again = Json::parse(&out).unwrap();
    let ch2dyn = again
        .get("ae_data")
        .unwrap()
        .get("ch")
        .unwrap()
        .get("2")
        .unwrap()
        .get("dyn")
        .unwrap();
    assert_eq!(ch2dyn.get("mdl"), Some(&Json::Str("LA".into())));
    assert!(ch2dyn.get("ingain").is_some() && ch2dyn.get("thr").is_none());
    // reload into a fresh store: identical entries
    let store2 = ParamStore::new();
    let (snap2, r2) = Snapshot::load(&store2, &out).unwrap();
    assert_eq!(r2.unmapped, ["ch/1/futurekey/x"]);
    let d = diff(&snap.entries(&store), &snap2.entries(&store2));
    assert!(d.is_empty(), "{d:?}");
}

#[test]
fn public_snap_migrates_with_no_unexplained_keys() {
    let store = ParamStore::new();
    let (snap, r) = Snapshot::load(&store, PUBLIC).unwrap();
    assert!(r.legacy);
    assert_eq!(r.format, "snapshot.8");
    assert!(r.unmapped.is_empty(), "{:?}", r.unmapped);
    assert!(r.rejected.is_empty(), "{:?}", r.rejected);
    assert_eq!(r.preserved, ["$ctl/$globals/usewheel"]);
    // every one of the 1,709 legacy keys has a note (plus keys the migration adds or converts)
    let count = |rule: &str| r.migrated.iter().filter(|n| n.rule == rule).count();
    assert_eq!(count("send-ind") + count("send-wid"), 1_536);
    assert_eq!(count("mtx-dir"), 80);
    assert_eq!(count("busmode"), 16);
    assert_eq!(count("out-dly-m"), 12);
    assert_eq!(count("aux-dyn-mdl"), 8);
    assert_eq!(count("safes-area"), 3);
    assert_eq!(count("talk-indiv"), 2);
    assert_eq!(count("warm-mix"), 8);
    // lossy: 2 sends with width 70 %, 6 dynxo band+slope tokens (slope dropped)
    let lossy: Vec<_> = r.lossy().map(|n| n.rule).collect();
    assert_eq!(lossy.iter().filter(|&&x| x == "send-wid").count(), 2);
    assert_eq!(lossy.iter().filter(|&&x| x == "dynxo-type").count(), 6);
    assert_eq!(lossy.len(), 8, "{lossy:?}");

    // mix intent
    let f = |p: &str| store.get_f64(store.lookup(p).unwrap()).unwrap();
    let t = |p: &str| {
        let mut s = String::new();
        store
            .format_token(store.lookup(p).unwrap(), &mut s)
            .unwrap();
        s
    };
    assert_eq!(t("ch/20/gate/mdl"), "WARM");
    assert_eq!(f("ch/20/gate/wmix"), 5.0);
    assert_eq!(f("ch/20/gate/mix"), 100.0);
    assert_eq!(t("mtx/1/dly/mode"), "M");
    assert_eq!(f("mtx/1/dly/dly"), 5.0);
    assert_eq!(t("aux/1/dyn/mdl"), "CMB");
    assert_eq!(f("aux/1/dyn/ingain"), 40.0);
    assert_eq!(t("cfg/solo/mon"), "PH");
    assert_eq!(t("ch/1/flt/lcs"), "12");
    assert_eq!(t("ch/9/dynxo/type"), "LO"); // LO12
    assert_eq!(t("main/1/dynxo/type"), "HI"); // HI6
    assert!((f("ch/1/flt/lcf") - 27.252).abs() < 0.01);
    assert_eq!(t("bus/1/name"), "VOX VERB");
    // ind=true sends keep their own mode; the others already equal their bus's busmode
    assert_eq!(t("ch/21/send/12/mode"), "POST");
    assert_eq!(t("ch/21/send/11/mode"), "PRE");

    // saving writes the 3.x layout: 3.x header, no legacy keys, reloads without migration
    let out = snap.save(&store);
    assert!(out.starts_with(r#"{"type":"snapshot.11","creator_fw":"3.0.6-"#));
    assert!(out.contains(r#""creator_model":"wing-fullsize""#));
    assert!(out.contains(r#""creator_name":"PUBLICWING""#));
    // 3.x-only keys: tree params get factory defaults, unmodelled GUI prefs the factory values
    assert!(out.contains(r#""lcs":"12""#) && out.contains(r#""dynsc":{"#));
    assert!(out.contains(r#""timefmt":"#) && out.contains(r#""usewheel":"#));
    for legacy in [r#""ind":"#, r#""busmode":"#, r#""m":0.1,"#, r#""L":"0""#] {
        assert!(!out.contains(legacy), "{legacy}");
    }
    let store2 = ParamStore::new();
    let (snap2, r2) = Snapshot::load(&store2, &out).unwrap();
    assert!(!r2.legacy);
    assert!(r2.unmapped.is_empty() && r2.rejected.is_empty() && r2.migrated.is_empty());
    assert_eq!(snap2.save(&store2), out);
}

#[test]
fn diff_reports_changes() {
    let a = ParamStore::new();
    let (sa, _) = Snapshot::load(&a, FACTORY_SCENE).unwrap();
    let b = ParamStore::new();
    let (sb, _) = Snapshot::load(&b, PUBLIC).unwrap();
    let d = diff(&sa.entries(&a), &sb.entries(&b));
    assert!(d
        .iter()
        .any(|e| matches!(e, DiffEntry::Changed { path, .. } if path == "bus/1/name")));
    // model switch: GATE params only in a, WARM params only in b
    assert!(d
        .iter()
        .any(|e| matches!(e, DiffEntry::OnlyA { path, .. } if path == "ch/20/gate/thr")));
    assert!(d
        .iter()
        .any(|e| matches!(e, DiffEntry::OnlyB { path, .. } if path == "ch/20/gate/wmix")));
    assert!(diff(&sa.entries(&a), &sa.entries(&a)).is_empty());
}

#[test]
fn rejects_non_snapshots() {
    let store = ParamStore::new();
    assert!(Snapshot::load(&store, "[1]").is_err());
    assert!(Snapshot::load(&store, "{\"ae_data\":").is_err());
    assert!(Snapshot::load(&store, "{\"type\":\"x\"}").is_err());
}
