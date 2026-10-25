//! Hash parity against the source of truth: every `path -> hash` pair in
//! data/schema/wing-tree.json (`hashes`, recorded from WING-Edit's own hash table) must match
//! `wing_hash`, resolve to a node via both path and hash, and round-trip the path. Also checks
//! the schema has no node the JSON does not list.

use std::collections::HashSet;

use wing_model::{wing_hash, Schema};

/// Minimal scanner for the flat `"hashes": { "path": "hex", ... }` object (no serde dependency).
fn json_hashes() -> Vec<(String, u32)> {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../data/schema/wing-tree.json"
    );
    let text = std::fs::read_to_string(path).expect("read wing-tree.json");
    let start = text.find("\"hashes\"").expect("hashes key");
    let body = &text[start + text[start..].find('{').unwrap() + 1..];
    let body = &body[..body.find('}').unwrap()];
    let mut out = Vec::new();
    let mut parts = body.split('"').skip(1).step_by(2); // strings at odd split positions
    while let (Some(k), Some(v)) = (parts.next(), parts.next()) {
        assert!(!k.contains('\\'), "escaped key {k}");
        out.push((k.to_string(), u32::from_str_radix(v, 16).expect("hex")));
    }
    out
}

#[test]
fn every_node_hash_matches_wing_edit() {
    let s = Schema::get();
    let pairs = json_hashes();
    assert_eq!(pairs.len(), 43_007);
    assert_eq!(pairs.len() + 1, s.node_count());
    let mut seen = HashSet::new();
    for (path, h) in &pairs {
        assert_eq!(wing_hash(path), *h, "wing_hash({path})");
        let id = s.by_path(path).unwrap_or_else(|| panic!("by_path({path})"));
        assert_eq!(s.hash(id), *h, "{path}");
        assert_eq!(s.by_hash(*h), Some(id), "{path}");
        assert_eq!(&s.path(id), path);
        assert!(seen.insert(id));
    }
}

#[test]
fn getpath_form_hashes() {
    // WING-Edit hashes getPath() text without the first char: "/ch.1.flt.lcf"
    let s = Schema::get();
    let id = s.by_path("ch/1/flt/lcf").unwrap();
    assert_eq!(wing_hash("/ch.1.flt.lcf"), s.hash(id));
}
