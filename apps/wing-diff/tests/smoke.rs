//! Smoke test: the factory scene (3.x) against the legacy 1.x `fixtures/PUBLIC.snap`.

use wing_diff::{diff_scenes, load_file, load_text, render, Filter, Format, Options};
use wing_model::FACTORY_SCENE;

fn public() -> wing_diff::Loaded {
    let p = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/PUBLIC.snap");
    load_file(p).expect("PUBLIC.snap loads")
}

#[test]
fn factory_vs_public() {
    let a = load_text("factory.snap", FACTORY_SCENE).unwrap();
    let b = public();
    assert!(b.report.legacy && !a.report.legacy);
    let r = diff_scenes(&a, &b, &Options::default());
    assert!(r.changes() > 100, "PUBLIC.snap differs widely from factory");
    let t = render(&r, Format::Text);
    assert!(t.contains("migrated 1.x"), "header notes the migration");
    // channel 3 is named HATS in PUBLIC.snap
    assert!(t.contains("Ch 3 'HATS'"), "{}", &t[..t.len().min(600)]);
    assert!(t.contains("mute: off \u{2192} on"));

    // UI/state filtering and strip filtering shrink the report
    let ui = diff_scenes(
        &a,
        &b,
        &Options {
            ignore_ui: true,
            filter: None,
        },
    );
    assert!(ui.ignored > 0 && ui.changes() < r.changes());
    let ch = diff_scenes(
        &a,
        &b,
        &Options {
            ignore_ui: false,
            filter: Some(Filter::parse("ch/3").unwrap()),
        },
    );
    assert_eq!(ch.strips.len(), 1);
    assert!(ch.strips[0].label.starts_with("Ch 3"));

    // all three formats render; json parses
    assert!(render(&r, Format::Markdown).starts_with("# Scene diff"));
    let j = render(&r, Format::Json);
    assert!(wing_model::json::Json::parse(&j).is_ok());
}

#[test]
fn migrated_scene_diffs_against_its_3x_resave() {
    // a 1.x file and its migrated 3.x re-save compare equal (both load through migrate.rs)
    let b = public();
    let saved = b.snap.save(&b.store);
    let again = load_text("resaved.snap", &saved).unwrap();
    let r = diff_scenes(&b, &again, &Options::default());
    assert_eq!(r.changes(), 0, "{}", render(&r, Format::Text));
}
