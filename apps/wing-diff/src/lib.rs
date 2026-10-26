//! Human-friendly scene diff for `.snap` files.
//!
//! Both scenes are loaded through `wing-model` (so 1.x files are migrated to 3.x first and a 1.x
//! scene compares cleanly with a 3.x one), flattened to tree paths, and the differing leaves are
//! grouped per strip (`ch/3`, `bus/2`, `main/1`, `fx/4`, ...) and described with the strip's
//! name, a section label (`EQ`, `COMP`, `Send -> Bus 3 'Vox Verb'`) and the schema's display
//! formatting (`+1.5 dB`, `2k00 Hz`, `-oo dB`).
//!
//! Rules worth knowing:
//! - When a processing block's model changes (`dyn/mdl COMP -> 76LA`) the model-specific
//!   parameters of that block differ by construction (the new model loads its own defaults), so
//!   they are collapsed into one "N model-specific parameters differ" line. Non-model parameters
//!   (`on`, `mix`, ...) are still listed.
//! - `--ignore-ui` drops console-surface and display state that does not affect the mix:
//!   everything under `$ctl/` (layers, user keys, faders banking, spill, view and sends-on-fader
//!   modes, ...), every strip's `led` (scribble-strip light), and the meter/RTA display
//!   settings `cfg/mtr/*` and `cfg/rta/*`. Names, colours and icons are NOT ignored: they are
//!   part of how a scene reads, and renames matter to the engineer.
//! - Keys the schema does not know are compared verbatim and shown with their raw JSON values.
//!
//! [`record`] (binary `wing-record-setup`) builds a scoped recording scene and a matching DAW
//! project from a show scene.

pub mod record;

use std::collections::{BTreeMap, HashMap, HashSet};

use wing_model::json::Json;
use wing_model::{diff, DiffEntry, LoadReport, ParamClass, ParamStore, ParamType, Snapshot};

/// Output format.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Format {
    Text,
    Markdown,
    Json,
}

impl Format {
    pub fn parse(s: &str) -> Option<Format> {
        match s {
            "text" => Some(Format::Text),
            "markdown" | "md" => Some(Format::Markdown),
            "json" => Some(Format::Json),
            _ => None,
        }
    }
}

/// Strip filter: `ch/1..40,bus,main/1` (category with an optional number or inclusive range).
#[derive(Clone, Debug, Default)]
pub struct Filter(Vec<(String, Option<(u32, u32)>)>);

impl Filter {
    pub fn parse(s: &str) -> Result<Filter, String> {
        let mut out = Vec::new();
        for tok in s.split(',').map(str::trim).filter(|t| !t.is_empty()) {
            let (cat, range) = match tok.split_once('/') {
                None => (tok, None),
                Some((c, r)) => {
                    let bad = || format!("bad filter range '{r}' in '{tok}'");
                    let range = match r.split_once("..") {
                        Some((lo, hi)) => (
                            lo.parse().map_err(|_| bad())?,
                            hi.parse().map_err(|_| bad())?,
                        ),
                        None => {
                            let n = r.parse().map_err(|_| bad())?;
                            (n, n)
                        }
                    };
                    (c, Some(range))
                }
            };
            out.push((cat.to_string(), range));
        }
        if out.is_empty() {
            return Err("empty filter".into());
        }
        Ok(Filter(out))
    }

    fn matches(&self, cat: &str, num: Option<u32>) -> bool {
        self.0.iter().any(|(c, r)| {
            c == cat
                && match (r, num) {
                    (None, _) => true,
                    (Some((lo, hi)), Some(n)) => (*lo..=*hi).contains(&n),
                    (Some(_), None) => false,
                }
        })
    }
}

/// Diff options.
#[derive(Clone, Debug, Default)]
pub struct Options {
    pub filter: Option<Filter>,
    pub ignore_ui: bool,
}

/// A loaded scene.
pub struct Loaded {
    pub name: String,
    pub store: ParamStore,
    pub snap: Snapshot,
    pub report: LoadReport,
}

/// Parse `.snap` text (1.x is migrated to 3.x).
pub fn load_text(name: &str, text: &str) -> Result<Loaded, String> {
    let store = ParamStore::new();
    let (snap, report) = Snapshot::load(&store, text).map_err(|e| format!("{name}: {e}"))?;
    Ok(Loaded {
        name: name.to_string(),
        store,
        snap,
        report,
    })
}

/// Load a `.snap` file.
pub fn load_file(path: &str) -> Result<Loaded, String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("{path}: {e}"))?;
    load_text(path, &text)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    Changed,
    /// Only present in B.
    Added,
    /// Only present in A.
    Removed,
    /// `count` model-specific parameters collapsed after a model change.
    Collapsed,
}

/// One described change.
#[derive(Clone, Debug)]
pub struct Item {
    pub path: String,
    pub section: String,
    pub param: String,
    pub a: String,
    pub b: String,
    pub kind: Kind,
    pub count: usize,
}

impl Item {
    /// `EQ band 2 gain: +1.5 dB -> +3.0 dB`
    fn title(&self) -> String {
        if self.kind == Kind::Collapsed {
            return format!(
                "{}{} model-specific parameters differ (new model defaults)",
                prefix(&self.section),
                self.count
            );
        }
        format!("{}{}", prefix(&self.section), self.param)
    }

    fn line(&self) -> String {
        if self.kind == Kind::Collapsed {
            return self.title();
        }
        format!("{}: {} \u{2192} {}", self.title(), self.a, self.b)
    }
}

fn prefix(section: &str) -> String {
    if section.is_empty() {
        String::new()
    } else {
        format!("{section} ")
    }
}

/// All changes of one strip (or console area).
#[derive(Clone, Debug)]
pub struct Strip {
    pub cat: String,
    pub num: Option<u32>,
    pub label: String,
    pub items: Vec<Item>,
}

/// Header info about one input.
#[derive(Clone, Debug)]
pub struct SceneInfo {
    pub name: String,
    pub format: String,
    pub fw: String,
    pub migrated: bool,
}

/// The full diff.
#[derive(Clone, Debug)]
pub struct Report {
    pub a: SceneInfo,
    pub b: SceneInfo,
    pub strips: Vec<Strip>,
    /// Differing keys dropped by `--ignore-ui`.
    pub ignored: usize,
    /// Differing keys dropped by `--filter`.
    pub filtered: usize,
}

impl Report {
    /// Number of reported lines.
    pub fn changes(&self) -> usize {
        self.strips.iter().map(|s| s.items.len()).sum()
    }
}

/// Load and diff two files.
pub fn diff_files(a: &str, b: &str, opts: &Options) -> Result<Report, String> {
    Ok(diff_scenes(&load_file(a)?, &load_file(b)?, opts))
}

fn info(l: &Loaded) -> SceneInfo {
    SceneInfo {
        name: l.name.clone(),
        format: l.report.format.clone(),
        fw: l.report.creator_fw.clone(),
        migrated: l.report.legacy,
    }
}

const STRIP_CATS: [&str; 8] = ["ch", "aux", "bus", "main", "mtx", "dca", "mgrp", "fx"];
const NAMED_CATS: [&str; 7] = ["ch", "aux", "bus", "main", "mtx", "dca", "mgrp"];
const CAT_ORDER: [&str; 9] = [
    "ch", "aux", "bus", "main", "mtx", "dca", "mgrp", "fx", "mon",
];

/// Split a tree path into (category, strip number, remaining components).
fn classify(path: &str) -> (String, Option<u32>, Vec<&str>) {
    let p: Vec<&str> = path.split('/').collect();
    if p.len() > 2 {
        if STRIP_CATS.contains(&p[0]) {
            if let Ok(n) = p[1].parse() {
                return (p[0].to_string(), Some(n), p[2..].to_vec());
            }
        }
        if p[0] == "cfg" && p[1] == "mon" && p.len() > 3 {
            if let Ok(n) = p[2].parse() {
                return ("mon".into(), Some(n), p[3..].to_vec());
            }
        }
    }
    (p[0].to_string(), None, p[1..].to_vec())
}

/// Whether a differing key is console-surface/display state (see the module docs).
fn is_ui(path: &str) -> bool {
    path.starts_with("$ctl/")
        || path.starts_with("cfg/mtr/")
        || path.starts_with("cfg/rta/")
        || (classify(path).2.last() == Some(&"led") && classify(path).1.is_some())
}

fn str_of(l: &Loaded, path: &str) -> Option<String> {
    let id = l.store.lookup(path)?;
    l.store.with_str(id, str::to_string)
}

fn token_of(l: &Loaded, path: &str) -> Option<String> {
    let id = l.store.lookup(path)?;
    let mut s = String::new();
    l.store.format_token(id, &mut s).ok()?;
    Some(s)
}

/// Name from B, else A.
fn either<T>(a: &Loaded, b: &Loaded, f: impl Fn(&Loaded) -> Option<T>) -> Option<T> {
    f(b).or_else(|| f(a))
}

fn strip_label(a: &Loaded, b: &Loaded, cat: &str, num: Option<u32>) -> String {
    let Some(n) = num else {
        return match cat {
            "io" => "I/O".into(),
            "cfg" => "Config".into(),
            "cards" => "Cards".into(),
            "$ctl" => "Console UI".into(),
            "$globals" => "Globals".into(),
            "play" | "rec" => format!("{} (USB/card)", cat.to_uppercase()),
            c => c.to_string(),
        };
    };
    let kind = match cat {
        "ch" => "Ch",
        "aux" => "Aux",
        "bus" => "Bus",
        "main" => "Main",
        "mtx" => "Matrix",
        "dca" => "DCA",
        "mgrp" => "Mute group",
        "fx" => "FX",
        _ => "Monitor",
    };
    let name = if NAMED_CATS.contains(&cat) {
        either(a, b, |l| str_of(l, &format!("{cat}/{n}/name")))
    } else if cat == "fx" {
        either(a, b, |l| token_of(l, &format!("fx/{n}/mdl")))
    } else {
        None
    };
    match name.filter(|s| !s.is_empty()) {
        Some(nm) => format!("{kind} {n} '{nm}'"),
        None => format!("{kind} {n}"),
    }
}

/// Section label (`EQ`, `Send -> Bus 3 'x'`) for the path components under the strip.
fn section(a: &Loaded, b: &Loaded, cat: &str, num: u32, rest: &[&str], dyn_model: bool) -> String {
    let head = rest.first().copied().unwrap_or("");
    let second = rest.get(1).copied().unwrap_or("");
    if rest.len() < 2 {
        return String::new();
    }
    let s = match head {
        "in" => match second {
            "set" => "Input".to_string(),
            "conn" => "Input routing".to_string(),
            o => format!("Input {o}"),
        },
        "flt" => "Filter".into(),
        "peq" => "PEQ".into(),
        "gate" => "Gate".into(),
        "gatesc" => "Gate SC".into(),
        "eq" => "EQ".into(),
        "dynxo" => "Dyn XO".into(),
        "dynsc" => "Dyn SC".into(),
        "preins" => "Pre-insert".into(),
        "postins" => "Post-insert".into(),
        "dyn" => {
            let tok = (!dyn_model)
                .then(|| either(a, b, |l| token_of(l, &format!("{cat}/{num}/dyn/mdl"))))
                .flatten();
            tok.unwrap_or_else(|| "Dyn".into())
        }
        "send" if rest.len() > 2 => {
            let nm = either(a, b, |l| str_of(l, &format!("bus/{second}/name")));
            match nm.filter(|s| !s.is_empty()) {
                Some(n) => format!("Send \u{2192} Bus {second} '{n}'"),
                None => format!("Send \u{2192} Bus {second}"),
            }
        }
        "main" if rest.len() > 2 => format!("Main {second} send"),
        "dir" => "Direct".into(),
        _ => rest[..rest.len() - 1].join(" "),
    };
    s
}

/// Section for paths outside strips (`io/in/LCL/1/g` -> `in LCL 1`).
fn other_section(rest: &[&str]) -> String {
    if rest.len() < 2 {
        String::new()
    } else {
        rest[..rest.len() - 1].join(" ")
    }
}

fn leaf_label(l: &Loaded, path: &str, leaf: &str, in_eq: bool) -> String {
    if in_eq {
        let c: Vec<char> = leaf.chars().collect();
        let what = |c: char| match c {
            'g' => Some("gain"),
            'f' => Some("freq"),
            'q' => Some("Q"),
            _ => None,
        };
        if c.len() == 2 {
            if let (Some(d), Some(w)) = (c[0].to_digit(10), what(c[1])) {
                return format!("band {d} {w}");
            }
            if let Some(w) = what(c[1]) {
                match c[0] {
                    'l' => return format!("low shelf {w}"),
                    'h' => return format!("high shelf {w}"),
                    _ => {}
                }
            }
        }
    }
    match leaf {
        "on" => return "on".into(),
        "fdr" => return "fader".into(),
        "wid" => return "width".into(),
        "col" => return "colour".into(),
        "mdl" => return "model".into(),
        "thr" => return "threshold".into(),
        "att" => return "attack".into(),
        "hld" => return "hold".into(),
        "rel" => return "release".into(),
        "lvl" => return "level".into(),
        _ => {}
    }
    match l.store.lookup(path).and_then(|id| l.store.desc(id)) {
        Some(d) if !d.long.is_empty() => d.long.to_lowercase(),
        _ => leaf.to_string(),
    }
}

/// Display text of one side's value, with units, as WING-Edit shows it.
fn fmt_value(l: &Loaded, path: &str, j: &Json) -> String {
    let id = l.store.lookup(path);
    if let Some((id, d)) = id.and_then(|id| l.store.desc(id).map(|d| (id, d))) {
        let mut s = String::new();
        let ok = if d.class == ParamClass::Bool {
            s.push_str(if l.store.get_raw(id) != 0 {
                "on"
            } else {
                "off"
            });
            true
        } else if d.ty == ParamType::Str {
            s.push('\'');
            let _ = l.store.with_str(id, |t| s.push_str(t));
            s.push('\'');
            true
        } else if path.ends_with("/mdl") {
            l.store.format_token(id, &mut s).is_ok()
        } else {
            l.store.format(id, &mut s).is_ok()
        };
        if ok {
            return s;
        }
    }
    match j {
        Json::Str(t) => format!("'{t}'"),
        o => o.to_text(),
    }
}

fn is_model_param(a: &Loaded, b: &Loaded, path: &str) -> bool {
    [a, b].iter().any(|l| {
        l.store
            .lookup(path)
            .and_then(|id| l.store.desc(id))
            .is_some_and(|d| d.model_param)
    })
}

/// Diff two loaded scenes.
pub fn diff_scenes(a: &Loaded, b: &Loaded, opts: &Options) -> Report {
    let ea = a.snap.entries(&a.store);
    let eb = b.snap.entries(&b.store);
    let entries = diff(&ea, &eb);

    let model_changed: HashSet<String> = entries
        .iter()
        .filter_map(|e| match e {
            DiffEntry::Changed { path, .. } => path.strip_suffix("/mdl").map(str::to_string),
            _ => None,
        })
        .collect();

    type Key = (usize, String, u32);
    let mut strips: BTreeMap<Key, Strip> = BTreeMap::new();
    let mut collapsed: HashMap<(Key, String), usize> = HashMap::new();
    let (mut ignored, mut filtered) = (0, 0);

    for e in &entries {
        let (path, ja, jb, kind) = match e {
            DiffEntry::Changed { path, a, b } => (path, Some(a), Some(b), Kind::Changed),
            DiffEntry::OnlyA { path, a } => (path, Some(a), None, Kind::Removed),
            DiffEntry::OnlyB { path, b } => (path, None, Some(b), Kind::Added),
        };
        if opts.ignore_ui && is_ui(path) {
            ignored += 1;
            continue;
        }
        let (cat, num, rest) = classify(path);
        if let Some(f) = &opts.filter {
            if !f.matches(&cat, num) {
                filtered += 1;
                continue;
            }
        }
        let order = CAT_ORDER
            .iter()
            .position(|c| *c == cat)
            .unwrap_or(CAT_ORDER.len());
        let key: Key = (order, cat.clone(), num.unwrap_or(0));
        let strip = strips.entry(key.clone()).or_insert_with(|| Strip {
            cat: cat.clone(),
            num,
            label: strip_label(a, b, &cat, num),
            items: Vec::new(),
        });

        let block = path.rsplit_once('/').map_or("", |(p, _)| p).to_string();
        let leaf = *rest.last().unwrap_or(&"");
        let sec_name = match num {
            Some(n) if cat != "mon" => {
                let dyn_model = model_changed.contains(&format!("{cat}/{n}/dyn"));
                section(a, b, &cat, n, &rest, dyn_model)
            }
            _ => other_section(&rest),
        };

        if model_changed.contains(&block) && leaf != "mdl" && is_model_param(a, b, path) {
            match collapsed.get(&(key.clone(), block.clone())) {
                Some(&i) => strip.items[i].count += 1,
                None => {
                    collapsed.insert((key, block), strip.items.len());
                    strip.items.push(Item {
                        path: block_path(path),
                        section: sec_name,
                        param: String::new(),
                        a: String::new(),
                        b: String::new(),
                        kind: Kind::Collapsed,
                        count: 1,
                    });
                }
            }
            continue;
        }

        let in_eq = matches!(rest.first(), Some(&"eq") | Some(&"peq")) && rest.len() == 2;
        let probe = if jb.is_some() { b } else { a };
        let param = leaf_label(probe, path, leaf, in_eq);
        let absent = "(absent)".to_string();
        let sa = ja.map_or(absent.clone(), |j| fmt_value(a, path, j));
        let sb = jb.map_or(absent, |j| fmt_value(b, path, j));
        strip.items.push(Item {
            path: path.clone(),
            section: sec_name,
            param,
            a: sa,
            b: sb,
            kind,
            count: 1,
        });
    }

    Report {
        a: info(a),
        b: info(b),
        strips: strips.into_values().collect(),
        ignored,
        filtered,
    }
}

fn block_path(path: &str) -> String {
    path.rsplit_once('/').map_or(path, |(p, _)| p).to_string()
}

fn scene_line(s: &SceneInfo) -> String {
    format!(
        "{} ({}, fw {}{})",
        s.name,
        s.format,
        s.fw,
        if s.migrated {
            ", migrated 1.x \u{2192} 3.x"
        } else {
            ""
        }
    )
}

fn footer(r: &Report) -> String {
    let mut s = format!(
        "{} change{} in {} group{}",
        r.changes(),
        if r.changes() == 1 { "" } else { "s" },
        r.strips.len(),
        if r.strips.len() == 1 { "" } else { "s" },
    );
    if r.ignored > 0 {
        s.push_str(&format!(" ({} UI/state keys ignored)", r.ignored));
    }
    if r.filtered > 0 {
        s.push_str(&format!(" ({} keys outside filter)", r.filtered));
    }
    s
}

/// Render a report.
pub fn render(r: &Report, f: Format) -> String {
    match f {
        Format::Text => render_text(r),
        Format::Markdown => render_markdown(r),
        Format::Json => render_json(r),
    }
}

fn render_text(r: &Report) -> String {
    let mut o = format!("A: {}\nB: {}\n", scene_line(&r.a), scene_line(&r.b));
    if r.changes() == 0 {
        o.push_str("\nNo differences.\n");
    }
    for s in &r.strips {
        o.push_str(&format!("\n{}\n", s.label));
        for i in &s.items {
            o.push_str(&format!("  {}\n", i.line()));
        }
    }
    o.push_str(&format!("\n{}\n", footer(r)));
    o
}

fn render_markdown(r: &Report) -> String {
    let mut o = String::from("# Scene diff\n\n");
    o.push_str(&format!(
        "- **A:** {}\n- **B:** {}\n",
        md_esc(&scene_line(&r.a)),
        md_esc(&scene_line(&r.b))
    ));
    if r.changes() == 0 {
        o.push_str("\nNo differences.\n");
    }
    for s in &r.strips {
        o.push_str(&format!("\n## {}\n\n", md_esc(&s.label)));
        for i in &s.items {
            if i.kind == Kind::Collapsed {
                o.push_str(&format!("- _{}_\n", i.title()));
            } else {
                o.push_str(&format!(
                    "- **{}**: `{}` \u{2192} `{}`\n",
                    md_esc(&i.title()),
                    i.a,
                    i.b
                ));
            }
        }
    }
    o.push_str(&format!("\n_{}_\n", footer(r)));
    o
}

fn md_esc(s: &str) -> String {
    s.replace('*', "\\*").replace('_', "\\_")
}

fn jstr(s: &str) -> Json {
    Json::Str(s.to_string())
}

fn jobj(v: Vec<(&str, Json)>) -> Json {
    Json::Obj(v.into_iter().map(|(k, v)| (k.to_string(), v)).collect())
}

fn render_json(r: &Report) -> String {
    let scene = |s: &SceneInfo| {
        jobj(vec![
            ("file", jstr(&s.name)),
            ("format", jstr(&s.format)),
            ("fw", jstr(&s.fw)),
            ("migrated", Json::Bool(s.migrated)),
        ])
    };
    let strips = r
        .strips
        .iter()
        .map(|s| {
            let items = s
                .items
                .iter()
                .map(|i| {
                    let kind = match i.kind {
                        Kind::Changed => "changed",
                        Kind::Added => "added",
                        Kind::Removed => "removed",
                        Kind::Collapsed => "model_params",
                    };
                    let mut v = vec![
                        ("kind", jstr(kind)),
                        ("path", jstr(&i.path)),
                        ("section", jstr(&i.section)),
                    ];
                    if i.kind == Kind::Collapsed {
                        v.push(("count", Json::Num(i.count.to_string())));
                    } else {
                        v.push(("param", jstr(&i.param)));
                        v.push(("a", jstr(&i.a)));
                        v.push(("b", jstr(&i.b)));
                    }
                    jobj(v)
                })
                .collect();
            jobj(vec![
                ("strip", jstr(&s.label)),
                ("category", jstr(&s.cat)),
                (
                    "number",
                    s.num.map_or(Json::Null, |n| Json::Num(n.to_string())),
                ),
                ("changes", Json::Arr(items)),
            ])
        })
        .collect();
    let doc = jobj(vec![
        ("a", scene(&r.a)),
        ("b", scene(&r.b)),
        ("changes", Json::Num(r.changes().to_string())),
        ("ignored_ui", Json::Num(r.ignored.to_string())),
        ("filtered", Json::Num(r.filtered.to_string())),
        ("strips", Json::Arr(strips)),
    ]);
    let mut t = doc.to_text();
    t.push('\n');
    t
}

#[cfg(test)]
mod tests {
    use super::*;
    use wing_model::{Origin, ORIGIN_SNAPSHOT};

    /// Build a scene text by applying `(path, display text)` edits to the factory scene.
    fn scene(edits: &[(&str, &str)]) -> String {
        let store = ParamStore::new();
        let snap = Snapshot::factory();
        for (p, t) in edits {
            let id = store.lookup(p).unwrap_or_else(|| panic!("no node {p}"));
            let o: Origin = ORIGIN_SNAPSHOT;
            store
                .set_text(id, t, o)
                .unwrap_or_else(|e| panic!("{p}={t}: {e:?}"));
        }
        snap.save(&store)
    }

    fn pair(a: &[(&str, &str)], b: &[(&str, &str)], o: &Options) -> Report {
        diff_scenes(
            &load_text("a.snap", &scene(a)).unwrap(),
            &load_text("b.snap", &scene(b)).unwrap(),
            o,
        )
    }

    #[test]
    fn synthetic_pair_exact_text() {
        let base = [
            ("ch/3/name", "Snare Top"),
            ("bus/2/name", "Drum Verb"),
            ("ch/3/eq/on", "1"),
            ("ch/3/eq/2g", "1.5"),
            ("ch/3/dyn/thr", "-18"),
            ("ch/3/fdr", "-4.2"),
            ("ch/3/send/2/lvl", "-10"),
            ("ch/3/mute", "0"),
        ];
        let changed = [
            ("ch/3/name", "Snare Top"),
            ("bus/2/name", "Drum Verb"),
            ("ch/3/eq/on", "1"),
            ("ch/3/eq/2g", "3.0"),
            ("ch/3/dyn/thr", "-22"),
            ("ch/3/fdr", "-2.0"),
            ("ch/3/send/2/lvl", "-5"),
            ("ch/3/mute", "1"),
            ("ch/5/name", "Kick"),
            ("main/1/fdr", "-3"),
        ];
        let r = pair(&base, &changed, &Options::default());
        let t = render(&r, Format::Text);
        let expect = "\
A: a.snap (snapshot.11, fw 3.0.6-193-g60c910ab-dirty:dev_unidyn)
B: b.snap (snapshot.11, fw 3.0.6-193-g60c910ab-dirty:dev_unidyn)

Ch 3 'Snare Top'
  mute: off \u{2192} on
  fader: -4.2 dB \u{2192} -2.0 dB
  EQ band 2 gain: +1.5 dB \u{2192} +3.0 dB
  COMP threshold: -18.0 dB \u{2192} -22.0 dB
  Send \u{2192} Bus 2 'Drum Verb' level: -10.0 dB \u{2192} -5.0 dB

Ch 5 'Kick'
  name: '' \u{2192} 'Kick'

Main 1
  fader: -oo dB \u{2192} -3.0 dB

7 changes in 3 groups
";
        assert_eq!(t, expect);
    }

    #[test]
    fn model_change_collapses_model_params() {
        let r = pair(
            &[("ch/1/dyn/mdl", "COMP")],
            &[("ch/1/dyn/mdl", "76LA")],
            &Options::default(),
        );
        let t = render(&r, Format::Text);
        assert!(t.contains("Dyn model: COMP \u{2192} 76LA"), "{t}");
        assert!(t.contains("model-specific parameters differ"), "{t}");
        assert!(!t.contains("threshold"), "{t}");
    }

    #[test]
    fn filter_and_ignore_ui() {
        let a = [("ch/1/fdr", "0"), ("ch/1/led", "1"), ("bus/1/fdr", "0")];
        let b = [("ch/1/fdr", "-5"), ("ch/1/led", "0"), ("bus/1/fdr", "-5")];
        let r = pair(&a, &b, &Options::default());
        assert_eq!(r.changes(), 3);
        let o = Options {
            ignore_ui: true,
            filter: Some(Filter::parse("ch/1..40").unwrap()),
        };
        let r = pair(&a, &b, &o);
        assert_eq!(r.changes(), 1);
        assert_eq!(r.ignored, 1);
        assert_eq!(r.filtered, 1);
        assert!(Filter::parse("ch/x").is_err());
    }

    #[test]
    fn formats_render() {
        let r = pair(&[], &[("ch/2/name", "Vox")], &Options::default());
        let j = render(&r, Format::Json);
        let doc = Json::parse(&j).unwrap();
        assert_eq!(doc.get("changes").and_then(Json::as_f64), Some(1.0));
        assert!(j.contains("\"b\":\"'Vox'\""));
        let m = render(&r, Format::Markdown);
        assert!(m.contains("## Ch 2 'Vox'") && m.contains("- **name**: `''` \u{2192} `'Vox'`"));
        let same = pair(&[], &[], &Options::default());
        assert!(render(&same, Format::Text).contains("No differences."));
    }
}
