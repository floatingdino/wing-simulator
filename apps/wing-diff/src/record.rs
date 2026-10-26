//! Recording kit (`wing-record-setup`): from a show scene, build
//!
//! - a **scoped recording scene** (`.snap` holding only the recording patch): every recorded
//!   strip's main source is sent to its own USB output, the strip's ALT source is that USB channel
//!   (ALT AUTOSW on, so the global ALT INPUT switch flips the whole console to DAW playback), the
//!   global ALT switch is off and the USB audio config is 48/48. Nothing else is in the file, so
//!   loading it changes only alt sources, the USB in/out patch and `io/altsw`;
//! - a **DAW project** (REAPER `.RPP`): one track per recorded strip, named after it, recording
//!   input = its USB channel(s), record-armed with input monitoring off, hardware output = the
//!   same USB channel(s) (that is the strip's alt source on playback), no master send.
//!
//! USB channels are allocated in strip order (`ch/1..40`, then `aux/1..8`); a stereo source
//! takes an odd-aligned pair, and strips sharing a source share its USB channel(s) and track. Strips whose main source is OFF, USB (already DAW playback) or a
//! mix re-input (BUS/MAIN/MTX) are skipped, as are strips beyond the 48 USB channels; the plan
//! lists why.

use std::fmt::Write as _;

use wing_model::json::Json;
use wing_model::{ParamStore, Snapshot};

use crate::Loaded;

/// USB audio channels each way (`ae_globals/usbacfg` 48/48).
pub const USB_CHANNELS: u32 = 48;

/// Source groups a recording split can take (raw inputs; not USB itself or mix re-inputs).
const RECORDABLE: [&str; 11] = [
    "LCL", "AUX", "A", "B", "C", "SC", "CRD", "MOD", "PLAY", "AES", "USR",
];

/// Which strips to record.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Selection {
    /// Input channels (1-40).
    pub channels: Vec<u32>,
    /// Aux strips (1-8).
    pub aux: Vec<u32>,
}

impl Default for Selection {
    fn default() -> Self {
        Self {
            channels: (1..=40).collect(),
            aux: (1..=8).collect(),
        }
    }
}

/// One recorded source and the strips that use it.
#[derive(Clone, Debug, PartialEq)]
pub struct Slot {
    /// `ch/3`, `aux/1`.
    pub strips: Vec<String>,
    /// Track name (strip names, else source name, else `Ch 3`; ` / `-joined when shared).
    pub name: String,
    /// Main source group and first index (`A`, 1).
    pub grp: String,
    pub input: u32,
    /// First USB channel (1-based); `stereo` takes `usb` and `usb + 1`.
    pub usb: u32,
    pub stereo: bool,
}

/// The allocation plus the strips that were left out.
#[derive(Clone, Debug, Default)]
pub struct Plan {
    pub slots: Vec<Slot>,
    /// `(strip, reason)`.
    pub skipped: Vec<(String, String)>,
    /// Header of the source scene (3.x after migration).
    header: Vec<(String, Json)>,
}

fn value(store: &ParamStore, path: &str) -> Option<Json> {
    Snapshot::value_json(store, store.lookup(path)?)
}

fn text(store: &ParamStore, path: &str) -> String {
    match value(store, path) {
        Some(Json::Str(s)) => s.trim().to_string(),
        _ => String::new(),
    }
}

fn int(store: &ParamStore, path: &str) -> Option<u32> {
    value(store, path)?.as_f64().map(|v| v as u32)
}

/// Allocate USB channels for `sel` from the scene's current input patch.
pub fn plan(scene: &Loaded, sel: &Selection) -> Plan {
    let st = &scene.store;
    let mut plan = Plan {
        header: scene.snap.header.clone(),
        ..Plan::default()
    };
    let mut next = 1u32;
    let strips = sel
        .channels
        .iter()
        .map(|n| ("ch", *n))
        .chain(sel.aux.iter().map(|n| ("aux", *n)));
    for (kind, n) in strips {
        let strip = format!("{kind}/{n}");
        let grp = text(st, &format!("{strip}/in/conn/grp"));
        let Some(input) = int(st, &format!("{strip}/in/conn/in")) else {
            plan.skipped.push((strip, "no such strip".into()));
            continue;
        };
        if !RECORDABLE.contains(&grp.as_str()) {
            let why = match grp.as_str() {
                "OFF" | "" => "no main source".to_string(),
                "USB" => "main source is already USB (DAW playback)".to_string(),
                g => format!("main source {g} is a mix re-input"),
            };
            plan.skipped.push((strip, why));
            continue;
        }
        let stereo = text(st, &format!("io/in/{grp}/{input}/mode")) == "ST";
        let name = [
            text(st, &format!("{strip}/name")),
            text(st, &format!("io/in/{grp}/{input}/name")),
        ]
        .into_iter()
        .find(|s| !s.is_empty())
        .unwrap_or_else(|| format!("{} {n}", if kind == "ch" { "Ch" } else { "Aux" }));
        if let Some(sl) = plan
            .slots
            .iter_mut()
            .find(|s| s.grp == grp && s.input == input && s.stereo == stereo)
        {
            sl.strips.push(strip);
            if !sl.name.split(" / ").any(|x| x == name) {
                sl.name = format!("{} / {name}", sl.name);
            }
            continue;
        }
        let usb = if stereo { next + (next + 1) % 2 } else { next };
        let last = usb + stereo as u32;
        if last > USB_CHANNELS {
            plan.skipped
                .push((strip, format!("no USB channels left (needs {usb}..{last})")));
            continue;
        }
        next = last + 1;
        plan.slots.push(Slot {
            strips: vec![strip],
            name,
            grp,
            input,
            usb,
            stereo,
        });
    }
    plan
}

fn obj(v: Vec<(&str, Json)>) -> Json {
    Json::Obj(v.into_iter().map(|(k, v)| (k.to_string(), v)).collect())
}

fn num(n: u32) -> Json {
    Json::Num(n.to_string())
}

fn s(v: &str) -> Json {
    Json::Str(v.to_string())
}

/// Insert `v` at `path` (object keys), creating objects on the way.
fn put(doc: &mut Json, path: &[&str], v: Json) {
    let Json::Obj(m) = doc else { return };
    let (first, rest) = path.split_first().expect("non-empty path");
    let i = match m.iter().position(|(k, _)| k == first) {
        Some(i) => i,
        None => {
            m.push((first.to_string(), Json::Obj(Vec::new())));
            m.len() - 1
        }
    };
    if rest.is_empty() {
        m[i].1 = v;
    } else {
        put(&mut m[i].1, rest, v);
    }
}

impl Plan {
    /// The scoped recording scene (`.snap` text, 3.x layout, only the recording patch).
    pub fn scene(&self) -> String {
        let mut doc = Json::Obj(
            self.header
                .iter()
                .map(|(k, v)| {
                    let v = if k == "creator_name" {
                        s("RECORDING")
                    } else {
                        v.clone()
                    };
                    (k.clone(), v)
                })
                .collect(),
        );
        put(&mut doc, &["ae_data", "io", "altsw"], Json::Bool(false));
        // every USB out is defined, so recalling the scene also clears stale patches
        let mut outs: Vec<Option<(&str, u32)>> = vec![None; USB_CHANNELS as usize];
        let mut modes = vec!["M"; USB_CHANNELS as usize];
        for sl in &self.slots {
            let legs = if sl.stereo { 2 } else { 1 };
            for l in 0..legs {
                outs[(sl.usb + l - 1) as usize] = Some((sl.grp.as_str(), sl.input + l));
                if sl.stereo {
                    modes[(sl.usb + l - 1) as usize] = "ST";
                }
            }
        }
        for (i, o) in outs.iter().enumerate() {
            let k = (i + 1).to_string();
            let (g, n) = o.unwrap_or(("OFF", 1));
            put(
                &mut doc,
                &["ae_data", "io", "out", "USB", &k],
                obj(vec![("grp", s(g)), ("in", num(n))]),
            );
            put(
                &mut doc,
                &["ae_data", "io", "in", "USB", &k, "mode"],
                s(modes[i]),
            );
        }
        for sl in &self.slots {
            for strip in &sl.strips {
                let (kind, n) = strip.split_once('/').expect("strip path");
                put(
                    &mut doc,
                    &["ae_data", kind, n, "in", "set"],
                    obj(vec![
                        ("srcauto", Json::Bool(true)),
                        ("altsrc", Json::Bool(false)),
                    ]),
                );
                put(
                    &mut doc,
                    &["ae_data", kind, n, "in", "conn"],
                    obj(vec![("altgrp", s("USB")), ("altin", num(sl.usb))]),
                );
            }
        }
        put(&mut doc, &["ae_globals", "usbacfg"], s("48/48"));
        doc.to_text()
    }

    /// A REAPER project: one armed track per slot on the USB channels (see module docs).
    pub fn rpp(&self) -> String {
        let mut o = String::new();
        let _ = writeln!(o, "<REAPER_PROJECT 0.1 \"7.0\" 0");
        let _ = writeln!(o, "  RECORD_PATH \"Media\" \"\"");
        let _ = writeln!(o, "  SAMPLERATE 48000 1 0");
        for sl in &self.slots {
            let ch = sl.usb - 1;
            // REC armed input monitor ...: mono input = index, stereo pair = 1024 + first index
            // HWOUT dest ... srcch: dest | 1024 = mono output; srcch 1024 = mono source ch 1
            let (rec_in, dest, src) = if sl.stereo {
                (1024 + ch, ch, 0)
            } else {
                (ch, 1024 + ch, 1024)
            };
            let _ = writeln!(o, "  <TRACK");
            let _ = writeln!(o, "    NAME \"{}\"", sl.name.replace('"', "'"));
            let _ = writeln!(o, "    NCHAN 2");
            let _ = writeln!(o, "    REC 1 {rec_in} 0 0 0 0 0 0");
            let _ = writeln!(o, "    MAINSEND 0 0");
            let _ = writeln!(o, "    HWOUT {dest} 0 1 0 0 0 {src} -1:U -1");
            let _ = writeln!(o, "  >");
        }
        let _ = writeln!(o, ">");
        o
    }

    /// Human-readable patch list.
    pub fn summary(&self) -> String {
        let mut o = String::new();
        for sl in &self.slots {
            let usb = if sl.stereo {
                format!("USB {}-{}", sl.usb, sl.usb + 1)
            } else {
                format!("USB {}", sl.usb)
            };
            let src = if sl.stereo {
                format!("{}.{}-{}", sl.grp, sl.input, sl.input + 1)
            } else {
                format!("{}.{}", sl.grp, sl.input)
            };
            let strips = sl.strips.join(",");
            let _ = writeln!(o, "{strips:<7} {usb:<10} <- {src:<8} {:?}", sl.name);
        }
        for (strip, why) in &self.skipped {
            let _ = writeln!(o, "{strip:<7} skipped: {why}");
        }
        o
    }
}

/// Parse a strip list: `1-40`, `1,3,5-8`, `none`.
pub fn parse_list(v: &str, max: u32) -> Result<Vec<u32>, String> {
    if v == "none" {
        return Ok(Vec::new());
    }
    let mut out = Vec::new();
    for part in v.split(',') {
        let (a, b) = part.split_once('-').unwrap_or((part, part));
        let p = |x: &str| {
            x.trim()
                .parse::<u32>()
                .map_err(|_| format!("bad number in '{v}'"))
        };
        let (a, b) = (p(a)?, p(b)?);
        if a == 0 || b > max || a > b {
            return Err(format!("'{part}' is outside 1-{max}"));
        }
        out.extend(a..=b);
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::load_text;
    use wing_model::FACTORY_SCENE;

    fn with(edits: &[(&str, &str)]) -> Loaded {
        let mut doc = Json::parse(FACTORY_SCENE).unwrap();
        for (path, v) in edits {
            let p: Vec<&str> = path.split('/').collect();
            put(&mut doc, &p, Json::parse(v).unwrap());
        }
        load_text("t.snap", &doc.to_text()).unwrap()
    }

    #[test]
    fn allocates_mono_and_stereo_and_skips() {
        let scene = with(&[
            ("ae_data/ch/2/name", "\"Kick\""),
            ("ae_data/ch/3/in/conn/grp", "\"A\""),
            ("ae_data/ch/3/in/conn/in", "5"),
            ("ae_data/io/in/A/5/mode", "\"ST\""),
            ("ae_data/io/in/A/6/mode", "\"ST\""),
            ("ae_data/io/in/A/5/name", "\"Keys\""),
            ("ae_data/ch/4/in/conn/grp", "\"USB\""),
            ("ae_data/ch/5/in/conn/grp", "\"BUS\""),
            ("ae_data/ch/7/in/conn/in", "2"),
            ("ae_data/ch/7/name", "\"Kick Sub\""),
        ]);
        let sel = Selection {
            channels: vec![1, 2, 3, 4, 5, 6, 7],
            aux: vec![],
        };
        let p = plan(&scene, &sel);
        let got: Vec<_> = p
            .slots
            .iter()
            .map(|s| {
                (
                    s.strips.join(","),
                    s.name.as_str(),
                    s.grp.as_str(),
                    s.input,
                    s.usb,
                    s.stereo,
                )
            })
            .collect();
        // factory: ch 1-8 on LCL 1-8; ch 3 moved to the stereo pair A.5/6 -> odd-aligned USB 3/4
        assert_eq!(
            got,
            [
                ("ch/1".to_string(), "Ch 1", "LCL", 1, 1, false),
                // ch 7 shares LCL.2 with ch 2: same USB channel, one track
                (
                    "ch/2,ch/7".to_string(),
                    "Kick / Kick Sub",
                    "LCL",
                    2,
                    2,
                    false
                ),
                ("ch/3".to_string(), "Keys", "A", 5, 3, true),
                ("ch/6".to_string(), "Ch 6", "LCL", 6, 5, false),
            ]
        );
        assert_eq!(p.skipped.len(), 2);
        assert!(p.skipped[0].1.contains("already USB"));
        assert!(p.skipped[1].1.contains("re-input"));
    }

    #[test]
    fn scene_is_scoped_and_loads_cleanly() {
        let scene = load_text("f.snap", FACTORY_SCENE).unwrap();
        let p = plan(&scene, &Selection::default());
        // factory: ch 1-8 on LCL 1-8, aux 2 on the stereo player PLAY.1/2 (USB 9/10)
        assert_eq!(p.slots.len(), 9);
        assert_eq!((p.slots[8].usb, p.slots[8].stereo), (9, true));
        let text = p.scene();
        let doc = Json::parse(&text).unwrap();
        let ae = doc.get("ae_data").unwrap();
        let mut keys: Vec<&str> = match ae {
            Json::Obj(m) => m.iter().map(|(k, _)| k.as_str()).collect(),
            _ => panic!(),
        };
        keys.sort();
        assert_eq!(
            keys,
            ["aux", "ch", "io"],
            "nothing outside the recording scope"
        );
        assert!(doc.get("ce_data").is_none());

        let back = load_text("rec.snap", &text).unwrap();
        assert!(
            back.report.unmapped.is_empty(),
            "{:?}",
            back.report.unmapped
        );
        assert!(
            back.report.rejected.is_empty(),
            "{:?}",
            back.report.rejected
        );
        let v = |path: &str| value(&back.store, path).unwrap().to_text();
        assert_eq!(v("io/out/USB/1/grp"), "\"LCL\"");
        assert_eq!(v("io/out/USB/8/in"), "8");
        assert_eq!(v("io/out/USB/11/grp"), "\"OFF\"");
        assert_eq!(v("ch/8/in/conn/altgrp"), "\"USB\"");
        assert_eq!(v("ch/8/in/conn/altin"), "8");
        assert_eq!(v("ch/8/in/set/srcauto"), "true");
        assert_eq!(v("io/altsw"), "false");
        assert_eq!(v("io/in/USB/10/mode"), "\"ST\"");
        assert_eq!(v("aux/2/in/conn/altin"), "9");
    }

    #[test]
    fn rpp_round_trips_through_the_renderer_layout() {
        let scene = with(&[
            ("ae_data/ch/1/name", "\"Kick \\\"In\\\"\""),
            ("ae_data/ch/2/in/conn/grp", "\"A\""),
            ("ae_data/ch/2/in/conn/in", "1"),
            ("ae_data/io/in/A/1/mode", "\"ST\""),
            ("ae_data/io/in/A/2/mode", "\"ST\""),
        ]);
        let sel = Selection {
            channels: vec![1, 2],
            aux: vec![],
        };
        let r = plan(&scene, &sel).rpp();
        assert!(r.contains("NAME \"Kick 'In'\""));
        assert!(r.contains("REC 1 0 0 0 0 0 0 0\n"), "{r}");
        assert!(r.contains("HWOUT 1024 0 1 0 0 0 1024 -1:U -1"));
        // stereo strip on USB 3/4
        assert!(r.contains("REC 1 1026 0 0 0 0 0 0\n"), "{r}");
        assert!(r.contains("HWOUT 2 0 1 0 0 0 0 -1:U -1"));
        assert_eq!(r.matches("MAINSEND 0 0").count(), 2);
    }

    /// The generic kit in `integrations/recording/`: ch 1-40 on AES50 A.1-40, aux 1-8 on the
    /// local AUX 1-8 line inputs. `WING_REGEN_KIT=1 cargo test -p wing-diff shipped_kit` rewrites it.
    #[test]
    fn shipped_kit_is_current() {
        let mut edits = Vec::new();
        for n in 1..=40 {
            edits.push((format!("ae_data/ch/{n}/in/conn/grp"), "\"A\"".to_string()));
            edits.push((format!("ae_data/ch/{n}/in/conn/in"), n.to_string()));
        }
        for n in 1..=8 {
            edits.push((
                format!("ae_data/aux/{n}/in/conn/grp"),
                "\"AUX\"".to_string(),
            ));
            edits.push((format!("ae_data/aux/{n}/in/conn/in"), n.to_string()));
            edits.push((format!("ae_data/io/in/AUX/{n}/mode"), "\"M\"".to_string()));
        }
        let edits: Vec<(&str, &str)> = edits
            .iter()
            .map(|(a, b)| (a.as_str(), b.as_str()))
            .collect();
        let p = plan(&with(&edits), &Selection::default());
        assert_eq!(p.slots.len(), 48);
        assert!(p.skipped.is_empty());
        let dir = concat!(env!("CARGO_MANIFEST_DIR"), "/../../integrations/recording");
        for (name, text) in [
            ("Recording-Scene.snap", p.scene()),
            ("Recording.RPP", p.rpp()),
        ] {
            let path = format!("{dir}/{name}");
            if std::env::var_os("WING_REGEN_KIT").is_some() {
                std::fs::create_dir_all(dir).unwrap();
                std::fs::write(&path, &text).unwrap();
            }
            let shipped = std::fs::read_to_string(&path).unwrap_or_default();
            assert!(
                shipped == text,
                "{path} is stale: WING_REGEN_KIT=1 cargo test -p wing-diff shipped_kit"
            );
        }
    }

    #[test]
    fn lists() {
        assert_eq!(parse_list("1-3,7", 40).unwrap(), [1, 2, 3, 7]);
        assert!(parse_list("none", 8).unwrap().is_empty());
        assert!(parse_list("0-3", 40).is_err());
        assert!(parse_list("39-41", 40).is_err());
        assert!(parse_list("x", 40).is_err());
    }
}
