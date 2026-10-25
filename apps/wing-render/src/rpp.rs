//! Read-only REAPER project (`.RPP`) import: builds the USB-input map from the tracks' hardware
//! outputs (see `docs/research/driver.md` §7: with REAPER on the "Wing Simulator" device, hardware
//! output n is the WING's USB input n).
//!
//! Per track: `NAME`, `MUTESOLO` (muted tracks are skipped), every unmuted `HWOUT` (destination
//! `index & 1023` = first output, 0-based; `| 1024` = mono, else a stereo pair; the source-channel
//! field `| 1024` = one mono source channel, `-1` = no audio, else the pair starting there) and the
//! track's first `<ITEM>` with a `<SOURCE WAVE>` (`FILE`, `POSITION`, `SOFFS`, `LENGTH`, `MUTE`).
//! Relative `FILE` paths are resolved against the project directory, then its `RECORD_PATH`.
//!
//! Limits (reported as notes where they apply): only the first audio item per track is used (no
//! edits, crossfades, fades, take/item/track volume, pan, FX or envelopes: the WING scene does the
//! mixing); a mono output fed from a stereo source plays the mono mix (L+R)/2; non-WAV sources,
//! folder-track and master hardware outputs (`MASTERHWOUT`) are ignored; the project's sample
//! rate must equal the files' (no resampling). The file is never written.

use std::path::{Path, PathBuf};

use crate::inputs::{InputPlan, Pick, Source};

/// Split one RPP line into tokens (quotes `"`, `'` and `` ` `` delimit strings).
fn tokens(line: &str) -> Vec<String> {
    let mut out = Vec::new();
    let mut it = line.trim().chars().peekable();
    while let Some(&c) = it.peek() {
        if c.is_whitespace() {
            it.next();
            continue;
        }
        if c == '"' || c == '\'' || c == '`' {
            it.next();
            let mut s = String::new();
            for d in it.by_ref() {
                if d == c {
                    break;
                }
                s.push(d);
            }
            out.push(s);
        } else {
            let mut s = String::new();
            while let Some(&d) = it.peek() {
                if d.is_whitespace() {
                    break;
                }
                s.push(d);
                it.next();
            }
            out.push(s);
        }
    }
    out
}

#[derive(Debug, Default, Clone)]
struct Item {
    position: f64,
    soffs: f64,
    length: Option<f64>,
    file: Option<String>,
    muted: bool,
    other_source: Option<String>,
}

#[derive(Debug, Default, Clone)]
struct Track {
    name: String,
    muted: bool,
    /// (dest field, src channel field)
    hwouts: Vec<(i64, i64)>,
    items: Vec<Item>,
}

/// A parsed project: tracks in order plus project settings.
#[derive(Debug, Default, Clone)]
struct Project {
    tracks: Vec<Track>,
    record_path: Option<String>,
    sample_rate: Option<u32>,
}

fn parse(text: &str) -> Project {
    let mut p = Project::default();
    let mut stack: Vec<String> = Vec::new();
    for line in text.lines() {
        let t = line.trim();
        if t == ">" {
            stack.pop();
            continue;
        }
        let tok = tokens(t);
        let Some(first) = tok.first() else { continue };
        if let Some(block) = first.strip_prefix('<') {
            let block = block.to_ascii_uppercase();
            match block.as_str() {
                "TRACK" => p.tracks.push(Track::default()),
                "ITEM" if stack.last().is_some_and(|b| b == "TRACK") => {
                    if let Some(tr) = p.tracks.last_mut() {
                        tr.items.push(Item::default());
                    }
                }
                "SOURCE" if stack.iter().any(|b| b == "ITEM") => {
                    let kind = tok
                        .get(1)
                        .map(|s| s.to_ascii_uppercase())
                        .unwrap_or_default();
                    if !matches!(kind.as_str(), "WAVE" | "SECTION") {
                        if let Some(it) = p.tracks.last_mut().and_then(|t| t.items.last_mut()) {
                            it.other_source.get_or_insert(kind);
                        }
                    }
                }
                _ => {}
            }
            stack.push(block);
            continue;
        }
        let num = |i: usize| tok.get(i).and_then(|s| s.parse::<f64>().ok());
        let int = |i: usize| tok.get(i).and_then(|s| s.parse::<i64>().ok());
        match (stack.last().map(String::as_str), first.as_str()) {
            (Some("REAPER_PROJECT"), "RECORD_PATH") => {
                p.record_path = tok.get(1).filter(|s| !s.is_empty()).cloned()
            }
            (Some("REAPER_PROJECT"), "SAMPLERATE") => p.sample_rate = num(1).map(|r| r as u32),
            (Some("TRACK"), "NAME") => {
                if let Some(tr) = p.tracks.last_mut() {
                    tr.name = tok.get(1).cloned().unwrap_or_default();
                }
            }
            (Some("TRACK"), "MUTESOLO") => {
                if let Some(tr) = p.tracks.last_mut() {
                    tr.muted = int(1).is_some_and(|m| m != 0);
                }
            }
            (Some("TRACK"), "HWOUT") => {
                // dest, mode, vol, pan, mute, phase, source channel, ...
                let muted = int(5).is_some_and(|m| m != 0);
                if let (Some(tr), Some(d), false) = (p.tracks.last_mut(), int(1), muted) {
                    tr.hwouts.push((d, int(7).unwrap_or(0)));
                }
            }
            (Some("ITEM"), k) => {
                if let Some(it) = p.tracks.last_mut().and_then(|t| t.items.last_mut()) {
                    match k {
                        "POSITION" => it.position = num(1).unwrap_or(0.0),
                        "SOFFS" => it.soffs = num(1).unwrap_or(0.0),
                        "LENGTH" => it.length = num(1),
                        "MUTE" => it.muted = int(1).is_some_and(|m| m != 0),
                        _ => {}
                    }
                }
            }
            (Some("SOURCE"), "FILE") => {
                if let Some(it) = p.tracks.last_mut().and_then(|t| t.items.last_mut()) {
                    if it.file.is_none() && it.other_source.is_none() {
                        it.file = tok.get(1).cloned();
                    }
                }
            }
            _ => {}
        }
    }
    p
}

fn resolve(file: &str, dir: &Path, record_path: Option<&str>) -> PathBuf {
    let f = Path::new(file);
    if f.is_absolute() {
        return f.to_path_buf();
    }
    let mut cands = vec![dir.join(f)];
    if let Some(rp) = record_path {
        let rp = Path::new(rp);
        let base = if rp.is_absolute() {
            rp.to_path_buf()
        } else {
            dir.join(rp)
        };
        cands.push(base.join(f));
        if let Some(name) = f.file_name() {
            cands.push(base.join(name));
        }
    }
    cands
        .iter()
        .find(|p| p.exists())
        .cloned()
        .unwrap_or_else(|| cands[0].clone())
}

/// Build the USB-input plan from the project at `path` for `rate` (item positions in frames).
pub fn plan_from_rpp(path: &Path, rate: u32) -> Result<InputPlan, String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
    if !text.trim_start().starts_with("<REAPER_PROJECT") {
        return Err(format!("{}: not a REAPER project", path.display()));
    }
    let dir = path.parent().unwrap_or(Path::new("."));
    let proj = parse(&text);
    let mut plan = InputPlan::default();
    if let Some(r) = proj.sample_rate.filter(|&r| r > 0 && r != rate) {
        plan.notes.push(format!(
            "project SAMPLERATE {r} differs from the render rate {rate} (files must be {rate} Hz)"
        ));
    }
    let frames = |s: f64| (s * rate as f64).round().max(0.0) as u64;
    for (i, tr) in proj.tracks.iter().enumerate() {
        let name = if tr.name.is_empty() {
            format!("track {}", i + 1)
        } else {
            format!("track {} \"{}\"", i + 1, tr.name)
        };
        if tr.hwouts.is_empty() {
            continue;
        }
        if tr.muted {
            plan.notes.push(format!("{name}: muted, skipped"));
            continue;
        }
        let items: Vec<&Item> = tr.items.iter().filter(|it| !it.muted).collect();
        let Some(item) = items.iter().find(|it| it.file.is_some()) else {
            if let Some(k) = items.iter().find_map(|it| it.other_source.clone()) {
                plan.notes.push(format!(
                    "{name}: {k} source not supported (WAV only), skipped"
                ));
            }
            continue;
        };
        if items.len() > 1 {
            plan.notes.push(format!(
                "{name}: {} items, only the first audio item is used",
                items.len()
            ));
        }
        let file = resolve(
            item.file.as_deref().unwrap_or_default(),
            dir,
            proj.record_path.as_deref(),
        );
        if !file.exists() {
            plan.notes
                .push(format!("{name}: {} not found, skipped", file.display()));
            continue;
        }
        let channels = wing_sim::wav::WavReader::open(&file)
            .map_err(|e| format!("{}: {e}", file.display()))?
            .info()
            .channels as usize;
        let mut lanes = Vec::new();
        for &(dest, src) in &tr.hwouts {
            let first = (dest & 1023) as u16 + 1;
            let mono_out = dest & 1024 != 0;
            if src < 0 {
                continue;
            }
            let (mono_src, sch) = (src & 1024 != 0, (src & 1023) as usize);
            let picks: Vec<Pick> = match (mono_out, mono_src) {
                (_, true) => vec![Pick::Channel(sch.min(channels - 1))],
                (true, false) if channels == 1 => vec![Pick::Channel(0)],
                (true, false) => {
                    plan.notes.push(format!(
                        "{name}: mono output {first} from a {channels}-channel source: mono mix"
                    ));
                    vec![Pick::Mix]
                }
                (false, false) if channels == 1 => vec![Pick::Channel(0), Pick::Channel(0)],
                (false, false) => vec![
                    Pick::Channel(sch.min(channels - 1)),
                    Pick::Channel((sch + 1).min(channels - 1)),
                ],
            };
            let picks = if mono_out { &picks[..1] } else { &picks[..] };
            for (k, &p) in picks.iter().enumerate() {
                lanes.push((p, first + k as u16));
            }
        }
        if lanes.is_empty() {
            continue;
        }
        let label = format!(
            "{name} ({})",
            file.file_name()
                .map_or(String::new(), |n| n.to_string_lossy().into())
        );
        plan.sources.push(Source {
            path: file,
            position: frames(item.position),
            src_offset: frames(item.soffs),
            length: item.length.map(frames),
            lanes,
            label,
        });
    }
    if plan.sources.is_empty() {
        return Err(format!(
            "{}: no track with a hardware output and a WAV item",
            path.display()
        ));
    }
    Ok(plan)
}

#[cfg(test)]
mod tests {
    use super::*;
    use wing_sim::wav::WavWriter;

    #[test]
    fn tokenizer() {
        assert_eq!(
            tokens(r#"  NAME "Lead 'Vox'" 1 `a "b"` x"#),
            ["NAME", "Lead 'Vox'", "1", "a \"b\"", "x"]
        );
    }

    #[test]
    fn imports_tracks_hwouts_and_items() {
        let d = std::env::temp_dir().join(format!("wingrender-rpp-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&d);
        std::fs::create_dir_all(d.join("Media")).unwrap();
        for (f, ch) in [("Media/kick.wav", 1u16), ("snare.wav", 1), ("keys.wav", 2)] {
            let mut w = WavWriter::create(&d.join(f), ch, 48_000).unwrap();
            w.write_interleaved(&vec![0.1; 4800 * ch as usize]).unwrap();
            w.finish().unwrap();
        }
        let rpp = r#"<REAPER_PROJECT 0.1 "7.34/macOS-arm64" 1751155200
  RECORD_PATH "Media" ""
  SAMPLERATE 48000 0 0
  MASTERHWOUT 0 0 1 0 0 0 0 -1
  <TRACK {A}
    NAME Kick
    MUTESOLO 0 0 0
    HWOUT 1024 1 1 0 0 0 0 -1:U -1
    <ITEM
      POSITION 0.5
      SOFFS 0.01
      LENGTH 0.05
      <SOURCE WAVE
        FILE "kick.wav"
      >
    >
  >
  <TRACK {B}
    NAME "Snare top"
    HWOUT 1025 0 1 0 0 0 0 -1:U -1
    <ITEM
      POSITION 0
      <SOURCE WAVE
        FILE "snare.wav"
      >
    >
    <ITEM
      POSITION 1
      <SOURCE WAVE
        FILE "snare.wav"
      >
    >
  >
  <TRACK {C}
    NAME Keys
    HWOUT 4 0 1 0 0 0 0 -1:U -1
    HWOUT 1030 0 1 0 0 0 0 -1:U -1
    <ITEM
      <SOURCE WAVE
        FILE "keys.wav"
      >
    >
  >
  <TRACK {D}
    NAME Muted
    MUTESOLO 1 0 0
    HWOUT 1031 0 1 0 0 0 0 -1:U -1
  >
  <TRACK {E}
    NAME Folder
  >
  <TRACK {F}
    NAME Synth
    HWOUT 1032 0 1 0 0 0 0 -1:U -1
    <ITEM
      <SOURCE MIDI
        HASDATA 1 960 QN
      >
    >
  >
>
"#;
        let p = d.join("song.RPP");
        std::fs::write(&p, rpp).unwrap();
        let plan = plan_from_rpp(&p, 48_000).unwrap();
        assert_eq!(plan.sources.len(), 3);
        let k = &plan.sources[0];
        assert!(k.path.ends_with("Media/kick.wav"));
        assert_eq!(k.lanes, [(Pick::Channel(0), 1)]);
        assert_eq!(
            (k.position, k.src_offset, k.length),
            (24_000, 480, Some(2400))
        );
        assert_eq!(plan.sources[1].lanes, [(Pick::Channel(0), 2)]);
        assert_eq!(
            plan.sources[2].lanes,
            [(Pick::Channel(0), 5), (Pick::Channel(1), 6), (Pick::Mix, 7)]
        );
        let notes = plan.notes.join("\n");
        assert!(notes.contains("only the first audio item"), "{notes}");
        assert!(notes.contains("\"Muted\": muted"), "{notes}");
        assert!(notes.contains("MIDI source not supported"), "{notes}");
        assert!(notes.contains("mono mix"), "{notes}");
        // read-only: the project is unchanged
        assert_eq!(std::fs::read_to_string(&p).unwrap(), rpp);
        assert!(plan_from_rpp(&d.join("kick.wav"), 48_000).is_err());
        let _ = std::fs::remove_dir_all(&d);
    }
}
