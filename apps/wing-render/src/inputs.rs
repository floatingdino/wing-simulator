//! Input files → USB inputs 1..48.
//!
//! An [`InputPlan`] is a list of [`Source`]s: a WAV file placed on the timeline (`position`,
//! `src_offset`, optional `length`, all in frames) with lanes `file channel (or L+R mono mix) →
//! USB input`. Lanes landing on the same USB input are summed. Plans come from:
//!
//! * [`plan_from_files`]: files (or every `*.wav` in a directory), sorted by name. If every file
//!   name starts with a distinct number (`03-Kick.wav`, `12 Vox.wav`: WING/REAPER multitrack
//!   naming) that number is the first USB input; otherwise files take USB inputs in order. Mono
//!   files take one input, stereo files two (L, R), wider files one per channel.
//! * [`plan_from_map`]: explicit `usb:1=kick.wav,usb:3-4=keys.wav,usb:5=ovh.wav:2` (`:N` picks
//!   channel N of the file, `:mix` the mono mix of all its channels). Only mapped files are used.
//! * `crate::rpp`: a REAPER project's hardware outputs.
//!
//! [`InputReader`] streams a plan as planar USB-input blocks; it is used by the renderer and by
//! the determinism test to feed the realtime path the same signal.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use wing_sim::wav::WavReader;

/// Number of USB inputs.
pub const USB_INPUTS: usize = 48;

/// What a lane takes from its file.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Pick {
    /// One channel (0-based).
    Channel(usize),
    /// The mean of all channels.
    Mix,
}

/// A file on the timeline and where its channels go.
#[derive(Debug, Clone, PartialEq)]
pub struct Source {
    pub path: PathBuf,
    /// Timeline frame where the file's `src_offset` frame plays.
    pub position: u64,
    /// First file frame played.
    pub src_offset: u64,
    /// Frames played at most (`None`: to the end of the file).
    pub length: Option<u64>,
    /// (what, USB input 1-based).
    pub lanes: Vec<(Pick, u16)>,
    /// Label for reports (track name, file name).
    pub label: String,
}

/// All sources of a render.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct InputPlan {
    pub sources: Vec<Source>,
    /// Notes for the user (skipped tracks, approximations).
    pub notes: Vec<String>,
}

impl InputPlan {
    /// Timeline end (frames) given each file's length; opens the files.
    pub fn end_frame(&self) -> Result<u64, String> {
        let mut end = 0;
        for s in &self.sources {
            let r = open(&s.path)?;
            let avail = r.info().frames.saturating_sub(s.src_offset);
            let n = s.length.map_or(avail, |l| l.min(avail));
            end = end.max(s.position + n);
        }
        Ok(end)
    }

    /// One line per lane: `USB 3 <- 03-Kick.wav ch1`.
    pub fn describe(&self) -> Vec<String> {
        let mut v: Vec<(u16, String)> = Vec::new();
        for s in &self.sources {
            for &(p, usb) in &s.lanes {
                let what = match p {
                    Pick::Channel(c) => format!("ch{}", c + 1),
                    Pick::Mix => "mono mix".into(),
                };
                let at = if s.position > 0 || s.src_offset > 0 {
                    format!(" (at frame {}, from {})", s.position, s.src_offset)
                } else {
                    String::new()
                };
                v.push((usb, format!("USB {usb:2} <- {} {what}{at}", s.label)));
            }
        }
        v.sort_by_key(|x| x.0);
        v.into_iter().map(|x| x.1).collect()
    }

    /// Check that every lane targets USB 1..48 and every file opens at `rate`.
    pub fn validate(&self, rate: u32) -> Result<(), String> {
        if self.sources.is_empty() {
            return Err("no input files".into());
        }
        for s in &self.sources {
            let r = open(&s.path)?;
            let info = r.info();
            if info.sample_rate != rate {
                return Err(format!(
                    "{}: sample rate {} Hz, render rate {rate} Hz (no resampling; use --rate {})",
                    s.path.display(),
                    info.sample_rate,
                    info.sample_rate
                ));
            }
            for &(p, usb) in &s.lanes {
                if usb == 0 || usb as usize > USB_INPUTS {
                    return Err(format!(
                        "{}: USB input {usb} out of range 1..{USB_INPUTS}",
                        s.label
                    ));
                }
                if let Pick::Channel(c) = p {
                    if c >= info.channels as usize {
                        return Err(format!(
                            "{}: has {} channel(s), channel {} requested",
                            s.path.display(),
                            info.channels,
                            c + 1
                        ));
                    }
                }
            }
        }
        Ok(())
    }
}

fn open(p: &Path) -> Result<WavReader, String> {
    WavReader::open(p).map_err(|e| format!("{}: {e}", p.display()))
}

/// `*.wav` files of `dir`, sorted by name.
pub fn wavs_in(dir: &Path) -> Result<Vec<PathBuf>, String> {
    let mut v: Vec<PathBuf> = std::fs::read_dir(dir)
        .map_err(|e| format!("{}: {e}", dir.display()))?
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| {
            p.is_file()
                && p.extension().is_some_and(|e| e.eq_ignore_ascii_case("wav"))
                && !p
                    .file_name()
                    .is_some_and(|n| n.to_string_lossy().starts_with('.'))
        })
        .collect();
    v.sort();
    Ok(v)
}

fn leading_number(p: &Path) -> Option<u16> {
    let name = p.file_name()?.to_string_lossy();
    let digits: String = name.chars().take_while(|c| c.is_ascii_digit()).collect();
    digits.parse().ok()
}

fn label(p: &Path) -> String {
    p.file_name()
        .map_or_else(|| p.display().to_string(), |n| n.to_string_lossy().into())
}

/// Plan for `paths` (files or directories) by name order / leading track numbers.
pub fn plan_from_files(paths: &[PathBuf]) -> Result<InputPlan, String> {
    let mut files = Vec::new();
    for p in paths {
        if p.is_dir() {
            files.extend(wavs_in(p)?);
        } else {
            files.push(p.clone());
        }
    }
    files.sort_by_key(|p| label(p));
    files.dedup();
    let nums: Vec<Option<u16>> = files.iter().map(|p| leading_number(p)).collect();
    let numbered = !files.is_empty() && nums.iter().all(|n| n.is_some_and(|n| n >= 1)) && {
        let mut s: Vec<u16> = nums.iter().flatten().copied().collect();
        s.sort_unstable();
        s.windows(2).all(|w| w[0] != w[1])
    };
    let mut plan = InputPlan::default();
    let mut next = 1u16;
    let mut used: BTreeMap<u16, String> = BTreeMap::new();
    for (p, n) in files.iter().zip(nums) {
        let ch = open(p)?.info().channels;
        let first = if numbered { n.unwrap_or(next) } else { next };
        let lanes: Vec<(Pick, u16)> = (0..ch)
            .map(|c| (Pick::Channel(c as usize), first + c))
            .collect();
        for &(_, u) in &lanes {
            if let Some(prev) = used.insert(u, label(p)) {
                return Err(format!(
                    "USB input {u} assigned twice ({prev} and {}); use --map",
                    label(p)
                ));
            }
        }
        next = first + ch;
        plan.sources.push(Source {
            path: p.clone(),
            position: 0,
            src_offset: 0,
            length: None,
            lanes,
            label: label(p),
        });
    }
    if numbered {
        plan.notes
            .push("USB inputs taken from the leading track numbers of the file names".into());
    }
    Ok(plan)
}

/// Plan for an explicit `--map` (`usb:N=file[:chan|:mix]`, `usb:N-M=file`), file names relative
/// to `base` unless absolute.
pub fn plan_from_map(map: &str, base: Option<&Path>) -> Result<InputPlan, String> {
    let mut plan = InputPlan::default();
    for item in map.split(',').map(str::trim).filter(|s| !s.is_empty()) {
        let (dst, file) = item
            .split_once('=')
            .ok_or_else(|| format!("--map '{item}': expected usb:N=file"))?;
        let dst = dst.trim();
        let range = dst
            .strip_prefix("usb:")
            .or_else(|| dst.strip_prefix("USB:"))
            .ok_or_else(|| format!("--map '{item}': destination must be usb:N or usb:N-M"))?;
        let num = |s: &str| -> Result<u16, String> {
            s.trim()
                .parse::<u16>()
                .map_err(|_| format!("--map '{item}': bad USB input '{s}'"))
        };
        let (a, b) = match range.split_once('-') {
            Some((a, b)) => (num(a)?, num(b)?),
            None => (num(range)?, num(range)?),
        };
        if a == 0 || b < a {
            return Err(format!("--map '{item}': bad range"));
        }
        // optional ":N" / ":mix" channel pick (a drive letter-free unix path has no other ':')
        let (fname, pick) = match file.rsplit_once(':') {
            Some((f, "mix")) => (f, Some(Pick::Mix)),
            Some((f, c)) if c.parse::<usize>().is_ok_and(|c| c >= 1) => {
                (f, Some(Pick::Channel(c.parse::<usize>().unwrap_or(1) - 1)))
            }
            _ => (file, None),
        };
        let path = match base {
            Some(b) if !Path::new(fname).is_absolute() => b.join(fname),
            _ => PathBuf::from(fname),
        };
        let lanes: Vec<(Pick, u16)> = match pick {
            Some(p) => {
                if a != b {
                    return Err(format!("--map '{item}': a channel pick maps to one input"));
                }
                vec![(p, a)]
            }
            None => {
                let ch = open(&path)?.info().channels;
                let width = b - a + 1;
                if width == 1 && ch > 1 {
                    plan.notes.push(format!(
                        "{}: {ch} channels on one input (USB {a}): mono mix",
                        label(&path)
                    ));
                    vec![(Pick::Mix, a)]
                } else if width != ch {
                    return Err(format!(
                        "--map '{item}': file has {ch} channel(s), range has {width}"
                    ));
                } else {
                    (0..ch)
                        .map(|c| (Pick::Channel(c as usize), a + c))
                        .collect()
                }
            }
        };
        plan.sources.push(Source {
            label: label(&path),
            path,
            position: 0,
            src_offset: 0,
            length: None,
            lanes,
        });
    }
    if plan.sources.is_empty() {
        return Err("--map is empty".into());
    }
    Ok(plan)
}

// ---- streaming ----------------------------------------------------------------------------------

struct Open {
    r: WavReader,
    src: Source,
    channels: usize,
    buf: Vec<f32>,
}

/// Streams an [`InputPlan`] as planar USB-input buffers.
pub struct InputReader {
    files: Vec<Open>,
    /// Next timeline frame.
    pos: u64,
}

impl InputReader {
    /// Open every file; the first frame read is timeline frame `start`.
    pub fn open(plan: &InputPlan, start: u64) -> Result<Self, String> {
        let mut files = Vec::new();
        for s in &plan.sources {
            let r = open(&s.path)?;
            let channels = r.info().channels as usize;
            files.push(Open {
                r,
                src: s.clone(),
                channels,
                buf: Vec::new(),
            });
        }
        Ok(Self { files, pos: start })
    }

    /// Fill `usb[i][..n]` (USB input i+1) with the next `n` timeline frames (silence where no
    /// file plays).
    pub fn read(&mut self, usb: &mut [Vec<f32>], n: usize) -> Result<(), String> {
        for u in usb.iter_mut() {
            u[..n].fill(0.0);
        }
        let (t0, t1) = (self.pos, self.pos + n as u64);
        for f in &mut self.files {
            let s = &f.src;
            let avail = f.r.info().frames.saturating_sub(s.src_offset);
            let len = s.length.map_or(avail, |l| l.min(avail));
            let (a, b) = (s.position.max(t0), (s.position + len).min(t1));
            if a >= b {
                continue;
            }
            let file_frame = s.src_offset + (a - s.position);
            let frames = (b - a) as usize;
            let off = (a - t0) as usize;
            f.r.seek(file_frame)
                .map_err(|e| format!("{}: {e}", s.path.display()))?;
            f.buf.resize(frames * f.channels, 0.0);
            let got =
                f.r.read_interleaved(&mut f.buf)
                    .map_err(|e| format!("{}: {e}", s.path.display()))?;
            let ch = f.channels;
            for &(pick, u) in &s.lanes {
                let dst = &mut usb[u as usize - 1][off..off + got];
                match pick {
                    Pick::Channel(c) => {
                        for (d, fr) in dst.iter_mut().zip(f.buf.chunks_exact(ch)) {
                            *d += fr[c];
                        }
                    }
                    Pick::Mix => {
                        let k = 1.0 / ch as f32;
                        for (d, fr) in dst.iter_mut().zip(f.buf.chunks_exact(ch)) {
                            *d += fr.iter().sum::<f32>() * k;
                        }
                    }
                }
            }
        }
        self.pos = t1;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use wing_sim::wav::{write_pcm, WavWriter};

    fn dir(tag: &str) -> PathBuf {
        let d = std::env::temp_dir().join(format!("wingrender-in-{tag}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&d);
        std::fs::create_dir_all(&d).unwrap();
        d
    }

    fn mk(d: &Path, name: &str, ch: u16, frames: usize, v: f32) -> PathBuf {
        let p = d.join(name);
        let mut w = WavWriter::create(&p, ch, 48_000).unwrap();
        w.write_interleaved(&vec![v; frames * ch as usize]).unwrap();
        w.finish().unwrap();
        p
    }

    #[test]
    fn numbered_and_sequential_assignment() {
        let d = dir("num");
        mk(&d, "03-Kick.wav", 1, 10, 0.1);
        mk(&d, "01-Bass.wav", 1, 10, 0.2);
        mk(&d, "05-OH.wav", 2, 10, 0.3);
        let p = plan_from_files(std::slice::from_ref(&d)).unwrap();
        let lanes: Vec<_> = p.sources.iter().map(|s| s.lanes.clone()).collect();
        assert_eq!(
            lanes,
            [
                vec![(Pick::Channel(0), 1)],
                vec![(Pick::Channel(0), 3)],
                vec![(Pick::Channel(0), 5), (Pick::Channel(1), 6)]
            ]
        );
        // a name without a number: sequential (stereo takes two inputs)
        mk(&d, "Click.wav", 1, 10, 0.4);
        let p = plan_from_files(std::slice::from_ref(&d)).unwrap();
        let firsts: Vec<_> = p.sources.iter().map(|s| s.lanes[0].1).collect();
        assert_eq!(firsts, [1, 2, 3, 5]);
        assert_eq!(p.describe()[0], "USB  1 <- 01-Bass.wav ch1");
    }

    #[test]
    fn explicit_map_and_reader() {
        let d = dir("map");
        mk(&d, "kick.wav", 1, 100, 0.5);
        mk(&d, "keys.wav", 2, 50, 0.25);
        let p = plan_from_map(
            "usb:1=kick.wav, usb:3-4=keys.wav, usb:7=keys.wav:2, usb:8=keys.wav:mix, usb:9=keys.wav",
            Some(&d),
        )
        .unwrap();
        p.validate(48_000).unwrap();
        assert!(p.validate(44_100).is_err());
        assert_eq!(p.end_frame().unwrap(), 100);
        assert_eq!(p.sources[3].lanes, [(Pick::Mix, 8)]);
        assert_eq!(p.sources[4].lanes, [(Pick::Mix, 9)]);
        assert!(plan_from_map("usb:1-3=keys.wav", Some(&d)).is_err());
        assert!(plan_from_map("bus:1=keys.wav", Some(&d)).is_err());
        let mut usb = vec![vec![0.0f32; 64]; USB_INPUTS];
        let mut r = InputReader::open(&p, 20).unwrap();
        r.read(&mut usb, 64).unwrap();
        assert_eq!(usb[0][63], 0.5);
        assert_eq!(usb[2][29], 0.25); // keys ends at frame 50 = offset 30
        assert_eq!(usb[2][30], 0.0);
        assert_eq!(usb[6][0], 0.25);
        assert_eq!(usb[7][0], 0.25);
        assert_eq!(usb[1][0], 0.0);
    }

    #[test]
    fn positioned_source_and_pcm24() {
        let d = dir("pos");
        let p24 = d.join("t.wav");
        let s: Vec<f32> = (0..100).map(|i| i as f32 / 1024.0).collect();
        write_pcm(&p24, 1, 48_000, 24, &s).unwrap();
        let plan = InputPlan {
            sources: vec![Source {
                path: p24,
                position: 10,
                src_offset: 5,
                length: Some(20),
                lanes: vec![(Pick::Channel(0), 2)],
                label: "t".into(),
            }],
            notes: vec![],
        };
        assert_eq!(plan.end_frame().unwrap(), 30);
        let mut usb = vec![vec![0.0f32; 40]; USB_INPUTS];
        InputReader::open(&plan, 0)
            .unwrap()
            .read(&mut usb, 40)
            .unwrap();
        assert_eq!(usb[1][9], 0.0);
        assert_eq!(usb[1][10], 5.0 / 1024.0);
        assert_eq!(usb[1][29], 24.0 / 1024.0);
        assert_eq!(usb[1][30], 0.0);
    }
}
