//! Record/bounce console outputs to WAV while the daemon runs (T061), and the tap-group syntax
//! shared with `wing-render` (T060).
//!
//! ```text
//!  control (OSC /wingsim/record/start, CLI)            RT driver thread
//!  Recorder::start ─ Box<RecordTap> ─► RecordSlot.incoming ─► Console::cycle installs it,
//!                                                              resolves tap ports, then per
//!                                                              cycle push_with(block) ─┐
//!  writer thread (non-RT) ◄── wing_hostio::ring SPSC (≥ 4 s) ◄──────────────────────────┘
//!     drains every 20 ms → one WavWriter (32-bit float) per tap group
//!  Recorder::stop ─ RecordSlot.stop ─► RT moves the tap to RecordSlot.retired ─► control drops it
//! ```
//!
//! RT side: two atomic pointer loads and one flag load per cycle; installing/retiring a tap is a
//! pointer exchange (no allocation, free or lock: every `RecordTap` is allocated and dropped on
//! control threads). The ring push is wait-free; if the writer falls behind by more than the ring
//! (4 s) whole blocks are dropped and counted (`dropped` in the status).
//!
//! Tap groups ([`parse_tap_groups`]): a comma-separated list of `wing_hostio` taps; stereo taps
//! (`main/1`, `bus/3`, `mtx/2`, `mon/1`) are one stereo file each (`main/1.L`: mono), flat taps
//! expand to one mono file per lane (`usb_out/1-48` → `usb_out_01.wav` … `usb_out_48.wav`).
//!
//! Limits: a console rebuild (driver sample-rate change, reattach) ends the recording (the files
//! are finalized); WAV files are limited to 4 GiB (≈ 3.1 h stereo at 48 kHz).

use std::path::{Path, PathBuf};
use std::ptr;
use std::sync::atomic::{AtomicBool, AtomicPtr, AtomicU32, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use wing_engine::Port;
use wing_hostio::config::parse_tap;
use wing_hostio::ring::{ring, RingConsumer, RingProducer};
use wing_hostio::{Tap, TapChannel};
use wing_proto::osc::{Arg, SimExtension};

use crate::audio::ConsoleNodes;
use crate::wav::WavWriter;

// ---- tap groups -----------------------------------------------------------------------------------

/// One output file: a name and its lanes (1 = mono, 2 = stereo).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TapGroup {
    pub name: String,
    pub lanes: Vec<TapChannel>,
}

/// Parse `main/1,bus/3,mon/1,usb_out/1-48` into file groups (see the module docs).
pub fn parse_tap_groups(spec: &str) -> Result<Vec<TapGroup>, String> {
    let mut out: Vec<TapGroup> = Vec::new();
    for item in spec.split(',').map(str::trim).filter(|s| !s.is_empty()) {
        let lanes = parse_tap(item).map_err(|e| e.to_string())?;
        let Some(first) = lanes.first() else {
            continue;
        };
        if first.tap == Tap::LocalOut {
            return Err(format!(
                "tap '{item}': local outputs are not modelled (use usb_out/.., main/.., bus/..)"
            ));
        }
        if first.tap.is_stereo() {
            let base = first.tap.to_string().replace('/', "_");
            let name = match lanes.as_slice() {
                [l] if l.lane == wing_hostio::config::LEFT => format!("{base}_L"),
                [_] => format!("{base}_R"),
                _ => base,
            };
            out.push(TapGroup { name, lanes });
        } else {
            for l in lanes {
                out.push(TapGroup {
                    name: format!("{}_{:02}", l.tap, l.lane),
                    lanes: vec![l],
                });
            }
        }
    }
    if out.is_empty() {
        return Err("no taps given".into());
    }
    let mut seen = std::collections::HashSet::new();
    for g in &out {
        if !seen.insert(g.name.clone()) {
            return Err(format!("tap {} given twice", g.name));
        }
    }
    Ok(out)
}

// ---- RT side -------------------------------------------------------------------------------------

/// What the console pushes while recording (allocated on the control thread).
pub struct RecordTap {
    producer: RingProducer,
    lanes: Box<[TapChannel]>,
    ports: Box<[Option<Port>]>,
    /// Cleared when this tap is dropped (the writer then drains and finalizes).
    alive: Arc<AtomicBool>,
    /// Frames pushed (dropped blocks included).
    frames: Arc<AtomicU64>,
}

impl RecordTap {
    /// Resolve the lanes against the console's nodes (RT-safe: no allocation).
    #[inline]
    pub fn resolve(&mut self, nodes: &ConsoleNodes) {
        for (p, &l) in self.ports.iter_mut().zip(self.lanes.iter()) {
            *p = nodes.tap_port(l);
        }
    }

    /// Push one block; `port(p)` reads an engine port, `silence` is a zero block. RT-safe.
    #[inline]
    pub fn push<'a>(&mut self, n: usize, silence: &'a [f32], port: impl Fn(Port) -> &'a [f32]) {
        let ports = &self.ports;
        self.producer.push_with(n, 0, |i| match ports[i] {
            Some(p) => port(p),
            None => silence,
        });
        self.frames.fetch_add(n as u64, Ordering::Relaxed);
    }
}

impl Drop for RecordTap {
    fn drop(&mut self) {
        self.alive.store(false, Ordering::Release);
    }
}

/// Lock-free hand-over of a [`RecordTap`] between the control side and the RT thread.
#[derive(Default)]
pub struct RecordSlot {
    incoming: AtomicPtr<RecordTap>,
    retired: AtomicPtr<RecordTap>,
    stop: AtomicBool,
    /// Rate of the console that polls this slot (0: none yet).
    sample_rate: AtomicU32,
}

impl RecordSlot {
    pub fn new() -> Arc<Self> {
        Arc::new(Self::default())
    }

    /// The console's rate (set when a console attaches).
    pub fn set_sample_rate(&self, r: u32) {
        self.sample_rate.store(r, Ordering::Relaxed);
    }

    pub fn sample_rate(&self) -> u32 {
        self.sample_rate.load(Ordering::Relaxed)
    }

    /// RT: take a pending stop/install. Returns whether a new tap was installed in `cur` (the
    /// caller then resolves its ports). RT-safe: atomics only; a retired tap is handed back,
    /// never dropped here.
    #[inline]
    pub fn poll(&self, cur: &mut Option<Box<RecordTap>>) -> bool {
        let retired_free = || self.retired.load(Ordering::Acquire).is_null();
        if self.stop.load(Ordering::Acquire) {
            if !retired_free() {
                return false; // the control side has not collected the previous one yet
            }
            if let Some(t) = cur.take() {
                self.retired.store(Box::into_raw(t), Ordering::Release);
            }
            self.stop.store(false, Ordering::Release);
            return false;
        }
        if self.incoming.load(Ordering::Acquire).is_null() {
            return false;
        }
        if cur.is_some() {
            // A tap whose stop was never served (no RT thread at the time): hand it back first.
            if !retired_free() {
                return false;
            }
            if let Some(t) = cur.take() {
                self.retired.store(Box::into_raw(t), Ordering::Release);
            }
        }
        let p = self.incoming.swap(ptr::null_mut(), Ordering::AcqRel);
        if p.is_null() {
            return false;
        }
        // SAFETY: `p` came from `Box::into_raw` in `Recorder::start`; the swap transfers
        // ownership exactly once.
        *cur = Some(unsafe { Box::from_raw(p) });
        true
    }

    fn take(p: &AtomicPtr<RecordTap>) -> Option<Box<RecordTap>> {
        let p = p.swap(ptr::null_mut(), Ordering::AcqRel);
        // SAFETY: as in `poll`.
        (!p.is_null()).then(|| unsafe { Box::from_raw(p) })
    }
}

impl Drop for RecordSlot {
    fn drop(&mut self) {
        drop(Self::take(&self.incoming));
        drop(Self::take(&self.retired));
    }
}

// ---- writer --------------------------------------------------------------------------------------

struct Progress {
    frames: AtomicU64,
    /// Frames the RT side dropped because the ring was full.
    dropped: AtomicU64,
    error: Mutex<Option<String>>,
}

fn writer_loop(
    mut cons: RingConsumer,
    mut files: Vec<(WavWriter, Vec<usize>)>,
    stop: Arc<AtomicBool>,
    alive: Arc<AtomicBool>,
    progress: Arc<Progress>,
) -> Result<u64, String> {
    let mut frame = vec![0.0f32; cons.channels()];
    let mut bufs: Vec<Vec<f32>> = files.iter().map(|_| Vec::with_capacity(1 << 16)).collect();
    let mut last_hdr = Instant::now();
    loop {
        // read `stop`/`alive` before draining so nothing pushed before they flipped is lost
        let done = stop.load(Ordering::Acquire) || !alive.load(Ordering::Acquire);
        loop {
            cons.begin();
            let mut n = 0u64;
            while n < 1 << 15 && cons.frame(&mut frame) {
                for ((_, lanes), b) in files.iter().zip(bufs.iter_mut()) {
                    b.extend(lanes.iter().map(|&i| frame[i]));
                }
                n += 1;
            }
            cons.commit();
            for ((w, _), b) in files.iter_mut().zip(bufs.iter_mut()) {
                w.write_interleaved(b).map_err(|e| e.to_string())?;
                b.clear();
            }
            progress.frames.fetch_add(n, Ordering::Relaxed);
            progress
                .dropped
                .store(cons.overflow_frames(), Ordering::Relaxed);
            if n < 1 << 15 {
                break;
            }
        }
        if last_hdr.elapsed() > Duration::from_secs(2) {
            // keep the files readable while recording
            last_hdr = Instant::now();
            for (w, _) in &mut files {
                w.flush_header().map_err(|e| e.to_string())?;
            }
        }
        if done {
            break;
        }
        std::thread::sleep(Duration::from_millis(20));
    }
    let mut frames = 0;
    for (w, _) in files {
        frames = w.finish().map_err(|e| e.to_string())?;
    }
    Ok(frames)
}

// ---- control side --------------------------------------------------------------------------------

/// Ring capacity in seconds of audio.
pub const RING_SECONDS: usize = 4;
/// How long [`Recorder::stop`] waits for the RT thread to hand the tap back.
pub const STOP_WAIT: Duration = Duration::from_millis(500);

struct Active {
    dir: PathBuf,
    files: usize,
    rate: u32,
    tap_frames: Arc<AtomicU64>,
    alive: Arc<AtomicBool>,
    stop: Arc<AtomicBool>,
    progress: Arc<Progress>,
    writer: JoinHandle<Result<u64, String>>,
}

/// Recording controller (control threads: OSC, CLI). Cheap to share (`Arc`).
pub struct Recorder {
    slot: Arc<RecordSlot>,
    state: Mutex<(Option<Active>, String)>,
}

impl Default for Recorder {
    fn default() -> Self {
        Self {
            slot: RecordSlot::new(),
            state: Mutex::new((None, "rec off".into())),
        }
    }
}

impl Recorder {
    pub fn new() -> Arc<Self> {
        Arc::new(Self::default())
    }

    /// The slot consoles poll (see `Console::attach_recorder`).
    pub fn slot(&self) -> &Arc<RecordSlot> {
        &self.slot
    }

    fn lock(&self) -> std::sync::MutexGuard<'_, (Option<Active>, String)> {
        self.state.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Start recording `taps` (see [`parse_tap_groups`]) into `dir` (created), stopping a
    /// running recording first. Files are `<dir>/<group>.wav`, 32-bit float at the console's
    /// rate (48 kHz while no console is attached). Returns a description.
    pub fn start(&self, taps: &str, dir: &Path) -> Result<String, String> {
        let groups = parse_tap_groups(taps)?;
        let mut st = self.lock();
        if let Some(a) = st.0.take() {
            st.1 = self.finish(a);
        }
        std::fs::create_dir_all(dir).map_err(|e| format!("{}: {e}", dir.display()))?;
        let rate = match self.slot.sample_rate() {
            0 => 48_000,
            r => r,
        };
        let mut lanes = Vec::new();
        let mut files = Vec::new();
        for g in &groups {
            let path = dir.join(format!("{}.wav", g.name));
            let w = WavWriter::create(&path, g.lanes.len() as u16, rate)
                .map_err(|e| format!("{}: {e}", path.display()))?;
            files.push((w, (lanes.len()..lanes.len() + g.lanes.len()).collect()));
            lanes.extend_from_slice(&g.lanes);
        }
        let (producer, consumer) = ring(lanes.len(), rate as usize * RING_SECONDS);
        let alive = Arc::new(AtomicBool::new(true));
        let tap_frames = Arc::new(AtomicU64::new(0));
        let n = lanes.len();
        let tap = Box::new(RecordTap {
            producer,
            lanes: lanes.into(),
            ports: vec![None; n].into(),
            alive: alive.clone(),
            frames: tap_frames.clone(),
        });
        let stop = Arc::new(AtomicBool::new(false));
        let progress = Arc::new(Progress {
            frames: AtomicU64::new(0),
            dropped: AtomicU64::new(0),
            error: Mutex::new(None),
        });
        let (s2, a2, p2) = (stop.clone(), alive.clone(), progress.clone());
        let nfiles = files.len();
        let writer = std::thread::Builder::new()
            .name("wing-rec-writer".into())
            .spawn(move || {
                let r = writer_loop(consumer, files, s2, a2, p2.clone());
                if let Err(e) = &r {
                    *p2.error.lock().unwrap_or_else(|e| e.into_inner()) = Some(e.clone());
                }
                r
            })
            .map_err(|e| format!("writer thread: {e}"))?;
        // A stale stop request (no RT thread consumed it) must not retire the new tap.
        self.slot.stop.store(false, Ordering::Release);
        drop(RecordSlot::take(&self.slot.retired));
        self.slot
            .incoming
            .store(Box::into_raw(tap), Ordering::Release);
        let msg = format!(
            "recording {} file(s) ({}) to {} at {rate} Hz",
            nfiles,
            groups
                .iter()
                .map(|g| g.name.as_str())
                .collect::<Vec<_>>()
                .join(" "),
            dir.display()
        );
        st.0 = Some(Active {
            dir: dir.to_path_buf(),
            files: nfiles,
            rate,
            tap_frames,
            alive,
            stop,
            progress,
            writer,
        });
        st.1 = msg.clone();
        Ok(msg)
    }

    /// Stop recording and finalize the files. Returns a summary.
    pub fn stop(&self) -> String {
        let mut st = self.lock();
        match st.0.take() {
            Some(a) => {
                st.1 = self.finish(a);
                st.1.clone()
            }
            None => "not recording".into(),
        }
    }

    fn finish(&self, a: Active) -> String {
        // Not installed yet (no RT thread): take it back ourselves.
        let mut tap = RecordSlot::take(&self.slot.incoming);
        if tap.is_none() {
            self.slot.stop.store(true, Ordering::Release);
            let t0 = Instant::now();
            while self.slot.stop.load(Ordering::Acquire) && t0.elapsed() < STOP_WAIT {
                std::thread::sleep(Duration::from_millis(2));
            }
            tap = RecordSlot::take(&self.slot.retired);
            self.slot.stop.store(false, Ordering::Release);
        }
        // Dropping the tap (here, on the control thread) clears `alive`; if the RT thread did
        // not answer (session ended meanwhile) the stop flag ends the writer anyway.
        drop(tap);
        a.stop.store(true, Ordering::Release);
        let res = a
            .writer
            .join()
            .unwrap_or_else(|_| Err("writer panicked".into()));
        let dropped = a.progress.dropped.load(Ordering::Relaxed);
        match res {
            Ok(frames) => format!(
                "recorded {:.1} s, {} file(s) to {}{}",
                frames as f64 / a.rate as f64,
                a.files,
                a.dir.display(),
                if dropped > 0 {
                    format!(", {dropped} frames DROPPED (writer too slow)")
                } else {
                    String::new()
                }
            ),
            Err(e) => format!("recording to {} failed: {e}", a.dir.display()),
        }
    }

    /// Whether a recording is running.
    pub fn is_recording(&self) -> bool {
        self.lock().0.is_some()
    }

    /// One-line status for the audio status line.
    pub fn status(&self) -> String {
        let st = self.lock();
        let Some(a) = &st.0 else {
            return if st.1.starts_with("recorded") || st.1.contains("failed") {
                format!("rec off (last: {})", st.1)
            } else {
                "rec off".into()
            };
        };
        if let Some(e) = a.progress.error.lock().ok().and_then(|e| e.clone()) {
            return format!("rec ERROR {e}");
        }
        if !a.alive.load(Ordering::Relaxed) {
            return format!(
                "rec ended (console rebuilt), stop to finalize {}",
                a.dir.display()
            );
        }
        let pushed = a.tap_frames.load(Ordering::Relaxed);
        if pushed == 0 {
            return format!("rec armed (waiting for audio) -> {}", a.dir.display());
        }
        let written = a.progress.frames.load(Ordering::Relaxed);
        format!(
            "rec {:.1} s, {} file(s) -> {}, dropped {}",
            written as f64 / a.rate as f64,
            a.files,
            a.dir.display(),
            a.progress.dropped.load(Ordering::Relaxed)
        )
    }
}

impl Drop for Recorder {
    fn drop(&mut self) {
        let a = self.lock().0.take();
        if let Some(a) = a {
            self.finish(a);
        }
    }
}

/// OSC: `/wingsim/record/start ,ss taps dir`, `/wingsim/record/stop`, `/wingsim/record/status`
/// (and `/wingsim/record` = status). Every command replies `/wingsim/record ,s <status>`;
/// errors reply `/wingsim/record ,s "ERROR ..."`.
impl SimExtension for Recorder {
    fn command(&self, addr: &str, args: &[Arg]) -> Option<Option<(String, Vec<Arg>)>> {
        let text = |i: usize| match args.get(i) {
            Some(Arg::Str(s)) if !s.trim().is_empty() => Some(s.trim().to_string()),
            _ => None,
        };
        let msg = match addr {
            "/wingsim/record/start" => match (text(0), text(1)) {
                (Some(taps), Some(dir)) => self
                    .start(&taps, Path::new(&dir))
                    .unwrap_or_else(|e| format!("ERROR {e}")),
                _ => "ERROR usage: /wingsim/record/start ,ss <taps> <dir>".into(),
            },
            "/wingsim/record/stop" => self.stop(),
            "/wingsim/record" | "/wingsim/record/status" => self.status(),
            _ => return None,
        };
        Some(Some(("/wingsim/record".into(), vec![Arg::Str(msg)])))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tap_groups() {
        let g = parse_tap_groups("main/1, bus/3,mon/1, usb_out/1-3,mtx/2.R").unwrap();
        let names: Vec<_> = g.iter().map(|g| g.name.as_str()).collect();
        assert_eq!(
            names,
            [
                "main_1",
                "bus_3",
                "mon_1",
                "usb_out_01",
                "usb_out_02",
                "usb_out_03",
                "mtx_2_R"
            ]
        );
        assert_eq!(g[0].lanes.len(), 2);
        assert_eq!(g[3].lanes.len(), 1);
        assert!(parse_tap_groups("main/9").is_err());
        assert!(parse_tap_groups("local_out/1").is_err());
        assert!(parse_tap_groups("main/1,main/1").is_err());
        assert!(parse_tap_groups("").is_err());
    }
}
