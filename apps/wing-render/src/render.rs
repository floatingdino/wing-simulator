//! The offline render: the daemon's [`Console`] driven by a free-running loop over file input.
//!
//! Same code path as `wing-sim`: scene loaded into a `ParamStore`, `wing_sim::audio::prepare`,
//! routing refresh, [`Console::build`], the session warm-up
//! ([`SESSION_WARM_UP_CYCLES`] silent cycles), then [`Console::cycle`] per engine block — exactly
//! what the daemon's RT thread runs per driver block — with USB In 1..48 from the input plan
//! instead of the driver ring. Nothing waits for a clock (the "offline clock": cycles run back to
//! back), so a render is faster than realtime and bit-identical to what the daemon outputs for
//! the same input from session start (`tests/determinism.rs`).
//!
//! I/O is done in chunks of [`CHUNK_BLOCKS`] engine blocks around the cycles.

use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::Instant;

use wing_engine::Port;
use wing_model::{ParamStore, Snapshot};
use wing_sim::audio::console::{Console, ConsoleConfig};
use wing_sim::audio::driver::SESSION_WARM_UP_CYCLES;
use wing_sim::audio::prepare;
use wing_sim::audio::stats::AudioStats;
use wing_sim::record::TapGroup;
use wing_sim::wav::WavWriter;

use crate::inputs::{InputPlan, InputReader, USB_INPUTS};
use crate::loudness::LoudnessMeter;

/// Engine blocks per I/O chunk.
pub const CHUNK_BLOCKS: usize = 64;

/// Render settings.
#[derive(Debug, Clone)]
pub struct RenderConfig {
    pub rate: u32,
    /// Engine block (the daemon's default is 64; bit-identity needs the same block).
    pub block: usize,
    /// Engine worker threads besides the render thread.
    pub threads: usize,
    /// Timeline range in frames: `[start, end)`; `end = None`: the inputs' end plus `tail`.
    pub start: u64,
    pub end: Option<u64>,
    pub tail: u64,
    pub taps: Vec<TapGroup>,
    pub out_dir: PathBuf,
}

/// Per-tap result.
#[derive(Debug, Clone)]
pub struct TapResult {
    pub name: String,
    pub file: PathBuf,
    pub channels: usize,
    pub peak_dbfs: f64,
    pub lufs_integrated: Option<f64>,
}

/// What a render did.
#[derive(Debug, Clone)]
pub struct RenderReport {
    pub frames: u64,
    pub seconds: f64,
    /// Wall time of the processing loop (I/O included, console build excluded).
    pub render_seconds: f64,
    pub build_seconds: f64,
    pub speed_x_realtime: f64,
    pub taps: Vec<TapResult>,
}

/// A store with `path` loaded (`None`: factory defaults). Returns a load summary.
pub fn load_scene(path: Option<&Path>) -> Result<(ParamStore, String), String> {
    let store = ParamStore::new();
    let Some(p) = path else {
        return Ok((store, "factory defaults".into()));
    };
    let text = std::fs::read_to_string(p).map_err(|e| format!("{}: {e}", p.display()))?;
    let (_, rep) = Snapshot::load(&store, &text).map_err(|e| format!("{}: {e}", p.display()))?;
    Ok((
        store,
        format!(
            "{}: {} (fw {}), {} keys, {} applied, {} unmapped, {} rejected",
            p.display(),
            rep.format,
            rep.creator_fw,
            rep.keys,
            rep.applied,
            rep.unmapped.len(),
            rep.rejected.len()
        ),
    ))
}

/// Build the console exactly as the daemon's audio service does for a session (store already
/// holding the scene). Returns the console, warmed up.
pub fn build_console(
    store: ParamStore,
    rate: u32,
    block: usize,
    threads: usize,
) -> Result<(Console, ParamStore), String> {
    build_console_with(
        store,
        ConsoleConfig {
            sample_rate: rate,
            block,
            workers: threads,
            rt_workers: false,
            workgroup: None,
            meters: None,
            rta: None,
            probe: None,
        },
    )
}

/// [`build_console`] with a full [`ConsoleConfig`] (e.g. with the T088 slot probe attached).
pub fn build_console_with(
    mut store: ParamStore,
    cfg: ConsoleConfig,
) -> Result<(Console, ParamStore), String> {
    let mut ctl = prepare(&mut store);
    ctl.refresh(&store);
    let mut console = Console::build(
        cfg,
        &store,
        &mut ctl,
        None,
        Vec::new(),
        Arc::new(AudioStats::default()),
    )?;
    console.warm_up(SESSION_WARM_UP_CYCLES);
    Ok((console, store))
}

struct Out {
    ports: Vec<Option<Port>>,
    writer: WavWriter,
    meter: LoudnessMeter,
    buf: Vec<f32>,
    name: String,
    file: PathBuf,
}

/// Render `plan` through the scene in `store`. `progress(fraction)` is called per chunk.
pub fn render(
    store: ParamStore,
    plan: &InputPlan,
    cfg: &RenderConfig,
    mut progress: impl FnMut(f64),
) -> Result<RenderReport, String> {
    plan.validate(cfg.rate)?;
    if !(16..=1024).contains(&cfg.block) {
        return Err("block must be 16..1024".into());
    }
    let end = match cfg.end {
        Some(e) => e,
        None => plan.end_frame()? + cfg.tail,
    };
    if end <= cfg.start {
        return Err(format!(
            "empty range: start {} >= end {end} frames",
            cfg.start
        ));
    }
    let total = end - cfg.start;
    std::fs::create_dir_all(&cfg.out_dir).map_err(|e| format!("{}: {e}", cfg.out_dir.display()))?;

    let t_build = Instant::now();
    let (mut console, _store) = build_console(store, cfg.rate, cfg.block, cfg.threads)?;
    let build_seconds = t_build.elapsed().as_secs_f64();

    let mut outs = Vec::new();
    for g in &cfg.taps {
        let file = cfg.out_dir.join(format!("{}.wav", g.name));
        outs.push(Out {
            ports: g
                .lanes
                .iter()
                .map(|&l| console.nodes().tap_port(l))
                .collect(),
            writer: WavWriter::create(&file, g.lanes.len() as u16, cfg.rate)
                .map_err(|e| format!("{}: {e}", file.display()))?,
            meter: LoudnessMeter::new(g.lanes.len(), cfg.rate),
            buf: Vec::with_capacity(cfg.block * CHUNK_BLOCKS * g.lanes.len()),
            name: g.name.clone(),
            file,
        });
    }

    let block = cfg.block;
    let chunk = block * CHUNK_BLOCKS;
    let mut reader = InputReader::open(plan, cfg.start)?;
    let mut usb = vec![vec![0.0f32; chunk]; USB_INPUTS];
    let silence = vec![0.0f32; block];
    let t0 = Instant::now();
    let mut done = 0u64;
    while done < total {
        let n = ((total - done) as usize).min(chunk);
        reader.read(&mut usb, n)?;
        // whole engine blocks; the last one is zero-padded and trimmed on output
        let cycles = n.div_ceil(block);
        for u in usb.iter_mut() {
            u[n..cycles * block].fill(0.0);
        }
        for o in &mut outs {
            o.buf.clear();
            o.buf.resize(n * o.ports.len(), 0.0);
        }
        for c in 0..cycles {
            let off = c * block;
            console.cycle(
                |ch, buf| buf.copy_from_slice(&usb[ch][off..off + block]),
                |_, _| {},
            );
            let keep = block.min(n - off);
            for o in &mut outs {
                let w = o.ports.len();
                for (k, p) in o.ports.iter().enumerate() {
                    let s = p.map_or(&silence[..], |p| console.port(p));
                    for (i, &v) in s[..keep].iter().enumerate() {
                        o.buf[(off + i) * w + k] = v;
                    }
                }
            }
        }
        for o in &mut outs {
            o.writer
                .write_interleaved(&o.buf)
                .map_err(|e| format!("{}: {e}", o.file.display()))?;
            o.meter.push(&o.buf);
        }
        done += n as u64;
        progress(done as f64 / total as f64);
    }
    let mut taps = Vec::new();
    for o in outs {
        let channels = o.writer.channels() as usize;
        o.writer
            .finish()
            .map_err(|e| format!("{}: {e}", o.file.display()))?;
        taps.push(TapResult {
            name: o.name,
            file: o.file,
            channels,
            peak_dbfs: o.meter.peak_dbfs(),
            lufs_integrated: o.meter.integrated_lufs(),
        });
    }
    let render_seconds = t0.elapsed().as_secs_f64();
    let seconds = total as f64 / cfg.rate as f64;
    Ok(RenderReport {
        frames: total,
        seconds,
        render_seconds,
        build_seconds,
        speed_x_realtime: seconds / render_seconds.max(1e-9),
        taps,
    })
}
