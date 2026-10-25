//! Performance suite (T070): worst-case scenes driven through the daemon's [`Console`] with the
//! real scheduler, per-cycle timing statistics, worker scaling, a per-node serial profile and a
//! per-model cost survey. Used by `wing-render --bench` and the criterion bench `benches/perf.rs`.
//!
//! Scenes ([`Scene`]):
//! * `public`: `fixtures/PUBLIC.snap` as is.
//! * `everything`: factory defaults with every processing block on and the heaviest models
//!   ([`everything_on`]): 40 ch + 8 aux on USB inputs with filter tool, gate, EQ, dynamics and both
//!   inserts; every send to all 16 busses, 4 mains and 8 matrices on; all 16 bus / 4 main / 8 matrix
//!   strips with EQ, dynamics, delay and inserts on; all 16 FX slots loaded with the heaviest model
//!   their group allows, each owned by an insert.
//!
//! Timing regimes:
//! * **hot**: cycles back to back, no RT policy (worker scaling; the kernel throttles an RT thread that
//!   runs flat out past its time-constraint budget, which would distort the numbers);
//! * **paced**: one cycle per period (48 kHz), RT time-constraint policy on the driver and the workers,
//!   workers sleep between cycles: wake-up latency included, the production regime (p99 / p99.9).
//!
//! Inputs are deterministic noise at about -18 dBFS RMS on all 48 USB inputs, so every detector works.

use std::sync::Arc;
use std::time::{Duration, Instant};

use wing_blocks::eq::EqVariant;
use wing_blocks::model::{catalog, create_model, BlockKind};
use wing_engine::{rt, TimeConstraint};
use wing_model::ParamStore;
use wing_sim::audio::console::{Console, ConsoleConfig};
use wing_sim::audio::driver::SESSION_WARM_UP_CYCLES;
use wing_sim::audio::prepare;
use wing_sim::audio::stats::AudioStats;

use crate::render::load_scene;

// The heaviest models per slot family, from the model survey (`wing-render --bench --models`,
// docs/perf.md). Gate and dynamics slots share one model table.

/// Gate-slot model of every channel in the "everything on" scene.
pub const HEAVY_GATE: &str = "NSTR";
/// Dynamics-slot model of every input / bus / main / matrix strip.
pub const HEAVY_DYN: &str = "76LA";
/// Dynamics model of the aux strips.
pub const HEAVY_AUX_DYN: &str = "76LA";
/// Channel EQ model.
pub const HEAVY_EQ: &str = "MACH4";
/// Aux EQ model (aux has no MACH4).
pub const HEAVY_AUX_EQ: &str = "PULSAR";
/// Bus / main / matrix EQ model.
pub const HEAVY_MIX_EQ: &str = "PIA";
/// Filter-tool model.
pub const HEAVY_FLT: &str = "AP2";
/// Models of FX slots 1..16 (1..8 full: any model; 9..16 lite: no reverb/mod/delay/pitch). `*MASTER*`
/// runs with all four stages on (TAPE + MACH4 + ENHANCE + LIMITER, the heaviest FX model); the
/// rest are the heaviest single models of the survey, plus the heaviest reverbs.
pub const HEAVY_FX: [&str; 16] = [
    "*MASTER*", "ANGEL", "EXCITER", "UKROCK", "SPRING", "DELUXE", "RACKAMP", "VSS3", "*MASTER*",
    "ENHANCE", "ANGEL", "EXCITER", "UKROCK", "DELUXE", "RACKAMP", "JAZZC",
];

/// A benchmark scene.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Scene {
    /// `fixtures/PUBLIC.snap` as is.
    Public,
    /// Every block on, heaviest models, every FX slot loaded ([`everything_on`]).
    Everything,
}

impl Scene {
    pub fn name(self) -> &'static str {
        match self {
            Scene::Public => "public",
            Scene::Everything => "everything",
        }
    }

    pub fn parse(s: &str) -> Option<Self> {
        match s {
            "public" | "PUBLIC" => Some(Scene::Public),
            "everything" | "all" => Some(Scene::Everything),
            _ => None,
        }
    }

    /// A fresh store with this scene loaded (plus the `--set` overrides of [`OVERRIDES`]).
    pub fn store(self) -> Result<ParamStore, String> {
        let s = match self {
            Scene::Public => {
                let p = public_snap();
                load_scene(Some(&p)).map(|(s, _)| s)?
            }
            Scene::Everything => {
                let (s, _) = load_scene(None)?;
                everything_on(&s)?;
                s
            }
        };
        for (p, v) in OVERRIDES.get().map_or(&[][..], |v| &v[..]) {
            set(&s, p, v)?;
        }
        Ok(s)
    }
}

/// Parameter overrides applied to every bench scene (`wing-render --bench --set path=value`;
/// what-if analyses such as "this scene without FX 5").
pub static OVERRIDES: std::sync::OnceLock<Vec<(String, String)>> = std::sync::OnceLock::new();

/// Path of `fixtures/PUBLIC.snap` (repo root relative to this crate).
pub fn public_snap() -> std::path::PathBuf {
    std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../fixtures/PUBLIC.snap")
}

fn set(store: &ParamStore, p: &str, v: &str) -> Result<(), String> {
    let id = store.lookup(p).ok_or_else(|| format!("no parameter {p}"))?;
    store
        .set_text(id, v, 0)
        .map(|_| ())
        .map_err(|e| format!("{p}={v}: {e:?}"))
}

/// Turn every processing block of `store` on with the heaviest models (see the module doc).
pub fn everything_on(store: &ParamStore) -> Result<(), String> {
    // FX slots: heaviest model each; owners: ch 1..16 pre-inserts.
    for (k, m) in HEAVY_FX.iter().enumerate() {
        let n = k + 1;
        set(store, &format!("fx/{n}/mdl"), m)?;
        set(store, &format!("fx/{n}/fxmix"), "100")?;
        if *m == "*MASTER*" {
            // every stage in: TAPE -> MACH4 -> ENHANCE -> LIMITER (the heaviest FX model)
            for s in ["t_on", "eq_on", "e_on", "l_on"] {
                set(store, &format!("fx/{n}/{s}"), "1")?;
            }
        }
    }
    for i in 0..48 {
        let (p, usb) = if i < 40 {
            (format!("ch/{}", i + 1), i + 1)
        } else {
            (format!("aux/{}", i - 39), i + 1)
        };
        set(store, &format!("{p}/in/conn/grp"), "USB")?;
        set(store, &format!("{p}/in/conn/in"), &usb.to_string())?;
        let aux = i >= 40;
        if !aux {
            set(store, &format!("{p}/flt/lc"), "1")?;
            set(store, &format!("{p}/flt/hc"), "1")?;
            set(store, &format!("{p}/flt/tf"), "1")?;
            set(store, &format!("{p}/flt/mdl"), HEAVY_FLT)?;
            set(store, &format!("{p}/gate/on"), "1")?;
            set(store, &format!("{p}/gate/mdl"), HEAVY_GATE)?;
            set(store, &format!("{p}/peq/on"), "1")?;
        }
        set(store, &format!("{p}/eq/on"), "1")?;
        set(
            store,
            &format!("{p}/eq/mdl"),
            if aux { HEAVY_AUX_EQ } else { HEAVY_EQ },
        )?;
        set(store, &format!("{p}/dyn/on"), "1")?;
        set(
            store,
            &format!("{p}/dyn/mdl"),
            if aux { HEAVY_AUX_DYN } else { HEAVY_DYN },
        )?;
        set(store, &format!("{p}/mute"), "0")?;
        set(store, &format!("{p}/fdr"), "-10")?;
        for m in 1..=4 {
            set(store, &format!("{p}/main/{m}/on"), "1")?;
        }
        for k in 1..=16 {
            set(store, &format!("{p}/send/{k}/on"), "1")?;
            set(store, &format!("{p}/send/{k}/lvl"), "-20")?;
        }
        for k in 1..=8 {
            set(store, &format!("{p}/send/MX{k}/on"), "1")?;
            set(store, &format!("{p}/send/MX{k}/lvl"), "-20")?;
        }
    }
    // FX 1..16 inserted on ch 1..16 (pre); the inserts of the other strips stay empty: a slot
    // serves one insert, so 16 owners is the maximum FX load.
    for n in 1..=16 {
        set(store, &format!("ch/{n}/preins/ins"), &format!("FX{n}"))?;
        set(store, &format!("ch/{n}/preins/on"), "1")?;
    }
    let mix = (1..=16)
        .map(|n| format!("bus/{n}"))
        .chain((1..=4).map(|n| format!("main/{n}")))
        .chain((1..=8).map(|n| format!("mtx/{n}")));
    for p in mix {
        set(store, &format!("{p}/eq/on"), "1")?;
        set(store, &format!("{p}/eq/mdl"), HEAVY_MIX_EQ)?;
        set(store, &format!("{p}/dyn/on"), "1")?;
        set(store, &format!("{p}/dyn/mdl"), HEAVY_DYN)?;
        set(store, &format!("{p}/dly/on"), "1")?;
        set(store, &format!("{p}/mute"), "0")?;
        set(store, &format!("{p}/fdr"), "-10")?;
    }
    for b in 1..=16 {
        for m in 1..=4 {
            set(store, &format!("bus/{b}/main/{m}/on"), "1")?;
        }
        for k in 1..=8 {
            set(store, &format!("bus/{b}/send/MX{k}/on"), "1")?;
        }
    }
    for m in 1..=4 {
        for k in 1..=8 {
            set(store, &format!("main/{m}/send/MX{k}/on"), "1")?;
        }
    }
    Ok(())
}

/// Longest-processing-time-first claim order in consoles built by [`console`] (the engine's
/// default; `--no-lpt` turns it off for comparisons).
pub static LPT: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(true);

/// Build the console exactly as the daemon does (meters on), warmed up.
pub fn console(
    store: &mut ParamStore,
    block: usize,
    workers: usize,
    rt_workers: bool,
) -> Result<Console, String> {
    let mut ctl = prepare(store);
    ctl.refresh(store);
    let mut c = Console::build(
        ConsoleConfig {
            sample_rate: 48_000,
            block,
            workers,
            rt_workers,
            workgroup: None,
            meters: Some(wing_engine::meters::LevelBank::shared()),
            rta: None,
        },
        store,
        &mut ctl,
        None,
        Vec::new(),
        Arc::new(AudioStats::default()),
    )?;
    c.engine_mut()
        .set_load_balancing(LPT.load(std::sync::atomic::Ordering::Relaxed));
    c.warm_up(SESSION_WARM_UP_CYCLES);
    Ok(c)
}

/// Deterministic noise source for the 48 USB inputs (~-18 dBFS RMS, uniform).
pub struct Noise(u32);

impl Default for Noise {
    fn default() -> Self {
        Noise(0x1234_5678)
    }
}

impl Noise {
    #[inline]
    pub fn fill(&mut self, buf: &mut [f32]) {
        for s in buf {
            self.0 = self.0.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            *s = ((self.0 >> 8) as f32 / (1u32 << 24) as f32 - 0.5) * 0.43;
        }
    }
}

/// One console cycle on noise input; returns its wall time.
#[inline]
pub fn timed_cycle(c: &mut Console, noise: &mut Noise) -> Duration {
    let t = Instant::now();
    c.cycle(|_, buf| noise.fill(buf), |_, _| {});
    t.elapsed()
}

/// Statistics of a run (µs).
#[derive(Debug, Clone, Copy)]
pub struct CycleStats {
    pub cycles: usize,
    pub mean: f64,
    pub p50: f64,
    pub p99: f64,
    pub p999: f64,
    pub max: f64,
    /// Period at 48 kHz (µs).
    pub period: f64,
}

impl CycleStats {
    pub fn from_ns(mut v: Vec<u64>, block: usize) -> Self {
        v.sort_unstable();
        let n = v.len().max(1);
        let pct = |p: f64| v[((v.len() - 1) as f64 * p).round() as usize] as f64 / 1e3;
        Self {
            cycles: v.len(),
            mean: v.iter().sum::<u64>() as f64 / n as f64 / 1e3,
            p50: pct(0.5),
            p99: pct(0.99),
            p999: pct(0.999),
            max: *v.last().unwrap_or(&0) as f64 / 1e3,
            period: block as f64 * 1e6 / 48_000.0,
        }
    }

    pub fn pct(&self, us: f64) -> f64 {
        us / self.period * 100.0
    }
}

/// `cycles` hot (back to back) cycles after `warm` untimed ones.
pub fn run_hot(c: &mut Console, warm: usize, cycles: usize) -> CycleStats {
    let mut noise = Noise::default();
    for _ in 0..warm {
        timed_cycle(c, &mut noise);
    }
    let v = (0..cycles)
        .map(|_| timed_cycle(c, &mut noise).as_nanos() as u64)
        .collect();
    CycleStats::from_ns(v, c.block())
}

/// `cycles` paced cycles (one per period) on an RT driver thread; the console must have been built
/// with RT workers. Runs on a spawned thread, so `scene` builds the console there.
pub fn run_paced(
    scene: Scene,
    block: usize,
    workers: usize,
    warm: usize,
    cycles: usize,
) -> Result<CycleStats, String> {
    std::thread::spawn(move || {
        let period = Duration::from_nanos((block as u64 * 1_000_000_000) / 48_000);
        let _ = rt::set_current_thread_time_constraint(TimeConstraint::for_period(
            period.as_nanos() as u64,
        ));
        let mut store = scene.store()?;
        let mut c = console(&mut store, block, workers, true)?;
        let mut noise = Noise::default();
        let mut next = Instant::now();
        let mut v = Vec::with_capacity(cycles);
        for i in 0..warm + cycles {
            next += period;
            let d = timed_cycle(&mut c, &mut noise);
            if i >= warm {
                v.push(d.as_nanos() as u64);
            }
            let now = Instant::now();
            if now < next {
                // sleep most of the remaining period, spin the rest (keeps the period exact)
                let rem = next - now;
                if rem > Duration::from_micros(300) {
                    std::thread::sleep(rem - Duration::from_micros(250));
                }
                while Instant::now() < next {
                    std::hint::spin_loop();
                }
            } else {
                next = now;
            }
        }
        Ok(CycleStats::from_ns(v, block))
    })
    .join()
    .map_err(|_| "paced run panicked".to_string())?
}

/// Digest (FNV-1a over the f32 bits) of every output channel of every graph node over `cycles`
/// cycles of noise input: two runs agree iff every buffer the console computed is bit-identical.
pub fn output_digest(c: &mut Console, cycles: usize) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    let mut noise = Noise::default();
    let ids: Vec<_> = c.engine().graph().schedule().collect();
    for _ in 0..cycles {
        c.cycle(|_, buf| noise.fill(buf), |_, _| {});
        let g = c.engine().graph();
        for &id in &ids {
            for ch in 0..g.num_outputs_of(id) {
                for &s in g.port(wing_engine::Port::node(id, ch as u32)) {
                    for b in s.to_bits().to_le_bytes() {
                        h = (h ^ b as u64).wrapping_mul(0x0100_0000_01b3);
                    }
                }
            }
        }
    }
    h
}

/// Per-node serial profile of a console: `(name, level, mean ns)` in schedule order.
pub fn node_profile(c: &mut Console, cycles: usize) -> Vec<(String, u32, f64)> {
    let n = c.engine().graph().num_nodes();
    let mut ns = vec![0u64; n];
    let mut noise = Noise::default();
    for _ in 0..cycles {
        c.cycle_profiled(|_, buf| noise.fill(buf), &mut ns);
    }
    let g = c.engine().graph();
    g.schedule()
        .zip(ns)
        .map(|(id, t)| {
            (
                g.name_of(id).to_string(),
                g.level_of(id),
                t as f64 / cycles.max(1) as f64,
            )
        })
        .collect()
}

/// Lower bounds from a node profile for `threads` threads (driver + workers) under the level
/// scheduler: per level `max(longest node, sum / threads)`, summed (ns).
pub fn level_bound(profile: &[(String, u32, f64)], threads: usize) -> f64 {
    let nl = profile.iter().map(|p| p.1 + 1).max().unwrap_or(0) as usize;
    let mut sum = vec![0f64; nl];
    let mut max = vec![0f64; nl];
    for (_, l, t) in profile {
        sum[*l as usize] += t;
        max[*l as usize] = max[*l as usize].max(*t);
    }
    (0..nl).map(|l| max[l].max(sum[l] / threads as f64)).sum()
}

/// Cost of one model instance: mean ns per stereo block of `block` samples (default params, noise),
/// and a digest of its output (bit-identity check across optimisations).
pub fn model_cost(kind: BlockKind, id: &'static str, block: usize, blocks: usize) -> (f64, u64) {
    let mut m = create_model(kind, id);
    m.prepare(48_000.0, block);
    m.load_defaults();
    m.commit();
    m.reset();
    let mut noise = Noise::default();
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    let (mut l, mut r) = (vec![0f32; block], vec![0f32; block]);
    for _ in 0..blocks / 4 + 1 {
        noise.fill(&mut l);
        noise.fill(&mut r);
        m.process(&mut l, &mut r, None);
        for s in l.iter().chain(&r) {
            h = (h ^ s.to_bits() as u64).wrapping_mul(0x0100_0000_01b3);
        }
    }
    // mean per-block time without the slowest 1 % (preemption by other processes; models with
    // periodic heavy blocks, e.g. FFT frames, still count them when they are more frequent)
    let mut t_ns: Vec<u64> = (0..blocks)
        .map(|_| {
            noise.fill(&mut l);
            noise.fill(&mut r);
            let t = Instant::now();
            m.process(&mut l, &mut r, None);
            let d = t.elapsed().as_nanos() as u64;
            std::hint::black_box(&l);
            d
        })
        .collect();
    t_ns.sort_unstable();
    let keep = &t_ns[..(blocks - blocks / 100).max(1).min(t_ns.len())];
    (
        keep.iter().sum::<u64>() as f64 / keep.len().max(1) as f64,
        h,
    )
}

/// Cost survey of every model of every block family (`(family, id, (ns per block, digest))`,
/// heaviest first within a family).
#[allow(clippy::type_complexity)]
pub fn model_survey(
    block: usize,
    blocks: usize,
    only: Option<&str>,
) -> Vec<(&'static str, &'static str, (f64, u64))> {
    let fams: [(&'static str, BlockKind); 4] = [
        ("flt", BlockKind::FilterTool),
        ("dyn", BlockKind::Dynamics),
        ("eq", BlockKind::Eq(EqVariant::Bus)),
        ("fx", BlockKind::Fx),
    ];
    let mut out = Vec::new();
    for (name, kind) in fams {
        let mut v: Vec<_> = catalog(kind)
            .iter()
            .filter(|id| only.is_none_or(|o| o.split(',').any(|o| o == **id)))
            .map(|&id| (name, id, model_cost(kind, id, block, blocks)))
            .collect();
        v.sort_by(|a, b| b.2 .0.total_cmp(&a.2 .0));
        out.extend(v);
    }
    out
}

// ---- `wing-render --bench` -------------------------------------------------------------------

/// Usage of `wing-render --bench`.
pub const BENCH_USAGE: &str = "usage: wing-render --bench [--scene public,everything] [--block 64,128] \
[--workers 0-12 | 0,3,7] [--cycles N] [--paced N] [--paced-workers 7,8,10] [--profile] [--models [--model ID,..] [--model-blocks N]] [--digest N] [--no-lpt] [--set path=value]...";

/// Options of `wing-render --bench`.
#[derive(Debug, Clone)]
pub struct BenchArgs {
    pub scenes: Vec<Scene>,
    pub blocks: Vec<usize>,
    pub workers: Vec<usize>,
    /// Hot cycles per configuration.
    pub cycles: usize,
    /// Paced cycles per configuration (0 = skip the paced runs), at `paced_workers` workers.
    pub paced: usize,
    pub paced_workers: Vec<usize>,
    pub profile: bool,
    pub models: bool,
    /// Only these model ids (comma-separated) in the model survey.
    pub model_filter: Option<String>,
    /// Timed blocks per model.
    pub model_blocks: usize,
    /// Cycles of the output digest (0 = none), printed at 0 and 7 workers.
    pub digest: usize,
}

fn parse_list(s: &str) -> Result<Vec<usize>, String> {
    let mut v = Vec::new();
    for part in s.split(',').filter(|p| !p.is_empty()) {
        let num = |x: &str| x.parse::<usize>().map_err(|_| format!("bad number {part}"));
        match part.split_once('-') {
            Some((a, b)) => v.extend(num(a)?..=num(b)?),
            None => v.push(num(part)?),
        }
    }
    Ok(v)
}

/// Parse the arguments after `--bench`.
pub fn parse_bench_args(mut it: impl Iterator<Item = String>) -> Result<BenchArgs, String> {
    let mut a = BenchArgs {
        scenes: vec![Scene::Public, Scene::Everything],
        blocks: vec![64, 128],
        workers: (0..=12).collect(),
        cycles: 3000,
        paced: 0,
        paced_workers: vec![7, 8, 10],
        profile: false,
        models: false,
        model_filter: None,
        model_blocks: 2000,
        digest: 0,
    };
    let mut sets: Vec<(String, String)> = Vec::new();
    while let Some(flag) = it.next() {
        let mut val = || it.next().ok_or(format!("{flag} needs a value"));
        match flag.as_str() {
            "--scene" => {
                a.scenes = val()?
                    .split(',')
                    .filter(|s| *s != "none")
                    .map(|s| Scene::parse(s).ok_or(format!("unknown scene {s}")))
                    .collect::<Result<_, _>>()?
            }
            "--block" => a.blocks = parse_list(&val()?)?,
            "--workers" => a.workers = parse_list(&val()?)?,
            "--cycles" => a.cycles = val()?.parse().map_err(|e| format!("--cycles: {e}"))?,
            "--paced" => a.paced = val()?.parse().map_err(|e| format!("--paced: {e}"))?,
            "--paced-workers" => a.paced_workers = parse_list(&val()?)?,
            "--profile" => a.profile = true,
            "--models" => a.models = true,
            "--no-lpt" => LPT.store(false, std::sync::atomic::Ordering::Relaxed),
            "--set" => {
                let v = val()?;
                let (p, x) = v
                    .split_once('=')
                    .ok_or(format!("--set '{v}': expected path=value"))?;
                sets.push((p.trim().trim_start_matches('/').into(), x.trim().into()));
            }
            "--model" => a.model_filter = Some(val()?),
            "--model-blocks" => {
                a.model_blocks = val()?.parse().map_err(|e| format!("--model-blocks: {e}"))?
            }
            "--digest" => a.digest = val()?.parse().map_err(|e| format!("--digest: {e}"))?,
            "-h" | "--help" => return Err(BENCH_USAGE.into()),
            other => return Err(format!("unknown argument {other}\n{BENCH_USAGE}")),
        }
    }
    if a.blocks.iter().any(|b| !(16..=1024).contains(b)) {
        return Err("--block must be 16..1024".into());
    }
    if a.cycles == 0 {
        return Err("--cycles must be > 0".into());
    }
    if !sets.is_empty() && OVERRIDES.set(sets).is_err() {
        return Err("--set: overrides already installed".into());
    }
    Ok(a)
}

fn row(label: &str, s: &CycleStats) -> String {
    format!(
        "| {label:<24} | {:8.1} | {:8.1} | {:8.1} | {:8.1} | {:8.1} | {:5.1}% | {:5.1}% | {:5.1}% |",
        s.mean,
        s.p50,
        s.p99,
        s.p999,
        s.max,
        s.pct(s.mean),
        s.pct(s.p99),
        s.pct(s.p999),
    )
}

const HEADER: &str = "| config                   |  mean µs |   p50 µs |   p99 µs |  p999 µs |   max µs |  mean% |   p99% |  p999% |\n|---|---|---|---|---|---|---|---|---|";

/// Run the bench report (Markdown tables per scene/block on stdout).
pub fn bench_main(a: &BenchArgs) -> Result<(), String> {
    println!(
        "wing-render perf suite (T070): 48 kHz, {} hot cycles/config{}, {} cores",
        a.cycles,
        if a.paced > 0 {
            format!(", {} paced cycles/config", a.paced)
        } else {
            String::new()
        },
        std::thread::available_parallelism().map_or(0, |n| n.get())
    );
    if a.models {
        println!(
            "\n== model cost (ns per stereo 64-sample block, mean of the fastest 99 %, default params; output digest) =="
        );
        for (fam, id, (ns, digest)) in model_survey(64, a.model_blocks, a.model_filter.as_deref()) {
            println!("{fam:<4} {id:<10} {ns:9.0}   {digest:016x}");
        }
    }
    for &scene in &a.scenes {
        for &block in &a.blocks {
            let period = block as f64 * 1e6 / 48_000.0;
            println!(
                "\n== scene {} | block {block} (period {period:.1} µs) ==",
                scene.name()
            );
            if a.digest > 0 {
                for w in [0, 7] {
                    let mut store = scene.store()?;
                    let mut c = console(&mut store, block, w, false)?;
                    let d = output_digest(&mut c, a.digest);
                    println!("output digest ({} cycles, {w} workers): {d:016x}", a.digest);
                }
            }
            if a.profile {
                print_profile(scene, block)?;
            }
            println!("{HEADER}");
            for &w in &a.workers {
                let mut store = scene.store()?;
                let mut c = console(&mut store, block, w, false)?;
                let s = run_hot(&mut c, 200, a.cycles);
                println!("{}", row(&format!("hot, {w} workers"), &s));
            }
            if a.paced > 0 {
                for &w in &a.paced_workers {
                    let s = run_paced(scene, block, w, 300, a.paced)?;
                    println!("{}", row(&format!("paced RT, {w} workers"), &s));
                }
            }
        }
    }
    Ok(())
}

fn print_profile(scene: Scene, block: usize) -> Result<(), String> {
    let mut store = scene.store()?;
    let mut c = console(&mut store, block, 0, false)?;
    let prof = node_profile(&mut c, 500);
    let total: f64 = prof.iter().map(|p| p.2).sum();
    let mut top = prof.clone();
    top.sort_by(|x, y| y.2.total_cmp(&x.2));
    println!(
        "serial node work {:.1} µs ({} nodes); heaviest nodes:",
        total / 1e3,
        prof.len()
    );
    for (n, l, t) in top.iter().take(12) {
        println!("  {n:<10} level {l:2} {:8.1} µs", t / 1e3);
    }
    let nl = prof.iter().map(|p| p.1 + 1).max().unwrap_or(0);
    for l in 0..nl {
        let (cnt, sum, max) = prof
            .iter()
            .filter(|p| p.1 == l)
            .fold((0, 0.0, 0f64), |(c, s, m), p| (c + 1, s + p.2, m.max(p.2)));
        println!(
            "  level {l:2}: {cnt:3} nodes, sum {:8.1} µs, longest {:7.1} µs",
            sum / 1e3,
            max / 1e3
        );
    }
    for t in [1, 4, 8, 11] {
        println!(
            "  level-schedule lower bound, {t:2} threads: {:7.1} µs",
            level_bound(&prof, t) / 1e3
        );
    }
    Ok(())
}
