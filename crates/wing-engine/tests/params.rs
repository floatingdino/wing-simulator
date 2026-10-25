//! T03A: control -> RT parameter pipeline (ordering, scene recall, zipper noise, model-swap
//! clicks, telemetry, engine integration).

use wing_engine::params::strip::{bind_channel, block, ParamStrip};
use wing_engine::params::{
    telemetry_kind, BindError, ParamMapBuilder, ParamMsg, ParamPipeline, PipelineConfig,
    ProcessorAccess, TARGET_BASE,
};
use wing_engine::{Engine, EngineConfig, GraphBuilder, NodeIo, Port, ProcessCtx, Processor};
use wing_model::{NodeId as ParamId, ParamStore, ParamType, Value};

const FS: f64 = 48_000.0;
const BLOCK: usize = 64;

/// A strip that also logs every message it receives.
struct Logged {
    strip: ParamStrip,
    log: Vec<ParamMsg>,
}

impl Processor for Logged {
    fn process(&mut self, ctx: &ProcessCtx, io: &mut NodeIo<'_>) {
        self.strip.process(ctx, io)
    }
    fn set_param(&mut self, msg: &ParamMsg) {
        self.log.push(*msg);
        self.strip.set_param(msg);
    }
    fn telemetry(&mut self, out: &mut wing_engine::params::TelemetryWriter<'_>) {
        self.strip.telemetry(out)
    }
}

struct Strips(Vec<Logged>);

impl ProcessorAccess for Strips {
    fn processor(&mut self, id: wing_engine::NodeId) -> &mut dyn Processor {
        &mut self.0[id.index()]
    }
}

fn setup(channels: usize, cfg: PipelineConfig) -> (ParamStore, ParamPipeline, Strips) {
    let store = ParamStore::new();
    let mut b = ParamMapBuilder::new();
    for ch in 0..channels {
        bind_channel(&mut b, ch as u32 + 1, wing_engine::NodeId(ch as u32)).unwrap();
    }
    let mut strips = Strips(
        (0..channels)
            .map(|_| Logged {
                strip: ParamStrip::new(FS, BLOCK),
                log: Vec::new(),
            })
            .collect(),
    );
    let mut pipe = ParamPipeline::new(&store, b.build(), cfg).unwrap();
    pipe.sync_all(&store, &mut strips);
    for s in &mut strips.0 {
        s.log.clear();
    }
    (store, pipe, strips)
}

fn id(st: &ParamStore, p: &str) -> ParamId {
    st.lookup(p).unwrap_or_else(|| panic!("{p}"))
}

/// Engineering value of a stored parameter, as the pipeline converts it.
fn eng(st: &ParamStore, id: ParamId) -> f32 {
    let d = st.desc(id).unwrap();
    match d.ty {
        ParamType::LinF | ParamType::LogF | ParamType::Fader => f32::from_bits(st.get_raw(id)),
        _ => d.as_f64(st.get_raw(id)) as f32,
    }
}

fn drain(pipe: &mut ParamPipeline, strips: &mut Strips) -> usize {
    let mut cycles = 0;
    loop {
        let r = pipe.apply(strips);
        if r.changes == 0 {
            return cycles;
        }
        cycles += 1;
    }
}

/// Check every bound parameter of channel `ch` (1-based) against the store.
fn assert_consistent(st: &ParamStore, s: &ParamStrip, ch: u32) {
    let close = |a: f32, b: f32, what: &str| {
        assert!(
            (a - b).abs() <= 1e-3 * b.abs().max(1.0),
            "ch{ch} {what}: rt {a} vs store {b}"
        )
    };
    let p = |name: &str| id(st, &format!("ch/{ch}/{name}"));
    let ip = s.input_params();
    close(ip.trim_db, eng(st, p("in/set/trim")), "trim");
    close(ip.balance_db, eng(st, p("in/set/bal")), "bal");
    close(ip.delay, eng(st, p("in/set/dly")), "dly");
    assert_eq!(
        ip.delay_mode as u32,
        st.get_raw(p("in/set/dlymode")),
        "dlymode"
    );
    assert_eq!(ip.invert, st.get_raw(p("in/set/inv")) != 0);
    let fp = s.filter_params();
    close(fp.lc_freq, eng(st, p("flt/lcf")), "lcf");
    close(fp.hc_freq, eng(st, p("flt/hcf")), "hcf");
    assert_eq!(fp.lc_on, st.get_raw(p("flt/lc")) != 0);
    assert_eq!(fp.lc_slope as u32, st.get_raw(p("flt/lcs")));
    for (slot, name) in [(s.gate_slot(), "gate"), (s.dyn_slot(), "dyn")] {
        let sel = st.get_raw(p(&format!("{name}/mdl")));
        assert_eq!(slot.model_index(), Some(sel as u16), "ch{ch} {name} model");
        let (on, mix, gain) = slot.wrapper();
        assert_eq!(on, st.get_raw(p(&format!("{name}/on"))) != 0);
        close(mix, eng(st, p(&format!("{name}/mix"))), "mix");
        close(gain, eng(st, p(&format!("{name}/gain"))), "gain");
        if let Some(m) = slot.model() {
            for pos in 0..19u16 {
                let tid = p(&format!("{name}/{}", pos + 1));
                if let Some(v) = m.param(pos) {
                    if st.is_inactive(tid) {
                        continue;
                    }
                    let want = eng(st, tid);
                    // the gate ratio is reported as its enum index
                    close(v, want, &format!("{name} slot {}", pos + 1));
                }
            }
        }
    }
    let fdr = eng(st, p("fdr"));
    let mute = st.get_raw(p("mute")) != 0;
    let want = if mute || fdr <= -144.0 {
        0.0
    } else {
        10f32.powf(fdr / 20.0)
    };
    close(s.fader().target(), want, "fader");
}

#[test]
fn bind_errors() {
    let mut b = ParamMapBuilder::new();
    let n = wing_engine::NodeId(0);
    assert!(matches!(
        bind_channel(&mut b, 99, n),
        Err(BindError::UnknownPath(_))
    ));
    bind_channel(&mut b, 1, n).unwrap();
    assert!(matches!(
        bind_channel(&mut b, 1, n),
        Err(BindError::AlreadyBound(_))
    ));
    let map = b.build();
    let st = ParamStore::new();
    let (node, blk, param) = map.binding_of(id(&st, "ch/1/dyn/3")).unwrap();
    assert_eq!((node, blk, param), (n, block::DYN, Some(TARGET_BASE + 2)));
    assert_eq!(map.binding_of(id(&st, "ch/1/dyn/mdl")).unwrap().2, None);
    assert!(map.binding_of(id(&st, "ch/2/fdr")).is_none());
}

#[test]
fn sync_all_matches_store_defaults() {
    let (st, _pipe, strips) = setup(2, PipelineConfig::default());
    for (i, s) in strips.0.iter().enumerate() {
        assert_consistent(&st, &s.strip, i as u32 + 1);
    }
    // factory defaults: gate slot GATE, dyn slot COMP
    assert_eq!(strips.0[0].strip.gate_slot().model_index(), Some(0));
    assert_eq!(strips.0[0].strip.dyn_slot().model_index(), Some(13));
}

/// A model parameter queued *before* the selector change (and the defaults queued after it) must
/// never reach the block under the old model, even when the drain is one change per cycle.
#[test]
fn ordering_selector_before_model_params() {
    let cfg = PipelineConfig {
        budget: 1,
        ..Default::default()
    };
    let (st, mut pipe, mut strips) = setup(1, cfg);
    let slot1 = id(&st, "ch/1/dyn/1");
    let slot2 = id(&st, "ch/1/dyn/2");
    // dirty slot 1 under COMP first (queued ahead of the selector)
    st.set(slot1, Value::Float(-30.0)).unwrap();
    st.set_enum_name(id(&st, "ch/1/dyn/mdl"), "GATE", 0)
        .unwrap();
    st.set(slot1, Value::Float(-50.0)).unwrap(); // GATE thr
    st.set(slot2, Value::Float(12.0)).unwrap(); // GATE range
    let cycles = drain(&mut pipe, &mut strips);
    assert!(cycles >= 3, "budget 1 must spread the work: {cycles}");

    let log = &strips.0[0].log;
    let model_at = log
        .iter()
        .position(|m| matches!(m, ParamMsg::Model { block: b, model: 0 } if *b == block::DYN))
        .expect("Model message");
    for (i, m) in log.iter().enumerate() {
        if let ParamMsg::Set {
            block: b, param, ..
        } = *m
        {
            if b == block::DYN && param >= TARGET_BASE {
                assert!(
                    i > model_at,
                    "target {param} delivered before the selector: {log:?}"
                );
            }
        }
    }
    // the -50 dB value was never delivered as a COMP threshold (-60..0 would accept it)
    let s = &strips.0[0].strip;
    assert_eq!(s.dyn_slot().model_index(), Some(0));
    let m = s.dyn_slot().model().unwrap();
    assert_eq!(m.param(0), Some(-50.0));
    assert_eq!(m.param(1), Some(12.0));
    assert_eq!(m.param(2), Some(10.0)); // GATE attack default
    assert_consistent(&st, s, 1);
}

/// A model switch that lands in one cycle and its defaults in later cycles: the block is given
/// the new model's defaults immediately (never the old model's raw values).
#[test]
fn model_switch_split_across_cycles_uses_defaults() {
    let cfg = PipelineConfig {
        budget: 1,
        ..Default::default()
    };
    let (st, mut pipe, mut strips) = setup(1, cfg);
    st.set_enum_name(id(&st, "ch/1/gate/mdl"), "DUCK", 0)
        .unwrap();
    pipe.apply(&mut strips); // selector only
    let s = &strips.0[0].strip;
    let m = s.gate_slot().model().unwrap();
    assert_eq!(s.gate_slot().model_index(), Some(1));
    assert_eq!(m.param(0), Some(-20.0)); // DUCK thr default, not GATE's -40
    let rel = m.param(4).unwrap(); // DUCK rel ~500 ms (GATE: ~200)
    assert!((rel - 500.0).abs() < 10.0, "{rel}");
    drain(&mut pipe, &mut strips);
    assert_consistent(&st, &strips.0[0].strip, 1);
}

fn sine(amp: f64, f: f64, n: usize) -> f32 {
    (amp * (2.0 * std::f64::consts::PI * f * n as f64 / FS).sin()) as f32
}

/// Tiny deterministic PRNG.
struct Lcg(u64);
impl Lcg {
    fn next_f32(&mut self) -> f32 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        (self.0 >> 40) as f32 / (1u64 << 24) as f32
    }
}

/// Write a random value to every numeric parameter of the tree (a scene recall). Returns the number
/// of changed parameters.
pub fn recall_random_scene(st: &ParamStore, seed: u64) -> usize {
    let mut rng = Lcg(seed);
    let s = st.schema();
    let mut n = 0;
    for id in s.ids() {
        let Some(d) = st.desc(id) else { continue };
        if d.unused_slot || s.is_live(id) {
            continue;
        }
        if d.ty == ParamType::Str {
            // names/tags are part of a scene too (queued, unbound on the RT side)
            if let Ok(true) = st.set_str(id, &format!("s{seed}"), 0) {
                n += 1;
            }
            continue;
        }
        // a recalled scene differs from the current one: retry positions that change nothing
        for _ in 0..4 {
            if let Ok(true) = st.set_norm(id, rng.next_f32(), 0) {
                n += 1;
                break;
            }
        }
    }
    n
}

/// Scene recall: every changeable parameter of the tree (~28k sets, ~29.5k queue entries with the
/// model defaults), and a second recall issued while the first is still being applied (> 30k sets
/// in total). Each budget-limited cycle stays bounded and the RT state converges to the store.
#[test]
fn scene_recall_30k_completes_within_budgeted_cycles() {
    let cfg = PipelineConfig::default();
    let (st, mut pipe, mut strips) = setup(40, cfg);
    let mut changed = recall_random_scene(&st, 7);
    assert!(changed > 25_000, "{changed}");
    let mut max = 0;
    for _ in 0..5 {
        let r = pipe.apply(&mut strips);
        assert!(r.saturated && r.changes == cfg.budget);
    }
    changed += recall_random_scene(&st, 8);
    assert!(changed >= 30_000, "{changed}");
    let mut cycles = 0;
    loop {
        let r = pipe.apply(&mut strips);
        if r.changes == 0 {
            break;
        }
        assert!(r.changes <= cfg.budget);
        max = max.max(r.messages);
        cycles += 1;
    }
    // the queue holds each node at most once: <= node count entries outstanding
    let bound = st.schema().node_count().div_ceil(cfg.budget);
    assert!(cycles <= bound, "{cycles} cycles > {bound}");
    let stats = pipe.stats().snapshot();
    println!(
        "recall: {changed} sets, {} queue entries applied in {} cycles, max {max} msgs/cycle",
        stats.changes,
        cycles + 5
    );
    assert!(stats.changes >= 30_000);
    for (i, s) in strips.0.iter().enumerate() {
        assert_consistent(&st, &s.strip, i as u32 + 1);
    }
}

// ---- audio tests -------------------------------------------------------------------------------

fn fft(re: &mut [f64], im: &mut [f64]) {
    let n = re.len();
    let mut j = 0;
    for i in 1..n {
        let mut bit = n >> 1;
        while j & bit != 0 {
            j ^= bit;
            bit >>= 1;
        }
        j |= bit;
        if i < j {
            re.swap(i, j);
            im.swap(i, j);
        }
    }
    let mut len = 2;
    while len <= n {
        let ang = -2.0 * std::f64::consts::PI / len as f64;
        for start in (0..n).step_by(len) {
            for k in 0..len / 2 {
                let (s, c) = (ang * k as f64).sin_cos();
                let (a, b) = (start + k, start + k + len / 2);
                let (tr, ti) = (re[b] * c - im[b] * s, re[b] * s + im[b] * c);
                re[b] = re[a] - tr;
                im[b] = im[a] - ti;
                re[a] += tr;
                im[a] += ti;
            }
        }
        len <<= 1;
    }
}

/// Energy above `f_lo` relative to the total, in dB (Blackman-Harris window).
fn hf_ratio_db(x: &[f32], f_lo: f64) -> f64 {
    let n = x.len();
    let (a0, a1, a2, a3) = (0.35875, 0.48829, 0.14128, 0.01168);
    let mut re: Vec<f64> = x
        .iter()
        .enumerate()
        .map(|(i, &v)| {
            let t = 2.0 * std::f64::consts::PI * i as f64 / n as f64;
            v as f64 * (a0 - a1 * t.cos() + a2 * (2.0 * t).cos() - a3 * (3.0 * t).cos())
        })
        .collect();
    let mut im = vec![0.0; n];
    fft(&mut re, &mut im);
    let k_lo = (f_lo / FS * n as f64) as usize;
    let p = |k: usize| re[k] * re[k] + im[k] * im[k];
    let total: f64 = (0..n / 2).map(p).sum();
    let hf: f64 = (k_lo..n / 2).map(p).sum();
    10.0 * (hf / total).log10()
}

/// Fader ramp -40 -> 0 dB over 1 s sent as 50 control updates (every 15 cycles); returns the
/// HF/total energy ratio of a 1 kHz sine through the strip.
fn fader_ramp(smoothing: bool) -> f64 {
    let (st, mut pipe, mut strips) = setup(1, PipelineConfig::default());
    if !smoothing {
        strips.0[0].strip.fader_mut().set_smoothing(0.0);
    }
    let fdr = id(&st, "ch/1/fdr");
    st.set(fdr, Value::Float(-40.0)).unwrap();
    let cycles = (FS as usize) / BLOCK;
    let mut out = Vec::with_capacity(cycles * BLOCK);
    let (mut l, mut r) = (vec![0.0f32; BLOCK], vec![0.0f32; BLOCK]);
    for c in 0..cycles {
        if c % 15 == 0 {
            let db = -40.0 + 40.0 * (c as f32 / cycles as f32);
            st.set(fdr, Value::Float(db)).unwrap();
        }
        pipe.apply(&mut strips);
        for (i, v) in l.iter_mut().enumerate() {
            *v = sine(0.5, 1000.0, c * BLOCK + i);
        }
        r.copy_from_slice(&l);
        strips.0[0].strip.process_stereo(&mut l, &mut r);
        out.extend_from_slice(&l);
    }
    hf_ratio_db(&out[16_384..16_384 + 16_384], 3_000.0)
}

#[test]
fn zipper_noise_fader_ramp() {
    let smooth = fader_ramp(true);
    let raw = fader_ramp(false);
    println!("fader ramp HF/total: smoothed {smooth:.1} dB, unsmoothed {raw:.1} dB");
    assert!(smooth < -90.0, "zipper noise {smooth:.1} dB");
    assert!(
        raw > smooth + 20.0,
        "test not sensitive: {raw:.1} vs {smooth:.1}"
    );
}

/// Swap models on a steady sine; returns max |second difference| after the first switch divided
/// by its steady-state value before it.
fn swap_click_ratio(xfade: bool) -> f32 {
    let (st, mut pipe, mut strips) = setup(1, PipelineConfig::default());
    if !xfade {
        strips.0[0].strip.dyn_slot_mut().set_xfade_samples(0);
    }
    st.set(id(&st, "ch/1/fdr"), Value::Float(0.0)).unwrap();
    st.set(id(&st, "ch/1/dyn/on"), Value::Int(1)).unwrap();
    let mdl = id(&st, "ch/1/dyn/mdl");
    let seq = ["GATE", "EXP", "DUCK", "LA", "COMP", "GATE", "COMP"];
    let per = 75; // cycles between switches (100 ms)
    let total = per * (seq.len() + 1);
    let (mut l, mut r) = (vec![0.0f32; BLOCK], vec![0.0f32; BLOCK]);
    let mut out = Vec::new();
    for c in 0..total {
        if c >= per && c % per == 0 {
            st.set_enum_name(mdl, seq[c / per - 1], 0).unwrap();
        }
        // a model switch and its parameters land within one cycle at the default budget
        pipe.apply(&mut strips);
        for (i, v) in l.iter_mut().enumerate() {
            *v = sine(0.25, 1000.0, c * BLOCK + i);
        }
        r.copy_from_slice(&l);
        strips.0[0].strip.process_stereo(&mut l, &mut r);
        out.extend_from_slice(&l);
    }
    let d2 = |s: &[f32]| {
        s.windows(3)
            .map(|w| (w[2] - 2.0 * w[1] + w[0]).abs())
            .fold(0.0f32, f32::max)
    };
    let steady = d2(&out[(per / 2) * BLOCK..per * BLOCK]);
    let after = d2(&out[per * BLOCK..]);
    after / steady
}

#[test]
fn model_swap_is_click_free() {
    let with = swap_click_ratio(true);
    let without = swap_click_ratio(false);
    println!("model swap d2 ratio: crossfade {with:.2}, hard switch {without:.2}");
    assert!(with < 1.5, "click on model swap: {with:.2}x steady state");
    assert!(without > 5.0, "test not sensitive: {without:.2}");
}

#[test]
fn telemetry_reports_gain_reduction() {
    let (st, mut pipe, mut strips) = setup(1, PipelineConfig::default());
    let mut rx = pipe.telemetry_rx().unwrap();
    assert!(pipe.telemetry_rx().is_none());
    st.set(id(&st, "ch/1/fdr"), Value::Float(0.0)).unwrap();
    st.set(id(&st, "ch/1/dyn/on"), Value::Int(1)).unwrap();
    st.set(id(&st, "ch/1/dyn/thr"), Value::Float(-30.0))
        .unwrap();
    let (mut l, mut r) = (vec![0.0f32; BLOCK], vec![0.0f32; BLOCK]);
    for c in 1..=320u64 {
        pipe.apply(&mut strips);
        for (i, v) in l.iter_mut().enumerate() {
            *v = sine(0.9, 500.0, c as usize * BLOCK + i);
        }
        r.copy_from_slice(&l);
        strips.0[0].strip.process_stereo(&mut l, &mut r);
        pipe.publish_telemetry(&mut strips, c);
    }
    let mut last_dyn = None;
    let mut n = 0;
    while let Some(t) = rx.pop() {
        n += 1;
        if t.block == block::DYN && t.kind == telemetry_kind::GAIN_REDUCTION_DB {
            last_dyn = Some(t.value);
        }
    }
    assert_eq!(n, 2 * 320 / 16);
    let gr = last_dyn.unwrap();
    assert!(gr > 10.0, "GR {gr}");
    assert_eq!(rx.dropped(), 0);
}

#[test]
fn engine_integration() {
    let st = ParamStore::new();
    let mut g = GraphBuilder::new(BLOCK, FS as f32, 1);
    let node = g
        .add_node("ch1", 2, &[Port::External(0)], ParamStrip::new(FS, BLOCK))
        .unwrap();
    let mut b = ParamMapBuilder::new();
    bind_channel(&mut b, 1, node).unwrap();
    let mut e = Engine::new(
        g.build().unwrap(),
        EngineConfig {
            workers: 1,
            rt_workers: false,
            ..Default::default()
        },
    );
    let mut pipe = ParamPipeline::new(&st, b.build(), PipelineConfig::default()).unwrap();
    pipe.sync_all(&st, &mut e);
    let run = |e: &mut Engine, pipe: &mut ParamPipeline| {
        let mut peak = 0.0f32;
        for _ in 0..80 {
            e.external_mut(0).fill(0.5);
            pipe.run_cycle(e);
            peak = e
                .port(Port::node(node, 0))
                .iter()
                .fold(0.0, |a, &v| a.max(v.abs()));
        }
        peak
    };
    // factory fader is -oo
    assert_eq!(run(&mut e, &mut pipe), 0.0);
    st.set(id(&st, "ch/1/fdr"), Value::Float(-6.0)).unwrap();
    let p = run(&mut e, &mut pipe);
    assert!((p - 0.5 * 10f32.powf(-6.0 / 20.0)).abs() < 1e-3, "{p}");
    st.set(id(&st, "ch/1/mute"), Value::Int(1)).unwrap();
    assert_eq!(run(&mut e, &mut pipe), 0.0); // exact silence once settled
}
