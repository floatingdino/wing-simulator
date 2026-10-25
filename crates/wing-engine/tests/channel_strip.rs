//! T035: channel/aux strip assembly: equality with a hand-built chain of wing-blocks blocks, process-order
//! switching, model catalog selectable through the strip, schema binding through the T03A pipeline,
//! side-chain keys and telemetry.

use wing_blocks::dynamics::{DynParams, Dynamics};
use wing_blocks::eq::{EqParams, EqVariant, PreEq, PreEqParams, StdEq};
use wing_blocks::filter::{FilterBlock, FilterParams, LcSlope};
use wing_blocks::gate::{Gate, GateParams};
use wing_blocks::input::{InputParams, InputStage};
use wing_engine::channel::{
    bind_aux_strip, bind_channel_strip, block, ChannelStrip, NoInserts, ProcBlock, ProcOrder,
    StripKeys, Tap, PROC_ORDERS,
};
use wing_engine::params::{
    telemetry_kind, ParamMapBuilder, ParamMsg, ParamPipeline, PipelineConfig, ProcessorAccess,
    TARGET_BASE,
};
use wing_engine::{NodeId, Processor};
use wing_model::{ParamStore, Schema};

const FS: f64 = 48_000.0;
const N: usize = 64;

fn set(s: &mut ChannelStrip, block: u8, param: u16, value: f32) {
    s.apply(&ParamMsg::Set {
        block,
        param,
        value,
    });
}

fn model(s: &mut ChannelStrip, block: u8, model: u16) {
    s.apply(&ParamMsg::Model { block, model });
}

/// A drum-ish test signal: decaying bursts with silence (exercises gate and compressor).
fn signal(len: usize) -> Vec<f32> {
    (0..len)
        .map(|i| {
            let ph = i % 4800;
            let env = if ph < 2400 {
                (-(ph as f32) / 600.0).exp()
            } else {
                0.0
            };
            env * 0.8 * (i as f32 * 2.0 * std::f32::consts::PI * 220.0 / 48_000.0).sin()
                + 0.002 * (i as f32 * 0.37).sin()
        })
        .collect()
}

/// Configure every block of a channel strip (trim, LC, gate, PRE EQ, EQ, comp).
fn configure(s: &mut ChannelStrip) {
    set(s, block::INPUT, 0, 3.0); // trim +3
    set(s, block::FILTER, 1, 1.0); // lc on
    set(s, block::FILTER, 2, 80.0); // lcf
    set(s, block::FILTER, 3, 3.0); // lcs 24
    model(s, block::GATE, 0); // GATE
    set(s, block::GATE, TARGET_BASE, -45.0); // thr
    set(s, block::GATE, TARGET_BASE + 1, 30.0); // range
    set(s, block::GATE, 0, 1.0); // on
    set(s, block::PEQ, 0, 1.0);
    set(s, block::PEQ, 4, 3.0); // 2g
    set(s, block::PEQ, 5, 1000.0); // 2f
    set(s, block::PEQ, 6, 1.0); // 2q
    model(s, block::EQ, 0); // STD
    set(s, block::EQ, TARGET_BASE + 7, 6.0); // 2g
    set(s, block::EQ, TARGET_BASE + 8, 300.0); // 2f
    set(s, block::EQ, 0, 1.0); // on
    model(s, block::DYN, 13); // COMP
    set(s, block::DYN, TARGET_BASE, -24.0); // thr
    set(s, block::DYN, TARGET_BASE + 1, 4.0); // ratio (float-enum value)
    set(s, block::DYN, 0, 1.0); // on
    s.settle();
}

/// The same processing built by hand from the wing-blocks blocks.
struct Manual {
    input: InputStage,
    filter: FilterBlock,
    gate: Gate,
    peq: PreEq,
    eq: StdEq,
    comp: Dynamics,
}

impl Manual {
    fn new() -> Self {
        let mut input = InputStage::new();
        input.prepare(FS, N);
        input.set_params(&InputParams {
            trim_db: 3.0,
            ..InputParams::default()
        });
        input.reset();
        let mut filter = FilterBlock::new();
        filter.prepare(FS, N);
        filter.set_params(&FilterParams {
            lc_on: true,
            lc_freq: 80.0,
            lc_slope: LcSlope::Db24,
            ..FilterParams::default()
        });
        filter.reset();
        let mut gate = Gate::new();
        gate.prepare(FS, N);
        gate.set_params(&GateParams {
            threshold_db: -45.0,
            range_db: 30.0,
            ..GateParams::gate_default()
        });
        gate.reset();
        let mut peq = PreEq::new();
        peq.prepare(FS, N);
        let mut pp = PreEqParams {
            on: true,
            ..PreEqParams::default()
        };
        pp.bands[1].gain_db = 3.0;
        pp.bands[1].freq = 1000.0;
        pp.bands[1].q = 1.0;
        peq.set_params(&pp);
        peq.reset();
        let mut eq = StdEq::new(EqVariant::Channel);
        eq.prepare(FS, N);
        let mut ep = EqParams {
            on: true,
            ..EqParams::defaults(EqVariant::Channel)
        };
        ep.bands[1].gain_db = 6.0;
        ep.bands[1].freq = 300.0;
        eq.set_params(&ep);
        eq.reset();
        let mut comp = Dynamics::new(DynParams {
            thr_db: -24.0,
            ratio: 4.0,
            ..DynParams::comp()
        });
        comp.prepare(FS, 2);
        Self {
            input,
            filter,
            gate,
            peq,
            eq,
            comp,
        }
    }

    fn process(&mut self, order: ProcOrder, l: &mut [f32], r: &mut [f32]) {
        self.input.process(l, r);
        self.filter.process(l, r);
        for b in order.0 {
            match b {
                ProcBlock::Gate => self.gate.process(l, r, None),
                ProcBlock::Eq => {
                    self.peq.process(l, r);
                    self.eq.process(l, r);
                }
                ProcBlock::Dyn => self.comp.process(&mut [&mut *l, &mut *r], None),
                ProcBlock::Insert => {}
            }
        }
    }
}

fn run_strip(s: &mut ChannelStrip, x: &[f32]) -> Vec<f32> {
    let (mut l, mut r) = (x.to_vec(), x.to_vec());
    for (a, b) in l.chunks_mut(N).zip(r.chunks_mut(N)) {
        s.process_stereo(a, b);
    }
    assert_eq!(
        l, r,
        "stereo-linked mono input stays identical on both sides"
    );
    l
}

fn run_manual(m: &mut Manual, order: ProcOrder, x: &[f32]) -> Vec<f32> {
    let (mut l, mut r) = (x.to_vec(), x.to_vec());
    for (a, b) in l.chunks_mut(N).zip(r.chunks_mut(N)) {
        m.process(order, a, b);
    }
    l
}

#[test]
fn strip_matches_manual_chain_in_every_order() {
    let x = signal(48_000);
    let mut outs = Vec::new();
    for (i, tok) in PROC_ORDERS.iter().enumerate() {
        let mut s = ChannelStrip::channel(FS, N);
        configure(&mut s);
        set(&mut s, block::STRIP, 0, i as f32);
        let order = ProcOrder::parse(tok).unwrap();
        assert_eq!(s.order(), order);
        let y = run_strip(&mut s, &x);
        let z = run_manual(&mut Manual::new(), order, &x);
        assert_eq!(y, z, "order {tok}: strip != manual chain");
        assert!(s.telemetry().dyn_gr_db > 0.0 || s.telemetry().gate_gr_db > 0.0);
        outs.push(y);
    }
    // The order matters: gate before vs after a +6 dB EQ and the compressor.
    let diff = |a: &[f32], b: &[f32]| {
        a.iter()
            .zip(b)
            .map(|(x, y)| (x - y).abs())
            .fold(0.0, f32::max)
    };
    assert!(diff(&outs[0], &outs[16]) > 1e-3, "GEDI vs DEGI");
    assert!(diff(&outs[0], &outs[8]) > 1e-3, "GEDI vs EGDI");
}

#[test]
fn order_switch_mid_stream_is_applied_at_block_boundary() {
    let x = signal(9600);
    let mut s = ChannelStrip::channel(FS, N);
    configure(&mut s);
    let mut m = Manual::new();
    let (mut l, mut r) = (x.clone(), x.clone());
    let (mut ml, mut mr) = (x.clone(), x.clone());
    for (k, ((a, b), (c, d))) in l
        .chunks_mut(N)
        .zip(r.chunks_mut(N))
        .zip(ml.chunks_mut(N).zip(mr.chunks_mut(N)))
        .enumerate()
    {
        let tok = if k < 50 { "GEDI" } else { "DEGI" };
        if k == 50 {
            set(&mut s, block::STRIP, 0, 16.0); // DEGI
        }
        s.process_stereo(a, b);
        m.process(ProcOrder::parse(tok).unwrap(), c, d);
    }
    assert_eq!(l, ml);
}

#[test]
fn bypass_semantics() {
    let x = signal(4800);
    let mut s = ChannelStrip::channel(FS, N);
    configure(&mut s);
    // everything off again -> transparent once the 5 ms fades are done
    set(&mut s, block::INPUT, 0, 0.0);
    set(&mut s, block::FILTER, 1, 0.0);
    for b in [block::GATE, block::EQ, block::DYN, block::PEQ] {
        set(&mut s, b, 0, 0.0);
    }
    s.settle();
    assert_eq!(run_strip(&mut s, &x), x);
    assert!(!s.gate_slot().is_running() && !s.dyn_slot().is_running());
    // switching a slot on fades in without a jump larger than the signal itself
    set(&mut s, block::EQ, 0, 1.0);
    s.apply(&ParamMsg::Commit);
    let y = run_strip(&mut s, &x);
    assert!(s.eq_slot().is_running());
    let step = y
        .windows(2)
        .map(|w| (w[1] - w[0]).abs())
        .fold(0.0, f32::max);
    assert!(step < 0.2, "{step}");
}

#[test]
fn every_catalog_model_is_selectable_through_the_strip() {
    let s_ = Schema::get();
    let x = signal(2048);
    for (kind, blk) in [
        ("ch/*/gate", block::GATE),
        ("ch/*/dyn", block::DYN),
        ("ch/*/eq", block::EQ),
        ("ch/*/flt", block::TOOL),
    ] {
        let g = s_.groups().iter().find(|g| g.key == kind).unwrap();
        let mut s = ChannelStrip::channel(FS, N);
        set(&mut s, blk, 0, 1.0);
        for m in s_.models(g) {
            model(&mut s, blk, m.enum_index as u16);
            s.apply(&ParamMsg::Commit);
            let y = run_strip(&mut s, &x);
            assert!(y.iter().all(|v| v.is_finite()), "{kind} {}", m.name);
            let slot = match blk {
                block::GATE => s.gate_slot(),
                block::DYN => s.dyn_slot(),
                block::EQ => s.eq_slot(),
                _ => s.tool_slot(),
            };
            assert_eq!(slot.model_index(), Some(m.enum_index as u16));
            assert!(slot.has_instance(), "{kind} {} has no instance", m.name);
        }
    }
    // aux dyn table (28 models, different indices)
    let g = s_.groups().iter().find(|g| g.key == "aux/*/dyn").unwrap();
    let mut a = ChannelStrip::aux(FS, N);
    for m in s_.models(g) {
        model(&mut a, block::DYN, m.enum_index as u16);
        assert!(a.dyn_slot().has_instance(), "aux {}", m.name);
    }
}

#[test]
fn external_key_drives_ducker_only_when_src_is_external() {
    let x = vec![0.05f32; N];
    let key = vec![0.5f32; N];
    let mut s = ChannelStrip::channel(FS, N);
    model(&mut s, block::GATE, 1); // DUCK
    set(&mut s, block::GATE, TARGET_BASE, -20.0); // thr
    set(&mut s, block::GATE, TARGET_BASE + 1, 20.0); // range
    set(&mut s, block::GATE, TARGET_BASE + 2, 0.0); // att
    set(&mut s, block::GATE, 0, 1.0);
    s.settle();
    let run = |s: &mut ChannelStrip| {
        let (mut l, mut r) = (x.clone(), x.clone());
        for _ in 0..50 {
            l.copy_from_slice(&x);
            r.copy_from_slice(&x);
            let keys = StripKeys {
                gate: Some((&key, &key)),
                dyn_: None,
            };
            s.process_with(&mut l, &mut r, keys, &mut NoInserts);
        }
        l[N - 1]
    };
    // src = SELF: the external key is ignored, the quiet input does not duck
    assert!((run(&mut s) - 0.05).abs() < 1e-4);
    set(&mut s, block::GATE_SC, 3, 1.0); // src = CH.1
    let y = run(&mut s);
    assert!((y - 0.005).abs() < 1e-3, "ducked by 20 dB: {y}");
    assert!(s.telemetry().gate_gr_db > 19.0);
    // key filter: HP at 2 kHz removes the DC key -> no ducking
    set(&mut s, block::GATE_SC, 0, 2.0); // HP12
    set(&mut s, block::GATE_SC, 1, 2000.0);
    for _ in 0..20 {
        run(&mut s);
    }
    assert!((run(&mut s) - 0.05).abs() < 1e-3);
}

#[test]
fn ptap_capture_and_tap_peaks() {
    let x = vec![0.25f32; N];
    let mut s = ChannelStrip::channel(FS, N);
    set(&mut s, block::INPUT, 0, 6.0206); // +6 dB trim: IN tap = 0.5
    set(&mut s, block::STRIP, 1, Tap::In as u8 as f32);
    s.settle();
    let (mut l, mut r) = (x.clone(), x.clone());
    s.process_stereo(&mut l, &mut r);
    let t = s.take_telemetry();
    assert!((t.tap_peak[Tap::In as usize] - 0.5).abs() < 1e-3);
    assert!((t.tap_peak[Tap::Pfl as usize] - 0.5).abs() < 1e-3);
    assert!((s.tap_signal().0[0] - 0.5).abs() < 1e-3);
    assert_eq!(s.take_telemetry().tap_peak, [0.0; 6]);
    assert_eq!(s.ptap(), Tap::In);
}

// ---- pipeline binding -------------------------------------------------------------------------------------

struct Strips(Vec<ChannelStrip>);

impl ProcessorAccess for Strips {
    fn processor(&mut self, id: NodeId) -> &mut dyn Processor {
        &mut self.0[id.index()]
    }
}

fn drain(pipe: &mut ParamPipeline, strips: &mut Strips) {
    while pipe.apply(strips).changes > 0 {}
}

#[test]
fn schema_binding_through_pipeline() {
    let store = ParamStore::new();
    let mut b = ParamMapBuilder::new();
    bind_channel_strip(&mut b, 1, NodeId(0)).unwrap();
    bind_channel_strip(&mut b, 40, NodeId(1)).unwrap();
    bind_aux_strip(&mut b, 8, NodeId(2)).unwrap();
    let mut strips = Strips(vec![
        ChannelStrip::channel(FS, N),
        ChannelStrip::channel(FS, N),
        ChannelStrip::aux(FS, N),
    ]);
    let mut pipe = ParamPipeline::new(&store, b.build(), PipelineConfig::default()).unwrap();
    pipe.sync_all(&store, &mut strips);
    let id = |p: &str| store.lookup(p).unwrap_or_else(|| panic!("{p}"));
    let setp = |p: &str, v: &str| {
        store
            .set_text(id(p), v, 0)
            .unwrap_or_else(|e| panic!("{p}={v}: {e}"));
    };
    setp("ch/1/proc", "EDGI");
    setp("ch/1/ptap", "PFL");
    setp("ch/1/dyn/mdl", "LA");
    setp("ch/1/dyn/on", "1");
    setp("ch/1/gate/mdl", "COMP"); // unidyn: a compressor in the gate slot
    setp("ch/1/gate/1", "-33");
    setp("ch/1/eq/on", "1");
    setp("ch/1/eq/mix", "120");
    setp("ch/1/eq/8", "4.5"); // 2g
    setp("ch/1/flt/tf", "1");
    setp("ch/1/flt/mdl", "AP2");
    setp("ch/1/gatesc/type", "BP");
    setp("ch/1/peq/2g", "-2");
    setp("ch/1/preins/on", "1");
    setp("ch/1/preins/ins", "FX3");
    setp("ch/1/postins/mode", "AUTO_X");
    setp("ch/40/in/set/trim", "-6");
    setp("aux/8/eq/mdl", "PULSAR");
    setp("aux/8/dyn/mdl", "COMP");
    setp("aux/8/dyn/1", "-12");
    drain(&mut pipe, &mut strips);
    let s = &strips.0[0];
    assert_eq!(s.order(), ProcOrder::parse("EDGI").unwrap());
    assert_eq!(s.ptap(), Tap::Pfl);
    assert_eq!(s.dyn_slot().model_index(), Some(23));
    assert!(s.dyn_slot().wrapper().0);
    assert_eq!(s.gate_slot().model_index(), Some(13));
    assert_eq!(s.gate_slot().model().unwrap().param(0), Some(-33.0));
    assert_eq!(s.eq_slot().wrapper().1, 120.0);
    assert_eq!(s.eq_slot().model().unwrap().param(7), Some(4.5));
    assert_eq!(s.tool_slot().model_index(), Some(3));
    assert!(s.tool_slot().wrapper().0);
    assert_eq!(
        s.gate_slot().key_filter().0,
        wing_blocks::gate::ScFilter::Bp
    );
    assert_eq!(s.pre_eq_params().bands[1].gain_db, -2.0);
    let (pre, post) = s.inserts();
    assert!(pre.on && pre.fx == 3 && pre.engaged() == Some(3));
    assert_eq!(post.mode, 1);
    assert!((strips.0[1].input().params().trim_db + 6.0).abs() < 1e-4);
    let a = &strips.0[2];
    assert_eq!(a.eq_slot().model_index(), Some(5));
    assert_eq!(a.dyn_slot().model_index(), Some(9));
    assert_eq!(a.dyn_slot().model().unwrap().param(0), Some(-12.0));

    // telemetry through the pipeline
    let mut rx = pipe.telemetry_rx().unwrap();
    let x = signal(N);
    for s in &mut strips.0 {
        let (mut l, mut r) = (x.clone(), x.clone());
        s.process_stereo(&mut l, &mut r);
    }
    pipe.publish_telemetry(&mut strips, 0);
    let mut peaks = 0;
    while let Some(t) = rx.pop() {
        if t.kind == telemetry_kind::TAP_PEAK && t.block == Tap::In as u8 && t.value > 0.0 {
            peaks += 1;
        }
    }
    assert_eq!(peaks, 3);
}

/// T039: the strip's meter cells: IN/PFL L/R peaks of a known sine, dyn key and GR equal to the strip
/// telemetry, gate key from the external key.
#[test]
fn strip_meters_levels_keys_and_gain_reduction() {
    use wing_engine::meters::{strip_cell, word, LevelBank, StripMeter};
    let bank = LevelBank::shared();
    let mut s = ChannelStrip::channel(FS, N);
    s.set_meter(StripMeter::at(bank.clone(), 2).into()); // ch 3
    set(&mut s, block::INPUT, 0, 6.0); // trim +6 dB
    set(&mut s, block::INPUT, 4, 1.0); // stereo source
    model(&mut s, block::DYN, 13); // COMP
    set(&mut s, block::DYN, TARGET_BASE, -30.0); // thr
    set(&mut s, block::DYN, TARGET_BASE + 1, 4.0); // ratio
    set(&mut s, block::DYN, 0, 1.0);
    set(&mut s, block::GATE_SC, 3, 1.0); // gate keyed externally (gate itself off)
    s.settle();
    let amp_l = 0.1f32; // -20 dBFS peak; R at half
    let key = vec![0.3f32; N];
    let mut n = 0usize;
    let mut run = |s: &mut ChannelStrip, blocks: usize| {
        for _ in 0..blocks {
            let mut l: Vec<f32> = (0..N)
                .map(|i| {
                    amp_l * ((n + i) as f32 * 2.0 * std::f32::consts::PI * 1000.0 / 48_000.0).sin()
                })
                .collect();
            let mut r: Vec<f32> = l.iter().map(|v| v * 0.5).collect();
            n += N;
            let keys = StripKeys {
                gate: Some((&key, &key)),
                dyn_: None,
            };
            s.process_with(&mut l, &mut r, keys, &mut NoInserts);
        }
    };
    run(&mut s, 200);
    bank.clear();
    run(&mut s, 20);
    let cell = |w| bank.get(strip_cell(2, w));
    let db = |v: f32| 20.0 * v.log10();
    // IN = after +6 dB trim: -14 dBFS (L), -20 dBFS (R)
    assert!(
        (db(cell(word::IN_L)) + 14.0).abs() < 0.05,
        "{}",
        db(cell(word::IN_L))
    );
    assert!((db(cell(word::IN_R)) + 20.0).abs() < 0.05);
    // PFL after the compressor: below IN
    assert!(cell(word::PRE_L) < cell(word::IN_L) * 0.9);
    let t = s.telemetry();
    assert!(t.dyn_gr_db > 3.0, "comp reduces: {}", t.dyn_gr_db);
    assert!(
        (cell(word::DYN_GR) - t.dyn_gr_db).abs() < 0.5,
        "GR {} vs {}",
        cell(word::DYN_GR),
        t.dyn_gr_db
    );
    // dyn key = its input (GEDI: post EQ = IN level here)
    assert!((db(cell(word::DYN_KEY)) + 14.0).abs() < 0.05);
    // gate is off: GR 0; its key meter reads the selected external key
    assert_eq!(cell(word::GATE_GR), 0.0);
    assert_eq!(cell(word::GATE_KEY), 0.3);
    // post-fader words are the mix node's
    assert_eq!(cell(word::OUT_L), 0.0);
}
