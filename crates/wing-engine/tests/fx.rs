//! T050 / T044: FX rack slots hosted by insert points in the full mix graph: insert vs "send" (bus
//! insert) use, FX return into an aux via the bus re-input, FX SEND legs on the output patch, slot
//! exclusivity and `$stat`, lite-slot restrictions, latency reporting and the channel automixer.

mod common;

use common::rig::*;
use wing_blocks::model::{Model, ParamValue};
use wing_blocks::models::fx_delay::StereoDelay;
use wing_engine::channel::{block as chb, ChannelStrip, InsertPos};
use wing_engine::fx::{FxRack, InsertStat};
use wing_engine::mix::{InputSignal as I, MixConfig, MixSignal as M, MixStrip};
use wing_engine::params::strip::RtModel;
use wing_engine::params::ParamMsg;
use wing_engine::routing::{Side, Strip};
use wing_engine::Port;
use wing_model::ParamStore;

/// Everything of an input strip off (transparent PFL).
fn plain_input(store: &ParamStore, p: &str) {
    for k in [
        "gate/on", "eq/on", "dyn/on", "peq/on", "flt/lc", "flt/hc", "flt/tf",
    ] {
        if store.lookup(&format!("{p}/{k}")).is_some() {
            seti(store, &format!("{p}/{k}"), 0);
        }
    }
}

/// EQ and dynamics of a bus/main/matrix off.
fn plain_mix(store: &ParamStore, p: &str) {
    seti(store, &format!("{p}/eq/on"), 0);
    seti(store, &format!("{p}/dyn/on"), 0);
}

/// Slot `n` = ST-DL with short times and wide-open filters; returns a standalone model with the
/// store's (quantised) values.
fn st_dl(store: &ParamStore, n: u8, time_ms: f32, feed: f32, mix: f32) -> StereoDelay {
    sett(store, &format!("fx/{n}/mdl"), "ST-DL");
    setf(store, &format!("fx/{n}/fxmix"), mix);
    setf(store, &format!("fx/{n}/1"), time_ms);
    setf(store, &format!("fx/{n}/6"), feed);
    setf(store, &format!("fx/{n}/7"), 20.0);
    setf(store, &format!("fx/{n}/8"), 20_000.0);
    setf(store, &format!("fx/{n}/9"), 20.0);
    setf(store, &format!("fx/{n}/10"), 20_000.0);
    let mut m = StereoDelay::new();
    m.prepare(48_000.0, BLOCK);
    for s in 1..=10u16 {
        m.set_param(s, ParamValue::Float(getf(store, &format!("fx/{n}/{s}"))));
    }
    m.commit();
    m.reset();
    m
}

fn send_leg(r: &Rig, leg: u16) -> Vec<f32> {
    r.port(r.nodes.send_port(leg).unwrap())
}

#[track_caller]
fn assert_close(a: &[f32], b: &[f32], tol: f32, what: &str) {
    assert_eq!(a.len(), b.len());
    let d = a
        .iter()
        .zip(b)
        .fold(0.0f32, |m, (x, y)| m.max((x - y).abs()));
    assert!(d <= tol, "{what}: max diff {d}");
}

fn peak(x: &[f32]) -> f32 {
    x.iter().fold(0.0f32, |a, v| a.max(v.abs()))
}

/// Insert use: ch 1 Ins1 = FX1 (ST-DL). The strip output is the slot's wet/dry blend of the model
/// running on the slot input (= the FX SEND leg, also routed to USB out 1).
#[test]
fn channel_insert_runs_slot_with_fxmix() {
    let store = ParamStore::new();
    connect_usb(&store, "ch/1", 1);
    plain_input(&store, "ch/1");
    seti(&store, "ch/1/preins/on", 1);
    sett(&store, "ch/1/preins/ins", "FX1");
    let mut model = st_dl(&store, 1, 3.0, 40.0, 100.0);
    sett(&store, "io/out/USB/1/grp", "SEND");
    seti(&store, "io/out/USB/1/in", 1);
    let mut r = rig(&store, MixConfig::default(), 2);
    assert_eq!(r.fx.owner(1), Some((Strip::Ch(1), InsertPos::Pre)));
    assert_eq!(
        r.fx.insert_state(Strip::Ch(1), InsertPos::Pre),
        InsertStat::Ok
    );
    assert_eq!(r.fx.owner(2), None);
    for k in 0..40 {
        r.cycle(&[1]);
        let (sl, sr) = (send_leg(&r, 1), send_leg(&r, 2));
        assert!(peak(&sl) > 0.01, "slot input captured");
        assert_eq!(
            r.port(Port::node(r.nodes.usb_out, 0)),
            sl,
            "SEND 1 -> USB out 1"
        );
        let (mut wl, mut wr) = (sl.clone(), sr.clone());
        model.process(&mut wl, &mut wr, None);
        if k >= 8 {
            // past the NONE -> ST-DL crossfade
            assert_close(&r.input(0, I::Pfl, Side::L), &wl, 1e-6, "PFL L = wet");
            assert_close(&r.input(0, I::Pfl, Side::R), &wr, 1e-6, "PFL R = wet");
        }
    }
    assert_eq!(peak(&send_leg(&r, 3)), 0.0, "unused slot: silent SEND leg");

    // fxmix 50 %: out = dry + 0.5 (wet - dry), smoothed over 5 ms
    setf(&store, "fx/1/fxmix", 50.0);
    for k in 0..20 {
        r.cycle(&[1]);
        let (sl, sr) = (send_leg(&r, 1), send_leg(&r, 2));
        let (mut wl, mut wr) = (sl.clone(), sr.clone());
        model.process(&mut wl, &mut wr, None);
        if k >= 8 {
            let mix: Vec<f32> = sl.iter().zip(&wl).map(|(d, w)| d + 0.5 * (w - d)).collect();
            assert_close(&r.input(0, I::Pfl, Side::L), &mix, 1e-5, "PFL = 50 % mix");
        }
    }

    // insert off: dry
    seti(&store, "ch/1/preins/on", 0);
    r.cycles(2, &[1]);
    let x: Vec<f32> = (0..BLOCK as u64)
        .map(|i| sig(0, r.t - BLOCK as u64 + i))
        .collect();
    assert_close(&r.input(0, I::Pfl, Side::L), &x, 1e-6, "insert off = dry");
    assert_eq!(peak(&send_leg(&r, 1)), 0.0, "bypassed slot does not run");
}

/// "Send" use: ch 1 -> bus 1 whose Ins1 hosts FX2; bus 1 returns to main 1 and is re-input on aux 1
/// (WING has no FX return source group: the return is the bus).
#[test]
fn bus_insert_send_fx_returns_to_main_and_aux() {
    let store = ParamStore::new();
    connect_usb(&store, "ch/1", 1);
    plain_input(&store, "ch/1");
    setf(&store, "ch/1/fdr", 0.0);
    seti(&store, "ch/1/main/1/on", 0);
    seti(&store, "ch/1/send/1/on", 1);
    setf(&store, "ch/1/send/1/lvl", 0.0);
    sett(&store, "ch/1/send/1/mode", "POST");
    plain_mix(&store, "bus/1");
    setf(&store, "bus/1/fdr", 0.0);
    seti(&store, "bus/1/preins/on", 1);
    sett(&store, "bus/1/preins/ins", "FX2");
    seti(&store, "bus/1/main/1/on", 1);
    setf(&store, "bus/1/main/1/lvl", 0.0);
    plain_mix(&store, "main/1");
    setf(&store, "main/1/fdr", 0.0);
    let mut model = st_dl(&store, 2, 4.0, 30.0, 100.0);
    // aux 1 <- BUS 1/2 (re-input, one block late)
    sett(&store, "aux/1/in/conn/grp", "BUS");
    seti(&store, "aux/1/in/conn/in", 1);
    plain_input(&store, "aux/1");
    let mut r = rig(&store, MixConfig::default(), 3);
    assert_eq!(r.fx.owner(2), Some((Strip::Bus(1), InsertPos::Pre)));
    let mut prev_post: Option<Vec<f32>> = None;
    for k in 0..40 {
        r.cycle(&[1]);
        let (sl, sr) = (send_leg(&r, 3), send_leg(&r, 4));
        let (mut wl, mut wr) = (sl.clone(), sr.clone());
        model.process(&mut wl, &mut wr, None);
        if k >= 8 {
            assert!(peak(&sl) > 0.01);
            assert_close(
                &r.mix(Strip::Bus(1), M::Pfl, Side::L),
                &wl,
                1e-6,
                "bus PFL = wet",
            );
            assert_close(
                &r.mix(Strip::Bus(1), M::Pfl, Side::R),
                &wr,
                1e-6,
                "bus PFL R",
            );
            let post = r.mix(Strip::Bus(1), M::Post, Side::L);
            let (g, res) = fit(&post, &r.mix(Strip::Main(1), M::Pfl, Side::L));
            assert!(
                res < 1e-4 && g > 0.1,
                "main 1 carries the FX return: g {g} res {res}"
            );
            if let Some(p) = &prev_post {
                assert_close(
                    &r.input(40, I::Pfl, Side::L),
                    p,
                    1e-6,
                    "aux 1 = bus 1 one block late",
                );
            }
            prev_post = Some(post);
        }
    }
}

/// One insert per slot: the lowest insert id owns it, the others pass through with `$stat` N/A; a
/// release hands the slot over.
#[test]
fn slot_serves_one_insert() {
    let store = ParamStore::new();
    connect_usb(&store, "ch/1", 1);
    plain_input(&store, "ch/1");
    seti(&store, "ch/1/preins/on", 1);
    sett(&store, "ch/1/preins/ins", "FX3");
    seti(&store, "ch/1/send/2/on", 1);
    setf(&store, "ch/1/send/2/lvl", 0.0);
    plain_mix(&store, "bus/2");
    seti(&store, "bus/2/postins/on", 1);
    sett(&store, "bus/2/postins/ins", "FX3");
    st_dl(&store, 3, 2.0, 0.0, 100.0);
    let mut r = rig(&store, MixConfig::default(), 2);
    r.cycles(10, &[1]);
    assert_eq!(r.fx.owner(3), Some((Strip::Ch(1), InsertPos::Pre)));
    assert_eq!(
        r.fx.insert_state(Strip::Bus(2), InsertPos::Post),
        InsertStat::NotAvailable
    );
    // ch 1 runs the delay: its PFL is not its input
    let dry: Vec<f32> = (0..BLOCK as u64)
        .map(|i| sig(0, r.t - BLOCK as u64 + i))
        .collect();
    assert_close(&send_leg(&r, 5), &dry, 1e-6, "SEND 5 = ch 1 insert input");
    assert!(r
        .input(0, I::Pfl, Side::L)
        .iter()
        .zip(&dry)
        .any(|(a, b)| (a - b).abs() > 1e-3));
    // bus 2 passes through: PFL = its sum = ch 1 PFL (send at 0 dB, centre pan, constant power)
    let (g, res) = fit(
        &r.input(0, I::Pfl, Side::L),
        &r.mix(Strip::Bus(2), M::Pfl, Side::L),
    );
    assert!(
        res < 1e-4 && (g - std::f64::consts::FRAC_1_SQRT_2).abs() < 1e-3,
        "{g} {res}"
    );

    sett(&store, "ch/1/preins/ins", "NONE");
    r.cycles(10, &[1]);
    assert_eq!(r.fx.owner(3), Some((Strip::Bus(2), InsertPos::Post)));
    assert_eq!(
        r.fx.insert_state(Strip::Bus(2), InsertPos::Post),
        InsertStat::Ok
    );
    assert_eq!(
        r.fx.insert_state(Strip::Ch(1), InsertPos::Pre),
        InsertStat::None
    );
    let dry: Vec<f32> = (0..BLOCK as u64)
        .map(|i| sig(0, r.t - BLOCK as u64 + i))
        .collect();
    assert_close(
        &r.input(0, I::Pfl, Side::L),
        &dry,
        1e-6,
        "ch 1 dry after release",
    );
    let (_, res) = fit(&send_leg(&r, 5), &r.mix(Strip::Bus(2), M::Pfl, Side::L));
    assert!(res > 0.1, "bus 2 now runs the delay");
}

/// Slots 9-16 only offer the lite models; their selector cannot even name a premium model.
#[test]
fn lite_slots_exclude_premium_models() {
    let store = ParamStore::new();
    for p in ["fx/9/mdl", "fx/16/mdl"] {
        assert!(store.set_text(id(&store, p), "ST-DL", 0).is_err());
        assert!(store.set_text(id(&store, p), "HALL", 0).is_err());
    }
    sett(&store, "fx/9/mdl", "GEQ");
    sett(&store, "fx/1/mdl", "GEQ");
    let r = rig(&store, MixConfig::default(), 0);
    assert_eq!(r.fx.slot_instances(1), 64);
    assert_eq!(r.fx.slot_instances(9), 38);
    // same model, different enum index per group
    assert_eq!(r.fx.slot_model(9), 2);
    assert_eq!(r.fx.slot_model(1), 28);
}

/// Pass-through test model with a fixed latency.
struct Lat(usize);

impl RtModel for Lat {
    fn load_defaults(&mut self) {}
    fn set_param(&mut self, _pos: u16, _value: f32) {}
    fn commit(&mut self) {}
    fn reset(&mut self) {}
    fn process(&mut self, _l: &mut [f32], _r: &mut [f32]) {}
    fn latency(&self) -> usize {
        self.0
    }
}

/// Slot latency is reported per slot and added to the strips whose engaged inserts own the slot.
#[test]
fn insert_latency_reporting() {
    let rack = FxRack::with_factory(48_000.0, BLOCK, |slot, _i, name| {
        (name == "GEQ").then(|| Box::new(Lat(10 * slot as usize)) as Box<dyn RtModel>)
    });
    let select = |n: u8, model: u16| unsafe {
        rack.with_slot(n, |s| {
            s.apply(&ParamMsg::Model { block: 0, model });
            s.apply(&ParamMsg::Commit);
            s.reset();
        })
    };
    select(4, 28); // GEQ in a full slot
    select(12, 2); // GEQ in a lite slot
    assert_eq!(rack.slot_latency(4), 40);
    assert_eq!(rack.slot_latency(12), 120);
    assert_eq!(rack.slot_latency(5), 0);

    let set = |s: &mut ChannelStrip, block, param, value| {
        s.apply(&ParamMsg::Set {
            block,
            param,
            value,
        })
    };
    let mut ch = ChannelStrip::channel(48_000.0, BLOCK);
    let mut host = rack.host(Strip::Ch(2));
    set(&mut ch, chb::PREINS, 0, 1.0);
    set(&mut ch, chb::PREINS, 1, 4.0);
    ch.apply(&ParamMsg::Commit);
    host.assign(InsertPos::Pre, 4);
    assert_eq!(ch.latency_with(&*host), ch.latency() + 40);
    // insert off: no latency
    set(&mut ch, chb::PREINS, 0, 0.0);
    assert_eq!(ch.latency_with(&*host), ch.latency());

    // a bus whose Ins2 claims FX12, and a main that claims FX4 too (N/A: no latency)
    let mut bus = MixStrip::new(Strip::Bus(3), 48_000.0, BLOCK);
    bus.set_insert_host(rack.host(Strip::Bus(3)));
    let hook = wing_engine::mix::strip::hook::POSTINS;
    bus.apply(&ParamMsg::Set {
        block: hook,
        param: 0,
        value: 1.0,
    });
    bus.apply(&ParamMsg::Set {
        block: hook,
        param: 1,
        value: 12.0,
    });
    bus.apply(&ParamMsg::Commit);
    assert_eq!(bus.latency(), 120);
    let mut main = MixStrip::new(Strip::Main(1), 48_000.0, BLOCK);
    main.set_insert_host(rack.host(Strip::Main(1)));
    let pre = wing_engine::mix::strip::hook::PREINS;
    main.apply(&ParamMsg::Set {
        block: pre,
        param: 0,
        value: 1.0,
    });
    main.apply(&ParamMsg::Set {
        block: pre,
        param: 1,
        value: 4.0,
    });
    main.apply(&ParamMsg::Commit);
    assert_eq!(rack.owner(4), Some((Strip::Ch(2), InsertPos::Pre)));
    assert_eq!(main.latency(), 0);
}

/// Channel post-insert AUTO_X / AUTO_Y: Dugan gain sharing with weights and group enables.
#[test]
fn automix_shares_gain() {
    let store = ParamStore::new();
    for ch in 1..=3 {
        let p = format!("ch/{ch}");
        connect_usb(&store, &p, 1);
        plain_input(&store, &p);
    }
    for ch in 1..=2 {
        seti(&store, &format!("ch/{ch}/postins/on"), 1);
        sett(&store, &format!("ch/{ch}/postins/mode"), "AUTO_X");
    }
    seti(&store, "cfg/amix/x", 1);
    let mut r = rig(&store, MixConfig::default(), 2);
    let gain = |r: &Rig, ch: usize| {
        let (g, res) = fit(&r.input(2, I::Pfl, Side::L), &r.input(ch, I::Pfl, Side::L));
        assert!(res < 1e-2, "ch{} residual {res}", ch + 1);
        db(g)
    };
    // two equal members: -3 dB each (constant total power)
    r.cycles(200, &[1]);
    assert!((gain(&r, 0) + 3.01).abs() < 0.1, "{}", gain(&r, 0));
    assert!((gain(&r, 1) + 3.01).abs() < 0.1, "{}", gain(&r, 1));

    // weight +6 dB on ch 1: 10^0.6 : 1 power share
    setf(&store, "ch/1/postins/w", 6.0);
    r.cycles(200, &[1]);
    let w = 10f64.powf(0.6);
    assert!((gain(&r, 0) - 10.0 * (w / (w + 1.0)).log10()).abs() < 0.1);
    assert!((gain(&r, 1) - 10.0 * (1.0 / (w + 1.0)).log10()).abs() < 0.1);

    // ch 2 moves to group Y: both alone in their group -> 0 dB
    sett(&store, "ch/2/postins/mode", "AUTO_Y");
    r.cycles(200, &[1]);
    assert!(gain(&r, 0).abs() < 0.1 && gain(&r, 1).abs() < 0.1);

    // back to X, group X disabled: pass-through
    sett(&store, "ch/2/postins/mode", "AUTO_X");
    seti(&store, "cfg/amix/x", 0);
    r.cycles(200, &[1]);
    assert!(gain(&r, 0).abs() < 0.01 && gain(&r, 1).abs() < 0.01);
}
