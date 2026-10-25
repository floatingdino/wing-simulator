//! Routing resolved from fixtures/PUBLIC.snap: which USB input feeds each input strip with the
//! global ALT switch off/on, what every USB output carries, and the signals through the graph.

mod common;

use std::collections::BTreeMap;
use std::sync::Arc;

use wing_engine::routing::*;
use wing_engine::{Engine, EngineConfig, GraphBuilder, Port};
use wing_model::{ParamStore, Value};

fn public_store() -> ParamStore {
    let store = ParamStore::new();
    let (applied, _skipped) = common::load_routing(&store, common::PUBLIC_SNAP);
    assert!(applied > 3000, "applied {applied}");
    store
}

fn strip_name(i: usize) -> String {
    if i < 40 {
        format!("ch{}", i + 1)
    } else {
        format!("aux{}", i - 39)
    }
}

/// strip -> USB inputs (1-based) feeding its L/R legs.
fn usb_map(t: &RoutingTable) -> BTreeMap<String, Vec<u8>> {
    let mut m = BTreeMap::new();
    for i in 0..INPUT_STRIPS {
        let r = t.input(i);
        let mut v: Vec<u8> = r
            .legs
            .iter()
            .filter_map(|l| match l.feed {
                Feed::Usb(k) => Some(k + 1),
                _ => None,
            })
            .collect();
        v.dedup();
        if !v.is_empty() {
            m.insert(strip_name(i), v);
        }
    }
    m
}

fn expect(pairs: &[(&str, &[u8])]) -> BTreeMap<String, Vec<u8>> {
    pairs
        .iter()
        .map(|(k, v)| (k.to_string(), v.to_vec()))
        .collect()
}

fn src(g: SrcGroup, n: u16) -> Option<SourceAddr> {
    SourceAddr::new(g, n)
}

#[test]
fn channel_sources_follow_altsw() {
    let store = public_store();
    let b = RoutingBuilder::new(&store);
    let t = b.build(&store);
    assert!(!t.altsw, "fixture has io.altsw off");

    // ALT off: stage inputs (AES50 A / local) everywhere; only ch25 and aux1 sit on USB 1.
    assert_eq!(
        usb_map(&t),
        expect(&[("ch25", &[1]), ("aux1", &[1])]),
        "altsw off"
    );
    let ch1 = t.channel(1);
    assert_eq!(ch1.main, src(SrcGroup::A, 1));
    assert_eq!(ch1.alt, src(SrcGroup::Usb, 1));
    assert!(!ch1.use_alt);
    assert_eq!(ch1.legs[0].feed, Feed::Silence); // AES50 has no hardware here
    assert!((ch1.legs[0].gain - 10f32.powf(30.5 / 20.0)).abs() < 1e-3); // A.1 g = 30.5 dB
    assert_eq!(t.channel(10).width, Width::Stereo); // A.25/26 ST
    assert_eq!(t.channel(39).source(), src(SrcGroup::Aux, 4));
    assert_eq!(
        t.channel(39).legs.map(|l| l.src),
        [src(SrcGroup::Aux, 3), src(SrcGroup::Aux, 4)]
    );
    let bus1 = |leg| {
        Feed::Tap(
            Tap::Mix {
                grp: MixGroup::Bus,
                leg,
            }
            .id()
            .unwrap(),
        )
    };
    assert_eq!(t.channel(33).legs.map(|l| l.feed), [bus1(1), bus1(2)]);

    // ALT on: every srcauto channel switches to its alt source (REAPER via USB).
    store
        .set(store.lookup("io/altsw").unwrap(), Value::Int(1))
        .unwrap();
    let t = b.build(&store);
    assert!(t.altsw);
    assert_eq!(
        usb_map(&t),
        expect(&[
            ("ch1", &[1]),
            ("ch2", &[2]),
            ("ch3", &[3]),
            ("ch4", &[4]),
            ("ch5", &[5]),
            ("ch6", &[6]),
            ("ch7", &[7]),
            ("ch8", &[9]),
            ("ch9", &[19]),
            ("ch10", &[25, 26]),
            ("ch12", &[20]),
            ("ch13", &[29]),
            ("ch15", &[18]),
            ("ch20", &[33]),
            ("ch21", &[34]),
            ("ch22", &[35]),
            ("ch23", &[37]),
            ("ch25", &[1]),
            ("ch32", &[17]),
            ("aux1", &[1]),
        ]),
        "altsw on"
    );
    assert_eq!(t.channel(10).width, Width::Stereo); // alt USB.26 of the ST pair 25/26
    assert_eq!(t.channel(10).source(), src(SrcGroup::Usb, 26));
    assert_eq!(t.channel(11).source(), src(SrcGroup::B, 20)); // alt on AES50 B: silent
    assert_eq!(t.channel(14).source(), None); // alt OFF
    assert_eq!(t.channel(14).legs, [Leg::SILENT; 2]);
    assert_eq!(t.channel(40).source(), src(SrcGroup::Lcl, 7)); // srcauto off: stays on main
    assert_eq!(t.channel(1).legs[0].gain, 1.0); // USB: no preamp, not muted

    // per-channel ALT overrides the global switch
    store
        .set(store.lookup("io/altsw").unwrap(), Value::Int(0))
        .unwrap();
    store
        .set(store.lookup("ch/2/in/set/altsrc").unwrap(), Value::Int(1))
        .unwrap();
    assert_eq!(usb_map(&b.build(&store)).get("ch2"), Some(&vec![2]));
}

#[test]
fn usb_outputs_from_snapshot() {
    let store = public_store();
    let t = RoutingBuilder::new(&store).build(&store);
    let mut seen = Vec::new();
    for n in 1..=48usize {
        let l = t.usb_out(n);
        let s = l.src.expect("all USB outs patched");
        seen.push(s.to_string());
        match n {
            20 => assert_eq!(l.feed, Feed::Usb(0)), // USB.1 loop-back
            45..=48 => {
                let tap = Tap::Mix {
                    grp: MixGroup::Mtx,
                    leg: n as u16 - 42,
                };
                assert_eq!(l.feed, Feed::Tap(tap.id().unwrap()));
            }
            _ => assert_eq!(l.feed, Feed::Silence, "USB out {n}"),
        }
        assert_eq!(
            *l,
            *t.output(OutAddr::new(OutGroup::Usb, n as u16).unwrap())
        );
    }
    let mut want: Vec<String> = (1..=19).map(|n| format!("A.{n}")).collect();
    want.push("USB.1".into());
    want.extend((21..=32).map(|n| format!("A.{n}")));
    want.extend((1..=8).map(|n| format!("LCL.{n}")));
    want.extend((1..=4).map(|n| format!("AUX.{n}")));
    want.extend((3..=6).map(|n| format!("MTX.{n}")));
    assert_eq!(seen, want);
    // other output groups are resolved too
    let lcl1 = t.output(OutAddr::new(OutGroup::Lcl, 1).unwrap());
    assert_eq!(
        lcl1.feed,
        Feed::Tap(
            Tap::Mix {
                grp: MixGroup::Bus,
                leg: 15
            }
            .id()
            .unwrap()
        )
    );
    let aux1 = t.output(OutAddr::new(OutGroup::Aux, 1).unwrap());
    assert_eq!(aux1.src, src(SrcGroup::Main, 7));
    assert_eq!(
        *t.output(OutAddr::new(OutGroup::C, 1).unwrap()),
        Leg::SILENT
    );
}

#[test]
fn signals_through_graph_and_live_altsw_toggle() {
    let mut store = public_store();
    let mut ctl = RoutingControl::new(&store);
    store.add_hook(ctl.change_hook());
    let store = Arc::new(store);

    // tap a placeholder "matrix 2 L" node so USB out 45 (MTX.3) carries something
    let mut g = GraphBuilder::new(32, 48_000.0, 48);
    let io = add_input_stage(&mut g, &mut ctl).unwrap();
    let mtx = g
        .add_node(
            "mtx-placeholder",
            2,
            &[],
            |_: &wing_engine::ProcessCtx, io: &mut wing_engine::NodeIo<'_>| {
                io.output(0).fill(0.25);
                io.output(1).fill(-0.25);
            },
        )
        .unwrap();
    let mut taps: Box<TapPorts> = Box::new(NO_TAPS);
    for (leg, ch) in [(3u16, 0u32), (4, 1)] {
        let id = Tap::Mix {
            grp: MixGroup::Mtx,
            leg,
        }
        .id()
        .unwrap();
        taps[id.0 as usize] = Some(Port::node(mtx, ch));
    }
    let out = add_usb_output(&mut g, &mut ctl, io.osc, &taps).unwrap();
    let mut e = Engine::new(
        g.build().unwrap(),
        EngineConfig {
            workers: 2,
            rt_workers: false,
            ..EngineConfig::default()
        },
    );
    let run = |e: &mut Engine| {
        for c in 0..48 {
            e.external_mut(c).fill(0.01 * (c + 1) as f32);
        }
        e.process_cycle();
    };
    let strip =
        |e: &Engine, i: usize, side: usize| e.port(InputRouter::port(io.inputs, i, side))[5];
    let usb_out = |e: &Engine, n: u32| e.port(Port::node(out, n - 1))[5];

    run(&mut e);
    assert_eq!(strip(&e, 0, 0), 0.0); // ch1 on A.1: silent
    assert_eq!(strip(&e, 24, 0), 0.01); // ch25 on USB 1
    assert_eq!(strip(&e, 40, 1), 0.01); // aux1 on USB 1 (mono -> both sides)
    assert_eq!(usb_out(&e, 20), 0.01); // USB out 20 <- USB in 1
    assert_eq!(usb_out(&e, 1), 0.0); // <- A.1
    assert_eq!(usb_out(&e, 45), 0.25); // <- MTX.3 (placeholder tap)
    assert_eq!(usb_out(&e, 46), -0.25);
    assert_eq!(usb_out(&e, 47), 0.0); // <- MTX.5 (not wired)

    // flip the global ALT switch through the store: hook -> refresh -> RT adopts the new table
    store
        .set(store.lookup("io/altsw").unwrap(), Value::Int(1))
        .unwrap();
    assert!(ctl.refresh(&store));
    run(&mut e);
    for (ch, usb) in [(1, 1), (8, 9), (9, 19), (23, 37), (32, 17)] {
        assert_eq!(strip(&e, ch - 1, 0), 0.01 * usb as f32, "ch{ch}");
    }
    assert_eq!(strip(&e, 9, 0), 0.25); // ch10 L = USB 25
    assert_eq!(strip(&e, 9, 1), 0.26); // ch10 R = USB 26
    assert_eq!(strip(&e, 13, 0), 0.0); // ch14 alt OFF

    // source settings: USB 1 polarity and mute
    store
        .set(store.lookup("io/in/USB/1/pol").unwrap(), Value::Int(1))
        .unwrap();
    assert!(ctl.refresh(&store));
    run(&mut e);
    assert_eq!(strip(&e, 0, 0), -0.01);
    assert_eq!(usb_out(&e, 20), -0.01);
    store
        .set(store.lookup("io/in/USB/1/mute").unwrap(), Value::Int(1))
        .unwrap();
    assert!(ctl.refresh(&store));
    run(&mut e);
    assert_eq!(strip(&e, 0, 0), 0.0);

    // a non-routing change does not rebuild
    store
        .set(store.lookup("ch/1/fdr").unwrap(), Value::Float(-3.0))
        .unwrap();
    assert!(!ctl.refresh(&store));
    run(&mut e);
    assert!(ctl.settled());
}

#[test]
fn oscillator_levels() {
    let store = ParamStore::new();
    let set = |p: &str, v: &str| store.set_text(store.lookup(p).unwrap(), v, 0).unwrap();
    set("io/in/OSC/1/osc/mode", "SINE");
    set("io/in/OSC/1/osc/lvl", "-20");
    set("io/in/OSC/1/osc/f", "1000");
    set("io/in/OSC/1/mute", "0");
    set("io/in/OSC/2/mute", "0");
    set("io/out/USB/1/grp", "OSC");
    set("io/out/USB/1/in", "1");
    set("io/out/USB/2/grp", "OSC");
    set("io/out/USB/2/in", "2");
    let rms_db = |x: &[f32]| {
        let p = x.iter().map(|v| (*v as f64).powi(2)).sum::<f64>() / x.len() as f64;
        10.0 * p.log10()
    };
    for (mode, tol) in [("PINK", 1.5), ("WHITE", 0.5)] {
        set("io/in/OSC/2/osc/mode", mode);
        set("io/in/OSC/2/osc/lvl", "-12");
        let mut ctl = RoutingControl::new(&store);
        let mut g = GraphBuilder::new(480, 48_000.0, 48);
        let io = add_input_stage(&mut g, &mut ctl).unwrap();
        let out = add_usb_output(&mut g, &mut ctl, io.osc, &NO_TAPS).unwrap();
        let mut e = Engine::new(g.build().unwrap(), EngineConfig::default());
        let (mut a, mut b) = (Vec::new(), Vec::new());
        for _ in 0..200 {
            e.process_cycle_serial();
            a.extend_from_slice(e.port(Port::node(out, 0)));
            b.extend_from_slice(e.port(Port::node(out, 1)));
        }
        assert!((rms_db(&a) + 20.0).abs() < 0.05, "sine {}", rms_db(&a));
        assert!((rms_db(&b) + 12.0).abs() < tol, "{mode} {}", rms_db(&b));
        // 1 kHz sine: 48 samples per period
        assert!((a[0] - a[48]).abs() < 1e-3 && (a[12] - 0.1 * 2f32.sqrt()).abs() < 1e-3);
    }
}
