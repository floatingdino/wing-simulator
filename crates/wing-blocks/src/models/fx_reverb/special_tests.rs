//! T05S tests: gate timing, reverse envelope + latency, shimmer pitch content, DEL/REV routing, stability, params.

use super::special::*;
use crate::model::{create_model, BlockKind, Model, ParamValue};

const FS: f64 = 48_000.0;
const IDS: [&str; 4] = ["GATED", "REVERSE", "DEL/REV", "SHIMMER"];

fn specs(id: &str) -> &'static [super::ParamSpec] {
    match id {
        "GATED" => &GATED_PARAMS,
        "REVERSE" => &REVERSE_PARAMS,
        "DEL/REV" => &DELREV_PARAMS,
        _ => &SHIMMER_PARAMS,
    }
}

fn model(id: &'static str) -> Box<dyn Model> {
    let mut m = create_model(BlockKind::Fx, id);
    assert!(!m.is_stub(), "{id}");
    m.prepare(FS, 64);
    m
}

fn set(m: &mut dyn Model, id: &str, key: &str, v: f32) {
    let slot = specs(id).iter().position(|p| p.key == key).unwrap() as u16 + 1;
    m.set_param(slot, ParamValue::Float(v));
}

/// Process `input` (mono, fed to both channels) in 64-sample blocks; returns the left output.
fn run(m: &mut dyn Model, input: &[f32]) -> Vec<f32> {
    let mut out = Vec::with_capacity(input.len());
    for c in input.chunks(64) {
        let mut l = c.to_vec();
        let mut r = c.to_vec();
        m.process(&mut l, &mut r, None);
        assert!(l.iter().chain(r.iter()).all(|x| x.is_finite()));
        out.extend_from_slice(&l);
    }
    out
}

fn noise(n: usize, amp: f32, seed: u32) -> Vec<f32> {
    let mut s = seed;
    (0..n)
        .map(|_| {
            s = s.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            ((s >> 8) as f32 / (1u32 << 24) as f32 - 0.5) * 2.0 * amp
        })
        .collect()
}

fn rms(x: &[f32]) -> f32 {
    (x.iter().map(|v| (*v as f64).powi(2)).sum::<f64>() / x.len().max(1) as f64).sqrt() as f32
}

fn at(sec: f32) -> usize {
    (sec * FS as f32) as usize
}

fn goertzel(x: &[f32], hz: f32) -> f32 {
    let w = std::f64::consts::TAU * hz as f64 / FS;
    let (mut re, mut im) = (0.0f64, 0.0f64);
    for (i, v) in x.iter().enumerate() {
        re += *v as f64 * (w * i as f64).cos();
        im += *v as f64 * (w * i as f64).sin();
    }
    ((re * re + im * im).sqrt() / x.len() as f64) as f32
}

/// RMS-summed Goertzel magnitude over `hz +- 40` in 1 Hz steps: a grain shifter spreads a partial into
/// +-(fs/W) sidebands (no coherent carrier), so partials are measured per band.
fn band(x: &[f32], hz: f32) -> f32 {
    (-40..=40)
        .map(|k| goertzel(x, hz + k as f32).powi(2))
        .sum::<f32>()
        .sqrt()
}

fn db(x: f32) -> f32 {
    20.0 * x.max(1e-12).log10()
}

#[test]
fn param_tables_match_models_md() {
    let keys = |id| specs(id).iter().map(|p| p.key).collect::<Vec<_>>();
    assert_eq!(
        keys("GATED"),
        ["pdel", "att", "dcy", "dens", "diff", "sprd", "lc", "hsf", "hsg"]
    );
    assert_eq!(
        keys("REVERSE"),
        ["pdel", "rise", "dcy", "diff", "sprd", "lc", "hsf", "hsg"]
    );
    assert_eq!(
        keys("DEL/REV"),
        ["time", "feed", "fhc", "dly", "d2r", "pdel", "size", "dcy", "damp", "rlc", "i2r"]
    );
    assert_eq!(
        keys("SHIMMER"),
        ["pdel", "size", "dcy", "lc", "hc", "damp", "shim", "shine"]
    );
    for id in IDS {
        let mut m = model(id);
        for (i, p) in specs(id).iter().enumerate() {
            let slot = i as u16 + 1;
            assert_eq!(m.param(slot), Some(p.default), "{id} {}", p.key);
            m.set_param(slot, ParamValue::Float(1e9));
            assert_eq!(m.param(slot), Some(p.max));
            m.set_param(slot, ParamValue::Float(-1e9));
            assert_eq!(m.param(slot), Some(p.min));
            m.set_param(slot, ParamValue::Float(f32::NAN));
            assert_eq!(m.param(slot), Some(p.min));
        }
        assert_eq!(m.param(specs(id).len() as u16 + 1), None);
        m.load_defaults();
        assert_eq!(m.param(1), Some(specs(id)[0].default));
    }
}

#[test]
fn gated_gate_timing() {
    for dcy in [0.14f32, 0.3, 0.8] {
        let mut m = model("GATED");
        set(&mut *m, "GATED", "pdel", 0.0);
        set(&mut *m, "GATED", "dcy", dcy);
        set(&mut *m, "GATED", "att", 0.0);
        set(&mut *m, "GATED", "hsg", 0.0);
        set(&mut *m, "GATED", "lc", 20.0);
        let mut x = vec![0.0f32; at(2.0)];
        let t0 = at(0.1);
        let burst = noise(at(0.02), 0.5, 7);
        x[t0..t0 + burst.len()].copy_from_slice(&burst);
        let y = run(&mut *m, &x);
        let open = rms(&y[t0 + at(0.03)..t0 + at(dcy * 0.9)]);
        assert!(open > 0.005, "dcy {dcy}: gate open level {open}");
        // closed: exactly after hold (+4 ms release) up to the end
        let closed = rms(&y[t0 + at(dcy) + at(0.01)..t0 + at(dcy) + at(0.4)]);
        assert!(
            closed < open * 1e-3,
            "dcy {dcy}: closed {closed} vs open {open}"
        );
        // the gate edge sits at t0 + dcy (within the 4 ms release): last audible sample
        let thr = open * 0.05;
        let last = y.iter().rposition(|v| v.abs() > thr).unwrap();
        let want = t0 + at(dcy);
        assert!(
            last >= want.saturating_sub(at(0.003)) && last <= want + at(0.006),
            "dcy {dcy}: gate closed at {last}, want ~{want}"
        );
        // retrigger after the gate closed: a second burst opens it again
        let mut m2 = model("GATED");
        set(&mut *m2, "GATED", "pdel", 0.0);
        set(&mut *m2, "GATED", "dcy", dcy);
        let mut x2 = vec![0.0f32; at(2.0)];
        x2[t0..t0 + burst.len()].copy_from_slice(&burst);
        let t1 = at(1.2);
        x2[t1..t1 + burst.len()].copy_from_slice(&burst);
        let y2 = run(&mut *m2, &x2);
        assert!(rms(&y2[t1 + at(0.03)..t1 + at(dcy * 0.9)]) > 0.005);
    }
}

#[test]
fn gated_attack_fades_in() {
    let mut fast = model("GATED");
    let mut slow = model("GATED");
    for (m, att) in [(&mut fast, 0.0), (&mut slow, 30.0)] {
        set(&mut **m, "GATED", "pdel", 0.0);
        set(&mut **m, "GATED", "att", att);
    }
    let mut x = vec![0.0f32; at(0.6)];
    let b = noise(at(0.3), 0.5, 3);
    x[100..100 + b.len()].copy_from_slice(&b);
    let a = rms(&run(&mut *fast, &x)[100..100 + at(0.012)]);
    let s = rms(&run(&mut *slow, &x)[100..100 + at(0.012)]);
    assert!(
        s < a,
        "slow attack {s} should be quieter than fast {a} in the first 12 ms"
    );
}

#[test]
fn reverse_envelope_rises_and_has_window_latency() {
    for dcy in [0.3f32, 0.6, 1.0] {
        let mut m = model("REVERSE");
        set(&mut *m, "REVERSE", "pdel", 0.0);
        set(&mut *m, "REVERSE", "dcy", dcy);
        set(&mut *m, "REVERSE", "rise", 0.0);
        set(&mut *m, "REVERSE", "hsg", 0.0);
        set(&mut *m, "REVERSE", "lc", 20.0);
        let t0 = at(0.05);
        let w = at(dcy);
        let mut x = vec![0.0f32; t0 + 2 * w + at(0.5)];
        let b = noise(at(0.01), 0.7, 11);
        x[t0..t0 + b.len()].copy_from_slice(&b);
        let y = run(&mut *m, &x);
        // causal latency: nothing before the window has been captured (t0 + W, minus one pre-delay sample)
        assert!(
            y[..t0 + w - 4].iter().all(|v| v.abs() < 1e-9),
            "dcy {dcy}: output before the window ended"
        );
        // the swell: energy grows through the playback, strongly
        let p0 = t0 + w;
        let q = w / 4;
        let e: Vec<f32> = (0..4)
            .map(|k| rms(&y[p0 + k * q..p0 + (k + 1) * q]))
            .collect();
        assert!(
            e[0] < e[1] && e[1] < e[2] && e[2] < e[3],
            "dcy {dcy}: quarter RMS must rise monotonically: {e:?}"
        );
        assert!(db(e[3]) - db(e[0]) > 12.0, "dcy {dcy}: swell depth {e:?}");
        // abrupt cut at the end of the window, silence after
        assert!(
            rms(&y[p0 + w + at(0.02)..]) < e[3] * 1e-3,
            "dcy {dcy}: tail after cut"
        );
    }
}

#[test]
fn reverse_rise_param_slows_the_fade_in() {
    let run_rise = |rise: f32| {
        let mut m = model("REVERSE");
        set(&mut *m, "REVERSE", "pdel", 0.0);
        set(&mut *m, "REVERSE", "dcy", 0.6);
        set(&mut *m, "REVERSE", "rise", rise);
        let mut x = vec![0.0f32; at(1.5)];
        let b = noise(at(0.01), 0.7, 5);
        x[100..100 + b.len()].copy_from_slice(&b);
        let y = run(&mut *m, &x);
        rms(&y[100 + at(0.6)..100 + at(0.6) + at(0.05)])
    };
    assert!(run_rise(50.0) < run_rise(0.0));
}

#[test]
fn shimmer_has_octave_partials() {
    let n = at(4.0);
    let sine: Vec<f32> = (0..n)
        .map(|i| 0.3 * (std::f32::consts::TAU * 440.0 * i as f32 / FS as f32).sin())
        .collect();
    let measure = |shim: f32, shine: f32, hz: f32| {
        let mut m = model("SHIMMER");
        set(&mut *m, "SHIMMER", "pdel", 0.0);
        set(&mut *m, "SHIMMER", "shim", shim);
        set(&mut *m, "SHIMMER", "shine", shine);
        set(&mut *m, "SHIMMER", "dcy", 3.0);
        set(&mut *m, "SHIMMER", "hc", 7000.0);
        let y = run(&mut *m, &sine);
        let tail = &y[at(3.0)..];
        db(band(tail, hz)) - db(band(tail, 440.0))
    };
    let off = measure(0.0, 0.0, 880.0);
    let on = measure(100.0, 0.0, 880.0);
    println!("880 Hz rel 440: shim off {off:.1} dB, on {on:.1} dB");
    assert!(
        on > off + 15.0,
        "octave partial must appear: off {off} on {on}"
    );
    assert!(on > -30.0, "octave partial level {on} dB re fundamental");
    // the fifth appears with SHINE
    let f_off = measure(100.0, 0.0, 659.3);
    let f_on = measure(100.0, 100.0, 659.3);
    println!("659 Hz rel 440: shine 0 {f_off:.1} dB, shine 100 {f_on:.1} dB");
    assert!(f_on > f_off + 10.0, "+7 partial: {f_off} -> {f_on}");
    assert!(f_on > -30.0);
}

#[test]
fn shimmer_is_stable_and_decays() {
    let mut m = model("SHIMMER");
    set(&mut *m, "SHIMMER", "shim", 100.0);
    set(&mut *m, "SHIMMER", "shine", 100.0);
    set(&mut *m, "SHIMMER", "dcy", 4.0);
    set(&mut *m, "SHIMMER", "pdel", 0.0);
    let mut x = vec![0.0f32; at(24.0)];
    let b = noise(at(1.0), 0.8, 21);
    x[..b.len()].copy_from_slice(&b);
    let y = run(&mut *m, &x);
    let peak = y.iter().fold(0.0f32, |a, v| a.max(v.abs()));
    assert!(peak < 10.0, "peak {peak}");
    let early = rms(&y[at(1.5)..at(3.5)]);
    let late = rms(&y[at(20.0)..at(24.0)]);
    println!("shimmer decay: early {early:.4} late {late:.6} peak {peak:.2}");
    assert!(late < early * 0.03, "must decay: {early} -> {late}");
}

#[test]
fn all_models_stable_at_extremes() {
    for id in IDS {
        for extreme in [0usize, 1] {
            let mut m = model(id);
            for (i, p) in specs(id).iter().enumerate() {
                let v = if (i + extreme) % 2 == 0 { p.max } else { p.min };
                m.set_param(i as u16 + 1, ParamValue::Float(v));
            }
            let x = noise(at(6.0), 1.0, 99 + extreme as u32);
            let y = run(&mut *m, &x);
            let peak = y.iter().fold(0.0f32, |a, v| a.max(v.abs()));
            assert!(peak < 100.0, "{id}/{extreme}: peak {peak}");
            // silence flushes: afterwards a long silent run decays (finite, bounded)
            let z = run(&mut *m, &vec![0.0; at(2.0)]);
            assert!(z.iter().all(|v| v.is_finite() && v.abs() < 100.0));
        }
    }
}

#[test]
fn silence_in_silence_out() {
    for id in IDS {
        let mut m = model(id);
        let y = run(&mut *m, &vec![0.0; at(1.0)]);
        assert!(y.iter().all(|&v| v == 0.0), "{id}");
    }
}

#[test]
fn delrev_routing() {
    let imp = |n: usize| {
        let mut x = vec![0.0f32; n];
        x[0] = 1.0;
        x
    };
    let mk = |dly: f32, d2r: f32, i2r: f32, feed: f32| {
        let mut m = model("DEL/REV");
        set(&mut *m, "DEL/REV", "time", 100.0);
        set(&mut *m, "DEL/REV", "feed", feed);
        set(&mut *m, "DEL/REV", "fhc", 20_000.0);
        set(&mut *m, "DEL/REV", "dly", dly);
        set(&mut *m, "DEL/REV", "d2r", d2r);
        set(&mut *m, "DEL/REV", "i2r", i2r);
        set(&mut *m, "DEL/REV", "pdel", 0.0);
        m.commit();
        m.reset(); // snap the glided delay time
        m
    };
    let n = at(1.0);
    let t = at(0.1);
    // delay only: clean echo at 100 ms, nothing else, second echo at 200 ms scaled by FEED
    let y = run(&mut *mk(100.0, 0.0, 0.0, 50.0), &imp(n));
    let pk = y
        .iter()
        .enumerate()
        .max_by(|a, b| a.1.abs().partial_cmp(&b.1.abs()).unwrap())
        .unwrap()
        .0;
    assert!((pk as i64 - t as i64).abs() <= 2, "echo at {pk}, want {t}");
    assert!(y[pk] > 0.9);
    assert!(y[..pk - 3].iter().all(|v| v.abs() < 1e-6));
    let e2 = y[2 * t - 3..2 * t + 4]
        .iter()
        .fold(0.0f32, |a, v| a.max(v.abs()));
    assert!((0.4..0.6).contains(&e2), "2nd echo {e2}");
    // reverb only via IN->REV: dense energy right after the input, no discrete echo
    let y = run(&mut *mk(0.0, 0.0, 100.0, 0.0), &imp(n));
    assert!(rms(&y[at(0.05)..at(0.5)]) > 1e-4);
    // delay -> reverb: reverb silent until the first echo arrives
    let y = run(&mut *mk(0.0, 100.0, 0.0, 0.0), &imp(n));
    assert!(y[..t - 4].iter().all(|v| v.abs() < 1e-9));
    assert!(rms(&y[t + at(0.05)..t + at(0.4)]) > 1e-4);
    // all sends off: silent
    let y = run(&mut *mk(0.0, 0.0, 0.0, 40.0), &imp(n));
    assert!(y.iter().all(|&v| v == 0.0));
}

#[test]
fn delrev_time_sweep_is_click_free() {
    let mut m = model("DEL/REV");
    set(&mut *m, "DEL/REV", "time", 300.0);
    let sine: Vec<f32> = (0..at(3.0))
        .map(|i| 0.3 * (std::f32::consts::TAU * 220.0 * i as f32 / FS as f32).sin())
        .collect();
    let mut out = Vec::new();
    for (k, c) in sine.chunks(64).enumerate() {
        if k % 40 == 0 {
            set(
                &mut *m,
                "DEL/REV",
                "time",
                300.0 + (k as f32 * 7.0) % 1500.0,
            );
        }
        let mut l = c.to_vec();
        let mut r = c.to_vec();
        m.process(&mut l, &mut r, None);
        out.extend_from_slice(&l);
    }
    let max_step = out
        .windows(2)
        .fold(0.0f32, |a, w| a.max((w[1] - w[0]).abs()));
    assert!(max_step < 0.6, "step {max_step}");
}

#[test]
fn block_size_independent_chunking() {
    // blocks larger than max_block are chunked internally; result must equal 64-sample processing
    for id in IDS {
        let x = noise(at(0.5), 0.5, 4);
        let mut a = model(id);
        let ya = run(&mut *a, &x);
        let mut b = model(id);
        let (mut l, mut r) = (x.clone(), x.clone());
        b.process(&mut l, &mut r, None);
        let diff = ya
            .iter()
            .zip(&l)
            .fold(0.0f32, |m, (p, q)| m.max((p - q).abs()));
        assert!(diff < 2e-3, "{id}: {diff}");
    }
}
