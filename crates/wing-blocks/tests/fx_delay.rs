//! T05D: delay family behaviour through the model factory: feedback stability at the maximum settings and
//! click-free time changes (crossfade for ST-DL / TAP-DL, rate-limited glide for TAPE-DL / OILCAN / BBD-DL).

use wing_blocks::model::{create_model, BlockKind, Model, ParamValue};

const FS: f64 = 48_000.0;
const BLOCK: usize = 64;

fn model(id: &'static str, params: &[(u16, f32)]) -> Box<dyn Model> {
    let mut m = create_model(BlockKind::Fx, id);
    assert!(!m.is_stub(), "{id}");
    m.prepare(FS, BLOCK);
    for &(s, v) in params {
        m.set_param(s, ParamValue::Float(v));
    }
    m.commit();
    m.reset();
    m
}

fn run(m: &mut dyn Model, l: &mut [f32], r: &mut [f32]) {
    for (a, b) in l.chunks_mut(BLOCK).zip(r.chunks_mut(BLOCK)) {
        m.process(a, b, None);
    }
}

fn rms(x: &[f32]) -> f32 {
    (x.iter().map(|v| v * v).sum::<f32>() / x.len() as f32).sqrt()
}

fn peak(x: &[f32]) -> f32 {
    x.iter().fold(0f32, |a, v| a.max(v.abs()))
}

/// Deterministic white noise in [-a, a).
fn noise(n: usize, a: f32, seed: u32) -> Vec<f32> {
    let mut s = seed;
    (0..n)
        .map(|_| {
            s ^= s << 13;
            s ^= s >> 17;
            s ^= s << 5;
            a * ((s >> 8) as f32 / 8_388_608.0 - 1.0)
        })
        .collect()
}

/// Maximum feedback, shortest time, widest filters / most modulation: the loop stays bounded under a loud
/// input and decays once the input stops.
#[test]
fn feedback_is_stable_at_maximum() {
    let cases: [(&'static str, &[(u16, f32)]); 5] = [
        // time 5 ms, ST, fact 1, 1:1, offs 0, FEED 100, open filters
        (
            "ST-DL",
            &[
                (1, 5.0),
                (6, 100.0),
                (7, 20.0),
                (8, 20_000.0),
                (9, 20.0),
                (10, 20_000.0),
            ],
        ),
        // no feedback path; most taps, max slope / diffusion
        ("TAP-DL", &[(1, 1.0), (2, 16.0), (3, 24.0), (8, 100.0)]),
        // 60 ms, SUSTAIN / DRIVE / FLUTTER 100
        ("TAPE-DL", &[(1, 60.0), (2, 100.0), (3, 100.0), (4, 100.0)]),
        // shortest, SUSTAIN / WOBBLE / TONE 10
        ("OILCAN", &[(1, 0.0), (2, 10.0), (3, 10.0), (4, 10.0)]),
        // 20 ms, FEED 100
        ("BBD-DL", &[(1, 0.0), (2, 100.0)]),
    ];
    for (id, params) in cases {
        let mut m = model(id, params);
        let n = 5 * FS as usize;
        let on = FS as usize / 2;
        let mut l = noise(n, 0.5, 1);
        let mut r = noise(n, 0.5, 2);
        for v in l[on..].iter_mut().chain(r[on..].iter_mut()) {
            *v = 0.0;
        }
        run(&mut *m, &mut l, &mut r);
        assert!(
            l.iter().chain(&r).all(|v| v.is_finite()),
            "{id}: not finite"
        );
        let pk = peak(&l).max(peak(&r));
        assert!(pk < 30.0, "{id}: peak {pk}");
        let after = rms(&l[on..on + 12_000]);
        let tail = rms(&l[n - 12_000..]);
        assert!(
            after > 1e-3,
            "{id}: no echo after the input stopped ({after})"
        );
        assert!(
            tail < 0.05 * after,
            "{id}: does not decay ({after} -> {tail})"
        );
    }
}

/// Changing the delay time on a running sine never produces a step: the largest sample-to-sample change
/// after the edit stays within a small multiple of the steady-state slope (a glide may raise the pitch up to
/// 1.8x; a hard jump of the read position would produce a step of up to twice the amplitude, ~17x).
#[test]
fn time_changes_do_not_click() {
    type Case = (&'static str, &'static [(u16, f32)], (u16, f32));
    let cases: [Case; 9] = [
        ("ST-DL", &[(1, 300.0), (6, 30.0)], (1, 120.0)),
        ("ST-DL", &[(1, 120.0), (6, 30.0), (2, 1.0)], (1, 700.0)),
        ("ST-DL", &[(1, 300.0), (6, 30.0)], (3, 4.0)),
        ("TAP-DL", &[(1, 300.0)], (1, 700.0)),
        ("TAP-DL", &[(1, 300.0)], (2, 9.0)),
        ("TAPE-DL", &[(1, 300.0), (2, 30.0)], (1, 100.0)),
        ("TAPE-DL", &[(1, 100.0), (2, 30.0)], (1, 650.0)),
        ("OILCAN", &[(1, 6.5), (2, 3.0)], (1, 2.0)),
        ("BBD-DL", &[(1, 65.0), (2, 30.0)], (1, 20.0)),
    ];
    for (id, params, (slot, to)) in cases {
        let mut m = model(id, params);
        let pre = FS as usize;
        let post = 3 * FS as usize / 2;
        let x: Vec<f32> = (0..pre + post)
            .map(|i| 0.25 * (std::f32::consts::TAU * 440.0 * i as f32 / FS as f32).sin())
            .collect();
        let (mut l, mut r) = (x[..pre].to_vec(), x[..pre].to_vec());
        run(&mut *m, &mut l, &mut r);
        m.set_param(slot, ParamValue::Float(to));
        let (mut l2, mut r2) = (x[pre..].to_vec(), x[pre..].to_vec());
        run(&mut *m, &mut l2, &mut r2);
        let max_step = |y: &[f32]| y.windows(2).fold(0f32, |a, w| a.max((w[1] - w[0]).abs()));
        let steady = max_step(&l[pre / 2..]);
        let after = max_step(&l2);
        assert!(steady > 1e-3, "{id}: no wet signal");
        assert!(
            after < 2.5 * steady,
            "{id} slot {slot} -> {to}: step {after} vs steady {steady}"
        );
    }
}
