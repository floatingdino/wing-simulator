//! Criterion bench for the delay family (T050 / T05D): ST-DL (ST and X), TAP-DL (16 taps, diffusion),
//! TAPE-DL (drive + flutter), OILCAN (wobble) and BBD-DL (compander, S&H); stereo 64-sample blocks at
//! 48 kHz with feedback on. Each iteration also changes the delay time every 32 blocks so the crossfade /
//! glide paths are part of the measurement.

use criterion::{black_box, criterion_group, criterion_main, Criterion, Throughput};
use wing_blocks::model::{create_model, ParamValue};
use wing_blocks::model::{BlockKind, Model};

const FS: f64 = 48_000.0;
const BLOCK: usize = 64;

fn bench(c: &mut Criterion) {
    let mut g = c.benchmark_group("fx_delay");
    g.throughput(Throughput::Elements(BLOCK as u64));
    type Case = (
        &'static str,
        &'static str,
        &'static [(u16, f32)],
        (u16, f32, f32),
    );
    let cases: [Case; 6] = [
        ("st_dl_st", "ST-DL", &[(6, 60.0)], (1, 300.0, 280.0)),
        (
            "st_dl_x",
            "ST-DL",
            &[(2, 1.0), (6, 60.0)],
            (1, 300.0, 280.0),
        ),
        (
            "tap_dl_16_diff",
            "TAP-DL",
            &[(2, 16.0), (8, 50.0)],
            (1, 300.0, 280.0),
        ),
        (
            "tape_dl",
            "TAPE-DL",
            &[(2, 60.0), (3, 50.0), (4, 50.0)],
            (1, 300.0, 280.0),
        ),
        ("oilcan", "OILCAN", &[(2, 6.0), (3, 5.0)], (1, 6.5, 6.0)),
        ("bbd_dl", "BBD-DL", &[(2, 60.0)], (1, 65.0, 60.0)),
    ];
    for (label, id, params, (ts, t0, t1)) in cases {
        let mut m: Box<dyn Model> = create_model(BlockKind::Fx, id);
        m.prepare(FS, BLOCK);
        for &(s, v) in params {
            m.set_param(s, ParamValue::Float(v));
        }
        m.commit();
        let mut l: Vec<f32> = (0..BLOCK).map(|i| (i as f32 * 0.03).sin() * 0.5).collect();
        let mut r = l.clone();
        let mut k = 0u32;
        g.bench_function(format!("{label}_stereo_64"), |b| {
            b.iter(|| {
                k = k.wrapping_add(1);
                if k.is_multiple_of(32) {
                    let t = if (k / 32).is_multiple_of(2) { t0 } else { t1 };
                    m.set_param(ts, ParamValue::Float(t));
                }
                m.process(black_box(&mut l), black_box(&mut r), None)
            })
        });
    }
    g.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
