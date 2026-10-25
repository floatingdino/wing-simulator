//! Criterion bench for the T05R reverb family (HALL, ROOM, CHAMBER, CONCERT, AMBI): one stereo 64-sample block
//! at 48 kHz with WING-Edit defaults (echoes/reflections/chorus on where the defaults enable them), noise input.
//! Target: < 15 us per block per instance.

use criterion::{black_box, criterion_group, criterion_main, Criterion, Throughput};
use wing_blocks::model::BlockKind;

const FS: f64 = 48_000.0;
const BLOCK: usize = 64;

fn bench(c: &mut Criterion) {
    let mut g = c.benchmark_group("fx_reverb");
    g.throughput(Throughput::Elements(BLOCK as u64));
    for id in ["HALL", "ROOM", "CHAMBER", "CONCERT", "AMBI"] {
        let mut m = wing_blocks::model::create_model(BlockKind::Fx, id);
        m.prepare(FS, BLOCK);
        let mut s = 1u32;
        let src: Vec<f32> = (0..BLOCK)
            .map(|_| {
                s = s.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                (s >> 8) as f32 / (1u32 << 24) as f32 - 0.5
            })
            .collect();
        let (mut l, mut r) = (src.clone(), src.clone());
        // warm up the tail so the bench runs on a dense state
        for _ in 0..2000 {
            l.copy_from_slice(&src);
            r.copy_from_slice(&src);
            m.process(&mut l, &mut r, None);
        }
        g.bench_function(format!("{}_stereo_64", id.to_lowercase()), |b| {
            b.iter(|| {
                l.copy_from_slice(&src);
                r.copy_from_slice(&src);
                m.process(black_box(&mut l), black_box(&mut r), None)
            })
        });
    }
    g.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
