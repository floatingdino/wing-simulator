//! Criterion bench for the T05T pitch FX models (stereo, 64-sample blocks, 48 kHz, voiced harmonic input so
//! the splice search / pitch detector run; PITCH at +7 semitones, PCORR at full amount and speed).

use criterion::{black_box, criterion_group, criterion_main, Criterion, Throughput};
use wing_blocks::model::{create_model, BlockKind, ParamValue};
use wing_blocks::models::fx_pitch::{pcorr_slot, pitch_slot};

const FS: f64 = 48_000.0;
const BLOCK: usize = 64;

fn bench(c: &mut Criterion) {
    let mut g = c.benchmark_group("fx_pitch");
    g.throughput(Throughput::Elements(BLOCK as u64));
    // 2 s of a 196 Hz harmonic tone, played in blocks
    let src: Vec<f32> = (0..96_000)
        .map(|i| {
            let t = core::f64::consts::TAU * 196.0 * i as f64 / FS;
            (0.3 * (t.sin() + 0.5 * (2.0 * t).sin() + 0.3 * (3.0 * t).sin())) as f32
        })
        .collect();
    for id in ["PITCH", "D-PITCH", "DOUBLE", "PCORR", "SUB"] {
        let mut m = create_model(BlockKind::Fx, id);
        m.prepare(FS, BLOCK);
        m.load_defaults();
        match id {
            "PITCH" => m.set_param(pitch_slot::SEMI, ParamValue::Int(7)),
            "PCORR" => {
                m.set_param(pcorr_slot::SPD, ParamValue::Float(100.0));
                m.set_param(pcorr_slot::AMNT, ParamValue::Float(100.0));
            }
            _ => {}
        }
        m.commit();
        let mut pos = 0;
        let mut l = [0.0f32; BLOCK];
        let mut r = [0.0f32; BLOCK];
        g.bench_function(format!("{}_stereo_64", id.to_lowercase()), |b| {
            b.iter(|| {
                l.copy_from_slice(&src[pos..pos + BLOCK]);
                r.copy_from_slice(&src[pos..pos + BLOCK]);
                pos = (pos + BLOCK) % (src.len() - BLOCK);
                m.process(black_box(&mut l), black_box(&mut r), None)
            })
        });
    }
    g.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
