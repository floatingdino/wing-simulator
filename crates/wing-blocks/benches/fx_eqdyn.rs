//! Criterion bench for the T05Q FX-rack EQ/dyn models (stereo, 64-sample blocks, 48 kHz), each in a
//! non-trivial setting.

use criterion::{black_box, criterion_group, criterion_main, Criterion, Throughput};
use wing_blocks::model::{create_model, BlockKind, Model, ParamValue};

const FS: f64 = 48_000.0;
const BLOCK: usize = 64;

fn setup(id: &'static str) -> Box<dyn Model> {
    let mut m = create_model(BlockKind::Fx, id);
    m.prepare(FS, BLOCK);
    let set = |m: &mut Box<dyn Model>, s: u16, v: ParamValue| m.set_param(s, v);
    match id {
        "GEQ" => {
            for s in 2..=32u16 {
                set(&mut m, s, ParamValue::Float(((s % 7) as f32 - 3.0) * 2.0));
            }
        }
        "PIA" | "SOUL" | "E88" | "F110" => {
            for s in 2..=8u16 {
                set(&mut m, s, ParamValue::Float(2.0));
            }
        }
        "LIMITER" => {
            set(&mut m, 1, ParamValue::Float(12.0));
            set(&mut m, 6, ParamValue::Float(0.5));
        }
        "DE-S2" => {
            set(&mut m, 1, ParamValue::Float(30.0));
            set(&mut m, 2, ParamValue::Float(30.0));
        }
        "DEQ3" => {
            for b in 0..3u16 {
                set(&mut m, 9 * b + 6, ParamValue::Float(-6.0));
            }
        }
        "C5-CMB" => set(&mut m, 1, ParamValue::Float(-20.0)),
        "SPKMAN" => {
            set(&mut m, 2, ParamValue::Enum(10));
            set(&mut m, 4, ParamValue::Enum(10));
            set(&mut m, 6, ParamValue::Float(3.0));
            set(&mut m, 7, ParamValue::Float(90.0));
            set(&mut m, 9, ParamValue::Float(2.0));
            set(&mut m, 11, ParamValue::Int(1));
            set(&mut m, 17, ParamValue::Float(-6.0));
            set(&mut m, 21, ParamValue::Int(1));
            set(&mut m, 22, ParamValue::Float(-12.0));
        }
        _ => {}
    }
    m.commit();
    m.reset();
    m
}

fn bench(c: &mut Criterion) {
    let mut g = c.benchmark_group("fx_eqdyn");
    g.throughput(Throughput::Elements(BLOCK as u64));
    for id in [
        "GEQ", "PIA", "SOUL", "PULSAR", "MACH4", "LIMITER", "DE-S2", "DEQ3", "C5-CMB", "SPKMAN",
    ] {
        let mut m = setup(id);
        let mut l: Vec<f32> = (0..BLOCK).map(|i| (i as f32 * 0.07).sin() * 0.5).collect();
        let mut r: Vec<f32> = (0..BLOCK).map(|i| (i as f32 * 0.05).cos() * 0.5).collect();
        g.bench_function(format!("{}_stereo_64", id.to_lowercase()), |b| {
            b.iter(|| m.process(black_box(&mut l), black_box(&mut r), None))
        });
    }
    g.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
