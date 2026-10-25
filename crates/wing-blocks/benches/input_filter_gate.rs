//! Criterion benches (64-sample stereo blocks @ 48 kHz) for the input stage, filter block and GATE/DUCK.

use criterion::{black_box, criterion_group, criterion_main, Criterion, Throughput};
use wing_blocks::filter::{FilterBlock, FilterParams, HcSlope, LcSlope};
use wing_blocks::gate::{Gate, GateParams, GateRatio, ScFilter, ScSource, SidechainParams};
use wing_blocks::input::{DelayMode, InputParams, InputStage, SourceMode};

const FS: f64 = 48_000.0;
const BLOCK: usize = 64;

fn input(n: usize) -> Vec<f32> {
    (0..n).map(|i| (i as f32 * 0.031).sin() * 0.5).collect()
}

fn bench_blocks(c: &mut Criterion) {
    let mut g = c.benchmark_group("blocks");
    g.throughput(Throughput::Elements(BLOCK as u64));
    let mut l = input(BLOCK);
    let mut r = input(BLOCK);

    let mut inp = InputStage::new();
    inp.set_params(&InputParams {
        trim_db: 3.0,
        balance_db: 1.0,
        invert: true,
        delay_on: true,
        delay: 12.5,
        delay_mode: DelayMode::Ms,
        source_mode: SourceMode::Stereo,
        ..Default::default()
    });
    inp.prepare(FS, BLOCK);
    g.bench_function("input_stage_delay_64", |b| {
        b.iter(|| inp.process(black_box(&mut l), black_box(&mut r)))
    });

    let mut flt = FilterBlock::new();
    flt.set_params(&FilterParams {
        lc_on: true,
        lc_freq: 80.0,
        lc_slope: LcSlope::Db24,
        hc_on: true,
        hc_freq: 12_000.0,
        hc_slope: HcSlope::Db12,
    });
    flt.prepare(FS, BLOCK);
    g.bench_function("filter_lc24_hc12_64", |b| {
        b.iter(|| flt.process(black_box(&mut l), black_box(&mut r)))
    });
    let mut k = 0u32;
    g.bench_function("filter_lc24_sweeping_64", |b| {
        b.iter(|| {
            k = k.wrapping_add(1);
            flt.set_params(&FilterParams {
                lc_on: true,
                lc_freq: if k.is_multiple_of(2) { 40.0 } else { 400.0 },
                ..Default::default()
            });
            flt.process(black_box(&mut l), black_box(&mut r))
        })
    });

    let mut gate = Gate::new();
    gate.set_params(&GateParams {
        ratio: GateRatio::R2,
        ..GateParams::gate_default()
    });
    gate.set_sidechain(&SidechainParams {
        filter: ScFilter::Hp12,
        freq: 200.0,
        ..Default::default()
    });
    gate.prepare(FS, BLOCK);
    g.bench_function("gate_expander_scfilter_64", |b| {
        b.iter(|| gate.process(black_box(&mut l), black_box(&mut r), None))
    });

    let mut duck = Gate::new();
    duck.set_params(&GateParams::duck_default());
    duck.set_sidechain(&SidechainParams {
        source: ScSource::Channel(1),
        ..Default::default()
    });
    duck.prepare(FS, BLOCK);
    let key = input(BLOCK);
    g.bench_function("ducker_external_key_64", |b| {
        b.iter(|| duck.process(black_box(&mut l), black_box(&mut r), Some((&key, &key))))
    });
    g.finish();
}

criterion_group!(benches, bench_blocks);
criterion_main!(benches);
