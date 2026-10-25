//! Criterion benches for wing-dsp. Every DSP block adds a bench here.

use criterion::{black_box, criterion_group, criterion_main, Criterion, Throughput};
use wing_dsp::biquad::{CutKind, SmoothedBiquad};
use wing_dsp::detector::{DetectMode, ReleaseMode};
use wing_dsp::svf::SvfMode;
use wing_dsp::*;

const FS: f64 = 48_000.0;
const BLOCK: usize = 64;

fn input(n: usize) -> Vec<f32> {
    (0..n).map(|i| (i as f32 * 0.031).sin() * 0.5).collect()
}

fn bench_biquad(c: &mut Criterion) {
    let mut g = c.benchmark_group("biquad");
    g.throughput(Throughput::Elements(BLOCK as u64));
    let mut buf = input(BLOCK);
    let mut bq = Biquad::new(Coeffs::peak(FS, 1000.0, 1.0, 6.0));
    g.bench_function("f32_tdf2_64", |b| {
        b.iter(|| bq.process_block(black_box(&mut buf)))
    });
    let mut bq64 = BiquadF64::new(Coeffs::peak(FS, 1000.0, 1.0, 6.0));
    g.bench_function("f64_tdf2_64", |b| {
        b.iter(|| bq64.process_block(black_box(&mut buf)))
    });
    let mut sb = SmoothedBiquad::new(Coeffs::peak(FS, 1000.0, 1.0, 6.0), 1 << 30);
    sb.set_target(Coeffs::peak(FS, 2000.0, 2.0, -6.0));
    g.bench_function("smoothed_ramping_64", |b| {
        b.iter(|| sb.process_block(black_box(&mut buf)))
    });
    let mut cas = Cascade::butterworth(CutKind::Highpass, FS, 80.0, 8);
    g.bench_function("cascade_bw8_64", |b| {
        b.iter(|| cas.process_block(black_box(&mut buf)))
    });
    let mut svf = Svf::new(SvfMode::Bell, FS, 1000.0, 1.0, 6.0);
    g.bench_function("svf_64", |b| {
        b.iter(|| svf.process_block(black_box(&mut buf)))
    });
    g.bench_function("svf_modulated_64", |b| {
        let mut f = 500.0;
        b.iter(|| {
            for x in buf.iter_mut() {
                f = if f > 5000.0 { 500.0 } else { f * 1.0001 };
                svf.set(SvfMode::Lowpass, FS, f, 2.0, 0.0);
                *x = svf.process(*x);
            }
        })
    });
    g.finish();
}

fn bench_bank(c: &mut Criterion) {
    let mut g = c.benchmark_group("bank_48ch");
    for stages in [1usize, 4] {
        g.throughput(Throughput::Elements((48 * BLOCK * stages) as u64));
        let mut bank = BiquadBank::new(48, stages);
        for ch in 0..48 {
            for s in 0..stages {
                bank.set_coeffs(ch, s, Coeffs::peak(FS, 100.0 * (ch + 1) as f64, 1.0, 3.0));
            }
        }
        let mut planar: Vec<Vec<f32>> = (0..48).map(|_| input(BLOCK)).collect();
        g.bench_function(format!("soa_planar_{stages}stage_64"), |b| {
            b.iter(|| {
                let mut it = planar.iter_mut();
                let mut refs: [&mut [f32]; 48] =
                    std::array::from_fn(|_| it.next().unwrap().as_mut_slice());
                bank.process_planar(black_box(&mut refs));
            })
        });
        let mut inter = input(48 * BLOCK);
        g.bench_function(format!("soa_interleaved_{stages}stage_64"), |b| {
            b.iter(|| bank.process_interleaved(black_box(&mut inter)))
        });
        let mut scalar: Vec<Vec<Biquad>> = (0..48)
            .map(|ch| {
                (0..stages)
                    .map(|_| Biquad::new(Coeffs::peak(FS, 100.0 * (ch + 1) as f64, 1.0, 3.0)))
                    .collect()
            })
            .collect();
        g.bench_function(format!("scalar_{stages}stage_64"), |b| {
            b.iter(|| {
                for (ch, buf) in planar.iter_mut().enumerate() {
                    for s in &mut scalar[ch] {
                        s.process_block(black_box(buf));
                    }
                }
            })
        });
    }
    g.finish();
}

fn bench_oversample(c: &mut Criterion) {
    let mut g = c.benchmark_group("oversample");
    g.throughput(Throughput::Elements(BLOCK as u64));
    let mut buf = input(BLOCK);
    let mut o2 = Oversampler2x::new(BLOCK);
    g.bench_function("2x_roundtrip_64", |b| {
        b.iter(|| o2.process(black_box(&mut buf), |_| {}))
    });
    let mut o4 = Oversampler4x::new(BLOCK);
    g.bench_function("4x_roundtrip_64", |b| {
        b.iter(|| o4.process(black_box(&mut buf), |_| {}))
    });
    g.finish();
}

fn bench_misc(c: &mut Criterion) {
    let mut g = c.benchmark_group("misc");
    g.throughput(Throughput::Elements(BLOCK as u64));
    let buf = input(BLOCK);
    let mut out = vec![0.0; BLOCK];
    let mut env = EnvelopeFollower::new(
        DetectMode::Peak,
        0.001,
        0.1,
        ReleaseMode::ProgramDependent {
            charge_tau_s: 0.3,
            slow_tau_s: 1.0,
        },
        FS,
    );
    g.bench_function("envelope_pd_64", |b| {
        b.iter(|| env.process_block(black_box(&buf), &mut out))
    });
    let mut rms = RmsWindow::new(4800, 2400);
    g.bench_function("rms_window_64", |b| {
        b.iter(|| rms.process_block(black_box(&buf)))
    });
    let mut dl = DelayLine::new(48_000);
    g.bench_function("delay_lagrange3_64", |b| {
        b.iter(|| {
            for (o, &x) in out.iter_mut().zip(&buf) {
                dl.push(x);
                *o = dl.read_lagrange3(black_box(1234.56));
            }
        })
    });
    let mut sm = OnePoleSmoother::new(0.0, 0.01, FS);
    sm.set_target(1.0);
    g.bench_function("smoother_fill_64", |b| {
        b.iter(|| sm.fill(black_box(&mut out)))
    });
    g.bench_function("db_to_lin_fast_64", |b| {
        b.iter(|| {
            for (o, &x) in out.iter_mut().zip(&buf) {
                *o = gain::db_to_lin_fast(black_box(x) * 40.0);
            }
        })
    });
    g.finish();
}

criterion_group!(
    benches,
    bench_biquad,
    bench_bank,
    bench_oversample,
    bench_misc
);
criterion_main!(benches);
