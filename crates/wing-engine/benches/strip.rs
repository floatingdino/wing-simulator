//! T035: 40 channel + 8 aux strips with every block engaged (trim, LC 24 dB + HC, filter tool slot, GATE,
//! PRE EQ, STD EQ with all bands active, COMP, inserts placeholders), 64 samples, single thread.
//! Also prints the mean time per cycle and its share of the 1.333 ms cycle budget (48 kHz / 64).

use std::hint::black_box;
use std::time::Instant;

use criterion::{criterion_group, Criterion};
use wing_engine::channel::{block, ChannelStrip};
use wing_engine::params::{ParamMsg, TARGET_BASE};

const N: usize = 64;
const FS: f64 = 48_000.0;

fn set(s: &mut ChannelStrip, block: u8, param: u16, value: f32) {
    s.apply(&ParamMsg::Set {
        block,
        param,
        value,
    });
}

fn full_strip(aux: bool) -> ChannelStrip {
    let mut s = if aux {
        ChannelStrip::aux(FS, N)
    } else {
        ChannelStrip::channel(FS, N)
    };
    set(&mut s, block::INPUT, 0, 2.0);
    set(&mut s, block::INPUT, 4, 1.0); // stereo source
    if !aux {
        set(&mut s, block::FILTER, 1, 1.0);
        set(&mut s, block::FILTER, 2, 90.0);
        set(&mut s, block::FILTER, 4, 1.0);
        set(&mut s, block::FILTER, 5, 15_000.0);
        set(&mut s, block::TOOL, 0, 1.0);
        s.apply(&ParamMsg::Model {
            block: block::GATE,
            model: 0,
        });
        set(&mut s, block::GATE, TARGET_BASE, -50.0);
        set(&mut s, block::GATE, 0, 1.0);
        set(&mut s, block::PEQ, 0, 1.0);
        for b in 0..3u16 {
            set(&mut s, block::PEQ, 1 + 3 * b, 2.0 - b as f32);
        }
    }
    for (i, slot) in [0u16, 7, 10, 13, 16].iter().enumerate() {
        set(&mut s, block::EQ, TARGET_BASE + slot, 3.0 - i as f32 * 1.5);
    }
    set(&mut s, block::EQ, 0, 1.0);
    s.apply(&ParamMsg::Model {
        block: block::DYN,
        model: if aux { 9 } else { 13 },
    });
    set(&mut s, block::DYN, TARGET_BASE, -20.0);
    set(&mut s, block::DYN, 0, 1.0);
    set(&mut s, block::DYN, 1, 80.0); // parallel compression: exercise the mix path
    s.settle();
    s
}

fn bench(c: &mut Criterion) {
    let mut strips: Vec<ChannelStrip> = (0..48).map(|i| full_strip(i >= 40)).collect();
    let src: Vec<f32> = (0..N * 64)
        .map(|i| (i as f32 * 0.031).sin() * 0.5 + (i as f32 * 0.0007).sin() * 0.3)
        .collect();
    let mut bufs: Vec<([f32; N], [f32; N])> = vec![([0.0; N], [0.0; N]); 48];
    let mut k = 0usize;
    let mut cycle = |strips: &mut Vec<ChannelStrip>, bufs: &mut Vec<([f32; N], [f32; N])>| {
        let off = (k % 64) * N;
        k += 1;
        for (s, (l, r)) in strips.iter_mut().zip(bufs.iter_mut()) {
            l.copy_from_slice(&src[off..off + N]);
            r.copy_from_slice(&src[off..off + N]);
            s.process_stereo(l, r);
        }
        black_box(&bufs[0].0[0]);
    };
    // warm-up + manual summary
    for _ in 0..2000 {
        cycle(&mut strips, &mut bufs);
    }
    let iters = 20_000;
    let t0 = Instant::now();
    for _ in 0..iters {
        cycle(&mut strips, &mut bufs);
    }
    let us = t0.elapsed().as_secs_f64() * 1e6 / iters as f64;
    eprintln!(
        "strip bench: 40 ch + 8 aux full strips @64 samples: {us:.1} us/cycle = {:.1} % of 1333 us ({:.2} us/strip)",
        us / 1333.3 * 100.0,
        us / 48.0
    );
    c.bench_function("strips_40ch_8aux_full_64", |b| {
        b.iter(|| cycle(&mut strips, &mut bufs))
    });
}

criterion_group!(benches, bench);

fn main() {
    benches();
    Criterion::default().configure_from_args().final_summary();
}
