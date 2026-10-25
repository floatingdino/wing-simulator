//! Criterion benches for the FX rack (T050): all 16 slots engaged as inserts (ch 1..16 Ins1), one
//! 64-sample stereo block each, through the strips' rack hosts (ownership check, SEND capture, model
//! slot). `16_stub`: every slot runs a pass-through stub model (GEQ); `8_stdl_8_stub`: FX1-8 run the
//! ST-DL stereo delay (feedback on), FX9-16 stubs.

use criterion::{black_box, criterion_group, criterion_main, Criterion, Throughput};
use wing_engine::channel::{InsertHost, InsertPos};
use wing_engine::fx::FxRack;
use wing_engine::params::ParamMsg;
use wing_engine::routing::Strip;

const FS: f64 = 48_000.0;
const BLOCK: usize = 64;
/// `mdl` enum indices: ST-DL (full group), GEQ (full / lite group).
const ST_DL: u16 = 19;
const GEQ_FULL: u16 = 28;
const GEQ_LITE: u16 = 2;

fn rack(delays: bool) -> (FxRack, Vec<Box<dyn InsertHost + Send>>) {
    let rack = FxRack::new(FS, BLOCK);
    let mut hosts = Vec::new();
    for n in 1..=16u8 {
        let model = match (n <= 8, delays) {
            (true, true) => ST_DL,
            (true, false) => GEQ_FULL,
            _ => GEQ_LITE,
        };
        // SAFETY: no engine is running.
        unsafe {
            rack.with_slot(n, |s| {
                s.apply(&ParamMsg::Model { block: 0, model });
                if model == ST_DL {
                    // FEED 60 %
                    s.apply(&ParamMsg::Set {
                        block: 0,
                        param: wing_engine::params::TARGET_BASE + 5,
                        value: 60.0,
                    });
                }
                s.apply(&ParamMsg::Commit);
                s.reset();
            })
        };
        let mut h = rack.host(Strip::Ch(n));
        h.assign(InsertPos::Pre, n);
        hosts.push(h);
    }
    (rack, hosts)
}

fn bench(c: &mut Criterion) {
    let mut g = c.benchmark_group("fx_rack");
    g.throughput(Throughput::Elements((16 * BLOCK) as u64));
    let x: Vec<f32> = (0..BLOCK).map(|i| (i as f32 * 0.03).sin() * 0.5).collect();
    for (label, delays) in [("16_stub", false), ("8_stdl_8_stub", true)] {
        let (_rack, mut hosts) = rack(delays);
        let mut bufs: Vec<(Vec<f32>, Vec<f32>)> = (0..16).map(|_| (x.clone(), x.clone())).collect();
        g.bench_function(label, |b| {
            b.iter(|| {
                for (k, (h, (l, r))) in hosts.iter_mut().zip(bufs.iter_mut()).enumerate() {
                    h.process_insert_at(InsertPos::Pre, k as u8 + 1, black_box(l), black_box(r));
                }
            })
        });
    }
    g.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
