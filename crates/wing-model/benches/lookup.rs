use std::hint::black_box;

use criterion::{criterion_group, criterion_main, Criterion};
use wing_model::{wing_hash, ParamStore, Schema, Value};

fn bench(c: &mut Criterion) {
    let st = ParamStore::new();
    let s = Schema::get();
    let fdr = st.lookup("ch/17/fdr").unwrap();
    let h = s.hash(fdr);
    c.bench_function("schema_init_and_store_new", |b| {
        b.iter(|| black_box(ParamStore::new()))
    });
    c.bench_function("wing_hash ch/17/eq/3f", |b| {
        b.iter(|| wing_hash(black_box("ch/17/eq/3f")))
    });
    c.bench_function("by_hash", |b| b.iter(|| st.by_hash(black_box(h))));
    c.bench_function("lookup path ch/17/fdr", |b| {
        b.iter(|| st.lookup(black_box("ch/17/fdr")))
    });
    c.bench_function("lookup model name ch/17/dyn/thr", |b| {
        b.iter(|| st.lookup(black_box("ch/17/dyn/thr")))
    });
    c.bench_function("get", |b| b.iter(|| st.get(black_box(fdr))));
    let mut v = 0.0f32;
    c.bench_function("set float (changed)", |b| {
        b.iter(|| {
            v = if v > -50.0 { v - 1.0 } else { 0.0 };
            st.set(black_box(fdr), Value::Float(v))
        })
    });
}

criterion_group!(benches, bench);
criterion_main!(benches);
