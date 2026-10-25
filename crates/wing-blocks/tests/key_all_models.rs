//! T080: an external side-chain key reaches every gate/dynamics model. For each implemented model of the
//! shared gate/dynamics table, a quiet programme keyed by a loud key must come out different from the same
//! programme keyed by silence.

use wing_blocks::model::{catalog, create_model, BlockKind, ParamValue};

const FS: f64 = 48_000.0;
const BLOCK: usize = 64;

/// Models whose output by design does not depend on a key: `WARM` is a preamp (no level detector).
const KEYLESS: &[&str] = &["WARM"];

/// Parameter overrides for models whose factory defaults are neutral (1-based slot, value).
fn overrides(id: &str) -> &'static [(u16, f32)] {
    match id {
        "WAVE" => &[(1, 12.0), (2, -12.0)],
        "DEQ" => &[(1, -40.0), (6, -15.0), (8, 0.5)],
        "DEQ2" => &[(1, -40.0), (6, -15.0), (10, -40.0), (15, -15.0)],
        // band thresholds (defaults 0 dB)
        "3CMP" => &[(5, -30.0), (10, -30.0), (15, -30.0)],
        _ => &[],
    }
}

/// Deterministic white noise in [-1, 1).
struct Noise(u32);

impl Noise {
    fn next(&mut self) -> f32 {
        self.0 = self.0.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        (self.0 >> 8) as f32 / (1u32 << 23) as f32 - 1.0
    }
}

fn render(id: &'static str, key_amp: f32) -> Vec<f32> {
    let mut m = create_model(BlockKind::Dynamics, id);
    m.prepare(FS, BLOCK);
    m.load_defaults();
    for &(slot, v) in overrides(id) {
        m.set_param(slot, ParamValue::Float(v));
    }
    m.commit();
    m.reset();
    let n = 48_000;
    let mut out = Vec::with_capacity(n);
    let (mut prog, mut keyn) = (Noise(1), Noise(7));
    let mut i = 0usize;
    while i < n {
        let (mut l, mut r) = ([0f32; BLOCK], [0f32; BLOCK]);
        let mut k = [0f32; BLOCK];
        for j in 0..BLOCK {
            // quiet broadband programme; the key: 50 ms noise bursts every 100 ms
            l[j] = 0.05 * prog.next();
            r[j] = l[j];
            let burst = (i + j) % 4800 < 2400;
            k[j] = if burst { key_amp * keyn.next() } else { 0.0 };
        }
        m.process(&mut l, &mut r, Some((&k, &k)));
        out.extend_from_slice(&l);
        i += BLOCK;
    }
    out
}

fn rms_db(x: &[f32]) -> f32 {
    let s: f64 = x.iter().map(|&v| (v as f64) * (v as f64)).sum();
    (10.0 * (s / x.len() as f64 + 1e-30).log10()) as f32
}

#[test]
fn external_key_reaches_every_dynamics_model() {
    let mut deaf = Vec::new();
    for &id in catalog(BlockKind::Dynamics) {
        if create_model(BlockKind::Dynamics, id).is_stub() || KEYLESS.contains(&id) {
            continue;
        }
        let a = render(id, 0.0);
        let b = render(id, 1.0);
        let tail = a.len() / 2;
        let d = (rms_db(&a[tail..]) - rms_db(&b[tail..])).abs();
        if d < 0.5 {
            deaf.push(format!("{id} (level diff {d:.3} dB)"));
        }
    }
    assert!(
        deaf.is_empty(),
        "models ignoring the external key: {deaf:?}"
    );
}
