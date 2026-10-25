//! dB <-> linear conversions.
//!
//! - [`db_to_lin`] / [`lin_to_db`]: exact (libm) versions, for control-rate use.
//! - [`fast_exp2`], [`fast_log2`], [`db_to_lin_fast`], [`lin_to_db_fast`]: branch-light polynomial versions for
//!   per-sample use (e.g. gain computers). Error bounds (verified by tests over the full range):
//!   - `fast_exp2`: relative error < 5e-6 for inputs in [-126, 126];
//!   - `db_to_lin_fast`: relative error < 6e-6 (< 0.00006 dB) over [-240, +120] dB;
//!   - `fast_log2`: absolute error < 4e-6 for positive normal inputs (dominated by f32 rounding of the
//!     result when |log2 x| is large; < 5e-7 for x in [2^-8, 2^8]);
//!   - `lin_to_db_fast`: absolute error < 4e-5 dB over [-240, +120] dB. Non-positive inputs return
//!     [`MIN_DB`].
//! - [`GainTable`]: precomputed dB->linear lookup with linear interpolation (fixed size, no allocation).

/// Floor used for "silence" in dB conversions.
pub const MIN_DB: f32 = -240.0;

/// 20*log10(2): dB per doubling.
const DB_PER_LOG2: f32 = 20.0 * core::f32::consts::LOG10_2;
/// log2(10)/20: log2 units per dB.
const LOG2_PER_DB: f32 = core::f32::consts::LOG2_10 / 20.0;

#[inline]
pub fn db_to_lin(db: f32) -> f32 {
    10f32.powf(db * 0.05)
}

#[inline]
pub fn lin_to_db(lin: f32) -> f32 {
    if lin > 0.0 {
        (20.0 * lin.log10()).max(MIN_DB)
    } else {
        MIN_DB
    }
}

#[inline]
pub fn db_to_lin_f64(db: f64) -> f64 {
    10f64.powf(db * 0.05)
}

#[inline]
pub fn lin_to_db_f64(lin: f64) -> f64 {
    if lin > 0.0 {
        20.0 * lin.log10()
    } else {
        MIN_DB as f64
    }
}

/// Fast 2^x. Splits x into round(x) + f, f in [-0.5, 0.5], evaluates e^(f ln2) with a degree-5 Taylor
/// polynomial (truncation error < 2.5e-6 relative) and builds 2^round(x) from the exponent bits.
/// Input is clamped to [-126, 126] (normal f32 range).
#[inline]
pub fn fast_exp2(x: f32) -> f32 {
    let x = x.clamp(-126.0, 126.0);
    let xi = x.round();
    let f = x - xi;
    // e^(f ln2) Taylor coefficients: (ln2)^k / k!
    const C1: f32 = core::f32::consts::LN_2;
    const C2: f32 = 0.240_226_5;
    const C3: f32 = 0.055_504_11;
    const C4: f32 = 0.009_618_129;
    const C5: f32 = 0.001_333_355_8;
    let p = 1.0 + f * (C1 + f * (C2 + f * (C3 + f * (C4 + f * C5))));
    let bits = ((xi as i32 + 127) as u32) << 23;
    p * f32::from_bits(bits)
}

/// Fast log2(x) for positive normal x. Splits x = m * 2^e with m in [sqrt(1/2), sqrt(2)) and evaluates
/// log2(m) = (2/ln2) * atanh(t), t = (m-1)/(m+1), |t| <= 0.1716, with a 4-term odd series.
/// Non-positive / denormal input returns -126 (i.e. treat as silence).
#[inline]
pub fn fast_log2(x: f32) -> f32 {
    if x < f32::MIN_POSITIVE {
        return -126.0;
    }
    let bits = x.to_bits();
    let mut e = ((bits >> 23) & 0xff) as i32 - 127;
    let mut m = f32::from_bits((bits & 0x007f_ffff) | 0x3f80_0000); // [1, 2)
    if m > core::f32::consts::SQRT_2 {
        m *= 0.5;
        e += 1;
    }
    let t = (m - 1.0) / (m + 1.0);
    let t2 = t * t;
    // 2/ln2 * (t + t^3/3 + t^5/5 + t^7/7)
    const K: f32 = 2.885_39;
    let s = t * (1.0 + t2 * (1.0 / 3.0 + t2 * (0.2 + t2 * (1.0 / 7.0))));
    e as f32 + K * s
}

/// Fast dB -> linear (see module docs for error bound).
#[inline]
pub fn db_to_lin_fast(db: f32) -> f32 {
    fast_exp2(db * LOG2_PER_DB)
}

/// Fast linear -> dB (see module docs for error bound).
#[inline]
pub fn lin_to_db_fast(lin: f32) -> f32 {
    if lin <= 0.0 {
        return MIN_DB;
    }
    (fast_log2(lin) * DB_PER_LOG2).max(MIN_DB)
}

/// Fixed-size dB -> linear lookup table over `[min_db, max_db]` with linear interpolation between entries.
/// Out-of-range inputs clamp. With the default 1025 points over [-144, +24] dB (0.164 dB spacing) the
/// relative interpolation error is < 0.04%.
#[derive(Clone)]
pub struct GainTable<const N: usize> {
    table: [f32; N],
    min_db: f32,
    inv_step: f32,
}

impl<const N: usize> GainTable<N> {
    pub fn new(min_db: f32, max_db: f32) -> Self {
        assert!(N >= 2 && max_db > min_db);
        let step = (max_db - min_db) / (N - 1) as f32;
        let mut table = [0.0; N];
        for (i, t) in table.iter_mut().enumerate() {
            *t = db_to_lin_f64((min_db + step * i as f32) as f64) as f32;
        }
        Self {
            table,
            min_db,
            inv_step: 1.0 / step,
        }
    }

    #[inline]
    pub fn lookup(&self, db: f32) -> f32 {
        let pos = ((db - self.min_db) * self.inv_step).clamp(0.0, (N - 1) as f32);
        let i = (pos as usize).min(N - 2);
        let frac = pos - i as f32;
        let a = self.table[i];
        let b = self.table[i + 1];
        a + (b - a) * frac
    }
}

/// Default table used for fader/gain lookups.
pub type FaderGainTable = GainTable<1025>;

impl FaderGainTable {
    pub fn fader() -> Self {
        Self::new(-144.0, 24.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exact_roundtrip() {
        for db in [-120.0f32, -6.0, 0.0, 6.020_6, 24.0] {
            assert!((lin_to_db(db_to_lin(db)) - db).abs() < 1e-4);
        }
        assert_eq!(lin_to_db(0.0), MIN_DB);
        assert!((db_to_lin(-6.020_6) - 0.5).abs() < 1e-6);
    }

    #[test]
    fn fast_exp2_error_bound() {
        let mut worst = 0.0f64;
        let mut x = -126.0f32;
        while x <= 126.0 {
            let exact = 2f64.powf(x as f64);
            let rel = ((fast_exp2(x) as f64 - exact) / exact).abs();
            worst = worst.max(rel);
            x += 0.001_7;
        }
        assert!(worst < 5e-6, "worst rel err {worst}");
    }

    #[test]
    fn db_to_lin_fast_error_bound() {
        let mut worst = 0.0f64;
        let mut db = -240.0f32;
        while db <= 120.0 {
            let exact = 10f64.powf(db as f64 / 20.0);
            let rel = ((db_to_lin_fast(db) as f64 - exact) / exact).abs();
            worst = worst.max(rel);
            db += 0.003_1;
        }
        assert!(worst < 6e-6, "worst rel err {worst}");
    }

    #[test]
    fn fast_log_error_bounds() {
        let mut worst_l2 = 0.0f64;
        let mut worst_l2_near = 0.0f64;
        let mut worst_db = 0.0f64;
        let mut x = f32::MIN_POSITIVE;
        while x < 1e38 {
            let l2 = (x as f64).log2();
            let e = (fast_log2(x) as f64 - l2).abs();
            worst_l2 = worst_l2.max(e);
            if l2.abs() <= 8.0 {
                worst_l2_near = worst_l2_near.max(e);
            }
            let db = 20.0 * (x as f64).log10();
            if (MIN_DB as f64..=120.0).contains(&db) {
                worst_db = worst_db.max((lin_to_db_fast(x) as f64 - db).abs());
            }
            x *= 1.000_37;
        }
        assert!(worst_l2 < 4e-6, "log2 err {worst_l2}");
        assert!(worst_l2_near < 5e-7, "log2 err near 0 {worst_l2_near}");
        assert!(worst_db < 4e-5, "dB err {worst_db}");
        assert_eq!(lin_to_db_fast(0.0), MIN_DB);
        assert_eq!(lin_to_db_fast(-1.0), MIN_DB);
    }

    #[test]
    fn gain_table_error_bound() {
        let t = FaderGainTable::fader();
        let mut db = -144.0f32;
        while db <= 24.0 {
            let exact = db_to_lin_f64(db as f64);
            let rel = ((t.lookup(db) as f64 - exact) / exact).abs();
            assert!(rel < 4e-4, "db {db} rel {rel}");
            db += 0.013;
        }
        assert_eq!(t.lookup(-1000.0), t.lookup(-144.0));
        assert!((t.lookup(1000.0) - db_to_lin(24.0)).abs() < 1e-4);
    }
}
