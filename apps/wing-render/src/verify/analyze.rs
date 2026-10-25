//! Signal analysis for the hardware verification kit: FFT, alignment, band levels, envelopes, harmonics.
//! All functions are offline (allocate freely) and work on mono `f32` slices at the suite rate.

use std::f64::consts::PI;

/// In-place radix-2 FFT (`buf.len()` a power of two).
pub fn fft(buf: &mut [(f64, f64)], inverse: bool) {
    let n = buf.len();
    assert!(n.is_power_of_two());
    let mut j = 0usize;
    for i in 1..n {
        let mut bit = n >> 1;
        while j & bit != 0 {
            j ^= bit;
            bit >>= 1;
        }
        j |= bit;
        if i < j {
            buf.swap(i, j);
        }
    }
    let mut len = 2;
    while len <= n {
        let ang = 2.0 * PI / len as f64 * if inverse { 1.0 } else { -1.0 };
        let (wr, wi) = (ang.cos(), ang.sin());
        for start in (0..n).step_by(len) {
            let (mut cr, mut ci) = (1.0f64, 0.0f64);
            for k in 0..len / 2 {
                let (a, b) = (buf[start + k], buf[start + k + len / 2]);
                let t = (b.0 * cr - b.1 * ci, b.0 * ci + b.1 * cr);
                buf[start + k] = (a.0 + t.0, a.1 + t.1);
                buf[start + k + len / 2] = (a.0 - t.0, a.1 - t.1);
                let nr = cr * wr - ci * wi;
                ci = cr * wi + ci * wr;
                cr = nr;
            }
        }
        len <<= 1;
    }
    if inverse {
        let s = 1.0 / n as f64;
        for v in buf.iter_mut() {
            v.0 *= s;
            v.1 *= s;
        }
    }
}

/// Power spectrum `|X[k]|^2` (k = 0..=n/2) of the Hann-windowed `x`, zero-padded to `n` (power of two).
pub fn power_spectrum(x: &[f32], n: usize) -> Vec<f64> {
    let m = x.len().min(n);
    let mut b = vec![(0.0, 0.0); n];
    for (i, v) in x.iter().take(m).enumerate() {
        let w = 0.5 - 0.5 * (2.0 * PI * (i as f64 + 0.5) / m as f64).cos();
        b[i].0 = *v as f64 * w;
    }
    fft(&mut b, false);
    b[..=n / 2].iter().map(|c| c.0 * c.0 + c.1 * c.1).collect()
}

/// Position of `needle` in `hay` (lag `k` maximising sum hay[n+k] needle[n]) and a normalised score in 0..1.
/// The lag search covers `0..=hay.len() - needle.len()`.
pub fn locate(hay: &[f32], needle: &[f32]) -> Option<(usize, f64)> {
    if hay.len() < needle.len() || needle.is_empty() {
        return None;
    }
    let n = (hay.len() + needle.len()).next_power_of_two();
    let mut a = vec![(0.0, 0.0); n];
    let mut b = vec![(0.0, 0.0); n];
    for (i, v) in hay.iter().enumerate() {
        a[i].0 = *v as f64;
    }
    for (i, v) in needle.iter().enumerate() {
        b[i].0 = *v as f64;
    }
    fft(&mut a, false);
    fft(&mut b, false);
    for (x, y) in a.iter_mut().zip(&b) {
        *x = (x.0 * y.0 + x.1 * y.1, x.1 * y.0 - x.0 * y.1);
    }
    fft(&mut a, true);
    let en: f64 = needle.iter().map(|&v| (v as f64).powi(2)).sum();
    // sliding energy of hay under the needle
    let mut cum = vec![0.0f64; hay.len() + 1];
    for (i, v) in hay.iter().enumerate() {
        cum[i + 1] = cum[i] + (*v as f64).powi(2);
    }
    let mut best = (0usize, f64::MIN);
    for k in 0..=(hay.len() - needle.len()) {
        let eh = cum[k + needle.len()] - cum[k];
        if eh < en * 1e-4 {
            continue; // (near) silence: roundoff would dominate the normalised score
        }
        let score = a[k].0 / (en * eh).sqrt();
        if score > best.1 {
            best = (k, score);
        }
    }
    Some((best.0, best.1))
}

pub fn db(x: f64) -> f64 {
    20.0 * x.max(1e-12).log10()
}

pub fn rms_db(x: &[f32]) -> f64 {
    if x.is_empty() {
        return -240.0;
    }
    let s: f64 = x.iter().map(|&v| (v as f64).powi(2)).sum::<f64>() / x.len() as f64;
    10.0 * s.max(1e-24).log10()
}

/// 1/3-octave band levels (power, linear) at centres `1000 * 2^(k/3)`, `k` in `ks`.
pub fn third_octave_power(
    x: &[f32],
    fs: f64,
    ks: std::ops::RangeInclusive<i32>,
) -> Vec<(f64, f64)> {
    let n = x.len().next_power_of_two().max(1 << 12);
    let p = power_spectrum(x, n);
    let df = fs / n as f64;
    ks.map(|k| {
        let fc = 1000.0 * 2f64.powf(k as f64 / 3.0);
        let (lo, hi) = (fc / 2f64.powf(1.0 / 6.0), fc * 2f64.powf(1.0 / 6.0));
        let (a, b) = (
            (lo / df).ceil() as usize,
            ((hi / df).floor() as usize).min(p.len() - 1),
        );
        (fc, p[a..=b.max(a)].iter().sum::<f64>())
    })
    .collect()
}

/// Sliding RMS envelope in dB, one value per `hop` samples, window `win` samples.
pub fn envelope_db(x: &[f32], win: usize, hop: usize) -> Vec<f64> {
    let mut cum = vec![0.0f64; x.len() + 1];
    for (i, v) in x.iter().enumerate() {
        cum[i + 1] = cum[i] + (*v as f64).powi(2);
    }
    (0..x.len().saturating_sub(win) / hop + 1)
        .map(|i| {
            let a = i * hop;
            let b = (a + win).min(x.len());
            10.0 * ((cum[b] - cum[a]) / (b - a).max(1) as f64)
                .max(1e-24)
                .log10()
        })
        .collect()
}

/// Level (dB re the fundamental) of harmonic `h` of `f0` in the power spectrum `p` (FFT size `n`), summing
/// +-3 bins around the harmonic; `None` above Nyquist.
pub fn harmonic_db(p: &[f64], n: usize, fs: f64, f0: f64, h: usize) -> Option<f64> {
    let df = fs / n as f64;
    let at = |f: f64| -> Option<f64> {
        let k = (f / df).round() as usize;
        if f >= fs / 2.0 - 100.0 || k + 3 >= p.len() {
            return None;
        }
        Some(p[k.saturating_sub(3)..=k + 3].iter().sum::<f64>())
    };
    let fund = at(f0)?;
    Some(10.0 * (at(f0 * h as f64)? / fund.max(1e-30)).max(1e-30).log10())
}

/// Time (ms) for an envelope to go from 10 % to 90 % of the way between its start value and its final value
/// (`env` is in dB at `hop_ms` spacing). `None` when the change is smaller than `min_change_db`.
pub fn settle_time_ms(env: &[f64], hop_ms: f64, min_change_db: f64) -> Option<f64> {
    let tail = env.len() / 5;
    let end = env[env.len() - tail.max(1)..].iter().sum::<f64>() / tail.max(1) as f64;
    let start = env[0];
    let d = end - start;
    if d.abs() < min_change_db {
        return None;
    }
    let t = |frac: f64| env.iter().position(|&v| (v - start) / d >= frac);
    Some((t(0.9)? as f64 - t(0.1)? as f64).max(0.0) * hop_ms)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fft_roundtrip_and_locate() {
        let mut b: Vec<(f64, f64)> = (0..64).map(|i| ((i as f64 * 0.3).sin(), 0.0)).collect();
        let orig = b.clone();
        fft(&mut b, false);
        fft(&mut b, true);
        assert!(b.iter().zip(&orig).all(|(a, o)| (a.0 - o.0).abs() < 1e-12));

        let mut seed = 1u32;
        let noise: Vec<f32> = (0..2000)
            .map(|_| {
                seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
                (seed >> 8) as f32 / 8e6 - 1.0
            })
            .collect();
        let mut hay = vec![0.0f32; 5000];
        hay[1234..1234 + 2000].copy_from_slice(&noise);
        let (k, s) = locate(&hay, &noise).unwrap();
        assert_eq!(k, 1234);
        assert!(s > 0.99);
    }

    #[test]
    fn harmonics_and_bands() {
        let fs = 48_000.0;
        let x: Vec<f32> = (0..65_536)
            .map(|i| {
                let t = i as f64 / fs;
                ((2.0 * PI * 1000.0 * t).sin() + 0.01 * (2.0 * PI * 3000.0 * t).sin()) as f32
            })
            .collect();
        let p = power_spectrum(&x, 65_536);
        let h3 = harmonic_db(&p, 65_536, fs, 1000.0, 3).unwrap();
        assert!((h3 + 40.0).abs() < 0.5, "{h3}");
        let b = third_octave_power(&x, fs, 0..=0);
        assert!(b[0].1 > 0.0);
    }

    #[test]
    fn settle() {
        let env: Vec<f64> = (0..200)
            .map(|i| -10.0 * (1.0 - (-(i as f64) / 20.0).exp()))
            .collect();
        let t = settle_time_ms(&env, 1.0, 1.0).unwrap();
        // 10..90 % of a first-order response: tau * ln(9) = 43.9 ms
        assert!((t - 44.0).abs() < 2.0, "{t}");
        assert!(settle_time_ms(&[0.0; 50], 1.0, 1.0).is_none());
    }
}
