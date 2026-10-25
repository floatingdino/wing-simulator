//! Dot products with a **fixed** floating-point evaluation order, explicitly vectorised (T070).
//!
//! Each function defines its result as a specific scalar computation (lane accumulators over
//! fixed-size chunks, a fixed reduction tree, then the tail added sequentially; separate multiply and
//! add, never fused). The NEON paths perform exactly those IEEE operations lane by lane, so they are
//! bit-identical to the scalar definitions, which stay the fallback on other targets. (LLVM does not
//! vectorise the scalar form by itself: it keeps 8 scalar accumulators, T070 profiling.)

/// `a · b[..a.len()]` with 8 lane accumulators: `acc[l] += a[8c+l] * b[8c+l]` over the full chunks
/// `c`, then `((acc0+acc4)+(acc1+acc5)) + ((acc2+acc6)+(acc3+acc7))`, then `s += a[i] * b[i]` for
/// the remaining `a.len() % 8` elements in order. Panics if `b` is shorter than `a`.
#[inline(always)]
pub fn dot_acc8(a: &[f32], b: &[f32]) -> f32 {
    let b = &b[..a.len()];
    #[cfg(target_arch = "aarch64")]
    {
        // SAFETY: NEON is mandatory on aarch64; every load is within `a` / `b` (`8 * n8 <= len`).
        unsafe {
            use core::arch::aarch64::*;
            let n8 = a.len() / 8;
            let (pa, pb) = (a.as_ptr(), b.as_ptr());
            let mut lo = vdupq_n_f32(0.0);
            let mut hi = vdupq_n_f32(0.0);
            for c in 0..n8 {
                let (x, y) = (pa.add(8 * c), pb.add(8 * c));
                lo = vaddq_f32(lo, vmulq_f32(vld1q_f32(x), vld1q_f32(y)));
                hi = vaddq_f32(hi, vmulq_f32(vld1q_f32(x.add(4)), vld1q_f32(y.add(4))));
            }
            // [acc0+acc4, acc1+acc5, acc2+acc6, acc3+acc7] -> pairwise -> (s0+s1) + (s2+s3)
            let s4 = vaddq_f32(lo, hi);
            let p = vpaddq_f32(s4, s4);
            let mut s = vgetq_lane_f32::<0>(p) + vgetq_lane_f32::<1>(p);
            for i in 8 * n8..a.len() {
                s += *pa.add(i) * *pb.add(i);
            }
            s
        }
    }
    #[cfg(not(target_arch = "aarch64"))]
    {
        dot_acc8_scalar(a, b)
    }
}

/// [`dot_acc8`]`(a, window(m))` for `m in 0..n` (`out(m, value)`), with `a.len() == 8 * C + R`
/// fixed at compile time so the taps stay in registers across the windows (FIR filters run one
/// dot per output; T070). Bit-identical to calling [`dot_acc8`] per window. Panics if
/// `a.len() != 8 * C + R` or a window is shorter than `a`.
#[inline(always)]
pub fn dot_acc8_each<'w, const C: usize, const R: usize>(
    a: &[f32],
    n: usize,
    mut window: impl FnMut(usize) -> &'w [f32],
    mut out: impl FnMut(usize, f32),
) {
    assert_eq!(a.len(), 8 * C + R);
    #[cfg(target_arch = "aarch64")]
    // SAFETY: NEON is mandatory on aarch64; tap loads are within `a` (8 * C + R), window loads
    // within the asserted window length.
    unsafe {
        use core::arch::aarch64::*;
        let pa = a.as_ptr();
        let lo: [float32x4_t; C] = core::array::from_fn(|c| vld1q_f32(pa.add(8 * c)));
        let hi: [float32x4_t; C] = core::array::from_fn(|c| vld1q_f32(pa.add(8 * c + 4)));
        let tail: [f32; R] = core::array::from_fn(|i| a[8 * C + i]);
        for m in 0..n {
            let w = window(m);
            assert!(w.len() >= a.len());
            let pb = w.as_ptr();
            let mut sl = vdupq_n_f32(0.0);
            let mut sh = vdupq_n_f32(0.0);
            for c in 0..C {
                sl = vaddq_f32(sl, vmulq_f32(lo[c], vld1q_f32(pb.add(8 * c))));
                sh = vaddq_f32(sh, vmulq_f32(hi[c], vld1q_f32(pb.add(8 * c + 4))));
            }
            let s4 = vaddq_f32(sl, sh);
            let p = vpaddq_f32(s4, s4);
            let mut s = vgetq_lane_f32::<0>(p) + vgetq_lane_f32::<1>(p);
            for (i, t) in tail.iter().enumerate() {
                s += *t * *pb.add(8 * C + i);
            }
            out(m, s);
        }
    }
    #[cfg(not(target_arch = "aarch64"))]
    for m in 0..n {
        out(m, dot_acc8_scalar(a, window(m)));
    }
}

/// The scalar definition of [`dot_acc8`].
#[inline(always)]
pub fn dot_acc8_scalar(a: &[f32], b: &[f32]) -> f32 {
    let (ac, ar) = a.as_chunks::<8>();
    let (bc, br) = b[..a.len()].as_chunks::<8>();
    let mut acc = [0.0f32; 8];
    for (x, y) in ac.iter().zip(bc) {
        for l in 0..8 {
            acc[l] += x[l] * y[l];
        }
    }
    let mut s = ((acc[0] + acc[4]) + (acc[1] + acc[5])) + ((acc[2] + acc[6]) + (acc[3] + acc[7]));
    for (x, y) in ar.iter().zip(br) {
        s += x * y;
    }
    s
}

/// `a · b[..a.len()]` with 4 lane accumulators: `acc[l] += a[4c+l] * b[4c+l]`, then
/// `(acc0+acc2) + (acc1+acc3)`, then the `a.len() % 4` tail added in order.
#[inline(always)]
pub fn dot_acc4(a: &[f32], b: &[f32]) -> f32 {
    let b = &b[..a.len()];
    #[cfg(target_arch = "aarch64")]
    {
        // SAFETY: as in `dot_acc8`.
        unsafe {
            use core::arch::aarch64::*;
            let n4 = a.len() / 4;
            let (pa, pb) = (a.as_ptr(), b.as_ptr());
            let mut acc = vdupq_n_f32(0.0);
            for c in 0..n4 {
                acc = vaddq_f32(
                    acc,
                    vmulq_f32(vld1q_f32(pa.add(4 * c)), vld1q_f32(pb.add(4 * c))),
                );
            }
            // [acc0+acc2, acc1+acc3] -> (acc0+acc2) + (acc1+acc3)
            let v = vadd_f32(vget_low_f32(acc), vget_high_f32(acc));
            let mut s = vget_lane_f32::<0>(v) + vget_lane_f32::<1>(v);
            for i in 4 * n4..a.len() {
                s += *pa.add(i) * *pb.add(i);
            }
            s
        }
    }
    #[cfg(not(target_arch = "aarch64"))]
    {
        dot_acc4_scalar(a, b)
    }
}

/// The scalar definition of [`dot_acc4`].
#[inline(always)]
pub fn dot_acc4_scalar(a: &[f32], b: &[f32]) -> f32 {
    let mut acc = [0.0f32; 4];
    let (ac, ar) = a.as_chunks::<4>();
    let (bc, br) = b[..a.len()].as_chunks::<4>();
    for (x, y) in ac.iter().zip(bc) {
        for l in 0..4 {
            acc[l] += x[l] * y[l];
        }
    }
    let mut s = (acc[0] + acc[2]) + (acc[1] + acc[3]);
    for (x, y) in ar.iter().zip(br) {
        s += x * y;
    }
    s
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn simd_dots_are_bit_exact_with_scalar_definitions() {
        let mut seed = 0x2545_f491u32;
        let mut rnd = || {
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            // wide dynamic range incl. signed zeros, tiny and large values
            let m = (seed >> 9) as f32 / (1u32 << 23) as f32 - 0.5;
            let e = ((seed >> 3) & 31) as i32 - 16;
            if seed & 0x7 == 0 {
                -0.0
            } else {
                m * 2f32.powi(e)
            }
        };
        for len in 0..100 {
            for _ in 0..20 {
                let a: Vec<f32> = (0..len).map(|_| rnd()).collect();
                let b: Vec<f32> = (0..len + 3).map(|_| rnd()).collect();
                assert_eq!(
                    dot_acc8(&a, &b).to_bits(),
                    dot_acc8_scalar(&a, &b).to_bits(),
                    "acc8 len {len}"
                );
                assert_eq!(
                    dot_acc4(&a, &b).to_bits(),
                    dot_acc4_scalar(&a, &b).to_bits(),
                    "acc4 len {len}"
                );
            }
        }
    }
}
