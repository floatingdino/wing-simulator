//! Planar, sample-time indexed f32 rings inside the segment.
//!
//! Accessed only through raw-pointer copies (never `&[f32]` into shared
//! memory, which another process mutates). Correctness relies on the
//! single-producer protocol: producers write at/after `write_time`, consumers
//! read before it and at most `RING_FRAMES - RING_GUARD_FRAMES` behind.

use crate::layout::{CHANNELS, RING_FRAMES};

/// One direction's ring (`[CHANNELS][RING_FRAMES]` f32).
#[derive(Clone, Copy)]
pub(crate) struct Ring {
    base: *mut f32,
}

// SAFETY: a plain pointer into a mapping owned by the Segment that outlives
// every Ring (they live in the same struct).
unsafe impl Send for Ring {}
unsafe impl Sync for Ring {}

#[inline]
fn index(t: i64) -> usize {
    (t as u64 & (RING_FRAMES as u64 - 1)) as usize
}

impl Ring {
    /// # Safety
    /// `base` must point to `CHANNELS * RING_FRAMES` f32 that stay mapped.
    pub(crate) unsafe fn new(base: *mut u8) -> Self {
        Self { base: base.cast() }
    }

    #[inline]
    fn chan(&self, ch: usize) -> *mut f32 {
        assert!(ch < CHANNELS, "channel out of range");
        // SAFETY: ch < CHANNELS keeps us inside the ring.
        unsafe { self.base.add(ch * RING_FRAMES) }
    }

    /// Copy frames `[t, t + dst.len())` of channel `ch` into `dst`.
    #[inline]
    pub(crate) fn read(&self, ch: usize, t: i64, dst: &mut [f32]) {
        let n = dst.len().min(RING_FRAMES);
        let c = self.chan(ch);
        let i = index(t);
        let first = n.min(RING_FRAMES - i);
        // SAFETY: i + first <= RING_FRAMES and n - first <= i.
        unsafe {
            std::ptr::copy_nonoverlapping(c.add(i), dst.as_mut_ptr(), first);
            std::ptr::copy_nonoverlapping(c, dst.as_mut_ptr().add(first), n - first);
        }
    }

    /// Copy `src` into frames `[t, t + src.len())` of channel `ch`.
    #[inline]
    pub(crate) fn write(&self, ch: usize, t: i64, src: &[f32]) {
        let n = src.len().min(RING_FRAMES);
        let c = self.chan(ch);
        let i = index(t);
        let first = n.min(RING_FRAMES - i);
        // SAFETY: as in `read`.
        unsafe {
            std::ptr::copy_nonoverlapping(src.as_ptr(), c.add(i), first);
            std::ptr::copy_nonoverlapping(src.as_ptr().add(first), c, n - first);
        }
    }

    /// Read one sample (for interleaving in the driver simulator).
    #[inline]
    pub(crate) fn get(&self, ch: usize, t: i64) -> f32 {
        // SAFETY: index within the channel.
        unsafe { self.chan(ch).add(index(t)).read() }
    }

    /// Write one sample.
    #[inline]
    pub(crate) fn set(&self, ch: usize, t: i64, v: f32) {
        // SAFETY: index within the channel.
        unsafe { self.chan(ch).add(index(t)).write(v) }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::layout::TO_DAEMON_OFFSET;
    use crate::Segment;

    #[test]
    fn wraps_across_ring_end() {
        let seg = Segment::anonymous(48_000).unwrap();
        // SAFETY: offset inside the segment.
        let r = unsafe { Ring::new(seg.base().add(TO_DAEMON_OFFSET)) };
        let src: Vec<f32> = (0..100).map(|i| i as f32).collect();
        let t = RING_FRAMES as i64 * 3 - 40; // crosses the wrap
        r.write(47, t, &src);
        let mut dst = vec![0.0; 100];
        r.read(47, t, &mut dst);
        assert_eq!(src, dst);
        assert_eq!(r.get(47, t + 40), 40.0);
        assert_eq!(r.get(47, (RING_FRAMES * 3) as i64), 40.0);
        let mut other = vec![1.0; 100];
        r.read(46, t, &mut other);
        assert!(other.iter().all(|&v| v == 0.0));
    }
}
