//! Lock-free single-producer / single-consumer ring of planar `f32` audio.
//!
//! The engine thread ([`RingProducer`]) writes whole blocks; the device IO thread
//! ([`RingConsumer`]) reads frames. Indices are monotonically increasing `u64` frame counters
//! (never wrap in practice); storage is one planar buffer per channel with a power-of-two
//! capacity. Besides the audio, the producer publishes a timestamp `(host time, write index)`
//! of its latest push (seqlock-protected so the pair is consistent), from which the consumer
//! models the producer's position as a smooth line instead of a block sawtooth.
//!
//! Wait-free on both sides: atomics + memcpy, no allocation after [`ring`].

use std::cell::UnsafeCell;
use std::sync::atomic::{fence, AtomicU64, Ordering};
use std::sync::Arc;

/// Cache-line padded value (128 bytes covers Apple silicon's line size).
#[repr(align(128))]
struct Padded<T>(T);

struct Shared {
    channels: usize,
    cap: usize,
    mask: u64,
    /// Planar storage: channel `c` occupies `[c * cap, (c + 1) * cap)`.
    data: Box<[UnsafeCell<f32>]>,
    /// Frames written (producer-owned).
    write: Padded<AtomicU64>,
    /// Frames read (consumer-owned).
    read: Padded<AtomicU64>,
    /// Seqlock over (`stamp_ns`, `stamp_end`): odd while the producer updates them.
    stamp_seq: Padded<AtomicU64>,
    /// Host time (ns) of the latest push.
    stamp_ns: AtomicU64,
    /// Write index right after that push.
    stamp_end: AtomicU64,
    /// Blocks (or partial blocks) dropped because the ring was full.
    overflow_frames: AtomicU64,
}

// SAFETY: the SPSC protocol guarantees producer and consumer never touch the same cells
// concurrently: the producer only writes cells in [write, read + cap), the consumer only reads
// cells in [read, write), with Release/Acquire on the counters ordering the data accesses.
unsafe impl Sync for Shared {}
// SAFETY: see above; `UnsafeCell<f32>` is plain data.
unsafe impl Send for Shared {}

impl Shared {
    #[inline]
    fn cell(&self, ch: usize, idx: u64) -> *mut f32 {
        self.data[ch * self.cap + (idx & self.mask) as usize].get()
    }
}

/// Create a ring with at least `min_frames` capacity (rounded up to a power of two).
pub fn ring(channels: usize, min_frames: usize) -> (RingProducer, RingConsumer) {
    assert!(channels > 0, "ring needs at least one channel");
    let cap = min_frames.max(16).next_power_of_two();
    let data: Box<[UnsafeCell<f32>]> = (0..channels * cap).map(|_| UnsafeCell::new(0.0)).collect();
    let shared = Arc::new(Shared {
        channels,
        cap,
        mask: cap as u64 - 1,
        data,
        write: Padded(AtomicU64::new(0)),
        read: Padded(AtomicU64::new(0)),
        stamp_seq: Padded(AtomicU64::new(0)),
        stamp_ns: AtomicU64::new(0),
        stamp_end: AtomicU64::new(0),
        overflow_frames: AtomicU64::new(0),
    });
    (
        RingProducer {
            shared: shared.clone(),
        },
        RingConsumer {
            shared,
            read: 0,
            limit: 0,
        },
    )
}

/// Engine-side writer.
pub struct RingProducer {
    shared: Arc<Shared>,
}

impl RingProducer {
    /// Number of channels.
    pub fn channels(&self) -> usize {
        self.shared.channels
    }

    /// Capacity in frames.
    pub fn capacity(&self) -> usize {
        self.shared.cap
    }

    /// Push `frames` frames; `source(ch)` returns channel `ch`'s samples (at least `frames`
    /// long). `now_ns` is the host time of the push (0 if unknown). If the ring cannot take the
    /// whole block it is dropped and counted as overflow (the consumer is then far behind and
    /// will resync). Returns whether the block was written.
    pub fn push_with<'a>(
        &mut self,
        frames: usize,
        now_ns: u64,
        mut source: impl FnMut(usize) -> &'a [f32],
    ) -> bool {
        let s = &*self.shared;
        let w = s.write.0.load(Ordering::Relaxed);
        let r = s.read.0.load(Ordering::Acquire);
        let free = s.cap as u64 - (w - r);
        if (frames as u64) > free {
            s.overflow_frames
                .fetch_add(frames as u64, Ordering::Relaxed);
            return false;
        }
        for ch in 0..s.channels {
            let src = &source(ch)[..frames];
            let start = (w & s.mask) as usize;
            let first = frames.min(s.cap - start);
            // SAFETY: cells [w, w+frames) are free (not visible to the consumer until the
            // Release store below); channel region bounds are respected by `first`.
            unsafe {
                let base = s.data[ch * s.cap].get();
                std::ptr::copy_nonoverlapping(src.as_ptr(), base.add(start), first);
                std::ptr::copy_nonoverlapping(src.as_ptr().add(first), base, frames - first);
            }
        }
        let end = w + frames as u64;
        s.write.0.store(end, Ordering::Release);
        if now_ns != 0 {
            self.publish_stamp(now_ns, end);
        }
        true
    }

    /// Publish `(now_ns, current write index)` as the producer timestamp without pushing
    /// audio, e.g. once at the end of a burst of blocks pushed with `now_ns = 0`.
    pub fn stamp(&mut self, now_ns: u64) {
        if now_ns != 0 {
            let end = self.shared.write.0.load(Ordering::Relaxed);
            self.publish_stamp(now_ns, end);
        }
    }

    fn publish_stamp(&self, now_ns: u64, end: u64) {
        let s = &*self.shared;
        let seq = s.stamp_seq.0.load(Ordering::Relaxed);
        s.stamp_seq.0.store(seq + 1, Ordering::Relaxed);
        fence(Ordering::Release);
        s.stamp_ns.store(now_ns, Ordering::Relaxed);
        s.stamp_end.store(end, Ordering::Relaxed);
        s.stamp_seq.0.store(seq + 2, Ordering::Release);
    }

    /// Push a planar block given as one slice per channel.
    pub fn push(&mut self, block: &[&[f32]], now_ns: u64) -> bool {
        let frames = block.iter().map(|c| c.len()).min().unwrap_or(0);
        self.push_with(frames, now_ns, |ch| block[ch])
    }

    /// Frames currently queued.
    pub fn len(&self) -> usize {
        let s = &*self.shared;
        (s.write.0.load(Ordering::Relaxed) - s.read.0.load(Ordering::Acquire)) as usize
    }

    /// Whether nothing is queued.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Total frames dropped because the ring was full.
    pub fn overflow_frames(&self) -> u64 {
        self.shared.overflow_frames.load(Ordering::Relaxed)
    }
}

/// Snapshot of the producer's state taken by [`RingConsumer::begin`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PushInfo {
    /// Frames available to read.
    pub available: u64,
    /// Frames consumed so far (the consumer's read index).
    pub read_index: u64,
    /// `(host time ns, write index after the push)` of the latest timestamped push; `None` if
    /// there is none or the producer was mid-update.
    pub stamp: Option<(u64, u64)>,
}

/// Device-side reader. Usage per callback: [`begin`](Self::begin), any number of
/// [`frame`](Self::frame)/[`skip`](Self::skip) calls, then [`commit`](Self::commit).
pub struct RingConsumer {
    shared: Arc<Shared>,
    /// Local read index (published on commit).
    read: u64,
    /// Write index snapshot from `begin`.
    limit: u64,
}

impl RingConsumer {
    /// Number of channels.
    pub fn channels(&self) -> usize {
        self.shared.channels
    }

    /// Capacity in frames.
    pub fn capacity(&self) -> usize {
        self.shared.cap
    }

    /// Snapshot what the producer has published.
    #[inline]
    pub fn begin(&mut self) -> PushInfo {
        let s = &*self.shared;
        self.limit = s.write.0.load(Ordering::Acquire);
        let s1 = s.stamp_seq.0.load(Ordering::Acquire);
        let t = s.stamp_ns.load(Ordering::Relaxed);
        let e = s.stamp_end.load(Ordering::Relaxed);
        fence(Ordering::Acquire);
        let s2 = s.stamp_seq.0.load(Ordering::Relaxed);
        let stamp = (s1 == s2 && s1.is_multiple_of(2) && t != 0).then_some((t, e));
        PushInfo {
            available: self.limit - self.read,
            read_index: self.read,
            stamp,
        }
    }

    /// Frames still readable since `begin`.
    #[inline]
    pub fn remaining(&self) -> u64 {
        self.limit - self.read
    }

    /// Read the next frame into `out` (one sample per channel). Returns `false` (and leaves
    /// `out` untouched) if no frame is available.
    #[inline]
    pub fn frame(&mut self, out: &mut [f32]) -> bool {
        if self.read >= self.limit {
            return false;
        }
        let s = &*self.shared;
        for (ch, o) in out.iter_mut().enumerate().take(s.channels) {
            // SAFETY: `read < limit` (Acquire-published), so the cell holds producer data.
            *o = unsafe { *s.cell(ch, self.read) };
        }
        self.read += 1;
        true
    }

    /// Drop up to `frames` readable frames; returns how many were dropped.
    pub fn skip(&mut self, frames: u64) -> u64 {
        let n = frames.min(self.limit - self.read);
        self.read += n;
        n
    }

    /// Publish consumed frames back to the producer.
    #[inline]
    pub fn commit(&mut self) {
        self.shared.read.0.store(self.read, Ordering::Release);
    }

    /// Total frames dropped by the producer because the ring was full.
    pub fn overflow_frames(&self) -> u64 {
        self.shared.overflow_frames.load(Ordering::Relaxed)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip_with_wrap() {
        let (mut p, mut c) = ring(2, 16);
        assert_eq!(p.capacity(), 16);
        let mut n = 0.0f32;
        let mut expect = 0.0f32;
        let mut out = [0.0f32; 2];
        for _ in 0..50 {
            let a: Vec<f32> = (0..5).map(|i| n + i as f32).collect();
            let b: Vec<f32> = a.iter().map(|x| -x).collect();
            assert!(p.push(&[&a, &b], 0));
            n += 5.0;
            let info = c.begin();
            assert_eq!(info.available, 5);
            assert_eq!(info.stamp, None);
            while c.frame(&mut out) {
                assert_eq!(out, [expect, -expect]);
                expect += 1.0;
            }
            c.commit();
        }
        assert_eq!(p.overflow_frames(), 0);
    }

    #[test]
    fn overflow_drops_whole_block() {
        let (mut p, mut c) = ring(1, 16);
        let blk = [1.0f32; 10];
        assert!(p.push(&[&blk], 77));
        assert!(!p.push(&[&blk], 0));
        assert_eq!(p.overflow_frames(), 10);
        let info = c.begin();
        assert_eq!(info.available, 10);
        assert_eq!(info.stamp, Some((77, 10)));
        assert_eq!(c.skip(4), 4);
        c.commit();
        assert_eq!(p.len(), 6);
    }

    #[test]
    fn threaded_ordering() {
        let (mut p, mut c) = ring(1, 256);
        let t = std::thread::spawn(move || {
            let mut v = 0u32;
            let mut blk = [0.0f32; 32];
            while v < 100_000 {
                for s in blk.iter_mut() {
                    *s = v as f32;
                    v += 1;
                }
                while !p.push(&[&blk], 0) {
                    v -= 32;
                    for s in blk.iter_mut() {
                        *s = v as f32;
                        v += 1;
                    }
                    std::thread::yield_now();
                }
            }
        });
        let mut expect = 0u32;
        let mut out = [0.0f32];
        while expect < 100_000 {
            c.begin();
            while c.frame(&mut out) {
                assert_eq!(out[0], expect as f32);
                expect += 1;
            }
            c.commit();
        }
        t.join().unwrap();
    }
}
