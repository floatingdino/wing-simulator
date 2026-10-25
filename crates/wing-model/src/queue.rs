//! Lock-free bounded MPMC ring (Dmitry Vyukov's algorithm), used as the MPSC change queue from
//! control threads to the RT engine. Fixed capacity, allocation only at construction.

use std::cell::UnsafeCell;
use std::sync::atomic::{AtomicUsize, Ordering};

struct Slot<T> {
    seq: AtomicUsize,
    val: UnsafeCell<T>,
}

/// Bounded lock-free queue of `Copy` items.
pub struct BoundedQueue<T: Copy> {
    slots: Box<[Slot<T>]>,
    mask: usize,
    head: CachePadded<AtomicUsize>, // dequeue position
    tail: CachePadded<AtomicUsize>, // enqueue position
}

#[repr(align(128))]
struct CachePadded<T>(T);

// SAFETY: a slot's value is only written by the producer that won the CAS on `tail` for that
// position and only read by the consumer that won the CAS on `head`; `seq` (release/acquire)
// orders the hand-over. T: Copy + Send makes moving values across threads sound.
unsafe impl<T: Copy + Send> Send for BoundedQueue<T> {}
unsafe impl<T: Copy + Send> Sync for BoundedQueue<T> {}

impl<T: Copy + Default> BoundedQueue<T> {
    /// Capacity is rounded up to a power of two (minimum 2).
    pub fn new(capacity: usize) -> Self {
        let cap = capacity.max(2).next_power_of_two();
        let slots = (0..cap)
            .map(|i| Slot {
                seq: AtomicUsize::new(i),
                val: UnsafeCell::new(T::default()),
            })
            .collect();
        Self {
            slots,
            mask: cap - 1,
            head: CachePadded(AtomicUsize::new(0)),
            tail: CachePadded(AtomicUsize::new(0)),
        }
    }
}

impl<T: Copy> BoundedQueue<T> {
    pub fn capacity(&self) -> usize {
        self.mask + 1
    }

    /// Enqueue; returns `Err(v)` when full. Lock-free, allocation-free.
    pub fn push(&self, v: T) -> Result<(), T> {
        let mut pos = self.tail.0.load(Ordering::Relaxed);
        loop {
            let slot = &self.slots[pos & self.mask];
            let seq = slot.seq.load(Ordering::Acquire);
            let diff = seq as isize - pos as isize;
            if diff == 0 {
                match self.tail.0.compare_exchange_weak(
                    pos,
                    pos + 1,
                    Ordering::Relaxed,
                    Ordering::Relaxed,
                ) {
                    Ok(_) => {
                        // SAFETY: we own this slot until seq is published below.
                        unsafe { *slot.val.get() = v };
                        slot.seq.store(pos + 1, Ordering::Release);
                        return Ok(());
                    }
                    Err(p) => pos = p,
                }
            } else if diff < 0 {
                return Err(v);
            } else {
                pos = self.tail.0.load(Ordering::Relaxed);
            }
        }
    }

    /// Dequeue; `None` when empty. Lock-free, allocation-free, wait-free for a single consumer
    /// in the absence of a stalled producer.
    pub fn pop(&self) -> Option<T> {
        let mut pos = self.head.0.load(Ordering::Relaxed);
        loop {
            let slot = &self.slots[pos & self.mask];
            let seq = slot.seq.load(Ordering::Acquire);
            let diff = seq as isize - (pos + 1) as isize;
            if diff == 0 {
                match self.head.0.compare_exchange_weak(
                    pos,
                    pos + 1,
                    Ordering::Relaxed,
                    Ordering::Relaxed,
                ) {
                    Ok(_) => {
                        // SAFETY: the producer published this slot (seq == pos+1) and we own it now.
                        let v = unsafe { *slot.val.get() };
                        slot.seq.store(pos + self.mask + 1, Ordering::Release);
                        return Some(v);
                    }
                    Err(p) => pos = p,
                }
            } else if diff < 0 {
                return None;
            } else {
                pos = self.head.0.load(Ordering::Relaxed);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;

    #[test]
    fn fifo_and_full() {
        let q = BoundedQueue::<u32>::new(4);
        for i in 0..4 {
            q.push(i).unwrap();
        }
        assert_eq!(q.push(9), Err(9));
        for i in 0..4 {
            assert_eq!(q.pop(), Some(i));
        }
        assert_eq!(q.pop(), None);
    }

    #[test]
    fn multi_producer() {
        let q = Arc::new(BoundedQueue::<u32>::new(1 << 16));
        let hs: Vec<_> = (0..4)
            .map(|t| {
                let q = q.clone();
                std::thread::spawn(move || {
                    for i in 0..10_000 {
                        q.push(t * 100_000 + i).unwrap();
                    }
                })
            })
            .collect();
        let mut got = Vec::new();
        while got.len() < 40_000 {
            if let Some(v) = q.pop() {
                got.push(v);
            }
        }
        for h in hs {
            h.join().unwrap();
        }
        // per-producer order is preserved
        for t in 0..4 {
            let mine: Vec<_> = got.iter().filter(|&&v| v / 100_000 == t).collect();
            assert!(mine.windows(2).all(|w| w[0] < w[1]));
            assert_eq!(mine.len(), 10_000);
        }
    }
}
