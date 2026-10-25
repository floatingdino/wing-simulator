//! Lock-free, allocation-free (RT side) hand-over of immutable tables from a control thread to
//! RT readers.
//!
//! Each reader owns one [`Mailbox`] with two pointer slots: `pending` (control -> RT, the newest
//! unpicked table) and `retired` (RT -> control, the table the reader replaced). The RT side
//! only moves `Box` pointers ([`Box::from_raw`] / [`Box::into_raw`], no heap traffic); every
//! allocation and every free happens on the control thread in [`TableSender::publish`] /
//! [`TableSender::collect`]. A reader whose `retired` slot is still occupied keeps its current
//! table until the control side collects (at the latest on the next publish).

use std::ptr;
use std::sync::atomic::{AtomicPtr, Ordering::*};
use std::sync::Arc;

struct Mailbox<T> {
    pending: AtomicPtr<T>,
    retired: AtomicPtr<T>,
}

impl<T> Mailbox<T> {
    fn new() -> Self {
        Self {
            pending: AtomicPtr::new(ptr::null_mut()),
            retired: AtomicPtr::new(ptr::null_mut()),
        }
    }

    /// Control side: free the reader's retired table.
    fn collect(&self) {
        let p = self.retired.swap(ptr::null_mut(), AcqRel);
        if !p.is_null() {
            // SAFETY: produced by Box::into_raw in `TableReader::update`; ownership moved to us
            // by the swap.
            drop(unsafe { Box::from_raw(p) });
        }
    }
}

impl<T> Drop for Mailbox<T> {
    fn drop(&mut self) {
        for slot in [&self.pending, &self.retired] {
            let p = slot.swap(ptr::null_mut(), AcqRel);
            if !p.is_null() {
                // SAFETY: both slots only ever hold pointers from Box::into_raw.
                drop(unsafe { Box::from_raw(p) });
            }
        }
    }
}

/// Control-side publisher fanning one table out to any number of readers.
pub struct TableSender<T: Clone + Send> {
    latest: T,
    boxes: Vec<Arc<Mailbox<T>>>,
}

impl<T: Clone + Send> TableSender<T> {
    pub fn new(initial: T) -> Self {
        Self {
            latest: initial,
            boxes: Vec::new(),
        }
    }

    /// The most recently published table.
    pub fn latest(&self) -> &T {
        &self.latest
    }

    /// A new reader starting at the latest table (control thread; allocates).
    pub fn subscribe(&mut self) -> TableReader<T> {
        let mb = Arc::new(Mailbox::new());
        self.boxes.push(mb.clone());
        TableReader {
            mb,
            current: Box::new(self.latest.clone()),
        }
    }

    /// Publish a new table to every reader (control thread; allocates one copy per reader and
    /// frees tables the readers have retired or never picked up).
    pub fn publish(&mut self, table: T) {
        for mb in &self.boxes {
            mb.collect();
            let p = Box::into_raw(Box::new(table.clone()));
            let old = mb.pending.swap(p, AcqRel);
            if !old.is_null() {
                // SAFETY: an unpicked pending table; the reader never saw it.
                drop(unsafe { Box::from_raw(old) });
            }
        }
        self.latest = table;
    }

    /// Free retired tables (call periodically on the control thread).
    pub fn collect(&self) {
        for mb in &self.boxes {
            mb.collect();
        }
    }

    /// Whether every reader runs the latest table (nothing pending).
    pub fn settled(&self) -> bool {
        self.boxes
            .iter()
            .all(|mb| mb.pending.load(Acquire).is_null())
    }
}

/// RT-side reader: owns its current table and adopts new ones in [`TableReader::update`].
pub struct TableReader<T> {
    mb: Arc<Mailbox<T>>,
    current: Box<T>,
}

// SAFETY: the reader owns `current` and exchanges Box<T> with the control side through atomics.
unsafe impl<T: Send> Send for TableReader<T> {}

impl<T> TableReader<T> {
    /// Adopt the newest published table, if any. Wait-free, no allocation or free. Returns
    /// whether the table changed.
    #[inline]
    pub fn update(&mut self) -> bool {
        if !self.mb.retired.load(Acquire).is_null() {
            return false; // control side has not collected the previous one yet
        }
        let p = self.mb.pending.swap(ptr::null_mut(), AcqRel);
        if p.is_null() {
            return false;
        }
        // SAFETY: produced by Box::into_raw in `TableSender::publish`; the swap moved ownership.
        let new = unsafe { Box::from_raw(p) };
        let old = std::mem::replace(&mut self.current, new);
        // Only this reader stores non-null into `retired`, and it was null above.
        self.mb.retired.store(Box::into_raw(old), Release);
        true
    }

    /// The current table.
    #[inline]
    pub fn get(&self) -> &T {
        &self.current
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn publish_update_collect() {
        let mut tx = TableSender::new(vec![0u32; 4]);
        let mut a = tx.subscribe();
        let mut b = tx.subscribe();
        assert!(!a.update());
        assert_eq!(a.get()[0], 0);
        tx.publish(vec![1; 4]);
        assert!(!tx.settled());
        assert!(a.update());
        assert_eq!(a.get()[0], 1);
        // a has a retired table: a further publish is deferred until collect.
        tx.publish(vec![2; 4]);
        assert!(a.update()); // publish collected first
        assert_eq!(a.get()[0], 2);
        tx.publish(vec![3; 4]);
        tx.publish(vec![4; 4]); // replaces unpicked 3
        assert!(b.update());
        assert_eq!(b.get()[0], 4);
        assert!(a.update());
        assert_eq!(a.get()[0], 4);
        assert!(!a.update());
        assert!(tx.settled());
        tx.collect();
        assert_eq!(tx.latest()[0], 4);
    }

    #[test]
    fn concurrent_swaps_are_consistent() {
        let mut tx = TableSender::new([0u64; 32]);
        let mut rx = tx.subscribe();
        let stop = Arc::new(std::sync::atomic::AtomicBool::new(false));
        let s2 = stop.clone();
        let reader = std::thread::spawn(move || {
            let mut last = 0;
            let mut swaps = 0;
            while !s2.load(Relaxed) {
                if rx.update() {
                    swaps += 1;
                }
                let t = rx.get();
                assert!(t.iter().all(|&x| x == t[0]), "torn table");
                assert!(t[0] >= last);
                last = t[0];
            }
            swaps
        });
        for i in 1..=20_000u64 {
            tx.publish([i; 32]);
        }
        while !tx.settled() {
            tx.collect();
            std::hint::spin_loop();
        }
        stop.store(true, Relaxed);
        assert!(reader.join().unwrap() > 0);
    }
}
