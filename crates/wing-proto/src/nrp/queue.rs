//! Reusable byte FIFO for the streaming decoders.

#[derive(Debug, Default)]
pub(crate) struct ByteQueue {
    pub(crate) buf: Vec<u8>,
    pub(crate) pos: usize,
}

impl ByteQueue {
    #[inline]
    pub(crate) fn push(&mut self, b: u8) {
        self.buf.push(b);
    }

    pub(crate) fn extend(&mut self, bytes: &[u8]) {
        self.buf.extend_from_slice(bytes);
    }

    pub(crate) fn pending(&self) -> &[u8] {
        &self.buf[self.pos..]
    }

    /// Drop consumed bytes; cheap when everything has been consumed (the common case).
    pub(crate) fn compact(&mut self) {
        if self.pos == 0 {
            return;
        }
        if self.pos == self.buf.len() {
            self.buf.clear();
            self.pos = 0;
        } else if self.pos >= 4096 && self.pos * 2 >= self.buf.len() {
            self.buf.drain(..self.pos);
            self.pos = 0;
        }
    }
}
