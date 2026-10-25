//! `#[repr(C)]` mirror of `driver/include/wing_shm.h` (version 2).
//!
//! Every offset is pinned by const asserts below (mirroring the C header's
//! static asserts), and `tests/layout.rs` compiles the C header and compares
//! every field offset and constant with this file.

use std::sync::atomic::{AtomicI64, AtomicU32, AtomicU64};

/// POSIX shm object name (leading '/', at most 31 chars on macOS).
pub const SHM_NAME: &str = "/wingsim.audio.v1";
/// `'WING'` little-endian.
pub const MAGIC: u32 = 0x474E_4957;
/// Layout version; bump on any layout or semantic change (both languages).
pub const VERSION: u32 = 2;
/// Channels per direction.
pub const CHANNELS: usize = 48;
/// Ring length in frames (power of two, ~170 ms at 48 kHz).
pub const RING_FRAMES: usize = 8192;
/// Bytes reserved for the header before the first ring.
pub const HEADER_BYTES: usize = 4096;
/// Apple silicon cache line; sections written by different processes are
/// on separate lines.
pub const CACHE_LINE: usize = 128;
/// Default `input_delay_frames` (reported to the HAL as input latency).
pub const DEFAULT_INPUT_DELAY_FRAMES: u32 = 256;
/// The driver treats the daemon as gone if its heartbeat is older than this.
pub const HEARTBEAT_TIMEOUT_NS: u64 = 100_000_000;
/// Frames at the old end of a ring a consumer must not read.
pub const RING_GUARD_FRAMES: usize = RING_FRAMES / 4;
/// Bytes of one planar ring (`CHANNELS * RING_FRAMES` f32).
pub const RING_BYTES: usize = CHANNELS * RING_FRAMES * 4;
/// Total segment size.
pub const TOTAL_BYTES: usize = HEADER_BYTES + 2 * RING_BYTES;
/// Byte offset of the driver -> daemon ring.
pub const TO_DAEMON_OFFSET: usize = HEADER_BYTES;
/// Byte offset of the daemon -> driver ring.
pub const FROM_DAEMON_OFFSET: usize = HEADER_BYTES + RING_BYTES;

/// One ring's control block: producer line + consumer line.
#[repr(C)]
pub struct RingCtl {
    /// Producer: exclusive end sample time of valid data (release store).
    pub write_time: AtomicI64,
    /// Producer: `io_epoch` the data belongs to.
    pub epoch: AtomicU32,
    _pad0: u32,
    _pad_p: [u8; CACHE_LINE - 16],
    /// Consumer: progress (informational).
    pub read_time: AtomicI64,
    /// Consumer fell more than a ring behind (incremented by the consumer).
    pub overruns: AtomicU32,
    /// Consumer wanted data not yet written (incremented by the consumer).
    pub underruns: AtomicU32,
    _pad_c: [u8; CACHE_LINE - 16],
}

/// `wing_shm_header`.
#[repr(C)]
pub struct Header {
    // --- immutable after creation (magic published last) ---
    /// [`MAGIC`], release-stored after the rest of the header.
    pub magic: AtomicU32,
    pub version: u32,
    pub header_bytes: u32,
    pub channels: u32,
    pub ring_frames: u32,
    pub input_delay_frames: u32,
    pub total_bytes: u64,
    pub to_daemon_offset: u64,
    pub from_daemon_offset: u64,
    /// Unique per creation; lets a peer detect that the name was recreated.
    pub segment_id: u64,
    pub creator_pid: u64,
    _pad_a: [u8; CACHE_LINE - 64],

    // --- driver-owned ---
    pub io_epoch: AtomicU32,
    pub sample_rate: AtomicU32,
    pub io_running: AtomicU32,
    pub loopback_active: AtomicU32,
    /// Incremented (seq_cst) whenever there is new work; os_sync wake address.
    pub cycle_seq: AtomicU32,
    /// Frames of the latest ReadInput (the HAL IO buffer size).
    pub input_frames: AtomicU32,
    /// `mach_absolute_time()` of the last IO cycle.
    pub driver_heartbeat: AtomicU64,
    pub driver_pid: AtomicU64,
    /// Exclusive end device sample time of the latest ReadInput.
    pub input_time: AtomicI64,
    _pad_b: [u8; CACHE_LINE - 48],

    // --- daemon-owned ---
    /// `mach_absolute_time()`; 0 = detached.
    pub daemon_heartbeat: AtomicU64,
    pub daemon_pid: AtomicU64,
    /// Non-zero while the daemon is blocked in an os_sync wait.
    pub daemon_waiting: AtomicU32,
    _pad_d: u32,
    _pad_c: [u8; CACHE_LINE - 24],

    pub to_daemon: RingCtl,
    pub from_daemon: RingCtl,
}

/// Mirrors `wing_shm_validate_header` return codes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LayoutError {
    /// Segment smaller than [`TOTAL_BYTES`] (-1).
    TooSmall,
    /// Magic is not [`MAGIC`] (-2); zero means possibly still being created.
    BadMagic(u32),
    /// Version mismatch (-3).
    Version(u32),
    /// Channel count or ring length mismatch (-4).
    Geometry,
    /// Total/header size mismatch (-5).
    Size,
    /// Ring offsets mismatch (-6).
    Offsets,
}

impl Header {
    /// Rust equivalent of `wing_shm_validate_header`.
    pub fn validate(&self, mapped_bytes: u64) -> Result<(), LayoutError> {
        use std::sync::atomic::Ordering::Acquire;
        if mapped_bytes < TOTAL_BYTES as u64 {
            return Err(LayoutError::TooSmall);
        }
        let magic = self.magic.load(Acquire);
        if magic != MAGIC {
            return Err(LayoutError::BadMagic(magic));
        }
        if self.version != VERSION {
            return Err(LayoutError::Version(self.version));
        }
        if self.channels as usize != CHANNELS || self.ring_frames as usize != RING_FRAMES {
            return Err(LayoutError::Geometry);
        }
        if self.total_bytes != TOTAL_BYTES as u64 || self.header_bytes as usize != HEADER_BYTES {
            return Err(LayoutError::Size);
        }
        if self.to_daemon_offset != TO_DAEMON_OFFSET as u64
            || self.from_daemon_offset != FROM_DAEMON_OFFSET as u64
        {
            return Err(LayoutError::Offsets);
        }
        Ok(())
    }

    /// Rust equivalent of `wing_shm_init_header`.
    ///
    /// # Safety
    /// `h` must point to a zeroed, writable mapping of at least
    /// [`HEADER_BYTES`] that no other process has validated yet.
    pub unsafe fn init(h: *mut Header, sample_rate: u32, segment_id: u64, creator_pid: u64) {
        use std::ptr::addr_of_mut;
        use std::sync::atomic::Ordering::{Relaxed, Release};
        addr_of_mut!((*h).version).write(VERSION);
        addr_of_mut!((*h).header_bytes).write(HEADER_BYTES as u32);
        addr_of_mut!((*h).channels).write(CHANNELS as u32);
        addr_of_mut!((*h).ring_frames).write(RING_FRAMES as u32);
        addr_of_mut!((*h).input_delay_frames).write(DEFAULT_INPUT_DELAY_FRAMES);
        addr_of_mut!((*h).total_bytes).write(TOTAL_BYTES as u64);
        addr_of_mut!((*h).to_daemon_offset).write(TO_DAEMON_OFFSET as u64);
        addr_of_mut!((*h).from_daemon_offset).write(FROM_DAEMON_OFFSET as u64);
        addr_of_mut!((*h).segment_id).write(segment_id);
        addr_of_mut!((*h).creator_pid).write(creator_pid);
        (*h).sample_rate.store(sample_rate, Relaxed);
        (*h).magic.store(MAGIC, Release);
    }
}

// Mirrors the static asserts in wing_shm.h.
const _: () = {
    use std::mem::{offset_of, size_of};
    assert!(size_of::<RingCtl>() == 256);
    assert!(offset_of!(RingCtl, read_time) == 128);
    assert!(offset_of!(Header, segment_id) == 48);
    assert!(offset_of!(Header, io_epoch) == 128);
    assert!(offset_of!(Header, cycle_seq) == 144);
    assert!(offset_of!(Header, input_time) == 168);
    assert!(offset_of!(Header, daemon_heartbeat) == 256);
    assert!(offset_of!(Header, daemon_waiting) == 272);
    assert!(offset_of!(Header, to_daemon) == 384);
    assert!(offset_of!(Header, from_daemon) == 640);
    assert!(size_of::<Header>() == 896);
    assert!(size_of::<Header>() <= HEADER_BYTES);
    assert!(RING_FRAMES.is_power_of_two());
    assert!(SHM_NAME.len() <= 31);
};
