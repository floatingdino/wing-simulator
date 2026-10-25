/*
 * wing_shm.h - shared-memory layout between the Wing Simulator HAL driver
 * (runs inside Apple's sandboxed Core Audio driver host) and the userland
 * wing-sim daemon.
 *
 * Version 2 (T020). The Rust mirror is crates/wing-shm/src/layout.rs. Both
 * sides pin every offset with static asserts, and crates/wing-shm/tests/
 * layout.rs compiles this header and compares all offsets and constants with
 * the Rust definition. Bump WING_SHM_VERSION on ANY layout or semantic change.
 *
 * Transport: POSIX shared memory object WING_SHM_NAME (shm_open + mmap).
 * See docs/research/driver.md for why this (and not a /tmp file, named
 * semaphores or Mach services) is what the coreaudiod sandbox allows. The
 * name keeps its ".v1" suffix; compatibility is decided by `version`, and a
 * segment with another version is unlinked and recreated by the driver.
 *
 * Layout (all little-endian, natural alignment, offsets fixed). Sections that
 * are written by different processes live on separate 128-byte lines (the
 * Apple silicon cache line), so driver and daemon never false-share:
 *
 *   [   0, 128)  immutable after creation
 *   [ 128, 256)  driver-owned   [atomic]
 *   [ 256, 384)  daemon-owned   [atomic]
 *   [ 384, 640)  to_daemon   ring ctl (producer line, consumer line)
 *   [ 640, 896)  from_daemon ring ctl (producer line, consumer line)
 *   [ 896, WING_SHM_HEADER_BYTES)                  zero
 *   [to_daemon_offset,   +WING_SHM_RING_BYTES)    float to_daemon  [channels][ring_frames]
 *   [from_daemon_offset, +WING_SHM_RING_BYTES)    float from_daemon[channels][ring_frames]
 *
 * Rings are planar float32 and indexed by ABSOLUTE DEVICE SAMPLE TIME:
 * frame for sample time t of channel c lives at ring[c][t & (ring_frames-1)].
 * Each ring has exactly one producer which publishes `write_time` (exclusive
 * end sample time of valid data, release store) after writing samples.
 *
 *   to_daemon   producer = driver (DAW playback mix, WING "USB in" 1..48)
 *   from_daemon producer = daemon (WING "USB out" 1..48, recorded by DAW)
 *
 * Creation: the creator (driver, or a daemon started first) ftruncates, fills
 * the header and publishes `magic` LAST with a release store. An attacher
 * that sees magic == 0 on a correctly sized segment is racing a creator and
 * should retry for a few ms before treating the segment as incompatible.
 * `segment_id` is unique per creation so a long-lived peer can detect that
 * the name now refers to a different (recreated) segment.
 *
 * Epochs: sample time restarts whenever the driver (re)starts IO or changes
 * rate; the driver then resets to_daemon.write_time/input_time and increments
 * `io_epoch` (release). Data in from_daemon is only used by the driver when
 * from_daemon.epoch == io_epoch.
 *
 * Timing contract: the driver reads input for device sample time T from
 * from_daemon at (T - input_delay_frames). input_delay_frames is reported to
 * the HAL as input stream latency so DAWs compensate. It gives the daemon a
 * processing window of roughly (IO buffer + input_delay_frames) frames.
 * The creator chooses input_delay_frames (the driver: its configured value,
 * T021). A driver whose configured delay differs from an existing segment's
 * treats the segment as incompatible and recreates it, exactly like a version
 * mismatch, so the value stays immutable for the segment's lifetime.
 *
 * Fallback (T021): when the daemon is dead (heartbeat), on another epoch, or
 * its from_daemon data for a block is late, the driver crossfades to loopback
 * and back; it only uses from_daemon data written contiguously since it last
 * saw the daemon (re)connect (pid change, write_time going backwards or
 * jumping) so stale ring content is never played.
 *
 * Record-only clients: if the HAL runs input IO without output IO,
 * to_daemon.write_time does not advance. The driver publishes `input_time`
 * (exclusive end device sample time of its latest ReadInput) and
 * `input_frames` (that call's size). When input_time > to_daemon.write_time
 * the daemon treats input past write_time as silence and produces from_daemon
 * up to (input_time - input_delay_frames + 2 * input_frames).
 *
 * Wake-up: the driver increments `cycle_seq` (seq_cst) whenever there is new
 * work for the daemon (after WriteOutput; after ReadInput only in record-only
 * mode; on every new epoch). If `daemon_waiting` is non-zero it then calls
 * os_sync_wake_by_address_all(&cycle_seq, 4, OS_SYNC_WAKE_BY_ADDRESS_SHARED)
 * (macOS 14.4+). The daemon increments daemon_waiting (seq_cst), re-checks
 * cycle_seq and blocks in os_sync_wait_on_address_with_timeout(..SHARED..).
 * POSIX named semaphores are denied by the coreaudiod sandbox; os_sync is not
 * a sandbox operation. Without os_sync the daemon falls back to polling
 * cycle_seq with nanosleep. See docs/research/driver.md section 4.
 *
 * Liveness: the daemon stores mach_absolute_time() into daemon_heartbeat at
 * least every few ms while running. If it is older than
 * WING_SHM_HEARTBEAT_TIMEOUT_NS (or zero) the driver acts as a loopback:
 * input(T) = to_daemon(T - input_delay_frames).
 *
 * Atomics: fields marked [atomic] must only be accessed with the helpers
 * below (__atomic builtins, usable from C and C++; Rust uses AtomicU64/
 * AtomicU32 on the same addresses). Everything else is written once at
 * creation, before `magic` is published.
 */
#ifndef WING_SHM_H
#define WING_SHM_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* POSIX shm names on macOS: leading '/', max 31 chars (PSHMNAMLEN). */
#define WING_SHM_NAME "/wingsim.audio.v1"

#define WING_SHM_MAGIC 0x474E4957u /* 'WING' little-endian */
#define WING_SHM_VERSION 2u

#define WING_SHM_CHANNELS 48u
#define WING_SHM_RING_FRAMES 8192u /* power of two; ~170 ms at 48 kHz */
#define WING_SHM_HEADER_BYTES 4096u
#define WING_SHM_CACHE_LINE 128u
#define WING_SHM_DEFAULT_INPUT_DELAY_FRAMES 128u /* T021: see driver.md 9.2 */
#define WING_SHM_HEARTBEAT_TIMEOUT_NS 100000000ull /* 100 ms */
/* Frames at the old end of a ring a consumer must not read: the producer may
 * already be overwriting them. */
#define WING_SHM_RING_GUARD_FRAMES (WING_SHM_RING_FRAMES / 4u)

#define WING_SHM_RING_BYTES \
    ((uint64_t)WING_SHM_CHANNELS * WING_SHM_RING_FRAMES * sizeof(float))
#define WING_SHM_TOTAL_BYTES \
    ((uint64_t)WING_SHM_HEADER_BYTES + 2u * WING_SHM_RING_BYTES)

/* One ring's control block: producer line + consumer line. */
typedef struct wing_shm_ring_ctl {
    /* producer-owned line */
    int64_t write_time;   /* [atomic] exclusive end sample time of data      */
    uint32_t epoch;       /* [atomic] io_epoch the data belongs to           */
    uint32_t _pad0;
    uint8_t _pad_p[WING_SHM_CACHE_LINE - 16];
    /* consumer-owned line */
    int64_t read_time;    /* [atomic] consumer progress (informational)      */
    uint32_t overruns;    /* [atomic] consumer fell > ring behind            */
    uint32_t underruns;   /* [atomic] consumer wanted data not yet written   */
    uint8_t _pad_c[WING_SHM_CACHE_LINE - 16];
} wing_shm_ring_ctl;

typedef struct wing_shm_header {
    /* --- immutable after creation (written by creator) --- */
    uint32_t magic;              /* [atomic] WING_SHM_MAGIC, published last   */
    uint32_t version;            /* WING_SHM_VERSION                          */
    uint32_t header_bytes;       /* WING_SHM_HEADER_BYTES                     */
    uint32_t channels;           /* WING_SHM_CHANNELS (same both directions)  */
    uint32_t ring_frames;        /* WING_SHM_RING_FRAMES                      */
    uint32_t input_delay_frames; /* see timing contract                       */
    uint64_t total_bytes;        /* WING_SHM_TOTAL_BYTES                      */
    uint64_t to_daemon_offset;   /* byte offset of to_daemon ring             */
    uint64_t from_daemon_offset; /* byte offset of from_daemon ring           */
    uint64_t segment_id;         /* unique per creation (creator's clock)     */
    uint64_t creator_pid;        /* pid of the process that created it        */
    uint8_t _pad_a[WING_SHM_CACHE_LINE - 64];

    /* --- driver-owned [atomic] --- */
    uint32_t io_epoch;           /* ++ on StartIO / sample-rate change        */
    uint32_t sample_rate;        /* current nominal rate (44100 / 48000)      */
    uint32_t io_running;         /* 1 while HAL IO is running                 */
    uint32_t loopback_active;    /* 1 while driver is in loopback mode        */
    uint32_t cycle_seq;          /* ++ when there is new work (wake address)  */
    uint32_t input_frames;       /* frames of the latest ReadInput            */
    uint64_t driver_heartbeat;   /* mach_absolute_time() of last IO cycle     */
    uint64_t driver_pid;         /* pid of Core Audio driver host             */
    int64_t input_time;          /* end device time of latest ReadInput       */
    uint8_t _pad_b[WING_SHM_CACHE_LINE - 48];

    /* --- daemon-owned [atomic] --- */
    uint64_t daemon_heartbeat;   /* mach_absolute_time(); 0 = detached        */
    uint64_t daemon_pid;
    uint32_t daemon_waiting;     /* >0 while blocked in os_sync wait          */
    uint32_t _pad_d;
    uint8_t _pad_c[WING_SHM_CACHE_LINE - 24];

    wing_shm_ring_ctl to_daemon;
    wing_shm_ring_ctl from_daemon;
} wing_shm_header;

/* Mirrored by const asserts in crates/wing-shm/src/layout.rs. */
#if defined(__cplusplus)
#define WING_SHM_STATIC_ASSERT static_assert
#else
#define WING_SHM_STATIC_ASSERT _Static_assert
#endif
WING_SHM_STATIC_ASSERT(sizeof(wing_shm_ring_ctl) == 256, "ring ctl layout");
WING_SHM_STATIC_ASSERT(offsetof(wing_shm_ring_ctl, read_time) == 128, "ring ctl layout");
WING_SHM_STATIC_ASSERT(offsetof(wing_shm_header, segment_id) == 48, "header layout");
WING_SHM_STATIC_ASSERT(offsetof(wing_shm_header, io_epoch) == 128, "header layout");
WING_SHM_STATIC_ASSERT(offsetof(wing_shm_header, cycle_seq) == 144, "header layout");
WING_SHM_STATIC_ASSERT(offsetof(wing_shm_header, input_time) == 168, "header layout");
WING_SHM_STATIC_ASSERT(offsetof(wing_shm_header, daemon_heartbeat) == 256, "header layout");
WING_SHM_STATIC_ASSERT(offsetof(wing_shm_header, daemon_waiting) == 272, "header layout");
WING_SHM_STATIC_ASSERT(offsetof(wing_shm_header, to_daemon) == 384, "header layout");
WING_SHM_STATIC_ASSERT(offsetof(wing_shm_header, from_daemon) == 640, "header layout");
WING_SHM_STATIC_ASSERT(sizeof(wing_shm_header) == 896, "header layout");
WING_SHM_STATIC_ASSERT(sizeof(wing_shm_header) <= WING_SHM_HEADER_BYTES, "header fits");
WING_SHM_STATIC_ASSERT((WING_SHM_RING_FRAMES & (WING_SHM_RING_FRAMES - 1u)) == 0, "pow2 ring");

/* ---- atomic helpers (lock-free on arm64/x86_64) ---- */
#define WING_LOAD_ACQ(p) __atomic_load_n((p), __ATOMIC_ACQUIRE)
#define WING_LOAD_RLX(p) __atomic_load_n((p), __ATOMIC_RELAXED)
#define WING_LOAD_SEQ(p) __atomic_load_n((p), __ATOMIC_SEQ_CST)
#define WING_STORE_REL(p, v) __atomic_store_n((p), (v), __ATOMIC_RELEASE)
#define WING_STORE_RLX(p, v) __atomic_store_n((p), (v), __ATOMIC_RELAXED)
#define WING_INC_RLX(p) __atomic_fetch_add((p), 1, __ATOMIC_RELAXED)
#define WING_INC_SEQ(p) __atomic_fetch_add((p), 1, __ATOMIC_SEQ_CST)

static inline float* wing_shm_ring(void* base, uint64_t offset)
{
    return (float*)((uint8_t*)base + offset);
}

static inline uint32_t wing_shm_ring_index(int64_t t, uint32_t ring_frames)
{
    return (uint32_t)((uint64_t)t & (uint64_t)(ring_frames - 1u));
}

/* Fill a freshly created (zeroed, ftruncate'd) segment's header. `magic` is
 * published last (release) so attachers never see a half-written header. */
static inline void wing_shm_init_header_delay(wing_shm_header* h,
    uint32_t sample_rate,
    uint64_t segment_id,
    uint64_t creator_pid,
    uint32_t input_delay_frames)
{
    h->version = WING_SHM_VERSION;
    h->header_bytes = WING_SHM_HEADER_BYTES;
    h->channels = WING_SHM_CHANNELS;
    h->ring_frames = WING_SHM_RING_FRAMES;
    h->input_delay_frames = input_delay_frames;
    h->total_bytes = WING_SHM_TOTAL_BYTES;
    h->to_daemon_offset = WING_SHM_HEADER_BYTES;
    h->from_daemon_offset = WING_SHM_HEADER_BYTES + WING_SHM_RING_BYTES;
    h->segment_id = segment_id;
    h->creator_pid = creator_pid;
    WING_STORE_RLX(&h->sample_rate, sample_rate);
    WING_STORE_REL(&h->magic, WING_SHM_MAGIC);
}

static inline void wing_shm_init_header(wing_shm_header* h,
    uint32_t sample_rate,
    uint64_t segment_id,
    uint64_t creator_pid)
{
    wing_shm_init_header_delay(
        h, sample_rate, segment_id, creator_pid, WING_SHM_DEFAULT_INPUT_DELAY_FRAMES);
}

/* Returns 0 if a mapped header is compatible with this build.
 * -2 with magic == 0 means "possibly still being created": retry briefly. */
static inline int wing_shm_validate_header(const wing_shm_header* h, uint64_t mapped_bytes)
{
    if (mapped_bytes < WING_SHM_TOTAL_BYTES) return -1;
    if (WING_LOAD_ACQ(&h->magic) != WING_SHM_MAGIC) return -2;
    if (h->version != WING_SHM_VERSION) return -3;
    if (h->channels != WING_SHM_CHANNELS || h->ring_frames != WING_SHM_RING_FRAMES) return -4;
    if (h->total_bytes != WING_SHM_TOTAL_BYTES || h->header_bytes != WING_SHM_HEADER_BYTES)
        return -5;
    if (h->to_daemon_offset != WING_SHM_HEADER_BYTES ||
        h->from_daemon_offset != WING_SHM_HEADER_BYTES + WING_SHM_RING_BYTES)
        return -6;
    return 0;
}

#ifdef __cplusplus
}
#endif

#endif /* WING_SHM_H */
