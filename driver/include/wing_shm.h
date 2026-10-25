/*
 * wing_shm.h - shared-memory layout between the Wing Simulator HAL driver
 * (runs inside Apple's sandboxed Core Audio driver host) and the userland
 * wing-sim daemon.
 *
 * Status: spike layout (T006). T020 productionizes it and must keep the Rust
 * definition in crates/wing-shm byte-for-byte identical (bump WING_SHM_VERSION
 * on any change).
 *
 * Transport: POSIX shared memory object WING_SHM_NAME (shm_open + mmap).
 * See docs/research/driver.md for why this (and not a /tmp file, named
 * semaphores or Mach services) is what the coreaudiod sandbox allows.
 *
 * Layout (all little-endian, natural alignment, offsets fixed):
 *
 *   [0, WING_SHM_HEADER_BYTES)           wing_shm_header
 *   [to_daemon_offset,   +ring_bytes)    float to_daemon  [channels][ring_frames]
 *   [from_daemon_offset, +ring_bytes)    float from_daemon[channels][ring_frames]
 *
 * Rings are planar float32 and indexed by ABSOLUTE DEVICE SAMPLE TIME:
 * frame for sample time t of channel c lives at ring[c][t & (ring_frames-1)].
 * Each ring has exactly one producer which publishes `write_time` (exclusive
 * end sample time of valid data, release store) after writing samples.
 *
 *   to_daemon   producer = driver (DAW playback mix, WING "USB in" 1..48)
 *   from_daemon producer = daemon (WING "USB out" 1..48, recorded by DAW)
 *
 * Sample time restarts whenever the driver (re)starts IO or changes rate; the
 * driver then increments `io_epoch`. Data in from_daemon is only used by the
 * driver when from_daemon.epoch == io_epoch.
 *
 * Timing contract: the driver reads input for device sample time T from
 * from_daemon at (T - input_delay_frames). input_delay_frames is reported to
 * the HAL as input stream latency so DAWs compensate. It gives the daemon a
 * processing window of roughly (IO buffer + input_delay_frames) frames.
 *
 * Liveness: the daemon stores mach_absolute_time() into daemon_heartbeat at
 * least every few ms while running. If it is older than
 * WING_SHM_HEARTBEAT_TIMEOUT_NS (or zero) the driver acts as a loopback:
 * input(T) = to_daemon(T - input_delay_frames).
 *
 * Atomics: fields marked [atomic] must only be accessed with the helpers
 * below (__atomic builtins, usable from C and C++; Rust uses AtomicU64/
 * AtomicU32 on the same addresses). Everything else is written once at
 * creation.
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
#define WING_SHM_VERSION 1u

#define WING_SHM_CHANNELS 48u
#define WING_SHM_RING_FRAMES 8192u /* power of two; ~170 ms at 48 kHz */
#define WING_SHM_HEADER_BYTES 4096u
#define WING_SHM_DEFAULT_INPUT_DELAY_FRAMES 256u
#define WING_SHM_HEARTBEAT_TIMEOUT_NS 100000000ull /* 100 ms */

#define WING_SHM_RING_BYTES \
    ((uint64_t)WING_SHM_CHANNELS * WING_SHM_RING_FRAMES * sizeof(float))
#define WING_SHM_TOTAL_BYTES \
    ((uint64_t)WING_SHM_HEADER_BYTES + 2u * WING_SHM_RING_BYTES)

/* One ring's control block; padded to its own cache lines. */
typedef struct wing_shm_ring_ctl {
    int64_t write_time;   /* [atomic] producer: exclusive end sample time   */
    int64_t read_time;    /* [atomic] consumer: progress (informational)    */
    uint32_t epoch;       /* [atomic] producer: io_epoch the data belongs to */
    uint32_t overruns;    /* [atomic] consumer fell > ring behind           */
    uint32_t underruns;   /* [atomic] consumer wanted data not yet written  */
    uint32_t _pad0;
    uint8_t _pad[128 - 32];
} wing_shm_ring_ctl;

typedef struct wing_shm_header {
    /* --- immutable after creation (written by creator) --- */
    uint32_t magic;              /* WING_SHM_MAGIC                            */
    uint32_t version;            /* WING_SHM_VERSION                          */
    uint32_t header_bytes;       /* WING_SHM_HEADER_BYTES                     */
    uint32_t channels;           /* WING_SHM_CHANNELS (same both directions)  */
    uint32_t ring_frames;        /* WING_SHM_RING_FRAMES                      */
    uint32_t input_delay_frames; /* see timing contract                       */
    uint64_t total_bytes;        /* WING_SHM_TOTAL_BYTES                      */
    uint64_t to_daemon_offset;   /* byte offset of to_daemon ring             */
    uint64_t from_daemon_offset; /* byte offset of from_daemon ring           */
    uint8_t _pad_a[64 - 48];

    /* --- driver-owned [atomic] --- */
    uint32_t io_epoch;           /* ++ on StartIO / sample-rate change        */
    uint32_t sample_rate;        /* current nominal rate (44100 / 48000)      */
    uint32_t io_running;         /* 1 while HAL IO is running                 */
    uint32_t loopback_active;    /* 1 while driver is in loopback mode        */
    uint64_t driver_heartbeat;   /* mach_absolute_time() of last IO cycle     */
    uint64_t driver_pid;         /* pid of Core Audio driver host             */
    uint8_t _pad_b[64 - 32];

    /* --- daemon-owned [atomic] --- */
    uint64_t daemon_heartbeat;   /* mach_absolute_time(); 0 = detached        */
    uint64_t daemon_pid;
    uint8_t _pad_c[64 - 16];

    wing_shm_ring_ctl to_daemon;
    wing_shm_ring_ctl from_daemon;
} wing_shm_header;

#if defined(__cplusplus)
static_assert(sizeof(wing_shm_ring_ctl) == 128, "ring ctl layout");
static_assert(offsetof(wing_shm_header, io_epoch) == 64, "header layout");
static_assert(offsetof(wing_shm_header, daemon_heartbeat) == 128, "header layout");
static_assert(offsetof(wing_shm_header, to_daemon) == 192, "header layout");
static_assert(sizeof(wing_shm_header) == 448, "header layout");
static_assert(sizeof(wing_shm_header) <= WING_SHM_HEADER_BYTES, "header fits");
#else
_Static_assert(sizeof(wing_shm_ring_ctl) == 128, "ring ctl layout");
_Static_assert(sizeof(wing_shm_header) == 448, "header layout");
#endif

/* ---- atomic helpers (lock-free on arm64/x86_64) ---- */
#define WING_LOAD_ACQ(p) __atomic_load_n((p), __ATOMIC_ACQUIRE)
#define WING_LOAD_RLX(p) __atomic_load_n((p), __ATOMIC_RELAXED)
#define WING_STORE_REL(p, v) __atomic_store_n((p), (v), __ATOMIC_RELEASE)
#define WING_STORE_RLX(p, v) __atomic_store_n((p), (v), __ATOMIC_RELAXED)
#define WING_INC_RLX(p) __atomic_fetch_add((p), 1, __ATOMIC_RELAXED)

static inline float* wing_shm_ring(void* base, uint64_t offset)
{
    return (float*)((uint8_t*)base + offset);
}

static inline uint32_t wing_shm_ring_index(int64_t t, uint32_t ring_frames)
{
    return (uint32_t)((uint64_t)t & (uint64_t)(ring_frames - 1u));
}

/* Fill a freshly created (zeroed, ftruncate'd) segment's header. */
static inline void wing_shm_init_header(wing_shm_header* h, uint32_t sample_rate)
{
    h->magic = WING_SHM_MAGIC;
    h->version = WING_SHM_VERSION;
    h->header_bytes = WING_SHM_HEADER_BYTES;
    h->channels = WING_SHM_CHANNELS;
    h->ring_frames = WING_SHM_RING_FRAMES;
    h->input_delay_frames = WING_SHM_DEFAULT_INPUT_DELAY_FRAMES;
    h->total_bytes = WING_SHM_TOTAL_BYTES;
    h->to_daemon_offset = WING_SHM_HEADER_BYTES;
    h->from_daemon_offset = WING_SHM_HEADER_BYTES + WING_SHM_RING_BYTES;
    WING_STORE_RLX(&h->sample_rate, sample_rate);
}

/* Returns 0 if a mapped header is compatible with this build. */
static inline int wing_shm_validate_header(const wing_shm_header* h, uint64_t mapped_bytes)
{
    if (mapped_bytes < WING_SHM_TOTAL_BYTES) return -1;
    if (h->magic != WING_SHM_MAGIC) return -2;
    if (h->version != WING_SHM_VERSION) return -3;
    if (h->channels != WING_SHM_CHANNELS || h->ring_frames != WING_SHM_RING_FRAMES) return -4;
    if (h->total_bytes != WING_SHM_TOTAL_BYTES) return -5;
    return 0;
}

#ifdef __cplusplus
}
#endif

#endif /* WING_SHM_H */
