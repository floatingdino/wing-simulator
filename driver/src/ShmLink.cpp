#include "ShmLink.hpp"

#include <mach/mach_time.h>
#if __has_include(<os/os_sync_wait_on_address.h>)
#include <os/os_sync_wait_on_address.h>
#define WINGSIM_HAVE_OS_SYNC 1
#endif

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace wingsim {

namespace {

// Frames at the old end of a ring that we refuse to read, so a producer that
// advances while we copy cannot overwrite what we are reading.
constexpr int64_t kRingGuardFrames = WING_SHM_RING_GUARD_FRAMES;

// Wake a daemon blocked in os_sync_wait_on_address(&cycle_seq, ..SHARED..).
// One syscall, only issued while the daemon has announced it is waiting.
void WakeDaemon(uint32_t* addr)
{
#ifdef WINGSIM_HAVE_OS_SYNC
    if (__builtin_available(macOS 14.4, *)) {
        os_sync_wake_by_address_all(addr, sizeof(uint32_t), OS_SYNC_WAKE_BY_ADDRESS_SHARED);
    }
#else
    (void)addr;
#endif
}

uint64_t NsToTicks(uint64_t ns)
{
    mach_timebase_info_data_t tb{};
    mach_timebase_info(&tb);
    if (tb.numer == 0) {
        return ns;
    }
    return ns * tb.denom / tb.numer;
}

std::string Errno(const char* what)
{
    return std::string(what) + ": " + std::strerror(errno);
}

} // namespace

ShmLink::~ShmLink()
{
    Close();
}

bool ShmLink::Open(uint32_t sampleRate, const char* name)
{
    Close();
    heartbeatTimeoutTicks_ = NsToTicks(WING_SHM_HEARTBEAT_TIMEOUT_NS);

    const size_t total = static_cast<size_t>(WING_SHM_TOTAL_BYTES);

    // 1. Attach to an existing segment (created by a previous driver instance
    //    or by a daemon started first). The segment survives coreaudiod
    //    restarts, so a running daemon keeps its mapping valid.
    int fd = shm_open(name, O_RDWR, 0);
    if (fd >= 0) {
        struct stat st{};
        if (fstat(fd, &st) == 0 && static_cast<uint64_t>(st.st_size) >= total) {
            void* p = mmap(nullptr, total, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
            if (p != MAP_FAILED) {
                auto* h = static_cast<wing_shm_header*>(p);
                int rc = wing_shm_validate_header(h, st.st_size);
                // magic == 0: a daemon may be creating it right now; give it
                // up to ~100 ms to publish the header before replacing it.
                for (int i = 0; i < 100 && rc == -2 && WING_LOAD_ACQ(&h->magic) == 0; i++) {
                    usleep(1000);
                    rc = wing_shm_validate_header(h, st.st_size);
                }
                if (rc == 0) {
                    close(fd);
                    base_ = p;
                    hdr_ = h;
                    mappedBytes_ = total;
                    shared_ = true;
                    WING_STORE_REL(&hdr_->driver_pid, static_cast<uint64_t>(getpid()));
                    BeginEpoch(sampleRate);
                    return true;
                }
                munmap(p, total);
            }
        }
        // Incompatible (old version / wrong size). macOS cannot re-ftruncate a
        // pshm object, so unlink and recreate under the same name; a daemon
        // still mapping the old one will notice via heartbeat/epoch and reattach.
        close(fd);
        shm_unlink(name);
    }

    // 2. Create. Note: on macOS the pshm mode is applied verbatim (umask is
    //    not applied) and fchmod() on a pshm fd returns EINVAL, so 0666 here
    //    is what lets the daemon (a different uid than _coreaudiod) attach.
    fd = shm_open(name, O_RDWR | O_CREAT | O_EXCL, 0666);
    if (fd >= 0) {
        if (ftruncate(fd, static_cast<off_t>(total)) == 0) {
            void* p = mmap(nullptr, total, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
            if (p != MAP_FAILED) {
                close(fd);
                std::memset(p, 0, total);
                base_ = p;
                hdr_ = static_cast<wing_shm_header*>(p);
                mappedBytes_ = total;
                shared_ = true;
                wing_shm_init_header(hdr_, sampleRate, mach_absolute_time(),
                    static_cast<uint64_t>(getpid()));
                WING_STORE_REL(&hdr_->driver_pid, static_cast<uint64_t>(getpid()));
                BeginEpoch(sampleRate);
                return true;
            }
            lastError_ = Errno("mmap");
        } else {
            lastError_ = Errno("ftruncate");
        }
        close(fd);
        shm_unlink(name);
    } else {
        lastError_ = Errno("shm_open");
    }

    // 3. Fallback: private memory, same layout. Loopback-only.
    void* p = mmap(nullptr, total, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANON, -1, 0);
    if (p == MAP_FAILED) {
        lastError_ += "; " + Errno("mmap(anon)");
        return false;
    }
    base_ = p;
    hdr_ = static_cast<wing_shm_header*>(p);
    mappedBytes_ = total;
    shared_ = false;
    wing_shm_init_header(hdr_, sampleRate, mach_absolute_time(), static_cast<uint64_t>(getpid()));
    BeginEpoch(sampleRate);
    return true;
}

void ShmLink::Close()
{
    if (base_) {
        WING_STORE_REL(&hdr_->io_running, 0u);
        munmap(base_, mappedBytes_);
    }
    base_ = nullptr;
    hdr_ = nullptr;
    mappedBytes_ = 0;
    shared_ = false;
}

void ShmLink::BeginEpoch(uint32_t sampleRate)
{
    if (!hdr_) {
        return;
    }
    // Order matters: reset our ring head, then publish the new epoch (release)
    // so a daemon that observes the new epoch also observes write_time == 0.
    WING_STORE_RLX(&hdr_->sample_rate, sampleRate);
    WING_STORE_RLX(&hdr_->to_daemon.write_time, int64_t{0});
    WING_STORE_RLX(&hdr_->input_time, int64_t{0});
    const uint32_t next = WING_LOAD_RLX(&hdr_->io_epoch) + 1u;
    WING_STORE_REL(&hdr_->to_daemon.epoch, next);
    WING_STORE_REL(&hdr_->io_epoch, next);
    Signal();
}

void ShmLink::Signal()
{
    if (!hdr_ || !shared_) {
        return;
    }
    // seq_cst pairs with the daemon's (daemon_waiting++, re-check cycle_seq):
    // either it sees the new cycle_seq or we see daemon_waiting != 0.
    WING_INC_SEQ(&hdr_->cycle_seq);
    if (WING_LOAD_SEQ(&hdr_->daemon_waiting) != 0) {
        WakeDaemon(&hdr_->cycle_seq);
    }
}

void ShmLink::SetRunning(bool running)
{
    if (hdr_) {
        WING_STORE_REL(&hdr_->io_running, running ? 1u : 0u);
    }
}

bool ShmLink::DaemonAlive() const
{
    if (!hdr_ || !shared_) {
        return false;
    }
    const uint64_t hb = WING_LOAD_ACQ(&hdr_->daemon_heartbeat);
    if (hb == 0) {
        return false;
    }
    const uint64_t now = mach_absolute_time();
    // Heartbeat may be marginally in the future relative to our read.
    return now < hb || (now - hb) < heartbeatTimeoutTicks_;
}

void ShmLink::WriteOutput(int64_t t,
    const float* interleaved,
    uint32_t frames,
    uint32_t channels)
{
    if (!hdr_) {
        return;
    }
    float* ring = wing_shm_ring(base_, hdr_->to_daemon_offset);
    const uint32_t n = hdr_->ring_frames;
    const uint32_t chans = channels < hdr_->channels ? channels : hdr_->channels;

    for (uint32_t f = 0; f < frames; f++) {
        const uint32_t idx = wing_shm_ring_index(t + f, n);
        const float* src = interleaved + static_cast<size_t>(f) * channels;
        for (uint32_t c = 0; c < chans; c++) {
            ring[static_cast<size_t>(c) * n + idx] = src[c];
        }
    }

    const int64_t end = t + frames;
    if (end > WING_LOAD_RLX(&hdr_->to_daemon.write_time)) {
        WING_STORE_REL(&hdr_->to_daemon.write_time, end);
    }
    WING_STORE_RLX(&hdr_->driver_heartbeat, mach_absolute_time());
    Signal();
}

bool ShmLink::CopyFromRing(const float* ring,
    const wing_shm_ring_ctl& ctl,
    int64_t t,
    float* interleaved,
    uint32_t frames,
    uint32_t channels)
{
    const int64_t wt = WING_LOAD_ACQ(&ctl.write_time);
    const int64_t oldest = wt - static_cast<int64_t>(WING_SHM_RING_FRAMES) + kRingGuardFrames;
    const uint32_t n = WING_SHM_RING_FRAMES;
    const uint32_t chans = channels < WING_SHM_CHANNELS ? channels : WING_SHM_CHANNELS;
    bool complete = true;

    for (uint32_t f = 0; f < frames; f++) {
        const int64_t s = t + f;
        float* dst = interleaved + static_cast<size_t>(f) * channels;
        if (s < 0 || s >= wt || s < oldest) {
            std::memset(dst, 0, sizeof(float) * channels);
            complete = false;
            continue;
        }
        const uint32_t idx = wing_shm_ring_index(s, n);
        for (uint32_t c = 0; c < chans; c++) {
            dst[c] = ring[static_cast<size_t>(c) * n + idx];
        }
        for (uint32_t c = chans; c < channels; c++) {
            dst[c] = 0.0f;
        }
    }
    return complete;
}

void ShmLink::ReadInput(int64_t t, float* interleaved, uint32_t frames, uint32_t channels)
{
    if (!hdr_) {
        std::memset(interleaved, 0, sizeof(float) * frames * channels);
        return;
    }
    const int64_t src = t - static_cast<int64_t>(hdr_->input_delay_frames);
    const uint32_t epoch = WING_LOAD_ACQ(&hdr_->io_epoch);
    const int64_t end = t + static_cast<int64_t>(frames);
    WING_STORE_RLX(&hdr_->input_frames, frames);
    WING_STORE_REL(&hdr_->input_time, end);

    if (DaemonAlive()) {
        WING_STORE_RLX(&hdr_->loopback_active, 0u);
        const bool sameEpoch = WING_LOAD_ACQ(&hdr_->from_daemon.epoch) == epoch;
        const float* ring = wing_shm_ring(base_, hdr_->from_daemon_offset);
        if (!sameEpoch ||
            !CopyFromRing(ring, hdr_->from_daemon, src, interleaved, frames, channels)) {
            if (!sameEpoch) {
                std::memset(interleaved, 0, sizeof(float) * frames * channels);
            }
            if (src >= 0) {
                WING_INC_RLX(&hdr_->from_daemon.underruns);
            }
        }
        WING_STORE_RLX(&hdr_->from_daemon.read_time, src + static_cast<int64_t>(frames));
        // Record-only (no output IO feeding to_daemon): our reads are the
        // daemon's clock, so wake it.
        if (WING_LOAD_RLX(&hdr_->to_daemon.write_time) < end) {
            Signal();
        }
    } else {
        WING_STORE_RLX(&hdr_->loopback_active, 1u);
        const float* ring = wing_shm_ring(base_, hdr_->to_daemon_offset);
        CopyFromRing(ring, hdr_->to_daemon, src, interleaved, frames, channels);
    }
    WING_STORE_RLX(&hdr_->driver_heartbeat, mach_absolute_time());
}

} // namespace wingsim
