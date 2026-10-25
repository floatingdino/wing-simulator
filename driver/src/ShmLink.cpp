#include "ShmLink.hpp"

#include <mach/mach_time.h>
#if __has_include(<os/os_sync_wait_on_address.h>)
#include <os/os_sync_wait_on_address.h>
#define WINGSIM_HAVE_OS_SYNC 1
#endif

#include <algorithm>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace wingsim {

namespace {

constexpr int64_t kRingFrames = WING_SHM_RING_FRAMES;
// Frames at the old end of a ring that we refuse to read, so a producer that
// advances while we copy cannot overwrite what we are reading.
constexpr int64_t kRingGuardFrames = WING_SHM_RING_GUARD_FRAMES;
constexpr uint32_t kRingChannels = WING_SHM_CHANNELS;

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

// ---- planar ring <-> interleaved buffer copies (realtime) ----
// Channel-outer loops: contiguous ring access, one strided side.

// ring[c][s0 .. s0+n) -> dst[f*stride + c], for c < chans. All frames valid.
void RingToInterleaved(const float* ring, int64_t s0, uint32_t n, float* dst, uint32_t stride,
    uint32_t chans)
{
    const uint32_t i0 = wing_shm_ring_index(s0, WING_SHM_RING_FRAMES);
    const uint32_t first = std::min<uint32_t>(n, WING_SHM_RING_FRAMES - i0);
    for (uint32_t c = 0; c < chans; c++) {
        const float* r = ring + static_cast<size_t>(c) * WING_SHM_RING_FRAMES;
        float* d = dst + c;
        for (uint32_t f = 0; f < first; f++) {
            d[static_cast<size_t>(f) * stride] = r[i0 + f];
        }
        for (uint32_t f = first; f < n; f++) {
            d[static_cast<size_t>(f) * stride] = r[f - first];
        }
    }
}

// src[f*stride + c] -> ring[c][s0 .. s0+n), for c < chans.
void InterleavedToRing(const float* src, uint32_t stride, uint32_t chans, float* ring, int64_t s0,
    uint32_t n)
{
    const uint32_t i0 = wing_shm_ring_index(s0, WING_SHM_RING_FRAMES);
    const uint32_t first = std::min<uint32_t>(n, WING_SHM_RING_FRAMES - i0);
    for (uint32_t c = 0; c < chans; c++) {
        float* r = ring + static_cast<size_t>(c) * WING_SHM_RING_FRAMES;
        const float* s = src + c;
        for (uint32_t f = 0; f < first; f++) {
            r[i0 + f] = s[static_cast<size_t>(f) * stride];
        }
        for (uint32_t f = first; f < n; f++) {
            r[f - first] = s[static_cast<size_t>(f) * stride];
        }
    }
}

// Zero ring[c][s0 .. s0+n) for all channels (n <= ring frames).
void ZeroRing(float* ring, int64_t s0, uint32_t n)
{
    const uint32_t i0 = wing_shm_ring_index(s0, WING_SHM_RING_FRAMES);
    const uint32_t first = std::min<uint32_t>(n, WING_SHM_RING_FRAMES - i0);
    for (uint32_t c = 0; c < kRingChannels; c++) {
        float* r = ring + static_cast<size_t>(c) * WING_SHM_RING_FRAMES;
        std::memset(r + i0, 0, sizeof(float) * first);
        std::memset(r, 0, sizeof(float) * (n - first));
    }
}

// Copy frames [s0, s0+n) whose sample time lies in [lo, hi) from the ring, zero
// the others (all `stride` channels of those frames).
void RingToInterleavedClipped(const float* ring, int64_t lo, int64_t hi, int64_t s0, uint32_t n,
    float* dst, uint32_t stride, uint32_t chans)
{
    const int64_t a = std::clamp<int64_t>(lo, s0, s0 + n);
    const int64_t b = std::clamp<int64_t>(hi, a, s0 + n);
    const size_t head = static_cast<size_t>(a - s0);
    const size_t mid = static_cast<size_t>(b - a);
    if (head) {
        std::memset(dst, 0, sizeof(float) * head * stride);
    }
    if (mid) {
        float* d = dst + head * stride;
        RingToInterleaved(ring, a, static_cast<uint32_t>(mid), d, stride, chans);
        if (chans < stride) {
            for (size_t f = 0; f < mid; f++) {
                std::memset(d + f * stride + chans, 0, sizeof(float) * (stride - chans));
            }
        }
    }
    const size_t tail = n - head - mid;
    if (tail) {
        std::memset(dst + (head + mid) * stride, 0, sizeof(float) * tail * stride);
    }
}

} // namespace

ShmLink::ShmLink()
    : cache_(static_cast<size_t>(kMaxCacheFrames) * WING_SHM_CHANNELS, 0.0f)
{
}

ShmLink::~ShmLink()
{
    Close();
}

bool ShmLink::Open(uint32_t sampleRate, const char* name, uint32_t inputDelayFrames)
{
    Close();
    heartbeatTimeoutTicks_ = NsToTicks(WING_SHM_HEARTBEAT_TIMEOUT_NS);
    delay_ = std::min(inputDelayFrames, kMaxInputDelayFrames);
    name_ = name;
    lastError_.clear();

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
                if (rc == 0 && h->input_delay_frames == delay_) {
                    close(fd);
                    base_ = p;
                    hdr_ = h;
                    mappedBytes_ = total;
                    shared_ = true;
                    WING_STORE_REL(&hdr_->driver_pid, static_cast<uint64_t>(getpid()));
                    BeginEpoch(sampleRate);
                    return true;
                }
                if (rc == 0) {
                    lastError_ = "existing segment has input delay " +
                                 std::to_string(h->input_delay_frames) + ", recreating";
                }
                munmap(p, total);
            }
        }
        // Incompatible (old version / wrong size / other input delay). macOS
        // cannot re-ftruncate a pshm object, so unlink and recreate under the
        // same name; a daemon still mapping the old one sees its driver
        // heartbeat stop and must reattach.
        close(fd);
        shm_unlink(name);
    }
    return MapNew(sampleRate);
}

bool ShmLink::MapNew(uint32_t sampleRate)
{
    const size_t total = static_cast<size_t>(WING_SHM_TOTAL_BYTES);
    const char* name = name_.c_str();

    // 2. Create. Note: on macOS the pshm mode is applied verbatim (umask is
    //    not applied) and fchmod() on a pshm fd returns EINVAL, so 0666 here
    //    is what lets the daemon (a different uid than _coreaudiod) attach.
    int fd = shm_open(name, O_RDWR | O_CREAT | O_EXCL, 0666);
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
                wing_shm_init_header_delay(hdr_, sampleRate, mach_absolute_time(),
                    static_cast<uint64_t>(getpid()), delay_);
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
    wing_shm_init_header_delay(
        hdr_, sampleRate, mach_absolute_time(), static_cast<uint64_t>(getpid()), delay_);
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
    stateValid_ = false;
    cacheValid_ = false;
}

bool ShmLink::Revalidate(uint32_t sampleRate)
{
    if (name_.empty()) {
        return false;
    }
    if (hdr_ && shared_) {
        // Same name, same segment? Compare the immutable segment_id.
        int fd = shm_open(name_.c_str(), O_RDONLY, 0);
        if (fd >= 0) {
            bool same = false;
            void* p = mmap(nullptr, WING_SHM_HEADER_BYTES, PROT_READ, MAP_SHARED, fd, 0);
            if (p != MAP_FAILED) {
                const auto* h = static_cast<const wing_shm_header*>(p);
                same = WING_LOAD_ACQ(&h->magic) == WING_SHM_MAGIC &&
                       h->segment_id == hdr_->segment_id && h->creator_pid == hdr_->creator_pid;
                munmap(p, WING_SHM_HEADER_BYTES);
            }
            close(fd);
            if (same) {
                return false;
            }
        }
    }
    // Name gone, replaced, or we are on private memory: (re)attach or create.
    const uint32_t delay = delay_;
    const std::string name = name_;
    Open(sampleRate, name.c_str(), delay);
    return true;
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

ShmLink::Stats ShmLink::GetStats() const
{
    Stats s;
    s.toFallback = toFallback_.load(std::memory_order_relaxed);
    s.toDaemon = toDaemon_.load(std::memory_order_relaxed);
    s.underrunBlocks = underrunBlocks_.load(std::memory_order_relaxed);
    s.resyncs = resyncs_.load(std::memory_order_relaxed);
    s.gapFills = gapFills_.load(std::memory_order_relaxed);
    return s;
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
    if (!hdr_ || frames == 0) {
        return;
    }
    float* ring = wing_shm_ring(base_, WING_SHM_HEADER_BYTES);
    const uint32_t chans = std::min(channels, kRingChannels);

    // Zero any gap since our last write (IO start, skipped cycles, output
    // client paused) so neither loopback nor the daemon reads stale audio.
    const int64_t wt = WING_LOAD_RLX(&hdr_->to_daemon.write_time);
    if (t > wt) {
        const int64_t gap = std::min<int64_t>(t - wt, kRingFrames);
        ZeroRing(ring, t - gap, static_cast<uint32_t>(gap));
        gapFills_.fetch_add(1, std::memory_order_relaxed);
    }

    // Frames beyond the ring length would overwrite themselves: keep the tail.
    uint32_t skip = 0;
    if (frames > kRingFrames) {
        skip = frames - static_cast<uint32_t>(kRingFrames);
    }
    InterleavedToRing(interleaved + static_cast<size_t>(skip) * channels, channels, chans, ring,
        t + skip, frames - skip);
    if (chans < kRingChannels) {
        // Channels the HAL did not supply read as silence.
        for (uint32_t c = chans; c < kRingChannels; c++) {
            float* r = ring + static_cast<size_t>(c) * WING_SHM_RING_FRAMES;
            for (uint32_t f = skip; f < frames; f++) {
                r[wing_shm_ring_index(t + f, WING_SHM_RING_FRAMES)] = 0.0f;
            }
        }
    }

    const int64_t end = t + frames;
    if (end > wt) {
        WING_STORE_REL(&hdr_->to_daemon.write_time, end);
    }
    WING_STORE_RLX(&hdr_->driver_heartbeat, mach_absolute_time());
    Signal();
}

void ShmLink::ResetInputState(uint32_t epoch)
{
    stateEpoch_ = epoch;
    stateValid_ = true;
    connected_ = false;
    validFrom_ = INT64_MAX;
    lastWt_ = 0;
    lastPid_ = 0;
    std::memset(held_, 0, sizeof held_);
    cacheValid_ = false;
    // Sample time restarts; the first `delay` frames of input are silence in
    // both sources, so the starting source can be chosen without a fade.
    const bool alive = DaemonAlive();
    target_ = alive;
    gain_ = alive ? 1.0f : 0.0f;
    healthy_ = alive ? kRecoverFrames : 0;
}

void ShmLink::ReadInput(int64_t t, float* out, uint32_t frames, uint32_t channels)
{
    const size_t samples = static_cast<size_t>(frames) * channels;
    if (!hdr_) {
        std::memset(out, 0, sizeof(float) * samples);
        return;
    }
    const uint32_t epoch = WING_LOAD_ACQ(&hdr_->io_epoch);

    // Second HAL client reading the same block: identical data, no state change.
    if (cacheValid_ && cacheT_ == t && cacheFrames_ == frames && cacheChannels_ == channels &&
        cacheEpoch_ == epoch) {
        std::memcpy(out, cache_.data(), sizeof(float) * samples);
        return;
    }

    const int64_t src = t - static_cast<int64_t>(delay_);
    const int64_t end = t + static_cast<int64_t>(frames);
    const int64_t srcEnd = src + static_cast<int64_t>(frames);
    WING_STORE_RLX(&hdr_->input_frames, frames);
    WING_STORE_REL(&hdr_->input_time, end);

    if (!stateValid_ || epoch != stateEpoch_) {
        ResetInputState(epoch);
    }

    // ---- observe the daemon ----
    const bool alive = DaemonAlive();
    bool ready = false;
    int64_t dwt = 0;
    if (alive && WING_LOAD_ACQ(&hdr_->from_daemon.epoch) == epoch) {
        // Load write_time after epoch: the daemon stores the resync
        // write_time before it publishes the epoch.
        dwt = WING_LOAD_ACQ(&hdr_->from_daemon.write_time);
        const uint64_t pid = WING_LOAD_RLX(&hdr_->daemon_pid);
        if (!connected_ || pid != lastPid_ || dwt < lastWt_ ||
            dwt - lastWt_ > kMaxDaemonJumpFrames + static_cast<int64_t>(frames)) {
            // (Re)connected, restarted or resynced: only data written from
            // now on is known to be contiguous.
            if (connected_) {
                resyncs_.fetch_add(1, std::memory_order_relaxed);
            }
            validFrom_ = dwt;
            connected_ = true;
        }
        lastWt_ = dwt;
        lastPid_ = pid;
        ready = true;
    } else {
        connected_ = false;
    }
    // Daemon data usable for s in [dLo, dwt). Sample times < 0 are silence in
    // every source, so a block starting before 0 only needs data from 0.
    const int64_t dLo =
        std::max<int64_t>({validFrom_, dwt - kRingFrames + kRingGuardFrames, int64_t{0}});
    const bool blockOk = ready && std::max<int64_t>(src, 0) >= dLo && srcEnd <= dwt;

    // ---- choose the target source ----
    if (srcEnd > 0) { // blocks entirely before sample 0 are silence either way
        healthy_ = blockOk ? healthy_ + frames : 0;
        if (target_ && !blockOk) {
            target_ = false;
            toFallback_.fetch_add(1, std::memory_order_relaxed);
            if (alive && src >= 0) {
                WING_INC_RLX(&hdr_->from_daemon.underruns);
                underrunBlocks_.fetch_add(1, std::memory_order_relaxed);
            }
        } else if (!target_ && blockOk && healthy_ >= kRecoverFrames) {
            target_ = true;
            toDaemon_.fetch_add(1, std::memory_order_relaxed);
        }
    }
    WING_STORE_RLX(&hdr_->loopback_active, target_ ? 0u : 1u);

    // ---- render ----
    const float* fromRing = wing_shm_ring(base_, WING_SHM_HEADER_BYTES + WING_SHM_RING_BYTES);
    const float* toRing = wing_shm_ring(base_, WING_SHM_HEADER_BYTES);
    const int64_t lwt = WING_LOAD_ACQ(&hdr_->to_daemon.write_time);
    const int64_t lLo = std::max<int64_t>(0, lwt - kRingFrames + kRingGuardFrames);
    const uint32_t chans = std::min(channels, kRingChannels);

    if (target_ && gain_ >= 1.0f) {
        // Steady daemon: blockOk holds, straight copy.
        RingToInterleavedClipped(fromRing, dLo, dwt, src, frames, out, channels, chans);
        if (frames) {
            std::memcpy(held_, out + static_cast<size_t>(frames - 1) * channels,
                sizeof(float) * chans);
        }
    } else if (!target_ && gain_ <= 0.0f) {
        // Steady fallback: loopback where the DAW wrote output, else silence.
        RingToInterleavedClipped(toRing, lLo, lwt, src, frames, out, channels, chans);
    } else {
        // Crossfade, per frame.
        const float step = 1.0f / static_cast<float>(kFadeFrames);
        float g = gain_;
        for (uint32_t f = 0; f < frames; f++) {
            const int64_t s = src + f;
            const bool dOk = ready && s >= dLo && s < dwt;
            const bool lOk = s >= lLo && s < lwt;
            const uint32_t idx = wing_shm_ring_index(s, WING_SHM_RING_FRAMES);
            g = target_ ? std::min(1.0f, g + step) : std::max(0.0f, g - step);
            float* o = out + static_cast<size_t>(f) * channels;
            for (uint32_t c = 0; c < chans; c++) {
                const size_t ri = static_cast<size_t>(c) * WING_SHM_RING_FRAMES + idx;
                if (dOk) {
                    held_[c] = fromRing[ri];
                }
                const float d = held_[c];
                const float l = lOk ? toRing[ri] : 0.0f;
                o[c] = l + g * (d - l);
            }
            for (uint32_t c = chans; c < channels; c++) {
                o[c] = 0.0f;
            }
        }
        gain_ = g;
    }

    if (ready) {
        WING_STORE_RLX(&hdr_->from_daemon.read_time, srcEnd);
    }
    // Record-only (no output IO feeding to_daemon): our reads are the
    // daemon's clock, so wake it.
    if (alive && lwt < end) {
        Signal();
    }
    WING_STORE_RLX(&hdr_->driver_heartbeat, mach_absolute_time());

    if (frames <= kMaxCacheFrames && channels <= WING_SHM_CHANNELS) {
        std::memcpy(cache_.data(), out, sizeof(float) * samples);
        cacheValid_ = true;
        cacheT_ = t;
        cacheFrames_ = frames;
        cacheChannels_ = channels;
        cacheEpoch_ = epoch;
    } else {
        cacheValid_ = false;
    }
}

} // namespace wingsim
