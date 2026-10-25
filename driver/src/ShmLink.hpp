// ShmLink - driver side of the wing_shm transport (layout v2).
//
// Owns (creates or attaches to) the POSIX shm segment WING_SHM_NAME and moves
// interleaved HAL buffers in/out of the planar sample-time-indexed rings.
// If shm cannot be opened at all, it falls back to private memory with the same
// layout so the device still works as a loopback.
//
// Input source selection (ReadInput, realtime):
//   daemon    from_daemon(T - delay), when the daemon is alive, on the current
//             epoch and its data for the whole block is present and contiguous
//   fallback  loopback to_daemon(T - delay); silence where the DAW wrote nothing
// Switching between the two is a linear crossfade of kFadeFrames frames, so a
// daemon that appears, crashes, stalls or restarts mid-stream never produces a
// click. While fading out of a stalled daemon, frames it never delivered are
// replaced by its last delivered sample (held), not by zeros. After falling
// back, the daemon must deliver kRecoverFrames frames on time before the
// driver fades back to it (hysteresis against flapping).
//
// Has no CoreAudio dependency so the host-side self test can drive it exactly
// like the HAL does.
#pragma once

#include "wing_shm.h"

#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

namespace wingsim {

class ShmLink
{
public:
    // Crossfade length between daemon and fallback input.
    static constexpr uint32_t kFadeFrames = 128;
    // Consecutive on-time daemon frames required before fading back to it.
    static constexpr uint32_t kRecoverFrames = 1024;
    // A jump of from_daemon.write_time larger than this (plus one IO buffer)
    // between two reads is treated as a daemon resync: older ring content is
    // considered stale.
    static constexpr int64_t kMaxDaemonJumpFrames = 2048;
    // ReadInput result cache (identical reads by several HAL clients in one
    // IO cycle). Larger IO buffers are rendered per client.
    static constexpr uint32_t kMaxCacheFrames = 4096;
    // Accepted input delay range (frames). The ring must hold
    // delay + 2 IO buffers + guard.
    static constexpr uint32_t kMaxInputDelayFrames = 2048;

    struct Stats
    {
        uint64_t toFallback = 0;    // crossfades daemon -> fallback
        uint64_t toDaemon = 0;      // crossfades fallback -> daemon
        uint64_t underrunBlocks = 0; // blocks the daemon was relied on but late
        uint64_t resyncs = 0;       // daemon data discontinuities detected
        uint64_t gapFills = 0;      // output gaps zero-filled in to_daemon
        // Smallest margin (frames) by which daemon data was ahead of what a
        // ReadInput needed while the daemon was the source, since
        // ResetMinSlack(). delay - minSlack is the delay actually consumed;
        // INT64_MAX if not measured.
        int64_t minSlack = INT64_MAX;
    };

    ShmLink();
    ~ShmLink();

    ShmLink(const ShmLink&) = delete;
    ShmLink& operator=(const ShmLink&) = delete;

    // ---- non-realtime ----

    // Attach to an existing compatible segment with the same input delay, else
    // (re)create it. Returns false only if even the private-memory fallback
    // failed. inputDelayFrames is clamped to [0, kMaxInputDelayFrames].
    bool Open(uint32_t sampleRate,
        const char* name = WING_SHM_NAME,
        uint32_t inputDelayFrames = WING_SHM_DEFAULT_INPUT_DELAY_FRAMES);
    void Close();

    // Call while IO is stopped (HAL StartIO). Re-attaches if the shm name was
    // removed or now refers to a different segment (e.g. recreated by a
    // daemon), and retries shared memory if we were on the private fallback.
    // Returns true if the mapping changed.
    bool Revalidate(uint32_t sampleRate);

    // Called when the HAL starts/stops IO (first client / last client) and when
    // the nominal sample rate changes. Starts a new epoch: sample time restarts.
    // Wakes the daemon so it sees the new epoch / rate immediately.
    void BeginEpoch(uint32_t sampleRate);
    void SetRunning(bool running);

    bool IsShared() const { return shared_; }
    const std::string& LastError() const { return lastError_; }
    const std::string& Name() const { return name_; }
    wing_shm_header* Header() const { return hdr_; }
    uint32_t InputDelayFrames() const { return delay_; }
    Stats GetStats() const;
    void ResetMinSlack() { minSlack_.store(INT64_MAX, std::memory_order_relaxed); }

    // ---- realtime (no alloc / locks / syscalls except the os_sync wake) ----

    // Store DAW output (interleaved, `channels` wide) for sample times [t, t+frames).
    // A gap since the previous write is zero-filled so stale audio is never read.
    void WriteOutput(int64_t t, const float* interleaved, uint32_t frames, uint32_t channels);

    // Produce DAW input (interleaved) for sample times [t, t+frames). See the
    // source selection above. Repeated calls for the same block (one per HAL
    // client) return identical data without advancing the crossfade.
    void ReadInput(int64_t t, float* interleaved, uint32_t frames, uint32_t channels);

    // True if the daemon heartbeat is fresh (realtime safe).
    bool DaemonAlive() const;

private:
    void Signal();
    void ResetInputState(uint32_t epoch);
    bool MapNew(uint32_t sampleRate);

    wing_shm_header* hdr_ = nullptr;
    void* base_ = nullptr;
    size_t mappedBytes_ = 0;
    bool shared_ = false;
    uint32_t delay_ = WING_SHM_DEFAULT_INPUT_DELAY_FRAMES;
    uint64_t heartbeatTimeoutTicks_ = 0;
    std::string name_;
    std::string lastError_;

    // ---- IO-thread state (ReadInput / WriteOutput) ----
    uint32_t stateEpoch_ = 0;
    bool stateValid_ = false;
    bool connected_ = false;   // daemon seen alive on this epoch, data tracked
    bool target_ = false;      // crossfade target: true = daemon
    float gain_ = 0.0f;        // current daemon weight (0..1)
    uint64_t healthy_ = 0;     // consecutive on-time daemon frames
    int64_t validFrom_ = 0;    // first contiguous from_daemon sample time
    int64_t lastWt_ = 0;
    uint64_t lastPid_ = 0;
    float held_[WING_SHM_CHANNELS] = {};

    std::vector<float> cache_;
    bool cacheValid_ = false;
    int64_t cacheT_ = 0;
    uint32_t cacheFrames_ = 0;
    uint32_t cacheChannels_ = 0;
    uint32_t cacheEpoch_ = 0;

    // Written on the IO thread, read by StopIO logging.
    std::atomic<uint64_t> toFallback_{0};
    std::atomic<uint64_t> toDaemon_{0};
    std::atomic<uint64_t> underrunBlocks_{0};
    std::atomic<uint64_t> resyncs_{0};
    std::atomic<uint64_t> gapFills_{0};
    std::atomic<int64_t> minSlack_{INT64_MAX};
};

} // namespace wingsim
