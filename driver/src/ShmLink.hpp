// ShmLink - driver side of the wing_shm transport.
//
// Owns (creates or attaches to) the POSIX shm segment WING_SHM_NAME and moves
// interleaved HAL buffers in/out of the planar sample-time-indexed rings.
// If shm cannot be opened at all, it falls back to private memory with the same
// layout so the device still works as a loopback.
//
// Has no CoreAudio dependency so the host-side self test can drive it exactly
// like the HAL does.
#pragma once

#include "wing_shm.h"

#include <cstdint>
#include <string>

namespace wingsim {

class ShmLink
{
public:
    ShmLink() = default;
    ~ShmLink();

    ShmLink(const ShmLink&) = delete;
    ShmLink& operator=(const ShmLink&) = delete;

    // ---- non-realtime ----

    // Attach to an existing compatible segment, else (re)create it.
    // Returns false only if even the private-memory fallback failed.
    bool Open(uint32_t sampleRate, const char* name = WING_SHM_NAME);
    void Close();

    // Called when the HAL starts/stops IO (first client / last client) and when
    // the nominal sample rate changes. Starts a new epoch: sample time restarts.
    void BeginEpoch(uint32_t sampleRate);
    void SetRunning(bool running);

    bool IsShared() const { return shared_; }
    const std::string& LastError() const { return lastError_; }
    wing_shm_header* Header() const { return hdr_; }
    uint32_t InputDelayFrames() const
    {
        return hdr_ ? hdr_->input_delay_frames : WING_SHM_DEFAULT_INPUT_DELAY_FRAMES;
    }

    // ---- realtime (no alloc / locks / syscalls) ----

    // Store DAW output (interleaved, `channels` wide) for sample times [t, t+frames).
    void WriteOutput(int64_t t, const float* interleaved, uint32_t frames, uint32_t channels);

    // Produce DAW input (interleaved) for sample times [t, t+frames).
    // Daemon alive -> from_daemon(t - delay); otherwise loopback to_daemon(t - delay).
    void ReadInput(int64_t t, float* interleaved, uint32_t frames, uint32_t channels);

    // True if the daemon heartbeat is fresh (realtime safe).
    bool DaemonAlive() const;

private:
    static bool CopyFromRing(const float* ring,
        const wing_shm_ring_ctl& ctl,
        int64_t t,
        float* interleaved,
        uint32_t frames,
        uint32_t channels);

    wing_shm_header* hdr_ = nullptr;
    void* base_ = nullptr;
    size_t mappedBytes_ = 0;
    bool shared_ = false;
    uint64_t heartbeatTimeoutTicks_ = 0;
    std::string lastError_;
};

} // namespace wingsim
