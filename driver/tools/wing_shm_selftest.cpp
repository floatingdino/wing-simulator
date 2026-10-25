// wing_shm_selftest - host-side integration test for the driver<->daemon
// transport. Needs no root and no installed driver.
//
// It drives wingsim::ShmLink (the exact code the HAL plug-in uses) with
// simulated HAL IO cycles on a private shm name, and checks:
//   1. loopback:   no daemon        -> input(T) == output(T - delay)
//   2. daemon:     wing_shm_client  -> input(T) == gain * output(T - delay)
//   3. new epoch:  IO restart (sample time back to 0) is followed by the daemon
//   4. detach:     client exits     -> back to loopback
//
//   wing_shm_selftest --client path/to/wing_shm_client
#include "ShmLink.hpp"

#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <spawn.h>
#include <string>
#include <sys/mman.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>

extern char** environ;

namespace {

constexpr uint32_t kCh = WING_SHM_CHANNELS;
constexpr uint32_t kBuf = 128;
constexpr uint32_t kRate = 48000;
constexpr float kGain = -0.5f;

int g_failures = 0;

void Check(bool ok, const char* what)
{
    std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) g_failures++;
}

// Deterministic test signal, distinct per channel and sample.
float Sig(uint32_t c, int64_t s)
{
    return (float)((s * 7 + (int64_t)c * 131) % 1000) / 1000.0f - 0.5f;
}

struct Stats
{
    long checked = 0;
    long mismatched = 0;
    long silent = 0;
};

// Simulates `cycles` HAL IO cycles starting at sample time `t0`.
// HAL order per cycle: ReadInput(now - buf) then WriteMix(now + buf).
// Verifies input frames against expect_gain * Sig(t - delay) where the source
// output had been written.
Stats RunCycles(wingsim::ShmLink& link,
    int64_t t0,
    int cycles,
    float expectGain,
    bool realtime)
{
    Stats st;
    std::vector<float> out(kBuf * kCh), in(kBuf * kCh);
    const int64_t delay = link.InputDelayFrames();
    const int64_t firstWritten = t0 + kBuf;

    for (int k = 0; k < cycles; k++) {
        const int64_t now = t0 + (int64_t)k * kBuf;
        const int64_t ti = now - kBuf;
        const int64_t to = now + kBuf;

        link.ReadInput(ti, in.data(), kBuf, kCh);
        for (uint32_t f = 0; f < kBuf; f++) {
            const int64_t src = ti + f - delay;
            if (src < firstWritten) continue;
            bool allZero = true, ok = true;
            for (uint32_t c = 0; c < kCh; c++) {
                const float v = in[f * kCh + c];
                if (v != 0.0f) allZero = false;
                if (std::fabs(v - expectGain * Sig(c, src)) > 1e-6f) ok = false;
            }
            st.checked++;
            if (allZero) st.silent++;
            if (!ok) st.mismatched++;
        }

        for (uint32_t f = 0; f < kBuf; f++) {
            for (uint32_t c = 0; c < kCh; c++) {
                out[f * kCh + c] = Sig(c, to + f);
            }
        }
        link.WriteOutput(to, out.data(), kBuf, kCh);

        if (realtime) {
            std::this_thread::sleep_for(std::chrono::microseconds(1000000 * kBuf / kRate));
        }
    }
    return st;
}

void Print(const char* label, const Stats& s)
{
    std::printf("      %s: checked %ld frames, mismatched %ld, silent %ld\n", label, s.checked,
        s.mismatched, s.silent);
}

} // namespace

int main(int argc, char** argv)
{
    std::string client;
    for (int i = 1; i < argc; i++) {
        if (std::string(argv[i]) == "--client" && i + 1 < argc) client = argv[++i];
    }
    if (client.empty()) {
        std::fprintf(stderr, "usage: wing_shm_selftest --client PATH\n");
        return 2;
    }

    char name[32];
    std::snprintf(name, sizeof name, "/wingsim.test.%d", (int)getpid());
    shm_unlink(name);

    wingsim::ShmLink link;
    Check(link.Open(kRate, name) && link.IsShared(), "driver side creates POSIX shm segment");
    if (!link.IsShared()) {
        std::printf("      error: %s\n", link.LastError().c_str());
        return 1;
    }
    link.SetRunning(true);

    // 1. Loopback (no daemon heartbeat).
    {
        Stats s = RunCycles(link, 0, 400, 1.0f, false);
        Print("loopback", s);
        Check(s.checked > 0 && s.mismatched == 0, "loopback: input(T) == output(T - delay)");
        Check(WING_LOAD_RLX(&link.Header()->loopback_active) == 1, "loopback flag set");
    }

    // 2. Daemon attached.
    char gainStr[16];
    std::snprintf(gainStr, sizeof gainStr, "%g", kGain);
    const char* args[] = {client.c_str(), "--name", name, "--gain", gainStr, "--seconds", "20",
        nullptr};
    pid_t pid = 0;
    if (posix_spawn(&pid, client.c_str(), nullptr, nullptr, const_cast<char**>(args), environ) !=
        0) {
        std::perror("posix_spawn");
        return 1;
    }
    bool alive = false;
    for (int i = 0; i < 200 && !alive; i++) {
        alive = link.DaemonAlive();
        if (!alive) usleep(10000);
    }
    Check(alive, "daemon heartbeat detected");

    link.BeginEpoch(kRate); // like HAL StartIO: daemon resyncs to the new epoch
    usleep(20000);
    {
        Stats s = RunCycles(link, 0, 600, kGain, true); // ~1.6 s real time
        Print("daemon", s);
        const uint32_t under = WING_LOAD_RLX(&link.Header()->from_daemon.underruns);
        std::printf("      driver-side underruns: %u\n", under);
        Check(s.checked > 0 && s.mismatched <= s.checked / 100,
            "daemon: input(T) == gain * output(T - delay) (<=1% late frames)");
        Check(WING_LOAD_RLX(&link.Header()->loopback_active) == 0, "loopback flag cleared");
    }

    // 3. IO restart: sample time back to 0, new epoch.
    link.BeginEpoch(kRate);
    usleep(20000);
    {
        Stats s = RunCycles(link, 0, 300, kGain, true);
        Print("after restart", s);
        Check(s.checked > 0 && s.mismatched <= s.checked / 100,
            "daemon follows new epoch after IO restart");
    }

    // 4. Detach -> loopback.
    kill(pid, SIGTERM);
    int status = 0;
    waitpid(pid, &status, 0);
    Check(WIFEXITED(status) && WEXITSTATUS(status) == 0, "client exits cleanly on SIGTERM");
    Check(!link.DaemonAlive(), "heartbeat cleared on detach");
    link.BeginEpoch(kRate);
    {
        Stats s = RunCycles(link, 0, 200, 1.0f, false);
        Print("loopback again", s);
        Check(s.checked > 0 && s.mismatched == 0, "falls back to loopback after detach");
    }

    link.Close();
    shm_unlink(name);
    std::printf("%s (%d failures)\n", g_failures ? "FAILED" : "ALL PASSED", g_failures);
    return g_failures ? 1 : 0;
}
