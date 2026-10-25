// wing_shm_selftest - host-side integration test for the driver<->daemon
// transport. Needs no root and no installed driver.
//
// It drives wingsim::ShmLink (the exact code the HAL plug-in uses) with
// simulated HAL IO cycles on a private shm name, and checks:
//   1. loopback:   no daemon        -> input(T) == output(T - delay)
//   2. daemon:     client process   -> input(T) == gain * output(T - delay)
//   3. new epoch:  IO restart (sample time back to 0) is followed by the daemon
//   3b. (--record-only) input IO without output IO: the daemon clocks from
//       input_time and keeps from_daemon ahead (no underruns, silence in)
//   5. stall:      SIGSTOP/SIGCONT the daemon mid-stream -> crossfade to
//                  loopback and back, no clicks
//   6. crash:      SIGKILL the daemon mid-stream, from_daemon ring scribbled
//                  with garbage, new daemon started -> crossfade out, stale
//                  ring data never played, crossfade back in, no clicks
//   4. detach:     client exits     -> back to loopback
//
// Phases 5/6 play a DC level per channel, so any discontinuity in the input is
// a click; a crossfade moves at most |daemon - loopback| / kFadeFrames per frame.
//
//   wing_shm_selftest --client path/to/client [--record-only]
//   wing_shm_selftest --client path/to/client --sweep [--sweep-seconds 20]
//       [--sweep-delay 1024] [--sweep-buffers 64,128,256] [--client-arg --rt]
//
// --sweep runs the "driver" on a time-constraint thread at real-time pace for
// each IO buffer size with a generous input delay and reports how much of the
// delay the daemon consumed at worst (delay - min slack): the minimum
// glitch-free input delay on this machine. --client-arg passes extra arguments
// to the client (e.g. the Rust gain_client's --rt). WINGSIM_SWEEP_DEBUG=1
// prints each late block.
//
// The client is the C++ wing_shm_client (ctest) or the Rust wing-shm daemon
// side (crates/wing-shm/tests/cpp_selftest.rs, which passes --record-only).
#include "ShmLink.hpp"

#include <mach/mach.h>
#include <mach/mach_time.h>
#include <mach/thread_policy.h>

#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <spawn.h>
#include <sstream>
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
// The functional phases use a generous input delay: the test clients run at
// normal priority (the C++ one even polls), and a late block would only show
// up as a (click-free) excursion to loopback. The delay sweep measures limits.
constexpr uint32_t kTestDelay = 512;

int g_failures = 0;
std::vector<std::string> g_clientExtra; // --client-arg (sweep only)

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

// DC level per channel (continuity tests).
float Level(uint32_t c)
{
    return 0.01f * (float)(c + 1);
}

enum class Signal { Saw, Dc };

struct Stats
{
    long checked = 0;
    long mismatched = 0;
    long silent = 0;
    // Continuity (Signal::Dc): frames whose step from the previous frame
    // exceeds a crossfade's per-frame slope, and the largest such ratio.
    long clicks = 0;
    double maxStepRatio = 0;
    long loopbackFrames = 0; // frames == +level (fallback)
    long tailExact = 0;      // frames == expect * level in the final 30 %
    long tailChecked = 0;
};

// Simulates `cycles` HAL IO cycles starting at sample time `t0`.
// HAL order per cycle: ReadInput(now - buf) then WriteMix(now + buf).
// Saw: verifies input frames against expect_gain * Sig(t - delay) where the
// source output had been written. Dc: continuity + tail level.
// `hook(k)` runs before cycle k (fault injection).
Stats RunCycles(wingsim::ShmLink& link,
    int64_t t0,
    int cycles,
    float expectGain,
    bool realtime,
    Signal sig = Signal::Saw,
    const std::function<void(int)>& hook = nullptr)
{
    Stats st;
    std::vector<float> out(kBuf * kCh), in(kBuf * kCh), prev(kCh, 0.0f);
    const int64_t delay = link.InputDelayFrames();
    const int64_t firstWritten = t0 + kBuf;
    bool havePrev = false;
    const auto period = std::chrono::nanoseconds(1000000000ll * kBuf / kRate);
    auto next = std::chrono::steady_clock::now();

    for (int k = 0; k < cycles; k++) {
        if (hook) hook(k);
        const int64_t now = t0 + (int64_t)k * kBuf;
        const int64_t ti = now - kBuf;
        const int64_t to = now + kBuf;
        const bool tail = k >= cycles - cycles * 3 / 10;

        link.ReadInput(ti, in.data(), kBuf, kCh);
        for (uint32_t f = 0; f < kBuf; f++) {
            const int64_t src = ti + f - delay;
            if (src < firstWritten) continue;
            const float* v = &in[f * kCh];
            if (sig == Signal::Saw) {
                bool allZero = true, ok = true;
                for (uint32_t c = 0; c < kCh; c++) {
                    if (v[c] != 0.0f) allZero = false;
                    if (std::fabs(v[c] - expectGain * Sig(c, src)) > 1e-6f) ok = false;
                }
                st.checked++;
                if (allZero) st.silent++;
                if (!ok) st.mismatched++;
                continue;
            }
            st.checked++;
            bool loop = true, exact = true, click = false;
            for (uint32_t c = 0; c < kCh; c++) {
                const float a = Level(c);
                if (std::fabs(v[c] - a) > 1e-6f) loop = false;
                if (std::fabs(v[c] - expectGain * a) > 1e-6f) exact = false;
                if (havePrev) {
                    // Largest legal step: a full-scale crossfade between +a and
                    // gain*a, or between a and 0 (held/silence), per frame.
                    const float maxStep =
                        std::max(std::fabs(a - expectGain * a), a) /
                            (float)wingsim::ShmLink::kFadeFrames * 1.01f + 1e-6f;
                    const double r = std::fabs(v[c] - prev[c]) / maxStep;
                    st.maxStepRatio = std::max(st.maxStepRatio, r);
                    if (r > 1.0) click = true;
                }
                prev[c] = v[c];
            }
            havePrev = true;
            if (loop) st.loopbackFrames++;
            if (click) st.clicks++;
            if (tail) {
                st.tailChecked++;
                if (exact) st.tailExact++;
            }
        }

        for (uint32_t f = 0; f < kBuf; f++) {
            for (uint32_t c = 0; c < kCh; c++) {
                out[f * kCh + c] = sig == Signal::Saw ? Sig(c, to + f) : Level(c);
            }
        }
        link.WriteOutput(to, out.data(), kBuf, kCh);

        if (realtime) {
            next += period;
            std::this_thread::sleep_until(next);
        }
    }
    return st;
}

void Print(const char* label, const Stats& s)
{
    std::printf("      %s: checked %ld frames, mismatched %ld, silent %ld", label, s.checked,
        s.mismatched, s.silent);
    if (s.clicks || s.maxStepRatio > 0 || s.tailChecked) {
        std::printf(", clicks %ld (max step %.2f x fade slope), loopback frames %ld, "
                    "tail exact %ld/%ld",
            s.clicks, s.maxStepRatio, s.loopbackFrames, s.tailExact, s.tailChecked);
    }
    std::printf("\n");
}

pid_t Spawn(const std::string& client, const char* name, float gain, int seconds)
{
    char gainStr[16], secStr[16];
    std::snprintf(gainStr, sizeof gainStr, "%g", gain);
    std::snprintf(secStr, sizeof secStr, "%d", seconds);
    std::vector<const char*> args = {client.c_str(), "--name", name, "--gain", gainStr,
        "--seconds", secStr};
    for (const auto& a : g_clientExtra) args.push_back(a.c_str());
    args.push_back(nullptr);
    pid_t pid = 0;
    if (posix_spawn(&pid, client.c_str(), nullptr, nullptr, const_cast<char**>(args.data()),
            environ) != 0) {
        std::perror("posix_spawn");
        return -1;
    }
    return pid;
}

bool WaitAlive(wingsim::ShmLink& link, int ms)
{
    for (int i = 0; i < ms / 10; i++) {
        if (link.DaemonAlive()) return true;
        usleep(10000);
    }
    return link.DaemonAlive();
}

// Make the calling thread a time-constraint (real-time) thread, like the HAL
// IO thread inside coreaudiod.
void MakeRealtime(uint32_t buf)
{
    mach_timebase_info_data_t tb{};
    mach_timebase_info(&tb);
    const double nsPerTick = (double)tb.numer / tb.denom;
    const double periodNs = 1e9 * buf / kRate;
    thread_time_constraint_policy_data_t p{};
    p.period = (uint32_t)(periodNs / nsPerTick);
    p.computation = (uint32_t)(periodNs * 0.25 / nsPerTick);
    p.constraint = (uint32_t)(periodNs * 0.75 / nsPerTick);
    p.preemptible = 1;
    thread_policy_set(mach_thread_self(), THREAD_TIME_CONSTRAINT_POLICY, (thread_policy_t)&p,
        THREAD_TIME_CONSTRAINT_POLICY_COUNT);
}

std::vector<uint32_t> ParseList(const std::string& s)
{
    std::vector<uint32_t> v;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, ',')) {
        if (!item.empty()) v.push_back((uint32_t)std::strtoul(item.c_str(), nullptr, 10));
    }
    return v;
}

struct SweepResult
{
    long late = -1;          // blocks the daemon delivered late
    long lateAfterSkip = 0;  // ... shortly after the driver thread itself overran
    long skips = 0;          // driver cycles skipped (thread late, like a HAL overload)
    int64_t minSlack = -1;   // smallest margin of daemon data ahead of need
};

// One sweep point: real-time "driver" thread, client process, `seconds` of IO
// after a 0.5 s warm-up.
SweepResult SweepPoint(const std::string& client, uint32_t buf, uint32_t delay, double seconds)
{
    SweepResult res;
    char name[32];
    std::snprintf(name, sizeof name, "/wingsim.swp.%d", (int)getpid());
    shm_unlink(name);
    wingsim::ShmLink link;
    if (!link.Open(kRate, name, delay) || !link.IsShared()) return res;
    link.SetRunning(true);
    pid_t pid = Spawn(client, name, -1.0f, (int)seconds + 5);
    if (pid <= 0 || !WaitAlive(link, 2000)) {
        if (pid > 0) kill(pid, SIGKILL);
        return res;
    }
    link.BeginEpoch(kRate);

    std::thread drv([&] {
        MakeRealtime(buf);
        std::vector<float> out(buf * kCh, 0.25f), in(buf * kCh);
        const int cycles = (int)(seconds * kRate / buf);
        const int warm = (int)(0.5 * kRate / buf);
        const auto period = std::chrono::nanoseconds(1000000000ll * buf / kRate);
        const auto start = std::chrono::steady_clock::now();
        // Like the HAL: sample time follows the clock. If this thread is late
        // by a whole period, the missed cycles are skipped (an overload), not
        // run back to back.
        const int64_t recover = (int64_t)(delay / buf) + 8;
        int64_t lastSkipK = -1000000;
        long late = 0;
        for (int64_t k = 0; k < warm + cycles;) {
            if (k == warm) {
                late = 0;
                res.lateAfterSkip = 0;
                res.skips = 0;
                link.ResetMinSlack();
            }
            const int64_t now = k * buf;
            const uint64_t u0 = link.GetStats().underrunBlocks;
            link.ReadInput(now - buf, in.data(), buf, kCh);
            if (link.GetStats().underrunBlocks != u0) {
                if (k - lastSkipK <= recover) res.lateAfterSkip++;
                else late++;
                if (std::getenv("WINGSIM_SWEEP_DEBUG")) {
                    const auto* h = link.Header();
                    std::printf("      late at k=%lld (%.2f s): daemon write_time %lld, need %lld, "
                                "to_daemon %lld, overruns %u\n",
                        (long long)k, (double)(k * buf) / kRate,
                        (long long)WING_LOAD_RLX(&h->from_daemon.write_time),
                        (long long)(now - (int64_t)delay),
                        (long long)WING_LOAD_RLX(&h->to_daemon.write_time),
                        WING_LOAD_RLX(&h->to_daemon.overruns));
                }
            }
            link.WriteOutput(now + buf, out.data(), buf, kCh);
            std::this_thread::sleep_until(start + period * (k + 1));
            const int64_t kWall = (std::chrono::steady_clock::now() - start) / period;
            if (kWall > k + 1) {
                res.skips += kWall - (k + 1);
                lastSkipK = kWall;
                k = kWall;
            } else {
                k++;
            }
        }
        res.late = late;
        res.minSlack = link.GetStats().minSlack;
    });
    drv.join();
    kill(pid, SIGTERM);
    int status = 0;
    waitpid(pid, &status, 0);
    link.Close();
    shm_unlink(name);
    return res;
}

// Runs each buffer size at a generous input delay and reports how much of it
// the daemon actually consumed (delay - min slack). The minimum glitch-free
// delay on this machine is the largest consumption over all buffer sizes.
int Sweep(const std::string& client, const std::vector<uint32_t>& buffers, uint32_t delay,
    double seconds)
{
    std::printf("sweep: input delay %u, %.0f s per buffer size, driver thread time-constraint, "
                "client %s\n",
        delay, seconds, client.c_str());
    std::printf("%-8s %-12s %-14s %-16s %s\n", "buffer", "late blocks", "min slack",
        "delay consumed", "driver skips (late blocks after them)");
    int64_t worst = 0;
    bool ok = true;
    for (uint32_t b : buffers) {
        const SweepResult r = SweepPoint(client, b, delay, seconds);
        const int64_t used = r.minSlack == INT64_MAX ? -1 : (int64_t)delay - r.minSlack;
        std::printf("%-8u %-12ld %-14lld %-16lld %ld (%ld)\n", b, r.late, (long long)r.minSlack,
            (long long)used, r.skips, r.lateAfterSkip);
        std::fflush(stdout);
        if (r.late != 0 || r.minSlack < 0) ok = false;
        worst = std::max(worst, used);
    }
    std::printf("minimum glitch-free input delay on this run: %lld frames%s\n", (long long)worst,
        ok ? "" : " (INVALID: late blocks at the sweep delay, rerun with a larger --sweep-delay)");
    return ok ? 0 : 1;
}

} // namespace

int main(int argc, char** argv)
{
    std::string client;
    bool recordOnly = false, sweep = false;
    double sweepSeconds = 20;
    uint32_t sweepDelay = 1024;
    std::vector<uint32_t> sweepBuffers = {64, 128, 256};
    for (int i = 1; i < argc; i++) {
        const std::string a = argv[i];
        if (a == "--client" && i + 1 < argc) client = argv[++i];
        else if (a == "--record-only") recordOnly = true;
        else if (a == "--sweep") sweep = true;
        else if (a == "--sweep-seconds" && i + 1 < argc) sweepSeconds = std::atof(argv[++i]);
        else if (a == "--sweep-delay" && i + 1 < argc) sweepDelay = (uint32_t)std::atoi(argv[++i]);
        else if (a == "--sweep-buffers" && i + 1 < argc) sweepBuffers = ParseList(argv[++i]);
        else if (a == "--client-arg" && i + 1 < argc) g_clientExtra.push_back(argv[++i]);
    }
    if (client.empty()) {
        std::fprintf(stderr, "usage: wing_shm_selftest --client PATH [--record-only] [--sweep "
                             "[--sweep-seconds S] [--sweep-delay D] [--sweep-buffers L]]\n");
        return 2;
    }
    if (sweep) {
        return Sweep(client, sweepBuffers, sweepDelay, sweepSeconds);
    }

    char name[32];
    std::snprintf(name, sizeof name, "/wingsim.test.%d", (int)getpid());
    shm_unlink(name);

    wingsim::ShmLink link;
    Check(link.Open(kRate, name, kTestDelay) && link.IsShared(),
        "driver side creates POSIX shm segment");
    if (!link.IsShared()) {
        std::printf("      error: %s\n", link.LastError().c_str());
        return 1;
    }
    link.SetRunning(true);

    // 0. Configurable input delay: a segment with another delay is recreated.
    {
        wingsim::ShmLink other;
        other.Open(kRate, name, 64);
        Check(other.IsShared() && other.Header()->input_delay_frames == 64 &&
                  other.Header()->segment_id != link.Header()->segment_id,
            "segment with a different input delay is recreated with the configured delay");
        other.Close();
        // Back to the test delay (recreates again); re-open ours.
        link.Open(kRate, name, kTestDelay);
        link.SetRunning(true);
        Check(link.Header()->input_delay_frames == kTestDelay, "input delay restored");
    }

    // 1. Loopback (no daemon heartbeat).
    {
        Stats s = RunCycles(link, 0, 400, 1.0f, false);
        Print("loopback", s);
        Check(s.checked > 0 && s.mismatched == 0, "loopback: input(T) == output(T - delay)");
        Check(WING_LOAD_RLX(&link.Header()->loopback_active) == 1, "loopback flag set");
    }

    // 1b. IO restart: frames before the first output of the new epoch are
    //     silence, not the previous epoch's audio left in the ring.
    {
        link.BeginEpoch(kRate);
        std::vector<float> in(kBuf * kCh);
        long stale = 0;
        for (int k = 0; k < 6; k++) {
            const int64_t now = (int64_t)k * kBuf;
            link.ReadInput(now - kBuf, in.data(), kBuf, kCh);
            for (float v : in) stale += v != 0.0f;
            std::vector<float> out(kBuf * kCh, 0.0f); // silence written
            link.WriteOutput(now + kBuf, out.data(), kBuf, kCh);
        }
        Check(stale == 0, "new epoch: no stale audio from the previous epoch");
    }

    // 2. Daemon attached.
    pid_t pid = Spawn(client, name, kGain, 30);
    if (pid <= 0) return 1;
    Check(WaitAlive(link, 2000), "daemon heartbeat detected");

    link.BeginEpoch(kRate); // like HAL StartIO: daemon resyncs to the new epoch
    usleep(20000);
    {
        Stats s = RunCycles(link, 0, 600, kGain, true); // ~1.6 s real time
        Print("daemon", s);
        const uint32_t under = WING_LOAD_RLX(&link.Header()->from_daemon.underruns);
        std::printf("      driver-side underruns: %u\n", under);
        Check(s.checked > 0 && s.mismatched <= s.checked / 20,
            "daemon: input(T) == gain * output(T - delay) (<=5% frames late or recovering)");
        Check(WING_LOAD_RLX(&link.Header()->loopback_active) == 0, "loopback flag cleared");
    }

    // 3. IO restart: sample time back to 0, new epoch.
    link.BeginEpoch(kRate);
    usleep(20000);
    {
        Stats s = RunCycles(link, 0, 300, kGain, true);
        Print("after restart", s);
        Check(s.checked > 0 && s.mismatched <= s.checked / 20,
            "daemon follows new epoch after IO restart");
    }

    // 3b. Record-only: ReadInput without WriteOutput. to_daemon.write_time
    //     stays at 0; the daemon must clock from input_time.
    if (recordOnly) {
        link.BeginEpoch(kRate);
        usleep(20000);
        std::vector<float> in(kBuf * kCh);
        const int kWarmup = 10, kCycles = 300;
        uint32_t underAtWarmup = 0;
        long nonZero = 0;
        for (int k = 0; k < kCycles; k++) {
            if (k == kWarmup) {
                underAtWarmup = WING_LOAD_RLX(&link.Header()->from_daemon.underruns);
            }
            link.ReadInput((int64_t)k * kBuf, in.data(), kBuf, kCh);
            for (float v : in) nonZero += v != 0.0f;
            std::this_thread::sleep_for(std::chrono::microseconds(1000000 * kBuf / kRate));
        }
        const uint32_t under =
            WING_LOAD_RLX(&link.Header()->from_daemon.underruns) - underAtWarmup;
        std::printf("      record-only: %d cycles, underruns after warmup %u, non-zero %ld\n",
            kCycles - kWarmup, under, nonZero);
        Check(under <= (kCycles - kWarmup) / 100 && nonZero == 0,
            "record-only: daemon clocks from input_time (<=1% underruns, silence)");
    }

    // 5. Stall: SIGSTOP the daemon mid-stream for ~0.5 s, then SIGCONT.
    link.BeginEpoch(kRate);
    usleep(20000);
    {
        const auto before = link.GetStats();
        Stats s = RunCycles(link, 0, 1100, kGain, true, Signal::Dc, [&](int k) {
            if (k == 200) kill(pid, SIGSTOP);
            if (k == 400) kill(pid, SIGCONT);
        });
        const auto after = link.GetStats();
        Print("stall", s);
        std::printf("      to-fallback %llu, to-daemon %llu\n",
            (unsigned long long)(after.toFallback - before.toFallback),
            (unsigned long long)(after.toDaemon - before.toDaemon));
        Check(s.clicks == 0, "stall: no clicks (crossfade to loopback and back)");
        Check(s.loopbackFrames > 0 && after.toFallback > before.toFallback,
            "stall: falls back to loopback while the daemon is stopped");
        Check(s.tailChecked > 0 && s.tailExact * 2 >= s.tailChecked &&
                  after.toDaemon > before.toDaemon,
            "stall: back on the daemon after it resumes");
    }

    // 6. Crash + restart: SIGKILL (heartbeat left stale, not cleared), scribble
    //    the from_daemon ring, start a new daemon.
    link.BeginEpoch(kRate);
    usleep(20000);
    {
        const auto before = link.GetStats();
        Stats s = RunCycles(link, 0, 1100, kGain, true, Signal::Dc, [&](int k) {
            if (k == 200) {
                kill(pid, SIGKILL);
                int st = 0;
                waitpid(pid, &st, 0);
            }
            if (k == 260) {
                float* r = wing_shm_ring(link.Header(), WING_SHM_HEADER_BYTES + WING_SHM_RING_BYTES);
                for (size_t i = 0; i < (size_t)kCh * WING_SHM_RING_FRAMES; i++) r[i] = 7.0f;
            }
            if (k == 300) pid = Spawn(client, name, kGain, 30);
        });
        const auto after = link.GetStats();
        Print("crash+restart", s);
        Check(s.clicks == 0, "crash: no clicks, stale ring data never played");
        Check(s.loopbackFrames > 0, "crash: falls back to loopback");
        Check(s.tailChecked > 0 && s.tailExact * 2 >= s.tailChecked &&
                  after.toDaemon > before.toDaemon,
            "restart: new daemon picked up, crossfaded back in");
        if (pid <= 0) return 1;
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

    // Revalidate: the name is removed / recreated behind the driver's back.
    {
        const uint64_t id = link.Header()->segment_id;
        Check(!link.Revalidate(kRate), "revalidate: same segment kept");
        shm_unlink(name);
        Check(link.Revalidate(kRate) && link.IsShared() && link.Header()->segment_id != id,
            "revalidate: removed name is recreated");
    }

    link.Close();
    shm_unlink(name);
    std::printf("%s (%d failures)\n", g_failures ? "FAILED" : "ALL PASSED", g_failures);
    return g_failures ? 1 : 0;
}
