// wing_driver_smoketest - loads the built WingSimulator.driver bundle in-process
// and drives it through the AudioServerPlugIn C interface the way coreaudiod
// does (fake host). Checks without root/install:
//   - plug-in factory + Initialize work, one device "Wing Simulator"
//   - 48-ch output stream + 48-ch input stream, float32, 44.1k/48k available
//   - channel names "USB In n" (output) / "USB Out n" (input), n = 1..48
//   - latency: input stream latency == configured input delay (default, or
//     --delay N via a config plist), device latency and safety offsets 0
//   - sample-rate change 48k -> 44.1k -> 48k propagates to streams and shm,
//     starts a new shm epoch and wakes the daemon (cycle_seq)
//   - IO cycles: loopback input(T) == output(T - delay); a restarted IO session
//     never replays the previous session's audio
//   - fallback crossfades with a synchronous fake daemon (no clicks, no stale
//     ring data): attach, stall, resume, fast restart (new pid + resync gap),
//     crash (stale heartbeat); a second HAL client reading the same cycle gets
//     identical data without speeding up the crossfade
//
//   wing_driver_smoketest path/to/WingSimulator.driver [--delay N]
#include "wing_shm.h"

#include <CoreAudio/AudioServerPlugIn.h>
#include <CoreFoundation/CoreFoundation.h>
#include <mach/mach_time.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <string>
#include <sys/mman.h>
#include <unistd.h>
#include <vector>

namespace {

int g_fail = 0;
void Check(bool ok, const std::string& what)
{
    std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what.c_str());
    if (!ok) g_fail++;
}

AudioServerPlugInDriverRef g_drv = nullptr;
struct PendingChange
{
    AudioObjectID dev;
    UInt64 action;
    void* info;
};
std::vector<PendingChange> g_pending;

// ---- fake host ----
OSStatus HostPropertiesChanged(AudioServerPlugInHostRef, AudioObjectID, UInt32,
    const AudioObjectPropertyAddress*)
{
    return noErr;
}
OSStatus HostCopyFromStorage(AudioServerPlugInHostRef, CFStringRef, CFPropertyListRef* out)
{
    *out = nullptr;
    return kAudioHardwareUnspecifiedError;
}
OSStatus HostWriteToStorage(AudioServerPlugInHostRef, CFStringRef, CFPropertyListRef)
{
    return noErr;
}
OSStatus HostDeleteFromStorage(AudioServerPlugInHostRef, CFStringRef)
{
    return noErr;
}
OSStatus HostRequestConfigChange(AudioServerPlugInHostRef, AudioObjectID dev, UInt64 action,
    void* info)
{
    g_pending.push_back({dev, action, info});
    return noErr;
}
AudioServerPlugInHostInterface g_host = {HostPropertiesChanged, HostCopyFromStorage,
    HostWriteToStorage, HostDeleteFromStorage, HostRequestConfigChange};

void RunPendingChanges()
{
    while (!g_pending.empty()) {
        auto p = g_pending.front();
        g_pending.erase(g_pending.begin());
        (*g_drv)->PerformDeviceConfigurationChange(g_drv, p.dev, p.action, p.info);
    }
}

template <typename T>
bool Get(AudioObjectID obj, AudioObjectPropertySelector sel, T& out,
    AudioObjectPropertyScope scope = kAudioObjectPropertyScopeGlobal, UInt32 el = 0)
{
    AudioObjectPropertyAddress a = {sel, scope, el};
    UInt32 size = sizeof(T);
    return (*g_drv)->GetPropertyData(g_drv, obj, 0, &a, 0, nullptr, size, &size, &out) == noErr;
}

template <typename T>
std::vector<T> GetVec(AudioObjectID obj, AudioObjectPropertySelector sel,
    AudioObjectPropertyScope scope = kAudioObjectPropertyScopeGlobal)
{
    AudioObjectPropertyAddress a = {sel, scope, 0};
    UInt32 size = 0;
    if ((*g_drv)->GetPropertyDataSize(g_drv, obj, 0, &a, 0, nullptr, &size) != noErr) return {};
    std::vector<T> v(size / sizeof(T));
    (*g_drv)->GetPropertyData(g_drv, obj, 0, &a, 0, nullptr, size, &size, v.data());
    v.resize(size / sizeof(T));
    return v;
}

std::string GetStr(AudioObjectID obj, AudioObjectPropertySelector sel,
    AudioObjectPropertyScope scope = kAudioObjectPropertyScopeGlobal, UInt32 el = 0)
{
    CFStringRef s = nullptr;
    if (!Get(obj, sel, s, scope, el) || !s) return "";
    char buf[256] = {};
    CFStringGetCString(s, buf, sizeof buf, kCFStringEncodingUTF8);
    CFRelease(s);
    return buf;
}

float Sig(uint32_t c, int64_t s)
{
    return (float)((s * 7 + (int64_t)c * 131) % 1000) / 1000.0f - 0.5f;
}

float Level(uint32_t c)
{
    return 0.01f * (float)(c + 1);
}

constexpr uint32_t kCh = 48;
constexpr UInt32 kBuf = 256;
// Must match wingsim::ShmLink (driver/src/ShmLink.hpp).
constexpr uint32_t kFadeFrames = 128;

struct Io
{
    AudioObjectID dev, si, so;
    std::vector<UInt32> clients;

    void Op(UInt32 client, UInt32 op, AudioObjectID stream, AudioServerPlugInIOCycleInfo& cyc,
        void* data) const
    {
        (*g_drv)->BeginIOOperation(g_drv, dev, client, op, kBuf, &cyc);
        (*g_drv)->DoIOOperation(g_drv, dev, stream, client, op, kBuf, &cyc, data, nullptr);
        (*g_drv)->EndIOOperation(g_drv, dev, client, op, kBuf, &cyc);
    }

    // One HAL cycle at anchor `now`: every client reads input [now-buf, now),
    // the first one writes the mixed output [now+buf, now+2buf).
    void Cycle(int64_t k, std::vector<std::vector<float>>& in, const std::vector<float>& out) const
    {
        const Float64 now = (Float64)k * kBuf;
        AudioServerPlugInIOCycleInfo cyc = {};
        cyc.mIOCycleCounter = (UInt64)k;
        cyc.mNominalIOBufferFrameSize = kBuf;
        cyc.mInputTime.mSampleTime = now - kBuf;
        cyc.mInputTime.mFlags = kAudioTimeStampSampleTimeValid;
        cyc.mOutputTime.mSampleTime = now + kBuf;
        cyc.mOutputTime.mFlags = kAudioTimeStampSampleTimeValid;
        for (size_t i = 0; i < clients.size(); i++) {
            Op(clients[i], kAudioServerPlugInIOOperationReadInput, si, cyc, in[i].data());
        }
        Op(clients[0], kAudioServerPlugInIOOperationWriteMix, so, cyc,
            const_cast<float*>(out.data()));
    }
};

// Synchronous stand-in for the daemon: from_daemon = -to_daemon, processed
// right after each WriteMix, behaving like wing-shm's DaemonLink on resync.
struct FakeDaemon
{
    wing_shm_header* h;
    uint64_t pid = 0;
    bool processing = false;
    uint32_t epoch = ~0u;
    int64_t cursor = 0;

    float* To() const { return wing_shm_ring(h, h->to_daemon_offset); }
    float* From() const { return wing_shm_ring(h, h->from_daemon_offset); }

    void Heartbeat() const
    {
        WING_STORE_REL(&h->daemon_pid, pid);
        WING_STORE_REL(&h->daemon_heartbeat, mach_absolute_time());
    }

    void Process()
    {
        if (!processing) return;
        Heartbeat();
        const uint32_t e = WING_LOAD_ACQ(&h->io_epoch);
        const int64_t w = WING_LOAD_ACQ(&h->to_daemon.write_time);
        if (e != epoch) {
            epoch = e;
            cursor = std::max<int64_t>(w - 512, 0);
            WING_STORE_REL(&h->from_daemon.write_time, cursor);
            WING_STORE_REL(&h->from_daemon.epoch, e);
        }
        if (w - cursor > WING_SHM_RING_FRAMES / 2) cursor = w - 512; // overrun resync
        const uint32_t n = WING_SHM_RING_FRAMES;
        for (int64_t t = cursor; t < w; t++) {
            const uint32_t i = wing_shm_ring_index(t, n);
            for (uint32_t c = 0; c < kCh; c++) From()[(size_t)c * n + i] = -To()[(size_t)c * n + i];
        }
        cursor = std::max(cursor, w);
        WING_STORE_REL(&h->from_daemon.write_time, cursor);
    }
};

std::string WriteConfig(uint32_t delay)
{
    char path[] = "/tmp/wingsim_smoke_cfg_XXXXXX";
    int fd = mkstemp(path);
    if (fd < 0) return "";
    std::string xml = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
                      "<!DOCTYPE plist PUBLIC \"-//Apple//DTD PLIST 1.0//EN\" "
                      "\"http://www.apple.com/DTDs/PropertyList-1.0.dtd\">\n"
                      "<plist version=\"1.0\"><dict><key>InputDelayFrames</key><integer>" +
                      std::to_string(delay) + "</integer></dict></plist>\n";
    const bool ok = write(fd, xml.data(), xml.size()) == (ssize_t)xml.size();
    close(fd);
    return ok ? path : "";
}

} // namespace

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::fprintf(stderr, "usage: wing_driver_smoketest WingSimulator.driver [--delay N]\n");
        return 2;
    }
    uint32_t wantDelay = WING_SHM_DEFAULT_INPUT_DELAY_FRAMES;
    std::string cfgPath;
    for (int i = 2; i < argc; i++) {
        if (std::string(argv[i]) == "--delay" && i + 1 < argc) {
            wantDelay = (uint32_t)std::strtoul(argv[++i], nullptr, 10);
            cfgPath = WriteConfig(wantDelay);
            Check(!cfgPath.empty(), "write config plist (InputDelayFrames " +
                                        std::to_string(wantDelay) + ")");
        }
    }
    // Never pick up the developer's real /Library/Preferences/Audio config.
    setenv("WINGSIM_CONFIG", cfgPath.empty() ? "/nonexistent/wingsim.plist" : cfgPath.c_str(), 1);

    char shmName[32];
    std::snprintf(shmName, sizeof shmName, "/wingsim.smoke.%d", (int)getpid());
    shm_unlink(shmName);
    setenv("WINGSIM_SHM_NAME", shmName, 1);

    const std::string exe = std::string(argv[1]) + "/Contents/MacOS/WingSimulator";
    void* lib = dlopen(exe.c_str(), RTLD_NOW | RTLD_LOCAL);
    Check(lib != nullptr, "dlopen bundle executable");
    if (!lib) {
        std::printf("      %s\n", dlerror());
        return 1;
    }
    using Factory = void* (*)(CFAllocatorRef, CFUUIDRef);
    auto factory = reinterpret_cast<Factory>(dlsym(lib, "WingSimulatorEntryPoint"));
    Check(factory != nullptr, "factory symbol exported");
    g_drv = static_cast<AudioServerPlugInDriverRef>(factory(nullptr, kAudioServerPlugInTypeUUID));
    Check(g_drv != nullptr, "factory returns driver interface");
    if (!g_drv) return 1;

    Check((*g_drv)->Initialize(g_drv, &g_host) == noErr, "Initialize");
    RunPendingChanges();
    if (!cfgPath.empty()) unlink(cfgPath.c_str()); // read once at load

    auto devices = GetVec<AudioObjectID>(kAudioObjectPlugInObject, kAudioPlugInPropertyDeviceList);
    Check(devices.size() == 1, "one device");
    if (devices.empty()) return 1;
    const AudioObjectID dev = devices[0];
    Check(GetStr(dev, kAudioObjectPropertyName) == "Wing Simulator", "device name 'Wing Simulator'");

    auto outStreams = GetVec<AudioObjectID>(dev, kAudioDevicePropertyStreams,
        kAudioObjectPropertyScopeOutput);
    auto inStreams = GetVec<AudioObjectID>(dev, kAudioDevicePropertyStreams,
        kAudioObjectPropertyScopeInput);
    Check(outStreams.size() == 1 && inStreams.size() == 1, "one output + one input stream");
    if (outStreams.size() != 1 || inStreams.size() != 1) return 1;
    const AudioObjectID so = outStreams[0], si = inStreams[0];

    AudioStreamBasicDescription fo{}, fi{};
    Get(so, kAudioStreamPropertyPhysicalFormat, fo);
    Get(si, kAudioStreamPropertyPhysicalFormat, fi);
    Check(fo.mChannelsPerFrame == 48 && fi.mChannelsPerFrame == 48, "48 ch out / 48 ch in");
    Check((fo.mFormatFlags & kAudioFormatFlagIsFloat) && fo.mBitsPerChannel == 32, "float32");
    Check(fo.mSampleRate == 48000 && fi.mSampleRate == 48000, "default 48 kHz");

    auto rates = GetVec<AudioValueRange>(dev, kAudioDevicePropertyAvailableNominalSampleRates);
    bool has441 = false, has48 = false;
    for (auto& r : rates) {
        has441 |= r.mMinimum == 44100;
        has48 |= r.mMinimum == 48000;
    }
    Check(has441 && has48 && rates.size() == 2, "available rates are exactly 44100 and 48000");

    {
        bool names = true;
        for (UInt32 n = 1; n <= 48; n++) {
            names &= GetStr(dev, kAudioObjectPropertyElementName, kAudioObjectPropertyScopeOutput,
                         n) == "USB In " + std::to_string(n);
            names &= GetStr(dev, kAudioObjectPropertyElementName, kAudioObjectPropertyScopeInput,
                         n) == "USB Out " + std::to_string(n);
        }
        Check(names, "channel names: output n 'USB In n', input n 'USB Out n' (n = 1..48)");
    }

    // ---- latency / safety offset ----
    UInt32 inLat = 99999, outLat = 99999, devLatIn = 99999, devLatOut = 99999, safeIn = 99999,
           safeOut = 99999;
    Get(si, kAudioStreamPropertyLatency, inLat);
    Get(so, kAudioStreamPropertyLatency, outLat);
    Get(dev, kAudioDevicePropertyLatency, devLatIn, kAudioObjectPropertyScopeInput);
    Get(dev, kAudioDevicePropertyLatency, devLatOut, kAudioObjectPropertyScopeOutput);
    Get(dev, kAudioDevicePropertySafetyOffset, safeIn, kAudioObjectPropertyScopeInput);
    Get(dev, kAudioDevicePropertySafetyOffset, safeOut, kAudioObjectPropertyScopeOutput);
    std::printf("      latency: input stream %u, output stream %u, device in/out %u/%u, "
                "safety in/out %u/%u\n",
        inLat, outLat, devLatIn, devLatOut, safeIn, safeOut);
    Check(inLat == wantDelay, "input stream latency == configured input delay (" +
                                  std::to_string(wantDelay) + ")");
    Check(outLat == 0 && devLatIn == 0 && devLatOut == 0, "output stream + device latency 0");
    Check(safeIn == 0 && safeOut == 0, "safety offsets 0");

    // Map the segment the driver created to observe it.
    int fd = shm_open(shmName, O_RDWR, 0);
    Check(fd >= 0, "driver created POSIX shm segment");
    if (fd < 0) return 1;
    auto* hdr = static_cast<wing_shm_header*>(
        mmap(nullptr, WING_SHM_TOTAL_BYTES, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0));
    close(fd);
    Check(wing_shm_validate_header(hdr, WING_SHM_TOTAL_BYTES) == 0, "segment header valid (v2)");
    Check(hdr->input_delay_frames == wantDelay, "segment input_delay_frames == configured delay");

    // Sample rate change through the HAL path; the daemon is told through a
    // new epoch + cycle_seq wake.
    for (Float64 want : {44100.0, 48000.0}) {
        const uint32_t e0 = WING_LOAD_RLX(&hdr->io_epoch), q0 = WING_LOAD_RLX(&hdr->cycle_seq);
        AudioObjectPropertyAddress a = {kAudioDevicePropertyNominalSampleRate,
            kAudioObjectPropertyScopeGlobal, 0};
        (*g_drv)->SetPropertyData(g_drv, dev, 0, &a, 0, nullptr, sizeof want, &want);
        RunPendingChanges();
        Float64 got = 0;
        Get(dev, kAudioDevicePropertyNominalSampleRate, got);
        Get(so, kAudioStreamPropertyPhysicalFormat, fo);
        Get(si, kAudioStreamPropertyVirtualFormat, fi);
        const std::string r = std::to_string((int)want);
        Check(got == want && fo.mSampleRate == want && fi.mSampleRate == want &&
                  WING_LOAD_RLX(&hdr->sample_rate) == (uint32_t)want,
            "rate change to " + r + " reaches device, streams, shm");
        Check(WING_LOAD_RLX(&hdr->io_epoch) == e0 + 1 && WING_LOAD_RLX(&hdr->cycle_seq) != q0,
            "rate change to " + r + " starts a new epoch and wakes the daemon");
    }
    {
        Float64 bad = 96000;
        AudioObjectPropertyAddress a = {kAudioDevicePropertyNominalSampleRate,
            kAudioObjectPropertyScopeGlobal, 0};
        (*g_drv)->SetPropertyData(g_drv, dev, 0, &a, 0, nullptr, sizeof bad, &bad);
        RunPendingChanges();
        Float64 got = 0;
        Get(dev, kAudioDevicePropertyNominalSampleRate, got);
        Check(got == 48000, "unsupported rate 96000 rejected");
    }

    // ---- IO session 1: one client, saw signal, loopback (no daemon) ----
    AudioServerPlugInClientInfo ci = {7, getpid(), true, nullptr};
    AudioServerPlugInClientInfo ci2 = {8, getpid(), true, nullptr};
    Check((*g_drv)->AddDeviceClient(g_drv, dev, &ci) == noErr, "AddDeviceClient");
    const uint32_t epochBefore = WING_LOAD_RLX(&hdr->io_epoch);
    Check((*g_drv)->StartIO(g_drv, dev, 7) == noErr, "StartIO");
    Check(WING_LOAD_RLX(&hdr->io_epoch) == epochBefore + 1 && WING_LOAD_RLX(&hdr->io_running) == 1,
        "StartIO begins a new shm epoch");
    {
        Boolean will = false, inPlace = false;
        (*g_drv)->WillDoIOOperation(g_drv, dev, 7, kAudioServerPlugInIOOperationReadInput, &will,
            &inPlace);
        Check(will, "device does ReadInput");
        (*g_drv)->WillDoIOOperation(g_drv, dev, 7, kAudioServerPlugInIOOperationWriteMix, &will,
            &inPlace);
        Check(will, "device does WriteMix");
    }

    Io io{dev, si, so, {7}};
    std::vector<std::vector<float>> in(2, std::vector<float>(kBuf * kCh));
    std::vector<float> out(kBuf * kCh);
    {
        long checked = 0, bad = 0;
        for (int k = 0; k < 200; k++) {
            const int64_t ti = (int64_t)k * kBuf - kBuf;
            const int64_t to = (int64_t)k * kBuf + kBuf;
            for (UInt32 f = 0; f < kBuf; f++)
                for (uint32_t c = 0; c < kCh; c++) out[f * kCh + c] = Sig(c, to + f);
            io.Cycle(k, in, out);
            for (UInt32 f = 0; f < kBuf; f++) {
                const int64_t src = ti + f - (int64_t)inLat;
                if (src < (int64_t)kBuf) continue; // first output written at sample time buf
                for (uint32_t c = 0; c < kCh; c++) {
                    if (std::fabs(in[0][f * kCh + c] - Sig(c, src)) > 1e-6f) {
                        bad++;
                        break;
                    }
                }
                checked++;
            }
        }
        std::printf("      loopback via HAL interface: %ld frames checked, %ld bad\n", checked, bad);
        Check(checked > 0 && bad == 0,
            "loopback through DoIOOperation: input(T) == output(T - delay)");
        Check(WING_LOAD_RLX(&hdr->loopback_active) == 1, "driver reports loopback mode (no daemon)");
    }
    Check((*g_drv)->StopIO(g_drv, dev, 7) == noErr, "StopIO");

    // ---- IO session 2: DC levels, two clients, fake daemon fault injection ----
    Check((*g_drv)->AddDeviceClient(g_drv, dev, &ci2) == noErr, "AddDeviceClient (second)");
    (*g_drv)->StartIO(g_drv, dev, 7);
    (*g_drv)->StartIO(g_drv, dev, 8);
    io.clients = {7, 8};
    for (UInt32 f = 0; f < kBuf; f++)
        for (uint32_t c = 0; c < kCh; c++) out[f * kCh + c] = Level(c);

    FakeDaemon d{hdr};
    std::vector<float> prev(kCh, 0.0f);
    bool havePrev = false;
    long clicks = 0, garbage = 0, stale = 0, differ = 0, frames = 0;
    double maxRatio = 0;
    // State at the end of each phase: +1 loopback, -1 daemon.
    struct Mark { int k; float gain; const char* what; };
    const Mark marks[] = {{99, 1, "no daemon: loopback"}, {199, -1, "daemon attached: crossfaded in"},
        {299, 1, "daemon stalled: crossfaded to loopback"},
        {399, -1, "daemon resumed: crossfaded back in"},
        {499, -1, "fast daemon restart with resync gap: back on the daemon"},
        {599, 1, "daemon crashed (stale heartbeat): loopback"}};
    for (int k = 0; k < 600; k++) {
        if (k == 100) { d.pid = 1001; d.processing = true; d.Heartbeat(); }
        if (k == 200) d.processing = false; // stall: heartbeat fresh, no data
        if (k == 300) d.processing = true;
        if (k == 399) d.processing = false; // daemon A lags one cycle (within the delay)
        if (k == 400) {
            // A dies and a new daemon process B takes over within one cycle:
            // B starts at the driver's write position without writing A's
            // unfinished span, which holds garbage that must never be played.
            d.pid = 1002;
            d.processing = true;
            const int64_t w = WING_LOAD_ACQ(&hdr->to_daemon.write_time);
            const uint32_t n = WING_SHM_RING_FRAMES;
            for (int64_t t = d.cursor; t < w; t++)
                for (uint32_t c = 0; c < kCh; c++)
                    d.From()[(size_t)c * n + wing_shm_ring_index(t, n)] = 7.0f;
            d.cursor = w;
            WING_STORE_REL(&hdr->from_daemon.write_time, w);
            d.Heartbeat();
        }
        if (k == 500) {
            d.processing = false;
            WING_STORE_REL(&hdr->daemon_heartbeat, (uint64_t)1); // stale, not cleared
        }
        if (d.processing) d.Heartbeat();

        io.Cycle(k, in, out);
        d.Process();

        if (std::memcmp(in[0].data(), in[1].data(), sizeof(float) * kBuf * kCh) != 0) differ++;
        const int64_t ti = (int64_t)k * kBuf - kBuf;
        for (UInt32 f = 0; f < kBuf; f++) {
            const int64_t src = ti + f - (int64_t)inLat;
            const float* v = &in[0][f * kCh];
            if (src < (int64_t)kBuf) {
                for (uint32_t c = 0; c < kCh; c++) stale += v[c] != 0.0f;
                continue;
            }
            frames++;
            bool click = false;
            for (uint32_t c = 0; c < kCh; c++) {
                const float a = Level(c);
                if (std::fabs(v[c]) > a + 1e-6f) garbage++;
                if (havePrev) {
                    const double r =
                        std::fabs(v[c] - prev[c]) / (2.0 * a / kFadeFrames * 1.01 + 1e-6);
                    maxRatio = std::max(maxRatio, r);
                    click |= r > 1.0;
                }
                prev[c] = v[c];
            }
            havePrev = true;
            clicks += click;
        }
        for (const Mark& m : marks) {
            if (m.k != k) continue;
            bool exact = true;
            for (UInt32 f = 0; f < kBuf; f++)
                for (uint32_t c = 0; c < kCh; c++)
                    exact &= std::fabs(in[0][f * kCh + c] - m.gain * Level(c)) < 1e-6f;
            const bool flag = WING_LOAD_RLX(&hdr->loopback_active) == (m.gain > 0 ? 1u : 0u);
            Check(exact && flag, m.what);
        }
    }
    std::printf("      fault injection: %ld frames, clicks %ld (max step %.2f x fade slope), "
                "garbage %ld, stale %ld, clients differ %ld cycles\n",
        frames, clicks, maxRatio, garbage, stale, differ);
    Check(stale == 0, "restarted IO session: no audio from the previous session");
    Check(clicks == 0, "all source transitions are crossfades (no clicks)");
    Check(garbage == 0, "stale from_daemon ring data never played after a daemon restart");
    Check(differ == 0, "second client in the same cycle gets identical input");
    Check(WING_LOAD_RLX(&hdr->from_daemon.underruns) > 0, "late daemon blocks counted as underruns");

    (*g_drv)->StopIO(g_drv, dev, 8);
    Check((*g_drv)->StopIO(g_drv, dev, 7) == noErr && WING_LOAD_RLX(&hdr->io_running) == 0,
        "StopIO (last client) clears io_running");
    (*g_drv)->RemoveDeviceClient(g_drv, dev, &ci2);
    (*g_drv)->RemoveDeviceClient(g_drv, dev, &ci);

    shm_unlink(shmName);
    std::printf("%s (%d failures)\n", g_fail ? "FAILED" : "ALL PASSED", g_fail);
    // Do not dlclose: the driver keeps static state, like inside coreaudiod.
    return g_fail ? 1 : 0;
}
