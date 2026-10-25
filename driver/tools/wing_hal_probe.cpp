// wing_hal_probe - post-install end-to-end check of the "Wing Simulator" device
// through the public CoreAudio HAL API (what REAPER sees).
//
//   wing_hal_probe [--info] [--rate 48000] [--buffer 128] [--seconds 3]
//                  [--expect-gain 1.0 | --transition] [--expect-latency N]
//                  [--strict] [--shm [/wingsim.audio.v1]] [--name "Wing Simulator"]
//
// Prints device properties (channels, rates, channel names, latencies, safety
// offsets), then runs a full-duplex IOProc that plays a distinct DC level
// 0.01 * (c+1) on every output channel c and checks what comes back on input c:
//   --expect-gain  1.0 -> loopback (no daemon)
//   --expect-gain -1.0 -> daemon running with gain -1 (gain_client / wing_shm_client)
//   --transition       -> the source may change during the run (daemon started,
//                         killed or stalled): only continuity is checked
// Always measured: clicks (a frame-to-frame step larger than a crossfade can
// make), HAL overloads (kAudioDeviceProcessorOverload), input timestamp gaps.
// --strict additionally fails on any glitch frame (|in - gain * level| > 1e-3
// after the settle time), overload, timestamp gap or (with --shm) driver-side
// from_daemon underrun. --buffer sets this process's IOBufferFrameSize.
// --expect-latency N checks total input latency (device + stream + safety)
// == N frames and total output latency == 0.
//
// Requires microphone permission for the terminal app (input capture).
#include "wing_shm.h"

#include <CoreAudio/CoreAudio.h>
#include <CoreFoundation/CoreFoundation.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fcntl.h>
#include <string>
#include <sys/mman.h>
#include <unistd.h>
#include <vector>

namespace {

constexpr int kMaxCh = 64;
constexpr float kFadeFrames = 128; // wingsim::ShmLink::kFadeFrames

std::string CFToStd(CFStringRef s)
{
    if (!s) return "";
    char buf[256] = {};
    CFStringGetCString(s, buf, sizeof buf, kCFStringEncodingUTF8);
    return buf;
}

AudioObjectPropertyAddress Addr(AudioObjectPropertySelector sel,
    AudioObjectPropertyScope scope = kAudioObjectPropertyScopeGlobal,
    AudioObjectPropertyElement el = kAudioObjectPropertyElementMain)
{
    return {sel, scope, el};
}

std::string StringProp(AudioObjectID obj, AudioObjectPropertyAddress a)
{
    CFStringRef s = nullptr;
    UInt32 size = sizeof s;
    if (AudioObjectGetPropertyData(obj, &a, 0, nullptr, &size, &s) != noErr || !s) return "";
    std::string r = CFToStd(s);
    CFRelease(s);
    return r;
}

UInt32 U32Prop(AudioObjectID obj, AudioObjectPropertyAddress a)
{
    UInt32 v = 0, size = sizeof v;
    AudioObjectGetPropertyData(obj, &a, 0, nullptr, &size, &v);
    return v;
}

AudioObjectID FindDevice(const std::string& name)
{
    auto a = Addr(kAudioHardwarePropertyDevices);
    UInt32 size = 0;
    AudioObjectGetPropertyDataSize(kAudioObjectSystemObject, &a, 0, nullptr, &size);
    std::vector<AudioObjectID> ids(size / sizeof(AudioObjectID));
    AudioObjectGetPropertyData(kAudioObjectSystemObject, &a, 0, nullptr, &size, ids.data());
    for (auto id : ids) {
        if (StringProp(id, Addr(kAudioObjectPropertyName)) == name) return id;
    }
    return kAudioObjectUnknown;
}

UInt32 ChannelCount(AudioObjectID dev, AudioObjectPropertyScope scope)
{
    auto a = Addr(kAudioDevicePropertyStreamConfiguration, scope);
    UInt32 size = 0;
    if (AudioObjectGetPropertyDataSize(dev, &a, 0, nullptr, &size) != noErr) return 0;
    std::vector<uint8_t> buf(size);
    auto* abl = reinterpret_cast<AudioBufferList*>(buf.data());
    if (AudioObjectGetPropertyData(dev, &a, 0, nullptr, &size, abl) != noErr) return 0;
    UInt32 n = 0;
    for (UInt32 i = 0; i < abl->mNumberBuffers; i++) n += abl->mBuffers[i].mNumberChannels;
    return n;
}

// Sum of stream latencies (first stream) in `scope`.
UInt32 StreamLatency(AudioObjectID dev, AudioObjectPropertyScope scope)
{
    auto a = Addr(kAudioDevicePropertyStreams, scope);
    UInt32 size = 0;
    if (AudioObjectGetPropertyDataSize(dev, &a, 0, nullptr, &size) != noErr || size == 0) return 0;
    std::vector<AudioObjectID> s(size / sizeof(AudioObjectID));
    AudioObjectGetPropertyData(dev, &a, 0, nullptr, &size, s.data());
    return U32Prop(s[0], Addr(kAudioStreamPropertyLatency));
}

struct Meter
{
    float expect = 1.0f;
    bool transition = false;
    std::atomic<bool> measuring{false};
    double sum[kMaxCh] = {};
    float prev[kMaxCh] = {};
    bool havePrev = false;
    long frames = 0;
    long glitchFrames = 0;
    long clicks = 0;
    double maxStep = 0; // in units of the largest crossfade step
    long tsGaps = 0;
    Float64 nextInTime = -1;
    std::atomic<long> cycles{0};
    std::atomic<long> overloads{0};
};

OSStatus IOProc(AudioObjectID,
    const AudioTimeStamp*,
    const AudioBufferList* inData,
    const AudioTimeStamp* inTime,
    AudioBufferList* outData,
    const AudioTimeStamp*,
    void* ctx)
{
    auto* m = static_cast<Meter*>(ctx);
    int ch = 0;
    for (UInt32 b = 0; b < outData->mNumberBuffers; b++) {
        auto& buf = outData->mBuffers[b];
        const UInt32 nch = buf.mNumberChannels;
        const UInt32 frames = buf.mDataByteSize / (sizeof(float) * nch);
        auto* p = static_cast<float*>(buf.mData);
        for (UInt32 f = 0; f < frames; f++)
            for (UInt32 c = 0; c < nch; c++) p[f * nch + c] = 0.01f * (float)(ch + c + 1);
        ch += nch;
    }
    if (m->measuring) {
        long frames = 0;
        // Interleaved single buffer (what the driver provides).
        if (inData->mNumberBuffers >= 1) {
            const auto& buf = inData->mBuffers[0];
            const UInt32 nch = std::min<UInt32>(buf.mNumberChannels, kMaxCh);
            const UInt32 stride = buf.mNumberChannels;
            frames = buf.mDataByteSize / (sizeof(float) * stride);
            const auto* p = static_cast<const float*>(buf.mData);
            if (m->nextInTime >= 0 && inTime &&
                std::fabs(inTime->mSampleTime - m->nextInTime) > 0.5) {
                m->tsGaps++;
                m->havePrev = false;
            }
            if (inTime) m->nextInTime = inTime->mSampleTime + frames;
            for (UInt32 f = 0; f < (UInt32)frames; f++) {
                bool glitch = false, click = false;
                for (UInt32 c = 0; c < nch; c++) {
                    const float a = 0.01f * (float)(c + 1);
                    const float v = p[f * stride + c];
                    m->sum[c] += v;
                    if (!m->transition && std::fabs(v - m->expect * a) > 1e-3f) glitch = true;
                    if (m->havePrev) {
                        const double r = std::fabs(v - m->prev[c]) / (2.0 * a / kFadeFrames);
                        if (r > m->maxStep) m->maxStep = r;
                        if (r > 1.05) click = true;
                    }
                    m->prev[c] = v;
                }
                m->havePrev = true;
                m->glitchFrames += glitch;
                m->clicks += click;
            }
        }
        m->frames += frames;
    }
    m->cycles++;
    return noErr;
}

OSStatus OnOverload(AudioObjectID, UInt32, const AudioObjectPropertyAddress*, void* ctx)
{
    static_cast<Meter*>(ctx)->overloads++;
    return noErr;
}

struct ShmView
{
    const wing_shm_header* h = nullptr;
    bool Open(const std::string& name)
    {
        int fd = shm_open(name.c_str(), O_RDONLY, 0);
        if (fd < 0) return false;
        void* p = mmap(nullptr, WING_SHM_HEADER_BYTES, PROT_READ, MAP_SHARED, fd, 0);
        close(fd);
        if (p == MAP_FAILED) return false;
        h = static_cast<const wing_shm_header*>(p);
        return true;
    }
    uint32_t Underruns() const { return h ? WING_LOAD_RLX(&h->from_daemon.underruns) : 0; }
    uint32_t Overruns() const { return h ? WING_LOAD_RLX(&h->to_daemon.overruns) : 0; }
};

} // namespace

int main(int argc, char** argv)
{
    std::string name = "Wing Simulator", shmName;
    double seconds = 3, expect = 1.0, rate = 0;
    long expectLatency = -1;
    UInt32 buffer = 0;
    bool infoOnly = false, transition = false, strict = false;
    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        if (a == "--seconds" && i + 1 < argc) seconds = atof(argv[++i]);
        else if (a == "--expect-gain" && i + 1 < argc) expect = atof(argv[++i]);
        else if (a == "--rate" && i + 1 < argc) rate = atof(argv[++i]);
        else if (a == "--buffer" && i + 1 < argc) buffer = (UInt32)atoi(argv[++i]);
        else if (a == "--expect-latency" && i + 1 < argc) expectLatency = atol(argv[++i]);
        else if (a == "--name" && i + 1 < argc) name = argv[++i];
        else if (a == "--shm") {
            shmName = (i + 1 < argc && argv[i + 1][0] == '/') ? argv[++i] : WING_SHM_NAME;
        } else if (a == "--info") infoOnly = true;
        else if (a == "--transition") transition = true;
        else if (a == "--strict") strict = true;
        else {
            fprintf(stderr, "usage: wing_hal_probe [--info] [--rate R] [--buffer N] [--seconds S] "
                            "[--expect-gain G | --transition] [--expect-latency N] [--strict] "
                            "[--shm [NAME]] [--name N]\n");
            return 2;
        }
    }

    AudioObjectID dev = FindDevice(name);
    if (dev == kAudioObjectUnknown) {
        printf("FAIL  device \"%s\" not found (installed? coreaudiod restarted?)\n", name.c_str());
        return 1;
    }
    const UInt32 nIn = ChannelCount(dev, kAudioObjectPropertyScopeInput);
    const UInt32 nOut = ChannelCount(dev, kAudioObjectPropertyScopeOutput);
    printf("device         \"%s\" id=%u uid=%s\n", name.c_str(), dev,
        StringProp(dev, Addr(kAudioDevicePropertyDeviceUID)).c_str());
    printf("channels       out %u (DAW->sim), in %u (sim->DAW)\n", nOut, nIn);
    bool ok = true;

    if (rate > 0) {
        auto a = Addr(kAudioDevicePropertyNominalSampleRate);
        Float64 r = rate;
        OSStatus st = AudioObjectSetPropertyData(dev, &a, 0, nullptr, sizeof r, &r);
        printf("set rate       %.0f -> %s\n", rate, st == noErr ? "ok" : "error");
        usleep(300000);
    }
    {
        auto a = Addr(kAudioDevicePropertyNominalSampleRate);
        Float64 r = 0;
        UInt32 size = sizeof r;
        AudioObjectGetPropertyData(dev, &a, 0, nullptr, &size, &r);
        printf("nominal rate   %.0f\n", r);
        if (rate > 0 && r != rate) {
            printf("FAIL  nominal rate %.0f != requested %.0f\n", r, rate);
            ok = false;
        }
        a = Addr(kAudioDevicePropertyAvailableNominalSampleRates);
        AudioObjectGetPropertyDataSize(dev, &a, 0, nullptr, &size);
        std::vector<AudioValueRange> rr(size / sizeof(AudioValueRange));
        AudioObjectGetPropertyData(dev, &a, 0, nullptr, &size, rr.data());
        printf("available      ");
        for (auto& v : rr) printf("%.0f ", v.mMinimum);
        printf("\n");
    }
    bool names = true;
    for (auto scope : {kAudioObjectPropertyScopeOutput, kAudioObjectPropertyScopeInput}) {
        const bool out = scope == kAudioObjectPropertyScopeOutput;
        const char* s = out ? "out" : "in ";
        printf("names %s      1=\"%s\" 2=\"%s\" 48=\"%s\"\n", s,
            StringProp(dev, Addr(kAudioObjectPropertyElementName, scope, 1)).c_str(),
            StringProp(dev, Addr(kAudioObjectPropertyElementName, scope, 2)).c_str(),
            StringProp(dev, Addr(kAudioObjectPropertyElementName, scope, 48)).c_str());
        for (UInt32 n = 1; n <= 48; n++) {
            names &= StringProp(dev, Addr(kAudioObjectPropertyElementName, scope, n)) ==
                     std::string(out ? "USB In " : "USB Out ") + std::to_string(n);
        }
        const UInt32 dl = U32Prop(dev, Addr(kAudioDevicePropertyLatency, scope));
        const UInt32 sl = StreamLatency(dev, scope);
        const UInt32 so = U32Prop(dev, Addr(kAudioDevicePropertySafetyOffset, scope));
        printf("latency %s    device %u, stream %u, safety %u (total %u frames)\n", s, dl, sl, so,
            dl + sl + so);
        if (expectLatency >= 0) {
            const long want = out ? 0 : expectLatency;
            const bool pass = (long)(dl + sl + so) == want;
            printf("%s  total %s latency == %ld\n", pass ? "PASS" : "FAIL", out ? "output" : "input",
                want);
            ok &= pass;
        }
    }
    printf("%s  channel names USB In/Out 1..48\n", names ? "PASS" : "FAIL");
    const bool chOk = nIn == 48 && nOut == 48;
    printf("%s  48x48 channels\n", chOk ? "PASS" : "FAIL");
    ok &= names && chOk;
    if (infoOnly) return ok ? 0 : 1;

    if (buffer > 0) {
        auto a = Addr(kAudioDevicePropertyBufferFrameSize);
        UInt32 b = buffer;
        AudioObjectSetPropertyData(dev, &a, 0, nullptr, sizeof b, &b);
        const UInt32 got = U32Prop(dev, a);
        printf("io buffer      %u frames (requested %u)\n", got, buffer);
        if (got != buffer) {
            printf("FAIL  IOBufferFrameSize %u != %u (another client forcing a smaller size?)\n",
                got, buffer);
            ok = false;
        }
    }

    ShmView shm;
    if (!shmName.empty() && !shm.Open(shmName)) {
        printf("FAIL  cannot map shm %s\n", shmName.c_str());
        ok = false;
    }

    Meter m;
    m.expect = (float)expect;
    m.transition = transition;
    auto ovAddr = Addr(kAudioDeviceProcessorOverload);
    AudioObjectAddPropertyListener(dev, &ovAddr, OnOverload, &m);
    AudioDeviceIOProcID procID = nullptr;
    if (AudioDeviceCreateIOProcID(dev, IOProc, &m, &procID) != noErr ||
        AudioDeviceStart(dev, procID) != noErr) {
        printf("FAIL  could not start IO\n");
        return 1;
    }
    usleep(500000); // settle: fill ring + input delay
    const uint32_t under0 = shm.Underruns(), over0 = shm.Overruns();
    const long ov0 = m.overloads.load();
    m.measuring = true;
    usleep((useconds_t)(seconds * 1e6));
    m.measuring = false;
    usleep(50000); // let the IOProc finish its current cycle
    const long overloads = m.overloads.load() - ov0;
    const uint32_t under = shm.Underruns() - under0, over = shm.Overruns() - over0;
    AudioDeviceStop(dev, procID);
    AudioDeviceDestroyIOProcID(dev, procID);
    AudioObjectRemovePropertyListener(dev, &ovAddr, OnOverload, &m);

    if (m.frames == 0) {
        printf("FAIL  no input frames (microphone permission for this terminal?)\n");
        return 1;
    }
    printf("io             %ld cycles, %ld input frames measured (%.1f s)\n", m.cycles.load(),
        m.frames, seconds);
    printf("xruns          %ld HAL overloads, %ld input timestamp gaps\n", overloads, m.tsGaps);
    printf("continuity     %ld clicks (max step %.2f x crossfade step)\n", m.clicks, m.maxStep);
    if (shm.h) {
        printf("shm            from_daemon underruns +%u, to_daemon overruns +%u, loopback %u, "
               "input delay %u\n",
            under, over, WING_LOAD_RLX(&shm.h->loopback_active), shm.h->input_delay_frames);
    }
    const bool clean = m.clicks == 0;
    printf("%s  no clicks\n", clean ? "PASS" : "FAIL");
    ok &= clean;
    if (!transition) {
        double gmin = 1e9, gmax = -1e9;
        const int n = (int)std::min<UInt32>(std::min(nIn, nOut), kMaxCh);
        for (int c = 0; c < n; c++) {
            const double g = (m.sum[c] / m.frames) / (0.01 * (c + 1));
            gmin = std::min(gmin, g);
            gmax = std::max(gmax, g);
        }
        printf("gain in/out    min %.4f max %.4f (expected %.4f), glitch frames %ld\n", gmin, gmax,
            expect, m.glitchFrames);
        const bool pass = std::fabs(gmin - expect) < 0.02 && std::fabs(gmax - expect) < 0.02;
        printf("%s  end-to-end gain on all %d channels\n", pass ? "PASS" : "FAIL", n);
        ok &= pass;
    }
    if (strict) {
        const bool s = overloads == 0 && m.tsGaps == 0 && under == 0 &&
                       (transition || m.glitchFrames == 0);
        printf("%s  strict: no overloads, timestamp gaps, glitch frames or shm underruns\n",
            s ? "PASS" : "FAIL");
        ok &= s;
    }
    return ok ? 0 : 1;
}
