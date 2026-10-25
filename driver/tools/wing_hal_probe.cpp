// wing_hal_probe - post-install end-to-end check of the "Wing Simulator" device
// through the public CoreAudio HAL API (what REAPER sees).
//
//   wing_hal_probe [--info] [--rate 48000] [--seconds 3] [--expect-gain 1.0]
//
// Prints device properties (channels, rates, channel names, latencies), then
// runs a full-duplex IOProc that plays a distinct DC level on every output
// channel c (0.01 * (c+1)) and measures the level coming back on input c.
//   expected gain  1.0  -> loopback (no daemon)
//   expected gain -1.0  -> wing_shm_client running with default --gain -1
// Requires microphone permission for the terminal app (input capture).
#include <CoreAudio/CoreAudio.h>
#include <CoreFoundation/CoreFoundation.h>

#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <unistd.h>
#include <vector>

namespace {

constexpr int kMaxCh = 64;

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

struct Meter
{
    std::atomic<bool> measuring{false};
    double sum[kMaxCh] = {};
    long frames = 0;
    std::atomic<long> cycles{0};
};

OSStatus IOProc(AudioObjectID,
    const AudioTimeStamp*,
    const AudioBufferList* inData,
    const AudioTimeStamp*,
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
        ch = 0;
        long frames = 0;
        for (UInt32 b = 0; b < inData->mNumberBuffers; b++) {
            const auto& buf = inData->mBuffers[b];
            const UInt32 nch = buf.mNumberChannels;
            frames = buf.mDataByteSize / (sizeof(float) * nch);
            const auto* p = static_cast<const float*>(buf.mData);
            for (UInt32 f = 0; f < frames; f++)
                for (UInt32 c = 0; c < nch && ch + (int)c < kMaxCh; c++)
                    m->sum[ch + c] += p[f * nch + c];
            ch += nch;
        }
        m->frames += frames;
    }
    m->cycles++;
    return noErr;
}

} // namespace

int main(int argc, char** argv)
{
    std::string name = "Wing Simulator";
    double seconds = 3, expect = 1.0, rate = 0;
    bool infoOnly = false;
    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        if (a == "--seconds" && i + 1 < argc) seconds = atof(argv[++i]);
        else if (a == "--expect-gain" && i + 1 < argc) expect = atof(argv[++i]);
        else if (a == "--rate" && i + 1 < argc) rate = atof(argv[++i]);
        else if (a == "--name" && i + 1 < argc) name = argv[++i];
        else if (a == "--info") infoOnly = true;
        else {
            fprintf(stderr, "usage: wing_hal_probe [--info] [--rate R] [--seconds S] "
                            "[--expect-gain G] [--name N]\n");
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
        a = Addr(kAudioDevicePropertyAvailableNominalSampleRates);
        AudioObjectGetPropertyDataSize(dev, &a, 0, nullptr, &size);
        std::vector<AudioValueRange> rr(size / sizeof(AudioValueRange));
        AudioObjectGetPropertyData(dev, &a, 0, nullptr, &size, rr.data());
        printf("available      ");
        for (auto& v : rr) printf("%.0f ", v.mMinimum);
        printf("\n");
    }
    for (auto scope : {kAudioObjectPropertyScopeOutput, kAudioObjectPropertyScopeInput}) {
        const char* s = scope == kAudioObjectPropertyScopeOutput ? "out" : "in ";
        printf("names %s      1=\"%s\" 2=\"%s\" 48=\"%s\"\n", s,
            StringProp(dev, Addr(kAudioObjectPropertyElementName, scope, 1)).c_str(),
            StringProp(dev, Addr(kAudioObjectPropertyElementName, scope, 2)).c_str(),
            StringProp(dev, Addr(kAudioObjectPropertyElementName, scope, 48)).c_str());
        printf("latency %s    device %u, safety %u\n", s,
            U32Prop(dev, Addr(kAudioDevicePropertyLatency, scope)),
            U32Prop(dev, Addr(kAudioDevicePropertySafetyOffset, scope)));
    }
    bool ok = nIn == 48 && nOut == 48;
    printf("%s  48x48 channels\n", ok ? "PASS" : "FAIL");
    if (infoOnly) return ok ? 0 : 1;

    Meter m;
    AudioDeviceIOProcID procID = nullptr;
    if (AudioDeviceCreateIOProcID(dev, IOProc, &m, &procID) != noErr ||
        AudioDeviceStart(dev, procID) != noErr) {
        printf("FAIL  could not start IO\n");
        return 1;
    }
    usleep(500000); // settle: fill ring + input delay
    m.measuring = true;
    usleep((useconds_t)(seconds * 1e6));
    m.measuring = false;
    AudioDeviceStop(dev, procID);
    AudioDeviceDestroyIOProcID(dev, procID);

    if (m.frames == 0) {
        printf("FAIL  no input frames (microphone permission for this terminal?)\n");
        return 1;
    }
    double gmin = 1e9, gmax = -1e9;
    const int n = (int)std::min<UInt32>(std::min(nIn, nOut), kMaxCh);
    for (int c = 0; c < n; c++) {
        const double g = (m.sum[c] / m.frames) / (0.01 * (c + 1));
        gmin = std::min(gmin, g);
        gmax = std::max(gmax, g);
    }
    printf("io             %ld cycles, %ld input frames measured\n", m.cycles.load(), m.frames);
    printf("gain in/out    min %.4f max %.4f (expected %.4f)\n", gmin, gmax, expect);
    const bool pass = std::fabs(gmin - expect) < 0.02 && std::fabs(gmax - expect) < 0.02;
    printf("%s  end-to-end gain on all %d channels\n", pass ? "PASS" : "FAIL", n);
    return ok && pass ? 0 : 1;
}
