// wing_driver_smoketest - loads the built WingSimulator.driver bundle in-process
// and drives it through the AudioServerPlugIn C interface the way coreaudiod
// does (fake host). Checks without root/install:
//   - plug-in factory + Initialize work, one device "Wing Simulator"
//   - 48-ch output stream + 48-ch input stream, float32, 44.1k/48k available
//   - channel names, input stream latency == shm input delay
//   - sample-rate change 48k -> 44.1k -> 48k propagates to streams and shm
//   - IO cycles: loopback input(T) == output(T - delay)
//
//   wing_driver_smoketest path/to/WingSimulator.driver
#include "wing_shm.h"

#include <CoreAudio/AudioServerPlugIn.h>
#include <CoreFoundation/CoreFoundation.h>

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

} // namespace

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::fprintf(stderr, "usage: wing_driver_smoketest WingSimulator.driver\n");
        return 2;
    }
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
    Check(has441 && has48, "available rates include 44100 and 48000");

    Check(GetStr(dev, kAudioObjectPropertyElementName, kAudioObjectPropertyScopeOutput, 1) ==
              "USB In 1",
        "output ch 1 named 'USB In 1'");
    Check(GetStr(dev, kAudioObjectPropertyElementName, kAudioObjectPropertyScopeInput, 48) ==
              "USB Out 48",
        "input ch 48 named 'USB Out 48'");

    UInt32 inLat = 0;
    Get(si, kAudioStreamPropertyLatency, inLat);
    Check(inLat == WING_SHM_DEFAULT_INPUT_DELAY_FRAMES, "input stream latency == shm input delay");

    // Map the segment the driver created to observe it.
    int fd = shm_open(shmName, O_RDWR, 0);
    Check(fd >= 0, "driver created POSIX shm segment");
    if (fd < 0) return 1;
    auto* hdr = static_cast<wing_shm_header*>(
        mmap(nullptr, WING_SHM_TOTAL_BYTES, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0));
    close(fd);
    Check(wing_shm_validate_header(hdr, WING_SHM_TOTAL_BYTES) == 0, "segment header valid");

    // Sample rate change through the HAL path.
    for (Float64 want : {44100.0, 48000.0}) {
        AudioObjectPropertyAddress a = {kAudioDevicePropertyNominalSampleRate,
            kAudioObjectPropertyScopeGlobal, 0};
        (*g_drv)->SetPropertyData(g_drv, dev, 0, &a, 0, nullptr, sizeof want, &want);
        RunPendingChanges();
        Float64 got = 0;
        Get(dev, kAudioDevicePropertyNominalSampleRate, got);
        Get(so, kAudioStreamPropertyPhysicalFormat, fo);
        Get(si, kAudioStreamPropertyVirtualFormat, fi);
        Check(got == want && fo.mSampleRate == want && fi.mSampleRate == want &&
                  WING_LOAD_RLX(&hdr->sample_rate) == (uint32_t)want,
            "rate change to " + std::to_string((int)want) + " reaches device, streams, shm");
    }

    // IO: one client, loopback (no daemon).
    AudioServerPlugInClientInfo ci = {7, getpid(), true, nullptr};
    Check((*g_drv)->AddDeviceClient(g_drv, dev, &ci) == noErr, "AddDeviceClient");
    const uint32_t epochBefore = WING_LOAD_RLX(&hdr->io_epoch);
    Check((*g_drv)->StartIO(g_drv, dev, 7) == noErr, "StartIO");
    Check(WING_LOAD_RLX(&hdr->io_epoch) == epochBefore + 1 && WING_LOAD_RLX(&hdr->io_running) == 1,
        "StartIO begins a new shm epoch");

    const UInt32 buf = 256;
    std::vector<float> out(buf * 48), in(buf * 48);
    long checked = 0, bad = 0;
    for (int k = 0; k < 200; k++) {
        const Float64 now = (Float64)k * buf;
        AudioServerPlugInIOCycleInfo cyc = {};
        cyc.mIOCycleCounter = k;
        cyc.mNominalIOBufferFrameSize = buf;
        cyc.mInputTime.mSampleTime = now - buf;
        cyc.mInputTime.mFlags = kAudioTimeStampSampleTimeValid;
        cyc.mOutputTime.mSampleTime = now + buf;
        cyc.mOutputTime.mFlags = kAudioTimeStampSampleTimeValid;

        Boolean will = false, inPlace = false;
        (*g_drv)->WillDoIOOperation(g_drv, dev, 7, kAudioServerPlugInIOOperationReadInput, &will,
            &inPlace);
        if (k == 0) Check(will, "device does ReadInput");
        (*g_drv)->BeginIOOperation(g_drv, dev, 7, kAudioServerPlugInIOOperationReadInput, buf,
            &cyc);
        (*g_drv)->DoIOOperation(g_drv, dev, si, 7, kAudioServerPlugInIOOperationReadInput, buf,
            &cyc, in.data(), nullptr);
        (*g_drv)->EndIOOperation(g_drv, dev, 7, kAudioServerPlugInIOOperationReadInput, buf, &cyc);

        for (UInt32 f = 0; f < buf; f++) {
            const int64_t src = (int64_t)(now - buf) + f - (int64_t)inLat;
            if (src < buf) continue; // first output written at sample time `buf`
            for (uint32_t c = 0; c < 48; c++) {
                if (std::fabs(in[f * 48 + c] - Sig(c, src)) > 1e-6f) {
                    bad++;
                    break;
                }
            }
            checked++;
        }

        for (UInt32 f = 0; f < buf; f++)
            for (uint32_t c = 0; c < 48; c++)
                out[f * 48 + c] = Sig(c, (int64_t)(now + buf) + f);
        (*g_drv)->WillDoIOOperation(g_drv, dev, 7, kAudioServerPlugInIOOperationWriteMix, &will,
            &inPlace);
        if (k == 0) Check(will, "device does WriteMix");
        (*g_drv)->BeginIOOperation(g_drv, dev, 7, kAudioServerPlugInIOOperationWriteMix, buf,
            &cyc);
        (*g_drv)->DoIOOperation(g_drv, dev, so, 7, kAudioServerPlugInIOOperationWriteMix, buf, &cyc,
            out.data(), nullptr);
        (*g_drv)->EndIOOperation(g_drv, dev, 7, kAudioServerPlugInIOOperationWriteMix, buf, &cyc);
    }
    std::printf("      loopback via HAL interface: %ld frames checked, %ld bad\n", checked, bad);
    Check(checked > 0 && bad == 0, "loopback through DoIOOperation: input(T) == output(T - delay)");
    Check(WING_LOAD_RLX(&hdr->loopback_active) == 1, "driver reports loopback mode (no daemon)");

    Check((*g_drv)->StopIO(g_drv, dev, 7) == noErr, "StopIO");
    (*g_drv)->RemoveDeviceClient(g_drv, dev, &ci);

    shm_unlink(shmName);
    std::printf("%s (%d failures)\n", g_fail ? "FAILED" : "ALL PASSED", g_fail);
    // Do not dlclose: the driver keeps static state, like inside coreaudiod.
    return g_fail ? 1 : 0;
}
