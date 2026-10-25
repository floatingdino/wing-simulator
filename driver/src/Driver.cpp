// Wing Simulator - AudioServerPlugIn (libASPL).
//
// One device "Wing Simulator":
//   48 output channels  DAW -> simulator  (WING "USB in"  1..48)
//   48 input channels   simulator -> DAW  (WING "USB out" 1..48)
//   44.1 kHz / 48 kHz, float32 interleaved.
// Audio is exchanged with the wing-sim daemon through ShmLink (POSIX shm).
// Without a live daemon the device loops output back to input; switching
// between the two is crossfaded (see ShmLink.hpp).
//
// Latency reporting: input(T) = source(T - inputDelay), where inputDelay is the
// configurable window that gives the daemon time to process (Config.hpp). It
// is reported as the input stream latency; device latency and safety offsets
// are 0 in both directions (no converters, no DMA: data is in memory before
// the HAL asks for it). A DAW therefore compensates exactly inputDelay frames
// of round trip.
//
// The driver is intentionally thin: no DSP, no allocation on the IO path, and
// os_log only from non-IO threads (load, StartIO/StopIO, rate changes).

#include "Config.hpp"
#include "ShmLink.hpp"

#include <aspl/Driver.hpp>

#include <CoreAudio/AudioServerPlugIn.h>
#include <os/log.h>

#include <cmath>
#include <cstdlib>
#include <memory>
#include <string>

namespace {

constexpr UInt32 kChannels = WING_SHM_CHANNELS;
constexpr UInt32 kDefaultRate = 48000;
// Default input delay (frames). The selftest sweep measured 0 extra frames
// needed with a time-constraint daemon; 128 is headroom for real DSP time and
// the DAW's share of the cycle (docs/research/driver.md 9.2). Overridden by
// InputDelayFrames in WINGSIM_CONFIG_PATH.
constexpr UInt32 kDefaultInputDelayFrames = WING_SHM_DEFAULT_INPUT_DELAY_FRAMES;
constexpr UInt32 kDeviceLatencyFrames = 0;
constexpr UInt32 kSafetyOffsetFrames = 0;
constexpr Float64 kRates[] = {44100.0, 48000.0};

constexpr const char* kDeviceName = "Wing Simulator";
constexpr const char* kManufacturer = "Wing Simulator Project";
constexpr const char* kDeviceUID = "WingSimulator_Device_UID";
constexpr const char* kModelUID = "WingSimulator_Model_UID";

os_log_t Log()
{
    static os_log_t log = os_log_create("com.wingsimulator.driver", "driver");
    return log;
}

AudioStreamBasicDescription MakeFormat(Float64 rate)
{
    AudioStreamBasicDescription f = {};
    f.mSampleRate = rate;
    f.mFormatID = kAudioFormatLinearPCM;
    f.mFormatFlags =
        kAudioFormatFlagIsFloat | kAudioFormatFlagsNativeEndian | kAudioFormatFlagIsPacked;
    f.mBitsPerChannel = 32;
    f.mChannelsPerFrame = kChannels;
    f.mBytesPerFrame = 4 * kChannels;
    f.mFramesPerPacket = 1;
    f.mBytesPerPacket = 4 * kChannels;
    return f;
}

std::vector<AudioStreamRangedDescription> AvailableFormats()
{
    std::vector<AudioStreamRangedDescription> out;
    for (Float64 r : kRates) {
        AudioStreamRangedDescription d = {};
        d.mFormat = MakeFormat(r);
        d.mSampleRateRange.mMinimum = r;
        d.mSampleRateRange.mMaximum = r;
        out.push_back(d);
    }
    return out;
}

bool IsSupportedRate(Float64 rate)
{
    for (Float64 r : kRates) {
        if (r == rate) {
            return true;
        }
    }
    return false;
}

// Stream that forwards sample-rate changes requested on the stream format to
// the device, so device and both streams always agree.
class WingStream : public aspl::Stream
{
public:
    WingStream(std::shared_ptr<const aspl::Context> ctx,
        std::shared_ptr<aspl::Device> device,
        const aspl::StreamParameters& params)
        : aspl::Stream(ctx, device, params)
        , device_(device)
    {
    }

protected:
    OSStatus SetPhysicalFormatImpl(const AudioStreamBasicDescription& format) override
    {
        if (!IsSupportedRate(format.mSampleRate) || format.mChannelsPerFrame != kChannels) {
            return kAudioDeviceUnsupportedFormatError;
        }
        OSStatus st = aspl::Stream::SetPhysicalFormatImpl(format);
        if (st == kAudioHardwareNoError) {
            st = aspl::Stream::SetVirtualFormatImpl(format);
        }
        if (auto dev = device_.lock(); dev && dev->GetNominalSampleRate() != format.mSampleRate) {
            dev->SetNominalSampleRateAsync(format.mSampleRate);
        }
        return st;
    }

    OSStatus SetVirtualFormatImpl(const AudioStreamBasicDescription& format) override
    {
        return SetPhysicalFormatImpl(format);
    }

private:
    std::weak_ptr<aspl::Device> device_;
};

// Device: adds per-channel names and keeps streams + shm in sync with rate.
class WingDevice : public aspl::Device
{
public:
    WingDevice(std::shared_ptr<const aspl::Context> ctx,
        const aspl::DeviceParameters& params,
        wingsim::ShmLink& link)
        : aspl::Device(ctx, params)
        , link_(link)
    {
    }

    Boolean HasProperty(AudioObjectID objectID,
        pid_t clientPID,
        const AudioObjectPropertyAddress* address) const override
    {
        if (IsChannelName(address)) {
            return true;
        }
        return aspl::Device::HasProperty(objectID, clientPID, address);
    }

    OSStatus IsPropertySettable(AudioObjectID objectID,
        pid_t clientPID,
        const AudioObjectPropertyAddress* address,
        Boolean* outIsSettable) const override
    {
        if (IsChannelName(address)) {
            *outIsSettable = false;
            return kAudioHardwareNoError;
        }
        return aspl::Device::IsPropertySettable(objectID, clientPID, address, outIsSettable);
    }

    OSStatus GetPropertyDataSize(AudioObjectID objectID,
        pid_t clientPID,
        const AudioObjectPropertyAddress* address,
        UInt32 qualifierDataSize,
        const void* qualifierData,
        UInt32* outDataSize) const override
    {
        if (IsChannelName(address)) {
            *outDataSize = sizeof(CFStringRef);
            return kAudioHardwareNoError;
        }
        return aspl::Device::GetPropertyDataSize(
            objectID, clientPID, address, qualifierDataSize, qualifierData, outDataSize);
    }

    OSStatus GetPropertyData(AudioObjectID objectID,
        pid_t clientPID,
        const AudioObjectPropertyAddress* address,
        UInt32 qualifierDataSize,
        const void* qualifierData,
        UInt32 inDataSize,
        UInt32* outDataSize,
        void* outData) const override
    {
        if (IsChannelName(address)) {
            if (inDataSize < sizeof(CFStringRef)) {
                return kAudioHardwareBadPropertySizeError;
            }
            // WING naming from the console's point of view: what the DAW plays
            // arrives at the console as "USB In n"; what the console sends to
            // the DAW is "USB Out n".
            const bool output = address->mScope == kAudioObjectPropertyScopeOutput;
            const std::string name = std::string(output ? "USB In " : "USB Out ") +
                                     std::to_string(address->mElement);
            // Caller owns the returned CFString.
            *static_cast<CFStringRef*>(outData) =
                CFStringCreateWithCString(nullptr, name.c_str(), kCFStringEncodingUTF8);
            *outDataSize = sizeof(CFStringRef);
            return kAudioHardwareNoError;
        }
        return aspl::Device::GetPropertyData(objectID,
            clientPID,
            address,
            qualifierDataSize,
            qualifierData,
            inDataSize,
            outDataSize,
            outData);
    }

protected:
    OSStatus SetNominalSampleRateImpl(Float64 rate) override
    {
        if (!IsSupportedRate(rate)) {
            return kAudioDeviceUnsupportedFormatError;
        }
        OSStatus st = aspl::Device::SetNominalSampleRateImpl(rate);
        if (st != kAudioHardwareNoError) {
            return st;
        }
        // We are inside a HAL configuration change, so these apply in place.
        for (auto dir : {aspl::Direction::Output, aspl::Direction::Input}) {
            for (UInt32 i = 0; i < GetStreamCount(dir); i++) {
                auto s = GetStreamByIndex(dir, i);
                if (s && s->GetPhysicalFormat().mSampleRate != rate) {
                    s->SetPhysicalFormatAsync(MakeFormat(rate));
                }
            }
        }
        link_.BeginEpoch(static_cast<uint32_t>(rate));
        os_log(Log(), "Wing Simulator: sample rate -> %{public}.0f", rate);
        return kAudioHardwareNoError;
    }

private:
    static bool IsChannelName(const AudioObjectPropertyAddress* a)
    {
        return a->mSelector == kAudioObjectPropertyElementName &&
               (a->mScope == kAudioObjectPropertyScopeInput ||
                   a->mScope == kAudioObjectPropertyScopeOutput) &&
               a->mElement >= 1 && a->mElement <= kChannels;
    }

    wingsim::ShmLink& link_;
};

class WingHandler : public aspl::ControlRequestHandler, public aspl::IORequestHandler
{
public:
    WingHandler(wingsim::ShmLink& link, aspl::Device*& device)
        : link_(link)
        , device_(device)
    {
    }

    // Not realtime (HAL StartIO, IO is stopped): may re-map shm and log.
    OSStatus OnStartIO() override
    {
        const auto rate = static_cast<uint32_t>(
            device_ ? device_->GetNominalSampleRate() : static_cast<Float64>(kDefaultRate));
        if (link_.Revalidate(rate)) {
            os_log(Log(), "Wing Simulator: shm %{public}s re-attached (shared=%d) %{public}s",
                link_.Name().c_str(), link_.IsShared() ? 1 : 0, link_.LastError().c_str());
        } else {
            link_.BeginEpoch(rate);
        }
        link_.SetRunning(true);
        link_.ResetMinSlack();
        os_log(Log(), "Wing Simulator: StartIO rate=%u delay=%u shared=%d daemon=%d", rate,
            link_.InputDelayFrames(), link_.IsShared() ? 1 : 0, link_.DaemonAlive() ? 1 : 0);
        return kAudioHardwareNoError;
    }

    void OnStopIO() override
    {
        link_.SetRunning(false);
        const auto s = link_.GetStats();
        os_log(Log(),
            "Wing Simulator: StopIO; totals: to-fallback %llu, to-daemon %llu, "
            "daemon-late blocks %llu, daemon resyncs %llu; this session: min daemon slack "
            "%lld of %u delay frames",
            static_cast<unsigned long long>(s.toFallback),
            static_cast<unsigned long long>(s.toDaemon),
            static_cast<unsigned long long>(s.underrunBlocks),
            static_cast<unsigned long long>(s.resyncs),
            static_cast<long long>(s.minSlack == INT64_MAX ? -1 : s.minSlack),
            link_.InputDelayFrames());
    }

    // RT: DAW playback mix (all clients mixed by HAL) -> to_daemon ring.
    void OnWriteMixedOutput(const std::shared_ptr<aspl::Stream>&,
        Float64,
        Float64 timestamp,
        const void* bytes,
        UInt32 bytesCount) override
    {
        const UInt32 frames = bytesCount / (sizeof(float) * kChannels);
        link_.WriteOutput(static_cast<int64_t>(std::llround(timestamp)),
            static_cast<const float*>(bytes),
            frames,
            kChannels);
    }

    // RT: from_daemon ring (or loopback) -> DAW input.
    void OnReadClientInput(const std::shared_ptr<aspl::Client>&,
        const std::shared_ptr<aspl::Stream>&,
        Float64,
        Float64 timestamp,
        void* bytes,
        UInt32 bytesCount) override
    {
        const UInt32 frames = bytesCount / (sizeof(float) * kChannels);
        link_.ReadInput(static_cast<int64_t>(std::llround(timestamp)),
            static_cast<float*>(bytes),
            frames,
            kChannels);
    }

private:
    wingsim::ShmLink& link_;
    aspl::Device*& device_;
};

std::shared_ptr<aspl::Driver> CreateDriver()
{
    static wingsim::ShmLink link;
    static aspl::Device* devicePtr = nullptr;

    // Test hook: the in-process smoke test loads this bundle outside coreaudiod
    // and must not touch a live segment. Never set inside coreaudiod.
    const char* shmName = std::getenv("WINGSIM_SHM_NAME");
    if (!shmName || !*shmName) {
        shmName = WING_SHM_NAME;
    }

    const auto cfg =
        wingsim::LoadDriverConfig(kDefaultInputDelayFrames, wingsim::ShmLink::kMaxInputDelayFrames);
    os_log(Log(), "Wing Simulator: input delay %u frames (%{public}s)", cfg.inputDelayFrames,
        cfg.source.c_str());
    if (!cfg.warning.empty()) {
        os_log_error(Log(), "Wing Simulator: config: %{public}s", cfg.warning.c_str());
    }

    if (!link.Open(kDefaultRate, shmName, cfg.inputDelayFrames)) {
        os_log_error(Log(), "Wing Simulator: no ring memory: %{public}s", link.LastError().c_str());
    } else if (!link.IsShared()) {
        os_log_error(Log(),
            "Wing Simulator: shm unavailable (%{public}s); loopback only",
            link.LastError().c_str());
    } else {
        os_log(Log(), "Wing Simulator: shm %{public}s attached", shmName);
    }

    auto context = std::make_shared<aspl::Context>();

    aspl::DeviceParameters dp;
    dp.Name = kDeviceName;
    dp.Manufacturer = kManufacturer;
    dp.DeviceUID = kDeviceUID;
    dp.ModelUID = kModelUID;
    dp.SampleRate = kDefaultRate;
    dp.ChannelCount = kChannels;
    dp.CanBeDefault = true;
    dp.CanBeDefaultForSystemSounds = false;
    dp.EnableMixing = true;
    dp.Latency = kDeviceLatencyFrames;
    dp.SafetyOffset = kSafetyOffsetFrames;

    auto device = std::make_shared<WingDevice>(context, dp, link);
    devicePtr = device.get();

    std::vector<AudioValueRange> rates;
    for (Float64 r : kRates) {
        rates.push_back({r, r});
    }
    device->SetAvailableSampleRatesAsync(rates);

    for (auto dir : {aspl::Direction::Output, aspl::Direction::Input}) {
        aspl::StreamParameters sp;
        sp.Direction = dir;
        sp.StartingChannel = 1;
        sp.Format = MakeFormat(kDefaultRate);
        sp.Latency = dir == aspl::Direction::Input ? link.InputDelayFrames() : 0;
        auto stream = std::make_shared<WingStream>(context, device, sp);
        stream->SetAvailablePhysicalFormatsAsync(AvailableFormats());
        stream->SetAvailableVirtualFormatsAsync(AvailableFormats());
        device->AddStreamAsync(stream);
    }

    auto handler = std::make_shared<WingHandler>(link, devicePtr);
    device->SetControlHandler(handler);
    device->SetIOHandler(handler);

    auto plugin = std::make_shared<aspl::Plugin>(context);
    plugin->AddDevice(device);

    return std::make_shared<aspl::Driver>(context, plugin);
}

} // namespace

extern "C" void* WingSimulatorEntryPoint(CFAllocatorRef, CFUUIDRef typeUUID)
{
    if (!CFEqual(typeUUID, kAudioServerPlugInTypeUUID)) {
        return nullptr;
    }
    static std::shared_ptr<aspl::Driver> driver = CreateDriver();
    return driver->GetReference();
}
