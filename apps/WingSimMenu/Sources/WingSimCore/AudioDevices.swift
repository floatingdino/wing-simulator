import CoreAudio
import Foundation

/// A CoreAudio device with output channels.
public struct OutputDevice: Equatable, Hashable, Sendable, Identifiable {
    public var name: String
    public var uid: String
    public var outputChannels: Int
    public var id: String { uid }

    public init(name: String, uid: String, outputChannels: Int) {
        self.name = name
        self.uid = uid
        self.outputChannels = outputChannels
    }
}

/// Lists CoreAudio output devices (read-only HAL queries).
public enum AudioDevices {
    /// The Wing Simulator's own virtual device: never a host-output target (it would feed back).
    public static let simulatorDeviceName = "Wing Simulator"

    /// Devices with at least one output channel, sorted by name; excludes the simulator's
    /// own device.
    public static func outputDevices() -> [OutputDevice] {
        var addr = AudioObjectPropertyAddress(
            mSelector: kAudioHardwarePropertyDevices,
            mScope: kAudioObjectPropertyScopeGlobal,
            mElement: kAudioObjectPropertyElementMain)
        var size: UInt32 = 0
        guard AudioObjectGetPropertyDataSize(AudioObjectID(kAudioObjectSystemObject), &addr, 0, nil, &size) == noErr
        else { return [] }
        var ids = [AudioObjectID](repeating: 0, count: Int(size) / MemoryLayout<AudioObjectID>.size)
        guard AudioObjectGetPropertyData(AudioObjectID(kAudioObjectSystemObject), &addr, 0, nil, &size, &ids) == noErr
        else { return [] }
        return ids.compactMap { id -> OutputDevice? in
            let chans = outputChannelCount(id)
            guard chans > 0, let name = string(id, kAudioObjectPropertyName),
                  name != simulatorDeviceName
            else { return nil }
            return OutputDevice(name: name, uid: string(id, kAudioDevicePropertyDeviceUID) ?? name,
                                outputChannels: chans)
        }
        .sorted { $0.name.localizedStandardCompare($1.name) == .orderedAscending }
    }

    private static func string(_ id: AudioObjectID, _ selector: AudioObjectPropertySelector) -> String? {
        var addr = AudioObjectPropertyAddress(
            mSelector: selector, mScope: kAudioObjectPropertyScopeGlobal,
            mElement: kAudioObjectPropertyElementMain)
        var value: Unmanaged<CFString>?
        var size = UInt32(MemoryLayout<Unmanaged<CFString>?>.size)
        guard AudioObjectGetPropertyData(id, &addr, 0, nil, &size, &value) == noErr,
              let v = value?.takeRetainedValue()
        else { return nil }
        return v as String
    }

    private static func outputChannelCount(_ id: AudioObjectID) -> Int {
        var addr = AudioObjectPropertyAddress(
            mSelector: kAudioDevicePropertyStreamConfiguration,
            mScope: kAudioObjectPropertyScopeOutput,
            mElement: kAudioObjectPropertyElementMain)
        var size: UInt32 = 0
        guard AudioObjectGetPropertyDataSize(id, &addr, 0, nil, &size) == noErr, size > 0 else { return 0 }
        let raw = UnsafeMutableRawPointer.allocate(byteCount: Int(size),
                                                   alignment: MemoryLayout<AudioBufferList>.alignment)
        defer { raw.deallocate() }
        guard AudioObjectGetPropertyData(id, &addr, 0, nil, &size, raw) == noErr else { return 0 }
        let list = UnsafeMutableAudioBufferListPointer(raw.assumingMemoryBound(to: AudioBufferList.self))
        return list.reduce(0) { $0 + Int($1.mNumberChannels) }
    }
}
