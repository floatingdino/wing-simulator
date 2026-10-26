import Foundation

/// Mirror of wing-sim's `control::DaemonStatus` (`<state-dir>/status.json`).
public struct DaemonStatus: Codable, Equatable, Sendable {
    public var running: Bool
    public var pid: Int32
    public var version: String
    /// Source commit and build time of the daemon binary (T087); optional for older daemons.
    public var commit: String?
    public var builtUnix: Double?
    public var name: String
    public var updatedUnix: Double
    public var uptimeS: Double
    public var stateDir: String
    public var nativeClients: Int
    public var meterReports: Int
    public var discoveryReplies: UInt64
    public var sceneFile: String?
    public var activeScene: String?
    public var hostioPath: String?
    public var hostio: String
    public var audio: AudioReport?
    /// Recorder (T061); optional so documents from older daemons still decode.
    public var recording: Bool?
    public var recorder: String?
    /// Monitor level knobs [A (PHONES), B (SPEAKERS)] in dB, -144 = -oo (T03F; `nil` with
    /// `--no-audio` or from older daemons). Set with the control command `monitor-level`.
    public var monitorLevelsDb: [Double]?

    /// Decode the JSON document the daemon writes.
    public static func decode(_ data: Data) throws -> DaemonStatus {
        let d = JSONDecoder()
        d.keyDecodingStrategy = .convertFromSnakeCase
        return try d.decode(DaemonStatus.self, from: data)
    }

    /// Whether this document describes a live daemon: it says it is running, was written in the
    /// last `maxAge` seconds and its process exists.
    public func isLive(now: Date = .now, maxAge: TimeInterval = 5,
                       processExists: (Int32) -> Bool = DaemonStatus.processExists) -> Bool {
        running && now.timeIntervalSince1970 - updatedUnix <= maxAge && processExists(pid)
    }

    /// `kill(pid, 0)`: the process exists (EPERM: exists but not ours).
    public static func processExists(_ pid: Int32) -> Bool {
        guard pid > 0 else { return false }
        return kill(pid, 0) == 0 || errno == EPERM
    }
}

/// Mirror of `audio::driver::AudioReport`.
public struct AudioReport: Codable, Equatable, Sendable {
    public var attached: Bool
    public var rate: UInt32
    public var driver: String
    public var loadPct: Double
    public var peakLoadPct: Double
    public var cycleP50Us: UInt64
    public var cycleP99Us: UInt64
    public var xruns: UInt32
    public var underruns: UInt32
    public var overruns: UInt32
    public var slowCycles: UInt64
    public var bridges: [BridgeReport]
    public var message: String
}

/// Mirror of `audio::hostio::BridgeReport`.
public struct BridgeReport: Codable, Equatable, Sendable, Identifiable {
    public var device: String
    public var state: String
    public var fillMs: Double
    public var targetMs: Double
    public var e2eMs: Double?
    public var correctionPpm: Double
    public var driftPpm: Double
    public var underruns: UInt64
    public var resyncs: UInt64

    public var id: String { device }
}

/// Reads the status file.
public struct StatusReader: Sendable {
    public let url: URL

    public init(stateDir: URL) {
        url = stateDir.appending(path: DaemonPaths.statusFile)
    }

    /// The current document, or `nil` when there is none (or it is unreadable/partial).
    public func read() -> DaemonStatus? {
        guard let data = try? Data(contentsOf: url) else { return nil }
        return try? DaemonStatus.decode(data)
    }
}
