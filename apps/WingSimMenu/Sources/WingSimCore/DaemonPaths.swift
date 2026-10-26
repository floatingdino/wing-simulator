import Foundation

/// Where the daemon keeps its state and where its binaries are.
public struct DaemonPaths: Sendable, Equatable {
    public static let statusFile = "status.json"
    public static let socketFile = "control.sock"
    public static let hostioFile = "hostio.toml"
    public static let logFile = "wing-sim.log"
    /// Info.plist key build.sh sets to the repo root (fallback binary location).
    public static let repoRootKey = "WingSimRepoRoot"

    public var stateDir: URL

    public init(stateDir: URL) {
        self.stateDir = stateDir
    }

    /// `$WINGSIM_STATE_DIR`, else `~/Library/Application Support/WingSimulator` (the daemon's
    /// default `--state-dir`).
    public static func standard(environment: [String: String] = ProcessInfo.processInfo.environment)
        -> DaemonPaths
    {
        if let d = environment["WINGSIM_STATE_DIR"], !d.isEmpty {
            return DaemonPaths(stateDir: URL(filePath: d, directoryHint: .isDirectory))
        }
        let home = FileManager.default.homeDirectoryForCurrentUser
        return DaemonPaths(stateDir: home.appending(path: "Library/Application Support/WingSimulator",
                                                    directoryHint: .isDirectory))
    }

    public var statusURL: URL { stateDir.appending(path: Self.statusFile) }
    public var socketURL: URL { stateDir.appending(path: Self.socketFile) }
    public var hostioURL: URL { stateDir.appending(path: Self.hostioFile) }
    public var logURL: URL { stateDir.appending(path: Self.logFile) }
}

/// Finds the `wing-sim` / `wing-diff` executables.
public struct BinaryLocator: Sendable {
    /// Candidate directories in order.
    public var searchDirs: [URL]

    public init(searchDirs: [URL]) {
        self.searchDirs = searchDirs
    }

    /// `$WINGSIM_BIN_DIR`, the app bundle's `Contents/Resources/bin`, then
    /// `<repo>/target/release` from the bundle's `WingSimRepoRoot` key.
    public static func standard(bundle: Bundle = .main,
                                environment: [String: String] = ProcessInfo.processInfo.environment)
        -> BinaryLocator
    {
        var dirs: [URL] = []
        if let d = environment["WINGSIM_BIN_DIR"], !d.isEmpty {
            dirs.append(URL(filePath: d, directoryHint: .isDirectory))
        }
        if let r = bundle.resourceURL {
            dirs.append(r.appending(path: "bin", directoryHint: .isDirectory))
        }
        if let root = bundle.object(forInfoDictionaryKey: DaemonPaths.repoRootKey) as? String, !root.isEmpty {
            dirs.append(URL(filePath: root, directoryHint: .isDirectory)
                .appending(path: "target/release", directoryHint: .isDirectory))
        }
        return BinaryLocator(searchDirs: dirs)
    }

    /// First executable named `name`.
    public func find(_ name: String) -> URL? {
        searchDirs.lazy
            .map { $0.appending(path: name) }
            .first { FileManager.default.isExecutableFile(atPath: $0.path(percentEncoded: false)) }
    }

    /// A build of `name` in a later search directory that is newer than the one `find` returns
    /// (T087): a bundled copy shadowing a fresher `<repo>/target/release` build. `nil` when the
    /// found binary is the newest (or there is only one). `slack` ignores copy-time noise.
    public func newerBuild(of name: String, slack: TimeInterval = 2) -> (found: URL, newer: URL)? {
        guard let found = find(name), let foundTime = Self.modified(found) else { return nil }
        let newest = searchDirs.map { $0.appending(path: name) }
            .filter { $0 != found && FileManager.default.isExecutableFile(atPath: $0.path(percentEncoded: false)) }
            .compactMap { url in Self.modified(url).map { (url, $0) } }
            .filter { $0.1.timeIntervalSince(foundTime) > slack }
            .max { $0.1 < $1.1 }
        return newest.map { (found, $0.0) }
    }

    private static func modified(_ url: URL) -> Date? {
        try? FileManager.default.attributesOfItem(atPath: url.path(percentEncoded: false))[.modificationDate] as? Date
    }
}
