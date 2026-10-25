import Foundation

/// A helper process failed (its stderr).
public struct ProcessFailure: Error, Equatable, Sendable, CustomStringConvertible {
    public let message: String
    public init(_ message: String) { self.message = message }
    public var description: String { message }
}

/// Starting wing-sim and running wing-diff.
public enum DaemonProcess {
    /// Command line for a daemon started by the app: its state dir, the app-managed hostio
    /// TOML when it exists, and an optional scene file to start from.
    public static func arguments(paths: DaemonPaths, hostioExists: Bool, scene: URL?) -> [String] {
        var args = ["--state-dir", paths.stateDir.path(percentEncoded: false)]
        if hostioExists {
            args += ["--hostio", paths.hostioURL.path(percentEncoded: false)]
        }
        if let scene {
            args += ["--scene", scene.path(percentEncoded: false)]
        }
        return args
    }

    /// Launch `binary` with `arguments`, stdout/stderr appended to the log file.
    public static func launch(binary: URL, paths: DaemonPaths, scene: URL?) throws -> Process {
        let fm = FileManager.default
        try fm.createDirectory(at: paths.stateDir, withIntermediateDirectories: true)
        let logPath = paths.logURL.path(percentEncoded: false)
        if !fm.fileExists(atPath: logPath) {
            fm.createFile(atPath: logPath, contents: nil)
        }
        let log = try FileHandle(forWritingTo: paths.logURL)
        try log.seekToEnd()
        let p = Process()
        p.executableURL = binary
        p.arguments = arguments(paths: paths,
                                hostioExists: fm.fileExists(atPath: paths.hostioURL.path(percentEncoded: false)),
                                scene: scene)
        p.standardOutput = log
        p.standardError = log
        p.standardInput = FileHandle.nullDevice
        try p.run()
        return p
    }

    /// Last `maxLines` lines of the daemon log (for error reports).
    public static func logTail(paths: DaemonPaths, maxLines: Int = 5) -> String {
        guard let data = try? Data(contentsOf: paths.logURL) else { return "" }
        let lines = String(decoding: data.suffix(16_384), as: UTF8.self)
            .split(separator: "\n").suffix(maxLines)
        return lines.joined(separator: "\n")
    }

    /// `wing-diff <a> <b>` (text format); returns stdout, throws stderr on failure.
    @concurrent
    public static func diff(wingDiff: URL, _ a: URL, _ b: URL) async throws -> String {
        let p = Process()
        p.executableURL = wingDiff
        p.arguments = [a.path(percentEncoded: false), b.path(percentEncoded: false)]
        let out = Pipe(), err = Pipe()
        p.standardOutput = out
        p.standardError = err
        p.standardInput = FileHandle.nullDevice
        try p.run()
        // read both pipes before waiting so a large diff cannot block the child
        async let errData = Task.detached { err.fileHandleForReading.readDataToEndOfFile() }.value
        let outData = out.fileHandleForReading.readDataToEndOfFile()
        let stderr = await errData
        p.waitUntilExit()
        guard p.terminationStatus == 0 else {
            throw ProcessFailure(String(decoding: stderr, as: UTF8.self)
                .trimmingCharacters(in: .whitespacesAndNewlines))
        }
        return String(decoding: outData, as: UTF8.self)
    }
}
