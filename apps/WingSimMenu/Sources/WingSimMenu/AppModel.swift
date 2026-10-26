import AppKit
import Foundation
import Observation
import UniformTypeIdentifiers
import WingSimCore

/// App state: daemon status (polled from `status.json`), daemon process, scene and routing
/// actions. Main-actor isolated (module default); blocking socket/process work runs in
/// `@concurrent` helpers of WingSimCore.
@Observable
final class AppModel {
    let paths: DaemonPaths
    let locator: BinaryLocator
    let client: ControlClient

    /// Latest status document while the daemon is live.
    private(set) var status: DaemonStatus?
    /// The daemon process this app started (if any, while it runs).
    private(set) var child: Process?
    private(set) var busy = false
    /// Last action result / error for the menu.
    private(set) var message: String?
    private(set) var isError = false
    /// Scene file to start the daemon from (chosen with Open while stopped).
    private(set) var pendingScene: URL?
    /// A fresher wing-sim build than the one the app would run (dev builds only, T087).
    private(set) var newerBuild: (found: URL, newer: URL)?
    /// The user chose to run `newerBuild` instead of the bundled binary.
    private(set) var useNewerBuild = false

    /// Routing being edited (the hostio TOML) and the devices to pick from.
    var routing: HostIOConfig
    private(set) var savedRouting: HostIOConfig
    private(set) var devices: [OutputDevice] = []

    private var poller: Task<Void, Never>?

    init(paths: DaemonPaths = .standard(), locator: BinaryLocator = .standard()) {
        self.paths = paths
        self.locator = locator
        client = ControlClient(socketURL: paths.socketURL)
        let loaded = Self.loadRouting(paths)
        routing = loaded.config
        savedRouting = loaded.config
        if let e = loaded.error { report(e, error: true) }
        refresh()
        startPolling()
    }

    // MARK: status

    var isRunning: Bool { status != nil }
    var startedHere: Bool { child?.isRunning == true }
    var sceneFile: URL? {
        status?.sceneFile.map { URL(filePath: $0) } ?? pendingScene
    }
    var routingDirty: Bool { routing != savedRouting }

    func startPolling() {
        guard poller == nil else { return }
        poller = Task { [weak self] in
            while !Task.isCancelled {
                self?.refresh()
                try? await Task.sleep(for: .seconds(1))
            }
        }
    }

    /// Re-read `status.json` (small file; cheap enough for the main actor once a second).
    func refresh() {
        let s = StatusReader(stateDir: paths.stateDir).read()
        status = (s?.isLive() == true) ? s : nil
        newerBuild = locator.newerBuild(of: "wing-sim")
        if let c = child, !c.isRunning {
            child = nil
            if status == nil, c.terminationStatus != 0 {
                let tail = DaemonProcess.logTail(paths: paths, maxLines: 3)
                report("wing-sim exited (\(c.terminationStatus)): \(tail)", error: true)
            }
        }
    }

    // MARK: daemon

    func start() {
        guard !isRunning else { return }
        let preferred = useNewerBuild ? newerBuild?.newer : nil
        guard let bin = preferred ?? locator.find("wing-sim") else {
            report("wing-sim not found (looked in \(locator.searchDirs.map { $0.path(percentEncoded: false) }.joined(separator: ", "))). Run `make build` or apps/WingSimMenu/build.sh.", error: true)
            return
        }
        do {
            child = try DaemonProcess.launch(binary: bin, paths: paths, scene: pendingScene)
            report("Starting wing-sim (pid \(child?.processIdentifier ?? 0))…")
            pendingScene = nil
        } catch {
            report("Starting wing-sim failed: \(error.localizedDescription)", error: true)
        }
    }

    /// Run the fresher `target/release` wing-sim from the next start on.
    func adoptNewerBuild() {
        useNewerBuild = true
        report("Using the newer wing-sim build\(isRunning ? "; stop and start wing-sim to switch" : "").")
    }

    func stop() {
        perform("Stopping wing-sim") { client in
            try await client.request("quit")
        } then: { [weak self] _ in
            guard let self else { return }
            self.report("wing-sim stopped.")
            // a child that ignores `quit` gets SIGTERM after 5 s
            if let c = self.child {
                Task {
                    try? await Task.sleep(for: .seconds(5))
                    if c.isRunning { c.terminate() }
                }
            }
        } onError: { [weak self] in
            self?.child?.terminate()
        }
    }

    // MARK: scenes

    func openScene() {
        guard let url = Self.choose(open: true, title: "Open Scene") else { return }
        if isRunning {
            perform("Loading \(url.lastPathComponent)") { client in
                try await client.request("load \(url.path(percentEncoded: false))")
            }
        } else {
            pendingScene = url
            report("\(url.lastPathComponent) will be loaded when wing-sim starts.")
        }
    }

    func saveScene() {
        guard let url = status?.sceneFile.map({ URL(filePath: $0) }) else { return saveSceneAs() }
        save(to: url)
    }

    func saveSceneAs() {
        guard let url = Self.choose(open: false, title: "Save Scene As",
                                    name: sceneFile?.lastPathComponent ?? "Scene.snap") else { return }
        save(to: url)
    }

    private func save(to url: URL) {
        perform("Saving \(url.lastPathComponent)") { client in
            try await client.request("save \(url.path(percentEncoded: false))")
        }
    }

    /// Export the current state to a temp file, `wing-diff <scene file> <current>`, copy the
    /// text to the pasteboard.
    func copyDiff() {
        guard let scene = status?.sceneFile.map({ URL(filePath: $0) }) else {
            report("No scene file: open or save one first.", error: true)
            return
        }
        guard let diffBin = locator.find("wing-diff") else {
            report("wing-diff not found. Run `make build` or apps/WingSimMenu/build.sh.", error: true)
            return
        }
        let tmp = FileManager.default.temporaryDirectory
            .appending(path: "wingsim-current-\(UUID().uuidString).snap")
        perform("Diffing against \(scene.lastPathComponent)") { client in
            _ = try await client.request("export \(tmp.path(percentEncoded: false))")
            defer { try? FileManager.default.removeItem(at: tmp) }
            return try await DaemonProcess.diff(wingDiff: diffBin, scene, tmp)
        } then: { [weak self] diff in
            NSPasteboard.general.clearContents()
            NSPasteboard.general.setString(diff, forType: .string)
            let lines = diff.split(separator: "\n").count
            self?.report("Copied diff vs \(scene.lastPathComponent) (\(lines) lines).")
        }
    }

    // MARK: routing

    func reloadDevices() {
        devices = AudioDevices.outputDevices()
    }

    func revertRouting() {
        routing = savedRouting
    }

    /// Validate, write `<state-dir>/hostio.toml`, and reload the daemon's bridges if it runs.
    func applyRouting() {
        do {
            try routing.validate()
            try FileManager.default.createDirectory(at: paths.stateDir, withIntermediateDirectories: true)
            try routing.toml().write(to: paths.hostioURL, atomically: true, encoding: .utf8)
            savedRouting = routing
        } catch {
            report("Routing not saved: \(error)", error: true)
            return
        }
        guard isRunning else {
            report("Routing saved; applies when wing-sim starts.")
            return
        }
        let path = paths.hostioURL.path(percentEncoded: false)
        perform("Reloading host outputs") { client in
            try await client.request("reload-hostio \(path)")
        }
    }

    // MARK: misc

    func openLog() {
        NSWorkspace.shared.open(paths.logURL)
    }

    func revealStateDir() {
        NSWorkspace.shared.activateFileViewerSelecting([paths.stateDir])
    }

    private func report(_ text: String, error: Bool = false) {
        message = text
        isError = error
    }

    /// Run a control request off the main actor, then report its reply.
    private func perform<T: Sendable>(_ what: String,
                                      _ body: @escaping @Sendable (ControlClient) async throws -> T,
                                      then: ((T) -> Void)? = nil,
                                      onError: (() -> Void)? = nil) {
        busy = true
        report("\(what)…")
        let client = client
        Task {
            defer { busy = false }
            do {
                let r = try await body(client)
                if let then {
                    then(r)
                } else {
                    let s = (r as? String) ?? ""
                    report(s.isEmpty ? "\(what): done" : s)
                }
            } catch {
                report("\(what) failed: \(error)", error: true)
                onError?()
            }
            refresh()
        }
    }

    private static func choose(open: Bool, title: String, name: String = "") -> URL? {
        NSApp.activate(ignoringOtherApps: true)
        let snap = UTType(filenameExtension: "snap") ?? .data
        if open {
            let p = NSOpenPanel()
            p.title = title
            p.allowedContentTypes = [snap]
            p.allowsOtherFileTypes = true
            return p.runModal() == .OK ? p.url : nil
        }
        let p = NSSavePanel()
        p.title = title
        p.allowedContentTypes = [snap]
        p.nameFieldStringValue = name
        return p.runModal() == .OK ? p.url : nil
    }

    private static func loadRouting(_ paths: DaemonPaths) -> (config: HostIOConfig, error: String?) {
        guard let text = try? String(contentsOf: paths.hostioURL, encoding: .utf8) else {
            return (.defaultConfig, nil)
        }
        do {
            return (try HostIOConfig.parse(toml: text), nil)
        } catch {
            return (.defaultConfig, "\(paths.hostioURL.lastPathComponent): \(error)")
        }
    }
}
