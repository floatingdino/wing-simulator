import Foundation
import Testing
@testable import WingSimCore

/// Repo root (this file is apps/WingSimMenu/Tests/WingSimCoreTests/...).
let repoRoot = URL(filePath: #filePath)
    .deletingLastPathComponent().deletingLastPathComponent()
    .deletingLastPathComponent().deletingLastPathComponent().deletingLastPathComponent()

func fixture(_ path: String) throws -> Data {
    try Data(contentsOf: repoRoot.appending(path: "fixtures/\(path)"))
}

func tempDir(_ tag: String) throws -> URL {
    // short path: sockaddr_un holds 104 bytes
    let d = URL(filePath: "/tmp/wsm-\(getpid())-\(tag)", directoryHint: .isDirectory)
    try? FileManager.default.removeItem(at: d)
    try FileManager.default.createDirectory(at: d, withIntermediateDirectories: true)
    return d
}

@Suite struct StatusTests {
    @Test func `decodes the daemon's status document`() throws {
        let s = try DaemonStatus.decode(fixture("control/status.json"))
        #expect(s.pid == 4242)
        #expect(s.nativeClients == 1)
        #expect(s.commit == "abc1234" && s.builtUnix == 1_793_000_000)
        #expect(s.sceneFile == "/Users/x/Scenes/Show.snap")
        #expect(s.activeScene == nil)
        let a = try #require(s.audio)
        #expect(a.attached && a.rate == 48_000 && a.xruns == 2)
        #expect(a.loadPct == 23.5 && a.cycleP99Us == 520)
        #expect(a.bridges.first?.e2eMs == 23.25)
        #expect(a.bridges.first?.device == "Scarlett 2i4 USB")
        #expect(s.monitorLevelsDb == [0.0, -12.5])
    }

    @Test func `liveness needs running, a fresh document and the process`() throws {
        var s = try DaemonStatus.decode(fixture("control/status.json"))
        let now = Date(timeIntervalSince1970: s.updatedUnix + 1)
        #expect(s.isLive(now: now, processExists: { _ in true }))
        #expect(!s.isLive(now: now, processExists: { _ in false }))
        #expect(!s.isLive(now: now.addingTimeInterval(10), processExists: { _ in true }))
        s.running = false
        #expect(!s.isLive(now: now, processExists: { _ in true }))
        #expect(DaemonStatus.processExists(getpid()))
        #expect(!DaemonStatus.processExists(0))
    }

    @Test func `status reader tolerates missing and partial files`() throws {
        let dir = try tempDir("status")
        defer { try? FileManager.default.removeItem(at: dir) }
        let r = StatusReader(stateDir: dir)
        #expect(r.read() == nil)
        try Data("{\"running\": tr".utf8).write(to: r.url)
        #expect(r.read() == nil)
        try fixture("control/status.json").write(to: r.url)
        #expect(r.read()?.pid == 4242)
    }
}

@Suite struct HostIOTests {
    @Test(arguments: [
        ("mon/1", 2, "mon/1"),
        ("bus/3.L", 1, "bus/3.L"),
        ("main/2.r", 1, "main/2.R"),
        ("matrix/8", 2, "mtx/8"),
        ("usb_out/45-46", 2, "usb_out/45/46"),
        ("usb_out 45/46", 2, "usb_out/45/46"),
        ("local_out/1-3,5", 4, "local_out/1-3,5"),
    ])
    func `parses tap references`(text: String, lanes: Int, canonical: String) throws {
        let t = try TapRef.parse(text)
        #expect(t.lanes == lanes)
        #expect(t.text == canonical)
        #expect(try TapRef.parse(t.text) == t)
    }

    @Test(arguments: ["mon/3", "bus/0", "bus/1.X", "usb_out/49", "foo/1", "mon", "main 1/2", "usb_out/2-1"])
    func `rejects bad taps`(text: String) {
        #expect(throws: HostIOError.self) { try TapRef.parse(text) }
    }

    @Test func `channel lists`() throws {
        #expect(try ChannelList.parse("1/2") == [1, 2])
        #expect(try ChannelList.parse("1-4, 7") == [1, 2, 3, 4, 7])
        #expect(throws: HostIOError.self) { try ChannelList.parse("0") }
        #expect(throws: HostIOError.self) { try ChannelList.parse("1//2") }
        #expect(ChannelList.format([3, 4]) == "3/4")
        #expect(ChannelList.format([1, 2, 3, 5]) == "1-3,5")
        #expect(ChannelList.format([7]) == "7")
    }

    @Test func `routes parse quoted and unquoted devices`() throws {
        let r = try HostRoute.parse(#"mon/1 -> "Scarlett 2i4 USB" 1/2"#)
        #expect(r.device == "Scarlett 2i4 USB" && r.deviceChannels == [1, 2])
        #expect(r.mapLine == #"mon/1 -> "Scarlett 2i4 USB" 1/2"#)
        let u = try HostRoute.parse("usb_out 45/46 -> BuiltInSpeakerDevice 1/2")
        #expect(u.device == "BuiltInSpeakerDevice")
        #expect(throws: HostIOError.self) { try HostRoute.parse(#"mon/1 -> "Dev" 1"#) }
        #expect(throws: HostIOError.self) { try HostRoute.parse("mon/1 Dev 1/2") }
    }

    /// The writer produces exactly the fixture the Rust parser test
    /// (`wing_hostio::config::tests::parses_wingsimmenu_output`) accepts.
    @Test func `writes the TOML wing-sim accepts`() throws {
        let cfg = HostIOConfig(latencyMs: 5, bufferFrames: 256, routes: [
            HostRoute(source: TapRef(kind: .monitor), device: "Scarlett 2i4 USB", deviceChannels: [1, 2]),
            HostRoute(source: TapRef(kind: .bus, index: 3, side: .left), device: "MacBook Pro Speakers",
                      deviceChannels: [2], latencyMs: 8.5),
            HostRoute(source: TapRef(kind: .usbOut, channels: [45, 46]), device: "Scarlett 2i4 USB",
                      deviceChannels: [3, 4]),
            HostRoute(source: TapRef(kind: .localOut, channels: [1, 2, 3, 5]), device: "BlackHole 16ch",
                      deviceChannels: [1, 2, 3, 4]),
        ])
        try cfg.validate()
        let expected = String(decoding: try fixture("hostio/wingsimmenu.toml"), as: UTF8.self)
        #expect(cfg.toml() == expected)
        #expect(try HostIOConfig.parse(toml: expected) == cfg)
    }

    @Test func `reads the repo's example and structured routes`() throws {
        let ex = try String(contentsOf: repoRoot.appending(path: "crates/wing-hostio/hostio.example.toml"),
                            encoding: .utf8)
        #expect(try HostIOConfig.parse(toml: ex) == .defaultConfig)
        let structured = """
        latency_ms = 3.0
        [[route]]   # top-level form
        source = "usb_out/45-46"
        device = "Scarlett 2i4 USB"
        channels = "3/4"
        latency_ms = 8
        """
        let c = try HostIOConfig.parse(toml: structured)
        #expect(c.latencyMs == 3)
        #expect(c.routes.first?.source == TapRef(kind: .usbOut, channels: [45, 46]))
        #expect(c.routes.first?.latencyMs == 8)
    }

    @Test(arguments: [
        "[hostio]\nbogus = 1",
        "[[hostio.route]]\nmap = 'mon/1 -> \"A\" 1/2'\nsource = 'x'",
        "[[hostio.route]]\nmap = 'mon/1 -> \"A\" 1/2'\n[[hostio.route]]\nmap = 'main/1 -> \"A\" 2/3'",
        "[hostio]\nlatency_ms = 2000",
        "[other]",
        "[[hostio.route]]\nmap = 'mon/1 -> \"A\" 1/2",
    ])
    func `rejects bad configs`(toml: String) {
        #expect(throws: HostIOError.self) { try HostIOConfig.parse(toml: toml) }
    }

    @Test func `quotes device names with apostrophes`() throws {
        let cfg = HostIOConfig(routes: [
            HostRoute(source: TapRef(kind: .main), device: "Sam's Speakers", deviceChannels: [1, 2]),
        ])
        #expect(try HostIOConfig.parse(toml: cfg.toml()) == cfg)
    }
}

@Suite struct DaemonTests {
    @Test func `daemon command line`() {
        let p = DaemonPaths(stateDir: URL(filePath: "/s/Wing Sim", directoryHint: .isDirectory))
        #expect(DaemonProcess.arguments(paths: p, hostioExists: false, scene: nil)
            == ["--state-dir", "/s/Wing Sim/"])
        #expect(DaemonProcess.arguments(paths: p, hostioExists: true, scene: URL(filePath: "/x/a b.snap"))
            == ["--state-dir", "/s/Wing Sim/", "--hostio", "/s/Wing Sim/hostio.toml", "--scene", "/x/a b.snap"])
    }

    @Test func `standard paths honour the environment`() {
        let p = DaemonPaths.standard(environment: ["WINGSIM_STATE_DIR": "/tmp/ws"])
        #expect(p.socketURL.path(percentEncoded: false) == "/tmp/ws/control.sock")
        let d = DaemonPaths.standard(environment: [:])
        #expect(d.stateDir.path(percentEncoded: false).hasSuffix("Library/Application Support/WingSimulator/"))
    }

    @Test func `binary locator takes the first executable`() throws {
        let a = try tempDir("bin-a"), b = try tempDir("bin-b")
        defer { [a, b].forEach { try? FileManager.default.removeItem(at: $0) } }
        let exe = b.appending(path: "wing-sim")
        try Data("#!/bin/sh\n".utf8).write(to: exe)
        try FileManager.default.setAttributes([.posixPermissions: 0o755], ofItemAtPath: exe.path(percentEncoded: false))
        try Data().write(to: a.appending(path: "wing-diff"))  // not executable
        let loc = BinaryLocator(searchDirs: [a, b])
        #expect(loc.find("wing-sim")?.path(percentEncoded: false) == exe.path(percentEncoded: false))
        #expect(loc.find("wing-diff") == nil)
    }

    @Test func `locator reports a newer build in a later directory`() throws {
        let dir = try tempDir("stale")
        defer { try? FileManager.default.removeItem(at: dir) }
        let a = dir.appending(path: "a", directoryHint: .isDirectory)
        let b = dir.appending(path: "b", directoryHint: .isDirectory)
        for d in [a, b] { try FileManager.default.createDirectory(at: d, withIntermediateDirectories: true) }
        let old = a.appending(path: "wing-sim"), new = b.appending(path: "wing-sim")
        for f in [old, new] {
            try Data("#!/bin/sh\n".utf8).write(to: f)
            try FileManager.default.setAttributes([.posixPermissions: 0o755], ofItemAtPath: f.path(percentEncoded: false))
        }
        let now = Date()
        try FileManager.default.setAttributes([.modificationDate: now.addingTimeInterval(-3600)],
                                              ofItemAtPath: old.path(percentEncoded: false))
        let loc = BinaryLocator(searchDirs: [a, b])
        #expect(loc.newerBuild(of: "wing-sim")?.newer == new)
        try FileManager.default.setAttributes([.modificationDate: now.addingTimeInterval(-7200)],
                                              ofItemAtPath: new.path(percentEncoded: false))
        #expect(loc.newerBuild(of: "wing-sim") == nil)
        #expect(loc.newerBuild(of: "wing-diff") == nil)
    }

    @Test func `diff runs the tool and surfaces its errors`() async throws {
        let dir = try tempDir("diff")
        defer { try? FileManager.default.removeItem(at: dir) }
        let tool = dir.appending(path: "fake-diff")
        try Data("#!/bin/sh\nif [ \"$1\" = /bad ]; then echo nope >&2; exit 2; fi\necho \"diff $1 $2\"\n".utf8)
            .write(to: tool)
        try FileManager.default.setAttributes([.posixPermissions: 0o755], ofItemAtPath: tool.path(percentEncoded: false))
        let out = try await DaemonProcess.diff(wingDiff: tool, URL(filePath: "/a.snap"), URL(filePath: "/b.snap"))
        #expect(out == "diff /a.snap /b.snap\n")
        await #expect(throws: ProcessFailure("nope")) {
            try await DaemonProcess.diff(wingDiff: tool, URL(filePath: "/bad"), URL(filePath: "/b"))
        }
    }

    @Test func `device listing does not crash`() {
        for d in AudioDevices.outputDevices() {
            #expect(d.outputChannels > 0)
            #expect(d.name != AudioDevices.simulatorDeviceName)
        }
    }
}

@Suite struct ControlClientTests {
    @Test func `reply parsing`() throws {
        #expect(try ControlClient.parseReply("ok saved 12 bytes\n") == "saved 12 bytes")
        #expect(try ControlClient.parseReply("ok\n") == "")
        #expect(throws: ControlError.daemon("no such file")) { try ControlClient.parseReply("err no such file\n") }
        #expect(throws: ControlError.badReply("(no reply)")) { try ControlClient.parseReply("") }
    }

    @Test func `talks to a line server`() async throws {
        let dir = try tempDir("sock")
        defer { try? FileManager.default.removeItem(at: dir) }
        let sock = dir.appending(path: "control.sock")
        let server = try FakeServer(path: sock.path(percentEncoded: false)) { line in
            line == "ping" ? "ok wing-sim 0.1.0 pid 1" : "err unknown command \"\(line)\""
        }
        let client = ControlClient(socketURL: sock, timeout: 5)
        #expect(try await client.request("ping") == "wing-sim 0.1.0 pid 1")
        await #expect(throws: ControlError.daemon("unknown command \"bogus\"")) {
            try await client.request("bogus")
        }
        server.stop()
        #expect(throws: ControlError.self) { try client.send("ping") }
    }

    @Test func `missing socket is an io error`() {
        let c = ControlClient(socketURL: URL(filePath: "/tmp/wsm-none-\(getpid()).sock"))
        #expect {
            try c.send("ping")
        } throws: { e in
            if case .io = e as? ControlError { return true }
            return false
        }
    }
}

/// Minimal Unix-socket line server: one reply line per request line.
final class FakeServer: @unchecked Sendable {
    private let fd: Int32
    private let path: String
    private let thread: Thread

    init(path: String, reply: @escaping @Sendable (String) -> String) throws {
        self.path = path
        unlink(path)
        let lfd = socket(AF_UNIX, SOCK_STREAM, 0)
        fd = lfd
        var addr = sockaddr_un()
        addr.sun_family = sa_family_t(AF_UNIX)
        withUnsafeMutableBytes(of: &addr.sun_path) { raw in
            let b = Array(path.utf8)
            raw.copyBytes(from: b)
            raw[b.count] = 0
        }
        let rc = withUnsafePointer(to: &addr) {
            $0.withMemoryRebound(to: sockaddr.self, capacity: 1) {
                bind(lfd, $0, socklen_t(MemoryLayout<sockaddr_un>.size))
            }
        }
        guard rc == 0, listen(lfd, 4) == 0 else { throw ControlError.io("bind \(path)") }
        thread = Thread {
            while true {
                let c = accept(lfd, nil, nil)
                if c < 0 { return }
                var buf = [UInt8](repeating: 0, count: 1024)
                var acc: [UInt8] = []
                while true {
                    let n = buf.withUnsafeMutableBytes { read(c, $0.baseAddress, $0.count) }
                    if n <= 0 { break }
                    acc += buf[..<n]
                    while let nl = acc.firstIndex(of: UInt8(ascii: "\n")) {
                        let line = String(decoding: acc[..<nl], as: UTF8.self)
                        acc.removeSubrange(...nl)
                        let out = Array((reply(line) + "\n").utf8)
                        _ = out.withUnsafeBytes { write(c, $0.baseAddress, $0.count) }
                    }
                }
                close(c)
            }
        }
        thread.start()
    }

    func stop() {
        shutdown(fd, SHUT_RDWR)
        close(fd)
        unlink(path)
    }
}
