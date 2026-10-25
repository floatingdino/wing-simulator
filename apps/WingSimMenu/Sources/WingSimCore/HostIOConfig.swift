import Foundation

/// A console signal tap (mirror of `wing_hostio::Tap`, see `crates/wing-hostio/src/config.rs`).
public enum TapKind: String, CaseIterable, Sendable, Identifiable {
    case monitor = "mon"
    case main
    case bus
    case matrix = "mtx"
    case usbOut = "usb_out"
    case localOut = "local_out"

    public var id: String { rawValue }

    /// Stereo taps have an index and L/R lanes; flat taps are numbered channels.
    public var isStereo: Bool { maxIndex != nil }

    /// Highest index of a stereo tap (`nil` for flat taps).
    public var maxIndex: Int? {
        switch self {
        case .monitor: 2
        case .main: 4
        case .bus: 16
        case .matrix: 8
        case .usbOut, .localOut: nil
        }
    }

    /// Channels of a flat tap (2 lanes for stereo taps).
    public var width: Int {
        switch self {
        case .usbOut: 48
        case .localOut: 8
        default: 2
        }
    }

    public var label: String {
        switch self {
        case .monitor: "Monitor"
        case .main: "Main"
        case .bus: "Bus"
        case .matrix: "Matrix"
        case .usbOut: "USB Out"
        case .localOut: "Local Out"
        }
    }

    /// Accepts the aliases the Rust parser accepts.
    static func named(_ s: String) -> TapKind? {
        switch s.lowercased() {
        case "mon", "monitor": .monitor
        case "main", "mains": .main
        case "bus": .bus
        case "mtx", "matrix": .matrix
        case "usb_out", "usb", "usbout": .usbOut
        case "local_out", "local", "out": .localOut
        default: nil
        }
    }
}

/// Which lanes of a stereo tap.
public enum StereoSide: String, CaseIterable, Sendable, Identifiable {
    case both = "L+R"
    case left = "L"
    case right = "R"
    public var id: String { rawValue }
}

/// Config error (message matches the Rust side's wording where it matters).
public struct HostIOError: Error, Equatable, Sendable, CustomStringConvertible {
    public let message: String
    public init(_ message: String) { self.message = message }
    public var description: String { message }
}

/// A tap reference: `mon/1`, `bus/3.L`, `usb_out/45-46`.
public struct TapRef: Equatable, Sendable {
    public var kind: TapKind
    /// Stereo taps: 1-based index.
    public var index: Int
    /// Stereo taps: lanes.
    public var side: StereoSide
    /// Flat taps: 1-based channels.
    public var channels: [Int]

    public init(kind: TapKind, index: Int = 1, side: StereoSide = .both, channels: [Int] = [1, 2]) {
        self.kind = kind
        self.index = index
        self.side = side
        self.channels = channels
    }

    /// Number of bridged lanes.
    public var lanes: Int {
        if kind.isStereo { return side == .both ? 2 : 1 }
        return channels.count
    }

    /// Canonical text (`wing_hostio::parse_tap` input).
    public var text: String {
        if kind.isStereo {
            return "\(kind.rawValue)/\(index)" + (side == .both ? "" : ".\(side.rawValue)")
        }
        return "\(kind.rawValue)/\(ChannelList.format(channels))"
    }

    public static func parse(_ raw: String) throws(HostIOError) -> TapRef {
        let s = raw.trimmingCharacters(in: .whitespaces)
        // legacy `usb_out 45/46`
        if let sp = s.firstIndex(where: \.isWhitespace) {
            guard let k = TapKind.named(String(s[..<sp])), !k.isStereo else {
                throw HostIOError("tap '\(s)': use 'kind/index' (e.g. 'mon/1', 'bus/3.L')")
            }
            return try flat(k, ChannelList.parse(String(s[sp...]).trimmingCharacters(in: .whitespaces)))
        }
        guard let slash = s.firstIndex(of: "/") else {
            throw HostIOError("tap '\(s)': expected 'kind/index'")
        }
        let kindText = String(s[..<slash]).trimmingCharacters(in: .whitespaces)
        let rest = String(s[s.index(after: slash)...])
        guard let kind = TapKind.named(kindText) else { throw HostIOError("unknown tap '\(kindText)'") }
        if !kind.isStereo {
            return try flat(kind, ChannelList.parse(rest.replacingOccurrences(of: "/", with: ",")))
        }
        var idxText = rest
        var side = StereoSide.both
        if let dot = rest.firstIndex(of: ".") {
            idxText = String(rest[..<dot])
            switch rest[rest.index(after: dot)...].trimmingCharacters(in: .whitespaces).uppercased() {
            case "L": side = .left
            case "R": side = .right
            case let x: throw HostIOError("tap '\(s)': side must be L or R, got '\(x)'")
            }
        }
        guard let n = Int(idxText.trimmingCharacters(in: .whitespaces)) else {
            throw HostIOError("tap '\(s)': bad index '\(idxText)'")
        }
        guard let max = kind.maxIndex, (1...max).contains(n) else {
            throw HostIOError("\(kind.rawValue)/\(n): index must be 1..\(kind.maxIndex ?? 0)")
        }
        return TapRef(kind: kind, index: n, side: side)
    }

    private static func flat(_ kind: TapKind, _ chans: [Int]) throws(HostIOError) -> TapRef {
        if let bad = chans.first(where: { $0 > kind.width }) {
            throw HostIOError("\(kind.rawValue) has \(kind.width) channels, got \(bad)")
        }
        return TapRef(kind: kind, channels: chans)
    }
}

/// 1-based channel lists: `1/2`, `45-46`, `1,3,5`.
public enum ChannelList {
    public static func parse(_ s: String) throws(HostIOError) -> [Int] {
        var out: [Int] = []
        for part in s.split(omittingEmptySubsequences: false, whereSeparator: { $0 == "/" || $0 == "," }) {
            let p = part.trimmingCharacters(in: .whitespaces)
            if p.isEmpty { throw HostIOError("empty channel in '\(s)'") }
            func num(_ t: Substring) throws(HostIOError) -> Int {
                guard let n = Int(t.trimmingCharacters(in: .whitespaces)), n > 0, n <= Int(UInt16.max) else {
                    throw HostIOError("bad channel '\(t)' in '\(s)' (1-based numbers)")
                }
                return n
            }
            if let dash = p.firstIndex(of: "-") {
                let a = try num(p[..<dash]), b = try num(p[p.index(after: dash)...])
                if a > b { throw HostIOError("descending range '\(p)'") }
                out.append(contentsOf: a...b)
            } else {
                out.append(try num(p[...]))
            }
        }
        return out
    }

    /// `[1,2]` -> `1/2`; runs of 3+ -> `a-b`; others `,`-joined.
    public static func format(_ chans: [Int]) -> String {
        if chans.count == 2 { return "\(chans[0])/\(chans[1])" }
        var parts: [String] = []
        var i = 0
        while i < chans.count {
            var j = i
            while j + 1 < chans.count, chans[j + 1] == chans[j] + 1 { j += 1 }
            parts.append(j - i >= 2 ? "\(chans[i])-\(chans[j])" : (i...j).map { String(chans[$0]) }.joined(separator: ","))
            i = j + 1
        }
        return parts.joined(separator: ",")
    }
}

/// One route: tap lanes -> device output channels.
public struct HostRoute: Equatable, Sendable, Identifiable {
    public var id = UUID()
    public var source: TapRef
    /// Device name or UID.
    public var device: String
    /// 1-based device output channels (one per tap lane).
    public var deviceChannels: [Int]
    /// Per-route extra latency override.
    public var latencyMs: Double?

    public init(source: TapRef, device: String, deviceChannels: [Int], latencyMs: Double? = nil) {
        self.source = source
        self.device = device
        self.deviceChannels = deviceChannels
        self.latencyMs = latencyMs
    }

    public static func == (a: HostRoute, b: HostRoute) -> Bool {
        a.source == b.source && a.device == b.device && a.deviceChannels == b.deviceChannels
            && a.latencyMs == b.latencyMs
    }

    /// `mon/1 -> "Scarlett 2i4 USB" 1/2`
    public var mapLine: String {
        "\(source.text) -> \"\(device)\" \(ChannelList.format(deviceChannels))"
    }

    /// Parse a `map` line.
    public static func parse(_ line: String) throws(HostIOError) -> HostRoute {
        guard let arrow = line.range(of: "->") else { throw HostIOError("route '\(line)': missing '->'") }
        let src = String(line[..<arrow.lowerBound])
        let dst = line[arrow.upperBound...].trimmingCharacters(in: .whitespaces)
        let device: String, chans: String
        if dst.hasPrefix("\"") {
            let rest = dst.dropFirst()
            guard let end = rest.firstIndex(of: "\"") else {
                throw HostIOError("route '\(line)': unterminated quote")
            }
            device = String(rest[..<end])
            chans = rest[rest.index(after: end)...].trimmingCharacters(in: .whitespaces)
        } else if let sp = dst.lastIndex(where: \.isWhitespace) {
            device = dst[..<sp].trimmingCharacters(in: .whitespaces)
            chans = dst[sp...].trimmingCharacters(in: .whitespaces)
        } else {
            throw HostIOError("route '\(line)': expected '<device> <channels>'")
        }
        return try build(TapRef.parse(src), device, ChannelList.parse(chans))
    }

    static func build(_ tap: TapRef, _ device: String, _ chans: [Int],
                      latencyMs: Double? = nil) throws(HostIOError) -> HostRoute {
        if device.isEmpty { throw HostIOError("empty device name") }
        if tap.lanes != chans.count {
            throw HostIOError("route to '\(device)': tap has \(tap.lanes) lanes but \(chans.count) device channels given")
        }
        return HostRoute(source: tap, device: device, deviceChannels: chans, latencyMs: latencyMs)
    }
}

/// The host-output config (`--hostio` TOML) the app edits.
public struct HostIOConfig: Equatable, Sendable {
    public var latencyMs: Double
    public var bufferFrames: Int?
    public var routes: [HostRoute]

    public init(latencyMs: Double = 5, bufferFrames: Int? = nil, routes: [HostRoute] = []) {
        self.latencyMs = latencyMs
        self.bufferFrames = bufferFrames
        self.routes = routes
    }

    /// The daemon's built-in default: monitor A -> Scarlett 2i4 USB 1/2.
    public static let defaultConfig = HostIOConfig(routes: [
        HostRoute(source: TapRef(kind: .monitor), device: "Scarlett 2i4 USB", deviceChannels: [1, 2]),
    ])

    /// Same checks as `HostIoConfig::devices`: lane counts, and no device channel used twice.
    public func validate() throws(HostIOError) {
        if !(0...1000).contains(latencyMs) { throw HostIOError("latency_ms \(latencyMs) out of range 0..1000") }
        var used: [String: Set<Int>] = [:]
        for r in routes {
            if r.device.isEmpty { throw HostIOError("empty device name") }
            if r.source.lanes == 0 { throw HostIOError("\(r.source.text): no channels") }
            if r.source.lanes != r.deviceChannels.count {
                throw HostIOError("route to '\(r.device)': tap has \(r.source.lanes) lanes but \(r.deviceChannels.count) device channels given")
            }
            if let l = r.latencyMs, !(0...1000).contains(l) {
                throw HostIOError("latency_ms \(l) out of range 0..1000")
            }
            for c in r.deviceChannels {
                if c < 1 { throw HostIOError("device channels are 1-based") }
                if !used[r.device, default: []].insert(c).inserted {
                    throw HostIOError("device '\(r.device)' channel \(c) used twice")
                }
            }
        }
    }

    /// TOML for `wing-sim --hostio`.
    public func toml() -> String {
        var s = "# Host output routing for wing-sim (--hostio), written by WingSimMenu.\n"
        s += "# Syntax: crates/wing-hostio/hostio.example.toml\n[hostio]\n"
        s += "latency_ms = \(Self.number(latencyMs))\n"
        if let b = bufferFrames { s += "buffer_frames = \(b)\n" }
        for r in routes {
            s += "\n[[hostio.route]]\nmap = \(Self.quote(r.mapLine))\n"
            if let l = r.latencyMs { s += "latency_ms = \(Self.number(l))\n" }
        }
        return s
    }

    /// Parse the TOML subset wing-sim accepts for host outputs: a `[hostio]` table (or its keys
    /// at top level) with `latency_ms`, `buffer_frames` and `[[hostio.route]]` entries in `map`
    /// or `source`/`device`/`channels` form.
    public static func parse(toml text: String) throws(HostIOError) -> HostIOConfig {
        var cfg = HostIOConfig()
        var section = ""  // "", "hostio", "route"
        var route: [String: TOMLValue]? = nil
        func flush() throws(HostIOError) {
            guard let r = route else { return }
            route = nil
            for k in r.keys where !["map", "source", "device", "channels", "latency_ms"].contains(k) {
                throw HostIOError("unknown route key '\(k)'")
            }
            var latency: Double? = nil
            if let l = r["latency_ms"] { latency = try l.number("latency_ms") }
            let built: HostRoute
            switch (r["map"], r["source"], r["device"], r["channels"]) {
            case (let m?, nil, nil, nil):
                built = try HostRoute.parse(m.string("map"))
            case (nil, let s?, let d?, let c?):
                built = try HostRoute.build(TapRef.parse(s.string("source")), d.string("device"),
                                            ChannelList.parse(c.string("channels")))
            default:
                throw HostIOError("route needs either 'map' or all of 'source', 'device', 'channels'")
            }
            var r2 = built
            r2.latencyMs = latency
            cfg.routes.append(r2)
        }
        for (n, rawLine) in text.split(separator: "\n", omittingEmptySubsequences: false).enumerated() {
            let line = TOMLValue.stripComment(String(rawLine)).trimmingCharacters(in: .whitespaces)
            if line.isEmpty { continue }
            if line.hasPrefix("[[") {
                try flush()
                let name = line.trimmingCharacters(in: CharacterSet(charactersIn: "[] "))
                guard name == "hostio.route" || name == "route" else {
                    throw HostIOError("line \(n + 1): unexpected table [[\(name)]]")
                }
                section = "route"
                route = [:]
                continue
            }
            if line.hasPrefix("[") {
                try flush()
                let name = line.trimmingCharacters(in: CharacterSet(charactersIn: "[] "))
                guard name == "hostio" else { throw HostIOError("line \(n + 1): unexpected table [\(name)]") }
                section = "hostio"
                continue
            }
            guard let eq = line.firstIndex(of: "=") else { throw HostIOError("line \(n + 1): expected key = value") }
            let key = line[..<eq].trimmingCharacters(in: .whitespaces)
            let value = try TOMLValue.parse(line[line.index(after: eq)...].trimmingCharacters(in: .whitespaces), line: n + 1)
            if section == "route" {
                route?[key] = value
            } else {
                switch key {
                case "latency_ms": cfg.latencyMs = try value.number(key)
                case "buffer_frames": cfg.bufferFrames = Int(try value.number(key))
                default: throw HostIOError("line \(n + 1): unknown key '\(key)'")
                }
            }
        }
        try flush()
        try cfg.validate()
        return cfg
    }

    static func number(_ v: Double) -> String {
        v == v.rounded() ? String(format: "%.1f", v) : String(v)
    }

    /// TOML literal string when possible, else a basic string.
    static func quote(_ s: String) -> String {
        if !s.contains("'") && !s.contains("\n") { return "'\(s)'" }
        return "\"" + s.replacingOccurrences(of: "\\", with: "\\\\").replacingOccurrences(of: "\"", with: "\\\"") + "\""
    }
}

/// The scalar TOML values the hostio config uses.
enum TOMLValue: Equatable {
    case string(String)
    case number(Double)

    func string(_ key: String) throws(HostIOError) -> String {
        guard case .string(let s) = self else { throw HostIOError("'\(key)' must be a string") }
        return s
    }

    func number(_ key: String) throws(HostIOError) -> Double {
        guard case .number(let v) = self else { throw HostIOError("'\(key)' must be a number") }
        return v
    }

    static func parse(_ s: String, line: Int) throws(HostIOError) -> TOMLValue {
        if s.hasPrefix("'") {
            guard s.count >= 2, s.hasSuffix("'") else { throw HostIOError("line \(line): unterminated string") }
            return .string(String(s.dropFirst().dropLast()))
        }
        if s.hasPrefix("\"") {
            guard s.count >= 2, s.hasSuffix("\"") else { throw HostIOError("line \(line): unterminated string") }
            var out = ""
            var escape = false
            for c in s.dropFirst().dropLast() {
                if escape {
                    switch c {
                    case "n": out.append("\n")
                    case "t": out.append("\t")
                    default: out.append(c)
                    }
                    escape = false
                } else if c == "\\" {
                    escape = true
                } else {
                    out.append(c)
                }
            }
            return .string(out)
        }
        guard let v = Double(s.replacingOccurrences(of: "_", with: "")) else {
            throw HostIOError("line \(line): unsupported value '\(s)'")
        }
        return .number(v)
    }

    /// Drop a `#` comment that is not inside a string.
    static func stripComment(_ line: String) -> String {
        var quote: Character? = nil
        var prev: Character = " "
        for (i, c) in zip(line.indices, line) {
            if let q = quote {
                if c == q && !(q == "\"" && prev == "\\") { quote = nil }
            } else if c == "'" || c == "\"" {
                quote = c
            } else if c == "#" {
                return String(line[..<i])
            }
            prev = c
        }
        return line
    }
}
