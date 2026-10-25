import Foundation

/// Errors from the control socket.
public enum ControlError: Error, Equatable, Sendable, CustomStringConvertible {
    /// Socket path too long for `sockaddr_un`.
    case pathTooLong(String)
    /// connect/read/write failed (errno text).
    case io(String)
    /// The daemon answered `err <message>`.
    case daemon(String)
    /// Reply was neither `ok` nor `err`.
    case badReply(String)

    public var description: String {
        switch self {
        case .pathTooLong(let p): "control socket path too long: \(p)"
        case .io(let m): "control socket: \(m)"
        case .daemon(let m): m
        case .badReply(let r): "unexpected reply from wing-sim: \(r)"
        }
    }
}

/// Line-protocol client for wing-sim's control socket (`<state-dir>/control.sock`): one
/// connection per command, reply `ok <text>` / `err <text>` (see `wing_sim::control`).
public struct ControlClient: Sendable {
    public let socketPath: String
    public var timeout: TimeInterval

    public init(socketURL: URL, timeout: TimeInterval = 20) {
        socketPath = socketURL.path(percentEncoded: false)
        self.timeout = timeout
    }

    /// Send `line`; blocking (call from a background task, see `request`).
    public func send(_ line: String) throws(ControlError) -> String {
        let fd = socket(AF_UNIX, SOCK_STREAM, 0)
        guard fd >= 0 else { throw .io(Self.errnoText("socket")) }
        defer { close(fd) }
        var one: Int32 = 1
        setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &one, socklen_t(MemoryLayout<Int32>.size))
        var tv = timeval(tv_sec: Int(timeout), tv_usec: Int32((timeout - timeout.rounded(.down)) * 1e6))
        setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, socklen_t(MemoryLayout<timeval>.size))
        setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, socklen_t(MemoryLayout<timeval>.size))

        var addr = sockaddr_un()
        addr.sun_family = sa_family_t(AF_UNIX)
        let pathBytes = Array(socketPath.utf8)
        let capacity = MemoryLayout.size(ofValue: addr.sun_path)
        guard pathBytes.count < capacity else { throw .pathTooLong(socketPath) }
        withUnsafeMutableBytes(of: &addr.sun_path) { raw in
            raw.copyBytes(from: pathBytes)
            raw[pathBytes.count] = 0
        }
        addr.sun_len = UInt8(MemoryLayout<sockaddr_un>.size)
        let rc = withUnsafePointer(to: &addr) {
            $0.withMemoryRebound(to: sockaddr.self, capacity: 1) {
                connect(fd, $0, socklen_t(MemoryLayout<sockaddr_un>.size))
            }
        }
        guard rc == 0 else { throw .io(Self.errnoText("connect \(socketPath)")) }

        let request = Array((line.trimmingCharacters(in: .whitespacesAndNewlines) + "\n").utf8)
        var sent = 0
        while sent < request.count {
            let n = request[sent...].withUnsafeBytes { write(fd, $0.baseAddress, $0.count) }
            guard n > 0 else { throw .io(Self.errnoText("write")) }
            sent += n
        }
        var reply: [UInt8] = []
        var buf = [UInt8](repeating: 0, count: 4096)
        while !reply.contains(UInt8(ascii: "\n")) {
            let n = buf.withUnsafeMutableBytes { read(fd, $0.baseAddress, $0.count) }
            if n < 0 { throw .io(Self.errnoText("read")) }
            if n == 0 { break }
            reply.append(contentsOf: buf[..<n])
        }
        return try Self.parseReply(String(decoding: reply, as: UTF8.self))
    }

    /// `send` on the concurrent pool.
    @concurrent
    public func request(_ line: String) async throws(ControlError) -> String {
        try send(line)
    }

    /// `ok text` -> `text`; `err text` -> `.daemon(text)`.
    public static func parseReply(_ raw: String) throws(ControlError) -> String {
        let line = raw.split(separator: "\n", maxSplits: 1, omittingEmptySubsequences: false)
            .first.map(String.init) ?? ""
        if line == "ok" { return "" }
        if line.hasPrefix("ok ") { return String(line.dropFirst(3)) }
        if line.hasPrefix("err") {
            throw .daemon(String(line.dropFirst(3)).trimmingCharacters(in: .whitespaces))
        }
        throw .badReply(line.isEmpty ? "(no reply)" : line)
    }

    private static func errnoText(_ what: String) -> String {
        "\(what): \(String(cString: strerror(errno)))"
    }
}
