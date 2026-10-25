import SwiftUI
import WingSimCore

/// The menu-bar window: status, daemon and scene actions.
struct MenuContentView: View {
    @Bindable var model: AppModel
    @Environment(\.openWindow) private var openWindow

    var body: some View {
        VStack(alignment: .leading, spacing: 10) {
            header
            if let s = model.status {
                StatusGrid(status: s)
            }
            Divider()
            daemonButtons
            Divider()
            sceneButtons
            Divider()
            HStack {
                Button("Host Output Routing…") {
                    model.reloadDevices()
                    openWindow(id: "routing")
                    NSApp.activate(ignoringOtherApps: true)
                }
                Spacer()
                Button("Log") { model.openLog() }
                Button("State Folder") { model.revealStateDir() }
            }
            if let m = model.message {
                Text(m)
                    .font(.caption)
                    .foregroundStyle(model.isError ? .red : .secondary)
                    .textSelection(.enabled)
                    .fixedSize(horizontal: false, vertical: true)
            }
            Divider()
            HStack {
                Spacer()
                Button("Quit WingSimMenu") { NSApp.terminate(nil) }
                    .keyboardShortcut("q")
            }
        }
        .padding(14)
        .frame(width: 380)
        .onAppear { model.startPolling() }
    }

    private var header: some View {
        HStack {
            Circle()
                .fill(model.isRunning ? Color.green : Color.secondary)
                .frame(width: 9, height: 9)
            if let s = model.status {
                Text("wing-sim \(s.version) running").font(.headline)
                Spacer()
                Text("pid \(String(s.pid))\(model.startedHere ? "" : " (external)")")
                    .font(.caption).foregroundStyle(.secondary)
            } else {
                Text(model.startedHere ? "wing-sim starting…" : "wing-sim stopped").font(.headline)
                Spacer()
            }
        }
    }

    private var daemonButtons: some View {
        HStack {
            if model.isRunning {
                Button("Stop wing-sim") { model.stop() }
            } else {
                Button("Start wing-sim") { model.start() }
                    .disabled(model.startedHere)
            }
            if model.busy { ProgressView().controlSize(.small) }
        }
    }

    private var sceneButtons: some View {
        VStack(alignment: .leading, spacing: 6) {
            Text("Scene: \(model.sceneFile?.lastPathComponent ?? "none")\(model.isRunning || model.sceneFile == nil ? "" : " (on start)")")
                .font(.subheadline)
                .help(model.sceneFile?.path(percentEncoded: false) ?? "")
            HStack {
                Button("Open…") { model.openScene() }
                Button("Save") { model.saveScene() }.disabled(!model.isRunning)
                Button("Save As…") { model.saveSceneAs() }.disabled(!model.isRunning)
            }
            Button("Copy Diff vs Scene File") { model.copyDiff() }
                .disabled(!model.isRunning || model.status?.sceneFile == nil)
        }
        .disabled(model.busy)
    }
}

/// Live engine numbers.
struct StatusGrid: View {
    let status: DaemonStatus

    var body: some View {
        Grid(alignment: .leading, horizontalSpacing: 12, verticalSpacing: 4) {
            row("WING-Edit clients", "\(status.nativeClients)")
            if let a = status.audio {
                if a.attached {
                    row("Audio", "\(a.rate) Hz, driver \(a.driver)")
                    row("Load", String(format: "%.1f%% (peak %.1f%%)", a.loadPct, a.peakLoadPct))
                    row("Cycle", "p50 \(a.cycleP50Us) µs, p99 \(a.cycleP99Us) µs")
                    row("Xruns", "\(a.xruns)  (slow cycles \(a.slowCycles))")
                        .foregroundStyle(a.xruns > 0 ? .orange : .primary)
                    if a.bridges.isEmpty {
                        row("Host outputs", "none")
                    }
                    ForEach(a.bridges) { b in
                        row(b.device, bridgeText(b))
                    }
                } else {
                    row("Audio", a.message.replacingOccurrences(of: "audio: ", with: ""))
                }
            } else {
                row("Audio", "off")
            }
            if let scene = status.activeScene {
                row("Library scene", scene)
            }
        }
        .font(.callout)
    }

    private func bridgeText(_ b: BridgeReport) -> String {
        let e2e = b.e2eMs.map { String(format: "e2e %.1f ms", $0) } ?? "e2e –"
        return String(format: "%@, fill %.1f/%.1f ms, ", b.state, b.fillMs, b.targetMs) + e2e
            + (b.underruns > 0 ? ", \(b.underruns) underruns" : "")
    }

    private func row(_ k: String, _ v: String) -> some View {
        GridRow {
            Text(k).foregroundStyle(.secondary).lineLimit(1)
            Text(v).lineLimit(2).textSelection(.enabled)
        }
    }
}
