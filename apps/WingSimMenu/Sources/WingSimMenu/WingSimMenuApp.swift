import AppKit
import SwiftUI
import WingSimCore

/// Menu-bar control app for the wing-sim daemon (T064).
///
/// `--smoke-test`: build the model (reads the status file, touches nothing else), print a line
/// and quit after a second; used by `smoke.sh` / `make app`.
@main
struct WingSimMenuApp: App {
    @State private var model = AppModel()

    init() {
        if CommandLine.arguments.contains("--smoke-test") {
            Task { @MainActor in
                try? await Task.sleep(for: .seconds(1))
                print("WingSimMenu smoke: ok (state dir \(AppModel.smokeStateDir()))")
                NSApp.terminate(nil)
            }
        }
    }

    var body: some Scene {
        MenuBarExtra {
            MenuContentView(model: model)
        } label: {
            Image(systemName: model.isRunning ? "slider.vertical.3" : "slider.horizontal.below.rectangle")
                .accessibilityLabel(model.isRunning ? "Wing Sim running" : "Wing Sim stopped")
        }
        .menuBarExtraStyle(.window)

        Window("Host Output Routing", id: "routing") {
            RoutingView(model: model)
        }
        .windowResizability(.contentSize)
    }
}

extension AppModel {
    static func smokeStateDir() -> String {
        DaemonPaths.standard().stateDir.path(percentEncoded: false)
    }
}
