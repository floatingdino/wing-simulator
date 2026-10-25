// swift-tools-version: 6.2
// WingSimMenu: menu-bar control app for the wing-sim daemon (T064). Build the .app with build.sh.
import PackageDescription

let package = Package(
    name: "WingSimMenu",
    platforms: [.macOS(.v14)],
    products: [
        .executable(name: "WingSimMenu", targets: ["WingSimMenu"]),
    ],
    targets: [
        // UI-free logic (status, control socket, hostio TOML, devices, daemon process): testable.
        .target(
            name: "WingSimCore",
            swiftSettings: [.enableUpcomingFeature("NonisolatedNonsendingByDefault")]
        ),
        .executableTarget(
            name: "WingSimMenu",
            dependencies: ["WingSimCore"],
            swiftSettings: [
                .defaultIsolation(MainActor.self),
                .enableUpcomingFeature("NonisolatedNonsendingByDefault"),
            ]
        ),
        .testTarget(name: "WingSimCoreTests", dependencies: ["WingSimCore"]),
    ]
)
