import SwiftUI
import WingSimCore

/// Editor for `<state-dir>/hostio.toml`: console taps -> host device channels.
struct RoutingView: View {
    @Bindable var model: AppModel

    var body: some View {
        VStack(alignment: .leading, spacing: 12) {
            Text("Console taps played on host audio devices (wing-sim --hostio).")
                .font(.callout).foregroundStyle(.secondary)
            List {
                ForEach($model.routing.routes) { $route in
                    RouteEditor(route: $route, devices: model.devices)
                        .padding(.vertical, 4)
                }
                .onDelete { model.routing.routes.remove(atOffsets: $0) }
            }
            .frame(minHeight: 220)
            HStack {
                Button("Add Route") { addRoute() }
                Button("Remove Last") { _ = model.routing.routes.popLast() }
                    .disabled(model.routing.routes.isEmpty)
                Spacer()
                Text("Extra latency")
                TextField("ms", value: $model.routing.latencyMs, format: .number.precision(.fractionLength(0...1)))
                    .frame(width: 50)
                Text("ms")
            }
            if let problem = validation {
                Text(problem).font(.caption).foregroundStyle(.red)
            }
            HStack {
                Button("Refresh Devices") { model.reloadDevices() }
                Spacer()
                Button("Revert") { model.revertRouting() }.disabled(!model.routingDirty)
                Button(model.isRunning ? "Apply & Reload" : "Save") { model.applyRouting() }
                    .keyboardShortcut(.defaultAction)
                    .disabled(validation != nil)
            }
            if let m = model.message {
                Text(m).font(.caption).foregroundStyle(model.isError ? .red : .secondary)
            }
        }
        .padding(16)
        .frame(minWidth: 620)
        .onAppear { model.reloadDevices() }
    }

    private var validation: String? {
        do {
            try model.routing.validate()
            for r in model.routing.routes {
                if let d = model.devices.first(where: { $0.name == r.device || $0.uid == r.device }),
                   let top = r.deviceChannels.max(), top > d.outputChannels {
                    return "\(d.name) has \(d.outputChannels) outputs; route uses channel \(top)."
                }
            }
            return nil
        } catch {
            return "\(error)"
        }
    }

    private func addRoute() {
        let device = model.devices.first?.name ?? "Scarlett 2i4 USB"
        let used = Set(model.routing.routes.filter { $0.device == device }.flatMap(\.deviceChannels))
        let start = stride(from: 1, to: 64, by: 2).first { !used.contains($0) && !used.contains($0 + 1) } ?? 1
        model.routing.routes.append(
            HostRoute(source: TapRef(kind: .main), device: device, deviceChannels: [start, start + 1]))
    }
}

/// One route row: tap picker, lanes, device and first device channel.
struct RouteEditor: View {
    @Binding var route: HostRoute
    let devices: [OutputDevice]

    var body: some View {
        VStack(alignment: .leading, spacing: 6) {
            HStack {
                Picker("Tap", selection: kind) {
                    ForEach(TapKind.allCases) { Text($0.label).tag($0) }
                }
                .frame(width: 150)
                if route.source.kind.isStereo {
                    Picker("", selection: index) {
                        ForEach(1...(route.source.kind.maxIndex ?? 1), id: \.self) { Text("\($0)").tag($0) }
                    }
                    .labelsHidden().frame(width: 60)
                    Picker("", selection: side) {
                        ForEach(StereoSide.allCases) { Text($0.rawValue).tag($0) }
                    }
                    .pickerStyle(.segmented).labelsHidden().frame(width: 110)
                } else {
                    Stepper("Ch \(firstSource.wrappedValue)", value: firstSource,
                            in: 1...route.source.kind.width)
                    Stepper("× \(route.source.channels.count)", value: sourceCount,
                            in: 1...max(1, route.source.kind.width - firstSource.wrappedValue + 1))
                }
            }
            HStack {
                Text("→")
                Picker("Device", selection: $route.device) {
                    ForEach(deviceNames, id: \.self) { name in
                        Text(devices.contains { $0.name == name } ? name : "\(name) (not connected)").tag(name)
                    }
                }
                .frame(width: 300)
                Stepper("Out \(ChannelList.format(route.deviceChannels))", value: firstDeviceChannel,
                        in: 1...maxFirstDeviceChannel)
            }
            Text(route.mapLine).font(.caption.monospaced()).foregroundStyle(.secondary)
        }
    }

    private var deviceNames: [String] {
        var names = devices.map(\.name)
        if !names.contains(route.device) { names.insert(route.device, at: 0) }
        return names
    }

    private var maxFirstDeviceChannel: Int {
        let outs = devices.first { $0.name == route.device || $0.uid == route.device }?.outputChannels ?? 64
        return max(1, outs - route.source.lanes + 1)
    }

    /// Keep the device channels contiguous from their first channel, one per lane.
    private func fitDeviceChannels(start: Int? = nil) {
        let s = start ?? route.deviceChannels.first ?? 1
        route.deviceChannels = Array(s..<(s + route.source.lanes))
    }

    private var kind: Binding<TapKind> {
        Binding {
            route.source.kind
        } set: { k in
            route.source = k.isStereo ? TapRef(kind: k) : TapRef(kind: k, channels: [1, 2])
            fitDeviceChannels()
        }
    }

    private var index: Binding<Int> {
        Binding { route.source.index } set: { route.source.index = $0 }
    }

    private var side: Binding<StereoSide> {
        Binding { route.source.side } set: {
            route.source.side = $0
            fitDeviceChannels()
        }
    }

    private var firstSource: Binding<Int> {
        Binding { route.source.channels.first ?? 1 } set: { first in
            let n = min(route.source.channels.count, route.source.kind.width - first + 1)
            route.source.channels = Array(first..<(first + max(1, n)))
            fitDeviceChannels()
        }
    }

    private var sourceCount: Binding<Int> {
        Binding { route.source.channels.count } set: { n in
            let first = route.source.channels.first ?? 1
            route.source.channels = Array(first..<(first + n))
            fitDeviceChannels()
        }
    }

    private var firstDeviceChannel: Binding<Int> {
        Binding { route.deviceChannels.first ?? 1 } set: { fitDeviceChannels(start: $0) }
    }
}
