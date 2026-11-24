import SwiftUI
import VehicleCAPI

enum ViewportMode: String, CaseIterable, Identifiable {
  case threeD = "3D", topDown = "Top-down", split = "Split"
  var id: String { rawValue }
}

struct ContentView: View {
  let engine: SimEngine
  @State private var viewport: ViewportMode = .split
  @State private var options = ViewOptions()
  @State private var showCharts = true

  var body: some View {
    NavigationSplitView {
      ParameterSidebar(engine: engine)
        .navigationSplitViewColumnWidth(min: 280, ideal: 320, max: 440)
    } detail: {
      VSplitView {
        viewportArea
          .frame(minHeight: 260, idealHeight: 460)
        if showCharts {
          ChartsPanel(engine: engine)
            .frame(minHeight: 300, idealHeight: 420)
        }
      }
    }
    .toolbar { toolbar }
    .navigationTitle("Vehicle Dynamics")
    .navigationSubtitle(engine.preset.name)
  }

  @ViewBuilder private var viewportArea: some View {
    ZStack(alignment: .topLeading) {
      switch viewport {
      case .threeD:
        MetalViewport(engine: engine, options: options)
      case .topDown:
        TopDownView(engine: engine, options: options)
      case .split:
        HSplitView {
          MetalViewport(engine: engine, options: options).frame(minWidth: 240)
          TopDownView(engine: engine, options: options).frame(minWidth: 200)
        }
      }
      HUDView(engine: engine).padding(10).allowsHitTesting(false)
      VStack {
        Spacer()
        HStack {
          if !engine.status.isEmpty {
            Text(engine.status).font(.caption).padding(.horizontal, 8).padding(.vertical, 4)
              .background(.ultraThinMaterial, in: Capsule())
          }
          Spacer()
          if engine.preset.json == nil {
            Text("W/↑ throttle · S/↓ brake · A D/← → steer · Space handbrake · R reset · P pause · drag to orbit")
              .font(.caption).foregroundStyle(.secondary)
              .padding(.horizontal, 8).padding(.vertical, 4)
              .background(.ultraThinMaterial, in: Capsule())
          }
        }
        .padding(10)
      }
      .allowsHitTesting(false)
    }
  }

  @ToolbarContentBuilder private var toolbar: some ToolbarContent {
    ToolbarItemGroup(placement: .navigation) {
      Button { engine.toggleRunning() } label: {
        Label(engine.running ? "Pause" : "Run", systemImage: engine.running ? "pause.fill" : "play.fill")
      }
      .keyboardShortcut(.space, modifiers: [.command])
      Button { engine.stepOnce() } label: { Label("Step", systemImage: "forward.frame.fill") }
      Button { engine.restart() } label: { Label("Restart", systemImage: "arrow.counterclockwise") }
        .keyboardShortcut("r", modifiers: [.command])
    }
    ToolbarItemGroup(placement: .primaryAction) {
      Picker("Maneuver", selection: Binding(get: { engine.preset }, set: { engine.preset = $0 })) {
        ForEach(engine.presets) { Text($0.name).tag($0) }
      }
      .frame(minWidth: 230)
      .help("Driver: manual keyboard/gamepad, or a scripted maneuver from the core")
      Button { FileActions.openScenario(engine) } label: { Label("Open Scenario", systemImage: "doc.badge.gearshape") }
        .help("Load a vdsim scenario JSON (vehicle, params and maneuver)")
      Picker("Speed", selection: Binding(get: { engine.timeScale }, set: { engine.timeScale = $0 })) {
        Text("0.1×").tag(0.1)
        Text("0.25×").tag(0.25)
        Text("0.5×").tag(0.5)
        Text("1×").tag(1.0)
        Text("2×").tag(2.0)
      }
      .frame(width: 80)
      .help("Simulation time scale")
      Picker("View", selection: $viewport) {
        ForEach(ViewportMode.allCases) { Text($0.rawValue).tag($0) }
      }
      .pickerStyle(.segmented)
      Menu {
        Picker("Camera", selection: $options.camera) {
          ForEach(CameraMode.allCases) { Text($0.rawValue).tag($0) }
        }
        Toggle("Tire forces", isOn: $options.showForces)
        Toggle("Wheel loads", isOn: $options.showLoads)
        Toggle("Trail", isOn: $options.showTrail)
        Toggle("Charts", isOn: $showCharts)
        Divider()
        Picker("Roll/pitch exaggeration", selection: $options.rollExaggeration) {
          ForEach([1.0, 2.0, 3.0, 5.0], id: \.self) { Text("\(Int($0))×").tag($0) }
        }
        Picker("Force vector scale", selection: $options.forceScale) {
          ForEach([0.5, 1.0, 2.0], id: \.self) { Text(String(format: "%g×", $0)).tag($0) }
        }
      } label: {
        Label("Display", systemImage: "eye")
      }
    }
  }
}

/// Live readouts over the viewport.
struct HUDView: View {
  let engine: SimEngine

  var body: some View {
    let t = engine.telemetry
    VStack(alignment: .leading, spacing: 6) {
      HStack(alignment: .firstTextBaseline, spacing: 4) {
        Text(String(format: "%.0f", t.speed * 3.6)).font(.system(size: 30, weight: .semibold, design: .rounded)).monospacedDigit()
        Text("km/h").font(.caption).foregroundStyle(.secondary)
        Text(t.gear == 0 ? "N" : "\(t.gear)").font(.title2.bold()).padding(.leading, 8)
        Text(String(format: "%.0f rpm", t.engine_rpm)).font(.caption).monospacedDigit().foregroundStyle(.secondary)
      }
      Grid(alignment: .leading, horizontalSpacing: 10, verticalSpacing: 2) {
        row("ay", String(format: "%+.2f g", t.ay / 9.80665))
        row("ax", String(format: "%+.2f g", t.ax / 9.80665))
        row("yaw rate", String(format: "%+.1f °/s", t.yaw_rate * 180 / .pi))
        row("sideslip β", String(format: "%+.2f °", t.sideslip * 180 / .pi))
        row("roll / pitch", String(format: "%+.2f° / %+.2f°", t.roll * 180 / .pi, t.pitch * 180 / .pi))
        row("steer", String(format: "%+.0f °", t.steer * 180 / .pi))
        row("time", String(format: "%.2f s", t.time))
      }
      .font(.caption.monospacedDigit())
      HStack(spacing: 6) {
        bar("THR", t.throttle, .green)
        bar("BRK", t.brake, .red)
      }
    }
    .padding(10)
    .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 10))
    .frame(maxWidth: 260, alignment: .leading)
  }

  private func row(_ k: String, _ v: String) -> some View {
    GridRow {
      Text(k).foregroundStyle(.secondary)
      Text(v)
    }
  }

  private func bar(_ label: String, _ v: Double, _ color: Color) -> some View {
    HStack(spacing: 4) {
      Text(label).font(.caption2).foregroundStyle(.secondary)
      ZStack(alignment: .leading) {
        Capsule().fill(.quaternary)
        Capsule().fill(color.opacity(0.8)).frame(width: 70 * max(0, min(1, v)))
      }
      .frame(width: 70, height: 6)
    }
  }
}
