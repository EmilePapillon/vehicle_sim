import SwiftUI

/// Auto-generated from the core's parameter registry: every registered parameter gets a control.
struct ParameterSidebar: View {
  let engine: SimEngine
  @State private var search = ""
  @State private var expanded: Set<String> = ["Suspension Front", "Suspension Rear", "Tires Front", "Tires Rear"]

  var body: some View {
    List {
      ForEach(engine.groups) { group in
        let params = filtered(group.params)
        if !params.isEmpty {
          Section(isExpanded: Binding(
            get: { !search.isEmpty || expanded.contains(group.name) },
            set: { if $0 { expanded.insert(group.name) } else { expanded.remove(group.name) } })
          ) {
            ForEach(params) { p in ParamRow(param: p) }
          } header: {
            HStack {
              Text(group.name)
              Spacer()
              let modified = group.params.filter(\.isModified).count
              if modified > 0 {
                Text("\(modified) changed").font(.caption2).foregroundStyle(.secondary)
              }
            }
          }
        }
      }
    }
    .listStyle(.sidebar)
    .searchable(text: $search, placement: .sidebar, prompt: "Filter parameters")
    .safeAreaInset(edge: .bottom) {
      HStack {
        Menu("Vehicle") {
          Button("Sedan (defaults)") { engine.resetParams() }
          Button("Sports coupe") { engine.applySportsPreset() }
          Divider()
          Button("Open Vehicle…") { FileActions.openVehicle(engine) }
          Button("Save Vehicle…") { FileActions.saveVehicle(engine) }
        }
        .fixedSize()
        Spacer()
        Button("Reset All") { engine.resetParams() }
      }
      .padding(10)
      .background(.bar)
    }
  }

  private func filtered(_ ps: [ParamState]) -> [ParamState] {
    guard !search.isEmpty else { return ps }
    return ps.filter { $0.label.localizedCaseInsensitiveContains(search) || $0.key.localizedCaseInsensitiveContains(search) }
  }
}

private struct ParamRow: View {
  @Bindable var param: ParamState

  var body: some View {
    Group {
      if param.isEnum {
        Picker(param.label, selection: Binding(get: { Int(param.value) }, set: { param.value = Double($0) })) {
          ForEach(param.choices.indices, id: \.self) { Text(param.choices[$0]).tag($0) }
        }
      } else if param.isToggle {
        Toggle(param.label, isOn: Binding(get: { param.value != 0 }, set: { param.value = $0 ? 1 : 0 }))
      } else if param.isInteger {
        Stepper("\(param.label): \(Int(param.value))", value: $param.value, in: param.min...param.max, step: 1)
      } else {
        VStack(alignment: .leading, spacing: 1) {
          HStack(alignment: .firstTextBaseline) {
            Text(param.label)
            Spacer()
            Text(formatted).monospacedDigit().foregroundStyle(param.isModified ? Color.accentColor : .secondary)
            Text(param.unit).font(.caption2).foregroundStyle(.tertiary).frame(minWidth: 28, alignment: .leading)
          }
          Slider(value: $param.value, in: param.min...param.max)
            .controlSize(.small)
        }
      }
    }
    .font(.callout)
    .help(param.key)
    .contextMenu {
      Button("Reset to Default (\(format(param.defaultValue)))") { param.value = param.defaultValue }
      Button("Copy Key") {
        NSPasteboard.general.clearContents()
        NSPasteboard.general.setString(param.key, forType: .string)
      }
    }
  }

  private var formatted: String { format(param.value) }

  private func format(_ v: Double) -> String {
    let a = abs(v)
    if a >= 1000 { return String(format: "%.0f", v) }
    if a >= 100 { return String(format: "%.1f", v) }
    if a >= 1 { return String(format: "%.2f", v) }
    return String(format: "%.3f", v)
  }
}

@MainActor
enum FileActions {
  static func openVehicle(_ engine: SimEngine) {
    let panel = NSOpenPanel()
    panel.allowedContentTypes = [.json]
    panel.message = "Choose a vehicle definition"
    guard panel.runModal() == .OK, let url = panel.url,
          let text = try? String(contentsOf: url, encoding: .utf8) else { return }
    _ = engine.loadVehicleJSON(text, name: url.lastPathComponent)
  }

  static func saveVehicle(_ engine: SimEngine) {
    let panel = NSSavePanel()
    panel.allowedContentTypes = [.json]
    panel.nameFieldStringValue = "vehicle.json"
    guard panel.runModal() == .OK, let url = panel.url else { return }
    do {
      try (engine.exportVehicleJSON() + "\n").write(to: url, atomically: true, encoding: .utf8)
      engine.status = "Saved \(url.lastPathComponent)"
    } catch {
      engine.status = "Save failed: \(error.localizedDescription)"
    }
  }

  static func openScenario(_ engine: SimEngine) {
    let panel = NSOpenPanel()
    panel.allowedContentTypes = [.json]
    panel.message = "Choose a scenario (same format as vdsim)"
    guard panel.runModal() == .OK, let url = panel.url else { return }
    engine.loadScenario(url: url)
  }
}
