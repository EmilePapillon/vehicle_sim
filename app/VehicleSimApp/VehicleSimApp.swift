import AppKit
import SwiftUI

final class AppDelegate: NSObject, NSApplicationDelegate {
  func applicationDidFinishLaunching(_ notification: Notification) {
    // Needed when launched as a bare SwiftPM executable (no .app bundle).
    NSApp.setActivationPolicy(.regular)
    NSApp.activate(ignoringOtherApps: true)
  }
  func applicationShouldTerminateAfterLastWindowClosed(_ sender: NSApplication) -> Bool { true }
}

@main
struct VehicleSimApp: App {
  @NSApplicationDelegateAdaptor(AppDelegate.self) private var delegate
  @State private var engine = SimEngine()

  var body: some Scene {
    WindowGroup("Vehicle Dynamics") {
      ContentView(engine: engine)
        .frame(minWidth: 1100, minHeight: 720)
        .onAppear {
          engine.start()
          // `--maneuver <id>` starts with a built-in maneuver (step, circle, sine, dlc, accel, brake).
          let args = CommandLine.arguments
          if let i = args.firstIndex(of: "--maneuver"), i + 1 < args.count,
             let p = engine.presets.first(where: { $0.id == args[i + 1] }) { engine.preset = p }
          Snapshot.runIfRequested(engine: engine)
        }
    }
    .defaultSize(width: 1500, height: 960)
    .commands {
      CommandGroup(replacing: .newItem) {
        Button("Open Vehicle…") { FileActions.openVehicle(engine) }.keyboardShortcut("o")
        Button("Save Vehicle…") { FileActions.saveVehicle(engine) }.keyboardShortcut("s")
        Button("Open Scenario…") { FileActions.openScenario(engine) }.keyboardShortcut("o", modifiers: [.command, .shift])
      }
    }
  }
}

/// `VehicleSimApp --snapshot <dir> [maneuver-id] [seconds]` drives a maneuver, writes PNGs of the
/// 3D view (offscreen Metal), the top-down view, the charts and the whole window, then quits.
/// Used for automated visual checks.
@MainActor
enum Snapshot {
  static func runIfRequested(engine: SimEngine) {
    let args = CommandLine.arguments
    guard let i = args.firstIndex(of: "--snapshot"), i + 1 < args.count else { return }
    let dir = URL(fileURLWithPath: args[i + 1], isDirectory: true)
    let presetID = i + 2 < args.count ? args[i + 2] : "step"
    let seconds = i + 3 < args.count ? Double(args[i + 3]) ?? 3 : 3
    try? FileManager.default.createDirectory(at: dir, withIntermediateDirectories: true)
    if let p = engine.presets.first(where: { $0.id == presetID }) { engine.preset = p }
    engine.running = false
    engine.fastForward(seconds)

    func write(_ data: Data?, _ name: String) {
      guard let data else { print("snapshot: failed \(name)"); return }
      try? data.write(to: dir.appendingPathComponent(name))
      print("snapshot: wrote \(dir.appendingPathComponent(name).path)")
    }
    if let device = MTLCreateSystemDefaultDevice(), let r = Renderer(device: device) {
      r.engine = engine
      write(r.renderPNG(width: 1280, height: 720), "view3d.png")
      r.options.camera = .orbit
      write(r.renderPNG(width: 1280, height: 720), "view3d_orbit.png")
    }
    func render<V: View>(_ v: V, _ size: CGSize, _ name: String) {
      let r = ImageRenderer(content: v.frame(width: size.width, height: size.height).environment(\.colorScheme, .dark)
        .background(Color(white: 0.12)))
      r.scale = 2
      write(r.nsImage.flatMap { img in
        img.tiffRepresentation.flatMap { NSBitmapImageRep(data: $0)?.representation(using: .png, properties: [:]) }
      }, name)
    }
    render(TopDownView(engine: engine, options: ViewOptions()), CGSize(width: 700, height: 500), "topdown.png")
    render(ChartsPanel(engine: engine), CGSize(width: 1100, height: 520), "charts.png")
    render(HUDView(engine: engine).padding(), CGSize(width: 300, height: 240), "hud.png")

    // Whole window after layout settles (Metal layers may appear blank in this capture).
    DispatchQueue.main.asyncAfter(deadline: .now() + 1.5) {
      if let win = NSApp.windows.first(where: { $0.isVisible }), let view = win.contentView,
         let rep = view.bitmapImageRepForCachingDisplay(in: view.bounds) {
        view.cacheDisplay(in: view.bounds, to: rep)
        write(rep.representation(using: .png, properties: [:]), "window.png")
      }
      NSApp.terminate(nil)
    }
  }
}
