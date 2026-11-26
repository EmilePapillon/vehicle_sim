import AppKit
import Foundation
import GameController
import Observation
import QuartzCore
import simd
import VehicleCAPI

// MARK: - Parameters

@MainActor @Observable
final class ParamState: Identifiable {
  let key: String
  let group: String
  let label: String
  let unit: String
  let min: Double
  let max: Double
  let step: Double
  let defaultValue: Double
  let kind: Int32
  let choices: [String]
  nonisolated var id: String { key }

  @ObservationIgnored weak var engine: SimEngine?
  var value: Double {
    didSet { if value != oldValue, !suppress { engine?.setParam(key, value) } }
  }
  @ObservationIgnored var suppress = false

  init(info: vd_param_info, value: Double) {
    key = String(cString: info.key)
    group = String(cString: info.group)
    label = String(cString: info.label)
    unit = String(cString: info.unit)
    let c = String(cString: info.choices)
    choices = c.isEmpty ? [] : c.components(separatedBy: "|")
    min = info.min
    max = info.max
    step = info.step
    defaultValue = info.default_value
    kind = info.kind
    self.value = value
  }

  var isEnum: Bool { kind == Int32(VD_PARAM_ENUM.rawValue) }
  var isToggle: Bool { kind == Int32(VD_PARAM_INTEGER.rawValue) && min == 0 && max == 1 }
  var isInteger: Bool { kind == Int32(VD_PARAM_INTEGER.rawValue) }
  var isModified: Bool { abs(value - defaultValue) > 1e-9 * Swift.max(1, abs(defaultValue)) }

  // Silently sync from the core (after loading a file) without echoing back.
  func sync(_ v: Double) {
    suppress = true
    value = v
    suppress = false
  }
}

struct ParamGroup: Identifiable {
  let name: String
  var params: [ParamState]
  var id: String { name }
}

// MARK: - Maneuvers

struct ManeuverPreset: Identifiable, Hashable {
  let id: String
  let name: String
  let json: String?  // nil = manual driving
  let speed: Double

  static let manual = ManeuverPreset(id: "manual", name: "Manual (keyboard / gamepad)", json: nil, speed: 0)
  static let builtIn: [ManeuverPreset] = [
    manual,
    ManeuverPreset(id: "step", name: "Step steer @ 80 km/h", json: """
      {"steer": {"type": "step", "angle_deg": 40, "t_start": 1.0, "ramp_time": 0.15},
       "speed": {"type": "hold", "target": 22.22}}
      """, speed: 22.22),
    ManeuverPreset(id: "circle", name: "Constant radius 40 m (speed ramp)", json: """
      {"steer": {"type": "circle", "radius": 40, "laps": 6},
       "speed": {"type": "hold", "target": 8, "accel": 0.25}}
      """, speed: 8),
    ManeuverPreset(id: "sine", name: "Sine sweep 0.2–3 Hz @ 100 km/h", json: """
      {"steer": {"type": "chirp", "amplitude_deg": 20, "f0_hz": 0.2, "f1_hz": 3.0, "t_start": 1.0, "duration": 12},
       "speed": {"type": "hold", "target": 27.78}}
      """, speed: 27.78),
    ManeuverPreset(id: "dlc", name: "Double lane change @ 80 km/h", json: """
      {"steer": {"type": "lane_change", "offset": 3.5},
       "speed": {"type": "hold", "target": 22.22}}
      """, speed: 22.22),
    ManeuverPreset(id: "accel", name: "Full-throttle acceleration", json: """
      {"speed": {"type": "throttle", "value": 1.0, "t_start": 0.5}}
      """, speed: 0),
    ManeuverPreset(id: "brake", name: "Emergency braking from 100 km/h", json: """
      {"speed": {"type": "brake", "value": 1.0, "t_start": 1.0}}
      """, speed: 27.78),
  ]
}

// MARK: - History

struct Sample: Identifiable {
  let id: Int
  let t: Double
  let slip: SIMD4<Double>  // deg, FL FR RL RR
  let fz: SIMD4<Double>    // N
  let ax: Double, ay: Double  // g
  let yawRate: Double    // deg/s
  let beta: Double       // deg
  let roll: Double       // deg
}

extension vd_telemetry {
  var wheels: [vd_wheel_telemetry] { [wheel.0, wheel.1, wheel.2, wheel.3] }
}

// MARK: - Engine

/// CADisplayLink needs an Objective-C target; this forwards to a closure on the main actor.
/// The link does not keep its target alive, so the engine holds it.
private final class DisplayLinkTarget: NSObject {
  let action: @MainActor () -> Void
  init(_ action: @escaping @MainActor () -> Void) { self.action = action }
  @objc func fire() { MainActor.assumeIsolated { action() } }
}

@MainActor @Observable
final class SimEngine {
  @ObservationIgnored let sim: OpaquePointer

  private(set) var telemetry = vd_telemetry()
  private(set) var geometry = vd_geometry()
  private(set) var history: [Sample] = []
  /// Telemetry as of the last history sample. The charts read this instead of `telemetry`,
  /// which changes every tick, so they only re-render at `historyRate`.
  private(set) var sampledTelemetry = vd_telemetry()
  /// Fy(alpha) at static load per axle; recomputed only when parameters change.
  private(set) var tireCurves: [[(alpha: Double, fy: Double)]] = [[], []]
  private(set) var trail: [SIMD2<Double>] = []
  private(set) var referencePath: [SIMD2<Double>] = []
  private(set) var groups: [ParamGroup] = []
  private(set) var paramVersion = 0
  private(set) var presets = ManeuverPreset.builtIn
  var status: String = ""

  var running = true
  var timeScale = 1.0
  var historyWindow = 8.0
  var preset: ManeuverPreset = .manual { didSet { restart() } }

  // Manual driver state (smoothed keyboard / gamepad inputs).
  private(set) var inputs = vd_inputs()
  @ObservationIgnored private var keys = Set<UInt16>()
  @ObservationIgnored private var keyMonitor: Any?
  @ObservationIgnored private var displayLink: CADisplayLink?
  @ObservationIgnored private var displayLinkTarget: DisplayLinkTarget?
  @ObservationIgnored private var lastTick = CACurrentMediaTime()
  @ObservationIgnored private var lastSampleTime = -1.0
  @ObservationIgnored private var sampleCounter = 0
  @ObservationIgnored private var paramsByKey: [String: ParamState] = [:]

  static let historyRate = 20.0
  static let trailLength = 4000

  init() {
    sim = vd_create()
    buildParams()
    vd_get_geometry(sim, &geometry)
    refreshTireCurves()
    restart()
  }

  func start() {
    guard displayLink == nil else { return }
    lastTick = CACurrentMediaTime()
    // Tick in step with the display instead of on an unsynchronised timer.
    let target = DisplayLinkTarget { [weak self] in self?.tick() }
    let link = (NSScreen.main ?? NSScreen.screens[0]).displayLink(target: target, selector: #selector(DisplayLinkTarget.fire))
    // 60 Hz matches the Metal view; on ProMotion displays the link would otherwise run at 120.
    link.preferredFrameRateRange = CAFrameRateRange(minimum: 30, maximum: 60, preferred: 60)
    link.add(to: .main, forMode: .common)
    displayLink = link
    displayLinkTarget = target
    keyMonitor = NSEvent.addLocalMonitorForEvents(matching: [.keyDown, .keyUp]) { [weak self] ev in
      nonisolated(unsafe) let event = ev  // local monitors are delivered on the main thread
      let consumed = MainActor.assumeIsolated { self?.handleKey(event) == nil }
      return consumed ? nil : event
    }
  }

  // MARK: Parameters

  private func buildParams() {
    var order: [String] = []
    var byGroup: [String: [ParamState]] = [:]
    for i in 0..<vd_param_count(sim) {
      var info = vd_param_info()
      guard vd_param_info_at(sim, i, &info) == 0 else { continue }
      var v = 0.0
      vd_get_param(sim, info.key, &v)
      let p = ParamState(info: info, value: v)
      p.engine = self
      if byGroup[p.group] == nil { order.append(p.group) }
      byGroup[p.group, default: []].append(p)
      paramsByKey[p.key] = p
    }
    groups = order.map { ParamGroup(name: $0, params: byGroup[$0]!) }
  }

  func param(_ key: String) -> ParamState? { paramsByKey[key] }

  func setParam(_ key: String, _ value: Double) {
    if vd_set_param(sim, key, value) != 0 {
      status = String(cString: vd_last_error(sim))
    }
    var actual = value
    vd_get_param(sim, key, &actual)
    if let p = paramsByKey[key], p.value != actual { p.sync(actual) }
    vd_get_geometry(sim, &geometry)
    paramsChanged()
  }

  private func syncAllParams() {
    for p in paramsByKey.values {
      var v = 0.0
      vd_get_param(sim, p.key, &v)
      p.sync(v)
    }
    vd_get_geometry(sim, &geometry)
    paramsChanged()
  }

  private func paramsChanged() {
    paramVersion += 1
    refreshTireCurves()
  }

  private func refreshTireCurves() {
    let loads = staticLoads
    tireCurves = [tireCurve(axle: 0, fz: loads.front), tireCurve(axle: 1, fz: loads.rear)]
  }

  func resetParams() {
    vd_reset_params(sim)
    syncAllParams()
    status = "Parameters reset to defaults"
  }

  func loadVehicleJSON(_ text: String, name: String) -> Bool {
    let ok = vd_load_vehicle_json(sim, text) == 0
    let err = String(cString: vd_last_error(sim))
    syncAllParams()
    status = ok ? (err.isEmpty ? "Loaded \(name)" : "Loaded \(name) — \(err)") : "Failed to load \(name): \(err)"
    return ok
  }

  func exportVehicleJSON() -> String {
    guard let c = vd_export_vehicle_json(sim) else { return "{}" }
    defer { vd_free_string(c) }
    return String(cString: c)
  }

  func applySportsPreset() {
    vd_reset_params(sim)
    let overrides: [String: Double] = [
      "chassis.mass": 1180, "chassis.cg_height": 0.46, "chassis.weight_front": 0.50,
      "suspension.front.spring_rate": 60000, "suspension.rear.spring_rate": 55000,
      "suspension.front.arb_rate": 20000, "suspension.rear.arb_rate": 12000,
      "tire.front.mu": 1.25, "tire.rear.mu": 1.25,
      "tire.front.cornering_stiffness": 19, "tire.rear.cornering_stiffness": 21,
      "diff.rear.type": 2, "engine.torque_scale": 1.35, "aero.lift_area": 0.6,
      "aids.abs": 1, "aids.tcs": 1,
    ]
    for (k, v) in overrides { vd_set_param(sim, k, v) }
    syncAllParams()
    status = "Sports coupe preset"
  }

  /// Applies a scenario file (vehicle, params, maneuver) the same way `vdsim run` does.
  func loadScenario(url: URL) {
    do {
      let data = try Data(contentsOf: url)
      guard let root = try JSONSerialization.jsonObject(with: data) as? [String: Any] else {
        status = "Scenario must be a JSON object"
        return
      }
      if let vehicle = root["vehicle"] as? String {
        let vurl = URL(fileURLWithPath: vehicle, relativeTo: url.deletingLastPathComponent())
        let text = try String(contentsOf: vurl, encoding: .utf8)
        _ = loadVehicleJSON(text, name: vurl.lastPathComponent)
      }
      if let params = root["params"] as? [String: Any] {
        for (k, v) in params { if let d = v as? Double { setParam(k, d) } }
      }
      let maneuver = root["maneuver"] ?? [String: Any]()
      let mdata = try JSONSerialization.data(withJSONObject: maneuver)
      let name = (root["name"] as? String) ?? url.deletingPathExtension().lastPathComponent
      let p = ManeuverPreset(id: "file:\(url.path)", name: "Scenario: \(name)",
                             json: String(decoding: mdata, as: UTF8.self),
                             speed: (root["initial_speed"] as? Double) ?? 0)
      presets.removeAll { $0.id == p.id }
      presets.append(p)
      preset = p
      status = "Loaded scenario \(name)"
    } catch {
      status = "Failed to load scenario: \(error.localizedDescription)"
    }
  }

  // MARK: Simulation control

  func restart() {
    if let json = preset.json {
      if vd_set_maneuver(sim, json, preset.speed) != 0 {
        status = "Maneuver error: \(String(cString: vd_last_error(sim)))"
      }
    } else {
      vd_set_maneuver(sim, nil, 0)
      vd_reset(sim, 0, 0, 0, 0)
    }
    inputs = vd_inputs()
    history.removeAll()
    trail.removeAll()
    lastSampleTime = -1
    var n = vd_maneuver_path(sim, nil, 1 << 20)
    var xy = [Double](repeating: 0, count: Int(n) * 2)
    n = vd_maneuver_path(sim, &xy, n)
    referencePath = (0..<Int(n)).map { SIMD2(xy[2 * $0], xy[2 * $0 + 1]) }
    vd_get_telemetry(sim, &telemetry)
    record()
  }

  func toggleRunning() { running.toggle() }

  func stepOnce() {
    running = false
    vd_advance(sim, 1.0 / 60.0)
    vd_get_telemetry(sim, &telemetry)
    record()
  }

  /// Advances the simulation by `seconds` immediately (used by snapshot / tests).
  func fastForward(_ seconds: Double) {
    let frame = 1.0 / 60.0
    var t = 0.0
    while t < seconds {
      vd_advance(sim, frame)
      vd_get_telemetry(sim, &telemetry)
      record()
      t += frame
    }
  }

  private func tick() {
    let now = CACurrentMediaTime()
    let dt = min(now - lastTick, 0.05)
    lastTick = now
    guard running else { return }
    updateManualInputs(dt: dt * timeScale)
    vd_advance(sim, dt * timeScale)
    vd_get_telemetry(sim, &telemetry)
    record()
  }

  private func record() {
    let t = telemetry
    if let last = trail.last {
      if simd_distance(last, SIMD2(t.x, t.y)) > 0.25 { trail.append(SIMD2(t.x, t.y)) }
    } else {
      trail.append(SIMD2(t.x, t.y))
    }
    if trail.count > Self.trailLength { trail.removeFirst(trail.count - Self.trailLength) }

    guard t.time - lastSampleTime >= 1.0 / Self.historyRate || t.time < lastSampleTime else { return }
    lastSampleTime = t.time
    sampledTelemetry = t
    let w = t.wheels
    sampleCounter += 1
    history.append(Sample(
      id: sampleCounter, t: t.time,
      slip: SIMD4(w.map { $0.slip_angle * 180 / .pi }),
      fz: SIMD4(w.map(\.fz)),
      ax: t.ax / 9.80665, ay: t.ay / 9.80665,
      yawRate: t.yaw_rate * 180 / .pi, beta: t.sideslip * 180 / .pi, roll: t.roll * 180 / .pi))
    let cutoff = t.time - historyWindow
    if let first = history.firstIndex(where: { $0.t >= cutoff }), first > 0 { history.removeFirst(first) }
  }

  // MARK: Manual driving

  private func pressed(_ codes: UInt16...) -> Bool { codes.contains { keys.contains($0) } }

  private func updateManualInputs(dt: Double) {
    guard preset.json == nil else { return }
    let speed = max(telemetry.speed, 0)
    var lockDeg = 540.0, ratio = 14.0
    vd_get_param(sim, "steering.max_angle", &lockDeg)
    vd_get_param(sim, "steering.ratio", &ratio)
    // Speed-sensitive steering: limit the road-wheel angle to roughly the grip limit.
    let roadMax = min(lockDeg * .pi / 180 / ratio,
                      atan(geometry.wheelbase * 9.81 * 1.1 / max(speed * speed, 1)) + 2.5 * .pi / 180)
    let swMax = roadMax * ratio

    var steerTarget = 0.0, throttleTarget = 0.0, brakeTarget = 0.0, handbrake = 0.0
    if pressed(123, 0) { steerTarget += swMax }   // left arrow, A
    if pressed(124, 2) { steerTarget -= swMax }   // right arrow, D
    if pressed(126, 13) { throttleTarget = 1 }    // up arrow, W
    if pressed(125, 1) { brakeTarget = 1 }        // down arrow, S
    if pressed(49) { handbrake = 1 }              // space

    var analog = false
    if let pad = GCController.current?.extendedGamepad {
      let x = Double(pad.leftThumbstick.xAxis.value)
      let thr = Double(pad.rightTrigger.value), brk = Double(pad.leftTrigger.value)
      if abs(x) > 0.05 || thr > 0.02 || brk > 0.02 || pad.buttonA.isPressed {
        analog = true
        steerTarget = -x * abs(x) * swMax
        throttleTarget = thr
        brakeTarget = brk
        handbrake = pad.buttonA.isPressed ? 1 : 0
      }
    }

    func approach(_ v: Double, _ target: Double, _ rate: Double) -> Double {
      let d = target - v
      return v + max(-rate * dt, min(rate * dt, d))
    }
    let steerRate = (abs(steerTarget) < abs(inputs.steer) ? 5.0 : 3.0) * max(swMax, 0.5)
    inputs.steer = analog ? steerTarget : approach(inputs.steer, steerTarget, steerRate)
    inputs.throttle = analog ? throttleTarget : approach(inputs.throttle, throttleTarget, 5)
    inputs.brake = analog ? brakeTarget : approach(inputs.brake, brakeTarget, 6)
    inputs.handbrake = handbrake
    inputs.gear = 0
    vd_set_inputs(sim, &inputs)
  }

  private func handleKey(_ ev: NSEvent) -> NSEvent? {
    // Let text fields keep their keys.
    if NSApp.keyWindow?.firstResponder is NSText { return ev }
    if ev.modifierFlags.contains(.command) { return ev }
    let code = ev.keyCode
    let driving: Set<UInt16> = [123, 124, 125, 126, 0, 1, 2, 13, 49]
    if ev.type == .keyDown {
      if driving.contains(code) {
        keys.insert(code)
        return nil
      }
      switch ev.charactersIgnoringModifiers?.lowercased() {
      case "r": restart(); return nil
      case "p": toggleRunning(); return nil
      default: return ev
      }
    } else if ev.type == .keyUp, driving.contains(code) {
      keys.remove(code)
      return nil
    }
    return ev
  }

  // MARK: Analysis helpers

  /// Lateral-force curve of an axle's tire at a given load (for the operating-point plot).
  func tireCurve(axle: Int32, fz: Double, maxDeg: Double = 14, points: Int = 57) -> [(alpha: Double, fy: Double)] {
    (0..<points).map { i in
      let a = -maxDeg + 2 * maxDeg * Double(i) / Double(points - 1)
      var fy = 0.0
      vd_tire_eval(sim, axle, fz, 0, a * .pi / 180, 0, nil, &fy, nil)
      return (a, fy)
    }
  }

  var staticLoads: (front: Double, rear: Double) {
    var m = 1350.0
    vd_get_param(sim, "chassis.mass", &m)
    let L = geometry.wheelbase
    return (0.5 * m * 9.80665 * geometry.b / L, 0.5 * m * 9.80665 * geometry.a / L)
  }
}
