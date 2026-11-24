import SwiftUI
import VehicleCAPI

/// 2D bird's-eye view: car, wheels with steer, tire force vectors, wheel-load circles,
/// velocity vector, driven trail and maneuver reference path. Follows the car.
struct TopDownView: View {
  let engine: SimEngine
  var options: ViewOptions
  @State private var zoom: CGFloat = 26  // px per metre

  var body: some View {
    Canvas { ctx, size in
      draw(ctx: ctx, size: size)
    }
    .background(Color(red: 0.16, green: 0.17, blue: 0.19))
    .gesture(MagnifyGesture().onChanged { v in zoom = min(max(26 * v.magnification, 6), 120) })
    .onScrollZoom { zoom = min(max(zoom * $0, 6), 120) }
  }

  private func draw(ctx: GraphicsContext, size: CGSize) {
    let t = engine.telemetry
    let g = engine.geometry
    let s = zoom
    let center = CGPoint(x: size.width / 2, y: size.height / 2)
    // World (x fwd, y left, z up) -> screen (y down). Camera follows the car, north up.
    func screen(_ x: Double, _ y: Double) -> CGPoint {
      CGPoint(x: center.x + CGFloat(x - t.x) * s, y: center.y - CGFloat(y - t.y) * s)
    }

    // Grid
    let spacing: Double = s > 15 ? 1 : 5
    let halfW = Double(size.width / s) / 2 + spacing, halfH = Double(size.height / s) / 2 + spacing
    var minor = Path(), major = Path()
    let isMajor = { (v: Double) in abs(v.truncatingRemainder(dividingBy: 10)) < 1e-6 }
    var x = floor((t.x - halfW) / spacing) * spacing
    while x <= t.x + halfW {
      let a = screen(x, t.y - halfH), b = screen(x, t.y + halfH)
      if isMajor(x) { major.move(to: a); major.addLine(to: b) } else { minor.move(to: a); minor.addLine(to: b) }
      x += spacing
    }
    var y = floor((t.y - halfH) / spacing) * spacing
    while y <= t.y + halfH {
      let a = screen(t.x - halfW, y), b = screen(t.x + halfW, y)
      if isMajor(y) { major.move(to: a); major.addLine(to: b) } else { minor.move(to: a); minor.addLine(to: b) }
      y += spacing
    }
    ctx.stroke(minor, with: .color(.white.opacity(0.05)), lineWidth: 1)
    ctx.stroke(major, with: .color(.white.opacity(0.14)), lineWidth: 1)

    // Reference path and trail
    if engine.referencePath.count > 1 {
      var p = Path()
      p.addLines(engine.referencePath.map { screen($0.x, $0.y) })
      ctx.stroke(p, with: .color(.yellow.opacity(0.7)), style: StrokeStyle(lineWidth: 2, dash: [6, 5]))
    }
    if options.showTrail, engine.trail.count > 1 {
      var p = Path()
      p.addLines(engine.trail.map { screen($0.x, $0.y) } + [screen(t.x, t.y)])
      ctx.stroke(p, with: .color(.cyan.opacity(0.75)), lineWidth: 2)
    }

    // Car-local drawing: rotate so the car's x axis follows its heading.
    var car = ctx
    car.translateBy(x: center.x, y: center.y)
    car.rotate(by: .radians(-t.yaw))
    car.scaleBy(x: s, y: -s)  // local metres, y left = up on screen before rotation

    let len = g.wheelbase + 1.75, wid = max(g.track_front, g.track_rear) + 0.3
    let cx = (g.a - g.b) / 2
    let bodyRect = CGRect(x: cx - len / 2, y: -wid / 2, width: len, height: wid)
    car.fill(Path(roundedRect: bodyRect, cornerRadius: 0.35), with: .color(Color(red: 0.78, green: 0.18, blue: 0.14).opacity(0.85)))
    car.fill(Path(roundedRect: CGRect(x: cx - 0.35 - len * 0.24, y: -wid * 0.43, width: len * 0.48, height: wid * 0.86), cornerRadius: 0.2),
             with: .color(.black.opacity(0.35)))
    car.stroke(Path(roundedRect: bodyRect, cornerRadius: 0.35), with: .color(.white.opacity(0.6)), lineWidth: 0.03)

    let pos: [(Double, Double)] = [(g.a, g.track_front / 2), (g.a, -g.track_front / 2),
                                   (-g.b, g.track_rear / 2), (-g.b, -g.track_rear / 2)]
    let (sf, sr) = engine.staticLoads
    let fscale = options.forceScale / 3000.0
    for (i, w) in t.wheels.enumerated() {
      var wc = car
      wc.translateBy(x: pos[i].0, y: pos[i].1)
      // Load circle (area ~ load, colour = ratio to static)
      if options.showLoads {
        let ratio = w.fz / (i < 2 ? sf : sr)
        let r = 0.55 * sqrt(max(w.fz, 0) / 4000)
        let col = Color(hue: 0.62 - 0.55 * min(max((ratio - 0.4) / 1.2, 0), 1), saturation: 0.75, brightness: 1)
        wc.fill(Path(ellipseIn: CGRect(x: -r, y: -r, width: 2 * r, height: 2 * r)), with: .color(col.opacity(0.35)))
        wc.stroke(Path(ellipseIn: CGRect(x: -r, y: -r, width: 2 * r, height: 2 * r)), with: .color(col), lineWidth: 0.03)
      }
      wc.rotate(by: .radians(w.steer))
      let r = i < 2 ? g.wheel_radius_front : g.wheel_radius_rear
      wc.fill(Path(roundedRect: CGRect(x: -r, y: -0.11, width: 2 * r, height: 0.22), cornerRadius: 0.04), with: .color(.black))
      if options.showForces {
        arrow(wc, to: CGPoint(x: w.fx * fscale, y: 0), color: .green)
        arrow(wc, to: CGPoint(x: 0, y: w.fy * fscale), color: .red)
      }
    }
    // Velocity vector at the CG
    arrow(car, to: CGPoint(x: t.vx * 0.12, y: t.vy * 0.12), color: .white, width: 0.05)
  }

  private func arrow(_ ctx: GraphicsContext, to tip: CGPoint, color: Color, width: CGFloat = 0.07) {
    let l = hypot(tip.x, tip.y)
    guard l > 0.03 else { return }
    var p = Path()
    p.move(to: .zero)
    p.addLine(to: tip)
    let ux = tip.x / l, uy = tip.y / l, h = min(0.25, l * 0.4)
    p.move(to: CGPoint(x: tip.x - ux * h - uy * h * 0.5, y: tip.y - uy * h + ux * h * 0.5))
    p.addLine(to: tip)
    p.addLine(to: CGPoint(x: tip.x - ux * h + uy * h * 0.5, y: tip.y - uy * h - ux * h * 0.5))
    ctx.stroke(p, with: .color(color), style: StrokeStyle(lineWidth: width, lineCap: .round, lineJoin: .round))
  }
}

private struct ScrollZoom: ViewModifier {
  let action: (CGFloat) -> Void
  func body(content: Content) -> some View {
    content.onContinuousHover { _ in }.background(ScrollCatcher(action: action))
  }
}

private struct ScrollCatcher: NSViewRepresentable {
  let action: (CGFloat) -> Void
  func makeNSView(context: Context) -> NSView {
    let v = CatcherView()
    v.action = action
    return v
  }
  func updateNSView(_ v: NSView, context: Context) { (v as? CatcherView)?.action = action }

  final class CatcherView: NSView {
    var action: ((CGFloat) -> Void)?
    private var monitor: Any?
    override func viewDidMoveToWindow() {
      super.viewDidMoveToWindow()
      if let monitor { NSEvent.removeMonitor(monitor) }
      monitor = nil
      guard window != nil else { return }
      monitor = NSEvent.addLocalMonitorForEvents(matching: .scrollWheel) { [weak self] ev in
        guard let self, let win = self.window, ev.window === win else { return ev }
        let p = self.convert(ev.locationInWindow, from: nil)
        guard self.bounds.contains(p) else { return ev }
        self.action?(1 + ev.scrollingDeltaY * 0.01)
        return nil
      }
    }
  }
}

extension View {
  func onScrollZoom(_ action: @escaping (CGFloat) -> Void) -> some View { modifier(ScrollZoom(action: action)) }
}
