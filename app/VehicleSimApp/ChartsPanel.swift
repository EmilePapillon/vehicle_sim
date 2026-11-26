import SwiftUI
import VehicleCAPI

/// Categorical series colors (validated for CVD separation, light and dark steps).
enum SeriesPalette {
  static let light = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]
  static let dark = ["#3987e5", "#d95926", "#199e70", "#c98500"]
  static func colors(_ scheme: ColorScheme) -> [Color] { (scheme == .dark ? dark : light).map(Color.init(hex:)) }
}

extension Color {
  init(hex: String) {
    var v: UInt64 = 0
    Scanner(string: hex.trimmingCharacters(in: CharacterSet(charactersIn: "#"))).scanHexInt64(&v)
    self.init(red: Double((v >> 16) & 0xFF) / 255, green: Double((v >> 8) & 0xFF) / 255, blue: Double(v & 0xFF) / 255)
  }
}

private let cornerNames = ["FL", "FR", "RL", "RR"]

// MARK: - Canvas line plot

// The plots are drawn with Canvas rather than Swift Charts: Charts re-ran its whole layout
// (scales, axes, legend) on every data update, about 18 ms per refresh for this panel, which
// made it the app's largest CPU cost. A Canvas redraw of a few hundred points is far cheaper.

struct PlotSeries {
  let name: String
  let color: Color
  var dash: [CGFloat] = []
  let points: [SIMD2<Double>]
}

struct PlotMarker {
  let x: Double, y: Double
  let color: Color
  let label: String
}

private enum PlotLayout {
  static let left: CGFloat = 46, right: CGFloat = 8, top: CGFloat = 6, bottom: CGFloat = 18, xLabel: CGFloat = 14

  static func rect(_ size: CGSize, hasXLabel: Bool) -> CGRect {
    let b = bottom + (hasXLabel ? xLabel : 0)
    return CGRect(x: left, y: top, width: max(size.width - left - right, 1), height: max(size.height - top - b, 1))
  }

  /// "Nice" tick values (steps of 1, 2, 2.5 or 5 × 10^k) covering `domain` with about `count` intervals.
  static func ticks(_ domain: ClosedRange<Double>, count: Int = 4) -> (values: [Double], step: Double) {
    let span = max(domain.upperBound - domain.lowerBound, 1e-9)
    let raw = span / Double(count)
    let mag = pow(10, floor(log10(raw)))
    let step = [1, 2, 2.5, 5, 10].map { $0 * mag }.first { $0 >= raw * (1 - 1e-9) } ?? 10 * mag
    var v = ceil(domain.lowerBound / step - 1e-9) * step
    var out: [Double] = []
    while v <= domain.upperBound + step * 1e-9 {
      out.append(abs(v) < step * 1e-9 ? 0 : v)
      v += step
    }
    return (out, step)
  }

  /// Auto y domain: data range including zero, widened to whole tick steps.
  static func niceDomain(_ values: [Double]) -> ClosedRange<Double> {
    var lo = min(values.min() ?? 0, 0), hi = max(values.max() ?? 1, 0)
    if hi - lo < 1e-9 { hi = lo + 1 }
    let step = ticks(lo...hi).step
    lo = floor(lo / step + 1e-9) * step
    hi = ceil(hi / step - 1e-9) * step
    return lo...hi
  }

  static func label(_ v: Double, step: Double) -> String {
    var decimals = 0, s = step
    while decimals < 6 && abs(s - s.rounded()) > 1e-6 * max(abs(s), 1) { s *= 10; decimals += 1 }
    return v.formatted(.number.precision(.fractionLength(decimals)))
  }
}

struct LinePlotCanvas: View {
  let series: [PlotSeries]
  var markers: [PlotMarker] = []
  let xDomain: ClosedRange<Double>
  var xLabel: String?
  /// Text drawn at a series' last point, keyed by series name, so identity does not rely on colour alone.
  var endLabels: [String: String] = [:]
  @Binding var hoverX: Double?

  var body: some View {
    let yDomain = PlotLayout.niceDomain(series.flatMap { $0.points.map(\.y) } + markers.map(\.y))
    GeometryReader { geo in
      let plot = PlotLayout.rect(geo.size, hasXLabel: xLabel != nil)
      Canvas { ctx, _ in draw(&ctx, plot: plot, yDomain: yDomain) }
        .onContinuousHover { phase in
          switch phase {
          case .active(let p) where plot.minX...plot.maxX ~= p.x:
            hoverX = xDomain.lowerBound + Double((p.x - plot.minX) / plot.width) * (xDomain.upperBound - xDomain.lowerBound)
          default:
            hoverX = nil
          }
        }
    }
  }

  private func draw(_ ctx: inout GraphicsContext, plot: CGRect, yDomain: ClosedRange<Double>) {
    let xs = xDomain.upperBound - xDomain.lowerBound, ys = yDomain.upperBound - yDomain.lowerBound
    func pt(_ x: Double, _ y: Double) -> CGPoint {
      CGPoint(x: plot.minX + CGFloat((x - xDomain.lowerBound) / xs) * plot.width,
              y: plot.maxY - CGFloat((y - yDomain.lowerBound) / ys) * plot.height)
    }
    func line(_ a: CGPoint, _ b: CGPoint) -> Path { Path { $0.move(to: a); $0.addLine(to: b) } }

    // Grid and tick labels.
    let yt = PlotLayout.ticks(yDomain)
    for v in yt.values {
      let y = pt(xDomain.lowerBound, v).y
      ctx.stroke(line(CGPoint(x: plot.minX, y: y), CGPoint(x: plot.maxX, y: y)), with: .style(.secondary.opacity(0.25)), lineWidth: 0.5)
      ctx.draw(Text(PlotLayout.label(v, step: yt.step)).font(.caption2).foregroundStyle(.secondary),
               at: CGPoint(x: plot.minX - 5, y: y), anchor: .trailing)
    }
    let xt = PlotLayout.ticks(xDomain, count: 5)
    for v in xt.values {
      let x = pt(v, yDomain.lowerBound).x
      ctx.stroke(line(CGPoint(x: x, y: plot.minY), CGPoint(x: x, y: plot.maxY)), with: .style(.secondary.opacity(0.15)), lineWidth: 0.5)
      ctx.draw(Text(PlotLayout.label(v, step: xt.step)).font(.caption2).foregroundStyle(.secondary),
               at: CGPoint(x: x, y: plot.maxY + 3), anchor: .top)
    }
    if let xLabel {
      ctx.draw(Text(xLabel).font(.caption2).foregroundStyle(.secondary),
               at: CGPoint(x: plot.maxX, y: plot.maxY + PlotLayout.bottom + 1), anchor: .topTrailing)
    }

    // Series and markers, clipped to the plot area.
    var inner = ctx
    inner.clip(to: Path(plot.insetBy(dx: 0, dy: -2)))
    for s in series where s.points.count > 1 {
      var path = Path()
      path.addLines(s.points.map { pt($0.x, $0.y) })
      inner.stroke(path, with: .color(s.color), style: StrokeStyle(lineWidth: 2, lineCap: .round, lineJoin: .round, dash: s.dash))
    }
    for s in series {
      if let text = endLabels[s.name], let last = s.points.last {
        let p = pt(last.x, last.y)
        ctx.draw(Text(text).font(.caption2).foregroundStyle(.secondary), at: CGPoint(x: p.x - 2, y: p.y - 3), anchor: .bottomTrailing)
      }
    }
    for m in markers {
      let c = pt(m.x, m.y)
      inner.fill(Path(ellipseIn: CGRect(x: c.x - 5, y: c.y - 5, width: 10, height: 10)), with: .color(m.color))
      ctx.draw(Text(m.label).font(.caption2.bold()).foregroundStyle(.secondary), at: CGPoint(x: c.x, y: c.y - 7), anchor: .bottom)
    }

    if let hoverX, xDomain ~= hoverX {
      let x = pt(hoverX, 0).x
      ctx.stroke(line(CGPoint(x: x, y: plot.minY), CGPoint(x: x, y: plot.maxY)), with: .style(.secondary.opacity(0.5)), lineWidth: 1)
    }
  }
}

/// Legend row: a short line swatch (dashed where the series is) and the name.
private struct PlotLegend: View {
  let series: [PlotSeries]

  var body: some View {
    HStack(spacing: 10) {
      ForEach(series.indices, id: \.self) { i in
        HStack(spacing: 4) {
          Path { $0.move(to: CGPoint(x: 0, y: 2)); $0.addLine(to: CGPoint(x: 14, y: 2)) }
            .stroke(series[i].color, style: StrokeStyle(lineWidth: 2, lineCap: .round, dash: series[i].dash.map { $0 * 0.6 }))
            .frame(width: 14, height: 4)
          Text(series[i].name).font(.caption2).foregroundStyle(.secondary)
        }
      }
    }
  }
}

// MARK: - Charts

/// Time-series chart of one quantity for several series, with a hover crosshair.
private struct TimeChart: View {
  let title: String
  let unit: String
  let series: [String]
  let samples: [Sample]
  let value: (Sample, Int) -> Double
  @Environment(\.colorScheme) private var scheme
  @State private var hoverT: Double?

  var body: some View {
    let colors = SeriesPalette.colors(scheme)
    let plotSeries = series.indices.map { i in
      PlotSeries(name: series[i], color: colors[i], points: samples.map { SIMD2($0.t, value($0, i)) })
    }
    let tMax = samples.last?.t ?? 0
    let tMin = max(0, tMax - (samples.first.map { tMax - $0.t } ?? 0))
    VStack(alignment: .leading, spacing: 4) {
      HStack(alignment: .firstTextBaseline) {
        Text(title).font(.headline)
        Text(unit).font(.caption).foregroundStyle(.secondary)
        Spacer()
        if let hoverT, let s = samples.min(by: { abs($0.t - hoverT) < abs($1.t - hoverT) }) {
          Text(series.indices.map { "\(series[$0]) \(format(value(s, $0)))" }.joined(separator: "  "))
            .font(.caption.monospacedDigit()).foregroundStyle(.secondary)
        }
      }
      PlotLegend(series: plotSeries)
      LinePlotCanvas(series: plotSeries, xDomain: tMin...max(tMax, tMin + 0.1), xLabel: "time [s]", hoverX: $hoverT)
    }
  }

  private func format(_ v: Double) -> String { abs(v) >= 100 ? String(format: "%.0f", v) : String(format: "%.2f", v) }
}

/// Lateral force vs slip angle for both axles at their static load, with live wheel operating points.
private struct TireCurveChart: View {
  let engine: SimEngine
  @Environment(\.colorScheme) private var scheme
  @State private var hoverAlpha: Double?

  var body: some View {
    let curves = engine.tireCurves
    let wheels = engine.sampledTelemetry.wheels
    let colors = SeriesPalette.colors(scheme)
    let series = [
      PlotSeries(name: "Front tire", color: .primary.opacity(0.55), points: curves[0].map { SIMD2($0.alpha, $0.fy) }),
      PlotSeries(name: "Rear tire", color: .secondary.opacity(0.55), dash: [5, 3], points: curves[1].map { SIMD2($0.alpha, $0.fy) }),
    ]
    let markers = (0..<4).map { i in
      PlotMarker(x: wheels[i].slip_angle * 180 / .pi, y: wheels[i].fy, color: colors[i], label: cornerNames[i])
    }
    VStack(alignment: .leading, spacing: 4) {
      HStack(alignment: .firstTextBaseline) {
        Text("Tire operating points").font(.headline)
        Text("Fy [N] vs slip angle [deg]").font(.caption).foregroundStyle(.secondary)
      }
      PlotLegend(series: series)
      LinePlotCanvas(series: series, markers: markers, xDomain: -14...14,
                     endLabels: ["Front tire": "front", "Rear tire": "rear"], hoverX: $hoverAlpha)
    }
  }
}

struct ChartsPanel: View {
  let engine: SimEngine

  var body: some View {
    let samples = engine.history
    Grid(horizontalSpacing: 14, verticalSpacing: 12) {
      GridRow {
        TimeChart(title: "Slip angle", unit: "deg", series: cornerNames, samples: samples) { $0.slip[$1] }
        TimeChart(title: "Wheel load (load transfer)", unit: "N", series: cornerNames, samples: samples) { $0.fz[$1] }
      }
      GridRow {
        TimeChart(title: "Acceleration", unit: "g", series: ["Lateral ay", "Longitudinal ax"], samples: samples) { $1 == 0 ? $0.ay : $0.ax }
        TireCurveChart(engine: engine)
      }
    }
    .padding(12)
  }
}
