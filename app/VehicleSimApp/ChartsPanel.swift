import Charts
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

private struct Point: Identifiable {
  let id: Int
  let t: Double
  let value: Double
  let series: String
}

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
    let points = samples.flatMap { s in series.indices.map { i in Point(id: s.id * 8 + i, t: s.t, value: value(s, i), series: series[i]) } }
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
      Chart {
        ForEach(points) { p in
          LineMark(x: .value("t", p.t), y: .value(title, p.value), series: .value("Series", p.series))
            .foregroundStyle(by: .value("Series", p.series))
            .lineStyle(StrokeStyle(lineWidth: 2))
            .interpolationMethod(.linear)
        }
        if let hoverT {
          RuleMark(x: .value("t", hoverT)).foregroundStyle(.secondary.opacity(0.5)).lineStyle(StrokeStyle(lineWidth: 1))
        }
      }
      .chartForegroundStyleScale(domain: series, range: Array(SeriesPalette.colors(scheme).prefix(series.count)))
      .chartXScale(domain: tMin...max(tMax, tMin + 0.1))
      .chartXAxisLabel("time [s]", alignment: .trailing)
      .chartLegend(position: .top, alignment: .leading, spacing: 4)
      .chartXSelection(value: $hoverT)
      .chartYAxis { AxisMarks(position: .leading) { _ in AxisGridLine().foregroundStyle(.secondary.opacity(0.25)); AxisValueLabel() } }
      .chartXAxis { AxisMarks { _ in AxisGridLine().foregroundStyle(.secondary.opacity(0.15)); AxisValueLabel() } }
    }
  }

  private func format(_ v: Double) -> String { abs(v) >= 100 ? String(format: "%.0f", v) : String(format: "%.2f", v) }
}

/// Lateral force vs slip angle for both axles at their static load, with live wheel operating points.
private struct TireCurveChart: View {
  let engine: SimEngine
  @Environment(\.colorScheme) private var scheme

  struct CurvePt: Identifiable { let id: Int; let alpha: Double; let fy: Double; let axle: String }

  var body: some View {
    let _ = engine.paramVersion  // recompute when parameters change
    let loads = engine.staticLoads
    let front = engine.tireCurve(axle: 0, fz: loads.front).enumerated().map { CurvePt(id: $0.offset, alpha: $0.element.0, fy: $0.element.1, axle: "Front tire") }
    let rear = engine.tireCurve(axle: 1, fz: loads.rear).enumerated().map { CurvePt(id: 1000 + $0.offset, alpha: $0.element.0, fy: $0.element.1, axle: "Rear tire") }
    let wheels = engine.telemetry.wheels
    let colors = SeriesPalette.colors(scheme)
    VStack(alignment: .leading, spacing: 4) {
      HStack(alignment: .firstTextBaseline) {
        Text("Tire operating points").font(.headline)
        Text("Fy [N] vs slip angle [deg]").font(.caption).foregroundStyle(.secondary)
      }
      Chart {
        ForEach(front + rear) { p in
          LineMark(x: .value("alpha", p.alpha), y: .value("Fy", p.fy), series: .value("Axle", p.axle))
            .foregroundStyle(by: .value("Axle", p.axle))
            .lineStyle(StrokeStyle(lineWidth: 2, dash: p.axle == "Rear tire" ? [5, 3] : []))
        }
        // Direct labels at the curve ends, so identity does not rely on dash/tone alone.
        if let f = front.last, let r = rear.last {
          PointMark(x: .value("alpha", f.alpha), y: .value("Fy", f.fy)).symbolSize(0)
            .annotation(position: .topLeading, spacing: 2) { Text("front").font(.caption2).foregroundStyle(.secondary) }
          PointMark(x: .value("alpha", r.alpha), y: .value("Fy", r.fy)).symbolSize(0)
            .annotation(position: .bottomLeading, spacing: 2) { Text("rear").font(.caption2).foregroundStyle(.secondary) }
        }
        ForEach(0..<4, id: \.self) { i in
          PointMark(x: .value("alpha", wheels[i].slip_angle * 180 / .pi), y: .value("Fy", wheels[i].fy))
            .symbolSize(90)
            .foregroundStyle(colors[i])
            .annotation(position: .top, spacing: 2) {
              Text(cornerNames[i]).font(.caption2.bold()).foregroundStyle(.secondary)
            }
        }
      }
      .chartForegroundStyleScale(domain: ["Front tire", "Rear tire"], range: [Color.primary.opacity(0.55), Color.secondary.opacity(0.55)])
      .chartXScale(domain: -14...14)
      .chartLegend(position: .top, alignment: .leading, spacing: 4)
      .chartYAxis { AxisMarks(position: .leading) { _ in AxisGridLine().foregroundStyle(.secondary.opacity(0.25)); AxisValueLabel() } }
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
