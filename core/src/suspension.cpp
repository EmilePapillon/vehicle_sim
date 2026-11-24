#include "vd/suspension.hpp"

namespace vd {

namespace {
double bumpStop(double compression, const AxleSuspensionParams& p) {
  const double pen = compression - p.bump_stop_gap;
  // Quadratic engagement avoids a force discontinuity when the stop is touched.
  return pen > 0.0 ? p.bump_stop_rate * pen * (1.0 + 10.0 * pen) : 0.0;
}
}  // namespace

double SuspensionModel::springForce(double compression, const AxleSuspensionParams& p) const {
  return wheelRate(p) * compression + bumpStop(compression, p);
}

// Digressive two-slope damper, coefficients defined at the damper and reflected through MR^2.
double SuspensionModel::damperForce(double rate, const AxleSuspensionParams& p) const {
  const double c = (rate >= 0.0 ? p.damper_bump : p.damper_rebound) * p.motion_ratio * p.motion_ratio;
  const double v = std::abs(rate);
  const double knee = std::max(p.damper_knee, 1e-4);
  const double f = v <= knee ? c * v : c * knee + c * p.damper_high_speed_ratio * (v - knee);
  return rate >= 0.0 ? f : -f;
}

double ProgressiveSuspension::springForce(double compression, const AxleSuspensionParams& p) const {
  const double x = compression;
  // Integral of k0 (1 + g x) dx, with the rate never dropping below 20% of nominal.
  const double k0 = wheelRate(p);
  const double g = p.progression;
  const double xlim = g > 0.0 ? -0.8 / g : -1e9;
  double f;
  if (x >= xlim) {
    f = k0 * (x + 0.5 * g * x * x);
  } else {
    const double flim = k0 * (xlim + 0.5 * g * xlim * xlim);
    f = flim + 0.2 * k0 * (x - xlim);
  }
  return f + bumpStop(compression, p);
}

std::unique_ptr<SuspensionModel> makeSuspensionModel(SuspensionModelType type) {
  if (type == SuspensionModelType::Progressive) return std::make_unique<ProgressiveSuspension>();
  return std::make_unique<LinearSuspension>();
}

}  // namespace vd
