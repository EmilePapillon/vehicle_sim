#include "vd/tire.hpp"

namespace vd {

namespace {

constexpr double kEps = 1e-9;

// Magic Formula y(x) = D sin(C atan(Bx - E(Bx - atan Bx))).
double magic(double x, double B, double C, double D, double E) {
  const double bx = B * x;
  return D * std::sin(C * std::atan(bx - E * (bx - std::atan(bx))));
}

// Slip value at which the Magic Formula peaks (solves Bx - E(Bx - atan Bx) = tan(pi/2C)).
double magicPeak(double B, double C, double E) {
  if (B < kEps) return 1.0;
  if (C <= 1.0 + 1e-6) return 6.0 / B;  // no peak: saturating curve
  const double target = std::tan(kPi / (2.0 * C));
  double z = target;  // z = B x
  for (int i = 0; i < 20; ++i) {
    const double f = z - E * (z - std::atan(z)) - target;
    const double df = 1.0 - E * (1.0 - 1.0 / (1.0 + z * z));
    if (std::abs(df) < kEps) break;
    const double dz = f / df;
    z -= dz;
    if (z < 1e-3) z = 1e-3;
    if (std::abs(dz) < 1e-10) break;
  }
  return z / B;
}

TireForces capToFrictionCircle(TireForces f, double fmax) {
  const double mag = std::hypot(f.fx, f.fy);
  if (mag > fmax && mag > kEps) {
    const double s = fmax / mag;
    f.fx *= s;
    f.fy *= s;
  }
  return f;
}

}  // namespace

double frictionCoefficient(const TireParams& p, double fz) {
  const double dfz = (fz - p.nominal_load) / std::max(p.nominal_load, 1.0);
  return std::max(0.05, p.mu * (1.0 + p.load_sensitivity * dfz));
}

double TireModel::corneringStiffness(const TireParams& p, double fz) const {
  return p.cornering_stiffness * std::max(fz, 0.0);
}

double TireModel::longitudinalStiffness(const TireParams& p, double fz) const {
  return p.longitudinal_stiffness * std::max(fz, 0.0);
}

// ---------------------------------------------------------------- Linear

TireForces LinearTire::compute(const TireInput& in, const TireParams& p) const {
  TireForces f;
  if (in.fz <= 0.0) return f;
  f.fx = longitudinalStiffness(p, in.fz) * in.kappa;
  f.fy = corneringStiffness(p, in.fz) * in.alpha + p.camber_stiffness * in.fz * in.gamma;
  f = capToFrictionCircle(f, frictionCoefficient(p, in.fz) * in.fz);
  f.mz = -p.pneumatic_trail * f.fy;
  return f;
}

// ---------------------------------------------------------------- Brush

TireForces BrushTire::compute(const TireInput& in, const TireParams& p) const {
  TireForces f;
  if (in.fz <= 0.0) return f;
  const double mu = frictionCoefficient(p, in.fz);
  const double fmax = mu * in.fz;
  const double cy = corneringStiffness(p, in.fz);
  const double cx = longitudinalStiffness(p, in.fz);
  // Camber thrust is modelled as an equivalent slip-angle shift.
  const double alpha = in.alpha + (cy > kEps ? p.camber_stiffness * in.fz * in.gamma / cy : 0.0);
  // Theoretical slip quantities.
  const double denom = std::max(1.0 + in.kappa, 0.05);
  const double sx = in.kappa / denom;
  const double sy = std::tan(clamp(alpha, -1.4, 1.4)) / denom;
  const double phix = cx * sx / (3.0 * fmax);
  const double phiy = cy * sy / (3.0 * fmax);
  const double phi = std::hypot(phix, phiy);
  if (phi < kEps) return f;
  double fmag;
  double trailFactor;
  if (phi < 1.0) {
    fmag = fmax * (3.0 * phi - 3.0 * phi * phi + phi * phi * phi);
    // Brush aligning-moment trail shrinks towards full sliding.
    trailFactor = (1.0 - phi) * (1.0 - phi) * (1.0 - phi) / std::max(1.0 - phi + phi * phi / 3.0, kEps);
  } else {
    fmag = fmax;
    trailFactor = 0.0;
  }
  f.fx = fmag * phix / phi;
  f.fy = fmag * phiy / phi;
  f.mz = -p.pneumatic_trail * trailFactor * f.fy;
  return f;
}

// ---------------------------------------------------------------- Pacejka

double PacejkaTire::corneringStiffness(const TireParams& p, double fz) const {
  // K = c * Fz0 * k * sin(2 atan(Fz / (k Fz0))), normalised so K(Fz0) = cornering_stiffness * Fz0.
  const double k = std::max(p.cs_load_peak, 0.1);
  const double fz0 = std::max(p.nominal_load, 1.0);
  const double norm = (k * k + 1.0) / (2.0 * k);
  return p.cornering_stiffness * fz0 * norm * std::sin(2.0 * std::atan(std::max(fz, 0.0) / (k * fz0)));
}

double PacejkaTire::pureLateral(const TireParams& p, double fz, double alpha) const {
  if (fz <= 0.0) return 0.0;
  const double D = frictionCoefficient(p, fz) * fz;
  const double C = p.shape_lat;
  const double B = corneringStiffness(p, fz) / std::max(C * D, kEps);
  return magic(alpha, B, C, D, std::min(p.curvature_lat, 1.0));
}

double PacejkaTire::pureLongitudinal(const TireParams& p, double fz, double kappa) const {
  if (fz <= 0.0) return 0.0;
  const double D = frictionCoefficient(p, fz) * fz;
  const double C = p.shape_long;
  const double B = longitudinalStiffness(p, fz) / std::max(C * D, kEps);
  return magic(kappa, B, C, D, std::min(p.curvature_long, 1.0));
}

double PacejkaTire::peakSlipAngle(const TireParams& p, double fz) const {
  const double D = frictionCoefficient(p, fz) * fz;
  const double B = corneringStiffness(p, fz) / std::max(p.shape_lat * D, kEps);
  return magicPeak(B, p.shape_lat, std::min(p.curvature_lat, 1.0));
}

double PacejkaTire::peakSlipRatio(const TireParams& p, double fz) const {
  const double D = frictionCoefficient(p, fz) * fz;
  const double B = longitudinalStiffness(p, fz) / std::max(p.shape_long * D, kEps);
  return magicPeak(B, p.shape_long, std::min(p.curvature_long, 1.0));
}

TireForces PacejkaTire::compute(const TireInput& in, const TireParams& p) const {
  TireForces f;
  if (in.fz <= 0.0) return f;
  const double ky = corneringStiffness(p, in.fz);
  const double alpha = in.alpha + (ky > kEps ? p.camber_stiffness * in.fz * in.gamma / ky : 0.0);
  const double ap = peakSlipAngle(p, in.fz);
  const double kp = peakSlipRatio(p, in.fz);
  // Normalised combined slip: each direction is scaled by its own peak slip, the total slip
  // magnitude is evaluated on the pure curves and redistributed along the slip direction.
  const double nx = in.kappa / kp;
  const double ny = alpha / ap;
  const double s = std::hypot(nx, ny);
  if (s < kEps) return f;
  f.fx = pureLongitudinal(p, in.fz, s * kp) * nx / s;
  f.fy = pureLateral(p, in.fz, s * ap) * ny / s;
  const double trail = p.pneumatic_trail * std::max(0.0, 1.0 - s);
  f.mz = -trail * f.fy;
  return f;
}

std::unique_ptr<TireModel> makeTireModel(TireModelType type) {
  switch (type) {
    case TireModelType::Linear: return std::make_unique<LinearTire>();
    case TireModelType::Brush: return std::make_unique<BrushTire>();
    case TireModelType::Pacejka: break;
  }
  return std::make_unique<PacejkaTire>();
}

}  // namespace vd
