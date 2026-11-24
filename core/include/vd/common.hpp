// Shared math helpers, constants and conventions for VehicleCore.
//
// Conventions (ISO 8855 style, right-handed):
//   body frame: x forward, y left, z up
//   roll  (phi)   positive = left side up
//   pitch (theta) positive = nose down
//   yaw   (psi)   positive = counter-clockwise seen from above (left turn)
//   slip angle alpha is defined so that positive alpha gives positive (leftward) Fy.
//   corners are indexed FL=0, FR=1, RL=2, RR=3.
#pragma once

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace vd {

constexpr double kPi = 3.14159265358979323846;
constexpr double kGravity = 9.80665;

constexpr double deg2rad(double d) { return d * kPi / 180.0; }
constexpr double rad2deg(double r) { return r * 180.0 / kPi; }
constexpr double radps2rpm(double w) { return w * 30.0 / kPi; }
constexpr double rpm2radps(double rpm) { return rpm * kPi / 30.0; }

inline double clamp(double v, double lo, double hi) { return std::min(std::max(v, lo), hi); }
inline double sgn(double v) { return double((v > 0.0) - (v < 0.0)); }
inline double smoothstep(double e0, double e1, double x) {
  const double t = clamp((x - e0) / (e1 - e0), 0.0, 1.0);
  return t * t * (3.0 - 2.0 * t);
}

enum Corner : int { FL = 0, FR = 1, RL = 2, RR = 3 };
enum Axle : int { Front = 0, Rear = 1 };
constexpr int axleOf(int corner) { return corner < 2 ? Front : Rear; }
constexpr bool isLeft(int corner) { return (corner & 1) == 0; }
constexpr double sideSign(int corner) { return isLeft(corner) ? 1.0 : -1.0; }

// Piecewise-linear lookup table, clamped at both ends. x must be ascending.
class Table1D {
 public:
  Table1D() = default;
  Table1D(std::vector<double> x, std::vector<double> y) : x_(std::move(x)), y_(std::move(y)) {}

  double operator()(double x) const {
    if (x_.empty()) return 0.0;
    if (x <= x_.front()) return y_.front();
    if (x >= x_.back()) return y_.back();
    const auto it = std::upper_bound(x_.begin(), x_.end(), x);
    const size_t i = size_t(it - x_.begin());
    const double t = (x - x_[i - 1]) / (x_[i] - x_[i - 1]);
    return y_[i - 1] + t * (y_[i] - y_[i - 1]);
  }
  double maxY() const { return y_.empty() ? 0.0 : *std::max_element(y_.begin(), y_.end()); }
  bool empty() const { return x_.empty(); }
  size_t size() const { return x_.size(); }
  const std::vector<double>& xs() const { return x_; }
  const std::vector<double>& ys() const { return y_; }

 private:
  std::vector<double> x_, y_;
};

}  // namespace vd
