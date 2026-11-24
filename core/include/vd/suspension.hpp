// Suspension: per-axle parameters and a swappable corner force model.
// Forces are deviations from static equilibrium, positive pushing the body up.
#pragma once

#include <memory>

#include "vd/common.hpp"

namespace vd {

enum class SuspensionModelType : int { Linear = 0, Progressive = 1 };
constexpr const char* kSuspensionModelChoices = "Linear|Progressive";

struct AxleSuspensionParams {
  int model = int(SuspensionModelType::Linear);
  double spring_rate = 32000.0;          // spring rate [N/m]
  double motion_ratio = 1.0;             // wheel travel / spring travel (wheel rate = k * MR^2)
  double progression = 4.0;              // Progressive: rate increase per metre of travel [1/m]
  double arb_rate = 12000.0;             // anti-roll bar rate at the wheel [N/m]
  double damper_bump = 3500.0;           // low-speed bump damping [N s/m] at the damper
  double damper_rebound = 5500.0;        // low-speed rebound damping [N s/m]
  double damper_knee = 0.12;             // knee speed [m/s]
  double damper_high_speed_ratio = 0.4;  // high-speed slope relative to low-speed slope
  double bump_stop_gap = 0.07;           // compression from static before bump stop [m]
  double bump_stop_rate = 250000.0;      // [N/m]
  double roll_center_height = 0.06;      // [m]
  double track = 1.56;                   // [m]
  double static_camber = -1.0;           // [deg], negative = top inwards
  double camber_roll_coeff = 0.6;        // fraction of body roll the wheel inherits [-]
  double toe = 0.0;                      // [deg], positive = toe-in
};

class SuspensionModel {
 public:
  virtual ~SuspensionModel() = default;
  virtual const char* name() const = 0;
  // compression [m] and its rate [m/s] at the wheel, relative to static ride height.
  virtual double springForce(double compression, const AxleSuspensionParams& p) const;
  virtual double damperForce(double rate, const AxleSuspensionParams& p) const;
  double cornerForce(double compression, double rate, const AxleSuspensionParams& p) const {
    return springForce(compression, p) + damperForce(rate, p);
  }
  static double wheelRate(const AxleSuspensionParams& p) { return p.spring_rate * p.motion_ratio * p.motion_ratio; }
};

class LinearSuspension final : public SuspensionModel {
 public:
  const char* name() const override { return "Linear"; }
};

// Rising-rate spring: k(x) = k0 (1 + progression * x) around static ride height.
class ProgressiveSuspension final : public SuspensionModel {
 public:
  const char* name() const override { return "Progressive"; }
  double springForce(double compression, const AxleSuspensionParams& p) const override;
};

std::unique_ptr<SuspensionModel> makeSuspensionModel(SuspensionModelType type);

}  // namespace vd
