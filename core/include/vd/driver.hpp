// Drivers produce DriverInputs from time and vehicle state. Maneuvers are built from JSON as
// a steering channel plus a speed/pedal channel, e.g.
//   { "steer": {"type": "step", "angle_deg": 40, "t_start": 1},
//     "speed": {"type": "hold", "target": 22.2} }
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "vd/json.hpp"
#include "vd/vehicle.hpp"

namespace vd {

class Driver {
 public:
  virtual ~Driver() = default;
  virtual DriverInputs update(double t, double dt, const Vehicle& vehicle) = 0;
  virtual void reset() {}
};

class SteerChannel {
 public:
  virtual ~SteerChannel() = default;
  virtual double steer(double t, double dt, const Vehicle& v) = 0;  // steering-wheel angle [rad]
  virtual void reset() {}
  // Optional reference path (world XY) for display / metrics.
  virtual const std::vector<std::pair<double, double>>* path() const { return nullptr; }
};

class SpeedChannel {
 public:
  virtual ~SpeedChannel() = default;
  virtual void pedals(double t, double dt, const Vehicle& v, double& throttle, double& brake) = 0;
  virtual void reset() {}
};

// Pure-pursuit path follower with integral cross-track correction.
class PathFollower final : public SteerChannel {
 public:
  explicit PathFollower(std::vector<std::pair<double, double>> pts, double lookaheadTime = 0.45,
                        double minLookahead = 4.0, double ki = 0.02);
  double steer(double t, double dt, const Vehicle& v) override;
  void reset() override { idx_ = 0; integ_ = 0.0; }
  const std::vector<std::pair<double, double>>* path() const override { return &pts_; }
  double crossTrackError() const { return lastError_; }

 private:
  std::vector<std::pair<double, double>> pts_;
  std::vector<double> s_;  // cumulative arc length
  double lookaheadTime_, minLookahead_, ki_;
  size_t idx_ = 0;
  double integ_ = 0.0, lastError_ = 0.0;
};

// PI speed controller acting on throttle and brake.
class SpeedHold final : public SpeedChannel {
 public:
  SpeedHold(double target, double accel, double kp = 0.35, double ki = 0.12)
      : target_(target), accel_(accel), kp_(kp), ki_(ki) {}
  void pedals(double t, double dt, const Vehicle& v, double& throttle, double& brake) override;
  void reset() override { integ_ = 0.0; }
  double target(double t) const { return target_ + accel_ * t; }

 private:
  double target_, accel_, kp_, ki_;
  double integ_ = 0.0;
};

class ManeuverDriver final : public Driver {
 public:
  ManeuverDriver(std::unique_ptr<SteerChannel> steer, std::unique_ptr<SpeedChannel> speed)
      : steer_(std::move(steer)), speed_(std::move(speed)) {}
  DriverInputs update(double t, double dt, const Vehicle& vehicle) override;
  void reset() override;
  const SteerChannel* steerChannel() const { return steer_.get(); }

 private:
  std::unique_ptr<SteerChannel> steer_;
  std::unique_ptr<SpeedChannel> speed_;
};

// Builds a maneuver from JSON. Returns nullptr and sets *error on invalid specs.
std::unique_ptr<ManeuverDriver> makeManeuver(const json::Value& spec, std::string* error);

// Runs `driver` on `vehicle` for `duration`, calling `sample` at `outputRate` Hz (and at t=0).
template <class SampleFn>
void simulate(Vehicle& vehicle, Driver& driver, double duration, double outputRate, SampleFn&& sample) {
  const double dt = vehicle.params().solver.dt;
  const double outDt = outputRate > 0 ? 1.0 / outputRate : dt;
  double nextOut = 0.0;
  const long steps = long(std::llround(duration / dt));
  for (long k = 0; k <= steps; ++k) {
    const double t = vehicle.time();
    if (t + 1e-9 >= nextOut) {
      sample(vehicle.telemetry());
      nextOut += outDt;
    }
    if (k == steps) break;
    vehicle.setInputs(driver.update(t, dt, vehicle));
    vehicle.step(dt);
  }
}

}  // namespace vd
