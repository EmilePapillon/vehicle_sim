// Full vehicle model.
//
// Degrees of freedom (28 states):
//   planar body: X, Y, yaw, u (vx), v (vy), r (yaw rate)
//   sprung mass: heave z, roll phi, pitch theta (+ rates) on per-corner spring/damper/ARB
//   4 wheel spin speeds, 4 longitudinal + 4 lateral transient slip states (relaxation length)
//   engine speed (coupled through an automatic clutch, gearbox and differentials)
//   3 torsional twist angles for locked differentials
//
// Wheel loads combine static load, elastic transfer through the suspension (body dynamics),
// geometric transfer through roll/pitch centres and unsprung-mass transfer.
#pragma once

#include <array>
#include <memory>

#include "vd/params.hpp"

namespace vd {

struct DriverInputs {
  double steer = 0.0;     // steering-wheel angle [rad], positive = left
  double throttle = 0.0;  // 0..1
  double brake = 0.0;     // 0..1
  double handbrake = 0.0; // 0..1
  int gear = 0;           // 0 = automatic, >0 = force gear, -1 = neutral
};

struct WheelTelemetry {
  double fz = 0, fx = 0, fy = 0, mz = 0;  // tire forces, wheel frame [N, N m]
  double slip_ratio = 0, slip_angle = 0;  // [-], [rad]
  double omega = 0, spin_angle = 0;       // [rad/s], [rad]
  double steer = 0, camber = 0;           // road-wheel steer, inclination [rad]
  double compression = 0;                 // suspension compression from static [m]
  double drive_torque = 0, brake_torque = 0;
};

struct Telemetry {
  double time = 0;
  double x = 0, y = 0, yaw = 0;            // world position [m], heading [rad]
  double heave = 0, roll = 0, pitch = 0;   // sprung-mass motion [m, rad]
  double vx = 0, vy = 0, yaw_rate = 0;     // body-frame velocities
  double speed = 0, ax = 0, ay = 0;        // [m/s], body accelerations [m/s^2]
  double sideslip = 0;                     // body slip angle beta [rad]
  double steer = 0, throttle = 0, brake = 0;
  double engine_rpm = 0, engine_torque = 0, clutch_torque = 0;
  int gear = 0;
  double distance = 0;                     // travelled path length [m]
  std::array<WheelTelemetry, 4> wheel{};
};

struct Geometry {
  double wheelbase, a, b, track_front, track_rear, cg_height;
  double wheel_radius_front, wheel_radius_rear;
};

class Vehicle {
 public:
  enum StateIndex : int {
    X = 0, Y, PSI, U, V, R,
    Z, ZD, PHI, PHID, THETA, THETAD,
    W0, K0 = W0 + 4, A0 = K0 + 4, WE = A0 + 4,
    TW0,  // locked-diff twist angles: front, rear, centre
    kNumStates = TW0 + 3
  };
  using State = std::array<double, kNumStates>;

  explicit Vehicle(const VehicleParams& p = VehicleParams{});
  Vehicle(const Vehicle&) = delete;
  Vehicle& operator=(const Vehicle&) = delete;

  VehicleParams& params() { return p_; }
  const VehicleParams& params() const { return p_; }
  ParamRegistry& registry() { return registry_; }
  const ParamRegistry& registry() const { return registry_; }

  // Sets a parameter by key and refreshes derived quantities / models. Safe mid-run.
  bool setParam(const std::string& key, double value);
  bool loadParams(const json::Value& root, std::vector<std::string>* unknownKeys, std::string* error);
  // Call after editing params() directly.
  void onParamsChanged();

  // Places the car at rest on its suspension, rolling straight at `speed`.
  void reset(double x = 0.0, double y = 0.0, double yaw = 0.0, double speed = 0.0);
  void setInputs(const DriverInputs& in) { inputs_ = in; }
  const DriverInputs& inputs() const { return inputs_; }

  void step(double dt);         // one integrator step of size dt
  void advance(double duration);// fixed steps of solver.dt; remainders carry over
  double time() const { return time_; }

  const Telemetry& telemetry() const { return tel_; }
  const State& state() const { return s_; }
  Geometry geometry() const;
  TireForces evalTire(int axle, const TireInput& in) const;
  const TireModel& tireModel(int axle) const { return *tires_[size_t(axle)]; }

 private:
  struct Derived {
    double L = 0, a = 0, b = 0, ms = 0, mu_total = 0;
    double hs = 0;                // sprung CG height
    double roll_arm = 0, pitch_arm = 0;
    std::array<double, 4> x{}, y{}, static_fz{}, sprung_static{}, mu{};
  };

  void computeDerived();
  void computeSteer();
  void updateGearbox(double dt);
  void updateAids(double dt);
  void derivatives(const State& s, State& ds, Telemetry* tel) const;

  VehicleParams p_;
  ParamRegistry registry_{p_};
  Derived d_;
  std::array<std::unique_ptr<TireModel>, 2> tires_;
  std::unique_ptr<SuspensionModel> suspension_[2];
  Drivetrain drivetrain_;

  State s_{};
  DriverInputs inputs_;
  Telemetry tel_;
  std::array<double, 4> steer_{};
  std::array<double, 4> spin_{};
  std::array<double, 4> absFactor_{1.0, 1.0, 1.0, 1.0};
  double tcsFactor_ = 1.0;
  double ax_prev_ = 0.0, ay_prev_ = 0.0;  // last accepted accelerations for load transfer
  double time_ = 0.0, accum_ = 0.0, distance_ = 0.0;
  int gear_ = 1, target_gear_ = 1;
  double shift_timer_ = 0.0;
};

}  // namespace vd
