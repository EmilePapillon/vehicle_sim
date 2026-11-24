// Drivetrain: engine, automatic clutch, gearbox, differentials (axle + centre) and brakes.
#pragma once

#include <array>
#include <memory>

#include "vd/common.hpp"

namespace vd {

constexpr int kMaxGears = 8;

struct EngineParams {
  Table1D torque_curve{{800, 1500, 2500, 3500, 4500, 5500, 6500, 7200},
                       {150, 205, 245, 268, 275, 265, 238, 205}};  // full-load [rpm -> N m]
  double torque_scale = 1.0;
  double idle_rpm = 900.0;
  double rev_limit = 7200.0;
  double inertia = 0.18;          // engine + flywheel [kg m^2]
  double friction_base = 15.0;    // closed-throttle drag [N m]
  double friction_per_krpm = 4.0; // additional drag per 1000 rpm [N m]
};

struct ClutchParams {
  double max_torque = 550.0;   // capacity [N m]
  double slip_speed = 30.0;    // slip speed for full capacity (smooth tanh law) [rad/s]
  double engage_rpm = 1800.0;  // automatic launch clutch fully engaged above this rpm
};

struct GearboxParams {
  int num_gears = 6;
  std::array<double, kMaxGears> ratios{3.40, 2.10, 1.52, 1.16, 0.94, 0.79, 0.70, 0.62};
  double final_drive = 3.90;
  double efficiency = 0.93;
  int automatic = 1;
  double shift_up_rpm = 6600.0;
  double shift_down_rpm = 2600.0;
  double shift_time = 0.18;  // torque interruption [s]
};

enum class DiffType : int { Open = 0, Locked = 1, ClutchLSD = 2, Viscous = 3 };
constexpr const char* kDiffChoices = "Open|Locked|Clutch LSD|Viscous";

struct DiffParams {
  int type = int(DiffType::Open);
  double preload = 40.0;          // ClutchLSD preload [N m]
  double locking = 0.3;           // ClutchLSD bias torque per input torque [-]
  double viscous = 60.0;           // Viscous coupling [N m s/rad]
  double lock_stiffness = 20000.0; // Locked: torsional stiffness between outputs [N m/rad]
  double lock_damping = 200.0;     // Locked: torsional damping [N m s/rad]
};

enum class DriveLayout : int { RWD = 0, FWD = 1, AWD = 2 };
constexpr const char* kLayoutChoices = "RWD|FWD|AWD";

struct DrivetrainParams {
  int layout = int(DriveLayout::RWD);
  double awd_front_split = 0.4;  // centre-diff torque fraction to the front axle
};

struct BrakeParams {
  double max_torque = 5600.0;      // total brake torque at full pedal, all wheels [N m]
  double front_bias = 0.68;        // fraction of torque at the front axle
  double handbrake_torque = 1600.0;// per rear wheel [N m]
  double lock_speed = 2.0;         // smoothing speed for torque sign near standstill [rad/s]
};

class Differential {
 public:
  virtual ~Differential() = default;
  virtual const char* name() const = 0;
  // Splits input torque between outputs 1 and 2 given their speeds; `split` is the static
  // torque fraction to output 1 (0.5 for axle diffs). `twist` is the integrated relative
  // angle of the outputs (a state of the vehicle), used by torsionally-compliant couplings.
  void split(double tin, double w1, double w2, double twist, double split, const DiffParams& p, double& t1,
             double& t2) const {
    const double tb = biasTorque(tin, w1 - w2, twist, p);
    t1 = split * tin - tb;
    t2 = (1.0 - split) * tin + tb;
  }
  // Torque transferred from the faster to the slower output (positive when w1 > w2).
  virtual double biasTorque(double tin, double dw, double twist, const DiffParams& p) const = 0;
};

class OpenDiff final : public Differential {
 public:
  const char* name() const override { return "Open"; }
  double biasTorque(double, double, double, const DiffParams&) const override { return 0.0; }
};
// Spool: outputs joined by a stiff torsional spring-damper (axle/driveline compliance), so the
// speed difference goes to zero in steady state without needing an implicit constraint.
class LockedDiff final : public Differential {
 public:
  const char* name() const override { return "Locked"; }
  double biasTorque(double, double dw, double twist, const DiffParams& p) const override {
    return p.lock_stiffness * twist + p.lock_damping * dw;
  }
};
class ClutchLSD final : public Differential {
 public:
  const char* name() const override { return "Clutch LSD"; }
  double biasTorque(double tin, double dw, double, const DiffParams& p) const override {
    return (p.preload + p.locking * std::abs(tin)) * std::tanh(dw / 1.0);
  }
};
class ViscousLSD final : public Differential {
 public:
  const char* name() const override { return "Viscous"; }
  double biasTorque(double, double dw, double, const DiffParams& p) const override { return p.viscous * dw; }
};

std::unique_ptr<Differential> makeDifferential(DiffType type);

double engineTorque(const EngineParams& p, double rpm, double throttle);
double gearRatio(const GearboxParams& p, int gear);  // gear 0 = neutral

struct DrivetrainInput {
  double engine_speed = 0.0;           // [rad/s]
  std::array<double, 4> wheel_speed{};  // [rad/s]
  double throttle = 0.0;
  int gear = 1;
  double engagement = 1.0;  // shift-interruption factor (0..1), multiplied with auto-clutch
  std::array<double, 3> twist{};  // relative output angle of front, rear, centre diffs [rad]
};

struct DrivetrainOutput {
  std::array<double, 4> wheel_torque{};
  double engine_accel = 0.0;
  double engine_torque = 0.0;
  double clutch_torque = 0.0;
  std::array<double, 3> twist_rate{};  // d(twist)/dt of front, rear, centre diffs
};

class Drivetrain {
 public:
  void configure(const EngineParams& e, const ClutchParams& c, const GearboxParams& g, const DrivetrainParams& d,
                 const std::array<DiffParams, 3>& diffs);
  DrivetrainOutput compute(const DrivetrainInput& in) const;
  // Speed of the gearbox output shaft implied by the wheel speeds.
  double propShaftSpeed(const std::array<double, 4>& w) const;
  bool usesTwist(int diff) const;  // whether that diff's twist state is active

 private:
  const EngineParams* e_ = nullptr;
  const ClutchParams* c_ = nullptr;
  const GearboxParams* g_ = nullptr;
  const DrivetrainParams* d_ = nullptr;
  const std::array<DiffParams, 3>* dp_ = nullptr;
  std::array<std::unique_ptr<Differential>, 3> diffs_;  // front, rear, centre
};

}  // namespace vd
