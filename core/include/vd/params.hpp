// Vehicle parameter set and a reflective registry over it.
//
// Every tunable value is registered once with a dotted key ("suspension.front.spring_rate"),
// metadata (group, label, unit, range) and a pointer into VehicleParams. The registry drives
// JSON load/save, CLI --set overrides, parameter sweeps, the C API and the GUI sliders, so
// adding a parameter here exposes it everywhere.
#pragma once

#include <array>
#include <string>
#include <vector>

#include "vd/drivetrain.hpp"
#include "vd/json.hpp"
#include "vd/suspension.hpp"
#include "vd/tire.hpp"

namespace vd {

struct ChassisParams {
  double mass = 1350.0;               // total vehicle mass [kg]
  double wheelbase = 2.65;            // [m]
  double weight_front = 0.56;         // static front weight fraction [-]
  double cg_height = 0.52;            // total CG height [m]
  double yaw_inertia = 2100.0;        // Izz [kg m^2]
  double roll_inertia = 520.0;        // sprung Ixx about the roll axis [kg m^2]
  double pitch_inertia = 2000.0;      // sprung Iyy [kg m^2]
  double unsprung_mass_front = 42.0;  // per corner [kg]
  double unsprung_mass_rear = 38.0;   // per corner [kg]
  double wheel_inertia = 1.1;         // spin inertia per wheel incl. brake disc [kg m^2]
  double pitch_center_height = 0.0;   // anti-dive/anti-squat equivalent [m]
};

struct SteeringParams {
  double ratio = 14.0;       // steering-wheel angle / road-wheel angle
  double ackermann = 0.4;    // 0 = parallel, 1 = ideal Ackermann
  double max_angle = 540.0;  // steering wheel lock [deg]
};

struct AeroParams {
  double air_density = 1.225;
  double drag_area = 0.68;  // Cd*A [m^2]
  double lift_area = 0.15;  // Cl*A, positive = downforce [m^2]
  double balance = 0.45;    // downforce fraction on the front axle
};

// Simple slip-based ABS and traction control acting on brake/throttle commands.
struct AidParams {
  int abs = 0;
  int tcs = 0;
  double abs_slip = 0.12;  // target |slip ratio| under braking
  double tcs_slip = 0.12;  // target slip ratio of driven wheels
};

enum class IntegratorType : int { Euler = 0, Heun = 1, RK4 = 2 };
constexpr const char* kIntegratorChoices = "Euler|Heun|RK4";

struct SolverParams {
  double dt = 0.0005;  // fixed internal step [s]
  int integrator = int(IntegratorType::RK4);
};

struct VehicleParams {
  ChassisParams chassis;
  SteeringParams steering;
  std::array<AxleSuspensionParams, 2> suspension;
  std::array<TireParams, 2> tire;
  EngineParams engine;
  ClutchParams clutch;
  GearboxParams gearbox;
  DrivetrainParams drivetrain;
  std::array<DiffParams, 3> diff;  // front, rear, centre
  BrakeParams brakes;
  AeroParams aero;
  AidParams aids;
  SolverParams solver;

  VehicleParams();  // sets rear-axle defaults that differ from the front
};

enum class ParamKind : int { Real = 0, Integer = 1, Enum = 2 };

struct ParamDesc {
  std::string key, group, label, unit;
  std::string choices;  // '|' separated labels for Enum
  double min = 0.0, max = 1.0, step = 0.0, def = 0.0;
  ParamKind kind = ParamKind::Real;
  double* dptr = nullptr;
  int* iptr = nullptr;
  bool structural = false;  // changing it rebuilds models (enum selections)

  double get() const { return dptr ? *dptr : double(*iptr); }
};

class ParamRegistry {
 public:
  explicit ParamRegistry(VehicleParams& p);
  ParamRegistry(const ParamRegistry&) = delete;
  ParamRegistry& operator=(const ParamRegistry&) = delete;

  const std::vector<ParamDesc>& all() const { return params_; }
  const ParamDesc* find(const std::string& key) const;
  // Values are clamped to [min, max]; integers and enums are rounded.
  bool set(const std::string& key, double value);
  bool get(const std::string& key, double& value) const;

  // JSON: nested objects whose leaf paths are registry keys, plus "engine.torque_curve".
  bool load(const json::Value& root, std::vector<std::string>* unknownKeys, std::string* error);
  json::Value save() const;

 private:
  VehicleParams& p_;
  std::vector<ParamDesc> params_;
};

}  // namespace vd
