// Tire models. Models are stateless: they map (load, slip, camber) to forces using a shared
// parameter block, so parameters can be edited live and models swapped at runtime.
// Transient behaviour (relaxation length) is handled by the vehicle's slip states.
#pragma once

#include <memory>

#include "vd/common.hpp"

namespace vd {

enum class TireModelType : int { Linear = 0, Brush = 1, Pacejka = 2 };
constexpr const char* kTireModelChoices = "Linear|Brush|Pacejka";

struct TireParams {
  int model = int(TireModelType::Pacejka);
  double radius = 0.315;                // effective rolling radius [m]
  double mu = 1.05;                     // peak friction coefficient at nominal load
  double nominal_load = 4000.0;         // Fz0 [N]
  double load_sensitivity = -0.12;      // d(mu)/mu per unit (Fz-Fz0)/Fz0
  double cornering_stiffness = 15.0;    // C_alpha / Fz at nominal load [1/rad]
  double cs_load_peak = 2.0;            // Pacejka: load (x Fz0) where C_alpha peaks
  double longitudinal_stiffness = 20.0; // C_kappa / Fz [-]
  double shape_lat = 1.35;              // Pacejka C (lateral)
  double curvature_lat = -0.6;          // Pacejka E (lateral)
  double shape_long = 1.65;             // Pacejka C (longitudinal)
  double curvature_long = 0.1;          // Pacejka E (longitudinal)
  double camber_stiffness = 0.8;        // camber thrust / Fz per rad of camber
  double pneumatic_trail = 0.035;       // at zero slip [m]
  double relaxation_long = 0.12;        // [m]
  double relaxation_lat = 0.35;         // [m]
  double rolling_resistance = 0.012;    // coefficient [-]
};

struct TireInput {
  double fz = 0.0;     // normal load [N]
  double kappa = 0.0;  // longitudinal slip ratio [-]
  double alpha = 0.0;  // slip angle [rad], positive gives +Fy
  double gamma = 0.0;  // inclination angle [rad], positive = top leaning towards +y
};

struct TireForces {
  double fx = 0.0, fy = 0.0, mz = 0.0;  // wheel (contact patch) frame
};

// Load-dependent friction coefficient shared by all models.
double frictionCoefficient(const TireParams& p, double fz);

class TireModel {
 public:
  virtual ~TireModel() = default;
  virtual TireModelType type() const = 0;
  virtual const char* name() const = 0;
  virtual TireForces compute(const TireInput& in, const TireParams& p) const = 0;
  // Small-slip stiffnesses at the given load (used for low-speed damping and analysis).
  virtual double corneringStiffness(const TireParams& p, double fz) const;
  virtual double longitudinalStiffness(const TireParams& p, double fz) const;
};

// Linear stiffness with a friction-circle cap. Useful for validation against linear theory.
class LinearTire final : public TireModel {
 public:
  TireModelType type() const override { return TireModelType::Linear; }
  const char* name() const override { return "Linear"; }
  TireForces compute(const TireInput& in, const TireParams& p) const override;
};

// Isotropic-friction brush model with parabolic pressure distribution, natively combined slip.
class BrushTire final : public TireModel {
 public:
  TireModelType type() const override { return TireModelType::Brush; }
  const char* name() const override { return "Brush"; }
  TireForces compute(const TireInput& in, const TireParams& p) const override;
};

// Simplified Magic Formula (Pacejka) with load-dependent stiffness and friction,
// camber thrust via slip shift and combined slip via normalised slip vector.
class PacejkaTire final : public TireModel {
 public:
  TireModelType type() const override { return TireModelType::Pacejka; }
  const char* name() const override { return "Pacejka"; }
  TireForces compute(const TireInput& in, const TireParams& p) const override;
  double corneringStiffness(const TireParams& p, double fz) const override;

  double pureLateral(const TireParams& p, double fz, double alpha) const;
  double pureLongitudinal(const TireParams& p, double fz, double kappa) const;
  double peakSlipAngle(const TireParams& p, double fz) const;
  double peakSlipRatio(const TireParams& p, double fz) const;
};

std::unique_ptr<TireModel> makeTireModel(TireModelType type);

}  // namespace vd
