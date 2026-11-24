// Unit tests for individual models, the parameter registry, JSON and the C API.
#include <cstdlib>
#include <cstring>
#include <set>

#include "test.hpp"
#include "vd/scenario.hpp"
#include "vd_capi.h"

using namespace vd;

// ---------------------------------------------------------------- JSON

TEST(json_roundtrip) {
  const std::string text = R"({"a": 1.5, "b": [1, 2, {"c": "x\"yé"}], "d": true, "e": null, // comment
    "nested": {"k": -3e2}})";
  std::string err;
  const auto v = json::parse(text, &err);
  CHECK(err.empty());
  CHECK_NEAR(v.number("a", 0), 1.5, 0);
  CHECK(v.get("b")->asArray().size() == 3);
  CHECK(v.get("b")->asArray()[2].string("c", "") == "x\"y\xc3\xa9");
  CHECK(v.get("d")->asBool());
  CHECK(v.get("e")->isNull());
  CHECK_NEAR(v.get("nested")->number("k", 0), -300, 0);
  const auto again = json::parse(json::dump(v), &err);
  CHECK(err.empty());
  CHECK(json::dump(again) == json::dump(v));
}

TEST(json_errors_report_position) {
  std::string err;
  const auto v = json::parse("{\n  \"a\": 1,\n  \"b\": }", &err);
  CHECK(v.isNull());
  CHECK(err.rfind("3:", 0) == 0);  // line 3
  json::parse("[1, 2", &err);
  CHECK(!err.empty());
  json::parse("{\"a\": 1} trailing", &err);
  CHECK(!err.empty());
}

// ---------------------------------------------------------------- Tires

namespace {
TireParams tireDefaults() { return TireParams{}; }
}  // namespace

TEST(tire_linear_small_slip) {
  const TireParams p = tireDefaults();
  LinearTire t;
  const double fz = 4000, a = deg2rad(0.5);
  const auto f = t.compute({fz, 0.0, a, 0.0}, p);
  CHECK_REL(f.fy, p.cornering_stiffness * fz * a, 1e-9);
  const auto fx = t.compute({fz, 0.01, 0.0, 0.0}, p);
  CHECK_REL(fx.fx, p.longitudinal_stiffness * fz * 0.01, 1e-9);
  CHECK(f.mz < 0);  // aligning moment opposes Fy
}

TEST(tire_pacejka_stiffness_and_peak) {
  TireParams p = tireDefaults();
  PacejkaTire t;
  const double fz0 = p.nominal_load;
  // Small-slip stiffness at nominal load matches the parameter.
  const double h = 1e-5;
  const double ky = (t.pureLateral(p, fz0, h) - t.pureLateral(p, fz0, -h)) / (2 * h);
  CHECK_REL(ky, p.cornering_stiffness * fz0, 1e-4);
  const double kx = (t.pureLongitudinal(p, fz0, h) - t.pureLongitudinal(p, fz0, -h)) / (2 * h);
  CHECK_REL(kx, p.longitudinal_stiffness * fz0, 1e-4);
  // The peak equals mu(Fz) * Fz and is located at peakSlipAngle.
  for (const double fz : {2000.0, 4000.0, 7000.0}) {
    double best = 0, bestA = 0;
    for (double a = 0; a < 0.5; a += 1e-4) {
      const double f = t.pureLateral(p, fz, a);
      if (f > best) { best = f; bestA = a; }
    }
    CHECK_REL(best, frictionCoefficient(p, fz) * fz, 1e-3);
    CHECK_NEAR(bestA, t.peakSlipAngle(p, fz), 2e-3);
  }
  // Load sensitivity: friction coefficient falls with load.
  CHECK(frictionCoefficient(p, 6000) < frictionCoefficient(p, 3000));
}

TEST(tire_brush_small_slip_and_saturation) {
  const TireParams p = tireDefaults();
  BrushTire t;
  const double fz = 4000;
  const auto f = t.compute({fz, 0.0, deg2rad(0.1), 0.0}, p);
  CHECK_REL(f.fy, p.cornering_stiffness * fz * std::tan(deg2rad(0.1)), 0.015);
  const auto sat = t.compute({fz, 0.0, deg2rad(30), 0.0}, p);
  CHECK_REL(sat.fy, frictionCoefficient(p, fz) * fz, 1e-6);
  CHECK_NEAR(sat.mz, 0.0, 1e-9);  // fully sliding: no aligning moment
}

TEST(tire_models_respect_friction_circle_and_symmetry) {
  const TireParams p = tireDefaults();
  for (int m = 0; m < 3; ++m) {
    auto t = makeTireModel(TireModelType(m));
    for (const double fz : {500.0, 4000.0, 9000.0}) {
      const double fmax = frictionCoefficient(p, fz) * fz;
      for (double k = -1.0; k <= 1.5; k += 0.05) {
        for (double a = -0.6; a <= 0.6; a += 0.02) {
          const auto f = t->compute({fz, k, a, 0.0}, p);
          if (std::hypot(f.fx, f.fy) > fmax * (1 + 1e-6)) {
            vdtest::report(__FILE__, __LINE__, std::string(t->name()) + " exceeds friction circle");
            return;
          }
          const auto g = t->compute({fz, k, -a, 0.0}, p);
          if (std::abs(f.fy + g.fy) > 1e-6 * fmax || std::abs(f.fx - g.fx) > 1e-6 * fmax) {
            vdtest::report(__FILE__, __LINE__, std::string(t->name()) + " not symmetric in alpha");
            return;
          }
        }
      }
    }
  }
}

TEST(tire_combined_slip_reduces_lateral_force) {
  const TireParams p = tireDefaults();
  for (int m = 1; m < 3; ++m) {  // Brush and Pacejka (linear is capped only)
    auto t = makeTireModel(TireModelType(m));
    const double pure = t->compute({4000, 0.0, deg2rad(4), 0.0}, p).fy;
    const double driven = t->compute({4000, 0.1, deg2rad(4), 0.0}, p).fy;
    const double braked = t->compute({4000, -0.1, deg2rad(4), 0.0}, p).fy;
    CHECK(driven < 0.8 * pure);
    CHECK(braked < 0.8 * pure);
  }
}

TEST(tire_camber_thrust_direction) {
  const TireParams p = tireDefaults();
  for (int m = 0; m < 3; ++m) {
    auto t = makeTireModel(TireModelType(m));
    const auto f = t->compute({4000, 0.0, 0.0, deg2rad(2)}, p);
    CHECK(f.fy > 0);  // leaning towards +y pushes towards +y
    CHECK_REL(f.fy, p.camber_stiffness * 4000 * deg2rad(2), 0.1);
  }
}

// ---------------------------------------------------------------- Suspension

TEST(suspension_spring_damper_bumpstop) {
  AxleSuspensionParams p;
  LinearSuspension lin;
  ProgressiveSuspension prog;
  CHECK_REL(lin.springForce(0.02, p), p.spring_rate * 0.02, 1e-12);
  p.motion_ratio = 0.8;
  CHECK_REL(lin.springForce(0.02, p), p.spring_rate * 0.64 * 0.02, 1e-12);
  p.motion_ratio = 1.0;
  // Bump stop adds force beyond the gap.
  CHECK(lin.springForce(p.bump_stop_gap + 0.01, p) > p.spring_rate * (p.bump_stop_gap + 0.01) + 1000);
  // Progressive spring is stiffer in compression and softer in droop.
  CHECK(prog.springForce(0.04, p) > lin.springForce(0.04, p));
  CHECK(prog.springForce(-0.04, p) > lin.springForce(-0.04, p));
  // Damper: asymmetric bump/rebound, digressive above the knee.
  CHECK_REL(lin.damperForce(0.05, p), p.damper_bump * 0.05, 1e-12);
  CHECK_REL(lin.damperForce(-0.05, p), -p.damper_rebound * 0.05, 1e-12);
  const double hi = lin.damperForce(0.5, p);
  CHECK_REL(hi, p.damper_bump * (p.damper_knee + p.damper_high_speed_ratio * (0.5 - p.damper_knee)), 1e-12);
}

// ---------------------------------------------------------------- Drivetrain

TEST(differentials_split_torque) {
  DiffParams p;
  double t1, t2;
  OpenDiff open;
  open.split(1000, 10, 20, 0.0, 0.5, p, t1, t2);
  CHECK_NEAR(t1, 500, 1e-9);
  CHECK_NEAR(t2, 500, 1e-9);
  ClutchLSD lsd;
  lsd.split(1000, 30, 10, 0.0, 0.5, p, t1, t2);  // output 1 spinning faster loses torque
  CHECK(t1 < t2);
  CHECK(t2 - t1 <= 2 * (p.preload + p.locking * 1000) + 1e-9);
  CHECK_NEAR(t1 + t2, 1000, 1e-9);  // torque conserved
  LockedDiff locked;
  locked.split(0, 10.5, 10, 0.01, 0.5, p, t1, t2);
  CHECK_NEAR(t2 - t1, 2 * (p.lock_stiffness * 0.01 + p.lock_damping * 0.5), 1e-9);
  ViscousLSD visc;
  visc.split(400, 12, 10, 0.0, 0.4, p, t1, t2);
  CHECK_NEAR(t1, 0.4 * 400 - p.viscous * 2, 1e-9);
}

TEST(engine_governor_and_limiter) {
  EngineParams e;
  CHECK(engineTorque(e, 600, 0.0) > 0);           // idle governor below idle
  CHECK(engineTorque(e, 3000, 0.0) < 0);          // engine braking
  CHECK(engineTorque(e, e.rev_limit + 10, 1.0) < 0);  // fuel cut
  CHECK_REL(engineTorque(e, 4500, 1.0), 275.0, 1e-9);
  GearboxParams g;
  CHECK_NEAR(gearRatio(g, 0), 0.0, 0);
  CHECK_NEAR(gearRatio(g, 1), g.ratios[0], 0);
  CHECK_NEAR(gearRatio(g, 7), 0.0, 0);  // beyond num_gears
}

// ---------------------------------------------------------------- Registry

TEST(registry_keys_unique_and_clamped) {
  Vehicle v;
  std::set<std::string> keys;
  for (const auto& d : v.registry().all()) {
    CHECK(keys.insert(d.key).second);
    CHECK(d.min <= d.def && d.def <= d.max);
    CHECK(!d.group.empty() && !d.label.empty());
  }
  CHECK(keys.size() > 100);
  CHECK(v.setParam("suspension.front.spring_rate", 1e9));
  double val = 0;
  CHECK(v.registry().get("suspension.front.spring_rate", val));
  CHECK_NEAR(val, 200000, 0);  // clamped to max
  CHECK(!v.setParam("does.not.exist", 1));
  CHECK(v.setParam("tire.front.model", 0.6));
  CHECK(v.params().tire[0].model == 1);  // enums are rounded
  CHECK(v.tireModel(Front).type() == TireModelType::Brush);  // and rebuild models
}

TEST(registry_json_roundtrip) {
  Vehicle a;
  a.setParam("chassis.mass", 1777);
  a.setParam("diff.rear.type", 2);
  a.setParam("suspension.rear.arb_rate", 4321);
  const std::string text = json::dump(a.registry().save());
  Vehicle b;
  std::string err;
  std::vector<std::string> unknown;
  CHECK(b.loadParams(json::parse(text), &unknown, &err));
  CHECK(unknown.empty());
  for (const auto& d : a.registry().all()) {
    double vb = 0;
    b.registry().get(d.key, vb);
    CHECK_NEAR(vb, d.get(), 1e-6 * std::max(1.0, std::abs(d.get())));
  }
  CHECK(b.params().engine.torque_curve.size() == a.params().engine.torque_curve.size());
  // Unknown keys are reported, not fatal.
  unknown.clear();
  CHECK(b.loadParams(json::parse(R"({"chassis": {"mass": 1200, "colour": 3}})"), &unknown, &err));
  CHECK(unknown.size() == 1 && unknown[0] == "chassis.colour");
}

TEST(scenario_parsing_and_sweep_expansion) {
  Scenario sc;
  std::string err;
  const auto root = json::parse(R"({"name": "x", "duration": 2, "params": {"chassis.mass": 1500},
      "sweep": [{"param": "a", "values": [1, 2, 3]}, {"param": "b", "from": 0, "to": 1, "steps": 2, "values": []}]})");
  CHECK(parseScenario(root, ".", sc, &err));
  const auto sets = expandSweep(sc.sweep);
  CHECK(sets.size() == 6);
  CHECK(sets[5].size() == 2 && sets[5][0].second == 3 && sets[5][1].second == 1);
  Scenario bad;
  CHECK(!parseScenario(json::parse(R"({"duration": -1})"), ".", bad, &err));
  CHECK(!makeManeuver(json::parse(R"({"steer": {"type": "nope"}})"), &err));
  CHECK(err.find("nope") != std::string::npos);
}

// ---------------------------------------------------------------- C API

TEST(capi_lifecycle) {
  CHECK(vd_api_version() == VD_API_VERSION);
  vd_sim* sim = vd_create();
  CHECK(sim != nullptr);
  const int n = vd_param_count(sim);
  CHECK(n > 100);
  vd_param_info info{};
  CHECK(vd_param_info_at(sim, 0, &info) == 0);
  CHECK(std::strcmp(info.key, "chassis.mass") == 0);
  CHECK(vd_param_info_at(sim, n, &info) != 0);
  CHECK(vd_set_param(sim, "suspension.front.arb_rate", 20000) == 0);
  double v = 0;
  CHECK(vd_get_param(sim, "suspension.front.arb_rate", &v) == 0 && v == 20000);
  CHECK(vd_set_param(sim, "bogus", 1) != 0);
  CHECK(std::strstr(vd_last_error(sim), "bogus") != nullptr);

  // Export -> import into a second instance.
  char* js = vd_export_vehicle_json(sim);
  CHECK(js != nullptr);
  vd_sim* other = vd_create();
  CHECK(vd_load_vehicle_json(other, js) == 0);
  CHECK(vd_get_param(other, "suspension.front.arb_rate", &v) == 0 && v == 20000);
  vd_free_string(js);
  CHECK(vd_load_vehicle_json(other, "{ broken") != 0);
  vd_reset_params(other);
  CHECK(vd_get_param(other, "suspension.front.arb_rate", &v) == 0 && v == 12000);
  vd_destroy(other);

  // Manual driving.
  vd_reset(sim, 0, 0, 0, 15);
  vd_inputs in{};
  in.steer = 0.5;
  in.throttle = 0.3;
  vd_set_inputs(sim, &in);
  vd_advance(sim, 2.0);
  vd_telemetry t{};
  vd_get_telemetry(sim, &t);
  CHECK_NEAR(t.time, 2.0, 1e-9);
  CHECK(t.yaw_rate > 0.1 && std::isfinite(t.x));
  CHECK(t.wheel[FR].fz > t.wheel[FL].fz);  // left turn loads the right side

  // Scripted maneuver with a reference path.
  CHECK(vd_set_maneuver(sim, R"({"steer": {"type": "circle", "radius": 30}, "speed": {"type": "hold", "target": 10}})", 10) == 0);
  CHECK(vd_has_maneuver(sim));
  CHECK(vd_maneuver_path(sim, nullptr, 1 << 20) > 100);
  vd_advance(sim, 5.0);
  vd_get_telemetry(sim, &t);
  CHECK(t.yaw_rate > 0.2);
  CHECK(vd_set_maneuver(sim, R"({"steer": {"type": "bad"}})", 0) != 0);
  CHECK(vd_set_maneuver(sim, nullptr, 0) == 0 && !vd_has_maneuver(sim));

  double fx, fy, mz;
  CHECK(vd_tire_eval(sim, 0, 4000, 0, 0.05, 0, &fx, &fy, &mz) == 0 && fy > 1000);
  CHECK(vd_tire_eval(sim, 2, 4000, 0, 0.05, 0, &fx, &fy, &mz) != 0);
  vd_geometry g{};
  vd_get_geometry(sim, &g);
  CHECK_NEAR(g.a + g.b, g.wheelbase, 1e-12);
  vd_destroy(sim);
}
