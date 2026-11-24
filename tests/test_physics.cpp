// Whole-vehicle validation against closed-form theory and physical invariants.
#include <cstring>
#include <random>

#include "test.hpp"
#include "vd/scenario.hpp"

using namespace vd;

namespace {

std::vector<Telemetry> run(Vehicle& v, const char* maneuver, double duration, double speed, double rate = 100) {
  std::string err;
  auto drv = makeManeuver(json::parse(maneuver), &err);
  if (!drv) {
    vdtest::report(__FILE__, __LINE__, "maneuver: " + err);
    return {};
  }
  v.reset(0, 0, 0, speed);
  std::vector<Telemetry> out;
  simulate(v, *drv, duration, rate, [&](const Telemetry& t) { out.push_back(t); });
  return out;
}

Telemetry meanOverLast(const std::vector<Telemetry>& s, double window) {
  Telemetry m{};
  int n = 0;
  for (const auto& t : s) {
    if (t.time < s.back().time - window) continue;
    ++n;
    m.vx += t.vx; m.ay += t.ay; m.ax += t.ax; m.yaw_rate += t.yaw_rate; m.roll += t.roll; m.pitch += t.pitch;
    for (size_t i = 0; i < 4; ++i) m.wheel[i].fz += t.wheel[i].fz;
  }
  m.vx /= n; m.ay /= n; m.ax /= n; m.yaw_rate /= n; m.roll /= n; m.pitch /= n;
  for (auto& w : m.wheel) w.fz /= n;
  return m;
}

// Strips everything the linear bicycle model does not contain.
void makeLinearBicycle(Vehicle& v) {
  for (const char* a : {"front", "rear"}) {
    const std::string t = std::string("tire.") + a + ".";
    const std::string s = std::string("suspension.") + a + ".";
    v.setParam(t + "model", 0);
    v.setParam(t + "load_sensitivity", 0);
    v.setParam(t + "camber_stiffness", 0);
    v.setParam(t + "rolling_resistance", 0);
    v.setParam(t + "pneumatic_trail", 0);
    v.setParam(s + "static_camber", 0);
    v.setParam(s + "camber_roll_coeff", 0);
    v.setParam(s + "toe", 0);
  }
  v.setParam("steering.ackermann", 0);
  v.setParam("aero.drag_area", 0);
  v.setParam("aero.lift_area", 0);
}

// Sprung-mass quantities, mirroring Vehicle::computeDerived.
struct SprungInfo {
  double ms, hs, roll_arm, pitch_arm;
};
SprungInfo sprung(const Vehicle& v) {
  const auto& p = v.params();
  const auto& c = p.chassis;
  const double mu = 2 * (c.unsprung_mass_front + c.unsprung_mass_rear);
  const double ms = c.mass - mu;
  const double hs = (c.mass * c.cg_height - 2 * (c.unsprung_mass_front * p.tire[0].radius + c.unsprung_mass_rear * p.tire[1].radius)) / ms;
  const double L = c.wheelbase, b = L * c.weight_front, a = L - b;
  const double hra = p.suspension[0].roll_center_height * b / L + p.suspension[1].roll_center_height * a / L;
  return {ms, hs, hs - hra, hs - c.pitch_center_height};
}

}  // namespace

TEST(static_equilibrium) {
  Vehicle v;
  v.reset();
  v.advance(3.0);
  const auto& t = v.telemetry();
  const auto& c = v.params().chassis;
  double sum = 0;
  for (const auto& w : t.wheel) sum += w.fz;
  CHECK_REL(sum, c.mass * kGravity, 1e-4);
  CHECK_REL(t.wheel[FL].fz + t.wheel[FR].fz, c.mass * kGravity * c.weight_front, 1e-3);
  CHECK_NEAR(t.x, 0.0, 0.01);  // no creep at idle (launch clutch open)
  CHECK_NEAR(t.y, 0.0, 1e-6);
  CHECK_NEAR(t.heave, 0.0, 1e-4);
  CHECK_NEAR(t.roll, 0.0, 1e-6);
}

TEST(coasting_conserves_energy_without_losses) {
  Vehicle v;
  makeLinearBicycle(v);
  v.setParam("tire.front.rolling_resistance", 0);
  v.reset(0, 0, 0, 20.0);
  DriverInputs in;
  in.gear = -1;  // neutral
  v.setInputs(in);
  v.advance(10.0);
  CHECK_REL(v.telemetry().speed, 20.0, 1e-4);
  CHECK_NEAR(v.telemetry().y, 0.0, 1e-6);
  CHECK_REL(v.telemetry().distance, 200.0, 1e-3);
}

TEST(coastdown_matches_drag_and_rolling_resistance) {
  // Neutral gear: deceleration = (drag + rolling resistance) / m, at speed.
  Vehicle v;
  v.setParam("aero.lift_area", 0);
  v.reset(0, 0, 0, 30.0);
  DriverInputs in;
  in.gear = -1;
  v.setInputs(in);
  v.advance(2.0);
  const auto& t = v.telemetry();
  const auto& p = v.params();
  const double drag = 0.5 * p.aero.air_density * p.aero.drag_area * t.vx * t.vx;
  const double rr = p.tire[0].rolling_resistance * p.chassis.mass * kGravity;  // same coefficient front/rear
  CHECK_REL(-t.ax, (drag + rr) / p.chassis.mass, 0.03);
}

TEST(steady_state_matches_linear_bicycle_model) {
  Vehicle v;
  makeLinearBicycle(v);
  const auto& p = v.params();
  const double L = p.chassis.wheelbase, u = 20.0;
  const double csf = p.tire[0].cornering_stiffness, csr = p.tire[1].cornering_stiffness;
  const double K = (1.0 / csf - 1.0 / csr) / kGravity;  // understeer gradient [rad/(m/s^2)]
  const double sw = deg2rad(14.0);                     // 1 degree road wheel
  const auto s = run(v, R"({"steer": {"type": "constant", "angle_deg": 14}, "speed": {"type": "hold", "target": 20}})", 8.0, u);
  const Telemetry m = meanOverLast(s, 1.0);
  const double delta = sw / p.steering.ratio;
  const double expected = m.vx * delta / (L + K * m.vx * m.vx);
  CHECK_REL(m.yaw_rate, expected, 0.01);
  CHECK_REL(m.ay, m.vx * m.yaw_rate, 0.01);  // steady state: ay = u r
}

TEST(understeer_gradient_from_ramp_steer) {
  Vehicle v;
  makeLinearBicycle(v);
  Scenario sc;
  sc.duration = 8.0;
  sc.initialSpeed = 20.0;
  sc.maneuver = json::parse(R"({"steer": {"type": "ramp", "rate_deg_s": 6, "t_start": 0.5},
                                "speed": {"type": "hold", "target": 20}})");
  std::vector<Telemetry> s;
  v.reset(0, 0, 0, 20.0);
  auto drv = makeManeuver(sc.maneuver, nullptr);
  simulate(v, *drv, sc.duration, 100, [&](const Telemetry& t) { s.push_back(t); });
  const auto m = computeMetrics(sc, v, s);
  const auto& p = v.params();
  const double K = rad2deg(1.0 / p.tire[0].cornering_stiffness - 1.0 / p.tire[1].cornering_stiffness);  // deg/g
  CHECK_NEAR(m.at("understeer_gradient_deg_per_g"), K, 0.15);
}

TEST(lateral_load_transfer_balances_moments) {
  // Steady cornering: sum(y_i * dFz_i) = -(m ay h + ms g d phi) (CG moment incl. roll offset).
  Vehicle v;
  const auto s = run(v, R"({"steer": {"type": "constant", "angle_deg": 50}, "speed": {"type": "hold", "target": 20}})", 7.0, 20.0);
  const Telemetry m = meanOverLast(s, 0.5);
  const auto& p = v.params();
  const double L = p.chassis.wheelbase, b = L * p.chassis.weight_front, a = L - b;
  const double mg = p.chassis.mass * kGravity;
  const double stat[4] = {0.5 * mg * b / L, 0.5 * mg * b / L, 0.5 * mg * a / L, 0.5 * mg * a / L};
  const double y[4] = {0.5 * p.suspension[0].track, -0.5 * p.suspension[0].track, 0.5 * p.suspension[1].track,
                       -0.5 * p.suspension[1].track};
  double moment = 0;
  for (size_t i = 0; i < 4; ++i) moment += y[i] * (m.wheel[i].fz - stat[i]);
  const SprungInfo si = sprung(v);
  const double expected = -(p.chassis.mass * m.ay * p.chassis.cg_height + si.ms * kGravity * si.roll_arm * m.roll);
  CHECK(m.ay > 5.0);
  CHECK_REL(moment, expected, 0.01);
  // Roll angle from roll stiffness: phi = ms ay d / (K_phi - ms g d).
  double kphi = 0;
  for (int ax = 0; ax < 2; ++ax) {
    const auto& sp = p.suspension[size_t(ax)];
    kphi += 0.5 * sp.track * sp.track * (sp.spring_rate * sp.motion_ratio * sp.motion_ratio + 2 * sp.arb_rate);
  }
  CHECK_REL(m.roll, si.ms * m.ay * si.roll_arm / (kphi - si.ms * kGravity * si.roll_arm), 0.03);
}

TEST(longitudinal_load_transfer_under_braking) {
  Vehicle v;
  v.setParam("aero.drag_area", 0);
  v.setParam("aero.lift_area", 0);
  const auto s = run(v, R"({"speed": {"type": "brake", "value": 0.25, "t_start": 0.2}})", 2.0, 30.0);
  const Telemetry m = meanOverLast(s, 0.3);
  const auto& p = v.params();
  const double mg = p.chassis.mass * kGravity, L = p.chassis.wheelbase;
  const double front = m.wheel[FL].fz + m.wheel[FR].fz;
  const SprungInfo si = sprung(v);
  // Total pitch moment: m ax h plus the CG shift from pitch angle.
  const double expected = mg * p.chassis.weight_front - p.chassis.mass * m.ax * p.chassis.cg_height / L +
                          si.ms * kGravity * si.pitch_arm * m.pitch / L;
  CHECK(m.ax < -2.0);
  CHECK(m.pitch > 0);  // nose dives
  CHECK_REL(front, expected, 0.01);
}

TEST(braking_distance_with_and_without_abs) {
  double dist[2];
  for (int abs = 0; abs < 2; ++abs) {
    Vehicle v;
    v.setParam("aids.abs", abs);
    Scenario sc;
    sc.duration = 6;
    sc.maneuver = json::parse(R"({"speed": {"type": "brake", "value": 1.0, "t_start": 0.2}})");
    v.reset(0, 0, 0, 27.78);
    auto drv = makeManeuver(sc.maneuver, nullptr);
    std::vector<Telemetry> s;
    simulate(v, *drv, sc.duration, 200, [&](const Telemetry& t) { s.push_back(t); });
    dist[abs] = computeMetrics(sc, v, s).at("stopping_distance");
    CHECK(std::isfinite(dist[abs]));
  }
  const double mu = TireParams{}.mu;
  const double ideal = 27.78 * 27.78 / (2 * mu * kGravity);
  CHECK(dist[1] < 0.85 * dist[0]);            // ABS shortens the stop vs locked wheels
  CHECK(dist[1] > ideal && dist[1] < 1.25 * ideal);
}

TEST(acceleration_and_gear_shifts) {
  Vehicle v;
  const auto s = run(v, R"({"speed": {"type": "throttle", "value": 1.0, "t_start": 0.0}})", 16.0, 0.0);
  int gear = 1, shifts = 0;
  double maxRpm = 0, t100 = -1;
  for (const auto& t : s) {
    if (t.gear > 0) {
      CHECK(t.gear >= gear);  // never downshifts at full throttle
      shifts += t.gear > gear;
      gear = t.gear;
    }
    maxRpm = std::max(maxRpm, t.engine_rpm);
    if (t100 < 0 && t.speed >= 100 / 3.6) t100 = t.time;
  }
  CHECK(shifts >= 3);
  CHECK(maxRpm < v.params().engine.rev_limit + 150);
  CHECK(t100 > 5.0 && t100 < 11.0);
}

TEST(traction_control_limits_wheelspin) {
  double maxSlip[2];
  for (int tcs = 0; tcs < 2; ++tcs) {
    Vehicle v;
    v.setParam("aids.tcs", tcs);
    const auto s = run(v, R"({"speed": {"type": "throttle", "value": 1.0}})", 4.0, 0.0);
    maxSlip[tcs] = 0;
    for (const auto& t : s)
      if (t.time > 1.0 && t.gear == 1) maxSlip[tcs] = std::max(maxSlip[tcs], t.wheel[RL].slip_ratio);
  }
  CHECK(maxSlip[0] > 0.3);
  CHECK(maxSlip[1] < 0.25);
}

TEST(front_antiroll_bar_increases_understeer) {
  // With load-sensitive tires, more front roll stiffness shifts load transfer forward.
  double prev = 1e9;
  for (const double arb : {0.0, 15000.0, 30000.0, 50000.0}) {
    Vehicle v;
    v.setParam("suspension.front.arb_rate", arb);
    const auto s = run(v, R"({"steer": {"type": "constant", "angle_deg": 45}, "speed": {"type": "hold", "target": 22}})", 6.0, 22.0);
    const double r = meanOverLast(s, 1.0).yaw_rate;
    CHECK(r < prev);
    prev = r;
  }
}

TEST(locked_diff_equalises_wheel_speeds) {
  double dw[2];
  for (int locked = 0; locked < 2; ++locked) {
    Vehicle v;
    v.setParam("diff.rear.type", locked ? 1 : 0);
    const auto s = run(v, R"({"steer": {"type": "circle", "radius": 15}, "speed": {"type": "hold", "target": 8}})", 10.0, 8.0);
    const auto& t = s.back();
    dw[locked] = std::abs(t.wheel[RL].omega - t.wheel[RR].omega);
    if (!locked) CHECK_NEAR(t.wheel[RL].drive_torque, t.wheel[RR].drive_torque, 1e-6);
  }
  CHECK(dw[0] > 1.0);          // open diff: inner/outer wheels follow their paths
  CHECK(dw[1] < 0.02 * dw[0]);  // locked: forced together
}

TEST(constant_radius_tracking) {
  Vehicle v;
  std::string err;
  auto drv = makeManeuver(json::parse(R"({"steer": {"type": "circle", "radius": 40, "laps": 3},
                                          "speed": {"type": "hold", "target": 12}})"), &err);
  v.reset(0, 0, 0, 12);
  double maxErr = 0;
  const auto* pf = dynamic_cast<const PathFollower*>(drv->steerChannel());
  CHECK(pf != nullptr);
  simulate(v, *drv, 25.0, 50, [&](const Telemetry& t) {
    if (t.time > 8.0) maxErr = std::max(maxErr, std::abs(pf->crossTrackError()));
  });
  const auto& t = v.telemetry();
  CHECK_REL(t.speed / t.yaw_rate, 40.0, 0.03);
  CHECK(maxErr < 0.3);
}

TEST(double_lane_change_tracking) {
  Vehicle v;
  auto drv = makeManeuver(json::parse(R"({"steer": {"type": "lane_change", "offset": 3.5},
                                          "speed": {"type": "hold", "target": 20}})"), nullptr);
  const auto* pf = dynamic_cast<const PathFollower*>(drv->steerChannel());
  v.reset(0, 0, 0, 20);
  double maxErr = 0, maxY = 0;
  simulate(v, *drv, 7.0, 100, [&](const Telemetry& t) {
    maxErr = std::max(maxErr, std::abs(pf->crossTrackError()));
    maxY = std::max(maxY, t.y);
  });
  CHECK(maxErr < 0.6);
  CHECK_NEAR(maxY, 3.5, 0.4);
  CHECK_NEAR(v.telemetry().y, 0.0, 0.3);
}

TEST(rk4_converges_with_step_size) {
  double r[2];
  for (int k = 0; k < 2; ++k) {
    Vehicle v;
    v.setParam("solver.dt", k == 0 ? 0.0005 : 0.00025);
    const auto s = run(v, R"({"steer": {"type": "step", "angle_deg": 60, "t_start": 0.5}, "speed": {"type": "hold", "target": 25}})", 2.0, 25.0);
    r[k] = s.back().yaw_rate;
  }
  CHECK_REL(r[0], r[1], 1e-3);
}

TEST(deterministic_replay) {
  const char* m = R"({"steer": {"type": "chirp", "amplitude_deg": 60, "f0_hz": 0.3, "f1_hz": 2, "t_start": 0.2, "duration": 4},
                      "speed": {"type": "hold", "target": 25}})";
  Vehicle a, b;
  const auto sa = run(a, m, 4.0, 25.0), sb = run(b, m, 4.0, 25.0);
  CHECK(sa.size() == sb.size());
  CHECK(a.state() == b.state());  // bit-identical
  CHECK(sa.back().yaw_rate == sb.back().yaw_rate);
}

TEST(wheel_lift_is_handled) {
  Vehicle v;
  v.setParam("chassis.cg_height", 0.9);
  v.setParam("suspension.front.arb_rate", 80000);
  v.setParam("suspension.rear.arb_rate", 80000);
  for (const char* a : {"tire.front.mu", "tire.rear.mu"}) v.setParam(a, 1.6);
  const auto s = run(v, R"({"steer": {"type": "constant", "angle_deg": 120}, "speed": {"type": "hold", "target": 18}})", 5.0, 18.0);
  double minFz = 1e9;
  for (const auto& t : s) {
    for (const auto& w : t.wheel) {
      CHECK(w.fz >= 0.0 && std::isfinite(w.fz));
      minFz = std::min(minFz, w.fz);
    }
  }
  CHECK(minFz == 0.0);  // inside wheel(s) leave the ground
}

TEST(all_models_and_layouts_stay_finite_under_random_inputs) {
  std::mt19937 rng(1234);
  std::uniform_real_distribution<double> U(0.0, 1.0);
  for (int tire = 0; tire < 3; ++tire) {
    for (int layout = 0; layout < 3; ++layout) {
      for (int diff = 0; diff < 4; ++diff) {
        Vehicle v;
        v.setParam("tire.front.model", tire);
        v.setParam("tire.rear.model", tire);
        v.setParam("drivetrain.layout", layout);
        v.setParam("diff.rear.type", diff);
        v.setParam("diff.front.type", (diff + 1) % 4);
        v.setParam("diff.center.type", (diff + 2) % 4);
        v.setParam("suspension.front.model", diff % 2);
        v.reset(0, 0, 0, U(rng) * 30);
        DriverInputs in;
        for (int seg = 0; seg < 40; ++seg) {
          in.steer = (U(rng) * 2 - 1) * 8.0;
          in.throttle = U(rng) < 0.5 ? U(rng) : 0.0;
          in.brake = in.throttle == 0.0 && U(rng) < 0.5 ? U(rng) : 0.0;
          in.handbrake = U(rng) < 0.1 ? 1.0 : 0.0;
          v.setInputs(in);
          v.advance(0.25);
        }
        bool finite = true;
        for (const double x : v.state()) finite = finite && std::isfinite(x);
        if (!finite) {
          char buf[96];
          std::snprintf(buf, sizeof buf, "non-finite state: tire=%d layout=%d diff=%d", tire, layout, diff);
          vdtest::report(__FILE__, __LINE__, buf);
        }
      }
    }
  }
}

TEST(parameter_changes_mid_run_are_stable) {
  Vehicle v;
  v.reset(0, 0, 0, 20);
  DriverInputs in;
  in.steer = 0.6;
  in.throttle = 0.3;
  v.setInputs(in);
  v.advance(2.0);
  for (const auto& d : v.registry().all()) {
    if (d.key == "solver.dt" || d.key == "solver.integrator") continue;
    const double old = d.get();
    v.setParam(d.key, d.min + 0.7 * (d.max - d.min));
    v.advance(0.05);
    v.setParam(d.key, old);
    v.advance(0.05);
  }
  bool finite = true;
  for (const double x : v.state()) finite = finite && std::isfinite(x);
  CHECK(finite);
}

TEST(batch_runner_parallel_matches_serial) {
  Scenario sc;
  sc.duration = 3.0;
  sc.initialSpeed = 20;
  sc.maneuver = json::parse(R"({"steer": {"type": "step", "angle_deg": 30}, "speed": {"type": "hold", "target": 20}})");
  sc.sweep.push_back({"suspension.front.spring_rate", {20000, 40000, 60000, 80000}});
  const auto serial = runBatch(sc, {}, 1);
  const auto parallel = runBatch(sc, {}, 4);
  CHECK(serial.size() == 4 && parallel.size() == 4);
  for (size_t i = 0; i < serial.size(); ++i) {
    CHECK(serial[i].ok && parallel[i].ok);
    CHECK(serial[i].metrics.at("ss_yaw_rate_dps") == parallel[i].metrics.at("ss_yaw_rate_dps"));
    CHECK(serial[i].overrides[0].second == sc.sweep[0].values[i]);
  }
}
