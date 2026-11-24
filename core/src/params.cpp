#include "vd/params.hpp"

#include <cstdio>

namespace vd {

VehicleParams::VehicleParams() {
  auto& rs = suspension[Rear];
  rs.spring_rate = 36000.0;
  rs.arb_rate = 6000.0;
  rs.damper_bump = 3200.0;
  rs.damper_rebound = 5000.0;
  rs.roll_center_height = 0.11;
  rs.track = 1.54;
  rs.static_camber = -1.5;
  rs.camber_roll_coeff = 0.7;
  rs.toe = 0.15;

  auto& rt = tire[Rear];
  rt.cornering_stiffness = 18.0;
}

namespace {

class Builder {
 public:
  explicit Builder(std::vector<ParamDesc>& out) : out_(out) {}

  Builder& group(std::string g) {
    group_ = std::move(g);
    return *this;
  }
  void real(std::string key, std::string label, std::string unit, double* p, double mn, double mx, double step = 0.0) {
    ParamDesc d;
    d.key = std::move(key);
    d.group = group_;
    d.label = std::move(label);
    d.unit = std::move(unit);
    d.dptr = p;
    d.min = mn;
    d.max = mx;
    d.step = step;
    d.def = *p;
    out_.push_back(std::move(d));
  }
  void integer(std::string key, std::string label, int* p, int mn, int mx) {
    ParamDesc d;
    d.key = std::move(key);
    d.group = group_;
    d.label = std::move(label);
    d.kind = ParamKind::Integer;
    d.iptr = p;
    d.min = mn;
    d.max = mx;
    d.step = 1;
    d.def = *p;
    out_.push_back(std::move(d));
  }
  void choice(std::string key, std::string label, int* p, const char* choices) {
    ParamDesc d;
    d.key = std::move(key);
    d.group = group_;
    d.label = std::move(label);
    d.kind = ParamKind::Enum;
    d.iptr = p;
    d.choices = choices;
    int n = 1;
    for (const char* c = choices; *c; ++c) n += (*c == '|');
    d.min = 0;
    d.max = n - 1;
    d.step = 1;
    d.def = *p;
    d.structural = true;
    out_.push_back(std::move(d));
  }

 private:
  std::vector<ParamDesc>& out_;
  std::string group_;
};

const char* axleName(int a) { return a == Front ? "front" : "rear"; }
const char* axleTitle(int a) { return a == Front ? "Front" : "Rear"; }

}  // namespace

ParamRegistry::ParamRegistry(VehicleParams& p) : p_(p) {
  Builder b(params_);

  auto& c = p.chassis;
  b.group("Chassis");
  b.real("chassis.mass", "Mass", "kg", &c.mass, 500, 4000, 10);
  b.real("chassis.wheelbase", "Wheelbase", "m", &c.wheelbase, 1.8, 4.0, 0.01);
  b.real("chassis.weight_front", "Front weight", "-", &c.weight_front, 0.3, 0.7, 0.005);
  b.real("chassis.cg_height", "CG height", "m", &c.cg_height, 0.2, 1.2, 0.005);
  b.real("chassis.yaw_inertia", "Yaw inertia", "kg m²", &c.yaw_inertia, 300, 8000, 10);
  b.real("chassis.roll_inertia", "Roll inertia", "kg m²", &c.roll_inertia, 100, 3000, 10);
  b.real("chassis.pitch_inertia", "Pitch inertia", "kg m²", &c.pitch_inertia, 300, 8000, 10);
  b.real("chassis.unsprung_mass_front", "Unsprung mass F", "kg", &c.unsprung_mass_front, 10, 120, 1);
  b.real("chassis.unsprung_mass_rear", "Unsprung mass R", "kg", &c.unsprung_mass_rear, 10, 120, 1);
  b.real("chassis.wheel_inertia", "Wheel inertia", "kg m²", &c.wheel_inertia, 0.3, 5, 0.05);
  b.real("chassis.pitch_center_height", "Pitch centre height", "m", &c.pitch_center_height, -0.2, 0.5, 0.01);

  auto& s = p.steering;
  b.group("Steering");
  b.real("steering.ratio", "Steering ratio", "-", &s.ratio, 5, 30, 0.1);
  b.real("steering.ackermann", "Ackermann", "-", &s.ackermann, -0.5, 1.2, 0.01);
  b.real("steering.max_angle", "Lock", "deg", &s.max_angle, 90, 1080, 5);

  for (int a = 0; a < 2; ++a) {
    auto& su = p.suspension[size_t(a)];
    const std::string k = std::string("suspension.") + axleName(a) + ".";
    b.group(std::string("Suspension ") + axleTitle(a));
    b.choice(k + "model", "Spring model", &su.model, kSuspensionModelChoices);
    b.real(k + "spring_rate", "Spring rate", "N/m", &su.spring_rate, 5000, 200000, 500);
    b.real(k + "motion_ratio", "Motion ratio", "-", &su.motion_ratio, 0.4, 1.2, 0.01);
    b.real(k + "progression", "Progression", "1/m", &su.progression, 0, 20, 0.1);
    b.real(k + "arb_rate", "Anti-roll bar", "N/m", &su.arb_rate, 0, 100000, 500);
    b.real(k + "damper_bump", "Bump damping", "N s/m", &su.damper_bump, 200, 20000, 50);
    b.real(k + "damper_rebound", "Rebound damping", "N s/m", &su.damper_rebound, 200, 20000, 50);
    b.real(k + "damper_knee", "Damper knee", "m/s", &su.damper_knee, 0.01, 0.5, 0.01);
    b.real(k + "damper_high_speed_ratio", "High-speed ratio", "-", &su.damper_high_speed_ratio, 0.05, 1.5, 0.01);
    b.real(k + "bump_stop_gap", "Bump stop gap", "m", &su.bump_stop_gap, 0.005, 0.2, 0.001);
    b.real(k + "bump_stop_rate", "Bump stop rate", "N/m", &su.bump_stop_rate, 10000, 2000000, 5000);
    b.real(k + "roll_center_height", "Roll centre height", "m", &su.roll_center_height, -0.1, 0.4, 0.005);
    b.real(k + "track", "Track", "m", &su.track, 1.0, 2.2, 0.01);
    b.real(k + "static_camber", "Static camber", "deg", &su.static_camber, -6, 3, 0.1);
    b.real(k + "camber_roll_coeff", "Camber/roll", "-", &su.camber_roll_coeff, -0.5, 1.5, 0.01);
    b.real(k + "toe", "Toe-in", "deg", &su.toe, -2, 2, 0.01);
  }

  for (int a = 0; a < 2; ++a) {
    auto& t = p.tire[size_t(a)];
    const std::string k = std::string("tire.") + axleName(a) + ".";
    b.group(std::string("Tires ") + axleTitle(a));
    b.choice(k + "model", "Tire model", &t.model, kTireModelChoices);
    b.real(k + "radius", "Radius", "m", &t.radius, 0.2, 0.5, 0.001);
    b.real(k + "mu", "Friction μ", "-", &t.mu, 0.1, 2.0, 0.01);
    b.real(k + "nominal_load", "Nominal load", "N", &t.nominal_load, 1000, 10000, 50);
    b.real(k + "load_sensitivity", "Load sensitivity", "-", &t.load_sensitivity, -0.5, 0.2, 0.01);
    b.real(k + "cornering_stiffness", "Cornering stiffness", "1/rad", &t.cornering_stiffness, 4, 40, 0.1);
    b.real(k + "cs_load_peak", "Stiffness load peak", "x Fz0", &t.cs_load_peak, 0.5, 5, 0.05);
    b.real(k + "longitudinal_stiffness", "Longitudinal stiffness", "-", &t.longitudinal_stiffness, 4, 50, 0.1);
    b.real(k + "shape_lat", "Shape C (lat)", "-", &t.shape_lat, 1.0, 2.0, 0.01);
    b.real(k + "curvature_lat", "Curvature E (lat)", "-", &t.curvature_lat, -3, 1, 0.01);
    b.real(k + "shape_long", "Shape C (long)", "-", &t.shape_long, 1.0, 2.0, 0.01);
    b.real(k + "curvature_long", "Curvature E (long)", "-", &t.curvature_long, -3, 1, 0.01);
    b.real(k + "camber_stiffness", "Camber stiffness", "1/rad", &t.camber_stiffness, 0, 3, 0.01);
    b.real(k + "pneumatic_trail", "Pneumatic trail", "m", &t.pneumatic_trail, 0, 0.1, 0.001);
    b.real(k + "relaxation_long", "Relaxation (long)", "m", &t.relaxation_long, 0.02, 1.0, 0.01);
    b.real(k + "relaxation_lat", "Relaxation (lat)", "m", &t.relaxation_lat, 0.05, 2.0, 0.01);
    b.real(k + "rolling_resistance", "Rolling resistance", "-", &t.rolling_resistance, 0, 0.05, 0.001);
  }

  auto& e = p.engine;
  b.group("Engine");
  b.real("engine.torque_scale", "Torque scale", "-", &e.torque_scale, 0.1, 4.0, 0.01);
  b.real("engine.idle_rpm", "Idle", "rpm", &e.idle_rpm, 500, 1500, 10);
  b.real("engine.rev_limit", "Rev limit", "rpm", &e.rev_limit, 3000, 12000, 50);
  b.real("engine.inertia", "Inertia", "kg m²", &e.inertia, 0.05, 1.0, 0.01);
  b.real("engine.friction_base", "Friction base", "N m", &e.friction_base, 0, 60, 1);
  b.real("engine.friction_per_krpm", "Friction /krpm", "N m", &e.friction_per_krpm, 0, 20, 0.5);

  auto& cl = p.clutch;
  b.real("clutch.max_torque", "Clutch capacity", "N m", &cl.max_torque, 100, 3000, 10);
  b.real("clutch.slip_speed", "Clutch slip speed", "rad/s", &cl.slip_speed, 5, 100, 1);
  b.real("clutch.engage_rpm", "Clutch engage", "rpm", &cl.engage_rpm, 1000, 5000, 50);

  auto& g = p.gearbox;
  b.group("Gearbox");
  b.integer("gearbox.num_gears", "Gears", &g.num_gears, 1, kMaxGears);
  for (int i = 0; i < kMaxGears; ++i) {
    b.real("gearbox.ratio_" + std::to_string(i + 1), "Gear " + std::to_string(i + 1), "-",
           &g.ratios[size_t(i)], 0.3, 6.0, 0.01);
  }
  b.real("gearbox.final_drive", "Final drive", "-", &g.final_drive, 1.5, 8.0, 0.01);
  b.real("gearbox.efficiency", "Efficiency", "-", &g.efficiency, 0.5, 1.0, 0.01);
  b.integer("gearbox.automatic", "Automatic", &g.automatic, 0, 1);
  b.real("gearbox.shift_up_rpm", "Upshift", "rpm", &g.shift_up_rpm, 2000, 12000, 50);
  b.real("gearbox.shift_down_rpm", "Downshift", "rpm", &g.shift_down_rpm, 1000, 8000, 50);
  b.real("gearbox.shift_time", "Shift time", "s", &g.shift_time, 0.0, 1.0, 0.01);

  auto& d = p.drivetrain;
  b.group("Drivetrain");
  b.choice("drivetrain.layout", "Layout", &d.layout, kLayoutChoices);
  b.real("drivetrain.awd_front_split", "AWD front split", "-", &d.awd_front_split, 0, 1, 0.01);
  const char* diffNames[3] = {"front", "rear", "center"};
  const char* diffTitles[3] = {"Front diff", "Rear diff", "Centre diff"};
  for (size_t i = 0; i < 3; ++i) {
    auto& df = p.diff[i];
    const std::string k = std::string("diff.") + diffNames[i] + ".";
    const std::string t = diffTitles[i];
    b.choice(k + "type", t, &df.type, kDiffChoices);
    b.real(k + "preload", t + " preload", "N m", &df.preload, 0, 500, 5);
    b.real(k + "locking", t + " locking", "-", &df.locking, 0, 1, 0.01);
    b.real(k + "viscous", t + " viscous", "N m s", &df.viscous, 0, 500, 5);
    b.real(k + "lock_stiffness", t + " lock stiffness", "N m/rad", &df.lock_stiffness, 1000, 60000, 500);
    b.real(k + "lock_damping", t + " lock damping", "N m s", &df.lock_damping, 0, 1000, 10);
  }

  auto& br = p.brakes;
  b.group("Brakes");
  b.real("brakes.max_torque", "Max torque", "N m", &br.max_torque, 500, 20000, 50);
  b.real("brakes.front_bias", "Front bias", "-", &br.front_bias, 0.3, 0.9, 0.01);
  b.real("brakes.handbrake_torque", "Handbrake", "N m", &br.handbrake_torque, 0, 5000, 50);
  b.real("brakes.lock_speed", "Lock smoothing", "rad/s", &br.lock_speed, 0.5, 10, 0.1);

  auto& ae = p.aero;
  b.group("Aero");
  b.real("aero.air_density", "Air density", "kg/m³", &ae.air_density, 0.5, 1.5, 0.005);
  b.real("aero.drag_area", "Drag area CdA", "m²", &ae.drag_area, 0, 3, 0.01);
  b.real("aero.lift_area", "Downforce ClA", "m²", &ae.lift_area, -1, 6, 0.01);
  b.real("aero.balance", "Aero balance F", "-", &ae.balance, 0, 1, 0.01);

  auto& ai = p.aids;
  b.group("Driver aids");
  b.integer("aids.abs", "ABS", &ai.abs, 0, 1);
  b.integer("aids.tcs", "Traction control", &ai.tcs, 0, 1);
  b.real("aids.abs_slip", "ABS target slip", "-", &ai.abs_slip, 0.03, 0.4, 0.01);
  b.real("aids.tcs_slip", "TCS target slip", "-", &ai.tcs_slip, 0.03, 0.4, 0.01);

  auto& so = p.solver;
  b.group("Solver");
  b.real("solver.dt", "Time step", "s", &so.dt, 0.0001, 0.002, 0.0001);
  b.choice("solver.integrator", "Integrator", &so.integrator, kIntegratorChoices);
}

const ParamDesc* ParamRegistry::find(const std::string& key) const {
  for (const auto& d : params_)
    if (d.key == key) return &d;
  return nullptr;
}

bool ParamRegistry::set(const std::string& key, double value) {
  const ParamDesc* d = find(key);
  if (!d || !std::isfinite(value)) return false;
  const double v = clamp(value, d->min, d->max);
  if (d->dptr) *d->dptr = v;
  else *d->iptr = int(std::lround(v));
  return true;
}

bool ParamRegistry::get(const std::string& key, double& value) const {
  const ParamDesc* d = find(key);
  if (!d) return false;
  value = d->get();
  return true;
}

namespace {

void flatten(const json::Value& v, const std::string& prefix, std::vector<std::pair<std::string, const json::Value*>>& out) {
  if (v.isObject()) {
    for (const auto& kv : v.asObject()) flatten(kv.second, prefix.empty() ? kv.first : prefix + "." + kv.first, out);
  } else {
    out.emplace_back(prefix, &v);
  }
}

}  // namespace

bool ParamRegistry::load(const json::Value& root, std::vector<std::string>* unknownKeys, std::string* error) {
  if (!root.isObject()) {
    if (error) *error = "vehicle definition must be a JSON object";
    return false;
  }
  std::vector<std::pair<std::string, const json::Value*>> leaves;
  flatten(root, "", leaves);
  for (const auto& [key, val] : leaves) {
    if (key == "name" || key == "description") continue;
    if (key == "engine.torque_curve") {
      std::vector<double> xs, ys;
      if (!val->isArray()) {
        if (error) *error = "engine.torque_curve must be an array of [rpm, torque] pairs";
        return false;
      }
      for (const auto& pt : val->asArray()) {
        if (!pt.isArray() || pt.asArray().size() != 2) {
          if (error) *error = "engine.torque_curve entries must be [rpm, torque]";
          return false;
        }
        xs.push_back(pt.asArray()[0].asNumber());
        ys.push_back(pt.asArray()[1].asNumber());
      }
      for (size_t i = 1; i < xs.size(); ++i) {
        if (xs[i] <= xs[i - 1]) {
          if (error) *error = "engine.torque_curve rpm values must be ascending";
          return false;
        }
      }
      if (xs.size() < 2) {
        if (error) *error = "engine.torque_curve needs at least two points";
        return false;
      }
      p_.engine.torque_curve = Table1D(std::move(xs), std::move(ys));
      continue;
    }
    if (!val->isNumber() && !val->isBool()) {
      if (unknownKeys) unknownKeys->push_back(key);
      continue;
    }
    if (!set(key, val->asNumber()) && unknownKeys) unknownKeys->push_back(key);
  }
  return true;
}

json::Value ParamRegistry::save() const {
  json::Value root = json::Value::makeObject();
  for (const auto& d : params_) {
    json::Value* node = &root;
    size_t start = 0;
    for (;;) {
      const size_t dot = d.key.find('.', start);
      if (dot == std::string::npos) {
        (*node)[d.key.substr(start)] = json::Value(d.get());
        break;
      }
      node = &(*node)[d.key.substr(start, dot - start)];
      start = dot + 1;
    }
  }
  json::Value curve = json::Value::makeArray();
  const auto& tc = p_.engine.torque_curve;
  for (size_t i = 0; i < tc.size(); ++i) {
    json::Value pt = json::Value::makeArray();
    pt.push(tc.xs()[i]);
    pt.push(tc.ys()[i]);
    curve.push(std::move(pt));
  }
  root["engine"]["torque_curve"] = std::move(curve);
  return root;
}

}  // namespace vd
