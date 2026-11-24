#include "vd_capi.h"

#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>

#include "vd/driver.hpp"

struct vd_sim {
  vd::Vehicle vehicle;
  std::unique_ptr<vd::ManeuverDriver> driver;
  std::string error;
  double carry = 0.0;  // unconsumed time when a driver is active
};

namespace {

void copyTelemetry(const vd::Telemetry& t, vd_telemetry* o) {
  std::memset(o, 0, sizeof *o);
  o->time = t.time;
  o->x = t.x;
  o->y = t.y;
  o->yaw = t.yaw;
  o->heave = t.heave;
  o->roll = t.roll;
  o->pitch = t.pitch;
  o->vx = t.vx;
  o->vy = t.vy;
  o->yaw_rate = t.yaw_rate;
  o->speed = t.speed;
  o->ax = t.ax;
  o->ay = t.ay;
  o->sideslip = t.sideslip;
  o->steer = t.steer;
  o->throttle = t.throttle;
  o->brake = t.brake;
  o->engine_rpm = t.engine_rpm;
  o->engine_torque = t.engine_torque;
  o->clutch_torque = t.clutch_torque;
  o->gear = t.gear;
  o->distance = t.distance;
  for (int i = 0; i < 4; ++i) {
    const auto& w = t.wheel[size_t(i)];
    auto& ow = o->wheel[i];
    ow.fz = w.fz;
    ow.fx = w.fx;
    ow.fy = w.fy;
    ow.mz = w.mz;
    ow.slip_ratio = w.slip_ratio;
    ow.slip_angle = w.slip_angle;
    ow.omega = w.omega;
    ow.spin_angle = w.spin_angle;
    ow.steer = w.steer;
    ow.camber = w.camber;
    ow.compression = w.compression;
    ow.drive_torque = w.drive_torque;
    ow.brake_torque = w.brake_torque;
  }
}

}  // namespace

extern "C" {

int vd_api_version(void) { return VD_API_VERSION; }

vd_sim* vd_create(void) {
  try {
    return new vd_sim();
  } catch (...) {
    return nullptr;
  }
}

void vd_destroy(vd_sim* sim) { delete sim; }

const char* vd_last_error(const vd_sim* sim) { return sim ? sim->error.c_str() : "null sim"; }

int vd_load_vehicle_json(vd_sim* sim, const char* json) {
  if (!sim || !json) return -1;
  std::string err;
  const vd::json::Value root = vd::json::parse(json, &err);
  if (!err.empty()) {
    sim->error = "JSON " + err;
    return -1;
  }
  std::vector<std::string> unknown;
  if (!sim->vehicle.loadParams(root, &unknown, &err)) {
    sim->error = err;
    return -1;
  }
  sim->error.clear();
  if (!unknown.empty()) {
    sim->error = "ignored unknown keys:";
    for (const auto& k : unknown) sim->error += " " + k;
  }
  return 0;
}

char* vd_export_vehicle_json(const vd_sim* sim) {
  if (!sim) return nullptr;
  const std::string s = vd::json::dump(sim->vehicle.registry().save());
  char* out = static_cast<char*>(std::malloc(s.size() + 1));
  if (out) std::memcpy(out, s.c_str(), s.size() + 1);
  return out;
}

void vd_free_string(char* s) { std::free(s); }

int vd_param_count(const vd_sim* sim) { return sim ? int(sim->vehicle.registry().all().size()) : 0; }

int vd_param_info_at(const vd_sim* sim, int index, vd_param_info* out) {
  if (!sim || !out) return -1;
  const auto& all = sim->vehicle.registry().all();
  if (index < 0 || size_t(index) >= all.size()) return -1;
  const auto& d = all[size_t(index)];
  out->key = d.key.c_str();
  out->group = d.group.c_str();
  out->label = d.label.c_str();
  out->unit = d.unit.c_str();
  out->choices = d.choices.c_str();
  out->min = d.min;
  out->max = d.max;
  out->step = d.step;
  out->default_value = d.def;
  out->kind = int32_t(d.kind);
  out->reserved = 0;
  return 0;
}

int vd_set_param(vd_sim* sim, const char* key, double value) {
  if (!sim || !key) return -1;
  if (!sim->vehicle.setParam(key, value)) {
    sim->error = std::string("unknown parameter '") + key + "'";
    return -1;
  }
  return 0;
}

int vd_get_param(const vd_sim* sim, const char* key, double* value) {
  if (!sim || !key || !value) return -1;
  return sim->vehicle.registry().get(key, *value) ? 0 : -1;
}

void vd_reset_params(vd_sim* sim) {
  if (!sim) return;
  for (const auto& d : sim->vehicle.registry().all()) sim->vehicle.registry().set(d.key, d.def);
  sim->vehicle.params().engine.torque_curve = vd::EngineParams{}.torque_curve;
  sim->vehicle.onParamsChanged();
}

void vd_reset(vd_sim* sim, double x, double y, double yaw, double speed) {
  if (!sim) return;
  sim->vehicle.reset(x, y, yaw, speed);
  sim->carry = 0.0;
  if (sim->driver) sim->driver->reset();
}

void vd_set_inputs(vd_sim* sim, const vd_inputs* in) {
  if (!sim || !in) return;
  vd::DriverInputs d;
  d.steer = in->steer;
  d.throttle = in->throttle;
  d.brake = in->brake;
  d.handbrake = in->handbrake;
  d.gear = in->gear;
  sim->vehicle.setInputs(d);
}

int vd_set_maneuver(vd_sim* sim, const char* maneuver_json, double initial_speed) {
  if (!sim) return -1;
  if (!maneuver_json) {
    sim->driver.reset();
    sim->vehicle.setInputs({});
    return 0;
  }
  std::string err;
  const vd::json::Value spec = vd::json::parse(maneuver_json, &err);
  if (!err.empty()) {
    sim->error = "JSON " + err;
    return -1;
  }
  auto drv = vd::makeManeuver(spec, &err);
  if (!drv) {
    sim->error = err;
    return -1;
  }
  sim->driver = std::move(drv);
  vd_reset(sim, 0, 0, 0, initial_speed);
  return 0;
}

int vd_has_maneuver(const vd_sim* sim) { return sim && sim->driver ? 1 : 0; }

int vd_maneuver_path(const vd_sim* sim, double* xy, int max_points) {
  if (!sim || !sim->driver || !sim->driver->steerChannel()) return 0;
  const auto* path = sim->driver->steerChannel()->path();
  if (!path) return 0;
  const int n = std::min(int(path->size()), max_points);
  for (int i = 0; i < n && xy; ++i) {
    xy[2 * i] = (*path)[size_t(i)].first;
    xy[2 * i + 1] = (*path)[size_t(i)].second;
  }
  return n;
}

void vd_advance(vd_sim* sim, double duration) {
  if (!sim || duration <= 0) return;
  if (!sim->driver) {
    sim->vehicle.advance(duration);
    return;
  }
  // With a driver, inputs are refreshed every solver step (same as batch mode).
  const double dt = sim->vehicle.params().solver.dt;
  sim->carry += duration;
  while (sim->carry >= dt * (1.0 - 1e-9)) {
    sim->vehicle.setInputs(sim->driver->update(sim->vehicle.time(), dt, sim->vehicle));
    sim->vehicle.step(dt);
    sim->carry -= dt;
  }
}

void vd_get_telemetry(const vd_sim* sim, vd_telemetry* out) {
  if (sim && out) copyTelemetry(sim->vehicle.telemetry(), out);
}

void vd_get_geometry(const vd_sim* sim, vd_geometry* out) {
  if (!sim || !out) return;
  const vd::Geometry g = sim->vehicle.geometry();
  out->wheelbase = g.wheelbase;
  out->a = g.a;
  out->b = g.b;
  out->track_front = g.track_front;
  out->track_rear = g.track_rear;
  out->cg_height = g.cg_height;
  out->wheel_radius_front = g.wheel_radius_front;
  out->wheel_radius_rear = g.wheel_radius_rear;
}

double vd_time(const vd_sim* sim) { return sim ? sim->vehicle.time() : 0.0; }

int vd_tire_eval(const vd_sim* sim, int axle, double fz, double slip_ratio, double slip_angle, double camber,
                 double* fx, double* fy, double* mz) {
  if (!sim || axle < 0 || axle > 1) return -1;
  const vd::TireForces f = sim->vehicle.evalTire(axle, vd::TireInput{fz, slip_ratio, slip_angle, camber});
  if (fx) *fx = f.fx;
  if (fy) *fy = f.fy;
  if (mz) *mz = f.mz;
  return 0;
}

}  // extern "C"
