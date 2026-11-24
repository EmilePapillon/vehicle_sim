#include "vd/scenario.hpp"

#include <atomic>
#include <cstdio>
#include <fstream>
#include <limits>
#include <mutex>
#include <thread>

namespace vd {

namespace {

std::string joinPath(const std::string& base, const std::string& rel) {
  if (rel.empty() || rel[0] == '/' || base.empty()) return rel;
  return base + "/" + rel;
}

std::string dirName(const std::string& path) {
  const size_t slash = path.find_last_of('/');
  return slash == std::string::npos ? "." : path.substr(0, slash);
}

const double kNaN = std::numeric_limits<double>::quiet_NaN();

}  // namespace

bool parseScenario(const json::Value& root, const std::string& baseDir, Scenario& out, std::string* error) {
  const auto fail = [&](const std::string& m) {
    if (error) *error = m;
    return false;
  };
  if (!root.isObject()) return fail("scenario must be a JSON object");
  out.name = root.string("name", out.name);
  if (const json::Value* v = root.get("vehicle")) {
    if (!v->isString()) return fail("\"vehicle\" must be a path string");
    out.vehicleFile = joinPath(baseDir, v->asString());
  }
  if (const json::Value* p = root.get("params")) {
    if (!p->isObject()) return fail("\"params\" must be an object of key: value");
    for (const auto& kv : p->asObject()) {
      if (!kv.second.isNumber()) return fail("param '" + kv.first + "' must be a number");
      out.params.emplace_back(kv.first, kv.second.asNumber());
    }
  }
  out.duration = root.number("duration", out.duration);
  out.outputRate = root.number("output_rate", out.outputRate);
  out.initialSpeed = root.number("initial_speed", out.initialSpeed);
  if (const json::Value* m = root.get("maneuver")) out.maneuver = *m;
  else out.maneuver = json::Value::makeObject();
  if (const json::Value* sw = root.get("sweep")) {
    if (!sw->isArray()) return fail("\"sweep\" must be an array of {param, values}");
    for (const auto& ax : sw->asArray()) {
      SweepAxis axis;
      axis.param = ax.string("param", "");
      const json::Value* vals = ax.get("values");
      if (axis.param.empty() || !vals || !vals->isArray()) return fail("sweep entries need \"param\" and \"values\"");
      for (const auto& v : vals->asArray()) axis.values.push_back(v.asNumber());
      // Optional range form: {"param": ..., "values": [], "from": a, "to": b, "steps": n}
      if (axis.values.empty() && ax.get("steps")) {
        const int n = std::max(2, int(ax.number("steps", 2)));
        const double a = ax.number("from", 0), b = ax.number("to", 1);
        for (int i = 0; i < n; ++i) axis.values.push_back(a + (b - a) * i / (n - 1));
      }
      if (axis.values.empty()) return fail("sweep '" + axis.param + "' has no values");
      out.sweep.push_back(std::move(axis));
    }
  }
  if (out.duration <= 0) return fail("duration must be positive");
  return true;
}

bool loadScenarioFile(const std::string& path, Scenario& out, std::string* error) {
  std::string text;
  if (!json::readFile(path, text)) {
    if (error) *error = "cannot read " + path;
    return false;
  }
  std::string perr;
  const json::Value root = json::parse(text, &perr);
  if (!perr.empty()) {
    if (error) *error = path + ":" + perr;
    return false;
  }
  return parseScenario(root, dirName(path), out, error);
}

std::vector<ParamSet> expandSweep(const std::vector<SweepAxis>& sweep) {
  std::vector<ParamSet> sets(1);
  for (const auto& axis : sweep) {
    std::vector<ParamSet> next;
    for (const auto& base : sets) {
      for (const double v : axis.values) {
        ParamSet s = base;
        s.emplace_back(axis.param, v);
        next.push_back(std::move(s));
      }
    }
    sets = std::move(next);
  }
  return sets;
}

RunResult runScenario(const Scenario& sc, const ParamSet& extra) {
  RunResult res;
  res.overrides = extra;
  Vehicle veh;
  if (!sc.vehicleFile.empty()) {
    std::string text, err;
    if (!json::readFile(sc.vehicleFile, text)) {
      res.error = "cannot read vehicle file " + sc.vehicleFile;
      return res;
    }
    const json::Value root = json::parse(text, &err);
    if (!err.empty() || !veh.loadParams(root, nullptr, &err)) {
      res.error = sc.vehicleFile + ": " + err;
      return res;
    }
  }
  for (const auto* set : {&sc.params, &extra}) {
    for (const auto& [k, v] : *set) {
      if (!veh.setParam(k, v)) {
        res.error = "unknown parameter '" + k + "'";
        return res;
      }
    }
  }
  std::string err;
  auto driver = makeManeuver(sc.maneuver, &err);
  if (!driver) {
    res.error = err;
    return res;
  }
  veh.reset(0, 0, 0, sc.initialSpeed);
  simulate(veh, *driver, sc.duration, sc.outputRate, [&](const Telemetry& t) { res.samples.push_back(t); });
  res.metrics = computeMetrics(sc, veh, res.samples);
  res.ok = true;
  for (const auto& t : res.samples) {
    if (!std::isfinite(t.x) || !std::isfinite(t.vx) || !std::isfinite(t.roll)) {
      res.ok = false;
      res.error = "simulation diverged (non-finite state)";
      break;
    }
  }
  return res;
}

std::vector<RunResult> runBatch(const Scenario& sc, const ParamSet& cliOverrides, unsigned threads,
                                const std::function<void(size_t, size_t)>& progress) {
  const auto sets = expandSweep(sc.sweep);
  std::vector<RunResult> results(sets.size());
  if (threads == 0) threads = std::max(1u, std::thread::hardware_concurrency());
  threads = std::min<unsigned>(threads, unsigned(sets.size()));
  std::atomic<size_t> next{0}, done{0};
  std::mutex progressMutex;
  const auto worker = [&] {
    for (size_t i; (i = next.fetch_add(1)) < sets.size();) {
      ParamSet all = cliOverrides;
      all.insert(all.end(), sets[i].begin(), sets[i].end());
      results[i] = runScenario(sc, all);
      results[i].overrides = sets[i];
      const size_t d = ++done;
      if (progress) {
        std::lock_guard<std::mutex> lock(progressMutex);
        progress(d, sets.size());
      }
    }
  };
  std::vector<std::thread> pool;
  for (unsigned t = 1; t < threads; ++t) pool.emplace_back(worker);
  worker();
  for (auto& t : pool) t.join();
  return results;
}

std::map<std::string, double> computeMetrics(const Scenario& sc, const Vehicle& veh, const std::vector<Telemetry>& s) {
  std::map<std::string, double> m;
  if (s.empty()) return m;
  const double g = kGravity;
  double maxAy = 0, maxAx = 0, minAx = 0, maxRoll = 0, maxBeta = 0, maxR = 0, minFz = 1e18, maxLtr = 0;
  for (const auto& t : s) {
    maxAy = std::max(maxAy, std::abs(t.ay));
    maxAx = std::max(maxAx, t.ax);
    minAx = std::min(minAx, t.ax);
    maxRoll = std::max(maxRoll, std::abs(t.roll));
    maxBeta = std::max(maxBeta, std::abs(t.sideslip));
    maxR = std::max(maxR, std::abs(t.yaw_rate));
    double total = 0;
    for (const auto& w : t.wheel) {
      minFz = std::min(minFz, w.fz);
      total += w.fz;
    }
    // Lateral load transfer ratio (0 = even, 1 = inside wheels lifting).
    const double left = t.wheel[FL].fz + t.wheel[RL].fz, right = t.wheel[FR].fz + t.wheel[RR].fz;
    if (total > 0) maxLtr = std::max(maxLtr, std::abs(left - right) / total);
  }
  m["max_ay_g"] = maxAy / g;
  m["max_ax_g"] = maxAx / g;
  m["min_ax_g"] = minAx / g;
  m["max_roll_deg"] = rad2deg(maxRoll);
  m["max_sideslip_deg"] = rad2deg(maxBeta);
  m["max_yaw_rate_dps"] = rad2deg(maxR);
  m["min_wheel_load_N"] = minFz;
  m["max_load_transfer_ratio"] = maxLtr;
  m["final_speed"] = s.back().speed;
  m["distance"] = s.back().distance;

  // Steady state: mean over the final second.
  const double tEnd = s.back().time;
  double n = 0, r = 0, ay = 0, beta = 0, roll = 0, spd = 0;
  for (const auto& t : s) {
    if (t.time < tEnd - 1.0) continue;
    ++n;
    r += t.yaw_rate;
    ay += t.ay;
    beta += t.sideslip;
    roll += t.roll;
    spd += t.speed;
  }
  if (n > 0) {
    r /= n; ay /= n; beta /= n; roll /= n; spd /= n;
    m["ss_yaw_rate_dps"] = rad2deg(r);
    m["ss_ay_g"] = ay / g;
    m["ss_sideslip_deg"] = rad2deg(beta);
    m["ss_roll_deg"] = rad2deg(roll);
    m["ss_speed"] = spd;
    m["roll_gradient_deg_per_g"] = std::abs(ay) > 0.05 * g ? rad2deg(roll) / (ay / g) : kNaN;
    m["ss_radius"] = std::abs(r) > 1e-3 ? spd / std::abs(r) : kNaN;
  }

  const json::Value* st = sc.maneuver.get("steer");
  const json::Value* sp = sc.maneuver.get("speed");
  const std::string stype = st ? st->string("type", "") : "";
  const std::string ptype = sp ? sp->string("type", "") : "";
  const double ratio = veh.params().steering.ratio;
  const double L = veh.geometry().wheelbase;

  if (stype == "step" && n > 0) {
    const double t0 = st->number("t_start", 1.0) + 0.5 * st->number("ramp_time", 0.15);
    double rise = kNaN, peak = 0;
    for (const auto& t : s) {
      if (t.time < t0) continue;
      if (std::isnan(rise) && std::abs(t.yaw_rate) >= 0.9 * std::abs(r)) rise = t.time - t0;
      peak = std::max(peak, std::abs(t.yaw_rate));
    }
    m["yaw_rate_response_time"] = rise;
    m["yaw_rate_overshoot_pct"] = std::abs(r) > 1e-6 ? 100.0 * (peak / std::abs(r) - 1.0) : kNaN;
    const double roadWheel = deg2rad(st->number("angle_deg", 30.0)) / ratio;
    m["yaw_rate_gain"] = std::abs(roadWheel) > 1e-9 ? r / roadWheel : kNaN;
  }

  if (stype == "ramp" || stype == "circle") {
    // Understeer gradient: d(road-wheel angle)/d(ay) minus the kinematic part L/u^2, in deg/g.
    double sx = 0, sy = 0, sxx = 0, sxy = 0, sv = 0;
    int k = 0;
    for (const auto& t : s) {
      const double ayg = std::abs(t.ay) / g;
      if (ayg < 0.05 || ayg > 0.4 || t.speed < 1.0) continue;
      const double delta = std::abs(rad2deg(t.steer / ratio));
      sx += ayg; sy += delta; sxx += ayg * ayg; sxy += ayg * delta; sv += t.speed;
      ++k;
    }
    if (k > 10) {
      const double slope = (k * sxy - sx * sy) / (k * sxx - sx * sx);
      const double u = sv / k;
      // Constant radius: delta = L/R + K ay, so K is the slope directly.
      // Constant speed:  delta = (L/u^2) ay + K ay, so the kinematic part is removed.
      m["understeer_gradient_deg_per_g"] = stype == "circle" ? slope : slope - rad2deg(L * g / (u * u));
    } else {
      m["understeer_gradient_deg_per_g"] = kNaN;
    }
  }

  if (ptype == "brake") {
    const double t0 = sp->number("t_start", 0.0);
    double d0 = kNaN, v0 = kNaN, dist = kNaN, tStop = kNaN;
    for (const auto& t : s) {
      if (t.time < t0) continue;
      if (std::isnan(d0)) { d0 = t.distance; v0 = t.speed; }
      if (t.speed < 0.3) { dist = t.distance - d0; tStop = t.time - t0; break; }
    }
    m["brake_initial_speed"] = v0;
    m["stopping_distance"] = dist;
    m["stopping_time"] = tStop;
    m["mean_decel_g"] = (std::isfinite(tStop) && tStop > 0) ? (v0 - 0.3) / tStop / g : kNaN;
  }

  if (ptype == "throttle") {
    const double t0 = sp->number("t_start", 0.0);
    const auto timeTo = [&](double target) {
      for (const auto& t : s)
        if (t.time >= t0 && t.speed >= target) return t.time - t0;
      return kNaN;
    };
    m["time_0_100kmh"] = timeTo(100.0 / 3.6);
    m["time_0_60mph"] = timeTo(26.8224);
  }
  return m;
}

std::string telemetryCsvHeader() {
  std::string h =
      "t,x,y,yaw_deg,speed,vx,vy,yaw_rate_dps,ax_g,ay_g,sideslip_deg,roll_deg,pitch_deg,heave_mm,"
      "steer_deg,throttle,brake,rpm,gear,distance";
  const char* names[4] = {"fl", "fr", "rl", "rr"};
  for (const char* f : {"fz", "fx", "fy", "slip_angle_deg", "slip_ratio", "compression_mm", "omega"})
    for (const char* n : names) h += std::string(",") + f + "_" + n;
  return h;
}

std::string telemetryCsvRow(const Telemetry& t) {
  char buf[512];
  std::snprintf(buf, sizeof buf, "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.5f,%.5f,%.4f,%.4f,%.4f,%.3f,%.3f,%.4f,%.4f,%.1f,%d,%.3f",
                t.time, t.x, t.y, rad2deg(t.yaw), t.speed, t.vx, t.vy, rad2deg(t.yaw_rate), t.ax / kGravity,
                t.ay / kGravity, rad2deg(t.sideslip), rad2deg(t.roll), rad2deg(t.pitch), t.heave * 1000.0,
                rad2deg(t.steer), t.throttle, t.brake, t.engine_rpm, t.gear, t.distance);
  std::string row = buf;
  const auto add = [&](double v, const char* fmt) {
    std::snprintf(buf, sizeof buf, fmt, v);
    row += ',';
    row += buf;
  };
  for (const auto& w : t.wheel) add(w.fz, "%.1f");
  for (const auto& w : t.wheel) add(w.fx, "%.1f");
  for (const auto& w : t.wheel) add(w.fy, "%.1f");
  for (const auto& w : t.wheel) add(rad2deg(w.slip_angle), "%.4f");
  for (const auto& w : t.wheel) add(w.slip_ratio, "%.5f");
  for (const auto& w : t.wheel) add(w.compression * 1000.0, "%.3f");
  for (const auto& w : t.wheel) add(w.omega, "%.3f");
  return row;
}

bool writeTelemetryCsv(const std::string& path, const std::vector<Telemetry>& samples) {
  std::ofstream f(path);
  if (!f) return false;
  f << telemetryCsvHeader() << '\n';
  for (const auto& t : samples) f << telemetryCsvRow(t) << '\n';
  return bool(f);
}

}  // namespace vd
