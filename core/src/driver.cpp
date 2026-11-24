#include "vd/driver.hpp"

namespace vd {

// ---------------------------------------------------------------- open-loop steering

namespace {

class NoSteer final : public SteerChannel {
 public:
  double steer(double, double, const Vehicle&) override { return 0.0; }
};

class ConstantSteer final : public SteerChannel {
 public:
  explicit ConstantSteer(double a) : a_(a) {}
  double steer(double, double, const Vehicle&) override { return a_; }

 private:
  double a_;
};

// Step with a linear ramp of duration rampTime starting at tStart.
class StepSteer final : public SteerChannel {
 public:
  StepSteer(double a, double t0, double ramp) : a_(a), t0_(t0), ramp_(std::max(ramp, 1e-3)) {}
  double steer(double t, double, const Vehicle&) override { return a_ * clamp((t - t0_) / ramp_, 0.0, 1.0); }

 private:
  double a_, t0_, ramp_;
};

class RampSteer final : public SteerChannel {
 public:
  RampSteer(double rate, double t0, double maxA) : rate_(rate), t0_(t0), max_(maxA) {}
  double steer(double t, double, const Vehicle&) override {
    return clamp(rate_ * std::max(t - t0_, 0.0), -max_, max_);
  }

 private:
  double rate_, t0_, max_;
};

// Sine (f0 == f1) or linear chirp from f0 to f1 over `duration`, for `cycles` periods if > 0.
class SineSteer final : public SteerChannel {
 public:
  SineSteer(double amp, double f0, double f1, double t0, double duration, double cycles)
      : amp_(amp), f0_(f0), f1_(f1), t0_(t0), dur_(std::max(duration, 1e-3)), cycles_(cycles) {}
  double steer(double t, double, const Vehicle&) override {
    const double tau = t - t0_;
    if (tau < 0.0) return 0.0;
    if (f0_ == f1_) {
      if (cycles_ > 0.0 && tau * f0_ > cycles_) return 0.0;
      return amp_ * std::sin(2.0 * kPi * f0_ * tau);
    }
    if (tau > dur_) return 0.0;
    const double k = (f1_ - f0_) / dur_;
    return amp_ * std::sin(2.0 * kPi * (f0_ * tau + 0.5 * k * tau * tau));
  }

 private:
  double amp_, f0_, f1_, t0_, dur_, cycles_;
};

// ---------------------------------------------------------------- pedal channels

class NoPedals final : public SpeedChannel {
 public:
  void pedals(double, double, const Vehicle&, double& thr, double& brk) override { thr = brk = 0.0; }
};

// Fixed throttle/brake applied between tStart and tEnd; optionally hold speed before tStart.
class PedalStep final : public SpeedChannel {
 public:
  PedalStep(double thr, double brk, double t0, double t1, std::unique_ptr<SpeedHold> before)
      : thr_(thr), brk_(brk), t0_(t0), t1_(t1), before_(std::move(before)) {}
  void pedals(double t, double dt, const Vehicle& v, double& thr, double& brk) override {
    if (t < t0_) {
      if (before_) before_->pedals(t, dt, v, thr, brk);
      else thr = brk = 0.0;
      return;
    }
    if (t > t1_) {
      thr = brk = 0.0;
      return;
    }
    thr = thr_;
    brk = brk_;
  }
  void reset() override {
    if (before_) before_->reset();
  }

 private:
  double thr_, brk_, t0_, t1_;
  std::unique_ptr<SpeedHold> before_;
};

std::vector<std::pair<double, double>> circlePath(double radius, double laps) {
  // Starts at the origin heading +X; positive radius turns left (centre at (0, R)).
  std::vector<std::pair<double, double>> pts;
  const double r = std::abs(radius);
  const double dir = radius >= 0 ? 1.0 : -1.0;
  const int n = std::max(64, int(laps * 2.0 * kPi * r / 0.5));
  for (int i = 0; i <= n; ++i) {
    const double th = laps * 2.0 * kPi * double(i) / n;
    pts.emplace_back(r * std::sin(th), dir * (r - r * std::cos(th)));
  }
  return pts;
}

// ISO 3888-1 style double lane change built from cosine transitions.
std::vector<std::pair<double, double>> laneChangePath(double offset, double entry, double transition1,
                                                      double hold, double transition2, double exitLen) {
  std::vector<std::pair<double, double>> pts;
  const double total = entry + transition1 + hold + transition2 + exitLen;
  for (double x = -20.0; x <= total + 60.0; x += 0.5) {
    double y = 0.0;
    const double x1 = entry, x2 = x1 + transition1, x3 = x2 + hold, x4 = x3 + transition2;
    if (x <= x1) y = 0.0;
    else if (x <= x2) y = offset * 0.5 * (1.0 - std::cos(kPi * (x - x1) / transition1));
    else if (x <= x3) y = offset;
    else if (x <= x4) y = offset * 0.5 * (1.0 + std::cos(kPi * (x - x3) / transition2));
    pts.emplace_back(x, y);
  }
  return pts;
}

}  // namespace

// ---------------------------------------------------------------- PathFollower

PathFollower::PathFollower(std::vector<std::pair<double, double>> pts, double lookaheadTime, double minLookahead, double ki)
    : pts_(std::move(pts)), lookaheadTime_(lookaheadTime), minLookahead_(minLookahead), ki_(ki) {
  s_.resize(pts_.size(), 0.0);
  for (size_t i = 1; i < pts_.size(); ++i)
    s_[i] = s_[i - 1] + std::hypot(pts_[i].first - pts_[i - 1].first, pts_[i].second - pts_[i - 1].second);
}

double PathFollower::steer(double, double dt, const Vehicle& veh) {
  if (pts_.size() < 2) return 0.0;
  const Telemetry& tel = veh.telemetry();
  const double px = tel.x, py = tel.y;
  // Advance the closest-point index monotonically within a search window.
  size_t best = idx_;
  double bestD = 1e18;
  const size_t end = std::min(pts_.size(), idx_ + 200);
  for (size_t i = idx_; i < end; ++i) {
    const double d = std::hypot(pts_[i].first - px, pts_[i].second - py);
    if (d < bestD) { bestD = d; best = i; }
  }
  idx_ = best;

  // Signed cross-track error (positive = path is to the left of the car).
  const size_t j = std::min(best + 1, pts_.size() - 1);
  const size_t i0 = j == best ? best - 1 : best;
  const double tx = pts_[j].first - pts_[i0].first, ty = pts_[j].second - pts_[i0].second;
  const double tn = std::max(std::hypot(tx, ty), 1e-9);
  lastError_ = -(tx * (py - pts_[i0].second) - ty * (px - pts_[i0].first)) / tn;

  const double ld = std::max(minLookahead_, lookaheadTime_ * tel.speed);
  size_t k = best;
  while (k + 1 < pts_.size() && s_[k] - s_[best] < ld) ++k;
  const double dx = pts_[k].first - px, dy = pts_[k].second - py;
  const double c = std::cos(tel.yaw), s = std::sin(tel.yaw);
  const double lx = c * dx + s * dy;
  const double ly = -s * dx + c * dy;
  const double l2 = std::max(lx * lx + ly * ly, 1e-6);
  const double curvature = 2.0 * ly / l2;

  integ_ = clamp(integ_ + lastError_ * dt, -20.0, 20.0);
  const double L = veh.geometry().wheelbase;
  const double roadWheel = std::atan(curvature * L) + ki_ * integ_;
  return roadWheel * veh.params().steering.ratio;
}

// ---------------------------------------------------------------- SpeedHold

void SpeedHold::pedals(double t, double dt, const Vehicle& v, double& throttle, double& brake) {
  const double e = target(t) - v.telemetry().vx;
  const double cmdRaw = kp_ * e + integ_;
  // Conditional integration (anti-windup): only integrate while not saturated.
  if (!((cmdRaw >= 1.0 && e > 0) || (cmdRaw <= -1.0 && e < 0))) integ_ = clamp(integ_ + ki_ * e * dt, -1.0, 1.0);
  const double cmd = clamp(kp_ * e + integ_, -1.0, 1.0);
  throttle = std::max(cmd, 0.0);
  brake = std::max(-cmd, 0.0) * 0.5;
}

// ---------------------------------------------------------------- ManeuverDriver

DriverInputs ManeuverDriver::update(double t, double dt, const Vehicle& vehicle) {
  DriverInputs in;
  in.steer = steer_->steer(t, dt, vehicle);
  speed_->pedals(t, dt, vehicle, in.throttle, in.brake);
  return in;
}

void ManeuverDriver::reset() {
  steer_->reset();
  speed_->reset();
}

std::unique_ptr<ManeuverDriver> makeManeuver(const json::Value& spec, std::string* error) {
  const auto fail = [&](const std::string& msg) -> std::unique_ptr<ManeuverDriver> {
    if (error) *error = msg;
    return nullptr;
  };
  if (!spec.isObject()) return fail("maneuver must be an object");

  std::unique_ptr<SteerChannel> steer;
  const json::Value* st = spec.get("steer");
  const std::string stype = st ? st->string("type", "none") : "none";
  if (stype == "none") {
    steer = std::make_unique<NoSteer>();
  } else if (stype == "constant") {
    steer = std::make_unique<ConstantSteer>(deg2rad(st->number("angle_deg", 0.0)));
  } else if (stype == "step") {
    steer = std::make_unique<StepSteer>(deg2rad(st->number("angle_deg", 30.0)), st->number("t_start", 1.0),
                                        st->number("ramp_time", 0.15));
  } else if (stype == "ramp") {
    steer = std::make_unique<RampSteer>(deg2rad(st->number("rate_deg_s", 10.0)), st->number("t_start", 1.0),
                                        deg2rad(st->number("max_deg", 540.0)));
  } else if (stype == "sine" || stype == "chirp") {
    const double f0 = st->number(stype == "sine" ? "freq_hz" : "f0_hz", 0.5);
    const double f1 = stype == "sine" ? f0 : st->number("f1_hz", 3.0);
    steer = std::make_unique<SineSteer>(deg2rad(st->number("amplitude_deg", 30.0)), f0, f1, st->number("t_start", 1.0),
                                        st->number("duration", 10.0), st->number("cycles", 0.0));
  } else if (stype == "circle") {
    steer = std::make_unique<PathFollower>(circlePath(st->number("radius", 40.0), st->number("laps", 3.0)),
                                           st->number("lookahead_time", 0.45), st->number("min_lookahead", 4.0),
                                           st->number("ki", 0.02));
  } else if (stype == "lane_change") {
    steer = std::make_unique<PathFollower>(
        laneChangePath(st->number("offset", 3.5), st->number("entry", 15.0), st->number("transition1", 30.0),
                       st->number("hold", 25.0), st->number("transition2", 25.0), st->number("exit", 15.0)),
        st->number("lookahead_time", 0.5), st->number("min_lookahead", 5.0), st->number("ki", 0.0));
  } else if (stype == "path") {
    const json::Value* pts = st->get("points");
    if (!pts || !pts->isArray()) return fail("steer.path requires \"points\": [[x, y], ...]");
    std::vector<std::pair<double, double>> p;
    for (const auto& e : pts->asArray()) {
      if (!e.isArray() || e.asArray().size() < 2) return fail("path points must be [x, y]");
      p.emplace_back(e.asArray()[0].asNumber(), e.asArray()[1].asNumber());
    }
    steer = std::make_unique<PathFollower>(std::move(p), st->number("lookahead_time", 0.45),
                                           st->number("min_lookahead", 4.0), st->number("ki", 0.02));
  } else {
    return fail("unknown steer type '" + stype + "'");
  }

  std::unique_ptr<SpeedChannel> speed;
  const json::Value* sp = spec.get("speed");
  const std::string ptype = sp ? sp->string("type", "none") : "none";
  if (ptype == "none") {
    speed = std::make_unique<NoPedals>();
  } else if (ptype == "hold") {
    speed = std::make_unique<SpeedHold>(sp->number("target", 20.0), sp->number("accel", 0.0), sp->number("kp", 0.35),
                                        sp->number("ki", 0.12));
  } else if (ptype == "throttle" || ptype == "brake") {
    std::unique_ptr<SpeedHold> before;
    if (const json::Value* h = sp->get("hold_before")) before = std::make_unique<SpeedHold>(h->asNumber(), 0.0);
    const double v = sp->number("value", 1.0);
    speed = std::make_unique<PedalStep>(ptype == "throttle" ? v : 0.0, ptype == "brake" ? v : 0.0,
                                        sp->number("t_start", 0.0), sp->number("t_end", 1e9), std::move(before));
  } else {
    return fail("unknown speed type '" + ptype + "'");
  }
  return std::make_unique<ManeuverDriver>(std::move(steer), std::move(speed));
}

}  // namespace vd
