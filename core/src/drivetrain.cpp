#include "vd/drivetrain.hpp"

namespace vd {

std::unique_ptr<Differential> makeDifferential(DiffType type) {
  switch (type) {
    case DiffType::Locked: return std::make_unique<LockedDiff>();
    case DiffType::ClutchLSD: return std::make_unique<ClutchLSD>();
    case DiffType::Viscous: return std::make_unique<ViscousLSD>();
    case DiffType::Open: break;
  }
  return std::make_unique<OpenDiff>();
}

double engineTorque(const EngineParams& p, double rpm, double throttle) {
  // Idle governor keeps the engine alive; rev limiter cuts fuel.
  double thr = clamp(throttle, 0.0, 1.0);
  thr = std::max(thr, clamp((p.idle_rpm - rpm) / 300.0, 0.0, 1.0) * 0.5);
  if (rpm > p.rev_limit) thr = 0.0;
  const double full = p.torque_curve(rpm) * p.torque_scale;
  const double drag = p.friction_base + p.friction_per_krpm * std::max(rpm, 0.0) / 1000.0;
  return thr * full - (1.0 - thr) * drag;
}

double gearRatio(const GearboxParams& p, int gear) {
  if (gear <= 0 || gear > std::min(p.num_gears, kMaxGears)) return 0.0;
  return p.ratios[size_t(gear - 1)];
}

void Drivetrain::configure(const EngineParams& e, const ClutchParams& c, const GearboxParams& g,
                           const DrivetrainParams& d, const std::array<DiffParams, 3>& diffs) {
  e_ = &e;
  c_ = &c;
  g_ = &g;
  d_ = &d;
  dp_ = &diffs;
  for (size_t i = 0; i < 3; ++i) diffs_[i] = makeDifferential(DiffType(diffs[i].type));
}

double Drivetrain::propShaftSpeed(const std::array<double, 4>& w) const {
  const double wf = 0.5 * (w[FL] + w[FR]);
  const double wr = 0.5 * (w[RL] + w[RR]);
  switch (DriveLayout(d_->layout)) {
    case DriveLayout::FWD: return wf;
    case DriveLayout::AWD: return d_->awd_front_split * wf + (1.0 - d_->awd_front_split) * wr;
    case DriveLayout::RWD: break;
  }
  return wr;
}

bool Drivetrain::usesTwist(int diff) const {
  if (DiffType((*dp_)[size_t(diff)].type) != DiffType::Locked) return false;
  switch (DriveLayout(d_->layout)) {
    case DriveLayout::RWD: return diff == Rear;
    case DriveLayout::FWD: return diff == Front;
    case DriveLayout::AWD: return true;
  }
  return false;
}

DrivetrainOutput Drivetrain::compute(const DrivetrainInput& in) const {
  DrivetrainOutput out;
  const double rpm = radps2rpm(in.engine_speed);
  out.engine_torque = engineTorque(*e_, rpm, in.throttle);

  const double ratio = gearRatio(*g_, in.gear) * g_->final_drive;
  if (ratio != 0.0) {
    // Centrifugal-style launch clutch: open at idle, closed above engage_rpm.
    const double autoClutch = smoothstep(e_->idle_rpm + 50.0, std::max(c_->engage_rpm, e_->idle_rpm + 100.0), rpm);
    const double engagement = clamp(in.engagement, 0.0, 1.0) * autoClutch;
    const double dw = in.engine_speed - propShaftSpeed(in.wheel_speed) * ratio;
    out.clutch_torque = engagement * c_->max_torque * std::tanh(dw / std::max(c_->slip_speed, 1e-3));
  }
  out.engine_accel = (out.engine_torque - out.clutch_torque) / std::max(e_->inertia, 1e-3);

  const double tshaft = out.clutch_torque * ratio * g_->efficiency;
  const auto& w = in.wheel_speed;
  auto& t = out.wheel_torque;
  const double wf = 0.5 * (w[FL] + w[FR]), wr = 0.5 * (w[RL] + w[RR]);
  out.twist_rate = {w[FL] - w[FR], w[RL] - w[RR], wf - wr};
  const auto axle = [&](int axleIdx, double tin, int left, int right) {
    const size_t a = size_t(axleIdx);
    diffs_[a]->split(tin, w[size_t(left)], w[size_t(right)], in.twist[a], 0.5, (*dp_)[a], t[size_t(left)],
                     t[size_t(right)]);
  };
  switch (DriveLayout(d_->layout)) {
    case DriveLayout::RWD: axle(Rear, tshaft, RL, RR); break;
    case DriveLayout::FWD: axle(Front, tshaft, FL, FR); break;
    case DriveLayout::AWD: {
      double tf = 0.0, tr = 0.0;
      diffs_[2]->split(tshaft, wf, wr, in.twist[2], d_->awd_front_split, (*dp_)[2], tf, tr);
      axle(Front, tf, FL, FR);
      axle(Rear, tr, RL, RR);
      break;
    }
  }
  return out;
}

}  // namespace vd
