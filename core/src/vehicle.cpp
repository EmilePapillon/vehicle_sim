#include "vd/vehicle.hpp"

namespace vd {

namespace {
constexpr double kLowSpeed = 3.0;        // below this, tire slip damping fades in [m/s]
constexpr double kMinRelaxSpeed = 0.3;   // slip states still decay at standstill [m/s]
constexpr double kCamberRollSpeed = 1.0; // camber thrust builds up with rolling speed [m/s]
}

Vehicle::Vehicle(const VehicleParams& p) : p_(p) {
  onParamsChanged();
  reset();
}

bool Vehicle::setParam(const std::string& key, double value) {
  if (!registry_.set(key, value)) return false;
  onParamsChanged();
  return true;
}

bool Vehicle::loadParams(const json::Value& root, std::vector<std::string>* unknownKeys, std::string* error) {
  const bool ok = registry_.load(root, unknownKeys, error);
  onParamsChanged();
  return ok;
}

void Vehicle::onParamsChanged() {
  for (int a = 0; a < 2; ++a) {
    const auto tt = TireModelType(p_.tire[size_t(a)].model);
    if (!tires_[size_t(a)] || tires_[size_t(a)]->type() != tt) tires_[size_t(a)] = makeTireModel(tt);
    suspension_[a] = makeSuspensionModel(SuspensionModelType(p_.suspension[size_t(a)].model));
  }
  drivetrain_.configure(p_.engine, p_.clutch, p_.gearbox, p_.drivetrain, p_.diff);
  gear_ = std::min(gear_, p_.gearbox.num_gears);
  target_gear_ = std::min(target_gear_, p_.gearbox.num_gears);
  computeDerived();
}

void Vehicle::computeDerived() {
  const auto& c = p_.chassis;
  Derived& d = d_;
  d.L = c.wheelbase;
  d.b = d.L * c.weight_front;  // CG to rear axle
  d.a = d.L - d.b;             // CG to front axle
  const double tf = p_.suspension[Front].track, tr = p_.suspension[Rear].track;
  d.x = {d.a, d.a, -d.b, -d.b};
  d.y = {0.5 * tf, -0.5 * tf, 0.5 * tr, -0.5 * tr};
  d.mu = {c.unsprung_mass_front, c.unsprung_mass_front, c.unsprung_mass_rear, c.unsprung_mass_rear};
  d.mu_total = 2.0 * (c.unsprung_mass_front + c.unsprung_mass_rear);
  d.ms = std::max(c.mass - d.mu_total, 0.3 * c.mass);
  const double rf = p_.tire[Front].radius, rr = p_.tire[Rear].radius;
  const double unsprungMoment = 2.0 * (c.unsprung_mass_front * rf + c.unsprung_mass_rear * rr);
  d.hs = std::max((c.mass * c.cg_height - unsprungMoment) / d.ms, 0.05);
  for (int i = 0; i < 4; ++i) {
    const double axleShare = axleOf(i) == Front ? d.b / d.L : d.a / d.L;
    d.static_fz[size_t(i)] = 0.5 * c.mass * kGravity * axleShare;
    d.sprung_static[size_t(i)] = std::max(d.static_fz[size_t(i)] - d.mu[size_t(i)] * kGravity, 1.0);
  }
  const double rollAxis = p_.suspension[Front].roll_center_height * (d.b / d.L) +
                          p_.suspension[Rear].roll_center_height * (d.a / d.L);
  d.roll_arm = d.hs - rollAxis;
  d.pitch_arm = d.hs - c.pitch_center_height;
}

Geometry Vehicle::geometry() const {
  return Geometry{d_.L, d_.a, d_.b, p_.suspension[Front].track, p_.suspension[Rear].track,
                  p_.chassis.cg_height, p_.tire[Front].radius, p_.tire[Rear].radius};
}

TireForces Vehicle::evalTire(int axle, const TireInput& in) const {
  const size_t a = axle == Rear ? 1 : 0;
  return tires_[a]->compute(in, p_.tire[a]);
}

void Vehicle::reset(double x, double y, double yaw, double speed) {
  s_.fill(0.0);
  s_[X] = x;
  s_[Y] = y;
  s_[PSI] = yaw;
  s_[U] = speed;
  for (int i = 0; i < 4; ++i) s_[size_t(W0 + i)] = speed / p_.tire[size_t(axleOf(i))].radius;
  time_ = accum_ = distance_ = 0.0;
  ax_prev_ = ay_prev_ = 0.0;
  spin_.fill(0.0);
  absFactor_.fill(1.0);
  tcsFactor_ = 1.0;
  shift_timer_ = 0.0;

  // Pick the gear that puts the engine in its operating band.
  gear_ = 1;
  std::array<double, 4> w{};
  for (int i = 0; i < 4; ++i) w[size_t(i)] = s_[size_t(W0 + i)];
  const double shaft = drivetrain_.propShaftSpeed(w);
  const auto& g = p_.gearbox;
  for (int k = 1; k <= g.num_gears; ++k) {
    gear_ = k;
    if (radps2rpm(shaft * gearRatio(g, k) * g.final_drive) < 0.5 * (g.shift_up_rpm + g.shift_down_rpm)) break;
  }
  target_gear_ = gear_;
  const double rpm = radps2rpm(shaft * gearRatio(g, gear_) * g.final_drive);
  s_[WE] = rpm2radps(std::max(rpm, p_.engine.idle_rpm));

  computeSteer();
  State ds{};
  derivatives(s_, ds, &tel_);
  tel_.time = 0.0;
}

void Vehicle::computeSteer() {
  const auto& st = p_.steering;
  const double lock = deg2rad(st.max_angle);
  const double sw = clamp(inputs_.steer, -lock, lock);
  const double dm = sw / std::max(st.ratio, 1e-3);
  double dl = dm, dr = dm;
  if (std::abs(dm) > 1e-6) {
    const double tf = p_.suspension[Front].track;
    const double rad = d_.L / std::tan(std::abs(dm));
    const double inner = std::atan(d_.L / std::max(rad - 0.5 * tf, 0.05));
    const double outer = std::atan(d_.L / (rad + 0.5 * tf));
    const double mag = std::abs(dm);
    const double in = mag + st.ackermann * (inner - mag);
    const double out = mag + st.ackermann * (outer - mag);
    if (dm > 0) { dl = in; dr = out; }  // left turn: left wheel is inside
    else { dl = -out; dr = -in; }
  }
  const double toeF = deg2rad(p_.suspension[Front].toe), toeR = deg2rad(p_.suspension[Rear].toe);
  steer_ = {dl - toeF, dr + toeF, -toeR, toeR};
}

void Vehicle::updateGearbox(double dt) {
  const auto& g = p_.gearbox;
  if (inputs_.gear < 0) { gear_ = 0; shift_timer_ = 0.0; return; }
  if (inputs_.gear > 0) { gear_ = std::min(inputs_.gear, g.num_gears); shift_timer_ = 0.0; return; }
  if (shift_timer_ > 0.0) {
    shift_timer_ -= dt;
    if (shift_timer_ <= 0.0) { gear_ = target_gear_; shift_timer_ = 0.0; }
    return;
  }
  if (gear_ <= 0) gear_ = 1;
  if (!g.automatic) return;

  std::array<double, 4> w{};
  for (int i = 0; i < 4; ++i) w[size_t(i)] = s_[size_t(W0 + i)];
  const double shaft = drivetrain_.propShaftSpeed(w);
  const auto rpmIn = [&](int gear) { return radps2rpm(shaft * gearRatio(g, gear) * g.final_drive); };
  int next = gear_;
  // Part-throttle upshifts happen earlier, but only if the next gear stays above the
  // downshift point (with margin) so the box cannot hunt between gears.
  const double upRpm = g.shift_down_rpm + (g.shift_up_rpm - g.shift_down_rpm) * clamp(inputs_.throttle, 0.3, 1.0);
  if (gear_ < g.num_gears && rpmIn(gear_) > upRpm && rpmIn(gear_ + 1) > 1.15 * g.shift_down_rpm) next = gear_ + 1;
  else if (gear_ > 1 && rpmIn(gear_) < g.shift_down_rpm && rpmIn(gear_ - 1) < 0.9 * g.shift_up_rpm) next = gear_ - 1;
  if (next != gear_) {
    target_gear_ = next;
    if (g.shift_time <= 0.0) gear_ = next;
    else shift_timer_ = g.shift_time;
  }
}

void Vehicle::updateAids(double dt) {
  // Proportional slip limiters with a first-order actuator lag (~25 ms).
  const auto& ai = p_.aids;
  const double lag = std::min(1.0, dt / 0.025);
  for (int i = 0; i < 4; ++i) {
    double target = 1.0;
    if (ai.abs) target = clamp(1.0 - (-s_[size_t(K0 + i)] - ai.abs_slip) / 0.08, 0.0, 1.0);
    absFactor_[size_t(i)] += (target - absFactor_[size_t(i)]) * lag;
  }
  double target = 1.0;
  if (ai.tcs) {
    double kmax = 0.0;
    const auto layout = DriveLayout(p_.drivetrain.layout);
    for (int i = 0; i < 4; ++i) {
      const bool driven = layout == DriveLayout::AWD || (layout == DriveLayout::FWD) == (axleOf(i) == Front);
      if (driven) kmax = std::max(kmax, s_[size_t(K0 + i)]);
    }
    target = clamp(1.0 - (kmax - ai.tcs_slip) / 0.08, 0.0, 1.0);
  }
  tcsFactor_ += (target - tcsFactor_) * lag;
}

void Vehicle::step(double dt) {
  computeSteer();
  updateGearbox(dt);
  updateAids(dt);

  State k1, k2, k3, k4, tmp;
  const auto axpy = [](State& out, const State& base, const State& k, double h) {
    for (size_t i = 0; i < size_t(kNumStates); ++i) out[i] = base[i] + h * k[i];
  };
  switch (IntegratorType(p_.solver.integrator)) {
    case IntegratorType::Euler:
      derivatives(s_, k1, nullptr);
      axpy(s_, s_, k1, dt);
      break;
    case IntegratorType::Heun:
      derivatives(s_, k1, nullptr);
      axpy(tmp, s_, k1, dt);
      derivatives(tmp, k2, nullptr);
      for (size_t i = 0; i < size_t(kNumStates); ++i) s_[i] += 0.5 * dt * (k1[i] + k2[i]);
      break;
    case IntegratorType::RK4:
      derivatives(s_, k1, nullptr);
      axpy(tmp, s_, k1, 0.5 * dt);
      derivatives(tmp, k2, nullptr);
      axpy(tmp, s_, k2, 0.5 * dt);
      derivatives(tmp, k3, nullptr);
      axpy(tmp, s_, k3, dt);
      derivatives(tmp, k4, nullptr);
      for (size_t i = 0; i < size_t(kNumStates); ++i) s_[i] += dt / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
      break;
  }

  // Keep transient slip states inside the range the tire models are defined on.
  for (int i = 0; i < 4; ++i) {
    s_[size_t(K0 + i)] = clamp(s_[size_t(K0 + i)], -1.0, 3.0);
    s_[size_t(A0 + i)] = clamp(s_[size_t(A0 + i)], -1.4, 1.4);
  }
  s_[WE] = std::max(s_[WE], 0.0);
  for (int k = 0; k < 3; ++k)
    if (!drivetrain_.usesTwist(k)) s_[size_t(TW0 + k)] = 0.0;  // only spools wind up

  time_ += dt;
  State ds{};
  derivatives(s_, ds, &tel_);
  ax_prev_ = tel_.ax;
  ay_prev_ = tel_.ay;
  for (int i = 0; i < 4; ++i) {
    spin_[size_t(i)] = std::fmod(spin_[size_t(i)] + s_[size_t(W0 + i)] * dt, 2.0 * kPi);
    tel_.wheel[size_t(i)].spin_angle = spin_[size_t(i)];
  }
  distance_ += tel_.speed * dt;
  tel_.distance = distance_;
  tel_.time = time_;
}

void Vehicle::advance(double duration) {
  const double dt = clamp(p_.solver.dt, 1e-5, 0.01);
  accum_ += duration;
  while (accum_ >= dt * (1.0 - 1e-9)) {
    step(dt);
    accum_ -= dt;
  }
}

void Vehicle::derivatives(const State& s, State& ds, Telemetry* tel) const {
  const auto& c = p_.chassis;
  const Derived& d = d_;
  const double u = s[U], v = s[V], r = s[R];
  const double m = c.mass;
  const double ax = ax_prev_, ay = ay_prev_;

  // --- Suspension forces (deviation from static, on the body, upward positive)
  std::array<double, 4> fs{}, comp{};
  for (size_t i = 0; i < 4; ++i) {
    const double zi = s[Z] + d.y[i] * s[PHI] - d.x[i] * s[THETA];
    const double zdi = s[ZD] + d.y[i] * s[PHID] - d.x[i] * s[THETAD];
    comp[i] = -zi;
    const size_t a = size_t(axleOf(int(i)));
    fs[i] = suspension_[a]->cornerForce(comp[i], -zdi, p_.suspension[a]);
  }
  for (size_t a = 0; a < 2; ++a) {
    const size_t l = 2 * a, rgt = 2 * a + 1;
    const double f = p_.suspension[a].arb_rate * (comp[l] - comp[rgt]);
    fs[l] += f;
    fs[rgt] -= f;
  }
  // A spring cannot pull the wheel: beyond full unloading the wheel lifts off.
  for (size_t i = 0; i < 4; ++i) fs[i] = std::max(fs[i], -d.sprung_static[i]);

  // --- Aero
  const auto& ae = p_.aero;
  const double q = 0.5 * ae.air_density * u * std::abs(u);
  const double drag = q * ae.drag_area;
  const double down = 0.5 * ae.air_density * u * u * ae.lift_area;
  const double downF = down * ae.balance, downR = down - downF;

  // --- Sprung-mass heave / roll / pitch
  double sumF = 0.0, rollM = 0.0, pitchM = 0.0;
  for (size_t i = 0; i < 4; ++i) {
    sumF += fs[i];
    rollM += d.y[i] * fs[i];
    pitchM -= d.x[i] * fs[i];
  }
  const double ms = d.ms;
  ds[Z] = s[ZD];
  ds[ZD] = (sumF - down) / ms;
  ds[PHI] = s[PHID];
  ds[PHID] = (ms * ay * d.roll_arm + ms * kGravity * d.roll_arm * s[PHI] + rollM) / c.roll_inertia;
  ds[THETA] = s[THETAD];
  ds[THETAD] = (-ms * ax * d.pitch_arm + ms * kGravity * d.pitch_arm * s[THETA] + pitchM + d.a * downF - d.b * downR) /
               c.pitch_inertia;

  // --- Wheel loads
  std::array<double, 4> fz{};
  const double L = d.L;
  const double geoLong = (ms * ax * c.pitch_center_height +
                          2.0 * ax * (c.unsprung_mass_front * p_.tire[Front].radius + c.unsprung_mass_rear * p_.tire[Rear].radius)) / L;
  for (size_t a = 0; a < 2; ++a) {
    const auto& sp = p_.suspension[a];
    const double share = a == Front ? d.b / L : d.a / L;
    const double unsprungAxle = 2.0 * d.mu[2 * a];
    const double geoLat = (ms * ay * share * sp.roll_center_height + unsprungAxle * ay * p_.tire[a].radius) / sp.track;
    const double longShare = a == Front ? -0.5 * geoLong : 0.5 * geoLong;
    fz[2 * a] = d.static_fz[2 * a] + fs[2 * a] - geoLat + longShare;
    fz[2 * a + 1] = d.static_fz[2 * a + 1] + fs[2 * a + 1] + geoLat + longShare;
  }
  for (auto& f : fz) f = std::max(f, 0.0);

  // --- Drivetrain
  const bool shifting = shift_timer_ > 0.0;
  DrivetrainInput din;
  din.engine_speed = s[WE];
  for (size_t i = 0; i < 4; ++i) din.wheel_speed[i] = s[size_t(W0) + i];
  din.throttle = shifting ? 0.0 : inputs_.throttle * tcsFactor_;
  din.gear = gear_;
  din.engagement = shifting ? 0.0 : 1.0;
  for (size_t k = 0; k < 3; ++k) din.twist[k] = s[size_t(TW0) + k];
  const DrivetrainOutput dout = drivetrain_.compute(din);
  ds[WE] = dout.engine_accel;
  for (size_t k = 0; k < 3; ++k) ds[size_t(TW0) + k] = dout.twist_rate[k];

  // --- Tires and wheels
  const auto& br = p_.brakes;
  const double brake = clamp(inputs_.brake, 0.0, 1.0);
  const std::array<double, 4> brakeT = {
      0.5 * br.max_torque * br.front_bias * brake, 0.5 * br.max_torque * br.front_bias * brake,
      0.5 * br.max_torque * (1.0 - br.front_bias) * brake + br.handbrake_torque * inputs_.handbrake,
      0.5 * br.max_torque * (1.0 - br.front_bias) * brake + br.handbrake_torque * inputs_.handbrake};

  double fxSum = -drag, fySum = 0.0, mzSum = 0.0;
  for (size_t i = 0; i < 4; ++i) {
    const size_t a = size_t(axleOf(int(i)));
    const TireParams& tp = p_.tire[a];
    const auto& sp = p_.suspension[a];
    const double delta = steer_[i];
    const double cs = std::cos(delta), sn = std::sin(delta);
    const double vxc = u - r * d.y[i];
    const double vyc = v + r * d.x[i];
    const double vxw = vxc * cs + vyc * sn;
    const double vyw = -vxc * sn + vyc * cs;
    const double w = s[size_t(W0) + i];
    const double kappa = s[size_t(K0) + i];
    const double alpha = s[size_t(A0) + i];
    const double R = tp.radius;
    const double absvx = std::abs(vxw);

    // Relaxation-length slip dynamics. The decay speed has a floor so the carcass deflection
    // bleeds off at standstill instead of integrating creep forever (identical above 0.3 m/s).
    const double relaxSpeed = std::max(absvx, kMinRelaxSpeed);
    ds[size_t(K0) + i] = (w * R - vxw - relaxSpeed * kappa) / tp.relaxation_long;
    ds[size_t(A0) + i] = (-vyw - relaxSpeed * alpha) / tp.relaxation_lat;

    const double gamma = sideSign(int(i)) * deg2rad(sp.static_camber) - sp.camber_roll_coeff * s[PHI];
    // Camber thrust is a rolling effect: a stationary tilted tire produces no side force.
    const double camberThrust = gamma * smoothstep(0.0, kCamberRollSpeed, absvx);
    TireInput tin{fz[i], clamp(kappa, -1.0, 3.0), clamp(alpha, -1.4, 1.4), camberThrust};
    TireForces f = tires_[a]->compute(tin, tp);

    // Low-speed slip damping (relaxation-length models are undamped at standstill).
    if (absvx < kLowSpeed && fz[i] > 0.0) {
      const double fade = 1.0 - absvx / kLowSpeed;
      const double cy = std::sqrt(tires_[a]->corneringStiffness(tp, fz[i]) / tp.relaxation_lat * fz[i] / kGravity);
      const double cx = std::sqrt(tires_[a]->longitudinalStiffness(tp, fz[i]) / tp.relaxation_long * c.wheel_inertia / (R * R));
      f.fy -= fade * cy * vyw;
      f.fx -= fade * cx * (vxw - w * R);
      const double fmax = 1.2 * frictionCoefficient(tp, fz[i]) * fz[i];
      f.fx = clamp(f.fx, -fmax, fmax);
      f.fy = clamp(f.fy, -fmax, fmax);
    }

    const double fxb = f.fx * cs - f.fy * sn;
    const double fyb = f.fx * sn + f.fy * cs;
    fxSum += fxb;
    fySum += fyb;
    mzSum += d.x[i] * fyb - d.y[i] * fxb + f.mz;

    const double tb = brakeT[i] * absFactor_[i] * std::tanh(w / br.lock_speed);
    const double trr = tp.rolling_resistance * fz[i] * R * std::tanh(w);
    ds[size_t(W0) + i] = (dout.wheel_torque[i] - tb - trr - f.fx * R) / c.wheel_inertia;

    if (tel) {
      WheelTelemetry& wt = tel->wheel[i];
      wt.fz = fz[i];
      wt.fx = f.fx;
      wt.fy = f.fy;
      wt.mz = f.mz;
      wt.slip_ratio = kappa;
      wt.slip_angle = alpha;
      wt.omega = w;
      wt.steer = delta;
      wt.camber = gamma;
      wt.compression = comp[i];
      wt.drive_torque = dout.wheel_torque[i];
      wt.brake_torque = tb;
    }
  }

  const double psi = s[PSI];
  ds[X] = u * std::cos(psi) - v * std::sin(psi);
  ds[Y] = u * std::sin(psi) + v * std::cos(psi);
  ds[PSI] = r;
  ds[U] = fxSum / m + v * r;
  ds[V] = fySum / m - u * r;
  ds[R] = mzSum / c.yaw_inertia;

  if (tel) {
    tel->x = s[X];
    tel->y = s[Y];
    tel->yaw = psi;
    tel->heave = s[Z];
    tel->roll = s[PHI];
    tel->pitch = s[THETA];
    tel->vx = u;
    tel->vy = v;
    tel->yaw_rate = r;
    tel->speed = std::hypot(u, v);
    tel->ax = fxSum / m;
    tel->ay = fySum / m;
    tel->sideslip = std::atan2(v, std::max(std::abs(u), 0.5));
    tel->steer = inputs_.steer;
    tel->throttle = inputs_.throttle;
    tel->brake = inputs_.brake;
    tel->engine_rpm = radps2rpm(s[WE]);
    tel->engine_torque = dout.engine_torque;
    tel->clutch_torque = dout.clutch_torque;
    tel->gear = shifting ? 0 : gear_;
  }
}

}  // namespace vd
