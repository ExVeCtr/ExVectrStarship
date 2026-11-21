// Native harness of the body simulator model: pio test -e native -f test_sim_aero
//
// Deterministic fixed-step runs of DSP::BodySimModel (lib/ExVectrDSP body_sim_model) with scripted actuator inputs.
// The same scenarios also run on a replica of the model before feature/sim-aero (legacy_model.hpp) and print both
// as a table ("SIMAERO" lines), so the before/after numbers in the docs can be reproduced:
//   pio test -e native -f test_sim_aero -v | grep SIMAERO
// Set SIM_AERO_CSV=<dir> to also write one CSV per scenario (time series) for plotting.
//
// Acceptance (new model): hover balance, belly-down terminal speed 6.4 m/s +- 3 %, belly-down statically stable
// and reached from a small tilt, flap and vane responses with the expected signs, bit-identical repeated runs,
// angular momentum conserved in a torque-free spin.
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unity.h>

#include "ExVectrDSP/body_sim_model.hpp"
#include "legacy_model.hpp"
#include "safety/sim_flap_fault.hpp"

using namespace VCTR;
using namespace VCTR::Math;
using DSP::BodySimModel;
using DSP::BodySimParams;

void setUp() {}
void tearDown() {}

static constexpr float kDeg = 3.14159265f / 180.0f;
static constexpr float kG = 9.807f;
static constexpr float kTopNeutral = 50 * kDeg, kBottomNeutral = 40 * kDeg; // ControlAttitudeFlaps neutral

// ---------------------------------------------------------------------------------------------------------------
// Adapters: one interface over the new model and the legacy replica.
// ---------------------------------------------------------------------------------------------------------------
struct NewSim {
  BodySimModel m;
  NewSim() : m(BodySimParams()) {}
  explicit NewSim(const BodySimParams &p) : m(p) {}
  const char *name() const { return "new"; }
  float dt() const { return m.params().step_s; }
  void setState(const Vector<float, 7> &a, const Vector<float, 6> &p) {
    m.setAttitudeState(a);
    m.setPositionState(p);
  }
  void setFlaps(float tl, float tr, float bl, float br) { m.setFlapInput(tl, tr, bl, br); }
  void setThrust(const Vector_F &f, bool on, float roll = 0) { m.setThrust(f, roll, on); }
  void step() { m.step(); }
  Vector<float, 7> att() { return m.attitudeState(); }
  Vector<float, 6> pos() { return m.positionState(); }
  float flap(int i) { return m.flapAngle(i); }
};

struct OldSim {
  legacy::LegacyModel m;
  float cmd[4] = {1.5708f, 1.5708f, 1.5708f, 1.5708f};
  Vector_F thrustCmd;
  float ax = 0, ay = 0;
  OldSim() { thrustCmd = {0, 0, 0}; }
  const char *name() const { return "old"; }
  float dt() const { return 0.01f; } // nominal task period, wall-clock dt
  void setState(const Vector<float, 7> &a, const Vector<float, 6> &p) {
    m.att = a;
    m.pos = p;
  }
  void setFlaps(float tl, float tr, float bl, float br) {
    cmd[0] = tl, cmd[1] = tr, cmd[2] = bl, cmd[3] = br;
  }
  void setThrust(const Vector_F &f, bool on, float = 0) { thrustCmd = on ? f : Vector_F{0, 0, 0}; }
  void step() {
    const float d = dt();
    for (int i = 0; i < 4; i++) { // old flap driver: 200 deg/s, clamp 15..90
      float c = cmd[i] < 15 * kDeg ? 15 * kDeg : (cmd[i] > 90 * kDeg ? 90 * kDeg : cmd[i]);
      float diff = c - m.flaps[i], mx = 200 * kDeg * d;
      m.flaps[i] += diff > mx ? mx : (diff < -mx ? -mx : diff);
    }
    // old TVC driver: thrust angles slewed at 600 deg/s
    float mag = thrustCmd.magnitude();
    if (mag > 1e-6f) {
      float tx = atan2f(thrustCmd(1), thrustCmd(2)), ty = atan2f(thrustCmd(0), thrustCmd(2));
      float mx = 600 * kDeg * d;
      ax += (tx - ax) > mx ? mx : ((tx - ax) < -mx ? -mx : (tx - ax));
      ay += (ty - ay) > mx ? mx : ((ty - ay) < -mx ? -mx : (ty - ay));
      m.thrust = {sinf(ay) * mag, sinf(ax) * mag, cosf(ax) * cosf(ay) * mag};
    } else {
      m.thrust = {0, 0, 0};
    }
    m.step(d);
  }
  Vector<float, 7> att() { return m.att; }
  Vector<float, 6> pos() { return m.pos; }
  float flap(int i) { return m.flaps[i]; }
};

// ---------------------------------------------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------------------------------------------
static Vector<float, 7> attOf(const Quat_F &q, const Vector_F &w = Vector_F{0, 0, 0}) {
  return {w(0), w(1), w(2), q(0), q(1), q(2), q(3)};
}
static Quat_F qOf(const Vector<float, 7> &a) { return Quat_F(a(3), a(4), a(5), a(6)); }
/// Belly down: body +X up, nose (body +Z) horizontal toward world -X (South): the bellyflop setpoint Ry(-90 deg).
static Quat_F bellyDown(float pitchDeg = 0, float rollDeg = 0) {
  // Positive pitchDeg = nose up (rotation about body +Y by +pitch tips the nose toward body +X = up).
  Quat_F q = Quat_F(Vector_F{0, 1, 0}, -90 * kDeg) * Quat_F(Vector_F{0, 1, 0}, pitchDeg * kDeg);
  return q * Quat_F(Vector_F{0, 0, 1}, rollDeg * kDeg);
}
static Vector_F axisWorld(const Vector<float, 7> &a, float x, float y, float z) {
  return qOf(a).rotate(Vector_F{x, y, z});
}
/// Tilt of body +Z from world up, deg.
static float tiltDeg(const Vector<float, 7> &a) {
  Vector_F z = axisWorld(a, 0, 0, 1);
  float c = z(2) > 1 ? 1 : (z(2) < -1 ? -1 : z(2));
  return acosf(c) / kDeg;
}
/// Pitch as the flap controller defines it: elevation of body +Z above the horizontal, deg (+ = nose up).
static float pitchDeg(const Vector<float, 7> &a) {
  Vector_F z = axisWorld(a, 0, 0, 1);
  return atan2f(z(2), sqrtf(z(0) * z(0) + z(1) * z(1))) / kDeg;
}
/// Belly-down angle: body +X vs world up, deg (0 = belly exactly down).
static float bellyDeg(const Vector<float, 7> &a) {
  Vector_F x = axisWorld(a, 1, 0, 0);
  float c = x(2) > 1 ? 1 : (x(2) < -1 ? -1 : x(2));
  return acosf(c) / kDeg;
}

struct Csv {
  FILE *f = nullptr;
  Csv(const char *sim, const char *scen) {
    const char *dir = getenv("SIM_AERO_CSV");
    if (!dir)
      return;
    char path[512];
    snprintf(path, sizeof path, "%s/%s_%s.csv", dir, scen, sim);
    f = fopen(path, "w");
    if (f)
      fprintf(f, "t,vN,vW,vU,pU,wx,wy,wz,tilt,pitch,belly,fTL,fTR,fBL,fBR\n");
  }
  template <class S> void row(double t, S &s) {
    if (!f)
      return;
    auto a = s.att();
    auto p = s.pos();
    fprintf(f, "%.4f,%.4f,%.4f,%.4f,%.3f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f\n", t, p(0), p(1), p(2),
            p(5), a(0), a(1), a(2), tiltDeg(a), pitchDeg(a), bellyDeg(a), s.flap(0) / kDeg, s.flap(1) / kDeg,
            s.flap(2) / kDeg, s.flap(3) / kDeg);
  }
  ~Csv() {
    if (f)
      fclose(f);
  }
};

// ---------------------------------------------------------------------------------------------------------------
// Scenarios (templated over the adapter)
// ---------------------------------------------------------------------------------------------------------------
struct HoverResult {
  float vU, dz, tilt;
};
template <class S> HoverResult runHover(S &s, float seconds) {
  s.setState(attOf(Quat_F(1, 0, 0, 0)), Vector<float, 6>{0, 0, 0, 0, 0, 10});
  s.setFlaps(90 * kDeg, 90 * kDeg, 90 * kDeg, 90 * kDeg);
  s.setThrust(Vector_F{0, 0, 1.1f * kG}, true);
  Csv csv(s.name(), "hover");
  int n = int(seconds / s.dt() + 0.5f);
  for (int i = 0; i < n; i++) {
    s.step();
    if (i % 5 == 0)
      csv.row(i * s.dt(), s);
  }
  return {s.pos()(2), s.pos()(5) - 10.0f, tiltDeg(s.att())};
}

/// Belly-down fall with the attitude held fixed (pure drag): steady vertical speed after `seconds`.
template <class S> float runTerminalHeld(S &s, float seconds, float top, float bottom) {
  const Quat_F q = bellyDown();
  s.setState(attOf(q), Vector<float, 6>{0, 0, 0, 0, 0, 5000});
  s.setFlaps(top, top, bottom, bottom);
  s.setThrust(Vector_F{0, 0, 0}, false);
  int n = int(seconds / s.dt() + 0.5f);
  for (int i = 0; i < n; i++) {
    auto p = s.pos();
    s.setState(attOf(q), p); // hold attitude, zero rates
    s.step();
  }
  return -s.pos()(2);
}

struct OscResult {
  float trimPitch;  ///< mean pitch after the transient, deg
  float period;     ///< s (0 = no oscillation found)
  float zeta;       ///< damping ratio from the log decrement
  float bellyMax;   ///< max belly-down angle over the run, deg
  float bellyEnd;   ///< belly-down angle at the end, deg
  float vEnd;       ///< vertical speed at the end, m/s (positive down)
  float vHorEnd;    ///< horizontal speed at the end (glide), m/s
  float aoaEnd;     ///< angle between the air velocity and the body axis at the end, deg (90 = broadside)
};
/// Free (uncontrolled) belly-down fall from `v0` down with an initial pitch perturbation, flaps fixed.
template <class S> OscResult runBellyFree(S &s, const char *scen, float v0, float pitch0, float roll0,
                                          float seconds, float top, float bottom) {
  s.setState(attOf(bellyDown(pitch0, roll0)), Vector<float, 6>{0, 0, -v0, 0, 0, 5000});
  s.setFlaps(top, top, bottom, bottom);
  // let the actuator models settle on the commanded angle first (rate limits), body held
  for (int i = 0; i < int(1.0f / s.dt()); i++) {
    s.setState(attOf(bellyDown(pitch0, roll0)), Vector<float, 6>{0, 0, -v0, 0, 0, 5000});
    s.step();
  }
  s.setState(attOf(bellyDown(pitch0, roll0)), Vector<float, 6>{0, 0, -v0, 0, 0, 5000});
  Csv csv(s.name(), scen);
  int n = int(seconds / s.dt() + 0.5f);
  static float pitch[40000];
  if (n > 40000)
    n = 40000;
  OscResult r{0, 0, 0, 0, 0, 0, 0, 0};
  for (int i = 0; i < n; i++) {
    s.step();
    auto a = s.att();
    pitch[i] = pitchDeg(a);
    float b = bellyDeg(a);
    r.bellyMax = b > r.bellyMax ? b : r.bellyMax;
    if (i % 5 == 0)
      csv.row(i * s.dt(), s);
  }
  r.bellyEnd = bellyDeg(s.att());
  r.vEnd = -s.pos()(2);
  {
    auto p = s.pos();
    r.vHorEnd = sqrtf(p(0) * p(0) + p(1) * p(1));
    Vector_F vb = qOf(s.att()).conjugate().rotate(Vector_F{p(0), p(1), p(2)});
    r.aoaEnd = atan2f(sqrtf(vb(0) * vb(0) + vb(1) * vb(1)), fabsf(vb(2))) / kDeg;
  }
  // Mean over the last half, peaks of pitch - mean.
  double mean = 0;
  for (int i = n / 2; i < n; i++)
    mean += pitch[i];
  mean /= (n - n / 2);
  r.trimPitch = float(mean);
  float peakT[64], peakV[64];
  int np = 0;
  for (int i = 1; i < n - 1 && np < 64; i++) {
    float x0 = pitch[i - 1] - float(mean), x1 = pitch[i] - float(mean), x2 = pitch[i + 1] - float(mean);
    if (x1 > x0 && x1 >= x2 && x1 > 0.05f) {
      peakT[np] = i * s.dt();
      peakV[np] = x1;
      np++;
    }
  }
  if (np >= 3) {
    r.period = (peakT[np > 4 ? 3 : np - 1] - peakT[0]) / float((np > 4 ? 3 : np - 1));
    float delta = logf(peakV[0] / peakV[1]);
    r.zeta = delta / sqrtf(4 * 9.8696f + delta * delta);
  } else if (np == 2) {
    r.period = peakT[1] - peakT[0];
    float delta = logf(peakV[0] / peakV[1]);
    r.zeta = delta / sqrtf(4 * 9.8696f + delta * delta);
  }
  return r;
}

struct StepResult {
  float wDot[3]; ///< mean angular acceleration over the first 0.2 s after the step, rad/s^2
  float tReach;  ///< time until the modelled actuator reached the new angle, s
};
/// At belly-down terminal speed, attitude held for the settle, then released with a flap step at t = 0.
template <class S> StepResult runFlapStep(S &s, const char *scen, float v0, float dTL, float dTR, float dBL,
                                          float dBR) {
  const Quat_F q = bellyDown();
  s.setFlaps(kTopNeutral, kTopNeutral, kBottomNeutral, kBottomNeutral);
  for (int i = 0; i < int(1.0f / s.dt()); i++) {
    s.setState(attOf(q), Vector<float, 6>{0, 0, -v0, 0, 0, 5000});
    s.step();
  }
  // Moment at neutral (to subtract the trim moment): one step from rest.
  s.setState(attOf(q), Vector<float, 6>{0, 0, -v0, 0, 0, 5000});
  s.step();
  Vector<float, 7> a0 = s.att();
  float w0[3] = {a0(0) / s.dt(), a0(1) / s.dt(), a0(2) / s.dt()};
  s.setState(attOf(q), Vector<float, 6>{0, 0, -v0, 0, 0, 5000});
  s.setFlaps(kTopNeutral + dTL, kTopNeutral + dTR, kBottomNeutral + dBL, kBottomNeutral + dBR);
  Csv csv(s.name(), scen);
  StepResult r{{0, 0, 0}, -1};
  const float T = 0.2f;
  int n = int(T / s.dt() + 0.5f);
  for (int i = 0; i < n; i++) {
    s.step();
    if (r.tReach < 0 && fabsf(s.flap(0) - (kTopNeutral + dTL)) < 0.01f * kDeg &&
        fabsf(s.flap(2) - (kBottomNeutral + dBL)) < 0.01f * kDeg)
      r.tReach = (i + 1) * s.dt();
    csv.row(i * s.dt(), s);
  }
  auto a = s.att();
  for (int k = 0; k < 3; k++)
    r.wDot[k] = a(k) / T - w0[k]; // mean acceleration minus the neutral (trim) acceleration
  return r;
}

struct FlipResult {
  float tUpright; ///< time until tilt < 15 deg, s (-1 = not reached)
  float peakRate; ///< max |body pitch rate|, deg/s
  float tiltEnd;  ///< tilt at the end, deg
  float minTilt;  ///< minimum tilt reached, deg
};
/// From belly-down at terminal speed: full thrust with full vane deflection toward upright for `seconds`.
template <class S> FlipResult runFlip(S &s, const char *scen, float v0, float thrust, float seconds) {
  s.setFlaps(kTopNeutral, kTopNeutral, kBottomNeutral, kBottomNeutral);
  for (int i = 0; i < int(1.0f / s.dt()); i++) {
    s.setState(attOf(bellyDown()), Vector<float, 6>{0, 0, -v0, 0, 0, 5000});
    s.step();
  }
  s.setState(attOf(bellyDown()), Vector<float, 6>{0, 0, -v0, 0, 0, 5000});
  // Upright from belly-down = rotate about body +Y by +90 deg (nose up): force toward -X at the tail
  // (tail at z < 0, r x F with F = -Fx: tau_y = z * Fx = (-0.35)(-) > 0).
  const float a = 3 * kDeg;
  s.setThrust(Vector_F{-sinf(a) * thrust, 0, cosf(a) * thrust}, true);
  Csv csv(s.name(), scen);
  FlipResult r{-1, 0, 0, 180};
  int n = int(seconds / s.dt() + 0.5f);
  for (int i = 0; i < n; i++) {
    s.step();
    auto at = s.att();
    float t = tiltDeg(at);
    float rate = fabsf(at(1)) / kDeg;
    r.peakRate = rate > r.peakRate ? rate : r.peakRate;
    r.minTilt = t < r.minTilt ? t : r.minTilt;
    if (r.tUpright < 0 && t < 15)
      r.tUpright = (i + 1) * s.dt();
    if (i % 5 == 0)
      csv.row(i * s.dt(), s);
  }
  r.tiltEnd = tiltDeg(s.att());
  s.setThrust(Vector_F{0, 0, 0}, false);
  return r;
}

struct VaneResult {
  float tReach;   ///< modelled thrust angle reaches 3 deg, s
  float wDotY;    ///< initial pitch angular acceleration at full deflection, rad/s^2
};
template <class S> VaneResult runVaneStep(S &s) {
  s.setState(attOf(Quat_F(1, 0, 0, 0)), Vector<float, 6>{0, 0, 0, 0, 0, 10});
  s.setFlaps(90 * kDeg, 90 * kDeg, 90 * kDeg, 90 * kDeg);
  s.setThrust(Vector_F{0, 0, 1.1f * kG}, true);
  for (int i = 0; i < int(0.5f / s.dt()); i++)
    s.step();
  const float a = 3 * kDeg, T = 1.1f * kG;
  s.setThrust(Vector_F{sinf(a) * T, 0, cosf(a) * T}, true);
  VaneResult r{-1, 0};
  Csv csv(s.name(), "vane_step");
  float wPrev = s.att()(1);
  for (int i = 0; i < int(0.5f / s.dt()); i++) {
    s.step();
    auto at = s.att();
    float wy = at(1);
    if (r.tReach < 0 && fabsf(wy - wPrev) / s.dt() > 0.98f * fabsf(T * sinf(a) * 0.35f / 0.044f))
      r.tReach = (i + 1) * s.dt();
    wPrev = wy;
    csv.row(i * s.dt(), s);
  }
  r.wDotY = T * sinf(a) * 0.35f / 0.044f; // full-deflection value for reference
  return r;
}

struct AlignResult {
  float tBelly;    ///< time until the belly-down angle < 20 deg, s (-1 = never)
  float bellyEnd;  ///< belly-down angle at the end, deg
  float tiltEnd;   ///< tilt at the end, deg
  float vEnd;      ///< vertical speed at the end, m/s (positive down)
};
/// Free fall from rest, upright with a small tilt, flaps fixed at neutral, no thrust.
template <class S> AlignResult runAlign(S &s, const char *scen, float tilt0, float seconds, float roll0 = 0) {
  // Tilt about body +Y by tilt0 (positive: nose toward +X, the back), then roll0 about the body axis.
  s.setState(attOf(Quat_F(Vector_F{0, 1, 0}, tilt0 * kDeg) * Quat_F(Vector_F{0, 0, 1}, roll0 * kDeg)),
             Vector<float, 6>{0, 0, 0, 0, 0, 5000});
  s.setFlaps(kTopNeutral, kTopNeutral, kBottomNeutral, kBottomNeutral);
  s.setThrust(Vector_F{0, 0, 0}, false);
  Csv csv(s.name(), scen);
  AlignResult r{-1, 0, 0, 0};
  int n = int(seconds / s.dt() + 0.5f);
  for (int i = 0; i < n; i++) {
    s.step();
    if (r.tBelly < 0 && bellyDeg(s.att()) < 20)
      r.tBelly = (i + 1) * s.dt();
    if (i % 5 == 0)
      csv.row(i * s.dt(), s);
  }
  r.bellyEnd = bellyDeg(s.att());
  r.tiltEnd = tiltDeg(s.att());
  r.vEnd = -s.pos()(2);
  return r;
}

// Replica of ControlAttitudeFlaps (Stabilize, lib/ExVectrControl control_attitude_flaps.cpp, gains of 6e07d38) for a
// closed-loop belly-down hold check in the harness. Runs at 50 Hz like the task. Target: pitch 0, azimuth of the
// start nose direction.
struct FlapCtrlReplica {
  float azSet = 0, pitchSet = 0;
  float out[4] = {kTopNeutral, kTopNeutral, kBottomNeutral, kBottomNeutral};
  void update(const Vector<float, 7> &a) {
    Vector_F x = axisWorld(a, 1, 0, 0), z = axisWorld(a, 0, 0, 1);
    float belly = acosf(x(2) > 1 ? 1 : (x(2) < -1 ? -1 : x(2)));
    float ff = (belly - 35 * kDeg) / (20 * kDeg);
    ff = ff < 0 ? 0 : (ff > 1 ? 1 : ff);
    ff = 1 - ff;
    float az = atan2f(z(1), z(0)), pitch = atan2f(z(2), sqrtf(z(0) * z(0) + z(1) * z(1)));
    float ea = azSet - az;
    if (ea > 3.14159265f)
      ea -= 2 * 3.14159265f;
    else if (ea < -3.14159265f)
      ea += 2 * 3.14159265f;
    float u0 = ea * 0.3f, u1 = (pitchSet - pitch) * 1.2f;
    u0 = u0 > 0.5f ? 0.5f : (u0 < -0.5f ? -0.5f : u0);
    u0 -= a(0) * 0.2f;
    u1 -= a(1) * 0.4f;
    float u2 = -a(2) * 0.1f;
    out[0] = kTopNeutral + (-u0 - u1 + u2) * ff;
    out[1] = kTopNeutral + (u0 - u1 - u2) * ff;
    out[2] = kBottomNeutral + (u0 + u1 + u2) * ff;
    out[3] = kBottomNeutral + (-u0 + u1 - u2) * ff;
    for (int i = 0; i < 4; i++) // StarshipFlaps clamp 15..90 deg
      out[i] = out[i] < 15 * kDeg ? 15 * kDeg : (out[i] > 90 * kDeg ? 90 * kDeg : out[i]);
  }
};
struct ClosedResult {
  float pitchMean, pitchStd, bellyMax, azErrEnd, vEnd;
  float tAlign = -1;      ///< time from which |azimuth error| stays below 5 deg to the end, s (-1 = never)
  float azErrLateMax = 0; ///< max |azimuth error| over the second half, deg
};
template <class S> ClosedResult runClosedLoop(S &s, const char *scen, float pitch0, float yaw0Deg, float v0,
                                              float seconds) {
  // yaw0Deg: initial nose azimuth error (rotation about world up)
  Quat_F q = Quat_F(Vector_F{0, 0, 1}, yaw0Deg * kDeg) * bellyDown(pitch0);
  s.setState(attOf(q), Vector<float, 6>{0, 0, -v0, 0, 0, 5000});
  FlapCtrlReplica c;
  c.azSet = atan2f(axisWorld(attOf(bellyDown()), 0, 0, 1)(1), axisWorld(attOf(bellyDown()), 0, 0, 1)(0));
  Csv csv(s.name(), scen);
  int n = int(seconds / s.dt() + 0.5f), every = int(0.02f / s.dt() + 0.5f);
  double sum = 0, sum2 = 0;
  int cnt = 0;
  ClosedResult r{0, 0, 0, 0, 0};
  float lastOutside = 0;
  for (int i = 0; i < n; i++) {
    if (i % every == 0) {
      c.update(s.att());
      s.setFlaps(c.out[0], c.out[1], c.out[2], c.out[3]);
    }
    s.step();
    auto a = s.att();
    {
      Vector_F zz = axisWorld(a, 0, 0, 1);
      float e = (c.azSet - atan2f(zz(1), zz(0))) / kDeg;
      e = e > 180 ? e - 360 : (e < -180 ? e + 360 : e);
      if (fabsf(e) >= 5.0f)
        lastOutside = (i + 1) * s.dt();
      if (i > n / 2 && fabsf(e) > r.azErrLateMax)
        r.azErrLateMax = fabsf(e);
    }
    float b = bellyDeg(a);
    r.bellyMax = (i * s.dt() > 1.0f && b > r.bellyMax) ? b : r.bellyMax;
    if (i > n / 2) {
      float p = pitchDeg(a);
      sum += p;
      sum2 += p * p;
      cnt++;
    }
    if (i % 5 == 0)
      csv.row(i * s.dt(), s);
  }
  r.pitchMean = float(sum / cnt);
  r.pitchStd = float(sqrt(fabs(sum2 / cnt - (sum / cnt) * (sum / cnt))));
  Vector_F z = axisWorld(s.att(), 0, 0, 1);
  float ea = (c.azSet - atan2f(z(1), z(0))) / kDeg;
  r.azErrEnd = ea > 180 ? ea - 360 : (ea < -180 ? ea + 360 : ea);
  r.vEnd = -s.pos()(2);
  r.tAlign = lastOutside < seconds - 1.0f ? lastOutside : -1.0f;
  return r;
}

// ---------------------------------------------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------------------------------------------
static float gNewTerminal = 0, gOldTerminal = 0;

static void test_hover_balance() {
  NewSim n;
  OldSim o;
  HoverResult rn = runHover(n, 5.0f), ro = runHover(o, 5.0f);
  printf("SIMAERO hover 5 s (T = m g)    | old: vU %.4f m/s dz %.4f m tilt %.3f deg | new: vU %.4f m/s dz %.4f m tilt %.3f deg\n",
         ro.vU, ro.dz, ro.tilt, rn.vU, rn.dz, rn.tilt);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, rn.vU);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, rn.dz);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, rn.tilt);
}

static void test_terminal_speed_belly_down() {
  NewSim n;
  OldSim o;
  gNewTerminal = runTerminalHeld(n, 20.0f, kTopNeutral, kBottomNeutral);
  gOldTerminal = runTerminalHeld(o, 20.0f, kTopNeutral, kBottomNeutral);
  NewSim n2, n3;
  float vRet = runTerminalHeld(n2, 20.0f, 90 * kDeg, 90 * kDeg);
  float vExt = runTerminalHeld(n3, 20.0f, 15 * kDeg, 15 * kDeg);
  OldSim o2, o3;
  float voRet = runTerminalHeld(o2, 20.0f, 90 * kDeg, 90 * kDeg);
  float voExt = runTerminalHeld(o3, 20.0f, 15 * kDeg, 15 * kDeg);
  printf("SIMAERO terminal belly-down held, flaps neutral 50/40 | old %.2f m/s | new %.2f m/s (target 6.4)\n",
         gOldTerminal, gNewTerminal);
  printf("SIMAERO terminal belly-down held, flaps folded 90/90  | old %.2f m/s | new %.2f m/s\n", voRet, vRet);
  printf("SIMAERO terminal belly-down held, flaps out 15/15     | old %.2f m/s | new %.2f m/s\n", voExt, vExt);
  TEST_ASSERT_FLOAT_WITHIN(0.03f * 6.4f, 6.4f, gNewTerminal);
  TEST_ASSERT_TRUE(vRet > gNewTerminal && vExt < gNewTerminal); // more flap area, slower fall
}

static void test_belly_down_free_oscillation() {
  NewSim n;
  OldSim o;
  float vn = gNewTerminal > 0 ? gNewTerminal : 6.4f;
  float vo = gOldTerminal > 0 ? gOldTerminal : 15.0f;
  OscResult rn = runBellyFree(n, "belly_free_pitch10", vn, 10, 0, 15.0f, kTopNeutral, kBottomNeutral);
  OscResult ro = runBellyFree(o, "belly_free_pitch10", vo, 10, 0, 15.0f, kTopNeutral, kBottomNeutral);
  printf("SIMAERO belly-down free, +10 deg pitch, flaps neutral | old: trim pitch %.1f deg, period %.2f s, zeta %.3f, belly max %.0f end %.0f deg, v %.2f, glide %.2f m/s, AoA %.1f deg\n",
         ro.trimPitch, ro.period, ro.zeta, ro.bellyMax, ro.bellyEnd, ro.vEnd, ro.vHorEnd, ro.aoaEnd);
  printf("SIMAERO                                               | new: trim pitch %.1f deg, period %.2f s, zeta %.3f, belly max %.0f end %.0f deg, v %.2f, glide %.2f m/s, AoA %.1f deg\n",
         rn.trimPitch, rn.period, rn.zeta, rn.bellyMax, rn.bellyEnd, rn.vEnd, rn.vHorEnd, rn.aoaEnd);
  NewSim n2;
  OldSim o2;
  OscResult rn2 = runBellyFree(n2, "belly_free_roll20", vn, 0, 20, 15.0f, kTopNeutral, kBottomNeutral);
  OscResult ro2 = runBellyFree(o2, "belly_free_roll20", vo, 0, 20, 15.0f, kTopNeutral, kBottomNeutral);
  printf("SIMAERO belly-down free, 20 deg roll about the axis   | old: belly max %.0f end %.0f deg | new: belly max %.0f end %.0f deg\n",
         ro2.bellyMax, ro2.bellyEnd, rn2.bellyMax, rn2.bellyEnd);
  // Back-down (body +X down, flaps folded toward the flow): is it an equilibrium the vehicle can stay in?
  NewSim n3;
  OldSim o3;
  OscResult rn3 = runBellyFree(n3, "back_down_roll20", vn, 0, 200, 15.0f, kTopNeutral, kBottomNeutral);
  OscResult ro3 = runBellyFree(o3, "back_down_roll20", vo, 0, 200, 15.0f, kTopNeutral, kBottomNeutral);
  printf("SIMAERO back-down free (roll 200 deg about the axis)   | old: belly end %.0f deg | new: belly end %.0f deg (180 = stays back-down)\n",
         ro3.bellyEnd, rn3.bellyEnd);
  // Belly-down must be statically stable (no tumble) in the new model.
  TEST_ASSERT_TRUE(rn.bellyEnd < 45.0f);
  TEST_ASSERT_TRUE(rn2.bellyEnd < 40.0f);
  TEST_ASSERT_TRUE(rn3.bellyEnd < 90.0f); // back-down is not stable: it rolls to the belly side
}

static void test_flap_steps() {
  float vn = gNewTerminal > 0 ? gNewTerminal : 6.4f;
  float vo = gOldTerminal > 0 ? gOldTerminal : 15.0f;
  struct {
    const char *name;
    float d[4];
  } steps[] = {
      {"pitch_up_top_out_10", {-10 * kDeg, -10 * kDeg, 0, 0}},
      {"pitch_up_u1_10", {-10 * kDeg, -10 * kDeg, 10 * kDeg, 10 * kDeg}},
      {"azimuth_u0_10", {-10 * kDeg, 10 * kDeg, 10 * kDeg, -10 * kDeg}},
      {"roll_u2_10", {10 * kDeg, -10 * kDeg, 10 * kDeg, -10 * kDeg}},
  };
  for (auto &st : steps) {
    NewSim n;
    OldSim o;
    StepResult rn = runFlapStep(n, st.name, vn, st.d[0], st.d[1], st.d[2], st.d[3]);
    StepResult ro = runFlapStep(o, st.name, vo, st.d[0], st.d[1], st.d[2], st.d[3]);
    printf("SIMAERO flap step %-20s | old: dw/dt [%.2f %.2f %.2f] rad/s2, reach %.3f s | new: dw/dt [%.2f %.2f %.2f] rad/s2, reach %.3f s\n",
           st.name, ro.wDot[0], ro.wDot[1], ro.wDot[2], ro.tReach, rn.wDot[0], rn.wDot[1], rn.wDot[2], rn.tReach);
    if (st.d[0] < 0 && st.d[1] < 0) // top flaps out -> nose up (+ about body Y)
      TEST_ASSERT_TRUE(rn.wDot[1] > 0);
    if (strcmp(st.name, "roll_u2_10") == 0) // u2 > 0 -> + torque about body Z (controller damping sign)
      TEST_ASSERT_TRUE(rn.wDot[2] > 0);
  }
  // Servo model: 10 deg at 300 deg/s = 33 ms (+ one step).
  NewSim n;
  StepResult r = runFlapStep(n, "servo_check", vn, -10 * kDeg, -10 * kDeg, 10 * kDeg, 10 * kDeg);
  TEST_ASSERT_FLOAT_WITHIN(0.004f, 10.0f / 300.0f, r.tReach);
}

static void test_static_flap_authority_table() {
  // Moments at belly-down terminal speed (attitude held) vs flap angle: the azimuth (tau_x), pitch (tau_y) and
  // roll (tau_z) derivatives of the controller's mixer directions u0, u1, u2 (+1 deg each).
  BodySimModel m;
  const float v = gNewTerminal > 0 ? gNewTerminal : 6.4f;
  printf("SIMAERO static moments at belly-down %.2f m/s, per +1 deg of u (N m/deg): neutral top/bottom | d tau/du0 (x) | d tau/du1 (y) | d tau/du2 (z) | tau at neutral\n", v);
  float tops[] = {50, 30, 40, 50, 60, 70};
  float bots[] = {40, 30, 40, 50, 60, 70};
  for (int k = 0; k < 6; k++) {
    float t = tops[k] * kDeg, b = bots[k] * kDeg;
    auto torque = [&](float tl, float tr, float bl, float br) {
      BodySimParams p;
      p.applyActuatorRateLimits = false;
      BodySimModel mm(p);
      mm.setAttitudeState(attOf(bellyDown()));
      mm.setPositionState({0, 0, -v, 0, 0, 100});
      mm.setFlapInput(tl, tr, bl, br);
      mm.step(); // applies the angles (no rate limit), computes the loads
      mm.setAttitudeState(attOf(bellyDown()));
      mm.setPositionState({0, 0, -v, 0, 0, 100});
      mm.computeAero();
      return mm.loads().aeroTorque;
    };
    const float d = 1 * kDeg;
    auto t0 = torque(t, t, b, b);
    auto tu0 = torque(t - d, t + d, b + d, b - d);
    auto tu1 = torque(t - d, t - d, b + d, b + d);
    auto tu2 = torque(t + d, t - d, b + d, b - d);
    printf("SIMAERO   %2.0f/%2.0f deg | %+.5f | %+.5f | %+.5f | [%+.4f %+.4f %+.4f]\n", tops[k], bots[k],
           tu0(0) - t0(0), tu1(1) - t0(1), tu2(2) - t0(2), t0(0), t0(1), t0(2));
  }
  (void)m;
  // Information for the owner: static pitch moment at belly-down vs the neutral angles (zero = trimmed belly-down).
  {
    auto tauY = [&](float tDeg, float bDeg) {
      BodySimParams p;
      p.applyActuatorRateLimits = false;
      BodySimModel mm(p);
      mm.setFlapInput(tDeg * kDeg, tDeg * kDeg, bDeg * kDeg, bDeg * kDeg);
      mm.step();
      mm.setAttitudeState(attOf(bellyDown()));
      mm.setPositionState({0, 0, -v, 0, 0, 100});
      mm.computeAero();
      return mm.loads().aeroTorque(1);
    };
    float bZero = -1, tZero = -1;
    for (float b = 0; b <= 90; b += 0.5f)
      if (bZero < 0 && tauY(50, b) * tauY(50, b + 0.5f) <= 0 && b < 89.9f)
        bZero = b;
    for (float t = 0; t <= 90; t += 0.5f)
      if (tZero < 0 && tauY(t, 40) * tauY(t + 0.5f, 40) <= 0 && t < 89.9f)
        tZero = t;
    printf("SIMAERO belly-down tau_y (N m): top50 bottom 0/40/90 = %+.3f %+.3f %+.3f; bottom40 top 0/50/90 = %+.3f %+.3f %+.3f\n",
           tauY(50, 0), tauY(50, 40), tauY(50, 90), tauY(0, 40), tauY(50, 40), tauY(90, 40));
    printf("SIMAERO belly-down pitch trim: top 50 deg -> bottom flaps %.1f deg; bottom 40 deg -> top flaps %.1f deg (-1 = none in 0..90)\n",
           bZero, tZero);
  }
  // Flat plate coefficient sanity: 0 at 0, C_N90 at 90 deg, positive in between.
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, m.flapCn(0));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, m.params().flapCn90, m.flapCn(90 * kDeg));
  for (int a = 1; a < 90; a++)
    TEST_ASSERT_TRUE(m.flapCn(a * kDeg) > 0);
}

// Mixer of ControlAttitudeFlaps (control_attitude_flaps.cpp, Stabilize, verbatim): u = attRateCtrlOutput, ff = 1.
static void flapMixer(float u0, float u1, float u2, float out[4]) {
  out[0] = kTopNeutral + (-u0 - u1 + u2);
  out[1] = kTopNeutral + (+u0 - u1 - u2);
  out[2] = kBottomNeutral + (+u0 + u1 + u2);
  out[3] = kBottomNeutral + (-u0 + u1 - u2);
}

static void test_control_axis_signs() {
  // Intended (controller): u0 > 0 (azimuth error > 0, i.e. turn the nose counter-clockwise seen from above = about
  // world up = body +X when belly-down) -> tau_x > 0; u1 > 0 (nose too low) -> tau_y > 0 (nose toward body +X = up);
  // u2 > 0 -> tau_z > 0 (the -wz rate term damps). Static torque per +1 deg of u, falling straight down at the
  // terminal speed, body pitched by the angle of attack offset (nose up > 0). The legacy replica gets the same
  // inputs (one explicit step from rest: dw = tau / I dt).
  const float v = gNewTerminal > 0 ? gNewTerminal : 6.4f, vo = gOldTerminal > 0 ? gOldTerminal : 15.0f;
  const float d = 1 * kDeg;
  // Variants: the default model (fold toward the back, fold shield 0.7 since 2026-11-21), the same without the fold
  // shield (documents the reversal it fixes) and the fold toward the belly without shield (the rejected alternative).
  auto newTau = [&](bool belly, const float a[4], float pDeg, bool shield = true) {
    BodySimParams p;
    p.applyActuatorRateLimits = false;
    p.flapFoldTowardBelly = belly;
    if (!shield || belly)
      p.flapFoldShieldFactor = 0.0f;
    BodySimModel mm(p);
    mm.setFlapInput(a[0], a[1], a[2], a[3]);
    mm.step();
    mm.setAttitudeState(attOf(bellyDown(pDeg)));
    mm.setPositionState({0, 0, -v, 0, 0, 100});
    mm.computeAero();
    return mm.loads().aeroTorque;
  };
  auto oldTau = [&](const float a[4], float pDeg) {
    legacy::LegacyModel lm;
    for (int i = 0; i < 4; i++)
      lm.flaps[i] = a[i];
    lm.att = attOf(bellyDown(pDeg));
    lm.pos = {0, 0, -vo, 0, 0, 100};
    const float h = 1e-3f;
    lm.step(h);
    return Vector_F{lm.att(0) * lm.I[0] / h, lm.att(1) * lm.I[1] / h, lm.att(2) * lm.I[2] / h};
  };
  printf("SIMAERO control axis signs (intended: all +) | d tau_x/du0, d tau_y/du1, d tau_z/du2 in N m/deg at neutral 50/40\n");
  float n0[4], a0[4], a1[4], a2[4];
  flapMixer(0, 0, 0, n0);
  flapMixer(d, 0, 0, a0);
  flapMixer(0, d, 0, a1);
  flapMixer(0, 0, d, a2);
  for (float pDeg : {-20.0f, -10.0f, 0.0f, 10.0f, 20.0f}) {
    const Vector_F ob = oldTau(n0, pDeg), o0 = oldTau(a0, pDeg), o1 = oldTau(a1, pDeg), o2 = oldTau(a2, pDeg);
    const Vector_F bb = newTau(false, n0, pDeg), b0 = newTau(false, a0, pDeg), b1 = newTau(false, a1, pDeg),
                   b2 = newTau(false, a2, pDeg);
    const Vector_F fb = newTau(true, n0, pDeg), f0 = newTau(true, a0, pDeg), f1 = newTau(true, a1, pDeg),
                   f2 = newTau(true, a2, pDeg);
    const Vector_F nb = newTau(false, n0, pDeg, false), x0 = newTau(false, a0, pDeg, false);
    printf("SIMAERO   AoA %+3.0f | old [%+.5f %+.5f %+.5f] | new DEFAULT (fold->back, shield 0.7) [%+.5f %+.5f %+.5f] | no shield yaw %+.5f | fold->belly [%+.5f %+.5f %+.5f]\n",
           pDeg, o0(0) - ob(0), o1(1) - ob(1), o2(2) - ob(2), b0(0) - bb(0), b1(1) - bb(1), b2(2) - bb(2),
           x0(0) - nb(0), f0(0) - fb(0), f1(1) - fb(1), f2(2) - fb(2));
    // The default model has the controller's sign on all three axes at every AoA -20..+20 (2026-11-21: fold
    // shield on). Without the shield the yaw sign is reversed (fold toward the back).
    TEST_ASSERT_TRUE(b0(0) - bb(0) > 0);
    TEST_ASSERT_TRUE(b1(1) - bb(1) > 0 && f1(1) - fb(1) > 0);
    TEST_ASSERT_TRUE(b2(2) - bb(2) > 0 && f2(2) - fb(2) > 0);
    TEST_ASSERT_TRUE(f0(0) - fb(0) > 0);
    TEST_ASSERT_TRUE(x0(0) - nb(0) < 0); // documents the reversal without the fold shield
    TEST_ASSERT_TRUE(o0(0) - ob(0) > 0 && o1(1) - ob(1) > 0 && o2(2) - ob(2) > 0);
  }
  // Per-flap derivation at belly-down, fold toward the back: sideways force and its tau_x when each flap extends 1 deg.
  for (int i = 0; i < 4; i++) {
    BodySimParams p;
    p.applyActuatorRateLimits = false;
    float a[4];
    flapMixer(0, 0, 0, a);
    BodySimModel m0(p), m1(p);
    m0.setFlapInput(a[0], a[1], a[2], a[3]);
    a[i] -= d;
    m1.setFlapInput(a[0], a[1], a[2], a[3]);
    BodySimModel *ms[2] = {&m0, &m1};
    for (auto *m : ms) {
      m->step();
      m->setAttitudeState(attOf(bellyDown()));
      m->setPositionState({0, 0, -v, 0, 0, 100});
      m->computeAero();
    }
    const char *nm[4] = {"TL", "TR", "BL", "BR"};
    printf("SIMAERO   %s extend 1 deg (fold->back): F [%+.3f %+.3f %+.3f] N, dFy %+.4f N, dtau_x %+.5f N m\n", nm[i],
           m0.loads().flapForce[i](0), m0.loads().flapForce[i](1), m0.loads().flapForce[i](2),
           m1.loads().flapForce[i](1) - m0.loads().flapForce[i](1),
           m1.loads().aeroTorque(0) - m0.loads().aeroTorque(0));
  }
  // Closed loop with the controller replica and the fold toward the belly (information for the owner).
  {
    BodySimParams p;
    p.flapFoldTowardBelly = true;
    p.flapFoldShieldFactor = 0.0f;
    NewSim n(p);
    ClosedResult rb = runClosedLoop(n, "closed_pitch20_yaw20_foldbelly", 20, 20, v, 15.0f);
    printf("SIMAERO flap controller replica, fold toward the belly, start pitch +20 yaw +20 deg: pitch %.1f +- %.1f deg, belly max %.0f, az err end %.1f deg, v %.2f\n",
           rb.pitchMean, rb.pitchStd, rb.bellyMax, rb.azErrEnd, rb.vEnd);
  }
}

// Static torque derivatives (N m/deg) of the controller's mixer directions u0, u1, u2 around arbitrary neutral
// angles (top t, bottom b, rad; evaluated at max(1, min(89 deg)) so the +-1 deg central difference stays inside the
// 0..90 deg travel), falling straight down at speed v, body pitched by pDeg.
struct AxisDeriv {
  float yaw, pitch, roll; // d tau_x/du0, d tau_y/du1, d tau_z/du2
};
static AxisDeriv axisDerivs(const BodySimParams &base, float t, float b, float pDeg, float v) {
  const float lo = 1 * kDeg, hi = 89 * kDeg, d = 1 * kDeg;
  t = t < lo ? lo : (t > hi ? hi : t);
  b = b < lo ? lo : (b > hi ? hi : b);
  auto tau = [&](float u0, float u1, float u2) {
    BodySimParams p = base;
    p.applyActuatorRateLimits = false;
    BodySimModel mm(p);
    mm.setFlapInput(t + (-u0 - u1 + u2), t + (u0 - u1 - u2), b + (u0 + u1 + u2), b + (-u0 + u1 - u2));
    mm.step();
    mm.setAttitudeState(attOf(bellyDown(pDeg)));
    mm.setPositionState({0, 0, -v, 0, 0, 100});
    mm.computeAero();
    return mm.loads().aeroTorque;
  };
  const Vector_F x0p = tau(d, 0, 0), x0m = tau(-d, 0, 0), x1p = tau(0, d, 0), x1m = tau(0, -d, 0),
                 x2p = tau(0, 0, d), x2m = tau(0, 0, -d);
  return {0.5f * (x0p(0) - x0m(0)), 0.5f * (x1p(1) - x1m(1)), 0.5f * (x2p(2) - x2m(2))};
}

static void test_neutral_sweep_effective_angle() {
  // Report only (owner question 2026-11-21): where does the yaw (azimuth) response have the controller's sign
  // (+) over the neutral-angle range, with the effective aerodynamic flap angle (a_eff = smoothmax(a, 20 deg))?
  const float v = gNewTerminal > 0 ? gNewTerminal : 6.4f;
  BodySimParams p;
  const float aoas[5] = {-20, -10, 0, 10, 20};
  printf("SIMAERO a_eff check: a = 0/10/20/40/50/90 deg -> a_eff = %.2f %.2f %.2f %.2f %.2f %.2f deg\n",
         BodySimModel(p).flapAeroAngle(0) / kDeg, BodySimModel(p).flapAeroAngle(10 * kDeg) / kDeg,
         BodySimModel(p).flapAeroAngle(20 * kDeg) / kDeg, BodySimModel(p).flapAeroAngle(40 * kDeg) / kDeg,
         BodySimModel(p).flapAeroAngle(50 * kDeg) / kDeg, BodySimModel(p).flapAeroAngle(90 * kDeg) / kDeg);
  const char *names[3] = {"yaw d tau_x/du0", "pitch d tau_y/du1", "roll d tau_z/du2"};
  static AxisDeriv grid[10][10][5];
  for (int i = 0; i < 10; i++)
    for (int j = 0; j < 10; j++)
      for (int k = 0; k < 5; k++)
        grid[i][j][k] = axisDerivs(p, i * 10 * kDeg, j * 10 * kDeg, aoas[k], v);
  for (int ax = 0; ax < 3; ax++) {
    printf("SIMAERO sweep %s at AoA 0, 1e-4 N m/deg; rows top 0..90, cols bottom 0..90; '*' = + at all AoA -20..+20\n",
           names[ax]);
    for (int i = 0; i < 10; i++) {
      char line[400];
      int n = snprintf(line, sizeof line, "SIMAERO   top %2d |", i * 10);
      for (int j = 0; j < 10; j++) {
        bool allPos = true;
        float val0 = 0;
        for (int k = 0; k < 5; k++) {
          const AxisDeriv &g = grid[i][j][k];
          const float val = ax == 0 ? g.yaw : (ax == 1 ? g.pitch : g.roll);
          allPos = allPos && val > 0;
          if (k == 2)
            val0 = val;
        }
        n += snprintf(line + n, sizeof line - n, " %+6.0f%c", val0 * 1e4f, allPos ? '*' : ' ');
      }
      printf("%s\n", line);
    }
  }
  printf("SIMAERO yaw d tau_x/du0 at neutral 50/40 vs AoA -20/-10/0/+10/+20: %+.5f %+.5f %+.5f %+.5f %+.5f N m/deg\n",
         axisDerivs(p, 50 * kDeg, 40 * kDeg, -20, v).yaw, axisDerivs(p, 50 * kDeg, 40 * kDeg, -10, v).yaw,
         axisDerivs(p, 50 * kDeg, 40 * kDeg, 0, v).yaw, axisDerivs(p, 50 * kDeg, 40 * kDeg, 10, v).yaw,
         axisDerivs(p, 50 * kDeg, 40 * kDeg, 20, v).yaw);
}

static void test_candidate_effects_yaw_sign() {
  // Report only: candidate flap/body interaction effects, one at a time, and their effect on the yaw sign at the
  // neutral 50/40 deg, on pitch/roll authority and on the terminal speed (fixed calibration factor).
  const float v = gNewTerminal > 0 ? gNewTerminal : 6.4f;
  struct Cand {
    const char *name;
    float side, cross, shield;
  } cands[] = {
      {"baseline (a_eff only)", 0, 0, 0},  {"(a) sidewash +0.5", 0.5f, 0, 0},  {"(a) sidewash +1", 1, 0, 0},
      {"(a) sidewash +2", 2, 0, 0},        {"(a) sidewash -0.5", -0.5f, 0, 0}, {"(a) sidewash -1", -1, 0, 0},
      {"(a) sidewash -2", -2, 0, 0},       {"(b) crossflow 0.5", 0, 0.5f, 0},  {"(b) crossflow 1", 0, 1, 0},
      {"(c) shield 0.3", 0, 0, 0.3f},      {"(c) shield 0.5", 0, 0, 0.5f},     {"(c) shield 0.7", 0, 0, 0.7f},
      {"(c) shield 0.9", 0, 0, 0.9f},
  };
  printf("SIMAERO candidates at neutral 50/40 (N m/deg) | yaw AoA -20/-10/0/+10/+20 | pitch AoA0 | roll AoA0 | terminal m/s (factor fixed)\n");
  for (auto &c : cands) {
    BodySimParams p;
    p.flapSidewashGain = c.side;
    p.flapBodyCrossflowGain = c.cross;
    p.flapFoldShieldFactor = c.shield;
    float y[5];
    const float aoas[5] = {-20, -10, 0, 10, 20};
    for (int k = 0; k < 5; k++)
      y[k] = axisDerivs(p, kTopNeutral, kBottomNeutral, aoas[k], v).yaw;
    const AxisDeriv d0 = axisDerivs(p, kTopNeutral, kBottomNeutral, 0, v);
    NewSim s(p);
    const float vt = runTerminalHeld(s, 20.0f, kTopNeutral, kBottomNeutral);
    printf("SIMAERO   %-22s | %+.5f %+.5f %+.5f %+.5f %+.5f | %+.5f | %+.5f | %.2f\n", c.name, y[0], y[1], y[2], y[3],
           y[4], d0.pitch, d0.roll, vt);
  }
  // The smallest shield factor that restores the sign at every AoA (0.7), recalibrated to 6.4 m/s, in the
  // flap-controller replica (information for the owner; v ~ 1/sqrt(factor), two fixed-point iterations).
  {
    BodySimParams p;
    p.flapFoldShieldFactor = 0.7f;
    for (int it = 0; it < 3; it++) {
      NewSim s(p);
      const float vt = runTerminalHeld(s, 20.0f, kTopNeutral, kBottomNeutral);
      p.aeroCalibration *= (vt / 6.4f) * (vt / 6.4f);
    }
    NewSim s(p);
    const float vt = runTerminalHeld(s, 20.0f, kTopNeutral, kBottomNeutral);
    const AxisDeriv d0 = axisDerivs(p, kTopNeutral, kBottomNeutral, 0, vt);
    NewSim sc(p);
    ClosedResult r = runClosedLoop(sc, "closed_pitch20_yaw20_shield07", 20, 20, vt, 15.0f);
    printf("SIMAERO (c) shield 0.7 recalibrated: factor %.3f, terminal %.2f m/s, yaw/pitch/roll at AoA0 %+.5f %+.5f %+.5f N m/deg; replica start pitch +20 yaw +20: pitch %.1f +- %.1f deg, belly max %.0f, az err end %.1f deg, v %.2f\n",
           p.aeroCalibration, vt, d0.yaw, d0.pitch, d0.roll, r.pitchMean, r.pitchStd, r.bellyMax, r.azErrEnd, r.vEnd);
  }
}

static void test_flap_geometry_and_cant() {
  BodySimParams p;
  BodySimModel m(p);
  const float cant = p.topFlapCantRad;
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 10 * kDeg, cant);
  // Bottom flaps: lower edge 1 cm above the ship bottom, centroid about 0.115 m above it (chamfer lowers it a bit).
  Vector_F cb, nb;
  m.flapPlate(2, 0.0f, cb, nb);
  const float zcBottom = cb(2) + p.cgFromBottom_m; // height above the bottom
  printf("SIMAERO bottom flap centroid %.4f m above the bottom (%.4f from the CoM)\n", zcBottom, cb(2));
  TEST_ASSERT_FLOAT_WITHIN(0.012f, 0.115f, zcBottom);
  TEST_ASSERT_TRUE(zcBottom - p.flapBottom.bottomFromShipBottom_m < 0.5f * p.flapBottom.height_m + 1e-4f);
  // Top flap cant: compare the canted flap with an uncanted copy (angle 0 and 90 deg, both sides).
  BodySimParams p0 = p;
  p0.topFlapCantRad = 0.0f;
  BodySimModel m0(p0);
  const float len = p.flapTop.height_m;
  for (int i = 0; i < 2; i++) { // TL (+Y), TR (-Y)
    const float side = i == 0 ? 1.0f : -1.0f;
    Vector_F c, nn, c0, n0;
    m.flapPlate(i, 0.0f, c, nn);
    m0.flapPlate(i, 0.0f, c0, n0);
    // Radius (distance from the axis) at the centroid: moves inward by about L sin(cant) / 2.
    const float inward = side * (c0(1) - c(1));
    const float expect = 0.5f * len * sinf(cant);
    printf("SIMAERO top flap %d cant: centroid inward %.2f mm (L sin(c)/2 = %.2f mm), up %.2f mm\n", i, inward * 1000,
           expect * 1000, (c(2) - c0(2)) * 1000);
    TEST_ASSERT_FLOAT_WITHIN(0.15f * expect, expect, inward);
    // Extended (angle 0): normal stays +X. Folded (90 deg): normal along -+Y tilted by the cant.
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, nn(0));
    Vector_F cf, nf;
    m.flapPlate(i, 90 * kDeg, cf, nf);
    const float tilt = acosf(fabsf(nf(1))); // angle to the Y axis
    TEST_ASSERT_FLOAT_WITHIN(0.1f * kDeg, cant, tilt);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, nf.magnitude());
    // Upper end of the flap is closer to the axis than the lower hinge point (hinge radius).
    TEST_ASSERT_TRUE(fabsf(c(1)) < fabsf(c0(1)));
  }
  // Bottom flaps and cant-free pair unchanged by the cant parameter.
  Vector_F cb0, nb0;
  m0.flapPlate(2, 0.0f, cb0, nb0);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, cb0(1), cb(1));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, cb0(2), cb(2));
  // Flap area unchanged by the cant: the effective-area direction follows the normal, so a canted flap in a
  // +X flow (extended, normal X) still sees the full area: force magnitude equal to the uncanted one.
}

static void test_vane_step() {
  NewSim n;
  OldSim o;
  VaneResult rn = runVaneStep(n), ro = runVaneStep(o);
  printf("SIMAERO vane step 0 -> 3 deg at hover thrust | old: full deflection after %.3f s | new: %.3f s (servo 300 deg/s / gain 15 -> 0.150 s) | full-deflection dw/dt %.2f rad/s2\n",
         ro.tReach, rn.tReach, rn.wDotY);
  TEST_ASSERT_FLOAT_WITHIN(0.006f, 0.150f, rn.tReach);
}

static void test_align_from_small_tilt() {
  NewSim n;
  OldSim o;
  AlignResult rn = runAlign(n, "align_tilt5", 5, 12.0f), ro = runAlign(o, "align_tilt5", 5, 12.0f);
  printf("SIMAERO free fall from rest, upright tilt 5 deg, flaps neutral | old: belly<20 at %.2f s, end belly %.0f tilt %.0f deg v %.2f | new: belly<20 at %.2f s, end belly %.0f tilt %.0f deg v %.2f\n",
         ro.tBelly, ro.bellyEnd, ro.tiltEnd, ro.vEnd, rn.tBelly, rn.bellyEnd, rn.tiltEnd, rn.vEnd);
  NewSim n2;
  OldSim o2;
  AlignResult rn2 = runAlign(n2, "align_tilt-5", -5, 12.0f), ro2 = runAlign(o2, "align_tilt-5", -5, 12.0f);
  printf("SIMAERO free fall from rest, tilt -5 deg (nose toward the belly) | old: belly<20 at %.2f s, end belly %.0f tilt %.0f | new: belly<20 at %.2f s, end belly %.0f tilt %.0f\n",
         ro2.tBelly, ro2.bellyEnd, ro2.tiltEnd, rn2.tBelly, rn2.bellyEnd, rn2.tiltEnd);
  NewSim n3;
  OldSim o3;
  AlignResult rn3 = runAlign(n3, "align_tilt-5_roll5", -5, 20.0f, 5), ro3 = runAlign(o3, "align_tilt-5_roll5", -5, 20.0f, 5);
  printf("SIMAERO free fall from rest, tilt -5 deg + 5 deg roll (asymmetric) | old: end belly %.0f tilt %.0f | new: end belly %.0f tilt %.0f (20 s)\n",
         ro3.bellyEnd, ro3.tiltEnd, rn3.bellyEnd, rn3.tiltEnd);
  TEST_ASSERT_TRUE(rn.tBelly > 0);
  TEST_ASSERT_TRUE(rn.bellyEnd < 30.0f);
  // Not asserted: in the model a gliding back-down fall is also a stable equilibrium once reached (the flaps then
  // run near the stall blend, where the roll stability changes sign). Reported for the owner.
  (void)rn3;
}

static void test_flip_and_kick() {
  float vn = gNewTerminal > 0 ? gNewTerminal : 6.4f;
  float vo = gOldTerminal > 0 ? gOldTerminal : 15.0f;
  NewSim n;
  OldSim o;
  FlipResult rn = runFlip(n, "flip_14N", vn, 14.0f, 3.0f), ro = runFlip(o, "flip_14N", vo, 14.0f, 3.0f);
  printf("SIMAERO flip from belly-down, 14 N, 3 deg vane, 3 s | old: upright(<15) at %.2f s, min tilt %.0f, peak rate %.0f deg/s, end tilt %.0f | new: upright at %.2f s, min tilt %.0f, peak rate %.0f deg/s, end tilt %.0f\n",
         ro.tUpright, ro.minTilt, ro.peakRate, ro.tiltEnd, rn.tUpright, rn.minTilt, rn.peakRate, rn.tiltEnd);
}

// Review 2026-11-21 (test 6): the powered kick at the 11 N cap with the 1.1 kg ship, vertical, flaps retracted: the
// altitude change over the 1 s kick stays small (the kick is not meant to fly, owner).
static void test_kick_cap_altitude_change_1p1_kg() {
  NewSim s;
  TEST_ASSERT_EQUAL_FLOAT(1.1f, s.m.params().mass_kg);
  s.setState(attOf(Quat_F(1, 0, 0, 0)), Vector<float, 6>{0, 0, 0, 0, 0, 10});
  s.setFlaps(90 * kDeg, 90 * kDeg, 90 * kDeg, 90 * kDeg);
  s.setThrust(Vector_F{0, 0, 11.0f}, true);
  const float z0 = 10.0f;
  for (int i = 0; i < int(1.0f / s.dt() + 0.5f); i++)
    s.step();
  const float dz = s.pos()(5) - z0;
  printf("SIMAERO kick cap 11 N, 1.1 kg, vertical, 1 s: altitude change %+.3f m (a bit of climb, no sinking)\n", dz);
  TEST_ASSERT_TRUE(dz > -0.05f && dz < 0.3f);
}

static void test_closed_loop_flap_hold() {
  float vn = gNewTerminal > 0 ? gNewTerminal : 6.4f;
  float vo = gOldTerminal > 0 ? gOldTerminal : 15.0f;
  OldSim o;
  ClosedResult ro = runClosedLoop(o, "closed_pitch20_yaw20", 20, 20, vo, 15.0f);
  printf("SIMAERO flap controller replica (old model), start pitch +20 yaw +20 deg: pitch %.1f +- %.1f deg, belly max %.0f, az err end %.1f deg, v %.2f\n",
         ro.pitchMean, ro.pitchStd, ro.bellyMax, ro.azErrEnd, ro.vEnd);
  // New model (fold shield 0.7 on by default, 2026-11-21), unchanged gains and neutral angles: heading errors of
  // +-20 deg with and without a pitch error, 30 s.
  struct Case {
    const char *scen;
    float pitch0, yaw0;
  } cases[] = {{"closed_pitch20_yaw20", 20, 20}, {"closed_pitch20_yaw-20", 20, -20}, {"closed_pitch0_yaw20", 0, 20},
               {"closed_pitch0_yaw-20", 0, -20}, {"closed_pitch-20_yaw20", -20, 20}};
  bool allHold = true;
  for (auto &c : cases) {
    NewSim n;
    ClosedResult r = runClosedLoop(n, c.scen, c.pitch0, c.yaw0, vn, 30.0f);
    printf("SIMAERO flap controller replica (new model), start pitch %+3.0f yaw %+3.0f deg: pitch %.1f +- %.1f deg, belly max %.0f, az err end %+.1f deg, |az err| < 5 deg from %.1f s, late max %.1f deg, v %.2f\n",
           c.pitch0, c.yaw0, r.pitchMean, r.pitchStd, r.bellyMax, r.azErrEnd, r.tAlign, r.azErrLateMax, r.vEnd);
    allHold = allHold && r.bellyMax < 45.0f && r.tAlign >= 0.0f && r.tAlign < 20.0f && r.azErrLateMax < 5.0f;
  }
  printf("SIMAERO flap controller replica (new model): %s\n", allHold ? "holds belly-down and aligns from +-20 deg in all cases" : "does NOT hold/align in every case");
  // Report only (2026-11-21): with the fold shield the yaw sign is right and the replica holds belly-down (max 32 deg),
  // but the heading settles into a yaw/roll limit cycle of about +-34 deg (period about 4 s) instead of aligning, with
  // the unchanged gains and neutral angles. Not asserted until the controller is retuned (ExVectrDSP.md Known issues).
  (void)allHold;
}

// Owner reproduction (2026-11-21): the last real flight had the top-left flap servo stuck at about 40-45 deg during the
// transition to the descent. Closed loop with the controller replica (current gains, untouched neutral angles) and the
// real SimFlapFaults model between the controller and the aerodynamic model, as StarshipFlaps does it (300 deg/s servo).
// Information only: nothing is asserted about the outcome.
struct FaultRunRow {
  float t, headUnwrapped, roll, pitch, belly, speed, rate;
};
static float rollAboutNoseDeg(const Vector<float, 7> &a) {
  Vector_F z = axisWorld(a, 0, 0, 1), x = axisWorld(a, 1, 0, 0);
  Vector_F up = {0, 0, 1};
  float d = up(0) * z(0) + up(1) * z(1) + up(2) * z(2);
  Vector_F u = {up(0) - d * z(0), up(1) - d * z(1), up(2) - d * z(2)};
  float um = sqrtf(u(0) * u(0) + u(1) * u(1) + u(2) * u(2));
  if (um < 1e-3f)
    return 0;
  float dot = (x(0) * u(0) + x(1) * u(1) + x(2) * u(2)) / um;
  Vector_F cr = {x(1) * u(2) - x(2) * u(1), x(2) * u(0) - x(0) * u(2), x(0) * u(1) - x(1) * u(0)};
  float sg = (cr(0) * z(0) + cr(1) * z(1) + cr(2) * z(2)) / um;
  return atan2f(sg, dot) / kDeg;
}
static void runFaultRepro(bool faulted, float v0, FaultRunRow *rows, int nRows) {
  NewSim n;
  n.setState(attOf(bellyDown(0, 0)), Vector<float, 6>{0, 0, -v0, 0, 0, 5000});
  FlapCtrlReplica c;
  c.azSet = atan2f(axisWorld(attOf(bellyDown()), 0, 0, 1)(1), axisWorld(attOf(bellyDown()), 0, 0, 1)(0));
  VCTR::Safety::SimFlapFaults faults; // TL stuck at 42 deg, applied at t = 1 s (immediate effect, redesign 2026-11-21)
  float servo[4] = {kTopNeutral, kTopNeutral, kBottomNeutral, kBottomNeutral}; // true angles
  const float rate = 60 * kDeg / 0.2f;
  const float dt = n.dt();
  const int total = int(20.0f / dt + 0.5f), every = int(0.02f / dt + 0.5f), perSec = int(1.0f / dt + 0.5f);
  float cmd[4] = {servo[0], servo[1], servo[2], servo[3]};
  float headPrev = c.azSet, headU = 0;
  int row = 0;
  for (int i = 0; i < total; i++) {
    if (i == perSec && faulted)
      faults.set(VCTR::Safety::decideSimFault(16, 0x00, 0x57, 42 * kDeg, true), true);
    if (i % every == 0) {
      c.update(n.att());
      for (int k = 0; k < 4; k++)
        cmd[k] = c.out[k];
      for (int k = 0; k < 4; k++) { // StarshipFlaps servo model + fault hook, 20 ms step
        float diff = cmd[k] - servo[k], mx = rate * faults.rateFactor(uint8_t(k), true) * 0.02f;
        servo[k] = fabsf(diff) <= mx ? cmd[k] : servo[k] + (diff > 0 ? mx : -mx);
        servo[k] = faults.effectiveAngle(uint8_t(k), true, servo[k]);
      }
      n.setFlaps(servo[0], servo[1], servo[2], servo[3]);
    }
    n.step();
    Vector_F z = axisWorld(n.att(), 0, 0, 1);
    float head = atan2f(z(1), z(0)), dh = head - headPrev;
    if (dh > 3.14159265f) dh -= 2 * 3.14159265f; else if (dh < -3.14159265f) dh += 2 * 3.14159265f;
    headU += dh / kDeg;
    headPrev = head;
    if ((i + 1) % (5 * perSec) == 0 && row < nRows) {
      auto a = n.att();
      float w = sqrtf(a(0) * a(0) + a(1) * a(1) + a(2) * a(2)) / kDeg;
      rows[row++] = FaultRunRow{(i + 1) * dt, headU, rollAboutNoseDeg(a), pitchDeg(a), bellyDeg(a), -n.pos()(2), w};
    }
  }
}
static void test_sim_fault_reproduction_tl_stuck() {
  float v = gNewTerminal > 0 ? gNewTerminal : 6.4f;
  FaultRunRow ok[4], bad[4];
  runFaultRepro(false, v, ok, 4);
  runFaultRepro(true, v, bad, 4);
  for (int r = 0; r < 4; r++)
    printf("SIMFAULT t=%2.0f s | unfaulted: heading %+7.1f deg, roll %+6.1f, pitch %+6.1f, belly %5.1f, rate %5.1f deg/s, v %.2f | TL stuck 42 deg: heading %+7.1f, roll %+6.1f, pitch %+6.1f, belly %5.1f, rate %5.1f deg/s, v %.2f\n",
           ok[r].t, ok[r].headUnwrapped, ok[r].roll, ok[r].pitch, ok[r].belly, ok[r].rate, ok[r].speed,
           bad[r].headUnwrapped, bad[r].roll, bad[r].pitch, bad[r].belly, bad[r].rate, bad[r].speed);
  TEST_ASSERT_TRUE(isfinite(bad[3].headUnwrapped) && isfinite(ok[3].headUnwrapped));
}

static void test_wind_frame() {
  // Steady wind 3 m/s toward West (direction 90 deg): a falling body drifts toward West only (no North component),
  // for any yaw; broadside to the wind (yaw 0: nose South) it reaches the wind speed within 20 s.
  for (float yaw : {0.0f, 90.0f, 45.0f}) {
    BodySimParams p;
    p.topFlapCantRad = 0.0f; // the cant tilts the flap normals axially: a crosswind then adds a small axial drift
    p.windSpeed_mps = 3.0f;
    p.windDirection_rad = 90 * kDeg;
    BodySimModel m(p);
    Quat_F q = Quat_F(Vector_F{0, 0, 1}, yaw * kDeg) * bellyDown();
    m.setFlapInput(kTopNeutral, kTopNeutral, kBottomNeutral, kBottomNeutral);
    for (int i = 0; i < int(20.0f / p.step_s); i++) {
      auto ps = m.positionState();
      if (i == 0)
        ps = {0, 0, 0, 0, 0, 5000};
      m.setPositionState(ps);
      m.setAttitudeState(attOf(q));
      m.step();
    }
    auto ps = m.positionState();
    printf("SIMAERO wind 3 m/s toward West, yaw %.0f deg: v = [%.3f %.3f %.3f] m/s\n", yaw, ps(0), ps(1), ps(2));
    if (yaw == 0.0f) {
      TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, ps(0));
      TEST_ASSERT_FLOAT_WITHIN(0.05f, 3.0f, ps(1));
    } else if (yaw == 90.0f) { // wind along the body axis: slower, still straight West
      TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, ps(0));
      TEST_ASSERT_TRUE(ps(1) > 0.5f && ps(1) <= 3.0f);
    } else {
      TEST_ASSERT_TRUE(ps(1) > 0.5f && ps(1) <= 3.01f);
    }
  }
  // Off by default
  BodySimModel m;
  TEST_ASSERT_EQUAL_FLOAT(0.0f, m.windWorld().magnitude());
}

static void test_gusts() {
  // Off by default.
  {
    BodySimModel m;
    float a, c;
    m.gustAt(12.3, a, c);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, a);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, c);
    m.setWind(2.0f, 0.5f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.0f, m.windWorld().magnitude());
  }
  // Clamping and non-finite input.
  {
    BodySimModel m;
    m.setGust(50.0f, 0.01f, 7);
    TEST_ASSERT_EQUAL_FLOAT(10.0f, m.params().gustStrength_mps);
    TEST_ASSERT_EQUAL_FLOAT(0.5f, m.params().gustPeriod_s);
    m.setGust(-1.0f, 100.0f, 7);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, m.params().gustStrength_mps);
    TEST_ASSERT_EQUAL_FLOAT(30.0f, m.params().gustPeriod_s);
    m.setGust(NAN, NAN, 8);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, m.params().gustStrength_mps);
    TEST_ASSERT_EQUAL_FLOAT(30.0f, m.params().gustPeriod_s);
    m.clearWind();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, m.params().gustStrength_mps);
    TEST_ASSERT_EQUAL_FLOAT(5.0f, m.params().gustPeriod_s);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, m.params().windSpeed_mps);
  }
  // Determinism and seed dependence; statistics over 1200 s at 0.1 s sampling.
  BodySimModel a, b, c;
  a.setGust(4.0f, 3.0f, 42);
  b.setGust(4.0f, 3.0f, 42);
  c.setGust(4.0f, 3.0f, 43);
  double sumSq = 0, sumMean = 0;
  float peak = 0;
  bool identical = true, differs = false;
  const int n = 12000;
  for (int i = 0; i < n; i++) {
    const double t = i * 0.1;
    float ga, gc, ha, hc, ia, ic;
    a.gustAt(t, ga, gc);
    b.gustAt(t, ha, hc);
    c.gustAt(t, ia, ic);
    identical = identical && ga == ha && gc == hc;
    differs = differs || ga != ia;
    sumSq += double(ga) * ga;
    sumMean += ga;
    peak = fabsf(ga) > peak ? fabsf(ga) : peak;
    TEST_ASSERT_TRUE(fabsf(ga) <= 4.0f + 1e-4f);
    TEST_ASSERT_TRUE(fabsf(gc) <= 0.3f * 4.0f + 1e-4f);
  }
  const float rms = sqrtf(float(sumSq / n));
  printf("SIMAERO gusts strength 4 m/s, period 3 s: rms %.2f m/s (model 1.00), peak %.2f m/s, mean %.2f\n", rms, peak,
         sumMean / n);
  TEST_ASSERT_TRUE(identical);
  TEST_ASSERT_TRUE(differs);
  TEST_ASSERT_FLOAT_WITHIN(0.25f, 1.0f, rms);          // strength / 4
  TEST_ASSERT_TRUE(peak > 0.5f * 4.0f && peak <= 4.0f); // realised peak 0.5..1 of the bound
  TEST_ASSERT_FLOAT_WITHIN(0.3f, 0.0f, float(sumMean / n));
  // Effect direction: mean wind toward West (90 deg), gusts along it: the horizontal wind vector stays on the
  // West axis for spread 0, and its Y component equals mean + gust.
  BodySimParams p;
  p.windSpeed_mps = 3.0f;
  p.windDirection_rad = 90 * kDeg;
  p.gustSpread = 0.0f;
  BodySimModel w(p);
  w.setGust(2.0f, 2.0f, 5);
  bool varies = false;
  for (int i = 0; i < 500; i++) { // advance the clock with steps
    w.step();
    Vector_F ww = w.windWorld();
    float ga, gc;
    w.gustAt(w.time(), ga, gc);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, ww(0));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 3.0f + ga, ww(1));
    varies = varies || fabsf(ga) > 0.2f;
  }
  TEST_ASSERT_TRUE(varies);
  // A falling, held-attitude body picks up the gusts: horizontal speed varies around the mean wind.
  BodySimParams p2;
  p2.windSpeed_mps = 3.0f;
  p2.windDirection_rad = 90 * kDeg;
  BodySimModel f(p2), g(p2);
  f.setGust(3.0f, 2.0f, 9);
  float minV = 1e9f, maxV = -1e9f, gMin = 1e9f, gMax = -1e9f;
  for (int i = 0; i < int(30.0f / p2.step_s); i++) {
    for (BodySimModel *mm : {&f, &g}) {
      auto ps = mm->positionState();
      if (i == 0)
        ps = {0, 0, 0, 0, 0, 5000};
      mm->setPositionState(ps);
      mm->setAttitudeState(attOf(bellyDown()));
      mm->setFlapInput(kTopNeutral, kTopNeutral, kBottomNeutral, kBottomNeutral);
      mm->step();
    }
    if (i > int(10.0f / p2.step_s)) {
      float vy = f.positionState()(1), vg = g.positionState()(1);
      minV = vy < minV ? vy : minV;
      maxV = vy > maxV ? vy : maxV;
      gMin = vg < gMin ? vg : gMin;
      gMax = vg > gMax ? vg : gMax;
    }
  }
  printf("SIMAERO gusty fall, mean wind 3 m/s West: Vy range [%.2f %.2f] with gusts, [%.2f %.2f] without\n", minV, maxV,
         gMin, gMax);
  TEST_ASSERT_TRUE(maxV - minV > gMax - gMin + 0.3f);
}

static void test_ground_contact_holds_in_wind() {
  BodySimParams p;
  p.windSpeed_mps = 10.0f;
  p.windDirection_rad = 1.0f;
  BodySimModel m(p);
  m.setAttitudeState(attOf(Quat_F(Vector_F{0, 1, 0}, 3 * kDeg)));
  m.setPositionState({0, 0, 0, 1.0f, 2.0f, 0});
  for (int i = 0; i < 5000; i++)
    m.step();
  auto ps = m.positionState();
  auto a = m.attitudeState();
  TEST_ASSERT_EQUAL_FLOAT(1.0f, ps(3));
  TEST_ASSERT_EQUAL_FLOAT(2.0f, ps(4));
  TEST_ASSERT_EQUAL_FLOAT(0.0f, ps(5));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 3.0f, tiltDeg(a));
}

static void test_determinism() {
  auto run = [](Vector<float, 7> &a, Vector<float, 6> &p) {
    BodySimModel m;
    m.setAttitudeState(attOf(Quat_F(Vector_F{0, 1, 0}, 7 * kDeg)));
    m.setPositionState({0, 0, 0, 0, 0, 500});
    FlapCtrlReplica c;
    for (int i = 0; i < 5000; i++) {
      if (i % 10 == 0) {
        c.update(m.attitudeState());
        m.setFlapInput(c.out[0], c.out[1], c.out[2], c.out[3]);
      }
      m.setThrust(Vector_F{0.2f, -0.1f, (i < 1000) ? 12.0f : 0.0f}, i < 1000, 0.01f);
      m.step();
    }
    a = m.attitudeState();
    p = m.positionState();
  };
  Vector<float, 7> a1, a2;
  Vector<float, 6> p1, p2;
  run(a1, p1);
  run(a2, p2);
  TEST_ASSERT_EQUAL_MEMORY(&a1, &a2, sizeof(a1));
  TEST_ASSERT_EQUAL_MEMORY(&p1, &p2, sizeof(p1));
  for (int i = 0; i < 7; i++)
    TEST_ASSERT_TRUE(isfinite(a1(i)));
}

static void test_advance_bounded_and_fixed_step() {
  BodySimModel m;
  m.setPositionState({0, 0, 0, 0, 0, 1000});
  TEST_ASSERT_EQUAL_INT(5, m.advance(0.010f));             // 10 ms = 5 fixed steps of 2 ms
  TEST_ASSERT_EQUAL_INT(25, m.advance(120.0f));            // a 2 min pause integrates only maxAdvance (50 ms)
  TEST_ASSERT_EQUAL_INT(0, m.advance(-1.0f));
  TEST_ASSERT_EQUAL_INT(0, m.advance(NAN));
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.060, m.time());
  // Same total time in different slices gives the same state as fixed steps.
  BodySimModel a, b;
  a.setPositionState({0, 0, 0, 0, 0, 1000});
  b.setPositionState({0, 0, 0, 0, 0, 1000});
  for (int i = 0; i < 100; i++)
    a.advance(0.010f);
  for (int i = 0; i < 500; i++)
    b.step();
  TEST_ASSERT_EQUAL_MEMORY(&a.positionState(), &b.positionState(), sizeof(Vector<float, 6>));
}

static void test_torque_free_spin_conserves_momentum() {
  BodySimParams p;
  p.airDensity_kgm3 = 0.0f; // vacuum
  BodySimModel m(p);
  m.setAttitudeState({0.5f, 3.0f, 2.0f, 1, 0, 0, 0});
  m.setPositionState({0, 0, 50, 0, 0, 10000}); // keep away from the ground
  auto L = [&]() {
    auto a = m.attitudeState();
    Quat_F q = qOf(a);
    Vector_F h = {p.inertiaXX * a(0), p.inertiaYY * a(1), p.inertiaZZ * a(2)};
    return q.rotate(h);
  };
  Vector_F L0 = L();
  for (int i = 0; i < 2000; i++)
    m.step();
  Vector_F L1 = L();
  printf("SIMAERO torque-free spin 4 s: |L| %.5f -> %.5f, direction change %.3f deg\n", L0.magnitude(),
         L1.magnitude(), L0.getAngleTo(L1) / kDeg);
  TEST_ASSERT_FLOAT_WITHIN(0.02f * L0.magnitude(), L0.magnitude(), L1.magnitude());
  TEST_ASSERT_TRUE(L0.getAngleTo(L1) < 2 * kDeg);
}

static void test_thrust_gating_and_nan_inputs() {
  BodySimModel m;
  m.setPositionState({0, 0, 0, 0, 0, 100});
  m.setThrust(Vector_F{0, 0, 10}, 0.05f, false); // disabled: no force, no roll torque (review F13)
  for (int i = 0; i < 100; i++)
    m.step();
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, m.attitudeState()(2));
  TEST_ASSERT_TRUE(m.positionState()(2) < -1.0f);
  m.setThrust(Vector_F{NAN, 0, 10}, NAN, true); // non-finite input = no thrust, never NaN state
  m.setFlapInput(NAN, NAN, NAN, NAN);
  for (int i = 0; i < 100; i++)
    m.step();
  for (int i = 0; i < 7; i++)
    TEST_ASSERT_TRUE(isfinite(m.attitudeState()(i)));
  for (int i = 0; i < 6; i++)
    TEST_ASSERT_TRUE(isfinite(m.positionState()(i)));
  // Cone clamp keeps the vector toward +Z (review F14): 10 deg request -> 3 deg.
  BodySimParams p;
  p.applyActuatorRateLimits = false;
  BodySimModel c(p);
  c.setThrust(Vector_F{sinf(10 * kDeg) * 10, sinf(10 * kDeg) * 10, 10}, 0, true);
  c.step();
  TEST_ASSERT_TRUE(fabsf(c.tvcAngleX()) < 3.01f * kDeg && fabsf(c.tvcAngleY()) < 3.01f * kDeg);
  TEST_ASSERT_TRUE(c.tvcAngleX() > 1.5f * kDeg && c.tvcAngleY() > 1.5f * kDeg);
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_hover_balance);
  RUN_TEST(test_terminal_speed_belly_down);
  RUN_TEST(test_belly_down_free_oscillation);
  RUN_TEST(test_flap_steps);
  RUN_TEST(test_static_flap_authority_table);
  RUN_TEST(test_control_axis_signs);
  RUN_TEST(test_neutral_sweep_effective_angle);
  RUN_TEST(test_candidate_effects_yaw_sign);
  RUN_TEST(test_flap_geometry_and_cant);
  RUN_TEST(test_vane_step);
  RUN_TEST(test_align_from_small_tilt);
  RUN_TEST(test_flip_and_kick);
  RUN_TEST(test_kick_cap_altitude_change_1p1_kg);
  RUN_TEST(test_closed_loop_flap_hold);
  RUN_TEST(test_sim_fault_reproduction_tl_stuck);
  RUN_TEST(test_wind_frame);
  RUN_TEST(test_gusts);
  RUN_TEST(test_ground_contact_holds_in_wind);
  RUN_TEST(test_determinism);
  RUN_TEST(test_advance_bounded_and_fixed_step);
  RUN_TEST(test_torque_free_spin_conserves_momentum);
  RUN_TEST(test_thrust_gating_and_nan_inputs);
  return UNITY_END();
}
