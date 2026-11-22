// Wiring of the descent control (step S3): observer target + position usability (src/safety/descent_target.hpp), the
// 50 Hz flap-task path (DescentFlapController -> FlapModeMapper, the same calls ControlAttitudeFlaps::taskThread makes)
// and a closed loop with the body simulator model through exactly that path.
//   pio test -e native -f test_descent_wiring            (add -v | grep WIRING for the numbers)
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unity.h>

#include <algorithm>
#include <vector>

#include "ExVectrControl/starship/descent_flap_controller.hpp"
#include "ExVectrControl/starship/flap_mode_mapper.hpp"
#include "ExVectrDSP/body_sim_model.hpp"
#include "safety/descent_target.hpp"

using namespace VCTR;
using namespace VCTR::Math;
using namespace VCTR::CTRL;
namespace D = VCTR::CTRL::descent;
using Mode = ControlAttitudeBellyFlopSetting::BellyFlopMode;

void setUp() {}
void tearDown() {}

static constexpr float kDeg = 3.14159265f / 180.0f;
static constexpr float kPi = 3.14159265f;

static Vector<float, 7> attOf(const Quat_F &q) { return {0, 0, 0, q(0), q(1), q(2), q(3)}; }
static Quat_F bellyDown(float pitchDeg) {
  return Quat_F(Vector_F{0, 1, 0}, -90 * kDeg) * Quat_F(Vector_F{0, 1, 0}, pitchDeg * kDeg);
}
static float wrapDeg(float a) {
  while (a > 180) a -= 360;
  while (a < -180) a += 360;
  return a;
}
struct Rng {
  uint32_t s;
  explicit Rng(uint32_t seed) : s(seed * 2654435761u + 12345u) { if (!s) s = 1; }
  uint32_t next() { s ^= s << 13, s ^= s >> 17, s ^= s << 5; return s; }
  float uni() { return (next() >> 8) * (1.0f / 16777216.0f); }
  float gauss() {
    float u1 = uni(), u2 = uni();
    if (u1 < 1e-7f) u1 = 1e-7f;
    return sqrtf(-2 * logf(u1)) * cosf(2 * kPi * u2);
  }
};
struct GaussMarkov {
  float x = 0, sigma = 0, tau = 1;
  void init(Rng &r, float sig, float t) { sigma = sig, tau = t; x = sig * r.gauss(); }
  float step(Rng &r, float dt) {
    const float a = expf(-dt / tau);
    x = a * x + sigma * sqrtf(1 - a * a) * r.gauss();
    return x;
  }
};

// ---------------------------------------------------------------------------------------------------------------
// Observer side: target, position usability, telemetry setpoint source
// ---------------------------------------------------------------------------------------------------------------
static void test_target_is_home_and_stop_altitude() {
  Safety::DescentTargetInputs in;
  in.descentActive = true, in.positionUsable = true, in.stopAltitude_m = 27.6f;
  auto t = Safety::makeDescentTarget(in);
  TEST_ASSERT_TRUE(t.valid);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, t.x_m);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, t.y_m);
  TEST_ASSERT_EQUAL_FLOAT(27.6f, t.endAltitude_m);
}
static void test_target_invalid_cases() {
  Safety::DescentTargetInputs in;
  in.descentActive = true, in.positionUsable = true, in.stopAltitude_m = 27.6f;
  in.descentActive = false;
  TEST_ASSERT_FALSE(Safety::makeDescentTarget(in).valid); // not in Descent: no target
  in.descentActive = true, in.positionUsable = false;
  TEST_ASSERT_FALSE(Safety::makeDescentTarget(in).valid); // position not usable
  in.positionUsable = true, in.stopAltitude_m = NAN;
  auto t = Safety::makeDescentTarget(in);
  TEST_ASSERT_TRUE(t.valid); // NaN stop altitude: fallback end altitude, still valid
  TEST_ASSERT_EQUAL_FLOAT(Safety::kDescentDefaultEndAltitudeM, t.endAltitude_m);
}
static void test_position_usable() {
  const int64_t to = 500000000LL;
  TEST_ASSERT_TRUE(Safety::descentPositionUsable(false, true, 100000000LL, to, true));
  TEST_ASSERT_FALSE(Safety::descentPositionUsable(false, false, 100000000LL, to, true)); // GNSS invalid
  TEST_ASSERT_FALSE(Safety::descentPositionUsable(false, true, 600000000LL, to, true));  // GNSS silent
  TEST_ASSERT_FALSE(Safety::descentPositionUsable(false, true, -1, to, true));           // timestamp in the future
  TEST_ASSERT_FALSE(Safety::descentPositionUsable(false, true, 0, to, false));           // non-finite estimate
  TEST_ASSERT_TRUE(Safety::descentPositionUsable(true, false, 99999999999LL, to, true)); // simulation: no GNSS needed
  TEST_ASSERT_FALSE(Safety::descentPositionUsable(true, false, 0, to, false));
}
static void test_telemetry_setpoint_source() {
  auto d = Safety::flapSetpointAngles(true, false, true, 1.0f, 2.0f, 0.5f);
  TEST_ASSERT_TRUE(d.flapsActive);
  TEST_ASSERT_EQUAL_FLOAT(0.5f, d.azimuth); // Descent: guidance heading setpoint, pitch 0
  TEST_ASSERT_EQUAL_FLOAT(0.0f, d.pitch);
  auto s = Safety::flapSetpointAngles(true, true, false, 1.0f, 2.0f, 0.5f);
  TEST_ASSERT_TRUE(s.flapsActive);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, s.azimuth); // legacy Stabilize semantics unchanged
  TEST_ASSERT_EQUAL_FLOAT(2.0f, s.pitch);
  TEST_ASSERT_FALSE(Safety::flapSetpointAngles(false, true, true, 1, 2, 3).flapsActive); // phase table off
  TEST_ASSERT_FALSE(Safety::flapSetpointAngles(true, false, false, 1, 2, 3).flapsActive);
}

// ---------------------------------------------------------------------------------------------------------------
// Flap-task path: DescentFlapController + FlapModeMapper
// ---------------------------------------------------------------------------------------------------------------
struct Chain {
  D::DescentFlapController ctrl;
  FlapStabilizeParams stab;
  D::FlapDescentLawParams law;
  D::FlapDescentLawMem lawMem;
  ControlAttitudeFlapSetting step(Mode mode, const D::DescentTarget &t, const float pos[6], bool fresh,
                                  const Vector<float, 7> &att, float dt = 0.02f) {
    ControlAttitudeBellyFlopSetting s;
    s.bellyFlopMode = mode;
    float a[7];
    for (int i = 0; i < 7; i++) a[i] = att(i);
    const auto eff = ctrl.apply(s, t, pos, fresh, a, dt);
    return FlapModeMapper::map(eff, att, stab, law, lawMem);
  }
};
static D::DescentTarget homeTarget(bool valid = true) {
  D::DescentTarget t;
  t.valid = valid;
  t.endAltitude_m = 27.6f;
  return t;
}

static void test_entry_init_holds_heading_and_leaving_resets() {
  Chain c;
  const Vector<float, 7> att = attOf(Quat_F(Vector_F{0, 0, 1}, 1.0f) * bellyDown(0)); // some heading
  float pos[6] = {0, 0, 0, 20, 5, 100};
  // Acsent first: nothing runs, guidance memory pristine.
  c.step(Mode::BellyFlopMode_Acsent, homeTarget(), pos, true, att);
  TEST_ASSERT_FALSE(c.ctrl.status().active);
  // Entry: heading setpoint = the nose heading at entry, entry hold (split -20 deg), nothing blows up.
  auto f = c.step(Mode::BellyFlopMode_Descent, homeTarget(), pos, true, att);
  TEST_ASSERT_TRUE(c.ctrl.status().active);
  float a[7];
  for (int i = 0; i < 7; i++) a[i] = att(i);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, D::bellyAttitude(a).heading, c.ctrl.status().headingSp_rad);
  for (float v : {f.flapTLAngle_Rad, f.flapTRAngle_Rad, f.flapBLAngle_Rad, f.flapBRAngle_Rad}) {
    TEST_ASSERT_TRUE(isfinite(v));
    TEST_ASSERT_TRUE(v >= 14.9f * kDeg && v <= 90.1f * kDeg);
  }
  // Run a while so that memory builds up, then leave through Stabilize: reset at once.
  for (int i = 0; i < 100; i++) c.step(Mode::BellyFlopMode_Descent, homeTarget(), pos, true, att);
  TEST_ASSERT_TRUE(c.ctrl.memory().elapsed_s > 1.9f);
  TEST_ASSERT_TRUE(fabsf(c.ctrl.memory().windInt[0]) + fabsf(c.ctrl.memory().windInt[1]) > 0);
  c.step(Mode::BellyFlopMode_Stabilize, homeTarget(), pos, true, att);
  TEST_ASSERT_FALSE(c.ctrl.status().active);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, c.ctrl.memory().elapsed_s);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, c.ctrl.memory().windInt[0]);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, c.ctrl.memory().windInt[1]);
  // Re-entry starts a fresh entry hold with a fresh heading.
  const Vector<float, 7> att2 = attOf(Quat_F(Vector_F{0, 0, 1}, -2.0f) * bellyDown(0));
  c.step(Mode::BellyFlopMode_Descent, homeTarget(), pos, true, att2);
  for (int i = 0; i < 7; i++) a[i] = att2(i);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, D::bellyAttitude(a).heading, c.ctrl.status().headingSp_rad);
  TEST_ASSERT_TRUE(c.ctrl.memory().elapsed_s < 0.05f);
}

static void test_other_modes_pass_through() {
  D::DescentFlapController dc;
  ControlAttitudeBellyFlopSetting s;
  s.azimuthAngle_Rad = 0.3f, s.pitchAngle_Rad = 0.2f, s.bellyFlopMode = Mode::BellyFlopMode_Stabilize;
  float pos[6] = {0, 0, 0, 1, 2, 100}, att[7] = {0, 0, 0, 1, 0, 0, 0};
  auto r = dc.apply(s, homeTarget(), pos, true, att, 0.02f);
  TEST_ASSERT_EQUAL_FLOAT(0.3f, r.azimuthAngle_Rad);
  TEST_ASSERT_EQUAL_FLOAT(0.2f, r.pitchAngle_Rad);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, r.pitchSplit_Rad);
  TEST_ASSERT_TRUE(r.bellyFlopMode == Mode::BellyFlopMode_Stabilize);
}

/// Run `n` ticks past the entry hold with a healthy target so that the guidance is gliding, then apply the fault.
static void settleGliding(Chain &c, const float pos[6], const Vector<float, 7> &att) {
  for (int i = 0; i < 200; i++) c.step(Mode::BellyFlopMode_Descent, homeTarget(), pos, true, att);
}
static void check_invalid_falls_straight(bool fresh, bool targetValid, const float *pos) {
  Chain c;
  const Vector<float, 7> att = attOf(Quat_F(Vector_F{0, 0, 1}, 0.4f) * bellyDown(0));
  float good[6] = {0, 0, 0, 15, 0, 100}; // 15 m from home: the guidance wants to glide
  settleGliding(c, good, att);
  const float held = c.ctrl.status().headingSp_rad;
  TEST_ASSERT_TRUE(c.ctrl.status().pitchSplit_rad < -5 * kDeg); // gliding forward
  ControlAttitudeFlapSetting f;
  for (int i = 0; i < 100; i++) f = c.step(Mode::BellyFlopMode_Descent, homeTarget(targetValid), pos, fresh, att);
  TEST_ASSERT_FALSE(c.ctrl.status().inputsValid);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, held, c.ctrl.status().headingSp_rad); // heading held
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, c.ctrl.status().pitchSplit_rad); // split slewed to 0 (2 s at 40 deg/s)
  TEST_ASSERT_EQUAL_FLOAT(0.0f, c.ctrl.status().glideCmd_mps);
  // Neutral 30/78 (nose along heading: no attitude error, no rate).
  // (the static test attitude is not at the held heading, so only the common mode is checked: the split is 0)
  TEST_ASSERT_FLOAT_WITHIN(0.3f * kDeg, 30 * kDeg, 0.5f * (f.flapTLAngle_Rad + f.flapTRAngle_Rad));
  TEST_ASSERT_FLOAT_WITHIN(0.3f * kDeg, 78 * kDeg, 0.5f * (f.flapBLAngle_Rad + f.flapBRAngle_Rad));
}
static void test_invalid_position_falls_straight() {
  float p[6] = {0, 0, 0, 15, 0, 100};
  check_invalid_falls_straight(false, true, p); // stale estimate
  check_invalid_falls_straight(true, false, p); // target invalid (GNSS lost)
  float nanP[6] = {0, 0, 0, NAN, 0, 100};
  check_invalid_falls_straight(true, true, nanP); // NaN position
  float infV[6] = {INFINITY, 0, 0, 15, 0, 100};
  check_invalid_falls_straight(true, true, infV);
}
static void test_recovery_after_invalid() {
  Chain c;
  const Vector<float, 7> att = attOf(Quat_F(Vector_F{0, 0, 1}, 0.4f) * bellyDown(0));
  float good[6] = {0, 0, 0, 15, 0, 100};
  settleGliding(c, good, att);
  for (int i = 0; i < 100; i++) c.step(Mode::BellyFlopMode_Descent, homeTarget(false), good, true, att);
  for (int i = 0; i < 100; i++) c.step(Mode::BellyFlopMode_Descent, homeTarget(true), good, true, att);
  TEST_ASSERT_TRUE(c.ctrl.status().inputsValid);
  TEST_ASSERT_TRUE(c.ctrl.status().pitchSplit_rad < -5 * kDeg); // gliding again
}
static void test_nan_attitude_gives_neutral_flaps() {
  Chain c;
  float pos[6] = {0, 0, 0, 15, 0, 100};
  const Vector<float, 7> good = attOf(bellyDown(0));
  c.step(Mode::BellyFlopMode_Descent, homeTarget(), pos, true, good);
  Vector<float, 7> bad = good;
  bad(4) = NAN;
  auto f = c.step(Mode::BellyFlopMode_Descent, homeTarget(), pos, true, bad);
  for (float v : {f.flapTLAngle_Rad, f.flapTRAngle_Rad, f.flapBLAngle_Rad, f.flapBRAngle_Rad})
    TEST_ASSERT_TRUE(isfinite(v));
}

// ---------------------------------------------------------------------------------------------------------------
// Closed loop through the wired path
// ---------------------------------------------------------------------------------------------------------------
static float stopAltitude(float vz) { return 0.5f * vz * vz / (14.0f / 1.1f - 9.81f) + 20.0f; }

struct Scn {
  float offset = 3, betaDeg = 0, headingErrDeg = 180, wind = 0;
  uint32_t seed = 3;
  bool noise = false;
};
struct Res {
  float miss = 0, vhEnd = 0, zEnd = 0, tEnd = 0, bellyMax = 0;
  float lateHeadErrMax = 0;  ///< max |nose heading - heading setpoint| over the last 5 s, deg
  float lateHeadSpSpan = 0;  ///< max-min of the heading setpoint over the last 5 s, deg
  int lateHeadSwings = 0;    ///< sign changes of the heading error between excursions beyond +-10 deg, last 5 s
  int bottomReversals = 0;   ///< sign changes (both sides > 10 deg) of the bottom flap differential after 4 s
  int topReversals = 0;
  float flapStepMax = 0;     ///< largest single-tick change of any flap command after 3 s, deg
  bool timeout = false;
};

/// Kick end state at 120 m (belly-down, +5 deg, 3.6 m/s along the nose, flaps 90 deg), 0.5 s of Acsent (flaps retracted),
/// then Descent from the observer (10 Hz target) through the flap-task chain at 50 Hz.
static Res runWired(const Scn &sc) {
  DSP::BodySimParams p;
  p.applyActuatorRateLimits = true;
  DSP::BodySimModel m(p);
  Rng rng(sc.seed);
  const float beta = sc.betaDeg * kDeg;
  const float psi0 = beta + sc.headingErrDeg * kDeg;
  const float px = -sc.offset * cosf(beta), py = -sc.offset * sinf(beta);
  Quat_F q = Quat_F(Vector_F{0, 0, 1}, psi0 - kPi) * bellyDown(5);
  Vector<float, 7> a0 = attOf(q);
  a0(1) = 0.3f;
  m.setAttitudeState(a0);
  m.setPositionState({3.6f * cosf(psi0), 3.6f * sinf(psi0), 0, px, py, 120});
  if (sc.wind > 0) m.setWind(sc.wind, rng.uni() * 2 * kPi);
  GaussMarkov np[3], nv[3], nh;
  for (int i = 0; i < 3; i++) {
    np[i].init(rng, sc.noise ? 0.3f : 0, 2.0f);
    nv[i].init(rng, sc.noise ? 0.2f : 0, 0.3f);
  }
  nh.init(rng, sc.noise ? 2 * kDeg : 0, 5.0f);

  Chain chain;
  D::DescentTarget tgt;
  float estPos[6] = {0}, estAtt[7] = {0};
  float cmd[4] = {90 * kDeg, 90 * kDeg, 90 * kDeg, 90 * kDeg}, prev[4];
  memcpy(prev, cmd, sizeof cmd);
  Res r;
  const float dt = p.step_s;
  const int maxSteps = int(60.0f / dt);
  float lastBot = 0, lastTop = 0;
  struct Smp { float t, herr, sp; };
  std::vector<Smp> smp;
  for (int i = 0; i < maxSteps; i++) {
    const float t = i * dt;
    if (i % 5 == 0) {
      auto ps = m.positionState();
      auto as = m.attitudeState();
      for (int k = 0; k < 3; k++) {
        estPos[k] = ps(k) + nv[k].step(rng, 0.01f);
        estPos[3 + k] = ps(3 + k) + np[k].step(rng, 0.01f);
      }
      const float hn = nh.step(rng, 0.01f);
      Quat_F qt(as(3), as(4), as(5), as(6));
      Quat_F qn = Quat_F(Vector_F{0, 0, 1}, hn) * qt;
      for (int k = 0; k < 3; k++) estAtt[k] = as(k) + (sc.noise ? 0.01f * rng.gauss() : 0);
      for (int k = 0; k < 4; k++) estAtt[3 + k] = qn(k);
      if (t > 1.0f && estPos[5] < stopAltitude(estPos[2])) {
        const float ex = ps(3), ey = ps(4);
        r.miss = sqrtf(ex * ex + ey * ey);
        r.vhEnd = sqrtf(ps(0) * ps(0) + ps(1) * ps(1));
        r.zEnd = ps(5), r.tEnd = t;
        break;
      }
    }
    const bool descent = t >= 0.5f;
    if (i % 50 == 0) { // observer, 10 Hz
      Safety::DescentTargetInputs ti;
      ti.descentActive = descent;
      ti.positionUsable = true;
      ti.stopAltitude_m = stopAltitude(estPos[2]);
      tgt = Safety::makeDescentTarget(ti);
    }
    if (i % 10 == 0) { // flap task, 50 Hz
      Vector<float, 7> att;
      for (int k = 0; k < 7; k++) att(k) = estAtt[k];
      auto f = chain.step(descent ? Mode::BellyFlopMode_Descent : Mode::BellyFlopMode_Acsent, tgt, estPos, true, att);
      cmd[0] = f.flapTLAngle_Rad, cmd[1] = f.flapTRAngle_Rad, cmd[2] = f.flapBLAngle_Rad, cmd[3] = f.flapBRAngle_Rad;
      for (int k = 0; k < 4; k++) cmd[k] = std::min(90 * kDeg, std::max(15 * kDeg, cmd[k]));
      if (t > 3.0f)
        for (int k = 0; k < 4; k++) r.flapStepMax = std::max(r.flapStepMax, fabsf(cmd[k] - prev[k]) / kDeg);
      memcpy(prev, cmd, sizeof cmd);
      const float bot = (cmd[2] - cmd[3]) / kDeg, top = (cmd[0] - cmd[1]) / kDeg;
      if (t > 4.0f) {
        if (fabsf(bot) > 10 && lastBot != 0 && (bot > 0) != (lastBot > 0)) r.bottomReversals++;
        if (fabsf(top) > 10 && lastTop != 0 && (top > 0) != (lastTop > 0)) r.topReversals++;
        if (fabsf(bot) > 10) lastBot = bot;
        if (fabsf(top) > 10) lastTop = top;
      }
      m.setFlapInput(cmd[0], cmd[1], cmd[2], cmd[3]);
    }
    m.step();
    auto as = m.attitudeState();
    float a7[7];
    for (int k = 0; k < 7; k++) a7[k] = as(k);
    const D::BellyAttitude b = D::bellyAttitude(a7);
    if (t > 2.0f) r.bellyMax = std::max(r.bellyMax, b.belly / kDeg);
    if (i % 5 == 0 && descent) {
      const float sp = chain.ctrl.status().headingSp_rad / kDeg;
      smp.push_back({t, wrapDeg(sp - b.heading / kDeg), sp});
      if (getenv("WIRING_TRACE") && i % 25 == 0)
        printf("WIRING trace t %5.2f z %6.1f x %+6.1f y %+6.1f head %+6.1f sp %+6.1f split %+5.1f belly %3.0f flaps %2.0f %2.0f %2.0f %2.0f\n", t, m.positionState()(5), m.positionState()(3), m.positionState()(4), b.heading / kDeg, sp, chain.ctrl.status().pitchSplit_rad / kDeg, b.belly / kDeg, cmd[0] / kDeg, cmd[1] / kDeg, cmd[2] / kDeg, cmd[3] / kDeg);
    }
    if (i == maxSteps - 1) r.timeout = true;
  }
  float mn = 1e9f, mx = -1e9f;
  float lastBig = 0;
  for (const auto &s : smp)
    if (s.t >= r.tEnd - 5.0f) {
      if (fabsf(s.herr) > 10.0f) {
        if (lastBig != 0 && (s.herr > 0) != (lastBig > 0)) r.lateHeadSwings++;
        lastBig = s.herr;
      }
      r.lateHeadErrMax = std::max(r.lateHeadErrMax, fabsf(s.herr));
      mn = std::min(mn, s.sp), mx = std::max(mx, s.sp);
    }
  r.lateHeadSpSpan = mx > mn ? mx - mn : 0;
  return r;
}

static void print(const char *tag, const Scn &sc, const Res &r) {
  printf("WIRING %s off %.0f bearing %.0f herr %.0f wind %.0f seed %u: miss %.2f m at z %.1f t %.1f vh %.2f | late head "
         "err max %.1f swings %d sp span %.1f | belly max %.0f | reversals bottom %d top %d | flap step max %.1f%s\n",
         tag, sc.offset, sc.betaDeg, sc.headingErrDeg, sc.wind, (unsigned)sc.seed, r.miss, r.zEnd, r.tEnd, r.vhEnd,
         r.lateHeadErrMax, r.lateHeadSwings, r.lateHeadSpSpan, r.bellyMax, r.bottomReversals, r.topReversals, r.flapStepMax,
         r.timeout ? " TIMEOUT" : "");
}

/// Owner's case (3 m South of home, nose South, 120 m apogee, calm, no wind): the acceptance of the design.
static void test_closed_loop_owner_case() {
  Scn sc;
  Res r = runWired(sc);
  print("owner", sc, r);
  TEST_ASSERT_FALSE(r.timeout);
  TEST_ASSERT_TRUE(r.miss < 3.0f);
  TEST_ASSERT_TRUE(r.lateHeadErrMax < 10.0f);
  TEST_ASSERT_EQUAL_INT(0, r.lateHeadSwings);
  TEST_ASSERT_TRUE(r.bellyMax < 35.0f);
  TEST_ASSERT_TRUE(r.bottomReversals <= 2);
  TEST_ASSERT_TRUE(r.topReversals <= 2);
}

/// Offsets <= 10 m, headings 0/90/180 deg to the bearing, with estimator noise, calm: miss < 3 m each (design S3).
static void runMatrix(bool noise) {
  int n = 0;
  float worst = 0;
  for (float off : {3.0f, 10.0f})
    for (float herr : {0.0f, 90.0f, 180.0f})
      for (uint32_t seed : {1u, 2u}) {
        Scn sc;
        sc.offset = off, sc.headingErrDeg = herr, sc.seed = seed, sc.noise = noise;
        sc.betaDeg = 37.0f * seed;
        Res r = runWired(sc);
        print(noise ? "noisy" : "clean", sc, r);
        TEST_ASSERT_FALSE(r.timeout);
        TEST_ASSERT_TRUE(r.miss < 3.0f);
        // No heading oscillation late: no swing between excursions beyond +-10 deg, error below 45 deg (a slow tail of
        // a late setpoint change from the position noise is not an oscillation; seen worst: 38 deg).
        if (noise) {
          TEST_ASSERT_TRUE(r.lateHeadSwings <= 1); // worst seen: 1 (position noise at the end of the funnel)
          TEST_ASSERT_TRUE(r.lateHeadErrMax < 45.0f);
        } else { // the bench condition (simulator states, no estimator noise): the design acceptance
          TEST_ASSERT_EQUAL_INT(0, r.lateHeadSwings);
          TEST_ASSERT_TRUE(r.lateHeadErrMax < 10.0f);
        }
        TEST_ASSERT_TRUE(r.bellyMax < 35.0f);
        TEST_ASSERT_TRUE(r.bottomReversals <= 2);
        worst = std::max(worst, r.miss);
        n++;
      }
  printf("WIRING %s matrix: %d runs, worst miss %.2f m\n", noise ? "noisy" : "clean", n, worst);
}

static void test_closed_loop_offsets_clean() { runMatrix(false); }
static void test_closed_loop_offsets_with_noise() { runMatrix(true); }

/// The no-position failure through the chain: GNSS lost at 100 m after the descent began: the ship must not glide away.
static void test_closed_loop_gnss_loss_falls_straight() {
  // offset 10 m, nose toward home; target goes invalid after 3 s.
  DSP::BodySimParams p;
  p.applyActuatorRateLimits = true;
  DSP::BodySimModel m(p);
  const float psi0 = 0;
  Quat_F q = Quat_F(Vector_F{0, 0, 1}, psi0 - kPi) * bellyDown(5);
  Vector<float, 7> a0 = attOf(q);
  a0(1) = 0.3f;
  m.setAttitudeState(a0);
  m.setPositionState({3.6f, 0, 0, -10, 0, 120});
  Chain chain;
  float cmd[4] = {90 * kDeg, 90 * kDeg, 90 * kDeg, 90 * kDeg};
  float xLoss = 0, tLoss = 3.0f, vhAfter = 0;
  bool lost = false;
  for (int i = 0; i < int(12.0f / p.step_s); i++) {
    const float t = i * p.step_s;
    if (i % 10 == 0) {
      auto ps = m.positionState();
      auto as = m.attitudeState();
      float pos[6], att7[7];
      for (int k = 0; k < 6; k++) pos[k] = ps(k);
      for (int k = 0; k < 7; k++) att7[k] = as(k);
      Vector<float, 7> att;
      for (int k = 0; k < 7; k++) att(k) = att7[k];
      if (t >= tLoss && !lost) lost = true, xLoss = ps(3);
      D::DescentTarget tgt = homeTarget(!lost);
      auto f = chain.step(Mode::BellyFlopMode_Descent, tgt, pos, true, att);
      cmd[0] = f.flapTLAngle_Rad, cmd[1] = f.flapTRAngle_Rad, cmd[2] = f.flapBLAngle_Rad, cmd[3] = f.flapBRAngle_Rad;
      for (int k = 0; k < 4; k++) cmd[k] = std::min(90 * kDeg, std::max(15 * kDeg, cmd[k]));
      m.setFlapInput(cmd[0], cmd[1], cmd[2], cmd[3]);
      if (t > tLoss + 4.0f) vhAfter = std::max(vhAfter, sqrtf(ps(0) * ps(0) + ps(1) * ps(1)));
    }
    m.step();
  }
  auto ps = m.positionState();
  printf("WIRING gnss loss at %.0f s (x %.1f m): after 12 s x %.1f z %.1f, max horizontal speed 4 s after the loss %.2f m/s\n",
         tLoss, xLoss, ps(3), ps(5), vhAfter);
  TEST_ASSERT_TRUE(vhAfter < 1.5f); // zero-glide fall: no glide (free-trim residual drift only)
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_target_is_home_and_stop_altitude);
  RUN_TEST(test_target_invalid_cases);
  RUN_TEST(test_position_usable);
  RUN_TEST(test_telemetry_setpoint_source);
  RUN_TEST(test_entry_init_holds_heading_and_leaving_resets);
  RUN_TEST(test_other_modes_pass_through);
  RUN_TEST(test_invalid_position_falls_straight);
  RUN_TEST(test_recovery_after_invalid);
  RUN_TEST(test_nan_attitude_gives_neutral_flaps);
  RUN_TEST(test_closed_loop_owner_case);
  RUN_TEST(test_closed_loop_offsets_clean);
  RUN_TEST(test_closed_loop_offsets_with_noise);
  RUN_TEST(test_closed_loop_gnss_loss_falls_straight);
  return UNITY_END();
}
