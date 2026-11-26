// Powered bellyflop kick end-criterion study: pio test -e native -f test_kick_sweep -v | grep KICK
//
// Closed loop from the apogee hover (120 m, vertical, at rest over home) through the kick into the freefall descent:
//   - kick: replica of ControlAttitudeTvc (P 1.2, rate 0.5, TVC limit compensation on, 50 Hz) + StarshipTVC cap
//     (11 N) + DSP::BodySimModel (vane servo rate, 3 deg cone), flaps retracted (90 deg), mission task at 100 Hz
//     evaluating Safety::kickEndTrigger (the production function, parameters varied by the sweep),
//   - motor cut at the trigger (thrust off 20 ms later), flap attitude control from the next 10 Hz safety tick,
//   - descent: DescentGuidance + FlapDescentLaw (strategy B of test_descent_guidance) with estimator noise, until
//     the freefall stop trigger.
// Simplification: the position controller's acceleration setpoint is replaced by the hover value (0, 0, g); with the
// TVC limit compensation on, the kick thrust is the 11 N cap in the direction set by the attitude torque, so the
// position term is irrelevant while the torque saturates the 3 deg cone.
//
// KICK_SEEDS=<n> (default 4) seeds per wind class. Report-only; the asserts only check the chosen rule.
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unity.h>

#include <algorithm>
#include <vector>

#include "ExVectrControl/starship/descent_guidance.hpp"
#include "ExVectrControl/starship/flap_descent_law.hpp"
#include "ExVectrDSP/body_sim_model.hpp"
#include "safety/bellyflop_kick.hpp"

using namespace VCTR;
using namespace VCTR::Math;
using DSP::BodySimModel;
using DSP::BodySimParams;
namespace D = VCTR::CTRL::descent;
namespace S = VCTR::Safety;

void setUp() {}
void tearDown() {}

static constexpr float kDeg = 3.14159265f / 180.0f;
static constexpr float kPi = 3.14159265f;

struct Rng {
  uint32_t s;
  explicit Rng(uint32_t seed) : s(seed * 2654435761u + 12345u) {
    if (!s)
      s = 1;
  }
  uint32_t next() {
    s ^= s << 13, s ^= s >> 17, s ^= s << 5;
    return s;
  }
  float uni() { return (next() >> 8) * (1.0f / 16777216.0f); }
  float gauss() {
    float u1 = uni(), u2 = uni();
    if (u1 < 1e-7f)
      u1 = 1e-7f;
    return sqrtf(-2 * logf(u1)) * cosf(2 * kPi * u2);
  }
};
struct GaussMarkov {
  float x = 0, sigma = 0, tau = 1;
  void init(Rng &r, float sig, float t) { sigma = sig, tau = t, x = sig * r.gauss(); }
  float step(Rng &r, float dt) {
    const float a = expf(-dt / tau);
    x = a * x + sigma * sqrtf(1 - a * a) * r.gauss();
    return x;
  }
};

static float stopAltitude(float vz) { return 0.5f * vz * vz / (14.0f / 1.1f - 9.81f) + 20.0f; }

// ---------------------------------------------------------------------------------------------------------------
// Replica of ControlAttitudeTvc::taskThread (attitude part) + StarshipTVC thrust cap
// ---------------------------------------------------------------------------------------------------------------
static Vector_F tvcKickForce(const float att[7], const float qTarget[4], float capN) {
  Quat_F attitude(att[3], att[4], att[5], att[6]);
  Quat_F wanted(qTarget[0], qTarget[1], qTarget[2], qTarget[3]);
  Vector_F w{att[0], att[1], att[2]};
  auto quatOut = attitude.conjugate() * wanted;
  if (quatOut(0) < 0)
    quatOut = -quatOut;
  Vector_F axisZ{0, 0, 1};
  auto wantedZInBody = quatOut.rotate(axisZ);
  float ang = axisZ.getAngleTo(wantedZInBody);
  Vector_F err = axisZ.cross(wantedZInBody).normalize() * ang;
  if (ang < 10 * kDeg)
    err(2) = atan2f(quatOut(3), quatOut(0));
  Vector_F ctrl{err(0) * 1.2f - w(0) * 0.5f, err(1) * 1.2f - w(1) * 0.5f, err(2) * 0.4f - w(2) * 0.08f};
  const float tvcCg = -0.35f;
  Vector_F torque = ctrl.cross(Vector_F{0, 0, 1 / tvcCg});
  Vector_F wantedBodyForce = attitude.conjugate().rotate(Vector_F{0, 0, 9.81f * 1.1f});
  float cosL = cosf(wantedBodyForce.getAngleTo(Vector_F{0, 0, 1}));
  if (cosL < 0)
    cosL = 0;
  Vector_F f = torque;
  f(2) += wantedBodyForce.magnitude() * cosL;
  const float limit = 3.0f * kDeg;
  float tvcAngle = f.getAngleTo(Vector_F{0, 0, 1});
  if (tvcAngle > limit) {
    Vector_F axis = (Vector_F{0, 0, 1}.cross(f)).normalize();
    Vector_F fn = Quat_F(axis, limit).rotate(Vector_F{0, 0, 1});
    float xy = sqrtf(f(0) * f(0) + f(1) * f(1));
    float xyn = sqrtf(fn(0) * fn(0) + fn(1) * fn(1));
    f = fn * (xy / xyn); // compensateTVCAngle_ = true (main.cpp)
  }
  const float mag = f.magnitude();
  if (mag > capN)
    f = f.normalize() * capN;
  return f;
}

// ---------------------------------------------------------------------------------------------------------------
struct Scenario {
  float wind = 0, gust = 0;
  uint32_t seed = 1;
  float yawDeg = 0;
  bool noise = true;
  float apogee = 120.0f;
};

struct Rule {
  const char *name;
  S::KickEndParams p;
};

struct Result {
  float tKick = 0, bellyEnd = 0, rateEnd = 0, speedEnd = 0, horizEnd = 0;
  float peak1 = 0;      // peak belly angle in the first second after the kick end (incl. the end value)
  float peakLate = 0;   // peak belly angle after that, until the flip start
  float belly1 = 0;     // belly angle 1 s after the kick end
  float miss = 0, vhFlip = 0, overshoot = 0, maxExcursion = 0, bellyFlip = 0;
  float zDrop = 0;      // altitude lost during the kick
  S::KickEnd trig = S::KickEnd::None;
  bool timeout = false;
};

static Result runKick(const Rule &rule, const Scenario &sc, bool trace = false) {
  BodySimParams p;
  p.applyActuatorRateLimits = true;
  BodySimModel m(p);
  Rng rng(sc.seed);
  const float windDir = rng.uni() * 2 * kPi;
  Quat_F q0 = Quat_F(Vector_F{0, 0, 1}, sc.yawDeg * kDeg);
  m.setAttitudeState({0, 0, 0, q0(0), q0(1), q0(2), q0(3)});
  m.setPositionState({0, 0, 0, 0, 0, sc.apogee});
  m.setFlapInput(90 * kDeg, 90 * kDeg, 90 * kDeg, 90 * kDeg);
  if (sc.wind > 0) {
    m.setWind(sc.wind, windDir);
    if (sc.gust > 0)
      m.setGust(sc.gust, 5.0f, sc.seed + 1000);
  }
  GaussMarkov np[3], nv[3], nh;
  for (int i = 0; i < 3; i++) {
    np[i].init(rng, sc.noise ? 0.3f : 0, 2.0f);
    nv[i].init(rng, sc.noise ? 0.2f : 0, 0.3f);
  }
  nh.init(rng, sc.noise ? 2 * kDeg : 0, 5.0f);

  float qA0[4] = {q0(0), q0(1), q0(2), q0(3)}, qT[4];
  S::kickTargetAttitude(qA0, S::kKickPitchAngleRad, qT);

  D::DescentGuidanceParams gp;
  D::FlapDescentLawParams lp;
  D::DescentGuidanceMem gm;
  D::FlapDescentLawMem lm;
  D::DescentTarget tgt;
  tgt.x_m = 0, tgt.y_m = 0, tgt.valid = true;
  bool guidInit = false;

  Result r;
  float cmd[4] = {90 * kDeg, 90 * kDeg, 90 * kDeg, 90 * kDeg};
  float estPos[6] = {0}, estAtt[7] = {0};
  Vector_F thrust{0, 0, 0};
  bool kicking = true, motorOn = true, flapsOn = false;
  float tEnd = -1, tMotorOff = 0, tFlaps = 0;
  float ux = 0, uy = 0;          // outward direction at the kick end (home -> ship)
  bool uSet = false;
  const float dt = p.step_s;
  const int maxSteps = int(60.0f / dt);
  for (int i = 0; i < maxSteps; i++) {
    const float t = i * dt;
    if (i % 5 == 0) { // estimator + mission task, 100 Hz
      auto ps = m.positionState();
      auto as = m.attitudeState();
      for (int k = 0; k < 3; k++) {
        estPos[k] = ps(k) + nv[k].step(rng, 0.01f);
        estPos[3 + k] = ps(3 + k) + np[k].step(rng, 0.01f);
      }
      const float hn = nh.step(rng, 0.01f);
      Quat_F qt(as(3), as(4), as(5), as(6));
      Quat_F qn = Quat_F(Vector_F{0, 0, 1}, hn) * qt;
      for (int k = 0; k < 3; k++)
        estAtt[k] = as(k) + (sc.noise ? 0.01f * rng.gauss() : 0);
      for (int k = 0; k < 4; k++)
        estAtt[3 + k] = qn(k);
      if (kicking) {
        const float qe[4] = {estAtt[3], estAtt[4], estAtt[5], estAtt[6]};
        const float rem = S::kickRemainingAngle(qe, qT);
        const int64_t el = int64_t(t * 1e9f);
        S::KickEnd tr = S::kickEndTrigger(el, rem, estAtt[1], rule.p);
        if (tr != S::KickEnd::None) {
          kicking = false;
          tEnd = t;
          r.tKick = t;
          r.trig = tr;
          float a7[7];
          for (int k = 0; k < 7; k++)
            a7[k] = as(k);
          r.bellyEnd = D::bellyAttitude(a7).belly / kDeg;
          r.rateEnd = -as(1) / kDeg;
          r.speedEnd = sqrtf(ps(0) * ps(0) + ps(1) * ps(1) + ps(2) * ps(2));
          r.horizEnd = sqrtf(ps(0) * ps(0) + ps(1) * ps(1));
          r.zDrop = sc.apogee - ps(5);
          tMotorOff = t + 0.02f;                              // controller publishes neutral within 20 ms
          tFlaps = (floorf((t + 0.0001f) / 0.1f) + 1) * 0.1f; // next 10 Hz safety tick: freefall, flaps on
        }
      }
      if (!kicking && !uSet) {
        const float d = sqrtf(ps(3) * ps(3) + ps(4) * ps(4));
        if (d > 0.05f) {
          ux = ps(3) / d, uy = ps(4) / d, uSet = true;
        }
      }
      if (!kicking && t >= tFlaps)
        flapsOn = true;
      if (!kicking && t >= tMotorOff && motorOn) {
        motorOn = false;
        m.setThrust(Vector_F{0, 0, 0}, 0, false);
      }
      if (flapsOn && estPos[5] < stopAltitude(estPos[2]) && t > tEnd + 0.3f) {
        const float ex = ps(3), ey = ps(4);
        r.miss = sqrtf(ex * ex + ey * ey);
        r.vhFlip = sqrtf(ps(0) * ps(0) + ps(1) * ps(1));
        float a7[7];
        for (int k = 0; k < 7; k++)
          a7[k] = as(k);
        r.bellyFlip = D::bellyAttitude(a7).belly / kDeg;
        break;
      }
    }
    if (i % 10 == 0) { // 50 Hz controllers
      if (kicking) {
        thrust = tvcKickForce(estAtt, qT, S::kKickThrustLimitN);
        m.setThrust(thrust, 0, true);
      } else if (flapsOn) {
        if (!guidInit) {
          D::DescentGuidance::init(gp, estAtt, gm);
          guidInit = true;
        }
        tgt.endAltitude_m = stopAltitude(estPos[2]);
        D::DescentGuidanceOut go = D::DescentGuidance::step(gp, tgt, estPos, estAtt, 0.02f, gm);
        D::FlapDescentCmd c;
        c.headingSp_rad = go.headingSp_rad;
        c.pitchSplit_rad = go.pitchSplit_rad;
        D::FlapDescentLawOut o = D::FlapDescentLaw::step(lp, estAtt, c, lm);
        memcpy(cmd, o.flap, sizeof cmd);
        m.setFlapInput(cmd[0], cmd[1], cmd[2], cmd[3]);
      }
    }
    m.step();
    auto as = m.attitudeState();
    auto ps = m.positionState();
    if (tEnd >= 0) {
      float a7[7];
      for (int k = 0; k < 7; k++)
        a7[k] = as(k);
      const float belly = D::bellyAttitude(a7).belly / kDeg;
      const float ts = t - tEnd;
      if (ts <= 1.0f)
        r.peak1 = std::max(r.peak1, belly);
      else
        r.peakLate = std::max(r.peakLate, belly);
      if (ts <= 1.0f)
        r.belly1 = belly;
      if (uSet) {
        r.overshoot = std::max(r.overshoot, -(ps(3) * ux + ps(4) * uy));
        r.maxExcursion = std::max(r.maxExcursion, sqrtf(ps(3) * ps(3) + ps(4) * ps(4)));
      }
    }
    if (trace && i % 50 == 0 && t < 4.0f) {
      float a7[7];
      for (int k = 0; k < 7; k++)
        a7[k] = as(k);
      printf("KICK trace %s t %5.2f belly %5.1f rate %+6.1f | x %+5.2f y %+5.2f z %6.2f v %+4.1f %+4.1f %+4.1f | thrust %.1f flaps %2.0f %2.0f %2.0f %2.0f %s\n",
             rule.name, t, D::bellyAttitude(a7).belly / kDeg, -as(1) / kDeg, ps(3), ps(4), ps(5), ps(0), ps(1), ps(2),
             motorOn ? thrust.magnitude() : 0.0f, m.flapAngle(0) / kDeg, m.flapAngle(1) / kDeg,
             m.flapAngle(2) / kDeg, m.flapAngle(3) / kDeg, kicking ? "kick" : (flapsOn ? "flaps" : "coast"));
    }
    if (i == maxSteps - 1)
      r.timeout = true;
  }
  if (tEnd < 0) { // never ended (cannot happen: the time limit)
    r.timeout = true;
  }
  return r;
}

// ---------------------------------------------------------------------------------------------------------------
struct Class {
  const char *name;
  float wind, gust;
  int seeds;
};

struct Agg {
  std::vector<float> tK, bE, rE, vE, hE, p1, pL, b1, miss, vh, ov, ex, zD;
  int n = 0, fade = 0, tumble = 0, tmo = 0, nTime = 0, nPred = 0, nAngle = 0, nRate = 0;
};

static float med(std::vector<float> v) {
  if (v.empty())
    return 0;
  std::sort(v.begin(), v.end());
  return v[v.size() / 2];
}
static float pct90(std::vector<float> v) {
  if (v.empty())
    return 0;
  std::sort(v.begin(), v.end());
  return v[std::min(v.size() - 1, size_t(0.9f * v.size()))];
}
static float mx(const std::vector<float> &v) { return v.empty() ? 0 : *std::max_element(v.begin(), v.end()); }

static void add(Agg &a, const Result &r) {
  a.n++;
  a.tK.push_back(r.tKick), a.bE.push_back(r.bellyEnd), a.rE.push_back(r.rateEnd), a.vE.push_back(r.speedEnd);
  a.hE.push_back(r.horizEnd), a.p1.push_back(r.peak1), a.pL.push_back(r.peakLate), a.b1.push_back(r.belly1);
  a.miss.push_back(r.miss), a.vh.push_back(r.vhFlip), a.ov.push_back(r.overshoot), a.ex.push_back(r.maxExcursion);
  a.zD.push_back(r.zDrop);
  if (r.belly1 > 35.0f)
    a.fade++;
  if (std::max(r.peakLate, r.peak1) > 90.0f || r.bellyFlip > 60.0f || r.timeout)
    a.tumble++;
  a.tmo += r.timeout;
  a.nTime += r.trig == S::KickEnd::Time, a.nPred += r.trig == S::KickEnd::Predicted;
  a.nAngle += r.trig == S::KickEnd::Angle, a.nRate += r.trig == S::KickEnd::Rate;
}

static int seedsPerClass() {
  const char *e = getenv("KICK_SEEDS");
  int n = e ? atoi(e) : 4;
  return n < 1 ? 1 : n;
}

static S::KickEndParams base() { return S::kKickEndDefault; }

static Rule mkOld(float angDeg) {
  static char names[8][24];
  static int k = 0;
  S::KickEndParams p = base();
  p.predictorMode = 0;
  p.angleRad = angDeg * kDeg;
  char *n = names[k++ % 8];
  snprintf(n, 24, "angle%.0f", angDeg);
  return {n, p};
}

// Sweep rules are named into a pool that outlives the run.
static std::vector<char *> gNames;
static Rule mkPred(int mode, float param, float gateDeg, float targetDeg = 0) {
  S::KickEndParams p = base();
  p.predictorMode = mode;
  if (mode == 1)
    p.leadS = param;
  else
    p.brakeDecelRadPs2 = param;
  p.targetRad = gateDeg * kDeg; // the remaining-angle limit
  (void)targetDeg;
  char *n = (char *)malloc(40);
  snprintf(n, 40, "%s%.2f/L%.0f", mode == 1 ? "lead" : "ball", param, gateDeg);
  gNames.push_back(n);
  return {n, p};
}

static void printHeader() {
  printf("KICK %-18s %-8s %5s %5s %5s %5s | %5s %5s %5s | %5s %5s | %4s %4s %4s | %5s %5s %5s %5s | %s\n", "rule", "class",
         "tKick", "bEnd", "rate", "vEnd", "peak1", "pkLat", "b@1s", "miss", "p90", "ovsh", "excu", "vhFl", "zDrop", "fade",
         "tumbl", "n", "end(A/P/R/T)");
}

static void runRule(const Rule &rule, bool perClass) {
  const Class cls[] = {{"calm", 0, 0, 0}, {"wind3", 3, 3, 0}, {"wind5", 5, 3, 0}};
  const int ns = seedsPerClass();
  Agg all;
  for (const Class &c : cls) {
    Agg a;
    for (int s = 1; s <= ns; s++) {
      Scenario sc;
      sc.wind = c.wind, sc.gust = c.gust, sc.seed = s;
      Result r = runKick(rule, sc);
      add(a, r);
      add(all, r);
    }
    if (perClass) {
      printf("KICK %-18s %-8s %5.2f %5.1f %5.0f %5.1f | %5.1f %5.1f %5.1f | %5.1f %5.1f | %4.1f %4.1f %4.1f | %5.2f %5d %5d %5d | %d/%d/%d/%d\n",
             rule.name, c.name, med(a.tK), med(a.bE), med(a.rE), med(a.vE), med(a.p1), med(a.pL), med(a.b1), med(a.miss),
             pct90(a.miss), med(a.ov), med(a.ex), med(a.vh), med(a.zD), a.fade, a.tumble, a.n, a.nAngle, a.nPred, a.nRate,
             a.nTime);
    }
  }
  printf("KICK %-18s %-8s %5.2f %5.1f %5.0f %5.1f | %5.1f %5.1f %5.1f | %5.1f %5.1f | %4.1f %4.1f %4.1f | %5.2f %5d %5d %5d | %d/%d/%d/%d\n",
         rule.name, "ALL", med(all.tK), med(all.bE), med(all.rE), med(all.vE), med(all.p1), med(all.pL), med(all.b1),
         med(all.miss), pct90(all.miss), med(all.ov), med(all.ex), med(all.vh), med(all.zD), all.fade, all.tumble, all.n,
         all.nAngle, all.nPred, all.nRate, all.nTime);
  (void)mx;
}

static void test_trace() {
  const char *e = getenv("KICK_TRACE");
  if (!e)
    return;
  Rule r = mkOld(15);
  const float ang = atof(e);
  if (ang > 0)
    r = mkOld(ang);
  Scenario sc;
  runKick(r, sc, true);
}

static void test_sweep() {
  const char *e = getenv("KICK_SWEEP");
  if (!e)
    return;
  printHeader();
  const bool perClass = strstr(e, "pc") != nullptr;
  const float gates[] = {40, 50, 60};
  runRule(mkOld(15), perClass);
  for (float g : gates)
    runRule(mkOld(g), perClass);
  if (strstr(e, "only"))
    return;
  if (strstr(e, "chosen")) {
    for (float g : gates)
      runRule(mkPred(2, 4.0f, g), perClass);
    return;
  }
  const float leads[] = {0.05f, 0.10f, 0.15f, 0.20f, 0.30f};
  const float alphas[] = {2, 3, 4, 6, 10};
  for (float g : gates)
    for (float l : leads)
      runRule(mkPred(1, l, g), perClass);
  for (float g : gates)
    for (float a : alphas)
      runRule(mkPred(2, a, g), perClass);
}

// Acceptance, calm air: the production default ends the kick by the predictor well before the 1 s limit and the
// ship settles belly-down; the legacy parameters still give the old 15 deg end near 0.95 s.
static void test_defaults_and_legacy() {
  Scenario sc;
  Result d = runKick({"default", S::kKickEndDefault}, sc);
  TEST_ASSERT_TRUE(d.trig == S::KickEnd::Predicted);
  TEST_ASSERT_TRUE(d.tKick > 0.2f && d.tKick < 0.7f);
  TEST_ASSERT_TRUE(d.belly1 < 35.0f);
  TEST_ASSERT_TRUE(d.peakLate < 35.0f);
  TEST_ASSERT_FALSE(d.timeout);
  Result l = runKick({"legacy", S::kKickEndLegacy}, sc);
  TEST_ASSERT_TRUE(l.trig == S::KickEnd::Angle);
  TEST_ASSERT_TRUE(l.tKick > 0.85f && l.tKick <= 1.011f);
  TEST_ASSERT_TRUE(d.tKick < l.tKick);
  // The hard limit holds even when nothing triggers (predictor off, angle never reached: huge limit angle 0).
  S::KickEndParams p = S::kKickEndLegacy;
  p.angleRad = 0.0f;
  Result t = runKick({"limit", p}, sc);
  TEST_ASSERT_TRUE(t.trig == S::KickEnd::Time || t.trig == S::KickEnd::Rate);
  TEST_ASSERT_TRUE(t.tKick <= 1.011f);
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_defaults_and_legacy);
  RUN_TEST(test_trace);
  RUN_TEST(test_sweep);
  return UNITY_END();
}
