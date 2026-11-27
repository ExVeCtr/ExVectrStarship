// Golden equivalence test of the new simulator architecture (ExVectrDynamics + src/sim StarshipPlant) against a FROZEN
// copy of the legacy DSP::BodySimModel (test/frozen/legacy_body_sim_model.hpp, ExVectrDSP 5aba4c6 + ExVectrControl
// 2c0c00e). Scenarios E1-E8 of docs/project/simulation-architecture.md section 7.2, plus the vehicle variant (V1) and
// the standalone aero function. Criterion: BIT-IDENTICAL (memcmp) state, flap angles, TVC angles, aero loads and time
// after every step, on the host. Run: pio test -e native -f test_sim_equivalence
#include <math.h>
#include <new>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unity.h>

#include "../frozen/legacy_body_sim_model.hpp"
#include "ExVectrDynamics/sim.hpp"
#include "sim/starship_param_sets.hpp"
#include "sim/starship_plant.hpp"

using namespace VCTR;
using Dyn::Vec3;
using Legacy = legacy_frozen::BodySimModel;
using LegacyParams = legacy_frozen::BodySimParams;
using ShipSim::StarshipPlant;
using ShipSim::StarshipPlantHarness;
using ShipSim::StarshipPlantParams;

void setUp() {}
void tearDown() {}

namespace {

constexpr float kDeg = 3.14159265f / 180.0f;
constexpr float kHalfPi = 1.5708f;

struct Rng {
  uint32_t s;
  explicit Rng(uint32_t seed) : s(seed ? seed : 1) {}
  uint32_t next() {
    s ^= s << 13;
    s ^= s >> 17;
    s ^= s << 5;
    return s;
  }
  float uni(float lo, float hi) { return lo + (hi - lo) * (float(next() >> 8) / 16777216.0f); }
  bool coin() { return next() & 1; }
  int pick(int n) { return int(next() % uint32_t(n)); }
};

template <class T> bool sameBits(const T &a, const T &b) { return memcmp(&a, &b, sizeof(T)) == 0; }

/// Same parameters in the legacy and the new representation (field by field).
StarshipPlantParams fromLegacy(const LegacyParams &l) {
  StarshipPlantParams p;
  p.mass_kg = l.mass_kg;
  p.inertiaXX = l.inertiaXX;
  p.inertiaYY = l.inertiaYY;
  p.inertiaZZ = l.inertiaZZ;
  p.inertiaXY = l.inertiaXY;
  p.inertiaXZ = l.inertiaXZ;
  p.inertiaYZ = l.inertiaYZ;
  p.hull.radius_m = l.bodyRadius_m;
  p.hull.tubeLength_m = l.tubeLength_m;
  p.hull.noseLength_m = l.noseLength_m;
  p.hull.cgFromBottom_m = l.cgFromBottom_m;
  p.hull.strips = l.bodyStrips;
  p.hullAero.cdCrossflow = l.cdCrossflow;
  p.hullAero.crossflowEta = l.crossflowEta;
  p.hullAero.potentialFactor = l.potentialFactor;
  p.hullAero.cdAxialTailFirst = l.cdAxialTailFirst;
  p.hullAero.cdAxialNoseFirst = l.cdAxialNoseFirst;
  p.flapTop = {l.flapTop.width_m, l.flapTop.height_m, l.flapTop.chamfer_m, l.flapTop.bottomFromShipBottom_m,
               l.flapTop.effectiveness, l.topFlapCantRad};
  p.flapBottom = {l.flapBottom.width_m, l.flapBottom.height_m, l.flapBottom.chamfer_m,
                  l.flapBottom.bottomFromShipBottom_m, l.flapBottom.effectiveness, 0.0f};
  p.flapAero.cnAlpha = l.flapCnAlpha;
  p.flapAero.cn90 = l.flapCn90;
  p.flapAero.stallAngle_rad = l.flapStallAngle_rad;
  p.flapAero.stallWidth_rad = l.flapStallWidth_rad;
  p.flapAero.effectiveMinAngle_rad = l.flapEffectiveMinAngleRad;
  p.flapAero.effectiveSmooth_rad = l.flapEffectiveSmoothRad;
  p.flapAero.effectiveRetractMargin_rad = l.flapEffectiveRetractMarginRad;
  p.flapAero.sidewashGain = l.flapSidewashGain;
  p.flapAero.bodyCrossflowGain = l.flapBodyCrossflowGain;
  p.flapAero.foldShieldFactor = l.flapFoldShieldFactor;
  p.flapFoldTowardBelly = l.flapFoldTowardBelly;
  p.hingeRadius_m = l.flapHingeRadius_m;
  p.aeroCalibration = l.aeroCalibration;
  p.air.rho_kgm3 = l.airDensity_kgm3;
  p.wind.speed_mps = l.windSpeed_mps;
  p.wind.direction_rad = l.windDirection_rad;
  p.wind.gustStrength_mps = l.gustStrength_mps;
  p.wind.gustPeriod_s = l.gustPeriod_s;
  p.wind.gustSeed = l.gustSeed;
  p.wind.gustSpread = l.gustSpread;
  p.tvc.thrustLimit_N = l.tvcThrustLimit_N;
  p.tvc.angleLimit_rad = l.tvcAngleLimit_rad;
  p.tvc.fromCg_m = l.tvcFromCg_m;
  p.tvc.servoRate_radps = l.tvcServoRate_radps;
  p.tvc.servoToThrustGain = l.tvcServoToThrustGain;
  p.flapServo.rate_radps = l.flapServoRate_radps;
  p.flapServo.lag_s = l.flapServoLag_s;
  p.flapServo.deadTime_s = l.flapServoDeadTime_s;
  p.flapServo.minAngle_rad = l.flapMinAngle_rad;
  p.flapServo.maxAngle_rad = l.flapMaxAngle_rad;
  p.applyActuatorModels = l.applyActuatorRateLimits;
  p.step_s = l.step_s;
  p.maxAdvance_s = l.maxAdvance_s;
  return p;
}

/// Legacy model and new plant side by side, driven with the same inputs; every comparison is a memcmp.
template <class Plant> struct Pair {
  Legacy lg;
  typename Plant::Config cfg;
  Dyn::Sim<Plant> ns;
  typename Plant::Input u;
  long steps = 0;
  bool inputPending = false; ///< An input was set since the last step (the plant takes inputs inside step).
  const char *name;

  // The Sim keeps a pointer to cfg: a Pair must not be copied or moved.
  Pair(const char *scenario, const LegacyParams &lp, const StarshipPlantParams &sp)
      : lg(lp), cfg(Plant::configure(sp)), ns(cfg), name(scenario) {
    u.thrustBody = {0, 0, 0};
    u.rollTorque = 0.0f;
    u.thrustEnabled = false;
    for (int i = 0; i < 4; i++)
      u.flapAngle[i] = 1.5708f; // the legacy default flap input
    check();
  }
  explicit Pair(const char *scenario, const LegacyParams &lp = LegacyParams()) : Pair(scenario, lp, fromLegacy(lp)) {}
  Pair(const Pair &) = delete;
  Pair &operator=(const Pair &) = delete;

  void fail(const char *what) {
    char buf[200];
    snprintf(buf, sizeof(buf), "%s: %s differs from the legacy model at step %ld", name, what, steps);
    TEST_FAIL_MESSAGE(buf);
  }

  void check() {
    const typename Plant::State &x = ns.state();
    const typename Plant::Output &y = ns.output();
    if (!sameBits(lg.attitudeState(), x.body.att)) fail("attitude state");
    if (!sameBits(lg.positionState(), x.body.pos)) fail("position state");
    for (int i = 0; i < 4; i++) {
      if (!sameBits(lg.flapAngle(i), x.flaps.angle[i])) fail("flap angle");
      if (!sameBits(lg.flapAngle(i), y.flapAngle[i]) && steps > 0) fail("flap angle output");
      if (!sameBits(lg.flapRateLimited_[i], x.flaps.rateLimited[i])) fail("flap rate-limited angle");
      if (!inputPending && !sameBits(lg.flapInput_[i], x.flaps.input[i])) fail("held flap input");
    }
    if (!sameBits(lg.tvcAngleX(), x.tvc.angleX) || !sameBits(lg.tvcAngleY(), x.tvc.angleY)) fail("TVC angle");
    if (!inputPending) {
      if (!sameBits(lg.thrustCmd_, x.thrustCmd) || !sameBits(lg.thrustMag_, x.thrustMag)) fail("held thrust");
      if (!sameBits(lg.rollTorque_, x.rollTorque) || lg.thrustEnabled_ != x.thrustEnabled) fail("held roll/enable");
    }
    if (!sameBits(lg.time(), x.t_s)) fail("time");
    if (steps > 0) {
      const legacy_frozen::BodySimLoads &l = lg.loads();
      if (!sameBits(l.aeroForce, y.aero.force)) fail("aero force");
      if (!sameBits(l.aeroTorque, y.aero.torque)) fail("aero torque");
      if (!sameBits(l.airVelocity, y.airVelBody)) fail("air velocity");
      for (int i = 0; i < 4; i++)
        if (!sameBits(l.flapForce[i], y.flapForce[i])) fail("flap force");
      if (!sameBits(lg.tvcAngleX(), y.tvcAngle[0]) || !sameBits(lg.tvcAngleY(), y.tvcAngle[1])) fail("TVC angle output");
    }
  }

  void setState(const Math::Vector<float, 7> &att, const Math::Vector<float, 6> &pos) {
    lg.setAttitudeState(att);
    lg.setPositionState(pos);
    ns.state().body.att = att;
    ns.state().body.pos = pos;
  }
  void setThrust(const Vec3 &f, float roll, bool en) {
    lg.setThrust(f, roll, en);
    u.thrustBody = f;
    u.rollTorque = roll;
    u.thrustEnabled = en;
    inputPending = true;
  }
  void setFlaps(float a, float b, float c, float d) {
    lg.setFlapInput(a, b, c, d);
    u.flapAngle[0] = a;
    u.flapAngle[1] = b;
    u.flapAngle[2] = c;
    u.flapAngle[3] = d;
    inputPending = true;
  }
  void setWind(float s, float d) {
    lg.setWind(s, d);
    Plant::setWind(cfg, s, d);
  }
  void setGust(float s, float p, uint32_t seed) {
    lg.setGust(s, p, seed);
    Plant::setGust(cfg, s, p, seed);
  }
  void clearWind() {
    lg.clearWind();
    Plant::clearWind(cfg);
  }
  void step() {
    lg.step();
    ns.step(u);
    steps++;
    inputPending = false;
    check();
  }
  void advance(float elapsed) {
    const int a = lg.advance(elapsed);
    const int b = ns.advance(elapsed, u);
    steps++;
    if (a != b) fail("advance step count");
    if (a > 0) inputPending = false;
    const auto snap = ns.snapshot();
    if (!sameBits(lg.carry_s_, snap.carry_s)) fail("time carry");
    check();
  }
  bool grounded() { return lg.positionState()(5) <= 0.0f; }
};

Math::Vector<float, 7> bellyDownAttitude(float pitchOff, float yawOff) {
  // Body -X (belly) points down: rotation of -90 deg about Y (body X -> world Z up), nose toward -X, plus offsets.
  Math::Quat_F q(Math::Vector<float, 3>{0, 1, 0}, -90.0f * kDeg);
  q = q * Math::Quat_F(Math::Vector<float, 3>{1, 0, 0}, pitchOff);
  q = q * Math::Quat_F(Math::Vector<float, 3>{0, 0, 1}, yawOff);
  return {0, 0, 0, q(0), q(1), q(2), q(3)};
}

Vec3 randomThrust(Rng &r) {
  switch (r.pick(10)) {
  case 0: return Vec3{NAN, 1, 2};
  case 1: return Vec3{0, INFINITY, 5};
  case 2: return Vec3{0, 0, 0};
  case 3: return Vec3{0, 0, r.uni(-20, 20)};
  case 4: return Vec3{r.uni(-5, 5), r.uni(-5, 5), r.uni(0, 25)}; // above 14 N and beyond 3 deg
  case 5: return Vec3{1e-9f, 0, 1e-7f};
  default: return Vec3{r.uni(-0.6f, 0.6f), r.uni(-0.6f, 0.6f), r.uni(6, 14)};
  }
}

} // namespace

// E1: upright, thrust = m g, flaps folded, 10 s: rigid body, thrust, axial aero.
void test_E1_hover_upright() {
  Pair<StarshipPlantHarness> p("E1");
  p.setThrust(Vec3{0, 0, 1.1f * 9.807f}, 0.0f, true);
  for (int i = 0; i < 5000; i++)
    p.step();
  TEST_ASSERT_TRUE(fabsf(p.ns.state().body.pos(5)) < 0.5f); // it really hovers
}

// E2: scripted open-loop ascent, wind 3 m/s + gusts 3 m/s (seed 7), 10 s.
void test_E2_ascent_wind_gusts() {
  Pair<StarshipPlantHarness> p("E2");
  p.setWind(3.0f, 0.7f);
  p.setGust(3.0f, 5.0f, 7);
  p.setState({0, 0, 0, 0.99f, 0.05f, -0.1f, 0.0f}, {0, 0, 0, 0, 0, 0});
  for (int i = 0; i < 5000; i++) {
    if (i % 10 == 0) {
      const float t = 0.002f * float(i);
      p.setThrust(Vec3{0.5f * sinf(2.0f * t), 0.4f * cosf(1.3f * t), 11.5f + 2.0f * sinf(t)}, 0.01f * sinf(3.0f * t),
                  true);
    }
    p.step();
  }
}

// E3: belly-down fall from 120 m at 50/40 deg with pitch/yaw offsets, until resting on the ground (arrival and resting).
static void runE3(const char *name, float pitchOff, float yawOff, float wind, bool vehicle) {
  LegacyParams lp;
  lp.applyActuatorRateLimits = !vehicle;
  if (vehicle) {
    Pair<StarshipPlant> p(name, lp);
    p.setWind(wind, 1.0f);
    p.setState(bellyDownAttitude(pitchOff, yawOff), {0, 0, 0, 0, 0, 120});
    p.setFlaps(50 * kDeg, 50 * kDeg, 40 * kDeg, 40 * kDeg);
    int rest = 0;
    for (int i = 0; i < 40000 && rest < 1500; i++) {
      p.step();
      if (p.grounded()) rest++;
    }
    TEST_ASSERT_TRUE(rest >= 1500);
    return;
  }
  Pair<StarshipPlantHarness> p(name, lp);
  p.setWind(wind, 1.0f);
  if (wind > 0.0f) p.setGust(2.0f, 4.0f, 3);
  p.setState(bellyDownAttitude(pitchOff, yawOff), {0, 0, 0, 0, 0, 120});
  p.setFlaps(50 * kDeg, 50 * kDeg, 40 * kDeg, 40 * kDeg);
  int rest = 0;
  for (int i = 0; i < 40000 && rest < 1500; i++) {
    p.step();
    if (p.grounded()) rest++;
  }
  TEST_ASSERT_TRUE_MESSAGE(rest >= 1500, "the body never came to rest on the ground");
}
void test_E3_belly_fall_to_ground() {
  runE3("E3a", 0.0f, 0.0f, 0.0f, false);
  runE3("E3b", 8.0f * kDeg, 0.0f, 0.0f, false);
  runE3("E3c", 0.0f, -6.0f * kDeg, 2.0f, false);
  runE3("E3d", 5.0f * kDeg, 5.0f * kDeg, 0.0f, false);
}

// E4: flap sweeps 0-90 deg, out-of-range and NaN inputs mixed in (input hold and clamps).
void test_E4_flap_sweeps_and_bad_inputs() {
  Pair<StarshipPlantHarness> p("E4");
  p.setState(bellyDownAttitude(3.0f * kDeg, 2.0f * kDeg), {0, 0, 0, 0, 0, 120});
  Rng r(44);
  int updates = 0;
  for (int i = 0; i < 4000; i++) {
    if (i % 25 == 0) {
      const float t = 0.002f * float(i);
      float a[4];
      for (int k = 0; k < 4; k++)
        a[k] = 0.5f * kHalfPi * (1.0f + sinf(t * (1.0f + 0.7f * float(k)) + float(k)));
      if (++updates % 7 == 0) { // bad values
        a[r.pick(4)] = NAN;
        a[r.pick(4)] = -0.5f;
        a[r.pick(4)] = 3.0f;
        if (r.coin()) a[r.pick(4)] = INFINITY;
        if (r.coin()) a[r.pick(4)] = -INFINITY;
      }
      p.setFlaps(a[0], a[1], a[2], a[3]);
    }
    p.step();
  }
}

// E5: thrust above 14 N, beyond 3 deg, NaN, enable toggles (TVC clamp and gating).
void test_E5_thrust_clamp_and_gating() {
  for (int variant = 0; variant < 2; variant++) {
    LegacyParams lp;
    lp.applyActuatorRateLimits = variant == 0; // harness (slew) and vehicle (pass-through)
    Pair<StarshipPlantHarness> p("E5", lp);
    p.setState({0, 0, 0, 1, 0, 0, 0}, {0, 0, 1, 0, 0, 50});
    Rng r(55 + uint32_t(variant));
    bool en = true;
    for (int i = 0; i < 3000; i++) {
      if (i % 20 == 0) {
        if (r.pick(4) == 0) en = !en;
        float roll = r.uni(-0.05f, 0.05f);
        if (r.pick(8) == 0) roll = NAN;
        p.setThrust(randomThrust(r), roll, en);
      }
      p.step();
    }
  }
  // Vehicle variant (capacity 1 servo line, pass-through actuators).
  LegacyParams lp;
  lp.applyActuatorRateLimits = false;
  Pair<StarshipPlant> v("V1-E5", lp);
  v.setState({0, 0, 0, 1, 0, 0, 0}, {0, 0, 1, 0, 0, 50});
  Rng r(56);
  bool en = true;
  for (int i = 0; i < 3000; i++) {
    if (i % 20 == 0) {
      if (r.pick(4) == 0) en = !en;
      v.setThrust(randomThrust(r), r.uni(-0.05f, 0.05f), en);
      if (i % 100 == 0)
        v.setFlaps(r.uni(0, kHalfPi), r.uni(0, kHalfPi), r.uni(0, kHalfPi), r.uni(0, kHalfPi));
    }
    v.step();
  }
}

// E6: advance with 0, negative, 1, 9.7, 49, 80 ms and NaN elapsed; clearTimeCarry; resetPose mid-run.
void test_E6_advance_carry_and_reset() {
  Pair<StarshipPlantHarness> p("E6");
  p.setState({0, 0, 0, 1, 0, 0, 0}, {0, 0, 0, 0, 0, 20});
  static const float kElapsed[] = {0.0f, -0.001f, 0.001f, 0.0097f, 0.049f, 0.08f, NAN, 0.002f, 0.0021f, 0.0199f, INFINITY};
  Rng r(66);
  for (int k = 0; k < 600; k++) {
    if (k % 10 == 0) {
      p.setThrust(Vec3{r.uni(-0.5f, 0.5f), r.uni(-0.5f, 0.5f), r.uni(8, 13)}, r.uni(-0.02f, 0.02f), true);
      p.setFlaps(r.uni(0, kHalfPi), r.uni(0, kHalfPi), r.uni(0, kHalfPi), r.uni(0, kHalfPi));
    }
    if (k == 150) {
      p.lg.clearTimeCarry();
      p.ns.clearTimeCarry();
    }
    if (k == 300) { // pose reset only: time, actuators and the carry continue
      p.lg.resetToStartPose();
      StarshipPlantHarness::resetPose(p.ns.state());
    }
    p.advance(kElapsed[r.pick(11)]);
  }
  TEST_ASSERT_TRUE(p.ns.state().t_s > 0.5);
}

// E7: harness mode: rate limits on, lag 20 ms, dead time 30 ms (servo and TVC slew models).
void test_E7_harness_servo_models() {
  for (int start = 0; start < 2; start++) {
    LegacyParams lp;
    lp.flapServoLag_s = 0.02f;
    lp.flapServoDeadTime_s = 0.03f;
    Pair<StarshipPlantHarness> p("E7", lp);
    if (start == 0)
      p.setState(bellyDownAttitude(2.0f * kDeg, 0.0f), {0, 0, 0, 0, 0, 120});
    else
      p.setState({0, 0, 0, 1, 0, 0, 0}, {0, 0, 0, 0, 0, 30});
    Rng r(77 + uint32_t(start));
    for (int i = 0; i < 3000; i++) {
      if (i % 40 == 0)
        p.setFlaps(r.uni(0, kHalfPi), r.uni(0, kHalfPi), r.uni(0, kHalfPi), r.uni(0, kHalfPi));
      if (i % 30 == 0)
        p.setThrust(Vec3{r.uni(-0.6f, 0.6f), r.uni(-0.6f, 0.6f), r.uni(8, 14)}, r.uni(-0.03f, 0.03f), true);
      p.step();
    }
  }
}

// E8: setWind/setGust/clearWind mid-run (runtime environment changes).
void test_E8_runtime_environment_changes() {
  Pair<StarshipPlantHarness> p("E8");
  p.setState(bellyDownAttitude(0.0f, 0.0f), {0, 0, 0, 0, 0, 150});
  p.setFlaps(50 * kDeg, 50 * kDeg, 40 * kDeg, 40 * kDeg);
  for (int i = 0; i < 5000; i++) {
    switch (i) {
    case 300: p.setWind(4.0f, 1.0f); break;
    case 800: p.setGust(2.0f, 3.0f, 11); break;
    case 1500: p.setGust(5.0f, 10.0f, 2); break;
    case 2000: p.setWind(NAN, INFINITY); break;   // ignored
    case 2200: p.setGust(NAN, NAN, 5); break;     // only the seed changes
    case 2600: p.clearWind(); break;
    case 3000: p.setWind(-2.0f, 0.3f); break;     // negative speed becomes 0
    case 3300: p.setGust(50.0f, 100.0f, 0); break; // clamped, seed 0
    case 4000: p.setWind(6.0f, -2.0f); break;
    default: break;
    }
    p.step();
  }
}

// The default plant parameters equal the legacy defaults (no converter involved), including the derived values.
void test_defaults_equal_legacy_defaults() {
  Pair<StarshipPlantHarness> p("defaults", LegacyParams(), StarshipPlantParams());
  p.setThrust(Vec3{0.1f, -0.1f, 11.0f}, 0.0f, true);
  p.setFlaps(0.8f, 0.9f, 0.7f, 0.6f);
  p.setWind(2.0f, 0.5f);
  p.setGust(1.0f, 5.0f, 3);
  for (int i = 0; i < 2000; i++)
    p.step();
  const legacy_frozen::BodySimParams l;
  const StarshipPlantParams d;
  const StarshipPlantHarness::Config c = StarshipPlantHarness::configure(d);
  TEST_ASSERT_TRUE(sameBits(c.d.aero.area_m2[0], p.lg.flapArea_[0]));
  TEST_ASSERT_TRUE(sameBits(c.d.aero.area_m2[1], p.lg.flapArea_[1]));
  TEST_ASSERT_TRUE(sameBits(c.d.aero.sc_m[0], p.lg.flapSc_[0]));
  TEST_ASSERT_TRUE(sameBits(c.d.aero.sc_m[1], p.lg.flapSc_[1]));
  TEST_ASSERT_TRUE(sameBits(c.d.aero.zLow_m[0], p.lg.flapZLow_[0]));
  TEST_ASSERT_TRUE(sameBits(c.d.aero.zLow_m[1], p.lg.flapZLow_[1]));
  TEST_ASSERT_TRUE(sameBits(c.d.body.inertiaInv, p.lg.body_.getInertiaTensor().inverse()));
  TEST_ASSERT_EQUAL_FLOAT(l.aeroCalibration, d.aeroCalibration);
}

// Random parameter sets (geometry, coefficients, candidate effects, actuators) in the same run.
void test_random_parameters_equivalence() {
  Rng r(99);
  for (int trial = 0; trial < 12; trial++) {
    LegacyParams lp;
    lp.mass_kg = r.uni(0.7f, 1.6f);
    lp.inertiaXX = lp.inertiaYY = r.uni(0.03f, 0.06f);
    lp.inertiaXY = r.uni(-0.002f, 0.002f);
    lp.inertiaXZ = r.uni(-0.001f, 0.001f);
    lp.flapFoldTowardBelly = r.coin();
    lp.aeroCalibration = r.uni(1.0f, 6.0f);
    lp.flapSidewashGain = r.coin() ? r.uni(-1, 1) : 0.0f;
    lp.flapBodyCrossflowGain = r.coin() ? r.uni(0, 1) : 0.0f;
    lp.flapEffectiveRetractMarginRad = r.coin() ? r.uni(0.05f, 0.3f) : 0.0f;
    lp.potentialFactor = r.uni(0.0f, 1.5f);
    lp.bodyStrips = 4 + r.pick(61);
    lp.flapServoLag_s = r.coin() ? 0.0f : r.uni(0.005f, 0.04f);
    lp.flapServoDeadTime_s = r.uni(0.0f, 0.06f);
    Pair<StarshipPlantHarness> p("random", lp);
    p.setState(bellyDownAttitude(r.uni(-0.2f, 0.2f), r.uni(-0.2f, 0.2f)), {0, 0, 0, r.uni(-2, 2), r.uni(-2, 2), 60});
    p.setWind(r.uni(0, 5), r.uni(-3, 3));
    p.setGust(r.uni(0, 4), r.uni(1, 8), uint32_t(r.pick(50)));
    for (int i = 0; i < 1500; i++) {
      if (i % 30 == 0)
        p.setFlaps(r.uni(0, kHalfPi), r.uni(0, kHalfPi), r.uni(0, kHalfPi), r.uni(0, kHalfPi));
      p.step();
    }
  }
}

// ---- starshipAero as a standalone, stateless callable -------------------------------------------------------------
void test_starship_aero_standalone_equals_legacy_and_is_stateless() {
  Rng r(123);
  const StarshipPlantParams sp;
  const ShipSim::AeroConfig cfg = ShipSim::makeAeroConfig(sp);
  const ShipSim::AeroConfig cfgCopy = cfg;
  for (int trial = 0; trial < 3000; trial++) {
    Legacy lg{LegacyParams()};
    lg.setWind(r.uni(0, 6), r.uni(-3, 3));
    Math::Vector<float, 7> att = {r.uni(-4, 4), r.uni(-4, 4), r.uni(-4, 4), r.uni(-1, 1), r.uni(-1, 1), r.uni(-1, 1), r.uni(-1, 1)};
    Math::Quat_F q = Math::Vector<float, 4>{att(3), att(4), att(5), att(6)};
    q = q.normalize();
    att(3) = q(0); att(4) = q(1); att(5) = q(2); att(6) = q(3);
    const Math::Vector<float, 6> pos = {r.uni(-10, 10), r.uni(-10, 10), r.uni(-10, 10), 0, 0, 0};
    lg.setAttitudeState(att);
    lg.setPositionState(pos);
    for (int i = 0; i < 4; i++)
      lg.flapAngle_[i] = r.pick(6) == 0 ? (r.coin() ? 0.0f : kHalfPi) : r.uni(0, kHalfPi);
    lg.computeAero();

    ShipSim::AeroInput in;
    const Math::Quat_F qb = att.block<4, 1>(3, 0);
    in.airVelBody = ShipSim::airVelocityBody(qb, pos.block<3, 1>(0, 0), lg.windWorld());
    in.omegaBody = att.block<3, 1>(0, 0);
    for (int i = 0; i < 4; i++)
      in.flapAngle[i] = lg.flapAngle_[i];
    in.rho = sp.air.rho_kgm3;
    ShipSim::AeroOutput a, b;
    ShipSim::starshipAero(cfg, in, a);
    ShipSim::starshipAero(cfg, in, b);
    TEST_ASSERT_TRUE(a.valid);
    TEST_ASSERT_TRUE(sameBits(in.airVelBody, lg.loads().airVelocity));
    TEST_ASSERT_TRUE(sameBits(a.total.force, lg.loads().aeroForce));
    TEST_ASSERT_TRUE(sameBits(a.total.torque, lg.loads().aeroTorque));
    for (int i = 0; i < 4; i++)
      TEST_ASSERT_TRUE(sameBits(a.flapForce[i], lg.loads().flapForce[i]));
    // Deterministic, no hidden state (field by field: the struct has padding).
    TEST_ASSERT_TRUE(sameBits(a.total.force, b.total.force) && sameBits(a.total.torque, b.total.torque));
    TEST_ASSERT_TRUE(sameBits(a.hull.force, b.hull.force) && sameBits(a.flaps.torque, b.flaps.torque));
    TEST_ASSERT_TRUE(sameBits(a.alpha_rad, b.alpha_rad) && a.valid == b.valid);
    // The split is consistent (to rounding).
    for (int i = 0; i < 3; i++)
      TEST_ASSERT_FLOAT_WITHIN(1e-3f * (1.0f + fabsf(a.total.force(i))), a.total.force(i),
                               a.hull.force(i) + a.flaps.force(i));
  }
  // The config is never written (it is const; this also guards against a future const_cast).
  TEST_ASSERT_TRUE(sameBits(cfg.calibration, cfgCopy.calibration) && sameBits(cfg.hull.d, cfgCopy.hull.d));
  TEST_ASSERT_TRUE(sameBits(cfg.area_m2, cfgCopy.area_m2) && sameBits(cfg.sc_m, cfgCopy.sc_m) &&
                   sameBits(cfg.zLow_m, cfgCopy.zLow_m));
}

void test_starship_aero_invalid_inputs_and_alpha() {
  const ShipSim::AeroConfig cfg = ShipSim::makeAeroConfig(StarshipPlantParams());
  ShipSim::AeroInput in;
  in.airVelBody = Vec3{1, 2, 3};
  in.omegaBody = Vec3{0, 0, 0};
  for (int i = 0; i < 4; i++)
    in.flapAngle[i] = 0.7f;
  in.rho = 1.2f;
  ShipSim::AeroOutput o;
  ShipSim::starshipAero(cfg, in, o);
  TEST_ASSERT_TRUE(o.valid);
  TEST_ASSERT_TRUE(o.total.force.magnitude() > 0.0f);
  ShipSim::AeroInput bad = in;
  bad.airVelBody(1) = NAN;
  ShipSim::starshipAero(cfg, bad, o);
  TEST_ASSERT_FALSE(o.valid);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, o.total.force.magnitude());
  TEST_ASSERT_EQUAL_FLOAT(0.0f, o.total.torque.magnitude());
  bad = in;
  bad.flapAngle[2] = INFINITY;
  ShipSim::starshipAero(cfg, bad, o);
  TEST_ASSERT_FALSE(o.valid);
  bad = in;
  bad.rho = NAN;
  ShipSim::starshipAero(cfg, bad, o);
  TEST_ASSERT_FALSE(o.valid);
  // Angle of attack of the hull: 0 axial, pi/2 broadside.
  in.airVelBody = Vec3{0, 0, 5};
  ShipSim::starshipAero(cfg, in, o);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, o.alpha_rad);
  in.airVelBody = Vec3{5, 0, 0};
  ShipSim::starshipAero(cfg, in, o);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.5708f, o.alpha_rad);
}

// ---- parameter sets, instances side by side, snapshots, sizes, concept, RK4 --------------------------------------
void test_parameter_variants_and_side_by_side_instances() {
  TEST_ASSERT_EQUAL(4, ShipSim::kNumVariants);
  TEST_ASSERT_EQUAL_FLOAT(4.64f, ShipSim::makeParams(ShipSim::kVariants[0]).aeroCalibration);
  TEST_ASSERT_EQUAL_FLOAT(4.64f * 0.7f, ShipSim::makeParams(ShipSim::kVariants[1]).aeroCalibration);
  TEST_ASSERT_EQUAL_FLOAT(4.64f * 1.3f, ShipSim::makeParams(ShipSim::kVariants[2]).aeroCalibration);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, ShipSim::makeParams(ShipSim::kVariants[3]).aeroCalibration);
  // "nominal" is the default parameter set.
  const StarshipPlantHarness::Config nominal = StarshipPlantHarness::configure(ShipSim::makeParams(ShipSim::kVariants[0]));
  const StarshipPlantHarness::Config deflt = StarshipPlantHarness::configure(StarshipPlantParams());
  TEST_ASSERT_TRUE(sameBits(nominal.d.aero.calibration, deflt.d.aero.calibration));

  StarshipPlantHarness::Config cfgs[ShipSim::kNumVariants];
  Dyn::Sim<StarshipPlantHarness> *sims[ShipSim::kNumVariants];
  alignas(Dyn::Sim<StarshipPlantHarness>) static char storage[ShipSim::kNumVariants][sizeof(Dyn::Sim<StarshipPlantHarness>)];
  StarshipPlantHarness::Input u;
  u.thrustBody = {0, 0, 0};
  u.rollTorque = 0.0f;
  u.thrustEnabled = false;
  for (int i = 0; i < 4; i++)
    u.flapAngle[i] = i < 2 ? 50 * kDeg : 40 * kDeg;
  for (int k = 0; k < ShipSim::kNumVariants; k++) {
    cfgs[k] = StarshipPlantHarness::configure(ShipSim::makeParams(ShipSim::kVariants[k]));
    sims[k] = new (storage[k]) Dyn::Sim<StarshipPlantHarness>(cfgs[k]);
    sims[k]->state().body.att = bellyDownAttitude(0.02f, 0.0f);
    sims[k]->state().body.pos(5) = 500.0f;
  }
  for (int i = 0; i < 4000; i++) // 8 s side by side
    for (int k = 0; k < ShipSim::kNumVariants; k++)
      sims[k]->step(u);
  const float vNom = -sims[0]->state().body.pos(2), vLo = -sims[1]->state().body.pos(2),
              vHi = -sims[2]->state().body.pos(2), vGeo = -sims[3]->state().body.pos(2);
  TEST_ASSERT_TRUE(vNom > 4.0f && vNom < 9.0f);  // about 6.4 m/s terminal speed
  TEST_ASSERT_TRUE(vLo > vNom && vNom > vHi);    // less drag falls faster
  TEST_ASSERT_TRUE(vGeo > vNom);                 // uncalibrated geometry falls faster (about 13.5 m/s)
  // Snapshot of one instance continues identically in another instance with the same config.
  const auto snap = sims[0]->snapshot();
  Dyn::Sim<StarshipPlantHarness> copy(cfgs[0]);
  copy.restore(snap);
  for (int i = 0; i < 500; i++) {
    sims[0]->step(u);
    copy.step(u);
  }
  TEST_ASSERT_TRUE(sameBits(sims[0]->state(), copy.state()));
  for (int k = 0; k < ShipSim::kNumVariants; k++)
    sims[k]->~Sim();
}

static_assert(Dyn::is_model<StarshipPlant>::value, "StarshipPlant must satisfy the Model concept");
static_assert(Dyn::is_model<StarshipPlantHarness>::value, "StarshipPlantHarness must satisfy the Model concept");
static_assert(sizeof(StarshipPlant::State) <= 256, "the vehicle plant state must stay small");

void test_state_sizes() {
  printf("SIMSIZE StarshipPlant::State %u B, Harness::State %u B, Config %u B, Sim<StarshipPlant> %u B\n",
         unsigned(sizeof(StarshipPlant::State)), unsigned(sizeof(StarshipPlantHarness::State)),
         unsigned(sizeof(StarshipPlant::Config)), unsigned(sizeof(Dyn::Sim<StarshipPlant>)));
  TEST_ASSERT_TRUE(sizeof(StarshipPlant::State) <= 256);
}

// RK4 at 10 ms against the model's own Euler at 0.5 ms (a loose convergence check of the rollout integrator; there is no
// legacy RK4 to be bit-identical with).
void test_rk4_close_to_fine_euler() {
  StarshipPlantParams pe;
  pe.applyActuatorModels = false; // flaps and thrust are applied directly: no servo transient
  pe.step_s = 0.0005f;
  StarshipPlantParams pr = pe;
  pr.step_s = 0.01f;
  const StarshipPlantHarness::Config ce = StarshipPlantHarness::configure(pe), cr = StarshipPlantHarness::configure(pr);
  Dyn::Sim<StarshipPlantHarness, Dyn::SemiImplicitEuler> eu(ce);
  Dyn::Sim<StarshipPlantHarness, Dyn::Rk4> rk(cr);
  StarshipPlantHarness::Input u;
  u.thrustBody = {0, 0, 0};
  u.rollTorque = 0.0f;
  u.thrustEnabled = false;
  for (int i = 0; i < 4; i++)
    u.flapAngle[i] = i < 2 ? 50 * kDeg : 40 * kDeg;
  eu.state().body.att = rk.state().body.att = bellyDownAttitude(0.05f, 0.03f);
  eu.state().body.pos(5) = rk.state().body.pos(5) = 300.0f;
  for (int i = 0; i < 4000; i++) // 2 s
    eu.step(u);
  for (int i = 0; i < 200; i++)
    rk.step(u);
  TEST_ASSERT_FLOAT_WITHIN(0.03f * fabsf(eu.state().body.pos(2)) + 0.05f, eu.state().body.pos(2), rk.state().body.pos(2));
  TEST_ASSERT_FLOAT_WITHIN(0.03f * fabsf(eu.state().body.pos(5) - 300.0f) + 0.1f, eu.state().body.pos(5), rk.state().body.pos(5));
  for (int i = 3; i < 7; i++)
    TEST_ASSERT_FLOAT_WITHIN(0.05f, eu.state().body.att(i), rk.state().body.att(i));
  TEST_ASSERT_FLOAT_WITHIN(1e-5, 2.0, rk.state().t_s);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_E1_hover_upright);
  RUN_TEST(test_E2_ascent_wind_gusts);
  RUN_TEST(test_E3_belly_fall_to_ground);
  RUN_TEST(test_E4_flap_sweeps_and_bad_inputs);
  RUN_TEST(test_E5_thrust_clamp_and_gating);
  RUN_TEST(test_E6_advance_carry_and_reset);
  RUN_TEST(test_E7_harness_servo_models);
  RUN_TEST(test_E8_runtime_environment_changes);
  RUN_TEST(test_defaults_equal_legacy_defaults);
  RUN_TEST(test_random_parameters_equivalence);
  RUN_TEST(test_starship_aero_standalone_equals_legacy_and_is_stateless);
  RUN_TEST(test_starship_aero_invalid_inputs_and_alpha);
  RUN_TEST(test_parameter_variants_and_side_by_side_instances);
  RUN_TEST(test_state_sizes);
  RUN_TEST(test_rk4_close_to_fine_euler);
  return UNITY_END();
}
