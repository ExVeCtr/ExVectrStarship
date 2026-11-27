// ExVectrDynamics S2: every building block against the frozen legacy BodySimModel (test/frozen/legacy_body_sim_model.hpp,
// ExVectrDSP 5aba4c6, verbatim except namespace/inline/public). Bit-identical over swept and random inputs.
// Run: pio test -e native -f test_dyn_blocks
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <unity.h>

#include "../frozen/legacy_body_sim_model.hpp"
#include "ExVectrDynamics/atmosphere.hpp"
#include "ExVectrDynamics/flat_plate.hpp"
#include "ExVectrDynamics/ground_contact.hpp"
#include "ExVectrDynamics/motor.hpp"
#include "ExVectrDynamics/servo.hpp"
#include "ExVectrDynamics/slender_body.hpp"
#include "ExVectrDynamics/tvc.hpp"
#include "ExVectrDynamics/wind.hpp"

using namespace VCTR;
using Dyn::Vec3;
using Legacy = legacy_frozen::BodySimModel;
using LegacyParams = legacy_frozen::BodySimParams;

void setUp() {}
void tearDown() {}

namespace {

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
  Vec3 vec(float a) { return Vec3{uni(-a, a), uni(-a, a), uni(-a, a)}; }
};

template <class T> bool sameBits(const T &a, const T &b) { return memcmp(&a, &b, sizeof(T)) == 0; }
bool sameF(float a, float b) { return memcmp(&a, &b, sizeof(float)) == 0 || (a == 0.0f && b == 0.0f); }
/// Equal bit for bit, except that +0 and -0 count as equal (used where the legacy value is a sum started from 0).
bool sameV(const Vec3 &a, const Vec3 &b) { return sameF(a(0), b(0)) && sameF(a(1), b(1)) && sameF(a(2), b(2)); }

constexpr float kHalfPi = 1.5708f;

LegacyParams randomParams(Rng &r) {
  LegacyParams p;
  p.mass_kg = r.uni(0.5f, 2.0f);
  p.bodyRadius_m = r.uni(0.04f, 0.1f);
  p.tubeLength_m = r.uni(0.3f, 0.8f);
  p.noseLength_m = r.uni(0.1f, 0.3f);
  p.cgFromBottom_m = r.uni(0.2f, 0.5f);
  p.flapHingeRadius_m = p.bodyRadius_m;
  p.flapTop = {r.uni(0.04f, 0.1f), r.uni(0.08f, 0.2f), r.uni(0.0f, 0.1f), r.uni(0.3f, 0.6f), r.uni(0.5f, 1.2f)};
  p.flapBottom = {r.uni(0.04f, 0.1f), r.uni(0.1f, 0.3f), r.uni(0.0f, 0.1f), r.uni(0.0f, 0.1f), r.uni(0.5f, 1.2f)};
  p.topFlapCantRad = r.uni(0.0f, 0.3f);
  p.flapFoldTowardBelly = r.coin();
  p.airDensity_kgm3 = r.uni(1.0f, 1.3f);
  p.cdCrossflow = r.uni(0.8f, 1.5f);
  p.crossflowEta = r.uni(0.5f, 1.0f);
  p.potentialFactor = r.pick(5) == 0 ? 0.0f : r.uni(0.5f, 1.5f);
  p.cdAxialTailFirst = r.uni(0.5f, 1.2f);
  p.cdAxialNoseFirst = r.uni(0.2f, 0.6f);
  p.bodyStrips = r.pick(6) == 0 ? r.pick(100) - 10 : 4 + r.pick(61); // includes out-of-range counts
  p.flapCnAlpha = r.uni(2.0f, 4.0f);
  p.flapCn90 = r.uni(0.9f, 1.4f);
  p.flapStallAngle_rad = r.uni(0.3f, 0.7f);
  p.flapStallWidth_rad = r.uni(0.05f, 0.15f);
  p.flapEffectiveMinAngleRad = r.pick(4) == 0 ? 0.0f : r.uni(0.1f, 0.5f);
  p.flapEffectiveSmoothRad = r.uni(0.02f, 0.15f);
  p.flapEffectiveRetractMarginRad = r.pick(3) == 0 ? r.uni(0.05f, 0.3f) : 0.0f;
  p.flapSidewashGain = r.pick(3) == 0 ? r.uni(-1.0f, 1.0f) : 0.0f;
  p.flapBodyCrossflowGain = r.pick(3) == 0 ? r.uni(0.0f, 1.0f) : 0.0f;
  p.flapFoldShieldFactor = r.uni(-0.2f, 1.3f);
  p.aeroCalibration = r.uni(0.5f, 6.0f);
  return p;
}

Dyn::SlenderBodyConfig hullConfig(const LegacyParams &p) {
  Dyn::SlenderBodyGeometry g;
  g.radius_m = p.bodyRadius_m;
  g.tubeLength_m = p.tubeLength_m;
  g.noseLength_m = p.noseLength_m;
  g.cgFromBottom_m = p.cgFromBottom_m;
  g.strips = p.bodyStrips;
  Dyn::SlenderBodyCoeffs c;
  c.cdCrossflow = p.cdCrossflow;
  c.crossflowEta = p.crossflowEta;
  c.potentialFactor = p.potentialFactor;
  c.cdAxialTailFirst = p.cdAxialTailFirst;
  c.cdAxialNoseFirst = p.cdAxialNoseFirst;
  return Dyn::makeSlenderBodyConfig(g, c);
}

Dyn::FlatPlateCoeffs plateCoeffs(const LegacyParams &p) {
  Dyn::FlatPlateCoeffs c;
  c.cnAlpha = p.flapCnAlpha;
  c.cn90 = p.flapCn90;
  c.stallAngle_rad = p.flapStallAngle_rad;
  c.stallWidth_rad = p.flapStallWidth_rad;
  c.effectiveMinAngle_rad = p.flapEffectiveMinAngleRad;
  c.effectiveSmooth_rad = p.flapEffectiveSmoothRad;
  c.effectiveRetractMargin_rad = p.flapEffectiveRetractMarginRad;
  c.sidewashGain = p.flapSidewashGain;
  c.bodyCrossflowGain = p.flapBodyCrossflowGain;
  c.foldShieldFactor = p.flapFoldShieldFactor;
  return c;
}

/// Random air velocity categories: zero, axial only, tiny, normal, large, crossflow only.
Vec3 randomAir(Rng &r) {
  switch (r.pick(8)) {
  case 0: return Vec3{0, 0, 0};
  case 1: return Vec3{0, 0, r.uni(-15, 15)};
  case 2: return r.vec(1e-4f);
  case 3: return Vec3{r.uni(-15, 15), r.uni(-15, 15), 0};
  case 4: return r.vec(30.0f);
  default: return r.vec(8.0f);
  }
}
Vec3 randomOmega(Rng &r) { return r.pick(3) == 0 ? Vec3{0, 0, 0} : r.vec(6.0f); }

} // namespace

// ---- atmosphere --------------------------------------------------------------------------------------------------
void test_atmosphere_matches_legacy_density() {
  Rng r(1);
  for (int i = 0; i < 200; i++) {
    LegacyParams p;
    p.airDensity_kgm3 = r.uni(0.5f, 1.5f);
    Dyn::AtmosphereParams a;
    a.rho_kgm3 = p.airDensity_kgm3;
    TEST_ASSERT_TRUE(sameBits(Dyn::airDensity(a, r.uni(-50, 3000)), p.airDensity_kgm3));
  }
  TEST_ASSERT_EQUAL_FLOAT(LegacyParams().airDensity_kgm3, Dyn::AtmosphereParams().rho_kgm3);
}

// ---- wind and gusts ----------------------------------------------------------------------------------------------
void test_wind_and_gust_bit_identical() {
  Rng r(2);
  for (int trial = 0; trial < 300; trial++) {
    Legacy lg{LegacyParams()};
    Dyn::WindParams wp;
    Dyn::GustTable gt = Dyn::buildGustTable(wp);
    TEST_ASSERT_TRUE(sameBits(lg.gustOmega_, gt.omega));
    TEST_ASSERT_TRUE(sameBits(lg.gustPhase_, gt.phase));
    for (int op = 0; op < 12; op++) {
      switch (r.pick(4)) {
      case 0: { // steady wind, with invalid values
        float sp = r.uni(-3, 12), dir = r.uni(-7, 7);
        if (r.pick(6) == 0) sp = NAN;
        if (r.pick(6) == 0) dir = INFINITY;
        lg.setWind(sp, dir);
        Dyn::setWind(wp, sp, dir);
        break;
      }
      case 1: { // gust, with invalid values and out-of-range values
        float st = r.uni(-2, 14), per = r.uni(0.0f, 40.0f);
        if (r.pick(6) == 0) st = NAN;
        if (r.pick(6) == 0) per = NAN;
        const uint32_t seed = r.pick(5) == 0 ? 0u : (r.pick(2) ? r.next() : uint32_t(r.pick(20)));
        lg.setGust(st, per, seed);
        Dyn::setGust(wp, gt, st, per, seed);
        break;
      }
      case 2:
        lg.clearWind();
        Dyn::clearWind(wp, gt);
        break;
      default:
        lg.params().gustSpread = r.uni(-0.5f, 1.5f);
        wp.gustSpread = lg.params().gustSpread;
        break;
      }
      TEST_ASSERT_TRUE(sameBits(lg.params().windSpeed_mps, wp.speed_mps));
      TEST_ASSERT_TRUE(sameBits(lg.params().windDirection_rad, wp.direction_rad));
      TEST_ASSERT_TRUE(sameBits(lg.params().gustStrength_mps, wp.gustStrength_mps));
      TEST_ASSERT_TRUE(sameBits(lg.params().gustPeriod_s, wp.gustPeriod_s));
      TEST_ASSERT_EQUAL_UINT32(lg.params().gustSeed, wp.gustSeed);
      TEST_ASSERT_TRUE(sameBits(lg.gustOmega_, gt.omega));
      TEST_ASSERT_TRUE(sameBits(lg.gustPhase_, gt.phase));
      for (int k = 0; k < 6; k++) {
        const double t = r.pick(3) == 0 ? 0.0 : double(r.uni(0.0f, 120.0f));
        float la, lc, ma, mc;
        lg.gustAt(t, la, lc);
        Dyn::gustAt(wp, gt, t, ma, mc);
        TEST_ASSERT_TRUE(sameBits(la, ma));
        TEST_ASSERT_TRUE(sameBits(lc, mc));
        lg.time_s_ = t;
        TEST_ASSERT_TRUE(sameBits(lg.windWorld(), Dyn::windAt(wp, gt, t)));
      }
    }
  }
}

// ---- slender body (hull) -----------------------------------------------------------------------------------------
// The legacy computeAero adds the four flaps; with effectiveness 0 their force is exactly zero, so the legacy total is
// the hull alone (compared with +0 == -0).
void test_slender_body_matches_legacy_hull() {
  Rng r(3);
  for (int trial = 0; trial < 3000; trial++) {
    LegacyParams p = randomParams(r);
    p.flapTop.effectiveness = 0.0f;
    p.flapBottom.effectiveness = 0.0f;
    Legacy lg(p);
    const Vec3 omega = randomOmega(r), v = randomAir(r) * -1.0f;
    lg.setAttitudeState({omega(0), omega(1), omega(2), 1, 0, 0, 0});
    lg.setPositionState({v(0), v(1), v(2), 0, 0, 0});
    lg.computeAero();
    const Vec3 u0 = lg.loads().airVelocity;
    const Dyn::SlenderBodyConfig c = hullConfig(p);
    TEST_ASSERT_TRUE(sameBits(lg.zNoseCp_, c.d.zNoseCp));
    TEST_ASSERT_TRUE(sameBits(lg.zBottom_, c.d.zBottom));
    TEST_ASSERT_TRUE(sameBits(lg.zTip_, c.d.zTip));
    TEST_ASSERT_TRUE(sameBits(lg.baseArea_, c.d.baseArea));
    const Dyn::Wrench w = Dyn::slenderBodyAero(c, p.airDensity_kgm3, p.aeroCalibration, u0, omega);
    TEST_ASSERT_TRUE(sameV(lg.loads().aeroForce, w.force));
    TEST_ASSERT_TRUE(sameV(lg.loads().aeroTorque, w.torque));
    for (int k = 0; k < 8; k++) {
      const float z = r.uni(-0.5f, 0.6f);
      TEST_ASSERT_TRUE(sameBits(lg.bodyDiameterAt(z), Dyn::slenderBodyDiameterAt(c, z)));
    }
  }
}

// ---- flat plate --------------------------------------------------------------------------------------------------
void test_flat_plate_cn_and_angle() {
  Rng r(4);
  for (int trial = 0; trial < 200; trial++) {
    const LegacyParams p = randomParams(r);
    const Legacy lg(p);
    const Dyn::FlatPlateCoeffs c = plateCoeffs(p);
    for (int k = 0; k < 100; k++) {
      const float a = r.pick(8) == 0 ? (r.coin() ? 0.0f : kHalfPi) : r.uni(-0.2f, 1.8f);
      TEST_ASSERT_TRUE(sameBits(lg.flapCn(a), Dyn::flatPlateCn(c, a)));
      TEST_ASSERT_TRUE(sameBits(lg.flapAeroAngle(a), Dyn::flatPlateAeroAngle(c, a)));
    }
  }
}

void test_flat_plate_flap_matches_legacy_addflap() {
  Rng r(5);
  int active = 0, inactive = 0;
  for (int trial = 0; trial < 3000; trial++) {
    const LegacyParams p = randomParams(r);
    Legacy lg(p);
    const Dyn::FlatPlateCoeffs c = plateCoeffs(p);
    const int index = r.pick(4), pair = index < 2 ? 0 : 1;
    const float side = (index == 0 || index == 2) ? 1.0f : -1.0f;
    const float angle = r.pick(8) == 0 ? (r.coin() ? 0.0f : kHalfPi) : r.uni(0.0f, kHalfPi);
    const Vec3 u0 = randomAir(r), omega = randomOmega(r);

    Vec3 n, d, e;
    lg.flapFrame(index, angle, n, d, e);
    const legacy_frozen::BodySimFlapGeometry &g = pair == 0 ? p.flapTop : p.flapBottom;
    Dyn::FlatPlateGeom geo;
    geo.hinge = Vec3{0.0f, side * p.flapHingeRadius_m, lg.flapZLow_[pair]} + e * lg.flapSc_[pair];
    geo.n = n;
    geo.d = d;
    geo.width_m = g.width_m;
    geo.area_m2 = lg.flapArea_[pair];
    geo.effectiveness = g.effectiveness;
    geo.bodyRadius_m = p.bodyRadius_m;

    lg.loads_.aeroForce = {0, 0, 0};
    lg.loads_.aeroTorque = {0, 0, 0};
    lg.loads_.flapForce[index] = {1, 2, 3}; // legacy must overwrite it
    lg.addFlap(index, angle, u0, omega);
    const Dyn::FlatPlateResult res =
        Dyn::flatPlateFlap(c, geo, p.airDensity_kgm3, p.aeroCalibration, angle, u0, omega);
    TEST_ASSERT_TRUE(sameBits(lg.loads_.flapForce[index], res.force));
    if (res.active) {
      active++;
      TEST_ASSERT_TRUE(sameV(lg.loads_.aeroForce, res.force));
      TEST_ASSERT_TRUE(sameV(lg.loads_.aeroTorque, res.cp.cross(res.force)));
    } else {
      inactive++;
      TEST_ASSERT_TRUE(sameV(lg.loads_.aeroForce, Vec3{0, 0, 0}));
      TEST_ASSERT_TRUE(sameV(lg.loads_.aeroTorque, Vec3{0, 0, 0}));
    }
  }
  TEST_ASSERT_TRUE(active > 2000);
  TEST_ASSERT_TRUE(inactive > 50); // zero air velocity cases exercise the early return
}

// ---- TVC clamp ---------------------------------------------------------------------------------------------------
static Vec3 randomThrust(Rng &r) {
  switch (r.pick(10)) {
  case 0: return Vec3{NAN, 1, 2};
  case 1: return Vec3{0, INFINITY, 5};
  case 2: return Vec3{0, 0, 0};
  case 3: return Vec3{0, 0, r.uni(-20, 20)};
  case 4: return Vec3{1e-9f, 0, 1e-7f};
  case 5: return r.vec(30.0f);
  case 6: return Vec3{r.uni(-1, 1), r.uni(-1, 1), r.uni(8, 20)};
  default: return Vec3{r.uni(-0.8f, 0.8f), r.uni(-0.8f, 0.8f), r.uni(0, 14)};
  }
}

void test_tvc_clamp_matches_legacy_setthrust() {
  Rng r(6);
  for (int trial = 0; trial < 20000; trial++) {
    LegacyParams p;
    p.tvcThrustLimit_N = r.uni(5, 20);
    p.tvcAngleLimit_rad = r.uni(0.01f, 0.5f);
    Legacy lg(p);
    Dyn::TvcParams tp;
    tp.thrustLimit_N = p.tvcThrustLimit_N;
    tp.angleLimit_rad = p.tvcAngleLimit_rad;
    const Vec3 f = randomThrust(r);
    lg.setThrust(f, 0.0f, true);
    const Dyn::TvcCommand c = Dyn::tvcClamp(tp, f);
    TEST_ASSERT_TRUE(sameBits(lg.thrustCmd_, c.force));
    TEST_ASSERT_TRUE(sameBits(lg.thrustMag_, c.mag));
  }
}

// ---- servo bank and TVC slew -------------------------------------------------------------------------------------
template <int Cap> static void runActuatorSequence(uint32_t seed, bool allowDeadTime) {
  Rng r(seed);
  for (int trial = 0; trial < 120; trial++) {
    LegacyParams p;
    p.applyActuatorRateLimits = r.coin();
    p.flapServoRate_radps = r.uni(1.0f, 12.0f);
    p.flapServoLag_s = r.coin() ? 0.0f : r.uni(0.001f, 0.05f);
    p.flapServoDeadTime_s = allowDeadTime ? (r.pick(5) == 0 ? r.uni(0.2f, 0.5f) : r.uni(0.0f, 0.08f)) : 0.0f;
    p.step_s = r.uni(0.001f, 0.005f);
    p.tvcServoRate_radps = r.uni(1.0f, 12.0f);
    p.tvcServoToThrustGain = r.uni(5.0f, 20.0f);
    Legacy lg(p);
    Dyn::ServoParams sp;
    sp.rate_radps = p.flapServoRate_radps;
    sp.lag_s = p.flapServoLag_s;
    sp.deadTime_s = p.flapServoDeadTime_s;
    sp.minAngle_rad = p.flapMinAngle_rad;
    sp.maxAngle_rad = p.flapMaxAngle_rad;
    Dyn::ServoState<4, Cap> ss;
    Dyn::servoInit(ss, 1.5708f);
    Dyn::TvcParams tp;
    tp.servoRate_radps = p.tvcServoRate_radps;
    tp.servoToThrustGain = p.tvcServoToThrustGain;
    tp.thrustLimit_N = p.tvcThrustLimit_N;
    tp.angleLimit_rad = p.tvcAngleLimit_rad;
    Dyn::TvcState ts;
    Dyn::TvcCommand cmd = Dyn::tvcClamp(tp, Vec3{0, 0, 0}); // legacy starts with thrustCmd_ = 0, mag 0
    TEST_ASSERT_TRUE(sameBits(lg.thrustMag_, cmd.mag));

    float in[4] = {kHalfPi, kHalfPi, kHalfPi, kHalfPi};
    for (int k = 0; k < 500; k++) {
      if (r.pick(10) == 0) { // new flap input, with invalid and out-of-range values
        for (int i = 0; i < 4; i++) {
          switch (r.pick(5)) {
          case 0: in[i] = NAN; break;
          case 1: in[i] = r.uni(-1.0f, 3.0f); break;
          default: in[i] = r.uni(0.0f, kHalfPi); break;
          }
        }
        lg.setFlapInput(in[0], in[1], in[2], in[3]);
        for (int i = 0; i < 4; i++)
          Dyn::servoSetInput(sp, ss, i, in[i]);
      }
      if (r.pick(10) == 0) {
        const Vec3 f = randomThrust(r);
        lg.setThrust(f, 0.0f, true);
        cmd = Dyn::tvcClamp(tp, f);
      }
      const float dt = r.pick(4) == 0 ? r.uni(0.0005f, 0.01f) : p.step_s;
      lg.updateActuators(dt);
      Dyn::servoStep(sp, ss, dt, p.step_s, p.applyActuatorRateLimits);
      Dyn::tvcSlewStep(tp, ts, cmd, p.applyActuatorRateLimits, dt);
      for (int i = 0; i < 4; i++) {
        TEST_ASSERT_TRUE(sameBits(lg.flapAngle_[i], ss.angle[i]));
        TEST_ASSERT_TRUE(sameBits(lg.flapRateLimited_[i], ss.rateLimited[i]));
        TEST_ASSERT_TRUE(sameBits(lg.flapInput_[i], ss.input[i]));
      }
      TEST_ASSERT_TRUE(sameBits(lg.tvcAngleX_, ts.angleX));
      TEST_ASSERT_TRUE(sameBits(lg.tvcAngleY_, ts.angleY));
      if (Cap == 128) // the legacy line has 128 slots; a capacity-1 line always uses slot 0
        TEST_ASSERT_EQUAL_INT(lg.delayIdx_, ss.delayIdx);
      // Thrust vector rebuilt from the slewed angles: verbatim expression of BodySimModel::step.
      const Vec3 fOld = {sinf(lg.tvcAngleY_) * lg.thrustMag_, sinf(lg.tvcAngleX_) * lg.thrustMag_,
                         cosf(lg.tvcAngleX_) * cosf(lg.tvcAngleY_) * lg.thrustMag_};
      TEST_ASSERT_TRUE(sameBits(fOld, Dyn::tvcForceFromAngles(ts, cmd.mag)));
    }
  }
}

void test_servo_and_tvc_slew_with_dead_time_match_legacy() { runActuatorSequence<128>(7, true); }
void test_servo_with_capacity_one_matches_legacy_without_dead_time() { runActuatorSequence<1>(8, false); }
void test_servo_state_sizes() {
  TEST_ASSERT_TRUE(sizeof(Dyn::ServoState<4, 1>) <= 80); // vehicle variant: about 72 B
  TEST_ASSERT_TRUE(sizeof(Dyn::ServoState<4, 128>) > 2048);
}

// ---- ground contact ----------------------------------------------------------------------------------------------
void test_ground_contact_matches_legacy() {
  Rng r(9);
  int touched = 0;
  for (int trial = 0; trial < 20000; trial++) {
    Legacy lg{LegacyParams()};
    auto rz = [&r](float lo, float hi) {
      const int c = r.pick(6);
      return c == 0 ? 0.0f : (c == 1 ? -0.0f : r.uni(lo, hi));
    };
    const Math::Vector<float, 6> posB = {r.uni(-3, 3), r.uni(-3, 3), r.uni(-5, 5), r.uni(-5, 5), r.uni(-5, 5), rz(-0.1f, 5.0f)};
    const Math::Vector<float, 7> attB = {r.uni(-3, 3), r.uni(-3, 3), r.uni(-3, 3), r.uni(0, 1), r.uni(-1, 1), r.uni(-1, 1), r.uni(-1, 1)};
    const Math::Vector<float, 6> posA = {r.uni(-3, 3), r.uni(-3, 3), r.uni(-5, 5), r.uni(-5, 5), r.uni(-5, 5), rz(-0.2f, 0.2f)};
    const Math::Vector<float, 7> attA = {r.uni(-3, 3), r.uni(-3, 3), r.uni(-3, 3), r.uni(0, 1), r.uni(-1, 1), r.uni(-1, 1), r.uni(-1, 1)};
    lg.setPositionState(posA);
    lg.setAttitudeState(attA);
    lg.groundContact(posB, attB);
    Dyn::RigidBodyState x, before;
    x.pos = posA;
    x.att = attA;
    before.pos = posB;
    before.att = attB;
    Dyn::groundContactStep(x, before);
    if (posA(5) < 0.0f)
      touched++;
    TEST_ASSERT_TRUE(sameBits(lg.positionState(), x.pos));
    TEST_ASSERT_TRUE(sameBits(lg.attitudeState(), x.att));
  }
  TEST_ASSERT_TRUE(touched > 2000);
}

// ---- motor -------------------------------------------------------------------------------------------------------
void test_motor_is_instantaneous_by_default() {
  Rng r(10);
  Dyn::MotorParams p;
  Dyn::MotorState s;
  for (int i = 0; i < 1000; i++) {
    const float cmd = r.uni(-2, 20);
    TEST_ASSERT_TRUE(sameBits(Dyn::motorThrust(p, cmd), cmd));
    TEST_ASSERT_TRUE(sameBits(Dyn::motorStep(p, s, cmd, 0.002f), cmd));
  }
}

void test_motor_spool_up_and_limit() {
  Dyn::MotorParams p;
  p.spoolTau_s = 0.05f;
  p.maxThrust_N = 10.0f;
  Dyn::MotorState s;
  float t = 0;
  for (int i = 0; i < 100; i++)
    t = Dyn::motorStep(p, s, 14.0f, 0.002f); // 0.2 s = 4 time constants
  TEST_ASSERT_TRUE(t > 5.0f && t < 10.0f);
  for (int i = 0; i < 2500; i++)
    t = Dyn::motorStep(p, s, 14.0f, 0.002f);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 10.0f, t); // limited
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_atmosphere_matches_legacy_density);
  RUN_TEST(test_wind_and_gust_bit_identical);
  RUN_TEST(test_slender_body_matches_legacy_hull);
  RUN_TEST(test_flat_plate_cn_and_angle);
  RUN_TEST(test_flat_plate_flap_matches_legacy_addflap);
  RUN_TEST(test_tvc_clamp_matches_legacy_setthrust);
  RUN_TEST(test_servo_and_tvc_slew_with_dead_time_match_legacy);
  RUN_TEST(test_servo_with_capacity_one_matches_legacy_without_dead_time);
  RUN_TEST(test_servo_state_sizes);
  RUN_TEST(test_ground_contact_matches_legacy);
  RUN_TEST(test_motor_is_instantaneous_by_default);
  RUN_TEST(test_motor_spool_up_and_limit);
  return UNITY_END();
}
