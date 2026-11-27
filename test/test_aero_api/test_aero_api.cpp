// S6: the public aero API. Run: pio test -e native -f test_aero_api
//  1. starshipAero called standalone equals the plant's internal loads bit for bit (random states, flaps, wind).
//  2. AeroScales at their defaults change nothing (bit-identical config), non-default scales act as documented.
//  3. valid flag: non-finite input gives valid=false and zero outputs.
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <unity.h>

#include "sim/starship_plant.hpp"

using namespace VCTR;
using Dyn::Vec3;
using ShipSim::StarshipPlant;
using ShipSim::StarshipPlantParams;

void setUp() {}
void tearDown() {}

namespace {
struct Rng {
  uint32_t s;
  explicit Rng(uint32_t seed) : s(seed) {}
  uint32_t next() { s ^= s << 13; s ^= s >> 17; s ^= s << 5; return s; }
  float uni(float lo, float hi) { return lo + (hi - lo) * (float(next() >> 8) / 16777216.0f); }
};
template <class T> bool sameBits(const T &a, const T &b) { return memcmp(&a, &b, sizeof(T)) == 0; }

struct Capture {
  ShipSim::AeroOutput a;
  void onAero(const ShipSim::AeroOutput &o) { a = o; }
};
} // namespace

void test_standalone_equals_plant_loads() {
  Rng r(12345);
  StarshipPlantParams p;
  p.wind.speed_mps = 3.0f;
  p.wind.gustStrength_mps = 1.5f;
  StarshipPlant::Config c = StarshipPlant::configure(p);
  for (int it = 0; it < 300; it++) {
    StarshipPlant::State x;
    StarshipPlant::init(c, x);
    for (int i = 0; i < 3; i++) {
      x.body.att(i) = r.uni(-3, 3);
      x.body.pos(i) = r.uni(-15, 15);
    }
    Math::Vector<float, 4> q{r.uni(-1, 1), r.uni(-1, 1), r.uni(-1, 1), r.uni(-1, 1)};
    const float n = sqrtf(q(0) * q(0) + q(1) * q(1) + q(2) * q(2) + q(3) * q(3)) + 1e-3f;
    for (int i = 0; i < 4; i++) x.body.att(3 + i) = q(i) / n;
    x.body.pos(5) = r.uni(0.0f, 50.0f);
    x.t_s = r.uni(0.0f, 20.0f);
    StarshipPlant::Input u;
    u.thrustBody = {0, 0, 0};
    u.rollTorque = 0;
    u.thrustEnabled = false;
    for (int i = 0; i < 4; i++) u.flapAngle[i] = r.uni(0.0f, 1.5708f);
    StarshipPlant::beginStep(c, x, u, 0.002f);

    Dyn::ForceSums sums;
    StarshipPlant::Output y;
    ShipSim::AeroOutput inner;
    StarshipPlant::evalSums(c, x, x.t_s, sums, y, inner);

    // The caller rebuilds the input from the state exactly as the plant does (public API only).
    ShipSim::AeroInput in;
    const Math::Quat_F qq = x.body.att.template block<4, 1>(3, 0);
    const Vec3 v = x.body.pos.template block<3, 1>(0, 0);
    in.airVelBody = ShipSim::airVelocityBody(qq, v, Dyn::windAt(p.wind, c.d.gust, x.t_s));
    in.omegaBody = x.body.att.template block<3, 1>(0, 0);
    for (int i = 0; i < 4; i++) in.flapAngle[i] = x.flaps.angle[i];
    in.rho = Dyn::airDensity(p.air, x.body.pos(5));
    ShipSim::AeroOutput outer;
    ShipSim::starshipAero(c.d.aero, in, outer);

    TEST_ASSERT_TRUE(outer.valid);
    TEST_ASSERT_TRUE(sameBits(outer.total, inner.total));
    TEST_ASSERT_TRUE(sameBits(outer.hull, inner.hull));
    TEST_ASSERT_TRUE(sameBits(outer.flaps, inner.flaps));
    TEST_ASSERT_TRUE(sameBits(outer.alpha_rad, inner.alpha_rad));
    TEST_ASSERT_TRUE(inner.valid);
    TEST_ASSERT_TRUE(sameBits(outer.total, y.aero));
    for (int i = 0; i < 4; i++) TEST_ASSERT_TRUE(sameBits(outer.flapForce[i], y.flapForce[i]));
    TEST_ASSERT_TRUE(sameBits(in.airVelBody, y.airVelBody));
  }
}

void test_step_probe_sees_the_same_loads() {
  StarshipPlantParams p;
  StarshipPlant::Config c = StarshipPlant::configure(p);
  StarshipPlant::State x;
  StarshipPlant::init(c, x);
  x.body.pos(0) = 4.0f;
  x.body.pos(5) = 10.0f;
  StarshipPlant::Input u;
  u.thrustBody = {0, 0, 0};
  u.rollTorque = 0;
  u.thrustEnabled = false;
  for (int i = 0; i < 4; i++) u.flapAngle[i] = 0.4f;
  StarshipPlant::Output y;
  Capture cap;
  StarshipPlant::step(c, x, u, 0.002f, y, cap);
  TEST_ASSERT_TRUE(cap.a.valid);
  TEST_ASSERT_TRUE(sameBits(cap.a.total, y.aero));
}

void test_default_scales_are_identity() {
  StarshipPlantParams p;
  p.hull.strips = 17;
  p.aeroCalibration = 3.3f;
  const ShipSim::AeroConfig a = ShipSim::makeAeroConfig(p);
  const ShipSim::AeroConfig b = ShipSim::makeScaledAeroConfig(p, ShipSim::AeroScales());
  TEST_ASSERT_TRUE(sameBits(a.calibration, b.calibration));
  TEST_ASSERT_TRUE(sameBits(a.hull.coeff, b.hull.coeff));
  TEST_ASSERT_TRUE(sameBits(a.pair[0].effectiveness, b.pair[0].effectiveness));
  TEST_ASSERT_TRUE(sameBits(a.pair[1].effectiveness, b.pair[1].effectiveness));
  // ... and so is the result, bit for bit.
  ShipSim::AeroInput in;
  in.airVelBody = {1.0f, -2.0f, 3.5f};
  in.omegaBody = {0.3f, 0.2f, -0.1f};
  for (int i = 0; i < 4; i++) in.flapAngle[i] = 0.2f * float(i + 1);
  in.rho = 1.2f;
  ShipSim::AeroOutput oa, ob;
  ShipSim::starshipAero(a, in, oa);
  ShipSim::starshipAero(b, in, ob);
  TEST_ASSERT_TRUE(sameBits(oa.total, ob.total));
}

void test_scales_act_as_documented() {
  StarshipPlantParams p;
  ShipSim::AeroScales s;
  s.global = 0.7f;
  s.hullCrossflow = 2.0f;
  s.hullPotential = 0.5f;
  s.hullAxial = 1.5f;
  s.flapTop = 0.25f;
  s.flapBottom = 2.0f;
  const ShipSim::AeroConfig a = ShipSim::makeScaledAeroConfig(p, s);
  TEST_ASSERT_EQUAL_FLOAT(p.aeroCalibration * 0.7f, a.calibration);
  TEST_ASSERT_EQUAL_FLOAT(p.hullAero.cdCrossflow * 2.0f, a.hull.coeff.cdCrossflow);
  TEST_ASSERT_EQUAL_FLOAT(p.hullAero.potentialFactor * 0.5f, a.hull.coeff.potentialFactor);
  TEST_ASSERT_EQUAL_FLOAT(p.hullAero.cdAxialTailFirst * 1.5f, a.hull.coeff.cdAxialTailFirst);
  TEST_ASSERT_EQUAL_FLOAT(p.hullAero.cdAxialNoseFirst * 1.5f, a.hull.coeff.cdAxialNoseFirst);
  TEST_ASSERT_EQUAL_FLOAT(0.25f, a.pair[0].effectiveness);
  TEST_ASSERT_EQUAL_FLOAT(2.0f, a.pair[1].effectiveness);

  // Pure axial flow: the hull axial force scales exactly with hullAxial.
  ShipSim::AeroInput in;
  in.airVelBody = {0, 0, 5.0f};
  in.omegaBody = {0, 0, 0};
  for (int i = 0; i < 4; i++) in.flapAngle[i] = 1.5708f;
  in.rho = 1.2f;
  ShipSim::AeroOutput o0, o1;
  ShipSim::starshipAero(ShipSim::makeAeroConfig(p), in, o0);
  ShipSim::AeroScales ax;
  ax.hullAxial = 2.0f;
  ShipSim::starshipAero(ShipSim::makeScaledAeroConfig(p, ax), in, o1);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f * fabsf(o0.hull.force(2)), 2.0f * o0.hull.force(2), o1.hull.force(2));
}

void test_valid_flag_and_reduced_config() {
  const ShipSim::AeroConfig a = ShipSim::makeAeroConfig(StarshipPlantParams());
  ShipSim::AeroInput in;
  in.airVelBody = {1, 2, 3};
  in.omegaBody = {0, 0, 0};
  for (int i = 0; i < 4; i++) in.flapAngle[i] = 0.3f;
  in.rho = 1.2f;
  ShipSim::AeroOutput o;
  ShipSim::starshipAero(a, in, o);
  TEST_ASSERT_TRUE(o.valid);
  in.airVelBody(1) = NAN;
  ShipSim::starshipAero(a, in, o);
  TEST_ASSERT_FALSE(o.valid);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, o.total.force(0));
  TEST_ASSERT_EQUAL_FLOAT(0.0f, o.total.torque(1));
  in.airVelBody(1) = 2.0f;
  in.flapAngle[2] = INFINITY;
  ShipSim::starshipAero(a, in, o);
  TEST_ASSERT_FALSE(o.valid);

  const ShipSim::AeroConfig r = ShipSim::makeReducedAeroConfig(StarshipPlantParams());
  TEST_ASSERT_EQUAL(8, r.hull.geo.strips);
  TEST_ASSERT_EQUAL(24, a.hull.geo.strips);
  TEST_ASSERT_TRUE(sameBits(r.calibration, a.calibration));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_standalone_equals_plant_loads);
  RUN_TEST(test_step_probe_sees_the_same_loads);
  RUN_TEST(test_default_scales_are_identity);
  RUN_TEST(test_scales_act_as_documented);
  RUN_TEST(test_valid_flag_and_reduced_config);
  return UNITY_END();
}
