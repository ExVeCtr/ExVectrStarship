// Stage 3 design study (docs/project/aero-control-design.md section 2): static aerodynamic maps of the upright,
// powered vehicle, from the public aero API (src/sim/starship_aero.hpp, the validated function the simulator uses).
// Report only, no flight code involved. Run: pio test -e native -f test_aero_control_study -v | grep AEROCTL
//
// For a body tilted by theta about world +Y (theta > 0: nose toward +X/North; NWU) at world velocity v and wind w,
// with zero body rate and the flaps in the given pattern, it prints
//   Fx0      world X aero force at theta = 0 (N)
//   dFx      d(world X aero force)/d(theta) at theta = 0 (N/rad, central difference over +-1 deg)
//   b14/b11  lateral control gain d(a_x)/d(theta) = (T + dFx)/m for thrust T = 14 N / 10.8 N (m/s^2/rad); the
//            position/velocity cascade assumes b = T/m > 0 (acc-to-att: "tilt = lateral force")
//   M0       body pitch moment about the CoM at theta = 0 (N m; + rotates the nose toward +X)
//   dM       d(M)/d(theta) (N m/rad; > 0 is statically destabilising); compare Kp = 1.2 N m/rad (ControlAttitudeTvc)
//   hold     tilt offset the TVC can hold against dM: A / dM, A = 14 N sin(3 deg) 0.35 m = 0.256 N m (deg)
// plus, per variant, the speeds where b14 and b11 reach 0 (sign flip of the tilt-to-lateral-force gain), where dM
// reaches Kp (static attitude divergence without saturation) and where hold drops to 5 deg.
#include <math.h>
#include <stdio.h>
#include <unity.h>

#include "sim/starship_plant.hpp"

using namespace VCTR;
using Dyn::Vec3;

void setUp() {}
void tearDown() {}

static constexpr float kDeg = 3.14159265f / 180.0f;
static constexpr float kMass = 1.1f, kKp = 1.2f, kAuthority = 0.2562f; // 14 * sin(3 deg) * 0.35

struct Variant {
  const char *name;
  ShipSim::AeroScales s;
};

static Variant variants[] = {
    {"nominal", {}},
    {"x0.7", {0.7f, 1, 1, 1, 1, 1}},
    {"x1.3", {1.3f, 1, 1, 1, 1, 1}},
    {"geometry", {1.0f / 4.64f, 1, 1, 1, 1, 1}},
    {"pot0.22", {1.0f, 1, 0.22f, 1, 1, 1}}, // nose/base potential force at geometry level (ascent study variant)
};

struct Sample {
  float fx, fz, my;
};

// Flap patterns: 0 = all folded (Ascent mode, RTH translation/descent), 1 = Upright (RTH stabilising: top 0, bottom 90).
static Sample eval(const ShipSim::AeroConfig &cfg, float theta, const Vec3 &v, const Vec3 &w, int flaps) {
  const Math::Quat_F q(Vec3{0, 1, 0}, theta);
  ShipSim::AeroInput in;
  in.airVelBody = ShipSim::airVelocityBody(q, v, w);
  in.omegaBody = Vec3{0, 0, 0};
  const float top = flaps == 1 ? 0.0f : 1.5707963f;
  in.flapAngle[0] = in.flapAngle[1] = top;
  in.flapAngle[2] = in.flapAngle[3] = 1.5707963f;
  in.rho = 1.2f;
  ShipSim::AeroOutput o;
  ShipSim::starshipAero(cfg, in, o);
  TEST_ASSERT_TRUE(o.valid);
  const Vec3 fw = q.rotate(o.total.force);
  return {fw(0), fw(2), o.total.torque(1)};
}

struct Point {
  float fx0, dfx, b14, b11, m0, dm, holdDeg;
};

static Point point(const ShipSim::AeroConfig &cfg, const Vec3 &v, const Vec3 &w, int flaps, float theta0 = 0.0f) {
  const float h = 1.0f * kDeg;
  const Sample s0 = eval(cfg, theta0, v, w, flaps);
  const Sample sp = eval(cfg, theta0 + h, v, w, flaps), sm = eval(cfg, theta0 - h, v, w, flaps);
  Point p;
  p.fx0 = s0.fx;
  p.dfx = (sp.fx - sm.fx) / (2 * h);
  p.b14 = (14.0f * cosf(theta0) + p.dfx) / kMass;
  p.b11 = (kMass * 9.81f * cosf(theta0) + p.dfx) / kMass;
  p.m0 = s0.my;
  p.dm = (sp.my - sm.my) / (2 * h);
  p.holdDeg = p.dm > 1e-6f ? (kAuthority / p.dm) / kDeg : 90.0f;
  return p;
}

static void printRow(const char *tag, const char *var, float a, float b, const Point &p) {
  printf("AEROCTL %s %-8s %5.1f %5.1f | Fx0 %7.2f dFx %7.2f b14 %6.2f b11 %6.2f | M0 %7.3f dM %7.3f hold %5.1f\n", tag,
         var, a, b, p.fx0, p.dfx, p.b14, p.b11, p.m0, p.dm, p.holdDeg);
}

// Bisection on the speed for f(V) = 0, f increasing or decreasing; returns NaN if no sign change in [lo, hi].
template <class F> static float findSpeed(F f, float lo, float hi) {
  float flo = f(lo), fhi = f(hi);
  if (!(flo * fhi <= 0))
    return NAN;
  for (int i = 0; i < 40; i++) {
    const float mid = 0.5f * (lo + hi), fm = f(mid);
    if (flo * fm <= 0)
      hi = mid, fhi = fm;
    else
      lo = mid, flo = fm;
  }
  return 0.5f * (lo + hi);
}

void test_frame_sanity() {
  const Math::Quat_F q(Vec3{0, 1, 0}, 10 * kDeg);
  const Vec3 z = q.rotate(Vec3{0, 0, 1});
  TEST_ASSERT_TRUE(z(0) > 0.17f && z(0) < 0.18f); // +theta tilts the nose toward +X
}

// Fast descent (RTH after freefall or an abort): v = (vh, 0, -Vd), calm air. theta > 0 tilts toward +X, i.e. toward
// the horizontal velocity; a braking tilt (toward home when flying away from it) is theta < 0.
void test_descent_maps() {
  const ShipSim::StarshipPlantParams pp;
  for (const Variant &var : variants) {
    const ShipSim::AeroConfig cfg = ShipSim::makeScaledAeroConfig(pp, var.s);
    for (int flaps = 0; flaps < 2; flaps++) {
      const char *tag = flaps ? "DESC-UPRIGHT" : "DESC";
      for (float vh : {0.0f, 3.0f, 6.0f})
        for (float vd : {2.0f, 4.0f, 6.0f, 8.0f, 10.0f, 12.0f, 14.0f, 16.0f, 20.0f})
          printRow(tag, var.name, vd, vh, point(cfg, Vec3{vh, 0, -vd}, Vec3{0, 0, 0}, flaps));
    }
    auto b14 = [&](float vd) { return point(cfg, Vec3{0, 0, -vd}, Vec3{0, 0, 0}, 0).b14; };
    auto b11 = [&](float vd) { return point(cfg, Vec3{0, 0, -vd}, Vec3{0, 0, 0}, 0).b11; };
    auto dmKp = [&](float vd) { return point(cfg, Vec3{0, 0, -vd}, Vec3{0, 0, 0}, 0).dm - kKp; };
    auto hold5 = [&](float vd) { return point(cfg, Vec3{0, 0, -vd}, Vec3{0, 0, 0}, 0).holdDeg - 5.0f; };
    printf("AEROCTL DESC-SPEEDS %-8s flip(T=14) %5.1f flip(T=10.8) %5.1f dM=Kp %5.1f hold=5deg %5.1f m/s\n",
           var.name, findSpeed(b14, 0.5f, 40.0f), findSpeed(b11, 0.5f, 40.0f), findSpeed(dmKp, 0.5f, 40.0f),
           findSpeed(hold5, 0.5f, 40.0f));
  }
}

// Ascent: v = (0, 0, Vc), wind blowing toward -X at w (air from +X): the position hold tilts the nose into the wind
// (theta > 0).
void test_ascent_maps() {
  const ShipSim::StarshipPlantParams pp;
  for (const Variant &var : variants) {
    const ShipSim::AeroConfig cfg = ShipSim::makeScaledAeroConfig(pp, var.s);
    for (float w : {0.0f, 3.0f, 5.0f})
      for (float vc : {0.0f, 2.0f, 4.0f, 5.0f, 6.0f, 8.0f, 10.0f})
        printRow("ASC", var.name, vc, w, point(cfg, Vec3{0, 0, vc}, Vec3{-w, 0, 0}, 0));
    auto b14 = [&](float vc) { return point(cfg, Vec3{0, 0, vc}, Vec3{0, 0, 0}, 0).b14; };
    auto dmKp = [&](float vc) { return point(cfg, Vec3{0, 0, vc}, Vec3{0, 0, 0}, 0).dm - kKp; };
    auto hold5 = [&](float vc) { return point(cfg, Vec3{0, 0, vc}, Vec3{0, 0, 0}, 0).holdDeg - 5.0f; };
    printf("AEROCTL ASC-SPEEDS %-8s flip(T=14) %5.1f dM=Kp %5.1f hold=5deg %5.1f m/s (calm)\n", var.name,
           findSpeed(b14, 0.5f, 40.0f), findSpeed(dmKp, 0.5f, 40.0f), findSpeed(hold5, 0.5f, 40.0f));
  }
}

// Large-angle lateral force map, fast descent at 16 m/s with 3 m/s sideways (owner's RTH case), nominal and
// geometry: total lateral acceleration a_x(theta) = (T sin(theta) + Fx_aero(theta)) / m for T = 14 N. Shows where
// a tilt "toward home" (theta < 0) actually produces force toward home.
void test_descent_large_angle() {
  const ShipSim::StarshipPlantParams pp;
  for (int vi : {0, 3}) {
    const ShipSim::AeroConfig cfg = ShipSim::makeScaledAeroConfig(pp, variants[vi].s);
    for (float vd : {6.0f, 10.0f, 16.0f}) {
      printf("AEROCTL DESC-ANGLE %-8s Vd %4.1f vh 3.0 ax(theta deg):", variants[vi].name, vd);
      for (int td = -35; td <= 35; td += 5) {
        const float th = td * kDeg;
        const Sample s = eval(cfg, th, Vec3{3.0f, 0, -vd}, Vec3{0, 0, 0}, 0);
        printf(" %d:%.1f", td, (14.0f * sinf(th) + s.fx) / kMass);
      }
      printf("\n");
    }
  }
}

// Cost proxy on the host (not the target): evaluations per second of the full and the reduced config.
void test_host_cost() {
  const ShipSim::StarshipPlantParams pp;
  const ShipSim::AeroConfig full = ShipSim::makeAeroConfig(pp), red = ShipSim::makeReducedAeroConfig(pp, 8);
  volatile float sink = 0;
  for (int i = 0; i < 20000; i++) {
    sink = sink + eval(full, 0.01f * (i % 50), Vec3{1, 0, -8}, Vec3{0, 0, 0}, 0).fx;
    sink = sink + eval(red, 0.01f * (i % 50), Vec3{1, 0, -8}, Vec3{0, 0, 0}, 0).fx;
  }
  TEST_ASSERT_TRUE(isfinite(sink));
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_frame_sanity);
  RUN_TEST(test_descent_maps);
  RUN_TEST(test_ascent_maps);
  RUN_TEST(test_descent_large_angle);
  RUN_TEST(test_host_cost);
  return UNITY_END();
}
