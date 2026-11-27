// ExVectrDynamics S1: rigid-body step bit-identical to the frozen legacy PhysicsBodySim, Model concept check,
// Sim<Model, Integrator> (snapshot/restore, advance, RK4). Run: pio test -e native -f test_dyn_rigid_body
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <unity.h>

#include "../frozen/legacy_physics_bodysim.hpp"
#include "ExVectrDynamics/sim.hpp"

using namespace VCTR;
using Dyn::Vec3;

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
  Vec3 vec(float a) { return Vec3{uni(-a, a), uni(-a, a), uni(-a, a)}; }
};

template <class T> bool sameBits(const T &a, const T &b) { return memcmp(&a, &b, sizeof(T)) == 0; }

Math::Matrix<float, 3, 3> randomInertia(Rng &r) {
  Math::Matrix<float, 3, 3> I = Math::Matrix<float, 3, 3>::eye();
  I(0, 0) = r.uni(0.01f, 0.1f);
  I(1, 1) = r.uni(0.01f, 0.1f);
  I(2, 2) = r.uni(0.002f, 0.02f);
  const float p = r.uni(-0.002f, 0.002f);
  I(0, 1) = I(1, 0) = p;
  I(0, 2) = I(2, 0) = r.uni(-0.001f, 0.001f);
  I(1, 2) = I(2, 1) = r.uni(-0.001f, 0.001f);
  return I;
}

} // namespace

// 1000 random sequences x 100 steps = 1e5 steps; random force and torque sums (body/world, with positions and the
// acceleration flag), random steps including invalid ones (0, negative, NaN, inf); memcmp after every step.
void test_euler_step_bit_identical_to_legacy() {
  Rng r(12345);
  long steps = 0;
  for (int seq = 0; seq < 1000; seq++) {
    const float mass = r.uni(0.3f, 3.0f);
    const auto I = randomInertia(r);
    legacy_frozen::PhysicsBodySim old(mass, I);
    const Dyn::RigidBodyParams p = Dyn::makeRigidBodyParams(mass, I);
    Dyn::RigidBodyState x = Dyn::makeRigidBodyState();
    TEST_ASSERT_TRUE(sameBits(old.getAttitudeState(), x.att));
    TEST_ASSERT_TRUE(sameBits(old.getPositionState(), x.pos));
    // Random start state (a normalised quaternion is not required: the step normalises).
    Math::Vector<float, 7> a = {r.uni(-3, 3), r.uni(-3, 3), r.uni(-3, 3), r.uni(-1, 1), r.uni(-1, 1), r.uni(-1, 1),
                                r.uni(-1, 1)};
    Math::Vector<float, 6> pp = {r.uni(-5, 5), r.uni(-5, 5), r.uni(-5, 5), r.uni(-5, 5), r.uni(-5, 5), r.uni(-5, 5)};
    old.setAttitudeState(a);
    old.setPositionState(pp);
    x.att = a;
    x.pos = pp;
    for (int k = 0; k < 100; k++) {
      old.clearForces();
      Dyn::ForceSums s;
      const int nf = int(r.next() % 4), nt = int(r.next() % 3);
      for (int i = 0; i < nf; i++) {
        const Vec3 f = r.vec(15.0f), pos = r.vec(0.5f);
        const bool body = r.next() & 1, accel = (r.next() % 4) == 0;
        old.addForce(f, pos, body, accel);
        Dyn::addForce(s, mass, f, pos, body, accel);
      }
      for (int i = 0; i < nt; i++) {
        const Vec3 t = r.vec(0.5f), pos = r.vec(0.5f);
        const bool body = r.next() & 1, accel = (r.next() % 4) == 0;
        old.addTorque(t, pos, body, accel);
        Dyn::addTorque(s, mass, t, body, accel);
      }
      TEST_ASSERT_TRUE(sameBits(old.getBodyForceSum(), s.forceBody));
      TEST_ASSERT_TRUE(sameBits(old.getNavForceSum(), s.forceWorld));
      TEST_ASSERT_TRUE(sameBits(old.getBodyTorqueSum(), s.torqueBody));
      TEST_ASSERT_TRUE(sameBits(old.getNavTorqueSum(), s.torqueWorld));
      float dt = 0.002f;
      switch (r.next() % 12) {
      case 0: dt = 0.0f; break;
      case 1: dt = -0.002f; break;
      case 2: dt = NAN; break;
      case 3: dt = INFINITY; break;
      case 4: dt = r.uni(1e-4f, 0.02f); break;
      default: break;
      }
      old.step(dt);
      Dyn::semiImplicitEulerStep(p, x, s, dt);
      steps++;
      TEST_ASSERT_TRUE_MESSAGE(sameBits(old.getAttitudeState(), x.att), "attitude state differs from legacy");
      TEST_ASSERT_TRUE_MESSAGE(sameBits(old.getPositionState(), x.pos), "position state differs from legacy");
    }
  }
  TEST_ASSERT_EQUAL(100000, steps);
}

// ---------------------------------------------------------------------------------------------------------------
// Toy model: rigid body with gravity, linear drag and a constant body torque. Exercises Sim and both integrators.
// ---------------------------------------------------------------------------------------------------------------
struct Toy {
  struct Params {
    float mass = 1.0f, drag = 0.5f, torqueZ = 0.01f, izz = 0.01f;
    float step_s = 0.002f, maxAdvance_s = 0.05f;
  };
  struct Config {
    Params p;
    Dyn::RigidBodyParams body;
  };
  struct State {
    Dyn::RigidBodyState body;
    double t_s;
  };
  struct Input {
    float torqueScale;
  };
  struct Output {
    float speed;
  };
  using DefaultIntegrator = Dyn::SemiImplicitEuler;

  static Config configure(const Params &p) {
    Config c;
    c.p = p;
    Math::Matrix<float, 3, 3> I = Math::Matrix<float, 3, 3>::eye();
    I(0, 0) = I(1, 1) = 0.05f;
    I(2, 2) = p.izz;
    c.body = Dyn::makeRigidBodyParams(p.mass, I);
    return c;
  }
  static void init(const Config &, State &x) {
    x.body = Dyn::makeRigidBodyState();
    x.t_s = 0.0;
  }
  static float stepSize_s(const Config &c) { return c.p.step_s; }
  static float maxAdvance_s(const Config &c) { return c.p.maxAdvance_s; }
  static const Dyn::RigidBodyParams &bodyParams(const Config &c) { return c.body; }
  static double time_s(const State &x) { return x.t_s; }
  static void beginStep(const Config &, State &, const Input &, float) {}
  static void forces(const Config &c, const State &x, const Input &u, double, Dyn::Wrench &body, Vec3 &world,
                     Output &y) {
    const Vec3 v = x.body.pos.block<3, 1>(0, 0);
    world = Vec3{0, 0, -c.p.mass * 9.807f} - v * c.p.drag;
    body.force = Vec3{0, 0, 0};
    body.torque = Vec3{0, 0, c.p.torqueZ * u.torqueScale};
    y.speed = v.magnitude();
  }
  static void endStep(const Config &, State &x, const Dyn::RigidBodyState &, float dt) { x.t_s += double(dt); }
  template <class Probe> static void step(const Config &c, State &x, const Input &u, float dt, Output &y, Probe &) {
    beginStep(c, x, u, dt);
    Dyn::Wrench wb;
    Vec3 ww;
    forces(c, x, u, x.t_s, wb, ww, y);
    Dyn::ForceSums s;
    s.forceBody = wb.force;
    s.torqueBody = wb.torque;
    s.forceWorld = ww;
    const Dyn::RigidBodyState before = x.body;
    Dyn::semiImplicitEulerStep(c.body, x.body, s, dt);
    endStep(c, x, before, dt);
  }
};

// Compile-time concept checks.
struct Incomplete {
  struct Params {};
  struct Config {};
  struct State {};
  struct Input {};
  struct Output {};
  static Config configure(const Params &) { return Config(); }
  // init, stepSize_s, maxAdvance_s and step are missing
};
struct WrongState : Toy {
  struct State {
    State() {}
    State(const State &) {}
  }; // not trivially copyable
};
static_assert(Dyn::is_model<Toy>::value, "Toy must satisfy the concept");
static_assert(!Dyn::is_model<Incomplete>::value, "an incomplete type must be rejected");
static_assert(!Dyn::is_model<int>::value, "a non-class must be rejected");
static_assert(!Dyn::is_model<WrongState>::value, "a non-trivially-copyable State must be rejected");

void test_is_model_rejects_incomplete_types() {
  TEST_ASSERT_TRUE(Dyn::is_model<Toy>::value);
  TEST_ASSERT_FALSE(Dyn::is_model<Incomplete>::value);
  TEST_ASSERT_FALSE(Dyn::is_model<WrongState>::value);
}

void test_sim_snapshot_restore_round_trip() {
  const Toy::Config cfg = Toy::configure(Toy::Params());
  Dyn::Sim<Toy> a(cfg);
  const Toy::Input u = {1.0f};
  for (int i = 0; i < 200; i++)
    a.step(u);
  a.advance(0.0071f, u); // leaves a carry
  const auto snap = a.snapshot();
  for (int i = 0; i < 300; i++)
    a.step(u);
  Dyn::Sim<Toy> b(cfg);
  b.restore(snap);
  TEST_ASSERT_EQUAL_INT64(snap.ticks, b.ticks());
  for (int i = 0; i < 300; i++)
    b.step(u);
  TEST_ASSERT_TRUE(sameBits(a.state(), b.state()));
  // A different config in the restored instance is allowed and takes effect.
  Toy::Params p2;
  p2.drag = 5.0f;
  const Toy::Config cfg2 = Toy::configure(p2);
  Dyn::Sim<Toy> c(cfg2);
  c.restore(snap);
  for (int i = 0; i < 300; i++)
    c.step(u);
  TEST_ASSERT_FALSE(sameBits(a.state(), c.state()));
}

void test_sim_advance_carry_and_bounds() {
  const Toy::Config cfg = Toy::configure(Toy::Params());
  Dyn::Sim<Toy> s(cfg);
  const Toy::Input u = {0.0f};
  TEST_ASSERT_EQUAL(0, s.advance(0.0f, u));
  TEST_ASSERT_EQUAL(0, s.advance(-1.0f, u));
  TEST_ASSERT_EQUAL(0, s.advance(NAN, u));
  TEST_ASSERT_EQUAL(1, s.advance(0.002f, u));
  TEST_ASSERT_EQUAL(4, s.advance(0.0097f, u)); // 9.7 ms: 4 steps, 1.7 ms carried
  TEST_ASSERT_EQUAL(25, s.advance(1.0f, u));   // bounded to maxAdvance 50 ms
  s.clearTimeCarry();
  TEST_ASSERT_EQUAL(0, s.advance(0.001f, u));
  TEST_ASSERT_EQUAL_INT64(30, s.ticks());
}

static float yawOf(const Dyn::RigidBodyState &x) {
  const float w = x.att(3), z = x.att(6);
  return 2.0f * atan2f(z, w);
}

// Free fall with linear drag + constant yaw torque: analytic solution. RK4 at 10 ms must be far closer than Euler at
// 10 ms, and close to the analytic value.
void test_rk4_against_analytic_solution() {
  Toy::Params p;
  const float T = 2.0f;
  const double k = double(p.drag) / p.mass, g = 9.807;
  const double vz = -(g / k) * (1.0 - exp(-k * T));
  const double zAn = -(g / k) * (T - (1.0 - exp(-k * T)) / k);
  const double wz = double(p.torqueZ) / p.izz * T;
  const double yawAn = 0.5 * double(p.torqueZ) / p.izz * T * T;

  p.step_s = 0.01f;
  const Toy::Config cfg = Toy::configure(p);
  Dyn::Sim<Toy, Dyn::Rk4> rk(cfg);
  Dyn::Sim<Toy, Dyn::SemiImplicitEuler> eu(cfg);
  const Toy::Input u = {1.0f};
  for (int i = 0; i < 200; i++) {
    rk.step(u);
    eu.step(u);
  }
  TEST_ASSERT_FLOAT_WITHIN(2e-4f, float(vz), rk.state().body.pos(2));
  TEST_ASSERT_FLOAT_WITHIN(2e-3f, float(zAn), rk.state().body.pos(5));
  TEST_ASSERT_FLOAT_WITHIN(2e-3f, float(wz), rk.state().body.att(2));
  TEST_ASSERT_FLOAT_WITHIN(2e-3f, float(yawAn), yawOf(rk.state().body));
  const float errRk = fabsf(rk.state().body.pos(5) - float(zAn));
  const float errEu = fabsf(eu.state().body.pos(5) - float(zAn));
  TEST_ASSERT_TRUE_MESSAGE(errRk * 10.0f < errEu, "RK4 is not clearly more accurate than Euler at 10 ms");
  const float qn = Dyn::quatOf(rk.state().body).magnitude();
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, qn);
  TEST_ASSERT_TRUE(rk.state().body.att(3) >= 0.0f);
  TEST_ASSERT_TRUE(rk.output().speed > 0.0f);
}

void test_rigid_body_deriv_matches_euler_increment() {
  Rng r(77);
  const auto I = randomInertia(r);
  const Dyn::RigidBodyParams p = Dyn::makeRigidBodyParams(1.3f, I);
  Dyn::RigidBodyState x = Dyn::makeRigidBodyState();
  x.att = {0.3f, -0.2f, 0.5f, 1, 0, 0, 0};
  Dyn::ForceSums s;
  s.forceBody = Vec3{1, 2, 3};
  s.torqueBody = Vec3{0.1f, -0.05f, 0.02f};
  s.forceWorld = Vec3{0, 0, -12.7f};
  Vec3 a, alpha;
  Dyn::rigidBodyDeriv(p, x, s, a, alpha);
  Dyn::RigidBodyState y = x;
  const float dt = 1e-4f;
  Dyn::semiImplicitEulerStep(p, y, s, dt);
  for (int i = 0; i < 3; i++) {
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, a(i), (y.pos(i) - x.pos(i)) / dt);
    TEST_ASSERT_FLOAT_WITHIN(2e-3f, alpha(i), (y.att(i) - x.att(i)) / dt);
  }
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_euler_step_bit_identical_to_legacy);
  RUN_TEST(test_is_model_rejects_incomplete_types);
  RUN_TEST(test_sim_snapshot_restore_round_trip);
  RUN_TEST(test_sim_advance_carry_and_bounds);
  RUN_TEST(test_rk4_against_analytic_solution);
  RUN_TEST(test_rigid_body_deriv_matches_euler_increment);
  return UNITY_END();
}
