// Host tests of src/safety/sim_flap_fault.cpp: pio test -e native -f test_sim_flap_fault
//
// Simulation-only flap servo faults (owner-approved 2026-11-21): the pure decision, the stuck / slow servo model,
// capture of the current angle, armed-for-descent, clear, and the guarantee that outside simulation nothing changes.
#include <math.h>
#include <string.h>
#include <unity.h>

#include "safety/sim_flap_fault.hpp"

using namespace VCTR::Safety;

void setUp() {}
void tearDown() {}

static constexpr float kDeg = 3.14159265f / 180.0f;
static constexpr float kRate = 60 * kDeg / 0.2f; // StarshipFlaps modelled servo rate

// Replica of ServoAngleSimulator::update (src/starship_flaps.cpp) plus the fault hooks, as called in StarshipFlaps.
struct Servo {
  float cur = 0, target = 0;
  void step(SimFlapFaults &f, uint8_t i, bool sim, int64_t dtNs) {
    const float limit = kRate * f.rateFactor(i, sim);
    float diff = target - cur, mx = limit * double(dtNs) / 1e9;
    if (fabs(diff) <= mx)
      cur = target;
    else
      cur += (diff > 0 ? 1 : -1) * mx;
    const float held = f.effectiveAngle(i, sim, cur);
    if (held != cur)
      cur = held;
  }
};

static void test_decision_refusals() {
  // not simulation: refused whatever the rest
  SimFaultDecision d = decideSimFault(16, 0, 0x57, 0.7f, false);
  TEST_ASSERT_EQUAL(int(SimFaultAction::Refuse), int(d.action));
  TEST_ASSERT_EQUAL(int(SimFaultRefusal::NotSimulation), int(d.reason));
  TEST_ASSERT_EQUAL(int(SimFaultAction::Refuse), int(decideSimFault(17, 0, 0x57, 0.5f, false).action));
  TEST_ASSERT_EQUAL(int(SimFaultAction::Refuse), int(decideSimFault(18, 0, 0x57, 0, false).action));
  // magic
  TEST_ASSERT_EQUAL(int(SimFaultRefusal::BadMagic), int(decideSimFault(16, 0, 0x56, 0.7f, true).reason));
  TEST_ASSERT_EQUAL(int(SimFaultRefusal::BadMagic), int(decideSimFault(18, 0, 0, 0, true).reason));
  // non finite
  TEST_ASSERT_EQUAL(int(SimFaultRefusal::NotFinite), int(decideSimFault(16, 0, 0x57, NAN, true).reason));
  TEST_ASSERT_EQUAL(int(SimFaultRefusal::NotFinite), int(decideSimFault(17, 1, 0x57, INFINITY, true).reason));
  // clear ignores the float
  TEST_ASSERT_EQUAL(int(SimFaultAction::Clear), int(decideSimFault(18, 0xFF, 0x57, NAN, true).action));
  // subsystem bits: bits 2,3,5..7 refused; slow takes no arm bit
  TEST_ASSERT_EQUAL(int(SimFaultRefusal::BadSubsystem), int(decideSimFault(16, 0x04, 0x57, 0.7f, true).reason));
  TEST_ASSERT_EQUAL(int(SimFaultRefusal::BadSubsystem), int(decideSimFault(16, 0x20, 0x57, 0.7f, true).reason));
  TEST_ASSERT_EQUAL(int(SimFaultRefusal::BadSubsystem), int(decideSimFault(17, 0x10, 0x57, 0.5f, true).reason));
  // wrong type
  TEST_ASSERT_EQUAL(int(SimFaultRefusal::BadType), int(decideSimFault(15, 0, 0x57, 1, true).reason));
  TEST_ASSERT_EQUAL(int(SimFaultRefusal::BadType), int(decideSimFault(19, 0, 0x57, 1, true).reason));
}

static void test_decision_values() {
  SimFaultDecision d = decideSimFault(16, 0x00, 0x57, 42 * kDeg, true);
  TEST_ASSERT_EQUAL(int(SimFaultAction::Stuck), int(d.action));
  TEST_ASSERT_EQUAL(0, d.flap);
  TEST_ASSERT_FALSE(d.armForDescent);
  TEST_ASSERT_FALSE(d.captureCurrent);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 42 * kDeg, d.value);
  d = decideSimFault(16, 0x13, 0x57, 5.0f, true); // flap 3, armed, clamped to pi/2
  TEST_ASSERT_EQUAL(3, d.flap);
  TEST_ASSERT_TRUE(d.armForDescent);
  TEST_ASSERT_EQUAL_FLOAT(kSimFlapMaxRad, d.value);
  d = decideSimFault(16, 0x01, 0x57, -0.3f, true); // negative (not -1) clamps to 0
  TEST_ASSERT_FALSE(d.captureCurrent);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, d.value);
  d = decideSimFault(16, 0x02, 0x57, -1.0f, true);
  TEST_ASSERT_TRUE(d.captureCurrent);
  TEST_ASSERT_EQUAL(2, d.flap);
  d = decideSimFault(17, 0x03, 0x57, 0.01f, true);
  TEST_ASSERT_EQUAL(int(SimFaultAction::Slow), int(d.action));
  TEST_ASSERT_EQUAL_FLOAT(0.05f, d.value);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, decideSimFault(17, 0, 0x57, 7.0f, true).value);
  TEST_ASSERT_EQUAL_FLOAT(0.4f, decideSimFault(17, 0, 0x57, 0.4f, true).value);
}

static void test_stuck_holds_angle() {
  SimFlapFaults f;
  TEST_ASSERT_TRUE(f.set(decideSimFault(16, 0, 0x57, 42 * kDeg, true), true));
  Servo s[4];
  for (int k = 0; k < 4; k++)
    s[k].target = 60 * kDeg;
  s[0].target = 20 * kDeg;
  for (int n = 0; n < 100; n++)
    for (uint8_t i = 0; i < 4; i++)
      s[i].step(f, i, true, 20000000);
  TEST_ASSERT_EQUAL_FLOAT(42 * kDeg, s[0].cur); // holds although commanded 20 deg
  s[0].target = 80 * kDeg;
  for (int n = 0; n < 50; n++)
    s[0].step(f, 0, true, 20000000);
  TEST_ASSERT_EQUAL_FLOAT(42 * kDeg, s[0].cur);
  for (uint8_t i = 1; i < 4; i++)
    TEST_ASSERT_EQUAL_FLOAT(60 * kDeg, s[i].cur); // others normal
}

static void test_capture_current_angle() {
  SimFlapFaults f;
  Servo s;
  s.target = 50 * kDeg;
  for (int n = 0; n < 3; n++) // 3 * 20 ms * 300 deg/s = 18 deg
    s.step(f, 1, true, 20000000);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 18 * kDeg, s.cur);
  TEST_ASSERT_TRUE(f.set(decideSimFault(16, 1, 0x57, -1.0f, true), true));
  s.target = 90 * kDeg;
  s.step(f, 1, true, 20000000); // captures the angle at the start of the update result (moved once more)
  const float held = s.cur;
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 24 * kDeg, held);
  for (int n = 0; n < 50; n++)
    s.step(f, 1, true, 20000000);
  TEST_ASSERT_EQUAL_FLOAT(held, s.cur);
}

static void test_slow_rate() {
  SimFlapFaults f;
  Servo a, b;
  a.target = b.target = 90 * kDeg;
  TEST_ASSERT_TRUE(f.set(decideSimFault(17, 2, 0x57, 0.25f, true), true));
  TEST_ASSERT_EQUAL_FLOAT(0.25f, f.rateFactor(2, true));
  TEST_ASSERT_EQUAL_FLOAT(1.0f, f.rateFactor(0, true));
  for (int n = 0; n < 5; n++) { // 100 ms
    a.step(f, 2, true, 20000000); // slow
    b.step(f, 0, true, 20000000); // normal
  }
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.25f * 30 * kDeg, a.cur);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 30 * kDeg, b.cur);
}

static void test_clear_and_mode_switch() {
  SimFlapFaults f;
  f.set(decideSimFault(16, 0, 0x57, 0.7f, true), true);
  f.set(decideSimFault(16, 0x11, 0x57, 0.7f, true), true);
  f.set(decideSimFault(17, 3, 0x57, 0.2f, true), true);
  TEST_ASSERT_TRUE(f.stuck(0) && f.armed(1) && f.any());
  TEST_ASSERT_TRUE(f.set(decideSimFault(18, 0, 0x57, 0, true), true));
  TEST_ASSERT_FALSE(f.any());
  TEST_ASSERT_EQUAL_FLOAT(1.0f, f.rateFactor(3, true));
  // the simulation mode switch calls clear()
  f.set(decideSimFault(16, 0, 0x57, 0.7f, true), true);
  f.set(decideSimFault(16, 0x12, 0x57, 0.7f, true), true);
  f.clear();
  TEST_ASSERT_FALSE(f.any());
  TEST_ASSERT_EQUAL_FLOAT(0.123f, f.effectiveAngle(0, true, 0.123f));
}

static void test_armed_for_descent() {
  SimFlapFaults f;
  TEST_ASSERT_TRUE(f.set(decideSimFault(16, 0x10, 0x57, 42 * kDeg, true), true)); // TL armed
  TEST_ASSERT_TRUE(f.armed(0));
  TEST_ASSERT_FALSE(f.stuck(0));
  TEST_ASSERT_EQUAL_FLOAT(0.9f, f.effectiveAngle(0, true, 0.9f)); // not stuck before the descent
  TEST_ASSERT_EQUAL(1, f.onDescentBegin(true));
  TEST_ASSERT_TRUE(f.stuck(0));
  TEST_ASSERT_FALSE(f.armed(0));
  TEST_ASSERT_EQUAL_FLOAT(42 * kDeg, f.effectiveAngle(0, true, 0.9f));
  TEST_ASSERT_EQUAL(0, f.onDescentBegin(true)); // nothing armed any more
  // armed capture
  SimFlapFaults g;
  g.set(decideSimFault(16, 0x12, 0x57, -1.0f, true), true);
  g.onDescentBegin(true);
  TEST_ASSERT_EQUAL_FLOAT(0.6f, g.effectiveAngle(2, true, 0.6f));
  TEST_ASSERT_EQUAL_FLOAT(0.6f, g.effectiveAngle(2, true, 0.9f));
  // descent outside simulation: armed faults are dropped, nothing takes effect
  SimFlapFaults h;
  h.set(decideSimFault(16, 0x10, 0x57, 0.7f, true), true);
  TEST_ASSERT_EQUAL(0, h.onDescentBegin(false));
  TEST_ASSERT_FALSE(h.any());
}

static void test_non_simulation_is_inert_and_bit_identical() {
  // Reference run without any fault object activity.
  SimFlapFaults clean;
  Servo ref[4];
  // Run A: refused commands in real mode (never stored). Run B: a fault state that exists (set while simulating),
  // then evaluated with the simulation flag false. Both must equal the clean run bit for bit.
  SimFlapFaults refused, leaked;
  TEST_ASSERT_FALSE(refused.set(decideSimFault(16, 0, 0x57, 0.7f, false), false));
  TEST_ASSERT_FALSE(refused.set(decideSimFault(17, 1, 0x57, 0.1f, false), false));
  TEST_ASSERT_FALSE(refused.any());
  // Even a handed-in "accepted" decision is rejected without the simulation flag.
  TEST_ASSERT_FALSE(refused.set(decideSimFault(16, 0, 0x57, 0.7f, true), false));
  TEST_ASSERT_TRUE(leaked.set(decideSimFault(16, 0, 0x57, 0.7f, true), true));
  TEST_ASSERT_TRUE(leaked.set(decideSimFault(16, 1, 0x57, -1.0f, true), true));
  TEST_ASSERT_TRUE(leaked.set(decideSimFault(17, 2, 0x57, 0.05f, true), true));
  Servo a[4], b[4];
  float t = 0.3f;
  for (int n = 0; n < 400; n++) {
    t = 0.2f + 1.2f * (0.5f + 0.5f * sinf(0.07f * float(n)));
    for (uint8_t i = 0; i < 4; i++) {
      ref[i].target = a[i].target = b[i].target = t + 0.1f * i;
      ref[i].step(clean, i, false, 20000000);
      a[i].step(refused, i, false, 20000000);
      b[i].step(leaked, i, false, 20000000);
      TEST_ASSERT_EQUAL_MEMORY(&ref[i].cur, &a[i].cur, sizeof(float));
      TEST_ASSERT_EQUAL_MEMORY(&ref[i].cur, &b[i].cur, sizeof(float));
    }
  }
  TEST_ASSERT_EQUAL_FLOAT(1.0f, leaked.rateFactor(2, false));
  // descent in real mode must not activate anything
  TEST_ASSERT_EQUAL(0, leaked.onDescentBegin(false));
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_decision_refusals);
  RUN_TEST(test_decision_values);
  RUN_TEST(test_stuck_holds_angle);
  RUN_TEST(test_capture_current_angle);
  RUN_TEST(test_slow_rate);
  RUN_TEST(test_clear_and_mode_switch);
  RUN_TEST(test_armed_for_descent);
  RUN_TEST(test_non_simulation_is_inert_and_bit_identical);
  return UNITY_END();
}
