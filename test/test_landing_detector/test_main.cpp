// Host tests of src/safety/landing_detector.hpp: pio test -e native -f test_landing_detector
#include <unity.h>

#include "safety/landing_detector.hpp"

using namespace VCTR::Safety;

void setUp() {}
void tearDown() {}

static constexpr int64_t kMs = 1000000LL;

/// Feed the same input for durationNs at 20 ms steps from t0; true if landed at any step.
static bool run(LandingDetector &d, LandingInput in, int64_t t0, int64_t durationNs) {
  bool landed = false;
  for (int64_t t = 0; t <= durationNs; t += 20 * kMs) {
    in.nowNs = t0 + t;
    landed |= d.update(in);
  }
  return landed;
}

void test_air_hover_never_landed() {
  LandingDetector d;
  LandingInput in; // hover at 10 m, setpoint far below (old trigger fired here)
  in.altitudeM = 10.0f; in.trackingErrorM = 0.5f;
  TEST_ASSERT_FALSE(run(d, in, 0, 30000 * kMs));
  in.altitudeM = 1.6f; // still above the altitude gate
  TEST_ASSERT_FALSE(run(d, in, 40000 * kMs, 30000 * kMs));
}

void test_hover_low_with_valid_far_range() {
  LandingDetector d;
  LandingInput in;
  in.altitudeM = 1.0f; in.trackingErrorM = 0.5f; // bad tracking at 1.4 m? range says 1.2 m
  in.rangeValid = true; in.rangeM = 1.2f;
  TEST_ASSERT_FALSE(run(d, in, 0, 20000 * kMs));
  // altitude estimate says low but range says clearly higher than the gate: veto also with range contact cue off
  in.rangeM = 1.05f;
  TEST_ASSERT_FALSE(run(d, in, 30000 * kMs, 20000 * kMs));
}

void test_slow_descent_in_air_not_landed() {
  LandingDetector d;
  LandingInput in; // 1.2 m, descending 0.2 m/s, tracking fine, range valid 1.1 m
  in.altitudeM = 1.2f; in.verticalSpeed = -0.2f; in.speed = 0.2f; in.trackingErrorM = 0.05f;
  in.rangeValid = true; in.rangeM = 1.1f;
  TEST_ASSERT_FALSE(run(d, in, 0, 20000 * kMs));
  in.rangeValid = false; // and without range
  TEST_ASSERT_FALSE(run(d, in, 30000 * kMs, 20000 * kMs));
  // tracking error with real vertical motion is not contact either
  in.trackingErrorM = 0.4f; in.verticalSpeed = -0.4f; in.speed = 0.4f;
  TEST_ASSERT_FALSE(run(d, in, 60000 * kMs, 20000 * kMs));
}

void test_tilted_ground_contact_landed_with_range() {
  LandingDetector d;
  LandingInput in; // resting tilted: range reads 8 cm, altitude 0.1 m, no motion, thrust still on
  in.altitudeM = 0.1f; in.rangeValid = true; in.rangeM = 0.08f;
  TEST_ASSERT_FALSE(run(d, in, 0, 200 * kMs)); // not before the hold time
  TEST_ASSERT_TRUE(run(d, in, 1000 * kMs, 400 * kMs));
}

void test_tilted_ground_contact_landed_without_range() {
  LandingDetector d;
  LandingInput in; // range missing: the setpoint runs away from the held vehicle for 2 s
  in.altitudeM = 0.15f; in.trackingErrorM = 0.3f;
  TEST_ASSERT_FALSE(run(d, in, 0, 1500 * kMs));
  TEST_ASSERT_TRUE(run(d, in, 2000 * kMs, 2500 * kMs));
}

void test_range_invalid_does_not_trigger_or_block() {
  LandingDetector d;
  LandingInput in; // invalid range with a garbage tiny value must not count as contact
  in.altitudeM = 0.5f; in.rangeValid = false; in.rangeM = 0.01f;
  TEST_ASSERT_FALSE(run(d, in, 0, 20000 * kMs));
  // invalid range with a huge value must not veto the tracking cue
  in.rangeM = 5.0f; in.trackingErrorM = 0.3f;
  TEST_ASSERT_TRUE(run(d, in, 30000 * kMs, 2500 * kMs));
}

void test_bounce_restarts_timers() {
  LandingDetector d;
  LandingInput in;
  in.altitudeM = 0.05f; in.rangeValid = true; in.rangeM = 0.05f;
  TEST_ASSERT_FALSE(run(d, in, 0, 200 * kMs));
  in.rangeM = 0.4f; in.verticalSpeed = 0.8f; in.speed = 0.8f; // bounce
  TEST_ASSERT_FALSE(run(d, in, 220 * kMs, 200 * kMs));
  in.rangeM = 0.05f; in.verticalSpeed = 0; in.speed = 0;      // contact again: needs the full hold again
  TEST_ASSERT_FALSE(run(d, in, 440 * kMs, 200 * kMs));
  TEST_ASSERT_TRUE(run(d, in, 700 * kMs, 400 * kMs));
}

void test_nonfinite_inputs_never_land() {
  LandingDetector d;
  LandingInput in;
  in.altitudeM = NAN; in.rangeValid = true; in.rangeM = 0.01f; in.trackingErrorM = 1.0f;
  TEST_ASSERT_FALSE(run(d, in, 0, 5000 * kMs));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_air_hover_never_landed);
  RUN_TEST(test_hover_low_with_valid_far_range);
  RUN_TEST(test_slow_descent_in_air_not_landed);
  RUN_TEST(test_tilted_ground_contact_landed_with_range);
  RUN_TEST(test_tilted_ground_contact_landed_without_range);
  RUN_TEST(test_range_invalid_does_not_trigger_or_block);
  RUN_TEST(test_bounce_restarts_timers);
  RUN_TEST(test_nonfinite_inputs_never_land);
  return UNITY_END();
}
