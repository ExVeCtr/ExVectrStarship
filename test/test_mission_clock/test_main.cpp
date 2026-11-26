// Host tests of src/safety/mission_clock.hpp: pio test -e native -f test_mission_clock
#include <math.h>
#include <unity.h>

#include "safety/mission_clock.hpp"

using VCTR::Safety::MissionClock;

static constexpr int64_t S = 1000000000LL;

void setUp() {}
void tearDown() {}

void test_nan_at_boot() {
  MissionClock c;
  TEST_ASSERT_FALSE(c.valid());
  TEST_ASSERT_TRUE(isnan(c.elapsedS(1000 * S)));
}

void test_default_countdown() {
  MissionClock c;
  c.begin(100 * S, -20 * S);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -20.0f, c.elapsedS(100 * S));
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -10.0f, c.elapsedS(110 * S));
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, c.elapsedS(120 * S));
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 45.5f, c.elapsedS(165 * S + S / 2));
}

void test_non_default_start_time() {
  MissionClock c;
  c.begin(5 * S, -12 * S);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -12.0f, c.elapsedS(5 * S));
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, c.elapsedS(17 * S));
  c.begin(50 * S, 0);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, c.elapsedS(50 * S));
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 3.0f, c.elapsedS(53 * S));
}

// The clock has no input from the steps: a step restarting its own mission time (step 2 begins at 130 s, mission
// time 0) does not disturb it, because only begin()/end()/reset() of the chain are fed.
void test_continuous_across_steps() {
  MissionClock c;
  c.begin(100 * S, -20 * S);
  float last = c.elapsedS(120 * S);
  for (int64_t t = 121; t <= 300; t++) {
    const float now = c.elapsedS(t * S);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, last + 1.0f, now);
    last = now;
  }
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 180.0f, last);
}

void test_held_after_end() {
  MissionClock c;
  c.begin(100 * S, -20 * S);
  c.end(400 * S);
  TEST_ASSERT_TRUE(c.valid());
  TEST_ASSERT_FALSE(c.running());
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 280.0f, c.elapsedS(400 * S));
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 280.0f, c.elapsedS(9999 * S));
  c.end(1000 * S); // a second end (RTH-ended steady state) does not change the held value
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 280.0f, c.elapsedS(2000 * S));
}

void test_end_without_begin_ignored() {
  MissionClock c;
  c.end(10 * S);
  TEST_ASSERT_FALSE(c.valid());
  TEST_ASSERT_TRUE(isnan(c.elapsedS(20 * S)));
}

void test_nan_after_reset() {
  MissionClock c;
  c.begin(100 * S, -20 * S);
  c.reset();
  TEST_ASSERT_TRUE(isnan(c.elapsedS(150 * S)));
  c.begin(200 * S, -20 * S);
  c.end(250 * S);
  c.reset(); // reset also clears a held value
  TEST_ASSERT_TRUE(isnan(c.elapsedS(300 * S)));
}

void test_second_mission_restarts() {
  MissionClock c;
  c.begin(100 * S, -20 * S);
  c.end(400 * S);
  c.begin(1000 * S, -20 * S);
  TEST_ASSERT_TRUE(c.running());
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -20.0f, c.elapsedS(1000 * S));
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 5.0f, c.elapsedS(1025 * S));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_nan_at_boot);
  RUN_TEST(test_default_countdown);
  RUN_TEST(test_non_default_start_time);
  RUN_TEST(test_continuous_across_steps);
  RUN_TEST(test_held_after_end);
  RUN_TEST(test_end_without_begin_ignored);
  RUN_TEST(test_nan_after_reset);
  RUN_TEST(test_second_mission_restarts);
  return UNITY_END();
}
