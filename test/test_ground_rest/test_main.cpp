// Host tests of src/safety/ground_rest.hpp: pio test -e native -f test_ground_rest
#include <unity.h>

#include "safety/ground_rest.hpp"

using namespace VCTR::Safety;

void setUp() {}
void tearDown() {}

static constexpr int64_t kMs = 1000000LL;

static bool run(GroundRest &g, float alt, bool rv, float r, float sp, int64_t t0, int64_t dur) {
  bool out = false;
  for (int64_t t = 0; t <= dur; t += 20 * kMs) out = g.update(alt, rv, r, sp, t0 + t);
  return out;
}

void test_resting_at_zero() {
  GroundRest g;
  TEST_ASSERT_FALSE(run(g, 0.0f, false, 0, 0.0f, 0, 1900 * kMs));
  TEST_ASSERT_TRUE(g.update(0.0f, false, 0, 0.0f, 2100 * kMs));
}

void test_flying_is_not_ground() {
  GroundRest g;
  TEST_ASSERT_FALSE(run(g, 1.0f, false, 0, 0.0f, 0, 20000 * kMs));      // hover at 1 m
  TEST_ASSERT_FALSE(run(g, 5.0f, false, 0, 0.2f, 30000 * kMs, 20000 * kMs)); // descending RTH at 5 m
  TEST_ASSERT_FALSE(run(g, 5.0f, true, 5.0f, 0.2f, 60000 * kMs, 20000 * kMs));
}

void test_bounce_restarts() {
  GroundRest g;
  TEST_ASSERT_FALSE(run(g, 0.0f, false, 0, 0.0f, 0, 1900 * kMs));
  TEST_ASSERT_FALSE(g.update(0.0f, false, 0, 0.8f, 1920 * kMs)); // speed spike
  TEST_ASSERT_FALSE(run(g, 0.0f, false, 0, 0.0f, 1940 * kMs, 1900 * kMs));
  TEST_ASSERT_TRUE(run(g, 0.0f, false, 0, 0.0f, 3900 * kMs, 300 * kMs));
}

void test_valid_range_alone_and_nonfinite() {
  GroundRest g;
  TEST_ASSERT_TRUE(run(g, 0.5f, true, 0.1f, 0.0f, 0, 2100 * kMs)); // altitude estimate high, range says ground
  GroundRest h;
  TEST_ASSERT_FALSE(run(h, NAN, false, 0, 0.0f, 0, 5000 * kMs));
  TEST_ASSERT_FALSE(run(h, 0.0f, false, 0, NAN, 6000 * kMs, 5000 * kMs));
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_resting_at_zero);
  RUN_TEST(test_flying_is_not_ground);
  RUN_TEST(test_bounce_restarts);
  RUN_TEST(test_valid_range_alone_and_nonfinite);
  return UNITY_END();
}
