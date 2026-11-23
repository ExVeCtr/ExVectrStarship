#include <math.h>
#include <unity.h>

#include "safety/rth_altitude.hpp"

using namespace VCTR::Safety;

void setUp() {}
void tearDown() {}

static void test_values() {
  TEST_ASSERT_EQUAL_FLOAT(1.0f, rthReturnAltitude(1.0f, true, 3.0f));
  TEST_ASSERT_EQUAL_FLOAT(2.0f, rthReturnAltitude(2.0f, true, 3.0f));
  TEST_ASSERT_EQUAL_FLOAT(2.9f, rthReturnAltitude(2.9f, true, 3.0f));
  TEST_ASSERT_EQUAL_FLOAT(3.0f, rthReturnAltitude(3.0f, true, 3.0f));
  TEST_ASSERT_EQUAL_FLOAT(3.0f, rthReturnAltitude(84.0f, true, 3.0f)); // abort from the descent: cap, as before
  TEST_ASSERT_EQUAL_FLOAT(0.2f, rthReturnAltitude(0.2f, true, 3.0f));  // low: never above the vehicle
  TEST_ASSERT_EQUAL_FLOAT(0.0f, rthReturnAltitude(0.0f, true, 3.0f));  // on the ground: immediate landing
  TEST_ASSERT_EQUAL_FLOAT(0.0f, rthReturnAltitude(-1.0f, true, 3.0f));
}

static void test_invalid() {
  TEST_ASSERT_EQUAL_FLOAT(3.0f, rthReturnAltitude(NAN, true, 3.0f));
  TEST_ASSERT_EQUAL_FLOAT(3.0f, rthReturnAltitude(INFINITY, true, 3.0f));
  TEST_ASSERT_EQUAL_FLOAT(3.0f, rthReturnAltitude(1.0f, false, 3.0f));
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_values);
  RUN_TEST(test_invalid);
  return UNITY_END();
}
