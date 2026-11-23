// Host tests of src/safety/stop_altitude.cpp: pio test -e native -f test_stop_altitude
#include <math.h>
#include <unity.h>

#include "safety/stop_altitude.hpp"

using namespace VCTR::Safety;

void setUp() {}
void tearDown() {}

void test_default_and_limits() {
  TEST_ASSERT_EQUAL_FLOAT(20.0f, kStopAltitudeDefaultM); // the stop trigger keeps its old constant by default
  TEST_ASSERT_EQUAL_FLOAT(5.0f, kStopAltitudeMinM);
  TEST_ASSERT_EQUAL_FLOAT(60.0f, kStopAltitudeMaxM);
}

void test_accept_on_ground() {
  StopAltContext c;
  TEST_ASSERT_EQUAL(StopAltResult::Accepted, decideSetStopAltitude(20.0f, c));
  TEST_ASSERT_EQUAL(StopAltResult::Accepted, decideSetStopAltitude(5.0f, c));
  TEST_ASSERT_EQUAL(StopAltResult::Accepted, decideSetStopAltitude(60.0f, c));
}

void test_value_limits() {
  StopAltContext c;
  TEST_ASSERT_EQUAL(StopAltResult::OutOfRange, decideSetStopAltitude(4.99f, c));
  TEST_ASSERT_EQUAL(StopAltResult::OutOfRange, decideSetStopAltitude(60.01f, c));
  TEST_ASSERT_EQUAL(StopAltResult::OutOfRange, decideSetStopAltitude(-20.0f, c));
  TEST_ASSERT_EQUAL(StopAltResult::NotFinite, decideSetStopAltitude(NAN, c));
  TEST_ASSERT_EQUAL(StopAltResult::NotFinite, decideSetStopAltitude(INFINITY, c));
}

void test_refused_off_ground() {
  StopAltContext c;
  c.committed = true; // in flight or mission pending
  TEST_ASSERT_EQUAL(StopAltResult::Committed, decideSetStopAltitude(30.0f, c));
  c = StopAltContext();
  c.interlockOpen = true;
  TEST_ASSERT_EQUAL(StopAltResult::InterlockOpen, decideSetStopAltitude(30.0f, c));
  c = StopAltContext();
  c.motorsEnabled = true;
  TEST_ASSERT_EQUAL(StopAltResult::MotorsEnabled, decideSetStopAltitude(30.0f, c));
}

void test_flip_start_gating() {
  TEST_ASSERT_EQUAL_FLOAT(96.0f, flipStartAltitude(true, 96.0f));
  TEST_ASSERT_TRUE(isnan(flipStartAltitude(false, 96.0f)));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_default_and_limits);
  RUN_TEST(test_accept_on_ground);
  RUN_TEST(test_value_limits);
  RUN_TEST(test_refused_off_ground);
  RUN_TEST(test_flip_start_gating);
  return UNITY_END();
}
