// Host tests of src/safety/mission_profile.cpp and decideGo's profile rule: pio test -e native -f test_mission_profile
#include <math.h>
#include <unity.h>

#include "safety/mission_profile.hpp"
#include "safety/rc_command.hpp"

using namespace VCTR::Safety;

void setUp() {}
void tearDown() {}

static void test_known_profiles() {
  TEST_ASSERT_TRUE(isKnownProfile(0));
  TEST_ASSERT_TRUE(isKnownProfile(1));
  TEST_ASSERT_TRUE(isKnownProfile(2));
  TEST_ASSERT_FALSE(isKnownProfile(3));
  TEST_ASSERT_FALSE(isKnownProfile(255));
  TEST_ASSERT_FALSE(missionProfileFor(3).valid);
  TEST_ASSERT_FALSE(missionProfileFor(255).valid);
}

static void test_flight_profile_is_not_described() {
  const MissionProfileDesc d = missionProfileFor(kProfileFlight);
  TEST_ASSERT_TRUE(d.valid);
  TEST_ASSERT_TRUE(d.flight);
  TEST_ASSERT_EQUAL_UINT8(0, d.count);
}

static void checkCommon(const MissionProfileDesc &d) {
  TEST_ASSERT_TRUE(d.valid);
  TEST_ASSERT_FALSE(d.flight);
  // pad point exactly as the flight list: 0.3 m, speed 0, threshold 5 m, no hold
  TEST_ASSERT_EQUAL_FLOAT(0.0f, d.wp[0].x);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, d.wp[0].y);
  TEST_ASSERT_EQUAL_FLOAT(0.3f, d.wp[0].z);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, d.wp[0].loiterS);
  for (int i = 0; i < d.count; i++) {
    TEST_ASSERT_TRUE(d.wp[i].timeLimitS > 0.0f && d.wp[i].timeLimitS < 60.0f); // explicit, finite
    TEST_ASSERT_TRUE(d.wp[i].z <= 1.0f);                                     // never above 1 m
    TEST_ASSERT_TRUE(isfinite(d.wp[i].x) && isfinite(d.wp[i].y) && isfinite(d.wp[i].z));
  }
  for (int i = 1; i < d.count; i++) {
    TEST_ASSERT_EQUAL_FLOAT(0.5f, d.wp[i].velocity);
    TEST_ASSERT_EQUAL_FLOAT(0.5f, d.wp[i].thresholdM);
  }
}

static void test_hover_test() {
  const MissionProfileDesc d = missionProfileFor(kProfileHoverTest);
  checkCommon(d);
  TEST_ASSERT_EQUAL_UINT8(2, d.count);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, d.wp[1].x);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, d.wp[1].y);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, d.wp[1].z);
  TEST_ASSERT_EQUAL_FLOAT(30.0f, d.wp[1].loiterS);
}

static void test_waypoint_test() {
  const MissionProfileDesc d = missionProfileFor(kProfileWaypointTest);
  checkCommon(d);
  TEST_ASSERT_EQUAL_UINT8(4, d.count);
  // climb to 1 m, hold 10 s
  TEST_ASSERT_EQUAL_FLOAT(0.0f, d.wp[1].x);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, d.wp[1].y);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, d.wp[1].z);
  TEST_ASSERT_EQUAL_FLOAT(10.0f, d.wp[1].loiterS);
  // +X (north) 2 m at 1 m, hold 15 s
  TEST_ASSERT_EQUAL_FLOAT(2.0f, d.wp[2].x);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, d.wp[2].y);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, d.wp[2].z);
  TEST_ASSERT_EQUAL_FLOAT(15.0f, d.wp[2].loiterS);
  // +X 2 m and +Y (west) 2 m at 1 m, hold 15 s
  TEST_ASSERT_EQUAL_FLOAT(2.0f, d.wp[3].x);
  TEST_ASSERT_EQUAL_FLOAT(2.0f, d.wp[3].y);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, d.wp[3].z);
  TEST_ASSERT_EQUAL_FLOAT(15.0f, d.wp[3].loiterS);
  // each 2 m leg is reachable in its time limit at the leg speed
  TEST_ASSERT_TRUE(d.wp[2].timeLimitS > 2.0f / d.wp[2].velocity * 1.5f);
  TEST_ASSERT_TRUE(d.wp[3].timeLimitS > 2.0f / d.wp[3].velocity * 1.5f);
}

static GoContext okContext(uint8_t profile) {
  GoContext c;
  c.modeReady = true;
  c.zeroValid = true;
  c.mode = 0;
  c.simulationApplied = false;
  c.profile = profile;
  return c;
}

static void test_decide_go_profiles() {
  TEST_ASSERT_TRUE(decideGo(okContext(0)) == GoRefusal::None);
  TEST_ASSERT_TRUE(decideGo(okContext(1)) == GoRefusal::None);
  TEST_ASSERT_TRUE(decideGo(okContext(2)) == GoRefusal::None);
  TEST_ASSERT_TRUE(decideGo(okContext(3)) == GoRefusal::SbProfile);
  TEST_ASSERT_TRUE(decideGo(okContext(255)) == GoRefusal::SbProfile);
  // the other preconditions still apply to the test profiles
  GoContext c = okContext(0);
  c.modeReady = false;
  TEST_ASSERT_TRUE(decideGo(c) == GoRefusal::NotReady);
  c = okContext(1);
  c.latched = true;
  TEST_ASSERT_TRUE(decideGo(c) == GoRefusal::Latched);
  c = okContext(1);
  c.simulationApplied = true;
  TEST_ASSERT_TRUE(decideGo(c) == GoRefusal::ScMismatch);
  c = okContext(0);
  c.requestBusy = true;
  TEST_ASSERT_TRUE(decideGo(c) == GoRefusal::RequestBusy);
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_known_profiles);
  RUN_TEST(test_flight_profile_is_not_described);
  RUN_TEST(test_hover_test);
  RUN_TEST(test_waypoint_test);
  RUN_TEST(test_decide_go_profiles);
  return UNITY_END();
}
