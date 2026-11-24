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

static void sameWp(const ProfileWaypoint &a, const ProfileWaypoint &b) {
  TEST_ASSERT_EQUAL_MEMORY(&a, &b, sizeof(ProfileWaypoint)); // bit-identical (all members are floats, no padding)
}

static void test_defaults_reproduce_the_hard_coded_lists() {
  const MissionParamSet p; // defaults
  const MissionProfileDesc h = missionProfileFor(kProfileHoverTest, p);
  const MissionProfileDesc w = missionProfileFor(kProfileWaypointTest, p);
  const ProfileWaypoint pad = {0, 0, 0.3f, 0, 3, 1, 5, 0, 10};
  sameWp(h.wp[0], pad);
  sameWp(h.wp[1], {0, 0, 1, 0.5f, 3, 1, 0.5f, 30, 10});
  sameWp(w.wp[0], pad);
  sameWp(w.wp[1], {0, 0, 1, 0.5f, 3, 1, 0.5f, 10, 10});
  sameWp(w.wp[2], {2, 0, 1, 0.5f, 3, 1, 0.5f, 15, 12});
  sameWp(w.wp[3], {2, 2, 1, 0.5f, 3, 1, 0.5f, 15, 12});
  // the flight apogee waypoint: {0, 0, 120} 7 m/s, accel 3 / 2, threshold 3 m, hold 5 s (main.cpp, as before)
  sameWp(flightApogeeWaypoint(p), {0, 0, 120, 7, 3, 2, 3, 5, 0});
  sameWp(flightApogeeWaypoint(), {0, 0, 120, 7, 3, 2, 3, 5, 0});
  // the default-argument form equals the explicit defaults
  sameWp(missionProfileFor(kProfileHoverTest).wp[1], h.wp[1]);
}

static void test_parameters_change_the_lists() {
  MissionParamSet p;
  TEST_ASSERT_TRUE(missionParamApply(p, 1, 4.0f));
  TEST_ASSERT_TRUE(missionParamApply(p, 2, 30.0f));
  TEST_ASSERT_TRUE(missionParamApply(p, 3, 5.0f));
  TEST_ASSERT_TRUE(missionParamApply(p, 4, 1.0f));
  TEST_ASSERT_TRUE(missionParamApply(p, 5, 2.0f));
  TEST_ASSERT_TRUE(missionParamApply(p, 6, 0.0f));
  const ProfileWaypoint a = flightApogeeWaypoint(p);
  TEST_ASSERT_EQUAL_FLOAT(30.0f, a.z);
  TEST_ASSERT_EQUAL_FLOAT(4.0f, a.velocity);
  TEST_ASSERT_EQUAL_FLOAT(5.0f, missionProfileFor(kProfileHoverTest, p).wp[1].loiterS);
  const MissionProfileDesc w = missionProfileFor(kProfileWaypointTest, p);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, w.wp[1].loiterS);
  TEST_ASSERT_EQUAL_FLOAT(2.0f, w.wp[2].loiterS);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, w.wp[3].loiterS);
  TEST_ASSERT_TRUE(a.z < kApogeeNoFlipBelowM);
}

static void test_new_params_apogee_waypoint() {
  MissionParamSet p;
  TEST_ASSERT_EQUAL_FLOAT(3.0f, p.ascentAccel);
  TEST_ASSERT_EQUAL_FLOAT(2.0f, p.ascentDecel);
  TEST_ASSERT_TRUE(missionParamApply(p, 7, 5.0f));
  TEST_ASSERT_TRUE(missionParamApply(p, 8, 1.5f));
  TEST_ASSERT_TRUE(missionParamApply(p, 9, 40.0f));
  TEST_ASSERT_TRUE(missionParamApply(p, 10, -25.0f));
  sameWp(flightApogeeWaypoint(p), {40, -25, 120, 7, 5, 1.5f, 3, 5, 0});
  // range edges and invalid values of ids 7..10
  for (uint8_t id = 7; id <= 8; id++) {
    TEST_ASSERT_TRUE(missionParamValid(id, 0.2f));
    TEST_ASSERT_TRUE(missionParamValid(id, 20.0f));
    TEST_ASSERT_FALSE(missionParamValid(id, 0.19f));
    TEST_ASSERT_FALSE(missionParamValid(id, 20.1f));
  }
  for (uint8_t id = 9; id <= 10; id++) {
    TEST_ASSERT_TRUE(missionParamValid(id, -2000.0f));
    TEST_ASSERT_TRUE(missionParamValid(id, 2000.0f));
    TEST_ASSERT_TRUE(missionParamValid(id, 0.0f));
    TEST_ASSERT_FALSE(missionParamValid(id, -2000.5f));
    TEST_ASSERT_FALSE(missionParamValid(id, 2000.5f));
    TEST_ASSERT_FALSE(missionParamValid(id, NAN));
    TEST_ASSERT_FALSE(missionParamValid(id, INFINITY));
  }
  MissionParamSet q;
  TEST_ASSERT_FALSE(missionParamApply(q, 9, 3000.0f));
  TEST_ASSERT_EQUAL_FLOAT(0.0f, q.apogeeX);
}

static void test_param_validation() {
  MissionParamSet p;
  TEST_ASSERT_FALSE(missionParamApply(p, 0, 1.0f));
  TEST_ASSERT_FALSE(missionParamApply(p, 11, 1.0f));
  TEST_ASSERT_FALSE(missionParamApply(p, 1, 0.4f));
  TEST_ASSERT_FALSE(missionParamApply(p, 1, 10.5f));
  TEST_ASSERT_FALSE(missionParamApply(p, 2, 1.5f));
  TEST_ASSERT_FALSE(missionParamApply(p, 2, 1001.0f));
  TEST_ASSERT_FALSE(missionParamApply(p, 3, -1.0f));
  TEST_ASSERT_FALSE(missionParamApply(p, 4, 601.0f));
  TEST_ASSERT_FALSE(missionParamApply(p, 5, NAN));
  TEST_ASSERT_FALSE(missionParamApply(p, 6, INFINITY));
  TEST_ASSERT_EQUAL_FLOAT(7.0f, p.ascentSpeed); // nothing changed
  TEST_ASSERT_EQUAL_FLOAT(120.0f, p.apogeeAltitude);
  TEST_ASSERT_EQUAL_FLOAT(30.0f, p.hoverHoldS);
  TEST_ASSERT_TRUE(missionParamApply(p, 2, 1000.0f));
  TEST_ASSERT_TRUE(missionParamApply(p, 2, 2.0f));
  TEST_ASSERT_EQUAL_FLOAT(2.0f, p.apogeeAltitude);
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
  RUN_TEST(test_defaults_reproduce_the_hard_coded_lists);
  RUN_TEST(test_parameters_change_the_lists);
  RUN_TEST(test_new_params_apogee_waypoint);
  RUN_TEST(test_param_validation);
  RUN_TEST(test_waypoint_test);
  RUN_TEST(test_decide_go_profiles);
  return UNITY_END();
}
