// Scenario tests of the Starship flight plan (src/executive/starship_plan.*) on the flight executive, closed loop with
// the follower plant (test/executive_support): pio test -e native -f test_executive_scenarios
//
// Whole flights from go to landed, the hover-check and ascent timeouts (parity differences P1 and P2), the three
// kick endings, both landing triggers, and a non-finite estimate. The follower plant is not a vehicle model: it only
// makes the guards fire the way a vehicle following the commands would.
#include <math.h>
#include <stdio.h>
#include <unity.h>

#include "../executive_support/exec_sim.hpp"
#include "executive/starship_constants.hpp"
#include "executive/starship_plan.hpp"

using namespace VCTR;
using namespace VCTR::Exec;
using ExecTest::ExecSim;
using ExecTest::PlantConfig;
using Starship::id;
using Starship::Phase;

void setUp() {}
void tearDown() {}

namespace {

const FlightPlan &plan() { return Starship::flightPlanProfile2(); }

/// Runs until the phase changes (or maxS), returns the new phase.
uint8_t runToNextPhase(ExecSim &s, double maxS) {
  const uint8_t p = s.exec.phase();
  const int64_t end = s.clock.nowNs + int64_t(maxS * 1e9);
  while (s.clock.nowNs < end && s.exec.phase() == p)
    s.tick();
  return s.exec.phase();
}

/// Records the phase sequence of a run until `last` or maxS.
struct Sequence {
  uint8_t phases[32];
  int n = 0;
};

Sequence runSequence(ExecSim &s, uint8_t last, double maxS) {
  Sequence q;
  q.phases[q.n++] = s.exec.phase();
  const int64_t end = s.clock.nowNs + int64_t(maxS * 1e9);
  while (s.clock.nowNs < end && s.exec.phase() != last && q.n < 32) {
    s.tick();
    if (s.exec.phase() != q.phases[q.n - 1])
      q.phases[q.n++] = s.exec.phase();
  }
  return q;
}

void expectSequence(const Sequence &q, const uint8_t *want, int n) {
  char msg[160];
  int len = snprintf(msg, sizeof(msg), "got:");
  for (int i = 0; i < q.n && len < 150; i++)
    len += snprintf(msg + len, sizeof(msg) - len, " %u", q.phases[i]);
  TEST_ASSERT_EQUAL_INT_MESSAGE(n, q.n, msg);
  for (int i = 0; i < n; i++)
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(want[i], q.phases[i], msg);
}

} // namespace

static void test_nominal_flight_go_to_landed() {
  ExecSim s(plan());
  s.tick();
  s.go(-15);
  TEST_ASSERT_EQUAL_UINT8(id(Phase::Countdown), s.exec.phase());
  const uint8_t want[] = {id(Phase::Countdown), id(Phase::PreLaunch), id(Phase::Hover),   id(Phase::Ascent),
                          id(Phase::ApogeeHold), id(Phase::Kick),     id(Phase::Descent), id(Phase::Flip),
                          id(Phase::RthTransit), id(Phase::Landing),  id(Phase::Landed)};
  int completeSteps = 0;
  Sequence q;
  q.phases[q.n++] = s.exec.phase();
  while (s.sinceGoS() < 400 && s.exec.phase() != id(Phase::Landed)) {
    s.tick();
    completeSteps += s.exec.status().missionComplete ? 1 : 0;
    if (s.exec.phase() != q.phases[q.n - 1])
      q.phases[q.n++] = s.exec.phase();
  }
  expectSequence(q, want, sizeof(want));
  TEST_ASSERT_EQUAL_INT(1, completeSteps);
  TEST_ASSERT_FALSE(s.sanitizedSeen); // the sanitiser never fires in a normal flight
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, s.plant.pos[5]);
  TEST_ASSERT_FLOAT_WITHIN(0.6f, 0.0f, s.plant.pos[3]); // landed at home XY
  // Landed stays (terminal until a reset, X10).
  s.runFor(10);
  TEST_ASSERT_EQUAL_UINT8(id(Phase::Landed), s.exec.phase());
  TEST_ASSERT_TRUE(ExecTest::allOff(s.exec.command()));
  TEST_ASSERT_FALSE(s.exec.committed());
}

static void test_countdown_timing_follows_legacy_start_time() {
  ExecSim s(plan());
  s.go(-15);
  TEST_ASSERT_EQUAL_INT64(-15 * ExecTest::kS, s.exec.command().missionTimeNs);
  TEST_ASSERT_TRUE(s.exec.command().estimatorZeroing);
  TEST_ASSERT_EQUAL_UINT8(id(Phase::PreLaunch), runToNextPhase(s, 10));
  TEST_ASSERT_TRUE(s.exec.command().missionTimeNs > -10 * ExecTest::kS); // strict, legacy Init -> Startup
  TEST_ASSERT_TRUE(s.exec.command().missionTimeNs <= -10 * ExecTest::kS + 10 * ExecTest::kMs);
  TEST_ASSERT_EQUAL_UINT16(1, s.exec.command().actuatorTestSeq);
  TEST_ASSERT_EQUAL_UINT8(id(Phase::Hover), runToNextPhase(s, 15));
  TEST_ASSERT_TRUE(s.exec.command().missionTimeNs > 0);
  // P7: a start time after T0 still enters both countdown phases for one step each (the actuator test is kept).
  ExecSim late(plan());
  late.go(2);
  TEST_ASSERT_EQUAL_UINT8(id(Phase::Countdown), late.exec.phase());
  late.tick();
  TEST_ASSERT_EQUAL_UINT8(id(Phase::PreLaunch), late.exec.phase());
  TEST_ASSERT_EQUAL_UINT16(1, late.exec.command().actuatorTestSeq);
  late.tick();
  TEST_ASSERT_EQUAL_UINT8(id(Phase::Hover), late.exec.phase());
}

static void test_hover_check_failure_goes_to_rth_P1() {
  PlantConfig cfg;
  cfg.maxClimbRate = 0.0f; // cannot lift off
  ExecSim s(plan(), cfg);
  s.go(-15);
  TEST_ASSERT_TRUE(s.runUntilPhase(id(Phase::Hover), 20));
  const int64_t entry = s.exec.phaseEntryNs();
  TEST_ASSERT_EQUAL_UINT8(id(Phase::RthBrake), runToNextPhase(s, 30));
  const int64_t took = s.clock.nowNs - entry;
  TEST_ASSERT_TRUE(took > 10 * ExecTest::kS && took <= 10 * ExecTest::kS + 10 * ExecTest::kMs);
  TEST_ASSERT_TRUE(s.exec.command().resetTvc); // climb group -> recovery group
  TEST_ASSERT_TRUE(s.exec.command().flaps == FlapMode::Upright);
}

static void test_ascent_timeout_above_min_altitude_kicks() {
  PlantConfig cfg;
  cfg.maxClimbRate = 4.6f; // reaches about 100 m in the 25.5 s limit, not 117 m
  ExecSim s(plan(), cfg);
  s.go(-15);
  TEST_ASSERT_TRUE(s.runUntilPhase(id(Phase::Ascent), 40));
  const int64_t entry = s.exec.phaseEntryNs();
  TEST_ASSERT_EQUAL_UINT8(id(Phase::Kick), runToNextPhase(s, 40));
  TEST_ASSERT_EQUAL_INT64(25500 * ExecTest::kMs, s.clock.nowNs - entry);
  TEST_ASSERT_TRUE(s.exec.status().transition == ExitReason::Timeout);
  TEST_ASSERT_FALSE(s.exec.status().entryRedirected);
  TEST_ASSERT_TRUE(s.plant.pos[5] >= Starship::kMinKickAltitudeM);
}

static void test_ascent_timeout_below_min_altitude_goes_to_rth_P2() {
  PlantConfig cfg;
  cfg.maxClimbRate = 2.0f; // about 50 m at the timeout
  ExecSim s(plan(), cfg);
  s.go(-15);
  TEST_ASSERT_TRUE(s.runUntilPhase(id(Phase::Ascent), 40));
  TEST_ASSERT_EQUAL_UINT8(id(Phase::RthBrake), runToNextPhase(s, 40));
  TEST_ASSERT_TRUE(s.exec.status().entryRedirected);
  TEST_ASSERT_TRUE(s.plant.pos[5] < Starship::kMinKickAltitudeM);
  // and the RTH completes
  const uint8_t want[] = {id(Phase::RthBrake), id(Phase::RthTransit), id(Phase::Landing), id(Phase::Landed)};
  expectSequence(runSequence(s, id(Phase::Landed), 300), want, 4);
}

static void runToKick(ExecSim &s) {
  s.go(-15);
  TEST_ASSERT_TRUE(s.runUntilPhase(id(Phase::Kick), 120));
}

static void test_kick_ends_by_angle_rate_or_time() {
  // Angle: ~100 deg/s pitch rate reaches 15 deg remaining after ~0.75 s.
  {
    ExecSim s(plan());
    runToKick(s);
    const ControlCommand &c = s.exec.command();
    TEST_ASSERT_TRUE(c.tvc == TvcMode::Attitude);
    TEST_ASSERT_TRUE(c.attSpSource == AttSpSource::Command);
    TEST_ASSERT_EQUAL_FLOAT(Starship::kKickThrustLimitN, c.thrustN);
    TEST_ASSERT_EQUAL_FLOAT(Starship::kKickThrustLimitN, c.thrustCapN);
    TEST_ASSERT_TRUE(c.resetTvc);
    TEST_ASSERT_TRUE(c.flaps == FlapMode::Retracted);
    const int64_t entry = s.exec.phaseEntryNs();
    TEST_ASSERT_EQUAL_UINT8(id(Phase::Descent), runToNextPhase(s, 2));
    TEST_ASSERT_TRUE(s.exec.status().transition == ExitReason::Exit0);
    const int64_t took = s.clock.nowNs - entry;
    TEST_ASSERT_TRUE(took > 600 * ExecTest::kMs && took < 900 * ExecTest::kMs);
    // Kick end -> Descent: TVC off, motors and TVC actuators off at once (P6).
    TEST_ASSERT_TRUE(s.exec.command().tvc == TvcMode::Off);
    TEST_ASSERT_FALSE(s.exec.command().motors);
    TEST_ASSERT_FALSE(s.exec.command().tvcActuators);
    TEST_ASSERT_TRUE(s.exec.command().flaps == FlapMode::Descent);
  }
  // Rate: a spin above 250 deg/s ends it on the first step that sees the rate.
  {
    PlantConfig cfg;
    cfg.kickPitchRateRadPs = -300.0f * Starship::kDeg;
    ExecSim s(plan(), cfg);
    runToKick(s);
    TEST_ASSERT_EQUAL_UINT8(id(Phase::Descent), runToNextPhase(s, 2));
    TEST_ASSERT_TRUE(s.exec.status().transition == ExitReason::Exit1);
  }
  // Time: a slow rotation hits the hard limit at exactly 1.0 s.
  {
    PlantConfig cfg;
    cfg.kickPitchRateRadPs = -20.0f * Starship::kDeg;
    ExecSim s(plan(), cfg);
    runToKick(s);
    const int64_t entry = s.exec.phaseEntryNs();
    TEST_ASSERT_EQUAL_UINT8(id(Phase::Descent), runToNextPhase(s, 2));
    TEST_ASSERT_TRUE(s.exec.status().transition == ExitReason::Timeout);
    TEST_ASSERT_EQUAL_INT64(Starship::kKickTimeLimitNs, s.clock.nowNs - entry);
  }
}

static void test_landing_by_ground_sensor() {
  PlantConfig cfg;
  cfg.groundSensor = true;
  ExecSim s(plan(), cfg);
  s.go(-15);
  TEST_ASSERT_TRUE(s.runUntilPhase(id(Phase::Landing), 400));
  const int64_t entry = s.exec.phaseEntryNs();
  TEST_ASSERT_EQUAL_UINT8(id(Phase::Landed), runToNextPhase(s, 60));
  // 3 m at 0.2 m/s is ~15 s; the ground window fires within a step of touchdown, before the 2 s tracking timer.
  TEST_ASSERT_TRUE(s.plant.pos[5] < 0.04f);
  const double tookS = double(s.clock.nowNs - entry) / 1e9;
  ExecSim t(plan()); // same flight without the sensor: landed by tracking error ~2 s later
  t.go(-15);
  TEST_ASSERT_TRUE(t.runUntilPhase(id(Phase::Landing), 400));
  const int64_t entryT = t.exec.phaseEntryNs();
  TEST_ASSERT_EQUAL_UINT8(id(Phase::Landed), runToNextPhase(t, 60));
  const double tookT = double(t.clock.nowNs - entryT) / 1e9;
  TEST_ASSERT_TRUE(tookT > tookS + 1.5);
}

static void test_non_finite_estimate_is_sanitized() {
  ExecSim s(plan());
  s.go(-15);
  TEST_ASSERT_TRUE(s.runUntilPhase(id(Phase::Flip), 400));
  s.plant.pos[5] = NAN; // the estimate goes bad while the setpoint holds the measured position
  const ControlCommand &c = s.tick();
  TEST_ASSERT_TRUE(ExecTest::commandFinite(c));
  TEST_ASSERT_TRUE(s.sanitizedSeen);
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_nominal_flight_go_to_landed);
  RUN_TEST(test_countdown_timing_follows_legacy_start_time);
  RUN_TEST(test_hover_check_failure_goes_to_rth_P1);
  RUN_TEST(test_ascent_timeout_above_min_altitude_kicks);
  RUN_TEST(test_ascent_timeout_below_min_altitude_goes_to_rth_P2);
  RUN_TEST(test_kick_ends_by_angle_rate_or_time);
  RUN_TEST(test_landing_by_ground_sensor);
  RUN_TEST(test_non_finite_estimate_is_sanitized);
  return UNITY_END();
}
