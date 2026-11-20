// Host tests of src/safety/control_phase.cpp and src/safety/mission_abort.cpp: pio test -e native
//
// The controller phase table and the MissionAbort decision as pure functions (the mission-chain model in
// test/test_mission_chain exercises both inside the configured chain).
#include <math.h>
#include <unity.h>

#include "safety/control_phase.hpp"
#include "safety/mission_abort.hpp"

using namespace VCTR::Safety;

void setUp() {}
void tearDown() {}

static const MissionModeIn kModes[] = {MissionModeIn::Idle, MissionModeIn::Initialisation, MissionModeIn::Startup,
                                       MissionModeIn::Running, MissionModeIn::Finished};
static const ActiveMission kMissions[] = {ActiveMission::Rth, ActiveMission::Waypoint, ActiveMission::Bellyflop,
                                          ActiveMission::Freefall, ActiveMission::Other};

static void test_table_full() {
  for (ActiveMission m : kMissions) {
    for (MissionModeIn mode : kModes) {
      for (int act = 0; act < 2; act++) {
        const ControlPhase p = controlPhaseFor(m, mode, act != 0);
        const bool unpowered = m == ActiveMission::Bellyflop || m == ActiveMission::Freefall;
        const bool expectTvc = !unpowered && mode == MissionModeIn::Running && act != 0;
        TEST_ASSERT_EQUAL(expectTvc, p.tvcEnabled);
        TEST_ASSERT_EQUAL(unpowered, p.bellyflopFlaps);
      }
    }
  }
}

static void test_table_named_phases() {
  // Powered: waypoint hover/ascent and RTH (mission actuators on in Running).
  TEST_ASSERT_TRUE(controlPhaseFor(ActiveMission::Waypoint, MissionModeIn::Running, true).tvcEnabled);
  TEST_ASSERT_TRUE(controlPhaseFor(ActiveMission::Rth, MissionModeIn::Running, true).tvcEnabled);
  // Countdown: controllers off (motors at idle at most).
  TEST_ASSERT_FALSE(controlPhaseFor(ActiveMission::Waypoint, MissionModeIn::Initialisation, false).tvcEnabled);
  TEST_ASSERT_FALSE(controlPhaseFor(ActiveMission::Waypoint, MissionModeIn::Startup, true).tvcEnabled);
  // Bellyflop transition (mission actuators on) and freefall: TVC set off, flap attitude control on.
  ControlPhase bf = controlPhaseFor(ActiveMission::Bellyflop, MissionModeIn::Running, true);
  TEST_ASSERT_FALSE(bf.tvcEnabled);
  TEST_ASSERT_TRUE(bf.bellyflopFlaps);
  // The old first-run state (bellyflop reporting Idle) gives the same result: the table does not depend on it.
  ControlPhase bfIdle = controlPhaseFor(ActiveMission::Bellyflop, MissionModeIn::Idle, true);
  TEST_ASSERT_EQUAL(bf.tvcEnabled, bfIdle.tvcEnabled);
  TEST_ASSERT_EQUAL(bf.bellyflopFlaps, bfIdle.bellyflopFlaps);
  ControlPhase ff = controlPhaseFor(ActiveMission::Freefall, MissionModeIn::Running, false);
  TEST_ASSERT_FALSE(ff.tvcEnabled);
  TEST_ASSERT_TRUE(ff.bellyflopFlaps);
  // RTH landed, boot (RTH Idle).
  TEST_ASSERT_FALSE(controlPhaseFor(ActiveMission::Rth, MissionModeIn::Finished, false).tvcEnabled);
  TEST_ASSERT_FALSE(controlPhaseFor(ActiveMission::Rth, MissionModeIn::Idle, false).tvcEnabled);
}

static void test_tracker_reset_triggers() {
  ControlPhaseTracker t;
  // Boot: off, no reset.
  ControlPhaseStep s = t.update(ActiveMission::Rth, MissionModeIn::Idle, false, false);
  TEST_ASSERT_FALSE(s.phase.tvcEnabled);
  TEST_ASSERT_FALSE(s.resetTvc);
  // Countdown.
  s = t.update(ActiveMission::Waypoint, MissionModeIn::Startup, true, true);
  TEST_ASSERT_FALSE(s.resetTvc);
  // Waypoint Running: enabled -> reset once.
  s = t.update(ActiveMission::Waypoint, MissionModeIn::Running, true, false);
  TEST_ASSERT_TRUE(s.phase.tvcEnabled);
  TEST_ASSERT_TRUE(s.resetTvc);
  TEST_ASSERT_TRUE(s.changed);
  s = t.update(ActiveMission::Waypoint, MissionModeIn::Running, true, false);
  TEST_ASSERT_FALSE(s.resetTvc);
  TEST_ASSERT_FALSE(s.changed);
  // Abort: waypoint -> RTH while enabled: reset (mission switched), stays enabled.
  s = t.update(ActiveMission::Rth, MissionModeIn::Running, true, true);
  TEST_ASSERT_TRUE(s.phase.tvcEnabled);
  TEST_ASSERT_TRUE(s.resetTvc);
  TEST_ASSERT_FALSE(s.changed);
  // Bellyflop: off; freefall: off; RTH after freefall: reset.
  s = t.update(ActiveMission::Bellyflop, MissionModeIn::Running, true, true);
  TEST_ASSERT_FALSE(s.phase.tvcEnabled);
  TEST_ASSERT_FALSE(s.resetTvc);
  TEST_ASSERT_TRUE(s.changed);
  s = t.update(ActiveMission::Freefall, MissionModeIn::Running, false, true);
  TEST_ASSERT_FALSE(s.resetTvc);
  s = t.update(ActiveMission::Rth, MissionModeIn::Running, true, true);
  TEST_ASSERT_TRUE(s.resetTvc);
  // SystemReset: next enable resets again even without a switch flag.
  t.reset();
  TEST_ASSERT_FALSE(t.tvcEnabled());
  s = t.update(ActiveMission::Rth, MissionModeIn::Running, true, false);
  TEST_ASSERT_TRUE(s.resetTvc);
}

static AbortAction abortFor(ActiveMission m, MissionModeIn mode, float alt, float speed, bool running = true) {
  AbortInput in;
  in.vehicleRunning = running;
  in.active = m;
  in.mode = mode;
  in.altitude_m = alt;
  in.speed_mps = speed;
  return decideAbort(in);
}

static void test_abort_decision() {
  // Vehicle not Running: ignored, whatever the mission says.
  TEST_ASSERT_TRUE(abortFor(ActiveMission::Waypoint, MissionModeIn::Running, 50, 5, false) == AbortAction::Ignore);
  // Countdown: always reset, even if the estimate is off (no ground check needed).
  TEST_ASSERT_TRUE(abortFor(ActiveMission::Waypoint, MissionModeIn::Initialisation, 3, 2) ==
                   AbortAction::ResetAndStop);
  TEST_ASSERT_TRUE(abortFor(ActiveMission::Waypoint, MissionModeIn::Startup, 0, 0) == AbortAction::ResetAndStop);
  // Nothing running.
  TEST_ASSERT_TRUE(abortFor(ActiveMission::Waypoint, MissionModeIn::Idle, 0, 0) == AbortAction::ResetAndStop);
  // In flight: hover, ascent, bellyflop, freefall -> RTH.
  TEST_ASSERT_TRUE(abortFor(ActiveMission::Waypoint, MissionModeIn::Running, 1.0f, 0.1f) == AbortAction::StartRth);
  TEST_ASSERT_TRUE(abortFor(ActiveMission::Waypoint, MissionModeIn::Running, 60, 7) == AbortAction::StartRth);
  TEST_ASSERT_TRUE(abortFor(ActiveMission::Bellyflop, MissionModeIn::Running, 120, 1) == AbortAction::StartRth);
  TEST_ASSERT_TRUE(abortFor(ActiveMission::Bellyflop, MissionModeIn::Finished, 119, 3) == AbortAction::StartRth);
  TEST_ASSERT_TRUE(abortFor(ActiveMission::Freefall, MissionModeIn::Running, 80, 15) == AbortAction::StartRth);
  TEST_ASSERT_TRUE(abortFor(ActiveMission::Freefall, MissionModeIn::Finished, 70, 20) == AbortAction::StartRth);
  // Low but moving (lift-off through 0.3 m at 1 m/s): in flight.
  TEST_ASSERT_TRUE(abortFor(ActiveMission::Waypoint, MissionModeIn::Running, 0.3f, 1.0f) == AbortAction::StartRth);
  // Ground check holds in a "flying" mission: reset (never lifted off).
  TEST_ASSERT_TRUE(abortFor(ActiveMission::Waypoint, MissionModeIn::Running, 0.1f, 0.1f) ==
                   AbortAction::ResetAndStop);
  TEST_ASSERT_TRUE(abortFor(ActiveMission::Bellyflop, MissionModeIn::Running, 0.2f, 0.0f) ==
                   AbortAction::ResetAndStop);
  // RTH running: keep; RTH landed: reset.
  TEST_ASSERT_TRUE(abortFor(ActiveMission::Rth, MissionModeIn::Running, 20, 1) == AbortAction::KeepRth);
  TEST_ASSERT_TRUE(abortFor(ActiveMission::Rth, MissionModeIn::Running, 0.1f, 0.0f) == AbortAction::KeepRth);
  TEST_ASSERT_TRUE(abortFor(ActiveMission::Rth, MissionModeIn::Finished, 0, 0) == AbortAction::ResetAndStop);
}

static void test_ground_check_bounds_and_nan() {
  TEST_ASSERT_TRUE(groundCheck(0.0f, 0.0f));
  TEST_ASSERT_TRUE(groundCheck(-0.5f, 0.0f)); // baro noise below the start point
  TEST_ASSERT_TRUE(groundCheck(0.49f, 0.49f));
  TEST_ASSERT_FALSE(groundCheck(0.5f, 0.0f));
  TEST_ASSERT_FALSE(groundCheck(0.0f, 0.5f));
  TEST_ASSERT_FALSE(groundCheck(NAN, 0.0f));
  TEST_ASSERT_FALSE(groundCheck(0.0f, NAN));
  // NaN estimate in a flying mission -> RTH, never a silent reset.
  TEST_ASSERT_TRUE(abortFor(ActiveMission::Freefall, MissionModeIn::Running, NAN, NAN) == AbortAction::StartRth);
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_table_full);
  RUN_TEST(test_table_named_phases);
  RUN_TEST(test_tracker_reset_triggers);
  RUN_TEST(test_abort_decision);
  RUN_TEST(test_ground_check_bounds_and_nan);
  return UNITY_END();
}
