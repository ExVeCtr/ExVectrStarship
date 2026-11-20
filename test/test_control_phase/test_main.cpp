// Host tests of src/safety/control_phase.cpp and src/safety/mission_abort.cpp: pio test -e native
//
// The controller phase table and the MissionAbort decision as pure functions (the mission-chain model in
// test/test_mission_chain exercises both inside the configured chain).
#include <math.h>
#include <unity.h>

#include "safety/control_phase.hpp"
#include "safety/mission_abort.hpp"

#include <initializer_list>

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

static AbortAction abortFor(ActiveMission m, MissionModeIn mode, bool running = true) {
  AbortInput in;
  in.vehicleRunning = running;
  in.active = m;
  in.mode = mode;
  return decideAbort(in);
}

static void test_abort_decision() {
  // Not Running: nothing to abort.
  TEST_ASSERT_TRUE(abortFor(ActiveMission::Waypoint, MissionModeIn::Running, false) == AbortAction::Ignore);
  // Before liftoff: countdown, Idle mission, RTH finished -> reset and stop.
  TEST_ASSERT_TRUE(abortFor(ActiveMission::Waypoint, MissionModeIn::Initialisation) == AbortAction::ResetAndStop);
  TEST_ASSERT_TRUE(abortFor(ActiveMission::Waypoint, MissionModeIn::Startup) == AbortAction::ResetAndStop);
  for (ActiveMission m : kMissions)
    TEST_ASSERT_TRUE(abortFor(m, MissionModeIn::Idle) == AbortAction::ResetAndStop);
  TEST_ASSERT_TRUE(abortFor(ActiveMission::Rth, MissionModeIn::Finished) == AbortAction::ResetAndStop);
  // In flight: waypoint Running / Finished, bellyflop, freefall (any non-Idle mode) -> RTH.
  TEST_ASSERT_TRUE(abortFor(ActiveMission::Waypoint, MissionModeIn::Running) == AbortAction::StartRth);
  TEST_ASSERT_TRUE(abortFor(ActiveMission::Waypoint, MissionModeIn::Finished) == AbortAction::StartRth);
  for (ActiveMission m : {ActiveMission::Bellyflop, ActiveMission::Freefall, ActiveMission::Other})
    for (MissionModeIn mode : kModes)
      if (mode != MissionModeIn::Idle)
        TEST_ASSERT_TRUE(abortFor(m, mode) == AbortAction::StartRth);
  // RTH running keeps running.
  TEST_ASSERT_TRUE(abortFor(ActiveMission::Rth, MissionModeIn::Running) == AbortAction::KeepRth);
}

static void test_abort_decision_has_no_state_input() {
  // Review 2026-11-20: the decision depends on mission and mode only. The input carries no altitude or speed, so
  // a low and slow vehicle right after liftoff (0.3 m, 0.2 m/s) cannot be classed "on the ground" and dropped.
  AbortInput in;
  in.vehicleRunning = true;
  in.active = ActiveMission::Waypoint;
  in.mode = MissionModeIn::Running;
  TEST_ASSERT_TRUE(decideAbort(in) == AbortAction::StartRth);
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_table_full);
  RUN_TEST(test_table_named_phases);
  RUN_TEST(test_tracker_reset_triggers);
  RUN_TEST(test_abort_decision);
  RUN_TEST(test_abort_decision_has_no_state_input);
  return UNITY_END();
}
