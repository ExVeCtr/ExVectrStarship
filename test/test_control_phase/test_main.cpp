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
        for (int expired = 0; expired < 2; expired++) {
          const ControlPhase p = controlPhaseFor(m, mode, act != 0, expired != 0);
          const bool tvcAllowed =
              m == ActiveMission::Freefall ? false
                                           : (m == ActiveMission::Bellyflop ? expired == 0 : true);
          const bool expectTvc = tvcAllowed && mode == MissionModeIn::Running && act != 0;
          TEST_ASSERT_EQUAL(expectTvc, p.tvcEnabled);
          // Bellyflop: kick (TVC) and flap attitude control exclusive; freefall: flaps.
          const bool expectFlaps = m == ActiveMission::Freefall || (m == ActiveMission::Bellyflop && !expectTvc);
          TEST_ASSERT_EQUAL(expectFlaps, p.bellyflopFlaps);
          TEST_ASSERT_FALSE(p.tvcEnabled && p.bellyflopFlaps);
          if (m == ActiveMission::Bellyflop)
            TEST_ASSERT_EQUAL_FLOAT(kKickThrustLimitN, p.tvcThrustCapN);
          else
            TEST_ASSERT_EQUAL_FLOAT(kNoTvcThrustCapN, p.tvcThrustCapN);
        }
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
  // Bellyflop kick (user decision 2026-11-21): Running with actuators on -> TVC set on (thrust capped), flaps not
  // in Stabilize. After the kick (Finished) or past the guard: TVC off, flap attitude control on.
  TEST_ASSERT_TRUE(kTvcDuringBellyflopTransition);
  ControlPhase bf = controlPhaseFor(ActiveMission::Bellyflop, MissionModeIn::Running, true);
  TEST_ASSERT_TRUE(bf.tvcEnabled);
  TEST_ASSERT_FALSE(bf.bellyflopFlaps);
  TEST_ASSERT_EQUAL_FLOAT(kKickThrustLimitN, bf.tvcThrustCapN);
  ControlPhase bfDone = controlPhaseFor(ActiveMission::Bellyflop, MissionModeIn::Finished, true);
  TEST_ASSERT_FALSE(bfDone.tvcEnabled);
  TEST_ASSERT_TRUE(bfDone.bellyflopFlaps);
  ControlPhase bfExpired = controlPhaseFor(ActiveMission::Bellyflop, MissionModeIn::Running, true, true);
  TEST_ASSERT_FALSE(bfExpired.tvcEnabled);
  TEST_ASSERT_TRUE(bfExpired.bellyflopFlaps);
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
  ControlPhaseStep s = t.update(ActiveMission::Rth, MissionModeIn::Idle, false, false, 0);
  TEST_ASSERT_FALSE(s.phase.tvcEnabled);
  TEST_ASSERT_FALSE(s.resetTvc);
  // Countdown.
  s = t.update(ActiveMission::Waypoint, MissionModeIn::Startup, true, true, 0);
  TEST_ASSERT_FALSE(s.resetTvc);
  // Waypoint Running: enabled -> reset once.
  s = t.update(ActiveMission::Waypoint, MissionModeIn::Running, true, false, 0);
  TEST_ASSERT_TRUE(s.phase.tvcEnabled);
  TEST_ASSERT_TRUE(s.resetTvc);
  TEST_ASSERT_TRUE(s.changed);
  s = t.update(ActiveMission::Waypoint, MissionModeIn::Running, true, false, 0);
  TEST_ASSERT_FALSE(s.resetTvc);
  TEST_ASSERT_FALSE(s.changed);
  // Abort: waypoint -> RTH while enabled: reset (mission switched), stays enabled.
  s = t.update(ActiveMission::Rth, MissionModeIn::Running, true, true, 0);
  TEST_ASSERT_TRUE(s.phase.tvcEnabled);
  TEST_ASSERT_TRUE(s.resetTvc);
  TEST_ASSERT_FALSE(s.changed);
  // Bellyflop kick (waypoint -> bellyflop while on): stays on, reset (mission switched). Kick finished: off.
  s = t.update(ActiveMission::Bellyflop, MissionModeIn::Running, true, true, 0);
  TEST_ASSERT_TRUE(s.phase.tvcEnabled);
  TEST_ASSERT_TRUE(s.resetTvc);
  TEST_ASSERT_FALSE(s.changed);
  s = t.update(ActiveMission::Bellyflop, MissionModeIn::Finished, true, false, 0);
  TEST_ASSERT_FALSE(s.phase.tvcEnabled);
  TEST_ASSERT_TRUE(s.changed);
  s = t.update(ActiveMission::Freefall, MissionModeIn::Running, false, true, 0);
  TEST_ASSERT_FALSE(s.resetTvc);
  s = t.update(ActiveMission::Rth, MissionModeIn::Running, true, true, 0);
  TEST_ASSERT_TRUE(s.resetTvc);
  // SystemReset: next enable resets again even without a switch flag.
  t.reset();
  TEST_ASSERT_FALSE(t.tvcEnabled());
  s = t.update(ActiveMission::Rth, MissionModeIn::Running, true, false, 0);
  TEST_ASSERT_TRUE(s.resetTvc);
}

static void test_tracker_kick_guard() {
  // Defence in depth: a bellyflop still Running after the time limit + guard margin gets the TVC set off.
  const int64_t t0 = 5000000000LL;
  ControlPhaseTracker t;
  ControlPhaseStep s = t.update(ActiveMission::Waypoint, MissionModeIn::Running, true, false, t0 - 100000000LL);
  TEST_ASSERT_TRUE(s.phase.tvcEnabled);
  s = t.update(ActiveMission::Bellyflop, MissionModeIn::Running, true, true, t0);
  TEST_ASSERT_TRUE(s.phase.tvcEnabled);
  s = t.update(ActiveMission::Bellyflop, MissionModeIn::Running, true, false, t0 + kKickTimeLimitNs);
  TEST_ASSERT_TRUE(s.phase.tvcEnabled);
  s = t.update(ActiveMission::Bellyflop, MissionModeIn::Running, true, false,
               t0 + kKickTimeLimitNs + kKickGuardMarginNs);
  TEST_ASSERT_TRUE(s.phase.tvcEnabled);
  s = t.update(ActiveMission::Bellyflop, MissionModeIn::Running, true, false,
               t0 + kKickTimeLimitNs + kKickGuardMarginNs + 1);
  TEST_ASSERT_FALSE(s.phase.tvcEnabled);
  TEST_ASSERT_TRUE(s.phase.bellyflopFlaps);
  TEST_ASSERT_TRUE(s.changed);
  // Stays off for the rest of this bellyflop.
  s = t.update(ActiveMission::Bellyflop, MissionModeIn::Running, true, false, t0 + 10 * kKickTimeLimitNs);
  TEST_ASSERT_FALSE(s.phase.tvcEnabled);
  // A bellyflop never seen being switched to (e.g. after reset()) counts as expired.
  t.reset();
  s = t.update(ActiveMission::Bellyflop, MissionModeIn::Running, true, false, t0);
  TEST_ASSERT_FALSE(s.phase.tvcEnabled);
  // A new bellyflop (switch) starts a new kick window.
  s = t.update(ActiveMission::Bellyflop, MissionModeIn::Running, true, true, t0 + 20 * kKickTimeLimitNs);
  TEST_ASSERT_TRUE(s.phase.tvcEnabled);
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
  RUN_TEST(test_tracker_kick_guard);
  RUN_TEST(test_abort_decision);
  RUN_TEST(test_abort_decision_has_no_state_input);
  return UNITY_END();
}
