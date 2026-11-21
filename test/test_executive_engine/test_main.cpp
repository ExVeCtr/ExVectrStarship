// Host tests of the flight executive engine (ExVectrControl/executive/flight_executive.*):
//   pio test -e native -f test_executive_engine
//
// Step semantics with a small test-only plan (not the Starship plan): global priority order, one transition per
// step, the new phase's command in the same step, timeouts, entry actions, phaseSeq, entry guards, behaviour
// continuation, the TVC reset rule, the mission clock and the reset to the boot command.
#include <math.h>
#include <unity.h>

#include "ExVectrControl/executive/flight_executive.hpp"
#include "ExVectrControl/executive/sanitize.hpp"
#include "ExVectrControl/executive/validate_plan.hpp"

#include "../executive_support/exec_sim.hpp"

using namespace VCTR::Exec;
using ExecTest::kMs;
using ExecTest::kS;

void setUp() {}
void tearDown() {}

namespace {

enum : PhaseId { kIdle = 0, kA = 1, kB = 2, kC = 3, kR = 4, kG = 5, kH = 6, kDone = 7, kD = 8, kF = 9 };
enum : uint8_t { kGuardStopped = 0, kGuardAlways, kGuardReached, kGuardAlt50 };

constexpr GuardParams kGuards[] = {
    GuardParams::speedBelow(0.5f),
    GuardParams::speedBelow(1.0e6f), // always true for the test estimates
    GuardParams::distance(0.5f),
    GuardParams::altitudeAtLeast(50.0f),
};
constexpr GoToParams kGoTo[] = {{{0.0f, 0.0f, 10.0f}, 2.0f, 1.0f, 1.0f, GoToEntry::FromMeasured, {0, 0, 0}}};

constexpr PhaseSpec kPhases[] = {
    PhaseSpec::make(kIdle, "Idle").withOnGround(true),
    // A: first phase; timeout 1 s -> B; stopped -> C.
    PhaseSpec::make(kA, "A")
        .withTvc(TvcMode::Position)
        .withActuators(true, true)
        .withFlaps(FlapMode::Retracted)
        .withBehaviour(Behaviour::HoldMeasured)
        .withCtrlGroup(1)
        .withEntryActions(kEntryTvcActuatorTest)
        .withTimeout(1 * kS, kB)
        .withAbort(AbortRule::ToRth, AbortRule::ResetAndStop)
        .withExit(Guard::SpeedBelow, kGuardStopped, kC),
    // B: TVC off, left on the next step (always-true exit).
    PhaseSpec::make(kB, "B")
        .withBehaviour(Behaviour::HoldMeasured)
        .withAbort(AbortRule::ToRth, AbortRule::ToRth)
        .withExit(Guard::SpeedBelow, kGuardAlways, kC),
    // C: go-to, group 2; reached -> D.
    PhaseSpec::make(kC, "C")
        .withTvc(TvcMode::Position)
        .withActuators(true, true)
        .withBehaviour(Behaviour::GoTo, 0)
        .withCtrlGroup(2)
        .withAbort(AbortRule::ToRth, AbortRule::ToRth)
        .withExit(Guard::Reached, kGuardReached, kD),
    // D: the same go-to continued; stopped -> G (entry guard: altitude >= 50 m, else H).
    PhaseSpec::make(kD, "D")
        .withTvc(TvcMode::Position)
        .withActuators(true, true)
        .withBehaviour(Behaviour::GoTo, 0, BehaviourEntry::Continue)
        .withCtrlGroup(2)
        .withAbort(AbortRule::ToRth, AbortRule::ToRth)
        .withExit(Guard::SpeedBelow, kGuardStopped, kG),
    PhaseSpec::make(kG, "G")
        .withTvc(TvcMode::Position)
        .withActuators(true, true)
        .withBehaviour(Behaviour::HoldMeasured)
        .withCtrlGroup(3)
        .withEntryGuard(Guard::AltitudeAtLeast, kGuardAlt50, kH)
        .withAbort(AbortRule::ToRth, AbortRule::ToRth)
        .withExit(Guard::SpeedBelow, kGuardStopped, kDone),
    PhaseSpec::make(kH, "H")
        .withTvc(TvcMode::Position)
        .withActuators(true, true)
        .withBehaviour(Behaviour::HoldMeasured)
        .withCtrlGroup(3)
        .withAbort(AbortRule::Stay, AbortRule::Stay)
        .withExit(Guard::SpeedBelow, kGuardStopped, kDone),
    PhaseSpec::make(kR, "R")
        .withTvc(TvcMode::Position)
        .withActuators(true, true)
        .withBehaviour(Behaviour::HoldMeasured)
        .withCtrlGroup(3)
        .withAbort(AbortRule::Stay, AbortRule::Stay)
        .withExit(Guard::SpeedBelow, kGuardStopped, kDone),
    PhaseSpec::make(kDone, "Done").withOnGround(true).withAbort(AbortRule::ResetAndStop, AbortRule::ResetAndStop),
    PhaseSpec::make(kF, "F"),
};

FlightPlan makePlan() {
  FlightPlan p;
  p.phases = kPhases;
  p.phaseCount = sizeof(kPhases) / sizeof(kPhases[0]);
  p.idle = kIdle;
  p.first = kA;
  p.failsafe = kF;
  p.rth = kR;
  p.complete = kDone;
  p.behaviours.goTo = kGoTo;
  p.behaviours.goToCount = 1;
  p.guards = kGuards;
  p.guardCount = sizeof(kGuards) / sizeof(kGuards[0]);
  return p;
}

const FlightPlan kPlan = makePlan();
const PlanLimits kLimits = {14.0f, 1 * kS, 15.0f, 5.0f};

/// Manual inputs: the test sets the estimate and the edges; time advances by dt per step.
struct Rig {
  FlightExecutive ex{kPlan};
  ExecInputs in;
  Rig() {
    in.nowNs = 5 * kS;
    in.dtNs = 10 * kMs;
    setState(1.0f, 10.0f); // moving at 1 m/s at 10 m: SpeedBelow(0.5) false
  }
  void setState(float speed, float alt) {
    in.est.pos[0] = speed;
    in.est.pos[1] = in.est.pos[2] = 0;
    in.est.pos[3] = in.est.pos[4] = 0;
    in.est.pos[5] = alt;
    in.est.posValid = true;
    in.est.attValid = true;
  }
  const ControlCommand &step() {
    in.nowNs += in.dtNs;
    const ControlCommand &c = ex.step(in);
    in.cmd.goEdge = false;
    in.cmd.resetEdge = false;
    return c;
  }
  void runFor(double s) {
    const int64_t end = in.nowNs + int64_t(s * 1e9);
    while (in.nowNs < end)
      step();
  }
  const ControlCommand &go(int16_t startS = 0) {
    in.cmd.goEdge = true;
    in.cmd.legacyStartS = startS;
    return step();
  }
};

} // namespace

static void test_plan_is_valid() {
  const PlanCheck c = validatePlan(kPlan, kLimits);
  TEST_ASSERT_EQUAL_STRING("ok", toString(c.error));
}

static void test_boot_command() {
  Rig r;
  TEST_ASSERT_EQUAL_UINT8(kIdle, r.ex.phase());
  TEST_ASSERT_FALSE(r.ex.committed());
  const ControlCommand &c = r.step();
  TEST_ASSERT_EQUAL_UINT8(kIdle, c.phase);
  TEST_ASSERT_TRUE(ExecTest::allOff(c));
  TEST_ASSERT_EQUAL_INT64(0, c.missionTimeNs);
  TEST_ASSERT_EQUAL_UINT32(0, c.phaseSeq);
  TEST_ASSERT_TRUE(c.onGround);
  TEST_ASSERT_FALSE(c.sanitized);
}

static void test_go_enters_first_phase_with_its_command_in_the_same_step() {
  Rig r;
  r.step();
  const ControlCommand &c = r.go();
  TEST_ASSERT_EQUAL_UINT8(kA, c.phase);
  TEST_ASSERT_TRUE(r.ex.status().go == ExecStatus::GoResult::Accepted);
  TEST_ASSERT_TRUE(r.ex.status().transition == ExitReason::Go);
  TEST_ASSERT_TRUE(c.tvc == TvcMode::Position);
  TEST_ASSERT_TRUE(c.resetTvc); // Off -> on
  TEST_ASSERT_TRUE(c.tvcActuators && c.motors && c.flapActuators);
  TEST_ASSERT_TRUE(c.attSpSource == AttSpSource::Mapper);
  TEST_ASSERT_EQUAL_UINT32(1, c.phaseSeq);
  TEST_ASSERT_EQUAL_UINT16(1, c.actuatorTestSeq); // entry action
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 10.0f, c.posSp[5]); // HoldMeasured from this step's estimate
  TEST_ASSERT_TRUE(r.ex.committed());
  // Next step: no reset any more, entry action not repeated.
  const ControlCommand &c2 = r.step();
  TEST_ASSERT_FALSE(c2.resetTvc);
  TEST_ASSERT_EQUAL_UINT16(1, c2.actuatorTestSeq);
  TEST_ASSERT_EQUAL_UINT32(1, c2.phaseSeq);
}

static void test_go_refused_outside_idle_and_while_aborting() {
  Rig r;
  r.go();
  r.go();
  TEST_ASSERT_TRUE(r.ex.status().go == ExecStatus::GoResult::RefusedNotIdle);
  TEST_ASSERT_EQUAL_UINT8(kA, r.ex.phase());
  Rig r2;
  r2.in.cmd.abortRequest = true;
  r2.go();
  TEST_ASSERT_TRUE(r2.ex.status().go == ExecStatus::GoResult::RefusedBlocked);
  TEST_ASSERT_TRUE(r2.ex.status().abort == ExecStatus::AbortOutcome::Ignored);
  TEST_ASSERT_EQUAL_UINT8(kIdle, r2.ex.phase());
}

static void test_failsafe_has_top_priority_and_is_left_only_by_reset() {
  Rig r;
  r.go();
  r.in.sup.failsafe = true;
  r.in.cmd.resetEdge = true;
  r.in.cmd.abortRequest = true;
  const ControlCommand &c = r.step();
  TEST_ASSERT_EQUAL_UINT8(kF, c.phase);
  TEST_ASSERT_TRUE(ExecTest::allOff(c));
  TEST_ASSERT_TRUE(r.ex.status().transition == ExitReason::Failsafe);
  // Latch still set: a reset edge does nothing.
  r.in.cmd.resetEdge = true;
  r.step();
  TEST_ASSERT_EQUAL_UINT8(kF, r.ex.phase());
  // Latch cleared but no reset: stays.
  r.in.sup.failsafe = false;
  r.in.cmd.abortRequest = false;
  r.runFor(1.0);
  TEST_ASSERT_EQUAL_UINT8(kF, r.ex.phase());
  r.in.cmd.resetEdge = true;
  r.step();
  TEST_ASSERT_EQUAL_UINT8(kIdle, r.ex.phase());
}

static void test_reset_beats_abort_and_no_pos_beats_abort() {
  Rig r;
  r.go();
  r.in.cmd.resetEdge = true;
  r.in.cmd.abortRequest = true;
  r.step();
  TEST_ASSERT_EQUAL_UINT8(kIdle, r.ex.phase());
  TEST_ASSERT_TRUE(r.ex.status().transition == ExitReason::Reset);

  Rig r2; // A: abort -> R, abortNoPos -> ResetAndStop (idle)
  r2.go();
  r2.in.cmd.abortRequest = true;
  r2.in.sup.abortNoPos = true;
  r2.step();
  TEST_ASSERT_EQUAL_UINT8(kIdle, r2.ex.phase());
  TEST_ASSERT_TRUE(r2.ex.status().transition == ExitReason::AbortNoPos);
  TEST_ASSERT_TRUE(r2.ex.status().abort == ExecStatus::AbortOutcome::ResetAndStop);

  Rig r3; // magnetometer timeout uses the abort rule
  r3.go();
  r3.in.sup.magTimeout = true;
  r3.step();
  TEST_ASSERT_EQUAL_UINT8(kR, r3.ex.phase());
}

static void test_abort_level_is_idempotent_and_stay_lets_exits_run() {
  Rig r;
  r.go();
  r.in.cmd.abortRequest = true;
  r.step();
  TEST_ASSERT_EQUAL_UINT8(kR, r.ex.phase());
  TEST_ASSERT_TRUE(r.ex.status().abort == ExecStatus::AbortOutcome::ToRth);
  const uint32_t seq = r.ex.command().phaseSeq;
  for (int i = 0; i < 100; i++) {
    r.step();
    TEST_ASSERT_TRUE(r.ex.status().abort == ExecStatus::AbortOutcome::Stayed);
  }
  TEST_ASSERT_EQUAL_UINT8(kR, r.ex.phase());
  TEST_ASSERT_EQUAL_UINT32(seq, r.ex.command().phaseSeq);
  // Abort still held, the phase's own exit still works.
  r.setState(0.0f, 10.0f);
  r.step();
  TEST_ASSERT_EQUAL_UINT8(kDone, r.ex.phase());
  TEST_ASSERT_TRUE(r.ex.status().missionComplete);
  TEST_ASSERT_TRUE(ExecTest::allOff(r.ex.command()));
  // Abort in the complete phase: reset and stop.
  r.step();
  TEST_ASSERT_FALSE(r.ex.status().missionComplete); // one step only
  TEST_ASSERT_EQUAL_UINT8(kIdle, r.ex.phase());
}

static void test_timeout_exact_and_before_exits() {
  Rig r;
  r.go();
  const int64_t entry = r.ex.phaseEntryNs();
  while (r.in.nowNs + r.in.dtNs < entry + 1 * kS) {
    r.step();
    TEST_ASSERT_EQUAL_UINT8(kA, r.ex.phase());
  }
  r.step(); // elapsed == 1 s exactly: timeout (>=)
  TEST_ASSERT_EQUAL_INT64(entry + 1 * kS, r.in.nowNs);
  TEST_ASSERT_EQUAL_UINT8(kB, r.ex.phase());
  TEST_ASSERT_TRUE(r.ex.status().transition == ExitReason::Timeout);

  // Timeout and exit true in the same step: the timeout wins.
  Rig r2;
  r2.go();
  r2.setState(0.0f, 10.0f);
  r2.in.dtNs = 2 * kS;
  r2.step();
  TEST_ASSERT_EQUAL_UINT8(kB, r2.ex.phase());
}

static void test_one_transition_per_step() {
  Rig r;
  r.go();
  r.in.dtNs = 1 * kS; // A times out on the next step
  r.step();
  TEST_ASSERT_EQUAL_UINT8(kB, r.ex.phase()); // not straight on to C although B's exit is always true
  r.in.dtNs = 10 * kMs;
  const ControlCommand &c = r.step();
  TEST_ASSERT_EQUAL_UINT8(kC, c.phase);
}

static void test_tvc_reset_rule() {
  Rig r;
  TEST_ASSERT_TRUE(r.go().resetTvc); // Off -> Position (A, group 1)
  r.in.dtNs = 1 * kS;
  const ControlCommand &b = r.step(); // -> B (Off)
  TEST_ASSERT_TRUE(b.tvc == TvcMode::Off);
  TEST_ASSERT_FALSE(b.resetTvc);
  r.in.dtNs = 10 * kMs;
  r.setState(1.0f, 0.0f); // away from the go-to target (0,0,10)
  TEST_ASSERT_TRUE(r.step().resetTvc); // -> C (Position after Off)
  TEST_ASSERT_FALSE(r.step().resetTvc); // still C
  // C -> D: same group 2, TVC stays on: no reset.
  r.setState(0.0f, 10.0f); // at the go-to target (0,0,10): Reached
  const ControlCommand &d = r.step();
  TEST_ASSERT_EQUAL_UINT8(kD, d.phase);
  TEST_ASSERT_FALSE(d.resetTvc);
  // D -> G (group 3, stays on): reset. Altitude 60 m passes G's entry guard.
  r.setState(0.0f, 60.0f);
  const ControlCommand &g = r.step();
  TEST_ASSERT_EQUAL_UINT8(kG, g.phase);
  TEST_ASSERT_TRUE(g.resetTvc);
}

static void test_entry_guard_redirects_to_fallback() {
  Rig r;
  r.go();
  r.in.dtNs = 1 * kS;
  r.step(); // B
  r.in.dtNs = 10 * kMs;
  r.step(); // C
  r.setState(0.0f, 10.0f);
  r.step(); // D (reached)
  TEST_ASSERT_EQUAL_UINT8(kD, r.ex.phase());
  const uint32_t seq = r.ex.command().phaseSeq;
  r.step(); // stopped at 10 m: G refuses entry below 50 m -> H
  TEST_ASSERT_EQUAL_UINT8(kH, r.ex.phase());
  TEST_ASSERT_TRUE(r.ex.status().entryRedirected);
  TEST_ASSERT_EQUAL_UINT32(seq + 1, r.ex.command().phaseSeq); // one entry, not two
}

static void test_behaviour_continue_keeps_memory() {
  Rig r;
  r.go();
  r.in.dtNs = 1 * kS;
  r.step(); // B
  r.in.dtNs = 10 * kMs;
  r.setState(1.0f, 0.0f); // start the go-to at 0 m
  r.step();               // C: go-to initialised at the measured position
  for (int i = 0; i < 50; i++)
    r.step();
  TEST_ASSERT_TRUE(r.ex.behaviourMemory().goTo.speedMps > 0.4f);
  const float spBefore = r.ex.behaviourMemory().goTo.spPos[2];
  TEST_ASSERT_TRUE(spBefore > 0.1f && spBefore < 1.0f);
  // Jump the estimate to the target: C -> D (Continue): the setpoint continues from where it was (Init would
  // restart it at the measured 10 m). The legacy braking cap sqrt(d * a) with d = 0 then stops the speed.
  r.setState(1.0f, 10.0f);
  const ControlCommand &d = r.step();
  TEST_ASSERT_EQUAL_UINT8(kD, d.phase);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, spBefore, d.posSp[5]);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, r.ex.behaviourMemory().goTo.speedMps);
}

static void test_reset_returns_to_the_boot_command() {
  Rig fresh;
  Rig r;
  r.go();
  r.runFor(0.5);
  r.in.cmd.resetEdge = true;
  const ControlCommand c = r.step();
  TEST_ASSERT_EQUAL_UINT8(kIdle, c.phase);
  TEST_ASSERT_TRUE(r.ex.status().transition == ExitReason::Reset);
  fresh.in = r.in;
  fresh.in.nowNs -= fresh.in.dtNs;
  const ControlCommand f = fresh.step();
  TEST_ASSERT_TRUE(ExecTest::sameCommand(f, c));
  TEST_ASSERT_TRUE(c.phaseSeq > f.phaseSeq); // monotonic counters survive
  TEST_ASSERT_EQUAL_UINT16(1, c.actuatorTestSeq);
  // reset() method: same boot state.
  Rig r2;
  r2.go();
  r2.runFor(0.3);
  r2.ex.reset();
  TEST_ASSERT_EQUAL_UINT8(kIdle, r2.ex.phase());
  const ControlCommand c2 = r2.step();
  fresh.in = r2.in;
  fresh.in.nowNs -= fresh.in.dtNs;
  TEST_ASSERT_TRUE(ExecTest::sameCommand(fresh.step(), c2));
}

static void test_mission_clock() {
  Rig r;
  r.step();
  TEST_ASSERT_EQUAL_INT64(0, r.ex.command().missionTimeNs);
  r.go(-15);
  TEST_ASSERT_EQUAL_INT64(-15 * kS, r.ex.command().missionTimeNs);
  r.step();
  TEST_ASSERT_EQUAL_INT64(-15 * kS + 10 * kMs, r.ex.command().missionTimeNs);
  r.in.cmd.resetEdge = true;
  r.step();
  TEST_ASSERT_EQUAL_INT64(0, r.ex.command().missionTimeNs);
}

static void test_sanitizer_unit() {
  EstimateSnapshot est;
  est.pos[3] = 1;
  est.pos[4] = 2;
  est.pos[5] = 3;
  const float last[3] = {7, 8, 9};
  ControlCommand c;
  TEST_ASSERT_FALSE(sanitizeCommand(c, est, last));
  c.posSp[1] = NAN;
  c.posSp[5] = 4;
  TEST_ASSERT_TRUE(sanitizeCommand(c, est, last));
  TEST_ASSERT_EQUAL_FLOAT(0.0f, c.posSp[1]);
  TEST_ASSERT_EQUAL_FLOAT(3.0f, c.posSp[5]); // hold measured
  est.pos[5] = INFINITY;
  c.posSp[0] = NAN;
  sanitizeCommand(c, est, last);
  TEST_ASSERT_EQUAL_FLOAT(9.0f, c.posSp[5]); // measured not finite: last finite setpoint
  ControlCommand k;
  k.tvc = TvcMode::Attitude;
  k.attSpSource = AttSpSource::Command;
  k.thrustN = 11;
  k.attSp[2] = NAN;
  TEST_ASSERT_TRUE(sanitizeCommand(k, est, last));
  TEST_ASSERT_TRUE(k.tvc == TvcMode::Off);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, k.thrustN);
  TEST_ASSERT_TRUE(k.attSpSource == AttSpSource::None);
  TEST_ASSERT_TRUE(ExecTest::commandFinite(k));
  ControlCommand f;
  f.flapPitchRad = NAN;
  f.thrustCapN = NAN;
  TEST_ASSERT_TRUE(sanitizeCommand(f, est, last));
  TEST_ASSERT_TRUE(ExecTest::commandFinite(f));
}

static void test_non_finite_estimate_is_sanitized_and_zero_dt_is_safe() {
  Rig r;
  r.go();
  r.in.est.pos[5] = NAN; // HoldMeasured would publish NaN
  const ControlCommand &c = r.step();
  TEST_ASSERT_TRUE(c.sanitized);
  TEST_ASSERT_TRUE(ExecTest::commandFinite(c));
  // Zero and negative dt in a go-to phase: finite output.
  Rig r2;
  r2.go();
  r2.in.dtNs = 1 * kS;
  r2.step();
  r2.in.dtNs = 0;
  r2.step(); // C
  for (int i = 0; i < 3; i++) {
    r2.in.dtNs = i == 0 ? 0 : -5 * kMs;
    TEST_ASSERT_TRUE(ExecTest::commandFinite(r2.step()));
    TEST_ASSERT_FALSE(r2.ex.command().sanitized);
  }
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_plan_is_valid);
  RUN_TEST(test_boot_command);
  RUN_TEST(test_go_enters_first_phase_with_its_command_in_the_same_step);
  RUN_TEST(test_go_refused_outside_idle_and_while_aborting);
  RUN_TEST(test_failsafe_has_top_priority_and_is_left_only_by_reset);
  RUN_TEST(test_reset_beats_abort_and_no_pos_beats_abort);
  RUN_TEST(test_abort_level_is_idempotent_and_stay_lets_exits_run);
  RUN_TEST(test_timeout_exact_and_before_exits);
  RUN_TEST(test_one_transition_per_step);
  RUN_TEST(test_tvc_reset_rule);
  RUN_TEST(test_entry_guard_redirects_to_fallback);
  RUN_TEST(test_behaviour_continue_keeps_memory);
  RUN_TEST(test_reset_returns_to_the_boot_command);
  RUN_TEST(test_mission_clock);
  RUN_TEST(test_sanitizer_unit);
  RUN_TEST(test_non_finite_estimate_is_sanitized_and_zero_dt_is_safe);
  return UNITY_END();
}
