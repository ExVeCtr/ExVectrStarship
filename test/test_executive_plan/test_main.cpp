// Host tests of the flight plan validator (ExVectrControl/executive/validate_plan.*) and of the Starship plan table
// (src/executive/starship_plan.*): pio test -e native -f test_executive_plan
//
// validatePlan accepts the Starship plan and rejects broken copies of it (one defect each).
#include <math.h>
#include <string.h>
#include <unity.h>

#include "ExVectrControl/executive/validate_plan.hpp"
#include "executive/starship_constants.hpp"
#include "executive/starship_plan.hpp"

using namespace VCTR;
using namespace VCTR::Exec;
using Starship::Phase;

void setUp() {}
void tearDown() {}

namespace {

/// A mutable copy of the Starship plan (phase rows and go-to/descent parameters copied, the rest shared).
struct PlanCopy {
  PhaseSpec phases[kMaxPlanPhases];
  GoToParams goTo[8];
  DescentParams descent[2];
  FlightPlan plan;
  PlanCopy() {
    plan = Starship::flightPlanProfile2();
    for (int i = 0; i < plan.phaseCount; i++)
      phases[i] = plan.phases[i];
    for (int i = 0; i < plan.behaviours.goToCount; i++)
      goTo[i] = plan.behaviours.goTo[i];
    for (int i = 0; i < plan.behaviours.descentCount; i++)
      descent[i] = plan.behaviours.descent[i];
    plan.phases = phases;
    plan.behaviours.goTo = goTo;
    plan.behaviours.descent = descent;
  }
  PhaseSpec &phase(Phase p) {
    for (int i = 0; i < plan.phaseCount; i++)
      if (phases[i].id == Starship::id(p))
        return phases[i];
    TEST_FAIL_MESSAGE("phase not in plan");
    return phases[0];
  }
  PlanError check() const { return validatePlan(plan, Starship::planLimits()).error; }
};

void expectError(PlanError want, const PlanCopy &c) { TEST_ASSERT_EQUAL_STRING(toString(want), toString(c.check())); }

} // namespace

static void test_starship_plan_is_valid() {
  const PlanCheck c = validatePlan(Starship::flightPlanProfile2(), Starship::planLimits());
  TEST_ASSERT_EQUAL_STRING("ok", toString(c.error));
  TEST_ASSERT_EQUAL_UINT8(0xFF, c.phaseIndex);
}

static void test_starship_constants_owner_decisions() {
  TEST_ASSERT_EQUAL_FLOAT(90.0f, Starship::kMinKickAltitudeM);                       // X6
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 250.0f * Starship::kDeg, Starship::kKickEndPitchRateRadPs); // X5
  TEST_ASSERT_EQUAL_INT64(Starship::kSecondNs, Starship::kKickTimeLimitNs);
  TEST_ASSERT_EQUAL_FLOAT(11.0f, Starship::kKickThrustLimitN);
  // The plan rows carry them.
  PlanCopy c;
  const PhaseSpec &kick = c.phase(Phase::Kick);
  TEST_ASSERT_EQUAL_INT64(Starship::kKickTimeLimitNs, kick.timeoutNs);
  TEST_ASSERT_EQUAL_FLOAT(Starship::kKickThrustLimitN, kick.thrustCapN);
  TEST_ASSERT_TRUE(kick.entryGuard.guard == Guard::AltitudeAtLeast);
  TEST_ASSERT_EQUAL_UINT8(Starship::id(Phase::RthBrake), kick.entryGuard.fallback);
  TEST_ASSERT_EQUAL_FLOAT(90.0f, c.plan.guards[kick.entryGuard.paramIdx].altitudeM);
}

static void test_rejects_unreachable_phase() {
  PlanCopy c;
  c.phases[c.plan.phaseCount] = PhaseSpec::make(42, "Orphan").withOnGround(true);
  c.plan.phaseCount++;
  expectError(PlanError::Unreachable, c);
}

static void test_rejects_attitude_phase_without_finite_timeout_or_cap() {
  PlanCopy a;
  a.phase(Phase::Kick).timeoutNs = kNoTimeout;
  a.phase(Phase::Kick).onTimeout = kNoPhase;
  expectError(PlanError::AttitudeWithoutTimeout, a);
  PlanCopy b;
  b.phase(Phase::Kick).timeoutNs = 2 * Starship::kSecondNs; // longer than the hard limit
  expectError(PlanError::AttitudeWithoutTimeout, b);
  PlanCopy c;
  c.phase(Phase::Kick).thrustCapN = kNoThrustCapN;
  expectError(PlanError::AttitudeWithoutCap, c);
  PlanCopy d; // kick thrust request above its phase's cap
  d.phase(Phase::Kick).thrustCapN = 10.0f;
  expectError(PlanError::BadParamValue, d);
}

static void test_rejects_airborne_phase_ignoring_abort() {
  PlanCopy c;
  c.phase(Phase::Hover).abort = AbortRule::Ignore;
  expectError(PlanError::AirborneAbortIgnored, c);
  PlanCopy d;
  d.phase(Phase::Descent).abortNoPos = AbortRule::Ignore;
  expectError(PlanError::AirborneAbortIgnored, d);
}

static void test_rejects_non_finite_and_out_of_limit_numbers() {
  PlanCopy a;
  a.goTo[0].speedMps = NAN;
  expectError(PlanError::BadParamValue, a);
  PlanCopy b;
  b.goTo[1].speedMps = 50.0f; // above maxSpeedMps
  expectError(PlanError::BadParamValue, b);
  PlanCopy c;
  c.goTo[0].target[2] = INFINITY;
  expectError(PlanError::BadParamValue, c);
  PlanCopy d; // thrust-to-weight below 1: the stop distance is not finite
  d.descent[0].thrustN = 10.0f;
  expectError(PlanError::BadParamValue, d);
  PlanCopy e;
  e.phase(Phase::Hover).thrustCapN = NAN;
  expectError(PlanError::BadParamValue, e);
}

static void test_rejects_unknown_targets_and_ids() {
  PlanCopy a;
  a.phase(Phase::Ascent).exits[0].target = 99;
  expectError(PlanError::UnknownTarget, a);
  PlanCopy b;
  b.phase(Phase::Ascent).onTimeout = 99;
  expectError(PlanError::BadTimeout, b);
  PlanCopy c;
  c.phase(Phase::Hover).id = Starship::id(Phase::Ascent);
  expectError(PlanError::DuplicateId, c);
  PlanCopy d;
  d.plan.rth = 99;
  expectError(PlanError::MissingSpecialPhase, d);
  PlanCopy e;
  e.plan.first = e.plan.idle;
  expectError(PlanError::FirstIsIdle, e);
  PlanCopy f;
  f.phase(Phase::Flip).timeoutNs = kNoTimeout;
  f.phase(Phase::Flip).onTimeout = Starship::id(Phase::RthTransit); // a target without a timeout
  expectError(PlanError::BadTimeout, f);
  PlanCopy g;
  g.phase(Phase::Hover).exits[0].paramIdx = 200;
  expectError(PlanError::BadParamIndex, g);
}

static void test_rejects_unsafe_actuator_requests() {
  PlanCopy a;
  a.phase(Phase::Hover).tvcActuators = false; // motors on without the TVC output stage
  expectError(PlanError::MotorsWithoutTvcActuators, a);
  PlanCopy b;
  b.phase(Phase::Landed).flaps = FlapMode::Retracted;
  expectError(PlanError::UnsafeTerminalPhase, b);
  PlanCopy c;
  c.phase(Phase::Failsafe).tvc = TvcMode::Position;
  expectError(PlanError::UnsafeTerminalPhase, c);
  PlanCopy d; // the failsafe phase must not leave by itself
  d.phase(Phase::Failsafe) = d.phase(Phase::Failsafe).withExit(Guard::SpeedBelow, 9, Starship::id(Phase::Idle));
  expectError(PlanError::UnsafeTerminalPhase, d);
}

static void test_rejects_guard_misuse_and_overflow() {
  PlanCopy a; // Reached needs a behaviour with a target (Flip holds the measured position)
  a.phase(Phase::Flip).exits[0].guard = Guard::Reached;
  a.phase(Phase::Flip).exits[0].paramIdx = a.phase(Phase::RthTransit).exits[0].paramIdx; // a valid distance
  expectError(PlanError::GuardNeedsTarget, a);
  PlanCopy b;
  b.phase(Phase::Hover).exits[0].guard = Guard::KickAngleBelow;
  expectError(PlanError::GuardNeedsBehaviour, b);
  PlanCopy c;
  PhaseSpec &h = c.phase(Phase::Hover);
  h = h.withExit(Guard::SpeedBelow, 9, Starship::id(Phase::Ascent))
          .withExit(Guard::SpeedBelow, 9, Starship::id(Phase::Ascent))
          .withExit(Guard::SpeedBelow, 9, Starship::id(Phase::Ascent));
  TEST_ASSERT_TRUE(h.exitOverflow);
  expectError(PlanError::TooManyExits, c);
  PlanCopy d;
  d.phase(Phase::Kick).entryGuard.fallback = Starship::id(Phase::Kick);
  expectError(PlanError::BadEntryGuard, d);
  PlanCopy e;
  e.phase(Phase::Hover).abortNoPos = AbortRule::ToNoPos; // no noPos phase in profile 2 yet (T9b-7)
  expectError(PlanError::MissingAbortTarget, e);
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_starship_plan_is_valid);
  RUN_TEST(test_starship_constants_owner_decisions);
  RUN_TEST(test_rejects_unreachable_phase);
  RUN_TEST(test_rejects_attitude_phase_without_finite_timeout_or_cap);
  RUN_TEST(test_rejects_airborne_phase_ignoring_abort);
  RUN_TEST(test_rejects_non_finite_and_out_of_limit_numbers);
  RUN_TEST(test_rejects_unknown_targets_and_ids);
  RUN_TEST(test_rejects_unsafe_actuator_requests);
  RUN_TEST(test_rejects_guard_misuse_and_overflow);
  return UNITY_END();
}
