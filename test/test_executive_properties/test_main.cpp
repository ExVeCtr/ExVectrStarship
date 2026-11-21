// Property tests of the Starship flight plan on the flight executive: pio test -e native -f test_executive_properties
//
// For EVERY phase of the plan (reached through scripted closed-loop paths, then branched by copying the simulation):
// abort -> the outcome of the abort table (design 4.1), repeated abort idempotent, magnetometer timeout = abort,
// reset -> idle and the boot command, failsafe -> Failsafe with everything off and left only by a reset.
// Plus: every attitude-mode phase has a finite timeout and cap, and a seeded fuzz of 10^5 steps with random events,
// estimates (including non-finite values) and step intervals checks the command invariants.
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unity.h>

#include "../executive_support/exec_sim.hpp"
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

/// One simulation snapshot per phase id, taken on the first step in that phase.
struct PhaseSnapshots {
  ExecSim *sims[256] = {};
  ~PhaseSnapshots() {
    for (ExecSim *s : sims)
      delete s;
  }
  void take(const ExecSim &s) {
    const uint8_t p = s.exec.phase();
    if (!sims[p])
      sims[p] = new ExecSim(s); // test code: heap is fine here
  }
  void collect(ExecSim s, double maxS) {
    const int64_t end = s.clock.nowNs + int64_t(maxS * 1e9);
    take(s);
    while (s.clock.nowNs < end) {
      s.tick();
      take(s);
    }
  }
};

PhaseSnapshots &snapshots() {
  static PhaseSnapshots snaps;
  static bool done = false;
  if (!done) {
    done = true;
    ExecSim nominal(plan());
    nominal.tick();
    snaps.take(nominal); // Idle
    nominal.go(-15);
    snaps.collect(nominal, 400); // Countdown ... Landed
    ExecSim abortHover(plan());
    abortHover.go(-15);
    abortHover.runUntilPhase(id(Phase::Hover), 30);
    abortHover.runFor(2);
    abortHover.next.cmd.abortRequest = true;
    abortHover.tick();
    abortHover.next.cmd.abortRequest = false;
    snaps.collect(abortHover, 5); // RthBrake
    ExecSim fail(plan());
    fail.go(-15);
    fail.next.sup.failsafe = true;
    fail.tick();
    fail.next.sup.failsafe = false; // latch cleared, phase stays Failsafe
    snaps.take(fail);
  }
  return snaps;
}

/// Expected phase after one abort step in phase p (design 4.1).
uint8_t abortTarget(uint8_t p) {
  switch (Phase(p)) {
  case Phase::Countdown:
  case Phase::PreLaunch:
  case Phase::Landed:
    return id(Phase::Idle); // reset and stop
  case Phase::Hover:
  case Phase::Ascent:
  case Phase::ApogeeHold:
  case Phase::Kick:
  case Phase::Descent:
    return id(Phase::RthBrake); // RTH
  default:
    return p; // Idle, Failsafe: ignored; Flip and the RTH phases: stay
  }
}

void forEveryPhase(void (*check)(const ExecSim &, uint8_t)) {
  PhaseSnapshots &snaps = snapshots();
  for (int i = 0; i < plan().phaseCount; i++) {
    const uint8_t p = plan().phases[i].id;
    char msg[64];
    snprintf(msg, sizeof(msg), "phase %u (%s) not reached by the scripted paths", p, plan().phases[i].name);
    TEST_ASSERT_NOT_NULL_MESSAGE(snaps.sims[p], msg);
    check(*snaps.sims[p], p);
  }
}

char gMsg[96];
const char *phaseMsg(uint8_t p, const char *what) {
  snprintf(gMsg, sizeof(gMsg), "phase %u: %s", p, what);
  return gMsg;
}

/// Checks one abort (or magnetometer-timeout) step from phase p. Where the rule is Stay or Ignore the phase's own
/// exits still run in that step, so the outcome is checked, not the phase.
void checkAbortStep(const ExecSim &s, uint8_t p, const char *what) {
  const uint8_t want = abortTarget(p);
  if (want != p) {
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(want, s.exec.phase(), phaseMsg(p, what));
    TEST_ASSERT_TRUE_MESSAGE(s.exec.status().transition == ExitReason::Abort, phaseMsg(p, what));
  } else {
    TEST_ASSERT_TRUE_MESSAGE(s.exec.status().transition != ExitReason::Abort, phaseMsg(p, what));
    const ExecStatus::AbortOutcome o = s.exec.status().abort;
    TEST_ASSERT_TRUE_MESSAGE(o == ExecStatus::AbortOutcome::Stayed || o == ExecStatus::AbortOutcome::Ignored,
                             phaseMsg(p, what));
  }
}

void checkAbort(const ExecSim &base, uint8_t p) {
  ExecSim s = base;
  s.next.cmd.abortRequest = true;
  s.tick();
  checkAbortStep(s, p, "abort outcome");
  // Held for 2 s: idempotent (only the target phase's own exits may move on, never another abort transition).
  for (int i = 0; i < 200; i++) {
    s.tick();
    TEST_ASSERT_TRUE_MESSAGE(s.exec.status().transition != ExitReason::Abort, phaseMsg(p, "repeated abort"));
  }
  // Magnetometer timeout: the same rule (X11).
  ExecSim m = base;
  m.next.sup.magTimeout = true;
  m.tick();
  checkAbortStep(m, p, "mag timeout outcome");
}

void checkReset(const ExecSim &base, uint8_t p) {
  ExecSim s = base;
  s.next.cmd.resetEdge = true;
  const ControlCommand c = s.tick();
  TEST_ASSERT_EQUAL_UINT8_MESSAGE(id(Phase::Idle), c.phase, phaseMsg(p, "reset"));
  ExecSim fresh(plan());
  fresh.clock = base.clock;
  fresh.plant = base.plant;
  const ControlCommand f = fresh.tick();
  TEST_ASSERT_TRUE_MESSAGE(ExecTest::sameCommand(f, c), phaseMsg(p, "reset command differs from boot"));
  TEST_ASSERT_FALSE(s.exec.committed());
  s.runFor(1); // nothing restarts
  TEST_ASSERT_EQUAL_UINT8_MESSAGE(id(Phase::Idle), s.exec.phase(), phaseMsg(p, "after reset"));
}

void checkFailsafe(const ExecSim &base, uint8_t p) {
  ExecSim s = base;
  s.next.sup.failsafe = true;
  s.next.cmd.abortRequest = true;
  s.tick();
  TEST_ASSERT_EQUAL_UINT8_MESSAGE(id(Phase::Failsafe), s.exec.phase(), phaseMsg(p, "failsafe"));
  TEST_ASSERT_TRUE_MESSAGE(ExecTest::allOff(s.exec.command()), phaseMsg(p, "failsafe not all off"));
  s.next.cmd.resetEdge = true; // latched: the reset is not taken
  s.tick();
  TEST_ASSERT_EQUAL_UINT8(id(Phase::Failsafe), s.exec.phase());
  s.next.sup.failsafe = false;
  s.next.cmd.abortRequest = false;
  s.runFor(1);
  TEST_ASSERT_EQUAL_UINT8(id(Phase::Failsafe), s.exec.phase());
  s.next.cmd.resetEdge = true;
  s.tick();
  TEST_ASSERT_EQUAL_UINT8(id(Phase::Idle), s.exec.phase());
}

} // namespace

static void test_abort_from_every_phase() { forEveryPhase(checkAbort); }
static void test_reset_from_every_phase_reaches_idle() { forEveryPhase(checkReset); }
static void test_failsafe_from_every_phase() { forEveryPhase(checkFailsafe); }

static void test_timeouts_finite_where_required() {
  const PlanLimits &lim = Starship::planLimits();
  int attitudePhases = 0;
  for (int i = 0; i < plan().phaseCount; i++) {
    const PhaseSpec &ph = plan().phases[i];
    if (ph.timeoutNs != kNoTimeout) {
      TEST_ASSERT_TRUE(ph.timeoutNs > 0);
      TEST_ASSERT_TRUE(ph.onTimeout != kNoPhase);
    }
    if (ph.tvc == TvcMode::Attitude) {
      attitudePhases++;
      TEST_ASSERT_TRUE(ph.timeoutNs <= lim.maxAttitudePhaseNs);
      TEST_ASSERT_TRUE(ph.thrustCapN <= lim.maxThrustN);
    }
  }
  TEST_ASSERT_EQUAL_INT(1, attitudePhases); // the kick
}

static void test_fuzz_invariants() {
  srand(20261121);
  ExecSim s(plan());
  const PlanLimits &lim = Starship::planLimits();
  bool failsafeLatched = false;
  int transitions = 0, phasesSeen[256] = {};
  for (int k = 0; k < 100000; k++) {
    const int r = rand() % 1000;
    s.next.cmd.goEdge = r < 10;
    s.next.cmd.legacyStartS = int16_t(rand() % 30 - 20);
    s.next.cmd.resetEdge = r >= 10 && r < 20;
    if (s.next.cmd.resetEdge)
      failsafeLatched = false; // the supervisor clears its latch together with the reset
    if (r >= 20 && r < 25)
      s.next.cmd.abortRequest = !s.next.cmd.abortRequest;
    if (r == 30)
      failsafeLatched = true;
    if (r >= 40 && r < 42)
      s.next.sup.magTimeout = !s.next.sup.magTimeout;
    s.next.sup.failsafe = failsafeLatched;
    // Random estimate disturbances, sometimes non-finite; random step interval, sometimes 0 or negative.
    if (r >= 100 && r < 110)
      s.plant.pos[rand() % 6] = (rand() % 3 == 0) ? NAN : float(rand() % 400 - 100);
    if (r >= 110 && r < 115)
      s.plant.att[rand() % 7] = (rand() % 2 == 0) ? INFINITY : float(rand() % 3 - 1);
    for (float &v : s.plant.pos)
      if (!isfinite(v) && rand() % 4 == 0)
        v = 0.0f;
    for (float &v : s.plant.att)
      if (!isfinite(v) && rand() % 4 == 0)
        v = 0.0f;
    s.clock.dtNs = (r >= 200 && r < 203) ? -int64_t(rand() % 5) * ExecTest::kMs : int64_t(rand() % 50) * ExecTest::kMs;

    const uint8_t before = s.exec.phase();
    const ControlCommand &c = s.tick();
    const PhaseSpec &ph = s.exec.phaseSpec();
    phasesSeen[c.phase]++;
    transitions += (c.phase != before) ? 1 : 0;

    char msg[64];
    snprintf(msg, sizeof(msg), "step %d phase %u", k, c.phase);
    TEST_ASSERT_TRUE_MESSAGE(findPhase(plan(), c.phase) >= 0, msg);
    TEST_ASSERT_TRUE_MESSAGE(ExecTest::commandFinite(c), msg);
    TEST_ASSERT_TRUE_MESSAGE(!c.motors || c.tvcActuators, msg);
    TEST_ASSERT_TRUE_MESSAGE(c.flapActuators == (c.flaps != FlapMode::Disabled), msg);
    if (c.tvc == TvcMode::Attitude)
      TEST_ASSERT_TRUE_MESSAGE(ph.timeoutNs <= lim.maxAttitudePhaseNs && c.thrustCapN <= lim.maxThrustN, msg);
    if (c.phase == id(Phase::Idle) || c.phase == id(Phase::Landed) || c.phase == id(Phase::Failsafe))
      TEST_ASSERT_TRUE_MESSAGE(ExecTest::allOff(c), msg);
    if (failsafeLatched)
      TEST_ASSERT_EQUAL_UINT8_MESSAGE(id(Phase::Failsafe), c.phase, msg);
    if (before == id(Phase::Failsafe) && c.phase != before)
      TEST_ASSERT_TRUE_MESSAGE(s.exec.status().transition == ExitReason::Reset, msg);
  }
  // The fuzz must actually exercise the plan.
  int distinct = 0;
  for (int n : phasesSeen)
    distinct += n > 0 ? 1 : 0;
  char cover[96];
  snprintf(cover, sizeof(cover), "fuzz: %d transitions, %d distinct phases", transitions, distinct);
  TEST_MESSAGE(cover);
  TEST_ASSERT_TRUE_MESSAGE(transitions > 400, cover); // every-phase coverage: the scripted tests above
  TEST_ASSERT_TRUE_MESSAGE(distinct >= 8, cover);
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_abort_from_every_phase);
  RUN_TEST(test_reset_from_every_phase_reaches_idle);
  RUN_TEST(test_failsafe_from_every_phase);
  RUN_TEST(test_timeouts_finite_where_required);
  RUN_TEST(test_fuzz_invariants);
  return UNITY_END();
}
