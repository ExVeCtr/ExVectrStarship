// Golden phase sequence of the whole configured flight (Starship plan profile 2, nominal, follower plant):
//   pio test -e native -f test_executive_golden
//
// The checked-in table below is the reference. Each phase entry is compared: phase, entry time after go (+- one
// 10 ms step), TVC mode, TVC reset, flap mode, TVC/flap actuator and motor requests, thrust cap. A change of this
// table must be explained in its commit and reviewed like code (design R10). On a mismatch the test prints the
// actual table in the same format.
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unity.h>

#include "../executive_support/exec_sim.hpp"
#include "executive/starship_plan.hpp"

using namespace VCTR;
using namespace VCTR::Exec;
using ExecTest::ExecSim;

void setUp() {}
void tearDown() {}

namespace {

struct GoldenRow {
  uint8_t phase;
  int32_t entryMs; ///< after go
  TvcMode tvc;
  bool resetTvc;
  FlapMode flaps;
  bool tvcActuators, flapActuators, motors;
  float thrustCapN;
};

constexpr float kNoCap = kNoThrustCapN;
using T = TvcMode;
using F = FlapMode;

// clang-format off
const GoldenRow kGolden[] = {
    // phase, entry ms after go, tvc, resetTvc, flaps, tvcAct, flapAct, motors, thrust cap
    { 8,      0, T::Off     , false, F::Disabled , false, false, false, kNoCap}, // Countdown (go at T-15 s)
    { 9,   5010, T::Off     , false, F::Retracted, true , true , true , kNoCap}, // PreLaunch (T-10 s, strict)
    { 1,  15010, T::Position, true , F::Retracted, true , true , true , kNoCap}, // Hover (T0)
    { 2,  21010, T::Position, false, F::Retracted, true , true , true , kNoCap}, // Ascent (hover check 5 s)
    {10,  40160, T::Position, false, F::Retracted, true , true , true , kNoCap}, // ApogeeHold (within 3 m)
    {11,  45170, T::Attitude, true , F::Retracted, true , true , true , 11.0f }, // Kick (5 s hold)
    { 3,  45920, T::Off     , false, F::Descent  , false, true , false, kNoCap}, // Descent (angle trigger)
    {12,  49290, T::Position, true , F::Upright  , true , true , true , kNoCap}, // Flip (stop altitude)
    { 5,  55440, T::Position, false, F::Retracted, true , true , true , kNoCap}, // RthTransit (< 0.5 m/s)
    { 6,  68530, T::Position, false, F::Retracted, true , true , true , kNoCap}, // Landing (home within 0.5 m)
    { 7,  89010, T::Off     , false, F::Disabled , false, false, false, kNoCap}, // Landed (tracking error 2 s)
};
// clang-format on

constexpr int32_t kToleranceMs = 10;

const char *tvcName(TvcMode t) { return t == T::Off ? "T::Off" : t == T::Position ? "T::Position" : "T::Attitude"; }
const char *flapName(FlapMode f) {
  switch (f) {
  case F::Disabled:
    return "F::Disabled";
  case F::Retracted:
    return "F::Retracted";
  case F::Extended:
    return "F::Extended";
  case F::Descent:
    return "F::Descent";
  case F::Upright:
    return "F::Upright";
  }
  return "?";
}

void printRow(const GoldenRow &r) {
  char line[200];
  snprintf(line, sizeof(line), "    {%2u, %6d, %-11s, %-5s, %-12s, %-5s, %-5s, %-5s, %s},", r.phase, int(r.entryMs),
           tvcName(r.tvc), r.resetTvc ? "true" : "false", flapName(r.flaps), r.tvcActuators ? "true" : "false",
           r.flapActuators ? "true" : "false", r.motors ? "true" : "false",
           r.thrustCapN >= kNoCap ? "kNoCap" : "11.0f");
  TEST_MESSAGE(line);
}

bool sameRow(const GoldenRow &a, const GoldenRow &b) {
  return a.phase == b.phase && labs(long(a.entryMs - b.entryMs)) <= kToleranceMs && a.tvc == b.tvc &&
         a.resetTvc == b.resetTvc && a.flaps == b.flaps && a.tvcActuators == b.tvcActuators &&
         a.flapActuators == b.flapActuators && a.motors == b.motors && a.thrustCapN == b.thrustCapN;
}

} // namespace

static void test_golden_nominal_flight() {
  ExecSim s(Starship::flightPlanProfile2());
  s.tick();
  s.go(-15);
  GoldenRow rows[32];
  int n = 0;
  uint8_t last = 0xFF;
  for (;;) {
    const ControlCommand &c = s.exec.command();
    if (c.phase != last && n < 32) {
      rows[n++] = {c.phase,  int32_t((s.clock.nowNs - s.goNs) / ExecTest::kMs), c.tvc, c.resetTvc, c.flaps,
                   c.tvcActuators, c.flapActuators, c.motors, c.thrustCapN};
      last = c.phase;
    }
    if (c.phase == Starship::id(Starship::Phase::Landed) || s.sinceGoS() > 400)
      break;
    s.tick();
  }
  const int want = int(sizeof(kGolden) / sizeof(kGolden[0]));
  bool ok = n == want;
  for (int i = 0; ok && i < n; i++)
    ok = sameRow(rows[i], kGolden[i]);
  if (!ok) {
    TEST_MESSAGE("actual golden table:");
    for (int i = 0; i < n; i++)
      printRow(rows[i]);
  }
  TEST_ASSERT_TRUE_MESSAGE(ok, "phase sequence differs from the golden table");
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_golden_nominal_flight);
  return UNITY_END();
}
