// Host tests of src/safety/motor_enable.cpp: pio test -e native -f test_motor_enable
//
// (1) The mapping with request Unchanged is bit-identical to the old vehicleShutdownControl behaviour for every input
// combination (the reference below is the old code, copied before the change). (2) Request semantics. (3) The
// freefall pre-spin timing: Enable once inside the lead, never before, never twice, off on shutdown, no thrust command.
#include <math.h>
#include <unity.h>

#include "safety/motor_enable.hpp"

using namespace VCTR::Safety;

void setUp() {}
void tearDown() {}

// The old main.cpp code (pre feature), as a pure reference. Old state: enableMotors was set only by events.
struct Old {
  bool motors, tvc, flaps, simTvc;
};
static Old oldVehicleShutdownControl(bool shutdown, bool missionAct, bool freefall, bool motorsNow) {
  Old o{motorsNow, false, false, false};
  if (shutdown)
    return o; // starshipTVC.enableActuators(false); flaps false; bodySimulator.enableTVC(false)
  o.flaps = freefall ? true : missionAct;
  o.tvc = missionAct;
  o.simTvc = missionAct;
  return o;
}

static void test_unchanged_bit_identical_to_old() {
  for (int sd = 0; sd < 2; sd++)
    for (int act = 0; act < 2; act++)
      for (int ff = 0; ff < 2; ff++)
        for (int mn = 0; mn < 2; mn++) {
          MotorEnableInput in;
          in.shutdown = sd; in.missionActuatorsEnabled = act; in.freefallActive = ff; in.motorsEnabledNow = mn;
          in.request = MotorsRequest::Unchanged;
          const MotorEnableOutput n = motorEnableFor(in);
          const Old o = oldVehicleShutdownControl(sd, act, ff, mn);
          TEST_ASSERT_EQUAL(o.motors, n.enableMotors);
          TEST_ASSERT_EQUAL(o.tvc, n.tvcActuators);
          TEST_ASSERT_EQUAL(o.flaps, n.flapActuators);
          TEST_ASSERT_EQUAL(o.simTvc, n.simTvc);
        }
}

static void test_shutdown_ignores_every_request() {
  const MotorsRequest reqs[] = {MotorsRequest::Unchanged, MotorsRequest::Enable, MotorsRequest::Disable};
  for (MotorsRequest r : reqs)
    for (int act = 0; act < 2; act++)
      for (int mn = 0; mn < 2; mn++) {
        MotorEnableInput in;
        in.shutdown = true; in.missionActuatorsEnabled = act; in.request = r; in.motorsEnabledNow = mn;
        const MotorEnableOutput o = motorEnableFor(in);
        TEST_ASSERT_FALSE(o.tvcActuators);
        TEST_ASSERT_FALSE(o.flapActuators);
        TEST_ASSERT_FALSE(o.simTvc);
        TEST_ASSERT_EQUAL(mn, o.enableMotors); // as before: untouched
      }
}

static void test_request_semantics() {
  MotorEnableInput in;
  in.shutdown = false; in.freefallActive = true; in.missionActuatorsEnabled = false; in.motorsEnabledNow = true;
  in.request = MotorsRequest::Enable;
  MotorEnableOutput o = motorEnableFor(in);
  TEST_ASSERT_TRUE(o.enableMotors);
  TEST_ASSERT_TRUE(o.tvcActuators); // the hard gate needs it
  TEST_ASSERT_TRUE(o.flapActuators);
  TEST_ASSERT_FALSE(o.simTvc);      // simulated thrust unchanged
  in.motorsEnabledNow = false;
  TEST_ASSERT_TRUE(motorEnableFor(in).enableMotors);
  in.request = MotorsRequest::Disable; in.missionActuatorsEnabled = true; in.motorsEnabledNow = true;
  o = motorEnableFor(in);
  TEST_ASSERT_FALSE(o.enableMotors);
  TEST_ASSERT_TRUE(o.tvcActuators); // Disable does not touch the actuators
}

// ---- pre-spin timing ----
static const float kThrust = 14.0f, kMass = 1.1f, kStop = 20.0f; // vehicle constants of main.cpp
static const float kA = kThrust / kMass - 9.81f;

static FreefallSpinupInput inp(float alt, float vz, bool running = true) {
  FreefallSpinupInput i;
  i.running = running; i.altitudeM = alt; i.vzMps = vz; i.stopTargetM = kStop; i.thrustLimitN = kThrust;
  i.massKg = kMass;
  return i;
}

static void test_prespin_timing_in_ballistic_fall() {
  FreefallSpinup sp;
  float alt = 150.0f, vz = 0.0f;
  const float dt = 0.01f;
  float tEnable = -1, tStop = -1, t = 0;
  int enables = 0;
  bool prev = false;
  while (alt > 0) {
    const MotorsRequest r = sp.update(inp(alt, vz));
    const bool en = r == MotorsRequest::Enable;
    if (en && !prev) { enables++; tEnable = t; }
    if (prev) TEST_ASSERT_TRUE(en); // never off again
    prev = en;
    if (tStop < 0 && freefallStopTriggered(alt, vz, kStop, kThrust, kMass)) { tStop = t; break; }
    vz -= 9.81f * dt; alt += vz * dt; t += dt;
  }
  TEST_ASSERT_EQUAL(1, enables);
  TEST_ASSERT_TRUE(tStop > 0 && tEnable > 0);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, kMotorSpinupLeadS, tStop - tEnable);
  TEST_ASSERT_TRUE(tStop - tEnable >= kMotorSpinupLeadS - 0.1f);
}

static void test_never_before_the_lead() {
  // Far outside the lead (trigger altitude at 20 m/s down is about 88 m, 225 m two seconds later at 35 m/s).
  TEST_ASSERT_FALSE(freefallSpinupDue(inp(300.0f, -20.0f)));
  TEST_ASSERT_FALSE(freefallSpinupDue(inp(150.0f, 0.0f)));
  FreefallSpinup sp;
  TEST_ASSERT_EQUAL(MotorsRequest::Unchanged, sp.update(inp(300.0f, -20.0f)));
  TEST_ASSERT_EQUAL(MotorsRequest::Unchanged, sp.update(inp(150.0f, -5.0f, false))); // not running
}

static void test_latched_and_reset() {
  FreefallSpinup sp;
  TEST_ASSERT_EQUAL(MotorsRequest::Enable, sp.update(inp(30.0f, -25.0f)));
  TEST_ASSERT_EQUAL(MotorsRequest::Enable, sp.update(inp(300.0f, 0.0f))); // estimator jump: stays, not toggled
  TEST_ASSERT_EQUAL(MotorsRequest::Enable, sp.update(inp(30.0f, -25.0f, false))); // ended: holds
  sp.reset();
  TEST_ASSERT_EQUAL(MotorsRequest::Unchanged, sp.update(inp(300.0f, 0.0f)));
}

static void test_invalid_estimates_use_margin() {
  TEST_ASSERT_FALSE(freefallSpinupDue(inp(NAN, -20.0f)));              // no altitude: never
  TEST_ASSERT_FALSE(freefallSpinupDue(inp(kStop + kMotorSpinupMarginM + 1, NAN)));
  TEST_ASSERT_TRUE(freefallSpinupDue(inp(kStop + kMotorSpinupMarginM - 1, NAN)));
  FreefallSpinupInput weak = inp(kStop + 5, -5); weak.thrustLimitN = 5.0f; // T/m < g: no stop logic margin
  TEST_ASSERT_TRUE(freefallSpinupDue(weak));
  weak.altitudeM = kStop + kMotorSpinupMarginM + 1;
  TEST_ASSERT_FALSE(freefallSpinupDue(weak));
}

static void test_request_goes_off_on_abort_and_failure() {
  // The request is applied through the mapping; an abort or failure is shutdown / another mission.
  FreefallSpinup sp;
  MotorEnableInput in;
  in.freefallActive = true; in.motorsEnabledNow = true; in.shutdown = false;
  in.request = sp.update(inp(30.0f, -25.0f));
  TEST_ASSERT_TRUE(motorEnableFor(in).tvcActuators); // pre-spin active
  in.shutdown = true;                                 // failure
  TEST_ASSERT_FALSE(motorEnableFor(in).tvcActuators); // motors gated off
  in.shutdown = false; in.freefallActive = false;     // abort: RTH is active, its own request is Unchanged
  in.request = MotorsRequest::Unchanged; in.missionActuatorsEnabled = false;
  TEST_ASSERT_FALSE(motorEnableFor(in).tvcActuators);
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_unchanged_bit_identical_to_old);
  RUN_TEST(test_shutdown_ignores_every_request);
  RUN_TEST(test_request_semantics);
  RUN_TEST(test_prespin_timing_in_ballistic_fall);
  RUN_TEST(test_never_before_the_lead);
  RUN_TEST(test_latched_and_reset);
  RUN_TEST(test_invalid_estimates_use_margin);
  RUN_TEST(test_request_goes_off_on_abort_and_failure);
  return UNITY_END();
}
