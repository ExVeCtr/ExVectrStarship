// Host tests of src/safety/motor_enable.cpp: pio test -e native -f test_motor_enable
//
// (1) The mapping with request Unchanged is bit-identical to the old vehicleShutdownControl behaviour for every input
// combination (the reference below is the old code, copied before the change). (2) Request semantics. (3) The
// freefall pre-spin: Enable once on entry to the tilt-back stage, never before, never twice, off on shutdown, no thrust command.
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

// ---- pre-spin on entry to the tilt-back stage ----
static const float kStopTrigger = 46.0f; // example: stopDist 6 m (6 m/s belly-flop fall) + stop target 20 m

static void test_enable_once_on_tilt_back_entry_never_before() {
  FreefallSpinup sp;
  // belly-flop fall at terminal speed from 150 m: the trigger altitude is constant, the stage starts 20 m above it
  const float entry = kStopTrigger + kFreefallTiltBackMarginM;
  int enables = 0;
  bool prev = false;
  for (float alt = 150.0f; alt > 5.0f; alt -= 0.06f) { // 6 m/s at 100 Hz
    const bool en = sp.update(true, alt, kStopTrigger) == MotorsRequest::Enable;
    if (alt >= entry) TEST_ASSERT_FALSE(en); // never earlier in the fall
    if (en && !prev) enables++;
    if (prev) TEST_ASSERT_TRUE(en);
    prev = en;
  }
  TEST_ASSERT_EQUAL(1, enables);
  TEST_ASSERT_TRUE(prev);
}

static void test_tilt_back_stage_condition() {
  TEST_ASSERT_FALSE(freefallTiltBackStage(150.0f, kStopTrigger));
  TEST_ASSERT_FALSE(freefallTiltBackStage(kStopTrigger + kFreefallTiltBackMarginM, kStopTrigger));
  TEST_ASSERT_TRUE(freefallTiltBackStage(kStopTrigger + kFreefallTiltBackMarginM - 0.1f, kStopTrigger));
  TEST_ASSERT_FALSE(freefallTiltBackStage(NAN, kStopTrigger)); // no altitude: never
  TEST_ASSERT_FALSE(freefallTiltBackStage(30.0f, NAN));
  FreefallSpinup sp;
  TEST_ASSERT_EQUAL(MotorsRequest::Unchanged, sp.update(false, 10.0f, kStopTrigger)); // not running
}

static void test_latched_and_reset() {
  FreefallSpinup sp;
  TEST_ASSERT_EQUAL(MotorsRequest::Enable, sp.update(true, 30.0f, kStopTrigger));
  TEST_ASSERT_EQUAL(MotorsRequest::Enable, sp.update(true, 300.0f, kStopTrigger)); // estimator jump: stays
  TEST_ASSERT_EQUAL(MotorsRequest::Enable, sp.update(false, 30.0f, kStopTrigger)); // ended: holds
  sp.reset();
  TEST_ASSERT_EQUAL(MotorsRequest::Unchanged, sp.update(true, 300.0f, kStopTrigger));
}

static void test_sim_thrust_zero_when_gate_closed() {
  const float in[4] = {0.5f, -0.3f, 9.0f, 0.02f};
  float out[4];
  gateSimulatedThrust(in, false, out);
  for (int i = 0; i < 4; i++) TEST_ASSERT_EQUAL_FLOAT(0.0f, out[i]);
  gateSimulatedThrust(in, true, out);
  for (int i = 0; i < 4; i++) TEST_ASSERT_EQUAL_FLOAT(in[i], out[i]); // open: bit-identical
}

static void test_request_goes_off_on_abort_and_failure() {
  // The request is applied through the mapping; an abort or failure is shutdown / another mission.
  FreefallSpinup sp;
  MotorEnableInput in;
  in.freefallActive = true; in.motorsEnabledNow = true; in.shutdown = false;
  in.request = sp.update(true, 30.0f, kStopTrigger);
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
  RUN_TEST(test_enable_once_on_tilt_back_entry_never_before);
  RUN_TEST(test_tilt_back_stage_condition);
  RUN_TEST(test_latched_and_reset);
  RUN_TEST(test_sim_thrust_zero_when_gate_closed);
  RUN_TEST(test_request_goes_off_on_abort_and_failure);
  return UNITY_END();
}
