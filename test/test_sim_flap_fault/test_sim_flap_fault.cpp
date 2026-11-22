// Host tests of src/safety/sim_flap_fault.cpp: pio test -e native -f test_sim_flap_fault
//
// Simulation-only flap servo faults (owner-approved 2026-11-21): the pure decision, the stuck / slow servo model,
// capture of the current angle, immediate effect, clear, the removed arm-for-descent bit, the one application point
// (TrueFlapModel::publishAngles, the angles StarshipFlaps publishes) and every consumer of it (telemetry ACTUATORS and
// AIR_ACTUATORS, body simulator flap input), and the guarantee that outside simulation nothing changes.
#include <math.h>
#include <string.h>
#include <unity.h>

#include "ExVectrDSP/body_sim_model.hpp"
#include "comms/telemetry_mapping.hpp"
#include "safety/sim_flap_fault.hpp"

using namespace VCTR::Safety;

void setUp() {}
void tearDown() {}

static constexpr float kDeg = 3.14159265f / 180.0f;
static constexpr float kRate = 60 * kDeg / 0.2f; // StarshipFlaps modelled servo rate

// The real StarshipFlaps true-angle stage (Safety::TrueFlapModel), driven as StarshipFlaps drives it, one flap
// exposed per adapter: powered tick = setTargets + advance, then publishAngles (every tick).
struct Servo {
  TrueFlapModel m{kRate};
  float cur = 0, target = 0;
  float out[4] = {0, 0, 0, 0};
  int idx = 0;
  void place(float a) { // start the servo at angle a (target a, settled)
    target = a;
    float t[4] = {a, a, a, a};
    m.setTargets(t);
    SimFlapFaults none;
    m.advance(10000000000LL, none, false);
    cur = a;
  }
  void step(SimFlapFaults &f, uint8_t i, bool sim, int64_t dtNs, bool powered = true) {
    idx = i;
    if (powered) {
      float t[4];
      for (int k = 0; k < 4; k++)
        t[k] = m.target(k);
      t[i] = target;
      m.setTargets(t);
      m.advance(dtNs, f, sim);
    }
    m.publishAngles(f, sim, out);
    cur = out[i];
  }
};

static void test_decision_refusals() {
  // not simulation: refused whatever the rest
  SimFaultDecision d = decideSimFault(16, 0, 0x57, 0.7f, false);
  TEST_ASSERT_EQUAL(int(SimFaultAction::Refuse), int(d.action));
  TEST_ASSERT_EQUAL(int(SimFaultRefusal::NotSimulation), int(d.reason));
  TEST_ASSERT_EQUAL(int(SimFaultAction::Refuse), int(decideSimFault(17, 0, 0x57, 0.5f, false).action));
  TEST_ASSERT_EQUAL(int(SimFaultAction::Refuse), int(decideSimFault(18, 0, 0x57, 0, false).action));
  // magic
  TEST_ASSERT_EQUAL(int(SimFaultRefusal::BadMagic), int(decideSimFault(16, 0, 0x56, 0.7f, true).reason));
  TEST_ASSERT_EQUAL(int(SimFaultRefusal::BadMagic), int(decideSimFault(18, 0, 0, 0, true).reason));
  // non finite
  TEST_ASSERT_EQUAL(int(SimFaultRefusal::NotFinite), int(decideSimFault(16, 0, 0x57, NAN, true).reason));
  TEST_ASSERT_EQUAL(int(SimFaultRefusal::NotFinite), int(decideSimFault(17, 1, 0x57, INFINITY, true).reason));
  // clear ignores the float
  TEST_ASSERT_EQUAL(int(SimFaultAction::Clear), int(decideSimFault(18, 0xFF, 0x57, NAN, true).action));
  // subsystem bits: only the flap index; the removed "arm for the descent" bit 0x10 is refused (redesign 2026-11-21)
  TEST_ASSERT_EQUAL(int(SimFaultRefusal::BadSubsystem), int(decideSimFault(16, 0x10, 0x57, 0.7f, true).reason));
  TEST_ASSERT_EQUAL(int(SimFaultRefusal::BadSubsystem), int(decideSimFault(16, 0x13, 0x57, 0.7f, true).reason));
  TEST_ASSERT_EQUAL(int(SimFaultRefusal::BadSubsystem), int(decideSimFault(16, 0x04, 0x57, 0.7f, true).reason));
  TEST_ASSERT_EQUAL(int(SimFaultRefusal::BadSubsystem), int(decideSimFault(16, 0x20, 0x57, 0.7f, true).reason));
  TEST_ASSERT_EQUAL(int(SimFaultRefusal::BadSubsystem), int(decideSimFault(17, 0x10, 0x57, 0.5f, true).reason));
  // wrong type
  TEST_ASSERT_EQUAL(int(SimFaultRefusal::BadType), int(decideSimFault(15, 0, 0x57, 1, true).reason));
  TEST_ASSERT_EQUAL(int(SimFaultRefusal::BadType), int(decideSimFault(19, 0, 0x57, 1, true).reason));
}

static void test_decision_values() {
  SimFaultDecision d = decideSimFault(16, 0x00, 0x57, 42 * kDeg, true);
  TEST_ASSERT_EQUAL(int(SimFaultAction::Stuck), int(d.action));
  TEST_ASSERT_EQUAL(0, d.flap);
  TEST_ASSERT_FALSE(d.captureCurrent);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 42 * kDeg, d.value);
  d = decideSimFault(16, 0x03, 0x57, 5.0f, true); // flap 3, clamped to pi/2
  TEST_ASSERT_EQUAL(3, d.flap);
  TEST_ASSERT_EQUAL_FLOAT(kSimFlapMaxRad, d.value);
  d = decideSimFault(16, 0x01, 0x57, -0.3f, true); // negative (not -1) clamps to 0
  TEST_ASSERT_FALSE(d.captureCurrent);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, d.value);
  d = decideSimFault(16, 0x02, 0x57, -1.0f, true);
  TEST_ASSERT_TRUE(d.captureCurrent);
  TEST_ASSERT_EQUAL(2, d.flap);
  d = decideSimFault(17, 0x03, 0x57, 0.01f, true);
  TEST_ASSERT_EQUAL(int(SimFaultAction::Slow), int(d.action));
  TEST_ASSERT_EQUAL_FLOAT(0.05f, d.value);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, decideSimFault(17, 0, 0x57, 7.0f, true).value);
  TEST_ASSERT_EQUAL_FLOAT(0.4f, decideSimFault(17, 0, 0x57, 0.4f, true).value);
}

static void test_stuck_holds_angle() {
  SimFlapFaults f;
  TEST_ASSERT_TRUE(f.set(decideSimFault(16, 0, 0x57, 42 * kDeg, true), true));
  Servo s[4];
  for (int k = 0; k < 4; k++)
    s[k].target = 60 * kDeg;
  s[0].target = 20 * kDeg;
  for (int n = 0; n < 100; n++)
    for (uint8_t i = 0; i < 4; i++)
      s[i].step(f, i, true, 20000000);
  TEST_ASSERT_EQUAL_FLOAT(42 * kDeg, s[0].cur); // holds although commanded 20 deg
  s[0].target = 80 * kDeg;
  for (int n = 0; n < 50; n++)
    s[0].step(f, 0, true, 20000000);
  TEST_ASSERT_EQUAL_FLOAT(42 * kDeg, s[0].cur);
  for (uint8_t i = 1; i < 4; i++)
    TEST_ASSERT_EQUAL_FLOAT(60 * kDeg, s[i].cur); // others normal
}

static void test_capture_current_angle() {
  SimFlapFaults f;
  Servo s;
  s.target = 50 * kDeg;
  for (int n = 0; n < 3; n++) // 3 * 20 ms * 300 deg/s = 18 deg
    s.step(f, 1, true, 20000000);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 18 * kDeg, s.cur);
  TEST_ASSERT_TRUE(f.set(decideSimFault(16, 1, 0x57, -1.0f, true), true));
  s.target = 90 * kDeg;
  s.step(f, 1, true, 20000000); // captures the angle at the start of the update result (moved once more)
  const float held = s.cur;
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 24 * kDeg, held);
  for (int n = 0; n < 50; n++)
    s.step(f, 1, true, 20000000);
  TEST_ASSERT_EQUAL_FLOAT(held, s.cur);
}

static void test_slow_rate() {
  SimFlapFaults f;
  Servo a, b;
  a.target = b.target = 90 * kDeg;
  TEST_ASSERT_TRUE(f.set(decideSimFault(17, 2, 0x57, 0.25f, true), true));
  TEST_ASSERT_EQUAL_FLOAT(0.25f, f.rateFactor(2, true));
  TEST_ASSERT_EQUAL_FLOAT(1.0f, f.rateFactor(0, true));
  for (int n = 0; n < 5; n++) { // 100 ms
    a.step(f, 2, true, 20000000); // slow
    b.step(f, 0, true, 20000000); // normal
  }
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.25f * 30 * kDeg, a.cur);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 30 * kDeg, b.cur);
}

static void test_clear_and_mode_switch() {
  SimFlapFaults f;
  f.set(decideSimFault(16, 0, 0x57, 0.7f, true), true);
  f.set(decideSimFault(16, 0x01, 0x57, 0.7f, true), true);
  f.set(decideSimFault(17, 3, 0x57, 0.2f, true), true);
  TEST_ASSERT_TRUE(f.stuck(0) && f.stuck(1) && f.any());
  TEST_ASSERT_TRUE(f.set(decideSimFault(18, 0, 0x57, 0, true), true));
  TEST_ASSERT_FALSE(f.any());
  TEST_ASSERT_EQUAL_FLOAT(1.0f, f.rateFactor(3, true));
  // the simulation mode switch calls clear()
  f.set(decideSimFault(16, 0, 0x57, 0.7f, true), true);
  f.set(decideSimFault(16, 0x02, 0x57, 0.7f, true), true);
  f.clear();
  TEST_ASSERT_FALSE(f.any());
  TEST_ASSERT_EQUAL_FLOAT(0.123f, f.effectiveAngle(0, true, 0.123f));
}

// Review 2026-11-21: finding 10 (SystemReset clears all faults, so nothing carries into the next run) and
// finding 11 (the servo model drops its forced angle when a stuck fault is released).
static void test_system_reset_clears_faults_and_servo_snaps_back() {
  SimFlapFaults f;
  Servo s;
  s.place(90 * kDeg);
  f.set(decideSimFault(16, 0, 0x57, 40 * kDeg, true), true); // TL stuck at 40 deg
  f.set(decideSimFault(17, 2, 0x57, 0.3f, true), true);        // BL slow
  s.step(f, 0, true, 2000000);
  TEST_ASSERT_EQUAL_FLOAT(40 * kDeg, s.cur);
  // SystemReset: the vehicle calls clear()
  f.clear();
  TEST_ASSERT_FALSE(f.any());
  TEST_ASSERT_FALSE(f.stuck(0) || f.stuck(1));
  TEST_ASSERT_EQUAL_FLOAT(1.0f, f.rateFactor(2, true));
  // finding 11: the next update snaps the model angle to the commanded target instead of slewing back for 0.3 s
  s.step(f, 0, true, 2000000);
  TEST_ASSERT_EQUAL_FLOAT(90 * kDeg, s.cur);
  // released once only, and only for the flap that was stuck
  TEST_ASSERT_FALSE(f.takeReleased(0));
  TEST_ASSERT_FALSE(f.takeReleased(1));
  TEST_ASSERT_FALSE(f.takeReleased(2));
  // a release by leaving simulation mode (clear while sim is already false) still snaps back
  SimFlapFaults g;
  Servo t;
  t.place(60 * kDeg);
  g.set(decideSimFault(16, 1, 0x57, 20 * kDeg, true), true);
  t.step(g, 1, true, 2000000);
  TEST_ASSERT_EQUAL_FLOAT(20 * kDeg, t.cur);
  g.clear();
  t.step(g, 1, false, 2000000);
  TEST_ASSERT_EQUAL_FLOAT(60 * kDeg, t.cur);
}

static void test_non_simulation_is_inert_and_bit_identical() {
  // Reference run without any fault object activity.
  SimFlapFaults clean;
  Servo ref[4];
  // Run A: refused commands in real mode (never stored). Run B: a fault state that exists (set while simulating),
  // then evaluated with the simulation flag false. Both must equal the clean run bit for bit.
  SimFlapFaults refused, leaked;
  TEST_ASSERT_FALSE(refused.set(decideSimFault(16, 0, 0x57, 0.7f, false), false));
  TEST_ASSERT_FALSE(refused.set(decideSimFault(17, 1, 0x57, 0.1f, false), false));
  TEST_ASSERT_FALSE(refused.any());
  // Even a handed-in "accepted" decision is rejected without the simulation flag.
  TEST_ASSERT_FALSE(refused.set(decideSimFault(16, 0, 0x57, 0.7f, true), false));
  TEST_ASSERT_TRUE(leaked.set(decideSimFault(16, 0, 0x57, 0.7f, true), true));
  TEST_ASSERT_TRUE(leaked.set(decideSimFault(16, 1, 0x57, -1.0f, true), true));
  TEST_ASSERT_TRUE(leaked.set(decideSimFault(17, 2, 0x57, 0.05f, true), true));
  Servo a[4], b[4];
  float t = 0.3f;
  for (int n = 0; n < 400; n++) {
    t = 0.2f + 1.2f * (0.5f + 0.5f * sinf(0.07f * float(n)));
    for (uint8_t i = 0; i < 4; i++) {
      ref[i].target = a[i].target = b[i].target = t + 0.1f * i;
      ref[i].step(clean, i, false, 20000000);
      a[i].step(refused, i, false, 20000000);
      b[i].step(leaked, i, false, 20000000);
      TEST_ASSERT_EQUAL_MEMORY(&ref[i].cur, &a[i].cur, sizeof(float));
      TEST_ASSERT_EQUAL_MEMORY(&ref[i].cur, &b[i].cur, sizeof(float));
    }
  }
  TEST_ASSERT_EQUAL_FLOAT(1.0f, leaked.rateFactor(2, false));
}

// Redesign 2026-11-21 (owner: 'Apply stuck fault' did nothing). Root cause on the bench: StarshipFlaps published the
// true angle only while the flaps were enabled, so on the ground (flaps off) the fault was accepted and logged but never
// reached the topic. Now the forced angle is published at the next tick, powered or not, and moves nothing else.
static void test_immediate_effect_powered_and_unpowered() {
  for (int powered = 0; powered < 2; powered++) {
    SimFlapFaults f;
    Servo s;
    s.place(50 * kDeg);
    s.step(f, 0, true, 20000000, powered);
    TEST_ASSERT_EQUAL_FLOAT(50 * kDeg, s.cur);
    TEST_ASSERT_TRUE(f.set(decideSimFault(16, 0, 0x57, 30 * kDeg, true), true));
    s.step(f, 0, true, 20000000, powered); // first tick after the command
    TEST_ASSERT_EQUAL_FLOAT(30 * kDeg, s.cur);
    TEST_ASSERT_EQUAL_FLOAT(30 * kDeg, s.out[0]);
    for (int k = 1; k < 4; k++)
      TEST_ASSERT_EQUAL_FLOAT(50 * kDeg, s.out[k]); // the other flaps are untouched
    // hold-current (-1) on an unpowered/powered flap: holds the angle it had
    TEST_ASSERT_TRUE(f.set(decideSimFault(16, 2, 0x57, -1.0f, true), true));
    s.step(f, 2, true, 20000000, powered);
    TEST_ASSERT_EQUAL_FLOAT(50 * kDeg, s.out[2]);
  }
}

// Every consumer of the flap angle reads the published true angle: the telemetry snapshot (ACTUATORS.flaps and the
// AIR_ACTUATORS samples, the ground station's FlapState is decoded from these) and the body simulator's flap input.
// The forced value must appear in all of them; the flap controller itself reads no flap angle.
static void test_forced_angle_reaches_every_consumer() {
  SimFlapFaults f;
  Servo s;
  s.place(50 * kDeg);
  TEST_ASSERT_TRUE(f.set(decideSimFault(16, 1, 0x57, 20 * kDeg, true), true)); // TR stuck at 20 deg
  for (int n = 0; n < 5; n++) {
    s.target = 80 * kDeg; // the controller commands something else
    s.step(f, 1, true, 20000000);
  }
  // (1) the published angles (true flap topic, ControlAttitudeFlapSetting [TL, TR, BL, BR])
  TEST_ASSERT_EQUAL_FLOAT(20 * kDeg, s.out[1]);
  // (2) telemetry: TelemetryProvider copies the topic into TelemetrySnapshot.flaps
  VCTR::Comms::TelemetrySnapshot snap;
  for (int k = 0; k < 4; k++)
    snap.flaps[k] = s.out[k];
  const auto act = VCTR::Comms::makeActuators(snap);
  TEST_ASSERT_EQUAL_FLOAT(20 * kDeg, act.flaps[1]);
  TEST_ASSERT_EQUAL_FLOAT(50 * kDeg, act.flaps[0]);
  VCTR::Comms::ActuatorHistory hist;
  hist.push(VCTR::Comms::makeActuatorSample(snap));
  hist.push(VCTR::Comms::makeActuatorSample(snap));
  const auto air = VCTR::Comms::makeAirActuators(hist);
  TEST_ASSERT_FLOAT_WITHIN(0.002f, 20 * kDeg, air.flaps[1]); // i16 on the wire, 4 per sample
  TEST_ASSERT_FLOAT_WITHIN(0.002f, 20 * kDeg, air.flaps[4 + 1]);
  // (3) body simulator: BodySimulator passes the topic to BodySimModel::setFlapInput; the modelled flap reaches it.
  VCTR::DSP::BodySimModel model{VCTR::DSP::BodySimParams()};
  model.setFlapInput(s.out[0], s.out[1], s.out[2], s.out[3]);
  for (int n = 0; n < 200; n++)
    model.step();
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 20 * kDeg, model.flapAngle(1));
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 50 * kDeg, model.flapAngle(0));
}


// ---- Servo PWM command stage (owner decision 2026-11-22) ----
static constexpr float kMin = 15 * kDeg, kMax = 90 * kDeg;
static constexpr int64_t kTick = 20000000LL;

static float pwmOf(ServoOutputStage &st, SimFlapFaults &f, uint8_t i, float cmd, bool sim) {
  return st.servoCommandRad(f, i, cmd, sim, kTick, kRate, kMin, kMax);
}

static void test_pwm_real_mode_bit_identical() {
  ServoOutputStage a, b;
  SimFlapFaults none, faulty;
  // an (impossible) fault state outside simulation: set() refuses, so force it through a simulation window and
  // leave the flag false at the call
  faulty.set(decideSimFault(16, 1, 0x57, 0.7f, true), true);
  faulty.set(decideSimFault(17, 2, 0x57, 0.1f, true), true);
  for (int k = 0; k < 200; k++) {
    const float c = kMin + (kMax - kMin) * (0.5f + 0.5f * sinf(0.13f * k));
    for (uint8_t i = 0; i < 4; i++) {
      float x = pwmOf(a, none, i, c, false), y = pwmOf(b, faulty, i, c, false);
      TEST_ASSERT_EQUAL_MEMORY(&c, &x, sizeof c);
      TEST_ASSERT_EQUAL_MEMORY(&x, &y, sizeof x);
    }
  }
}

static void test_pwm_stuck_equals_forced_and_clamped() {
  ServoOutputStage st;
  SimFlapFaults f;
  f.set(decideSimFault(16, 0, 0x57, 0.7f, true), true);
  for (float c : {kMin, 0.5f, kMax, 1.2f})
    TEST_ASSERT_EQUAL_FLOAT(0.7f, pwmOf(st, f, 0, c, true));
  // other flaps unaffected
  TEST_ASSERT_EQUAL_FLOAT(0.9f, pwmOf(st, f, 1, 0.9f, true));
  // forced angle outside the servo range (0.1 rad < 15 deg) is clamped
  SimFlapFaults g;
  g.set(decideSimFault(16, 2, 0x57, 0.1f, true), true);
  TEST_ASSERT_EQUAL_FLOAT(kMin, pwmOf(st, g, 2, 1.0f, true));
  SimFlapFaults h;
  h.set(decideSimFault(16, 3, 0x57, 5.0f, true), true);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, kMax, pwmOf(st, h, 3, 0.4f, true));
}

static void test_pwm_slow_limits_rate_release_slews_back() {
  ServoOutputStage st;
  SimFlapFaults f;
  float prev = pwmOf(st, f, 0, 0.4f, true);
  TEST_ASSERT_EQUAL_FLOAT(0.4f, prev);
  f.set(decideSimFault(17, 0, 0x57, 0.25f, true), true);
  const float maxStep = kRate * 0.25f * 0.02f;
  for (int k = 0; k < 10; k++) {
    float o = pwmOf(st, f, 0, 1.4f, true);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, maxStep, o - prev);
    prev = o;
  }
  // release: no jump, slews back at the NORMAL rate, then exactly the command
  f.clear();
  const float normStep = kRate * 0.02f;
  float o = pwmOf(st, f, 0, 1.4f, true);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, normStep, o - prev);
  for (int k = 0; k < 20; k++)
    o = pwmOf(st, f, 0, 1.4f, true);
  TEST_ASSERT_EQUAL_FLOAT(1.4f, o);
}

static void test_pwm_stuck_release_no_jump() {
  ServoOutputStage st;
  SimFlapFaults f;
  f.set(decideSimFault(16, 0, 0x57, 0.5f, true), true);
  TEST_ASSERT_EQUAL_FLOAT(0.5f, pwmOf(st, f, 0, 1.4f, true));
  f.clear();
  const float o = pwmOf(st, f, 0, 1.4f, true);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.5f + kRate * 0.02f, o);
  float p = o;
  for (int k = 0; k < 30; k++)
    p = pwmOf(st, f, 0, 1.4f, true);
  TEST_ASSERT_EQUAL_FLOAT(1.4f, p);
}

static void test_pwm_sim_off_returns_to_command() {
  ServoOutputStage st;
  SimFlapFaults f;
  f.set(decideSimFault(16, 0, 0x57, 0.5f, true), true);
  TEST_ASSERT_EQUAL_FLOAT(0.5f, pwmOf(st, f, 0, 1.2f, true));
  TEST_ASSERT_EQUAL_FLOAT(1.2f, pwmOf(st, f, 0, 1.2f, false)); // flag read live, fault state still present
  TEST_ASSERT_EQUAL_FLOAT(0.5f, pwmOf(st, f, 0, 1.2f, true));  // flag back on, state still there: forced again
}

static void test_pwm_hold_current_angle() {
  ServoOutputStage st;
  Servo s;
  SimFlapFaults f;
  s.place(0.6f);
  f.set(decideSimFault(16, 0, 0x57, -1.0f, true), true);
  TEST_ASSERT_TRUE(f.capturePending(0));
  s.step(f, 0, true, kTick); // resolves the capture at the true angle 0.6
  TEST_ASSERT_FALSE(f.capturePending(0));
  TEST_ASSERT_EQUAL_FLOAT(0.6f, pwmOf(st, f, 0, 1.3f, true));
  TEST_ASSERT_EQUAL_FLOAT(0.6f, pwmOf(st, f, 0, 0.9f, true));
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_decision_refusals);
  RUN_TEST(test_decision_values);
  RUN_TEST(test_stuck_holds_angle);
  RUN_TEST(test_capture_current_angle);
  RUN_TEST(test_slow_rate);
  RUN_TEST(test_clear_and_mode_switch);
  RUN_TEST(test_system_reset_clears_faults_and_servo_snaps_back);
  RUN_TEST(test_non_simulation_is_inert_and_bit_identical);
  RUN_TEST(test_immediate_effect_powered_and_unpowered);
  RUN_TEST(test_forced_angle_reaches_every_consumer);
  RUN_TEST(test_pwm_real_mode_bit_identical);
  RUN_TEST(test_pwm_stuck_equals_forced_and_clamped);
  RUN_TEST(test_pwm_slow_limits_rate_release_slews_back);
  RUN_TEST(test_pwm_stuck_release_no_jump);
  RUN_TEST(test_pwm_sim_off_returns_to_command);
  RUN_TEST(test_pwm_hold_current_angle);
  return UNITY_END();
}
