// Host tests of src/calibration/calibration_validity.cpp: pio test -e native -f test_calibration_validity
// The "all three calibrations must be good to fly" rule (gyro per power cycle, accel/mag stored values and the live
// still-window check), the simulation exemption, the countdown and gyro-run planning, and the preconditions of the
// calibration commands. The bench values of docs/project/hardware-and-drivers.md 8a are used as the stored values.
#include <math.h>
#include <string.h>
#include <unity.h>

#include "calibration/calibration_validity.hpp"

using namespace VCTR::Calibration;

void setUp() {}
void tearDown() {}

static constexpr int64_t S = kSecondNs;

// Bench 2026-11-20: accel b = (0.42849, 0.55833, 2.14256), M = C T = diag(-0.9984, 1.003, -0.9673) with
// T = diag(-1, 1, -1) (so C = diag(0.9984, 1.003, 0.9673)).
static StoredCalibration benchAccel(double bz = 2.14256) {
  StoredCalibration s;
  s.keyPresent = true;
  s.offset[0] = 0.42849;
  s.offset[1] = 0.55833;
  s.offset[2] = bz;
  s.C[0] = 0.9984;
  s.C[4] = 1.003;
  s.C[8] = 0.9673;
  s.T[0] = -1;
  s.T[8] = -1;
  return s;
}

// Bench mag: b = (1.0491e-2, -1.467e-3, 8.154e-3) gauss, C = 0.1622 I, T = [[0,0,-1],[1,0,0],[0,-1,0]].
static StoredCalibration benchMag() {
  StoredCalibration s;
  s.keyPresent = true;
  s.offset[0] = 1.0491e-2;
  s.offset[1] = -1.467e-3;
  s.offset[2] = 8.154e-3;
  s.C[0] = s.C[4] = s.C[8] = 0.1622;
  const double T[9] = {0, 0, -1, 1, 0, 0, 0, -1, 0};
  for (int i = 0; i < 9; i++)
    s.T[i] = T[i];
  return s;
}

/// A raw accel vector that the stored calibration maps to norm n (any direction).
static void rawWithNorm(const StoredCalibration &s, double n, double raw[3]) {
  // cal = M (raw - b) is linear in (raw - b): scale a probe along x until the calibrated norm is n
  double probe[3] = {s.offset[0] + 1, s.offset[1], s.offset[2]};
  const double pn = calibratedNorm(s, probe);
  const double k = n / pn;
  for (int i = 0; i < 3; i++)
    raw[i] = s.offset[i] + k * (probe[i] - s.offset[i]);
}

static void test_simulation_exemption_constant() {
  TEST_ASSERT_TRUE(kCheckExemptInSimulation);
  TEST_ASSERT_TRUE(checkRequired(false));
  TEST_ASSERT_FALSE(checkRequired(true));
}

static void test_gyro_validity_per_power_cycle() {
  ValidityInput in;
  TEST_ASSERT_EQUAL_UINT8(uint8_t(Reason::NotCalibrated), uint8_t(calibrationsValid(in).gyro));
  in.gyro = GyroRunState::Ok;
  TEST_ASSERT_EQUAL_UINT8(uint8_t(Reason::Ok), uint8_t(calibrationsValid(in).gyro));
  in.gyro = GyroRunState::Failed;
  TEST_ASSERT_EQUAL_UINT8(uint8_t(Reason::RunFailed), uint8_t(calibrationsValid(in).gyro));
}

static void test_bench_accel_offset_out_of_range() {
  // The currently stored accel offset b_z = 2.14 is above the 1.5 m/s^2 limit: arming is refused until recalibrated.
  TEST_ASSERT_EQUAL_UINT8(uint8_t(Reason::OutOfRange), uint8_t(checkAccelParams(benchAccel())));
  TEST_ASSERT_EQUAL_UINT8(uint8_t(Reason::Ok), uint8_t(checkAccelParams(benchAccel(1.4))));
}

static void test_accel_params() {
  StoredCalibration s = benchAccel(1.0);
  TEST_ASSERT_EQUAL_UINT8(uint8_t(Reason::Ok), uint8_t(checkAccelParams(s)));
  s.keyPresent = false;
  TEST_ASSERT_EQUAL_UINT8(uint8_t(Reason::KeyMissing), uint8_t(checkAccelParams(s)));
  StoredCalibration d; // identity default
  d.keyPresent = true;
  TEST_ASSERT_EQUAL_UINT8(uint8_t(Reason::Default), uint8_t(checkAccelParams(d)));
  s = benchAccel(1.0);
  s.C[0] = 0.90; // scale 10 % off
  TEST_ASSERT_EQUAL_UINT8(uint8_t(Reason::OutOfRange), uint8_t(checkAccelParams(s)));
  s = benchAccel(1.0);
  s.C[1] = 0.05; // coupling
  TEST_ASSERT_EQUAL_UINT8(uint8_t(Reason::OutOfRange), uint8_t(checkAccelParams(s)));
  s = benchAccel(1.0);
  s.offset[0] = NAN;
  TEST_ASSERT_EQUAL_UINT8(uint8_t(Reason::OutOfRange), uint8_t(checkAccelParams(s)));
}

static void test_mag_params() {
  StoredCalibration s = benchMag();
  TEST_ASSERT_EQUAL_UINT8(uint8_t(Reason::Ok), uint8_t(checkMagParams(s)));
  s.keyPresent = false;
  TEST_ASSERT_EQUAL_UINT8(uint8_t(Reason::KeyMissing), uint8_t(checkMagParams(s)));
  StoredCalibration d;
  d.keyPresent = true;
  TEST_ASSERT_EQUAL_UINT8(uint8_t(Reason::Default), uint8_t(checkMagParams(d)));
  s = benchMag();
  s.C[0] = s.C[4] = s.C[8] = 1.0; // R = 0.05 < 0.1
  TEST_ASSERT_EQUAL_UINT8(uint8_t(Reason::OutOfRange), uint8_t(checkMagParams(s)));
  s = benchMag();
  s.offset[0] = 0.3; // |b| > 0.8 R (R = 0.308 -> 0.246)
  TEST_ASSERT_EQUAL_UINT8(uint8_t(Reason::OutOfRange), uint8_t(checkMagParams(s)));
  s = benchMag();
  s.C[4] = 0.10; // not a uniform scale
  TEST_ASSERT_EQUAL_UINT8(uint8_t(Reason::OutOfRange), uint8_t(checkMagParams(s)));
}

static void test_live_check_and_all_ok() {
  ValidityInput in;
  in.gyro = GyroRunState::Ok;
  in.accel = benchAccel(1.0);
  in.mag = benchMag();
  // no live data yet while the gyro run is Ok: not trusted
  Validity v = calibrationsValid(in);
  TEST_ASSERT_EQUAL_UINT8(uint8_t(Reason::NoLiveData), uint8_t(v.accel));
  TEST_ASSERT_EQUAL_UINT8(uint8_t(Reason::NoLiveData), uint8_t(v.mag));
  TEST_ASSERT_FALSE(v.allOk());

  in.live.haveAccel = in.live.haveMag = true;
  rawWithNorm(in.accel, 9.81, in.live.accelRaw);
  rawWithNorm(in.mag, 0.05, in.live.magRaw);
  v = calibrationsValid(in);
  TEST_ASSERT_EQUAL_UINT8(uint8_t(Reason::Ok), uint8_t(v.accel));
  TEST_ASSERT_EQUAL_UINT8(uint8_t(Reason::Ok), uint8_t(v.mag));
  TEST_ASSERT_TRUE(v.allOk());

  rawWithNorm(in.accel, 9.81 + 0.3, in.live.accelRaw); // the bench symptom: |a_cal| = 10.3 m/s^2
  v = calibrationsValid(in);
  TEST_ASSERT_EQUAL_UINT8(uint8_t(Reason::LiveCheck), uint8_t(v.accel));
  rawWithNorm(in.accel, 9.81 + 0.2, in.live.accelRaw);
  TEST_ASSERT_EQUAL_UINT8(uint8_t(Reason::Ok), uint8_t(calibrationsValid(in).accel));

  rawWithNorm(in.mag, 0.05 * 1.35, in.live.magRaw);
  TEST_ASSERT_EQUAL_UINT8(uint8_t(Reason::LiveCheck), uint8_t(calibrationsValid(in).mag));
  rawWithNorm(in.mag, 0.05 * 0.65, in.live.magRaw);
  TEST_ASSERT_EQUAL_UINT8(uint8_t(Reason::LiveCheck), uint8_t(calibrationsValid(in).mag));
  rawWithNorm(in.mag, 0.05 * 0.8, in.live.magRaw);
  TEST_ASSERT_EQUAL_UINT8(uint8_t(Reason::Ok), uint8_t(calibrationsValid(in).mag));

  // a failed gyro run blocks on its own
  in.gyro = GyroRunState::Failed;
  TEST_ASSERT_FALSE(calibrationsValid(in).allOk());
}

static void test_missing_gyro_run_does_not_report_live_noise() {
  ValidityInput in; // gyro NoRun
  in.accel = benchAccel(1.0);
  in.mag = benchMag();
  const Validity v = calibrationsValid(in);
  TEST_ASSERT_EQUAL_UINT8(uint8_t(Reason::NotCalibrated), uint8_t(v.gyro));
  TEST_ASSERT_EQUAL_UINT8(uint8_t(Reason::Ok), uint8_t(v.accel)); // live check follows with the run's data
  TEST_ASSERT_FALSE(v.allOk());
}

static void test_describe() {
  Validity v;
  char buf[160];
  const size_t n = describe(v, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_size_t(strlen(buf), n);
  TEST_ASSERT_NOT_NULL(strstr(buf, "gyro: not calibrated since power-up"));
  TEST_ASSERT_NOT_NULL(strstr(buf, "mag: not found in memory"));
  char tiny[8];
  describe(v, tiny, sizeof(tiny));
  TEST_ASSERT_EQUAL_size_t(7, strlen(tiny));
}

static void test_gyro_run_planning() {
  GyroRunPlan p = planGyroRun(100 * S, 110 * S); // boot / ZeroEstimators default 10 s
  TEST_ASSERT_TRUE(p.run);
  TEST_ASSERT_EQUAL_INT64(109 * S, p.deadlineNs); // window end - 1 s: the EKF still zeroes for 1 s
  p = planGyroRun(100 * S, 107 * S);
  TEST_ASSERT_TRUE(p.run);
  TEST_ASSERT_EQUAL_INT64(106 * S, p.deadlineNs);
  p = planGyroRun(100 * S, 107 * S - 1);
  TEST_ASSERT_FALSE(p.run);
  p = planGyroRun(100 * S, 101 * S);
  TEST_ASSERT_FALSE(p.run);
}

static void test_countdown_rule() {
  // Initialisation lasts -paramInt - 10 s: -18 gives 8 s (5 s window + 3 s slack).
  TEST_ASSERT_TRUE(countdownLongEnough(-18));
  TEST_ASSERT_TRUE(countdownLongEnough(-30));
  TEST_ASSERT_FALSE(countdownLongEnough(-17));
  TEST_ASSERT_FALSE(countdownLongEnough(-10));
  TEST_ASSERT_FALSE(countdownLongEnough(0));
  TEST_ASSERT_EQUAL_INT64(108 * S, countdownWindowEndNs(100 * S, -18));
  // the gyro run planned for the shortest accepted countdown fits: 8 s window, deadline after 7 s >= 5 s window
  const int64_t now = 50 * S;
  const GyroRunPlan p = planGyroRun(now, countdownWindowEndNs(now, kMinCountdownParamInt));
  TEST_ASSERT_TRUE(p.run);
  TEST_ASSERT_EQUAL_INT64(now + 7 * S, p.deadlineNs);
}

static void test_command_preconditions() {
  CommandContext c;
  c.initialised = true;
  c.modeAllowed = true;
  c.missionIdleOrFinished = true;
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CommandResult::Ok), uint8_t(checkCommand(c)));
  CommandContext x = c;
  x.simulation = true;
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CommandResult::Simulation), uint8_t(checkCommand(x)));
  x = c;
  x.initialised = false;
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CommandResult::NotInitialised), uint8_t(checkCommand(x)));
  x = c;
  x.armed = true;
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CommandResult::Armed), uint8_t(checkCommand(x)));
  x = c;
  x.motorsEnabled = true;
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CommandResult::MotorsEnabled), uint8_t(checkCommand(x)));
  x = c;
  x.modeAllowed = false;
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CommandResult::NotOnGround), uint8_t(checkCommand(x)));
  x = c;
  x.missionIdleOrFinished = false;
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CommandResult::NotOnGround), uint8_t(checkCommand(x)));
}

static void test_countdown_end_decision() {
  Validity good;
  good.gyro = good.accel = good.mag = Reason::Ok;
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CountdownAction::Continue), uint8_t(decideCountdownEnd(true, good, true)));
  Validity bad = good;
  bad.accel = Reason::OutOfRange;
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CountdownAction::Abort), uint8_t(decideCountdownEnd(true, bad, true)));
  bad = good;
  bad.gyro = Reason::RunFailed; // the run that closes the countdown failed
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CountdownAction::Abort), uint8_t(decideCountdownEnd(true, bad, true)));
  bad = good;
  bad.mag = Reason::LiveCheck;
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CountdownAction::Abort), uint8_t(decideCountdownEnd(true, bad, true)));
  // mission already reset / no longer in the countdown: nothing to abort
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CountdownAction::BadButNotInCountdown), uint8_t(decideCountdownEnd(true, bad, false)));
  // simulation exemption: never aborts
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CountdownAction::Continue), uint8_t(decideCountdownEnd(checkRequired(true), bad, true)));
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_simulation_exemption_constant);
  RUN_TEST(test_gyro_validity_per_power_cycle);
  RUN_TEST(test_bench_accel_offset_out_of_range);
  RUN_TEST(test_accel_params);
  RUN_TEST(test_mag_params);
  RUN_TEST(test_live_check_and_all_ok);
  RUN_TEST(test_missing_gyro_run_does_not_report_live_noise);
  RUN_TEST(test_describe);
  RUN_TEST(test_gyro_run_planning);
  RUN_TEST(test_countdown_rule);
  RUN_TEST(test_command_preconditions);
  RUN_TEST(test_countdown_end_decision);
  return UNITY_END();
}
