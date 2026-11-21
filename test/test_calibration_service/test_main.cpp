// Host tests of src/calibration/calibration_service.cpp: pio test -e native -f test_calibration_service
// The calibration session engine with synthetic raw sensor streams: the automatic gyro run (bias recovered, motion
// gives up, cancel), the 12-pose accel session (automatic capture, fit, legacy-key conversion M = C T = T A), the mag
// tumble session (centre recovered), refusal while busy, discard, applied/saved states, failed fits, and the verify
// mode against an active calibration.
#include <math.h>
#include <string.h>
#include <unity.h>

#include "../cal_test_support.hpp"
#include "calibration/calibration_math.hpp"
#include "calibration/calibration_service.hpp"

using namespace VCTR::Calibration;
using namespace CalTest;
using VCTR::DSP::Calib::CalResult;

void setUp() {}
void tearDown() {}

static constexpr int64_t MS = 1000000LL;
static constexpr int64_t S = 1000000000LL;

static float g_magBuf[CalibrationService::kMagBufferCapacity][3];

struct Log {
  char lines[64][160];
  int n = 0;
  static void sink(void *ctx, const char *line) {
    Log *l = static_cast<Log *>(ctx);
    if (l->n < 64) {
      strncpy(l->lines[l->n], line, 159);
      l->lines[l->n][159] = 0;
      l->n++;
    }
  }
  bool has(const char *needle) const {
    for (int i = 0; i < n; i++)
      if (strstr(lines[i], needle))
        return true;
    return false;
  }
};

/// Simulated ship on the bench: raw gyro/accel/mag at 1 kHz with a clock.
struct Sim {
  Rng rng{7};
  int64_t t = 0;
  double gyroBias[3] = {0.016, 0.035, -0.010};
  double gyroNoise = 0.0015, accNoise = 0.02;
  // accel truth: raw = Ainv (g u) + b
  double b[3] = {0.35, -0.42, 1.10};
  double A[9] = {1.012, 0.006, -0.004, 0.006, 0.991, 0.008, -0.004, 0.008, 1.018};
  double Ainv[9];
  double u[3] = {0, 0, 1};
  double rate = 0;   // handling motion (rad/s about x, sign flips every 0.1 s)
  double shake = 0;  // extra accel noise
  int n = 0;
  Sim() { mat3Inverse(A, Ainv); }

  void accelRaw(double a[3]) {
    const double v[3] = {9.81 * u[0], 9.81 * u[1], 9.81 * u[2]};
    rawFromTrue(Ainv, b, v, a);
    for (int k = 0; k < 3; k++)
      a[k] += (accNoise + shake) * rng.normal();
  }
  void step(CalibrationService &svc, bool gyroAccel = true) {
    if (gyroAccel) {
      double w[3], a[3];
      const double sgn = ((n++ / 100) % 2) ? -1.0 : 1.0;
      for (int k = 0; k < 3; k++)
        w[k] = gyroBias[k] + gyroNoise * rng.normal() + (k == 0 ? sgn * rate : 0);
      accelRaw(a);
      svc.addGyro(t, w);
      svc.addAccel(t, a);
    }
    t += MS;
  }
  void run(CalibrationService &svc, double seconds) {
    const int steps = int(seconds * 1000);
    for (int i = 0; i < steps; i++) {
      step(svc);
      if (i % 100 == 99)
        svc.tick(t);
    }
  }
};

static void test_gyro_auto_run_recovers_bias() {
  CalibrationService svc(g_magBuf, CalibrationService::kMagBufferCapacity);
  Sim sim;
  TEST_ASSERT_TRUE(svc.startGyroRun(sim.t, sim.t + 9 * S));
  TEST_ASSERT_TRUE(svc.gyroRunActive());
  TEST_ASSERT_TRUE(svc.wantsGyro());
  sim.run(svc, 7.0);
  GyroRunDone done;
  TEST_ASSERT_TRUE(svc.takeGyroRunDone(done));
  TEST_ASSERT_FALSE(svc.takeGyroRunDone(done)); // once
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::Pass), uint8_t(done.result.result));
  for (int k = 0; k < 3; k++)
    TEST_ASSERT_DOUBLE_WITHIN(0.0005, sim.gyroBias[k], done.result.bias[k]);
  // the still window also supplied the mean raw accel for the live check (|raw| about 9.81 / A, a few percent)
  TEST_ASSERT_TRUE(done.result.nAccel > 1000);
  TEST_ASSERT_FALSE(svc.gyroRunActive());
  TEST_ASSERT_FALSE(svc.wantsGyro());
}

static void test_gyro_auto_run_gives_up_with_motion_and_cancel() {
  CalibrationService svc(g_magBuf, CalibrationService::kMagBufferCapacity);
  Sim sim;
  sim.rate = 0.05; // 3 deg/s handling: every block is rejected
  TEST_ASSERT_TRUE(svc.startGyroRun(sim.t, sim.t + 6 * S));
  sim.run(svc, 7.0);
  GyroRunDone done;
  TEST_ASSERT_TRUE(svc.takeGyroRunDone(done));
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::Motion), uint8_t(done.result.result));

  Sim sim2;
  TEST_ASSERT_TRUE(svc.startGyroRun(sim2.t, sim2.t + 9 * S));
  sim2.run(svc, 2.0);
  svc.cancelGyroRun();
  TEST_ASSERT_FALSE(svc.gyroRunActive());
  sim2.run(svc, 9.0);
  TEST_ASSERT_FALSE(svc.takeGyroRunDone(done)); // a cancelled run reports nothing
}

/// Moves the ship to direction u: a short shaky handling phase, then still for holdS seconds.
static void movePose(CalibrationService &svc, Sim &sim, const double u[3], double holdS) {
  sim.rate = 0.4;
  sim.shake = 1.0;
  sim.run(svc, 0.6);
  sim.rate = 0;
  sim.shake = 0;
  for (int k = 0; k < 3; k++)
    sim.u[k] = u[k];
  sim.run(svc, holdS);
}

static void captureTwelvePoses(CalibrationService &svc, Sim &sim, int count) {
  double u[12][3];
  Rng pr(5);
  twelvePoses(u, pr, 6.0);
  for (int i = 0; i < count; i++)
    movePose(svc, sim, u[i], 4.0);
}

static void test_accel_session_twelve_poses_fit_and_legacy_conversion() {
  CalibrationService svc(g_magBuf, CalibrationService::kMagBufferCapacity);
  Log log;
  svc.setLogSink(Log::sink, &log);
  Sim sim;
  sim.gyroBias[0] = sim.gyroBias[1] = sim.gyroBias[2] = 0; // active gyro bias equals the truth
  const double I[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  const double zero[3] = {0, 0, 0};
  svc.setActive(Sensor::Gyro, zero, I);

  TEST_ASSERT_EQUAL_UINT8(uint8_t(StartResult::Started), uint8_t(svc.start(Sensor::Accel, false, sim.t)));
  TEST_ASSERT_TRUE(svc.collecting(Sensor::Accel));
  TEST_ASSERT_TRUE(svc.wantsAccel());
  TEST_ASSERT_EQUAL_UINT8(uint8_t(StartResult::Busy), uint8_t(svc.start(Sensor::Mag, false, sim.t)));
  TEST_ASSERT_FALSE(svc.startGyroRun(sim.t, sim.t + 9 * S)); // no automatic run while a session collects
  captureTwelvePoses(svc, sim, 12);
  TEST_ASSERT_EQUAL_size_t(12, svc.poseCount());
  TEST_ASSERT_TRUE(log.has("pose 12 captured"));
  TEST_ASSERT_TRUE(log.has("directions covered"));
  TEST_ASSERT_TRUE(log.has("coverage complete with"));

  TEST_ASSERT_TRUE(svc.stop(Sensor::Accel, sim.t));
  TEST_ASSERT_EQUAL_UINT8(uint8_t(State::Fitting), uint8_t(svc.state(Sensor::Accel)));
  svc.tick(sim.t);
  TEST_ASSERT_EQUAL_UINT8(uint8_t(State::Review), uint8_t(svc.state(Sensor::Accel)));
  TEST_ASSERT_TRUE(svc.applicable(Sensor::Accel));
  const auto &r = svc.accelResult();
  TEST_ASSERT_TRUE(r.result == CalResult::Pass || r.result == CalResult::Warn); // |b_z| = 1.1 > 0.8 warns
  for (int k = 0; k < 3; k++)
    TEST_ASSERT_DOUBLE_WITHIN(0.02, sim.b[k], r.offset[k]);

  // legacy keys: cal_body = T A (raw - b) must equal M (raw - b) with M = C T
  const double T[9] = {-1, 0, 0, 0, 1, 0, 0, 0, -1};
  double off[3], C[9];
  TEST_ASSERT_TRUE(accelCandidateToLegacy(r, T, off, C));
  double M[9], TA[9];
  mat3Mul(C, T, M);
  mat3Mul(T, r.A, TA);
  for (int i = 0; i < 9; i++)
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, TA[i], M[i]);
  // a raw sample of any pose maps to |a| = g with the legacy form
  const double dirs[3][3] = {{0.3, -0.5, 0.8124}, {-0.9, 0.1, 0.42}, {0.0, 0.0, -1.0}};
  for (int p = 0; p < 3; p++) {
    double u[3] = {dirs[p][0], dirs[p][1], dirs[p][2]};
    normalise(u);
    const double vv[3] = {9.81 * u[0], 9.81 * u[1], 9.81 * u[2]};
    double raw[3], dd[3], out[3];
    rawFromTrue(sim.Ainv, sim.b, vv, raw);
    for (int k = 0; k < 3; k++)
      dd[k] = raw[k] - off[k];
    mat3Vec(M, dd, out);
    TEST_ASSERT_DOUBLE_WITHIN(0.05, 9.81, vec3Norm(out));
  }

  svc.markApplied(Sensor::Accel);
  TEST_ASSERT_EQUAL_UINT8(uint8_t(State::Applied), uint8_t(svc.state(Sensor::Accel)));
  svc.markSaved();
  TEST_ASSERT_EQUAL_UINT8(uint8_t(State::Saved), uint8_t(svc.state(Sensor::Accel)));
}

static void test_stale_gyro_bias_does_not_block_pose_capture() {
  // Active gyro offset 0.03 rad/s off the truth (a stale stored value after a failed boot run).
  for (int trusted = 0; trusted < 2; trusted++) {
    CalibrationService svc(g_magBuf, CalibrationService::kMagBufferCapacity);
    Sim sim;
    sim.gyroBias[0] = sim.gyroBias[1] = sim.gyroBias[2] = 0.03;
    const double I[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    const double stale[3] = {0, 0, 0};
    svc.setActive(Sensor::Gyro, stale, I);
    svc.setGyroBiasTrusted(trusted != 0);
    svc.start(Sensor::Accel, false, sim.t);
    captureTwelvePoses(svc, sim, 3);
    if (trusted)
      TEST_ASSERT_EQUAL_size_t(0, svc.poseCount()); // |w - bias| = 0.052 rad/s > 0.02: never still (by design)
    else
      TEST_ASSERT_EQUAL_size_t(3, svc.poseCount()); // bias-free stillness: std and peak-to-peak only
  }
}

static void test_accel_too_few_poses_fails_and_discard() {
  CalibrationService svc(g_magBuf, CalibrationService::kMagBufferCapacity);
  Log log;
  svc.setLogSink(Log::sink, &log);
  Sim sim;
  sim.gyroBias[0] = sim.gyroBias[1] = sim.gyroBias[2] = 0;
  const double I[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  const double zero[3] = {0, 0, 0};
  svc.setActive(Sensor::Gyro, zero, I);
  svc.start(Sensor::Accel, false, sim.t);
  captureTwelvePoses(svc, sim, 4);
  svc.stop(Sensor::Accel, sim.t);
  svc.tick(sim.t);
  TEST_ASSERT_EQUAL_UINT8(uint8_t(State::Failed), uint8_t(svc.state(Sensor::Accel)));
  TEST_ASSERT_FALSE(svc.applicable(Sensor::Accel));
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::FailCoverage), uint8_t(svc.result(Sensor::Accel)));
  TEST_ASSERT_TRUE(log.has("missing: "));
  TEST_ASSERT_FALSE(log.has("coverage complete"));
  svc.discard(Sensor::Accel);
  TEST_ASSERT_EQUAL_UINT8(uint8_t(State::Idle), uint8_t(svc.state(Sensor::Accel)));
  TEST_ASSERT_FALSE(svc.stop(Sensor::Accel, sim.t)); // not collecting
}

static void test_duplicate_pose_not_stored() {
  CalibrationService svc(g_magBuf, CalibrationService::kMagBufferCapacity);
  Log log;
  svc.setLogSink(Log::sink, &log);
  Sim sim;
  sim.gyroBias[0] = sim.gyroBias[1] = sim.gyroBias[2] = 0;
  const double I[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  const double zero[3] = {0, 0, 0};
  svc.setActive(Sensor::Gyro, zero, I);
  svc.start(Sensor::Accel, false, sim.t);
  const double up[3] = {0, 0, 1}, nearUp[3] = {0.05, 0, 0.9987};
  movePose(svc, sim, up, 4.0);
  movePose(svc, sim, nearUp, 4.0); // 3 deg away: the same pose
  TEST_ASSERT_EQUAL_size_t(1, svc.poseCount());
  TEST_ASSERT_TRUE(log.has("same orientation"));
}

/// Mag tumble: field of radius R about centre c, random directions, 100 Hz samples.
static void feedMagTumble(CalibrationService &svc, Rng &rng, int64_t &t, double seconds, const double c[3],
                          double R) {
  const int steps = int(seconds * 100);
  for (int i = 0; i < steps; i++) {
    double u[3];
    // slowly varying direction: golden-angle walk plus noise so thinning and coverage both work
    rng.unit(u);
    double m[3] = {c[0] + R * u[0] + 0.001 * rng.normal(), c[1] + R * u[1] + 0.001 * rng.normal(),
                   c[2] + R * u[2] + 0.001 * rng.normal()};
    svc.addMag(t, m);
    t += 10 * MS;
    if (i % 10 == 9)
      svc.tick(t);
  }
}

static void test_mag_session_recovers_centre() {
  CalibrationService svc(g_magBuf, CalibrationService::kMagBufferCapacity);
  Log log;
  svc.setLogSink(Log::sink, &log);
  Rng rng(3);
  int64_t t = 0;
  const double c[3] = {0.012, -0.021, 0.008};
  const double R = 0.31;
  TEST_ASSERT_EQUAL_UINT8(uint8_t(StartResult::Started), uint8_t(svc.start(Sensor::Mag, false, t)));
  TEST_ASSERT_TRUE(svc.wantsMag());
  feedMagTumble(svc, rng, t, 100.0, c, R);
  TEST_ASSERT_TRUE(svc.stop(Sensor::Mag, t));
  svc.tick(t);
  TEST_ASSERT_EQUAL_UINT8(uint8_t(State::Review), uint8_t(svc.state(Sensor::Mag)));
  const auto &r = svc.magResult();
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::Pass), uint8_t(r.result));
  for (int k = 0; k < 3; k++)
    TEST_ASSERT_DOUBLE_WITHIN(0.002, c[k], r.offset[k]);
  TEST_ASSERT_DOUBLE_WITHIN(0.003, R, r.radius);
  double off[3], C[9];
  magCandidateToLegacy(r, off, C);
  TEST_ASSERT_DOUBLE_WITHIN(0.002, 0.05 / R, C[0]);
  TEST_ASSERT_EQUAL_DOUBLE(0.0, C[1]);
  TEST_ASSERT_TRUE(log.has("Mag candidate ready"));
  // samples are decimated to 10 Hz: about 1000 of the 10000 offered (600 buffer limit applies to the stored ones)
  TEST_ASSERT_TRUE(r.nAccumulated > 900 && r.nAccumulated < 1100);
}

static void test_mag_poor_coverage_fails() {
  CalibrationService svc(g_magBuf, CalibrationService::kMagBufferCapacity);
  Rng rng(4);
  int64_t t = 0;
  svc.start(Sensor::Mag, false, t);
  // only the upper half sphere
  const double c[3] = {0, 0, 0};
  for (int i = 0; i < 3000; i++) {
    double u[3];
    rng.unit(u);
    if (u[2] < 0)
      u[2] = -u[2];
    double m[3] = {c[0] + 0.3 * u[0], c[1] + 0.3 * u[1], c[2] + 0.3 * u[2]};
    svc.addMag(t, m);
    t += 10 * MS;
  }
  svc.stop(Sensor::Mag, t);
  svc.tick(t);
  TEST_ASSERT_EQUAL_UINT8(uint8_t(State::Failed), uint8_t(svc.state(Sensor::Mag)));
  TEST_ASSERT_FALSE(svc.applicable(Sensor::Mag));
}

static void test_accel_verify_mode() {
  // active calibration equal to the truth: PASS; a wrong one: FAIL. Verify ends in Idle with no candidate.
  Sim sim;
  sim.gyroBias[0] = sim.gyroBias[1] = sim.gyroBias[2] = 0;
  sim.accNoise = 0.005;
  const double I[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  const double zero[3] = {0, 0, 0};
  for (int pass = 0; pass < 2; pass++) {
    CalibrationService svc(g_magBuf, CalibrationService::kMagBufferCapacity);
    Log log;
    svc.setLogSink(Log::sink, &log);
    svc.setActive(Sensor::Gyro, zero, I);
    double off[3] = {sim.b[0], sim.b[1], sim.b[2] + (pass ? 0.4 : 0.0)};
    svc.setActive(Sensor::Accel, off, sim.A);
    svc.start(Sensor::Accel, true, sim.t);
    double u[3][3] = {{0, 0, 1}, {0.7071, 0, 0.7071}, {0, 0.6, 0.8}};
    for (int i = 0; i < 3; i++)
      movePose(svc, sim, u[i], 4.0);
    svc.stop(Sensor::Accel, sim.t);
    svc.tick(sim.t);
    TEST_ASSERT_EQUAL_UINT8(uint8_t(State::Idle), uint8_t(svc.state(Sensor::Accel)));
    TEST_ASSERT_FALSE(svc.applicable(Sensor::Accel));
    TEST_ASSERT_TRUE(log.has(pass ? "Accel VERIFY FAIL" : "Accel VERIFY PASS"));
  }
}

static void test_timeouts_start_fit() {
  ServiceConfig cfg;
  cfg.magMaxNs = 20 * S;
  CalibrationService svc(g_magBuf, CalibrationService::kMagBufferCapacity, cfg);
  svc.start(Sensor::Mag, false, 0);
  svc.tick(10 * S);
  TEST_ASSERT_TRUE(svc.collecting(Sensor::Mag));
  svc.tick(21 * S);
  TEST_ASSERT_EQUAL_UINT8(uint8_t(State::Failed), uint8_t(svc.state(Sensor::Mag))); // nothing collected
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::FailCoverage), uint8_t(svc.result(Sensor::Mag)));
}

static void test_manual_gyro_session_goes_to_review() {
  CalibrationService svc(g_magBuf, CalibrationService::kMagBufferCapacity);
  Sim sim;
  TEST_ASSERT_EQUAL_UINT8(uint8_t(StartResult::Started), uint8_t(svc.start(Sensor::Gyro, false, sim.t)));
  sim.run(svc, 8.0);
  TEST_ASSERT_EQUAL_UINT8(uint8_t(State::Review), uint8_t(svc.state(Sensor::Gyro)));
  TEST_ASSERT_TRUE(svc.applicable(Sensor::Gyro));
  GyroRunDone done;
  TEST_ASSERT_FALSE(svc.takeGyroRunDone(done)); // manual sessions do not use the automatic channel
  for (int k = 0; k < 3; k++)
    TEST_ASSERT_DOUBLE_WITHIN(0.0005, sim.gyroBias[k], svc.gyroResult().bias[k]);
}

// Review finding 5 and test 3: a finished run that was not taken is neither dropped nor overwritten by a new start.
static void test_start_gyro_run_keeps_pending_result() {
  CalibrationService svc(g_magBuf, CalibrationService::kMagBufferCapacity);
  Sim sim;
  TEST_ASSERT_TRUE(svc.startGyroRun(sim.t, sim.t + 9 * S));
  sim.run(svc, 7.0); // finished, NOT taken
  TEST_ASSERT_FALSE(svc.gyroRunActive());
  const double firstBias = sim.gyroBias[0];
  sim.gyroBias[0] = 0.040; // a different truth for the second run
  TEST_ASSERT_TRUE(svc.startGyroRun(sim.t, sim.t + 9 * S)); // started while the first result is pending
  TEST_ASSERT_TRUE(svc.gyroRunActive());
  GyroRunDone done;
  TEST_ASSERT_TRUE(svc.takeGyroRunDone(done)); // the pending result of the first run is still delivered
  TEST_ASSERT_DOUBLE_WITHIN(0.0005, firstBias, done.result.bias[0]);
  TEST_ASSERT_FALSE(svc.takeGyroRunDone(done));
  sim.run(svc, 7.0);
  TEST_ASSERT_TRUE(svc.takeGyroRunDone(done)); // and the second one arrives as its own result
  TEST_ASSERT_DOUBLE_WITHIN(0.0005, 0.040, done.result.bias[0]);
}

// Test 3 (rest): start() while an automatic run collects cancels it; an automatic run neither starts nor disturbs a
// manual gyro candidate waiting in Review.
static void test_manual_candidate_survives_automatic_run() {
  CalibrationService svc(g_magBuf, CalibrationService::kMagBufferCapacity);
  Sim sim;
  TEST_ASSERT_TRUE(svc.startGyroRun(sim.t, sim.t + 9 * S));
  sim.run(svc, 1.0);
  TEST_ASSERT_EQUAL_UINT8(uint8_t(StartResult::Started), uint8_t(svc.start(Sensor::Gyro, false, sim.t)));
  TEST_ASSERT_FALSE(svc.gyroRunActive()); // the automatic run was cancelled by the session
  sim.run(svc, 8.0);
  TEST_ASSERT_EQUAL_UINT8(uint8_t(State::Review), uint8_t(svc.state(Sensor::Gyro)));
  const double candidate = svc.gyroResult().bias[0];
  sim.gyroBias[0] = 0.050;
  TEST_ASSERT_TRUE(svc.startGyroRun(sim.t, sim.t + 9 * S)); // allowed in Review (nothing is collecting)
  sim.run(svc, 7.0);
  TEST_ASSERT_EQUAL_UINT8(uint8_t(State::Review), uint8_t(svc.state(Sensor::Gyro)));
  TEST_ASSERT_TRUE(svc.applicable(Sensor::Gyro));
  TEST_ASSERT_EQUAL_DOUBLE(candidate, svc.gyroResult().bias[0]); // the manual candidate is untouched
  GyroRunDone done;
  TEST_ASSERT_TRUE(svc.takeGyroRunDone(done));
  TEST_ASSERT_DOUBLE_WITHIN(0.0005, 0.050, done.result.bias[0]);
}

// Review finding 8 and test 7: a candidate counts as applicable only if it also passes the arming range checks, so a
// saved calibration is never rejected by Arm as "out of range".
static void test_candidate_must_pass_arming_param_checks() {
  // mag with a raw field radius of 1.5 (above the 1.00 limit of checkMagParams): the sphere fit itself passes
  {
    CalibrationService svc(g_magBuf, CalibrationService::kMagBufferCapacity);
    Log log;
    svc.setLogSink(Log::sink, &log);
    Rng rng(3);
    int64_t t = 0;
    const double c[3] = {0.012, -0.021, 0.008};
    TEST_ASSERT_EQUAL_UINT8(uint8_t(StartResult::Started), uint8_t(svc.start(Sensor::Mag, false, t)));
    feedMagTumble(svc, rng, t, 100.0, c, 1.5);
    TEST_ASSERT_TRUE(svc.stop(Sensor::Mag, t));
    svc.tick(t);
    TEST_ASSERT_TRUE(svc.magResult().result == CalResult::Pass || svc.magResult().result == CalResult::Warn);
    TEST_ASSERT_EQUAL_UINT8(uint8_t(Reason::OutOfRange), uint8_t(magCandidateParams(svc.magResult())));
    TEST_ASSERT_FALSE(svc.applicable(Sensor::Mag));
    TEST_ASSERT_EQUAL_UINT8(uint8_t(State::Failed), uint8_t(svc.state(Sensor::Mag)));
    TEST_ASSERT_TRUE(log.has("not acceptable for arming"));
  }
  // a good mag candidate (R = 0.31) is applicable and its legacy form passes checkMagParams
  {
    CalibrationService svc(g_magBuf, CalibrationService::kMagBufferCapacity);
    Rng rng(3);
    int64_t t = 0;
    const double c[3] = {0.012, -0.021, 0.008};
    svc.start(Sensor::Mag, false, t);
    feedMagTumble(svc, rng, t, 100.0, c, 0.31);
    svc.stop(Sensor::Mag, t);
    svc.tick(t);
    TEST_ASSERT_TRUE(svc.applicable(Sensor::Mag));
    double off[3], C[9];
    magCandidateToLegacy(svc.magResult(), off, C);
    StoredCalibration st;
    st.keyPresent = true;
    for (int i = 0; i < 3; i++)
      st.offset[i] = off[i];
    for (int i = 0; i < 9; i++)
      st.C[i] = C[i];
    TEST_ASSERT_EQUAL_UINT8(uint8_t(Reason::Ok), uint8_t(checkMagParams(st)));
  }
}

// Accel: the legacy form of an applicable candidate (any mounting matrix T) passes checkAccelParams.
static void test_accel_candidate_legacy_form_passes_check() {
  CalibrationService svc(g_magBuf, CalibrationService::kMagBufferCapacity);
  Sim sim;
  sim.gyroBias[0] = sim.gyroBias[1] = sim.gyroBias[2] = 0;
  const double I[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  const double zero[3] = {0, 0, 0};
  svc.setActive(Sensor::Gyro, zero, I);
  svc.start(Sensor::Accel, false, sim.t);
  captureTwelvePoses(svc, sim, 12);
  svc.stop(Sensor::Accel, sim.t);
  svc.tick(sim.t);
  TEST_ASSERT_TRUE(svc.applicable(Sensor::Accel));
  TEST_ASSERT_EQUAL_UINT8(uint8_t(Reason::Ok), uint8_t(accelCandidateParams(svc.accelResult())));
  const double T[9] = {0, 0, -1, 1, 0, 0, 0, -1, 0}; // the vehicle's accel mounting
  StoredCalibration st;
  st.keyPresent = true;
  TEST_ASSERT_TRUE(accelCandidateToLegacy(svc.accelResult(), T, st.offset, st.C));
  for (int i = 0; i < 9; i++)
    st.T[i] = T[i];
  TEST_ASSERT_EQUAL_UINT8(uint8_t(Reason::Ok), uint8_t(checkAccelParams(st)));
  // an offset above the 2.5 m/s^2 limit of the arming check is not applicable
  VCTR::DSP::Calib::AccelFitResult bad = svc.accelResult();
  bad.offset[2] = 2.6;
  TEST_ASSERT_EQUAL_UINT8(uint8_t(Reason::OutOfRange), uint8_t(accelCandidateParams(bad)));
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_gyro_auto_run_recovers_bias);
  RUN_TEST(test_gyro_auto_run_gives_up_with_motion_and_cancel);
  RUN_TEST(test_accel_session_twelve_poses_fit_and_legacy_conversion);
  RUN_TEST(test_stale_gyro_bias_does_not_block_pose_capture);
  RUN_TEST(test_accel_too_few_poses_fails_and_discard);
  RUN_TEST(test_duplicate_pose_not_stored);
  RUN_TEST(test_mag_session_recovers_centre);
  RUN_TEST(test_mag_poor_coverage_fails);
  RUN_TEST(test_accel_verify_mode);
  RUN_TEST(test_timeouts_start_fit);
  RUN_TEST(test_manual_gyro_session_goes_to_review);
  RUN_TEST(test_start_gyro_run_keeps_pending_result);
  RUN_TEST(test_manual_candidate_survives_automatic_run);
  RUN_TEST(test_candidate_must_pass_arming_param_checks);
  RUN_TEST(test_accel_candidate_legacy_form_passes_check);
  return UNITY_END();
}
