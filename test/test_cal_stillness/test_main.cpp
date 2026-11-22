// Host tests of the stillness detector, the automatic pose collector and the stationary gyro bias collector
// (lib/ExVectrDSP calibration/stillness_detector, pose_collector, gyro_bias_collector):
// pio test -e native -f test_cal_stillness
#include <math.h>
#include <string.h>
#include <unity.h>

#include "../cal_test_support.hpp"
#include "ExVectrDSP/calibration/gyro_bias_collector.hpp"
#include "ExVectrDSP/calibration/pose_collector.hpp"
#include "ExVectrDSP/calibration/stillness_detector.hpp"

using namespace VCTR::DSP::Calib;
using namespace CalTest;

void setUp() {}
void tearDown() {}

static constexpr int64_t MS = 1000000LL;
static constexpr int64_t S = 1000000000LL;

/// IMU stream at 1 kHz: gyro = bias + noise (+ rate while moving), accel = g in direction u + noise (+ shake).
struct ImuSim {
  Rng rng{99};
  double bias[3] = {0.016, 0.035, -0.010};
  double gyroNoise = 0.0015;  // rad/s per sample
  double accNoise = 0.02;     // m/s^2 per sample
  double u[3] = {0, 0, 1};
  double rate = 0;            // rad/s magnitude of motion (about x)
  double shake = 0;           // m/s^2 extra accel noise
  int n = 0;
  void sample(double w[3], double a[3]) {
    const double sgn = ((n++ / 100) % 2) ? -1.0 : 1.0; // handling motion: the rate changes sign every 0.1 s
    for (int k = 0; k < 3; k++) {
      w[k] = bias[k] + gyroNoise * rng.normal() + (k == 0 ? sgn * rate : 0);
      a[k] = 9.81 * u[k] + (accNoise + shake) * rng.normal();
    }
  }
};

static void test_stillness_detects_still_and_motion() {
  StillnessDetector det;
  ImuSim sim;
  det.setGyroBias(sim.bias);
  int64_t t = 0;
  bool wasStill = false;
  int64_t stillAt = -1;
  for (int i = 0; i < 3000; i++, t += MS) {
    double w[3], a[3];
    sim.sample(w, a);
    det.addGyro(t, w);
    det.addAccel(t, a);
    if (det.still() && !wasStill)
      stillAt = t;
    wasStill = det.still();
  }
  TEST_ASSERT_TRUE(det.still());
  TEST_ASSERT_TRUE(stillAt >= 1500 * MS && stillAt <= 1700 * MS); // after 1.5 s of data
  sim.rate = 0.05; // 3 deg/s: motion
  for (int i = 0; i < 300; i++, t += MS) {
    double w[3], a[3];
    sim.sample(w, a);
    det.addGyro(t, w);
    det.addAccel(t, a);
  }
  TEST_ASSERT_FALSE(det.still());
  sim.rate = 0;
  sim.shake = 0.1; // hand-held tremor: accel std too high
  for (int i = 0; i < 3000; i++, t += MS) {
    double w[3], a[3];
    sim.sample(w, a);
    det.addGyro(t, w);
    det.addAccel(t, a);
  }
  TEST_ASSERT_FALSE(det.still());
  // a data gap empties the window
  sim.shake = 0;
  for (int i = 0; i < 2000; i++, t += MS) {
    double w[3], a[3];
    sim.sample(w, a);
    det.addGyro(t, w);
    det.addAccel(t, a);
  }
  TEST_ASSERT_TRUE(det.still());
  t += 1 * S;
  double w[3], a[3];
  sim.sample(w, a);
  det.addAccel(t, a);
  det.addAccel(t + 150 * MS, a);
  TEST_ASSERT_FALSE(det.still());
}

/// Holds pose u still for stillS seconds, then moves for moveS seconds. Returns the events seen.
static int runPose(StillnessDetector &det, PoseCollector &pc, ImuSim &sim, int64_t &t, const double u[3],
                   double stillS, double moveS, PoseEvent *last) {
  int captured = 0;
  for (int k = 0; k < 3; k++)
    sim.u[k] = u[k];
  sim.rate = 0;
  for (int i = 0; i < int(stillS * 1000); i++, t += MS) {
    double w[3], a[3];
    sim.sample(w, a);
    det.addGyro(t, w);
    det.addAccel(t, a);
    const PoseEvent e = pc.addAccel(t, a, det.still());
    if (e == PoseEvent::Captured)
      captured++;
    if (e != PoseEvent::None && e != PoseEvent::Averaging)
      *last = e;
  }
  sim.rate = 0.5;
  for (int i = 0; i < int(moveS * 1000); i++, t += MS) {
    double w[3], a[3];
    sim.sample(w, a);
    det.addGyro(t, w);
    det.addAccel(t, a);
    const PoseEvent e = pc.addAccel(t, a, det.still());
    if (e != PoseEvent::None && e != PoseEvent::Averaging)
      *last = e;
  }
  return captured;
}

static void test_pose_collector_twelve_poses() {
  StillnessDetector det;
  PoseCollector pc;
  ImuSim sim;
  det.setGyroBias(sim.bias);
  Rng rng(5);
  double u[12][3];
  twelvePoses(u, rng, 5.0);
  int64_t t = 0;
  PoseEvent last = PoseEvent::None;
  for (int i = 0; i < 12; i++)
    TEST_ASSERT_EQUAL_INT(1, runPose(det, pc, sim, t, u[i], 4.0, 1.0, &last));
  TEST_ASSERT_EQUAL(12, pc.count());
  // pose means within the averaging noise of the true g vectors
  for (int i = 0; i < 12; i++)
    for (int k = 0; k < 3; k++)
      TEST_ASSERT_DOUBLE_WITHIN(0.01, 9.81 * u[i][k], pc.mean(i)[k]);
  // the same pose again (5 degrees off) is a duplicate
  double again[3];
  poseDirection(5, 0, again);
  TEST_ASSERT_EQUAL_INT(0, runPose(det, pc, sim, t, again, 4.0, 1.0, &last));
  TEST_ASSERT_EQUAL_UINT8(uint8_t(PoseEvent::Duplicate), uint8_t(last));
  TEST_ASSERT_EQUAL(12, pc.count());
}

static void test_pose_collector_one_event_per_stillness_and_abort() {
  StillnessDetector det;
  PoseCollector pc;
  ImuSim sim;
  det.setGyroBias(sim.bias);
  int64_t t = 0;
  PoseEvent last = PoseEvent::None;
  const double up[3] = {0, 0, 1};
  // 20 s lying still: exactly one capture, no stream of duplicates
  TEST_ASSERT_EQUAL_INT(1, runPose(det, pc, sim, t, up, 20.0, 0.5, &last));
  TEST_ASSERT_EQUAL_UINT8(uint8_t(PoseEvent::Captured), uint8_t(last));
  // still for 2.5 s only (1.5 s detection + 1 s of the 2 s average): aborted by the following motion
  double side[3];
  poseDirection(90, 0, side);
  TEST_ASSERT_EQUAL_INT(0, runPose(det, pc, sim, t, side, 2.5, 1.0, &last));
  TEST_ASSERT_EQUAL_UINT8(uint8_t(PoseEvent::Aborted), uint8_t(last));
  TEST_ASSERT_EQUAL(1, pc.count());
}

static void feedGyroWindow(GyroBiasCollector &g, ImuSim &sim, int64_t &t, double seconds, double tempStart,
                           double tempRate) {
  for (int i = 0; i < int(seconds * 1000); i++, t += MS) {
    double w[3], a[3];
    sim.sample(w, a);
    g.addGyro(t, w);
    g.addAccel(t, a);
    if (i % 20 == 0) {
      const double m[3] = {0.1, 0.2, -0.25};
      g.addMag(t, m);
      g.addTemperature(t, tempStart + tempRate * double(i) / 1000.0);
    }
    g.poll(t);
  }
}

static void test_gyro_bias_recovered() {
  GyroBiasCollector g;
  ImuSim sim;
  int64_t t = 0;
  g.start(t, t + 10 * S);
  feedGyroWindow(g, sim, t, 7.0, 300.0, 0.0);
  TEST_ASSERT_TRUE(g.state() == GyroBiasCollector::State::Done);
  const GyroBiasResult &r = g.result();
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::Pass), uint8_t(r.result));
  for (int k = 0; k < 3; k++) {
    TEST_ASSERT_DOUBLE_WITHIN(1e-4, sim.bias[k], r.bias[k]);
    TEST_ASSERT_DOUBLE_WITHIN(0.0005, sim.gyroNoise, r.noiseStd[k]);
  }
  TEST_ASSERT_DOUBLE_WITHIN(0.01, 9.81, r.accelMean[2]);
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, -0.25, r.magMean[2]);
  TEST_ASSERT_TRUE(r.haveTemp);
  TEST_ASSERT_EQUAL(0, r.restarts);
  TEST_ASSERT_EQUAL_UINT16(0x0F, r.checks);
}

static void test_gyro_motion_restarts_and_gives_up() {
  GyroBiasCollector g;
  ImuSim sim;
  int64_t t = 0;
  g.start(t, t + 11 * S);
  feedGyroWindow(g, sim, t, 3.0, 300.0, 0.0);
  sim.rate = 0.1; // bumped
  feedGyroWindow(g, sim, t, 0.5, 300.0, 0.0);
  sim.rate = 0;
  feedGyroWindow(g, sim, t, 6.0, 300.0, 0.0); // the bumped block restarts the window: done at about 9 s
  TEST_ASSERT_TRUE(g.state() == GyroBiasCollector::State::Done);
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::Pass), uint8_t(g.result().result));
  TEST_ASSERT_TRUE(g.result().restarts >= 1);

  // continuous motion until the deadline: MOTION
  g.start(t, t + 6 * S);
  sim.rate = 0.1;
  feedGyroWindow(g, sim, t, 7.0, 300.0, 0.0);
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::Motion), uint8_t(g.result().result));
  TEST_ASSERT_TRUE((g.result().checks & kGyroCheckComplete) == 0);

  // deadline shorter than the window, no motion: TIMEOUT
  sim.rate = 0;
  g.start(t, t + 3 * S);
  feedGyroWindow(g, sim, t, 4.0, 300.0, 0.0);
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::Timeout), uint8_t(g.result().result));
}

static void test_gyro_steady_rotation_detected() {
  // A steady rotation (10 deg/s) keeps the gyro std at zero; the turning gravity vector (accel std about 0.5 m/s^2
  // over a 1 s block, limit 0.3) restarts the window. (1 deg/s is now accepted: below the accel limit and the bias limit.)
  GyroBiasCollector g;
  ImuSim sim;
  int64_t t = 0;
  g.start(t, t + 8 * S);
  double ang = 0;
  for (int i = 0; i < 9000; i++, t += MS) {
    double w[3], a[3];
    sim.sample(w, a);
    w[0] += 10.0 * kDeg;
    ang += 10.0 * kDeg * 0.001;
    const double ay = a[1], az = a[2];
    a[1] = cos(ang) * ay - sin(ang) * az;
    a[2] = sin(ang) * ay + cos(ang) * az;
    g.addGyro(t, w);
    g.addAccel(t, a);
    g.poll(t);
  }
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::Motion), uint8_t(g.result().result));
}

static void test_gyro_checks() {
  GyroBiasCollector g;
  ImuSim sim;
  int64_t t = 0;
  sim.bias[1] = 6.0 * kDeg; // beyond the +-5 deg/s ZRO
  g.start(t, t + 10 * S);
  feedGyroWindow(g, sim, t, 6.0, 300.0, 0.0);
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::FailRange), uint8_t(g.result().result));

  sim.bias[1] = 0.03;
  g.start(t, t + 10 * S);
  feedGyroWindow(g, sim, t, 6.0, 300.0, 0.2); // 0.2 K/s: 1 K over the window
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::FailResidual), uint8_t(g.result().result));
  TEST_ASSERT_TRUE((g.result().checks & kGyroCheckTemp) == 0);
}

/// One 1 kHz sample stream with a hook to disturb it; returns the collector result after `seconds`.
template <typename F>
static GyroBiasResult runCustom(const GyroBiasConfig &cfg, double seconds, F disturb, double deadlineS = 6.5) {
  GyroBiasCollector g(cfg);
  ImuSim sim;
  sim.gyroNoise = 1e-5;
  sim.accNoise = 1e-4;
  int64_t t = 0;
  g.start(t, t + int64_t(deadlineS * S));
  for (int i = 0; i < int(seconds * 1000); i++, t += MS) {
    double w[3], a[3];
    sim.sample(w, a);
    disturb(i, w, a);
    g.addGyro(t, w);
    g.addAccel(t, a);
    g.poll(t);
  }
  return g.result();
}

static void test_gyro_owner_like_noise_passes() {
  // Bench data: gyro std 0.7 deg/s per axis, accel std 0.2 m/s^2 per axis: passes with the loosened limits.
  GyroBiasCollector g;
  ImuSim sim;
  sim.gyroNoise = 0.7 * kDeg;
  sim.accNoise = 0.2;
  int64_t t = 0;
  g.start(t, t + 10 * S);
  feedGyroWindow(g, sim, t, 7.0, 300.0, 0.0);
  TEST_ASSERT_TRUE(g.state() == GyroBiasCollector::State::Done);
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::Pass), uint8_t(g.result().result));
  TEST_ASSERT_EQUAL(0, g.result().restarts);
  for (int k = 0; k < 3; k++)
    TEST_ASSERT_DOUBLE_WITHIN(0.002, sim.bias[k], g.result().bias[k]); // 5000 samples of 0.012 rad/s noise
}

static void test_gyro_real_motion_fails_with_reason() {
  // Handled / rotating: 10 deg/s rate changing sign every 0.1 s, accel shake 1 m/s^2.
  GyroBiasCollector g;
  ImuSim sim;
  sim.rate = 10.0 * kDeg;
  sim.shake = 1.0;
  int64_t t = 0;
  g.start(t, t + 8 * S);
  feedGyroWindow(g, sim, t, 9.0, 300.0, 0.0);
  const GyroBiasResult &r = g.result();
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::Motion), uint8_t(r.result));
  TEST_ASSERT_EQUAL(0, r.bestRun);
  TEST_ASSERT_TRUE(r.rejectedBlocks >= 7);
  char txt[160];
  describeGyroRun(r, g.config(), txt, sizeof txt);
  TEST_ASSERT_NOT_NULL(strstr(txt, "best 0 of 5 blocks"));
  TEST_ASSERT_NOT_NULL(strstr(txt, "worst: gyro std X"));  // 10 deg/s / 1.5 deg/s is the largest ratio, on the x axis
  TEST_ASSERT_NOT_NULL(strstr(txt, "> limit 1.50 deg/s"));
}

static void test_gyro_block_limit_boundaries() {
  const GyroBiasConfig cfg;
  const double gl = cfg.maxBlockStd, pl = cfg.maxBlockP2P;
  char txt[160];
  // Gyro std: alternating +-A per sample on y gives std A and p2p 2A (below the p2p limit for A < 2.5 deg/s).
  auto alt = [&](double amp) {
    return runCustom(cfg, 7.0, [&](int i, double *w, double *) { w[1] += (i & 1) ? amp : -amp; });
  };
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::Pass), uint8_t(alt(0.95 * gl).result));
  GyroBiasResult r = alt(1.1 * gl);
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::Motion), uint8_t(r.result));
  TEST_ASSERT_TRUE(r.worstViolation.kind == GyroViolation::GyroStd);
  TEST_ASSERT_EQUAL(1, r.worstViolation.axis);
  TEST_ASSERT_DOUBLE_WITHIN(0.02 * gl, 1.1 * gl, r.worstViolation.value);
  describeGyroRun(r, cfg, txt, sizeof txt);
  TEST_ASSERT_NOT_NULL(strstr(txt, "gyro std Y 1.65 deg/s > limit 1.50 deg/s"));

  // Gyro p2p: one spike per block on z (low std) of height P.
  auto spike = [&](double h) {
    return runCustom(cfg, 7.0, [&](int i, double *w, double *) { if (i % 1000 == 500) w[2] += h; });
  };
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::Pass), uint8_t(spike(0.95 * pl).result));
  r = spike(1.1 * pl);
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::Motion), uint8_t(r.result));
  TEST_ASSERT_TRUE(r.worstViolation.kind == GyroViolation::GyroP2P);
  TEST_ASSERT_EQUAL(2, r.worstViolation.axis);
  describeGyroRun(r, cfg, txt, sizeof txt);
  TEST_ASSERT_NOT_NULL(strstr(txt, "gyro p2p Z"));

  // Accel axis std: alternating +-A per sample on x (g is on z).
  const double al = cfg.maxAccelAxisStd;
  auto accAlt = [&](double amp) {
    return runCustom(cfg, 7.0, [&](int i, double *, double *a) { a[0] += (i & 1) ? amp : -amp; });
  };
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::Pass), uint8_t(accAlt(0.95 * al).result));
  r = accAlt(1.1 * al);
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::Motion), uint8_t(r.result));
  TEST_ASSERT_TRUE(r.worstViolation.kind == GyroViolation::AccelAxisStd);
  TEST_ASSERT_EQUAL(0, r.worstViolation.axis);
  describeGyroRun(r, cfg, txt, sizeof txt);
  TEST_ASSERT_NOT_NULL(strstr(txt, "accel std X 0.330 m/s^2 > limit 0.300 m/s^2"));

  // |a| std alone (per-axis accel check disabled): alternating +-A along the gravity axis.
  GyroBiasConfig nc = cfg;
  nc.maxAccelAxisStd = 0;
  const double nl = nc.maxAccelNormStd;
  auto normAlt = [&](double amp) {
    return runCustom(nc, 7.0, [&](int i, double *, double *a) { a[2] += (i & 1) ? amp : -amp; });
  };
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::Pass), uint8_t(normAlt(0.95 * nl).result));
  r = normAlt(1.1 * nl);
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::Motion), uint8_t(r.result));
  TEST_ASSERT_TRUE(r.worstViolation.kind == GyroViolation::AccelNormStd);
  describeGyroRun(r, nc, txt, sizeof txt);
  TEST_ASSERT_NOT_NULL(strstr(txt, "|a| std 0.330 m/s^2"));
}

static void test_gyro_isolated_bad_block_costs_only_its_predecessors() {
  // 3 good blocks, one disturbed block, then good data: the run needs 5 consecutive good blocks AFTER the bad one.
  auto bump = [](int i, double *w, double *) { if (i >= 3000 && i < 4000) w[0] += (i & 1) ? 0.2 : -0.2; };
  GyroBiasResult r = runCustom(GyroBiasConfig(), 10.0, bump, 11.0);
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::Pass), uint8_t(r.result));
  TEST_ASSERT_EQUAL(1, r.restarts);
  TEST_ASSERT_EQUAL(1, r.rejectedBlocks);
  TEST_ASSERT_EQUAL(5, r.bestRun);
  // Deadline at 7.5 s: only 3 + 3 good blocks fit around the bad one: MOTION, and the text says best 3 of 5.
  r = runCustom(GyroBiasConfig(), 8.5, bump, 7.5);
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::Motion), uint8_t(r.result));
  TEST_ASSERT_EQUAL(3, r.bestRun);
  char txt[160];
  describeGyroRun(r, GyroBiasConfig(), txt, sizeof txt);
  TEST_ASSERT_NOT_NULL(strstr(txt, "best 3 of 5 blocks, 1 rejected, worst: gyro std X"));
}

static void test_gyro_noise_check_follows_block_limit() {
  // Accepted blocks near the limit must not fail the final window noise check.
  GyroBiasConfig cfg;
  const GyroBiasResult r =
      runCustom(cfg, 7.0, [&](int i, double *w, double *) { w[0] += (i & 1) ? 0.9 * cfg.maxBlockStd : -0.9 * cfg.maxBlockStd; });
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::Pass), uint8_t(r.result));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_stillness_detects_still_and_motion);
  RUN_TEST(test_pose_collector_twelve_poses);
  RUN_TEST(test_pose_collector_one_event_per_stillness_and_abort);
  RUN_TEST(test_gyro_bias_recovered);
  RUN_TEST(test_gyro_motion_restarts_and_gives_up);
  RUN_TEST(test_gyro_steady_rotation_detected);
  RUN_TEST(test_gyro_checks);
  RUN_TEST(test_gyro_owner_like_noise_passes);
  RUN_TEST(test_gyro_real_motion_fails_with_reason);
  RUN_TEST(test_gyro_block_limit_boundaries);
  RUN_TEST(test_gyro_isolated_bad_block_costs_only_its_predecessors);
  RUN_TEST(test_gyro_noise_check_follows_block_limit);
  return UNITY_END();
}
