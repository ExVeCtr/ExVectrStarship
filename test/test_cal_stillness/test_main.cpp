// Host tests of the stillness detector, the automatic pose collector and the stationary gyro bias collector
// (lib/ExVectrDSP calibration/stillness_detector, pose_collector, gyro_bias_collector):
// pio test -e native -f test_cal_stillness
#include <math.h>
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
  // A slow steady rotation (1 deg/s) keeps the gyro std low; the turning gravity vector restarts the window.
  GyroBiasCollector g;
  ImuSim sim;
  int64_t t = 0;
  g.start(t, t + 8 * S);
  double ang = 0;
  for (int i = 0; i < 9000; i++, t += MS) {
    double w[3], a[3];
    sim.sample(w, a);
    w[0] += 1.0 * kDeg;
    ang += 1.0 * kDeg * 0.001;
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

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_stillness_detects_still_and_motion);
  RUN_TEST(test_pose_collector_twelve_poses);
  RUN_TEST(test_pose_collector_one_event_per_stillness_and_abort);
  RUN_TEST(test_gyro_bias_recovered);
  RUN_TEST(test_gyro_motion_restarts_and_gives_up);
  RUN_TEST(test_gyro_steady_rotation_detected);
  RUN_TEST(test_gyro_checks);
  return UNITY_END();
}
