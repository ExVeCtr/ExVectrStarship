// Host tests of the accelerometer multi-pose magnitude fit (lib/ExVectrDSP calibration/accel_ellipsoid_fit) and the
// small linear algebra it uses: pio test -e native -f test_cal_accel_fit
// Synthetic poses with known offset b and symmetric A (exact and noisy, imprecise pose angles), the diagonal
// fallback, too few poses, missing coverage, degenerate (face-only) pose sets, non-finite input and range failures.
#include <math.h>
#include <unity.h>

#include "../cal_test_support.hpp"
#include "ExVectrDSP/calibration/accel_ellipsoid_fit.hpp"
#include "ExVectrDSP/calibration/cal_linalg.hpp"

using namespace VCTR::DSP::Calib;
using namespace CalTest;

void setUp() {}
void tearDown() {}

// True parameters: offset like the bench vehicle's z drift, scale within +-2 %, small non-orthogonality.
static const double kB[3] = {0.35, -0.42, 1.10};
static const double kA[9] = {1.012, 0.006, -0.004, 0.006, 0.991, 0.008, -0.004, 0.008, 1.018};

static size_t makePoses(double (*poses)[3], Rng &rng, double noise, double jitterDeg, int count = 12) {
  double Ainv[9];
  TEST_ASSERT_TRUE(invert3(kA, Ainv));
  double u[12][3];
  twelvePoses(u, rng, jitterDeg);
  for (int i = 0; i < count; i++) {
    const double v[3] = {9.81 * u[i][0], 9.81 * u[i][1], 9.81 * u[i][2]};
    rawFromTrue(Ainv, kB, v, poses[i]);
    for (int k = 0; k < 3; k++)
      poses[i][k] += noise * rng.normal();
  }
  return size_t(count);
}

static void test_linalg() {
  double A[9] = {4, 1, 0, 1, 3, 1, 0, 1, 2};
  double b[3] = {1, 2, 3};
  double Ac[9];
  for (int i = 0; i < 9; i++)
    Ac[i] = A[i];
  TEST_ASSERT_TRUE(solveLinear(Ac, b, 3));
  for (int r = 0; r < 3; r++) {
    const double s = A[3 * r] * b[0] + A[3 * r + 1] * b[1] + A[3 * r + 2] * b[2];
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, double(r + 1), s);
  }
  double eig[3];
  TEST_ASSERT_TRUE(symmetricEigenvalues(A, 3, eig));
  // trace and determinant are invariants
  TEST_ASSERT_DOUBLE_WITHIN(1e-10, 9.0, eig[0] + eig[1] + eig[2]);
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 18.0, eig[0] * eig[1] * eig[2]);
  TEST_ASSERT_TRUE(eig[0] <= eig[1] && eig[1] <= eig[2]);
  double S[9] = {1, 2, 3, 2, 4, 6, 3, 6, 9}; // rank 1
  double bs[3] = {1, 1, 1};
  TEST_ASSERT_FALSE(solveLinear(S, bs, 3));
  double Mi[9];
  const double M[9] = {0, 0, -1, 1, 0, 0, 0, -1, 0};
  TEST_ASSERT_TRUE(invert3(M, Mi));
  double P[9];
  mul3(M, Mi, P);
  for (int i = 0; i < 9; i++)
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, (i % 4 == 0) ? 1.0 : 0.0, P[i]);
}

static void test_exact_recovery() {
  Rng rng(1);
  double poses[12][3];
  const size_t n = makePoses(poses, rng, 0.0, 8.0);
  const AccelFitResult r = fitAccel(poses, n);
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::Warn), uint8_t(r.result)); // |b_z| 1.10 > 0.8: warning only
  TEST_ASSERT_FALSE(r.diagonal);
  for (int k = 0; k < 3; k++)
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, kB[k], r.offset[k]);
  for (int k = 0; k < 9; k++)
    TEST_ASSERT_DOUBLE_WITHIN(1e-7, kA[k], r.A[k]);
  TEST_ASSERT_TRUE(r.residualRms < 1e-7);
  TEST_ASSERT_TRUE(r.condition < 1e4);
  TEST_ASSERT_TRUE((r.checks & kAccelCheckOffsetNominal) == 0);
  TEST_ASSERT_TRUE((r.checks & kAccelCheckOffset) != 0);
}

static void test_noisy_recovery_pass() {
  // Offset inside the nominal range: verdict PASS. 2 s averages: noise of a pose mean about 0.005 m/s^2.
  static const double b[3] = {0.20, -0.30, 0.55};
  double Ainv[9];
  TEST_ASSERT_TRUE(invert3(kA, Ainv));
  Rng rng(7);
  double u[12][3], poses[12][3];
  twelvePoses(u, rng, 15.0);
  for (int i = 0; i < 12; i++) {
    const double v[3] = {9.81 * u[i][0], 9.81 * u[i][1], 9.81 * u[i][2]};
    rawFromTrue(Ainv, b, v, poses[i]);
    for (int k = 0; k < 3; k++)
      poses[i][k] += 0.005 * rng.normal();
  }
  const AccelFitResult r = fitAccel(poses, 12);
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::Pass), uint8_t(r.result));
  for (int k = 0; k < 3; k++)
    TEST_ASSERT_DOUBLE_WITHIN(0.02, b[k], r.offset[k]); // < 1 % of the offset range and of g
  for (int k = 0; k < 9; k++)
    TEST_ASSERT_DOUBLE_WITHIN(3e-3, kA[k], r.A[k]); // 12 poses with 0.005 m/s^2 noise each: about 0.1-0.3 %
  TEST_ASSERT_TRUE(r.residualRms < 0.01);
  TEST_ASSERT_TRUE(r.iterations > 0 && r.iterations <= 50);
}

static void test_offset_limit_boundary() {
  // 2.4 m/s^2 z offset is accepted (limit 2.5, owner decision 2026-11-21); it only warns (above warnOffset 0.8).
  static const double b[3] = {0.1, 0.1, 2.4};
  double Ainv[9];
  TEST_ASSERT_TRUE(invert3(kA, Ainv));
  Rng rng(9);
  double u[12][3], poses[12][3];
  twelvePoses(u, rng, 10.0);
  for (int i = 0; i < 12; i++) {
    const double v[3] = {9.81 * u[i][0], 9.81 * u[i][1], 9.81 * u[i][2]};
    rawFromTrue(Ainv, b, v, poses[i]);
  }
  TEST_ASSERT_EQUAL_DOUBLE(2.5, kAccelOffsetLimit);
  TEST_ASSERT_EQUAL_DOUBLE(2.5, AccelFitConfig().maxOffset);
  const AccelFitResult r = fitAccel(poses, 12);
  TEST_ASSERT_TRUE((r.checks & kAccelCheckOffset) != 0);
  TEST_ASSERT_TRUE((r.checks & kAccelCheckOffsetNominal) == 0);
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::Warn), uint8_t(r.result));
}

static void test_coverage_mask_and_text() {
  const double g = 9.81;
  const double all[6][3] = {{g, 0, 0}, {-g, 0, 0}, {0, g, 0}, {0, -g, 0}, {0, 0, g}, {0, 0, -g}};
  char txt[32];
  TEST_ASSERT_EQUAL_UINT8(0x3F, accelCoverageMask(all, 6, 25.0 * M_PI / 180));
  TEST_ASSERT_EQUAL(6, accelCoverageCount(0x3F));
  accelMissingText(0x3F, txt, sizeof(txt));
  TEST_ASSERT_EQUAL_STRING("", txt);
  // one missing (-Z)
  uint8_t m = accelCoverageMask(all, 5, 25.0 * M_PI / 180);
  TEST_ASSERT_EQUAL_UINT8(0x1F, m);
  TEST_ASSERT_EQUAL(5, accelCoverageCount(m));
  accelMissingText(m, txt, sizeof(txt));
  TEST_ASSERT_EQUAL_STRING("-Z", txt);
  // several missing: only +Z and -X present
  const double few[2][3] = {{0, 0, g}, {-g, 0, 0}};
  m = accelCoverageMask(few, 2, 25.0 * M_PI / 180);
  TEST_ASSERT_EQUAL_UINT8(0x12, m);
  accelMissingText(m, txt, sizeof(txt));
  TEST_ASSERT_EQUAL_STRING("+X, +Y, -Y, -Z", txt);
  accelMissingText(0, txt, sizeof(txt));
  TEST_ASSERT_EQUAL_STRING("+X, -X, +Y, -Y, +Z, -Z", txt);
  char tiny[5];
  accelMissingText(0, tiny, sizeof(tiny)); // truncates at whole entries, stays terminated
  TEST_ASSERT_EQUAL_STRING("+X", tiny);
  TEST_ASSERT_EQUAL_UINT8(0, accelCoverageMask(all, 0, 25.0 * M_PI / 180));
  // agrees with the fit's coverage check
  TEST_ASSERT_TRUE(accelPoseCoverage(all, 6, 25.0 * M_PI / 180));
  TEST_ASSERT_FALSE(accelPoseCoverage(all, 5, 25.0 * M_PI / 180));
}

static void test_coverage_angle_boundary() {
  const double lim = 25.0 * M_PI / 180;
  for (int axis = 0; axis < 3; axis++) {
    for (int sgn = -1; sgn <= 1; sgn += 2) {
      const int bit = 2 * axis + (sgn > 0 ? 0 : 1);
      // tilted from the axis by 24 and 26 degrees (towards the next axis)
      for (int pass = 0; pass < 2; pass++) {
        const double a = (pass == 0 ? 24.0 : 26.0) * M_PI / 180;
        double p[1][3] = {{0, 0, 0}};
        p[0][axis] = sgn * 9.81 * cos(a);
        p[0][(axis + 1) % 3] = 9.81 * sin(a);
        const uint8_t m = accelCoverageMask(p, 1, lim);
        if (pass == 0)
          TEST_ASSERT_TRUE((m & (1u << bit)) != 0);
        else
          TEST_ASSERT_TRUE((m & (1u << bit)) == 0);
      }
    }
  }
}

static void test_coverage_complete_needs_nine_poses() {
  TEST_ASSERT_FALSE(accelCoverageComplete(8, 0x3F, 9));
  TEST_ASSERT_TRUE(accelCoverageComplete(9, 0x3F, 9));
  TEST_ASSERT_TRUE(accelCoverageComplete(12, 0x3F, 9));
  TEST_ASSERT_FALSE(accelCoverageComplete(12, 0x3E, 9));
  TEST_ASSERT_FALSE(accelCoverageComplete(6, 0x3F, 9)); // the six faces alone are not enough poses
}

static void test_diagonal_fallback() {
  Rng rng(3);
  double poses[12][3];
  makePoses(poses, rng, 0.0, 0.0);
  const AccelFitResult r = fitAccel(poses, 6); // nose up, nose down, 4 horizontal: the six faces
  TEST_ASSERT_TRUE(r.diagonal);
  TEST_ASSERT_TRUE((r.checks & kAccelCheckPosesFull) == 0);
  TEST_ASSERT_TRUE(r.result == CalResult::Warn || r.result == CalResult::FailResidual);
  TEST_ASSERT_EQUAL_DOUBLE(0.0, r.A[1]);
}

static void test_too_few_and_coverage() {
  Rng rng(4);
  double poses[12][3];
  makePoses(poses, rng, 0.0, 0.0);
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::FailCoverage), uint8_t(fitAccel(poses, 5).result));
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::FailCoverage), uint8_t(fitAccel(poses, 0).result));
  // 12 poses but never nose down: remove pose 1 (nose down) and poses 10-11 (nose down 45): -z not covered.
  double p2[12][3];
  size_t n = 0;
  for (int i = 0; i < 12; i++) {
    if (i == 1 || i == 10 || i == 11)
      continue;
    for (int k = 0; k < 3; k++)
      p2[n][k] = poses[i][k];
    n++;
  }
  TEST_ASSERT_EQUAL(9, n);
  const AccelFitResult r = fitAccel(p2, n);
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::FailCoverage), uint8_t(r.result));
  TEST_ASSERT_TRUE((r.checks & kAccelCheckCoverage) == 0);
}

static void test_degenerate_faces_only() {
  // The six faces twice: the off-diagonal terms are not observable -> singular normal matrix -> NUMERIC.
  Rng rng(5);
  double poses[12][3];
  makePoses(poses, rng, 0.0, 0.0);
  double p2[12][3];
  for (int i = 0; i < 12; i++)
    for (int k = 0; k < 3; k++)
      p2[i][k] = poses[i % 6][k];
  const AccelFitResult r = fitAccel(p2, 12);
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::Numeric), uint8_t(r.result));
}

static void test_non_finite() {
  Rng rng(6);
  double poses[12][3];
  makePoses(poses, rng, 0.0, 5.0);
  poses[3][1] = NAN;
  const AccelFitResult r = fitAccel(poses, 12);
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::Numeric), uint8_t(r.result));
}

static void test_range_and_residual_failures() {
  // Offset 2.7 m/s^2 in z (beyond 2.5): FAIL_RANGE although the fit is perfect.
  static const double b[3] = {0.1, 0.1, 2.7};
  double Ainv[9];
  TEST_ASSERT_TRUE(invert3(kA, Ainv));
  Rng rng(8);
  double u[12][3], poses[12][3];
  twelvePoses(u, rng, 10.0);
  for (int i = 0; i < 12; i++) {
    const double v[3] = {9.81 * u[i][0], 9.81 * u[i][1], 9.81 * u[i][2]};
    rawFromTrue(Ainv, b, v, poses[i]);
  }
  AccelFitResult r = fitAccel(poses, 12);
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::FailRange), uint8_t(r.result));
  TEST_ASSERT_DOUBLE_WITHIN(1e-6, 2.7, r.offset[2]);

  // One bad pose (vehicle moved, 0.3 m/s^2 off): FAIL_RESIDUAL.
  static const double b2[3] = {0.1, 0.1, 0.2};
  for (int i = 0; i < 12; i++) {
    const double v[3] = {9.81 * u[i][0], 9.81 * u[i][1], 9.81 * u[i][2]};
    rawFromTrue(Ainv, b2, v, poses[i]);
  }
  poses[4][0] += 0.3;
  r = fitAccel(poses, 12);
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::FailResidual), uint8_t(r.result));
}

static void test_bench_case_identity_start() {
  // The 2026-11-20 bench situation: upright raw z about -8.44 m/s^2 (sensor z down), offset +1.4 in z. Check that
  // the fit finds it from identity start with the sign-flipped sensor axes.
  static const double b[3] = {0.43, 0.56, 1.37};
  static const double A[9] = {0.998, 0, 0, 0, 1.003, 0, 0, 0, 0.997};
  double Ainv[9];
  TEST_ASSERT_TRUE(invert3(A, Ainv));
  Rng rng(9);
  double u[12][3], poses[12][3];
  twelvePoses(u, rng, 12.0);
  for (int i = 0; i < 12; i++) {
    const double v[3] = {-9.81 * u[i][0], 9.81 * u[i][1], -9.81 * u[i][2]};
    rawFromTrue(Ainv, b, v, poses[i]);
    for (int k = 0; k < 3; k++)
      poses[i][k] += 0.004 * rng.normal();
  }
  const AccelFitResult r = fitAccel(poses, 12);
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::Warn), uint8_t(r.result)); // 1.37 > 0.8 warn
  TEST_ASSERT_DOUBLE_WITHIN(0.02, 1.37, r.offset[2]);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_linalg);
  RUN_TEST(test_exact_recovery);
  RUN_TEST(test_noisy_recovery_pass);
  RUN_TEST(test_offset_limit_boundary);
  RUN_TEST(test_coverage_mask_and_text);
  RUN_TEST(test_coverage_angle_boundary);
  RUN_TEST(test_coverage_complete_needs_nine_poses);
  RUN_TEST(test_diagonal_fallback);
  RUN_TEST(test_too_few_and_coverage);
  RUN_TEST(test_degenerate_faces_only);
  RUN_TEST(test_non_finite);
  RUN_TEST(test_range_and_residual_failures);
  RUN_TEST(test_bench_case_identity_start);
  return UNITY_END();
}
