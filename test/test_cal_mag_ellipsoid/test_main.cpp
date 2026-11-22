// Host tests of the magnetometer ellipsoid (hard + soft iron) fit (lib/ExVectrDSP calibration/mag_ellipsoid_fit):
// pio test -e native -f test_cal_mag_ellipsoid
// Known (b, A) recovered from distorted data while the sphere fit shows the residual, outlier rejection, a sphere
// dataset giving A = s I, poor coverage, a planar tumble, softIron=false versus true on the same data, plausibility
// limits, and non-finite input.
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unity.h>

#include "../cal_test_support.hpp"
#include "ExVectrDSP/calibration/mag_ellipsoid_fit.hpp"
#include "ExVectrDSP/calibration/sphere_coverage.hpp"

using namespace VCTR::DSP::Calib;
using namespace CalTest;

void setUp() {}
void tearDown() {}

static float gBuf[600][3];

static const double kB[3] = {0.0105, -0.0147, 0.0082};
static const double kR = 0.31;
// Soft-iron distortion S: raw = b + R S u (eigenvalues about 0.89..1.16, axis ratio 1.3).
static const double kS[9] = {1.12, 0.05, -0.03, 0.05, 0.90, 0.04, -0.03, 0.04, 1.05};

static void inv3(const double *m, double *o) {
  const double c00 = m[4] * m[8] - m[5] * m[7], c01 = m[5] * m[6] - m[3] * m[8], c02 = m[3] * m[7] - m[4] * m[6];
  const double det = m[0] * c00 + m[1] * c01 + m[2] * c02, id = 1 / det;
  o[0] = c00 * id;
  o[1] = (m[2] * m[7] - m[1] * m[8]) * id;
  o[2] = (m[1] * m[5] - m[2] * m[4]) * id;
  o[3] = c01 * id;
  o[4] = (m[0] * m[8] - m[2] * m[6]) * id;
  o[5] = (m[2] * m[3] - m[0] * m[5]) * id;
  o[6] = c02 * id;
  o[7] = (m[1] * m[6] - m[0] * m[7]) * id;
  o[8] = (m[0] * m[4] - m[1] * m[3]) * id;
}

/// Feeds n samples: raw = b + R S u + noise. soft == nullptr: a sphere. outlierFrac of the samples are replaced by
/// disturbed ones (field length scaled by 0.7..1.4). mode: 0 sphere of directions, 1 upper half only, 2 planar (z = 0).
static void feed(MagEllipsoidFit &fit, Rng &rng, size_t n, double noise, const double *soft, double outlierFrac = 0,
                 int mode = 0) {
  for (size_t i = 0; i < n; i++) {
    double u[3];
    rng.unit(u);
    if (mode == 1 && u[2] < 0)
      u[2] = -u[2];
    if (mode == 2) {
      u[2] = 0;
      normalise(u);
    }
    double v[3] = {kR * u[0], kR * u[1], kR * u[2]};
    if (soft != nullptr) {
      const double w[3] = {soft[0] * v[0] + soft[1] * v[1] + soft[2] * v[2],
                           soft[3] * v[0] + soft[4] * v[1] + soft[5] * v[2],
                           soft[6] * v[0] + soft[7] * v[1] + soft[8] * v[2]};
      v[0] = w[0];
      v[1] = w[1];
      v[2] = w[2];
    }
    if (outlierFrac > 0 && rng.uniform() < outlierFrac) {
      const double f = (rng.uniform() < 0.5 ? 0.7 : 1.4) + 0.1 * rng.uniform();
      v[0] *= f;
      v[1] *= f;
      v[2] *= f;
    }
    const double m[3] = {v[0] + kB[0] + noise * rng.normal(), v[1] + kB[1] + noise * rng.normal(),
                         v[2] + kB[2] + noise * rng.normal()};
    fit.addSample(m);
    if (i % 50 == 49)
      fit.updateEstimate();
  }
}

/// True A = (rRef / R) S^-1.
static void trueA(double *A) {
  double Si[9];
  inv3(kS, Si);
  for (int i = 0; i < 9; i++)
    A[i] = 0.05 / kR * Si[i];
}

static void test_recovers_soft_and_hard_iron() {
  MagEllipsoidFit fit(gBuf, 600);
  Rng rng(7);
  feed(fit, rng, 8000, 0.0015, kS);
  const MagEllipsoidResult r = fit.evaluate();
  printf("# soft: result %s checks 0x%03x stored %u rms %.3f%% max %.3f%% ratio %.3f sigma %.4f %.4f rejected %u\n",
         toString(r.result), (unsigned)r.checks, (unsigned)r.nStored, r.residualRms * 100, r.residualMax * 100,
         r.axisRatio, r.sigmaOffset, r.sigmaMatrix, (unsigned)r.nRejected);
  printf("# b %.5f %.5f %.5f A %.4f %.4f %.4f %.4f %.4f %.4f\n", r.offset[0], r.offset[1], r.offset[2], r.A[0], r.A[4],
         r.A[8], r.A[1], r.A[2], r.A[5]);
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::Pass), uint8_t(r.result));
  double At[9];
  trueA(At);
  for (int k = 0; k < 3; k++)
    TEST_ASSERT_DOUBLE_WITHIN(0.002, kB[k], r.offset[k]);
  const double mean = (At[0] + At[4] + At[8]) / 3;
  for (int k = 0; k < 9; k++)
    TEST_ASSERT_DOUBLE_WITHIN(0.01 * mean, At[k], r.A[k]);
  TEST_ASSERT_TRUE(r.axisRatio > 1.2 && r.axisRatio < 1.4);
  TEST_ASSERT_TRUE(r.residualRms < 0.015);
}

static void test_sphere_mode_shows_residual() {
  // Same distorted data, both modes.
  MagEllipsoidConfig sc;
  sc.softIron = false;
  MagEllipsoidFit sph(gBuf, 600, sc);
  Rng rng(7);
  feed(sph, rng, 8000, 0.0015, kS);
  const MagEllipsoidResult rs = sph.evaluate();
  printf("# sphere mode: result %s rms %.3f%% max %.3f%% ratio %.3f\n", toString(rs.result), rs.residualRms * 100,
         rs.residualMax * 100, rs.axisRatio);
  TEST_ASSERT_TRUE(rs.residualRms > 0.015); // clearly above the 1.5 % warn level
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::FailResidual), uint8_t(rs.result));
  TEST_ASSERT_EQUAL_DOUBLE(rs.A[0], rs.A[4]); // A = s I
  TEST_ASSERT_EQUAL_DOUBLE(0.0, rs.A[1]);
  TEST_ASSERT_EQUAL_UINT8(0, uint8_t(rs.nRejected));
  // The ellipsoid fit on the same samples (same seed) passes.
  static float buf2[600][3];
  MagEllipsoidFit el(buf2, 600);
  Rng rng2(7);
  feed(el, rng2, 8000, 0.0015, kS);
  const MagEllipsoidResult re = el.evaluate();
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::Pass), uint8_t(re.result));
  TEST_ASSERT_TRUE(re.residualRms < rs.residualRms / 3);
}

static void test_outliers_rejected() {
  MagEllipsoidFit fit(gBuf, 600);
  Rng rng(11);
  feed(fit, rng, 8000, 0.0015, kS, 0.08);
  const MagEllipsoidResult r = fit.evaluate();
  printf("# outliers: result %s rejected %u of %u rms %.3f%% max %.3f%%\n", toString(r.result),
         (unsigned)r.nRejected, (unsigned)r.nStored, r.residualRms * 100, r.residualMax * 100);
  TEST_ASSERT_TRUE(acceptable(r.result));
  TEST_ASSERT_TRUE(r.nRejected >= r.nStored / 30); // about 8 % were disturbed
  double At[9];
  trueA(At);
  for (int k = 0; k < 3; k++)
    TEST_ASSERT_DOUBLE_WITHIN(0.002, kB[k], r.offset[k]);
  const double mean = (At[0] + At[4] + At[8]) / 3;
  for (int k = 0; k < 9; k++)
    TEST_ASSERT_DOUBLE_WITHIN(0.015 * mean, At[k], r.A[k]);
}

static void test_too_many_outliers_fail() {
  MagEllipsoidFit fit(gBuf, 600);
  Rng rng(12);
  feed(fit, rng, 8000, 0.0015, kS, 0.45);
  const MagEllipsoidResult r = fit.evaluate();
  TEST_ASSERT_FALSE(acceptable(r.result));
}

static void test_sphere_data_gives_sI() {
  MagEllipsoidFit fit(gBuf, 600);
  Rng rng(5);
  feed(fit, rng, 8000, 0.0015, nullptr);
  const MagEllipsoidResult r = fit.evaluate();
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::Pass), uint8_t(r.result));
  const double s = 0.05 / kR;
  for (int k = 0; k < 9; k++)
    TEST_ASSERT_DOUBLE_WITHIN(0.01 * s, (k % 4 == 0) ? s : 0.0, r.A[k]);
  TEST_ASSERT_TRUE(r.axisRatio < 1.05);
  for (int k = 0; k < 3; k++)
    TEST_ASSERT_DOUBLE_WITHIN(0.002, kB[k], r.offset[k]);
}

static void test_poor_coverage_fails() {
  MagEllipsoidFit fit(gBuf, 600);
  Rng rng(9);
  feed(fit, rng, 8000, 0.0015, kS, 0, 1); // upper half only
  const MagEllipsoidResult r = fit.evaluate();
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::FailCoverage), uint8_t(r.result));
  TEST_ASSERT_TRUE(r.coverage < 0.75);
  TEST_ASSERT_NOT_NULL(strstr(magFailReason(r), "not covered"));
}

static void test_planar_tumble_fails_clearly() {
  MagEllipsoidFit fit(gBuf, 600);
  Rng rng(13);
  feed(fit, rng, 8000, 0.0015, kS, 0, 2);
  const MagEllipsoidResult r = fit.evaluate();
  printf("# planar: result %s reason '%s'\n", toString(r.result), magFailReason(r));
  TEST_ASSERT_FALSE(acceptable(r.result));
  TEST_ASSERT_TRUE(r.result == CalResult::FailCoverage || r.result == CalResult::Numeric);
  TEST_ASSERT_TRUE(strlen(magFailReason(r)) > 10);
}

static void test_too_few_samples() {
  MagEllipsoidFit fit(gBuf, 600);
  Rng rng(2);
  feed(fit, rng, 3, 0.0015, kS);
  const MagEllipsoidResult r = fit.evaluate();
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::FailCoverage), uint8_t(r.result));
  MagEllipsoidFit empty(gBuf, 600);
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::FailCoverage), uint8_t(empty.evaluate().result));
}

static void test_implausible_soft_iron_fails_range() {
  // axis ratio 1.8: perfectly fitted but outside the plausibility limit
  const double S2[9] = {1.4, 0, 0, 0, 0.8, 0, 0, 0, 1.0};
  MagEllipsoidFit fit(gBuf, 600);
  Rng rng(21);
  feed(fit, rng, 8000, 0.0015, S2);
  const MagEllipsoidResult r = fit.evaluate();
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::FailRange), uint8_t(r.result));
  TEST_ASSERT_TRUE(r.axisRatio > 1.5);
  TEST_ASSERT_NOT_NULL(strstr(magFailReason(r), "soft-iron"));
}

static void test_non_finite_ignored() {
  MagEllipsoidFit fit(gBuf, 600);
  const double bad[3] = {NAN, 0, 1};
  fit.addSample(bad);
  TEST_ASSERT_EQUAL_UINT32(0, fit.accumulatedCount());
}

static void test_dense_data_beyond_buffer() {
  // far more samples than the buffer: the normal matrix has no limit, the result stays good
  MagEllipsoidFit fit(gBuf, 600);
  Rng rng(31);
  feed(fit, rng, 60000, 0.0015, kS);
  TEST_ASSERT_TRUE(fit.storedCount() <= 600);
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::Pass), uint8_t(fit.evaluate().result));
}

static void test_lattice() {
  for (size_t i = 0; i < SphereCoverage::kBins; i++) {
    double c[3];
    SphereCoverage::binCentre(i, c);
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, 1.0, c[0] * c[0] + c[1] * c[1] + c[2] * c[2]);
    TEST_ASSERT_EQUAL_INT(int(i), SphereCoverage::binOf(c));
  }
  double c0[3];
  SphereCoverage::binCentre(0, c0);
  TEST_ASSERT_DOUBLE_WITHIN(1e-12, 1.0 - 1.0 / 64.0, c0[2]); // z = 1 - (2i+1)/64
  const double zero[3] = {0, 0, 0};
  TEST_ASSERT_EQUAL_INT(-1, SphereCoverage::binOf(zero));
  const double nanv[3] = {NAN, 0, 1};
  TEST_ASSERT_EQUAL_INT(-1, SphereCoverage::binOf(nanv));
  // Equal-area bins: uniform directions hit every bin, each with roughly 1/64 of the samples.
  SphereCoverage cov;
  Rng rng(11);
  size_t hits[64] = {0};
  for (int i = 0; i < 64000; i++) {
    double u[3];
    rng.unit(u);
    const int b = cov.add(u);
    TEST_ASSERT_TRUE(b >= 0);
    hits[b]++;
  }
  for (int b = 0; b < 64; b++) {
    TEST_ASSERT_TRUE(hits[b] > 600 && hits[b] < 1400);
    TEST_ASSERT_EQUAL_UINT8(255, cov.count(b)); // saturating
  }
  TEST_ASSERT_EQUAL_DOUBLE(1.0, cov.fractionAtLeast(2));
  cov.reset();
  TEST_ASSERT_EQUAL_DOUBLE(0.0, cov.fractionAtLeast(1));
}

static void test_offset_beyond_radius_fails_range() {
  MagEllipsoidFit fit(gBuf, 600);
  Rng rng(25);
  for (int i = 0; i < 3000; i++) {
    double u[3];
    rng.unit(u);
    const double m[3] = {0.1 * u[0] + 0.09, 0.1 * u[1], 0.1 * u[2]}; // |c| = 0.9 R
    fit.addSample(m);
    if (i % 50 == 49)
      fit.updateEstimate();
  }
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::FailRange), uint8_t(fit.evaluate().result));
}

static void test_degenerate_identical_samples() {
  MagEllipsoidFit fit(gBuf, 600);
  const double same[3] = {0.1, 0.2, 0.3};
  for (int i = 0; i < 500; i++)
    fit.addSample(same);
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::Numeric), uint8_t(fit.evaluate().result));
}

static void test_buffer_capacity() {
  static float small[10][3];
  MagEllipsoidFit fit(small, 10);
  Rng rng(26);
  feed(fit, rng, 2000, 0.0, nullptr);
  TEST_ASSERT_EQUAL(10, fit.storedCount());
  TEST_ASSERT_EQUAL(2000, fit.accumulatedCount());
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::FailCoverage), uint8_t(fit.evaluate().result));
}

/// Sphere data with Gaussian field noise of the given relative sigma (noise per axis = rel * R gives a length rms of
/// about rel), evaluated with the default limits or with maxResidualMax overridden.
static MagEllipsoidResult noisyFit(double rel, uint32_t seed, double maxMax = -1) {
  static float buf3[600][3];
  MagEllipsoidConfig cfg;
  if (maxMax > 0)
    cfg.maxResidualMax = maxMax;
  MagEllipsoidFit fit(buf3, 600, cfg);
  Rng rng(seed);
  feed(fit, rng, 8000, rel * kR, nullptr);
  return fit.evaluate();
}

static void test_residual_pass_warn_fail_bands() {
  // Clean: rms well under 1.5 % -> Pass, no residual warning.
  MagEllipsoidResult r = noisyFit(0.005, 21);
  printf("# band 0.5%%: %s rms %.3f%% max %.3f%%\n", toString(r.result), r.residualRms * 100, r.residualMax * 100);
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::Pass), uint8_t(r.result));
  TEST_ASSERT_TRUE(r.checks & kMagCheckResidualNominal);
  // The owner's tumble: rms about 2.9 % -> Warn, still applicable, with the noise reason.
  r = noisyFit(0.029, 22);
  printf("# band 2.9%%: %s rms %.3f%% max %.3f%% reason '%s'\n", toString(r.result), r.residualRms * 100,
         r.residualMax * 100, magFailReason(r));
  TEST_ASSERT_TRUE(r.residualRms > 0.025 && r.residualRms < 0.032);
  TEST_ASSERT_TRUE(r.residualMax < 0.10);
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::Warn), uint8_t(r.result));
  TEST_ASSERT_TRUE(acceptable(r.result));
  TEST_ASSERT_NOT_NULL(strstr(magFailReason(r), "field noise or disturbance during the tumble (rms 2."));
  // rms 4 % -> FailResidual.
  r = noisyFit(0.04, 23);
  printf("# band 4%%: %s rms %.3f%% max %.3f%%\n", toString(r.result), r.residualRms * 100, r.residualMax * 100);
  TEST_ASSERT_TRUE(r.residualRms > 0.035);
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::FailResidual), uint8_t(r.result));
  TEST_ASSERT_FALSE(acceptable(r.result));
}

static void test_residual_max_limit() {
  // The same 2.9 % data has a worst inlier of about 8 %: it passes the 10 % limit and fails a 5 % limit.
  MagEllipsoidResult ok = noisyFit(0.029, 22);
  TEST_ASSERT_TRUE(ok.residualMax > 0.05 && ok.residualMax < 0.10);
  MagEllipsoidResult bad = noisyFit(0.029, 22, 0.05);
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::FailResidual), uint8_t(bad.result));
  // A worst inlier above the max limit (rms still under 3.5 %) fails: limit set just under the observed max.
  MagEllipsoidResult tight = noisyFit(0.029, 22, ok.residualMax * 0.9);
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::FailResidual), uint8_t(tight.result));
  MagEllipsoidConfig def;
  TEST_ASSERT_EQUAL_DOUBLE(0.10, def.maxResidualMax);
  TEST_ASSERT_EQUAL_DOUBLE(0.035, def.maxResidualRms);
  TEST_ASSERT_EQUAL_DOUBLE(0.015, def.warnResidualRms);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_recovers_soft_and_hard_iron);
  RUN_TEST(test_sphere_mode_shows_residual);
  RUN_TEST(test_residual_pass_warn_fail_bands);
  RUN_TEST(test_residual_max_limit);
  RUN_TEST(test_outliers_rejected);
  RUN_TEST(test_too_many_outliers_fail);
  RUN_TEST(test_sphere_data_gives_sI);
  RUN_TEST(test_poor_coverage_fails);
  RUN_TEST(test_planar_tumble_fails_clearly);
  RUN_TEST(test_too_few_samples);
  RUN_TEST(test_implausible_soft_iron_fails_range);
  RUN_TEST(test_non_finite_ignored);
  RUN_TEST(test_dense_data_beyond_buffer);
  RUN_TEST(test_lattice);
  RUN_TEST(test_offset_beyond_radius_fails_range);
  RUN_TEST(test_degenerate_identical_samples);
  RUN_TEST(test_buffer_capacity);
  return UNITY_END();
}
