// Host tests of the magnetometer sphere (hard-iron) fit and the 64-bin sphere coverage (lib/ExVectrDSP
// calibration/mag_sphere_fit, sphere_coverage): pio test -e native -f test_cal_mag_fit
// Known offset and radius recovered (exact and noisy), coverage and sample-count checks for a half tumble, soft-iron
// distortion failing the residual check, offset beyond the field radius, degenerate and non-finite input, and the
// Fibonacci lattice definition shared with the ground station.
#include <math.h>
#include <unity.h>

#include "../cal_test_support.hpp"
#include "ExVectrDSP/calibration/mag_sphere_fit.hpp"
#include "ExVectrDSP/calibration/sphere_coverage.hpp"

using namespace VCTR::DSP::Calib;
using namespace CalTest;

void setUp() {}
void tearDown() {}

static float gBuf[600][3];

// Bench-like raw values: radius 0.31 driver units (0.73 x 0.42 G), hard-iron offset of a few percent.
static const double kC[3] = {0.0105, -0.0147, 0.0082};
static const double kR = 0.31;

static void feedTumble(MagSphereFit &fit, Rng &rng, size_t n, double noise, bool upperHalfOnly, const double *soft) {
  for (size_t i = 0; i < n; i++) {
    double u[3];
    rng.unit(u);
    if (upperHalfOnly && u[2] < 0)
      u[2] = -u[2];
    double v[3] = {kR * u[0], kR * u[1], kR * u[2]};
    if (soft != nullptr) {
      const double w[3] = {soft[0] * v[0] + soft[1] * v[1] + soft[2] * v[2],
                           soft[3] * v[0] + soft[4] * v[1] + soft[5] * v[2],
                           soft[6] * v[0] + soft[7] * v[1] + soft[8] * v[2]};
      v[0] = w[0];
      v[1] = w[1];
      v[2] = w[2];
    }
    const double m[3] = {v[0] + kC[0] + noise * rng.normal(), v[1] + kC[1] + noise * rng.normal(),
                         v[2] + kC[2] + noise * rng.normal()};
    fit.addSample(m);
    if (i % 50 == 49)
      fit.updateEstimate(); // the vehicle refreshes the centre at 1 Hz
  }
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

static void test_exact_and_noisy_recovery() {
  MagSphereFit fit(gBuf, 600);
  Rng rng(21);
  feedTumble(fit, rng, 3000, 0.0, false, nullptr);
  MagFitResult r = fit.evaluate();
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::Pass), uint8_t(r.result));
  for (int k = 0; k < 3; k++)
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, kC[k], r.offset[k]);
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, kR, r.radius);
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.05 / kR, r.scale);
  TEST_ASSERT_TRUE(r.nStored >= 300 && r.nStored <= 600);
  TEST_ASSERT_TRUE(r.coverage >= 0.75);
  TEST_ASSERT_TRUE(r.residualRms < 1e-6);
  TEST_ASSERT_EQUAL_UINT16(0x3F, r.checks);

  // QMC5883 noise about 2 mG at 8 G range: 0.002 driver units.
  fit.reset();
  feedTumble(fit, rng, 3000, 0.002, false, nullptr);
  r = fit.evaluate();
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::Pass), uint8_t(r.result));
  for (int k = 0; k < 3; k++)
    TEST_ASSERT_DOUBLE_WITHIN(0.01 * kR, kC[k], r.offset[k]); // within 1 % of the field radius
  TEST_ASSERT_DOUBLE_WITHIN(0.01 * kR, kR, r.radius);
  TEST_ASSERT_TRUE(r.residualRms < 0.015);
}

static void test_half_tumble_coverage_fails() {
  MagSphereFit fit(gBuf, 600);
  Rng rng(22);
  feedTumble(fit, rng, 3000, 0.0, true, nullptr);
  const MagFitResult r = fit.evaluate();
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::FailCoverage), uint8_t(r.result));
  TEST_ASSERT_TRUE(r.coverage < 0.75);
}

static void test_few_samples_fail() {
  MagSphereFit fit(gBuf, 600);
  Rng rng(23);
  feedTumble(fit, rng, 120, 0.0, false, nullptr);
  const MagFitResult r = fit.evaluate();
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::FailCoverage), uint8_t(r.result));
  TEST_ASSERT_TRUE((r.checks & kMagCheckSamples) == 0);
}

static void test_soft_iron_fails_residual() {
  // 20 % axis ratio of soft iron: a sphere cannot fit it -> FAIL_RESIDUAL (the owner chose bias only, so a vehicle
  // with strong soft iron is detected, not corrected).
  const double soft[9] = {1.10, 0, 0, 0, 0.92, 0, 0, 0, 1.0};
  MagSphereFit fit(gBuf, 600);
  Rng rng(24);
  feedTumble(fit, rng, 3000, 0.0, false, soft);
  const MagFitResult r = fit.evaluate();
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::FailResidual), uint8_t(r.result));
}

static void test_offset_beyond_radius_fails_range() {
  MagSphereFit fit(gBuf, 600);
  Rng rng(25);
  for (int i = 0; i < 3000; i++) {
    double u[3];
    rng.unit(u);
    const double m[3] = {0.1 * u[0] + 0.09, 0.1 * u[1], 0.1 * u[2]}; // |c| = 0.9 R
    fit.addSample(m);
    if (i % 50 == 49)
      fit.updateEstimate();
  }
  const MagFitResult r = fit.evaluate();
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::FailRange), uint8_t(r.result));
}

static void test_degenerate_and_non_finite() {
  MagSphereFit fit(gBuf, 600);
  MagFitResult r = fit.evaluate();
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::FailCoverage), uint8_t(r.result)); // no samples
  const double same[3] = {0.1, 0.2, 0.3};
  for (int i = 0; i < 500; i++)
    fit.addSample(same);
  r = fit.evaluate();
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::Numeric), uint8_t(r.result));
  fit.reset();
  const double bad[3] = {NAN, 0, 0};
  fit.addSample(bad);
  TEST_ASSERT_EQUAL(0, fit.accumulatedCount());
}

static void test_buffer_capacity() {
  static float small[10][3];
  MagSphereFit fit(small, 10);
  Rng rng(26);
  feedTumble(fit, rng, 2000, 0.0, false, nullptr);
  TEST_ASSERT_EQUAL(10, fit.storedCount());
  TEST_ASSERT_EQUAL(2000, fit.accumulatedCount());
  const MagFitResult r = fit.evaluate(); // the fit still uses all samples
  for (int k = 0; k < 3; k++)
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, kC[k], r.offset[k]);
  TEST_ASSERT_EQUAL_UINT8(uint8_t(CalResult::FailCoverage), uint8_t(r.result));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_lattice);
  RUN_TEST(test_exact_and_noisy_recovery);
  RUN_TEST(test_half_tumble_coverage_fails);
  RUN_TEST(test_few_samples_fail);
  RUN_TEST(test_soft_iron_fails_residual);
  RUN_TEST(test_offset_beyond_radius_fails_range);
  RUN_TEST(test_degenerate_and_non_finite);
  RUN_TEST(test_buffer_capacity);
  return UNITY_END();
}
