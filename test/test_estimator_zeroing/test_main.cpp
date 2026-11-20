// Host tests of src/safety/estimator_zeroing.cpp: pio test -e native
// The ZeroEstimators validation and preconditions, the duration clamp, the vehicle-side window timer (restart,
// completion), the "valid zero since boot" rule for Ready, the boot auto-zero condition, the refusal of Arm and
// MissionBegin during a window, and SystemReset (abort, keeps the valid-zero record).
#include <math.h>
#include <unity.h>

#include "safety/estimator_zeroing.hpp"

using namespace VCTR::Safety;

static constexpr int64_t S = EstimatorZeroing::kSecondNs;

void setUp() {}
void tearDown() {}

static ZeroRequestContext groundCtx() {
  ZeroRequestContext c;
  c.initialised = true;
  c.armed = false;
  c.motorsEnabled = false;
  c.onGround = true;
  return c;
}

static void test_duration_clamp() {
  TEST_ASSERT_EQUAL_INT64(10 * S, EstimatorZeroing::durationNs(NAN));
  TEST_ASSERT_EQUAL_INT64(10 * S, EstimatorZeroing::durationNs(0.0f));
  TEST_ASSERT_EQUAL_INT64(10 * S, EstimatorZeroing::durationNs(-0.0f));
  TEST_ASSERT_EQUAL_INT64(1 * S, EstimatorZeroing::durationNs(0.2f));
  TEST_ASSERT_EQUAL_INT64(1 * S, EstimatorZeroing::durationNs(-5.0f));
  TEST_ASSERT_EQUAL_INT64(1 * S, EstimatorZeroing::durationNs(-INFINITY));
  TEST_ASSERT_EQUAL_INT64(25 * S, EstimatorZeroing::durationNs(25.0f));
  TEST_ASSERT_EQUAL_INT64(S + S / 2, EstimatorZeroing::durationNs(1.5f));
  TEST_ASSERT_EQUAL_INT64(60 * S, EstimatorZeroing::durationNs(61.0f));
  TEST_ASSERT_EQUAL_INT64(60 * S, EstimatorZeroing::durationNs(INFINITY));
}

static void test_validation_and_preconditions() {
  EstimatorZeroing z;
  ZeroRequestContext c = groundCtx();

  TEST_ASSERT_EQUAL(ZeroRequestResult::BadParam, z.request(0x2F, 10, c, 0));
  TEST_ASSERT_EQUAL(ZeroRequestResult::BadParam, z.request(0, 10, c, 0));

  c = groundCtx();
  c.initialised = false;
  TEST_ASSERT_EQUAL(ZeroRequestResult::NotInitialised, z.request(0x2E, 10, c, 0));
  c = groundCtx();
  c.armed = true;
  TEST_ASSERT_EQUAL(ZeroRequestResult::Armed, z.request(0x2E, 10, c, 0));
  c = groundCtx();
  c.motorsEnabled = true;
  TEST_ASSERT_EQUAL(ZeroRequestResult::MotorsEnabled, z.request(0x2E, 10, c, 0));
  c = groundCtx();
  c.onGround = false;
  TEST_ASSERT_EQUAL(ZeroRequestResult::NotOnGround, z.request(0x2E, 10, c, 0));

  TEST_ASSERT_FALSE(z.active());
  TEST_ASSERT_EQUAL_UINT32(6, z.refusedCount());

  TEST_ASSERT_EQUAL(ZeroRequestResult::Started, z.request(0x2E, 10, groundCtx(), 100));
  TEST_ASSERT_TRUE(z.active());
  TEST_ASSERT_EQUAL_INT64(100 + 10 * S, z.endNs());
  TEST_ASSERT_EQUAL_UINT32(6, z.refusedCount());
  TEST_ASSERT_TRUE(accepted(ZeroRequestResult::Started));
  TEST_ASSERT_TRUE(accepted(ZeroRequestResult::Restarted));
  TEST_ASSERT_FALSE(accepted(ZeroRequestResult::Armed));
}

static void test_window_completes_and_records_zero() {
  EstimatorZeroing z;
  TEST_ASSERT_FALSE(z.zeroedSinceBoot());
  TEST_ASSERT_FALSE(z.readyAllowed());
  z.request(0x2E, NAN, groundCtx(), 1 * S); // default 10 s: window [1 s, 11 s)
  TEST_ASSERT_FALSE(z.update(5 * S));
  TEST_ASSERT_TRUE(z.active());
  TEST_ASSERT_FALSE(z.update(11 * S - 1));
  TEST_ASSERT_TRUE(z.active());
  TEST_ASSERT_FALSE(z.readyAllowed());
  TEST_ASSERT_TRUE(z.update(11 * S)); // completion edge, exactly once
  TEST_ASSERT_FALSE(z.active());
  TEST_ASSERT_TRUE(z.zeroedSinceBoot());
  TEST_ASSERT_TRUE(z.readyAllowed());
  TEST_ASSERT_FALSE(z.update(12 * S));
}

static void test_second_request_restarts_window() {
  EstimatorZeroing z;
  z.request(0x2E, 10, groundCtx(), 0);
  z.update(8 * S);
  TEST_ASSERT_EQUAL(ZeroRequestResult::Restarted, z.request(0x2E, 5, groundCtx(), 8 * S));
  TEST_ASSERT_EQUAL_INT64(13 * S, z.endNs());
  TEST_ASSERT_FALSE(z.update(10 * S)); // the first window's end no longer applies
  TEST_ASSERT_TRUE(z.active());
  TEST_ASSERT_TRUE(z.update(13 * S));
  TEST_ASSERT_TRUE(z.zeroedSinceBoot());
}

static void test_refused_request_does_not_touch_active_window() {
  EstimatorZeroing z;
  z.request(0x2E, 10, groundCtx(), 0);
  ZeroRequestContext armed = groundCtx();
  armed.armed = true;
  TEST_ASSERT_EQUAL(ZeroRequestResult::Armed, z.request(0x2E, 60, armed, 5 * S));
  TEST_ASSERT_EQUAL_INT64(10 * S, z.endNs());
  TEST_ASSERT_TRUE(z.update(10 * S));
}

static void test_arm_and_mission_begin_refused_while_zeroing() {
  EstimatorZeroing z;
  TEST_ASSERT_TRUE(z.armAllowed());
  TEST_ASSERT_TRUE(z.missionBeginAllowed());
  z.request(0x2E, 2, groundCtx(), 0);
  TEST_ASSERT_FALSE(z.armAllowed());
  TEST_ASSERT_FALSE(z.missionBeginAllowed());
  z.update(2 * S);
  TEST_ASSERT_TRUE(z.armAllowed());
  TEST_ASSERT_TRUE(z.missionBeginAllowed());
}

static void test_ready_requires_prior_zero_and_boot_auto_zero() {
  EstimatorZeroing z;
  // before the inputs are initialised nothing starts, and Ready is not allowed
  TEST_ASSERT_FALSE(z.shouldStartAutoZero(false, true));
  TEST_ASSERT_FALSE(z.readyAllowed());
  // only in Startup
  TEST_ASSERT_FALSE(z.shouldStartAutoZero(true, false));
  TEST_ASSERT_TRUE(z.shouldStartAutoZero(true, true));
  z.startAuto(3 * S);
  TEST_ASSERT_TRUE(z.active());
  TEST_ASSERT_EQUAL_INT64(13 * S, z.endNs());
  TEST_ASSERT_FALSE(z.shouldStartAutoZero(true, true)); // not while running
  TEST_ASSERT_FALSE(z.readyAllowed());
  TEST_ASSERT_TRUE(z.update(13 * S));
  TEST_ASSERT_TRUE(z.readyAllowed());
  TEST_ASSERT_FALSE(z.shouldStartAutoZero(true, true)); // never again once a zero is valid
}

static void test_system_reset_aborts_and_keeps_record() {
  // SystemReset during the boot zero: no zero recorded, Ready not allowed, the auto-zero restarts
  EstimatorZeroing boot;
  boot.startAuto(0);
  TEST_ASSERT_TRUE(boot.abort());
  TEST_ASSERT_FALSE(boot.active());
  TEST_ASSERT_FALSE(boot.zeroedSinceBoot());
  TEST_ASSERT_FALSE(boot.readyAllowed());
  TEST_ASSERT_FALSE(boot.update(100 * S)); // aborted window never completes
  TEST_ASSERT_TRUE(boot.shouldStartAutoZero(true, true));

  // SystemReset with a valid zero and no window: nothing changes, Ready allowed (state reset only)
  EstimatorZeroing z;
  z.startAuto(0);
  z.update(10 * S);
  TEST_ASSERT_FALSE(z.abort());
  TEST_ASSERT_TRUE(z.zeroedSinceBoot());
  TEST_ASSERT_TRUE(z.readyAllowed());
  TEST_ASSERT_FALSE(z.shouldStartAutoZero(true, true));

  // SystemReset during a commanded re-zero: aborted, the earlier valid zero is kept
  z.request(0x2E, 10, groundCtx(), 20 * S);
  TEST_ASSERT_TRUE(z.abort());
  TEST_ASSERT_TRUE(z.zeroedSinceBoot());
  TEST_ASSERT_TRUE(z.readyAllowed());
}

static void test_log_rate_limit() {
  LogRateLimit l(1 * S);
  TEST_ASSERT_TRUE(l.due(0));
  TEST_ASSERT_FALSE(l.due(S / 2));
  TEST_ASSERT_TRUE(l.due(S));
  TEST_ASSERT_FALSE(l.due(S + 1));
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_duration_clamp);
  RUN_TEST(test_validation_and_preconditions);
  RUN_TEST(test_window_completes_and_records_zero);
  RUN_TEST(test_second_request_restarts_window);
  RUN_TEST(test_refused_request_does_not_touch_active_window);
  RUN_TEST(test_arm_and_mission_begin_refused_while_zeroing);
  RUN_TEST(test_ready_requires_prior_zero_and_boot_auto_zero);
  RUN_TEST(test_system_reset_aborts_and_keeps_record);
  RUN_TEST(test_log_rate_limit);
  return UNITY_END();
}
