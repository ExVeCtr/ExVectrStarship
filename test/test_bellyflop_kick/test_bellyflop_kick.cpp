// Host tests of src/safety/bellyflop_kick.cpp: pio test -e native -f test_bellyflop_kick
//
// The powered bellyflop kick (user decision 2026-11-21): end triggers (hard time limit first), the pitch-over
// target (pure pitch about body +Y from the start heading, = old fixed Quat(y, -90 deg) at identity) and the
// remaining-angle metric.
#include <math.h>
#include <unity.h>

#include "safety/bellyflop_kick.hpp"
#include "starship_hardware.h"

using namespace VCTR::Safety;

void setUp() {}
void tearDown() {}

static constexpr int64_t MS = 1000000LL;
static const float kDeg = kKickDeg;

static void quatAxisAngle(float ax, float ay, float az, float ang, float q[4]) {
  const float s = sinf(0.5f * ang);
  q[0] = cosf(0.5f * ang);
  q[1] = ax * s;
  q[2] = ay * s;
  q[3] = az * s;
}

static void quatMul(const float a[4], const float b[4], float o[4]) {
  o[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
  o[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
  o[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
  o[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
}

static void axis(const float q[4], float x, float y, float z, float out[3]) {
  const float v[3] = {x, y, z};
  kickRotate(q, v, out);
}

static void test_constants_sane() {
  TEST_ASSERT_TRUE(kKickTimeLimitNs > 0 && kKickTimeLimitNs <= 1000 * MS); // short: at most 1 s
  TEST_ASSERT_TRUE(kKickThrustLimitN > 0.0f && kKickThrustLimitN <= 14.0f); // never above the motor limit
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 90.0f * kDeg, kKickPitchAngleRad);
  TEST_ASSERT_TRUE(kKickGuardMarginNs > 0 && kKickGuardMarginNs <= 200 * MS);
}

static void test_end_trigger() {
  // Nothing reached: continue.
  TEST_ASSERT_TRUE(kickEndTrigger(100 * MS, 90 * kDeg, 0, kKickEndLegacy) == KickEnd::None);
  TEST_ASSERT_TRUE(kickEndTrigger(kKickTimeLimitNs, 90 * kDeg, 0, kKickEndLegacy) == KickEnd::None);
  // Hard time limit, whatever the attitude (also with invalid inputs).
  TEST_ASSERT_TRUE(kickEndTrigger(kKickTimeLimitNs + 1, 90 * kDeg, 0, kKickEndLegacy) == KickEnd::Time);
  TEST_ASSERT_TRUE(kickEndTrigger(kKickTimeLimitNs + 1, NAN, NAN, kKickEndLegacy) == KickEnd::Time);
  TEST_ASSERT_TRUE(kickEndTrigger(kKickTimeLimitNs + 1, 10 * kDeg, 200 * kDeg, kKickEndLegacy) == KickEnd::Time);
  // Angle.
  const float a = kKickEndRemainingAngleRad, r = kKickEndPitchRateRadPs;
  TEST_ASSERT_TRUE(kickEndTrigger(200 * MS, a - 1 * kDeg, 0, kKickEndLegacy) == KickEnd::Angle);
  TEST_ASSERT_TRUE(kickEndTrigger(200 * MS, a + 1 * kDeg, 0, kKickEndLegacy) == KickEnd::None);
  // Rate (spin guard), either sign.
  TEST_ASSERT_TRUE(kickEndTrigger(200 * MS, 80 * kDeg, r + 1 * kDeg, kKickEndLegacy) == KickEnd::Rate);
  TEST_ASSERT_TRUE(kickEndTrigger(200 * MS, 80 * kDeg, -r - 1 * kDeg, kKickEndLegacy) == KickEnd::Rate);
  TEST_ASSERT_TRUE(kickEndTrigger(200 * MS, 80 * kDeg, r - 1 * kDeg, kKickEndLegacy) == KickEnd::None);
  TEST_ASSERT_TRUE(r > 150 * kDeg); // above the bench peak (135 deg/s): a guard, not the normal end
  // Non-finite angle/rate never end the kick early.
  TEST_ASSERT_TRUE(kickEndTrigger(200 * MS, NAN, NAN, kKickEndLegacy) == KickEnd::None);
  TEST_ASSERT_TRUE(kickEndTrigger(200 * MS, INFINITY, -INFINITY, kKickEndLegacy) == KickEnd::None);
}

// Coast predictor (feature/kick-end-criterion). Closing rate towards the target = -body Y rate.
static void test_predictor() {
  KickEndParams p = kKickEndDefault;
  TEST_ASSERT_EQUAL_INT(2, p.predictorMode);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 40 * kDeg, p.targetRad);
  const float closing = 100 * kDeg; // rad/s towards the target
  // Ballistic: remaining - w^2 / (2 alpha). alpha = 4: 100 deg/s travels 0.381 rad = 21.8 deg.
  const float travel = closing * closing / (2.0f * p.brakeDecelRadPs2);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.38077f, travel);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 70 * kDeg - 0.38077f, kickPredictedRemaining(70 * kDeg, -closing, p));
  // 60 deg remaining, 100 deg/s: predicted 38.2 <= 40: end. 70 deg: predicted 48.2: continue.
  TEST_ASSERT_TRUE(kickEndTrigger(300 * MS, 60 * kDeg, -closing, p) == KickEnd::Predicted);
  TEST_ASSERT_TRUE(kickEndTrigger(300 * MS, 70 * kDeg, -closing, p) == KickEnd::None);
  // Turning away (opposite rate): the prediction gets worse, never an early end.
  TEST_ASSERT_TRUE(kickEndTrigger(300 * MS, 70 * kDeg, +closing, p) == KickEnd::None);
  TEST_ASSERT_TRUE(kickEndTrigger(300 * MS, 80 * kDeg, 0, p) == KickEnd::None);
  // Lead-time mode: remaining - w * lead (0.15 s: 15 deg).
  p.predictorMode = 1;
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 55.0f * kDeg, kickPredictedRemaining(70 * kDeg, -closing, p));
  TEST_ASSERT_TRUE(kickEndTrigger(300 * MS, 54 * kDeg, -closing, p) == KickEnd::Predicted);
  TEST_ASSERT_TRUE(kickEndTrigger(300 * MS, 56 * kDeg, -closing, p) == KickEnd::None);
  // Off (legacy): no predicted end, the old triggers only.
  TEST_ASSERT_TRUE(isnan(kickPredictedRemaining(70 * kDeg, -closing, kKickEndLegacy)));
  TEST_ASSERT_TRUE(kickEndTrigger(300 * MS, 40 * kDeg, -closing, kKickEndLegacy) == KickEnd::None);
  // Invalid input or alpha never triggers, the hard limit and the guards still win.
  p = kKickEndDefault;
  TEST_ASSERT_TRUE(kickEndTrigger(300 * MS, NAN, -closing, p) == KickEnd::None);
  TEST_ASSERT_TRUE(kickEndTrigger(300 * MS, 80 * kDeg, NAN, p) == KickEnd::None);
  p.brakeDecelRadPs2 = 0;
  TEST_ASSERT_TRUE(isnan(kickPredictedRemaining(70 * kDeg, -closing, p)));
  TEST_ASSERT_TRUE(kickEndTrigger(300 * MS, 80 * kDeg, -closing, p) == KickEnd::None);
  p = kKickEndDefault;
  TEST_ASSERT_TRUE(kickEndTrigger(kKickTimeLimitNs + 1, 90 * kDeg, 0, p) == KickEnd::Time);
  TEST_ASSERT_TRUE(kickEndTrigger(300 * MS, 70 * kDeg, -(kKickEndPitchRateRadPs + 0.1f), p) != KickEnd::None);
  TEST_ASSERT_TRUE(kickEndTrigger(300 * MS, 10 * kDeg, 0, p) == KickEnd::Angle); // old 15 deg trigger stays
  TEST_ASSERT_EQUAL(1000 * MS, kKickTimeLimitNs);                                 // 1 s hard limit unchanged
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 250 * kDeg, kKickEndPitchRateRadPs);
}

static void test_target_identity_is_old_setpoint() {
  const float q0[4] = {1, 0, 0, 0};
  float t[4], old[4];
  kickTargetAttitude(q0, kKickPitchAngleRad, t);
  quatAxisAngle(0, 1, 0, -90 * kDeg, old);
  for (int i = 0; i < 4; i++)
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, old[i], t[i]);
  // Belly (body -X) down, nose (body +Z) horizontal towards world -X (south).
  float b[3], n[3];
  axis(t, -1, 0, 0, b);
  axis(t, 0, 0, 1, n);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, -1.0f, b[2]);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, -1.0f, n[0]);
}

static void test_target_keeps_heading() {
  // Vertical vehicle rolled about its nose by various angles, and slightly tilted: the target is always belly
  // down, nose horizontal towards the start belly direction, and a pure pitch about body +Y from the start.
  for (int k = -6; k <= 6; k++) {
    const float yaw = k * 30.0f * kDeg;
    float qYaw[4], qTilt[4], q0[4];
    quatAxisAngle(0, 0, 1, yaw, qYaw);
    quatAxisAngle(1, 0, 0, 3 * kDeg, qTilt);
    quatMul(qYaw, qTilt, q0);
    float t[4];
    kickTargetAttitude(q0, kKickPitchAngleRad, t);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.0f, t[0] * t[0] + t[1] * t[1] + t[2] * t[2] + t[3] * t[3]);
    TEST_ASSERT_TRUE(t[0] >= 0);
    float bt[3], b0[3], nt[3], y0[3], yt[3];
    axis(t, -1, 0, 0, bt);
    axis(q0, -1, 0, 0, b0);
    axis(t, 0, 0, 1, nt);
    axis(q0, 0, 1, 0, y0);
    axis(t, 0, 1, 0, yt);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -1.0f, bt[2]); // belly down
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, nt[2]);  // nose horizontal
    const float bh = sqrtf(b0[0] * b0[0] + b0[1] * b0[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.0f, (nt[0] * b0[0] + nt[1] * b0[1]) / bh); // nose to start belly side
    // Pitch axis body +Y nearly unchanged (pure pitch, up to the small start tilt).
    TEST_ASSERT_TRUE(y0[0] * yt[0] + y0[1] * yt[1] + y0[2] * yt[2] > cosf(4 * kDeg));
  }
}

static void test_target_degenerate_heading() {
  // Already lying with the belly straight down: heading 0, still a valid unit quaternion.
  float q0[4];
  quatAxisAngle(0, 1, 0, -90 * kDeg, q0);
  float t[4];
  kickTargetAttitude(q0, kKickPitchAngleRad, t);
  float b[3];
  axis(t, -1, 0, 0, b);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -1.0f, b[2]);
}

static void test_remaining_angle() {
  const float q0[4] = {1, 0, 0, 0};
  float t[4];
  kickTargetAttitude(q0, kKickPitchAngleRad, t);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 90 * kDeg, kickRemainingAngle(q0, t));
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, kickRemainingAngle(t, t));
  float q40[4];
  quatAxisAngle(0, 1, 0, -40 * kDeg, q40); // pitched 40 deg towards the belly
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 50 * kDeg, kickRemainingAngle(q40, t));
  TEST_ASSERT_TRUE(kickEndTrigger(100 * MS, kickRemainingAngle(q40, t), 0, kKickEndLegacy) == KickEnd::None);
  float q80[4];
  quatAxisAngle(0, 1, 0, -80 * kDeg, q80);
  TEST_ASSERT_TRUE(kickEndTrigger(100 * MS, kickRemainingAngle(q80, t), 0, kKickEndLegacy) == KickEnd::Angle);
  const float bad[4] = {NAN, 0, 0, 0};
  TEST_ASSERT_TRUE(isnan(kickRemainingAngle(bad, t)));
}

// Owner decision 2026-11-21: the vehicle mass is 1.1 kg (the measured value; a 1.2 kg change was reverted) and the kick
// cap stays 11 N. The kick is not meant to fly: it must merely not throw the ship off while it pitches over.
static void test_mass_and_kick_cap_at_1p1_kg() {
  TEST_ASSERT_EQUAL_FLOAT(1.1f, VEHICLE_MASS_KG);
  const float weight = VEHICLE_MASS_KG * 9.81f; // 10.79 N
  TEST_ASSERT_EQUAL_FLOAT(11.0f, kKickThrustLimitN);
  // Net vertical acceleration of a vertical kick at the cap: +0.19 m/s^2, so at most 0.1 m of climb in the 1 s limit.
  const float aNet = (kKickThrustLimitN - weight) / VEHICLE_MASS_KG;
  TEST_ASSERT_TRUE(aNet > 0.0f && aNet < 0.5f);
  const float t = float(kKickTimeLimitNs) * 1e-9f;
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.095f, 0.5f * aNet * t * t);
}

// MissionFreefall::calculateStoppingDistance (stopDist = 0.5 Vz^2 / (T/m - g), private): with 14 N and 1.1 kg the
// trigger altitude is 0.171 Vz^2 + 20 m. Same formula here, fed with the vehicle constants.
static void test_freefall_trigger_at_1p1_kg() {
  const float aSum = TVC_THRUST_LIMIT_N / VEHICLE_MASS_KG - 9.81f;
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.917f, aSum);
  const float coeff = 0.5f / aSum;
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.171f, coeff);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 88.4f, coeff * 20.0f * 20.0f + 20.0f); // at 20 m/s down
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_constants_sane);
  RUN_TEST(test_end_trigger);
  RUN_TEST(test_predictor);
  RUN_TEST(test_target_identity_is_old_setpoint);
  RUN_TEST(test_target_keeps_heading);
  RUN_TEST(test_target_degenerate_heading);
  RUN_TEST(test_remaining_angle);
  RUN_TEST(test_mass_and_kick_cap_at_1p1_kg);
  RUN_TEST(test_freefall_trigger_at_1p1_kg);
  return UNITY_END();
}
