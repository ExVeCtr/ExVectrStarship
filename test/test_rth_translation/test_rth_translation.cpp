#include <math.h>
#include <stdlib.h>
#include <unity.h>

#include "safety/rth_translation.hpp"
#include "safety/setpoint_sanitiser.hpp"

using namespace VCTR::Safety;

void setUp() {}
void tearDown() {}

// The original formulas (pre-fix), for the non-degenerate equivalence sweep.
static RthTranslationOut oldStep(const RthTranslationState &s, const float ret[3], float dTime) {
  RthTranslationOut o;
  float travel[3], mag = 0;
  for (int i = 0; i < 3; i++) { travel[i] = ret[i] - s.setpoint[i]; mag += travel[i] * travel[i]; }
  mag = sqrtf(mag);
  float norm[3];
  for (int i = 0; i < 3; i++) norm[i] = travel[i] / mag;
  float v = s.velocity;
  if (v < 2) v += dTime * 0.5f;
  if (v > 2) v = 2;
  if (mag / dTime > v) for (int i = 0; i < 3; i++) travel[i] = norm[i] * v * dTime;
  float lim = sqrtf(mag * 0.5f);
  if (v > lim) v = lim;
  for (int i = 0; i < 3; i++) { o.velSetpoint[i] = norm[i] * v; o.setpoint[i] = s.setpoint[i] + travel[i]; }
  o.velocity = v;
  return o;
}

static float rnd(float lo, float hi) { return lo + (hi - lo) * (rand() / (float)RAND_MAX); }

static void test_arrival_mag_zero() {
  RthTranslationState s = {{1, 2, 3}, 1.0f};
  const float ret[3] = {1, 2, 3};
  RthTranslationOut o = rthTranslationStep(s, ret, 0.01f);
  for (int i = 0; i < 3; i++) {
    TEST_ASSERT_TRUE(isfinite(o.velSetpoint[i]));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, o.velSetpoint[i]);
    TEST_ASSERT_EQUAL_FLOAT(ret[i], o.setpoint[i]);
  }
  TEST_ASSERT_EQUAL_FLOAT(0.0f, o.velocity);
}

static void test_equals_old_formulas() {
  srand(1);
  for (int n = 0; n < 20000; n++) {
    RthTranslationState s = {{rnd(-20, 20), rnd(-20, 20), rnd(0, 10)}, rnd(0, 2.5f)};
    const float ret[3] = {rnd(-20, 20), rnd(-20, 20), rnd(0, 10)};
    const float dt = rnd(0.001f, 0.5f);
    RthTranslationOut a = rthTranslationStep(s, ret, dt), b = oldStep(s, ret, dt);
    for (int i = 0; i < 3; i++) {
      TEST_ASSERT_EQUAL_FLOAT(b.setpoint[i], a.setpoint[i]);
      TEST_ASSERT_EQUAL_FLOAT(b.velSetpoint[i], a.velSetpoint[i]);
    }
    TEST_ASSERT_EQUAL_FLOAT(b.velocity, a.velocity);
  }
}

static void test_sweep_always_finite() {
  srand(2);
  const float specials[] = {0.0f, -1.0f, 1e-30f, 1e30f, 1e38f, NAN, INFINITY, -INFINITY};
  for (int n = 0; n < 100000; n++) {
    RthTranslationState s;
    float ret[3];
    for (int i = 0; i < 3; i++) {
      s.setpoint[i] = (n % 7 == 0) ? specials[rand() % 8] : rnd(-50, 50);
      ret[i] = (n % 11 == 0) ? specials[rand() % 8] : (n % 3 == 0 ? s.setpoint[i] : rnd(-50, 50));
    }
    s.velocity = (n % 5 == 0) ? specials[rand() % 8] : rnd(0, 3);
    const float dt = (n % 4 == 0) ? specials[rand() % 8] : rnd(0.0f, 100.0f);
    RthTranslationOut o = rthTranslationStep(s, ret, dt);
    for (int i = 0; i < 3; i++) {
      TEST_ASSERT_TRUE(isfinite(o.setpoint[i]));
      TEST_ASSERT_TRUE(isfinite(o.velSetpoint[i]));
    }
    TEST_ASSERT_TRUE(isfinite(o.velocity));
  }
}

static void test_dt_zero_and_huge() {
  RthTranslationState s = {{0, 0, 3}, 0.0f};
  const float ret[3] = {5, 0, 3};
  RthTranslationOut o = rthTranslationStep(s, ret, 0.0f);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, o.setpoint[0]);
  o = rthTranslationStep(s, ret, 1e9f);
  TEST_ASSERT_TRUE(isfinite(o.setpoint[0]));
  TEST_ASSERT_TRUE(o.setpoint[0] <= 5.0f + 1e-3f);
}

// Closed loop: the vehicle lags the setpoint by lagM (wind offset). The descent must start within 3 s of arrival.
static void closedLoop(float lagM) {
  RthTranslationState s = {{4, 0, 3}, 0.0f};
  const float ret[3] = {0, 0, 3};
  RthHandoverRule rule;
  const float dt = 0.02f;
  int64_t t = 0, arrivedAt = -1;
  RthHandover h = RthHandover::None;
  for (int k = 0; k < 20000 && h == RthHandover::None; k++) {
    RthTranslationOut o = rthTranslationStep(s, ret, dt);
    for (int i = 0; i < 3; i++) {
      TEST_ASSERT_TRUE(isfinite(o.setpoint[i]));
      TEST_ASSERT_TRUE(isfinite(o.velSetpoint[i]));
      s.setpoint[i] = o.setpoint[i];
    }
    s.velocity = o.velocity;
    const float spDist = fabsf(ret[0] - o.setpoint[0]);
    if (spDist < kRthSpArrivedM && arrivedAt < 0) arrivedAt = t;
    // vehicle: setpoint minus a lag, never closer to the return point than lagM
    const float vehX = fminf(o.setpoint[0] + lagM, 1e9f);
    t += 20000000LL;
    h = rule.update(t, spDist, fabsf(ret[0] - vehX));
  }
  TEST_ASSERT_TRUE(h != RthHandover::None);
  TEST_ASSERT_TRUE(arrivedAt >= 0);
  TEST_ASSERT_TRUE(t - arrivedAt <= 3200000000LL + 100000000LL);
}

static void test_wind_lag_06() { closedLoop(0.6f); }
static void test_wind_lag_14() { closedLoop(1.4f); }

static void test_no_handover_beyond_wind_radius() {
  RthHandoverRule r;
  int64_t t = 0;
  for (int k = 0; k < 1000; k++, t += 20000000LL)
    TEST_ASSERT_TRUE(r.update(t, 0.0f, 1.6f) == RthHandover::None);
  TEST_ASSERT_TRUE(r.update(t, 5.0f, 0.4f) == RthHandover::VehicleNear);
}

static void test_sanitiser() {
  SetpointSanitiser s;
  float out[6];
  const float pos[3] = {7, 8, 9};
  const float nanv[6] = {NAN, 0, 0, 1, 2, 3};
  // first tick NaN: zero velocity, hold current position
  TEST_ASSERT_EQUAL_UINT8(SetpointSanitiser::kBadVelocity, s.sanitise(nanv, pos, out));
  for (int i = 0; i < 3; i++) TEST_ASSERT_EQUAL_FLOAT(0.0f, out[i]);
  TEST_ASSERT_EQUAL_FLOAT(7, out[3]); TEST_ASSERT_EQUAL_FLOAT(8, out[4]); TEST_ASSERT_EQUAL_FLOAT(9, out[5]);
  // finite passes through unchanged
  const float good[6] = {0.1f, 0.2f, 0.3f, 4, 5, 6};
  TEST_ASSERT_EQUAL_UINT8(0, s.sanitise(good, pos, out));
  for (int i = 0; i < 6; i++) TEST_ASSERT_EQUAL_FLOAT(good[i], out[i]);
  // NaN afterwards: hold last finite position, velocity zero
  const float bad[6] = {1, 1, 1, NAN, INFINITY, 0};
  TEST_ASSERT_EQUAL_UINT8(SetpointSanitiser::kBadPosition, s.sanitise(bad, pos, out));
  for (int i = 0; i < 3; i++) TEST_ASSERT_EQUAL_FLOAT(0.0f, out[i]);
  TEST_ASSERT_EQUAL_FLOAT(4, out[3]); TEST_ASSERT_EQUAL_FLOAT(5, out[4]); TEST_ASSERT_EQUAL_FLOAT(6, out[5]);
  TEST_ASSERT_EQUAL_UINT32(2, s.count());
}

static void test_log_rate_limit() {
  SetpointSanitiser s;
  TEST_ASSERT_TRUE(s.takeLogSlot(0));
  TEST_ASSERT_FALSE(s.takeLogSlot(500000000LL));
  TEST_ASSERT_TRUE(s.takeLogSlot(1000000000LL));
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_arrival_mag_zero);
  RUN_TEST(test_equals_old_formulas);
  RUN_TEST(test_sweep_always_finite);
  RUN_TEST(test_dt_zero_and_huge);
  RUN_TEST(test_wind_lag_06);
  RUN_TEST(test_wind_lag_14);
  RUN_TEST(test_no_handover_beyond_wind_radius);
  RUN_TEST(test_sanitiser);
  RUN_TEST(test_log_rate_limit);
  return UNITY_END();
}
