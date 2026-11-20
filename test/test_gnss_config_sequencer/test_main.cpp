// Host tests of src/gnss_config_sequencer.cpp: pio test -e native
// The u-blox configuration sequence of UbloxSerialGNSS: same steps and order as the old blocking taskInit, the 1 s
// wait before GPS off as a timed state (no blocking), baud fallback, retries, skip-and-continue, Failed states.
#include <unity.h>

#include "gnss_config_sequencer.hpp"

using namespace VCTR::SNSR;
using A = GnssConfigSequencer::Action;

static constexpr int64_t MS = 1000000LL;

void setUp() {}
void tearDown() {}

static const A kConfigOrder[] = {A::SetUart1Ubx,    A::SetNavRate10Hz, A::SetAutoPvt,  A::SetDynModelAirborne2g,
                                 A::DisableGlonass, A::EnableGalileo,  A::DisableBeidou, A::DisableSbas,
                                 A::DisableQzss,    A::DisableImes,    A::DisableGps,  A::SaveConfig};

/// Drives the sequencer with a 1 ms task, every action succeeding in `stepMs` ms. Records the actions in order.
struct Driver {
  GnssConfigSequencer seq;
  int64_t now = 5000 * MS; // first run of the GNSS task at t = 5 s
  A log[64];
  int64_t at[64];
  int n = 0;
  int64_t stepMs = 0;

  // Optional failure injection: fail(action, attemptIndex) -> true to fail.
  bool (*fail)(A, int, uint32_t) = nullptr;
  int attemptsOf[32] = {0};

  void run(int64_t untilMs) {
    const int64_t until = now + untilMs * MS;
    while (now < until && !seq.done() && !seq.failed()) {
      const A a = seq.next(now);
      if (a != A::None) {
        TEST_ASSERT_TRUE(n < 64);
        log[n] = a;
        at[n] = now;
        n++;
        const int k = attemptsOf[(int)a]++;
        const bool ok = fail == nullptr || !fail(a, k, seq.baud());
        now += stepMs * MS; // time the (bounded) library call takes
        seq.report(ok, now);
      }
      now += 1 * MS; // next task run
    }
  }
};

static void test_happy_path_order_and_gps_wait() {
  Driver d;
  d.stepMs = 20;
  d.run(10000);
  TEST_ASSERT_TRUE(d.seq.done());
  TEST_ASSERT_FALSE(d.seq.failed());
  TEST_ASSERT_EQUAL_UINT8(0, d.seq.skippedSteps());

  // begin (115200), setSerialRate, reopen, then the configuration steps in the old order.
  TEST_ASSERT_EQUAL_INT(3 + 12, d.n);
  TEST_ASSERT_EQUAL(A::Begin, d.log[0]);
  TEST_ASSERT_EQUAL(A::SetSerialRate, d.log[1]);
  TEST_ASSERT_EQUAL(A::Reopen, d.log[2]);
  for (int i = 0; i < 12; i++)
    TEST_ASSERT_EQUAL(kConfigOrder[i], d.log[3 + i]);

  // GPS off comes at least 1 s after IMES off finished (was delay(1000)), and not much later.
  const int imes = 3 + 9, gps = 3 + 10;
  const int64_t gap = d.at[gps] - (d.at[imes] + d.stepMs * MS);
  TEST_ASSERT_TRUE(gap >= 1000 * MS);
  TEST_ASSERT_TRUE(gap <= 1002 * MS);

  // No call ever waits: the sequencer only hands out one action per task run.
  for (int i = 1; i < d.n; i++)
    TEST_ASSERT_TRUE(d.at[i] > d.at[i - 1]);
}

static void test_waiting_returns_none_and_is_idempotent() {
  GnssConfigSequencer s;
  TEST_ASSERT_EQUAL(A::Begin, s.next(0));
  TEST_ASSERT_EQUAL(A::Begin, s.next(0)); // same action until reported
  TEST_ASSERT_EQUAL_UINT32(115200, s.baud());
  s.report(true, 0);
  TEST_ASSERT_EQUAL(A::SetSerialRate, s.next(1));
  s.report(false, 1); // ignored
  TEST_ASSERT_EQUAL(A::Reopen, s.next(2));
  s.report(true, 2);
  for (int i = 0; i < 10; i++) {
    TEST_ASSERT_EQUAL(kConfigOrder[i], s.next(3));
    s.report(true, 3);
  }
  // In the 1 s wait: nothing to do.
  TEST_ASSERT_EQUAL(A::None, s.next(3));
  TEST_ASSERT_EQUAL(A::None, s.next(3 + 999 * MS));
  TEST_ASSERT_EQUAL(A::DisableGps, s.next(3 + 1000 * MS));
}

static bool failBauds(A a, int, uint32_t baud) { return a == A::Begin && baud != 38400; }

static void test_baud_fallback() {
  Driver d;
  d.fail = failBauds;
  d.run(10000);
  TEST_ASSERT_TRUE(d.seq.done());
  TEST_ASSERT_EQUAL(A::Begin, d.log[0]);
  TEST_ASSERT_EQUAL(A::Begin, d.log[1]);
  TEST_ASSERT_EQUAL(A::Begin, d.log[2]);
  TEST_ASSERT_EQUAL(A::SetSerialRate, d.log[3]);
  TEST_ASSERT_EQUAL(A::Reopen, d.log[4]);
}

static bool failAllBegins(A a, int, uint32_t) { return a == A::Begin; }

static void test_no_module_fails_after_three_bauds() {
  Driver d;
  d.fail = failAllBegins;
  d.run(10000);
  TEST_ASSERT_TRUE(d.seq.failed());
  TEST_ASSERT_FALSE(d.seq.done());
  TEST_ASSERT_EQUAL_INT(3, d.n);
  TEST_ASSERT_EQUAL(A::None, d.seq.next(d.now + 100000 * MS));
}

static bool failReopen(A a, int, uint32_t) { return a == A::Reopen; }

static void test_reopen_retries_then_fails() {
  Driver d;
  d.fail = failReopen;
  d.run(10000);
  TEST_ASSERT_TRUE(d.seq.failed());
  TEST_ASSERT_EQUAL_INT(2 + GnssConfigSequencer::kMaxAttempts, d.n);
  // Retries are spaced by kRetryWaitNs.
  TEST_ASSERT_TRUE(d.at[3] - d.at[2] >= GnssConfigSequencer::kRetryWaitNs);
}

static bool failGalileoTwice(A a, int k, uint32_t) { return a == A::EnableGalileo && k < 2; }

static void test_step_retry_succeeds() {
  Driver d;
  d.fail = failGalileoTwice;
  d.run(10000);
  TEST_ASSERT_TRUE(d.seq.done());
  TEST_ASSERT_EQUAL_UINT8(0, d.seq.skippedSteps());
  TEST_ASSERT_EQUAL_INT(3 + 12 + 2, d.n);
}

static bool failNavRateAlways(A a, int, uint32_t) { return a == A::SetNavRate10Hz; }

static void test_step_skipped_after_max_attempts() {
  Driver d;
  d.fail = failNavRateAlways;
  d.run(10000);
  TEST_ASSERT_TRUE(d.seq.done());
  TEST_ASSERT_EQUAL_UINT8(1, d.seq.skippedSteps());
  TEST_ASSERT_EQUAL_INT(3 + 12 + GnssConfigSequencer::kMaxAttempts - 1, d.n);
  TEST_ASSERT_EQUAL(A::SetAutoPvt, d.log[3 + 1 + GnssConfigSequencer::kMaxAttempts]);
}

static void test_step_max_wait_below_sensor_timeout() {
  // One bounded step (ACK wait) must stay well below the 0.5 s sensor data timeout of the safety task.
  TEST_ASSERT_TRUE(GnssConfigSequencer::kStepMaxWaitMs < 500);
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_happy_path_order_and_gps_wait);
  RUN_TEST(test_waiting_returns_none_and_is_idempotent);
  RUN_TEST(test_baud_fallback);
  RUN_TEST(test_no_module_fails_after_three_bauds);
  RUN_TEST(test_reopen_retries_then_fails);
  RUN_TEST(test_step_retry_succeeds);
  RUN_TEST(test_step_skipped_after_max_attempts);
  RUN_TEST(test_step_max_wait_below_sensor_timeout);
  return UNITY_END();
}
