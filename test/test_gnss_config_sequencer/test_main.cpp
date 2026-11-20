// Host tests of src/gnss_config_sequencer.cpp: pio test -e native
// The u-blox configuration sequence of UbloxSerialGNSS: step order (GPS + Galileo in one CFG-GNSS write, then the
// read-back of CFG-GNSS, MON-GNSS and CFG-RATE), the 1 s wait after the constellation change as a timed state (no
// blocking), baud fallback, retries, skip-and-continue, Failed states, and the pure UBX payload helpers.
#include <unity.h>

#include "gnss_config_sequencer.hpp"

using namespace VCTR::SNSR;
using A = GnssConfigSequencer::Action;

static constexpr int64_t MS = 1000000LL;

void setUp() {}
void tearDown() {}

static const A kConfigOrder[] = {A::SetUart1Ubx,       A::SetNavRate10Hz, A::SetAutoPvt,
                                 A::SetDynModelAirborne2g, A::PollGnssForWrite, A::SetGnssGpsGalileo, A::SaveConfig,
                                 A::ReadGnssConfig,    A::ReadGnssInUse,  A::ReadNavRate};
static constexpr int kN = sizeof(kConfigOrder) / sizeof(kConfigOrder[0]);

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

  // begin (115200), setSerialRate, reopen, then the configuration steps.
  TEST_ASSERT_EQUAL_INT(3 + kN, d.n);
  TEST_ASSERT_EQUAL(A::Begin, d.log[0]);
  TEST_ASSERT_EQUAL(A::SetSerialRate, d.log[1]);
  TEST_ASSERT_EQUAL(A::Reopen, d.log[2]);
  for (int i = 0; i < kN; i++)
    TEST_ASSERT_EQUAL(kConfigOrder[i], d.log[3 + i]);

  // Save follows the constellation write at once; the read-back comes at least 1 s after the save (GNSS restart).
  const int save = 3 + 6, readBack = 3 + 7;
  TEST_ASSERT_TRUE(d.at[save] - d.at[save - 1] <= (d.stepMs + 2) * MS);
  const int64_t gap = d.at[readBack] - (d.at[save] + d.stepMs * MS);
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
  for (int i = 0; i < 7; i++) {
    TEST_ASSERT_EQUAL(kConfigOrder[i], s.next(3));
    s.report(true, 3);
  }
  // In the 1 s wait after the save: nothing to do.
  TEST_ASSERT_EQUAL(A::None, s.next(3));
  TEST_ASSERT_EQUAL(A::None, s.next(3 + 999 * MS));
  for (int i = 7; i < kN; i++) {
    TEST_ASSERT_EQUAL(kConfigOrder[i], s.next(3 + 1001 * MS));
    s.report(true, 3 + 1001 * MS);
  }
  TEST_ASSERT_TRUE(s.done());
  TEST_ASSERT_EQUAL(A::None, s.next(3 + 2000 * MS));
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

static bool failGalileoTwice(A a, int k, uint32_t) { return a == A::SetGnssGpsGalileo && k < 2; }

static void test_step_retry_succeeds() {
  Driver d;
  d.fail = failGalileoTwice;
  d.run(10000);
  TEST_ASSERT_TRUE(d.seq.done());
  TEST_ASSERT_EQUAL_UINT8(0, d.seq.skippedSteps());
  TEST_ASSERT_EQUAL_INT(3 + kN + 2, d.n);
}

static bool failNavRateAlways(A a, int, uint32_t) { return a == A::SetNavRate10Hz; }

static void test_step_skipped_after_max_attempts() {
  Driver d;
  d.fail = failNavRateAlways;
  d.run(10000);
  TEST_ASSERT_TRUE(d.seq.done());
  TEST_ASSERT_EQUAL_UINT8(1, d.seq.skippedSteps());
  TEST_ASSERT_EQUAL_INT(3 + kN + GnssConfigSequencer::kMaxAttempts - 1, d.n);
  TEST_ASSERT_EQUAL(A::SetAutoPvt, d.log[3 + 1 + GnssConfigSequencer::kMaxAttempts]);
}

static void test_step_max_wait_below_sensor_timeout() {
  // One bounded step (ACK wait) must stay well below the 0.5 s sensor data timeout of the safety task.
  TEST_ASSERT_TRUE(GnssConfigSequencer::kStepMaxWaitMs < 500);
}

static bool failMonGnssAlways(A a, int, uint32_t) { return a == A::ReadGnssInUse; }

static void test_readback_failure_is_skipped_not_fatal() {
  // A receiver without MON-GNSS (or a lost reply) must not stop PVT reading: skipped, sequence done.
  Driver d;
  d.fail = failMonGnssAlways;
  d.run(10000);
  TEST_ASSERT_TRUE(d.seq.done());
  TEST_ASSERT_EQUAL_UINT8(1, d.seq.skippedSteps());
  TEST_ASSERT_EQUAL(A::ReadNavRate, d.log[d.n - 1]);
}

// ---- UBX payload helpers ----------------------------------------------------------------------------------------

/// A u-blox M8 CFG-GNSS poll reply as shipped (GPS, SBAS, QZSS, GLONASS on; Galileo, BeiDou, IMES off).
static uint16_t m8Default(uint8_t *p) {
  const uint8_t blocks[7][8] = {
      {0, 8, 16, 0, 0x01, 0x00, 0x01, 0x01}, // GPS on, L1C/A
      {1, 1, 3, 0, 0x01, 0x00, 0x01, 0x01},  // SBAS on
      {2, 4, 8, 0, 0x00, 0x00, 0x01, 0x01},  // Galileo off, E1OS
      {3, 8, 16, 0, 0x00, 0x00, 0x01, 0x01}, // BeiDou off
      {4, 0, 8, 0, 0x00, 0x00, 0x01, 0x03},  // IMES off
      {5, 0, 3, 0, 0x01, 0x00, 0x01, 0x05},  // QZSS on
      {6, 8, 14, 0, 0x01, 0x00, 0x01, 0x01}, // GLONASS on
  };
  p[0] = 0;
  p[1] = 32;
  p[2] = 32;
  p[3] = 7;
  for (int b = 0; b < 7; b++)
    for (int i = 0; i < 8; i++)
      p[4 + 8 * b + i] = blocks[b][i];
  return 4 + 8 * 7;
}

static void test_cfg_gnss_decode_and_apply() {
  uint8_t p[64];
  const uint16_t len = m8Default(p);
  GnssConstellations c;
  TEST_ASSERT_TRUE(decodeCfgGnss(p, len, c));
  TEST_ASSERT_TRUE(c.gps && c.sbas && c.qzss && c.glonass);
  TEST_ASSERT_FALSE(c.galileo || c.beidou || c.imes);
  TEST_ASSERT_FALSE(constellationsAsIntended(c));
  TEST_ASSERT_EQUAL_UINT8(32, c.numTrkChUse);

  uint8_t before[64];
  for (int i = 0; i < len; i++)
    before[i] = p[i];
  TEST_ASSERT_TRUE(applyGpsGalileo(p, len));
  TEST_ASSERT_TRUE(decodeCfgGnss(p, len, c));
  TEST_ASSERT_TRUE(constellationsAsIntended(c));
  TEST_ASSERT_TRUE(c.gps && c.galileo && c.qzss);
  TEST_ASSERT_FALSE(c.sbas || c.beidou || c.imes || c.glonass);
  // Only the enable bits changed: header, channel counts, sigCfgMask and the other flag bytes untouched.
  for (int i = 0; i < len; i++) {
    const bool flagByte0 = i >= 4 && ((i - 4) % 8) == 4;
    if (!flagByte0)
      TEST_ASSERT_EQUAL_UINT8(before[i], p[i]);
    else
      TEST_ASSERT_EQUAL_UINT8(before[i] & 0xFE, p[i] & 0xFE);
  }
}

static void test_cfg_gnss_apply_sets_empty_signal_mask_and_rejects_bad() {
  uint8_t p[64];
  uint16_t len = m8Default(p);
  p[4 + 8 * 2 + 6] = 0; // Galileo sigCfgMask empty
  TEST_ASSERT_TRUE(applyGpsGalileo(p, len));
  TEST_ASSERT_EQUAL_UINT8(0x01, p[4 + 8 * 2 + 6]);
  // Truncated payload, missing Galileo block, null.
  TEST_ASSERT_FALSE(applyGpsGalileo(p, len - 1));
  len = m8Default(p);
  p[3] = 2; // only GPS and SBAS blocks
  TEST_ASSERT_FALSE(applyGpsGalileo(p, 4 + 16));
  TEST_ASSERT_FALSE(applyGpsGalileo(nullptr, 60));
  GnssConstellations c;
  TEST_ASSERT_FALSE(decodeCfgGnss(p, 3, c));
  TEST_ASSERT_FALSE(c.valid);
}

static void test_intended_set() {
  GnssConstellations c;
  c.valid = c.gps = c.galileo = true;
  TEST_ASSERT_TRUE(constellationsAsIntended(c));
  c.qzss = true; // follows GPS, allowed
  TEST_ASSERT_TRUE(constellationsAsIntended(c));
  c.glonass = true;
  TEST_ASSERT_FALSE(constellationsAsIntended(c));
  c.glonass = false;
  c.galileo = false;
  TEST_ASSERT_FALSE(constellationsAsIntended(c));
  c.galileo = true;
  c.valid = false;
  TEST_ASSERT_FALSE(constellationsAsIntended(c));
}

static void test_mon_gnss_and_cfg_rate_decode() {
  const uint8_t mon[8] = {0, 0x0F, 0x03, 0x09, 3, 0, 0, 0}; // supported all four, enabled GPS + Galileo
  GnssInUse u;
  TEST_ASSERT_TRUE(decodeMonGnss(mon, 8, u));
  TEST_ASSERT_EQUAL_UINT8(0x0F, u.supported);
  TEST_ASSERT_EQUAL_UINT8(0x09, u.enabled);
  TEST_ASSERT_FALSE(decodeMonGnss(mon, 4, u));

  const uint8_t rate[6] = {100, 0, 1, 0, 1, 0}; // 100 ms, 1 cycle, GPS time
  GnssNavRate r;
  TEST_ASSERT_TRUE(decodeCfgRate(rate, 6, r));
  TEST_ASSERT_EQUAL_UINT16(100, r.measRateMs);
  TEST_ASSERT_EQUAL_UINT16(1, r.navRateCycles);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 10.0f, r.hz());
  TEST_ASSERT_FALSE(decodeCfgRate(rate, 5, r));
  TEST_ASSERT_EQUAL_FLOAT(0.0f, r.hz());
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
  RUN_TEST(test_readback_failure_is_skipped_not_fatal);
  RUN_TEST(test_cfg_gnss_decode_and_apply);
  RUN_TEST(test_cfg_gnss_apply_sets_empty_signal_mask_and_rejects_bad);
  RUN_TEST(test_intended_set);
  RUN_TEST(test_mon_gnss_and_cfg_rate_decode);
  return UNITY_END();
}
