// Host tests of src/safety/control_source.cpp (T9b-1, T9b-3): pio test -e native -f test_control_source
// Covers the (src, link) table, the sequence rule per source, SOURCE_OK, freshness, radio priority, the 0.5 s
// handover hysteresis, the latching input, the control failsafe predicate (T9b-3: timing, rejected frames, D2-B, ignored source) and the seqlock snapshot.
#include <unity.h>

#include <atomic>
#include <thread>

#include "safety/control_source.hpp"

using namespace VCTR::Safety;

static constexpr int64_t MS = 1000000LL;
static constexpr int64_t S = 1000 * MS;
static ControlSourceTable *t = nullptr;
static uint8_t seqR, seqB;

void setUp() {
  t = new ControlSourceTable();
  seqR = seqB = 0;
}
void tearDown() { delete t; }

static RcFrame okFrame(int16_t ch0 = 0) {
  RcFrame f;
  f.flags = RcFrame::kFlagSourceOk;
  f.ch[0] = ch0;
  return f;
}
static bool radio(int64_t now, RcFrame f = okFrame()) {
  return t->onRcFrame(2, RcLink::Uart, seqR++, f, now);
}
static bool bench(int64_t now, RcFrame f = okFrame()) {
  return t->onRcFrame(0, RcLink::Usb, seqB++, f, now);
}

void test_only_two_keys_accepted() {
  TEST_ASSERT_TRUE(radio(1 * S));
  TEST_ASSERT_TRUE(bench(1 * S));
  TEST_ASSERT_FALSE(t->onRcFrame(0, RcLink::Uart, 0, okFrame(), 1 * S)); // GS RC over the radio path
  TEST_ASSERT_FALSE(t->onRcFrame(2, RcLink::Usb, 0, okFrame(), 1 * S));
  TEST_ASSERT_FALSE(t->onRcFrame(1, RcLink::Usb, 0, okFrame(), 1 * S));
  TEST_ASSERT_FALSE(t->onRcFrame(3, RcLink::Uart, 0, okFrame(), 1 * S));
  TEST_ASSERT_EQUAL_UINT8(4, t->rejectCounts()[1]);
  TEST_ASSERT_EQUAL_UINT8(4, t->live().rejectedTotal);
}

void test_source_ok_clear_rejected_and_does_not_refresh() {
  TEST_ASSERT_TRUE(radio(1 * S));
  RcFrame f = okFrame();
  f.flags = 0;
  TEST_ASSERT_FALSE(radio(2 * S, f));
  TEST_ASSERT_EQUAL_UINT8(1, t->rejectCounts()[3]);
  TEST_ASSERT_EQUAL_INT64(1 * S, t->live().src[0].lastRxNs);
  TEST_ASSERT_EQUAL_UINT32(1, t->live().src[0].accepted);
}

void test_malformed_counted() {
  t->onMalformed();
  t->onMalformed();
  TEST_ASSERT_EQUAL_UINT8(2, t->rejectCounts()[0]);
  TEST_ASSERT_EQUAL_UINT8(2, t->live().rejectedTotal);
}

void test_sequence_per_source_and_stale_rejected() {
  TEST_ASSERT_TRUE(t->onRcFrame(2, RcLink::Uart, 10, okFrame(), 1 * S));
  TEST_ASSERT_FALSE(t->onRcFrame(2, RcLink::Uart, 10, okFrame(), 1 * S + 10 * MS)); // repeat
  TEST_ASSERT_FALSE(t->onRcFrame(2, RcLink::Uart, 9, okFrame(), 1 * S + 20 * MS));  // older
  TEST_ASSERT_TRUE(t->onRcFrame(2, RcLink::Uart, 11, okFrame(), 1 * S + 30 * MS));
  // the bench has its own sequence state: seq 10 is new there
  TEST_ASSERT_TRUE(t->onRcFrame(0, RcLink::Usb, 10, okFrame(), 1 * S + 30 * MS));
  TEST_ASSERT_EQUAL_UINT8(2, t->rejectCounts()[2]);
  // wrap
  TEST_ASSERT_TRUE(t->onRcFrame(2, RcLink::Uart, 138, okFrame(), 1 * S + 40 * MS)); // +127
  TEST_ASSERT_FALSE(t->onRcFrame(2, RcLink::Uart, 11, okFrame(), 1 * S + 50 * MS)); // -127 = 129 ahead: not newer
}

void test_sequence_resync_after_silence() {
  TEST_ASSERT_TRUE(t->onRcFrame(2, RcLink::Uart, 200, okFrame(), 1 * S));
  TEST_ASSERT_FALSE(t->onRcFrame(2, RcLink::Uart, 100, okFrame(), 1 * S + 500 * MS));
  TEST_ASSERT_TRUE(t->onRcFrame(2, RcLink::Uart, 100, okFrame(), 3 * S)); // silent > 1 s: resync (reboot of the Pocket)
}

void test_wrong_link_does_not_touch_sequence_state() {
  TEST_ASSERT_TRUE(t->onRcFrame(0, RcLink::Usb, 50, okFrame(), 1 * S));
  TEST_ASSERT_FALSE(t->onRcFrame(0, RcLink::Uart, 200, okFrame(), 1 * S + 10 * MS)); // rejected by key
  TEST_ASSERT_TRUE(t->onRcFrame(0, RcLink::Usb, 51, okFrame(), 1 * S + 20 * MS));
}

void test_nothing_active_at_boot_then_radio_priority() {
  t->update(0, false);
  TEST_ASSERT_EQUAL((int)ControlSourceId::None, (int)t->active());
  TEST_ASSERT_EQUAL_INT64(-1, t->activeAgeNs(0));
  TEST_ASSERT_NULL(t->lastActiveState());
  bench(1 * S);
  t->update(1 * S, false);
  TEST_ASSERT_EQUAL((int)ControlSourceId::Bench, (int)t->active());
  TEST_ASSERT_TRUE(t->sourceChangedSinceLast());
  TEST_ASSERT_FALSE(t->sourceChangedSinceLast()); // consumed
  radio(1 * S + 10 * MS);
  bench(1 * S + 20 * MS);
  t->update(1 * S + 20 * MS, false);
  // radio just arrived while BENCH is active and fresh: hysteresis, stays BENCH
  TEST_ASSERT_EQUAL((int)ControlSourceId::Bench, (int)t->active());
  TEST_ASSERT_EQUAL_UINT8(3, t->freshMask(1 * S + 20 * MS));
}

void test_radio_first_when_both_present_from_none() {
  radio(1 * S);
  bench(1 * S);
  t->update(1 * S, false);
  TEST_ASSERT_EQUAL((int)ControlSourceId::Radio, (int)t->active());
}

void test_handover_back_to_radio_needs_500ms_continuous() {
  bench(1 * S);
  t->update(1 * S, false);
  TEST_ASSERT_EQUAL((int)ControlSourceId::Bench, (int)t->active());
  int64_t now = 1 * S;
  // radio frames every 40 ms, bench too
  for (int i = 0; i < 12; i++) { // 440 ms of radio
    now += 40 * MS;
    radio(now);
    bench(now);
    t->update(now, false);
    TEST_ASSERT_EQUAL((int)ControlSourceId::Bench, (int)t->active());
  }
  now += 40 * MS; // 480 ms since the first radio state (first was at 1.04 s): run = 480
  radio(now);
  bench(now);
  t->update(now, false);
  TEST_ASSERT_EQUAL((int)ControlSourceId::Bench, (int)t->active());
  now += 40 * MS; // 520 ms
  radio(now);
  bench(now);
  t->update(now, false);
  TEST_ASSERT_EQUAL((int)ControlSourceId::Radio, (int)t->active());
  TEST_ASSERT_TRUE(t->sourceChangedSinceLast());
}

void test_flickering_radio_never_takes_over_from_bench() {
  bench(1 * S);
  t->update(1 * S, false);
  int64_t now = 1 * S;
  for (int i = 0; i < 40; i++) { // radio bursts of 200 ms, then 400 ms gaps (> continuity gap)
    for (int k = 0; k < 5; k++) {
      now += 40 * MS;
      radio(now);
      bench(now);
      t->update(now, false);
    }
    for (int k = 0; k < 10; k++) {
      now += 40 * MS;
      bench(now);
      t->update(now, false);
    }
    TEST_ASSERT_EQUAL((int)ControlSourceId::Bench, (int)t->active());
  }
}

void test_radio_lost_falls_back_to_bench_then_none() {
  radio(1 * S);
  bench(1 * S);
  t->update(1 * S, false);
  TEST_ASSERT_EQUAL((int)ControlSourceId::Radio, (int)t->active());
  t->sourceChangedSinceLast();
  bench(3 * S);
  t->update(3 * S + 1, false); // radio 2 s + 1 ns old
  TEST_ASSERT_EQUAL((int)ControlSourceId::Bench, (int)t->active());
  TEST_ASSERT_TRUE(t->sourceChangedSinceLast());
  t->update(3 * S, false);
  t->update(5 * S + 1, false); // bench 2 s + 1 ns
  TEST_ASSERT_EQUAL((int)ControlSourceId::None, (int)t->active());
  TEST_ASSERT_TRUE(t->sourceChangedSinceLast());
}

void test_freshness_is_exactly_two_seconds() {
  radio(1 * S);
  t->update(3 * S, false); // exactly 2 s: still fresh
  TEST_ASSERT_EQUAL((int)ControlSourceId::Radio, (int)t->active());
  TEST_ASSERT_EQUAL_INT64(2 * S, t->activeAgeNs(3 * S));
  t->update(3 * S + 1, false);
  TEST_ASSERT_EQUAL((int)ControlSourceId::None, (int)t->active());
}

void test_bench_radio_dead_while_bench_stale_switches_immediately() {
  bench(1 * S);
  t->update(1 * S, false);
  radio(5 * S); // bench is stale now, radio fresh (run only just started): no hysteresis needed
  t->update(5 * S, false);
  TEST_ASSERT_EQUAL((int)ControlSourceId::Radio, (int)t->active());
}

void test_committed_latches_the_active_source() {
  bench(1 * S);
  t->update(1 * S, false);
  TEST_ASSERT_EQUAL((int)ControlSourceId::Bench, (int)t->active());
  // committed: a fresh radio never takes over, even after the hysteresis time
  int64_t now = 1 * S;
  for (int i = 0; i < 50; i++) {
    now += 40 * MS;
    radio(now);
    bench(now);
    t->update(now, true);
  }
  TEST_ASSERT_EQUAL((int)ControlSourceId::Bench, (int)t->active());
  TEST_ASSERT_TRUE(t->live().latched);
  TEST_ASSERT_EQUAL_UINT8(3, t->freshMask(now)); // the ignored source is still shown fresh
  // the latched source going silent does not hand over either
  for (int i = 0; i < 100; i++) {
    now += 40 * MS;
    radio(now);
    t->update(now, true);
  }
  TEST_ASSERT_EQUAL((int)ControlSourceId::Bench, (int)t->active());
  TEST_ASSERT_TRUE(t->failsafeDue(now, true)); // only the latched source's silence counts
  // not committed: arbitration resumes
  t->update(now, false);
  TEST_ASSERT_FALSE(t->live().latched);
  TEST_ASSERT_EQUAL((int)ControlSourceId::Radio, (int)t->active());
}

void test_latch_with_nothing_active_stays_none() {
  t->update(1 * S, true);
  radio(1 * S);
  t->update(1 * S + 10 * MS, true);
  TEST_ASSERT_EQUAL((int)ControlSourceId::None, (int)t->active());
}

void test_failsafe_predicate() {
  radio(1 * S);
  t->update(1 * S, false);
  TEST_ASSERT_FALSE(t->failsafeDue(100 * S, false)); // not committed: never due
  t->update(1 * S, true);
  TEST_ASSERT_FALSE(t->failsafeDue(3 * S, true));
  TEST_ASSERT_TRUE(t->failsafeDue(3 * S + 1, true));
}

// T9b-3 acceptance: 10 Hz safety tick, the failsafe is due 2.0-2.1 s after the last accepted state of the latched source.
void test_failsafe_timing_on_10hz_tick() {
  int64_t last = 0;
  for (int64_t n = 0; n <= 100; n++) { // radio at 25 Hz for 4 s, committed from 1 s
    const int64_t now = n * 40 * MS;
    radio(now);
    last = now;
    t->update(now, now >= 1 * S);
  }
  int64_t dueAt = -1;
  for (int64_t now = last; now <= last + 3 * S; now += 100 * MS) { // radio silent, 10 Hz ticks
    t->update(now, true);
    ControlSourceSnapshot s;
    TEST_ASSERT_TRUE(t->snapshot(s));
    TEST_ASSERT_EQUAL(t->failsafeDue(now, true), s.failsafeDue(now, true, ControlSourceTable::kCtrlTimeoutNs));
    if (t->failsafeDue(now, true)) {
      dueAt = now - last;
      break;
    }
  }
  TEST_ASSERT_TRUE(dueAt > 2 * S && dueAt <= 2 * S + 100 * MS);
}

// Only accepted RC_STATE keeps the latched source alive: rejected frames (SOURCE_OK clear, wrong key, stale
// sequence, bad length) from either source never refresh it.
void test_failsafe_not_kept_alive_by_rejected_frames() {
  radio(1 * S);
  t->update(1 * S, false);
  t->update(1 * S, true);
  TEST_ASSERT_EQUAL((int)ControlSourceId::Radio, (int)t->active());
  RcFrame bad = okFrame(0);
  bad.flags = 0; // SOURCE_OK clear
  uint8_t seq = 100;
  for (int64_t now = 1 * S + 40 * MS; now <= 4 * S; now += 40 * MS) {
    t->onRcFrame(2, RcLink::Uart, seq++, bad, now);        // Pocket gone, module still sends SOURCE_OK clear
    t->onRcFrame(0, RcLink::Uart, seq++, okFrame(0), now); // GS RC on the radio: never accepted
    t->onMalformed();
    t->update(now, true);
  }
  TEST_ASSERT_TRUE(t->failsafeDue(4 * S, true));
}

// D2-B: not committed, every source gone: ctrl_source NONE, no failsafe, whatever the time; the latch only
// matters once committed. A source coming back is picked up again.
void test_not_committed_all_sources_gone_is_none_not_failsafe() {
  radio(1 * S);
  t->update(1 * S, false);
  TEST_ASSERT_EQUAL((int)ControlSourceId::Radio, (int)t->active());
  t->update(10 * S, false);
  TEST_ASSERT_EQUAL((int)ControlSourceId::None, (int)t->active());
  TEST_ASSERT_FALSE(t->failsafeDue(10 * S, false));
  TEST_ASSERT_FALSE(t->failsafeDue(1000 * S, false));
  radio(11 * S);
  t->update(11 * S, false);
  TEST_ASSERT_EQUAL((int)ControlSourceId::Radio, (int)t->active());
}

// Committed with the other (ignored) source fresh: only the latched source's silence counts.
void test_failsafe_ignores_fresh_other_source() {
  radio(1 * S);
  t->update(1 * S, false);
  t->update(1 * S, true);
  for (int64_t now = 1 * S; now <= 4 * S; now += 40 * MS) {
    bench(now);
    t->update(now, true);
  }
  TEST_ASSERT_EQUAL((int)ControlSourceId::Radio, (int)t->active());
  TEST_ASSERT_TRUE(t->failsafeDue(4 * S, true));
}

void test_snapshot_matches_live_and_age_helpers() {
  radio(1 * S, okFrame(123));
  t->update(1 * S, false);
  ControlSourceSnapshot s;
  TEST_ASSERT_TRUE(t->snapshot(s));
  TEST_ASSERT_EQUAL((int)ControlSourceId::Radio, (int)s.active);
  TEST_ASSERT_EQUAL_INT16(123, s.record(ControlSourceId::Radio)->frame.ch[0]);
  TEST_ASSERT_EQUAL_INT64(500 * MS, s.ageNs(ControlSourceId::Radio, 1 * S + 500 * MS));
  TEST_ASSERT_EQUAL_INT64(-1, s.ageNs(ControlSourceId::Bench, 1 * S));
  TEST_ASSERT_NULL(s.record(ControlSourceId::None));
}

// The writer publishes a timestamp whose two 32-bit halves are different and a channel carrying the same counter;
// a torn read would show a mismatch between them.
void test_snapshot_never_torn_under_concurrent_writes() {
  std::atomic<bool> stop{false};
  std::thread w([&] {
    for (uint32_t k = 1; !stop.load(); k++) {
      RcFrame f = okFrame(static_cast<int16_t>(k & 0x7FFF));
      const int64_t ts = (static_cast<int64_t>(k) << 32) | k; // high half == low half == k
      t->onRcFrame(2, RcLink::Uart, static_cast<uint8_t>(k), f, ts);
      if ((k & 0x3F) == 0) t->update(ts, false);
    }
  });
  int bad = 0, good = 0;
  for (int i = 0; i < 300000; i++) {
    ControlSourceSnapshot s;
    if (!t->snapshot(s)) continue;
    const RcSourceRecord &r = s.src[0];
    if (!r.seen) continue;
    good++;
    const uint32_t hi = static_cast<uint32_t>(r.lastRxNs >> 32), lo = static_cast<uint32_t>(r.lastRxNs);
    if (hi != lo || static_cast<int16_t>(lo & 0x7FFF) != r.frame.ch[0] || r.accepted != lo) bad++;
  }
  stop = true;
  w.join();
  TEST_ASSERT_GREATER_THAN(0, good);
  TEST_ASSERT_EQUAL_INT(0, bad);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_only_two_keys_accepted);
  RUN_TEST(test_source_ok_clear_rejected_and_does_not_refresh);
  RUN_TEST(test_malformed_counted);
  RUN_TEST(test_sequence_per_source_and_stale_rejected);
  RUN_TEST(test_sequence_resync_after_silence);
  RUN_TEST(test_wrong_link_does_not_touch_sequence_state);
  RUN_TEST(test_nothing_active_at_boot_then_radio_priority);
  RUN_TEST(test_radio_first_when_both_present_from_none);
  RUN_TEST(test_handover_back_to_radio_needs_500ms_continuous);
  RUN_TEST(test_flickering_radio_never_takes_over_from_bench);
  RUN_TEST(test_radio_lost_falls_back_to_bench_then_none);
  RUN_TEST(test_freshness_is_exactly_two_seconds);
  RUN_TEST(test_bench_radio_dead_while_bench_stale_switches_immediately);
  RUN_TEST(test_committed_latches_the_active_source);
  RUN_TEST(test_latch_with_nothing_active_stays_none);
  RUN_TEST(test_failsafe_predicate);
  RUN_TEST(test_failsafe_timing_on_10hz_tick);
  RUN_TEST(test_failsafe_not_kept_alive_by_rejected_frames);
  RUN_TEST(test_not_committed_all_sources_gone_is_none_not_failsafe);
  RUN_TEST(test_failsafe_ignores_fresh_other_source);
  RUN_TEST(test_snapshot_matches_live_and_age_helpers);
  RUN_TEST(test_snapshot_never_torn_under_concurrent_writes);
  return UNITY_END();
}
