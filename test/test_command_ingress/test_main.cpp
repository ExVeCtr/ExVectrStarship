// Host tests of src/safety/command_ingress.cpp (GS-control cutover, replaces test_control_source):
// pio test -e native -f test_command_ingress
// Covers the single source (node 0), the session latch before the sequence filter (two sessions, spoofer, restart,
// the Failure exception, the 1 s silence), per-link copies deduped, per-link freshness, strict-field rejection
// counting, MULTI_MASTER, REQUEST session acceptance, the control failsafe predicate (committed, timing, D2-B,
// rejected frames never refresh) and the seqlock snapshot.
#include <unity.h>

#include <atomic>
#include <thread>

#include "safety/command_ingress.hpp"

using namespace VCTR::Safety;

static constexpr int64_t MS = 1000000LL;
static constexpr int64_t S = 1000 * MS;
static constexpr uint32_t A = 0xAAAA0001u, B = 0xBBBB0002u;
static CommandIngress *t = nullptr;
static uint8_t seq;

void setUp() {
  t = new CommandIngress();
  seq = 0;
}
void tearDown() { delete t; }

static CmdFrame frame(uint32_t session = A) {
  CmdFrame f;
  f.session = session;
  return f;
}
/// A new frame (new sequence number) on a link.
static CommandIngress::Result send(int64_t now, uint32_t session = A, CmdLink l = CmdLink::Uart, uint8_t src = 0) {
  return t->onFrame(src, l, ++seq, frame(session), now);
}
/// The copy of the last frame on the other link (same sequence number).
static CommandIngress::Result copy(int64_t now, uint32_t session = A, CmdLink l = CmdLink::Usb) {
  return t->onFrame(0, l, seq, frame(session), now);
}

void test_only_node0_is_accepted() {
  TEST_ASSERT_FALSE(send(1 * S, A, CmdLink::Uart, 2).accepted); // the Router / Pocket
  TEST_ASSERT_FALSE(send(1 * S, A, CmdLink::Usb, 3).accepted);
  TEST_ASSERT_FALSE(send(1 * S, A, CmdLink::Uart, 1).accepted);
  TEST_ASSERT_EQUAL_UINT8(3, t->rejectCounts()[1]);
  TEST_ASSERT_FALSE(t->sessionLatched()); // a wrong source never latches a session
  TEST_ASSERT_TRUE(send(1 * S).accepted);
  TEST_ASSERT_TRUE(send(1 * S + 40 * MS, A, CmdLink::Usb).accepted); // node 0 on either link
}

void test_first_session_latches_and_reports_the_change_once() {
  const auto r = send(1 * S);
  TEST_ASSERT_TRUE(r.accepted);
  TEST_ASSERT_TRUE(r.sessionChanged);
  uint32_t s = 0;
  TEST_ASSERT_TRUE(t->takeSessionChange(s));
  TEST_ASSERT_EQUAL_HEX32(A, s);
  TEST_ASSERT_FALSE(t->takeSessionChange(s));
  const auto r2 = send(1 * S + 40 * MS);
  TEST_ASSERT_TRUE(r2.accepted);
  TEST_ASSERT_FALSE(r2.sessionChanged);
}

void test_malformed_counted_and_never_latches() {
  t->onMalformed();
  t->onMalformed();
  TEST_ASSERT_EQUAL_UINT8(2, t->rejectCounts()[0]);
  TEST_ASSERT_FALSE(t->sessionLatched());
  TEST_ASSERT_EQUAL_UINT8(2, t->live().rejectedTotal);
}

void test_per_link_copies_are_deduped() {
  TEST_ASSERT_TRUE(send(1 * S, A, CmdLink::Uart).accepted);
  TEST_ASSERT_FALSE(copy(1 * S + 2 * MS, A, CmdLink::Usb).accepted); // second copy: duplicate
  TEST_ASSERT_EQUAL_UINT8(0, t->rejectCounts()[3]); // a duplicate is not a reject
  TEST_ASSERT_EQUAL_UINT8(0, t->live().rejectedTotal);
  TEST_ASSERT_EQUAL_UINT8(1, t->live().duplicates);
  TEST_ASSERT_EQUAL_UINT32(1, t->live().accepted);
  // the first copy may arrive on either link
  TEST_ASSERT_TRUE(send(1 * S + 40 * MS, A, CmdLink::Usb).accepted);
  TEST_ASSERT_FALSE(copy(1 * S + 41 * MS, A, CmdLink::Uart).accepted);
  TEST_ASSERT_EQUAL_UINT32(2, t->live().accepted);
}

void test_duplicates_refresh_the_link_age_not_the_state_age() {
  send(1 * S, A, CmdLink::Uart);
  copy(1 * S + 2 * MS, A, CmdLink::Usb);
  const CmdIngressSnapshot &s = t->live();
  TEST_ASSERT_EQUAL_INT64(500 * MS, s.ageNs(1 * S + 500 * MS));
  TEST_ASSERT_EQUAL_INT64(500 * MS, s.linkAgeNs(CmdLink::Uart, 1 * S + 500 * MS));
  TEST_ASSERT_EQUAL_INT64(498 * MS, s.linkAgeNs(CmdLink::Usb, 1 * S + 500 * MS));
}

void test_per_link_freshness_is_information_only() {
  send(1 * S, A, CmdLink::Uart);
  copy(1 * S, A, CmdLink::Usb);
  // only the UART keeps delivering
  for (int i = 1; i <= 100; i++) send(1 * S + i * 40 * MS, A, CmdLink::Uart);
  const int64_t now = 1 * S + 100 * 40 * MS;
  const CmdIngressSnapshot &s = t->live();
  TEST_ASSERT_TRUE(s.linkFresh(CmdLink::Uart, now, CommandIngress::kCtrlTimeoutNs));
  TEST_ASSERT_FALSE(s.linkFresh(CmdLink::Usb, now, CommandIngress::kCtrlTimeoutNs));
  TEST_ASSERT_TRUE(s.fresh(now, CommandIngress::kCtrlTimeoutNs));
  TEST_ASSERT_FALSE(t->failsafeDue(now, true)); // one dead link changes nothing
}

void test_late_radio_copy_refreshes_the_uart_link_age() {
  // USB copy leads, the UART copy arrives 2 frames (80 ms) later: stale for acceptance, but the UART link is alive.
  t->onFrame(0, CmdLink::Usb, 10, frame(), 1 * S);
  t->onFrame(0, CmdLink::Usb, 11, frame(), 1 * S + 40 * MS);
  t->onFrame(0, CmdLink::Usb, 12, frame(), 1 * S + 80 * MS);
  TEST_ASSERT_FALSE(t->onFrame(0, CmdLink::Uart, 10, frame(), 1 * S + 90 * MS).accepted);
  const CmdIngressSnapshot &s = t->live();
  TEST_ASSERT_TRUE(s.linkSeen[static_cast<size_t>(CmdLink::Uart)]);
  TEST_ASSERT_EQUAL_INT64(10 * MS, s.linkAgeNs(CmdLink::Uart, 1 * S + 100 * MS));
  TEST_ASSERT_EQUAL_UINT8(0, s.rejects[3]); // a late copy is not a reject
  TEST_ASSERT_EQUAL_UINT8(1, s.lateCopies);
  // A copy far behind (replay) does not refresh the link.
  t->onFrame(0, CmdLink::Usb, 100, frame(), 1 * S + 120 * MS);
  t->onFrame(0, CmdLink::Uart, 50, frame(), 1 * S + 130 * MS);
  TEST_ASSERT_EQUAL_INT64(50 * MS, t->live().linkAgeNs(CmdLink::Uart, 1 * S + 140 * MS)); // still the 1.090 s copy
  TEST_ASSERT_EQUAL_UINT8(1, t->live().rejects[3]); // 50 frames old: a genuine stale frame counts
}

void test_sequence_stale_rejected_and_wraps() {
  t->onFrame(0, CmdLink::Uart, 250, frame(), 1 * S);
  TEST_ASSERT_FALSE(t->onFrame(0, CmdLink::Uart, 249, frame(), 1 * S + 10 * MS).accepted); // older
  TEST_ASSERT_TRUE(t->onFrame(0, CmdLink::Uart, 3, frame(), 1 * S + 20 * MS).accepted);    // wrap: +9
  TEST_ASSERT_FALSE(t->onFrame(0, CmdLink::Uart, 200, frame(), 1 * S + 30 * MS).accepted); // 197 ahead = behind
  TEST_ASSERT_EQUAL_UINT8(1, t->rejectCounts()[3]); // 249 is a late copy (not counted), 200 is stale
}

void test_sequence_resync_after_silence() {
  t->onFrame(0, CmdLink::Uart, 100, frame(), 1 * S);
  TEST_ASSERT_FALSE(t->onFrame(0, CmdLink::Uart, 50, frame(), 1 * S + 900 * MS).accepted);
  TEST_ASSERT_TRUE(t->onFrame(0, CmdLink::Uart, 50, frame(), 2 * S + 1).accepted); // silent for over 1 s
}

void test_session_survives_its_own_resync_without_changing() {
  send(1 * S);
  const auto r = t->onFrame(0, CmdLink::Uart, 7, frame(), 3 * S); // restarted sequence of the SAME session
  TEST_ASSERT_TRUE(r.accepted);
  TEST_ASSERT_FALSE(r.sessionChanged);
}

// ---- session latch ----

void test_foreign_session_rejected_while_the_latched_one_is_alive() {
  send(1 * S);
  const auto r = t->onFrame(0, CmdLink::Uart, 5, frame(B), 1 * S + 20 * MS);
  TEST_ASSERT_FALSE(r.accepted);
  TEST_ASSERT_FALSE(r.sessionChanged);
  TEST_ASSERT_EQUAL_UINT8(1, t->rejectCounts()[2]);
  TEST_ASSERT_EQUAL_HEX32(A, t->session());
  TEST_ASSERT_TRUE(t->live().multiMaster(1 * S + 20 * MS, CommandIngress::kMultiMasterShowNs));
  TEST_ASSERT_FALSE(t->live().multiMaster(1 * S + 2 * S + 21 * MS, CommandIngress::kMultiMasterShowNs));
}

void test_spoofer_interleaving_never_touches_the_masters_sequence() {
  // the master streams at 25 Hz, a spoofer sends session B with colliding sequence numbers in between
  int rejected = 0, accepted = 0;
  for (int i = 0; i < 200; i++) {
    const int64_t now = 1 * S + i * 40 * MS;
    if (send(now).accepted) accepted++;
    if (!t->onFrame(0, CmdLink::Uart, seq, frame(B), now + 5 * MS).accepted) rejected++;
    if (!t->onFrame(0, CmdLink::Uart, static_cast<uint8_t>(seq + 1), frame(B), now + 6 * MS).accepted) rejected++;
  }
  TEST_ASSERT_EQUAL_INT(200, accepted); // the master never lost a frame to the spoofer
  TEST_ASSERT_EQUAL_INT(400, rejected);
  TEST_ASSERT_EQUAL_HEX32(A, t->session());
}

void test_new_session_not_committed_needs_one_second_of_silence() {
  t->update(0, false, false);
  send(1 * S);
  // 0.99 s after the last frame of A: B is still a contender
  TEST_ASSERT_FALSE(t->onFrame(0, CmdLink::Uart, 1, frame(B), 1 * S + 990 * MS).accepted);
  // exactly 1 s: silent, takeover allowed
  const auto r = t->onFrame(0, CmdLink::Uart, 1, frame(B), 1 * S + 1000 * MS);
  TEST_ASSERT_TRUE(r.accepted);
  TEST_ASSERT_TRUE(r.sessionChanged);
  TEST_ASSERT_EQUAL_HEX32(B, t->session());
  // the old session is now the foreign one
  TEST_ASSERT_FALSE(t->onFrame(0, CmdLink::Uart, 9, frame(A), 1 * S + 1040 * MS).accepted);
}

void test_duplicate_copy_of_the_latched_session_keeps_it_alive() {
  t->update(0, false, false);
  send(1 * S, A, CmdLink::Uart);
  copy(1 * S + 900 * MS, A, CmdLink::Usb); // a duplicate copy is still a frame of the session
  TEST_ASSERT_FALSE(t->onFrame(0, CmdLink::Uart, 1, frame(B), 1 * S + 1500 * MS).accepted); // 0.6 s since the copy
  TEST_ASSERT_TRUE(t->onFrame(0, CmdLink::Uart, 1, frame(B), 1 * S + 1900 * MS).accepted);
}

void test_committed_rejects_a_new_session_even_after_silence() {
  send(1 * S);
  t->update(1 * S, true, false); // committed, not in Failure
  TEST_ASSERT_FALSE(t->onFrame(0, CmdLink::Uart, 1, frame(B), 1 * S + 1500 * MS).accepted); // GS restart: no takeover
  TEST_ASSERT_TRUE(t->live().multiMaster(1 * S + 1500 * MS, CommandIngress::kMultiMasterShowNs));
  TEST_ASSERT_EQUAL_HEX32(A, t->session());
  TEST_ASSERT_TRUE(t->live().commitLatched);
}

void test_failure_mode_accepts_a_new_session_after_silence_even_if_committed() {
  send(1 * S);
  t->update(1 * S, true, true); // committed AND mode Failure: the actuators are already cut
  TEST_ASSERT_FALSE(t->onFrame(0, CmdLink::Uart, 1, frame(B), 1 * S + 500 * MS).accepted); // still needs the silence
  const auto r = t->onFrame(0, CmdLink::Uart, 1, frame(B), 3 * S);
  TEST_ASSERT_TRUE(r.accepted);
  TEST_ASSERT_TRUE(r.sessionChanged);
}

void test_gs_restart_after_commit_scenario() {
  // committed flight, the GS dies: no new session is accepted until the failsafe puts the mode in Failure
  t->update(0, true, false);
  send(1 * S);
  for (int i = 1; i <= 25; i++) send(1 * S + i * 40 * MS);
  const int64_t last = 1 * S + 25 * 40 * MS;
  TEST_ASSERT_FALSE(t->onFrame(0, CmdLink::Usb, 1, frame(B), last + 1500 * MS).accepted); // committed: rejected
  TEST_ASSERT_TRUE(t->failsafeDue(last + 2 * S + 1, true));                               // failsafe latches at 2 s
  t->update(last + 2 * S + 1, true, true);                                                 // mode Failure
  TEST_ASSERT_TRUE(t->onFrame(0, CmdLink::Usb, 1, frame(B), last + 2 * S + 40 * MS).accepted); // restarted GS
}

void test_new_session_resets_sequence_and_links() {
  send(1 * S, A, CmdLink::Uart);
  copy(1 * S + 1 * MS, A, CmdLink::Usb);
  t->update(0, false, false);
  const auto r = t->onFrame(0, CmdLink::Uart, 200, frame(B), 3 * S);
  TEST_ASSERT_TRUE(r.accepted);
  TEST_ASSERT_FALSE(t->live().linkSeen[static_cast<size_t>(CmdLink::Usb)]); // the new session has no USB copy yet
  TEST_ASSERT_TRUE(t->onFrame(0, CmdLink::Uart, 201, frame(B), 3 * S + 40 * MS).accepted); // its own sequence
}

void test_boot_restart_of_the_vehicle_latches_whatever_session_arrives() {
  // a rebooted vehicle has no session: the first valid one latches (a restarted GS cannot be told from a fresh one)
  TEST_ASSERT_TRUE(send(1 * S, B).accepted);
  TEST_ASSERT_EQUAL_HEX32(B, t->session());
}

// ---- REQUEST ----

void test_requests_only_from_the_latched_session() {
  TEST_ASSERT_FALSE(t->requestAllowed(A)); // nothing latched
  TEST_ASSERT_FALSE(t->onRequest(0));
  send(1 * S);
  TEST_ASSERT_TRUE(t->requestAllowed(A));
  TEST_ASSERT_FALSE(t->requestAllowed(B));
  TEST_ASSERT_FALSE(t->requestAllowed(0)); // a sender without the field decodes as 0
  TEST_ASSERT_TRUE(t->onRequest(A));
  TEST_ASSERT_FALSE(t->onRequest(B));
  TEST_ASSERT_EQUAL_UINT16(1, t->requestsAccepted());
  TEST_ASSERT_EQUAL_UINT16(2, t->requestsRefused());
}

// ---- control failsafe rule ----

void test_failsafe_predicate() {
  const int64_t T = CommandIngress::kCtrlTimeoutNs;
  const CmdIngressSnapshot none;
  TEST_ASSERT_TRUE(none.failsafeDue(1 * S, true, T));   // committed, no session ever
  TEST_ASSERT_FALSE(none.failsafeDue(1 * S, false, T)); // D2-B
  send(1 * S);
  const CmdIngressSnapshot &s = t->live();
  TEST_ASSERT_FALSE(s.failsafeDue(1 * S + T, true, T));     // exactly 2 s: fresh
  TEST_ASSERT_TRUE(s.failsafeDue(1 * S + T + 1, true, T));  // 1 ns later: due
  TEST_ASSERT_FALSE(s.failsafeDue(1 * S + T + 1, false, T)); // not committed: never
}

void test_failsafe_timing_on_10hz_tick() {
  send(1 * S);
  int due = -1;
  for (int k = 1; k <= 30; k++)
    if (t->failsafeDue(1 * S + k * 100 * MS, true)) {
      due = k;
      break;
    }
  TEST_ASSERT_EQUAL_INT(21, due); // the first tick strictly after 2.0 s
}

void test_failsafe_not_kept_alive_by_rejected_frames() {
  t->update(0, true, false); // committed: a foreign session cannot take over
  send(1 * S);
  for (int i = 1; i <= 60; i++) {
    const int64_t now = 1 * S + i * 40 * MS;
    t->onFrame(2, CmdLink::Uart, ++seq, frame(), now); // wrong source
    t->onFrame(0, CmdLink::Uart, ++seq, frame(B), now); // foreign session
    t->onMalformed();                                    // bad field
  }
  TEST_ASSERT_TRUE(t->failsafeDue(1 * S + 60 * 40 * MS, true)); // 2.4 s since the only accepted frame
}

void test_not_committed_session_gone_is_none_not_failsafe() {
  send(1 * S);
  TEST_ASSERT_FALSE(t->failsafeDue(1 * S + 30 * S, false));
  TEST_ASSERT_FALSE(t->live().fresh(1 * S + 30 * S, CommandIngress::kCtrlTimeoutNs)); // ctrl_source reads NONE
}

void test_failsafe_level_of_the_last_state_is_visible() {
  CmdFrame f = frame();
  f.failsafe = 1;
  t->onFrame(0, CmdLink::Uart, 1, f, 1 * S);
  TEST_ASSERT_EQUAL_UINT8(1, t->lastState()->failsafe);
  f.failsafe = 0;
  t->onFrame(0, CmdLink::Uart, 2, f, 1 * S + 40 * MS);
  TEST_ASSERT_EQUAL_UINT8(0, t->lastState()->failsafe);
}

void test_snapshot_matches_live_and_age_helpers() {
  CmdIngressSnapshot s;
  TEST_ASSERT_TRUE(t->snapshot(s));
  TEST_ASSERT_FALSE(s.latched);
  TEST_ASSERT_EQUAL_INT64(-1, s.ageNs(1 * S));
  send(1 * S);
  TEST_ASSERT_TRUE(t->snapshot(s));
  TEST_ASSERT_TRUE(s.latched);
  TEST_ASSERT_EQUAL_HEX32(A, s.session);
  TEST_ASSERT_EQUAL_INT64(500 * MS, s.ageNs(1 * S + 500 * MS));
  TEST_ASSERT_EQUAL_INT64(-1, s.linkAgeNs(CmdLink::Usb, 1 * S));
}

// The writer publishes a timestamp whose two 32-bit halves are different and a field carrying the same counter;
// a torn read would show a mismatch between them.
void test_snapshot_never_torn_under_concurrent_writes() {
  std::atomic<bool> stop{false};
  std::thread w([&] {
    for (uint32_t k = 1; !stop.load(); k++) {
      CmdFrame f = frame();
      f.profile = static_cast<uint8_t>(k % 3);
      const int64_t ts = (static_cast<int64_t>(k) << 32) | k; // high half == low half == k
      t->onFrame(0, CmdLink::Uart, static_cast<uint8_t>(k), f, ts);
      if ((k & 0x3F) == 0) t->update(ts, false, false);
    }
  });
  int bad = 0, good = 0;
  for (int i = 0; i < 300000; i++) {
    CmdIngressSnapshot s;
    if (!t->snapshot(s)) continue;
    if (!s.seen) continue;
    good++;
    const uint32_t hi = static_cast<uint32_t>(s.lastRxNs >> 32), lo = static_cast<uint32_t>(s.lastRxNs);
    if (hi != lo || s.frame.profile != lo % 3 || s.accepted != lo) bad++;
  }
  stop = true;
  w.join();
  TEST_ASSERT_GREATER_THAN(0, good);
  TEST_ASSERT_EQUAL_INT(0, bad);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_only_node0_is_accepted);
  RUN_TEST(test_first_session_latches_and_reports_the_change_once);
  RUN_TEST(test_malformed_counted_and_never_latches);
  RUN_TEST(test_per_link_copies_are_deduped);
  RUN_TEST(test_duplicates_refresh_the_link_age_not_the_state_age);
  RUN_TEST(test_per_link_freshness_is_information_only);
  RUN_TEST(test_late_radio_copy_refreshes_the_uart_link_age);
  RUN_TEST(test_sequence_stale_rejected_and_wraps);
  RUN_TEST(test_sequence_resync_after_silence);
  RUN_TEST(test_session_survives_its_own_resync_without_changing);
  RUN_TEST(test_foreign_session_rejected_while_the_latched_one_is_alive);
  RUN_TEST(test_spoofer_interleaving_never_touches_the_masters_sequence);
  RUN_TEST(test_new_session_not_committed_needs_one_second_of_silence);
  RUN_TEST(test_duplicate_copy_of_the_latched_session_keeps_it_alive);
  RUN_TEST(test_committed_rejects_a_new_session_even_after_silence);
  RUN_TEST(test_failure_mode_accepts_a_new_session_after_silence_even_if_committed);
  RUN_TEST(test_gs_restart_after_commit_scenario);
  RUN_TEST(test_new_session_resets_sequence_and_links);
  RUN_TEST(test_boot_restart_of_the_vehicle_latches_whatever_session_arrives);
  RUN_TEST(test_requests_only_from_the_latched_session);
  RUN_TEST(test_failsafe_predicate);
  RUN_TEST(test_failsafe_timing_on_10hz_tick);
  RUN_TEST(test_failsafe_not_kept_alive_by_rejected_frames);
  RUN_TEST(test_not_committed_session_gone_is_none_not_failsafe);
  RUN_TEST(test_failsafe_level_of_the_last_state_is_visible);
  RUN_TEST(test_snapshot_matches_live_and_age_helpers);
  RUN_TEST(test_snapshot_never_torn_under_concurrent_writes);
  return UNITY_END();
}
