// Host tests of src/safety/motor_interlock.cpp (T9b-2): pio test -e native -f test_motor_interlock
// Fake clock (every call takes the time). Covers invariants I1-I4, I6, I7 and I9 of docs/t9b-design.md 3.4.3 at the
// class level (I5, I7 and I8 end to end through the real command taps: test_vehicle_network), the observed-transition
// rule, D3-B, the D4-A re-arm, the gate AND (shim of StarshipTVC, no simulation input) and the
// watchdog staleness rule.
#include <unity.h>

#include "safety/motor_interlock.hpp"

using namespace VCTR::Safety;
using CR = MotorInterlock::CloseReason;

static constexpr int64_t MS = 1000000LL;
static constexpr int64_t S = 1000 * MS;
static constexpr int64_t PERIOD = 40 * MS; // 25 Hz RC

static MotorInterlock *il = nullptr;
static int64_t now = 0;
static bool sessionStarted = false;

void setUp() {
  il = new MotorInterlock();
  sessionStarted = false;
  now = 100 * S;
}
void tearDown() { delete il; }

/// One tick: the comms task polls (update), then one accepted state of the latched session arrives. The first
/// frame starts the session (the ingress calls onSessionChanged() before the first state of a new session).
static void frame(bool sd) {
  if (!sessionStarted) {
    il->onSessionChanged();
    sessionStarted = true;
  }
  il->update(now);
  il->onCmdState(sd, now);
}
static void step(int64_t dt = PERIOD) { now += dt; }
/// n frames of the same SD value, one per 40 ms.
static void run(bool sd, int n) {
  for (int i = 0; i < n; i++) {
    frame(sd);
    step();
  }
}

void test_boot_is_closed_without_source() {
  TEST_ASSERT_FALSE(il->open(now));
  TEST_ASSERT_EQUAL(CR::NoSource, il->closeReason(now));
  TEST_ASSERT_FALSE(il->needToggle(now));
}

// I1: boot (or first connect) with SD ENABLED: never opens, needs a toggle
void test_I1_sd_enabled_at_boot_never_opens() {
  run(true, 200); // 8 s of ENABLED
  TEST_ASSERT_FALSE(il->open(now));
  TEST_ASSERT_TRUE(il->needToggle(now));
  TEST_ASSERT_EQUAL(CR::NeedToggle, il->closeReason(now));
}

void test_stuck_disabled_never_opens() {
  run(false, 200);
  TEST_ASSERT_FALSE(il->open(now));
  TEST_ASSERT_FALSE(il->needToggle(now));
  TEST_ASSERT_EQUAL(CR::SdDisabled, il->closeReason(now));
}

// I2: off -> on opens on the first ENABLED frame
void test_I2_off_then_on_opens_at_once() {
  run(false, 5);
  frame(true);
  TEST_ASSERT_TRUE(il->open(now));
  TEST_ASSERT_EQUAL(CR::None, il->closeReason(now));
  TEST_ASSERT_FALSE(il->needToggle(now));
}

void test_disabled_closes_at_once_and_reopen_needs_fresh_edge() {
  run(false, 3);
  run(true, 10);
  TEST_ASSERT_TRUE(il->open(now));
  frame(false);
  TEST_ASSERT_FALSE(il->open(now));
  step();
  frame(true); // DISABLED -> ENABLED again: a new edge, opens
  TEST_ASSERT_TRUE(il->open(now));
}

void test_disabled_after_open_closes() {
  run(false, 3);
  run(true, 3);
  TEST_ASSERT_TRUE(il->open(now));
  frame(false);
  TEST_ASSERT_FALSE(il->open(now));
}

// I3: stop RC with SD ENABLED: closed at 1.00 s after the last accepted ENABLED state
void test_I3_stale_closes_at_1s() {
  run(false, 3);
  frame(true);
  const int64_t last = now;
  TEST_ASSERT_TRUE(il->open(now));
  TEST_ASSERT_TRUE(il->open(last + 1 * S));      // exactly 1.000 s: still open
  TEST_ASSERT_FALSE(il->open(last + 1 * S + 1)); // one ns later: closed
  TEST_ASSERT_EQUAL(CR::Stale, il->closeReason(last + 1 * S + 1));
  TEST_ASSERT_FALSE(il->needToggle(last + 1 * S + 1)); // stale state does not claim "needs toggle"
}

// I4 / D3-B: a gap of 1..2 s only closes, a fresh ENABLED reopens without a toggle; a gap > 2 s needs a toggle
void test_I4_gap_1p5s_reopens_without_toggle() {
  run(false, 3);
  run(true, 10);
  TEST_ASSERT_TRUE(il->open(now));
  step(1500 * MS);
  il->update(now);
  TEST_ASSERT_FALSE(il->open(now)); // closed during the gap
  frame(true);                      // RC is back, still ENABLED
  TEST_ASSERT_TRUE(il->open(now));
}

void test_I4_gap_2p5s_needs_toggle_via_frame_arrival() {
  run(false, 3);
  run(true, 10);
  step(2500 * MS);
  frame(true); // no update() in between: the frame itself detects the gap
  TEST_ASSERT_FALSE(il->open(now));
  TEST_ASSERT_TRUE(il->needToggle(now));
  step();
  run(true, 20);
  TEST_ASSERT_FALSE(il->open(now));
  run(false, 2);
  frame(true);
  TEST_ASSERT_TRUE(il->open(now));
}

void test_I4_gap_2p5s_needs_toggle_via_update() {
  run(false, 3);
  run(true, 10);
  step(2100 * MS);
  il->update(now); // silence of > 2 s forgets the transition
  step(PERIOD);
  frame(true);
  TEST_ASSERT_FALSE(il->open(now));
  TEST_ASSERT_TRUE(il->needToggle(now));
}

void test_gap_of_exactly_2s_is_still_a_short_gap() {
  run(false, 3);
  frame(true);
  const int64_t t0 = now;
  now = t0 + 2 * S;
  frame(true);
  TEST_ASSERT_TRUE(il->open(now));
}

void test_edge_needs_consecutive_states_within_1s() {
  run(false, 3);
  step(1500 * MS); // DISABLED, then nothing for 1.5 s
  frame(true);     // ENABLED: not a consecutive edge (gap > 1 s)
  TEST_ASSERT_FALSE(il->open(now));
  TEST_ASSERT_TRUE(il->needToggle(now));
  step();
  run(true, 10);
  TEST_ASSERT_FALSE(il->open(now));
}

void test_edge_at_exactly_1s_gap_counts() {
  run(false, 3);
  step(1 * S - PERIOD); // run() already advanced one period
  frame(true);
  TEST_ASSERT_TRUE(il->open(now));
}

// I6: a new session closes and needs a toggle; states before any session are ignored
void test_I6_session_change_clears_and_needs_toggle() {
  run(false, 3);
  run(true, 5);
  TEST_ASSERT_TRUE(il->open(now));
  il->onSessionChanged(); // a new GS session was latched
  TEST_ASSERT_FALSE(il->open(now));
  run(true, 10); // the new session shows ENABLED from its first state: never opens
  TEST_ASSERT_FALSE(il->open(now));
  TEST_ASSERT_TRUE(il->needToggle(now));
  run(false, 2);
  frame(true);
  TEST_ASSERT_TRUE(il->open(now));
}

void test_states_before_any_session_are_ignored() {
  il->onCmdState(true, now);
  il->onCmdState(false, now);
  TEST_ASSERT_FALSE(il->open(now));
  TEST_ASSERT_EQUAL(CR::NoSource, il->closeReason(now));
  TEST_ASSERT_FALSE(il->needToggle(now));
}

void test_session_change_while_enabled_closes_at_once() {
  run(false, 3);
  run(true, 3);
  il->onSessionChanged();
  TEST_ASSERT_FALSE(il->open(now));
  il->onCmdState(true, now); // first state of the new session is ENABLED: baseline only
  TEST_ASSERT_FALSE(il->open(now));
  TEST_ASSERT_TRUE(il->needToggle(now));
}

// I7: frames rejected by the ingress (foreign session, stale sequence, bad field) never reach the interlock: the gate closes by age
void test_I7_no_data_closes_at_1s_whatever_the_last_sd_was() {
  run(false, 3);
  frame(true);
  const int64_t last = now;
  // nothing is delivered (rejected frames never call onCmdState)
  TEST_ASSERT_FALSE(il->open(last + 1 * S + 1));
}

// I9 / D4-A: a re-arm trigger closes at once and needs a fresh off -> on, even with SD still ENABLED
void test_I9_rearm_closes_and_needs_toggle() {
  run(false, 3);
  run(true, 10);
  TEST_ASSERT_TRUE(il->open(now));
  il->requireRearm(MotorInterlock::RearmReason::MissionEnd);
  TEST_ASSERT_FALSE(il->open(now));
  TEST_ASSERT_EQUAL_UINT8(1, il->rearmCount());
  TEST_ASSERT_EQUAL((int)MotorInterlock::RearmReason::MissionEnd, (int)il->lastRearmReason());
  run(true, 25); // SD stays ENABLED
  TEST_ASSERT_FALSE(il->open(now));
  TEST_ASSERT_TRUE(il->needToggle(now));
  run(false, 2);
  frame(true);
  TEST_ASSERT_TRUE(il->open(now));
}

void test_rearm_while_sd_disabled_needs_a_disabled_frame_after_it() {
  run(false, 5);
  il->requireRearm(MotorInterlock::RearmReason::SystemReset);
  frame(true); // ENABLED right after the trigger, no DISABLED seen since: no edge
  TEST_ASSERT_FALSE(il->open(now));
  step();
  frame(false);
  step();
  frame(true);
  TEST_ASSERT_TRUE(il->open(now));
}

void test_rearm_every_tick_keeps_closed() { // Failure mode re-arms on every safety-task run
  run(false, 3);
  for (int i = 0; i < 100; i++) {
    il->requireRearm(MotorInterlock::RearmReason::Failure);
    frame(i % 2 == 0);
    step();
  }
  TEST_ASSERT_FALSE(il->open(now));
}

void test_out_of_order_and_negative_age_fail_closed() {
  run(false, 3);
  frame(true);
  TEST_ASSERT_TRUE(il->open(now));
  TEST_ASSERT_FALSE(il->open(now - 1)); // a clock that runs backwards: closed
  il->onCmdState(false, now - 5 * MS); // out of order state is ignored
  TEST_ASSERT_TRUE(il->open(now));
}

void test_enabled_state_lease_extends_open_while_frames_flow() {
  run(false, 3);
  run(true, 500); // 20 s
  TEST_ASSERT_TRUE(il->open(now));
}

// ---- the gate AND (shim of StarshipTVC: evaluateMotorGate is the function the TVC task calls) ----

static MotorGateInput allOn() {
  MotorGateInput i;
  i.interlockOpen = true;
  i.forcedOff = false;
  i.enableMotors = true;
  i.enableActuators = true;
  i.testIdle = true;
  return i;
}

void test_gate_all_conditions_open() {
  const MotorGateResult r = evaluateMotorGate(allOn());
  TEST_ASSERT_TRUE(r.outputsEnabled);
}

void test_gate_closed_interlock_blocks_outputs() {
  MotorGateInput i = allOn();
  i.interlockOpen = false;
  const MotorGateResult r = evaluateMotorGate(i);
  TEST_ASSERT_FALSE(r.outputsEnabled);
}

void test_gate_forced_off_blocks_even_with_open_interlock() {
  MotorGateInput i = allOn();
  i.forcedOff = true;
  const MotorGateResult r = evaluateMotorGate(i);
  TEST_ASSERT_FALSE(r.outputsEnabled);
}

void test_gate_every_other_condition_needed_for_outputs() {
  MotorGateInput i = allOn();
  i.enableMotors = false;
  MotorGateResult r = evaluateMotorGate(i);
  TEST_ASSERT_FALSE(r.outputsEnabled);
  i = allOn();
  i.enableActuators = false;
  r = evaluateMotorGate(i);
  TEST_ASSERT_FALSE(r.outputsEnabled);
  i = allOn();
  i.testIdle = false;
  r = evaluateMotorGate(i);
  TEST_ASSERT_FALSE(r.outputsEnabled);
}

void test_gate_truth_table_outputs_imply_gate() { // exhaustive: outputs never enabled with a closed gate
  for (int m = 0; m < 32; m++) {
    MotorGateInput i;
    i.interlockOpen = m & 1;
    i.forcedOff = m & 2;
    i.enableMotors = m & 4;
    i.enableActuators = m & 8;
    i.testIdle = m & 16;
    const MotorGateResult r = evaluateMotorGate(i);
    if (r.outputsEnabled) {
      TEST_ASSERT_TRUE(i.interlockOpen && !i.forcedOff && i.enableMotors && i.enableActuators && i.testIdle);
    }
    if (!i.interlockOpen || i.forcedOff) {
      TEST_ASSERT_FALSE(r.outputsEnabled);
    }
  }
}

void test_unset_interlock_pointer_means_closed() { // the TVC builds interlockOpen = (ptr != nullptr && ptr->open())
  const MotorInterlock *none = nullptr;
  MotorGateInput i = allOn();
  i.interlockOpen = (none != nullptr && none->open(now));
  TEST_ASSERT_FALSE(evaluateMotorGate(i).outputsEnabled);
}

void test_watchdog_stale_rule() {
  TEST_ASSERT_TRUE(interlockWatchdogStale(5 * S, 0)); // never ran
  TEST_ASSERT_FALSE(interlockWatchdogStale(5 * S + 10 * MS, 5 * S));
  TEST_ASSERT_FALSE(interlockWatchdogStale(5 * S + 100 * MS, 5 * S)); // exactly 100 ms
  TEST_ASSERT_TRUE(interlockWatchdogStale(5 * S + 100 * MS + 1, 5 * S));
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_boot_is_closed_without_source);
  RUN_TEST(test_I1_sd_enabled_at_boot_never_opens);
  RUN_TEST(test_stuck_disabled_never_opens);
  RUN_TEST(test_I2_off_then_on_opens_at_once);
  RUN_TEST(test_disabled_closes_at_once_and_reopen_needs_fresh_edge);
  RUN_TEST(test_disabled_after_open_closes);
  RUN_TEST(test_I3_stale_closes_at_1s);
  RUN_TEST(test_I4_gap_1p5s_reopens_without_toggle);
  RUN_TEST(test_I4_gap_2p5s_needs_toggle_via_frame_arrival);
  RUN_TEST(test_I4_gap_2p5s_needs_toggle_via_update);
  RUN_TEST(test_gap_of_exactly_2s_is_still_a_short_gap);
  RUN_TEST(test_edge_needs_consecutive_states_within_1s);
  RUN_TEST(test_edge_at_exactly_1s_gap_counts);
  RUN_TEST(test_I6_session_change_clears_and_needs_toggle);
  RUN_TEST(test_states_before_any_session_are_ignored);
  RUN_TEST(test_session_change_while_enabled_closes_at_once);
  RUN_TEST(test_I7_no_data_closes_at_1s_whatever_the_last_sd_was);
  RUN_TEST(test_I9_rearm_closes_and_needs_toggle);
  RUN_TEST(test_rearm_while_sd_disabled_needs_a_disabled_frame_after_it);
  RUN_TEST(test_rearm_every_tick_keeps_closed);
  RUN_TEST(test_out_of_order_and_negative_age_fail_closed);
  RUN_TEST(test_enabled_state_lease_extends_open_while_frames_flow);
  RUN_TEST(test_gate_all_conditions_open);
  RUN_TEST(test_gate_closed_interlock_blocks_outputs);
  RUN_TEST(test_gate_forced_off_blocks_even_with_open_interlock);
  RUN_TEST(test_gate_every_other_condition_needed_for_outputs);
  RUN_TEST(test_gate_truth_table_outputs_imply_gate);
  RUN_TEST(test_unset_interlock_pointer_means_closed);
  RUN_TEST(test_watchdog_stale_rule);
  return UNITY_END();
}
