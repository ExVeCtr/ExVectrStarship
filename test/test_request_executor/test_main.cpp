// Host tests of src/safety/request_executor.cpp: pio test -e native -f test_request_executor
#include <math.h>
#include <unity.h>

#include "safety/request_executor.hpp"

using namespace VCTR::Safety;

struct Actions : RequestActionsI {
  int zeroCalls = 0, stopCalls = 0;
  float zeroArg = -1.0f, stopArg = -1.0f;
  bool zeroOk = true;
  bool zeroEstimators(float w) override {
    zeroCalls++;
    zeroArg = w;
    return zeroOk;
  }
  void setStopAltitude(float m) override {
    stopCalls++;
    stopArg = m;
  }
};

static constexpr uint32_t kSess = 0xA1B2C3D4u;
static RequestExecutor ex;
static Actions act;
static RequestContext ctx;

void setUp() {
  ex = RequestExecutor();
  act = Actions();
  ctx = RequestContext();
  ctx.sessionLatched = true;
  ctx.latchedSession = kSess;
}
void tearDown() {}

static RequestMsg msg(uint16_t id, uint8_t action, float f = 0.0f, uint8_t src = 0) {
  RequestMsg m;
  m.src = src;
  m.reqId = id;
  m.action = action;
  m.target = 1;
  m.argF = f;
  m.session = kSess;
  return m;
}
static RequestResult run(const RequestMsg &m) { return ex.handle(m, ctx, act); }
static void expect(const RequestResult &r, uint8_t status, uint8_t detail) {
  TEST_ASSERT_EQUAL_UINT8(status, r.status);
  TEST_ASSERT_EQUAL_UINT8(detail, r.detail);
}

void test_zero_done_passes_window() {
  RequestResult r = run(msg(1, 12, 5.0f));
  expect(r, 0, 0);
  TEST_ASSERT_TRUE(r.executed);
  TEST_ASSERT_FALSE(r.duplicate);
  TEST_ASSERT_EQUAL(1, act.zeroCalls);
  TEST_ASSERT_EQUAL_FLOAT(5.0f, act.zeroArg);
  TEST_ASSERT_EQUAL_UINT16(1, ex.lastReqId());
}

void test_zero_nan_window_is_passed_through_for_the_zeroing_default() {
  expect(run(msg(1, 12, NAN)), 0, 0);
  TEST_ASSERT_TRUE(isnan(act.zeroArg));
}

void test_zero_state_refusal_detail_2() {
  act.zeroOk = false;
  expect(run(msg(1, 12, 5.0f)), 2, 2);
  TEST_ASSERT_EQUAL(1, act.zeroCalls);
}

void test_stop_altitude_done_and_range() {
  expect(run(msg(1, 13, 25.0f)), 0, 0);
  TEST_ASSERT_EQUAL_FLOAT(25.0f, act.stopArg);
  expect(run(msg(2, 13, 1.0f)), 0, 0);
  expect(run(msg(3, 13, 100.0f)), 0, 0);
  expect(run(msg(4, 13, 0.99f)), 3, 1);
  expect(run(msg(5, 13, NAN)), 3, 1);
  expect(run(msg(6, 13, INFINITY)), 3, 1);
  expect(run(msg(7, 13, -4.0f)), 3, 1);
  TEST_ASSERT_EQUAL(3, act.stopCalls);
}

void test_not_on_the_ground_detail_1() {
  ctx.committed = true;
  expect(run(msg(1, 12, 5.0f)), 2, 1);
  expect(run(msg(2, 13, 25.0f)), 2, 1);
  ctx.committed = false;
  ctx.interlockOpen = true; // armed
  expect(run(msg(3, 12, 5.0f)), 2, 1);
  expect(run(msg(4, 13, 25.0f)), 2, 1);
  TEST_ASSERT_EQUAL(0, act.zeroCalls + act.stopCalls);
}

void test_session_cases_detail_3() {
  RequestMsg m = msg(1, 12, 5.0f);
  m.session = 0;
  expect(run(m), 2, 3);
  m = msg(2, 12, 5.0f);
  m.session = kSess + 1;
  expect(run(m), 2, 3);
  ctx.sessionLatched = false;
  expect(run(msg(3, 12, 5.0f)), 2, 3);
  ctx.sessionLatched = true;
  ctx.latchedSession = 0; // nothing latched cannot be matched by a zero or any session
  m = msg(4, 12, 5.0f);
  m.session = 0;
  expect(run(m), 2, 3);
  TEST_ASSERT_EQUAL(0, act.zeroCalls);
}

void test_wrong_target_and_unknown_action() {
  RequestMsg m = msg(1, 12, 5.0f);
  m.target = 2;
  expect(run(m), 3, 2);
  m.reqId = 2;
  m.target = 0;
  expect(run(m), 3, 2);
  expect(run(msg(3, 9)), 3, 3);   // SYSTEM_RESET
  expect(run(msg(4, 0)), 3, 3);   // NONE
  expect(run(msg(5, 7)), 3, 3);   // CALIB_APPLY, not implemented
  expect(run(msg(6, 200)), 3, 3);
  TEST_ASSERT_EQUAL(0, act.zeroCalls + act.stopCalls);
}

void test_duplicate_is_reacked_not_reexecuted() {
  RequestResult a = run(msg(7, 13, 30.0f));
  expect(a, 0, 0);
  RequestResult b = run(msg(7, 13, 30.0f));
  expect(b, 0, 0);
  TEST_ASSERT_TRUE(b.duplicate);
  TEST_ASSERT_FALSE(b.executed);
  TEST_ASSERT_EQUAL(1, act.stopCalls);
  TEST_ASSERT_EQUAL_UINT32(1, ex.duplicateCount());
}

void test_duplicate_of_a_refusal_returns_the_stored_refusal() {
  ctx.committed = true;
  expect(run(msg(7, 12, 5.0f)), 2, 1);
  ctx.committed = false; // state changed meanwhile: the same id still gets the stored result
  RequestResult b = run(msg(7, 12, 5.0f));
  expect(b, 2, 1);
  TEST_ASSERT_TRUE(b.duplicate);
  TEST_ASSERT_EQUAL(0, act.zeroCalls);
  expect(run(msg(8, 12, 5.0f)), 0, 0); // a new id works
}

void test_same_id_from_another_node_is_a_new_request() {
  expect(run(msg(7, 13, 30.0f, 0)), 0, 0);
  RequestResult r = run(msg(7, 13, 31.0f, 5));
  TEST_ASSERT_FALSE(r.duplicate);
  TEST_ASSERT_EQUAL(2, act.stopCalls);
}

void test_ninth_distinct_request_evicts_the_oldest() {
  for (uint16_t id = 1; id <= 8; id++) expect(run(msg(id, 13, 20.0f)), 0, 0);
  TEST_ASSERT_EQUAL(8, act.stopCalls);
  TEST_ASSERT_TRUE(run(msg(2, 13, 20.0f)).duplicate); // still remembered
  expect(run(msg(9, 13, 20.0f)), 0, 0);               // evicts id 1
  TEST_ASSERT_EQUAL(9, act.stopCalls);
  TEST_ASSERT_TRUE(run(msg(2, 13, 20.0f)).duplicate);
  RequestResult again = run(msg(1, 13, 20.0f)); // forgotten: executes again
  TEST_ASSERT_FALSE(again.duplicate);
  TEST_ASSERT_EQUAL(10, act.stopCalls);
}

void test_counters_and_inbox() {
  run(msg(1, 13, 20.0f));
  run(msg(2, 9));
  run(msg(1, 13, 20.0f));
  TEST_ASSERT_EQUAL_UINT32(1, ex.executedCount());
  TEST_ASSERT_EQUAL_UINT32(1, ex.refusedCount());
  TEST_ASSERT_EQUAL_UINT32(1, ex.duplicateCount());
  RequestInbox q;
  RequestMsg m;
  TEST_ASSERT_FALSE(q.pop(m));
  for (uint16_t i = 1; i <= RequestInbox::kSize; i++) TEST_ASSERT_TRUE(q.push(msg(i, 12)));
  TEST_ASSERT_FALSE(q.push(msg(99, 12)));
  TEST_ASSERT_EQUAL_UINT32(1, q.dropped());
  for (uint16_t i = 1; i <= RequestInbox::kSize; i++) {
    TEST_ASSERT_TRUE(q.pop(m));
    TEST_ASSERT_EQUAL_UINT16(i, m.reqId);
  }
  TEST_ASSERT_FALSE(q.pop(m));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_zero_done_passes_window);
  RUN_TEST(test_zero_nan_window_is_passed_through_for_the_zeroing_default);
  RUN_TEST(test_zero_state_refusal_detail_2);
  RUN_TEST(test_stop_altitude_done_and_range);
  RUN_TEST(test_not_on_the_ground_detail_1);
  RUN_TEST(test_session_cases_detail_3);
  RUN_TEST(test_wrong_target_and_unknown_action);
  RUN_TEST(test_duplicate_is_reacked_not_reexecuted);
  RUN_TEST(test_duplicate_of_a_refusal_returns_the_stored_refusal);
  RUN_TEST(test_same_id_from_another_node_is_a_new_request);
  RUN_TEST(test_ninth_distinct_request_evicts_the_oldest);
  RUN_TEST(test_counters_and_inbox);
  return UNITY_END();
}
