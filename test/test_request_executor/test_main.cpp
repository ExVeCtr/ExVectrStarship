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
  int mpCalls = 0;
  uint8_t mpId = 0;
  float mpVal = -1.0f;
  void setMissionParam(uint8_t id, float v) override {
    mpCalls++;
    mpId = id;
    mpVal = v;
  }
  int calls = 0; // all other actions
  uint8_t sub = 99, code = 99;
  bool calibOk = true;
  float wind = -9, dir = -9, gustS = -9, gustP = -9;
  uint32_t seed = 12345678;
  SimFaultDecision fault;
  bool calibApply(uint8_t s) override { calls++; sub = s; return calibOk; }
  bool saveConfig() override { calls++; return calibOk; }
  bool calibControl(uint8_t s, uint8_t c) override { calls++; sub = s; code = c; return calibOk; }
  void simWindSpeed(float v) override { calls++; wind = v; }
  void simWindDirection(float v) override { calls++; dir = v; }
  void simGustStrength(float v) override { calls++; gustS = v; }
  void simGustPeriod(float v) override { calls++; gustP = v; }
  void simGustSeed(uint32_t v) override { calls++; seed = v; }
  bool simFault(const SimFaultDecision &d) override { calls++; fault = d; return true; }
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
  ctx.calibGatesOk = true;
  ctx.homeAvailable = true;
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
static RequestMsg msgI(uint16_t id, uint8_t action, int32_t i, float f = 0.0f) {
  RequestMsg m = msg(id, action, f);
  m.argI = i;
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
  for (uint8_t a = 1; a <= 6; a++) expect(run(msg(10 + a, a)), 3, 3); // retired
  expect(run(msg(30, 10)), 3, 3); // link actions are not vehicle actions
  expect(run(msg(31, 11)), 3, 3);
  expect(run(msg(6, 200)), 3, 3);
  TEST_ASSERT_EQUAL(0, act.calls);
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

void test_calib_apply() {
  expect(run(msgI(1, 7, 2)), 0, 0);
  TEST_ASSERT_EQUAL_UINT8(2, act.sub);
  expect(run(msgI(2, 7, 3)), 3, 1);
  expect(run(msgI(3, 7, -1)), 3, 1);
  ctx.calibGatesOk = false;
  expect(run(msgI(4, 7, 0)), 2, 6);
  ctx.calibGatesOk = true;
  act.calibOk = false; // no passing candidate
  expect(run(msgI(5, 7, 1)), 2, 6);
  // not tied to the ground rule, and allowed in simulation
  act.calibOk = true;
  ctx.committed = true;
  ctx.simulationMode = true;
  expect(run(msgI(6, 7, 1)), 0, 0);
}

void test_save_config() {
  expect(run(msg(1, 8)), 0, 0);
  ctx.calibGatesOk = false;
  expect(run(msg(2, 8)), 2, 6);
  ctx.calibGatesOk = true;
  act.calibOk = false;
  expect(run(msg(3, 8)), 2, 6);
}

void test_calib_control() {
  const int32_t ok[][3] = {{1, 0, 1}, {2, 0, 2}, {0, 0, 0}, {9, 0, 9}, {11, 1, 1}, {20, 2, 0}, {29, 2, 9}, {12, 1, 2}};
  uint16_t id = 1;
  for (const auto &c : ok) {
    expect(run(msgI(id++, 14, c[0])), 0, 0);
    TEST_ASSERT_EQUAL_UINT8(c[1], act.sub);
    TEST_ASSERT_EQUAL_UINT8(c[2], act.code);
  }
  const int32_t bad[] = {3, 4, 5, 8, 13, 30, 31, -1, -10, 100};
  for (int32_t b : bad) expect(run(msgI(id++, 14, b)), 3, 1);
  ctx.calibGatesOk = false;
  expect(run(msgI(id++, 14, 1)), 2, 6);
  ctx.calibGatesOk = true;
  ctx.simulationMode = true; // simulation must be OFF
  expect(run(msgI(id++, 14, 1)), 2, 6);
  ctx.simulationMode = false;
  act.calibOk = false; // service refusal (busy, not collecting)
  expect(run(msgI(id++, 14, 1)), 2, 6);
}

void test_sim_actions_only_in_simulation() {
  for (uint8_t a = 15; a <= 22; a++) expect(run(msg(a, a, 1.0f)), 2, 5);
  TEST_ASSERT_EQUAL(0, act.calls);
  ctx.committed = true; // not tied to the ground rule
  ctx.interlockOpen = true;
  ctx.simulationMode = true;
  for (uint8_t a = 15; a <= 19; a++) expect(run(msg(100 + a, a, 1.0f)), 0, 0);
  expect(run(msg(120, 22)), 0, 0);
}

void test_sim_wind_gust_clamping() {
  ctx.simulationMode = true;
  expect(run(msg(1, 15, 50.0f)), 0, 0);
  TEST_ASSERT_EQUAL_FLOAT(20.0f, act.wind);
  expect(run(msg(2, 15, -3.0f)), 0, 0);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, act.wind);
  expect(run(msg(3, 15, 7.5f)), 0, 0);
  TEST_ASSERT_EQUAL_FLOAT(7.5f, act.wind);
  expect(run(msg(4, 16, 4.0f)), 0, 0); // wraps to 4 - 2pi
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 4.0f - 6.2831853f, act.dir);
  expect(run(msg(5, 17, 99.0f)), 0, 0);
  TEST_ASSERT_EQUAL_FLOAT(10.0f, act.gustS);
  expect(run(msg(6, 18, 0.1f)), 0, 0);
  TEST_ASSERT_EQUAL_FLOAT(0.5f, act.gustP);
  expect(run(msg(7, 18, 100.0f)), 0, 0);
  TEST_ASSERT_EQUAL_FLOAT(30.0f, act.gustP);
  expect(run(msg(8, 19, 41.6f)), 0, 0);
  TEST_ASSERT_EQUAL_UINT32(42, act.seed);
  expect(run(msg(9, 19, 1e9f)), 0, 0);
  TEST_ASSERT_EQUAL_UINT32(16777215u, act.seed);
  expect(run(msg(10, 19, -5.0f)), 0, 0);
  TEST_ASSERT_EQUAL_UINT32(0, act.seed);
  const int before = act.calls;
  for (uint8_t a = 15; a <= 19; a++) {
    expect(run(msg(200 + a, a, NAN)), 3, 1);
    expect(run(msg(220 + a, a, INFINITY)), 3, 1);
  }
  TEST_ASSERT_EQUAL(before, act.calls);
}

void test_sim_flap_faults() {
  ctx.simulationMode = true;
  expect(run(msgI(1, 20, 2, 0.5f)), 0, 0);
  TEST_ASSERT_TRUE(act.fault.action == SimFaultAction::Stuck);
  TEST_ASSERT_EQUAL_UINT8(2, act.fault.flap);
  TEST_ASSERT_EQUAL_FLOAT(0.5f, act.fault.value);
  TEST_ASSERT_FALSE(act.fault.captureCurrent);
  expect(run(msgI(2, 20, 3, 9.0f)), 0, 0); // clamped to pi/2
  TEST_ASSERT_EQUAL_FLOAT(kSimFlapMaxRad, act.fault.value);
  expect(run(msgI(3, 20, 0, -1.0f)), 0, 0); // hold the current true angle
  TEST_ASSERT_TRUE(act.fault.captureCurrent);
  expect(run(msgI(4, 20, 1, -0.5f)), 0, 0); // other negatives clamp to 0
  TEST_ASSERT_EQUAL_FLOAT(0.0f, act.fault.value);
  TEST_ASSERT_FALSE(act.fault.captureCurrent);
  expect(run(msgI(5, 21, 1, 0.01f)), 0, 0);
  TEST_ASSERT_TRUE(act.fault.action == SimFaultAction::Slow);
  TEST_ASSERT_EQUAL_FLOAT(0.05f, act.fault.value);
  expect(run(msgI(6, 21, 1, 5.0f)), 0, 0);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, act.fault.value);
  const int before = act.calls;
  expect(run(msgI(7, 20, 4, 0.5f)), 3, 1);
  expect(run(msgI(8, 20, -1, 0.5f)), 3, 1);
  expect(run(msgI(9, 21, 16, 0.5f)), 3, 1); // bit 0x10 style values are refused, flap exactly 0..3
  expect(run(msgI(10, 20, 1, NAN)), 3, 1);
  expect(run(msgI(11, 21, 1, INFINITY)), 3, 1);
  TEST_ASSERT_EQUAL(before, act.calls);
  expect(run(msg(12, 22)), 0, 0);
  TEST_ASSERT_TRUE(act.fault.action == SimFaultAction::Clear);
  ctx.simulationMode = false;
  expect(run(msgI(13, 20, 1, 0.5f)), 2, 5);
  expect(run(msg(14, 22)), 2, 5);
}

void test_set_mission_param() {
  RequestResult r = run(msgI(1, 24, 2, 150.0f));
  expect(r, 0, 0);
  TEST_ASSERT_EQUAL_INT(1, act.mpCalls);
  TEST_ASSERT_EQUAL_UINT8(2, act.mpId);
  TEST_ASSERT_EQUAL_FLOAT(150.0f, act.mpVal);
  // idempotent: the same value again is accepted (new request id)
  expect(run(msgI(2, 24, 2, 150.0f)), 0, 0);
  // range edges
  expect(run(msgI(3, 24, 1, 0.5f)), 0, 0);
  expect(run(msgI(4, 24, 1, 10.0f)), 0, 0);
  expect(run(msgI(5, 24, 6, 600.0f)), 0, 0);
  expect(run(msgI(6, 24, 3, 0.0f)), 0, 0);
  expect(run(msgI(50, 24, 7, 0.2f)), 0, 0);
  expect(run(msgI(51, 24, 8, 20.0f)), 0, 0);
  expect(run(msgI(52, 24, 9, -2000.0f)), 0, 0);
  expect(run(msgI(53, 24, 10, 2000.0f)), 0, 0);
  TEST_ASSERT_EQUAL_UINT8(10, act.mpId);
  TEST_ASSERT_EQUAL_FLOAT(2000.0f, act.mpVal);
  const int ok = act.mpCalls;
  // invalid: unknown id, out of range, non-finite
  expect(run(msgI(10, 24, 0, 5.0f)), 3, 1);
  expect(run(msgI(11, 24, 11, 5.0f)), 3, 1);
  expect(run(msgI(60, 24, 7, 0.19f)), 3, 1);
  expect(run(msgI(61, 24, 8, 20.1f)), 3, 1);
  expect(run(msgI(62, 24, 9, 2000.5f)), 3, 1);
  expect(run(msgI(63, 24, -0 + 10, -2000.5f)), 3, 1);
  expect(run(msgI(12, 24, -1, 5.0f)), 3, 1);
  expect(run(msgI(13, 24, 1, 0.49f)), 3, 1);
  expect(run(msgI(14, 24, 1, 10.1f)), 3, 1);
  expect(run(msgI(15, 24, 2, 1.9f)), 3, 1);
  expect(run(msgI(16, 24, 2, 1000.5f)), 3, 1);
  expect(run(msgI(17, 24, 4, -0.1f)), 3, 1);
  expect(run(msgI(18, 24, 5, 600.1f)), 3, 1);
  expect(run(msgI(19, 24, 6, NAN)), 3, 1);
  expect(run(msgI(20, 24, 1, INFINITY)), 3, 1);
  TEST_ASSERT_EQUAL_INT(ok, act.mpCalls);
  // ground only: committed or interlock open
  ctx.committed = true;
  expect(run(msgI(30, 24, 2, 100.0f)), 2, 1);
  ctx.committed = false;
  ctx.interlockOpen = true;
  expect(run(msgI(31, 24, 2, 100.0f)), 2, 1);
  ctx.interlockOpen = false;
  // session
  RequestMsg m = msgI(32, 24, 2, 100.0f);
  m.session = kSess + 1;
  expect(run(m), 2, 3);
  TEST_ASSERT_EQUAL_INT(ok, act.mpCalls);
}

void test_get_home_position() {
  RequestResult r = run(msg(1, 23));
  expect(r, 0, 0);
  TEST_ASSERT_TRUE(r.sendHome);
  ctx.homeAvailable = false;
  r = run(msg(2, 23));
  expect(r, 2, 4);
  TEST_ASSERT_FALSE(r.sendHome);
  // duplicate of the accepted copy: acked again, no second packet
  ctx.homeAvailable = true;
  r = run(msg(1, 23));
  expect(r, 0, 0);
  TEST_ASSERT_TRUE(r.duplicate);
  TEST_ASSERT_FALSE(r.sendHome);
  ctx.committed = true; // allowed in flight too
  TEST_ASSERT_TRUE(run(msg(3, 23)).sendHome);
}

void test_new_actions_need_session_and_dedupe() {
  ctx.simulationMode = true;
  RequestMsg m = msg(1, 15, 5.0f);
  m.session = kSess + 1;
  expect(run(m), 2, 3);
  m = msg(2, 14);
  m.argI = 1;
  m.session = 0;
  expect(run(m), 2, 3);
  expect(run(msg(3, 15, 5.0f)), 0, 0);
  TEST_ASSERT_TRUE(run(msg(3, 15, 6.0f)).duplicate);
  TEST_ASSERT_EQUAL_FLOAT(5.0f, act.wind);
  TEST_ASSERT_EQUAL(1, act.calls);
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
  RUN_TEST(test_calib_apply);
  RUN_TEST(test_save_config);
  RUN_TEST(test_calib_control);
  RUN_TEST(test_sim_actions_only_in_simulation);
  RUN_TEST(test_sim_wind_gust_clamping);
  RUN_TEST(test_sim_flap_faults);
  RUN_TEST(test_set_mission_param);
  RUN_TEST(test_get_home_position);
  RUN_TEST(test_new_actions_need_session_and_dedupe);
  RUN_TEST(test_counters_and_inbox);
  return UNITY_END();
}
