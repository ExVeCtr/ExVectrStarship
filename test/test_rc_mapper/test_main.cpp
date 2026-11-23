// Host tests of src/safety/rc_mapper.cpp and src/safety/rc_command.cpp (CMD_STATE decode, edge events, SE classifier,
// command-intent layer; T9b-4 ported to the GS-control input): pio test -e native -f test_rc_mapper
#include <unity.h>

#include <initializer_list>

#include "calibration/calibration_validity.hpp"
#include "safety/rc_command.hpp"
#include "safety/rc_mapper.hpp"

using namespace VCTR::Safety;
static constexpr int64_t S = 1000000000LL;
static constexpr int64_t MS = 1000000LL;

void setUp() {}
void tearDown() {}

// ======================= T9b-4: edge events, SE classifier, command-intent layer =============================

// The tests were written with the handset's three switch levels; they are mapped to the CMD_STATE fields the way the
// ground station does it: SA HI = GO, SB LO/MID/HI = profile 0/1/2, SC LO = SIMULATION and MID = REAL (the old
// test numbering; the field itself is 0 REAL, 1 SIMULATION), SD HI = ENABLED, SE HI = PRESSED.
static constexpr int16_t LO = -1000, HI = 1000, MID = 0;

static CmdFrame frame(int16_t sa, int16_t sb, int16_t sc, int16_t sd, int16_t se) {
  CmdFrame f;
  f.session = 0x1234;
  f.go = sa == HI ? 1 : 0;
  f.profile = sb == LO ? 0 : sb == MID ? 1 : 2;
  f.vehicleMode = sc == MID ? 0 : 1;
  f.motors = sd == HI ? 1 : 0;
  f.resetButton = se == HI ? 1 : 0;
  return f;
}

/// Feeds the mapper like the comms task: accepted states of the latched session at 25 Hz (40 ms).
struct Rig {
  RcCommandMapper m;
  int64_t t = 100 * S;
  Rig() { m.onSessionChanged(); }
  void feed(int16_t sa, int16_t sb, int16_t sc, int16_t sd, int16_t se, int64_t dt = 40 * MS) {
    t += dt;
    m.onAccepted(frame(sa, sb, sc, sd, se), t);
  }
  /// n states with the same values
  void hold(int n, int16_t sa, int16_t sb, int16_t sc, int16_t sd, int16_t se) {
    for (int i = 0; i < n; i++) feed(sa, sb, sc, sd, se);
  }
  void safeIdle() { hold(3, LO, LO, LO, LO, LO); } // all safe values
};

static void test_go_edge_needs_observed_not_go_then_go() {
  Rig r;
  r.hold(3, LO, HI, MID, LO, LO); // SA not-go, SB 2, SC 1
  RcEvents e = r.m.takeEvents();
  TEST_ASSERT_FALSE(e.goEdge);
  r.feed(HI, HI, MID, LO, LO);
  e = r.m.takeEvents();
  TEST_ASSERT_TRUE(e.goEdge);
  TEST_ASSERT_EQUAL_UINT8(2, e.goProfile);
  TEST_ASSERT_FALSE(r.m.takeEvents().goEdge); // consumed
  r.hold(5, HI, HI, MID, LO, LO); // SA stays GO: no second edge
  TEST_ASSERT_FALSE(r.m.takeEvents().goEdge);
  r.feed(LO, HI, MID, LO, LO);
  r.feed(HI, HI, MID, LO, LO); // released and flipped again: a new edge
  TEST_ASSERT_TRUE(r.m.takeEvents().goEdge);
}

static void test_go_edge_is_latched_between_ticks() {
  Rig r;
  r.hold(3, LO, HI, MID, LO, LO);
  r.feed(HI, HI, MID, LO, LO);
  r.hold(2, HI, HI, MID, LO, LO); // the 10 Hz tick comes later and still sees the edge
  TEST_ASSERT_TRUE(r.m.takeEvents().goEdge);
}

static void test_boot_with_every_switch_combination_does_nothing() {
  // 32 combinations of SA GO/NOT_GO x SC 0/1 x SD ENABLED/DISABLED x SE pressed/released (SB any): at boot (first
  // state of a source) and while the switches stay put, no event is produced.
  for (int comb = 0; comb < 16; comb++) {
    for (int scPos : {0, 1}) {
      Rig r;
      const int16_t sa = (comb & 1) ? HI : LO, sd = (comb & 2) ? HI : LO, se = (comb & 4) ? HI : LO;
      const int16_t sb = (comb & 8) ? HI : MID;
      const int16_t sc = scPos == 0 ? LO : MID;
      r.hold(100, sa, sb, sc, sd, se); // 4 s, includes a pressed SE held far beyond 1 s
      const RcEvents e = r.m.takeEvents();
      TEST_ASSERT_FALSE(e.goEdge);
      TEST_ASSERT_EQUAL_UINT8(RcEvents::kNone, e.scEdgeTo);
      TEST_ASSERT_FALSE(e.sePress);
      TEST_ASSERT_FALSE(r.m.hold().active);
    }
  }
}

static void test_each_switch_must_be_cycled_after_boot() {
  Rig r;
  r.hold(5, HI, HI, MID, HI, HI); // go GO, REAL, motors ENABLED, reset PRESSED at the session start
  r.m.takeEvents();
  r.feed(LO, HI, LO, LO, LO); // everything to its safe value, vehicle_mode REAL -> SIMULATION (the first change)
  const RcEvents e0 = r.m.takeEvents();
  TEST_ASSERT_EQUAL_UINT8(1, e0.scEdgeTo); // an observed change to SIMULATION
  r.feed(HI, HI, LO, LO, LO); // go to GO: edge
  TEST_ASSERT_TRUE(r.m.takeEvents().goEdge);
  r.feed(HI, HI, MID, LO, LO); // SIMULATION -> REAL: edge
  TEST_ASSERT_EQUAL_UINT8(0, r.m.takeEvents().scEdgeTo);
}

static void test_sc_only_an_observed_change_acts_both_directions() {
  Rig r;
  r.hold(3, LO, HI, MID, LO, LO); // vehicle_mode REAL at the session start
  TEST_ASSERT_EQUAL_UINT8(RcEvents::kNone, r.m.takeEvents().scEdgeTo);
  r.feed(LO, HI, LO, LO, LO); // REAL -> SIMULATION
  TEST_ASSERT_EQUAL_UINT8(1, r.m.takeEvents().scEdgeTo);
  r.hold(3, LO, HI, LO, LO, LO);
  TEST_ASSERT_EQUAL_UINT8(RcEvents::kNone, r.m.takeEvents().scEdgeTo); // staying: nothing
  r.feed(LO, HI, MID, LO, LO); // SIMULATION -> REAL
  TEST_ASSERT_EQUAL_UINT8(0, r.m.takeEvents().scEdgeTo);
}

static void test_sc_simulation_at_session_start_makes_no_edge() {
  Rig r;
  r.hold(50, LO, HI, LO, LO, LO); // SIMULATION from the first state: nothing until it is cycled
  TEST_ASSERT_EQUAL_UINT8(RcEvents::kNone, r.m.takeEvents().scEdgeTo);
  TEST_ASSERT_TRUE(r.m.decoded().mode == 1);
}

static void test_gap_over_one_second_or_session_change_forgets_the_base() {
  Rig r;
  r.hold(3, LO, HI, LO, LO, LO);
  r.feed(HI, HI, LO, LO, LO, 1100 * MS); // SA flips after a 1.1 s gap: previous state too old for an edge
  TEST_ASSERT_FALSE(r.m.takeEvents().goEdge);
  Rig q;
  q.hold(3, LO, HI, LO, LO, LO);
  q.m.onSessionChanged(); // new session: all detectors forget
  q.feed(HI, HI, LO, LO, LO);
  TEST_ASSERT_FALSE(q.m.takeEvents().goEdge);
  q.feed(LO, HI, LO, LO, LO);
  q.feed(HI, HI, LO, LO, LO);
  TEST_ASSERT_TRUE(q.m.takeEvents().goEdge);
}

// --- SE classifier (D10, D11) ---

static void press(Rig &r, int n) { r.hold(n, LO, HI, MID, LO, HI); }

static void test_se_short_press_resets() {
  Rig r;
  r.safeIdle();
  press(r, 5); // 200 ms pressed
  r.feed(LO, HI, MID, LO, LO);
  const RcEvents e = r.m.takeEvents();
  TEST_ASSERT_TRUE(e.sePress);
  TEST_ASSERT_FALSE(r.m.hold().active);
}

static void test_se_single_sample_is_ignored() {
  Rig r;
  r.safeIdle();
  press(r, 1);
  r.feed(LO, HI, MID, LO, LO);
  TEST_ASSERT_FALSE(r.m.takeEvents().sePress);
}

static void test_se_press_without_release_seen_first_does_nothing() {
  Rig r;
  press(r, 6);
  r.feed(LO, HI, MID, LO, LO); // the first release only arms the classifier
  TEST_ASSERT_FALSE(r.m.takeEvents().sePress);
}

static void test_se_press_of_one_second_or_more_is_not_a_short_press() {
  Rig r;
  r.safeIdle();
  press(r, 25); // 25th pressed sample at +960 ms, release at +1000 ms after the first: no longer short
  r.feed(LO, HI, MID, LO, LO);
  TEST_ASSERT_FALSE(r.m.takeEvents().sePress);
}

static void test_se_gap_while_pressed_cancels_without_action() {
  Rig r;
  r.safeIdle();
  press(r, 3);
  r.feed(LO, HI, MID, LO, HI, 300 * MS); // pressed again after a 300 ms gap: cancelled, back to "release first"
  r.feed(LO, HI, MID, LO, LO);           // released: arms only
  TEST_ASSERT_FALSE(r.m.takeEvents().sePress);
  press(r, 3);
  r.feed(LO, HI, MID, LO, LO);
  TEST_ASSERT_TRUE(r.m.takeEvents().sePress); // a clean press works again
}

static void test_se_lost_release_sample_during_hold_cannot_become_a_reset() {
  Rig r;
  r.safeIdle();
  press(r, 30); // 1.2 s pressed: hold started
  TEST_ASSERT_TRUE(r.m.hold().active);
  const uint32_t serial = r.m.hold().serial;
  r.feed(LO, HI, MID, LO, LO, 400 * MS); // the release arrives after a gap (several samples lost)
  const RcEvents e = r.m.takeEvents();
  TEST_ASSERT_FALSE(e.sePress);
  TEST_ASSERT_FALSE(e.holdReleased); // the gap cancelled the hold: the consumer ends it by staleness
  TEST_ASSERT_FALSE(r.m.hold().active);
  TEST_ASSERT_EQUAL_UINT32(serial, r.m.hold().serial);
}

static void test_se_lost_release_sample_in_a_short_press_still_a_short_press() {
  Rig r;
  r.safeIdle();
  press(r, 5);
  r.feed(LO, HI, MID, LO, LO, 80 * MS); // one release sample lost, the next is 80 ms later (below the 200 ms gap)
  TEST_ASSERT_TRUE(r.m.takeEvents().sePress);
}

static void test_se_hold_starts_after_one_second_and_release_event() {
  Rig r;
  r.safeIdle();
  press(r, 20); // 800 ms
  TEST_ASSERT_FALSE(r.m.hold().active);
  press(r, 8);  // past 1 s
  TEST_ASSERT_TRUE(r.m.hold().active);
  const RcHoldState h = r.m.hold();
  TEST_ASSERT_TRUE(h.serial != 0);
  press(r, 10);
  TEST_ASSERT_EQUAL_INT64(r.t, r.m.hold().lastNs);
  TEST_ASSERT_EQUAL_INT64(h.startNs, r.m.hold().startNs);
  r.feed(LO, HI, MID, LO, LO);
  const RcEvents e = r.m.takeEvents();
  TEST_ASSERT_TRUE(e.holdReleased);
  TEST_ASSERT_EQUAL_UINT32(h.serial, e.holdReleasedSerial);
  TEST_ASSERT_FALSE(e.sePress); // a hold never resets
  TEST_ASSERT_FALSE(r.m.hold().active);
}

static void test_se_stuck_pressed_for_a_long_time_never_resets() {
  Rig r;
  r.safeIdle();
  press(r, 25 * 70); // 70 s
  TEST_ASSERT_TRUE(r.m.hold().active);
  TEST_ASSERT_FALSE(r.m.takeEvents().sePress);
}

static void test_se_session_change_cancels_a_hold() {
  Rig r;
  r.safeIdle();
  press(r, 30);
  TEST_ASSERT_TRUE(r.m.hold().active);
  r.m.onSessionChanged();
  TEST_ASSERT_FALSE(r.m.hold().active);
}

// --- command-intent layer ---

static RcPollInput readyInput(const Rig &r) {
  RcPollInput in;
  in.nowNs = r.t;
  in.haveSource = true;
  in.decoded = r.m.decoded();
  in.onGround = true;
  in.go.modeReady = true;
  in.go.zeroValid = true;
  in.simulationApplied = true;
  return in;
}

static void test_go_accepted_with_all_preconditions() {
  Rig r;
  RcCommandLogic L;
  r.hold(3, LO, HI, LO, LO, LO); // SC 0 = simulation, applied simulation
  r.feed(HI, HI, LO, LO, LO);
  const RcIntents o = L.evaluate(r.m, readyInput(r));
  TEST_ASSERT_TRUE(o.go);
  TEST_ASSERT_EQUAL_UINT8(2, o.goProfile);
  TEST_ASSERT_TRUE(o.goRefused == GoRefusal::None);
  TEST_ASSERT_EQUAL_UINT8(0, static_cast<uint8_t>(L.lastGoRefusal()));
}

static void test_go_countdown_is_15s_and_accepted_by_the_calibration_gate() {
  TEST_ASSERT_EQUAL_INT(-15, kGoCountdownParamInt);
  TEST_ASSERT_TRUE(VCTR::Calibration::countdownLongEnough(kGoCountdownParamInt));
  TEST_ASSERT_EQUAL_INT(kGoCountdownParamInt, VCTR::Calibration::kMinCountdownParamInt);
}

static void test_go_interlock_is_not_a_precondition() {
  Rig r;
  RcCommandLogic L;
  r.hold(3, LO, HI, LO, LO, LO);
  r.feed(HI, HI, LO, HI, LO); // SD ENABLED as well
  RcPollInput in = readyInput(r);
  in.interlockOpen = true;
  TEST_ASSERT_TRUE(L.evaluate(r.m, in).go);
  Rig q;
  RcCommandLogic M;
  q.hold(3, LO, HI, LO, LO, LO);
  q.feed(HI, HI, LO, LO, LO);
  in = readyInput(q);
  in.interlockOpen = false; // closed: still a go
  TEST_ASSERT_TRUE(M.evaluate(q.m, in).go);
}

static GoRefusal refusalFor(void (*tweak)(RcPollInput &), int16_t sb = HI, int16_t sc = LO) {
  Rig r;
  RcCommandLogic L;
  r.hold(3, LO, sb, sc, LO, LO);
  r.feed(HI, sb, sc, LO, LO);
  RcPollInput in = readyInput(r);
  tweak(in);
  const RcIntents o = L.evaluate(r.m, in);
  TEST_ASSERT_FALSE(o.go);
  TEST_ASSERT_TRUE(o.goEdgeSeen);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(o.goRefused), static_cast<uint8_t>(L.lastGoRefusal()));
  return o.goRefused;
}

static void test_go_refusal_reasons() {
  TEST_ASSERT_TRUE(refusalFor([](RcPollInput &i) { i.go.modeReady = false; }) == GoRefusal::NotReady);
  TEST_ASSERT_TRUE(refusalFor([](RcPollInput &i) { i.go.latched = true; i.go.modeReady = false; }) == GoRefusal::Latched);
  TEST_ASSERT_TRUE(refusalFor([](RcPollInput &i) { i.go.zeroingActive = true; }) == GoRefusal::Zeroing);
  TEST_ASSERT_TRUE(refusalFor([](RcPollInput &i) { i.go.zeroValid = false; }) == GoRefusal::Zeroing);
  TEST_ASSERT_TRUE(refusalFor([](RcPollInput &i) { i.go.requestBusy = true; }) == GoRefusal::RequestBusy);
  // vehicle_mode SIMULATION while the applied mode is real
  TEST_ASSERT_TRUE(refusalFor([](RcPollInput &i) { i.simulationApplied = false; }) == GoRefusal::ScMismatch);
  // vehicle_mode REAL while the applied mode is simulation
  TEST_ASSERT_TRUE(refusalFor([](RcPollInput &) {}, HI, MID) == GoRefusal::ScMismatch);
  // profiles 0, 1 and 2 are all accepted since the test missions (test_mission_profile covers values above 2)
}

static void test_go_needs_sa_still_at_go_when_evaluated() {
  Rig r;
  RcCommandLogic L;
  r.hold(3, LO, HI, LO, LO, LO);
  r.feed(HI, HI, LO, LO, LO);
  r.feed(LO, HI, LO, LO, LO); // SA back to NOT_GO before the tick
  const RcIntents o = L.evaluate(r.m, readyInput(r));
  TEST_ASSERT_FALSE(o.go);
  TEST_ASSERT_TRUE(o.goRefused == GoRefusal::None);
}

static void test_sa_stuck_go_gives_one_go_only() {
  Rig r;
  RcCommandLogic L;
  r.hold(3, LO, HI, LO, LO, LO);
  r.feed(HI, HI, LO, LO, LO);
  TEST_ASSERT_TRUE(L.evaluate(r.m, readyInput(r)).go);
  for (int i = 0; i < 50; i++) {
    r.hold(3, HI, HI, LO, LO, LO);
    TEST_ASSERT_FALSE(L.evaluate(r.m, readyInput(r)).go);
  }
}

static void test_abort_level_every_tick_while_pending_or_running_and_sa_released() {
  Rig r;
  RcCommandLogic L;
  r.hold(3, HI, HI, LO, LO, LO);
  RcPollInput in = readyInput(r);
  in.chainPendingOrRunning = true;
  TEST_ASSERT_FALSE(L.evaluate(r.m, in).abortLevel); // SA GO
  for (int i = 0; i < 5; i++) {
    r.feed(LO, HI, LO, LO, LO);
    in = readyInput(r);
    in.chainPendingOrRunning = true;
    TEST_ASSERT_TRUE(L.evaluate(r.m, in).abortLevel); // every tick
  }
  r.feed(LO, HI, LO, LO, LO);
  in = readyInput(r);
  in.chainPendingOrRunning = false; // nothing pending: nothing to abort
  TEST_ASSERT_FALSE(L.evaluate(r.m, in).abortLevel);
  // a stale source does not abort (the control failsafe covers it)
  in.haveSource = false;
  TEST_ASSERT_FALSE(L.evaluate(r.m, in).abortLevel);
}

static void test_sc_decisions() {
  ScContext c;
  c.onGround = true;
  c.simulationApplied = false;
  TEST_ASSERT_TRUE(decideSc(1, c) == ScAction::EnterSimulation); // edge to SIMULATION
  TEST_ASSERT_TRUE(decideSc(0, c) == ScAction::None); // already real
  c.simulationApplied = true;
  TEST_ASSERT_TRUE(decideSc(0, c) == ScAction::EnterReal);
  TEST_ASSERT_TRUE(decideSc(RcEvents::kNone, c) == ScAction::None);
  c.committed = true;
  TEST_ASSERT_TRUE(decideSc(0, c) == ScAction::Refused);
  c.committed = false;
  c.onGround = false;
  TEST_ASSERT_TRUE(decideSc(0, c) == ScAction::Refused);
  c.onGround = true;
  c.modeFailure = true; // a mode switch would re-initialise and clear a latched failsafe
  TEST_ASSERT_TRUE(decideSc(0, c) == ScAction::Refused);
}

static void test_sc_mismatch_flag_rule() {
  TEST_ASSERT_FALSE(RcCommandLogic::scMismatch(false, 0, true)); // no session: no flag
  TEST_ASSERT_FALSE(RcCommandLogic::scMismatch(true, 1, true));  // SIMULATION applied
  TEST_ASSERT_FALSE(RcCommandLogic::scMismatch(true, 0, false)); // REAL applied
  TEST_ASSERT_TRUE(RcCommandLogic::scMismatch(true, 0, true));
  TEST_ASSERT_TRUE(RcCommandLogic::scMismatch(true, 1, false));
}

static void test_on_ground_rule_d12() {
  GroundContext g;
  g.modeGround = true;
  g.missionIdleOrFinished = true;
  TEST_ASSERT_TRUE(onGround(g));
  g.interlockOpen = true;
  TEST_ASSERT_FALSE(onGround(g));
  g.interlockOpen = false;
  g.motorsEnabled = true;
  TEST_ASSERT_FALSE(onGround(g));
  g.motorsEnabled = false;
  g.missionIdleOrFinished = false; // a mission is running
  TEST_ASSERT_FALSE(onGround(g));
  GroundContext f; // Failure: actuators are cut, the reset works whatever the mission says
  f.modeFailure = true;
  TEST_ASSERT_TRUE(onGround(f));
  f.interlockOpen = true;
  TEST_ASSERT_FALSE(onGround(f));
  GroundContext run; // Running mode, mission still active: in flight
  run.modeRunning = true;
  run.chainPending = true;
  run.missionIdleOrFinished = false;
  TEST_ASSERT_FALSE(onGround(run));
  run.missionIdleOrFinished = true; // pending chain, mission momentarily Finished: still in flight
  TEST_ASSERT_FALSE(onGround(run));
  run.chainPending = false; // chain ended (landed): the reset must be possible
  TEST_ASSERT_TRUE(onGround(run));
}

static GroundContext landedRunning() { // mode Running after an RTH landed, nothing pending, interlock closed
  GroundContext g;
  g.modeRunning = true;
  g.missionIdleOrFinished = true;
  return g;
}

static void test_committed_landed_running_is_not_committed() {
  GroundContext g = landedRunning();
  TEST_ASSERT_FALSE(committed(g));
  g.missionIdleOrFinished = true; // Finished or Idle are both the same here
  TEST_ASSERT_FALSE(committed(g));
  GroundContext boot; // Startup/Ready on the ground
  boot.modeGround = true;
  boot.missionIdleOrFinished = true;
  TEST_ASSERT_FALSE(committed(boot));
  GroundContext fail; // Failure latched, actuators cut
  fail.modeFailure = true;
  TEST_ASSERT_FALSE(committed(fail));
}

static void test_committed_in_flight_countdown_and_interlock() {
  GroundContext g = landedRunning();
  g.missionIdleOrFinished = false; // mid-flight: mission active
  TEST_ASSERT_TRUE(committed(g));
  g = landedRunning();
  g.chainPending = true; // chain pending, mission momentarily Finished
  TEST_ASSERT_TRUE(committed(g));
  g = landedRunning();
  g.motorsEnabled = true; // motors on: not landed
  TEST_ASSERT_TRUE(committed(g));
  g = landedRunning();
  g.interlockOpen = true; // interlock open: always, even with the mission Finished
  TEST_ASSERT_TRUE(committed(g));
  GroundContext cd; // countdown: chain pending in Ready/Startup, mission Idle
  cd.modeGround = true;
  cd.missionIdleOrFinished = true;
  cd.chainPending = true;
  TEST_ASSERT_TRUE(committed(cd));
  GroundContext il; // interlock open on the ground, mission Finished
  il.modeGround = true;
  il.missionIdleOrFinished = true;
  il.interlockOpen = true;
  TEST_ASSERT_TRUE(committed(il));
}

static void test_se_reset_only_on_the_ground_with_interlock_closed() {
  for (int variant = 0; variant < 4; variant++) {
    Rig r;
    RcCommandLogic L;
    r.safeIdle();
    press(r, 5);
    r.feed(LO, HI, MID, LO, LO);
    RcPollInput in = readyInput(r);
    if (variant == 1) in.onGround = false;
    if (variant == 2) in.interlockOpen = true;
    if (variant == 3) in.decoded.failsafe = 1; // owner Q6: the GS failsafe request must be cleared first
    const RcIntents o = L.evaluate(r.m, in);
    if (variant == 0) {
      TEST_ASSERT_TRUE(o.se == SeAction::Reset);
    } else {
      TEST_ASSERT_TRUE(o.se == SeAction::Refused);
      TEST_ASSERT_TRUE(L.seRefusedActive(in.nowNs + 1900 * MS));
      TEST_ASSERT_FALSE(L.seRefusedActive(in.nowNs + 2100 * MS));
    }
  }
}

static void test_hold_intents_start_refresh_release() {
  Rig r;
  RcCommandLogic L;
  r.safeIdle();
  press(r, 30);
  RcPollInput in = readyInput(r);
  RcIntents o = L.evaluate(r.m, in);
  TEST_ASSERT_TRUE(o.hold == HoldAction::Start);
  L.holdStartResult(true, in.nowNs);
  press(r, 5);
  in = readyInput(r);
  in.zeroingHoldActive = true;
  o = L.evaluate(r.m, in);
  TEST_ASSERT_TRUE(o.hold == HoldAction::Refresh);
  TEST_ASSERT_EQUAL_INT64(r.t, o.holdNs);
  r.feed(LO, HI, MID, LO, LO);
  in = readyInput(r);
  in.zeroingHoldActive = true;
  o = L.evaluate(r.m, in);
  TEST_ASSERT_TRUE(o.hold == HoldAction::Release);
  TEST_ASSERT_EQUAL_INT64(r.t, o.holdNs);
  TEST_ASSERT_FALSE(o.se == SeAction::Reset);
  TEST_ASSERT_FALSE(L.holdRunning());
}

static void test_hold_refused_is_not_retried_and_shows_se_refused() {
  Rig r;
  RcCommandLogic L;
  r.safeIdle();
  press(r, 30);
  RcPollInput in = readyInput(r);
  TEST_ASSERT_TRUE(L.evaluate(r.m, in).hold == HoldAction::Start);
  L.holdStartResult(false, in.nowNs); // refused (for example armed)
  TEST_ASSERT_TRUE(L.seRefusedActive(in.nowNs));
  for (int i = 0; i < 20; i++) {
    press(r, 3);
    in = readyInput(r);
    TEST_ASSERT_TRUE(L.evaluate(r.m, in).hold == HoldAction::None); // the same hold is not retried each tick
  }
}

static void test_hold_aborts_when_the_interlock_opens() {
  Rig r;
  RcCommandLogic L;
  r.safeIdle();
  press(r, 30);
  RcPollInput in = readyInput(r);
  L.evaluate(r.m, in);
  L.holdStartResult(true, in.nowNs);
  press(r, 3);
  in = readyInput(r);
  in.zeroingHoldActive = true;
  in.interlockOpen = true;
  TEST_ASSERT_TRUE(L.evaluate(r.m, in).hold == HoldAction::Abort);
  TEST_ASSERT_FALSE(L.holdRunning());
}

static void test_hold_that_ended_by_staleness_is_forgotten() {
  Rig r;
  RcCommandLogic L;
  r.safeIdle();
  press(r, 30);
  RcPollInput in = readyInput(r);
  L.evaluate(r.m, in);
  L.holdStartResult(true, in.nowNs);
  in = readyInput(r);
  in.zeroingHoldActive = false; // EstimatorZeroing ended it (staleness or the 60 s cap)
  TEST_ASSERT_TRUE(L.evaluate(r.m, in).hold == HoldAction::None);
  TEST_ASSERT_FALSE(L.holdRunning());
}

static void test_hold_start_only_with_fresh_samples() {
  Rig r;
  RcCommandLogic L;
  r.safeIdle();
  press(r, 30);
  RcPollInput in = readyInput(r);
  in.nowNs = r.t + 500 * MS; // the safety tick is late: the last held sample is older than the grace
  TEST_ASSERT_TRUE(L.evaluate(r.m, in).hold == HoldAction::None);
}

static void test_stuck_fields_never_act_after_a_session_start() {
  // A new session starts with go GO, REAL, motors ENABLED, reset PRESSED: nothing until cycled.
  Rig r;
  RcCommandLogic L;
  r.hold(200, HI, HI, MID, HI, HI);
  RcPollInput in = readyInput(r);
  const RcIntents o = L.evaluate(r.m, in);
  TEST_ASSERT_FALSE(o.go);
  TEST_ASSERT_FALSE(o.goEdgeSeen);
  TEST_ASSERT_TRUE(o.sc == ScAction::None);
  TEST_ASSERT_TRUE(o.se == SeAction::None);
  TEST_ASSERT_TRUE(o.hold == HoldAction::None);
}

static void test_decode_maps_every_field() {
  CmdFrame f;
  f.go = 1;
  f.profile = 2;
  f.vehicleMode = 1;
  f.motors = 1;
  f.resetButton = 1;
  f.failsafe = 1;
  const RcDecoded d = RcCommandMapper::decode(f);
  TEST_ASSERT_EQUAL_UINT8(1, d.go);
  TEST_ASSERT_EQUAL_UINT8(2, d.profile);
  TEST_ASSERT_EQUAL_UINT8(1, d.mode);
  TEST_ASSERT_EQUAL_UINT8(1, d.motors);
  TEST_ASSERT_EQUAL_UINT8(1, d.reset);
  TEST_ASSERT_EQUAL_UINT8(1, d.failsafe);
}

static void test_safe_seen_bits_follow_safe_values() {
  Rig r;
  r.feed(HI, LO, LO, HI, HI); // GO, SIMULATION, ENABLED, PRESSED: all unsafe
  TEST_ASSERT_EQUAL_UINT8(0, r.m.safeSeen());
  r.feed(LO, LO, MID, HI, HI); // go NOT_GO, REAL
  TEST_ASSERT_EQUAL_UINT8(kSafeSeenGo | kSafeSeenScReal, r.m.safeSeen());
  r.feed(HI, LO, LO, LO, LO); // motors DISABLED, reset RELEASED
  TEST_ASSERT_EQUAL_UINT8(0x0F, r.m.safeSeen()); // bits stay when the field moves to the unsafe value
  TEST_ASSERT_EQUAL_UINT8(1, r.m.decoded().go);
}

static void test_safe_seen_cleared_by_gap_and_session_change() {
  Rig r;
  r.feed(LO, LO, MID, LO, LO);
  r.feed(HI, LO, LO, HI, HI);
  TEST_ASSERT_EQUAL_UINT8(0x0F, r.m.safeSeen());
  r.feed(HI, LO, LO, HI, HI, 2 * S); // a gap of exactly 2 s keeps the bits
  TEST_ASSERT_EQUAL_UINT8(0x0F, r.m.safeSeen());
  r.feed(HI, LO, LO, HI, HI, 2 * S + 1); // longer clears them
  TEST_ASSERT_EQUAL_UINT8(0, r.m.safeSeen());
  r.feed(LO, LO, MID, LO, LO);
  TEST_ASSERT_EQUAL_UINT8(0x0F, r.m.safeSeen());
  r.m.onSessionChanged();
  TEST_ASSERT_EQUAL_UINT8(0, r.m.safeSeen());
  TEST_ASSERT_FALSE(r.m.seen());
}

static void test_session_change_after_a_pending_edge_drops_the_event() {
  Rig r;
  r.hold(3, LO, HI, MID, LO, LO);
  r.feed(HI, HI, MID, LO, LO); // a go edge is pending
  r.m.onSessionChanged();      // a new session latched before the tick: the event is gone
  TEST_ASSERT_FALSE(r.m.takeEvents().goEdge);
}

static void test_se_reset_refused_while_gs_failsafe_requested_then_accepted_when_cleared() {
  Rig r;
  RcCommandLogic L;
  r.safeIdle();
  press(r, 5);
  r.feed(LO, HI, MID, LO, LO);
  RcPollInput in = readyInput(r);
  in.decoded.failsafe = 1;
  const RcIntents o1 = L.evaluate(r.m, in);
  TEST_ASSERT_TRUE(o1.se == SeAction::Refused);
  press(r, 5); // pressed again with the request cleared
  r.feed(LO, HI, MID, LO, LO);
  in = readyInput(r);
  in.decoded.failsafe = 0;
  TEST_ASSERT_TRUE(L.evaluate(r.m, in).se == SeAction::Reset);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_go_edge_needs_observed_not_go_then_go);
  RUN_TEST(test_go_edge_is_latched_between_ticks);
  RUN_TEST(test_boot_with_every_switch_combination_does_nothing);
  RUN_TEST(test_each_switch_must_be_cycled_after_boot);
  RUN_TEST(test_sc_only_an_observed_change_acts_both_directions);
  RUN_TEST(test_sc_simulation_at_session_start_makes_no_edge);
  RUN_TEST(test_gap_over_one_second_or_session_change_forgets_the_base);
  RUN_TEST(test_se_short_press_resets);
  RUN_TEST(test_se_single_sample_is_ignored);
  RUN_TEST(test_se_press_without_release_seen_first_does_nothing);
  RUN_TEST(test_se_press_of_one_second_or_more_is_not_a_short_press);
  RUN_TEST(test_se_gap_while_pressed_cancels_without_action);
  RUN_TEST(test_se_lost_release_sample_during_hold_cannot_become_a_reset);
  RUN_TEST(test_se_lost_release_sample_in_a_short_press_still_a_short_press);
  RUN_TEST(test_se_hold_starts_after_one_second_and_release_event);
  RUN_TEST(test_se_stuck_pressed_for_a_long_time_never_resets);
  RUN_TEST(test_se_session_change_cancels_a_hold);
  RUN_TEST(test_go_accepted_with_all_preconditions);
  RUN_TEST(test_go_countdown_is_15s_and_accepted_by_the_calibration_gate);
  RUN_TEST(test_go_interlock_is_not_a_precondition);
  RUN_TEST(test_go_refusal_reasons);
  RUN_TEST(test_go_needs_sa_still_at_go_when_evaluated);
  RUN_TEST(test_sa_stuck_go_gives_one_go_only);
  RUN_TEST(test_abort_level_every_tick_while_pending_or_running_and_sa_released);
  RUN_TEST(test_sc_decisions);
  RUN_TEST(test_sc_mismatch_flag_rule);
  RUN_TEST(test_on_ground_rule_d12);
  RUN_TEST(test_committed_landed_running_is_not_committed);
  RUN_TEST(test_committed_in_flight_countdown_and_interlock);
  RUN_TEST(test_se_reset_only_on_the_ground_with_interlock_closed);
  RUN_TEST(test_hold_intents_start_refresh_release);
  RUN_TEST(test_hold_refused_is_not_retried_and_shows_se_refused);
  RUN_TEST(test_hold_aborts_when_the_interlock_opens);
  RUN_TEST(test_hold_that_ended_by_staleness_is_forgotten);
  RUN_TEST(test_hold_start_only_with_fresh_samples);
  RUN_TEST(test_stuck_fields_never_act_after_a_session_start);
  RUN_TEST(test_decode_maps_every_field);
  RUN_TEST(test_safe_seen_bits_follow_safe_values);
  RUN_TEST(test_safe_seen_cleared_by_gap_and_session_change);
  RUN_TEST(test_session_change_after_a_pending_edge_drops_the_event);
  RUN_TEST(test_se_reset_refused_while_gs_failsafe_requested_then_accepted_when_cleared);
  return UNITY_END();
}
