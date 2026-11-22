// Host tests of src/safety/rc_mapper.cpp and src/safety/rc_command.cpp (T9b-1 decode, T9b-4 edge events, SE classifier,
// command-intent layer): pio test -e native -f test_rc_mapper
#include <unity.h>

#include <initializer_list>

#include "calibration/calibration_validity.hpp"
#include "safety/rc_command.hpp"
#include "safety/rc_mapper.hpp"

using namespace VCTR::Safety;
static constexpr int64_t S = 1000000000LL;

void setUp() {}
void tearDown() {}

static RcFrame frame(int16_t sa, int16_t sb, int16_t sc, int16_t sd, int16_t se) {
  RcFrame f;
  f.flags = RcFrame::kFlagSourceOk;
  f.ch[4] = sa;
  f.ch[5] = sb;
  f.ch[6] = sc;
  f.ch[7] = sd;
  f.ch[8] = se;
  return f;
}

void test_two_position_bands() {
  TEST_ASSERT_EQUAL_UINT8(0, RcCommandMapper::decodeTwoPos(-1000));
  TEST_ASSERT_EQUAL_UINT8(0, RcCommandMapper::decodeTwoPos(-500));
  TEST_ASSERT_EQUAL_UINT8(255, RcCommandMapper::decodeTwoPos(-499));
  TEST_ASSERT_EQUAL_UINT8(255, RcCommandMapper::decodeTwoPos(0));
  TEST_ASSERT_EQUAL_UINT8(255, RcCommandMapper::decodeTwoPos(499));
  TEST_ASSERT_EQUAL_UINT8(1, RcCommandMapper::decodeTwoPos(500));
  TEST_ASSERT_EQUAL_UINT8(1, RcCommandMapper::decodeTwoPos(1000));
}

void test_three_position_bands() {
  TEST_ASSERT_EQUAL_UINT8(0, RcCommandMapper::decodeThreePos(-1000));
  TEST_ASSERT_EQUAL_UINT8(0, RcCommandMapper::decodeThreePos(-500));
  TEST_ASSERT_EQUAL_UINT8(255, RcCommandMapper::decodeThreePos(-499));
  TEST_ASSERT_EQUAL_UINT8(255, RcCommandMapper::decodeThreePos(-201));
  TEST_ASSERT_EQUAL_UINT8(1, RcCommandMapper::decodeThreePos(-200));
  TEST_ASSERT_EQUAL_UINT8(1, RcCommandMapper::decodeThreePos(0));
  TEST_ASSERT_EQUAL_UINT8(1, RcCommandMapper::decodeThreePos(200));
  TEST_ASSERT_EQUAL_UINT8(255, RcCommandMapper::decodeThreePos(201));
  TEST_ASSERT_EQUAL_UINT8(255, RcCommandMapper::decodeThreePos(499));
  TEST_ASSERT_EQUAL_UINT8(2, RcCommandMapper::decodeThreePos(500));
  TEST_ASSERT_EQUAL_UINT8(2, RcCommandMapper::decodeThreePos(1000));
}

void test_invalid_channel_and_out_of_range_are_invalid_not_low() {
  // CH_INVALID = -32768 is far below -500 but must NOT decode as position 0
  TEST_ASSERT_EQUAL_UINT8(255, RcCommandMapper::decodeTwoPos(RcFrame::kChInvalid));
  TEST_ASSERT_EQUAL_UINT8(255, RcCommandMapper::decodeThreePos(RcFrame::kChInvalid));
  TEST_ASSERT_EQUAL_UINT8(255, RcCommandMapper::decodeTwoPos(1001)); // never GO from garbage
  TEST_ASSERT_EQUAL_UINT8(255, RcCommandMapper::decodeTwoPos(32767));
  TEST_ASSERT_EQUAL_UINT8(255, RcCommandMapper::decodeTwoPos(-1001));
  TEST_ASSERT_EQUAL_UINT8(255, RcCommandMapper::decodeThreePos(1001));
}

void test_pocket_and_gs_values_decode() {
  // CRSF mapping: -1000 / 0 / +1000; GS 1000/1500/2000 us -> -1000 / 0 / +1000
  RcDecoded d = RcCommandMapper::decode(frame(1000, 1000, 0, -1000, 1000));
  TEST_ASSERT_EQUAL_UINT8(1, d.sa);
  TEST_ASSERT_EQUAL_UINT8(2, d.sb);
  TEST_ASSERT_EQUAL_UINT8(1, d.sc);
  TEST_ASSERT_EQUAL_UINT8(0, d.sd);
  TEST_ASSERT_EQUAL_UINT8(1, d.se);
}

void test_sticks_and_other_channels_do_not_matter() {
  RcFrame f = frame(-1000, -1000, -1000, -1000, -1000);
  for (int i : {0, 1, 2, 3, 9, 10, 15}) f.ch[i] = 1000;
  RcDecoded d = RcCommandMapper::decode(f);
  TEST_ASSERT_EQUAL_UINT8(0, d.sa);
  TEST_ASSERT_EQUAL_UINT8(0, d.sd);
  TEST_ASSERT_EQUAL_UINT8(0, d.se);
}

void test_invalid_is_safe_for_actions() {
  RcFrame f = frame(RcFrame::kChInvalid, RcFrame::kChInvalid, RcFrame::kChInvalid, 100, 300);
  RcDecoded d = RcCommandMapper::decode(f);
  TEST_ASSERT_EQUAL_UINT8(255, d.sa);
  TEST_ASSERT_EQUAL_UINT8(0, d.saSafe());
  TEST_ASSERT_EQUAL_UINT8(255, d.sb);
  TEST_ASSERT_EQUAL_UINT8(255, d.sc);
  TEST_ASSERT_EQUAL_UINT8(0, d.sdSafe());
  TEST_ASSERT_EQUAL_UINT8(0, d.seSafe());
}

void test_sc_position_two_is_reported_as_reserved() {
  TEST_ASSERT_EQUAL_UINT8(2, RcCommandMapper::decode(frame(-1000, 0, 1000, -1000, -1000)).sc);
}

void test_safe_seen_bits_set_only_by_explicit_safe_values() {
  RcCommandMapper m;
  const ControlSourceId R = ControlSourceId::Radio;
  m.onAccepted(R, frame(1000, 0, 1000, 1000, 1000), 1 * S); // all unsafe
  TEST_ASSERT_EQUAL_UINT8(0, m.safeSeen(R));
  m.onAccepted(R, frame(RcFrame::kChInvalid, 0, 0, -1000, -1000), 1 * S + 40000000);
  // invalid SA does not count as seen; SC=1 (pos 1) is not the safe value 0; SD 0 and SE 0 are
  TEST_ASSERT_EQUAL_UINT8(kSafeSeenSdEnable | kSafeSeenSePress, m.safeSeen(R));
  m.onAccepted(R, frame(-1000, 0, -1000, 1000, 1000), 1 * S + 80000000);
  TEST_ASSERT_EQUAL_UINT8(kSafeSeenGo | kSafeSeenScReal | kSafeSeenSdEnable | kSafeSeenSePress, m.safeSeen(R));
  // bits stay set when the switch moves to the unsafe value (that is the observed edge)
  TEST_ASSERT_EQUAL_UINT8(1, m.decoded(R).sd);
}

void test_safe_seen_cleared_by_gap_and_source_change() {
  RcCommandMapper m;
  const ControlSourceId R = ControlSourceId::Radio, B = ControlSourceId::Bench;
  m.onAccepted(R, frame(-1000, 0, -1000, -1000, -1000), 1 * S);
  m.onAccepted(R, frame(1000, 0, 1000, 1000, 1000), 1 * S + 100000000);
  TEST_ASSERT_EQUAL_UINT8(0x0F, m.safeSeen(R));
  // gap of exactly 2 s keeps the bits, longer clears them (and the new state, unsafe, sets none)
  m.onAccepted(R, frame(1000, 0, 1000, 1000, 1000), 3 * S + 100000000);
  TEST_ASSERT_EQUAL_UINT8(0x0F, m.safeSeen(R));
  m.onAccepted(R, frame(1000, 0, 1000, 1000, 1000), 5 * S + 100000001);
  TEST_ASSERT_EQUAL_UINT8(0, m.safeSeen(R));
  m.onAccepted(B, frame(-1000, 0, -1000, -1000, -1000), 6 * S);
  TEST_ASSERT_EQUAL_UINT8(0x0F, m.safeSeen(B));
  m.onActiveChanged();
  TEST_ASSERT_EQUAL_UINT8(0, m.safeSeen(B));
  TEST_ASSERT_EQUAL_UINT8(0, m.safeSeen(R));
  m.onAccepted(B, frame(-1000, 0, -1000, -1000, -1000), 6 * S + 40000000);
  TEST_ASSERT_EQUAL_UINT8(0x0F, m.safeSeen(B));
}

void test_none_source_and_unseen_report_all_invalid() {
  RcCommandMapper m;
  m.onAccepted(ControlSourceId::None, frame(1000, 1000, 1000, 1000, 1000), 1 * S);
  RcDecoded d = m.decoded(ControlSourceId::None);
  TEST_ASSERT_EQUAL_UINT8(255, d.sa);
  d = m.decoded(ControlSourceId::Radio);
  TEST_ASSERT_EQUAL_UINT8(255, d.se);
  TEST_ASSERT_EQUAL_UINT8(0, m.safeSeen(ControlSourceId::None));
}


// ======================= T9b-4: edge events, SE classifier, command-intent layer =============================

static constexpr int64_t MS = 1000000LL;
static constexpr int16_t LO = -1000, HI = 1000, MID = 0;

/// Feeds the mapper like the comms task: states of one source at 25 Hz (40 ms), the source is the active one.
struct Rig {
  RcCommandMapper m;
  int64_t t = 100 * S;
  ControlSourceId src = ControlSourceId::Radio;
  explicit Rig(ControlSourceId s = ControlSourceId::Radio) : src(s) { m.onActiveChanged(src); }
  void feed(int16_t sa, int16_t sb, int16_t sc, int16_t sd, int16_t se, int64_t dt = 40 * MS) {
    t += dt;
    m.onAccepted(src, frame(sa, sb, sc, sd, se), t);
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
  TEST_ASSERT_EQUAL_UINT8(2, e.goSb);
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
  r.hold(5, HI, HI, MID, HI, HI); // SA GO, SC 1, SD ENABLED, SE PRESSED at boot
  r.m.takeEvents();
  r.feed(LO, HI, LO, LO, LO); // everything to its safe value, SC to 0 (the first change)
  const RcEvents e0 = r.m.takeEvents();
  TEST_ASSERT_EQUAL_UINT8(0, e0.scEdgeTo); // 1 -> 0 is an observed change (simulation)
  r.feed(HI, HI, LO, LO, LO); // SA to GO: edge
  TEST_ASSERT_TRUE(r.m.takeEvents().goEdge);
  r.feed(HI, HI, MID, LO, LO); // SC 0 -> 1: edge to real (position 0 was seen)
  TEST_ASSERT_EQUAL_UINT8(1, r.m.takeEvents().scEdgeTo);
}

static void test_sc_into_real_needs_zero_first_and_sim_needs_a_change() {
  Rig r;
  r.hold(3, LO, HI, MID, LO, LO); // SC 1 at boot
  TEST_ASSERT_EQUAL_UINT8(RcEvents::kNone, r.m.takeEvents().scEdgeTo);
  r.feed(LO, HI, LO, LO, LO); // 1 -> 0
  TEST_ASSERT_EQUAL_UINT8(0, r.m.takeEvents().scEdgeTo);
  r.hold(3, LO, HI, LO, LO, LO);
  TEST_ASSERT_EQUAL_UINT8(RcEvents::kNone, r.m.takeEvents().scEdgeTo); // staying at 0: nothing
  r.feed(LO, HI, MID, LO, LO); // 0 -> 1
  TEST_ASSERT_EQUAL_UINT8(1, r.m.takeEvents().scEdgeTo);
}

static void test_sc_reserved_and_invalid_positions_make_no_edge() {
  Rig r;
  r.hold(3, LO, HI, LO, LO, LO);
  r.feed(LO, HI, HI, LO, LO); // position 2 (reserved)
  TEST_ASSERT_EQUAL_UINT8(RcEvents::kNone, r.m.takeEvents().scEdgeTo);
  r.feed(LO, HI, MID, LO, LO); // 2 -> 1: out of a reserved position is no edge
  TEST_ASSERT_EQUAL_UINT8(RcEvents::kNone, r.m.takeEvents().scEdgeTo);
  r.feed(LO, HI, RcFrame::kChInvalid, LO, LO);
  r.feed(LO, HI, LO, LO, LO); // invalid -> 0: no edge
  TEST_ASSERT_EQUAL_UINT8(RcEvents::kNone, r.m.takeEvents().scEdgeTo);
}

static void test_gap_over_one_second_or_handover_forgets_the_base() {
  Rig r;
  r.hold(3, LO, HI, LO, LO, LO);
  r.feed(HI, HI, LO, LO, LO, 1100 * MS); // SA flips after a 1.1 s gap: previous state too old for an edge
  TEST_ASSERT_FALSE(r.m.takeEvents().goEdge);
  Rig q;
  q.hold(3, LO, HI, LO, LO, LO);
  q.m.onActiveChanged(ControlSourceId::Radio); // handover: all detectors forget
  q.feed(HI, HI, LO, LO, LO);
  TEST_ASSERT_FALSE(q.m.takeEvents().goEdge);
  q.feed(LO, HI, LO, LO, LO);
  q.feed(HI, HI, LO, LO, LO);
  TEST_ASSERT_TRUE(q.m.takeEvents().goEdge);
}

static void test_events_only_for_the_active_source() {
  Rig r(ControlSourceId::Radio);
  const ControlSourceId B = ControlSourceId::Bench;
  for (int i = 0; i < 3; i++) { r.t += 40 * MS; r.m.onAccepted(B, frame(LO, HI, MID, LO, LO), r.t); }
  r.t += 40 * MS;
  r.m.onAccepted(B, frame(HI, HI, MID, LO, LO), r.t); // GO on the ignored source
  TEST_ASSERT_FALSE(r.m.takeEvents().goEdge);
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

static void test_se_invalid_during_press_cancels() {
  Rig r;
  r.safeIdle();
  press(r, 3);
  r.feed(LO, HI, MID, LO, RcFrame::kChInvalid);
  r.feed(LO, HI, MID, LO, LO);
  TEST_ASSERT_FALSE(r.m.takeEvents().sePress);
}

static void test_se_source_change_cancels_a_hold() {
  Rig r;
  r.safeIdle();
  press(r, 30);
  TEST_ASSERT_TRUE(r.m.hold().active);
  r.m.onActiveChanged(ControlSourceId::Bench);
  TEST_ASSERT_FALSE(r.m.hold().active);
}

// --- command-intent layer ---

static RcPollInput readyInput(const Rig &r) {
  RcPollInput in;
  in.nowNs = r.t;
  in.haveSource = true;
  in.decoded = r.m.decoded(r.src);
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
  // SC differs from the applied mode (SC 0 = simulation, applied real)
  TEST_ASSERT_TRUE(refusalFor([](RcPollInput &i) { i.simulationApplied = false; }) == GoRefusal::ScMismatch);
  // SC reserved (position 2) or invalid
  TEST_ASSERT_TRUE(refusalFor([](RcPollInput &) {}, HI, HI) == GoRefusal::ScMismatch);
  TEST_ASSERT_TRUE(refusalFor([](RcPollInput &) {}, HI, RcFrame::kChInvalid) == GoRefusal::ScMismatch);
  // SB 0 and 1 are not implemented (D9), invalid SB too
  TEST_ASSERT_TRUE(refusalFor([](RcPollInput &) {}, LO) == GoRefusal::SbProfile);
  TEST_ASSERT_TRUE(refusalFor([](RcPollInput &) {}, MID) == GoRefusal::SbProfile);
  TEST_ASSERT_TRUE(refusalFor([](RcPollInput &) {}, 300) == GoRefusal::SbProfile);
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
  // invalid SA is the safe value (NOT_GO)
  r.feed(RcFrame::kChInvalid, HI, LO, LO, LO);
  in = readyInput(r);
  in.chainPendingOrRunning = true;
  TEST_ASSERT_TRUE(L.evaluate(r.m, in).abortLevel);
  // a stale source does not abort (the control failsafe covers it)
  in.haveSource = false;
  TEST_ASSERT_FALSE(L.evaluate(r.m, in).abortLevel);
}

static void test_sc_decisions() {
  ScContext c;
  c.onGround = true;
  c.simulationApplied = false;
  TEST_ASSERT_TRUE(decideSc(0, c) == ScAction::EnterSimulation);
  TEST_ASSERT_TRUE(decideSc(1, c) == ScAction::None); // already real
  c.simulationApplied = true;
  TEST_ASSERT_TRUE(decideSc(1, c) == ScAction::EnterReal);
  TEST_ASSERT_TRUE(decideSc(RcEvents::kNone, c) == ScAction::None);
  c.committed = true;
  TEST_ASSERT_TRUE(decideSc(1, c) == ScAction::Refused);
  c.committed = false;
  c.onGround = false;
  TEST_ASSERT_TRUE(decideSc(1, c) == ScAction::Refused);
  c.onGround = true;
  c.modeFailure = true; // a mode switch would re-initialise and clear a latched failsafe
  TEST_ASSERT_TRUE(decideSc(1, c) == ScAction::Refused);
}

static void test_sc_mismatch_flag_rule() {
  TEST_ASSERT_FALSE(RcCommandLogic::scMismatch(false, 255, true)); // no source: no flag
  TEST_ASSERT_FALSE(RcCommandLogic::scMismatch(true, 0, true));
  TEST_ASSERT_FALSE(RcCommandLogic::scMismatch(true, 1, false));
  TEST_ASSERT_TRUE(RcCommandLogic::scMismatch(true, 1, true));
  TEST_ASSERT_TRUE(RcCommandLogic::scMismatch(true, 0, false));
  TEST_ASSERT_TRUE(RcCommandLogic::scMismatch(true, 2, false));
  TEST_ASSERT_TRUE(RcCommandLogic::scMismatch(true, 255, true));
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

static void test_se_reset_only_on_the_ground_with_interlock_closed() {
  for (int variant = 0; variant < 3; variant++) {
    Rig r;
    RcCommandLogic L;
    r.safeIdle();
    press(r, 5);
    r.feed(LO, HI, MID, LO, LO);
    RcPollInput in = readyInput(r);
    if (variant == 1) in.onGround = false;
    if (variant == 2) in.interlockOpen = true;
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

static void test_stuck_switches_never_act_after_a_handover() {
  // The Pocket takes over (or the GS lease starts) with SA GO, SC 1, SD ENABLED, SE pressed: nothing until cycled.
  Rig r(ControlSourceId::Bench);
  RcCommandLogic L;
  r.m.onActiveChanged(ControlSourceId::Bench);
  r.hold(200, HI, HI, MID, HI, HI);
  RcPollInput in = readyInput(r);
  const RcIntents o = L.evaluate(r.m, in);
  TEST_ASSERT_FALSE(o.go);
  TEST_ASSERT_FALSE(o.goEdgeSeen);
  TEST_ASSERT_TRUE(o.sc == ScAction::None);
  TEST_ASSERT_TRUE(o.se == SeAction::None);
  TEST_ASSERT_TRUE(o.hold == HoldAction::None);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_two_position_bands);
  RUN_TEST(test_three_position_bands);
  RUN_TEST(test_invalid_channel_and_out_of_range_are_invalid_not_low);
  RUN_TEST(test_pocket_and_gs_values_decode);
  RUN_TEST(test_sticks_and_other_channels_do_not_matter);
  RUN_TEST(test_invalid_is_safe_for_actions);
  RUN_TEST(test_sc_position_two_is_reported_as_reserved);
  RUN_TEST(test_safe_seen_bits_set_only_by_explicit_safe_values);
  RUN_TEST(test_safe_seen_cleared_by_gap_and_source_change);
  RUN_TEST(test_none_source_and_unseen_report_all_invalid);
  RUN_TEST(test_go_edge_needs_observed_not_go_then_go);
  RUN_TEST(test_go_edge_is_latched_between_ticks);
  RUN_TEST(test_boot_with_every_switch_combination_does_nothing);
  RUN_TEST(test_each_switch_must_be_cycled_after_boot);
  RUN_TEST(test_sc_into_real_needs_zero_first_and_sim_needs_a_change);
  RUN_TEST(test_sc_reserved_and_invalid_positions_make_no_edge);
  RUN_TEST(test_gap_over_one_second_or_handover_forgets_the_base);
  RUN_TEST(test_events_only_for_the_active_source);
  RUN_TEST(test_se_short_press_resets);
  RUN_TEST(test_se_single_sample_is_ignored);
  RUN_TEST(test_se_press_without_release_seen_first_does_nothing);
  RUN_TEST(test_se_press_of_one_second_or_more_is_not_a_short_press);
  RUN_TEST(test_se_gap_while_pressed_cancels_without_action);
  RUN_TEST(test_se_lost_release_sample_during_hold_cannot_become_a_reset);
  RUN_TEST(test_se_lost_release_sample_in_a_short_press_still_a_short_press);
  RUN_TEST(test_se_hold_starts_after_one_second_and_release_event);
  RUN_TEST(test_se_stuck_pressed_for_a_long_time_never_resets);
  RUN_TEST(test_se_invalid_during_press_cancels);
  RUN_TEST(test_se_source_change_cancels_a_hold);
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
  RUN_TEST(test_se_reset_only_on_the_ground_with_interlock_closed);
  RUN_TEST(test_hold_intents_start_refresh_release);
  RUN_TEST(test_hold_refused_is_not_retried_and_shows_se_refused);
  RUN_TEST(test_hold_aborts_when_the_interlock_opens);
  RUN_TEST(test_hold_that_ended_by_staleness_is_forgotten);
  RUN_TEST(test_hold_start_only_with_fresh_samples);
  RUN_TEST(test_stuck_switches_never_act_after_a_handover);

  return UNITY_END();
}
