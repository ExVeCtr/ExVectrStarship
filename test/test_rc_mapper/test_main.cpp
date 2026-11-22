// Host tests of src/safety/rc_mapper.cpp, decode part (T9b-1): pio test -e native -f test_rc_mapper
#include <unity.h>

#include <initializer_list>

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
  return UNITY_END();
}
