#ifndef SAFETY_RC_MAPPER_HPP
#define SAFETY_RC_MAPPER_HPP

/**
 * RC command mapper, DECODE PART ONLY (T9b-1, docs/t9b-design.md 3.2.1, 3.2.3, appendix 6.1). Pure, no heap, no
 * dependence on the simulation flag. Host-tested (test/test_rc_mapper).
 *
 * Decodes the switch channels of an accepted RC_STATE (ch5 SA, ch6 SB, ch7 SC, ch8 SD, ch9 SE, 1-based, fixed in
 * firmware) and tracks each unsafe-direction detector's "safe value seen" bit. NO command acts on any of it and
 * no edge event is produced yet (T9b-4).
 *
 * Decoding rules: CH_INVALID and any value outside -1000..1000 decode as INVALID (255). 2-pos switches:
 * v <= -500 -> 0, v >= +500 -> 1, otherwise invalid. 3-pos (SB, SC): v <= -500 -> 0, -200..+200 -> 1,
 * v >= +500 -> 2, otherwise invalid. For action purposes invalid is the safe value (saSafe(), sdSafe(),
 * seSafe() = 0); SB and SC invalid mean "no valid selection".
 *
 * Safe-seen bits (3.2.3): set by an accepted state showing the safe value EXPLICITLY (a valid position; an invalid
 * channel does not set the bit), cleared by boot, by an active-source change (onActiveChanged) and by a gap of more
 * than kGapClearNs between two accepted states of the same source.
 */

#include <stdint.h>

#include "control_source.hpp"

namespace VCTR {
namespace Safety {

struct RcDecoded {
  static constexpr uint8_t kInvalid = 255;
  uint8_t sa = kInvalid, sb = kInvalid, sc = kInvalid, sd = kInvalid, se = kInvalid;
  /// Value for action purposes: invalid is the safe value 0 (NOT_GO, DISABLED, RELEASED).
  uint8_t saSafe() const { return sa == kInvalid ? 0 : sa; }
  uint8_t sdSafe() const { return sd == kInvalid ? 0 : sd; }
  uint8_t seSafe() const { return se == kInvalid ? 0 : se; }
};

/// RC_APPLIED.safe_seen bits.
enum RcSafeSeenBits : uint8_t {
  kSafeSeenGo = 1u << 0,
  kSafeSeenScReal = 1u << 1,
  kSafeSeenSdEnable = 1u << 2,
  kSafeSeenSePress = 1u << 3,
};

class RcCommandMapper {
public:
  static constexpr size_t kChSa = 4, kChSb = 5, kChSc = 6, kChSd = 7, kChSe = 8; ///< 0-based channel indices
  static constexpr int64_t kGapClearNs = 2000000000LL; ///< a gap longer than this on a source clears its bits

  /// Pure decode of the switch channels (no state).
  static RcDecoded decode(const RcFrame &f);
  static uint8_t decodeTwoPos(int16_t v);
  static uint8_t decodeThreePos(int16_t v);
  /// Safe-seen bits a single decoded state would set.
  static uint8_t safeBits(const RcDecoded &d);

  /// One accepted state of `source` (None is ignored).
  void onAccepted(ControlSourceId source, const RcFrame &f, int64_t rxNs);
  /// The active source changed: every detector must see the safe value again.
  void onActiveChanged();
  void reset();

  /// Decoded positions of the last state of `source`; all invalid if none.
  RcDecoded decoded(ControlSourceId source) const;
  uint8_t safeSeen(ControlSourceId source) const;

private:
  struct Slot {
    bool seen = false;
    int64_t lastNs = 0;
    RcDecoded d;
    uint8_t safeSeen = 0;
  };
  Slot slot_[2];
};

} // namespace Safety
} // namespace VCTR

#endif
