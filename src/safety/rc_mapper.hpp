#ifndef SAFETY_RC_MAPPER_HPP
#define SAFETY_RC_MAPPER_HPP

/**
 * RC command mapper (T9b-1 decode, T9b-4 edge events; docs/t9b-design.md 3.2.1, 3.2.3, appendix 6.1 and 6.4). Pure, no
 * heap, no dependence on the simulation flag. Host-tested (test/test_rc_mapper).
 *
 * Decodes the switch channels of an accepted RC_STATE (ch5 SA, ch6 SB, ch7 SC, ch8 SD, ch9 SE, 1-based, fixed in
 * firmware), tracks each unsafe-direction detector's "safe value seen" bit and, for the ACTIVE source only, turns
 * the unsafe-direction transitions into latched events (RcEvents) that the safety task consumes with takeEvents()
 * (an edge between two 10 Hz ticks is never lost). The mapper itself acts on nothing: the decisions
 * (preconditions, refusal reasons) are made by RcCommandLogic (rc_command.hpp) and applied by main.cpp.
 *
 * Events (active source only, cleared by onActiveChanged):
 * - GO edge: SA NOT_GO -> GO in two consecutive accepted states at most kEdgeMaxAgeNs apart, with the NOT_GO
 *   explicitly seen since the last clear. Carries the SB position of the GO sample.
 * - SC edge: SC changes between the explicit positions 0 (simulation) and 1 (real) in consecutive states at most
 *   kEdgeMaxAgeNs apart. 0 -> 1 additionally needs the safe-seen bit (position 0 seen since the last clear). The
 *   first state after boot, a handover or a gap is never an edge, so every SC position at boot does nothing until
 *   the switch is cycled. Position 2 and invalid are no position (reserved), no edge into or out of them.
 * - SE classifier (D10, D11, appendix 6.4): a SHORT PRESS event (two or more pressed states without a gap above
 *   kSeGapNs, released within kSeShortMaxNs of the first pressed state, RELEASED seen first) and a HOLD (SE
 *   pressed for kSeHoldStartNs: hold serial, start and last-held times; a release event closes it). Any gap above
 *   kSeGapNs between states while pressed, or an invalid SE, cancels the press with no action (a lost release
 *   sample therefore cannot turn a hold into a reset). A hold whose samples stop is closed by the consumer's own
 *   staleness rule, the mapper only reports hold().lastNs.
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

/// Latched one-shot events of the active source (see the class comment). Consumed by takeEvents().
struct RcEvents {
  static constexpr uint8_t kNone = 255;
  bool goEdge = false;
  uint8_t goSb = RcDecoded::kInvalid; ///< SB position decoded from the GO sample
  uint8_t scEdgeTo = kNone;           ///< 0 simulation, 1 real, kNone no edge (the latest edge wins)
  bool sePress = false;               ///< a valid short press completed
  bool holdReleased = false;          ///< the hold of `holdReleasedSerial` ended by a RELEASED state
  uint32_t holdReleasedSerial = 0;
  int64_t holdReleasedNs = 0;
};

/// State of the SE hold of the active source (level, not consumed).
struct RcHoldState {
  bool active = false;     ///< SE classifier is HOLDING
  uint32_t serial = 0;     ///< counts hold starts since boot (wraps, never 0 once started)
  int64_t startNs = 0;     ///< rx time of the sample that started the hold (SE pressed for kSeHoldStartNs)
  int64_t lastNs = 0;      ///< rx time of the last pressed sample
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
  static constexpr int64_t kEdgeMaxAgeNs = 1000000000LL; ///< previous state at most this old for an edge (3.2.3)
  static constexpr int64_t kSeGapNs = 200000000LL;       ///< D10: gap between pressed samples that cancels a press
  static constexpr int64_t kSeShortMaxNs = 1000000000LL; ///< D10: a short press is released within this
  static constexpr int64_t kSeHoldStartNs = 1000000000LL; ///< D11: pressed this long starts the hold

  /// Pure decode of the switch channels (no state).
  static RcDecoded decode(const RcFrame &f);
  static uint8_t decodeTwoPos(int16_t v);
  static uint8_t decodeThreePos(int16_t v);
  /// Safe-seen bits a single decoded state would set.
  static uint8_t safeBits(const RcDecoded &d);

  /// One accepted state of `source` (None is ignored).
  void onAccepted(ControlSourceId source, const RcFrame &f, int64_t rxNs);
  /// The active source changed (`newActive`, None allowed): every detector must see the safe value again, the SE
  /// classifier and all pending events are cleared and the previous state of every source no longer counts for an
  /// edge. The default keeps the T9b-1 behaviour of tests that do not care about events.
  void onActiveChanged(ControlSourceId newActive = ControlSourceId::None);
  void reset();

  /// Returns the pending events of the active source and clears the one-shot ones.
  RcEvents takeEvents();
  /// The SE hold of the active source.
  RcHoldState hold() const { return hold_; }
  ControlSourceId active() const { return active_; }

  /// Decoded positions of the last state of `source`; all invalid if none.
  RcDecoded decoded(ControlSourceId source) const;
  uint8_t safeSeen(ControlSourceId source) const;

private:
  enum class SeState : uint8_t { Idle, Ready, Press1, Pressing, Holding };
  struct Slot {
    bool seen = false;
    int64_t lastNs = 0;
    RcDecoded d;
    uint8_t safeSeen = 0;
    bool edgeBase = false; ///< the previous state counts as the base of an edge (cleared by gap, handover, boot)
    SeState se = SeState::Idle;
    int64_t seT0 = 0;
  };
  void classifySe(Slot &s, uint8_t se, int64_t t);
  void clearDetectors(Slot &s);
  Slot slot_[2];
  ControlSourceId active_ = ControlSourceId::None;
  RcEvents ev_;
  RcHoldState hold_;
  uint32_t holdSerial_ = 0;
};

} // namespace Safety
} // namespace VCTR

#endif
