#ifndef SAFETY_RC_MAPPER_HPP
#define SAFETY_RC_MAPPER_HPP

/**
 * Command mapper (T9b-1 decode, T9b-4 edge events; GS-control cutover: docs/gs-control-design.md 3.1, 4). Pure, no
 * heap, no dependence on the simulation flag. Host-tested (test/test_rc_mapper). The class name is historical: its
 * input is the accepted CMD_STATE of the latched ground-station session (command_ingress.hpp), not handset RC.
 *
 * Takes the fields of each accepted CMD_STATE (go = SA, profile = SB, vehicle_mode = SC, motors = SD,
 * reset_button = SE, failsafe), tracks each unsafe-direction detector's "safe value seen" bit and turns the
 * unsafe-direction transitions into latched events (RcEvents) that the safety task consumes with takeEvents()
 * (an edge between two 10 Hz ticks is never lost). The mapper acts on nothing: the decisions (preconditions,
 * refusal reasons) are made by RcCommandLogic (rc_command.hpp) and applied by main.cpp.
 *
 * Events (cleared by onSessionChanged):
 * - GO edge: go NOT_GO -> GO in two consecutive accepted states at most kEdgeMaxAgeNs apart, with the NOT_GO
 *   explicitly seen since the last clear. Carries the profile of the GO sample.
 * - SC edge: vehicle_mode changes (REAL <-> SIMULATION) in consecutive states at most kEdgeMaxAgeNs apart. The
 *   first state after boot, a session change or a gap is never an edge, so every vehicle_mode at boot does nothing
 *   until the field is cycled.
 * - SE classifier (D10, D11): a SHORT PRESS event (two or more pressed states without a gap above kSeGapNs, released
 *   within kSeShortMaxNs of the first pressed state, RELEASED seen first) and a HOLD (reset_button pressed for
 *   kSeHoldStartNs: hold serial, start and last-held times; a release event closes it). Any gap above kSeGapNs
 *   between states while pressed cancels the press with no action (a lost release sample therefore cannot turn a
 *   hold into a reset). A hold whose samples stop is closed by the consumer's own staleness rule, the mapper only
 *   reports hold().lastNs. The ground station stretches every press to at least 160 ms (4 frames).
 *
 * Every field of an accepted state is valid (the strict decoder rejected anything else), so there is no "invalid"
 * value any more. Safe values: NOT_GO, DISABLED, RELEASED.
 *
 * Safe-seen bits (3.2.3): set by an accepted state showing the safe value, cleared by boot, by a session change
 * (onSessionChanged) and by a gap of more than kGapClearNs between two accepted states.
 */

#include <stdint.h>

#include "command_ingress.hpp"

namespace VCTR {
namespace Safety {

struct RcDecoded {
  uint8_t go = 0;      ///< 0 NOT_GO, 1 GO (SA)
  uint8_t profile = 0; ///< 0, 1, 2 (SB)
  uint8_t mode = 0;    ///< 0 REAL, 1 SIMULATION (SC; the Pocket's SC 1 and 2 both arrive as SIMULATION)
  uint8_t motors = 0;  ///< 0 DISABLED, 1 ENABLED (SD: motors disarmed / armed)
  uint8_t reset = 0;   ///< 0 RELEASED, 1 PRESSED (SE)
  uint8_t failsafe = 0; ///< 0 none, 1 GS failsafe requested
};

/// Latched one-shot events of the active source (see the class comment). Consumed by takeEvents().
struct RcEvents {
  static constexpr uint8_t kNone = 255;
  bool goEdge = false;
  uint8_t goProfile = 0;              ///< profile of the GO sample
  uint8_t scEdgeTo = kNone;           ///< 0 REAL, 1 SIMULATION, kNone no edge (the latest edge wins)
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

/// CMD_APPLIED.safe_seen bits.
enum RcSafeSeenBits : uint8_t {
  kSafeSeenGo = 1u << 0,
  kSafeSeenScReal = 1u << 1, ///< vehicle_mode REAL seen
  kSafeSeenSdEnable = 1u << 2,
  kSafeSeenSePress = 1u << 3,
};

class RcCommandMapper {
public:
  static constexpr int64_t kGapClearNs = 2000000000LL; ///< a gap longer than this clears the bits
  static constexpr int64_t kEdgeMaxAgeNs = 1000000000LL; ///< previous state at most this old for an edge (3.2.3)
  static constexpr int64_t kSeGapNs = 200000000LL;       ///< D10: gap between pressed samples that cancels a press
  static constexpr int64_t kSeShortMaxNs = 1000000000LL; ///< D10: a short press is released within this
  static constexpr int64_t kSeHoldStartNs = 1000000000LL; ///< D11: pressed this long starts the hold

  /// Pure decode of the command fields (no state).
  static RcDecoded decode(const CmdFrame &f);
  /// Safe-seen bits a single decoded state would set.
  static uint8_t safeBits(const RcDecoded &d);

  /// One accepted state of the latched session.
  void onAccepted(const CmdFrame &f, int64_t rxNs);
  /// A new session was latched: every detector must see the safe value again, the SE classifier and all pending
  /// events are cleared and the previous state no longer counts for an edge.
  void onSessionChanged();
  void reset();

  /// Returns the pending events and clears the one-shot ones.
  RcEvents takeEvents();
  /// The SE hold.
  RcHoldState hold() const { return hold_; }

  /// Decoded fields of the last accepted state; all safe if none.
  RcDecoded decoded() const { return s_.d; }
  bool seen() const { return s_.seen; }
  uint8_t safeSeen() const { return s_.safeSeen; }

private:
  enum class SeState : uint8_t { Idle, Ready, Press1, Pressing, Holding };
  struct Slot {
    bool seen = false;
    int64_t lastNs = 0;
    RcDecoded d;
    uint8_t safeSeen = 0;
    bool edgeBase = false; ///< the previous state counts as the base of an edge (cleared by gap, session change, boot)
    SeState se = SeState::Idle;
    int64_t seT0 = 0;
  };
  void classifySe(Slot &s, uint8_t pressed, int64_t t);
  void clearDetectors(Slot &s);
  Slot s_;
  RcEvents ev_;
  RcHoldState hold_;
  uint32_t holdSerial_ = 0;
};

} // namespace Safety
} // namespace VCTR

#endif
