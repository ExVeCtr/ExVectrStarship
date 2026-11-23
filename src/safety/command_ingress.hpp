#ifndef SAFETY_COMMAND_INGRESS_HPP
#define SAFETY_COMMAND_INGRESS_HPP

/**
 * Command ingress (GS-control cutover, docs/gs-control-design.md section 4; replaces the T9b-1 ControlSourceTable).
 * Pure and deterministic: no Arduino, no ExVectrCore, no heap, no dependence on the simulation flag. Host-tested
 * (test/test_command_ingress).
 *
 * There is ONE control source: the ground station, node 0, sending CMD_STATE (port 0x13) on the USB and the UART
 * link. Both links carry copies of one stream (same session, same sequence number), so the first copy is accepted
 * and the second is a duplicate; there is no link priority and no handover. Per-link freshness is reported only.
 * Nothing else commands the vehicle: handset RC_STATE (port 0x10) is not interpreted at all.
 *
 * Checks of onFrame(), in this order (the first failing one rejects the frame and is counted per reason):
 *   1. length and field range (caller: CmdIngressTap decodes with the strict generated decoder -> onMalformed()),
 *   2. source node 0,
 *   3. session latch, BEFORE the sequence filter:
 *        - the first valid session after boot is latched;
 *        - another session is latched only while the vehicle is NOT committed or is in mode Failure (actuators
 *          already cut, so a restarted ground station can send the SE reset) AND the latched session has been
 *          silent for kSessionSilentNs; acceptance resets every detector (the caller reacts to
 *          Result::sessionChanged before it feeds the frame to the mapper and the interlock);
 *        - otherwise a foreign session is rejected and MULTI_MASTER is set for kMultiMasterShowNs;
 *   4. the per-session sequence filter ((seq - last) mod 256 in 1..127, or the session was silent for
 *      kSeqResyncNs). An exact duplicate (same sequence, the copy on the other link) and a late copy are NOT rejects (counted in
 *      the internal duplicates / lateCopies); a duplicate refreshes the link freshness and the session silence timer.
 *
 * Control failsafe rule (T9b-3 rule, new source; failsafeDue()): committed and no accepted state of the latched
 * session within kCtrlTimeoutNs. Not committed: never due (D2-B), the source only reads NONE.
 *
 * Vehicle-directed REQUESTs (design Q5): requestAllowed(session) is true only for the latched, non-zero session.
 *
 * Threading: all writers run in the comms task. snapshot() is a seqlock read, so any other task (safety task)
 * gets a consistent copy even when the 64-bit timestamps are written in two 32-bit halves.
 */

#include <stddef.h>
#include <stdint.h>

namespace VCTR {
namespace Safety {

/// The fields of an accepted CMD_STATE (every field is a valid enum value, enforced by the strict decoder).
struct CmdFrame {
  static constexpr uint8_t kFlagPocketLive = 1u << 1;
  static constexpr uint8_t kFlagInputVirtual = 1u << 2;
  uint8_t flags = 0;       ///< info flags (POCKET_LIVE, INPUT_VIRTUAL), reported only
  uint32_t session = 0;
  uint8_t go = 0;          ///< 0 NOT_GO, 1 GO
  uint8_t profile = 0;     ///< 0, 1, 2
  uint8_t vehicleMode = 0; ///< 0 REAL, 1 SIMULATION
  uint8_t motors = 0;      ///< 0 DISABLED (motors disarmed), 1 ENABLED (motors armed)
  uint8_t resetButton = 0; ///< 0 RELEASED, 1 PRESSED
  uint8_t failsafe = 0;    ///< 0 none, 1 GS failsafe requested
};

enum class CmdLink : uint8_t { Usb = 0, Uart = 1 };

/// Reject reasons, the index of CMD_APPLIED.rejects[].
enum class CmdReject : uint8_t { BadLength = 0, SourceNotNode0 = 1, ForeignSession = 2, StaleSequence = 3 };

/// Consistent copy for readers in other tasks.
struct CmdIngressSnapshot {
  bool latched = false;        ///< a session is latched
  uint32_t session = 0;        ///< the latched session (0 = none)
  bool seen = false;           ///< at least one state of the latched session accepted
  int64_t lastRxNs = 0;        ///< receive time of the last accepted state
  uint32_t accepted = 0;       ///< accepted states since boot (wraps)
  CmdFrame frame;              ///< the last accepted state of the latched session
  bool linkSeen[2] = {false, false};
  int64_t linkLastNs[2] = {0, 0}; ///< last accepted or duplicate copy of the latched session per link
  bool commitLatched = false;  ///< the session is held because the vehicle is committed (SOURCE_LATCHED)
  bool foreignSeen = false;
  int64_t foreignLastNs = 0;   ///< last rejected foreign-session frame
  uint8_t rejects[4] = {};     ///< wrapping counts per CmdReject
  uint8_t rejectedTotal = 0;   ///< wrapping count over all reasons
  uint8_t lateCopies = 0;      ///< wrapping count of late copies (not a reject, internal, not on the wire)
  uint8_t duplicates = 0;      ///< wrapping count of exact duplicates (not a reject, internal, not on the wire)

  /// Age of the last accepted state in ns, -1 if none.
  int64_t ageNs(int64_t nowNs) const { return (latched && seen) ? nowNs - lastRxNs : -1; }
  bool fresh(int64_t nowNs, int64_t timeoutNs) const {
    const int64_t a = ageNs(nowNs);
    return a >= 0 && a <= timeoutNs;
  }
  int64_t linkAgeNs(CmdLink l, int64_t nowNs) const {
    const size_t i = static_cast<size_t>(l);
    return (latched && linkSeen[i]) ? nowNs - linkLastNs[i] : -1;
  }
  bool linkFresh(CmdLink l, int64_t nowNs, int64_t timeoutNs) const {
    const int64_t a = linkAgeNs(l, nowNs);
    return a >= 0 && a <= timeoutNs;
  }
  bool multiMaster(int64_t nowNs, int64_t showNs) const {
    return foreignSeen && nowNs - foreignLastNs >= 0 && nowNs - foreignLastNs <= showNs;
  }
  /// The control failsafe condition (safety.md 4.1): committed and the latched session has no accepted state within
  /// `timeoutNs` (or none at all). Not committed: never due (D2-B). Pure function of the snapshot.
  bool failsafeDue(int64_t nowNs, bool committed, int64_t timeoutNs) const {
    if (!committed) return false;
    return !fresh(nowNs, timeoutNs);
  }
};

class CommandIngress {
public:
  static constexpr uint8_t kGsNode = 0;                     ///< the only accepted source
  static constexpr int64_t kCtrlTimeoutNs = 2000000000LL;   ///< T_ctrl: fresh = at most 2 s old
  static constexpr int64_t kSessionSilentNs = 1000000000LL; ///< the latched session must be silent this long
  static constexpr int64_t kSeqResyncNs = 1000000000LL;     ///< a silent session resyncs its sequence
  static constexpr uint8_t kLateCopyFrames = 16;           ///< a copy up to this many frames behind refreshes the link age only
  static constexpr int64_t kMultiMasterShowNs = 2000000000LL;

  struct Result {
    bool accepted = false;       ///< the state is new and valid: feed it to the mapper and the interlock
    bool sessionChanged = false; ///< a session was latched by this frame: reset every detector BEFORE feeding it
  };

  /// Offers one decoded CMD_STATE (length and range already checked by the strict decoder).
  Result onFrame(uint8_t srcNode, CmdLink link, uint8_t seq, const CmdFrame &frame, int64_t rxNs);
  /// A port 0x13 frame with the wrong length or an out-of-range field (CmdReject::BadLength).
  void onMalformed();

  /// Once per comms tick: the vehicle context that gates a session takeover. `committed` = Safety::committed(),
  /// `modeFailure` = vehicle mode Failure (set by the safety task through VehicleNetwork).
  void update(int64_t nowNs, bool committed, bool modeFailure);

  /// A vehicle-directed REQUEST (target 1) is acted on only if its session equals the latched session (non-zero).
  bool requestAllowed(uint32_t session) const { return cur_.latched && session != 0 && session == cur_.session; }
  /// Counts a vehicle-directed REQUEST by its session; returns requestAllowed().
  bool onRequest(uint32_t session);
  uint16_t requestsAccepted() const { return requestsAccepted_; }
  uint16_t requestsRefused() const { return requestsRefused_; }

  bool sessionLatched() const { return cur_.latched; }
  uint32_t session() const { return cur_.session; }
  /// True once after each session change; `session` receives the new session (for the log line).
  bool takeSessionChange(uint32_t &session) {
    if (!changed_) return false;
    changed_ = false;
    session = cur_.session;
    return true;
  }
  /// Age of the last accepted state in ns, -1 if none.
  int64_t ageNs(int64_t nowNs) const { return cur_.ageNs(nowNs); }
  /// The last accepted state of the latched session, nullptr if none.
  const CmdFrame *lastState() const { return (cur_.latched && cur_.seen) ? &cur_.frame : nullptr; }
  const uint8_t *rejectCounts() const { return cur_.rejects; }
  /// Comms task only (live view); other tasks use CmdIngressSnapshot::failsafeDue.
  bool failsafeDue(int64_t nowNs, bool committed) const { return cur_.failsafeDue(nowNs, committed, kCtrlTimeoutNs); }

  /// Seqlock read of the published state. False only if the writer was busy on every retry (never on a
  /// single-core cooperative scheduler); the caller keeps its previous copy.
  bool snapshot(CmdIngressSnapshot &out) const;
  /// The writer's own live view (comms task only).
  const CmdIngressSnapshot &live() const { return cur_; }

private:
  void reject(CmdReject r);
  void publish();

  CmdIngressSnapshot cur_;
  bool committed_ = false;
  bool modeFailure_ = false;
  int64_t sessionRxNs_ = 0; ///< last accepted or duplicate frame of the latched session (silence timer)
  bool seqSeen_ = false;
  uint8_t seqLast_ = 0;
  int64_t seqAcceptedNs_ = 0;
  bool changed_ = false;
  uint16_t requestsAccepted_ = 0, requestsRefused_ = 0;

  uint32_t version_ = 0; ///< seqlock counter: odd while publishing
  CmdIngressSnapshot pub_;
};

} // namespace Safety
} // namespace VCTR

#endif
