#ifndef SAFETY_CONTROL_SOURCE_HPP
#define SAFETY_CONTROL_SOURCE_HPP

/**
 * Control-source table (T9b-1, docs/t9b-design.md 3.1, appendix 6.2). Pure and deterministic: no Arduino, no
 * ExVectrCore, no heap, no dependence on the simulation flag. Host-tested (test/test_control_source).
 *
 * Two sources are accepted, keyed by (source node, ingress link):
 *   RADIO = node 2 on the UART (the Pocket through Router, air link and ship node),
 *   BENCH = node 0 on USB (the ground station's virtual radio on the vehicle's USB).
 * Every other RC_STATE is rejected and counted per reason (RcReject).
 *
 * T9b-1 is OBSERVE ONLY: nothing acts on the active source yet and the failure rule is unchanged.
 * `committed` (interlock open, mission chain pending or mode Running) is an input that update() already honours
 * (the active source latches), but main.cpp passes false until the interlock exists (T9b-2).
 *
 * Arbitration (update): while not committed the radio is preferred while it is fresh (last accepted state at most
 * kCtrlTimeoutNs old). A handover back to RADIO from BENCH needs kHandoverNs of continuous radio states (a gap
 * longer than kRadioGapNs restarts that run), unless BENCH itself is no longer fresh. While committed the active
 * source is latched. A source change is reported once through sourceChangedSinceLast().
 *
 * Threading: all writers run in the comms task. snapshot() is a seqlock read, so any other task (safety task,
 * interlock) gets a consistent copy even when the 64-bit timestamps are written in two 32-bit halves.
 */

#include <stddef.h>
#include <stdint.h>

namespace VCTR {
namespace Safety {

/// The fields of RC_STATE the vehicle uses (flags and 16 channels, -1000..1000, kChInvalid = not provided).
struct RcFrame {
  static constexpr int16_t kChInvalid = -32768;
  static constexpr uint8_t kFlagSourceOk = 1u << 0;
  uint8_t flags = 0;
  int16_t ch[16] = {};
};

/// Values match VehicleStatus.ctrl_source for NONE (0), RADIO (1) and USB (2, the bench).
enum class ControlSourceId : uint8_t { None = 0, Radio = 1, Bench = 2 };

enum class RcLink : uint8_t { Usb = 0, Uart = 1 };

/// Reject reasons, the index of RC_APPLIED.rejects[].
enum class RcReject : uint8_t { BadLength = 0, SourceNotInTable = 1, StaleSequence = 2, SourceNotOk = 3 };

/// State of one accepted source.
struct RcSourceRecord {
  bool seen = false;       ///< at least one state accepted
  int64_t lastRxNs = 0;    ///< receive time of the last accepted state
  uint32_t accepted = 0;   ///< accepted states since boot (wraps)
  RcFrame frame;           ///< the last accepted state
};

/// Consistent copy for readers in other tasks.
struct ControlSourceSnapshot {
  ControlSourceId active = ControlSourceId::None;
  bool latched = false; ///< active source is latched because the vehicle is committed
  RcSourceRecord src[2]; ///< index = static_cast<size_t>(id) - 1 (Radio, Bench)
  uint8_t rejects[4] = {}; ///< wrapping counts per RcReject
  uint8_t rejectedTotal = 0; ///< wrapping count over all reasons

  static size_t index(ControlSourceId id) { return static_cast<size_t>(id) - 1; }
  const RcSourceRecord *record(ControlSourceId id) const {
    return (id == ControlSourceId::Radio || id == ControlSourceId::Bench) ? &src[index(id)] : nullptr;
  }
  /// Age of the last accepted state of `id` in ns, -1 if none.
  int64_t ageNs(ControlSourceId id, int64_t nowNs) const {
    const RcSourceRecord *r = record(id);
    return (r && r->seen) ? nowNs - r->lastRxNs : -1;
  }
  bool fresh(ControlSourceId id, int64_t nowNs, int64_t timeoutNs) const {
    const int64_t a = ageNs(id, nowNs);
    return a >= 0 && a <= timeoutNs;
  }
};

class ControlSourceTable {
public:
  static constexpr uint8_t kRadioNode = 2; ///< RADIO = (2, UART)
  static constexpr uint8_t kBenchNode = 0; ///< BENCH = (0, USB)
  static constexpr int64_t kCtrlTimeoutNs = 2000000000LL; ///< T_ctrl: fresh = at most 2 s old
  static constexpr int64_t kHandoverNs = 500000000LL;     ///< continuous radio before handing back to RADIO
  /// Continuity of the radio run: a gap longer than this between two accepted radio states restarts the 0.5 s
  /// run (25 Hz frames are 40 ms apart). Not in the design table (6.6): chosen as 6 missed frames.
  static constexpr int64_t kRadioGapNs = 250000000LL;
  static constexpr int64_t kSeqResyncNs = 1000000000LL; ///< as the transport PortMux: a silent source resyncs

  /// Offers one decoded RC_STATE (length already checked by the caller). Checks, in this order: the (src, link)
  /// key, the sequence (per accepted source), SOURCE_OK. Returns true if accepted (refreshes the source's
  /// record); on false the reject counter of the reason is incremented. `accepted` receives the source.
  bool onRcFrame(uint8_t srcNode, RcLink link, uint8_t seq, const RcFrame &frame, int64_t rxNs,
                 ControlSourceId *accepted = nullptr);
  /// An RC_STATE frame with the wrong length or that does not decode (RcReject::BadLength).
  void onMalformed();

  /// Arbitration step; call every comms tick. `committed` latches the active source.
  void update(int64_t nowNs, bool committed);

  ControlSourceId active() const { return cur_.active; }
  /// Age of the last accepted state of the active source in ns, -1 if there is none.
  int64_t activeAgeNs(int64_t nowNs) const { return cur_.ageNs(cur_.active, nowNs); }
  const RcFrame *lastActiveState() const {
    const RcSourceRecord *r = cur_.record(cur_.active);
    return (r && r->seen) ? &r->frame : nullptr;
  }
  /// True once after each change of the active source (including None <-> source).
  bool sourceChangedSinceLast() {
    const bool c = changed_;
    changed_ = false;
    return c;
  }
  /// Bit 0 RADIO fresh, bit 1 BENCH fresh (also for the ignored source).
  uint8_t freshMask(int64_t nowNs) const;
  const uint8_t *rejectCounts() const { return cur_.rejects; }
  /// The control failsafe condition (NOT wired in T9b-1): committed and the active source is None or older than
  /// kCtrlTimeoutNs.
  bool failsafeDue(int64_t nowNs, bool committed) const;

  /// Seqlock read of the published state. False only if the writer was busy on every retry (never on a
  /// single-core cooperative scheduler); the caller keeps its previous copy.
  bool snapshot(ControlSourceSnapshot &out) const;
  /// The writer's own live view (comms task only).
  const ControlSourceSnapshot &live() const { return cur_; }

  /// Maps (src, link) to a source. None = not in the table.
  static ControlSourceId classify(uint8_t srcNode, RcLink link);

private:
  void reject(RcReject r);
  void publish();

  ControlSourceSnapshot cur_;
  // sequence state per accepted source
  struct Seq {
    bool seen = false;
    uint8_t last = 0;
    int64_t acceptedNs = 0;
  } seq_[2];
  bool radioRunValid_ = false;
  int64_t radioRunStartNs_ = 0;
  bool changed_ = false;

  uint32_t version_ = 0; ///< seqlock counter: odd while publishing
  ControlSourceSnapshot pub_;
};

} // namespace Safety
} // namespace VCTR

#endif
