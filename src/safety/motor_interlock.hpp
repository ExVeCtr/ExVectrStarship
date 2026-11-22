#ifndef SAFETY_MOTOR_INTERLOCK_HPP
#define SAFETY_MOTOR_INTERLOCK_HPP

/**
 * Hard motor interlock (T9b-2, docs/t9b-design.md 3.4 and appendix 6.3, safety.md section 3). Pure and
 * deterministic: no Arduino, no ExVectrCore, no heap, no clock of its own (every method takes the time), and NO
 * dependence on the simulation flag. Host-tested (test/test_motor_interlock).
 *
 * The interlock is a pure GATE: it only ever answers "may the motors run now". It is enabled solely by the SD
 * switch state of the ACTIVE control source (Pocket SD, or the ground station's virtual SD on the bench source):
 *
 *   open(now) = transitionSeen && an accepted ENABLED state of the active source is at most kEnableTimeoutNs old
 *
 * A DISABLED (or invalid) SD closes it at once. Enabling needs an OBSERVED transition: two consecutive accepted
 * states of the active source, DISABLED then ENABLED, at most kEnableTimeoutNs apart, after the last clear. The
 * observed-transition memory (transitionSeen, seenDisabled) is cleared by: boot, an active-source change
 * (update), a gap of more than kReconnectGapNs between two accepted states of the source (D3-B: a gap of 1 to 2 s
 * only closes the gate, a fresh ENABLED reopens it without a toggle), and the re-arm triggers of D4-A
 * (requireRearm: mission end, failure/latched abort, system reset, simulation/real change).
 *
 * Inputs: onRcState() for every ACCEPTED RC_STATE (the control-source table already rejected unknown sources,
 * stale sequences and SOURCE_OK = 0, so those never count as data and the gate closes by the 1 s age), update()
 * once per comms tick with the table's active source, requireRearm() from the safety task.
 *
 * Threading: single-core cooperative scheduler (as control_source.hpp). onRcState()/update() run in the comms
 * task, requireRearm() in the safety task, open()/needToggle() in the TVC task and the 100 Hz watchdog task. No
 * method yields and no interrupt touches the object, so a reader never sees a half-written field. An unset
 * interlock pointer in the reader must be treated as closed (fail safe).
 */

#include <stdint.h>

#include "control_source.hpp"

namespace VCTR {
namespace Safety {

class MotorInterlock {
public:
  static constexpr int64_t kEnableTimeoutNs = 1000000000LL; ///< valid ENABLED state within 1 s (safety.md 3.2)
  static constexpr int64_t kReconnectGapNs = 2000000000LL;  ///< D3-B: gap that forgets the observed transition

  /// Why the gate is closed (open = None). Priority order as listed.
  enum class CloseReason : uint8_t {
    None = 0,      ///< open
    NoSource = 1,  ///< no active control source
    NeedToggle = 2, ///< SD shows ENABLED but no off -> on transition was observed (INTERLOCK_NEED_TOGGLE)
    SdDisabled = 3, ///< the last state shows DISABLED (or the SD channel is invalid)
    Stale = 4,     ///< no accepted state within 1 s, or no ENABLED state yet
  };

  /// D4-A re-arm triggers (information for the log and the tests).
  enum class RearmReason : uint8_t { MissionEnd = 1, Failure = 2, SystemReset = 3, ModeChange = 4, Abort = 5 };

  /// One ACCEPTED RC_STATE. `source` must be the source the state came from; states of any source but the active
  /// one are ignored. `sdEnabled` is true only for a valid ENABLED SD position (invalid decodes as false).
  void onRcState(ControlSourceId source, bool sdEnabled, int64_t rxNs);

  /// Per comms tick: `active` is the control-source table's active source. A change clears everything; a silence
  /// of more than kReconnectGapNs on the active source forgets the observed transition.
  void update(int64_t nowNs, ControlSourceId active);

  /// D4-A: the next opening needs a fresh observed DISABLED -> ENABLED transition. Closes the gate immediately.
  void requireRearm(RearmReason why);

  /// The gate. True only when the motors may run.
  bool open(int64_t nowNs) const;
  /// SD shows ENABLED in a fresh state but no transition was observed: the pilot must toggle SD off and on.
  bool needToggle(int64_t nowNs) const;
  CloseReason closeReason(int64_t nowNs) const;

  ControlSourceId activeSource() const { return active_; }
  uint8_t rearmCount() const { return rearmCount_; }
  RearmReason lastRearmReason() const { return lastRearm_; }

private:
  void clearTransition();

  ControlSourceId active_ = ControlSourceId::None;
  bool haveLast_ = false;        ///< an accepted state of the active source is recorded
  bool lastSdEnabled_ = false;   ///< SD of that state
  int64_t lastRxNs_ = 0;         ///< its receive time
  bool seenDisabled_ = false;    ///< an accepted DISABLED state since the last clear
  bool transitionSeen_ = false;  ///< the DISABLED -> ENABLED edge was observed
  bool enabledValid_ = false;    ///< the last accepted state is ENABLED and transitionSeen_ held
  int64_t lastEnabledNs_ = 0;    ///< receive time of the last accepted ENABLED state while transitionSeen_
  uint8_t rearmCount_ = 0;       ///< wrapping count of requireRearm() calls
  RearmReason lastRearm_ = RearmReason::MissionEnd;
};

/// Inputs of the motor output gate (StarshipTVC, last point before the motor commands).
struct MotorGateInput {
  bool interlockOpen = false;   ///< MotorInterlock::open(now); false when no interlock is attached
  bool forcedOff = false;       ///< the watchdog's sticky "closed" flag (StarshipTVC::forceMotorsOff)
  bool enableMotors = false;    ///< vehicle logic: motors enabled
  bool enableActuators = false; ///< vehicle logic: TVC actuators enabled
  bool testIdle = true;         ///< no actuator test running
};

struct MotorGateResult {
  bool outputsEnabled = false;      ///< motor PWM may be enabled and written (the AND at the output)
  bool thrustModelEnabled = false;  ///< the modelled actual thrust (the body simulator's input) may be non-zero
};

/**
 * The AND at the lowest output level. The interlock and the forced-off flag gate BOTH results; no simulation flag
 * is an input (D16: the modelled thrust loses the gate exactly as hardware would lose thrust; the existing
 * enableMotors logic, which differs in simulation, is NOT part of the thrust model's gate).
 */
inline MotorGateResult evaluateMotorGate(const MotorGateInput &in) {
  MotorGateResult r;
  const bool gate = in.interlockOpen && !in.forcedOff;
  r.thrustModelEnabled = gate && in.enableActuators && in.testIdle;
  r.outputsEnabled = r.thrustModelEnabled && in.enableMotors;
  return r;
}

/// Watchdog liveness (design 3.4.2): the safety task latches a failsafe when the 100 Hz watchdog has not run for
/// longer than this.
constexpr int64_t kInterlockWatchdogStaleNs = 100000000LL;

/// True if the watchdog is stale. `lastRunNs` = 0 means it never ran (stale).
inline bool interlockWatchdogStale(int64_t nowNs, int64_t lastRunNs) {
  return lastRunNs <= 0 || nowNs - lastRunNs > kInterlockWatchdogStaleNs;
}

} // namespace Safety
} // namespace VCTR

#endif
