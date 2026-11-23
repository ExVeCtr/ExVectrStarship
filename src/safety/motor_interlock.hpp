#ifndef SAFETY_MOTOR_INTERLOCK_HPP
#define SAFETY_MOTOR_INTERLOCK_HPP

/**
 * Hard motor interlock (T9b-2, docs/t9b-design.md 3.4 and appendix 6.3, safety.md section 3). Pure and
 * deterministic: no Arduino, no ExVectrCore, no heap, no clock of its own (every method takes the time), and NO
 * dependence on the simulation flag. Host-tested (test/test_motor_interlock).
 *
 * The interlock is a pure GATE: it only ever answers "may the motors run now". It is enabled solely by the motors
 * field (SD, "motors armed / disarmed") of the accepted CMD_STATE of the latched ground-station session:
 *
 *   open(now) = transitionSeen && an accepted ENABLED state of the latched session is at most kEnableTimeoutNs old
 *
 * A DISABLED motors field closes it at once. Enabling needs an OBSERVED transition: two consecutive accepted
 * states, DISABLED then ENABLED, at most kEnableTimeoutNs apart, after the last clear. The
 * observed-transition memory (transitionSeen, seenDisabled) is cleared by: boot, a new session
 * (onSessionChanged), a gap of more than kReconnectGapNs between two accepted states (D3-B: a gap of 1 to 2 s
 * only closes the gate, a fresh ENABLED reopens it without a toggle), and the re-arm triggers of D4-A
 * (requireRearm: mission end, failure/latched abort, system reset, simulation/real change).
 *
 * Inputs: onCmdState() for every ACCEPTED CMD_STATE (the command ingress already rejected other sources, foreign
 * sessions and stale sequences, so those never count as data and the gate closes by the 1 s age),
 * onSessionChanged() when the ingress latches a new session (before its first state), update() once per comms
 * tick, requireRearm() from the safety task.
 *
 * Threading: single-core cooperative scheduler (as control_source.hpp). onCmdState()/update() run in the comms
 * task, requireRearm() in the safety task, open()/needToggle() in the TVC task and the 100 Hz watchdog task. No
 * method yields and no interrupt touches the object, so a reader never sees a half-written field. An unset
 * interlock pointer in the reader must be treated as closed (fail safe).
 */

#include <stdint.h>


namespace VCTR {
namespace Safety {

class MotorInterlock {
public:
  static constexpr int64_t kEnableTimeoutNs = 1000000000LL; ///< valid ENABLED state within 1 s (safety.md 3.2)
  static constexpr int64_t kReconnectGapNs = 2000000000LL;  ///< D3-B: gap that forgets the observed transition

  /// Why the gate is closed (open = None). Priority order as listed.
  enum class CloseReason : uint8_t {
    None = 0,      ///< open
    NoSource = 1,  ///< no ground-station session latched
    NeedToggle = 2, ///< the motors field shows ENABLED but no off -> on transition was observed (INTERLOCK_NEED_TOGGLE)
    SdDisabled = 3, ///< the last state shows DISABLED
    Stale = 4,     ///< no accepted state within 1 s, or no ENABLED state yet
  };

  /// D4-A re-arm triggers (information for the log and the tests).
  enum class RearmReason : uint8_t { MissionEnd = 1, Failure = 2, SystemReset = 3, ModeChange = 4, Abort = 5 };

  /// One ACCEPTED CMD_STATE of the latched session. `motorsEnabled` is the motors field (1 = ENABLED).
  void onCmdState(bool motorsEnabled, int64_t rxNs);

  /// The ingress latched a new session: clears everything (the first state of the session is the baseline, so an
  /// ENABLED field at the session start needs a DISABLED -> ENABLED transition) and marks the session present.
  void onSessionChanged();

  /// Per comms tick: a silence of more than kReconnectGapNs forgets the observed transition.
  void update(int64_t nowNs);

  /// D4-A: the next opening needs a fresh observed DISABLED -> ENABLED transition. Closes the gate immediately.
  void requireRearm(RearmReason why);

  /// The gate. True only when the motors may run.
  bool open(int64_t nowNs) const;
  /// The motors field shows ENABLED in a fresh state but no transition was observed: the operator must toggle SD off and on.
  bool needToggle(int64_t nowNs) const;
  CloseReason closeReason(int64_t nowNs) const;

  uint8_t rearmCount() const { return rearmCount_; }
  RearmReason lastRearmReason() const { return lastRearm_; }

private:
  void clearTransition();

  bool haveSession_ = false;     ///< a ground-station session has been latched
  bool haveLast_ = false;        ///< an accepted state is recorded
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
  bool outputsEnabled = false; ///< motor PWM may be enabled and written (the AND at the output)
};

/**
 * The AND at the lowest output level (motor PWM only). No simulation flag is an input, and the modelled actual
 * thrust that the body simulator consumes is NOT gated (D16 rejected, design 1.3).
 */
inline MotorGateResult evaluateMotorGate(const MotorGateInput &in) {
  MotorGateResult r;
  r.outputsEnabled = in.interlockOpen && !in.forcedOff && in.enableMotors && in.enableActuators && in.testIdle;
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
