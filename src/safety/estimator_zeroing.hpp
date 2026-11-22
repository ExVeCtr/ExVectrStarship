#ifndef SAFETY_ESTIMATOR_ZEROING_HPP
#define SAFETY_ESTIMATOR_ZEROING_HPP

/**
 * Estimator zeroing window and the "valid zero since boot" readiness rule (user decision 2026-11-20).
 *
 * Pure logic with injected time: no Arduino, no ExVectrCore, no globals. Host-tested with
 * `pio test -e native` (test/test_estimator_zeroing). VehicleSafetyAndControlTask (main.cpp) owns one instance,
 * feeds it the vehicle state and NowNs(), and applies the result to the estimators (enableZeroingMode of the
 * position KF, the attitude EKF and the body simulator).
 *
 * Rules (docs/project/overview.md, "Estimator zeroing"):
 * - A zeroing window has a fixed end time on the vehicle (one-shot command, no dependence on the sender staying
 *   connected). While it is active the estimators run in zeroing mode; the vehicle must be still.
 * - A window that runs to its end records a valid zero (zeroedSinceBoot()). Only an aborted window (below) or a
 *   reboot clears that record.
 * - The vehicle may become Ready only with a valid zero and no window active (readyAllowed()).
 * - After boot, once all inputs have published, the vehicle starts the automatic boot zero (default duration)
 *   if no valid zero exists yet (shouldStartAutoZero()).
 * - Arm (0xA5, removed in T9b-4) and MissionBegin (SA go) are refused while a window is active.
 * - SystemReset and the start of a mag calibration abort an active window (abort()). An active window has always
 *   begun applying zeroing (the estimators enter zeroing mode on the tick it starts), so the estimators hold a
 *   partial zero: the valid-zero record is CLEARED (review 2026-11-20), and Ready (hence a mission) needs a new
 *   complete window. The vehicle is not left waiting: in Startup shouldStartAutoZero() is true again, so the
 *   automatic 10 s boot-style zero runs (after SystemReset at once; after a mag calibration when it ends and the
 *   vehicle returns to Startup). Aborting with no window active changes nothing.
 */

#include <stdint.h>

namespace VCTR {
namespace Safety {

/// Vehicle state that decides whether a ZeroEstimators request is accepted.
struct ZeroRequestContext {
  bool initialised = false;   ///< all inputs (sensors, estimators) have published since the last (re)initialisation
  bool armed = false;         ///< applied arm state
  bool motorsEnabled = false; ///< motor outputs enabled (StarshipTVC motors)
  bool onGround = false;      ///< vehicle mode Startup or Ready (not Running, Failure, SensorCalib) and the active mission Idle or Finished
};

enum class ZeroRequestResult : uint8_t {
  Started = 0,     ///< accepted, a new window was started
  Restarted,       ///< accepted, the active window was restarted with the new duration
  BadParam,        ///< paramInt is not kParamInt
  NotInitialised,  ///< inputs have not all published yet
  Armed,           ///< vehicle armed
  MotorsEnabled,   ///< motor outputs enabled
  NotOnGround,     ///< vehicle mode does not guarantee the vehicle is on the ground
  HoldActive,      ///< an SE hold window is already running (holdStart only)
};

/// True for Started and Restarted.
inline bool accepted(ZeroRequestResult r) {
  return r == ZeroRequestResult::Started || r == ZeroRequestResult::Restarted;
}

/// Short text for logs.
const char *toString(ZeroRequestResult r);

class EstimatorZeroing {
public:
  static constexpr int16_t kParamInt = 0x2E;        ///< ZeroEstimators validation value
  static constexpr int64_t kSecondNs = 1000000000LL;
  static constexpr float kDefaultDurationS = 10.0f; ///< NaN or 0 in paramFloat, and the automatic boot zero
  static constexpr float kMinDurationS = 1.0f;
  static constexpr float kMaxDurationS = 60.0f;

  /// Window length for a requested duration: NaN or 0 -> 10 s, otherwise clamped to 1..60 s (+inf -> 60 s,
  /// negative -> 1 s).
  static int64_t durationNs(float requestedS);

  /// ZeroEstimators telecommand. Accepted only with paramInt == kParamInt, initialised, disarmed, motors off and
  /// on the ground; then starts (or restarts) the window [now, now + durationNs(paramFloat)).
  ZeroRequestResult request(int16_t paramInt, float paramFloat, const ZeroRequestContext &ctx, int64_t nowNs);

  /// True when the automatic boot zero should start now: inputs initialised, no valid zero since boot, no
  /// window active, and the vehicle is in Startup (the only mode in which the boot zero runs).
  bool shouldStartAutoZero(bool initialised, bool inStartupMode) const {
    return initialised && inStartupMode && !zeroed_ && !active_;
  }

  /// Starts the automatic boot zero (default duration). Caller checks shouldStartAutoZero() first.
  void startAuto(int64_t nowNs) { start(nowNs, durationNs(kDefaultDurationS)); }

  /// Advances the window. Returns true exactly once, on the tick the window completes (valid zero recorded).
  bool update(int64_t nowNs);

  // ---- SE hold zeroing (T9b-4, docs/t9b-design.md D11, appendix 6.4) ----
  // The Pocket's SE held for 1 s zeroes the estimators WHILE HELD. One extra "hold" window shares this object with the
  // fixed window above: active() is true while either runs, the end is the later of the two, and ending the hold
  // never ends a fixed window. The hold starts with holdStart() (same acceptance rules as request()), is kept alive
  // by holdRefresh() with the time of each fresh "held" sample (it ends kHoldGraceNs after the last one), and ends
  // on release (holdRelease), on staleness (update), at kHoldCapNs after its start, or aborted (holdAbort: the
  // interlock opened). It records a valid zero if the zeroing ran for at least kHoldMinNs, otherwise the estimators
  // hold a partial zero and the record is cleared like abort() does (the caller returns the vehicle to Startup).
  static constexpr int64_t kHoldGraceNs = 300000000LL;  ///< window kept after the last fresh "held" sample
  static constexpr int64_t kHoldCapNs = 60000000000LL;  ///< a stuck SE stops zeroing this long after the hold start
  static constexpr int64_t kHoldMinNs = 1000000000LL;   ///< zeroing shorter than this is not a valid zero

  /// Result of a hold ending (update, holdRelease, holdAbort).
  enum class HoldEnd : uint8_t { None = 0, Valid, Short, Aborted };

  /// Starts the hold window at `nowNs` under the request() rules (initialised, disarmed, motors off, on the
  /// ground; the paramInt check does not apply). Fails (and counts a refusal) if a hold is already active.
  ZeroRequestResult holdStart(const ZeroRequestContext &ctx, int64_t nowNs);
  /// A fresh "held" sample at `sampleNs` (no effect without an active hold).
  void holdRefresh(int64_t sampleNs);
  /// SE released at `nowNs`.
  HoldEnd holdRelease(int64_t nowNs);
  /// The hold is cancelled (interlock opened): clears the valid-zero record, like abort().
  HoldEnd holdAbort();
  bool holdActive() const { return holdActive_; }
  int64_t holdStartNs() const { return holdStartNs_; }
  /// Zeroing duration of the last hold that ended (ns), for the log.
  int64_t lastHoldRanNs() const { return lastHoldRanNs_; }
  /// HoldEnd that update() produced on its last call (None if no hold ended there); read once after update().
  HoldEnd takeHoldEnd() {
    const HoldEnd e = pendingHoldEnd_;
    pendingHoldEnd_ = HoldEnd::None;
    return e;
  }

  /// SystemReset / mag calibration start: aborts an active window without recording a zero and clears the
  /// valid-zero record (the estimators hold a partial zero). No window active: nothing changes.
  /// Returns true if a window was aborted.
  bool abort();

  /// A sensor calibration was applied (accel, mag or gyro): the estimators hold a zero taken with the old inputs.
  /// Clears the valid-zero record so Ready (and a mission) needs a new complete window; the automatic zero then runs
  /// again in Startup. A window that is active is NOT touched (callers abort() it first; calibration is only applied
  /// on the ground). Returns true if a valid zero was cleared.
  bool invalidate() {
    const bool was = zeroed_;
    zeroed_ = false;
    return was;
  }

  bool active() const { return active_ || holdActive_; }
  bool zeroedSinceBoot() const { return zeroed_; }
  int64_t endNs() const { return endNs_; }

  /// Arm (0xA5, the unsafe direction) is allowed only outside a window. Disarm is always allowed.
  bool armAllowed() const { return !active(); }
  /// MissionBegin is allowed only outside a window.
  bool missionBeginAllowed() const { return !active(); }
  /// Startup -> Ready requires a valid zero since boot and no active window.
  bool readyAllowed() const { return zeroed_ && !active(); }

  /// Number of refused ZeroEstimators requests since boot (any reason).
  uint32_t refusedCount() const { return refused_; }

private:
  void start(int64_t nowNs, int64_t lengthNs) {
    active_ = true;
    endNs_ = nowNs + lengthNs;
  }

  HoldEnd finishHold(int64_t ranNs, bool aborted);

  bool active_ = false;
  bool zeroed_ = false;
  int64_t endNs_ = 0;
  uint32_t refused_ = 0;
  bool holdActive_ = false;
  int64_t holdStartNs_ = 0, holdLastNs_ = 0, lastHoldRanNs_ = 0;
  HoldEnd pendingHoldEnd_ = HoldEnd::None;
};

/**
 * Log rate limiter for refused requests (logging only): due on the first refusal, then at most once per period.
 * The refusal count itself is kept by EstimatorZeroing.
 */
class LogRateLimit {
public:
  explicit LogRateLimit(int64_t periodNs) : periodNs_(periodNs) {}
  bool due(int64_t nowNs) {
    if (!logged_ || nowNs - lastNs_ >= periodNs_) {
      logged_ = true;
      lastNs_ = nowNs;
      return true;
    }
    return false;
  }

private:
  int64_t periodNs_;
  int64_t lastNs_ = 0;
  bool logged_ = false;
};

} // namespace Safety
} // namespace VCTR

#endif
