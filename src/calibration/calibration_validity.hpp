#ifndef CALIBRATION_VALIDITY_HPP
#define CALIBRATION_VALIDITY_HPP

/**
 * "All three calibrations must be good to fly" (owner decision 2026-11-21, workspace docs/calibration-design.md
 * section 6.1 D7) and the surrounding policy: pure, host-tested logic with no Arduino, no ExVectrCore, no globals
 * (test/test_calibration_validity). The vehicle task (main.cpp) feeds it the stored calibration, the gyro run state and
 * the last still-window sample, and applies the verdict to Arm, MissionBegin and the countdown.
 *
 * Validity per sensor:
 * - gyro: valid only after a successful automatic bias run in the CURRENT power cycle (the stored key 100 value is only
 *   the boot offset, never proof). A later failed run makes it invalid again until a run succeeds.
 * - accel, mag: the legacy EEPROM key must have been read from memory (not created as a default), must not be the
 *   identity default, and its parameters must be in range. Then a LIVE check over the last still window (the gyro
 *   run's mean raw accel and mag): |a_cal| = 9.81 +- 0.25 m/s^2, |B_cal| = 0.05 +- 30 % (driver units).
 * The tolerances below are proposals from the design (not measured on this vehicle): tune them here after the first
 * calibration session.
 *
 * SIMULATION-MODE EXEMPTION (flight-safety item, documented as a sim/flight difference in overview.md): the check is
 * NOT required in simulation mode (kCheckExemptInSimulation). Everything else (the gyro run, the calibration itself)
 * behaves identically in both modes; only the refusal of Arm / MissionBegin and the countdown abort are skipped.
 */

#include <stddef.h>
#include <stdint.h>

namespace VCTR {
namespace Calibration {

// ---- Simulation exemption (the one place) ----
constexpr bool kCheckExemptInSimulation = true;
/// True if the all-calibrations-valid rule applies in the given mode.
inline bool checkRequired(bool simulationMode) { return !(kCheckExemptInSimulation && simulationMode); }

// ---- Tolerances (named constants, see the header comment) ----
constexpr double kGravity = 9.81;                 ///< m/s^2 (= AccelFitConfig::gRef)
constexpr double kAccelLiveTolerance = 0.25;      ///< |a_cal| = g +- this in a still window, m/s^2
constexpr double kMagRef = 0.05;                  ///< calibrated |B| in driver units (= MagFitConfig::rRef)
constexpr double kMagLiveToleranceRatio = 0.30;   ///< |B_cal| = kMagRef * (1 +- this) in a still window
constexpr double kAccelOffsetMax = 1.5;           ///< |b_k| m/s^2 (= AccelFitConfig::maxOffset)
constexpr double kAccelDiagMin = 0.95;            ///< sensor-frame A: diagonal in [min, max] (= scale limits)
constexpr double kAccelDiagMax = 1.05;
constexpr double kAccelOffDiagMax = 0.02;         ///< |A_jk|, j != k (= AccelFitConfig::maxOffDiag)
constexpr double kMagRadiusMin = 0.10;            ///< raw field radius R = kMagRef / s, driver units
constexpr double kMagRadiusMax = 1.00;
constexpr double kMagOffsetRatioMax = 0.8;        ///< |b| < ratio * R (= MagFitConfig::maxOffsetRatio)
constexpr double kMagMatrixDiagSpread = 0.20;     ///< C = s I: each diagonal within +- this of the mean
constexpr double kMagMatrixOffDiagRatio = 0.05;   ///< |C_jk| <= this * mean diagonal
constexpr double kDefaultEps = 1e-6;              ///< identity-default detection

// ---- Gyro automatic run planning ----
constexpr int64_t kSecondNs = 1000000000LL;
/// A zeroing window runs the gyro bias run only if it is at least this long: the 5 s still window plus the 1 s
/// reserve before the window end (deadline = end - 1 s, so the new bias is applied while the EKF still zeroes) plus
/// 1 s slack. (The first plan said 6 s; 7 s leaves the collector one second of slack against a late start.)
constexpr int64_t kGyroRunMinWindowNs = 7 * kSecondNs;
constexpr int64_t kGyroRunEndReserveNs = 1 * kSecondNs;

/// MissionBegin refuses a countdown whose waypoint Initialisation phase (lasting -paramInt - 10 s) cannot hold the
/// gyro run: 5 s window + 3 s slack = 8 s, so paramInt <= -18.
constexpr int16_t kMinCountdownParamInt = -18;
constexpr int64_t kCountdownInitialisationEndS = 10; ///< Initialisation lasts until mission time -10 s

/// Window end of the waypoint mission's Initialisation phase, from MissionBegin at nowNs with paramInt seconds.
inline int64_t countdownWindowEndNs(int64_t nowNs, int16_t paramInt) {
  return nowNs + (-static_cast<int64_t>(paramInt) - kCountdownInitialisationEndS) * kSecondNs;
}

/// Whether MissionBegin's paramInt is long enough while the calibration check is required.
inline bool countdownLongEnough(int16_t paramInt) { return paramInt <= kMinCountdownParamInt; }

struct GyroRunPlan {
  bool run = false;
  int64_t deadlineNs = 0;
};

/// Plans the automatic gyro run for a zeroing window [nowNs, windowEndNs): runs if the window is at least
/// kGyroRunMinWindowNs, with the deadline kGyroRunEndReserveNs before the window end.
inline GyroRunPlan planGyroRun(int64_t nowNs, int64_t windowEndNs) {
  GyroRunPlan p;
  if (windowEndNs - nowNs >= kGyroRunMinWindowNs) {
    p.run = true;
    p.deadlineNs = windowEndNs - kGyroRunEndReserveNs;
  }
  return p;
}

// ---- Calibration command preconditions (legacy telecommands, design section 3.3) ----
struct CommandContext {
  bool initialised = false;
  bool armed = false;
  bool motorsEnabled = false;
  bool modeAllowed = false;        ///< vehicle mode Startup, Ready or SensorCalib
  bool missionIdleOrFinished = false;
  bool simulation = false;
};

enum class CommandResult : uint8_t { Ok = 0, NotInitialised, Armed, MotorsEnabled, NotOnGround, Simulation };

CommandResult checkCommand(const CommandContext &ctx);
const char *toString(CommandResult r);

// ---- Validity ----

enum class Reason : uint8_t {
  Ok = 0,
  NotCalibrated, ///< gyro: no successful run in this power cycle
  RunFailed,     ///< gyro: the last run failed
  KeyMissing,    ///< accel/mag: the key was not found in memory (created as default, or memory cleared)
  Default,       ///< accel/mag: identity default (never calibrated)
  OutOfRange,    ///< accel/mag: parameters outside the plausible range
  LiveCheck,     ///< accel/mag: still-window check against gravity / the field radius failed
  NoLiveData,    ///< accel/mag: the gyro run supplied no accel / mag samples to check
};

const char *toString(Reason r);

enum class GyroRunState : uint8_t { NoRun = 0, Ok, Failed };

/// What is stored for one sensor, in the legacy form: cal = C * T * (raw - offset).
struct StoredCalibration {
  bool keyPresent = false;
  double offset[3] = {0, 0, 0};
  double C[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  double T[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
};

/// Mean raw accel and mag of the last successful still window (the gyro run).
struct LiveStill {
  bool haveAccel = false;
  bool haveMag = false;
  double accelRaw[3] = {0, 0, 0};
  double magRaw[3] = {0, 0, 0};
};

struct ValidityInput {
  GyroRunState gyro = GyroRunState::NoRun;
  StoredCalibration accel, mag;
  LiveStill live;
};

struct Validity {
  Reason gyro = Reason::NotCalibrated;
  Reason accel = Reason::KeyMissing;
  Reason mag = Reason::KeyMissing;
  bool allOk() const { return gyro == Reason::Ok && accel == Reason::Ok && mag == Reason::Ok; }
};

Reason checkAccelParams(const StoredCalibration &s);
Reason checkMagParams(const StoredCalibration &s);
Validity calibrationsValid(const ValidityInput &in);

/// What to do when the gyro run that closes the mission countdown has finished (owner decision D7): re-check ALL THREE
/// calibrations; if one is bad the mission is aborted before liftoff (reset and stop) with the reason in the log.
enum class CountdownAction : uint8_t {
  Continue = 0,   ///< check not required (simulation mode) or all three good
  Abort,          ///< a calibration is bad and the countdown is still running: reset and stop
  BadButNotInCountdown, ///< a calibration is bad but the mission is no longer in its countdown (nothing to abort)
};

inline CountdownAction decideCountdownEnd(bool checkRequired, const Validity &v, bool inCountdown) {
  if (!checkRequired || v.allOk())
    return CountdownAction::Continue;
  return inCountdown ? CountdownAction::Abort : CountdownAction::BadButNotInCountdown;
}

/// "gyro: ok, accel: offset out of range, mag: default" into buf (always terminated). Returns the length.
size_t describe(const Validity &v, char *buf, size_t n);

/// |M (raw - offset)| with M = C * T.
double calibratedNorm(const StoredCalibration &s, const double raw[3]);

} // namespace Calibration
} // namespace VCTR

#endif
