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
 * NO SIMULATION EXEMPTION (owner decision 2026-11-21, safety.md rule 9): nothing in this file takes the simulation
 * flag. In simulation the sensors and the stored calibration keys are real, so the same rules apply to Arm,
 * MissionBegin, the countdown re-check and the armed-in-Ready disarm. The decisions are the pure functions
 * decideArm, decideMissionBegin, mustDisarmInReady, decideCountdownEnd and gyroBiasMayApply below (host-tested in
 * test/test_calibration_validity); main.cpp only gathers the inputs and acts on the verdict.
 */

#include <stddef.h>
#include <stdint.h>

#include "ExVectrDSP/calibration/accel_ellipsoid_fit.hpp"
#include "ExVectrDSP/calibration/mag_ellipsoid_fit.hpp"

namespace VCTR {
namespace Calibration {

// ---- Tolerances (named constants, see the header comment) ----
constexpr double kGravity = 9.81;                 ///< m/s^2 (= AccelFitConfig::gRef)
constexpr double kAccelLiveTolerance = 0.25;      ///< |a_cal| = g +- this in a still window, m/s^2
constexpr double kMagRef = 0.05;                  ///< calibrated |B| in driver units (= MagFitConfig::rRef)
constexpr double kMagLiveToleranceRatio = 0.30;   ///< |B_cal| = kMagRef * (1 +- this) in a still window
constexpr double kAccelOffsetMax = DSP::Calib::kAccelOffsetLimit; ///< |b_k| m/s^2 (= AccelFitConfig::maxOffset, 2.5)
constexpr double kAccelDiagMin = 0.95;            ///< sensor-frame A: diagonal in [min, max] (= scale limits)
constexpr double kAccelDiagMax = 1.05;
constexpr double kAccelOffDiagMax = 0.02;         ///< |A_jk|, j != k (= AccelFitConfig::maxOffDiag)
constexpr double kMagRadiusMin = 0.10;            ///< raw field radius R = kMagRef / s, driver units
constexpr double kMagRadiusMax = 1.00;
constexpr double kMagOffsetRatioMax = 0.8;        ///< |b| < ratio * R (= MagFitConfig::maxOffsetRatio)
constexpr double kMagAxisRatioMax = DSP::Calib::kMagAxisRatioMax; ///< sensor-frame A = T^-1 C T: max/min eigenvalue
constexpr double kMagEigMinRel = DSP::Calib::kMagEigMinRel;       ///< every eigenvalue of A within [min, max] x mean
constexpr double kMagEigMaxRel = DSP::Calib::kMagEigMaxRel;
constexpr double kMagSymmetryTol = 0.02;          ///< |A_jk - A_kj| <= this * mean eigenvalue (A must be symmetric)
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
};

enum class CommandResult : uint8_t { Ok = 0, NotInitialised, Armed, MotorsEnabled, NotOnGround };

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
  Continue = 0,   ///< all three good
  Abort,          ///< a calibration is bad and the countdown is still running: reset and stop
  BadButNotInCountdown, ///< a calibration is bad but the mission is no longer in its countdown (nothing to abort)
};

inline CountdownAction decideCountdownEnd(const Validity &v, bool inCountdown) {
  if (v.allOk())
    return CountdownAction::Continue;
  return inCountdown ? CountdownAction::Abort : CountdownAction::BadButNotInCountdown;
}

// ---- Gates (identical in simulation and real mode: no simulation input exists) ----
enum class GateRefusal : uint8_t { None = 0, CalibrationInvalid, CountdownTooShort };

/// Arm (valid magic): all three calibrations must be good.
inline GateRefusal decideArm(const Validity &v) {
  return v.allOk() ? GateRefusal::None : GateRefusal::CalibrationInvalid;
}

/// MissionBegin from Ready: all three calibrations good, and a countdown (paramInt seconds, negative) long enough for
/// the gyro run in the Initialisation phase (paramInt <= kMinCountdownParamInt). Validity is reported first.
inline GateRefusal decideMissionBegin(const Validity &v, int16_t paramInt) {
  if (!v.allOk())
    return GateRefusal::CalibrationInvalid;
  if (!countdownLongEnough(paramInt))
    return GateRefusal::CountdownTooShort;
  return GateRefusal::None;
}

/// Invariant "armed implies calibrations good", enforced on the ground only (mode Ready): true = disarm now.
inline bool mustDisarmInReady(bool armed, bool modeReady, const Validity &v) {
  return armed && modeReady && !v.allOk();
}

/// A passing automatic gyro run may change the applied bias only on the ground: in Startup, Ready or SensorCalib, or
/// when it is the run of the countdown and the countdown is still running. Anything else is discarded (and logged).
inline bool gyroBiasMayApply(bool groundMode, bool countdownRun, bool inCountdown) {
  return groundMode || (countdownRun && inCountdown);
}

/// "gyro: ok, accel: offset out of range, mag: default" into buf (always terminated). Returns the length.
size_t describe(const Validity &v, char *buf, size_t n);

/// |M (raw - offset)| with M = C * T.
double calibratedNorm(const StoredCalibration &s, const double raw[3]);

} // namespace Calibration
} // namespace VCTR

#endif
