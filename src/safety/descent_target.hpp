#ifndef SAFETY_DESCENT_TARGET_HPP
#define SAFETY_DESCENT_TARGET_HPP

/**
 * Observer side of the descent control (step S3 of docs/descent-control-design.md): which target the flap task's
 * DescentGuidance gets, and whether the position may be used. Pure, header-only, host-tested in
 * test/test_descent_wiring. The guidance itself runs in CTRL::ControlAttitudeFlaps (50 Hz).
 *
 * Policy (owner decisions 2026-11-22): the target is the home XY (the position reference, the world origin 0,0 in
 * the NWU frame) and the end altitude is the freefall stop altitude. If the position is not trustworthy (GNSS lost or
 * invalid, no fix, non-finite) the target is invalid: the guidance holds the heading and the ship falls straight down
 * (zero glide) to the stop-altitude guard, then flips. The no-position return-to-home during the ascent is T9b's, not
 * this file's.
 */

#include <math.h>
#include <stdint.h>

#include "ExVectrControl/starship/descent_guidance.hpp"

namespace VCTR {
namespace Safety {

/// Fallback end altitude (flip start estimate, m) if the stop altitude is not a finite number.
constexpr float kDescentDefaultEndAltitudeM = 27.6f;

struct DescentTargetInputs {
  bool descentActive = false;   ///< flap mode is Descent this tick (phase table bellyflop set, new law selected)
  bool positionUsable = false;  ///< see descentPositionUsable()
  float stopAltitude_m = NAN;   ///< MissionFreefall::getStopAltitude()
};

/// Home XY = the origin (0, 0) of the world frame (the position reference is set at reset / zeroing).
inline CTRL::descent::DescentTarget makeDescentTarget(const DescentTargetInputs &in) {
  CTRL::descent::DescentTarget t;
  t.x_m = 0.0f;
  t.y_m = 0.0f;
  t.endAltitude_m = isfinite(in.stopAltitude_m) ? in.stopAltitude_m : kDescentDefaultEndAltitudeM;
  t.valid = in.descentActive && in.positionUsable;
  return t;
}

/**
 * The estimated position may steer the descent only if it is based on a valid GNSS fix. In simulation mode the
 * estimators are replaced by the body simulator, so the (absent) GNSS is not required.
 * gnssAgeNs = now - GNSS timestamp (a GNSS that has not delivered a recent sample is no fix).
 */
inline bool descentPositionUsable(bool simulationMode, bool gnssPositionValid, int64_t gnssAgeNs, int64_t timeoutNs,
                                  bool posFinite) {
  if (!posFinite) return false;
  if (simulationMode) return true;
  return gnssPositionValid && gnssAgeNs >= 0 && gnssAgeNs <= timeoutNs;
}

/// Telemetry attitude setpoint source for the flap controller: the heading setpoint and pitch of the nose direction.
/// Descent: heading setpoint of the guidance with pitch 0 (the split is not representable as an attitude; the GS
/// sees it in the actual flap angles). Stabilize: the setting's azimuth and pitch.
struct FlapSetpointAngles {
  bool flapsActive = false;
  float azimuth = 0, pitch = 0;
};
inline FlapSetpointAngles flapSetpointAngles(bool phaseFlaps, bool modeStabilize, bool modeDescent, float stabAzimuth,
                                             float stabPitch, float descentHeading) {
  FlapSetpointAngles r;
  if (!phaseFlaps) return r;
  if (modeDescent) {
    r.flapsActive = true;
    r.azimuth = descentHeading;
    r.pitch = 0.0f;
  } else if (modeStabilize) {
    r.flapsActive = true;
    r.azimuth = stabAzimuth;
    r.pitch = stabPitch;
  }
  return r;
}

}  // namespace Safety
}  // namespace VCTR

#endif
