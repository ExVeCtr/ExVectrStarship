#ifndef SAFETY_BELLYFLOP_KICK_HPP
#define SAFETY_BELLYFLOP_KICK_HPP

/**
 * LEGACY re-export (until E6 of the flight executive, docs/project/flight-executive.md). The powered bellyflop kick
 * of the old mission code (src/mission/mission_bellyflop.hpp, src/safety/control_phase.*) uses these names.
 *
 * The pure kick functions moved unchanged into the ExVectrControl executive core (ExVectrControl/executive/kick.hpp,
 * namespace VCTR::Exec); the Starship constants moved to src/executive/starship_constants.hpp. This header keeps
 * the old names in VCTR::Safety so the old code compiles and behaves unchanged. See docs/project/missions.md
 * section 3 for the legacy behaviour.
 */

#include <stdint.h>

#include "ExVectrControl/executive/kick.hpp"
#include "executive/starship_constants.hpp"

namespace VCTR {
namespace Safety {

constexpr float kKickPi = Starship::kPi;
constexpr float kKickDeg = Starship::kDeg;

using Starship::kKickEndPitchRateRadPs;
using Starship::kKickEndRemainingAngleRad;
using Starship::kKickPitchAngleRad;
using Starship::kKickThrustLimitN;
using Starship::kKickTimeLimitNs;

/// Defence in depth in the 10 Hz safety task (ControlPhaseTracker): if the bellyflop mission is still active this
/// long after the time limit (one safety tick), the TVC set is switched off anyway. Legacy only.
constexpr int64_t kKickGuardMarginNs = 100000000LL;

using Exec::KickEnd;
using Exec::kickRemainingAngle;
using Exec::kickRotate;
using Exec::kickTargetAttitude;

/**
 * End trigger of the kick, checked every mission tick: elapsed > kKickTimeLimitNs (time), else remaining angle <
 * kKickEndRemainingAngleRad (angle), else |pitch rate| > kKickEndPitchRateRadPs (rate). Non-finite angle or rate
 * never trigger (the time limit still does).
 */
inline KickEnd kickEndTrigger(int64_t elapsedNs, float remainingAngleRad, float pitchRateRadPs) {
  const Exec::KickEndLimits limits = {kKickTimeLimitNs, kKickEndRemainingAngleRad, kKickEndPitchRateRadPs};
  return Exec::kickEndTrigger(elapsedNs, remainingAngleRad, pitchRateRadPs, limits);
}

} // namespace Safety
} // namespace VCTR

#endif
