#ifndef SAFETY_STOP_ALTITUDE_HPP
#define SAFETY_STOP_ALTITUDE_HPP

/**
 * Stop altitude parameter of the freefall mission (the height above the position reference where the vehicle ends up
 * stopped; the stop trigger is Pz < stopDist(Vz) + stopAlt). The ONE runtime value lives in MissionFreefall
 * (stopAlt_, set by setStopTargetAltitude); this header holds the default, the minimum and the pure decisions.
 * Pure, no heap, host-tested (test/test_stop_altitude).
 */

#include <math.h>
#include <stdint.h>

namespace VCTR {
namespace Safety {

constexpr float kStopAltitudeDefaultM = 20.0f; ///< boot default (not persisted)
constexpr float kStopAltitudeMinM = 1.0f; ///< no maximum (owner decision 2026-11-23); must be finite

enum class StopAltResult : uint8_t { Accepted = 0, NotFinite, OutOfRange, InterlockOpen, MotorsEnabled, Committed };

struct StopAltContext {
  bool interlockOpen = false;
  bool motorsEnabled = false;
  bool committed = false; ///< Safety::committed(): chain pending or mode Running not on the ground
};

/// SetStopAltitude (telecommand 19): accepted only on the ground (interlock closed, motors not enabled, not committed),
/// finite, at least kStopAltitudeMinM (no maximum). First failing rule wins.
StopAltResult decideSetStopAltitude(float valueM, const StopAltContext &c);
const char *toString(StopAltResult r);

/// MISSION / AIR_MISSION flip_start_altitude: the stop trigger's own threshold (getStopAltitude() = stopDist(Vz) +
/// stopAlt) while the freefall phase is active, NaN otherwise.
inline float flipStartAltitude(bool freefallActive, float triggerAltitudeM) {
  return freefallActive ? triggerAltitudeM : NAN;
}

} // namespace Safety
} // namespace VCTR

#endif
