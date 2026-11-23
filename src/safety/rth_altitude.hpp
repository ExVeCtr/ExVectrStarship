#ifndef SAFETY_RTH_ALTITUDE_HPP
#define SAFETY_RTH_ALTITUDE_HPP

/**
 * RTH return altitude (owner decision 2026-11-23): RTH no longer climbs to its home point's z when triggered low.
 * The return altitude is the altitude at the trigger, capped at the home point's z (3 m) and floored at
 * kRthMinAltitudeM (so the landing detector and the descent start from a defined altitude). A high trigger (the
 * bellyflop, 84 m) gives exactly the cap, as before. Not valid or not finite altitude: the cap (the old behaviour).
 * Pure, host-tested (test/test_rth_altitude).
 */

namespace VCTR {
namespace Safety {

constexpr float kRthMinAltitudeM = 0.5f;

/// altitudeM: position estimate z (m above the position reference) at the RTH start; capM: the home point z.
float rthReturnAltitude(float altitudeM, bool valid, float capM);

} // namespace Safety
} // namespace VCTR

#endif
