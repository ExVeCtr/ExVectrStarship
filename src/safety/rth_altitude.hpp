#ifndef SAFETY_RTH_ALTITUDE_HPP
#define SAFETY_RTH_ALTITUDE_HPP

/**
 * RTH return altitude (owner decision 2026-11-23): RTH no longer climbs to its home point's z when triggered low.
 * The return altitude is the altitude at the trigger, capped at the home point's z (3 m) and floored at 0
 * (a vehicle on the ground gets a setpoint at its own height, never above it, so the landing detector lands it at once;
 * bug 2026-11-23: a 0.5 m floor left a grounded vehicle stuck in RTH). A high trigger (the
 * bellyflop, 84 m) gives exactly the cap, as before. Not valid or not finite altitude: the cap (the old behaviour).
 * Pure, host-tested (test/test_rth_altitude).
 */

namespace VCTR {
namespace Safety {

/// altitudeM: position estimate z (m above the position reference) at the RTH start; capM: the home point z.
float rthReturnAltitude(float altitudeM, bool valid, float capM);

} // namespace Safety
} // namespace VCTR

#endif
