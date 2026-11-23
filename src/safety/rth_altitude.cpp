#include "safety/rth_altitude.hpp"

#include <math.h>

namespace VCTR {
namespace Safety {

float rthReturnAltitude(float altitudeM, bool valid, float capM) {
  if (!valid || !isfinite(altitudeM)) return capM;
  if (altitudeM > capM) return capM;
  if (altitudeM < 0.0f) return 0.0f; // on or below the reference: never a setpoint above the vehicle (immediate landing)
  return altitudeM;
}

} // namespace Safety
} // namespace VCTR
