#include "safety/rth_altitude.hpp"

#include <math.h>

namespace VCTR {
namespace Safety {

float rthReturnAltitude(float altitudeM, bool valid, float capM) {
  if (!valid || !isfinite(altitudeM)) return capM;
  const float floorM = kRthMinAltitudeM < capM ? kRthMinAltitudeM : capM;
  if (altitudeM > capM) return capM;
  if (altitudeM < floorM) return floorM;
  return altitudeM;
}

} // namespace Safety
} // namespace VCTR
