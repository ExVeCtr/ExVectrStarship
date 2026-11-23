#include "stop_altitude.hpp"

namespace VCTR {
namespace Safety {

StopAltResult decideSetStopAltitude(float valueM, const StopAltContext &c) {
  if (c.interlockOpen) return StopAltResult::InterlockOpen;
  if (c.motorsEnabled) return StopAltResult::MotorsEnabled;
  if (c.committed) return StopAltResult::Committed;
  if (!isfinite(valueM)) return StopAltResult::NotFinite;
  if (valueM < kStopAltitudeMinM || valueM > kStopAltitudeMaxM) return StopAltResult::OutOfRange;
  return StopAltResult::Accepted;
}

const char *toString(StopAltResult r) {
  switch (r) {
  case StopAltResult::Accepted: return "accepted";
  case StopAltResult::NotFinite: return "value not finite";
  case StopAltResult::OutOfRange: return "value outside 5..60 m";
  case StopAltResult::InterlockOpen: return "motor interlock open";
  case StopAltResult::MotorsEnabled: return "motors enabled";
  case StopAltResult::Committed: return "mission committed (in flight or pending)";
  }
  return "?";
}

} // namespace Safety
} // namespace VCTR
